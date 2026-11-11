"""
frames.py: Coordinate transformations and frame conversion between Fusion 360,
WPILib NWU (Schema v1 standard), and glTF/Engine (OpenGL) frames.

Team standard convention in Fusion 360:
  Forward:  Fusion +Z
  Left:     Fusion +X
  Up:       Fusion +Y
WPILib NWU standard (manifest.json):
  +X: Forward
  +Y: Left
  +Z: Up
Engine / glTF 2.0 convention:
  +X: Left
  +Y: Up
  +Z: Forward
"""

from typing import Optional, Sequence, Tuple
from .linalg import (
    Mat3,
    Vec3,
    mat3_det,
    mat3_from_rows,
    mat3_mul,
    mat3_mul_vec,
    mat3_scale,
    mat3_transpose,
    vec3_cross,
    vec3_dot,
    vec3_make,
    vec3_scale,
    vec3_sub,
)

# Scale factor from Fusion centimeters to SI meters
CM_TO_M = 0.01
M_TO_CM = 100.0

# Inertia scale factor: (kg * cm^2) -> (kg * m^2) = 1e-4
INERTIA_CM2_TO_M2 = 1e-4
INERTIA_M2_TO_CM2 = 1e4

AXIS_VECTORS = {
    "+X": (1.0, 0.0, 0.0),
    "-X": (-1.0, 0.0, 0.0),
    "+Y": (0.0, 1.0, 0.0),
    "-Y": (0.0, -1.0, 0.0),
    "+Z": (0.0, 0.0, 1.0),
    "-Z": (0.0, 0.0, -1.0),
}


class FrameConverter:
    """
    Handles frame and unit conversions between Fusion 360 and NWU.
    Configured with fusionUp and fusionForward strings (e.g. '+Y', '+Z').
    """

    def __init__(self, fusion_up: str = "+Y", fusion_forward: str = "+Z"):
        self.fusion_up_str = fusion_up.upper().strip()
        self.fusion_forward_str = fusion_forward.upper().strip()

        if self.fusion_up_str not in AXIS_VECTORS:
            raise ValueError(f"Invalid fusionUp '{fusion_up}'. Expected one of {list(AXIS_VECTORS.keys())}")
        if self.fusion_forward_str not in AXIS_VECTORS:
            raise ValueError(f"Invalid fusionForward '{fusion_forward}'. Expected one of {list(AXIS_VECTORS.keys())}")

        u = AXIS_VECTORS[self.fusion_up_str]
        f = AXIS_VECTORS[self.fusion_forward_str]

        if abs(vec3_dot(u, f)) > 1e-6:
            raise ValueError(f"fusionUp ({fusion_up}) and fusionForward ({fusion_forward}) must be orthogonal!")

        # In NWU: Forward x Left = Up => Left = Up x Forward
        l = vec3_cross(u, f)

        # R_F_to_N maps a Fusion vector to NWU:
        # [x_N]   [ f_x  f_y  f_z ] [x_F]
        # [y_N] = [ l_x  l_y  l_z ] [y_F]
        # [z_N]   [ u_x  u_y  u_z ] [z_F]
        self.R_F_to_N: Mat3 = mat3_from_rows(f, l, u)
        det_R = mat3_det(self.R_F_to_N)
        if abs(det_R - 1.0) > 1e-6:
            raise ValueError(f"Configured axes produce non-proper rotation (det = {det_R:.4f}). Must be +1.0.")

        self.R_N_to_F: Mat3 = mat3_transpose(self.R_F_to_N)

    def fusion_to_nwu_point(
        self,
        point_fusion_cm: Sequence[float],
        origin_fusion_m: Optional[Sequence[float]] = None,
    ) -> Vec3:
        """
        Converts a position from Fusion space (cm) to WPILib NWU space (m).
        p_N = R_F_to_N * (0.01 * p_F - o_F)
        where o_F is in meters in Fusion coordinates.
        """
        px_m = float(point_fusion_cm[0]) * CM_TO_M
        py_m = float(point_fusion_cm[1]) * CM_TO_M
        pz_m = float(point_fusion_cm[2]) * CM_TO_M

        if origin_fusion_m is not None:
            px_m -= float(origin_fusion_m[0])
            py_m -= float(origin_fusion_m[1])
            pz_m -= float(origin_fusion_m[2])

        return mat3_mul_vec(self.R_F_to_N, (px_m, py_m, pz_m))

    def nwu_to_fusion_point(
        self,
        point_nwu_m: Sequence[float],
        origin_fusion_m: Optional[Sequence[float]] = None,
    ) -> Vec3:
        """
        Inverse conversion: NWU (m) to Fusion (cm).
        """
        pF_m = mat3_mul_vec(self.R_N_to_F, point_nwu_m)
        if origin_fusion_m is not None:
            pF_m = (
                pF_m[0] + float(origin_fusion_m[0]),
                pF_m[1] + float(origin_fusion_m[1]),
                pF_m[2] + float(origin_fusion_m[2]),
            )
        return vec3_scale(pF_m, M_TO_CM)

    def fusion_to_nwu_vector(self, vec_fusion: Sequence[float]) -> Vec3:
        """
        Converts a direction vector (joint axis, normal) from Fusion to NWU.
        Does not apply origin translation or unit scaling.
        """
        return mat3_mul_vec(self.R_F_to_N, vec_fusion)

    def nwu_to_fusion_vector(self, vec_nwu: Sequence[float]) -> Vec3:
        """
        Converts a direction vector from NWU to Fusion.
        """
        return mat3_mul_vec(self.R_N_to_F, vec_nwu)

    def fusion_to_nwu_inertia(self, I_fusion_kgcm2: Mat3) -> Mat3:
        """
        Converts a 3x3 inertia tensor from Fusion coordinates (kg*cm^2) to NWU (kg*m^2):
        I_N = 10^-4 * R_F_to_N * I_F * (R_F_to_N)^T
        """
        scaled_I = mat3_scale(I_fusion_kgcm2, INERTIA_CM2_TO_M2)
        r_i = mat3_mul(self.R_F_to_N, scaled_I)
        return mat3_mul(r_i, self.R_N_to_F)

    def nwu_to_fusion_inertia(self, I_nwu_kgm2: Mat3) -> Mat3:
        """
        Converts a 3x3 inertia tensor from NWU (kg*m^2) to Fusion coordinates (kg*cm^2):
        I_F = 10^4 * R_N_to_F * I_N * (R_N_to_F)^T
        """
        scaled_I = mat3_scale(I_nwu_kgm2, INERTIA_M2_TO_CM2)
        r_i = mat3_mul(self.R_N_to_F, scaled_I)
        return mat3_mul(r_i, self.R_F_to_N)


# ---------------------------------------------------------------------------
# glTF / Engine Frame Conversion
# ---------------------------------------------------------------------------

def nwu_to_engine(vec_nwu: Sequence[float]) -> Vec3:
    """
    Converts vector from WPILib NWU (+X Fwd, +Y Left, +Z Up)
    to Engine / glTF (+X Left, +Y Up, +Z Fwd).
    Matches ConvertNWUVectorToEngine: (y_N, z_N, x_N)
    """
    return (float(vec_nwu[1]), float(vec_nwu[2]), float(vec_nwu[0]))


def engine_to_nwu(vec_engine: Sequence[float]) -> Vec3:
    """
    Inverse conversion from Engine / glTF to WPILib NWU.
    (z_E, x_E, y_E)
    """
    return (float(vec_engine[2]), float(vec_engine[0]), float(vec_engine[1]))
