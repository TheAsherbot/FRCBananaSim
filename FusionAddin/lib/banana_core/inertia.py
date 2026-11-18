"""
inertia.py: Rigid body mass and inertia tensor calculations.
Supports parallel axis theorem, coordinate frame rotations, Sylvester positive-definiteness,
and principal moment triangle inequality validation.
"""

from dataclasses import dataclass
import math
from typing import Any, Dict, List, Sequence, Tuple
from .linalg import (
    Mat3,
    Vec3,
    mat3_det,
    mat3_from_rows,
    mat3_mul,
    mat3_transpose,
    symmetric_eigenvalues,
    vec3_dot,
)


@dataclass
class InertiaTensor:
    """
    3x3 symmetric inertia tensor represented in tensor-element convention
    (off-diagonals are placed directly into the matrix, I_xy = -int(x*y*dm)).
    Units: kg * m^2 (or kg * cm^2 before frame conversion).
    """
    ixx: float = 0.0
    iyy: float = 0.0
    izz: float = 0.0
    ixy: float = 0.0
    ixz: float = 0.0
    iyz: float = 0.0

    def to_matrix(self) -> Mat3:
        """Returns the 3x3 symmetric matrix representation."""
        return (
            (float(self.ixx), float(self.ixy), float(self.ixz)),
            (float(self.ixy), float(self.iyy), float(self.iyz)),
            (float(self.ixz), float(self.iyz), float(self.izz)),
        )

    @classmethod
    def from_matrix(cls, m: Mat3) -> "InertiaTensor":
        """Builds from a 3x3 symmetric matrix, averaging symmetric off-diagonals."""
        ixx = float(m[0][0])
        iyy = float(m[1][1])
        izz = float(m[2][2])
        ixy = float((m[0][1] + m[1][0]) * 0.5)
        ixz = float((m[0][2] + m[2][0]) * 0.5)
        iyz = float((m[1][2] + m[2][1]) * 0.5)
        return cls(ixx=ixx, iyy=iyy, izz=izz, ixy=ixy, ixz=ixz, iyz=iyz)

    def to_dict(self, digits: int = 9) -> Dict[str, float]:
        """Returns dictionary formatted for Schema v1 manifest.json."""
        def _round(val: float) -> float:
            r = round(float(val), digits)
            return 0.0 if abs(r) == 0.0 else r

        return {
            "ixx": _round(self.ixx),
            "iyy": _round(self.iyy),
            "izz": _round(self.izz),
            "ixy": _round(self.ixy),
            "ixz": _round(self.ixz),
            "iyz": _round(self.iyz),
        }

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> "InertiaTensor":
        return cls(
            ixx=float(d.get("ixx", 0.0)),
            iyy=float(d.get("iyy", 0.0)),
            izz=float(d.get("izz", 0.0)),
            ixy=float(d.get("ixy", 0.0)),
            ixz=float(d.get("ixz", 0.0)),
            iyz=float(d.get("iyz", 0.0)),
        )

    def principal_moments(self) -> Tuple[float, float, float]:
        """Returns (I1, I2, I3) sorted descending (I1 >= I2 >= I3)."""
        return symmetric_eigenvalues(self.to_matrix())

    def is_positive_definite(self, eps_rel: float = 1e-12) -> bool:
        """
        Scale-invariant Sylvester's criterion (physical positive-definiteness).
        The tensor is normalized by its largest diagonal term first, so small but
        perfectly physical links (e.g. a 0.2 kg bracket, I ~ 1e-4 kg*m^2) pass.
        """
        scale = max(self.ixx, self.iyy, self.izz)
        if scale <= 0.0:
            return False
        inv = 1.0 / scale
        n = InertiaTensor(
            self.ixx * inv, self.iyy * inv, self.izz * inv,
            self.ixy * inv, self.ixz * inv, self.iyz * inv,
        )
        if n.ixx <= eps_rel:
            return False
        if n.ixx * n.iyy - n.ixy * n.ixy <= eps_rel:
            return False
        return mat3_det(n.to_matrix()) > eps_rel

    def passes_loader_check(self) -> bool:
        """
        Exact mirror of the C++ `InertiaTensor::IsPositiveDefinite()` used by
        `RobotSchemaLoader::Validate()`. It uses ABSOLUTE 1e-9 thresholds on all three
        leading minors, so physically valid but small tensors (roughly any diagonal
        term below ~1e-3 kg*m^2) are rejected by the loader.
        """
        if self.ixx <= 1e-9:
            return False
        if self.ixx * self.iyy - self.ixy * self.ixy <= 1e-9:
            return False
        return mat3_det(self.to_matrix()) > 1e-9

    def check_triangle_inequality(self, tol_rel: float = 1e-6) -> Tuple[bool, Tuple[float, float, float]]:
        """
        Checks principal triangle inequalities (tolerance is relative to the largest moment):
          I1 + I2 >= I3
          I1 + I3 >= I2
          I2 + I3 >= I1
        Returns (is_valid, (I1, I2, I3)).
        """
        i1, i2, i3 = self.principal_moments()
        tol = tol_rel * max(abs(i1), abs(i2), abs(i3), 1e-30)
        valid = (
            (i1 + i2 >= i3 - tol)
            and (i1 + i3 >= i2 - tol)
            and (i2 + i3 >= i1 - tol)
        )
        return valid, (i1, i2, i3)


# ---------------------------------------------------------------------------
# Parallel Axis Theorem
# ---------------------------------------------------------------------------

def parallel_axis_matrix(mass: float, offset: Sequence[float]) -> Mat3:
    """
    Computes the parallel axis matrix:
      M = mass * (||c||^2 * I_3 - c * c^T)
    """
    m = float(mass)
    cx, cy, cz = float(offset[0]), float(offset[1]), float(offset[2])
    c_sq = cx * cx + cy * cy + cz * cz

    return (
        (m * (c_sq - cx * cx), -m * (cx * cy), -m * (cx * cz)),
        (-m * (cx * cy), m * (c_sq - cy * cy), -m * (cy * cz)),
        (-m * (cx * cz), -m * (cy * cz), m * (c_sq - cz * cz)),
    )


def origin_to_com_tensor(
    I_origin: InertiaTensor,
    mass: float,
    com: Sequence[float],
) -> InertiaTensor:
    """
    Transfers an inertia tensor from the origin to the Center of Mass:
      I_com = I_origin - mass * (||c||^2 * I_3 - c * c^T)
    """
    m_mat = parallel_axis_matrix(mass, com)
    o_mat = I_origin.to_matrix()

    c_mat = (
        (o_mat[0][0] - m_mat[0][0], o_mat[0][1] - m_mat[0][1], o_mat[0][2] - m_mat[0][2]),
        (o_mat[1][0] - m_mat[1][0], o_mat[1][1] - m_mat[1][1], o_mat[1][2] - m_mat[1][2]),
        (o_mat[2][0] - m_mat[2][0], o_mat[2][1] - m_mat[2][1], o_mat[2][2] - m_mat[2][2]),
    )
    return InertiaTensor.from_matrix(c_mat)


def com_to_offset_tensor(
    I_com: InertiaTensor,
    mass: float,
    offset: Sequence[float],
) -> InertiaTensor:
    """
    Transfers an inertia tensor from the Center of Mass to an offset point:
      I_offset = I_com + mass * (||d||^2 * I_3 - d * d^T)
    """
    m_mat = parallel_axis_matrix(mass, offset)
    c_mat = I_com.to_matrix()

    off_mat = (
        (c_mat[0][0] + m_mat[0][0], c_mat[0][1] + m_mat[0][1], c_mat[0][2] + m_mat[0][2]),
        (c_mat[1][0] + m_mat[1][0], c_mat[1][1] + m_mat[1][1], c_mat[1][2] + m_mat[1][2]),
        (c_mat[2][0] + m_mat[2][0], c_mat[2][1] + m_mat[2][1], c_mat[2][2] + m_mat[2][2]),
    )
    return InertiaTensor.from_matrix(off_mat)


@dataclass
class BodyMassProps:
    """Single body mass properties (in link or world coordinates)."""
    mass_kg: float
    com: Vec3
    inertia_at_com: InertiaTensor


def aggregate_link_mass_props(
    bodies: Sequence[BodyMassProps],
) -> Tuple[float, Vec3, InertiaTensor]:
    """
    Aggregates N bodies into a single composite link using the parallel axis theorem:
      m_L = sum(m_b)
      c_L = sum(m_b * c_b) / m_L
      I_L = sum( I_com_b + m_b * (||d_b||^2 * I_3 - d_b * d_b^T) )  where d_b = c_b - c_L

    Returns (total_mass, composite_com, composite_inertia_at_com).
    """
    if not bodies:
        return 0.0, (0.0, 0.0, 0.0), InertiaTensor()

    total_mass = sum(b.mass_kg for b in bodies)
    if total_mass <= 0.0:
        return 0.0, (0.0, 0.0, 0.0), InertiaTensor()

    cx = sum(b.mass_kg * b.com[0] for b in bodies) / total_mass
    cy = sum(b.mass_kg * b.com[1] for b in bodies) / total_mass
    cz = sum(b.mass_kg * b.com[2] for b in bodies) / total_mass
    composite_com: Vec3 = (cx, cy, cz)

    # Accumulate composite inertia about composite_com
    total_ixx = 0.0
    total_iyy = 0.0
    total_izz = 0.0
    total_ixy = 0.0
    total_ixz = 0.0
    total_iyz = 0.0

    for b in bodies:
        dx = b.com[0] - composite_com[0]
        dy = b.com[1] - composite_com[1]
        dz = b.com[2] - composite_com[2]
        shifted = com_to_offset_tensor(b.inertia_at_com, b.mass_kg, (dx, dy, dz))
        total_ixx += shifted.ixx
        total_iyy += shifted.iyy
        total_izz += shifted.izz
        total_ixy += shifted.ixy
        total_ixz += shifted.ixz
        total_iyz += shifted.iyz

    composite_inertia = InertiaTensor(
        ixx=total_ixx,
        iyy=total_iyy,
        izz=total_izz,
        ixy=total_ixy,
        ixz=total_ixz,
        iyz=total_iyz,
    )

    return total_mass, composite_com, composite_inertia


# ---------------------------------------------------------------------------
# Fusion product-of-inertia sign handling
# ---------------------------------------------------------------------------

# Fallback used only when auto-detection has no informative body to vote on.
# +1: Fusion's returned products already are tensor elements (I_xy = -integral(xy dm)).
# -1: Fusion returns the positive integral, so products must be negated once at read time.
FUSION_PRODUCT_SIGN = 1


def flip_products(t: Tuple[float, float, float, float, float, float]) -> Tuple[float, float, float, float, float, float]:
    """Negates the three products of an (ixx, iyy, izz, ixy, ixz, iyz) tuple."""
    return (t[0], t[1], t[2], -t[3], -t[4], -t[5])


def detect_product_sign(
    samples: Sequence[Tuple[float, Sequence[float], Sequence[float], Sequence[float]]],
    informative_rel: float = 1e-3,
) -> Tuple[int, bool]:
    """
    Decides FUSION_PRODUCT_SIGN by comparing, per body, the principal moments Fusion reports
    (about the CoM) against the principal moments obtained from the origin-referenced tensor
    under both sign hypotheses.

    samples: (mass, com, (ixx, iyy, izz, ixy, ixz, iyz) about origin, (p1, p2, p3) about CoM),
             all in the same units (kg, cm, kg*cm^2).
    Returns (sign, confident). `confident` is False when no body distinguishes the hypotheses.
    """
    err_pos = 0.0
    err_neg = 0.0
    informative = 0

    for mass, com, io, principal in samples:
        if mass <= 0.0 or principal is None:
            continue
        p_sorted = sorted((float(principal[0]), float(principal[1]), float(principal[2])), reverse=True)
        p_sum = sum(abs(p) for p in p_sorted)
        if p_sum <= 0.0:
            continue

        errs = []
        eigs_by_sign = []
        for s in (1.0, -1.0):
            tensor_o = InertiaTensor(io[0], io[1], io[2], s * io[3], s * io[4], s * io[5])
            tensor_c = origin_to_com_tensor(tensor_o, mass, com)
            eigs = tensor_c.principal_moments()
            eigs_by_sign.append(eigs)
            errs.append(sum(abs(e - p) for e, p in zip(eigs, p_sorted)) / p_sum)

        separation = sum(abs(a - b) for a, b in zip(eigs_by_sign[0], eigs_by_sign[1])) / p_sum
        if separation < informative_rel:
            continue

        informative += 1
        err_pos += errs[0]
        err_neg += errs[1]

    if informative == 0:
        return FUSION_PRODUCT_SIGN, False
    return (1 if err_pos <= err_neg else -1), True


def compute_reference_properties(
    total_mass: float,
    total_com: Sequence[float],
    total_inertia_origin: InertiaTensor,
    excluded: Sequence[Tuple[float, Sequence[float], InertiaTensor]],
) -> Tuple[float, Vec3, InertiaTensor]:
    """
    Builds the Gate-4 reference (mass, CoM, inertia about CoM) from Fusion's whole-design
    properties, after subtracting bodies the exporter intentionally leaves out (visible
    COL_* / ZONE_* proxies, which carry real mass in Fusion but are not robot mass).

    Inputs are in one consistent frame/units (Fusion: kg, cm, kg*cm^2); the whole-design
    inertia is about the WORLD ORIGIN. `excluded` entries are (mass, com, inertia about origin).
    Returns (mass, com, inertia_about_com).
    """
    mass = float(total_mass)
    mx = mass * float(total_com[0])
    my = mass * float(total_com[1])
    mz = mass * float(total_com[2])
    m_o = total_inertia_origin.to_matrix()
    ixx, iyy, izz = m_o[0][0], m_o[1][1], m_o[2][2]
    ixy, ixz, iyz = m_o[0][1], m_o[0][2], m_o[1][2]

    for b_mass, b_com, b_inertia in excluded:
        mass -= b_mass
        mx -= b_mass * b_com[0]
        my -= b_mass * b_com[1]
        mz -= b_mass * b_com[2]
        ixx -= b_inertia.ixx
        iyy -= b_inertia.iyy
        izz -= b_inertia.izz
        ixy -= b_inertia.ixy
        ixz -= b_inertia.ixz
        iyz -= b_inertia.iyz

    if mass <= 0.0:
        return 0.0, (0.0, 0.0, 0.0), InertiaTensor()

    com: Vec3 = (mx / mass, my / mass, mz / mass)
    i_origin = InertiaTensor(ixx, iyy, izz, ixy, ixz, iyz)
    return mass, com, origin_to_com_tensor(i_origin, mass, com)
