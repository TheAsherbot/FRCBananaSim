import math
import unittest
import sys
import os

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "lib")))

from banana_core.frames import (
    FrameConverter,
    nwu_to_engine,
    engine_to_nwu,
    CM_TO_M,
    INERTIA_CM2_TO_M2,
)
from banana_core.linalg import symmetric_eigenvalues, mat3_det


class TestFrames(unittest.TestCase):
    def test_default_team_frame(self):
        # Default: Up = +Y, Forward = +Z
        conv = FrameConverter(fusion_up="+Y", fusion_forward="+Z")
        self.assertAlmostEqual(mat3_det(conv.R_F_to_N), 1.0, places=12)

        # 100 cm in Fusion +Z (Forward) -> 1.0 m in NWU +X
        pF_fwd = (0.0, 0.0, 100.0)
        pN_fwd = conv.fusion_to_nwu_point(pF_fwd)
        self.assertAlmostEqual(pN_fwd[0], 1.0, places=12)
        self.assertAlmostEqual(pN_fwd[1], 0.0, places=12)
        self.assertAlmostEqual(pN_fwd[2], 0.0, places=12)

        # 100 cm in Fusion +X (Left) -> 1.0 m in NWU +Y
        pF_left = (100.0, 0.0, 0.0)
        pN_left = conv.fusion_to_nwu_point(pF_left)
        self.assertAlmostEqual(pN_left[0], 0.0, places=12)
        self.assertAlmostEqual(pN_left[1], 1.0, places=12)
        self.assertAlmostEqual(pN_left[2], 0.0, places=12)

        # 100 cm in Fusion +Y (Up) -> 1.0 m in NWU +Z
        pF_up = (0.0, 100.0, 0.0)
        pN_up = conv.fusion_to_nwu_point(pF_up)
        self.assertAlmostEqual(pN_up[0], 0.0, places=12)
        self.assertAlmostEqual(pN_up[1], 0.0, places=12)
        self.assertAlmostEqual(pN_up[2], 1.0, places=12)

    def test_point_round_trip_with_origin(self):
        conv = FrameConverter(fusion_up="+Y", fusion_forward="+Z")
        origin_m = (0.35, 0.12, 0.78)
        p_original_cm = (45.0, 120.0, -85.0)

        pN = conv.fusion_to_nwu_point(p_original_cm, origin_fusion_m=origin_m)
        pF_back = conv.nwu_to_fusion_point(pN, origin_fusion_m=origin_m)

        self.assertAlmostEqual(pF_back[0], p_original_cm[0], places=10)
        self.assertAlmostEqual(pF_back[1], p_original_cm[1], places=10)
        self.assertAlmostEqual(pF_back[2], p_original_cm[2], places=10)

    def test_vector_conversion(self):
        conv = FrameConverter(fusion_up="+Y", fusion_forward="+Z")
        vF = (0.0, 1.0, 0.0)  # Fusion Up
        vN = conv.fusion_to_nwu_vector(vF)
        self.assertAlmostEqual(vN[0], 0.0)
        self.assertAlmostEqual(vN[1], 0.0)
        self.assertAlmostEqual(vN[2], 1.0)  # NWU Up (+Z)

        vF_back = conv.nwu_to_fusion_vector(vN)
        self.assertAlmostEqual(vF_back[0], vF[0])
        self.assertAlmostEqual(vF_back[1], vF[1])
        self.assertAlmostEqual(vF_back[2], vF[2])

    def test_inertia_conversion_and_eigenvalue_invariance(self):
        conv = FrameConverter(fusion_up="+Y", fusion_forward="+Z")
        # Arbitrary symmetric tensor in Fusion space (kg * cm^2)
        I_F = (
            (10000.0, 500.0, -200.0),
            (500.0, 8000.0, 100.0),
            (-200.0, 100.0, 12000.0),
        )
        I_N = conv.fusion_to_nwu_inertia(I_F)

        # Eigenvalues in NWU must equal eigenvalues in Fusion scaled by 1e-4
        eigs_F = symmetric_eigenvalues(I_F)
        eigs_N = symmetric_eigenvalues(I_N)

        for eF, eN in zip(eigs_F, eigs_N):
            self.assertAlmostEqual(eN, eF * INERTIA_CM2_TO_M2, places=10)

    def test_z_up_config(self):
        # Alternative convention: Up = +Z, Forward = +X
        conv = FrameConverter(fusion_up="+Z", fusion_forward="+X")
        self.assertAlmostEqual(mat3_det(conv.R_F_to_N), 1.0, places=12)

        # Fwd (+X) -> NWU (+X)
        vN = conv.fusion_to_nwu_vector((1.0, 0.0, 0.0))
        self.assertEqual(vN, (1.0, 0.0, 0.0))

        # Up (+Z) -> NWU (+Z)
        vN = conv.fusion_to_nwu_vector((0.0, 0.0, 1.0))
        self.assertEqual(vN, (0.0, 0.0, 1.0))

    def test_invalid_axis_configs(self):
        with self.assertRaises(ValueError):
            FrameConverter(fusion_up="+Z", fusion_forward="+Z")  # Parallel
        with self.assertRaises(ValueError):
            FrameConverter(fusion_up="+Z", fusion_forward="-Z")  # Anti-parallel
        with self.assertRaises(ValueError):
            FrameConverter(fusion_up="UP", fusion_forward="+X")  # Invalid token

    def test_engine_gltf_conversion(self):
        # NWU (+X Fwd, +Y Left, +Z Up) -> Engine (+X Left, +Y Up, +Z Fwd)
        nwu = (1.5, 2.5, 3.5)
        eng = nwu_to_engine(nwu)
        self.assertEqual(eng, (2.5, 3.5, 1.5))
        back = engine_to_nwu(eng)
        self.assertEqual(back, nwu)


if __name__ == "__main__":
    unittest.main()
