import math
import unittest
import sys
import os

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "lib")))

from banana_core.inertia import (
    InertiaTensor,
    parallel_axis_matrix,
    origin_to_com_tensor,
    com_to_offset_tensor,
    aggregate_link_mass_props,
    BodyMassProps,
)


class TestInertia(unittest.TestCase):
    def test_tensor_serialization(self):
        t = InertiaTensor(ixx=1.23456789, iyy=2.34567891, izz=3.45678912,
                          ixy=-0.12345678, ixz=0.0001, iyz=-0.0002)
        d = t.to_dict(digits=6)
        self.assertEqual(d["ixx"], 1.234568)
        self.assertEqual(d["ixy"], -0.123457)

        t_back = InertiaTensor.from_dict(d)
        self.assertAlmostEqual(t_back.ixx, 1.234568)
        self.assertAlmostEqual(t_back.ixy, -0.123457)

    def test_parallel_axis_analytic_cube(self):
        # 10 kg cube of side L = 0.2 m
        m = 10.0
        L = 0.2
        # Inertia at CoM for a cube: (1/6) * m * L^2
        I_diag_com = (1.0 / 6.0) * m * (L * L)  # (1/6) * 10 * 0.04 = 0.06666666666666667
        I_com = InertiaTensor(
            ixx=I_diag_com, iyy=I_diag_com, izz=I_diag_com,
            ixy=0.0, ixz=0.0, iyz=0.0
        )

        # Place cube corner at origin, so CoM is at (0.1, 0.1, 0.1)
        com = (0.1, 0.1, 0.1)
        # Shift to origin: com_to_offset_tensor with offset = (0.1, 0.1, 0.1)
        I_origin = com_to_offset_tensor(I_com, m, com)

        # Expected at origin:
        # I_xx = I_com + m * (cy^2 + cz^2) = 0.0666667 + 10 * (0.01 + 0.01) = 0.2666667
        # I_xy = I_com - m * cx * cy = 0 - 10 * (0.01) = -0.10
        self.assertAlmostEqual(I_origin.ixx, I_diag_com + 0.20, places=12)
        self.assertAlmostEqual(I_origin.iyy, I_diag_com + 0.20, places=12)
        self.assertAlmostEqual(I_origin.izz, I_diag_com + 0.20, places=12)
        self.assertAlmostEqual(I_origin.ixy, -0.10, places=12)
        self.assertAlmostEqual(I_origin.ixz, -0.10, places=12)
        self.assertAlmostEqual(I_origin.iyz, -0.10, places=12)

        # Transfer back from origin to CoM
        I_recovered_com = origin_to_com_tensor(I_origin, m, com)
        self.assertAlmostEqual(I_recovered_com.ixx, I_diag_com, places=12)
        self.assertAlmostEqual(I_recovered_com.iyy, I_diag_com, places=12)
        self.assertAlmostEqual(I_recovered_com.izz, I_diag_com, places=12)
        self.assertAlmostEqual(I_recovered_com.ixy, 0.0, places=12)
        self.assertAlmostEqual(I_recovered_com.ixz, 0.0, places=12)
        self.assertAlmostEqual(I_recovered_com.iyz, 0.0, places=12)

    def test_composite_aggregation_two_cubes(self):
        # Two identical 5 kg cubes at x = +0.5 m and x = -0.5 m
        m = 5.0
        I_each = InertiaTensor(ixx=0.05, iyy=0.05, izz=0.05, ixy=0.0, ixz=0.0, iyz=0.0)

        b1 = BodyMassProps(mass_kg=m, com=(0.5, 0.0, 0.0), inertia_at_com=I_each)
        b2 = BodyMassProps(mass_kg=m, com=(-0.5, 0.0, 0.0), inertia_at_com=I_each)

        total_mass, composite_com, composite_I = aggregate_link_mass_props([b1, b2])

        self.assertAlmostEqual(total_mass, 10.0, places=12)
        self.assertAlmostEqual(composite_com[0], 0.0, places=12)
        self.assertAlmostEqual(composite_com[1], 0.0, places=12)
        self.assertAlmostEqual(composite_com[2], 0.0, places=12)

        # Expected composite inertia about (0,0,0):
        # I_xx = 0.05 + 0.05 = 0.10 (no y or z offsets)
        # I_yy = 0.05 + 0.05 + 5*(0.5^2) + 5*((-0.5)^2) = 0.10 + 1.25 + 1.25 = 2.60
        # I_zz = 0.05 + 0.05 + 5*(0.5^2) + 5*((-0.5)^2) = 2.60
        # Off diagonals = 0
        self.assertAlmostEqual(composite_I.ixx, 0.10, places=12)
        self.assertAlmostEqual(composite_I.iyy, 2.60, places=12)
        self.assertAlmostEqual(composite_I.izz, 2.60, places=12)
        self.assertAlmostEqual(composite_I.ixy, 0.0, places=12)
        self.assertAlmostEqual(composite_I.ixz, 0.0, places=12)
        self.assertAlmostEqual(composite_I.iyz, 0.0, places=12)

    def test_sylvester_positive_definiteness(self):
        # Good physical tensor
        good = InertiaTensor(ixx=1.0, iyy=1.5, izz=2.0, ixy=0.1, ixz=-0.1, iyz=0.05)
        self.assertTrue(good.is_positive_definite())

        # Negative diagonal element
        bad1 = InertiaTensor(ixx=-1.0, iyy=1.5, izz=2.0)
        self.assertFalse(bad1.is_positive_definite())

        # Off-diagonal too large (violating det(2x2) > 0)
        bad2 = InertiaTensor(ixx=1.0, iyy=1.0, izz=1.0, ixy=2.0)
        self.assertFalse(bad2.is_positive_definite())

    def test_triangle_inequality(self):
        # Real physical box: I1 = 0.10, I2 = 0.08, I3 = 0.04
        # 0.08 + 0.04 = 0.12 >= 0.10 -> Valid
        t_valid = InertiaTensor(ixx=0.10, iyy=0.08, izz=0.04)
        valid, moments = t_valid.check_triangle_inequality()
        self.assertTrue(valid)

        # Nonphysical tensor: I1 = 10.0, I2 = 1.0, I3 = 1.0
        # 1.0 + 1.0 = 2.0 < 10.0 -> Invalid!
        t_invalid = InertiaTensor(ixx=10.0, iyy=1.0, izz=1.0)
        valid, moments = t_invalid.check_triangle_inequality()
        self.assertFalse(valid)


class TestInertiaAudit(unittest.TestCase):
    def test_small_physical_tensor_is_positive_definite_but_fails_cpp_loader(self):
        # ~0.3 kg bracket: perfectly physical, but below the C++ loader's absolute 1e-9 minor test.
        small = InertiaTensor(ixx=6e-4, iyy=8e-4, izz=9e-4)
        self.assertTrue(small.is_positive_definite())
        self.assertFalse(small.passes_loader_check())

        healthy = InertiaTensor(ixx=0.2, iyy=0.3, izz=0.4)
        self.assertTrue(healthy.is_positive_definite())
        self.assertTrue(healthy.passes_loader_check())

    def test_triangle_inequality_is_scale_relative(self):
        # Same shape at 1e-6 scale must still be flagged as nonphysical.
        bad_small = InertiaTensor(ixx=10e-6, iyy=1e-6, izz=1e-6)
        valid, _ = bad_small.check_triangle_inequality()
        self.assertFalse(valid)

    def _origin_tuple(self, I_com, mass, com):
        o = com_to_offset_tensor(I_com, mass, com)
        return (o.ixx, o.iyy, o.izz, o.ixy, o.ixz, o.iyz)

    def test_detect_product_sign(self):
        from banana_core.inertia import detect_product_sign
        I_com = InertiaTensor(ixx=100.0, iyy=150.0, izz=200.0, ixy=-20.0, ixz=10.0, iyz=-15.0)
        mass, com = 3.0, (10.0, 20.0, 5.0)
        principal = I_com.principal_moments()
        tensor_elem = self._origin_tuple(I_com, mass, com)

        # Fusion returning tensor-element products -> sign +1
        sign, confident = detect_product_sign([(mass, com, tensor_elem, principal)])
        self.assertTrue(confident)
        self.assertEqual(sign, 1)

        # Fusion returning positive-integral products -> sign -1
        integral = (tensor_elem[0], tensor_elem[1], tensor_elem[2], -tensor_elem[3], -tensor_elem[4], -tensor_elem[5])
        sign, confident = detect_product_sign([(mass, com, integral, principal)])
        self.assertTrue(confident)
        self.assertEqual(sign, -1)

    def test_detect_product_sign_not_confident_without_products(self):
        from banana_core.inertia import detect_product_sign
        I_com = InertiaTensor(ixx=1.0, iyy=1.0, izz=1.0)
        sign, confident = detect_product_sign([(2.0, (0.0, 0.0, 0.0), (1.0, 1.0, 1.0, 0.0, 0.0, 0.0), I_com.principal_moments())])
        self.assertFalse(confident)

    def test_reference_properties_subtract_excluded_bodies(self):
        from banana_core.inertia import compute_reference_properties
        a_I = InertiaTensor(ixx=0.05, iyy=0.05, izz=0.05)
        b_I = InertiaTensor(ixx=0.02, iyy=0.02, izz=0.02)
        a_com, b_com = (1.0, 0.0, 0.0), (0.0, 2.0, 0.0)
        a_o = com_to_offset_tensor(a_I, 5.0, a_com)
        b_o = com_to_offset_tensor(b_I, 3.0, b_com)
        total_I = InertiaTensor(
            a_o.ixx + b_o.ixx, a_o.iyy + b_o.iyy, a_o.izz + b_o.izz,
            a_o.ixy + b_o.ixy, a_o.ixz + b_o.ixz, a_o.iyz + b_o.iyz,
        )
        total_com = ((5.0 * 1.0) / 8.0, (3.0 * 2.0) / 8.0, 0.0)

        mass, com, I = compute_reference_properties(8.0, total_com, total_I, [(3.0, b_com, b_o)])
        self.assertAlmostEqual(mass, 5.0, places=12)
        self.assertAlmostEqual(com[0], 1.0, places=12)
        self.assertAlmostEqual(com[1], 0.0, places=12)
        self.assertAlmostEqual(I.ixx, 0.05, places=12)
        self.assertAlmostEqual(I.iyy, 0.05, places=12)
        self.assertAlmostEqual(I.ixy, 0.0, places=12)


if __name__ == "__main__":
    unittest.main()
