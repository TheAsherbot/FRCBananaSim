import math
import unittest
import sys
import os

# Ensure lib is on sys.path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "lib")))

from banana_core.linalg import (
    vec3_add,
    vec3_sub,
    vec3_scale,
    vec3_dot,
    vec3_cross,
    vec3_norm,
    vec3_normalized,
    mat3_identity,
    mat3_zeros,
    mat3_from_rows,
    mat3_transpose,
    mat3_mul,
    mat3_mul_vec,
    mat3_scale,
    mat3_det,
    mat3_trace,
    mat3_frobenius_norm,
    mat3_relative_error,
    is_symmetric,
    symmetric_eigenvalues,
)

class TestLinalg(unittest.TestCase):
    def test_vec3_basic(self):
        v1 = (1.0, 2.0, 3.0)
        v2 = (4.0, 5.0, 6.0)
        self.assertEqual(vec3_add(v1, v2), (5.0, 7.0, 9.0))
        self.assertEqual(vec3_sub(v1, v2), (-3.0, -3.0, -3.0))
        self.assertEqual(vec3_scale(v1, 2.0), (2.0, 4.0, 6.0))
        self.assertAlmostEqual(vec3_dot(v1, v2), 4.0 + 10.0 + 18.0)
        self.assertEqual(vec3_cross((1.0, 0.0, 0.0), (0.0, 1.0, 0.0)), (0.0, 0.0, 1.0))
        self.assertAlmostEqual(vec3_norm((3.0, 4.0, 0.0)), 5.0)
        u = vec3_normalized((0.0, 10.0, 0.0))
        self.assertAlmostEqual(u[0], 0.0)
        self.assertAlmostEqual(u[1], 1.0)
        self.assertAlmostEqual(u[2], 0.0)

    def test_mat3_basic(self):
        I = mat3_identity()
        v = (1.5, -2.0, 4.0)
        self.assertEqual(mat3_mul_vec(I, v), v)
        self.assertAlmostEqual(mat3_det(I), 1.0)
        self.assertAlmostEqual(mat3_trace(I), 3.0)

        A = (
            (1.0, 2.0, 3.0),
            (0.0, 1.0, 4.0),
            (5.0, 6.0, 0.0),
        )
        self.assertAlmostEqual(mat3_det(A), 1.0)
        AT = mat3_transpose(A)
        self.assertEqual(AT[0], (1.0, 0.0, 5.0))
        self.assertEqual(AT[1], (2.0, 1.0, 6.0))
        self.assertEqual(AT[2], (3.0, 4.0, 0.0))

        I_test = mat3_mul(A, (
            (-24.0, 18.0, 5.0),
            (20.0, -15.0, -4.0),
            (-5.0, 4.0, 1.0),
        ))
        for r in range(3):
            for c in range(3):
                self.assertAlmostEqual(I_test[r][c], I[r][c], places=10)

    def test_symmetric_eigenvalues_diagonal(self):
        D = (
            (5.0, 0.0, 0.0),
            (0.0, 2.0, 0.0),
            (0.0, 0.0, 8.0),
        )
        e1, e2, e3 = symmetric_eigenvalues(D)
        self.assertAlmostEqual(e1, 8.0, places=12)
        self.assertAlmostEqual(e2, 5.0, places=12)
        self.assertAlmostEqual(e3, 2.0, places=12)

    def test_symmetric_eigenvalues_tridiagonal(self):
        # Matrix with known eigenvalues 2+sqrt(2), 2, 2-sqrt(2)
        A = (
            (2.0, 1.0, 0.0),
            (1.0, 2.0, 1.0),
            (0.0, 1.0, 2.0),
        )
        e1, e2, e3 = symmetric_eigenvalues(A)
        expected_e1 = 2.0 + math.sqrt(2.0)
        expected_e2 = 2.0
        expected_e3 = 2.0 - math.sqrt(2.0)
        self.assertAlmostEqual(e1, expected_e1, places=12)
        self.assertAlmostEqual(e2, expected_e2, places=12)
        self.assertAlmostEqual(e3, expected_e3, places=12)

        # Trace and determinant invariant
        self.assertAlmostEqual(e1 + e2 + e3, mat3_trace(A), places=12)
        self.assertAlmostEqual(e1 * e2 * e3, mat3_det(A), places=12)

    def test_symmetric_eigenvalues_rotated(self):
        # Rotate diagonal tensor D by a 45 degree yaw
        theta = math.pi / 4.0
        cos_t = math.cos(theta)
        sin_t = math.sin(theta)
        R = (
            (cos_t, -sin_t, 0.0),
            (sin_t, cos_t, 0.0),
            (0.0, 0.0, 1.0),
        )
        RT = mat3_transpose(R)
        D = (
            (10.0, 0.0, 0.0),
            (0.0, 3.0, 0.0),
            (0.0, 0.0, 1.0),
        )
        # A = R * D * R^T
        A = mat3_mul(mat3_mul(R, D), RT)
        self.assertTrue(is_symmetric(A))

        e1, e2, e3 = symmetric_eigenvalues(A)
        self.assertAlmostEqual(e1, 10.0, places=12)
        self.assertAlmostEqual(e2, 3.0, places=12)
        self.assertAlmostEqual(e3, 1.0, places=12)

    def test_symmetric_eigenvalues_isotropic(self):
        # Scalar matrix (sphere)
        A = (
            (4.2, 0.0, 0.0),
            (0.0, 4.2, 0.0),
            (0.0, 0.0, 4.2),
        )
        e1, e2, e3 = symmetric_eigenvalues(A)
        self.assertAlmostEqual(e1, 4.2, places=12)
        self.assertAlmostEqual(e2, 4.2, places=12)
        self.assertAlmostEqual(e3, 4.2, places=12)


if __name__ == "__main__":
    unittest.main()
