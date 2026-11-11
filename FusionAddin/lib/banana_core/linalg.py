"""
linalg.py: Pure-Python 3D vector and 3x3 matrix linear algebra.
No adsk, no numpy. Optimized for clarity and deterministic numerical stability.
"""

import math
from typing import Sequence, Tuple, Union

Vec3 = Tuple[float, float, float]
Mat3 = Tuple[Vec3, Vec3, Vec3]


# ---------------------------------------------------------------------------
# Vector 3 Operations
# ---------------------------------------------------------------------------

def vec3_make(x: float, y: float, z: float) -> Vec3:
    return (float(x), float(y), float(z))


def vec3_add(a: Sequence[float], b: Sequence[float]) -> Vec3:
    return (float(a[0] + b[0]), float(a[1] + b[1]), float(a[2] + b[2]))


def vec3_sub(a: Sequence[float], b: Sequence[float]) -> Vec3:
    return (float(a[0] - b[0]), float(a[1] - b[1]), float(a[2] - b[2]))


def vec3_scale(v: Sequence[float], s: float) -> Vec3:
    s_f = float(s)
    return (float(v[0] * s_f), float(v[1] * s_f), float(v[2] * s_f))


def vec3_dot(a: Sequence[float], b: Sequence[float]) -> float:
    return float(a[0] * b[0] + a[1] * b[1] + a[2] * b[2])


def vec3_cross(a: Sequence[float], b: Sequence[float]) -> Vec3:
    return (
        float(a[1] * b[2] - a[2] * b[1]),
        float(a[2] * b[0] - a[0] * b[2]),
        float(a[0] * b[1] - a[1] * b[0]),
    )


def vec3_norm_sq(v: Sequence[float]) -> float:
    return float(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])


def vec3_norm(v: Sequence[float]) -> float:
    return math.sqrt(vec3_norm_sq(v))


def vec3_normalized(v: Sequence[float], eps: float = 1e-12) -> Vec3:
    n = vec3_norm(v)
    if n < eps:
        return (0.0, 0.0, 0.0)
    inv = 1.0 / n
    return (float(v[0] * inv), float(v[1] * inv), float(v[2] * inv))


# ---------------------------------------------------------------------------
# Matrix 3x3 Operations
# ---------------------------------------------------------------------------

def mat3_zeros() -> Mat3:
    return (
        (0.0, 0.0, 0.0),
        (0.0, 0.0, 0.0),
        (0.0, 0.0, 0.0),
    )


def mat3_identity() -> Mat3:
    return (
        (1.0, 0.0, 0.0),
        (0.0, 1.0, 0.0),
        (0.0, 0.0, 1.0),
    )


def mat3_from_rows(
    r0: Sequence[float], r1: Sequence[float], r2: Sequence[float]
) -> Mat3:
    return (
        (float(r0[0]), float(r0[1]), float(r0[2])),
        (float(r1[0]), float(r1[1]), float(r1[2])),
        (float(r2[0]), float(r2[1]), float(r2[2])),
    )


def mat3_transpose(m: Mat3) -> Mat3:
    return (
        (m[0][0], m[1][0], m[2][0]),
        (m[0][1], m[1][1], m[2][1]),
        (m[0][2], m[1][2], m[2][2]),
    )


def mat3_mul(a: Mat3, b: Mat3) -> Mat3:
    return (
        (
            a[0][0] * b[0][0] + a[0][1] * b[1][0] + a[0][2] * b[2][0],
            a[0][0] * b[0][1] + a[0][1] * b[1][1] + a[0][2] * b[2][1],
            a[0][0] * b[0][2] + a[0][1] * b[1][2] + a[0][2] * b[2][2],
        ),
        (
            a[1][0] * b[0][0] + a[1][1] * b[1][0] + a[1][2] * b[2][0],
            a[1][0] * b[0][1] + a[1][1] * b[1][1] + a[1][2] * b[2][1],
            a[1][0] * b[0][2] + a[1][1] * b[1][2] + a[1][2] * b[2][2],
        ),
        (
            a[2][0] * b[0][0] + a[2][1] * b[1][0] + a[2][2] * b[2][0],
            a[2][0] * b[0][1] + a[2][1] * b[1][1] + a[2][2] * b[2][1],
            a[2][0] * b[0][2] + a[2][1] * b[1][2] + a[2][2] * b[2][2],
        ),
    )


def mat3_mul_vec(m: Mat3, v: Sequence[float]) -> Vec3:
    vx, vy, vz = float(v[0]), float(v[1]), float(v[2])
    return (
        float(m[0][0] * vx + m[0][1] * vy + m[0][2] * vz),
        float(m[1][0] * vx + m[1][1] * vy + m[1][2] * vz),
        float(m[2][0] * vx + m[2][1] * vy + m[2][2] * vz),
    )


def mat3_scale(m: Mat3, s: float) -> Mat3:
    s_f = float(s)
    return (
        (m[0][0] * s_f, m[0][1] * s_f, m[0][2] * s_f),
        (m[1][0] * s_f, m[1][1] * s_f, m[1][2] * s_f),
        (m[2][0] * s_f, m[2][1] * s_f, m[2][2] * s_f),
    )


def mat3_add(a: Mat3, b: Mat3) -> Mat3:
    return (
        (a[0][0] + b[0][0], a[0][1] + b[0][1], a[0][2] + b[0][2]),
        (a[1][0] + b[1][0], a[1][1] + b[1][1], a[1][2] + b[1][2]),
        (a[2][0] + b[2][0], a[2][1] + b[2][1], a[2][2] + b[2][2]),
    )


def mat3_sub(a: Mat3, b: Mat3) -> Mat3:
    return (
        (a[0][0] - b[0][0], a[0][1] - b[0][1], a[0][2] - b[0][2]),
        (a[1][0] - b[1][0], a[1][1] - b[1][1], a[1][2] - b[1][2]),
        (a[2][0] - b[2][0], a[2][1] - b[2][1], a[2][2] - b[2][2]),
    )


def mat3_trace(m: Mat3) -> float:
    return float(m[0][0] + m[1][1] + m[2][2])


def mat3_det(m: Mat3) -> float:
    return float(
        m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
        - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
        + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])
    )


def mat3_frobenius_norm(m: Mat3) -> float:
    total = 0.0
    for r in range(3):
        for c in range(3):
            total += m[r][c] * m[r][c]
    return math.sqrt(total)


def mat3_relative_error(test: Mat3, ref: Mat3, eps: float = 1e-12) -> float:
    diff = mat3_sub(test, ref)
    ref_norm = mat3_frobenius_norm(ref)
    if ref_norm < eps:
        return mat3_frobenius_norm(diff)
    return mat3_frobenius_norm(diff) / ref_norm


def is_symmetric(m: Mat3, tol: float = 1e-9) -> bool:
    return (
        abs(m[0][1] - m[1][0]) <= tol
        and abs(m[0][2] - m[2][0]) <= tol
        and abs(m[1][2] - m[2][1]) <= tol
    )


# ---------------------------------------------------------------------------
# Symmetric 3x3 Analytical Eigenvalues (Closed-form Cardano/Trig method)
# ---------------------------------------------------------------------------

def symmetric_eigenvalues(m: Mat3) -> Tuple[float, float, float]:
    """
    Computes all three real eigenvalues of a real symmetric 3x3 matrix in closed form
    using the trigonometric Cardano method (Nickalls 1993 / Smith 1961).
    Returns (lambda_1, lambda_2, lambda_3) sorted in descending order:
    lambda_1 >= lambda_2 >= lambda_3.
    """
    a = float(m[0][0])
    b = float(m[1][1])
    c = float(m[2][2])
    d = float((m[0][1] + m[1][0]) * 0.5)
    e = float((m[0][2] + m[2][0]) * 0.5)
    f = float((m[1][2] + m[2][1]) * 0.5)

    q = (a + b + c) / 3.0

    b00 = a - q
    b11 = b - q
    b22 = c - q

    p_sq = (b00 * b00 + b11 * b11 + b22 * b22 + 2.0 * (d * d + e * e + f * f)) / 6.0

    if p_sq < 1e-24:
        # Matrix is effectively a scalar multiple of identity: A = q * I
        return (q, q, q)

    p = math.sqrt(p_sq)

    # Determinant of B = A - q * I
    det_B = (
        b00 * (b11 * b22 - f * f)
        - d * (d * b22 - e * f)
        + e * (d * f - e * b11)
    )

    r = det_B / (2.0 * p * p * p)

    # Guard against float rounding exceeding [-1, 1]
    if r <= -1.0:
        phi = math.pi / 3.0
    elif r >= 1.0:
        phi = 0.0
    else:
        phi = math.acos(r) / 3.0

    # Because phi in [0, pi/3], the roots in descending order are:
    # cos(phi) >= cos(phi + 4*pi/3) >= cos(phi + 2*pi/3)
    eig1 = q + 2.0 * p * math.cos(phi)
    eig3 = q + 2.0 * p * math.cos(phi + 2.0 * math.pi / 3.0)
    eig2 = 3.0 * q - eig1 - eig3  # ensures exact trace conservation

    # Final guarantee of descending order
    res = [eig1, eig2, eig3]
    res.sort(reverse=True)
    return (res[0], res[1], res[2])
