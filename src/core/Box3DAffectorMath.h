// Box3DAffectorMath.h
//
// Houdini-free matrix helpers for affector (Static Object) colliders. Matrices are 16 doubles in
// Houdini's row-vector layout (UT_DMatrix4: p' = p * M, translation in the last row).
#pragma once

#include <cmath>

namespace b3dop {

// If the matrix is a pure rotation + translation (no scale, shear or perspective), returns true and
// fills `pos` and `quat` (x, y, z, w). Otherwise false: the caller bakes the matrix into the vertices.
inline bool
decomposeRigid(const double m[16], double pos[3], float quat[4], double tol = 1e-4)
{
    // No perspective.
    if (std::fabs(m[3]) > tol || std::fabs(m[7]) > tol || std::fabs(m[11]) > tol || std::fabs(m[15] - 1.0) > tol)
        return false;

    // Rows of the upper 3x3 are the images of the axes: they must be unit length and orthogonal.
    const double *r0 = m, *r1 = m + 4, *r2 = m + 8;
    auto dot = [](const double *a, const double *b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; };
    if (std::fabs(dot(r0, r0) - 1.0) > tol || std::fabs(dot(r1, r1) - 1.0) > tol ||
        std::fabs(dot(r2, r2) - 1.0) > tol)
        return false;
    if (std::fabs(dot(r0, r1)) > tol || std::fabs(dot(r0, r2)) > tol || std::fabs(dot(r1, r2)) > tol)
        return false;
    // Right-handed (a mirror cannot be a body rotation).
    const double det = r0[0] * (r1[1] * r2[2] - r1[2] * r2[1]) - r0[1] * (r1[0] * r2[2] - r1[2] * r2[0]) +
                       r0[2] * (r1[0] * r2[1] - r1[1] * r2[0]);
    if (det < 0.0)
        return false;

    pos[0] = m[12]; pos[1] = m[13]; pos[2] = m[14];

    // R (column-vector convention) is the transpose of the 3x3: R[i][j] = m[4 * j + i].
    auto R = [&](int i, int j) { return m[4 * j + i]; };
    const double tr = R(0, 0) + R(1, 1) + R(2, 2);
    double x, y, z, w;
    if (tr > 0.0)
    {
        const double s = std::sqrt(tr + 1.0) * 2.0;
        w = 0.25 * s;
        x = (R(2, 1) - R(1, 2)) / s;
        y = (R(0, 2) - R(2, 0)) / s;
        z = (R(1, 0) - R(0, 1)) / s;
    }
    else if (R(0, 0) > R(1, 1) && R(0, 0) > R(2, 2))
    {
        const double s = std::sqrt(1.0 + R(0, 0) - R(1, 1) - R(2, 2)) * 2.0;
        w = (R(2, 1) - R(1, 2)) / s;
        x = 0.25 * s;
        y = (R(0, 1) + R(1, 0)) / s;
        z = (R(0, 2) + R(2, 0)) / s;
    }
    else if (R(1, 1) > R(2, 2))
    {
        const double s = std::sqrt(1.0 + R(1, 1) - R(0, 0) - R(2, 2)) * 2.0;
        w = (R(0, 2) - R(2, 0)) / s;
        x = (R(0, 1) + R(1, 0)) / s;
        y = 0.25 * s;
        z = (R(1, 2) + R(2, 1)) / s;
    }
    else
    {
        const double s = std::sqrt(1.0 + R(2, 2) - R(0, 0) - R(1, 1)) * 2.0;
        w = (R(1, 0) - R(0, 1)) / s;
        x = (R(0, 2) + R(2, 0)) / s;
        y = (R(1, 2) + R(2, 1)) / s;
        z = 0.25 * s;
    }
    const double n = std::sqrt(x * x + y * y + z * z + w * w);
    if (!(n > 0.0) || !std::isfinite(n))
        return false;
    quat[0] = float(x / n); quat[1] = float(y / n); quat[2] = float(z / n); quat[3] = float(w / n);
    return true;
}

// A rotation, a positive scale per axis and a translation (M = S * R + T in row-vector terms: each
// row of the 3x3 is a rotated axis times its scale). No shear, mirror or perspective. Fills `pos`,
// `quat` and `scale` and returns true; false otherwise (the caller bakes the matrix).
inline bool
decomposeScaledRigid(const double m[16], double pos[3], float quat[4], float scale[3], double tol = 1e-4)
{
    double n[16];
    for (int i = 0; i < 16; ++i)
        n[i] = m[i];
    for (int r = 0; r < 3; ++r)
    {
        double *row = n + 4 * r;
        const double len = std::sqrt(row[0] * row[0] + row[1] * row[1] + row[2] * row[2]);
        if (!(len > 1e-6) || !std::isfinite(len))
            return false;
        for (int c = 0; c < 3; ++c)
            row[c] /= len;
        scale[r] = float(len);
    }
    return decomposeRigid(n, pos, quat, tol);
}

// p * M for a point (row-vector convention), in double.
inline void
transformPoint(const double m[16], const double p[3], double out[3])
{
    for (int j = 0; j < 3; ++j)
        out[j] = p[0] * m[j] + p[1] * m[4 + j] + p[2] * m[8 + j] + m[12 + j];
}

} // namespace b3dop
