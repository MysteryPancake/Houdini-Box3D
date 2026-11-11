// Box3DShapes.h
//
// Houdini-free helpers that turn point-attribute values into Box3D shape
// parameters. Header-only, no Box3D and no HDK includes, so it is unit-tested
// in tests/core_physics.cpp without Houdini.
//
// The solver reads attributes off the points in the object's Geometry and
// calls these. See README.md for the attribute table.
#pragma once

#include <cmath>
#include <cstdint>
#include <cstring>
#include <vector>

namespace b3dop {

enum class ShapeType : int
{
    Box     = 0,
    Sphere  = 1,
    Convex  = 2,    // one convex hull of the packed geometry
    Concave = 3,    // several convex hulls on ONE body (packed geometry pieces, grouped by `name`)
};

// A primitive shape (box or sphere) in body-local space. `center` is its offset
// from the body origin: zero for point-attribute shapes, non-zero when a box or
// sphere is fitted to packed geometry whose bounds are not centred on the pivot.
// Convex / Concave are not described by a ShapeSpec: they need point data, see
// Box3DWorld::createHull().
struct ShapeSpec
{
    ShapeType type           = ShapeType::Box;
    float     halfExtents[3] = { 0.5f, 0.5f, 0.5f };   // Box
    float     radius         = 0.5f;                   // Sphere
    float     center[3]      = { 0.0f, 0.0f, 0.0f };   // Box and Sphere
};

// Case-insensitive compare, ASCII only. (strcasecmp is not portable to MSVC.)
inline bool
shapeNameEquals(const char *a, const char *b)
{
    for (; *a && *b; ++a, ++b)
    {
        char ca = *a, cb = *b;
        if (ca >= 'A' && ca <= 'Z') ca = char(ca - 'A' + 'a');
        if (cb >= 'A' && cb <= 'Z') cb = char(cb - 'A' + 'a');
        if (ca != cb) return false;
    }
    return *a == *b;
}

// Parse the value of the `shape` point attribute.
// Returns false for null / empty / unknown strings (caller uses its default).
inline bool
parseShapeType(const char *name, ShapeType &out)
{
    if (!name || !*name)
        return false;
    if (shapeNameEquals(name, "box") || shapeNameEquals(name, "cube"))
    {
        out = ShapeType::Box;
        return true;
    }
    if (shapeNameEquals(name, "sphere") || shapeNameEquals(name, "ball"))
    {
        out = ShapeType::Sphere;
        return true;
    }
    if (shapeNameEquals(name, "convex") || shapeNameEquals(name, "hull"))
    {
        out = ShapeType::Convex;
        return true;
    }
    if (shapeNameEquals(name, "concave") || shapeNameEquals(name, "compound"))
    {
        out = ShapeType::Concave;
        return true;
    }
    return false;
}

// True for the shapes that are built from point data (hulls) rather than from
// size attributes.
inline bool
shapeNeedsGeometry(ShapeType t)
{
    return t == ShapeType::Convex || t == ShapeType::Concave;
}

// Size convention: the shape is a UNIT shape (box 1x1x1, sphere diameter 1)
// scaled by pscale * scale, which is what Copy to Points does to a unit
// primitive. For a sphere the largest scale component is used, since a sphere
// cannot be non-uniformly scaled.
//
// Zero / negative / non-finite components are clamped to a tiny positive size
// so Box3D never sees a degenerate shape.
inline ShapeSpec
makeShapeSpec(ShapeType type, float pscale, const float scale[3])
{
    const float kMin = 1e-4f;
    auto sane = [&](float v) {
        v = std::fabs(v);
        return (std::isfinite(v) && v > kMin) ? v : kMin;
    };

    const float sx = sane(pscale * scale[0]);
    const float sy = sane(pscale * scale[1]);
    const float sz = sane(pscale * scale[2]);

    ShapeSpec s;
    // Convex / Concave have no size attributes of their own; this is only the
    // box a caller falls back to when it has no geometry to hull.
    s.type = (type == ShapeType::Sphere) ? ShapeType::Sphere : ShapeType::Box;
    if (type == ShapeType::Sphere)
    {
        float m = sx > sy ? sx : sy;
        m = m > sz ? m : sz;
        s.radius = 0.5f * m;
    }
    else
    {
        s.halfExtents[0] = 0.5f * sx;
        s.halfExtents[1] = 0.5f * sy;
        s.halfExtents[2] = 0.5f * sz;
    }
    return s;
}

inline float
shapeVolume(const ShapeSpec &s)
{
    if (s.type == ShapeType::Sphere)
        return (4.0f / 3.0f) * 3.14159265358979f * s.radius * s.radius * s.radius;
    return 8.0f * s.halfExtents[0] * s.halfExtents[1] * s.halfExtents[2];
}

// Houdini's `mass` attribute -> Box3D density (mass / volume). Returns
// fallback if mass or volume is not usable.
inline float
densityFromMass(float mass, const ShapeSpec &s, float fallback)
{
    const float vol = shapeVolume(s);
    if (!(mass > 0.0f) || !std::isfinite(mass) || !(vol > 0.0f))
        return fallback;
    return mass / vol;
}

// One world-unique body id per point: the DOP object id in the high 32 bits
// and the point's id (the `id` attribute, else its point number) in the low 32.
inline uint64_t
makeBodyKey(int objectId, int64_t pointId)
{
    return (uint64_t(uint32_t(objectId)) << 32) | uint64_t(uint32_t(pointId));
}

// ---------------------------------------------------------------------------
// Packed primitives and hull point preparation
//
// Houdini matrices are ROW-vector: p' = p * M, and UT_Matrix4D stores the
// translation in row 3. Everything below uses that layout (m3[row*3+col],
// m4[row*4+col]) so the HDK side can copy a matrix straight in. Quaternions are
// x, y, z, w and rotate vectors in the usual way (same as Box3D / UT_Quaternion).
// ---------------------------------------------------------------------------

// Row-vector rotation matrix for unit quaternion q (x, y, z, w).
inline void
quatToMatrix3(const float q[4], double m[9])
{
    const double x = q[0], y = q[1], z = q[2], w = q[3];
    // Column-vector matrix R, then transposed into row-vector layout.
    const double r00 = 1 - 2 * (y * y + z * z), r01 = 2 * (x * y - z * w), r02 = 2 * (x * z + y * w);
    const double r10 = 2 * (x * y + z * w), r11 = 1 - 2 * (x * x + z * z), r12 = 2 * (y * z - x * w);
    const double r20 = 2 * (x * z - y * w), r21 = 2 * (y * z + x * w), r22 = 1 - 2 * (x * x + y * y);
    m[0] = r00; m[1] = r10; m[2] = r20;
    m[3] = r01; m[4] = r11; m[5] = r21;
    m[6] = r02; m[7] = r12; m[8] = r22;
}

// Split a row-vector matrix M = S * R (scale each row, then rotate) into the
// per-row scale and a unit rotation quaternion. Returns false (identity,
// scale 1) if the matrix is degenerate. Shear is dropped (rows are
// orthonormalised), a mirrored matrix loses its mirror.
inline bool
matrix3ToQuatScale(const double m[9], float outQ[4], double outScale[3])
{
    outQ[0] = outQ[1] = outQ[2] = 0.0f;  outQ[3] = 1.0f;
    outScale[0] = outScale[1] = outScale[2] = 1.0;

    double e0[3] = { m[0], m[1], m[2] };
    double e1[3] = { m[3], m[4], m[5] };
    double e2[3] = { m[6], m[7], m[8] };
    auto len = [](const double *v) { return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]); };
    const double l0 = len(e0), l1 = len(e1), l2 = len(e2);
    if (!(l0 > 1e-12) || !(l1 > 1e-12) || !(l2 > 1e-12) ||
        !std::isfinite(l0) || !std::isfinite(l1) || !std::isfinite(l2))
        return false;
    outScale[0] = l0;  outScale[1] = l1;  outScale[2] = l2;

    for (int i = 0; i < 3; ++i) e0[i] /= l0;
    double d = e1[0] * e0[0] + e1[1] * e0[1] + e1[2] * e0[2];
    for (int i = 0; i < 3; ++i) e1[i] -= d * e0[i];
    const double n1 = len(e1);
    if (!(n1 > 1e-12)) return false;
    for (int i = 0; i < 3; ++i) e1[i] /= n1;
    // Right-handed third axis (drops a mirror).
    e2[0] = e0[1] * e1[2] - e0[2] * e1[1];
    e2[1] = e0[2] * e1[0] - e0[0] * e1[2];
    e2[2] = e0[0] * e1[1] - e0[1] * e1[0];

    // Rows e0,e1,e2 are the row-vector matrix; column-vector R[i][j] = row[j][i].
    const double r00 = e0[0], r01 = e1[0], r02 = e2[0];
    const double r10 = e0[1], r11 = e1[1], r12 = e2[1];
    const double r20 = e0[2], r21 = e1[2], r22 = e2[2];

    double x, y, z, w;
    const double tr = r00 + r11 + r22;
    if (tr > 0.0)
    {
        const double s = std::sqrt(tr + 1.0) * 2.0;
        w = 0.25 * s;  x = (r21 - r12) / s;  y = (r02 - r20) / s;  z = (r10 - r01) / s;
    }
    else if (r00 > r11 && r00 > r22)
    {
        const double s = std::sqrt(1.0 + r00 - r11 - r22) * 2.0;
        w = (r21 - r12) / s;  x = 0.25 * s;  y = (r01 + r10) / s;  z = (r02 + r20) / s;
    }
    else if (r11 > r22)
    {
        const double s = std::sqrt(1.0 + r11 - r00 - r22) * 2.0;
        w = (r02 - r20) / s;  x = (r01 + r10) / s;  y = 0.25 * s;  z = (r12 + r21) / s;
    }
    else
    {
        const double s = std::sqrt(1.0 + r22 - r00 - r11) * 2.0;
        w = (r10 - r01) / s;  x = (r02 + r20) / s;  y = (r12 + r21) / s;  z = 0.25 * s;
    }
    const double n = std::sqrt(x * x + y * y + z * z + w * w);
    if (!(n > 1e-12)) return false;
    outQ[0] = float(x / n);  outQ[1] = float(y / n);  outQ[2] = float(z / n);  outQ[3] = float(w / n);
    return true;
}

// A new local 3x3 for a packed primitive: keep the scale of `oldM3` (per row)
// and replace the rotation with q. For uniform scale this is exact.
inline void
composeMatrix3(const float q[4], const double oldM3[9], double outM3[9])
{
    float  oq[4];
    double sc[3];
    matrix3ToQuatScale(oldM3, oq, sc);          // sc stays 1 if the old matrix is degenerate
    double r[9];
    quatToMatrix3(q, r);
    for (int row = 0; row < 3; ++row)
        for (int col = 0; col < 3; ++col)
            outM3[row * 3 + col] = sc[row] * r[row * 3 + col];
}

// Axis-aligned bounds of a point set. Returns false if empty / non-finite.
inline bool
computeBoundsImpl(const float *xyz, int count, float outMin[3], float outMax[3])
{
    if (count <= 0) return false;
    for (int j = 0; j < 3; ++j) { outMin[j] = xyz[j]; outMax[j] = xyz[j]; }
    for (int i = 0; i < count; ++i)
        for (int j = 0; j < 3; ++j)
        {
            const float v = xyz[3 * i + j];
            if (!std::isfinite(v)) return false;
            if (v < outMin[j]) outMin[j] = v;
            if (v > outMax[j]) outMax[j] = v;
        }
    return true;
}

// Round to 5 significant digits. Used to make a derived value (the scale)
// identical across instances whose transforms differ only by float noise.
inline double
quantizeSig5(double v)
{
    if (!(std::fabs(v) > 0.0) || !std::isfinite(v))
        return v;
    const double step = std::pow(10.0, std::floor(std::log10(std::fabs(v))) - 4.0);
    return std::round(v / step) * step;
}

// The per-axis scale that packedPointsToBodyLocal bakes into the points: the row
// lengths of the 3x3 of m4, rounded to 5 significant digits so instances that
// differ only by float noise agree. Returns false if the matrix is degenerate.
inline bool
packedScale(const double m4[16], double S[3])
{
    const double m3[9] = { m4[0], m4[1], m4[2],  m4[4], m4[5], m4[6],  m4[8], m4[9], m4[10] };
    float q[4];
    if (!matrix3ToQuatScale(m3, q, S))
        return false;
    for (int j = 0; j < 3; ++j)
        S[j] = quantizeSig5(S[j]);
    return true;
}

// FNV-1a style hashing, used to key hulls. hashFloats hashes the raw bytes of a
// point set (once per shared packed source); mixHash folds one more value in.
inline uint64_t
hashFloats(const float *xyz, size_t floatCount)
{
    uint64_t h = 1469598103934665603ull ^ uint64_t(floatCount);
    const unsigned char *b = reinterpret_cast<const unsigned char *>(xyz);
    for (size_t i = 0, n = floatCount * sizeof(float); i < n; ++i)
    {
        h ^= b[i];
        h *= 1099511628211ull;
    }
    return h;
}

inline uint64_t
mixHash(uint64_t h, uint64_t v)
{
    for (int i = 0; i < 8; ++i)
    {
        h ^= (v >> (8 * i)) & 0xffull;
        h *= 1099511628211ull;
    }
    return h;
}

// The cache key of a packed hull. Houdini says which prims share one source
// (the same packed detail), so the source's points are hashed ONCE
// (`sourceHash`) and an instance only adds what differs: the pivot and the
// scale. `kind` separates one merged hull (convex) from per-piece hulls
// (concave). Same inputs as packedPointsToBodyLocal, so the same key means the
// same body-local points: the points need only be built on a cache miss.
inline uint64_t
packedHullKey(uint64_t sourceHash, const double pivot[3], const double S[3], uint32_t kind)
{
    uint64_t h = mixHash(sourceHash, kind);
    for (int j = 0; j < 3; ++j)
    {
        const double pv = pivot[j] + 0.0, sv = S[j] + 0.0;      // +0.0: -0 and +0 hash alike
        uint64_t a, b;
        std::memcpy(&a, &pv, sizeof a);
        std::memcpy(&b, &sv, sizeof b);
        h = mixHash(mixHash(h, a), b);
    }
    return h ? h : 1ull;
}

// Bring the points of a packed primitive's inner geometry into BODY-LOCAL space.
//
//   m4     the primitive's full transform (inner space -> world), row-vector,
//          i.e. GU_PrimPacked::getFullTransform4. Only its 3x3 (scale and
//          rotation) is used; its translation row is NOT read.
//   pivot  the primitive's pivot in inner space, i.e. GEO_PrimPacked::getPivot.
//          It is the inner point that sits at the packed point's P, so it is
//          the body origin.
//
// With M = S * R (scale, then rotate), the result is
//
//        (p - pivot) * S
//
// the points relative to the pivot, with the primitive's scale baked in and its
// rotation removed (the body's rotation puts it back).
//
// Instancing: instances of one packed primitive share the same source points and
// the same pivot, so the pivot is used exactly as stored. Only the scale is
// derived from the matrix (row lengths), and a rotated instance's row lengths
// carry float noise (1 vs 0.99999994), which would give each instance a
// different hull key. So S is rounded to 5 significant digits; the output is
// then identical across instances and they hit the hull cache. The cost is a
// relative error of about 1e-5 in the scale.
// If the matrix is degenerate the points are returned unchanged.
inline void
packedPointsToBodyLocal(const float *xyz, int count, const double m4[16],
                        const double pivotIn[3], std::vector<float> &out)
{
    out.assign(xyz, xyz + size_t(count) * 3);
    if (count <= 0)
        return;

    double S[3];
    if (!packedScale(m4, S))
        return;

    for (int i = 0; i < count; ++i)
        for (int j = 0; j < 3; ++j)
            out[size_t(3 * i + j)] = float((double(xyz[3 * i + j]) - pivotIn[j]) * S[j]);
}

// Multiply every point by (pscale * scale).
inline void
scalePoints(std::vector<float> &xyz, float pscale, const float scale[3])
{
    for (size_t i = 0; i + 2 < xyz.size(); i += 3)
        for (int j = 0; j < 3; ++j)
            xyz[i + j] *= pscale * scale[j];
}

// A box or sphere fitted to a point set: the fallback when there are no
// hullable points (fewer than 4, or flat), and the shape for `box` / `sphere`
// on a packed primitive. The box is the bounds; the sphere is centred on the
// bounds and sized to the largest extent, like the point-attribute sphere.
// Degenerate (zero) extents are clamped like makeShapeSpec does.
inline bool
fitShapeToPoints(ShapeType type, const float *xyz, int count, ShapeSpec &out)
{
    float lo[3], hi[3];
    if (!computeBoundsImpl(xyz, count, lo, hi))
        return false;

    const float kMin = 1e-4f;
    float half[3], mid[3];
    for (int j = 0; j < 3; ++j)
    {
        mid[j]  = 0.5f * (lo[j] + hi[j]);
        half[j] = 0.5f * (hi[j] - lo[j]);
        if (!(half[j] > 0.5f * kMin)) half[j] = 0.5f * kMin;
    }

    out = ShapeSpec();
    for (int j = 0; j < 3; ++j) out.center[j] = mid[j];
    if (type == ShapeType::Sphere)
    {
        out.type = ShapeType::Sphere;
        float m = half[0] > half[1] ? half[0] : half[1];
        out.radius = m > half[2] ? m : half[2];
    }
    else
    {
        out.type = ShapeType::Box;
        for (int j = 0; j < 3; ++j) out.halfExtents[j] = half[j];
    }
    return true;
}

} // namespace b3dop
