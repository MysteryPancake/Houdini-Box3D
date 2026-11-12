// Box3DShapes.h
//
// Houdini-free, header-only helpers: shape parameters from point attributes, packed-prim
// matrix math, hashing, collision-group patterns. No Box3D or HDK includes, so
// tests/core_physics.cpp covers them without Houdini.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>
#include <unordered_map>
#include <vector>

namespace b3dop {

enum class ShapeType : int
{
    Box      = 0,
    Sphere   = 1,
    Convex   = 2,   // one convex hull of the packed geometry
    Concave  = 3,   // several convex hulls on ONE body (packed geometry pieces, grouped by `name`)
    Capsule  = 4,   // axis along the body's Y
    None     = 5,   // no collision shape: a static body that nothing collides with
};

// A box, sphere or capsule in body-local space. `center` is its offset from the body origin
// (non-zero e.g. when fitted to packed geometry not centred on the pivot). Convex / concave
// shapes are hulls instead (Box3DWorld::createHull).
struct ShapeSpec
{
    ShapeType type           = ShapeType::Box;
    float     halfExtents[3] = { 0.5f, 0.5f, 0.5f };   // Box
    float     radius         = 0.5f;                   // Sphere, Capsule
    float     halfLength     = 0.5f;                   // Capsule: half the straight section, along Y
    float     center[3]      = { 0.0f, 0.0f, 0.0f };   // every primitive
    float     rot[4]         = { 0.0f, 0.0f, 0.0f, 1.0f };   // x, y, z, w: orientation in body space. Box and capsule; a sphere ignores it
};

// `bullet_georep` -> ShapeType, Houdini's lowercase names, exact match. `compound` and
// `concave` are both Concave. False (out untouched) for null, empty or unknown values,
// including `cylinder` and `plane`, which Box3D has no shape for.
inline bool
parseShapeType(const char *name, ShapeType &out)
{
    if (!name || !*name)
        return false;
    struct Entry { const char *name; ShapeType type; };
    static const Entry kEntries[] = {
        { "convexhull", ShapeType::Convex   },
        { "concave",    ShapeType::Concave  },
        { "box",        ShapeType::Box      },
        { "capsule",    ShapeType::Capsule  },
        { "compound",   ShapeType::Concave  },
        { "sphere",     ShapeType::Sphere   },
        { "none",       ShapeType::None     },
    };
    for (const Entry &e : kEntries)
        if (std::strcmp(name, e.name) == 0)
        {
            out = e.type;
            return true;
        }
    return false;
}

// Smallest box half-size component / sphere / capsule size a shape may have.
constexpr float kMinExtent = 1e-4f;

// |v|, at least kMinExtent; non-finite -> kMinExtent.
inline float
clampExtent(float v)
{
    v = std::fabs(v);
    return (std::isfinite(v) && v > kMinExtent) ? v : kMinExtent;
}

inline bool
finite3(const float v[3])
{
    return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
}

// The shape fills a box of size pscale * scale (Copy to Points on a unit primitive). Sphere:
// diameter = largest component. Capsule along Y: radius = half the larger of x / z, total
// height = y. Components are made positive and at least 1e-4.
inline ShapeSpec
makeShapeSpec(ShapeType type, float pscale, const float scale[3])
{
    const float sx = clampExtent(pscale * scale[0]);
    const float sy = clampExtent(pscale * scale[1]);
    const float sz = clampExtent(pscale * scale[2]);

    ShapeSpec s;
    // Convex / concave: the fallback box when there is no geometry to hull.
    s.type = (type == ShapeType::Sphere || type == ShapeType::Capsule) ? type : ShapeType::Box;
    if (type == ShapeType::Sphere)
        s.radius = 0.5f * std::max({ sx, sy, sz });
    else if (type == ShapeType::Capsule)
    {
        s.radius     = 0.5f * std::max(sx, sz);
        s.halfLength = std::max(0.5f * sy - s.radius, 0.0f);
    }
    else
    {
        s.halfExtents[0] = 0.5f * sx;
        s.halfExtents[1] = 0.5f * sy;
        s.halfExtents[2] = 0.5f * sz;
    }
    return s;
}

// ---------------------------------------------------------------------------
// Explicit primitives (Bullet Data with AutoFit off). Each is optional:
//   bullet_primS   box HALF extents (Bullet Data documents Box Size as a half-size)
//   bullet_radius  sphere / capsule radius
//   bullet_length  capsule straight section along Y, caps not included (assumed)
//   bullet_primT   offset from the body origin
//   bullet_primR   Euler degrees, order XYZ (assumed; the solver builds the quaternion)
// ---------------------------------------------------------------------------
struct PrimOverrides
{
    bool  hasSize = false, hasRot = false, hasPos = false, hasRadius = false, hasLength = false;
    float size[3]   = { 1.0f, 1.0f, 1.0f };
    float rot[4]    = { 0.0f, 0.0f, 0.0f, 1.0f };   // unit quaternion x, y, z, w (the solver builds it with the HDK)
    float pos[3]    = { 0.0f, 0.0f, 0.0f };
    float radius    = 0.5f;
    float length    = 1.0f;

    bool any() const { return hasSize || hasRot || hasPos || hasRadius || hasLength; }
};

// Rotate v by the unit quaternion q.
inline void
quatRotate(const float q[4], const float v[3], float out[3])
{
    const float qx = q[0], qy = q[1], qz = q[2], qw = q[3];
    // t = 2 * cross(q.xyz, v);  v' = v + w * t + cross(q.xyz, t)
    const float tx = 2.0f * (qy * v[2] - qz * v[1]);
    const float ty = 2.0f * (qz * v[0] - qx * v[2]);
    const float tz = 2.0f * (qx * v[1] - qy * v[0]);
    out[0] = v[0] + qw * tx + (qy * tz - qz * ty);
    out[1] = v[1] + qw * ty + (qz * tx - qx * tz);
    out[2] = v[2] + qw * tz + (qx * ty - qy * tx);
}

// Non-finite values are ignored; sizes are clamped like makeShapeSpec.
inline void
applyPrimOverrides(ShapeSpec &s, const PrimOverrides &o)
{
    if (o.hasSize && s.type == ShapeType::Box && finite3(o.size))
        for (int i = 0; i < 3; ++i)
            s.halfExtents[i] = clampExtent(o.size[i]);

    if (o.hasRadius && std::isfinite(o.radius) && (s.type == ShapeType::Sphere || s.type == ShapeType::Capsule))
        s.radius = clampExtent(o.radius);

    if (o.hasLength && std::isfinite(o.length) && s.type == ShapeType::Capsule)
        s.halfLength = std::max(0.5f * o.length, 0.0f);

    if (o.hasPos && finite3(o.pos))
        for (int i = 0; i < 3; ++i)
            s.center[i] = o.pos[i];

    if (o.hasRot && finite3(o.rot) && std::isfinite(o.rot[3]))
        for (int i = 0; i < 4; ++i)
            s.rot[i] = o.rot[i];
}

inline float
shapeVolume(const ShapeSpec &s)
{
    const float pi = 3.14159265358979f;
    const float r  = s.radius;
    switch (s.type)
    {
        case ShapeType::Sphere:
            return (4.0f / 3.0f) * pi * r * r * r;
        case ShapeType::Capsule:
            return pi * r * r * (2.0f * s.halfLength) + (4.0f / 3.0f) * pi * r * r * r;
        default:
            return 8.0f * s.halfExtents[0] * s.halfExtents[1] * s.halfExtents[2];
    }
}

// Object id in the high 32 bits, the point's `id` (else its number) in the low 32.
inline uint64_t
makeBodyKey(int objectId, int64_t pointId)
{
    return (uint64_t(uint32_t(objectId)) << 32) | uint64_t(uint32_t(pointId));
}

// ---------------------------------------------------------------------------
// Packed primitives and hull point preparation
//
// Houdini matrices are row-vector (p' = p * M, translation in row 3); everything here uses
// that layout (m3[row*3+col], m4[row*4+col]). Quaternions are x, y, z, w.
// ---------------------------------------------------------------------------

// The upper-left 3x3 of a 4x4 (both row-major).
inline void
upper3x3(const double m4[16], double m3[9])
{
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            m3[r * 3 + c] = m4[r * 4 + c];
}

// A 3x3's row lengths (the per-row scale of M = S * R). False if a row is zero or non-finite.
// The plugin splits rotations with the HDK (UT_Matrix3D::extractScales); this Houdini-free
// version is only for scale keys.
inline bool
matrixRowLengths(const double m[9], double out[3])
{
    out[0] = out[1] = out[2] = 1.0;
    double l[3];
    for (int r = 0; r < 3; ++r)
    {
        l[r] = std::sqrt(m[r * 3] * m[r * 3] + m[r * 3 + 1] * m[r * 3 + 1] + m[r * 3 + 2] * m[r * 3 + 2]);
        if (!(l[r] > 1e-12) || !std::isfinite(l[r]))
            return false;
    }
    out[0] = l[0];  out[1] = l[1];  out[2] = l[2];
    return true;
}

// The scale packedPointsToBodyLocal bakes in: m4's row lengths, rounded to 5 significant
// digits so instances differing by float noise share a hull. False if degenerate.
inline bool
packedScale(const double m4[16], double S[3])
{
    double m3[9];
    upper3x3(m4, m3);
    if (!matrixRowLengths(m3, S))
        return false;
    // Round to 5 significant digits.
    for (int j = 0; j < 3; ++j)
    {
        const double v = S[j];
        if (!(std::fabs(v) > 0.0) || !std::isfinite(v))
            continue;
        const double step = std::pow(10.0, std::floor(std::log10(std::fabs(v))) - 4.0);
        S[j] = std::round(v / step) * step;
    }
    return true;
}

// Hashing for hull keys and signatures: 8 bytes per round (a multiply and a shift), the tail
// byte by byte. Keys only live for the process, so the exact function may change freely.
constexpr uint64_t kHashSeed  = 1469598103934665603ull;
constexpr uint64_t kHashMul   = 0xff51afd7ed558ccdull;

// Mixes one 64-bit value in (by value, so the same on any byte order).
inline uint64_t
mixHash(uint64_t h, uint64_t v)
{
    h = (h ^ v) * kHashMul;
    return h ^ (h >> 32);
}

inline uint64_t
hashBytes(uint64_t h, const void *data, size_t byteCount)
{
    const unsigned char *b = static_cast<const unsigned char *>(data);
    size_t i = 0;
    for (; i + 8 <= byteCount; i += 8)
    {
        uint64_t w;
        std::memcpy(&w, b + i, 8);
        h = mixHash(h, w);
    }
    uint64_t tail = 0;
    for (size_t k = 0; i < byteCount; ++i, ++k)
        tail |= uint64_t(b[i]) << (8 * k);
    return mixHash(h, tail ^ (uint64_t(byteCount) << 56));
}

// The raw bytes of `floatCount` floats, with the count mixed in.
inline uint64_t
hashFloats(const float *xyz, size_t floatCount)
{
    return hashBytes(kHashSeed ^ uint64_t(floatCount), xyz, floatCount * sizeof(float));
}

// A double's bits; -0 and +0 hash alike.
inline uint64_t
mixDouble(uint64_t h, double d)
{
    d += 0.0;
    uint64_t u;
    std::memcpy(&u, &d, sizeof u);
    return mixHash(h, u);
}

// A packed hull's cache key: the source's point hash plus what differs per instance (pivot,
// scale). `kind`: 1 merged hull, 2 per piece. Same inputs as packedPointsToBodyLocal, so a
// hit means the same body-local points. Never 0.
inline uint64_t
packedHullKey(uint64_t sourceHash, const double pivot[3], const double S[3], uint32_t kind)
{
    uint64_t h = mixHash(sourceHash, kind);
    for (int j = 0; j < 3; ++j)
        h = mixDouble(mixDouble(h, pivot[j]), S[j]);
    return h ? h : 1ull;
}

// Connected sets of primitives (sharing points, directly or not). primPoints[i]: points of
// primitive i. Returns each primitive's set, numbered in order of first appearance.
template <class PointId>
std::vector<int>
connectedSets(const std::vector<std::vector<PointId>> &primPoints)
{
    const int        n = int(primPoints.size());
    std::vector<int> parent(n);
    for (int i = 0; i < n; ++i)
        parent[i] = i;
    auto root = [&](int i) {
        while (parent[i] != i)
            i = parent[i] = parent[parent[i]];
        return i;
    };

    std::unordered_map<PointId, int> firstUser;     // point -> first primitive that uses it
    for (int i = 0; i < n; ++i)
        for (const PointId &pt : primPoints[i])
        {
            const auto ins = firstUser.emplace(pt, i);
            if (ins.second)
                continue;
            const int a = root(ins.first->second), b = root(i);
            if (a != b)
                parent[a > b ? a : b] = a > b ? b : a;
        }

    std::vector<int> label(n, -1), sets(n);
    int              next = 0;
    for (int i = 0; i < n; ++i)
    {
        const int r = root(i);
        if (label[r] < 0)
            label[r] = next++;
        sets[i] = label[r];
    }
    return sets;
}

// A packed primitive's inner points in body-local space: (p - pivot) * S, with m4 the full
// transform (only its 3x3 is read, M = S * R) and pivot GEO_PrimPacked::getPivot (the inner
// point at the packed point's P, i.e. the body origin and centre of mass). S is rounded
// (packedScale) so rotated instances produce identical points. Degenerate m4: unchanged.
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

// A box / sphere / capsule fitted to the points' bounds: auto-fit on packed geometry, and the
// fallback when points cannot be hulled. Extents are clamped like makeShapeSpec.
inline bool
fitShapeToPoints(ShapeType type, const float *xyz, int count, ShapeSpec &out)
{
    if (count <= 0)
        return false;
    float lo[3] = { xyz[0], xyz[1], xyz[2] };
    float hi[3] = { xyz[0], xyz[1], xyz[2] };
    for (int i = 0; i < count; ++i)
        for (int j = 0; j < 3; ++j)
        {
            const float v = xyz[3 * i + j];
            if (!std::isfinite(v)) return false;
            if (v < lo[j]) lo[j] = v;
            if (v > hi[j]) hi[j] = v;
        }

    float half[3], mid[3];
    for (int j = 0; j < 3; ++j)
    {
        mid[j]  = 0.5f * (lo[j] + hi[j]);
        half[j] = 0.5f * clampExtent(hi[j] - lo[j]);
    }

    out = ShapeSpec();
    for (int j = 0; j < 3; ++j) out.center[j] = mid[j];
    if (type == ShapeType::Sphere)
    {
        out.type   = ShapeType::Sphere;
        out.radius = std::max({ half[0], half[1], half[2] });
    }
    else if (type == ShapeType::Capsule)
    {
        // Y axis; radius covers the wider of x / z; the caps sit inside the bounds.
        out.type       = ShapeType::Capsule;
        out.radius     = std::max(half[0], half[2]);
        out.halfLength = std::max(half[1] - out.radius, 0.0f);
    }
    else
    {
        out.type = ShapeType::Box;
        for (int j = 0; j < 3; ++j) out.halfExtents[j] = half[j];
    }
    return true;
}

// ---------------------------------------------------------------------------
// How values change on their way into and out of the geometry. Shared by the read path and
// the write-back's prediction, so the solver can tell its own writes from other edits.
// ---------------------------------------------------------------------------

// Box3D asserts on non-unit quaternions; attributes may hold zeros or off-unit values.
inline void sanitizeQuat(float q[4])
{
    double len2 = 0.0;
    for (int i = 0; i < 4; ++i) len2 += double(q[i]) * double(q[i]);
    if (!(len2 > 1e-12) || !std::isfinite(len2))
    {
        q[0] = q[1] = q[2] = 0.0f;  q[3] = 1.0f;
        return;
    }
    const float inv = float(1.0 / std::sqrt(len2));
    for (int i = 0; i < 4; ++i) q[i] *= inv;
}

// P is stored as float: double -> float -> double. False if not finite.
inline bool storedPosition(const double in[3], double out[3])
{
    for (int i = 0; i < 3; ++i)
        out[i] = double(float(in[i]));
    return std::isfinite(out[0]) && std::isfinite(out[1]) && std::isfinite(out[2]);
}

} // namespace b3dop
