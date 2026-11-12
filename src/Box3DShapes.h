// Box3DShapes.h
//
// Houdini-free, header-only helpers: shape parameters from point attributes, packed-prim
// matrix math, hashing, collision-group patterns. No Box3D or HDK includes, so
// tests/core_physics.cpp covers them without Houdini.
#pragma once

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

// parseShapeType with a memo by string value (a scene has a handful, usually one).
class ShapeTypeCache
{
public:
    bool parse(const char *name, ShapeType &out)
    {
        if (!name || !*name)
            return false;
        if (myLast < myEntries.size() && myEntries[myLast].text == name)
            return apply(myEntries[myLast], out);
        for (size_t i = 0; i < myEntries.size(); ++i)
            if (myEntries[i].text == name)
            {
                myLast = i;
                return apply(myEntries[i], out);
            }
        Entry e;
        e.text = name;
        e.ok   = parseShapeType(name, e.type);
        if (myEntries.size() >= kMaxEntries)     // pathological: every point has its own string
            myEntries.clear();
        myEntries.push_back(e);
        myLast = myEntries.size() - 1;
        return apply(e, out);
    }
    size_t size() const { return myEntries.size(); }

private:
    struct Entry
    {
        std::string text;
        ShapeType   type = ShapeType::Box;
        bool        ok   = false;
    };
    static constexpr size_t kMaxEntries = 32;
    static bool apply(const Entry &e, ShapeType &out)
    {
        if (e.ok)
            out = e.type;
        return e.ok;
    }
    std::vector<Entry> myEntries;
    size_t             myLast = 0;
};

// The shape fills a box of size pscale * scale (Copy to Points on a unit primitive). Sphere:
// diameter = largest component. Capsule along Y: radius = half the larger of x / z, total
// height = y. Components are made positive and at least 1e-4.
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
    // Convex / concave: the fallback box when there is no geometry to hull.
    s.type = (type == ShapeType::Sphere || type == ShapeType::Capsule) ? type : ShapeType::Box;
    if (type == ShapeType::Sphere)
    {
        float m = sx > sy ? sx : sy;
        m = m > sz ? m : sz;
        s.radius = 0.5f * m;
    }
    else if (type == ShapeType::Capsule)
    {
        const float r = 0.5f * (sx > sz ? sx : sz);
        s.radius     = r;
        s.halfLength = (0.5f * sy - r) > 0.0f ? (0.5f * sy - r) : 0.0f;
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
    const float kMin = 1e-4f;
    auto sane = [&](float v) {
        v = std::fabs(v);
        return v > kMin ? v : kMin;
    };
    auto finite3 = [](const float v[3]) {
        return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
    };

    if (o.hasSize && s.type == ShapeType::Box && finite3(o.size))
        for (int i = 0; i < 3; ++i)
            s.halfExtents[i] = sane(o.size[i]);

    if (o.hasRadius && std::isfinite(o.radius) && (s.type == ShapeType::Sphere || s.type == ShapeType::Capsule))
        s.radius = sane(o.radius);

    if (o.hasLength && std::isfinite(o.length) && s.type == ShapeType::Capsule)
        s.halfLength = o.length > 0.0f ? 0.5f * o.length : 0.0f;

    if (o.hasPos && finite3(o.pos))
        for (int i = 0; i < 3; ++i)
            s.center[i] = o.pos[i];

    if (o.hasRot && std::isfinite(o.rot[0]) && std::isfinite(o.rot[1]) &&
        std::isfinite(o.rot[2]) && std::isfinite(o.rot[3]))
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
    const double m3[9] = { m4[0], m4[1], m4[2],  m4[4], m4[5], m4[6],  m4[8], m4[9], m4[10] };
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

// FNV-1a style hashing for hull keys and signatures.
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

// A packed hull's cache key: the source's point hash plus what differs per instance (pivot,
// scale). `kind`: 1 merged hull, 2 per piece. Same inputs as packedPointsToBodyLocal, so a
// hit means the same body-local points.
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
    else if (type == ShapeType::Capsule)
    {
        // Y axis; radius covers the wider of x / z; the caps sit inside the bounds.
        out.type       = ShapeType::Capsule;
        out.radius     = half[0] > half[2] ? half[0] : half[2];
        out.halfLength = (half[1] - out.radius) > 0.0f ? (half[1] - out.radius) : 0.0f;
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

// ---------------------------------------------------------------------------
// Collision groups (`collisiongroup`, `collisionignore`, `activationignore`)
//
// Houdini patterns: globs (`*`, `?`, `[abc]`) separated by spaces or commas, left to right;
// `^pat` removes what it matches. In Houdini the HDK's UT_String::multiMatch does the matching
// (see patternMatcher); globMatch / matchPatternList are the Houdini-free stand-in for tests.
// ---------------------------------------------------------------------------

// One case-sensitive glob against one name.
inline bool
globMatch(const char *pat, const char *str)
{
    const char *starPat = nullptr, *starStr = nullptr;
    while (*str)
    {
        if (*pat == '[')
        {
            const char *q = pat + 1;
            bool neg = (*q == '!' || *q == '^');
            if (neg) ++q;
            bool hit = false;
            const char *close = q;
            while (*close && *close != ']') ++close;
            if (*close == ']')
            {
                for (const char *c = q; c < close; ++c)
                {
                    if (c + 2 < close && c[1] == '-')
                    {
                        hit = hit || (*str >= c[0] && *str <= c[2]);
                        c += 2;
                    }
                    else
                        hit = hit || (*str == *c);
                }
                if (hit != neg)
                {
                    pat = close + 1;
                    ++str;
                    continue;
                }
            }
            else if (*str == '[')   // unterminated: a literal '['
            {
                ++pat; ++str;
                continue;
            }
        }
        else if (*pat == '?' || (*pat && *pat != '*' && *pat == *str))
        {
            ++pat; ++str;
            continue;
        }
        if (*pat == '*')
        {
            starPat = ++pat;
            starStr = str;
            continue;
        }
        if (!starPat)
            return false;
        pat = starPat;
        str = ++starStr;
    }
    while (*pat == '*') ++pat;
    return *pat == 0;
}

// A pattern list against one name. Empty patterns or names never match.
inline bool
matchPatternList(const std::string &patterns, const std::string &name)
{
    if (patterns.empty() || name.empty())
        return false;
    bool matched = false;
    size_t i = 0;
    std::string tok;
    while (i < patterns.size())
    {
        while (i < patterns.size() && (patterns[i] == ' ' || patterns[i] == '\t' || patterns[i] == ','))
            ++i;
        size_t j = i;
        while (j < patterns.size() && patterns[j] != ' ' && patterns[j] != '\t' && patterns[j] != ',')
            ++j;
        if (j > i)
        {
            const bool neg = patterns[i] == '^';
            tok.assign(patterns, neg ? i + 1 : i, j - (neg ? i + 1 : i));
            if (!tok.empty() && globMatch(tok.c_str(), name.c_str()))
                matched = !neg;
        }
        i = j;
    }
    return matched;
}

// The matcher patternsMatchProfile uses: matchPatternList by default (tests); the plugin
// installs Houdini's own (UT_String::multiMatch) in initializeSIM. Set before any solve; read
// only afterwards, so it is safe from Box3D's threads.
using PatternMatchFn = bool (*)(const std::string &patterns, const std::string &name);
inline PatternMatchFn &
patternMatcher()
{
    static PatternMatchFn fn = &matchPatternList;
    return fn;
}

// A body's names for pattern matching (group, object name, object id) and its ignore patterns.
struct FilterProfile
{
    std::string group;          // `collisiongroup` (may be empty)
    std::string objectName;     // the DOP object's name
    std::string objectId;       // the DOP object's id, as text
    std::string ignore;         // `collisionignore` patterns (may be empty)
};

// Does `patterns` match any name of `who`?
inline bool
patternsMatchProfile(const std::string &patterns, const FilterProfile &who)
{
    const PatternMatchFn match = patternMatcher();
    return match(patterns, who.group) || match(patterns, who.objectName) || match(patterns, who.objectId);
}

// Every distinct profile, and whether two collide (neither ignores the other). Profiles are
// only added. finish() precomputes an answer table (past kMaxMatrix: matched on the fly), so
// shouldCollide is read-only and safe from Box3D's worker threads during a step.
class CollisionFilterTable
{
public:
    static constexpr size_t kMaxMatrix = 2048;

    int profile(const FilterProfile &p)
    {
        std::string key;
        key.reserve(p.group.size() + p.objectName.size() + p.objectId.size() + p.ignore.size() + 3);
        key += p.group;  key += '\x1f';  key += p.objectName;  key += '\x1f';
        key += p.objectId;  key += '\x1f';  key += p.ignore;
        auto it = myIndex.find(key);
        if (it != myIndex.end())
            return it->second;
        const int idx = int(myProfiles.size());
        myProfiles.push_back(p);
        myIndex.emplace(std::move(key), idx);
        myAnyIgnore = myAnyIgnore || !p.ignore.empty();
        myDirty = true;
        return idx;
    }

    void finish()
    {
        if (!myDirty)
            return;
        myDirty = false;
        const size_t n = myProfiles.size();
        if (!myAnyIgnore || n > kMaxMatrix)
        {
            myMatrix.clear();
            myMatrixN = 0;
            return;
        }
        myMatrix.assign(n * n, 1);
        for (size_t a = 0; a < n; ++a)
            for (size_t b = a; b < n; ++b)
            {
                const bool c = computeCollide(int(a), int(b));
                myMatrix[a * n + b] = myMatrix[b * n + a] = c ? 1 : 0;
            }
        myMatrixN = n;
    }

    // a / b: profile indices; a negative index (no profile) always collides.
    bool shouldCollide(int a, int b) const
    {
        if (!myAnyIgnore || a < 0 || b < 0 || size_t(a) >= myProfiles.size() || size_t(b) >= myProfiles.size())
            return true;
        if (size_t(a) < myMatrixN && size_t(b) < myMatrixN)
            return myMatrix[size_t(a) * myMatrixN + size_t(b)] != 0;
        return computeCollide(a, b);
    }

    const FilterProfile *get(int i) const
    {
        return (i >= 0 && size_t(i) < myProfiles.size()) ? &myProfiles[size_t(i)] : nullptr;
    }
    size_t size() const { return myProfiles.size(); }

private:
    bool computeCollide(int a, int b) const
    {
        const FilterProfile &pa = myProfiles[size_t(a)], &pb = myProfiles[size_t(b)];
        return !patternsMatchProfile(pa.ignore, pb) && !patternsMatchProfile(pb.ignore, pa);
    }

    std::vector<FilterProfile>           myProfiles;
    std::unordered_map<std::string, int> myIndex;
    std::vector<uint8_t>                 myMatrix;
    size_t                               myMatrixN   = 0;
    bool                                 myAnyIgnore = false;
    bool                                 myDirty     = false;
};

} // namespace b3dop
