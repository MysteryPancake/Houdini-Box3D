// Box3DShapes.h
//
// Houdini-free, header-only helpers: shape parameters from point attributes,
// packed-prim matrix math, hashing, collision-group patterns. (Plain vector /
// quaternion math: Box3DMath.h.) No Box3D or HDK includes, so
// tests/core_physics.cpp covers them without Houdini.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>
#include <unordered_map>
#include <vector>

#include "Box3DMath.h" // finite3, copy3 / copy4, kDegToRad, quaternion helpers

namespace b3dop {

enum class ShapeType : int {
  Box = 0,
  Sphere = 1,
  Convex = 2, // one convex hull of the packed geometry
  // several convex hulls on ONE body (packed geometry pieces, grouped by
  // `name`)
  Concave = 3,
  Capsule = 4, // axis along the body's Y
  None = 5,    // no collision shape: a static body that nothing collides with
};

// A box, sphere or capsule in body-local space. `center` is its offset from the
// body origin (non-zero e.g. when fitted to packed geometry not centred on the
// pivot). Convex / concave shapes are hulls instead (Box3DWorld::createHull).
struct ShapeSpec {
  ShapeType type = ShapeType::Box;
  float halfExtents[3] = {0.5f, 0.5f, 0.5f}; // Box
  float radius = 0.5f;                       // Sphere, Capsule
  float halfLength = 0.5f; // Capsule: half the straight section, along Y
  float center[3] = {0.0f, 0.0f, 0.0f}; // every primitive
  float rot[4] = {0.0f, 0.0f, 0.0f,
                  // x, y, z, w: orientation in body space. Box and capsule; a
                  // sphere ignores it
                  1.0f};
};

// `bullet_georep` -> ShapeType, Houdini's lowercase names, exact match.
// `compound` and `concave` are both Concave. False (out untouched) for null,
// empty or unknown values, including `cylinder` and `plane`, which Box3D has no
// shape for.
inline bool parseShapeType(const char *name, ShapeType &out) {
  if (!name || !*name)
    return false;
  struct Entry {
    const char *name;
    ShapeType type;
  };
  static const Entry kEntries[] = {
      {"convexhull", ShapeType::Convex}, {"concave", ShapeType::Concave},
      {"box", ShapeType::Box},           {"capsule", ShapeType::Capsule},
      {"compound", ShapeType::Concave},  {"sphere", ShapeType::Sphere},
      {"none", ShapeType::None},
  };
  for (const Entry &e : kEntries)
    if (std::strcmp(name, e.name) == 0) {
      out = e.type;
      return true;
    }
  return false;
}

// Smallest box half-size component / sphere / capsule size a shape may have.
constexpr float kMinExtent = 1e-4f;

// |v|, at least kMinExtent; non-finite -> kMinExtent.
inline float clampExtent(float v) {
  v = std::fabs(v);
  return (std::isfinite(v) && v > kMinExtent) ? v : kMinExtent;
}

// The shape fills a box of size pscale * scale (Copy to Points on a unit
// primitive). Sphere: diameter = largest component. Capsule along Y: radius =
// half the larger of x / z, total height = y. Components are made positive and
// at least 1e-4.
inline ShapeSpec makeShapeSpec(ShapeType type, float pscale,
                               const float scale[3]) {
  const float sx = clampExtent(pscale * scale[0]);
  const float sy = clampExtent(pscale * scale[1]);
  const float sz = clampExtent(pscale * scale[2]);

  ShapeSpec s;
  // Convex / concave: the fallback box when there is no geometry to hull.
  s.type = (type == ShapeType::Sphere || type == ShapeType::Capsule)
               ? type
               : ShapeType::Box;
  if (type == ShapeType::Sphere)
    s.radius = 0.5f * std::max({sx, sy, sz});
  else if (type == ShapeType::Capsule) {
    s.radius = 0.5f * std::max(sx, sz);
    s.halfLength = std::max(0.5f * sy - s.radius, 0.0f);
  } else {
    s.halfExtents[0] = 0.5f * sx;
    s.halfExtents[1] = 0.5f * sy;
    s.halfExtents[2] = 0.5f * sz;
  }
  return s;
}

// ---------------------------------------------------------------------------
// Explicit primitives (Bullet Data with AutoFit off). Each is optional:
//   bullet_primS   box HALF extents (Bullet Data documents Box Size as a
//   half-size) bullet_radius  sphere / capsule radius bullet_length  capsule
//   straight section along Y, caps not included bullet_primT   offset from the
//   body origin bullet_primR   Euler degrees, order XYZ (the solver builds the
//   quaternion)
// ---------------------------------------------------------------------------
struct PrimOverrides {
  bool hasSize = false, hasRot = false, hasPos = false, hasRadius = false,
       hasLength = false;
  float size[3] = {1.0f, 1.0f, 1.0f};
  float rot[4] = {
      0.0f, 0.0f, 0.0f,
      1.0f}; // unit quaternion x, y, z, w (the solver builds it with the HDK)
  float pos[3] = {0.0f, 0.0f, 0.0f};
  float radius = 0.5f;
  float length = 1.0f;

  bool any() const {
    return hasSize || hasRot || hasPos || hasRadius || hasLength;
  }
};

// Rotate v by the unit quaternion q.
inline void quatRotate(const float q[4], const float v[3], float out[3]) {
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
inline void applyPrimOverrides(ShapeSpec &s, const PrimOverrides &o) {
  if (o.hasSize && s.type == ShapeType::Box && finite3(o.size))
    for (int i = 0; i < 3; ++i)
      s.halfExtents[i] = clampExtent(o.size[i]);

  if (o.hasRadius && std::isfinite(o.radius) &&
      (s.type == ShapeType::Sphere || s.type == ShapeType::Capsule))
    s.radius = clampExtent(o.radius);

  if (o.hasLength && std::isfinite(o.length) && s.type == ShapeType::Capsule)
    s.halfLength = std::max(0.5f * o.length, 0.0f);

  if (o.hasPos && finite3(o.pos))
    copy3(s.center, o.pos);

  if (o.hasRot && finite3(o.rot) && std::isfinite(o.rot[3]))
    copy4(s.rot, o.rot);
}

inline float shapeVolume(const ShapeSpec &s) {
  const float pi = 3.14159265358979f;
  const float r = s.radius;
  switch (s.type) {
  case ShapeType::Sphere:
    return (4.0f / 3.0f) * pi * r * r * r;
  case ShapeType::Capsule:
    return pi * r * r * (2.0f * s.halfLength) + (4.0f / 3.0f) * pi * r * r * r;
  default:
    return 8.0f * s.halfExtents[0] * s.halfExtents[1] * s.halfExtents[2];
  }
}

// Object id in the high 32 bits, the point's `id` (else its number) in the
// low 32.
inline uint64_t makeBodyKey(int objectId, int64_t pointId) {
  return (static_cast<uint64_t>(static_cast<uint32_t>(objectId)) << 32) |
         static_cast<uint64_t>(static_cast<uint32_t>(pointId));
}

// ---------------------------------------------------------------------------
// Packed primitives and hull point preparation
//
// Houdini matrices are row-vector (p' = p * M, translation in row 3);
// everything here uses that layout (m3[row*3+col], m4[row*4+col]). Quaternions
// are x, y, z, w.
// ---------------------------------------------------------------------------

// The upper-left 3x3 of a 4x4 (both row-major).
inline void upper3x3(const double m4[16], double m3[9]) {
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c)
      m3[r * 3 + c] = m4[r * 4 + c];
}

// A 3x3's row lengths (the per-row scale of M = S * R). False if a row is zero
// or non-finite. The plugin splits rotations with the HDK
// (UT_Matrix3D::extractScales); this Houdini-free version is only for scale
// keys.
inline bool matrixRowLengths(const double m[9], double out[3]) {
  out[0] = out[1] = out[2] = 1.0;
  double l[3];
  for (int r = 0; r < 3; ++r) {
    l[r] = std::sqrt(m[r * 3] * m[r * 3] + m[r * 3 + 1] * m[r * 3 + 1] +
                     m[r * 3 + 2] * m[r * 3 + 2]);
    if (!(l[r] > 1e-12) || !std::isfinite(l[r]))
      return false;
  }
  out[0] = l[0];
  out[1] = l[1];
  out[2] = l[2];
  return true;
}

// The scale packedPointsToBodyLocal bakes in: m4's row lengths, rounded to 5
// significant digits so instances differing by float noise share a hull. False
// if degenerate.
inline bool packedScale(const double m4[16], double S[3]) {
  double m3[9];
  upper3x3(m4, m3);
  if (!matrixRowLengths(m3, S))
    return false;
  // Round to 5 significant digits.
  for (int j = 0; j < 3; ++j) {
    const double v = S[j];
    if (!(std::fabs(v) > 0.0) || !std::isfinite(v))
      continue;
    const double step =
        std::pow(10.0, std::floor(std::log10(std::fabs(v))) - 4.0);
    S[j] = std::round(v / step) * step;
  }
  return true;
}

// Hashing for hull keys and signatures: 8 bytes per round (a multiply and a
// shift), the tail byte by byte. Keys only live for the process, so the exact
// function may change freely.
constexpr uint64_t kHashSeed = 1469598103934665603ull;
constexpr uint64_t kHashMul = 0xff51afd7ed558ccdull;

// Mixes one 64-bit value in (by value, so the same on any byte order).
inline uint64_t mixHash(uint64_t h, uint64_t v) {
  h = (h ^ v) * kHashMul;
  return h ^ (h >> 32);
}

inline uint64_t hashBytes(uint64_t h, const void *data, size_t byteCount) {
  const unsigned char *b = static_cast<const unsigned char *>(data);
  size_t i = 0;
  for (; i + 8 <= byteCount; i += 8) {
    uint64_t w;
    std::memcpy(&w, b + i, 8);
    h = mixHash(h, w);
  }
  uint64_t tail = 0;
  for (size_t k = 0; i < byteCount; ++i, ++k)
    tail |= static_cast<uint64_t>(b[i]) << (8 * k);
  return mixHash(h, tail ^ (static_cast<uint64_t>(byteCount) << 56));
}

// The raw bytes of `floatCount` floats, with the count mixed in.
inline uint64_t hashFloats(const float *xyz, size_t floatCount) {
  return hashBytes(kHashSeed ^ static_cast<uint64_t>(floatCount), xyz,
                   floatCount * sizeof(float));
}

// A double's bits; -0 and +0 hash alike.
inline uint64_t mixDouble(uint64_t h, double d) {
  d += 0.0;
  uint64_t u;
  std::memcpy(&u, &d, sizeof u);
  return mixHash(h, u);
}

// A packed hull's cache key: the source's point hash plus what differs per
// instance (pivot, scale). `kind`: 1 merged hull, 2 per piece. Same inputs as
// packedPointsToBodyLocal, so a hit means the same body-local points. Never 0.
inline uint64_t packedHullKey(uint64_t sourceHash, const double pivot[3],
                              const double S[3], uint32_t kind) {
  uint64_t h = mixHash(sourceHash, kind);
  for (int j = 0; j < 3; ++j)
    h = mixDouble(mixDouble(h, pivot[j]), S[j]);
  return h ? h : 1ull;
}

// Connected sets of primitives (sharing points, directly or not).
// primPoints[i]: points of primitive i. Returns each primitive's set, numbered
// in order of first appearance.
template <class PointId>
std::vector<int>
connectedSets(const std::vector<std::vector<PointId>> &primPoints) {
  const size_t n = primPoints.size();
  std::vector<size_t> parent(n);
  for (size_t i = 0; i < n; ++i)
    parent[i] = i;
  auto root = [&](size_t i) {
    while (parent[i] != i)
      i = parent[i] = parent[parent[i]];
    return i;
  };

  std::unordered_map<PointId, size_t>
      firstUser; // point -> first primitive that uses it
  for (size_t i = 0; i < n; ++i)
    for (const PointId &pt : primPoints[i]) {
      const auto ins = firstUser.emplace(pt, i);
      if (ins.second)
        continue;
      const size_t a = root(ins.first->second), b = root(i);
      if (a != b)
        parent[a > b ? a : b] = a > b ? b : a;
    }

  std::vector<int> label(n, -1), sets(n);
  int next = 0;
  for (size_t i = 0; i < n; ++i) {
    const size_t r = root(i);
    if (label[r] < 0)
      label[r] = next++;
    sets[i] = label[r];
  }
  return sets;
}

// A packed primitive's inner points in body-local space: (p - pivot) * S, with
// m4 the full transform (only its 3x3 is read, M = S * R) and pivot
// GEO_PrimPacked::getPivot (the inner point at the packed point's P, i.e. the
// body origin and centre of mass). S is rounded (packedScale) so rotated
// instances produce identical points. Degenerate m4: unchanged.
inline void packedPointsToBodyLocal(const float *xyz, int count,
                                    const double m4[16],
                                    const double pivotIn[3],
                                    std::vector<float> &out) {
  out.assign(xyz, xyz + static_cast<size_t>(count) * 3);
  if (count <= 0)
    return;

  double S[3];
  if (!packedScale(m4, S))
    return;

  for (int i = 0; i < count; ++i)
    for (int j = 0; j < 3; ++j)
      out[static_cast<size_t>(3 * i + j)] = static_cast<float>(
          (static_cast<double>(xyz[3 * i + j]) - pivotIn[j]) * S[j]);
}

// The body axes as a fitting frame (rows: the frame's axes in body space).
inline const float (&identityFrame())[3][3] {
  static const float kIdentity[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
  return kIdentity;
}

// A box / sphere / capsule fitted to the points along `axes` (three
// orthonormal, right-handed rows in body space), as Bullet's AutoFit does
// (measured against it, test autofit_bullet). Box: the points' bounds along
// the axes, turned with them. Sphere: centred on those bounds, just enclosing
// every point. Capsule: along the second axis (the one closest to the body's
// Y after canonicalFrame), centred on the bounds, radius just enclosing every
// point around that axis, ending where the bounds end along it (measured:
// Bullet's capsule on a standing 0.4 x 1.2 x 0.4 piece tops out at 0.6).
// Extents are clamped like makeShapeSpec. False for no points or a non-finite
// one.
inline bool fitShapeInFrame(ShapeType type, const float *xyz, int count,
                            const float axes[3][3], ShapeSpec &out) {
  if (count <= 0)
    return false;
  float lo[3], hi[3];
  for (int i = 0; i < count; ++i) {
    const float *p = xyz + 3 * i;
    if (!finite3(p))
      return false;
    for (int k = 0; k < 3; ++k) {
      const float d = p[0] * axes[k][0] + p[1] * axes[k][1] + p[2] * axes[k][2];
      if (i == 0 || d < lo[k])
        lo[k] = d;
      if (i == 0 || d > hi[k])
        hi[k] = d;
    }
  }

  float half[3], mid[3];
  for (int k = 0; k < 3; ++k) {
    mid[k] = 0.5f * (lo[k] + hi[k]);
    half[k] = 0.5f * clampExtent(hi[k] - lo[k]);
  }

  out = ShapeSpec();
  for (int j = 0; j < 3; ++j)
    out.center[j] =
        mid[0] * axes[0][j] + mid[1] * axes[1][j] + mid[2] * axes[2][j];
  const bool turned = !(axes[0][0] == 1.0f && axes[1][1] == 1.0f &&
                        axes[2][2] == 1.0f); // keeps an exact identity

  // Per point, relative to the centre: along the second axis, and the squared
  // distance from that axis (in doubles: r^2 - d^2 cancels at the rim).
  auto relative = [&](int i, double &along, double &dist2) {
    double v[3];
    for (int j = 0; j < 3; ++j)
      v[j] = static_cast<double>(xyz[3 * i + j]) - out.center[j];
    along = v[0] * axes[1][0] + v[1] * axes[1][1] + v[2] * axes[1][2];
    dist2 =
        std::max(0.0, v[0] * v[0] + v[1] * v[1] + v[2] * v[2] - along * along);
  };

  if (type == ShapeType::Sphere) {
    double r2 = 0.0;
    for (int i = 0; i < count; ++i) {
      double along, dist2;
      relative(i, along, dist2);
      r2 = std::max(r2, dist2 + along * along);
    }
    out.type = ShapeType::Sphere;
    out.radius = clampExtent(static_cast<float>(std::sqrt(r2)));
  } else if (type == ShapeType::Capsule) {
    double r2 = 0.0;
    for (int i = 0; i < count; ++i) {
      double along, dist2;
      relative(i, along, dist2);
      r2 = std::max(r2, dist2);
    }
    out.type = ShapeType::Capsule;
    out.radius = clampExtent(static_cast<float>(std::sqrt(r2)));
    // Bullet keeps the whole capsule within the bounds along the axis: the
    // caps end where the piece does (a sphere when the radius is larger).
    out.halfLength = std::max(half[1] - out.radius, 0.0f);
    if (turned)
      quatFromAxes(axes[0], axes[1], axes[2], out.rot);
  } else {
    out.type = ShapeType::Box;
    copy3(out.halfExtents, half);
    if (turned)
      quatFromAxes(axes[0], axes[1], axes[2], out.rot);
  }
  return true;
}

// A box / sphere / capsule fitted to the points' bounds along the body axes:
// the fallback when points cannot be hulled (fitShapeOriented in Box3DWorld.h
// is AutoFit, which also tries turned frames).
inline bool fitShapeToPoints(ShapeType type, const float *xyz, int count,
                             ShapeSpec &out) {
  return fitShapeInFrame(type, xyz, count, identityFrame(), out);
}

// Turns an orthonormal frame (rows) into the closest one to the body axes that
// spans the same box: each body axis in turn takes the remaining frame axis
// most parallel to it, pointing its way; the third is their cross product, so
// the result is right-handed. A frame already along the body axes becomes the
// exact identity.
inline void canonicalFrame(float axes[3][3]) {
  float in[3][3];
  std::memcpy(in, axes, sizeof in);
  bool used[3] = {false, false, false};
  for (int j = 0; j < 2; ++j) {
    int best = -1;
    for (int k = 0; k < 3; ++k)
      if (!used[k] &&
          (best < 0 || std::fabs(in[k][j]) > std::fabs(in[best][j])))
        best = k;
    used[best] = true;
    const float s = in[best][j] < 0.0f ? -1.0f : 1.0f;
    for (int c = 0; c < 3; ++c)
      axes[j][c] = s * in[best][c];
  }
  axes[2][0] = axes[0][1] * axes[1][2] - axes[0][2] * axes[1][1];
  axes[2][1] = axes[0][2] * axes[1][0] - axes[0][0] * axes[1][2];
  axes[2][2] = axes[0][0] * axes[1][1] - axes[0][1] * axes[1][0];
  // Snap float noise so an unturned frame is exactly the identity.
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c) {
      const float want = (r == c) ? 1.0f : 0.0f;
      if (std::fabs(axes[r][c] - want) < 1e-6f)
        axes[r][c] = want;
    }
}

} // namespace b3dop
