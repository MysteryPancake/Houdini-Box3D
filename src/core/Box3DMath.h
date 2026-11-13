// Box3DMath.h
//
// Houdini-free, header-only vector and quaternion helpers with no other
// dependencies: unit constants, finite checks, small fixed-size copies,
// quaternion handling. Quaternions are x, y, z, w; matrices are row-vector (p'
// = [p 1] * M, translation in row 3).
#pragma once

#include <cmath>

namespace b3dop {

// Non-finite check for a 3-vector.
inline bool finite3(const float v[3]) {
  return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
}

// dst[0..2] = src[0..2] / dst[0..3] = src[0..3]. The element types may differ
// (double -> float).
template <class D, class S> inline void copy3(D *dst, const S *src) {
  for (int i = 0; i < 3; ++i)
    dst[i] = static_cast<D>(src[i]);
}

template <class D, class S> inline void copy4(D *dst, const S *src) {
  for (int i = 0; i < 4; ++i)
    dst[i] = static_cast<D>(src[i]);
}

// ---------------------------------------------------------------------------
// How values change on their way into and out of the geometry. Shared by the
// read path and the write-back's prediction, so the solver can tell its own
// writes from other edits.
// ---------------------------------------------------------------------------

constexpr float kDegToRad = 0.017453292519943295f;
constexpr float kRadToDeg = 57.29577951308232f;

// Two rotations differ beyond float noise (q and -q are the same rotation):
// components compared after aligning signs. A dot-product test would only see
// rotations above ~0.16 degrees.
inline bool quatsDiffer(const float a[4], const float b[4], double eps = 1e-5) {
  double dot = 0.0;
  for (int i = 0; i < 4; ++i)
    dot += double(a[i]) * double(b[i]);
  const double sign = dot < 0.0 ? -1.0 : 1.0;
  for (int i = 0; i < 4; ++i)
    if (std::fabs(double(a[i]) - sign * double(b[i])) > eps)
      return true;
  return false;
}

// A point through a row-vector 4x4 (Houdini layout: p' = [p 1] * M, translation
// in row 3).
inline void xformPoint(const double m[16], const double p[3], double out[3]) {
  for (int c = 0; c < 3; ++c)
    out[c] = p[0] * m[c] + p[1] * m[4 + c] + p[2] * m[8 + c] + m[12 + c];
}

// Box3D asserts on non-unit quaternions; attributes may hold zeros or off-unit
// values.
inline void sanitizeQuat(float q[4]) {
  double len2 = 0.0;
  for (int i = 0; i < 4; ++i)
    len2 += double(q[i]) * double(q[i]);
  if (!(len2 > 1e-12) || !std::isfinite(len2)) {
    q[0] = q[1] = q[2] = 0.0f;
    q[3] = 1.0f;
    return;
  }
  const float inv = float(1.0 / std::sqrt(len2));
  for (int i = 0; i < 4; ++i)
    q[i] *= inv;
}

// P is stored as float: double -> float -> double. False if not finite.
inline bool storedPosition(const double in[3], double out[3]) {
  for (int i = 0; i < 3; ++i)
    out[i] = double(float(in[i]));
  return std::isfinite(out[0]) && std::isfinite(out[1]) &&
         std::isfinite(out[2]);
}

} // namespace b3dop
