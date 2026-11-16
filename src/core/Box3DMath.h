// Box3DMath.h
//
// Houdini-free, header-only vector and quaternion helpers with no other
// dependencies: unit constants, finite checks, small fixed-size copies,
// quaternion handling. Quaternions are x, y, z, w; matrices are row-vector (p'
// = [p 1] * M, translation in row 3).
#pragma once

#include <cmath>

namespace b3dop {

// Every component of a 3-vector (float or double) is finite.
template <class T> inline bool finite3(const T *v) {
  return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
}

// Finite and >= 0, else `fallback` (a negative value is replaced, not clamped).
inline float nonNegOr(float v, float fallback = 0.0f) {
  return (std::isfinite(v) && v >= 0.0f) ? v : fallback;
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
    dot += static_cast<double>(a[i]) * static_cast<double>(b[i]);
  const double sign = dot < 0.0 ? -1.0 : 1.0;
  for (int i = 0; i < 4; ++i)
    if (std::fabs(static_cast<double>(a[i]) -
                  sign * static_cast<double>(b[i])) > eps)
      return true;
  return false;
}

// The inverse inertia of a solid box of `mass` whose full side lengths are
// `extents`, about its centre, as xx yy zz xy xz yz (the layout of
// BodyProps::inverseInertia). False, `out` untouched, if the mass or a side is
// not positive and finite.
inline bool boundsInverseInertia(const float extents[3], float mass,
                                 float out[6]) {
  if (!(mass > 0.0f) || !std::isfinite(mass))
    return false;
  double sq[3];
  for (int i = 0; i < 3; ++i) {
    if (!(extents[i] > 0.0f) || !std::isfinite(extents[i]))
      return false;
    sq[i] = static_cast<double>(extents[i]) * extents[i];
  }
  const double m = mass / 12.0;
  const double inertia[3] = {m * (sq[1] + sq[2]), m * (sq[0] + sq[2]),
                             m * (sq[0] + sq[1])};
  float inv[3];
  for (int i = 0; i < 3; ++i) {
    inv[i] = static_cast<float>(1.0 / inertia[i]);
    if (!std::isfinite(inv[i]))
      return false;
  }
  out[0] = inv[0];
  out[1] = inv[1];
  out[2] = inv[2];
  out[3] = out[4] = out[5] = 0.0f;
  return true;
}

// Inverse of a symmetric 3x3 matrix stored as xx, yy, zz, xy, xz, yz (the
// layout of BodyProps::inverseInertia). False, `out` untouched, if the matrix
// is not finite or not positive definite (an inertia tensor and its inverse
// always are).
inline bool invertSymmetric3(const float in[6], float out[6]) {
  for (int i = 0; i < 6; ++i)
    if (!std::isfinite(in[i]))
      return false;
  const double a = in[0], b = in[1], c = in[2];
  const double d = in[3], e = in[4], f = in[5];
  // Leading principal minors, all positive for a positive definite matrix.
  const double m2 = a * b - d * d;
  const double det =
      a * (b * c - f * f) - d * (d * c - f * e) + e * (d * f - b * e);
  if (!(a > 0.0) || !(m2 > 0.0) || !(det > 0.0) || !std::isfinite(det))
    return false;
  const double inv[6] = {(b * c - f * f) / det, (a * c - e * e) / det,
                         (a * b - d * d) / det, (e * f - d * c) / det,
                         (d * f - b * e) / det, (d * e - a * f) / det};
  for (int i = 0; i < 6; ++i)
    if (!std::isfinite(inv[i]))
      return false;
  for (int i = 0; i < 6; ++i)
    out[i] = static_cast<float>(inv[i]);
  return true;
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
    len2 += static_cast<double>(q[i]) * static_cast<double>(q[i]);
  if (!(len2 > 1e-12) || !std::isfinite(len2)) {
    q[0] = q[1] = q[2] = 0.0f;
    q[3] = 1.0f;
    return;
  }
  const float inv = static_cast<float>(1.0 / std::sqrt(len2));
  for (int i = 0; i < 4; ++i)
    q[i] *= inv;
}

// The unit quaternion of the rotation whose matrix has columns x, y, z (an
// orthonormal right-handed frame: it turns the unit axes into x, y, z).
inline void quatFromAxes(const float x[3], const float y[3], const float z[3],
                         float q[4]) {
  // m[r][c]: column c is the image of unit axis c.
  const double m00 = x[0], m10 = x[1], m20 = x[2];
  const double m01 = y[0], m11 = y[1], m21 = y[2];
  const double m02 = z[0], m12 = z[1], m22 = z[2];
  const double trace = m00 + m11 + m22;
  double qx, qy, qz, qw;
  if (trace > 0.0) {
    const double s = 0.5 / std::sqrt(trace + 1.0);
    qw = 0.25 / s;
    qx = (m21 - m12) * s;
    qy = (m02 - m20) * s;
    qz = (m10 - m01) * s;
  } else if (m00 > m11 && m00 > m22) {
    const double s = 2.0 * std::sqrt(1.0 + m00 - m11 - m22);
    qw = (m21 - m12) / s;
    qx = 0.25 * s;
    qy = (m01 + m10) / s;
    qz = (m02 + m20) / s;
  } else if (m11 > m22) {
    const double s = 2.0 * std::sqrt(1.0 + m11 - m00 - m22);
    qw = (m02 - m20) / s;
    qx = (m01 + m10) / s;
    qy = 0.25 * s;
    qz = (m12 + m21) / s;
  } else {
    const double s = 2.0 * std::sqrt(1.0 + m22 - m00 - m11);
    qw = (m10 - m01) / s;
    qx = (m02 + m20) / s;
    qy = (m12 + m21) / s;
    qz = 0.25 * s;
  }
  q[0] = static_cast<float>(qx);
  q[1] = static_cast<float>(qy);
  q[2] = static_cast<float>(qz);
  q[3] = static_cast<float>(qw);
  sanitizeQuat(q);
}

// P is stored as float: double -> float -> double. False if not finite.
inline bool storedPosition(const double in[3], double out[3]) {
  for (int i = 0; i < 3; ++i)
    out[i] = static_cast<double>(static_cast<float>(in[i]));
  return std::isfinite(out[0]) && std::isfinite(out[1]) &&
         std::isfinite(out[2]);
}

} // namespace b3dop
