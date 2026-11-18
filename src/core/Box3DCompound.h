// Box3DCompound.h
//
// Header-only, Houdini-free: the maths of gluing pieces into one rigid body
// (a compound) and splitting it again.
//   * glueComponents: the groups of nodes joined by glue bonds (union-find).
//   * mergePieces: the one body that carries the pieces' momentum: its centre
//     of mass, mass, inertia, and the linear and angular velocity that keep the
//     pieces' total linear and angular momentum.
//   * pieceVelocity: a piece's velocity on a rigid body, v + w x r. Used to
//     write the pieces back after a step, and to start the bodies a compound
//     splits into.
// A piece's position is its centre of mass and its inertia is about it, in its
// own axes.
#pragma once

#include "core/Box3DGlue.h"

#include <algorithm>
#include <cmath>
#include <vector>

namespace b3dop {

// The nodes (bodies) joined by bonds. `of[node]` is the component's number,
// counted in order of each component's first node, so the result does not
// depend on the bond order. A bond with an end out of range is ignored.
struct GlueComponents {
  std::vector<int> of;
  std::vector<int> size; // nodes per component
  int count = 0;
};

inline GlueComponents glueComponents(size_t nodeCount,
                                     const std::vector<GlueBond> &bonds) {
  std::vector<int> parent(nodeCount);
  for (size_t i = 0; i < nodeCount; ++i)
    parent[i] = static_cast<int>(i);
  auto find = [&](int x) {
    while (parent[static_cast<size_t>(x)] != x) {
      parent[static_cast<size_t>(x)] =
          parent[static_cast<size_t>(parent[static_cast<size_t>(x)])];
      x = parent[static_cast<size_t>(x)];
    }
    return x;
  };
  for (const GlueBond &b : bonds) {
    if (b.a < 0 || b.b < 0 || static_cast<size_t>(b.a) >= nodeCount ||
        static_cast<size_t>(b.b) >= nodeCount)
      continue;
    const int ra = find(b.a), rb = find(b.b);
    if (ra != rb)
      parent[static_cast<size_t>(std::max(ra, rb))] = std::min(ra, rb);
  }
  GlueComponents out;
  out.of.assign(nodeCount, -1);
  std::vector<int> number(nodeCount, -1);
  for (size_t i = 0; i < nodeCount; ++i) {
    const size_t root = static_cast<size_t>(find(static_cast<int>(i)));
    if (number[root] < 0) {
      number[root] = out.count++;
      out.size.push_back(0);
    }
    out.of[i] = number[root];
    ++out.size[static_cast<size_t>(number[root])];
  }
  return out;
}

struct RigidPiece {
  double pos[3] = {0, 0, 0}; // centre of mass
  float rot[4] = {0, 0, 0, 1};
  float mass = 0.0f;
  // The inertia tensor about the centre of mass in the piece's own axes: xx yy
  // zz, and the off-diagonal terms xy xz yz (zero for principal axes).
  float inertia[3] = {0, 0, 0};
  float inertiaOff[3] = {0, 0, 0};
  float lin[3] = {0, 0, 0};
  float ang[3] = {0, 0, 0};
};

// The body the pieces make together.
struct MergedBody {
  bool ok = false; // false: no mass, or something was not finite
  double com[3] = {0, 0, 0};
  float mass = 0.0f;
  double inertia[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0}; // world axes, about com
  float lin[3] = {0, 0, 0};                        // of the centre of mass
  float ang[3] = {0, 0, 0};
};

namespace compound_detail {

// q (x, y, z, w) rotating v.
inline void rotate(const float q[4], const double v[3], double out[3]) {
  const double x = q[0], y = q[1], z = q[2], w = q[3];
  const double tx = 2.0 * (y * v[2] - z * v[1]);
  const double ty = 2.0 * (z * v[0] - x * v[2]);
  const double tz = 2.0 * (x * v[1] - y * v[0]);
  out[0] = v[0] + w * tx + (y * tz - z * ty);
  out[1] = v[1] + w * ty + (z * tx - x * tz);
  out[2] = v[2] + w * tz + (x * ty - y * tx);
}

inline void cross(const double a[3], const double b[3], double out[3]) {
  out[0] = a[1] * b[2] - a[2] * b[1];
  out[1] = a[2] * b[0] - a[0] * b[2];
  out[2] = a[0] * b[1] - a[1] * b[0];
}

// out = m^-1 * v for a row-major 3x3; false if it is singular.
inline bool solve3(const double m[9], const double v[3], double out[3]) {
  const double c0 = m[4] * m[8] - m[5] * m[7];
  const double c1 = m[5] * m[6] - m[3] * m[8];
  const double c2 = m[3] * m[7] - m[4] * m[6];
  const double det = m[0] * c0 + m[1] * c1 + m[2] * c2;
  if (!(std::fabs(det) > 1e-30))
    return false;
  const double inv = 1.0 / det;
  out[0] =
      ((m[4] * m[8] - m[5] * m[7]) * v[0] + (m[2] * m[7] - m[1] * m[8]) * v[1] +
       (m[1] * m[5] - m[2] * m[4]) * v[2]) *
      inv;
  out[1] = (c1 * v[0] + (m[0] * m[8] - m[2] * m[6]) * v[1] +
            (m[2] * m[3] - m[0] * m[5]) * v[2]) *
           inv;
  out[2] = (c2 * v[0] + (m[1] * m[6] - m[0] * m[7]) * v[1] +
            (m[0] * m[4] - m[1] * m[3]) * v[2]) *
           inv;
  return true;
}

} // namespace compound_detail

// Merge `pieces` into one body. The centre of mass, mass and inertia follow
// from the pieces (parallel axis theorem); the velocity is the one that keeps
// their total linear momentum and their angular momentum about the centre of
// mass. A body whose pieces already move together keeps that velocity.
inline MergedBody mergePieces(const std::vector<RigidPiece> &pieces) {
  using namespace compound_detail;
  MergedBody out;
  double mass = 0.0, c[3] = {0, 0, 0}, p[3] = {0, 0, 0};
  for (const RigidPiece &q : pieces) {
    if (!(q.mass > 0.0f) || !std::isfinite(q.mass))
      return out;
    mass += q.mass;
    for (int i = 0; i < 3; ++i) {
      c[i] += static_cast<double>(q.mass) * q.pos[i];
      p[i] += static_cast<double>(q.mass) * q.lin[i];
    }
  }
  if (!(mass > 0.0))
    return out;
  for (int i = 0; i < 3; ++i) {
    c[i] /= mass;
    out.com[i] = c[i];
    out.lin[i] = static_cast<float>(p[i] / mass);
  }
  out.mass = static_cast<float>(mass);

  double inertia[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
  double angular[3] = {0, 0, 0};
  for (const RigidPiece &q : pieces) {
    // The piece's inertia in world axes: R M R^T, M the tensor in its own axes
    // and the columns of R its axes.
    const double local[9] = {q.inertia[0],    q.inertiaOff[0], q.inertiaOff[1],
                             q.inertiaOff[0], q.inertia[1],    q.inertiaOff[2],
                             q.inertiaOff[1], q.inertiaOff[2], q.inertia[2]};
    double axes[3][3]; // axes[k] = the piece's k-th axis in world space
    for (int k = 0; k < 3; ++k) {
      const double e[3] = {k == 0 ? 1.0 : 0.0, k == 1 ? 1.0 : 0.0,
                           k == 2 ? 1.0 : 0.0};
      rotate(q.rot, e, axes[k]);
    }
    double iw[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    for (int k = 0; k < 3; ++k)
      for (int l = 0; l < 3; ++l)
        for (int r = 0; r < 3; ++r)
          for (int s = 0; s < 3; ++s)
            iw[3 * r + s] += local[3 * k + l] * axes[k][r] * axes[l][s];
    const double r[3] = {q.pos[0] - c[0], q.pos[1] - c[1], q.pos[2] - c[2]};
    const double rr = r[0] * r[0] + r[1] * r[1] + r[2] * r[2];
    for (int a = 0; a < 3; ++a)
      for (int b = 0; b < 3; ++b)
        inertia[3 * a + b] +=
            iw[3 * a + b] + q.mass * ((a == b ? rr : 0.0) - r[a] * r[b]);
    const double w[3] = {q.ang[0], q.ang[1], q.ang[2]};
    const double v[3] = {q.lin[0], q.lin[1], q.lin[2]};
    double rxv[3];
    cross(r, v, rxv);
    for (int a = 0; a < 3; ++a)
      angular[a] += iw[3 * a] * w[0] + iw[3 * a + 1] * w[1] +
                    iw[3 * a + 2] * w[2] + q.mass * rxv[a];
  }
  for (int i = 0; i < 9; ++i)
    out.inertia[i] = inertia[i];
  double omega[3] = {0, 0, 0};
  solve3(inertia, angular, omega); // singular: the body does not spin
  for (int i = 0; i < 3; ++i)
    out.ang[i] = static_cast<float>(omega[i]);
  out.ok = true;
  for (int i = 0; i < 3; ++i)
    out.ok = out.ok && std::isfinite(out.lin[i]) && std::isfinite(out.ang[i]);
  return out;
}

// Quaternions (x, y, z, w): out = a * b (b first, then a), the inverse, and a
// vector turned by one.
inline void quatMul(const float a[4], const float b[4], float out[4]) {
  const float x = a[3] * b[0] + b[3] * a[0] + a[1] * b[2] - a[2] * b[1];
  const float y = a[3] * b[1] + b[3] * a[1] + a[2] * b[0] - a[0] * b[2];
  const float z = a[3] * b[2] + b[3] * a[2] + a[0] * b[1] - a[1] * b[0];
  const float w = a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2];
  out[0] = x;
  out[1] = y;
  out[2] = z;
  out[3] = w;
}
inline void quatConj(const float q[4], float out[4]) {
  out[0] = -q[0];
  out[1] = -q[1];
  out[2] = -q[2];
  out[3] = q[3];
}
inline void quatRotate(const float q[4], const double v[3], double out[3]) {
  compound_detail::rotate(q, v, out);
}

// The velocity of a point at `pos` that rides on `body`: v + w x (pos - com).
inline void pieceVelocity(const MergedBody &body, const double pos[3],
                          float lin[3], float ang[3]) {
  using namespace compound_detail;
  const double r[3] = {pos[0] - body.com[0], pos[1] - body.com[1],
                       pos[2] - body.com[2]};
  const double w[3] = {body.ang[0], body.ang[1], body.ang[2]};
  double wxr[3];
  cross(w, r, wxr);
  for (int i = 0; i < 3; ++i) {
    lin[i] = static_cast<float>(static_cast<double>(body.lin[i]) + wxr[i]);
    ang[i] = body.ang[i];
  }
}

} // namespace b3dop
