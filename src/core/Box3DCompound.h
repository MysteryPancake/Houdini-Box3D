// Box3DCompound.h
//
// Header-only, Houdini-free: the maths of gluing pieces into one rigid body
// (a compound) and splitting it again.
//   * glueComponents: the groups of nodes joined by glue bonds (union-find).
//   * mergePieces: the one body that carries the pieces' momentum: its centre
//     of mass, mass, inertia, and the linear and angular velocity that keep the
//     pieces' total linear and angular momentum.
//   * pieceLoad: the impulse a piece takes when the body's motion changes.
//   * pieceVelocity: a piece's velocity on a rigid body, v + w x r. Used to
//     write the pieces back after a step, and to start the bodies a compound
//     splits into.
// A piece's position is its centre of mass and its inertia is about it, in its
// own axes.
#pragma once

#include "core/Box3DGlue.h"
#include "core/Box3DMath.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <unordered_map>
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
  // Relative to the matrix's size, so a tiny piece (small inertia, any units)
  // still gets a spin. The determinant scales with the cube of the entries.
  double biggest = 0.0;
  for (int i = 0; i < 9; ++i)
    biggest = std::max(biggest, std::fabs(m[i]));
  if (!(biggest > 0.0) ||
      !(std::fabs(det) > 1e-12 * biggest * biggest * biggest))
    return false;
  const double inv = 1.0 / det;
  out[0] = (c0 * v[0] + (m[2] * m[7] - m[1] * m[8]) * v[1] +
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
      quatRotate(q.rot, e, axes[k]);
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

// The impulse a piece of `mass` at `pos` takes from the rest of a rigid body
// whose motion went from `before` to `after`: mass * |change of the piece's
// velocity|. A body slowed at one point turns, so the pieces far from it change
// velocity most. A body at steady motion (weight held up by a contact) changes
// none, so nothing is taken.
inline float pieceLoad(const MergedBody &before, const MergedBody &after,
                       const double pos[3], float mass) {
  float lin0[3], ang0[3], lin1[3], ang1[3];
  pieceVelocity(before, pos, lin0, ang0);
  pieceVelocity(after, pos, lin1, ang1);
  const double d[3] = {static_cast<double>(lin1[0]) - lin0[0],
                       static_cast<double>(lin1[1]) - lin0[1],
                       static_cast<double>(lin1[2]) - lin0[2]};
  return mass *
         static_cast<float>(std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]));
}

// The box a body covers going through a compound: its own bounds (half size
// `half` about `com`) swept along its velocity `lin` until its centre leaves
// the compound's bounds [`lo`, `hi`], so every piece it will plow through is
// inside. False (and just its own bounds) when it is not heading into or
// through them: at rest, or with its centre already out of the bounds on an
// axis it is moving away along, from where it is only leaving them.
inline bool pathThroughBounds(const double com[3], const float lin[3],
                              double half, const double lo[3],
                              const double hi[3], double outLo[3],
                              double outHi[3]) {
  double t = std::numeric_limits<double>::infinity();
  for (int i = 0; i < 3; ++i) {
    const double v = lin[i];
    if (v > 1e-6)
      t = std::min(t, (hi[i] - com[i]) / v);
    else if (v < -1e-6)
      t = std::min(t, (lo[i] - com[i]) / v);
  }
  const bool through = std::isfinite(t) && t >= 0.0;
  if (!through)
    t = 0.0;
  for (int i = 0; i < 3; ++i) {
    const double d = static_cast<double>(lin[i]) * t;
    outLo[i] = com[i] - half + std::min(0.0, d);
    outHi[i] = com[i] + half + std::max(0.0, d);
  }
  return through;
}

// The pieces held out of their compounds while a fast body comes through
// (holdCompoundsForApproach), each with the time it may merge again.
// Houdini-free so its rules are tested in tests/core_compound.cpp; pieces are
// named by BodyHandle.
class GlueHolds {
public:
  bool empty() const { return myUntil.empty(); }
  size_t size() const { return myUntil.size(); } // (the tests count them)
  bool held(uint64_t piece) const { return myUntil.count(piece) != 0; }
  // Holds `piece` until `until` (never shortens a hold it has).
  void hold(uint64_t piece, double until) {
    double &t = myUntil[piece];
    t = std::max(t, until);
  }
  // Holds `start` and the held pieces reached from it through `neighbours`
  // (called as neighbours(piece, visit), visit(other) for each bonded piece),
  // until `until`: the structure a body is still coming through, not every held
  // piece in the scene.
  template <class Neighbours>
  void refresh(uint64_t start, double until, Neighbours &&neighbours) {
    std::vector<uint64_t> stack{start};
    while (!stack.empty()) {
      const uint64_t piece = stack.back();
      stack.pop_back();
      const auto it = myUntil.find(piece);
      if (it == myUntil.end() || it->second >= until)
        continue;
      it->second = until;
      neighbours(piece, [&stack](uint64_t other) { stack.push_back(other); });
    }
  }
  // Lets go of the pieces whose time is up at `now`. True if any were.
  bool expire(double now) {
    bool released = false;
    for (auto it = myUntil.begin(); it != myUntil.end();) {
      if (it->second > now) {
        ++it;
        continue;
      }
      it = myUntil.erase(it);
      released = true;
    }
    return released;
  }

private:
  std::unordered_map<uint64_t, double> myUntil;
};

// A frame given in a piece's own axes (`pos`, `rot`), in the axes of the
// compound body that carries the piece at `lp` / `lq` (the piece's place in
// it): what a weld to the compound's body needs in place of the weld to the
// piece.
inline void frameInCompound(const float lp[3], const float lq[4],
                            const double pos[3], const float rot[4],
                            double outPos[3], float outRot[4]) {
  double r[3];
  quatRotate(lq, pos, r);
  for (int i = 0; i < 3; ++i)
    outPos[i] = static_cast<double>(lp[i]) + r[i];
  quatMul(lq, rot, outRot);
  sanitizeQuat(outRot);
}

} // namespace b3dop
