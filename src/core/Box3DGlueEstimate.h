// Box3DGlueEstimate.h
//
// Header-only, Houdini-free: the impulse estimate Bullet's Glue breaks on.
// Before each step RBD Bullet Solver glues a group into one rigid body, moves
// it to where the step ends and gives every contact point the impulse that
// would stop the approach at that point alone (SIM_BulletState::preBreakGlue
// and simResolveCollision in the HDK's SIM_SolverBullet):
//
//   impulse = (1 + e * v) * v / (dA + dB)
//
// v is the speed the two bodies approach at there (along the normal, as they
// move at the end of the step), e the two restitutions multiplied, and dX the
// body's impulse denominator at the point: 1 / mass + n . ((I^-1 (r x n)) x r)
// about its centre of mass (0 for a body that does not move). Each point is
// worked out as if it were the only one, so the sum over the points of a
// landing is several times the impulse the step then carries, and a bond breaks
// when it passes the Strength (core/Box3DGlue.h).
#pragma once

#include "core/Box3DCompound.h"

#include <cmath>

namespace b3dop {

// A rigid body as the estimate sees it.
struct GlueBody {
  bool moves = false; // dynamic: takes an impulse
  float mass = 0.0f;
  double com[3] = {0, 0, 0};
  double inertia[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0}; // world axes, about com
};

// A group's merged body (the velocities in it are not used).
inline GlueBody glueBodyOf(const MergedBody &mb) {
  GlueBody out;
  out.moves = mb.ok && mb.mass > 0.0f;
  out.mass = mb.mass;
  for (int i = 0; i < 3; ++i)
    out.com[i] = mb.com[i];
  for (int i = 0; i < 9; ++i)
    out.inertia[i] = mb.inertia[i];
  return out;
}

// btRigidBody::computeImpulseDenominator: how much one unit of impulse along
// `n` at `pos` changes the speed along `n` there. 0 for a body that does not
// move, and when its inertia cannot be inverted.
inline double glueDenominator(const GlueBody &b, const double pos[3],
                              const float n[3]) {
  if (!b.moves || !(b.mass > 0.0f))
    return 0.0;
  const double nn[3] = {n[0], n[1], n[2]};
  const double r[3] = {pos[0] - b.com[0], pos[1] - b.com[1], pos[2] - b.com[2]};
  double rxn[3], w[3], wxr[3];
  cross3(r, nn, rxn);
  if (!compound_detail::solve3(b.inertia, rxn, w))
    return 1.0 / static_cast<double>(b.mass);
  cross3(w, r, wxr);
  return 1.0 / static_cast<double>(b.mass) + dot3(nn, wxr);
}

// The estimate for one contact point at `pos` with normal `n` pointing from `a`
// into `b`, the two approaching at `approach` m/s along it (as they move at the
// end of the step, which is where Bullet reads them). 0 when they are not
// approaching or nothing there takes an impulse.
inline float glueContactEstimate(const GlueBody &a, const GlueBody &b,
                                 const double pos[3], const float n[3],
                                 float approach, float restitution) {
  if (!(approach > 0.0f))
    return 0.0f;
  const double d = glueDenominator(a, pos, n) + glueDenominator(b, pos, n);
  if (!(d > 0.0))
    return 0.0f;
  const double v = static_cast<double>(approach);
  const double impulse = (1.0 + static_cast<double>(restitution) * v) * v / d;
  return std::isfinite(impulse) ? static_cast<float>(impulse) : 0.0f;
}

} // namespace b3dop
