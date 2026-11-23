// Shared helpers for the core tests that build bodies and joints in a
// Box3DWorld: a box body, a joint frame, a body's position, a few steps.
#pragma once

#include "core/Box3DWorld.h"

#include <cmath>

inline const float kQid[4] = {0, 0, 0, 1};
inline const float kDt = 1.0f / 60.0f;

// A box body of half size `half` at (x, y, z).
inline b3dop::BodyHandle
box(b3dop::Box3DWorld &w, double x, double y, double z,
    b3dop::BodySettings::Type type = b3dop::BodySettings::Type::Dynamic,
    float half = 0.25f) {
  b3dop::BodySettings b;
  b.type = type;
  const double p[3] = {x, y, z};
  const b3dop::BodyHandle h = w.createBody(b, p, kQid);
  b3dop::ShapeSpec s;
  s.type = b3dop::ShapeType::Box;
  s.halfExtents[0] = s.halfExtents[1] = s.halfExtents[2] = half;
  w.createShape(h, b, s);
  return h;
}

inline b3dop::JointFrame frame(double x, double y, double z,
                               const float q[4] = kQid) {
  b3dop::JointFrame f;
  f.pos[0] = x;
  f.pos[1] = y;
  f.pos[2] = z;
  for (int i = 0; i < 4; ++i)
    f.rot[i] = q[i];
  return f;
}

inline void pos(b3dop::Box3DWorld &w, b3dop::BodyHandle h, double out[3]) {
  float q[4];
  w.getTransformByHandle(h, out, q);
}

inline void steps(b3dop::Box3DWorld &w, int n) {
  for (int i = 0; i < n; ++i)
    w.step(kDt);
}

inline double dist(const double a[3], const double b[3]) {
  const double dx = a[0] - b[0], dy = a[1] - b[1], dz = a[2] - b[2];
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}
