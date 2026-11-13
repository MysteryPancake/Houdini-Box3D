// Box3DSideFX.cpp
#include "Box3DSideFX.h"
#include "Box3DWorld.h"

#include <algorithm>
#include <cfloat>
#include <cmath>

namespace b3dop {

namespace {

constexpr float kTwoPi = 6.283185307179586f;

// "Rigid" for the motor joint's springs, which have no rigid mode: Box3D's own
// joint stiffness.
constexpr float kRigidHertz = 60.0f;
constexpr float kRigidDampingRatio = 2.0f;

// Mass seen by a spring between two bodies: both dynamic, the reduced mass;
// one, its own.
float effective(const SideFXBody &a, const SideFXBody &b,
                float SideFXBody::*m) {
  const float ma = a.dynamic ? a.*m : 0.0f, mb = b.dynamic ? b.*m : 0.0f;
  if (ma > 0.0f && mb > 0.0f)
    return ma * mb / (ma + mb);
  return std::max(ma, mb);
}

// Hooke's constant k and absolute damping c on mass m -> Box3D's Hz and damping
// ratio.
void springToHertz(float k, float c, float m, float &hertz, float &ratio) {
  hertz = ratio = 0.0f;
  if (!(k > 0.0f) || !(m > 0.0f) || !std::isfinite(k) || !std::isfinite(m))
    return;
  hertz = std::sqrt(k / m) / kTwoPi;
  ratio = std::isfinite(c) && c > 0.0f ? c / (2.0f * std::sqrt(k * m)) : 0.0f;
}

// A motor joint that only pulls with springs (no velocity drive). Stiffness 0
// turns a part off.
JointSettings springMotor(float linHz, float linRatio, float angHz,
                          float angRatio) {
  JointSettings s = Box3DWorld::defaultJointSettings(JointType::Motor);
  for (int k = 0; k < 3; ++k)
    s.linearVelocity[k] = s.angularVelocity[k] = 0.0f;
  s.maxVelocityForce = 0.0f;
  s.maxVelocityTorque = 0.0f;
  s.linearStiffness = std::max(0.0f, linHz);
  s.linearDampingRatio = std::max(0.0f, linRatio);
  s.maxSpringForce = s.linearStiffness > 0.0f ? FLT_MAX : 0.0f;
  s.angularStiffness = std::max(0.0f, angHz);
  s.angularDampingRatio = std::max(0.0f, angRatio);
  s.maxSpringTorque = s.angularStiffness > 0.0f ? FLT_MAX : 0.0f;
  return s;
}

// Soft and Spring, once both are in Hz: by constraint_type.
void mapSprings(SideFXJoint &j, ConstraintMode mode, float restLength,
                float linHz, float linRatio, float angHz, float angRatio) {
  const bool hasRest = restLength > kSideFXMinRestLength;
  switch (mode) {
  case ConstraintMode::Position:
    if (hasRest) {
      j.type = JointType::Distance;
      j.settings = Box3DWorld::defaultJointSettings(JointType::Distance);
      j.settings.restLength = restLength;
      j.settings.enableSpring = true;
      j.settings.stiffness = std::max(0.0f, linHz);
      j.settings.dampingRatio = std::max(0.0f, linRatio);
      j.settings.enableLimit = false;
    } else {
      j.type = JointType::Motor;
      j.settings = springMotor(linHz, linRatio, 0.0f, 0.0f);
    }
    break;
  case ConstraintMode::Rotation:
    j.type = JointType::Motor;
    j.settings = springMotor(0.0f, 0.0f, angHz, angRatio);
    break;
  case ConstraintMode::All:
    j.type = JointType::Motor;
    j.settings = springMotor(linHz, linRatio, angHz, angRatio);
    if (hasRest)
      j.separation = restLength;
    break;
  }
}

float length3(const float v[3]) {
  return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

// Rotation (x, y, z, w) whose z axis is `twist` and x axis is `up` (made
// perpendicular): Bullet's cone twist turns about x, Box3D's spherical joint
// about z.
void twistFrame(const float twistIn[3], const float upIn[3], float q[4]) {
  float z[3] = {twistIn[0], twistIn[1], twistIn[2]};
  float lz = length3(z);
  if (!(lz > 1e-6f) || !std::isfinite(lz)) {
    z[0] = 1.0f;
    z[1] = 0.0f;
    z[2] = 0.0f;
    lz = 1.0f;
  }
  for (float &c : z)
    c /= lz;
  float x[3] = {upIn[0], upIn[1], upIn[2]};
  float d = x[0] * z[0] + x[1] * z[1] + x[2] * z[2];
  for (int k = 0; k < 3; ++k)
    x[k] -= d * z[k];
  float lx = length3(x);
  if (!(lx > 1e-6f) || !std::isfinite(lx)) {
    // Up parallel to the twist axis (or missing): any perpendicular.
    const float ref[3] = {std::fabs(z[1]) < 0.9f ? 0.0f : 1.0f,
                          std::fabs(z[1]) < 0.9f ? 1.0f : 0.0f, 0.0f};
    d = ref[0] * z[0] + ref[1] * z[1] + ref[2] * z[2];
    for (int k = 0; k < 3; ++k)
      x[k] = ref[k] - d * z[k];
    lx = length3(x);
  }
  for (float &c : x)
    c /= lx;
  const float y[3] = {z[1] * x[2] - z[2] * x[1], z[2] * x[0] - z[0] * x[2],
                      z[0] * x[1] - z[1] * x[0]};

  // Columns x, y, z of the rotation matrix -> quaternion.
  const float m00 = x[0], m01 = y[0], m02 = z[0];
  const float m10 = x[1], m11 = y[1], m12 = z[1];
  const float m20 = x[2], m21 = y[2], m22 = z[2];
  const float tr = m00 + m11 + m22;
  if (tr > 0.0f) {
    const float s = std::sqrt(tr + 1.0f) * 2.0f;
    q[3] = 0.25f * s;
    q[0] = (m21 - m12) / s;
    q[1] = (m02 - m20) / s;
    q[2] = (m10 - m01) / s;
  } else if (m00 > m11 && m00 > m22) {
    const float s = std::sqrt(1.0f + m00 - m11 - m22) * 2.0f;
    q[3] = (m21 - m12) / s;
    q[0] = 0.25f * s;
    q[1] = (m01 + m10) / s;
    q[2] = (m02 + m20) / s;
  } else if (m11 > m22) {
    const float s = std::sqrt(1.0f + m11 - m00 - m22) * 2.0f;
    q[3] = (m02 - m20) / s;
    q[0] = (m01 + m10) / s;
    q[1] = 0.25f * s;
    q[2] = (m12 + m21) / s;
  } else {
    const float s = std::sqrt(1.0f + m22 - m00 - m11) * 2.0f;
    q[3] = (m10 - m01) / s;
    q[0] = (m02 + m20) / s;
    q[1] = (m12 + m21) / s;
    q[2] = 0.25f * s;
  }
  sanitizeQuat(q);
}

void quatMul(const float a[4], const float b[4], float out[4]) {
  const float x = a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1];
  const float y = a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0];
  const float z = a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3];
  const float w = a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2];
  out[0] = x;
  out[1] = y;
  out[2] = z;
  out[3] = w;
}

void quatConj(const float q[4], float out[4]) {
  out[0] = -q[0];
  out[1] = -q[1];
  out[2] = -q[2];
  out[3] = q[3];
}

} // namespace

SideFXJoint mapSideFX(SideFXType t, const SideFXSettings &s,
                      ConstraintMode mode, const SideFXContext &c) {
  SideFXJoint j;
  const bool collide = s.disableCollisions == 0;
  switch (t) {
  case SideFXType::Glue: {
    j.type = JointType::Weld;
    j.settings = Box3DWorld::defaultJointSettings(JointType::Weld);
    j.settings.enableLinearSpring = false;
    j.settings.enableAngularSpring = false;
    // Bullet's glue is an impulse; Box3D reports a force, so the impulse over
    // one step.
    const float dt = c.timeStep > 1e-9f ? c.timeStep : 1e-9f;
    j.settings.forceThreshold =
        (s.strength >= 0.0f && std::isfinite(s.strength)) ? s.strength / dt
                                                          : -1.0f;
    j.holdSeparation = true;
    j.settings.collideConnected = false; // glued pieces never collide in Bullet
    return j;
  }
  case SideFXType::Hard:
    switch (mode) {
    case ConstraintMode::All:
      // Rigid. Like Bullet's (checked in Houdini 22.0: a piece held 2.06 away
      // ends up at the rest length, 1 by default), the anchors are kept the
      // rest length apart; without one, where they are.
      j.type = JointType::Weld;
      j.settings = Box3DWorld::defaultJointSettings(JointType::Weld);
      j.settings.enableLinearSpring = false;
      j.settings.enableAngularSpring = false;
      if (s.restLength > kSideFXMinRestLength)
        j.separation = s.restLength;
      else
        j.holdSeparation = true;
      break;
    case ConstraintMode::Position:
      if (s.restLength > kSideFXMinRestLength) {
        j.type = JointType::Distance;
        j.settings = Box3DWorld::defaultJointSettings(JointType::Distance);
        j.settings.restLength = s.restLength;
        j.settings.enableSpring = false; // rigid at the rest length
        j.settings.enableLimit = false;
      } else {
        j.type = JointType::Spherical;
        j.settings = Box3DWorld::defaultJointSettings(JointType::Spherical);
      }
      break;
    case ConstraintMode::Rotation:
      j.type = JointType::Motor;
      j.settings = springMotor(0.0f, 0.0f, kRigidHertz, kRigidDampingRatio);
      break;
    }
    break;
  case SideFXType::Soft: {
    const bool own = s.enableAngular != 0;
    const float angHz = own ? s.angularStiffness : s.stiffness;
    const float angZ = own ? s.angularDampingRatio : s.dampingRatio;
    mapSprings(j, mode, s.restLength, s.stiffness, s.dampingRatio, angHz, angZ);
    break;
  }
  case SideFXType::Spring: {
    float linHz, linZ, angHz, angZ;
    springToHertz(s.strength, s.damping, effective(c.a, c.b, &SideFXBody::mass),
                  linHz, linZ);
    springToHertz(s.strength, s.damping,
                  effective(c.a, c.b, &SideFXBody::inertia), angHz, angZ);
    mapSprings(j, mode, s.restLength, linHz, linZ, angHz, angZ);
    break;
  }
  case SideFXType::ConeTwist: {
    j.type = JointType::Spherical;
    j.settings = Box3DWorld::defaultJointSettings(JointType::Spherical);
    // Box3D's cone is round: the smaller of the two limits, so neither is
    // exceeded. Box3D caps it at 90 degrees; 180 or more is no limit (Houdini's
    // default).
    const float cone = std::min(s.maxUpRotation, s.maxOutRotation);
    j.settings.enableConeLimit = std::isfinite(cone) && cone < 180.0f;
    j.settings.coneAngle =
        j.settings.enableConeLimit ? std::max(0.0f, cone) * kDegToRad : 0.0f;
    const float twist = s.maxTwist;
    j.settings.enableTwistLimit = std::isfinite(twist) && twist < 180.0f;
    const float tw =
        j.settings.enableTwistLimit ? std::max(0.0f, twist) * kDegToRad : 0.0f;
    j.settings.lowerTwistAngle = -tw;
    j.settings.upperTwistAngle = tw;
    twistFrame(s.constrainedTwistAxis, s.constrainedUpAxis, j.rotA);
    twistFrame(s.goalTwistAxis, s.goalUpAxis, j.rotB);
    break;
  }
  }
  j.settings.collideConnected = collide;
  return j;
}

void sideFXFrameOffsets(const SideFXJoint &j, const JointFrame &anchorA,
                        const JointFrame &anchorB, JointFrame &adjA,
                        JointFrame &adjB) {
  adjA = JointFrame();
  adjB = JointFrame();
  for (int k = 0; k < 4; ++k) {
    adjA.rot[k] = j.rotA[k];
    adjB.rot[k] = j.rotB[k];
  }

  const double d[3] = {anchorB.pos[0] - anchorA.pos[0],
                       anchorB.pos[1] - anchorA.pos[1],
                       anchorB.pos[2] - anchorA.pos[2]};
  const double dist = std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
  const double sep =
      j.holdSeparation ? dist : static_cast<double>(j.separation);
  if (!(sep > 0.0) || !std::isfinite(sep))
    return;

  // Along the anchors' line; anchors on top of each other: anchor A's x axis.
  float u[3];
  if (dist > 1e-9)
    for (int k = 0; k < 3; ++k)
      u[k] = static_cast<float>(d[k] / dist);
  else {
    const float x[3] = {1, 0, 0};
    quatRotate(anchorA.rot, x, u);
  }
  const float half = static_cast<float>(0.5 * sep);
  float qa[4], qb[4];
  quatConj(anchorA.rot, qa);
  quatConj(anchorB.rot, qb);
  const float wa[3] = {u[0] * half, u[1] * half, u[2] * half};
  const float wb[3] = {-wa[0], -wa[1], -wa[2]};
  float la[3], lb[3];
  quatRotate(qa, wa, la);
  quatRotate(qb, wb, lb);
  for (int k = 0; k < 3; ++k) {
    adjA.pos[k] = la[k];
    adjB.pos[k] = lb[k];
  }
}

JointFrame applyFrameOffset(const JointFrame &anchor, const JointFrame &adj) {
  JointFrame out;
  const float p[3] = {static_cast<float>(adj.pos[0]),
                      static_cast<float>(adj.pos[1]),
                      static_cast<float>(adj.pos[2])};
  float w[3];
  quatRotate(anchor.rot, p, w);
  for (int k = 0; k < 3; ++k)
    out.pos[k] = anchor.pos[k] + static_cast<double>(w[k]);
  quatMul(anchor.rot, adj.rot, out.rot);
  sanitizeQuat(out.rot);
  return out;
}

JointFrame removeFrameOffset(const JointFrame &joint, const JointFrame &adj) {
  JointFrame out;
  float inv[4];
  quatConj(adj.rot, inv);
  quatMul(joint.rot, inv, out.rot);
  sanitizeQuat(out.rot);
  const float p[3] = {static_cast<float>(adj.pos[0]),
                      static_cast<float>(adj.pos[1]),
                      static_cast<float>(adj.pos[2])};
  float w[3];
  quatRotate(out.rot, p, w);
  for (int k = 0; k < 3; ++k)
    out.pos[k] = joint.pos[k] - static_cast<double>(w[k]);
  return out;
}

} // namespace b3dop
