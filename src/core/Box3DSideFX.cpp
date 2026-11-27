// Box3DSideFX.cpp
#include "Box3DSideFX.h"
#include "Box3DMath.h"
#include "Box3DWorld.h"

#include <box3d/math_functions.h>

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

// Soft and Spring, once both are in Hz: by constraint_type. `apart`: the
// anchors have a direction at rest (SideFXContext::anchorDistance).
void mapSprings(SideFXJoint &j, ConstraintMode mode, float restLength,
                bool apart, float linHz, float linRatio, float angHz,
                float angRatio) {
  const bool hasRest = restLength > kSideFXMinRestLength;
  switch (mode) {
  case ConstraintMode::Position:
    if (hasRest) {
      // Bullet springs the anchors to a point the Rest Length from anchor A,
      // along a direction fixed in A: a spring in all three axes there. With
      // the anchors together the direction is the current one, every frame.
      j.type = JointType::Motor;
      j.settings = springMotor(linHz, linRatio, 0.0f, 0.0f);
      j.separation = restLength;
      j.offsetOnA = true;
      j.reaim = !apart;
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

// Quaternions here are x, y, z, w arrays; Box3D's b3Quat is (v = xyz, s = w).
b3Quat toB3(const float q[4]) { return b3Quat{{q[0], q[1], q[2]}, q[3]}; }

void fromB3(const b3Quat &q, float out[4]) {
  out[0] = q.v.x;
  out[1] = q.v.y;
  out[2] = q.v.z;
  out[3] = q.s;
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
  float d = dot3(x, z);
  for (int k = 0; k < 3; ++k)
    x[k] -= d * z[k];
  float lx = length3(x);
  if (!(lx > 1e-6f) || !std::isfinite(lx)) {
    // Up parallel to the twist axis (or missing): any perpendicular.
    const float ref[3] = {std::fabs(z[1]) < 0.9f ? 0.0f : 1.0f,
                          std::fabs(z[1]) < 0.9f ? 1.0f : 0.0f, 0.0f};
    d = dot3(ref, z);
    for (int k = 0; k < 3; ++k)
      x[k] = ref[k] - d * z[k];
    lx = length3(x);
  }
  for (float &c : x)
    c /= lx;
  const b3Vec3 bx{x[0], x[1], x[2]};
  const b3Vec3 bz{z[0], z[1], z[2]};

  // Columns x, y, z of the rotation matrix -> quaternion.
  const b3Matrix3 m{bx, b3Cross(bz, bx), bz};
  fromB3(b3MakeQuatFromMatrix(&m), q);
  sanitizeQuat(q);
}

void rotateVec(const float q[4], const float v[3], float out[3]) {
  const b3Vec3 r = b3RotateVector(toB3(q), b3Vec3{v[0], v[1], v[2]});
  out[0] = r.x;
  out[1] = r.y;
  out[2] = r.z;
}

} // namespace

SideFXJoint mapSideFX(SideFXType t, const SideFXSettings &s,
                      ConstraintMode mode, const SideFXContext &c) {
  SideFXJoint j;
  const bool collide = s.disableCollisions == 0;
  const bool apart = !(c.anchorDistance >= 0.0f &&
                       c.anchorDistance <= kSideFXCoincidentAnchors);
  switch (t) {
  case SideFXType::Glue: {
    j.type = JointType::Weld;
    j.settings = Box3DWorld::defaultJointSettings(JointType::Weld);
    j.settings.enableLinearSpring = false;
    j.settings.enableAngularSpring = false;
    // Bullet's glue breaks on the impulse of the hits it takes, not on a load:
    // the weld never breaks by force or torque, and the wrapper breaks it
    // (Box3DGlue.h).
    j.settings.forceThreshold = -1.0f;
    j.settings.torqueThreshold = -1.0f;
    j.holdSeparation = true;
    j.settings.collideConnected = false; // glued pieces never collide in Bullet
    return j;
  }
  case SideFXType::Hard:
    switch (mode) {
    case ConstraintMode::All:
      // Rigid. Like Bullet's (checked in Houdini 22.0: a piece held 2.06 away
      // ends up at the rest length, 1 by default), the anchors are kept the
      // rest length apart; with none (0, or below it) they are brought together
      // (`bullet_hard_rest_zero`: anchors 0.87 apart end on top of each other).
      j.type = JointType::Weld;
      j.settings = Box3DWorld::defaultJointSettings(JointType::Weld);
      j.settings.enableLinearSpring = false;
      j.settings.enableAngularSpring = false;
      if (s.restLength > kSideFXMinRestLength)
        j.separation = s.restLength;
      break;
    case ConstraintMode::Position:
      if (s.restLength > kSideFXMinRestLength) {
        // Bullet pins anchor B to a point the Rest Length from anchor A, along
        // a direction fixed in A: a ball joint there, not a distance. With the
        // anchors together the direction is the current one, every frame.
        j.type = JointType::Spherical;
        j.settings = Box3DWorld::defaultJointSettings(JointType::Spherical);
        j.separation = s.restLength;
        j.offsetOnA = true;
        j.reaim = !apart;
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
    mapSprings(j, mode, s.restLength, apart, s.stiffness, s.dampingRatio, angHz,
               angZ);
    break;
  }
  case SideFXType::Spring: {
    float linHz, linZ, angHz, angZ;
    springToHertz(s.strength, s.damping, effective(c.a, c.b, &SideFXBody::mass),
                  linHz, linZ);
    springToHertz(s.strength, s.damping,
                  effective(c.a, c.b, &SideFXBody::inertia), angHz, angZ);
    mapSprings(j, mode, s.restLength, apart, linHz, linZ, angHz, angZ);
    break;
  }
  case SideFXType::ConeTwist: {
    j.type = JointType::Spherical;
    j.settings = Box3DWorld::defaultJointSettings(JointType::Spherical);
    // An elliptical cone: the frame's x axis is the up axis, so Max Up
    // Rotation (the turn about it) limits the swing toward y and Max Out
    // Rotation the swing toward x. Box3D caps each at 90 degrees; 180 or more
    // is no limit (Houdini's default), and the cone is off when both are.
    const auto coneLimit = [](float degrees) {
      return std::isfinite(degrees) && degrees < 180.0f
                 ? std::clamp(degrees, 0.0f, 90.0f) * kDegToRad
                 : 90.0f * kDegToRad;
    };
    j.settings.enableConeLimit =
        (std::isfinite(s.maxUpRotation) && s.maxUpRotation < 180.0f) ||
        (std::isfinite(s.maxOutRotation) && s.maxOutRotation < 180.0f);
    j.settings.coneAngle =
        j.settings.enableConeLimit ? coneLimit(s.maxOutRotation) : 0.0f;
    j.settings.coneAngleY =
        j.settings.enableConeLimit ? coneLimit(s.maxUpRotation) : 0.0f;
    j.settings.ellipticalCone = j.settings.enableConeLimit &&
                                j.settings.coneAngle != j.settings.coneAngleY;
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
  const double dist = length3(d);
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
    rotateVec(anchorA.rot, x, u);
  }
  if (j.reaim && dist <= kSideFXAimMinDistance)
    return; // no direction yet: the frames stay on the anchors
  const float half = static_cast<float>(j.offsetOnA ? sep : 0.5 * sep);
  float qa[4], qb[4];
  fromB3(b3Conjugate(toB3(anchorA.rot)), qa);
  fromB3(b3Conjugate(toB3(anchorB.rot)), qb);
  const float wa[3] = {u[0] * half, u[1] * half, u[2] * half};
  const float wb[3] = {j.offsetOnA ? 0.0f : -wa[0], j.offsetOnA ? 0.0f : -wa[1],
                       j.offsetOnA ? 0.0f : -wa[2]};
  float la[3], lb[3];
  rotateVec(qa, wa, la);
  rotateVec(qb, wb, lb);
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
  rotateVec(anchor.rot, p, w);
  for (int k = 0; k < 3; ++k)
    out.pos[k] = anchor.pos[k] + static_cast<double>(w[k]);
  fromB3(b3MulQuat(toB3(anchor.rot), toB3(adj.rot)), out.rot);
  sanitizeQuat(out.rot);
  return out;
}

JointFrame removeFrameOffset(const JointFrame &joint, const JointFrame &adj) {
  JointFrame out;
  fromB3(b3MulQuat(toB3(joint.rot), b3Conjugate(toB3(adj.rot))), out.rot);
  sanitizeQuat(out.rot);
  const float p[3] = {static_cast<float>(adj.pos[0]),
                      static_cast<float>(adj.pos[1]),
                      static_cast<float>(adj.pos[2])};
  float w[3];
  rotateVec(out.rot, p, w);
  for (int k = 0; k < 3; ++k)
    out.pos[k] = joint.pos[k] - static_cast<double>(w[k]);
  return out;
}

} // namespace b3dop
