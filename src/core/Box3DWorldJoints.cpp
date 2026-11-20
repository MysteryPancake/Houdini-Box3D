// Box3DWorldJoints.cpp
//
// Box3DWorld's joints: creating, updating, reading back and breaking them.
#include "Box3DDebug.h"
#include "Box3DWorldInternal.h"

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <cstdint>
#include <vector>

namespace b3dop {

// ---------------------------------------------------------------------------
// Joints
// ---------------------------------------------------------------------------

namespace {

// A cap: < 0 (or non-finite) is unlimited.
float cap(float v) { return (std::isfinite(v) && v >= 0.0f) ? v : FLT_MAX; }

float finiteOr(float v) { return std::isfinite(v) ? v : 0.0f; }

// A distance joint's spring force limits and maximum length: a negative setting
// means unlimited.
float distanceLowerSpringForce(const JointSettings &s) {
  return s.maxTensionForce >= 0.0f ? -s.maxTensionForce : -FLT_MAX;
}

float distanceUpperSpringForce(const JointSettings &s) {
  return s.maxCompressionForce >= 0.0f ? s.maxCompressionForce : FLT_MAX;
}

float distanceMaxLength(const JointSettings &s) {
  return s.maxLength >= 0.0f ? s.maxLength : B3_HUGE;
}

// A local frame carried by a body at `bodyXf`, as a world-space JointFrame.
void frameToWorld(const b3WorldTransform &bodyXf, const b3Transform &local,
                  JointFrame &out) {
  const b3Vec3 d = b3RotateVector(bodyXf.q, local.p);
  out.pos[0] = static_cast<double>(bodyXf.p.x) + static_cast<double>(d.x);
  out.pos[1] = static_cast<double>(bodyXf.p.y) + static_cast<double>(d.y);
  out.pos[2] = static_cast<double>(bodyXf.p.z) + static_cast<double>(d.z);
  storeQuat(b3NormalizeQuat(b3MulQuat(bodyXf.q, local.q)), out.rot);
}

// lower <= upper, both finite.
void orderedPair(float &lo, float &hi) {
  lo = finiteOr(lo);
  hi = finiteOr(hi);
  if (lo > hi)
    std::swap(lo, hi);
}

// Every value as Box3D accepts it (its asserts would otherwise fire).
JointSettings sanitized(const JointSettings &in) {
  JointSettings s = in;
  s.constraintStiffness = nonNegOr(s.constraintStiffness, 60.0f);
  s.constraintDampingRatio = nonNegOr(s.constraintDampingRatio, 2.0f);
  s.stiffness = nonNegOr(s.stiffness);
  s.dampingRatio = nonNegOr(s.dampingRatio);
  s.linearStiffness = nonNegOr(s.linearStiffness);
  s.linearDampingRatio = nonNegOr(s.linearDampingRatio);
  s.angularStiffness = nonNegOr(s.angularStiffness);
  s.angularDampingRatio = nonNegOr(s.angularDampingRatio);
  s.suspensionStiffness = nonNegOr(s.suspensionStiffness);
  s.suspensionDampingRatio = nonNegOr(s.suspensionDampingRatio);
  s.steeringStiffness = nonNegOr(s.steeringStiffness);
  s.steeringDampingRatio = nonNegOr(s.steeringDampingRatio);
  s.maxMotorTorque = nonNegOr(s.maxMotorTorque);
  s.maxMotorForce = nonNegOr(s.maxMotorForce);
  s.maxVelocityForce = nonNegOr(s.maxVelocityForce);
  s.maxVelocityTorque = nonNegOr(s.maxVelocityTorque);
  s.maxSpringForce = nonNegOr(s.maxSpringForce);
  s.maxSpringTorque = nonNegOr(s.maxSpringTorque);
  s.maxSpinTorque = nonNegOr(s.maxSpinTorque);
  s.maxSteeringTorque = nonNegOr(s.maxSteeringTorque);
  s.motorSpeed = finiteOr(s.motorSpeed);
  s.spinSpeed = finiteOr(s.spinSpeed);
  s.targetTranslation = finiteOr(s.targetTranslation);
  s.targetSteeringAngle = finiteOr(s.targetSteeringAngle);
  s.targetAngle = std::clamp(finiteOr(s.targetAngle), -B3_PI, B3_PI);
  s.coneAngle = std::clamp(finiteOr(s.coneAngle), 0.0f, 0.5f * B3_PI);
  orderedPair(s.lowerAngle, s.upperAngle);
  orderedPair(s.lowerTranslation, s.upperTranslation);
  orderedPair(s.lowerTwistAngle, s.upperTwistAngle);
  orderedPair(s.lowerSuspensionLimit, s.upperSuspensionLimit);
  orderedPair(s.lowerSteeringLimit, s.upperSteeringLimit);
  s.minLength = nonNegOr(s.minLength);
  if (s.maxLength >= 0.0f && std::isfinite(s.maxLength) &&
      s.maxLength < s.minLength)
    s.maxLength = s.minLength;
  sanitizeQuat(s.targetRotation);
  return s;
}

// The base definition every joint shares.
void fillBase(b3JointDef &base, const JointSettings &s, b3BodyId a, b3BodyId b,
              const b3Transform &fa, const b3Transform &fb, void *userData) {
  base.bodyIdA = a;
  base.bodyIdB = b;
  base.localFrameA = fa;
  base.localFrameB = fb;
  base.forceThreshold = cap(s.forceThreshold);
  base.torqueThreshold = cap(s.torqueThreshold);
  base.constraintHertz = s.constraintStiffness;
  base.constraintDampingRatio = s.constraintDampingRatio;
  base.collideConnected = s.collideConnected;
  base.userData = userData;
}

// A world frame relative to a body's current transform.
b3Transform toLocal(b3BodyId body, const JointFrame &f) {
  float q[4] = {f.rot[0], f.rot[1], f.rot[2], f.rot[3]};
  sanitizeQuat(q);
  const b3WorldTransform t = b3Body_GetTransform(body);
  b3Vec3 d;
  d.x = static_cast<float>(f.pos[0] - static_cast<double>(t.p.x));
  d.y = static_cast<float>(f.pos[1] - static_cast<double>(t.p.y));
  d.z = static_cast<float>(f.pos[2] - static_cast<double>(t.p.z));
  b3Transform out;
  out.p = b3InvRotateVector(t.q, d);
  out.q = b3NormalizeQuat(b3InvMulQuat(t.q, makeQuat(q)));
  return out;
}

// A frame already relative to its body. Kept bit for bit when already unit
// length (a stored local frame must come back exact).
b3Transform asLocalFrame(const JointFrame &f) {
  float q[4] = {f.rot[0], f.rot[1], f.rot[2], f.rot[3]};
  const double len2 =
      static_cast<double>(q[0]) * q[0] + static_cast<double>(q[1]) * q[1] +
      static_cast<double>(q[2]) * q[2] + static_cast<double>(q[3]) * q[3];
  if (!(std::fabs(len2 - 1.0) < 1e-6))
    sanitizeQuat(q);
  b3Transform t;
  t.p.x = static_cast<float>(f.pos[0]);
  t.p.y = static_cast<float>(f.pos[1]);
  t.p.z = static_cast<float>(f.pos[2]);
  t.q = makeQuat(q);
  return t;
}

// What every per-type create function needs: the world, the sanitized settings,
// both bodies with their local frames, and (distance joints) the rest length to
// make the joint with.
struct JointBuild {
  b3WorldId world;
  const JointSettings &s;
  b3BodyId a, b;
  b3Transform fa, fb;
  void *userData;
  float distanceLength = 0.0f;

  void fillBase(b3JointDef &base) const {
    ::b3dop::fillBase(base, s, a, b, fa, fb, userData);
  }
};

b3JointId createWeldJoint(const JointBuild &j) {
  const JointSettings &s = j.s;
  b3WeldJointDef d = b3DefaultWeldJointDef();
  j.fillBase(d.base);
  d.linearHertz = s.enableLinearSpring ? s.linearStiffness : 0.0f; // 0: rigid
  d.linearDampingRatio = s.linearDampingRatio;
  d.angularHertz = s.enableAngularSpring ? s.angularStiffness : 0.0f;
  d.angularDampingRatio = s.angularDampingRatio;
  return b3CreateWeldJoint(j.world, &d);
}

b3JointId createSphericalJoint(const JointBuild &j) {
  const JointSettings &s = j.s;
  b3SphericalJointDef d = b3DefaultSphericalJointDef();
  j.fillBase(d.base);
  d.enableSpring = s.enableSpring;
  d.hertz = s.stiffness;
  d.dampingRatio = s.dampingRatio;
  d.targetRotation = makeQuat(s.targetRotation);
  d.enableConeLimit = s.enableConeLimit;
  d.coneAngle = s.coneAngle;
  d.enableTwistLimit = s.enableTwistLimit;
  d.lowerTwistAngle = s.lowerTwistAngle;
  d.upperTwistAngle = s.upperTwistAngle;
  d.enableMotor = s.enableMotor;
  d.maxMotorTorque = s.maxMotorTorque;
  d.motorVelocity = finiteVec3(s.motorVelocity);
  return b3CreateSphericalJoint(j.world, &d);
}

b3JointId createRevoluteJoint(const JointBuild &j) {
  const JointSettings &s = j.s;
  b3RevoluteJointDef d = b3DefaultRevoluteJointDef();
  j.fillBase(d.base);
  d.targetAngle = s.targetAngle;
  d.enableSpring = s.enableSpring;
  d.hertz = s.stiffness;
  d.dampingRatio = s.dampingRatio;
  d.enableLimit = s.enableLimit;
  d.lowerAngle = s.lowerAngle;
  d.upperAngle = s.upperAngle;
  d.enableMotor = s.enableMotor;
  d.maxMotorTorque = s.maxMotorTorque;
  d.motorSpeed = s.motorSpeed;
  return b3CreateRevoluteJoint(j.world, &d);
}

b3JointId createPrismaticJoint(const JointBuild &j) {
  const JointSettings &s = j.s;
  b3PrismaticJointDef d = b3DefaultPrismaticJointDef();
  j.fillBase(d.base);
  d.enableSpring = s.enableSpring;
  d.hertz = s.stiffness;
  d.dampingRatio = s.dampingRatio;
  d.targetTranslation = s.targetTranslation;
  d.enableLimit = s.enableLimit;
  d.lowerTranslation = s.lowerTranslation;
  d.upperTranslation = s.upperTranslation;
  d.enableMotor = s.enableMotor;
  d.maxMotorForce = s.maxMotorForce;
  d.motorSpeed = s.motorSpeed;
  return b3CreatePrismaticJoint(j.world, &d);
}

b3JointId createDistanceJoint(const JointBuild &j) {
  const JointSettings &s = j.s;
  b3DistanceJointDef d = b3DefaultDistanceJointDef();
  j.fillBase(d.base);
  d.length = std::max(j.distanceLength,
                      1e-4f); // Box3D wants > 0 (and clamps to its slop)
  d.enableSpring = s.enableSpring;
  d.lowerSpringForce = distanceLowerSpringForce(s);
  d.upperSpringForce = distanceUpperSpringForce(s);
  d.hertz = s.stiffness;
  d.dampingRatio = s.dampingRatio;
  d.enableLimit = s.enableLimit;
  d.minLength = s.minLength;
  d.maxLength = distanceMaxLength(s);
  d.enableMotor = s.enableMotor;
  d.maxMotorForce = s.maxMotorForce;
  d.motorSpeed = s.motorSpeed;
  return b3CreateDistanceJoint(j.world, &d);
}

b3JointId createMotorJoint(const JointBuild &j) {
  const JointSettings &s = j.s;
  b3MotorJointDef d = b3DefaultMotorJointDef();
  j.fillBase(d.base);
  d.linearVelocity = finiteVec3(s.linearVelocity);
  d.maxVelocityForce = s.maxVelocityForce;
  d.angularVelocity = finiteVec3(s.angularVelocity);
  d.maxVelocityTorque = s.maxVelocityTorque;
  d.linearHertz = s.linearStiffness;
  d.linearDampingRatio = s.linearDampingRatio;
  d.maxSpringForce = s.maxSpringForce;
  d.angularHertz = s.angularStiffness;
  d.angularDampingRatio = s.angularDampingRatio;
  d.maxSpringTorque = s.maxSpringTorque;
  return b3CreateMotorJoint(j.world, &d);
}

b3JointId createParallelJoint(const JointBuild &j) {
  const JointSettings &s = j.s;
  b3ParallelJointDef d = b3DefaultParallelJointDef();
  j.fillBase(d.base);
  d.hertz = s.stiffness;
  d.dampingRatio = s.dampingRatio;
  d.maxTorque = cap(s.maxTorque);
  return b3CreateParallelJoint(j.world, &d);
}

b3JointId createWheelJoint(const JointBuild &j) {
  const JointSettings &s = j.s;
  b3WheelJointDef d = b3DefaultWheelJointDef();
  j.fillBase(d.base);
  d.enableSuspensionSpring = s.enableSuspensionSpring;
  d.suspensionHertz = s.suspensionStiffness;
  d.suspensionDampingRatio = s.suspensionDampingRatio;
  d.enableSuspensionLimit = s.enableSuspensionLimit;
  d.lowerSuspensionLimit = s.lowerSuspensionLimit;
  d.upperSuspensionLimit = s.upperSuspensionLimit;
  d.enableSpinMotor = s.enableSpinMotor;
  d.maxSpinTorque = s.maxSpinTorque;
  d.spinSpeed = s.spinSpeed;
  d.enableSteering = s.enableSteering;
  d.steeringHertz = s.steeringStiffness;
  d.steeringDampingRatio = s.steeringDampingRatio;
  d.targetSteeringAngle = s.targetSteeringAngle;
  d.maxSteeringTorque = s.maxSteeringTorque;
  d.enableSteeringLimit = s.enableSteeringLimit;
  d.lowerSteeringLimit = s.lowerSteeringLimit;
  d.upperSteeringLimit = s.upperSteeringLimit;
  return b3CreateWheelJoint(j.world, &d);
}

b3JointId createFilterJoint(const JointBuild &j) {
  b3FilterJointDef d = b3DefaultFilterJointDef();
  j.fillBase(d.base);
  d.base.collideConnected = false; // its whole purpose
  return b3CreateFilterJoint(j.world, &d);
}

b3JointId createJointOfType(JointType type, const JointBuild &j) {
  switch (type) {
  case JointType::Weld:
    return createWeldJoint(j);
  case JointType::Spherical:
    return createSphericalJoint(j);
  case JointType::Revolute:
    return createRevoluteJoint(j);
  case JointType::Prismatic:
    return createPrismaticJoint(j);
  case JointType::Distance:
    return createDistanceJoint(j);
  case JointType::Motor:
    return createMotorJoint(j);
  case JointType::Parallel:
    return createParallelJoint(j);
  case JointType::Wheel:
    return createWheelJoint(j);
  case JointType::Filter:
    return createFilterJoint(j);
  }
  return b3_nullJointId;
}

} // namespace

JointSettings Box3DWorld::defaultJointSettings(JointType type) {
  JointSettings s;
  const b3JointDef *base = nullptr;
  // Each b3Default*JointDef, copied field by field. Box3D's FLT_MAX caps are
  // our -1 (unlimited).
  auto unlimited = [](float v) { return v >= 0.5f * FLT_MAX ? -1.0f : v; };
  switch (type) {
  case JointType::Weld: {
    static const b3WeldJointDef d = b3DefaultWeldJointDef();
    base = &d.base;
    s.enableLinearSpring = d.linearHertz > 0.0f;
    s.enableAngularSpring = d.angularHertz > 0.0f;
    s.linearStiffness = d.linearHertz;
    s.linearDampingRatio = d.linearDampingRatio;
    s.angularStiffness = d.angularHertz;
    s.angularDampingRatio = d.angularDampingRatio;
    break;
  }
  case JointType::Spherical: {
    static const b3SphericalJointDef d = b3DefaultSphericalJointDef();
    base = &d.base;
    s.enableSpring = d.enableSpring;
    s.stiffness = d.hertz;
    s.dampingRatio = d.dampingRatio;
    s.targetRotation[0] = d.targetRotation.v.x;
    s.targetRotation[1] = d.targetRotation.v.y;
    s.targetRotation[2] = d.targetRotation.v.z;
    s.targetRotation[3] = d.targetRotation.s;
    s.enableConeLimit = d.enableConeLimit;
    s.coneAngle = d.coneAngle;
    s.enableTwistLimit = d.enableTwistLimit;
    s.lowerTwistAngle = d.lowerTwistAngle;
    s.upperTwistAngle = d.upperTwistAngle;
    s.enableMotor = d.enableMotor;
    s.maxMotorTorque = d.maxMotorTorque;
    storeVec3(d.motorVelocity, s.motorVelocity);
    break;
  }
  case JointType::Revolute: {
    static const b3RevoluteJointDef d = b3DefaultRevoluteJointDef();
    base = &d.base;
    s.targetAngle = d.targetAngle;
    s.enableSpring = d.enableSpring;
    s.stiffness = d.hertz;
    s.dampingRatio = d.dampingRatio;
    s.enableLimit = d.enableLimit;
    s.lowerAngle = d.lowerAngle;
    s.upperAngle = d.upperAngle;
    s.enableMotor = d.enableMotor;
    s.maxMotorTorque = d.maxMotorTorque;
    s.motorSpeed = d.motorSpeed;
    break;
  }
  case JointType::Prismatic: {
    static const b3PrismaticJointDef d = b3DefaultPrismaticJointDef();
    base = &d.base;
    s.enableSpring = d.enableSpring;
    s.stiffness = d.hertz;
    s.dampingRatio = d.dampingRatio;
    s.targetTranslation = d.targetTranslation;
    s.enableLimit = d.enableLimit;
    s.lowerTranslation = d.lowerTranslation;
    s.upperTranslation = d.upperTranslation;
    s.enableMotor = d.enableMotor;
    s.maxMotorForce = d.maxMotorForce;
    s.motorSpeed = d.motorSpeed;
    break;
  }
  case JointType::Distance: {
    static const b3DistanceJointDef d = b3DefaultDistanceJointDef();
    base = &d.base;
    s.restLength = -1.0f; // ours: the anchors' distance at creation
    s.enableSpring = d.enableSpring;
    s.maxTensionForce = unlimited(-d.lowerSpringForce);
    s.maxCompressionForce = unlimited(d.upperSpringForce);
    s.stiffness = d.hertz;
    s.dampingRatio = d.dampingRatio;
    s.enableLimit = d.enableLimit;
    s.minLength = d.minLength;
    s.maxLength = d.maxLength >= 0.5f * B3_HUGE ? -1.0f : d.maxLength;
    s.enableMotor = d.enableMotor;
    s.maxMotorForce = d.maxMotorForce;
    s.motorSpeed = d.motorSpeed;
    break;
  }
  case JointType::Motor: {
    static const b3MotorJointDef d = b3DefaultMotorJointDef();
    base = &d.base;
    storeVec3(d.linearVelocity, s.linearVelocity);
    s.maxVelocityForce = d.maxVelocityForce;
    storeVec3(d.angularVelocity, s.angularVelocity);
    s.maxVelocityTorque = d.maxVelocityTorque;
    s.linearStiffness = d.linearHertz;
    s.linearDampingRatio = d.linearDampingRatio;
    s.maxSpringForce = d.maxSpringForce;
    s.angularStiffness = d.angularHertz;
    s.angularDampingRatio = d.angularDampingRatio;
    s.maxSpringTorque = d.maxSpringTorque;
    break;
  }
  case JointType::Parallel: {
    static const b3ParallelJointDef d = b3DefaultParallelJointDef();
    base = &d.base;
    s.stiffness = d.hertz;
    s.dampingRatio = d.dampingRatio;
    s.maxTorque = unlimited(d.maxTorque);
    break;
  }
  case JointType::Wheel: {
    static const b3WheelJointDef d = b3DefaultWheelJointDef();
    base = &d.base;
    s.enableSuspensionSpring = d.enableSuspensionSpring;
    s.suspensionStiffness = d.suspensionHertz;
    s.suspensionDampingRatio = d.suspensionDampingRatio;
    s.enableSuspensionLimit = d.enableSuspensionLimit;
    s.lowerSuspensionLimit = d.lowerSuspensionLimit;
    s.upperSuspensionLimit = d.upperSuspensionLimit;
    s.enableSpinMotor = d.enableSpinMotor;
    s.maxSpinTorque = d.maxSpinTorque;
    s.spinSpeed = d.spinSpeed;
    s.enableSteering = d.enableSteering;
    s.steeringStiffness = d.steeringHertz;
    s.steeringDampingRatio = d.steeringDampingRatio;
    s.targetSteeringAngle = d.targetSteeringAngle;
    s.maxSteeringTorque = d.maxSteeringTorque;
    s.enableSteeringLimit = d.enableSteeringLimit;
    s.lowerSteeringLimit = d.lowerSteeringLimit;
    s.upperSteeringLimit = d.upperSteeringLimit;
    break;
  }
  case JointType::Filter: {
    static const b3FilterJointDef d = b3DefaultFilterJointDef();
    base = &d.base;
    break;
  }
  }
  if (base) {
    s.collideConnected = base->collideConnected;
    s.forceThreshold = unlimited(base->forceThreshold);
    s.torqueThreshold = unlimited(base->torqueThreshold);
    s.constraintStiffness = base->constraintHertz;
    s.constraintDampingRatio = base->constraintDampingRatio;
  }
  return s;
}

JointHandle Box3DWorld::createJoint(JointType type, BodyHandle a, BodyHandle b,
                                    const JointFrame &frameA,
                                    const JointFrame &frameB,
                                    const JointSettings &in, void *userData,
                                    bool framesAreLocal) {
  if (!isValid() || (!a && !b) || a == b)
    return 0;
  if (!finite3(frameA.pos) || !finite3(frameB.pos))
    return 0;
  auto bodyOf = [&](BodyHandle h) -> b3BodyId {
    if (h)
      return b3LoadBodyId(h);
    if (B3_IS_NULL(myImpl->anchor)) {
      b3BodyDef def = b3DefaultBodyDef();
      def.type = b3_staticBody;
      myImpl->anchor = b3CreateBody(myImpl->worldId, &def);
    }
    return myImpl->anchor;
  };
  const b3BodyId ida = bodyOf(a), idb = bodyOf(b);
  if (!b3Body_IsValid(ida) || !b3Body_IsValid(idb))
    return 0;

  const JointSettings s = sanitized(in);
  const b3Transform fa =
      framesAreLocal ? asLocalFrame(frameA) : toLocal(ida, frameA);
  const b3Transform fb =
      framesAreLocal ? asLocalFrame(frameB) : toLocal(idb, frameB);
  if (!b3IsValidVec3(fa.p) || !b3IsValidVec3(fb.p))
    return 0;

  JointBuild build{myImpl->worldId, s, ida, idb, fa, fb, userData};
  if (type == JointType::Distance) {
    float len = s.restLength;
    if (!(len >= 0.0f) || !std::isfinite(len)) {
      // The anchors' distance now (from the local frames: through the bodies'
      // poses).
      JointFrame wa = frameA, wb = frameB;
      if (framesAreLocal) {
        localToWorld(a, frameA, wa);
        localToWorld(b, frameB, wb);
      }
      const double dx = wb.pos[0] - wa.pos[0], dy = wb.pos[1] - wa.pos[1],
                   dz = wb.pos[2] - wa.pos[2];
      len = static_cast<float>(std::sqrt(dx * dx + dy * dy + dz * dz));
    }
    build.distanceLength = len;
  }
  const b3JointId id = createJointOfType(type, build);
  return B3_IS_NULL(id) ? 0 : b3StoreJointId(id);
}

void Box3DWorld::destroyJoint(JointHandle h) {
  if (isJointValid(h))
    b3DestroyJoint(b3LoadJointId(h), true);
}

bool Box3DWorld::isJointValid(JointHandle h) const {
  return h && isValid() && b3Joint_IsValid(b3LoadJointId(h));
}

void Box3DWorld::updateJoint(JointHandle h, JointType type,
                             const JointSettings &in) {
  if (!isJointValid(h))
    return;
  const b3JointId id = b3LoadJointId(h);
  const JointSettings s = sanitized(in);
  b3Joint_SetCollideConnected(
      id, type == JointType::Filter ? false : s.collideConnected);
  b3Joint_SetForceThreshold(id, cap(s.forceThreshold));
  b3Joint_SetTorqueThreshold(id, cap(s.torqueThreshold));
  b3Joint_SetConstraintTuning(id, s.constraintStiffness,
                              s.constraintDampingRatio);
  switch (type) {
  case JointType::Weld:
    b3WeldJoint_SetLinearHertz(id,
                               s.enableLinearSpring ? s.linearStiffness : 0.0f);
    b3WeldJoint_SetLinearDampingRatio(id, s.linearDampingRatio);
    b3WeldJoint_SetAngularHertz(id, s.enableAngularSpring ? s.angularStiffness
                                                          : 0.0f);
    b3WeldJoint_SetAngularDampingRatio(id, s.angularDampingRatio);
    break;
  case JointType::Spherical:
    b3SphericalJoint_EnableSpring(id, s.enableSpring);
    b3SphericalJoint_SetSpringHertz(id, s.stiffness);
    b3SphericalJoint_SetSpringDampingRatio(id, s.dampingRatio);
    b3SphericalJoint_SetTargetRotation(id, makeQuat(s.targetRotation));
    b3SphericalJoint_EnableConeLimit(id, s.enableConeLimit);
    b3SphericalJoint_SetConeLimit(id, s.coneAngle);
    b3SphericalJoint_EnableTwistLimit(id, s.enableTwistLimit);
    b3SphericalJoint_SetTwistLimits(id, s.lowerTwistAngle, s.upperTwistAngle);
    b3SphericalJoint_EnableMotor(id, s.enableMotor);
    b3SphericalJoint_SetMotorVelocity(id, finiteVec3(s.motorVelocity));
    b3SphericalJoint_SetMaxMotorTorque(id, s.maxMotorTorque);
    break;
  case JointType::Revolute:
    b3RevoluteJoint_EnableSpring(id, s.enableSpring);
    b3RevoluteJoint_SetSpringHertz(id, s.stiffness);
    b3RevoluteJoint_SetSpringDampingRatio(id, s.dampingRatio);
    b3RevoluteJoint_SetTargetAngle(id, s.targetAngle);
    b3RevoluteJoint_EnableLimit(id, s.enableLimit);
    b3RevoluteJoint_SetLimits(id, s.lowerAngle, s.upperAngle);
    b3RevoluteJoint_EnableMotor(id, s.enableMotor);
    b3RevoluteJoint_SetMotorSpeed(id, s.motorSpeed);
    b3RevoluteJoint_SetMaxMotorTorque(id, s.maxMotorTorque);
    break;
  case JointType::Prismatic:
    b3PrismaticJoint_EnableSpring(id, s.enableSpring);
    b3PrismaticJoint_SetSpringHertz(id, s.stiffness);
    b3PrismaticJoint_SetSpringDampingRatio(id, s.dampingRatio);
    b3PrismaticJoint_SetTargetTranslation(id, s.targetTranslation);
    b3PrismaticJoint_EnableLimit(id, s.enableLimit);
    b3PrismaticJoint_SetLimits(id, s.lowerTranslation, s.upperTranslation);
    b3PrismaticJoint_EnableMotor(id, s.enableMotor);
    b3PrismaticJoint_SetMotorSpeed(id, s.motorSpeed);
    b3PrismaticJoint_SetMaxMotorForce(id, s.maxMotorForce);
    break;
  case JointType::Distance:
    if (s.restLength >= 0.0f &&
        std::isfinite(s.restLength)) // else keep the length it was made with
      b3DistanceJoint_SetLength(id, std::max(s.restLength, 1e-4f));
    b3DistanceJoint_EnableSpring(id, s.enableSpring);
    b3DistanceJoint_SetSpringForceRange(id, distanceLowerSpringForce(s),
                                        distanceUpperSpringForce(s));
    b3DistanceJoint_SetSpringHertz(id, s.stiffness);
    b3DistanceJoint_SetSpringDampingRatio(id, s.dampingRatio);
    b3DistanceJoint_EnableLimit(id, s.enableLimit);
    b3DistanceJoint_SetLengthRange(id, s.minLength, distanceMaxLength(s));
    b3DistanceJoint_EnableMotor(id, s.enableMotor);
    b3DistanceJoint_SetMotorSpeed(id, s.motorSpeed);
    b3DistanceJoint_SetMaxMotorForce(id, s.maxMotorForce);
    break;
  case JointType::Motor:
    b3MotorJoint_SetLinearVelocity(id, finiteVec3(s.linearVelocity));
    b3MotorJoint_SetMaxVelocityForce(id, s.maxVelocityForce);
    b3MotorJoint_SetAngularVelocity(id, finiteVec3(s.angularVelocity));
    b3MotorJoint_SetMaxVelocityTorque(id, s.maxVelocityTorque);
    b3MotorJoint_SetLinearHertz(id, s.linearStiffness);
    b3MotorJoint_SetLinearDampingRatio(id, s.linearDampingRatio);
    b3MotorJoint_SetMaxSpringForce(id, s.maxSpringForce);
    b3MotorJoint_SetAngularHertz(id, s.angularStiffness);
    b3MotorJoint_SetAngularDampingRatio(id, s.angularDampingRatio);
    b3MotorJoint_SetMaxSpringTorque(id, s.maxSpringTorque);
    break;
  case JointType::Parallel:
    b3ParallelJoint_SetSpringHertz(id, s.stiffness);
    b3ParallelJoint_SetSpringDampingRatio(id, s.dampingRatio);
    b3ParallelJoint_SetMaxTorque(id, cap(s.maxTorque));
    break;
  case JointType::Wheel:
    b3WheelJoint_EnableSuspension(id, s.enableSuspensionSpring);
    b3WheelJoint_SetSuspensionHertz(id, s.suspensionStiffness);
    b3WheelJoint_SetSuspensionDampingRatio(id, s.suspensionDampingRatio);
    b3WheelJoint_EnableSuspensionLimit(id, s.enableSuspensionLimit);
    b3WheelJoint_SetSuspensionLimits(id, s.lowerSuspensionLimit,
                                     s.upperSuspensionLimit);
    b3WheelJoint_EnableSpinMotor(id, s.enableSpinMotor);
    b3WheelJoint_SetSpinMotorSpeed(id, s.spinSpeed);
    b3WheelJoint_SetMaxSpinTorque(id, s.maxSpinTorque);
    b3WheelJoint_EnableSteering(id, s.enableSteering);
    b3WheelJoint_SetSteeringHertz(id, s.steeringStiffness);
    b3WheelJoint_SetSteeringDampingRatio(id, s.steeringDampingRatio);
    b3WheelJoint_SetTargetSteeringAngle(id, s.targetSteeringAngle);
    b3WheelJoint_SetMaxSteeringTorque(id, s.maxSteeringTorque);
    b3WheelJoint_EnableSteeringLimit(id, s.enableSteeringLimit);
    b3WheelJoint_SetSteeringLimits(id, s.lowerSteeringLimit,
                                   s.upperSteeringLimit);
    break;
  case JointType::Filter:
    break;
  }
  b3Joint_WakeBodies(id);
}

bool Box3DWorld::jointFeedback(JointHandle h, JointType type,
                               JointFeedback &out) const {
  out = JointFeedback();
  if (!isJointValid(h))
    return false;
  const b3JointId id = b3LoadJointId(h);
  storeVec3(b3Joint_GetConstraintForce(id), out.force);
  storeVec3(b3Joint_GetConstraintTorque(id), out.torque);
  out.linearSeparation = b3Joint_GetLinearSeparation(id);
  // Box3D has not implemented the wheel's angular separation yet (a todo that
  // asserts): 0.
  out.angularSeparation =
      type == JointType::Wheel ? 0.0f : b3Joint_GetAngularSeparation(id);
  auto extra = [&](const char *name, float v, bool angle) {
    if (out.extraCount >= JointFeedback::kMaxExtra)
      return;
    out.extraName[out.extraCount] = name;
    out.extra[out.extraCount] = v;
    out.extraIsAngle[out.extraCount] = angle;
    ++out.extraCount;
  };
  switch (type) {
  case JointType::Spherical:
    extra("current_cone_angle", b3SphericalJoint_GetConeAngle(id), true);
    extra("current_twist_angle", b3SphericalJoint_GetTwistAngle(id), true);
    extra("motor_torque", b3Length(b3SphericalJoint_GetMotorTorque(id)),
          false); // a vector: its size
    break;
  case JointType::Revolute:
    extra("current_angle", b3RevoluteJoint_GetAngle(id), true);
    extra("motor_torque", b3RevoluteJoint_GetMotorTorque(id), false);
    break;
  case JointType::Prismatic:
    extra("current_translation", b3PrismaticJoint_GetTranslation(id), false);
    extra("current_speed", b3PrismaticJoint_GetSpeed(id), false);
    extra("motor_force", b3PrismaticJoint_GetMotorForce(id), false);
    break;
  case JointType::Distance:
    extra("current_length", b3DistanceJoint_GetCurrentLength(id), false);
    extra("motor_force", b3DistanceJoint_GetMotorForce(id), false);
    break;
  case JointType::Wheel:
    extra("current_spin_speed", b3WheelJoint_GetSpinSpeed(id), true);
    extra("current_steering_angle", b3WheelJoint_GetSteeringAngle(id), true);
    extra("spin_torque", b3WheelJoint_GetSpinTorque(id), false);
    break;
  default:
    break;
  }
  return true;
}

bool Box3DWorld::jointWorldFrames(JointHandle h, JointFrame &a,
                                  JointFrame &b) const {
  if (!isJointValid(h))
    return false;
  const b3JointId id = b3LoadJointId(h);
  frameToWorld(b3Body_GetTransform(b3Joint_GetBodyA(id)),
               b3Joint_GetLocalFrameA(id), a);
  frameToWorld(b3Body_GetTransform(b3Joint_GetBodyB(id)),
               b3Joint_GetLocalFrameB(id), b);
  return true;
}

bool Box3DWorld::jointLocalFrames(JointHandle h, JointFrame &a,
                                  JointFrame &b) const {
  if (!isJointValid(h))
    return false;
  const b3JointId id = b3LoadJointId(h);
  auto store = [](const b3Transform &t, JointFrame &out) {
    out.pos[0] = t.p.x;
    out.pos[1] = t.p.y;
    out.pos[2] = t.p.z;
    storeQuat(t.q, out.rot);
  };
  store(b3Joint_GetLocalFrameA(id), a);
  store(b3Joint_GetLocalFrameB(id), b);
  return true;
}

bool Box3DWorld::localToWorld(BodyHandle h, const JointFrame &local,
                              JointFrame &out) const {
  if (!isValid())
    return false;
  if (!h) {
    out = local; // the world body sits at the origin, unrotated
    return true;
  }
  const b3BodyId id = b3LoadBodyId(h);
  if (!b3Body_IsValid(id))
    return false;
  float q[4] = {local.rot[0], local.rot[1], local.rot[2], local.rot[3]};
  sanitizeQuat(q);
  b3Transform lt;
  lt.p.x = static_cast<float>(local.pos[0]);
  lt.p.y = static_cast<float>(local.pos[1]);
  lt.p.z = static_cast<float>(local.pos[2]);
  lt.q = makeQuat(q);
  frameToWorld(b3Body_GetTransform(id), lt, out);
  return true;
}

int Box3DWorld::brokenJoints(std::vector<void *> &userData) const {
  userData.clear();
  if (!isValid())
    return 0;
  const b3JointEvents ev = b3World_GetJointEvents(myImpl->worldId);
  for (int i = 0; i < ev.count; ++i)
    userData.push_back(ev.jointEvents[i].userData);
  return static_cast<int>(userData.size());
}

int Box3DWorld::jointCount() const {
  return isValid() ? b3World_GetCounters(myImpl->worldId).jointCount : 0;
}

} // namespace b3dop
