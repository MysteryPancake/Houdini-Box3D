// Box3DJoints.h
//
// Houdini-free description of Box3D joints: the joint types, their settings,
// and one table of parameters (attribute / parameter names, kinds, which types
// use them) that the Houdini side uses for the Box3D constraint relationship
// nodes, primitive attributes and change detection. Box3DWorld creates the
// joints. Settings are in Box3D units: radians, radians per second, Hz.
#pragma once

#include <algorithm>
#include <array>
#include <cstring>
#include <string>
#include <vector>

#include "Box3DMath.h" // kDegToRad, kRadToDeg

namespace b3dop {

// The Box3D joints that make sense for rigid bodies (no character-mover
// joints).
enum class JointType : int {
  Weld = 0,  // rigid, or soft with linear / angular springs
  Spherical, // ball and socket: cone and twist limits, spring, motor
  Revolute,  // hinge about frame z: limits, spring, motor
  Prismatic, // slider along frame A's x: limits, spring, motor
  Distance,  // keeps two points at a distance: length range, spring, motor
  // drives B relative to A by velocity and springs, with capped force / torque
  Motor,
  Parallel, // keeps the frames' z axes parallel with a spring
  Wheel,    // wheel on a chassis: suspension, spin motor, steering
  Filter,   // no constraint: the two bodies do not collide
};
constexpr int kJointTypeCount = 9;

constexpr unsigned jointBit(JointType t) {
  return 1u << static_cast<unsigned>(t);
}
constexpr unsigned kAllJoints = (1u << kJointTypeCount) - 1u;

inline const char *jointTypeName(JointType t) {
  static const char *const kNames[kJointTypeCount] = {
      "weld",  "spherical", "revolute", "prismatic", "distance",
      "motor", "parallel",  "wheel",    "filter"};
  const int i = static_cast<int>(t);
  return (i >= 0 && i < kJointTypeCount) ? kNames[i] : "";
}

inline const char *jointTypeLabel(JointType t) {
  static const char *const kLabels[kJointTypeCount] = {
      "Weld",  "Spherical", "Revolute", "Prismatic", "Distance",
      "Motor", "Parallel",  "Wheel",    "Filter"};
  const int i = static_cast<int>(t);
  return (i >= 0 && i < kJointTypeCount) ? kLabels[i] : "";
}

// Every setting of every joint type, in Box3D units. Caps marked "< 0:
// unlimited" use a negative value where Box3D uses FLT_MAX.
// Box3DWorld::defaultJointSettings gives Box3D's defaults per type.
struct JointSettings {
  // Every type.
  bool collideConnected = false;
  float forceThreshold = -1.0f;  // breaks above this force; < 0: never
  float torqueThreshold = -1.0f; // breaks above this torque; < 0: never
  float constraintStiffness =
      60.0f; // Hz: how hard the joint pulls back to its frames
  float constraintDampingRatio = 2.0f;

  // Springs (spherical, revolute, prismatic, distance; parallel without the
  // toggle).
  bool enableSpring = false;
  float stiffness = 0.0f; // Hz
  float dampingRatio = 0.0f;

  // Limits.
  bool enableLimit = false; // revolute, prismatic, distance
  float lowerAngle = 0.0f;  // revolute
  float upperAngle = 0.0f;
  float lowerTranslation = 0.0f; // prismatic
  float upperTranslation = 0.0f;
  float minLength = 0.0f;  // distance
  float maxLength = -1.0f; // distance; < 0: unlimited

  // Motors.
  bool enableMotor = false;    // spherical, revolute, prismatic, distance
  float maxMotorTorque = 0.0f; // spherical, revolute
  float maxMotorForce = 0.0f;  // prismatic, distance
  float motorSpeed = 0.0f; // revolute (rad/s); prismatic, distance (units/s)
  float motorVelocity[3] = {0, 0, 0}; // spherical (rad/s)

  // Spring targets.
  float targetAngle = 0.0f;               // revolute
  float targetTranslation = 0.0f;         // prismatic
  float targetRotation[4] = {0, 0, 0, 1}; // spherical, quaternion x, y, z, w

  // Spherical limits.
  bool enableConeLimit = false;
  float coneAngle = 0.0f; // 0 .. pi/2, about frame A's z
  bool enableTwistLimit = false;
  float lowerTwistAngle = 0.0f; // about frame B's z
  float upperTwistAngle = 0.0f;

  // Weld springs (off: rigid in that part) and the motor joint's springs.
  bool enableLinearSpring = false;  // weld
  bool enableAngularSpring = false; // weld
  float linearStiffness = 0.0f;     // weld, motor (Hz)
  float linearDampingRatio = 0.0f;
  float angularStiffness = 0.0f; // weld, motor (Hz)
  float angularDampingRatio = 0.0f;

  // Distance.
  float restLength = -1.0f; // < 0: the anchors' distance when the joint is made
  float maxTensionForce = -1.0f;     // spring: < 0: unlimited
  float maxCompressionForce = -1.0f; // spring: < 0: unlimited

  // Motor joint.
  float linearVelocity[3] = {0, 0, 0};
  float maxVelocityForce = 0.0f;
  float angularVelocity[3] = {0, 0, 0}; // rad/s
  float maxVelocityTorque = 0.0f;
  float maxSpringForce = 0.0f;
  float maxSpringTorque = 0.0f;

  // Parallel.
  float maxTorque = -1.0f; // < 0: unlimited

  // Wheel.
  bool enableSuspensionSpring = false;
  float suspensionStiffness = 0.0f; // Hz
  float suspensionDampingRatio = 0.0f;
  bool enableSuspensionLimit = false;
  float lowerSuspensionLimit = 0.0f;
  float upperSuspensionLimit = 0.0f;
  bool enableSpinMotor = false;
  float maxSpinTorque = 0.0f;
  float spinSpeed = 0.0f; // rad/s
  bool enableSteering = false;
  float steeringStiffness = 0.0f; // Hz
  float steeringDampingRatio = 0.0f;
  float targetSteeringAngle = 0.0f;
  float maxSteeringTorque = 0.0f;
  bool enableSteeringLimit = false;
  float lowerSteeringLimit = 0.0f;
  float upperSteeringLimit = 0.0f;
};

// One setting as Houdini sees it: attribute name (also the parameter name on
// the type's Box3D constraint relationship node), label, kind and the joint
// types that use it.
struct JointParam {
  enum Kind {
    Toggle, // int attribute / toggle
    Float,  // float, Box3D units
    // float, degrees in Houdini (degrees per second for speeds), radians here
    Angle,
    Vec3,      // vector
    Vec3Angle, // vector of angles / angular speeds: degrees in Houdini
    Rotation,  // Euler angles in degrees (XYZ) in Houdini, a quaternion here
  };

  const char *name = "";
  const char *label = "";
  Kind kind = Float;
  unsigned types = 0; // jointBit mask
  const char *help = "";

  bool JointSettings::*b = nullptr;       // Toggle
  float JointSettings::*f = nullptr;      // Float, Angle
  float (JointSettings::*v)[3] = nullptr; // Vec3, Vec3Angle
  float (JointSettings::*q)[4] = nullptr; // Rotation

  unsigned angleTypes =
      0; // Float only: types for which the value is an angle (speed)
  const char *enabledBy =
      nullptr; // name of the toggle that switches this setting on, if any

  bool usedBy(JointType t) const { return (types & jointBit(t)) != 0; }
  // Degrees in Houdini, radians here, for joint type `t`.
  bool isAngle(JointType t) const {
    return kind == Angle || kind == Vec3Angle || kind == Rotation ||
           (angleTypes & jointBit(t)) != 0;
  }
};

namespace detail {
inline JointParam jp(const char *name, const char *label, unsigned types,
                     bool JointSettings::*m, const char *help) {
  JointParam p;
  p.name = name;
  p.label = label;
  p.kind = JointParam::Toggle;
  p.types = types;
  p.b = m;
  p.help = help;
  return p;
}
inline JointParam jp(const char *name, const char *label, unsigned types,
                     float JointSettings::*m, const char *help,
                     JointParam::Kind kind = JointParam::Float) {
  JointParam p;
  p.name = name;
  p.label = label;
  p.kind = kind;
  p.types = types;
  p.f = m;
  p.help = help;
  return p;
}
inline JointParam jp(const char *name, const char *label, unsigned types,
                     float (JointSettings::*m)[3], const char *help,
                     JointParam::Kind kind = JointParam::Vec3) {
  JointParam p;
  p.name = name;
  p.label = label;
  p.kind = kind;
  p.types = types;
  p.v = m;
  p.help = help;
  return p;
}
inline JointParam withAngleTypes(JointParam p, unsigned angleTypes) {
  p.angleTypes = angleTypes;
  return p;
}
inline JointParam enabledBy(JointParam p, const char *toggle) {
  p.enabledBy = toggle;
  return p;
}
inline JointParam jp(const char *name, const char *label, unsigned types,
                     float (JointSettings::*m)[4], const char *help) {
  JointParam p;
  p.name = name;
  p.label = label;
  p.kind = JointParam::Rotation;
  p.types = types;
  p.q = m;
  p.help = help;
  return p;
}
} // namespace detail

// Every setting, common ones first (types == kAllJoints), then grouped by joint
// type.
inline const std::vector<JointParam> &jointParams() {
  using detail::enabledBy;
  using detail::jp;
  using detail::withAngleTypes;
  using K = JointParam;
  using S = JointSettings;
  constexpr unsigned W = jointBit(JointType::Weld),
                     Sp = jointBit(JointType::Spherical),
                     R = jointBit(JointType::Revolute),
                     P = jointBit(JointType::Prismatic),
                     D = jointBit(JointType::Distance),
                     M = jointBit(JointType::Motor),
                     Pa = jointBit(JointType::Parallel),
                     Wh = jointBit(JointType::Wheel);
  // Every type except filter, which has no constraint to tune or break.
  constexpr unsigned C = kAllJoints & ~jointBit(JointType::Filter);
  static const std::vector<JointParam> kParams = {
      // ---- every type ----
      jp("collide_connected", "Collide Connected", kAllJoints,
         &S::collideConnected,
         "Let the two joined bodies collide with each other."),
      enabledBy(jp("force_threshold", "Break Force", C, &S::forceThreshold,
                   "The joint breaks when its force goes above this."),
                "breakable"),
      enabledBy(jp("torque_threshold", "Break Torque", C, &S::torqueThreshold,
                   "The joint breaks when its torque goes above this."),
                "breakable"),
      jp("constraint_stiffness", "Constraint Stiffness", C,
         &S::constraintStiffness,
         "How hard the joint pulls its frames back together, in Hz. Lower is "
         "softer."),
      jp("constraint_damping_ratio", "Constraint Damping Ratio", C,
         &S::constraintDampingRatio,
         "How much the joint damps that pull. 1 is critical damping."),

      // ---- distance (ahead of the shared springs, so it is not shown inside a
      // toggle's block) ----
      enabledBy(jp("restlength", "Rest Length", D, &S::restLength,
                   "Length the joint keeps."),
                "set_rest_length"),

      // ---- weld ----
      jp("enable_linear_spring", "Enable Linear Spring", W,
         &S::enableLinearSpring,
         "Off: position is held rigidly. On: a spring with Linear Stiffness."),
      enabledBy(jp("linear_stiffness", "Linear Stiffness", W | M,
                   &S::linearStiffness,
                   "Stiffness of the position spring, in Hz."),
                "enable_linear_spring"),
      enabledBy(jp("linear_damping_ratio", "Linear Damping Ratio", W | M,
                   &S::linearDampingRatio,
                   "Damping of the position spring. 1 is critical damping."),
                "enable_linear_spring"),
      jp("enable_angular_spring", "Enable Angular Spring", W,
         &S::enableAngularSpring,
         "Off: rotation is held rigidly. On: a spring with Angular Stiffness."),
      enabledBy(jp("angular_stiffness", "Angular Stiffness", W | M,
                   &S::angularStiffness,
                   "Stiffness of the rotation spring, in Hz."),
                "enable_angular_spring"),
      enabledBy(jp("angular_damping_ratio", "Angular Damping Ratio", W | M,
                   &S::angularDampingRatio,
                   "Damping of the rotation spring. 1 is critical damping."),
                "enable_angular_spring"),

      // ---- springs shared by spherical, revolute, prismatic, distance,
      // parallel ----
      jp("enable_spring", "Enable Spring", Sp | R | P | D, &S::enableSpring,
         "Pull toward the target (or rest length) with a spring."),
      enabledBy(jp("stiffness", "Stiffness", Sp | R | P | D | Pa, &S::stiffness,
                   "Spring stiffness, in Hz. Higher is stiffer."),
                "enable_spring"),
      enabledBy(jp("damping_ratio", "Damping Ratio", Sp | R | P | D | Pa,
                   &S::dampingRatio, "Spring damping. 1 is critical damping."),
                "enable_spring"),

      // ---- spherical ----
      enabledBy(jp("target_rotation", "Target Rotation", Sp, &S::targetRotation,
                   "Rotation the spring pulls toward, as XYZ Euler angles in "
                   "degrees."),
                "enable_spring"),
      jp("enable_cone_limit", "Enable Cone Limit", Sp, &S::enableConeLimit,
         "Limit how far the second body can swing away from the first anchor's "
         "z axis."),
      enabledBy(jp("cone_angle", "Cone Angle", Sp, &S::coneAngle,
                   "Half-angle of the cone, 0 to 90 degrees.", K::Angle),
                "enable_cone_limit"),
      jp("enable_twist_limit", "Enable Twist Limit", Sp, &S::enableTwistLimit,
         "Limit the twist about the second anchor's z axis."),
      enabledBy(jp("lower_twist_angle", "Lower Twist Angle", Sp,
                   &S::lowerTwistAngle,
                   "Lowest twist angle allowed, in degrees.", K::Angle),
                "enable_twist_limit"),
      enabledBy(jp("upper_twist_angle", "Upper Twist Angle", Sp,
                   &S::upperTwistAngle,
                   "Highest twist angle allowed, in degrees.", K::Angle),
                "enable_twist_limit"),
      enabledBy(
          jp("motor_velocity", "Motor Velocity", Sp, &S::motorVelocity,
             "Angular velocity the motor drives toward, in degrees per second.",
             K::Vec3Angle),
          "enable_motor"),

      // ---- motors shared ----
      jp("enable_motor", "Enable Motor", Sp | R | P | D, &S::enableMotor,
         "Drive the joint at a speed with a capped effort."),
      enabledBy(jp("max_motor_torque", "Max Motor Torque", Sp | R,
                   &S::maxMotorTorque, "Largest torque the motor may use."),
                "enable_motor"),
      enabledBy(jp("max_motor_force", "Max Motor Force", P | D,
                   &S::maxMotorForce, "Largest force the motor may use."),
                "enable_motor"),

      // ---- revolute ----
      enabledBy(jp("target_angle", "Target Angle", R, &S::targetAngle,
                   "Angle the spring pulls toward, -180 to 180 degrees.",
                   K::Angle),
                "enable_spring"),
      jp("enable_limit", "Enable Limit", R | P | D, &S::enableLimit,
         "Keep the joint between its lower and upper limits."),
      enabledBy(jp("lower_angle", "Lower Angle", R, &S::lowerAngle,
                   "Lowest angle allowed, in degrees.", K::Angle),
                "enable_limit"),
      enabledBy(jp("upper_angle", "Upper Angle", R, &S::upperAngle,
                   "Highest angle allowed, in degrees.", K::Angle),
                "enable_limit"),
      enabledBy(withAngleTypes(jp("motor_speed", "Motor Speed", R | P | D,
                                  &S::motorSpeed,
                                  "Revolute: degrees per second. Prismatic and "
                                  "distance: units per second."),
                               R),
                "enable_motor"),

      // ---- prismatic ----
      enabledBy(jp("target_translation", "Target Translation", P,
                   &S::targetTranslation,
                   "Slide position the spring pulls toward."),
                "enable_spring"),
      enabledBy(jp("lower_translation", "Lower Translation", P,
                   &S::lowerTranslation, "Lower slide limit."),
                "enable_limit"),
      enabledBy(jp("upper_translation", "Upper Translation", P,
                   &S::upperTranslation, "Upper slide limit."),
                "enable_limit"),

      // ---- distance ----
      enabledBy(jp("min_length", "Min Length", D, &S::minLength,
                   "Shortest allowed length."),
                "enable_limit"),
      enabledBy(jp("max_length", "Max Length", D, &S::maxLength,
                   "Longest allowed length."),
                "limit_max_length"),
      enabledBy(jp("max_tension_force", "Max Tension Force", D,
                   &S::maxTensionForce,
                   "Largest pulling force the spring may use."),
                "limit_spring_force"),
      enabledBy(jp("max_compression_force", "Max Compression Force", D,
                   &S::maxCompressionForce,
                   "Largest pushing force the spring may use."),
                "limit_spring_force"),

      // ---- motor joint ----
      jp("linear_velocity", "Linear Velocity", M, &S::linearVelocity,
         "Velocity of the second body relative to the first to drive toward, "
         "in units per second."),
      jp("max_velocity_force", "Max Velocity Force", M, &S::maxVelocityForce,
         "Largest force used to reach the velocity."),
      jp("angular_velocity", "Angular Velocity", M, &S::angularVelocity,
         "Angular velocity of the second body relative to the first, in "
         "degrees per second.",
         K::Vec3Angle),
      jp("max_velocity_torque", "Max Velocity Torque", M, &S::maxVelocityTorque,
         "Largest torque used to reach the angular velocity."),
      jp("max_spring_force", "Max Spring Force", M, &S::maxSpringForce,
         "Largest force the position spring may use."),
      jp("max_spring_torque", "Max Spring Torque", M, &S::maxSpringTorque,
         "Largest torque the rotation spring may use."),

      // ---- parallel ----
      enabledBy(jp("max_torque", "Max Torque", Pa, &S::maxTorque,
                   "Largest torque the spring may use."),
                "limit_torque"),

      // ---- wheel ----
      jp("enable_suspension_spring", "Enable Suspension Spring", Wh,
         &S::enableSuspensionSpring, "Spring along the first anchor's x axis."),
      enabledBy(jp("suspension_stiffness", "Suspension Stiffness", Wh,
                   &S::suspensionStiffness,
                   "Stiffness of the suspension spring, in Hz."),
                "enable_suspension_spring"),
      enabledBy(jp("suspension_damping_ratio", "Suspension Damping Ratio", Wh,
                   &S::suspensionDampingRatio,
                   "Damping of the suspension spring. 1 is critical damping."),
                "enable_suspension_spring"),
      jp("enable_suspension_limit", "Enable Suspension Limit", Wh,
         &S::enableSuspensionLimit, "Limit the suspension travel."),
      enabledBy(jp("lower_suspension_limit", "Lower Suspension Limit", Wh,
                   &S::lowerSuspensionLimit,
                   "Lowest suspension position allowed."),
                "enable_suspension_limit"),
      enabledBy(jp("upper_suspension_limit", "Upper Suspension Limit", Wh,
                   &S::upperSuspensionLimit,
                   "Highest suspension position allowed."),
                "enable_suspension_limit"),
      jp("enable_spin_motor", "Enable Spin Motor", Wh, &S::enableSpinMotor,
         "Drive the wheel's spin about the second anchor's z axis."),
      enabledBy(jp("max_spin_torque", "Max Spin Torque", Wh, &S::maxSpinTorque,
                   "Largest torque the spin motor may use."),
                "enable_spin_motor"),
      enabledBy(jp("spin_speed", "Spin Speed", Wh, &S::spinSpeed,
                   "Speed the spin motor drives toward, in degrees per second.",
                   K::Angle),
                "enable_spin_motor"),
      jp("enable_steering", "Enable Steering", Wh, &S::enableSteering,
         "Let the wheel steer about the first anchor's x axis."),
      enabledBy(jp("steering_stiffness", "Steering Stiffness", Wh,
                   &S::steeringStiffness,
                   "Stiffness of the steering spring, in Hz."),
                "enable_steering"),
      enabledBy(jp("steering_damping_ratio", "Steering Damping Ratio", Wh,
                   &S::steeringDampingRatio,
                   "Damping of the steering spring. 1 is critical damping."),
                "enable_steering"),
      enabledBy(jp("target_steering_angle", "Target Steering Angle", Wh,
                   &S::targetSteeringAngle,
                   "Angle the steering spring pulls toward, in degrees.",
                   K::Angle),
                "enable_steering"),
      enabledBy(jp("max_steering_torque", "Max Steering Torque", Wh,
                   &S::maxSteeringTorque,
                   "Largest torque the steering spring may use."),
                "enable_steering"),
      enabledBy(jp("enable_steering_limit", "Enable Steering Limit", Wh,
                   &S::enableSteeringLimit, "Limit the steering angle."),
                "enable_steering"),
      enabledBy(jp("lower_steering_limit", "Lower Steering Limit", Wh,
                   &S::lowerSteeringLimit,
                   "Lowest steering angle allowed, in degrees.", K::Angle),
                "enable_steering_limit"),
      enabledBy(jp("upper_steering_limit", "Upper Steering Limit", Wh,
                   &S::upperSteeringLimit,
                   "Highest steering angle allowed, in degrees.", K::Angle),
                "enable_steering_limit"),
  };
  return kParams;
}

// Indices into jointParams() of the settings type `t` uses, in table order.
// Built once, so per-type loops (sameJointSettings runs for every joint every
// step) skip the other types' entries. Empty for a value that is not a
// JointType.
inline const std::vector<int> &jointParamIndices(JointType t) {
  static const std::array<std::vector<int>, kJointTypeCount> kIndices = [] {
    std::array<std::vector<int>, kJointTypeCount> idx;
    const std::vector<JointParam> &params = jointParams();
    for (int ty = 0; ty < kJointTypeCount; ++ty)
      for (size_t i = 0; i < params.size(); ++i)
        if (params[i].usedBy(JointType(ty)))
          idx[static_cast<size_t>(ty)].push_back(static_cast<int>(i));
    return idx;
  }();
  static const std::vector<int> kNone;
  const int i = static_cast<int>(t);
  return (i >= 0 && i < kJointTypeCount) ? kIndices[static_cast<size_t>(i)]
                                         : kNone;
}

// The settings type `t` uses are equal (exact compare; the rest is ignored).
inline bool sameJointSettings(JointType t, const JointSettings &a,
                              const JointSettings &b) {
  const std::vector<JointParam> &params = jointParams();
  for (const int idx : jointParamIndices(t)) {
    const JointParam &p = params[static_cast<size_t>(idx)];
    switch (p.kind) {
    case JointParam::Toggle:
      if (a.*p.b != b.*p.b)
        return false;
      break;
    case JointParam::Float:
    case JointParam::Angle:
      if (a.*p.f != b.*p.f)
        return false;
      break;
    case JointParam::Vec3:
    case JointParam::Vec3Angle:
      for (int i = 0; i < 3; ++i)
        if ((a.*p.v)[i] != (b.*p.v)[i])
          return false;
      break;
    case JointParam::Rotation:
      for (int i = 0; i < 4; ++i)
        if ((a.*p.q)[i] != (b.*p.q)[i])
          return false;
      break;
    }
  }
  return true;
}

// Toggles that exist only on the constraint relationship nodes, in place of the
// "< 0: unlimited / never" values: off writes -1 into every setting naming it
// in `enabledBy`, so the settings and primitive attributes keep their meaning.
// Not settings themselves (no member pointer): only the node UI and
// SIM_Box3DConRel::settings read them; attributes, sameJointSettings and the
// SideFX mapping go through jointParams() and never see them. Default off
// (Box3D's defaults are all -1).
inline const std::vector<JointParam> &jointNodeToggles() {
  constexpr unsigned C = kAllJoints & ~jointBit(JointType::Filter);
  constexpr unsigned D = jointBit(JointType::Distance);
  constexpr unsigned Pa = jointBit(JointType::Parallel);
  auto t = [](const char *name, const char *label, unsigned types,
              const char *help, const char *enabled) {
    JointParam p;
    p.name = name;
    p.label = label;
    p.kind = JointParam::Toggle;
    p.types = types;
    p.help = help;
    p.enabledBy = enabled;
    return p;
  };
  static const std::vector<JointParam> kToggles = {
      t("breakable", "Breakable", C,
        "Let the joint break when its force or torque goes above Break Force / "
        "Torque.",
        nullptr),
      t("set_rest_length", "Set Rest Length", D,
        "Off: keep the anchors' distance when the joint is made. On: use Rest "
        "Length.",
        nullptr),
      t("limit_spring_force", "Limit Spring Force", D,
        "Cap the force the spring may use.", "enable_spring"),
      t("limit_max_length", "Limit Max Length", D,
        "Also limit the longest length. Off: only Min Length applies.",
        "enable_limit"),
      t("limit_torque", "Limit Torque", Pa,
        "Cap the torque the spring may use.", nullptr),
  };
  return kToggles;
}

// The setting or node toggle named `name`, or nullptr.
inline const JointParam *findJointParam(const char *name) {
  for (const JointParam &p : jointParams())
    if (std::strcmp(p.name, name) == 0)
      return &p;
  for (const JointParam &p : jointNodeToggles())
    if (std::strcmp(p.name, name) == 0)
      return &p;
  return nullptr;
}

// The node toggle setting `p` is written as -1 under while off (see
// jointNodeToggles), or nullptr.
inline const JointParam *jointNodeToggleOf(const JointParam &p) {
  if (!p.enabledBy)
    return nullptr;
  for (const JointParam &q : jointNodeToggles())
    if (std::strcmp(q.name, p.enabledBy) == 0)
      return &q;
  return nullptr;
}

// A parameter's slider: [min, max], each end hard or only the slider's. Houdini
// units.
struct JointRange {
  float min = 0.0f, max = 1.0f;
  bool hardMin = false, hardMax = false;
};

// The slider of setting `p` on type `t`'s node. Values the core clamps or Box3D
// rejects below 0 get a hard 0 minimum. Vectors and toggles have none (the
// slider is unused).
inline JointRange jointParamRange(const JointParam &p, JointType t) {
  struct Entry {
    const char *name;
    JointRange r;
  };
  static const Entry kRanges[] = {
      {"force_threshold", {0, 10000, true}},
      {"torque_threshold", {0, 10000, true}},
      {"constraint_stiffness", {0, 120, true}},
      {"constraint_damping_ratio", {0, 10, true}},
      {"stiffness", {0, 30, true}},
      {"damping_ratio", {0, 2, true}},
      {"linear_stiffness", {0, 30, true}},
      {"linear_damping_ratio", {0, 2, true}},
      {"angular_stiffness", {0, 30, true}},
      {"angular_damping_ratio", {0, 2, true}},
      {"suspension_stiffness", {0, 30, true}},
      {"suspension_damping_ratio", {0, 2, true}},
      {"steering_stiffness", {0, 30, true}},
      {"steering_damping_ratio", {0, 2, true}},
      {"max_motor_torque", {0, 1000, true}},
      {"max_motor_force", {0, 1000, true}},
      {"max_velocity_force", {0, 1000, true}},
      {"max_velocity_torque", {0, 1000, true}},
      {"max_spring_force", {0, 1000, true}},
      {"max_spring_torque", {0, 1000, true}},
      {"max_spin_torque", {0, 1000, true}},
      {"max_steering_torque", {0, 1000, true}},
      {"max_torque", {0, 1000, true}},
      {"max_tension_force", {0, 1000, true}},
      {"max_compression_force", {0, 1000, true}},
      {"restlength", {0, 10, true}},
      {"min_length", {0, 10, true}},
      {"max_length", {0, 10, true}},
      {"target_angle", {-180, 180, true, true}},
      {"cone_angle", {0, 90, true, true}},
      {"lower_angle", {-180, 180}},
      {"upper_angle", {-180, 180}},
      {"lower_twist_angle", {-180, 180}},
      {"upper_twist_angle", {-180, 180}},
      {"target_translation", {-1, 1}},
      {"lower_translation", {-1, 1}},
      {"upper_translation", {-1, 1}},
      {"lower_suspension_limit", {-1, 1}},
      {"upper_suspension_limit", {-1, 1}},
      {"spin_speed", {-720, 720}},
      {"target_steering_angle", {-90, 90}},
      {"lower_steering_limit", {-90, 90}},
      {"upper_steering_limit", {-90, 90}},
  };
  if (std::strcmp(p.name, "motor_speed") ==
      0) // degrees per second for some types, units for others
    return p.isAngle(t) ? JointRange{-360, 360} : JointRange{-10, 10};
  for (const Entry &e : kRanges)
    if (std::strcmp(e.name, p.name) == 0)
      return e.r;
  return JointRange{0, 10};
}

// Box3D's defaults (`box3d`, from Box3DWorld::defaultJointSettings) changed to
// what a node starts with, so turning a toggle on visibly does something:
// springs with some stiffness, motors with effort and speed, limits with a
// range instead of locking at 0..0. Only the nodes' UI defaults; joints made
// without a node keep Box3D's. Box3D units (radians). Values left at -1 sit
// behind a node toggle (jointNodeToggles), whose parameters start at the "on"
// values set here.
inline JointSettings nodeJointDefaults(JointType t, JointSettings s) {
  const float deg = kDegToRad;
  auto spring = [](float &hz, float &ratio) {
    if (hz <= 0.0f) {
      hz = 5.0f;
      ratio = 0.7f;
    }
  };
  switch (t) {
  case JointType::Weld:
    spring(s.linearStiffness, s.linearDampingRatio);
    spring(s.angularStiffness, s.angularDampingRatio);
    break;
  case JointType::Spherical:
    spring(s.stiffness, s.dampingRatio);
    s.maxMotorTorque = 100.0f;
    s.coneAngle = 45.0f * deg;
    s.lowerTwistAngle = -45.0f * deg;
    s.upperTwistAngle = 45.0f * deg;
    break;
  case JointType::Revolute:
    spring(s.stiffness, s.dampingRatio);
    s.maxMotorTorque = 100.0f;
    s.motorSpeed = 90.0f * deg;
    s.lowerAngle = -45.0f * deg;
    s.upperAngle = 45.0f * deg;
    break;
  case JointType::Prismatic:
    spring(s.stiffness, s.dampingRatio);
    s.maxMotorForce = 100.0f;
    s.motorSpeed = 1.0f;
    s.lowerTranslation = -0.5f;
    s.upperTranslation = 0.5f;
    break;
  case JointType::Distance:
    spring(s.stiffness, s.dampingRatio);
    s.maxMotorForce = 100.0f;
    s.motorSpeed = 1.0f;
    s.restLength = 1.0f;         // behind Set Rest Length
    s.maxLength = 1.0f;          // behind Limit Max Length
    s.maxTensionForce = 1000.0f; // behind Limit Spring Force
    s.maxCompressionForce = 1000.0f;
    break;
  case JointType::Parallel:
    s.maxTorque = 1000.0f; // behind Limit Torque
    break;
  case JointType::Wheel:
    s.maxSpinTorque = 100.0f;
    s.spinSpeed = 360.0f * deg;
    s.maxSteeringTorque = 100.0f;
    s.lowerSuspensionLimit = -0.25f;
    s.upperSuspensionLimit = 0.25f;
    s.lowerSteeringLimit = -30.0f * deg;
    s.upperSteeringLimit = 30.0f * deg;
    break;
  default:
    break;
  }
  if (t != JointType::Filter) {
    s.forceThreshold = 1000.0f; // behind Breakable
    s.torqueThreshold = 1000.0f;
  }
  return s;
}

// When setting `p` has no effect on joint type `t` because a toggle is off, as
// a Houdini disable-when string ("{ a == 0 } { b == 0 }": disabled if any group
// holds). Follows the chain of enabling toggles (Steering Limit needs Steering)
// and skips toggles type `t` does not have (the parallel joint's spring and the
// motor joint's springs are always on). Empty: never disabled.
inline std::string jointParamDisableWhen(const JointParam &p, JointType t) {
  std::string cond;
  for (const JointParam *q = &p; q && q->enabledBy;) {
    const JointParam *toggle = findJointParam(q->enabledBy);
    if (!toggle || !toggle->usedBy(t))
      break;
    cond += cond.empty() ? "" : " ";
    cond += std::string("{ ") + toggle->name + " == 0 }";
    q = toggle;
  }
  return cond;
}

// The settings and node toggles type `t` uses, in the order its node shows
// them: table order (a node toggle just before the first setting it gates),
// except that each setting a toggle switches on comes right after that toggle
// (and after the toggle's other settings before it, in table order), so a
// toggle disables the parameters just below it. Chains nest (Steering, its
// settings, Steering Limit, the limits). A toggle the type does not have does
// not move anything (the parallel joint's Stiffness stays where the table has
// it).
inline std::vector<const JointParam *> jointParamUiOrder(JointType t) {
  std::vector<const JointParam *> params;
  for (const JointParam &p : jointParams()) {
    if (const JointParam *nt = jointNodeToggleOf(p))
      if (std::find(params.begin(), params.end(), nt) == params.end())
        params.push_back(nt);
    params.push_back(&p);
  }
  auto parentOf = [t](const JointParam &p) -> const JointParam * {
    if (!p.enabledBy)
      return nullptr;
    const JointParam *toggle = findJointParam(p.enabledBy);
    return (toggle && toggle->usedBy(t)) ? toggle : nullptr;
  };
  std::vector<const JointParam *> order;
  // Appends `p`, then everything it switches on (recursively), in table order.
  auto emit = [&](const JointParam &p, auto &&self) -> void {
    order.push_back(&p);
    for (const JointParam *q : params)
      if (q->usedBy(t) && parentOf(*q) == &p)
        self(*q, self);
  };
  for (const JointParam *p : params)
    if (p->usedBy(t) && !parentOf(*p))
      emit(*p, emit);
  return order;
}

// Houdini side of a setting: how many values it has (1, or 3 for vectors and
// rotations).
inline int jointParamSize(const JointParam &p) {
  return (p.kind == JointParam::Toggle || p.kind == JointParam::Float ||
          p.kind == JointParam::Angle)
             ? 1
             : 3;
}

// Setting `p` of type `t` from its Houdini value(s): toggles non-zero = on,
// angles in degrees (degrees per second for speeds), rotations as Euler XYZ
// degrees through `eulerToQuat` (a Houdini call, so the core does not depend on
// Houdini's rotate order).
inline void setJointParam(JointSettings &s, const JointParam &p, JointType t,
                          const float v[3],
                          void (*eulerToQuat)(const float degrees[3],
                                              float q[4])) {
  const float scale = p.isAngle(t) ? kDegToRad : 1.0f;
  switch (p.kind) {
  case JointParam::Toggle:
    s.*p.b = v[0] != 0.0f;
    break;
  case JointParam::Float:
  case JointParam::Angle:
    s.*p.f = v[0] * scale;
    break;
  case JointParam::Vec3:
  case JointParam::Vec3Angle:
    for (int i = 0; i < 3; ++i)
      (s.*p.v)[i] = v[i] * scale;
    break;
  case JointParam::Rotation:
    eulerToQuat(v, s.*p.q);
    break;
  }
}

// The Houdini value(s) of setting `p` of type `t` (the inverse of
// setJointParam). Rotations read as zero Euler angles: only used for defaults,
// and Box3D's default targets are the identity.
inline void getJointParam(const JointSettings &s, const JointParam &p,
                          JointType t, float out[3]) {
  const float scale = p.isAngle(t) ? kRadToDeg : 1.0f;
  out[0] = out[1] = out[2] = 0.0f;
  switch (p.kind) {
  case JointParam::Toggle:
    out[0] = (s.*p.b) ? 1.0f : 0.0f;
    break;
  case JointParam::Float:
  case JointParam::Angle:
    out[0] = s.*p.f * scale;
    break;
  case JointParam::Vec3:
  case JointParam::Vec3Angle:
    for (int i = 0; i < 3; ++i)
      out[i] = (s.*p.v)[i] * scale;
    break;
  case JointParam::Rotation:
    break;
  }
}

// A joint frame in world space (an anchor point's P and orient).
struct JointFrame {
  double pos[3] = {0, 0, 0};
  float rot[4] = {0, 0, 0, 1}; // x, y, z, w
};

// What a joint reports after a step (Box3DWorld::jointFeedback).
struct JointFeedback {
  float force[3] = {0, 0, 0}; // constraint force on B (world)
  float torque[3] = {0, 0, 0};
  float linearSeparation = 0.0f;  // how far the joint is from being satisfied
  float angularSeparation = 0.0f; // radians
  // Per type (attribute names; none is also a setting name): revolute
  // current_angle / motor_torque, prismatic current_translation / current_speed
  // / motor_force, distance current_length / motor_force, spherical
  // current_cone_angle / current_twist_angle / motor_torque, wheel
  // current_spin_speed / current_steering_angle / spin_torque.
  static constexpr int kMaxExtra = 3;
  int extraCount = 0;
  const char *extraName[kMaxExtra] = {nullptr, nullptr, nullptr};
  float extra[kMaxExtra] = {0, 0, 0};
  bool extraIsAngle[kMaxExtra] = {false, false,
                                  false}; // radians (written in degrees)
};

} // namespace b3dop
