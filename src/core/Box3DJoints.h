// Box3DJoints.h
//
// Houdini-free description of Box3D joints: the joint types, their settings, and one table of
// parameters (attribute / parameter names, kinds, which types use them) that the Houdini side
// uses for the Box3D constraint relationship nodes, primitive attributes and change detection.
// Box3DWorld creates the joints. Settings are in Box3D units: radians, radians per second, Hz.
#pragma once

#include <array>
#include <cstring>
#include <string>
#include <vector>

#include "Box3DMath.h"       // kDegToRad, kRadToDeg

namespace b3dop {

// The Box3D joints that make sense for rigid bodies (no character-mover joints).
enum class JointType : int
{
    Weld = 0,       // rigid, or soft with linear / angular springs
    Spherical,      // ball and socket: cone and twist limits, spring, motor
    Revolute,       // hinge about frame z: limits, spring, motor
    Prismatic,      // slider along frame A's x: limits, spring, motor
    Distance,       // keeps two points at a distance: length range, spring, motor
    Motor,          // drives B relative to A by velocity and springs, with capped force / torque
    Parallel,       // keeps the frames' z axes parallel with a spring
    Wheel,          // wheel on a chassis: suspension, spin motor, steering
    Filter,         // no constraint: the two bodies do not collide
};
constexpr int kJointTypeCount = 9;

constexpr unsigned jointBit(JointType t) { return 1u << unsigned(t); }
constexpr unsigned kAllJoints = (1u << kJointTypeCount) - 1u;

inline const char *
jointTypeName(JointType t)
{
    static const char *const kNames[kJointTypeCount] = { "weld", "spherical", "revolute", "prismatic", "distance",
                                                          "motor", "parallel", "wheel", "filter" };
    const int i = int(t);
    return (i >= 0 && i < kJointTypeCount) ? kNames[i] : "";
}

inline const char *
jointTypeLabel(JointType t)
{
    static const char *const kLabels[kJointTypeCount] = { "Weld", "Spherical", "Revolute", "Prismatic", "Distance",
                                                           "Motor", "Parallel", "Wheel", "Filter" };
    const int i = int(t);
    return (i >= 0 && i < kJointTypeCount) ? kLabels[i] : "";
}

// Every setting of every joint type, in Box3D units. Caps marked "< 0: unlimited" use a negative
// value where Box3D uses FLT_MAX. Box3DWorld::defaultJointSettings gives Box3D's defaults per type.
struct JointSettings
{
    // Every type.
    bool  collideConnected       = false;
    float forceThreshold         = -1.0f;   // breaks above this force; < 0: never
    float torqueThreshold        = -1.0f;   // breaks above this torque; < 0: never
    float constraintStiffness    = 60.0f;   // Hz: how hard the joint pulls back to its frames
    float constraintDampingRatio = 2.0f;

    // Springs (spherical, revolute, prismatic, distance; parallel without the toggle).
    bool  enableSpring = false;
    float stiffness    = 0.0f;              // Hz
    float dampingRatio = 0.0f;

    // Limits.
    bool  enableLimit       = false;        // revolute, prismatic, distance
    float lowerAngle        = 0.0f;         // revolute
    float upperAngle        = 0.0f;
    float lowerTranslation  = 0.0f;         // prismatic
    float upperTranslation  = 0.0f;
    float minLength         = 0.0f;         // distance
    float maxLength         = -1.0f;        // distance; < 0: unlimited

    // Motors.
    bool  enableMotor       = false;        // spherical, revolute, prismatic, distance
    float maxMotorTorque    = 0.0f;         // spherical, revolute
    float maxMotorForce     = 0.0f;         // prismatic, distance
    float motorSpeed        = 0.0f;         // revolute (rad/s); prismatic, distance (units/s)
    float motorVelocity[3]  = { 0, 0, 0 };  // spherical (rad/s)

    // Spring targets.
    float targetAngle       = 0.0f;         // revolute
    float targetTranslation = 0.0f;         // prismatic
    float targetRotation[4] = { 0, 0, 0, 1 };   // spherical, quaternion x, y, z, w

    // Spherical limits.
    bool  enableConeLimit   = false;
    float coneAngle         = 0.0f;         // 0 .. pi/2, about frame A's z
    bool  enableTwistLimit  = false;
    float lowerTwistAngle   = 0.0f;         // about frame B's z
    float upperTwistAngle   = 0.0f;

    // Weld springs (off: rigid in that part) and the motor joint's springs.
    bool  enableLinearSpring  = false;      // weld
    bool  enableAngularSpring = false;      // weld
    float linearStiffness     = 0.0f;       // weld, motor (Hz)
    float linearDampingRatio  = 0.0f;
    float angularStiffness    = 0.0f;       // weld, motor (Hz)
    float angularDampingRatio = 0.0f;

    // Distance.
    float restLength           = -1.0f;     // < 0: the anchors' distance when the joint is made
    float maxTensionForce      = -1.0f;     // spring: < 0: unlimited
    float maxCompressionForce  = -1.0f;     // spring: < 0: unlimited

    // Motor joint.
    float linearVelocity[3]  = { 0, 0, 0 };
    float maxVelocityForce   = 0.0f;
    float angularVelocity[3] = { 0, 0, 0 }; // rad/s
    float maxVelocityTorque  = 0.0f;
    float maxSpringForce     = 0.0f;
    float maxSpringTorque    = 0.0f;

    // Parallel.
    float maxTorque = -1.0f;                // < 0: unlimited

    // Wheel.
    bool  enableSuspensionSpring = false;
    float suspensionStiffness    = 0.0f;    // Hz
    float suspensionDampingRatio = 0.0f;
    bool  enableSuspensionLimit  = false;
    float lowerSuspensionLimit   = 0.0f;
    float upperSuspensionLimit   = 0.0f;
    bool  enableSpinMotor        = false;
    float maxSpinTorque          = 0.0f;
    float spinSpeed              = 0.0f;    // rad/s
    bool  enableSteering         = false;
    float steeringStiffness      = 0.0f;    // Hz
    float steeringDampingRatio   = 0.0f;
    float targetSteeringAngle    = 0.0f;
    float maxSteeringTorque      = 0.0f;
    bool  enableSteeringLimit    = false;
    float lowerSteeringLimit     = 0.0f;
    float upperSteeringLimit     = 0.0f;
};

// One setting as Houdini sees it: attribute name (also the parameter name on the type's Box3D
// constraint relationship node), label, kind and the joint types that use it.
struct JointParam
{
    enum Kind
    {
        Toggle,         // int attribute / toggle
        Float,          // float, Box3D units
        Angle,          // float, degrees in Houdini (degrees per second for speeds), radians here
        Vec3,           // vector
        Vec3Angle,      // vector of angles / angular speeds: degrees in Houdini
        Rotation,       // Euler angles in degrees (XYZ) in Houdini, a quaternion here
    };

    const char *name  = "";
    const char *label = "";
    Kind        kind  = Float;
    unsigned    types = 0;                  // jointBit mask
    const char *help  = "";

    bool  JointSettings::*b    = nullptr;   // Toggle
    float JointSettings::*f    = nullptr;   // Float, Angle
    float (JointSettings::*v)[3] = nullptr; // Vec3, Vec3Angle
    float (JointSettings::*q)[4] = nullptr; // Rotation

    unsigned    angleTypes = 0;             // Float only: types for which the value is an angle (speed)
    const char *enabledBy  = nullptr;       // name of the toggle that switches this setting on, if any

    bool usedBy(JointType t) const { return (types & jointBit(t)) != 0; }
    // Degrees in Houdini, radians here, for joint type `t`.
    bool isAngle(JointType t) const
    {
        return kind == Angle || kind == Vec3Angle || kind == Rotation || (angleTypes & jointBit(t)) != 0;
    }
};

namespace detail {
inline JointParam
jp(const char *name, const char *label, unsigned types, bool JointSettings::*m, const char *help)
{
    JointParam p; p.name = name; p.label = label; p.kind = JointParam::Toggle; p.types = types; p.b = m; p.help = help;
    return p;
}
inline JointParam
jp(const char *name, const char *label, unsigned types, float JointSettings::*m, const char *help,
   JointParam::Kind kind = JointParam::Float)
{
    JointParam p; p.name = name; p.label = label; p.kind = kind; p.types = types; p.f = m; p.help = help;
    return p;
}
inline JointParam
jp(const char *name, const char *label, unsigned types, float (JointSettings::*m)[3], const char *help,
   JointParam::Kind kind = JointParam::Vec3)
{
    JointParam p; p.name = name; p.label = label; p.kind = kind; p.types = types; p.v = m; p.help = help;
    return p;
}
inline JointParam
withAngleTypes(JointParam p, unsigned angleTypes)
{
    p.angleTypes = angleTypes;
    return p;
}
inline JointParam
enabledBy(JointParam p, const char *toggle)
{
    p.enabledBy = toggle;
    return p;
}
inline JointParam
jp(const char *name, const char *label, unsigned types, float (JointSettings::*m)[4], const char *help)
{
    JointParam p; p.name = name; p.label = label; p.kind = JointParam::Rotation; p.types = types; p.q = m; p.help = help;
    return p;
}
} // namespace detail

// Every setting, common ones first (types == kAllJoints), then grouped by joint type.
inline const std::vector<JointParam> &
jointParams()
{
    using detail::enabledBy;
    using detail::jp;
    using detail::withAngleTypes;
    using K = JointParam;
    using S = JointSettings;
    constexpr unsigned W = jointBit(JointType::Weld), Sp = jointBit(JointType::Spherical),
                       R = jointBit(JointType::Revolute), P = jointBit(JointType::Prismatic),
                       D = jointBit(JointType::Distance), M = jointBit(JointType::Motor),
                       Pa = jointBit(JointType::Parallel), Wh = jointBit(JointType::Wheel);
    // Every type except filter, which has no constraint to tune or break.
    constexpr unsigned C = kAllJoints & ~jointBit(JointType::Filter);
    static const std::vector<JointParam> kParams = {
        // ---- every type ----
        jp("collide_connected", "Collide Connected", kAllJoints, &S::collideConnected,
           "Let the two joined bodies collide with each other."),
        jp("force_threshold", "Break Force", C, &S::forceThreshold,
           "The joint breaks when its force goes above this. Negative never breaks."),
        jp("torque_threshold", "Break Torque", C, &S::torqueThreshold,
           "The joint breaks when its torque goes above this. Negative never breaks."),
        jp("constraint_stiffness", "Constraint Stiffness", C, &S::constraintStiffness,
           "How hard the joint pulls its frames back together, in Hz. Lower is softer."),
        jp("constraint_damping_ratio", "Constraint Damping Ratio", C, &S::constraintDampingRatio,
           "How much the joint damps that pull. 1 is critical damping."),

        // ---- weld ----
        jp("enable_linear_spring", "Linear Spring", W, &S::enableLinearSpring,
           "Off: position is held rigidly. On: a spring with Linear Stiffness."),
        enabledBy(jp("linear_stiffness", "Linear Stiffness", W | M, &S::linearStiffness,
                     "Stiffness of the position spring, in Hz."), "enable_linear_spring"),
        enabledBy(jp("linear_damping_ratio", "Linear Damping Ratio", W | M, &S::linearDampingRatio,
                     "Damping of the position spring. 1 is critical damping."), "enable_linear_spring"),
        jp("enable_angular_spring", "Angular Spring", W, &S::enableAngularSpring,
           "Off: rotation is held rigidly. On: a spring with Angular Stiffness."),
        enabledBy(jp("angular_stiffness", "Angular Stiffness", W | M, &S::angularStiffness,
                     "Stiffness of the rotation spring, in Hz."), "enable_angular_spring"),
        enabledBy(jp("angular_damping_ratio", "Angular Damping Ratio", W | M, &S::angularDampingRatio,
                     "Damping of the rotation spring. 1 is critical damping."), "enable_angular_spring"),

        // ---- springs shared by spherical, revolute, prismatic, distance, parallel ----
        jp("enable_spring", "Spring", Sp | R | P | D, &S::enableSpring,
           "Pull toward the target (or rest length) with a spring."),
        enabledBy(jp("stiffness", "Stiffness", Sp | R | P | D | Pa, &S::stiffness, "Spring stiffness, in Hz. Higher is stiffer."), "enable_spring"),
        enabledBy(jp("damping_ratio", "Damping Ratio", Sp | R | P | D | Pa, &S::dampingRatio,
                     "Spring damping. 1 is critical damping."), "enable_spring"),

        // ---- spherical ----
        enabledBy(jp("target_rotation", "Target Rotation", Sp, &S::targetRotation,
                     "Rotation the spring pulls toward, as XYZ Euler angles in degrees."), "enable_spring"),
        jp("enable_cone_limit", "Cone Limit", Sp, &S::enableConeLimit, "Limit how far B can swing away from A's z axis."),
        enabledBy(jp("cone_angle", "Cone Angle", Sp, &S::coneAngle, "Half-angle of the cone, 0 to 90 degrees.", K::Angle), "enable_cone_limit"),
        jp("enable_twist_limit", "Twist Limit", Sp, &S::enableTwistLimit, "Limit the twist about B's z axis."),
        enabledBy(jp("lower_twist_angle", "Lower Twist Angle", Sp, &S::lowerTwistAngle, "Lowest twist angle allowed, in degrees.", K::Angle), "enable_twist_limit"),
        enabledBy(jp("upper_twist_angle", "Upper Twist Angle", Sp, &S::upperTwistAngle, "Highest twist angle allowed, in degrees.", K::Angle), "enable_twist_limit"),
        enabledBy(jp("motor_velocity", "Motor Velocity", Sp, &S::motorVelocity,
                     "Angular velocity the motor drives toward, in degrees per second.", K::Vec3Angle), "enable_motor"),

        // ---- motors shared ----
        jp("enable_motor", "Motor", Sp | R | P | D, &S::enableMotor, "Drive the joint at a speed with a capped effort."),
        enabledBy(jp("max_motor_torque", "Max Motor Torque", Sp | R, &S::maxMotorTorque, "Largest torque the motor may use."), "enable_motor"),
        enabledBy(jp("max_motor_force", "Max Motor Force", P | D, &S::maxMotorForce, "Largest force the motor may use."), "enable_motor"),

        // ---- revolute ----
        enabledBy(jp("target_angle", "Target Angle", R, &S::targetAngle,
                     "Angle the spring pulls toward, -180 to 180 degrees.", K::Angle), "enable_spring"),
        jp("enable_limit", "Limit", R | P | D, &S::enableLimit, "Keep the joint between its lower and upper limits."),
        enabledBy(jp("lower_angle", "Lower Angle", R, &S::lowerAngle, "Lowest angle allowed, in degrees.", K::Angle), "enable_limit"),
        enabledBy(jp("upper_angle", "Upper Angle", R, &S::upperAngle, "Highest angle allowed, in degrees.", K::Angle), "enable_limit"),
        enabledBy(withAngleTypes(jp("motor_speed", "Motor Speed", R | P | D, &S::motorSpeed,
                                    "Revolute: degrees per second. Prismatic and distance: units per second."), R), "enable_motor"),

        // ---- prismatic ----
        enabledBy(jp("target_translation", "Target Translation", P, &S::targetTranslation, "Slide position the spring pulls toward."), "enable_spring"),
        enabledBy(jp("lower_translation", "Lower Translation", P, &S::lowerTranslation, "Lower slide limit."), "enable_limit"),
        enabledBy(jp("upper_translation", "Upper Translation", P, &S::upperTranslation, "Upper slide limit."), "enable_limit"),

        // ---- distance ----
        jp("restlength", "Rest Length", D, &S::restLength,
           "Length the joint keeps. Negative uses the anchors' distance when the joint is made."),
        enabledBy(jp("min_length", "Min Length", D, &S::minLength, "Shortest allowed length (with Limit on)."), "enable_limit"),
        enabledBy(jp("max_length", "Max Length", D, &S::maxLength, "Longest allowed length (with Limit on). Negative is unlimited."), "enable_limit"),
        enabledBy(jp("max_tension_force", "Max Tension Force", D, &S::maxTensionForce,
                     "Largest pulling force the spring may use. Negative is unlimited."), "enable_spring"),
        enabledBy(jp("max_compression_force", "Max Compression Force", D, &S::maxCompressionForce,
                     "Largest pushing force the spring may use. Negative is unlimited."), "enable_spring"),

        // ---- motor joint ----
        jp("linear_velocity", "Linear Velocity", M, &S::linearVelocity, "Velocity of B relative to A to drive toward."),
        jp("max_velocity_force", "Max Velocity Force", M, &S::maxVelocityForce, "Largest force used to reach the velocity."),
        jp("angular_velocity", "Angular Velocity", M, &S::angularVelocity,
           "Angular velocity of B relative to A to drive toward, in degrees per second.", K::Vec3Angle),
        jp("max_velocity_torque", "Max Velocity Torque", M, &S::maxVelocityTorque,
           "Largest torque used to reach the angular velocity."),
        jp("max_spring_force", "Max Spring Force", M, &S::maxSpringForce, "Largest force the position spring may use."),
        jp("max_spring_torque", "Max Spring Torque", M, &S::maxSpringTorque, "Largest torque the rotation spring may use."),

        // ---- parallel ----
        jp("max_torque", "Max Torque", Pa, &S::maxTorque, "Largest torque the spring may use. Negative is unlimited."),

        // ---- wheel ----
        jp("enable_suspension_spring", "Suspension Spring", Wh, &S::enableSuspensionSpring,
           "Spring along frame A's x axis."),
        enabledBy(jp("suspension_stiffness", "Suspension Stiffness", Wh, &S::suspensionStiffness, "Stiffness of the suspension spring, in Hz."), "enable_suspension_spring"),
        enabledBy(jp("suspension_damping_ratio", "Suspension Damping Ratio", Wh, &S::suspensionDampingRatio,
                     "Damping of the suspension spring. 1 is critical damping."), "enable_suspension_spring"),
        jp("enable_suspension_limit", "Suspension Limit", Wh, &S::enableSuspensionLimit, "Limit the suspension travel."),
        enabledBy(jp("lower_suspension_limit", "Lower Suspension Limit", Wh, &S::lowerSuspensionLimit, "Lowest suspension position allowed."), "enable_suspension_limit"),
        enabledBy(jp("upper_suspension_limit", "Upper Suspension Limit", Wh, &S::upperSuspensionLimit, "Highest suspension position allowed."), "enable_suspension_limit"),
        jp("enable_spin_motor", "Spin Motor", Wh, &S::enableSpinMotor, "Drive the wheel's spin about frame B's z axis."),
        enabledBy(jp("max_spin_torque", "Max Spin Torque", Wh, &S::maxSpinTorque, "Largest torque the spin motor may use."), "enable_spin_motor"),
        enabledBy(jp("spin_speed", "Spin Speed", Wh, &S::spinSpeed, "Speed the spin motor drives toward, in degrees per second.", K::Angle), "enable_spin_motor"),
        jp("enable_steering", "Steering", Wh, &S::enableSteering, "Let the wheel steer about frame A's x axis."),
        enabledBy(jp("steering_stiffness", "Steering Stiffness", Wh, &S::steeringStiffness, "Stiffness of the steering spring, in Hz."), "enable_steering"),
        enabledBy(jp("steering_damping_ratio", "Steering Damping Ratio", Wh, &S::steeringDampingRatio, "Damping of the steering spring. 1 is critical damping."), "enable_steering"),
        enabledBy(jp("target_steering_angle", "Target Steering Angle", Wh, &S::targetSteeringAngle, "Angle the steering spring pulls toward, in degrees.", K::Angle), "enable_steering"),
        enabledBy(jp("max_steering_torque", "Max Steering Torque", Wh, &S::maxSteeringTorque, "Largest torque the steering spring may use."), "enable_steering"),
        enabledBy(jp("enable_steering_limit", "Steering Limit", Wh, &S::enableSteeringLimit, "Limit the steering angle."), "enable_steering"),
        enabledBy(jp("lower_steering_limit", "Lower Steering Limit", Wh, &S::lowerSteeringLimit, "Lowest steering angle allowed, in degrees.", K::Angle), "enable_steering_limit"),
        enabledBy(jp("upper_steering_limit", "Upper Steering Limit", Wh, &S::upperSteeringLimit, "Highest steering angle allowed, in degrees.", K::Angle), "enable_steering_limit"),
    };
    return kParams;
}

// Indices into jointParams() of the settings type `t` uses, in table order. Built once, so per-type
// loops (sameJointSettings runs for every joint every step) skip the other types' entries.
// Empty for a value that is not a JointType.
inline const std::vector<int> &
jointParamIndices(JointType t)
{
    static const std::array<std::vector<int>, kJointTypeCount> kIndices = [] {
        std::array<std::vector<int>, kJointTypeCount> idx;
        const std::vector<JointParam> &params = jointParams();
        for (int ty = 0; ty < kJointTypeCount; ++ty)
            for (size_t i = 0; i < params.size(); ++i)
                if (params[i].usedBy(JointType(ty)))
                    idx[size_t(ty)].push_back(int(i));
        return idx;
    }();
    static const std::vector<int> kNone;
    const int i = int(t);
    return (i >= 0 && i < kJointTypeCount) ? kIndices[size_t(i)] : kNone;
}

// The settings type `t` uses are equal (exact compare; the rest is ignored).
inline bool
sameJointSettings(JointType t, const JointSettings &a, const JointSettings &b)
{
    const std::vector<JointParam> &params = jointParams();
    for (const int idx : jointParamIndices(t))
    {
        const JointParam &p = params[size_t(idx)];
        switch (p.kind)
        {
            case JointParam::Toggle:
                if (a.*p.b != b.*p.b) return false;
                break;
            case JointParam::Float:
            case JointParam::Angle:
                if (a.*p.f != b.*p.f) return false;
                break;
            case JointParam::Vec3:
            case JointParam::Vec3Angle:
                for (int i = 0; i < 3; ++i)
                    if ((a.*p.v)[i] != (b.*p.v)[i]) return false;
                break;
            case JointParam::Rotation:
                for (int i = 0; i < 4; ++i)
                    if ((a.*p.q)[i] != (b.*p.q)[i]) return false;
                break;
        }
    }
    return true;
}

// The setting named `name` in jointParams(), or nullptr.
inline const JointParam *
findJointParam(const char *name)
{
    for (const JointParam &p : jointParams())
        if (std::strcmp(p.name, name) == 0)
            return &p;
    return nullptr;
}

// When setting `p` has no effect on joint type `t` because a toggle is off, as a Houdini
// disable-when string ("{ a == 0 } { b == 0 }": disabled if any group holds). Follows the chain
// of enabling toggles (Steering Limit needs Steering) and skips toggles type `t` does not have
// (the parallel joint's spring and the motor joint's springs are always on). Empty: never disabled.
inline std::string
jointParamDisableWhen(const JointParam &p, JointType t)
{
    std::string cond;
    for (const JointParam *q = &p; q && q->enabledBy; )
    {
        const JointParam *toggle = findJointParam(q->enabledBy);
        if (!toggle || !toggle->usedBy(t))
            break;
        cond += cond.empty() ? "" : " ";
        cond += std::string("{ ") + toggle->name + " == 0 }";
        q = toggle;
    }
    return cond;
}

// Houdini side of a setting: how many values it has (1, or 3 for vectors and rotations).
inline int
jointParamSize(const JointParam &p)
{
    return (p.kind == JointParam::Toggle || p.kind == JointParam::Float || p.kind == JointParam::Angle) ? 1 : 3;
}

// Setting `p` of type `t` from its Houdini value(s): toggles non-zero = on, angles in degrees
// (degrees per second for speeds), rotations as Euler XYZ degrees through `eulerToQuat` (a Houdini
// call, so the core does not depend on Houdini's rotate order).
inline void
setJointParam(JointSettings &s, const JointParam &p, JointType t, const float v[3],
              void (*eulerToQuat)(const float degrees[3], float q[4]))
{
    const float scale = p.isAngle(t) ? kDegToRad : 1.0f;
    switch (p.kind)
    {
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

// The Houdini value(s) of setting `p` of type `t` (the inverse of setJointParam). Rotations read
// as zero Euler angles: only used for defaults, and Box3D's default targets are the identity.
inline void
getJointParam(const JointSettings &s, const JointParam &p, JointType t, float out[3])
{
    const float scale = p.isAngle(t) ? kRadToDeg : 1.0f;
    out[0] = out[1] = out[2] = 0.0f;
    switch (p.kind)
    {
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
struct JointFrame
{
    double pos[3] = { 0, 0, 0 };
    float  rot[4] = { 0, 0, 0, 1 };     // x, y, z, w
};

// What a joint reports after a step (Box3DWorld::jointFeedback).
struct JointFeedback
{
    float force[3]          = { 0, 0, 0 };  // constraint force on B (world)
    float torque[3]         = { 0, 0, 0 };
    float linearSeparation  = 0.0f;         // how far the joint is from being satisfied
    float angularSeparation = 0.0f;         // radians
    // Per type (attribute names; none is also a setting name): revolute current_angle /
    // motor_torque, prismatic current_translation / current_speed / motor_force, distance
    // current_length / motor_force, spherical current_cone_angle / current_twist_angle /
    // motor_torque, wheel current_spin_speed / current_steering_angle / spin_torque.
    static constexpr int kMaxExtra = 3;
    int         extraCount = 0;
    const char *extraName[kMaxExtra]    = { nullptr, nullptr, nullptr };
    float       extra[kMaxExtra]        = { 0, 0, 0 };
    bool        extraIsAngle[kMaxExtra] = { false, false, false };   // radians (written in degrees)
};

} // namespace b3dop
