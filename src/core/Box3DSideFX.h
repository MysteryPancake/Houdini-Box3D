// Box3DSideFX.h
//
// SideFX's own constraint relationships (Glue, Hard, Spring, Bullet Soft, Cone Twist), mapped onto
// Box3D joints. Approximate: Box3D has no glue impulses, no CFM / ERP, no elliptical cones. The
// Houdini side reads each data's options and the primitive overrides into SideFXSettings (through
// sideFXParams(), so the names live in one place); mapSideFX() picks the Box3D joint and its
// settings, and sideFXFrameOffsets() says where its frames sit relative to the two anchors.
// Houdini-free, covered by tests/core_sidefx.cpp.
#pragma once

#include <cstring>
#include <vector>

#include "Box3DJoints.h"

namespace b3dop {

enum class SideFXType : int
{
    Glue = 0,       // SIM_ConRelGlue        -> rigid weld that holds the pieces' offset, breaks on strength
    Hard,           // SIM_ConRelHard        -> weld / spherical / rigid distance / motor (by constraint_type)
    Spring,         // SIM_ConRelSpring      -> like Soft, stiffness and damping converted from the masses
    Soft,           // SIM_ConRelBulletSoft  -> motor joint springs, or a sprung distance joint
    ConeTwist,      // SIM_ConRelConeTwist   -> spherical with cone and twist limits
};
constexpr int kSideFXTypeCount = 5;

constexpr unsigned sideFXBit(SideFXType t) { return 1u << unsigned(t); }

// The SIM data type of each (what SIM_Data::getDataType() returns).
inline const char *
sideFXDataType(SideFXType t)
{
    static const char *const kNames[kSideFXTypeCount] = { "SIM_ConRelGlue", "SIM_ConRelHard", "SIM_ConRelSpring",
                                                           "SIM_ConRelBulletSoft", "SIM_ConRelConeTwist" };
    const int i = int(t);
    return (i >= 0 && i < kSideFXTypeCount) ? kNames[i] : "";
}

inline bool
sideFXTypeFromDataType(const char *dataType, SideFXType &out)
{
    if (!dataType)
        return false;
    for (int i = 0; i < kSideFXTypeCount; ++i)
        if (std::strcmp(dataType, sideFXDataType(SideFXType(i))) == 0)
        {
            out = SideFXType(i);
            return true;
        }
    return false;
}

// The Constraint Network's `constraint_type` primitive attribute.
enum class ConstraintMode : int
{
    All = 0,        // position and rotation (also when the attribute is missing or unknown)
    Position,
    Rotation,
};

inline ConstraintMode
constraintModeFromString(const char *s)
{
    if (s && std::strcmp(s, "position") == 0)
        return ConstraintMode::Position;
    if (s && std::strcmp(s, "rotation") == 0)
        return ConstraintMode::Rotation;
    return ConstraintMode::All;
}

// Every option read from the SideFX data, under its own units (degrees for angles). Defaults are
// Houdini's, used only when the data lacks the option.
struct SideFXSettings
{
    float strength          = 10000.0f;     // glue: impulse to break, < 0 never; spring: Hooke's constant
    float restLength        = 1.0f;         // hard, spring, soft
    int   disableCollisions = 0;            // hard, spring, soft, cone twist
    float damping           = 0.1f;         // spring: absolute (force per speed)

    float stiffness           = 10.0f;      // soft: Hz
    float dampingRatio        = 1.0f;
    int   enableAngular       = 0;          // soft: angular part uses its own stiffness / damping
    float angularStiffness    = 10.0f;
    float angularDampingRatio = 1.0f;

    float maxTwist         = 0.0f;          // cone twist, degrees
    float maxOutRotation   = 180.0f;
    float maxUpRotation    = 180.0f;
    float goalTwistAxis[3]        = { 1, 0, 0 };
    float goalUpAxis[3]           = { 0, 1, 0 };
    float constrainedTwistAxis[3] = { 1, 0, 0 };
    float constrainedUpAxis[3]    = { 0, 1, 0 };
};

// Houdini's defaults for type `t` (only strength differs between types).
inline SideFXSettings
defaultSideFXSettings(SideFXType t)
{
    SideFXSettings s;
    if (t == SideFXType::Spring)
        s.strength = 1.0f;
    return s;
}

// One option: its name on the data (also the primitive attribute that modifies it), kind and the
// types that have it.
struct SideFXParam
{
    enum Kind
    {
        Float,      // primitive attribute multiplies
        Int,        // toggle / int: primitive attribute replaces
        Vec3,       // primitive attribute replaces
    };
    const char *name  = "";
    Kind        kind  = Float;
    unsigned    types = 0;                          // sideFXBit mask
    float SideFXSettings::*f       = nullptr;
    int   SideFXSettings::*i       = nullptr;
    float (SideFXSettings::*v)[3]  = nullptr;

    bool usedBy(SideFXType t) const { return (types & sideFXBit(t)) != 0; }
};

inline const std::vector<SideFXParam> &
sideFXParams()
{
    using S = SideFXSettings;
    constexpr unsigned G = sideFXBit(SideFXType::Glue), H = sideFXBit(SideFXType::Hard),
                       Sp = sideFXBit(SideFXType::Spring), So = sideFXBit(SideFXType::Soft),
                       C = sideFXBit(SideFXType::ConeTwist);
    auto f = [](const char *n, unsigned t, float S::*m) { SideFXParam p; p.name = n; p.kind = SideFXParam::Float; p.types = t; p.f = m; return p; };
    auto i = [](const char *n, unsigned t, int S::*m)   { SideFXParam p; p.name = n; p.kind = SideFXParam::Int;   p.types = t; p.i = m; return p; };
    auto v = [](const char *n, unsigned t, float (S::*m)[3]) { SideFXParam p; p.name = n; p.kind = SideFXParam::Vec3; p.types = t; p.v = m; return p; };
    static const std::vector<SideFXParam> kParams = {
        f("strength",               G | Sp,          &S::strength),
        f("restlength",             H | Sp | So,     &S::restLength),
        i("disablecollisions",      H | Sp | So | C, &S::disableCollisions),
        f("damping",                Sp,              &S::damping),
        f("stiffness",              So,              &S::stiffness),
        f("dampingratio",           So,              &S::dampingRatio),
        i("enableangular",          So,              &S::enableAngular),
        f("angularstiffness",       So,              &S::angularStiffness),
        f("angulardampingratio",    So,              &S::angularDampingRatio),
        f("max_twist",              C,               &S::maxTwist),
        f("max_out_rotation",       C,               &S::maxOutRotation),
        f("max_up_rotation",        C,               &S::maxUpRotation),
        v("goal_twist_axis",        C,               &S::goalTwistAxis),
        v("goal_up_axis",           C,               &S::goalUpAxis),
        v("constrained_twist_axis", C,               &S::constrainedTwistAxis),
        v("constrained_up_axis",    C,               &S::constrainedUpAxis),
    };
    return kParams;
}

// The options type `t` uses are equal (exact compare).
inline bool
sameSideFXSettings(SideFXType t, const SideFXSettings &a, const SideFXSettings &b)
{
    for (const SideFXParam &p : sideFXParams())
    {
        if (!p.usedBy(t))
            continue;
        switch (p.kind)
        {
            case SideFXParam::Float: if (a.*p.f != b.*p.f) return false; break;
            case SideFXParam::Int:   if (a.*p.i != b.*p.i) return false; break;
            case SideFXParam::Vec3:
                for (int k = 0; k < 3; ++k)
                    if ((a.*p.v)[k] != (b.*p.v)[k]) return false;
                break;
        }
    }
    return true;
}

// A body as the mapping needs it (Spring's conversion to Hz). Static, kinematic and the world:
// dynamic = false.
struct SideFXBody
{
    bool  dynamic = false;
    float mass    = 0.0f;
    float inertia = 0.0f;       // mean of the principal moments
};

struct SideFXContext
{
    float      timeStep = 1.0f / 24.0f;     // glue: strength (an impulse) / timeStep = break force
    SideFXBody a, b;
};

// The Box3D joint for one primitive, and how its frames sit relative to the anchors.
struct SideFXJoint
{
    JointType     type = JointType::Weld;
    JointSettings settings;

    // Each frame is its anchor's frame turned by this rotation (in the anchor's own axes) ...
    float rotA[4] = { 0, 0, 0, 1 };
    float rotB[4] = { 0, 0, 0, 1 };
    // ... and, with a separation >= 0, moved toward the other along the line between the anchors so
    // the two frames meet when the anchors are that far apart. holdSeparation: the anchors' distance
    // when the joint is made (the joint holds the pieces where they are).
    float separation     = -1.0f;
    bool  holdSeparation = false;
};

// Below this a rest length counts as zero.
constexpr float kSideFXMinRestLength = 1e-6f;

// Implemented in Box3DSideFX.cpp (needs Box3D's joint defaults).
SideFXJoint mapSideFX(SideFXType t, const SideFXSettings &s, ConstraintMode mode, const SideFXContext &c);

// The joint's frames relative to the anchors' frames (world: the anchors' P / orient): adjA / adjB
// are offsets in each anchor's own axes, so joint frame = applyFrameOffset(anchor, adj).
void sideFXFrameOffsets(const SideFXJoint &j, const JointFrame &anchorA, const JointFrame &anchorB,
                        JointFrame &adjA, JointFrame &adjB);

// joint = anchor * adj (position offset in the anchor's axes, then its rotation).
JointFrame applyFrameOffset(const JointFrame &anchor, const JointFrame &adj);
// The inverse: the anchor a joint frame came from.
JointFrame removeFrameOffset(const JointFrame &joint, const JointFrame &adj);

// True for an identity offset (no rotation, no translation).
inline bool
isIdentityOffset(const JointFrame &adj)
{
    return adj.pos[0] == 0.0 && adj.pos[1] == 0.0 && adj.pos[2] == 0.0 && adj.rot[0] == 0.0f &&
           adj.rot[1] == 0.0f && adj.rot[2] == 0.0f && adj.rot[3] == 1.0f;
}

} // namespace b3dop
