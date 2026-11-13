// SIM_Box3DData.cpp
#include "SIM_Box3DData.h"

#include "SIM_Box3DParms.h"

#include <SIM/SIM_DopDescription.h>

SIM_Box3DData::SIM_Box3DData(const SIM_DataFactory *factory)
    : BaseClass(factory),
      SIM_OptionsUser(this)
{
}

SIM_Box3DData::~SIM_Box3DData()
{
}

const SIM_DopDescription *
SIM_Box3DData::getDescription()
{
    // Parameter names match the point attributes that override them. Box3D asserts on negative
    // values, so ranges have a hard minimum of 0 (larger values than the slider can be typed).
    using R = Box3DParms::Range;
    static Box3DParms p;
    static const PRM_Template *templates = p
        .tab("Shape")
        .menu("defaultshape", "Default Shape", 0,
              { "Box", "Sphere", "Convex Hull", "Concave / Compound (Hull per Piece)", "Capsule", "None (No Collision)" },
              "Collision shape for points without a bullet_georep attribute. Concave builds one convex hull per piece.")
        // Off: box / sphere / capsule from bullet_primS / radius / length / primT / primR.
        .toggle("autofit", "Auto Fit Box / Sphere / Capsule to Geometry", true,
                "Fit Box, Sphere and Capsule shapes to each piece's geometry. Off uses pscale / scale and the bullet_prim* attributes.")
        .toggle("hullperconnected", "Create Convex Hull per Set of Connected Primitives", false,
                "Build one convex hull per set of connected primitives, instead of one hull around the whole piece.")
        .tab("Body")
        .menu("bodytype", "Body Type", 2, { "Static", "Kinematic", "Dynamic" },
              "Static never moves, Kinematic follows its animation, Dynamic is fully simulated. Points with active = 0 are static.")
        .toggle("computemass", "Compute Mass", false,
                "Work out mass from Density and shape volume instead of using Mass. Overridden by the computemass attribute.")
        .number("mass", "Mass", 1.0, R{ 0, 10 },
                "Mass of each body (used when Compute Mass is off). Overridden by the mass point attribute.",
                "{ computemass == 1 }")
        .number("density", "Density", 1000.0, R{ 0, 2000 },
                "Mass per unit volume, used when Compute Mass is on (1000 is about water). Overridden by the density attribute.",
                "{ computemass == 0 }")
        .number("gravityscale", "Gravity Scale", 1.0,
                "Multiplies the solver's Gravity: 0 floats, negative falls upward. Overridden by the gravityscale attribute.")
        .tab("Material")
        .number("friction", "Friction", 0.6, R{ 0, 1 },
                "How much surfaces resist sliding; 0 is ice-slick. Overridden by the friction attribute.")
        .number("bounce", "Bounce", 0.0, R{ 0, 1 },
                "Speed kept after a collision: 0 doesn't bounce, 1 bounces at full speed. Overridden by the bounce attribute.")
        .number("rollingresistance", "Rolling Resistance", 0.0, R{ 0, 1 },
                "How much round bodies resist rolling; 0 rolls freely. Overridden by the rollingresistance attribute.")
        .vector("tangentvelocity", "Conveyor Velocity", { 0, 0, 0 },
                "Makes this body a conveyor belt: touching bodies are dragged along this velocity, in the body's own axes. "
                "Needs friction above 0. Overridden by the tangentvelocity attribute.", PRM_XYZ_J)
        .number("lineardamping", "Linear Damping", 0.0, R{ 0, 10 },
                "Slows movement like air drag; 0 is none. Overridden by the lineardamping attribute.")
        .number("angulardamping", "Angular Damping", 0.0, R{ 0, 10 },
                "Slows spinning; 0 is none. Overridden by the angulardamping attribute.")
        .tab("Sleep & Contacts")
        .toggle("enablesleep", "Allow Sleeping", true,
                "Let resting bodies stop simulating until touched. Needs the solver's Allow Sleeping. Overridden by the enablesleep attribute.")
        .toggle("startasleep", "Start Asleep", false,
                "Bodies begin asleep until touched. Needs Allow Sleeping. Overridden by the awake attribute.")
        // Box3D: one threshold, fixed half second to sleep.
        .number("sleepthreshold", "Sleep Threshold", 0.05, R{ 0, 1 },
                "A body falls asleep once its speed stays below this for half a second. Overridden by the sleepthreshold attribute.")
        .number("ccdthreshold", "CCD Threshold", 0.5, R{ 0, 1 },
                "Move distance per step, as a fraction of the body's thinnest size, above which continuous collision kicks in. "
                "Lower stops more tunnelling but costs more. Overridden by the ccdthreshold attribute.")
        // Box3D's "bullet" flag (Houdini's `isbullet` means something else).
        .toggle("fastobject", "Fast Object", false,
                "Also run continuous collision against moving bodies, so fast bodies can't pass through them.")
        .toggle("contactrecycling", "Contact Recycling", true,
                "Reuse contacts between steps for speed. Turn off if thin or fast bodies show ghost collisions.")
        .tab("Output")
        // Only read when this node sits on an affector object (a Static Object DOP) that collides with the solver.
        .toggle("flipwinding", "Flip Static Mesh Winding", false,
                "Static meshes are one-sided. Turn on if bodies fall through from the side that should be solid.")
        .toggle("addimpact", "Add Impact Data", false,
                "Record impacts in Impacts data on the object. Costs extra time and memory.")
        .templates();

    static SIM_DopDescription theDopDescription(
        true,                   // true: also creates a DOP node that applies this data
        "box3d_data",           // internal node name
        "Box3D Data",           // node label
        kBox3DDataName,         // data name on the object
        classname(),
        templates);

    return &theDopDescription;
}

// The menus list the enums in order.
static_assert(int(b3dop::BodySettings::Type::Static) == 0 && int(b3dop::BodySettings::Type::Kinematic) == 1 &&
              int(b3dop::BodySettings::Type::Dynamic) == 2, "bodytype menu order");
static_assert(int(b3dop::ShapeType::Box) == 0 && int(b3dop::ShapeType::Sphere) == 1 &&
              int(b3dop::ShapeType::Convex) == 2 && int(b3dop::ShapeType::Concave) == 3 &&
              int(b3dop::ShapeType::Capsule) == 4 && int(b3dop::ShapeType::None) == 5, "defaultshape menu order");

b3dop::BodySettings
SIM_Box3DData::toBodySettings() const
{
    b3dop::BodySettings s;

    // Out-of-range menu values (set from a script) fall back to the defaults.
    const int bodyType = int(getBodyType());
    if (bodyType >= 0 && bodyType <= 2)
        s.type = b3dop::BodySettings::Type(bodyType);
    const int shape = int(getDefaultShape());
    if (shape >= 0 && shape <= 5)
        s.defaultShape = b3dop::ShapeType(shape);

    b3dop::BodyProps &p = s.props;
    p.mass           = static_cast<float>(getMass());
    p.computeMass    = getComputeMass();
    p.density        = static_cast<float>(getDensity());
    p.friction       = static_cast<float>(getFriction());
    p.restitution    = static_cast<float>(getBounce());
    p.rollingResistance = static_cast<float>(getRollingResistance());
    {
        const UT_Vector3 t = getTangentVelocity();
        for (int i = 0; i < 3; ++i)
            p.tangentVelocity[i] = static_cast<float>(t(i));
    }
    p.linearDamping  = static_cast<float>(getLinearDamping());
    p.angularDamping = static_cast<float>(getAngularDamping());
    p.gravityScale   = static_cast<float>(getGravityScale());
    p.enableSleep    = getEnableSleep();
    s.startAwake     = !getStartAsleep();
    p.fastObject     = getFastObject();
    p.contactRecycling = getContactRecycling();
    p.sleepThreshold = static_cast<float>(getSleepThreshold());
    p.ccdThreshold   = static_cast<float>(getCcdThreshold());
    s.hullPerConnectedSet = getHullPerConnected();
    s.autoFit        = getAutoFit();
    s.flipMeshWinding = getFlipWinding();
    return s;
}
