// SIM_Box3DData.cpp
#include "SIM_Box3DData.h"

#include <PRM/PRM_Conditional.h>   // PRM_Template.h only forward-declares PRM_Conditional
#include <PRM/PRM_Include.h>
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
    static PRM_Name theBodyTypeName("bodytype", "Body Type");
    static PRM_Name theBodyTypeChoices[] = {
        PRM_Name("0", "Static"),
        PRM_Name("1", "Kinematic"),
        PRM_Name("2", "Dynamic"),
        PRM_Name(0)
    };
    static PRM_ChoiceList theBodyTypeMenu(PRM_CHOICELIST_SINGLE, theBodyTypeChoices);
    static PRM_Default    theBodyTypeDefault(2);              // dynamic

    // Box3D asserts on negatives: hard minimum 0 (larger values than the slider can be typed).
    // Parameter names match the point attributes that override them.
    static PRM_Range theMassRange       (PRM_RANGE_RESTRICTED, 0, PRM_RANGE_UI, 10);
    static PRM_Range theFrictionRange   (PRM_RANGE_RESTRICTED, 0, PRM_RANGE_UI, 1);
    static PRM_Range theBounceRange     (PRM_RANGE_RESTRICTED, 0, PRM_RANGE_UI, 1);
    static PRM_Range theDampingRange    (PRM_RANGE_RESTRICTED, 0, PRM_RANGE_UI, 10);

    static PRM_Name    theMassName("mass", "Mass");
    static PRM_Default theMassDefault(1.0);

    static PRM_Name    theComputeMassName("computemass", "Compute Mass");
    static PRM_Default theComputeMassDefault(0);
    static PRM_Conditional theMassDisable("{ computemass == 1 }", PRM_CONDTYPE_DISABLE);
    static PRM_Conditional theDensityDisable("{ computemass == 0 }", PRM_CONDTYPE_DISABLE);
    static PRM_Name    theDensityName("density", "Density");
    static PRM_Default theDensityDefault(1000.0);
    static PRM_Range   theDensityRange(PRM_RANGE_RESTRICTED, 0, PRM_RANGE_UI, 2000);

    static PRM_Name    theFrictionName("friction", "Friction");
    static PRM_Default theFrictionDefault(0.6);

    static PRM_Name    theBounceName("bounce", "Bounce");
    static PRM_Default theBounceDefault(0.0);

    static PRM_Name    theLinDampName("lineardamping", "Linear Damping");
    static PRM_Default theLinDampDefault(0.0);

    static PRM_Name    theAngDampName("angulardamping", "Angular Damping");
    static PRM_Default theAngDampDefault(0.0);

    static PRM_Name    theGravScaleName("gravityscale", "Gravity Scale");
    static PRM_Default theGravScaleDefault(1.0);

    static PRM_Name    theSleepName("enablesleep", "Allow Sleeping");
    static PRM_Default theSleepDefault(1);

    static PRM_Name    theStartAsleepName("startasleep", "Start Asleep");
    static PRM_Default theStartAsleepDefault(0);

    // Box3D's "bullet" flag (Houdini's `isbullet` means something else).
    static PRM_Name    theFastObjectName("fastobject", "Fast Object");
    static PRM_Default theFastObjectDefault(0);

    static PRM_Name    theRecyclingName("contactrecycling", "Contact Recycling");
    static PRM_Default theRecyclingDefault(1);

    // Box3D: one threshold, fixed half second to sleep.
    static PRM_Name    theCcdThresholdName("ccdthreshold", "CCD Threshold");
    static PRM_Default theCcdThresholdDefault(0.5);
    static PRM_Range   theCcdThresholdRange(PRM_RANGE_RESTRICTED, 0, PRM_RANGE_UI, 1);
    static PRM_Name    theSleepThresholdName("sleepthreshold", "Sleep Threshold");
    static PRM_Default theSleepThresholdDefault(0.05);
    static PRM_Range   theSleepThresholdRange(PRM_RANGE_RESTRICTED, 0, PRM_RANGE_UI, 1);

    static PRM_Name    theAddImpactName("addimpact", "Add Impact Data");
    static PRM_Default theAddImpactDefault(0);

    static PRM_Name theShapeName("defaultshape", "Default Shape");
    static PRM_Name theShapeChoices[] = {
        PRM_Name("0", "Box"),
        PRM_Name("1", "Sphere"),
        PRM_Name("2", "Convex Hull"),
        PRM_Name("3", "Concave / Compound (Hull per Piece)"),
        PRM_Name("4", "Capsule"),
        PRM_Name("5", "None (No Collision)"),
        PRM_Name(0)
    };
    static PRM_ChoiceList theShapeMenu(PRM_CHOICELIST_SINGLE, theShapeChoices);
    static PRM_Default    theShapeDefault(0);                 // box

    static PRM_Name    theHullPerConnectedName("hullperconnected",
                                               "Create Convex Hull per Set of Connected Primitives");
    static PRM_Default theHullPerConnectedDefault(0);

    // Off: box / sphere / capsule from bullet_primS / radius / length / primT / primR.
    static PRM_Name    theAutoFitName("autofit", "Auto Fit Box / Sphere / Capsule to Geometry");
    static PRM_Default theAutoFitDefault(1);

    // Only read when this node sits on an affector object (a Static Object DOP) that collides with the solver.
    static PRM_Name    theFlipWindingName("flipwinding", "Flip Static Mesh Winding");
    static PRM_Default theFlipWindingDefault(0);

    // Tabs: each PRM_Default is (number of parameters that follow, tab label).
    static PRM_Name    theTabsName("b3dtabs");
    static PRM_Default theTabsDefaults[] = {
        PRM_Default(3, "Shape"),
        PRM_Default(5, "Body"),
        PRM_Default(4, "Material"),
        PRM_Default(6, "Sleep & Contacts"),
        PRM_Default(2, "Output")
    };

    static PRM_Template theTemplates[] = {
        PRM_Template(PRM_SWITCHER, int(sizeof(theTabsDefaults) / sizeof(PRM_Default)), &theTabsName, theTabsDefaults),
        // --- Shape ---
        PRM_Template(PRM_ORD,    1, &theShapeName,       &theShapeDefault, &theShapeMenu, 0, 0, 0, 1,
                     "Collision shape for points without a bullet_georep attribute. Concave / Compound builds one convex hull per piece."),
        PRM_Template(PRM_TOGGLE, 1, &theAutoFitName,     &theAutoFitDefault, 0, 0, 0, 0, 1,
                     "Fit Box, Sphere and Capsule shapes to each piece's geometry. When off, they come from pscale / scale and the bullet_prim* attributes."),
        PRM_Template(PRM_TOGGLE, 1, &theHullPerConnectedName, &theHullPerConnectedDefault, 0, 0, 0, 0, 1,
                     "For Convex Hull shapes, build a separate hull for each set of connected primitives in a piece "
                     "instead of one hull around the whole piece."),
        // --- Body ---
        PRM_Template(PRM_ORD,    1, &theBodyTypeName,    &theBodyTypeDefault, &theBodyTypeMenu, 0, 0, 0, 1,
                     "Static never moves, Kinematic follows its animation, Dynamic is fully simulated. Inactive points (active = 0) are static."),
        PRM_Template(PRM_TOGGLE, 1, &theComputeMassName, &theComputeMassDefault, 0, 0, 0, 0, 1,
                     "Work out each body's mass from its Density and collision shape volume, instead of using Mass. "
                     "Overridden by the computemass point attribute."),
        PRM_Template(PRM_FLT_J,  1, &theMassName, &theMassDefault, 0, &theMassRange, 0, 0, 1,
                     "Mass of each body (used when Compute Mass is off). Overridden by the mass point attribute.",
                     &theMassDisable),
        PRM_Template(PRM_FLT_J,  1, &theDensityName, &theDensityDefault, 0, &theDensityRange, 0, 0, 1,
                     "Mass per unit volume (used when Compute Mass is on). 1000 is roughly water. "
                     "Overridden by the density point attribute.", &theDensityDisable),
        PRM_Template(PRM_FLT_J,  1, &theGravScaleName,   &theGravScaleDefault, 0, 0, 0, 0, 1,
                     "Multiplies the solver's Gravity for these bodies: 0 floats, negative falls upward. "
                     "Overridden by the gravityscale point attribute."),
        // --- Material ---
        PRM_Template(PRM_FLT_J,  1, &theFrictionName, &theFrictionDefault, 0, &theFrictionRange, 0, 0, 1,
                     "How much surfaces resist sliding against each other. 0 is ice-slick. "
                     "Overridden by the friction point attribute."),
        PRM_Template(PRM_FLT_J,  1, &theBounceName, &theBounceDefault, 0, &theBounceRange, 0, 0, 1,
                     "How much speed is kept after a collision: 0 doesn't bounce, 1 bounces back at full speed. "
                     "Overridden by the bounce point attribute."),
        PRM_Template(PRM_FLT_J,  1, &theLinDampName, &theLinDampDefault, 0, &theDampingRange, 0, 0, 1,
                     "Slows movement over time, like air drag. 0 is no drag. "
                     "Overridden by the lineardamping point attribute."),
        PRM_Template(PRM_FLT_J,  1, &theAngDampName, &theAngDampDefault, 0, &theDampingRange, 0, 0, 1,
                     "Slows spinning over time. 0 is no drag. Overridden by the angulardamping point attribute."),
        // --- Sleep & Contacts ---
        PRM_Template(PRM_TOGGLE, 1, &theSleepName,       &theSleepDefault, 0, 0, 0, 0, 1,
                     "Let resting bodies stop simulating until touched. Needs the solver's Allow Sleeping on. "
                     "Overridden by the enablesleep point attribute."),
        PRM_Template(PRM_TOGGLE, 1, &theStartAsleepName, &theStartAsleepDefault, 0, 0, 0, 0, 1,
                     "Bodies begin asleep until something touches them. Has no effect if Allow Sleeping is off. "
                     "Overridden by the awake point attribute."),
        PRM_Template(PRM_FLT_J,  1, &theSleepThresholdName, &theSleepThresholdDefault, 0, &theSleepThresholdRange, 0, 0, 1,
                     "A body may fall asleep once its speed has stayed below this for half a second. "
                     "Overridden by the sleepthreshold point attribute."),
        PRM_Template(PRM_FLT_J,  1, &theCcdThresholdName, &theCcdThresholdDefault, 0, &theCcdThresholdRange, 0, 0, 1,
                     "How far a body must move in one step, as a fraction of its thinnest size, before continuous "
                     "collision sweeps it so it can't pass through things. Lower catches more tunnelling on thin, "
                     "fast pieces but costs more. Needs the solver's Continuous Collision on. "
                     "Overridden by the ccdthreshold point attribute."),
        PRM_Template(PRM_TOGGLE, 1, &theFastObjectName,  &theFastObjectDefault, 0, 0, 0, 0, 1,
                     "Continuous collision against moving bodies too, so a fast body can't pass through them."),
        PRM_Template(PRM_TOGGLE, 1, &theRecyclingName,   &theRecyclingDefault, 0, 0, 0, 0, 1,
                     "Reuse this body's contacts between steps, which is faster. Turn off if you see ghost collisions on thin or fast bodies."),
        // --- Output ---
        PRM_Template(PRM_TOGGLE, 1, &theFlipWindingName, &theFlipWindingDefault, 0, 0, 0, 0, 1,
                     "Static meshes are one-sided. Turn on if bodies fall through from the side you expect to be solid."),
        PRM_Template(PRM_TOGGLE, 1, &theAddImpactName,   &theAddImpactDefault, 0, 0, 0, 0, 1,
                     "Record impacts in Impacts data on the DOP object. Can increase simulation time and memory use."),
        PRM_Template()
    };

    static SIM_DopDescription theDopDescription(
        true,                   // true: also creates a DOP node that applies this data
        "box3d_data",           // internal node name
        "Box3D Data",           // node label
        kBox3DDataName,         // data name on the object
        classname(),
        theTemplates);

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

    s.mass           = static_cast<float>(getMass());
    s.computeMass    = getComputeMass();
    s.density        = static_cast<float>(getDensity());
    s.friction       = static_cast<float>(getFriction());
    s.restitution    = static_cast<float>(getBounce());
    s.linearDamping  = static_cast<float>(getLinearDamping());
    s.angularDamping = static_cast<float>(getAngularDamping());
    s.gravityScale   = static_cast<float>(getGravityScale());
    s.enableSleep    = getEnableSleep();
    s.startAwake     = !getStartAsleep();
    s.isBullet       = getFastObject();
    s.contactRecycling = getContactRecycling();
    s.sleepThreshold = static_cast<float>(getSleepThreshold());
    s.ccdThreshold   = static_cast<float>(getCcdThreshold());
    s.hullPerConnectedSet = getHullPerConnected();
    s.autoFit        = getAutoFit();
    s.flipMeshWinding = getFlipWinding();
    return s;
}
