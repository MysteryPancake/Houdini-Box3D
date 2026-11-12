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

    // Box3D requires density, friction, restitution and damping to be >= 0
    // (B3_ASSERT in body.c / shape.c). Hard minimum 0; the slider max is only
    // a UI convenience, so larger values can still be typed in.
    //
    // Parameter names match the point attribute that overrides each one
    // (mass, friction, bounce, lineardamping, ...), so the node and the
    // attributes read the same.
    static PRM_Range theMassRange       (PRM_RANGE_RESTRICTED, 0, PRM_RANGE_UI, 10);
    static PRM_Range theFrictionRange   (PRM_RANGE_RESTRICTED, 0, PRM_RANGE_UI, 1);
    static PRM_Range theBounceRange     (PRM_RANGE_RESTRICTED, 0, PRM_RANGE_UI, 1);
    static PRM_Range theDampingRange    (PRM_RANGE_RESTRICTED, 0, PRM_RANGE_UI, 10);

    // Mass of each body; the solver turns it into density = mass / shape volume.
    static PRM_Name    theMassName("mass", "Mass");
    static PRM_Default theMassDefault(1.0);

    // Compute Mass on: mass = density * shape volume (point attributes: computemass, density).
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

    // Box3D's "bullet" flag, under a name that does not clash with Houdini's `isbullet`.
    static PRM_Name    theFastObjectName("fastobject", "Fast Object");
    static PRM_Default theFastObjectDefault(0);

    // Box3D has one sleep threshold (speed) and a fixed time to sleep (half a second).
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

    // On: a box / sphere / capsule on a packed point is fitted to the packed
    // geometry's bounds. Off: it is built from the point attributes bullet_primS,
    // bullet_radius, bullet_length, bullet_primT and bullet_primR instead (anything
    // missing falls back to pscale / scale). The `bullet_autofit` point attribute
    // overrides this per point.
    static PRM_Name    theAutoFitName("autofit", "Auto Fit Box / Sphere / Capsule to Geometry");
    static PRM_Default theAutoFitDefault(1);

    static PRM_Template theTemplates[] = {
        PRM_Template(PRM_ORD,    1, &theShapeName,       &theShapeDefault, &theShapeMenu),
        PRM_Template(PRM_TOGGLE, 1, &theAutoFitName,     &theAutoFitDefault),
        PRM_Template(PRM_TOGGLE, 1, &theHullPerConnectedName, &theHullPerConnectedDefault),
        PRM_Template(PRM_ORD,    1, &theBodyTypeName,    &theBodyTypeDefault, &theBodyTypeMenu),
        PRM_Template(PRM_TOGGLE, 1, &theComputeMassName, &theComputeMassDefault),
        PRM_Template(PRM_FLT_J,  1, &theMassName, &theMassDefault, 0, &theMassRange, 0, 0, 1,
                     "Mass of each body (used when Compute Mass is off).", &theMassDisable),
        PRM_Template(PRM_FLT_J,  1, &theDensityName, &theDensityDefault, 0, &theDensityRange, 0, 0, 1,
                     "Mass per unit volume (used when Compute Mass is on).", &theDensityDisable),
        PRM_Template(PRM_FLT_J,  1, &theFrictionName, &theFrictionDefault, 0, &theFrictionRange),
        PRM_Template(PRM_FLT_J,  1, &theBounceName, &theBounceDefault, 0, &theBounceRange),
        PRM_Template(PRM_FLT_J,  1, &theLinDampName, &theLinDampDefault, 0, &theDampingRange),
        PRM_Template(PRM_FLT_J,  1, &theAngDampName, &theAngDampDefault, 0, &theDampingRange),
        PRM_Template(PRM_FLT_J,  1, &theGravScaleName,   &theGravScaleDefault),
        PRM_Template(PRM_TOGGLE, 1, &theSleepName,       &theSleepDefault),
        PRM_Template(PRM_FLT_J,  1, &theSleepThresholdName, &theSleepThresholdDefault, 0, &theSleepThresholdRange, 0, 0, 1,
                     "A body may fall asleep once its speed has stayed below this for half a second."),
        PRM_Template(PRM_TOGGLE, 1, &theFastObjectName,  &theFastObjectDefault, 0, 0, 0, 0, 1,
                     "Continuous collision against other moving bodies too, so a fast body cannot pass through them. "
                     "Every body already has it against static ones."),
        PRM_Template(PRM_TOGGLE, 1, &theAddImpactName,   &theAddImpactDefault, 0, 0, 0, 0, 1,
                     "When turned on, any impacts that occur during the simulation are recorded in the "
                     "Impacts data on the DOP object. Turning this on may increase simulation time and memory use."),
        PRM_Template()
    };

    static SIM_DopDescription theDopDescription(
        true,                   // true: also creates a DOP node that applies this data
        "box3d_data",           // internal node name
        "Box3D Data",           // node label
        "Box3DData",            // data name on the object
        classname(),
        theTemplates);

    return &theDopDescription;
}

b3dop::BodySettings
SIM_Box3DData::toBodySettings() const
{
    b3dop::BodySettings s;

    switch (getBodyType())
    {
        case 0:  s.type = b3dop::BodySettings::Type::Static;    break;
        case 1:  s.type = b3dop::BodySettings::Type::Kinematic; break;
        default: s.type = b3dop::BodySettings::Type::Dynamic;   break;
    }

    s.mass           = static_cast<float>(getMass());
    s.computeMass    = getComputeMass();
    s.density        = static_cast<float>(getDensity());
    s.friction       = static_cast<float>(getFriction());
    s.restitution    = static_cast<float>(getBounce());
    s.linearDamping  = static_cast<float>(getLinearDamping());
    s.angularDamping = static_cast<float>(getAngularDamping());
    s.gravityScale   = static_cast<float>(getGravityScale());
    s.enableSleep    = getEnableSleep();
    s.isBullet       = getFastObject();
    s.sleepThreshold = static_cast<float>(getSleepThreshold());
    s.hullPerConnectedSet = getHullPerConnected();
    s.autoFit        = getAutoFit();
    switch (getDefaultShape())
    {
        case 1:  s.defaultShape = b3dop::ShapeType::Sphere;  break;
        case 2:  s.defaultShape = b3dop::ShapeType::Convex;  break;
        case 3:  s.defaultShape = b3dop::ShapeType::Concave;  break;
        case 4:  s.defaultShape = b3dop::ShapeType::Capsule;  break;
        case 5:  s.defaultShape = b3dop::ShapeType::None;     break;
        default: s.defaultShape = b3dop::ShapeType::Box;     break;
    }
    return s;
}
