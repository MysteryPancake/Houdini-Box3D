// SIM_Box3DData.cpp
#include "SIM_Box3DData.h"

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

    static PRM_Name    theDensityName("density", "Density");
    static PRM_Default theDensityDefault(1.0);

    static PRM_Name    theFrictionName("friction", "Friction");
    static PRM_Default theFrictionDefault(0.6);

    static PRM_Name    theRestitutionName("restitution", "Restitution");
    static PRM_Default theRestitutionDefault(0.0);

    static PRM_Name    theLinDampName("lineardamping", "Linear Damping");
    static PRM_Default theLinDampDefault(0.0);

    static PRM_Name    theAngDampName("angulardamping", "Angular Damping");
    static PRM_Default theAngDampDefault(0.0);

    static PRM_Name    theGravScaleName("gravityscale", "Gravity Scale");
    static PRM_Default theGravScaleDefault(1.0);

    static PRM_Name    theSleepName("enablesleep", "Allow Sleeping");
    static PRM_Default theSleepDefault(1);

    static PRM_Name    theBulletName("isbullet", "Bullet (Fast Object CCD)");
    static PRM_Default theBulletDefault(0);

    static PRM_Name theShapeName("defaultshape", "Default Shape");
    static PRM_Name theShapeChoices[] = {
        PRM_Name("0", "Box"),
        PRM_Name("1", "Sphere"),
        PRM_Name("2", "Convex Hull"),
        PRM_Name("3", "Concave (Hull per Piece)"),
        PRM_Name(0)
    };
    static PRM_ChoiceList theShapeMenu(PRM_CHOICELIST_SINGLE, theShapeChoices);
    static PRM_Default    theShapeDefault(0);                 // box

    static PRM_Name    theHullPerConnectedName("hullperconnected",
                                               "Create Convex Hull per Set of Connected Primitives");
    static PRM_Default theHullPerConnectedDefault(0);

    static PRM_Template theTemplates[] = {
        PRM_Template(PRM_ORD,    1, &theShapeName,       &theShapeDefault, &theShapeMenu),
        PRM_Template(PRM_TOGGLE, 1, &theHullPerConnectedName, &theHullPerConnectedDefault),
        PRM_Template(PRM_ORD,    1, &theBodyTypeName,    &theBodyTypeDefault, &theBodyTypeMenu),
        PRM_Template(PRM_FLT_J,  1, &theDensityName,     &theDensityDefault),
        PRM_Template(PRM_FLT_J,  1, &theFrictionName,    &theFrictionDefault),
        PRM_Template(PRM_FLT_J,  1, &theRestitutionName, &theRestitutionDefault),
        PRM_Template(PRM_FLT_J,  1, &theLinDampName,     &theLinDampDefault),
        PRM_Template(PRM_FLT_J,  1, &theAngDampName,     &theAngDampDefault),
        PRM_Template(PRM_FLT_J,  1, &theGravScaleName,   &theGravScaleDefault),
        PRM_Template(PRM_TOGGLE, 1, &theSleepName,       &theSleepDefault),
        PRM_Template(PRM_TOGGLE, 1, &theBulletName,      &theBulletDefault),
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

    s.density        = static_cast<float>(getDensity());
    s.friction       = static_cast<float>(getFriction());
    s.restitution    = static_cast<float>(getRestitution());
    s.linearDamping  = static_cast<float>(getLinearDamping());
    s.angularDamping = static_cast<float>(getAngularDamping());
    s.gravityScale   = static_cast<float>(getGravityScale());
    s.enableSleep    = getEnableSleep();
    s.isBullet       = getIsBullet();
    s.hullPerConnectedSet = getHullPerConnected();
    switch (getDefaultShape())
    {
        case 1:  s.defaultShape = b3dop::ShapeType::Sphere;  break;
        case 2:  s.defaultShape = b3dop::ShapeType::Convex;  break;
        case 3:  s.defaultShape = b3dop::ShapeType::Concave; break;
        default: s.defaultShape = b3dop::ShapeType::Box;     break;
    }
    return s;
}
