// SIM_Box3DData.h
//
// Box3D Data: per-object defaults (like Bullet Data). Point attributes of the same names
// override them; Add Impact Data has no attribute.
#pragma once

#include <SIM/SIM_Data.h>
#include <SIM/SIM_DataUtils.h>
#include <SIM/SIM_OptionsUser.h>

#include "core/Box3DWorld.h"

// The name Box3D Data is attached under on a DOP object.
inline constexpr const char *kBox3DDataName = "Box3DData";

class SIM_Box3DData : public SIM_Data, public SIM_OptionsUser
{
public:
    GETSET_DATA_FUNCS_I("bodytype",       BodyType);        // 0 static, 1 kinematic, 2 dynamic
    GETSET_DATA_FUNCS_F("mass",           Mass);
    GETSET_DATA_FUNCS_B("computemass",    ComputeMass);     // mass = density * shape volume
    GETSET_DATA_FUNCS_F("density",        Density);
    GETSET_DATA_FUNCS_F("friction",       Friction);
    GETSET_DATA_FUNCS_F("bounce",         Bounce);          // Box3D restitution
    GETSET_DATA_FUNCS_F("lineardamping",  LinearDamping);
    GETSET_DATA_FUNCS_F("angulardamping", AngularDamping);
    GETSET_DATA_FUNCS_F("gravityscale",   GravityScale);
    GETSET_DATA_FUNCS_B("enablesleep",    EnableSleep);
    GETSET_DATA_FUNCS_B("fastobject",     FastObject);      // Box3D "bullet"
    GETSET_DATA_FUNCS_B("contactrecycling", ContactRecycling);  // contacts may be recycled (solver's Contact Recycle Distance)
    GETSET_DATA_FUNCS_F("sleepthreshold", SleepThreshold);
    GETSET_DATA_FUNCS_B("addimpact",      AddImpact);       // Impacts data
    GETSET_DATA_FUNCS_I("defaultshape",   DefaultShape);    // 0 box, 1 sphere, 2 convexhull, 3 concave/compound, 4 capsule, 5 none
    GETSET_DATA_FUNCS_B("hullperconnected", HullPerConnected);
    GETSET_DATA_FUNCS_B("autofit",        AutoFit);         // point attribute: bullet_autofit
    GETSET_DATA_FUNCS_B("flipwinding",    FlipWinding);     // affector (Static Object) meshes only

    b3dop::BodySettings toBodySettings() const;

protected:
    explicit SIM_Box3DData(const SIM_DataFactory *factory);
    virtual ~SIM_Box3DData();

private:
    static const SIM_DopDescription *getDescription();

    DECLARE_STANDARD_GETCASTTOTYPE();
    DECLARE_DATAFACTORY(SIM_Box3DData, SIM_Data, "Box3D Data", getDescription());
};
