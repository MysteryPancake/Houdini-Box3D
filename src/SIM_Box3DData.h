// SIM_Box3DData.h
//
// Box3D Data: default Box3D settings for every point of a DOP object (like
// Bullet Data). Attach to a DOP object (the registered DOP node does this).
//
// Per-point attributes (shape, friction, bounce, mass, active, ...) override
// these; this node is only the fallback for points that do not have them.
//
// Mirrors b3dop::BodySettings, which mirrors the relevant parts of b3BodyDef
// and b3ShapeDef.
#pragma once

#include <SIM/SIM_Data.h>
#include <SIM/SIM_DataUtils.h>
#include <SIM/SIM_OptionsUser.h>

#include "Box3DWorld.h"

class SIM_Box3DData : public SIM_Data, public SIM_OptionsUser
{
public:
    // Generates getXxx()/setXxx() backed by the option names.
    GETSET_DATA_FUNCS_I("bodytype",       BodyType);        // 0 static, 1 kinematic, 2 dynamic
    GETSET_DATA_FUNCS_F("density",        Density);
    GETSET_DATA_FUNCS_F("friction",       Friction);
    GETSET_DATA_FUNCS_F("restitution",    Restitution);
    GETSET_DATA_FUNCS_F("lineardamping",  LinearDamping);
    GETSET_DATA_FUNCS_F("angulardamping", AngularDamping);
    GETSET_DATA_FUNCS_F("gravityscale",   GravityScale);
    GETSET_DATA_FUNCS_B("enablesleep",    EnableSleep);
    GETSET_DATA_FUNCS_B("isbullet",       IsBullet);
    GETSET_DATA_FUNCS_I("defaultshape",   DefaultShape);    // 0 box, 1 sphere, 2 convexhull, 3 concave/compound, 4 capsule, 5 none
    GETSET_DATA_FUNCS_B("hullperconnected", HullPerConnected); // convex: one hull per set of connected prims

    // Convenience: pack the options into the Houdini-free struct.
    b3dop::BodySettings toBodySettings() const;

protected:
    explicit SIM_Box3DData(const SIM_DataFactory *factory);
    virtual ~SIM_Box3DData();

private:
    static const SIM_DopDescription *getDescription();

    DECLARE_STANDARD_GETCASTTOTYPE();
    DECLARE_DATAFACTORY(SIM_Box3DData, SIM_Data, "Box3D Data", getDescription());
};
