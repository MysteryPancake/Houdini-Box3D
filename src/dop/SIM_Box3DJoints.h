// SIM_Box3DJoints.h
//
// Box3D Joints: per-object joint defaults, one tab per Box3D joint type, plus the name of the
// geometry data holding the joint network (two-point primitives; see Box3DJointSync.h).
// Primitive attributes of the same names override the defaults. Its parameters are generated
// from b3dop::jointParams(), so the node, the attributes and the core always agree.
#pragma once

#include <SIM/SIM_Data.h>
#include <SIM/SIM_DataUtils.h>
#include <SIM/SIM_OptionsUser.h>

#include <string>

#include "core/Box3DJoints.h"

// The name Box3D Joints is attached under, and the default name of the network's geometry data.
inline constexpr const char *kBox3DJointsDataName   = "Box3DJoints";
inline constexpr const char *kDefaultJointsGeometry = "Joints";

class SIM_Box3DJoints : public SIM_Data, public SIM_OptionsUser
{
public:
    GETSET_DATA_FUNCS_B("writefeedback", WriteFeedback);

    // The data name of the joint network geometry on the object (default "Joints").
    std::string geometryDataName() const;

    // The node's defaults for a joint type, in Box3D units.
    b3dop::JointSettings defaults(b3dop::JointType type) const;

    // The node's parameter name for a setting: common settings use the attribute name, the rest
    // are prefixed with their type ("revolute_lower_angle").
    static std::string parmName(b3dop::JointType type, const b3dop::JointParam &p);
    static bool        isCommon(const b3dop::JointParam &p);

protected:
    explicit SIM_Box3DJoints(const SIM_DataFactory *factory);
    ~SIM_Box3DJoints() override;

private:
    static const SIM_DopDescription *getDescription();

    DECLARE_STANDARD_GETCASTTOTYPE();
    DECLARE_DATAFACTORY(SIM_Box3DJoints, SIM_Data, "Box3D Joints", getDescription());
};

// Euler angles in degrees (Houdini's XYZ rotate order) -> quaternion x, y, z, w.
void box3dEulerToQuat(const float degrees[3], float q[4]);
