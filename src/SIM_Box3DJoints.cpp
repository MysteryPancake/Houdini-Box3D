// SIM_Box3DJoints.cpp
#include "SIM_Box3DJoints.h"
#include "Box3DWorld.h"

#include <PRM/PRM_Include.h>
#include <SIM/SIM_DopDescription.h>
#include <UT/UT_Quaternion.h>
#include <UT/UT_Vector3.h>
#include <UT/UT_XformOrder.h>

#include <array>
#include <cmath>
#include <deque>
#include <vector>

namespace {

constexpr float kRadToDeg = 57.29577951308232f;
constexpr float kDegToRad = 0.017453292519943295f;

} // namespace

void
box3dEulerToQuat(const float degrees[3], float q[4])
{
    const UT_Vector3F rad(degrees[0] * kDegToRad, degrees[1] * kDegToRad, degrees[2] * kDegToRad);
    const UT_QuaternionF uq(rad, UT_XformOrder(UT_XformOrder::TRS, UT_XformOrder::XYZ));
    q[0] = uq.x();  q[1] = uq.y();  q[2] = uq.z();  q[3] = uq.w();
}

SIM_Box3DJoints::SIM_Box3DJoints(const SIM_DataFactory *factory)
    : BaseClass(factory),
      SIM_OptionsUser(this)
{
}

SIM_Box3DJoints::~SIM_Box3DJoints()
{
}

bool
SIM_Box3DJoints::isCommon(const b3dop::JointParam &p)
{
    // Settings every (constraining) type has: one parameter for all of them.
    return p.usedBy(b3dop::JointType::Weld) && p.usedBy(b3dop::JointType::Spherical) &&
           p.usedBy(b3dop::JointType::Revolute) && p.usedBy(b3dop::JointType::Prismatic) &&
           p.usedBy(b3dop::JointType::Distance) && p.usedBy(b3dop::JointType::Motor) &&
           p.usedBy(b3dop::JointType::Parallel) && p.usedBy(b3dop::JointType::Wheel);
}

std::string
SIM_Box3DJoints::parmName(b3dop::JointType type, const b3dop::JointParam &p)
{
    return isCommon(p) ? std::string(p.name) : std::string(b3dop::jointTypeName(type)) + "_" + p.name;
}

std::string
SIM_Box3DJoints::geometryDataName() const
{
    const UT_StringHolder &name = getOptions().getOptionS("geometrydata");
    return name.isstring() ? std::string(name.c_str()) : std::string(kDefaultJointsGeometry);
}

b3dop::JointSettings
SIM_Box3DJoints::defaults(b3dop::JointType type) const
{
    using b3dop::JointParam;
    b3dop::JointSettings s = b3dop::Box3DWorld::defaultJointSettings(type);
    const auto &opts = getOptions();
    for (const JointParam &p : b3dop::jointParams())
    {
        if (!p.usedBy(type))
            continue;
        const std::string name  = parmName(type, p);
        const float       scale = p.isAngle(type) ? kDegToRad : 1.0f;
        switch (p.kind)
        {
            case JointParam::Toggle:
                s.*p.b = opts.getOptionB(name.c_str());
                break;
            case JointParam::Float:
            case JointParam::Angle:
                s.*p.f = float(opts.getOptionF(name.c_str())) * scale;
                break;
            case JointParam::Vec3:
            case JointParam::Vec3Angle:
            {
                const UT_Vector3D v = opts.getOptionV3(name.c_str());
                for (int i = 0; i < 3; ++i)
                    (s.*p.v)[i] = float(v(i)) * scale;
                break;
            }
            case JointParam::Rotation:
            {
                const UT_Vector3D v = opts.getOptionV3(name.c_str());
                const float deg[3] = { float(v(0)), float(v(1)), float(v(2)) };
                box3dEulerToQuat(deg, s.*p.q);
                break;
            }
        }
    }
    return s;
}

const SIM_DopDescription *
SIM_Box3DJoints::getDescription()
{
    using b3dop::JointParam;
    using b3dop::JointType;

    // Everything the templates point at lives here for the life of the plugin (deques: stable
    // addresses as they grow).
    static std::deque<std::string>                strings;
    static std::deque<PRM_Name>                   names;
    static std::deque<std::array<PRM_Default, 3>> defaults;
    static std::vector<PRM_Template>              templates;
    static std::vector<PRM_Default>               tabs;
    static bool                                   built = false;
    static PRM_Name                               theTabsName("b3djointtabs");
    static PRM_Name                               theGeoName("geometrydata", "Geometry Data");
    static PRM_Default                            theGeoDefault(0, kDefaultJointsGeometry);
    static PRM_Name                               theFeedbackName("writefeedback", "Write Feedback");
    static PRM_Default                            theFeedbackDefault(1);

    if (!built)
    {
        built = true;
        auto keep = [&](const std::string &str) { strings.push_back(str); return strings.back().c_str(); };
        // One template for setting `p` of type `t` (the default from Box3D, shown in Houdini units).
        auto add = [&](JointType t, const JointParam &p) {
            const b3dop::JointSettings d = b3dop::Box3DWorld::defaultJointSettings(t);
            const float scale = p.isAngle(t) ? kRadToDeg : 1.0f;
            names.emplace_back(keep(SIM_Box3DJoints::parmName(t, p)), p.label);
            defaults.emplace_back();
            std::array<PRM_Default, 3> &def = defaults.back();
            int size = 1;
            PRM_Type type = PRM_FLT_J;
            switch (p.kind)
            {
                case JointParam::Toggle:
                    type   = PRM_TOGGLE;
                    def[0] = PRM_Default((d.*p.b) ? 1 : 0);
                    break;
                case JointParam::Float:
                case JointParam::Angle:
                    def[0] = PRM_Default(d.*p.f * scale);
                    break;
                case JointParam::Vec3:
                case JointParam::Vec3Angle:
                    size = 3;
                    for (int i = 0; i < 3; ++i)
                        def[size_t(i)] = PRM_Default((d.*p.v)[i] * scale);
                    break;
                case JointParam::Rotation:
                    size = 3;           // Euler degrees; Box3D's default target is the identity
                    break;
            }
            templates.push_back(PRM_Template(type, size, &names.back(), def.data(), 0, 0, 0, 0, 1, p.help));
        };

        templates.push_back(PRM_Template(PRM_STRING, 1, &theGeoName, &theGeoDefault, 0, 0, 0, 0, 1,
                                         "Data name of the joint network geometry on the object (two-point "
                                         "primitives, attached with a SOP Geometry DOP)."));
        templates.push_back(PRM_Template(PRM_TOGGLE, 1, &theFeedbackName, &theFeedbackDefault, 0, 0, 0, 0, 1,
                                         "Write each joint's force, torque and readings onto its primitive every step."));

        // Tabs: Common, then one per type that has settings of its own. The switcher goes first,
        // so remember where and fill in its folders afterwards.
        const size_t switcherAt = templates.size();
        templates.emplace_back();
        int count = 0;
        for (const JointParam &p : b3dop::jointParams())
            if (SIM_Box3DJoints::isCommon(p))
            {
                add(JointType::Weld, p);
                ++count;
            }
        tabs.emplace_back(count, "Common");
        for (int t = 0; t < b3dop::kJointTypeCount; ++t)
        {
            count = 0;
            for (const JointParam &p : b3dop::jointParams())
                if (p.usedBy(JointType(t)) && !SIM_Box3DJoints::isCommon(p))
                {
                    add(JointType(t), p);
                    ++count;
                }
            if (count)
                tabs.emplace_back(count, b3dop::jointTypeLabel(JointType(t)));
        }
        templates[switcherAt] = PRM_Template(PRM_SWITCHER, int(tabs.size()), &theTabsName, tabs.data());
        templates.emplace_back();       // terminator
    }

    static SIM_DopDescription theDopDescription(
        true,                   // true: also creates a DOP node that applies this data
        "box3d_joints",         // internal node name
        "Box3D Joints",         // node label
        kBox3DJointsDataName,   // data name on the object
        classname(),
        templates.data());

    return &theDopDescription;
}
