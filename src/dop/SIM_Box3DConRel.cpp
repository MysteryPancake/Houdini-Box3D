// SIM_Box3DConRel.cpp
#include "SIM_Box3DConRel.h"
#include "core/Box3DWorld.h"

#include <PRM/PRM_Include.h>
#include <SIM/SIM_DopDescription.h>
#include <UT/UT_Quaternion.h>
#include <UT/UT_Vector3.h>
#include <UT/UT_XformOrder.h>

#include <array>
#include <deque>
#include <memory>
#include <string>
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

SIM_Box3DConRel::SIM_Box3DConRel(const SIM_DataFactory *factory)
    : BaseClass(factory),
      SIM_OptionsUser(this)
{
}

SIM_Box3DConRel::~SIM_Box3DConRel()
{
}

b3dop::JointSettings
SIM_Box3DConRel::settings() const
{
    using b3dop::JointParam;
    const b3dop::JointType type = jointType();
    b3dop::JointSettings   s    = b3dop::Box3DWorld::defaultJointSettings(type);
    const auto            &opts = getOptions();
    for (const JointParam &p : b3dop::jointParams())
    {
        if (!p.usedBy(type))
            continue;
        const float scale = p.isAngle(type) ? kDegToRad : 1.0f;
        switch (p.kind)
        {
            case JointParam::Toggle:
                s.*p.b = opts.getOptionB(p.name);
                break;
            case JointParam::Float:
            case JointParam::Angle:
                s.*p.f = float(opts.getOptionF(p.name)) * scale;
                break;
            case JointParam::Vec3:
            case JointParam::Vec3Angle:
            {
                const UT_Vector3D v = opts.getOptionV3(p.name);
                for (int i = 0; i < 3; ++i)
                    (s.*p.v)[i] = float(v(i)) * scale;
                break;
            }
            case JointParam::Rotation:
            {
                const UT_Vector3D v = opts.getOptionV3(p.name);
                const float deg[3] = { float(v(0)), float(v(1)), float(v(2)) };
                box3dEulerToQuat(deg, s.*p.q);
                break;
            }
        }
    }
    return s;
}

const SIM_DopDescription *
SIM_Box3DConRel::describe(b3dop::JointType t, const UT_StringHolder &classname)
{
    using b3dop::JointParam;

    // Everything a type's templates point at, kept for the life of the plugin (deques: stable
    // addresses as they grow). One per joint type.
    struct Storage
    {
        std::deque<std::string>                strings;
        std::deque<PRM_Name>                   names;
        std::deque<std::array<PRM_Default, 3>> defaults;
        std::vector<PRM_Template>              templates;
        std::string                            nodeName, label, dataName;
        std::unique_ptr<SIM_DopDescription>    description;
    };
    static Storage storage[b3dop::kJointTypeCount];

    Storage &st = storage[int(t)];
    if (st.description)
        return st.description.get();

    auto keep = [&](const std::string &str) { st.strings.push_back(str); return st.strings.back().c_str(); };

    // Common settings first (jointParams() lists them first), then the type's own, each named
    // like the primitive attribute that modifies it. Defaults are Box3D's, in Houdini units.
    const b3dop::JointSettings d = b3dop::Box3DWorld::defaultJointSettings(t);
    for (const JointParam &p : b3dop::jointParams())
    {
        if (!p.usedBy(t))
            continue;
        const float scale = p.isAngle(t) ? kRadToDeg : 1.0f;
        st.names.emplace_back(p.name, p.label);
        st.defaults.emplace_back();
        std::array<PRM_Default, 3> &def = st.defaults.back();
        int      size = 1;
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
                size = 3;               // Euler degrees; Box3D's default target is the identity
                break;
        }
        st.templates.push_back(PRM_Template(type, size, &st.names.back(), def.data(), 0, 0, 0, 0, 1, p.help));
    }
    st.templates.emplace_back();        // terminator

    const std::string typeName = b3dop::jointTypeName(t);
    st.nodeName = "box3d_" + typeName + "conrel";
    st.label    = std::string("Box3D ") + b3dop::jointTypeLabel(t) + " Constraint Relationship";
    st.dataName = typeName;             // what `constraint_name` refers to by default

    st.description = std::make_unique<SIM_DopDescription>(
        true,                           // also creates a DOP node that applies this data
        keep(st.nodeName),              // internal node name
        keep(st.label),                 // node label
        keep(st.dataName),              // default Data Name
        classname,
        st.templates.data());
    return st.description.get();
}
