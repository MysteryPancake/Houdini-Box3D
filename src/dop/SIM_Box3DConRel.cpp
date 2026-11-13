// SIM_Box3DConRel.cpp
#include "SIM_Box3DConRel.h"
#include "SIM_Box3DParms.h"
#include "core/Box3DWorld.h"
#include "sync/Box3DHoudiniUtil.h"

#include <SIM/SIM_DopDescription.h>
#include <UT/UT_Quaternion.h>
#include <UT/UT_Vector3.h>
#include <UT/UT_XformOrder.h>

#include <memory>
#include <string>
#include <vector>

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
    const b3dop::JointType type = jointType();
    b3dop::JointSettings   s    = b3dop::Box3DWorld::defaultJointSettings(type);
    const auto            &opts = getOptions();
    for (const b3dop::JointParam &p : b3dop::jointParams())
    {
        if (!p.usedBy(type))
            continue;
        float v[3] = { 0, 0, 0 };
        if (p.kind == b3dop::JointParam::Toggle)
            v[0] = opts.getOptionB(p.name) ? 1.0f : 0.0f;
        else if (b3dop::jointParamSize(p) == 1)
            v[0] = float(opts.getOptionF(p.name));
        else
        {
            const UT_Vector3D o = opts.getOptionV3(p.name);
            for (int i = 0; i < 3; ++i)
                v[i] = float(o(i));
        }
        b3dop::setJointParam(s, p, type, v, box3dEulerToQuat);
    }
    return s;
}

const SIM_DopDescription *
SIM_Box3DConRel::describe(b3dop::JointType t, const UT_StringHolder &classname)
{
    using b3dop::JointParam;

    // Per joint type, kept for the life of the plugin (Houdini keeps pointers into it).
    struct Storage
    {
        Box3DParms                          parms;
        std::string                         nodeName, label, dataName;
        std::unique_ptr<SIM_DopDescription> description;
    };
    static Storage storage[b3dop::kJointTypeCount];

    Storage &st = storage[int(t)];
    if (st.description)
        return st.description.get();

    // Common settings first (jointParams() lists them first), then the type's own, each named
    // like the primitive attribute that modifies it. Defaults are Box3D's, in Houdini units.
    // Settings a toggle switches on (springs, motors, limits) are disabled while it is off.
    const b3dop::JointSettings d = b3dop::Box3DWorld::defaultJointSettings(t);
    for (const JointParam &p : b3dop::jointParams())
    {
        if (!p.usedBy(t))
            continue;
        float v[3];
        b3dop::getJointParam(d, p, t, v);
        const std::string disableWhen = b3dop::jointParamDisableWhen(p, t);
        if (p.kind == JointParam::Toggle)
            st.parms.toggle(p.name, p.label, v[0] != 0.0f, p.help, disableWhen.c_str());
        else if (b3dop::jointParamSize(p) == 1)
            st.parms.number(p.name, p.label, v[0], p.help, disableWhen.c_str());
        else
            st.parms.vector(p.name, p.label, { v[0], v[1], v[2] }, p.help, PRM_FLT_J, disableWhen.c_str());
    }

    const std::string typeName = b3dop::jointTypeName(t);
    st.nodeName = "box3d_" + typeName + "conrel";
    st.label    = std::string("Box3D ") + b3dop::jointTypeLabel(t) + " Constraint Relationship";
    st.dataName = typeName;             // what `constraint_name` refers to by default

    st.description = std::make_unique<SIM_DopDescription>(
        true,                           // also creates a DOP node that applies this data
        st.nodeName.c_str(),            // internal node name
        st.label.c_str(),               // node label
        st.dataName.c_str(),            // default Data Name
        classname,
        st.parms.templates());
    return st.description.get();
}
