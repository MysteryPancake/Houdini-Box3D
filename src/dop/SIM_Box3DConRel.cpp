// SIM_Box3DConRel.cpp
#include "SIM_Box3DConRel.h"
#include "SIM_Box3DParms.h"
#include "core/Box3DBond.h"
#include "core/Box3DWorld.h"
#include "sync/Box3DHoudiniUtil.h"

#include <SIM/SIM_DopDescription.h>
#include <UT/UT_Quaternion.h>
#include <UT/UT_Vector3.h>
#include <UT/UT_XformOrder.h>

#include <cstring>
#include <new>
#include <string>
#include <vector>

SIM_Box3DConRel::SIM_Box3DConRel(const SIM_DataFactory *factory)
    : BaseClass(factory), SIM_OptionsUser(this) {}

SIM_Box3DConRel::~SIM_Box3DConRel() {}

b3dop::JointSettings SIM_Box3DConRel::settings() const {
  const b3dop::JointType type = jointType();
  b3dop::JointSettings s = b3dop::Box3DWorld::defaultJointSettings(type);
  const auto &opts = getOptions();
  for (const b3dop::JointParam &p : b3dop::jointParams()) {
    if (!p.usedBy(type))
      continue;
    float v[3] = {0, 0, 0};
    if (p.kind == b3dop::JointParam::Toggle)
      v[0] = opts.getOptionB(p.name) ? 1.0f : 0.0f;
    else if (b3dop::jointParamSize(p) == 1)
      v[0] = static_cast<float>(opts.getOptionF(p.name));
    else {
      const UT_Vector3D o = opts.getOptionV3(p.name);
      for (unsigned i = 0; i < 3; ++i)
        v[i] = static_cast<float>(o(i));
    }
    b3dop::setJointParam(s, p, type, v, box3dEulerToQuat);
  }
  // A node toggle that is off (Breakable, Set Rest Length, ...) means
  // "unlimited / never": -1.
  for (const b3dop::JointParam &p : b3dop::jointParams()) {
    const b3dop::JointParam *toggle = b3dop::jointNodeToggleOf(p);
    if (!toggle || !p.usedBy(type) || !toggle->usedBy(type) ||
        opts.getOptionB(toggle->name))
      continue;
    const float off[3] = {-1.0f, -1.0f, -1.0f};
    b3dop::setJointParam(s, p, type, off, box3dEulerToQuat);
  }
  // A Bond's material replaces its strengths (times the scale), which a
  // primitive attribute then modifies like any setting.
  if (type == b3dop::JointType::Bond) {
    const auto &materials = b3dop::bondMaterials();
    const int index = static_cast<int>(opts.getOptionI("bond_material")) - 1;
    if (index >= 0 && index < static_cast<int>(materials.size())) {
      const b3dop::BondMaterial &m = materials[static_cast<size_t>(index)];
      const float scale =
          static_cast<float>(opts.getOptionF("bond_material_scale"));
      s.bondTensile = m.tensile * scale;
      s.bondShear = m.shear * scale;
      s.bondBending = m.bending * scale;
      s.bondCompressive = m.compressive * scale;
      s.bondFracture = m.fracture;
    }
  }
  return s;
}

bool SIM_Box3DConRel::attributesReplace() const {
  return jointType() != b3dop::JointType::Filter &&
         getOptions().getOptionB(b3dop::kAttributesReplaceName);
}

const SIM_DopDescription *
SIM_Box3DConRel::describe(b3dop::JointType t,
                          const UT_StringHolder &classname) {
  using b3dop::JointParam;

  // Per joint type, kept for the life of the plugin (Houdini keeps pointers
  // into it).
  struct Storage {
    Box3DParms parms;
    std::string nodeName, label, dataName;
    SIM_DopDescription *description = nullptr;
  };
  static Storage storage[b3dop::kJointTypeCount];

  // Each description is built in zero-filled static memory, never on the heap.
  // That is how every other description exists (SideFX's and our Solver's and
  // Box3D Data's are function-local statics, zeroed before their constructor
  // runs). A heap description crashed Houdini now and then while the joint
  // node cooked (in DOP_Auto::cookMe, at SIM_DopDescription::computeLiveInput):
  // consistent with a member the constructor leaves unset, which only static
  // memory guarantees to be zero. Never destroyed: Houdini keeps the pointer.
  alignas(SIM_DopDescription) static unsigned char
      descriptionMemory[b3dop::kJointTypeCount][sizeof(SIM_DopDescription)];

  Storage &st = storage[static_cast<int>(t)];
  if (st.description)
    return st.description;

  // A "Joint" folder with the settings every type has, then a folder with the
  // type's own. Each is named like the primitive attribute that modifies it.
  // Settings a toggle switches on (springs, motors, limits) come right after it
  // and are disabled while it is off. Parameters are saved by name, so the
  // order is free to change. Defaults: nodeJointDefaults, in Houdini units;
  // node toggles (Breakable, ...) start off.
  constexpr unsigned kCommon =
      b3dop::kAllJoints & ~b3dop::jointBit(b3dop::JointType::Filter);
  const b3dop::JointSettings d =
      b3dop::nodeJointDefaults(t, b3dop::Box3DWorld::defaultJointSettings(t));
  st.parms.folder("Joint");
  if (t != b3dop::JointType::Filter)
    st.parms.toggle(b3dop::kAttributesReplaceName, "Attributes Replace Values",
                    false,
                    "Float primitive attributes set the value (degrees for "
                    "angles). Off: they multiply this node's value.");
  bool ownTab = false;
  for (const JointParam *pp : b3dop::jointParamUiOrder(t)) {
    const JointParam &p = *pp;
    if (!ownTab && (p.types & kCommon) != kCommon) {
      st.parms.folder(b3dop::jointTypeLabel(t));
      ownTab = true;
      if (t == b3dop::JointType::Bond) {
        std::vector<const char *> items = {"Custom"};
        for (const b3dop::BondMaterial &m : b3dop::bondMaterials())
          items.push_back(m.label);
        st.parms.menu("bond_material", "Material", 0, items,
                      "A material's typical strengths in place of the ones "
                      "below. Custom uses them.");
        Box3DParms::Range scaleRange;
        scaleRange.min = 0.001;
        scaleRange.max = 10.0;
        st.parms.number("bond_material_scale", "Material Strength Scale", 1.0,
                        scaleRange,
                        "Multiplies the material's strengths. 1 is the typical "
                        "value.",
                        "{ bond_material == 0 }");
      }
    }
    std::string disableWhen = b3dop::jointParamDisableWhen(p, t);
    if (t == b3dop::JointType::Bond &&
        (std::strcmp(p.name, "bond_tensile") == 0 ||
         std::strcmp(p.name, "bond_shear") == 0 ||
         std::strcmp(p.name, "bond_bending") == 0 ||
         std::strcmp(p.name, "bond_compressive") == 0 ||
         std::strcmp(p.name, "bond_fracture_energy") == 0))
      disableWhen = "{ bond_material != 0 }";
    if (!p.b && p.kind == JointParam::Toggle) // node toggle, not a setting
    {
      st.parms.toggle(p.name, p.label, false, p.help, disableWhen.c_str());
      continue;
    }
    float v[3];
    b3dop::getJointParam(d, p, t, v);
    if (p.kind == JointParam::Toggle)
      st.parms.toggle(p.name, p.label, v[0] != 0.0f, p.help,
                      disableWhen.c_str());
    else if (b3dop::jointParamSize(p) == 1) {
      const b3dop::JointRange r = b3dop::jointParamRange(p, t);
      Box3DParms::Range range;
      range.min = r.min;
      range.max = r.max;
      range.hardMin = r.hardMin;
      range.hardMax = r.hardMax;
      st.parms.number(p.name, p.label, v[0], range, p.help,
                      disableWhen.c_str());
    } else
      st.parms.vector(p.name, p.label, {v[0], v[1], v[2]}, p.help, PRM_FLT_J,
                      disableWhen.c_str());
  }

  const std::string typeName = b3dop::jointTypeName(t);
  st.nodeName = "box3d_" + typeName + "conrel";
  st.label = std::string("Box3D ") + b3dop::jointTypeLabel(t) +
             " Constraint Relationship";
  // What `constraint_name` refers to by default: capitalised, like SideFX's own
  // relationship nodes (Glue, Hard, ConeTwist).
  st.dataName = b3dop::jointTypeLabel(t);

  st.description = new (descriptionMemory[static_cast<int>(t)])
      SIM_DopDescription(true, // also creates a DOP node that applies this data
                         st.nodeName.c_str(), // internal node name
                         st.label.c_str(),    // node label
                         st.dataName.c_str(), // default Data Name
                         classname, st.parms.templates());
  return st.description;
}
