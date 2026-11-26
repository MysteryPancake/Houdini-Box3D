// Box3DJointRead.cpp
//
// Reading a constraint network: its geometry and fingerprint, the settings
// attributes of the joint nodes and of SideFX data, and the table of constraint
// kinds.
#include "Box3DHoudiniUtil.h"
#include "Box3DJointNetwork.h"
#include "Box3DJointSync.h"
#include "Box3DJointSyncInternal.h"
#include "core/Box3DDebug.h"
#include "dop/SIM_Box3DConRel.h"
#include "dop/SIM_Box3DWorld.h"
#include <GA/GA_Handle.h>
#include <GA/GA_Iterator.h>
#include <GA/GA_Primitive.h>
#include <GU/GU_Detail.h>
#include <GU/GU_DetailHandle.h>
#include <GU/GU_PackedFragment.h>
#include <GU/GU_PrimPacked.h>
#include <SIM/SIM_ConstraintNetworkRelationship.h>
#include <SIM/SIM_Geometry.h>
#include <SIM/SIM_GeometryCopy.h>
#include <SIM/SIM_Object.h>
#include <SIM/SIM_ObjectArray.h>
#include <SIM/SIM_Relationship.h>
#include <SIM/SIM_RelationshipArray.h>
#include <SIM/SIM_Utils.h>
#include <UT/UT_Matrix4.h>
#include <UT/UT_Quaternion.h>
#include <UT/UT_String.h>
#include <UT/UT_StringHolder.h>
#include <UT/UT_Vector3.h>
#include <algorithm>
#include <cmath>
#include <string>
#include <string_view>
#include <unordered_map>
#include <unordered_set>
#include <vector>

using namespace jointsync;

namespace jointsync {

// The Constraint Network data a relationship carries, or null for other
// relationships.
const SIM_ConstraintNetworkRelationship *
networkData(const SIM_Relationship *rel) {
  const SIM_Data *type = rel ? rel->getConstRelationshipTypeData() : nullptr;
  return type ? SIM_DATA_CASTCONST(type, SIM_ConstraintNetworkRelationship)
              : nullptr;
}

} // namespace jointsync

namespace {

const SIM_Relationship *findRelationship(const SIM_Object &obj,
                                         const NetworkSource &src) {
  return src.relName.empty()
             ? nullptr
             : obj.getRelationship(src.inGroup, src.relName.c_str());
}

} // namespace

const SIM_Geometry *networkGeometry(const SIM_Object &obj,
                                    const NetworkSource &src) {
  if (src.relName.empty())
    return SIM_DATA_GETCONST(obj, kInternalGeometry, SIM_Geometry);
  const SIM_ConstraintNetworkRelationship *cn =
      networkData(findRelationship(obj, src));
  return cn ? cn->getGeometry() : nullptr;
}

SIM_GeometryCopy *writableNetworkGeometry(SIM_Object &obj,
                                          const NetworkSource &src) {
  if (src.relName.empty())
    return box3dWritableGeometry(obj, kInternalGeometry);
  SIM_Relationship *rel = obj.getRelationships(src.inGroup)
                              .findRelationshipByName(src.relName.c_str());
  SIM_Data *type = rel ? rel->getRelationshipTypeData() : nullptr;
  SIM_ConstraintNetworkRelationship *cn =
      type ? SIM_DATA_CAST(type, SIM_ConstraintNetworkRelationship) : nullptr;
  return cn ? cn->getGeometry() : nullptr;
}

namespace {

// A SideFX constraint data's options (b3dop::sideFXParams), over Houdini's
// defaults. Their classes are not in the HDK: read by name (box3dDataOption).
b3dop::SideFXSettings readSideFXSettings(const SIM_Data &data,
                                         b3dop::SideFXType type) {
  using b3dop::SideFXParam;
  b3dop::SideFXSettings s = b3dop::defaultSideFXSettings(type);
  for (const SideFXParam &p : b3dop::sideFXParams()) {
    if (!p.usedBy(type))
      continue;
    if (p.kind == SideFXParam::Vec3) {
      UT_Vector3D v;
      if (box3dDataOption(data, p.name, v))
        for (unsigned k = 0; k < 3; ++k)
          (s.*p.v)[k] = static_cast<float>(v(k));
      continue;
    }
    fpreal64 v = 0.0;
    if (!box3dDataOption(data, p.name, v))
      continue;
    if (p.kind == SideFXParam::Float)
      s.*p.f = static_cast<float>(v);
    else
      s.*p.i = static_cast<int>(v);
  }
  return s;
}

} // namespace

namespace jointsync {

// The Box3D constraint relationships a network's `constraint_name` can refer
// to, by data name. The Constraint Network DOP attaches the nodes wired into
// its second input under the network's relationship; the relationship, its
// Constraint Network data and the geometry are all searched (the first name
// found wins), and for an internal network the geometry and the object.
// SideFX's Glue / Hard / Spring / Bullet Soft / Cone Twist data count too
// (mapped per primitive).
std::unordered_map<std::string, Box3DJointKind>
networkKinds(const SIM_Object &obj, const NetworkSource &src) {
  std::vector<const SIM_Data *> roots;
  if (src.relName.empty()) {
    roots.push_back(obj.getConstNamedSubData(kInternalGeometry));
    roots.push_back(&obj);
  } else if (const SIM_Relationship *rel = findRelationship(obj, src)) {
    roots.push_back(rel);
    const SIM_ConstraintNetworkRelationship *cn = networkData(rel);
    roots.push_back(cn);
    roots.push_back(cn ? cn->getGeometry() : nullptr);
  }

  std::unordered_map<std::string, Box3DJointKind> kinds;
  for (const SIM_Data *root : roots) {
    if (!root)
      continue;
    for (int i = 0; i < root->getNumSubData(); ++i) {
      const SIM_Data *sub = root->getConstSubData(i);
      if (!sub)
        continue;
      const SIM_Box3DConRel *conrel = SIM_DATA_CASTCONST(sub, SIM_Box3DConRel);
      b3dop::SideFXType sideFXType = b3dop::SideFXType::Glue;
      const bool sideFX =
          !conrel &&
          b3dop::sideFXTypeFromDataType(sub->getDataType().c_str(), sideFXType);
      if (!conrel && !sideFX)
        continue;
      const UT_StringHolder &name = root->getSubDataName(i);
      if (!name.isstring() || kinds.count(std::string(name.c_str())))
        continue;
      Box3DJointKind k;
      if (conrel) {
        k.type = conrel->jointType();
        k.settings = conrel->settings();
        k.attributesReplace = conrel->attributesReplace();
      } else {
        k.sideFX = true;
        k.sideFXType = sideFXType;
        k.sideFXSettings = readSideFXSettings(*sub, sideFXType);
      }
      kinds.emplace(std::string(name.c_str()), k);
    }
  }
  return kinds;
}

bool sameKinds(const std::unordered_map<std::string, Box3DJointKind> &a,
               const std::unordered_map<std::string, Box3DJointKind> &b) {
  if (a.size() != b.size())
    return false;
  for (const auto &kv : a) {
    const auto it = b.find(kv.first);
    if (it == b.end() || it->second.sideFX != kv.second.sideFX)
      return false;
    if (kv.second.sideFX) {
      if (it->second.sideFXType != kv.second.sideFXType ||
          !b3dop::sameSideFXSettings(kv.second.sideFXType,
                                     kv.second.sideFXSettings,
                                     it->second.sideFXSettings))
        return false;
    } else if (it->second.type != kv.second.type ||
               it->second.attributesReplace != kv.second.attributesReplace ||
               !b3dop::sameJointSettings(kv.second.type, kv.second.settings,
                                         it->second.settings))
      return false;
  }
  return true;
}

} // namespace jointsync

namespace {

uint64_t idOf(const GA_Attribute *a) {
  return a ? static_cast<uint64_t>(a->getDataId()) : ~0ull;
}

} // namespace

// Everything the joints are read from: the geometry's identity, topology, and
// the data ids of every attribute and group the sync reads. Equal: nothing to
// re-read. Not the detail's unique id: the engine copies the geometry into a
// new detail every timestep, and the copy has the same contents.
uint64_t networkFingerprint(const GU_Detail *gdp) {
  using b3dop::mixHash;
  uint64_t h = geometryIdentity(gdp, false);
  h = mixHash(h, idOf(gdp->findPointAttribute("name")));
  h = mixHash(h, idOf(gdp->findPointAttribute("orient")));
  h = mixHash(h, idOf(gdp->findPointAttribute(kCurrentP)));
  h = mixHash(h, idOf(gdp->findPointAttribute(kCurrentOrient)));
  h = mixHash(h, idOf(gdp->findPointAttribute(kAnchorId)));
  h = mixHash(h, idOf(gdp->findPointAttribute(kAnchorType)));
  h = mixHash(h, idOf(gdp->findPrimitiveAttribute(kConstraintName)));
  h = mixHash(h, idOf(gdp->findPrimitiveAttribute(kAutoRestLength)));
  h = mixHash(h, idOf(gdp->findPrimitiveAttribute(kConstraintType)));
  h = mixHash(h, idOf(gdp->findGlobalAttribute(kPropagateIteration)));
  for (const b3dop::JointParam &p : b3dop::jointParams())
    h = mixHash(h, idOf(gdp->findPrimitiveAttribute(p.name)));
  for (const b3dop::SideFXParam &p : b3dop::sideFXParams())
    h = mixHash(h, idOf(gdp->findPrimitiveAttribute(p.name)));
  const GA_PrimitiveGroup *broken = gdp->findPrimitiveGroup(kBrokenGroup);
  return mixHash(h,
                 broken ? static_cast<uint64_t>(broken->getDataId()) : ~0ull);
}

namespace jointsync {

std::vector<SettingReader> bindSettings(const GU_Detail *gdp) {
  std::vector<SettingReader> out;
  for (const b3dop::JointParam &p : b3dop::jointParams()) {
    if (!gdp->findPrimitiveAttribute(p.name))
      continue;
    SettingReader r;
    r.param = &p;
    switch (p.kind) {
    case b3dop::JointParam::Toggle:
      r.i = GA_ROHandleI(gdp, GA_ATTRIB_PRIMITIVE, p.name);
      if (!r.i.isValid())
        continue;
      break;
    case b3dop::JointParam::Float:
    case b3dop::JointParam::Angle:
      r.f = GA_ROHandleF(gdp, GA_ATTRIB_PRIMITIVE, p.name);
      if (!r.f.isValid())
        continue;
      break;
    default:
      r.v = GA_ROHandleV3(gdp, GA_ATTRIB_PRIMITIVE, p.name);
      if (!r.v.isValid())
        continue;
      break;
    }
    out.push_back(r);
  }
  return out;
}

// The primitive's attributes on top of its constraint data's settings, as the
// Constraint Network DOP documents: a float attribute multiplies the setting
// (angles too: it is a factor, not degrees), any other kind replaces it
// (vectors and Euler angles in Houdini units). With `replaceFloats` (the node's
// Attributes Replace Values) floats replace too, in Houdini units.
void applyOverrides(const std::vector<SettingReader> &readers,
                    b3dop::JointType type, GA_Offset prim, bool replaceFloats,
                    b3dop::JointSettings &s) {
  using b3dop::JointParam;
  for (const SettingReader &r : readers) {
    const JointParam &p = *r.param;
    if (!p.usedBy(type))
      continue;
    if (p.kind == JointParam::Float || p.kind == JointParam::Angle) {
      const float value = r.f.get(prim);
      if (!b3dop::isUnsetFloat(value))
        b3dop::applyFloatAttribute(s, p, type, value, replaceFloats);
      continue;
    }
    float v[3] = {0, 0, 0};
    if (p.kind == JointParam::Toggle) {
      const int value = static_cast<int>(r.i.get(prim));
      if (value < 0)
        continue; // not set
      v[0] = static_cast<float>(value);
    } else {
      const UT_Vector3 a = r.v.get(prim);
      if (b3dop::isUnsetFloat(a.x()))
        continue;
      v[0] = a.x();
      v[1] = a.y();
      v[2] = a.z();
    }
    b3dop::setJointParam(s, p, type, v, box3dEulerToQuat);
  }
}

std::vector<SideFXReader> bindSideFX(const GU_Detail *gdp) {
  using b3dop::SideFXParam;
  std::vector<SideFXReader> out;
  for (const SideFXParam &p : b3dop::sideFXParams()) {
    if (!gdp->findPrimitiveAttribute(p.name))
      continue;
    SideFXReader r;
    r.param = &p;
    bool ok = false;
    switch (p.kind) {
    case SideFXParam::Float:
      r.f = GA_ROHandleF(gdp, GA_ATTRIB_PRIMITIVE, p.name);
      ok = r.f.isValid();
      break;
    case SideFXParam::Int:
      r.i = GA_ROHandleI(gdp, GA_ATTRIB_PRIMITIVE, p.name);
      ok = r.i.isValid();
      break;
    case SideFXParam::Vec3:
      r.v = GA_ROHandleV3(gdp, GA_ATTRIB_PRIMITIVE, p.name);
      ok = r.v.isValid();
      break;
    }
    if (ok)
      out.push_back(r);
  }
  return out;
}

// The Constraint Network DOP's rule on SideFX data too: floats multiply, the
// rest replace.
void applySideFXOverrides(const std::vector<SideFXReader> &readers,
                          b3dop::SideFXType type, GA_Offset prim,
                          b3dop::SideFXSettings &s) {
  using b3dop::SideFXParam;
  for (const SideFXReader &r : readers) {
    const SideFXParam &p = *r.param;
    if (!p.usedBy(type))
      continue;
    switch (p.kind) {
    case SideFXParam::Float: {
      // "Not set" (Box3D Constraint Properties writes it on lines it does not
      // edit, under names both tables share): leave the option alone.
      const float value = r.f.get(prim);
      if (b3dop::isUnsetFloat(value))
        break;
      // Glue's Strength has its own rule for negative values.
      if (type == b3dop::SideFXType::Glue &&
          p.f == &b3dop::SideFXSettings::strength)
        s.strength = b3dop::combineGlueStrength(s.strength, value);
      else
        s.*p.f *= value;
      break;
    }
    case SideFXParam::Int:
      s.*p.i = r.i.get(prim);
      break;
    case SideFXParam::Vec3: {
      const UT_Vector3 a = r.v.get(prim);
      if (b3dop::isUnsetFloat(a.x()))
        break;
      (s.*p.v)[0] = a.x();
      (s.*p.v)[1] = a.y();
      (s.*p.v)[2] = a.z();
      break;
    }
    }
  }
}

// A body as the SideFX mapping needs it: Spring converts its stiffness from the
// masses.
b3dop::SideFXBody sideFXBody(const b3dop::Box3DWorld &world,
                             b3dop::BodyHandle h) {
  b3dop::SideFXBody body;
  if (!h || world.typeByHandle(h) != b3dop::BodySettings::Type::Dynamic)
    return body; // the world, static or kinematic
  float center[3], inertia[3];
  if (world.getMassDataByHandle(h, body.mass, center, inertia)) {
    body.dynamic = body.mass > 0.0f;
    body.inertia = (inertia[0] + inertia[1] + inertia[2]) / 3.0f;
  }
  return body;
}

// Two readings of a body are the same for the Spring conversion (the same mass
// and inertia, to a part in ten thousand).
bool sameSideFXBody(const b3dop::SideFXBody &a, const b3dop::SideFXBody &b) {
  const auto close = [](float x, float y) {
    return std::fabs(x - y) <= 1e-4f * std::max(std::fabs(x), std::fabs(y));
  };
  return a.dynamic == b.dynamic && close(a.mass, b.mass) &&
         close(a.inertia, b.inertia);
}

// What decides where a SideFX joint's frames sit (not the anchors' positions:
// those are live).
uint64_t sideFXFrameSignature(const b3dop::SideFXJoint &m) {
  uint64_t h = b3dop::hashBytes(b3dop::kHashSeed, m.rotA, sizeof(m.rotA));
  h = b3dop::hashBytes(h, m.rotB, sizeof(m.rotB));
  h = b3dop::hashBytes(h, &m.separation, sizeof(m.separation));
  return b3dop::mixHash(h, m.holdSeparation ? 1 : 0);
}

// A joint frame from its anchor and offset, and back. Identity offsets (Box3D
// nodes) pass the frame through untouched, bit for bit.
b3dop::JointFrame jointFrameOf(const b3dop::JointFrame &anchor,
                               const b3dop::JointFrame &adj) {
  return b3dop::isIdentityOffset(adj) ? anchor
                                      : b3dop::applyFrameOffset(anchor, adj);
}

} // namespace jointsync

b3dop::JointFrame anchorFrameOf(const b3dop::JointFrame &joint,
                                const b3dop::JointFrame &adj) {
  return b3dop::isIdentityOffset(adj) ? joint
                                      : b3dop::removeFrameOffset(joint, adj);
}
