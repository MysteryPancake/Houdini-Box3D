// Box3DJointSync.cpp
#include "Box3DJointSync.h"
#include "Box3DHoudiniUtil.h"
#include "core/Box3DDebug.h"
#include "dop/SIM_Box3DConRel.h"
#include "dop/SIM_Box3DWorld.h"

#include <GA/GA_Handle.h>
#include <GA/GA_Iterator.h>
#include <GA/GA_Primitive.h>
#include <GU/GU_Detail.h>
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
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace {

// The Constraint Network DOP's attribute and group names.
const char *const kConstraintName = "constraint_name";
const char *const kConstraintType = "constraint_type"; // SideFX data only
const char *const kNextConstraintName = "next_constraint_name";
const char *const kBrokenGroup = "broken";
// With Attach Internal Constraints to Object, the network is this data on the
// object itself.
const char *const kInternalGeometry = "ConstraintGeometry";
// Ours: the length a distance joint made with a negative Rest Length started
// at.
const char *const kAutoRestLength = "auto_restlength";
// Where each live joint's anchors are after the last step (point attributes).
// P / orient stay the authored anchors: Houdini reads them as the anchors'
// initial world frames and moves its constraint guide with the pieces itself,
// so writing the live frames there drew every moving anchor twice as far.
const char *const kCurrentP = "current_P";
const char *const kCurrentOrient = "current_orient";
// The Constraint Network's anchor attributes: anchor_id (point attribute, int)
// binds an anchor to a point (or vertex) of the named object; anchor_type
// (string "point" / "vertex" / "agent", or an int 0 / 1 / 2) says which.
const char *const kAnchorId = "anchor_id";
const char *const kAnchorType = "anchor_type";

// The Constraint Network data a relationship carries, or null for other
// relationships.
const SIM_ConstraintNetworkRelationship *
networkData(const SIM_Relationship *rel) {
  const SIM_Data *type = rel ? rel->getConstRelationshipTypeData() : nullptr;
  return type ? SIM_DATA_CASTCONST(type, SIM_ConstraintNetworkRelationship)
              : nullptr;
}

// Where a network was found: a relationship of `object` (by name), or, with an
// empty name, the object's ConstraintGeometry.
struct NetworkSource {
  bool inGroup = true;
  std::string relName;
};

const SIM_Relationship *findRelationship(const SIM_Object &obj,
                                         const NetworkSource &src) {
  return src.relName.empty()
             ? nullptr
             : obj.getRelationship(src.inGroup, src.relName.c_str());
}

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
        for (int k = 0; k < 3; ++k)
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

uint64_t idOf(const GA_Attribute *a) {
  return a ? static_cast<uint64_t>(a->getDataId()) : ~0ull;
}

// Everything the joints are read from: the geometry's identity, topology, and
// the data ids of every attribute and group the sync reads. Equal: nothing to
// re-read.
uint64_t networkFingerprint(const GU_Detail *gdp) {
  using b3dop::mixHash;
  uint64_t h = geometryIdentity(gdp);
  h = mixHash(h, idOf(gdp->findPointAttribute("name")));
  h = mixHash(h, idOf(gdp->findPointAttribute("orient")));
  h = mixHash(h, idOf(gdp->findPointAttribute(kCurrentP)));
  h = mixHash(h, idOf(gdp->findPointAttribute(kCurrentOrient)));
  h = mixHash(h, idOf(gdp->findPointAttribute(kAnchorId)));
  h = mixHash(h, idOf(gdp->findPointAttribute(kAnchorType)));
  h = mixHash(h, idOf(gdp->findPrimitiveAttribute(kConstraintName)));
  h = mixHash(h, idOf(gdp->findPrimitiveAttribute(kAutoRestLength)));
  h = mixHash(h, idOf(gdp->findPrimitiveAttribute(kConstraintType)));
  for (const b3dop::JointParam &p : b3dop::jointParams())
    h = mixHash(h, idOf(gdp->findPrimitiveAttribute(p.name)));
  for (const b3dop::SideFXParam &p : b3dop::sideFXParams())
    h = mixHash(h, idOf(gdp->findPrimitiveAttribute(p.name)));
  const GA_PrimitiveGroup *broken = gdp->findPrimitiveGroup(kBrokenGroup);
  return mixHash(h,
                 broken ? static_cast<uint64_t>(broken->getDataId()) : ~0ull);
}

// A resolved anchor: its body (0: the world) and, for a point body, its mirror
// entry (the rest pose).
struct AnchorBody {
  b3dop::BodyHandle handle = 0;
  const Box3DBodyMirror *entry = nullptr;
  // The DOP object of a whole-object body (an anchor_id names a point of its
  // geometry), else null.
  const SIM_Object *object = nullptr;
};

// An anchor's anchor_type.
enum class AnchorKind { Point, Vertex, Agent };

// World position of point (or vertex) `id` of the object's geometry through its
// geometry transform. False if the object has no geometry or no such point.
bool pointAnchorWorld(const SIM_Object &obj, int id, bool vertex,
                      double out[3]) {
  const SIM_Geometry *geo = obj.getGeometry();
  if (!geo || id < 0)
    return false;
  SIM_GeometryAutoReadLock lock(geo);
  const GU_Detail *gdp = lock.getGdp();
  if (!gdp)
    return false;
  GA_Offset pt = GA_INVALID_OFFSET;
  if (vertex) {
    if (static_cast<GA_Size>(id) >= gdp->getNumVertices())
      return false;
    pt = gdp->vertexPoint(gdp->vertexOffset(static_cast<GA_Index>(id)));
  } else {
    if (static_cast<GA_Size>(id) >= gdp->getNumPoints())
      return false;
    pt = gdp->pointOffset(static_cast<GA_Index>(id));
  }
  if (!GAisValid(pt))
    return false;
  UT_DMatrix4 xf;
  SIMgetGeometryTransform(xf, obj);
  const UT_Vector3 p = gdp->getPos3(pt);
  UT_Vector3D w(p.x(), p.y(), p.z());
  w.rowVecMult(xf);
  for (int k = 0; k < 3; ++k) {
    if (!std::isfinite(w[k]))
      return false;
    out[k] = w[k];
  }
  return true;
}

// Anchor names -> bodies, over the objects of this solve (in their order) and
// the affectors. The name tables are built on the first resolve(): a step whose
// networks are all unchanged never resolves a name, so it pays nothing for
// them. After that a name is a hash lookup, not a scan of every object per
// anchor.
class Anchors {
public:
  Anchors(const Box3DSharedState &st, SIM_ObjectArray &objects) : myState(st) {
    for (exint i = 0; i < objects.entries(); ++i) {
      const SIM_Object *obj = objects(i);
      const auto it = st.objectSync.find(obj->getObjectId());
      if (it == st.objectSync.end() || it->second.visited != st.syncStamp)
        continue;
      const UT_String &nm = obj->getName();
      myObjects.push_back(
          {nm.isstring() ? std::string(nm.buffer()) : std::string(),
           &it->second, obj});
    }
  }

  // Empty: the world (0). Else a piece's `name` in any object, "object/piece",
  // an object with a single body, or an affector object. False if nothing
  // matches.
  bool resolve(const std::string &name, AnchorBody &out) const {
    out = AnchorBody();
    if (name.empty())
      return true;
    buildTables();
    const size_t slash = name.find('/');
    if (slash != std::string::npos) {
      // The first object with this name decides, whether or not it has the
      // piece.
      const auto o = myObjectByName.find(name.substr(0, slash));
      return o != myObjectByName.end() &&
             findIn(*o->second, name.substr(slash + 1), out);
    }
    // Order: a piece in any object, an object with one body, an affector.
    for (const auto *table : {&myPiece, &mySingleBody, &myAffector}) {
      const auto it = table->find(name);
      if (it != table->end()) {
        out = it->second;
        return true;
      }
    }
    return false;
  }

private:
  using HandleTable = std::unordered_map<std::string, AnchorBody>;

  static AnchorBody bodyOf(const Box3DBodyMirror *e) {
    AnchorBody b;
    b.handle = e->handle;
    b.entry = e;
    return b;
  }

  static bool findIn(const Box3DObjectSync &os, const std::string &piece,
                     AnchorBody &out) {
    const auto it = os.byName.find(piece);
    if (it == os.byName.end() || !it->second->handle)
      return false;
    out = bodyOf(it->second);
    return true;
  }

  // Each table keeps the first match in object order (what a scan in that order
  // would find).
  void buildTables() const {
    if (myBuilt)
      return;
    myBuilt = true;
    for (const auto &o : myObjects) {
      myObjectByName.try_emplace(o.name, o.sync);
      for (const auto &p : o.sync->byName)
        if (p.second->handle)
          myPiece.try_emplace(p.first, bodyOf(p.second));
      if (o.sync->bodies.size() == 1 && o.sync->bodies[0]->handle) {
        AnchorBody b = bodyOf(o.sync->bodies[0]);
        // A whole object's points are its geometry's: anchor_id can name one.
        if (b.entry->wholeObject)
          b.object = o.object;
        mySingleBody.try_emplace(o.name, b);
      }
    }
    for (const auto &kv : myState.affectors)
      if (kv.second.handle) {
        AnchorBody b;
        b.handle = kv.second.handle;
        myAffector.try_emplace(kv.second.name, b);
      }
  }

  const Box3DSharedState &myState;
  struct ObjectRef {
    std::string name;
    const Box3DObjectSync *sync;
    const SIM_Object *object;
  };
  std::vector<ObjectRef> myObjects;

  mutable bool myBuilt = false;
  mutable std::unordered_map<std::string, const Box3DObjectSync *>
      myObjectByName;
  mutable HandleTable myPiece;
  mutable HandleTable mySingleBody;
  mutable HandleTable myAffector;
};

// The primitive attributes that override a setting, bound once per network.
struct SettingReader {
  const b3dop::JointParam *param = nullptr;
  GA_ROHandleI i;
  GA_ROHandleF f;
  GA_ROHandleV3 v;
};

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
      b3dop::applyFloatAttribute(s, p, type, r.f.get(prim), replaceFloats);
      continue;
    }
    float v[3] = {0, 0, 0};
    if (p.kind == JointParam::Toggle)
      v[0] = static_cast<float>(r.i.get(prim));
    else {
      const UT_Vector3 a = r.v.get(prim);
      v[0] = a.x();
      v[1] = a.y();
      v[2] = a.z();
    }
    b3dop::setJointParam(s, p, type, v, box3dEulerToQuat);
  }
}

// The primitive attributes that modify a SideFX data's options
// (b3dop::sideFXParams), bound once per network.
struct SideFXReader {
  const b3dop::SideFXParam *param = nullptr;
  GA_ROHandleF f;
  GA_ROHandleI i;
  GA_ROHandleV3 v;
};

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
    case SideFXParam::Float:
      s.*p.f *= r.f.get(prim);
      break;
    case SideFXParam::Int:
      s.*p.i = r.i.get(prim);
      break;
    case SideFXParam::Vec3: {
      const UT_Vector3 a = r.v.get(prim);
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

b3dop::JointFrame anchorFrameOf(const b3dop::JointFrame &joint,
                                const b3dop::JointFrame &adj) {
  return b3dop::isIdentityOffset(adj) ? joint
                                      : b3dop::removeFrameOffset(joint, adj);
}

// What a joint is made from: its type and bodies, plus its frames when they are
// not live (a network that is not written back keeps the authored frames, so a
// change there is an edit).
uint64_t jointSignature(b3dop::JointType type, b3dop::BodyHandle a,
                        b3dop::BodyHandle b, const b3dop::JointFrame &fa,
                        const b3dop::JointFrame &fb, bool withFrames) {
  using b3dop::mixHash;
  uint64_t h = mixHash(
      mixHash(mixHash(b3dop::kHashSeed, static_cast<uint64_t>(type)), a), b);
  if (withFrames)
    for (const b3dop::JointFrame *f : {&fa, &fb}) {
      for (int k = 0; k < 3; ++k)
        h = b3dop::mixDouble(h, f->pos[k]);
      h = b3dop::hashBytes(h, f->rot, sizeof(f->rot));
    }
  return h ? h : 1;
}

// An anchor moved away from what the solver last wrote: edited upstream (q and
// -q are equal).
bool frameEdited(const b3dop::JointFrame &now, const b3dop::JointFrame &wrote) {
  for (int k = 0; k < 3; ++k)
    if (std::fabs(now.pos[k] - wrote.pos[k]) >
        1e-5 * std::max(1.0, std::fabs(wrote.pos[k])))
      return true;
  return b3dop::quatsDiffer(now.rot, wrote.rot);
}

// A frame as the geometry stores it (float P), so it compares equal when read
// back.
b3dop::JointFrame asStored(const b3dop::JointFrame &f) {
  b3dop::JointFrame out = f;
  for (int k = 0; k < 3; ++k)
    out.pos[k] = static_cast<double>(static_cast<float>(f.pos[k]));
  return out;
}

// What processNetwork() reads from a network's geometry, bound once.
struct NetworkReader {
  const GU_Detail *gdp;
  const GA_ROHandleS conName, conType, name;
  const GA_ROHandleF autoRest;
  const GA_ROHandleQ orient, localQ, currentQ;
  const GA_ROHandleV3 localP, currentP;
  const GA_PrimitiveGroup *broken;
  const GA_ROHandleI anchorIdH, anchorTypeI;
  const GA_ROHandleS anchorTypeS;

  explicit NetworkReader(const GU_Detail *g)
      : gdp(g), conName(g, GA_ATTRIB_PRIMITIVE, kConstraintName),
        conType(g, GA_ATTRIB_PRIMITIVE, kConstraintType),
        name(g, GA_ATTRIB_POINT, "name"),
        autoRest(g, GA_ATTRIB_PRIMITIVE, kAutoRestLength),
        orient(g, GA_ATTRIB_POINT, "orient"),
        localQ(g, GA_ATTRIB_POINT, "local_orient"),
        currentQ(g, GA_ATTRIB_POINT, kCurrentOrient),
        localP(g, GA_ATTRIB_POINT, "local_P"),
        currentP(g, GA_ATTRIB_POINT, kCurrentP),
        broken(g->findPrimitiveGroup(kBrokenGroup)),
        anchorIdH(g, GA_ATTRIB_POINT, kAnchorId),
        anchorTypeI(g, GA_ATTRIB_POINT, kAnchorType),
        anchorTypeS(g, GA_ATTRIB_POINT, kAnchorType) {}

  // The anchor's anchor_id: -1 (also: no attribute) is the object's own
  // offset, which P / orient already give.
  int anchorId(GA_Offset pt) const {
    return anchorIdH.isValid() ? anchorIdH.get(pt) : -1;
  }

  // The anchor's anchor_type: a string ("point", "vertex", "agent") or an int
  // (0, 1, 2); missing or unknown: point.
  AnchorKind anchorKind(GA_Offset pt) const {
    if (anchorTypeS.isValid()) {
      const char *t = static_cast<const char *>(anchorTypeS.get(pt));
      if (t && UT_String(t).equal("vertex", false))
        return AnchorKind::Vertex;
      if (t && UT_String(t).equal("agent", false))
        return AnchorKind::Agent;
    } else if (anchorTypeI.isValid()) {
      const int t = anchorTypeI.get(pt);
      if (t == 1)
        return AnchorKind::Vertex;
      if (t == 2)
        return AnchorKind::Agent;
    }
    return AnchorKind::Point;
  }

  std::string anchorName(GA_Offset pt) const {
    const char *n =
        name.isValid() ? static_cast<const char *>(name.get(pt)) : nullptr;
    return n ? std::string(n) : std::string();
  }

  // The anchor as authored: its P / orient, world space (the solver never
  // writes them).
  b3dop::JointFrame authored(GA_Offset pt) const {
    b3dop::JointFrame f;
    const UT_Vector3 p = gdp->getPos3(pt);
    f.pos[0] = p.x();
    f.pos[1] = p.y();
    f.pos[2] = p.z();
    if (orient.isValid()) {
      const UT_QuaternionF q = orient.get(pt);
      f.rot[0] = q.x();
      f.rot[1] = q.y();
      f.rot[2] = q.z();
      f.rot[3] = q.w();
    }
    return f;
  }

  // Where the anchor was after the last step (current_P / current_orient, a
  // zero quaternion: none), if the solver wrote it.
  bool live(GA_Offset pt, b3dop::JointFrame &out) const {
    if (!currentP.isValid() || !currentQ.isValid())
      return false;
    const UT_Vector3 p = currentP.get(pt);
    const UT_QuaternionF q = currentQ.get(pt);
    if ((q.x() == 0.0f && q.y() == 0.0f && q.z() == 0.0f && q.w() == 0.0f) ||
        !std::isfinite(p.x()) || !std::isfinite(p.y()) || !std::isfinite(p.z()))
      return false;
    out.pos[0] = p.x();
    out.pos[1] = p.y();
    out.pos[2] = p.z();
    out.rot[0] = q.x();
    out.rot[1] = q.y();
    out.rot[2] = q.z();
    out.rot[3] = q.w();
    return true;
  }

  // The anchor's frame now: live if written, else as authored.
  b3dop::JointFrame frame(GA_Offset pt) const {
    b3dop::JointFrame f;
    return live(pt, f) ? f : authored(pt);
  }

  // The primitive's constraint_type (SideFX data only; missing: all).
  b3dop::ConstraintMode mode(GA_Offset prim) const {
    const char *t = conType.isValid()
                        ? static_cast<const char *>(conType.get(prim))
                        : nullptr;
    return b3dop::constraintModeFromString(t);
  }

  // The primitive's constraint data (by `constraint_name`), or null.
  const Box3DJointKind *
  kind(GA_Offset prim,
       const std::unordered_map<std::string, Box3DJointKind> &kinds) const {
    const char *n = conName.isValid()
                        ? static_cast<const char *>(conName.get(prim))
                        : nullptr;
    if (!n || !*n)
      return nullptr;
    const auto it = kinds.find(std::string(n));
    if (it != kinds.end())
      return &it->second;
    // No exact match: ignore case, so lowercase names (the joint nodes' Data
    // Names before they were capitalised, e.g. "weld") still find "Weld".
    for (const auto &kv : kinds)
      if (UT_String(kv.first.c_str()).equal(n, false))
        return &kv.second;
    return nullptr;
  }

  // The anchor's saved local frame (local_P / local_orient), if it has one (a
  // zero quaternion: none).
  bool savedLocal(GA_Offset pt, b3dop::JointFrame &out) const {
    if (!localP.isValid() || !localQ.isValid())
      return false;
    const UT_Vector3 p = localP.get(pt);
    const UT_QuaternionF q = localQ.get(pt);
    if ((q.x() == 0.0f && q.y() == 0.0f && q.z() == 0.0f && q.w() == 0.0f) ||
        !std::isfinite(p.x()) || !std::isfinite(p.y()) || !std::isfinite(p.z()))
      return false;
    out.pos[0] = p.x();
    out.pos[1] = p.y();
    out.pos[2] = p.z();
    out.rot[0] = q.x();
    out.rot[1] = q.y();
    out.rot[2] = q.z();
    out.rot[3] = q.w();
    return true;
  }
};

// A saved local frame still describes its anchor: through the body's pose now
// it lands on the anchor's P / orient (written together after the same step). A
// renamed, moved or new anchor does not, and is made from P / orient instead.
bool localFrameAgrees(const b3dop::Box3DWorld &world, b3dop::BodyHandle body,
                      const b3dop::JointFrame &local,
                      const b3dop::JointFrame &live) {
  b3dop::JointFrame w;
  if (!world.localToWorld(body, local, w))
    return false;
  for (int k = 0; k < 3; ++k)
    if (std::fabs(w.pos[k] - live.pos[k]) >
        1e-5 * std::max(1.0, std::fabs(live.pos[k])))
      return false;
  return !b3dop::quatsDiffer(w.rot, live.rot);
}

// An authored anchor frame (world space at rest) moved with its body since
// rest, as Bullet maps anchors through restxform: so a joint made after its
// pieces moved (a new primitive, a remake after an upstream edit) attaches
// where the anchor sits on the piece now. Unchanged for the world, affectors,
// whole objects and points without a rest; identity before anything moved.
b3dop::JointFrame restToNow(const b3dop::Box3DWorld &world,
                            const AnchorBody &body,
                            const b3dop::JointFrame &f) {
  const Box3DBodyMirror *e = body.entry;
  double pos[3];
  float rot[4];
  if (!body.handle || !e || !e->packed || !e->hasRest ||
      !world.getTransformByHandle(body.handle, pos, rot))
    return f;
  float since[4];
  box3dAbsoluteToRest(e->restRot, rot, since);
  UT_DMatrix3 d;
  UT_QuaternionD(since[0], since[1], since[2], since[3]).getRotationMatrix(d);
  const UT_Vector3D offset(f.pos[0] - e->restPos[0], f.pos[1] - e->restPos[1],
                           f.pos[2] - e->restPos[2]);
  const UT_Vector3D moved = offset * d;
  b3dop::JointFrame out;
  for (int k = 0; k < 3; ++k)
    out.pos[k] = moved[k] + pos[k];
  float frameRot[4] = {f.rot[0], f.rot[1], f.rot[2], f.rot[3]};
  b3dop::sanitizeQuat(frameRot);
  box3dRestToAbsolute(frameRot, since, out.rot);
  return out;
}

// A body's rotation at rest for the `angle` reading: its restxform's, else its
// rotation now (when the joint is made), identity for the world.
void restRotationOf(const b3dop::Box3DWorld &world, const AnchorBody &body,
                    float out[4]) {
  out[0] = out[1] = out[2] = 0.0f;
  out[3] = 1.0f;
  if (!body.handle)
    return;
  if (body.entry && body.entry->packed && body.entry->hasRest) {
    b3dop::copy4(out, body.entry->restRot);
    return;
  }
  double pos[3];
  if (!world.getTransformByHandle(body.handle, pos, out)) {
    out[0] = out[1] = out[2] = 0.0f;
    out[3] = 1.0f;
  }
}

struct NetworkRef {
  SIM_Object *object =
      nullptr; // an object of this solve the network was found on
  NetworkSource source;
  uint64_t key = 0;
};

// A primitive's joint: type and settings from its constraint data (with the
// primitive's own overrides), and the frame offsets of SideFX data.
struct JointSpec {
  b3dop::JointType type = b3dop::JointType::Weld;
  b3dop::JointSettings s;
  b3dop::JointFrame adjA,
      adjB; // the frames relative to the anchors: identity but for SideFX data
            // and Hold Separation
  uint64_t frameSig = 0;
};

// One network's primitives, read from its geometry, into joints: make, update,
// drop.
class NetworkPass {
public:
  NetworkPass(Box3DSharedState &st, const NetworkRef &ref,
              Box3DJointNetwork &net, const Anchors &anchors,
              const GU_Detail *gdp, float timeStep, Box3DJointCounts &counts)
      : mySt(st), myWorld(st.world), myRef(ref), myNet(net), myAnchors(anchors),
        myGdp(gdp), myTimeStep(timeStep), myCounts(counts), myRd(gdp),
        myReaders(bindSettings(gdp)), mySfReaders(bindSideFX(gdp)) {}

  void run() {
    for (GA_Iterator it(myGdp->getPrimitiveRange()); !it.atEnd(); ++it) {
      const GA_Offset off = *it;
      const GA_Primitive *prim = myGdp->getPrimitive(off);
      if (prim)
        processPrimitive(*prim, off);
    }
  }

private:
  void dropJoint(Box3DJointEntry &e) {
    myWorld.destroyJoint(e.handle); // harmless when already gone with a body
    e.handle = 0;
  }

  // The joint entry of a primitive (made on first sight).
  Box3DJointEntry &entryFor(GA_Offset off) {
    auto ins = mySt.joints.try_emplace(b3dop::mixHash(
        myRef.key, static_cast<uint64_t>(static_cast<int64_t>(off))));
    Box3DJointEntry &e = ins.first->second;
    if (ins.second) {
      e.network = myRef.key;
      e.prim = off;
    }
    return e;
  }

  // The type, settings and frame offsets for the primitive.
  JointSpec specFor(const Box3DJointKind &kind, GA_Offset off,
                    b3dop::BodyHandle a, b3dop::BodyHandle b,
                    const b3dop::JointFrame &fa,
                    const b3dop::JointFrame &fb) const {
    JointSpec j;
    if (kind.sideFX) {
      // SideFX data: the primitive's overrides on its options, then mapped onto
      // a Box3D joint.
      b3dop::SideFXSettings sf = kind.sideFXSettings;
      applySideFXOverrides(mySfReaders, kind.sideFXType, off, sf);
      b3dop::SideFXContext ctx;
      ctx.timeStep = myTimeStep;
      ctx.a = sideFXBody(myWorld, a);
      ctx.b = sideFXBody(myWorld, b);
      const b3dop::SideFXJoint m =
          b3dop::mapSideFX(kind.sideFXType, sf, myRd.mode(off), ctx);
      j.type = m.type;
      j.s = m.settings;
      j.frameSig = sideFXFrameSignature(m);
      b3dop::sideFXFrameOffsets(m, fa, fb, j.adjA, j.adjB);
    } else {
      j.type = kind.type;
      j.s = kind.settings;
      applyOverrides(myReaders, j.type, off, kind.attributesReplace, j.s);
      // A distance joint that measured its own rest length keeps it (see
      // writeLiveFrames).
      if (j.type == b3dop::JointType::Distance && !(j.s.restLength >= 0.0f) &&
          myRd.autoRest.isValid()) {
        const float made = myRd.autoRest.get(off);
        if (made >= 0.0f)
          j.s.restLength = made;
      }
      // Hold Separation: the frame offsets SideFX's Glue gets, so the weld
      // holds the anchors as far apart as they are when it is made.
      if (j.type == b3dop::JointType::Weld && j.s.holdSeparation) {
        b3dop::SideFXJoint m;
        m.holdSeparation = true;
        j.frameSig = sideFXFrameSignature(m);
        b3dop::sideFXFrameOffsets(m, fa, fb, j.adjA, j.adjB);
      }
    }
    return j;
  }

  // (Re)make the joint from scratch. Exactly as before from the saved local
  // frames when they still apply (a world rebuilt mid-simulation); else from P
  // / orient, and the local frames are saved after the step. The local frames
  // are the joint's (with any SideFX offset), so they are compared with the
  // frames the anchors give now.
  void makeJoint(Box3DJointEntry &e, const JointSpec &j, const AnchorBody &ab,
                 const AnchorBody &bb, GA_Offset p0, GA_Offset p1,
                 const b3dop::JointFrame &fa, const b3dop::JointFrame &fb,
                 uint64_t sig, bool edited, bool fromLive) {
    const b3dop::BodyHandle a = ab.handle, b = bb.handle;
    dropJoint(e);
    e.type = j.type;
    e.bodyA = a;
    e.bodyB = b;
    e.sig = sig;
    e.settings = j.s;
    e.wrote = false;
    e.adjA = j.adjA;
    e.adjB = j.adjB;
    e.madeRestLength = -1.0f;
    if (j.type == b3dop::JointType::Distance && !(j.s.restLength >= 0.0f)) {
      const double dx = fb.pos[0] - fa.pos[0], dy = fb.pos[1] - fa.pos[1],
                   dz = fb.pos[2] - fa.pos[2];
      e.madeRestLength =
          static_cast<float>(std::sqrt(dx * dx + dy * dy + dz * dz));
    }
    restRotationOf(myWorld, ab, e.restRotA);
    restRotationOf(myWorld, bb, e.restRotB);
    const b3dop::JointFrame ja = jointFrameOf(fa, j.adjA),
                            jb = jointFrameOf(fb, j.adjB);
    b3dop::JointFrame la, lb;
    const bool useLocal = !edited && myRd.savedLocal(p0, la) &&
                          myRd.savedLocal(p1, lb) &&
                          localFrameAgrees(myWorld, a, la, ja) &&
                          localFrameAgrees(myWorld, b, lb, jb);
    // Live frames are where the anchors are now; authored ones are at rest, so
    // a distance joint measuring its own length takes it at rest (what
    // auto_restlength keeps), not between the moved anchors.
    b3dop::JointSettings restSettings = j.s;
    if (e.madeRestLength >= 0.0f)
      restSettings.restLength = e.madeRestLength;
    e.handle =
        useLocal ? myWorld.createJoint(j.type, a, b, la, lb, j.s, &e, true)
        : fromLive
            ? myWorld.createJoint(j.type, a, b, ja, jb, j.s, &e)
            : myWorld.createJoint(j.type, a, b, restToNow(myWorld, ab, ja),
                                  restToNow(myWorld, bb, jb), restSettings, &e);
    e.writeLocal = !useLocal;
    if (e.handle)
      ++myCounts.created;
    else
      ++myCounts.failed;
  }

  // The anchor as authored (P / orient). With an anchor_id on a whole object it
  // sits at that point (or vertex) of the object's geometry, as it was the
  // first time this joint saw it: kept in the entry, so the object moving is no
  // edit. `bad` is set when the id names no point.
  b3dop::JointFrame authoredAnchor(Box3DJointEntry &e, bool second,
                                   GA_Offset pt, const AnchorBody &body,
                                   bool &bad) const {
    b3dop::JointFrame f = myRd.authored(pt);
    b3dop::JointFrame &pin = second ? e.pinB : e.pinA;
    int &pinId = second ? e.pinIdB : e.pinIdA;
    bool &pinVertex = second ? e.pinVertexB : e.pinVertexA;
    const int id = myRd.anchorId(pt);
    if (id < 0 || !body.handle || !body.object) {
      pinId = -1; // no point to follow (the id is ignored on a piece or world)
      return f;
    }
    const bool vertex = myRd.anchorKind(pt) == AnchorKind::Vertex;
    if (pinId != id || pinVertex != vertex) {
      double w[3];
      if (!pointAnchorWorld(*body.object, id, vertex, w)) {
        bad = true;
        return f;
      }
      for (int k = 0; k < 3; ++k)
        pin.pos[k] = w[k];
      pinId = id;
      pinVertex = vertex;
    }
    for (int k = 0; k < 3; ++k)
      f.pos[k] = pin.pos[k];
    return f;
  }

  void processPrimitive(const GA_Primitive &prim, GA_Offset off) {
    if (prim.getVertexCount() != 2) {
      ++myCounts.notLines;
      return;
    }
    const Box3DJointKind *kind = myRd.kind(off, myNet.kinds);
    if (!kind) {
      ++myCounts.unknown;
      return;
    }

    Box3DJointEntry &e = entryFor(off);

    // Broken (by us, or marked in the geometry): never remade.
    if (e.broken || (myRd.broken && myRd.broken->containsOffset(off))) {
      e.broken = true;
      dropJoint(e);
      e.stamp = myNet.liveStamp;
      return;
    }

    const GA_Offset p0 = prim.getPointOffset(0), p1 = prim.getPointOffset(1);
    // Agent anchors name a transform of an agent's rig: nothing here simulates
    // one.
    if (myRd.anchorKind(p0) == AnchorKind::Agent ||
        myRd.anchorKind(p1) == AnchorKind::Agent) {
      ++myCounts.agentAnchors;
      dropJoint(e);
      return;
    }
    AnchorBody ab, bb;
    if (!myAnchors.resolve(myRd.anchorName(p0), ab) ||
        !myAnchors.resolve(myRd.anchorName(p1), bb)) {
      ++myCounts.unresolved;
      dropJoint(e);
      return; // not stamped: removed; retried when bodies change
    }
    const b3dop::BodyHandle a = ab.handle, b = bb.handle;
    // On a written network current_P / current_orient say where the anchors
    // are now, so a world rebuilt mid-simulation makes its joints from them
    // exactly as frame 1 does from the authored P / orient.
    bool badAnchor = false;
    const b3dop::JointFrame authA = asStored(
                                authoredAnchor(e, false, p0, ab, badAnchor)),
                            authB = asStored(
                                authoredAnchor(e, true, p1, bb, badAnchor));
    if (badAnchor) {
      ++myCounts.badAnchorId;
      dropJoint(e);
      return; // not stamped: removed
    }
    b3dop::JointFrame la, lb;
    const bool hasLive = myRd.live(p0, la) && myRd.live(p1, lb);
    // Edited upstream (a SOP Solver): the authored anchors differ from the ones
    // the joint was made from, or the live ones from what was written. Remade
    // from the authored anchors then (current_P is the old joint's).
    const bool authoredEdited =
        e.made && (frameEdited(authA, e.authA) || frameEdited(authB, e.authB));
    const bool liveEdited =
        hasLive && e.wrote &&
        (frameEdited(la, e.wroteA) || frameEdited(lb, e.wroteB));
    const bool useLive = hasLive && !authoredEdited;
    const b3dop::JointFrame fa = useLive ? la : authA,
                            fb = useLive ? lb : authB;
    const JointSpec j = specFor(*kind, off, a, b, fa, fb);

    const bool live = myNet.writable;
    uint64_t sig = jointSignature(j.type, a, b, fa, fb, !live);
    if (j.frameSig)
      sig = b3dop::mixHash(sig, j.frameSig) | 1;
    const bool edited = live && (authoredEdited || liveEdited);

    if (myWorld.isJointValid(e.handle) && e.sig == sig && e.type == j.type &&
        !edited) {
      if (!b3dop::sameJointSettings(j.type, j.s, e.settings)) {
        myWorld.updateJoint(e.handle, j.type, j.s);
        e.settings = j.s;
        ++myCounts.updated;
      }
    } else {
      makeJoint(e, j, ab, bb, p0, p1, fa, fb, sig, edited, useLive);
      e.authA = authA;
      e.authB = authB;
      e.made = true;
    }
    e.stamp = myNet.liveStamp;
  }

  Box3DSharedState &mySt;
  b3dop::Box3DWorld &myWorld;
  const NetworkRef &myRef;
  Box3DJointNetwork &myNet;
  const Anchors &myAnchors;
  const GU_Detail *myGdp;
  const float myTimeStep;
  Box3DJointCounts &myCounts;
  const NetworkReader myRd;
  const std::vector<SettingReader> myReaders;
  const std::vector<SideFXReader> mySfReaders;
};

void processNetwork(Box3DSharedState &st, const NetworkRef &ref,
                    const Anchors &anchors, float timeStep,
                    Box3DJointCounts &counts) {
  Box3DJointNetwork &net = st.jointNetworks[ref.key];
  net.visited = st.syncStamp;
  net.objectId = ref.object->getObjectId();
  net.relName = ref.source.relName;
  net.inGroup = ref.source.inGroup;

  // The constraint data the network can name; any change re-reads every
  // primitive.
  std::unordered_map<std::string, Box3DJointKind> kinds =
      networkKinds(*ref.object, ref.source);
  const bool kindsChanged = !net.kindsKnown || !sameKinds(kinds, net.kinds);
  if (kindsChanged)
    net.kinds = std::move(kinds);
  net.kindsKnown = true;

  const SIM_Geometry *geo = networkGeometry(*ref.object, ref.source);
  if (!geo)
    return;
  SIM_GeometryAutoReadLock lock(geo);
  const GU_Detail *gdp = lock.getGdp();
  if (!gdp)
    return;

  // Unchanged geometry, bodies and constraint data: the joints stay as they are
  // (they keep liveStamp).
  const uint64_t fp = networkFingerprint(gdp);
  if (fp == net.fingerprint && net.bodyVersion == st.bodyVersion &&
      !kindsChanged)
    return;
  net.fingerprint = fp;
  net.bodyVersion = st.bodyVersion;
  net.liveStamp = st.syncStamp;

  NetworkPass(st, ref, net, anchors, gdp, timeStep, counts).run();
}

// After the step, for a written network: every live joint's anchors get
// current_P / current_orient set to where its frames are now (P / orient stay
// as authored), and joints made from P / orient save
// their local frames to local_P / local_orient (relative to their pieces: exact
// on a rebuild, wherever the pieces are). A distance joint made with a negative
// rest length gets its starting distance written to `auto_restlength` (not
// `restlength`, which multiplies the node's value), so a world rebuilt
// mid-simulation keeps it.
template <class PrimOk>
void writeLiveFrames(const b3dop::Box3DWorld &world, GU_Detail *gdp,
                     const std::vector<Box3DJointEntry *> &entries,
                     const PrimOk &primOk) {
  if (entries.empty())
    return;
  auto ensure = [&](const char *name, int size, bool quat, bool &created) {
    created = false;
    if (!gdp->findPointAttribute(name)) {
      gdp->addFloatTuple(GA_ATTRIB_POINT, name, size);
      created = true;
      if (quat)
        if (GA_Attribute *attr = gdp->findPointAttribute(name))
          attr->setTypeInfo(GA_TYPE_QUATERNION);
    }
  };
  // A new current_orient is 0,0,0,0 on every point: "not written" for the
  // points of joints that are not live (see NetworkReader::live).
  bool created = false;
  ensure(kCurrentP, 3, false, created);
  ensure(kCurrentOrient, 4, true, created);
  GA_RWHandleV3 pH(gdp, GA_ATTRIB_POINT, kCurrentP);
  GA_RWHandleQ orientH(gdp, GA_ATTRIB_POINT, kCurrentOrient);
  if (!pH.isValid() || !orientH.isValid())
    return;
  bool wroteRestLength = false, wroteLocal = false;
  GA_RWHandleF restLengthH;
  GA_RWHandleV3 localPH;
  GA_RWHandleQ localQH;

  for (Box3DJointEntry *e : entries) {
    if (!primOk(e->prim))
      continue;
    const GA_Primitive *prim = gdp->getPrimitive(e->prim);
    if (!prim || prim->getVertexCount() != 2)
      continue;
    const GA_Offset pts[2] = {prim->getPointOffset(0), prim->getPointOffset(1)};
    if (e->madeRestLength >= 0.0f) {
      if (!restLengthH.isValid()) {
        if (!gdp->findPrimitiveAttribute(kAutoRestLength))
          gdp->addFloatTuple(GA_ATTRIB_PRIMITIVE, kAutoRestLength, 1,
                             GA_Defaults(-1.0f));
        restLengthH = GA_RWHandleF(gdp, GA_ATTRIB_PRIMITIVE, kAutoRestLength);
      }
      if (restLengthH.isValid()) {
        restLengthH.set(e->prim, e->madeRestLength);
        wroteRestLength = true;
      }
      e->madeRestLength = -1.0f;
    }

    if (e->writeLocal) {
      b3dop::JointFrame la, lb;
      if (world.jointLocalFrames(e->handle, la, lb)) {
        if (!localPH.isValid()) {
          ensure("local_P", 3, false, created);
          ensure("local_orient", 4, true, created);
          localPH = GA_RWHandleV3(gdp, GA_ATTRIB_POINT, "local_P");
          localQH = GA_RWHandleQ(gdp, GA_ATTRIB_POINT, "local_orient");
        }
        if (localPH.isValid() && localQH.isValid()) {
          const b3dop::JointFrame *frames[2] = {&la, &lb};
          for (int k = 0; k < 2; ++k) {
            const b3dop::JointFrame &f = *frames[k];
            localPH.set(pts[k], UT_Vector3(static_cast<float>(f.pos[0]),
                                           static_cast<float>(f.pos[1]),
                                           static_cast<float>(f.pos[2])));
            localQH.set(pts[k],
                        UT_QuaternionF(f.rot[0], f.rot[1], f.rot[2], f.rot[3]));
          }
          wroteLocal = true;
        }
      }
      e->writeLocal = false;
    }

    b3dop::JointFrame fa, fb;
    if (world.jointWorldFrames(e->handle, fa, fb)) {
      // The anchors, not the frames (SideFX data may offset the frames from
      // them).
      e->wroteA = asStored(anchorFrameOf(fa, e->adjA));
      e->wroteB = asStored(anchorFrameOf(fb, e->adjB));
      e->wrote = true;
    } else if (!e->switchFrames)
      continue;              // broken, or never made
    e->switchFrames = false; // (switched: wroteA / wroteB are set already)
    const b3dop::JointFrame *frames[2] = {&e->wroteA, &e->wroteB};
    for (int k = 0; k < 2; ++k) {
      const b3dop::JointFrame &f = *frames[k];
      pH.set(pts[k], UT_Vector3(static_cast<float>(f.pos[0]),
                                static_cast<float>(f.pos[1]),
                                static_cast<float>(f.pos[2])));
      orientH.set(pts[k],
                  UT_QuaternionF(f.rot[0], f.rot[1], f.rot[2], f.rot[3]));
    }
  }
  pH.getAttribute()->bumpDataId();
  orientH.getAttribute()->bumpDataId();
  if (wroteRestLength)
    restLengthH.getAttribute()->bumpDataId();
  if (wroteLocal) {
    localPH.getAttribute()->bumpDataId();
    localQH.getAttribute()->bumpDataId();
  }
}

} // namespace

bool anyJointNetwork(SIM_ObjectArray &objects) {
  for (exint i = 0; i < objects.entries(); ++i) {
    const SIM_Object *obj = objects(i);
    if (obj->getConstNamedSubData(kInternalGeometry))
      return true;
    for (const bool inGroup : {true, false})
      for (int r = 0; r < obj->getNumRelationships(inGroup); ++r)
        if (networkData(obj->getRelationship(inGroup, r)))
          return true;
  }
  return false;
}

void syncJoints(Box3DSharedState &st, SIM_ObjectArray &objects, float timeStep,
                Box3DJointCounts &counts) {
  // ---- gather: each Constraint Network relationship once (it is on every
  // object it constrains), and each object's internal ConstraintGeometry ----
  std::vector<NetworkRef> refs;
  std::unordered_set<std::string> seenRels;
  for (exint i = 0; i < objects.entries(); ++i) {
    SIM_Object *obj = objects(i);
    for (const bool inGroup : {true, false})
      for (int r = 0; r < obj->getNumRelationships(inGroup); ++r) {
        const SIM_Relationship *rel = obj->getRelationship(inGroup, r);
        if (!networkData(rel))
          continue;
        NetworkRef ref;
        ref.object = obj;
        ref.source.inGroup = inGroup;
        const UT_String relName = rel->getName();
        ref.source.relName =
            relName.isstring() ? std::string(relName.buffer()) : std::string();
        if (ref.source.relName.empty() ||
            !seenRels.insert(ref.source.relName).second)
          continue;
        ref.key = b3dop::mixHash(b3dop::hashBytes(b3dop::kHashSeed,
                                                  ref.source.relName.data(),
                                                  ref.source.relName.size()),
                                 1);
        refs.push_back(ref);
      }
    if (obj->getConstNamedSubData(kInternalGeometry)) {
      NetworkRef ref;
      ref.object = obj;
      ref.key = b3dop::mixHash(
          static_cast<uint64_t>(static_cast<uint32_t>(obj->getObjectId())), 2);
      refs.push_back(ref);
    }
  }
  counts.networks = static_cast<int>(refs.size());

  if (!refs.empty()) {
    const Anchors anchors(st, objects);
    for (const NetworkRef &ref : refs)
      processNetwork(st, ref, anchors, timeStep, counts);
  }

  // ---- remove: joints whose primitive or network went away ----
  for (auto it = st.joints.begin(); it != st.joints.end();) {
    const Box3DJointEntry &e = it->second;
    const auto net = st.jointNetworks.find(e.network);
    const bool live = net != st.jointNetworks.end() &&
                      net->second.visited == st.syncStamp &&
                      e.stamp == net->second.liveStamp;
    if (live) {
      ++it;
      continue;
    }
    if (st.world.isJointValid(e.handle)) {
      st.world.destroyJoint(e.handle);
      ++counts.removed;
    }
    it = st.joints.erase(it);
  }
  for (auto it = st.jointNetworks.begin(); it != st.jointNetworks.end();)
    it = (it->second.visited == st.syncStamp) ? std::next(it)
                                              : st.jointNetworks.erase(it);
}

namespace {

using EntriesByNetwork =
    std::unordered_map<uint64_t, std::vector<Box3DJointEntry *>>;

// Joints over their force / torque threshold: read their local frames (a next
// constraint may start from them), then destroy them. Read the events before
// destroying anything.
int breakJoints(Box3DSharedState &st, EntriesByNetwork &brokeIn) {
  b3dop::Box3DWorld &world = st.world;
  std::vector<void *> events;
  if (st.brokenCollected)
    events = st.brokenThisFrame;
  else
    world.brokenJoints(events);
  int broke = 0;
  for (void *p : events) {
    Box3DJointEntry *e = static_cast<Box3DJointEntry *>(p);
    if (!e || !e->handle)
      continue;
    e->brokeLocalKnown =
        world.jointLocalFrames(e->handle, e->brokeLocalA, e->brokeLocalB);
    world.destroyJoint(e->handle);
    e->handle = 0;
    e->broken = true;
    brokeIn[e->network].push_back(e);
    ++broke;
  }
  return broke;
}

// Whether a primitive offset still names a primitive of the (written) geometry.
struct PrimExists {
  const GU_Detail *gdp;
  bool operator()(GA_Offset off) const {
    return off >= 0 && off < gdp->getNumPrimitiveOffsets() &&
           gdp->getPrimitive(off) != nullptr;
  }
};

// A broken joint with a next_constraint_name switches to it: its joint is
// remade as that constraint at the next step, from where the pieces are now,
// and the attribute is cleared. The anchors go where the broken joint's frames
// are now (writeLiveFrames).
void switchToNextConstraint(const b3dop::Box3DWorld &world,
                            Box3DJointEntry &e) {
  e.broken = false;
  b3dop::JointFrame fa, fb;
  if (e.brokeLocalKnown && world.localToWorld(e.bodyA, e.brokeLocalA, fa) &&
      world.localToWorld(e.bodyB, e.brokeLocalB, fb)) {
    e.wroteA = asStored(anchorFrameOf(fa, e.adjA));
    e.wroteB = asStored(anchorFrameOf(fb, e.adjB));
    e.wrote = true;
    e.switchFrames = true;
  }
}

// The joints that broke: a primitive with a next_constraint_name switches to
// it, the rest go into the `broken` group. True if any switched.
bool writeBrokenJoints(const b3dop::Box3DWorld &world, GU_Detail *gdp,
                       const PrimExists &primOk,
                       const std::vector<Box3DJointEntry *> &broken) {
  GA_RWHandleS conNameH(gdp, GA_ATTRIB_PRIMITIVE, kConstraintName);
  GA_RWHandleS nextH(gdp, GA_ATTRIB_PRIMITIVE, kNextConstraintName);
  GA_PrimitiveGroup *grp = nullptr;
  bool switched = false;
  for (Box3DJointEntry *e : broken) {
    if (!primOk(e->prim))
      continue;
    UT_StringHolder nextName;
    if (nextH.isValid() && conNameH.isValid())
      nextName = nextH.get(e->prim);
    if (nextName.isstring()) {
      conNameH.set(e->prim, nextName);
      nextH.set(e->prim, UT_StringHolder());
      switchToNextConstraint(world, *e);
      switched = true;
      continue;
    }
    if (!grp) {
      grp = gdp->findPrimitiveGroup(kBrokenGroup);
      if (!grp)
        grp = gdp->newPrimitiveGroup(kBrokenGroup);
    }
    if (grp)
      grp->addOffset(e->prim);
  }
  if (grp)
    grp->bumpDataId();
  if (switched) {
    conNameH.getAttribute()->bumpDataId();
    nextH.getAttribute()->bumpDataId();
  }
  return switched;
}

// The bodies' relative rotation since rest, in radians (Bullet's `angle`): 0
// while the joint holds them as they started, however they were rotated then.
// Box3D's own reading compares the bodies' absolute rotations instead.
float angleSinceRest(const b3dop::Box3DWorld &world, const Box3DJointEntry &e,
                     float fallback) {
  float now[2][4] = {{0, 0, 0, 1}, {0, 0, 0, 1}};
  const b3dop::BodyHandle bodies[2] = {e.bodyA, e.bodyB};
  for (int i = 0; i < 2; ++i) {
    double pos[3];
    if (bodies[i] && !world.getTransformByHandle(bodies[i], pos, now[i]))
      return fallback;
  }
  float sinceA[4], sinceB[4], rel[4];
  box3dAbsoluteToRest(e.restRotA, now[0], sinceA);
  box3dAbsoluteToRest(e.restRotB, now[1], sinceB);
  box3dAbsoluteToRest(sinceA, sinceB, rel);
  const float w = std::min(1.0f, std::fabs(rel[3]));
  return 2.0f * std::acos(w);
}

// Feedback, under the Constraint Network DOP's names: force / distance for the
// linear part, torque / angle (radians) for the angular part, plus the per-type
// readings.
void writeFeedback(const b3dop::Box3DWorld &world, GU_Detail *gdp,
                   const PrimExists &primOk,
                   const std::vector<Box3DJointEntry *> &entries) {
  std::unordered_map<std::string, GA_RWHandleF> scalars;
  auto scalar = [&](const char *name) -> GA_RWHandleF & {
    auto it = scalars.find(name);
    if (it == scalars.end()) {
      if (!gdp->findPrimitiveAttribute(name))
        gdp->addFloatTuple(GA_ATTRIB_PRIMITIVE, name, 1);
      it = scalars.emplace(name, GA_RWHandleF(gdp, GA_ATTRIB_PRIMITIVE, name))
               .first;
    }
    return it->second;
  };
  auto length = [](const float *v) {
    return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
  };
  GA_RWHandleF &forceH = scalar("force");
  GA_RWHandleF &distH = scalar("distance");
  GA_RWHandleF &torqueH = scalar("torque");
  GA_RWHandleF &angleH = scalar("angle");
  for (const Box3DJointEntry *e : entries) {
    b3dop::JointFeedback fb;
    if (!primOk(e->prim) || !world.jointFeedback(e->handle, e->type, fb))
      continue;
    if (forceH.isValid())
      forceH.set(e->prim, length(fb.force));
    if (distH.isValid())
      distH.set(e->prim, fb.linearSeparation);
    if (torqueH.isValid())
      torqueH.set(e->prim, length(fb.torque));
    if (angleH.isValid())
      angleH.set(e->prim, angleSinceRest(world, *e, fb.angularSeparation));
    for (int k = 0; k < fb.extraCount; ++k) {
      GA_RWHandleF &h = scalar(fb.extraName[k]);
      if (h.isValid())
        h.set(e->prim, fb.extraIsAngle[k] ? fb.extra[k] * b3dop::kRadToDeg
                                          : fb.extra[k]);
    }
  }
  for (auto &sc : scalars)
    if (sc.second.isValid())
      sc.second.getAttribute()->bumpDataId();
}

// One network's results into its (locked, writable) geometry. True if a
// constraint_name switched.
bool writeNetworkResults(const b3dop::Box3DWorld &world, GU_Detail *gdp,
                         const std::vector<Box3DJointEntry *> &entries,
                         const std::vector<Box3DJointEntry *> *broken) {
  const PrimExists primOk{gdp};
  const bool switched =
      broken && writeBrokenJoints(world, gdp, primOk, *broken);
  writeLiveFrames(world, gdp, entries, primOk);
  writeFeedback(world, gdp, primOk, entries);
  return switched;
}

} // namespace

int jointsAfterStep(Box3DSharedState &st, SIM_ObjectArray &objects) {
  b3dop::Box3DWorld &world = st.world;

  EntriesByNetwork brokeIn; // network -> joints that broke
  const int broke = breakJoints(st, brokeIn);

  // ---- write-back: broken group / next_constraint_name, live frames, feedback
  // ----
  EntriesByNetwork entriesOf;
  bool entriesGathered = false;
  for (auto &kv : st.jointNetworks) {
    Box3DJointNetwork &net = kv.second;
    const auto broken = brokeIn.find(kv.first);
    const bool anyBroken = broken != brokeIn.end();
    if (net.visited != st.syncStamp)
      continue;
    if (!entriesGathered) {
      entriesGathered = true;
      for (auto &je : st.joints)
        entriesOf[je.second.network].push_back(&je.second);
    }
    const auto ents = entriesOf.find(kv.first);
    if (ents == entriesOf.end() && !anyBroken)
      continue; // no joints to keep live, nothing broke
    SIM_Object *obj = box3dObjectById(objects, net.objectId, st.objectsSorted);
    NetworkSource src;
    src.inGroup = net.inGroup;
    src.relName = net.relName;
    SIM_GeometryCopy *geo = obj ? writableNetworkGeometry(*obj, src) : nullptr;
    net.writable = geo != nullptr;
    if (!geo)
      continue;
    static const std::vector<Box3DJointEntry *> kNone;
    const std::vector<Box3DJointEntry *> &entries =
        ents != entriesOf.end() ? ents->second : kNone;
    bool switched = false;
    {
      SIM_GeometryAutoWriteLock lock(geo);
      GU_Detail *gdp = &lock.getGdp();
      switched = writeNetworkResults(world, gdp, entries,
                                     anyBroken ? &broken->second : nullptr);
    }
    // Our own write is not an edit: re-record the fingerprint (the lock is
    // released). A switched constraint_name is ours too, but must be read:
    // force a re-read.
    if (const SIM_Geometry *rgeo = networkGeometry(*obj, src)) {
      SIM_GeometryAutoReadLock rlock(rgeo);
      if (const GU_Detail *rgdp = rlock.getGdp())
        net.fingerprint = networkFingerprint(rgdp);
    }
    if (switched)
      net.fingerprint = 0;
  }
  return broke;
}
