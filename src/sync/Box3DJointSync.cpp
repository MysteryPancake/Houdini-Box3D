// Box3DJointSync.cpp
#include "Box3DJointSync.h"
#include "Box3DHoudiniUtil.h"
#include "Box3DJointNetwork.h"
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
#include <cctype>
#include <cmath>
#include <string>
#include <string_view>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace {

// More attribute names are in Box3DJointNetwork.h (shared with the
// after-step code).
const char *const kConstraintType = "constraint_type"; // SideFX data only
// With Attach Internal Constraints to Object, the network is this data on the
// object itself.
const char *const kInternalGeometry = "ConstraintGeometry";
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

namespace {

// A resolved anchor: its body (0: the world) and, for a point body, its mirror
// entry (the rest pose).
struct AnchorBody {
  b3dop::BodyHandle handle = 0;
  const Box3DBodyMirror *entry = nullptr;
  // The DOP object of a body in a solved object (an anchor_id names a point of
  // a whole object's geometry, or of a packed piece's own), else null.
  const SIM_Object *object = nullptr;
  // Found as an object with a single body: adding or removing a body of that
  // object changes it.
  bool byObjectName = false;
};

// An anchor's anchor_type.
enum class AnchorKind { Point, Vertex, Agent };

// Point `id` of `gdp`, or the point of its vertex `id`. Invalid if there is no
// such one.
GA_Offset anchorPointOf(const GU_Detail &gdp, int id, bool vertex) {
  if (id < 0)
    return GA_INVALID_OFFSET;
  if (vertex) {
    if (static_cast<GA_Size>(id) >= gdp.getNumVertices())
      return GA_INVALID_OFFSET;
    return gdp.vertexPoint(gdp.vertexOffset(static_cast<GA_Index>(id)));
  }
  if (static_cast<GA_Size>(id) >= gdp.getNumPoints())
    return GA_INVALID_OFFSET;
  return gdp.pointOffset(static_cast<GA_Index>(id));
}

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
  const GA_Offset pt = anchorPointOf(*gdp, id, vertex);
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

// Why an anchor_id found no point.
enum class AnchorLookup { Ok, NoPoint, NoGeometry };

// Point `id` (or vertex `id`'s point) among the points of a packed fragment's
// own primitives in the shared source geometry: its points in source order, its
// vertices in primitive order, as the fragment on its own would number them
// (Bullet: `bullet_anchor_id`). Invalid if there is no such one.
GA_Offset fragmentAnchorPoint(const GU_Detail &src, const GA_Range &prims,
                              int id, bool vertex) {
  if (id < 0)
    return GA_INVALID_OFFSET;
  std::vector<GA_Offset> points;
  GA_Size vertexNumber = 0;
  for (GA_Iterator it(prims); !it.atEnd(); ++it) {
    const GA_Primitive *prim = src.getPrimitive(*it);
    if (!prim)
      continue;
    for (GA_Size i = 0; i < prim->getVertexCount(); ++i) {
      const GA_Offset pt = prim->getPointOffset(i);
      if (vertex && vertexNumber++ == static_cast<GA_Size>(id))
        return pt;
      points.push_back(pt);
    }
  }
  if (vertex)
    return GA_INVALID_OFFSET;
  std::sort(points.begin(), points.end());
  points.erase(std::unique(points.begin(), points.end()), points.end());
  return static_cast<size_t>(id) < points.size()
             ? points[static_cast<size_t>(id)]
             : GA_INVALID_OFFSET;
}

// World position of point (or vertex) `id` of a packed piece's own geometry (a
// fragment's own primitives of the shared source), now: the packed point's
// position plus the geometry point from the pivot through the piece's matrix.
// `pointOff` is the piece's point in the object's geometry.
AnchorLookup packedAnchorWorld(const SIM_Object &obj, int64_t pointOff, int id,
                               bool vertex, double out[3]) {
  const SIM_Geometry *geo = obj.getGeometry();
  if (!geo || pointOff < 0)
    return AnchorLookup::NoGeometry;
  SIM_GeometryAutoReadLock lock(geo);
  const GU_Detail *gdp = lock.getGdp();
  if (!gdp)
    return AnchorLookup::NoGeometry;
  const GA_Offset ppt = static_cast<GA_Offset>(pointOff);
  const GA_Offset vtx = gdp->pointVertex(ppt);
  if (!GAisValid(vtx))
    return AnchorLookup::NoGeometry;
  const GA_Offset po = gdp->vertexPrimitive(vtx);
  const GA_Primitive *prim = GAisValid(po) ? gdp->getPrimitive(po) : nullptr;
  if (!prim || !GU_PrimPacked::isPackedPrimitive(*prim))
    return AnchorLookup::NoGeometry;
  const GU_PrimPacked &packed = static_cast<const GU_PrimPacked &>(*prim);

  const GU_PackedFragment *frag =
      dynamic_cast<const GU_PackedFragment *>(packed.sharedImplementation());
  GU_DetailHandleAutoReadLock inner(frag ? frag->detailPtr()
                                         : packed.getPackedDetail());
  const GU_Detail *igdp = inner.getGdp();
  if (!igdp)
    return AnchorLookup::NoGeometry;
  const GA_Offset ipt =
      frag ? fragmentAnchorPoint(*igdp, frag->getPrimitiveRange(), id, vertex)
           : anchorPointOf(*igdp, id, vertex);
  if (!GAisValid(ipt))
    return AnchorLookup::NoPoint;

  UT_Matrix4D xf;
  packed.getFullTransform4(xf);
  UT_Vector3 pv(0, 0, 0);
  packed.getPivot(pv);
  const UT_Vector3 p = igdp->getPos3(ipt);
  const UT_Vector3 at = gdp->getPos3(ppt);
  // (p - pivot) * M3: scale and turn about the pivot; the packed point is
  // where the pivot is.
  const UT_Vector3D rel(static_cast<double>(p.x()) - pv.x(),
                        static_cast<double>(p.y()) - pv.y(),
                        static_cast<double>(p.z()) - pv.z());
  for (unsigned k = 0; k < 3; ++k) {
    out[k] = static_cast<double>(at[k]) + rel.x() * xf(0, k) +
             rel.y() * xf(1, k) + rel.z() * xf(2, k);
    if (!std::isfinite(out[k]))
      return AnchorLookup::NoGeometry;
  }
  return AnchorLookup::Ok;
}

// Anchor names -> bodies, over the objects of this solve (in their order) and
// the affectors. The name tables are built on the first resolve(): a step whose
// networks are all unchanged never resolves a name, so it pays nothing for
// them. After that a name is a hash lookup, not a scan of every object per
// anchor.
class Anchors {
public:
  Anchors(const Box3DSharedState &st, SIM_ObjectArray &objects) : myState(st) {
    for (int i = 0; i < objects.entries(); ++i) {
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
    if (myPieces) {
      const auto piece = myPieceTable.find(name);
      if (piece != myPieceTable.end()) {
        out = piece->second;
        return true;
      }
    } else {
      for (const ObjectRef &o : myObjects)
        if (findIn(o, name, out))
          return true;
    }
    const auto single = mySingleBody.find(name);
    if (single != mySingleBody.end()) {
      out = single->second;
      out.byObjectName = true;
      return true;
    }
    const auto affector = myAffector.find(name);
    if (affector != myAffector.end()) {
      out = affector->second;
      return true;
    }
    return false;
  }

private:
  using HandleTable = std::unordered_map<std::string, AnchorBody>;

  // `object`: the DOP object the body is in (an anchor_id names a point of its
  // geometry).
  static AnchorBody bodyOf(const Box3DBodyMirror *e, const SIM_Object *object) {
    AnchorBody b;
    b.handle = e->handle;
    b.entry = e;
    b.object = object;
    return b;
  }

  struct ObjectRef {
    std::string name;
    const Box3DObjectSync *sync;
    const SIM_Object *object;
  };

  static bool findIn(const ObjectRef &o, const std::string &piece,
                     AnchorBody &out) {
    const auto it = o.sync->byName.find(piece);
    if (it == o.sync->byName.end() || !it->second->handle)
      return false;
    out = bodyOf(it->second, o.object);
    return true;
  }

  // Each table keeps the first match in object order (what a scan in that order
  // would find).
  void buildTables() const {
    if (myBuilt)
      return;
    myBuilt = true;
    for (const auto &o : myObjects) {
      myObjectByName.try_emplace(o.name, &o);
      if (o.sync->bodies.size() == 1 && o.sync->bodies[0]->handle)
        mySingleBody.try_emplace(o.name, bodyOf(o.sync->bodies[0], o.object));
    }
    for (const auto &kv : myState.affectors)
      if (kv.second.handle) {
        AnchorBody b;
        b.handle = kv.second.handle;
        myAffector.try_emplace(kv.second.name, b);
      }
    // With many objects a scan per anchor costs more than one table of every
    // piece (the first object in order wins, as in the scan). The keys point
    // into the objects' own name maps, which do not change during a sync.
    if (myObjects.size() > kPieceTableObjects) {
      myPieces = true;
      for (const ObjectRef &o : myObjects)
        for (const auto &kv : o.sync->byName)
          if (kv.second->handle)
            myPieceTable.try_emplace(std::string_view(kv.first),
                                     bodyOf(kv.second, o.object));
    }
  }

  static constexpr size_t kPieceTableObjects = 16;

  const Box3DSharedState &myState;
  std::vector<ObjectRef> myObjects;

  mutable bool myBuilt = false;
  mutable std::unordered_map<std::string, const ObjectRef *> myObjectByName;
  mutable HandleTable mySingleBody;
  mutable HandleTable myAffector;
  mutable bool myPieces = false; // myPieceTable is in use
  mutable std::unordered_map<std::string_view, AnchorBody> myPieceTable;
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
    case SideFXParam::Float: {
      // "Not set" (Box3D Constraint Properties writes it on lines it does not
      // edit, under names both tables share): leave the option alone.
      const float value = r.f.get(prim);
      if (!b3dop::isUnsetFloat(value))
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

} // namespace

b3dop::JointFrame anchorFrameOf(const b3dop::JointFrame &joint,
                                const b3dop::JointFrame &adj) {
  return b3dop::isIdentityOffset(adj) ? joint
                                      : b3dop::removeFrameOffset(joint, adj);
}

namespace {

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
      for (const double p : f->pos)
        h = b3dop::mixDouble(h, p);
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

} // namespace

// A frame as the geometry stores it (float P), so it compares equal when read
// back.
b3dop::JointFrame asStored(const b3dop::JointFrame &f) {
  b3dop::JointFrame out = f;
  for (int k = 0; k < 3; ++k)
    out.pos[k] = static_cast<double>(static_cast<float>(f.pos[k]));
  return out;
}

namespace {

// ASCII case-insensitive equality, with no allocation (kind() runs for every
// primitive of a network that is read again).
bool equalIgnoreCase(const char *a, const char *b) {
  for (; *a && *b; ++a, ++b)
    if (std::tolower(static_cast<unsigned char>(*a)) !=
        std::tolower(static_cast<unsigned char>(*b)))
      return false;
  return *a == *b;
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
  const GA_ROHandleF impact;      // Glue's accumulated impact (feedback)
  const GA_ROHandleF bondImpulse; // a Bond's accumulated impulse (feedback)
  // The network's global Glue propagation iterations (detail attribute)
  const GA_ROHandleI globalIterations;
  const GA_ROHandleF globalIterationsF;

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
        anchorTypeS(g, GA_ATTRIB_POINT, kAnchorType),
        impact(g, GA_ATTRIB_PRIMITIVE, kImpact),
        bondImpulse(g, GA_ATTRIB_PRIMITIVE, kBondImpulse),
        globalIterations(g, GA_ATTRIB_GLOBAL, kPropagateIteration),
        globalIterationsF(g, GA_ATTRIB_GLOBAL, kPropagateIteration) {}

  // Glue rounds of propagation: the bond's own count, or for a negative one
  // the detail attribute's (one round when there is none).
  int glueIterations(int bondCount) const {
    if (bondCount >= 0)
      return std::clamp(bondCount, 1, b3dop::kMaxGlueIterations);
    int n = 1;
    if (globalIterations.isValid())
      n = globalIterations.get(GA_Offset(0));
    else if (globalIterationsF.isValid()) {
      const float f = globalIterationsF.get(GA_Offset(0));
      if (std::isfinite(f))
        n = static_cast<int>(
            std::clamp(f, 1.0f, static_cast<float>(b3dop::kMaxGlueIterations)));
    }
    return std::clamp(n, 1, b3dop::kMaxGlueIterations);
  }

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
    return readFrame(currentP, currentQ, pt, out);
  }

  // The anchor's frame now: live if written, else as authored.
  b3dop::JointFrame frame(GA_Offset pt) const {
    b3dop::JointFrame f;
    return live(pt, f) ? f : authored(pt);
  }

  // The primitive's constraint_type (SideFX data only; missing: position).
  b3dop::ConstraintMode mode(GA_Offset prim) const {
    const char *t = conType.isValid()
                        ? static_cast<const char *>(conType.get(prim))
                        : nullptr;
    return b3dop::constraintModeFromString(t);
  }

  // The primitive says constraint_type position or rotation.
  bool narrowsType(GA_Offset prim) const {
    const char *t = conType.isValid()
                        ? static_cast<const char *>(conType.get(prim))
                        : nullptr;
    return t &&
           (std::strcmp(t, "position") == 0 || std::strcmp(t, "rotation") == 0);
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
    // No exact match: ignore case, so "weld" finds a node named "Weld".
    for (const auto &kv : kinds)
      if (equalIgnoreCase(kv.first.c_str(), n))
        return &kv.second;
    return nullptr;
  }

  // The anchor's saved local frame (local_P / local_orient), if it has one (a
  // zero quaternion: none).
  bool savedLocal(GA_Offset pt, b3dop::JointFrame &out) const {
    return readFrame(localP, localQ, pt, out);
  }

private:
  // A frame stored as a position and a quaternion point attribute. False when
  // either is missing, the quaternion is all zero (not written) or the position
  // is not finite.
  static bool readFrame(const GA_ROHandleV3 &posH, const GA_ROHandleQ &quatH,
                        GA_Offset pt, b3dop::JointFrame &out) {
    if (!posH.isValid() || !quatH.isValid())
      return false;
    const UT_Vector3 p = posH.get(pt);
    const UT_QuaternionF q = quatH.get(pt);
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

// The inverse of restToNow for a position: where the world position `p` of a
// point on the piece now was at rest. Unchanged where restToNow leaves a frame
// alone.
void nowToRest(const b3dop::Box3DWorld &world, const AnchorBody &body,
               double p[3]) {
  const Box3DBodyMirror *e = body.entry;
  double pos[3];
  float rot[4];
  if (!body.handle || !e || !e->packed || !e->hasRest ||
      !world.getTransformByHandle(body.handle, pos, rot))
    return;
  float since[4];
  box3dAbsoluteToRest(e->restRot, rot, since);
  UT_DMatrix3 d;
  UT_QuaternionD(since[0], since[1], since[2], since[3]).getRotationMatrix(d);
  d.transpose(); // a rotation: its transpose is its inverse
  const UT_Vector3D moved(p[0] - pos[0], p[1] - pos[1], p[2] - pos[2]);
  const UT_Vector3D offset = moved * d;
  for (int k = 0; k < 3; ++k)
    p[k] = offset[k] + e->restPos[k];
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
  bool glue = false; // SideFX Glue: breaks on impact impulse (core/Box3DGlue.h)
  b3dop::GlueParams glueParams;
  bool massDependent = false;     // SideFX Spring: Hz from the bodies' masses
  b3dop::SideFXBody bodyA, bodyB; // the masses it was converted with
};

// One network's primitives, read from its geometry, into joints: make, update,
// drop.
class NetworkPass {
public:
  NetworkPass(Box3DSharedState &st, const NetworkRef &ref,
              Box3DJointNetwork &net, const Anchors &anchors,
              const GU_Detail *gdp, Box3DJointCounts &counts)
      : mySt(st), myWorld(st.world), myRef(ref), myNet(net), myAnchors(anchors),
        myGdp(gdp), myCounts(counts), myRd(gdp), myReaders(bindSettings(gdp)),
        mySfReaders(bindSideFX(gdp)) {}

  void run() {
    myNet.needsBodies = false;
    // A network made all at once (a first frame, a rebuilt world) would
    // otherwise rehash the entry map every time it doubles.
    mySt.joints.reserve(mySt.joints.size() +
                        static_cast<size_t>(myGdp->getNumPrimitives()));
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
      ctx.a = sideFXBody(myWorld, a);
      ctx.b = sideFXBody(myWorld, b);
      const b3dop::SideFXJoint m =
          b3dop::mapSideFX(kind.sideFXType, sf, myRd.mode(off), ctx);
      j.type = m.type;
      j.s = m.settings;
      j.frameSig = sideFXFrameSignature(m);
      b3dop::sideFXFrameOffsets(m, fa, fb, j.adjA, j.adjB);
      if (kind.sideFXType == b3dop::SideFXType::Spring) {
        j.massDependent = true;
        j.bodyA = ctx.a;
        j.bodyB = ctx.b;
      }
      if (kind.sideFXType == b3dop::SideFXType::Glue) {
        j.glue = true;
        j.glueParams.strength = sf.strength;
        j.glueParams.halfLife = sf.impulseHalfLife;
        j.glueParams.rate = sf.propagateRate;
        j.glueParams.iterations = myRd.glueIterations(sf.propagationIterations);
      }
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
    if (e.type != j.type)
      mySt.networkEntriesDirty = true; // the Bond list follows the types
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

  // The anchor as authored (P / orient). With an anchor_id on a whole object or
  // a packed piece it sits at that point (or vertex) of the object's geometry
  // (a piece's own geometry), as it was the first time this joint saw it: kept
  // in the entry, so the body moving is no edit. `bad` is set when the id names
  // no point.
  b3dop::JointFrame authoredAnchor(Box3DJointEntry &e, bool second,
                                   GA_Offset pt, const AnchorBody &body,
                                   AnchorLookup &bad) const {
    b3dop::JointFrame f = myRd.authored(pt);
    const int id = myRd.anchorId(pt);
    // A whole object's geometry points, or a packed piece's own geometry's.
    const bool whole = body.entry && body.entry->wholeObject;
    const bool piece = body.entry && body.entry->packed && !whole;
    if (id < 0 || !body.handle || !body.object || !(whole || piece)) {
      // No point to follow (the id is ignored on the world and affectors):
      // forget a pin from an earlier id.
      if (e.extra)
        (second ? e.extra->pinIdB : e.extra->pinIdA) = -1;
      return f;
    }
    Box3DJointExtra &x = e.ensureExtra();
    b3dop::JointFrame &pin = second ? x.pinB : x.pinA;
    int &pinId = second ? x.pinIdB : x.pinIdA;
    bool &pinVertex = second ? x.pinVertexB : x.pinVertexA;
    const bool vertex = myRd.anchorKind(pt) == AnchorKind::Vertex;
    if (pinId != id || pinVertex != vertex) {
      double w[3];
      AnchorLookup found = AnchorLookup::Ok;
      if (whole)
        found = pointAnchorWorld(*body.object, id, vertex, w)
                    ? AnchorLookup::Ok
                    : AnchorLookup::NoPoint;
      else
        found = packedAnchorWorld(*body.object, body.entry->pointOff, id,
                                  vertex, w);
      if (found != AnchorLookup::Ok) {
        bad = found;
        return f;
      }
      // Authored anchors are at rest: a piece that has moved gives the point
      // where it was.
      if (piece)
        nowToRest(myWorld, body, w);
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
    if (!kind->sideFX && myRd.narrowsType(off))
      ++myCounts.typeIgnored;

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
      myNet.needsBodies = true;
      dropJoint(e);
      return; // not stamped: removed; retried when bodies change
    }
    const b3dop::BodyHandle a = ab.handle, b = bb.handle;
    if (ab.byObjectName || bb.byObjectName)
      myNet.needsBodies = true;
    // On a written network current_P / current_orient say where the anchors
    // are now, so a world rebuilt mid-simulation makes its joints from them
    // exactly as frame 1 does from the authored P / orient.
    AnchorLookup badAnchor = AnchorLookup::Ok;
    const b3dop::JointFrame authA = asStored(
                                authoredAnchor(e, false, p0, ab, badAnchor)),
                            authB = asStored(
                                authoredAnchor(e, true, p1, bb, badAnchor));
    if (badAnchor != AnchorLookup::Ok) {
      myNet.needsBodies = true;
      ++(badAnchor == AnchorLookup::NoGeometry ? myCounts.anchorNoGeometry
                                               : myCounts.badAnchorId);
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
    e.glue = j.glue;
    e.massDependent = j.massDependent;
    if (j.type == b3dop::JointType::Bond) {
      // In the glue graph, but never broken by the Glue's impulse.
      b3dop::GlueParams never;
      never.strength = -1.0f;
      Box3DJointExtra &bx = e.ensureExtra();
      bx.glueParams = never;
      if (!e.made && myRd.bondImpulse.isValid()) {
        // A world rebuilt mid-simulation: carry on from what the geometry
        // holds.
        const float kept = myRd.bondImpulse.get(off);
        bx.bondImpulse = std::isfinite(kept) && kept > 0.0f ? kept : 0.0f;
      }
    }
    if (j.glue || j.massDependent) {
      Box3DJointExtra &x = e.ensureExtra();
      if (j.glue)
        x.glueParams = j.glueParams;
      if (j.massDependent) {
        x.massA = j.bodyA;
        x.massB = j.bodyB;
      }
      if (j.glue && !e.made && myRd.impact.isValid()) {
        // A world rebuilt mid-simulation: carry on from what the geometry
        // holds.
        const float kept = myRd.impact.get(off);
        x.glueImpact = std::isfinite(kept) && kept > 0.0f ? kept : 0.0f;
      }
    }

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
  Box3DJointCounts &myCounts;
  const NetworkReader myRd;
  const std::vector<SettingReader> myReaders;
  const std::vector<SideFXReader> mySfReaders;
};

// True if the network's joints were read again (false: left as they are).
bool processNetwork(Box3DSharedState &st, const NetworkRef &ref,
                    const Anchors &anchors, bool massChanged, bool bodyRemoved,
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
    return false;
  SIM_GeometryAutoReadLock lock(geo);
  const GU_Detail *gdp = lock.getGdp();
  if (!gdp)
    return false;

  // Unchanged geometry, constraint data and bodies it depends on: the joints
  // stay as they are (they keep liveStamp). Bodies coming or going matter only
  // to a network with anchors that did not resolve, one with a joint on a
  // removed body, and when pieces were renamed or an affector came or went.
  const uint64_t fp = networkFingerprint(gdp);
  const bool bodiesMatter =
      st.allNetworksDirty || bodyRemoved ||
      (net.needsBodies && (st.bodiesAdded || !st.removedBodies.empty())) ||
      net.nameVersion != st.nameVersion;
  if (fp == net.fingerprint && !bodiesMatter && !kindsChanged && !massChanged)
    return false;
  net.fingerprint = fp;
  net.nameVersion = st.nameVersion;
  net.liveStamp = st.syncStamp;

  NetworkPass(st, ref, net, anchors, gdp, counts).run();
  return true;
}

} // namespace

namespace {

// Takes the bonds that broke out of the glue lists: swapped out of the bond
// list, out of their nodes' partners and bonds, and out of the signature. The
// nodes they joined are kept for regrouping, and a node left without bonds
// stays (its hits are never used) until too many have, then the lists are
// rebuilt.
void removeGlueBonds(Box3DSharedState &st) {
  for (Box3DJointEntry *e : st.glueRemoved) {
    const int i = e->glueIndex;
    if (i < 0 || static_cast<size_t>(i) >= st.glueBonds.size() ||
        st.glueBonds[static_cast<size_t>(i)] != e)
      continue;
    const std::pair<int, int> ab = st.glueBondNodes[static_cast<size_t>(i)];
    Box3DGlueNode *na = &st.glueNodes[static_cast<size_t>(ab.first)];
    Box3DGlueNode *nb = &st.glueNodes[static_cast<size_t>(ab.second)];
    st.glueSignature -= kGlueCountSalt + glueBondHash(na->body, nb->body);
    auto dropFrom = [](Box3DGlueNode &n, b3dop::BodyHandle other,
                       Box3DJointEntry *bond) {
      const auto p = std::find(n.partners.begin(), n.partners.end(), other);
      if (p != n.partners.end())
        n.partners.erase(p);
      const auto b = std::find(n.bonds.begin(), n.bonds.end(), bond);
      if (b != n.bonds.end())
        n.bonds.erase(b);
      return n.bonds.empty();
    };
    if (dropFrom(*na, nb->body, e))
      ++st.glueOrphans;
    if (dropFrom(*nb, na->body, e))
      ++st.glueOrphans;
    st.glueTouched.push_back(ab.first);
    st.glueTouched.push_back(ab.second);
    const size_t last = st.glueBonds.size() - 1;
    if (static_cast<size_t>(i) != last) {
      st.glueBonds[static_cast<size_t>(i)] = st.glueBonds[last];
      st.glueBondNodes[static_cast<size_t>(i)] = st.glueBondNodes[last];
      st.glueBonds[static_cast<size_t>(i)]->glueIndex = i;
    }
    st.glueBonds.pop_back();
    st.glueBondNodes.pop_back();
    e->glueIndex = -1;
  }
  st.glueRemoved.clear();
  if (st.glueOrphans * 4 > st.glueNodes.size())
    st.glueFull = true;
}

} // namespace

bool anyJointNetwork(SIM_ObjectArray &objects) {
  for (int i = 0; i < objects.entries(); ++i) {
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

void syncJoints(Box3DSharedState &st, SIM_ObjectArray &objects,
                Box3DJointCounts &counts) {
  // ---- gather: each Constraint Network relationship once (it is on every
  // object it constrains), and each object's internal ConstraintGeometry ----
  std::vector<NetworkRef> refs;
  std::unordered_set<std::string> seenRels;
  for (int i = 0; i < objects.entries(); ++i) {
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

  st.glueBroken.clear();

  // Whether any joint may have come or gone since the last sync: a network read
  // again, or one that went away. Without that (and without a joint that broke
  // last step: glueDirty) the removal loop and the Glue graph are left as they
  // are.
  bool changed = false;
  if (!refs.empty()) {
    // A Spring whose bodies' masses changed since it was made is reprocessed
    // (its Hz and damping ratio follow the masses): the networks to read again.
    // Only while there are Springs (counted by the removal loop last step).
    std::unordered_set<uint64_t> massChanged;
    if (st.springEntries > 0)
      for (const auto &kv : st.joints) {
        const Box3DJointEntry &e = kv.second;
        if (e.massDependent && e.extra && !e.broken &&
            st.world.isJointValid(e.handle) &&
            (!sameSideFXBody(e.extra->massA, sideFXBody(st.world, e.bodyA)) ||
             !sameSideFXBody(e.extra->massB, sideFXBody(st.world, e.bodyB))))
          massChanged.insert(e.network);
      }
    // The networks with a joint on a body removed this sync.
    std::unordered_set<uint64_t> removedOn;
    if (!st.removedBodies.empty()) {
      const std::unordered_set<b3dop::BodyHandle> gone(st.removedBodies.begin(),
                                                       st.removedBodies.end());
      for (const auto &kv : st.joints)
        if (gone.count(kv.second.bodyA) || gone.count(kv.second.bodyB))
          removedOn.insert(kv.second.network);
    }
    const Anchors anchors(st, objects);
    for (const NetworkRef &ref : refs)
      changed |=
          processNetwork(st, ref, anchors, massChanged.count(ref.key) != 0,
                         removedOn.count(ref.key) != 0, counts);
  }
  for (const auto &kv : st.jointNetworks)
    changed = changed || kv.second.visited != st.syncStamp;

  // ---- remove: joints whose primitive or network went away; list the Glue
  // bonds and count the Springs of the rest ----
  // Bonds that only broke are taken out of the lists in place.
  if (!changed && !st.glueFull && st.glueDirty && !st.glueRemoved.empty()) {
    st.glueDirty = false;
    st.networkEntriesDirty = true;
    removeGlueBonds(st);
  } else if (changed || st.glueDirty || st.glueFull) {
    st.glueDirty = false;
    st.glueFull = false;
    st.glueRemoved.clear();
    st.glueTouched.clear();
    st.glueOrphans = 0;
    st.networkEntriesDirty = true; // joints may have come or gone
    st.groupsRecheck = true;       // a piece may have gained or lost a joint
    st.bondsInGraph = false;
    st.glueNodes.clear();
    st.glueNodeOf.clear();
    st.glueBonds.clear();
    st.glueBondNodes.clear();
    st.glueAnchored.clear();
    auto nodeOf = [&st](b3dop::BodyHandle h) {
      const auto ins =
          st.glueNodeOf.try_emplace(h, static_cast<int>(st.glueNodes.size()));
      if (ins.second) {
        st.glueNodes.emplace_back();
        st.glueNodes.back().body = h;
      }
      return ins.first->second;
    };
    size_t springs = 0;
    for (auto it = st.joints.begin(); it != st.joints.end();) {
      Box3DJointEntry &e = it->second;
      const auto net = st.jointNetworks.find(e.network);
      const bool live = net != st.jointNetworks.end() &&
                        net->second.visited == st.syncStamp &&
                        e.stamp == net->second.liveStamp;
      e.glueIndex = -1;
      if (live) {
        if (e.massDependent && !e.broken)
          ++springs;
        if (e.type == b3dop::JointType::Bond && e.extra && !e.broken &&
            (!e.bodyA != !e.bodyB) && st.world.isJointValid(e.handle))
          st.glueAnchored.insert(e.bodyA ? e.bodyA : e.bodyB);
        // Glue to the world is not a bond (Bullet does not support it either).
        if (inGlueGraph(e) && e.extra && !e.broken && e.bodyA && e.bodyB &&
            st.world.isJointValid(e.handle)) {
          if (e.type == b3dop::JointType::Bond)
            st.bondsInGraph = true;
          const int a = nodeOf(e.bodyA), b = nodeOf(e.bodyB);
          st.glueNodes[static_cast<size_t>(a)].partners.push_back(e.bodyB);
          st.glueNodes[static_cast<size_t>(b)].partners.push_back(e.bodyA);
          e.glueIndex = static_cast<int>(st.glueBonds.size());
          st.glueNodes[static_cast<size_t>(a)].bonds.push_back(&e);
          st.glueNodes[static_cast<size_t>(b)].bonds.push_back(&e);
          st.glueBonds.push_back(&e);
          st.glueBondNodes.emplace_back(a, b);
        }
        ++it;
        continue;
      }
      if (st.world.isJointValid(e.handle)) {
        st.world.destroyJoint(e.handle);
        ++counts.removed;
      }
      it = st.joints.erase(it);
    }
    st.springEntries = springs;
    st.glueSignature = st.computeGlueSignature();
  }
  for (auto it = st.jointNetworks.begin(); it != st.jointNetworks.end();)
    it = (it->second.visited == st.syncStamp) ? std::next(it)
                                              : st.jointNetworks.erase(it);
}
