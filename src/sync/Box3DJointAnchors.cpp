// Box3DJointAnchors.cpp
//
// Where a joint's anchors are: point and vertex anchors on a whole object or
// packed piece, fragments, and the rest-frame conversions.
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

namespace {

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

} // namespace

namespace jointsync {

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

} // namespace jointsync

namespace {

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

} // namespace

namespace jointsync {

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

} // namespace jointsync
