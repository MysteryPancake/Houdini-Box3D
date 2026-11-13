// Box3DAffectors.cpp
#include "Box3DAffectors.h"
#include "Box3DHoudiniUtil.h"
#include "core/Box3DAffectorMath.h"
#include "core/Box3DDebug.h"
#include "dop/SIM_Box3DData.h"
#include "dop/SIM_Box3DWorld.h"

#include <GA/GA_Iterator.h>
#include <GA/GA_Primitive.h>
#include <GA/GA_PrimitiveTypes.h>
#include <GU/GU_Detail.h>
#include <SIM/SIM_Geometry.h>
#include <SIM/SIM_Names.h>
#include <SIM/SIM_Object.h>
#include <SIM/SIM_ObjectArray.h>
#include <SIM/SIM_Utils.h>
#include <UT/UT_Matrix4.h>
#include <UT/UT_String.h>
#include <UT/UT_Vector3.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <unordered_set>
#include <vector>

namespace {

// The relationship type name SIM_Object::getConstAffectors() matches (it is the
// data type name: SideFX's own hair solver passes "SIM_RelationshipSource" the
// same way).
const char *const kCollideRelationship = "SIM_RelationshipCollide";

// Triangles from the object's polygons (fan-triangulated; open curves, packed
// primitives and the like are ignored). Vertices are the object's points, in
// point order of first use.
struct TriMesh {
  std::vector<float> xyz; // object space
  std::vector<int32_t> tris;
};

void buildMesh(const GU_Detail *gdp, bool houdiniWinding, TriMesh &out) {
  out.xyz.clear();
  out.tris.clear();
  std::vector<int32_t> remap(static_cast<size_t>(gdp->getNumPointOffsets()),
                             -1); // point offset -> vertex

  auto vertexOf = [&](GA_Offset pt) -> int32_t {
    int32_t &idx = remap[static_cast<size_t>(pt)];
    if (idx < 0) {
      const UT_Vector3 p = gdp->getPos3(pt);
      idx = static_cast<int32_t>(out.xyz.size() / 3);
      out.xyz.push_back(p.x());
      out.xyz.push_back(p.y());
      out.xyz.push_back(p.z());
    }
    return idx;
  };

  for (GA_Iterator it(gdp->getPrimitiveRange()); !it.atEnd(); ++it) {
    const GA_Primitive *prim = gdp->getPrimitive(*it);
    if (!prim || prim->getTypeId() != GA_PRIMPOLY)
      continue;
    const GA_Size nv = prim->getVertexCount();
    if (nv < 3)
      continue;
    const int32_t first = vertexOf(prim->getPointOffset(0));
    for (GA_Size i = 1; i + 1 < nv; ++i) {
      const int32_t b = vertexOf(prim->getPointOffset(i));
      const int32_t c = vertexOf(prim->getPointOffset(i + 1));
      // Houdini polygons wind clockwise seen from their front (Flip Static Mesh
      // Winding on the Box3D Data node reverses this); Box3D wants
      // counter-clockwise front faces, and its meshes are one-sided.
      out.tris.push_back(first);
      out.tris.push_back(houdiniWinding ? c : b);
      out.tris.push_back(houdiniWinding ? b : c);
    }
  }
}

void destroyAffector(Box3DSharedState &st, Box3DAffectorBody &ab) {
  if (ab.handle)
    st.world.destroyBody(ab.handle);
  ab.handle = 0;
}

// ---- gather: every affector of a solved object, once, that we do not solve
// ourselves ----
std::vector<const SIM_Object *> gatherAffectors(SIM_ObjectArray &objects) {
  std::unordered_set<int> solved;
  for (exint i = 0; i < objects.entries(); ++i)
    solved.insert(objects(i)->getObjectId());

  std::vector<const SIM_Object *> affectors;
  std::unordered_set<int> seen;
  for (exint i = 0; i < objects.entries(); ++i) {
    SIM_ConstObjectArray aff;
    objects(i)->getConstAffectors(aff, kCollideRelationship);
    for (exint j = 0; j < aff.entries(); ++j) {
      const SIM_Object *a = aff(j);
      if (!a)
        continue;
      const int id = a->getObjectId();
      if (solved.count(id) || !seen.insert(id).second)
        continue;
      affectors.push_back(a);
    }
  }
  return affectors;
}

// An affector as one sync reads it: its transform (split into pose and scale),
// its material, and the signature of what its mesh shape depends on.
struct AffectorInput {
  double m[16] = {}; // world transform, row-vector layout
  double pos[3] = {};
  float rot[4] = {0, 0, 0, 1};
  float scale[3] = {1, 1, 1};
  bool rigid = false; // pose + per-axis scale; else the matrix is baked
  b3dop::BodySettings body;
  uint64_t sig = 0;
};

// The object's world transform (position + any scale). False if it is not
// finite.
bool readAffectorTransform(const SIM_Object &obj, AffectorInput &in) {
  UT_DMatrix4 xf;
  SIMgetGeometryTransform(xf, obj);
  box3dToArray(xf, in.m);
  for (double d : in.m)
    if (!std::isfinite(d))
      return false;

  // Rotation + per-axis scale + translation: the body takes the pose and the
  // mesh shape the scale, so an animated transform never rebuilds the mesh.
  // Shear or mirroring: baked.
  in.rigid = b3dop::decomposeScaledRigid(in.m, in.pos, in.rot, in.scale);
  return true;
}

// Material: a Box3D Data node on the affector sets friction and bounce (bodies
// are always static).
b3dop::BodySettings affectorBodySettings(const SIM_Object &obj) {
  b3dop::BodySettings body;
  if (const SIM_Box3DData *node =
          SIM_DATA_GETCONST(obj, kBox3DDataName, SIM_Box3DData))
    body = node->toBodySettings();
  body.type = b3dop::BodySettings::Type::Static;
  return body;
}

// What the shape depends on: the points and polygons, the material, and (when
// it cannot be split into pose and scale) the whole matrix, which is baked into
// the vertices.
uint64_t affectorSignature(const GU_Detail *gdp, const AffectorInput &in) {
  uint64_t sig = geometryIdentity(gdp, /*withUniqueId=*/false);
  const float mat[3] = {in.body.props.friction, in.body.props.restitution,
                        in.body.flipMeshWinding ? 1.0f : 0.0f};
  sig = b3dop::mixHash(sig, b3dop::hashFloats(mat, 3));
  sig = b3dop::mixHash(sig, in.rigid ? 1 : 2);
  if (!in.rigid)
    for (double d : in.m)
      sig = b3dop::mixDouble(sig, d);
  return sig;
}

// ---- update: the same shape as last time
// ------------------------------------------------ A moved (animated) object
// only needs a new pose and / or mesh scale; a failed build (handle 0) stays
// failed until the signature changes.
void updateAffector(Box3DSharedState &st, Box3DAffectorBody &ab,
                    const AffectorInput &in, Box3DAffectorCounts &counts) {
  if (!ab.handle) {
    ++counts.failed;
    return;
  }

  // Rolling resistance and conveyor velocity are not in the signature: a change
  // (an animated belt speed) updates the shapes instead of rebuilding the mesh.
  const b3dop::BodyProps &props = in.body.props;
  if (ab.rolling != props.rollingResistance ||
      !std::equal(ab.tangent, ab.tangent + 3, props.tangentVelocity)) {
    st.world.setSurfaceByHandle(ab.handle, props.rollingResistance,
                                props.tangentVelocity);
    ab.rolling = props.rollingResistance;
    b3dop::copy3(ab.tangent, props.tangentVelocity);
  }
  if (!in.rigid)
    return;

  const double *pos = in.pos;
  const float *rot = in.rot;
  const bool samePose = std::fabs(ab.pos[0] - pos[0]) < 1e-9 &&
                        std::fabs(ab.pos[1] - pos[1]) < 1e-9 &&
                        std::fabs(ab.pos[2] - pos[2]) < 1e-9 &&
                        std::fabs(ab.rot[0] - rot[0]) < 1e-7f &&
                        std::fabs(ab.rot[1] - rot[1]) < 1e-7f &&
                        std::fabs(ab.rot[2] - rot[2]) < 1e-7f &&
                        std::fabs(ab.rot[3] - rot[3]) < 1e-7f;
  const bool sameScale = ab.scale[0] == in.scale[0] &&
                         ab.scale[1] == in.scale[1] &&
                         ab.scale[2] == in.scale[2];
  if (!samePose) {
    st.world.setTransformByHandle(ab.handle, pos, rot);
    b3dop::copy3(ab.pos, pos);
    b3dop::copy4(ab.rot, rot);
  }
  if (!sameScale) {
    st.world.setMeshScaleByHandle(ab.handle, in.scale);
    b3dop::copy3(ab.scale, in.scale);
  }
  if (!samePose || !sameScale)
    ++counts.moved;
}

// ---- build
// ----------------------------------------------------------------------------------
// Shear or mirroring cannot be a body transform plus mesh scale: bake the
// matrix into the vertices (`baked`) and keep the body at the origin.
// (Mirroring flips the winding, so flip the triangles back.)
void bakeTransform(const double m[16], TriMesh &mesh,
                   std::vector<float> &baked) {
  const size_t n = mesh.xyz.size() / 3;
  baked.resize(mesh.xyz.size());
  for (size_t i = 0; i < n; ++i) {
    const double p[3] = {mesh.xyz[3 * i], mesh.xyz[3 * i + 1],
                         mesh.xyz[3 * i + 2]};
    double q[3];
    b3dop::transformPoint(m, p, q);
    for (int j = 0; j < 3; ++j)
      baked[3 * i + static_cast<size_t>(j)] = static_cast<float>(q[j]);
  }
  const double det = m[0] * (m[5] * m[10] - m[6] * m[9]) -
                     m[1] * (m[4] * m[10] - m[6] * m[8]) +
                     m[2] * (m[4] * m[9] - m[5] * m[8]);
  if (det < 0.0)
    for (size_t t = 0; t + 2 < mesh.tris.size(); t += 3)
      std::swap(mesh.tris[t + 1], mesh.tris[t + 2]);
}

// (Re)build from scratch: one static body with one triangle mesh shape.
void buildAffector(Box3DSharedState &st, Box3DAffectorBody &ab, int objectId,
                   const GU_Detail *gdp, const AffectorInput &in, TriMesh &mesh,
                   std::vector<float> &baked, Box3DAffectorCounts &counts) {
  destroyAffector(st, ab);
  ab.signature = in.sig;
  ab.poseKnown = true;

  const b3dop::BodySettings &body = in.body;
  buildMesh(gdp, !body.flipMeshWinding, mesh);
  if (mesh.tris.empty()) {
    ++counts.failed;
    return;
  }
  const float *verts = mesh.xyz.data();
  double bodyPos[3] = {0, 0, 0};
  float bodyRot[4] = {0, 0, 0, 1};
  float bodyScale[3] = {1, 1, 1};
  if (in.rigid) {
    b3dop::copy3(bodyPos, in.pos);
    b3dop::copy4(bodyRot, in.rot);
    b3dop::copy3(bodyScale, in.scale);
  } else {
    bakeTransform(in.m, mesh, baked);
    verts = baked.data();
    ++counts.baked;
  }

  const b3dop::BodyHandle h =
      st.world.createBody(body, bodyPos, bodyRot, nullptr);
  if (!h) {
    ++counts.failed;
    return;
  }
  if (!st.world.createMeshShape(
          h, body, verts, static_cast<int>(mesh.xyz.size() / 3),
          mesh.tris.data(), static_cast<int>(mesh.tris.size() / 3), 0.0f,
          bodyScale)) {
    st.world.destroyBody(h);
    ++counts.failed;
    return;
  }
  ab.handle = h;
  ab.rolling = body.props.rollingResistance;
  b3dop::copy3(ab.tangent, body.props.tangentVelocity);
  b3dop::copy3(ab.pos, bodyPos);
  b3dop::copy4(ab.rot, bodyRot);
  b3dop::copy3(ab.scale, bodyScale);
  ++counts.added;
  B3D_LOG("  affector id=%d: static mesh built, %d triangle(s)%s", objectId,
          static_cast<int>(mesh.tris.size() / 3),
          in.rigid ? "" : " (transform baked)");
}

// ---- remove: objects that are no longer affectors
// ---------------------------------------
void removeStaleAffectors(Box3DSharedState &st, Box3DAffectorCounts &counts) {
  for (auto it = st.affectors.begin(); it != st.affectors.end();) {
    if (it->second.stamp != st.syncStamp) {
      if (it->second.handle)
        ++counts.removed;
      destroyAffector(st, it->second);
      it = st.affectors.erase(it);
    } else
      ++it;
  }
}

} // namespace

void syncAffectorBodies(Box3DSharedState &st, SIM_ObjectArray &objects,
                        Box3DAffectorCounts &counts) {
  const std::vector<const SIM_Object *> affectors = gatherAffectors(objects);
  counts.total = static_cast<int>(affectors.size());

  // Build or update each one.
  TriMesh mesh;
  std::vector<float> baked;
  for (const SIM_Object *obj : affectors) {
    const SIM_Geometry *geo = obj->getGeometry();
    if (!geo) {
      ++counts.noGeo;
      continue;
    }
    SIM_GeometryAutoReadLock lock(geo);
    const GU_Detail *gdp = lock.getGdp();
    if (!gdp) {
      ++counts.noGeo;
      continue;
    }

    Box3DAffectorBody &ab = st.affectors[obj->getObjectId()];
    ab.stamp = st.syncStamp;
    {
      const UT_String &nm = obj->getName();
      ab.name = nm.isstring() ? std::string(nm.buffer()) : std::string();
    }

    AffectorInput in;
    if (!readAffectorTransform(*obj, in)) {
      ++counts.failed;
      continue;
    }
    in.body = affectorBodySettings(*obj);
    in.sig = affectorSignature(gdp, in);

    if (ab.poseKnown && ab.signature == in.sig)
      updateAffector(st, ab, in, counts);
    else
      buildAffector(st, ab, obj->getObjectId(), gdp, in, mesh, baked, counts);
  }

  removeStaleAffectors(st, counts);
}
