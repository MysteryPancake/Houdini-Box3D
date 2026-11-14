// Box3DAffectors.cpp
#include "Box3DAffectors.h"
#include "Box3DHoudiniUtil.h"
#include "core/Box3DAffectorMath.h"
#include "core/Box3DDebug.h"
#include "dop/SIM_Box3DData.h"
#include "dop/SIM_Box3DWorld.h"

#include <GA/GA_Handle.h>
#include <GA/GA_Iterator.h>
#include <GA/GA_Primitive.h>
#include <GA/GA_PrimitiveTypes.h>
#include <GEO/GEO_PrimVolume.h>
#include <GU/GU_Detail.h>
#include <SIM/SIM_Geometry.h>
#include <SIM/SIM_Names.h>
#include <SIM/SIM_Object.h>
#include <SIM/SIM_ObjectArray.h>
#include <SIM/SIM_Utils.h>
#include <UT/UT_Matrix3.h>
#include <UT/UT_Matrix4.h>
#include <UT/UT_String.h>
#include <UT/UT_Vector3.h>
#include <UT/UT_VoxelArray.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <initializer_list>
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

// ---- height fields
// --------------------------------------------------------------------
// Houdini's height fields are 2D float volumes: one voxel thick along one index
// axis, each voxel the terrain height at its centre, in world units along that
// thin axis (the volume's local z for every HeightField SOP orientation). The
// default height layer is the volume named "height"; masks and other layers
// are ignored.

// The affector's height layer: the float volume named "height", or, when no
// primitive is named, the first flat float volume. Null if there is none.
const GEO_PrimVolume *findHeightVolume(const GU_Detail *gdp) {
  const GA_ROHandleS nameH(gdp, GA_ATTRIB_PRIMITIVE, "name");
  const GEO_PrimVolume *unnamed = nullptr;
  for (GA_Iterator it(gdp->getPrimitiveRange()); !it.atEnd(); ++it) {
    const GA_Primitive *prim = gdp->getPrimitive(*it);
    if (!prim || prim->getTypeId() != GA_PRIMVOLUME)
      continue;
    const auto *vol = static_cast<const GEO_PrimVolume *>(prim);
    if (vol->getStorageType() != GEO_PrimVolume::VOLUME_TYPE_FLOAT)
      continue;
    int rx = 0, ry = 0, rz = 0;
    vol->getRes(rx, ry, rz);
    if (rx != 1 && ry != 1 && rz != 1)
      continue; // a 3D volume (fog, SDF): not a height field
    if (nameH.isValid()) {
      const char *nm = static_cast<const char *>(nameH.get(*it));
      if (nm && std::strcmp(nm, "height") == 0)
        return vol;
    } else if (!unnamed)
      unnamed = vol;
  }
  return unnamed;
}

// Where the samples are, in the volume's (object) space: sample (i, j) sits
// at origin + i * e1 + j * e2, and its height h at that plus h * up.
struct HeightFrame {
  int n1 = 0, n2 = 0;           // samples along the two in-plane index axes
  int a1 = 0, a2 = 1, flat = 2; // which index axes those are, and the thin one
  UT_Vector3D origin, e1, e2;
  UT_Vector3D up; // unit
};

bool readHeightFrame(const GEO_PrimVolume *vol, HeightFrame &f) {
  int r[3] = {0, 0, 0};
  vol->getRes(r[0], r[1], r[2]);
  f.flat = -1;
  for (int k = 2; k >= 0; --k)
    if (r[k] == 1) {
      f.flat = k;
      break;
    }
  if (f.flat < 0)
    return false;
  f.a1 = f.flat == 0 ? 1 : 0;
  f.a2 = f.flat == 2 ? 1 : 2;
  f.n1 = r[f.a1];
  f.n2 = r[f.a2];
  if (f.n1 < 2 || f.n2 < 2)
    return false;

  exint i1[3] = {0, 0, 0}, i2[3] = {0, 0, 0};
  i1[f.a1] = 1;
  i2[f.a2] = 1;
  UT_Vector3D p1, p2;
  vol->indexToPos(exint(0), exint(0), exint(0), f.origin);
  vol->indexToPos(i1[0], i1[1], i1[2], p1);
  vol->indexToPos(i2[0], i2[1], i2[2], p2);
  f.e1 = p1 - f.origin;
  f.e2 = p2 - f.origin;

  // Heights run along the thin axis of the volume's transform.
  const UT_Matrix3 &x = vol->getTransform();
  f.up = UT_Vector3D(x(f.flat, 0), x(f.flat, 1), x(f.flat, 2));
  if (!(f.up.length() > 1e-12))
    f.up = cross(f.e1, f.e2);
  const double len = f.up.length();
  if (!(len > 1e-12) || !(f.e1.length() > 1e-12) || !(f.e2.length() > 1e-12))
    return false;
  f.up /= len;
  for (const UT_Vector3D *v : {&f.origin, &f.e1, &f.e2, &f.up})
    for (int k = 0; k < 3; ++k)
      if (!std::isfinite((*v)[k]))
        return false;
  return true;
}

// The heights, [i + j * n1]. False if any is not finite.
bool readHeights(const GEO_PrimVolume *vol, const HeightFrame &f,
                 std::vector<float> &out) {
  out.assign(static_cast<size_t>(f.n1) * static_cast<size_t>(f.n2), 0.0f);
  UT_VoxelArrayReadHandleF handle;
  handle = vol->getVoxelHandle();
  UT_VoxelArrayIteratorF vit;
  vit.setConstArray(&*handle);
  for (vit.rewind(); !vit.atEnd(); vit.advance()) {
    const int idx[3] = {vit.x(), vit.y(), vit.z()};
    const float v = vit.getValue();
    if (!std::isfinite(v))
      return false;
    out[static_cast<size_t>(idx[f.a1]) +
        static_cast<size_t>(idx[f.a2]) * static_cast<size_t>(f.n1)] = v;
  }
  return true;
}

uint64_t hashFrame(uint64_t sig, const HeightFrame &f) {
  sig = b3dop::mixHash(sig, static_cast<uint64_t>(f.n1));
  sig = b3dop::mixHash(sig, static_cast<uint64_t>(f.n2));
  for (const UT_Vector3D *v : {&f.origin, &f.e1, &f.e2, &f.up})
    for (int k = 0; k < 3; ++k)
      sig = b3dop::mixDouble(sig, (*v)[k]);
  return sig;
}

// The height field as triangles in object space, facing up (counter-clockwise
// seen from above): for a transform a Box3D height field cannot take (shear).
void buildHeightMesh(const GEO_PrimVolume *vol, const HeightFrame &f,
                     const std::vector<float> &heights, TriMesh &out) {
  out.xyz.clear();
  out.tris.clear();
  out.xyz.reserve(heights.size() * 3);
  for (int j = 0; j < f.n2; ++j)
    for (int i = 0; i < f.n1; ++i) {
      exint idx[3] = {0, 0, 0};
      idx[f.a1] = i;
      idx[f.a2] = j;
      UT_Vector3D p;
      vol->indexToPos(idx[0], idx[1], idx[2], p); // follows any taper
      p += f.up *
           static_cast<double>(
               heights[static_cast<size_t>(i) +
                       static_cast<size_t>(j) * static_cast<size_t>(f.n1)]);
      for (int k = 0; k < 3; ++k)
        out.xyz.push_back(static_cast<float>(p[k]));
    }
  // (e1, e2, up) right-handed: (i, j) -> (i+1, j) -> (i+1, j+1) turns
  // counter-clockwise seen from up.
  const bool ccw = dot(cross(f.e1, f.e2), f.up) > 0.0;
  for (int j = 0; j + 1 < f.n2; ++j)
    for (int i = 0; i + 1 < f.n1; ++i) {
      const int32_t v00 = j * f.n1 + i, v10 = v00 + 1, v01 = v00 + f.n1,
                    v11 = v01 + 1;
      const int32_t t[6] = {v00, v10, v11, v00, v11, v01};
      for (int k = 0; k < 6; k += 3) {
        out.tris.push_back(t[k]);
        out.tris.push_back(ccw ? t[k + 1] : t[k + 2]);
        out.tris.push_back(ccw ? t[k + 2] : t[k + 1]);
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

  // A height field (see prepareHeightField): `heightField` builds a Box3D
  // height field posed by pos / rot with `scale` as its grid spacing and
  // height scale; otherwise `volume` set means its triangles, as a mesh.
  const GEO_PrimVolume *volume = nullptr;
  HeightFrame frame;
  bool heightField = false;
  bool transposed =
      false; // grid columns along a2 (keeps the frame right-handed)
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

// Material (bodies are always static): a Box3D Data node on the affector sets
// friction and bounce; without one, the object's own Friction and Bounce
// (PhysicalParms: Static Object's and Ground Plane's Physical tab, as RBD
// Object's are read), else Box3D Data's defaults.
b3dop::BodySettings affectorBodySettings(const SIM_Object &obj) {
  b3dop::BodySettings body = b3dop::nodeBodyDefaults();
  if (const SIM_Box3DData *node = findBox3DData(obj))
    body = node->toBodySettings();
  else if (const SIM_Data *phys = obj.getConstNamedSubData("PhysicalParms")) {
    // Box3D asserts on negative material values: those keep the default.
    fpreal64 v = 0.0;
    if (box3dDataOption(*phys, "friction", v) && std::isfinite(v) && v >= 0.0)
      body.props.friction = static_cast<float>(v);
    if (box3dDataOption(*phys, "bounce", v) && std::isfinite(v) && v >= 0.0)
      body.props.restitution = static_cast<float>(v);
  }
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

// A height-field affector. The Box3D height field's frame (columns along x,
// heights up y, rows along z) is the volume's (e1, up, e2), or (e2, up, e1)
// when that one is mirrored; times the object's transform it must still be a
// rotation with a positive scale per axis. The pose then goes to the body and
// the scale into the height field (so a scale change rebuilds it). Anything
// else (shear, a scale along a slanted axis) falls back to its triangles.
// The heights are hashed only when the geometry changed (`ab` keeps the hash).
bool prepareHeightField(Box3DAffectorBody &ab, const GU_Detail *gdp,
                        const GEO_PrimVolume *vol, AffectorInput &in,
                        std::vector<float> &heights, bool &heightsRead) {
  heightsRead = false;
  if (!readHeightFrame(vol, in.frame))
    return false;
  in.volume = vol;
  const HeightFrame &f = in.frame;

  const uint64_t source = geometryIdentity(gdp, /*withUniqueId=*/true);
  if (!ab.heightsKnown || ab.heightSource != source) {
    if (!readHeights(vol, f, heights))
      return false;
    heightsRead = true;
    ab.heightHash = b3dop::hashFloats(heights.data(), heights.size());
    ab.heightSource = source;
    ab.heightsKnown = true;
  }

  const UT_Vector3D u1 = f.e1 / f.e1.length(), u2 = f.e2 / f.e2.length();
  for (int attempt = 0; attempt < 2 && !in.heightField; ++attempt) {
    const bool transposed = attempt == 1;
    const UT_Vector3D &bx = transposed ? u2 : u1, &bz = transposed ? u1 : u2;
    const double local[16] = {
        bx[0], bx[1], bx[2], 0, f.up[0],     f.up[1],     f.up[2],     0,
        bz[0], bz[1], bz[2], 0, f.origin[0], f.origin[1], f.origin[2], 1};
    double world[16];
    for (int r = 0; r < 4; ++r)
      for (int c = 0; c < 4; ++c) {
        double v = 0.0;
        for (int k = 0; k < 4; ++k)
          v += local[4 * r + k] * in.m[4 * k + c];
        world[4 * r + c] = v;
      }
    double pos[3];
    float rot[4], sc[3];
    if (!b3dop::decomposeScaledRigid(world, pos, rot, sc))
      continue;
    const double stepX = (transposed ? f.e2 : f.e1).length();
    const double stepZ = (transposed ? f.e1 : f.e2).length();
    in.heightField = true;
    in.transposed = transposed;
    in.rigid = true; // the pose and scale above
    b3dop::copy3(in.pos, pos);
    b3dop::copy4(in.rot, rot);
    in.scale[0] = static_cast<float>(stepX * sc[0]);
    in.scale[1] = sc[1];
    in.scale[2] = static_cast<float>(stepZ * sc[2]);
  }

  // Heights, grid, material and mode; a height field also its scale, the
  // triangle fallback (like a mesh) its matrix when baked.
  uint64_t sig = b3dop::mixHash(b3dop::kHashSeed, ab.heightHash);
  sig = hashFrame(sig, f);
  const float mat[2] = {in.body.props.friction, in.body.props.restitution};
  sig = b3dop::mixHash(sig, b3dop::hashFloats(mat, 2));
  if (in.heightField) {
    sig = b3dop::mixHash(sig, in.transposed ? 3 : 4);
    sig = b3dop::mixHash(sig, b3dop::hashFloats(in.scale, 3));
  } else {
    sig = b3dop::mixHash(sig, in.rigid ? 1 : 2);
    if (!in.rigid)
      for (double d : in.m)
        sig = b3dop::mixDouble(sig, d);
  }
  in.sig = sig;
  return true;
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

// A Box3D height field from the volume: one static body at in.pos / in.rot.
bool buildHeightFieldBody(Box3DSharedState &st, Box3DAffectorBody &ab,
                          const AffectorInput &in,
                          const std::vector<float> &heights,
                          std::vector<float> &grid) {
  const HeightFrame &f = in.frame;
  // Box3D's grid: [row * countX + column], columns along its x.
  const int countX = in.transposed ? f.n2 : f.n1;
  const int countZ = in.transposed ? f.n1 : f.n2;
  if (!in.transposed)
    grid = heights; // [i + j * n1]: column i, row j
  else {
    grid.resize(heights.size());
    for (int j = 0; j < f.n2; ++j)
      for (int i = 0; i < f.n1; ++i) // column j, row i
        grid[static_cast<size_t>(i) * static_cast<size_t>(countX) +
             static_cast<size_t>(j)] =
            heights[static_cast<size_t>(i) +
                    static_cast<size_t>(j) * static_cast<size_t>(f.n1)];
  }
  const b3dop::BodySettings &body = in.body;
  const b3dop::BodyHandle h =
      st.world.createBody(body, in.pos, in.rot, nullptr);
  if (!h)
    return false;
  if (!st.world.createHeightFieldShape(h, body, grid.data(), countX, countZ,
                                       in.scale)) {
    st.world.destroyBody(h);
    return false;
  }
  ab.handle = h;
  return true;
}

// (Re)build from scratch: one static body with one triangle mesh shape, or
// one height field. `mesh` holds the object-space triangles for a mesh.
void buildAffector(Box3DSharedState &st, Box3DAffectorBody &ab, int objectId,
                   const AffectorInput &in, TriMesh &mesh,
                   std::vector<float> &baked, const std::vector<float> &heights,
                   Box3DAffectorCounts &counts) {
  destroyAffector(st, ab);
  ab.signature = in.sig;
  ab.poseKnown = true;
  const b3dop::BodySettings &body = in.body;

  if (in.heightField) {
    if (!buildHeightFieldBody(st, ab, in, heights, baked)) {
      ++counts.failed;
      return;
    }
    ab.rolling = body.props.rollingResistance;
    b3dop::copy3(ab.tangent, body.props.tangentVelocity);
    b3dop::copy3(ab.pos, in.pos);
    b3dop::copy4(ab.rot, in.rot);
    b3dop::copy3(ab.scale, in.scale);
    ++counts.added;
    ++counts.heightFields;
    B3D_LOG("  affector id=%d: static height field built, %d x %d", objectId,
            in.frame.n1, in.frame.n2);
    return;
  }

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
  if (in.volume)
    ++counts.heightFields;
  B3D_LOG("  affector id=%d: static mesh built, %d triangle(s)%s%s", objectId,
          static_cast<int>(mesh.tris.size() / 3),
          in.volume ? " from a height field" : "",
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
  std::vector<float> baked, heights;
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

    // A height layer makes the affector a height field (its other primitives
    // are ignored); otherwise its polygons are a mesh.
    bool heightsRead = false;
    if (const GEO_PrimVolume *vol = findHeightVolume(gdp)) {
      if (!prepareHeightField(ab, gdp, vol, in, heights, heightsRead)) {
        destroyAffector(st, ab);
        ab.poseKnown = false;
        ++counts.failed;
        continue;
      }
    } else
      in.sig = affectorSignature(gdp, in);

    if (ab.poseKnown && ab.signature == in.sig) {
      updateAffector(st, ab, in, counts);
      continue;
    }
    if (in.volume && !heightsRead &&
        !readHeights(in.volume, in.frame, heights)) {
      destroyAffector(st, ab);
      ab.poseKnown = false;
      ++counts.failed;
      continue;
    }
    mesh.xyz.clear();
    mesh.tris.clear();
    if (!in.volume)
      buildMesh(gdp, !in.body.flipMeshWinding, mesh);
    else if (!in.heightField)
      buildHeightMesh(in.volume, in.frame, heights, mesh);
    buildAffector(st, ab, obj->getObjectId(), in, mesh, baked, heights, counts);
  }

  removeStaleAffectors(st, counts);
}
