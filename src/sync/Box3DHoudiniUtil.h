// Box3DHoudiniUtil.h
//
// Small Houdini helpers shared by the solver, the joint sync and the affectors:
// writable geometry data, objects by id, matrices as plain arrays, Euler angles
// to quaternions, rotations since rest, options of SIM data whose class is not
// in the HDK.
#pragma once

#include <GA/GA_Iterator.h>
#include <GA/GA_Primitive.h>
#include <GA/GA_PrimitiveTypes.h>
#include <GA/GA_Range.h>
#include <GU/GU_Detail.h>
#include <SIM/SIM_DataUtils.h>
#include <SIM/SIM_Geometry.h>
#include <SIM/SIM_GeometryCopy.h>
#include <SIM/SIM_Object.h>
#include <SIM/SIM_ObjectArray.h>
#include <SIM/SIM_Options.h>
#include <SIM/SIM_OptionsUser.h>
#include <SIM/SIM_Query.h>
#include <UT/UT_Matrix3.h>
#include <UT/UT_Matrix4.h>
#include <UT/UT_ParallelUtil.h>
#include <UT/UT_Quaternion.h>
#include <UT/UT_StringHolder.h>
#include <UT/UT_Vector3.h>
#include <UT/UT_XformOrder.h>

#include "core/Box3DShapes.h"

#include <cstdint>
#include <unordered_map>
#include <vector>

// Houdini's quaternion product composes in the order of the row-vector
// matrices: M(a * b) = M(b) * M(a). So Bullet's packed convention, absolute =
// rest * since as matrices, is the product `since * rest` here.
// f(i) for i in [0, n), on Houdini's TBB pool when n >= minParallel. f may only
// read shared data and write element i of its own output.
template <class F>
void box3dParallelFor(size_t n, const F &f, size_t minParallel,
                      size_t grain = 1) {
  if (n < minParallel) {
    for (size_t i = 0; i < n; ++i)
      f(i);
    return;
  }
  UTparallelFor(UT_BlockedRange<size_t>(0, n, grain),
                [&f](const UT_BlockedRange<size_t> &r) {
                  for (size_t i = r.begin(); i != r.end(); ++i)
                    f(i);
                });
}

inline UT_QuaternionD box3dQuatD(const float q[4]) {
  return UT_QuaternionD(q[0], q[1], q[2], q[3]);
}

// `q` normalised, with w >= 0 (q and -q are the same rotation), as floats.
inline void box3dStoreQuat(UT_QuaternionD q, float out[4]) {
  q.normalize();
  const double sign = q.w() < 0.0 ? -1.0 : 1.0;
  out[0] = static_cast<float>(sign * q.x());
  out[1] = static_cast<float>(sign * q.y());
  out[2] = static_cast<float>(sign * q.z());
  out[3] = static_cast<float>(sign * q.w());
  b3dop::sanitizeQuat(out);
}

// Bullet's packed convention: absolute = rest * since (row-vector matrices,
// rest first), `since` being the point's `orient` and `rest` its restxform's
// rotation.
inline void box3dRestToAbsolute(const float rest[4], const float since[4],
                                float out[4]) {
  box3dStoreQuat(box3dQuatD(since) * box3dQuatD(rest), out);
}

// The inverse: since = rest^T * absolute (as matrices), absolute * conj(rest)
// as quaternions.
inline void box3dAbsoluteToRest(const float rest[4], const float absolute[4],
                                float out[4]) {
  box3dStoreQuat(box3dQuatD(absolute) *
                     UT_QuaternionD(-rest[0], -rest[1], -rest[2], rest[3]),
                 out);
}

// A writable SIM_GeometryCopy for the object's data `name`, or null. A plain
// SIM_Geometry (RBD Packed Object, a SOP Geometry DOP) is replaced once by a
// SIM_GeometryCopy that adopts its geometry.
inline SIM_GeometryCopy *box3dWritableGeometry(SIM_Object &obj,
                                               const char *name) {
  if (SIM_GeometryCopy *geo = SIM_DATA_GET(obj, name, SIM_GeometryCopy))
    return geo;
  if (!SIM_DATA_GETCONST(obj, name, SIM_Geometry))
    return nullptr;
  return SIM_DATA_CREATE(obj, name, SIM_GeometryCopy,
                         SIM_DATA_RETURN_EXISTING |
                             SIM_DATA_ADOPT_EXISTING_ON_DELETE);
}

// True if `objects` is in ascending object id order (checked once per solve).
inline bool box3dObjectsSortedById(SIM_ObjectArray &objects) {
  for (int i = 1; i < objects.entries(); ++i)
    if (objects(i - 1)->getObjectId() > objects(i)->getObjectId())
      return false;
  return true;
}

// The object with this id, or null. Binary search when `sorted`
// (box3dObjectsSortedById: then a miss is final, no scan); an array not known
// to be sorted is scanned linearly.
inline SIM_Object *box3dObjectById(SIM_ObjectArray &objects, int id,
                                   bool sorted) {
  if (sorted) {
    int lo = 0, hi = static_cast<int>(objects.entries());
    while (lo < hi) {
      const int mid = lo + (hi - lo) / 2;
      const int mid_id = objects(mid)->getObjectId();
      if (mid_id == id)
        return objects(mid);
      if (mid_id < id)
        lo = mid + 1;
      else
        hi = mid;
    }
    return nullptr;
  }
  for (int i = 0; i < objects.entries(); ++i)
    if (objects(i)->getObjectId() == id)
      return objects(i);
  return nullptr;
}

// The part of a geometry's identity that every hash of it shares: P's data id,
// the primitive list's data id, the point and primitive counts and, with
// `withUniqueId`, the detail's unique id. Callers mix their own markers on top.
// An absent P and a list with no primitives (its data id means nothing then)
// both count as ~0. The affector signature passes false: a mesh that is copied
// into a new detail with the same contents must not rebuild.
inline uint64_t geometryIdentity(const GU_Detail *gdp,
                                 bool withUniqueId = true) {
  using b3dop::mixHash;
  uint64_t h = b3dop::kHashSeed;
  if (withUniqueId)
    h = mixHash(h, static_cast<uint64_t>(gdp->getUniqueId()));
  const GA_Attribute *P = gdp->getP();
  h = mixHash(h, P ? static_cast<uint64_t>(P->getDataId()) : ~0ull);
  h = mixHash(h,
              gdp->getNumPrimitives() > 0
                  ? static_cast<uint64_t>(gdp->getPrimitiveList().getDataId())
                  : ~0ull);
  h = mixHash(h, static_cast<uint64_t>(gdp->getNumPoints()));
  return mixHash(h, static_cast<uint64_t>(gdp->getNumPrimitives()));
}

// The polygons of `prims` as triangles, each fanned from its first vertex
// (open curves, packed and other primitives are skipped). Vertices are the
// points in order of first use. `reverse` flips the winding. `sparse` for a
// range that is a small part of `gdp` (a packed fragment): the point lookup is
// a hash map instead of an array the size of the whole detail.
inline void box3dTriangulate(const GU_Detail *gdp, const GA_Range &prims,
                             bool sparse, bool reverse, std::vector<float> &xyz,
                             std::vector<int32_t> &tris) {
  std::vector<int32_t> dense;
  std::unordered_map<GA_Offset, int32_t> hashed;
  if (!sparse)
    dense.assign(static_cast<size_t>(gdp->getNumPointOffsets()), -1);
  auto vertexOf = [&](GA_Offset pt) -> int32_t {
    int32_t &idx = sparse ? hashed.try_emplace(pt, -1).first->second
                          : dense[static_cast<size_t>(pt)];
    if (idx < 0) {
      const UT_Vector3 p = gdp->getPos3(pt);
      idx = static_cast<int32_t>(xyz.size() / 3);
      xyz.push_back(p.x());
      xyz.push_back(p.y());
      xyz.push_back(p.z());
    }
    return idx;
  };
  for (GA_Iterator it(prims); !it.atEnd(); ++it) {
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
      tris.push_back(first);
      tris.push_back(reverse ? c : b);
      tris.push_back(reverse ? b : c);
    }
  }
}

// A Houdini 4x4 (row-vector layout) as 16 doubles, row by row.
template <class Matrix4>
inline void box3dToArray(const Matrix4 &m, double out[16]) {
  for (unsigned r = 0; r < 4; ++r)
    for (unsigned c = 0; c < 4; ++c)
      out[4 * r + c] = static_cast<double>(m(r, c));
}

// 16 floats, row by row, as a Houdini 4x4.
inline UT_Matrix4F box3dMatrix4F(const float in[16]) {
  UT_Matrix4F m;
  for (unsigned r = 0; r < 4; ++r)
    for (unsigned c = 0; c < 4; ++c)
      m(r, c) = in[4 * r + c];
  return m;
}

// A double position as a float UT_Vector3.
inline UT_Vector3 box3dVec3F(const double v[3]) {
  return UT_Vector3(static_cast<float>(v[0]), static_cast<float>(v[1]),
                    static_cast<float>(v[2]));
}

// Euler angles in degrees (Houdini's XYZ rotate order) -> quaternion x, y, z,
// w.
inline void box3dEulerToQuat(const float degrees[3], float q[4]) {
  const UT_Vector3F rad(degrees[0] * b3dop::kDegToRad,
                        degrees[1] * b3dop::kDegToRad,
                        degrees[2] * b3dop::kDegToRad);
  const UT_QuaternionF uq(
      rad, UT_XformOrder(UT_XformOrder::TRS, UT_XformOrder::XYZ));
  q[0] = uq.x();
  q[1] = uq.y();
  q[2] = uq.z();
  q[3] = uq.w();
}

// An option of any SIM data by name, for data whose class is not in the HDK
// (SideFX's constraint data, RBD State, Bullet Data, ...). They are
// SIM_OptionsUser data (polymorphic: virtual destructor), so a dynamic_cast
// reaches their options; failing that, the generic query that dopfield() uses
// gives scalars and strings. False if the data has no such option.
inline const SIM_Options *box3dDataOptions(const SIM_Data &data) {
  const SIM_OptionsUser *user = dynamic_cast<const SIM_OptionsUser *>(&data);
  return user ? &user->getOptions() : nullptr;
}

// The same with the options looked up once (box3dDataOptions), for reading many
// options of one data.
inline bool box3dDataOption(const SIM_Data &data, const SIM_Options *opts,
                            const char *name, fpreal64 &out) {
  if (opts)
    return opts->importOption(name, out);
  fpreal v = 0.0;
  if (!data.getQueryObject().getFieldFloat("Options", 0, name, v))
    return false;
  out = v;
  return true;
}

inline bool box3dDataOption(const SIM_Data &, const SIM_Options *opts,
                            const char *name, UT_Vector3D &out) {
  return opts && opts->importOption(name, out);
}

inline bool box3dDataOption(const SIM_Data &data, const SIM_Options *opts,
                            const char *name, UT_StringHolder &out) {
  if (opts)
    return opts->importOption(name, out);
  return data.getQueryObject().getFieldString("Options", 0, name, out);
}

template <class T>
inline bool box3dDataOption(const SIM_Data &data, const char *name, T &out) {
  return box3dDataOption(data, box3dDataOptions(data), name, out);
}
