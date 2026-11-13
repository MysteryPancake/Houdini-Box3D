// Box3DHoudiniUtil.h
//
// Small Houdini helpers shared by the solver, the joint sync and the affectors:
// writable geometry data, objects by id, matrices as plain arrays, Euler angles
// to quaternions, options of SIM data whose class is not in the HDK.
#pragma once

#include <GU/GU_Detail.h>
#include <SIM/SIM_DataUtils.h>
#include <SIM/SIM_Geometry.h>
#include <SIM/SIM_GeometryCopy.h>
#include <SIM/SIM_Object.h>
#include <SIM/SIM_ObjectArray.h>
#include <SIM/SIM_Options.h>
#include <SIM/SIM_OptionsUser.h>
#include <SIM/SIM_Query.h>
#include <UT/UT_Quaternion.h>
#include <UT/UT_StringHolder.h>
#include <UT/UT_Vector3.h>
#include <UT/UT_XformOrder.h>

#include "core/Box3DShapes.h"

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
  for (exint i = 1; i < objects.entries(); ++i)
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
    exint lo = 0, hi = objects.entries();
    while (lo < hi) {
      const exint mid = lo + (hi - lo) / 2;
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
  for (exint i = 0; i < objects.entries(); ++i)
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
    h = mixHash(h, uint64_t(gdp->getUniqueId()));
  const GA_Attribute *P = gdp->getP();
  h = mixHash(h, P ? uint64_t(P->getDataId()) : ~0ull);
  h = mixHash(h, gdp->getNumPrimitives() > 0
                     ? uint64_t(gdp->getPrimitiveList().getDataId())
                     : ~0ull);
  h = mixHash(h, uint64_t(gdp->getNumPoints()));
  return mixHash(h, uint64_t(gdp->getNumPrimitives()));
}

// A Houdini 4x4 (row-vector layout) as 16 doubles, row by row.
template <class Matrix4>
inline void box3dToArray(const Matrix4 &m, double out[16]) {
  for (int r = 0; r < 4; ++r)
    for (int c = 0; c < 4; ++c)
      out[4 * r + c] = double(m(r, c));
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

inline bool box3dDataOption(const SIM_Data &data, const char *name,
                            fpreal64 &out) {
  if (const SIM_Options *opts = box3dDataOptions(data))
    return opts->importOption(name, out);
  fpreal v = 0.0;
  if (!data.getQueryObject().getFieldFloat("Options", 0, name, v))
    return false;
  out = v;
  return true;
}

inline bool box3dDataOption(const SIM_Data &data, const char *name,
                            UT_Vector3D &out) {
  const SIM_Options *opts = box3dDataOptions(data);
  return opts && opts->importOption(name, out);
}

inline bool box3dDataOption(const SIM_Data &data, const char *name,
                            UT_StringHolder &out) {
  if (const SIM_Options *opts = box3dDataOptions(data))
    return opts->importOption(name, out);
  return data.getQueryObject().getFieldString("Options", 0, name, out);
}
