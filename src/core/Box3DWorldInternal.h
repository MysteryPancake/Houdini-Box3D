// Box3DWorldInternal.h
//
// What Box3DWorld.cpp and Box3DWorldJoints.cpp share: the world's private
// state and a few Box3D conversions. It includes Box3D headers, so only those
// two files include it.
#pragma once

#include "Box3DWorld.h"

#include <box3d/box3d.h>

#include <cmath>
#include <memory>
#include <shared_mutex>
#include <unordered_map>
#include <utility>
#include <vector>

namespace b3dop {

struct SharedHull {
  b3HullData *hull = nullptr;
  float volume = 0.0f;
  size_t bytes = 0;
  ~SharedHull() {
    if (hull)
      b3DestroyHull(hull);
  }
};
using HullRef = std::shared_ptr<const SharedHull>;

struct Box3DWorld::Impl {
  b3WorldId worldId = b3_nullWorldId;
  WorldSettings settings;
  std::shared_ptr<TaskRunner>
      tasks;          // host threads (null: Box3D's own pool); outlives worldId
  int liveBodies = 0; // every live body
  std::vector<std::pair<BodyHandle, BodyHandle>>
      hitPairs; // updateHitEvents(): (lower, higher) handle, sorted, unique

  // Shared hulls this world found or built since the last pruneHulls()
  // (findHull may run on several threads, hence the lock; most calls only
  // read).
  std::shared_mutex held_mutex;
  std::unordered_map<HullId, HullRef> held;

  HullRef heldHull(HullId id) {
    std::shared_lock<std::shared_mutex> lock(held_mutex);
    auto it = held.find(id);
    return it == held.end() ? nullptr : it->second;
  }
  void hold(HullId id, const HullRef &ref) {
    std::unique_lock<std::shared_mutex> lock(held_mutex);
    held.emplace(id, ref);
  }

  // The world end of joints with a null BodyHandle: a shapeless static body at
  // the origin, made on first use and not counted in liveBodies.
  b3BodyId anchor = b3_nullBodyId;

  // Triangle meshes and height fields for static colliders. Box3D keeps a
  // pointer to the b3MeshData / b3HeightFieldData (it does not clone them), so
  // we own them: freed after their body is destroyed, or after the world.
  struct OwnedGeometry {
    std::vector<b3MeshData *> meshes;
    std::vector<b3HeightFieldData *> heightFields;
    void release() {
      for (b3MeshData *m : meshes)
        b3DestroyMesh(m);
      for (b3HeightFieldData *hf : heightFields)
        b3DestroyHeightField(hf);
      meshes.clear();
      heightFields.clear();
    }
  };
  std::unordered_map<BodyHandle, OwnedGeometry> bodyGeometry;

  // The meshes and height fields a body's shapes used. Call after the body or
  // its shapes are destroyed.
  void releaseBodyResources(BodyHandle body) {
    auto it = bodyGeometry.find(body);
    if (it == bodyGeometry.end())
      return;
    it->second.release();
    bodyGeometry.erase(it);
  }
  void destroyAllMeshes() {
    for (auto &kv : bodyGeometry)
      kv.second.release();
    bodyGeometry.clear();
  }
};

// Non-finite components become 0 (Box3D asserts on them, and a NaN velocity
// spreads to the body's position and its broad-phase bounds).
inline b3Vec3 finiteVec3(const float v[3]) {
  b3Vec3 out;
  out.x = std::isfinite(v[0]) ? v[0] : 0.0f;
  out.y = std::isfinite(v[1]) ? v[1] : 0.0f;
  out.z = std::isfinite(v[2]) ? v[2] : 0.0f;
  return out;
}

inline b3Quat makeQuat(const float q[4]) // x, y, z, w
{
  b3Quat r;
  r.v.x = q[0];
  r.v.y = q[1];
  r.v.z = q[2];
  r.s = q[3];
  return r;
}

inline void storeQuat(const b3Quat &q, float out[4]) {
  out[0] = q.v.x;
  out[1] = q.v.y;
  out[2] = q.v.z;
  out[3] = q.s;
}

inline void storeVec3(const b3Vec3 &v, float out[3]) {
  out[0] = v.x;
  out[1] = v.y;
  out[2] = v.z;
}

} // namespace b3dop
