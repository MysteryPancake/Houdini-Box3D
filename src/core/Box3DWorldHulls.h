// Box3DWorldHulls.h
//
// The process-wide hull and fit caches, and the helpers that build hulls from
// point sets: shared by Box3DWorldShapes.cpp and Box3DWorldHulls.cpp. It
// includes Box3D headers, so only the Box3DWorld*.cpp files include it.
#pragma once

#include "Box3DShapes.h"
#include "Box3DWorldInternal.h"

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <mutex>
#include <shared_mutex>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace b3dop {

// ---- the process-wide hull cache
// Box3D adds a hull to the world's own database when a shape is made (identical
// hulls share one clone), so entries here are only needed until then: they can
// be shared by every world (a rebuilt world finds what its predecessor built)
// and evicted at any time. Worlds hold the entries they use in a step
// (Box3DWorld::Impl::held) so eviction never frees one between findHull and
// createHullShape.
class HullLibrary {
public:
  static HullLibrary &instance() {
    static HullLibrary lib;
    return lib;
  }

  HullRef find(HullId key) {
    std::lock_guard<std::mutex> lock(myMutex);
    auto it = myEntries.find(key);
    if (it == myEntries.end())
      return nullptr;
    it->second.lastUse = ++myTick;
    return it->second.ref;
  }

  // Takes `hull`; returns the cached entry (an earlier insert of the same key
  // wins).
  HullRef insert(HullId key, b3HullData *hull) {
    auto fresh = std::make_shared<SharedHull>();
    fresh->hull = hull;
    fresh->volume = hull->volume;
    // The hull's whole allocation: header, vertex / point / edge / plane / face
    // arrays, the SoA vertex and normal arrays and the edge cosines.
    fresh->bytes = static_cast<size_t>(hull->byteCount);
    std::lock_guard<std::mutex> lock(myMutex);
    auto ins = myEntries.try_emplace(key);
    if (ins.second) {
      ins.first->second.ref = fresh;
      myBytes += fresh->bytes;
    }
    ins.first->second.lastUse = ++myTick;
    HullRef out = ins.first->second.ref;
    if (myBytes > myBudget)
      evictLocked();
    return out;
  }

  // The vertex budget that last worked for a group (0: none yet).
  int budgetHint(uint64_t group) {
    if (!group)
      return 0;
    std::lock_guard<std::mutex> lock(myMutex);
    auto it = myHints.find(group);
    return it == myHints.end() ? 0 : it->second;
  }
  void setBudgetHint(uint64_t group, int budget) {
    if (!group)
      return;
    std::lock_guard<std::mutex> lock(myMutex);
    if (myHints.size() > 65536)
      myHints.clear();
    myHints[group] = budget;
  }

  // Keys whose points cannot be hulled (flat, too complex): a key always means
  // the same points, so they are not tried again.
  bool knownFailure(HullId key) {
    std::lock_guard<std::mutex> lock(myMutex);
    return myFailed.count(key) != 0;
  }
  void noteFailure(HullId key) {
    std::lock_guard<std::mutex> lock(myMutex);
    if (myFailed.size() >= kMaxFailedKeys)
      myFailed.clear();
    myFailed.insert(key);
  }

  size_t bytes() {
    std::lock_guard<std::mutex> lock(myMutex);
    return myBytes;
  }
  void setBudget(size_t b) {
    std::lock_guard<std::mutex> lock(myMutex);
    myBudget = b;
    if (myBytes > myBudget)
      evictLocked();
  }

  std::atomic<int> builds{0}, attempts{0};

private:
  struct Entry {
    HullRef ref;
    uint64_t lastUse = 0;
  };

  // Least recently used first, down to 3/4 of the budget. Held entries stay
  // alive through their HullRef; they just stop being found.
  void evictLocked() {
    std::vector<std::pair<uint64_t, HullId>> order;
    order.reserve(myEntries.size());
    for (const auto &kv : myEntries)
      order.emplace_back(kv.second.lastUse, kv.first);
    std::sort(order.begin(), order.end());
    const size_t target = myBudget / 4 * 3;
    for (const auto &o : order) {
      if (myBytes <= target)
        break;
      auto it = myEntries.find(o.second);
      myBytes -= it->second.ref->bytes;
      myEntries.erase(it);
    }
  }

  std::mutex myMutex;
  std::unordered_map<HullId, Entry> myEntries;
  std::unordered_map<uint64_t, int> myHints;
  static constexpr size_t kMaxFailedKeys = 65536;
  std::unordered_set<HullId> myFailed;
  size_t myBytes = 0;
  size_t myBudget = static_cast<size_t>(256) << 20;
  uint64_t myTick = 0;
};

// ---- the process-wide fit cache ---------------------------------------------
// What fitShapeOriented found for a point set (by content and shape type, or
// under a caller's key: findFit / storeFit), so many packed instances of one
// piece fit once. Half is dropped when it grows too big.
class FitLibrary {
public:
  static FitLibrary &instance() {
    static FitLibrary lib;
    return lib;
  }
  // Under a shared lock: every packed instance of a fresh scan asks, from
  // several threads, and a lookup changes nothing.
  bool find(uint64_t key, ShapeSpec &out) {
    // The thread's last hit first (a fit is a function of its key, so it never
    // changes), skipping the shared lock every thread would take per point.
    thread_local uint64_t memoKey = 0;
    thread_local ShapeSpec memo;
    if (memoKey == key && key != 0) {
      out = memo;
      return true;
    }
    std::shared_lock<std::shared_mutex> lock(myMutex);
    const auto it = myEntries.find(key);
    if (it == myEntries.end())
      return false;
    memoKey = key;
    memo = it->second;
    out = it->second;
    return true;
  }
  void insert(uint64_t key, const ShapeSpec &spec) {
    std::unique_lock<std::shared_mutex> lock(myMutex);
    if (myEntries.size() >= kMaxFitEntries) {
      // Half goes (in the map's order), so a scene past the cap keeps most of
      // what it just used instead of losing all of it.
      for (auto it = myEntries.begin();
           it != myEntries.end() && myEntries.size() > kMaxFitEntries / 2;)
        it = myEntries.erase(it);
    }
    myEntries.emplace(key, spec);
  }

private:
  static constexpr size_t kMaxFitEntries = 262144;
  std::shared_mutex myMutex;
  std::unordered_map<uint64_t, ShapeSpec> myEntries;
};

// A content key for body-local points. Never 0.
inline HullId hashPoints(const float *xyz, int count) {
  const uint64_t h = hashFloats(xyz, static_cast<size_t>(count) * 3);
  return h ? h : 1ull;
}

// The points as b3Vec3. False if any is not finite (Box3D asserts on them).
inline bool loadPoints(const float *xyz, int count, std::vector<b3Vec3> &pts) {
  for (int i = 0; i < count * 3; ++i)
    if (!std::isfinite(xyz[i]))
      return false;
  pts.resize(static_cast<size_t>(count));
  for (int i = 0; i < count; ++i)
    pts[static_cast<size_t>(i)] = makeVec3(xyz + 3 * i);
  return true;
}

// b3CreateHull fails for degenerate input or when the result would exceed
// Box3D's vertex / face / edge limits; a smaller vertex budget keeps fewer
// vertices, so dense input still gets a (slightly smaller) hull. A
// triangulated hull hits the 256 half-edge limit at about 44 vertices, which
// is why there is no rung between the maximum and 44: a 64-vertex budget
// fails whenever the maximum did.
inline constexpr int kHullBudgets[] = {B3_MAX_HULL_VERTICES, 44, 32, 16, 8};

// The hull with the largest budget that works, skipping budgets above
// `maxBudget` (0: none skipped). `usedBudget` is the budget that worked,
// `tries` the b3CreateHull calls made. Null if none works.
inline b3HullData *createHullLadder(const std::vector<b3Vec3> &pts,
                                    int maxBudget, int &usedBudget,
                                    int &tries) {
  usedBudget = 0;
  tries = 0;
  for (const int budget : kHullBudgets) {
    if (maxBudget && budget > maxBudget)
      continue;
    ++tries;
    b3HullData *hull =
        b3CreateHull(pts.data(), static_cast<int>(pts.size()), budget);
    if (hull) {
      usedBudget = budget;
      return hull;
    }
  }
  return nullptr;
}

} // namespace b3dop
