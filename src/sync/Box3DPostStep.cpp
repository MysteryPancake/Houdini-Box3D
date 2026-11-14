// Box3DPostStep.cpp
//
// After the step: the write-back of awake bodies to their points (and whole
// objects' Position data), Impacts, queued point writes, aging and dead point
// deletion.
#include "Box3DSolverInternal.h"
#include "dop/SIM_SolverBox3D.h"

#include <GA/GA_SplittableRange.h>

#include <array>

namespace box3dsolver {

// Adds the float point attribute `name` with the given type info unless it
// exists. True if added.
static bool addPointAttribIfMissing(GU_Detail *gdp, const char *name,
                                    int tupleSize, GA_TypeInfo typeInfo) {
  if (gdp->findPointAttribute(name))
    return false;
  // addFloatTuple returns the new attribute, so no second lookup.
  if (GA_Attribute *attr = gdp->addFloatTuple(GA_ATTRIB_POINT, name, tupleSize))
    attr->setTypeInfo(typeInfo);
  return true;
}

// Creates `orient`, `v`, `w` if missing. True if `orient` was created.
bool ensureResultAttribs(GU_Detail *gdp) {
  addPointAttribIfMissing(gdp, "v", 3, GA_TYPE_VECTOR);
  addPointAttribIfMissing(gdp, "w", 3, GA_TYPE_VECTOR);
  return addPointAttribIfMissing(gdp, "orient", 4, GA_TYPE_QUATERNION);
}

// Harden only the offset ranges the parallel write pass touches (hardenAllPages
// takes a range). Written offsets are bucketed by 1024 and runs of buckets
// hardened; nearly all hit: one full call. `end` is the first offset after a
// run, right whether the HDK reads it as exclusive or not.
void hardenWrittenPages(const GU_Detail *gdp,
                        const std::vector<Box3DWriteItem> &items, size_t begin,
                        size_t end,
                        const std::array<GA_Attribute *, 5> &attrs) {
  constexpr int64_t kBucketBits = 10; // 1024 offsets
  const int64_t numOffsets = static_cast<int64_t>(gdp->getNumPointOffsets());
  const size_t numBuckets =
      static_cast<size_t>((numOffsets >> kBucketBits) + 1);
  std::vector<char> hit(numBuckets, 0);
  size_t hitCount = 0;
  for (size_t k = begin; k < end; ++k) {
    const GA_Offset off = items[k].off;
    if (off == GA_INVALID_OFFSET || static_cast<int64_t>(off) < 0 ||
        static_cast<int64_t>(off) >= numOffsets)
      continue;
    char &h =
        hit[static_cast<size_t>(static_cast<int64_t>(off) >> kBucketBits)];
    hitCount += h ? 0 : 1;
    h = 1;
  }
  if (hitCount == 0)
    return;

  const bool nearlyAll = hitCount * 10 >= numBuckets * 9;
  for (GA_Attribute *attr : attrs) {
    if (!attr)
      continue;
    if (nearlyAll) {
      attr->hardenAllPages();
      continue;
    }
    for (size_t b = 0; b < numBuckets;) {
      if (!hit[b]) {
        ++b;
        continue;
      }
      size_t e = b;
      while (e + 1 < numBuckets && hit[e + 1])
        ++e;
      attr->hardenAllPages(
          GA_Offset(static_cast<int64_t>(b) << kBucketBits),
          GA_Offset(static_cast<int64_t>(e + 1) << kBucketBits));
      b = e + 1;
    }
  }
}

// Each item's point offset in the geometry about to be written: the entry's
// stored offset if it still names the point (in range, same id / number), else
// looked up by id; GA_INVALID_OFFSET if not found.
template <class Item> // needs .entry and .off
void resolvePointOffsets(const GU_Detail *gdp, const PointAttrs &a,
                         std::vector<Item> &items, size_t begin, size_t end) {
  std::unordered_map<int64_t, GA_Offset> byId; // built only when needed
  bool builtById = false;

  for (size_t k = begin; k < end; ++k) {
    Item &w = items[k];
    const Box3DBodyMirror &m = *w.entry;
    const GA_Offset stored = GA_Offset(m.pointOff);

    bool ok = stored >= 0 && stored < gdp->getNumPointOffsets() &&
              !gdp->getPointMap().isOffsetVacant(stored);
    if (ok)
      ok = a.id.isValid()
               ? static_cast<int64_t>(a.id.get(stored)) == m.pointId
               : static_cast<int64_t>(gdp->pointIndex(stored)) == m.pointId;
    if (ok) {
      w.off = stored;
      continue;
    }

    w.off = GA_INVALID_OFFSET;
    if (!a.id.isValid()) {
      if (m.pointId >= 0 &&
          m.pointId < static_cast<int64_t>(gdp->getNumPoints()))
        w.off = gdp->pointOffset(GA_Index(m.pointId));
      continue;
    }
    if (!builtById) {
      builtById = true;
      for (GA_Iterator it(gdp->getPointRange()); !it.atEnd(); ++it)
        byId.emplace(static_cast<int64_t>(a.id.get(*it)),
                     *it); // first point of a duplicate id wins
    }
    const auto f = byId.find(m.pointId);
    if (f != byId.end())
      w.off = f->second;
  }
}

// A writable SIM_GeometryCopy for the object's Geometry. Objects from RBD
// Packed Object carry a plain SIM_Geometry: it is replaced once by a
// SIM_GeometryCopy that adopts its geometry.
SIM_GeometryCopy *getOrMakeWritableGeometry(SIM_Object &obj) {
  return box3dWritableGeometry(obj, SIM_GEOMETRY_DATANAME);
}

// Re-record the fingerprint after our own write, so it does not look like an
// external edit. Call with every write lock released (releasing one may bump
// ids).
void recordFingerprintAfterWrite(Box3DSharedState &st, const SIM_Object &obj) {
  const auto f = st.objectSync.find(obj.getObjectId());
  if (f == st.objectSync.end() || f->second.visited != st.syncStamp)
    return;
  const SIM_Geometry *geo =
      SIM_DATA_GETCONST(obj, SIM_GEOMETRY_DATANAME, SIM_Geometry);
  if (!geo)
    return;
  SIM_GeometryAutoReadLock rlock(geo);
  const GU_Detail *gdp = rlock.getGdp();
  if (!gdp)
    return;
  const PointAttrs a(gdp);
  Box3DSyncFingerprint &fp = f->second.fp;
  fp = makeFingerprint(gdp, a, fp.nodeType, fp.nodeProps, fp.addImpact);
}

// Make the next sync scan the object, after a write the scan must act on
// (active, dead, deletions).
void forceScan(Box3DSharedState &st, int objectId) {
  const auto f = st.objectSync.find(objectId);
  if (f != st.objectSync.end())
    f->second.canSkip = false;
}

// f(begin, end) per run of items with the same objectId. Items are first
// grouped by object, in O(n): a stable bucket pass (items of one object keep
// their relative order), not a sort, since objects are few and the order of the
// move events changes from step to step. Runs come in the order each object
// first appears, not in id order. Already grouped items are not moved.
template <class Item, class F>
void forEachObjectRun(std::vector<Item> &items, const F &f) {
  const size_t n = items.size();
  if (n == 0)
    return;

  // Pass 1: the bucket of every item, and the size of every bucket.
  std::unordered_map<int, uint32_t>
      bucketOf; // objectId -> bucket (first-seen order)
  std::vector<uint32_t> bucket(n);
  std::vector<size_t> start; // size of a bucket, then its first slot
  size_t runs = 0;
  int lastId = items[0].objectId;
  uint32_t lastBucket = 0;
  bool haveLast = false;
  for (size_t i = 0; i < n; ++i) {
    const int id = items[i].objectId;
    if (!haveLast || id != lastId) {
      const auto ins =
          bucketOf.try_emplace(id, static_cast<uint32_t>(bucketOf.size()));
      lastBucket = ins.first->second;
      lastId = id;
      haveLast = true;
      ++runs;
    }
    if (lastBucket >= start.size())
      start.resize(static_cast<size_t>(lastBucket) + 1, 0);
    ++start[lastBucket];
    bucket[i] = lastBucket;
  }

  // More runs than objects: an object's items are not together, so move them.
  if (runs != bucketOf.size()) {
    size_t sum = 0;
    for (size_t &c : start) {
      const size_t count = c;
      c = sum;
      sum += count;
    }
    thread_local std::vector<Item> scratch; // kept between steps: no allocation
    scratch.resize(n);
    for (size_t i = 0; i < n; ++i)
      scratch[start[bucket[i]]++] = items[i];
    items.swap(scratch);
  }

  for (size_t runBegin = 0; runBegin < n;) {
    size_t runEnd = runBegin;
    while (runEnd < n && items[runEnd].objectId == items[runBegin].objectId)
      ++runEnd;
    f(runBegin, runEnd);
    runBegin = runEnd;
  }
}

// Add Impact Data: this step's contact points with impulse go into a fresh
// "Impacts" data (SIM_Impacts) on the objects with the option on, only on steps
// with at least one; otherwise the data is removed. Contacts are read from the
// awake bodies (this step's move events, in st.moves) and recorded for both
// bodies, so sleeping bodies cost nothing and each pair is read once. Contacts
// where neither body is awake are skipped, as are asleep dynamic bodies.
// Normal: of the surface hit, pointing at the body that gets the impact. No
// Feedback data (that is for interaction with other solvers).
void recordImpacts(Box3DSharedState &st, SIM_ObjectArray &objects,
                   const SIM_Time &now) {
  // `fromOther`: cp was read from the other body's side.
  struct Hit {
    const Box3DBodyMirror *self, *other;
    b3dop::ContactPoint cp;
    bool fromOther;
  };
  std::unordered_map<int, std::vector<Hit>> hitsOf;

  bool any = false;
  for (const auto &kv : st.objectSync)
    any = any || (kv.second.visited == st.syncStamp && kv.second.addImpact);
  if (any) {
    b3dop::Box3DWorld &world = st.world;
    const bool speedFilter = world.settings().impactMinSpeed > 0.0f;
    if (speedFilter && !st.hitsCollected)
      world.updateHitEvents();
    for (const b3dop::BodyMove &mv : st.moves) {
      const Box3DBodyMirror *e =
          static_cast<const Box3DBodyMirror *>(mv.userData);
      if (!e || !e->handle || !st.isLive(*e) ||
          !world.isAwakeByHandle(e->handle))
        continue;
      const bool selfWants = e->sync->addImpact;
      world.getContactsByHandle(e->handle, st.contacts);
      for (const b3dop::ContactPoint &cp : st.contacts) {
        const Box3DBodyMirror *o =
            static_cast<const Box3DBodyMirror *>(cp.otherUserData);
        // The other side, unless it is awake (it records its own) or an asleep
        // dynamic body.
        const bool otherWants = o && st.isLive(*o) && o->sync->addImpact &&
                                !o->dynamic() &&
                                !world.isAwakeByHandle(cp.other);
        if ((!selfWants && !otherWants) ||
            (speedFilter && !world.hadHitEvent(e->handle, cp.other)))
          continue;
        if (selfWants)
          hitsOf[e->objectId].push_back({e, o, cp, false});
        if (otherWants)
          hitsOf[o->objectId].push_back({o, e, cp, true});
      }
    }
  }

  std::vector<int> wrote;
  for (const auto &kv : hitsOf) {
    SIM_Object *obj = box3dObjectById(objects, kv.first, st.objectsSorted);
    if (!obj)
      continue;
    SIM_Impacts *imp =
        SIM_DATA_CREATE(*obj, SIM_IMPACTS_DATANAME, SIM_Impacts, 0);
    if (!imp)
      continue;
    wrote.push_back(kv.first);
    for (const Hit &h : kv.second) {
      // cp.normal points from the body it was read from into the other one.
      const b3dop::ContactPoint &cp = h.cp;
      const float sgn = h.fromOther ? 1.0f : -1.0f;
      imp->addPointImpact(UT_Vector3(static_cast<float>(cp.pos[0]),
                                     static_cast<float>(cp.pos[1]),
                                     static_cast<float>(cp.pos[2])),
                          UT_Vector3(sgn * cp.normal[0], sgn * cp.normal[1],
                                     sgn * cp.normal[2]),
                          cp.impulse, static_cast<int>(h.self->pointIndex),
                          h.other ? h.other->objectId : -1,
                          h.other ? static_cast<int>(h.other->pointIndex) : -1,
                          -1, 0.0, 0.0, 0.0, now, 0);
    }
  }
  // Had Impacts last step, none now (or the option is off).
  std::sort(wrote.begin(), wrote.end());
  for (const int id : st.impactObjects)
    if (!std::binary_search(wrote.begin(), wrote.end(), id))
      if (SIM_Object *obj = box3dObjectById(objects, id, st.objectsSorted))
        obj->removeNamedSubData(SIM_IMPACTS_DATANAME);
  st.impactObjects.swap(wrote);
}

// The queued point writes: bullet_autofit_valid = 1 and the computed `mass`. A
// point that cannot be found keeps its value (a missed bullet_autofit_valid
// just refits again).
void writePointAttrs(Box3DSharedState &st, SIM_ObjectArray &objects) {
  std::vector<Box3DPointWrite> &items = st.pointWrites;
  if (items.empty())
    return;
  forEachObjectRun(items, [&](size_t begin, size_t end) {
    SIM_Object *obj =
        box3dObjectById(objects, items[begin].objectId, st.objectsSorted);
    if (!obj)
      return;
    SIM_GeometryCopy *geo = getOrMakeWritableGeometry(*obj);
    if (!geo)
      return; // writeResultsBack warned already
    unsigned want = 0;
    for (size_t k = begin; k < end; ++k)
      want |= items[k].flags;
    unsigned wrote = 0;
    {
      SIM_GeometryAutoWriteLock lock(geo);
      GU_Detail *gdp = &lock.getGdp();
      if (!gdp)
        return;
      if ((want & Box3DPointWrite::kMass) && !gdp->findPointAttribute("mass"))
        gdp->addFloatTuple(GA_ATTRIB_POINT, "mass", 1);
      GA_RWHandleI valid(gdp, GA_ATTRIB_POINT, "bullet_autofit_valid");
      GA_RWHandleF mass(gdp, GA_ATTRIB_POINT, "mass");
      const PointAttrs a(gdp);
      resolvePointOffsets(gdp, a, items, begin, end);
      for (size_t k = begin; k < end; ++k) {
        const Box3DPointWrite &w = items[k];
        if (w.off == GA_INVALID_OFFSET)
          continue;
        if ((w.flags & Box3DPointWrite::kAutofitValid) && valid.isValid()) {
          valid.set(w.off, 1);
          wrote |= Box3DPointWrite::kAutofitValid;
        }
        if ((w.flags & Box3DPointWrite::kMass) && mass.isValid()) {
          mass.set(w.off, w.mass);
          wrote |= Box3DPointWrite::kMass;
        }
      }
      if (wrote & Box3DPointWrite::kAutofitValid)
        valid.getAttribute()->bumpDataId();
      if (wrote & Box3DPointWrite::kMass)
        mass.getAttribute()->bumpDataId();
    }
    if (wrote)
      recordFingerprintAfterWrite(st, *obj);
  });
  items.clear();
}

// Enable Aging: age += dt on every point; dead = 1 once age exceeds a positive
// life (as the POP Solver). With Reap Dead Points and Reap At Frame End,
// points that die are queued in st.deletions, so deleteDeadPoints removes them
// in this same step; otherwise the next step's sync queues them.
void ageObjects(Box3DSharedState &st, SIM_ObjectArray &objects, float dt) {
  for (exint i = 0; i < objects.entries(); ++i) {
    SIM_Object *obj = objects(i);
    if (!SIM_DATA_GETCONST(*obj, SIM_GEOMETRY_DATANAME, SIM_Geometry))
      continue;
    const auto osIt = st.objectSync.find(obj->getObjectId());
    if (osIt != st.objectSync.end() && osIt->second.wholeObject)
      continue; // one body: its points are not particles
    SIM_GeometryCopy *geo = getOrMakeWritableGeometry(*obj);
    if (!geo)
      continue;
    bool markedDead = false;
    {
      SIM_GeometryAutoWriteLock lock(geo);
      GU_Detail *gdp = &lock.getGdp();
      if (!gdp)
        continue;
      if (!gdp->findPointAttribute("age"))
        gdp->addFloatTuple(GA_ATTRIB_POINT, "age", 1);
      const bool hasLife = gdp->findPointAttribute("life") != nullptr;
      if (hasLife && !gdp->findPointAttribute("dead"))
        gdp->addIntTuple(GA_ATTRIB_POINT, "dead", 1);
      GA_RWHandleF age(gdp, GA_ATTRIB_POINT, "age");
      GA_ROHandleF life(gdp, GA_ATTRIB_POINT, "life");
      GA_RWHandleI dead(gdp, GA_ATTRIB_POINT, "dead");
      if (!age.isValid())
        continue;
      const GA_Range range(gdp->getPointRange());
      const bool killing = life.isValid() && dead.isValid();
      // The deletion check matches points by `id`, or by point number without
      // it (as the sync queues them).
      const GA_ROHandleI pointIds(gdp, GA_ATTRIB_POINT, "id");
      std::atomic<bool> anyDead{false};
      std::vector<Box3DDeletion> died; // rare: a lock is cheap enough
      std::mutex diedLock;
      // Each point touches only its own `age` / `dead` slot.
      const auto agePoint = [&](GA_Offset off) {
        const float a = age.get(off) + dt;
        age.set(off, a);
        if (killing) {
          const float l = life.get(off);
          if (l > 0.0f && a > l && dead.get(off) == 0) {
            dead.set(off, 1);
            anyDead.store(true, std::memory_order_relaxed);
            if (!st.reapDead || !st.reapAtFrameEnd)
              return;
            Box3DDeletion d;
            d.objectId = obj->getObjectId();
            d.off = off;
            d.pointId = pointIds.isValid()
                            ? static_cast<int64_t>(pointIds.get(off))
                            : static_cast<int64_t>(gdp->pointIndex(off));
            const std::lock_guard<std::mutex> guard(diedLock);
            died.push_back(d);
          }
        }
      };
      if (static_cast<size_t>(range.getEntries()) < minParallelPoints()) {
        for (GA_Iterator it(range); !it.atEnd(); ++it)
          agePoint(*it);
      } else {
        // Shared copy-on-write pages would race: harden what the parallel pass
        // writes.
        age.getAttribute()->hardenAllPages();
        if (killing)
          dead.getAttribute()->hardenAllPages();
        UTparallelFor(GA_SplittableRange(range),
                      [&](const GA_SplittableRange &r) {
                        GA_Offset start, end;
                        for (GA_Iterator it(r); it.blockAdvance(start, end);)
                          for (GA_Offset off = start; off < end; ++off)
                            agePoint(off);
                      });
      }
      markedDead = anyDead.load();
      st.deletions.insert(st.deletions.end(), died.begin(), died.end());
      age.getAttribute()->bumpDataId();
      if (markedDead)
        dead.getAttribute()->bumpDataId();
    }
    recordFingerprintAfterWrite(st, *obj);
    if (markedDead)
      forceScan(st, obj->getObjectId());
  }
}

// Delete the `dead` points (packed ones with their primitive), each checked to
// still be the same point first.
void deleteDeadPoints(Box3DSharedState &st, SIM_ObjectArray &objects) {
  std::vector<Box3DDeletion> &items = st.deletions;
  if (items.empty())
    return;
  forEachObjectRun(items, [&](size_t begin, size_t end) {
    SIM_Object *obj =
        box3dObjectById(objects, items[begin].objectId, st.objectsSorted);
    if (!obj)
      return;
    SIM_GeometryCopy *geo = getOrMakeWritableGeometry(*obj);
    if (!geo)
      return;
    bool deleted = false;
    {
      SIM_GeometryAutoWriteLock lock(geo);
      GU_Detail *gdp = &lock.getGdp();
      if (!gdp)
        return;
      const GA_ROHandleI id(gdp, GA_ATTRIB_POINT, "id");
      // Check every point first: deleting changes point numbers.
      std::vector<GA_Offset> doomed;
      for (size_t k = begin; k < end; ++k) {
        const GA_Offset off = items[k].off;
        bool ok = off >= 0 && off < gdp->getNumPointOffsets() &&
                  !gdp->getPointMap().isOffsetVacant(off);
        if (ok)
          ok = id.isValid()
                   ? static_cast<int64_t>(id.get(off)) == items[k].pointId
                   : static_cast<int64_t>(gdp->pointIndex(off)) ==
                         items[k].pointId;
        if (ok)
          doomed.push_back(off);
      }
      for (const GA_Offset off : doomed) {
        const GA_Offset vtx = gdp->pointVertex(off);
        GA_Primitive *prim = GAisValid(vtx)
                                 ? gdp->getPrimitive(gdp->vertexPrimitive(vtx))
                                 : nullptr;
        if (prim && GU_PrimPacked::isPackedPrimitive(*prim))
          gdp->destroyPrimitive(*prim, true); // and its point
        else
          gdp->destroyPointOffset(off, GA_Detail::GA_DESTROY_DEGENERATE);
        deleted = true;
      }
      if (deleted)
        gdp->bumpAllDataIds();
    }
    if (deleted) {
      recordFingerprintAfterWrite(st, *obj);
      forceScan(st, obj->getObjectId()); // point numbers changed
    }
  });
  items.clear();
}

namespace {

// ---- writeResultsBack, step 0 (serial): the bodies that moved
// ----------------- Box3D's move events (read into st.moves right after the
// step): every awake body and its transform (a body falling asleep appears once
// more). Each body's user data is its entry.
struct MovedBodies {
  std::vector<int> wholeMoves; // move events of whole-object bodies
  int missing = 0;             // moved bodies without a handle
};

// Fills st.writeItems and st.packedWrites with the point bodies to write.
MovedBodies collectMovedBodies(Box3DSharedState &st) {
  MovedBodies out;
  const int moveCount = static_cast<int>(st.moves.size());
  std::vector<Box3DWriteItem> &items = st.writeItems;
  items.clear();
  items.reserve(static_cast<size_t>(moveCount));
  st.packedWrites.clear();
  for (int mi = 0; mi < moveCount; ++mi) {
    const b3dop::BodyMove &mv = st.moves[static_cast<size_t>(mi)];
    if (!mv.userData)
      continue;
    Box3DBodyMirror *entry = static_cast<Box3DBodyMirror *>(mv.userData);
    if (!entry->dynamic() || !st.isLive(*entry))
      continue; // only dynamic bodies seen by this step's sync are written
    if (!entry->handle) {
      ++out.missing;
      continue;
    }
    if (entry->wholeObject) {
      out.wholeMoves.push_back(mi); // written to the object's Position data
      continue;
    }
    items.emplace_back();
    Box3DWriteItem &w = items.back();
    w.entry = entry;
    w.move = mi;
    w.objectId = entry->objectId;
    if (entry->packed) {
      w.packedIdx = static_cast<int>(st.packedWrites.size());
      st.packedWrites.emplace_back();
    }
    // `awake`: only changes are written (and only if the attribute exists).
    // entry->awake is updated where the write happens, so an unresolved point
    // is retried next step.
    const bool nowAwake = !mv.fellAsleep;
    if (nowAwake != entry->awake)
      w.awakeState = nowAwake ? 1 : 0;
  }
  return out;
}

// Whole-object bodies: one Position data each (serial; few objects). Returns
// how many could not be written.
int writeWholeObjects(Box3DSharedState &st, SIM_ObjectArray &objects,
                      const std::vector<int> &wholeMoves) {
  int notWritten = 0;
  for (const int mi : wholeMoves) {
    const b3dop::BodyMove &mv = st.moves[static_cast<size_t>(mi)];
    Box3DBodyMirror *entry = static_cast<Box3DBodyMirror *>(mv.userData);
    SIM_Object *obj =
        box3dObjectById(objects, entry->objectId, st.objectsSorted);
    float lin[3], ang[3];
    if (!obj || !st.world.getVelocityByHandle(entry->handle, lin, ang))
      continue;
    entry->awake = !mv.fellAsleep;
    if (!writeWholeObject(*obj, *entry, mv.pos, mv.rot, lin, ang))
      ++notWritten;
  }
  return notWritten;
}

// Velocities (not in move events): Box3D allows read-only calls from several
// threads.
void readVelocities(Box3DSharedState &st) {
  std::vector<Box3DWriteItem> &items = st.writeItems;
  parallelForEach(
      items.size(),
      [&](size_t i) {
        Box3DWriteItem &w = items[i];
        st.world.getVelocityByHandle(w.entry->handle, w.lin, w.ang);
      },
      minParallelWrite());
}

// A new quaternion attribute is 0,0,0,0: start every point at identity.
void startOrientAtIdentity(GU_Detail *gdp) {
  GA_RWHandleQ qOut(gdp, GA_ATTRIB_POINT, "orient");
  if (!qOut.isValid())
    return;
  for (GA_Iterator it(gdp->getPointRange()); !it.atEnd(); ++it)
    qOut.set(*it, UT_QuaternionF(0.0f, 0.0f, 0.0f, 1.0f));
}

// ---- step 2 (parallel): write one body's attributes
// ------------------------------ The attribute handles of one object's
// geometry, and what it takes to write one item. write() touches only its own
// point and its own item's entry, so items can be written concurrently.
class BodyWriter {
public:
  // The items [begin, end) of st.writeItems belong to `gdp`; their points are
  // resolved.
  BodyWriter(Box3DSharedState &st, GU_Detail *gdp, const PointAttrs &a,
             size_t begin, size_t end)
      : myMoves(st.moves), myPackedWrites(st.packedWrites), myGdp(gdp), myA(a),
        myAnyPrims(gdp->getNumPrimitives() > 0), myP(gdp, GA_ATTRIB_POINT, "P"),
        myV(gdp, GA_ATTRIB_POINT, "v"), myW(gdp, GA_ATTRIB_POINT, "w"),
        myQ(gdp, GA_ATTRIB_POINT, "orient"),
        myAwake(gdp, GA_ATTRIB_POINT, "awake") // only if it exists
  {
    if (myAwake.isValid())
      for (size_t k = begin; k < end && !myAnyAwakeWrite; ++k)
        myAnyAwakeWrite = st.writeItems[k].awakeState >= 0 &&
                          st.writeItems[k].off != GA_INVALID_OFFSET;
#ifdef BOX3DDOP_DEBUG
    myVerifyReadback = B3D_ENV_INT("BOX3DDOP_VERIFY_READBACK", 0) != 0;
#endif
  }

  // The attributes write() writes (null: not written).
  std::array<GA_Attribute *, 5> writtenAttributes() const {
    return {myP.isValid() ? myP.getAttribute() : nullptr,
            myQ.isValid() ? myQ.getAttribute() : nullptr,
            myV.isValid() ? myV.getAttribute() : nullptr,
            myW.isValid() ? myW.getAttribute() : nullptr,
            myAnyAwakeWrite ? myAwake.getAttribute() : nullptr};
  }

  void write(Box3DWriteItem &w) const {
    if (w.off == GA_INVALID_OFFSET)
      return;
    const b3dop::BodyMove &mv = myMoves[static_cast<size_t>(w.move)];
    writeAttributes(w, mv);
    Box3DPackedWrite *pw = composePacked(w, mv);
    updateMirror(w, mv, pw != nullptr);
#ifdef BOX3DDOP_DEBUG
    if (myVerifyReadback)
      verifyReadback(w, pw);
#endif
  }

private:
  void writeAttributes(const Box3DWriteItem &w,
                       const b3dop::BodyMove &mv) const {
    const GA_Offset off = w.off;
    if (myP.isValid())
      myP.set(off, UT_Vector3(static_cast<float>(mv.pos[0]),
                              static_cast<float>(mv.pos[1]),
                              static_cast<float>(mv.pos[2])));
    if (myQ.isValid())
      myQ.set(off, UT_QuaternionF(mv.rot[0], mv.rot[1], mv.rot[2], mv.rot[3]));
    if (myV.isValid())
      myV.set(off, UT_Vector3(w.lin[0], w.lin[1], w.lin[2]));
    if (myW.isValid())
      myW.set(off, UT_Vector3(w.ang[0], w.ang[1], w.ang[2]));
    if (myAnyAwakeWrite && w.awakeState >= 0) {
      myAwake.set(off, static_cast<int>(w.awakeState));
      w.entry->awake = (w.awakeState != 0);
    }
  }

  // Packed: the new 3x3 (the scale the sync read, the new rotation) is computed
  // here and written in step 3, to the primitive the last sync stored. Null if
  // the body is not packed.
  Box3DPackedWrite *composePacked(const Box3DWriteItem &w,
                                  const b3dop::BodyMove &mv) const {
    Box3DBodyMirror &me = *w.entry;
    Box3DPackedWrite *pw =
        w.packedIdx >= 0 ? &myPackedWrites[static_cast<size_t>(w.packedIdx)]
                         : nullptr;
    if (!(myAnyPrims && me.packed && pw))
      return nullptr;
    composePackedMatrix(mv.rot, me.rowScale, pw->newM3);
    pw->primOff = GA_Offset(me.primOff);
    pw->has = true;
    std::copy(pw->newM3, pw->newM3 + 9,
              me.m3); // a skipped sync does not refresh it
    return pw;
  }

  // What the read path will see, so our own write does not look like an
  // external edit: P through float; the rotation as written (poseDiffers allows
  // for the float noise of a packed transform's round trip and for q vs -q).
  // Non-finite P: Box3D's values.
  void updateMirror(const Box3DWriteItem &w, const b3dop::BodyMove &mv,
                    bool packed) const {
    Box3DBodyPose &mirror = *w.entry;
    const bool hasRot =
        packed || myQ.isValid(); // else the read path sees identity
    const float identity[4] = {0, 0, 0, 1};
    std::copy(hasRot ? mv.rot : identity, (hasRot ? mv.rot : identity) + 4,
              mirror.rot);
    b3dop::sanitizeQuat(mirror.rot);
    for (int i = 0; i < 3; ++i) {
      mirror.lin[i] = myV.isValid() ? w.lin[i] : 0.0f;
      mirror.ang[i] = myW.isValid() ? w.ang[i] : 0.0f;
    }
    if (!b3dop::storedPosition(mv.pos, mirror.pos))
      std::copy(mv.pos, mv.pos + 3, mirror.pos);
  }

#ifdef BOX3DDOP_DEBUG
  // BOX3DDOP_VERIFY_READBACK=1: read back; a difference poseDiffers sees would
  // make the next scan treat our own write as an external edit.
  void verifyReadback(const Box3DWriteItem &w,
                      const Box3DPackedWrite *pw) const {
    const Box3DBodyPose &mirror = *w.entry;
    Box3DBodyPose stored;
    PackedInfo pk;
    if (pw) {
      pk.prim =
          static_cast<const GU_PrimPacked *>(myGdp->getPrimitive(pw->primOff));
      double sc[3];
      splitPackedMatrix(pw->newM3, pk.rot, sc);
    }
    if (readPointState(myGdp, myA, w.off, pk, stored) &&
        poseDiffers(stored, mirror)) {
      B3D_LOG("  READBACK MISMATCH object=%d point=%lld: predicted pos=(%.9g "
              "%.9g %.9g) rot=(%.9g %.9g %.9g %.9g) "
              "read pos=(%.9g %.9g %.9g) rot=(%.9g %.9g %.9g %.9g)",
              w.objectId, (long long)w.entry->pointId, mirror.pos[0],
              mirror.pos[1], mirror.pos[2], mirror.rot[0], mirror.rot[1],
              mirror.rot[2], mirror.rot[3], stored.pos[0], stored.pos[1],
              stored.pos[2], stored.rot[0], stored.rot[1], stored.rot[2],
              stored.rot[3]);
    }
  }
#endif

  const std::vector<b3dop::BodyMove> &myMoves;
  std::vector<Box3DPackedWrite> &myPackedWrites;
  [[maybe_unused]] GU_Detail *myGdp; // read back only by the debug check
  [[maybe_unused]] const PointAttrs &myA;
  const bool myAnyPrims;
  mutable GA_RWHandleV3 myP, myV, myW;
  mutable GA_RWHandleQ myQ;
  mutable GA_RWHandleI myAwake;
  bool myAnyAwakeWrite = false;
#ifdef BOX3DDOP_DEBUG
  bool myVerifyReadback = false;
#endif
};

// ---- step 3 (serial): packed primitive rotations
// --------------------------------- Serial: the HDK does not document
// setLocalTransform as thread-safe. Returns the bodies whose primitive is gone
// (the topology changed without a new id).
int writePackedRotations(GU_Detail *gdp,
                         const std::vector<Box3DWriteItem> &items, size_t begin,
                         size_t end,
                         const std::vector<Box3DPackedWrite> &packedWrites) {
  int missing = 0;
  bool wrotePacked = false;
  for (size_t k = begin; k < end; ++k) {
    const Box3DWriteItem &w = items[k];
    if (w.packedIdx < 0 || !packedWrites[static_cast<size_t>(w.packedIdx)].has)
      continue;
    const Box3DPackedWrite &pw = packedWrites[static_cast<size_t>(w.packedIdx)];
    const GA_Primitive *raw =
        (pw.primOff >= 0 && pw.primOff < gdp->getNumPrimitiveOffsets())
            ? gdp->getPrimitive(pw.primOff)
            : nullptr;
    if (!raw || !GU_PrimPacked::isPackedPrimitive(*raw)) {
      ++missing;
      continue;
    }
    setLocal3(*static_cast<GU_PrimPacked *>(gdp->getPrimitive(pw.primOff)),
              pw.newM3);
    wrotePacked = true;
  }
  // setLocalTransform does not mark the primitive list dirty; the HDK leaves
  // that to us.
  if (wrotePacked)
    gdp->getPrimitiveList().bumpDataId();
  return missing;
}

// One object's results, st.writeItems[begin, end), into its (locked, writable)
// geometry. Returns how many bodies could not be matched to a point or
// primitive.
int writeObjectResults(Box3DSharedState &st, GU_Detail *gdp, size_t begin,
                       size_t end) {
  std::vector<Box3DWriteItem> &items = st.writeItems;

  if (ensureResultAttribs(gdp))
    startOrientAtIdentity(gdp);
  const PointAttrs a(gdp); // after the attributes exist

  // ---- step 1 (serial): find each body's point in the geometry ----
  resolvePointOffsets(gdp, a, items, begin, end);
  int missing = 0;
  for (size_t k = begin; k < end; ++k)
    if (items[k].off == GA_INVALID_OFFSET)
      ++missing;

  // ---- step 2 (parallel): write attributes, one task per body ----
  const BodyWriter writer(st, gdp, a, begin, end);
  const std::array<GA_Attribute *, 5> written = writer.writtenAttributes();
  // Shared copy-on-write pages would race: a parallel pass hardens what it
  // writes first.
  if (end - begin >= minParallelWrite())
    hardenWrittenPages(gdp, items, begin, end, written);
  parallelForEach(
      end - begin, [&](size_t j) { writer.write(items[begin + j]); },
      minParallelWrite());
  for (GA_Attribute *attr : written)
    if (attr)
      attr->bumpDataId();

  return missing +
         writePackedRotations(gdp, items, begin, end, st.packedWrites);
}

} // namespace

} // namespace box3dsolver

using namespace box3dsolver;

void SIM_SolverBox3D::writeResultsBack(SIM_Box3DWorld &worldData,
                                       SIM_ObjectArray &objects) {
  Box3DSharedState *st = worldData.state();
  if (st->moves.empty())
    return;

  MovedBodies moved = collectMovedBodies(*st);
  int missing = moved.missing;
  warnCount(objects, writeWholeObjects(*st, objects, moved.wholeMoves),
            "object(s) have Position data the solver cannot write (not a "
            "Position or Motion); their results were not written back");

  std::vector<Box3DWriteItem> &items = st->writeItems;
  if (items.empty())
    return;
  readVelocities(*st);

  // Grouped by object, so each geometry is locked once.
  std::vector<SIM_Object *> refresh; // objects written to
  forEachObjectRun(items, [&](size_t begin, size_t end) {
    SIM_Object *obj =
        box3dObjectById(objects, items[begin].objectId, st->objectsSorted);
    if (!obj)
      return;

    SIM_GeometryCopy *geo = getOrMakeWritableGeometry(*obj);
    if (!geo) {
      B3D_LOG_VERBOSE("  writeBack: object id=%d Geometry is not writable and "
                      "could not be converted",
                      obj->getObjectId());
      addError(obj, SIM_MESSAGE,
               "Box3D: this object's Geometry is not writable and could not be "
               "converted to a SIM_GeometryCopy; results are not written back",
               UT_ERROR_WARNING);
      return;
    }

    SIM_GeometryAutoWriteLock lock(geo);
    GU_Detail *gdp = &lock.getGdp();
    if (!gdp)
      return;
    refresh.push_back(obj);
    missing += writeObjectResults(*st, gdp, begin, end);
  });

  // Re-record the fingerprints now that every lock is released.
  for (SIM_Object *obj : refresh)
    recordFingerprintAfterWrite(*st, *obj);

  if (missing) {
    B3D_LOG_VERBOSE("  writeBack: %d moved bodies had no readable state or no "
                    "point to write to",
                    missing);
    warnCount(objects, missing,
              "body(ies) could not be matched to a point; their results were "
              "not written back");
  }
}
