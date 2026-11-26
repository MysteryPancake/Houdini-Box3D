// Box3DImpacts.cpp
//
// After the step: Impacts, the contact points of the step recorded as impact
// data on the objects that ask for it (Add Impact Data).
#include "Box3DSolverInternal.h"
#include "dop/SIM_SolverBox3D.h"
#include <GA/GA_SplittableRange.h>
#include <algorithm>
#include <array>
#include <unordered_map>

namespace box3dsolver {

// Add Impact Data: this step's contact points with impulse go into a fresh
// "Impacts" data (SIM_Impacts) on the objects with the option on, only on steps
// with at least one; otherwise the data is removed. Contacts are Box3D's
// impulse events of the step (on while some object wants Impacts), one record
// per manifold point of a contact in the awake set, recorded for both bodies,
// so sleeping bodies cost nothing and each pair is read once. Contacts where
// neither body is awake are not reported, and an asleep dynamic body gets none.
// Normal: of the surface hit, pointing at the body that gets the impact. No
// Feedback data (that is for interaction with other solvers).
bool wantsImpacts(const Box3DSharedState &st) {
  return std::any_of(
      st.objectSync.begin(), st.objectSync.end(), [&](const auto &kv) {
        return kv.second.visited == st.syncStamp && kv.second.addImpact;
      });
}

// Which contact a hit is: its two bodies, its side and its position to about
// a centimetre. Never 0 (an empty slot).
uint64_t impactKey(const Box3DBodyMirror *self, const Box3DBodyMirror *other,
                   b3dop::BodyHandle otherHandle, bool fromOther,
                   const double pos[3]) {
  uint64_t k = b3dop::mixHash(
      b3dop::mixHash(0x9e3779b97f4a7c15ull, reinterpret_cast<uintptr_t>(self)),
      reinterpret_cast<uintptr_t>(other));
  k = b3dop::mixHash(k, static_cast<uint64_t>(otherHandle));
  k = b3dop::mixHash(k, fromOther ? 1u : 0u);
  for (int c = 0; c < 3; ++c)
    k = b3dop::mixHash(k, static_cast<uint64_t>(static_cast<int64_t>(
                              std::floor(pos[c] * 100.0))));
  return k | 1u;
}

// Slot of `key` in the open-addressing table (a power of two long): its own if
// present, else the empty one it would go in.
size_t impactSlot(const std::vector<Box3DImpactSlot> &table, uint64_t key) {
  const size_t mask = table.size() - 1;
  size_t i = static_cast<size_t>(key) & mask;
  while (table[i].key && table[i].key != key)
    i = (i + 1) & mask;
  return i;
}

namespace {

// A compound's body has no entry of its own, so its side of an event is the
// piece whose shape the point is on: the event's user data is pointed at that
// piece's entry, and the record lands on its point like any other body's. Only
// compounds with a piece that asks for Impacts are looked at, and a compound's
// shapes are indexed once, on its first event.
void attributeCompoundEvents(Box3DSharedState &st,
                             std::vector<b3dop::ContactImpulse> &events) {
  struct Lookup {
    const Box3DCompound *compound = nullptr;
    bool indexed = false;
    std::unordered_map<uint64_t, Box3DBodyMirror *> pieceOfShape;
  };
  std::unordered_map<b3dop::BodyHandle, Lookup> compounds;
  for (const auto &c : st.compounds)
    if (std::any_of(c->members.begin(), c->members.end(),
                    [&](const Box3DCompoundMember &m) {
                      return m.entry && m.entry->sync &&
                             m.entry->sync->addImpact;
                    }))
      compounds[c->body].compound = c.get();
  if (compounds.empty())
    return;
  thread_local std::vector<uint64_t> keys;
  const auto pieceOf = [&](b3dop::BodyHandle body,
                           uint64_t shape) -> Box3DBodyMirror * {
    const auto it = compounds.find(body);
    if (it == compounds.end())
      return nullptr;
    Lookup &l = it->second;
    if (!l.indexed) {
      l.indexed = true;
      st.world.shapeKeysByHandle(body, keys);
      for (const Box3DCompoundMember &m : l.compound->members)
        for (int i = m.shapeFirst; i < m.shapeFirst + m.shapeCount; ++i)
          if (static_cast<size_t>(i) < keys.size())
            l.pieceOfShape[keys[static_cast<size_t>(i)]] = m.entry;
    }
    const auto p = l.pieceOfShape.find(shape);
    return p == l.pieceOfShape.end() ? nullptr : p->second;
  };
  for (b3dop::ContactImpulse &ev : events) {
    if (!ev.userDataA)
      ev.userDataA = pieceOf(ev.a, ev.shapeA);
    if (!ev.userDataB)
      ev.userDataB = pieceOf(ev.b, ev.shapeB);
  }
}

// Plan::same for a contact no earlier step of the frame recorded.
constexpr uint32_t kNewHit = ~0u;

// Which of an event's four possible records exist, and the object each belongs
// to (impactPlanRecords). Record r is side r / 2 (the contact as that awake
// body sees it), the body itself when r is even, the other body when odd.
struct ImpactPlan {
  uint8_t mask = 0;
  int objectId[4];
  uint64_t key[4];  // impactKey, when the steps are merged
  uint32_t same[4]; // the earlier step's hit it adds to, or kNewHit
};

// The hits of one object in st scratch (objects are few: linear search). Groups
// are reused, not freed, so the hit vectors keep their capacity.
size_t impactGroupFor(Box3DSharedState &st, int objectId) {
  for (size_t i = 0; i < st.impactGroupCount; ++i)
    if (st.impactGroups[i].objectId == objectId)
      return i;
  if (st.impactGroupCount == st.impactGroups.size())
    st.impactGroups.emplace_back();
  Box3DImpactGroup &g = st.impactGroups[st.impactGroupCount];
  g.objectId = objectId;
  g.hits.clear();
  return st.impactGroupCount++;
}

// Substeps: a contact an earlier step of the frame already recorded (the same
// bodies, side and spot) takes this step's impulse instead of a record of its
// own, so the frame's impulses still add up with one record per contact
// point, not one per step. Only the hits of the earlier steps are looked up
// (the table is built here, before this step adds its own), so two points of
// one step stay two records.
void indexEarlierImpactHits(Box3DSharedState &st) {
  using Hit = Box3DImpactHit;
  std::vector<Box3DImpactSlot> &table = st.impactSlots;
  size_t total = 0;
  for (size_t g = 0; g < st.impactGroupCount; ++g)
    total += st.impactGroups[g].hits.size();
  size_t size = 16;
  while (size < 2 * total)
    size *= 2;
  table.assign(size, Box3DImpactSlot());
  for (size_t g = 0; g < st.impactGroupCount; ++g) {
    const std::vector<Hit> &hits = st.impactGroups[g].hits;
    for (size_t k = 0; k < hits.size(); ++k) {
      const uint64_t key = hits[k].key;
      Box3DImpactSlot &s = table[impactSlot(table, key)];
      if (!s.key) // the first of equal keys keeps the slot
        s = {key, static_cast<uint32_t>(g), static_cast<uint32_t>(k)};
    }
  }
}

// Pass 1 (parallel): which of each event's records exist, and the object each
// belongs to. This is the part that reads the bodies' entries. `merge`: the
// table of the earlier steps' hits is built (indexEarlierImpactHits).
void impactPlanRecords(Box3DSharedState &st,
                       const std::vector<b3dop::ContactImpulse> &events,
                       std::vector<ImpactPlan> &plans, bool merge) {
  b3dop::Box3DWorld &world = st.world;
  const bool speedFilter = world.settings().impactMinSpeed > 0.0f;
  const bool keys = st.impactKeys;
  const std::vector<Box3DImpactSlot> &table = st.impactSlots;
  const size_t n = events.size();
  // The groups that exist before this step adds its hits: where an earlier
  // step's hits are (read-only while this pass runs).
  const size_t oldGroups = st.impactGroupCount;
  const auto oldGroupOf = [&](int objectId) -> size_t {
    for (size_t g = 0; g < oldGroups; ++g)
      if (st.impactGroups[g].objectId == objectId)
        return g;
    return oldGroups;
  };
  // A contact an earlier step recorded (same key, same object): the index of
  // its hit in its group, else kNewHit.
  const auto findSame = [&](uint64_t key, int objectId) -> uint32_t {
    const Box3DImpactSlot &s = table[impactSlot(table, key)];
    return s.key && s.group == oldGroupOf(objectId) ? s.hit : kNewHit;
  };
  plans.resize(n);
  box3dParallelFor(
      n,
      [&](size_t k) {
        const b3dop::ContactImpulse &ev = events[k];
        ImpactPlan &plan = plans[k];
        plan.mask = 0;
        for (int side = 0; side < 2; ++side) {
          const bool first = side == 0;
          const Box3DBodyMirror *e = static_cast<const Box3DBodyMirror *>(
              first ? ev.userDataA : ev.userDataB);
          const Box3DBodyMirror *o = static_cast<const Box3DBodyMirror *>(
              first ? ev.userDataB : ev.userDataA);
          const bool selfAwake = first ? ev.awakeA : ev.awakeB;
          const bool otherAwake = first ? ev.awakeB : ev.awakeA;
          if (!selfAwake || !e || !e->handle || !st.isLive(*e))
            continue;
          const bool selfWants = e->sync->addImpact;
          // The other side, unless it is awake (it records its own) or an
          // asleep dynamic body.
          const bool otherWants = o && st.isLive(*o) && o->sync->addImpact &&
                                  !o->dynamic() && !otherAwake;
          // The bodies of the event (a compound's, not its piece's).
          const b3dop::BodyHandle selfHandle = first ? ev.a : ev.b;
          const b3dop::BodyHandle otherHandle = first ? ev.b : ev.a;
          if ((!selfWants && !otherWants) ||
              (speedFilter && !world.hadHitEvent(selfHandle, otherHandle)))
            continue;
          if (selfWants) {
            plan.mask |= static_cast<uint8_t>(1u << (2 * side));
            plan.objectId[2 * side] = e->objectId;
            plan.key[2 * side] =
                keys ? impactKey(e, o, otherHandle, false, ev.pos) : 0;
            plan.same[2 * side] =
                merge ? findSame(plan.key[2 * side], e->objectId) : kNewHit;
          }
          if (otherWants) {
            plan.mask |= static_cast<uint8_t>(2u << (2 * side));
            plan.objectId[2 * side + 1] = o->objectId;
            plan.key[2 * side + 1] =
                keys ? impactKey(o, e, otherHandle, true, ev.pos) : 0;
            plan.same[2 * side + 1] =
                merge ? findSame(plan.key[2 * side + 1], o->objectId) : kNewHit;
          }
        }
      },
      minParallelWrite(), kCheapGrain);
}

// Pass 2 (serial): the hits, in event order, so the records do not depend on
// the threads.
void impactAddHits(Box3DSharedState &st,
                   const std::vector<b3dop::ContactImpulse> &events,
                   const std::vector<ImpactPlan> &plans) {
  using Hit = Box3DImpactHit;
  const size_t n = events.size();
  for (size_t k = 0; k < n; ++k) {
    const ImpactPlan &plan = plans[k];
    if (!plan.mask)
      continue;
    const b3dop::ContactImpulse &ev = events[k];
    for (int r = 0; r < 4; ++r) {
      if (!(plan.mask & (1u << r)))
        continue;
      const bool first = (r >> 1) == 0;
      const Box3DBodyMirror *e = static_cast<const Box3DBodyMirror *>(
          first ? ev.userDataA : ev.userDataB);
      const Box3DBodyMirror *o = static_cast<const Box3DBodyMirror *>(
          first ? ev.userDataB : ev.userDataA);
      b3dop::ContactPoint cp;
      cp.other = first ? ev.b : ev.a;
      b3dop::copy3(cp.pos, ev.pos);
      const float sgn = first ? 1.0f : -1.0f;
      for (int c = 0; c < 3; ++c)
        cp.normal[c] = sgn * ev.normal[c];
      cp.impulse = ev.impulse;
      const size_t group = impactGroupFor(st, plan.objectId[r]);
      if (plan.same[r] != kNewHit) {
        st.impactGroups[group].hits[plan.same[r]].cp.impulse += cp.impulse;
        continue;
      }
      if (!(r & 1))
        st.impactGroups[group].hits.push_back({e, o, cp, false, plan.key[r]});
      else
        st.impactGroups[group].hits.push_back({o, e, cp, true, plan.key[r]});
    }
  }
}

} // namespace

void collectImpacts(Box3DSharedState &st, bool append) {
  st.impactsCollected = true;
  if (!append)
    st.impactGroupCount = 0;
  const bool merge = append && st.impactGroupCount > 0;
  if (merge)
    indexEarlierImpactHits(st);

  if (!wantsImpacts(st))
    return;
  b3dop::Box3DWorld &world = st.world;
  if (world.settings().impactMinSpeed > 0.0f && !st.hitsCollected)
    world.updateHitEvents();
  world.getContactImpulseEvents(st.impulseEvents);
  attributeCompoundEvents(st, st.impulseEvents);
  thread_local std::vector<ImpactPlan> planStore; // kept between steps
  impactPlanRecords(st, st.impulseEvents, planStore, merge);
  impactAddHits(st, st.impulseEvents, planStore);
}

void recordImpacts(Box3DSharedState &st, SIM_ObjectArray &objects,
                   const SIM_Time &now) {
  using Hit = Box3DImpactHit;
  // One step: the contacts of this step's move events. Substeps: the loop
  // collected them after every step.
  if (!st.impactsCollected)
    collectImpacts(st, false);

  std::vector<int> wrote;
  for (size_t gi = 0; gi < st.impactGroupCount; ++gi) {
    const Box3DImpactGroup &grp = st.impactGroups[gi];
    SIM_Object *obj = box3dObjectById(objects, grp.objectId, st.objectsSorted);
    if (!obj)
      continue;
    SIM_Impacts *imp =
        SIM_DATA_CREATE(*obj, SIM_IMPACTS_DATANAME, SIM_Impacts, 0);
    if (!imp)
      continue;
    wrote.push_back(grp.objectId);
    for (const Hit &h : grp.hits) {
      // cp.normal points from the body it was read from into the other one.
      const b3dop::ContactPoint &cp = h.cp;
      const float sgn = h.fromOther ? 1.0f : -1.0f;
      imp->addPointImpact(box3dVec3F(cp.pos),
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

} // namespace box3dsolver
