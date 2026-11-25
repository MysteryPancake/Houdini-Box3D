// Box3DWorldContacts.cpp
//
// Box3DWorld's contacts and events: the impulse a body's contacts carried, the
// contact points and supports, impulse, hit and move events, and which bodies a
// fast mover is about to reach.
#include "Box3DDebug.h"
#include "Box3DShapes.h"
#include "Box3DWorldInternal.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <mutex>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace b3dop {

namespace {
// Scratch for a body's shapes (reused, so a call does not allocate).
thread_local std::vector<b3ShapeId> tShapes;
} // namespace

namespace {

// Shape id -> creation index of the first `got` of `shapes` (newest first),
// sorted by id and searched with findShapeIndex. (A hash map cleared for every
// call costs its bucket count, which a large compound left behind.)
using ShapeIndex = std::vector<std::pair<uint64_t, int>>;

void indexShapes(const std::vector<b3ShapeId> &shapes, int got,
                 ShapeIndex &out) {
  out.clear();
  for (int i = 0; i < got; ++i)
    out.emplace_back(b3StoreShapeId(shapes[static_cast<size_t>(i)]),
                     got - 1 - i);
  std::sort(out.begin(), out.end());
}

// The entry of `key` in `index`, or null.
const std::pair<uint64_t, int> *findShapeIndex(const ShapeIndex &index,
                                               uint64_t key) {
  const auto it = std::lower_bound(index.begin(), index.end(), key,
                                   [](const std::pair<uint64_t, int> &e,
                                      uint64_t k) { return e.first < k; });
  return it == index.end() || it->first != key ? nullptr : &*it;
}

void toBodyMove(const b3BodyMoveEvent &e, BodyMove &out) {
  out.userData = e.userData;
  out.handle = b3StoreBodyId(e.bodyId);
  storePos(e.transform.p, out.pos);
  storeQuat(e.transform.q, out.rot);
  storeVec3(e.linearVelocity, out.lin);
  storeVec3(e.angularVelocity, out.ang);
  out.fellAsleep = e.fellAsleep;
}

bool sameBody(const b3BodyId &a, const b3BodyId &b) {
  return a.index1 == b.index1 && a.world0 == b.world0 &&
         a.generation == b.generation;
}

// The body's contacts into `data` (a scratch buffer that only grows, so a call
// does not zero what it is about to overwrite); how many.
int fetchContacts(b3BodyId id, std::vector<b3ContactData> &data) {
  const int cap = b3Body_GetContactCapacity(id);
  if (cap <= 0)
    return 0;
  if (data.size() < static_cast<size_t>(cap))
    data.resize(static_cast<size_t>(cap));
  return b3Body_GetContactData(id, data.data(), cap);
}

} // namespace

float Box3DWorld::contactImpulseByHandle(
    BodyHandle h, const BodyHandle *skip, size_t skipCount,
    const std::function<bool(BodyHandle)> *ignore) const {
  if (!h)
    return 0.0f;
  const b3BodyId id = b3LoadBodyId(h);
  thread_local std::vector<b3ContactData> data;
  const int n = fetchContacts(id, data);
  float total = 0.0f;
  for (int i = 0; i < n; ++i) {
    const b3ContactData &cd = data[static_cast<size_t>(i)];
    if (skipCount > 0 || ignore) {
      const b3BodyId a = b3Shape_GetBody(cd.shapeIdA);
      const b3BodyId other = sameBody(a, id) ? b3Shape_GetBody(cd.shapeIdB) : a;
      const BodyHandle otherHandle = b3StoreBodyId(other);
      if (std::find(skip, skip + skipCount, otherHandle) != skip + skipCount)
        continue;
      if (ignore && (*ignore)(otherHandle))
        continue;
    }
    for (int m = 0; m < cd.manifoldCount; ++m) {
      const b3Manifold &man = cd.manifolds[m];
      for (int k = 0; k < man.pointCount; ++k)
        if (man.points[k].totalNormalImpulse > 0.0f)
          total += man.points[k].totalNormalImpulse;
    }
  }
  return total;
}

namespace {

struct SweptBodies {
  std::vector<b3BodyId> ids;
  b3BodyId self;
};

bool collectSweptBody(b3ShapeId shape, void *context) {
  SweptBodies &c = *static_cast<SweptBodies *>(context);
  const b3BodyId body = b3Shape_GetBody(shape);
  if (sameBody(body, c.self))
    return true;
  // One entry per shape: the caller sorts and drops the repeats.
  c.ids.push_back(body);
  return true;
}

} // namespace

void Box3DWorld::approachingBodies(const BodyHandle *movers, size_t count,
                                   float dt, float minSpeed, float minMomentum,
                                   std::vector<Approach> &out) const {
  if (!isValid() || !(dt > 0.0f))
    return;
  SweptBodies hit;
  for (size_t i = 0; i < count; ++i) {
    if (!movers[i])
      continue;
    const b3BodyId id = b3LoadBodyId(movers[i]);
    if (!b3Body_IsValid(id) || b3Body_GetType(id) == b3_staticBody ||
        !b3Body_IsAwake(id) || b3Body_GetShapeCount(id) == 0)
      continue;
    const b3Vec3 v = b3Body_GetLinearVelocity(id);
    if (!b3IsValidVec3(v) || b3Length(v) < minSpeed)
      continue;
    const bool dynamic = b3Body_GetType(id) == b3_dynamicBody;
    const float mass = dynamic ? b3Body_GetMass(id) : 0.0f;
    if (dynamic && !(mass * b3Length(v) >= minMomentum))
      continue;
    b3AABB box = b3Body_ComputeAABB(id);
    const b3Vec3 size = b3Sub(box.upperBound, box.lowerBound);
    const float half = 0.5f * std::max(size.x, std::max(size.y, size.z));
    const b3Vec3 d = b3MulSV(dt, v);
    box.lowerBound = b3Add(box.lowerBound, b3Min(d, b3Vec3_zero));
    box.upperBound = b3Add(box.upperBound, b3Max(d, b3Vec3_zero));
    hit.ids.clear();
    hit.self = id;
    b3World_OverlapAABB(myImpl->worldId, box, b3DefaultQueryFilter(),
                        collectSweptBody, &hit);
    std::sort(hit.ids.begin(), hit.ids.end(),
              [](const b3BodyId &x, const b3BodyId &y) {
                return b3StoreBodyId(x) < b3StoreBodyId(y);
              });
    hit.ids.erase(std::unique(hit.ids.begin(), hit.ids.end(), sameBody),
                  hit.ids.end());
    const b3Pos centre = b3Body_GetWorldCenter(id);
    for (const b3BodyId &target : hit.ids) {
      // The mover's velocity relative to what the target does at the mover's
      // centre: a body that moves with the target (a carrier, a platform) is
      // not coming at it.
      const b3Vec3 r = b3SubPos(centre, b3Body_GetWorldCenter(target));
      const b3Vec3 atMover =
          b3Add(b3Body_GetLinearVelocity(target),
                b3Cross(b3Body_GetAngularVelocity(target), r));
      const b3Vec3 rel = b3Sub(v, atMover);
      if (!b3IsValidVec3(rel) || b3Length(rel) < minSpeed)
        continue;
      Approach a;
      a.mover = movers[i];
      a.target = b3StoreBodyId(target);
      a.moverMass = mass;
      storePos(centre, a.moverCom);
      storeVec3(v, a.moverLin);
      storeVec3(rel, a.relLin);
      a.moverHalf = half;
      out.push_back(a);
    }
  }
}

int Box3DWorld::contactImpulsesByShape(
    BodyHandle h, std::vector<float> &out,
    const std::function<bool(BodyHandle)> *ignore) const {
  out.clear();
  if (!h)
    return 0;
  const b3BodyId id = b3LoadBodyId(h);
  thread_local std::vector<b3ShapeId> shapes;
  const int got = bodyShapes(id, shapes);
  if (got <= 0)
    return 0;
  out.assign(static_cast<size_t>(got), 0.0f);
  thread_local std::vector<b3ContactData> data;
  const int n = fetchContacts(id, data);
  if (n <= 0)
    return got; // no contacts: no need to index the shapes
  thread_local ShapeIndex indexOf;
  indexShapes(shapes, got, indexOf);
  for (int i = 0; i < n; ++i) {
    const b3ContactData &cd = data[static_cast<size_t>(i)];
    const bool aIsMine = sameBody(b3Shape_GetBody(cd.shapeIdA), id);
    const b3ShapeId mine = aIsMine ? cd.shapeIdA : cd.shapeIdB;
    const uint64_t mineKey = b3StoreShapeId(mine);
    const auto *it = findShapeIndex(indexOf, mineKey);
    if (!it)
      continue;
    if (ignore && (*ignore)(b3StoreBodyId(
                      b3Shape_GetBody(aIsMine ? cd.shapeIdB : cd.shapeIdA))))
      continue;
    float total = 0.0f;
    for (int m = 0; m < cd.manifoldCount; ++m) {
      const b3Manifold &man = cd.manifolds[m];
      for (int k = 0; k < man.pointCount; ++k)
        if (man.points[k].totalNormalImpulse > 0.0f)
          total += man.points[k].totalNormalImpulse;
    }
    out[static_cast<size_t>(it->second)] += total;
  }
  return got;
}

int Box3DWorld::getContactsByHandle(BodyHandle h,
                                    std::vector<ContactPoint> &out) const {
  out.clear();
  if (!h)
    return 0;
  const b3BodyId id = b3LoadBodyId(h);
  thread_local std::vector<b3ContactData> data;
  const int n = fetchContacts(id, data);
  for (int i = 0; i < n; ++i) {
    const b3ContactData &cd = data[static_cast<size_t>(i)];
    const b3BodyId a = b3Shape_GetBody(cd.shapeIdA);
    const b3BodyId b = b3Shape_GetBody(cd.shapeIdB);
    const bool thisIsA = sameBody(a, id);
    const b3BodyId other = thisIsA ? b : a;
    const b3Pos centerA =
        b3Body_GetWorldCenter(a); // anchors are relative to A's centre of mass
    for (int m = 0; m < cd.manifoldCount; ++m) {
      const b3Manifold &man = cd.manifolds[m];
      for (int k = 0; k < man.pointCount; ++k) {
        const b3ManifoldPoint &mp = man.points[k];
        if (!(mp.totalNormalImpulse > 0.0f))
          continue;
        ContactPoint cp;
        cp.other = b3StoreBodyId(other);
        cp.pos[0] =
            static_cast<double>(centerA.x) + static_cast<double>(mp.anchorA.x);
        cp.pos[1] =
            static_cast<double>(centerA.y) + static_cast<double>(mp.anchorA.y);
        cp.pos[2] =
            static_cast<double>(centerA.z) + static_cast<double>(mp.anchorA.z);
        const float sgn =
            thisIsA ? 1.0f : -1.0f; // the manifold normal points from A to B
        cp.normal[0] = sgn * man.normal.x;
        cp.normal[1] = sgn * man.normal.y;
        cp.normal[2] = sgn * man.normal.z;
        cp.impulse = mp.totalNormalImpulse;
        out.push_back(cp);
      }
    }
  }
  return static_cast<int>(out.size());
}

void Box3DWorld::enableContactImpulseEvents(bool on) {
  if (isValid())
    b3World_EnableContactImpulseEvents(myImpl->worldId, on);
}

int Box3DWorld::getContactImpulseEvents(
    std::vector<ContactImpulse> &out) const {
  if (!isValid()) {
    out.clear();
    return 0;
  }
  const b3ContactEvents ev = b3World_GetContactEvents(myImpl->worldId);
  // Every element is overwritten below, so a vector of the right size is not
  // cleared first (it would be written twice).
  out.resize(static_cast<size_t>(ev.impulseCount));
  for (int i = 0; i < ev.impulseCount; ++i) {
    const b3ContactImpulseEvent &e = ev.impulseEvents[i];
    ContactImpulse &c = out[static_cast<size_t>(i)];
    c.a = b3StoreBodyId(e.bodyIdA);
    c.b = b3StoreBodyId(e.bodyIdB);
    c.userDataA = e.userDataA;
    c.userDataB = e.userDataB;
    storePos(e.point, c.pos);
    storeVec3(e.normal, c.normal);
    c.impulse = e.impulse;
    c.awakeA = e.awakeA;
    c.awakeB = e.awakeB;
  }
  return ev.impulseCount;
}

int Box3DWorld::getContactImpulseSums(
    std::vector<ContactImpulseSum> &out) const {
  out.clear();
  if (!isValid())
    return 0;
  const b3ContactEvents ev = b3World_GetContactEvents(myImpl->worldId);
  out.reserve(static_cast<size_t>(ev.impulseCount) / 2 + 1);
  for (int i = 0; i < ev.impulseCount; ++i) {
    const b3ContactImpulseEvent &e = ev.impulseEvents[i];
    if (!(e.impulse > 0.0f))
      continue;
    const BodyHandle a = b3StoreBodyId(e.bodyIdA);
    const BodyHandle b = b3StoreBodyId(e.bodyIdB);
    if (!out.empty() && out.back().a == a && out.back().b == b)
      out.back().impulse += e.impulse;
    else
      out.push_back({a, b, e.impulse});
  }
  return static_cast<int>(out.size());
}

void Box3DWorld::updateHitEvents(bool append) {
  if (!append)
    myImpl->hitPairs.clear();
  if (!isValid())
    return;
  const b3ContactEvents ev = b3World_GetContactEvents(myImpl->worldId);
  for (int i = 0; i < ev.hitCount; ++i) {
    const BodyHandle a =
        b3StoreBodyId(b3Shape_GetBody(ev.hitEvents[i].shapeIdA));
    const BodyHandle b =
        b3StoreBodyId(b3Shape_GetBody(ev.hitEvents[i].shapeIdB));
    myImpl->hitPairs.emplace_back(std::min(a, b), std::max(a, b));
  }
  // Sorted once per step (the vector keeps its capacity), so a lookup is a
  // binary search.
  std::sort(myImpl->hitPairs.begin(), myImpl->hitPairs.end());
  myImpl->hitPairs.erase(
      std::unique(myImpl->hitPairs.begin(), myImpl->hitPairs.end()),
      myImpl->hitPairs.end());
}

bool Box3DWorld::hadHitEvent(BodyHandle a, BodyHandle b) const {
  return std::binary_search(myImpl->hitPairs.begin(), myImpl->hitPairs.end(),
                            std::make_pair(std::min(a, b), std::max(a, b)));
}

int Box3DWorld::getMoveEvents(std::vector<BodyMove> &out) const {
  if (!isValid()) {
    out.clear();
    return 0;
  }
  const b3BodyEvents ev = b3World_GetBodyEvents(myImpl->worldId);
  // Every element is overwritten, so only resize (clear + resize would zero it
  // every step).
  if (out.size() != static_cast<size_t>(ev.moveCount))
    out.resize(static_cast<size_t>(ev.moveCount));
  for (int i = 0; i < ev.moveCount; ++i)
    toBodyMove(ev.moveEvents[i], out[static_cast<size_t>(i)]);
  return ev.moveCount;
}

} // namespace b3dop
