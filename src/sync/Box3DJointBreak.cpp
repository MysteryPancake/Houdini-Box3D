// Box3DJointBreak.cpp
//
// Constraint networks after the step: Glue's impulse model, joints that break
// (or switch to their next constraint) and the live frames and feedback
// attributes written onto the networks. Box3DJointSync.cpp reads the networks
// and makes the joints.
#include "Box3DHoudiniUtil.h"
#include "Box3DJointNetwork.h"
#include "Box3DJointSync.h"
#include "core/Box3DDebug.h"
#include "dop/SIM_Box3DWorld.h"

#include <GA/GA_Handle.h>
#include <GA/GA_Primitive.h>
#include <GU/GU_Detail.h>
#include <SIM/SIM_Geometry.h>
#include <SIM/SIM_GeometryCopy.h>
#include <SIM/SIM_Object.h>
#include <SIM/SIM_ObjectArray.h>
#include <UT/UT_Quaternion.h>
#include <UT/UT_StringHolder.h>
#include <UT/UT_Vector3.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <deque>
#include <string>
#include <unordered_map>
#include <vector>

namespace {

// After the step, for a written network: every live joint's anchors get
// current_P / current_orient set to where its frames are now (P / orient stay
// as authored), and joints made from P / orient save
// their local frames to local_P / local_orient (relative to their pieces: exact
// on a rebuild, wherever the pieces are). A distance joint made with a negative
// rest length gets its starting distance written to `auto_restlength` (not
// `restlength`, which multiplies the node's value), so a world rebuilt
// mid-simulation keeps it.
template <class PrimOk>
void writeLiveFrames(const b3dop::Box3DWorld &world, GU_Detail *gdp,
                     const std::vector<Box3DJointEntry *> &entries,
                     const PrimOk &primOk) {
  if (entries.empty())
    return;
  auto ensure = [&](const char *name, int size, bool quat, bool &created) {
    created = false;
    if (!gdp->findPointAttribute(name)) {
      gdp->addFloatTuple(GA_ATTRIB_POINT, name, size);
      created = true;
      if (quat)
        if (GA_Attribute *attr = gdp->findPointAttribute(name))
          attr->setTypeInfo(GA_TYPE_QUATERNION);
    }
  };
  // A new current_orient is 0,0,0,0 on every point: "not written" for the
  // points of joints that are not live (see NetworkReader::live).
  bool created = false;
  ensure(kCurrentP, 3, false, created);
  ensure(kCurrentOrient, 4, true, created);
  GA_RWHandleV3 pH(gdp, GA_ATTRIB_POINT, kCurrentP);
  GA_RWHandleQ orientH(gdp, GA_ATTRIB_POINT, kCurrentOrient);
  if (!pH.isValid() || !orientH.isValid())
    return;
  bool wroteRestLength = false, wroteLocal = false;
  GA_RWHandleF restLengthH;
  GA_RWHandleV3 localPH;
  GA_RWHandleQ localQH;

  for (Box3DJointEntry *e : entries) {
    if (e->quiet || !primOk(e->prim))
      continue;
    const GA_Primitive *prim = gdp->getPrimitive(e->prim);
    if (!prim || prim->getVertexCount() != 2)
      continue;
    const GA_Offset pts[2] = {prim->getPointOffset(0), prim->getPointOffset(1)};
    if (e->madeRestLength >= 0.0f) {
      if (!restLengthH.isValid()) {
        if (!gdp->findPrimitiveAttribute(kAutoRestLength))
          gdp->addFloatTuple(GA_ATTRIB_PRIMITIVE, kAutoRestLength, 1,
                             GA_Defaults(-1.0f));
        restLengthH = GA_RWHandleF(gdp, GA_ATTRIB_PRIMITIVE, kAutoRestLength);
      }
      if (restLengthH.isValid()) {
        restLengthH.set(e->prim, e->madeRestLength);
        wroteRestLength = true;
      }
      e->madeRestLength = -1.0f;
    }

    if (e->writeLocal) {
      b3dop::JointFrame la, lb;
      if (world.jointLocalFrames(e->handle, la, lb)) {
        if (!localPH.isValid()) {
          ensure("local_P", 3, false, created);
          ensure("local_orient", 4, true, created);
          localPH = GA_RWHandleV3(gdp, GA_ATTRIB_POINT, "local_P");
          localQH = GA_RWHandleQ(gdp, GA_ATTRIB_POINT, "local_orient");
        }
        if (localPH.isValid() && localQH.isValid()) {
          const b3dop::JointFrame *frames[2] = {&la, &lb};
          for (int k = 0; k < 2; ++k) {
            const b3dop::JointFrame &f = *frames[k];
            localPH.set(pts[k], box3dVec3F(f.pos));
            localQH.set(pts[k],
                        UT_QuaternionF(f.rot[0], f.rot[1], f.rot[2], f.rot[3]));
          }
          wroteLocal = true;
        }
      }
      e->writeLocal = false;
    }

    b3dop::JointFrame fa, fb;
    if (world.jointWorldFrames(e->handle, fa, fb)) {
      // The anchors, not the frames (SideFX data may offset the frames from
      // them).
      e->wroteA = asStored(anchorFrameOf(fa, e->adjA));
      e->wroteB = asStored(anchorFrameOf(fb, e->adjB));
      e->wrote = true;
    } else if (!e->switchFrames)
      continue;              // broken, or never made
    e->switchFrames = false; // (switched: wroteA / wroteB are set already)
    const b3dop::JointFrame *frames[2] = {&e->wroteA, &e->wroteB};
    for (int k = 0; k < 2; ++k) {
      const b3dop::JointFrame &f = *frames[k];
      pH.set(pts[k], box3dVec3F(f.pos));
      orientH.set(pts[k],
                  UT_QuaternionF(f.rot[0], f.rot[1], f.rot[2], f.rot[3]));
    }
  }
  pH.getAttribute()->bumpDataId();
  orientH.getAttribute()->bumpDataId();
  if (wroteRestLength)
    restLengthH.getAttribute()->bumpDataId();
  if (wroteLocal) {
    localPH.getAttribute()->bumpDataId();
    localQH.getAttribute()->bumpDataId();
  }
}

} // namespace

void collectGlueHits(Box3DSharedState &st) {
  b3dop::Box3DWorld &world = st.world;
  for (Box3DGlueNode &n : st.glueNodes) {
    // Hits land on moving bodies; a sleeping one has none this step.
    if (world.typeByHandle(n.body) != b3dop::BodySettings::Type::Dynamic ||
        !world.isAwakeByHandle(n.body))
      continue;
    // A glued neighbour is not a hit (the pair never collides anyway).
    n.hits += world.contactImpulseByHandle(n.body, n.partners.data(),
                                           n.partners.size());
  }
}

void advanceGlue(Box3DSharedState &st, float dt) {
  if (st.glueBonds.empty())
    return;
  std::vector<float> hits(st.glueNodes.size());
  for (size_t i = 0; i < hits.size(); ++i)
    hits[i] = st.glueNodes[i].hits;
  std::vector<b3dop::GlueBond> bonds;
  bonds.reserve(st.glueBonds.size());
  for (size_t i = 0; i < st.glueBonds.size(); ++i) {
    const Box3DJointEntry *e = st.glueBonds[i];
    b3dop::GlueBond g;
    g.a = st.glueBondNodes[i].first;
    g.b = st.glueBondNodes[i].second;
    g.params = e->extra->glueParams;
    g.impact = e->extra->glueImpact;
    bonds.push_back(g);
  }
  std::vector<int> broken;
  b3dop::glueAdvance(bonds, std::move(hits), dt, broken);
  for (size_t i = 0; i < bonds.size(); ++i)
    st.glueBonds[i]->extra->glueImpact = bonds[i].impact;
  for (const int i : broken)
    st.glueBroken.push_back(st.glueBonds[static_cast<size_t>(i)]);
  for (Box3DGlueNode &n : st.glueNodes)
    n.hits = 0.0f;
}

namespace {

using EntriesByNetwork =
    std::unordered_map<uint64_t, std::vector<Box3DJointEntry *>>;

// Joints over their force / torque threshold: read their local frames (a next
// constraint may start from them), then destroy them. Read the events before
// destroying anything.
int breakJoints(Box3DSharedState &st, EntriesByNetwork &brokeIn) {
  b3dop::Box3DWorld &world = st.world;
  std::vector<void *> events;
  if (st.brokenCollected)
    events = st.brokenThisFrame;
  else
    world.brokenJoints(events);
  // Glue bonds over their strength (advanceGlue).
  events.insert(events.end(), st.glueBroken.begin(), st.glueBroken.end());
  st.glueBroken.clear();
  if (!events.empty())
    st.glueDirty = true; // the Glue graph loses a bond
  int broke = 0;
  for (void *p : events) {
    Box3DJointEntry *e = static_cast<Box3DJointEntry *>(p);
    if (!e || !e->handle)
      continue;
    Box3DJointExtra &x = e->ensureExtra();
    e->brokeLocalKnown =
        world.jointLocalFrames(e->handle, x.brokeLocalA, x.brokeLocalB);
    world.destroyJoint(e->handle);
    e->handle = 0;
    e->broken = true;
    brokeIn[e->network].push_back(e);
    ++broke;
  }
  return broke;
}

// Whether a primitive offset still names a primitive of the (written) geometry.
struct PrimExists {
  const GU_Detail *gdp;
  bool operator()(GA_Offset off) const {
    return off >= 0 && off < gdp->getNumPrimitiveOffsets() &&
           gdp->getPrimitive(off) != nullptr;
  }
};

// A broken joint with a next_constraint_name switches to it: its joint is
// remade as that constraint at the next step, from where the pieces are now,
// and the attribute is cleared. The anchors go where the broken joint's frames
// are now (writeLiveFrames).
void switchToNextConstraint(const b3dop::Box3DWorld &world,
                            Box3DJointEntry &e) {
  e.broken = false;
  b3dop::JointFrame fa, fb;
  if (e.brokeLocalKnown && e.extra &&
      world.localToWorld(e.bodyA, e.extra->brokeLocalA, fa) &&
      world.localToWorld(e.bodyB, e.extra->brokeLocalB, fb)) {
    e.wroteA = asStored(anchorFrameOf(fa, e.adjA));
    e.wroteB = asStored(anchorFrameOf(fb, e.adjB));
    e.wrote = true;
    e.switchFrames = true;
  }
}

// The joints that broke: a primitive with a next_constraint_name switches to
// it, the rest go into the `broken` group. True if any switched.
bool writeBrokenJoints(const b3dop::Box3DWorld &world, GU_Detail *gdp,
                       const PrimExists &primOk,
                       const std::vector<Box3DJointEntry *> &broken) {
  GA_RWHandleS conNameH(gdp, GA_ATTRIB_PRIMITIVE, kConstraintName);
  GA_RWHandleS nextH(gdp, GA_ATTRIB_PRIMITIVE, kNextConstraintName);
  GA_PrimitiveGroup *grp = nullptr;
  bool switched = false;
  for (Box3DJointEntry *e : broken) {
    if (!primOk(e->prim))
      continue;
    UT_StringHolder nextName;
    if (nextH.isValid() && conNameH.isValid())
      nextName = nextH.get(e->prim);
    if (nextName.isstring()) {
      conNameH.set(e->prim, nextName);
      nextH.set(e->prim, UT_StringHolder());
      switchToNextConstraint(world, *e);
      switched = true;
      continue;
    }
    if (!grp) {
      grp = gdp->findPrimitiveGroup(kBrokenGroup);
      if (!grp)
        grp = gdp->newPrimitiveGroup(kBrokenGroup);
    }
    if (grp)
      grp->addOffset(e->prim);
  }
  if (grp)
    grp->bumpDataId();
  if (switched) {
    conNameH.getAttribute()->bumpDataId();
    nextH.getAttribute()->bumpDataId();
  }
  return switched;
}

// The bodies' relative rotation since rest, in radians (Bullet's `angle`): 0
// while the joint holds them as they started, however they were rotated then.
// Box3D's own reading compares the bodies' absolute rotations instead.
float angleSinceRest(const b3dop::Box3DWorld &world, const Box3DJointEntry &e,
                     float fallback) {
  float now[2][4] = {{0, 0, 0, 1}, {0, 0, 0, 1}};
  const b3dop::BodyHandle bodies[2] = {e.bodyA, e.bodyB};
  for (int i = 0; i < 2; ++i) {
    double pos[3];
    if (bodies[i] && !world.getTransformByHandle(bodies[i], pos, now[i]))
      return fallback;
  }
  float sinceA[4], sinceB[4], rel[4];
  box3dAbsoluteToRest(e.restRotA, now[0], sinceA);
  box3dAbsoluteToRest(e.restRotB, now[1], sinceB);
  box3dAbsoluteToRest(sinceA, sinceB, rel);
  const float w = std::min(1.0f, std::fabs(rel[3]));
  return 2.0f * std::acos(w);
}

// Feedback, under the Constraint Network DOP's names: force / distance for the
// linear part, torque / angle (radians) for the angular part, plus the per-type
// readings.
void writeFeedback(const b3dop::Box3DWorld &world, GU_Detail *gdp,
                   const PrimExists &primOk,
                   const std::vector<Box3DJointEntry *> &entries) {
  // The handles by name (a handful, all string literals): a linear search with
  // no string built per lookup. A deque, so the references stay valid.
  std::deque<std::pair<const char *, GA_RWHandleF>> scalars;
  auto scalar = [&](const char *name) -> GA_RWHandleF & {
    for (auto &s : scalars)
      if (s.first == name || std::strcmp(s.first, name) == 0)
        return s.second;
    if (!gdp->findPrimitiveAttribute(name))
      gdp->addFloatTuple(GA_ATTRIB_PRIMITIVE, name, 1);
    scalars.emplace_back(name, GA_RWHandleF(gdp, GA_ATTRIB_PRIMITIVE, name));
    return scalars.back().second;
  };
  auto length = [](const float *v) {
    return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
  };
  GA_RWHandleF &forceH = scalar("force");
  GA_RWHandleF &distH = scalar("distance");
  GA_RWHandleF &torqueH = scalar("torque");
  GA_RWHandleF &angleH = scalar("angle");
  for (const Box3DJointEntry *e : entries) {
    if (e->quiet)
      continue;
    if (e->glue && e->extra && primOk(e->prim)) {
      GA_RWHandleF &impactH = scalar(kImpact);
      if (impactH.isValid())
        impactH.set(e->prim, e->extra->glueImpact);
    }
    b3dop::JointFeedback fb;
    if (!primOk(e->prim) || !world.jointFeedback(e->handle, e->type, fb))
      continue;
    if (forceH.isValid())
      forceH.set(e->prim, length(fb.force));
    if (distH.isValid())
      distH.set(e->prim, fb.linearSeparation);
    if (torqueH.isValid())
      torqueH.set(e->prim, length(fb.torque));
    if (angleH.isValid())
      angleH.set(e->prim, angleSinceRest(world, *e, fb.angularSeparation));
    for (int k = 0; k < fb.extraCount; ++k) {
      GA_RWHandleF &h = scalar(fb.extraName[k]);
      if (h.isValid())
        h.set(e->prim, fb.extraIsAngle[k] ? fb.extra[k] * b3dop::kRadToDeg
                                          : fb.extra[k]);
    }
  }
  for (auto &sc : scalars)
    if (sc.second.isValid())
      sc.second.getAttribute()->bumpDataId();
}

// A body that can still move: not the world, not static, and awake.
bool bodyMoves(const b3dop::Box3DWorld &world, b3dop::BodyHandle h) {
  return h && world.typeByHandle(h) != b3dop::BodySettings::Type::Static &&
         world.isAwakeByHandle(h);
}

// Joints whose bodies are both at rest and were already written that way: their
// frames and feedback cannot have changed, so this step does not write them.
void markQuietJoints(const b3dop::Box3DWorld &world,
                     const std::vector<Box3DJointEntry *> &entries) {
  for (Box3DJointEntry *e : entries) {
    const bool asleep =
        e->handle && !bodyMoves(world, e->bodyA) && !bodyMoves(world, e->bodyB);
    e->quiet = asleep && e->asleepWritten && e->wrote && !e->writeLocal &&
               !e->switchFrames && e->madeRestLength < 0.0f && !e->glue;
    e->asleepWritten = asleep;
  }
}

// One network's results into its (locked, writable) geometry. True if a
// constraint_name switched.
bool writeNetworkResults(const b3dop::Box3DWorld &world, GU_Detail *gdp,
                         const std::vector<Box3DJointEntry *> &entries,
                         const std::vector<Box3DJointEntry *> *broken) {
  const PrimExists primOk{gdp};
  const bool switched =
      broken && writeBrokenJoints(world, gdp, primOk, *broken);
  markQuietJoints(world, entries);
  writeLiveFrames(world, gdp, entries, primOk);
  writeFeedback(world, gdp, primOk, entries);
  return switched;
}

} // namespace

int jointsAfterStep(Box3DSharedState &st, SIM_ObjectArray &objects) {
  b3dop::Box3DWorld &world = st.world;

  EntriesByNetwork brokeIn; // network -> joints that broke
  const int broke = breakJoints(st, brokeIn);

  // ---- write-back: broken group / next_constraint_name, live frames, feedback
  // ----
  EntriesByNetwork entriesOf;
  bool entriesGathered = false;
  for (auto &kv : st.jointNetworks) {
    Box3DJointNetwork &net = kv.second;
    const auto broken = brokeIn.find(kv.first);
    const bool anyBroken = broken != brokeIn.end();
    if (net.visited != st.syncStamp)
      continue;
    if (!entriesGathered) {
      entriesGathered = true;
      for (auto &je : st.joints)
        entriesOf[je.second.network].push_back(&je.second);
    }
    const auto ents = entriesOf.find(kv.first);
    if (ents == entriesOf.end() && !anyBroken)
      continue; // no joints to keep live, nothing broke
    SIM_Object *obj = box3dObjectById(objects, net.objectId, st.objectsSorted);
    NetworkSource src;
    src.inGroup = net.inGroup;
    src.relName = net.relName;
    SIM_GeometryCopy *geo = obj ? writableNetworkGeometry(*obj, src) : nullptr;
    net.writable = geo != nullptr;
    if (!geo)
      continue;
    static const std::vector<Box3DJointEntry *> kNone;
    const std::vector<Box3DJointEntry *> &entries =
        ents != entriesOf.end() ? ents->second : kNone;
    bool switched = false;
    {
      SIM_GeometryAutoWriteLock lock(geo);
      GU_Detail *gdp = &lock.getGdp();
      switched = writeNetworkResults(world, gdp, entries,
                                     anyBroken ? &broken->second : nullptr);
    }
    // Our own write is not an edit: re-record the fingerprint (the lock is
    // released). A switched constraint_name is ours too, but must be read:
    // force a re-read.
    if (const SIM_Geometry *rgeo = networkGeometry(*obj, src)) {
      SIM_GeometryAutoReadLock rlock(rgeo);
      if (const GU_Detail *rgdp = rlock.getGdp())
        net.fingerprint = networkFingerprint(rgdp);
    }
    if (switched)
      net.fingerprint = 0;
  }
  return broke;
}
