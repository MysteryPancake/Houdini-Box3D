// Box3DJointBreak.cpp
//
// Constraint networks after the step: Glue's impulse model, joints that break
// (or switch to their next constraint) and the live frames and feedback
// attributes written onto the networks. Box3DJointSync.cpp reads the networks
// and makes the joints.
#include "Box3DCompoundSync.h"
#include "Box3DHoudiniUtil.h"
#include "Box3DJointNetwork.h"
#include "Box3DJointSync.h"
#include "core/Box3DBond.h"
#include "core/Box3DCompound.h"
#include "core/Box3DDebug.h"
#include "core/Box3DPatch.h"
#include "dop/SIM_Box3DWorld.h"

#include <GA/GA_AIFStringTuple.h>
#include <GA/GA_AIFTuple.h>
#include <GA/GA_AttributeDict.h>
#include <GA/GA_Defaults.h>
#include <GA/GA_Handle.h>
#include <GA/GA_Primitive.h>
#include <GU/GU_Detail.h>
#include <SIM/SIM_Geometry.h>
#include <SIM/SIM_GeometryCopy.h>
#include <SIM/SIM_Object.h>
#include <SIM/SIM_ObjectArray.h>
#include <UT/UT_ParallelUtil.h>
#include <UT/UT_Quaternion.h>
#include <UT/UT_StringHolder.h>
#include <UT/UT_Vector3.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstring>
#include <deque>
#include <functional>
#include <initializer_list>
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

  // The rare per-joint work (rest length, local frames) is serial; the live
  // frames, one write per bond every step, go in parallel afterwards.
  struct Work {
    Box3DJointEntry *e;
    GA_Offset pts[2];
  };
  // Kept between steps: no allocation.
  thread_local std::vector<Work> work;
  work.clear();
  work.reserve(entries.size());
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

    work.push_back({e, {pts[0], pts[1]}});
  }

  // Each item reads the world (read-only calls may run in parallel) and writes
  // its own entry and its own two points, whose pages are hardened first.
  // A reference: `work` is thread_local, so naming it inside the lambda would
  // find each worker thread's own (empty) copy.
  const std::vector<Work> &items = work;
  std::atomic<bool> wroteFrames{false};
  auto writeOne = [&](size_t i) {
    Box3DJointEntry *e = items[i].e;
    const GA_Offset *pts = items[i].pts;
    b3dop::JointFrame fa, fb;
    if (world.jointWorldFrames(e->handle, fa, fb)) {
      // The anchors, not the frames (SideFX data may offset the frames from
      // them).
      e->wroteA = asStored(anchorFrameOf(fa, e->adjA));
      e->wroteB = asStored(anchorFrameOf(fb, e->adjB));
      e->wrote = true;
    } else if (!e->switchFrames)
      return;                // broken, or never made
    e->switchFrames = false; // (switched: wroteA / wroteB are set already)
    const b3dop::JointFrame *frames[2] = {&e->wroteA, &e->wroteB};
    for (int k = 0; k < 2; ++k) {
      const b3dop::JointFrame &f = *frames[k];
      pH.set(pts[k], box3dVec3F(f.pos));
      orientH.set(pts[k],
                  UT_QuaternionF(f.rot[0], f.rot[1], f.rot[2], f.rot[3]));
    }
    wroteFrames.store(true, std::memory_order_relaxed);
  };
  constexpr size_t kMinParallelFrames = 4096;
  if (work.size() >= kMinParallelFrames) {
    pH.getAttribute()->hardenAllPages();
    orientH.getAttribute()->hardenAllPages();
    UTparallelFor(UT_BlockedRange<size_t>(0, work.size()),
                  [&writeOne](const UT_BlockedRange<size_t> &r) {
                    for (size_t i = r.begin(); i != r.end(); ++i)
                      writeOne(i);
                  });
  } else {
    for (size_t i = 0; i < work.size(); ++i)
      writeOne(i);
  }
  // Only what was written changes its data id (anything watching these
  // attributes sees a change only when there is one).
  if (wroteFrames.load()) {
    pH.getAttribute()->bumpDataId();
    orientH.getAttribute()->bumpDataId();
  }
  if (wroteRestLength)
    restLengthH.getAttribute()->bumpDataId();
  if (wroteLocal) {
    localPH.getAttribute()->bumpDataId();
    localQH.getAttribute()->bumpDataId();
  }
}

} // namespace

namespace {

// Joint local frames are read first (a next constraint may start from them),
// then the joint is destroyed and the entry marked broken.
void destroyBrokenJoint(b3dop::Box3DWorld &world, Box3DJointEntry &e) {
  Box3DJointExtra &x = e.ensureExtra();
  e.brokeLocalKnown =
      world.jointLocalFrames(e.handle, x.brokeLocalA, x.brokeLocalB);
  world.destroyJoint(e.handle);
  e.handle = 0;
  e.broken = true;
}

} // namespace

// The glue group of each node (st.glueGroup), if the bonds changed.
void refreshGlueGroups(Box3DSharedState &st) {
  if (st.glueGroupKnown && st.glueGroupSignature == st.glueSignature &&
      st.glueGroupNodes == st.glueNodes.size())
    return;
  st.glueGroup =
      b3dop::glueComponents(st.glueNodes.size(), glueBondList(st, false, true))
          .of;
  st.glueGroupSignature = st.glueSignature;
  st.glueGroupNodes = st.glueNodes.size();
  st.glueGroupKnown = true;
}

void collectGlueHits(Box3DSharedState &st) {
  b3dop::Box3DWorld &world = st.world;
  ++st.hitsTick;
  refreshGlueGroups(st);
  st.compoundGroup.clear();
  for (const auto &c : st.compounds)
    st.compoundGroup[c->body] = compoundGlueGroup(st, *c);
  // A hit is a contact Box3D made a hit event for: one approaching faster than
  // its hit threshold, with the impulse of the whole contact. A body resting on
  // something has none, so the weight it carries is never a hit. An event with
  // a piece of the body's own group (or the body of a compound of it) is not
  // one either: pieces of one glued structure knock into each other when it
  // moves (the hit on the structure itself counts).
  int group = -1;
  const std::function<bool(b3dop::BodyHandle)> sameGroup =
      [&st, &group](b3dop::BodyHandle other) {
        return glueSameGroup(st, other, group);
      };
  for (size_t i = 0; i < st.glueNodes.size(); ++i) {
    Box3DGlueNode &n = st.glueNodes[i];
    // A piece of a compound takes the hits on its shapes of the compound.
    float pieceHits = 0.0f;
    if (compoundGlueHits(st, n.body, pieceHits)) {
      n.hits += pieceHits;
      continue;
    }
    // Hits land on moving bodies; a sleeping one has none this step.
    if (world.typeByHandle(n.body) != b3dop::BodySettings::Type::Dynamic ||
        !world.isAwakeByHandle(n.body))
      continue;
    group = st.glueGroup[i];
    n.hits += world.hitImpulseByHandle(n.body, &sameGroup);
  }
}

std::vector<b3dop::GlueBond> glueBondList(const Box3DSharedState &st,
                                          bool withParams, bool blankBroken) {
  std::vector<b3dop::GlueBond> bonds;
  bonds.reserve(st.glueBonds.size());
  for (size_t i = 0; i < st.glueBonds.size(); ++i) {
    const Box3DJointEntry *e = st.glueBonds[i];
    b3dop::GlueBond g;
    g.a = st.glueBondNodes[i].first;
    g.b = st.glueBondNodes[i].second;
    if (withParams) {
      g.params = e->extra->glueParams;
      g.impact = e->extra->glueImpact;
    }
    if (blankBroken && e->broken)
      g.a = g.b = -1;
    bonds.push_back(g);
  }
  return bonds;
}

namespace {
// How far apart two faces may be (or how far into each other) and still touch:
// Box3D's speculative distance.
constexpr double kPatchTolerance = 0.03;

constexpr float kFixedMass = std::numeric_limits<float>::infinity();

// A body's mass for a bond's side: infinity for one that does not move.
float sideBodyMass(b3dop::Box3DWorld &world, b3dop::BodyHandle h) {
  b3dop::BodyState s;
  if (!h || world.typeByHandle(h) != b3dop::BodySettings::Type::Dynamic ||
      !world.bodyStateByHandle(h, s) || !(s.mass > 0.0f))
    return kFixedMass;
  return s.mass;
}

// The mass of everything bonded to `start` without going through `skip`: the
// pieces on its side of the cut. Infinite when it holds a fixed body.
// `reached` is set when `target` is among them (the bond is not the only way).
float bondSideMass(Box3DSharedState &st, b3dop::BodyHandle start,
                   const Box3DJointEntry *skip, b3dop::BodyHandle target,
                   bool &reached) {
  reached = false;
  const auto first = st.glueNodeOf.find(start);
  if (first == st.glueNodeOf.end())
    return sideBodyMass(st.world, start);
  std::vector<char> seen(st.glueNodes.size(), 0);
  std::vector<int> stack = {first->second};
  seen[static_cast<size_t>(first->second)] = 1;
  float mass = 0.0f;
  while (!stack.empty()) {
    const int n = stack.back();
    stack.pop_back();
    const Box3DGlueNode &node = st.glueNodes[static_cast<size_t>(n)];
    if (node.body == target)
      reached = true;
    mass += sideBodyMass(st.world, node.body);
    for (const Box3DJointEntry *b : node.bonds) {
      if (b == skip || !b->handle || b->broken)
        continue;
      const b3dop::BodyHandle other =
          b->bodyA == node.body ? b->bodyB : b->bodyA;
      const auto next = st.glueNodeOf.find(other);
      if (next == st.glueNodeOf.end() ||
          seen[static_cast<size_t>(next->second)])
        continue;
      seen[static_cast<size_t>(next->second)] = 1;
      stack.push_back(next->second);
    }
  }
  return mass;
}

// The reduced mass of the two sides the bond holds together, kept until the
// bonds change. Cutting it separates two groups when nothing else joins them;
// where other bonds do (a loop, a wall), tearing it frees nothing alone, and
// the two pieces are what it holds.
float bondReducedMassOf(Box3DSharedState &st, Box3DJointEntry &e) {
  Box3DJointExtra &x = e.ensureExtra();
  if (!x.bondSidesKnown || x.bondSidesSignature != st.glueSignature) {
    bool reached = false;
    float a = bondSideMass(st, e.bodyA, &e, e.bodyB, reached);
    float b = reached ? 0.0f : bondSideMass(st, e.bodyB, &e, e.bodyA, reached);
    if (!e.bodyA)
      a = kFixedMass;
    if (reached) {
      a = sideBodyMass(st.world, e.bodyA);
      b = sideBodyMass(st.world, e.bodyB);
    }
    x.bondSideMass[0] = a;
    x.bondSideMass[1] = b;
    x.bondSidesSignature = st.glueSignature;
    x.bondSidesKnown = true;
  }
  return b3dop::bondReducedMass(x.bondSideMass[0], x.bondSideMass[1]);
}

// The joints of each network, again when the joints changed (syncJoints erases
// and adds entries, so the lists must not be read before this).
void refreshNetworkEntries(Box3DSharedState &st) {
  if (!st.networkEntriesDirty)
    return;
  st.networkEntriesDirty = false;
  st.networkEntries.clear();
  for (auto &je : st.joints)
    st.networkEntries[je.second.network].push_back(&je.second);
}
} // namespace

bool bondGeometry(Box3DSharedState &st, Box3DJointEntry &e) {
  Box3DJointExtra &x = e.ensureExtra();
  if (x.bondReady && b3dop::sameFloat(x.bondAreaSetting, e.settings.bondArea))
    return true;
  x.bondReady = false;
  b3dop::Box3DWorld &world = st.world;
  // The normal runs from A's centre to B's (from the anchor when A is the
  // world), the area is the smaller face of the two boxes across it.
  double posA[3] = {0, 0, 0};
  float rotA[4] = {0, 0, 0, 1};
  if (e.bodyA && !world.getTransformByHandle(e.bodyA, posA, rotA))
    return false;
  b3dop::BodyState sb;
  b3dop::JointFrame wa, wb;
  if (!world.bodyStateByHandle(e.bodyB, sb) ||
      !world.jointWorldFrames(e.handle, wa, wb))
    return false;
  double from[3] = {wa.pos[0], wa.pos[1], wa.pos[2]};
  b3dop::BodyState sa;
  if (e.bodyA && world.bodyStateByHandle(e.bodyA, sa))
    std::copy(sa.com, sa.com + 3, from);
  float n[3];
  float len = 0.0f;
  for (int i = 0; i < 3; ++i) {
    n[i] = static_cast<float>(sb.com[i] - from[i]);
    len += n[i] * n[i];
  }
  len = std::sqrt(len);
  if (!(len > 1e-6f)) {
    n[0] = 0.0f;
    n[1] = 1.0f;
    n[2] = 0.0f;
  } else
    for (float &v : n)
      v /= len;
  float conj[4];
  b3dop::quatConj(rotA, conj);
  b3dop::quatRotate(conj, n, x.bondNormalLocal);
  float eb[3], ea[3];
  if (!world.tightExtentByHandle(e.bodyB, eb))
    return false;
  if (!e.bodyA || !world.tightExtentByHandle(e.bodyA, ea))
    std::copy(eb, eb + 3, ea);
  x.bondAreaUsed = e.settings.bondArea > 0.0f ? e.settings.bondArea
                                              : b3dop::bondAreaOf(ea, eb, n);
  // A piece thinner than Box3D's margins has no area without them: its whole
  // box instead.
  if (!(x.bondAreaUsed > 0.0f)) {
    float fullB[3], fullA[3];
    if (world.extentByHandle(e.bodyB, fullB)) {
      if (!e.bodyA || !world.extentByHandle(e.bodyA, fullA))
        std::copy(fullB, fullB + 3, fullA);
      x.bondAreaUsed = b3dop::bondAreaOf(fullA, fullB, n);
    }
  }
  // The real patch where the two pieces' convex faces touch: its normal, area,
  // centroid and section moduli replace the estimates above.
  x.patchOk = false;
  std::vector<b3dop::PatchFace> facesA, facesB;
  if (e.bodyA && world.hullFacesByHandle(e.bodyA, facesA) > 0 &&
      world.hullFacesByHandle(e.bodyB, facesB) > 0) {
    const b3dop::ContactPatch patch =
        b3dop::contactPatchOf(facesA, facesB, kPatchTolerance);
    if (patch.ok && patch.extentU > 0.0 && patch.extentV > 0.0 &&
        patch.extentR > 0.0) {
      const auto toLocal = [&](const double vec[3], float out[3]) {
        const float v[3] = {static_cast<float>(vec[0]),
                            static_cast<float>(vec[1]),
                            static_cast<float>(vec[2])};
        b3dop::quatRotate(conj, v, out);
      };
      const double fromA[3] = {patch.centroid[0] - posA[0],
                               patch.centroid[1] - posA[1],
                               patch.centroid[2] - posA[2]};
      toLocal(fromA, x.patchCentroidLocal);
      toLocal(patch.normal, x.bondNormalLocal);
      toLocal(patch.u, x.patchULocal);
      toLocal(patch.v, x.patchVLocal);
      // A set Contact Area scales the patch (its shape kept).
      const double scale =
          e.settings.bondArea > 0.0f ? e.settings.bondArea / patch.area : 1.0;
      x.bondAreaUsed = static_cast<float>(patch.area * scale);
      const double lengthScale = std::sqrt(scale);
      x.modulusU = static_cast<float>(patch.momentU / patch.extentV * scale *
                                      lengthScale);
      x.modulusV = static_cast<float>(patch.momentV / patch.extentU * scale *
                                      lengthScale);
      x.modulusTwist = static_cast<float>((patch.momentU + patch.momentV) /
                                          patch.extentR * scale * lengthScale);
      x.patchOk = true;
    }
  }
  // No area to stress: not ready, so it is looked at again rather than kept
  // as a bond that never breaks.
  if (!(x.bondAreaUsed > 0.0f))
    return false;
  x.bondAreaSetting = e.settings.bondArea;
  x.bondReady = true;
  return true;
}

void breakBonds(Box3DSharedState &st, float dt) {
  b3dop::Box3DWorld &world = st.world;
  refreshNetworkEntries(st);
  solveCompoundBonds(st, dt);
  for (auto &network : st.networkEntries)
    for (Box3DJointEntry *e : network.second) {
      if (e->type != b3dop::JointType::Bond || !e->handle || e->broken)
        continue;
      Box3DJointExtra &x = e->ensureExtra();
      if (x.bondQueued)
        continue;
      // The load of a bond in a compound, or one held by a bridge, was worked
      // out this step (solveCompoundBonds); a compound that is asleep has none.
      const bool worked = x.bondLoadTick == st.hitsTick;
      // A bond over its capacity goes on taking damage while its pieces sleep,
      // under the load it last had (a steady one): creep to failure.
      const bool carrying = x.bondOverCapacity;
      if (e->inCompound && !worked && !carrying)
        continue;
      // Nothing moves, so nothing changes, while both pieces are asleep.
      if (!worked && !carrying && !world.isAwakeByHandle(e->bodyA) &&
          !world.isAwakeByHandle(e->bodyB))
        continue;
      if (!bondGeometry(st, *e))
        continue;
      // A bond inside a compound is posed by the compound (the parked pieces
      // are a step behind), the others by their bodies.
      const bool posed = e->inCompound && x.bondPoseTick == st.hitsTick;
      double posA[3] = {0, 0, 0};
      float rotA[4] = {0, 0, 0, 1};
      if (posed) {
        std::copy(x.bondPosA, x.bondPosA + 3, posA);
        std::copy(x.bondRotA, x.bondRotA + 4, rotA);
      } else if (e->bodyA && !world.getTransformByHandle(e->bodyA, posA, rotA))
        continue;
      b3dop::JointFeedback fb;
      if (worked || e->inCompound) {
        std::copy(x.bondForce, x.bondForce + 3, fb.force);
        std::copy(x.bondTorque, x.bondTorque + 3, fb.torque);
      } else {
        if (!world.jointFeedback(e->handle, e->type, fb))
          continue;
        // The load over the whole step, not the last substep's: an impact in an
        // earlier substep is in the first and mostly gone from the second.
        for (int i = 0; i < 3; ++i) {
          fb.force[i] = fb.impulse[i] / dt;
          fb.torque[i] = fb.angularImpulse[i] / dt;
        }
      }
      float normal[3];
      b3dop::quatRotate(rotA, x.bondNormalLocal, normal);
      float patchU[3] = {1, 0, 0}, patchV[3] = {0, 1, 0};
      if (x.patchOk) {
        b3dop::quatRotate(rotA, x.patchULocal, patchU);
        b3dop::quatRotate(rotA, x.patchVLocal, patchV);
        // The load is at the weld's anchor; the stress is about the patch's
        // centroid: the moment there is the couple plus the force's moment
        // from the anchor to it.
        b3dop::JointFrame wa, wb;
        float centre[3];
        b3dop::quatRotate(rotA, x.patchCentroidLocal, centre);
        if (posed)
          std::copy(x.bondAnchor, x.bondAnchor + 3, wa.pos);
        if (posed || world.jointWorldFrames(e->handle, wa, wb)) {
          const double r[3] = {wa.pos[0] - (posA[0] + centre[0]),
                               wa.pos[1] - (posA[1] + centre[1]),
                               wa.pos[2] - (posA[2] + centre[2])};
          fb.torque[0] +=
              static_cast<float>(r[1] * fb.force[2] - r[2] * fb.force[1]);
          fb.torque[1] +=
              static_cast<float>(r[2] * fb.force[0] - r[0] * fb.force[2]);
          fb.torque[2] +=
              static_cast<float>(r[0] * fb.force[1] - r[1] * fb.force[0]);
        }
      }
      b3dop::BondParams p;
      p.tensile = e->settings.bondTensile;
      p.shear = e->settings.bondShear;
      p.bending = e->settings.bondBending;
      p.compressive = e->settings.bondCompressive;
      p.area = x.bondAreaUsed;
      p.fracture = e->settings.bondFracture;
      if (x.patchOk) {
        p.modulusU = x.modulusU;
        p.modulusV = x.modulusV;
        p.modulusTwist = x.modulusTwist;
      }
      const b3dop::BondLoad load =
          x.patchOk ? b3dop::bondLoadInPatch(fb.force, fb.torque, normal,
                                             patchU, patchV)
                    : b3dop::bondLoadOf(fb.force, fb.torque, normal);
      const float ratio = b3dop::bondRatio(p, load);
      x.bondOverCapacity = ratio >= 1.0f;
      // What the load beyond the capacity has to accelerate apart: the two
      // sides the bond holds together.
      float tear = 0.0f;
      if (p.fracture > 0.0f)
        tear = b3dop::bondTearImpulse(p.fracture, p.area,
                                      bondReducedMassOf(st, *e));
      if (b3dop::bondAdvance(x.bondImpulse, ratio, b3dop::bondCapacity(p), dt,
                             tear)) {
        x.bondQueued = true;
        st.glueBroken.push_back(e);
      }
    }
}

void advanceGlue(Box3DSharedState &st, float dt) {
  if (st.glueBonds.empty())
    return;
  // Nothing hit this frame and no bond carries any impact: the decay and the
  // spread would change nothing, so skip building the bond list.
  bool idle = true;
  for (const Box3DGlueNode &n : st.glueNodes)
    if (n.hits > 0.0f) {
      idle = false;
      break;
    }
  if (idle)
    for (const Box3DJointEntry *e : st.glueBonds)
      if (e->extra->glueImpact > 0.0f || std::isnan(e->extra->glueImpact)) {
        idle = false;
        break;
      }
  if (idle)
    return;
  std::vector<float> hits(st.glueNodes.size());
  for (size_t i = 0; i < hits.size(); ++i)
    hits[i] = st.glueNodes[i].hits;
  std::vector<b3dop::GlueBond> bonds = glueBondList(st, true, true);
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
  // Glue bonds over their strength (advanceGlue) and Bonds (breakBonds).
  events.insert(events.end(), st.glueBroken.begin(), st.glueBroken.end());
  st.glueBroken.clear();
  if (!events.empty())
    st.glueDirty = true; // the Glue graph loses a bond
  int broke = 0;
  for (void *p : events) {
    Box3DJointEntry *e = static_cast<Box3DJointEntry *>(p);
    if (!e || !e->handle)
      continue;
    destroyBrokenJoint(world, *e);
    // A bond is taken out of the glue lists; any other joint changes what is
    // tied to what, so they are made again.
    if (inGlueGraph(*e) && e->glueIndex >= 0)
      st.glueRemoved.push_back(e);
    else
      st.glueFull = true;
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

// The attributes of a switching constraint's next settings: every primitive
// attribute `next_<setting>` (but next_constraint_name) holds the value the
// primitive's `<setting>` takes when it switches, "not set" (kUnsetFloat for
// numbers, a negative integer for toggles, an empty string) meaning none.
struct NextSettings {
  std::vector<GA_Attribute *> attribs;

  explicit NextSettings(GU_Detail *gdp) {
    constexpr const char *kPrefix = "next_";
    const GA_AttributeDict &dict =
        gdp->getAttributes().getDict(GA_ATTRIB_PRIMITIVE);
    for (GA_AttributeDict::iterator it = dict.begin(GA_SCOPE_PUBLIC);
         !it.atEnd(); ++it) {
      GA_Attribute *a = it.attrib();
      const char *name = a->getName().c_str();
      if (std::strncmp(name, kPrefix, 5) == 0 &&
          std::strcmp(name, kNextConstraintName) != 0)
        attribs.push_back(a);
    }
  }
};

// Moves the primitive's next settings onto its own: each `next_<x>` that is set
// becomes `<x>` (made, with every other primitive "not set", if the geometry
// has none) and is cleared, so it applies once. `next_constraint_type` is
// `constraint_type`. True if any moved.
bool applyNextSettings(GU_Detail *gdp, const NextSettings &next,
                       GA_Offset prim) {
  bool moved = false;
  for (GA_Attribute *from : next.attribs) {
    const std::string base = std::string(from->getName().c_str() + 5);
    if (from->getAIFStringTuple()) {
      GA_ROHandleS value(from);
      const UT_StringHolder text =
          value.isValid() ? value.get(prim) : UT_StringHolder();
      if (!text.isstring() || text.length() == 0)
        continue;
      if (!gdp->findPrimitiveAttribute(base.c_str()))
        gdp->addStringTuple(GA_ATTRIB_PRIMITIVE, base.c_str(), 1);
      GA_RWHandleS to(gdp, GA_ATTRIB_PRIMITIVE, base.c_str());
      if (!to.isValid())
        continue;
      to.set(prim, text);
      GA_RWHandleS(from).set(prim, UT_StringHolder());
      to.getAttribute()->bumpDataId();
      from->bumpDataId();
      moved = true;
      continue;
    }
    const GA_AIFTuple *tuple = from->getAIFTuple();
    if (!tuple)
      continue;
    const int n = std::min(tuple->getTupleSize(from), 4);
    const bool isFloat = tuple->getStorage(from) == GA_STORE_REAL32 ||
                         tuple->getStorage(from) == GA_STORE_REAL64;
    fpreal64 v[4] = {0, 0, 0, 0};
    tuple->get(from, prim, v, n);
    // "Not set": a number at kUnsetFloat, a toggle or integer below 0.
    if (isFloat ? b3dop::isUnsetFloat(static_cast<float>(v[0])) : v[0] < 0)
      continue;
    GA_Attribute *to = gdp->findPrimitiveAttribute(base.c_str());
    if (!to) {
      fpreal64 unset[4] = {
          isFloat ? static_cast<fpreal64>(b3dop::kUnsetFloat) : -1.0, 0, 0, 0};
      const GA_Defaults defaults(unset, n);
      to = isFloat ? gdp->addFloatTuple(GA_ATTRIB_PRIMITIVE, base.c_str(), n,
                                        defaults)
                   : gdp->addIntTuple(GA_ATTRIB_PRIMITIVE, base.c_str(), n,
                                      defaults);
    }
    const GA_AIFTuple *target = to ? to->getAIFTuple() : nullptr;
    if (!target)
      continue;
    target->set(to, prim, v, n);
    fpreal64 clear[4] = {
        isFloat ? static_cast<fpreal64>(b3dop::kUnsetFloat) : -1.0, 0, 0, 0};
    tuple->set(from, prim, clear, n);
    to->bumpDataId();
    from->bumpDataId();
    moved = true;
  }
  return moved;
}

// The joints that broke: a primitive with a next_constraint_name switches to
// it, with its next settings (applyNextSettings); the rest go into the `broken`
// group. True if any switched.
bool writeBrokenJoints(const b3dop::Box3DWorld &world, GU_Detail *gdp,
                       const PrimExists &primOk,
                       const std::vector<Box3DJointEntry *> &broken) {
  GA_RWHandleS conNameH(gdp, GA_ATTRIB_PRIMITIVE, kConstraintName);
  GA_RWHandleS nextH(gdp, GA_ATTRIB_PRIMITIVE, kNextConstraintName);
  GA_PrimitiveGroup *grp = nullptr;
  bool switched = false;
  const NextSettings next(gdp);
  for (Box3DJointEntry *e : broken) {
    if (!primOk(e->prim))
      continue;
    UT_StringHolder nextName;
    if (nextH.isValid() && conNameH.isValid())
      nextName = nextH.get(e->prim);
    if (nextName.isstring()) {
      conNameH.set(e->prim, nextName);
      nextH.set(e->prim, UT_StringHolder());
      applyNextSettings(gdp, next, e->prim);
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

// Every joint is quiet (markQuietJoints): nothing of it needs writing.
bool allQuiet(const std::vector<Box3DJointEntry *> &entries) {
  return std::all_of(entries.begin(), entries.end(),
                     [](const Box3DJointEntry *e) { return e->quiet; });
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
  if (allQuiet(entries))
    return;
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
  for (Box3DJointEntry *e : entries) {
    if (e->quiet)
      continue;
    if (e->glue && e->inCompound) {
      // A weld inside a compound carries nothing: zeros once, then `impact`
      // when it changed.
      if (e->extra && primOk(e->prim)) {
        if (!e->feedbackZeroed) {
          for (GA_RWHandleF *h : {&forceH, &distH, &torqueH, &angleH})
            if (h->isValid())
              h->set(e->prim, 0.0f);
          e->feedbackZeroed = true;
        }
        if (e->extra->glueImpact != e->impactWritten) {
          GA_RWHandleF &impactH = scalar(kImpact);
          if (impactH.isValid())
            impactH.set(e->prim, e->extra->glueImpact);
          e->impactWritten = e->extra->glueImpact;
        }
      }
      continue;
    }
    e->feedbackZeroed = false;
    if (e->type == b3dop::JointType::Bond && e->extra && primOk(e->prim) &&
        e->extra->bondImpulse != e->impactWritten) {
      // A Bond's damage, kept in the geometry (impactWritten is its record).
      GA_RWHandleF &impulseH = scalar(kBondImpulse);
      if (impulseH.isValid())
        impulseH.set(e->prim, e->extra->bondImpulse);
      e->impactWritten = e->extra->bondImpulse;
    }
    if (e->glue && e->extra && primOk(e->prim)) {
      GA_RWHandleF &impactH = scalar(kImpact);
      if (impactH.isValid())
        impactH.set(e->prim, e->extra->glueImpact);
      e->impactWritten = e->extra->glueImpact;
    }
    b3dop::JointFeedback fb;
    if (!primOk(e->prim) || !world.jointFeedback(e->handle, e->type, fb))
      continue;
    // A Bond inside a compound has a parked weld that carries nothing: its load
    // is the compound's spring network's (the last step's).
    if (e->type == b3dop::JointType::Bond && e->inCompound && e->extra) {
      std::copy(e->extra->bondForce, e->extra->bondForce + 3, fb.force);
      std::copy(e->extra->bondTorque, e->extra->bondTorque + 3, fb.torque);
    }
    if (forceH.isValid())
      forceH.set(e->prim, length(fb.force));
    if (distH.isValid()) {
      // As RBD Bullet Solver: how far apart the two anchors are now (the ones
      // written to current_P), not how far the joint is from satisfied.
      // (writeLiveFrames, just before, stored this step's anchors.)
      float distance = fb.linearSeparation;
      if (e->wrote) {
        const double d[3] = {e->wroteA.pos[0] - e->wroteB.pos[0],
                             e->wroteA.pos[1] - e->wroteB.pos[1],
                             e->wroteA.pos[2] - e->wroteB.pos[2]};
        distance = static_cast<float>(
            std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]));
      }
      distH.set(e->prim, distance);
    }
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

// Joints whose bodies are both at rest and were already written that way, and
// joints that broke and had their last values written: their frames and
// feedback cannot have changed, so this step does not write them.
void markQuietJoints(const b3dop::Box3DWorld &world,
                     const std::vector<Box3DJointEntry *> &entries) {
  for (Box3DJointEntry *e : entries) {
    const bool asleep =
        e->handle && !bodyMoves(world, e->bodyA) && !bodyMoves(world, e->bodyB);
    // A bond inside a compound joins parked (never awake) bodies that move with
    // their compound: never quiet.
    const bool nothingPending =
        !e->inCompound && !e->writeLocal && !e->switchFrames &&
        e->madeRestLength < 0.0f &&
        // a Glue's impact decays each step until it reaches zero
        (!e->glue || (e->extra && e->extra->glueImpact == e->impactWritten)) &&
        // a Bond's damage is written when it changes
        (e->type != b3dop::JointType::Bond ||
         (e->extra && e->extra->bondImpulse == e->impactWritten));
    // A joint that broke has no frames or feedback left to write, only a
    // Glue's final impact.
    const bool spent = e->broken && !e->handle && nothingPending;
    e->quiet =
        spent || (asleep && e->asleepWritten && e->wrote && nothingPending);
    e->asleepWritten = asleep;
  }
}

// Enable Aging, as RBD Bullet Solver: every constraint primitive's `age` (made
// if missing) grows by the time step.
void ageConstraints(GU_Detail *gdp, float dt) {
  if (!gdp->findPrimitiveAttribute("age"))
    gdp->addFloatTuple(GA_ATTRIB_PRIMITIVE, "age", 1);
  GA_RWHandleF age(gdp, GA_ATTRIB_PRIMITIVE, "age");
  if (!age.isValid())
    return;
  for (GA_Iterator it(gdp->getPrimitiveRange()); !it.atEnd(); ++it)
    age.set(*it, age.get(*it) + dt);
  age.getAttribute()->bumpDataId();
}

// One network's results into its (locked, writable) geometry. True if a
// constraint_name switched.
bool writeNetworkResults(const b3dop::Box3DWorld &world, GU_Detail *gdp,
                         const std::vector<Box3DJointEntry *> &entries,
                         const std::vector<Box3DJointEntry *> *broken,
                         float ageDt) {
  const PrimExists primOk{gdp};
  const bool switched =
      broken && writeBrokenJoints(world, gdp, primOk, *broken);
  writeLiveFrames(world, gdp, entries, primOk);
  writeFeedback(world, gdp, primOk, entries);
  if (ageDt > 0.0f)
    ageConstraints(gdp, ageDt);
  return switched;
}

} // namespace

int jointsAfterStep(Box3DSharedState &st, SIM_ObjectArray &objects) {
  b3dop::Box3DWorld &world = st.world;

  EntriesByNetwork brokeIn; // network -> joints that broke
  const int broke = breakJoints(st, brokeIn);

  // ---- write-back: broken group / next_constraint_name, live frames, feedback
  // ----
  EntriesByNetwork &entriesOf = st.networkEntries;
  for (auto &kv : st.jointNetworks) {
    Box3DJointNetwork &net = kv.second;
    const auto broken = brokeIn.find(kv.first);
    const bool anyBroken = broken != brokeIn.end();
    if (net.visited != st.syncStamp)
      continue;
    refreshNetworkEntries(st);
    const auto ents = entriesOf.find(kv.first);
    if (ents == entriesOf.end() && !anyBroken)
      continue; // no joints to keep live, nothing broke
    static const std::vector<Box3DJointEntry *> kNone;
    const std::vector<Box3DJointEntry *> &entries =
        ents != entriesOf.end() ? ents->second : kNone;
    // Nothing broke, no age to add and every joint is as it was last written
    // (both bodies at rest): the geometry is left alone, with no write lock,
    // no new data ids and no fingerprint to take again.
    markQuietJoints(world, entries);
    if (!anyBroken && st.constraintAgeDt <= 0.0f && allQuiet(entries))
      continue;
    SIM_Object *obj = box3dObjectById(objects, net.objectId, st.objectsSorted);
    NetworkSource src;
    src.inGroup = net.inGroup;
    src.relName = net.relName;
    SIM_GeometryCopy *geo = obj ? writableNetworkGeometry(*obj, src) : nullptr;
    net.writable = geo != nullptr;
    if (!geo)
      continue;
    bool switched = false;
    {
      SIM_GeometryAutoWriteLock lock(geo);
      GU_Detail *gdp = &lock.getGdp();
      switched = writeNetworkResults(world, gdp, entries,
                                     anyBroken ? &broken->second : nullptr,
                                     st.constraintAgeDt);
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
