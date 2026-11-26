// Box3DJointSync.cpp
//
// Constraint networks -> Box3D joints: the pass over a network's primitives
// that makes, updates and removes joints, and `syncJoints`, which also keeps
// the Glue lists.
#include "Box3DJointSync.h"
#include "Box3DHoudiniUtil.h"
#include "Box3DJointNetwork.h"
#include "Box3DJointSyncInternal.h"
#include "core/Box3DDebug.h"
#include "dop/SIM_Box3DConRel.h"
#include "dop/SIM_Box3DWorld.h"
#include <GA/GA_Handle.h>
#include <GA/GA_Iterator.h>
#include <GA/GA_Primitive.h>
#include <GU/GU_Detail.h>
#include <GU/GU_DetailHandle.h>
#include <GU/GU_PackedFragment.h>
#include <GU/GU_PrimPacked.h>
#include <SIM/SIM_ConstraintNetworkRelationship.h>
#include <SIM/SIM_Geometry.h>
#include <SIM/SIM_GeometryCopy.h>
#include <SIM/SIM_Object.h>
#include <SIM/SIM_ObjectArray.h>
#include <SIM/SIM_Relationship.h>
#include <SIM/SIM_RelationshipArray.h>
#include <SIM/SIM_Utils.h>
#include <UT/UT_Matrix4.h>
#include <UT/UT_Quaternion.h>
#include <UT/UT_String.h>
#include <UT/UT_StringHolder.h>
#include <UT/UT_Vector3.h>
#include <algorithm>
#include <cmath>
#include <string>
#include <string_view>
#include <unordered_map>
#include <unordered_set>
#include <vector>

using namespace jointsync;

namespace {

// What a joint is made from: its type and bodies, plus its frames when they are
// not live (a network that is not written back keeps the authored frames, so a
// change there is an edit).
uint64_t jointSignature(b3dop::JointType type, b3dop::BodyHandle a,
                        b3dop::BodyHandle b, const b3dop::JointFrame &fa,
                        const b3dop::JointFrame &fb, bool withFrames) {
  using b3dop::mixHash;
  uint64_t h = mixHash(
      mixHash(mixHash(b3dop::kHashSeed, static_cast<uint64_t>(type)), a), b);
  if (withFrames)
    for (const b3dop::JointFrame *f : {&fa, &fb}) {
      for (const double p : f->pos)
        h = b3dop::mixDouble(h, p);
      h = b3dop::hashBytes(h, f->rot, sizeof(f->rot));
    }
  return h ? h : 1;
}

// An anchor moved away from what the solver last wrote: edited upstream (q and
// -q are equal).
bool frameEdited(const b3dop::JointFrame &now, const b3dop::JointFrame &wrote) {
  for (int k = 0; k < 3; ++k)
    if (std::fabs(now.pos[k] - wrote.pos[k]) >
        1e-5 * std::max(1.0, std::fabs(wrote.pos[k])))
      return true;
  return b3dop::quatsDiffer(now.rot, wrote.rot);
}

} // namespace

// A frame as the geometry stores it (float P), so it compares equal when read
// back.
b3dop::JointFrame asStored(const b3dop::JointFrame &f) {
  b3dop::JointFrame out = f;
  for (int k = 0; k < 3; ++k)
    out.pos[k] = static_cast<double>(static_cast<float>(f.pos[k]));
  return out;
}

namespace {

// What processNetwork() reads from a network's geometry, bound once.
struct NetworkReader {
  const GU_Detail *gdp;
  const GA_ROHandleS conName, conType, name;
  const GA_ROHandleF autoRest;
  const GA_ROHandleQ orient, localQ, currentQ;
  const GA_ROHandleV3 localP, currentP;
  const GA_PrimitiveGroup *broken;
  const GA_ROHandleI anchorIdH, anchorTypeI;
  const GA_ROHandleS anchorTypeS;
  const GA_ROHandleF impact; // Glue's accumulated impact (feedback)
  // The network's global Glue propagation iterations (detail attribute)
  const GA_ROHandleI globalIterations;
  const GA_ROHandleF globalIterationsF;

  explicit NetworkReader(const GU_Detail *g)
      : gdp(g), conName(g, GA_ATTRIB_PRIMITIVE, kConstraintName),
        conType(g, GA_ATTRIB_PRIMITIVE, kConstraintType),
        name(g, GA_ATTRIB_POINT, "name"),
        autoRest(g, GA_ATTRIB_PRIMITIVE, kAutoRestLength),
        orient(g, GA_ATTRIB_POINT, "orient"),
        localQ(g, GA_ATTRIB_POINT, "local_orient"),
        currentQ(g, GA_ATTRIB_POINT, kCurrentOrient),
        localP(g, GA_ATTRIB_POINT, "local_P"),
        currentP(g, GA_ATTRIB_POINT, kCurrentP),
        broken(g->findPrimitiveGroup(kBrokenGroup)),
        anchorIdH(g, GA_ATTRIB_POINT, kAnchorId),
        anchorTypeI(g, GA_ATTRIB_POINT, kAnchorType),
        anchorTypeS(g, GA_ATTRIB_POINT, kAnchorType),
        impact(g, GA_ATTRIB_PRIMITIVE, kImpact),
        globalIterations(g, GA_ATTRIB_GLOBAL, kPropagateIteration),
        globalIterationsF(g, GA_ATTRIB_GLOBAL, kPropagateIteration) {}

  // Glue rounds of propagation: the bond's own count, or for a negative one
  // the detail attribute's (one round when there is none).
  int glueIterations(int bondCount) const {
    if (bondCount >= 0)
      return std::clamp(bondCount, 1, b3dop::kMaxGlueIterations);
    int n = 1;
    if (globalIterations.isValid())
      n = globalIterations.get(GA_Offset(0));
    else if (globalIterationsF.isValid()) {
      const float f = globalIterationsF.get(GA_Offset(0));
      if (std::isfinite(f))
        n = static_cast<int>(
            std::clamp(f, 1.0f, static_cast<float>(b3dop::kMaxGlueIterations)));
    }
    return std::clamp(n, 1, b3dop::kMaxGlueIterations);
  }

  // The anchor's anchor_id: -1 (also: no attribute) is the object's own
  // offset, which P / orient already give.
  int anchorId(GA_Offset pt) const {
    return anchorIdH.isValid() ? anchorIdH.get(pt) : -1;
  }

  // The anchor's anchor_type: a string ("point", "vertex", "agent") or an int
  // (0, 1, 2); missing or unknown: point.
  AnchorKind anchorKind(GA_Offset pt) const {
    if (anchorTypeS.isValid()) {
      const char *t = static_cast<const char *>(anchorTypeS.get(pt));
      if (t && UT_String(t).equal("vertex", false))
        return AnchorKind::Vertex;
      if (t && UT_String(t).equal("agent", false))
        return AnchorKind::Agent;
    } else if (anchorTypeI.isValid()) {
      const int t = anchorTypeI.get(pt);
      if (t == 1)
        return AnchorKind::Vertex;
      if (t == 2)
        return AnchorKind::Agent;
    }
    return AnchorKind::Point;
  }

  std::string anchorName(GA_Offset pt) const {
    const char *n =
        name.isValid() ? static_cast<const char *>(name.get(pt)) : nullptr;
    return n ? std::string(n) : std::string();
  }

  // The anchor as authored: its P / orient, world space (the solver never
  // writes them).
  b3dop::JointFrame authored(GA_Offset pt) const {
    b3dop::JointFrame f;
    const UT_Vector3 p = gdp->getPos3(pt);
    f.pos[0] = p.x();
    f.pos[1] = p.y();
    f.pos[2] = p.z();
    if (orient.isValid()) {
      const UT_QuaternionF q = orient.get(pt);
      f.rot[0] = q.x();
      f.rot[1] = q.y();
      f.rot[2] = q.z();
      f.rot[3] = q.w();
    }
    return f;
  }

  // Where the anchor was after the last step (current_P / current_orient, a
  // zero quaternion: none), if the solver wrote it.
  bool live(GA_Offset pt, b3dop::JointFrame &out) const {
    return readFrame(currentP, currentQ, pt, out);
  }

  // The anchor's frame now: live if written, else as authored.
  b3dop::JointFrame frame(GA_Offset pt) const {
    b3dop::JointFrame f;
    return live(pt, f) ? f : authored(pt);
  }

  // The primitive's constraint_type (SideFX data only; missing: position).
  b3dop::ConstraintMode mode(GA_Offset prim) const {
    const char *t = conType.isValid()
                        ? static_cast<const char *>(conType.get(prim))
                        : nullptr;
    return b3dop::constraintModeFromString(t);
  }

  // The primitive says constraint_type position or rotation.
  bool narrowsType(GA_Offset prim) const {
    const char *t = conType.isValid()
                        ? static_cast<const char *>(conType.get(prim))
                        : nullptr;
    return t &&
           (std::strcmp(t, "position") == 0 || std::strcmp(t, "rotation") == 0);
  }

  // The primitive's constraint data (by `constraint_name`), or null.
  const Box3DJointKind *
  kind(GA_Offset prim,
       const std::unordered_map<std::string, Box3DJointKind> &kinds) const {
    const char *n = conName.isValid()
                        ? static_cast<const char *>(conName.get(prim))
                        : nullptr;
    if (!n || !*n)
      return nullptr;
    const auto it = kinds.find(std::string(n));
    return it != kinds.end() ? &it->second : nullptr;
  }

  // The anchor's saved local frame (local_P / local_orient), if it has one (a
  // zero quaternion: none).
  bool savedLocal(GA_Offset pt, b3dop::JointFrame &out) const {
    return readFrame(localP, localQ, pt, out);
  }

private:
  // A frame stored as a position and a quaternion point attribute. False when
  // either is missing, the quaternion is all zero (not written) or the position
  // is not finite.
  static bool readFrame(const GA_ROHandleV3 &posH, const GA_ROHandleQ &quatH,
                        GA_Offset pt, b3dop::JointFrame &out) {
    if (!posH.isValid() || !quatH.isValid())
      return false;
    const UT_Vector3 p = posH.get(pt);
    const UT_QuaternionF q = quatH.get(pt);
    if ((q.x() == 0.0f && q.y() == 0.0f && q.z() == 0.0f && q.w() == 0.0f) ||
        !std::isfinite(p.x()) || !std::isfinite(p.y()) || !std::isfinite(p.z()))
      return false;
    out.pos[0] = p.x();
    out.pos[1] = p.y();
    out.pos[2] = p.z();
    out.rot[0] = q.x();
    out.rot[1] = q.y();
    out.rot[2] = q.z();
    out.rot[3] = q.w();
    return true;
  }
};

struct NetworkRef {
  SIM_Object *object =
      nullptr; // an object of this solve the network was found on
  NetworkSource source;
  uint64_t key = 0;
};

// A primitive's joint: type and settings from its constraint data (with the
// primitive's own overrides), and the frame offsets of SideFX data.
struct JointSpec {
  b3dop::JointType type = b3dop::JointType::Weld;
  b3dop::JointSettings s;
  b3dop::JointFrame adjA,
      adjB; // the frames relative to the anchors: identity but for SideFX data
            // and Hold Separation
  uint64_t frameSig = 0;
  bool glue = false; // SideFX Glue: breaks on impact impulse (core/Box3DGlue.h)
  b3dop::GlueParams glueParams;
  bool massDependent = false;     // SideFX Spring: Hz from the bodies' masses
  bool reaim = false;             // SideFXJoint::reaim
  float reaimRest = 0.0f;         // its shift's length
  b3dop::SideFXBody bodyA, bodyB; // the masses it was converted with
};

// One network's primitives, read from its geometry, into joints: make, update,
// drop.
class NetworkPass {
public:
  NetworkPass(Box3DSharedState &st, const NetworkRef &ref,
              Box3DJointNetwork &net, const Anchors &anchors,
              const GU_Detail *gdp, Box3DJointCounts &counts)
      : mySt(st), myWorld(st.world), myRef(ref), myNet(net), myAnchors(anchors),
        myGdp(gdp), myCounts(counts), myRd(gdp), myReaders(bindSettings(gdp)),
        mySfReaders(bindSideFX(gdp)) {}

  void run() {
    myNet.needsBodies = false;
    // A network made all at once (a first frame, a rebuilt world) would
    // otherwise rehash the entry map every time it doubles.
    mySt.joints.reserve(mySt.joints.size() +
                        static_cast<size_t>(myGdp->getNumPrimitives()));
    for (GA_Iterator it(myGdp->getPrimitiveRange()); !it.atEnd(); ++it) {
      const GA_Offset off = *it;
      const GA_Primitive *prim = myGdp->getPrimitive(off);
      if (prim)
        processPrimitive(*prim, off);
    }
  }

private:
  void dropJoint(Box3DJointEntry &e) {
    myWorld.destroyJoint(e.handle); // harmless when already gone with a body
    e.handle = 0;
  }

  // The joint entry of a primitive (made on first sight).
  Box3DJointEntry &entryFor(GA_Offset off) {
    auto ins = mySt.joints.try_emplace(b3dop::mixHash(
        myRef.key, static_cast<uint64_t>(static_cast<int64_t>(off))));
    Box3DJointEntry &e = ins.first->second;
    if (ins.second) {
      e.network = myRef.key;
      e.prim = off;
    }
    return e;
  }

  // The type, settings and frame offsets for the primitive.
  JointSpec specFor(const Box3DJointKind &kind, GA_Offset off,
                    b3dop::BodyHandle a, b3dop::BodyHandle b,
                    const b3dop::JointFrame &fa, const b3dop::JointFrame &fb,
                    const b3dop::JointFrame &authA,
                    const b3dop::JointFrame &authB) const {
    JointSpec j;
    if (kind.sideFX) {
      // SideFX data: the primitive's overrides on its options, then mapped onto
      // a Box3D joint.
      b3dop::SideFXSettings sf = kind.sideFXSettings;
      applySideFXOverrides(mySfReaders, kind.sideFXType, off, sf);
      b3dop::SideFXContext ctx;
      ctx.a = sideFXBody(myWorld, a);
      ctx.b = sideFXBody(myWorld, b);
      // From the authored anchors (at rest), so the choice does not change as
      // the pieces move.
      ctx.anchorDistance = static_cast<float>(std::sqrt(
          (authB.pos[0] - authA.pos[0]) * (authB.pos[0] - authA.pos[0]) +
          (authB.pos[1] - authA.pos[1]) * (authB.pos[1] - authA.pos[1]) +
          (authB.pos[2] - authA.pos[2]) * (authB.pos[2] - authA.pos[2])));
      const b3dop::SideFXJoint m =
          b3dop::mapSideFX(kind.sideFXType, sf, myRd.mode(off), ctx);
      j.type = m.type;
      j.s = m.settings;
      j.frameSig = sideFXFrameSignature(m);
      j.reaim = m.reaim;
      j.reaimRest = m.separation;
      // The direction of a shift is fixed at rest (Bullet takes it from the
      // rest transforms), so it comes from the authored anchors, not from where
      // the pieces are now. A joint that holds the pieces where they are keeps
      // the current distance, and one aimed again each frame starts from where
      // the anchors are.
      const bool live = m.holdSeparation || m.reaim;
      b3dop::sideFXFrameOffsets(m, live ? fa : authA, live ? fb : authB, j.adjA,
                                j.adjB);
      if (kind.sideFXType == b3dop::SideFXType::Spring) {
        j.massDependent = true;
        j.bodyA = ctx.a;
        j.bodyB = ctx.b;
      }
      if (kind.sideFXType == b3dop::SideFXType::Glue) {
        j.glue = true;
        j.glueParams.strength = sf.strength;
        j.glueParams.halfLife = sf.impulseHalfLife;
        j.glueParams.rate = sf.propagateRate;
        j.glueParams.iterations = myRd.glueIterations(sf.propagationIterations);
      }
    } else {
      j.type = kind.type;
      j.s = kind.settings;
      applyOverrides(myReaders, j.type, off, kind.attributesReplace, j.s);
      // A distance joint that measured its own rest length keeps it (see
      // writeLiveFrames).
      if (j.type == b3dop::JointType::Distance && !(j.s.restLength >= 0.0f) &&
          myRd.autoRest.isValid()) {
        const float made = myRd.autoRest.get(off);
        if (made >= 0.0f)
          j.s.restLength = made;
      }
      // Hold Separation: the frame offsets SideFX's Glue gets, so the weld
      // holds the anchors as far apart as they are when it is made.
      if (j.type == b3dop::JointType::Weld && j.s.holdSeparation) {
        b3dop::SideFXJoint m;
        m.holdSeparation = true;
        j.frameSig = sideFXFrameSignature(m);
        b3dop::sideFXFrameOffsets(m, fa, fb, j.adjA, j.adjB);
      }
    }
    return j;
  }

  // (Re)make the joint from scratch. Exactly as before from the saved local
  // frames when they still apply (a world rebuilt mid-simulation); else from P
  // / orient, and the local frames are saved after the step. The local frames
  // are the joint's (with any SideFX offset), so they are compared with the
  // frames the anchors give now.
  void makeJoint(Box3DJointEntry &e, const JointSpec &j, const AnchorBody &ab,
                 const AnchorBody &bb, GA_Offset p0, GA_Offset p1,
                 const b3dop::JointFrame &fa, const b3dop::JointFrame &fb,
                 uint64_t sig, bool edited, bool fromLive) {
    const b3dop::BodyHandle a = ab.handle, b = bb.handle;
    dropJoint(e);
    e.type = j.type;
    e.bodyA = a;
    e.bodyB = b;
    e.sig = sig;
    e.settings = j.s;
    e.wrote = false;
    e.adjA = j.adjA;
    e.adjB = j.adjB;
    e.madeRestLength = -1.0f;
    if (j.type == b3dop::JointType::Distance && !(j.s.restLength >= 0.0f)) {
      const double dx = fb.pos[0] - fa.pos[0], dy = fb.pos[1] - fa.pos[1],
                   dz = fb.pos[2] - fa.pos[2];
      e.madeRestLength =
          static_cast<float>(std::sqrt(dx * dx + dy * dy + dz * dz));
    }
    restRotationOf(myWorld, ab, e.restRotA);
    restRotationOf(myWorld, bb, e.restRotB);
    const b3dop::JointFrame ja = jointFrameOf(fa, j.adjA),
                            jb = jointFrameOf(fb, j.adjB);
    b3dop::JointFrame la, lb;
    const bool useLocal = !edited && myRd.savedLocal(p0, la) &&
                          myRd.savedLocal(p1, lb) &&
                          localFrameAgrees(myWorld, a, la, ja) &&
                          localFrameAgrees(myWorld, b, lb, jb);
    // Live frames are where the anchors are now; authored ones are at rest, so
    // a distance joint measuring its own length takes it at rest (what
    // auto_restlength keeps), not between the moved anchors.
    b3dop::JointSettings restSettings = j.s;
    if (e.madeRestLength >= 0.0f)
      restSettings.restLength = e.madeRestLength;
    e.handle =
        useLocal ? myWorld.createJoint(j.type, a, b, la, lb, j.s, &e, true)
        : fromLive
            ? myWorld.createJoint(j.type, a, b, ja, jb, j.s, &e)
            : myWorld.createJoint(j.type, a, b, restToNow(myWorld, ab, ja),
                                  restToNow(myWorld, bb, jb), restSettings, &e);
    e.writeLocal = !useLocal;
    if (e.handle)
      ++myCounts.created;
    else
      ++myCounts.failed;
  }

  // The anchor as authored (P / orient). With an anchor_id on a whole object or
  // a packed piece it sits at that point (or vertex) of the object's geometry
  // (a piece's own geometry), as it was the first time this joint saw it: kept
  // in the entry, so the body moving is no edit. `bad` is set when the id names
  // no point.
  b3dop::JointFrame authoredAnchor(Box3DJointEntry &e, bool second,
                                   GA_Offset pt, const AnchorBody &body,
                                   AnchorLookup &bad) const {
    b3dop::JointFrame f = myRd.authored(pt);
    const int id = myRd.anchorId(pt);
    // A whole object's geometry points, or a packed piece's own geometry's.
    const bool whole = body.entry && body.entry->wholeObject;
    const bool piece = body.entry && body.entry->packed && !whole;
    if (id < 0 || !body.handle || !body.object || !(whole || piece)) {
      // No point to follow (the id is ignored on the world and affectors):
      // forget a pin from an earlier id.
      if (e.extra)
        (second ? e.extra->pinIdB : e.extra->pinIdA) = -1;
      return f;
    }
    Box3DJointExtra &x = e.ensureExtra();
    b3dop::JointFrame &pin = second ? x.pinB : x.pinA;
    int &pinId = second ? x.pinIdB : x.pinIdA;
    bool &pinVertex = second ? x.pinVertexB : x.pinVertexA;
    const bool vertex = myRd.anchorKind(pt) == AnchorKind::Vertex;
    if (pinId != id || pinVertex != vertex) {
      double w[3];
      AnchorLookup found = AnchorLookup::Ok;
      if (whole)
        found = pointAnchorWorld(*body.object, id, vertex, w)
                    ? AnchorLookup::Ok
                    : AnchorLookup::NoPoint;
      else
        found = packedAnchorWorld(*body.object, body.entry->pointOff, id,
                                  vertex, w);
      if (found != AnchorLookup::Ok) {
        bad = found;
        return f;
      }
      // Authored anchors are at rest: a piece that has moved gives the point
      // where it was.
      if (piece)
        nowToRest(myWorld, body, w);
      for (int k = 0; k < 3; ++k)
        pin.pos[k] = w[k];
      pinId = id;
      pinVertex = vertex;
    }
    for (int k = 0; k < 3; ++k)
      f.pos[k] = pin.pos[k];
    return f;
  }

  // The two bodies a primitive joins and its authored anchors, or false when it
  // gets no joint this time (the entry is dropped).
  bool resolveEnds(Box3DJointEntry &e, GA_Offset p0, GA_Offset p1,
                   AnchorBody &ab, AnchorBody &bb, b3dop::JointFrame &authA,
                   b3dop::JointFrame &authB) {
    // Agent anchors name a transform of an agent's rig: nothing here simulates
    // one.
    if (myRd.anchorKind(p0) == AnchorKind::Agent ||
        myRd.anchorKind(p1) == AnchorKind::Agent) {
      ++myCounts.agentAnchors;
      dropJoint(e);
      return false;
    }
    if (!myAnchors.resolve(myRd.anchorName(p0), ab) ||
        !myAnchors.resolve(myRd.anchorName(p1), bb)) {
      ++myCounts.unresolved;
      myNet.needsBodies = true;
      dropJoint(e);
      return false; // not stamped: removed; retried when bodies change
    }
    if (ab.byObjectName || bb.byObjectName)
      myNet.needsBodies = true;
    AnchorLookup badAnchor = AnchorLookup::Ok;
    authA = asStored(authoredAnchor(e, false, p0, ab, badAnchor));
    authB = asStored(authoredAnchor(e, true, p1, bb, badAnchor));
    if (badAnchor != AnchorLookup::Ok) {
      myNet.needsBodies = true;
      ++(badAnchor == AnchorLookup::NoGeometry ? myCounts.anchorNoGeometry
                                               : myCounts.badAnchorId);
      dropJoint(e);
      return false; // not stamped: removed
    }
    return true;
  }

  // What the entry keeps for its kind: a Glue's strength settings (and the
  // impact a rebuilt world carries on from), a Spring's bodies.
  void noteKind(Box3DJointEntry &e, const JointSpec &j, GA_Offset off) {
    e.glue = j.glue;
    e.massDependent = j.massDependent;
    e.reaim = j.reaim;
    e.reaimRest = j.reaimRest;
    if (j.reaim && !e.reaimListed) {
      e.reaimListed = true;
      mySt.reaimEntries.push_back(&e);
    }
    if (j.glue || j.massDependent) {
      Box3DJointExtra &x = e.ensureExtra();
      if (j.glue)
        x.glueParams = j.glueParams;
      if (j.massDependent) {
        x.massA = j.bodyA;
        x.massB = j.bodyB;
      }
      if (j.glue && !e.made && myRd.impact.isValid()) {
        // A world rebuilt mid-simulation: carry on from what the geometry
        // holds.
        const float kept = myRd.impact.get(off);
        x.glueImpact = std::isfinite(kept) && kept > 0.0f ? kept : 0.0f;
      }
      mySt.touchGlueBonds();
    }
  }

  void processPrimitive(const GA_Primitive &prim, GA_Offset off) {
    if (prim.getVertexCount() != 2) {
      ++myCounts.notLines;
      return;
    }
    const Box3DJointKind *kind = myRd.kind(off, myNet.kinds);
    if (!kind) {
      ++myCounts.unknown;
      return;
    }
    if (!kind->sideFX && myRd.narrowsType(off))
      ++myCounts.typeIgnored;

    Box3DJointEntry &e = entryFor(off);

    // Broken (by us, or marked in the geometry): never remade.
    if (e.broken || (myRd.broken && myRd.broken->containsOffset(off))) {
      e.broken = true;
      mySt.touchGlueBonds();
      dropJoint(e);
      e.stamp = myNet.liveStamp;
      return;
    }

    const GA_Offset p0 = prim.getPointOffset(0), p1 = prim.getPointOffset(1);
    AnchorBody ab, bb;
    b3dop::JointFrame authA, authB;
    if (!resolveEnds(e, p0, p1, ab, bb, authA, authB))
      return;
    const b3dop::BodyHandle a = ab.handle, b = bb.handle;
    // On a written network current_P / current_orient say where the anchors
    // are now, so a world rebuilt mid-simulation makes its joints from them
    // exactly as frame 1 does from the authored P / orient.
    b3dop::JointFrame la, lb;
    const bool hasLive = myRd.live(p0, la) && myRd.live(p1, lb);
    // Edited upstream (a SOP Solver): the authored anchors differ from the ones
    // the joint was made from, or the live ones from what was written. Remade
    // from the authored anchors then (current_P is the old joint's).
    const bool authoredEdited =
        e.made && (frameEdited(authA, e.authA) || frameEdited(authB, e.authB));
    const bool liveEdited =
        hasLive && e.wrote &&
        (frameEdited(la, e.wroteA) || frameEdited(lb, e.wroteB));
    const bool useLive = hasLive && !authoredEdited;
    const b3dop::JointFrame fa = useLive ? la : authA,
                            fb = useLive ? lb : authB;
    const JointSpec j = specFor(*kind, off, a, b, fa, fb, authA, authB);
    noteKind(e, j, off);

    const bool live = myNet.writable;
    uint64_t sig = jointSignature(j.type, a, b, fa, fb, !live);
    if (j.frameSig)
      sig = b3dop::mixHash(sig, j.frameSig) | 1;
    const bool edited = live && (authoredEdited || liveEdited);

    if (myWorld.isJointValid(e.handle) && e.sig == sig && e.type == j.type &&
        !edited) {
      if (!b3dop::sameJointSettings(j.type, j.s, e.settings)) {
        myWorld.updateJoint(e.handle, j.type, j.s);
        e.settings = j.s;
        ++myCounts.updated;
      }
    } else {
      makeJoint(e, j, ab, bb, p0, p1, fa, fb, sig, edited, useLive);
      e.authA = authA;
      e.authB = authB;
      e.made = true;
    }
    e.stamp = myNet.liveStamp;
  }

  Box3DSharedState &mySt;
  b3dop::Box3DWorld &myWorld;
  const NetworkRef &myRef;
  Box3DJointNetwork &myNet;
  const Anchors &myAnchors;
  const GU_Detail *myGdp;
  Box3DJointCounts &myCounts;
  const NetworkReader myRd;
  const std::vector<SettingReader> myReaders;
  const std::vector<SideFXReader> mySfReaders;
};

// True if the network's joints were read again (false: left as they are).
bool processNetwork(Box3DSharedState &st, const NetworkRef &ref,
                    const Anchors &anchors, bool massChanged, bool bodyRemoved,
                    Box3DJointCounts &counts) {
  Box3DJointNetwork &net = st.jointNetworks[ref.key];
  net.visited = st.syncStamp;
  net.objectId = ref.object->getObjectId();
  net.relName = ref.source.relName;
  net.inGroup = ref.source.inGroup;

  // The constraint data the network can name; any change re-reads every
  // primitive.
  std::unordered_map<std::string, Box3DJointKind> kinds =
      networkKinds(*ref.object, ref.source);
  const bool kindsChanged = !net.kindsKnown || !sameKinds(kinds, net.kinds);
  if (kindsChanged)
    net.kinds = std::move(kinds);
  net.kindsKnown = true;

  const SIM_Geometry *geo = networkGeometry(*ref.object, ref.source);
  if (!geo)
    return false;
  SIM_GeometryAutoReadLock lock(geo);
  const GU_Detail *gdp = lock.getGdp();
  if (!gdp)
    return false;

  // Unchanged geometry, constraint data and bodies it depends on: the joints
  // stay as they are (they keep liveStamp). Bodies coming or going matter only
  // to a network with anchors that did not resolve, one with a joint on a
  // removed body, and when pieces were renamed or an affector came or went.
  const uint64_t fp = networkFingerprint(gdp);
  const bool bodiesMatter =
      st.allNetworksDirty || bodyRemoved ||
      (net.needsBodies && (st.bodiesAdded || !st.removedBodies.empty())) ||
      net.nameVersion != st.nameVersion;
  if (fp == net.fingerprint && !bodiesMatter && !kindsChanged && !massChanged)
    return false;
  net.fingerprint = fp;
  net.nameVersion = st.nameVersion;
  net.liveStamp = st.syncStamp;

  NetworkPass(st, ref, net, anchors, gdp, counts).run();
  return true;
}

// Takes the bonds that broke out of the glue lists: swapped out of the bond
// list, out of their nodes' partners and bonds, and out of the signature. The
// nodes they joined are kept for regrouping, and a node left without bonds
// stays (its hits are never used) until too many have, then the lists are
// rebuilt.
void removeGlueBonds(Box3DSharedState &st) {
  for (Box3DJointEntry *e : st.glueRemoved) {
    const int i = e->glueIndex;
    if (i < 0 || static_cast<size_t>(i) >= st.glueBonds.size() ||
        st.glueBonds[static_cast<size_t>(i)] != e)
      continue;
    const std::pair<int, int> ab = st.glueBondNodes[static_cast<size_t>(i)];
    Box3DGlueNode *na = &st.glueNodes[static_cast<size_t>(ab.first)];
    Box3DGlueNode *nb = &st.glueNodes[static_cast<size_t>(ab.second)];
    st.glueSignature -= kGlueCountSalt + glueBondHash(na->body, nb->body);
    auto dropFrom = [](Box3DGlueNode &n, b3dop::BodyHandle other,
                       Box3DJointEntry *bond) {
      const auto p = std::find(n.partners.begin(), n.partners.end(), other);
      if (p != n.partners.end())
        n.partners.erase(p);
      const auto b = std::find(n.bonds.begin(), n.bonds.end(), bond);
      if (b != n.bonds.end())
        n.bonds.erase(b);
      return n.bonds.empty();
    };
    if (dropFrom(*na, nb->body, e))
      ++st.glueOrphans;
    if (dropFrom(*nb, na->body, e))
      ++st.glueOrphans;
    st.glueTouched.push_back(ab.first);
    st.glueTouched.push_back(ab.second);
    const size_t last = st.glueBonds.size() - 1;
    if (static_cast<size_t>(i) != last) {
      st.glueBonds[static_cast<size_t>(i)] = st.glueBonds[last];
      st.glueBondNodes[static_cast<size_t>(i)] = st.glueBondNodes[last];
      st.glueBonds[static_cast<size_t>(i)]->glueIndex = i;
    }
    st.glueBonds.pop_back();
    st.glueBondNodes.pop_back();
    e->glueIndex = -1;
  }
  st.glueRemoved.clear();
  st.touchGlueBonds();
  if (st.glueOrphans * 4 > st.glueNodes.size())
    st.glueFull = true;
}

} // namespace

bool anyJointNetwork(SIM_ObjectArray &objects) {
  for (int i = 0; i < objects.entries(); ++i) {
    const SIM_Object *obj = objects(i);
    if (obj->getConstNamedSubData(kInternalGeometry))
      return true;
    for (const bool inGroup : {true, false})
      for (int r = 0; r < obj->getNumRelationships(inGroup); ++r)
        if (networkData(obj->getRelationship(inGroup, r)))
          return true;
  }
  return false;
}

void syncJoints(Box3DSharedState &st, SIM_ObjectArray &objects,
                Box3DJointCounts &counts) {
  // ---- gather: each Constraint Network relationship once (it is on every
  // object it constrains), and each object's internal ConstraintGeometry ----
  std::vector<NetworkRef> refs;
  std::unordered_set<std::string> seenRels;
  for (int i = 0; i < objects.entries(); ++i) {
    SIM_Object *obj = objects(i);
    for (const bool inGroup : {true, false})
      for (int r = 0; r < obj->getNumRelationships(inGroup); ++r) {
        const SIM_Relationship *rel = obj->getRelationship(inGroup, r);
        if (!networkData(rel))
          continue;
        NetworkRef ref;
        ref.object = obj;
        ref.source.inGroup = inGroup;
        const UT_String relName = rel->getName();
        ref.source.relName =
            relName.isstring() ? std::string(relName.buffer()) : std::string();
        if (ref.source.relName.empty() ||
            !seenRels.insert(ref.source.relName).second)
          continue;
        ref.key = b3dop::mixHash(b3dop::hashBytes(b3dop::kHashSeed,
                                                  ref.source.relName.data(),
                                                  ref.source.relName.size()),
                                 1);
        refs.push_back(ref);
      }
    if (obj->getConstNamedSubData(kInternalGeometry)) {
      NetworkRef ref;
      ref.object = obj;
      ref.key = b3dop::mixHash(
          static_cast<uint64_t>(static_cast<uint32_t>(obj->getObjectId())), 2);
      refs.push_back(ref);
    }
  }
  counts.networks = static_cast<int>(refs.size());

  st.glueBroken.clear();

  // Whether any joint may have come or gone since the last sync: a network read
  // again, or one that went away. Without that (and without a joint that broke
  // last step: glueDirty) the removal loop and the Glue graph are left as they
  // are.
  bool changed = false;
  if (!refs.empty()) {
    // A Spring whose bodies' masses changed since it was made is reprocessed
    // (its Hz and damping ratio follow the masses): the networks to read again.
    // Only while there are Springs (counted by the removal loop last step).
    std::unordered_set<uint64_t> massChanged;
    if (st.springEntries > 0)
      for (const auto &kv : st.joints) {
        const Box3DJointEntry &e = kv.second;
        if (e.massDependent && e.extra && !e.broken &&
            st.world.isJointValid(e.handle) &&
            (!sameSideFXBody(e.extra->massA, sideFXBody(st.world, e.bodyA)) ||
             !sameSideFXBody(e.extra->massB, sideFXBody(st.world, e.bodyB))))
          massChanged.insert(e.network);
      }
    // The networks with a joint on a body removed this sync.
    std::unordered_set<uint64_t> removedOn;
    if (!st.removedBodies.empty()) {
      const std::unordered_set<b3dop::BodyHandle> gone(st.removedBodies.begin(),
                                                       st.removedBodies.end());
      for (const auto &kv : st.joints)
        if (gone.count(kv.second.bodyA) || gone.count(kv.second.bodyB))
          removedOn.insert(kv.second.network);
    }
    const Anchors anchors(st, objects);
    for (const NetworkRef &ref : refs)
      changed |=
          processNetwork(st, ref, anchors, massChanged.count(ref.key) != 0,
                         removedOn.count(ref.key) != 0, counts);
  }
  for (const auto &kv : st.jointNetworks)
    changed = changed || kv.second.visited != st.syncStamp;

  // ---- remove: joints whose primitive or network went away; list the Glue
  // bonds and count the Springs of the rest ----
  // Bonds that only broke are taken out of the lists in place.
  if (!changed && !st.glueFull && st.glueDirty && !st.glueRemoved.empty()) {
    st.glueDirty = false;
    st.networkEntriesDirty = true;
    removeGlueBonds(st);
  } else if (changed || st.glueDirty || st.glueFull) {
    st.glueDirty = false;
    st.glueFull = false;
    st.touchGlueBonds();
    st.glueRemoved.clear();
    st.glueTouched.clear();
    st.glueOrphans = 0;
    st.networkEntriesDirty = true; // joints may have come or gone
    st.groupsRecheck = true;       // a piece may have gained or lost a joint
    st.glueNodes.clear();
    st.glueNodeOf.clear();
    st.glueBonds.clear();
    st.glueBondNodes.clear();
    auto nodeOf = [&st](b3dop::BodyHandle h) {
      const auto ins =
          st.glueNodeOf.try_emplace(h, static_cast<int>(st.glueNodes.size()));
      if (ins.second) {
        st.glueNodes.emplace_back();
        st.glueNodes.back().body = h;
      }
      return ins.first->second;
    };
    size_t springs = 0;
    for (auto it = st.joints.begin(); it != st.joints.end();) {
      Box3DJointEntry &e = it->second;
      const auto net = st.jointNetworks.find(e.network);
      const bool live = net != st.jointNetworks.end() &&
                        net->second.visited == st.syncStamp &&
                        e.stamp == net->second.liveStamp;
      e.glueIndex = -1;
      if (live) {
        if (e.massDependent && !e.broken)
          ++springs;
        // Glue to the world is not a bond (Bullet does not support it either).
        if (inGlueGraph(e) && e.extra && !e.broken && e.bodyA && e.bodyB &&
            st.world.isJointValid(e.handle)) {
          const int a = nodeOf(e.bodyA), b = nodeOf(e.bodyB);
          st.glueNodes[static_cast<size_t>(a)].partners.push_back(e.bodyB);
          st.glueNodes[static_cast<size_t>(b)].partners.push_back(e.bodyA);
          e.glueIndex = static_cast<int>(st.glueBonds.size());
          st.glueNodes[static_cast<size_t>(a)].bonds.push_back(&e);
          st.glueNodes[static_cast<size_t>(b)].bonds.push_back(&e);
          st.glueBonds.push_back(&e);
          st.glueBondNodes.emplace_back(a, b);
        }
        ++it;
        continue;
      }
      if (st.world.isJointValid(e.handle)) {
        st.world.destroyJoint(e.handle);
        ++counts.removed;
      }
      if (e.reaimListed)
        st.reaimEntries.erase(
            std::remove(st.reaimEntries.begin(), st.reaimEntries.end(), &e),
            st.reaimEntries.end());
      it = st.joints.erase(it);
    }
    st.springEntries = springs;
    st.glueSignature = st.computeGlueSignature();
  }
  for (auto it = st.jointNetworks.begin(); it != st.jointNetworks.end();)
    it = (it->second.visited == st.syncStamp) ? std::next(it)
                                              : st.jointNetworks.erase(it);
}

void reaimJoints(Box3DSharedState &st) {
  // An entry whose joint is gone or no longer re-aimed leaves the list.
  st.reaimEntries.erase(
      std::remove_if(st.reaimEntries.begin(), st.reaimEntries.end(),
                     [&](Box3DJointEntry *e) {
                       const bool keep = e->reaim && !e->broken &&
                                         st.world.isJointValid(e->handle);
                       if (!keep && !e->reaim)
                         e->reaimListed = false;
                       return !e->reaim;
                     }),
      st.reaimEntries.end());
  for (Box3DJointEntry *e : st.reaimEntries) {
    if (e->broken || !st.world.isJointValid(e->handle))
      continue;
    // A joint nothing moves is left alone (moving its frame would wake it).
    if (!((e->bodyA && st.world.isAwakeByHandle(e->bodyA)) ||
          (e->bodyB && st.world.isAwakeByHandle(e->bodyB))))
      continue;
    double shift[3] = {e->adjA.pos[0], e->adjA.pos[1], e->adjA.pos[2]};
    if (st.world.reaimJointFrameA(e->handle, e->reaimRest,
                                  b3dop::kSideFXAimMinDistance, shift))
      std::copy(shift, shift + 3, e->adjA.pos);
  }
}
