// Box3DState.h
//
// The solver's state, shared between timestep copies of SIM_Box3DWorld: the
// body mirror, per-object sync records, affector bodies, joint entries and the
// per-step scratch arrays. Everything here is a rebuildable cache of what the
// DOP objects hold; nothing in it is a SIM_Data (the DOP class that owns it is
// dop/SIM_Box3DWorld.h). The state's stamp (sim time of the last step) lives
// here, not in the copies, so an older timestep cannot claim the world is
// current.
#pragma once

#include <GA/GA_Types.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cstdint>
#include <deque>
#include <iterator>
#include <memory>
#include <mutex>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "core/Box3DCompound.h"
#include "core/Box3DGlue.h"
#include "core/Box3DShapes.h"
#include "core/Box3DSideFX.h"
#include "core/Box3DWorld.h"
#include "sync/Box3DAttribNames.h"
#include "sync/Box3DBodyState.h"
#include "sync/Box3DCompoundState.h"
#include "sync/Box3DJointState.h"

// One contact recorded for the Impacts data. `fromOther`: cp was read from the
// other body's side.
struct Box3DImpactHit {
  const Box3DBodyMirror *self, *other;
  b3dop::ContactPoint cp;
  bool fromOther;
  uint64_t key; // impactKey, set only while the frame's steps are merged
};

// One slot of the table collectImpacts uses to find, in a later substep, the
// hit an earlier one already made for the same contact (open addressing; key 0
// is an empty slot).
struct Box3DImpactSlot {
  uint64_t key = 0;
  uint32_t group = 0, hit = 0;
};

// The impact hits of one object this step (see recordImpacts).
struct Box3DImpactGroup {
  int objectId = 0;
  std::vector<Box3DImpactHit> hits;
};

// Export Collision Shapes: one body's outline in body space (xyz per point;
// `idx` the points of every polyline, `ends` the cumulative count of it after
// each) and where its points and polylines sit in the object's exported
// geometry.
struct Box3DShapeExportBody {
  std::vector<float> pts;
  std::vector<int> idx;
  std::vector<uint32_t> ends;
  uint32_t pointStart = 0, lineStart = 0;
};

// What the last full export of one object wrote. `sig` says which bodies and
// shapes it was made for (also stored in the geometry, which is what decides
// whether the lines can be moved instead of rebuilt).
struct Box3DShapeExport {
  uint64_t sig = 0;
  uint32_t points = 0, lines = 0;
  std::vector<Box3DShapeExportBody> bodies;
};

// A piece's pose in its source SOP, as last seen (Box3DSourceSync.cpp).
struct Box3DSourcePose {
  double pos[3] = {0, 0, 0};
  float rot[4] = {0, 0, 0, 1};
};

// What followSourceSops keeps per object so a step where nothing changed does
// not walk every point again (Box3DSourceSync.cpp).
struct Box3DSourceFollower {
  GA_Offset off = GA_INVALID_OFFSET;
  std::string name;
  float rest[4] = {0, 0, 0, 1}; // the rotation of its restxform
};

struct Box3DSourceCache {
  // The pieces that follow, and the markers of the geometry they were read
  // from (not P or orient: the solver writes those every step).
  bool followersKnown = false;
  uint64_t followersKey = 0;
  std::vector<Box3DSourceFollower> followers;
  std::unordered_map<std::string, bool> wanted; // their names
  // The source SOP's point for each wanted name, and the markers of the
  // source it was read from.
  bool indexKnown = false;
  uint64_t indexKey = 0;
  std::unordered_map<std::string, GA_Offset> index;
};

// Everything shared between timestep copies.
struct Box3DSharedState {
  b3dop::Box3DWorld world;
  Box3DMirrorMap mirror; // key: makeBodyKey(object id, point id)
  // Per object, by piece name: the pose its source SOP had when last read. A
  // piece is moved only when this changes, so other edits (a SOP Solver) stand.
  std::unordered_map<int, std::unordered_map<std::string, Box3DSourcePose>>
      sourcePoses;
  std::unordered_map<int, Box3DSourceCache> sourceCaches; // per object

  // Bumped once per sync; stampedCount current entries (equal to mirror.size():
  // nothing to remove).
  uint64_t syncStamp = 0;
  size_t stampedCount = 0;

  std::unordered_map<int, Box3DObjectSync>
      objectSync; // per object seen by the last sync
  std::unordered_map<int, Box3DAffectorBody>
      affectors; // key: object id of the affector

  // Joints (Box3DJointSync.cpp). Entries are joint user data, so nodes must not
  // move.
  std::unordered_map<uint64_t, Box3DJointEntry>
      joints; // key: network key + primitive offset
  std::unordered_map<uint64_t, Box3DJointNetwork>
      jointNetworks; // key: relationship name (or object id)
  // some object has a constraint network: keep Box3DObjectSync::byName
  bool wantNames = false;
  // solver Reap Dead Points: `dead` points are deleted (else kept, out of the
  // simulation)
  bool reapDead = true;
  // solver Reap At Frame End: points that die during the step (aging) are
  // deleted at its end; else kept this frame and deleted by the next step
  bool reapAtFrameEnd = true;
  // this solve's objects are in id order (box3dObjectById may binary search)
  bool objectsSorted = false;
  uint64_t bodyVersion = 0; // bumped whenever bodies are added or removed
  // Objects the sync scanned in full this step (an object whose markers are
  // unchanged is skipped, and none of its bodies can differ from the mirror).
  uint32_t fullScans = 0;
  // This sync's changes of the body set, for the constraint networks: handles
  // of the bodies removed, whether any was added, whether an affector came or
  // went (every network is read again), and a counter of renamed pieces.
  std::vector<b3dop::BodyHandle> removedBodies;
  bool bodiesAdded = false;
  bool allNetworksDirty = false;
  uint64_t nameVersion = 0;

  // Scratch reused every step.
  std::unordered_map<int, std::vector<Box3DPointSlot>> slots; // key: object id
  std::vector<Box3DWriteItem> writeItems;
  std::vector<b3dop::BodyMove>
      moves; // this step's move events (read once after the step)
  // Substeps: one step's events and broken joints, and each body's slot in
  // `moves` while the frame's steps are merged. Kept so nothing is allocated
  // per frame.
  std::vector<b3dop::BodyMove> stepEvents;
  std::vector<void *> stepBroken;
  std::unordered_map<b3dop::BodyHandle, size_t> moveSlots;
  // The frame's moves came from one step's events as they are (mergeMoveEvents)
  // and their bodies have not noted their slot yet.
  bool moveSlotsPending = false;
  std::vector<b3dop::ContactImpulse> impulseEvents; // this step's, for Impacts
  // recordImpacts scratch: the first `impactGroupCount` entries are in use this
  // step. Entries (and their hit vectors) are kept so capacity is reused.
  std::vector<Box3DImpactGroup> impactGroups;
  size_t impactGroupCount = 0;
  std::vector<Box3DImpactSlot> impactSlots; // see Box3DImpactSlot
  // Export Collision Shapes: the outlines of each object's bodies, by object
  // id (exportCollisionShapes).
  std::unordered_map<int, Box3DShapeExport> shapeExports;
  // The last step wrote the exported shapes, for the bodies of bodyVersion
  // `shapesVersion`: a step with no move, no scan and the same bodies leaves
  // them as they are.
  bool shapesFresh = false;
  uint64_t shapesVersion = 0;
  // Filled by the sync (in object order), used after the step:
  std::vector<Box3DPointWrite> pointWrites;
  std::vector<Box3DDeletion> deletions;
  // Objects whose Geometry this step's own writes touched (noteOwnWrite).
  std::vector<int> writtenObjects;
  std::vector<int>
      impactObjects; // objects with Impacts data from last step (sorted)

  float stepDt = 0.0f; // this step's Box3D time step (DOP step * Time Scale)
  // Enable Aging: the time step added to each constraint's `age` this step (the
  // DOP step, not Time Scale-adjusted), 0 when aging is off.
  float constraintAgeDt = 0.0f;
  // Joints that went over their threshold in any of this frame's Box3D steps
  // (Substeps > 1): Box3D clears its joint events every step, so the last
  // step's would otherwise be all breakJoints saw. Used only when
  // brokenCollected is set.
  std::vector<void *> brokenThisFrame;
  bool brokenCollected = false;
  // Glue (syncJoints, collectGlueHits, advanceGlue in Box3DJointSync.cpp): the
  // live bonds and their bodies this frame, and the bonds that went over their
  // strength (joint user data, broken by jointsAfterStep with the force ones).
  // The graph is rebuilt only when a network was read again, a network went
  // away or a joint broke (glueDirty); glueBondNodes holds each bond's two
  // node indices, parallel to glueBonds.
  std::vector<Box3DGlueNode> glueNodes;
  std::unordered_map<b3dop::BodyHandle, int> glueNodeOf;
  // The glue group (the pieces joined by bonds) of each node, which
  // collectGlueHits keeps contacts inside from counting as hits. Worked out
  // again when the bond list or the node count changed (a bond broken in the
  // middle of a frame splits its group from the next frame).
  std::vector<int> glueGroup;
  uint64_t glueGroupSignature = 0;
  size_t glueGroupNodes = 0;
  bool glueGroupKnown = false;
  std::vector<Box3DJointEntry *> glueBonds;
  std::vector<std::pair<int, int>> glueBondNodes;
  // The bonds that broke since the lists were made: syncJoints takes them out
  // of the lists (and the signature) in place instead of rebuilding, and the
  // nodes they joined (glueTouched) are the components syncCompounds works out
  // again. glueFull: a change the lists cannot follow (a joint that is not a
  // bond broke, many nodes lost all their bonds): rebuild them.
  std::vector<Box3DJointEntry *> glueRemoved;
  std::vector<int> glueTouched;
  bool glueFull = false;
  size_t glueOrphans = 0; // nodes that lost every bond, kept until a rebuild
  // The hash of the bond list (computeGlueSignature), kept up to date.
  uint64_t glueSignature = 0;
  // advanceGlue's copy of the glue bonds (their nodes, strength settings and
  // impact), kept between frames so the frame does not walk every bond's entry
  // to make it. It stays good while glueEpoch is the one it was made at: every
  // place that adds, removes, breaks or remakes a bond, or sets its settings or
  // impact, calls touchGlueBonds. glueMirrorQuiet: no bond has an impact (or
  // a NaN) after the last advance, so with no hits either the frame is idle.
  std::vector<b3dop::GlueBond> glueMirror;
  uint64_t glueEpoch = 0;
  uint64_t glueMirrorEpoch = ~0ull;
  bool glueMirrorQuiet = false;
  void touchGlueBonds() { ++glueEpoch; }
  uint64_t computeGlueSignature() const {
    uint64_t sum = static_cast<uint64_t>(glueBondNodes.size()) * kGlueCountSalt;
    for (const auto &ab : glueBondNodes)
      sum += glueBondHash(glueNodes[static_cast<size_t>(ab.first)].body,
                          glueNodes[static_cast<size_t>(ab.second)].body);
    return sum;
  }
  std::vector<void *> glueBroken;
  bool glueDirty = true;
  // The joint entries of each network (key: network key) for jointsAfterStep,
  // rebuilt only after syncJoints changed the joints.
  std::unordered_map<uint64_t, std::vector<Box3DJointEntry *>> networkEntries;
  bool networkEntriesDirty = true;
  // Glued groups as compound bodies (syncCompounds): the solver's switch, the
  // live compounds, each piece's own body -> its compound and index, and what
  // the groups were last worked out from (the graph and body versions), so an
  // unchanged scene skips the work.
  // The bodies on a joint that is not a glue bond, as of the last full
  // regrouping (tiedKnown): partial regrouping after bond breaks reuses it.
  std::unordered_set<b3dop::BodyHandle> tiedBodies;
  bool tiedKnown = false;
  bool glueCompounds = false;
  // Write `compound_id` onto the points of glued pieces.
  bool compoundIds = false;
  // The pieces whose point shows a `compound_id` (keys into `mirror`).
  std::vector<uint64_t> compoundIdKeys;
  // Glued compounds: scales the load slowing the whole body puts on each
  // piece's bonds (0: off). See compoundGlueHits.
  float glueCompoundLoad = 0.0f;
  std::vector<std::unique_ptr<Box3DCompound>> compounds;
  std::unordered_map<b3dop::BodyHandle, std::pair<Box3DCompound *, int>>
      compoundOf;
  uint64_t hitsTick = 0; // bumped by every collectGlueHits
  uint64_t groupsSignature =
      ~static_cast<uint64_t>(0); // of the Glue graph, as grouped
  uint64_t groupsBodyVersion = ~static_cast<uint64_t>(0);
  bool groupsRecheck = true;
  // The pieces taken out of their compounds for a fast body coming at them
  // (holdCompoundsForApproach), with the glueClock time each may merge again.
  // glueClock is the simulated time, advanced a frame at a time by
  // releaseExpiredHolds.
  b3dop::GlueHolds glueHold;
  double glueClock = 0.0;
  // Each compound's body -> the glue group its pieces are in, for contacts
  // with a compound of the same group (collectGlueHits makes it).
  std::unordered_map<b3dop::BodyHandle, int> compoundGroup;
  // A compound lost or gained pieces while some are held: its bridges
  // (Box3DBridge) are worked out again.
  bool bridgesDirty = false;
  // The weakest break impulse of any bond (the Glue's Strength over
  // kGlueStrengthScale; infinity: none can break), worked out again once per
  // glueClock value that is asked for it: what a body has to be able to deliver
  // to be looked at at all.
  float glueMinBreak = 0.0f;
  double glueMinBreakClock = -1.0;
  uint64_t glueMinBreakSignature = 0;
  // Live SideFX Spring joints last step: syncJoints looks for changed masses
  // only while there are any.
  size_t springEntries = 0;
  // The same for hit events (Impact Min Speed): hitPairs already holds every
  // Box3D step of this frame, so recordImpacts must not refill it from the
  // last.
  bool hitsCollected = false;
  // Glue hits are read from the world's impulse events (one pass over the
  // step's contact points) instead of asking each glued body for its contacts:
  // set by the solver before it steps, while there are glued bodies.
  bool glueHitsFromEvents = false;
  // The frame's impact hits were collected after each Box3D step already
  // (Substeps > 1; collectImpacts): the impulses of every step add up, where
  // reading the contacts after the last alone found a resting body's.
  bool impactsCollected = false;
  // The frame has more than one Box3D step: the hits carry their `key`, so a
  // later step can find them.
  bool impactKeys = false;

  double stampTime = 0.0; // SIM_Engine::getSimulationTime() at the last solve
  bool stamped = false;

  // The size reported to the cache (SIM_Box3DWorld::getMemorySizeSubclass),
  // set after each solve: the timestep copies share this state, so only the
  // one that stepped it last (memoryOwner) reports it.
  std::atomic<size_t> memoryBytes{0};
  std::atomic<const void *> memoryOwner{nullptr};

  // A rough size of everything this state holds: Box3D's world, the per-body,
  // per-object and per-joint records, and the scratch kept between steps.
  size_t estimatedBytes() const {
    constexpr size_t kNode = 2 * sizeof(void *); // a hash node's links
    auto mapBytes = [](const auto &m) {
      using Value = typename std::decay_t<decltype(m)>::value_type;
      return m.size() * (sizeof(Value) + kNode) +
             m.bucket_count() * sizeof(void *);
    };
    size_t n = world.estimatedBytes() + mirror.bytes() + mapBytes(objectSync) +
               mapBytes(affectors) + mapBytes(joints) +
               mapBytes(jointNetworks) + mapBytes(slots);
    for (const auto &kv : objectSync) {
      const Box3DObjectSync &os = kv.second;
      n += os.bodies.capacity() * sizeof(Box3DBodyMirror *) +
           mapBytes(os.byName) + os.overridesReported.capacity();
    }
    for (const auto &kv : slots)
      n += kv.second.capacity() * sizeof(Box3DPointSlot);
    return n + writeItems.capacity() * sizeof(Box3DWriteItem) +
           moves.capacity() * sizeof(b3dop::BodyMove) +
           stepEvents.capacity() * sizeof(b3dop::BodyMove) +
           impulseEvents.capacity() * sizeof(b3dop::ContactImpulse) +
           impactGroups.capacity() * sizeof(Box3DImpactGroup) +
           pointWrites.capacity() * sizeof(Box3DPointWrite) +
           deletions.capacity() * sizeof(Box3DDeletion);
  }

  std::mutex mutex;

  // Current: its object was visited by the latest sync and it carries the
  // object's liveStamp. (m.sync is valid here: records are only erased after
  // their unvisited bodies are removed.)
  bool isLive(const Box3DBodyMirror &m) const {
    return m.sync && m.sync->visited == syncStamp &&
           m.stamp == m.sync->liveStamp;
  }

  // Last stepped exactly one timestep before `now` (as a difference, so it does
  // not matter whether the engine reports the start or end of the step).
  bool canContinue(double now, double timestep) const {
    if (!stamped)
      return false;
    const double tol = 1e-3 * (timestep > 0.0 ? timestep : 1.0);
    const double d = (now - stampTime) - timestep;
    return (d < 0 ? -d : d) <= tol;
  }
};
