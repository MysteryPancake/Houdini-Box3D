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

#include <array>
#include <cstdint>
#include <iterator>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "core/Box3DSideFX.h"
#include "core/Box3DWorld.h"
#include "sync/Box3DAttribNames.h"

// One body's pose and velocity as last given to / read from Box3D (DOP terms).
// Compared with the point to detect edits by other nodes.
struct Box3DBodyPose {
  double pos[3] = {0, 0, 0};
  float rot[4] = {0, 0, 0, 1}; // x, y, z, w
  float lin[3] = {0, 0, 0};
  float ang[3] = {0, 0, 0}; // radians per second
};

struct Box3DObjectSync;

struct Box3DBodyMirror : Box3DBodyPose {
  // Overwrites the pose part only (position, orientation, velocities), leaving
  // the rest as is.
  void setPose(const Box3DBodyPose &pose) {
    static_cast<Box3DBodyPose &>(*this) = pose;
  }

  // The settings the body was built with or last given (attribute, else the
  // Box3D Data value), so a scan calls only the setters for what differs.
  b3dop::BodyProps props;
  float volume = 0.0f;    // total shape volume
  uint64_t shapeSig = 0;  // a rebuild giving the same signature is a no-op
  bool shapeless = false; // `none`, or a rebuild left no shape: always static
  bool autoScale = false; // convex / concave: rebuilt when the scale changes
  double sizeKey[3] = {1, 1, 1}; // the scale the shapes were built at

  bool animated = false; // inactive + `animated`: kinematic, follows its point
  bool driven = false;   // given a target last step (velocity to clear)
  bool awake =
      true; // the value of `awake` as last read from or written to the point
  int64_t pointIndex = -1; // point number at the last scan (Impacts data)

  // A whole DOP object as one body (RBD Object etc., see syncWholeObject): no
  // point; its pose lives in the object's Position data, about `pivot` (object
  // space).
  bool wholeObject = false;
  double pivot[3] = {0, 0, 0};

  // The body's point, refreshed by every sync, so the write-back needs no
  // lookup. Box3D's body user data points at this entry, so entries never move
  // (unordered_map nodes) and are only erased with their body.
  int64_t pointOff = -1; // GA_Offset of the point in the object's geometry
  int64_t pointId = 0;   // the `id` attribute, else the point number
  int objectId = 0;
  uint64_t stamp = 0; // syncStamp of the last sync that saw it
  Box3DObjectSync *sync =
      nullptr; // its object's record (stable: objectSync is node-based)

  // The point's packed primitive and its 3x3 as the last sync read it (reused
  // by the write-back), with the 3x3's per-row scale, which the write-back
  // keeps.
  bool packed = false;
  int64_t primOff = -1; // GA_Offset of the primitive
  double m3[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  double rowScale[3] = {1, 1, 1};

  // The body's handle and the type the solver last set.
  b3dop::BodyHandle handle = 0;
  b3dop::BodySettings::Type type = b3dop::BodySettings::Type::Dynamic;

  bool dynamic() const {
    return type == b3dop::BodySettings::Type::Dynamic;
  } // written back
};

// One point as syncObject() reads it. Kept between steps; `entry` caches the
// mirror entry while the point's offset and key are unchanged (all slots are
// dropped when an entry is erased).
struct Box3DPointSlot {
  GA_Offset off = GA_INVALID_OFFSET;
  int64_t bid = 0;
  uint64_t key = 0;
  Box3DBodyMirror *entry = nullptr;
  Box3DBodyPose cur;
  bool ok = false;          // readPointState succeeded (P is finite)
  bool badVelocity = false; // v or w was not finite (read as zero)
  bool active = true;
  bool packed = false; // the point belongs to a packed primitive
  int64_t index = 0;   // point number
  int64_t primOff = -1;
  double m3[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  double rowScale[3] = {1, 1, 1}; // m3's per-row scale
  // This scan's shape decisions (pass 1) and pre-built shape plan (-1: none).
  bool deforming = false;   // `deforming`
  bool refitWanted = false; // bullet_autofit_valid == 0
  bool rebuild = false;     // an existing body whose shapes are rebuilt
  bool dead =
      false; // `dead` != 0, read once in pass 1 (pass 1b and pass 2 reuse it)
  int32_t plan = -1;
};

// One awake body's result on its way to the geometry. Its pose is the move
// event (Box3DSharedState::moves[move]); packed matrices live in a side array
// (Box3DPackedWrite).
struct Box3DWriteItem {
  GA_Offset off = GA_INVALID_OFFSET;
  Box3DBodyMirror *entry = nullptr;
  int move = 0;             // index into Box3DSharedState::moves
  float lin[3] = {0, 0, 0}; // velocities (not in move events)
  float ang[3] = {0, 0, 0};
  int objectId = 0;
  int packedIdx = -1;     // index into Box3DSharedState::packedWrites, or -1
  int8_t awakeState = -1; // `awake` to write: -1 none, 0 asleep, 1 awake
};

// A point attribute written after the step, for any body (not only awake ones).
struct Box3DPointWrite {
  enum : unsigned { kAutofitValid = 1, kMass = 2 };
  GA_Offset off = GA_INVALID_OFFSET;
  Box3DBodyMirror *entry = nullptr;
  int objectId = 0;
  unsigned flags = 0;
  float mass = 0.0f; // kMass: computed mass (computemass)
};

// A `dead` point to delete after the step (its body is already gone).
struct Box3DDeletion {
  int objectId = 0;
  GA_Offset off = GA_INVALID_OFFSET;
  int64_t pointId = 0;
};

// A packed rotation computed in the write-back's parallel pass, written in its
// serial pass.
struct Box3DPackedWrite {
  bool has = false; // newM3 / primOff were filled in
  GA_Offset primOff = GA_INVALID_OFFSET;
  double newM3[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
};

// Change markers of an object's geometry when the solver last finished with it
// (attribute data ids, counts, node values). An identical fingerprint next step
// means the scan can be skipped.
struct Box3DSyncFingerprint {
  static constexpr GA_DataId kNoId = GA_DataId(-1); // attribute absent

  // Prop markers (kPropAttribNames in Box3DAttribNames.h, primsId, nodeProps,
  // addImpact): when any changed, the scan also compares every body's settings.
  // The count comes from that table.
  static constexpr int kNumPropIds =
      static_cast<int>(std::size(box3dsolver::kPropAttribNames));
  using PropIds = std::array<GA_DataId, kNumPropIds>;

  int64_t numPoints =
      -1; // -1 with primitives: packed topology is assumed fixed
  int64_t numOffsets = -1;
  GA_DataId pId = kNoId, orientId = kNoId, vId = kNoId, wId = kNoId;
  GA_DataId idId = kNoId, activeId = kNoId;
  int nodeType = -1; // Box3D Data body type
  PropIds propIds = filledPropIds();
  GA_DataId primsId = kNoId;  // primitive list (packed transforms)
  b3dop::BodyProps nodeProps; // Box3D Data values that change existing bodies
  bool addImpact = false;     // Box3D Data "Add Impact Data"

  static PropIds filledPropIds() {
    PropIds ids;
    ids.fill(kNoId);
    return ids;
  }
  bool samePropMarkers(const Box3DSyncFingerprint &o) const {
    return propIds == o.propIds && primsId == o.primsId &&
           nodeProps == o.nodeProps && addImpact == o.addImpact;
  }
  bool operator==(const Box3DSyncFingerprint &o) const {
    return numPoints == o.numPoints && numOffsets == o.numOffsets &&
           pId == o.pId && orientId == o.orientId && vId == o.vId &&
           wId == o.wId && idId == o.idId && activeId == o.activeId &&
           nodeType == o.nodeType && samePropMarkers(o);
  }
};

// Per object. A skipped sync leaves its entries untouched, so "current" is the
// object's liveStamp rather than the step's stamp.
struct Box3DObjectSync {
  uint64_t liveStamp = 0;  // stamp the object's current entries carry
  uint64_t visited = 0;    // last sync that saw the object
  size_t entryCount = 0;   // entries stamped by the last full scan
  bool canSkip = false;    // fp describes a clean, fully scanned object
  bool propsKnown = false; // fp's prop markers are recorded
  bool wholeObject =
      false; // the object is one body (syncWholeObject), not one per point
  bool anyDeforming = false; // deforming bodies: never skip
  bool anyDriven =
      false; // a moved animated body (velocity to clear): never skip
  bool addImpact = false; // Box3D Data "Add Impact Data"
  // The point attributes last reported as overriding this object's Box3D Data
  // (noteOverriddenData): reported again only when the set changes.
  std::string overridesReported;
  std::vector<Box3DBodyMirror *>
      bodies; // every body of the object (rebuilt by each full scan)
  // `force` / `torque` data ids right after the solver last zeroed them (or
  // found them all zero): unchanged ids mean nothing new to apply.
  GA_DataId forceId = GA_DataId(-1);
  GA_DataId torqueId = GA_DataId(-1);

  // Bodies by their point's `name` (first point wins), for joint anchors.
  // Rebuilt by each full scan while some object has a joint network
  // (Box3DSharedState::wantNames).
  std::unordered_map<std::string, Box3DBodyMirror *> byName;
  bool namesKnown = false; // byName describes the current entries
  Box3DSyncFingerprint fp;
};

// A static collider made from an affector object (an object that is not solved
// by us but affects our objects through a collision relationship, e.g. a Static
// Object DOP). One body, one triangle mesh.
struct Box3DAffectorBody {
  b3dop::BodyHandle handle =
      0; // 0: the last build failed (not retried until the signature changes)
  uint64_t stamp = 0; // syncStamp that last saw the object as an affector
  uint64_t signature =
      0; // geometry + material + baked transform the shape was built from
  std::string name; // the affector object's name (joint anchors can name it)
  double pos[3] = {0, 0, 0}; // the body's pose as last given to Box3D
  float rot[4] = {0, 0, 0, 1};
  float scale[3] = {1, 1, 1}; // the mesh shape's scale as last given to Box3D
  float rolling =
      0.0f; // rolling resistance / conveyor velocity as last given to Box3D
  float tangent[3] = {
      0, 0,
      0}; // (not in the signature: animating them must not rebuild the mesh)
  bool poseKnown = false;
};

// One joint made from a primitive of a constraint network (Box3DJointSync.cpp).
struct Box3DJointEntry {
  b3dop::JointHandle handle =
      0; // 0: none (broken, or its anchors did not resolve)
  b3dop::JointType type = b3dop::JointType::Weld;
  b3dop::BodyHandle bodyA = 0; // 0: the world
  b3dop::BodyHandle bodyB = 0;
  uint64_t sig = 0; // type, bodies, frames: a change recreates the joint
  b3dop::JointSettings settings; // as last given to Box3D
  uint64_t network = 0;          // key of its Box3DJointNetwork
  GA_Offset prim = GA_INVALID_OFFSET;
  uint64_t stamp = 0;  // its network's liveStamp when last seen
  bool broken = false; // broke (force / torque threshold): never remade

  // Live frames: after each step the anchors' current_P / current_orient are
  // set to where the joint's frames are (written networks only). What was
  // written, to tell our writes from edits upstream.
  b3dop::JointFrame wroteA, wroteB;
  bool wrote = false;
  // The authored anchors (P / orient, as stored) the joint was made from: an
  // upstream edit of them remakes the joint.
  b3dop::JointFrame authA, authB;
  bool made = false;
  // distance made with restlength < 0: written to `auto_restlength` once
  float madeRestLength = -1.0f;
  // made from P / orient: write local_P / local_orient after the step
  bool writeLocal = false;

  // next_constraint_name: the broken joint's local frames, so the anchors can
  // be put where its frames were at the break (switchFrames) and the next
  // constraint made exactly from there.
  b3dop::JointFrame brokeLocalA, brokeLocalB;
  bool brokeLocalKnown = false;
  bool switchFrames = false;

  // SideFX data only: the joint's frames relative to its anchors
  // (b3dop::sideFXFrameOffsets), so the anchors written back are the anchors,
  // not the frames. Identity for Box3D nodes.
  b3dop::JointFrame adjA, adjB;
};

// The constraint data a `constraint_name` refers to (a Box3D constraint
// relationship node, or SideFX constraint data mapped onto a Box3D joint per
// primitive), as last used.
struct Box3DJointKind {
  b3dop::JointType type = b3dop::JointType::Weld;
  b3dop::JointSettings settings;
  bool sideFX =
      false; // SideFX data: type / settings come from mapSideFX per primitive
  b3dop::SideFXType sideFXType = b3dop::SideFXType::Glue;
  b3dop::SideFXSettings sideFXSettings;
};

// A constraint network: the two-point primitives of a Constraint Network DOP's
// relationship (or an object's ConstraintGeometry, with Attach Internal
// Constraints to Object).
struct Box3DJointNetwork {
  uint64_t fingerprint = 0; // geometry identity + data ids of everything read
  uint64_t bodyVersion = 0; // Box3DSharedState::bodyVersion when last processed
  uint64_t liveStamp = 0;   // stamp its current entries carry
  uint64_t visited = 0;     // last sync that saw it
  int objectId = -1; // an object it was found on (and is written back through)
  // the relationship's name; empty: the object's ConstraintGeometry
  std::string relName;
  bool inGroup =
      true; // found among the object's group (not affector) relationships
  bool writable = true; // the last write-back reached the geometry
  std::unordered_map<std::string, Box3DJointKind>
      kinds; // by data name, as last used
  bool kindsKnown = false;
};

// Everything shared between timestep copies.
struct Box3DSharedState {
  b3dop::Box3DWorld world;
  std::unordered_map<uint64_t, Box3DBodyMirror>
      mirror; // key: makeBodyKey(object id, point id)

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
  // this solve's objects are in id order (box3dObjectById may binary search)
  bool objectsSorted = false;
  uint64_t bodyVersion = 0; // bumped whenever bodies are added or removed

  // Scratch reused every step.
  std::unordered_map<int, std::vector<Box3DPointSlot>> slots; // key: object id
  std::vector<Box3DWriteItem> writeItems;
  std::vector<Box3DPackedWrite> packedWrites;
  std::vector<b3dop::BodyMove>
      moves; // this step's move events (read once after the step)
  std::vector<b3dop::ContactPoint> contacts;
  // Filled by the sync (in object order), used after the step:
  std::vector<Box3DPointWrite> pointWrites;
  std::vector<Box3DDeletion> deletions;
  std::vector<int>
      impactObjects; // objects with Impacts data from last step (sorted)

  float stepDt = 0.0f; // this step's Box3D time step (DOP step * Time Scale)

  double stampTime = 0.0; // SIM_Engine::getSimulationTime() at the last solve
  bool stamped = false;

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
