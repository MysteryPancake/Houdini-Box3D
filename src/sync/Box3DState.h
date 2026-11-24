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

#include "core/Box3DBondNetwork.h"
#include "core/Box3DCompound.h"
#include "core/Box3DGlue.h"
#include "core/Box3DShapes.h"
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

  // The fields are in the order the per-step passes touch them: the pose and
  // the hot fields first (the write-back, forces, Impacts and the sync's quiet
  // check read these for every moved body, in Box3D's order rather than the
  // slab's), the settings the body was built from last. Keep new per-step
  // fields in the first part.

  // ---- hot: read or written every step ----

  // The body's handle and the type the solver last set.
  b3dop::BodyHandle handle = 0;
  b3dop::BodySettings::Type type = b3dop::BodySettings::Type::Dynamic;
  int objectId = 0;
  uint64_t stamp = 0; // syncStamp of the last sync that saw it
  // Where this frame's merged move event of the body is in
  // Box3DSharedState::moves, valid when moveStamp is the frame's syncStamp
  // (merging the events of a frame's steps).
  uint64_t moveStamp = 0;
  size_t moveSlot = 0;
  // Its object's record (stable: objectSync is node-based).
  Box3DObjectSync *sync = nullptr;

  // The body's point, refreshed by every sync, so the write-back needs no
  // lookup. Box3D's body user data points at this entry, so entries never move
  // (Box3DMirrorMap keeps them in a deque) and are only erased with their
  // body.
  int64_t pointOff = -1;   // GA_Offset of the point in the object's geometry
  int64_t pointId = 0;     // the `id` attribute, else the point number
  int64_t pointIndex = -1; // point number at the last scan (Impacts data)
  int64_t primOff = -1;    // GA_Offset of the point's packed primitive

  bool awake =
      true; // the value of `awake` as last read from or written to the point
  bool packed = false;
  bool hasRest = false;
  bool wholeObject = false; // see below
  // Glued into a compound body (Box3DCompoundSync.cpp): this entry's own body
  // is disabled and the compound moves it. glueEdited: another node changed
  // the point's pose, so the compound is dissolved before the next step.
  bool inCompound = false;
  bool glueEdited = false;
  // The `compound_id` last queued for the point (-1: none shown).
  int compoundShown = -1;
  bool animated = false;  // inactive + `animated`: kinematic, follows its point
  bool driven = false;    // given a target last step (velocity to clear)
  bool shapeless = false; // `none`, or a rebuild left no shape: always static
  bool meshShapes = false; // a mesh was built (static until rebuilt)

  // Packed: the rotation of the point's `restxform` (read when the body was
  // made, or the rotation it was made with when the solver sets restxform), so
  // the write-back can write `orient` as the rotation since rest.
  float restRot[4] = {0, 0, 0, 1};
  // The point's packed primitive's 3x3 as the last sync read it (reused by the
  // write-back), with the 3x3's per-row scale, which the write-back keeps.
  double rowScale[3] = {1, 1, 1};
  // Whether that 3x3 mirrors (its determinant is negative): composing a new one
  // keeps it. The write-back never changes it, and every scan sets it again.
  bool mirrored = false;
  // This step's velocity of the piece, from its compound (the write-back reads
  // it in place of its own, parked body's).
  float glueLin[3] = {0, 0, 0};
  float glueAng[3] = {0, 0, 0};

  // ---- cold: what the body was built from, read when a scan compares ----

  // The settings the body was built with or last given (attribute, else the
  // Box3D Data value), so a scan calls only the setters for what differs.
  b3dop::BodyProps props;
  float volume = 0.0f;    // total shape volume
  uint64_t shapeSig = 0;  // a rebuild giving the same signature is a no-op
  bool autoScale = false; // convex / concave: rebuilt when the scale changes
  double sizeKey[3] = {1, 1, 1}; // the scale the shapes were built at
  // `concave` shapes depend on the body type: a static body gets a triangle
  // mesh, any other hulls. plannedStatic is the type they were built for;
  // meshShapes says a mesh was built (the body must then stay static until its
  // shapes are rebuilt).
  bool concave = false;
  bool plannedStatic = false;

  // A whole DOP object as one body (RBD Object etc., see syncWholeObject): no
  // point; its pose lives in the object's Position data, about `pivot` (object
  // space). (`wholeObject` is above.)
  double pivot[3] = {0, 0, 0};
  // What the pivot was computed from (0: it was not), so it is not recomputed
  // while the geometry is unchanged.
  uint64_t pivotKey = 0;
  // The same for the bounding box sizes bounding-box inertia uses.
  uint64_t boundsKey = 0;
  float boundsExtents[3] = {0, 0, 0};

  // restPos: the body's origin at rest (the packed pivot through restxform).
  double restPos[3] = {0, 0, 0};

  bool dynamic() const {
    return type == b3dop::BodySettings::Type::Dynamic;
  } // written back
};

// The body mirror: entries in a slab of fixed-size chunks (entries never move,
// which the cached Box3DPointSlot::entry pointers and Box3D's body user data
// rely on; a std::deque would put each of these large entries in a block of its
// own) with a free list, so live entries stay together in memory and iteration
// walks the slab. A key -> slot index table (open addressing, linear probing,
// no allocation per entry) serves the lookups that miss the cache. An entry
// lives until erase(); its slot is then reset and reused.
class Box3DMirrorMap {
public:
  struct Ref {
    Box3DBodyMirror *entry = nullptr;
    uint32_t index = 0;    // for erase()
    bool inserted = false; // new, default-constructed
  };

  Box3DMirrorMap() = default;
  Box3DMirrorMap(const Box3DMirrorMap &) = delete;
  Box3DMirrorMap &operator=(const Box3DMirrorMap &) = delete;
  ~Box3DMirrorMap() {
    for (size_t i = 0; i < used_; ++i)
      at(i).~Slot();
    for (Slot *chunk : chunks_)
      std::allocator<Slot>().deallocate(chunk, kChunk);
  }

  size_t size() const { return count_; }

  // A rough heap size: the entries, the free list and the key table.
  size_t bytes() const {
    return chunks_.size() * kChunk * sizeof(Slot) +
           free_.capacity() * sizeof(uint32_t) +
           table_.capacity() * sizeof(Bucket);
  }

  // Room for `n` entries without growing the table or the slab. A big slab is
  // memory nothing has touched yet, so `parallel(count, f)`, which runs f(i)
  // for every i < count (on several threads if it likes), allocates its chunks
  // and touches their pages: the first touch is then not one thread's. The
  // entries are made when they are handed out, so they are in the cache then.
  template <class P> void reserve(size_t n, const P &parallel) {
    growFor(n);
    const size_t want = (n + kChunk - 1) / kChunk;
    if (want <= chunks_.size())
      return;
    const size_t first = chunks_.size();
    chunks_.resize(want, nullptr);
    parallel(want - first, [&](size_t i) {
      Slot *chunk = std::allocator<Slot>().allocate(kChunk);
      char *bytes = reinterpret_cast<char *>(chunk);
      for (size_t off = 0; off < kChunk * sizeof(Slot); off += 4096)
        bytes[off] = 0;
      chunks_[first + i] = chunk;
    });
  }
  void reserve(size_t n) {
    reserve(n, [](size_t count, const auto &f) {
      for (size_t i = 0; i < count; ++i)
        f(i);
    });
  }

  Box3DBodyMirror *find(uint64_t key) {
    const uint32_t s = lookup(key);
    return s == kEmpty ? nullptr : &at(s).entry;
  }
  const Box3DBodyMirror *find(uint64_t key) const {
    const uint32_t s = lookup(key);
    return s == kEmpty ? nullptr : &at(s).entry;
  }

  // The entry for `key`, inserted (default-constructed) if missing.
  Ref tryEmplace(uint64_t key) {
    growFor(count_ + 1);
    const size_t mask = table_.size() - 1;
    size_t b = hashOf(key) & mask;
    Ref r;
    while (table_[b].slot != kEmpty) {
      if (table_[b].key == key) {
        r.index = table_[b].slot;
        r.entry = &at(r.index).entry;
        return r;
      }
      b = (b + 1) & mask;
    }
    if (!free_.empty()) {
      r.index = free_.back();
      free_.pop_back();
    } else {
      r.index = static_cast<uint32_t>(used_);
      if (used_ == chunks_.size() * kChunk)
        chunks_.push_back(std::allocator<Slot>().allocate(kChunk));
      new (&chunks_[used_ / kChunk][used_ % kChunk]) Slot();
      ++used_;
    }
    table_[b] = {key, r.index};
    ++count_;
    Slot &s = at(r.index);
    s.key = key;
    s.live = true;
    r.entry = &s.entry;
    r.inserted = true;
    return r;
  }

  void erase(uint32_t index) {
    Slot &s = at(index);
    removeKey(s.key);
    s.entry = Box3DBodyMirror(); // so a reused slot starts clean
    s.live = false;
    free_.push_back(index);
  }

  // Erases every entry for which f(entry) is true; returns how many.
  template <class F> size_t eraseIf(F &&f) {
    size_t n = 0;
    for (size_t i = 0; i < used_; ++i)
      if (at(i).live && f(at(i).entry)) {
        erase(static_cast<uint32_t>(i));
        ++n;
      }
    return n;
  }

private:
  struct Slot {
    Box3DBodyMirror entry;
    uint64_t key = 0;
    bool live = false;
  };
  static constexpr size_t kChunk = 1024;  // entries per chunk
  static constexpr uint32_t kEmpty = ~0u; // an unused bucket
  struct Bucket {
    uint64_t key = 0;
    uint32_t slot = kEmpty;
  };

  static size_t hashOf(uint64_t key) {
    return static_cast<size_t>(b3dop::mixHash(b3dop::kHashSeed, key));
  }

  uint32_t lookup(uint64_t key) const {
    if (table_.empty())
      return kEmpty;
    const size_t mask = table_.size() - 1;
    for (size_t b = hashOf(key) & mask; table_[b].slot != kEmpty;
         b = (b + 1) & mask)
      if (table_[b].key == key)
        return table_[b].slot;
    return kEmpty;
  }

  // The table holds at most 70% full buckets: grown (to a power of two) when
  // `n` entries would pass that.
  void growFor(size_t n) {
    if (!table_.empty() && n * 10 <= table_.size() * 7)
      return;
    size_t cap = 16;
    while (n * 10 > cap * 7)
      cap *= 2;
    if (cap <= table_.size())
      return;
    std::vector<Bucket> old;
    old.swap(table_);
    table_.assign(cap, Bucket());
    const size_t mask = cap - 1;
    for (const Bucket &o : old) {
      if (o.slot == kEmpty)
        continue;
      size_t b = hashOf(o.key) & mask;
      while (table_[b].slot != kEmpty)
        b = (b + 1) & mask;
      table_[b] = o;
    }
  }

  // Takes `key` out and shifts the entries after it back, so no probe run is
  // broken and no tombstone is left.
  void removeKey(uint64_t key) {
    if (table_.empty())
      return;
    const size_t mask = table_.size() - 1;
    size_t i = hashOf(key) & mask;
    while (table_[i].slot != kEmpty && table_[i].key != key)
      i = (i + 1) & mask;
    if (table_[i].slot == kEmpty)
      return;
    for (size_t j = i;;) {
      j = (j + 1) & mask;
      if (table_[j].slot == kEmpty)
        break;
      const size_t home = hashOf(table_[j].key) & mask;
      // An entry whose home lies cyclically in (i, j] stays where it is.
      const bool stays =
          i <= j ? (i < home && home <= j) : (i < home || home <= j);
      if (stays)
        continue;
      table_[i] = table_[j];
      i = j;
    }
    table_[i] = Bucket();
    --count_;
  }

  Slot &at(size_t i) { return chunks_[i / kChunk][i % kChunk]; }
  const Slot &at(size_t i) const { return chunks_[i / kChunk][i % kChunk]; }

  std::vector<Slot *> chunks_; // raw storage; the first used_ slots are made
  size_t used_ = 0;            // slots handed out (live or on the free list)
  std::vector<uint32_t> free_;
  std::vector<Bucket> table_;
  size_t count_ = 0;
};

// A point's `restxform` split into its rotation (restRotation), kept on its
// slot while the attribute's data id is unchanged. `ok` false: the matrix was
// unset or degenerate.
struct Box3DRestCache {
  bool known = false;
  bool ok = false;
  float rot[4] = {0, 0, 0, 1};
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
  Box3DRestCache rest;
  bool ok = false;          // readPointState succeeded (P is finite)
  bool badVelocity = false; // v or w was not finite (read as zero)
  bool active = true;
  bool animated = false; // inactive and `animated`: a kinematic body
  bool packed = false;   // the point belongs to a packed primitive
  // The point belongs to a primitive that is not packed (a polygon's point):
  // no body, as RBD Bullet Solver (docs/agents/bodies.md, "Unpacked geometry").
  bool unpackedPrim = false;
  int64_t index = 0; // point number
  int64_t primOff = -1;
  double m3[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  double rowScale[3] = {1, 1, 1}; // m3's per-row scale
  // This scan's shape decisions (pass 1) and pre-built shape plan (-1: none).
  bool deforming = false;   // `deforming`
  bool refitWanted = false; // bullet_autofit_valid == 0
  bool rebuild = false;     // an existing body whose shapes are rebuilt
  bool staticBody = false;  // the body wanted is static (concave: a mesh)
  bool dead =
      false; // `dead` != 0, read once in pass 1 (pass 1b and pass 2 reuse it)
  // Index into this scan's shape plans and their packed primitives
  // (SlotPlans in Box3DPointSync.cpp).
  int32_t plan = -1;
  // Index into this scan's settings read in the parallel pass
  // (SlotPlans::props), -1: not read.
  int32_t props = -1;
  // The parallel pass found nothing to do for the point and has stamped its
  // entry already (stampQuietSlots): the apply pass only lists it.
  bool quiet = false;
};

// One awake body's result on its way to the geometry. Its pose is the move
// event (Box3DSharedState::moves[move]).
struct Box3DWriteItem {
  GA_Offset off = GA_INVALID_OFFSET;
  Box3DBodyMirror *entry = nullptr;
  int move = 0;             // index into Box3DSharedState::moves
  float lin[3] = {0, 0, 0}; // velocities (not in move events)
  float ang[3] = {0, 0, 0};
  int objectId = 0;
  int8_t awakeState = -1; // `awake` to write: -1 none, 0 asleep, 1 awake
};

// A point attribute written after the step, for any body (not only awake ones).
struct Box3DPointWrite {
  enum : unsigned { kAutofitValid = 1, kMass = 2, kRest = 4, kCompound = 8 };
  GA_Offset off = GA_INVALID_OFFSET;
  Box3DBodyMirror *entry = nullptr;
  int objectId = 0;
  unsigned flags = 0;
  float mass = 0.0f;   // kMass: computed mass (computemass)
  int compoundId = -1; // kCompound: the piece's `compound_id`
  // kRest: a new packed body's `restxform` (its packed transform when it was
  // made, row-major) and `pivot` (its P then), as Bullet initializes them.
  float restxform[16] = {};
  float pivot[3] = {};
};

// A `dead` point to delete after the step (its body is already gone).
struct Box3DDeletion {
  int objectId = 0;
  GA_Offset off = GA_INVALID_OFFSET;
  int64_t pointId = 0;
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
  // `restxform` data id the slots' rest rotations were split at (kNoId: none).
  GA_DataId restCacheId = GA_DataId(-1);
  GA_DataId forceId = GA_DataId(-1);
  GA_DataId torqueId = GA_DataId(-1);

  // Bodies by their point's `name` (first point wins), for joint anchors.
  // Rebuilt by each full scan while some object has a joint network
  // (Box3DSharedState::wantNames).
  std::unordered_map<std::string, Box3DBodyMirror *> byName;
  bool namesKnown = false; // byName describes the current entries
  // Order-sensitive hash of the (name, entry) pairs byName was built from; 0:
  // byName is not built. A scan that hashes the same leaves byName untouched.
  uint64_t nameSig = 0;
  // Scratch of the scan in progress: its named points (sorted by finish).
  std::vector<std::pair<GA_Offset, Box3DBodyMirror *>> namedPoints;
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
  // A mesh body follows a moved pose as a kinematic body, so it pushes what it
  // touches: retyped on its first move. Height fields and grounds are
  // teleported.
  bool movable = false;
  bool kinematic = false;
  bool driven = false; // given a target last step (velocity to clear)
  // Height fields: the heights' hash, and the geometry identity it was taken
  // from (the heights are read again only when that changes).
  uint64_t heightSource = 0;
  uint64_t heightHash = 0;
  bool heightsKnown = false;
};

// What a Bond shows on its line, when the solver's Write Bond Attributes
// is on: the attribute for each, in this order.
enum BondReadout {
  kReadoutRatio,   // the stress ratio: 1 is at its capacity
  kReadoutDamage,  // the impulse taken as a share of the tear impulse (0 to 1)
  kReadoutTension, // N along the normal (negative: pushed together)
  kReadoutShear,   // N across it
  kReadoutBending, // N*m about axes across the normal
  kReadoutTwist,   // N*m about the normal
  kReadoutArea,    // m^2 of the patch
  kReadoutCount
};
inline constexpr const char *kBondReadoutNames[kReadoutCount] = {
    "bond_ratio",   "bond_damage", "bond_tension", "bond_shear",
    "bond_bending", "bond_twist",  "bond_area"};

// The point attribute of a glued piece: its compound (Write Bond Attributes).
inline constexpr const char *kCompoundIdName = "compound_id";

struct BondReadouts {
  float value[kReadoutCount] = {};
  float written[kReadoutCount];
  BondReadouts() { std::fill(std::begin(written), std::end(written), -1.0f); }
  // Something to write: a value differs from the one on the line.
  bool pending() const {
    return !std::equal(std::begin(value), std::end(value), std::begin(written));
  }
};

// The parts of a joint entry that few joints use (Box3DJointEntry::extra).
struct Box3DJointExtra {
  // A point / vertex anchor (anchor_id) on a whole object or packed piece:
  // where that point was the first time this joint saw it, so the body moving
  // is not an edit. pin*Id is the id it was taken for (-1: none), pin*Vertex
  // whether a vertex.
  b3dop::JointFrame pinA, pinB;
  int pinIdA = -1, pinIdB = -1;
  bool pinVertexA = false, pinVertexB = false;
  // SideFX Glue: the bond's options, and its accumulated `impact` (the
  // attribute of the same name), kept from step to step.
  b3dop::GlueParams glueParams;
  float glueImpact = 0.0f;
  // SideFX Spring: the masses its Hz and damping ratio were converted with.
  b3dop::SideFXBody massA, massB;
  // A joint that broke: its local frames at the break, so a next constraint can
  // start from them (switchFrames).
  b3dop::JointFrame brokeLocalA, brokeLocalB;
  // Bond (core/Box3DBond.h): the normal from piece A to piece B in body A's
  // frame and the patch area, found on the first step after it is made, the
  // impulse its load has delivered beyond its capacity (it breaks at the tear
  // impulse), and whether it is queued to break.
  float bondNormalLocal[3] = {0, 0, 0};
  float bondAreaUsed = 0.0f;
  float bondImpulse = 0.0f;
  // The load was over the capacity when last looked at: damage goes on while
  // the pieces sleep, under the load they last had.
  bool bondOverCapacity = false;
  // The readouts of the last load (BondReadout), and what was last written to
  // each attribute (-1: nothing yet).
  BondReadouts bondReadouts;
  // The mass of what the bond holds together on each side (infinity: a fixed
  // body), found for the glue graph `bondSidesSignature` was taken at.
  float bondSideMass[2] = {0.0f, 0.0f};
  uint64_t bondSidesSignature = 0;
  bool bondSidesKnown = false;
  bool bondReady = false;
  // The Contact Area setting the geometry was found with: found again when it
  // changes.
  float bondAreaSetting = 0.0f;
  bool bondQueued = false;
  // A bond inside a compound has no joint to read: the load its compound's
  // spring network gave it (the force and couple on piece B, world axes) and
  // the `hitsTick` it was worked out at.
  float bondForce[3] = {0, 0, 0};
  float bondTorque[3] = {0, 0, 0};
  uint64_t bondLoadTick = 0;
  // Where piece A and the anchor are for a bond inside a compound, from the
  // compound's pose after the step (the parked piece bodies are placed later,
  // so they are a step behind), at `bondPoseTick`.
  double bondPosA[3] = {0, 0, 0};
  float bondRotA[4] = {0, 0, 0, 1};
  double bondAnchor[3] = {0, 0, 0};
  uint64_t bondPoseTick = 0;
  // The contact patch of the two pieces (core/Box3DPatch.h), when their convex
  // faces could be clipped: its centroid and in-plane axes in body A's frame,
  // and its section moduli (bending about each axis, twisting about the
  // normal). Without it the normal runs centre to centre and the patch is a
  // square of the area.
  bool patchOk = false;
  float patchCentroidLocal[3] = {0, 0, 0};
  float patchULocal[3] = {1, 0, 0};
  float patchVLocal[3] = {0, 1, 0};
  float modulusU = 0.0f;
  float modulusV = 0.0f;
  float modulusTwist = 0.0f;
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
  // Both bodies asleep (or fixed) when the write-back last visited the joint,
  // and then already written: its live frames and feedback are not written
  // again until a body wakes. `quiet` is this step's decision.
  bool asleepWritten = false;
  bool quiet = false;
  // distance made with restlength < 0: written to `auto_restlength` once
  float madeRestLength = -1.0f;
  // made from P / orient: write local_P / local_orient after the step
  bool writeLocal = false;

  // next_constraint_name: the broken joint's local frames, so the anchors can
  // be put where its frames were at the break (switchFrames) and the next
  // constraint made exactly from there.
  // (the frames themselves are in `extra`)
  bool brokeLocalKnown = false;
  bool switchFrames = false;

  // SideFX data only: the joint's frames relative to its anchors
  // (b3dop::sideFXFrameOffsets), so the anchors written back are the anchors,
  // not the frames. Identity for Box3D nodes.
  b3dop::JointFrame adjA, adjB;

  // Each body's rotation at rest (its restxform's, else its rotation when the
  // joint was made; identity for the world), for the `angle` reading: the
  // bodies' relative rotation since rest, as Bullet.
  float restRotA[4] = {0, 0, 0, 1};
  float restRotB[4] = {0, 0, 0, 1};

  // SideFX Glue: breaks on the impulse of the hits its bodies take, not on a
  // load (core/Box3DGlue.h, docs/agents/joints-sidefx.md); its bond and
  // `impact` are in `extra`.
  bool glue = false;
  // Its place in Box3DSharedState::glueBonds, -1 when it is not a live bond.
  int glueIndex = -1;
  // A Glue bond between two pieces of one compound (Box3DCompoundSync.cpp):
  // the weld carries nothing, so its force, torque, distance and angle are
  // written as zero once and only `impact` after that. feedbackZeroed /
  // impactWritten say what was written.
  bool inCompound = false;
  bool feedbackZeroed = false;
  float impactWritten = -1.0f;
  // SideFX Spring: its Hz and damping ratio come from the bodies' masses
  // (Hooke's constant and absolute damping converted, b3dop::mapSideFX), so the
  // masses they were made from are kept (in `extra`) and the joint is remade
  // when they change.
  bool massDependent = false;

  // What only some joints keep, made on first use so the others do not carry
  // it (about a quarter of an entry).
  std::unique_ptr<Box3DJointExtra> extra;
  Box3DJointExtra &ensureExtra() {
    if (!extra)
      extra = std::make_unique<Box3DJointExtra>();
    return *extra;
  }
};

// Whether the joint is an edge of the glue graph (the groups that run as one
// body, the contacts inside a group that are no hits): a SideFX Glue bond, or a
// Bond (which never breaks by the Glue's impulse: its strength there is < 0).
inline bool inGlueGraph(const Box3DJointEntry &e) {
  return e.glue || e.type == b3dop::JointType::Bond;
}

// A body in a Glue network this frame and the impulse its contacts put on it
// over the frame's Box3D steps.
struct Box3DGlueNode {
  b3dop::BodyHandle body = 0;
  float hits = 0.0f;
  std::vector<b3dop::BodyHandle> partners; // bodies it is glued to
  std::vector<Box3DJointEntry *> bonds;    // the live bonds it is in
};

// The constraint data a `constraint_name` refers to (a Box3D constraint
// relationship node, or SideFX constraint data mapped onto a Box3D joint per
// primitive), as last used.
struct Box3DJointKind {
  b3dop::JointType type = b3dop::JointType::Weld;
  b3dop::JointSettings settings;
  // Attributes Replace Values on the node: float primitive attributes are the
  // value instead of a factor (b3dop::applyFloatAttribute).
  bool attributesReplace = false;
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
  // Box3DSharedState::nameVersion when last processed
  uint64_t nameVersion = 0;
  // Some primitive's anchors did not resolve (or resolve by an object's name):
  // bodies coming or going can change what it makes. Else only the removal of
  // a body one of its joints is on, or a rename, can.
  bool needsBodies = false;
  uint64_t liveStamp = 0; // stamp its current entries carry
  uint64_t visited = 0;   // last sync that saw it
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

// One piece of a compound: its own body (disabled while merged) and where it
// sits in the compound body's space. `entry` is refreshed by every
// syncCompounds before use; key / body / props / shapeSig are what the piece
// was merged with, so a change is seen.
struct Box3DCompoundMember {
  uint64_t key = 0; // makeBodyKey of the entry
  b3dop::BodyHandle body = 0;
  Box3DBodyMirror *entry = nullptr;
  float lp[3] = {0, 0, 0};     // position in the compound body's space
  float lq[4] = {0, 0, 0, 1};  // rotation in the compound body's space
  float centre[3] = {0, 0, 0}; // the piece's centre of mass, in its own space
  float mass = 0.0f;           // the piece's own mass
  float radius = 0.0f;         // half the diagonal of its bounds
  int shapeFirst = 0;          // its shapes are [shapeFirst, +shapeCount) in
  int shapeCount = 0;          // the compound's, in creation order
  b3dop::BodyProps props;      // as merged
  uint64_t shapeSig = 0;       // as merged
};

// A Glue bond between a piece of a compound and a piece that is not in it (one
// taken out while a fast body comes through, `Box3DSharedState::glueHold`). The
// bond's own weld is parked with the piece that is in the compound, so this
// weld joins the free piece to the compound's body at that piece's place in
// it. `original` is the bond's joint, which wakes by itself when its piece
// leaves the compound.
struct Box3DBridge {
  b3dop::JointHandle original = 0;
  b3dop::JointHandle handle = 0;
  b3dop::BodyHandle member = 0; // the piece in the compound
};

// A glued group run as one Box3D body (core/Box3DCompound.h).
struct Box3DCompound {
  b3dop::BodyHandle body = 0;
  std::vector<Box3DCompoundMember> members;
  std::vector<Box3DBridge> bridges;
  // The impulse on each of its shapes in the last Box3D step read for the glue
  // (contactImpulsesByShape), the collectGlueHits call it belongs to, and
  // whether the body was awake and dynamic then (a sleeping one has no hits).
  std::vector<float> shapeImpulse;
  // Where its body is after the last step, read once before the live frames
  // are written (refreshLivePoses).
  b3dop::BodyPose livePose;
  bool livePoseOk = false;
  uint64_t hitsTick = 0;
  bool hitsActive = false;
  // For the inertial load (glueCompoundLoad): its motion before and after the
  // last Box3D step, and its pose after it, read once per step
  // (loadTick: the collectGlueHits call they belong to).
  b3dop::MergedBody stepPre, stepPost;
  bool stepPreOk = false;
  b3dop::BodyPose postPose;
  uint64_t loadTick = 0;
  // The last bond network solve's displacement (6 per member), the next one's
  // starting guess. Empty when none, or when the members changed.
  std::vector<double> bondWarm;
  // What the last bond network solve was given and found, kept while the next
  // asks for nearly the same (b3dop::bondProblemAlike): then its loads stand.
  struct BondMemo {
    bool valid = false;
    std::vector<b3dop::BondNode> nodes;
    std::vector<b3dop::BondLink> links;
    std::vector<b3dop::BondSupport> supports;
    std::vector<Box3DJointEntry *> used; // the bond of each link
    std::vector<b3dop::BondForce> out;
  } bondMemo;
};

// What one bond adds to the signature of the bond list (kGlueCountSalt for the
// bond itself): the same whichever way round its two bodies are.
constexpr uint64_t kGlueCountSalt = 0x9E3779B97F4A7C15ull;
inline uint64_t glueBondHash(b3dop::BodyHandle bodyA, b3dop::BodyHandle bodyB) {
  uint64_t a = static_cast<uint64_t>(bodyA), b = static_cast<uint64_t>(bodyB);
  if (a > b)
    std::swap(a, b);
  return b3dop::mixHash(b3dop::mixHash(0x5bd1e995ull, a), b);
}

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
  // The bodies a live Bond ties to the world (such a Bond is not in the graph):
  // a side holding one does not move.
  std::unordered_set<b3dop::BodyHandle> glueAnchored;
  // The signature computeBondSides last ran for (every bond's sides at once).
  uint64_t bondSidesPass = ~0ull;
  uint64_t computeGlueSignature() const {
    uint64_t sum = static_cast<uint64_t>(glueBondNodes.size()) * kGlueCountSalt;
    for (const auto &ab : glueBondNodes)
      sum += glueBondHash(glueNodes[static_cast<size_t>(ab.first)].body,
                          glueNodes[static_cast<size_t>(ab.second)].body);
    for (const b3dop::BodyHandle h : glueAnchored)
      sum += glueBondHash(h, 0);
    return sum;
  }
  std::vector<void *> glueBroken;
  bool glueDirty = true;
  // The joint entries of each network (key: network key) for jointsAfterStep,
  // rebuilt only after syncJoints changed the joints.
  std::unordered_map<uint64_t, std::vector<Box3DJointEntry *>> networkEntries;
  bool networkEntriesDirty = true;
  // The Bonds among them, in the same order, so breakBonds does not walk the
  // other joints every step. Rebuilt with networkEntries.
  std::vector<Box3DJointEntry *> bondEntries;
  // Glued groups as compound bodies (syncCompounds): the solver's switch, the
  // live compounds, each piece's own body -> its compound and index, and what
  // the groups were last worked out from (the graph and body versions), so an
  // unchanged scene skips the work.
  // The bodies on a joint that is not a glue bond, as of the last full
  // regrouping (tiedKnown): partial regrouping after bond breaks reuses it.
  std::unordered_set<b3dop::BodyHandle> tiedBodies;
  bool tiedKnown = false;
  bool glueCompounds = false;
  // Write each Bond's attributes onto its line, and `compound_id` onto the
  // points of glued pieces.
  bool bondReadouts = false;
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
  // A Bond is among the graph's bonds (the compounds then record their motion
  // for the bonds' loads).
  bool bondsInGraph = false;
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
