// Box3DBodyState.h
//
// The solver's per-body state: the body mirror (what a point's body was last
// given), the per-object sync records and fingerprint, point slots and queued
// writes, and affector bodies.
#pragma once

#include <GA/GA_Types.h>
#include <array>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "core/Box3DCompound.h"
#include "core/Box3DGlue.h"
#include "core/Box3DMotion.h"
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
  // A point `force` / `torque` was applied to it in the last step that visited
  // it (Box3DForces.cpp: the body is woken once when it ends).
  bool pointForced = false;
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

// One point's drag and speed limits (Box3DMotion.cpp), kept for the points that
// have any.
struct Box3DMotionEntry {
  GA_Offset off = 0;
  b3dop::MotionParams params;
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
  // Some body has `pointForced`: the object is visited for forces even when it
  // has none this step.
  bool pointForcedAny = false;
  // The points with a drag or a speed limit (airresist ... speedmax), sorted
  // by offset, read again when the attributes' data ids change.
  std::vector<Box3DMotionEntry> motion;
  uint64_t motionSig = 0;
  // Some point has a non-zero airresist, spinresist, targetv or targetw: they
  // are inputs for one step, zeroed after it (as Bullet's clearForces).
  bool motionInputs = false;

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
