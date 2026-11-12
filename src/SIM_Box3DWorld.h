// SIM_Box3DWorld.h
//
// SIM_Data that owns the live Box3D world.
//
// The world is a REBUILDABLE CACHE, not authoritative state. The authoritative
// state lives in the DOP objects (position, orientation, velocities, ...).
//
// How it survives DOPs copying data:
//   * DOPs share data between timesteps and copy it (makeEqual) on the first
//     non-const access. The solver writes every step, so a copy happens every
//     step. If makeEqualSubclass reset the world we would rebuild every step.
//   * So the copy shares a std::shared_ptr<Box3DSharedState>. Every timestep's
//     SIM_Box3DWorld that came from the same lineage points at the same state.
//   * The state records the sim time the world was last stepped at (its
//     "stamp"). The stamp lives IN the shared state, not in each copy: if it
//     lived in the copy, an older timestep's data would keep claiming the
//     world was current after a newer timestep stepped it.
//   * The solver continues with the world only if the stamp says "I was last
//     stepped exactly one timestep ago". Anything else (first frame, scrub
//     back, re-solve of the same step, reload from sim cache, ...) builds a NEW
//     state and swaps the pointer in this data only. Other timesteps keep
//     pointing at the old state, which stays alive until the last one goes.
#pragma once

#include <SIM/SIM_Data.h>
#include <SIM/SIM_DataUtils.h>

#include <GA/GA_Types.h>

#include <cstdint>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <vector>

#include "Box3DWorld.h"

// What we last handed to / read back from Box3D for one body (one POINT of a
// DOP object), in DOP terms. Compared against the point's current values each
// step to detect edits made by other nodes (the stamp alone cannot see those).
struct Box3DBodyPose
{
    double pos[3] = { 0, 0, 0 };
    float  rot[4] = { 0, 0, 0, 1 };   // x, y, z, w
    float  lin[3] = { 0, 0, 0 };
    float  ang[3] = { 0, 0, 0 };      // radians per second
};

struct Box3DBodyMirror : Box3DBodyPose
{
    bool   dynamic = false;           // a body the solver writes back (not static)

    // Where this body's point is, refreshed by every sync. The write-back reads
    // Box3D's move events (one per awake body) and finds the point from here, so
    // it needs no lookup by key. Box3D's body user data points at this entry,
    // which is why entries must not move: they live in an unordered_map (nodes
    // are stable across inserts) and are only erased together with their body.
    int64_t  pointOff = -1;           // GA_Offset of the point in the object's geometry
    int64_t  pointId  = 0;            // the `id` attribute, else the point number
    int      objectId = 0;
    uint64_t stamp    = 0;            // Box3DSharedState::syncStamp of the last sync that saw it

    // The packed primitive this point belongs to, as the last sync read it: the 3x3
    // of its full transform. Write-back reuses it instead of reading the primitive
    // again (the geometry does not change between the two).
    bool     packed   = false;
    int64_t  primOff  = -1;           // GA_Offset of the primitive
    double   m3[9]    = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

    // Cached so the per-step loops need no hash lookups into the Box3D world:
    // the body's handle, and the body type the solver last set (the same value
    // Box3DWorld::getBodyType would return). Valid for the life of the entry:
    // the mirror is per world, and an entry is erased when its body is removed.
    b3dop::BodyHandle        handle = 0;
    b3dop::BodySettings::Type type  = b3dop::BodySettings::Type::Dynamic;
};

// One point of an object's geometry as syncObject() reads it. The vector of
// these is kept between steps (Box3DSharedState::slots) so a step allocates
// nothing, and `entry` caches the point's mirror entry so a steady-state step does
// no hash lookups. A cached entry is only trusted while the point's offset and
// body key are unchanged; the slots are all dropped whenever an entry is erased.
struct Box3DPointSlot
{
    GA_Offset         off    = GA_INVALID_OFFSET;
    int64_t           bid    = 0;
    uint64_t          key    = 0;
    Box3DBodyMirror  *entry  = nullptr;
    Box3DBodyPose     cur;
    bool              ok     = false;    // readPointState succeeded (P is finite)
    bool              active = true;
    bool              packed = false;    // the point belongs to a packed primitive
    int64_t           primOff = -1;
    double            m3[9]  = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };
};

// One awake body's result on its way from Box3D to the geometry (writeResultsBack).
struct Box3DWriteItem
{
    GA_Offset         off      = GA_INVALID_OFFSET;
    Box3DBodyMirror  *entry    = nullptr;       // not copied
    Box3DBodyPose     s;                        // what Box3D reports now
    int               objectId = 0;
    bool              hasM3    = false;         // packed rotation to write in the serial pass
    GA_Offset         primOff  = GA_INVALID_OFFSET;
    double            newM3[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };
};

// What an object's geometry looked like when the solver last finished with it. If the
// next step finds exactly the same fingerprint, nothing the sync pass looks at can have
// changed, and the per-point scan is skipped. Every field is a "did anything touch this"
// marker: attribute data ids are bumped by any write to the attribute (the solver bumps
// P / orient / v / w after its own write-back, and records the fingerprint after that).
struct Box3DSyncFingerprint
{
    static constexpr GA_DataId kNoId = GA_DataId(-1);   // attribute absent

    int64_t   numPoints  = -1;      // -1 for geometry with primitives: packed instances are assumed
    int64_t   numOffsets = -1;      // to keep their topology (a new instance gets a new `id`)
    GA_DataId pId = kNoId, orientId = kNoId, vId = kNoId, wId = kNoId;
    GA_DataId idId = kNoId, activeId = kNoId;
    int       nodeType = -1;        // Box3D Data body type: a change retypes existing bodies

    bool operator==(const Box3DSyncFingerprint &o) const
    {
        return numPoints == o.numPoints && numOffsets == o.numOffsets &&
               pId == o.pId && orientId == o.orientId && vId == o.vId && wId == o.wId &&
               idId == o.idId && activeId == o.activeId && nodeType == o.nodeType;
    }
    bool operator!=(const Box3DSyncFingerprint &o) const { return !(*this == o); }
};

// Per object, kept in the shared state. Besides the fingerprint it carries the stamp
// its mirror entries are current under: a skipped sync does not touch the entries, so
// "seen this step" cannot be a per-entry comparison with the step's own stamp.
struct Box3DObjectSync
{
    uint64_t liveStamp  = 0;        // entries of this object carry this stamp while they are current
    uint64_t visited    = 0;        // syncStamp of the last sync that looked at the object at all
    size_t   entryCount = 0;        // entries stamped by the last full scan
    bool     hasDynamic = false;    // result of the last full scan
    bool     canSkip    = false;    // fp describes a clean, fully scanned object
    Box3DSyncFingerprint fp;
};

// Everything that is shared between timestep copies.
struct Box3DSharedState
{
    b3dop::Box3DWorld                           world;
    std::unordered_map<uint64_t, Box3DBodyMirror> mirror; // key: b3dop::makeBodyKey(object id, point id)
    uint64_t                                    generation = 0;  // for logs

    // Bumped once per sync. An entry whose stamp is not its object's live stamp
    // (Box3DObjectSync) was not seen (its point is gone); stampedCount says how many
    // entries are current, so "nothing to remove" is one comparison instead of a scan.
    uint64_t                                    syncStamp    = 0;
    size_t                                      stampedCount = 0;

    // One record per object seen by the last sync (see Box3DObjectSync). std::unordered_map
    // nodes are stable, so a pointer to a record survives other inserts.
    std::unordered_map<int, Box3DObjectSync>    objectSync;

    // Scratch reused every step (see Box3DPointSlot / Box3DWriteItem).
    std::unordered_map<int, std::vector<Box3DPointSlot>> slots;     // key: object id
    std::vector<Box3DWriteItem>                 writeItems;
    std::vector<b3dop::BodyMove>                moves;          // this step's move events

    // The sim time reading (SIM_Engine::getSimulationTime()) at the last
    // solve, and whether there has been one.
    double stampTime  = 0.0;
    bool   stamped    = false;

    std::mutex                                  mutex;

    // Is this entry current: its object was visited by the latest sync, and the entry
    // carries the stamp that object's entries are current under?
    bool isLive(const Box3DBodyMirror &m) const
    {
        const auto it = objectSync.find(m.objectId);
        return it != objectSync.end() && it->second.visited == syncStamp &&
               m.stamp == it->second.liveStamp;
    }

    // True if this world was last stepped exactly one timestep before `now`.
    // Written as (now - stamp) == timestep so it does not matter whether the
    // engine reports the start or the end of the step during a solve.
    bool canContinue(double now, double timestep) const
    {
        if (!stamped) return false;
        const double tol = 1e-3 * (timestep > 0.0 ? timestep : 1.0);
        const double d   = (now - stampTime) - timestep;
        return (d < 0 ? -d : d) <= tol;
    }
};

class SIM_Box3DWorld : public SIM_Data
{
public:
    // The shared state, or null if there is no world yet.
    Box3DSharedState *state() const { return myState.get(); }

    // Create a NEW shared state with a fresh Box3D world and point this data at
    // it. Does not touch the old state (other timesteps may still use it).
    bool createWorld(const b3dop::WorldSettings &settings);

protected:
    explicit SIM_Box3DWorld(const SIM_DataFactory *factory);
    ~SIM_Box3DWorld() override;

    void initializeSubclass() override;

    // Copies SHARE the state (see top of file).
    void makeEqualSubclass(const SIM_Data *source) override;

    // Report a small constant. Every sharing copy would otherwise count the
    // whole world against the DOP cache memory budget.
    int64 getMemorySizeSubclass() const override;

    // No saveSubclass/loadSubclass on purpose: a world cannot be serialised
    // here, so a sim reloaded from disk has no world and rebuilds.

private:
    std::shared_ptr<Box3DSharedState> myState;

    static const SIM_DopDescription *getDescription();

    DECLARE_STANDARD_GETCASTTOTYPE();
    DECLARE_DATAFACTORY(SIM_Box3DWorld, SIM_Data, "Box3D World", getDescription());
};
