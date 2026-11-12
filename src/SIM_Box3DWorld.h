// SIM_Box3DWorld.h
//
// SIM_Data that owns the live Box3D world: a rebuildable cache of what the DOP objects hold.
//
// DOPs copy data every step (makeEqual), so copies share one Box3DSharedState. The state's
// stamp (sim time of the last step) lives in the shared state, not the copies, so an older
// timestep cannot claim the world is current. The solver continues only if the world was
// stepped exactly one timestep ago; otherwise (first frame, scrub, re-solve, cache reload) it
// builds a new state for this data only, and other timesteps keep the old one.
#pragma once

#include <SIM/SIM_Data.h>
#include <SIM/SIM_DataUtils.h>

#include <GA/GA_Types.h>

#include <array>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "Box3DWorld.h"

// One body's pose and velocity as last given to / read from Box3D (DOP terms). Compared with
// the point to detect edits by other nodes.
struct Box3DBodyPose
{
    double pos[3] = { 0, 0, 0 };
    float  rot[4] = { 0, 0, 0, 1 };   // x, y, z, w
    float  lin[3] = { 0, 0, 0 };
    float  ang[3] = { 0, 0, 0 };      // radians per second
};

// Settings that may change during the simulation, as last applied (attribute, else the Box3D
// Data value). A scan calls only the setters for what differs.
struct Box3DBodyProps
{
    float mass           = 1.0f;      // `mass`; Box3D gets density = mass / shape volume
    float friction       = 0.6f;      // `friction`
    float restitution    = 0.0f;      // `bounce`
    float linearDamping  = 0.0f;      // `lineardamping`
    float angularDamping = 0.0f;      // `angulardamping`
    float gravityScale   = 1.0f;      // `gravityscale`
    bool  sleepOk        = true;      // creation: bullet_want_deactivate, else enablesleep; later: the former only
    bool  ignored        = false;     // `bullet_ignore`
    bool  computeMass    = false;     // `computemass`
    float density        = 1000.0f;   // `density`
    float inertiaScale   = 1.0f;      // `inertialtensorstiffness`
    float sleepThreshold = 0.05f;     // `sleepthreshold`
    bool  fastObject     = false;     // `fastobject`
    bool  contactRecycling = true;    // `contactrecycling`
    int   filterProfile  = -1;        // collisiongroup / collisionignore / object
    float minActivationImpulse = 0.0f;    // `min_activation_impulse`
    std::string activationIgnore;     // `activationignore`
};

struct Box3DBodyMirror : Box3DBodyPose
{
    // What the body was built with, so a scan can tell what changed.
    Box3DBodyProps props;
    float    volume    = 0.0f;        // total shape volume
    uint64_t shapeSig  = 0;           // a rebuild giving the same signature is a no-op
    bool     shapeless = false;       // `none`, or a rebuild left no shape: always static
    bool     autoScale = false;       // convex / concave: rebuilt when the scale changes
    double   sizeKey[3] = { 1, 1, 1 };   // the scale the shapes were built at

    bool     animated  = false;       // inactive + `animated`: kinematic, follows its point
    bool     driven    = false;       // given a target last step (velocity to clear)
    bool     sleeping  = false;       // asleep, as last reported by a move event
    int64_t  pointIndex = -1;         // point number at the last scan (Impacts data)

    // The body's point, refreshed by every sync, so the write-back needs no lookup. Box3D's
    // body user data points at this entry, so entries never move (unordered_map nodes) and
    // are only erased with their body.
    int64_t  pointOff = -1;           // GA_Offset of the point in the object's geometry
    int64_t  pointId  = 0;            // the `id` attribute, else the point number
    int      objectId = 0;
    uint64_t stamp    = 0;            // syncStamp of the last sync that saw it

    // The point's packed primitive and its 3x3 as the last sync read it (reused by the write-back).
    bool     packed   = false;
    int64_t  primOff  = -1;           // GA_Offset of the primitive
    double   m3[9]    = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

    // The body's handle and the type the solver last set.
    b3dop::BodyHandle        handle = 0;
    b3dop::BodySettings::Type type  = b3dop::BodySettings::Type::Dynamic;

    bool dynamic() const { return type == b3dop::BodySettings::Type::Dynamic; }   // written back
};

// One point as syncObject() reads it. Kept between steps; `entry` caches the mirror entry
// while the point's offset and key are unchanged (all slots are dropped when an entry is erased).
struct Box3DPointSlot
{
    GA_Offset         off    = GA_INVALID_OFFSET;
    int64_t           bid    = 0;
    uint64_t          key    = 0;
    Box3DBodyMirror  *entry  = nullptr;
    Box3DBodyPose     cur;
    bool              ok     = false;    // readPointState succeeded (P is finite)
    bool              badVelocity = false;   // v or w was not finite (read as zero)
    bool              active = true;
    bool              packed = false;    // the point belongs to a packed primitive
    int64_t           index  = 0;        // point number
    int64_t           primOff = -1;
    double            m3[9]  = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };
};

// One awake body's result on its way to the geometry. Packed matrices live in a side array
// (Box3DPackedWrite) to keep this small.
struct Box3DWriteItem
{
    GA_Offset         off      = GA_INVALID_OFFSET;
    Box3DBodyMirror  *entry    = nullptr;
    Box3DBodyPose     s;                        // what Box3D reports
    int               objectId = 0;
    int               packedIdx = -1;           // index into Box3DSharedState::packedWrites, or -1
    int8_t            sleepState = -1;          // bullet_sleeping to write: -1 none, 0 awake, 1 asleep
};

// A point attribute written after the step, for any body (not only awake ones).
struct Box3DPointWrite
{
    enum : unsigned { kAutofitValid = 1, kMass = 2, kActivate = 4 };
    GA_Offset         off      = GA_INVALID_OFFSET;
    Box3DBodyMirror  *entry    = nullptr;
    int               objectId = 0;
    unsigned          flags    = 0;
    float             mass     = 0.0f;      // kMass: computed mass (computemass)
};

// A `dead` point to delete after the step (its body is already gone).
struct Box3DDeletion
{
    int       objectId = 0;
    GA_Offset off      = GA_INVALID_OFFSET;
    int64_t   pointId  = 0;
};

// A packed rotation computed in the write-back's parallel pass, written in its serial pass.
struct Box3DPackedWrite
{
    bool      has      = false;                 // newM3 / primOff were filled in
    GA_Offset primOff  = GA_INVALID_OFFSET;
    double    newM3[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };
};

// Change markers of an object's geometry when the solver last finished with it (attribute data
// ids, counts, node values). An identical fingerprint next step means the scan can be skipped.
struct Box3DSyncFingerprint
{
    static constexpr GA_DataId kNoId = GA_DataId(-1);   // attribute absent

    // Prop markers (kPropAttribNames in SIM_SolverBox3D.cpp, primsId, nodeHash): when any
    // changed, the scan also compares every body's settings. The .cpp static_asserts the count.
    static constexpr int kNumPropIds = 25;
    using PropIds = std::array<GA_DataId, kNumPropIds>;

    int64_t   numPoints  = -1;      // -1 with primitives: packed topology is assumed fixed
    int64_t   numOffsets = -1;
    GA_DataId pId = kNoId, orientId = kNoId, vId = kNoId, wId = kNoId;
    GA_DataId idId = kNoId, activeId = kNoId;
    int       nodeType = -1;        // Box3D Data body type
    PropIds   propIds  = filledPropIds();
    GA_DataId primsId  = kNoId;     // primitive list (packed transforms)
    uint64_t  nodeHash = 0;         // Box3D Data values that change existing bodies

    static PropIds filledPropIds()
    {
        PropIds ids;
        ids.fill(kNoId);
        return ids;
    }
    bool samePropMarkers(const Box3DSyncFingerprint &o) const
    {
        return propIds == o.propIds && primsId == o.primsId && nodeHash == o.nodeHash;
    }
    bool operator==(const Box3DSyncFingerprint &o) const
    {
        return numPoints == o.numPoints && numOffsets == o.numOffsets &&
               pId == o.pId && orientId == o.orientId && vId == o.vId && wId == o.wId &&
               idId == o.idId && activeId == o.activeId && nodeType == o.nodeType &&
               samePropMarkers(o);
    }
};

// Per object. A skipped sync leaves its entries untouched, so "current" is the object's
// liveStamp rather than the step's stamp.
struct Box3DObjectSync
{
    uint64_t liveStamp  = 0;        // stamp the object's current entries carry
    uint64_t visited    = 0;        // last sync that saw the object
    size_t   entryCount = 0;        // entries stamped by the last full scan
    bool     canSkip    = false;    // fp describes a clean, fully scanned object
    bool     propsKnown = false;    // fp's prop markers are recorded
    bool     anyDeforming = false;  // deforming bodies: never skip
    bool     anyDriven    = false;  // a moved animated body (velocity to clear): never skip
    bool     addImpact    = false;  // Box3D Data "Add Impact Data"
    std::vector<Box3DBodyMirror *> impacts;     // with addImpact: every body (rebuilt by each full scan)
    std::vector<Box3DBodyMirror *> activation;  // inactive bodies with min_activation_impulse > 0
    Box3DSyncFingerprint fp;
};

// A static collider made from an affector object (an object that is not solved by us but affects
// our objects through a collision relationship, e.g. a Static Object DOP). One body, one triangle mesh.
struct Box3DAffectorBody
{
    b3dop::BodyHandle handle    = 0;        // 0: the last build failed (not retried until the signature changes)
    uint64_t          stamp     = 0;        // syncStamp that last saw the object as an affector
    uint64_t          signature = 0;        // geometry + material + baked transform the shape was built from
    double            pos[3]    = { 0, 0, 0 };      // the body's pose as last given to Box3D
    float             rot[4]    = { 0, 0, 0, 1 };
    bool              poseKnown = false;
};

// Everything shared between timestep copies.
struct Box3DSharedState
{
    b3dop::Box3DWorld                           world;
    std::unordered_map<uint64_t, Box3DBodyMirror> mirror; // key: makeBodyKey(object id, point id)

    // Bumped once per sync; stampedCount current entries (equal to mirror.size(): nothing to remove).
    uint64_t                                    syncStamp    = 0;
    size_t                                      stampedCount = 0;

    std::unordered_map<int, Box3DObjectSync>    objectSync;     // per object seen by the last sync
    std::unordered_map<int, Box3DAffectorBody>  affectors;      // key: object id of the affector

    // Scratch reused every step.
    std::unordered_map<int, std::vector<Box3DPointSlot>> slots;     // key: object id
    std::vector<Box3DWriteItem>                 writeItems;
    std::vector<Box3DPackedWrite>               packedWrites;
    std::vector<b3dop::BodyMove>                moves;
    std::vector<b3dop::ContactPoint>            contacts;
    // Filled by the sync (in object order), used after the step:
    std::vector<Box3DPointWrite>                pointWrites;
    std::vector<Box3DDeletion>                  deletions;
    std::vector<int>                            impactObjects;      // objects with Impacts data from last step (sorted)

    float stepDt = 0.0f;                        // this step's Box3D time step (DOP step * Time Scale)

    double stampTime  = 0.0;                    // SIM_Engine::getSimulationTime() at the last solve
    bool   stamped    = false;

    std::mutex                                  mutex;

    // Current: its object was visited by the latest sync and it carries the object's liveStamp.
    bool isLive(const Box3DBodyMirror &m) const
    {
        const auto it = objectSync.find(m.objectId);
        return it != objectSync.end() && it->second.visited == syncStamp &&
               m.stamp == it->second.liveStamp;
    }

    // Last stepped exactly one timestep before `now` (as a difference, so it does not matter
    // whether the engine reports the start or end of the step).
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
    Box3DSharedState *state() const { return myState.get(); }     // null: no world yet

    // A new shared state with a fresh world, for this data only.
    bool createWorld(const b3dop::WorldSettings &settings);

protected:
    explicit SIM_Box3DWorld(const SIM_DataFactory *factory);
    ~SIM_Box3DWorld() override;

    void initializeSubclass() override;

    void makeEqualSubclass(const SIM_Data *source) override;     // copies share the state

    // A small constant: otherwise every sharing copy counts the whole world against the cache.
    int64 getMemorySizeSubclass() const override;

    // No save / load: a reloaded sim rebuilds its world.

private:
    std::shared_ptr<Box3DSharedState> myState;

    static const SIM_DopDescription *getDescription();

    DECLARE_STANDARD_GETCASTTOTYPE();
    DECLARE_DATAFACTORY(SIM_Box3DWorld, SIM_Data, "Box3D World", getDescription());
};
