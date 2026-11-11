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

#include <cstdint>
#include <memory>
#include <mutex>
#include <unordered_map>

#include "Box3DWorld.h"

// What we last handed to / read back from Box3D for one body (one POINT of a
// DOP object), in DOP terms. Compared against the point's current values each
// step to detect edits made by other nodes (the stamp alone cannot see those).
struct Box3DBodyMirror
{
    double pos[3] = { 0, 0, 0 };
    float  rot[4] = { 0, 0, 0, 1 };   // x, y, z, w
    float  lin[3] = { 0, 0, 0 };
    float  ang[3] = { 0, 0, 0 };      // radians per second
    bool   dynamic = false;           // a body the solver writes back (not static)

    // Cached so the per-step loops need no hash lookups into the Box3D world:
    // the body's handle, and the body type the solver last set (the same value
    // Box3DWorld::getBodyType would return). Valid for the life of the entry:
    // the mirror is per world, and an entry is erased when its body is removed.
    b3dop::BodyHandle        handle = 0;
    b3dop::BodySettings::Type type  = b3dop::BodySettings::Type::Dynamic;
};

// Everything that is shared between timestep copies.
struct Box3DSharedState
{
    b3dop::Box3DWorld                           world;
    std::unordered_map<uint64_t, Box3DBodyMirror> mirror; // key: b3dop::makeBodyKey(object id, point id)
    uint64_t                                    generation = 0;  // for logs

    // The sim time reading (SIM_Engine::getSimulationTime()) at the last
    // solve, and whether there has been one.
    double stampTime  = 0.0;
    bool   stamped    = false;

    std::mutex                                  mutex;

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
    void markStepped(double now) { stampTime = now; stamped = true; }
};

class SIM_Box3DWorld : public SIM_Data
{
public:
    // The shared state, or null if there is no world yet.
    Box3DSharedState *state() const { return myState.get(); }

    // The wrapped Box3D world, or null.
    b3dop::Box3DWorld *world() const { return myState ? &myState->world : nullptr; }

    // Create a NEW shared state with a fresh Box3D world and point this data at
    // it. Does not touch the old state (other timesteps may still use it).
    bool createWorld(const b3dop::WorldSettings &settings);

    // Drop this data's reference. The solver rebuilds next step.
    void resetWorld();

    bool hasValidWorld() const;

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
