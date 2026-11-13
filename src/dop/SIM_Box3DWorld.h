// SIM_Box3DWorld.h
//
// SIM_Data that owns the live Box3D world: a rebuildable cache of what the DOP objects hold.
//
// DOPs copy data every step (makeEqual), so copies share one Box3DSharedState (sync/Box3DState.h).
// The solver continues only if the world was stepped exactly one timestep ago; otherwise (first
// frame, scrub, re-solve, cache reload) it builds a new state for this data only, and other
// timesteps keep the old one.
#pragma once

#include <SIM/SIM_Data.h>
#include <SIM/SIM_DataUtils.h>

#include <memory>

#include "core/Box3DWorld.h"
#include "sync/Box3DState.h"

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
