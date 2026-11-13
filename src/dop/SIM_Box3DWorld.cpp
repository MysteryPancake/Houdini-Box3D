// SIM_Box3DWorld.cpp
#include "SIM_Box3DWorld.h"
#include "SIM_Box3DData.h"
#include "core/Box3DDebug.h"
#include "sync/Box3DHoudiniTasks.h"

#include <PRM/PRM_Include.h>
#include <SIM/SIM_DopDescription.h>
#include <SIM/SIM_Utils.h>

SIM_Box3DWorld::SIM_Box3DWorld(const SIM_DataFactory *factory)
    : BaseClass(factory) {}

SIM_Box3DWorld::~SIM_Box3DWorld() {
  // The world goes when the last timestep copy sharing it does.
}

const SIM_DopDescription *SIM_Box3DWorld::getDescription() {
  static PRM_Template theTemplates[] = {
      PRM_Template() // no parameters
  };

  static SIM_DopDescription theDopDescription(
      false,               // false: not a DOP node of its own
      "box3d_world",       // internal name
      "Box3D World",       // label
      kBox3DWorldDataName, // data name
      classname(), theTemplates);

  return &theDopDescription;
}

void SIM_Box3DWorld::initializeSubclass() {
  SIM_Data::initializeSubclass();
  myState.reset();
}

void SIM_Box3DWorld::makeEqualSubclass(const SIM_Data *source) {
  SIM_Data::makeEqualSubclass(source);

  const SIM_Box3DWorld *src = SIM_DATA_CASTCONST(source, SIM_Box3DWorld);

  if (src) {
    myState = src->myState; // share, do not rebuild
  } else {
    myState.reset();
    B3D_LOG("World[%p] makeEqual       from NON-world source %p, state dropped",
            (const void *)this, (const void *)source);
  }
}

int64 SIM_Box3DWorld::getMemorySizeSubclass() const {
  return SIM_Data::getMemorySizeSubclass() + 64;
}

bool SIM_Box3DWorld::createWorld(const b3dop::WorldSettings &settings) {
  auto fresh = std::make_shared<Box3DSharedState>();
  // One runner per world: the world's parallel work goes to Houdini's TBB pool,
  // no threads of its own.
  if (!fresh->world.create(settings, std::make_shared<Box3DHoudiniTasks>())) {
    B3D_LOG("World[%p] createWorld FAILED", (const void *)this);
    return false;
  }
  myState = std::move(fresh);
  return true;
}
