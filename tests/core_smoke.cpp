// Proves Box3D builds, links, and a world can be created/destroyed.
// Run this first, before touching Houdini.
#include "KeyedWorld.h"
#include "core/Box3DCpu.h"
#include <cstdio>
#include <thread>

int main() {
  int maj = 0, min = 0, rev = 0;
  b3dop::Box3DWorld::version(maj, min, rev);
  std::printf("Box3D version %d.%d.%d\n", maj, min, rev);

  b3dop::KeyedWorld world;
  if (!world.create(b3dop::WorldSettings())) {
    std::printf("FAIL: could not create world\n");
    return 1;
  }
  std::printf("world valid: %s, bodies: %d\n", world.isValid() ? "yes" : "no",
              world.bodyCount());

  // The solver's Threads default.
  const int cores = b3dop::performanceCoreCount();
  const int workers = b3dop::defaultWorkerCount();
  const unsigned logical = std::thread::hardware_concurrency();
  std::printf("performance cores: %d (logical %u), default workers: %d\n",
              cores, logical, workers);
  if (workers < 1 || workers > b3dop::kMaxWorkerCount || cores < 0 ||
      (cores > 0 && logical > 0 && cores > static_cast<int>(logical))) {
    std::printf("FAIL: core count out of range\n");
    return 1;
  }

  world.destroy();
  if (world.isValid()) {
    std::printf("FAIL: world still valid after destroy\n");
    return 1;
  }
  std::printf("OK\n");
  return 0;
}
