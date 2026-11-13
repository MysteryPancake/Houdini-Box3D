// Proves Box3D builds, links, and a world can be created/destroyed.
// Run this first, before touching Houdini.
#include "KeyedWorld.h"
#include <cstdio>

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

  world.destroy();
  if (world.isValid()) {
    std::printf("FAIL: world still valid after destroy\n");
    return 1;
  }
  std::printf("OK\n");
  return 0;
}
