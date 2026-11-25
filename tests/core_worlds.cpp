// Several threads create, step and destroy their own worlds at once, as
// separate DOP networks cooking together would: Box3D's world list is not
// thread-safe, so Box3DWorld serializes creating and destroying. Checks no
// crash, every world steps correctly (a box falls the same distance in each),
// and no world is left alive. usage: b3dop_core_worlds [threads=8] [rounds=40]
#include "KeyedWorld.h"
#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <thread>
#include <vector>

static bool fallOnce(double &y) {
  b3dop::WorldSettings ws;
  ws.workerCount = 1;
  b3dop::KeyedWorld w;
  if (!w.create(ws))
    return false;
  const float qid[4] = {0, 0, 0, 1};
  const double p[3] = {0, 10, 0};
  const float half[3] = {0.5f, 0.5f, 0.5f};
  w.addBody(1, b3dop::BodySettings(), p, qid);
  w.addBoxShape(1, half);
  for (int i = 0; i < 6; ++i)
    w.step(1.0f / 60.0f);
  double pos[3];
  float q[4];
  w.getTransform(1, pos, q);
  y = pos[1];
  return true; // the world is destroyed with `w`
}

int main(int argc, char **argv) {
  const int threads = argc > 1 ? std::atoi(argv[1]) : 8;
  const int rounds = argc > 2 ? std::atoi(argv[2]) : 40;
  const int before = b3dop::Box3DWorld::liveWorldCount();

  std::atomic<int> failed{0};
  std::atomic<int> wrong{0};
  std::vector<std::thread> pool;
  pool.reserve(static_cast<size_t>(threads));
  for (int t = 0; t < threads; ++t)
    pool.emplace_back([&] {
      for (int r = 0; r < rounds; ++r) {
        double y = 0;
        if (!fallOnce(y))
          ++failed;
        else if (!(y < 10.0 && y > 9.9)) // 6 steps of 1/60 s fall ~0.05
          ++wrong;
      }
    });
  for (auto &t : pool)
    t.join();

  const int after = b3dop::Box3DWorld::liveWorldCount();
  std::printf("%d threads x %d worlds: %d failed to create, %d fell wrongly, "
              "%d worlds alive before / %d after\n",
              threads, rounds, failed.load(), wrong.load(), before, after);
  if (failed || wrong || after != before) {
    std::printf("FAIL\n");
    return 1;
  }
  std::printf("OK\n");
  return 0;
}
