// Runs the same scene with 1 worker and with N workers (Box3D's own thread
// pool): reports step time and checks the results are identical.
// usage: b3dop_core_threads [workers=4] [boxes per side=12] [steps=120]
#include "KeyedWorld.h"
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>

struct Result {
  double ms = 0;
  std::vector<double> pos;
  int bodies = 0;
};

static bool run(int workers, int side, int steps, Result &r) {
  b3dop::WorldSettings ws;
  ws.workerCount = workers;
  b3dop::KeyedWorld w;
  if (!w.create(ws))
    return false;

  const float qid[4] = {0, 0, 0, 1};
  b3dop::BodySettings ground;
  ground.type = b3dop::BodySettings::Type::Static;
  const double gp[3] = {0, -0.5, 0};
  const float gh[3] = {60, 0.5f, 60};
  w.addBody(1, ground, gp, qid);
  w.addBoxShape(1, gh);

  // side x side x side grid of boxes, with a little jitter so they topple
  const float bh[3] = {0.5f, 0.5f, 0.5f};
  uint64_t id = 100;
  srand(1);
  for (int y = 0; y < side; ++y)
    for (int x = 0; x < side; ++x)
      for (int z = 0; z < side; ++z) {
        const double p[3] = {(x - side / 2) * 1.05 + (rand() % 100) * 0.0004,
                             0.5 + y * 1.02,
                             (z - side / 2) * 1.05 + (rand() % 100) * 0.0004};
        w.addBody(id, b3dop::BodySettings(), p, qid);
        w.addBoxShape(id, bh);
        ++id;
      }
  r.bodies = int(id - 100);

  const auto t0 = std::chrono::steady_clock::now();
  for (int i = 0; i < steps; ++i)
    w.step(1.0f / 60.0f);
  const auto t1 = std::chrono::steady_clock::now();
  r.ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

  for (uint64_t b = 100; b < id; ++b) {
    double p[3];
    float q[4];
    w.getTransform(b, p, q);
    r.pos.insert(r.pos.end(), p, p + 3);
  }
  return true;
}

int main(int argc, char **argv) {
  const int workers = argc > 1 ? std::atoi(argv[1]) : 4;
  const int side = argc > 2 ? std::atoi(argv[2]) : 12;
  const int steps = argc > 3 ? std::atoi(argv[3]) : 120;

  Result a, b;
  if (!run(1, side, steps, a) || !run(workers, side, steps, b)) {
    std::printf("FAIL: could not create a world\n");
    return 1;
  }
  double maxDiff = 0;
  for (size_t i = 0; i < a.pos.size(); ++i)
    maxDiff = std::fmax(maxDiff, std::fabs(a.pos[i] - b.pos[i]));

  std::printf("%d bodies, %d steps\n", a.bodies, steps);
  std::printf("1 worker : %8.1f ms  (%.2f ms/step)\n", a.ms, a.ms / steps);
  std::printf("%d workers: %8.1f ms  (%.2f ms/step)  speedup x%.2f\n", workers,
              b.ms, b.ms / steps, a.ms / b.ms);
  std::printf("max position difference between runs: %g\n", maxDiff);
  if (maxDiff > 1e-4) {
    std::printf("FAIL: threaded result differs from serial\n");
    return 1;
  }
  std::printf("OK\n");
  return 0;
}
