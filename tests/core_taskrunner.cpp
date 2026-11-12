// A host-supplied TaskRunner (the hook the plugin implements on Houdini's TBB pool): checks Box3D
// runs through it, that the world starts no threads of its own, and that results match 1 worker.
// Uses std::thread here only because this test has no Houdini; the plugin never does.
// usage: b3dop_core_taskrunner [workers=4]
#include "Box3DWorld.h"
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <thread>
#include <vector>

// Runs everything on one fixed set of threads, shared by every world.
struct Pool
{
    std::vector<std::thread> threads;
    std::atomic<int>         pending{ 0 };
    std::atomic<bool>        quit{ false };
    struct Job { void (*fn)(void *); void *ctx; };
    std::vector<Job>         jobs;
    std::atomic_flag         lock = ATOMIC_FLAG_INIT;
    std::atomic<long>        executed{ 0 };

    void acquire() { while (lock.test_and_set(std::memory_order_acquire)) {} }
    void release() { lock.clear(std::memory_order_release); }
    bool pop(Job &j)
    {
        acquire();
        const bool ok = !jobs.empty();
        if (ok) { j = jobs.back(); jobs.pop_back(); }
        release();
        return ok;
    }
    explicit Pool(int n)
    {
        for (int i = 0; i < n; ++i)
            threads.emplace_back([this] {
                while (!quit)
                {
                    Job j;
                    if (pop(j)) { j.fn(j.ctx); ++executed; --pending; }
                    else std::this_thread::yield();
                }
            });
    }
    ~Pool() { quit = true; for (auto &t : threads) t.join(); }
};

struct PoolRunner : b3dop::TaskRunner
{
    Pool &pool;
    explicit PoolRunner(Pool &p) : pool(p) {}
    bool run(void (*fn)(void *), void *ctx) override
    {
        ++pool.pending;
        pool.acquire(); pool.jobs.push_back({ fn, ctx }); pool.release();
        return true;
    }
    void wait() override
    {
        // The waiting thread helps, like TBB's wait().
        while (pool.pending > 0)
        {
            Pool::Job j;
            if (pool.pop(j)) { j.fn(j.ctx); ++pool.executed; --pool.pending; }
            else std::this_thread::yield();
        }
    }
};

static bool run(int workers, Pool *pool, std::vector<double> &out)
{
    b3dop::WorldSettings ws;
    ws.workerCount = workers;
    b3dop::Box3DWorld w;
    if (!w.create(ws, pool ? std::make_shared<PoolRunner>(*pool) : nullptr))
        return false;
    const float qid[4] = { 0, 0, 0, 1 };
    b3dop::BodySettings ground; ground.type = b3dop::BodySettings::Type::Static;
    const double gp[3] = { 0, -0.5, 0 };
    const float  gh[3] = { 60, 0.5f, 60 };
    w.addBody(1, ground, gp, qid);
    w.addBoxShape(1, gh);
    const float bh[3] = { 0.5f, 0.5f, 0.5f };
    uint64_t id = 100;
    srand(1);
    for (int y = 0; y < 8; ++y)
        for (int x = 0; x < 8; ++x)
            for (int z = 0; z < 8; ++z)
            {
                const double p[3] = { (x - 4) * 1.05 + (rand() % 100) * 0.0004, 0.5 + y * 1.02,
                                      (z - 4) * 1.05 + (rand() % 100) * 0.0004 };
                w.addBody(id, b3dop::BodySettings(), p, qid);
                w.addBoxShape(id, bh);
                ++id;
            }
    for (int i = 0; i < 90; ++i)
        w.step(1.0f / 60.0f);
    for (uint64_t b = 100; b < id; ++b)
    {
        double p[3]; float q[4];
        w.getTransform(b, p, q);
        out.insert(out.end(), p, p + 3);
    }
    return true;
}

int main(int argc, char **argv)
{
    const int workers = argc > 1 ? std::max(2, atoi(argv[1])) : 4;
    std::vector<double> serial, shared;
    if (!run(1, nullptr, serial))
        return 1;

    Pool pool(workers - 1);
    // Many worlds alive at once on the one pool: the case that used to pile up threads.
    const int before = int(pool.threads.size());
    if (!run(workers, &pool, shared))
        return 1;
    for (int i = 0; i < 40; ++i)
    {
        std::vector<double> scratch;
        if (!run(workers, &pool, scratch))
            return 1;
    }

    double maxDiff = 0;
    for (size_t i = 0; i < serial.size(); ++i)
        maxDiff = std::max(maxDiff, std::fabs(serial[i] - shared[i]));
    std::printf("pool threads %d (before %d), tasks run %ld, max position diff vs 1 worker %.3g\n",
                int(pool.threads.size()), before, pool.executed.load(), maxDiff);
    if (pool.executed.load() == 0 || maxDiff > 1e-4)
    {
        std::printf("FAIL\n");
        return 1;
    }
    std::printf("OK\n");
    return 0;
}
