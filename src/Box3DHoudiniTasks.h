// Box3DHoudiniTasks.h
//
// Runs Box3D's parallel work on Houdini's shared TBB pool, so no Box3D world starts OS threads of
// its own (Box3D's built-in scheduler keeps workerCount - 1 threads alive per world, and old worlds
// stay alive while cached frames use them).
//
// Why this is safe with Box3D's solver (box3d/src/solver.c):
//  * Tasks carry their own workerIndex; nothing depends on which OS thread runs them.
//  * b3SolverTask worker 0 is also run by the stepping thread, and its orchestrator can finish the
//    whole solve alone, so a pool with fewer free threads than workerCount cannot deadlock.
//    Workers > 0 only spin on stage sync.
//  * run() returning false (ran inline) is supported: Box3D enqueues solve[0] first.
//  * finishTask is only called by the stepping thread after it has queued what it waits for, so
//    waiting for the whole group is fine.
#pragma once

#include "Box3DWorld.h"

#include <UT/UT_TaskGroup.h>
#include <UT/UT_TaskScope.h>
#include <UT/UT_Thread.h>

#if __has_include(<oneapi/tbb/task_arena.h>)
    #include <oneapi/tbb/task_arena.h>      // oneTBB (newer Houdini)
#else
    #include <tbb/task_arena.h>             // classic TBB
#endif

// One per world. Never share one between worlds: wait() waits for every task in the group.
class Box3DHoudiniTasks final : public b3dop::TaskRunner
{
public:
    ~Box3DHoudiniTasks() override { (void)myGroup.wait(); }    // a task_group must be waited

    bool run(void (*fn)(void *), void *ctx) override
    {
        if (!UT_Thread::isThreadingEnabled())
        {
            fn(ctx);                    // threading is off on this thread: run inline
            return false;
        }
        // UT_TaskGroup does not carry the UT_TaskScope over (see UT_TaskList.h).
        const UT_TaskScope *parent = UT_TaskScope::getCurrent();
        myGroup.run([fn, ctx, parent]() {
            UT_TaskScope scope(parent);
            fn(ctx);
        });
        return true;
    }

    void wait() override { (void)myGroup.wait(); }

    // Waiting threads then only run this step's tasks, not unrelated Houdini work (another cook,
    // another world's spinning solver task).
    void isolate(void (*fn)(void *), void *ctx) override
    {
        tbb::this_task_arena::isolate([&] { fn(ctx); });
    }

private:
    UT_TaskGroup myGroup;
};
