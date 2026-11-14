# CLAUDE.md

Technical notes for AI agents (and humans) working on this codebase: how the code works and why.
Build and usage instructions for users are in `README.md`.

## What this is

A Houdini DOP plugin (HDK, C++17) wrapping [Box3D](https://github.com/erincatto/box3d) (Erin Catto's 3D rigid
body engine, C17, a git submodule in `extern/box3d`). It registers these `SIM_Data` classes:

| DOP name | Class | Role |
|---|---|---|
| Box3D Solver | `SIM_SolverBox3D` | Multi-object solver: one call per timestep with all its objects |
| Box3D Data | `SIM_Box3DData` | Per-object defaults (fallbacks for missing point attributes) |
| (hidden) Box3D World | `SIM_Box3DWorld` | Holds the live Box3D world; not a DOP node |
| Box3D <Type> Constraint Relationship | `SIM_Box3DConRel<Type>` | One per joint type: the joint's settings, named by `constraint_name` in a Constraint Network |

Design goals: Box3D's native performance (handle-based hot paths, no per-body hash lookups, threaded per-point passes
where safe) and Houdini Bullet behaviour (same attribute names and rules, as documented on the RBD Packed Object page).
The exception is sleeping and enabling, which follow Box3D's own model (`enablesleep`, `awake`, `enabled`; see "Sleep and
enable state"); Bullet's `bullet_want_deactivate`, `bullet_sleeping`, `bullet_ignore` and `min_activation_impulse` are not read.

The code is largely LLM-written by an author still learning the HDK. Treat HDK usage with healthy skepticism.
Everything not yet tested or verified is listed in one place: "Untested and unverified" in `docs/agents/status.md`; everything still to do in
another: "Known gaps / TODO" in the same file. Keep them there and nowhere else (no TODO or "unverified" notes in code comments, the
README or other sections here).

## Documentation split

`README.md` is for people using the plugin in Houdini: how to build it, which nodes and attributes exist, what they do,
and what behaves differently from Houdini's own solvers. Keep it short. Leave out anything a user cannot act on or would
not notice: how the solver does something, Box3D internals and terms (density, restitution, move events, hull caches),
performance mechanics. Those belong here (unverified details go in "Untested and unverified", `docs/agents/status.md`). When a change adds a feature, add one line or table
row to the README if users need it, and put the explanation here.

README rules (hard limits, not suggestions):
- Never add more than one or two sentences, or one table row, per change. Do not grow a section to explain a feature.
- No implementation details: no function, class or HDK names, locks, fingerprints, step ordering, clearing or caching
  behaviour. If a sentence says how the code works, it goes in this file.
- Do not rewrite or expand existing README text while making an unrelated change. Trim, don't pad.
- Before finishing a task, re-read any README edit and cut it down.

UI tooltips (the help text on parameters) follow the same rule: one or two short sentences, under about 150
characters. Say what the control does and, if needed, when to change it. No implementation detail, no lists of point
attributes, no restating how the sibling controls work.

## Where to look

Load only the file for the area you are changing.

| File | Covers |
|---|---|
| [docs/agents/build.md](docs/agents/build.md) | CMake options, build scripts, tests, debug logging, the Houdini package |
| [docs/agents/core.md](docs/agents/core.md) | `b3dop::Box3DWorld`: handle API, mass, filtering, shapes and the hull cache, world settings |
| [docs/agents/solver.md](docs/agents/solver.md) | Shared world state, step order, `syncObject`, the skip fingerprint, write-back, threading |
| [docs/agents/bodies.md](docs/agents/bodies.md) | Mid-simulation updates, per-attribute features (sleep, CCD, locks, filters, impacts, ...), building shapes |
| [docs/agents/forces-affectors.md](docs/agents/forces-affectors.md) | Force DOPs, point `force` / `torque`, Static Object colliders |
| [docs/agents/whole-object.md](docs/agents/whole-object.md) | RBD Object style bodies (one body per DOP object) |
| [docs/agents/joints.md](docs/agents/joints.md) | Constraint networks, joint nodes, SideFX constraint data mapping |
| [docs/agents/reference.md](docs/agents/reference.md) | Parameters, point attributes, node warnings, what is not supported, the parameter-tab builder |
| [docs/agents/status.md](docs/agents/status.md) | Untested and unverified, HDK calls checked, Known gaps / TODO |

**Values that live in code stay in code.** Defaults, counts, thresholds and sizes are not copied into these files: name
the constant or table and its file (`kMinParallelPoints` in `Box3DSolverInternal.h`, `kPropAttribNames` in
`Box3DAttribNames.h`, the `Box3DParms` tables in `SIM_*.cpp`). A number in a doc is only for a measurement or a fact
about Box3D / Houdini that the code does not hold. `python tools/check_docs.py` checks that the `k...` constants and the
relative links the docs mention still exist.

## Layout

```
CMakeLists.txt        Targets: b3dop_core, the core test exes, Box3DDOP (plugin)
build.sh / build.bat  Convenience wrappers (HFS discovery, submodule fetch, stale build/ reset)
test.bat              Runs the Houdini checks (tests/houdini/run_tests.py) with hython (Windows)
make_examples.bat     Builds the example scenes (examples/make_examples.py) with hython (Windows)
extern/box3d/         Box3D submodule (never edit; static lib, EXCLUDE_FROM_ALL)
icons/box3d.svg     Node icon; the build copies it to config/Icons as DOP_<node name>.svg for every node
src/                  The include root: includes are written "core/Box3DWorld.h" (bare name within a folder)
  core/               Houdini-free; built as b3dop_core and covered by the tests
    Box3DShapes.h       Header-only: shape math, packed-prim matrices, hashing
    Box3DJoints.h       Header-only: joint types, JointSettings, the joint parameter table
    Box3DSideFX.h/.cpp  SideFX's Glue / Hard / Spring / Bullet Soft / Cone Twist data -> Box3D joints (mapping, frame offsets)
    Box3DCpu.h/.cpp     Physical performance core count (the solver's Threads default)
    Box3DRbdObject.h    Header-only: an RBD Object's own mass / friction / bounce / Bullet Data shape -> whole-object settings
    Box3DWorld.h/.cpp   Pimpl wrapper over one b3World. The ONLY file that includes Box3D headers; defines TaskRunner
    Box3DAffectorMath.h Header-only: affector (Static Object) transform math
    Box3DDebug.h        B3D_LOG / counters, compiled out unless BOX3DDOP_DEBUG (args still type-checked, never evaluated)
  dop/                Houdini SIM_Data classes: the nodes and the world cache
    SIM_Box3DWorld.*    SIM_Data owning the shared world state (the rebuildable cache); the state itself is sync/Box3DState.h
    SIM_Box3DData.*     Default-settings DOP node
    SIM_Box3DConRel.*   Box3D <Type> Constraint Relationship DOP nodes (one per joint type), parameters from the table
    SIM_Box3DParms.h    Box3DParms: builds a node's parameter templates (tabs or simple folders, ranges, menus) from a table
    SIM_SolverBox3D.*   The solver node: parameters, node messages and the solve loop (the parts are in sync/)
    SIM_Box3DRegister.cpp  initializeSIM(); includes UT_DSOVersion.h (exactly once, keep it here)
  sync/               Houdini data -> the Box3D world, used by the solver
    Box3DState.h           Box3DSharedState: mirror entries, per-object sync records, affector bodies, joint entries, scratch
    Box3DSolverInternal.h  What the solver's parts share (namespace box3dsolver); only the solver's files include it
    Box3DShapeBuild.cpp    Packed primitives, collision source geometry (pieces, summary cache), buildShapes
    Box3DPointSync.cpp     One body per point: point state, fingerprint, body settings, shape plans, syncObject
    Box3DWholeObject.cpp   One body per DOP object (RBD Object etc.): sync and Position write-back
    Box3DPostStep.cpp      After the step: writeResultsBack, Impacts, point writes, aging, dead points
    Box3DForces.cpp        Force DOPs and point force / torque
    Box3DGuide.cpp         The solver's guide geometry: every body's collision shapes
    Box3DJointSync.*    Constraint networks -> Box3D joints: read, resolve names, create / update / remove, break, feedback
    Box3DAffectors.*    Affector objects (Static Object DOP etc.) -> static colliders
    Box3DHoudiniUtil.h  Shared Houdini helpers: writable geometry, object by id, 4x4 to array, Euler to quaternion,
                        options of SIM data by name (box3dDataOption)
    Box3DHoudiniTasks.h TaskRunner on Houdini's TBB pool (UT_TaskGroup). Only SIM_Box3DWorld.cpp includes it
examples/            make_examples.py: builds demo scenes into examples/scenes (git-ignored)
docs/agents/         Technical notes by area; CLAUDE.md is the index ("Where to look")
tests/
  KeyedWorld.h        Test-only Box3DWorld subclass: bodies by caller-chosen id, over the handle API
  core_smoke.cpp      Version print + world create/destroy
  core_physics.cpp    Unit + physics checks (CHECK macro, nonzero exit on failure)
  core_mesh.cpp       Static triangle meshes, mesh scale, decomposeScaledRigid
  core_heightfield.cpp Static height fields: pose, scale, ramps, holes, refused input
  core_threads.cpp    1 worker vs N workers: timing + determinism
  core_taskrunner.cpp Host-supplied TaskRunner shared by 40+ worlds: no extra threads, same results as 1 worker
  core_joints.cpp     Every joint type, world anchors, live updates, breaking, feedback, the parameter table
  core_sidefx.cpp     SideFX constraint data: option table, mapping per type / constraint_type, frame offsets, behaviour
  core_rbdobject.cpp  RBD Object settings without Box3D Data: defaults, invalid values, Bullet Data shapes, density -> mass
```

Layering (keep it):

1. Everything in `src/core/` knows nothing about Houdini, so the tests run without Houdini.
2. `SIM_*` files never include Box3D headers; they use `b3dop::Box3DWorld`.
3. `box3d` is linked `PRIVATE` into `b3dop_core`, so its defines (notably `BOX3D_DOUBLE_PRECISION`) cannot leak into
   plugin code.

## Conventions and gotchas

* A `SIM_DopDescription` must live in zero-filled static memory (a function-local static, or placement new into a
  static buffer as `SIM_Box3DConRel::describe` does), never on the heap: built with `std::make_unique`, the joint
  nodes' descriptions crashed Houdini 22 now and then while the node cooked (`SIM_DopDescription::computeLiveInput`).
  Building them in static memory fixed it (60 runs clean), consistent with a member its constructor leaves unset.
* Box3D headers only in `Box3DWorld.cpp`; `UT/UT_DSOVersion.h` only in `SIM_Box3DRegister.cpp`; new `SIM_Data` classes
  registered in `initializeSIM`.
* The solver's parts live in `namespace box3dsolver`; anything one part uses from another is declared in
  `Box3DSolverInternal.h` (default arguments there, not on the definitions). Small Houdini helpers used by several
  files go in `Box3DHoudiniUtil.h`, Houdini-free math (`kDegToRad`, `quatsDiffer`, `xformPoint`) in `Box3DShapes.h`.
* Node messages go through `SIM_SolverBox3D::message` / `warnCount` ("Box3D: " prefix, first object; counts only when
  non-zero). Whole objects count separately from points (`failedObjects`, `objectFallbacks`).
* Anything kept in the world must be reconstructible from point attributes, or a scrub / reload loses it.
* Hull keys must always imply identical body-local points (`createHullWithKey` trusts them).
* Sanitize every new input path: Box3D asserts on non-unit quaternions and negative material values.
* Row-vector matrices on the Houdini side; never mix with column-vector math.
* Style: clang-format's default (LLVM) style, pinned by the root `.clang-format`. Run `clang-format -i` on every
  `.cpp` / `.h` file you create or change under `src/` and `tests/` before finishing, and don't hand-format or
  hand-align around it. Never format `extern/` (it has its own `.clang-format` and is never edited). Keep comments
  inside macros on their own line: a trailing comment after a `\` continuation gets wrapped into the macro. A
  comment too long to sit after its code goes on its own line above it.
* Casts: `static_cast<T>(x)`, never `T(x)` or `(T)x` (clang-tidy's `google-readability-casting`).
* Before finishing a change: the core builds with no warnings (`-Wall -Wextra`), `ctest` passes, `clang-tidy -p build`
  is clean on the files you touched (`.clang-tidy`), and `python tools/check_docs.py` passes. CI
  (`.github/workflows/ci.yml`) runs the same checks except clang-tidy. Text files are UTF-8 with LF line endings
  (`.editorconfig`).
