# CLAUDE.md

Technical notes for AI agents (and humans) working on this codebase: how the code works and why.
Build and usage instructions for users are in `README.md`; the user reference (attributes, shapes, joints) is in `docs/*.md`.

## What this is

A Houdini DOP plugin (HDK, C++17) wrapping [Box3D](https://github.com/erincatto/box3d) (Erin Catto's 3D rigid
body engine, C17, a git submodule in `extern/box3d`). It registers these `SIM_Data` classes:

| DOP name | Class | Role |
|---|---|---|
| Box3D Solver | `SIM_SolverBox3D` | Multi-object solver: one call per timestep with all its objects |
| Box3D Data | `SIM_Box3DData` | Per-object defaults (fallbacks for missing point attributes) |
| (hidden) Box3D World | `SIM_Box3DWorld` | Holds the live Box3D world; not a DOP node |
| Box3D <Type> Constraint Relationship | `SIM_Box3DConRel<Type>` | One per joint type (Bond breaks by stress, `core/Box3DBond.h`): the joint's settings, named by `constraint_name` in a Constraint Network |
| Box3D Packed Object | (digital asset) | RBD Packed Object for Box3D: a SOP stage writes its settings as point attributes, built by `tools/make_packed_object.py` |
| Box3D Solver (SOP) | (digital asset) | SOP wrapping a DOP network with the nodes above, built by `tools/make_solver_sop.py` |
| Box3D Configure (SOP) | (digital asset) | Box3D Data's settings as point attributes, built by `tools/make_configure_sop.py` |
| Box3D Constraint Properties (SOP) | (digital asset) | Joint settings as primitive attributes, built by `tools/make_constraint_properties_sop.py` |

Design goals: Box3D's native performance (handle-based hot paths, no per-body hash lookups, threaded per-point passes
where safe) and Houdini Bullet behaviour (same attribute names and rules, as documented on the RBD Packed Object page).
The exception is sleeping and enabling, which follow Box3D's own model (`enablesleep`, `awake`, `enabled`; see "Sleep and
enable state"); Bullet's `bullet_want_deactivate`, `bullet_sleeping`, `bullet_ignore` and `min_activation_impulse` are not read.

The code is largely LLM-written by an author still learning the HDK. Treat HDK usage with healthy skepticism.
What is unverified, broken or still to do lives only in `docs/agents/status.md` ("Unverified", "TODO"):
no TODO or "unverified" notes in code comments, the README or other docs.

## Documentation split

`README.md` is for people using the plugin in Houdini: how to build it, which nodes and attributes exist, what they do,
and what behaves differently from Houdini's own solvers. Keep it short. Leave out anything a user cannot act on or would
not notice: how the solver does something, Box3D internals and terms (density, restitution, move events, hull caches),
performance mechanics. Those belong in `docs/agents/`. When a change adds a feature, add one line or table row to
the README if users need it, and put the explanation in the matching `docs/agents/` file.
The README holds the install, build and quick start, and links to the user reference in `docs/` (`attributes.md`,
`shapes.md`, `static-objects.md`, `joints.md`, `nodes.md`, `notes.md`); the README rules below apply to all of them.

README rules (hard limits, not suggestions):
- The top part, from the title through the Building section, is the author's: never change it unless asked.
  Everything after it is user reference: short, organized, no history.
- Never add more than one or two sentences, or one table row, per change. Do not grow a section to explain a feature.
- No implementation details: no function, class or HDK names, locks, fingerprints, step ordering, clearing or caching
  behaviour. If a sentence says how the code works, it goes in this file.
- No semicolons: write separate sentences. No trailing asides with details a user does not need (which node sets a
  value, how it is stored, what else happens behind the scenes). Keep what they act on, cut the rest.
- Do not rewrite or expand existing README text while making an unrelated change. Trim, don't pad.
- Before finishing a task, re-read any README edit and cut it down.

Rules for `docs/agents/` (and this file):
- Describe how the code works now, not how it got there: no history, no "was", "used to", "since fixed", "tried".
  A failed approach earns one clause only when it stops someone retrying it.
- Keep entries to a few lines; prefer a bullet to a paragraph. When something is settled, delete it from status.md
  rather than marking it done.
- One fact in one place: link to the file that owns it instead of repeating it.

UI tooltips (the help text on parameters) follow the same rule: one or two short sentences, under about 150
characters. Say what the control does and, if needed, when to change it. No implementation detail, no lists of point
attributes, no restating how the sibling controls work.

## Where to look

Load only the file for the area you are changing.

| File | Covers |
|---|---|
| [docs/agents/build.md](docs/agents/build.md) | CMake options, build scripts, tests, debug logging, the Houdini package |
| [docs/agents/assets.md](docs/agents/assets.md) | The digital assets' shared notes (menus, layout) and the Box3D Packed Object |
| [docs/agents/solver-sop.md](docs/agents/solver-sop.md) | Box3D Solver SOP: inner DOP network, collision, ground, forces, outputs, World Scale |
| [docs/agents/solver-sop-ui.md](docs/agents/solver-sop-ui.md) | Box3D Solver SOP: parameter tabs, joint settings, Break Thresholds, viewer state |
| [docs/agents/constraint-properties.md](docs/agents/constraint-properties.md) | Box3D Constraint Properties asset |
| [docs/agents/configure.md](docs/agents/configure.md) | Box3D Configure asset |
| [docs/agents/core.md](docs/agents/core.md) | `b3dop::Box3DWorld`: handle API, mass, filtering, shapes and the hull cache, world settings |
| [docs/agents/solver.md](docs/agents/solver.md) | Shared world state, step order, `syncObject`, the skip fingerprint, write-back, threading |
| [docs/agents/bodies.md](docs/agents/bodies.md) | Mid-simulation updates, per-attribute features (sleep, CCD, locks, filters, impacts, ...), building shapes |
| [docs/agents/forces-affectors.md](docs/agents/forces-affectors.md) | Force DOPs, point `force` / `torque`, Static Object colliders |
| [docs/agents/whole-object.md](docs/agents/whole-object.md) | RBD Object style bodies (one body per DOP object) |
| [docs/agents/joints.md](docs/agents/joints.md) | Constraint networks, joint nodes, live frames |
| [docs/agents/joints-sidefx.md](docs/agents/joints-sidefx.md) | SideFX constraint data (Glue, Hard, Spring, Soft, Cone Twist) mapped to joints, frame offsets |
| [docs/agents/glue-compound.md](docs/agents/glue-compound.md) | Glue as One Body (`gluecompound`) |
| [docs/agents/reference.md](docs/agents/reference.md) | Parameters, point attributes, node warnings, what is not supported, the parameter-tab builder |
| [docs/agents/status.md](docs/agents/status.md) | Unverified, TODO |

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
extern/box3d/         Box3D submodule, the author's fork (MysteryPancake/Box3D-Fork; static lib, EXCLUDE_FROM_ALL)
icons/box3d.svg     Node icon; the build copies it to config/Icons as DOP_<node name>.svg for every node
src/                  The include root: includes are written "core/Box3DWorld.h" (bare name within a folder)
  core/               Houdini-free; built as b3dop_core and covered by the tests
    Box3DMath.h         Header-only: unit constants, finite checks, small copies, dot / cross / length of 3-vectors, quaternion helpers, xformPoint
    Box3DShapes.h       Header-only: shape math, packed-prim matrices, hashing
    Box3DQuadrics.h     Header-only: surface points for Houdini's quadric primitives (Sphere, Tube, Circle, Metaball)
    Box3DJoints.h       Header-only: joint types, JointSettings, the joint parameter table
    Box3DSideFX.h/.cpp  SideFX's Glue / Hard / Spring / Bullet Soft / Cone Twist data -> Box3D joints (mapping, frame offsets)
    Box3DGlue.h         Header-only: SideFX Glue's impulse model (spread over bonds, half-life, break on strength)
    Box3DBond.h         Header-only: the Bond's stress ratio (force and torque over the contact area), tear impulse, area
    Box3DBondNetwork.h  Header-only: the load of each bond in a rigid group, as a spring network (conjugate gradient)
    Box3DPatch.h        Header-only: the contact patch of two pieces' convex faces (area, centroid, normal, moments)
    Box3DCompound.h     Header-only: glue components (union-find), merging pieces into one body and splitting its velocity
    Box3DCpu.h/.cpp     Physical performance core count (the solver's Threads default)
    Box3DRbdObject.h    Header-only: an RBD Object's own mass / friction / bounce / Bullet Data shape -> whole-object settings
    Box3DWorld.h/.cpp   Pimpl wrapper over one b3World: bodies, shapes, hulls, settings; defines TaskRunner
    Box3DWorldJoints.cpp  Box3DWorld's joint methods
    Box3DWorldInternal.h  The world's private state (Impl) shared by those two .cpp files; with them (and `Box3DSideFX.cpp`, for `<box3d/math_functions.h>`) the only files that include Box3D headers
    Box3DAffectorMath.h Header-only: affector (Static Object) transform math
    Box3DDebug.h        B3D_DEBUG_LOG / counters, compiled out unless BOX3DDOP_DEBUG (args still type-checked, never evaluated)
  dop/                Houdini SIM_Data classes: the nodes and the world cache
    SIM_Box3DWorld.*    SIM_Data owning the shared world state (the rebuildable cache); the state itself is sync/Box3DState.h
    SIM_Box3DData.*     Default-settings DOP node
    SIM_Box3DConRel.*   Box3D <Type> Constraint Relationship DOP nodes (one per joint type), parameters from the table
    SIM_Box3DParms.h    Box3DParms: builds a node's parameter templates (tabs or simple folders, ranges, menus) from a table
    SIM_SolverBox3D.*   The solver node: parameters, node messages and the solve loop (the parts are in sync/)
    SIM_Box3DRegister.cpp  initializeSIM(); includes UT_DSOVersion.h (exactly once, keep it here)
  sync/               Houdini data -> the Box3D world, used by the solver
    Box3DState.h           Box3DSharedState: mirror entries, per-object sync records, affector bodies, joint entries, scratch
    Box3DAttribNames.h     The point attributes the solver reads, by name (PointAttrs and the prop markers come from it)
    Box3DSolverInternal.h  What the solver's parts share (namespace box3dsolver); only the solver's files include it
    Box3DSourceSync.cpp    Animated bodies following their source SOP (`followSourceSops`)
    Box3DShapeBuild.cpp    Packed primitives, collision source geometry (pieces, summary cache), buildShapes
    Box3DPointSync.cpp     One body per point: point state, fingerprint, body settings, shape plans, syncObject
    Box3DWholeObject.cpp   One body per DOP object (RBD Object etc.): sync and Position write-back
    Box3DPostStep.cpp      After the step: writeResultsBack, Impacts, point writes, aging, dead points
    Box3DForces.cpp        Force DOPs and point force / torque
    Box3DGuide.cpp         The solver's guide geometry: every body's collision shapes
    Box3DJointSync.*    Constraint networks -> Box3D joints: read, resolve names, create / update / remove
    Box3DCompoundSync.* Glued groups as one compound body (`gluecompound`): merge, dissolve, pieces' moves, forces, glue hits
    Box3DJointBreak.cpp After the step: Glue, breaking, feedback and live frames written onto the networks
    Box3DJointNetwork.h What those two share: attribute names, network geometry and fingerprint helpers
    Box3DAffectors.*    Affector objects (Static Object DOP etc.) -> static colliders
    Box3DHoudiniUtil.h  Shared Houdini helpers: writable geometry, object by id, 4x4 to array, Euler to quaternion,
                        options of SIM data by name (box3dDataOption), the threaded loop (box3dParallelFor)
    Box3DHoudiniTasks.h TaskRunner on Houdini's TBB pool (UT_TaskGroup). Only SIM_Box3DWorld.cpp includes it
examples/            make_examples.py: builds demo scenes into examples/scenes (git-ignored)
package/             Box3D.json.in: the Houdini package file the build configures
tools/               make_assets.py (the four asset builders in one hython, as the build does), make_packed_object.py,
                     make_configure_sop.py, make_constraint_properties_sop.py, make_solver_sop.py (the assets; its inner builders are
                     b3d_solver_*.py, the SOPs' viewer states b3d_*_state.py, read as text by the builders), b3d.py (finds Houdini; runs
                     the checks, examples, benchmarks or ctest on any OS), b3d_hda.py (helpers the builders share), b3d_prepare.py
                     (input packing), b3d_materials.py, b3d_settings.py, b3d_layouts.py (node placement), find_houdini.bat,
                     check_docs.py, houdini_network_dump.py (dumps a node's networks to text)
docs/agents/         Technical notes by area; CLAUDE.md is the index ("Where to look")
tests/
  core_*.cpp          One exe per area: smoke, physics (with physics_shapes.cpp and physics_bodies.cpp), mesh, heightfield, ground,
                      quadrics, threads, taskrunner, worlds, joints, glue, bond, patch, sidefx, compound, rbdobject
  TestUtil.h, WorldTestUtil.h, KeyedWorld.h   Shared helpers: CHECK macro, box / joint / step helpers, a world keyed by caller ids
  houdini/            Houdini checks: run_tests.py (runner), checks_*.py (tests by area), b3dtest.py (helpers),
                      review.py (tidies saved scenes), baseline.json; out/ holds each test's scene (git-ignored).
                      bench.py + bench_scenes.py: timing runs of Box3D against Bullet, outside run_tests.py (build.md)
  python/             test_harness.py: checks on the Houdini scripts that need no Houdini (stand-in `hou`)
```

Layering (keep it):

1. Everything in `src/core/` knows nothing about Houdini, so the tests run without Houdini.
2. `SIM_*` files never include Box3D headers; they use `b3dop::Box3DWorld`.
3. `box3d` is linked `PRIVATE` into `b3dop_core`, so its defines (notably `BOX3D_DOUBLE_PRECISION`) cannot leak into
   plugin code.

## Conventions and gotchas

* A `SIM_DopDescription` must live in zero-filled static memory (a function-local static, or placement new into a
  static buffer as `SIM_Box3DConRel::describe` does), never on the heap: heap ones crash Houdini 22 now and then in
  `SIM_DopDescription::computeLiveInput` (a member its constructor leaves unset).
* Box3D headers only in `Box3DWorld.cpp`, `Box3DWorldJoints.cpp`, `Box3DWorldInternal.h` and (`math_functions.h` alone) `Box3DSideFX.cpp`; `UT/UT_DSOVersion.h` only in `SIM_Box3DRegister.cpp`; new `SIM_Data` classes
  registered in `initializeSIM`.
* The solver's parts live in `namespace box3dsolver`; anything one part uses from another is declared in
  `Box3DSolverInternal.h` (default arguments there, not on the definitions). Small Houdini helpers used by several
  files go in `Box3DHoudiniUtil.h`, Houdini-free math (`kDegToRad`, `quatsDiffer`, `xformPoint`) in `Box3DMath.h`, shape math and hashing in
  `Box3DShapes.h`.
* Node messages go through `SIM_SolverBox3D::message` / `warnCount` (no prefix: they show on the solver's node; first
  object; counts only when non-zero). Whole objects count separately from points (`failedObjects`, `objectFallbacks`).
* Anything kept in the world must be reconstructible from point attributes, or a scrub / reload loses it.
* Hull keys must always imply identical body-local points (`createHullWithKey` trusts them).
* Sanitize every new input path: Box3D asserts on non-unit quaternions and negative material values.
* Row-vector matrices on the Houdini side; never mix with column-vector math.
* Style: clang-format's default (LLVM) style, pinned by the root `.clang-format`. Run `clang-format -i` on every
  `.cpp` / `.h` file you create or change under `src/` and `tests/` before finishing, and don't hand-format or
  hand-align around it. Before committing, format the code with `cmake --build build --target format`; CI checks it
  with `--target format-check`. Never format `extern/` (it has its own `.clang-format`). Keep comments
  inside macros on their own line: a trailing comment after a `\` continuation gets wrapped into the macro. A
  comment too long to sit after its code goes on its own line above it.
* Casts: `static_cast<T>(x)`, never `T(x)` or `(T)x` (clang-tidy's `google-readability-casting`).
* Before finishing a change: the core builds with no warnings (`-Wall -Wextra -Wshadow -Wconversion`, MSVC `/W4`), `ctest` passes, `clang-tidy -p build`
  is clean on the files you touched (`.clang-tidy`), and `python tools/check_docs.py` passes. CI
  (`.github/workflows/ci.yml`) runs the same checks except clang-tidy. Text files are UTF-8 with LF line endings
  (`.editorconfig`).
