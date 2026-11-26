# CLAUDE.md

Technical notes for AI agents (and humans) working on this codebase: how the code works and why. Build and usage
instructions for users are in `README.md`; the user reference (attributes, shapes, joints) is in `docs/*.md`.

## What this is

A Houdini DOP plugin (HDK, C++17) wrapping [Box3D](https://github.com/erincatto/box3d) (Erin Catto's 3D rigid
body engine, C17, a git submodule in `extern/box3d`). It registers these `SIM_Data` classes:

| DOP name | Class | Role |
|---|---|---|
| Box3D Solver | `SIM_SolverBox3D` | Multi-object solver: one call per timestep with all its objects |
| Box3D Data | `SIM_Box3DData` | Per-object defaults (fallbacks for missing point attributes) |
| (hidden) Box3D World | `SIM_Box3DWorld` | Holds the live Box3D world; not a DOP node |
| Box3D <Type> Constraint Relationship | `SIM_Box3DConRel<Type>` | One per joint type: the joint's settings, named by `constraint_name` in a Constraint Network |
| Box3D Packed Object | (digital asset) | RBD Packed Object for Box3D: a SOP stage writes its settings as point attributes, built by `tools/make_packed_object.py` |
| Box3D Solver (SOP) | (digital asset) | SOP wrapping a DOP network with the nodes above, built by `tools/make_solver_sop.py` |
| Box3D Configure (SOP) | (digital asset) | Box3D Data's settings as point attributes, built by `tools/make_configure_sop.py` |
| Box3D Constraint Properties (SOP) | (digital asset) | Joint settings as primitive attributes, built by `tools/make_constraint_properties_sop.py` |

Design goals: Box3D's native performance (handle-based hot paths, no per-body hash lookups, threaded per-point passes
where safe) and Houdini Bullet behaviour (same attribute names and rules, as documented on the RBD Packed Object
page). The exception is sleeping and enabling, which follow Box3D's own model (`enablesleep`, `awake`, `enabled`; see
"Sleep and enable state"); Bullet's `bullet_want_deactivate`, `bullet_sleeping`, `bullet_ignore` and
`min_activation_impulse` are not read.

The code is largely LLM-written by an author still learning the HDK. Treat HDK usage with healthy skepticism.
No TODO or "unverified" notes in code comments, the README or other docs (see Documentation below).

## Documentation

Docs split by reader: `README.md` and `docs/*.md` are for people using the plugin in Houdini, `docs/agents/` is how the
code works. Before editing any of them read [docs/agents/doc-rules.md](docs/agents/doc-rules.md): the README rules are
hard limits (short, no implementation detail, the top part is the author's), and agent docs describe how the code works
now, not its history. What is unverified, broken or still to do lives only in `status.md` and `todo.md`.

## Where to look

Load only the file for the area you are changing.

| File | Covers |
|---|---|
| [docs/agents/doc-rules.md](docs/agents/doc-rules.md) | Rules for editing the README, `docs/` and these notes |
| [docs/agents/layout.md](docs/agents/layout.md) | File-by-file source layout |
| [docs/agents/build.md](docs/agents/build.md) | CMake options, build scripts, tests, benchmarks, debug logging, the package |
| [docs/agents/core.md](docs/agents/core.md) | `b3dop::Box3DWorld`: handle API, mass, filtering, shapes, hull cache, settings |
| [docs/agents/solver-state.md](docs/agents/solver-state.md) | The shared world state (`SIM_Box3DWorld`, `Box3DSharedState`) |
| [docs/agents/solver.md](docs/agents/solver.md) | The solver step: order, substeps, forces, Glue and joint stages |
| [docs/agents/solver-sync.md](docs/agents/solver-sync.md) | `syncObject`, the skip fingerprint, write-back, threading |
| [docs/agents/bodies.md](docs/agents/bodies.md) | Mid-simulation updates, building shapes |
| [docs/agents/body-features.md](docs/agents/body-features.md) | Per-attribute features: sleep, CCD, locks, filters, impacts, ... |
| [docs/agents/motion-limits.md](docs/agents/motion-limits.md) | Drag, speed and acceleration limit attributes, and on Glue as One Body |
| [docs/agents/forces-affectors.md](docs/agents/forces-affectors.md) | Force DOPs, point `force` / `torque`, Static Object colliders |
| [docs/agents/whole-object.md](docs/agents/whole-object.md) | RBD Object style bodies (one body per DOP object) |
| [docs/agents/joints.md](docs/agents/joints.md) | Joint types, settings table, `Box3DWorld` joint API |
| [docs/agents/joint-networks.md](docs/agents/joint-networks.md) | Constraint relationship nodes, networks, break and write-back |
| [docs/agents/joint-live-frames.md](docs/agents/joint-live-frames.md) | Live and local frames on the networks, upstream edits, moving world anchors |
| [docs/agents/joints-sidefx.md](docs/agents/joints-sidefx.md) | SideFX data (Glue, Hard, Spring, Soft, Cone Twist) mapped to joints |
| [docs/agents/glue-compound.md](docs/agents/glue-compound.md) | Glue as One Body (`gluecompound`) |
| [docs/agents/reference.md](docs/agents/reference.md) | Parameters, point attributes, node warnings, unsupported features |
| [docs/agents/assets.md](docs/agents/assets.md) | The digital assets' shared notes and the Box3D Packed Object |
| [docs/agents/solver-sop.md](docs/agents/solver-sop.md) | Solver SOP: inner DOP network, collision, ground, forces |
| [docs/agents/solver-sop-io.md](docs/agents/solver-sop-io.md) | Solver SOP: input preparation, outputs, World Scale, guide |
| [docs/agents/solver-sop-ui.md](docs/agents/solver-sop-ui.md) | Solver SOP: parameter tabs, joint settings, Break Thresholds, viewer state |
| [docs/agents/constraint-properties.md](docs/agents/constraint-properties.md) | Box3D Constraint Properties asset |
| [docs/agents/configure.md](docs/agents/configure.md) | Box3D Configure asset |
| [docs/agents/status.md](docs/agents/status.md), [status-areas.md](docs/agents/status-areas.md) | Unverified |
| [docs/agents/todo.md](docs/agents/todo.md) | TODO, features to add to the Box3D fork |

**Values that live in code stay in code.** Defaults, counts, thresholds and sizes are not copied into the docs: name
the constant or table and its file (`kMinParallelPoints` in `Box3DSolverInternal.h`, `kPropAttribNames` in
`Box3DAttribNames.h`). A number in a doc is only for a measurement or a fact about Box3D / Houdini that the code does
not hold. `python tools/check_docs.py` checks that the `k...` constants and the relative links the docs mention exist.

## Layout

The full file-by-file map is [docs/agents/layout.md](docs/agents/layout.md). In short:

* `src/core/` is Houdini-free (built as `b3dop_core`, covered by the C++ tests); `Box3DWorld` wraps one Box3D world.
* `src/dop/` holds the `SIM_Data` classes: the solver node, Box3D Data, the constraint relationship nodes.
* `src/sync/` moves Houdini data into the world each step: point and object sync, shapes, joints, compounds, forces,
  affectors, write-back. `Box3DState.h` is the shared state; `Box3DSolverInternal.h` what the solver's parts share.
* `tools/make_*.py` build the four digital assets; `tools/check_docs.py` checks the docs against the code.
* `tests/core_*.cpp` are the C++ tests (`ctest`); `tests/houdini/` the Houdini checks (`test.bat`, `run_tests.py`)
  and the benchmarks (`bench.py`).

Layering (keep it):

1. Everything in `src/core/` knows nothing about Houdini, so the tests run without Houdini.
2. `SIM_*` files never include Box3D headers; they use `b3dop::Box3DWorld`.
3. `box3d` is linked `PRIVATE` into `b3dop_core`, so its defines (notably `BOX3D_DOUBLE_PRECISION`) cannot leak into
   plugin code.

## Commands (Windows)

* Build: `build.bat` (add `debug` for debug logging), or `cmake --build build --config Release` (`--target Box3DDOP`
  for the plugin alone). Close Houdini first. The plugin lands in `C:\Users\<you>\Documents\houdini22.0\dso` and the
  assets in `...\otls`.
* C++ tests: `ctest --test-dir build -C Release`.
* Houdini checks: `test.bat <names>` (`test.bat --list` for the names; with none it runs everything, which is slow).
  `python -I tests/python/test_harness.py` needs no Houdini.
* Benchmarks: `python tools/b3d.py bench <scenario>[:key=value,...] --kinds box3d` (scenarios and options: build.md).
* Docs: `python tools/check_docs.py`.

**`HOME` decides where Houdini looks.** On Windows Houdini uses `%HOME%` as its preferences folder when it is set, and
Git Bash sets it. Then Houdini (the plugin install, `hython`) uses `C:\Users\<you>\houdini22.0`, where the plugin is
not, and creates that folder. The symptoms are a plugin built into `C:\Users\<you>\houdini22.0` instead of
`Documents\houdini22.0`, `hython` failing with "Invalid node type name" for `box3d_*` nodes, and the asset build
warning that no node types exist. `CMakeLists.txt` (configure and the asset step), `build.bat`, `test.bat` and
`tools/b3d.py` drop `HOME` themselves, so use them. Never start `hython` directly from Git Bash: run
`env -u HOME hython ...`, or `subprocess` with `HOME` removed from the environment. If a `C:\Users\<you>\houdini22.0`
folder appears, something ran Houdini with `HOME` set: delete the folder and find what did.

## Conventions and gotchas

* A `SIM_DopDescription` must live in zero-filled static memory (a function-local static, or placement new into a
  static buffer as `SIM_Box3DConRel::describe` does), never on the heap: heap ones crash Houdini 22 now and then in
  `SIM_DopDescription::computeLiveInput` (a member its constructor leaves unset).
* Box3D headers only in the `Box3DWorld*.cpp` files, `Box3DWorldInternal.h`, `Box3DWorldHulls.h` and
  (`math_functions.h` alone) `Box3DSideFX.cpp`; `UT/UT_DSOVersion.h` only in `SIM_Box3DRegister.cpp`; new `SIM_Data`
  classes registered in `initializeSIM`.
* The solver's parts live in `namespace box3dsolver`; anything one part uses from another is declared in
  `Box3DSolverInternal.h` (default arguments there, not on the definitions). Small Houdini helpers used by several
  files go in `Box3DHoudiniUtil.h`, Houdini-free math (`kDegToRad`, `quatsDiffer`, `xformPoint`) in `Box3DMath.h`,
  shape math and hashing in `Box3DShapes.h`.
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
* Before finishing a change: the core builds with no warnings (`-Wall -Wextra -Wshadow -Wconversion`, MSVC `/W4`),
  `ctest` passes, `clang-tidy -p build` is clean on the files you touched (`.clang-tidy`), and
  `python tools/check_docs.py` passes. CI (`.github/workflows/ci.yml`) runs the same checks except clang-tidy. Text
  files are UTF-8 with LF line endings (`.editorconfig`).
