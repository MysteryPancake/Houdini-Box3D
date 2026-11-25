# Build, tests, debug, package

How the project is built, tested and packaged. The digital assets' own notes are in [assets.md](assets.md), [solver-sop.md](solver-sop.md), [solver-sop-ui.md](solver-sop-ui.md), [constraint-properties.md](constraint-properties.md) and [configure.md](configure.md). Index and rules: [CLAUDE.md](../../CLAUDE.md).

## Build system

* `BOX3DDOP_CORE_ONLY=ON` builds only `box3d`, `b3dop_core` and the tests, then `return()`s before any HDK use.
* Otherwise `find_package(Houdini)` (with `$HFS/toolkit/cmake`) and `houdini_configure_target`, which builds the plugin
  into `~/houdiniX.Y/dso` (`Documents/houdiniX.Y/dso` on Windows; close Houdini to overwrite it there).
* Post-build steps:
  * `icons/box3d.svg` to `<dso>/../config/Icons/DOP_<node name>.svg` for every node in `BOX3DDOP_NODES` (Houdini's
    default icon name is `<TABLE>_<name>`; renaming a node means renaming its icon). The three SOP assets reuse DOP
    icons (`ICON` in their scripts), so they are not in the list.
  * The digital assets (`BOX3DDOP_MAKE_HDA`, default ON), with hython (found at `${Houdini_DIR}/../../bin` or
    `$HFS/bin`), into `<dso>/../otls/`: one `tools/make_assets.py` run (one hython session) builds
    `tools/make_packed_object.py` (`box3d_packedobject.hda`), `tools/make_configure_sop.py` (`box3d_configure.hda`),
    `tools/make_constraint_properties_sop.py` (`box3d_constraint_properties.hda`), then `tools/make_solver_sop.py` (`box3d_solver_sop.hda`, which contains the Packed Object). It is the custom target
    `Box3DAssets` (part of ALL), re-run when the plugin relinks or any `tools/make_*.py` / `b3d_*.py` changes; a stamp
    file in the build folder records it (OUTPUT cannot hold the dso-relative path). Without hython a configure warning
    names the command to run by hand. The assets copy parameters from the plugin just built ([assets.md](assets.md)).
* Warnings: `b3dop_warnings(target)` adds `-Wall -Wextra -Wshadow -Wconversion` (`/W4` on MSVC) to our targets, never to
  Box3D; Houdini's headers are system includes. `BOX3DDOP_WARNINGS_AS_ERRORS` (default OFF, on in CI) adds `-Werror` /
  `/WX`.
* `BOX3DDOP_PCH` (default ON) precompiles the HDK and standard headers most plugin files include (`target_precompile_headers`
  on `Box3DDOP`: no Box3D headers, no `UT_DSOVersion.h`); turn it off if a header ever needs a different include order.
* `CMAKE_EXPORT_COMPILE_COMMANDS` feeds clang-tidy (`.clang-tidy`); the plugin's files only appear in it after an HDK
  build. `format` / `format-check` targets run clang-format; `.githooks/pre-commit` checks staged files once enabled
  (`git config core.hooksPath .githooks`).
* CI (`.github/workflows/ci.yml`): format check (clang-format 18.1.3), `tools/check_docs.py`, core-only build and
  `ctest` on Linux and Windows. No HDK, so no plugin build.
* `BOX3DDOP_DEBUG` (default OFF) defines `BOX3DDOP_DEBUG=1` for the plugin and `b3dop_core`; with it off nothing writes
  to stdout / stderr. The value is cached in `build/`: reconfigure or delete `build/` to clear a stale ON.
* `BOX3D_DIR` (default `extern/box3d`) points at Box3D's `CMakeLists.txt`.
* `BOX3DDOP_MAX_WORLDS` is passed to `box3d` as PUBLIC `B3_MAX_WORLDS`. Rebuilds keep old worlds alive while cached
  frames share them; `Box3DWorld::create` checks the count itself (Box3D would assert), and a failed rebuild fails the
  solve (`reportWorldCreationFailure` names the limit).
* `-DBOX3D_DOUBLE_PRECISION=ON` works: positions cross the boundary as `double[3]`.
* Release or RelWithDebInfo only. RelWithDebInfo enables `B3_ENABLE_ASSERT`; `Box3DWorld::create` installs an assert
  handler that logs and continues (Box3D's default breaks to the debugger and kills Houdini). Inputs Box3D asserts on
  (non-finite velocities, gravity) are sanitized in the core.
* Linux follows SideFX's HDK CMake route (`Box3DDOP.so`, no `lib` prefix, into `$HOME/houdiniX.Y/dso`). Use the gcc
  version named in the Houdini download; never link an allocator (Houdini overrides malloc). `BUILD_TYPE=RelWithDebInfo
  ./build.sh` for gdb: `gdb $HFS/bin/houdini-bin`, `run -foreground`; `HOUDINI_DSO_ERROR=1` says why a plugin failed to
  load.
* Single-config generators default to `CMAKE_BUILD_TYPE=Release` when none is given.
* `build.sh` and `build.bat` both wipe `build/` if the repository moved (`build/.configured_from`, rewritten every
  build). `build.sh` finds `HFS` (env var, else the newest `/opt/hfs*` or macOS `/Applications/Houdini` install);
  `build.bat` (env var, then the registry's ActiveVersion, then the newest `Program Files\Side Effects Software\Houdini
  X.Y.Z`, in `tools\find_houdini.bat`, which `test.bat` and `make_examples.bat` share).
* `python tools/b3d.py test|examples|ctest|hfs` is the cross-platform launcher (`tools/b3d.py`: the same HFS search
  for Windows, Linux and macOS, then hython on `tests/houdini/run_tests.py` or `examples/make_examples.py`, or
  `ctest`). Its search duplicates `find_houdini.bat` and `build.sh`: change all three together.


## Example scenes

`examples/make_examples.py` (or `make_examples.bat`; `--check` also simulates each and prints timings) builds the demo
scenes listed in its docstring into `examples/scenes/<number>_<name>.hip` (git-ignored), each from ordinary SOPs and one
Box3D Packed Object, with a sticky note saying what to try. Not part of the build or the tests.

## Tests

Houdini checks: `hython tests/houdini/run_tests.py` (test names, `--list`, `--failed`, `--info`, `--repeat N`,
`--update-baseline`), or `test.bat` on Windows. Each test builds its scene from scratch in its own hython process (a
crash, or no result after 30 minutes, is reported as CRASH; an exception as ERROR), prints PASS / FAIL / INFO, and
saves the scene to `tests/houdini/out/` tidied for opening by hand (`review.py`, with a sticky
note holding the test's docstring and results, so write docstrings for a reader of the scene). SETUP means the script
guessed a node or parameter name wrong; the message lists the real ones. The last status of each test is in
`tests/houdini/out/last_results.json`.

In Git Bash run hython with `HOME` unset (`unset HOME`): Houdini takes `HOME` as its preferences folder, so the plugin and assets in `Documents/houdiniX.Y` are not found and every test reports SETUP or a missing node type (`test.bat` does this itself).

Tests are in `tests/houdini/checks_*.py` by area, each module with a `TESTS` dict listed in `MODULES`
(`run_tests.py`); helpers in `b3dtest.py`. The exit code is 1 if anything failed, errored or crashed. A test
function can carry `.env` (environment for its process, e.g. `HOUDINI_MAXTHREADS=1`). When a check settles an item in
[status.md](status.md), delete the item.

`baseline` (`checks_regress.py`) compares one fixed mixed scene on frame 48 with `tests/houdini/baseline.json` (1e-3).
After an intended change of behaviour, or of `baseline_scene`, re-record it with `--update-baseline`.

`checks_bullet.py` runs one scene in Box3D and in RBD Bullet Solver and compares the numbers (`solver_for`).
`python -I tests/python/test_harness.py` needs no Houdini: it loads the Houdini checks against a stand-in `hou` and checks
that every module is registered and every test named and documented, lint (pyflakes, if installed), the pure helpers, that
the docs name tests that exist, `baseline.json`, and that no file mixes line endings.

Core tests (`ctest`; each `tests/core_<name>.cpp` is `b3dop_core_<name>`, see `CMakeLists.txt`): smoke, physics
(handle API, shapes, mass, setters, CCD, filtering, forces, ...), mesh, heightfield, ground, quadrics, joints,
sidefx, glue, rbdobject, worlds, taskrunner and threads (`[workers] ...` arguments; threads fails if 1 vs N workers
differ by more than 1e-4). The file's header comment says what it covers. The physics cases are in `core_physics.cpp`
(case list and `main`), `physics_shapes.cpp` and `physics_bodies.cpp`, declared in `PhysicsCases.h`.

## Benchmarks

`hython tests/houdini/bench.py` (or `python tools/b3d.py bench`) times scenes in Box3D and RBD Bullet Solver; it is not
part of `run_tests.py` and nothing in it passes or fails. `--list` names the scenarios (`bench_scenes.py`:
pile, edit, collider, animated, joints, wall, fracture, bondwall, objects, static, churn, solversop, hulls, features), each with parameters (`name:key=value,...`) and a default sweep.

* Each run is its own hython process: frames 1..N timed as `setFrame` plus the DOP cook, grouped into the scene's
  phases (median, p95, max per frame), then Reset Simulation and frame 2 again (`rewind`: the world rebuild; the forced `cook` is
  what makes it re-solve). Frame 1 is its own `first` phase: the scene's SOPs cook there and Box3D builds nothing yet
  (new objects are solved from frame 2).
* `hou.perfMon` records the solvers' stages (`Pre-Solve/...`, `Stepping Simulation/...`, `Post-Solve/...` are nested
  under the solver node; Bullet has none, only its node time) and the time of every DOP node. Stepping Simulation/Box3D Step is the time of Box3D's `world.step` (`solver.md`); the rest of the solver's stages are the plugin's own code, the part to optimise. A frame's total minus its nodes is the DOP engine.
* Results go to `tests/houdini/out/bench/` (`latest.json` and a timestamped copy); `--compare file.json` prints the
  ratio per phase. `--repeat N` keeps the median run, `--threads N` sets `HOUDINI_MAXTHREADS`, `--budget` caps a run.
* `fracture` is a Voronoi-fractured cube (Points from Volume) glued by the fracture's own constraints, dropped on the ground, through the Box3D Solver SOP or the RBD Bullet Solver SOP (`sep`: smaller is more pieces). Its frame time includes the SOP's Break Thresholds.
* `bondwall` is the wall on Box3D Bonds (Box3D only; `wait=60,sleep=0` stands it awake for 60 frames before the ball). The Bond load solve and patch work run in Stepping Simulation: the
  plugin's share is `Breaking Bonds` and `Glue Hits`.
* `objects` is the whole-object path (RBD Objects from an RBD Point Object), `static` a pile on a Static Object (`motion`
  none / move / deform), `churn` bodies dying (Enable Aging, every piece carrying an `id`: without one the point number is the body, and each death renumbers and teleports the bodies after it) and, with `births 1`, appearing each frame (a SOP Solver; Box3D
  only), and `solversop` the Box3D Solver SOP asset: its frame is the SOP cook (`Scene.sop`), with the inner DOP network
  as `dop`, so the asset's own SOP nodes are in the frame but not in the stage list.
* Copy to Points takes no point attributes onto packed copies unless `targetattribs` / `applyattribs1` are set
  (`copy_packed`): without it every body is named by number and dynamic, and a constraint network matches nothing.
* A SideFX Hard / Soft / Spring node's Rest Length defaults to 1: anchors that start closer are pulled to it, which
  whips a chain of 0.5 m links across the scene in both solvers (Box3D harder). The `joints` scenes set it to 0.
* A scratch file named like a standard module (`types.py`) next to a hython script crashes hython at startup.

## Debug logging (`Box3DDebug.h`)

`B3D_DEBUG_LOG` for one-off events (rebuilds, bodies added / removed, fallbacks); `B3D_DEBUG_LOG_VERBOSE` per step, printed only
with `BOX3DDOP_VERBOSE=1`; `B3D_DEBUG_LOG_IF(cond, ...)` always when `cond`, else only when verbose. All compile to nothing
without `BOX3DDOP_DEBUG`. Output: stderr, `[b3dop]`; `run_tests.py` drops a test's stderr, so set `BOX3DDOP_TEST_LOG=1`
to print its `[b3dop]` lines. Other environment variables of debug builds (the header lists
them): `BOX3DDOP_REQUESTED_CACHE` (what `getRequestedCacheSubclass` returns), `BOX3DDOP_PROFILE=1` (the Bond phases'
time each step, `B3D_DEBUG_PROFILE` / `B3D_DEBUG_PROFILE_REPORT`) and `BOX3DDOP_VERIFY_READBACK=1`
([solver.md](solver.md), writeResultsBack).

## Package

`install()` rules and CPack (zip) lay out a Houdini package: `Box3D.json` (from `package/Box3D.json.in`, enabled only
for the Houdini major.minor it was built with) and `Box3D/dso`, `Box3D/config/Icons`, `Box3D/otls`.
