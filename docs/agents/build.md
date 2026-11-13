# Build, tests, debug, package

How the project is built, tested and packaged. Index and rules: [CLAUDE.md](../../CLAUDE.md).

## Build system

* `BOX3DDOP_CORE_ONLY=ON` builds only `box3d`, `b3dop_core` and the tests, then `return()`s before any HDK use.
* Otherwise `find_package(Houdini)` (with `$HFS/toolkit/cmake`) and `houdini_configure_target`, which also copies the
  plugin into `~/houdiniX.Y/dso` (`Documents/houdiniX.Y/dso` on Windows). Close Houdini to overwrite it on Windows.
* A post-build step copies `icons/box3d.svg` to `<dso>/../config/Icons/DOP_<node name>.svg` (Houdini's default icon
  name for a node type is `<TABLE>_<name>`; the HDK alternative, `OP_Operator::setIconName`, needs the operator, which
  the automatic DOPs don't expose to us). Renaming a node's internal name means renaming its icon.
* Warnings: `b3dop_warnings(target)` adds `-Wall -Wextra` (`/W4` on MSVC) to `b3dop_core`, the tests and the plugin,
  never to Box3D. Houdini's headers come through an imported target, so they are system includes and not checked.
  `BOX3DDOP_WARNINGS_AS_ERRORS` (default OFF) adds `-Werror` / `/WX`; CI sets it on both platforms. A full MSVC
  build (plugin included, Houdini 22) was clean apart from test shadowing, since fixed.
* `CMAKE_EXPORT_COMPILE_COMMANDS` is on: `build/compile_commands.json` feeds clang-tidy (`.clang-tidy`) and IDEs. The
  plugin's sources only appear in it from a plugin (HDK) build.
* `format` / `format-check` targets run clang-format over `src/` and `tests/` (only when clang-format is found).
  `.githooks/pre-commit` rejects unformatted staged files once enabled with `git config core.hooksPath .githooks`.
* CI (`.github/workflows/ci.yml`): format check (clang-format pinned to 18.1.3; the default style shifts between
  versions), `tools/check_docs.py`, and the core-only build plus `ctest` on Linux and Windows. No HDK there, so no
  plugin build.
* `BOX3DDOP_DEBUG` (default OFF) defines `BOX3DDOP_DEBUG=1` for the plugin and `b3dop_core` (not Box3D). With it OFF nothing writes to stdout/stderr: `Box3DWorld` replaces Box3D's printf log handler and its own assert handler with `B3D_LOG`. The option is cached in `build/`, so a stale ON survives `build.bat`; reconfigure with `-DBOX3DDOP_DEBUG=OFF` or delete `build/`.
* `BOX3D_DIR` (default `extern/box3d`) must point at Box3D's own `CMakeLists.txt`.
* `BOX3DDOP_MAX_WORLDS` (default set in `CMakeLists.txt`, above Box3D's own) is passed to the `box3d` target as PUBLIC
  `B3_MAX_WORLDS` (Box3D wraps it in `#ifndef`). Each slot is a static `b3World` (several KB; `sizeof(b3World)` is the real figure).
  Rebuilds keep the previous world alive while cached frames share it, so worlds accumulate.
  `Box3DWorld::create` checks `b3GetWorldCount() >= B3_MAX_WORLDS` itself before `b3CreateWorld`, which would
  otherwise hit a `B3_ASSERT` (on in RelWithDebInfo) before failing. A failed rebuild returns false from
  `createFreshWorld` and the solve fails (the old state is valid but from another time and must not be continued);
  `reportWorldCreationFailure` names the limit when it is the cause.
* `-DBOX3D_DOUBLE_PRECISION=ON` works: positions cross the boundary as `double[3]` and `Box3DWorld.cpp` converts through
  `decltype(b3Pos::x)`.
* Release or RelWithDebInfo only. RelWithDebInfo turns on `B3_ENABLE_ASSERT`. Box3D's default assert handler breaks
  to the debugger (kills Houdini without one), so `Box3DWorld::create` installs `assertHandler` once
  (`b3SetAssertFcn`): logs to stderr (first 20), counts, returns 0 (no break). Box3D then carries on past the failed
  check, so results after an assert may be wrong, but Houdini survives.
* Inputs Box3D asserts on are guarded in the core too: velocities (`finiteVec3`: non-finite components -> 0) and
  gravity (`create`: non-finite components -> 0; `applyTuning`: non-finite gravity ignored, previous kept).
* `build.sh` wipes `build/` if the repo moved (`build/.configured_from`). `build.bat` finds `HFS` itself (env var, then
  registry `HKLM\SOFTWARE\Side Effects Software` ActiveVersion -> `Houdini <ver>\InstallPath`, then the newest
  `Program Files\Side Effects Software\Houdini X.Y.Z`). It still names the VS 2022 generator.

## Tests

Houdini checks: `hython tests/houdini/run_tests.py` (`--list` for the tests); on Windows `test_houdini.bat`, which
finds Houdini the same way as `build.bat` (its search block is copied: change both together) and passes its
arguments on. Each builds its scene from scratch
(no dependence on `box3d_test.hiplc`), simulates and prints PASS / FAIL / INFO; scenes are saved to
`tests/houdini/out/`. They cover items from "Untested and unverified" in [status.md](status.md): when a check passes
in Houdini, remove its item there. SETUP results mean the script guessed a node or parameter name wrong (the message
lists the real ones), not a plugin failure. Each test runs in its own hython process, so a crash is reported as CRASH
and the rest still run; the `probe_*` tests isolate the parts of the `network` scene. Not in CI (needs Houdini).
The tests are in `tests/houdini/checks_*.py` by area (core, solver / UI / caching,
outputs / packed geometry, constraints / whole objects), each with a `TESTS` dict the runner collects; helpers in
`tests/houdini/b3dtest.py`. A test function can carry `.env` (environment variables for its process, e.g.
`HOUDINI_MAXTHREADS=1`).

Core tests:

Built in both configurations (`-DBOX3DDOP_CORE_ONLY=ON` builds only these):

| Target | Checks |
|---|---|
| `b3dop_core_smoke` | Box3D version, create / destroy; expects `OK` |
| `b3dop_core_physics` | Handle API (and the test-side `KeyedWorld`), shape mapping, explicit primitives, mass, body types, resting heights, hulls (limits, caching, instancing), packed matrix math, `sanitizeQuat` / `storedPosition`, setters on existing bodies (material, damping, gravity, sleep, density, enable, shape replacement), live tuning, motion locks (world axes, live changes), fast rotation (the cap, live toggle), CCD threshold (sweep vs tunnel, live, invalid values), collision filtering (bits, positive / negative groups, re-filtering), contacts, mass centring / inertia scaling, sleep threshold, kinematic targets, external forces (force / torque per step, summing, non-finite input, static / kinematic / sleeping bodies, gravity as a force), contact recycling (world distance live, per-body flag at creation and on existing bodies, velocity kept, sleepers not woken) |
| `b3dop_core_joints` | Every joint type (weld holds, distance keeps length, revolute motor + live speed change, prismatic limit along a rotated frame, motor joint, wheel spin, filter), world anchors, refused joints, clamping (no Box3D asserts), breaking events, frames on moved bodies, live frames (a world rebuilt from `jointWorldFrames` matches the continued one), local frames (a joint remade from `jointLocalFrames` at x = 12000 is bit for bit the same), Box3D defaults, the parameter table, feedback names never equal setting names |
| `b3dop_core_sidefx` | SideFX data: data types and option table, `constraint_type` parsing, mapping of every type and mode (glue threshold, Spring's Hz conversion, cone twist axes / smaller cone), frame offsets (hold, separation, round trip), and in a world: glue holds the centroid offset and breaks, Soft all keeps its rest length, Soft / Spring sag as their spring constants predict, Hard position is a rigid rod, cone twist stops at the smaller limit |
| `b3dop_core_rbdobject` | RBD Object settings without Box3D Data (`rbdObjectSetup`): nothing read = Box3D Data's defaults, Houdini's RBD Object defaults, unusable values ignored, Bullet Data shapes (unknown names, none, compound, connected sets, AutoFit off box / sphere / capsule), density 1000 on a 1 m box = 1000 kg |
| `b3dop_core_taskrunner [workers]` | Worlds on one shared host pool: tasks run, results match 1 worker |
| `b3dop_core_threads [workers] [side] [steps]` | 1 vs N workers: speedup, fails if positions differ by > 1e-4 |

`tests/box3d_test.hiplc` is a manual scene for the `SIM_*` code, which has no automated tests. No CI.

## Debug logging (`Box3DDebug.h`)

Per-step diagnostics use `B3D_LOG_VERBOSE` / `B3D_LOG_IF(cond, ...)` and only print with `BOX3DDOP_VERBOSE=1` (a
flushed line per frame slows Houdini a lot); plain `B3D_LOG` is for one-off events (rebuilds, bodies added / removed /
rebuilt, fallback notes). All macros compile to nothing without `BOX3DDOP_DEBUG`; arguments used only by logs get
`(void)x`. Output: stderr, `[b3dop]` prefix. The only counter is `rebuilds`.

## Package

`CMakeLists.txt` has `install()` rules and CPack (zip) that lay out a Houdini package: `Box3D.json` (from
`package/Box3D.json.in`, `hpath` is `$HOUDINI_USER_PREF_DIR/packages/Box3D`) and `Box3D/dso`, `Box3D/config/Icons`. The
JSON is enabled only for the Houdini major.minor it was built with, taken from `Houdini_VERSION`; if that is not set
it is always enabled. Checked with a stub project, not with a real Houdini build.
