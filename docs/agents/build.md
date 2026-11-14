# Build, tests, debug, package

How the project is built, tested and packaged. Index and rules: [CLAUDE.md](../../CLAUDE.md).

## Build system

* `BOX3DDOP_CORE_ONLY=ON` builds only `box3d`, `b3dop_core` and the tests, then `return()`s before any HDK use.
* Otherwise `find_package(Houdini)` (with `$HFS/toolkit/cmake`) and `houdini_configure_target`, which builds the plugin
  into `~/houdiniX.Y/dso` (`Documents/houdiniX.Y/dso` on Windows; close Houdini to overwrite it there).
* Post-build steps:
  * `icons/box3d.svg` to `<dso>/../config/Icons/DOP_<node name>.svg` for every node in `BOX3DDOP_NODES` (Houdini's
    default icon name is `<TABLE>_<name>`; renaming a node means renaming its icon).
  * The Box3D Packed Object asset (`BOX3DDOP_MAKE_HDA`, default ON): `tools/make_packed_object.py` under hython (found
    at `${Houdini_DIR}/../../bin` or `$HFS/bin`) writes `<dso>/../otls/box3d_packedobject.hda`. See below.
* Warnings: `b3dop_warnings(target)` adds `-Wall -Wextra -Wshadow -Wconversion` (`/W4` on MSVC) to our targets, never to
  Box3D; Houdini's headers are system includes. `BOX3DDOP_WARNINGS_AS_ERRORS` (default OFF, on in CI) adds `-Werror` /
  `/WX`.
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
* `build.sh` wipes `build/` if the repository moved (`build/.configured_from`). `build.bat` finds `HFS` (env var, then
  the registry's ActiveVersion, then the newest `Program Files\Side Effects Software\Houdini X.Y.Z`); `test.bat` and
  `make_examples.bat` copy that block, so change all three together.

## Box3D Packed Object asset

`tools/make_packed_object.py` builds it from the plugin just compiled, so it always matches Box3D Data. Inside: Empty
Object -> SOP Geometry (data name `Geometry`) -> Box3D Data, which is the output (no input). Parameters: an Object tab
(Object Name `$OS`, Creation Frame, Solve on Creation Frame, SOP Path, Use Object Transform), then each Box3D Data
folder as a tab, every inner parameter a channel reference to the outer one. No Colliders label, so the solver syncs
it per point. The script prints one line, or warnings, and never fails the build.

Traps it handles:
* Parameters are found by label, skipping the `parmop_*` menus that share their labels.
* Empty Object has no creation frame: its Activation is `$SF == ch("../createframe")`, as RBD Packed Object's.
* Every inner `parmop_*` is set to Set Initial: a new data node starts at Use Default, and only an edit (not a channel
  reference) switches it, so the node would otherwise keep its built-in values.
* Box3D Data's own DOP settings outside its folders (Activation, Group, Data Name) are not promoted: promoted copies
  kept it from attaching.
* A second Box3D Data after the asset only changes Set Always parameters: Set Initial applies when the data is created,
  and the asset's already has been.

## Box3D Solver SOP asset

`tools/make_solver_sop.py <out.hda> [<packedobject.hda>]` builds `box3d_solver` in the SOP table (the DOP of the same
name is another table), after the Packed Object, which it installs first. The module docstring draws the network.
Four inputs, four matching outputs: Geometry, Constraints, Proxy, Collision.
* A SOP-level DOP network whose inputs 0-2 are the simulated pieces (Geometry, or Proxy when connected), Constraints
  and Collision, read inside with `dopcontextgeo(".", i)`.
  Pieces and collision are Box3D Packed Objects, Use Object Transform off (SOP space). Unconnected inputs are switched
  out by point count, so an empty input counts as unconnected everywhere alike: SOP Switches with the input's `IN_*`
  null as spare input and `npoints(-1) > 0`, DOP Switches with `npoints(opinputpath("..", i)) > 0` (the DOP
  network's inputs; collision also needs Use External Collisions), as RBD Bullet Solver does.
* Collision: Default Shape Convex Hull. Animated Collision (default on) drives Body Type (Kinematic / Static) and every
  Parameter Operation of the collision Packed Object's inner SOP Geometry (Set Always / Set Initial), so the SOP is
  re-imported each step and the solver turns pose changes into kinematic targets. That nested Packed Object is made
  editable for this; the asset is rebuilt every build anyway. A point with `active = 0` and no `animated` stays static.
* The Constraint Network DOP uses Second Context Geometry and Attach Internal Constraints to Object (as RBD Bullet
  Solver), so the network is the pieces object's `ConstraintGeometry`. Only the pieces go through it, the collision
  object is merged after: a second object would get its own copy and make every joint twice. Every relationship in
  `RELATIONSHIPS` (ours and SideFX's) is merged into it with default Data Names, so any `constraint_name` finds one.
  Their parameters are not promoted: set values per primitive (floats multiply the node's).
* Proxy: Transform Pieces moves Geometry by `name` from the proxy as connected (rest) to the simulated proxy, wired
  straight in (it reads packed transforms itself).
* Constraints output: DOP Import of the pieces object, Geometry Data Path `ConstraintGeometry`, so it carries `broken`,
  `force` and the rest.
* Parameters: Reset Simulation and Start Frame above the tabs, then the solver's tabs (folders found under its Data
  Options, a tab of simple folders such as Misc kept whole; Guide Options skipped; `TAB_RENAMES` / `TAB_MOVES` cover older plugins). Tab order: Solver, Collision, Forces,
  Pieces, the solver's other tabs (Misc), Advanced last. Solver: the solver's own settings, Cache Simulation, then the
  solver's Collision tab (contact settings) as a Contacts folder. Collision: Use External Collisions
  (`usecollisions`), Animated Collision and the collision object's Shape, Material and Sleep & Contacts folders (`COLLIDER_FOLDERS`,
  all named `collision_*`). Pieces: the pieces' Box3D Data folders; names the solver uses get `piece_`
  (`enablesleep`). Conditions are rewritten with the names. Inner solver `parmop_*` set to
  Set Initial, as in the Packed Object.
* Input / output labels are written into the DialogScript after the last `updateFromNode` (no hou setter). Wire
  colours come from the subnet's indirect input nodes (`INPUT_COLORS`: the constraints input pink, and its IN /
  OUT nodes).

## Tests

Houdini checks: `hython tests/houdini/run_tests.py` (`--list`, `--failed`, `--info`, `--update-baseline`), or `test.bat`
on Windows. Each test builds its scene from scratch in its own hython process (a crash is reported as CRASH), prints
PASS / FAIL / INFO, and saves the scene to `tests/houdini/out/` tidied for opening by hand (`review.py`, with a sticky
note holding the test's docstring and results, so write docstrings for a reader of the scene). SETUP means the script
guessed a node or parameter name wrong; the message lists the real ones. The last status of each test is in
`tests/houdini/out/last_results.json`.

Tests are in `tests/houdini/checks_*.py` by area, each module with a `TESTS` dict; helpers in `b3dtest.py`. A test
function can carry `.env` (environment for its process, e.g. `HOUDINI_MAXTHREADS=1`). When a check settles an item in
[status.md](status.md), delete the item.

`baseline` (`checks_regress.py`) compares one fixed mixed scene on frame 48 with `tests/houdini/baseline.json` (1e-3).
After an intended change of behaviour, or of `baseline_scene`, re-record it with `--update-baseline`.

Core tests (`ctest`; each `tests/core_<name>.cpp` is `b3dop_core_<name>`, see `CMakeLists.txt`): smoke, physics
(handle API, shapes, mass, setters, locks, CCD, filtering, forces, ...), mesh, heightfield, joints, sidefx, rbdobject,
taskrunner and threads (`[workers] ...` arguments; threads fails if 1 vs N workers differ by more than 1e-4). The
file's header comment says what it covers.

## Debug logging (`Box3DDebug.h`)

`B3D_LOG` for one-off events (rebuilds, bodies added / removed, fallbacks); `B3D_LOG_VERBOSE` / `B3D_LOG_IF` per step,
printed only with `BOX3DDOP_VERBOSE=1`. All compile to nothing without `BOX3DDOP_DEBUG`. Output: stderr, `[b3dop]`.

## Package

`install()` rules and CPack (zip) lay out a Houdini package: `Box3D.json` (from `package/Box3D.json.in`, enabled only
for the Houdini major.minor it was built with) and `Box3D/dso`, `Box3D/config/Icons`, `Box3D/otls`.
