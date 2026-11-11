# Box3D DOP skeleton

Blank-function skeleton for a Houdini DOP solver backed by Box3D.
No simulation code yet, just structure that should compile.

## Layout

```
CMakeLists.txt
extern/box3d/            <- put Box3D source here (or pass -DBOX3D_DIR=...)
src/
  Box3DWorld.h/.cpp      Houdini-free wrapper around one b3World (pimpl, no HDK, no Box3D headers in the .h)
  SIM_Box3DWorld         SIM_Data owning the world (a rebuildable cache)
  SIM_Box3DBody          per-object settings, also creates a "Box3D Body" DOP node
  SIM_SolverBox3D        SIM_Solver (multi-object), creates a "Box3D Solver" DOP node
  SIM_Box3DRegister.cpp  initializeSIM()
tests/core_smoke.cpp
```

## Build, step 1: Box3D only (no Houdini needed)

```
cmake -S . -B build_core -G "Visual Studio 17 2022" -A x64 -DBOX3DDOP_CORE_ONLY=ON
cmake --build build_core --config Release
build_core\Release\b3dop_core_smoke.exe
```
Expected: prints the Box3D version, `world valid: yes`, `OK`.

## Build, step 2: the Houdini plugin

Open "Houdini X.Y.Z > Command Line Tools" from the Start menu so %HFS% is set, then:

```
cmake -S . -B build -G "Visual Studio 17 2022" -A x64
cmake --build build --config Release
```
or open `build\Box3DDOP.sln` in Visual Studio. Use Release or RelWithDebInfo (not Debug).
The DLL is copied to your Documents\houdiniX.Y\dso. Close Houdini before rebuilding.

In Houdini: DOP network -> TAB -> "Box3D Solver" / "Box3D Body".

## What is and isn't verified

* Box3D side (CMake, static lib, C++17 consumer, create/destroy world): built and run on Linux/GCC.
* HDK side (`SIM_*`): written from memory of the HDK samples (SIM_SolverHair etc.),
  then cross-checked against the HDK `DOP_GroupAndApply` sample (which exposed the
  missing `UT_DSOVersion.h`, now included in `SIM_Box3DRegister.cpp`). NOT compiled. Expect to fix a few things on first build. Most likely spots:
  - the DECLARE_DATAFACTORY / IMPLEMENT_DATAFACTORY macros
  - `override` signatures (`solveObjectsSubclass`, `initializeSubclass`, `makeEqualSubclass`)
  - SIM_DopDescription constructor arguments
  Compare against `%HFS%\toolkit\samples\SIM\`.

## Debugging the three open questions

Debug logging is compiled in by default (`-DBOX3DDOP_DEBUG=ON`; set OFF later).
On Windows, GUI Houdini has no console, so log to a file. Set this BEFORE
starting Houdini:

```
set BOX3DDOP_LOG=C:\temp\box3ddop.log
set BOX3DDOP_PROBE_DEFAULTS=1          (question 2, one-shot)
set BOX3DDOP_REQUESTED_CACHE=0         (question 3, try 0 then 1)
houdini
```

Build a DOP net: two or more objects, a Box3D Body node on each, a Box3D Solver.

**1. Copy timing.** Look for `World[...] ctor / initialize / makeEqual / dtor`
lines and the `counters:` line printed every step. Then try:
* play frames 1..5: does `copied` go up by 1 per step? `rebuilds` should stay
  at 1 and `continues` should grow (that is the design working).
* scrub back to an earlier frame and step forward: expect one REBUILD with a
  `stamp mismatch` reason.
* change a solver parameter, or re-cook the same frame: see which path it takes.
* the first `Step` line shows whether engine time during a solve is the step
  start or end, and whether `getStartTime` is ever valid.

**2. Parameter defaults.** Every step prints the solver parms and each object's
`Box3DBody` values (data made by its DOP node). `PROBE:` lines show a
SIM_Box3DBody created from code, which is the case the question was about.
Expected defaults are printed on the line before it.

**3. getRequestedCache.** Printed in `cache:` each step, plus a line each time
the engine calls `getRequestedCacheSubclass()`. Run with 0 and 1 and compare
earliest/latest cache time and whether scrubbing triggers rebuilds.

## Current state

Implemented: shared world state + stamp, staleness detection, rebuild, object
add/remove sync, external-edit detection, write-back, and the Box3D core
(bodies, box/sphere shapes, step, transform/velocity get/set), checked against
the Box3D source in single and double precision (`b3dop_core_physics`).

Still TODO in the solver: shapes (`addObjectToWorld` adds bodies only, so
objects currently fall under gravity with no collision), kinematic targets,
gravity/sleep changes via `b3World_SetGravity` / `b3World_EnableSleeping`
instead of rebuilding, pivot / centre-of-mass handling, events.

Box3D build notes: point `BOX3D_DIR` at the folder containing Box3D's
CMakeLists.txt (the inner `box3d` folder if your zip is `box3d/box3d/...`).
Large-world mode is `-DBOX3D_DOUBLE_PRECISION=ON`; it is PUBLIC on the box3d
target and `Box3DWorld.cpp` is the only file that sees Box3D headers, so the
plugin cannot end up with a precision mismatch.
