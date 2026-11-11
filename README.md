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

## Verified in Houdini 22.0

The plugin builds and loads in Houdini 22.0.479 and has been run in a DOP network
(Object -> Box3D Body -> Box3D Solver):

* Objects reach the solver as `newobjects` on the start frame and as `objects` from the next frame.
* The world is created once, then carried between timesteps (shared state, one
  `SIM_Box3DWorld` copy per cached timestep). Playing forward does not rebuild.
* Reset simulation rebuilds the world from the object's Position / Motion data.
* Scrubbing over cached frames does not call the solver at all.
* `getRequestedCacheSubclass()` returns 1.
* Gravity, velocity and write-back to the object give the expected results.

The object needs Position data (an Empty Object has none; use an RBD Object or add a Position).

## Debug logging

Compiled in by default (`-DBOX3DDOP_DEBUG=ON`; pass OFF for a quiet build). Output goes to
stderr, which shows up in Houdini's console window. It only prints when something happens:
`REBUILD #n: <reason>`, bodies added/removed, external edits, and warnings/failures.

Optional, set BEFORE starting Houdini (debug builds only):

```
set BOX3DDOP_REQUESTED_CACHE=0         (override the default of 1)
```

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
