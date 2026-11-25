# Source layout

Where each file lives and what it holds. Index and rules: [CLAUDE.md](../../CLAUDE.md).

```
CMakeLists.txt        Targets: b3dop_core, the core test exes, Box3DDOP (plugin)
build.sh / build.bat  Convenience wrappers (HFS discovery, submodule fetch, stale build/ reset)
test.bat              Runs the Houdini checks (tests/houdini/run_tests.py) with hython (Windows)
make_examples.bat     Builds the example scenes (examples/make_examples.py) with hython (Windows)
extern/box3d/         Box3D submodule, the author's fork (MysteryPancake/Box3D-Fork; static lib, EXCLUDE_FROM_ALL)
icons/box3d.svg       Node icon; the build copies it to config/Icons as DOP_<node name>.svg for every node
src/                  The include root: includes are written "core/Box3DWorld.h" (bare name within a folder)
  core/               Houdini-free; built as b3dop_core and covered by the tests
    Box3DMath.h         Header-only: unit constants, finite checks, small copies, 3-vector and quaternion
                        helpers, xformPoint
    Box3DShapes.h       Header-only: shape math, packed-prim matrices, hashing
    Box3DQuadrics.h     Header-only: surface points for Houdini's quadric primitives (Sphere, Tube, Circle,
                        Metaball)
    Box3DJoints.h       Header-only: joint types, JointSettings, the joint parameter table
    Box3DSideFX.h/.cpp  SideFX's Glue / Hard / Spring / Bullet Soft / Cone Twist data -> Box3D joints
                        (mapping, frame offsets)
    Box3DGlue.h         Header-only: SideFX Glue's impulse model (spread over bonds, half-life, break on
                        strength)
    Box3DCompound.h     Header-only: glue components (union-find), merging pieces into one body and splitting
                        its velocity
    Box3DCpu.h/.cpp     Physical performance core count (the solver's Threads default)
    Box3DRbdObject.h    Header-only: an RBD Object's own mass / friction / bounce / Bullet Data shape ->
                        whole-object settings
    Box3DWorld.h/.cpp   Pimpl wrapper over one b3World: the world's life (create, step), bodies and their
                        properties, mass, transforms, lazy poses; defines TaskRunner
    Box3DWorldShapes.cpp  Box3DWorld's shapes: box / sphere / capsule / cylinder / hull / mesh / height field,
                        copying and destroying them, outlines, filters, bounds
    Box3DWorldHulls.cpp   Hulls and fits: building a hull from points, the fit cache, pruning, statistics
    Box3DWorldContacts.cpp  Contact impulses and points, impulse / hit / move events, fast movers' approach
    Box3DWorldJoints.cpp  Box3DWorld's joint methods
    Box3DWorldInternal.h  The world's private state (Impl) and the conversions the Box3DWorld*.cpp files share
    Box3DWorldHulls.h     The process-wide hull and fit caches (HullLibrary, FitLibrary) and hull-building helpers.
                        With the Box3DWorld*.cpp files and Box3DSideFX.cpp (for <box3d/math_functions.h>) the only
                        files that include Box3D headers
    Box3DAffectorMath.h Header-only: affector (Static Object) transform math
    Box3DDebug.h        B3D_DEBUG_LOG / counters, compiled out unless BOX3DDOP_DEBUG (args still type-checked,
                        never evaluated)
  dop/                Houdini SIM_Data classes: the nodes and the world cache
    SIM_Box3DWorld.*    SIM_Data owning the shared world state (the rebuildable cache); the state itself is
                        sync/Box3DState.h
    SIM_Box3DData.*     Default-settings DOP node
    SIM_Box3DConRel.*   Box3D <Type> Constraint Relationship DOP nodes (one per joint type), parameters from
                        the table
    SIM_Box3DParms.h    Box3DParms: builds a node's parameter templates (tabs or simple folders, ranges, menus)
                        from a table
    SIM_SolverBox3D.*   The solver node: parameters, node messages and the solve loop (the parts are in sync/)
    SIM_Box3DRegister.cpp  initializeSIM(); includes UT_DSOVersion.h (exactly once, keep it here)
  sync/               Houdini data -> the Box3D world, used by the solver
    Box3DState.h           Box3DSharedState: the world, mirrors, scratch and flags; includes the three below
    Box3DBodyState.h       The body mirror and its map, per-object sync records, point slots and writes,
                           affector bodies
    Box3DJointState.h      Joint entries (and what few joints keep), Glue nodes, constraint kinds and networks
    Box3DCompoundState.h   Compound members, bridges and the compound (glued groups run as one body)
    Box3DAttribNames.h     The point attributes the solver reads, by name (PointAttrs and the prop markers come
                           from it)
    Box3DSolverInternal.h  What the solver's parts share (namespace box3dsolver); only the solver's files
                           include it
    Box3DSourceSync.cpp    Animated bodies following their source SOP (`followSourceSops`)
    Box3DShapeBuild.cpp    Packed primitives, collision source geometry (pieces, summary cache), buildShapes
    Box3DPointSync.cpp     One body per point: point state, fingerprint, the slot passes, syncObject
    Box3DPointShapes.cpp   A point's shape plan (`planShapes`): size keys, primitive overrides, shape signature
    Box3DPointBody.cpp     Body settings (dynamic props, mass, type), what a changed setting does, making a body
    Box3DWholeObject.cpp   One body per DOP object (RBD Object etc.): sync and Position write-back
    Box3DWriteBack.cpp     After the step: writeResultsBack (awake bodies and whole objects back to their points)
    Box3DImpacts.cpp       After the step: Impacts data (collectImpacts, recordImpacts)
    Box3DPostStep.cpp      After the step: queued point writes, merged move events, aging, dead points
    Box3DForces.cpp        Force DOPs and point force / torque
    Box3DGuide.cpp         The solver's guide geometry: every body's collision shapes
    Box3DJointSync.*       Constraint networks -> Box3D joints: the pass that makes, updates and removes joints,
                           `syncJoints`, the Glue lists
    Box3DJointRead.cpp     Reading a network: its geometry and fingerprint, settings and SideFX attributes, the
                           table of constraint kinds
    Box3DJointAnchors.cpp  Where a joint's anchors are (point / vertex / fragment anchors, rest frames)
    Box3DJointSyncInternal.h  What those three share (namespace `jointsync`); only they include it
    Box3DCompoundSync.*    Glued groups as one compound body (`gluecompound`): merge, dissolve, shrink,
                           `syncCompounds` (the public functions are declared in the .h)
    Box3DCompoundSplit.cpp Pieces that leave a compound when bonds break (`splitOffSmallPieces`)
    Box3DCompoundHold.cpp  Pieces freed for a fast body (`holdCompoundsForApproach`), bridges, hold expiry
    Box3DCompoundPieces.cpp  Per-step piece queries: moves, live poses, glue hits, state and force for Force DOPs
    Box3DCompoundInternal.h  What those four share (namespace `compoundsync`); only they include it
    Box3DJointBreak.cpp    After the step: Glue, breaking, feedback and live frames written onto the networks
    Box3DJointNetwork.h    What those two share: attribute names, network geometry and fingerprint helpers
    Box3DAffectors.*       Affector objects (Static Object DOP etc.) -> static colliders
    Box3DHoudiniUtil.h     Shared Houdini helpers: writable geometry, object by id, 4x4 to array, Euler to
                           quaternion, options of SIM data by name (box3dDataOption), the threaded loop
                           (box3dParallelFor)
    Box3DHoudiniTasks.h    TaskRunner on Houdini's TBB pool (UT_TaskGroup). Only SIM_Box3DWorld.cpp includes it
examples/             make_examples.py: builds demo scenes into examples/scenes (git-ignored)
package/              Box3D.json.in: the Houdini package file the build configures
tools/                make_assets.py (the four asset builders in one hython, as the build does),
                      make_packed_object.py, make_configure_sop.py, make_constraint_properties_sop.py and
                      make_solver_sop.py (the assets; its inner builders are b3d_solver_*.py, the SOPs' viewer
                      states b3d_*_state.py, read as text by the builders), b3d.py (finds Houdini; runs the
                      checks, examples, benchmarks or ctest on any OS), b3d_hda.py (helpers the builders share),
                      b3d_prepare.py (input packing), b3d_materials.py, b3d_settings.py, b3d_layouts.py (node
                      placement), find_houdini.bat, check_docs.py, houdini_network_dump.py (dumps a node's
                      networks to text)
docs/agents/          Technical notes by area; CLAUDE.md is the index ("Where to look")
tests/
  core_*.cpp          One exe per area: smoke, physics (with physics_shapes.cpp and physics_bodies.cpp), mesh,
                      heightfield, ground, quadrics, threads, taskrunner, worlds, joints, glue, sidefx,
                      compound, rbdobject
  TestUtil.h, WorldTestUtil.h, KeyedWorld.h
                      Shared helpers: CHECK macro, box / joint / step helpers, a world keyed by caller ids
  houdini/            Houdini checks: run_tests.py (runner), checks_*.py (tests by area, listed in the runner's
                      MODULES; each opens with a docstring naming its area, and checks_sop*, checks_output,
                      checks_packed and checks_geometry share helpers in *_common.py), b3dtest.py (helpers),
                      review.py (tidies saved scenes), baseline.json; out/ holds each test's scene (git-ignored).
                      bench.py + bench_scenes.py: timing runs of Box3D against Bullet, outside run_tests.py
                      (build.md)
  python/             test_harness.py: checks on the Houdini scripts that need no Houdini (stand-in `hou`)
```
