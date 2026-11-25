# Solver and world state

The shared world state, the step order, the sync pass, the skip fingerprint, write-back and threading. What the sync
does to a body is in [bodies.md](bodies.md); forces in [forces-affectors.md](forces-affectors.md). Index and rules:
[CLAUDE.md](../../CLAUDE.md).

## Solver step (`solveObjectsSubclass`)

**Two step counts.** *Substeps* (`substeps`) splits the DOP step into whole Box3D steps, each with its own `b3Collide`
(contact detection), so collisions are found `fps * Substeps` times a second. *Constraint Iterations* (`solverpasses`)
is Box3D's own `subStepCount`: each pass is a smaller time step for the contact / joint solve, not another iteration
at the same positions (Box3D runs one solve and one relax per pass). Not to be confused with `restitutionIterations`.
Kinematic targets (`setTargetTransformByHandle`) and the Glue impact decay use the whole frame's `stepDt`, so they
hold for any Substeps. Box3D clears its move, joint and hit events at the start of every step, so each is merged after
every Box3D step: moves by body, joints over their force / torque threshold into `brokenThisFrame` (broken once, in
Updating Constraint States), and hit pairs into `hitPairs` when Impact Min Speed is on (`hitsCollected`). Glue's hits
(`glueHitsFromEvents`) and Impacts read Box3D's contact impulse events after every Box3D step (`collectImpacts`,
`impactsCollected`, appended; the world makes them only while some object has Add Impact Data or there are glued
bodies), so a frame's impulses add up over its steps: a contact an earlier step already recorded (same bodies and
side, position to about 1 cm; looked up in `impactSlots`) takes the impulse instead of a new record, so Substeps do
not multiply the records (test `impacts_substeps`); with one step they are read after it. Point `force` / `torque` are
read and cleared by the first step and applied again from `PointForceCarry` in the others.

```
StepProgress (UTgetInterrupt)                  Escape checks; Houdini's own "Stepping simulation" bar stays
worldData = findOrCreateWorldData(objects)     first object carrying "Box3DWorld", else created on objects(0)
rebuild if invalid / thread count changed / stamp mismatch -> createFreshWorld
lock st->mutex
stepDt = dt * timescale; world.applyTuning(currentWorldSettings())   (stepDt stays the whole frame's time)
syncFromGeometry   per object syncObject(); syncAffectorBodies (Static Objects); remove gone bodies; syncJoints; syncCompounds;
                   pruneHulls; warnings
world.step         n = Substeps (`substeps`, at least 1; 1 when stepDt is 0). Each of the n steps: applyExternalForces (Box3D clears
                   forces every step), b3World_Step(stepDt / n, subSteps = Constraint Iterations), then its move events are merged
                   into st.moves by body (Box3D clears them every step; the last event wins, so fell-asleep survives).
                   applyExternalForces (Use Force DOPs): SIM_Force data per object -> getForce at each dynamic body's
                   centre of mass, plus point force / torque -> applyForceByHandle
expandCompoundMoves  Glue as One Body: a move event per piece of a moved compound (glue-compound.md)
writeResultsBack   awake dynamic bodies (move events): P / orient / v / w / packed transform / awake
jointsAfterStep    broken joints, feedback and live frames onto the networks (joint-networks.md)
recordImpacts      Add Impact Data -> SIM_Impacts (only on steps with an impact)
writePointAttrs    queued point writes: bullet_autofit_valid = 1, computed mass, rest attributes
ageObjects         Enable Aging only
deleteDeadPoints   `dead` points
stamp = now
```

**Performance Monitor.** Each stage is a `UT_PerfMonAutoSolveEvent(this, "<category>}/{<stage>")` (names written out
in `SIM_SolverBox3D.cpp`; Houdini wraps a name in braces and nests at a `/` outside them). The Stats view then shows
`{Category}` > `{Stage}`, as Bullet's Pre-Solve / Stepping Simulation / Post-Solve. A plain `A/B` or `A/{B}` gives
unbalanced braces. Pre-Solve: Rebuilding World, Updating Bodies, Updating Colliders, Removing Bodies, Updating
Constraints. Stepping Simulation: Applying Forces, Compound Motion, Glue Hits, Compound Bridges, Collecting Contacts
(the per-step contacts for Recording Impacts) and Box3D Step, the measured `world.step` (`stepWorld`), so Box3D's time
shows apart from the plugin's own stages. Post-Solve: Exporting Simulation Results, Updating Constraint States,
Recording Impacts, Updating Points, and Exporting Collision Shapes (Export Collision Shapes on). Sub-stages nest one
level deeper, written out the same way (opening an event inside another does not nest them): Updating Bodies has
Checking Changes, Reading Points, Planning Shapes, Checking Unchanged Points and Applying to Box3D (`syncObject`,
which gets the solver from `StepProgress::solver`); Exporting Simulation Results has Compound Pieces and Writing Back;
Updating Constraint States has Advancing Glue and Writing Joints (`jointsAfterStep`); Updating Points has Writing
Attributes, Aging and Deleting Dead Points. Opened only on the solve thread. Box3D's own per-part step timing
(`Box3DWorld::stepProfile`, `b3World_GetProfile`) is not reported to the monitor: an overridden event adds its time to
the solver node's, so the parts would count the step twice (a negative event cancelling them was rejected).

**Guide geometry** (`Box3DGuide.cpp`; Guide Options: Show Collision Shapes, Shape Color). A `SIM_GuidePerObject`
built by `buildCollisionGuide` from the live world (`bodyShapeOutlines`: hulls, spheres, capsules), each body placed at
the pose the displayed frame's geometry shows, so cached frames draw where the pieces were. Open polylines with `Cd`,
dimmed for sleeping and static / kinematic bodies. The mutex is only tried: a guide asked for mid-step is skipped.

**Export Collision Shapes** (`exportshapes`, Visualization tab; `exportCollisionShapes` in `Box3DGuide.cpp`). The
guide's lines (`drawObjectShapes`) as world-space open polylines in a `SIM_GeometryCopy` named `kShapesDataName`
(`Box3DShapes`) on every object with bodies, updated each step after the point writes (the solver holds the mutex;
poses are the mirror's, so no geometry is read; a whole object is read from its Position data). A hull's or mesh's
lines share its vertices as points. The outlines are kept in body space (`Box3DShapeExport`, `shapeExports`); a step
only moves the points of bodies whose pose changed and recolours those whose shade did (in parallel past
`kMinParallelMovers` bodies). A step with no move event, no scanned object and the same `bodyVersion` as the last
export (`shapesFresh`) writes nothing. The geometry stores the signature of what it was built for (`kShapesSigName`
detail attribute: handle, `shapeSig`, shape count, body type, scale of each); a different signature or a geometry from
another timeline builds it again. `Cd` is white times the dimming; the SOP tints it. The Box3D Solver SOP's
`GUIDE_GEO` imports it ([solver-sop.md](solver-sop.md)); turning the toggle off stops writing but leaves the last data
in the cache. A static concave piece's mesh is drawn as its boundary and creased edges (`meshOutline` in
`Box3DWorld.cpp`, thinned past `kMaxMeshOutlineEdges`; test `collision_shapes_mesh`). Not drawn: affector colliders,
and anything on an object's creation frame without Solve On Creation Frame (no body yet).

**New objects.** Objects in `newobjects` (created this timestep) keep their initial state unless their object node has
Solve On Creation Frame on (`SIM_Object::getSolveFirstFrame()`, as Bullet): `solveObjectsSubclass` drops the others
by object id before anything else, so for this step they get no body, no write-back, no aging and no joints, and the
rest of the step sees only the other objects. They are solved from the next timestep, as ordinary objects. A step whose
objects are all dropped returns at once without a world or a stamp, so the next step builds the world as for a first
step. New objects with the toggle on are solved in this step like the rest (added from `newobjects` if `allObjects`
lacks them, never twice). Both work whether the engine lists new objects in `newobjects` only or in both arrays.

**Escape.** `StepProgress::check()` calls `UTgetInterrupt()->opInterrupt()` with no percent and no `opStart` of our
own (SideFX's `SIM_VolumetricConnectedComponentBuilder` does the same), so Houdini's own "Stepping simulation" message
and bar are left alone. SIM_Engine discards and re-cooks an interrupted step itself (`setSimulationTime`,
`myLastCookInterrupted`); the stamp check makes that re-cook rebuild the world. `check()` is called at the
start of the solve, before each object, every `kProgressEvery` points in `syncObject` pass 2 (serial, between
points, so no half-built body), and once before the step. A cancel latches. Cancelled after the sync began:
`st->stamped = false` (the half-synced world can never be continued; the next solve that would continue it rebuilds),
a `UT_ERROR_MESSAGE` "step cancelled", and `SIM_SOLVER_FAIL`. `b3World_Step` itself is not interruptible, and
once it has run the step always finishes (write-back etc. are quick), so a cached step is always whole.
`StepProgress` is at global scope in `Box3DSolverInternal.h` (the solver header forward-declares it).

Data names: `"Box3DWorld"`, `kBox3DDataName` (`"Geometry/Box3DData"`, under the object's Geometry like Bullet Data;
SIM_Box3DData.h), `SIM_GEOMETRY_DATANAME`. Houdini warns on data it has no `SIM_DataHint` for under its parent
("Unexpected subdata found" on the Geometry, "attached to an unexpected parent data" on the data node):
`initializeSIM` adds one for Box3DData under `SIM_SopGeometry` (a Packed Object's Geometry) and `SIM_GeometryCopy`
(the Solver SOP's).
