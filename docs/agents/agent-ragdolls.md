# Agent ragdolls

Crowd agents (`GU_Agent` primitives with a `active_agent` array) as one body per rig transform, as the RBD Bullet Solver
does for SideFX's crowd ragdolls. Bodies, their joints, the per-transform arrays and the agent's pose are built; the motors and
the rest are not ([todo.md](todo.md), "Agent ragdolls"). Index and rules: [CLAUDE.md](../../CLAUDE.md).

## What the solver does (`Box3DAgent.cpp`)

* A point whose primitive is a `GU_Agent` (`isAgentPrim`, checked only when the geometry has agents:
  `hasAgentPrims`; `Box3DPointSlot::agent`, set in `readSlot`) gets no body of its own, like a point of an unpacked
  primitive, as Bullet makes none, and an object with one never skips its scan (`Box3DObjectSync::anyAgents`). It is a
  ragdoll agent when its `active_agent` int array has elements (`isRagdollAgent`, `Box3DPointSlot::agentRagdoll`);
  one that is not gets no bodies at all, and its name goes in `Box3DObjectSync::agentsNotRagdoll`.
* `syncAgentBodies` runs after the apply pass of `syncObject`. For each ragdoll agent it takes the first collision layer's
  shape bindings (`GU_AgentLayer::ShapeBinding`, one body per transform that has one) and makes the body once: a hull of
  the shape's geometry (in the transform's own space) through the transform's world matrix M times the transpose of the
  body's rotation R (`splitPackedMatrix`'s), so the hull keeps the matrix's scale (the agent's units), shear and
  mirroring (a Sphere primitive stays round, sized by the matrix's first row), posed at R and the agent primitive's
  own transform, origin at the shape's centre as Bullet puts `pivot_agent` (`shapeCentre`): the point of a lone Tube
  or Sphere primitive (a capsule, a sphere), else the centroid of the convex hull of its points, packed geometry
  unpacked (`geometryCentreOfMass`: a box's centre; a Custom shape's), else, flat, its bounding box centre. The
  centres are found once per agent, with its rest. Its rest (`pivot`, `restRot`, what the arrays and the pose measure
  from) is the rig's at rest: `Box3DAgentRest`, made once per agent by `restOfAgent` from the pose when its
  first bodies are made, or, when the point's arrays describe the pose it has now (a reloaded or rebuilt simulation),
  by moving each transform back by its body's motion since rest (the inverse of `writeAgentPose`); the bodies then also
  start at `v_agent` / `w_agent`. An agent that stops being a ragdoll loses its rest. Its `BodyProps` are the agent
  point's (`readDynamicProps`),
  with the mass the density times the shape's own volume (the primitives' `calcVolume` summed, so a closed polygon
  mesh's faces add up to its volume, times M's determinant; the hull's when that is not within `kMinVolumeOfHull` to
  `kMaxVolumeOfHull` of it, as for faces that close no volume or wind both ways; for the layer's capsule, a Tube with a
  `tubeCapt` attribute, times `kAgentMassScale`, which is what Bullet's `mass_agent` comes to) spread over the hull.
* Bodies are mirror entries with `agentBody` set, no point (`pointOff` -1) and the id `kAgentIdBase + point id *
  kAgentTransforms + transform` in the object's key space. Each scan stamps them, so they go when their agent does, and
  retypes a body when its `active_agent` element flips. A 0 is a transform that follows the animation: a kinematic
  body (`followAnimation`) driven each scan to where the agent's pose has its shape now
  (`setTargetTransformByHandle` over the step), so it moves with the animation's velocity as Bullet's does, the
  ragdoll's joints hang from it, and a body turned dynamic carries on from the animated pose with that velocity; a
  body just turned kinematic drops the velocity it had as a ragdoll (else it would coast on it for a step). Else the
  node's type.
* `writeAgentArrays` (called from `writeResultsBack` on every way out, so also on a frame where nothing moved) writes
  per agent point the arrays Bullet writes, one element per rig transform and zeros where there is no body:
  `pivot_agent` (the body's centre at rest), `trans_agent` (centre now minus that), `orient_agent` (rotation since
  rest: `box3dAbsoluteToRest`), `v_agent`, `w_agent` and `mass_agent`. Move events of agent bodies
  (animated, kinematic ones too) update the mirror's pose there (`collectMovedBodies`, `kAgent`).
* Joints: a constraint network names its anchors `<agent name>/<rig transform>` (the agent primitive's `name`, else
  its point's; `anchor_type` agent) and the solver finds them in `Box3DObjectSync::agentByName`, which
  `syncAgentBodies` rebuilds every scan: a transform with a body maps to it, one without to the body of its closest
  ancestor transform that has one (the Agent Constraint Network's Cone Twist lines join a body to its parent
  transform, which may have none). `Anchors::resolve` looks there first for a name with a slash. A transform with no
  body on or above it (the rig's root and what hangs from it beside the first body) is in
  `Box3DObjectSync::agentWithoutBody`: a constraint naming it, or any transform of an agent that is not a ragdoll,
  makes no joint (`agentAnchors`, warned once), as Bullet makes none. An agent that left the ragdoll state (its
  arrays still list bodies: `Box3DObjectSync::agentsLeftRagdoll`) drops its constraints without the warning, and so
  does one that left the crowd (its point went: `Box3DObjectSync::agentsGone` on that scan, which each network that
  names it remembers in `Box3DJointNetwork::agentsGone`; an agent of that name coming back resolves again). An agent
  anchor is where
  Bullet has it (`resolveEnds`): its `local_P` through its transform's world matrix at rest (`Box3DAgentRest::world`,
  by name in `Box3DObjectSync::agentRest`), its frame the transform's rotation (`Box3DAgentRest::frames`, the matrix's
  rotation as `splitPackedMatrix` splits it) turned by its `local_orient`; its `P` is not read, and each is read on its own:
  without `local_P` the anchor is at `P`, without `local_orient` its frame is the transform's rotation. Those two are the anchor's place in the
  transform, so an agent end is never given saved local frames (`agentEndA` / `agentEndB` on the entry: not written
  after the step, not used to remake the joint). A mirrored rig transform (the mocap biped's left arm: a negative
  determinant) is that rotation with its z row negated (`Box3DAgentFrame::mirrored`, `AnchorBody::mirrored`), so the
  `local_orient` turn is mirrored to (-x, -y, z, w) and `specFor` negates the z of that end's Cone Twist axes, which
  are in the transform's own space. A ragdoll's first step solves none of its joints, as Bullet's: a joint with
  an end on a body made this step (`Box3DBodyMirror::agentFreshStamp`, not one rebuilt from its arrays) is a Filter
  joint for that step, so the two bodies, which may overlap, do not push apart, and the network is read again on the
  next (`Box3DJointNetwork::retryNextStep`), which makes the real joint. The Cone Twist data
  then maps as for any anchors ([joints-sidefx.md](joints-sidefx.md)), and a Pin
  from a transform to a Static Object is an ordinary Hard line. The two ends of a joint are bodies of one ragdoll, so
  their collisions follow the data's Disable Collisions.
* The pose (`writeAgentPose`, after the arrays, in the same write lock): a body's transform moves rigidly with it, so a
  simulated body's transform is its rest world matrix (`Box3DAgentRest::world`, in
  `Box3DObjectSync::agentRestWorld`) with its origin put where the body's centre plus the rest offset turned by `orient_agent` lands and its axes
  turned by it. The root body (no simulated ancestor) gives the agent point's `P` / `orient` when nothing above it has
  a body (the Hips) and the point's `bullet_updateagentxform` is not 0 (the Crowd Solver writes 1 for a whole ragdoll,
  0 for a partial one, and Bullet leaves the agent's transform alone at 0): the agent's transform is the one that
  keeps the root's transform where the agent has it, and the point's `orient` is its rotation. Every other simulated body's transform is placed in that space, a transform with
  no simulated body keeps its local transform under its parent (`GU_Agent::setWorldTransforms`), and with no
  simulated body at all nothing is written, so an animated agent keeps its animation. This reproduces Bullet's pose
  exactly from the same arrays (its joints' origins equal the arrays' prediction to 1e-4).
* Counts: `SyncCounts::agentBodies`, `agentFailed` (a transform whose shape or pose failed) and `agentNoShapes` (an
  agent without a collision layer).

## What Bullet does (measured, Houdini 22.0, the PinnedRagdolls and PartialRagdolls examples)

* The DOP object's geometry holds the agent primitives with their full rig; the Crowd Solver runs before the rigid body
  solver in the Multiple Solver.
* Per agent point, one array element per rig transform (index = the rig's transform index; zero where there is no body):
  `pivot_agent` (vec3, the body's centre of mass when made, constant), `trans_agent` (vec3, centre now minus
  `pivot_agent`), `orient_agent` (quaternion, rotation since then), `v_agent` / `w_agent` (vec3 world velocities),
  `mass_agent`, `active_agent` (int, 1 simulated, 0 follows the animation), in the simulation's space: a packed
  piece's `pivot` / `P` / `orient` convention. Also `bullet_updateagentxform` (1), `ragdoll_stiffness`,
  `ragdoll_motorstrength`, `state`, per-shape overrides `bullet_primT_agent` / `S` / `R`, `bullet_radius_agent`,
  `bullet_length_agent` (empty by default), and the sticky collision and sleeping arrays.
* Bullet writes the posed agent itself: the agent point's `P` / `orient` follow the Hips body and every other
  transform's local transform changes so that each joint stays rigidly attached to its body (the offset from a body's
  centre to its joint's origin, in the joint's axes, stays constant).
* PartialRagdolls changes `active_agent` mid-simulation (a crowd trigger and state transition).

## Checking it

`tests/houdini/agent_oracle.py [bullet|box3d] [PinnedRagdolls|PartialRagdolls] frames...` dumps the arrays of the first
agent for either solver (the Box3D Solver takes the place of the example's Bullet solver in its Multiple Solver,
with its own Gravity at 0: the example's Gravity DOP is the gravity). Tests (their limits are in `checks_agent.py`):
`agent_bodies_bullet` (frame 1: the same rig transforms have bodies, with Bullet's masses and centres of mass),
`agent_shapes_bullet` (the same with every shape a box, a sphere, then a Custom tetrahedron and cone: Bullet samples a
custom shape's volume, so its masses and the cone's centre are only near ours),
`agent_bodies_partial` (40 frames: 15 bodies an agent throughout, the flips of `active_agent` do not break it),
`agent_joints_bullet` (PinnedRagdolls: the bodies near Bullet's on frame 6; on frames 24 and 48 the pinned hand, the
ragdoll's centre and its spread against Bullet's), `agent_pose_box3d` (the posed joints are where the arrays say, the
agent's point has moved), `agent_rebuild` (the world rebuilt from a checkpoint at frame 15: `pivot_agent` kept, the
bodies on frame 30 near a straight run's), `agent_anchor_local` (moving the anchors' `P` changes nothing, moving their
`local_P` moves the bodies as in Bullet), `agent_anchor_no_body` (a Pin to the rig's root makes no joint, with the
warning) and `agent_not_ragdoll` (an agent out of the ragdoll state gets no body, and a Pin to it makes no joint) and
`agent_partial_bullet` (PartialRagdolls: the animated bodies on Bullet's, and the parts turned ragdoll near Bullet's),
`agent_update_xform` (`bullet_updateagentxform` 0 keeps the agents' points) and `agent_leave_ragdoll` (agents leaving
the ragdoll state: all animated, their bodies follow the animation; no longer ragdolls, their bodies go and their
arrays stay as last written, as Bullet's) and `agent_crowd_changes` (an agent leaving the crowd leaves the others as
they were; agents joining get their bodies on that frame and move as Bullet's).
