"""Box3D Solver SOP: the help card (F1 on the node), the asset's "Help" section in Houdini's wiki markup."""

HELP_CARD = '''\
= Box3D Solver =

#type: node
#context: sop
#internal: box3d_solver
#icon: DOP/box3d_solver
#tags: dynamics, rbd, box3d

"""Simulates packed pieces with Box3D, like the RBD Bullet Solver SOP."""

The Box3D Solver takes pieces, constraints, a proxy and colliders, simulates them with the Box3D physics engine and
outputs the pieces, constraints and proxy moved by the simulation, plus one simulation point per piece. It uses the
same attributes and rules as RBD Bullet Solver, so most Bullet setups work with only the node swapped.

Some Bullet settings differ. `bullet_want_deactivate`, `bullet_sleeping`, `bullet_ignore` and `min_activation_impulse`
are not read: use `enablesleep`, `awake` and `enabled`. Collision padding and Bullet's solver settings (ERP, CFM,
iterations) are ignored: use Substeps and the contact settings. `collisiongroup` and `collisionignore` work for up to
30 groups, and two pieces collide unless each ignores the other's group, as in Bullet. Ignoring a single piece by
name is not supported.

A few things behave differently from Bullet. A Force DOP's Sample Mode is used: Circle and Sphere act over the piece's
size, where Bullet samples every force at a point. A constraint anchored to the world can be animated, and carries its
piece along at the anchor's speed, where Bullet drags it behind.

Unpacked geometry is packed by `name`, or by connected pieces when nothing is named.

@inputs

Geometry:
    #id: geometry
    The pieces to simulate. Ignored while a proxy is connected.

Constraint Geometry:
    #id: constraints
    Two-point lines naming the pieces they join. `constraint_name` picks the joint type: a Box3D joint or a SideFX
    Glue, Hard, Spring, Soft or Cone Twist constraint. An empty name is the world.

Proxy Geometry:
    #id: proxy
    Pieces simulated in place of Geometry. Geometry follows them.

Collision Geometry:
    #id: collision
    Colliders. Packed pieces, or a mesh or height field with Collide As Static Object.

@outputs

Geometry:
    The Geometry input moved by the simulation.

Constraint Geometry:
    The constraints, moved with their pieces. Broken ones are removed unless Export Broken Constraints is on.

Proxy Geometry:
    The Proxy input moved by the simulation.

Simulation Points:
    One point per piece with its `name`, position, orientation, pivot, rest transform, velocity (`v`, `w`) and the
    Transfer Attributes.

@parameters

Reset Simulation and Start Frame sit above the tabs. Before the Start Frame the Geometry, Constraint Geometry and Proxy
Geometry outputs pass the inputs through.

== Setup ==

World Scale:
    #id: scale
    Simulates the scene scaled by this and scales the outputs back. Use it when pieces are much smaller or larger
    than about 1 unit. Gravity, joint settings and forces follow it, so enter real values.

Time Scale, Global Substeps, Substeps and Constraint Iterations:
    Trade speed for stability. Global Substeps splits the whole network's time step, Substeps only Box3D's. Contacts
    have their own settings in the Contacts folder. Cache Simulation and Cache Memory (MB) are the simulation cache.

Solve on Creation Frame:
    #id: solvefirstframe
    Simulates the pieces on the frame they appear. Off: they start moving one frame later, as with Bullet.

Emit RBDs:
    #id: useemission
    Adds the input pieces again every frame as new pieces, as RBD Bullet Solver's: each copy's name gets
    `_` and the frame times 100 (`piece0` emitted on frame 2 is `piece0_200`), and the constraints between two emitted
    pieces come with them (a later copy has none to the world or to a piece that is not emitted). A piece whose
    `rbdbullet_emit` point attribute is 0 is simulated once. Animate the input or `rbdbullet_emit` to choose what is
    emitted when. A new piece starts with its own `v` and `w`. Override Attributes from SOP, Enable Attribute Update
    from SOPs, Overwrite with SOP and the breaks' Update from Input are off while emitting.

Replicate RBDs:
    #id: replicate
    For pieces copied during the simulation, for example by a SOP Solver in the FORCES subnet: each copy gets its own
    copy of its source's geometry in the Geometry and Proxy outputs. Copies that share a name get `_r` and a number
    (`piece0`, `piece0_r1`, `piece0_r2`), in the Simulation Points output too. A copy's `__restxform` point attribute,
    if set, is its starting transform.

Allow Sleeping:
    The one switch for sleeping, for every piece and collider.

== Properties ==

The settings of the simulated pieces, in Shape, Body, Material, CCD & Contacts and Sleep folders. A point attribute of
the same name overrides each one.

Override Attributes from SOP:
    #id: overwritefromsop
    Copies the listed point attributes from the pieces every step, matched by name, so ones that change over time
    (such as `active`) reach the simulation. Off: each piece's attributes are read once, when it appears. It costs time
    every step, and is off while Emit RBDs is on.

== Collision ==

Enable Collision Geometry:
    The checkbox of the Collision Geometry folder. Off: the Collision input is ignored.

Collide As:
    #id: collisionsource
    Packed Pieces makes every piece of the Collision input a body. Static Object makes the whole input one static
    mesh, or a height field when it has one.

Collision Type:
    #id: collisiontype
    Static keeps the collider where it starts. Animated follows its animation and pushes what it hits. Deforming also
    rebuilds its shape every step. A Static Object collider only follows Deforming.

Ground Collision:
    #id: useground
    A ground for the pieces to land on, besides the Collision input. Ground Box is a very wide, deep box (Position,
    Rotation, Size). Height Field uses a height field node, rebuilt every step with Use Deforming Geometry. Its
    Friction, Bounce, Rolling Resistance and Conveyor Velocity are under Physical.

== Forces ==

The Gravity folder (its checkbox off turns Gravity off) and Use Force DOPs are here, with the Bounce / Friction and
Glue folders.

Glue:
    Glue as One Body runs each group of glued pieces as one rigid body, which is stiffer and faster for large chunks.
    A glued chunk bounces and slides with its pieces' friction and bounce averaged by mass, as in Bullet. Glue Inertial
    Load also breaks bonds away from a hit. Write Compound IDs writes each glued piece's `compound_id`.

Wire POP forces into PRESOLVE (dive into the node, then FORCES). With World Scale, force and torque are scaled for you.

== Constraints ==

Break Thresholds:
    #id: enable_constraintbreaks
    Breaks the constraints named in Constraint Names once their angle, distance, impact or torque passes a threshold,
    or at a chosen frame, as RBD Bullet Solver does. Only SideFX Glue has an impact. There is no Force Threshold.
    Scale by Attribute multiplies the threshold by a primitive attribute (`strength` makes it relative to each glue's
    own strength). Mode Switch to Next Constraint gives the constraint its `next_constraint_name` and any
    `next_<setting>` values it has. Off costs nothing.

Enable Attribute Update from SOPs:
    #id: enable_constraintupdates
    Copies the listed primitive attributes from the live Constraints input onto the simulation's constraints every
    step, so ones that change over time reach it. Group limits it to some constraints.

Export Broken Constraints:
    #id: constraint_keepbroken
    Keeps broken constraints in the Constraint Geometry output.

== Visualization ==

Show Constraints draws the constraint lines, and False Color Mode colours them blue to red by angle, distance, force,
impact, torque or an attribute you name. Show Guides draws them as thick wires, which is slow with many constraints.
Guide Radius sets how thick the wires are, and each constraint type's Guide Scale (Advanced, Constraints) multiplies
it. Under Impact Data, Show Impacts draws the impact points (needs Add Impact Data).

Show Collision Shape draws the pieces' collision shapes, which costs time every step. Under Collision Geometry,
Show Geometry draws the Collision input, and Show Collision Shape and Color draw its collision shapes. Show Ground draws
the Ground Collision. The Inspector shows the chosen attributes of the piece under the cursor.

== Advanced ==

Stability:
    Warm Starting, Continuous Collision and Max Linear Speed, the speed limit for every piece.

Mass & Inertia:
    Bounding Box Inertia gives a convex hull, or a piece of several shapes, the inertia of the box around it, as
    Bullet does, so pieces spin as they do there. Near-Sphere Tolerance treats a piece whose inertia is nearly a
    sphere's as a sphere.

Performance:
    Use All Threads and Threads.

Collision:
    Solve on Creation Frame for the colliders, which follows the Setup tab's until changed.

Constraints:
    Overwrite with SOP reads the whole Constraints input again every step, so constraints that broke come back: use it
    only when the input changes over time. Below it, the settings of each constraint type, a closed folder per joint.
    A float primitive attribute on the constraint lines multiplies the value here, so set the typical value here and
    vary it per constraint with an attribute. Each folder also has Enable (off leaves the type out), Data Name (what
    `constraint_name` picks it by), Default Operation (Set Always applies the values every step, so animated ones act)
    and Guide Scale (the thickness of its lines in the viewport, times Guide Radius).

== Output ==

Impact Data records collisions on the pieces. Fetch Impacts adds an Object Merge beside the node with them as points.

Attributes has Enable Aging, which adds the time step to each point's `age` and marks points dead once it passes `life`,
and Add Active Age Attribute and Just Activated, for the time each piece has been active and the pieces that just
became active.

Attribute Transfer chooses which point attributes of the simulation are copied onto the Simulation Points, Geometry and
Proxy Geometry. Off for an output: only `w` is copied.

@related

- [Box3D Packed Object|Node:dop/box3d_packedobject]
- [Box3D Configure|Node:sop/box3d_configure]
- [Box3D Constraint Properties|Node:sop/box3d_constraint_properties]
'''
