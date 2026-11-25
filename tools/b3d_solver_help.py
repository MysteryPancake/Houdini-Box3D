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
30 groups, but ignoring a single piece by name is not supported.

Unpacked geometry is packed by `name`, or by connected pieces when nothing is named.

@inputs

Geometry:
    #id: geometry
    The pieces to simulate. Ignored while a proxy is connected.

Constraint Geometry:
    #id: constraints
    Two-point lines naming the pieces they join. `constraint_name` picks the joint type: a Box3D joint or a SideFX
    Glue, Hard, Spring, Soft or Cone Twist constraint.

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

Time Scale, Substeps and Constraint Iterations:
    Trade speed for stability. Contacts have their own settings in the Contacts folder. Cache Simulation and Cache
    Memory (MB) are the simulation cache.

Allow Sleeping:
    The one switch for sleeping, for every piece and collider.

== Properties ==

The settings of the simulated pieces, in Shape, Body, Material, CCD & Contacts and Sleep folders. A point attribute of
the same name overrides each one.

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
    A ground plane or a height field for the pieces to land on, besides the Collision input.

== Constraints ==

Break Thresholds:
    #id: enable_constraintbreaks
    Breaks the constraints named in Constraint Names once their angle, distance, impact or torque passes a threshold,
    or at a chosen frame, as RBD Bullet Solver does. Only SideFX Glue has an impact. There is no Force Threshold.
    Scale by Attribute multiplies the threshold by a primitive attribute (`strength` makes it relative to each glue's
    own strength). Mode Switch to Next Constraint gives the constraint its `next_constraint_name` and any `next_<setting>` values it has. Off costs nothing.

Export Broken Constraints:
    #id: constraint_keepbroken
    Keeps broken constraints in the Constraint Geometry output.

== Forces ==

The Gravity folder (its checkbox off multiplies Gravity by 0) and Use Force DOPs are here, with the Bounce, Friction and Glue folders.

Wire POP forces into PRESOLVE (dive into the node, then FORCES). With World Scale, force and torque are scaled for you.

== Visualization ==

Show Constraints draws the constraint lines, and False Color Mode colours them blue to red by angle, distance, force,
impact, torque or an attribute you name. Show Guides draws them as thick wires, which is slow with many constraints.
Guide Radius sets how thick the wires are, and each constraint type's Guide Scale (Advanced, Constraints) multiplies it. Under Impact Data, Show Impacts draws the impact points (needs Add Impact Data).
Show Collision Shape draws the pieces' collision shapes, which costs time every step. Under Collision Geometry,
Show Geometry draws the Collision input, and Show Collision Shape and Color draw its collision shapes. Show Ground draws
the Ground Collision.

== Advanced ==

Stability and Performance settings: warm starting, continuous collision, speed limits and threads. Collision has Solve on Creation Frame for the colliders, which follows the Solver tab's until changed.
Constraints:
    The settings of each constraint type: a closed folder per joint. A float primitive attribute on the constraint
    lines multiplies the value here, so set the typical value here and vary it per constraint with an attribute. Each
    folder also has Enable (off leaves the type out), Data Name (what `constraint_name` picks it by), Default
    Operation (Set Always applies the values every step, so animated ones act) and Guide Scale (the thickness of its
    lines in the viewport, times Guide Radius).

== Output ==

Impact Data records collisions on the pieces. Fetch Impacts adds an Object Merge beside the node with them as points.

Attributes has Enable Aging, which adds the time step to each point's `age` and marks points dead once it passes `life`.

Attribute Transfer chooses which point attributes of the simulation are copied onto the Simulation Points, Geometry and
Proxy Geometry. Off for an output: only `w` is copied.

@related

- [Box3D Packed Object|Node:dop/box3d_packedobject]
- [Box3D Configure|Node:sop/box3d_configure]
- [Box3D Constraint Properties|Node:sop/box3d_constraint_properties]
'''
