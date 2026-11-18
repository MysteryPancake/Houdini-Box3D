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
    One point per piece with its position, orientation, velocity and the Transfer Attributes.

@parameters

== Solver ==

World Scale:
    #id: scale
    Simulates the scene scaled by this and scales the outputs back. Use it when pieces are much smaller or larger
    than about 1 unit. Gravity, joint settings and forces follow it, so enter real values.

== Pieces ==

The settings of the simulated pieces. A point attribute of the same name overrides each one.

Initial State:
    #id: vel
    Velocity and Angular Velocity replace the points' own `v` and `w` while Inherit Velocity from Point Velocity is
    off.

== Collision ==

Collide As:
    #id: collisionsource
    Packed Pieces makes every piece of the Collision input a body. Static Object makes the whole input one static
    mesh, or a height field when it has one. Use Flip Mesh Winding if pieces fall through a mesh.

Collision Type:
    #id: collisiontype
    Static keeps the collider where it starts. Animated follows its animation and pushes what it hits. Deforming also
    rebuilds its shape every step.

Ground Collision:
    A ground plane or a height field for the pieces to land on, besides the Collision input.

== Constraints ==

The settings of each constraint type: a closed folder per joint. A float primitive attribute on the constraint lines
multiplies the value here, so set the typical value here and vary it per constraint with an attribute.

Break Thresholds:
    #id: enable_constraintbreaks
    Breaks the constraints named in Constraint Names once their angle, distance, impact or torque passes a threshold,
    or at a chosen frame, as RBD Bullet Solver does. Only SideFX Glue has an impact. There is no Force Threshold. Scale by Attribute multiplies the threshold by a primitive attribute
    (`strength` makes it relative to each glue's own strength). Mode Switch to Next Constraint gives the constraint
    its `next_constraint_name`. Off costs nothing.

== Forces ==

Wire POP forces into PRESOLVE (dive into the node, then FORCES). With World Scale, force and torque are scaled for you.

== Output ==

Impact Data records collisions on the pieces. Fetch Impacts adds an Object Merge beside the node with them as points.

@related

- [Box3D Packed Object|Node:dop/box3d_packedobject]
- [Box3D Configure|Node:sop/box3d_configure]
'''
