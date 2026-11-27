"""Box3D Constraint Properties: the help card (F1 on the node), the asset's "Help" section in Houdini's wiki markup."""

HELP_CARD = '''\
= Box3D Constraint Properties =

#type: node
#context: sop
#internal: box3d_constraint_properties
#icon: DOP/box3d_data
#tags: dynamics, rbd, box3d, constraints

"""Sets Box3D joint settings on constraint lines as primitive attributes, like RBD Constraint Properties."""

Box3D Constraint Properties writes the settings of the Box3D joint types (Weld, Spherical, Revolute, Prismatic,
Distance, Motor, Parallel, Wheel and Motion Lock) onto the constraint lines in a group, ready for the Box3D Solver. A
float setting on a line multiplies the value of the constraint type on the solver (or replaces it, with that type's
Attributes Replace Values), so set the typical value on the solver and vary it here per constraint.

SideFX's own constraint types (Glue, Hard, Soft, Spring, Cone Twist) are set with RBD Constraint Properties as usual:
the Box3D Solver reads them.

@inputs

Geometry:
    #id: geometry
    The pieces, passed through. Scaling by an attribute can read the pieces' point attributes.

Constraint Geometry:
    #id: constraints
    The constraint lines to edit.

Proxy Geometry:
    #id: proxy
    Passed through, and read like Geometry for scaling.

@outputs

Geometry:
    The Geometry input, unchanged.

Constraint Geometry:
    The constraints with the settings written.

Proxy Geometry:
    The Proxy input, unchanged.

@parameters

Constraint Group:
    #id: constraintgroup
    The constraints to edit: a primitive group or a pattern. Empty edits every constraint.

Filter by Pieces:
    #id: groupfromgeo
    Edits the constraints between the pieces of Group 1 and those of Group 2 (or every piece) instead.

Enable Color:
    #id: enablecolor
    Colours the edited constraints with Color, written as `Cd`.

Guide Scale:
    #id: guidescale
    The size of the anchor frames drawn in the viewport while the node is selected. 0 hides them.

Action:
    #id: action
    Edit writes the ticked settings. Set writes the settings of the type in Constraint Type that differ from its
    defaults, and the Constraint Name.

== Viewport ==

In the viewport, pick constraint lines and turn the handle to rotate the frames of their two anchors, which set the
joint's axes. U resets, A aligns the frame's X axis with the line, M switches the motor on or off, and the mouse wheel
changes a spherical joint's motor speed. Clear Edits forgets the viewport edits, Import Edits copies the selected
lines' frames into them, and Edit Parms makes the handle edit Anchor Rotation for every line in the group instead.

== Constraint Properties ==

Constraint Name:
    #id: constraintname
    The Data Name of the constraint type the lines use, written as `constraint_name`.

Disable Collisions:
    #id: collisions
    Whether the two joined pieces collide with each other, written as `collide_connected`.

Constraint Type:
    #id: constrainttype
    Which joint type's folder shows. Its ticked settings are written.

== Joint and joint type folders ==

The Joint folder holds what every type has: Anchor Rotation (written as the anchors' `orient`), Collide Connected,
Break Force and Break Torque, and Constraint Stiffness and Damping Ratio. Each type's folder holds its own settings,
such as springs, limits and motors.

Scaling:
    Each number can be multiplied by the line's own attribute of the same name (Scale by Value), by a primitive
    attribute of the line or a point attribute of its pieces (Scale by Attribute), or both. The Weld's stiffnesses can
    also be randomized per constraint.

Use VEXpressions:
    Runs a VEX snippet over the constraints in the group after the settings are written, with the Constraint
    Geometry, Geometry and Proxy Geometry as its inputs.

== Next Constraint ==

Switch Constraint Type When Broken:
    #id: usenextconstraint
    A constraint that breaks becomes the Next Constraint Name instead, with the next settings below
    (`next_constraint_name` and `next_<setting>`). The Box3D Solver's Break Thresholds can switch them too.

== Geometry Attribute Transfer ==

Cluster Attribute Name and Transfer Mode:
    For constraints made between larger pieces than the ones simulated: which simulated pieces a constraint's piece
    stands for, and which of them gives the attribute Scale by Attribute reads (the closest, the smallest, the largest
    or the average).

@related

- [Box3D Solver|Node:sop/box3d_solver]
- [Box3D Configure|Node:sop/box3d_configure]
'''
