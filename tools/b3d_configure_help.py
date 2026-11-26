"""Box3D Configure: the help card (F1 on the node), the asset's "Help" section in Houdini's wiki markup."""

HELP_CARD = '''\
= Box3D Configure =

#type: node
#context: sop
#internal: box3d_configure
#icon: DOP/box3d_data
#tags: dynamics, rbd, box3d

"""Sets Box3D body settings on pieces as point attributes, like RBD Configure."""

Box3D Configure writes the settings of Box3D Data (shape, mass, material, collision, sleep and more) onto the pieces
as point attributes, group by group, ready for the Box3D Solver. Point attributes win over the solver's and Box3D
Data's own values, so each group of pieces can have its own.

Unpacked geometry is packed first, as RBD Configure does: by `name`, or by connected pieces when nothing is named.

@inputs

Geometry:
    #id: geometry
    The pieces to configure. They pass through unchanged when a proxy is connected.

Constraint Geometry:
    #id: constraints
    Passed through. Pins add their constraint lines here.

Proxy Geometry:
    #id: proxy
    Simulation pieces for the Geometry: when connected, the settings go onto these instead.

Static Bounds Geometry:
    #id: bounds
    Geometry whose bounding box or volume Use Bounds tests, with Bounding Type set to use it.

@outputs

Geometry:
    The pieces, packed, with the settings when no proxy is connected.

Constraint Geometry:
    The constraints, with a line from the world to each pinned piece.

Proxy Geometry:
    The proxy with the settings.

@parameters

== Visualization ==

Visualize:
    #id: visualize
    Colours the pieces in the viewport by Bounce, Density, Friction or Rotational Stiffness, blue for the lowest value
    and red for the highest, with a histogram of the values. Nothing downstream sees the colours.

Exploded Scale:
    #id: scale
    Moves the pieces apart in the viewport to see inside. 0 is off.

== Bounds ==

Bounding Type:
    #id: boundingtype
    What Use Bounds tests: a box set by Size, Center and Rotation (with a handle in the viewport), or the Static Bounds
    Geometry input's bounding box or volume.

== Attributes ==

Name Prefix, Transfer Simulation Attributes, Transfer Render Geo Attributes, Create Path Attribute and Create Packed
Fragments control how unpacked geometry is packed, as RBD Configure's.

Number of Attributes:
    Entries of settings, applied in order, so a later entry wins where two set the same thing. Each has:

    Enable:
        Off skips the entry.

    Group:
        The pieces to set: a point or primitive group, or a pattern such as `piece*` or `@name=piece0`. Empty sets
        every piece.

    Settings:
        The folders of Box3D Data's settings (Shape, Body, Material, CCD & Contacts, Sleep), plus Initial State
        (velocity, spin, Enabled, Deforming), Motion and Collision Filtering. Only ticked settings are written. Var
        scatters Density, Bounce, Friction or Rotational Stiffness per piece. Use Bounds gives a setting's value
        inside the bounds and the opposite outside. A Material preset fills Density, Bounce and Friction from a list
        of real materials, as RBD Configure's.

    Constraints:
        Add Pin adds a constraint from the world to each piece in the group, a Weld that holds it rigidly or a Soft one
        that holds it with a spring. Use Bounds pins only the pieces inside the bounds.

@related

- [Box3D Solver|Node:sop/box3d_solver]
- [Box3D Constraint Properties|Node:sop/box3d_constraint_properties]
- [Box3D Packed Object|Node:dop/box3d_packedobject]
'''
