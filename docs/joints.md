# Joints

[Back to the README](../README.md)

Joints use Houdini's **Constraint Network** DOP: wire Box3D Constraint Relationship nodes into its second input. Each two-point line is one joint:

- `constraint_name` names a relationship's Data Name (by default its type: `Weld`, `Revolute`, ...).
- Each end's `name` picks a piece (empty = the world). Its `P` and `orient` set where the joint is and which way its axes point, on the pieces as they were at the start (like Bullet).
- Weld's Hold Separation keeps the pieces as far apart as they start, for anchors at the pieces' centres (Connect Adjacent Pieces). Off, the weld pulls the anchors together.
- Primitive attributes named like the node's parameters modify them: floats multiply, others replace. With Attributes Replace Values on, floats set the value too (in degrees for angles).
- A float of -3.4e38 or a toggle of -1 means not set: the node's value is kept. **Box3D Constraint Properties** writes these for the lines it does not edit. It can also scale a number by an attribute of the constraint or of the pieces at its two ends (a setting's Scaling and Attribute, Geometry Attribute Transfer).
- **Bond** is a rigid weld that breaks by stress: its strengths are in Pa over the contact area, taken from where the two pieces touch, so weight and impacts both count. Fracture Energy makes it hold on briefly past its strength (0: it goes at its strength) With Write Bond Attributes on (Advanced tab), its line gets `bond_ratio` (how close its load is to its strength, 1 breaks it), `bond_damage` (how much of the tear it has taken, 1 tears it), the load as `bond_tension`, `bond_shear` (N), `bond_bending`, `bond_twist` (N*m) and `bond_area`. The same switch writes `compound_id` on the points of glued pieces that move as one body, the lowest point number in the group (-1: not in one). The Material menu fills in typical values.
- With **Breakable** on, a joint past its Break Force or Torque goes into the `broken` group, or switches to its `next_constraint_name`.
- Each step the solver writes `force`, `distance`, `torque` and `angle` (radians, the pieces' rotation relative to each other since the start) onto each line.
