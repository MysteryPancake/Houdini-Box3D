# Joints

[Back to the README](../README.md)

Joints use Houdini's **Constraint Network** DOP: wire Box3D Constraint Relationship nodes into its second input. Each two-point line is one joint:

- `constraint_name` names a relationship's Data Name (by default its type: `Weld`, `Revolute`, ...).
- Each end's `name` picks a piece (empty = the world). Its `P` and `orient` set where the joint is and which way its axes point, on the pieces as they were at the start (like Bullet).
- Weld's Hold Separation keeps the pieces as far apart as they start, for anchors at the pieces' centres (Connect Adjacent Pieces). Off, the weld pulls the anchors together.
- Primitive attributes named like the node's parameters modify them: floats multiply, others replace. With Attributes Replace Values on, floats set the value too (in degrees for angles).
- A float of -3.4e38 or a toggle of -1 means not set: the node's value is kept. **Box3D Constraint Properties** writes these for the lines it does not edit. It can also scale a number by an attribute of the constraint or of the pieces at its two ends (a setting's Scaling and Attribute, Geometry Attribute Transfer).
- With **Breakable** on, a joint past its Break Force or Torque goes into the `broken` group, or switches to its `next_constraint_name`.
- Each step the solver writes `force`, `distance`, `torque` and `angle` (radians, the pieces' rotation relative to each other since the start) onto each line.
