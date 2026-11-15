# Joints

[Back to the README](../README.md)

Joints use Houdini's **Constraint Network** DOP: wire Box3D Constraint Relationship nodes into its second input. Each two-point line is one joint:

- `constraint_name` names a relationship's Data Name (by default its type: `Weld`, `Revolute`, ...).
- Each end's `name` picks a piece (empty = the world); its `P` and `orient` are the joint frame, placed on the pieces as they were at the start (like Bullet).
- Primitive attributes named like the node's parameters modify them: floats multiply, others replace.
- With **Breakable** on, a joint past its Break Force or Torque goes into the `broken` group, or switches to its `next_constraint_name`.
- Each step the solver writes `force`, `distance`, `torque` and `angle` (radians, the pieces' rotation relative to each other since the start) onto each line.
