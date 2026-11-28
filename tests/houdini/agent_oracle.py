"""Dumps what a solver does to the agents of SideFX's crowd ragdoll examples (PinnedRagdolls, PartialRagdolls), frame by
frame: the per-transform arrays on the agent's point and the posed agent. The RBD Bullet Solver is the oracle for the
agent ragdoll support (docs/agents/todo.md).

    hython tests/houdini/agent_oracle.py [bullet|box3d] [PinnedRagdolls|PartialRagdolls] [frames...]

`box3d` swaps the Box3D Solver in for the example's Bullet solver (in its Multiple Solver). Needs the examples in the
Houdini install (help/examples/nodes/dop/crowdsolver). Prints, per frame and for the first agent: its point's P and
orient, the posed hand transform, and per rig transform with a body: the arrays' pivot + trans (the body's centre of
mass), orient, v, w and mass."""

import os
import re
import sys

sys.dont_write_bytecode = True

import hou  # noqa: E402

EXAMPLES = os.path.join(os.environ.get("HFS", ""), "houdini", "help", "examples", "nodes", "dop", "crowdsolver")


def retarget_paths(top, original):
    """Points the paths inside `top` that name `original` (the example's own node, /obj/<Example>1) at `top` itself,
    wherever they are: a string parameter's value (also inside backticks) or any parameter's expression (ch(),
    chs(), op: paths). A second copy in the same scene is named <Example>2, and without this it reads the first copy's
    nodes: its pin goal (an Object Merge into its own object) then stays where the first copy is when the copy is
    moved. A parameter that cannot be set (locked) is left as it is."""
    if top.path() == original:
        return
    # (not a longer name that starts the same, /obj/<Example>10)
    pattern = re.compile(re.escape(original) + r"(?![\w.])")
    for node in top.allSubChildren():
        for parm in node.parms():
            try:
                keys = parm.keyframes()
                if keys:
                    for key in keys:
                        if key.isExpressionSet() and pattern.search(key.expression()):
                            key.setExpression(pattern.sub(top.path(), key.expression()), key.expressionLanguage())
                            parm.setKeyframe(key)
                elif parm.parmTemplate().type() == hou.parmTemplateType.String:
                    raw = parm.unexpandedString()
                    if pattern.search(raw):
                        parm.set(pattern.sub(top.path(), raw))
            except hou.Error:
                continue


def load(example, solver):
    """The example's crowd geometry object and its DOP network, with the solver `solver` ("bullet" or "box3d")."""
    path = os.path.join(EXAMPLES, example + ".hda")
    hou.hda.installFile(path)
    # (named as the example's own node: its Object Merges refer to /obj/<Example>1 by path)
    top = hou.node("/obj").createNode(example, example + "1")
    top.allowEditingOfContents()  # (some examples keep their DOP network inside the locked asset)
    retarget_paths(top, "/obj/" + example + "1")
    crowd = next(c for c in top.children() if c.type().name() == "geo" and c.node("sim") is not None)
    sim = crowd.node("sim")
    if solver == "box3d":
        multi = next(c for c in sim.children() if c.type().name() == "multisolver")
        bullet = next(c for c in sim.children() if c.type().name() == "bulletrbdsolver")
        mine = sim.createNode("box3d_solver", "box3d_solver")
        # The example's Gravity DOP is the gravity: the solver's own would double it.
        mine.parmTuple("gravity").set((0.0, 0.0, 0.0))
        for i, node in enumerate(multi.inputs()):
            if node == bullet:
                multi.setInput(i, mine)
    return crowd, sim


def chunks(values, size):
    values = list(values)
    return [tuple(round(x, 3) for x in values[i * size:(i + 1) * size]) for i in range(len(values) // size)]


def dump(example, solver, frames):
    crowd, sim = load(example, solver)
    for f in frames:
        hou.setFrame(f)
        obj = next((o for o in sim.simulation().objects()
                    if o.geometry() is not None and o.geometry().prims() and o.geometry().prims()[0].type().name() == "Agent"), None)
        if obj is None:
            print(f"frame {f}: no agent object")
            continue
        geo = obj.geometry()
        pt, prim = geo.points()[0], geo.prims()[0]
        rig = prim.rig()
        names = [rig.transformName(i) for i in range(rig.transformCount())]
        print(f"=== {solver} {example} frame {f}: {len(geo.points())} agents, agent 0 at "
              f"{tuple(round(c, 3) for c in pt.position())}, orient {tuple(round(c, 3) for c in pt.attribValue('orient'))}")
        arrays = {a: pt.attribValue(a) for a in ("trans_agent", "orient_agent", "pivot_agent", "v_agent", "w_agent",
                                                 "mass_agent", "active_agent") if geo.findPointAttrib(a) is not None}
        if not arrays.get("trans_agent"):
            print("  the per-transform arrays are empty")
            continue
        trans, orient = chunks(arrays["trans_agent"], 3), chunks(arrays["orient_agent"], 4)
        pivot, vel, ang = chunks(arrays["pivot_agent"], 3), chunks(arrays["v_agent"], 3), chunks(arrays["w_agent"], 3)
        mass, active = arrays["mass_agent"], arrays["active_agent"]
        for i, name in enumerate(names):
            if mass[i] <= 0:
                continue
            com = tuple(round(pivot[i][k] + trans[i][k], 3) for k in range(3))
            print(f"  {i:2d} {name[:30]:30s} active {active[i]} mass {mass[i]:6.2f} centre {com} orient {orient[i]} "
                  f"v {vel[i]} w {ang[i]}")


if __name__ == "__main__":
    args = sys.argv[1:]
    solver = args.pop(0) if args and args[0] in ("bullet", "box3d") else "bullet"
    example = args.pop(0) if args and not args[0].isdigit() else "PinnedRagdolls"
    dump(example, solver, [int(a) for a in args] or [1, 12])
