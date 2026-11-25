"""Helpers the checks_sop* modules share: building a Solver SOP, playing it, reading positions."""

import hou
from b3dtest import create, display, new_geo, set_parm


def solver_sop(name, geometry=None, constraints=None, proxy=None, collision=None):
    """A geo object holding a Box3D Solver SOP; each input is a SOP path (Object Merge) or None.
    Returns the solver SOP."""
    geo = new_geo(name)
    solver = create(geo, "box3d_solver", "box3d_solver")
    for i, path in enumerate((geometry, constraints, proxy, collision)):
        if path is None:
            continue
        merge = create(geo, "object_merge", f"input{i}")
        set_parm(merge, ["objpath1"], path)
        solver.setInput(i, merge)
    display(solver)
    return solver


def play(sop, frames, sample):
    """Plays frames 1..frames in order (the simulation needs every frame); sample(sop, f) per frame."""
    out = {}
    for f in range(1, frames + 1):
        hou.setFrame(f)
        sop.cook(force=False)
        out[f] = sample(sop, f)
    return out


def named_positions(geo):
    has_name = geo.findPointAttrib("name") is not None
    return {(pt.attribValue("name") if has_name else str(pt.number())): pt.position() for pt in geo.points()}


def bullet_sop(name, geometry=None, constraints=None, proxy=None, collision=None):
    """solver_sop's scene with the RBD Bullet Solver SOP (rbdbulletsolver) in place of ours: same inputs,
    same order. SetupError if the node does not exist."""
    geo = new_geo(name)
    solver = create(geo, "rbdbulletsolver", "rbdbulletsolver")
    for i, path in enumerate((geometry, constraints, proxy, collision)):
        if path is None:
            continue
        merge = create(geo, "object_merge", f"input{i}")
        set_parm(merge, ["objpath1"], path)
        solver.setInput(i, merge)
    display(solver)
    return solver


def _both_sops(name, **inputs):
    """{"box3d": our Solver SOP, "bullet": RBD Bullet Solver SOP} on the same inputs."""
    return {"box3d": solver_sop(f"{name}_box3d", **inputs), "bullet": bullet_sop(f"{name}_bullet", **inputs)}
