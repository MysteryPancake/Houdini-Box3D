"""Crowd agent ragdolls (docs/agents/todo.md, "Agent ragdolls"): SideFX's PinnedRagdolls and PartialRagdolls examples run
with the RBD Bullet Solver and with the Box3D Solver in its place, and the agents' bodies compared through the arrays
the solver writes on each agent's point."""

import os
import tempfile

import hou
from b3dtest import Result, SetupError, set_parm

import agent_oracle


def agent_arrays(example, solver, frames):
    """The first object's agent points of `example` after the solver played frames 1..`frames`: per point, a dict of
    its per-transform arrays as lists (pivot_agent, trans_agent, mass_agent, active_agent...), and the solver node."""
    try:
        crowd, sim = agent_oracle.load(example, solver)
    except (StopIteration, hou.OperationFailed, hou.LoadWarning) as e:
        raise SetupError(f"the {example} example could not be loaded: {e}")
    for f in range(1, frames + 1):
        hou.setFrame(f)
        simulation = sim.simulation()
        simulation.objects()
    obj = next((o for o in simulation.objects()
                if o.geometry() is not None and o.geometry().prims()
                and o.geometry().prims()[0].type().name() == "Agent"), None)
    if obj is None:
        raise SetupError(f"{example} has no agent object")
    geo = obj.geometry()
    out = []
    for pt in geo.points():
        arrays = {a: list(pt.attribValue(a)) for a in ("pivot_agent", "trans_agent", "orient_agent", "mass_agent",
                                                       "active_agent") if geo.findPointAttrib(a) is not None}
        arrays["P"] = list(pt.position())
        out.append(arrays)
    node = next((c for c in sim.children() if c.type().name() == "box3d_solver"), None)
    return out, node


def bodies(arrays):
    """{transform index: (mass, centre)} of the transforms with a body: those with a mass."""
    found = {}
    for i, mass in enumerate(arrays.get("mass_agent", [])):
        if mass > 0.0:
            p, t = arrays["pivot_agent"][3 * i:3 * i + 3], arrays["trans_agent"][3 * i:3 * i + 3]
            found[i] = (mass, hou.Vector3(p) + hou.Vector3(t))
    return found


def test_agent_bodies_bullet():
    """PinnedRagdolls, frame 1: Box3D makes a body for the same rig transforms as Bullet (those with a shape in the
    agent's collision layer, 15 of 27), with the same mass (within 1%: Bullet's `mass_agent`) and the centre of mass
    where Bullet's is (within 0.2 after the first step: the joints of the left forearm correct it by 0.16, as its Cone
    Twist limits are read), for each of the agents, and the arrays have an element for every rig transform."""
    bullet, _ = agent_arrays("PinnedRagdolls", "bullet", 1)
    ours, node = agent_arrays("PinnedRagdolls", "box3d", 1)
    problems = []
    if len(bullet) != len(ours):
        problems.append(f"{len(ours)} agents, Bullet's {len(bullet)}")
    worst_mass, worst_centre, count = 0.0, 0.0, 0
    for k, (b, o) in enumerate(zip(bullet, ours)):
        bb, ob = bodies(b), bodies(o)
        if sorted(bb) != sorted(ob):
            problems.append(f"agent {k}: bodies on transforms {sorted(ob)}, Bullet's {sorted(bb)}")
            continue
        count += len(ob)
        for i in bb:
            worst_mass = max(worst_mass, abs(ob[i][0] - bb[i][0]) / bb[i][0])
            worst_centre = max(worst_centre, (ob[i][1] - bb[i][1]).length())
        for name in ("pivot_agent", "trans_agent", "orient_agent"):
            if len(o.get(name, [])) != len(b.get(name, [])):
                problems.append(f"agent {k}: {name} has {len(o.get(name, []))} values, Bullet's {len(b.get(name, []))}")
    if worst_mass > 0.01:
        problems.append(f"a mass differs from Bullet's by {worst_mass:.1%}")
    if worst_centre > 0.2:
        problems.append(f"a centre of mass is {worst_centre:.3f} from Bullet's")
    return [Result("agent_bodies_bullet", Result.FAIL if problems else Result.PASS,
                   f"{count} bodies on {len(ours)} agents; largest mass difference {worst_mass:.2%}, largest centre "
                   f"difference {worst_centre:.3f}" + (f"; PROBLEMS: {'; '.join(problems)}" if problems else ""))]


def test_agent_bodies_partial():
    """PartialRagdolls, 40 frames: the Box3D Solver makes the bodies when the agents are still animated (every
    `active_agent` 0: static bodies), turns them dynamic when the crowd state flips them (frame 36 has a mix in
    Bullet's run), keeps the same bodies throughout, leaves the agents' animated pose alone while no body is simulated
    (frame 12: each agent's point within 0.02 of Bullet's) and the solver node has no error."""
    first, _ = agent_arrays("PartialRagdolls", "box3d", 1)
    late, node = agent_arrays("PartialRagdolls", "box3d", 40)
    problems = []
    counts = [len(bodies(a)) for a in first], [len(bodies(a)) for a in late]
    if not first or any(n != 15 for n in counts[0] + counts[1]):
        problems.append(f"bodies per agent on frame 1 {counts[0]}, on frame 40 {counts[1]} (expected 15 each)")
    flags = sum(1 for a in late for v in a.get("active_agent", []) if v)
    # Before the crowd turns them into ragdolls the agents are animated, bodies static: their points are where
    # Bullet's are (the solver leaves the animation's pose alone).
    bullet_early, _ = agent_arrays("PartialRagdolls", "bullet", 12)
    ours_early, _ = agent_arrays("PartialRagdolls", "box3d", 12)
    drift = max((hou.Vector3(b["P"]) - hou.Vector3(o["P"])).length() for b, o in zip(bullet_early, ours_early))
    if drift > 0.02:
        problems.append(f"on frame 12 an agent's point is {drift:.3f} from Bullet's")
    errors = list(node.errors()) if node is not None else []
    if errors:
        problems.append(f"solver errors: {errors[:2]}")
    return [Result("agent_bodies_partial", Result.FAIL if problems else Result.PASS,
                   f"bodies per agent {counts[0]} on frame 1, {counts[1]} on frame 40; {flags} simulated transforms on "
                   f"frame 40" + (f"; PROBLEMS: {'; '.join(problems)}" if problems else ""))]


def test_agent_joints_bullet():
    """PinnedRagdolls against Bullet: the agents' internal Cone Twist joints and the Pin of each right hand to the box
    hold the ragdolls as Bullet's do. On frame 6, before the ragdolls swing apart chaotically, every body is on average
    within 0.08 of Bullet's (0.056 measured, Bullet's motors on as the example has them; 0.16 with a mirrored rig
    transform's Cone Twist axes taken unmirrored, which turned the left arm over on the first step). On frames 24 and
    48 the pinned hand's body is within 0.05 of Bullet's (the pin is the same), each ragdoll's centre of its bodies
    within 0.3 (0.09 and 0.25 measured) and no body further than 1.3 from it (Bullet's farthest is 1: without the
    joints the bodies fall apart), and the solver node has no warning."""
    hand = 18  # RightFingerBase_To_RightHandIndex1 in the biped's rig
    rows, problems = [], []
    for frames in (6, 24, 48):
        found = {}
        for solver in ("bullet", "box3d"):
            arrays, node = agent_arrays("PinnedRagdolls", solver, frames)
            found[solver] = [bodies(a) for a in arrays]
            if solver == "box3d" and node is not None and node.warnings():
                problems.append(f"solver warnings: {list(node.warnings())[:2]}")
            hou.hipFile.clear(suppress_save_prompt=True)
        if frames == 6:
            gaps = [(b[t][1] - o[t][1]).length() for b, o in zip(found["bullet"], found["box3d"]) for t in b if t in o]
            mean = sum(gaps) / len(gaps)
            rows.append(f"frame 6: bodies {mean:.3f} from Bullet's on average")
            if mean > 0.08:
                problems.append(f"frame 6: bodies {mean:.3f} from Bullet's on average")
            continue
        pin = centre = spread = 0.0
        for b, o in zip(found["bullet"], found["box3d"]):
            pin = max(pin, (b[hand][1] - o[hand][1]).length())
            cb = sum((v[1] for v in b.values()), hou.Vector3()) / len(b)
            co = sum((v[1] for v in o.values()), hou.Vector3()) / len(o)
            centre = max(centre, (cb - co).length())
            spread = max(spread, max((v[1] - co).length() for v in o.values()))
        rows.append(f"frame {frames}: pinned hand {pin:.3f} from Bullet's, centre {centre:.3f}, farthest body {spread:.2f}")
        if pin > 0.05 or centre > 0.3 or spread > 1.3:
            problems.append(f"frame {frames}: pin {pin:.3f}, centre {centre:.3f}, spread {spread:.2f}")
    return [Result("agent_joints_bullet", Result.FAIL if problems else Result.PASS,
                   "; ".join(rows) + (f"; PROBLEMS: {'; '.join(problems)}" if problems else ""))]


def agent_point_arrays(sim):
    """The agent points' arrays of the simulation now, as agent_arrays gives them."""
    obj = next((o for o in sim.simulation().objects()
                if o.geometry() is not None and o.geometry().prims()
                and o.geometry().prims()[0].type().name() == "Agent"), None)
    if obj is None:
        raise SetupError("no agent object")
    geo = obj.geometry()
    return [{a: list(pt.attribValue(a)) for a in ("pivot_agent", "trans_agent", "mass_agent")
             if geo.findPointAttrib(a) is not None} for pt in geo.points()]


def test_agent_rebuild():
    """PinnedRagdolls in the Box3D Solver, its world rebuilt mid-simulation (a checkpoint written at frame 15, the scene
    saved and reopened, played on to 30) against a straight run. The rebuilt bodies take their rest from the agent
    points' arrays, so pivot_agent stays what it was (within 1e-4) and the bodies on frame 30 are on average within
    0.05 of the straight run's (warm starting and contacts start again, so not exactly)."""
    try:
        crowd, sim = agent_oracle.load("PinnedRagdolls", "box3d")
    except (StopIteration, hou.OperationFailed, hou.LoadWarning) as e:
        raise SetupError(f"the PinnedRagdolls example could not be loaded: {e}")
    for f in range(1, 31):
        hou.setFrame(f)
        sim.simulation().objects()
    straight = agent_point_arrays(sim)
    hou.hipFile.clear(suppress_save_prompt=True)

    crowd, sim = agent_oracle.load("PinnedRagdolls", "box3d")
    folder = tempfile.mkdtemp(prefix="box3d_agent_")
    set_parm(sim, ["explicitcache", "savecheckpoints"], 1)
    set_parm(sim, ["explicitcachename", "checkpointname"], os.path.join(folder, "ckpt.$SF.sim").replace("\\", "/"))
    try:
        set_parm(sim, ["explicitcachensteps", "checkpointtrail"], 1)
    except SetupError:
        pass
    for f in range(1, 16):
        hou.setFrame(f)
        sim.simulation().objects()
    path = sim.path()
    hip = os.path.join(folder, "saved.hip")
    hou.hipFile.save(hip)
    hou.hipFile.load(hip, suppress_save_prompt=True)
    sim = hou.node(path)
    for f in range(15, 31):
        hou.setFrame(f)
        sim.simulation().objects()
    resumed = agent_point_arrays(sim)
    hou.hipFile.clear(suppress_save_prompt=True)

    pivot_off, gaps = 0.0, []
    for a, b in zip(straight, resumed):
        mass = a.get("mass_agent", [])
        for i, m in enumerate(mass):
            if m <= 0.0:
                continue
            pa = hou.Vector3(a["pivot_agent"][3 * i:3 * i + 3])
            pb = hou.Vector3(b["pivot_agent"][3 * i:3 * i + 3])
            pivot_off = max(pivot_off, (pa - pb).length())
            ca = pa + hou.Vector3(a["trans_agent"][3 * i:3 * i + 3])
            cb = pb + hou.Vector3(b["trans_agent"][3 * i:3 * i + 3])
            gaps.append((ca - cb).length())
    if not gaps:
        raise SetupError("no agent bodies in the straight run")
    mean = sum(gaps) / len(gaps)
    ok = pivot_off < 1e-4 and mean < 0.05 and len(straight) == len(resumed)
    return [Result("agent_rebuild", Result.PASS if ok else Result.FAIL,
                   f"after a rebuild at frame 15: pivot_agent moved at most {pivot_off:.6f}, the bodies on frame 30 are "
                   f"{mean:.3f} from the straight run's on average ({len(resumed)} agents, {len(straight)} straight)")]

def test_agent_pose_box3d():
    """PinnedRagdolls with the Box3D Solver, frames 12 and 36: the agents' own pose follows their bodies. Every body's
    transform in the posed agent (its joint origin, through the agent's transform) is where the arrays say its body has
    taken it: the rest origin turned about the body's creation centre by `orient_agent`, moved by `trans_agent` (the
    convention Bullet's arrays and pose follow, checked on its run: residual 0). The first agent's point has moved from
    where it was and is not far from it."""
    crowd, sim = agent_oracle.load("PinnedRagdolls", "box3d")
    hou.setFrame(1)
    rest_agents = crowd.node("AGENTS").geometry().prims()
    count = rest_agents[0].rig().transformCount()
    rest = [[(agent.worldTransform(i) * agent.fullTransform()).extractTranslates() for i in range(count)]
            for agent in rest_agents]
    start = crowd.node("AGENTS").geometry().points()[0].position()
    rows, problems = [], []
    for frame in range(1, 37):
        hou.setFrame(frame)
        simulation = sim.simulation()
        simulation.objects()
        if frame not in (12, 36):
            continue
        obj = simulation.findObject("crowdobject")
        geo = obj.geometry()
        worst, moved = 0.0, 0.0
        for k, (pt, prim) in enumerate(zip(geo.points(), geo.prims())):
            full = prim.fullTransform()
            pivot, trans = list(pt.attribValue("pivot_agent")), list(pt.attribValue("trans_agent"))
            turn, mass = list(pt.attribValue("orient_agent")), list(pt.attribValue("mass_agent"))
            for i in range(count):
                if mass[i] <= 0.0:
                    continue
                joint = (prim.worldTransform(i) * full).extractTranslates()
                c0 = hou.Vector3(pivot[3 * i:3 * i + 3])
                rotation = hou.Quaternion(turn[4 * i:4 * i + 4]).extractRotationMatrix3()
                predicted = c0 + hou.Vector3(trans[3 * i:3 * i + 3]) + (rest[k][i] - c0) * rotation
                worst = max(worst, (joint - predicted).length())
        pt0 = geo.points()[0].position()
        moved = (pt0 - start).length()
        rows.append(f"frame {frame}: largest distance of a posed joint from the arrays' {worst:.4f}, agent 0's point "
                    f"{moved:.2f} from its start")
        if worst > 1e-3:
            problems.append(f"frame {frame}: the pose differs from the arrays by {worst:.4f}")
        if not 0.05 < moved < 6.0:
            problems.append(f"frame {frame}: the agent's point is {moved:.2f} from its start")
    return [Result("agent_pose_box3d", Result.FAIL if problems else Result.PASS,
                   "; ".join(rows) + (f"; PROBLEMS: {'; '.join(problems)}" if problems else ""))]


TESTS = {
    "agent_bodies_bullet": test_agent_bodies_bullet,
    "agent_bodies_partial": test_agent_bodies_partial,
    "agent_joints_bullet": test_agent_joints_bullet,
    "agent_pose_box3d": test_agent_pose_box3d,
    "agent_rebuild": test_agent_rebuild,
}
