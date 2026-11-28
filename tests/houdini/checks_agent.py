"""Crowd agent ragdolls (docs/agents/todo.md, "Agent ragdolls"): SideFX's PinnedRagdolls and PartialRagdolls examples run
with the RBD Bullet Solver and with the Box3D Solver in its place, and the agents' bodies compared through the arrays
the solver writes on each agent's point."""

import os
import tempfile

import hou
from b3dtest import Result, SetupError, keep_scene, set_parm

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
    `active_agent` 0: kinematic bodies), turns them dynamic when the crowd state flips them (frame 36 has a mix in
    Bullet's run), keeps the same bodies throughout, leaves the agents' animated pose alone while no body is simulated
    (frame 12: each agent's point within 0.02 of Bullet's) and the solver node has no error."""
    first, _ = agent_arrays("PartialRagdolls", "box3d", 1)
    late, node = agent_arrays("PartialRagdolls", "box3d", 40)
    problems = []
    counts = [len(bodies(a)) for a in first], [len(bodies(a)) for a in late]
    if not first or any(n != 15 for n in counts[0] + counts[1]):
        problems.append(f"bodies per agent on frame 1 {counts[0]}, on frame 40 {counts[1]} (expected 15 each)")
    flags = sum(1 for a in late for v in a.get("active_agent", []) if v)
    # Before the crowd turns them into ragdolls the agents are animated, bodies kinematic: their points are where
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


def test_agent_partial_bullet():
    """PartialRagdolls against Bullet: an `active_agent` 0 transform's body follows the animation (kinematic, with the
    animation's velocity), so on frame 30, all animated, every agent's bodies are where Bullet's are (within 0.01 on
    average), and the parts the crowd turns into ragdolls on frame 34 carry on from the animated pose: on frame 40 the
    bodies are within 0.08 of Bullet's on average (0.034 measured; bodies left where they were made were 1.5 off)."""
    rows, problems = [], []
    for frame, limit in ((30, 0.01), (40, 0.08)):
        found = {}
        for solver in ("bullet", "box3d"):
            arrays, _ = agent_arrays("PartialRagdolls", solver, frame)
            found[solver] = [bodies(a) for a in arrays]
            keep_scene()
            hou.hipFile.clear(suppress_save_prompt=True)
        gaps = [(b[t][1] - o[t][1]).length() for b, o in zip(found["bullet"], found["box3d"]) for t in b if t in o]
        mean = sum(gaps) / len(gaps) if gaps else float("inf")
        rows.append(f"frame {frame}: bodies {mean:.3f} from Bullet's on average")
        if mean > limit:
            problems.append(f"frame {frame}: {mean:.3f} (expected within {limit})")
    return [Result("agent_partial_bullet", Result.FAIL if problems else Result.PASS,
                   "; ".join(rows) + (f"; PROBLEMS: {'; '.join(problems)}" if problems else ""))]

def test_agent_joints_bullet():
    """PinnedRagdolls against Bullet: the agents' internal Cone Twist joints and the Pin of each right hand to the box
    hold the ragdolls as Bullet's do. On frame 6, before the ragdolls swing apart chaotically, every body is on average
    within 0.08 of Bullet's (0.055 measured, the example's motors on in both; 0.16 with a mirrored rig
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
            keep_scene()
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


def spread_agents(crowd):
    """PinnedRagdolls' five agents 2 apart instead of 1.375 (still over the box their hands are pinned to, which spans
    -5 to 5 in x): they never swing into each other, so a change to one leaves the others' paths alone."""
    line = crowd.node("line1")
    line.parm("originx").set(-4.0)
    line.parm("dist").set(8.0)


def test_agent_rebuild():
    """PinnedRagdolls in the Box3D Solver, its world rebuilt mid-simulation (a checkpoint written at frame 15, the scene
    saved and reopened, played on to 30) against a straight run, the agents spread apart (spread_agents: else they
    swing into each other, which makes any difference grow). The rebuilt bodies take their rest from the agent points'
    arrays, so pivot_agent stays what it was (within 1e-4) and the bodies on frame 30 are on average within 0.05 of the
    straight run's (warm starting and contacts start again, so not exactly)."""
    try:
        crowd, sim = agent_oracle.load("PinnedRagdolls", "box3d")
    except (StopIteration, hou.OperationFailed, hou.LoadWarning) as e:
        raise SetupError(f"the PinnedRagdolls example could not be loaded: {e}")
    spread_agents(crowd)
    for f in range(1, 31):
        hou.setFrame(f)
        sim.simulation().objects()
    straight = agent_point_arrays(sim)
    keep_scene()
    hou.hipFile.clear(suppress_save_prompt=True)

    crowd, sim = agent_oracle.load("PinnedRagdolls", "box3d")
    spread_agents(crowd)
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
    keep_scene()
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

# A Pin from agent 0's left hand to its rig transform `{target}`, added to the ragdolls' constraints: the hand's anchor
# at its transform's origin, the other at the hand's place in `{target}`'s space (local_P), as an Agent Constraint
# Network writes them.
HAND_PIN_SOP = r"""
import hou
node = hou.pwd()
geo = node.geometry()
agents = node.inputs()[1].geometry()
pr = agents.prims()[0]
name = pr.attribValue("name")
rig = pr.rig()
full = pr.fullTransform()
hand_world = pr.worldTransform(rig.findTransform("LeftFingerBase_To_LeftHandIndex1")) * full
target_world = pr.worldTransform(rig.findTransform("{target}")) * full
hand = hou.Vector3(hand_world.extractTranslates())
prim = geo.createPolygon(False)
for anchor, local in ((name + "/LeftFingerBase_To_LeftHandIndex1", (0.0, 0.0, 0.0)),
                      (name + "/{target}", tuple(hand * target_world.inverted()))):
    pt = geo.createPoint()
    pt.setPosition(hand)
    pt.setAttribValue("name", anchor)
    pt.setAttribValue("anchor_type", "agent")
    pt.setAttribValue("local_P", local)
    pt.setAttribValue("local_orient", (0.0, 0.0, 0.0, 1.0))
    prim.addVertex(pt)
prim.setAttribValue("constraint_name", "Pin")
prim.setAttribValue("constraint_type", "all")
"""


def edited_ragdolls(solver, frames, snippet=None, pin_to=None):
    """PinnedRagdolls played to each of `frames` with the ragdolls' constraint points edited by the VEX `snippet` or a
    hand Pin to `pin_to` added: {frame: {transform: body centre}} of agent 0, and the solver's warnings per frame."""
    crowd, sim = agent_oracle.load("PinnedRagdolls", solver)
    if snippet:
        edit = crowd.createNode("attribwrangle", "edit_anchors")
        edit.setInput(0, sim.inputs()[1])
        edit.parm("class").set(2)  # points
        edit.parm("snippet").set(snippet)
        sim.setInput(1, edit)
    if pin_to:
        pin = crowd.createNode("python", "hand_pin")
        pin.setInput(0, sim.inputs()[1])
        pin.setInput(1, sim.inputs()[0])
        pin.parm("python").set(HAND_PIN_SOP.replace("{target}", pin_to))
        sim.setInput(1, pin)
    node = next((c for c in sim.children() if c.type().name() == "box3d_solver"), None)
    found, warnings = {}, {}
    for f in range(1, max(frames) + 1):
        hou.setFrame(f)
        objects = sim.simulation().objects()
        warnings[f] = list(node.warnings()) if node is not None else []
        if f not in frames:
            continue
        obj = next(o for o in objects if o.geometry() is not None and o.geometry().prims()
                   and o.geometry().prims()[0].type().name() == "Agent")
        found[f] = {i: centre for i, (mass, centre) in bodies(agent_point_arrays_of(obj.geometry())[0]).items()}
    keep_scene()
    hou.hipFile.clear(suppress_save_prompt=True)
    return found, warnings


def agent_point_arrays_of(geo):
    """Per agent point of `geo`, its arrays as agent_arrays gives them."""
    return [{a: list(pt.attribValue(a)) for a in ("pivot_agent", "trans_agent", "mass_agent")
             if geo.findPointAttrib(a) is not None} for pt in geo.points()]


def mean_gap(a, b):
    """Mean distance between the bodies of two {transform: centre} dicts."""
    gaps = [(a[t] - b[t]).length() for t in a if t in b]
    return sum(gaps) / len(gaps) if gaps else float("inf")


def test_agent_anchor_local():
    """An agent anchor is its rig transform's matrix at rest applied to its local_P, turned by its local_orient, as RBD
    Bullet Solver has it: its P is not read. PinnedRagdolls with every agent anchor's P moved 0.3 in x: the bodies on
    frame 6 are where they are without the edit (within 1e-6). With every local_P moved 2 along the transform's z (about
    0.11): they move (more than 0.02), and stay near Bullet's with the same edit (within 0.08 on average, as
    agent_joints_bullet's unedited run)."""
    base, _ = edited_ragdolls("box3d", (6,))
    moved_p, _ = edited_ragdolls("box3d", (6,), 'if (s@anchor_type == "agent") v@P += {0.3, 0, 0};')
    local = 'if (s@anchor_type == "agent") v@local_P += {0, 0, -2};'
    moved_local, _ = edited_ragdolls("box3d", (6,), local)
    bullet_local, _ = edited_ragdolls("bullet", (6,), local)
    p_off = mean_gap(base[6], moved_p[6])
    local_off = mean_gap(base[6], moved_local[6])
    bullet_gap = mean_gap(bullet_local[6], moved_local[6])
    ok = p_off < 1e-6 and local_off > 0.02 and bullet_gap < 0.08
    return [Result("agent_anchor_local", Result.PASS if ok else Result.FAIL,
                   f"frame 6: P moved changes the bodies by {p_off:.6f}, local_P moved by {local_off:.3f}; with local_P "
                   f"moved the bodies are {bullet_gap:.3f} from Bullet's on average")]


def test_agent_anchor_no_body():
    """A constraint naming an agent transform with no body on or above it (the rig's root, MASTER) makes no joint, as
    RBD Bullet Solver makes none: a Pin from agent 0's left hand to MASTER leaves the hand where it is without the Pin
    (frame 12, within 1e-6), and frame 1's warning names the ignored anchor."""
    free, _ = edited_ragdolls("box3d", (12,))
    pinned, warnings = edited_ragdolls("box3d", (12,), pin_to="MASTER")
    hand = 4  # LeftFingerBase_To_LeftHandIndex1 in the biped's rig
    off = (free[12][hand] - pinned[12][hand]).length()
    warned = any("no ragdoll body" in w for w in warnings.get(1, []))
    ok = off < 1e-6 and warned
    return [Result("agent_anchor_no_body", Result.PASS if ok else Result.FAIL,
                   f"frame 12: the hand pinned to MASTER is {off:.6f} from the free hand's; frame 1 warning "
                   f"{'given' if warned else 'missing'}: {warnings.get(1, [])[:1]}")]

# A Pin from agent 0's left hand to agent 1's Hips (agent 1 has no bodies: not a ragdoll).
PIN_TO_AGENT1_SOP = r"""
import hou
node = hou.pwd()
geo = node.geometry()
agents = node.inputs()[1].geometry()
a0, a1 = agents.prims()[0], agents.prims()[1]
hand_world = a0.worldTransform(a0.rig().findTransform("LeftFingerBase_To_LeftHandIndex1")) * a0.fullTransform()
hips_world = a1.worldTransform(a1.rig().findTransform("Hips")) * a1.fullTransform()
hand = hou.Vector3(hand_world.extractTranslates())
prim = geo.createPolygon(False)
for anchor, local in ((a0.attribValue("name") + "/LeftFingerBase_To_LeftHandIndex1", (0.0, 0.0, 0.0)),
                      (a1.attribValue("name") + "/Hips", tuple(hand * hips_world.inverted()))):
    pt = geo.createPoint()
    pt.setPosition(hand)
    pt.setAttribValue("name", anchor)
    pt.setAttribValue("anchor_type", "agent")
    pt.setAttribValue("local_P", local)
    pt.setAttribValue("local_orient", (0.0, 0.0, 0.0, 1.0))
    prim.addVertex(pt)
prim.setAttribValue("constraint_name", "Pin")
prim.setAttribValue("constraint_type", "all")
"""


def agent1_not_ragdoll_run(pin):
    """PinnedRagdolls in the Box3D Solver with agent 1 taken out of the ragdoll state (its `state` one the crowd does not
    have), and with `pin` agent 0's left hand pinned to agent 1's Hips: agent 0's hand on frame 12, the number of agent
    1's bodies, and the solver's warnings per frame."""
    crowd, sim = agent_oracle.load("PinnedRagdolls", "box3d")
    out = crowd.createNode("attribwrangle", "agent1_not_ragdoll")
    out.setInput(0, sim.inputs()[0])
    out.parm("snippet").set('if (@ptnum == 1) s@state = "idle";')
    sim.setInput(0, out)
    if pin:
        sop = crowd.createNode("python", "pin_to_agent1")
        sop.setInput(0, sim.inputs()[1])
        sop.setInput(1, out)
        sop.parm("python").set(PIN_TO_AGENT1_SOP)
        sim.setInput(1, sop)
    node = next((c for c in sim.children() if c.type().name() == "box3d_solver"), None)
    warnings = {}
    for f in range(1, 13):
        hou.setFrame(f)
        objects = sim.simulation().objects()
        warnings[f] = list(node.warnings()) if node is not None else []
    obj = next(o for o in objects if o.geometry() is not None and o.geometry().prims()
               and o.geometry().prims()[0].type().name() == "Agent")
    arrays = agent_point_arrays_of(obj.geometry())
    hand = bodies(arrays[0]).get(4, (0.0, hou.Vector3()))[1]  # LeftFingerBase_To_LeftHandIndex1
    keep_scene()
    hou.hipFile.clear(suppress_save_prompt=True)
    return hand, len(bodies(arrays[1])), warnings


def test_agent_not_ragdoll():
    """A crowd agent that is not a ragdoll (an empty `active_agent`: here PinnedRagdolls' agent 1, taken out of the
    ragdoll state) gets no body, as in RBD Bullet Solver: not one per transform, and not one for the agent as a packed
    piece (no "used a box" or unmatched-body warning). A Pin from agent 0's hand to it makes no joint, as Bullet makes
    none: the hand on frame 12 is where it is without the Pin (within 1e-6), and frame 1's warning names the ignored
    anchors."""
    free_hand, free_bodies, free_warnings = agent1_not_ragdoll_run(False)
    pinned_hand, _, warnings = agent1_not_ragdoll_run(True)
    stray = sorted({w.split(": ", 1)[-1] for ws in free_warnings.values() for w in ws
                    if "used a box" in w or "could not be matched" in w})
    off = (free_hand - pinned_hand).length()
    warned = any("no ragdoll body" in w for w in warnings.get(1, []))
    ok = free_bodies == 0 and not stray and off < 1e-6 and warned
    return [Result("agent_not_ragdoll", Result.PASS if ok else Result.FAIL,
                   f"agent 1 has {free_bodies} bodies; stray-body warnings {stray or 'none'}; the hand pinned to agent 1 "
                   f"is {off:.6f} from the free hand's; frame 1 agent-anchor warning {'given' if warned else 'missing'}")]

def after_crowd_solver(sim, name, snippet):
    """A Geometry Wrangle DOP running `snippet` on the agents right after the Crowd Solver in the Multiple Solver, before
    the rigid body solver."""
    multi = next(c for c in sim.children() if c.type().name() == "multisolver")
    crowd_solver = next(c for c in sim.children() if c.type().name().startswith("crowdsolver"))
    wrangle = sim.createNode("geometrywrangle", name)
    wrangle.parm("snippet").set(snippet)
    inputs = list(multi.inputs())
    at = inputs.index(crowd_solver)
    for i in range(len(inputs) - 1, at, -1):
        multi.setInput(i + 1, inputs[i])
    multi.setInput(at + 1, wrangle)


def agent_bodies_off_pose(geo, point=0):
    """The mean distance of an agent's bodies (pivot_agent + trans_agent) from their shapes in its pose now, and its
    arrays."""
    prim = geo.prims()[point]
    arrays = agent_point_arrays_of(geo)[point]
    full = prim.fullTransform()
    offs = []
    for binding in prim.collisionLayers()[0].bindings():
        t = binding.transformId()
        found = bodies(arrays).get(t)
        if found is None:
            continue
        shape = binding.shape().geometry().points()[0].position() * (prim.worldTransform(t) * full)
        offs.append((found[1] - shape).length())
    return (sum(offs) / len(offs) if offs else 0.0), arrays


def test_agent_update_xform():
    """The Crowd Solver's bullet_updateagentxform says whether a ragdoll moves its agent's own transform (1 for a whole
    ragdoll, 0 for a partial one), and the Box3D Solver reads it as RBD Bullet Solver does: PinnedRagdolls with it
    forced to 0 keeps every agent's point where it was (P and orient on frame 12 as on frame 1, within 1e-5), while the
    rig is still posed from the bodies (the hips transform follows its body: within 0.1)."""
    crowd, sim = agent_oracle.load("PinnedRagdolls", "box3d")
    after_crowd_solver(sim, "updxform_0", "i@bullet_updateagentxform = 0;")
    start = moved = None
    hips_gap = 0.0
    for f in range(1, 13):
        hou.setFrame(f)
        objects = sim.simulation().objects()
        if f not in (1, 12):
            continue
        obj = next(o for o in objects if o.geometry() is not None and o.geometry().prims()
                   and o.geometry().prims()[0].type().name() == "Agent")
        geo = obj.geometry()
        now = [(hou.Vector3(p.position()), hou.Vector4(p.attribValue("orient"))) for p in geo.points()]
        if f == 1:
            start = now
            continue
        moved = max(max((a[0] - b[0]).length(), (a[1] - b[1]).length()) for a, b in zip(start, now))
        prim, arrays = geo.prims()[0], agent_point_arrays_of(geo)[0]
        hips = prim.rig().findTransform("Hips")
        pose = hou.Vector3((prim.worldTransform(hips) * prim.fullTransform()).extractTranslates())
        hips_gap = (bodies(arrays)[hips][1] - pose).length()
    keep_scene()
    hou.hipFile.clear(suppress_save_prompt=True)
    ok = moved is not None and moved < 1e-5 and hips_gap < 0.1
    return [Result("agent_update_xform", Result.PASS if ok else Result.FAIL,
                   f"bullet_updateagentxform 0: the agents' points moved at most {moved:.6f} by frame 12; agent 0's hips "
                   f"transform is {hips_gap:.3f} from its body")]


def test_agent_leave_ragdoll():
    """Agents leaving the ragdoll state, as RBD Bullet Solver handles it (PartialRagdolls, the agents' active_agent
    changed after the Crowd Solver from frame 46). Every element 0: the bodies follow the animation from that frame on
    (frames 46 and 50, agent 0's bodies within 0.001 of their shapes in its pose: a body just turned kinematic keeps no
    velocity of its own). An empty array (no longer a ragdoll): its bodies go, and its arrays
    stay as they were last written (frames 47 and 50 equal), as Bullet leaves them; its ragdoll's constraints are
    dropped without a warning (frame 46), as is expected there."""
    rows, ok = [], True
    for mode, body in (("zero", "int a[] = i[]@active_agent; for (int k = 0; k < len(a); ++k) a[k] = 0; "
                                "i[]@active_agent = a;"),
                       ("empty", "int a[]; i[]@active_agent = a;")):
        crowd, sim = agent_oracle.load("PartialRagdolls", "box3d")
        after_crowd_solver(sim, "leave_ragdoll", "if (@Frame >= 46) { " + body + " }")
        seen, warned = {}, []
        node = next((c for c in sim.children() if c.type().name() == "box3d_solver"), None)
        for f in range(1, 51):
            hou.setFrame(f)
            objects = sim.simulation().objects()
            if f == 46 and node is not None:
                warned = [w for w in node.warnings() if "no ragdoll body" in w]
            if f not in (46, 47, 50):
                continue
            obj = next(o for o in objects if o.geometry() is not None and o.geometry().prims()
                       and o.geometry().prims()[0].type().name() == "Agent")
            seen[f] = agent_bodies_off_pose(obj.geometry())
        keep_scene()
        hou.hipFile.clear(suppress_save_prompt=True)
        if mode == "zero":
            off = max(seen[46][0], seen[50][0])
            ok = ok and off < 0.001
            rows.append(f"all animated: bodies {seen[46][0]:.4f} and {seen[50][0]:.4f} from the animation on frames 46 "
                        f"and 50")
        else:
            frozen = seen[47][1] == seen[50][1]
            ok = ok and frozen and bool(bodies(seen[50][1])) and not warned
            rows.append(f"not a ragdoll: arrays {'unchanged' if frozen else 'CHANGED'} from frame 47 to 50 "
                        f"({len(bodies(seen[50][1]))} bodies listed), agent-anchor warning "
                        f"{'GIVEN' if warned else 'none'}")
    return [Result("agent_leave_ragdoll", Result.PASS if ok else Result.FAIL, "; ".join(rows))]

def crowd_change_run(solver, change, spread=False):
    """PinnedRagdolls played to frame 30 with the crowd changed from frame 20: `leave` removes agent 2's point (a
    Geometry Wrangle after the Crowd Solver), `join` has the crowd source emit the agents again (renamed `late_*`, 3
    higher), None changes nothing; `spread`: the agents spread apart (spread_agents). Body centres per agent name on
    frame 30, body counts per name on frame 20, and the solver's warnings from frame 20 on."""
    crowd, sim = agent_oracle.load("PinnedRagdolls", solver)
    if spread:
        spread_agents(crowd)
    if change == "leave":
        after_crowd_solver(sim, "leave_crowd", 'if (@Frame >= 20 && s@name == "mocapbiped3_2") removepoint(0, @ptnum);')
    elif change == "join":
        late = crowd.createNode("attribwrangle", "late_agents")
        late.setInput(0, sim.inputs()[0])
        late.parm("snippet").set('if (@Frame >= 20) { s@name = "late_" + s@name; v@P += {0, 3, 0}; }')
        sim.setInput(0, late)
        sim.node("crowdsource").parm("impulseactiveate").setExpression("$SF == 1 || $SF == 20")
    node = next((c for c in sim.children() if c.type().name() == "box3d_solver"), None)
    warnings, counts = [], {}
    for f in range(1, 31):
        hou.setFrame(f)
        objects = sim.simulation().objects()
        if f >= 20 and node is not None:
            warnings += list(node.warnings())
        if f in (20, 30):
            obj = next(o for o in objects if o.geometry() is not None and o.geometry().prims()
                       and o.geometry().prims()[0].type().name() == "Agent")
            geo = obj.geometry()
            found = {pt.attribValue("name"): {t: c for t, (m, c) in bodies(a).items()}
                     for pt, a in zip(geo.points(), agent_point_arrays_of(geo))}
            if f == 20:
                counts = {n: len(b) for n, b in found.items()}
    keep_scene()
    hou.hipFile.clear(suppress_save_prompt=True)
    return found, counts, warnings


def test_agent_crowd_changes():
    """Agents leaving and joining the crowd mid-simulation (PinnedRagdolls, from frame 20), as RBD Bullet Solver handles
    them. An agent removed: its bodies go, the others carry on exactly as without the change (frame 30, within 1e-9, the
    agents spread apart so that it would not have hit them), and its ragdoll's constraints are dropped without a
    warning. A second batch of agents emitted: each gets its 15
    bodies on the frame it joins, and on frame 30 they are where Bullet's are (within 0.02 on average; 0.002 measured),
    with no warning."""
    rows, ok = [], True
    base, _, _ = crowd_change_run("box3d", None, spread=True)
    left, _, warned = crowd_change_run("box3d", "leave", spread=True)
    same = all(max((left[n][t] - base[n][t]).length() for t in left[n]) < 1e-9 for n in left)
    gone = "mocapbiped3_2" not in left
    ok = ok and same and gone and not warned
    rows.append(f"left: agent gone {gone}, the others {'unchanged' if same else 'CHANGED'}, warnings {warned[:1] or 'none'}")
    joined, counts, warned = crowd_change_run("box3d", "join")
    bullet, _, _ = crowd_change_run("bullet", "join")
    late = [n for n in joined if n.startswith("late_")]
    gaps = [(joined[n][t] - bullet[n][t]).length() for n in late if n in bullet for t in joined[n] if t in bullet[n]]
    mean = sum(gaps) / len(gaps) if gaps else float("inf")
    bodies_ok = len(late) == 5 and all(counts.get(n) == 15 for n in late)
    ok = ok and bodies_ok and mean < 0.02 and not warned
    rows.append(f"joined: {len(late)} agents with {[counts.get(n) for n in late]} bodies on frame 20, {mean:.3f} from "
                f"Bullet's on frame 30, warnings {warned[:1] or 'none'}")
    return [Result("agent_crowd_changes", Result.PASS if ok else Result.FAIL, "; ".join(rows))]


def shaped_ragdolls(solver, kind):
    """PinnedRagdolls with every collision shape of the Agent Collision Layer set to `kind`: box, capsule, sphere, or a
    Custom shape from a SOP, `tetra` (a Platonic Solids tetrahedron of radius 5, centred on its centroid) or `cone` (a
    polygon tube of radius 4 narrowing to a point, 12 high, its centroid 3 off its bounding box centre). Agent 0's
    {transform: (mass_agent, pivot_agent)} on frame 1."""
    crowd, sim = agent_oracle.load("PinnedRagdolls", solver)
    layer = next(n for n in crowd.parent().allSubChildren() if n.type().name() == "agentcollisionlayer")
    sop = None
    if kind == "tetra":
        sop = layer.parent().createNode("platonic", "custom_shape")
        sop.parm("radius").set(5.0)
    elif kind == "cone":
        sop = layer.parent().createNode("tube", "custom_shape")
        for name, value in (("type", "poly"), ("rad1", 4.0), ("rad2", 0.0), ("height", 12.0), ("cols", 12),
                            ("cap", True), ("ty", 3.0)):
            sop.parm(name).set(value)
    i = 0
    while layer.parm(f"shape{i}") is not None:
        if sop is None:
            layer.parm(f"shape{i}").set(kind)
        else:
            layer.parm(f"shape{i}").set("custom")
            layer.parm(f"soppath{i}").set(sop.path())
            # Unset (0) when the type is set from a script.
            layer.parmTuple(f"customshapes{i}").set((1.0, 1.0, 1.0))
        i += 1
    hou.setFrame(1)
    objects = sim.simulation().objects()
    obj = next(o for o in objects if o.geometry() is not None and o.geometry().prims()
               and o.geometry().prims()[0].type().name() == "Agent")
    arrays = agent_point_arrays_of(obj.geometry())[0]
    found = {t: (m, hou.Vector3(arrays["pivot_agent"][3 * t:3 * t + 3]))
             for t, m in enumerate(arrays.get("mass_agent", [])) if m > 0.0}
    keep_scene()
    hou.hipFile.clear(suppress_save_prompt=True)
    return found


def test_agent_shapes_bullet():
    """Collision shapes other than capsules against RBD Bullet Solver (PinnedRagdolls with every shape a box, a sphere,
    then a Custom tetrahedron and cone): a body's centre (`pivot_agent`) is a one-point primitive's point, else the
    centroid of the shape's convex hull, and its mass the density times the shape's volume with no factor
    (kAgentMassScale is a capsule's). Bullet's are the same for the box and sphere; for a custom shape it samples the
    volume, so its mass is off by up to 10% (the tetrahedron 9% light, the cone 1 to 8% heavy, measured) and the
    cone's centre is 0.03 from the centroid (its tetrahedron's is exact)."""
    # kind: (lowest and highest mass ratio, largest centre gap)
    limits = {"box": (0.99, 1.01, 0.005), "sphere": (0.99, 1.01, 0.005), "tetra": (1.0, 1.15, 0.005),
              "cone": (0.9, 1.05, 0.04)}
    rows, ok = [], True
    for kind, (lo, hi, most) in limits.items():
        ours, theirs = shaped_ragdolls("box3d", kind), shaped_ragdolls("bullet", kind)
        both = [t for t in theirs if t in ours]
        ratio = [ours[t][0] / theirs[t][0] for t in both if theirs[t][0] > 0]
        if not ratio:
            ok = False
            rows.append(f"{kind}: no bodies")
            continue
        gap = max((ours[t][1] - theirs[t][1]).length() for t in both)
        ok = ok and len(both) == len(theirs) and lo < min(ratio) and max(ratio) < hi and gap < most
        rows.append(f"{kind}: {len(both)} bodies, mass ratio {min(ratio):.3f} to {max(ratio):.3f}, centres within "
                    f"{gap:.4f}")
    return [Result("agent_shapes_bullet", Result.PASS if ok else Result.FAIL, "; ".join(rows))]


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
    "agent_anchor_local": test_agent_anchor_local,
    "agent_anchor_no_body": test_agent_anchor_no_body,
    "agent_not_ragdoll": test_agent_not_ragdoll,
    "agent_partial_bullet": test_agent_partial_bullet,
    "agent_update_xform": test_agent_update_xform,
    "agent_leave_ragdoll": test_agent_leave_ragdoll,
    "agent_crowd_changes": test_agent_crowd_changes,
    "agent_shapes_bullet": test_agent_shapes_bullet,
}
