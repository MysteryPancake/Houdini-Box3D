"""The Box3D Solver SOP's constraints: Break Thresholds against RBD Bullet Solver, Export Broken Constraints, next
constraints, constraint settings and guide, the constraint output, and Enable Aging."""

import hou
from b3dtest import append_sop, GROUND, Result, close, constraint_lines, packed_boxes, set_parm

from checks_sop_common import _both_sops, bullet_sop, named_positions, play, solver_sop


def test_solver_sop_constraint_settings():
    """The joint settings (Advanced, Constraints): a Weld's Torque Threshold on the node reaches the inner Weld node, and World Scale 2
    makes it four times as large there (a torque is the square of the scale). The asset has its help card."""
    sop = solver_sop("sop_constraints_tab")
    inner = sop.node("dopnet/weld")
    parm = sop.parm("weld_torque_threshold")
    if inner is None or parm is None or inner.parm("torque_threshold") is None:
        return [Result("solver_sop_constraint_settings", Result.FAIL,
                       "no weld_torque_threshold on the asset, or no inner dopnet/weld torque_threshold")]
    parm.set(5.0)
    plain = inner.parm("torque_threshold").eval()
    set_parm(sop, ["usescale"], 1)
    set_parm(sop, ["scale"], 2)
    scaled = inner.parm("torque_threshold").eval()
    ok = close(plain, 5.0, 1e-6) and close(scaled, 20.0, 1e-6)
    results = [Result("solver_sop_constraint_settings", Result.PASS if ok else Result.FAIL,
                      f"Weld Torque Threshold 5: inner {plain}, with World Scale 2 inner {scaled} (expected 5, 20)")]
    glue = [p.name() for p in sop.parms() if p.name().startswith("glue_")]
    results.append(Result("solver_sop_constraint_settings", Result.PASS if glue else Result.FAIL,
                          f"Glue settings on the Constraints tab: {glue[:6]}"))
    helped = "Help" in sop.type().definition().sections()
    results.append(Result("solver_sop_constraint_settings", Result.PASS if helped else Result.FAIL,
                          f"the asset has a Help section: {helped}"))
    return results


def test_solver_sop_keep_broken():
    """Export Broken Constraints, as RBD Bullet Solver: a glued pair (SideFX Glue far too weak to hold
    a landing) dropped onto the ground breaks on the landing (Glue breaks on hit impulse, so a pair that never hits
    anything holds); off, the constraints output drops it; on, it keeps it in the broken group."""
    pieces, names = packed_boxes("sop_broken_pieces", [{"size": (1, 1, 1), "center": (0, 2, 0), "attrs": {"active": 1}},
                                                       {"size": (1, 1, 1), "center": (0, 1, 0), "attrs": {"active": 1}}])
    ground, _ = packed_boxes("sop_broken_ground", [GROUND])
    lines = constraint_lines("sop_broken_network",
                             [(names[0], (0, 1.5, 0), names[1], (0, 1.5, 0), "Glue", {"strength": 1e-6})])
    results = []
    for keep in (0, 1):
        sop = solver_sop(f"sop_keep_broken_{keep}", geometry=pieces, constraints=lines, collision=ground)
        set_parm(sop, ["constraint_keepbroken"], keep)

        def sample(s, f):
            if f != 12:
                return None
            con = s.geometry(1)
            grp = con.findPrimGroup("broken")
            return len(con.prims()), len(grp.prims()) if grp else 0

        nprims, nbroken = play(sop, 12, sample)[12]
        ok = (nprims, nbroken) == ((1, 1) if keep else (0, 0))
        results.append(Result("solver_sop_keep_broken", Result.PASS if ok else Result.FAIL,
                              f"Export Broken Constraints {keep}: {nprims} constraint(s), {nbroken} broken on frame "
                              f"12 (expected {'1, 1' if keep else '0, 0'})"))
    return results


def _active_age_run(tag, bullet, as_attribute):
    """Two boxes far apart, one active from the start and one only from frame 6 (its `active` is set by a wrangle
    each step, read through Override Attributes from SOP), through the Solver SOP (bullet: RBD Bullet Solver SOP)
    with Add Active Age Attribute on and Just Activated named `fresh`. Returns {frame: {piece: (active_age, just
    activated)}} for frames 1 to 12, read from the simulation points."""
    pieces, names = packed_boxes(f"age_{tag}_pieces", [{"size": (1, 1, 1), "center": (0, 3, 0)},
                                                       {"size": (1, 1, 1), "center": (10, 3, 0)}])
    pieces = append_sop(pieces, "attribwrangle", lambda n: (
        set_parm(n, ["class"], "point"), set_parm(n, ["snippet"], "i@active = @ptnum == 0 || @Frame >= 6;")))
    sop = (bullet_sop if bullet else solver_sop)(f"age_{tag}", geometry=pieces)
    set_parm(sop, ["overwritefromsop"], 1)
    set_parm(sop, ["doage" if bullet else "enableaging"], 1)
    set_parm(sop, ["doactiveage" if bullet else "addactiveage"], 1)
    set_parm(sop, ["justactivatedgroup"], "fresh")
    set_parm(sop, ["justactivatedtype"], 1 if as_attribute else 0)
    set_parm(sop, ["transferattributes"], "age w active_age" + (" fresh" if as_attribute else ""))

    def sample(s, f):
        geo = s.geometry(3)
        found = {}
        for pt in geo.points():
            name = pt.attribValue("name")
            age = pt.attribValue("active_age") if geo.findPointAttrib("active_age") else None
            if as_attribute:
                fresh = pt.attribValue("fresh") if geo.findPointAttrib("fresh") else None
            else:
                group = geo.findPointGroup("fresh")
                fresh = int(group.contains(pt)) if group else None
            found[name] = (None if age is None else round(age, 4), fresh)
        return found

    return names, play(sop, 12, sample)


def test_solver_sop_active_age_bullet():
    """Add Active Age Attribute and Just Activated, against the RBD Bullet Solver SOP: a box active from the start and
    one made active on frame 6 have the same active_age (time active, 0 while inactive) and the same Just
    Activated membership on frames 1 to 12, as a point group and as an attribute."""
    results = []
    for as_attribute in (False, True):
        kind = "attribute" if as_attribute else "group"
        runs = {label: _active_age_run(f"{label}_{kind}", label == "bullet", as_attribute)
                for label in ("box3d", "bullet")}
        # The pieces are named in order of creation: compare by position in the sorted names.
        def series(label):
            names, track = runs[label]
            ordered = sorted(names)
            return {f: [track[f].get(n) for n in ordered] for f in track}
        mine, theirs = series("box3d"), series("bullet")
        same = mine == theirs
        sample_frames = (1, 5, 6, 7, 12)
        show = lambda d: {f: d[f] for f in sample_frames}  # noqa: E731
        results.append(Result("solver_sop_active_age_bullet", Result.PASS if same else Result.FAIL,
                              f"Just Activated as a {kind}, (active_age, just activated) per piece: Box3D "
                              f"{show(mine)}, Bullet {show(theirs)}"))
    return results


AGING_CASES = {
    "no life attribute": None,
    "life 0 and 0.25": "f@life = @ptnum == 0 ? 0.25 : 0.0;",
    "life 0.25 and -1": "f@life = @ptnum == 0 ? 0.25 : -1.0;",
    "life 100 on both": "f@life = 100.0;",
    "dead set on one": "i@dead = @ptnum == 1;",
}


def test_solver_sop_aging_bullet():
    """Enable Aging against the RBD Bullet Solver SOP's Age Attribute, on two boxes with different `life` / `dead`
    set on the input: with no life attribute nothing dies, a life of 0 or less dies on the first step, a life of
    0.25 on frame 8, a long life never, and a `dead` set by hand is deleted on the first step. `age` is there from
    frame 1 and grows alike. Compared: the frame each piece is gone on and (age, dead) of what is left on frames 1,
    2 and 12."""
    results = []
    for case, snippet in AGING_CASES.items():
        tag = "".join(ch for ch in case if ch.isalnum())
        pieces, names = packed_boxes(f"agecmp_{tag}_pieces", [{"size": (1, 1, 1), "center": (0, 3, 0)},
                                                             {"size": (1, 1, 1), "center": (10, 3, 0)}])
        if snippet:
            pieces = append_sop(pieces, "attribwrangle", lambda n, code=snippet: (
                set_parm(n, ["class"], "point"), set_parm(n, ["snippet"], code)))
        rows = {}
        for label in ("bullet", "box3d"):
            sop = (bullet_sop if label == "bullet" else solver_sop)(f"agecmp_{tag}_{label}", geometry=pieces)
            set_parm(sop, ["doage" if label == "bullet" else "enableaging"], 1)
            set_parm(sop, ["transferattributes"], "age life dead")

            def sample(s, f):
                geo = s.geometry(3)
                return {pt.attribValue("name"): (round(pt.attribValue("age"), 3) if geo.findPointAttrib("age") else None,
                                                 pt.attribValue("dead") if geo.findPointAttrib("dead") else None)
                        for pt in geo.points()}

            track = play(sop, 12, sample)
            gone = {n: next((f for f in sorted(track) if n not in track[f]), None) for n in names}
            rows[label] = (gone, track[1], track[2], track[12])
        results.append(Result("solver_sop_aging_bullet", Result.PASS if rows["box3d"] == rows["bullet"] else Result.FAIL,
                              f"{case}: (gone on, frame 1, frame 2, frame 12) Box3D {rows['box3d']}, "
                              f"Bullet {rows['bullet']}"))
    return results


def test_solver_sop_constraint_aging_bullet():
    """Enable Aging on constraints, against the RBD Bullet Solver SOP: a glued pair's constraint has `age` 0 on frame
    1 and the time step more each step, alike in both, and a `life` on the constraint changes nothing (it is not
    killed)."""
    pieces, names = packed_boxes("cage_pieces", [{"size": (1, 1, 1), "center": (0, 2, 0), "attrs": {"active": 1}},
                                                 {"size": (1, 1, 1), "center": (0, 1, 0), "attrs": {"active": 1}}])
    ground, _ = packed_boxes("cage_ground", [GROUND])
    results = []
    for case, extra in (("plain", {}), ("with_life", {"life": 0.25})):
        lines = constraint_lines(f"cage_net_{case}", [(names[0], (0, 1.5, 0), names[1], (0, 1.5, 0), "Glue",
                                                       dict({"strength": 1e6}, **extra))])
        rows = {}
        for label in ("bullet", "box3d"):
            sop = (bullet_sop if label == "bullet" else solver_sop)(f"cage_{case}_{label}", geometry=pieces,
                                                                    constraints=lines, collision=ground)
            set_parm(sop, ["doage" if label == "bullet" else "enableaging"], 1)

            def sample(s, f):
                geo = s.geometry(1)
                return [round(p.attribValue("age"), 4) if geo.findPrimAttrib("age") else None for p in geo.prims()]

            track = play(sop, 12, sample)
            rows[label] = {f: track[f] for f in (1, 2, 3, 8, 9, 12)}
        results.append(Result("solver_sop_constraint_aging_bullet",
                              Result.PASS if rows["box3d"] == rows["bullet"] and rows["box3d"][12] == [0.4583] else Result.FAIL,
                              f"constraint {case}, age by frame: Box3D {rows['box3d']}, Bullet {rows['bullet']}"))
    return results


def test_solver_sop_keep_broken_bullet():
    """Export Broken Constraints, against the RBD Bullet Solver SOP, on the scene of solver_sop_keep_broken (a
    glued pair dropped on the ground, Glue far too weak to hold the landing): with it off and on, the number of
    constraint primitives and of those in the `broken` group on frame 12 are the same in both."""
    pieces, names = packed_boxes("sop_broken_b_pieces", [{"size": (1, 1, 1), "center": (0, 2, 0), "attrs": {"active": 1}},
                                                         {"size": (1, 1, 1), "center": (0, 1, 0), "attrs": {"active": 1}}])
    ground, _ = packed_boxes("sop_broken_b_ground", [GROUND])
    lines = constraint_lines("sop_broken_b_network",
                             [(names[0], (0, 1.5, 0), names[1], (0, 1.5, 0), "Glue", {"strength": 1e-6})])
    results = []
    for keep in (0, 1):
        sops = _both_sops(f"sop_keep_broken_b{keep}", geometry=pieces, constraints=lines, collision=ground)
        set_parm(sops["box3d"], ["constraint_keepbroken"], keep)
        set_parm(sops["bullet"], ["constraint_keepbroken"], keep)

        def sample(s, f):
            if f != 12:
                return None
            con = s.geometry(1)
            grp = con.findPrimGroup("broken")
            return len(con.prims()), len(grp.prims()) if grp else 0

        got = {kind: play(sop, 12, sample)[12] for kind, sop in sops.items()}
        results.append(Result("solver_sop_keep_broken_bullet", Result.PASS if got["box3d"] == got["bullet"] else Result.FAIL,
                              f"Export Broken Constraints {keep}: (constraints, broken) on frame 12 Box3D "
                              f"{got['box3d']}, Bullet {got['bullet']}"))
    return results


def _break_impact_held(kind, threshold, tag, strength=1e12, scale_by_strength=False, from_frame=None):
    """Whether a SideFX Glue pair (Strength 1e12: it cannot break by itself) landing from 3 holds with an Impact
    Threshold of `threshold` on the Break Thresholds of the Box3D Solver SOP ("box3d") or the RBD Bullet Solver SOP
    ("bullet"): the constraint primitives left on frame 48. `strength`: the Glue's own (a line attribute);
    `scale_by_strength`: Scale by Attribute, the attribute `strength`; `from_frame`: the From Frame."""
    parms = {"constraint_impactthreshold1": threshold}
    if from_frame is not None:
        parms.update({"constraint_usefromframe1": 1, "constraint_fromframe1": from_frame})
    if scale_by_strength:
        parms.update({"constraint_scaleimpact1": 1, "constraint_impactattrib1": "strength"})
    return _first_break(kind, tag, "Glue", ((0.5, 3, 0), (0.5, 3, 0)), hang={}, enable=("impact",), parms=parms,
                        line_attrs={"strength": strength}, frames=48, centers=((0, 3, 0), (1, 3, 0)), fixed=False,
                        ground=True) is None


def test_solver_sop_break_impact_bullet():
    """Break Thresholds, Impact Threshold (the Constraints tab), against the RBD Bullet Solver SOP: a glued pair
    landing from 3 breaks with a threshold of 5000 and holds with 2e6 in both; the threshold at which it stops
    breaking is found by bisection in each and must agree within a factor of 2. A threshold scaled by the
    `strength` attribute (Scale by Attribute) breaks a pair of Strength 1e7 (too strong to break itself) with a
    threshold of 0.001 and not with one of 0.1, in both. From Frame 100 holds the pair through frame 48, in both."""
    runs = iter(range(1000))
    tag = lambda: f"sop_break_{next(runs)}"  # noqa: E731
    problems, found = [], {}
    for kind in ("box3d", "bullet"):
        if _break_impact_held(kind, 5000.0, tag()):
            problems.append(f"{kind} held with a threshold of 5000")
        if not _break_impact_held(kind, 2e6, tag()):
            problems.append(f"{kind} broke with a threshold of 2e6")
        lo, hi = 5000.0, 2e6  # lo breaks it, hi holds it
        for _ in range(8):
            mid = (lo * hi) ** 0.5
            if _break_impact_held(kind, mid, tag()):
                hi = mid
            else:
                lo = mid
        found[kind] = hi
    ratio = found["bullet"] / found["box3d"]
    if not 0.5 <= ratio <= 2.0:
        problems.append(f"the thresholds differ by {ratio:.2f}x")
    for kind in ("box3d", "bullet"):  # From Frame 100: nothing breaks within the 48 frames played
        if not _break_impact_held(kind, 5000.0, tag(), from_frame=100):
            problems.append(f"{kind} broke before From Frame 100")
    for kind in ("box3d", "bullet"):  # Scale by Attribute: threshold x strength (1e7, too strong to break by itself)
        if _break_impact_held(kind, 0.001, tag(), strength=1e7, scale_by_strength=True):
            problems.append(f"{kind} held with a threshold of 0.001 x strength 1e7")
        if not _break_impact_held(kind, 0.1, tag(), strength=1e7, scale_by_strength=True):
            problems.append(f"{kind} broke with a threshold of 0.1 x strength 1e7")
    return [Result("solver_sop_break_impact_bullet", Result.FAIL if problems else Result.PASS,
                   f"pair stops breaking at an Impact Threshold of {found['box3d']:.0f} (Box3D) and "
                   f"{found['bullet']:.0f} (Bullet), Bullet / Box3D {ratio:.2f}" +
                   (f"; PROBLEMS: {'; '.join(problems)}" if problems else ""))]


def _first_break(kind, tag, cname, anchors, hang=None, enable=(), parms=None, names="Glue", line_attrs=None, frames=24,
                 centers=((0, 4, 0), (0, 2, 0)), fixed=True, ground=False, animate=None):
    """The first frame on which a two-piece scene (a fixed box over a free one, 2 apart) has no constraint left on
    the Break Thresholds of the Box3D Solver SOP ("box3d") or the RBD Bullet Solver SOP ("bullet"), None if it
    holds for `frames`. `anchors`: the two line points; `hang`: the free box's point attributes; `enable`: the
    thresholds to turn on (`parms` sets their values and the rest); every other one is off. `centers`: the boxes'
    centres; `fixed`: the first box is fixed; `ground`: a ground plane under them."""
    pieces, pnames = packed_boxes(f"{tag}_pieces", [{"size": (1, 1, 1), "center": centers[0], "attrs": {"active": 0 if fixed else 1}},
                                                    {"size": (1, 1, 1), "center": centers[1],
                                                     "attrs": dict(hang or {}, active=1)}])
    collision = packed_boxes(f"{tag}_ground", [GROUND])[0] if ground else None
    lines = constraint_lines(f"{tag}_network", [(pnames[0], anchors[0], pnames[1], anchors[1], cname, line_attrs or {})])
    if animate:  # VEX run over the constraint lines every frame
        lines = append_sop(lines, "attribwrangle", lambda n: (set_parm(n, ["class"], "primitive"),
                                                              set_parm(n, ["snippet"], animate)))
    inputs = {"collision": collision} if ground else {}
    sop = (solver_sop if kind == "box3d" else bullet_sop)(f"{tag}_{kind}", geometry=pieces, constraints=lines,
                                                          **inputs)
    set_parm(sop, ["enable_constraintbreaks"], 1)
    set_parm(sop, ["constraint_names1"], names)
    for key in ("angle", "distance", "impact", "torque"):
        set_parm(sop, [f"constraint_use{key}1"], 1 if key in enable else 0)
    for name, value in (parms or {}).items():
        set_parm(sop, [name], value)
    gone = []
    play(sop, frames, lambda s, f: None if s.geometry(1).prims() else gone.append(f))
    for node in [n for n in hou.node("/obj").children() if n.name().startswith(tag)]:
        node.destroy()
    return gone[0] if gone else None


def test_solver_sop_break_atframe_bullet():
    """Break Thresholds, At Frame, against the RBD Bullet Solver SOP: a Glue too strong to break (Strength 1e12) with no
    threshold on goes at Frame 5 in both: the frame it is gone on differs by at most 1. Relative to Start Frame
    with a Start Frame of 3 moves that to 8. Nothing breaks before."""
    problems = []
    got = {}
    for kind in ("box3d", "bullet"):
        for rel, at in ((0, 5), (1, 5)):
            parms = {"constraint_useatframe1": 1, "constraint_atframe1": at, "constraint_relatframe1": rel}
            if rel:
                parms["startframe"] = 3
            got[kind, rel] = _first_break(kind, f"sop_atframe_{kind}{rel}", "Glue", ((0, 3.5, 0), (0, 3.5, 0)),
                                          parms=parms, line_attrs={"strength": 1e12})
    for rel, want in ((0, 5), (1, 8)):
        for kind in ("box3d", "bullet"):
            f = got[kind, rel]
            if f is None or not want <= f <= want + 1:
                problems.append(f"{kind} (relative {rel}) went on frame {f}, expected {want}")
    return [Result("solver_sop_break_atframe_bullet", Result.FAIL if problems else Result.PASS,
                   f"gone on frame (Box3D / Bullet): At Frame 5 {got['box3d', 0]} / {got['bullet', 0]}, relative to "
                   f"Start Frame 3 {got['box3d', 1]} / {got['bullet', 1]}" +
                   (f"; PROBLEMS: {'; '.join(problems)}" if problems else ""))]


def _break_to_next(tag, cname, names, line_attrs, anchors, frames):
    """A fixed box over a free one joined by a `cname` line with `line_attrs` (a next constraint among them) on the Box3D
    Solver SOP, Break Thresholds At Frame 5 in Switch to Next Constraint mode: (the line's attributes, the free box's
    position) on the last of `frames`."""
    pieces, pnames = packed_boxes(f"{tag}_pieces", [{"size": (1, 1, 1), "center": (0, 4, 0), "attrs": {"active": 0}},
                                                    {"size": (1, 1, 1), "center": (0, 2, 0), "attrs": {"active": 1}}])
    lines = constraint_lines(f"{tag}_network", [(pnames[0], anchors[0], pnames[1], anchors[1], cname, line_attrs)])
    sop = solver_sop(f"{tag}_sop", geometry=pieces, constraints=lines)
    set_parm(sop, ["enable_constraintbreaks"], 1)
    set_parm(sop, ["constraint_names1"], names)
    for key in ("angle", "distance", "impact", "torque"):
        set_parm(sop, [f"constraint_use{key}1"], 0)
    for name, value in {"constraint_useatframe1": 1, "constraint_atframe1": 5, "constraint_mode1": 1}.items():
        set_parm(sop, [name], value)

    def sample(s, f):
        if f != frames:
            return None
        con = s.geometry(1)
        prim = con.prims()[0] if con.prims() else None
        attrs = {a.name(): prim.attribValue(a.name()) for a in con.primAttribs()} if prim is not None else {}
        return attrs, named_positions(s.geometry(0)).get(pnames[1])

    return play(sop, frames, sample)[frames]


def test_solver_sop_break_next_settings():
    """The Solver SOP's break wrangle (Break Thresholds, At Frame 5, Switch to Next Constraint) gives a switching
    constraint its next settings: a Weld with next_constraint_name Spherical, next_enable_cone_limit 1 and
    next_cone_angle 0.25 is a Spherical line with enable_cone_limit 1 and cone_angle 0.25 once it has switched, and
    the next attributes are "not set" again."""
    attrs, _ = _break_to_next("sop_next_settings", "Weld", "Weld",
                              {"next_constraint_name": "Spherical", "next_enable_cone_limit": 1,
                               "next_cone_angle": 0.25}, ((0, 3.5, 0), (0, 3.5, 0)), 8)
    ok = (attrs.get("constraint_name") == "Spherical" and attrs.get("enable_cone_limit") == 1
          and close(attrs.get("cone_angle", -1.0), 0.25, 1e-4) and attrs.get("next_enable_cone_limit", 0) < 0
          and attrs.get("next_cone_angle", 0.0) <= -3.0e38 and not attrs.get("next_constraint_name"))
    return [Result("solver_sop_break_next_settings", Result.PASS if ok else Result.FAIL,
                   f"line attributes on frame 8: { {k: attrs.get(k) for k in ('constraint_name', 'enable_cone_limit', 'cone_angle', 'next_enable_cone_limit', 'next_cone_angle', 'next_constraint_name')} } "
                   f"(expected Spherical, 1, 0.25 and the next ones cleared)")]


def test_solver_sop_next_constraint_type():
    """next_constraint_type reaches a SideFX-typed line. A SideFX Hard line holding a box by an anchor 1 beside its centre
    (constraint_type all: rigid) breaks at frame 5 and switches to Hard again. With next_constraint_type position it
    is made as a position-only joint, so the box swings about the anchor and ends further along x (1.7 against 1.0);
    with next_constraint_type all it stays rigid. The line has the next type afterwards (without one the type is cleared,
    which reads as position, as in Bullet)."""
    anchors = ((1, 3.5, 0), (1, 3.5, 0))
    base = {"constraint_type": "all", "next_constraint_name": "Hard"}
    swung_attrs, swung = _break_to_next("sop_next_type_a", "Hard", "Hard", dict(base, next_constraint_type="position"),
                                        anchors, 16)
    held_attrs, held = _break_to_next("sop_next_type_b", "Hard", "Hard", dict(base, next_constraint_type="all"), anchors,
                                      16)
    ok = (swung is not None and held is not None and swung[0] - held[0] > 0.3
          and swung_attrs.get("constraint_type") == "position" and not swung_attrs.get("next_constraint_type"))
    return [Result("solver_sop_next_constraint_type", Result.PASS if ok else Result.FAIL,
                   f"box centre x on frame 16 with next_constraint_type position {swung[0] if swung else None} "
                   f"(expected more than 0.3 above the held one: swung), with all {held[0] if held else None} (held); "
                   f"constraint_type after the switch {swung_attrs.get('constraint_type')!r}, "
                   f"next_constraint_type {swung_attrs.get('next_constraint_type')!r}")]


def test_solver_sop_break_distance_bullet():
    """Break Thresholds, Distance Threshold, against the RBD Bullet Solver SOP: a Hard constraint whose anchors start
    2 apart breaks within 8 frames with a threshold of 0.2 and holds for 8 frames with one of 50, in both: the
    threshold compares the anchors' distance, beyond the constraint's Rest Length."""
    problems, rows = [], []
    for kind in ("box3d", "bullet"):
        for thr, want_break in ((0.2, True), (50.0, False)):
            f = _first_break(kind, f"sop_distance_{kind}{int(thr * 10)}", "Hard", ((0, 3.5, 0), (0, 1.5, 0)),
                             enable=("distance",), parms={"constraint_distancethreshold1": thr}, names="Hard", frames=8)
            rows.append(f"{kind} {thr}: {f}")
            if (f is not None) != want_break:
                problems.append(f"{kind} with a threshold of {thr}: gone on {f}")
    return [Result("solver_sop_break_distance_bullet", Result.FAIL if problems else Result.PASS,
                   f"gone on frame: {', '.join(rows)}" + (f"; PROBLEMS: {'; '.join(problems)}" if problems else ""))]


def test_solver_sop_break_attrib_update_bullet():
    """Break Thresholds, Update from Input, against the RBD Bullet Solver SOP: a Hard constraint 2 apart with a
    Distance Threshold of 0.2 scaled by an attribute (`mystr`) that is 1000 until frame 5 and 1 from frame 6 on the
    Constraints input. With Update from Input off the value it had when the constraint was made counts, so it holds;
    on, the live value counts and it breaks soon after frame 6. The same in both."""
    problems, rows = [], []
    for update in (0, 1):
        got = {}
        for kind in ("box3d", "bullet"):
            got[kind] = _first_break(
                kind, f"sop_update_{kind}{update}", "Hard", ((0, 3.5, 0), (0, 1.5, 0)), enable=("distance",),
                parms={"constraint_distancethreshold1": 0.2, "constraint_scaledistance1": 1,
                       "constraint_distanceattrib1": "mystr", "constraint_distanceattribupdate1": update},
                names="Hard", line_attrs={"mystr": 1000.0}, frames=12, animate="f@mystr = @Frame < 6 ? 1000.0 : 1.0;")
        rows.append(f"Update from Input {update}: Box3D {got['box3d']}, Bullet {got['bullet']}")
        if update == 0 and (got["box3d"] is not None or got["bullet"] is not None):
            problems.append("a constraint broke with Update from Input off")
        if update == 1 and (got["box3d"] is None or got["bullet"] is None or abs(got["box3d"] - got["bullet"]) > 1
                            or got["box3d"] < 6):
            problems.append(f"with it on they went on {got['box3d']} / {got['bullet']}, expected the same frame from 6")
    return [Result("solver_sop_break_attrib_update_bullet", Result.FAIL if problems else Result.PASS,
                   "gone on frame: " + "; ".join(rows) + (f"; PROBLEMS: {'; '.join(problems)}" if problems else ""))]


def test_solver_sop_constraint_updates_bullet():
    """Enable Attribute Update from SOPs and Overwrite with SOP, against the RBD Bullet Solver SOP: the scene of
    solver_sop_break_attrib_update_bullet (a Distance Threshold of 0.2 scaled by `mystr`, 1000 until frame 5 and 1
    from frame 6 on the Constraints input, Update from Input off). Holds with neither; breaks soon after frame 6 with
    Enable Attribute Update from SOPs listing `mystr` or with Overwrite with SOP; holds again when the update's Group
    names no constraint. The same frame in both."""
    base = {"constraint_distancethreshold1": 0.2, "constraint_scaledistance1": 1, "constraint_distanceattrib1": "mystr",
            "constraint_distanceattribupdate1": 0}
    cases = (
        ("neither", {}, False),
        ("update listing mystr", {"enable_constraintupdates": 1, "constraint_attributes": "mystr"}, True),
        ("update listing mystr, group of none", {"enable_constraintupdates": 1, "constraint_attributes": "mystr",
                                                 "constraint_attribgroup": "@constraint_name=Nothing"}, False),
        ("update listing another attribute", {"enable_constraintupdates": 1, "constraint_attributes": "other"}, False),
    )
    problems, rows = [], []
    for i, (label, extra, breaks) in enumerate(cases):
        got = {}
        for kind in ("box3d", "bullet"):
            got[kind] = _first_break(
                kind, f"sop_cupd{i}_{kind}", "Hard", ((0, 3.5, 0), (0, 1.5, 0)), enable=("distance",),
                parms=dict(base, **extra), names="Hard", line_attrs={"mystr": 1000.0}, frames=12,
                animate="f@mystr = @Frame < 6 ? 1000.0 : 1.0;")
        rows.append(f"{label}: Box3D {got['box3d']}, Bullet {got['bullet']}")
        a, b = got["box3d"], got["bullet"]
        if (a is None) != (b is None) or (a is not None and abs(a - b) > 1) or (a is not None) != breaks:
            problems.append(f"{label}: {a} / {b}, expected {'a break from frame 6' if breaks else 'no break'} in both")
    # Overwrite with SOP: the constraints' own `mystr` on frame 9 (1000 when read once, 1 when read again every step)
    pieces, pnames = packed_boxes("sop_reload_pieces", [{"size": (1, 1, 1), "center": (0, 4, 0), "attrs": {"active": 0}},
                                                        {"size": (1, 1, 1), "center": (0, 2, 0), "attrs": {"active": 1}}])
    lines = constraint_lines("sop_reload_net", [(pnames[0], (0, 3.5, 0), pnames[1], (0, 1.5, 0), "Hard",
                                                 {"mystr": 1000.0})])
    lines = append_sop(lines, "attribwrangle", lambda n: (set_parm(n, ["class"], "primitive"), set_parm(
        n, ["snippet"], "f@mystr = @Frame < 6 ? 1000.0 : 1.0;")))
    for reload in (0, 1):
        seen = {}
        for kind in ("box3d", "bullet"):
            sop = (solver_sop if kind == "box3d" else bullet_sop)(f"sop_reload{reload}_{kind}", geometry=pieces,
                                                                  constraints=lines)
            set_parm(sop, ["enable_constraintbreaks"], 0)  # RBD Bullet Solver's is on by default
            set_parm(sop, ["constraint_reloadfromsops"], reload)
            seen[kind] = play(sop, 9, lambda s, f: [p.attribValue("mystr") for p in s.geometry(1).prims()])[9]
        rows.append(f"Overwrite with SOP {reload}: mystr on frame 9 Box3D {seen['box3d']}, Bullet {seen['bullet']}")
        if seen["box3d"] != seen["bullet"] or seen["box3d"] != [1.0 if reload else 1000.0]:
            problems.append(f"Overwrite with SOP {reload}: {seen['box3d']} / {seen['bullet']}")
    return [Result("solver_sop_constraint_updates_bullet", Result.FAIL if problems else Result.PASS,
                   "gone on frame: " + "; ".join(rows) + (f"; PROBLEMS: {'; '.join(problems)}" if problems else ""))]


def test_solver_sop_break_angle_torque_bullet():
    """INFO, Break Thresholds, Angle and Torque Threshold, against the RBD Bullet Solver SOP: a Soft constraint on a
    box spinning at 6 rad/s about z, thresholds from small to large. The frame it goes on in each is listed. They
    are not expected to agree: the two solvers' `angle` and `torque` readings differ (docs/notes.md)."""
    rows = []
    for key, values in (("angle", (0.01, 0.05, 0.2)), ("torque", (100.0, 2000.0, 20000.0))):
        for value in values:
            got = {}
            for kind in ("box3d", "bullet"):
                got[kind] = _first_break(kind, f"sop_{key}_{kind}{int(value * 100)}", "Soft",
                                         ((0, 3.5, 0), (0, 3.5, 0)), hang={"w": (0, 0, 6)}, enable=(key,),
                                         parms={f"constraint_{key}threshold1": value}, names="Soft", frames=16)
            rows.append(f"{key} {value}: {got['box3d']} / {got['bullet']}")
    return [Result("solver_sop_break_angle_torque_bullet", Result.INFO,
                   "gone on frame (Box3D / Bullet): " + "; ".join(rows))]


def test_solver_sop_constraints():
    """A box welded to the world on the Constraints input (constraint_name Weld) stays put for 1 s,
    and the constraints output carries the solver's feedback (a force primitive attribute)."""
    pieces, names = packed_boxes("sop_weld_pieces", [{"size": (1, 1, 1), "center": (0, 3, 0)}])
    lines = constraint_lines("sop_weld_network", [("", (0, 3, 0), names[0], (0, 3, 0), "Weld")])
    sop = solver_sop("sop_weld", geometry=pieces, constraints=lines)

    def sample(s, f):
        if f != 24:
            return None
        con = s.geometry(1)
        return named_positions(s.geometry(0)).get(names[0]), con.findPrimAttrib("force") is not None, len(con.prims())

    pos, has_force, nprims = play(sop, 24, sample)[24]
    ok = pos is not None and close(pos[1], 3.0, 1e-3) and has_force and nprims == 1
    return [Result("solver_sop_constraints", Result.PASS if ok else Result.FAIL,
                   f"welded box y={pos[1] if pos else None} after 1 s (expected 3.0); constraints output: "
                   f"{nprims} primitive(s), force attribute {'present' if has_force else 'missing'}")]


def test_solver_sop_constraint_guide_radius():
    """Guide Radius and Guide Scale (the constraint lines in the viewport): a Weld line shown with Show
    Constraints has a radius of Guide Radius times its type's Guide Scale (0.05 by default; 0.15 with the Weld's
    Guide Scale 3; 0.1 with Guide Radius 0.1), on the line's points before the wire is made, and the wire in the
    asset's Guide Geometry is 0.2 thicker with Guide Scale 3 (the radius grows 0.1, twice that across)."""
    pieces, names = packed_boxes("sop_guide_radius_pieces", [{"size": (1, 1, 1), "center": (0, 3, 0)}])
    lines = constraint_lines("sop_guide_radius_network", [("", (3, 3, 0), names[0], (0, 3, 0), "Weld")])
    results = []
    thickness = {}
    for label, parms, want in (("defaults", {}, 0.05),
                               ("Weld Guide Scale 3", {"weld_guidescale": 3.0}, 0.15),
                               ("Guide Radius 0.1", {"showconstraintguidesradius": 0.1}, 0.1)):
        sop = solver_sop("sop_guide_radius_" + label.split()[0].lower(), geometry=pieces, constraints=lines)
        set_parm(sop, ["showconstraints"], 1)
        set_parm(sop, ["showconstraintguides"], 1)
        for name, value in parms.items():
            set_parm(sop, [name], value)

        def sample(s, f):
            if f != 3:
                return None
            node = s.node("constraint_guide_radius")
            geo = node.geometry() if node else None
            attr = geo.findPointAttrib("guide_radius") if geo else None
            radius = geo.points()[0].attribValue("guide_radius") if attr and geo.points() else None
            box = s.node("GUIDE_GEO").geometry().boundingBox()
            return radius, box.sizevec()[1]

        radius, height = play(sop, 3, sample)[3]
        thickness[label] = height
        ok = radius is not None and close(radius, want, 1e-4) and height > 0
        results.append(Result("solver_sop_constraint_guide_radius", Result.PASS if ok else Result.FAIL,
                              f"{label}: guide_radius on the line's points {radius} (expected {want}); wire thickness "
                              f"{height:.4f}"))
    # The wire's height is its diameter plus a little the polywire adds, so the changes are compared.
    grew = thickness["Weld Guide Scale 3"] - thickness["defaults"]
    results.append(Result("solver_sop_constraint_guide_radius", Result.PASS if close(grew, 0.2, 0.02) else Result.FAIL,
                          f"wire thickness grew {grew:.3f} with Weld Guide Scale 3 (expected 0.2: the radius grows "
                          f"from 0.05 to 0.15, twice that across)"))
    # Show Guides off: the line is drawn plain (a horizontal line has no height), with no wire made.
    sop = solver_sop("sop_guide_radius_plain", geometry=pieces, constraints=lines)
    set_parm(sop, ["showconstraints"], 1)
    plain = play(sop, 3, lambda s, f: s.node("GUIDE_GEO").geometry().boundingBox().sizevec()[1] if f == 3 else None)[3]
    results.append(Result("solver_sop_constraint_guide_radius", Result.PASS if plain < 0.01 else Result.FAIL,
                          f"Show Guides off: the line's height is {plain:.4f} (expected 0: a plain line, not a wire)"))
    return results


TESTS = {
    "solver_sop_keep_broken_bullet": test_solver_sop_keep_broken_bullet,
    "solver_sop_break_impact_bullet": test_solver_sop_break_impact_bullet,
    "solver_sop_break_atframe_bullet": test_solver_sop_break_atframe_bullet,
    "solver_sop_break_distance_bullet": test_solver_sop_break_distance_bullet,
    "solver_sop_break_angle_torque_bullet": test_solver_sop_break_angle_torque_bullet,
    "solver_sop_break_attrib_update_bullet": test_solver_sop_break_attrib_update_bullet,
    "solver_sop_constraint_updates_bullet": test_solver_sop_constraint_updates_bullet,
    "solver_sop_keep_broken": test_solver_sop_keep_broken,
    "solver_sop_constraint_settings": test_solver_sop_constraint_settings,
    "solver_sop_constraint_guide_radius": test_solver_sop_constraint_guide_radius,
    "solver_sop_break_next_settings": test_solver_sop_break_next_settings,
    "solver_sop_next_constraint_type": test_solver_sop_next_constraint_type,
    "solver_sop_active_age_bullet": test_solver_sop_active_age_bullet,
    "solver_sop_aging_bullet": test_solver_sop_aging_bullet,
    "solver_sop_constraint_aging_bullet": test_solver_sop_constraint_aging_bullet,
    "solver_sop_constraints": test_solver_sop_constraints,
}
