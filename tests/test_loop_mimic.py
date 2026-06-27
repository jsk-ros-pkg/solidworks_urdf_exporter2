"""A closed-loop (four-bar) linkage is exported as driver + mimic followers.

A SolidWorks closed linkage is fully constrained -- four revolute hinges about
parallel axes around one loop, so only ONE joint is really free.  The spanning
tree keeps three hinges as revolute and drops the fourth as a loop closure;
without coupling the three move independently and the linkage flies apart.
``_auto_loop_mimic`` detects the four-bar and makes the two passive tree hinges
``<mimic>`` the driver.  For a parallelogram the coupling is exactly +-1, which
these tests pin so the detection (and multiplier maths) cannot silently regress.
"""
import numpy as np

from sw2robot.exporter.model import build_model
from sw2robot.exporter.state import ComponentState, GraphState, MateEdge, MateGeo

CYL, PLANE = 4, 3


def _comp(name, xyz=(0, 0, 0), fixed=False):
    w = np.eye(4)
    w[:3, 3] = xyz
    return ComponentState(
        name=name, link_name=name.replace(" ", "_"), part_path=None,
        is_subassembly=False, world=[float(x) for x in w.flatten()],
        fixed=fixed)


def _geo(mtype, ents):
    return MateGeo(type=mtype,
                   etypes=[e[0] for e in ents],
                   points=[list(map(float, e[1])) for e in ents],
                   dirs=[list(map(float, e[2])) for e in ents],
                   radii=[None] * len(ents))


def _hinge_z(a, b, p):
    """A pure revolute about world +Z at point ``p``: a concentric Z cylinder
    (rotation + axial slide) pinned axially by a coincident Z-normal plane."""
    p = list(map(float, p))
    return MateEdge(a=a, b=b, types=["CONCENTRIC", "COINCIDENT"],
                    axis_point=p, axis_dir=[0.0, 0.0, 1.0],
                    mates=[_geo("CONCENTRIC", [(CYL, p, [0, 0, 1]),
                                               (CYL, p, [0, 0, 1])]),
                           _geo("COINCIDENT", [(PLANE, p, [0, 0, 1]),
                                               (PLANE, p, [0, 0, 1])])])


def _graph(comps, edges, ground):
    return GraphState(robot_name="t", source_assembly="t.SLDASM",
                      components=comps, edges=edges, ground=ground)


def _parallelogram():
    # ground bar A(0,0)->D(2,0); equal input A->B(0,1) and output D->C(2,1);
    # coupler B->C has the same length as the ground bar -> a parallelogram.
    base = _comp("base", fixed=True)
    inl = _comp("inlink", (0, 1, 0))
    cpl = _comp("coupler", (1, 1, 0))
    outl = _comp("outlink", (2, 1, 0))
    edges = [
        _hinge_z("base", "inlink", [0, 0, 0]),       # A
        _hinge_z("inlink", "coupler", [0, 1, 0]),    # B
        _hinge_z("coupler", "outlink", [2, 1, 0]),   # C
        _hinge_z("outlink", "base", [2, 0, 0]),      # D
    ]
    return _graph([base, inl, cpl, outl], edges, ground=["base"])


def _crank_rocker():
    # a NON-parallelogram four-bar (unequal bars) -> a triple rocker that BINDS
    # at a toggle within its travel, the case where a naive IK flips assembly
    # branch.  Pivots A,B,C,D (m).
    A, B, C, D = (0, 0, 0), (0.4, 0.4, 0), (1.2, 0.7, 0), (1.5, 0, 0)
    base = _comp("base", fixed=True)
    inl = _comp("inlink", A)
    cpl = _comp("coupler", B)
    outl = _comp("outlink", C)
    edges = [
        _hinge_z("base", "inlink", A),
        _hinge_z("inlink", "coupler", B),
        _hinge_z("coupler", "outlink", C),
        _hinge_z("outlink", "base", D),
    ]
    return _graph([base, inl, cpl, outl], edges, ground=["base"])


def test_four_bar_loop_becomes_driver_plus_two_mimics():
    model = build_model(_parallelogram())
    rev = [j for j in model.joints if j.jtype == "revolute"]
    # 4 physical hinges, one dropped as the loop closure -> 3 revolute tree edges
    assert len(rev) == 3
    mimics = [j for j in rev if j.mimic]
    free = [j for j in rev if not j.mimic]
    assert len(mimics) == 2 and len(free) == 1      # one driver, two followers
    driver = free[0]
    # both followers mimic the SAME (driver) joint, offset 0
    assert all(j.mimic["joint"] == driver.name for j in mimics)
    assert all(j.mimic["offset"] == 0.0 for j in mimics)
    # a parallelogram couples exactly +-1
    for j in mimics:
        assert abs(abs(j.mimic["multiplier"]) - 1.0) < 1e-3


def test_loop_mimic_round_trips_through_joints_yaml(tmp_path):
    # The editor rebuilds via build(--config joints.yaml), which takes the
    # DIRECTED branch and does NOT re-run auto loop detection -- so the mimic
    # must be persisted in the template and read back, or it (and the viewer's
    # purple axis) vanishes on the next build.
    import yaml

    from sw2robot.exporter import jointcfg
    from sw2robot.exporter.model import build_model

    graph = _parallelogram()
    model = build_model(graph)
    follower = next(j for j in model.joints if j.mimic)

    tmpl = tmp_path / "j.yaml"
    jointcfg.write_template(model, str(tmpl))
    cfg = yaml.safe_load(tmpl.read_text(encoding="utf-8"))
    entry = next(j for j in cfg["joints"]
                 if j.get("mimic", {}).get("joint") == follower.mimic["joint"])
    assert entry["mimic"]["joint"] == follower.mimic["joint"]
    assert "poly" in entry["mimic"] and len(entry["mimic"]["poly"]) >= 2

    # feeding that config back keeps the <mimic> (directed path, no auto detect)
    model2 = build_model(graph, config=cfg)
    f2 = next((j for j in model2.joints
               if j.name == follower.name), None)
    assert f2 is not None and f2.mimic is not None
    assert f2.mimic["joint"] == follower.mimic["joint"]


def test_loop_closures_exported_for_runtime_ik():
    # the model carries general loop-closure data for the runtime-IK relay:
    # the cut hinge (two links + base-frame point/axis) and which joints are
    # driven (independent) vs solved (dependent)
    model = build_model(_parallelogram())
    lc = model.loop_closures
    assert lc is not None
    assert len(lc["closures"]) == 1
    c = lc["closures"][0]
    assert {c["link_a"], c["link_b"]} <= {"base_link", "inlink", "coupler",
                                          "outlink"}
    assert len(c["point"]) == 3 and len(c["axis"]) == 3
    # one driver, the rest solved (a 1-DOF four-bar)
    assert len(lc["independent"]) == 1 and len(lc["dependent"]) == 2
    # dependent = the <mimic> followers; independent = the driver
    mimics = {j.name for j in model.joints if j.mimic}
    assert set(lc["dependent"]) == mimics
    assert lc["independent"][0] not in mimics


def _fk_chain(urdf_path):
    """Tiny URDF FK (the same maths the shipped relay embeds) for the test."""
    import xml.etree.ElementTree as ET

    from sw2robot.exporter.geometry import matrix_from_rpy
    root = ET.parse(urdf_path).getroot()
    js = {}
    for j in root.findall("joint"):
        o = j.find("origin")
        xyz = ([float(x) for x in o.get("xyz", "0 0 0").split()]
               if o is not None else [0, 0, 0])
        rpy = ([float(x) for x in o.get("rpy", "0 0 0").split()]
               if o is not None else [0, 0, 0])
        ax = j.find("axis")
        axis = (np.array([float(x) for x in ax.get("xyz").split()])
                if ax is not None else np.array([0.0, 0.0, 1.0]))
        t = matrix_from_rpy(rpy).copy()
        t[:3, 3] = xyz
        js[j.get("name")] = (j.get("type"), j.find("child").get("link"),
                             j.find("parent").get("link"), t, axis)
    cj = {v[1]: n for n, v in js.items()}

    def rot(a, q):
        a = a / (np.linalg.norm(a) or 1.0)
        x, y, z = a
        c, s = np.cos(q), np.sin(q)
        cc = 1 - c
        return np.array([[c+x*x*cc, x*y*cc-z*s, x*z*cc+y*s, 0],
                         [y*x*cc+z*s, c+y*y*cc, y*z*cc-x*s, 0],
                         [z*x*cc-y*s, z*y*cc+x*s, c+z*z*cc, 0], [0, 0, 0, 1.0]])

    def world(link, Q):
        chain, ln = [], link
        while ln in cj:
            n = cj[ln]
            chain.append(n)
            ln = js[n][2]
        m = np.eye(4)
        for n in reversed(chain):
            typ, _c, _p, t, axis = js[n]
            m = m @ (t @ rot(axis, Q.get(n, 0.0)) if typ in
                     ("revolute", "continuous") else t)
        return m
    return world


def test_runtime_ik_relay_closes_the_loop(tmp_path):
    # End-to-end: build the parallelogram, write its URDF, and run the relay's
    # loop-closure IK (pure-numpy FK, no extra deps) -- the cut hinge must
    # coincide and the parallelogram coupling must come out +-1.
    from sw2robot.exporter.urdf_writer import write_urdf
    model = build_model(_parallelogram())
    lc = model.loop_closures
    urdf = tmp_path / "p.urdf"
    write_urdf(model, str(urdf))
    world = _fk_chain(str(urdf))

    c = lc["closures"][0]
    la, lb = c["link_a"], c["link_b"]
    p = np.asarray(c["point"], float)
    a = np.asarray(c["axis"], float)
    a = a / np.linalg.norm(a)
    t0a, t0b = world(la, {}), world(lb, {})
    wit = []
    for q in (p, p + a * 0.03):
        wit.append((la, (np.linalg.inv(t0a) @ np.append(q, 1.0))[:3],
                    lb, (np.linalg.inv(t0b) @ np.append(q, 1.0))[:3]))

    def resid(Q):
        return np.concatenate([(world(L, Q) @ np.append(ll, 1.0))[:3]
                               - (world(M, Q) @ np.append(ml, 1.0))[:3]
                               for L, ll, M, ml in wit])

    deps = lc["dependent"]
    drv = lc["independent"][0]

    def solve(qd, warm):
        Q = {drv: qd}
        x = warm.copy()
        for _ in range(40):
            for i, dn in enumerate(deps):
                Q[dn] = x[i]
            r = resid(Q)
            if np.linalg.norm(r) < 1e-10:
                break
            jac = np.zeros((len(r), len(deps)))
            for i, dn in enumerate(deps):
                Q2 = dict(Q)
                Q2[dn] = x[i] + 1e-7
                jac[:, i] = (resid(Q2) - r) / 1e-7
            x = x + np.linalg.lstsq(jac, -r, rcond=None)[0]
        return x

    x = solve(np.radians(5.0), np.zeros(len(deps)))
    Q = {drv: np.radians(5.0)}
    Q.update({deps[i]: x[i] for i in range(len(deps))})
    assert np.linalg.norm(resid(Q)) < 1e-6         # loop closed
    for v in x:
        assert abs(abs(v) - np.radians(5.0)) < 1e-3   # parallelogram = +-1


def test_runtime_ik_relay_holds_at_toggle_no_branch_flip(tmp_path):
    # A non-parallelogram four-bar BINDS at a toggle.  Driving the driver out
    # PAST the toggle and back must NOT flip the assembly branch / spin the
    # dependent joints (the bug): the relay's damped + step-clamped solve holds
    # the last valid pose past the toggle and tracks back to zero.
    from sw2robot.exporter.urdf_writer import write_urdf
    model = build_model(_crank_rocker())
    lc = model.loop_closures
    urdf = tmp_path / "cr.urdf"
    write_urdf(model, str(urdf))
    world = _fk_chain(str(urdf))
    c = lc["closures"][0]
    la, lb = c["link_a"], c["link_b"]
    p = np.asarray(c["point"], float)
    a = np.asarray(c["axis"], float) / np.linalg.norm(c["axis"])
    t0a, t0b = world(la, {}), world(lb, {})
    wit = [(la, (np.linalg.inv(t0a) @ np.append(q, 1.0))[:3],
            lb, (np.linalg.inv(t0b) @ np.append(q, 1.0))[:3])
           for q in (p, p + a * 0.03)]

    def resid(Q):
        return np.concatenate([(world(L, Q) @ np.append(ll, 1.0))[:3]
                               - (world(M, Q) @ np.append(ml, 1.0))[:3]
                               for L, ll, M, ml in wit])

    deps = lc["dependent"]
    drv = lc["independent"][0]

    def solve(qd, warm):                       # == the shipped relay's _solve
        x0 = warm.copy()
        x = x0.copy()
        nd = len(deps)
        Q = {drv: qd}
        for _ in range(50):
            for i, dn in enumerate(deps):
                Q[dn] = x[i]
            r = resid(Q)
            if np.linalg.norm(r) < 1e-10:
                break
            jac = np.zeros((len(r), nd))
            for i, dn in enumerate(deps):
                Q2 = dict(Q)
                Q2[dn] = x[i] + 1e-7
                jac[:, i] = (resid(Q2) - r) / 1e-7
            dx = np.linalg.solve(jac.T @ jac + 1e-6 * np.eye(nd), -jac.T @ r)
            s = float(np.linalg.norm(dx))
            if s > 0.15:
                dx *= 0.15 / s
            x = x + dx
        for i, dn in enumerate(deps):
            Q[dn] = x[i]
        if np.linalg.norm(resid(Q)) > 1e-5:    # can't close -> hold last valid
            return x0
        return x

    # drive in BIG jumps per callback (a fast slider drag / clicking a far
    # value) -- the case that flips: each callback sub-steps the driver like the
    # relay's _cb, out well past the toggle and back to zero
    state = {"warm": np.zeros(len(deps)), "prev": None}

    def step(qd):
        prev = state["prev"] if state["prev"] is not None else qd
        nsub = max(1, int(abs(qd - prev) / 0.05))
        for k in range(1, nsub + 1):
            state["warm"] = solve(prev + (qd - prev) * k / nsub, state["warm"])
        state["prev"] = qd
        return state["warm"]

    peak = 0.0
    for qd in (0.0, 1.4, -1.4, 1.4, 0.7, 0.0):     # big jumps incl past toggle
        peak = max(peak, float(np.max(np.abs(step(qd)))))
    # never wrapped/spun (a flip sends deps to many radians), and it came back
    assert peak < 2.5, f"dependent joints spun ({peak} rad) -- branch flip"
    assert np.max(np.abs(state["warm"])) < 1e-3, \
        "did not return to home after the toggle"


def test_four_bar_driver_carries_no_mimic_and_loop_is_open():
    model = build_model(_parallelogram())
    # exactly one revolute hinge is dropped (the URDF tree stays acyclic): 4
    # links, and link count - 1 == joint count for a tree
    assert len(model.joints) == len(model.components) - 1
    # the driver is a real, independent joint (no <mimic> on it)
    rev = [j for j in model.joints if j.jtype == "revolute"]
    driver = next(j for j in rev if not j.mimic)
    assert driver.mimic is None


# --- flipping a joint's axis must keep the mimic coupling consistent ---------
_MIMIC_URDF = '''<?xml version="1.0"?>
<robot name="fb">
  <link name="base_link"/>
  <link name="L1"/><link name="L2"/><link name="L3"/>
  <joint name="drv" type="revolute">
    <parent link="base_link"/><child link="L1"/>
    <axis xyz="0 0 1"/>
    <limit lower="-0.3" upper="0.45" effort="0" velocity="0"/>
  </joint>
  <joint name="f1" type="revolute">
    <parent link="L1"/><child link="L2"/>
    <axis xyz="0 0 1"/>
    <limit lower="-0.85" upper="0.88" effort="0" velocity="0"/>
    <mimic joint="drv" multiplier="1.35269" offset="0.1"/>
  </joint>
  <joint name="f2" type="revolute">
    <parent link="L1"/><child link="L3"/>
    <axis xyz="0 0 1"/>
    <limit lower="-0.3" upper="0.45" effort="0" velocity="0"/>
    <mimic joint="drv" multiplier="0.581031" offset="0"/>
  </joint>
</robot>
'''


def _mults(txt):
    import re
    return {m.group(1): float(m.group(2)) for m in re.finditer(
        r'<joint name="(f\d)".*?multiplier="([^"]*)".*?offset="([^"]*)"',
        txt, re.DOTALL)}


def _offsets(txt):
    import re
    return {m.group(1): float(m.group(2)) for m in re.finditer(
        r'<joint name="(f\d)".*?offset="([^"]*)"', txt, re.DOTALL)}


def _write(tmp_path):
    (tmp_path / "urdf").mkdir(exist_ok=True)
    (tmp_path / "urdf" / "fb.urdf").write_text(_MIMIC_URDF, encoding="utf-8")
    return str(tmp_path)


def test_flip_driver_negates_follower_multipliers(tmp_path):
    """Flipping the DRIVER (master) joint must negate every follower's mimic
    multiplier (offset unchanged), or the coupled links move the wrong way."""
    from sw2robot.editor.webserver import _flip_axis_in_urdf
    pkg = _write(tmp_path)
    n = _flip_axis_in_urdf(pkg, "urdf/fb.urdf", "drv")
    assert n == 1
    txt = (tmp_path / "urdf" / "fb.urdf").read_text(encoding="utf-8")
    # driver axis negated, limits swapped+negated
    import re
    drv = re.search(r'<joint name="drv".*?</joint>', txt, re.DOTALL).group(0)
    assert 'xyz="0 0 -1"' in drv
    assert 'lower="-0.45"' in drv and 'upper="0.3"' in drv
    # followers: multiplier negated, offset kept
    assert _mults(txt) == {"f1": -1.35269, "f2": -0.581031}
    assert _offsets(txt) == {"f1": 0.1, "f2": 0.0}


def test_flip_driver_is_self_inverse(tmp_path):
    from sw2robot.editor.webserver import _flip_axis_in_urdf
    pkg = _write(tmp_path)
    _flip_axis_in_urdf(pkg, "urdf/fb.urdf", "drv")
    _flip_axis_in_urdf(pkg, "urdf/fb.urdf", "drv")
    after = (tmp_path / "urdf" / "fb.urdf").read_text(encoding="utf-8")
    assert _mults(after) == {"f1": 1.35269, "f2": 0.581031}
    assert _offsets(after) == {"f1": 0.1, "f2": 0.0}


def test_flip_follower_negates_its_own_mimic(tmp_path):
    """Flipping a FOLLOWER negates its OWN multiplier AND offset (its axis
    reversed); the driver and the other follower are untouched."""
    from sw2robot.editor.webserver import _flip_axis_in_urdf
    pkg = _write(tmp_path)
    _flip_axis_in_urdf(pkg, "urdf/fb.urdf", "f1")
    txt = (tmp_path / "urdf" / "fb.urdf").read_text(encoding="utf-8")
    assert _mults(txt) == {"f1": -1.35269, "f2": 0.581031}   # f2 untouched
    assert _offsets(txt) == {"f1": -0.1, "f2": 0.0}          # f1 offset negated
