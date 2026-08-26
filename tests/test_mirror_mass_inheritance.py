"""A SolidWorks mirror copy loses its source's per-body materials and its
Override Mass Properties value, so it silently weighs whatever the 1000 kg/m^3
default makes it.  These cover the recovery: the reflection maths, the
geometry-solved mirror plane, the "only when it was never set" predicate, and
the sub-assembly whose total is wrong because one child inside it is a copy.

The shape of the end-to-end case is the one seen on a real robot: a shell part
that reads 38.188 kg where its source reads 0.245, sitting inside a
sub-assembly that reads 38.480 where the source side reads 0.538.
"""

import numpy as np
import pytest

from sw2robot.exporter.mirror import (
    _recompose,
    inheritance_reason,
    reflect_consistently,
    reflect_inertial,
    solve_reflections,
)

# ---------------------------------------------------------------- reflection

def test_reflect_inertial_flips_only_the_products_touching_the_axis():
    mass, com = 3.0, [0.1, -0.2, 0.3]
    inertia = (1.0, 0.01, 0.02, 2.0, 0.03, 3.0)   # ixx ixy ixz iyy iyz izz
    m, c, i = reflect_inertial(mass, com, inertia, (-1, 1, 1))
    assert m == 3.0
    assert np.allclose(c, [-0.1, -0.2, 0.3])
    # x is mirrored -> ixy and ixz change sign, iyz does not; moments unchanged
    assert np.allclose(i, (1.0, -0.01, -0.02, 2.0, 0.03, 3.0))


def test_reflect_inertial_is_its_own_inverse():
    mass, com = 1.7, [0.4, 0.5, -0.6]
    inertia = (1.0, 0.1, 0.2, 2.0, 0.3, 3.0)
    once = reflect_inertial(mass, com, inertia, (1, 1, -1))
    twice = reflect_inertial(once[0], once[1], once[2], (1, 1, -1))
    assert np.allclose(twice[1], com)
    assert np.allclose(twice[2], inertia)


# ---------------------------------------------------------------- plane solve

def _unit_props(volume, centroid, inertia):
    """A :func:`unit_density_props` result, without SolidWorks."""
    return (volume, list(centroid), tuple(inertia))


def test_solve_reflections_finds_the_mirrored_axis():
    src = _unit_props(0.002, [0.05, -0.01, 0.2], (1.0, 0.1, 0.2, 2.0, 0.3, 3.0))
    for signs in ((-1, 1, 1), (1, -1, 1), (1, 1, -1)):
        _m, com, inertia = reflect_inertial(src[0], src[1], src[2], signs)
        assert solve_reflections(
            src, _unit_props(src[0], com, inertia)) == (signs,)


def test_solve_reflections_refuses_a_different_part():
    src = _unit_props(0.002, [0.05, -0.01, 0.2], (1.0, 0.1, 0.2, 2.0, 0.3, 3.0))
    # same shape, 3% more volume -- a revision, not a mirror
    bigger = _unit_props(0.00206, [-0.05, -0.01, 0.2],
                         (1.0, -0.1, -0.2, 2.0, 0.3, 3.0))
    assert solve_reflections(src, bigger) == ()


def test_solve_reflections_refuses_a_rotation():
    """A 90-degree rotation about z keeps the volume and the centroid norm but
    is not a reflection; accepting it would flip an inertia tensor that was
    never mirrored."""
    src = _unit_props(0.002, [0.05, -0.01, 0.2], (1.0, 0.1, 0.2, 2.0, 0.3, 3.0))
    rotated = _unit_props(0.002, [0.01, 0.05, 0.2],
                          (2.0, -0.1, -0.3, 1.0, 0.2, 3.0))
    assert solve_reflections(src, rotated) == ()


def test_a_doubly_symmetric_shape_fits_two_planes():
    """Centroid on x=0 and y=0 with only ixy off-diagonal: flipping x and
    flipping y explain the copy equally well.  The shape genuinely does not
    say which, so both come back."""
    src = _unit_props(0.002, [0.0, 0.0, 0.2], (1.0, 0.05, 0.0, 2.0, 0.0, 3.0))
    _m, com, inertia = reflect_inertial(src[0], src[1], src[2], (1, -1, 1))
    assert set(solve_reflections(src, _unit_props(src[0], com, inertia))) == {
        (-1, 1, 1), (1, -1, 1)}


def test_reflect_consistently_accepts_an_ambiguity_that_changes_nothing():
    """Two candidate planes, but the mass sits symmetrically enough that both
    give the same inertial -- so the ambiguity is not one."""
    out = reflect_consistently([(-1, 1, 1), (1, -1, 1)],
                               2.0, [0.0, 0.0, 0.3],
                               (1.0, 0.05, 0.0, 2.0, 0.0, 3.0))
    assert out is not None
    assert np.allclose(out[2], (1.0, -0.05, 0.0, 2.0, 0.0, 3.0))


def test_reflect_consistently_refuses_when_the_planes_disagree():
    """The shape is ambiguous but the MASS is not symmetric: flipping x and
    flipping y put ixz and iyz on different sides, so no answer is honest."""
    assert reflect_consistently([(-1, 1, 1), (1, -1, 1)],
                                2.0, [0.0, 0.0, 0.3],
                                (1.0, 0.05, 0.02, 2.0, 0.04, 3.0)) is None


# ---------------------------------------------------------------- the predicate

def _props(material=None, density=None, mass=1.0, overridden=False):
    return {"material": material, "density": density, "mass": mass,
            "overridden": overridden}


def test_inherit_when_the_copy_has_no_material():
    reason = inheritance_reason(
        _props(material=None, density=1000.0, mass=38.188),
        _props(material=None, density=6.42, mass=0.245))
    assert reason and "no material" in reason


def test_inherit_when_the_source_carries_a_mass_override():
    """A sheet cover: the copy HAS a material and is still several times too
    heavy, because the source's weight is a hand-entered override."""
    reason = inheritance_reason(
        _props(material="alloy A", density=2680.0, mass=6.697),
        _props(material="alloy A", density=400.21, mass=1.0, overridden=True))
    assert reason and "Override Mass Properties" in reason


def test_inherit_when_the_same_material_resolves_to_a_different_density():
    """Per-body materials: both sides name the same alloy, only the source's
    bodies differ, so its resolved density is lower."""
    reason = inheritance_reason(
        _props(material="alloy B", density=2810.0, mass=2.582),
        _props(material="alloy B", density=2202.01, mass=2.023))
    assert reason and "per-body materials" in reason


def test_leave_a_copy_that_names_a_different_material_alone():
    """"only if it was not set" -- a copy carrying its own material made a
    choice, even a stale one, and is never overwritten."""
    assert inheritance_reason(
        _props(material="alloy A", density=2680.0, mass=0.080),
        _props(material="alloy B", density=2810.0, mass=0.084)) is None


def test_leave_a_copy_with_its_own_solidworks_override_alone():
    assert inheritance_reason(
        _props(material="alloy A", density=2680.0, mass=2.0, overridden=True),
        _props(material="alloy A", density=400.0, mass=1.0,
               overridden=True)) is None


def test_no_source_mass_means_nothing_to_inherit():
    assert inheritance_reason(_props(material=None, density=1000.0),
                              _props(mass=None)) is None


# ---------------------------------------------------------------- sub-assembly

def _identity():
    return np.eye(4)


def _shift(x, y, z):
    T = np.eye(4)
    T[:3, 3] = (x, y, z)
    return T


def test_recompose_swaps_one_child_out_of_an_assembly_total():
    """The sub-assembly reads 38.480 kg because the mirrored part inside it
    reads 38.188 instead of 0.245.  Swapping the contribution must land on the
    source side's 0.538 kg."""
    wrong = (38.188, [0.0, 0.0, 0.05], (1.0, 0.0, 0.0, 1.0, 0.0, 1.0))
    right = (0.245, [0.0, 0.0, 0.05], (0.01, 0.0, 0.0, 0.01, 0.0, 0.01))
    total = (38.480, [0.0, 0.0, 0.05], (1.02, 0.0, 0.0, 1.02, 0.0, 1.02))
    fixed = _recompose(total, [(_identity(), wrong, right)])
    assert fixed is not None
    assert fixed[0] == pytest.approx(38.480 - 38.188 + 0.245, abs=1e-9)


def test_recompose_moves_the_centre_of_mass_off_the_heavy_child():
    """The wrong child dominated the assembly's centre of mass; once its weight
    drops by 150x, the light child it was hiding has to take over."""
    heavy = (38.0, [0.0, 0.0, 0.0], (1.0, 0.0, 0.0, 1.0, 0.0, 1.0))
    light = (0.25, [0.0, 0.0, 0.0], (0.01, 0.0, 0.0, 0.01, 0.0, 0.01))
    # assembly = the mirrored child at the origin + 2 kg sitting at x = 1 m
    total_mass = 38.0 + 2.0
    total_com = [2.0 * 1.0 / total_mass, 0.0, 0.0]
    other_i = 2.0 * (1.0 - total_com[0]) ** 2
    ixx = 1.0 + 38.0 * 0.0
    total_i = (ixx + 0.0,
               0.0, 0.0,
               1.0 + 38.0 * total_com[0] ** 2 + other_i,
               0.0,
               1.0 + 38.0 * total_com[0] ** 2 + other_i)
    fixed = _recompose((total_mass, total_com, total_i),
                       [(_identity(), heavy, light)])
    assert fixed[0] == pytest.approx(2.25)
    # the 2 kg at x=1 now dominates: com moves from 0.05 m out to ~0.89 m
    assert fixed[1][0] == pytest.approx(2.0 / 2.25, abs=1e-9)


def test_recompose_respects_the_child_transform():
    """A child mounted away from the assembly origin contributes through the
    parallel-axis theorem, so the swap has to be done in the assembly frame."""
    wrong = (10.0, [0.0, 0.0, 0.0], (0.1, 0.0, 0.0, 0.1, 0.0, 0.1))
    right = (1.0, [0.0, 0.0, 0.0], (0.01, 0.0, 0.0, 0.01, 0.0, 0.01))
    T = _shift(0.3, 0.0, 0.0)
    # an assembly holding just that child, 0.3 m out along x
    total = (10.0, [0.3, 0.0, 0.0], (0.1, 0.0, 0.0, 0.1, 0.0, 0.1))
    fixed = _recompose(total, [(T, wrong, right)])
    assert fixed[0] == pytest.approx(1.0)
    assert np.allclose(fixed[1], [0.3, 0.0, 0.0])
    # iyy about the new COM is the child's own, NOT inflated by the 0.3 m arm
    assert fixed[2][3] == pytest.approx(0.01, abs=1e-9)


def test_recompose_refuses_to_produce_a_negative_mass():
    wrong = (50.0, [0.0, 0.0, 0.0], (1.0, 0.0, 0.0, 1.0, 0.0, 1.0))
    right = (1.0, [0.0, 0.0, 0.0], (0.01, 0.0, 0.0, 0.01, 0.0, 0.01))
    total = (2.0, [0.0, 0.0, 0.0], (0.1, 0.0, 0.0, 0.1, 0.0, 0.1))
    assert _recompose(total, [(_identity(), wrong, right)]) is None


# ---------------------------------------------------------------- provenance

def test_inherited_mass_survives_the_graph_round_trip():
    """The correction runs at extract time, so it has to reach graph.json and
    come back -- every later build reads that file with no SolidWorks."""
    from sw2robot.exporter.model import Component, _component_states, from_graph
    from sw2robot.exporter.state import GraphState

    c = Component(name="Part-1", link_name="shell_link", part_path="m.SLDPRT",
                  is_subassembly=False, world=np.eye(4), fixed=True, dof=0,
                  sw_mass=0.245, sw_com=[0.0, 0.0, 0.05],
                  sw_inertia=[0.01, 0.0, 0.0, 0.01, 0.0, 0.01],
                  mass_inherited_from="shell.SLDPRT")
    graph = GraphState(robot_name="r", source_assembly="a.SLDASM",
                       components=_component_states([c]), edges=[], ground=[])
    back, _adjacency, _ground = from_graph(graph)
    assert back[0].mass_inherited_from == "shell.SLDPRT"
    assert back[0].sw_mass == pytest.approx(0.245)


def test_editor_does_not_flag_a_link_whose_mass_came_from_its_mirror_source():
    """Its material really IS unset -- but the weight it carries is the source
    part's real one, so the export gate must not stop on it."""
    import json
    import os
    import tempfile

    from sw2robot.editor.webserver import _default_mass_links
    from sw2robot.exporter.model import Component, _component_states
    from sw2robot.exporter.state import GraphState

    def _comp(link, inherited):
        return Component(name=link + "-1", link_name=link, part_path="p.SLDPRT",
                         is_subassembly=False, world=np.eye(4), fixed=True,
                         dof=0, material=None, density=1000.0, sw_mass=1.0,
                         mass_inherited_from=inherited)

    graph = GraphState(
        robot_name="r", source_assembly="a.SLDASM",
        components=_component_states([_comp("mirrored", "src.SLDPRT"),
                                      _comp("plain", None)]),
        edges=[], ground=[])
    with tempfile.TemporaryDirectory() as d:
        with open(os.path.join(d, "graph.json"), "w", encoding="utf-8") as f:
            json.dump(json.loads(graph.model_dump_json()), f)
        flagged = _default_mass_links(d, "urdf/r.urdf")
    assert flagged == {"plain"}


# ------------------------------------------------- captured from real CAD

def _fixture():
    """Five mirror-copy/source pairs, as ``IBody2.GetMassProperties(1.0)`` came
    back from SolidWorks.

    Only the raw body arrays are kept -- no names, materials or masses -- since
    all these tests need is the array LAYOUT and the geometric relationship
    between the two sides.
    """
    import json
    import os
    path = os.path.join(os.path.dirname(__file__), "fixtures",
                        "mirror_body_props.json")
    with open(path, encoding="utf-8") as f:
        return json.load(f)


def test_body_array_layout_holds_on_real_documents():
    """``IBody2.GetMassProperties`` returns 12 bare doubles and the API docs are
    the only statement of what is in them; get the order wrong and every pair
    would silently look unrelated.

    Each captured pair must resolve to exactly ONE reflection -- an ordering
    mistake puts the products of inertia on the wrong axis and no candidate
    survives.  Body counts run 1 to 13, so single-body and multi-body parts are
    both covered.
    """
    from sw2robot.exporter.mirror import props_from_body_arrays, solve_reflections

    rows = _fixture()
    assert len(rows) == 5
    for row in rows:
        source = props_from_body_arrays(row["source_bodies"])
        mirror = props_from_body_arrays(row["mirror_bodies"])
        found = solve_reflections(source, mirror)
        assert found == ((1, 1, -1),), (
            f"pair {row['pair']}: expected one reflection, got {found}")


def test_real_pairs_are_separated_from_every_wrong_reflection():
    """The tolerance is only meaningful if the true reflection is far clear of
    the false ones.  Measured, the true fit is <= 4.4e-5 and the nearest wrong
    one >= 8.2e-2 -- so hold the gap at three orders of magnitude."""
    from sw2robot.exporter.mirror import (
        props_from_body_arrays,
        reflect_inertial,
    )

    for row in _fixture():
        source = props_from_body_arrays(row["source_bodies"])
        mirror = props_from_body_arrays(row["mirror_bodies"])
        scale = max(float(np.linalg.norm(source[1])), source[0] ** (1 / 3))
        i_scale = max(abs(x) for x in source[2])

        def residual(signs, source=source, mirror=mirror, scale=scale,
                     i_scale=i_scale):
            _m, com, inertia = reflect_inertial(source[0], source[1],
                                                source[2], signs)
            return max(
                np.max(np.abs(np.asarray(com) - np.asarray(mirror[1]))) / scale,
                np.max(np.abs(np.asarray(inertia)
                              - np.asarray(mirror[2]))) / i_scale)

        true_fit = residual((1, 1, -1))
        wrong = min(residual(s) for s in ((-1, 1, 1), (1, -1, 1), (-1, -1, -1)))
        assert true_fit < 1e-4
        assert wrong > 1e3 * true_fit


def test_the_real_copies_are_all_confirmed_mirror_features():
    """The external reference alone does not prove a reflection -- Insert Part
    and a split leave one too.  Every captured copy carries MirrorStock, and no
    source does."""
    for row in _fixture():
        assert row["mirror_has_mirror_feature"], row["pair"]
        assert not row["source_has_mirror_feature"], row["pair"]


# ------------------------------------------------------------- the driver

def _component(link, path, is_sub=False, mass=1.0, com=None, inertia=None,
               world=None):
    from sw2robot.exporter.model import Component
    return Component(
        name=link + "-1", link_name=link, part_path=path,
        is_subassembly=is_sub,
        world=np.eye(4) if world is None else world,
        fixed=False, dof=None, sw_mass=mass,
        sw_com=list(com or [0.0, 0.0, 0.0]),
        sw_inertia=list(inertia or [0.1, 0.0, 0.0, 0.1, 0.0, 0.1]))


def _stub_plans(monkeypatch, plans):
    """Replace the SolidWorks-facing planner with a fixed lookup by part path."""
    from sw2robot.exporter import mirror
    monkeypatch.setattr(mirror, "_plan_for_part",
                        lambda app, path, doc_of: plans.get(path))


class _NoApp:
    """Stands in for ISldWorks: the stubbed planner never reaches it."""

    def GetDocuments(self):
        return []


def test_driver_fixes_a_mirrored_part_link(monkeypatch):
    from sw2robot.exporter.mirror import apply_mirror_mass_inheritance

    plan = {"source": "/cad/src.SLDPRT", "reason": "no material on the copy",
            "wrong": {"mass": 38.188, "com": [0.0, 0.0, 0.05],
                      "inertia": (1.0, 0.0, 0.0, 1.0, 0.0, 1.0)},
            "mass": 0.245, "com": [0.0, 0.0, -0.05],
            "inertia": (0.01, 0.0, 0.0, 0.01, 0.0, 0.01)}
    _stub_plans(monkeypatch, {"/cad/mir.SLDPRT": plan})
    comp = _component("foot", "/cad/mir.SLDPRT", mass=38.188)
    reports = apply_mirror_mass_inheritance(_NoApp(), [comp])

    assert comp.sw_mass == pytest.approx(0.245)
    assert comp.sw_com == [0.0, 0.0, -0.05]
    assert comp.mass_inherited_from == "src.SLDPRT"
    assert len(reports) == 1 and reports[0]["before"] == pytest.approx(38.188)


def test_driver_fixes_the_sub_assembly_that_contains_the_copy(monkeypatch):
    """The link is the rigid foot sub-assembly; SolidWorks summed its total
    from the copy's default density, so correcting the part file alone would
    leave the LINK at 38.480 kg."""
    from sw2robot.exporter.mirror import apply_mirror_mass_inheritance

    plan = {"source": "/cad/src.SLDPRT", "reason": "no material on the copy",
            "wrong": {"mass": 38.188, "com": [0.0, 0.0, 0.0],
                      "inertia": (1.0, 0.0, 0.0, 1.0, 0.0, 1.0)},
            "mass": 0.245, "com": [0.0, 0.0, 0.0],
            "inertia": (0.01, 0.0, 0.0, 0.01, 0.0, 0.01)}
    _stub_plans(monkeypatch, {"/cad/mir.SLDPRT": plan})

    child = _component("mir", "/cad/mir.SLDPRT", mass=38.188,
                       inertia=[1.0, 0.0, 0.0, 1.0, 0.0, 1.0])
    link = _component("foot_link", "/cad/foot.SLDASM", is_sub=True,
                      mass=38.480, inertia=[1.02, 0.0, 0.0, 1.02, 0.0, 1.02])
    subs = {"/cad/foot.SLDASM": ([child], {}, set())}
    apply_mirror_mass_inheritance(_NoApp(), [link], subs)

    assert link.sw_mass == pytest.approx(38.480 - 38.188 + 0.245)
    assert link.mass_inherited_from == "src.SLDPRT"
    # the child state is corrected too -- a movable sub-assembly is expanded at
    # build time and this child becomes the link instead
    assert child.sw_mass == pytest.approx(0.245)
    assert child.mass_inherited_from == "src.SLDPRT"


def test_driver_repairs_a_nested_sub_assembly_bottom_up(monkeypatch):
    """foot sub-assembly inside a shin sub-assembly: the shin's total has to be
    corrected by the FIXED foot total, not the raw one, or the 37.9 kg would be
    subtracted once and added back."""
    from sw2robot.exporter.mirror import apply_mirror_mass_inheritance

    plan = {"source": "/cad/src.SLDPRT", "reason": "no material on the copy",
            "wrong": {"mass": 38.188, "com": [0.0, 0.0, 0.0],
                      "inertia": (1.0, 0.0, 0.0, 1.0, 0.0, 1.0)},
            "mass": 0.245, "com": [0.0, 0.0, 0.0],
            "inertia": (0.01, 0.0, 0.0, 0.01, 0.0, 0.01)}
    _stub_plans(monkeypatch, {"/cad/mir.SLDPRT": plan})

    child = _component("mir", "/cad/mir.SLDPRT", mass=38.188,
                       inertia=[1.0, 0.0, 0.0, 1.0, 0.0, 1.0])
    foot = _component("foot_link", "/cad/foot.SLDASM", is_sub=True,
                      mass=38.480, inertia=[1.02, 0.0, 0.0, 1.02, 0.0, 1.02])
    shin = _component("shin_link", "/cad/shin.SLDASM", is_sub=True,
                      mass=44.9, inertia=[2.0, 0.0, 0.0, 2.0, 0.0, 2.0])
    subs = {"/cad/foot.SLDASM": ([child], {}, set()),
            "/cad/shin.SLDASM": ([foot], {}, set())}
    apply_mirror_mass_inheritance(_NoApp(), [shin], subs)

    assert foot.sw_mass == pytest.approx(38.480 - 38.188 + 0.245)
    assert shin.sw_mass == pytest.approx(44.9 - 38.480 + foot.sw_mass)


def test_driver_leaves_an_ordinary_part_untouched(monkeypatch):
    from sw2robot.exporter.mirror import apply_mirror_mass_inheritance

    _stub_plans(monkeypatch, {})
    comp = _component("plain", "/cad/plain.SLDPRT", mass=1.5)
    assert apply_mirror_mass_inheritance(_NoApp(), [comp]) == []
    assert comp.sw_mass == 1.5
    assert comp.mass_inherited_from is None


def test_driver_reports_a_skip_without_changing_anything(monkeypatch):
    """A pair that IS a mirror but did not check out has to be visible -- a
    wrong mass nobody mentions is the failure mode this whole module exists to
    avoid."""
    from sw2robot.exporter.mirror import apply_mirror_mass_inheritance

    _stub_plans(monkeypatch, {"/cad/mir.SLDPRT": {
        "source": "/cad/src.SLDPRT", "skip": "no reflection maps it"}})
    comp = _component("odd", "/cad/mir.SLDPRT", mass=9.0)
    reports = apply_mirror_mass_inheritance(_NoApp(), [comp])

    assert comp.sw_mass == 9.0
    assert comp.mass_inherited_from is None
    assert reports == [{"link": "odd", "source": "/cad/src.SLDPRT",
                        "skip": "no reflection maps it"}]


# ------------------------------------------------- reporting the right number

def test_the_total_counts_one_mass_error_once(capsys):
    """A mirrored part and the sub-assembly holding it are BOTH corrected --
    only one of them becomes a link, so summing both would report double.

    38.188 -> 0.245 on the part, 38.480 -> 0.538 on the sub-assembly around it:
    the robot loses 37.942 kg, not 75.9.
    """
    from sw2robot.exporter.mirror import print_mirror_report

    print_mirror_report([
        {"link": "shell_link", "source": "src.SLDPRT", "top": True,
         "before": 38.480, "after": 0.538, "reason": "r"},
        {"link": "mir_shell", "source": "src.SLDPRT", "top": False,
         "before": 38.188, "after": 0.245, "reason": "r"},
    ])
    out = capsys.readouterr().out
    assert "loses 37.942 kg" in out
    assert "(inside) mir_shell" in out         # listed, but not counted


def test_report_is_silent_when_nothing_was_inherited(capsys):
    from sw2robot.exporter.mirror import print_mirror_report

    print_mirror_report([])
    assert capsys.readouterr().out == ""


def test_identical_materials_do_not_produce_a_not_inherited_line(monkeypatch):
    """Twenty instances of one mirrored screw once filled a whole extract log
    with "the copy sets its own material ('X' vs the source's 'X')".  Rebuilt
    geometry differs in the last decimal, and on a 5 g screw that clears any
    relative threshold -- but agreeing on the material is not a divergence."""
    from sw2robot.exporter import mirror

    monkeypatch.setattr(mirror, "mirror_source_path",
                        lambda app, path: "/cad/src.SLDPRT")
    monkeypatch.setattr(mirror, "has_mirror_feature", lambda doc: True)
    monkeypatch.setattr(mirror, "_read_props", lambda doc: {
        "material": "steel", "density": 7850.0,
        "mass": 0.005 if doc == "mirror" else 0.0049,
        "com": [0, 0, 0], "inertia": (1, 0, 0, 1, 0, 1), "overridden": False})

    plan = mirror._plan_for_part(
        None, "/cad/mir.SLDPRT",
        lambda p: "mirror" if p.endswith("mir.SLDPRT") else "source")
    assert plan is None


def test_a_different_material_still_gets_reported(monkeypatch):
    """The other half of that rule: a copy that names its own material is left
    alone AND said out loud, because someone should look at it."""
    from sw2robot.exporter import mirror

    monkeypatch.setattr(mirror, "mirror_source_path",
                        lambda app, path: "/cad/src.SLDPRT")
    monkeypatch.setattr(mirror, "has_mirror_feature", lambda doc: True)
    monkeypatch.setattr(mirror, "_read_props", lambda doc: {
        "material": "alloy A" if doc == "mirror" else "alloy B",
        "density": 2680.0 if doc == "mirror" else 2810.0,
        "mass": 0.080 if doc == "mirror" else 0.084,
        "com": [0, 0, 0], "inertia": (1, 0, 0, 1, 0, 1), "overridden": False})

    plan = mirror._plan_for_part(
        None, "/cad/mir.SLDPRT",
        lambda p: "mirror" if p.endswith("mir.SLDPRT") else "source")
    assert plan and "sets its own material" in plan["skip"]
