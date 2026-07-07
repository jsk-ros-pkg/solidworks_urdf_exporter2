"""A single .SLDPRT exports as a trivial 1-link, 0-joint URDF.

A lone part has no mates to infer a kinematic tree from, so ``extract_part_graph``
yields exactly one component (the part, at identity, in its own frame) and the
build emits a one-link robot carrying the part's SolidWorks-native inertial.
The SolidWorks reads inside ``extract_part_graph`` are fully guarded, so a plain
object with no COM methods exercises the whole structure headlessly.
"""
import os
import xml.etree.ElementTree as ET

import numpy as np


class _NoComDoc:
    """Stands in for an IModelDoc2; every property lookup fails -> None props."""


def test_extract_part_graph_single_component():
    from sw2robot.exporter.model import extract_part_graph

    # os.path.join so the separator is native on every OS -- a hard-coded
    # backslash path would leave basename() unstripped on macOS/Linux CI
    part = os.path.join("cad", "Base Plate.SLDPRT")
    comps, adjacency, ground = extract_part_graph(_NoComDoc(), "robot", part)

    assert len(comps) == 1
    assert adjacency == {}
    c = comps[0]
    assert ground == [c.name]              # the sole part is its own base
    assert c.link_name == "Base_Plate"     # sanitized file stem
    assert c.part_path.endswith("Base Plate.SLDPRT")
    assert c.fixed is True and c.dof == 0
    assert np.allclose(c.world, np.eye(4))
    assert c.is_subassembly is False


def test_single_part_builds_one_link_zero_joint_urdf(tmp_path):
    """The full graph.json -> build path for a lone part: one link, no joints,
    SolidWorks inertial + the part mesh as visual/collision."""
    from sw2robot.exporter import build as buildmod
    from sw2robot.exporter.model import Component, to_graph_state

    pkg = tmp_path
    (pkg / "meshes").mkdir()
    comp = Component(
        name="widget", link_name="widget", part_path=r"C:\cad\widget.SLDPRT",
        is_subassembly=False, world=np.eye(4), fixed=True, dof=0,
        material="ABS", density=1020.0,
        sw_mass=0.35, sw_com=[0.01, 0.0, 0.02],
        sw_inertia=[1e-4, 0.0, 0.0, 1.2e-4, 0.0, 1.1e-4])
    comp.mesh_file = "meshes/widget.3dxml"
    graph = to_graph_state([comp], {}, [comp.name], "widget", comp.part_path)
    graph.save(str(pkg / "graph.json"))

    urdf_path = buildmod.build(str(pkg))
    root = ET.parse(urdf_path).getroot()

    links = root.findall("link")
    assert len(links) == 1
    assert root.findall("joint") == []
    link = links[0]
    assert link.get("name") == "base_link"          # root is renamed
    # SolidWorks-native inertial passes through
    assert float(link.find("inertial/mass").get("value")) == 0.35
    assert link.find("visual/geometry/mesh").get("filename").endswith(
        "widget.3dxml")
