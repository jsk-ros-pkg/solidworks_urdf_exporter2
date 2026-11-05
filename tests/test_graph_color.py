"""graph.json ``color``: one colour per part for meshes that carry none.

A part written from code (STL) has no colour of its own.  Its graph colour must
reach the URDF and every export built from it, a joint config ``colors:``
entry must still win, and a part with no colour at all must come out the
default grey rather than black.
"""
import glob
import importlib
import os
import re
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import trimesh
import yaml
from pydantic import ValidationError

from sw2robot.exporter.graph_builder import GraphBuilder
from sw2robot.exporter.state import ComponentState


def _import_extra(name):
    """Import an optional dependency, skipping when it is absent -- unless CI
    lists it in SW2ROBOT_REQUIRE_EXTRAS, where a broken install must fail."""
    required = os.environ.get("SW2ROBOT_REQUIRE_EXTRAS", "").split(",")
    if name in required:
        return importlib.import_module(name)
    return pytest.importorskip(name)


def _package(tmp_path):
    g = GraphBuilder("c", cad="code")
    g.add_part("base", trimesh.creation.box((0.1, 0.1, 0.02)), fixed=True,
               density=1000, color="#d08770")
    g.add_part("arm", trimesh.creation.box((0.02, 0.02, 0.1)),
               pose=np.eye(4), density=1000)                 # no colour
    g.add_joint("base", "arm", "revolute", axis_dir=[0, 0, 1],
                lower=-1.0, upper=1.0)
    return g.write(str(tmp_path))


def _mjcf_visual_rgba(out_dir):
    xml = open(glob.glob(os.path.join(out_dir, "c_mjcf", "mjcf", "*.xml"))[0],
               encoding="utf-8").read()
    return {name: [float(v) for v in rgba.split()] for rgba, name in
            re.findall(r'rgba="([^"]+)" name="([a-z_]+)_visual0"', xml)}


@pytest.mark.parametrize("color, rgba", [
    ("#d08770", [0xd0 / 255, 0x87 / 255, 0x70 / 255, 1.0]),
    ("#ff000080", [1.0, 0.0, 0.0, 0x80 / 255]),
    ((0.2, 0.4, 0.6), [0.2, 0.4, 0.6, 1.0]),
])
def test_builder_accepts_hex_and_floats(tmp_path, color, rgba):
    g = GraphBuilder("r")
    g.add_part("a", trimesh.creation.box((0.1, 0.1, 0.1)), color=color)
    from sw2robot.exporter.state import GraphState
    graph = GraphState.load(os.path.join(g.write(str(tmp_path)), "graph.json"))
    np.testing.assert_allclose(graph.components[0].color, rgba)


@pytest.mark.parametrize("color", ["#12345", "#zzzzzz", (1.5, 0, 0), (1, 0)])
def test_builder_rejects_a_bad_colour(color):
    with pytest.raises(ValueError, match="colour"):
        GraphBuilder("r").add_part("a", trimesh.creation.box((1, 1, 1)),
                                   color=color)


def test_graph_colour_is_four_channels_in_0_1():
    world = [float(x) for x in np.eye(4).flatten()]
    with pytest.raises(ValidationError):
        ComponentState(name="a", link_name="a", world=world,
                       color=[1.2, 0, 0, 1])
    with pytest.raises(ValidationError):
        ComponentState(name="a", link_name="a", world=world, color=[1, 0, 0])


def test_the_urdf_carries_the_graph_colour(tmp_path):
    from sw2robot.exporter.export import build
    pkg = _package(tmp_path)
    build(pkg)
    root = ET.parse(os.path.join(pkg, "urdf", "c.urdf")).getroot()
    colors = {link.get("name"): link.find("visual/material/color")
              for link in root.findall("link")}
    rgba = [float(v) for v in colors["base_link"].get("rgba").split()]
    np.testing.assert_allclose(rgba, [0xd0 / 255, 0x87 / 255, 0x70 / 255, 1],
                               atol=1e-6)
    assert colors["arm"] is None          # no colour: the mesh decides


def test_mujoco_gets_the_graph_colour_and_grey_for_none(tmp_path):
    _import_extra("mujoco")
    from sw2robot.exporter.export import build
    build(_package(tmp_path), mujoco=True, mujoco_fixed_base=True)
    rgba = _mjcf_visual_rgba(str(tmp_path))
    np.testing.assert_allclose(rgba["base_link"],
                               [0xd0 / 255, 0x87 / 255, 0x70 / 255, 1],
                               atol=1e-6)
    # a colourless STL used to come out black (a dae material with no
    # diffuse); it now gets the grey an STL <visual> gets
    np.testing.assert_allclose(rgba["arm"], [191 / 255] * 3 + [1], atol=1e-6)


def test_a_colors_override_beats_the_graph_colour(tmp_path):
    _import_extra("mujoco")
    from sw2robot.exporter.export import build
    pkg = _package(tmp_path)
    cfg = tmp_path / "colors.yaml"
    cfg.write_text(yaml.safe_dump({"colors": {"base": "#00ff00"}}),
                   encoding="utf-8")
    build(pkg, config_path=str(cfg), mujoco=True, mujoco_fixed_base=True)
    np.testing.assert_allclose(_mjcf_visual_rgba(str(tmp_path))["base_link"],
                               [0, 1, 0, 1], atol=1e-6)


def test_a_colourless_mesh_is_grey_in_a_dae():
    from sw2robot.exporter.ros_export import _DEFAULT_VISUAL_RGBA, _collada_meshes
    (mesh,) = _collada_meshes(trimesh.creation.box((1, 1, 1)))
    assert (mesh.visual.face_colors == _DEFAULT_VISUAL_RGBA).all()
