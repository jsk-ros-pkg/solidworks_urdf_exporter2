"""PBR finish on ``.glb`` visuals and the Isaac Sim package.  No SolidWorks
needed -- a synthetic package around the committed sample ``.3dxml``."""
import io
import os
import shutil
import xml.etree.ElementTree as ET

import pytest

_SAMPLE_MESH = os.path.join("examples", "fingertip", "meshes",
                            "fingertip_back_1.3dxml")


def _make_pkg(tmp_path, robot="pb", comment=None):
    if not os.path.exists(_SAMPLE_MESH):
        pytest.skip("sample .3dxml mesh not present")
    (tmp_path / "meshes").mkdir(parents=True)
    (tmp_path / "urdf").mkdir()
    shutil.copy(_SAMPLE_MESH, tmp_path / "meshes" / "part.3dxml")
    prov = f"    <!-- {comment} -->\n" if comment else ""
    urdf = (
        f'<?xml version="1.0"?>\n<robot name="{robot}">\n'
        '  <link name="base_link">\n' + prov +
        '    <visual><geometry>'
        '<mesh filename="../meshes/part.3dxml"/></geometry></visual>\n'
        '    <collision><geometry>'
        '<mesh filename="../meshes/part.3dxml"/></geometry></collision>\n'
        '  </link>\n</robot>\n')
    (tmp_path / "urdf" / f"{robot}.urdf").write_text(urdf, encoding="utf-8")
    return str(tmp_path)


# ------------------------------------------------------------------ presets
def test_presets_from_appearance_and_material():
    from sw2robot.exporter import pbr

    assert pbr.from_appearance("metal/aluminum/satin finish aluminum") == "aluminum_satin"
    assert pbr.from_appearance(r"C:\\SOLIDWORKS\\data\\graphics\\materials\\metal"
                               r"\\chrome\\chrome plate.p2m") == "chrome"
    assert pbr.from_appearance("metal/aluminum/polished aluminum") == "aluminum_polished"
    assert pbr.from_appearance("metal/aluminum/cast aluminum") == "aluminum_cast"
    assert pbr.from_appearance("plastic/high gloss/red high gloss plastic") == "plastic_glossy"
    assert pbr.from_appearance("rubber/matte/black rubber") == "rubber"
    # the names SolidWorks ships (data/graphics/materials/...)
    assert pbr.from_appearance("metal/chrome/chromium plate") == "chrome"
    assert pbr.from_appearance("metal/aluminum/brushed aluminum") == "aluminum_brushed"
    assert pbr.from_appearance("metal/aluminum/sandblasted aluminum") == "aluminum_cast"
    assert pbr.from_appearance("plastic/low gloss/black low gloss plastic") == "plastic_matte"
    assert pbr.from_appearance("plastic/high gloss/black high gloss plastic") == "plastic_glossy"
    assert pbr.from_appearance("metal/steel/polished steel") == "steel_polished"
    # a colour-only appearance says nothing about the finish
    assert pbr.from_appearance("color.p2m") is None
    assert pbr.from_appearance(None) is None

    assert pbr.from_material("6061 Alloy") == "aluminum_satin"
    assert pbr.from_material("AISI 304") == "steel_satin"
    assert pbr.from_material("Chrome Stainless Steel") == "chrome"
    assert pbr.from_material("ABS PC") == "plastic_matte"
    assert pbr.from_material("PLA") == "plastic_matte"
    assert pbr.from_material("Silicone Rubber") == "rubber"
    assert pbr.from_material("Unobtainium") is None
    # a Japanese-UI SolidWorks reports its library's Japanese names
    assert pbr.from_material("6061 合金") == "aluminum_satin"
    assert pbr.from_material("炭素鋼(普通)") == "steel_satin"
    assert pbr.from_material("天然ゴム") == "rubber"


def test_resolve_priority_and_overrides():
    from sw2robot.exporter import pbr

    r = pbr.resolve(appearance="metal/chrome/chrome plate", material="ABS PC")
    assert (r["preset"], r["source"]) == ("chrome", "appearance")
    r = pbr.resolve(appearance="color.p2m", material="6061 Alloy")
    assert (r["preset"], r["source"]) == ("aluminum_satin", "material")
    r = pbr.resolve()
    assert (r["preset"], r["source"], r["metallic"]) == ("default", "default", 0.0)
    # override: a preset name, a partial mapping, a full mapping
    r = pbr.resolve(material="ABS PC", override="chrome")
    assert (r["preset"], r["source"], r["metallic"]) == ("chrome", "override", 1.0)
    r = pbr.resolve(material="6061 Alloy", override={"roughness": 0.2})
    assert r["metallic"] == 1.0 and r["roughness"] == 0.2 and r["source"] == "override"
    r = pbr.resolve(override={"metallic": 0.5, "roughness": 0.9})
    assert (r["metallic"], r["roughness"]) == (0.5, 0.9)
    with pytest.raises(ValueError):
        pbr.resolve(override="not_a_preset")
    with pytest.raises(ValueError):
        pbr.resolve(override={"metallic": 1.5})


def test_provenance_comment_is_read_back():
    from sw2robot.exporter import pbr

    parser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
    link = ET.fromstring(
        '<link name="a">\n  <!-- sw2robot material="6061 Alloy" '
        'appearance="metal/aluminum/satin finish aluminum" density="2700" '
        'inertia="hull" -->\n  <visual/>\n</link>', parser=parser)
    prov = pbr.provenance(link)
    assert prov["material"] == "6061 Alloy"
    assert prov["appearance"] == "metal/aluminum/satin finish aluminum"
    assert prov["inertia"] == "hull"
    assert pbr.provenance(ET.fromstring("<link/>")) == {}


def test_urdf_writer_records_the_appearance():
    from sw2robot.exporter.urdf_writer import _provenance_comment

    class C:
        material = "6061 Alloy"
        density = 2700.0
        appearance = "metal/aluminum/satin finish aluminum"

    line = _provenance_comment(C(), "hull")
    assert 'appearance="metal/aluminum/satin finish aluminum"' in line
    assert 'material="6061 Alloy"' in line


# ------------------------------------------------------------------ GLB write
def test_glb_with_materials_keeps_colours_and_finish():
    import numpy as np
    import trimesh

    from sw2robot.exporter import pbr
    from sw2robot.exporter.ros_export import _load_mesh_metres, _mesh_to_glb_bytes

    if not os.path.exists(_SAMPLE_MESH):
        pytest.skip("sample .3dxml mesh not present")
    plain = _mesh_to_glb_bytes(_SAMPLE_MESH)
    assert pbr.glb_pbr(plain) is None                 # trimesh writes no material
    shiny = _mesh_to_glb_bytes(_SAMPLE_MESH,
                               pbr={"metallic": 1.0, "roughness": 0.08,
                                    "preset": "chrome"})
    assert pbr.glb_pbr(shiny) == (1.0, 0.08, "chrome")
    # same geometry, and the CAD colour now lives in baseColorFactor, where a
    # trimesh-based loader (scikit-robot) reads it back as the material
    src = _load_mesh_metres(_SAMPLE_MESH)
    back = trimesh.load(io.BytesIO(shiny), file_type="glb")
    parts = list(back.geometry.values()) if isinstance(back, trimesh.Scene) else [back]
    assert sum(len(g.faces) for g in parts) == len(src.faces)
    assert np.allclose(np.vstack([g.bounds for g in parts]).reshape(-1, 3).min(0), src.bounds[0])
    want = {tuple(c) for c in np.unique(np.asarray(src.visual.face_colors), axis=0)}
    got = {tuple(int(v) for v in g.visual.material.baseColorFactor) for g in parts}
    assert got == want
    assert all(g.visual.material.metallicFactor == 1.0
               and g.visual.material.roughnessFactor == 0.08 for g in parts)

    # a two-tone mesh becomes one primitive per colour, each with its colour
    two = src.copy()
    fc = np.asarray(two.visual.face_colors).copy()
    fc[: len(fc) // 2] = [255, 0, 0, 255]
    two.visual.face_colors = fc
    glb = pbr.glb_with_materials(two, 0.0, 0.6, name="plastic_matte")
    mats = pbr.glb_materials(glb)
    assert len(mats) == 2 and {m[0] for m in mats} == {"plastic_matte"}
    assert [1.0, 0.0, 0.0, 1.0] in [m[1] for m in mats]


# ----------------------------------------------------------------- the export
def test_glb_visual_takes_its_finish_from_the_link_provenance(tmp_path):
    from sw2robot.exporter import pbr
    from sw2robot.exporter.ros_export import ISAAC_CTX_FMT, build_ros_description

    pkg_dir = _make_pkg(tmp_path, robot="pb", comment=(
        'sw2robot material="ABS PC" appearance="metal/chrome/chrome plate" '
        'density="1070" inertia="hull"'))
    files = dict(build_ros_description(pkg_dir, "pb", ctx_fmt=ISAAC_CTX_FMT))
    assert pbr.glb_pbr(files["pb_description/meshes/part.glb"]) == (1.0, 0.08, "chrome")
    assert "pb_description/meshes/part.stl" in files        # collision untouched

    # the override by link name beats the appearance; by mesh basename too
    files = dict(build_ros_description(pkg_dir, "pb", ctx_fmt=ISAAC_CTX_FMT,
                                       materials={"base_link": "rubber"}))
    assert pbr.glb_pbr(files["pb_description/meshes/part.glb"]) == (0.0, 0.85, "rubber")
    files = dict(build_ros_description(
        pkg_dir, "pb", ctx_fmt=ISAAC_CTX_FMT,
        materials={"part": {"metallic": 1.0, "roughness": 0.3}}))
    assert pbr.glb_pbr(files["pb_description/meshes/part.glb"])[:2] == (1.0, 0.3)


def test_glb_visual_without_provenance_gets_the_default_finish(tmp_path):
    from sw2robot.exporter import pbr
    from sw2robot.exporter.ros_export import GLB_CTX_FMT, build_ros_description

    pkg_dir = _make_pkg(tmp_path, robot="pd")
    files = dict(build_ros_description(pkg_dir, "pd", ctx_fmt=GLB_CTX_FMT))
    # the default (dielectric) finish; visual and collision share the one file
    assert pbr.glb_pbr(files["pd_description/meshes/part.glb"]) == (0.0, 0.6, "default")
    assert [a for a in files if a.endswith(".glb")] == ["pd_description/meshes/part.glb"]


def test_dae_visual_ignores_materials(tmp_path):
    from sw2robot.exporter.ros_export import build_ros_description

    pkg_dir = _make_pkg(tmp_path, robot="pe", comment='sw2robot material="6061 Alloy"')
    files = dict(build_ros_description(pkg_dir, "pe", materials={"base_link": "chrome"}))
    assert "pe_description/meshes/part.dae" in files
    assert not any(a.endswith(".glb") for a in files)


def test_isaac_package_layout(tmp_path):
    from sw2robot.exporter import pbr
    from sw2robot.exporter.ros_export import write_isaac_package

    pkg_dir = _make_pkg(tmp_path / "src", robot="rig", comment=(
        'sw2robot material="6061 Alloy" appearance="metal/aluminum/satin finish aluminum"'))
    out = write_isaac_package(pkg_dir, "rig", str(tmp_path / "out"))
    assert os.path.basename(out) == "rig_isaac"
    names = sorted(os.path.relpath(os.path.join(d, f), out).replace("\\", "/")
                   for d, _, fs in os.walk(out) for f in fs)
    assert names == ["README.md", "meshes/part.glb", "meshes/part.stl",
                     "urdf/rig_isaac.urdf"]
    link = ET.parse(os.path.join(out, "urdf", "rig_isaac.urdf")).getroot().find("link")
    assert link.find("visual").find(".//mesh").get("filename") == "../meshes/part.glb"
    assert link.find("collision").find(".//mesh").get("filename") == "../meshes/part.stl"
    with open(os.path.join(out, "meshes", "part.glb"), "rb") as f:
        assert pbr.glb_pbr(f.read()) == (1.0, 0.45, "aluminum_satin")
    # the relative refs resolve from the URDF's own directory
    assert os.path.exists(os.path.join(out, "urdf", "..", "meshes", "part.glb"))

    out2 = write_isaac_package(pkg_dir, "rig", str(tmp_path / "out2"),
                               pkg_name="my_isaac_pkg")
    assert os.path.basename(out2) == "my_isaac_pkg"


def test_url_style_is_validated(tmp_path):
    from sw2robot.exporter.ros_export import build_ros_description

    pkg_dir = _make_pkg(tmp_path, robot="pv")
    with pytest.raises(ValueError):
        build_ros_description(pkg_dir, "pv", url_style="absolute")
