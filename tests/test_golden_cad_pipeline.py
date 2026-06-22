"""Golden (characterization) tests that PIN the current CAD->URDF edit output.

These do not assert that the output is *correct* -- they assert it does not
*change*.  They exist as the safety net for the planned refactor that unifies
the two parallel edit systems (the joints.yaml + ``build()`` rebuild used by the
web server, and the ``JointEdit`` overlay + ``build_urdf`` used by core.py) onto
a single overlay applied directly to the URDF.  Every edit that will migrate is
locked here against the committed ``tests/golden`` snapshots, so the refactor is
verified by "the bytes are identical".

Coverage (all on the committed ``examples/fingertip`` package, headless -- no
SolidWorks):
  * base build                      -> fingertip_base.urdf
  * config build (rename/type/      -> fingertip_edited.urdf
    limits/mimic)
  * _flip_axis_in_urdf (axis flip)  -> fingertip_flip.urdf
  * _rename_in_urdf (direct rename) -> fingertip_rename_direct.urdf
  * colors: round-trip through joints.yaml -> _read_colors

To intentionally re-baseline after a deliberate output change, run with
``SW2ROBOT_UPDATE_GOLDEN=1`` and review the diff before committing.
"""

import os
import shutil
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parent.parent
GOLDEN = REPO_ROOT / "tests" / "golden"
FINGERTIP = REPO_ROOT / "examples" / "fingertip"
URDF_REL = "urdf/fingertip.urdf"

# joint / link names in the committed fingertip fixture
REV_JOINT = "fingertip_front_2__fingertip_back_1"
TIP_LINK = "fingertip_back_1"


def _require_fixture():
    if not (FINGERTIP / "graph.json").is_file():
        pytest.skip(f"missing fixture graph: {FINGERTIP / 'graph.json'}")


def _fresh_pkg(tmp_path):
    """A throwaway copy of the fingertip package (graph.json + meshes) so a build
    never dirties the committed fixture."""
    work = tmp_path / "pkg"
    (work / "meshes").mkdir(parents=True)
    shutil.copy2(FINGERTIP / "graph.json", work / "graph.json")
    for f in (FINGERTIP / "meshes").iterdir():
        if f.is_file():
            shutil.copy2(f, work / "meshes" / f.name)
    return work


def _assert_golden(name, actual):
    """Compare ``actual`` text to ``tests/golden/<name>`` byte for byte (line
    list, for a readable assert), or re-baseline it under SW2ROBOT_UPDATE_GOLDEN."""
    golden = GOLDEN / name
    if os.environ.get("SW2ROBOT_UPDATE_GOLDEN"):
        golden.write_text(actual, encoding="utf-8")
        pytest.skip(f"re-baselined {name}")
    assert golden.is_file(), f"missing golden {golden} (run SW2ROBOT_UPDATE_GOLDEN=1)"
    expected = golden.read_text(encoding="utf-8")
    assert actual.splitlines() == expected.splitlines(), (
        f"{name} drifted from the committed golden -- the edit pipeline changed "
        f"its URDF output.  If intentional, re-baseline with "
        f"SW2ROBOT_UPDATE_GOLDEN=1 and review the diff."
    )


def test_golden_base_build(tmp_path):
    """build() with no config -> the baseline URDF."""
    _require_fixture()
    from sw2robot.exporter.export import build

    work = _fresh_pkg(tmp_path)
    urdf = build(str(work))
    _assert_golden("fingertip_base.urdf", Path(urdf).read_text(encoding="utf-8"))


def test_golden_edited_build(tmp_path):
    """build() with the edit config -> rename + type override + limits + mimic
    all baked into the URDF (the edits that migrate to the overlay)."""
    _require_fixture()
    from sw2robot.exporter.export import build

    work = _fresh_pkg(tmp_path)
    cfg = GOLDEN / "fingertip_edit.joints.yaml"
    urdf = build(str(work), config_path=str(cfg))
    _assert_golden("fingertip_edited.urdf", Path(urdf).read_text(encoding="utf-8"))


def test_golden_flip_axis_direct(tmp_path):
    """_flip_axis_in_urdf negates the revolute joint's axis directly in the
    served URDF (no rebuild) -- the existing URDF-direct edit path."""
    _require_fixture()
    from sw2robot.editor.webserver import _flip_axis_in_urdf
    from sw2robot.exporter.export import build

    work = _fresh_pkg(tmp_path)
    build(str(work))
    flipped = _flip_axis_in_urdf(str(work), URDF_REL, REV_JOINT)
    assert flipped == 1, "expected exactly one axis flipped"
    _assert_golden("fingertip_flip.urdf",
                   (work / "urdf" / "fingertip.urdf").read_text(encoding="utf-8"))


def test_golden_rename_direct(tmp_path):
    """_rename_in_urdf renames a link and a joint (and their references) directly
    in the served URDF -- the existing URDF-direct edit path."""
    _require_fixture()
    from sw2robot.editor.webserver import _rename_in_urdf
    from sw2robot.exporter.export import build

    work = _fresh_pkg(tmp_path)
    build(str(work))
    n_link = _rename_in_urdf(str(work), URDF_REL, "link", TIP_LINK, "tip")
    n_joint = _rename_in_urdf(str(work), URDF_REL, "joint", REV_JOINT, "bend")
    assert (n_link, n_joint) == (2, 1), "rename touched an unexpected ref count"
    _assert_golden("fingertip_rename_direct.urdf",
                   (work / "urdf" / "fingertip.urdf").read_text(encoding="utf-8"))


def test_color_config_round_trips(tmp_path):
    """A per-link colour stored in the package's <name>.joints.yaml `colors:`
    block is read back verbatim by _read_colors (the colour storage/read contract
    the refactor must preserve)."""
    _require_fixture()
    from sw2robot.editor.webserver import _read_colors
    from sw2robot.exporter.export import build

    work = _fresh_pkg(tmp_path)
    # the web server keys the config off the URDF stem; mirror that layout so
    # _read_colors (which reads <stem>.joints.yaml) finds the colours block
    shutil.copy2(GOLDEN / "fingertip_edit.joints.yaml", work / "fingertip.joints.yaml")
    build(str(work), config_path=str(work / "fingertip.joints.yaml"))
    assert _read_colors(str(work), URDF_REL) == {TIP_LINK: "#ff0000"}
