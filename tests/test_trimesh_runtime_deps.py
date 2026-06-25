"""Regression guard for PR #88: a bare ``trimesh`` dependency installs without
the optional backends (notably ``networkx`` + ``scipy``) that trimesh needs as
soon as a *Scene* is touched.  ``pip install -e .`` then succeeds, but loading a
real (multi-body) robot blows up with ``ModuleNotFoundError: networkx`` the
moment the web editor converts the scene to a single mesh.

Two tests, on purpose:

* ``test_pyproject_declares_trimesh_easy_extra`` is the deterministic guard --
  it reads the declared dependency and fails if anyone reverts ``trimesh[easy]``
  back to plain ``trimesh``.  This catches the regression even in an environment
  that happens to have networkx/scipy from some other package.
* ``test_scene_to_single_mesh_works`` exercises the actual code path the robot
  view runs on every load, so the suite also fails outright on a fresh install
  that is genuinely missing the backends.
"""

import tomllib
from pathlib import Path

from packaging.requirements import Requirement

REPO_ROOT = Path(__file__).resolve().parent.parent
PYPROJECT = REPO_ROOT / "pyproject.toml"


def _trimesh_requirement():
    deps = tomllib.loads(PYPROJECT.read_text())["project"]["dependencies"]
    for raw in deps:
        req = Requirement(raw)
        if req.name == "trimesh":
            return req
    raise AssertionError("trimesh is not declared in [project].dependencies")


def test_pyproject_declares_trimesh_easy_extra():
    # ``trimesh[easy]`` is what pulls networkx + scipy (and the other mesh
    # backends) in; a bare ``trimesh`` install is missing them and breaks robot
    # loading.  Guard against reverting that extra.
    req = _trimesh_requirement()
    assert "easy" in req.extras, (
        f"trimesh must be declared with the 'easy' extra to pull networkx/scipy; "
        f"got '{req}'"
    )


def test_easy_extra_pulls_networkx_and_scipy():
    # The two backends whose absence caused the PR #88 failure must be reachable
    # through the trimesh[easy] extra.
    import importlib.metadata as md

    required = {
        Requirement(r).name
        for r in (md.requires("trimesh") or [])
        if 'extra == "easy"' in r
    }
    assert {"networkx", "scipy"} <= required, (
        f"trimesh[easy] is expected to require networkx + scipy; got {sorted(required)}"
    )


def test_scene_to_single_mesh_works():
    # The exact operation every robot view performs: load -> Scene ->
    # to single mesh.  Needs networkx (scene graph) + scipy under the hood, so it
    # fails loudly on an install that lacks them instead of only at runtime.
    import trimesh

    from sw2robot.editor.webserver import _to_single_mesh

    scene = trimesh.Scene()
    scene.add_geometry(trimesh.creation.box(extents=(1, 1, 1)))
    scene.add_geometry(
        trimesh.creation.box(extents=(1, 1, 1)),
        transform=trimesh.transformations.translation_matrix((2, 0, 0)),
    )

    mesh = _to_single_mesh(scene)
    assert isinstance(mesh, trimesh.Trimesh)
    assert len(mesh.vertices) > 0
