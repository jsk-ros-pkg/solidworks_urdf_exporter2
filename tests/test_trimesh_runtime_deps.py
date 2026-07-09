"""Regression guard for PR #88: a bare ``trimesh`` dependency installs without
the optional backends (notably ``networkx`` + ``scipy``) that trimesh needs as
soon as a *Scene* is touched.  ``pip install -e .`` then succeeds, but loading a
real (multi-body) robot blows up with ``ModuleNotFoundError: networkx`` the
moment the web editor converts the scene to a single mesh.

Two tests, on purpose:

* ``test_pyproject_declares_trimesh_mesh_backends`` is the deterministic guard --
  it reads the declared dependencies and fails if networkx/scipy stop being
  pulled in, whether via the ``trimesh[easy]`` extra OR as explicit direct deps
  (we do the latter so embreex -- the one [easy] backend without a linux-aarch64
  wheel -- can be dropped there).  This catches the regression even in an
  environment that happens to have networkx/scipy from some other package.
* ``test_scene_to_single_mesh_works`` exercises the actual code path the robot
  view runs on every load, so the suite also fails outright on a fresh install
  that is genuinely missing the backends.
"""

import tomllib
from pathlib import Path

from packaging.requirements import Requirement

REPO_ROOT = Path(__file__).resolve().parent.parent
PYPROJECT = REPO_ROOT / "pyproject.toml"


def _declared_dependencies():
    deps = tomllib.loads(PYPROJECT.read_text())["project"]["dependencies"]
    return [Requirement(raw) for raw in deps]


def test_pyproject_declares_trimesh_mesh_backends():
    # networkx + scipy are what trimesh needs the moment a Scene is flattened to a
    # single mesh (PR #88); a bare ``trimesh`` install is missing them and breaks
    # robot loading.  They may be pulled EITHER via the ``trimesh[easy]`` extra OR
    # as explicit direct deps -- we list them explicitly so embreex (the one
    # [easy] backend without a linux-aarch64 wheel) can be dropped there; see
    # pyproject.toml.  Guard that networkx/scipy stay declared one way or another.
    reqs = _declared_dependencies()
    names = {r.name for r in reqs}
    assert any(r.name == "trimesh" for r in reqs), (
        "trimesh is not declared in [project].dependencies"
    )
    via_easy = any(r.name == "trimesh" and "easy" in r.extras for r in reqs)
    via_explicit = {"networkx", "scipy"} <= names
    assert via_easy or via_explicit, (
        "networkx + scipy must be pulled in -- either via trimesh[easy] or as "
        "explicit direct deps -- so Scene->single-mesh doesn't ModuleNotFoundError "
        f"(PR #88); got dependencies: {sorted(names)}"
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
