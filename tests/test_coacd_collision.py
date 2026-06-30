"""CoACD convex parts driving the live self-collision model: the preview-GLB
loader, and that SelfCollision(parts=...) ignores intra-link part seams while
still catching real inter-link collisions.  Uses lightweight robot stubs +
box meshes (trimesh + python-fcl only, no skrobot)."""
import numpy as np
import trimesh

from sw2robot.editor.autoinit import SelfCollision, load_collision_parts


def _box(center):
    b = trimesh.creation.box(extents=(1, 1, 1))
    b.apply_translation(center)
    return b


class _Coords:
    def __init__(self, T):
        self._T = np.asarray(T, float)

    def T(self):
        return self._T


class _Link:
    def __init__(self, name, T=None):
        self.name = name
        self._c = _Coords(np.eye(4) if T is None else T)

    def worldcoords(self):
        return self._c

    def move_to(self, xyz):
        self._c._T = np.eye(4)
        self._c._T[:3, 3] = xyz


class _Robot:
    def __init__(self, links):
        self.link_list = links
        self.joint_list = []          # no joints -> no adjacency baseline


def test_load_collision_parts_roundtrip(tmp_path):
    """A preview GLB of N parts loads back as N meshes in the same frame."""
    preview = tmp_path / "preview"
    preview.mkdir()
    scene = trimesh.Scene()
    scene.add_geometry(_box((0, 0, 0)))
    scene.add_geometry(_box((3, 0, 0)))
    (preview / "my_link.glb").write_bytes(scene.export(file_type="glb"))

    parts = load_collision_parts(str(preview), ["my_link", "absent_link"])
    assert set(parts) == {"my_link"}            # absent link omitted
    assert len(parts["my_link"]) == 2
    # frame preserved: the two boxes still span x in [-0.5, 3.5]
    allv = np.vstack([p.vertices for p in parts["my_link"]])
    assert allv[:, 0].min() == -0.5 and abs(allv[:, 0].max() - 3.5) < 1e-6


def test_parts_ignore_intra_link_seams_catch_inter_link():
    # link a: two boxes that OVERLAP (a CoACD seam) -- must NOT self-flag
    a, b = _Link("a"), _Link("b")
    robot = _Robot([a, b])
    meshes = {"a": _box((0, 0, 0)), "b": _box((0, 0, 0))}   # hull fallback only
    parts = {
        "a": [_box((0, 0, 0)), _box((0.9, 0, 0))],          # overlap at x~0.4..0.5
        "b": [_box((5, 0, 0))],                              # far away at rest
    }
    sc = SelfCollision(robot, meshes, parts=parts)

    # the intra-link overlap is not a collision, and b is far -> nothing flagged
    assert frozenset(("a", "b")) not in sc.baseline
    new, offenders = sc.offenders()
    assert new == set() and offenders == set()

    # move b so its part lands on link a's parts -> a real inter-link collision
    b.move_to((-4, 0, 0))                # b's part 5->1, overlaps a's 0.9-box
    new, offenders = sc.offenders()
    assert frozenset(("a", "b")) in new
    assert offenders == {"a", "b"}
