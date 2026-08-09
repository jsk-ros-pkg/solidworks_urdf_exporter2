import numpy as np
import pytest

from sw2robot.exporter import model
from sw2robot.exporter.model import (
    extract_coordinate_systems,
    extract_reference_axes,
    to_graph_state,
)
from sw2robot.exporter.state import (
    CoordinateSystemState,
    GraphState,
    ReferenceAxisState,
)


class _Transform:
    def __init__(self, values):
        self.ArrayData = values


class _CoordinateData:
    def __init__(self, values):
        self.Transform = _Transform(values)
        self.accessed = False
        self.released = False

    def AccessSelections(self, _doc, _component):
        self.accessed = True
        return True

    def ReleaseSelectionAccess(self):
        self.released = True


class _ReferenceAxis:
    def __init__(self, params):
        self._params = params

    def GetRefAxisParams(self):
        return self._params


class _DocumentExtension:
    def __init__(self, transforms):
        self._transforms = transforms

    def GetCoordinateSystemTransformByName(self, name):
        return self._transforms.get(name)


class _FeatureManager:
    def __init__(self, features):
        self._features = features
        self.top_only_args = []

    def GetFeatures(self, top_only):
        self.top_only_args.append(top_only)
        return self._features


class _Feature:
    def __init__(self, name, kind, data=None):
        self.Name = name
        self._kind = kind
        self._data = data
        self._next = None

    def GetTypeName2(self):
        return self._kind

    def GetDefinition(self):
        return self._data

    def GetSpecificFeature2(self):
        return self._data

    def GetNextFeature(self):
        return self._next


class _Document:
    def __init__(self, features, coordinate_transforms=None):
        self._features = features
        self.Extension = _DocumentExtension(coordinate_transforms) \
            if coordinate_transforms is not None else None
        for index in range(len(features) - 1):
            features[index]._next = features[index + 1]

    def FirstFeature(self):
        return self._features[0] if self._features else None


def _sw_transform(M):
    """4x4 column transform -> SolidWorks row-vector ArrayData."""
    R = M[:3, :3]
    return [
        R[0, 0], R[1, 0], R[2, 0],
        R[0, 1], R[1, 1], R[2, 1],
        R[0, 2], R[1, 2], R[2, 2],
        M[0, 3], M[1, 3], M[2, 3], 1.0,
    ]


def test_extract_coordinate_systems_uses_official_transform_directly(
        monkeypatch):
    monkeypatch.setattr(model, "as_iface", lambda obj, _name: obj)
    document_from_frame = np.eye(4)
    document_from_frame[:3, :3] = np.array([
        [0.0, -1.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0],
    ])
    document_from_frame[:3, 3] = [0.12, -0.03, 0.4]
    data = _CoordinateData(_sw_transform(np.linalg.inv(document_from_frame)))
    doc = _Document([
        _Feature("Origin", "OriginProfileFeature"),
        _Feature("fl_hip", "CoordSys", data),
    ], coordinate_transforms={
        "fl_hip": _Transform(_sw_transform(document_from_frame)),
    })

    frames = extract_coordinate_systems(doc)

    assert [frame.name for frame in frames] == ["fl_hip"]
    assert np.allclose(
        frames[0].document_from_frame_matrix(), document_from_frame)
    assert data.accessed is False
    assert data.released is False


def test_extract_coordinate_systems_feature_transform_fallback_is_direct(
        monkeypatch):
    monkeypatch.setattr(model, "as_iface", lambda obj, _name: obj)
    document_from_frame = np.eye(4)
    document_from_frame[:3, 3] = [-0.2, 0.08, 0.06]
    data = _CoordinateData(_sw_transform(document_from_frame))
    doc = _Document([_Feature("fr_hip", "CoordSys", data)])

    frames = extract_coordinate_systems(doc)

    assert [frame.name for frame in frames] == ["fr_hip"]
    assert np.allclose(
        frames[0].document_from_frame_matrix(), document_from_frame)
    assert data.accessed is True
    assert data.released is True


def test_feature_manager_enumerates_reference_geometry(monkeypatch):
    monkeypatch.setattr(model, "as_iface", lambda obj, _name: obj)
    coordinate = np.eye(4)
    coordinate[:3, 3] = [0.1, 0.2, 0.3]
    features = [
        _Feature("frame", "CoordSys"),
        _Feature("axis", "RefAxis", _ReferenceAxis([
            0.1, 0.2, 0.3,
            0.1, 0.2, 0.4,
        ])),
    ]
    manager = _FeatureManager(features)
    doc = _Document([], coordinate_transforms={
        "frame": _Transform(_sw_transform(coordinate)),
    })
    doc.FeatureManager = manager

    frames = extract_coordinate_systems(doc)
    axes = extract_reference_axes(doc)

    assert [frame.name for frame in frames] == ["frame"]
    assert [axis.name for axis in axes] == ["axis"]
    assert manager.top_only_args == [False, False]


def test_graph_coordinate_system_fields_are_backward_compatible():
    graph = to_graph_state([], {}, [], "robot", "robot.SLDASM")

    assert graph.coordinate_systems == []
    assert graph.reference_axes == []
    assert graph.subassemblies == {}

    legacy = GraphState.model_validate({
        "robot_name": "legacy",
        "source_assembly": "legacy.SLDASM",
        "subassemblies": {
            "legacy-sub.SLDASM": {
                "components": [],
                "edges": [],
                "ground": [],
            },
        },
    })
    assert legacy.coordinate_systems == []
    assert legacy.reference_axes == []
    assert legacy.subassemblies[
        "legacy-sub.SLDASM"].coordinate_systems == []


def test_to_graph_state_preserves_reference_geometry(tmp_path):
    top_frame = CoordinateSystemState(
        name="base_frame",
        document_from_frame=[float(x) for x in np.eye(4).flatten()],
    )
    local_frame = CoordinateSystemState(
        name="joint_frame",
        document_from_frame=[float(x) for x in np.eye(4).flatten()],
    )
    top_axis = ReferenceAxisState(
        name="fl_hip",
        document_point=[0.1, 0.2, 0.3],
        document_direction=[0.0, -1.0, 0.0],
    )
    graph = to_graph_state(
        [], {}, [], "robot", "robot.SLDASM",
        subassemblies={"leg.SLDASM": ([], {}, [])},
        coordinate_systems=[top_frame],
        subassembly_coordinate_systems={"leg.SLDASM": [local_frame]},
        reference_axes=[top_axis],
    )
    path = tmp_path / "graph.json"
    graph.save(path)

    loaded = GraphState.load(path)

    assert [frame.name for frame in loaded.coordinate_systems] == [
        "base_frame"]
    assert [axis.name for axis in loaded.reference_axes] == ["fl_hip"]
    assert [frame.name for frame in
            loaded.subassemblies["leg.SLDASM"].coordinate_systems] == [
                "joint_frame"]


def test_extract_reference_axes_matches_official_exporter_direction(
        monkeypatch):
    monkeypatch.setattr(model, "as_iface", lambda obj, _name: obj)
    doc = _Document([
        _Feature("Origin", "OriginProfileFeature"),
        _Feature("fl_hip", "RefAxis", _ReferenceAxis([
            1.0, 2.0, 3.0,
            1.0, 5.0, 3.0,
        ])),
    ])

    axes = extract_reference_axes(doc)

    assert [axis.name for axis in axes] == ["fl_hip"]
    assert axes[0].document_point == [1.0, 2.0, 3.0]
    # Official SW2URDF uses start - end, not end - start.
    assert axes[0].document_direction == pytest.approx([0.0, -1.0, 0.0])


def test_extract_reference_axes_skips_degenerate_axis(monkeypatch, capsys):
    monkeypatch.setattr(model, "as_iface", lambda obj, _name: obj)
    doc = _Document([
        _Feature("bad_axis", "RefAxis", _ReferenceAxis([0.0] * 6)),
    ])

    assert extract_reference_axes(doc) == []
    assert "bad_axis" in capsys.readouterr().out
