"""Light unit coverage for the post-build dropped-geometry validator's pure
helpers (the full geometry check needs trimesh+scipy+skrobot + a real package
and is exercised end-to-end by the pipeline tests)."""
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from sw2robot.exporter.validate import _load_glb_verts, _origin_mat


def test_origin_mat_none_is_identity():
    assert np.allclose(_origin_mat(None), np.eye(4))


def test_origin_mat_reads_xyz_and_rpy():
    el = ET.fromstring('<origin xyz="1 2 3" rpy="0 0 0"/>')
    T = _origin_mat(el)
    assert np.allclose(T[:3, :3], np.eye(3))
    assert np.allclose(T[:3, 3], [1.0, 2.0, 3.0])


def test_origin_mat_applies_rotation():
    # 90 deg about Z: x-axis maps to +y
    el = ET.fromstring(f'<origin xyz="0 0 0" rpy="0 0 {np.pi / 2}"/>')
    T = _origin_mat(el)
    assert np.allclose(T[:3, :3] @ [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], atol=1e-9)


def test_load_glb_verts_missing_returns_none():
    assert _load_glb_verts("does_not_exist_anywhere.glb") is None


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
