"""Real-CoACD smoke test: actually run the ``coacd`` wheel (no stubbing) on a
tiny concave mesh.  Unlike the other CoACD tests (which stub the slow C call),
this proves the optional ``coacd`` package installs AND runs on the host OS --
the CI signal that catches a missing/broken wheel on Linux/macOS.

Skipped where ``coacd`` is not installed, so it is a no-op for the default
(coacd-less) environment and only bites in the CI job that installs ``[coacd]``.
"""
import pytest

from sw2robot.exporter.ros_export import convex_decomposition, is_coacd_available

if not is_coacd_available():
    pytest.skip("coacd not installed", allow_module_level=True)


def _l_shape():
    """A small NON-convex L-prism (two overlapping boxes) -- something CoACD has
    a reason to split, kept tiny so the real run is a couple of seconds."""
    import trimesh

    a = trimesh.creation.box(extents=(2, 1, 1))
    b = trimesh.creation.box(extents=(1, 2, 1))
    b.apply_translation((0.5, 0.5, 0))
    return trimesh.util.concatenate([a, b])


def test_real_coacd_runs_and_decomposes():
    # coarse + cheap params: this is an "it runs on this OS" check, not a
    # quality check, so keep the MCTS search small
    parts = convex_decomposition(
        _l_shape(), threshold=0.2, max_convex_hull=4,
        preprocess_resolution=20, mcts_iterations=20)

    assert len(parts) >= 1                       # produced at least one hull
    for part in parts:
        assert part.vertices.shape[1] == 3 and len(part.vertices) >= 4
        assert part.faces.shape[1] == 3 and len(part.faces) >= 4
        assert part.is_watertight
