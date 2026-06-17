"""SolidWorks-native mass properties: the tensor reconstruction (principal
moments + axes -> tensor about the COM), the link-frame transform, and the
urdf_writer source priority (SolidWorks > mesh > placeholder)."""
import xml.etree.ElementTree as ET

import numpy as np


# ---------------------------------------------------------------- reconstruction
def test_sw_mass_props_reconstructs_tensor_from_principal_axes():
    """A fake IMassProperty whose principal axes are a known rotation of the
    part frame must round-trip to the full tensor in part axes: I = R^T D R."""
    from sw2robot.exporter.model import _sw_mass_props

    # principal moments and a rotation (its rows are the principal axes in the
    # part frame -- SolidWorks' PrincipalAxesOfInertia layout)
    pm = [2.0, 5.0, 9.0]
    from scipy.spatial.transform import Rotation
    R = Rotation.from_euler("xyz", [0.3, -0.7, 1.1]).as_matrix()
    expected = R.T @ np.diag(pm) @ R

    class FakeMP:
        Mass = 1.25
        CenterOfMass = (0.01, -0.02, 0.03)
        PrincipalMomentsOfInertia = tuple(pm)
        PrincipalAxesOfInertia = tuple(R.flatten())

    mass, com, inertia6 = _sw_mass_props(FakeMP())
    assert mass == 1.25
    assert np.allclose(com, [0.01, -0.02, 0.03])
    ixx, ixy, ixz, iyy, iyz, izz = inertia6
    got = np.array([[ixx, ixy, ixz], [ixy, iyy, iyz], [ixz, iyz, izz]])
    assert np.allclose(got, expected, atol=1e-9)


def test_sw_mass_props_rejects_degenerate():
    from sw2robot.exporter.model import _sw_mass_props

    class NoMass:
        Mass = 0.0
        CenterOfMass = (0, 0, 0)
        PrincipalMomentsOfInertia = (1, 1, 1)
        PrincipalAxesOfInertia = tuple(np.eye(3).flatten())

    assert _sw_mass_props(NoMass()) == (None, None, None)


# ---------------------------------------------------------------- transform
def test_link_inertial_from_sw_identity_origin():
    """With an identity visual origin the SW values pass through unchanged."""
    from sw2robot.exporter.inertia import link_inertial_from_sw

    info = link_inertial_from_sw(
        2.0, [0.1, 0.0, 0.0], (3.0, 0.0, 0.0, 4.0, 0.0, 5.0),
        visual_xyz=[0, 0, 0], visual_rpy=[0, 0, 0])
    assert info["method"] == "solidworks"
    assert info["mass"] == 2.0
    assert np.allclose(info["com"], [0.1, 0.0, 0.0])
    assert np.allclose(info["inertia"], (3.0, 0.0, 0.0, 4.0, 0.0, 5.0))


def test_link_inertial_from_sw_rotation_translation():
    """A 90 deg rotation about Z swaps the x/y diagonal terms; the COM is both
    rotated and translated."""
    from sw2robot.exporter.inertia import link_inertial_from_sw

    info = link_inertial_from_sw(
        1.0, [1.0, 0.0, 0.0], (2.0, 0.0, 0.0, 7.0, 0.0, 9.0),
        visual_xyz=[0.0, 0.0, 0.5], visual_rpy=[0.0, 0.0, np.pi / 2])
    # com: Rz(90)@[1,0,0] = [0,1,0], then + [0,0,0.5]
    assert np.allclose(info["com"], [0.0, 1.0, 0.5], atol=1e-9)
    ixx, ixy, ixz, iyy, iyz, izz = info["inertia"]
    # ixx/iyy swap under a 90 deg Z rotation; izz unchanged
    assert np.isclose(ixx, 7.0, atol=1e-9)
    assert np.isclose(iyy, 2.0, atol=1e-9)
    assert np.isclose(izz, 9.0, atol=1e-9)


def test_link_inertial_from_sw_none_on_missing():
    from sw2robot.exporter.inertia import link_inertial_from_sw
    assert link_inertial_from_sw(None, None, None, [0, 0, 0], [0, 0, 0]) is None


# ---------------------------------------------------------------- writer priority
def _model_with_sw():
    from sw2robot.exporter.model import Component, RobotModel
    c = Component(name="Part-1", link_name="base_link", part_path=None,
                  is_subassembly=False, world=np.eye(4), fixed=True, dof=0,
                  sw_mass=3.5, sw_com=[0.0, 0.0, 0.1],
                  sw_inertia=[0.2, 0.0, 0.0, 0.3, 0.0, 0.4])
    return RobotModel(name="demo", components=[c], joints=[],
                      base_link="base_link")


def test_writer_prefers_solidworks_values(tmp_path):
    from sw2robot.exporter import urdf_writer
    out = tmp_path / "urdf" / "demo.urdf"
    urdf_writer.write_urdf(_model_with_sw(), str(out))   # no mesh, no density
    inertial = ET.parse(out).getroot().find("link/inertial")
    assert float(inertial.find("mass").get("value")) == 3.5
    assert np.allclose([float(x) for x in inertial.find("origin").get("xyz").split()],
                       [0.0, 0.0, 0.1])
    inr = inertial.find("inertia")
    assert np.isclose(float(inr.get("ixx")), 0.2)
    assert np.isclose(float(inr.get("izz")), 0.4)


def test_explicit_density_overrides_solidworks(tmp_path):
    """An explicit global density means 'drive mass from this density' -> the
    SolidWorks values are NOT used (here there is no mesh either, so it falls
    through to the placeholder, proving the SW branch was skipped)."""
    from sw2robot.exporter import urdf_writer
    out = tmp_path / "urdf" / "demo.urdf"
    urdf_writer.write_urdf(_model_with_sw(), str(out), density=1200.0)
    inertial = ET.parse(out).getroot().find("link/inertial")
    # placeholder mass (0.1), not the SW 3.5
    assert float(inertial.find("mass").get("value")) == 0.1


# ---------------------------------------------------------------- validate_inertia
def test_validate_inertia_accepts_a_real_tensor():
    from sw2robot.exporter.inertia import validate_inertia
    # a plausible solid block: diagonal, triangle inequality holds
    assert validate_inertia(2.0, (3.0, 0.0, 0.0, 4.0, 0.0, 5.0)) == []
    # off-diagonal but still SPD + valid
    assert validate_inertia(1.0, (5.0, 0.5, 0.0, 5.0, 0.0, 6.0)) == []


def test_validate_inertia_flags_bad_mass():
    from sw2robot.exporter.inertia import validate_inertia
    probs = validate_inertia(0.0, (1.0, 0.0, 0.0, 1.0, 0.0, 1.0))
    assert any("mass" in p for p in probs)
    assert validate_inertia(-1.0, (1.0, 0.0, 0.0, 1.0, 0.0, 1.0))


def test_validate_inertia_flags_non_positive_definite():
    from sw2robot.exporter.inertia import validate_inertia
    # a negative principal moment (e.g. a sign/units bug) -> not SPD
    probs = validate_inertia(1.0, (-1.0, 0.0, 0.0, 2.0, 0.0, 3.0))
    assert any("positive definite" in p for p in probs)


def test_validate_inertia_flags_triangle_violation():
    from sw2robot.exporter.inertia import validate_inertia
    # all positive, but I1+I2 < I3 (1 + 1 < 10) -> physically impossible
    probs = validate_inertia(1.0, (1.0, 0.0, 0.0, 1.0, 0.0, 10.0))
    assert any("triangle inequality" in p for p in probs)


def test_validate_inertia_placeholder_is_valid():
    from sw2robot.exporter.inertia import validate_inertia
    from sw2robot.exporter.urdf_writer import _PLACEHOLDER_INERTIAL as P
    assert validate_inertia(P["mass"], P["inertia"]) == []
