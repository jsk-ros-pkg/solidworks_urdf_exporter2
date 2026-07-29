"""_sw_mass_props robustness: the inertia tensor is read across the ways
different SolidWorks builds / typelibs expose it -- the "Principle*" (sic)
spelling vs the corrected "Principal*", a property vs a method getter, and a
plain 9-element MomentOfInertia when the principal decomposition will not
marshal.  When no tensor is available it must still keep mass + COM (inertia
falls back to the mesh estimate downstream)."""
import numpy as np
import pytest

from sw2robot.exporter.model import _sw_mass_props

IDENT9 = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]


class _MP:
    """A stand-in IMassProperty: attributes are properties; a callable value
    models a method-style getter (safe_prop calls it)."""

    def __init__(self, **props):
        for k, v in props.items():
            setattr(self, k, v)


def test_principal_axes_reconstruct_tensor():
    mp = _MP(Mass=2.0, CenterOfMass=[0.1, 0.2, 0.3],
             PrincipalMomentsOfInertia=[3.0, 4.0, 5.0],
             PrincipalAxesOfInertia=IDENT9)
    mass, com, inertia6 = _sw_mass_props(mp)
    assert mass == 2.0
    assert com == [0.1, 0.2, 0.3]
    # identity axes -> diagonal tensor (ixx,ixy,ixz,iyy,iyz,izz)
    assert inertia6 == pytest.approx((3.0, 0.0, 0.0, 4.0, 0.0, 5.0))


def test_misspelled_principle_variant_is_accepted():
    # a build that only exposes the typelib's "Principle*" spelling
    mp = _MP(Mass=1.0, CenterOfMass=[0.0, 0.0, 0.0],
             PrincipleMomentsOfInertia=[3.0, 4.0, 5.0],
             PrincipleAxesOfInertia=IDENT9)
    _mass, _com, inertia6 = _sw_mass_props(mp)
    assert inertia6 == pytest.approx((3.0, 0.0, 0.0, 4.0, 0.0, 5.0))


def test_method_style_getter_is_called():
    # the moments/axes marshal as no-arg methods, not properties
    mp = _MP(Mass=1.0, CenterOfMass=[0.0, 0.0, 0.0],
             PrincipalMomentsOfInertia=lambda: [3.0, 4.0, 5.0],
             PrincipalAxesOfInertia=lambda: IDENT9)
    _mass, _com, inertia6 = _sw_mass_props(mp)
    assert inertia6 == pytest.approx((3.0, 0.0, 0.0, 4.0, 0.0, 5.0))


def test_moment_of_inertia_9elem_fallback():
    # no principal decomposition, but a full 9-element tensor is available
    mp = _MP(Mass=1.0, CenterOfMass=[0.0, 0.0, 0.0],
             MomentOfInertia=[3.0, 0.0, 0.0, 0.0, 4.0, 0.0, 0.0, 0.0, 5.0])
    _mass, _com, inertia6 = _sw_mass_props(mp)
    assert inertia6 == pytest.approx((3.0, 0.0, 0.0, 4.0, 0.0, 5.0))


def test_no_tensor_keeps_mass_and_com():
    mp = _MP(Mass=2.5, CenterOfMass=[0.1, 0.2, 0.3])
    mass, com, inertia6 = _sw_mass_props(mp)
    assert mass == 2.5
    assert com == [0.1, 0.2, 0.3]
    assert inertia6 is None            # -> mesh inertia downstream


def test_nonpositive_mass_returns_all_none():
    mp = _MP(Mass=0.0, CenterOfMass=[0.1, 0.2, 0.3],
             PrincipalMomentsOfInertia=[3.0, 4.0, 5.0],
             PrincipalAxesOfInertia=IDENT9)
    assert _sw_mass_props(mp) == (None, None, None)


def test_nonfinite_tensor_falls_back_to_mass_com():
    mp = _MP(Mass=1.0, CenterOfMass=[0.0, 0.0, 0.0],
             PrincipalMomentsOfInertia=[3.0, np.inf, 5.0],
             PrincipalAxesOfInertia=IDENT9)
    mass, com, inertia6 = _sw_mass_props(mp)
    assert mass == 1.0 and com == [0.0, 0.0, 0.0]
    assert inertia6 is None            # non-finite tensor rejected, mass/COM kept


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
