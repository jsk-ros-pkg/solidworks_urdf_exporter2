"""Transform helpers: SolidWorks IMathTransform -> 4x4 and URDF xyz/rpy.

SolidWorks ``IMathTransform.ArrayData`` is ``[r0..r8, tx, ty, tz, scale, ...]``.
The documented point transform (local -> global) is the row-vector form::

    gx = x*r0 + y*r3 + z*r6 + tx
    gy = x*r1 + y*r4 + z*r7 + ty
    gz = x*r2 + y*r5 + z*r8 + tz

so the column-vector rotation matrix is ``reshape(r,(3,3)).T``.  Translations
are in METRES (the SW API is metric), which is what URDF wants.

Only helpers with sw2robot-specific behaviour live here (SW marshalling, URDF
text stability, strict ``<origin>`` parsing).  Plain rotation/transform math is
imported straight from ``skrobot.coordinates.math`` at each call site --
``matrix_relative`` for parent^-1 @ child, ``rpy2homogeneous`` / ``rpy2matrix``
for rpy -> matrix (URDF convention, extrinsic X-Y-Z: R = Rz @ Ry @ Rx).
NB: skrobot's legacy ``rpy_matrix(yaw, pitch, roll)`` takes its arguments in
the OPPOSITE order -- avoid it.
"""

from __future__ import annotations

import numpy as np
from skrobot.coordinates.math import (
    matrix2xyzrpy,
    orthonormalize_rotation_matrix,
    rpy2homogeneous,
)


def transform_to_matrix(array_data):
    """SolidWorks ArrayData (len>=12) -> 4x4 numpy (local->global, metres)."""
    a = list(array_data)
    r = a[:9]
    t = a[9:12]
    # column-vector rotation = transpose of the row-major 3x3
    R = np.array([[r[0], r[3], r[6]],
                  [r[1], r[4], r[7]],
                  [r[2], r[5], r[8]]], dtype=float)
    M = np.eye(4)
    # SW matrices carry float drift after long mate/assembly chains; project
    # back onto SO(3) here, at the single entry point, so downstream
    # rpy/quaternion conversions never see a non-orthogonal rotation.
    M[:3, :3] = orthonormalize_rotation_matrix(R)
    M[:3, 3] = t
    return M


def matrix_to_xyz_rpy(M):
    """4x4 -> (xyz, rpy) with URDF rpy = extrinsic XYZ (roll,pitch,yaw)."""
    xyz, rpy = matrix2xyzrpy(np.asarray(M, dtype=float))
    # snap sub-1e-12 values (metres / radians -- far below any physical
    # significance) to EXACT 0: the SVD in orthonormalize_rotation_matrix
    # leaves BLAS-dependent noise (~1e-16..1e-33) that differs per platform,
    # and %.8g would faithfully print it, breaking cross-platform golden
    # comparisons of the URDF text.  Also folds IEEE negative zeros.
    def _snap(v):
        v = float(v)
        return 0.0 if abs(v) < 1e-12 else v
    return [_snap(v) for v in xyz], [_snap(v) for v in rpy]


def urdf_origin_matrix(el):
    """4x4 from a URDF ``<origin>`` element (xyz + extrinsic-XYZ rpy
    attributes), or identity when the element is absent.  The ONE parser for
    ``<origin>`` -- exporter and editor both route through it so the rpy
    convention cannot drift between modules."""
    if el is None:
        return np.eye(4)
    xyz = [float(x) for x in (el.get("xyz") or "0 0 0").split()]
    rpy = [float(x) for x in (el.get("rpy") or "0 0 0").split()]
    if len(xyz) != 3 or len(rpy) != 3:
        raise ValueError(
            "malformed <origin>: expected 3 values each, got "
            f"xyz={el.get('xyz')!r} rpy={el.get('rpy')!r}")
    M = rpy2homogeneous(*rpy)
    M[:3, 3] = xyz
    return M
