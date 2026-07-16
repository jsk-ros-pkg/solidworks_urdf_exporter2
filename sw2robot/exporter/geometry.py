"""Transform helpers: SolidWorks IMathTransform -> 4x4 and URDF xyz/rpy.

SolidWorks ``IMathTransform.ArrayData`` is ``[r0..r8, tx, ty, tz, scale, ...]``.
The documented point transform (local -> global) is the row-vector form::

    gx = x*r0 + y*r3 + z*r6 + tx
    gy = x*r1 + y*r4 + z*r7 + ty
    gz = x*r2 + y*r5 + z*r8 + tz

so the column-vector rotation matrix is ``reshape(r,(3,3)).T``.  Translations
are in METRES (the SW API is metric), which is what URDF wants.
"""

from __future__ import annotations

import numpy as np

# rpy2matrix / matrix2rpy follow the URDF convention (extrinsic X-Y-Z:
# R = Rz @ Ry @ Rx).  NB: skrobot's legacy ``rpy_matrix(yaw, pitch, roll)``
# takes its arguments in the OPPOSITE order -- use rpy2matrix here.
from skrobot.coordinates.math import matrix2rpy, rpy2matrix


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
    M[:3, :3] = R
    M[:3, 3] = t
    return M


def matrix_to_xyz_rpy(M):
    """4x4 -> (xyz, rpy) with URDF rpy = extrinsic XYZ (roll,pitch,yaw)."""
    xyz = [float(M[0, 3]), float(M[1, 3]), float(M[2, 3])]
    roll, pitch, yaw = matrix2rpy(np.asarray(M[:3, :3], dtype=float))
    # + 0.0 folds IEEE negative zeros so the URDF text stays stable
    return xyz, [float(roll) + 0.0, float(pitch) + 0.0, float(yaw) + 0.0]


def relative_matrix(parent_world, child_world):
    """child expressed in parent frame = parent^-1 * child."""
    return np.linalg.inv(parent_world) @ child_world


def matrix_from_rpy(rpy):
    """4x4 rotation from extrinsic-XYZ roll/pitch/yaw (URDF convention)."""
    roll, pitch, yaw = rpy
    M = np.eye(4)
    M[:3, :3] = rpy2matrix(float(roll), float(pitch), float(yaw))
    return M


def frame_at_point(point, R=None):
    """4x4 frame located at ``point`` (world); rotation defaults to identity."""
    M = np.eye(4)
    if R is not None:
        M[:3, :3] = R
    M[:3, 3] = point
    return M


def transform_point_dir(M, point_local, dir_local):
    """Map a (point, direction) from a local frame to world via 4x4 M."""
    R = M[:3, :3]
    t = M[:3, 3]
    p = R @ np.asarray(point_local, float) + t
    d = R @ np.asarray(dir_local, float)
    n = np.linalg.norm(d)
    return p, (d / n if n > 1e-12 else d)
