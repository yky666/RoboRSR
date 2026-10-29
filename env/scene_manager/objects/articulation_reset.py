"""Preserve the auto-selected PhysX root's offset from the asset instance frame."""

import numpy as np


def _unit(q):
    q = np.asarray(q, dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all() or np.linalg.norm(q) < 1e-12:
        raise ValueError("Finite nonzero quaternion required")
    return q / np.linalg.norm(q)


def _mul(a, b):
    w, x, y, z = a
    v, i, j, k = b
    return np.array(
        [
            w * v - x * i - y * j - z * k,
            w * i + x * v + y * k - z * j,
            w * j - x * k + y * v + z * i,
            w * k + x * j - y * i + z * v,
        ]
    )


def map_physics_root(root_xyzw, initial_position, initial_orientation, position, orientation):
    """Move an initialized root by the requested instance-frame rigid transform.

    Instance quaternions are wxyz; PhysX root transform quaternions are xyzw.
    The root may be a middle link, not the USD instance's origin.
    """
    root = np.asarray(root_xyzw)
    if root.shape[-1] != 7 or not np.isfinite(root).all():
        raise ValueError("Finite (...,7) root transform required")
    before = _unit(initial_orientation)
    after = _unit(orientation)
    inverse = before.copy()
    inverse[1:] *= -1
    delta = _mul(after, inverse)
    w = delta[0]
    xyz = delta[1:]
    source = np.asarray(initial_position, float)
    target = np.asarray(position, float)
    if source.shape != (3,) or target.shape != (3,) or not np.isfinite(np.r_[source, target]).all():
        raise ValueError("Finite instance positions required")
    p = root[..., :3] - source
    rotated = p + 2 * np.cross(xyz, np.cross(xyz, p) + w * p)
    result = root.copy()
    result[..., :3] = target + rotated
    for row, out in zip(root.reshape(-1, 7), result.reshape(-1, 7)):
        q = _mul(delta, _unit(row[[6, 3, 4, 5]]))
        out[3:] = q[[1, 2, 3, 0]]
    return result
