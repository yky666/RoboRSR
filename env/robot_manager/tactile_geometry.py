"""Orthographic ray observations of recorded analytic simulation fixtures.

This observes the same sphere/box shapes and measured poses as PhysX. It does
not invent contact masks or apply forces. +Z points from the gel into the gap.
The G1-WS effective FEM area is cropped from the CAD pad, without rescaling CAD.
"""

import numpy as np

GRID_WIDTH = 100
GRID_HEIGHT = 175
ACTIVE_WIDTH_M = 0.0173
ACTIVE_HEIGHT_M = 0.02914
FEM_CENTRE_Y_M = 0.01557


def rotation_wxyz(q):
    q = np.asarray(q, dtype=float)
    norm = np.linalg.norm(q)
    if not np.isfinite(q).all() or abs(norm - 1.0) > 0.01:
        raise ValueError("Invalid quaternion")
    w, x, y, z = q / norm
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def object_matrix(pose_wxyz):
    t = np.eye(4)
    t[:3, :3], t[:3, 3] = rotation_wxyz(pose_wxyz[3:]), pose_wxyz[:3]
    return t


def depth_from_scene(pad_world, objects, geometry, *, include_clearance=False):
    """Return (H,W) float32 mm; zero=no intrusion, negative=inside gel.

    Pixel centres span the effective FEM area. Row zero is negative local Y;
    this computational convention is not a claim of measured optical alignment.
    Rays start 4 mm behind the CAD contact plane and travel along +Z to 5 mm.
    """
    pad = np.asarray(pad_world, dtype=float)
    if pad.shape != (4, 4) or not np.isfinite(pad).all():
        raise ValueError("Invalid pad matrix")
    x = (np.arange(GRID_WIDTH) + 0.5) / GRID_WIDTH * ACTIVE_WIDTH_M - ACTIVE_WIDTH_M / 2
    y = (np.arange(GRID_HEIGHT) + 0.5) / GRID_HEIGHT * ACTIVE_HEIGHT_M - ACTIVE_HEIGHT_M / 2
    yy, xx = np.meshgrid(y, x, indexing="ij")
    origins = np.stack([xx, yy, np.full_like(xx, -0.004)], -1).reshape(-1, 3)
    world_origins = origins @ pad[:3, :3].T + pad[:3, 3]
    world_dir = pad[:3, 2]
    nearest = np.full(len(origins), np.inf)
    for name, state in objects.items():
        geo = geometry[name]
        t = object_matrix(state["pose_wxyz"])
        o = (world_origins - t[:3, 3]) @ t[:3, :3]
        d = world_dir @ t[:3, :3]
        if geo["kind"] == "sphere":
            b = o @ d
            c = np.sum(o * o, axis=1) - geo["radius_m"] ** 2
            disc = b * b - c
            hit = -b - np.sqrt(np.maximum(disc, 0))
            valid = (disc >= 0) & (hit >= 0)
        elif geo["kind"] == "box":
            half = np.asarray(geo["size_m"]) / 2
            entry = np.full_like(o, -np.inf)
            leave = np.full_like(o, np.inf)
            valid = np.ones(len(o), dtype=bool)
            for axis in range(3):
                if abs(d[axis]) < 1e-12:
                    valid &= np.abs(o[:, axis]) <= half[axis]
                else:
                    a = (-half[axis] - o[:, axis]) / d[axis]
                    b = (half[axis] - o[:, axis]) / d[axis]
                    entry[:, axis], leave[:, axis] = np.minimum(a, b), np.maximum(a, b)
            hit = entry.max(axis=1)
            valid &= (leave.min(axis=1) >= hit) & (hit >= 0)
        else:
            raise ValueError("Unsupported geometry; no guessed tactile mask")
        valid &= hit <= 0.009
        nearest = np.minimum(nearest, np.where(valid, hit, np.inf))
    surface = np.where(np.isfinite(nearest), nearest - 0.004, 0.005)
    # FEM needs positive separation to release contacts. A zero-only background
    # means an infinite tool exactly on the rest plane, not an absent object.
    depth = surface if include_clearance else np.minimum(surface, 0.0)
    return (depth.reshape(GRID_HEIGHT, GRID_WIDTH) * 1000).astype(np.float32)


def fem_world_matrix(contact_world):
    """Native FEM nodes have y=[1,30.14] mm, z=[-3,0] mm."""
    shift = np.eye(4)
    shift[1, 3] = -FEM_CENTRE_Y_M
    return np.asarray(contact_world) @ shift
