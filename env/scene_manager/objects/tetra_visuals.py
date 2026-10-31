"""Pure tetrahedral embedding for separate visual meshes; not a material model."""

import numpy as np
from scipy.spatial import cKDTree


def bind_tetrahedra(points, rest_nodes, tetrahedra, neighbors=24):
    points = np.asarray(points, dtype=float)
    nodes = np.asarray(rest_nodes, dtype=float)
    tets = np.asarray(tetrahedra, dtype=int)
    valid = (tets >= 0).all(axis=1) & (tets < len(nodes)).all(axis=1)
    tets = tets[valid]
    cell = nodes[tets]
    mat = np.stack((cell[:, 1] - cell[:, 0], cell[:, 2] - cell[:, 0], cell[:, 3] - cell[:, 0]), axis=-1)
    valid = np.abs(np.linalg.det(mat)) > 1e-15
    tets = tets[valid]
    mat = mat[valid]
    cell = cell[valid]
    if not len(tets) or not np.isfinite(points).all():
        raise ValueError("Finite points and nonsingular tetrahedra required")
    distance, candidates = cKDTree(cell.mean(axis=1)).query(points, k=min(neighbors, len(tets)))
    if candidates.ndim == 1:
        candidates = candidates[:, None]
        distance = distance[:, None]
    inverse = np.linalg.inv(mat)
    local = np.einsum("nkij,nkj->nki", inverse[candidates], points[:, None, :] - cell[candidates, 0])
    weights = np.concatenate((1 - local.sum(axis=-1, keepdims=True), local), axis=-1)
    violation = np.maximum(0, -weights.min(axis=-1))
    pick = np.argmin(violation + distance * 1e-9, axis=1)
    row = np.arange(len(points))
    indices = tets[candidates[row, pick]]
    weights = weights[row, pick]
    reconstructed = apply_embedding(nodes, indices, weights)
    return (
        indices,
        weights,
        {
            "max_rest_error_m": float(np.linalg.norm(reconstructed - points, axis=1).max()),
            "maximum_outside_weight": float(violation[row, pick].max()),
        },
    )


def apply_embedding(nodes, indices, weights):
    return np.einsum("ni,nij->nj", weights, np.asarray(nodes)[indices])
