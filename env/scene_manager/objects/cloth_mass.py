"""Explicit per-vertex cloth masses with checksum and particle-count validation."""

import hashlib
from pathlib import Path

import numpy as np


def area_vertex_masses(points, faces, total_mass_kg):
    points, faces = np.asarray(points, dtype=float), np.asarray(faces, dtype=int)
    areas = (
        np.linalg.norm(
            np.cross(points[faces[:, 1]] - points[faces[:, 0]], points[faces[:, 2]] - points[faces[:, 0]]), axis=1
        )
        / 2
    )
    if not np.isfinite(areas).all() or (areas <= 0).any() or total_mass_kg <= 0:
        raise ValueError("Finite nondegenerate cloth and positive mass required")
    weights = np.zeros(len(points))
    np.add.at(weights, faces.ravel(), np.repeat(areas / 3, 3))
    if (weights <= 0).any():
        raise ValueError("Unreferenced cloth vertices")
    return weights / weights.sum() * total_mass_kg


def load_vertex_masses(usd_path, config, count):
    root = Path(usd_path).resolve().parent
    path = (root / config["file"]).resolve()
    if not path.is_relative_to(root):
        raise ValueError("Vertex masses must stay inside asset folder")
    if hashlib.sha256(path.read_bytes()).hexdigest() != config["sha256"]:
        raise ValueError("Vertex mass SHA changed")
    values = np.load(path, allow_pickle=False)
    if values.shape != (count,) or not np.isfinite(values).all() or (values <= 0).any():
        raise ValueError("Cloth particle order/count or mass values invalid")
    if not np.isclose(values.sum(), config["total_mass_kg"], rtol=1e-7, atol=1e-10):
        raise ValueError("Vertex masses do not sum to the registered whole mass")
    return values
