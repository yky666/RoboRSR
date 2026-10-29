"""Optional pinned-physics tests; the platform contract remains testable without Newton."""

import json
from pathlib import Path
import sys

import numpy as np
import pytest

pytest.importorskip("newton")
pytest.importorskip("pxr")
sys.path.insert(0, str(Path(__file__).resolve().parent))
from make_deformable import build, cloth_grid, mesh_hash


def test_cloth_topology_and_hash():
    p, f = cloth_grid(np.array([0.4, 0.6]), 8)
    assert p.shape == (81, 3) and f.shape == (128, 3)
    normal = np.cross(p[f[:, 1]] - p[f[:, 0]], p[f[:, 2]] - p[f[:, 0]])
    assert np.all(normal[:, 2] > 0)
    assert mesh_hash(p, f) != mesh_hash(p * 2, f)


def test_soft_block_preserves_mass_and_relocatable_configuration(tmp_path):
    spec = {
        "id": "Dynamic/test_soft/00000",
        "estimated_mass_g": 30,
        "mass_policy": "estimate",
        "deformable": {"family": "volume", "geometry": {"size_mm": [100, 100, 100], "cells": 2}},
    }
    asset = build(spec, tmp_path, Path("."))
    meta = json.loads((asset / "metadata.json").read_text())
    assert np.isclose(meta["physics"]["mass"], 0.03)
    np.testing.assert_allclose(meta["physics"]["size"], [0.1, 0.1, 0.1], atol=1e-6)
    assert (asset / "object.usdz").is_file() and not meta["asset_accepted"]
    assert len(meta["deformable"]["mesh_sha256"]) == 64
    with pytest.raises(FileExistsError):
        build(spec, tmp_path, Path("."))


def test_missing_mass_or_unsupported_topology_rejected(tmp_path):
    spec = {"id": "Garment/test/00000", "deformable": {"family": "bag", "geometry": {"size_mm": [200, 300]}}}
    with pytest.raises(ValueError, match="supports"):
        build(spec, tmp_path, Path("."))
    spec["deformable"]["family"] = "cloth"
    with pytest.raises(ValueError, match="mass"):
        build(spec, tmp_path, Path("."))
    assert not (tmp_path / spec["id"]).exists()
