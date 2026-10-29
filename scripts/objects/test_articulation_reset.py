"""Instance-frame reset and flexible runtime contracts, without simulator imports."""

import hashlib
import json
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "env/scene_manager/objects"))
from articulation_reset import map_physics_root
from asset_defaults import with_asset_defaults
from cloth_mass import area_vertex_masses, load_vertex_masses


def test_selected_middle_link_offset_is_not_instance_origin():
    root = np.array([[-0.024941666, 0, 0.210632, 0, 0, 0, 1]], dtype=np.float32)
    anchor = np.array([-1.471558333, 0, 0.22])
    same = map_physics_root(root, anchor, [1, 0, 0, 0], anchor, [1, 0, 0, 0])
    assert np.array_equal(same, root)
    moved = map_physics_root(root, anchor, [1, 0, 0, 0], anchor + [0.4, -0.2, 0.3], [1, 0, 0, 0])
    assert np.allclose(moved[0, :3], root[0, :3] + [0.4, -0.2, 0.3], atol=1e-7)
    assert np.allclose(moved[0, 3:], root[0, 3:])


def test_instance_rotation_rotates_the_root_offset():
    root = np.array([[2.0, 0, 1, 0, 0, 0, 1]])
    q = [2**-0.5, 0, 0, 2**-0.5]
    got = map_physics_root(root, [1, 0, 1], [1, 0, 0, 0], [3, 4, 5], q)
    assert np.allclose(got[0, :3], [3, 5, 5])
    assert np.allclose(got[0, 3:], [0, 0, 2**-0.5, 2**-0.5])
    assert np.array_equal(root, [[2, 0, 1, 0, 0, 0, 1]])
    with pytest.raises(ValueError):
        map_physics_root(root, [1, 0, 1], [0, 0, 0, 0], [3, 4, 5], q)


def test_runtime_defaults_preserve_explicit_layout_and_do_not_mutate(tmp_path):
    usd = tmp_path / "object.usdz"
    usd.touch()
    defaults = {"physics": {"reset_full_state": True, "mass": 0.1}, "visual": {"visible": True}}
    (tmp_path / "runtime_config.json").write_text(json.dumps(defaults))
    layout = {"physics": {"mass": 0.2}}
    got = with_asset_defaults(str(usd), layout)
    assert got == {"physics": {"mass": 0.2, "reset_full_state": True}, "visual": {"visible": True}}
    assert layout == {"physics": {"mass": 0.2}}
    assert with_asset_defaults(None, layout) is layout
    (tmp_path / "runtime_config.json").write_text('{"unexpected":1}')
    with pytest.raises(ValueError):
        with_asset_defaults(str(usd), layout)


def test_unequal_cloth_faces_distribute_mass_by_area():
    p = np.array([[0, 0, 0], [2, 0, 0], [0, 1, 0], [2, 3, 0]], float)
    f = np.array([[0, 1, 2], [1, 3, 2]])
    mass = area_vertex_masses(p, f, 0.4)
    assert np.allclose(mass, [1 / 30, 2 / 15, 2 / 15, 1 / 10])
    assert np.isclose(mass.sum(), 0.4)


def test_vertex_mass_loading_rejects_stale_or_wrong_particle_order(tmp_path):
    p = tmp_path / "masses.npy"
    np.save(p, np.array([0.1, 0.2]))
    cfg = {"file": p.name, "sha256": hashlib.sha256(p.read_bytes()).hexdigest(), "total_mass_kg": 0.3}
    assert np.allclose(load_vertex_masses(tmp_path / "object.usdz", cfg, 2), [0.1, 0.2])
    with pytest.raises(ValueError):
        load_vertex_masses(tmp_path / "object.usdz", cfg, 3)
    np.save(p, np.array([0.2, 0.1]))
    with pytest.raises(ValueError):
        load_vertex_masses(tmp_path / "object.usdz", cfg, 2)
