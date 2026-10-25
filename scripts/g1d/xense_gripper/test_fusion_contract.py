"""Independent analytic fixtures and observation association checks (no GPU/SDK)."""

import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from env.robot_manager.aurora_contract import CHANNELS, mjcf_closure_positions, urdf_jaw_positions
from env.robot_manager.tactile_bridge import XenseTactileBridge
from env.robot_manager.tactile_geometry import depth_from_scene


class FusionContractTests(unittest.TestCase):
    def test_no_contact_and_front_plane(self):
        depth = depth_from_scene(np.eye(4), {}, {})
        self.assertFalse(np.any(depth))
        self.assertTrue(np.all(depth_from_scene(np.eye(4), {}, {}, include_clearance=True) == 5.0))
        obj = {"p": dict(pose_wxyz=[0, 0, 0.0018, 1, 0, 0, 0])}
        geo = {"p": dict(kind="box", size_m=[0.010, 0.012, 0.004])}
        depth = depth_from_scene(np.eye(4), obj, geo)
        self.assertAlmostEqual(float(depth.min()), -0.2, places=5)
        self.assertTrue(3900 < np.count_nonzero(depth < 0) < 4400)

    def test_sphere_sign_and_lateral_alignment(self):
        obj = {"p": dict(pose_wxyz=[0.004, 0, 0.003, 1, 0, 0, 0])}
        depth = depth_from_scene(np.eye(4), obj, {"p": dict(kind="sphere", radius_m=0.004)})
        self.assertTrue(-1.001 < depth.min() < -0.99)
        rows, cols = np.nonzero(depth < 0)
        self.assertTrue(71 < cols.mean() < 75)
        self.assertAlmostEqual(float(rows.mean()), 87.0, delta=1.0)

    def test_covariance_under_world_rotation_and_translation(self):
        pad = np.array([[0, 0, 1, 0.3], [1, 0, 0, 0.4], [0, 1, 0, 0.5], [0, 0, 0, 1.0]])
        # 120-degree rotation about (1,1,1), quaternion (.5,.5,.5,.5).
        world = pad[:3, 3] + pad[:3, :3] @ np.array([0, 0, 0.0018])
        obj = {"p": dict(pose_wxyz=[*world, 0.5, 0.5, 0.5, 0.5])}
        geo = {"p": dict(kind="box", size_m=[0.010, 0.012, 0.004])}
        moved = depth_from_scene(pad, obj, geo)
        reference = depth_from_scene(np.eye(4), {"p": dict(pose_wxyz=[0, 0, 0.0018, 1, 0, 0, 0])}, geo)
        np.testing.assert_allclose(moved, reference, atol=1e-6)

    def test_invalid_openings_rejected(self):
        for value in [-0.001, 0.086, float("nan"), float("inf")]:
            for conversion in (urdf_jaw_positions, mjcf_closure_positions):
                with self.assertRaises(ValueError):
                    conversion(value)

    def test_stale_observation_rejected(self):
        with tempfile.TemporaryDirectory() as d:
            root = Path(d)
            (root / "packets").mkdir()
            (root / "tactile_contract.json").write_text(json.dumps(dict(channels=CHANNELS)))
            np.savez(root / "packets/00003.npz", frame_id=3, simulation_time_s=0.5, depth_mm=np.zeros((4, 175, 100)))
            reader = XenseTactileBridge(root)
            self.assertEqual(int(reader.observation(3, 0.5)["frame_id"]), 3)
            with self.assertRaises(ValueError):
                reader.observation(3, 0.501)


if __name__ == "__main__":
    unittest.main()
