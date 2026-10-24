"""Regression tests for data integrity and offline identification, not calibration claims."""

import json
import threading
import time

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from env.robot_manager.aurora_contract import CHANNELS
from env.robot_manager.tactile_bridge import XenseTactileBridge, policy_observation
from scripts.g1d.xense_gripper.calibration_fit import friction, pivot_tcp, servo
from scripts.g1d.xense_gripper.tactile_worker import input_frames


def test_tcp_recovers_known_pivot_and_rejects_degenerate():
    rot = Rotation.from_euler("xyz", np.random.default_rng(9).uniform(-1, 1, (12, 3))).as_matrix()
    tcp = np.array([0.14, 0.002, -0.003])
    pivot = np.array([0.5, 0.2, 0.8])
    t = np.tile(np.eye(4), (12, 1, 1))
    t[:, :3, :3] = rot
    t[:, :3, 3] = pivot - rot @ tcp
    assert np.allclose(pivot_tcp(t)["tcp_in_wrist_m"], tcp, atol=1e-12)
    with pytest.raises(ValueError):
        pivot_tcp(np.tile(t[0], (8, 1, 1)))


def test_force_units_and_servo():
    assert friction([[2, 1], [4, 2], [6, 3]])["static_friction"] == pytest.approx(0.5)
    assert servo([[i * 0.1, i * 0.001, 0.01] for i in range(6)])["peak_opening_speed_m_s"] == pytest.approx(0.01)
    with pytest.raises(ValueError):
        friction([[0, 1], [2, 1], [3, 1]])


def test_four_camera_policy_requires_exact_shape():
    pkt = {"fem_rgb": np.zeros((4, 700, 400, 3), dtype=np.uint8)}
    result = policy_observation(pkt, np.zeros(16))
    assert len(result) == 5 and result["observation.images.tactile_right_negative"].shape == (3, 700, 400)
    with pytest.raises(ValueError):
        policy_observation({"fem_rgb": pkt["fem_rgb"][:3]}, np.zeros(16))


def test_live_bridge_waits_and_rejects_stale(tmp_path):
    (tmp_path / "packets").mkdir()
    (tmp_path / "tactile_contract.json").write_text(json.dumps({"channels": CHANNELS}))

    def publish():
        time.sleep(0.04)
        np.savez(tmp_path / "packets/00001.npz", frame_id=1, simulation_time_s=0.1)

    thread = threading.Thread(target=publish)
    thread.start()
    b = XenseTactileBridge(tmp_path)
    assert int(b.wait_observation(1, 0.1, timeout_s=1)["frame_id"]) == 1
    thread.join()
    with pytest.raises(ValueError):
        b.observation(1, 0.2)
    with pytest.raises(TimeoutError):
        b.wait_observation(2, 0.2, timeout_s=0.02)


def test_follow_stream_refuses_partial_or_missing_frames(tmp_path):
    (tmp_path / "frames.jsonl").write_text('{"frame_id":0}\n')
    (tmp_path / "recording_complete.json").write_text('{"frames":1}')
    assert list(input_frames(tmp_path, True)) == [{"frame_id": 0}]
    (tmp_path / "recording_complete.json").write_text('{"frames":2}')
    with pytest.raises(ValueError):
        list(input_frames(tmp_path, True))
