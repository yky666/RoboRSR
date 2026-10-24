"""Isolated simulated whole-body and jaw motion probe. Does not connect to real hardware."""

import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--out", required=True)
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
args.headless = True
args.enable_cameras = True
app = AppLauncher(args).app

import hashlib  # noqa: E402
import json  # noqa: E402
import os  # noqa: E402
from pathlib import Path  # noqa: E402
import sys  # noqa: E402
import traceback  # noqa: E402

from isaaclab.assets import Articulation  # noqa: E402
import isaaclab.sim as sim_utils  # noqa: E402
from isaacsim.sensors.camera import Camera  # noqa: E402
import numpy as np  # noqa: E402
from PIL import Image  # noqa: E402
import torch  # noqa: E402
from transforms3d.quaternions import mat2quat  # noqa: E402

from env.robot_manager.robot_config.g1d_aurora_lite_ws import get_robot_config  # noqa: E402


def look_at(eye, target):
    forward = (target - eye) / np.linalg.norm(target - eye)
    left = np.cross([0.0, 0.0, 1.0], forward)
    left /= np.linalg.norm(left)
    return mat2quat(np.stack([forward, left, np.cross(forward, left)], axis=1))


def main():
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=False)
    sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=1 / 120, device=args.device))
    sim_utils.DomeLightCfg(intensity=1200).func("/World/light", sim_utils.DomeLightCfg(intensity=1200))
    config = get_robot_config()
    config.prim_path = "/World/Robot"
    robot = Articulation(config)
    camera = Camera("/World/overview", resolution=(1280, 960))
    sim.reset()
    robot.update(sim.get_physics_dt())
    camera.initialize()
    # Keep the sensor's default lens and frame the complete body.
    camera.set_world_pose(
        np.array([5.0, -5.0, 3.5]), look_at(np.array([5.0, -5.0, 3.5]), np.array([0.1, 0.0, 0.7])), camera_axes="world"
    )
    target = robot.data.default_joint_pos.clone()
    robot.write_joint_state_to_sim(target, torch.zeros_like(target))
    names = robot.joint_names
    required = [
        f"{side}_{part}_joint"
        for side in ("left", "right")
        for part in (
            "shoulder_pitch",
            "shoulder_roll",
            "shoulder_yaw",
            "elbow",
            "wrist_roll",
            "wrist_pitch",
            "wrist_yaw",
        )
    ]
    required += ["torso_Joint", "Yaw_Joint", "LZ_mt_Joint", "LZ_it_Joint"]
    jaws = [f"{side}_aurora_{sign}_jaw_joint" for side in ("left", "right") for sign in ("positive", "negative")]
    if set(required + jaws) - set(names):
        raise ValueError(f"Missing imported DOF: {set(required + jaws) - set(names)}")
    rows, issues = [], []

    def settle(label, steps=240):
        for _ in range(steps):
            robot.set_joint_position_target(target)
            robot.write_data_to_sim()
            sim.step()
            robot.update(sim.get_physics_dt())
        actual = robot.data.joint_pos[0].cpu().numpy()
        expected = target[0].cpu().numpy()
        error = np.abs(actual - expected)
        if not np.isfinite(actual).all():
            issues.append(label + ":nonfinite_joint_state")
        for i, name in enumerate(names):
            tolerance = 0.002 if name in jaws or name.startswith("LZ_") else 0.04
            if error[i] > tolerance:
                issues.append(f"{label}:{name}:tracking_error={error[i]:.6f}")
        rows.append(
            {"step": label, "target": expected.tolist(), "measured": actual.tolist(), "max_error": float(error.max())}
        )
        for _ in range(8):
            sim.render()
        Image.fromarray(camera.get_rgba()[:, :, :3]).save(out / f"{label}.png")

    settle("initial_open")
    for label, opening in (("closed", 0.0), ("half_open", 0.0425), ("full_open", 0.085)):
        for name in jaws:
            target[0, names.index(name)] = opening / 2
        settle(label)
    initial = target.clone()
    for name in required:
        index = names.index(name)
        target[0, index] += 0.015 if name.startswith("LZ_") else 0.10
        settle("move_" + name, steps=120)
        target.copy_(initial)
        settle("return_" + name, steps=120)
    model = Path(config.spawn.asset_path)
    report = {
        "status": "passed" if not issues else "failed",
        "kind": "isolated Isaac Sim robot joint-motion probe",
        "source_urdf_sha256": hashlib.sha256(model.read_bytes()).hexdigest(),
        "joint_names": names,
        "rows": rows,
        "issues": issues,
        "hardware_connected": False,
        "contact_calibrated": False,
        "tactile_response_simulated": False,
    }
    (out / "joint_probe.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({"status": report["status"], "issues": issues, "out": str(out)}), flush=True)
    return not issues


try:
    passed = main()
except Exception:
    traceback.print_exc()
    passed = False
sys.stdout.flush()
sys.stderr.flush()
os._exit(0 if passed else 1)
