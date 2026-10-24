"""Export aligned simulation/tactile episodes and verify their policy inputs.

This opt-in tactile schema is separate from RoboRSR's three-RGB benchmark
converter. It never relabels tactile images as physical wrist cameras.
"""

import argparse
import hashlib
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
from env.robot_manager.aurora_contract import CHANNELS  # noqa: E402
from env.robot_manager.tactile_bridge import XenseTactileBridge, policy_observation  # noqa: E402


def state_vector(frame, field):
    joints = dict(zip(frame["joint_names"], frame[field]))
    result = []
    for side in ("left", "right"):
        result += [
            joints[f"{side}_{j}_joint"]
            for j in (
                "shoulder_pitch",
                "shoulder_roll",
                "shoulder_yaw",
                "elbow",
                "wrist_roll",
                "wrist_pitch",
                "wrist_yaw",
            )
        ]
        result.append(sum(joints[f"{side}_aurora_{s}_jaw_joint"] for s in ("positive", "negative")) / 0.085)
    return np.array(result, dtype=np.float32)


def export(recording, tactile, output, lerobot=False):
    import h5py

    output.mkdir(parents=True, exist_ok=False)
    frames = [json.loads(line) for line in (recording / "frames.jsonl").read_text().splitlines()]
    scene = json.loads((recording / "scene_schema.json").read_text())
    source_digest = hashlib.sha256((recording / "frames.jsonl").read_bytes()).hexdigest()
    tactile_report = json.loads((tactile / "report.json").read_text())
    if tactile_report.get("status") != "completed" or tactile_report.get("source_frames_sha256") != source_digest:
        raise ValueError("Tactile output and robot trajectory have different source recordings")
    bridge = XenseTactileBridge(tactile)
    t = np.array([f["simulation_time_s"] for f in frames])
    if len(t) < 2 or np.any(np.diff(t) <= 0) or not np.allclose(np.diff(t), 1 / scene["fps"], atol=1e-5):
        raise ValueError("Non-uniform or non-monotonic frame times")
    state = np.stack([state_vector(f, "joint_pos_m_or_rad") for f in frames])
    action = np.stack([state_vector(f, "target_m_or_rad") for f in frames])
    if not scene.get("source_frames", "").startswith("pre-action"):
        raise ValueError("This exporter requires verified pre-action samples")
    dataset = None
    if lerobot:
        from lerobot.datasets.lerobot_dataset import LeRobotDataset

        features = {
            k: dict(dtype="float32", shape=(16,), names=[f"joint_{i}" for i in range(16)])
            for k in ("observation.state", "action")
        }
        for ch in CHANNELS:
            features["observation.images.tactile_" + ch] = dict(
                dtype="video", shape=(700, 400, 3), names=["height", "width", "channels"]
            )
        features["observation.tactile.native_normal_force_w_n"] = dict(dtype="float32", shape=(12,), names=None)
        dataset = LeRobotDataset.create(
            repo_id="local/g1d_aurora_tactile",
            root=output / "lerobot",
            fps=scene["fps"],
            robot_type="g1d_aurora_lite_ws",
            features=features,
            use_videos=True,
            vcodec="libsvtav1",
            encoder_threads=2,
        )
    packet_hashes = []
    with h5py.File(output / "episode.hdf5", "x") as h:
        h.attrs["schema"] = "g1d_four_pad_episode.v1"
        h.attrs["frame_alignment"] = "observation[t] precedes action[t]"
        h.attrs["instruction"] = scene["instruction"]
        h.attrs["channels"] = json.dumps(CHANNELS)
        h.create_dataset("frame_id", data=[f["frame_id"] for f in frames])
        h.create_dataset("simulation_time_s", data=t)
        h.create_dataset("observation/state", data=state)
        h.create_dataset("action", data=action)
        for i, f in enumerate(frames):
            packet = bridge.observation(f["frame_id"], f["simulation_time_s"])
            policy = policy_observation(packet, state[i])
            for key, value in packet.items():
                if key in ("frame_id", "simulation_time_s"):
                    continue
                path = "tactile/" + key
                if i == 0:
                    h.create_dataset(
                        path,
                        shape=(len(frames), *value.shape),
                        dtype=value.dtype,
                        chunks=(1, *value.shape),
                        compression="gzip",
                        compression_opts=1,
                    )
                h[path][i] = value
            path = tactile / "packets" / f"{f['frame_id']:05d}.npz"
            packet_hashes.append(hashlib.sha256(path.read_bytes()).hexdigest())
            if dataset is not None:
                # LeRobot writer takes HWC uint8, reader returns normalized CHW.
                row = {
                    k: packet["fem_rgb"][j]
                    for j, k in enumerate(k for k in policy if k.startswith("observation.images."))
                }
                row.update(
                    {
                        "observation.state": state[i],
                        "action": action[i],
                        "task": scene["instruction"],
                        "observation.tactile.native_normal_force_w_n": packet["native_normal_force_w_n"]
                        .reshape(-1)
                        .astype(np.float32),
                    }
                )
                dataset.add_frame(row)
    if dataset is not None:
        dataset.save_episode()
        dataset.finalize()
    report = dict(
        schema="g1d_four_pad_episode.v1",
        frames=len(frames),
        channels=CHANNELS,
        source_frames_sha256=source_digest,
        source_packets_sha256=packet_hashes,
        simulation_time_origin_s=float(t[0]),
        lerobot_timestamp="relative to first frame; absolute physics time in HDF5",
        lerobot_exported=lerobot,
        hardware_connected=False,
        training_compatibility="new four-camera policy configuration required; no existing policy weights validated",
        source_tactile_contract=bridge.contract,
    )
    (output / "manifest.json").write_text(json.dumps(report, indent=2) + "\n")
    return audit(output)


def audit(output):
    import h5py

    manifest = json.loads((output / "manifest.json").read_text())
    with h5py.File(output / "episode.hdf5", "r") as h:
        n = len(h["frame_id"])
        if not np.array_equal(h["frame_id"][:], np.arange(n)):
            raise ValueError("Non-contiguous IDs")
        if len(h["simulation_time_s"]) != n or np.any(np.diff(h["simulation_time_s"][:]) <= 0):
            raise ValueError("Bad times")
        if h["observation/state"].shape != (n, 16) or h["action"].shape != (n, 16):
            raise ValueError("Bad joint order/shape")
        checked = []

        def visit(name, value):
            if isinstance(value, h5py.Dataset):
                for start in range(0, n, 8):
                    if not np.isfinite(value[start : start + 8]).all():
                        raise ValueError("Nonfinite " + name)
                if len(value) != n:
                    raise ValueError("Truncated " + name)
                checked.append(name)

        h.visititems(visit)
        for key in ("fem_rgb", "simple_rgb"):
            if h["tactile/" + key].shape != (n, 4, 700, 400, 3):
                raise ValueError("Missing camera")
    result = dict(passed=n == manifest["frames"], frames=n, checked=checked, independent_readback=True)
    (output / "audit.json").write_text(json.dumps(result, indent=2) + "\n")
    return result


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--recording", type=Path)
    p.add_argument("--tactile", type=Path)
    p.add_argument("--out", type=Path, required=True)
    p.add_argument("--lerobot", action="store_true")
    p.add_argument("--audit-only", action="store_true")
    a = p.parse_args()
    print(json.dumps(audit(a.out) if a.audit_only else export(a.recording, a.tactile, a.out, a.lerobot)))
