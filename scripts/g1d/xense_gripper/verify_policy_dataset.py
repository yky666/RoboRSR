"""Independently load LeRobot observations and compare to lossless HDF5 source."""

import argparse
import json
from pathlib import Path
import subprocess

import h5py
from lerobot.datasets.lerobot_dataset import LeRobotDataset
import numpy as np


def verify(directory):
    data = LeRobotDataset("local/g1d_aurora_tactile", root=directory / "lerobot", video_backend="pyav")
    errors = []
    with h5py.File(directory / "episode.hdf5", "r") as h:
        ids = sorted(set([0, len(data) // 4, len(data) // 2, 3 * len(data) // 4, len(data) - 1]))
        for i in ids:
            row = data[i]
            if not np.array_equal(row["observation.state"].numpy(), h["observation/state"][i]):
                raise ValueError("State order/value changed during export")
            if not np.array_equal(row["action"].numpy(), h["action"][i]):
                raise ValueError("Action changed")
            if abs(float(row["timestamp"]) - i / 10) > 1e-5:
                raise ValueError("Relative timestamp changed")
            for j, ch in enumerate(json.loads(h.attrs["channels"])):
                image = row["observation.images.tactile_" + ch].numpy()
                if image.shape != (3, 700, 400) or not np.isfinite(image).all() or image.min() < 0 or image.max() > 1:
                    raise ValueError("Invalid policy image")
                errors.append(
                    float(abs(image.transpose(1, 2, 0) * 255 - h["tactile/fem_rgb"][i, j].astype(float)).mean())
                )
        count = len(h["frame_id"])
    videos = list((directory / "lerobot/videos").rglob("*.mp4"))
    if len(videos) != 4:
        raise ValueError("Expected four tactile videos")
    for p in videos:
        subprocess.run(["ffmpeg", "-v", "error", "-i", str(p), "-f", "null", "-"], check=True, capture_output=True)
    if len(data) != count or max(errors) > 8:
        raise ValueError("Frame count or image reproduction error")
    result = dict(
        passed=True,
        frames=len(data),
        sampled_policy_frames=ids,
        decoded_complete_videos=len(videos),
        max_mean_absolute_video_error_uint8=max(errors),
        reader="LeRobotDataset 0.4.4 / PyAV",
        observation_keys=list(data[0]),
        trained_policy_verified=False,
    )
    (directory / "policy_readback.json").write_text(json.dumps(result, indent=2) + "\n")
    return result


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("directory", type=Path)
    a = p.parse_args()
    print(json.dumps(verify(a.directory)))
