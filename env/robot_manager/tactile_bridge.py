"""Optional recorded/streamed tactile observation interface; no vendor imports.

The separate Python 3.10 worker publishes immutable per-frame NPZ packets. A
consumer must match both frame ID and physics timestamp; stale data is rejected.
Existing robot control and 16-dimensional actions are unchanged.
"""

import json
from pathlib import Path
import time

import numpy as np

from env.robot_manager.aurora_contract import CHANNELS


class XenseTactileBridge:
    def __init__(self, directory):
        self.directory = Path(directory)
        self.contract = json.loads((self.directory / "tactile_contract.json").read_text())
        if tuple(self.contract["channels"]) != CHANNELS:
            raise ValueError("Tactile channel order mismatch")

    def observation(self, frame_id, simulation_time_s):
        """Return a packet for exactly one physics state; never reuse an old frame."""
        path = self.directory / "packets" / f"{int(frame_id):05d}.npz"
        with np.load(path, allow_pickle=False) as packet:
            result = {key: packet[key].copy() for key in packet.files}
        if int(result["frame_id"]) != frame_id or abs(float(result["simulation_time_s"]) - simulation_time_s) > 1e-8:
            raise ValueError("Tactile frame/timestamp does not match robot state")
        for name, value in result.items():
            if value.dtype.kind in "fciu" and not np.isfinite(value).all():
                raise ValueError(f"Non-finite tactile field: {name}")
        return result

    def wait_observation(self, frame_id, simulation_time_s, timeout_s=30):
        """Bounded lockstep wait; missing data never becomes an all-zero image."""
        deadline = time.monotonic() + timeout_s
        while True:
            try:
                return self.observation(frame_id, simulation_time_s)
            except FileNotFoundError:
                if time.monotonic() >= deadline:
                    raise TimeoutError(f"Tactile packet {frame_id} was not published") from None
                time.sleep(0.02)


def policy_observation(packet, state, *, image_kind="fem_rgb"):
    """Explicit four-camera policy input. Does not alias tactile to wrist RGB."""
    state = np.asarray(state, dtype=np.float32)
    if state.shape != (16,) or not np.isfinite(state).all():
        raise ValueError("Expected finite 16-dimensional G1-D state")
    if image_kind not in ("fem_rgb", "simple_rgb"):
        raise ValueError("Unknown tactile renderer")
    images = packet[image_kind]
    if images.shape != (4, 700, 400, 3) or images.dtype != np.uint8:
        raise ValueError("Missing or incorrectly shaped four-channel tactile images")
    result = {"observation.state": state.copy()}
    for i, channel in enumerate(CHANNELS):
        result[f"observation.images.tactile_{channel}"] = images[i].transpose(2, 0, 1).astype(np.float32) / 255
    return result
