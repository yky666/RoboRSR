"""Offline identification from explicitly supplied measurements; no robot I/O.

Input JSON must include hardware_id and mount_id. Results are candidates until
independent held-out measurements are checked. Synthetic tests are not hardware
calibration. SI units throughout, except explicitly named encoder/model radians.
"""

import argparse
import json
from pathlib import Path

import numpy as np


def pivot_tcp(transforms):
    ts = np.asarray(transforms, dtype=float)
    if ts.ndim != 3 or ts.shape[1:] != (4, 4) or len(ts) < 6 or not np.isfinite(ts).all():
        raise ValueError("At least six finite wrist-to-world transforms required")
    if not np.allclose(ts[:, 3, :], [0, 0, 0, 1]) or not np.allclose(
        ts[:, :3, :3].transpose(0, 2, 1) @ ts[:, :3, :3], np.eye(3), atol=1e-5
    ):
        raise ValueError("Invalid rigid transform")
    a = np.concatenate([np.c_[t[:3, :3], -np.eye(3)] for t in ts])
    b = -ts[:, :3, 3].reshape(-1)
    x, _, rank, s = np.linalg.lstsq(a, b, rcond=None)
    if rank < 6 or s[0] / s[-1] > 1e4:
        raise ValueError("Insufficient orientation diversity for pivot TCP")
    residual = (a @ x - b).reshape(-1, 3)
    return dict(
        tcp_in_wrist_m=x[:3].tolist(),
        pivot_world_m=x[3:].tolist(),
        rms_m=float(np.sqrt(np.mean(np.sum(residual**2, axis=1)))),
        condition=float(s[0] / s[-1]),
    )


def friction(samples):
    x = np.asarray(samples, dtype=float)
    if (
        x.ndim != 2
        or x.shape[1] != 2
        or len(x) < 3
        or np.any(x[:, 0] <= 0)
        or np.any(x[:, 1] < 0)
        or not np.isfinite(x).all()
    ):
        raise ValueError("Need >=3 [total_normal_N, tangential_N_at_slip] rows")
    mu = float(x[:, 0] @ x[:, 1] / (x[:, 0] @ x[:, 0]))
    return dict(
        static_friction=mu,
        rms_n=float(np.sqrt(np.mean((x[:, 1] - mu * x[:, 0]) ** 2))),
        interpretation="slope at independently detected incipient slip; not gel Young modulus",
    )


def servo(samples):
    x = np.asarray(samples, dtype=float)
    if x.ndim != 2 or x.shape[1] != 3 or len(x) < 5 or not np.isfinite(x).all():
        raise ValueError("Need >=5 [time_s, measured_gap_m, target_gap_m] rows")
    dt = np.diff(x[:, 0])
    if np.any(dt <= 0):
        raise ValueError("Time must increase")
    speed = np.diff(x[:, 1]) / dt
    return dict(
        peak_opening_speed_m_s=float(abs(speed).max()),
        rms_tracking_error_m=float(np.sqrt(np.mean((x[:, 1] - x[:, 2]) ** 2))),
        final_error_m=float(x[-1, 1] - x[-1, 2]),
        identified_gain=False,
    )


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("input", type=Path)
    p.add_argument("--out", type=Path, required=True)
    a = p.parse_args()
    s = json.loads(a.input.read_text())
    if not s.get("hardware_id") or not s.get("mount_id"):
        raise ValueError("Hardware and mount identity required")
    fit = {"pivot_tcp": pivot_tcp, "friction": friction, "servo": servo}[s["kind"]](s["samples"])
    a.out.write_text(
        json.dumps(
            dict(
                status="candidate_requires_holdout",
                hardware_id=s["hardware_id"],
                mount_id=s["mount_id"],
                input_kind=s["kind"],
                result=fit,
                hardware_connected=False,
            ),
            indent=2,
        )
        + "\n"
    )
