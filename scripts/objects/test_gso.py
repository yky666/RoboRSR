"""gso.py shoe pose on a synthetic shoe (no usd-core, no Isaac)."""

from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import gso  # noqa: E402


def shoe(length=0.28, heel_up=0.11, toe_up=0.06, yaw_deg=37.0):
    """Points of a shoe-like block: sole 0.1 wide, the heel end (+x before the turn) taller than the toe."""
    x = np.linspace(-length / 2, length / 2, 40)
    pts = [
        [xi, y, z] for xi in x for y in (-0.05, 0.05) for z in (0.0, toe_up + (heel_up - toe_up) * (xi > length / 4))
    ]
    t = np.radians(yaw_deg)
    rz = np.array([[np.cos(t), -np.sin(t), 0], [np.sin(t), np.cos(t), 0], [0, 0, 1]])
    return np.asarray(pts) @ rz.T


def test_shoe_pose():
    for yaw in (37.0, 200.0, -95.0):
        pts = shoe(yaw_deg=yaw)
        rot, s = gso.shoe_pose(pts)
        p = (pts @ rot.T) * s
        lo, hi = p.min(0), p.max(0)
        assert np.isclose(hi[0] - lo[0], gso.SHOE_LENGTH_M[1]) and hi[1] - lo[1] < 0.1  # long axis on x, scaled
        heel = p[p[:, 0] > hi[0] - 0.25 * (hi[0] - lo[0]), 2].max()
        toe = p[p[:, 0] < lo[0] + 0.25 * (hi[0] - lo[0]), 2].max()
        assert heel > toe  # the heel at +x, as every official Rigid/shoe
    assert gso.shoe_pose(shoe(heel_up=0.2)) is None  # a boot: too tall at the official length
    assert gso.shoe_pose(shoe(heel_up=0.04, toe_up=0.03)) is None  # a flip-flop: too flat
