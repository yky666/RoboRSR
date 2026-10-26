"""x5_expert.py grasp frames and the stack_bowls acceptance geometry (no Isaac)."""

from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from scripts.datagen import x5_expert as X  # noqa: E402
from scripts.datagen.plugins import stack_bowls as S  # noqa: E402


def test_rim_pinch():
    for inward, tilt in (((0.3, 0.9), 55), ((-1.0, 0.2), 75), ((0.0, 1.0), 90)):
        m = X.rim_pinch(inward, tilt).as_matrix()
        assert np.allclose(m.T @ m, np.eye(3), atol=1e-9) and np.isclose(np.linalg.det(m), 1)
        approach, closing = m[:, 0], m[:, 1]
        assert np.isclose(np.degrees(np.arcsin(-approach[2])), tilt)  # leans tilt below horizontal
        u = np.array([*inward, 0.0]) / np.hypot(*inward)
        assert approach[:2] @ u[:2] >= -1e-9 and abs(np.cross(approach, u) @ closing) < 1e-9  # in the vertical plane
    home = X.rot([0.707, 0.0, 0.0, 0.707]).as_matrix()  # the arms' start: fingers forward (+y), closing along -x
    assert np.allclose(home[:, 0], [0, 1, 0], atol=1e-3) and np.allclose(home[:, 1], [-1, 0, 0], atol=1e-3)


def test_wall_pinch():
    for inward, lean, side in (((0.3, 0.9), 60, 1), ((-1.0, 0.2), 45, -1), ((0.0, 1.0), 90, 1)):
        m = X.wall_pinch(inward, lean, side).as_matrix()
        assert np.allclose(m.T @ m, np.eye(3), atol=1e-9) and np.isclose(np.linalg.det(m), 1)
        u = np.array([*inward, 0.0]) / np.hypot(*inward)
        assert np.allclose(m[:, 1], u)  # closing horizontally across the wall
        assert np.isclose(np.degrees(np.arcsin(-m[2, 0])), lean) and abs(m[:, 0] @ u) < 1e-9


def test_link6_for():
    r = X.rim_pinch((0.0, 1.0), 45)
    pinch = np.array([0.3, -0.1, 0.8])
    link6 = X.link6_for(pinch, r)
    assert np.isclose(np.linalg.norm(pinch - link6), X.TCP_M) and link6[2] > pinch[2]  # the wrist sits above


def test_stacked():
    up = [1.0, 0.0, 0.0, 0.0]
    tilted = list(X.wxyz(X.Rotation.from_euler("x", 20, degrees=True)))
    poses = np.array([[0.0, 0.0, 0.79, *up], [0.01, 0.0, 0.81, *tilted], [0.0, 0.02, 0.83, *up]])
    assert S.stacked(poses)
    assert not S.stacked(poses + np.array([[0, 0, 0, 0, 0, 0, 0], [0.05, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0]]))
    low_tilted = poses.copy()
    low_tilted[0, 3:] = tilted
    assert not S.stacked(low_tilted)  # the lowest bowl must be within 7 deg
