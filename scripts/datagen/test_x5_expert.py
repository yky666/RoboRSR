"""x5_expert.py grasp frames and the stack_bowls acceptance geometry (no Isaac)."""

from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from scripts.datagen import x5_expert as X  # noqa: E402
from scripts.datagen.plugins import (
    arrange_largest_number as N,  # noqa: E402
    hang_mugs as H,  # noqa: E402
    sort_nesting_dolls_by_size as D,  # noqa: E402
    stack_blocks as B,  # noqa: E402
    stack_bowls as S,  # noqa: E402
)


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


def test_unwrap():
    seed = np.array([0.1, -0.2, 3.0, 0.0, 0.5, 3.0])
    q = seed + np.array([0.05, 2 * np.pi - 0.1, -2 * np.pi, 0.2, 0.0, -2 * np.pi + 0.2])
    out = X.unwrap(q, seed)
    assert np.allclose(out[:5], seed[:5] + [0.05, -0.1, 0.0, 0.2, 0.0])
    assert np.isclose(out[5], q[5])  # 3.2 would leave joint6's +-3.14


def test_held_goal():
    obj = X.matrix(np.r_[0.1, -0.2, 0.8, X.wxyz(X.Rotation.from_euler("z", 30, degrees=True))])
    link6 = X.matrix(np.r_[0.1, -0.3, 0.95, X.wxyz(X.rim_pinch((0.0, 1.0), 60))])
    goal = obj.copy()
    goal[:3, 3] += [0.2, 0.1, 0.05]
    moved = X.held_goal(obj, link6, goal)
    assert np.allclose(moved[:3, :3], link6[:3, :3]) and np.allclose(moved[:3, 3], link6[:3, 3] + [0.2, 0.1, 0.05])
    turn = X.matrix(np.r_[0.1, -0.2, 0.8, X.wxyz(X.Rotation.from_euler("z", 120, degrees=True))])
    moved = X.held_goal(obj, link6, turn)  # the grip (object-to-link6) is kept
    assert np.allclose(np.linalg.inv(turn) @ moved, np.linalg.inv(obj) @ link6)


def test_stacked_blocks():
    up = [1.0, 0.0, 0.0, 0.0]
    poses = np.array([[0.0, 0.0, 0.78, *up], [0.01, 0.0, 0.815, *up], [0.005, 0.012, 0.85, *up]])
    assert B.stacked(poses, 0.035)
    assert B.stacked(poses[[2, 0, 1]], 0.035)  # bottom to top by height, whatever the label order
    assert not B.stacked(poses, 0.02)  # 1.2 cm off the one below: over the 1 cm threshold of a 2 cm cube
    flat = poses.copy()
    flat[:, 2] = 0.78
    assert not B.stacked(flat, 0.035)


def test_checked_hooks():
    def hook(x, z):
        x, z = np.asarray(x, float), np.asarray(z, float)
        m = np.eye(4)
        m[:3, 0], m[:3, 2] = x / np.linalg.norm(x), z / np.linalg.norm(z)
        m[:3, 1] = np.cross(m[:3, 2], m[:3, 0])
        return m

    # the official cup_holder: two hooks per height, +-x low and high, +-y in the middle (holder frame, yaw 30 deg)
    hooks = [
        hook([0.93, 0, 0.37], [-0.37, 0, 0.93]),
        hook([-0.91, 0, 0.41], [0.41, 0, 0.91]),
        hook([0, -0.93, 0.37], [0, 0.37, 0.93]),
        hook([0, 0.88, 0.47], [0, -0.47, 0.88]),
        hook([0.87, 0, 0.5], [-0.5, 0, 0.87]),
        hook([-0.93, 0, 0.37], [0.37, 0, 0.93]),
    ]
    yaw = np.eye(4)
    yaw[:3, :3] = X.Rotation.from_euler("z", 30, degrees=True).as_matrix()
    assert H.checked_hooks([yaw @ h for h in hooks]) == [0, 1, 4, 5]


def test_rim_points():
    lo, hi = np.array([-0.035, -0.035, 0.0]), np.array([0.07, 0.035, 0.1])  # body radius 3.5 cm, handle to +7 cm
    points = H.rim_points(lo, hi, hi[2], degrees=(180, 90))
    (p, inward, a), (q, _, b) = points
    assert (a, b) == (180, 90)
    assert np.allclose(p, [-0.035 + H.WALL_IN_M, 0.0, 0.1 - H.GRIP_DEPTH_M]) and np.allclose(inward, [1, 0, 0])
    assert np.allclose(q[:2], [0.0, 0.035 - H.WALL_IN_M])


def test_upright_and_yawed():
    tilted = X.matrix(np.r_[0.1, 0.2, 0.8, X.wxyz(X.Rotation.from_euler("zx", [40, 25], degrees=True))])
    up = X.upright(tilted)
    assert np.allclose(up[:3, 2], [0, 0, 1]) and np.allclose(up[:3, 3], tilted[:3, 3])
    assert np.isclose(np.degrees(np.arctan2(up[1, 0], up[0, 0])), 40)
    turned = X.yawed(up, np.radians(-10))
    assert np.allclose(turned[:3, 2], [0, 0, 1]) and np.isclose(np.degrees(np.arctan2(turned[1, 0], turned[0, 0])), -10)


def test_doll_row():
    assert D.row_order([3, 9, 5, 7, 6]) == ["doll1", "doll0", "doll3", "doll4", "doll2"]  # index mod 5, descending
    up = [1.0, 0.0, 0.0, 0.0]
    row = np.array([[x, -0.12 + 0.01 * (i % 2), 0.8, *up] for i, x in enumerate(D.SPOTS[:, 0])])
    assert D.in_row(row)
    assert not D.in_row(row[::-1])  # right to left
    off = row.copy()
    off[2, 1] -= 0.04
    assert not D.in_row(off)


def test_digits_laid():
    assert N.digit_order([2, 8, 7, 0]) == [1, 2, 0, 3]
    pad = np.array([0.1, -0.08])

    def pose(xy, yaw):
        return np.r_[xy, 0.78, X.wxyz(X.Rotation.from_euler("z", yaw, degrees=True))]

    assert N.laid(pose(pad + 0.01, 30), 7, pad) and not N.laid(pose(pad + 0.02, 0), 7, pad)
    assert not N.laid(pose(pad, 50), 7, pad) and not N.laid(pose(pad, 180), 7, pad)
    assert N.laid(pose(pad, 180), 0, pad)  # a 0 reads the same turned round
    assert not N.laid(pose(pad, 180), 8, pad)  # an 8 may not lie upside down (its y axis is checked)


def test_stacked():
    up = [1.0, 0.0, 0.0, 0.0]
    tilted = list(X.wxyz(X.Rotation.from_euler("x", 20, degrees=True)))
    poses = np.array([[0.0, 0.0, 0.79, *up], [0.01, 0.0, 0.81, *tilted], [0.0, 0.02, 0.83, *up]])
    assert S.stacked(poses)
    assert not S.stacked(poses + np.array([[0, 0, 0, 0, 0, 0, 0], [0.05, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0]]))
    low_tilted = poses.copy()
    low_tilted[0, 3:] = tilted
    assert not S.stacked(low_tilted)  # the lowest bowl must be within 7 deg
