"""Motion for the scripted dual ARX X5 experts (scripts/datagen/plugins/<benchmark task>.py): privileged object poses,
Cartesian waypoints of each arm's link6 (the ee_pose of the observation), solved into joint actions with the env's
cuRobo IK, each solution seeded from the previous one so the arm stays on one branch.

Pace follows the official RoboDojo demos (HF RoboDojo-Benchmark, 25 Hz): link6 moves at most STEP_M per action, the
gripper action changes GRIP_STEP per action, one arm moves while the other holds, both start and end at the joint
zeros. A segment the IK cannot follow raises Unreachable; plugins plan one phase at a time from the live poses.
"""

import math

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

STEP_M = 0.006  # link6 travel per action: the official demos' median while moving
STEP_RAD = 0.04  # link6 rotation per action
JOINT_STEP_RAD = 0.03  # per joint per action on joint-space moves (home)
GRIP_STEP = 0.1  # gripper action per action, as the official demos ramp it
OPEN, CLOSED = 1.0, 0.0
JUMP_RAD = 0.35  # largest joint change between two consecutive IK solutions before it counts as a branch flip
TCP_M = 0.135  # link6 origin -> the point between the finger pads, along link6 x (the approach axis)


class Unreachable(RuntimeError):
    pass


def rot(wxyz):
    w, x, y, z = wxyz
    return Rotation.from_quat([x, y, z, w])


def wxyz(rotation):
    x, y, z, w = rotation.as_quat()
    return [float(w), float(x), float(y), float(z)]


def pinch_rotation(approach, closing):
    """link6 orientation with its x axis along `approach` (the way the fingers point) and the fingers closing along
    `closing` (link6 y; made orthogonal to the approach)."""
    x = np.asarray(approach, float) / np.linalg.norm(approach)
    y = np.asarray(closing, float) - np.dot(closing, x) * x
    y /= np.linalg.norm(y)
    return Rotation.from_matrix(np.stack([x, y, np.cross(x, y)], axis=1))


def rim_pinch(inward, tilt_deg):
    """Approach leaning `tilt_deg` below horizontal towards `inward` (horizontal), fingers closing in the vertical
    plane of the approach: they straddle a vertical wall facing `inward`."""
    u = np.array([inward[0], inward[1], 0.0]) / np.hypot(inward[0], inward[1])
    t = math.radians(tilt_deg)
    up = np.array([0.0, 0.0, 1.0])
    return pinch_rotation(math.cos(t) * u - math.sin(t) * up, math.sin(t) * u + math.cos(t) * up)


def wall_pinch(inward, lean_deg, side=1):
    """Fingers closing horizontally along `inward` across a wall facing it (no lever on the wall when they squeeze),
    the approach leaning `lean_deg` below horizontal along the wall's tangent (`side` picks which way)."""
    u = np.array([inward[0], inward[1], 0.0]) / np.hypot(inward[0], inward[1])
    t = math.radians(lean_deg)
    up = np.array([0.0, 0.0, 1.0])
    tangent = side * np.cross(up, u)
    return pinch_rotation(math.cos(t) * tangent - math.sin(t) * up, u)


def link6_for(pinch_point, rotation):
    """link6 position that puts the finger pads' midpoint on `pinch_point`."""
    return np.asarray(pinch_point, float) - TCP_M * rotation.as_matrix()[:, 0]


class Arm:
    """One arm's planned joints and gripper, appended segment by segment from its live state."""

    def __init__(self, env, prefix):
        rm = env.robot_manager
        self.prefix, self.rm = prefix, rm
        self.robot = next(r for r in rm.robot_list if r.type == "target" and r.arm_name.startswith(prefix))
        self.ik = rm.ik_solver[self.robot.robot_name]
        state = env.get_obs()["state"]
        self.q = [np.asarray(state[f"{prefix}_arm_joint_state"], float).ravel()]
        self.grip = [float(np.asarray(state[f"{prefix}_ee_joint_state"]).ravel()[0])]
        p = np.asarray(state[f"{prefix}_ee_pose"], float).ravel()
        self.pose = (p[:3], rot(p[3:]))

    def solve(self, position, rotation, seed):
        res = self.ik.solve_ik_to_joint(
            seed, list(position) + wxyz(rotation), real_robot_pose=list(self.robot.entity_origin_pose)
        )
        if res["status"] != "Success":
            raise Unreachable(f"{self.prefix} IK fails at {np.round(position, 3).tolist()}")
        q = np.asarray(res["joint_value"], float).ravel()
        if np.abs(q - seed).max() > JUMP_RAD:
            raise Unreachable(f"{self.prefix} IK jumps {np.abs(q - seed).max():.2f} rad at {np.round(position, 3)}")
        return q

    def move(self, position, rotation=None, step_m=STEP_M):
        """Straight line of link6 to `position` (and `rotation`), the orientation slerped along."""
        p0, r0 = self.pose
        p1, r1 = np.asarray(position, float), rotation if rotation is not None else r0
        angle = (r0.inv() * r1).magnitude()
        n = max(1, math.ceil(np.linalg.norm(p1 - p0) / step_m), math.ceil(angle / STEP_RAD))
        slerp = Slerp([0, 1], Rotation.concatenate([r0, r1]))
        q = self.q[-1]
        for i in range(1, n + 1):
            s = i / n
            q = self.solve(p0 + (p1 - p0) * s, slerp(s), q)
            self.q.append(q)
            self.grip.append(self.grip[-1])
        self.pose = (p1, r1)

    def gripper(self, value, settle=8):
        """Ramp the gripper action to `value`, then hold `settle` actions for the fingers to arrive."""
        g = self.grip[-1]
        while abs(value - g) > 1e-9:
            g += float(np.clip(value - g, -GRIP_STEP, GRIP_STEP))
            self.q.append(self.q[-1])
            self.grip.append(g)
        self.wait(settle)

    def wait(self, n):
        self.q.extend([self.q[-1]] * n)
        self.grip.extend([self.grip[-1]] * n)

    def home(self, q_home, pose_home):
        """Joint-space line to `q_home` (the start joints); link6 ends on `pose_home`."""
        q0, q1 = self.q[-1], np.asarray(q_home, float)
        n = max(1, math.ceil(np.abs(q1 - q0).max() / JOINT_STEP_RAD))
        for i in range(1, n + 1):
            self.q.append(q0 + (q1 - q0) * i / n)
            self.grip.append(self.grip[-1])
        self.pose = pose_home

    def take(self):
        """The planned actions since the last take, and keep only the last as the start of the next segment."""
        out = list(zip(self.q[1:], self.grip[1:]))
        self.q, self.grip = self.q[-1:], self.grip[-1:]
        return out


def actions(arms, moving, frames):
    """Action dicts for `frames` [(q, grip)] of the `moving` arm; the others hold their last planned state."""
    for q, g in frames:
        action = {}
        for arm in arms:
            qa, ga = (q, g) if arm is moving else (arm.q[-1], arm.grip[-1])
            action[f"{arm.prefix}_arm_joint_state"] = np.asarray(qa, float)
            action[f"{arm.prefix}_ee_joint_state"] = np.array([ga], float)
        yield action
