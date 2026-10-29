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
IK_TRIES = 4  # IK solves per waypoint before a branch flip counts as unreachable
JOINT_LIMIT_RAD = np.array([10.0, 10.0, 10.0, 10.0, 10.0, 3.14])  # X5A.urdf joint1-6
TCP_M = 0.135  # link6 origin -> the point between the finger pads, along link6 x (the approach axis)


class Unreachable(RuntimeError):
    pass


def unwrap(q, seed):
    """`q` with each joint moved by whole turns to the value nearest `seed` that stays inside JOINT_LIMIT_RAD: the
    same link6 pose, without the full-turn flips the IK returns on the wide joints."""
    near = seed + (q - seed + math.pi) % (2 * math.pi) - math.pi
    return np.where(np.abs(near) <= JOINT_LIMIT_RAD[: len(q)], near, q)


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

    def solve(self, position, rotation, seed, tries=IK_TRIES):
        """Joints for link6 at (position, rotation) on the branch of `seed`: the closest of `tries` IK solves (cuRobo
        draws fresh seeds per solve) that stays within JUMP_RAD of it."""
        best = None
        for _ in range(tries):
            res = self.ik.solve_ik_to_joint(
                seed, list(position) + wxyz(rotation), real_robot_pose=list(self.robot.entity_origin_pose)
            )
            if res["status"] != "Success":
                raise Unreachable(f"{self.prefix} IK fails at {np.round(position, 3).tolist()}")
            q = unwrap(np.asarray(res["joint_value"], float).ravel(), seed)
            if best is None or np.abs(q - seed).max() < np.abs(best - seed).max():
                best = q
            if np.abs(best - seed).max() <= JUMP_RAD:
                return best
        raise Unreachable(f"{self.prefix} IK jumps {np.abs(best - seed).max():.2f} rad at {np.round(position, 3)}")

    def reachable(self, poses):
        """True when IK solves every (position, rotation) of `poses` in turn, each seeded by the last: a cheap check of
        a plan's key poses before planning the path between them."""
        q = self.q[-1]
        for position, rotation in poses:
            res = self.ik.solve_ik_to_joint(
                q, list(position) + wxyz(rotation), real_robot_pose=list(self.robot.entity_origin_pose)
            )
            if res["status"] != "Success":
                return False
            q = unwrap(np.asarray(res["joint_value"], float).ravel(), q)
        return True

    def snapshot(self):
        return list(self.q), list(self.grip), self.pose

    def restore(self, saved):
        self.q, self.grip, self.pose = list(saved[0]), list(saved[1]), saved[2]

    def matrix(self):
        """4x4 world pose of link6 at the end of the plan."""
        m = np.eye(4)
        m[:3, :3], m[:3, 3] = self.pose[1].as_matrix(), self.pose[0]
        return m

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

    def move_joints(self, position, rotation, tries=IK_TRIES):
        """Joint-space line to the IK solution for link6 at (`position`, `rotation`) nearest the current joints: for
        large re-orientations in free space, where a Cartesian line would pass a wrist singularity."""
        q0, best = self.q[-1], None
        for _ in range(tries):
            res = self.ik.solve_ik_to_joint(
                q0, list(position) + wxyz(rotation), real_robot_pose=list(self.robot.entity_origin_pose)
            )
            if res["status"] != "Success":
                raise Unreachable(f"{self.prefix} IK fails at {np.round(position, 3).tolist()}")
            q = unwrap(np.asarray(res["joint_value"], float).ravel(), q0)
            if best is None or np.abs(q - q0).max() < np.abs(best - q0).max():
                best = q
        self.home(best, (np.asarray(position, float), rotation))

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

    def pick(self, candidates, pre_m, lift_m, then=None):
        """Plan the first of `candidates` [(pinch point, rotation, tag)] whose approach (from `pre_m` back along the
        approach), close and straight lift by `lift_m` the IK follows, and for which `then(arm)` (the rest of the plan
        with the object rigid in the grip, planned and dropped) raises no Unreachable. Returns its tag; the plan ends
        at the lift. Raises Unreachable with the last reason."""
        saved, last = self.snapshot(), f"{self.prefix}: no pinch"
        for pinch, r, tag in candidates:
            approach = r.as_matrix()[:, 0]
            grasp = link6_for(pinch, r)
            if not self.reachable([(grasp - pre_m * approach, r), (grasp, r)]):
                last = f"{self.prefix} IK fails at pinch {tag}"
                continue
            try:
                self.move(grasp - pre_m * approach, r)
                self.move(grasp)
                self.gripper(CLOSED)
                self.move(grasp + np.r_[0.0, 0.0, lift_m])
                at_lift = self.snapshot()
                if then is not None:
                    then(self)
            except Unreachable as e:
                self.restore(saved)
                last = f"{e} (pinch {tag})"
                continue
            self.restore(at_lift)
            return tag
        raise Unreachable(last)

    def place(self, obj_now, obj_goal, carry_z, pre_m, home):
        """Plan carrying the held object (pose `obj_now`, 4x4) with link6 at least at `carry_z`, lowering it to
        `obj_goal`, letting go, backing off along the approach and up, and going to `home` (joints, link6 pose)."""
        link6 = held_goal(obj_now, self.matrix(), obj_goal)
        r = Rotation.from_matrix(link6[:3, :3])
        p0 = self.pose[0]
        z = max(p0[2], carry_z, link6[2, 3])
        self.move(np.r_[p0[:2], z])
        self.move(np.r_[link6[:2, 3], z], r)
        self.move(link6[:3, 3])
        self.gripper(OPEN)
        self.move(link6[:3, 3] - pre_m * link6[:3, 0] + np.r_[0.0, 0.0, 0.03])
        self.home(*home)


def task_checks(env):
    """The check list the task's run_reward hands to reward_manager.check, captured without registering it."""
    rm, captured = env.reward_manager, []
    check = rm.check
    rm.check = captured.append
    try:
        env.run_reward()
    finally:
        rm.check = check
    if len(captured) != 1:
        raise ValueError(f"{env.task_name}.run_reward registers {len(captured)} check lists, expected 1")
    return captured[0]


def task_holds(env, checks):
    """The task's success check now, evaluated as RewardManager does: every item, a nested list any of its items."""
    return bool(env.reward_manager.check_once(checks, 0, op="and"))


def task_report(env, checks):
    """Each top-level item of the task's check list now: [name, holds] (a nested list: any of its items)."""
    rm = env.reward_manager
    report = []
    for item in checks:
        name = item[0] if isinstance(item, tuple) else "any(" + ",".join(c[0] for c in item) + ")"
        if isinstance(item, tuple):
            args = {k: v for k, v in item[1].items() if isinstance(v, str | int | float)}
            name += str(args)
        report.append([name, bool(rm.check_once(item, 0, op="or" if isinstance(item, list) else "and"))])
    return report


def pose7(lm, label):
    """[x, y, z, qw, qx, qy, qz] of a labelled object now (layout frame)."""
    pose = lm.get_instance_pose(0, label)
    return np.concatenate([np.asarray(v.cpu() if hasattr(v, "cpu") else v, float).ravel() for v in pose])


def matrix(pose):
    """4x4 of [x, y, z, qw, qx, qy, qz]."""
    m = np.eye(4)
    m[:3, :3] = rot(pose[3:]).as_matrix()
    m[:3, 3] = pose[:3]
    return m


def held_goal(obj_now, link6_now, obj_goal):
    """link6 4x4 that puts a rigidly held object at `obj_goal` (4x4), as it is held now."""
    return obj_goal @ np.linalg.inv(obj_now) @ link6_now


def upright(T):
    """`T` (4x4) turned about its position until its z axis is vertical, keeping its heading about z."""
    z = T[:3, 2]
    axis = np.cross(z, [0.0, 0.0, 1.0])
    out = T.copy()
    if np.linalg.norm(axis) > 1e-9:
        tilt = Rotation.from_rotvec(axis / np.linalg.norm(axis) * np.arccos(np.clip(z[2], -1.0, 1.0)))
        out[:3, :3] = tilt.as_matrix() @ T[:3, :3]
    return out


def yawed(T, yaw):
    """`T` (4x4) with its heading about z replaced by `yaw` (rad), its tilt kept."""
    out = T.copy()
    x = T[:3, 0]
    out[:3, :3] = Rotation.from_euler("z", yaw - math.atan2(x[1], x[0])).as_matrix() @ T[:3, :3]
    return out


def box_of(lm, label, kind="Rigid"):
    """A labelled object as it stands now (layout frame), from its metadata's aligned bbox (the oriented one of some
    official objects is turned 45 deg about z): 4x4 pose, world bbox corners lo / hi, object-frame extents."""
    name = lm.get_instance_name(0, label)
    meta = lm.object_records_by_type[kind].metadata_by_env[0][name]
    scale = np.asarray(getattr(lm.get_scene_object(0, name), "scale", (1.0, 1.0, 1.0)), float)
    local = np.asarray(meta["geometry"]["aligned_bbox"]["vertices"], float).reshape(-1, 3) * scale
    T = matrix(pose7(lm, label))
    world = local @ T[:3, :3].T + T[:3, 3]
    return dict(T=T, lo=world.min(0), hi=world.max(0), extents=local.max(0) - local.min(0), local=local)


def actions(arms, moving, frames):
    """Action dicts for `frames` [(q, grip)] of the `moving` arm; the others hold their last planned state."""
    for q, g in frames:
        action = {}
        for arm in arms:
            qa, ga = (q, g) if arm is moving else (arm.q[-1], arm.grip[-1])
            action[f"{arm.prefix}_arm_joint_state"] = np.asarray(qa, float)
            action[f"{arm.prefix}_ee_joint_state"] = np.array([ga], float)
        yield action
