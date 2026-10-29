"""G1-D grasp test of the FEM paper cup Dynamic/paper_cup/00000 (scene object type `deformable`).

The scripted expert's side grasp around a cup body (scripts/g1d/scripted_demo.cup_grasp: fingers 35 deg down, wrist
turned inward, pads closing across the body) at CUP_GRIP_UP above the cup bottom, where the cone is narrower than
the 95 mm full pad opening; close, lift LIFT, hold. Logs the cup from its FEM nodes every few actions: bottom height,
cross-section at the grasp height (min / max width), the nodal pose, minimum element Jacobian, the pad contact
forces (obs gripper_force sensor) and the wall clock per action. Writes CUP_REPORT (json).

Environment: CUP_GRIP_UP (m, default 0.045), CUP_CLOSE ("full" = command 0, the Dex1-1 force limit, or a pad gap in
mm to close to), CUP_REPORT. Layout: a g1d_general_pickup layout whose target is the paper cup (see HANDOFF.md).
    CUDA_VISIBLE_DEVICES=0 PYTHONPATH=.:XPolicyLab EVAL_NUM=1 python scripts/objects/grasp_paper_cup_fem.py \
        --env_cfg_type g1d --task_name g1d_general_pickup --seed 90
"""

import json
import os
from pathlib import Path
import sys
import time

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from scripts.g1d import scripted_demo as demo  # noqa: E402

GRIP_UP = float(os.environ.get("CUP_GRIP_UP", 0.045))
CLOSE = os.environ.get("CUP_CLOSE", "full")
REPORT = Path(os.environ.get("CUP_REPORT", "paper_cup_grasp.json"))
LIFT, HOLD = 0.10, 60


def cup_state(cup, arm, env):
    origin = np.asarray(cup.env_origin, dtype=np.float32)
    nodes = cup.view.get_nodal_positions()[0].cpu().numpy() - origin  # collision mesh: conforms to the surface
    # cross-section GRIP_UP above the cup's current bottom (it rises with the hand)
    band = nodes[np.abs(nodes[:, 2] - nodes[:, 2].min() - GRIP_UP) < 0.004][:, :2]
    band = band - band.mean(0) if len(band) else np.zeros((2, 2))
    axes = np.linalg.eigh(band.T @ band)[1]
    widths = np.ptp(band @ axes, axis=0)
    pos, quat, _ = cup._get_object_transform()
    grad = cup.view.get_sim_element_deformation_gradients().reshape(-1, 3, 3)
    force = env.robot_manager.get_gripper_force(arm.robot, [0])
    fingers = demo.as_np(arm.key.data.joint_pos[0, list(arm.robot.gripper_joint_indices)])
    flags = cup.view.get_sim_kinematic_targets()[0, :, 3]
    return {
        "kinematic_nodes": int((flags < 0.5).sum()),
        "max_node_speed": float(cup.view.get_sim_nodal_velocities()[0].norm(dim=-1).max()),
        "bottom_z": float(nodes[:, 2].min()),
        "top_z": float(nodes[:, 2].max()),
        "section_min_mm": float(widths.min() * 1e3),
        "section_max_mm": float(widths.max() * 1e3),
        "section_nodes": int(len(band)),
        "pose": [round(float(v), 5) for v in [*pos.cpu().numpy(), *quat.cpu().numpy()]],
        "min_jacobian": float(np.linalg.det(grad.cpu().numpy()).min()),
        "finite": bool(np.isfinite(nodes).all()),
        "pad_force_n": None if force is None else [float(f) for f in force[0]],
        "fingers_m": [float(f) for f in fingers],
        "grasp_point": [round(float(v), 4) for v in arm.grasp_point()],
    }


def eval_one_episode(TASK_ENV, model_client):
    env = TASK_ENV
    env.end_on_success = False  # keep stepping through the hold after the task's lift check passes
    env.step_lim = max(env.step_lim, 300)
    env.get_obs()
    robot = demo.Robot(env)
    lm = robot.lm
    cup = lm.get_scene_object(0, lm.get_instance_name(0, "target"))
    centre = robot.object("target")[0]
    arm = robot.arm_for(centre[0])
    out = -1.0 if arm.arm_key.startswith("left") else 1.0
    grip, hand, rim, radius, open_cmd = demo.cup_grasp(lm, out, "target", GRIP_UP)
    approach = grip - hand.apply([demo.CUP_SLIDE, 0, 0])
    rest = cup_state(cup, arm, env)
    if CLOSE == "full":
        close_cmd = 0.0
    else:
        close_cmd = float(np.clip((float(CLOSE) * 1e-3 - demo.PAD_CLOSED) / demo.PAD_TRAVEL, 0.0, 1.0))
    phases = [
        ("approach", [(approach + [0, 0, demo.HOVER], hand, open_cmd, 45), (approach, hand, open_cmd, 25)]),
        ("slide_in", [(grip, hand, open_cmd, 35)]),
        ("close", [(grip, hand, close_cmd, 30)]),
        ("lift", [(grip + [0, 0, LIFT], hand, close_cmd, 50)]),
        ("hold", [(grip + [0, 0, LIFT], hand, close_cmd, HOLD)]),
    ]
    log, marks, step_s, success_at = [], {}, [], None
    count = 0
    for name, keys in phases:
        for action in robot.move(arm, keys):
            t0 = time.perf_counter()
            env.take_action(action)
            step_s.append(time.perf_counter() - t0)
            count += 1
            env.is_episode_end()
            if success_at is None and env.task_success[0]:
                success_at = count
            env.get_obs()
            if count % 5 == 0:
                log.append({"action": count, "phase": name, **cup_state(cup, arm, env)})
        marks[name] = count
        log.append({"action": count, "phase": name + "_end", **cup_state(cup, arm, env)})
    end = log[-1]
    lifted = log[[e["phase"] for e in log].index("close_end")]
    hand_rise = end["grasp_point"][2] - lifted["grasp_point"][2]
    cup_rise = end["bottom_z"] - rest["bottom_z"]
    report = {
        "grip_up_m": GRIP_UP,
        "close": CLOSE,
        "close_cmd": close_cmd,
        "open_cmd": open_cmd,
        "arm": arm.arm_key,
        "grip_point": grip.round(4).tolist(),
        "rim_z": float(rim),
        "rest": rest,
        "closed": lifted,
        "end_of_hold": end,
        "cup_rise_m": cup_rise,
        "hand_rise_m": hand_rise,
        "slip_m": hand_rise - cup_rise,
        "task_success_at_action": success_at,
        "phase_end_actions": marks,
        "wall_s_per_action": {"mean": float(np.mean(step_s)), "max": float(np.max(step_s)), "n": len(step_s)},
        "log": log,
    }
    REPORT.write_text(json.dumps(report, indent=1) + "\n")
    print(f"[grasp_paper_cup_fem] cup rise {cup_rise:.3f} m, hand rise {hand_rise:.3f} m, report {REPORT}", flush=True)
    env.success[0] = bool(success_at) and cup_rise > 0.05
    env.end_flag[0] = True


if __name__ == "__main__":
    demo.main(policy_module="scripts.objects.grasp_paper_cup_fem")
