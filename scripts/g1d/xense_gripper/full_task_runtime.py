"""Simulation-only dual-arm free-body pick/transport/place verification.

Objects have gravity and remain dynamic throughout an episode. Only the initial
layout is written; no attachment, root-pose reset, or artificial holding force.
The JSONL sample precedes its action (and all cameras see that same state).
"""

import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--out", required=True)
parser.add_argument("--plan", required=True)
parser.add_argument("--block-width", type=float, default=0.03)
parser.add_argument("--mass", type=float, default=0.02)
parser.add_argument("--friction", type=float, default=0.5)
parser.add_argument("--grip-overtravel", type=float, default=0.001, help="Candidate total gap preload in metres")
parser.add_argument("--fast-exit", action="store_true", help="Exit isolated worker after all files are closed")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
args.headless = True
args.enable_cameras = True
app = AppLauncher(args).app

import hashlib  # noqa: E402
import json  # noqa: E402
import os  # noqa: E402
from pathlib import Path  # noqa: E402
import traceback  # noqa: E402
import xml.etree.ElementTree as ET  # noqa: E402

from isaaclab.assets import Articulation, RigidObject, RigidObjectCfg  # noqa: E402
from isaaclab.sensors import ContactSensor, ContactSensorCfg  # noqa: E402
import isaaclab.sim as sim_utils  # noqa: E402
from isaacsim.sensors.camera import Camera  # noqa: E402
import numpy as np  # noqa: E402
from PIL import Image  # noqa: E402
import torch  # noqa: E402
from transforms3d.quaternions import mat2quat, quat2mat  # noqa: E402

from env.robot_manager.aurora_contract import CHANNELS  # noqa: E402
from env.robot_manager.robot_config.g1d_aurora_lite_ws import get_robot_config  # noqa: E402
from scripts.g1d.xense_gripper.urdf_tools import origin_transform  # noqa: E402


def transform(p, q):
    t = np.eye(4)
    t[:3, :3], t[:3, 3] = quat2mat(q), p
    return t


def look(eye, target):
    forward = np.array(target) - eye
    forward /= np.linalg.norm(forward)
    left = np.cross([0.0, 0.0, 1.0], forward)
    left /= np.linalg.norm(left)
    return mat2quat(np.stack([forward, left, np.cross(forward, left)], axis=1))


def main():
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=False)
    plan = json.loads(Path(args.plan).read_text())
    dt, fps = 1 / 240, 10
    sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=dt, device=args.device))
    cfg = sim_utils.DomeLightCfg(intensity=1800)
    cfg.func("/World/Light", cfg)
    cfg = sim_utils.GroundPlaneCfg(color=(0.25, 0.28, 0.32))
    cfg.func("/World/Floor", cfg)
    cfg = sim_utils.CuboidCfg(
        size=(0.55, 0.85, 0.04),
        collision_props=sim_utils.CollisionPropertiesCfg(),
        visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.42, 0.30, 0.18)),
    )
    cfg.func("/World/Table", cfg, translation=(0.49, 0, 0.67))
    robot_cfg = get_robot_config()
    robot_cfg.prim_path = "/World/Robot"
    robot_cfg.spawn.activate_contact_sensors = True
    robot = Articulation(robot_cfg)
    material = sim_utils.RigidBodyMaterialCfg(
        static_friction=args.friction, dynamic_friction=args.friction * 0.8, restitution=0.0
    )
    material.func("/World/Material", material)
    objects, schema = {}, {}
    for side in ["left", "right"]:
        p = plan[side]["waypoints"]["source"]["tcp_m"]
        spawn = sim_utils.CuboidCfg(
            size=(0.04, args.block_width, 0.06),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(disable_gravity=False, max_depenetration_velocity=0.1),
            collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=0.0001, rest_offset=0.0),
            mass_props=sim_utils.MassPropertiesCfg(mass=args.mass),
            physics_material=material,
            visual_material=sim_utils.PreviewSurfaceCfg(
                diffuse_color=(0.85, 0.18, 0.08) if side == "left" else (0.1, 0.42, 0.86)
            ),
        )
        objects[side] = RigidObject(
            RigidObjectCfg(
                prim_path=f"/World/Block_{side}", spawn=spawn, init_state=RigidObjectCfg.InitialStateCfg(pos=tuple(p))
            )
        )
        schema[side] = dict(kind="box", size_m=[0.04, args.block_width, 0.06], mass_kg=args.mass, kinematic=False)
    contacts = {
        ch: ContactSensor(
            ContactSensorCfg(
                prim_path=f"/World/Robot/{ch.split('_')[0]}_aurora_{ch.split('_')[1]}_gel_link",
                update_period=0.0,
                history_length=1,
                track_friction_forces=True,
                filter_prim_paths_expr=["/World/Block_left", "/World/Block_right"],
                max_contact_data_count_per_prim=64,
            )
        )
        for ch in CHANNELS
    }
    cameras = {
        n: Camera(f"/World/Camera_{n}", resolution=(640, 480) if n == "overview" else (320, 240))
        for n in ["overview", "left", "right"]
    }
    for n in cameras:
        (out / n).mkdir()
    sim.reset()
    robot.update(dt)
    names = robot.joint_names
    target = robot.data.default_joint_pos.clone()
    for side in plan:
        for name, q in zip(plan[side]["joint_names"], plan[side]["waypoints"]["pregrasp"]["q"]):
            target[0, names.index(name)] = q
    robot.write_joint_state_to_sim(target, torch.zeros_like(target))
    urdf_path = Path(robot_cfg.spawn.asset_path)
    urdf = ET.parse(urdf_path).getroot()
    ids, local = {}, {}
    for ch in CHANNELS:
        side, sign = ch.split("_")
        link = f"{side}_aurora_{sign}_gel_link"
        ids[ch] = robot.body_names.index(link)
        local[ch] = origin_transform(urdf.find(f"joint[@name='{side}_aurora_{sign}_contact_frame_fixed']/origin"))
        sim_utils.bind_physics_material("/World/Robot/" + link, "/World/Material")
    for n, cam in cameras.items():
        cam.initialize()
        cam.set_clipping_range(near_distance=0.001, far_distance=15.0)
        eye = np.array([2.8, -3.2, 2.1]) if n == "overview" else np.array([0.85, 0.55 if n == "left" else -0.55, 1.12])
        focus = [0.2, 0, 0.75] if n == "overview" else [0.4, 0.23 if n == "left" else -0.23, 0.8]
        cam.set_world_pose(eye, look(eye, focus), camera_axes="world")
    for _ in range(120):
        robot.set_joint_position_target(target)
        robot.write_data_to_sim()
        sim.step()
        robot.update(dt)
        for obj in objects.values():
            obj.update(dt)
        for c in contacts.values():
            c.update(dt)
    scene = dict(
        channels=CHANNELS,
        objects=schema,
        sim_dt_s=dt,
        fps=fps,
        source_frames="pre-action sample; target applied after this frame",
        task="dual_arm_pick_transport_place",
        instruction="Pick both blocks, lift them, move them forward and place them on the table.",
        table_top_m=0.69,
        target_centres_m={s: plan[s]["waypoints"]["place"]["tcp_m"] for s in plan},
        material=dict(static_friction=args.friction, dynamic_friction=args.friction * 0.8, calibrated=False),
        grip_overtravel_m=args.grip_overtravel,
        runtime_script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        plan_sha256=hashlib.sha256(Path(args.plan).read_bytes()).hexdigest(),
        urdf_sha256=hashlib.sha256(urdf_path.read_bytes()).hexdigest(),
        layout_initial_m={s: plan[s]["waypoints"]["source"]["tcp_m"] for s in plan},
        gravity_m_s2=[0, 0, -9.81],
        objects_attached=False,
        object_resets_after_initial_layout=0,
        artificial_holding_forces=False,
        hardware_connected=False,
    )
    (out / "scene_schema.json").write_text(json.dumps(scene, indent=2) + "\n")
    count = 0

    def sample(phase):
        nonlocal count
        sim.render()
        poses = {
            ch: (
                transform(
                    robot.data.body_pos_w[0, ids[ch]].cpu().numpy(), robot.data.body_quat_w[0, ids[ch]].cpu().numpy()
                )
                @ local[ch]
            ).tolist()
            for ch in CHANNELS
        }
        frame = dict(
            frame_id=count,
            simulation_time_s=float(sim.current_time),
            phase=phase,
            joint_names=names,
            joint_pos_m_or_rad=robot.data.joint_pos[0].cpu().tolist(),
            target_m_or_rad=target[0].cpu().tolist(),
            pad_world=poses,
            objects={
                s: dict(pose_wxyz=o.data.root_pose_w[0].cpu().tolist(), velocity_w=o.data.root_vel_w[0].cpu().tolist())
                for s, o in objects.items()
            },
            native_contact_normal_force_w_n={
                ch: c.data.force_matrix_w[0, 0].sum(0).cpu().tolist() for ch, c in contacts.items()
            },
            native_friction_force_w_n={
                ch: torch.nan_to_num(c.data.friction_forces_w[0, 0]).sum(0).cpu().tolist() for ch, c in contacts.items()
            },
        )
        for n, cam in cameras.items():
            Image.fromarray(cam.get_rgba()[:, :, :3]).save(out / n / f"{count:05d}.png")
        with (out / "frames.jsonl").open("a") as f:
            f.write(json.dumps(frame) + "\n")
        count += 1
        for _ in range(24):
            robot.set_joint_position_target(target)
            robot.write_data_to_sim()
            sim.step()
            robot.update(dt)
            for obj in objects.values():
                obj.update(dt)
            for c in contacts.values():
                c.update(dt)

    def move(phase, waypoint, n=15):
        start = target.clone()
        goal = target.clone()
        for side in plan:
            for name, q in zip(plan[side]["joint_names"], plan[side]["waypoints"][waypoint]["q"]):
                goal[0, names.index(name)] = q
        for i in range(n):
            a = (i + 1) / n
            a = a * a * (3 - 2 * a)
            target[:] = start + (goal - start) * a
            for side in plan:
                sequence = plan[side].get("trajectories", {}).get(phase)
                if sequence is not None:
                    if len(sequence) != n:
                        raise ValueError("Cartesian plan/control sample count mismatch")
                    for name, q in zip(plan[side]["joint_names"], sequence[i]):
                        target[0, names.index(name)] = q
            sample(phase)
        for _ in range(4):
            sample(phase + "_hold")
        print(
            json.dumps(
                dict(
                    phase=phase,
                    frame=count,
                    block_positions={s: o.data.root_pos_w[0].cpu().tolist() for s, o in objects.items()},
                )
            ),
            flush=True,
        )

    def gap(value):
        for side in plan:
            for sign in ["positive", "negative"]:
                target[0, names.index(f"{side}_aurora_{sign}_jaw_joint")] = value / 2

    for _ in range(5):
        sample("initial")
    move("approach", "source")
    gap(args.block_width - args.grip_overtravel)
    for _ in range(12):
        sample("close")
    move("lift", "lift", 20)
    move("transport", "transfer", 20)
    move("lower", "place", 20)
    gap(0.085)
    for _ in range(12):
        sample("release")
    move("retreat", "retreat", 15)
    for _ in range(15):
        sample("final")
    (out / "recording_complete.json").write_text(json.dumps(dict(frames=count, hardware_connected=False)))
    print(json.dumps(dict(recording_complete=True, frames=count)), flush=True)
    if args.fast_exit:
        # All JSONL/PNG files and the completion receipt have been closed. This
        # avoids Isaac 5.1 camera destructor/Replicator shutdown waits in an
        # isolated worker; never use it before the durable completion receipt.
        os._exit(0)


if __name__ == "__main__":
    try:
        main()
    except Exception:
        traceback.print_exc()
        raise
    finally:
        app.close(wait_for_replicator=False)
