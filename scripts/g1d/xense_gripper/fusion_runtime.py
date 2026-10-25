"""Record the complete G1-D and controlled contact fixtures in isolated Isaac Sim.

Simulation only. Fixtures are analytic boxes/spheres whose measured poses are
recorded for a separate, one-way XENSE tactile process. No hardware/DDS imports.
"""

import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--out", required=True)
parser.add_argument("--fps", type=int, default=10)
parser.add_argument("--replay-recording", help="Render saved joint/object states without advancing their physics")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
args.headless = True
args.enable_cameras = True
app = AppLauncher(args).app

import hashlib  # noqa: E402
import json  # noqa: E402
import os  # noqa: E402
from pathlib import Path  # noqa: E402
import sys  # noqa: E402
import traceback  # noqa: E402
import xml.etree.ElementTree as ET  # noqa: E402

from isaaclab.assets import Articulation, RigidObject, RigidObjectCfg  # noqa: E402
from isaaclab.sensors import ContactSensor, ContactSensorCfg  # noqa: E402
import isaaclab.sim as sim_utils  # noqa: E402
from isaacsim.sensors.camera import Camera  # noqa: E402
import numpy as np  # noqa: E402
import omni.usd  # noqa: E402
from PIL import Image  # noqa: E402
from pxr import Gf, UsdGeom  # noqa: E402
import torch  # noqa: E402
from transforms3d.quaternions import mat2quat, quat2mat  # noqa: E402

from env.robot_manager.aurora_contract import CHANNELS, urdf_jaw_positions  # noqa: E402
from env.robot_manager.robot_config.g1d_aurora_lite_ws import get_robot_config  # noqa: E402
from scripts.g1d.xense_gripper.urdf_tools import origin_transform  # noqa: E402


def matrix(position, quaternion):
    t = np.eye(4)
    t[:3, :3], t[:3, 3] = quat2mat(quaternion), position
    return t


def pose(t):
    return np.r_[t[:3, 3], mat2quat(t[:3, :3])]


def look_at(eye, target):
    forward = target - eye
    forward /= np.linalg.norm(forward)
    left = np.cross([0.0, 0.0, 1.0], forward)
    left /= np.linalg.norm(left)
    return mat2quat(np.stack([forward, left, np.cross(forward, left)], axis=1))


def main():
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=False)
    for name in ("overview", "left", "right"):
        (out / name).mkdir()
    dt = 1 / 120
    if 120 % args.fps:
        raise ValueError("fps must divide 120")
    steps = 120 // args.fps
    sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=dt, device=args.device))
    light_cfg = sim_utils.DomeLightCfg(intensity=1800)
    light_cfg.func("/World/Light", light_cfg)
    floor = sim_utils.GroundPlaneCfg(color=(0.22, 0.24, 0.27))
    floor.func("/World/Floor", floor)
    config = get_robot_config()
    config.prim_path = "/World/Robot"
    config.spawn.activate_contact_sensors = True
    robot = Articulation(config)
    contacts = {
        ch: ContactSensor(
            ContactSensorCfg(
                prim_path=f"/World/Robot/{ch.split('_')[0]}_aurora_{ch.split('_')[1]}_gel_link",
                update_period=0.0,
                history_length=1,
                track_friction_forces=True,
                max_contact_data_count_per_prim=64,
                filter_prim_paths_expr=[f"/World/{kind}_{ch}" for kind in ("Flat", "Sphere")]
                + [f"/World/Gauge{width}_{side}" for width in (30, 50) for side in ("left", "right")],
            )
        )
        for ch in CHANNELS
    }
    material = sim_utils.RigidBodyMaterialCfg(static_friction=0.5, dynamic_friction=0.4, restitution=0.0)
    material.func("/World/CandidateMaterial", material)
    objects, geometry = {}, {}

    def add(name, kind, size, kinematic, color):
        rigid = sim_utils.RigidBodyPropertiesCfg(
            kinematic_enabled=kinematic, disable_gravity=True, max_depenetration_velocity=0.1
        )
        common = dict(
            rigid_props=rigid,
            collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=0.0001, rest_offset=0.0),
            mass_props=sim_utils.MassPropertiesCfg(mass=0.02),
            physics_material=material,
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=color),
        )
        spawn = (
            sim_utils.SphereCfg(radius=size, **common) if kind == "sphere" else sim_utils.CuboidCfg(size=size, **common)
        )
        objects[name] = RigidObject(
            RigidObjectCfg(
                prim_path=f"/World/{name}", spawn=spawn, init_state=RigidObjectCfg.InitialStateCfg(pos=(0, 0, -4))
            )
        )
        geometry[name] = dict(kind=kind, radius_m=size) if kind == "sphere" else dict(kind=kind, size_m=size)
        geometry[name].update(kinematic=kinematic, mass_kg=0.02)

    for ch in CHANNELS:
        add("Flat_" + ch, "box", (0.010, 0.012, 0.004), True, (0.85, 0.25, 0.12))
        add("Sphere_" + ch, "sphere", 0.004, True, (0.15, 0.45, 0.90))
    for side in ("left", "right"):
        for width in (30, 50):
            add(f"Gauge{width}_{side}", "box", (0.010, 0.012, width / 1000), False, (0.9, 0.65, 0.1))
    cameras = {
        name: Camera(f"/World/Camera_{name}", resolution=(960, 720) if name == "overview" else (640, 480))
        for name in ("overview", "left", "right")
    }
    sim.reset()
    robot.update(dt)
    for o in objects.values():
        o.update(dt)
    for camera in cameras.values():
        camera.initialize()
        camera.set_clipping_range(near_distance=0.001, far_distance=20.0)
    target = robot.data.default_joint_pos.clone()
    robot.write_joint_state_to_sim(target, torch.zeros_like(target))
    names = robot.joint_names
    urdf_path = Path(config.spawn.asset_path)
    urdf = ET.parse(urdf_path).getroot()
    local_frames = {}
    body_ids = {}
    for ch in CHANNELS:
        side, sign = ch.split("_")
        link = f"{side}_aurora_{sign}_gel_link"
        body_ids[ch] = robot.body_names.index(link)
        local_frames[ch] = origin_transform(
            urdf.find(f"joint[@name='{side}_aurora_{sign}_contact_frame_fixed']/origin")
        )
        sim_utils.bind_physics_material(f"/World/Robot/{link}", "/World/CandidateMaterial")

    # Explicit candidate material only; collision geometry/filters and gains stay unchanged.
    def frames():
        p = robot.data.body_pos_w[0].cpu().numpy()
        q = robot.data.body_quat_w[0].cpu().numpy()
        return {ch: matrix(p[body_ids[ch]], q[body_ids[ch]]) @ local_frames[ch] for ch in CHANNELS}

    for _ in range(60):
        robot.set_joint_position_target(target)
        robot.write_data_to_sim()
        sim.step()
        robot.update(dt)
    initial = frames()
    eyes = dict(overview=np.array([2.8, -3.2, 2.3]))
    overview_target = np.array([0.0, 0.0, 0.75])
    cameras["overview"].set_world_pose(
        eyes["overview"], look_at(eyes["overview"], overview_target), camera_axes="world"
    )
    for side in ("left", "right"):
        t = initial[f"{side}_positive"]
        centre = (t[:3, 3] + initial[f"{side}_negative"][:3, 3]) / 2
        eye = centre + t[:3, 0] * 0.18 + t[:3, 1] * 0.15 + t[:3, 2] * 0.09
        cameras[side].set_world_pose(eye, look_at(eye, centre), camera_axes="world")
    parked = {name: np.array([4 + i * 0.2, 0, -4, 1, 0, 0, 0], dtype=float) for i, name in enumerate(objects)}
    active, forces = set(), {}
    records = []
    objects_schema = {name: dict(**geo, world_path=f"/World/{name}") for name, geo in geometry.items()}
    (out / "scene_schema.json").write_text(
        json.dumps(
            dict(
                channels=CHANNELS,
                objects=objects_schema,
                sim_dt_s=dt,
                fps=args.fps,
                material=dict(static_friction=0.5, dynamic_friction=0.4, restitution=0, calibrated=False),
                gravity_protocol="fixtures unloaded during placement; measured mass*9.81 world-down load during hold",
                urdf_sha256=hashlib.sha256(urdf_path.read_bytes()).hexdigest(),
                force_semantics="PhysX net contact normal-force vector on each gel; FEM forces recorded separately",
                hardware_connected=False,
                sensor_frames="CAD inner-face centres; optical calibration unverified",
            ),
            indent=2,
        )
        + "\n"
    )

    def write_pose(name, value):
        objects[name].write_root_pose_to_sim(torch.tensor(value, device=sim.device, dtype=torch.float32).unsqueeze(0))
        objects[name].write_root_velocity_to_sim(torch.zeros((1, 6), device=sim.device))

    def park_all():
        nonlocal active, forces
        active, forces = set(), {}
        for name in objects:
            write_pose(name, parked[name])

    def opening(value):
        q = urdf_jaw_positions(value)
        for side in ("left", "right"):
            for sign, value in zip(("positive", "negative"), q):
                target[0, names.index(f"{side}_aurora_{sign}_jaw_joint")] = value

    if args.replay_recording:
        source = Path(args.replay_recording)
        source_rows = [json.loads(line) for line in (source / "frames.jsonl").read_text().splitlines()]
        checks = []
        stage = omni.usd.get_context().get_stage()
        visual_roots, visual_local = {}, {}
        for ch in CHANNELS:
            side, sign = ch.split("_")
            visual_roots[ch] = UsdGeom.Xformable(
                stage.GetPrimAtPath(f"/World/Robot/{side}_aurora_{sign}_gel_link/visuals")
            )
            visual_local[ch] = np.array(visual_roots[ch].GetLocalTransformation()).T
        for frame in source_rows:
            q = torch.tensor(frame["joint_pos_m_or_rad"], device=sim.device, dtype=torch.float32).unsqueeze(0)
            if names != frame["joint_names"]:
                raise ValueError("Replay joint order changed")
            robot.write_joint_state_to_sim(q, torch.zeros_like(q))
            for name in objects:
                value = frame["objects"][name]["pose_wxyz"] if name in frame["objects"] else parked[name]
                write_pose(name, value)
            sim.forward()
            robot.update(dt)
            actual = frames()
            error = max(float(np.max(np.abs(actual[ch] - frame["pad_world"][ch]))) for ch in CHANNELS)
            if error > 2e-4:
                raise ValueError(f"Render replay does not match recorded geometry: {error}")
            corrected = []
            # Fixed-joint solver compliance is not fully represented by q alone.
            # Restore the actual recorded pad visual pose, without changing any
            # physical state, force, collision, or captured tactile observation.
            for ch in CHANNELS:
                current_link = actual[ch] @ np.linalg.inv(local_frames[ch])
                recorded_link = np.array(frame["pad_world"][ch]) @ np.linalg.inv(local_frames[ch])
                delta = np.linalg.inv(current_link) @ recorded_link
                visual_roots[ch].MakeMatrixXform().Set(Gf.Matrix4d((delta @ visual_local[ch]).T.tolist()))
                corrected.append(
                    float(np.max(np.abs(current_link @ delta @ local_frames[ch] - frame["pad_world"][ch])))
                )
            checks.append(
                dict(
                    frame_id=frame["frame_id"],
                    max_pad_matrix_difference=max(corrected),
                    joint_fk_vs_recorded_matrix_difference=error,
                )
            )
            for _ in range(3):
                sim.render()
            for name, camera in cameras.items():
                Image.fromarray(camera.get_rgba()[:, :, :3]).save(out / name / f"{frame['frame_id']:05d}.png")
            if frame["frame_id"] % 20 == 0:
                print(json.dumps(dict(replay_frame=frame["frame_id"], pose_error=error)), flush=True)
        (out / "render_provenance.json").write_text(
            json.dumps(
                dict(
                    source_frames_sha256=hashlib.sha256((source / "frames.jsonl").read_bytes()).hexdigest(),
                    no_physics_steps_between_saved_states=True,
                    frames=len(checks),
                    checks=checks,
                    reason=("near plane 1mm; saved gel visual poses restored, including fixed-joint compliance"),
                ),
                indent=2,
            )
            + "\n"
        )
        return

    def capture(label, before_step=None, *, external_load=False, shear_n=0.0):
        for _ in range(steps):
            if before_step:
                before_step()
            for name in active:
                if not geometry[name]["kinematic"]:
                    f = np.array([0.0, 0.0, -0.02 * 9.81]) if external_load else np.zeros(3)
                    if shear_n:
                        side = name.split("_")[-1]
                        f += initial[f"{side}_positive"][:3, 0] * shear_n
                    # Force API is body-local by default; rotate world load into the measured object frame.
                    q = objects[name].data.root_quat_w[0].cpu().numpy()
                    f = quat2mat(q).T @ f
                    objects[name].set_external_force_and_torque(
                        torch.tensor(f, device=sim.device, dtype=torch.float32).reshape(1, 1, 3),
                        torch.zeros((1, 1, 3), device=sim.device),
                    )
                    objects[name].write_data_to_sim()
            robot.set_joint_position_target(target)
            robot.write_data_to_sim()
            sim.step()
            robot.update(dt)
            for o in objects.values():
                o.update(dt)
            for sensor in contacts.values():
                sensor.update(dt)
        sim.render()
        fid = len(records)
        frame = dict(
            frame_id=fid,
            simulation_time_s=float(sim.current_time),
            phase=label,
            joint_names=names,
            joint_pos_m_or_rad=robot.data.joint_pos[0].cpu().tolist(),
            target_m_or_rad=target[0].cpu().tolist(),
            pad_world={ch: t.tolist() for ch, t in frames().items()},
            objects={},
            native_contact_normal_force_w_n={},
            native_friction_force_w_n={},
            applied_world_shear_n=shear_n,
            applied_weight=external_load,
        )
        for name in sorted(active):
            o = objects[name]
            frame["objects"][name] = dict(
                pose_wxyz=o.data.root_pose_w[0].cpu().tolist(), velocity_w=o.data.root_vel_w[0].cpu().tolist()
            )
        for ch, sensor in contacts.items():
            frame["native_contact_normal_force_w_n"][ch] = sensor.data.force_matrix_w[0, 0].sum(0).cpu().tolist()
            frame["native_friction_force_w_n"][ch] = (
                torch.nan_to_num(sensor.data.friction_forces_w[0, 0]).sum(0).cpu().tolist()
            )
        for name, camera in cameras.items():
            # All images belong to this state; no physics steps are taken between cameras.
            rgba = camera.get_rgba()
            Image.fromarray(rgba[:, :, :3]).save(out / name / f"{fid:05d}.png")
        records.append(frame)
        with (out / "frames.jsonl").open("a") as stream:
            stream.write(json.dumps(frame) + "\n")
        if fid % 10 == 0:
            (out / "progress.json").write_text(
                json.dumps(dict(frame=fid, phase=label, simulation_time_s=sim.current_time))
            )
            print(json.dumps(dict(frame=fid, phase=label)), flush=True)

    park_all()
    for gap in (0.0, 0.0425, 0.085):
        opening(gap)
        for _ in range(10):
            capture(f"opening_{gap * 1000:g}mm")
    for width in (30, 50):
        park_all()
        opening(width / 1000)
        for _ in range(4):
            capture(f"gauge_{width}_setup")
        current = frames()
        for side in ("left", "right"):
            t = current[f"{side}_positive"].copy()
            t[:3, 3] = (t[:3, 3] + current[f"{side}_negative"][:3, 3]) / 2
            name = f"Gauge{width}_{side}"
            write_pose(name, pose(t))
            active.add(name)
        opening(width / 1000 - 0.0006)
        for _ in range(5):
            capture(f"gauge_{width}_clamp")
        for _ in range(8):
            capture(f"gauge_{width}_hold", external_load=True)
        if width == 30:
            for _ in range(5):
                capture("gauge_30_slip", external_load=True, shear_n=5.0)
    park_all()
    opening(0.085)
    for _ in range(4):
        capture("probe_setup")
    anchor = frames()
    for kind in ("Flat", "Sphere"):
        park_all()
        active = {kind + "_" + ch for ch in CHANNELS}
        # Four independent probes contact all four actual gel surfaces.
        schedule = [("approach", d, 0.0) for d in np.linspace(-0.0005, 0.00025, 10)]
        schedule += [("hold", 0.00025, 0.0)] * 5
        if kind == "Flat":
            schedule += [("shear", 0.00025, x) for x in np.linspace(0, 0.003, 10)]
        schedule += [("unload", d, 0.003 if kind == "Flat" else 0.0) for d in np.linspace(0.00025, -0.0008, 10)]
        schedule += [("released", -0.0008, 0.0)] * 5
        for phase, depth, shift in schedule:

            def place(depth=depth, shift=shift, kind=kind):
                for ch in CHANNELS:
                    t = anchor[ch].copy()
                    offset = np.array([shift, 0.0, (0.002 if kind == "Flat" else 0.004) - depth])
                    t[:3, 3] += t[:3, :3] @ offset
                    write_pose(kind + "_" + ch, pose(t))

            capture(kind.lower() + "_" + phase, place)
    park_all()
    for _ in range(5):
        capture("final_unloaded")
    values = np.array([r["joint_pos_m_or_rad"] for r in records])
    report = dict(
        status="recorded",
        frames=len(records),
        simulation_s=len(records) / args.fps,
        finite_joint_states=bool(np.isfinite(values).all()),
        channels=list(CHANNELS),
        native_contact_peak_n={
            ch: float(max(np.linalg.norm(r["native_contact_normal_force_w_n"][ch]) for r in records)) for ch in CHANNELS
        },
        physics_owner="Isaac Sim 5.1 / PhysX",
        tactile_force_feedback=False,
        calibration_status="candidate simulation parameters; no hardware calibration",
        hardware_connected=False,
    )
    (out / "runtime_report.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report), flush=True)


try:
    main()
    code = 0
except Exception:
    traceback.print_exc()
    code = 1
sys.stdout.flush()
sys.stderr.flush()
os._exit(code)
