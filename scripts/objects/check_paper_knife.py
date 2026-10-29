"""Paper-knife project-factory reuse, contact clamp and neutral RTX preview.

Prescribed kinematic jaws are a contact diagnostic, not a robot success test.
Real friction, material response and blade cutting are not calibrated.
"""

import argparse
import hashlib
import json
from pathlib import Path
import traceback
from types import SimpleNamespace

import numpy as np


def arr(value):
    return value.detach().cpu().numpy() if hasattr(value, "detach") else np.asarray(value)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    from isaacsim import SimulationApp

    app = SimulationApp({"headless": True, "fast_shutdown": True})
    report = {"passed": False, "robot_grasp_validated": False, "real_material_calibrated": False}
    try:
        import carb

        carb.settings.get_settings().set_bool("/isaaclab/render/offscreen", True)
        from isaaclab.assets import RigidObject as NativeRigid, RigidObjectCfg
        import isaaclab.sim as sim_utils
        from isaacsim.core.api import World
        from isaacsim.core.api.objects import FixedCuboid
        from isaacsim.core.utils.stage import get_current_stage
        from isaacsim.core.utils.viewports import set_camera_view
        from isaacsim.sensors.camera import Camera
        from omegaconf import OmegaConf
        from PIL import Image
        from pxr import UsdGeom, UsdLux, UsdPhysics
        import torch

        import env.scene_manager.objects.rigid as rigid_module
        from env.scene_manager.scene_manager import SceneManager

        report["project_factory"] = {
            "source": str(Path(rigid_module.__file__).resolve()),
            "rigid_source_sha256": hashlib.sha256(Path(rigid_module.__file__).read_bytes()).hexdigest(),
            "factory_source_sha256": hashlib.sha256(
                Path(__import__("inspect").getfile(SceneManager)).read_bytes()
            ).hexdigest(),
            "note": "deployed project working tree; evidence hashes identify tested implementation",
        }
        world = World(physics_dt=1 / 240, rendering_dt=1 / 30, backend="torch", device="cpu")
        world.get_physics_context().enable_gpu_dynamics(False)
        world.get_physics_context().set_broadphase_type("MBP")
        world.scene.add(
            FixedCuboid(
                prim_path="/World/Floor",
                name="floor",
                position=np.array([0.3, 0, -0.01]),
                scale=np.array([3, 3, 0.02]),
                size=1,
                color=np.array([0.42, 0.43, 0.45]),
            )
        )
        factory = SceneManager.__new__(SceneManager)
        factory.sim = SimpleNamespace(scene=SimpleNamespace(env_origins=torch.zeros((1, 3))))
        objs, metas = [], []
        H = 0.01714
        for idx in range(4):
            d = args.root / f"{idx:05d}"
            metas.append(json.loads((d / "metadata.json").read_text()))
            obj = factory.instantiate_object_wrapper(
                obj_type="rigid",
                primitive_type=None,
                env_id=0,
                prim_path=f"/World/env_0/Object/Rigid/paper_knife_{idx:05d}_1",
                usd_path=str((d / "object.usdz").resolve()),
                inst_cfg=OmegaConf.create({"physics": {}}),
                default_pos=(idx * 0.28, 0, H / 2 + 0.06),
                default_ori=(1, 0, 0, 0),
                scale=(1, 1, 1),
            )
            objs.append(obj)

        def spawn(path, cfg, position):
            return NativeRigid(
                RigidObjectCfg(prim_path=path, spawn=cfg, init_state=RigidObjectCfg.InitialStateCfg(pos=position))
            )

        def cuboid(size):
            return sim_utils.CuboidCfg(
                size=size,
                mass_props=sim_utils.MassPropertiesCfg(mass=1),
                rigid_props=sim_utils.RigidBodyPropertiesCfg(disable_gravity=True, kinematic_enabled=True),
                collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=0.0001, rest_offset=0),
                visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.14, 0.42, 0.67)),
            )

        jaws = [
            spawn(f"/World/Jaw{i}", cuboid((0.025, 0.01, 0.016)), (0, (-1 if i == 0 else 1) * 0.05, 0.01))
            for i in range(2)
        ]
        pusher = spawn("/World/Pusher", cuboid((0.012, 0.07, 0.035)), (1.5, 1.5, 1))
        probe = spawn(
            "/World/Probe",
            sim_utils.SphereCfg(
                radius=0.0012,
                mass_props=sim_utils.MassPropertiesCfg(mass=0.001),
                rigid_props=sim_utils.RigidBodyPropertiesCfg(disable_gravity=True),
                collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=0.0001, rest_offset=0),
            ),
            (1.5, 1.5, 1),
        )
        stage = get_current_stage()
        UsdLux.DomeLight.Define(stage, "/World/NeutralFill").CreateIntensityAttr(650)
        key = UsdLux.DistantLight.Define(stage, "/World/NeutralKey")
        key.CreateIntensityAttr(1200)
        key.CreateAngleAttr(12)
        camera = Camera("/World/Camera", resolution=(1400, 1050))
        camera.set_clipping_range(0.001, 10)
        camera.prim.GetAttribute("focalLength").Set(42)
        set_camera_view((0, -0.22, 0.32), (0, 0, 0.01), camera_prim_path=camera.prim_path)
        world.reset()
        for obj in objs:
            obj.initialize()
        camera.initialize()

        def step(n):
            for _ in range(n):
                world.step(render=False)

        def place(probe, pos, vel=(0, 0, 0)):
            state = probe.data.default_root_state.clone()
            state[:, :3] = torch.tensor(pos)
            state[:, 7:10] = torch.tensor(vel)
            state[:, 10:] = 0
            probe.write_root_state_to_sim(state)

        def target(probe, pos):
            transforms = probe.root_physx_view.get_transforms().clone()
            transforms[:, :3] = torch.tensor(pos)
            probe.root_physx_view.set_kinematic_targets(transforms, torch.tensor([0], dtype=torch.int32))

        def pose_obj(obj, pos, ori=(1, 0, 0, 0)):
            obj.set_world_pose(position=torch.tensor(pos), orientation=torch.tensor(ori))
            obj._rigid_prim_view.set_velocities(torch.zeros((1, 6)))

        def capture(name, center=(0, 0, 0.01), offset=(0, -0.12, 0.32)):
            set_camera_view(np.array(center) + np.array(offset) * 1.28, center, camera_prim_path=camera.prim_path)
            for _ in range(20):
                world.render()
            rgba = camera.get_rgba()
            assert rgba is not None and rgba.shape == (1050, 1400, 4)
            Image.fromarray(rgba).save(args.output / name)

        for p in ["/World/Jaw0", "/World/Jaw1", "/World/Pusher", "/World/Probe"]:
            UsdGeom.Imageable(stage.GetPrimAtPath(p)).MakeInvisible()
        step(720)
        report["factory_load_and_drop"] = []
        for idx, obj in enumerate(objs):
            pos, q = map(arr, obj.get_local_pose())
            tilt = float(np.degrees(np.arccos(np.clip(1 - 2 * (q[1] ** 2 + q[2] ** 2), -1, 1))))
            speed = float(np.linalg.norm(arr(obj.get_linear_velocity())))
            mass = float(arr(obj.get_mass()).item())
            report["factory_load_and_drop"].append(
                dict(
                    id=f"Rigid/paper_knife/{idx:05d}",
                    mass_kg=mass,
                    position_m=pos.tolist(),
                    tilt_deg=tilt,
                    speed_m_s=speed,
                    height_error_m=float(abs(pos[2] - H / 2)),
                    wrapper=type(obj).__name__,
                    passed=bool(
                        abs(mass - 0.0589) < 1e-6 and tilt < 5 and abs(pos[2] - H / 2) < 0.005 and speed < 0.01
                    ),
                )
            )
            capture(f"variant_{idx:05d}.png", (idx * 0.28, 0, H / 2))
        obj = objs[0]
        # Neutral multi-view and approximate original-photo camera directions.
        for name, offset in [
            ("sim.png", (0, -0.13, 0.28)),
            ("top.png", (0, -0.001, 0.34)),
            ("front.png", (0, -0.34, 0.035)),
            ("tail.png", (0.30, -0.14, 0.19)),
            ("nose.png", (-0.30, -0.14, 0.19)),
            ("reference_0.png", (0, -0.15, 0.31)),
            ("reference_1.png", (-0.08, -0.29, 0.38)),
            ("reference_2.png", (0, -0.25, 0.17)),
            ("reference_3.png", (0.24, -0.21, 0.29)),
        ]:
            capture(name, offset=offset)
        # Hold the second independent factory-loaded instance away from the floor.
        other = objs[1]
        UsdPhysics.RigidBodyAPI(
            stage.GetPrimAtPath("/World/env_0/Object/Rigid/paper_knife_00001_1")
        ).CreateKinematicEnabledAttr(True)
        pose_obj(other, (0.28, 0, 0.1))
        step(10)
        # Tail-centre XY uses the same bbox transform as the generator.
        aperture_xy = (0.28 + (141 - 76.44) / 1000, (-16.5 + 2.28) / 1000)
        results = []
        for kind, xy in [("hole", aperture_xy), ("solid", (0.28, 0))]:
            place(probe, [*xy, 0.14], [0, 0, -0.12])
            step(200)
            xyz = arr(probe.root_physx_view.get_transforms())[0, :3]
            results.append(
                dict(
                    kind=kind, position_m=xyz.tolist(), passed=bool(xyz[2] < 0.085 if kind == "hole" else xyz[2] > 0.09)
                )
            )
        report["collision_channel"] = {"probes": results, "passed": all(x["passed"] for x in results)}
        place(probe, [1.5, 1.5, 1])
        before = arr(obj.get_local_pose()[0]).copy()
        for i in range(241):
            target(pusher, [-0.095 + 0.06 * i / 240, 0, 0.014])
            step(1)
        after = arr(obj.get_local_pose()[0]).copy()
        report["solid_blocking"] = dict(
            displacement_m=float(after[0] - before[0]), passed=bool(after[0] - before[0] > 0.008)
        )
        place(pusher, [1.5, 1.5, 1])
        pose_obj(obj, [0, 0, H / 2 + 0.0002])
        step(180)
        for i in range(2):
            UsdGeom.Imageable(stage.GetPrimAtPath(f"/World/Jaw{i}")).MakeVisible()
        close_y = 0.03283 / 2 + 0.005 - 0.00025
        for i in range(181):
            y = 0.05 + (close_y - 0.05) * i / 180
            for jaw, sign in zip(jaws, [-1, 1]):
                target(jaw, [0, sign * y, 0.01])
            step(1)
        start = arr(obj.get_local_pose()[0]).copy()
        samples = []
        for i in range(241):
            lift = 0.05 * i / 240
            for jaw, sign in zip(jaws, [-1, 1]):
                target(jaw, [0, sign * close_y, 0.01 + lift])
            step(1)
            if i % 12 == 0:
                samples.append(dict(jaw_lift_m=lift, position_m=arr(obj.get_local_pose()[0]).tolist()))
        step(120)
        held = arr(obj.get_local_pose()[0]).copy()
        lift = float(held[2] - start[2])
        capture("clamp_lift.png", center=(0, 0, float(held[2])), offset=(0.17, -0.2, 0.18))
        for jaw, sign in zip(jaws, [-1, 1]):
            target(jaw, [0, sign * 0.07, 0.06])
        step(720)
        released = arr(obj.get_local_pose()[0]).copy()
        report["clamp_lift_release"] = dict(
            lift_m=lift,
            slip_m=abs(0.05 - lift),
            released_position_m=released.tolist(),
            samples=samples,
            passed=bool(lift > 0.035 and abs(0.05 - lift) < 0.005 and released[2] < 0.03),
            scope="two prescribed kinematic jaws; no robot, measured force or real-friction claim",
        )
        for jaw in jaws:
            place(jaw, [1.5, 1.5, 1])
        obj.set_world_pose(
            position=torch.tensor([0.1, -0.1, 0.3]), orientation=torch.tensor([0.9238795, 0, 0, 0.3826834])
        )
        obj._rigid_prim_view.set_velocities(torch.tensor([[0.1, 0.2, 0.3, 0.2, 0.1, 0.3]]))
        injected = arr(obj._rigid_prim_view.get_velocities())
        obj.apply_saved_pose()
        rp, rq = map(arr, obj.get_local_pose())
        rv = arr(obj._rigid_prim_view.get_velocities())
        report["reset"] = dict(
            position_error_m=float(np.max(abs(rp - obj.default_pos))),
            velocity=rv.tolist(),
            passed=bool(
                np.max(abs(rp - obj.default_pos)) < 1e-5 and np.max(abs(rv)) < 1e-5 and np.linalg.norm(injected) > 0.2
            ),
        )
        step(720)
        report["reset"]["resettled_z_m"] = float(arr(obj.get_local_pose()[0])[2])
        report["reset"]["passed"] &= abs(report["reset"]["resettled_z_m"] - H / 2) < 0.005
        report["asset_file_sha256"] = {
            f"{idx:05d}/{name}": hashlib.sha256((args.root / f"{idx:05d}" / name).read_bytes()).hexdigest()
            for idx in range(4)
            for name in ["object.usdz", "metadata.json", "description.json", "runtime_config.json"]
        }
        report["backend"] = "SceneManager.instantiate_object_wrapper / PhysX CPU, 240 Hz"
        report["reference_camera_scope"] = (
            "approximate same side/direction; no camera intrinsics or extrinsics supplied"
        )
        report["passed"] = all(r["passed"] for r in report["factory_load_and_drop"]) and all(
            report[k]["passed"] for k in ["collision_channel", "solid_blocking", "clamp_lift_release", "reset"]
        )
    except Exception:
        report["error"] = traceback.format_exc()
        traceback.print_exc()
    finally:
        (args.output / "report.json").write_text(json.dumps(report, indent=2) + "\n")
        print(json.dumps(report), flush=True)
        if not report["passed"]:
            import os
            import sys

            sys.stdout.flush()
            sys.stderr.flush()
            os._exit(1)
        app.close(skip_cleanup=True)


if __name__ == "__main__":
    main()
