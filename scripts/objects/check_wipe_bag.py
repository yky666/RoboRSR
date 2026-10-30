"""Wipe packet: actual project factory, FEM contact, recovery and reset validation."""

import argparse
import hashlib
import json
from pathlib import Path
import traceback
from types import SimpleNamespace

import numpy as np


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--asset", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--reload-only", action="store_true")
    p.add_argument("--replay-only", action="store_true")
    p.add_argument("--mesh-only", action="store_true")
    p.add_argument("--clamp-only", action="store_true")
    a = p.parse_args()
    a.output.mkdir(parents=True, exist_ok=False)
    from isaacsim import SimulationApp

    app = SimulationApp({"headless": True, "fast_shutdown": True})
    report = dict(passed=False, real_material_calibrated=False, robot_grasp_validated=False)
    obj = None
    try:
        import carb

        carb.settings.get_settings().set_bool("/isaaclab/render/offscreen", True)
        from isaaclab.assets import RigidObject, RigidObjectCfg
        import isaaclab.sim as sim_utils
        from isaacsim.core.utils.viewports import set_camera_view
        from isaacsim.sensors.camera import Camera
        from omegaconf import OmegaConf
        from PIL import Image
        from scipy.spatial.transform import Rotation
        import torch

        from env.scene_manager.objects.deformable import DeformableObject
        from env.scene_manager.scene_manager import SceneManager

        meta = json.loads((a.asset / "metadata.json").read_text())
        size = np.array(meta["physics"]["size"])
        height = size[2]
        sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=1 / 240, device="cuda:0"))
        floor = sim_utils.CuboidCfg(size=(1.5, 1.5, 0.02), collision_props=sim_utils.CollisionPropertiesCfg())
        floor.func("/World/Floor", floor, translation=(0, 0, -0.01))
        factory = SceneManager.__new__(SceneManager)
        factory.sim = SimpleNamespace(scene=SimpleNamespace(env_origins=torch.zeros((1, 3))))
        obj = factory.instantiate_object_wrapper(
            obj_type="rigid",
            primitive_type=None,
            env_id=0,
            prim_path="/World/env_0/Rigid/wipe_bag_00000_1",
            usd_path=str((a.asset / "object.usdz").resolve()),
            inst_cfg=OmegaConf.create({}),
            default_pos=(0, 0, float(height / 2 + 0.08)),
            default_ori=(1, 0, 0, 0),
            scale=(1, 1, 1),
        )
        assert isinstance(obj, DeformableObject)
        plate = RigidObject(
            RigidObjectCfg(
                prim_path="/World/Plate",
                spawn=sim_utils.CuboidCfg(
                    size=(0.22, 0.13, 0.01),
                    rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
                    mass_props=sim_utils.MassPropertiesCfg(mass=1),
                    collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=0.001, rest_offset=0),
                    visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.5, 0.5, 0.55), opacity=0.35),
                ),
                init_state=RigidObjectCfg.InitialStateCfg(pos=(0.9, 0, 0.6)),
            )
        )
        lower = RigidObject(
            RigidObjectCfg(
                prim_path="/World/LowerFinger",
                spawn=sim_utils.CuboidCfg(
                    size=(0.060, 0.045, 0.008),
                    rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
                    mass_props=sim_utils.MassPropertiesCfg(mass=0.1),
                    collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=0.0005, rest_offset=0),
                ),
                init_state=RigidObjectCfg.InitialStateCfg(pos=(0.8, 0, -0.004)),
            )
        )
        light = sim_utils.DomeLightCfg(intensity=650, visible_in_primary_ray=False)
        light.func("/World/Light", light)
        camera = Camera("/World/Camera", resolution=(1600, 1200))
        camera.set_clipping_range(0.001, 10)
        camera.prim.GetAttribute("focalLength").Set(28.0)
        set_camera_view((0.22, -0.29, 0.21), (0, 0, float(height / 2)), camera_prim_path=camera.prim_path)
        sim.reset()
        obj.initialize()
        camera.initialize()
        obj._capture_rest()
        report["initial_nodal_bounds_m"] = [
            obj.fem.root_physx_view.get_sim_nodal_positions().amin(dim=-2).cpu().tolist(),
            obj.fem.root_physx_view.get_sim_nodal_positions().amax(dim=-2).cpu().tolist(),
        ]
        (finite, minimum_j, phase) = (True, 1.0, "drop")
        minima = {}

        def nodes():
            return obj.fem.root_physx_view.get_sim_nodal_positions().clone()

        def step(count):
            nonlocal finite, minimum_j
            for _ in range(count):
                sim.step(render=False)
                obj.fem.update(1 / 240)
                gradient = obj.fem.root_physx_view.get_sim_element_deformation_gradients().reshape(-1, 3, 3)
                finite &= bool(torch.isfinite(nodes()).all() and torch.isfinite(gradient).all())
                j = float(torch.linalg.det(gradient).min().item())
                minimum_j = min(minimum_j, j)
                minima[phase] = min(minima.get(phase, 1.0), j)

        def capture(name):
            for _ in range(10):
                sim.render()
            Image.fromarray(camera.get_rgba()).save(a.output / name)

        def extent():
            n = obj.fem.root_physx_view.get_nodal_positions()
            return float((n[..., 2].max() - n[..., 2].min()).item())

        def multiview(prefix):
            values = obj.fem.root_physx_view.get_nodal_positions().reshape(-1, 3)
            center = ((values.amin(dim=0) + values.amax(dim=0)) / 2).cpu().numpy()
            for view, delta in (
                ("front", (0, -0.28, 0.07)),
                ("side", (0.30, 0, 0.08)),
                ("rear", (0, 0.28, 0.07)),
                ("top", (0.001, 0, 0.32)),
            ):
                set_camera_view(center + np.array(delta), center, camera_prim_path=camera.prim_path)
                capture(prefix + view + ".png")
            set_camera_view(center + np.array([0.16, -0.28, 0.1]), center, camera_prim_path=camera.prim_path)

        if a.clamp_only:
            obj.default_pos = (0, 0, float(height / 2 + 0.002))
            obj.apply_saved_pose()
            step(360)
            ptop = plate.data.default_root_state[:, :7].clone()
            pbottom = lower.data.default_root_state[:, :7].clone()
            pbottom[:, 0] = 0
            pbottom[:, 2] = -0.004
            lower.write_root_pose_to_sim(pbottom)
            ptop[:, 0] = 0
            ptop[:, 2] = 0.065
            plate.write_root_pose_to_sim(ptop)
            base_centre = float(nodes()[..., 2].mean())
            for i in range(240):
                ptop[:, 2] = 0.065 + (0.035 - 0.065) * (i + 1) / 240
                plate.write_root_pose_to_sim(ptop)
                step(1)
            capture("clamp_closed.png")
            for i in range(360):
                rise = 0.080 * (i + 1) / 360
                ptop[:, 2] = 0.035 + rise
                pbottom[:, 2] = -0.004 + rise
                plate.write_root_pose_to_sim(ptop)
                lower.write_root_pose_to_sim(pbottom)
                step(1)
            step(240)
            lift = float(nodes()[..., 2].mean()) - base_centre
            capture("clamp_lift.png")
            ptop[:, 0] = 0.8
            pbottom[:, 0] = 0.8
            plate.write_root_pose_to_sim(ptop)
            lower.write_root_pose_to_sim(pbottom)
            step(720)
            released = float(nodes()[..., 2].mean()) - base_centre
            capture("clamp_release.png")
            report.update(
                passed=bool(finite and minimum_j > 0.05 and lift > 0.05 and abs(released) < 0.015),
                clamp_only=True,
                lift_m=lift,
                released_centre_delta_m=released,
                jaw_gap_m=0.030,
                commanded_lift_m=0.080,
                method="physical opposing horizontal plates; lower finger supplies support; no node pinning",
                limitations="support-assisted clamp proxy, not robot frictional grasp or calibrated slip",
                minimum_element_jacobian=minimum_j,
                finite=finite,
                object_sha256=hashlib.sha256((a.asset / "object.usdz").read_bytes()).hexdigest(),
            )
            return
        if a.mesh_only:
            view = obj.fem.root_physx_view
            simulation_rest = obj._rest_local[0]
            simulation_indices = view.get_sim_element_indices()[0].detach().cpu().numpy()
            collision_world = view.get_nodal_positions()[0].detach().cpu().numpy()
            collision_indices = view.get_element_indices()[0].detach().cpu().numpy()
            origin, _, rotation = obj._frame()
            collision_local = (collision_world - origin) @ rotation
            np.savez_compressed(
                a.output / "cooked_fem.npz",
                simulation_rest_local=simulation_rest,
                simulation_indices=simulation_indices,
                collision_initial_local=collision_local,
                collision_indices=collision_indices,
            )
            report.update(
                mesh_only=True,
                passed=True,
                object_sha256=hashlib.sha256((a.asset / "object.usdz").read_bytes()).hexdigest(),
                simulation_nodes=len(simulation_rest),
                simulation_tetrahedra=len(simulation_indices),
                collision_nodes=len(collision_local),
                collision_tetrahedra=len(collision_indices),
                simulation_mesh_sha256=hashlib.sha256(
                    np.asarray(simulation_rest, dtype="<f8").tobytes()
                    + np.asarray(simulation_indices, dtype="<i4").tobytes()
                ).hexdigest(),
                collision_snapshot_sha256=hashlib.sha256(
                    np.asarray(collision_local, dtype="<f8").tobytes()
                    + np.asarray(collision_indices, dtype="<i4").tobytes()
                ).hexdigest(),
                collision_snapshot_state="object-local coordinates after initialization warmup; not undeformed rest",
            )
            return
        if a.replay_only:
            episodes = []
            starts = []
            start_velocities = []
            for episode in range(2):
                obj.apply_saved_pose()
                starts.append(nodes().cpu().numpy().copy())
                start_velocities.append(float(obj.fem.root_physx_view.get_sim_nodal_velocities().abs().max().item()))
                trace = []
                for _ in range(720):
                    step(1)
                    trace.append(nodes().cpu().numpy().copy())
                episodes.append(np.asarray(trace))
            position_error = float(np.linalg.norm(episodes[1] - episodes[0], axis=-1).max())
            initial_error = float(np.linalg.norm(starts[1] - starts[0], axis=-1).max())
            report.update(
                replay_only=True,
                passed=bool(
                    finite
                    and minimum_j > 0.05
                    and position_error < 0.001
                    and initial_error < 1e-7
                    and max(start_velocities) < 1e-7
                ),
                object_sha256=hashlib.sha256((a.asset / "object.usdz").read_bytes()).hexdigest(),
                initial_position_error_m=initial_error,
                initial_velocities_m_s=start_velocities,
                replay_max_vertex_error_m=position_error,
                minimum_element_jacobian=minimum_j,
            )
            np.savez_compressed(a.output / "replay.npz", first=episodes[0], second=episodes[1])
            return
        if a.reload_only:
            from pxr import Usd

            asset_stage = Usd.Stage.Open(str(a.asset / "object.usdz"))
            mass_rows = []
            material_rows = []
            for prim in asset_stage.Traverse():
                for attr in prim.GetAttributes():
                    name = attr.GetName()
                    value = attr.Get()
                    if name == "physics:mass" and isinstance(value, int | float) and value > 0:
                        mass_rows.append({"prim": str(prim.GetPath()), "value_kg": float(value)})
                    if any(
                        word in name.lower() for word in ("friction", "youngsmodulus", "poissonsratio", "dampingscale")
                    ) and isinstance(value, int | float):
                        material_rows.append({"prim": str(prim.GetPath()), "attribute": name, "value": float(value)})
            step(180)
            from env.scene_manager.objects.tetra_visuals import apply_embedding

            current = obj.fem.root_physx_view.get_sim_nodal_positions()[0].detach().cpu().numpy()
            error = max(
                float(
                    np.linalg.norm(
                        np.array(b["mesh"].GetPointsAttr().Get())
                        - apply_embedding(current, b["indices"], b["weights"]),
                        axis=1,
                    ).max()
                )
                for b in obj._visual_bindings
            )
            report.update(
                reload_only=True,
                passed=bool(
                    finite
                    and minimum_j > 0.05
                    and len(obj._visual_bindings) == 8
                    and error < 1e-6
                    and abs(sum(row["value_kg"] for row in mass_rows) - 0.2402) < 1e-6
                ),
                object_sha256=hashlib.sha256((a.asset / "object.usdz").read_bytes()).hexdigest(),
                factory="SceneManager.instantiate_object_wrapper",
                wrapper=type(obj).__name__,
                usd_mass_properties=mass_rows,
                usd_material_parameters=material_rows,
                label_following_max_error_m=error,
                label_update_count=obj._visual_update_count,
            )
            return
        label_initial = [np.array(b["mesh"].GetPointsAttr().Get()) for b in obj._visual_bindings]
        report["label_binding_diagnostics"] = [b["diagnostics"] for b in obj._visual_bindings]
        multiview("rest_")
        step(720)
        (drop_height, drop_min) = (extent(), float(obj.fem.root_physx_view.get_nodal_positions()[..., 2].min().item()))
        multiview("settled_")
        capture("sim.png")
        phase = "compression"
        obj.default_pos = (0, 0, float(height / 2 + 0.002))
        obj.apply_saved_pose()
        step(240)
        before = extent()
        pose = plate.data.default_root_state[:, :7].clone()
        pose[:, 0] = 0.0
        support_z = float(obj.fem.root_physx_view.get_nodal_positions()[..., 2].min().item())
        (start, end) = (support_z + before + 0.045, support_z + before * 0.82 + 0.005)
        for i in range(360):
            pose[:, 2] = start + (end - start) * (i + 1) / 360
            plate.write_root_pose_to_sim(pose)
            step(1)
        step(240)
        compressed = extent()
        capture("compression.png")
        report["solid_blocking"] = dict(
            passed=bool(compressed < 0.95 * before and support_z > -0.004),
            method="physical plate obstructed by FEM packet against floor",
        )
        phase = "recovery"
        for i in range(240):
            pose[:, 2] = end + (start - end) * (i + 1) / 240
            plate.write_root_pose_to_sim(pose)
            step(1)
        step(720)
        recovered = extent()
        capture("recovery.png")
        rotation = Rotation.from_euler("z", 0.6)
        xyzw = rotation.as_quat()
        obj.default_pos = (0.11, -0.03, float(height / 2 + 0.12))
        obj.default_ori = (float(xyzw[3]), *map(float, xyzw[:3]))
        obj.apply_saved_pose()
        expected = obj._rest_local @ rotation.as_matrix().T + np.array(obj.default_pos)
        reset_error = float(np.max(abs(nodes().cpu().numpy() - expected)))
        velocity = float(obj.fem.root_physx_view.get_sim_nodal_velocities().abs().max().item())
        free = bool(torch.all(obj.fem.root_physx_view.get_sim_kinematic_targets()[..., 3] == 1))
        phase = "second_drop"
        (pose[:, 0], pose[:, 2]) = (0.5, 0.5)
        plate.write_root_pose_to_sim(pose)
        step(720)
        second_min = float(obj.fem.root_physx_view.get_nodal_positions()[..., 2].min().item())
        capture("reset_drop.png")
        report["usd_authored_mass_kg"] = float(meta["physics"]["mass"])
        report["factory"] = "SceneManager.instantiate_object_wrapper"
        actual_mass = (
            float(obj.fem.root_physx_view.get_sim_nodal_masses().sum().item())
            if hasattr(obj.fem.root_physx_view, "get_sim_nodal_masses")
            else None
        )
        from env.scene_manager.objects.tetra_visuals import apply_embedding

        current_nodes = obj.fem.root_physx_view.get_sim_nodal_positions()[0].detach().cpu().numpy()
        errors = []
        motion = []
        for binding, initial_label in zip(obj._visual_bindings, label_initial):
            actual = np.array(binding["mesh"].GetPointsAttr().Get())
            expected_label = apply_embedding(current_nodes, binding["indices"], binding["weights"])
            errors.append(float(np.linalg.norm(actual - expected_label, axis=1).max()))
            motion.append(float(np.linalg.norm(actual - initial_label, axis=1).max()))
        report["label_mesh_count"] = len(errors)
        report["label_following_max_error_m"] = max(errors)
        report["label_max_motion_m"] = max(motion)
        report["label_update_count"] = obj._visual_update_count
        report["whole_package_only"] = True
        report["thin_film_validated"] = False
        report["independent_rigid_lid_validated"] = False
        report.update(
            passed=bool(
                finite
                and minimum_j > 0.05
                and (drop_min > -0.004)
                and (0.7 * height < drop_height < 1.2 * height)
                and (compressed < 0.95 * before)
                and (0.8 * before < recovered < 1.15 * before)
                and (reset_error < 1e-06)
                and (velocity < 1e-06)
                and free
                and (second_min > -0.004)
            ),
            object_sha256=hashlib.sha256((a.asset / "object.usdz").read_bytes()).hexdigest(),
            wrapper=type(obj).__name__,
            physics_dt_s=1 / 240,
            nodes=int(nodes().shape[-2]),
            finite=finite,
            minimum_element_jacobian=minimum_j,
            stage_minimum_jacobians=minima,
            jacobian_checked_every_step=True,
            mass_kg=actual_mass,
            drop=dict(height_m=drop_height, min_z_m=drop_min, steps=720),
            compression=dict(
                before_height_m=before,
                compressed_height_m=compressed,
                plate_gap_m=end - 0.005,
                ramp_steps=360,
                hold_steps=240,
                method="physical rigid plate contact, no node pinning",
            ),
            recovery=dict(height_m=recovered, steps=720, release_ramp_steps=240),
            reset=dict(
                max_nodal_error_m=reset_error,
                max_velocity_m_s=velocity,
                all_nodes_free=free,
                second_drop_min_z_m=second_min,
            ),
        )
        report["passed"] = bool(
            report["passed"] and len(errors) == 8 and max(errors) < 1e-6 and obj._visual_update_count >= 3000
        )
        pose[:, 0] = 1.0
        pose[:, 2] = 0.5
        plate.write_root_pose_to_sim(pose)
        step(1)
        # Macro views preserve the actual printed photo pixels.
        for binding in obj._visual_bindings:
            if not binding["mesh"].GetPrim().GetName().startswith("label_"):
                continue
            p = np.array(binding["mesh"].GetPointsAttr().Get())
            center = (p.min(0) + p.max(0)) / 2
            span = max(np.ptp(p, axis=0)[:2])
            distance = max(0.12, span * 1.8)
            set_camera_view(center + np.array([0, -0.001, distance]), center, camera_prim_path=camera.prim_path)
            capture(binding["mesh"].GetPrim().GetName() + "_closeup.png")
    except Exception:
        report["error"] = traceback.format_exc()
        traceback.print_exc()
    finally:
        if obj is not None:
            from isaacsim.core.simulation_manager import SimulationManager

            callback_id = obj._visual_callback_id
            obj.close_visual_meshes()
            report["visual_callback_cleaned"] = callback_id not in SimulationManager._callbacks
        (a.output / "report.json").write_text(json.dumps(report, indent=2) + "\n")
        print(json.dumps(report), flush=True)
        app.close(skip_cleanup=True)


if __name__ == "__main__":
    main()
