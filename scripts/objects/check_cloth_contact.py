"""Native particle-cloth contact diagnostic; only numerical settings may vary.

Preserve the measured mesh, area-weighted mass and material candidates. Report
sampled self-intersections explicitly; zero samples cannot prove continuous contact.
"""

import argparse
import hashlib
import json
from pathlib import Path
import sys
import traceback
from types import SimpleNamespace

import numpy as np


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--asset", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--dt", type=float, default=1 / 240)
    p.add_argument("--seconds", type=float, default=3)
    p.add_argument("--iterations", type=int, default=64)
    p.add_argument("--mesh-tools", type=Path)
    a = p.parse_args()
    a.output.mkdir(parents=True, exist_ok=False)
    if a.mesh_tools:
        sys.path.insert(0, str(a.mesh_tools))
    from isaacsim import SimulationApp
    import pymeshlab

    app = SimulationApp({"headless": True, "fast_shutdown": True})
    report = {"passed": False, "material_calibrated": False}
    try:
        from isaacsim.core.api import World
        from isaacsim.core.api.objects import FixedCuboid
        from omegaconf import OmegaConf
        import torch

        from env.scene_manager.scene_manager import SceneManager

        world = World(physics_dt=a.dt, rendering_dt=1 / 30, backend="torch", device="cuda:0")
        world.get_physics_context().enable_gpu_dynamics(True)
        world.get_physics_context().set_broadphase_type("GPU")
        world.scene.add(
            FixedCuboid(
                prim_path="/World/Floor",
                name="floor",
                size=1,
                position=np.array([0, 0, -0.01]),
                scale=np.array([2, 2, 0.02]),
            )
        )
        factory = SceneManager.__new__(SceneManager)
        factory.sim = SimpleNamespace(scene=SimpleNamespace(env_origins=torch.zeros((1, 3), device="cuda:0")))
        obj = factory.instantiate_object_wrapper(
            obj_type="garment",
            prim_path="/World/env_0/Object/Garment/long_zip_bag_00000_1",
            usd_path=str((a.asset / "object.usdz").resolve()),
            primitive_type=None,
            env_id=0,
            default_pos=(0, 0, 0.22),
            default_ori=(1, 0, 0, 0),
            scale=(1, 1, 1),
            inst_cfg=OmegaConf.create(
                {"physics": {"particle_system": {"solver_position_iteration_count": a.iterations}}}
            ),
        )
        obj.particle_system.set_simulation_owner(world.get_physics_context().prim_path)
        world.reset()
        obj.initialize()
        obj.apply_saved_pose()

        def positions():
            return obj._cloth_prim_view.get_world_positions().detach().cpu().numpy().reshape(-1, 3).copy()

        data = np.load(a.asset / "mesh.npz")
        faces = data["faces"]
        groups = data["groups"]
        edges = np.unique(
            np.sort(np.concatenate([faces[:, [0, 1]], faces[:, [1, 2]], faces[:, [2, 0]]]), axis=1), axis=0
        )
        initial = positions()
        rest = np.linalg.norm(initial[edges[:, 0]] - initial[edges[:, 1]], axis=1)
        mass = float(obj._cloth_prim_view.get_particle_masses().sum().item())
        samples = []
        frames = []
        max_strain = 0.0
        minimum_z = float(initial[:, 2].min())
        sample_every = max(1, round((1 / 60) / a.dt))
        for i in range(round(a.seconds / a.dt)):
            world.step(render=False)
            points = positions()
            if not np.isfinite(points).all():
                raise RuntimeError("Non-finite cloth state")
            minimum_z = min(minimum_z, float(points[:, 2].min()))
            max_strain = max(
                max_strain,
                float(abs(np.linalg.norm(points[edges[:, 0]] - points[edges[:, 1]], axis=1) / rest - 1).max()),
            )
            if i % sample_every == 0:
                ms = pymeshlab.MeshSet()
                ms.add_mesh(pymeshlab.Mesh(points, faces))
                ms.compute_selection_by_self_intersections_per_face()
                mask = ms.current_mesh().face_selection_array()
                samples.append(
                    dict(
                        step=i + 1,
                        time_s=(i + 1) * a.dt,
                        intersecting_faces=int(mask.sum()),
                        by_group={str(g): int(mask[groups == g].sum()) for g in np.unique(groups)},
                    )
                )
                frames.append(points)
        report.update(
            asset_id="Garment/long_zip_bag/" + a.asset.name,
            factory="SceneManager.instantiate_object_wrapper",
            object_sha256=hashlib.sha256((a.asset / "object.usdz").read_bytes()).hexdigest(),
            mass_kg=mass,
            physics_dt_s=a.dt,
            duration_s=a.seconds,
            solver_position_iterations=a.iterations,
            initial_mass_order="Area-weighted vertex_masses.npy with SHA validation",
            minimum_z_m=minimum_z,
            maximum_edge_strain=max_strain,
            samples=samples,
            maximum_intersecting_faces=max(s["intersecting_faces"] for s in samples),
            scope="60Hz sampled intersection diagnostic; not continuous collision acceptance",
            parameters_changed="dt and solver iterations only; no thickness, mass, stiffness or mesh alteration",
            passed=bool(not any(s["intersecting_faces"] for s in samples) and minimum_z > -0.008 and max_strain < 0.3),
        )
        np.savez_compressed(
            a.output / "sampled_states.npz", initial=initial, frames=np.array(frames), faces=faces, groups=groups
        )
    except Exception:
        report["error"] = traceback.format_exc()
        print(report["error"], flush=True)
    finally:
        (a.output / "report.json").write_text(json.dumps(report, indent=2) + "\n")
        print("CLOTH RESULT", report.get("passed"), report.get("maximum_intersecting_faces"), flush=True)
        app.close(skip_cleanup=True)


if __name__ == "__main__":
    main()
