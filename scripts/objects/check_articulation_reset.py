"""Project-factory cable reset/replay regression, with all physical links checked.

The D6 chain is a proxy, not a calibrated rod or a robot grasp validation.
"""

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
    p.add_argument("--force-full-reset", action="store_true")
    p.add_argument("--parent-frame", action="store_true")
    p.add_argument("--steps", type=int, default=720)
    p.add_argument("--cycles", type=int, default=2)
    a = p.parse_args()
    a.output.mkdir(parents=True, exist_ok=False)
    from isaacsim import SimulationApp

    app = SimulationApp({"headless": True, "fast_shutdown": True})
    report = {"passed": False, "scope": "D6 proxy reset only", "material_calibrated": False}
    try:
        from isaacsim.core.api import World
        from isaacsim.core.api.objects import FixedCuboid
        from omegaconf import OmegaConf
        import torch

        from env.scene_manager.scene_manager import SceneManager

        world = World(physics_dt=1 / 240, rendering_dt=1 / 30, backend="torch", device="cuda:0")
        world.get_physics_context().enable_gpu_dynamics(True)
        world.get_physics_context().set_broadphase_type("GPU")
        world.scene.add(
            FixedCuboid(
                prim_path="/World/Floor",
                name="floor",
                position=np.array([0, 0, -0.01]),
                scale=np.array([7, 7, 0.02]),
                size=1,
            )
        )
        if a.parent_frame:
            from isaacsim.core.utils.stage import get_current_stage
            from pxr import UsdGeom

            parent = UsdGeom.Xform.Define(get_current_stage(), "/World/env_0")
            parent.AddTranslateOp().Set((1.0, 0.5, 0.0))
            parent.AddRotateZOp().Set(30.0)
        meta = json.loads((a.asset / "metadata.json").read_text())
        factory = SceneManager.__new__(SceneManager)
        factory.sim = SimpleNamespace(scene=SimpleNamespace(env_origins=torch.zeros((1, 3), device="cuda:0")))
        obj = factory.instantiate_object_wrapper(
            obj_type="articulation",
            prim_path="/World/env_0/Object/Articulation/cable_00000_1",
            usd_path=str((a.asset / "object.usdz").resolve()),
            primitive_type=None,
            env_id=0,
            default_pos=(-meta["geometry"]["center"][0], 0, 0.22),
            default_ori=(1, 0, 0, 0),
            scale=(1, 1, 1),
            inst_cfg=OmegaConf.create({"physics": {"reset_full_state": True}} if a.force_full_reset else {}),
        )
        world.reset()
        obj.initialize()
        view = obj._articulation_view._physics_view

        def arr(x):
            return x.detach().cpu().numpy().copy()

        def links():
            return arr(view.get_link_transforms())

        obj.apply_saved_pose()
        initial = links()
        report.update(
            factory="SceneManager.instantiate_object_wrapper",
            links=initial.shape[1],
            dofs=len(obj.dof_names),
            object_sha256=hashlib.sha256((a.asset / "object.usdz").read_bytes()).hexdigest(),
            runtime_sha256=hashlib.sha256((a.asset / "runtime_config.json").read_bytes()).hexdigest(),
            mass_kg=float(view.get_masses().sum().item()),
            effective_full_reset=bool(obj.physics_cfg["reset_full_state"]),
            full_reset_source="explicit test override" if a.force_full_reset else "asset runtime defaults",
            initial_max_link_speed=float(abs(arr(view.get_link_velocities())).max()),
            initialized_physics_root=arr(view.get_root_transforms()).tolist(),
            instance_default_pos=list(obj.default_pos),
            code_sha256={
                str(p.relative_to(Path(__file__).resolve().parents[2])): hashlib.sha256(p.read_bytes()).hexdigest()
                for p in [
                    Path(__file__).resolve().parents[2] / "env/scene_manager/objects" / n
                    for n in ["articulation.py", "articulation_reset.py", "asset_defaults.py"]
                ]
            },
        )
        cycles = []
        trajectories = []
        for cycle in range(a.cycles):
            states = []
            for _ in range(a.steps):
                world.step(render=False)
                states.append(links())
            trajectories.append(np.array(states))
            # Perturb targets and velocities as well as testing return from a deformed pose.
            idx = torch.tensor([0], device="cuda:0", dtype=torch.int32)
            view.set_dof_velocities(torch.ones_like(view.get_dof_velocities()) * 0.25, idx)
            view.set_root_velocities(torch.ones_like(view.get_root_velocities()) * 0.2, idx)
            obj.apply_saved_pose()
            after = links()
            err = float(np.linalg.norm(after[..., :3] - initial[..., :3], axis=-1).max())
            velocity = float(abs(arr(view.get_link_velocities())).max())
            target_error = float(
                abs(arr(view.get_dof_position_targets()) - arr(obj._full_reset_state["dof_position_targets"])).max()
            )
            cycles.append(
                dict(
                    position_error_m=err,
                    max_link_speed=velocity,
                    drive_target_error=target_error,
                    passed=err < 1e-5 and velocity < 1e-5 and target_error < 1e-6,
                )
            )
        replay = float(np.linalg.norm(trajectories[-1][..., :3] - trajectories[0][..., :3], axis=-1).max())
        # Relocation must preserve the selected physical-root offset, then return exactly.
        obj.relocate_offscreen()
        far = links()
        relocation = bool(abs(far[..., :3]).min() > 1000)
        obj.apply_saved_pose()
        returned = links()
        relocation_error = float(np.linalg.norm(returned[..., :3] - initial[..., :3], axis=-1).max())
        report.update(
            cycles=cycles,
            controlled_replay_max_error_m=replay,
            relocation_offscreen=relocation,
            return_from_relocation_error_m=relocation_error,
            physics_dt_s=1 / 240,
            steps=a.steps,
            mass_matches=abs(report["mass_kg"] - meta["physics"]["mass"]) < 1e-5,
            passed=all(c["passed"] for c in cycles) and replay < 0.001 and relocation and relocation_error < 1e-5,
        )
        np.savez_compressed(
            a.output / "states.npz",
            initial=initial,
            first=trajectories[0],
            replay=trajectories[-1],
            after_reset=returned,
        )
    except Exception:
        report["error"] = traceback.format_exc()
        print(report["error"], flush=True)
    finally:
        (a.output / "report.json").write_text(json.dumps(report, indent=2) + "\n")
        print(json.dumps(report), flush=True)
        app.close(skip_cleanup=True)


if __name__ == "__main__":
    main()
