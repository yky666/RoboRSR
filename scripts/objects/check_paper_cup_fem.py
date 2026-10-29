"""Deformation test of the FEM paper cup with official IsaacLab APIs (GPU pipeline).

Per episode: reset (``write_nodal_state_to_sim``), settle upright on a ground box, side squeeze at mid-height between
two PD-driven rigid plates (force = applied PD wrench, equal to the contact force when quasi-static), hold, release
and settle again. Geometry is measured from the collision-mesh nodes, which conform to the cup surface; element
inversion is checked from the simulation-mesh deformation gradients every step. A second episode must reproduce
the first. Raw numbers are written to ``report.json``; failures are kept.
"""

import argparse
import hashlib
import json
from pathlib import Path
import time
import traceback

import numpy as np

PLATE = (0.006, 0.06, 0.03)
PLATE_MASS = 0.1
KP, KD = 1000.0, 20.0
KR, KDR = 0.4, 0.008


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--asset", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--dt", type=float, default=1 / 300)
    p.add_argument("--travel-mm", type=float, default=20.0)
    p.add_argument("--speed-mm-s", type=float, default=10.0)
    p.add_argument("--squeeze-z-mm", type=float, default=68.0)
    p.add_argument("--episodes", type=int, default=2)
    p.add_argument("--no-render", action="store_true")
    a = p.parse_args()
    a.output.mkdir(parents=True, exist_ok=False)
    from isaacsim import SimulationApp

    app = SimulationApp({"headless": True})
    report = {"passed": False, "real_material_calibrated": False, "asset": str(a.asset)}
    try:
        run(a, report)
    except Exception:
        report["error"] = traceback.format_exc()
        traceback.print_exc()
    finally:
        (a.output / "report.json").write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n")
        print(json.dumps({k: v for k, v in report.items() if k != "episodes"}, ensure_ascii=False), flush=True)
        app.close(skip_cleanup=True)


def run(a, report):
    import carb

    # IsaacLab renders headless only with this flag (what AppLauncher sets for --enable_cameras).
    carb.settings.get_settings().set_bool("/isaaclab/render/offscreen", not a.no_render)
    from isaaclab.assets import DeformableObject, DeformableObjectCfg, RigidObject, RigidObjectCfg
    import isaaclab.sim as sim_utils
    import torch

    dt = a.dt
    sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=dt, device="cuda:0"))
    friction = sim_utils.RigidBodyMaterialCfg(static_friction=0.6, dynamic_friction=0.5)
    ground = sim_utils.CuboidCfg(
        size=(1.0, 1.0, 0.02),
        collision_props=sim_utils.CollisionPropertiesCfg(),
        physics_material=friction,
        visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.75, 0.75, 0.73)),
    )
    ground.func("/World/Ground", ground, translation=(0, 0, -0.01))
    sim_utils.DomeLightCfg(intensity=900).func("/World/Light", sim_utils.DomeLightCfg(intensity=900))
    distant = sim_utils.DistantLightCfg(intensity=1500, angle=1.0)
    distant.func("/World/Sun", distant, orientation=(0.92, 0.2, 0.3, 0.1))
    cup = DeformableObject(
        DeformableObjectCfg(
            prim_path="/World/Cup",
            spawn=sim_utils.UsdFileCfg(usd_path=str(a.asset)),
            init_state=DeformableObjectCfg.InitialStateCfg(pos=(0, 0, 0.001)),
        )
    )
    z_mid = a.squeeze_z_mm * 1e-3
    open_x = 0.06
    plates = []
    for i, sign in enumerate((1, -1)):
        plates.append(
            RigidObject(
                RigidObjectCfg(
                    prim_path=f"/World/Plate{i}",
                    spawn=sim_utils.CuboidCfg(
                        size=PLATE,
                        rigid_props=sim_utils.RigidBodyPropertiesCfg(
                            disable_gravity=True, linear_damping=0.0, angular_damping=0.0
                        ),
                        mass_props=sim_utils.MassPropertiesCfg(mass=PLATE_MASS),
                        collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=0.001, rest_offset=0.0),
                        physics_material=friction,
                        visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.35, 0.55, 0.85)),
                    ),
                    init_state=RigidObjectCfg.InitialStateCfg(pos=(sign * open_x, 0, z_mid)),
                )
            )
        )
    camera = None
    if not a.no_render:
        from isaacsim.core.utils.viewports import set_camera_view
        from isaacsim.sensors.camera import Camera

        camera = Camera("/World/Camera", resolution=(640, 720))
        set_camera_view((0.0, -0.36, 0.2), (0, 0, 0.065), camera_prim_path=camera.prim_path)
        camera.set_clipping_range(0.01, 10)
    sim.reset()
    if camera is not None:
        camera.initialize()
    view = cup.root_physx_view
    report["sim_nodes"] = int(cup.data.nodal_pos_w.shape[1])
    report["sim_elements"] = int(cup.data.sim_element_deform_gradient_w.shape[1])
    report["collision_nodes"] = int(view.get_nodal_positions().shape[1])
    report["collision_elements"] = int(view.get_element_indices().shape[1])
    report["physics_dt_s"] = dt
    report["device"] = sim.device
    report["asset_sha256"] = hashlib.sha256(a.asset.read_bytes()).hexdigest()
    report["bound_material"] = bound_material("/World/Cup")
    rest_collision = view.get_nodal_positions()[0].cpu().numpy()
    report["rest_geometry_mm"] = geometry(rest_collision, z_mid)
    default_nodal_state = cup.data.default_nodal_state_w.clone()

    state = {"finite": True, "min_jacobian": 1.0, "step_s": []}
    sign = torch.tensor([1.0, -1.0], device=sim.device)

    def plate_wrench(target_x, target_v):
        measured = []
        for i, plate in enumerate(plates):
            pos = plate.data.root_pos_w[0]
            vel = plate.data.root_lin_vel_w[0]
            target = torch.tensor([sign[i] * target_x, 0.0, z_mid], device=sim.device)
            target_vel = torch.tensor([sign[i] * target_v, 0.0, 0.0], device=sim.device)
            f = KP * (target - pos) + KD * (target_vel - vel)
            quat = plate.data.root_quat_w[0]
            rotvec = 2 * quat[1:] * torch.sign(quat[0])
            t = -KR * rotvec - KDR * plate.data.root_ang_vel_w[0]
            plate.set_external_force_and_torque(f.view(1, 1, 3), t.view(1, 1, 3), is_global=True)
            measured.append(float(-sign[i] * f[0]))
        return measured

    def step(target_x, timed=False, target_v=0.0):
        forces = plate_wrench(target_x, target_v)
        for plate in plates:
            plate.write_data_to_sim()
        cup.write_data_to_sim()
        t0 = time.perf_counter()
        sim.step(render=False)
        if timed:
            torch.cuda.synchronize()
            state["step_s"].append(time.perf_counter() - t0)
        cup.update(dt)
        for plate in plates:
            plate.update(dt)
        nodes = cup.data.nodal_pos_w
        grad = cup.data.sim_element_deform_gradient_w
        ok = bool(torch.isfinite(nodes).all()) and bool(torch.isfinite(grad).all())
        state["finite"] = state["finite"] and ok
        if ok:
            state["min_jacobian"] = min(state["min_jacobian"], float(torch.linalg.det(grad).min()))
        return forces

    def gap():
        return float(plates[0].data.root_pos_w[0, 0] - plates[1].data.root_pos_w[0, 0]) - PLATE[0]

    def snapshot(name):
        if camera is None:
            return None
        from PIL import Image

        rgba = None
        for i in range(40):
            sim.render()
            rgba = np.asarray(camera.get_rgba())
            # annotator output lags the stage by a few frames; never accept the first buffers
            if i >= 12 and rgba.ndim == 3 and rgba.shape[-1] >= 3 and rgba[..., :3].max() > 0:
                break
        if rgba is None or rgba.ndim != 3:
            return f"capture failed: shape {None if rgba is None else rgba.shape}"
        rgb = rgba[..., :3]
        if rgb.dtype != np.uint8:
            rgb = (np.clip(rgb, 0, 1) * 255).astype(np.uint8) if rgb.max() <= 1.0 else rgb.astype(np.uint8)
        path = a.output / f"{name}.png"
        Image.fromarray(rgb).save(path)
        return str(path)

    def collision_geometry():
        return geometry(view.get_nodal_positions()[0].cpu().numpy(), z_mid)

    def reset_plates():
        for i, plate in enumerate(plates):
            pose = plate.data.default_root_state.clone()
            pose[:, 0] = sign[i] * open_x
            plate.write_root_state_to_sim(pose)
            plate.reset()

    episodes = []
    for episode in range(a.episodes):
        ep = {}
        cup.write_nodal_state_to_sim(default_nodal_state)
        cup.reset()
        reset_plates()
        reset_err = (view.get_sim_nodal_positions()[0] - default_nodal_state[0, :, :3]).abs().max()
        ep["reset_max_nodal_error_m"] = float(reset_err)
        state["min_jacobian"] = 1.0
        for _ in range(round(1.0 / dt)):
            step(open_x)
        ep["settled"] = collision_geometry()
        ep["settled"]["min_jacobian_so_far"] = state["min_jacobian"]
        if episode == 0:
            ep["settled_png"] = snapshot("settled")

        # approach until both plates touch, then squeeze by the requested travel at constant speed
        v = a.speed_mm_s * 1e-3
        target = open_x
        gap0 = None
        curve = []
        steps = 0
        near = ep["settled"]["mouth_dx"] / 2e3 + PLATE[0] / 2
        while steps < round(8.0 / dt):
            steps += 1
            speed = v if gap0 is not None else 2 * v
            target -= speed * dt
            forces = step(target, timed=True, target_v=-speed)
            if gap0 is None and target < near and min(forces) > 0.05:
                gap0 = gap()
            if gap0 is not None:
                travel = gap0 - gap()
                if steps % 6 == 0:
                    curve.append({"travel_mm": travel * 1e3, "force_n": float(np.mean(forces))})
                if travel >= a.travel_mm * 1e-3:
                    break
        ep["contact_gap_mm"] = None if gap0 is None else gap0 * 1e3
        ep["force_travel_curve"] = curve
        hold_forces = []
        for i in range(round(1.0 / dt)):
            forces = step(target)
            hold_forces.append(float(np.mean(forces)))
        ep["hold"] = {
            "travel_mm": None if gap0 is None else (gap0 - gap()) * 1e3,
            "force_n_mean_last_0p2s": float(np.mean(hold_forces[-round(0.2 / dt) :])),
            "force_n_std_last_0p2s": float(np.std(hold_forces[-round(0.2 / dt) :])),
            "plate_forces_n": forces,
            "geometry": collision_geometry(),
            "min_jacobian": state["min_jacobian"],
        }
        if episode == 0:
            ep["squeezed_png"] = snapshot("squeezed")
        for i in range(round(0.5 / dt)):
            target = min(open_x, target + 2 * v * dt * 3)
            step(target)
        for _ in range(round(1.5 / dt)):
            step(open_x)
        ep["released"] = collision_geometry()
        ep["released"]["min_jacobian"] = state["min_jacobian"]
        if episode == 0:
            ep["released_png"] = snapshot("released")
        ep["finite"] = state["finite"]
        episodes.append(ep)

    report["episodes"] = episodes
    steps_s = np.asarray(state["step_s"])
    report["wall_clock_ms_per_step"] = {
        "mean": float(steps_s.mean() * 1e3),
        "p95": float(np.percentile(steps_s, 95) * 1e3),
        "real_time_factor": float(dt / steps_s.mean()),
        "n": int(len(steps_s)),
    }
    report["checks"] = evaluate(report, a.travel_mm)
    report["passed"] = all(c["passed"] for c in report["checks"].values())


def bound_material(root):
    """Physics material actually bound on the deformable mesh in the live stage, plus its deformable-body attrs."""
    from isaacsim.core.utils.stage import get_current_stage
    from pxr import PhysxSchema, Usd, UsdShade

    stage = get_current_stage()
    for prim in Usd.PrimRange(stage.GetPrimAtPath(root)):
        if prim.HasAPI(PhysxSchema.PhysxDeformableBodyAPI):
            out = {"mesh": str(prim.GetPath())}
            for attr in prim.GetAttributes():
                if attr.GetName().startswith("physxDeformable:") and not attr.GetName().endswith(
                    ("Points", "Indices", "Rotations", "Positions", "Velocities", "Poses")
                ):
                    out[attr.GetName()] = _plain(attr.Get())
            targets = UsdShade.MaterialBindingAPI(prim).GetDirectBindingRel("physics").GetTargets()
            for target in targets:
                mat = stage.GetPrimAtPath(target)
                out["material"] = str(target)
                for attr in mat.GetAttributes():
                    if attr.GetName().startswith("physxDeformableBodyMaterial:"):
                        out[attr.GetName()] = _plain(attr.Get())
            mass = prim.GetAttribute("physics:mass")
            out["physics:mass"] = _plain(mass.Get()) if mass else None
            return out
    return None


def _plain(value):
    if isinstance(value, (int, float, bool, str)) or value is None:
        return value
    return str(value)[:80]


def geometry(nodes, z_mid):
    """Cup geometry from surface nodes (mm): height, mouth/bottom diameters, tilt, mid-height cross-section."""
    p = nodes * 1e3
    z = p[:, 2]
    top = p[z > z.max() - 6]
    bottom = p[z < z.min() + 3]
    axis = top.mean(0) - bottom.mean(0)
    band = p[np.abs(z - (z_mid * 1e3 + 10)) < 4]

    def extent(q, k):
        return float(q[:, k].max() - q[:, k].min())

    return {
        "height": float(z.max() - z.min()),
        "z_min": float(z.min()),
        "mouth_dx": extent(top, 0),
        "mouth_dy": extent(top, 1),
        "bottom_dx": extent(bottom, 0),
        "bottom_dy": extent(bottom, 1),
        "section_dx": extent(band, 0),
        "section_dy": extent(band, 1),
        "section_z": float(z_mid * 1e3 + 10),
        "tilt_deg": float(np.degrees(np.arccos(axis[2] / np.linalg.norm(axis)))),
        "center_xy": [float(v) for v in p[:, :2].mean(0)],
    }


def evaluate(r, travel_mm):
    rest = r["rest_geometry_mm"]
    checks = {}
    eps = r["episodes"]
    e0 = eps[0]
    s = e0["settled"]
    checks["settle_upright_shape"] = {
        "passed": abs(s["height"] - rest["height"]) < 3
        and abs(s["mouth_dx"] - rest["mouth_dx"]) < 3
        and abs(s["mouth_dy"] - rest["mouth_dy"]) < 3
        and s["tilt_deg"] < 3
        and s["z_min"] > -2,
        "height_change_mm": s["height"] - rest["height"],
        "mouth_dx_change_mm": s["mouth_dx"] - rest["mouth_dx"],
        "mouth_dy_change_mm": s["mouth_dy"] - rest["mouth_dy"],
        "tilt_deg": s["tilt_deg"],
        "z_min_mm": s["z_min"],
    }
    h = e0["hold"]
    squeeze = s["section_dx"] - h["geometry"]["section_dx"]
    checks["squeeze"] = {
        "passed": h["travel_mm"] is not None
        and h["travel_mm"] >= travel_mm - 1
        and h["force_n_mean_last_0p2s"] > 0
        and squeeze > 0.5 * travel_mm
        and h["min_jacobian"] > 0,
        "travel_mm": h["travel_mm"],
        "hold_force_n": h["force_n_mean_last_0p2s"],
        "section_dx_reduction_mm": squeeze,
        "section_dy_increase_mm": h["geometry"]["section_dy"] - s["section_dy"],
        "min_jacobian": h["min_jacobian"],
    }
    rel = e0["released"]
    checks["recovery"] = {
        "passed": abs(rel["section_dx"] - s["section_dx"]) < 2
        and abs(rel["height"] - s["height"]) < 2
        and rel["tilt_deg"] < 3,
        "section_dx_residual_mm": rel["section_dx"] - s["section_dx"],
        "height_residual_mm": rel["height"] - s["height"],
        "mouth_dx_residual_mm": rel["mouth_dx"] - s["mouth_dx"],
        "tilt_deg": rel["tilt_deg"],
    }
    checks["no_inverted_elements_finite"] = {
        "passed": all(e["finite"] for e in eps) and min(e["released"]["min_jacobian"] for e in eps) > 0,
        "min_jacobian_per_episode": [e["released"]["min_jacobian"] for e in eps],
    }
    if len(eps) > 1:
        e1 = eps[1]
        checks["reset_second_episode"] = {
            "passed": e1["reset_max_nodal_error_m"] < 1e-5
            and abs(e1["hold"]["force_n_mean_last_0p2s"] - h["force_n_mean_last_0p2s"])
            < 0.1 * abs(h["force_n_mean_last_0p2s"]) + 0.05
            and abs(e1["settled"]["height"] - s["height"]) < 0.5,
            "reset_max_nodal_error_m": e1["reset_max_nodal_error_m"],
            "hold_force_n": [h["force_n_mean_last_0p2s"], e1["hold"]["force_n_mean_last_0p2s"]],
            "settled_height_mm": [s["height"], e1["settled"]["height"]],
        }
    return checks


if __name__ == "__main__":
    main()
