"""CAD-derived Aurora Lite / WS variant; calibration remains explicitly pending."""

import json
from pathlib import Path
import re

from isaaclab.actuators import ImplicitActuatorCfg
import isaaclab.sim as sim_utils
from isaaclab.sim.spawners.from_files import from_files
from pxr import UsdPhysics, UsdShade

from env.global_configs import ROBOTS_PATH
from env.robot_manager.robot_config.g1d import (
    LINK_MATERIALS,
    MATERIALS,
    RING_LINKS,
    _paint_ring,
    get_robot_config as _g1d_config,
)


@sim_utils.clone
def _spawn_g1d_aurora(prim_path, cfg, *args, **kwargs):
    prim = from_files.spawn_from_urdf.__wrapped__(prim_path, cfg, *args, **kwargs)
    # Isaac's URDF import does not consistently retain inline STL colours.
    # Reuse the G1-D visual finish and give the new CAD groups explicit PBR
    # bindings. These visual estimates do not change collision/contact physics.
    colors = {
        **MATERIALS,
        "aurora_base": sim_utils.PreviewSurfaceCfg(diffuse_color=(0.35, 0.37, 0.40), metallic=0.7, roughness=0.5),
        "aurora_black": sim_utils.PreviewSurfaceCfg(diffuse_color=(0.025, 0.03, 0.035), roughness=0.65),
        "aurora_gel": sim_utils.PreviewSurfaceCfg(diffuse_color=(0.12, 0.18, 0.23), roughness=0.65),
    }
    materials = {}
    for name, material_cfg in colors.items():
        path = f"{prim_path}/Looks/{name}"
        material_cfg.func(path, material_cfg)
        materials[name] = UsdShade.Material(prim.GetStage().GetPrimAtPath(path))
    for link in prim.GetChildren():
        visuals = link.GetChild("visuals")
        if not visuals:
            continue
        name = link.GetName()
        material = next((m for pattern, m in LINK_MATERIALS.items() if re.fullmatch(pattern, name)), None)
        if "_aurora_" in name:
            material = (
                "aurora_gel" if "_gel_link" in name else "aurora_base" if name.endswith("base_link") else "aurora_black"
            )
        if material is not None:
            visuals.SetInstanceable(False)
            sim_utils.bind_visual_material(str(visuals.GetPath()), str(materials[material].GetPath()))
            if re.fullmatch(RING_LINKS, name):
                _paint_ring(link, materials["black"])
    contract_path = Path(cfg.asset_path).parent / "physics_contract.json"
    contract = json.loads(contract_path.read_text())
    pairs = [(a, b) for a, others in contract["filtered_pairs"].items() for b in others]
    pairs += contract["additional_internal_slider_carrier_pairs"]
    for first, second in pairs:
        a, b = prim.GetChild(first), prim.GetChild(second)
        if not a.IsValid() or not b.IsValid():
            raise ValueError(f"Invalid Aurora collision-filter pair: {first}, {second}")
        UsdPhysics.FilteredPairsAPI.Apply(a).CreateFilteredPairsRel().AddTarget(b.GetPath())
    return prim


def get_robot_config():
    config = _g1d_config()
    config.spawn.func = _spawn_g1d_aurora
    config.spawn.asset_path = f"{ROBOTS_PATH}/g1d_aurora_lite_ws/g1d_aurora_lite_ws.urdf"
    config.spawn.usd_dir = f"{ROBOTS_PATH}/g1d_aurora_lite_ws/usd"
    config.init_state.joint_pos = {
        pattern: value for pattern, value in config.init_state.joint_pos.items() if "dex1" not in pattern
    }
    config.init_state.joint_pos[".*_aurora_.*_jaw_joint"] = 0.0425
    # Per-jaw limit from the product specification. Gains and contact response
    # are simulation estimates; this does not establish real force calibration.
    config.actuators["gripper"] = ImplicitActuatorCfg(
        joint_names_expr=[".*_aurora_.*_jaw_joint"],
        effort_limit_sim=40.0,
        velocity_limit_sim=0.07,
        stiffness=1e4,
        damping=100.0,
    )
    return config
