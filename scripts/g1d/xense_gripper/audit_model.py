"""Audit complete-body CAD-derived URDF structure and physical unit contracts."""

import argparse
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import trimesh
from urdf_tools import fk
import yaml


def audit(model_dir, base_urdf):
    root = ET.parse(model_dir / "g1d_aurora_lite_ws.urdf").getroot()
    original = ET.parse(base_urdf).getroot()
    links = {x.get("name"): x for x in root.findall("link")}
    joints = {x.get("name"): x for x in root.findall("joint")}
    issues = []
    for joint in original.findall("joint"):
        name = joint.get("name")
        if "dex1" in name or name.endswith("Wheel_Joint"):
            continue
        if name not in joints or ET.tostring(joint).strip() != ET.tostring(joints[name]).strip():
            issues.append("changed_g1d_joint:" + name)
    if any("dex1" in name for name in [*links, *joints]):
        issues.append("old_dex1_subtree_remains")
    transforms = fk(root)
    if set(transforms) != set(links):
        issues.append("disconnected_links")
    if len(links) != len(root.findall("link")) or len(joints) != len(root.findall("joint")):
        issues.append("duplicate_names")
    files = {}
    for mesh in root.iter("mesh"):
        relative = mesh.get("filename")
        path = (model_dir / relative).resolve()
        if not path.is_relative_to(model_dir.resolve()) or not path.is_file():
            issues.append("invalid_mesh_path:" + relative)
            continue
        data = path.read_bytes()
        if data.startswith(b"version https://git-lfs"):
            issues.append("unresolved_lfs_pointer:" + relative)
        files[relative] = hashlib.sha256(data).hexdigest()
    for name, link in links.items():
        inertial = link.find("inertial")
        if inertial is None:
            issues.append("missing_inertial:" + name)
            continue
        mass = float(inertial.find("mass").get("value"))
        tensor = inertial.find("inertia")
        matrix = np.array([[float(tensor.get("i" + a + b if a <= b else "i" + b + a)) for b in "xyz"] for a in "xyz"])
        eig = np.linalg.eigvalsh(matrix)
        if mass <= 0 or eig.min() <= 0 or not np.isfinite(matrix).all():
            issues.append("invalid_mass_or_inertia:" + name)
        if eig.max() > eig.sum() - eig.max() + 1e-8:
            issues.append("inertia_triangle_inequality:" + name)
    standalone = ET.parse(model_dir / "aurora_lite_ws.urdf").getroot()
    gel_meshes = {
        side: trimesh.load(model_dir / f"meshes/aurora/{side}_gel_visual.stl", force="mesh", process=False)
        for side in ("positive", "negative")
    }
    openings = []
    for opening in (0.0, 0.0425, 0.085):
        poses = fk(standalone, {"aurora_positive_jaw_joint": opening / 2})
        bounds = {}
        for side, mesh in gel_meshes.items():
            moved = mesh.copy()
            moved.apply_transform(poses[f"aurora_{side}_gel_link"])
            bounds[side] = moved.bounds
        gap = bounds["positive"][0, 0] - bounds["negative"][1, 0]
        if not np.isclose(gap, opening, atol=2e-6):
            issues.append(f"opening_mismatch:{opening}")
        openings.append({"command_opening_m": opening, "geometry_gap_m": float(gap)})
    side_configs = {}
    for side in ("left", "right"):
        config = yaml.safe_load((model_dir / f"{side}_robot_config.yml").read_text())
        if len(config["arm_joints_name"]) != 7 or any(n not in joints for n in config["arm_joints_name"]):
            issues.append("invalid_arm_config:" + side)
        if any(n not in joints for n in config["gripper_joints_name"]):
            issues.append("invalid_gripper_config:" + side)
        if any(c["link"] not in links or c["link"] != "head_link" for c in config["camera"]):
            issues.append("unverified_wrist_camera_inherited:" + side)
        side_configs[side] = {
            "arm_dof": len(config["arm_joints_name"]),
            "jaw_joint_count": len(config["gripper_joints_name"]),
            "independent_gripper_dof": 1,
            "camera_count": len(config["camera"]),
        }
    expected = json.loads((model_dir / "sha256.json").read_text())
    for relative, digest in expected.items():
        if hashlib.sha256((model_dir / relative).read_bytes()).hexdigest() != digest:
            issues.append("hash_mismatch:" + relative)
    return {
        "status": "passed" if not issues else "failed",
        "scope": (
            "URDF graph, mesh resolution/hashes, unchanged G1-D joint definitions, positive "
            "inertias, CAD gel gap at closed/half/open, side config bindings"
        ),
        "links": len(links),
        "joints": len(joints),
        "mesh_files": len(files),
        "opening_checks": openings,
        "side_configs": side_configs,
        "issues": issues,
        "manufacturer_urdf": False,
        "hardware_mount_verified": False,
        "dynamics_calibrated": False,
        "tactile_response_validated": False,
        "simulation_runtime_verified": False,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("model", type=Path)
    parser.add_argument("--base-urdf", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    report = audit(args.model.resolve(), args.base_urdf.resolve())
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps(report, ensure_ascii=False))
    raise SystemExit(0 if report["status"] == "passed" else 1)


if __name__ == "__main__":
    main()
