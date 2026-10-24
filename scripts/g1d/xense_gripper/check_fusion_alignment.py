"""Compare actual pad meshes and frames at three physical openings (CPU only)."""

import argparse
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial import cKDTree
import trimesh

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
from env.robot_manager.aurora_contract import (  # noqa: E402
    CAD_TO_MOUNT_Z_M,
    measured_opening,
    mjcf_closure_positions,
    normalized_action,
    urdf_jaw_positions,
)
from scripts.g1d.xense_gripper.urdf_tools import fk, origin_transform  # noqa: E402


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=False)
    robot_dir = ROOT / "real2sim/robot/g1d_aurora_lite_ws"
    other = ROOT / "real2sim/grippers/aurora_fusion/mujoco_reference"
    urdf = ET.parse(robot_dir / "aurora_lite_ws.urdf").getroot()
    mjcf = ET.parse(other / "model.xml").getroot()
    meshes = {m.get("name"): m for m in mjcf.findall("asset/mesh")}
    rows, issues = [], []
    for opening in (0.0, 0.0425, 0.085):
        q_urdf = urdf_jaw_positions(opening)
        q_mjcf = mjcf_closure_positions(opening)
        transforms = fk(urdf, {"aurora_positive_jaw_joint": q_urdf[0]})
        surfaces, refs, frames, samples = {}, {}, {}, {}
        for sign, mj_side, direction, q_close in zip(("positive", "negative"), ("left", "right"), (-1, 1), q_mjcf):
            link = urdf.find(f"link[@name='aurora_{sign}_gel_link']")
            v = link.find("visual")
            mesh = trimesh.load(robot_dir / v.find("geometry/mesh").get("filename"), force="mesh", process=False)
            mesh.apply_transform(transforms[link.get("name")] @ origin_transform(v.find("origin")))
            surfaces[sign] = float(mesh.vertices[:, 0].min() if sign == "positive" else mesh.vertices[:, 0].max())
            m = meshes[f"{mj_side}_pad_0"]
            ref = trimesh.load(other / m.get("file"), force="mesh", process=False)
            ref.apply_scale(0.001)
            ref.apply_translation([direction * q_close, 0, CAD_TO_MOUNT_Z_M])
            refs[sign] = float(ref.vertices[:, 0].min() if sign == "positive" else ref.vertices[:, 0].max())
            t = transforms[f"aurora_{sign}_contact_frame"]
            on_face = np.max(np.abs(mesh.triangles[:, :, 0] - surfaces[sign]), axis=1) < 1e-6
            derived_normal = np.sum(mesh.face_normals[on_face] * mesh.area_faces[on_face, None], axis=0)
            derived_normal /= np.linalg.norm(derived_normal)
            if np.dot(derived_normal, t[:3, 2]) < 0.999999:
                issues.append(f"opening {opening}/{sign}: frame normal differs from actual mesh face")
            frames[sign] = dict(centre_m=t[:3, 3].tolist(), normal=t[:3, 2].tolist(), matrix=t.tolist())
            # Compare point-to-triangle surfaces, independent of tessellation density.
            # Dense samples include vertices and triangle centroids; nearest vertices
            # are reported separately rather than mistaken for exact surface error.
            samples[sign] = dict(
                urdf_bounds_m=mesh.bounds.tolist(),
                reference_bounds_m=ref.bounds.tolist(),
                max_bounds_delta_m=float(np.max(np.abs(mesh.bounds - ref.bounds))),
                reference_vertices_to_urdf_vertices_m=float(cKDTree(mesh.vertices).query(ref.vertices)[0].max()),
                inner_face_normal_from_mesh=derived_normal.tolist(),
            )
        gap = surfaces["positive"] - surfaces["negative"]
        ref_gap = refs["positive"] - refs["negative"]
        normal_dot = float(np.dot(frames["positive"]["normal"], frames["negative"]["normal"]))
        tcp = np.mean([frames[s]["centre_m"] for s in ("positive", "negative")], axis=0)
        if max(abs(gap - opening), abs(ref_gap - opening)) > 2e-6:
            issues.append(f"opening {opening}: actual surface gap mismatch")
        if normal_dot > -0.999999:
            issues.append(f"opening {opening}: contact normals do not oppose")
        if any(v["max_bounds_delta_m"] > 1e-5 for v in samples.values()):
            issues.append(f"opening {opening}: transformed pad bounds differ")
        rows.append(
            dict(
                opening_m=opening,
                normalized_action=normalized_action(opening),
                urdf_q_m=q_urdf,
                reference_closure_q_m=q_mjcf,
                urdf_surface_gap_m=gap,
                reference_surface_gap_m=ref_gap,
                measured_urdf_opening_m=measured_opening(*q_urdf),
                measured_reference_opening_m=measured_opening(*q_mjcf, convention="mjcf"),
                contact_frames=frames,
                opposing_normal_dot=normal_dot,
                tcp_m=tcp.tolist(),
                meshes=samples,
            )
        )
    report = dict(
        status="passed" if not issues else "failed",
        rows=rows,
        issues=issues,
        cad_to_mount_z_m=CAD_TO_MOUNT_Z_M,
        uses_actual_mesh_surfaces=True,
        reference_placeholder_dynamics_imported=False,
        wrist_mount_verified=False,
    )
    (args.out / "alignment.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(dict(status=report["status"], issues=issues)))
    return not issues


if __name__ == "__main__":
    raise SystemExit(0 if main() else 1)
