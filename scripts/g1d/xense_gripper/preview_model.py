"""Render CAD-derived URDF geometry on CPU; these are geometric previews, not simulation evidence."""

import argparse
import hashlib
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np
import trimesh
from urdf_tools import visuals


def draw(ax, items, elev=18, azim=-45):
    points = np.concatenate([m.vertices for _, m, _ in items])
    lo, hi = points.min(axis=0), points.max(axis=0)
    center, extent = (lo + hi) / 2, max(hi - lo) * 0.56
    for _, mesh, rgba in items:
        triangles = mesh.triangles
        normal = mesh.face_normals
        light = np.array([0.5, -0.4, 0.8])
        light /= np.linalg.norm(light)
        shade = 0.55 + 0.45 * np.abs(normal @ light)
        colors = np.column_stack([np.clip(shade[:, None] * rgba[:3], 0, 1), np.ones(len(shade))])
        ax.add_collection3d(Poly3DCollection(triangles, facecolors=colors, linewidth=0, rasterized=True))
    for i, setter in enumerate((ax.set_xlim, ax.set_ylim, ax.set_zlim)):
        setter(center[i] - extent, center[i] + extent)
    ax.set_box_aspect([1, 1, 1], zoom=1.35)
    ax.view_init(elev=elev, azim=azim)
    ax.set_axis_off()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("model", type=Path)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=False)
    q = {f"{side}_elbow_joint": -1.0 for side in ("left", "right")}
    q.update({f"{side}_aurora_positive_jaw_joint": 0.0425 for side in ("left", "right")})
    items = visuals(args.model / "g1d_aurora_lite_ws.urdf", q, simplify_faces=4500)
    fig = plt.figure(figsize=(11, 9), facecolor="#f4f6f8")
    draw(fig.add_subplot(111, projection="3d"), items)
    fig.suptitle("G1-D + dual Aurora Lite / G1-WS", fontsize=19, y=0.95)
    fig.text(
        0.5, 0.045, "Official CAD geometry | Proposed wrist adapter | Calibration pending", ha="center", fontsize=11
    )
    fig.savefig(args.out / "complete_robot.png", dpi=160, facecolor=fig.get_facecolor())
    plt.close(fig)
    scene = trimesh.Scene()
    for name, mesh, _ in items:
        scene.add_geometry(mesh, node_name=name, geom_name=name)
    scene.export(args.out / "complete_robot.glb")
    fig = plt.figure(figsize=(15, 6), facecolor="#f4f6f8")
    for i, opening in enumerate((0.0, 0.0425, 0.085)):
        gripper = visuals(
            args.model / "aurora_lite_ws.urdf", {"aurora_positive_jaw_joint": opening / 2}, simplify_faces=8000
        )
        ax = fig.add_subplot(1, 3, i + 1, projection="3d")
        draw(ax, gripper, elev=10, azim=-60)
        ax.set_title(f"{opening * 1000:g} mm opening", fontsize=15)
    fig.suptitle("Aurora Lite: matched factory WS sensors and CAD-derived motion", fontsize=17)
    fig.text(
        0.5, 0.04, "Rigid geometry preview; no simulated deformation or tactile response", ha="center", fontsize=11
    )
    fig.savefig(args.out / "gripper_openings.png", dpi=150, facecolor=fig.get_facecolor())
    plt.close(fig)
    (args.out / "preview_metadata.json").write_text(
        json.dumps(
            {
                "kind": "CPU URDF forward-kinematics geometry preview",
                "source_urdf_sha256": hashlib.sha256((args.model / "g1d_aurora_lite_ws.urdf").read_bytes()).hexdigest(),
                "preview_only_face_cap_per_link": 4500,
                "full_STL_reduced": False,
                "simulation_evidence": False,
                "joints_rad_or_m": q,
            },
            indent=2,
        )
        + "\n"
    )


if __name__ == "__main__":
    main()
