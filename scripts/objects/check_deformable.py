"""Exercise generated candidates and record physical checks separately from replay diagnostics."""

import argparse
import copy
import json
from pathlib import Path
import sys

import numpy as np

sys.path[:0] = [
    str(Path(__file__).resolve().parents[2] / "src"),
    str(Path(__file__).resolve().parents[2] / "src/real2sim_platform"),
]
from deformables import registration_fingerprint

from real2sim_deformables.client import Client


def check(asset, output, python, genie_root, device="cpu", frames=180, config_override=None):
    output.mkdir(parents=True, exist_ok=False)
    config = copy.deepcopy(config_override or json.loads((asset / "runtime_config.json").read_text()))
    for obj in config["objects"]:
        if "usd" in obj:
            obj["usd"] = str((asset / obj["usd"]).resolve())
    cfg = output / "config.json"
    cfg.write_text(json.dumps(config, indent=2))
    trace, bodies, minimum_j = [], [], 1.0
    with Client(python, cfg, genie_root, output / "worker.log", device, timeout=300) as client:
        initial = client.request("state")
        name = next(iter(initial["objects"]))
        for _ in range(frames):
            state = client.request("step")
            obj = state["objects"][name]
            trace.append(obj["points_m"])
            bodies.append([state["bodies"][body["name"]] for body in config.get("bodies", [])])
            minimum_j = min(minimum_j, obj.get("minimum_J", 1.0))
        overflow = state["contact_overflow"]
        reset = client.request("reset")
        p0 = np.asarray(initial["objects"][name]["points_m"])
        reset_error = float(np.max(np.abs(np.asarray(reset["objects"][name]["points_m"]) - p0)))
        reset_velocity_error = float(np.max(np.abs(reset["objects"][name]["velocities_m_s"])))
        replay = []
        for _ in range(min(60, frames)):
            replay.append(client.request("step")["objects"][name]["points_m"])
        replay_error = np.linalg.norm(np.asarray(replay) - np.asarray(trace[: len(replay)]), axis=-1)
    clean = client.process.returncode == 0
    trace = np.asarray(trace)
    # Remove translation and rotation when measuring shape change.
    centered = p0 - p0.mean(0)
    deformation = []
    for q in trace:
        q = q - q.mean(0)
        u, _, vt = np.linalg.svd(centered.T @ q)
        rot = u @ np.diag([1, 1, np.linalg.det(u @ vt)]) @ vt
        deformation.append(float(np.sqrt(np.mean(np.sum((centered @ rot - q) ** 2, axis=1)))))
    contact_control_separation = None
    node_control_separation = None
    if config.get("bodies"):
        negative = copy.deepcopy(config)
        for body in negative["bodies"]:
            body["particle_contact"] = False
        control_cfg = output / "no_particle_contact.json"
        control_cfg.write_text(json.dumps(negative, indent=2))
        negative_bodies, negative_nodes = [], []
        with Client(python, control_cfg, genie_root, output / "control.log", device, timeout=300) as control:
            for _ in range(frames):
                s = control.request("step")
                negative_bodies.append([s["bodies"][b["name"]] for b in config["bodies"]])
                negative_nodes.append(s["objects"][name]["points_m"])
        # Dynamic bodies can show two-way response; fixed obstacles cannot move.
        dynamic = [i for i, b in enumerate(config["bodies"]) if not b.get("kinematic", False)]
        if dynamic:
            contact_control_separation = float(
                np.linalg.norm(
                    np.asarray(bodies)[:, dynamic, :3] - np.asarray(negative_bodies)[:, dynamic, :3], axis=-1
                ).max()
            )
        else:
            contact_control_separation = None
        node_control_separation = float(np.linalg.norm(trace - np.asarray(negative_nodes), axis=-1).max())
        np.savez_compressed(output / "control_bodies.npz", poses=negative_bodies)
    spec = json.loads((asset / "spec.json").read_text())
    meta = json.loads((asset / "metadata.json").read_text())
    evidence = "report.json"

    def row(status):
        return {"status": status, "evidence": evidence}

    checks = {
        "geometry_mass": row(
            "pass" if np.isclose(initial["objects"][name]["mass_kg"], meta["physics"]["mass"]) else "fail"
        ),
        "deformation": row("pass" if np.isfinite(trace).all() and minimum_j > 0 else "fail"),
        "contact": row("pass" if node_control_separation is not None and node_control_separation > 1e-6 else "pending"),
        "self_collision": row("pending"),
        "reset_state": row("pass" if reset_error < 1e-6 and reset_velocity_error < 1e-6 else "fail"),
        "shutdown": row("pass" if clean else "fail"),
        "material_fit": row("pending"),
    }
    software = {
        "finite": bool(np.isfinite(trace).all()),
        "no_tet_inversion": minimum_j > 0,
        "ground_boundary": float(trace[..., 2].min()) > -0.005,
        "no_contact_overflow": not overflow,
        "state_reset": checks["reset_state"]["status"] == "pass",
        "clean_shutdown": clean,
    }
    report = {
        "asset_id": spec["id"],
        "asset_accepted": False,
        "probe_passed": all(software.values()),
        "software_checks": software,
        "checks": checks,
        "material_calibrated": False,
        "mesh_sha256": meta["deformable"]["mesh_sha256"],
        "registration_sha256": registration_fingerprint(
            spec["deformable"], spec.get("mass_g"), spec.get("dimensions_mm")
        ),
        "minimum_J": minimum_j,
        "minimum_node_z_m": float(trace[..., 2].min()),
        "shape_deformation_rms_max_m": max(deformation),
        "body_contact_control_separation_max_m": contact_control_separation,
        "node_contact_control_separation_max_m": node_control_separation,
        "diagnostics": {
            "replay": {
                "max_node_error_m": float(replay_error.max()),
                "rms_node_error_m": float(np.sqrt(np.mean(replay_error**2))),
                "frames": len(replay),
                "acceptance_threshold": None,
                "note": "No universal 1 mm threshold; task tolerances require an explicit protocol.",
            }
        },
        "frame_dt_s": config["frame_dt"],
        "frames": frames,
        "robot_task_validated": False,
        "full_self_intersection_audit": False,
    }
    np.savez_compressed(
        output / "nodes.npz",
        initial=p0,
        points=trace,
        faces=np.asarray(initial["objects"][name]["faces"]),
        bodies=np.asarray(bodies),
        deformation_rms_m=np.asarray(deformation),
    )
    (output / "report.json").write_text(json.dumps(report, indent=2))
    (output / "deformable_check.json").write_text(json.dumps(report, indent=2))
    return report


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--asset", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--python", type=Path, default=Path(sys.executable))
    p.add_argument("--genie-root", type=Path, default=Path("."))
    p.add_argument("--device", default="cpu")
    p.add_argument("--frames", type=int, default=180)
    a = p.parse_args()
    if not 1 <= a.frames <= 1200:
        p.error("frames must be 1..1200")
    report = check(a.asset.resolve(), a.output.resolve(), a.python, a.genie_root, a.device, a.frames)
    print(json.dumps(report))
    return 0 if report["probe_passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
