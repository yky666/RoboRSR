"""Generate cloth-over-sphere and sphere-impact-soft-block contact demonstrations."""

import argparse
import json
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from check_deformable import check
from make_deformable import build


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--genie-root", type=Path, required=True)
    p.add_argument("--device", default="cpu")
    p.add_argument("--frames", type=int, default=240)
    p.add_argument("--case", choices=("all", "cloth_over_sphere", "sphere_soft_block"), default="all")
    a = p.parse_args()
    out = a.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    definitions = [
        ("cloth_over_sphere", "cloth", [500, 500], 30, 80),
        ("sphere_soft_block", "volume", [160, 160, 160], 6, 300),
    ]
    reports = {}
    for name, family, size, cells, mass in definitions:
        if a.case != "all" and a.case != name:
            continue
        spec = {
            "id": ("Garment" if family == "cloth" else "Dynamic") + "/demo_" + name + "/00000",
            "name_en": name,
            "estimated_mass_g": mass,
            "mass_policy": "estimate",
            "deformable": {
                "family": family,
                "rest_state": "synthetic primitive",
                "parameter_source": "candidate demonstration; not fitted to a real material",
                "geometry": {"size_mm": size, "cells": cells},
            },
        }
        if family == "volume":
            spec["deformable"]["solver_parameters"] = {"young_pa": 3000, "poisson": 0.3}
        asset = build(spec, out / "assets", a.genie_root, a.device)
        config = json.loads((asset / "runtime_config.json").read_text())
        if family == "cloth":
            config["objects"][0]["pose_xyzw"] = [0, 0, 0.48, 0, 0, 0, 1]
            points = np.load(asset / "mesh.npz")["points"]
            config["objects"][0]["pins"] = np.flatnonzero(np.isclose(points[:, 1], points[:, 1].min())).tolist()
            config["bodies"] = [
                {
                    "name": "sphere",
                    "shape": "sphere",
                    "radius_m": 0.12,
                    "pose_xyzw": [0, 0.03, 0.16, 0, 0, 0, 1],
                    "kinematic": True,
                }
            ]
        else:
            config["bodies"] = [
                {
                    "name": "sphere",
                    "shape": "sphere",
                    "radius_m": 0.04,
                    "density": 930,
                    "pose_xyzw": [0, 0, 0.55, 0, 0, 0, 1],
                }
            ]
        reports[name] = check(asset, out / name, Path(sys.executable), a.genie_root, a.device, a.frames, config)
    (out / "summary.json").write_text(json.dumps(reports, indent=2))
    print(json.dumps(reports))
    return 0 if all(r["probe_passed"] for r in reports.values()) else 2


if __name__ == "__main__":
    raise SystemExit(main())
