"""Build primitive cloth/soft-block candidates from a registered spec; never claim material calibration.

Run with the pinned Newton environment. Garments, bags and ropes need their own topology generators.
"""

import argparse
import hashlib
import json
from pathlib import Path
import re
import sys

import numpy as np
from pxr import Usd, UsdGeom, UsdUtils, Vt

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src"))
from real2sim_deformables.runtime import Runtime


def mesh_hash(points, faces):
    return hashlib.sha256(
        np.asarray(points, dtype="<f4").tobytes() + np.asarray(faces, dtype="<i4").tobytes()
    ).hexdigest()


def cloth_grid(size, cells):
    x, y = np.meshgrid(
        np.linspace(-size[0] / 2, size[0] / 2, cells + 1), np.linspace(-size[1] / 2, size[1] / 2, cells + 1)
    )
    points = np.c_[x.ravel(), y.ravel(), np.zeros(x.size)]
    faces = []
    for j in range(cells):
        for i in range(cells):
            k = j * (cells + 1) + i
            faces.extend([[k, k + 1, k + cells + 2], [k, k + cells + 2, k + cells + 1]])
    return points, np.array(faces, dtype=np.int32)


def write_mesh(path, points, faces):
    stage = Usd.Stage.CreateNew(str(path))
    UsdGeom.SetStageMetersPerUnit(stage, 1)
    UsdGeom.SetStageUpAxis(stage, "Z")
    root = UsdGeom.Xform.Define(stage, "/root")
    stage.SetDefaultPrim(root.GetPrim())
    mesh = UsdGeom.Mesh.Define(stage, "/root/model")
    mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(np.asarray(points, dtype=np.float32)))
    mesh.CreateFaceVertexCountsAttr([3] * len(faces))
    mesh.CreateFaceVertexIndicesAttr(np.asarray(faces).ravel().tolist())
    mesh.CreateSubdivisionSchemeAttr("none")
    mesh.CreateDoubleSidedAttr(True)
    stage.GetRootLayer().Save()


def build(spec, output, genie_root, device="cpu"):
    oid = spec["id"]
    if not re.fullmatch(r"\w+/\w+/[0-9]{5}", oid):
        raise ValueError("Expected an existing type/category/00000 asset id")
    profile = spec["deformable"]
    family = profile["family"]
    if family not in ("cloth", "soft_body"):
        raise ValueError(
            "This primitive generator supports cloth/soft_body only; preserve real seams and openings separately"
        )
    g = profile["geometry"]
    size = np.asarray(g["size_mm"], dtype=float) / 1000
    expected = 2 if family == "cloth" else 3
    if size.shape != (expected,) or not np.isfinite(size).all() or np.any(size <= 0):
        raise ValueError("Positive finite size_mm required (cloth: width,length; soft_body: width,length,height)")
    cells = g.get("cells", 24 if family == "cloth" else 6)
    if type(cells) is not int or not 2 <= cells <= 80:
        raise ValueError("cells must be an integer in 2..80")
    if family == "soft_body" and cells > 16:
        raise ValueError("Primitive soft blocks are limited to 16 cells per axis")
    mass_g = spec.get("mass_g", spec.get("estimated_mass_g"))
    if isinstance(mass_g, bool) or not isinstance(mass_g, int | float) or not np.isfinite(mass_g) or mass_g <= 0:
        raise ValueError("Provide a positive mass_g or explicitly estimated_mass_g; no invented mass")
    folder = output / oid
    folder.mkdir(parents=True, exist_ok=False)
    item = {"name": "object", "asset_id": oid, "mass_kg": mass_g / 1000}
    if family == "cloth":
        points, faces = cloth_grid(size, cells)
        write_mesh(folder / "cloth.usda", points, faces)
        item.update(
            kind="cloth",
            usd=str(folder / "cloth.usda"),
            prim_path="/root/model",
            pose_xyzw=[0, 0, float(size.max() / 2 + 0.15), 0, 0, 0, 1],
        )
        item["material"] = profile.get("solver_parameters", {})
    else:
        item.update(
            kind="soft_grid",
            size_m=size.tolist(),
            center_m=[0, 0, float(size[2] / 2 + 0.002)],
            cells=cells,
            **profile.get("solver_parameters", {}),
        )
    config = {"frame_dt": 1 / 60, "substeps": 12, "iterations": 16, "objects": [item], "bodies": []}
    runtime = Runtime(config, genie_root, device)
    initial = runtime.snapshot()["objects"]["object"]
    world = np.asarray(initial["points_m"])
    points = world - (world.min(0) + world.max(0)) / 2
    faces = np.asarray(initial["faces"])
    write_mesh(folder / "object.usda", points, faces)
    if not UsdUtils.CreateNewUsdzPackage(str(folder / "object.usda"), str(folder / "object.usdz")):
        raise RuntimeError("USDZ packaging failed")
    mesh_sha = mesh_hash(points, faces)
    # Store relative cloth references so the generated candidate can be moved.
    if family == "cloth":
        item["usd"] = "cloth.usda"
    (folder / "runtime_config.json").write_text(json.dumps(config, indent=2))
    (folder / "metadata.json").write_text(
        json.dumps(
            {
                "physics": {"mass": initial["mass_kg"], "size": np.ptp(points, axis=0).tolist()},
                "deformable": {"family": family, "mesh_sha256": mesh_sha, "material_calibrated": False},
                "measurements_mm": {},
                "parameter_source": profile.get("parameter_source", "candidate; not calibrated"),
                "mass_source": spec.get("mass_policy", "unspecified"),
                "asset_accepted": False,
            },
            indent=2,
        )
    )
    (folder / "description.json").write_text(json.dumps({"caption": {"name": [spec.get("name_en", family)]}}))
    (folder / "spec.json").write_text(json.dumps(spec, ensure_ascii=False, indent=2))
    np.savez_compressed(folder / "mesh.npz", points=points, faces=faces)
    return folder


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--spec", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--genie-root", type=Path, default=Path("."))
    p.add_argument("--device", default="cpu")
    a = p.parse_args()
    print(build(json.loads(a.spec.read_text()), a.output.resolve(), a.genie_root, a.device))


if __name__ == "__main__":
    main()
