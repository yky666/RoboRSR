"""Select and adapt GenieSim tabletop rigid assets; keep the original checkout and CC-BY-NC-SA attribution.

Download the original objects with git-lfs first. This tool never executes source item.py files.
    python scripts/objects/geniesim.py select --source SOURCE --manifest selection.json
    python scripts/objects/geniesim.py convert --source SOURCE --manifest selection.json --out OUTPUT
The USDZ wrapper references the pinned original checkout, which must remain available.
Conversion checks USD dependencies and physics structure; drop/render validation is a separate step.
"""

import argparse
import ast
import hashlib
import json
import math
from pathlib import Path
import re
import subprocess
import tempfile
import uuid

LICENSE = "CC-BY-NC-SA-4.0"
URL = "https://modelscope.cn/datasets/agibot_world/GenieSimAssets"


def read_json(path):
    return json.loads(Path(path).read_text())


def tabletop(parameters, item):
    if item.get("has_joint") or item.get("has_articulation"):
        return "articulated"
    size = parameters.get("size")
    if parameters.get("unit") != "m" or not isinstance(size, list) or len(size) != 3:
        return "missing metric dimensions"
    if not all(isinstance(v, (int, float)) and math.isfinite(v) and v > 0 for v in size):
        return "invalid dimensions"
    if not 0.03 <= max(size) <= 0.5 or min(size) < 0.003:
        return "outside tabletop size range"
    return None


def select(source):
    source = Path(source)
    chosen, dropped = [], []
    revision = subprocess.check_output(["git", "-C", str(source), "rev-parse", "HEAD"], text=True).strip()
    for path in sorted((source / "objects").rglob("object_parameters.json")):
        relative = path.parent.relative_to(source).as_posix()
        try:
            parameters = read_json(path)
            item = ast.literal_eval((path.parent / "item.py").read_text())
            reason = tabletop(parameters, item)
            if reason:
                dropped.append({"src": relative, "reason": reason})
                continue
            slug = re.sub(r"[^A-Za-z0-9_]", "_", relative.removeprefix("objects/"))
            chosen.append({"src": relative, "id": f"Rigid/geniesim_{slug}/00000"})
        except (OSError, ValueError, SyntaxError, TypeError) as exc:
            dropped.append({"src": relative, "reason": str(exc)})
    assert len({x["id"] for x in chosen}) == len(chosen), "duplicate asset IDs"
    return {"source": URL, "license": LICENSE, "revision": revision, "chosen": chosen, "dropped": dropped}


def choose_mass(authored, parameters):
    values = [float(v) for v in authored if v is not None and math.isfinite(v) and v > 0]
    if len(values) > 1:
        raise ValueError("multiple authored masses: needs compound-body review")
    value = values[0] if values else parameters.get("mass")
    if not isinstance(value, (int, float)) or not math.isfinite(value) or not 0.001 <= value <= 5:
        raise ValueError(f"invalid/unsupported tabletop mass: {value}")
    return {"mass": float(value), "mass_cap": max(0.5, float(value))}, {
        "usd_mass_kg": values[0] if values else None,
        "json_mass_kg": parameters.get("mass"),
        "selected": "authored USD mass" if values else "source JSON mass",
        "measured": False,
    }


def placement_metadata(size):
    """Conservative box geometry and a bottom placement frame, without invented grasp/contact frames."""
    corners = [
        [x, y, z]
        for x in (-size[0] / 2, size[0] / 2)
        for y in (-size[1] / 2, size[1] / 2)
        for z in (-size[2] / 2, size[2] / 2)
    ]
    radius = math.hypot(size[0], size[1]) / 2
    box = {"extents": list(size), "vertices": corners}
    return {"aligned_bbox": box, "oriented_bbox": box, "radius": radius}, {
        "place": {
            "up": {
                "orientation": [1, 0, 0, 0],
                "projection_circle": {"center": [0, 0, -size[2] / 2, 1, 0, 0, 0], "radius": radius},
            }
        }
    }


def missing_dependencies(path):
    from pxr import Sdf, UsdUtils

    _, _, unresolved = UsdUtils.ComputeAllDependencies(Sdf.AssetPath(str(path)))
    # These shader modules ship with Isaac Sim, rather than with individual object packages.
    builtins = {"OmniPBR.mdl", "OmniPBR_Opacity.mdl", "OmniGlass.mdl", "OmniSurface.mdl", "OmniSurfaceBase.mdl"}
    return [str(p) for p in unresolved if str(p) not in builtins]


def convert_one(source, entry, out, revision):
    import numpy as np
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

    source = Path(source).resolve()
    folder = (source / entry["src"]).resolve()
    if not folder.is_relative_to(source):
        raise ValueError("source path escapes checkout")
    native = folder / "Aligned.usda"
    if missing := missing_dependencies(native):
        raise ValueError(f"unresolved dependencies: {missing}")
    parameters = read_json(folder / "object_parameters.json")
    original = Usd.Stage.Open(str(native))
    if not original or not original.GetDefaultPrim():
        raise ValueError("missing USD default prim")
    for prim in original.Traverse():
        if prim.IsA(UsdPhysics.Joint) or prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            raise ValueError("USD contains articulation/joint despite rigid annotation")
    physics, mass_source = choose_mass(
        [UsdPhysics.MassAPI(p).GetMassAttr().Get() for p in original.Traverse() if p.HasAPI(UsdPhysics.MassAPI)],
        parameters,
    )
    materials = [UsdPhysics.MaterialAPI(p) for p in original.Traverse() if p.HasAPI(UsdPhysics.MaterialAPI)]
    for key, getter in [("static_friction", "GetStaticFrictionAttr"), ("dynamic_friction", "GetDynamicFrictionAttr")]:
        values = [getattr(material, getter)().Get() for material in materials]
        values = [float(v) for v in values if v is not None and math.isfinite(v) and v >= 0]
        if not values or max(values) - min(values) > 1e-6:
            raise ValueError(f"missing or conflicting {key}: {values}")
        physics[key] = values[0]
    target = Path(out) / "Object/RoboDojo" / entry["id"]
    target.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory() as temporary:
        wrapper = Path(temporary) / "object.usda"
        stage = Usd.Stage.CreateNew(str(wrapper))
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 1)
        world = UsdGeom.Xform.Define(stage, "/World")
        stage.SetDefaultPrim(world.GetPrim())
        normalizer = UsdGeom.Xform.Define(stage, "/World/normalize")
        metric = UsdGeom.Xform.Define(stage, "/World/normalize/metric")
        visual = UsdGeom.Xform.Define(stage, "/World/normalize/metric/asset")
        visual.GetPrim().GetReferences().AddReference(str(native))
        # Use the USD stage's axis; Aligned.usda may already rotate Y-up source meshes to Z-up.
        axis = UsdGeom.GetStageUpAxis(original)
        rotation = np.eye(3) if axis == UsdGeom.Tokens.z else np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]])
        unit = UsdGeom.GetStageMetersPerUnit(original)
        cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
        bounds = cache.ComputeWorldBound(original.GetDefaultPrim()).ComputeAlignedRange()
        lo, hi = np.asarray(bounds.GetMin()), np.asarray(bounds.GetMax())
        corners = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
        corners = corners @ rotation.T * unit
        lo, hi = corners.min(0), corners.max(0)
        size, center = hi - lo, (lo + hi) / 2
        if not np.isfinite(size).all() or not 0.03 <= max(size) <= 0.5 or min(size) < 0.003:
            raise ValueError(f"USD bounds outside tabletop range: {size.tolist()}")
        # A parent translation works even if the referenced default prim already has xform ops.
        normalizer.AddTranslateOp().Set(Gf.Vec3d(*map(float, -center)))
        if axis != UsdGeom.Tokens.z or unit != 1:
            transform = Gf.Matrix4d(1)
            if axis != UsdGeom.Tokens.z:
                transform.SetRotate(Gf.Rotation(Gf.Vec3d(1, 0, 0), 90))
            transform = Gf.Matrix4d().SetScale(unit) * transform
            metric.AddTransformOp(opSuffix="metric_z_up").Set(transform)
        colliders = 0
        for prim in list(stage.Traverse()):
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                prim.RemoveAPI(UsdPhysics.RigidBodyAPI)
            if prim.HasAPI(UsdPhysics.MassAPI):
                prim.RemoveAPI(UsdPhysics.MassAPI)
            if prim.IsA(UsdGeom.Camera) or "Light" in prim.GetTypeName():
                prim.SetActive(False)
            if prim.HasAPI(UsdPhysics.CollisionAPI) and UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get():
                colliders += 1
        if not colliders:
            raise ValueError("no enabled collision geometry")
        UsdPhysics.RigidBodyAPI.Apply(world.GetPrim())
        UsdPhysics.MassAPI.Apply(world.GetPrim()).CreateMassAttr(physics["mass"])
        material = UsdShade.Material.Define(stage, "/World/import_physics")
        contact = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
        contact.CreateStaticFrictionAttr(physics["static_friction"])
        contact.CreateDynamicFrictionAttr(physics["dynamic_friction"])
        UsdShade.MaterialBindingAPI.Apply(world.GetPrim()).Bind(material, materialPurpose="physics")
        stage.GetRootLayer().Save()
        package = target / "object.usdz"
        with Sdf.ZipFileWriter.CreateNew(str(package)) as archive:
            if not archive.AddFile(str(wrapper), "object.usda"):
                raise ValueError("USDZ wrapper packaging failed")
    if missing := missing_dependencies(package):
        raise ValueError(f"packaged dependency missing: {missing}")
    packaged = Usd.Stage.Open(str(package))
    bodies = [p.GetPath().pathString for p in packaged.Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI)]
    if bodies != ["/World"]:
        raise ValueError(f"nested rigid bodies remain: {bodies}")
    bound = cache.ComputeWorldBound(packaged.GetDefaultPrim()).ComputeAlignedRange()
    if np.linalg.norm(np.asarray(bound.GetMidpoint())) > 1e-5:
        raise ValueError("packaged origin is not the bounding-box centre")
    geometry, active = placement_metadata(size.tolist())
    source_info = {
        "dataset": "agibot_world/GenieSimAssets",
        "url": URL,
        "revision": revision,
        "object": entry["src"],
        "license": LICENSE,
        "attribution": "Genie Sim Team / AgiBot",
        "dependency_root": str(source),
        "packaging": "USDZ wrapper; original checkout must remain available",
        "mass": mass_source,
        "axis": {"usd": str(axis), "json": parameters.get("upAxis"), "meters_per_unit": unit},
        "validation": {"dependencies": "pass", "single_rigid_body": "pass", "collision_geometry": "pass"},
    }
    metadata = {
        "uuid": str(uuid.uuid5(uuid.NAMESPACE_URL, f"{URL}/{entry['src']}")),
        "geometry": geometry,
        "active": active,
        "physics": {**physics, "size": size.tolist()},
        "source": source_info,
    }
    description = {
        "caption": {"name": [folder.name]},
        "names": {"en": [parameters.get("category") or folder.name.replace("_", " ")]},
    }
    for name, value in [("metadata.json", metadata), ("description.json", description)]:
        (target / name).write_text(json.dumps(value, indent=2, ensure_ascii=False) + "\n")
    return {
        "id": entry["id"],
        "src": entry["src"],
        "dir": str(target),
        "size_m": size.tolist(),
        "mass_kg": physics["mass"],
        "source": source_info,
        "sha256": hashlib.sha256(package.read_bytes()).hexdigest(),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("select", "convert"))
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--out", type=Path)
    args = parser.parse_args()
    if args.command == "select":
        result = select(args.source)
        args.manifest.write_text(json.dumps(result, indent=2) + "\n")
        print(f"selected={len(result['chosen'])}, excluded={len(result['dropped'])}")
        return
    if args.out is None:
        parser.error("convert requires --out")
    manifest = read_json(args.manifest)
    result = {"revision": manifest["revision"], "license": LICENSE, "converted": [], "failed": []}
    args.out.mkdir(parents=True, exist_ok=True)
    for entry in manifest["chosen"]:
        try:
            result["converted"].append(convert_one(args.source, entry, args.out, manifest["revision"]))
        except (OSError, ValueError, RuntimeError, TypeError) as exc:
            result["failed"].append({**entry, "error": str(exc)})
        (args.out / "conversion.json").write_text(json.dumps(result, indent=2) + "\n")
        print(f"converted={len(result['converted'])}, failed={len(result['failed'])}", flush=True)


if __name__ == "__main__":
    main()
