"""Extract visual / collision meshes and stats from the local object USDs (needs pxr; starts Kit if necessary).

Usage (on 5090-2, RoboDojo env):
    python extract_objects.py SRC_DIR objects.json OUT_DIR

Writes OUT_DIR/meshes/<scene>__<id>_vis.ply (face colors) and _col.ply, and OUT_DIR/stats.json.
All geometry is converted to metres, Z up.
"""

import json
import os
import sys
import traceback

try:
    from pxr import Sdf, Usd, UsdGeom, UsdPhysics, UsdShade, UsdUtils

    APP = None
except ImportError:
    from isaacsim import SimulationApp

    APP = SimulationApp({"headless": True})
    from pxr import Sdf, Usd, UsdGeom, UsdPhysics, UsdShade, UsdUtils

import numpy as np
from PIL import Image
import trimesh

COLOR_INPUTS = ("diffuseColor", "diffuse_color_constant", "base_color", "diffuse_tint", "baseColor", "albedo")
OPACITY_INPUTS = ("opacity", "opacity_constant", "transmission_weight")
TEXTURE_INPUTS = ("diffuse_texture", "file", "base_color_texture", "albedo_texture", "diffuseColor_texture")
_tex_cache = {}


def load_texture(path):
    if path not in _tex_cache:
        try:
            _tex_cache[path] = np.asarray(Image.open(path).convert("RGB"), dtype=np.uint8)
        except Exception:
            _tex_cache[path] = None
    return _tex_cache[path]


def material_info(material):
    """Base color, opacity, texture path and shader ids of a bound material."""
    info = {"name": None, "color": None, "opacity": 1.0, "texture": None, "texture_exists": None, "shaders": []}
    if not material:
        return info
    info["name"] = material.GetPrim().GetName()
    for prim in Usd.PrimRange(material.GetPrim()):
        if not prim.IsA(UsdShade.Shader):
            continue
        shader = UsdShade.Shader(prim)
        sid = shader.GetIdAttr().Get()
        if not sid:
            src = prim.GetAttribute("info:mdl:sourceAsset")
            sid = str(src.Get().path) if src and src.Get() else "mdl"
        info["shaders"].append(str(sid))
        for inp in shader.GetInputs():
            name = inp.GetBaseName()
            val = inp.Get()
            if val is None:
                continue
            if name in COLOR_INPUTS and info["color"] is None and hasattr(val, "__len__") and len(val) >= 3:
                info["color"] = [float(val[0]), float(val[1]), float(val[2])]
            elif name in OPACITY_INPUTS and isinstance(val, (int, float)):
                info["opacity"] = float(1.0 - val) if name == "transmission_weight" else float(val)
            elif isinstance(val, Sdf.AssetPath) and (name in TEXTURE_INPUTS or "tex" in name.lower()):
                if info["texture"] is None:
                    info["texture"] = val.resolvedPath or val.path
                    info["texture_exists"] = bool(val.resolvedPath) and os.path.exists(val.resolvedPath)
    return info


def gprim_to_mesh(prim):
    """Local-space (vertices, triangles, per-triangle source face index, uv per face-vertex or None)."""
    if prim.IsA(UsdGeom.Mesh):
        mesh = UsdGeom.Mesh(prim)
        pts = np.asarray(mesh.GetPointsAttr().Get() or [], dtype=float).reshape(-1, 3)
        counts = np.asarray(mesh.GetFaceVertexCountsAttr().Get() or [], dtype=int)
        idx = np.asarray(mesh.GetFaceVertexIndicesAttr().Get() or [], dtype=int)
        if len(pts) == 0 or len(counts) == 0:
            return None
        tris, src, corner = [], [], []
        start = 0
        for f, c in enumerate(counts):
            for k in range(1, c - 1):
                tris.append((idx[start], idx[start + k], idx[start + k + 1]))
                corner.append((start, start + k, start + k + 1))
                src.append(f)
            start += c
        uv = None
        pv = UsdGeom.PrimvarsAPI(prim)
        for nm in ("st", "UVMap", "uv", "st0", "map1"):
            p = pv.GetPrimvar(nm)
            if p and p.HasValue():
                vals = np.asarray(p.Get(), dtype=float).reshape(-1, 2)
                interp = p.GetInterpolation()
                pidx = p.GetIndices()
                if pidx:
                    vals = vals[np.asarray(pidx, dtype=int)]
                if interp == UsdGeom.Tokens.faceVarying and len(vals) == len(idx):
                    uv = vals[np.asarray(corner)]
                elif interp in (UsdGeom.Tokens.vertex, UsdGeom.Tokens.varying) and len(vals) == len(pts):
                    uv = vals[np.asarray(tris)]
                break
        return pts, np.asarray(tris, dtype=int), np.asarray(src, dtype=int), uv
    axis_rot = {
        "X": trimesh.transformations.rotation_matrix(np.pi / 2, [0, 1, 0]),
        "Y": trimesh.transformations.rotation_matrix(-np.pi / 2, [1, 0, 0]),
        "Z": np.eye(4),
    }
    if prim.IsA(UsdGeom.Cube):
        s = UsdGeom.Cube(prim).GetSizeAttr().Get() or 2.0
        tm = trimesh.creation.box(extents=[s, s, s])
    elif prim.IsA(UsdGeom.Sphere):
        tm = trimesh.creation.icosphere(subdivisions=3, radius=UsdGeom.Sphere(prim).GetRadiusAttr().Get() or 1.0)
    elif prim.IsA(UsdGeom.Cylinder):
        g = UsdGeom.Cylinder(prim)
        tm = trimesh.creation.cylinder(radius=g.GetRadiusAttr().Get(), height=g.GetHeightAttr().Get(), sections=32)
        tm.apply_transform(axis_rot[g.GetAxisAttr().Get() or "Z"])
    elif prim.IsA(UsdGeom.Capsule):
        g = UsdGeom.Capsule(prim)
        tm = trimesh.creation.capsule(radius=g.GetRadiusAttr().Get(), height=g.GetHeightAttr().Get(), count=[16, 16])
        tm.apply_transform(axis_rot[g.GetAxisAttr().Get() or "Z"])
    elif prim.IsA(UsdGeom.Cone):
        g = UsdGeom.Cone(prim)
        tm = trimesh.creation.cone(radius=g.GetRadiusAttr().Get(), height=g.GetHeightAttr().Get(), sections=32)
        tm.apply_translation([0, 0, -g.GetHeightAttr().Get() / 2])
        tm.apply_transform(axis_rot[g.GetAxisAttr().Get() or "Z"])
    else:
        return None
    return np.asarray(tm.vertices), np.asarray(tm.faces), np.arange(len(tm.faces)), None


def is_collision(prim):
    p = prim
    while p and p.IsValid() and not p.IsPseudoRoot():
        if p.HasAPI(UsdPhysics.CollisionAPI):
            return True
        p = p.GetParent()
    return False


def face_colors(prim, src, uv, ntri):
    """Per-triangle RGBA (0-255) from material bindings (incl. GeomSubsets), textures and displayColor."""
    rgba = np.tile(np.array([180, 180, 180, 255], dtype=np.uint8), (ntri, 1))
    mats_used = []
    binding = UsdShade.MaterialBindingAPI(prim)
    assign = [(None, binding.ComputeBoundMaterial()[0])]
    for subset in UsdShade.MaterialBindingAPI(prim).GetMaterialBindSubsets():
        faces = np.asarray(subset.GetIndicesAttr().Get() or [], dtype=int)
        assign.append((faces, UsdShade.MaterialBindingAPI(subset.GetPrim()).ComputeBoundMaterial()[0]))
    disp = UsdGeom.PrimvarsAPI(prim).GetPrimvar("displayColor")
    disp_val = np.asarray(disp.Get(), dtype=float).reshape(-1, 3) if disp and disp.HasValue() else None
    if disp_val is not None and len(disp_val) >= 1 and assign[0][1] is None:
        rgba[:, :3] = np.clip(disp_val[0] * 255, 0, 255).astype(np.uint8)
    for faces, mat in assign:
        info = material_info(mat)
        if mat:
            mats_used.append(info)
        mask = np.ones(ntri, bool) if faces is None else np.isin(src, faces)
        if not mask.any() or not mat:
            continue
        if info["color"] is not None:
            rgba[mask, :3] = np.clip(np.asarray(info["color"]) * 255, 0, 255).astype(np.uint8)
        rgba[mask, 3] = int(np.clip(info["opacity"], 0.05, 1.0) * 255)
        tex = load_texture(info["texture"]) if info["texture_exists"] else None
        if tex is not None and uv is not None:
            c = uv[mask].mean(axis=1)
            h, w = tex.shape[:2]
            px = np.clip((c[:, 0] % 1.0) * (w - 1), 0, w - 1).astype(int)
            py = np.clip((1 - (c[:, 1] % 1.0)) * (h - 1), 0, h - 1).astype(int)
            rgba[mask, :3] = tex[py, px]
    return rgba, mats_used


def extract(entry):
    stage = Usd.Stage.Open(entry)
    mpu = UsdGeom.GetStageMetersPerUnit(stage) or 1.0
    up = UsdGeom.GetStageUpAxis(stage)
    to_z = np.eye(4) if up == "Z" else trimesh.transformations.rotation_matrix(np.pi / 2, [1, 0, 0])
    _, assets, unresolved = UsdUtils.ComputeAllDependencies(Sdf.AssetPath(entry))
    cache = UsdGeom.XformCache()
    vis, col, parts = [], [], []
    mass_total, rigid_bodies, mass_attrs = 0.0, 0, []
    for prim in Usd.PrimRange.Stage(stage, Usd.TraverseInstanceProxies()):
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            rigid_bodies += 1
        if prim.HasAPI(UsdPhysics.MassAPI):
            m = UsdPhysics.MassAPI(prim).GetMassAttr().Get()
            if m:
                mass_total += m
                mass_attrs.append((str(prim.GetPath()), float(m)))
        if not prim.IsA(UsdGeom.Gprim):
            continue
        res = gprim_to_mesh(prim)
        if res is None:
            continue
        pts, tris, src, uv = res
        img = UsdGeom.Imageable(prim)
        purpose = img.ComputePurpose()
        visible = img.ComputeVisibility() != UsdGeom.Tokens.invisible
        coll = is_collision(prim)
        approx = None
        if coll and prim.HasAPI(UsdPhysics.MeshCollisionAPI):
            approx = UsdPhysics.MeshCollisionAPI(prim).GetApproximationAttr().Get()
        elif coll and prim.IsA(UsdGeom.Mesh):
            approx = "none(triangle)"
        elif coll:
            approx = "primitive:" + prim.GetTypeName()
        M = np.asarray(cache.GetLocalToWorldTransform(prim), dtype=float)
        wp = np.c_[pts, np.ones(len(pts))] @ M
        wp = (wp[:, :3] * mpu) @ to_z[:3, :3].T
        is_visual = visible and purpose in (UsdGeom.Tokens.default_, UsdGeom.Tokens.render)
        part = {
            "path": str(prim.GetPath()),
            "type": prim.GetTypeName(),
            "purpose": str(purpose),
            "visible": visible,
            "collision": coll,
            "approximation": approx,
            "points": int(len(pts)),
            "triangles": int(len(tris)),
            "visual": bool(is_visual),
        }
        if is_visual:
            rgba, mats = face_colors(prim, src, uv, len(tris))
            part["materials"] = mats
            vis.append(trimesh.Trimesh(wp, tris, face_colors=rgba, process=False))
        if coll:
            col.append(trimesh.Trimesh(wp, tris, process=False))
        parts.append(part)
    return (
        {
            "meters_per_unit": mpu,
            "up_axis": up,
            "default_prim": str(stage.GetDefaultPrim().GetPath()) if stage.GetDefaultPrim() else None,
            "dependencies": sorted({a for a in assets}),
            "unresolved": sorted(set(unresolved)),
            "rigid_bodies": rigid_bodies,
            "mass_kg": mass_total,
            "mass_attrs": mass_attrs,
            "parts": parts,
        },
        vis,
        col,
    )


def main():
    src_dir, obj_json, out_dir = sys.argv[1:4]
    objs = json.load(open(obj_json, encoding="utf-8"))
    os.makedirs(os.path.join(out_dir, "meshes"), exist_ok=True)
    stats = {}
    for o in objs:
        tag = f"{o['scene']}__{o['id']}"
        entry = os.path.join(src_dir, o["entry"])
        try:
            info, vis, col = extract(entry)
            if vis:
                vm = trimesh.util.concatenate(vis)
                vm.export(os.path.join(out_dir, "meshes", tag + "_vis.ply"))
                info["vis_bbox_min"] = vm.bounds[0].tolist()
                info["vis_bbox_max"] = vm.bounds[1].tolist()
                info["vis_extents"] = vm.extents.tolist()
                info["vis_triangles"] = int(len(vm.faces))
                info["vis_vertices"] = int(len(vm.vertices))
            if col:
                cm = trimesh.util.concatenate(col)
                cm.export(os.path.join(out_dir, "meshes", tag + "_col.ply"))
                info["col_extents"] = cm.extents.tolist()
                info["col_parts"] = len(col)
            info["ok"] = True
            print(
                f"OK   {tag}: vis={len(vis)} parts/{info.get('vis_triangles', 0)} tris, col={len(col)}, "
                f"ext={np.round(info.get('vis_extents', [0, 0, 0]), 3).tolist()}, unresolved={info['unresolved']}"
            )
        except Exception as e:
            info = {"ok": False, "error": f"{type(e).__name__}: {e}", "trace": traceback.format_exc()}
            print(f"FAIL {tag}: {info['error']}")
        stats[o["key"]] = {**o, **info}
    with open(os.path.join(out_dir, "stats.json"), "w", encoding="utf-8") as f:
        json.dump(stats, f, ensure_ascii=False, indent=1)
    print("done", sum(v.get("ok", False) for v in stats.values()), "/", len(stats))


if __name__ == "__main__":
    try:
        main()
    except Exception:
        traceback.print_exc()
        if APP is not None:
            os._exit(1)
        raise
    if APP is not None:
        APP.close()
