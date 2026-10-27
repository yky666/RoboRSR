"""Per-part extraction (geometry, UVs, materials, collision hulls) for the kept objects.

Usage (5090-2, RoboDojo env, OMNI_KIT_ACCEPT_EULA=YES):
    python extract_parts.py SRC_DIR objects.json keep_assets.json OUT_DIR

Writes OUT_DIR/<category>_<idx:05d>/parts.npz and materials.json. Geometry in metres, Z up, source world frame.
"""

import json
import os
import shutil
import sys
import traceback

import extract_objects as eo  # starts Kit / provides pxr
from extract_objects import Usd, UsdGeom, UsdShade
import numpy as np
import trimesh

ROUGH_INPUTS = ("roughness", "reflection_roughness_constant", "specular_reflection_roughness")
METAL_INPUTS = ("metallic", "metallic_constant", "metalness")
IOR_INPUTS = ("ior", "glass_ior", "specular_reflection_ior")


def material_params(material, display_rgb=None):
    info = eo.material_info(material)
    p = {
        "name": info["name"] or "display",
        "color": info["color"],
        "opacity": info["opacity"],
        "texture": None,
        "roughness": None,
        "metallic": None,
        "ior": None,
        "shaders": info["shaders"],
    }
    if info["texture_exists"]:
        p["texture"] = info["texture"]
    if material:
        for prim in Usd.PrimRange(material.GetPrim()):
            if not prim.IsA(UsdShade.Shader):
                continue
            for inp in UsdShade.Shader(prim).GetInputs():
                v, n = inp.Get(), inp.GetBaseName()
                if not isinstance(v, (int, float)) or isinstance(v, bool):
                    continue
                if n in ROUGH_INPUTS and p["roughness"] is None:
                    p["roughness"] = float(v)
                elif n in METAL_INPUTS and p["metallic"] is None:
                    p["metallic"] = float(v)
                elif n in IOR_INPUTS and p["ior"] is None:
                    p["ior"] = float(v)
        if any("Glass" in s for s in info["shaders"]):
            p["opacity"] = min(p["opacity"], 0.3)
    if p["color"] is None:
        p["color"] = list(display_rgb) if display_rgb is not None else [0.7, 0.7, 0.7]
    return p


def extract(entry):
    stage = Usd.Stage.Open(entry)
    mpu = UsdGeom.GetStageMetersPerUnit(stage) or 1.0
    up = UsdGeom.GetStageUpAxis(stage)
    to_z = np.eye(3) if up == "Z" else trimesh.transformations.rotation_matrix(np.pi / 2, [1, 0, 0])[:3, :3]
    cache = UsdGeom.XformCache()
    mats, mat_index = [], {}
    meshes, hulls = [], []

    def mat_id(material, disp):
        key = str(material.GetPath()) if material else f"display:{None if disp is None else tuple(np.round(disp, 3))}"
        if key not in mat_index:
            mat_index[key] = len(mats)
            mats.append(material_params(material, disp))
        return mat_index[key]

    for prim in Usd.PrimRange.Stage(stage, Usd.TraverseInstanceProxies()):
        if not prim.IsA(UsdGeom.Gprim):
            continue
        res = eo.gprim_to_mesh(prim)
        if res is None:
            continue
        pts, tris, src, uv = res
        img = UsdGeom.Imageable(prim)
        visible = img.ComputeVisibility() != UsdGeom.Tokens.invisible
        is_visual = visible and img.ComputePurpose() in (UsdGeom.Tokens.default_, UsdGeom.Tokens.render)
        M = np.asarray(cache.GetLocalToWorldTransform(prim), dtype=float)
        wp = (np.c_[pts, np.ones(len(pts))] @ M)[:, :3] * mpu @ to_z.T
        if eo.is_collision(prim):
            hulls.append(wp)
        if not is_visual:
            continue
        disp = UsdGeom.PrimvarsAPI(prim).GetPrimvar("displayColor")
        disp = np.asarray(disp.Get(), dtype=float).reshape(-1, 3)[0] if disp and disp.HasValue() else None
        tri_mat = np.full(len(tris), mat_id(UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()[0], disp))
        for subset in UsdShade.MaterialBindingAPI(prim).GetMaterialBindSubsets():
            faces = np.asarray(subset.GetIndicesAttr().Get() or [], dtype=int)
            m = UsdShade.MaterialBindingAPI(subset.GetPrim()).ComputeBoundMaterial()[0]
            if m:
                tri_mat[np.isin(src, faces)] = mat_id(m, disp)
        meshes.append({"name": prim.GetName(), "v": wp, "f": tris, "uv": uv, "mat": tri_mat})
    return meshes, hulls, mats


def main():
    src_dir, obj_json, keep_json, out_dir = sys.argv[1:5]
    objs = {o["key"]: o for o in json.load(open(obj_json, encoding="utf-8"))}
    keep = json.load(open(keep_json, encoding="utf-8"))
    for k in keep:
        tag = f"{k['category']}_{k['idx']:05d}"
        d = os.path.join(out_dir, tag)
        os.makedirs(d, exist_ok=True)
        try:
            meshes, hulls, mats = extract(os.path.join(src_dir, objs[k["key"]]["entry"]))
            arrays = {}
            for i, m in enumerate(meshes):
                arrays[f"v{i}"], arrays[f"f{i}"], arrays[f"mat{i}"] = m["v"], m["f"], m["mat"]
                if m["uv"] is not None:
                    arrays[f"uv{i}"] = m["uv"]
            for j, h in enumerate(hulls):
                arrays[f"h{j}"] = h
            np.savez_compressed(os.path.join(d, "parts.npz"), **arrays)
            tex_dir = os.path.join(d, "textures")
            for mt in mats:
                if mt["texture"]:
                    os.makedirs(tex_dir, exist_ok=True)
                    dst = os.path.join(tex_dir, os.path.basename(mt["texture"]))
                    shutil.copyfile(mt["texture"], dst)
                    mt["texture"] = os.path.basename(mt["texture"])
            info = {"key": k["key"], "meshes": [m["name"] for m in meshes], "n_hulls": len(hulls), "materials": mats}
            json.dump(
                info, open(os.path.join(d, "materials.json"), "w", encoding="utf-8"), ensure_ascii=False, indent=1
            )
            print(
                f"OK   {tag} <- {k['key']}: {len(meshes)} meshes, {len(hulls)} hulls, {len(mats)} materials, "
                f"tex={[m['texture'] for m in mats if m['texture']]}",
                flush=True,
            )
        except Exception as e:
            print(f"FAIL {tag}: {type(e).__name__}: {e}", flush=True)
            traceback.print_exc()


if __name__ == "__main__":
    try:
        main()
    except Exception:
        traceback.print_exc()
        os._exit(1)
    if eo.APP is not None:
        eo.APP.close()
