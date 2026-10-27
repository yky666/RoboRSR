"""Build RoboDojo rigid assets (base + generalization variants) from the extracted local object parts.

Usage (5090-2, RoboDojo env, OMNI_KIT_ACCEPT_EULA=YES):
    python build_assets.py PARTS_DIR keep_assets.json objects.json OUT_TYPE_DIR REAL2SIM_OBJECTS_DIR \
        [--only cat_00000 ...]

Writes OUT_TYPE_DIR/<category>/<idx>/{object.usdz, metadata.json, description.json} for every base asset and its
VARIANTS generalization variants (next free indices of the category, metadata "variant_of"), and
REAL2SIM_OBJECTS_DIR/Rigid/<category>/<idx>/spec.json for each base asset.

Geometry: Z up, metres, origin at the bbox centre; handles on +X; long lying objects along X; bottles upright.
USD layout follows scripts/objects/make_plastic_cups.py: /root (RigidBodyAPI), /root/visual/model (first mesh),
/root/visual/part_NN, /root/_materials/*, /root/collision/hull_NN (guide, invisible, convexHull, <= 64 vertices).
No physics checks are run here.
"""

import argparse
import json
import os
from pathlib import Path
import shutil
import sys
import tempfile
import time
import traceback
import uuid

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import make_pitcher as mp  # noqa: E402  (starts Kit when pxr is not importable)
import numpy as np  # noqa: E402
import open3d as o3d  # noqa: E402
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade, Vt  # noqa: E402
from scipy.cluster.vq import kmeans2  # noqa: E402
from scipy.spatial import ConvexHull  # noqa: E402
import trimesh  # noqa: E402

VARIANTS = 5
MAX_HULL_VERTS = 64
MAX_HULLS = 48
GRASP_MAX_NARROW = 0.08  # Dex1-1 opening: graspable objects keep their narrow side <= 80 mm
MASS_CAP = 0.5  # env/scene_manager/objects/rigid.py clamps rigid mass to <= 0.5 kg
CREASE_DEG = 40.0

# material classes: (mdl, density factor relative to plastic, english words, chinese)
MATERIALS = {
    "plastic_matte": dict(
        kind="pbr", rough=0.55, metal=0.0, dens=1.0, en=["matte plastic", "PP plastic", "plastic"], zh="哑光塑料"
    ),
    "plastic_glossy": dict(
        kind="pbr", rough=0.18, metal=0.0, dens=1.0, en=["glossy plastic", "ABS plastic", "plastic"], zh="亮面塑料"
    ),
    "ceramic": dict(
        kind="pbr", rough=0.07, metal=0.0, dens=2.3, en=["glazed ceramic", "porcelain", "ceramic"], zh="陶瓷"
    ),
    "metal_brushed": dict(
        kind="pbr", rough=0.32, metal=1.0, dens=3.0, en=["brushed metal", "stainless steel", "metal"], zh="拉丝金属"
    ),
    "metal_glossy": dict(
        kind="pbr", rough=0.12, metal=1.0, dens=3.0, en=["polished metal", "aluminium", "metal"], zh="亮面金属"
    ),
    "glass": dict(
        kind="glass", rough=0.0, metal=0.0, dens=2.5, en=["clear glass", "glass", "transparent glass"], zh="玻璃"
    ),
    "clear_plastic": dict(
        kind="clear", milky=0.1, rough=0.15, dens=1.0, en=["clear plastic", "transparent PET", "plastic"], zh="透明塑料"
    ),
    "frosted_plastic": dict(
        kind="clear",
        milky=0.45,
        rough=0.35,
        dens=1.0,
        en=["frosted plastic", "translucent PP", "plastic"],
        zh="磨砂半透明塑料",
    ),
    "rubber": dict(kind="pbr", rough=0.9, metal=0.0, dens=1.2, en=["rubber", "soft rubber", "silicone"], zh="橡胶"),
    "cardboard": dict(
        kind="pbr", rough=0.85, metal=0.0, dens=0.7, en=["cardboard", "paperboard", "printed card"], zh="纸板"
    ),
    "paper": dict(kind="pbr", rough=0.9, metal=0.0, dens=0.7, en=["paper", "crumpled paper", "printer paper"], zh="纸"),
    "paper_glossy": dict(
        kind="pbr", rough=0.4, metal=0.0, dens=0.8, en=["glossy paper", "coated paper", "paper"], zh="铜版纸"
    ),
    "wicker": dict(kind="pbr", rough=0.75, metal=0.0, dens=0.6, en=["woven wicker", "rattan", "wicker"], zh="藤编"),
    "wood": dict(kind="pbr", rough=0.6, metal=0.0, dens=0.6, en=["wood", "light wood", "wooden"], zh="木材"),
}
PALETTE = {  # name: (rgb, chinese)
    "red": ((0.74, 0.10, 0.09), "红色"),
    "orange": ((0.90, 0.42, 0.08), "橙色"),
    "yellow": ((0.93, 0.78, 0.15), "黄色"),
    "green": ((0.18, 0.55, 0.22), "绿色"),
    "teal": ((0.07, 0.52, 0.55), "青色"),
    "blue": ((0.12, 0.33, 0.78), "蓝色"),
    "navy": ((0.08, 0.12, 0.35), "藏青色"),
    "purple": ((0.45, 0.20, 0.62), "紫色"),
    "pink": ((0.92, 0.48, 0.62), "粉色"),
    "white": ((0.92, 0.92, 0.90), "白色"),
    "black": ((0.05, 0.05, 0.06), "黑色"),
    "grey": ((0.45, 0.46, 0.48), "灰色"),
    "brown": ((0.40, 0.24, 0.12), "棕色"),
    "beige": ((0.85, 0.76, 0.60), "米色"),
}
SIZES = [(0.80, 1.00), (1.15, 0.85), (0.90, 1.25), (1.20, 1.15), (0.85, 0.80)]  # (cross-section, long axis)
FAMILY_SHAPES = {
    "round": [("taper", 0.28), ("taper", -0.22), ("bulge", 0.13), ("bulge", -0.11), ("squarish", 1.0)],
    "box": [("taper", 0.18), ("taper", -0.15), ("pillow", 0.14), ("bulge", 0.09), ("shear", 0.12)],
    "flat_round": [("oval", 0.14), ("squarish", 1.0), ("taper", 0.25), ("taper", -0.2), ("bulge", 0.12)],
    "flat_box": [("taper", 0.16), ("pillow", 0.2), ("oval", 0.1), ("bulge", 0.1), ("shear", 0.1)],
    "long": [("taper", 0.3), ("taper", -0.25), ("bulge", 0.18), ("bulge", -0.12), ("squarish", 1.0)],
    "crumple": [("noise", 0.12), ("bulge", 0.15), ("oval", 0.18), ("noise", 0.2), ("taper", 0.25)],
}
SHAPE_EN = {
    ("taper", 1): "wider at the top",
    ("taper", -1): "wider at the bottom",
    ("bulge", 1): "bulging belly",
    ("bulge", -1): "narrow waist",
    ("squarish", 1): "rounded-square cross-section",
    ("pillow", 1): "domed top",
    ("shear", 1): "slanted sides",
    ("oval", 1): "oval footprint",
    ("noise", 1): "lumpier folds",
}
FAMILY = {
    "measuring_jug": "round",
    "wide_cup": "round",
    "game_controller": "long",
    "tapered_cup": "round",
    "water_pitcher": "round",
    "marker_box": "flat_box",
    "car_key": "flat_box",
    "paper_wad": "crumple",
    "storage_basket": "box",
    "drink_bottle": "round",
    "glass_bottle": "round",
    "empty_bottle": "round",
    "water_bottle": "round",
    "bottle_cap": "flat_round",
    "round_lid": "flat_round",
    "tumbler_lid": "flat_round",
    "storage_box_lid": "flat_box",
    "marker_pen": "long",
    "soda_can": "round",
    "tumbler": "round",
    "straw": "round",
    "storage_box": "box",
    "ring_disc": "flat_round",
    "tissue_box": "box",
}
LARGE = {"storage_basket", "storage_box", "storage_box_lid", "tissue_box"}  # not grasped across the narrow side


# ----------------------------------------------------------------------------------------------- geometry helpers
def load_parts(d):
    z = np.load(d / "parts.npz")
    info = json.loads((d / "materials.json").read_text())
    meshes, i = [], 0
    while f"v{i}" in z:
        meshes.append(
            {
                "v": z[f"v{i}"].astype(float),
                "f": z[f"f{i}"].astype(int),
                "mat": z[f"mat{i}"].astype(int),
                "uv": z[f"uv{i}"].astype(float) if f"uv{i}" in z else None,
            }
        )
        i += 1
    hulls, j = [], 0
    while f"h{j}" in z:
        hulls.append(z[f"h{j}"].astype(float))
        j += 1
    return meshes, hulls, info["materials"]


def split_by_material(meshes):
    """One mesh per (part, material); drops degenerate and duplicate triangles."""
    out = []
    for m in meshes:
        v, f = m["v"], m["f"]
        area = 0.5 * np.linalg.norm(np.cross(v[f[:, 1]] - v[f[:, 0]], v[f[:, 2]] - v[f[:, 0]]), axis=1)
        _, first = np.unique(np.sort(f, axis=1), axis=0, return_index=True)
        keep = np.zeros(len(f), bool)
        keep[first] = True
        keep &= area > 1e-12
        for mid in np.unique(m["mat"][keep]):
            sel = keep & (m["mat"] == mid)
            used, inv = np.unique(f[sel], return_inverse=True)
            out.append(
                {
                    "v": v[used],
                    "f": inv.reshape(-1, 3),
                    "mat": int(mid),
                    "uv": None if m["uv"] is None else m["uv"][sel],
                }
            )
    return out


def merge_same_material(parts):
    by = {}
    for p in parts:
        if p["uv"] is not None:
            by.setdefault(("uv", id(p)), []).append(p)
        else:
            by.setdefault(p["mat"], []).append(p)
    out = []
    for group in by.values():
        if len(group) == 1:
            out.append(group[0])
            continue
        vs, fs, off = [], [], 0
        for p in group:
            vs.append(p["v"])
            fs.append(p["f"] + off)
            off += len(p["v"])
        out.append({"v": np.vstack(vs), "f": np.vstack(fs), "mat": group[0]["mat"], "uv": None})
    return out


def weld(v, f, tol=1e-6):
    key = np.round(v / tol).astype(np.int64)
    _, idx, inv = np.unique(key, axis=0, return_index=True, return_inverse=True)
    return v[idx], inv.reshape(-1)[f]


def decimate(parts, target):
    total = sum(len(p["f"]) for p in parts)
    if total <= target:
        return parts
    out = []
    for p in parts:
        if p["uv"] is not None or len(p["f"]) < 200:
            out.append(p)
            continue
        v, f = weld(p["v"], p["f"])
        m = o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(v), o3d.utility.Vector3iVector(f))
        m = m.simplify_quadric_decimation(max(100, int(len(f) * target / total)))
        m.remove_degenerate_triangles()
        m.remove_duplicated_triangles()
        m.remove_unreferenced_vertices()
        out.append({"v": np.asarray(m.vertices), "f": np.asarray(m.triangles), "mat": p["mat"], "uv": None})
    return out


def corner_normals(v, f, crease=CREASE_DEG):
    """Face-varying normals, smoothed across edges whose dihedral angle is below the crease angle."""
    wv, wf = weld(v, f, 1e-5)
    fn = np.cross(wv[wf[:, 1]] - wv[wf[:, 0]], wv[wf[:, 2]] - wv[wf[:, 0]])
    area = np.linalg.norm(fn, axis=1)
    fn = fn / np.maximum(area, 1e-20)[:, None]
    tm = trimesh.Trimesh(wv, wf, process=False)
    vf = tm.vertex_faces
    corners_f = np.repeat(np.arange(len(wf)), 3)
    nb = vf[wf.reshape(-1)]
    valid = nb >= 0
    nbc = np.where(valid, nb, 0)
    dots = np.einsum("ckj,cj->ck", fn[nbc], fn[corners_f])
    w = valid * (dots > np.cos(np.radians(crease))) * area[nbc]
    n = np.einsum("ck,ckj->cj", w, fn[nbc])
    n /= np.maximum(np.linalg.norm(n, axis=1), 1e-20)[:, None]
    bad = ~np.isfinite(n).all(1) | (np.linalg.norm(n, axis=1) < 0.5)
    n[bad] = fn[corners_f[bad]]
    return n


def fps(points, k):
    idx = [int(np.argmax(np.linalg.norm(points - points.mean(0), axis=1)))]
    d = np.linalg.norm(points - points[idx[0]], axis=1)
    for _ in range(k - 1):
        i = int(np.argmax(d))
        idx.append(i)
        d = np.minimum(d, np.linalg.norm(points - points[i], axis=1))
    return points[idx]


def hull_points(p):
    """Vertices of the convex hull of p, reduced to <= MAX_HULL_VERTS."""
    p = np.unique(np.round(p, 7), axis=0)
    if len(p) < 4:
        return None
    try:
        h = ConvexHull(p)
    except Exception:
        return None
    v = p[h.vertices]
    if len(v) > MAX_HULL_VERTS:
        v = fps(v, MAX_HULL_VERTS)
        v = v[ConvexHull(v).vertices]
    return v


def hulls_mesh(hulls):
    ms = []
    for h in hulls:
        ch = ConvexHull(h)
        ms.append(trimesh.Trimesh(h, ch.simplices, process=False))
    return trimesh.util.concatenate(ms)


def opening_depth(mesh, lo, hi):
    """Median depth (fraction of height) of the first hit of downward rays around the top centre."""
    c = (lo + hi) / 2
    r = 0.12 * min(hi[0] - lo[0], hi[1] - lo[1])
    o = np.array([[c[0] + dx, c[1] + dy, hi[2] + 0.01] for dx, dy in [(0, 0), (r, 0), (-r, 0), (0, r), (0, -r)]])
    locs, idx, _ = mesh.ray.intersects_location(o, np.tile([0, 0, -1.0], (len(o), 1)), multiple_hits=False)
    if len(locs) == 0:
        return 1.0
    return float(np.median([(hi[2] - locs[idx == i][:, 2].max()) / (hi[2] - lo[2]) for i in set(idx.tolist())]))


def build_hulls(raw, lo, hi):
    hulls = [h for h in (hull_points(p) for p in raw) if h is not None]
    if len(hulls) <= MAX_HULLS:
        return hulls, None
    ref = opening_depth(hulls_mesh(hulls), lo, hi)
    for k in (MAX_HULLS, 64, 96):
        if k >= len(hulls):
            break
        cent = np.array([h.mean(0) for h in hulls])
        _, lab = kmeans2(cent, k, seed=0, minit="++")
        merged = [
            h
            for h in (
                hull_points(np.vstack([hulls[i] for i in np.where(lab == c)[0]])) for c in range(k) if (lab == c).any()
            )
            if h is not None
        ]
        d = opening_depth(hulls_mesh(merged), lo, hi)
        if d >= 0.8 * ref - 1e-3:
            return merged, {
                "from": len(hulls),
                "to": len(merged),
                "opening_before": round(ref, 3),
                "opening_after": round(d, 3),
            }
    return hulls, {"from": len(hulls), "to": len(hulls), "note": "merge would close the opening; kept"}


# ----------------------------------------------------------------------------------------------- canonical frame
def rot_z(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def rot_x(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1.0, 0, 0], [0, c, -s], [0, s, c]])


def canonical_rotation(V, cfg):
    R = np.eye(3)
    ext = V.max(0) - V.min(0)
    if cfg.get("upright"):
        ax = int(np.argmax(ext))
        if ax == 1:
            R = rot_x(np.pi / 2)
        elif ax == 0:
            R = np.array([[0, 0, -1.0], [0, 1, 0], [1, 0, 0]])
        W = V @ R.T
        lo, hi = W.min(0), W.max(0)
        c = (lo + hi) / 2
        h = hi[2] - lo[2]
        rad = np.linalg.norm(W[:, :2] - c[:2], axis=1)
        top, bot = rad[W[:, 2] > hi[2] - 0.15 * h].mean(), rad[W[:, 2] < lo[2] + 0.15 * h].mean()
        if top > bot:
            R = rot_x(np.pi) @ R
    elif not cfg.get("handle") and FAMILY[cfg["category"]] in ("long", "flat_box", "box") and ext[1] > 1.2 * ext[0]:
        R = rot_z(np.pi / 2)
    return R


def handle_frame(V):
    """Body axis x_c, body radius and grip point; rotates handle onto +X. Returns (R, x_c, grip)."""
    lo, hi = V.min(0), V.max(0)
    h = hi[2] - lo[2]
    band = V[(V[:, 2] > lo[2] + 0.3 * h) & (V[:, 2] < lo[2] + 0.7 * h)]
    xc = float(np.median(band[:, 0]))
    R = np.eye(3)
    if xc - lo[0] > hi[0] - xc:
        R = rot_z(np.pi)
    W = V @ R.T
    lo, hi = W.min(0), W.max(0)
    band = W[(W[:, 2] > lo[2] + 0.3 * h) & (W[:, 2] < lo[2] + 0.7 * h)]
    xc = float(np.median(band[:, 0]))
    rb = (hi[1] - lo[1]) / 2
    hp = W[W[:, 0] > xc + rb + 0.006]
    grip = hp.mean(0) if len(hp) else np.array([hi[0] - 0.01, 0, (lo[2] + hi[2]) / 2])
    return R, xc, rb, grip


# ----------------------------------------------------------------------------------------------- deformations
def deform(P, op, amount, axis, center, ext, seed=0):
    """Smooth deformation in the centred frame. axis 2 = vertical; axis 0 = lying long objects."""
    P = P.copy()
    other = [i for i in range(3) if i != axis]
    half = ext[axis] / 2
    t = np.clip((P[:, axis] + half) / max(2 * half, 1e-9), 0, 1)
    rel = P[:, other] - center[other]
    if op == "taper":
        P[:, other] = center[other] + rel * (1 + amount * (t - 0.5))[:, None]
    elif op == "bulge":
        P[:, other] = center[other] + rel * (1 + amount * np.sin(np.pi * t))[:, None]
    elif op == "squarish":
        th = np.arctan2(rel[:, 1], rel[:, 0])
        k = (np.abs(np.cos(th)) ** 4 + np.abs(np.sin(th)) ** 4) ** -0.25
        P[:, other] = center[other] + rel * (1 + amount * (k - 1))[:, None]
    elif op == "oval":
        P[:, other[0]] = center[other[0]] + rel[:, 0] * (1 + amount)
        P[:, other[1]] = center[other[1]] + rel[:, 1] * (1 - amount)
    elif op == "pillow":
        hx, hy = ext[other[0]] / 2, ext[other[1]] / 2
        u = np.clip(1 - (rel[:, 0] / max(hx, 1e-9)) ** 2, 0, 1) * np.clip(1 - (rel[:, 1] / max(hy, 1e-9)) ** 2, 0, 1)
        P[:, axis] += amount * ext[axis] * u * t
    elif op == "shear":
        P[:, other[0]] += amount * ext[axis] * (t - 0.5)
    elif op == "noise":
        rng = np.random.default_rng(seed)
        k = rng.normal(size=(6, 3)) * 2.5 / max(ext.max(), 1e-6)
        ph = rng.uniform(0, 2 * np.pi, 6)
        disp = sum(np.sin(P @ k[i] + ph[i]) for i in range(6)) / 6
        r = P - center
        P = center + r * (1 + amount * disp)[:, None]
    return P


# ----------------------------------------------------------------------------------------------- USD writing
def classify_source(mt):
    if mt.get("texture"):
        return "textured"
    if mt["opacity"] < 0.6:
        return (
            "glass"
            if (mt.get("roughness") or 0.2) < 0.2 or any("Glass" in s for s in mt["shaders"])
            else "clear_plastic"
        )
    if mt["opacity"] < 0.95:
        return "frosted_plastic"
    if (mt.get("metallic") or 0) > 0.5:
        return "metal_brushed"
    return "plastic_glossy" if (mt.get("roughness") or 0.5) < 0.3 else "plastic_matte"


def add_material(st, path, cls, rgb, tex=None, tint=None, src=None):
    """MDL (OmniPBR / OmniSurface / OmniGlass) + UsdPreviewSurface fallback."""
    mat = UsdShade.Material.Define(st, path)
    sh = UsdShade.Shader.Define(st, f"{path}/Shader")
    pv = UsdShade.Shader.Define(st, f"{path}/Preview")
    pv.CreateIdAttr("UsdPreviewSurface")
    F, C, B, A = Sdf.ValueTypeNames.Float, Sdf.ValueTypeNames.Color3f, Sdf.ValueTypeNames.Bool, Sdf.ValueTypeNames.Asset
    rgb = tuple(float(x) for x in rgb)
    if tex is not None or cls == "textured":
        spec = MATERIALS.get(src or "plastic_matte", MATERIALS["plastic_matte"])
        mdl, rough, metal = "OmniPBR", spec.get("rough", 0.5), spec.get("metal", 0.0)
    else:
        spec = MATERIALS[cls]
        mdl = {"pbr": "OmniPBR", "glass": "OmniGlass", "clear": "OmniSurface"}[spec["kind"]]
        rough, metal = spec.get("rough", 0.5), spec.get("metal", 0.0)
    sh.SetSourceAsset(Sdf.AssetPath(f"{mdl}.mdl"), "mdl")
    sh.SetSourceAssetSubIdentifier(mdl, "mdl")
    opacity = 1.0
    if mdl == "OmniPBR":
        sh.CreateInput("diffuse_color_constant", C).Set(rgb)
        sh.CreateInput("reflection_roughness_constant", F).Set(float(rough))
        sh.CreateInput("metallic_constant", F).Set(float(metal))
        if tex is not None:
            sh.CreateInput("diffuse_texture", A).Set(Sdf.AssetPath(f"./textures/{tex}"))
            if tint is not None:
                sh.CreateInput("diffuse_tint", C).Set(tuple(float(x) for x in tint))
    elif mdl == "OmniGlass":
        sh.CreateInput("glass_color", C).Set(tuple(0.25 + 0.75 * x for x in rgb))
        sh.CreateInput("glass_ior", F).Set(1.5)
        sh.CreateInput("thin_walled", B).Set(False)
        opacity = 0.25
    else:  # translucent plastic, see make_plastic_cups.clear_pp
        sh.CreateInput("diffuse_reflection_weight", F).Set(1.0)
        sh.CreateInput("diffuse_reflection_color", C).Set(rgb)
        sh.CreateInput("enable_specular_transmission", B).Set(True)
        sh.CreateInput("specular_transmission_weight", F).Set(float(1.0 - spec["milky"]))
        sh.CreateInput("specular_reflection_weight", F).Set(0.1)
        sh.CreateInput("specular_reflection_roughness", F).Set(float(spec["rough"]))
        sh.CreateInput("specular_reflection_ior", F).Set(1.49)
        sh.CreateInput("thin_walled", B).Set(False)
        opacity = 0.35 + 0.5 * spec["milky"]
    mat.CreateSurfaceOutput("mdl").ConnectToSource(sh.CreateOutput("out", Sdf.ValueTypeNames.Token))
    pv.CreateInput("roughness", F).Set(float(rough))
    pv.CreateInput("metallic", F).Set(float(metal))
    pv.CreateInput("opacity", F).Set(float(opacity))
    pv.CreateInput("ior", F).Set(1.5)
    if tex is not None:
        st_reader = UsdShade.Shader.Define(st, f"{path}/st")
        st_reader.CreateIdAttr("UsdPrimvarReader_float2")
        st_reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
        tx = UsdShade.Shader.Define(st, f"{path}/Texture")
        tx.CreateIdAttr("UsdUVTexture")
        tx.CreateInput("file", A).Set(Sdf.AssetPath(f"./textures/{tex}"))
        tx.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(
            st_reader.CreateOutput("result", Sdf.ValueTypeNames.Float2)
        )
        if tint is not None:
            tx.CreateInput("scale", Sdf.ValueTypeNames.Float4).Set(Gf.Vec4f(*[float(x) for x in tint], 1.0))
        pv.CreateInput("diffuseColor", C).ConnectToSource(tx.CreateOutput("rgb", Sdf.ValueTypeNames.Float3))
    else:
        pv.CreateInput("diffuseColor", C).Set(rgb)
    mat.CreateSurfaceOutput().ConnectToSource(pv.CreateOutput("surface", Sdf.ValueTypeNames.Token))
    return mat


def write_usdz(out, parts, mat_specs, hulls, tex_dir):
    tmpd = Path(tempfile.mkdtemp())
    tmp = tmpd / "object.usdc"
    st = Usd.Stage.CreateNew(str(tmp))
    UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(st, 1.0)
    root = mp.add_xform(st, "/root")
    st.SetDefaultPrim(root.GetPrim())
    UsdPhysics.RigidBodyAPI.Apply(root.GetPrim())
    mats = {}
    for mid, ms in mat_specs.items():
        mats[mid] = add_material(
            st, f"/root/_materials/mat_{mid:02d}", ms["cls"], ms["rgb"], ms.get("tex"), ms.get("tint"), ms.get("src")
        )
    mp.add_xform(st, "/root/visual")
    for i, p in enumerate(parts):
        path = "/root/visual/model" if i == 0 else f"/root/visual/part_{i:02d}"
        m = UsdGeom.Mesh.Define(st, path)
        v = p["v"].astype(np.float32)
        m.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(v))
        m.CreateFaceVertexCountsAttr(Vt.IntArray([3] * len(p["f"])))
        m.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(p["f"].astype(np.int32).reshape(-1)))
        m.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
        m.CreateExtentAttr(Vt.Vec3fArray([Gf.Vec3f(*v.min(0).tolist()), Gf.Vec3f(*v.max(0).tolist())]))
        m.CreateNormalsAttr(Vt.Vec3fArray.FromNumpy(p["n"].astype(np.float32)))
        m.SetNormalsInterpolation(UsdGeom.Tokens.faceVarying)
        if p["uv"] is not None:
            pvar = UsdGeom.PrimvarsAPI(m).CreatePrimvar(
                "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.faceVarying
            )
            pvar.Set(Vt.Vec2fArray.FromNumpy(p["uv"].reshape(-1, 2).astype(np.float32)))
        UsdShade.MaterialBindingAPI.Apply(m.GetPrim()).Bind(mats[p["mat"]])
    mp.add_xform(st, "/root/collision")
    for i, h in enumerate(hulls):
        hull = ConvexHull(h)
        tri = hull.simplices.copy()
        n = np.cross(h[tri[:, 1]] - h[tri[:, 0]], h[tri[:, 2]] - h[tri[:, 0]])
        flip = np.einsum("ij,ij->i", n, hull.equations[:, :3]) < 0
        tri[flip] = tri[flip][:, ::-1]
        used, tri = np.unique(tri, return_inverse=True)
        cm = mp.add_mesh(st, f"/root/collision/hull_{i:02d}", h[used], tri.reshape(-1, 3).tolist())
        prim = cm.GetPrim()
        cm.CreatePurposeAttr(UsdGeom.Tokens.guide)
        vis = UsdGeom.VisibilityAPI.Apply(prim)
        vis.CreateRenderVisibilityAttr(UsdGeom.Tokens.invisible)
        vis.CreateGuideVisibilityAttr(UsdGeom.Tokens.invisible)
        UsdPhysics.CollisionAPI.Apply(prim)
        UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr(UsdPhysics.Tokens.convexHull)
    st.Save()
    zip_writer = getattr(Sdf, "ZipFileWriter", None) or Usd.ZipFileWriter
    zw = zip_writer.CreateNew(str(out))
    zw.AddFile(str(tmp), "object.usdc")
    for ms in mat_specs.values():
        if ms.get("tex"):
            zw.AddFile(str(tex_dir / ms["tex"]), f"textures/{ms['tex']}")
    zw.Save()
    shutil.rmtree(tmpd, ignore_errors=True)


# ----------------------------------------------------------------------------------------------- metadata / text
def r4(v):
    return [round(float(x), 4) for x in v]


NAME_ONLY = {  # naming only, never sampled for variants
    "light blue": ((0.55, 0.74, 0.80), "浅蓝色"),
    "dark green": ((0.06, 0.25, 0.10), "深绿色"),
    "clear": (None, "透明"),
    "translucent": (None, "半透明"),
}


def nearest_color(rgb):
    rgb = np.asarray(rgb)
    return min(PALETTE, key=lambda k: np.linalg.norm(np.asarray(PALETTE[k][0]) - rgb))


def base_color_name(rgb, base_material):
    """Colour word for the base asset; low-saturation transparent bodies are 'clear'."""
    rgb = np.asarray(rgb)
    spec = MATERIALS[base_material]
    if spec["kind"] in ("glass", "clear") and rgb.max() - rgb.min() < 0.2 and rgb.mean() > 0.4:
        return "translucent" if spec.get("milky", 0) > 0.3 else "clear"
    names = dict(PALETTE, **{k: NAME_ONLY[k] for k in ("light blue", "dark green")})
    return min(names, key=lambda k: np.linalg.norm(np.asarray(names[k][0]) - rgb))


def color_zh(name):
    return (PALETTE.get(name) or NAME_ONLY[name])[1]


def metadata(uid, V, n_faces, mass, grip, source, variant_of=None, params=None):
    lo, hi = V.min(0), V.max(0)
    ext = hi - lo
    corners = [r4((x, y, z)) for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])]
    bbox = {"vertices": corners, "extents": r4(ext)}
    foot = V[V[:, 2] < lo[2] + 0.003]
    contact_r = (
        float(np.percentile(np.linalg.norm(foot[:, :2], axis=1), 90)) if len(foot) > 3 else float(min(ext[:2]) / 2)
    )
    contact_r = min(contact_r, float(min(ext[:2]) / 2))
    functional = {}
    if grip is not None:  # same frames as mug/00015 and make_plastic_cups: vertical handle bar on +X
        functional["handle"] = {"frame": [r4((*grip, 0.5, 0.5, 0.5, 0.5)), r4((*grip, 0.5, -0.5, 0.5, -0.5))]}
    meta = {
        "uuid": uid,
        "physics": {"mass": round(float(mass), 4), "size": r4(ext)},
        "geometry": {
            "faces": int(n_faces),
            "vertices": int(len(V)),
            "aligned_bbox": bbox,
            "oriented_bbox": bbox,
            "radius": round(float(np.linalg.norm(V, axis=1).max()), 4),
            "bbox": r4(ext),
            "center": r4((lo + hi) / 2),
        },
        "active": {
            "place": {
                "up": {
                    "orientation": [1.0, 0.0, 0.0, 0.0],
                    "projection_circle": {
                        "center": r4((0, 0, lo[2], 1, 0, 0, 0)),
                        "radius": round(float(max(ext[:2]) / 2), 4),
                    },
                    "contact_circle": {"center": r4((0, 0, lo[2], 1, 0, 0, 0)), "radius": round(contact_r, 4)},
                }
            },
            "functional": functional,
        },
        "passive": {"functional": {}, "support": {}},
        "source_asset": source,
    }
    if variant_of:
        meta["variant_of"] = variant_of
        meta["variant_params"] = params
    return meta


def size_phrases(ext):
    h, w = ext[2] * 100, max(ext[0], ext[1]) * 100
    fit = "fits in one hand" if max(ext) < 0.2 else ("needs two hands" if max(ext) > 0.3 else "hand-sized")
    return [f"about {h:.0f} cm tall", f"about {w:.0f} cm across", fit]


def dedup_words(s):
    out = []
    for w in s.split(" "):
        if w.strip(",").lower() not in {x.strip(",").lower() for x in out}:
            out.append(w)
    return " ".join(out)


def description(uid, cfg, color, mat_words, ext, shape_word=None):
    name = cfg["name_en"]
    shape = list(cfg["shape"])
    if shape_word:
        shape = [shape_word] + shape[:2]
    sz = size_phrases(ext)
    return {
        "uuid": uid,
        "caption": {
            "color": [color, f"mostly {color}", f"{color} body"],
            "material": mat_words,
            "size": sz,
            "name": name,
            "shape": shape,
            "function": cfg["function"],
        },
        "description": [
            dedup_words(f"{color} {mat_words[0]} {name[0]}"),
            f"{name[1]} with a {shape[0]}, {sz[0]}",
            f"{name[0]} made of {mat_words[1]}, {sz[1]}",
            f"{name[2]} with a {shape[1]} and a {shape[2]}",
            f"{name[0]} used for {cfg['function'][0]}",
        ],
    }


# ----------------------------------------------------------------------------------------------- main build
def prepare(cfg, parts_dir):
    d = parts_dir / f"{cfg['category']}_{cfg['idx']:05d}"
    meshes, raw_hulls, mats = load_parts(d)
    parts = split_by_material(meshes)
    for p in parts:
        if not mats[p["mat"]].get("texture"):
            p["uv"] = None
    if cfg.get("decimate"):
        parts = decimate(merge_same_material(parts), cfg["decimate"])
    V = np.vstack([p["v"] for p in parts])
    R = canonical_rotation(V, cfg)
    grip, xc = None, 0.0
    if cfg.get("handle"):
        R2, xc, rb, grip = handle_frame(V @ R.T)
        R = R2 @ R
    V = V @ R.T
    shift = -(V.min(0) + V.max(0)) / 2
    for p in parts:
        p["v"] = p["v"] @ R.T + shift
    hulls_raw = [h @ R.T + shift for h in raw_hulls]
    if grip is not None:
        grip = grip + shift
        xc = xc + shift[0]
    area = {}
    for p in parts:
        a = (
            0.5
            * np.linalg.norm(
                np.cross(p["v"][p["f"][:, 1]] - p["v"][p["f"][:, 0]], p["v"][p["f"][:, 2]] - p["v"][p["f"][:, 0]]),
                axis=1,
            ).sum()
        )
        area[p["mat"]] = area.get(p["mat"], 0) + a
    parts.sort(key=lambda p: (-area[p["mat"]], -len(p["f"])))
    primary = max(area, key=area.get)
    body = [i for i, mt in enumerate(mats) if mt["name"] in cfg.get("body", [])] or [primary]
    if primary not in body:
        primary = max(body, key=lambda i: area.get(i, 0))
    return {
        "parts": parts,
        "hulls_raw": hulls_raw,
        "mats": mats,
        "grip": grip,
        "xc": xc,
        "primary": primary,
        "body": body,
        "area": area,
        "tex_dir": d / "textures",
    }


def realize(cfg, prep, out_dir, idx, mass, source, variant=None, base_id=None):
    """Apply an optional variant (deform, scale, recolor, rematerial), recentre, write the asset dir."""
    family = FAMILY[cfg["category"]]
    parts = [dict(p) for p in prep["parts"]]
    hulls = [h.copy() for h in prep["hulls_raw"]]
    grip = None if prep["grip"] is None else prep["grip"].copy()
    V = np.vstack([p["v"] for p in parts])
    ext0 = V.max(0) - V.min(0)
    axis = 0 if family == "long" else 2
    center = np.array([prep["xc"], 0.0, 0.0])
    base_area = sum(prep["area"].values())
    if variant:
        op, amt = variant["shape"]
        for p in parts:
            p["v"] = deform(p["v"], op, amt, axis, center, ext0, seed=variant["seed"])
        hulls = [deform(h, op, amt, axis, center, ext0, seed=variant["seed"]) for h in hulls]
        if grip is not None:
            grip = deform(grip[None], op, amt, axis, center, ext0, seed=variant["seed"])[0]
        s_cross, s_long = variant["scale"]
        S = np.full(3, s_cross)
        S[axis] = s_long
        for p in parts:
            p["v"] = center + (p["v"] - center) * S
        hulls = [center + (h - center) * S for h in hulls]
        if grip is not None:
            grip = center + (grip - center) * S
    V = np.vstack([p["v"] for p in parts])
    shift = -(V.min(0) + V.max(0)) / 2
    for p in parts:
        p["v"] = p["v"] + shift
        p["n"] = corner_normals(p["v"], p["f"])
    hulls = [h + shift for h in hulls]
    if grip is not None:
        grip = grip + shift
    V = V + shift
    lo, hi = V.min(0), V.max(0)
    hulls, merge_info = build_hulls(hulls, lo, hi)
    if not hulls:
        hulls = [hull_points(V)]

    mat_specs = {}
    for mid, mt in enumerate(prep["mats"]):
        src_cls = classify_source(mt)
        ms = {
            "cls": src_cls if src_cls != "textured" else "textured",
            "rgb": mt["color"],
            "tex": mt.get("texture"),
            "src": "plastic_glossy",
        }
        in_body = mid in prep["body"]
        if not variant and in_body and not ms["tex"] and MATERIALS[cfg["base_material"]]["kind"] in ("glass", "clear"):
            ms["cls"] = cfg["base_material"]
        if variant:
            rgb = np.asarray(PALETTE[variant["color"]][0])
            if ms["tex"]:  # labels and prints follow the variant colour as a light tint
                ms["tint"] = tuple(0.4 + 0.6 * rgb)
                ms["src"] = variant["material"] if MATERIALS[variant["material"]]["kind"] == "pbr" else "plastic_glossy"
            elif in_body:
                ms["cls"] = variant["material"]
                ms["rgb"] = tuple(rgb if mid == prep["primary"] else rgb * 0.8)
        mat_specs[mid] = ms
    used = {p["mat"] for p in parts}
    mat_specs = {k: v for k, v in mat_specs.items() if k in used}

    area = sum(
        0.5
        * np.linalg.norm(
            np.cross(p["v"][p["f"][:, 1]] - p["v"][p["f"][:, 0]], p["v"][p["f"][:, 2]] - p["v"][p["f"][:, 0]]), axis=1
        ).sum()
        for p in parts
    )
    if variant:
        base_dens = MATERIALS[cfg["base_material"]]["dens"]
        mass = mass * area / base_area * MATERIALS[variant["material"]]["dens"] / base_dens
    mass = float(np.clip(mass, 0.001, MASS_CAP))

    d = out_dir / cfg["category"] / f"{idx:05d}"
    d.mkdir(parents=True, exist_ok=True)
    uid = str(uuid.uuid5(uuid.NAMESPACE_URL, f"robodojo/rigid/{cfg['category']}/{idx}"))
    write_usdz(d / "object.usdz", parts, mat_specs, hulls, prep["tex_dir"])
    n_faces = sum(len(p["f"]) for p in parts)
    params = None
    if variant:
        params = {
            "color": variant["color"],
            "material": variant["material"],
            "shape": list(variant["shape"]),
            "scale_cross_long": list(variant["scale"]),
            "seed": variant["seed"],
        }
    meta = metadata(uid, V, n_faces, mass, grip, source, base_id, params)
    if merge_info:
        meta["collision_merge"] = merge_info
    (d / "metadata.json").write_text(json.dumps(meta, indent=4))
    if variant:
        color = variant["color"]
        mwords = MATERIALS[variant["material"]]["en"]
        op, amt = variant["shape"]
        sw = SHAPE_EN[(op, 1 if amt > 0 or op in ("squarish", "oval", "pillow", "shear", "noise") else -1)]
    else:
        color = base_color_name(prep["mats"][prep["primary"]]["color"], cfg["base_material"])
        mwords = MATERIALS[cfg["base_material"]]["en"]
        sw = None
    (d / "description.json").write_text(json.dumps(description(uid, cfg, color, mwords, hi - lo, sw), indent=4))
    return d, meta, len(hulls), n_faces


def plan_variants(cfg, prep, base_ext):
    rng = np.random.default_rng(int(uuid.uuid5(uuid.NAMESPACE_URL, cfg["key"]).int % (2**32)))
    base_color = nearest_color(prep["mats"][prep["primary"]]["color"])
    colors = [c for c in PALETTE if c != base_color]
    rng.shuffle(colors)
    mats = list(cfg["materials"])
    rng.shuffle(mats)
    shapes = list(FAMILY_SHAPES[FAMILY[cfg["category"]]])
    rng.shuffle(shapes)
    sizes = list(SIZES)
    rng.shuffle(sizes)
    family = FAMILY[cfg["category"]]
    axis = 0 if family == "long" else 2
    cross = [base_ext[i] for i in range(3) if i != axis]
    narrow = min(cross)
    out = []
    for i in range(VARIANTS):
        mat = mats[i % len(mats)]
        color = colors[i]
        if MATERIALS[mat]["kind"] in ("glass", "clear") and color in ("black", "navy", "brown"):
            color = colors[(i + VARIANTS) % len(colors)]
        s_cross, s_long = sizes[i]
        if cfg["category"] not in LARGE and family not in ("flat_round", "flat_box", "long", "crumple"):
            limit = GRASP_MAX_NARROW / narrow if narrow <= GRASP_MAX_NARROW else 1.0
            s_cross = min(s_cross, limit)
        out.append(
            {
                "color": color,
                "material": mat,
                "shape": shapes[i],
                "scale": (round(s_cross, 3), s_long),
                "seed": int(rng.integers(0, 2**31)),
            }
        )
    return out


def spec_for(cfg, obj, meta, base_cls, color, now):
    ext = [round(x * 1000, 1) for x in meta["physics"]["size"]]
    zh_mat = MATERIALS.get(base_cls, MATERIALS["plastic_glossy"])["zh"]
    rep = cfg.get("replaces", [])
    return {
        "id": f"Rigid/{cfg['category']}/{cfg['idx']:05d}",
        "name": obj["name"],
        "name_en": cfg["name_en"][0],
        "material": f"{zh_mat}（按原模型材质判断）",
        "color": color_zh(color),
        "features": "、".join(cfg["shape"]),
        "requirements": "由本地已建模资产导入并优化；不做物理测试。",
        "notes": (
            f"来源：本地资产 {cfg['key']}（{obj['category']}）。去重时合并了 {len(rep)} 个重复/旧版本"
            + (f"：{'、'.join(rep)}" if rep else "")
            + "。尺寸、质量均为原模型的估计值，非实测；"
            f"原清单质量 {obj['mass_g']:.0f} g"
            + ("（超过 0.5 kg 运行时上限，已截断）" if obj["mass_g"] > 500 else "")
            + "。"
        ),
        "estimated_dimensions_mm": {"X": ext[0], "Y": ext[1], "Z": ext[2]},
        "estimated_mass_g": round(meta["physics"]["mass"] * 1000, 1),
        "measured": False,
        "generalization": {
            "count": VARIANTS,
            "requirements": "参数由 coding agent 设计，彼此差异明显",
            "color": True,
            "material": True,
            "size": True,
            "shape": True,
        },
        "updated_by": "coding agent（本地资产导入）",
        "updated_at": now,
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("parts_dir")
    ap.add_argument("keep_json")
    ap.add_argument("objects_json")
    ap.add_argument("out_type_dir")
    ap.add_argument("real2sim_dir")
    ap.add_argument("--only", nargs="*")
    a = ap.parse_args()
    keep = json.loads(Path(a.keep_json).read_text(encoding="utf-8"))
    objs = {o["key"]: o for o in json.loads(Path(a.objects_json).read_text(encoding="utf-8"))}
    out_dir, r2s = Path(a.out_type_dir), Path(a.real2sim_dir)
    now = time.strftime("%Y-%m-%d %H:%M:%S")
    # variant indices: after all base indices of the category
    bases_per_cat = {}
    for c in keep:
        bases_per_cat.setdefault(c["category"], []).append(c["idx"])
    next_idx = {cat: max(v) + 1 for cat, v in bases_per_cat.items()}
    report = []
    for cfg in keep:
        tag = f"{cfg['category']}_{cfg['idx']:05d}"
        var_idx = list(range(next_idx[cfg["category"]], next_idx[cfg["category"]] + VARIANTS))
        next_idx[cfg["category"]] += VARIANTS
        if a.only and tag not in a.only:
            continue
        try:
            obj = objs[cfg["key"]]
            prep = prepare(cfg, Path(a.parts_dir))
            mass = min(obj["mass_g"], 500) / 1000
            base_id = f"Rigid/{cfg['category']}/{cfg['idx']:05d}"
            d, meta, nh, nf = realize(cfg, prep, out_dir, cfg["idx"], mass, cfg["key"])
            ext = np.asarray(meta["physics"]["size"])
            base_cls = cfg["base_material"]
            color = base_color_name(prep["mats"][prep["primary"]]["color"], base_cls)
            sd = r2s / "Rigid" / cfg["category"] / f"{cfg['idx']:05d}"
            sd.mkdir(parents=True, exist_ok=True)
            spec = spec_for(cfg, obj, meta, base_cls, color, now)
            (sd / "spec.json").write_text(json.dumps(spec, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
            row = {
                "id": base_id,
                "source": cfg["key"],
                "faces": nf,
                "hulls": nh,
                "size_mm": [round(x * 1000, 1) for x in ext],
                "mass_g": round(meta["physics"]["mass"] * 1000, 1),
                "collision_merge": meta.get("collision_merge"),
                "variants": [],
            }
            print(f"BASE {base_id}: {nf} faces, {nh} hulls, size {row['size_mm']} mm, {row['mass_g']} g", flush=True)
            for vi, var in zip(var_idx, plan_variants(cfg, prep, ext)):
                vd, vmeta, vnh, vnf = realize(cfg, prep, out_dir, vi, mass, cfg["key"], var, base_id)
                vrow = {
                    "id": f"Rigid/{cfg['category']}/{vi:05d}",
                    "params": vmeta["variant_params"],
                    "hulls": vnh,
                    "size_mm": [round(x * 1000, 1) for x in vmeta["physics"]["size"]],
                    "mass_g": round(vmeta["physics"]["mass"] * 1000, 1),
                }
                row["variants"].append(vrow)
                print(
                    f"  VAR {vrow['id']}: {var['color']} {var['material']} {var['shape']} scale={var['scale']} "
                    f"size {vrow['size_mm']} mm {vrow['mass_g']} g, {vnh} hulls",
                    flush=True,
                )
            report.append(row)
        except Exception as e:
            print(f"FAIL {tag}: {type(e).__name__}: {e}", flush=True)
            traceback.print_exc()
            report.append({"id": tag, "error": f"{type(e).__name__}: {e}"})
    Path(out_dir.parent / "build_report.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=1), encoding="utf-8"
    )
    print("done", sum("error" not in r for r in report), "/", len(report))


if __name__ == "__main__":
    try:
        main()
    except Exception:
        traceback.print_exc()
        os._exit(1)
    if mp.APP is not None:
        mp.APP.close()
