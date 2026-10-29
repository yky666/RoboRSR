#!/usr/bin/env python3
"""Photo-fitted paper knife: Rigid/paper_knife/00000 and three colour-only variants.

Source: platform spec, 杨开元 2026-10-29; user supplemented measured thickness
17.14 mm in conversation 2026-10-30. Current input revision is checked at build.
Measured: length 152.88, maximum width 39.44, middle width 32.83, thickness 17.14 mm;
mass 58.9 g. Other contour/detail coordinates are photo-derived reconstruction
parameters, NOT additional physical measurements. Back face is not photographed.
Usage: python scripts/objects/make_paper_knife.py [RIGID_DIR]
Frame: metres, Z up, X nose-to-tail, origin all-visible-geometry bbox centre.
Rigid snapshot: blade and slider cannot move; no cutting or articulated-use claim.
USD: /root/visual/model first, separate metal/slider/tail_rim meshes and explicit
convex /root/collision/hull_NN. Tail and blade apertures remain open.
Dependencies: shared make_pitcher/make_plastic_cups helpers, numpy/scipy, Shapely 2.2,
Pillow/OpenCV for rectifying the photographed worn metal; pxr (USD or Isaac).
"""

import hashlib
import json
import os
from pathlib import Path
import sys
import tempfile
import traceback
import uuid

import cv2
import make_pitcher as mp
from make_pitcher import Sdf, Usd, UsdGeom, UsdPhysics, UsdShade
import make_plastic_cups as pc
import numpy as np
from PIL import Image
from scipy.spatial import ConvexHull
from shapely import constrained_delaunay_triangles
from shapely.geometry import Polygon, box

ROOT = Path(__file__).resolve().parents[2]
INPUT = ROOT / "real2sim/objects/Rigid/paper_knife/00000"
REVISION = "e249245a730f5980ca6d5b91285c3dad45f476e8a4ded6581e561aa9bbf0fdd4"
L, W, MID, T = 152.88, 39.44, 32.83, 17.14  # measured, mm, converted once at mesh boundary
MASS = 0.0589  # measured kg, variants retain this value as design assumption
PALETTES = [
    ("yellow", (0.66, 0.24, 0.003), "yellow", (0.66, 0.24, 0.003)),
    ("teal", (0.015, 0.48, 0.38), "ivory", (0.87, 0.83, 0.67)),
    ("blue", (0.018, 0.11, 0.55), "orange", (1.0, 0.22, 0.015)),
    ("violet", (0.43, 0.045, 0.63), "light gray", (0.65, 0.67, 0.71)),
]


def rounded(poly, r):
    return poly.buffer(-r, join_style=1).buffer(r, join_style=1, quad_segs=3)


def partitions(poly):
    """Merge constrained triangles into small convex pieces without sealing holes."""
    polys = list(constrained_delaunay_triangles(poly).geoms)
    changed = True
    while changed:
        changed = False
        for i in range(len(polys)):
            for j in range(i + 1, len(polys)):
                if polys[i].boundary.intersection(polys[j].boundary).length < 1e-6:
                    continue
                u = polys[i].union(polys[j])
                if u.geom_type == "Polygon" and len(u.exterior.coords) <= 30:
                    if abs(u.convex_hull.area - u.area) < 1e-7:
                        polys[i] = u
                        polys.pop(j)
                        changed = True
                        break
            if changed:
                break
    return polys


def prism(poly, z0, z1):
    """Watertight extrusion, with constrained caps and outward-oriented sidewalls."""
    from shapely.geometry.polygon import orient

    poly = orient(poly, sign=1)
    pts, fs, index = [], [], {}

    def vertex(x, y, z):
        key = (round(x, 8), round(y, 8), z)
        if key not in index:
            index[key] = len(pts)
            pts.append(key)
        return index[key]

    for tri in constrained_delaunay_triangles(poly).geoms:
        xy = np.array(tri.exterior.coords)[:3]
        if np.cross(xy[1] - xy[0], xy[2] - xy[0]) < 0:
            xy = xy[::-1]
        fs.append([vertex(x, y, z1) for x, y in xy])
        fs.append([vertex(x, y, z0) for x, y in xy[::-1]])
    for ring in [poly.exterior, *poly.interiors]:
        xy = list(ring.coords)
        for a, b in zip(xy, xy[1:]):
            fs.append([vertex(*a, z0), vertex(*b, z0), vertex(*b, z1), vertex(*a, z1)])
    p = np.array(pts, dtype=float) / 1000
    assert mp.signed_volume(p, fs) > 0
    return p, fs


def merge(meshes):
    pts, faces, n = [], [], 0
    for p, fs in meshes:
        pts.append(p)
        faces += [[k + n for k in f] for f in fs]
        n += len(p)
    return np.concatenate(pts), faces


def build():
    from shapely.geometry import Point

    parts, hulls = {}, []

    def add(name, polygon, lo, hi, collision=True):
        assert polygon.is_valid and polygon.area > 0, name
        mesh = prism(polygon, lo, hi)
        parts.setdefault(name, []).append(mesh)
        if collision:
            for piece in partitions(polygon):
                xy = np.array(piece.exterior.coords)[:-1]
                h = np.vstack([np.c_[xy, np.full(len(xy), z)] for z in (lo, hi)]) / 1000
                assert len(h) <= 64
                hulls.append(h)

    # Photo-traced shell: narrow nose, straight waist, flared pierced tail.
    shell = Polygon(
        [
            (22.6, 16.415),
            (46, 16.415),
            (48, 15.8),
            (137, 15.8),
            (144, 15.1),
            (146.1, 12.5),
            (145.2, 10.2),
            (143, 9.5),
            (143, -9.0),
            (L, -11.7),
            (149.7, -17.6),
            (146.5, -20.6),
            (140, -22),
            (135, -21.4),
            (126, -17.1),
            (118, -16.415),
            (48, -16.415),
            (37, -15.6),
            (30, -13),
            (29, -10.5),
            (22.6, -10.5),
        ]
    )
    # Maximum width is specifically the whole object's plan-view Y span: +17.44 to -22.
    upper = rounded(
        Polygon(
            [
                (20.5, 11),
                (20.5, 16.415),
                (43, 17.44),
                (47, 16.415),
                (135, 16.415),
                (141, 16.1),
                (145, 14.7),
                (146.3, 12),
                (145, 10.9),
            ]
        ),
        0.65,
    )
    # Keep exact measured extrema as planar segments after local rounding.
    upper = upper.union(box(39, 16.8, 43, 17.44))
    aperture = rounded(Polygon([(134, -14), (148.6, -13.4), (146, -18.2), (142, -19.3), (133.7, -18.7)]), 0.8)
    shell = shell.difference(aperture)
    add("model", shell, 0, 3.0)
    add("model", upper, 3, 14.4)
    lower = rounded(
        Polygon(
            [
                (29, -10.7),
                (38, -12.4),
                (116, -12.4),
                (128, -13.7),
                (139, -14),
                (139, -20.7),
                (133, -20.2),
                (123, -16.415),
                (50, -16.415),
                (38, -15.5),
                (31, -13.8),
            ]
        ),
        0.65,
    ).difference(aperture)
    add("model", lower, 3, 13.2)
    # The molded tail's highlighted border is a separate, fixed-colour mesh.
    rim_outer = aperture.buffer(1.0, quad_segs=3).intersection(shell.buffer(0.01))
    add("tail_rim", rim_outer.difference(aperture), 3, 8.7)
    # Steel tray extends to the measured nose (x=0), with folded lips, no sealed top.
    nose = Polygon([(0, -10.3), (4.2, 11.0), (138, 11.0), (140.2, -10.3)])
    add("steel_tray", nose, 3.2, 4.7)
    add("steel_nose", Polygon([(0, -10.3), (4.2, 11), (5.5, 11), (1.3, -10.3)]), 4.7, 10.9)
    for sign in (-1, 1):
        lip = box(4.2, 9.4, 140.2, 11.0) if sign > 0 else box(0, -11.0, 140.2, -9.4)
        add("steel_lips", lip, 4.7, 10.9)
        for x in np.arange(46, 112, 5.0):
            tooth = rounded(box(float(x), sign * 9.1 - 1.0, float(x + 2.2), sign * 9.1 + 1.0), 0.35)
            add("steel_teeth", tooth, 10.9, 12.5)
    # Visible segmented blade, recessed between the folded guide edges.
    blade_shape = Polygon([(5, -8.9), (13, 8.9), (121, 8.9), (113, -8.9)])
    blade_hole = Point(100, 0).buffer(2.6, quad_segs=8)
    add("blade", blade_shape.difference(blade_hole), 9.4, 10.2)
    add("label_blade_stamp", box(92.9, -1.8, 98.2, 1.8), 10.21, 10.31, collision=False)
    for x in np.arange(19, 110, 10.1):
        line = Polygon([(x, -8.85), (x + 0.17, -8.85), (x + 8.17, 8.85), (x + 8, 8.85)])
        add("blade_score", line, 10.2, 10.25, collision=False)
    # Trapezoidal thumb slider with diagonal raised grip ribs.
    slider = Polygon([(110.5, 10.2), (142, 10.2), (135, -10.5), (118, -10.5)])
    add("slider", slider, 10.3, T - 0.65)
    for x in np.arange(112, 144, 1.55):
        rib = Polygon([(x, 11), (x + 0.45, 11), (x + 9.45, -11), (x + 9, -11)]).intersection(slider)
        if rib.geom_type == "Polygon" and rib.area > 0.05:
            add("slider_ribs", rib, T - 0.65, T, collision=False)
    parts = {k: merge(v) for k, v in parts.items()}
    for name, (pts, faces) in parts.items():
        assert mp.signed_volume(pts, faces) > 0, name
    all_pts = np.concatenate([p for p, _ in parts.values()])
    shift = -(all_pts.min(0) + all_pts.max(0)) / 2
    parts = {k: (p + shift, f) for k, (p, f) in parts.items()}
    hulls = [h + shift for h in hulls]
    # Exact width across waist: exposed bottom face at X=76.44 mm, Y=±16.415.
    body_pts = parts["model"][0] - shift
    section = body_pts[(body_pts[:, 0] > 0.045) & (body_pts[:, 0] < 0.12) & (body_pts[:, 2] < 0.00301)]
    measurements = {
        "长度": float(np.ptp(all_pts[:, 0]) * 1000),
        "最大宽度": float(np.ptp(all_pts[:, 1]) * 1000),
        "中间宽度": float(np.ptp(section[:, 1]) * 1000),
        "厚度": float(np.ptp(all_pts[:, 2]) * 1000),
    }
    assert np.allclose(list(measurements.values()), [L, W, MID, T], atol=0.01), measurements
    # Probe vertical tail aperture against every collider, not just the visual mesh.
    probe_xy = np.array([141, -16.5]) / 1000 + shift[:2]
    for h in hulls:
        eq = ConvexHull(h).equations
        for z in np.linspace(-T / 2000 - 0.001, T / 2000 + 0.001, 20):
            assert np.max(eq[:, :3] @ np.r_[probe_xy, z] + eq[:, 3]) > 1e-7, "sealed tail aperture"
    return parts, hulls, shift, measurements


def metal_texture(output):
    src = INPUT / "reference/03fc5f3d5bd16f7edd80c9819179e94d.jpg"
    im = cv2.imread(str(src))
    # Four corners of a central metal panel, on the 1400px-wide review image,
    # scaled to the untouched 4032px reference. No logos or writing are invented.
    quad = np.float32([[247, 478], [733, 487], [713, 542], [218, 530]]) * (4032 / 1400)
    h = cv2.getPerspectiveTransform(quad, np.float32([[0, 0], [2047, 0], [2047, 255], [0, 255]]))
    panel = cv2.warpPerspective(im, h, (2048, 256)).astype(float)
    light = cv2.GaussianBlur(panel, (0, 0), 55)
    corrected = np.clip((panel / np.maximum(light, 20) * 150) * 0.65 + 52.5, 0, 255).astype(np.uint8)
    output.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(cv2.cvtColor(corrected, cv2.COLOR_BGR2RGB)).save(output)
    stamp_quad = np.float32([[806, 499], [850, 500], [848, 536], [804, 535]]) * (4032 / 1400)
    stamp_h = cv2.getPerspectiveTransform(stamp_quad, np.float32([[0, 0], [511, 0], [511, 383], [0, 383]]))
    stamp = cv2.warpPerspective(im, stamp_h, (512, 384)).astype(float)
    stamp_light = cv2.GaussianBlur(stamp, (0, 0), 80)
    stamp = np.clip(stamp / np.maximum(stamp_light, 20) * 150, 0, 255).astype(np.uint8)
    Image.fromarray(cv2.cvtColor(stamp, cv2.COLOR_BGR2RGB)).save(output.with_name("blade_stamp.png"))
    return {
        "photo": str(src.relative_to(ROOT)),
        "sha256": hashlib.sha256(src.read_bytes()).hexdigest(),
        "panel_quad_pixels": quad.tolist(),
        "texture_pixels": [2048, 256],
        "purpose": "photographed steel wear; separate faint pressed blade mark, no invented characters",
        "stamp": {
            "photo_quad_pixels": stamp_quad.tolist(),
            "mesh": "label_blade_stamp",
            "coverage_mm_before_bbox_shift": [92.9, 98.2, -1.8, 1.8],
            "texture_pixels": [512, 384],
            "substrate": "opaque metal engraving, not a paper sticker",
            "source_legibility": "faint partial mark; preserved as photographed, not transcribed",
        },
        "unseen_surface": "back: missing photograph, plain PP assumption; no invented printing",
    }


def add_look(st, name, rgb, metallic=0.0, roughness=0.4):
    return mp.add_material(
        st,
        "/root/_materials/" + name,
        "OmniPBR",
        {
            "diffuse_color_constant": rgb,
            "reflection_roughness_constant": roughness,
            "metallic_constant": metallic,
        },
        rgb,
        1.0,
        roughness,
    )


def write_usdz(path, parts, hulls, palette, texture):
    from pxr import Gf

    tmp = Path(tempfile.mkdtemp())
    st = Usd.Stage.CreateNew(str(tmp / "object.usdc"))
    UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(st, 1)
    root = mp.add_xform(st, "/root")
    st.SetDefaultPrim(root.GetPrim())
    UsdPhysics.RigidBodyAPI.Apply(root.GetPrim())
    UsdPhysics.MassAPI.Apply(root.GetPrim()).CreateMassAttr(MASS)
    mp.add_xform(st, "/root/visual")
    body = add_look(st, "PP", palette[1], roughness=0.43)
    rim = add_look(st, "MoldedTailRim", palette[3], roughness=0.43)
    steel = add_look(st, "Steel", (0.46, 0.48, 0.50), metallic=0.85, roughness=0.34)
    black = add_look(st, "ThumbSlider", (0.016, 0.017, 0.018), roughness=0.47)
    score = add_look(st, "ScoreGrooves", (0.13, 0.14, 0.15), metallic=0.65, roughness=0.5)

    # USD Preview Surface texture, portable and packaged into USDZ.
    def photo_look(name, filename):
        base = "/root/_materials/" + name
        material = UsdShade.Material.Define(st, base)
        shader = UsdShade.Shader.Define(st, base + "/Shader")
        shader.CreateIdAttr("UsdPreviewSurface")
        shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.75)
        shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.5)
        tex = UsdShade.Shader.Define(st, base + "/Texture")
        tex.CreateIdAttr("UsdUVTexture")
        tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath("textures/" + filename))
        tex.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set("sRGB")
        reader = UsdShade.Shader.Define(st, base + "/UV")
        reader.CreateIdAttr("UsdPrimvarReader_float2")
        reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
        tex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(reader.ConnectableAPI(), "result")
        tex.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(tex.ConnectableAPI(), "rgb")
        material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
        return material

    worn = photo_look("PhotographedSteel", "steel_photo.png")
    stamp = photo_look("PhotographedBladeStamp", "blade_stamp.png")
    allpts = np.concatenate([p for p, _ in parts.values()])
    for name, (pts, faces) in parts.items():
        mesh = mp.add_mesh(st, "/root/visual/" + name, pts, faces, mp.vertex_normals(pts, faces))
        mat = (
            body
            if name == "model"
            else rim
            if name == "tail_rim"
            else black
            if name.startswith("slider")
            else score
            if name == "blade_score"
            else worn
            if name == "blade"
            else stamp
            if name == "label_blade_stamp"
            else steel
        )
        UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(mat)
        if name in ("blade", "label_blade_stamp"):
            lo, hi = pts.min(0), pts.max(0)
            uv = (pts[:, :2] - lo[:2]) / (hi - lo)[:2]
            uv[:, 1] = 1 - uv[:, 1]
            UsdGeom.PrimvarsAPI(mesh).CreatePrimvar(
                "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex
            ).Set([Gf.Vec2f(float(u), float(v)) for u, v in uv])
    pc.add_collision(st, hulls)
    UsdGeom.ModelAPI(root).SetExtentsHint([Gf.Vec3f(*allpts.min(0)), Gf.Vec3f(*allpts.max(0))])
    st.GetRootLayer().Save()
    writer = (getattr(Sdf, "ZipFileWriter", None) or Usd.ZipFileWriter).CreateNew(str(path))
    writer.AddFile(str(tmp / "object.usdc"), "object.usdc")
    writer.AddFile(str(texture), "textures/steel_photo.png")
    writer.AddFile(str(texture.with_name("blade_stamp.png")), "textures/blade_stamp.png")
    writer.Save()


def main(rigid_dir):
    spec = json.loads((INPUT / "spec.json").read_text())
    assert spec["modeling_confirmation"]["revision"] == REVISION, "Input revision changed: stop"
    assert "厚度17.14" in spec["dimensions_mm"] and spec["mass_g"] == 58.9
    parts, hulls, shift, measured = build()
    texture = INPUT / "derived/steel_photo.png"
    texture_provenance = metal_texture(texture)
    (INPUT / "derived/texture_provenance.json").write_text(
        json.dumps(texture_provenance, ensure_ascii=False, indent=2) + "\n"
    )
    every = np.concatenate([p for p, _ in parts.values()])
    faces = [f for _, fs in parts.values() for f in fs]
    for idx, palette in enumerate(PALETTES):
        d = Path(rigid_dir) / "paper_knife" / f"{idx:05d}"
        d.mkdir(parents=True, exist_ok=True)
        oid = f"Rigid/paper_knife/{idx:05d}"
        uid = str(uuid.uuid5(uuid.NAMESPACE_URL, f"robodojo/rigid/paper_knife/{idx}"))
        write_usdz(d / "object.usdz", parts, hulls, palette, texture)
        meta = pc.metadata(uid, {"pts": every, "faces": faces, "contact_r": 0.01}, MASS, shift)
        meta["physics"]["size"] = np.ptp(every, axis=0).tolist()
        meta["measurements_mm"] = {k: round(v, 2) for k, v in measured.items()}
        meta["visual"] = {"color_parts": ["model"]}
        meta["randomization"] = {"rt_color": True, "rt_scale": 0, "requires_task_opt_in": True}
        meta["provenance"] = {
            "registration_revision": REVISION,
            "reference_source": "physical photographs",
            "simulation_use": "rigid grasping snapshot, no cutting or blade actuation",
            "measurements": "four exterior dimensions and base mass measured",
            "local_geometry": "photo-fitted contours, local thicknesses and slider/rib detail; not measured",
            "missing_photos": ["back face", "bottom face"],
            "friction": "project defaults, not measured",
            "material_calibrated": False,
            "mass_basis": "measured" if idx == 0 else "inherited design assumption",
        }
        meta["reconstruction_parameters_mm"] = {
            "base_wall": 3.0,
            "steel_sheet": 1.5,
            "slider_rib_pitch": 1.55,
            "source": "photo-fitted estimates, not measured",
        }
        meta["collision"] = {
            "representation": "explicit convex decomposition",
            "hulls": len(hulls),
            "max_vertices": max(map(len, hulls)),
            "tail_aperture_preserved": True,
        }
        if idx:
            meta["variant_of"] = "Rigid/paper_knife/00000"
            meta["variant_attributes"] = {"color": {"shell": palette[0], "fixed_tail_rim": palette[2]}}
        (d / "metadata.json").write_text(json.dumps(meta, ensure_ascii=False, indent=2) + "\n")
        desc = {
            "uuid": uid,
            "caption": {
                "color": [palette[0] + " shell", palette[2] + " tail rim", "black slider"],
                "material": ["PP shell", "steel guide", "steel segmented blade"],
                "size": ["152.88 mm long", "39.44 mm maximum width", "17.14 mm thick"],
                "name": ["paper knife", "utility knife", "box cutter"],
                "shape": ["elongated open guide", "pierced flared tail", "ribbed trapezoid slider"],
                "function": ["being grasped", "rigid manipulation", "tabletop pickup"],
            },
            "description": [
                "plastic paper knife with an exposed steel guide",
                "segmented metal blade recessed between folded guide lips",
                "ribbed black thumb slider fixed in its photographed pose",
                f"flared tail with an open hanging hole and {palette[2]} rim",
                "elongated rigid knife with a plastic waist suitable for clamping",
            ],
        }
        (d / "description.json").write_text(json.dumps(desc, indent=2) + "\n")
        runtime = {
            "physics": {"mass": MASS},
            "visual": {"color_parts": ["model"]},
        }
        (d / "runtime_config.json").write_text(json.dumps(runtime, indent=2) + "\n")
        print(oid, measured, len(hulls), "hulls", hashlib.sha256((d / "object.usdz").read_bytes()).hexdigest())


if __name__ == "__main__":
    try:
        main(sys.argv[1] if len(sys.argv) > 1 else ROOT / "Assets/Object/Real2Sim/Rigid")
    except Exception:
        if mp.APP is None:
            raise
        traceback.print_exc()
        sys.stderr.flush()
        os._exit(1)
    if mp.APP is not None:
        mp.APP.close()
