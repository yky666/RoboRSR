"""Measured wipe packet, photo textures and three appearance variants.

Source: state registration Rigid/wipe_bag/00000, 2026-10-30 09:20:27.
Metres, Z up, X length; origin is the COMPLETE visible bounding-box centre.
Whole filled-packet equivalent FEM is a research assumption, not film calibration.
The closed lid is separate visual geometry, embedded in the bulk FEM; its rigid
attachment, hinge, film layers and individual wipes are NOT independently solved.
Usage: python scripts/objects/make_wipe_bag.py [TYPE_DIR]
"""

import argparse
import hashlib
import json
from pathlib import Path
import shutil
import uuid

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
REVISION = "13fbcc90080f426434b76059f60cf51ffa0631d4e1dbac275c8f89dda584d8d5"
L, W, H = 0.190, 0.09602, 0.038  # measured complete packet, including lid and seals
PROFILES = [
    ("original", (0.63, 0.45, 0.76), (0.91, 0.89, 0.81), 0.39, 0.27),
    ("mint", (0.20, 0.65, 0.53), (0.91, 0.94, 0.88), 0.64, 0.46),
    ("coral", (0.89, 0.32, 0.27), (0.98, 0.89, 0.72), 0.24, 0.18),
    ("navy", (0.12, 0.27, 0.53), (0.76, 0.86, 0.95), 0.48, 0.34),
]
CAPS = [(0.91, 0.90, 0.86), (0.11, 0.39, 0.25), (0.59, 0.10, 0.08), (0.08, 0.17, 0.35)]


def dump(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def validate(spec_path):
    spec = json.loads(spec_path.read_text(encoding="utf-8"))
    prompt = spec_path.with_name("modeling_prompt.md").read_text(encoding="utf-8")
    snapshot = json.JSONDecoder().raw_decode(prompt[prompt.index("{", prompt.index("##")) :])[0]
    digest = hashlib.sha256(
        json.dumps({"version": "real-object-v3", "inputs": snapshot}, sort_keys=True, ensure_ascii=False).encode()
    ).hexdigest()
    assert digest == REVISION == spec["modeling_confirmation"]["revision"], "stale prompt"
    aliases = {
        "dimensions": "dimensions_mm",
        "real_mass": "mass_g",
        "real_friction": "static_friction",
        "gen": "generalization",
    }
    for key, expected in snapshot.items():
        if key in ("photos", "existing_model", "asset_origin"):
            continue
        actual = spec.get(aliases.get(key, key))
        if key == "gen":
            actual = {k: v for k, v in (actual or {}).items() if v not in (None, "", False, 0)}
            expected = {k: v for k, v in (expected or {}).items() if v not in (None, "", False, 0)}
        assert actual == expected, (key, actual, expected)
    assert spec["asset_origin"] == "physical_object" and spec["twin"] is True
    for entry in snapshot["photos"]:
        data = (spec_path.parent / "reference" / Path(entry["path"]).name).read_bytes()
        assert len(data) == entry["bytes"] and hashlib.sha256(data).hexdigest() == entry["sha256"]
    assert spec["dimensions_mm"] == "长度190，宽度96.02，高度38" and spec["mass_g"] == 240.2
    return spec


def section(x, theta):
    """Photo-fit section and end folds; all local proportions are design assumptions."""
    t = abs(x) / (L / 2)
    end = np.clip((t - 0.72) / 0.28, 0, 1)
    width = W / 2 * (1 - 0.13 * end)
    half = 0.01725 * (1 - 0.60 * end**1.7)
    centre = -0.00175 + 0.0028 * end
    c, s = np.cos(theta), np.sin(theta)
    y = width * np.sign(c) * abs(c) ** 0.5
    z = centre + half * np.sign(s) * abs(s) ** 0.5
    wrinkle = 0.0030 * np.sin(164 * x + 4 * theta) * np.sin(5 * theta) ** 2
    wrinkle *= (0.15 + end) * abs(c) ** 2
    wrinkle += 0.0018 * np.sin(86 * x + 2 * theta) * abs(c) ** 6 * (1 - end)
    z += wrinkle
    return np.array([x, y, z])


def packet():
    xs = np.linspace(-L / 2, L / 2, 81)
    n = 80
    points = np.array([section(x, k * 2 * np.pi / n) for x in xs for k in range(n)])
    faces = []
    groups = []
    for i in range(len(xs) - 1):
        for k in range(n):
            a, b = i * n + k, i * n + (k + 1) % n
            c, d = a + n, b + n
            faces.extend([(a, b, c), (b, d, c)])
            groups.extend([int(abs((xs[i] + xs[i + 1]) / 2) > 0.069)] * 2)
    for i, sign in ((0, -1), (len(xs) - 1, 1)):
        centre = len(points)
        points = np.vstack([points, [xs[i], 0, 0.00105]])
        for k in range(n):
            a, b = i * n + k, i * n + (k + 1) % n
            faces.append((centre, b, a) if sign < 0 else (centre, a, b))
            groups.append(1)
    # Preserve measured maxima despite local folds; lid sets the upper Z bound.
    points[:, 1] *= W / np.ptp(points[:, 1])
    points[points[:, 1] < 0, 1] *= (W / 2 - 0.00015) / (W / 2)
    points[:, 2] += -0.019 - points[:, 2].min()
    return points, np.array(faces), np.array(groups)


def rounded_loop(a, b, radius, count=12):
    result = []
    for cx, cy, angle in [
        (a - radius, b - radius, 0),
        (-a + radius, b - radius, 90),
        (-a + radius, -b + radius, 180),
        (a - radius, -b + radius, 270),
    ]:
        for t in np.linspace(angle, angle + 90, count, endpoint=False):
            result.append([cx + radius * np.cos(np.deg2rad(t)), cy + radius * np.sin(np.deg2rad(t))])
    return np.array(result)


def slab(a, b, radius, z0, z1, inner=None):
    outer = rounded_loop(a, b, radius)
    n = len(outer)
    if inner:
        hole = rounded_loop(*inner)
        points = np.array([[x, y, z] for loop in (outer, hole) for z in (z0, z1) for x, y in loop])
        faces = []
        for k in range(n):
            j = (k + 1) % n
            for a0, b0, c0, d0 in [
                (k, j, n + k, n + j),
                (2 * n + j, 2 * n + k, 3 * n + j, 3 * n + k),
                (n + k, n + j, 3 * n + k, 3 * n + j),
                (j, k, 2 * n + j, 2 * n + k),
            ]:
                faces.extend([(a0, b0, c0), (b0, d0, c0)])
    else:
        points = np.array([[x, y, z] for z in (z0, z1) for x, y in outer] + [[0, 0, z0], [0, 0, z1]])
        faces = []
        for k in range(n):
            j = (k + 1) % n
            faces.extend([(k, j, n + k), (j, n + j, n + k), (2 * n, j, k), (2 * n + 1, n + k, n + j)])
    return points, np.array(faces)


def grid_panel(x0, x1, v0, v1, mapper, nx=31, ny=17):
    points, uv, faces = [], [], []
    for j, v in enumerate(np.linspace(v0, v1, ny)):
        for i, x in enumerate(np.linspace(x0, x1, nx)):
            points.append(mapper(x, v))
            uv.append([i / (nx - 1), j / (ny - 1)])
    for j in range(ny - 1):
        for i in range(nx - 1):
            a = j * nx + i
            faces.extend([(a, a + 1, a + nx), (a + 1, a + nx + 1, a + nx)])
    return np.array(points), np.array(faces), np.array(uv)


def textures(reference, output, variant):
    """Rectify real pixels; 10 px/mm canvases do not imply added source information."""
    import cv2
    from PIL import Image, ImageDraw

    scale = 4032 / 1824
    entries = [
        (
            "label_lid",
            "72757b43e310bc58a2b3916b07e9755a.jpg",
            [[515, 486], [1107, 469], [1188, 789], [501, 815]],
            [1000, 460],
            [100, 46],
        ),
        (
            "label_instructions",
            "31c71df6cf84f97880e007a08224aff0.jpg",
            [[610, 449], [1294, 509], [1255, 736], [607, 681]],
            [900, 240],
            [90, 24],
        ),
        (
            "label_count",
            "72757b43e310bc58a2b3916b07e9755a.jpg",
            [[1170, 937], [1359, 968], [1358, 1040], [1161, 1007]],
            [250, 95],
            [25, 9.5],
        ),
    ]
    info = []
    for name, source, quad, res, mm in entries:
        im = Image.open(reference / source).convert("RGB")
        q = np.float32(quad) * scale
        target = np.float32([[0, res[1] - 1], [res[0] - 1, res[1] - 1], [res[0] - 1, 0], [0, 0]])
        # Target bottom-up here; USD UV v increases upward. Save conventional image orientation below.
        target[:, 1] = res[1] - 1 - target[:, 1]
        mat = cv2.getPerspectiveTransform(q, target)
        crop = cv2.warpPerspective(np.array(im), mat, tuple(res), flags=cv2.INTER_CUBIC)
        # Remove broad luminance gradients only, preserving brand colours and print strokes.
        lum = cv2.cvtColor(crop, cv2.COLOR_RGB2GRAY).astype(np.float32)
        broad = cv2.GaussianBlur(lum, (0, 0), max(res) / 9)
        gain = np.clip(np.median(broad) / np.maximum(broad, 1), 0.82, 1.18)
        crop = np.clip(crop.astype(float) * gain[..., None], 0, 255).astype(np.uint8)
        if name != "label_lid":
            # Appearance variants tint only the unprinted film background; photo print pixels remain.
            gray = cv2.cvtColor(crop, cv2.COLOR_RGB2GRAY)
            hsv = cv2.cvtColor(crop, cv2.COLOR_RGB2HSV)
            mask = (gray > 110) & (hsv[..., 0] > 115) & (hsv[..., 0] < 170) & (hsv[..., 1] > 15)
            colour = np.array(PROFILES[variant][1]) * 255
            value = np.clip(gray[mask, None] / max(float(np.median(gray[mask])), 1), 0.94, 1.06)
            crop[mask] = np.clip(colour * value, 0, 255).astype(np.uint8)
        Image.fromarray(crop).save(output / (name + ".png"))
        draw = ImageDraw.Draw(im)
        draw.line([tuple(p) for p in q] + [tuple(q[0])], fill="red", width=10)
        draw.text(tuple(q[0]), name, fill="red", stroke_width=2)
        im.thumbnail((1200, 900))
        im.save(output / (name + "_source_bounds.jpg"))
        info.append(
            dict(
                name=name,
                source=source,
                source_quad_px=q.tolist(),
                size_mm=mm,
                texture_size_px=res,
                canvas_px_per_mm=10,
                source_effective_px_per_mm="lid approximately 12-15; side approximately 12-17; count 15-17",
                source_sha256=hashlib.sha256((reference / source).read_bytes()).hexdigest(),
            )
        )
    return info


def mesh(stage, path, points, faces, uv=None):
    from pxr import UsdGeom, Vt

    m = UsdGeom.Mesh.Define(stage, path)
    m.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(points.astype(np.float32)))
    m.CreateFaceVertexCountsAttr([3] * len(faces))
    m.CreateFaceVertexIndicesAttr(faces.ravel().tolist())
    m.CreateSubdivisionSchemeAttr("none")
    m.CreateDoubleSidedAttr(True)
    if uv is not None:
        from pxr import Sdf

        p = UsdGeom.PrimvarsAPI(m).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray, "vertex")
        p.Set(Vt.Vec2fArray.FromNumpy(uv.astype(np.float32)))
    return m


def material(stage, path, colour, roughness, texture=None):
    from pxr import Sdf, UsdShade

    m = UsdShade.Material.Define(stage, path)
    shader = UsdShade.Shader.Define(stage, path + "/pbr")
    shader.CreateIdAttr("UsdPreviewSurface")
    c = np.array(colour)
    linear = np.where(c <= 0.04045, c / 12.92, ((c + 0.055) / 1.055) ** 2.4)
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(tuple(linear))
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0)
    if texture:
        tex = UsdShade.Shader.Define(stage, path + "/texture")
        tex.CreateIdAttr("UsdUVTexture")
        tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(str(texture.resolve()))
        tex.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set("sRGB")
        reader = UsdShade.Shader.Define(stage, path + "/uv")
        reader.CreateIdAttr("UsdPrimvarReader_float2")
        reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
        tex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(reader.ConnectableAPI(), "result")
        shader.GetInput("diffuseColor").ConnectToSource(tex.ConnectableAPI(), "rgb")
    m.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return m


def build(type_dir, spec_path):
    import isaaclab.sim as sim_utils
    from isaacsim.core.utils.stage import get_current_stage
    from pxr import Sdf, Usd, UsdGeom, UsdShade, UsdUtils
    import trimesh

    spec = validate(spec_path)
    points, faces, groups = packet()
    surface = trimesh.Trimesh(points, faces, process=False)
    assert surface.is_watertight and surface.volume > 0
    sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=1 / 240, device="cuda:0"))
    stage = get_current_stage()
    UsdGeom.SetStageUpAxis(stage, "Z")
    UsdGeom.SetStageMetersPerUnit(stage, 1)
    outputs = []
    for index, (tag, colour, cream, rough, caprough) in enumerate(PROFILES):
        dest = type_dir / "wipe_bag" / f"{index:05d}"
        dest.mkdir(parents=True, exist_ok=False)
        texdir = dest / "textures"
        texdir.mkdir()
        labels = textures(spec_path.parent / "reference", texdir, index)
        root = f"/asset{index}"
        UsdGeom.Xform.Define(stage, root)
        cfg = sim_utils.MeshCuboidCfg(
            size=(L, W, H),
            mass_props=sim_utils.MassPropertiesCfg(mass=0.2402),
            deformable_props=sim_utils.DeformableBodyPropertiesCfg(
                simulation_hexahedral_resolution=24,
                solver_position_iteration_count=64,
                rest_offset=0,
                contact_offset=0.0005,
                sleep_threshold=0,
                settling_threshold=0,
                sleep_damping=0,
            ),
            physics_material=sim_utils.DeformableBodyMaterialCfg(youngs_modulus=80000, poissons_ratio=0.3),
        )
        cfg.func(root + "/model", cfg)
        body = mesh(stage, root + "/model/geometry/mesh", points, faces)
        film = material(stage, root + "/_materials/film", colour, rough)
        seal = material(stage, root + "/_materials/seal", cream, rough)
        UsdShade.MaterialBindingAPI.Apply(body.GetPrim()).Bind(film)
        subset = UsdGeom.Subset.CreateGeomSubset(
            body, "cream_ends", UsdGeom.Tokens.face, np.flatnonzero(groups).tolist(), "materialBind"
        )
        UsdShade.MaterialBindingAPI.Apply(subset.GetPrim()).Bind(seal)
        white = material(stage, root + "/_materials/cap", CAPS[index], caprough)
        dark = material(stage, root + "/_materials/seam", (0.56, 0.54, 0.52), 0.7)
        parts = []
        parts.append(
            ("lid_frame", *slab(0.060, 0.032, 0.009, 0.0134, 0.0189, inner=(0.0546, 0.0268, 0.007)), None, white)
        )
        parts.append(("lid_seam", *slab(0.0545, 0.0267, 0.007, 0.0155, 0.01825), None, dark))
        parts.append(("lid_panel", *slab(0.0542, 0.0264, 0.0068, 0.016, 0.01855), None, white))
        latch_p, latch_f = slab(0.010, 0.004, 0.0015, 0.016, 0.019)
        latch_p[:, 1] -= 0.0305
        parts.append(("lid_latch", latch_p, latch_f, None, white))
        hinge_p, hinge_f = slab(0.018, 0.0018, 0.001, 0.014, 0.0184)
        hinge_p[:, 1] += 0.0295
        parts.append(("lid_hinge", hinge_p, hinge_f, None, white))
        # Label lid artwork is a separate rounded panel. UV uses full rectified photo.
        lp = rounded_loop(0.050, 0.023, 0.004)
        lp = np.column_stack([lp, np.full(len(lp), 0.0187)])
        lp = np.vstack([lp, [0, 0, 0.0187]])
        lf = np.array([(len(lp) - 1, k, (k + 1) % (len(lp) - 1)) for k in range(len(lp) - 1)])
        uv = np.column_stack([lp[:, 0] / 0.100 + 0.5, lp[:, 1] / 0.046 + 0.5])
        parts.append(("label_lid", lp, lf, uv, None))

        def side(x, z):
            # Match printed front on the -Y face. Surface section has a fourth-power cross section.
            half = 0.01725
            v = np.clip((z + 0.00175) / half, -0.99, 0.99)
            theta = np.pi - np.arcsin(np.sign(v) * abs(v) ** 2)
            point = section(x, theta)
            point[1] *= (W / 2 - 0.00015) / (W / 2)
            point[1] -= 0.00015
            return point

        sp, sf, suv = grid_panel(-0.045, 0.045, -0.014, 0.010, side)
        # Reverse horizontal UV to keep type readable when viewed from -Y.
        parts.append(("label_instructions", sp, sf, suv, None))

        def top(x, y):
            theta = np.arccos(np.sign(y) * (abs(y) / (W / 2)) ** 2)
            point = section(x, theta)
            point[1] *= (W / 2 - 0.00015) / (W / 2)
            point[2] += 0.00015
            return point

        cp, cf, cuv = grid_panel(0.026, 0.051, -0.0425, -0.033, top)
        parts.append(("label_count", cp, cf, cuv, None))
        descriptors = []
        allpoints = [points]
        for name, p, f, uv, mat in parts:
            m = mesh(stage, root + "/model/visual/" + name, p, f, uv)
            if mat is None:
                mat = material(stage, root + "/_materials/" + name, (1, 1, 1), rough, texdir / (name + ".png"))
            UsdShade.MaterialBindingAPI.Apply(m.GetPrim()).Bind(mat)
            sha = hashlib.sha256(p.astype("<f4").tobytes() + f.astype("<i4").ravel().tobytes()).hexdigest()
            descriptors.append({"prim": "model/visual/" + name, "mesh_sha256": sha})
            allpoints.append(p)
        visible = np.vstack(allpoints)
        measured = np.ptp(visible, axis=0) * 1000
        lo, hi = visible.min(0), visible.max(0)
        assert np.allclose((lo + hi) / 2, 0, atol=0.00001), (lo, hi)
        assert np.allclose(measured, [190, 96.02, 38], atol=0.01), measured
        outputs.append((root, dest, index, labels, descriptors, visible, surface.volume))
    sim.reset()  # PhysX cooks real tetrahedra; output is a reusable FEM USD, not only a surface mesh.
    for root, dest, index, labels, descriptors, visible, volume in outputs:
        standalone = Usd.Stage.CreateInMemory()
        Sdf.CopySpec(stage.Flatten(), root, standalone.GetRootLayer(), "/root")
        # CopySpec preserves absolute material targets; remap these into the exported root.
        standalone.GetRootLayer().ImportFromString(
            standalone.GetRootLayer().ExportToString().replace(root + "/", "/root/")
        )
        standalone.SetDefaultPrim(standalone.GetPrimAtPath("/root"))
        UsdGeom.SetStageUpAxis(standalone, "Z")
        UsdGeom.SetStageMetersPerUnit(standalone, 1)
        standalone.GetRootLayer().Export(str(dest / "object.usdc"))
        assert UsdUtils.CreateNewUsdzPackage(Sdf.AssetPath(str(dest / "object.usdc")), str(dest / "object.usdz"))
        uid = str(uuid.uuid5(uuid.NAMESPACE_URL, f"robodojo/rigid/wipe_bag/{index}"))
        lo, hi = visible.min(0), visible.max(0)
        box = {
            "vertices": [[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])],
            "extents": (hi - lo).tolist(),
        }
        meta = {
            "uuid": uid,
            "asset_id": f"Rigid/wipe_bag/{index:05d}",
            "physics": {"mass": 0.2402, "size": (hi - lo).tolist()},
            "geometry": {"aligned_bbox": box, "oriented_bbox": box, "bbox": (hi - lo).tolist(), "center": [0, 0, 0]},
            "measurements_mm": dict(zip(["长度", "宽度", "高度"], np.round((hi - lo) * 1000, 2).tolist())),
            "active": {"place": {}, "functional": {}},
            "passive": {},
            "backend": "isaac_physx_fem",
            "deformable": {"family": "bag"},
            "visual": {"color_parts": ["model/geometry/mesh"]},
            "generator": "scripts/objects/make_wipe_bag.py",
            "registration_revision": REVISION,
            "origin": "complete visible bbox centre; metres; Z up; X length; Y width",
            "acceptance": "research_only",
            "real_material_calibrated": False,
            "scope": "filled packet equivalent bulk FEM; closed lid visuals follow bulk, no independent rigid lid",
            "candidate_material": {
                "youngs_modulus_pa": 80000,
                "poissons_ratio": 0.3,
                "source": "unfitted hypothesis",
                "equivalent_density_kg_m3": 0.2402 / volume,
            },
            "labels": labels,
            "dimension_policy": "measured" if index == 0 else "inherited",
            "mass_policy": "measured" if index == 0 else "inherited",
            "estimated_dimensions_mm": {"lid_outer_length": 120, "lid_outer_width": 64, "lid_thickness": 5.6},
            "missing_photos": ["bottom", "opposite long side", "open lid and dispensing aperture"],
            "variant_parameters": {
                "film_colour": PROFILES[index][1],
                "end_colour": PROFILES[index][2],
                "film_roughness": PROFILES[index][3],
                "lid_roughness": PROFILES[index][4],
            },
            "object_sha256": hashlib.sha256((dest / "object.usdz").read_bytes()).hexdigest(),
        }
        if index:
            meta["variant_of"] = "Rigid/wipe_bag/00000"
        dump(dest / "metadata.json", meta)
        dump(
            dest / "runtime_config.json", {"physics": {"backend": "isaac_physx_fem", "fem_visual_meshes": descriptors}}
        )
        dump(
            dest / "description.json",
            {
                "uuid": uid,
                "caption": {
                    "name": ["bag", "wipe bag", "pet wipe bag"],
                    "color": ["printed film", ["white lid", "green lid", "red lid", "blue lid"][index], "cream seals"],
                    "material": ["plastic film", "plastic lid", "laminated packet"],
                    "size": ["190 mm long", "96.02 mm wide", "38 mm high"],
                    "shape": ["pillow packet", "sealed ends", "rounded rectangular lid"],
                    "function": ["holding wipes", "dispensing wipes", "protecting moist wipes"],
                },
                "description": [
                    "A filled pet wipe packet with sealed ends.",
                    "A rounded rectangular lid covers the top.",
                    "Pet illustrations are printed on the lid.",
                    "Instructions are printed on one long side.",
                    "The packet has a wrinkled opaque plastic film exterior.",
                ],
            },
        )
        dump(
            dest / "parameter_sources.json",
            {
                "registration_updated_at": spec["updated_at"],
                "registration_by": spec["updated_by"],
                "modeling_by": "刘小凡 / Codex",
                "measured": {"length_mm": 190, "width_mm": 96.02, "height_mm": 38, "mass_g": 240.2},
                "unknown": {
                    "film_thickness_mm": None,
                    "film_mass_g": None,
                    "lid_mass_g": None,
                    "contents_mass_g": None,
                    "friction": None,
                    "material_law": None,
                },
                "local_geometry": "photo-fit assumptions; not additional measurements",
                "mass_decomposition": "none: 240.2 g applies to the complete equivalent package",
                "material": meta["candidate_material"],
                "missing_photos": meta["missing_photos"],
            },
        )
        dump(dest / "label_sources.json", labels)
        shutil.copy2(spec_path, dest / "registration_snapshot.json")
        print("GENERATED", dest, meta["object_sha256"], flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("type_dir", type=Path, nargs="?", default=Path("Assets/Object/Real2Sim/Rigid"))
    parser.add_argument("--spec", type=Path, default=ROOT / "real2sim/objects/Rigid/wipe_bag/00000/spec.json")
    parser.add_argument(
        "--refresh-descriptions",
        action="store_true",
        help="Refresh generated lid colour captions without changing USD or physics",
    )
    args = parser.parse_args()
    validate(args.spec)
    if args.refresh_descriptions:
        for index, colour in enumerate(("white", "green", "red", "blue")):
            path = args.type_dir / "wipe_bag" / f"{index:05d}" / "description.json"
            data = json.loads(path.read_text())
            data["caption"]["color"][1] = colour + " lid"
            dump(path, data)
        return
    from isaacsim import SimulationApp

    app = SimulationApp({"headless": True, "fast_shutdown": True})
    try:
        build(args.type_dir, args.spec)
    except Exception:
        import os
        import traceback

        traceback.print_exc()
        os._exit(1)
    app.close(skip_cleanup=True)


if __name__ == "__main__":
    main()
