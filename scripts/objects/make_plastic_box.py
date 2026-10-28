#!/usr/bin/env python3
"""Two-tone square PP storage box (open top, no lid) as RoboDojo `Rigid` assets.

- plastic_box/00000: Real2Sim platform registration real2sim/objects/Rigid/plastic_box/00000 (spec.json, by 徐鹤鸣,
  2026-10-28 12:45:09): PP plastic, white lower body, grey upper band, 151 g; 外边长 151.5 mm, 内边长 140.2 mm
  (measured). A white body with a flat floor, slightly tapered walls and rounded bottom edges, under a thick grey band
  around the open top that overhangs the body; height, band height, overhang, taper and radii are estimated from the
  two photos (BOX).
- plastic_box/00001-00006: generalization variants of 00000 (not measured; colour, material, size and shape differ):
  pink / white, tall beige / brown, shallow rectangular navy tray, very rounded yellow, translucent body with a black
  band, small mint box (BOX_VARIANTS).

Usage:
    python scripts/objects/make_plastic_box.py [RIGID_DIR]    # default Assets/Object/Real2Sim/Rigid
    python scripts/objects/check_object.py Assets/Object/Real2Sim/Rigid/plastic_box/0000{0..6}
Writes RIGID_DIR/plastic_box/<index>/{object.usdz, metadata.json, description.json}. Needs what make_pitcher.py needs.

Frame: Z up, metres, origin at bbox centre; box sides face +-X and +-Y.
USD:   /root (RigidBodyAPI) -> visual/model (white body), visual/band (grey band around the top)
                            -> collision/hull_NN (floor slab + wall staves around the sides and corners, open top)
Both parts are lofted rounded-rectangle rings along Z. The band wraps the top of the body: its inner face drops LEDGE
below the rim to a small ledge where the narrower white wall starts, and it hangs BAND below the rim outside.
"""

import json
import os
from pathlib import Path
import sys
import tempfile
import traceback
import uuid

import numpy as np
from scipy.spatial import ConvexHull

sys.path.insert(0, str(Path(__file__).resolve().parent))
import make_pitcher as mp  # noqa: E402  (shared USD helpers; starts Kit when pxr needs it)
from make_pitcher import Sdf, Usd, UsdGeom, UsdPhysics, UsdShade  # noqa: E402
import make_plastic_cups as pc  # noqa: E402  (loft, wall_staves, cavity_probe, add_collision, metadata)

RIGID_DIR = Path(__file__).resolve().parents[2] / "Assets/Object/Real2Sim/Rigid"
N_THETA = pc.N_THETA  # 192: per quadrant 8 points along each half side and 32 around the corner
N_SIDE, N_ARC = 8, N_THETA // 4 - 16
HULL_MAX, FACE_MAX = 64, 32  # PhysX GPU convex limits: vertices per hull, vertices per face
SLAB_ARC = (0, 6, 13, 19, 26, 32)  # corner points of the floor slab, per quadrant: 24 in all
EPS = 0.0003  # hidden overlaps: band inside the body wall, body top just under the band's ledge

BOX = dict(
    A=0.07575, B=0.07575,  # half 外边长, measured: outer side across the grey band (X, Y)
    RIM_T=0.00565,  # band wall at the rim, (外边长 151.5 - 内边长 140.2) / 2, measured; 内边长: opening in the band
    H=0.100,  # total height, estimated: front photo, front face 478 px high on a 726 px wide band
    BAND=0.027,  # grey band height, estimated: 127 px of 726 in the front photo
    LEDGE=0.008,  # band inner face depth below the rim, down to the white wall, estimated (far side, front photo)
    OVERHANG=0.0085,  # band outer face to the white body outer face just under the band, estimated (body 134.5 mm)
    TAPER=0.0025,  # white body narrows by this much per side from the band to the floor, estimated
    R=0.018, R_BODY=0.014,  # plan-view corner radii of the band and of the body outside, estimated
    T_WALL=0.002, T_BOTTOM=0.0025,  # body wall and floor thickness, estimated
    FILLET=0.008, IN_FILLET=0.006,  # outer bottom edge and inner floor edge radii, estimated
    R_TOP=0.003, R_LOW=0.0015,  # band top outer edge and bottom outer edge radii, estimated
    MASS=0.151,  # measured
    LOOK_BODY=((0.82, 0.83, 0.84), 0.40), LOOK_BAND=((0.20, 0.20, 0.21), 0.45),  # white PP, mid grey PP (photos)
    WORDS=dict(
        color=["white and grey", "white body with a grey rim", "two-tone"],
        material=["plastic", "polypropylene", "PP"],
        name=["storage box", "plastic box", "square bin"],
        shape=["rounded square", "open top without a lid", "thick rim band around the top"],
        function=["storing small items", "holding objects", "sorting things on a desk"],
        look="white square plastic storage box with a thick grey band around its open top",
    ),
)  # fmt: skip
# generalization variants 00001-00006: overrides of BOX, all estimated; mass = MASS x shell area / area of 00000;
# LOOK_*: (rgb, roughness) opaque OmniPBR, or ("clear", rgb) translucent PP (pc.clear_pp)
BOX_VARIANTS = [
    dict(
        LOOK_BODY=((0.82, 0.45, 0.52), 0.45), LOOK_BAND=((0.88, 0.88, 0.87), 0.40),
        WORDS=dict(
            color=["pink and white", "pink body with a white rim", "pastel pink"],
            material=["plastic", "polypropylene", "matte plastic"],
            name=["storage box", "pink plastic box", "square bin"],
            shape=["rounded square", "open top without a lid", "white rim band around the top"],
            function=["storing toys", "holding objects", "sorting things on a desk"],
            look="pastel pink square plastic box with a white band around its open top",
        ),
    ),
    dict(
        A=0.090, B=0.065, H=0.140, BAND=0.030, R=0.016, R_BODY=0.012, FILLET=0.009, TAPER=0.004,
        LOOK_BODY=((0.74, 0.64, 0.48), 0.50), LOOK_BAND=((0.22, 0.14, 0.09), 0.45),
        WORDS=dict(
            color=["beige and brown", "beige body with a brown rim", "two-tone"],
            material=["plastic", "polypropylene", "matte plastic"],
            name=["tall storage bin", "plastic container", "rectangular bin"],
            shape=["tall rounded rectangle", "open top without a lid", "brown rim band around the top"],
            function=["storing kitchen items", "holding objects", "keeping things upright"],
            look="tall beige rectangular plastic bin with a dark brown band around its open top",
        ),
    ),
    dict(
        A=0.100, B=0.070, H=0.060, BAND=0.020, LEDGE=0.006, TAPER=0.002, FILLET=0.006, IN_FILLET=0.005,
        LOOK_BODY=((0.12, 0.32, 0.65), 0.40), LOOK_BAND=((0.03, 0.06, 0.18), 0.40),
        WORDS=dict(
            color=["blue and navy", "blue body with a navy rim", "two-tone blue"],
            material=["plastic", "polypropylene", "PP"],
            name=["shallow tray", "plastic tray box", "sorting tray"],
            shape=["shallow rounded rectangle", "open top without a lid", "navy rim band"],
            function=["sorting small parts", "holding objects", "serving as a desk tray"],
            look="shallow blue rectangular plastic tray with a navy band around its open top",
        ),
    ),
    dict(
        R=0.040, R_BODY=0.034, OVERHANG=0.008,
        LOOK_BODY=((0.90, 0.72, 0.18), 0.45), LOOK_BAND=((0.42, 0.42, 0.43), 0.45),
        WORDS=dict(
            color=["yellow and grey", "yellow body with a grey rim", "sunny yellow"],
            material=["plastic", "polypropylene", "matte plastic"],
            name=["storage box", "round-cornered bin", "plastic box"],
            shape=["strongly rounded square", "open top without a lid", "grey rim band around the top"],
            function=["storing small items", "holding objects", "sorting things on a desk"],
            look="yellow plastic box with big rounded corners and a grey band around its open top",
        ),
    ),
    dict(
        T_WALL=0.0018, LOOK_BODY=("clear", (0.93, 0.95, 0.96)), LOOK_BAND=((0.05, 0.05, 0.06), 0.40),
        WORDS=dict(
            color=["translucent with a black rim", "frosted clear", "clear and black"],
            material=["plastic", "translucent polypropylene", "PP"],
            name=["clear storage box", "translucent bin", "plastic box"],
            shape=["rounded square", "open top without a lid", "black rim band around the top"],
            function=["storing small items", "showing what is inside", "sorting things on a desk"],
            look="frosted translucent square plastic box with a black band around its open top",
        ),
    ),
    dict(
        A=0.056, B=0.056, H=0.075, BAND=0.020, LEDGE=0.006, RIM_T=0.005, OVERHANG=0.007, TAPER=0.002,
        R=0.014, R_BODY=0.010, FILLET=0.006, IN_FILLET=0.004,
        LOOK_BODY=((0.55, 0.80, 0.66), 0.30), LOOK_BAND=((0.05, 0.28, 0.18), 0.35),
        WORDS=dict(
            color=["mint and green", "mint body with a dark green rim", "glossy mint"],
            material=["plastic", "polypropylene", "glossy plastic"],
            name=["small storage box", "plastic cube box", "desk bin"],
            shape=["small rounded square", "open top without a lid", "dark green rim band"],
            function=["holding pens", "storing small items", "sorting things on a desk"],
            look="small glossy mint plastic box with a dark green band around its open top",
        ),
    ),
]  # fmt: skip


def ring(a, b, r):
    """Rounded rectangle of half-sizes (a, b), corner radius r, N_THETA points CCW from (a, 0). Every ring has the same
    point layout (corner arcs at the same indices), so loft and wall staves join rings corner to corner."""
    s1 = np.c_[np.full(N_SIDE, a), np.linspace(0, b - r, N_SIDE + 1)[:-1]]
    t = np.linspace(0, np.pi / 2, N_ARC + 1)[:-1]
    arc = np.c_[a - r + r * np.cos(t), b - r + r * np.sin(t)]
    s2 = np.c_[np.linspace(a - r, 0, N_SIDE + 1), np.full(N_SIDE + 1, b)]
    q = np.concatenate([s1, arc, s2])  # (a, 0) .. (0, b) inclusive
    m = [(1, 1, False), (-1, 1, True), (-1, -1, False), (1, -1, True)]
    return np.concatenate([(q[::-1] if rev else q)[:-1] * (sx, sy) for sx, sy, rev in m])


def shape_of(g):
    zc, zl = g["H"] - g["BAND"], g["H"] - g["LEDGE"]

    def body(z, k=0.0):
        """Body outline at height z (linear taper, continued inside the band), grown inwards by k."""
        s = g["A"] - g["OVERHANG"] - g["TAPER"] * (1 - z / zc) - k
        return ring(s, s - (g["A"] - g["B"]), max(g["R_BODY"] - k, 0.002))

    def band(k=0.0):
        return ring(g["A"] - k, g["B"] - k, g["R"] - k)

    def inner(z):
        return body(z, g["T_WALL"]) if z < zl else band(g["RIM_T"])

    return zc, zl, body, band, inner


def quarter(z0, k0, z1, k1, n=7):
    """Rounded edge from (inset k0, z0) to (inset k1, z1): quarter circle, horizontal tangent at the z0 end."""
    t = np.linspace(0, np.pi / 2, n)
    return [(k0 + (k1 - k0) * np.sin(a), z0 + (z1 - z0) * (1 - np.cos(a))) for a in t]


def build_box(g):
    H, TB, F, IF, T = g["H"], g["T_BOTTOM"], g["FILLET"], g["IN_FILLET"], g["T_WALL"]
    zc, zl, body, band, inner = shape_of(g)
    assert body(zl)[0, 0] + 0.001 < band(g["RIM_T"])[0, 0], "white wall must start inside the band's opening"
    zt = zl - EPS

    outer_edge = [(body(z, k), z) for k, z in quarter(0.0, F, F, 0.0)]
    floor_edge = [(body(z, k), z) for k, z in quarter(TB, T + IF, TB + IF, T)][::-1]
    body_prof = (
        [(0.0, 0.0)] + outer_edge
        + [(body(zt), zt)] * 2 + [(body(zt, T), zt)] * 2
        + floor_edge + [(0.0, TB)]
    )  # fmt: skip
    model = pc.loft(body_prof)

    low = [(band(k), z) for k, z in quarter(zc, g["R_LOW"], zc + g["R_LOW"], 0.0)]
    top = [(band(k), z) for k, z in quarter(H, g["R_TOP"], H - g["R_TOP"], 0.0)][::-1]
    band_prof = (
        [(body(zc, EPS), zc)] + low + top
        + [(band(g["RIM_T"]), H)] * 2 + [(band(g["RIM_T"]), zl)] * 2 + [(body(zl, EPS), zl)] * 2
        + [(body(zc, EPS), zc)]
    )  # fmt: skip
    rim = pc.loft(band_prof)
    parts = {"model": model, "band": rim}
    for name, part in parts.items():
        assert mp.signed_volume(*part) > 0, f"{name} has inward-facing winding"

    sel = [q * N_THETA // 4 + N_SIDE + i for q in range(4) for i in SLAB_ARC]
    hulls = [np.array([(*p, z) for p in body(0.0)[sel] for z in (0.0, TB)])]
    hulls += pc.wall_staves([(0.0, body(0.0), body(0.0, T)), (zc, body(zc), body(zc, T))])
    hulls += pc.wall_staves([(zc, band(), body(zc, T)), (zl, band(), body(zl, T))])
    hulls += pc.wall_staves([(zl, band(), band(g["RIM_T"])), (H, band(), band(g["RIM_T"]))])
    return dict(
        parts=parts, hulls=hulls, probe=pc.cavity_probe(inner, TB, H), ml=pc.volume_ml(inner, TB, H),
        contact_r=min(g["A"], g["B"]) - g["OVERHANG"] - g["TAPER"] - F, area=sum(area(*p) for p in parts.values()),
    )  # fmt: skip


def area(pts, faces):
    return sum(
        np.linalg.norm(np.cross(pts[f[i]] - pts[f[0]], pts[f[i + 1]] - pts[f[0]])) / 2
        for f in faces
        for i in range(1, len(f) - 1)
    )


def self_check(c):
    """Hollow open collider within the PhysX GPU convex limits; pc.self_check: no hull intrudes into the cavity."""
    pc.self_check(c)
    for h in c["hulls"]:
        hull = ConvexHull(h)
        _, face = np.unique(np.round(hull.equations, 5), axis=0, return_inverse=True)
        per_face = max(len(np.unique(hull.simplices[face.ravel() == i])) for i in range(face.max() + 1))
        assert len(hull.vertices) <= HULL_MAX and per_face <= FACE_MAX, f"hull too big: {len(hull.vertices)} points"


def plastic(path, look):
    if look[0] == "clear":
        return lambda st: pc.clear_pp(st, look[1])
    rgb, roughness = look
    return lambda st: mp.add_material(
        st,
        path,
        "OmniPBR",
        {"diffuse_color_constant": rgb, "reflection_roughness_constant": roughness, "metallic_constant": 0.0},
        rgb,
        1.0,
        roughness,
    )


def write_usdz(out, c, materials):
    tmp = Path(tempfile.mkdtemp()) / "object.usdc"
    st = Usd.Stage.CreateNew(str(tmp))
    UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(st, 1.0)
    root = mp.add_xform(st, "/root")
    st.SetDefaultPrim(root.GetPrim())
    UsdPhysics.RigidBodyAPI.Apply(root.GetPrim())
    mp.add_xform(st, "/root/visual")
    for name, (pts, faces) in c["parts"].items():  # model first: loaders read the first Mesh
        mesh = mp.add_mesh(st, f"/root/visual/{name}", pts, faces, mp.vertex_normals(pts, faces))
        UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(materials[name](st))
    pc.add_collision(st, c["hulls"])
    st.Save()
    zip_writer = getattr(Sdf, "ZipFileWriter", None) or Usd.ZipFileWriter
    zw = zip_writer.CreateNew(str(out))
    zw.AddFile(str(tmp), "object.usdc")
    zw.Save()


def description(uid, g, ml):
    w = g["WORDS"]
    dims = f"{2 * g['A'] * 100:.0f} x {2 * g['B'] * 100:.0f} cm"
    size = [f"about {dims} across", f"about {g['H'] * 100:.0f} cm tall", f"holds about {ml / 1000:.1f} L"]
    caption = {k: size if k == "size" else w[k] for k in ("color", "material", "size", "name", "shape", "function")}
    name, color = w["name"][0], w["color"][0]
    desc = [
        w["look"],
        f"{color} {name} about {dims} across and {g['H'] * 100:.0f} cm tall",
        f"open {name} with a flat floor and rounded corners, holds about {ml / 1000:.1f} litres",
        f"{w['material'][1]} {name} whose rim band sticks out past the lower body",
        f"light {name} grasped by its rim or lifted from below",
    ]
    return {"uuid": uid, "caption": caption, "description": desc}


def measurements(band):
    """spec dimensions_mm of 00000 on the band mesh: 外边长 across the band (X), 内边长 across its top opening (X), on
    the y = 0 line at the rim."""
    pts, _ = band
    line = pts[(np.abs(pts[:, 1]) < 1e-9) & (pts[:, 2] > pts[:, 2].max() - 1e-6)]
    inner = line[:, 0][line[:, 0] > 0].min() - line[:, 0][line[:, 0] < 0].max()
    return {"外边长": round(float(np.ptp(pts[:, 0])) * 1000, 1), "内边长": round(float(inner) * 1000, 1)}


def save(rigid_dir, idx, c, g, mass, extra):
    every = np.concatenate([p for p, _ in c["parts"].values()])
    shift = -(every.min(0) + every.max(0)) / 2
    parts = {k: (p + shift, f) for k, (p, f) in c["parts"].items()}
    c = dict(c, parts=parts, hulls=[h + shift for h in c["hulls"]])
    d = Path(rigid_dir) / "plastic_box" / f"{idx:05d}"
    d.mkdir(parents=True, exist_ok=True)
    uid = str(uuid.uuid5(uuid.NAMESPACE_URL, f"robodojo/rigid/plastic_box/{idx}"))
    materials = {
        "model": plastic("/root/_materials/BodyPlastic", g["LOOK_BODY"]),
        "band": plastic("/root/_materials/BandPlastic", g["LOOK_BAND"]),
    }
    write_usdz(d / "object.usdz", c, materials)
    faces = [f for _, fs in parts.values() for f in fs]
    meta = pc.metadata(uid, dict(pts=every + shift, faces=faces, contact_r=c["contact_r"]), mass, shift)
    meta |= extra(parts)
    (d / "metadata.json").write_text(json.dumps(meta, indent=4, ensure_ascii=False))
    (d / "description.json").write_text(json.dumps(description(uid, g, c["ml"]), indent=4))
    return d, meta


def main(rigid_dir):
    area0 = None
    for idx, over in enumerate([{}] + BOX_VARIANTS):
        g = {**BOX, **over}
        c = build_box(g)
        self_check(c)
        if idx == 0:
            area0, mass = c["area"], BOX["MASS"]
            extra = lambda parts: {"measurements_mm": measurements(parts["band"])}  # noqa: E731
        else:
            mass = round(BOX["MASS"] * c["area"] / area0, 4)
            extra = lambda parts: {"variant_of": "Rigid/plastic_box/00000"}  # noqa: E731
        d, meta = save(rigid_dir, idx, c, g, mass, extra)
        print(
            f"{d}: {c['ml']:.0f} ml, mass {mass} kg, size {meta['physics']['size']}, {len(c['hulls'])} collision hulls "
            f"(max {max(len(ConvexHull(h).vertices) for h in c['hulls'])} points), {meta.get('measurements_mm', '')}"
        )


if __name__ == "__main__":
    try:
        main(sys.argv[1] if len(sys.argv) > 1 else RIGID_DIR)
    except Exception:
        if mp.APP is None:
            raise
        traceback.print_exc()
        sys.stderr.flush()
        os._exit(1)
    if mp.APP is not None:
        mp.APP.close()
