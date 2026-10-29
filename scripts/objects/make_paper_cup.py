#!/usr/bin/env python3
"""Single-wall paper cup (black outside, white inside) and its generalization variants as RoboDojo `Clutter` assets.

- paper_cup/00000: the registered cup, Real2Sim Clutter/paper_cup/00000 (spec.json updated 2026-10-29 13:46:11 by
  徐鹤鸣): mouth outer diameter over the rim 87.9 mm, bottom diameter 60.5 mm, height 136.0 mm, 22 g (measured). Rim
  bead, recessed floor and skirt, side seam and colours are estimated from the three reference photos.
- paper_cup/00001-00006: generalization variants (not measured, VARIANTS): white classic, kraft 12 oz, red espresso,
  wide navy, sage double-wall, tall mustard 20 oz; mass scaled from 00000 by paper area.
The cross-section lives in paper_cup_shape.py (numpy only, `paper_cup_profile`), shared with the deformable twin.

Usage:
    python scripts/objects/make_paper_cup.py [CLUTTER_DIR]    # default Assets/Object/Real2Sim/Clutter
    python scripts/objects/check_object.py Assets/Object/Real2Sim/Clutter/paper_cup/0000{0..6}
Writes CLUTTER_DIR/paper_cup/<index>/{object.usdz, metadata.json, description.json}. Needs what make_pitcher.py needs.

Frame: Z up, metres, origin at bbox centre; the side seam is at azimuth SEAM_AZ.
USD:   /root (RigidBodyAPI) -> visual/model (outer wall, skirt and folded bottom edge; outer colour)
                            -> visual/inner, visual/rim, visual/floor (cavity wall, rolled bead, recessed floor; inner
                               colour), visual/crease (floor-wall crease ring), visual/seam_outer, visual/seam_inner
                               (side seam strips)
                            -> collision/hull_NN (skirt ring, floor slab, wall and rim staves; hollow, open top)
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
import make_plastic_cups as pc  # noqa: E402  (loft, wall staves, cavity check, collision, metadata)
import paper_cup_shape as shape  # noqa: E402
from paper_cup_shape import paper_cup_profile  # noqa: E402,F401  (re-exported for callers of this script)

CLUTTER_DIR = Path(__file__).resolve().parents[2] / "Assets/Object/Real2Sim/Clutter"
N_THETA = pc.N_THETA
SEC = pc.unit_section(2.0)
MASS = 0.022  # measured
SEAM_AZ = -0.45  # rad; side seam (paper blank overlap), one per cup
SEAM_W, SEAM_RAISE = 0.0008, 0.00015  # estimated: thin raised strip outside
SEAM_IN_W, SEAM_IN_RAISE = 0.0005, 0.0001  # estimated: faint line inside
COLLIDER_OUT = 0.0003  # collision wall reaches this far outside the visual wall (~1 mm thick hulls)
FLAT_TOL = 0.0015  # hull points either on the floor plane or at least this far above it (PhysX plane merging)
MAX_HULL_PTS, MAX_FACE_PTS = 64, 32  # PhysX GPU convex limits

# look: sRGB outer colour, outer roughness, sRGB inner (cavity, rim, floor) colour, inner roughness
LOOK = ((0.10, 0.10, 0.11), 0.65, (0.93, 0.90, 0.80), 0.75)  # estimated: matte black / cream white (photos)
SPECULAR = 0.3  # OmniPBR specular_level (default 0.5): uncoated paper, only a faint sheen
WORDS = dict(
    color=["black", "black and white", "matte black"], material=["paper", "coated paperboard", "single-wall paper"],
    size=["about 13.6 cm tall", "large 16 oz cup"], name=["paper cup", "disposable coffee cup", "black paper cup"],
    shape=["tall tapered cylinder", "white rolled rim", "recessed bottom"],
    function=["holding hot drinks", "holding coffee", "drinking on the go"],
    look="tall matte black single-wall paper cup with a white rolled rim and a white inside",
)  # fmt: skip
# generalization variants 00001-00006: overrides of paper_cup_shape.CUP, all estimated
VARIANTS = [
    dict(
        LOOK=((0.96, 0.955, 0.94), 0.7, (0.94, 0.925, 0.88), 0.75),
        WORDS=dict(
            color=["white", "plain white", "off-white"], material=["paper", "coated paperboard", "single-wall paper"],
            size=["about 13.6 cm tall", "large 16 oz cup"], name=["paper cup", "disposable cup", "white paper cup"],
            shape=["tall tapered cylinder", "rolled rim", "recessed bottom"],
            function=["holding hot drinks", "holding water", "drinking on the go"],
            look="plain white single-wall paper cup with a rolled rim",
        ),
    ),
    dict(
        MOUTH_D=0.090, BOTTOM_D=0.059, H=0.112, LOOK=((0.62, 0.45, 0.30), 0.85, (0.80, 0.66, 0.50), 0.85),
        WORDS=dict(
            color=["kraft brown", "light brown", "natural brown"],
            material=["kraft paper", "unbleached paper", "paper"],
            size=["about 11 cm tall", "medium 12 oz cup"], name=["paper cup", "kraft paper cup", "brown coffee cup"],
            shape=["tapered cylinder", "rolled rim", "recessed bottom"],
            function=["holding coffee", "holding tea", "drinking on the go"],
            look="medium kraft brown paper cup made of unbleached paper",
        ),
    ),
    dict(
        MOUTH_D=0.062, BOTTOM_D=0.044, H=0.058, BEAD=0.0026, FLOOR_Z=0.0065,
        LOOK=((0.70, 0.12, 0.13), 0.6, (0.94, 0.925, 0.88), 0.75),
        WORDS=dict(
            color=["red", "deep red", "red and white"], material=["paper", "coated paperboard", "single-wall paper"],
            size=["about 6 cm tall", "small 4 oz cup"], name=["paper cup", "espresso cup", "small red paper cup"],
            shape=["short tapered cylinder", "thin rolled rim", "recessed bottom"],
            function=["holding espresso", "tasting samples", "holding a small drink"],
            look="small red paper espresso cup with a white inside",
        ),
    ),
    dict(
        MOUTH_D=0.090, BOTTOM_D=0.070, H=0.085, FLOOR_Z=0.0080,
        LOOK=((0.12, 0.17, 0.34), 0.6, (0.94, 0.925, 0.88), 0.75),
        WORDS=dict(
            color=["navy blue", "dark blue", "blue and white"],
            material=["paper", "coated paperboard", "single-wall paper"],
            size=["about 8.5 cm tall", "wide 10 oz cup"], name=["paper cup", "wide paper cup", "navy paper cup"],
            shape=["short wide tapered cylinder", "rolled rim", "recessed bottom"],
            function=["holding soup", "holding hot drinks", "holding snacks"],
            look="short wide navy blue paper cup with a white inside",
        ),
    ),
    dict(
        MOUTH_D=0.0895, BOTTOM_D=0.064, SLEEVE=0.0022, LOOK=((0.47, 0.58, 0.47), 0.7, (0.94, 0.925, 0.88), 0.75),
        WORDS=dict(
            color=["sage green", "muted green", "green and white"],
            material=["paper", "double-wall paperboard", "paper"],
            size=["about 13.6 cm tall", "large 16 oz cup"],
            name=["paper cup", "double-wall cup", "green insulated cup"],
            shape=["tall tapered cylinder", "smooth outer sleeve", "rolled rim"],
            function=["holding hot coffee", "keeping hands cool", "drinking on the go"],
            look="tall sage green double-wall paper cup with a smooth insulating outer sleeve",
        ),
    ),
    dict(
        MOUTH_D=0.090, BOTTOM_D=0.061, H=0.158, FLOOR_Z=0.0110,
        LOOK=((0.84, 0.64, 0.16), 0.65, (0.94, 0.925, 0.88), 0.75),
        WORDS=dict(
            color=["mustard yellow", "golden yellow", "yellow and white"],
            material=["paper", "coated paperboard", "paper"],
            size=["about 16 cm tall", "extra large 20 oz cup"],
            name=["paper cup", "tall paper cup", "yellow paper cup"],
            shape=["tall slim tapered cylinder", "rolled rim", "recessed bottom"],
            function=["holding iced drinks", "holding large coffees", "drinking on the go"],
            look="extra tall mustard yellow paper cup with a white inside",
        ),
    ),
]  # fmt: skip


def revolve(loop, welded):
    pts, faces = pc.loft(loop, SEC)
    if welded:  # last ring repeats the first one: merge so the closing edge is smooth
        n = len(pts) - N_THETA
        faces = [[i - n if i >= n else i for i in f] for f in faces]
        pts = pts[:n]
    return pts, faces, mp.vertex_normals(pts, faces)


def strip(r_fn, z0, z1, lo, hi, w, az=SEAM_AZ):
    """Straight seam strip on a conical surface r_fn(z): radially r + lo .. r + hi, w wide, at azimuth az."""
    u, t = np.array([np.cos(az), np.sin(az)]), np.array([-np.sin(az), np.cos(az)])

    def poly(z):
        r = float(r_fn(z))
        return [(r + lo) * u - w / 2 * t, (r + hi) * u - w / 2 * t, (r + hi) * u + w / 2 * t, (r + lo) * u + w / 2 * t]

    pts = np.array([(*q, z) for z in (z0, z1) for q in poly(z)])
    faces = [[3, 2, 1, 0], [4, 5, 6, 7]] + [[i, (i + 1) % 4, 4 + (i + 1) % 4, 4 + i] for i in range(4)]
    pts, faces = mp.unweld(pts, faces)
    return pts, faces, mp.vertex_normals(pts, faces)


def band(z0, z1, outer, inner):
    return pc.wall_staves([(z0, outer(z0) * SEC, inner(z0) * SEC), (z1, outer(z1) * SEC, inner(z1) * SEC)])


def build_cup(params):
    prof = paper_cup_profile(params)
    p = prof["params"]
    parts = {name: revolve(*loop) for name, loop in shape.section_loops(prof).items()}
    for name, part in parts.items():
        assert mp.signed_volume(*part[:2]) > 0, f"{name} has inward-facing winding"
    zt, zb, zf, rc, rb = prof["z_top"], prof["floor_bottom"], prof["floor_z"], prof["bead_rc"], prof["bead_r"]
    z_seam = prof["sleeve_z"] if p["SLEEVE"] > 0 else zt
    parts["seam_outer"] = strip(prof["r_shell"], 2 * prof["fold"], z_seam, -0.0002, SEAM_RAISE, SEAM_W)
    parts["seam_inner"] = strip(prof["r_inner"], zf - p["GROOVE_D"], zt, -SEAM_IN_RAISE, 0.0002, SEAM_IN_W)

    e, ro, ri, rs = COLLIDER_OUT, prof["r_outer"], prof["r_inner"], prof["r_shell"]
    hulls = band(0.0, zb, lambda z: rs(z) + e, prof["r_skirt"])
    hulls.append(np.array([(*(float(ri(zb)) * q), z) for q in SEC[:: N_THETA // pc.BOTTOM_SIDES] for z in (zb, zf)]))
    z_rim = prof["H"] - 2 * rb
    if p["SLEEVE"] > 0:
        zs = prof["sleeve_z"]
        hulls += band(zb, zs, lambda z: ro(z) + p["SLEEVE"] + e, ri) + band(zs, z_rim, lambda z: ro(z) + e, ri)
    else:
        hulls += band(zb, z_rim, lambda z: ro(z) + e, ri)
    hulls += band(z_rim, prof["H"], lambda z: rc + rb + 0 * z, lambda z: rc - rb + 0 * z)

    pts = np.concatenate([q for q, _, _ in parts.values()])
    c = dict(
        parts=parts, pts=pts, faces=[f for _, fs, _ in parts.values() for f in fs], hulls=hulls, prof=prof,
        probe=pc.cavity_probe(lambda z: ri(z) * SEC, zf, prof["H"]), ml=shape.volume_ml(prof),
        contact_r=float(prof["r_skirt"](0.0)) + prof["fold"], area=shape.paper_area(prof),
    )  # fmt: skip
    pc.self_check(c)
    check_hulls(hulls)
    return c


def check_hulls(hulls):
    for h in hulls:
        hull = ConvexHull(h)
        v = h[hull.vertices]
        assert len(v) <= MAX_HULL_PTS, f"hull has {len(v)} points"
        low = v[:, 2] < FLAT_TOL
        assert np.all(v[low, 2] == 0.0), "hull point within FLAT_TOL of the floor plane but not on it"
        assert low.sum() <= MAX_FACE_PTS, "bottom face too large"


def linear(srgb):
    return tuple(float(x) ** 2.2 for x in srgb)


def materials(st, look):
    outer, r_out, inner, r_in = look
    o, i = linear(outer), linear(inner)
    spec = {
        "Outer": (o, r_out),
        "Inner": (i, r_in),
        "SeamOuter": (tuple(0.8 * x + 0.008 for x in o), 0.45),  # catches a little more light than the matte wall
        "SeamInner": (tuple(0.8 * x for x in i), r_in),
        "Crease": (tuple(0.55 * x for x in i), r_in),  # the thin darker ring where the floor meets the wall (pc_2)
    }
    return {
        k: mp.add_material(
            st,
            f"/root/_materials/{k}",
            "OmniPBR",
            {
                "diffuse_color_constant": rgb,
                "reflection_roughness_constant": r,
                "metallic_constant": 0.0,
                "specular_level": SPECULAR,
            },
            rgb,
            1.0,
            r,
        )
        for k, (rgb, r) in spec.items()
    }


MATERIAL_OF = dict(
    model="Outer", inner="Inner", rim="Inner", floor="Inner", crease="Crease", seam_outer="SeamOuter",
    seam_inner="SeamInner",
)  # fmt: skip


def write_usdz(out, parts, hulls, look):
    tmp = Path(tempfile.mkdtemp()) / "object.usdc"
    st = Usd.Stage.CreateNew(str(tmp))
    UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(st, 1.0)
    root = mp.add_xform(st, "/root")
    st.SetDefaultPrim(root.GetPrim())
    UsdPhysics.RigidBodyAPI.Apply(root.GetPrim())
    mats = materials(st, look)
    mp.add_xform(st, "/root/visual")
    for name, (pts, faces, nrm) in parts.items():  # model first: loaders take the first mesh
        mesh = mp.add_mesh(st, f"/root/visual/{name}", pts, faces, nrm)
        UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(mats[MATERIAL_OF[name]])
    pc.add_collision(st, hulls)
    st.Save()
    zip_writer = getattr(Sdf, "ZipFileWriter", None) or Usd.ZipFileWriter
    zw = zip_writer.CreateNew(str(out))
    zw.AddFile(str(tmp), "object.usdc")
    zw.Save()
    st = Usd.Stage.Open(str(out))
    first = next(q for q in Usd.PrimRange(st.GetDefaultPrim()) if q.IsA(UsdGeom.Mesh))
    assert first.GetPath() == Sdf.Path("/root/visual/model"), first.GetPath()


def measurements(c):
    """spec dimensions_mm on the centred mesh: mouth outer diameter over the rim bead, bottom diameter (outer, at the
    folded bottom edge), height."""
    z = c["pts"][:, 2] - c["pts"][:, 2].min()
    r = np.hypot(c["pts"][:, 0], c["pts"][:, 1])
    model = c["parts"]["model"][0]
    zm, rm = model[:, 2] - c["pts"][:, 2].min(), np.hypot(model[:, 0], model[:, 1])
    out = {"开口外径": 2 * r[z > z.max() - c["prof"]["params"]["BEAD"]].max(), "底部直径": 2 * rm[zm < 0.002].max()}
    out["高度"] = z.max()
    return {k: round(float(v) * 1000, 1) for k, v in out.items()}


def description(uid, words, ml):
    caption = {k: words[k] for k in ("color", "material", "name", "shape", "function")}
    caption["size"] = [f"holds about {ml:.0f} ml", *words["size"]]
    name, color, mat = words["name"], words["color"][0], words["material"][0]
    desc = [
        words["look"],
        f"{color} {name[0]} that holds about {ml:.0f} ml",
        f"{mat} {name[1]} with a {words['shape'][0]} body",
        f"{name[2]} with a {words['shape'][1]} and a {words['shape'][-1]}",
        f"light {color} {name[0]} grasped around its body",
    ]
    return {"uuid": uid, "caption": {k: caption[k] for k in WORDS if k != "look"}, "description": desc}


def save(clutter_dir, idx, c, look, mass, words, extra):
    shift = -(c["pts"].min(0) + c["pts"].max(0)) / 2
    parts = {k: (q + shift, f, n) for k, (q, f, n) in c["parts"].items()}
    c = dict(c, parts=parts, pts=c["pts"] + shift, hulls=[h + shift for h in c["hulls"]])
    d = Path(clutter_dir) / "paper_cup" / f"{idx:05d}"
    d.mkdir(parents=True, exist_ok=True)
    uid = str(uuid.uuid5(uuid.NAMESPACE_URL, f"robodojo/clutter/paper_cup/{idx}"))  # Rigid/paper_cup uses rigid/
    write_usdz(d / "object.usdz", parts, c["hulls"], look)
    meta = pc.metadata(uid, c, mass, shift)
    meta |= {"measurements_mm": measurements(c)} if extra is None else extra
    (d / "metadata.json").write_text(json.dumps(meta, indent=4, ensure_ascii=False))
    (d / "description.json").write_text(json.dumps(description(uid, words, c["ml"]), indent=4))
    return d, meta


def main(clutter_dir):
    base = build_cup({})
    for idx, over in enumerate([{}] + VARIANTS):
        over = dict(over)
        look, words = over.pop("LOOK", LOOK), over.pop("WORDS", WORDS)
        c = base if idx == 0 else build_cup(over)
        if idx == 0:
            mass, extra = MASS, None
        else:
            mass, extra = round(MASS * c["area"] / base["area"], 4), {"variant_of": "Clutter/paper_cup/00000"}
        d, meta = save(clutter_dir, idx, c, look, mass, words, extra)
        print(
            f"{d}: {c['ml']:.0f} ml, mass {mass} kg, size {meta['physics']['size']}, {len(c['hulls'])} hulls "
            f"(max {max(len(ConvexHull(h).vertices) for h in c['hulls'])} pts), {meta.get('measurements_mm', '')}"
        )
    sys.stdout.flush()  # APP.close() exits without flushing a redirected stdout


if __name__ == "__main__":
    try:
        main(sys.argv[1] if len(sys.argv) > 1 else CLUTTER_DIR)
    except Exception:
        if mp.APP is None:
            raise
        traceback.print_exc()
        sys.stderr.flush()
        os._exit(1)
    if mp.APP is not None:
        mp.APP.close()
