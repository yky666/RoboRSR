#!/usr/bin/env python3
"""Procedural plastic cups from the 2026-10-17 desk photos as RoboDojo `Rigid` assets.

- plastic_mug/00000: white PP mug, straight round body (measured), flat D-shaped strap handle on +X.
- plastic_mug/00001-00005: generalization variants of 00000 (not measured; colour, material, size and shape differ):
  kids cup, tall tapered cup, ceramic coffee mug, smoky translucent cup, rounded-square cup (MUG_VARIANTS).
- measuring_cup/00000, 00001: milky translucent PP measuring cups (400 ml / 500 ml graduations), 14 / 17 cm tall:
  chamfered-square bottom widening to a rounded-square mouth, snap-lid band with a flange and a small hanging tab.

Mug and measuring-cup dimensions and masses are measured (measuring cups: measuring_cup_shape.py).

Usage:
    python scripts/objects/make_plastic_cups.py [RIGID_DIR]    # default Assets/Object/Real2Sim/Rigid
    python scripts/objects/check_object.py Assets/Object/Real2Sim/Rigid/{plastic_mug/0000{0..5},measuring_cup/0000{0,1}}
Writes RIGID_DIR/<category>/<index>/{object.usdz, metadata.json, description.json}. Needs what make_pitcher.py needs.

Frame: Z up, metres, origin at bbox centre.
USD:   /root (RigidBodyAPI) -> visual/model (whole cup, one material)
                            -> collision/hull_NN (bottom slab + wall staves + collar/handle pieces, hollow inside)
Profiles loft rings (N_THETA, 2), or scalars times a unit section, along Z.
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
import measuring_cup_shape as shape  # noqa: E402
import plastic_mug_shape  # noqa: E402
from plastic_mug_shape import MUG  # noqa: E402

RIGID_DIR = Path(__file__).resolve().parents[2] / "Assets/Object/Real2Sim/Rigid"
N_THETA = shape.N_THETA
STAVE_TOL = 0.0003  # max collision-wall intrusion into the cavity
BOTTOM_SIDES = 24  # bottom slab polygon: PhysX GPU convexes (particle contacts) take <= 64 vertices, <= 32 per face

# plastic mug: dimensions in plastic_mug_shape.py (measured)
WHITE_PP = (0.90, 0.89, 0.85)
# generalization variants 00001-00005: overrides of MUG (SQ: section superellipse exponent, 2 = round), all estimated;
# mass = DENSITY (kg/m^3) x mesh volume; LOOK picks the material: (rgb, roughness) opaque, or "clear" with a tint
PP, CERAMIC = 900.0, 2300.0
MUG_VARIANTS = [
    dict(
        H=0.080, R0=0.036, R1=0.036, HANDLE_TOP=0.064, HANDLE_BOT=0.016, HANDLE_OUT=0.028, BEND=0.012, HANDLE_W=0.011,
        DENSITY=PP, LOOK=((0.10, 0.36, 0.78), 0.45),
        WORDS=dict(
            color=["sky blue", "light blue", "pastel blue"], material=["plastic", "polypropylene", "matte plastic"],
            size=["small", "about 8 cm tall", "child-sized"], name=["kids cup", "small plastic mug", "children's mug"],
            shape=["short round body", "small rounded handle", "open top"],
            function=["drinking milk", "rinsing after brushing teeth", "holding beans"],
            look="small sky blue plastic cup for children with a rounded handle",
        ),
    ),
    dict(
        H=0.120, R0=0.035, R1=0.042, HANDLE_TOP=0.102, HANDLE_BOT=0.032, HANDLE_OUT=0.030, BEND=0.006,
        DENSITY=PP, LOOK=((0.80, 0.10, 0.18), 0.30),
        WORDS=dict(
            color=["coral pink", "rose red", "glossy pink"], material=["plastic", "polypropylene", "glossy plastic"],
            size=["tall", "about 12 cm tall", "large capacity"],
            name=["tall mug", "plastic tumbler with handle", "pink mug"],
            shape=["tapered body widening to the rim", "square-cornered handle", "open top"],
            function=["drinking juice", "carrying a drink", "holding beans"],
            look="tall pink plastic mug that widens towards the rim",
        ),
    ),
    dict(
        H=0.095, R0=0.041, R1=0.041, T_WALL=0.004, T_BOTTOM=0.006, FILLET=0.005, HANDLE_TOP=0.078, HANDLE_BOT=0.022,
        HANDLE_OUT=0.030, BEND=0.020, HANDLE_W=0.010, HANDLE_T=0.008, DENSITY=CERAMIC,
        LOOK=((0.010, 0.025, 0.10), 0.12),
        WORDS=dict(
            color=["navy blue", "dark blue", "glazed blue"], material=["ceramic", "glazed stoneware", "porcelain"],
            size=["standard mug size", "about 9.5 cm tall", "fits in one hand"],
            name=["coffee mug", "ceramic mug", "navy mug"],
            shape=["straight thick-walled body", "rounded loop handle", "open top"],
            function=["drinking coffee", "drinking tea", "holding beans"],
            look="glossy navy blue ceramic coffee mug with a thick wall and a rounded handle",
        ),
    ),
    dict(
        H=0.105, R0=0.040, R1=0.040, HANDLE_TOP=0.088, HANDLE_BOT=0.024, HANDLE_OUT=0.032, BEND=0.014,
        DENSITY=PP, LOOK=("clear", (0.45, 0.33, 0.22)),
        WORDS=dict(
            color=["smoky brown", "translucent", "tinted"], material=["plastic", "translucent polypropylene", "PP"],
            size=["about 10 cm tall", "standard mug size", "fits in one hand"],
            name=["translucent mug", "smoky plastic cup", "clear mug"],
            shape=["straight round body", "D-shaped handle", "open top"],
            function=["drinking water", "showing what is inside", "holding beans"],
            look="smoky translucent plastic mug through which the contents show",
        ),
    ),
    dict(
        SQ=4.0, H=0.078, R0=0.040, R1=0.040, FILLET=0.004, HANDLE_TOP=0.062, HANDLE_BOT=0.018, HANDLE_OUT=0.030,
        BEND=0.010, HANDLE_W=0.016, DENSITY=PP, LOOK=((0.22, 0.62, 0.42), 0.50),
        WORDS=dict(
            color=["mint green", "pale green", "matte green"], material=["plastic", "polypropylene", "matte plastic"],
            size=["short and wide", "about 8 cm tall", "fits in one hand"],
            name=["square mug", "plastic cup", "mint mug"],
            shape=["rounded-square body", "wide flat handle", "open top"],
            function=["drinking water", "holding pens", "holding beans"],
            look="short mint green plastic mug with a rounded-square body",
        ),
    ),
]  # fmt: skip
MUG_WORDS = dict(
    color=["white", "off-white", "cream white"], material=["plastic", "polypropylene", "PP"],
    size=["about 10 cm tall", "standard mug size", "fits in one hand"], name=["plastic mug", "rinse cup", "white mug"],
    shape=["straight cylindrical body", "flat D-shaped handle", "open top"],
    function=["drinking water", "rinsing after brushing teeth", "holding beans"],
    look="plain off-white polypropylene mug with a flat D-shaped handle",
)  # fmt: skip

CLEAR_PP = (0.94, 0.95, 0.96)
# milky share of the wall (rest transmits), specular reflection weight, surface roughness (also blurs transmission;
# above 0.35 OmniSurface switches to its strong-roughness transmission and the wall darkens)
# milky 0.25: whitish like the photos in the bright counter scene, beans still visible through the wall (beans greyer
# in check_cup_beans.py's dim grey-floor scene)
CLEAR_MIX = dict(milky=0.25, reflection=0.1, roughness=0.3)


def unit_section(sq):
    """Superellipse of half-width 1 (sq=2: circle), N_THETA points uniform by arc length, CCW from +X."""
    t = np.linspace(0, 2 * np.pi, 8192, endpoint=False)
    rho = (np.abs(np.cos(t)) ** sq + np.abs(np.sin(t)) ** sq) ** (-1 / sq)
    closed = np.vstack([np.c_[rho * np.cos(t), rho * np.sin(t)], [[1.0, 0.0]]])
    s = np.r_[0, np.cumsum(np.linalg.norm(np.diff(closed, axis=0), axis=1))]
    u = np.linspace(0, s[-1], N_THETA, endpoint=False)
    return np.c_[np.interp(u, s, closed[:, 0]), np.interp(u, s, closed[:, 1])]


def loft(profile, sec=None):
    """Closed surface through sections [(r, z)] stacked along Z: r is a ring (N_THETA, 2) or a scale of `sec`;
    r == 0 is a pole. Repeat a point for a sharp edge."""
    pts, rings = [], []
    for r, z in profile:
        if np.isscalar(r) and r == 0:
            rings.append([len(pts)] * N_THETA)
            pts.append((0.0, 0.0, z))
        else:
            rings.append(list(range(len(pts), len(pts) + N_THETA)))
            pts += [(x, y, z) for x, y in (r * sec if np.isscalar(r) else r)]
    faces = []
    for (p, q), a, b in zip(zip(profile, profile[1:]), rings, rings[1:]):
        if p[1] == q[1] and np.array_equal(p[0], q[0]):
            continue
        for j in range(N_THETA):
            k = (j + 1) % N_THETA
            quad = [a[j], a[k], b[k], b[j]]
            faces.append([v for n, v in enumerate(quad) if v not in quad[:n]])
    return np.array(pts), faces


def chord_sag(ring, a, b):
    p = ring[np.arange(a, b + 1) % N_THETA]
    d, v = p[-1] - p[0], p - p[0]
    return np.max(np.abs(d[0] * v[:, 1] - d[1] * v[:, 0])) / np.linalg.norm(d)


def wall_staves(levels):
    """Convex wall pieces between consecutive levels [(z, outer ring, inner ring)]. Inner face uses only the stave's
    end columns so it stays a chord, at most STAVE_TOL inside the inner wall."""
    hulls = []
    for (z0, o0, i0), (z1, o1, i1) in zip(levels, levels[1:]):
        cuts = [0]
        while cuts[-1] < N_THETA:
            a, b = cuts[-1], cuts[-1] + 1
            while b < N_THETA and max(chord_sag(r, a, b + 1) for r in (i0, i1)) <= STAVE_TOL:
                b += 1
            cuts.append(b)
        for a, b in zip(cuts, cuts[1:]):
            idx = np.arange(a, b + 1) % N_THETA
            ring = [(*p, z) for r, z in ((o0, z0), (o1, z1)) for p in r[idx]]
            hulls.append(np.array(ring + [(*p, z) for r, z in ((i0, z0), (i1, z1)) for p in r[idx[[0, -1]]]]))
    return hulls


def fillet(r, f, n=6):
    """Outer bottom edge: (r - f, 0) -> (r, f)."""
    return mp.arc(r - f, f, f, -np.pi / 2, 0, n)


def cavity_probe(inner, z0, z1):
    """Points filling the cavity; inner(z) is the inner wall ring."""
    return np.array(
        [
            (*(f * p), z)
            for z in np.linspace(z0 + 1e-3, z1 - 1e-3, 12)
            for f in np.linspace(0, 0.97, 6)
            for p in inner(z)[:: N_THETA // 48]
        ]
    )


def build_mug(g=MUG):
    sec = unit_section(g.get("SQ", 2.0))

    def outer(z):
        return g["R0"] + (g["R1"] - g["R0"]) * z / g["H"]

    def inner(z):
        return outer(z) - g["T_WALL"]

    H, TB = g["H"], g["T_BOTTOM"]
    profile = (
        [(0.0, 0.0)]
        + fillet(g["R0"], g["FILLET"])
        + [(g["R1"], H), (g["R1"], H), (inner(H), H), (inner(H), H), (inner(TB), TB), (inner(TB), TB), (0.0, TB)]
    )
    body = loft(profile, sec)

    zt, zb, b = g["HANDLE_TOP"], g["HANDLE_BOT"], g["BEND"]
    x_out = g["R1"] + g["HANDLE_OUT"]
    path = (
        [(outer(zt) - 0.001, zt)]
        + mp.arc(x_out - b, zt - b, b, np.pi / 2, 0)
        + mp.arc(x_out - b, zb + b, b, 0, -np.pi / 2)
        + [(outer(zb) - 0.001, zb)]
    )
    h_pts, h_faces, h_rings = mp.sweep(path, g["HANDLE_W"], g["HANDLE_T"])
    pts = np.concatenate([body[0], h_pts])
    faces = body[1] + [[i + len(body[0]) for i in f] for f in h_faces]
    normals = np.concatenate([mp.vertex_normals(*body), mp.vertex_normals(h_pts, h_faces)])

    hulls = [np.array([(*(g["R0"] * p), z) for p in sec[:: N_THETA // BOTTOM_SIDES] for z in (0.0, TB)])]
    hulls += wall_staves([(0.0, g["R0"] * sec, inner(0.0) * sec), (H, g["R1"] * sec, inner(H) * sec)])
    hulls += [np.concatenate([h_rings[i], h_rings[i + 1]]) for i in range(len(h_rings) - 1)]

    for part in (body, (h_pts, h_faces)):
        assert mp.signed_volume(*part) > 0, "part has inward-facing winding"
    ml = volume_ml(lambda z: inner(z) * sec, TB, H)
    return dict(
        pts=pts, faces=faces, normals=normals, hulls=hulls, probe=cavity_probe(lambda z: inner(z) * sec, TB, H), ml=ml,
        contact_r=g["R0"] - g["FILLET"], grip=np.array([x_out, 0.0, (zt + zb) / 2]), n_body=len(body[0]),
        volume=mp.signed_volume(*body) + mp.signed_volume(h_pts, h_faces),
    )  # fmt: skip


def build_measuring_cup(H):
    g, top = shape.CUP, shape.TOP
    zf, TB = shape.flange_z(H), g["T_BOTTOM"]
    zl = zf + g["LIP_T"]

    def inner(z):
        return shape.inner_ring(z, H)

    lip, collar = g["A_LIP"] * top, g["A_COLLAR"] * top
    assert g["A_COLLAR"] - inner(H)[0, 0] >= g["T_WALL"], "lid band thinner than the wall"
    profile = (
        [(0.0, 0.0)]
        + [(r * shape.BOTTOM, z) for r, z in fillet(g["A0"], g["FILLET"])]
        + [(shape.outer_ring(zf, H), zf)] * 2
        + [(lip, zf)] * 2
        + [(lip, zl)] * 2
        + [(collar, zl)] * 2
        + [(collar, H)] * 2
        + [(inner(H), H)] * 2
        + [(inner(TB), TB)] * 2
        + [(0.0, TB)]
    )
    body = loft(profile)
    a, w = g["A_LIP"], shape.TAB_W / 2
    tab = [(a - 0.002, -w), (a + shape.TAB_OUT, -0.75 * w), (a + shape.TAB_OUT, 0.75 * w), (a - 0.002, w)]
    t_pts, t_faces = mp.prism(tab, zf, zl)
    pts = np.concatenate([body[0], t_pts])
    faces = body[1] + [[i + len(body[0]) for i in f] for f in t_faces]
    normals = np.concatenate([mp.vertex_normals(*body), mp.vertex_normals(t_pts, t_faces)])

    bottom = g["A0"] * shape.BOTTOM[:: N_THETA // BOTTOM_SIDES]
    hulls = [np.array([(*p, z) for p in bottom for z in (0.0, TB)])]
    hulls += wall_staves([(0.0, g["A0"] * shape.BOTTOM, inner(0.0)), (zf, shape.outer_ring(zf, H), inner(zf))])
    hulls += wall_staves([(zf, lip, inner(zf)), (zl, lip, inner(zl))])
    hulls += wall_staves([(zl, collar, inner(zl)), (H, collar, inner(H))])
    hulls.append(np.array([(x, y, z) for x, y in tab for z in (zf, zl)]))

    for part in (body, (t_pts, t_faces)):
        assert mp.signed_volume(*part) > 0, "part has inward-facing winding"
    return dict(
        pts=pts, faces=faces, normals=normals, hulls=hulls, probe=cavity_probe(inner, TB, H),
        ml=volume_ml(inner, TB, H), contact_r=g["A0"] - g["FILLET"], inner=inner, TB=TB, H=H,
    )  # fmt: skip


def section_area(ring):
    x, y = ring.T
    return 0.5 * abs(np.dot(x, np.roll(y, 1)) - np.dot(y, np.roll(x, 1)))


def volume_ml(inner, z0, z1, n=400):
    z = np.linspace(z0, z1, n)
    a = np.array([section_area(inner(h)) for h in z])
    return float(np.sum((a[1:] + a[:-1]) / 2 * np.diff(z)) * 1e6)


def fill_height(c, ml):
    """Height above the outside bottom at which the cup holds `ml`."""
    z = np.linspace(c["TB"], c["H"], 2000)
    a = np.array([section_area(c["inner"](h)) for h in z])
    v = np.r_[0, np.cumsum((a[1:] + a[:-1]) / 2 * np.diff(z))] * 1e6
    return float(np.interp(ml, v, z))


def self_check(c):
    for h in c["hulls"]:
        eq = ConvexHull(h).equations
        assert not np.any(np.all(c["probe"] @ eq[:, :3].T + eq[:, 3] < -1e-6, axis=1)), "hull intrudes into cavity"


def write_usdz(out, c, material):
    tmp = Path(tempfile.mkdtemp()) / "object.usdc"
    st = Usd.Stage.CreateNew(str(tmp))
    UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(st, 1.0)
    root = mp.add_xform(st, "/root")
    st.SetDefaultPrim(root.GetPrim())
    UsdPhysics.RigidBodyAPI.Apply(root.GetPrim())
    mat = material(st)
    mp.add_xform(st, "/root/visual")
    mesh = mp.add_mesh(st, "/root/visual/model", c["pts"], c["faces"], c["normals"])
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(mat)
    add_collision(st, c["hulls"])
    st.Save()
    zip_writer = getattr(Sdf, "ZipFileWriter", None) or Usd.ZipFileWriter
    zw = zip_writer.CreateNew(str(out))
    zw.AddFile(str(tmp), "object.usdc")
    zw.Save()


def add_collision(st, hulls):
    """Invisible convex-hull colliders /root/collision/hull_NN."""
    mp.add_xform(st, "/root/collision")
    for i, h in enumerate(hulls):
        hull = ConvexHull(h)
        tri = hull.simplices.copy()
        n = np.cross(h[tri[:, 1]] - h[tri[:, 0]], h[tri[:, 2]] - h[tri[:, 0]])
        flip = np.einsum("ij,ij->i", n, hull.equations[:, :3]) < 0
        tri[flip] = tri[flip][:, ::-1]
        used, tri = np.unique(tri, return_inverse=True)
        m = mp.add_mesh(st, f"/root/collision/hull_{i:02d}", h[used], tri.reshape(-1, 3).tolist())
        prim = m.GetPrim()
        m.CreatePurposeAttr(UsdGeom.Tokens.guide)
        vis = UsdGeom.VisibilityAPI.Apply(prim)
        vis.CreateRenderVisibilityAttr(UsdGeom.Tokens.invisible)
        vis.CreateGuideVisibilityAttr(UsdGeom.Tokens.invisible)
        UsdPhysics.CollisionAPI.Apply(prim)
        UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr(UsdPhysics.Tokens.convexHull)


def white_pp(st, rgb=WHITE_PP, roughness=0.42):
    return mp.add_material(
        st,
        "/root/_materials/WhitePlastic",
        "OmniPBR",
        {"diffuse_color_constant": rgb, "reflection_roughness_constant": roughness, "metallic_constant": 0.0},
        rgb,
        1.0,
        roughness,
    )


def clear_pp(st, rgb=CLEAR_PP):
    """Hazy translucent PP: rough transmission layered over a white diffuse base. OmniSurface weights are layered
    (diffuse gets 1 - transmission of the energy, times its own weight), so the diffuse weight stays 1 and the milky
    share is set by the transmission weight; a diffuse weight below 1 loses energy and greys the wall. OmniGlass
    frosting only blurs and greys the refraction, it never turns the wall white like the photos."""
    return mp.add_material(
        st,
        "/root/_materials/ClearPlastic",
        "OmniSurface",
        {
            "diffuse_reflection_weight": 1.0,
            "diffuse_reflection_color": rgb,
            "enable_specular_transmission": True,
            "specular_transmission_color": rgb,
            "specular_transmission_weight": 1.0 - CLEAR_MIX["milky"],
            "specular_reflection_weight": CLEAR_MIX["reflection"],
            "specular_reflection_roughness": CLEAR_MIX["roughness"],
            "specular_reflection_ior": 1.49,
            "thin_walled": False,
        },
        rgb,
        0.5,
        CLEAR_MIX["roughness"],
    )


def metadata(uid, c, mass, shift):
    pts = c["pts"]
    lo, hi = pts.min(0), pts.max(0)
    ext = hi - lo
    corners = [mp.r4((x, y, z)) for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])]
    bbox = {"vertices": corners, "extents": mp.r4(ext)}
    functional = {}
    if "grip" in c:  # same grasp frames as mug/00015, whose handle is also a vertical bar on +X
        grip = c["grip"] + shift
        functional["handle"] = {"frame": [mp.r4((*grip, 0.5, 0.5, 0.5, 0.5)), mp.r4((*grip, 0.5, -0.5, 0.5, -0.5))]}
    contact = {"center": mp.r4((*shift[:2], lo[2], 1, 0, 0, 0)), "radius": round(c["contact_r"], 4)}
    return {
        "uuid": uid,
        "physics": {"mass": mass, "size": mp.r4(ext)},  # friction unmeasured: runtime defaults
        "geometry": {
            "faces": len(c["faces"]),
            "vertices": len(pts),
            "aligned_bbox": bbox,
            "oriented_bbox": bbox,
            "radius": round(float(np.linalg.norm(pts, axis=1).max()), 4),
            "bbox": mp.r4(ext),
            "center": mp.r4((lo + hi) / 2),
        },
        "active": {
            "place": {
                "up": {
                    "orientation": [1.0, 0.0, 0.0, 0.0],
                    "projection_circle": {
                        "center": mp.r4((0, 0, lo[2], 1, 0, 0, 0)),
                        "radius": round(float(max(ext[:2]) / 2), 4),
                    },
                    "contact_circle": contact,
                }
            },
            "functional": functional,
        },
        "passive": {"functional": {}, "support": {}},
    }


def mug_description(uid, ml, words):
    caption = {k: words[k] for k in ("color", "material", "size", "name", "shape", "function")}
    caption["size"] = [f"holds about {ml:.0f} ml", *caption["size"][:2]]
    name, color = words["name"][0], words["color"][0]
    desc = [
        words["look"],
        f"{color} {name} with a handle, holds about {ml:.0f} ml",
        f"{words['material'][0]} {name} grasped by its handle",
        f"{name} with a {words['shape'][0]}",
        f"{color} {words['material'][-1]} {name} that can hold beans",
    ]
    return {"uuid": uid, "caption": caption, "description": desc}


def measuring_description(uid, mark, ml, H):
    caption = {
        "color": ["translucent", "milky white", "frosted clear"],
        "material": ["plastic", "polypropylene", "PP"],
        "size": [f"{mark} ml graduation", f"holds about {ml:.0f} ml to the brim", f"about {H * 100:.1f} cm tall"],
        "name": ["measuring cup", "plastic measuring cup", "storage cup"],
        "shape": ["chamfered square bottom", "rounded square mouth", "collar around the mouth"],
        "function": ["measuring beans", "measuring water", "pouring into another cup"],
    }
    desc = [
        f"translucent rounded-square plastic measuring cup marked up to {mark} ml",
        "translucent square cup, chamfered at the bottom, with a snap-lid collar",
        f"tall milky polypropylene cup that holds about {ml:.0f} ml",
        "frosted measuring cup with a small hanging tab on the rim",
        "light plastic cup grasped around its square body",
    ]
    return {"uuid": uid, "caption": caption, "description": desc}


def save(rigid_dir, category, idx, c, material, mass, desc_fn, extra=None):
    shift = -(c["pts"].min(0) + c["pts"].max(0)) / 2
    c = dict(c, pts=c["pts"] + shift, hulls=[h + shift for h in c["hulls"]])
    d = Path(rigid_dir) / category / f"{idx:05d}"
    d.mkdir(parents=True, exist_ok=True)
    uid = str(uuid.uuid5(uuid.NAMESPACE_URL, f"robodojo/rigid/{category}/{idx}"))
    write_usdz(d / "object.usdz", c, material)
    meta = metadata(uid, c, mass, shift) | (extra(c) if extra else {})
    (d / "metadata.json").write_text(json.dumps(meta, indent=4, ensure_ascii=False))
    (d / "description.json").write_text(json.dumps(desc_fn(uid), indent=4))
    return d, meta


def mug_measurements(c):
    """spec dimensions_mm of 00000 measured on the mesh: length with the handle, body outer diameter, height."""
    body = c["pts"][: c["n_body"]]
    return {
        "measurements_mm": {
            "总长（含把手）": round(float(np.ptp(c["pts"][:, 0])) * 1000, 1),
            "外径": round(float(np.ptp(body[:, 1])) * 1000, 1),
            "高": round(float(np.ptp(c["pts"][:, 2])) * 1000, 1),
        }
    }


def mug_material(look):
    if look[0] == "clear":
        return lambda st: clear_pp(st, look[1])
    return lambda st: white_pp(st, *look)


def main(rigid_dir):
    for idx, over in enumerate([{}] + MUG_VARIANTS):
        g = {**MUG, **over}
        mug = build_mug(g)
        self_check(mug)
        if idx == 0:
            lo, hi = mug["pts"].min(0), mug["pts"].max(0)
            assert np.allclose(-(lo + hi) / 2, plastic_mug_shape.origin_shift(), atol=2e-4), "bbox != origin_shift()"
            mass, material, extra = MUG["MASS"], white_pp, mug_measurements
        else:
            mass = round(g["DENSITY"] * mug["volume"], 4)
            material, extra = mug_material(g["LOOK"]), lambda c: {"variant_of": "Rigid/plastic_mug/00000"}
        words = g.get("WORDS", MUG_WORDS)
        d, meta = save(
            rigid_dir, "plastic_mug", idx, mug, material, mass, lambda u: mug_description(u, mug["ml"], words), extra
        )
        print(
            f"{d}: {mug['ml']:.0f} ml, mass {mass} kg (PP-density estimate {PP * mug['volume'] * 1000:.0f} g), "
            f"size {meta['physics']['size']}, {len(mug['hulls'])} collision hulls, {meta.get('measurements_mm', '')}"
        )

    for idx, (H, mark, mass) in enumerate(shape.MEASURING):
        c = build_measuring_cup(H)
        self_check(c)
        lo, hi = c["pts"].min(0), c["pts"].max(0)
        assert np.allclose(-(lo + hi) / 2, shape.origin_shift(H), atol=1e-6), "bbox differs from origin_shift()"
        z = fill_height(c, mark)
        d, meta = save(
            rigid_dir, "measuring_cup", idx, c, clear_pp, mass, lambda u: measuring_description(u, mark, c["ml"], H)
        )
        print(
            f"{d}: {c['ml']:.0f} ml brim, {mark} ml at z={z * 1000:.0f} mm ({z / H:.0%} of height), "
            f"size {meta['physics']['size']}, {len(c['hulls'])} collision hulls"
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
