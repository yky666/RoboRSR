#!/usr/bin/env python3
"""Frosted translucent plastic tumblers (plain rounded lip, no handle) from platform registrations, as `Rigid` assets.

- plastic_cup/00001: Real2Sim platform registration real2sim/objects/Rigid/plastic_cup/00001 (spec.json, by 种政,
  2026-10-28 11:05:02): translucent blue PP cup, 24 g, 开口外径 80.2 mm, 底部直径 54.8 mm, 高度 132.6 mm (measured).
  From the three photos: frosted (matte) sky-blue PP that lets light through, straight tapered wall, plain rounded
  lip, small rounded outer bottom edge, flat inner bottom with a step ring where it meets the wall. Wall thickness
  follows from the mass; lip, bottom and step sizes are estimated from the photos.
- plastic_cup/00002-00007: generalization variants of 00001 (not measured; colour, material, size and shape differ).
plastic_cup/00000 (the clear disposable cup of the G1D-PourBeansPlus clips) stays with make_water_cup.py --plastic.

Usage:
    python scripts/objects/make_plastic_cup.py [RIGID_DIR]    # default Assets/Object/Real2Sim/Rigid
Writes RIGID_DIR/plastic_cup/<index>/{object.usdz, metadata.json, description.json}. Needs what make_pitcher.py needs.

Frame: Z up, metres, origin at bbox centre.
USD:   /root (RigidBodyAPI) -> visual/model (one closed shell: outer wall, lip, inner wall, bottom)
                            -> collision/hull_NN (bottom slab + wall staves, hollow and open)
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
import make_metal_cup as mc  # noqa: E402  (wall_staves, description, shell_area)
import make_pitcher as mp  # noqa: E402  (shared USD helpers; starts Kit when pxr needs it)
from make_pitcher import Sdf, Usd, UsdGeom, UsdPhysics, UsdShade  # noqa: E402
import make_plastic_cups as pc  # noqa: E402  (loft, unit_section, add_collision, metadata)

RIGID_DIR = Path(__file__).resolve().parents[2] / "Assets/Object/Real2Sim/Rigid"
CATEGORY = "plastic_cup"
FIRST = 1  # 00000 belongs to make_water_cup.py --plastic
N_THETA = pc.N_THETA
T_COLL = 0.002  # collider wall, grown inward from the outer surface
LEVEL_TOL = 0.0005  # max deviation of the collider from the outer profile between two collision levels
SLAB_SIDES = (24, 24, 16)  # bottom slab rings (base, fillet top, inner bottom): 64 hull points, PhysX GPU limit

BLUE_PP = dict(
    H=0.1326,  # 高度, measured: standing surface to the top of the lip
    R_TOP=0.0401,  # 开口外径 / 2, measured 80.2 mm: outside of the lip
    R_BOT=0.0274,  # 底部直径 / 2, measured 54.8 mm: outer wall at the top of the bottom fillet
    SQ=2.0, BULGE=0.0,  # section superellipse exponent (2 = round), outward belly at mid-height (m)
    T_WALL=0.0009,  # estimated: 24 g of PP (900 kg/m^3) spread over the ~0.03 m^2 shell
    FIL=0.002, RECESS=0.0008, RING=0.003,  # outer bottom fillet, recess inside the standing ring, ring width: estimated
    TB=0.0025, STEP=0.0012, LEDGE=0.0015,  # inner bottom height, step ring height and width at the wall: estimated
    # frosted sky blue (photos): 20% mid-blue milky share, 80% transmission absorbed to `tint` per 1 mm of PP, so the
    # outside (seen through two walls) is deeper blue than the inside and the rim and edges, seen edge-on, darker
    LOOK=dict(kind="translucent", diffuse=(0.06, 0.38, 0.80), tint=(0.10, 0.45, 0.88), transmit=0.8),
    MASS=0.024,  # measured
    WORDS=dict(
        color=["translucent blue", "sky blue", "frosted blue"],
        material=["plastic", "polypropylene", "frosted PP"],
        name=["plastic cup", "drinking cup", "tumbler"],
        shape=["tapered cylinder", "rounded lip", "no handle"],
        function=["drinking water", "holding beans", "pouring into another cup"],
        look="frosted translucent sky blue plastic tumbler, wider at the rim",
    ),
)  # fmt: skip

VARIANTS = [  # generalization variants 00002-00007: overrides of BLUE_PP, all unmeasured (estimated)
    dict(
        LOOK=dict(kind="translucent", diffuse=(0.80, 0.81, 0.80), tint=(0.85, 0.86, 0.86), transmit=0.5),
        WORDS=dict(
            color=["frosted white", "milky white", "translucent white"],
            material=["plastic", "frosted PP", "polypropylene"],
            name=["plastic cup", "frosted cup", "tumbler"],
            shape=["tapered cylinder", "rounded lip", "no handle"],
            function=["drinking water", "holding beans", "pouring into another cup"],
            look="frosted milky white plastic tumbler",
        ),
    ),
    dict(
        H=0.110, R_TOP=0.0370, R_BOT=0.0260, LOOK=dict(kind="clear", rgb=(0.97, 0.98, 0.98)),
        WORDS=dict(
            color=["clear", "transparent", "colourless"],
            material=["plastic", "clear PET", "transparent plastic"],
            name=["clear cup", "drinking cup", "plastic cup"],
            shape=["tapered cylinder", "rounded lip", "no handle"],
            function=["drinking juice", "holding beans", "pouring into another cup"],
            look="small clear transparent plastic cup",
        ),
    ),
    dict(
        H=0.090, R_TOP=0.0400, R_BOT=0.0300, LOOK=dict(kind="opaque", rgb=(0.55, 0.03, 0.03), rough=0.4),
        WORDS=dict(
            color=["red", "opaque red", "bright red"],
            material=["plastic", "opaque PP", "polypropylene"],
            name=["red party cup", "plastic cup", "short cup"],
            shape=["short wide tapered body", "rounded lip", "no handle"],
            function=["holding drinks", "holding snacks", "being stacked"],
            look="short wide opaque red plastic cup",
        ),
    ),
    dict(
        H=0.120, R_TOP=0.0360, R_BOT=0.0260, SQ=4.0,
        LOOK=dict(kind="translucent", diffuse=(0.10, 0.55, 0.18), tint=(0.18, 0.62, 0.22), transmit=0.8),
        WORDS=dict(
            color=["translucent green", "lime green", "frosted green"],
            material=["plastic", "polypropylene", "frosted PP"],
            name=["square plastic cup", "green cup", "tumbler"],
            shape=["rounded square section", "tapered body", "no handle"],
            function=["drinking water", "holding beans", "pouring into another cup"],
            look="frosted translucent green plastic cup with a rounded square section",
        ),
    ),
    dict(
        H=0.125, R_TOP=0.0380, R_BOT=0.0270, BULGE=0.004,
        LOOK=dict(kind="translucent", diffuse=(0.85, 0.42, 0.06), tint=(0.90, 0.50, 0.10), transmit=0.8),
        WORDS=dict(
            color=["translucent amber", "orange", "frosted amber"],
            material=["plastic", "polypropylene", "frosted PP"],
            name=["bellied plastic cup", "amber cup", "tumbler"],
            shape=["bellied curved body", "rounded lip", "no handle"],
            function=["drinking tea", "holding beans", "pouring into another cup"],
            look="frosted translucent amber plastic cup with a bellied body",
        ),
    ),
    dict(
        H=0.150, R_TOP=0.0340, R_BOT=0.0250,
        LOOK=dict(kind="translucent", diffuse=(0.38, 0.12, 0.65), tint=(0.45, 0.18, 0.75), transmit=0.8),
        WORDS=dict(
            color=["translucent purple", "violet", "frosted purple"],
            material=["plastic", "polypropylene", "frosted PP"],
            name=["tall plastic cup", "purple cup", "tumbler"],
            shape=["tall slim tapered body", "rounded lip", "no handle"],
            function=["drinking juice", "holding beans", "pouring into another cup"],
            look="tall slim frosted translucent purple plastic cup",
        ),
    ),
]  # fmt: skip


class Cup:
    """Profile of one tumbler: straight (or bellied) outer wall r(z), rounded lip, recessed bottom with a standing
    ring outside, flat inner bottom with a step ring at the wall."""

    def __init__(self, g):
        self.g = g
        self.sec = pc.unit_section(g["SQ"])
        self.zc = g["H"] - g["T_WALL"] / 2  # lip centre height
        self.z_step = g["TB"] + g["STEP"]  # where the inner wall meets the step ring
        self.r_disc = self.wall(self.z_step) - g["T_WALL"] - g["LEDGE"]  # flat inner bottom
        self.r_foot = g["R_BOT"] - g["FIL"] - g["RING"]  # inside edge of the standing ring
        assert self.r_disc > self.r_foot > 0, "wall too steep for the bottom"
        assert g["TB"] > g["FIL"] > g["RECESS"], "bottom thinner than its fillet"

    def ring(self, r):
        return r * self.sec

    def wall(self, z):
        """Outer wall radius from the fillet top (z = FIL) to the lip centre (z = zc)."""
        g = self.g
        t = (z - g["FIL"]) / (self.zc - g["FIL"])
        return g["R_BOT"] + (g["R_TOP"] - g["R_BOT"]) * t + g["BULGE"] * np.sin(np.pi * t)

    def profile(self):
        """One closed section [(r, z)]: bottom pole -> outer wall -> lip -> inner wall -> inner bottom pole.
        A repeated point is a sharp edge."""
        g = self.g
        FIL, T, REC = g["FIL"], g["T_WALL"], g["RECESS"]
        out = [(0.0, REC), (self.r_foot, REC), (self.r_foot, REC), (self.r_foot + 0.0005, 0.0)]
        out += [(self.r_foot + 0.0005, 0.0)] + pc.fillet(g["R_BOT"], FIL)
        out += [(self.wall(z), z) for z in np.linspace(FIL, self.zc, 60)[1:]]
        out += mp.arc(g["R_TOP"] - T / 2, self.zc, T / 2, 0, np.pi, 9)[1:]  # plain rounded lip
        out += [(self.wall(z) - T, z) for z in np.linspace(self.zc, self.z_step, 60)[1:]]
        out += [out[-1], (self.r_disc, self.z_step), (self.r_disc, self.z_step), (self.r_disc, g["TB"])]
        out += [(self.r_disc, g["TB"]), (0.0, g["TB"])]
        return out

    def hulls(self):
        g = self.g
        slab = []
        for sides, r, z in zip(SLAB_SIDES, (g["R_BOT"] - g["FIL"], g["R_BOT"], self.r_disc), (0.0, g["FIL"], g["TB"])):
            slab += [(*p, z) for p in self.ring(r)[:: N_THETA // sides]]
        hulls = [np.array(slab)]
        zs = np.unique(np.r_[np.arange(g["FIL"], self.zc, 0.001), self.zc])
        rings = [self.ring(self.wall(z)) for z in zs]
        idx, i = [0], 0
        while i < len(zs) - 1:  # greedy levels: the collider follows a curved wall within LEVEL_TOL
            j = i + 1
            while j + 1 < len(zs):
                k = np.arange(i + 1, j + 1)
                w = ((zs[k] - zs[i]) / (zs[j + 1] - zs[i]))[:, None, None]
                lerp = (1 - w) * rings[i] + w * rings[j + 1]
                if np.max(np.linalg.norm(np.stack([rings[m] for m in k]) - lerp, axis=2)) > LEVEL_TOL:
                    break
                j += 1
            idx.append(j)
            i = j
        levels = [(z, self.ring(self.wall(z)), self.ring(self.wall(z) - T_COLL)) for z in zs[idx]]
        levels.append((g["H"], self.ring(g["R_TOP"]), self.ring(g["R_TOP"] - T_COLL)))  # covers the lip
        return hulls + mc.wall_staves(levels)

    def cavity(self, z):
        return self.ring(self.wall(min(z, self.zc)) - T_COLL)

    def brim_ml(self):
        z = np.linspace(self.g["TB"], self.zc, 600)
        a = np.array([pc.section_area(self.ring(self.wall(v) - self.g["T_WALL"])) for v in z])
        return float(np.sum((a[1:] + a[:-1]) / 2 * np.diff(z)) * 1e6)


def build(g):
    c = Cup(g)
    model = pc.loft([(0.0, z) if r == 0 else (c.ring(r), z) for r, z in c.profile()])
    assert mp.signed_volume(*model) > 0, "shell has inward-facing winding"
    hulls = c.hulls()
    probe = np.array(
        [
            (*(f * p), z)
            for z in np.linspace(g["TB"] + 0.001, g["H"] - 0.001, 40)
            for f in np.linspace(0, 0.94, 6)
            for p in c.cavity(z)[:: N_THETA // 48]
        ]
    )
    for h in hulls:
        assert len(h) <= 64, f"collision hull has {len(h)} points"
        eq = ConvexHull(h).equations
        assert not np.any(np.all(probe @ eq[:, :3].T + eq[:, 3] < -1e-6, axis=1)), "hull intrudes into cavity"
    bottom = hulls[0][:, 2]
    assert np.all((bottom == 0) | (bottom >= 0.0015)), "slab points within PhysX plane tolerance of the bottom"
    return dict(parts={"model": model}, hulls=hulls, ml=c.brim_ml(), contact_r=g["R_BOT"] - g["FIL"])


def material(look):
    """Frosted translucent PP: OmniSurface, diffuse weight 1 (see make_plastic_cups.clear_pp) and a specular
    transmission share `transmit`, solid (not thin-walled) with volume absorption reaching `tint` after `depth`. An
    interface tint instead (no depth) colours the inside as deeply as the outside and the cup reads opaque. RoboDojo
    renders real-time RTX without roughness sampling, so transmission stays sharp; the milky share does the frosting.
    clear: OmniGlass; opaque: OmniPBR."""

    def make(st):
        path = "/root/_materials/Plastic"
        if look["kind"] == "clear":
            rgb = look["rgb"]
            inputs = {"glass_color": rgb, "glass_ior": 1.49, "frosting_roughness": 0.0, "thin_walled": False}
            return mp.add_material(st, path, "OmniGlass", inputs, rgb, 0.25, 0.05)
        if look["kind"] == "opaque":
            rgb = look["rgb"]
            inputs = {"diffuse_color_constant": rgb, "reflection_roughness_constant": look["rough"]}
            inputs["metallic_constant"] = 0.0
            return mp.add_material(st, path, "OmniPBR", inputs, rgb, 1.0, look["rough"])
        rough = look.get("rough", 0.3)  # <= 0.35: rougher switches OmniSurface to its darkening transmission
        inputs = {
            "diffuse_reflection_weight": 1.0,
            "diffuse_reflection_color": look["diffuse"],
            "enable_specular_transmission": True,
            "specular_transmission_weight": look["transmit"],
            "specular_transmission_color": look["tint"],
            "specular_transmission_scattering_depth": look.get("depth", 0.001),  # ~ one wall thickness
            "specular_reflection_weight": look.get("sheen", 0.2),
            "specular_reflection_roughness": rough,
            "specular_reflection_ior": 1.49,
            "thin_walled": False,
        }
        return mp.add_material(st, path, "OmniSurface", inputs, look["diffuse"], 1.0 - look["transmit"] / 2, rough)

    return make


def write_usdz(out, c, g):
    tmp = Path(tempfile.mkdtemp()) / "object.usdc"
    st = Usd.Stage.CreateNew(str(tmp))
    UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(st, 1.0)
    root = mp.add_xform(st, "/root")
    st.SetDefaultPrim(root.GetPrim())
    UsdPhysics.RigidBodyAPI.Apply(root.GetPrim())
    mat = material(g["LOOK"])(st)
    mp.add_xform(st, "/root/visual")
    for name, (pts, faces) in c["parts"].items():  # model first: loaders read the first Mesh
        mesh = mp.add_mesh(st, f"/root/visual/{name}", pts, faces, mp.vertex_normals(pts, faces))
        UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(mat)
    pc.add_collision(st, c["hulls"])
    st.Save()
    zip_writer = getattr(Sdf, "ZipFileWriter", None) or Usd.ZipFileWriter
    zw = zip_writer.CreateNew(str(out))
    zw.AddFile(str(tmp), "object.usdc")
    zw.Save()


def save(rigid_dir, idx, c, g, mass):
    every = np.concatenate([p for p, _ in c["parts"].values()])
    shift = -(every.min(0) + every.max(0)) / 2
    parts = {k: (p + shift, f) for k, (p, f) in c["parts"].items()}
    c = dict(c, parts=parts, hulls=[h + shift for h in c["hulls"]])
    d = Path(rigid_dir) / CATEGORY / f"{idx:05d}"
    d.mkdir(parents=True, exist_ok=True)
    uid = str(uuid.uuid5(uuid.NAMESPACE_URL, f"robodojo/rigid/{CATEGORY}/{idx}"))
    write_usdz(d / "object.usdz", c, g)
    faces = [f for _, fs in parts.values() for f in fs]
    meta = pc.metadata(uid, dict(pts=every + shift, faces=faces, contact_r=c["contact_r"]), mass, shift)
    meta["physics"].pop("friction", None)  # no measured friction: runtime defaults apply
    if idx == FIRST:
        pts = parts["model"][0]
        lo = pts[:, 2].min()
        foot = pts[np.abs(pts[:, 2] - (lo + g["FIL"])) < 1e-6]  # outer wall at the top of the bottom fillet
        meta["measurements_mm"] = {
            "开口外径": round(float(np.ptp(pts[:, 0])) * 1000, 1),  # outside of the lip
            "底部直径": round(float(np.ptp(foot[:, 0])) * 1000, 1),
            "高度": round(float(np.ptp(pts[:, 2])) * 1000, 1),
        }
    else:
        meta["variant_of"] = f"Rigid/{CATEGORY}/{FIRST:05d}"
    (d / "metadata.json").write_text(json.dumps(meta, indent=4, ensure_ascii=False))
    (d / "description.json").write_text(json.dumps(mc.description(uid, g, c["ml"]), indent=4))
    return d, meta


def main(rigid_dir):
    base_area = None
    for k, over in enumerate([{}] + VARIANTS):
        g = {**BLUE_PP, **over}
        c = build(g)
        area = mc.shell_area(c["parts"])
        if k == 0:
            base_area, mass = area, g["MASS"]
        else:  # estimated: same wall per area as 00001
            mass = round(BLUE_PP["MASS"] * area / base_area, 4)
        d, meta = save(rigid_dir, FIRST + k, c, g, mass)
        print(
            f"{d}: {c['ml']:.0f} ml, mass {mass} kg, size {meta['physics']['size']}, "
            f"{len(c['hulls'])} collision hulls, measurements_mm {meta.get('measurements_mm', '-')}",
            flush=True,
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
