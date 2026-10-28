#!/usr/bin/env python3
"""Thin-walled plastic cups (rolled rim, no handle) from platform registrations, as RoboDojo `Rigid` assets.

- plastic_cup/00001: Real2Sim platform registration real2sim/objects/Rigid/plastic_cup/00001 (spec.json, by 种政,
  2026-10-28 11:05:02): translucent blue PP cup, 24 g, 开口外径 80.2 mm, 底部直径 54.8 mm, 高度 132.6 mm (measured).
  Wall and rim bead thickness and the bottom recess are estimated from the three photos.
- plastic_cup/00002-00007: generalization variants of 00001 (not measured; colour, material, size and shape differ).
plastic_cup/00000 (the clear disposable cup of the G1D-PourBeansPlus clips) stays with make_water_cup.py --plastic.

Usage:
    python scripts/objects/make_plastic_cup.py [RIGID_DIR]    # default Assets/Object/Real2Sim/Rigid
Writes RIGID_DIR/plastic_cup/<index>/{object.usdz, metadata.json, description.json}. Needs what make_pitcher.py needs.

Frame: Z up, metres, origin at bbox centre.
USD:   /root (RigidBodyAPI) -> visual/model (outside wall and bottom), visual/inside (rim, inner wall and bottom)
                            -> collision/hull_NN (bottom slab + wall staves, hollow and open)
Geometry is make_metal_cup.Cup (tapered wall, rolled rim, recessed bottom); only the materials differ.
"""

import json
import os
from pathlib import Path
import sys
import tempfile
import traceback
import uuid

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import make_metal_cup as mc  # noqa: E402  (tumbler geometry, collision and self-checks)
import make_pitcher as mp  # noqa: E402  (shared USD helpers; starts Kit when pxr needs it)
from make_pitcher import Sdf, Usd, UsdGeom, UsdPhysics, UsdShade  # noqa: E402
import make_plastic_cups as pc  # noqa: E402  (add_collision, metadata)

RIGID_DIR = Path(__file__).resolve().parents[2] / "Assets/Object/Real2Sim/Rigid"
CATEGORY = "plastic_cup"
FIRST = 1  # 00000 belongs to make_water_cup.py --plastic

BLUE_PP = dict(H=0.1326, R_TOP=0.0401, R_BOT=0.0274)  # 高度, 开口外径 / 2 (rim bead), 底部直径 / 2: measured
BLUE_PP.update(
    SQ=2.0, BULGE=0.0,
    T_WALL=0.0007, RB=0.001,  # wall, rolled-rim bead radius: estimated
    FIL=0.003, RECESS=0.0015, RING=0.003, TB=0.0025, DOME=0.0003, DISC=0.5,  # bottom: estimated
    LOOK=dict(kind="translucent", rgb=(0.04, 0.28, 0.80), milky=0.35, rough=0.3),
    MASS=0.024,  # measured
    WORDS=dict(
        color=["translucent blue", "light blue", "frosted blue"],
        material=["plastic", "polypropylene", "PP"],
        name=["plastic cup", "drinking cup", "tumbler"],
        shape=["tapered cylinder", "rolled rim", "no handle"],
        function=["drinking water", "holding beans", "pouring into another cup"],
        look="translucent blue plastic cup, wider at the rim",
    ),
)  # fmt: skip

VARIANTS = [  # generalization variants 00002-00007: overrides of BLUE_PP, all unmeasured (estimated)
    dict(
        LOOK=dict(kind="translucent", rgb=(0.85, 0.86, 0.86), milky=0.6, rough=0.3),
        WORDS=dict(
            color=["frosted white", "milky white", "translucent white"],
            material=["plastic", "frosted PP", "polypropylene"],
            name=["plastic cup", "frosted cup", "tumbler"],
            shape=["tapered cylinder", "rolled rim", "no handle"],
            function=["drinking water", "holding beans", "pouring into another cup"],
            look="frosted milky white plastic cup",
        ),
    ),
    dict(
        H=0.110, R_TOP=0.0370, R_BOT=0.0260, LOOK=dict(kind="clear", rgb=(0.97, 0.98, 0.98)),
        WORDS=dict(
            color=["clear", "transparent", "colourless"],
            material=["plastic", "clear PET", "transparent plastic"],
            name=["clear cup", "disposable cup", "plastic cup"],
            shape=["tapered cylinder", "rolled rim", "no handle"],
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
            shape=["short wide tapered body", "rolled rim", "no handle"],
            function=["holding drinks", "holding snacks", "being stacked"],
            look="short wide opaque red plastic cup",
        ),
    ),
    dict(
        H=0.120, R_TOP=0.0360, R_BOT=0.0260, SQ=4.0,
        LOOK=dict(kind="translucent", rgb=(0.10, 0.55, 0.20), milky=0.35, rough=0.3),
        WORDS=dict(
            color=["translucent green", "lime green", "frosted green"],
            material=["plastic", "polypropylene", "PP"],
            name=["square plastic cup", "green cup", "tumbler"],
            shape=["rounded square section", "tapered body", "no handle"],
            function=["drinking water", "holding beans", "pouring into another cup"],
            look="translucent green plastic cup with a rounded square section",
        ),
    ),
    dict(
        H=0.125, R_TOP=0.0380, R_BOT=0.0270, BULGE=0.004,
        LOOK=dict(kind="translucent", rgb=(0.85, 0.45, 0.08), milky=0.3, rough=0.3),
        WORDS=dict(
            color=["translucent amber", "orange", "frosted amber"],
            material=["plastic", "polypropylene", "PP"],
            name=["bellied plastic cup", "amber cup", "tumbler"],
            shape=["bellied curved body", "rolled rim", "no handle"],
            function=["drinking tea", "holding beans", "pouring into another cup"],
            look="translucent amber plastic cup with a bellied body",
        ),
    ),
    dict(
        H=0.150, R_TOP=0.0340, R_BOT=0.0250,
        LOOK=dict(kind="translucent", rgb=(0.40, 0.12, 0.65), milky=0.35, rough=0.3),
        WORDS=dict(
            color=["translucent purple", "violet", "frosted purple"],
            material=["plastic", "polypropylene", "PP"],
            name=["tall plastic cup", "purple cup", "tumbler"],
            shape=["tall slim tapered body", "rolled rim", "no handle"],
            function=["drinking juice", "holding beans", "pouring into another cup"],
            look="tall slim translucent purple plastic cup",
        ),
    ),
]  # fmt: skip


def material(look):
    """Translucent PP: OmniSurface with a white-diffuse milky share as make_plastic_cups.clear_pp, tinted by the
    transmission colour; clear: OmniGlass; opaque: OmniPBR."""
    rgb = look["rgb"]

    def make(st):
        path = "/root/_materials/Plastic"
        if look["kind"] == "clear":
            inputs = {"glass_color": rgb, "glass_ior": 1.49, "frosting_roughness": 0.0, "thin_walled": False}
            return mp.add_material(st, path, "OmniGlass", inputs, rgb, 0.25, 0.05)
        if look["kind"] == "opaque":
            inputs = {"diffuse_color_constant": rgb, "reflection_roughness_constant": look["rough"]}
            inputs["metallic_constant"] = 0.0
            return mp.add_material(st, path, "OmniPBR", inputs, rgb, 1.0, look["rough"])
        inputs = {
            "diffuse_reflection_weight": 1.0,
            "diffuse_reflection_color": rgb,
            "enable_specular_transmission": True,
            "specular_transmission_weight": 1.0 - look["milky"],
            "specular_transmission_color": rgb,
            "specular_reflection_weight": 0.1,
            "specular_reflection_roughness": look["rough"],
            "specular_reflection_ior": 1.49,
            "thin_walled": False,
        }
        return mp.add_material(st, path, "OmniSurface", inputs, rgb, 0.5, look["rough"])

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
    del meta["physics"]["friction"]  # no measured friction: runtime defaults apply
    if idx == FIRST:
        pts = every + shift
        outside = parts["model"][0]
        foot = outside[np.abs(outside[:, 2] - (pts[:, 2].min() + g["FIL"])) < 1e-6]  # top of the bottom fillet
        meta["measurements_mm"] = {
            "开口外径": round(float(np.ptp(pts[:, 0])) * 1000, 1),  # outside of the rim bead
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
        c = mc.build(g)
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
