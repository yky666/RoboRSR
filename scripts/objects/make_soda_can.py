#!/usr/bin/env python3
"""Opened aluminium drink cans as RoboDojo `Rigid` assets.

- soda_can/00006: Real2Sim platform registration real2sim/objects/Rigid/soda_can/00006 (spec.json, by 种政,
  2026-10-28 10:48:05): Coke Zero sleek can, opened, 13 g, 直径 57.7 mm, 高度 146.5 mm (measured). Painted body (red
  with a black and yellow chequer band, text and logos not modelled), bare aluminium bottom, neck and lid. Bottom dome,
  neck, double seam, lid countersink, pour opening and the raised ring-pull are estimated from the five photos.
- soda_can/00007-00012: generalization variants of 00006 (not measured; paint, finish, size and shape differ).
soda_can/00000-00005 are imported meshes (scripts/objects/local_import/).

Usage:
    python scripts/objects/make_soda_can.py [RIGID_DIR]    # default Assets/Object/Real2Sim/Rigid
Writes RIGID_DIR/soda_can/<index>/{object.usdz, metadata.json, description.json}. Needs what make_pitcher.py needs.

Frame: Z up, metres, origin at bbox centre (the raised ring-pull reaches above the rim). Pour opening toward +Y, the
ring-pull stands up toward -Y.
USD:   /root (RigidBodyAPI) -> visual/model (painted body), visual/bottom, visual/top (bare aluminium), visual/inside
                               (dark inner wall, seen through the opening), visual/lid, visual/tab, visual/decal_NN
                            -> collision/hull_NN (bottom, wall staves, lid pieces around the opening, ring-pull)
The can is hollow: the lid is four convex plates leaving the pour opening free, colliders included.
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
import make_plastic_cups as pc  # noqa: E402  (loft, unit_section, add_collision, metadata)

RIGID_DIR = Path(__file__).resolve().parents[2] / "Assets/Object/Real2Sim/Rigid"
CATEGORY = "soda_can"
FIRST = 6  # 00000-00005 are the imported meshes
MM = 0.001
SEC = pc.unit_section(2.0)
T_VIS = 0.3 * MM  # visual wall
T_COLL = 1.5 * MM  # collider wall, grown inward
STAVE_TOL, LEVEL_TOL, MAX_COLS, CIRCLE = 0.0006, 0.0005, 28, 48
OPEN = dict(x=0.34, y0=0.34, y1=0.90)  # pour opening |x| < x, y0 < y < y1, as shares of the lid panel radius: estimated
TAB_TILT = np.radians(80)  # ring-pull raised to 80 deg above the lid: photos

ALU = dict(rgb=(0.82, 0.82, 0.84), rough=0.3, metal=1.0)
COKE_ZERO = dict(
    R=57.7 / 2 * MM, H=146.5 * MM,  # 直径 / 2, 高度 (bottom to the top of the seam): measured
    SEAM_R=26.3 * MM, NECK_H=11.0 * MM, DOME=9.5 * MM, BULGE=0.0,  # 202 lid seam, neck, bottom dome: estimated
    PAINT=dict(rgb=(0.55, 0.01, 0.015), rough=0.25, metal=0.2),
    DECALS=[  # (z0, z1 in m, colours, columns, rows): chequer band low on the body, estimated from the photos
        (14 * MM, 50 * MM, [(0.80, 0.60, 0.02), (0.01, 0.01, 0.01)], 12, 3),
        (50 * MM, 52 * MM, [(0.01, 0.01, 0.01)], 1, 1),
    ],
    MASS=0.013,  # measured
    WORDS=dict(
        color=["red", "black and yellow chequer band", "silver lid"],
        material=["aluminium", "painted aluminium", "metal"],
        name=["soda can", "Coke Zero can", "drink can"],
        shape=["tall slim cylinder", "opened ring-pull lid", "domed bottom"],
        function=["holding a soft drink", "pouring a drink", "being picked up and placed"],
        look="opened slim red Coke Zero can with a black and yellow chequer band",
    ),
)  # fmt: skip

VARIANTS = [  # generalization variants 00007-00012: overrides of COKE_ZERO, all unmeasured (estimated)
    dict(
        R=33.1 * MM,
        H=115.2 * MM,
        NECK_H=14.0 * MM,
        DOME=10.0 * MM,
        DECALS=[(46 * MM, 56 * MM, [(0.90, 0.90, 0.88)], 1, 1)],
        WORDS=dict(
            color=["red", "white band", "silver lid"],
            material=["aluminium", "painted aluminium", "metal"],
            name=["cola can", "330 ml can", "soda can"],
            shape=["standard cylinder", "opened ring-pull lid", "domed bottom"],
            function=["holding a soft drink", "pouring a drink", "being picked up and placed"],
            look="opened standard red cola can with a white band",
        ),
    ),
    dict(
        PAINT=dict(rgb=(0.02, 0.30, 0.08), rough=0.25, metal=0.2),
        DECALS=[(20 * MM, 32 * MM, [(0.85, 0.75, 0.05)], 1, 1), (100 * MM, 104 * MM, [(0.85, 0.85, 0.85)], 1, 1)],
        WORDS=dict(
            color=["green", "yellow band", "silver lid"],
            material=["aluminium", "painted aluminium", "metal"],
            name=["lemon soda can", "slim can", "soda can"],
            shape=["tall slim cylinder", "opened ring-pull lid", "domed bottom"],
            function=["holding a soft drink", "pouring a drink", "being picked up and placed"],
            look="opened slim green soda can with a yellow band",
        ),
    ),
    dict(
        R=33.1 * MM,
        H=168.0 * MM,
        NECK_H=14.0 * MM,
        DOME=10.0 * MM,
        PAINT=dict(rgb=(0.01, 0.05, 0.35), rough=0.25, metal=0.2),
        DECALS=[(60 * MM, 75 * MM, [(0.90, 0.90, 0.88)], 1, 1), (75 * MM, 80 * MM, [(0.60, 0.02, 0.03)], 1, 1)],
        WORDS=dict(
            color=["blue", "white and red bands", "silver lid"],
            material=["aluminium", "painted aluminium", "metal"],
            name=["tall soda can", "500 ml can", "drink can"],
            shape=["tall cylinder", "opened ring-pull lid", "domed bottom"],
            function=["holding a soft drink", "pouring a drink", "being picked up and placed"],
            look="opened tall blue 500 ml can with white and red bands",
        ),
    ),
    dict(
        R=26.5 * MM,
        H=134.0 * MM,
        SEAM_R=24.0 * MM,
        NECK_H=9.0 * MM,
        DOME=8.5 * MM,
        PAINT=dict(rgb=(0.02, 0.02, 0.022), rough=0.7, metal=0.1),
        DECALS=[(30 * MM, 33 * MM, [(0.70, 0.05, 0.05)], 1, 1)],
        WORDS=dict(
            color=["matte black", "red stripe", "silver lid"],
            material=["aluminium", "matte painted aluminium", "metal"],
            name=["slim black can", "250 ml can", "energy drink can"],
            shape=["slim cylinder", "opened ring-pull lid", "domed bottom"],
            function=["holding an energy drink", "pouring a drink", "being picked up and placed"],
            look="opened slim matte black can with a red stripe",
        ),
    ),
    dict(
        R=33.1 * MM,
        H=92.0 * MM,
        NECK_H=14.0 * MM,
        DOME=10.0 * MM,
        PAINT=dict(rgb=(0.80, 0.80, 0.82), rough=0.35, metal=1.0),
        DECALS=[],
        WORDS=dict(
            color=["bare silver", "brushed aluminium", "silver"],
            material=["aluminium", "brushed aluminium", "metal"],
            name=["short can", "stubby can", "plain aluminium can"],
            shape=["short wide cylinder", "opened ring-pull lid", "domed bottom"],
            function=["holding a drink", "pouring a drink", "being picked up and placed"],
            look="opened short plain brushed aluminium can",
        ),
    ),
    dict(
        BULGE=2.0 * MM,
        PAINT=dict(rgb=(0.75, 0.52, 0.12), rough=0.3, metal=0.8),
        DECALS=[(60 * MM, 90 * MM, [(0.02, 0.02, 0.02)], 1, 1)],
        WORDS=dict(
            color=["gold", "black band", "silver lid"],
            material=["aluminium", "metallic painted aluminium", "metal"],
            name=["barrel can", "energy drink can", "gold can"],
            shape=["barrel-shaped body", "opened ring-pull lid", "domed bottom"],
            function=["holding an energy drink", "pouring a drink", "being picked up and placed"],
            look="opened gold barrel-shaped energy drink can with a black band",
        ),
    ),
]


class Can:
    def __init__(self, g):
        self.g = g
        R, H = g["R"], g["H"]
        self.z_body0 = 10.0 * MM  # bottom taper meets the straight body
        self.neck_r = g["SEAM_R"] - 0.6 * MM
        self.z_neck1 = H - 3.5 * MM
        self.z_neck0 = self.z_neck1 - g["NECK_H"]
        self.z_panel = H - 6.3 * MM  # lid panel: countersink depth
        self.chime_in = g["SEAM_R"] - 1.9 * MM
        self.panel_r = self.chime_in - 1.0 * MM
        assert self.neck_r < R - 1.0 * MM and self.z_neck0 > self.z_body0 + 0.02

    def outer(self, z):
        g = self.g
        if z <= self.z_neck0:
            t = (z - self.z_body0) / (self.z_neck0 - self.z_body0)
            return g["R"] + g["BULGE"] * np.sin(np.pi * np.clip(t, 0, 1))
        t = min((z - self.z_neck0) / (self.z_neck1 - self.z_neck0), 1.0)
        return self.neck_r + (g["R"] - self.neck_r) * (1 + np.cos(np.pi * t)) / 2

    def bottom_profile(self):
        R, D = self.g["R"], self.g["DOME"]
        pts = [(0.0, D), (0.3 * R, 0.95 * D), (0.55 * R, 0.75 * D), (0.68 * R, 0.25 * D), (0.72 * R, 0.0)]
        return pts + [(0.78 * R, 0.0), (0.84 * R, 2.5 * MM), (0.93 * R, 6.5 * MM), (R, self.z_body0)]

    def parts(self):
        g, H = self.g, self.g["H"]
        body_z = np.linspace(self.z_body0, self.z_neck0, 40)
        neck_z = np.linspace(self.z_neck0, self.z_neck1, 14)
        sr, cx, cz = g["SEAM_R"], g["SEAM_R"] - 1.0 * MM, H - 1.0 * MM
        bead = [(cx + 1.0 * MM * np.cos(a), cz + 1.0 * MM * np.sin(a)) for a in np.radians(np.linspace(0, 180, 10))]
        top = [(self.outer(z), z) for z in neck_z] + [(sr, H - 3.0 * MM), (sr, cz)] + bead[1:]
        top += [(self.chime_in, self.z_panel + 0.8 * MM), (self.chime_in - 0.5 * MM, self.z_panel - 0.6 * MM)]
        top += [(self.panel_r, self.z_panel)]
        zin = np.linspace(self.z_panel - 0.9 * MM, self.z_body0, 50)
        inside = [(self.panel_r, self.z_panel), (self.panel_r, self.z_panel - 0.9 * MM)]
        inside += [(self.outer(z) - T_VIS, z) for z in zin]
        inside += [(r, z + T_VIS) for r, z in self.bottom_profile()[::-1][1:]]
        parts = {
            "model": loft([(self.outer(z), z) for z in body_z]),
            "bottom": loft(self.bottom_profile()),
            "top": loft(top),
            "inside": loft(inside),
        }
        lid = [mp.prism(p, self.z_panel - T_VIS, self.z_panel) for p in self.lid_pieces(self.panel_r)]
        lid.append(back_disc(2.2 * MM, self.z_panel, self.z_panel + 0.7 * MM))  # rivet
        parts["lid"] = merge(lid)
        parts["tab"] = merge(self.tab())
        for i, (z0, z1, colours, cols, rows) in enumerate(g["DECALS"]):
            for j, rgb in enumerate(colours):
                patches = [
                    arc_patch(self, z0 + (z1 - z0) * r / rows, z0 + (z1 - z0) * (r + 1) / rows, c, cols)
                    for r in range(rows)
                    for c in range(cols)
                    if (r + c) % len(colours) == j
                ]
                parts[f"decal_{i}{j}"] = (merge(patches), rgb)
        return parts

    def opening(self):
        return {k: v * self.panel_r for k, v in OPEN.items()}

    def lid_pieces(self, radius):
        """Convex plates covering a disc of `radius` except the pour opening."""
        o = self.opening()
        boxes = [
            (-1, 1, -1, o["y0"]),
            (-1, -o["x"], o["y0"], o["y1"]),
            (o["x"], 1, o["y0"], o["y1"]),
            (-1, 1, o["y1"], 1),
        ]
        circle = [(radius * np.cos(a), radius * np.sin(a)) for a in np.linspace(0, 2 * np.pi, CIRCLE, endpoint=False)]
        return [clip(circle, *b) for b in boxes]

    def tab(self):
        """Ring-pull plates (nose, two side bars, far bar) raised about the rivet toward -Y."""
        s, c = np.sin(TAB_TILT), np.cos(TAB_TILT)
        ex, d, n = np.array([1.0, 0, 0]), np.array([0, -c, s]), np.array([0, -s, -c])  # right-handed: ex x d = n
        pivot = np.array([0.0, 0.0, self.z_panel + 0.7 * MM])
        plates = [(-6, 6, -1, 6), (-6, -3.5, 6, 17), (3.5, 6, 6, 17), (-6, 6, 17, 20)]  # (a0, a1, b0, b1) mm
        out = []
        for a0, a1, b0, b1 in plates:
            pts, faces = mp.prism(
                [(a0 * MM, b0 * MM), (a1 * MM, b0 * MM), (a1 * MM, b1 * MM), (a0 * MM, b1 * MM)], 0.0, 0.6 * MM
            )
            out.append((pivot + pts[:, :1] * ex + pts[:, 1:2] * d + pts[:, 2:3] * n, faces))
        return out

    def hulls(self):
        g, H = self.g, self.g["H"]
        ring = SEC[:: pc.N_THETA // 24]
        hulls = [
            np.array(
                [(*(0.78 * g["R"] * p), 0.0) for p in ring]
                + [(*(self.outer(self.z_body0) * p), self.z_body0) for p in ring]
            )
        ]
        zs = np.unique(np.r_[np.arange(self.z_body0, self.z_neck1, 0.0005), self.z_neck1])
        rings = [self.outer(z) * SEC for z in zs]
        idx, i = [0], 0
        while i < len(zs) - 1:
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
        levels = [(z, self.outer(z) * SEC, (self.outer(z) - T_COLL) * SEC) for z in zs[idx]]
        levels.append((H, g["SEAM_R"] * SEC, (g["SEAM_R"] - T_COLL) * SEC))
        hulls += wall_staves(levels)
        lid_r = g["SEAM_R"] - T_COLL + 0.5 * MM  # overlaps the staves' inner face
        for p in self.lid_pieces(lid_r):
            hulls.append(np.array([(x, y, z) for z in (self.z_panel - T_COLL, self.z_panel) for x, y in p]))
        hulls.append(
            np.unique(np.concatenate([p for p, _ in self.tab()]).round(9), axis=0)
        )  # prism points are unwelded
        return hulls

    def check(self, hulls):
        g = self.g
        cavity = [
            (*(f * (self.outer(z) - T_COLL) * p), z)
            for z in np.linspace(self.z_body0 + 1 * MM, self.z_panel - 2 * MM, 30)
            for f in np.linspace(0, 0.94, 6)
            for p in SEC[:: pc.N_THETA // 48]
        ]
        o = self.opening()
        opening = [
            (x, y, z)
            for x in np.linspace(-0.8, 0.8, 5) * o["x"]
            for y in np.linspace(o["y0"] + 1 * MM, o["y1"] - 1 * MM, 5)
            for z in np.linspace(self.z_panel - 3 * MM, g["H"] + 2 * MM, 8)
        ]
        probe = np.array(cavity + opening)
        for h in hulls:
            assert len(h) <= 64, f"collision hull has {len(h)} points"
            eq = ConvexHull(h).equations
            assert not np.any(np.all(probe @ eq[:, :3].T + eq[:, 3] < -1e-6, axis=1)), (
                "hull intrudes into cavity/opening"
            )
        bottom = hulls[0][:, 2]
        assert np.all((bottom == 0) | (bottom >= 0.0015)), "bottom points within PhysX plane tolerance"


def loft(profile):
    return pc.loft([(0.0, z) if r == 0 else (r, z) for r, z in profile], SEC)


def back_disc(r, z0, z1):
    return pc.loft([(0.0, z0), (r, z0), (r, z1), (0.0, z1)], SEC)


def merge(parts):
    pts, faces = [], []
    for p, f in parts:
        base = sum(len(x) for x in pts)
        pts.append(p)
        faces += [[i + base for i in face] for face in f]
    return np.concatenate(pts), faces


def clip(poly, x0, x1, y0, y1):
    """Sutherland-Hodgman clip of a convex CCW polygon to a box; returns a convex CCW polygon."""
    for axis, val, keep in ((0, x0, 1), (0, x1, -1), (1, y0, 1), (1, y1, -1)):
        out = []
        for a, b in zip(poly, poly[1:] + poly[:1]):
            ina, inb = keep * (a[axis] - val) >= 0, keep * (b[axis] - val) >= 0
            if ina:
                out.append(a)
            if ina != inb:
                t = (val - a[axis]) / (b[axis] - a[axis])
                out.append((a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])))
        poly = out
    return poly


def arc_patch(can, z0, z1, col, cols, n=6):
    """Closed paint patch on the body surface between z0 and z1 and around column `col` of `cols`."""
    th = np.linspace(2 * np.pi * col / cols, 2 * np.pi * (col + 1) / cols, n)
    zs = np.linspace(z0, z1, 4)
    pts, faces = [], []

    def ring(z, off):
        r = can.outer(z) + off
        return [(r * np.cos(t), r * np.sin(t), z) for t in th]

    grid = [ring(z, 0.25 * MM) for z in zs] + [ring(z, 0.05 * MM) for z in zs[::-1]]
    for row in grid:
        pts += row
    m = len(grid)
    for i in range(m):
        a, b = i * n, ((i + 1) % m) * n
        faces += [[a + j, a + j + 1, b + j + 1, b + j] for j in range(n - 1)]
    for e in (0, n - 1):  # side caps: the section loop runs counter-clockwise in (r, z), facing -theta
        faces.append([i * n + e for i in range(m)][:: -1 if e else 1])
    return np.array(pts), faces


def wall_staves(levels):
    hulls = []
    for (z0, o0, i0), (z1, o1, i1) in zip(levels, levels[1:]):
        cuts = [0]
        while cuts[-1] < pc.N_THETA:
            a, b = cuts[-1], cuts[-1] + 1
            while b < pc.N_THETA and b - a < MAX_COLS and max(pc.chord_sag(r, a, b + 1) for r in (i0, i1)) <= STAVE_TOL:
                b += 1
            cuts.append(b)
        for a, b in zip(cuts, cuts[1:]):
            idx = np.arange(a, b + 1) % pc.N_THETA
            ring = [(*p, z) for r, z in ((o0, z0), (o1, z1)) for p in r[idx]]
            hulls.append(np.array(ring + [(*p, z) for r, z in ((i0, z0), (i1, z1)) for p in r[idx[[0, -1]]]]))
    return hulls


def surface(name, m):
    def material(st):
        inputs = {
            "diffuse_color_constant": m["rgb"],
            "reflection_roughness_constant": m["rough"],
            "metallic_constant": m["metal"],
        }
        return mp.add_material(st, f"/root/_materials/{name}", "OmniPBR", inputs, m["rgb"], 1.0, m["rough"])

    return material


def write_usdz(out, parts, hulls, g):
    looks = {
        "model": surface("Paint", g["PAINT"]),
        "inside": surface("Inside", dict(rgb=(0.20, 0.20, 0.21), rough=0.5, metal=1.0)),
    }
    for k in ("bottom", "top", "lid", "tab"):
        looks[k] = surface("Aluminium", ALU)
    tmp = Path(tempfile.mkdtemp()) / "object.usdc"
    st = Usd.Stage.CreateNew(str(tmp))
    UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(st, 1.0)
    root = mp.add_xform(st, "/root")
    st.SetDefaultPrim(root.GetPrim())
    UsdPhysics.RigidBodyAPI.Apply(root.GetPrim())
    mp.add_xform(st, "/root/visual")
    bound = {}
    for name, part in parts.items():  # model first: loaders read the first Mesh
        if name.startswith("decal"):
            (pts, faces), rgb = part
            make = surface(f"Paint_{name}", dict(rgb=rgb, rough=g["PAINT"]["rough"], metal=g["PAINT"]["metal"] * 0.5))
        else:
            pts, faces = part
            make = looks[name]
        mesh = mp.add_mesh(st, f"/root/visual/{name}", pts, faces, mp.vertex_normals(pts, faces))
        key = "Aluminium" if name in ("bottom", "top", "lid", "tab") else name
        if key not in bound:
            bound[key] = make(st)
        UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(bound[key])
    pc.add_collision(st, hulls)
    st.Save()
    zip_writer = getattr(Sdf, "ZipFileWriter", None) or Usd.ZipFileWriter
    zw = zip_writer.CreateNew(str(out))
    zw.AddFile(str(tmp), "object.usdc")
    zw.Save()


def description(uid, g):
    w = g["WORDS"]
    ml = np.pi * (g["R"] - T_VIS) ** 2 * (g["H"] - g["DOME"]) * 1e6 * 0.92
    caption = {
        "color": w["color"],
        "material": w["material"],
        "size": [f"about {ml:.0f} ml", f"about {g['H'] * 100:.1f} cm tall", "fits in one hand"],
        "name": w["name"],
        "shape": w["shape"],
        "function": w["function"],
    }
    desc = [
        w["look"],
        f"{w['name'][0]} with its ring-pull raised and the pour opening open",
        f"{w['material'][1]} {w['name'][2]} about {g['H'] * 100:.1f} cm tall",
        f"{w['name'][0]} with a {w['shape'][0]} and a {w['shape'][2]}",
        f"light empty {w['name'][2]} grasped around its body",
    ]
    return {"uuid": uid, "caption": caption, "description": desc}


def build(g):
    can = Can(g)
    parts = can.parts()
    closed = sum(mp.signed_volume(*parts[k]) for k in ("model", "bottom", "top", "inside"))
    assert closed > 0, "can shell has inward-facing winding"
    for k, v in parts.items():
        if k in ("lid", "tab") or k.startswith("decal"):
            pts, faces = v if k in ("lid", "tab") else v[0]
            assert mp.signed_volume(pts, faces) > 0, f"{k} has inward-facing winding"
    hulls = can.hulls()
    can.check(hulls)
    return can, parts, hulls


def area(parts):
    pts, faces = parts["model"]
    return sum(
        np.linalg.norm(np.cross(pts[f[i]] - pts[f[0]], pts[f[i + 1]] - pts[f[0]])) / 2
        for f in faces
        for i in range(1, len(f) - 1)
    )


def save(rigid_dir, idx, can, parts, hulls, g, mass):
    geo = [v if not k.startswith("decal") else v[0] for k, v in parts.items()]
    every = np.concatenate([p for p, _ in geo])
    shift = -(every.min(0) + every.max(0)) / 2
    moved = {}
    for k, v in parts.items():
        if k.startswith("decal"):
            (p, f), rgb = v
            moved[k] = ((p + shift, f), rgb)
        else:
            moved[k] = (v[0] + shift, v[1])
    hulls = [h + shift for h in hulls]
    d = Path(rigid_dir) / CATEGORY / f"{idx:05d}"
    d.mkdir(parents=True, exist_ok=True)
    uid = str(uuid.uuid5(uuid.NAMESPACE_URL, f"robodojo/rigid/{CATEGORY}/{idx}"))
    write_usdz(d / "object.usdz", moved, hulls, g)
    faces = [x for _, fs in geo for x in fs]
    meta = pc.metadata(uid, dict(pts=every + shift, faces=faces, contact_r=0.78 * g["R"]), mass, shift)
    del meta["physics"]["friction"]  # no measured friction: runtime defaults apply
    meta["visual"] = {"color_parts": ["model"]}  # recolour the painted body only
    if idx == FIRST:
        body = moved["model"][0]
        shell = np.concatenate([moved[k][0] for k in ("model", "bottom", "top")])
        meta["measurements_mm"] = {
            "直径": round(float(np.ptp(body[:, 0])) * 1000, 1),  # painted body
            "高度": round(
                float(np.ptp(shell[:, 2])) * 1000, 1
            ),  # bottom to the top of the seam, ring-pull not included
        }
    else:
        meta["variant_of"] = f"Rigid/{CATEGORY}/{FIRST:05d}"
    (d / "metadata.json").write_text(json.dumps(meta, indent=4, ensure_ascii=False))
    (d / "description.json").write_text(json.dumps(description(uid, g), indent=4))
    return d, meta


def main(rigid_dir):
    base = None
    for k, over in enumerate([{}] + VARIANTS):
        g = {**COKE_ZERO, **over}
        can, parts, hulls = build(g)
        a = area(parts)
        if k == 0:
            base, mass = a, g["MASS"]
        else:  # estimated: same wall per area as 00006
            mass = round(COKE_ZERO["MASS"] * a / base, 4)
        d, meta = save(rigid_dir, FIRST + k, can, parts, hulls, g, mass)
        print(
            f"{d}: mass {mass} kg, size {meta['physics']['size']}, {len(hulls)} collision hulls, "
            f"measurements_mm {meta.get('measurements_mm', '-')}",
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
