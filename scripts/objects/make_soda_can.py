#!/usr/bin/env python3
"""Opened aluminium drink cans as RoboDojo `Rigid` assets.

- soda_can/00006: Real2Sim platform registration real2sim/objects/Rigid/soda_can/00006 (spec.json, by 种政,
  2026-10-28 10:48:05): Coke Zero sleek can, opened, 13 g, 直径 57.7 mm, 高度 146.5 mm (measured). The painted body
  (straight wall and neck taper) carries a texture unwrapped from the reference photos (soda_can_label.py); bottom,
  double seam, lid and ring-pull are bare aluminium. Seam, neck, countersink, the teardrop pour opening with its
  pushed-in flap and the two-hole ring-pull are estimated from the photos.
- soda_can/00007-00012: generalization variants of 00006 (not measured; paint, finish, size and shape differ), plain
  paint with bands, same opening and ring-pull.
soda_can/00000-00005 are imported meshes (scripts/objects/local_import/).

Usage:
    python scripts/objects/make_soda_can.py [RIGID_DIR]    # default Assets/Object/Real2Sim/Rigid
Writes RIGID_DIR/soda_can/<index>/{object.usdz, metadata.json, description.json}. Needs what make_pitcher.py needs.

Frame: Z up, metres, origin at bbox centre (the raised ring-pull reaches above the seam). The printed front (logo,
无糖, trophy) faces -Y; the pour opening points from the rivet toward OPEN_DIR, the ring-pull stands on the rivet and
leans away from it.
USD:   /root (RigidBodyAPI) -> visual/model (painted body, textured for 00006), visual/bottom, visual/top (seam and
                               countersink), visual/inside (dark inner wall), visual/lid (panel with the opening and
                               rivet), visual/flap (scored panel pushed into the can), visual/tab, visual/decal_NN
                            -> collision/hull_NN (bottom, wall staves, lid sectors around the opening, ring-pull)
The can is hollow and the pour opening stays open, colliders included. textures/label.png inside the usdz (00006).
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
from make_pitcher import Sdf, Usd, UsdGeom, UsdPhysics, UsdShade, Vt  # noqa: E402
import make_plastic_cups as pc  # noqa: E402  (loft, unit_section, add_collision, metadata)
import soda_can_label  # noqa: E402

RIGID_DIR = Path(__file__).resolve().parents[2] / "Assets/Object/Real2Sim/Rigid"
CATEGORY = "soda_can"
FIRST = 6  # 00000-00005 are the imported meshes
MM = 0.001
SEC = pc.unit_section(2.0)
N = pc.N_THETA
T_VIS = 0.3 * MM  # visual wall
T_COLL = 1.5 * MM  # collider wall, grown inward
STAVE_TOL, LEVEL_TOL, MAX_COLS = 0.0006, 0.0005, 28
LABEL_TEX = "textures/label.png"
OPEN_DIR = np.radians(10.0)  # opening direction from the rivet: behind the tab in the can_1 photo (camera at 190 deg)
# Lid features for a 24.5 mm panel radius (scaled down on smaller lids), estimated from the photos (can_2, can_4):
HOLE = ((5.5, 2.5), (14.0, 7.0))  # teardrop = hull of two circles (centre along OPEN_DIR, radius) mm: 3-21 mm, 14 wide
HINGE = 2.5  # tab hinge / flap hinge line, mm from the rivet toward the opening
FLAP_DROP = np.radians(70)  # scored panel pushed down into the can about its hinge at the narrow end
TAB_TILT = np.radians(80)  # ring-pull raised 80 deg: almost vertical, leaning away from the opening
TAB = dict(  # ring-pull in its own plane, mm: b along the tab from the hinge to the ring end, a across
    outline=((1.5, 4.5), (11.5, 6.0)),  # hull of two circles (b, radius): b -3 .. 17.5, 12 wide
    finger=((11.6, 4.0),),  # large finger hole
    lower=((2.8, 2.3), (3.5, 2.3)),  # small hole over the rivet tongue
    split=6.7,  # bar between the holes
    t=0.5,
)

ALU = dict(rgb=(0.82, 0.82, 0.84), rough=0.3, metal=1.0)
COKE_ZERO = dict(
    R=57.7 / 2 * MM, H=146.5 * MM,  # 直径 / 2, 高度 (bottom to the top of the seam): measured
    SEAM_R=27.4 * MM, NECK_H=9.0 * MM, DOME=9.5 * MM, BULGE=0.0,  # 202 seam (54.8 mm), neck, bottom dome: estimated
    PAINT=dict(rgb=(0.55, 0.03, 0.03), rough=0.22, metal=0.15),
    TEXTURE=True,
    DECALS=[],
    MASS=0.013,  # measured
    WORDS=dict(
        color=["glossy red", "black Coca-Cola calligraphy", "black and yellow lower band"],
        material=["aluminium", "painted aluminium", "metal"],
        name=["soda can", "Coke Zero can", "drink can"],
        shape=["tall slim cylinder", "opened ring-pull lid", "domed bottom"],
        function=["holding a soft drink", "pouring a drink", "being picked up and placed"],
        look="opened slim red Coke Zero can with the black Chinese Coca-Cola logo and a FIFA World Cup band",
    ),
)  # fmt: skip

VARIANTS = [  # generalization variants 00007-00012: overrides of COKE_ZERO, all unmeasured (estimated)
    dict(
        R=33.1 * MM,
        H=115.2 * MM,
        NECK_H=14.0 * MM,
        DOME=10.0 * MM,
        TEXTURE=False,
        PAINT=dict(rgb=(0.60, 0.02, 0.03), rough=0.25, metal=0.2),
        DECALS=[(46 * MM, 56 * MM, (0.90, 0.90, 0.88))],
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
        TEXTURE=False,
        PAINT=dict(rgb=(0.02, 0.30, 0.08), rough=0.25, metal=0.2),
        DECALS=[(20 * MM, 32 * MM, (0.85, 0.75, 0.05)), (100 * MM, 104 * MM, (0.85, 0.85, 0.85))],
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
        TEXTURE=False,
        PAINT=dict(rgb=(0.01, 0.05, 0.35), rough=0.25, metal=0.2),
        DECALS=[(60 * MM, 75 * MM, (0.90, 0.90, 0.88)), (75 * MM, 80 * MM, (0.60, 0.02, 0.03))],
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
        SEAM_R=24.6 * MM,
        NECK_H=8.0 * MM,
        DOME=8.5 * MM,
        TEXTURE=False,
        PAINT=dict(rgb=(0.02, 0.02, 0.022), rough=0.7, metal=0.1),
        DECALS=[(30 * MM, 33 * MM, (0.70, 0.05, 0.05))],
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
        TEXTURE=False,
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
        TEXTURE=False,
        PAINT=dict(rgb=(0.75, 0.52, 0.12), rough=0.3, metal=0.8),
        DECALS=[(60 * MM, 90 * MM, (0.02, 0.02, 0.02))],
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


def circle_hull(circles, n=64):
    """Convex CCW polygon: hull of circles [(centre along the axis, radius)] in (along, across) coordinates."""
    pts = np.concatenate(
        [np.c_[c + r * np.cos(a), r * np.sin(a)] for c, r in circles for a in [np.linspace(0, 2 * np.pi, n, False)]]
    )
    return pts[ConvexHull(pts).vertices]


def ray_poly(poly, c, u):
    """Distance from c along unit directions u (K, 2) to the boundary of the convex polygon around c."""
    p, q = poly, np.roll(poly, -1, axis=0)
    e = q - p
    best = np.full(len(u), np.inf)
    for (px, py), (ex, ey) in zip(p - c, e):
        den = u[:, 0] * ey - u[:, 1] * ex
        with np.errstate(divide="ignore", invalid="ignore"):
            t = (px * ey - py * ex) / den
            s = (px * u[:, 1] - py * u[:, 0]) / den
        ok = (np.abs(den) > 1e-12) & (t > 0) & (s >= -1e-9) & (s <= 1 + 1e-9)
        best[ok] = np.minimum(best[ok], t[ok])
    return best


def ray_circle(r, c, u):
    cu = u @ c
    return -cu + np.sqrt(cu**2 - c @ c + r * r)


def holed_slab(outer, hole, c, z0, z1, k=96):
    """Closed plate z0..z1 between two curves star-shaped about c: outer(u) and hole(u) give the distances along
    unit directions u. Returns 3-D points and outward faces."""
    ang = np.linspace(0, 2 * np.pi, k, endpoint=False)
    u = np.c_[np.cos(ang), np.sin(ang)]
    ro, rh = outer(u), hole(u)
    rings = [(rh, z1), (ro, z1), (ro, z0), (rh, z0)]
    pts = np.concatenate([np.c_[c + r[:, None] * u, np.full(k, z)] for r, z in rings])
    faces = []
    for i in range(4):
        a, b = i * k, ((i + 1) % 4) * k
        faces += [[a + j, b + j, b + (j + 1) % k, a + (j + 1) % k] for j in range(k)]
    if mp.signed_volume(pts, faces) < 0:
        faces = [f[::-1] for f in faces]
    return pts, faces


def orient(pts, faces):
    return (pts, faces) if mp.signed_volume(pts, faces) > 0 else (pts, [f[::-1] for f in faces])


class Can:
    def __init__(self, g):
        self.g = g
        R, H = g["R"], g["H"]
        self.z_body0 = 6.0 * MM  # bottom taper meets the painted wall
        self.seam_bot = H - 3.0 * MM  # double seam: 3 mm tall, bare aluminium
        self.neck_r = g["SEAM_R"] - 0.5 * MM
        self.z_neck0 = self.seam_bot - g["NECK_H"]
        self.z_panel = H - 6.3 * MM  # lid panel: countersink depth
        self.chime_in = g["SEAM_R"] - 1.9 * MM
        self.panel_r = self.chime_in - 1.0 * MM
        self.k = min(1.0, self.panel_r / (24.5 * MM)) * MM  # lid feature scale, m per drawing mm
        e = np.array([np.cos(OPEN_DIR), np.sin(OPEN_DIR)])
        self.e, self.w = e, np.array([-e[1], e[0]])
        self.hole = self.to_lid(circle_hull(HOLE))
        self.hole_c = self.hole.mean(0)
        assert self.neck_r < R - 0.5 * MM and self.z_neck0 > self.z_body0 + 0.02

    def to_lid(self, sw):
        """(along OPEN_DIR, across) in drawing mm -> lid-plane metres."""
        return sw[:, :1] * self.k * self.e + sw[:, 1:2] * self.k * self.w

    def outer(self, z):
        g = self.g
        if z <= self.z_neck0:
            t = (z - self.z_body0) / (self.z_neck0 - self.z_body0)
            return g["R"] + g["BULGE"] * np.sin(np.pi * np.clip(t, 0, 1))
        t = min((z - self.z_neck0) / (self.seam_bot - self.z_neck0), 1.0)
        return self.neck_r + (g["R"] - self.neck_r) * (1 + np.cos(np.pi * t)) / 2

    def bottom_profile(self):
        R, D = self.g["R"], self.g["DOME"]
        pts = [(0.0, D), (0.3 * R, 0.95 * D), (0.55 * R, 0.75 * D), (0.68 * R, 0.25 * D), (0.72 * R, 0.0)]
        return pts + [(0.78 * R, 0.0), (0.84 * R, 1.8 * MM), (0.93 * R, 4.2 * MM), (R, self.z_body0)]

    def body_z(self):
        return np.r_[np.linspace(self.z_body0, self.z_neck0, 40)[:-1], np.linspace(self.z_neck0, self.seam_bot, 14)]

    def parts(self):
        g, H = self.g, self.g["H"]
        sr, cx, cz = g["SEAM_R"], g["SEAM_R"] - 1.0 * MM, H - 1.0 * MM
        bead = [(cx + 1.0 * MM * np.cos(a), cz + 1.0 * MM * np.sin(a)) for a in np.radians(np.linspace(0, 180, 10))]
        top = [(self.neck_r, self.seam_bot), (sr, self.seam_bot), (sr, cz)] + bead[1:]
        top += [(self.chime_in, self.z_panel + 0.8 * MM), (self.chime_in - 0.5 * MM, self.z_panel - 0.6 * MM)]
        top += [(self.panel_r, self.z_panel)]
        zin = np.linspace(self.z_panel - 0.9 * MM, self.z_body0, 50)
        inside = [(self.panel_r, self.z_panel), (self.panel_r, self.z_panel - 0.9 * MM)]
        inside += [(self.outer(z) - T_VIS, z) for z in zin]
        inside += [(r, z + T_VIS) for r, z in self.bottom_profile()[::-1][1:]]
        parts = {
            "model": loft([(self.outer(z), z) for z in self.body_z()]),
            "bottom": loft(self.bottom_profile()),
            "top": loft(top),
            "inside": loft(inside),
        }
        c = self.hole_c
        lid = [
            holed_slab(
                lambda u: ray_circle(self.panel_r, c, u),
                lambda u: ray_poly(self.hole, c, u),
                c,
                self.z_panel - T_VIS,
                self.z_panel,
            )
        ]
        tongue = self.to_lid(np.array([[-2.8, -2.4], [HINGE, -2.4], [HINGE, 2.4], [-2.8, 2.4]]))
        lid.append(mp.prism([tuple(p) for p in tongue], self.z_panel, self.z_panel + 0.4 * MM))
        lid.append(back_disc(2.0 * self.k, self.z_panel + 0.4 * MM, self.z_panel + 1.0 * MM))  # rivet
        parts["lid"] = merge(lid)
        parts["flap"] = self.flap()
        parts["tab"] = merge(self.tab())
        for i, (z0, z1, rgb) in enumerate(g["DECALS"]):
            parts[f"decal_{i:02d}"] = (band(self, z0, z1), rgb)
        return parts

    def flap(self):
        """Scored panel (the opening, 0.4 mm smaller) hinged at its narrow end and pushed down into the can."""
        poly = circle_hull([(s, r - 0.4) for s, r in HOLE], 32)
        pts, faces = mp.prism([tuple(p) for p in poly], -T_VIS, 0.0)
        s = pts[:, 0] - (HINGE + 0.9)
        along = HINGE + 0.9 + s * np.cos(FLAP_DROP)
        z = self.z_panel - 0.6 * MM + (pts[:, 2] / self.k - s * np.sin(FLAP_DROP)) * self.k
        xy = self.to_lid(np.c_[along, pts[:, 1]])
        return orient(np.c_[xy, z], faces)

    def tab_frame(self):
        s, c = np.sin(TAB_TILT), np.cos(TAB_TILT)
        e3, w3 = np.r_[self.e, 0.0], np.r_[self.w, 0.0]
        d = -c * e3 + np.array([0, 0, s])  # along the tab, ring end up and away from the opening
        n = -s * e3 - c * np.array([0, 0, 1.0])  # thickness: away from the opening
        pivot = np.r_[HINGE * self.k * self.e, self.z_panel + 0.4 * MM]
        return pivot, w3, d, n

    def tab_map(self, pts):
        pivot, w3, d, n = self.tab_frame()
        return pivot + (pts[:, :1] * w3 + pts[:, 1:2] * d) * self.k + pts[:, 2:3] * n * self.k

    def tab(self):
        """Ring-pull: two holed plates (nose + small hole, ring + finger hole) split at the bar between the holes."""
        outline = circle_hull(TAB["outline"])[:, ::-1]  # (a across, b along)
        out = []
        for hole, keep in ((TAB["lower"], -1), (TAB["finger"], 1)):
            region = clip_half(outline, TAB["split"], keep)
            hp = circle_hull(hole, 48)[:, ::-1]
            c = hp.mean(0)
            pts, faces = holed_slab(lambda u: ray_poly(region, c, u), lambda u: ray_poly(hp, c, u), c, 0.0, TAB["t"])
            out.append(orient(self.tab_map(pts), faces))
        return out

    def hulls(self):
        g, H = self.g, self.g["H"]
        ring = SEC[:: N // 24]
        hulls = [
            np.array(
                [(*(0.78 * g["R"] * p), 0.0) for p in ring]
                + [(*(self.outer(self.z_body0) * p), self.z_body0) for p in ring]
            )
        ]
        zs = np.unique(np.r_[np.arange(self.z_body0, self.seam_bot, 0.0005), self.seam_bot])
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
        c = self.hole_c
        for a0, a1 in zip(*[np.linspace(0, 2 * np.pi, 41)[s:] for s in (0, 1)]):  # sectors around the opening
            ang = np.linspace(a0, a1, 3)
            u = np.c_[np.cos(ang), np.sin(ang)]
            arc = np.r_[c + ray_poly(self.hole, c, u)[:, None] * u, c + ray_circle(lid_r, c, u)[:, None] * u]
            hulls.append(np.array([(x, y, z) for z in (self.z_panel - T_COLL, self.z_panel) for x, y in arc]))
        outline = circle_hull(TAB["outline"], 12)[:, ::-1]
        outline = clip_half(outline, 0.0, 1)  # above the hinge: the nose inside the opening is left out
        flat = np.c_[outline, np.zeros(len(outline))]
        hulls.append(self.tab_map(np.r_[flat, flat + [0, 0, TAB["t"]]]))
        return hulls

    def probes(self):
        cavity = [
            (*(f * (self.outer(z) - T_COLL) * p), z)
            for z in np.linspace(self.z_body0 + 1 * MM, self.z_panel - 2 * MM, 30)
            for f in np.linspace(0, 0.94, 6)
            for p in SEC[:: N // 48]
        ]
        c = self.hole_c
        ang = np.linspace(0, 2 * np.pi, 48, endpoint=False)
        u = np.c_[np.cos(ang), np.sin(ang)]
        rh = ray_poly(self.hole, c, u)
        opening = [
            (*(c + f * (r - 0.6 * MM) * v), z)
            for f in (0.0, 0.4, 0.75, 1.0)
            for r, v in zip(rh, u)
            for z in np.linspace(self.z_panel - 3 * MM, self.g["H"] + 2 * MM, 8)
        ]
        return np.array(cavity), np.array(opening)

    def check(self, hulls):
        cavity, opening = self.probes()
        for name, probe in (("cavity", cavity), ("opening", opening)):
            for h in hulls:
                assert len(h) <= 64, f"collision hull has {len(h)} points"
                eq = ConvexHull(h).equations
                assert not np.any(np.all(probe @ eq[:, :3].T + eq[:, 3] < -1e-6, axis=1)), f"hull intrudes {name}"
        bottom = hulls[0][:, 2]
        assert np.all((bottom == 0) | (bottom >= 0.0015)), "bottom points within PhysX plane tolerance"
        return len(cavity), len(opening)


def clip_half(poly, b, keep):
    """Clip a convex CCW polygon [(a, b)] to keep * (b - value) >= 0."""
    out = []
    for p, q in zip(poly, np.roll(poly, -1, axis=0)):
        ip, iq = keep * (p[1] - b) >= 0, keep * (q[1] - b) >= 0
        if ip:
            out.append(p)
        if ip != iq:
            t = (b - p[1]) / (q[1] - p[1])
            out.append(p + t * (q - p))
    return np.array(out)


def loft(profile):
    return pc.loft([(0.0, z) if r == 0 else (r, z) for r, z in profile], SEC)


def back_disc(r, z0, z1):
    return pc.loft([(0.0, z0), (r, z0), (r, z1), (0.0, z1)], SEC)


def band(can, z0, z1):
    """Closed paint band 0.05-0.25 mm proud of the body between z0 and z1."""
    zs = np.linspace(z0, z1, 6)
    out = [(can.outer(z) + 0.25 * MM, z) for z in zs]
    inn = [(can.outer(z) + 0.05 * MM, z) for z in zs[::-1]]
    return loft(out + [out[-1]] + inn + [inn[-1], out[0]])


def merge(parts):
    pts, faces = [], []
    for p, f in parts:
        base = sum(len(x) for x in pts)
        pts.append(p)
        faces += [[i + base for i in face] for face in f]
    return np.concatenate(pts), faces


def body_uv(can, pts, faces):
    """Per-face-vertex UVs of the painted body: u = angle / 2pi CCW from +X, v up from the paint bottom."""
    uv = []
    z0, z1 = can.z_body0, can.seam_bot
    for f in faces:  # vertex i is column i % N of its ring
        col = np.array(f) % N
        if np.ptp(col) > N / 2:
            col[col < N / 2] += N
        uv += [(c / N, (pts[i, 2] - z0) / (z1 - z0)) for c, i in zip(col, f)]
    return np.array(uv)


def wall_staves(levels):
    hulls = []
    for (z0, o0, i0), (z1, o1, i1) in zip(levels, levels[1:]):
        cuts = [0]
        while cuts[-1] < N:
            a, b = cuts[-1], cuts[-1] + 1
            while b < N and b - a < MAX_COLS and max(pc.chord_sag(r, a, b + 1) for r in (i0, i1)) <= STAVE_TOL:
                b += 1
            cuts.append(b)
        for a, b in zip(cuts, cuts[1:]):
            idx = np.arange(a, b + 1) % N
            ring = [(*p, z) for r, z in ((o0, z0), (o1, z1)) for p in r[idx]]
            hulls.append(np.array(ring + [(*p, z) for r, z in ((i0, z0), (i1, z1)) for p in r[idx[[0, -1]]]]))
    return hulls


def surface(name, m, texture=None):
    def material(st):
        inputs = {
            "diffuse_color_constant": m["rgb"],
            "reflection_roughness_constant": m["rough"],
            "metallic_constant": m["metal"],
        }
        mat = mp.add_material(st, f"/root/_materials/{name}", "OmniPBR", inputs, m["rgb"], 1.0, m["rough"])
        if texture:
            sh = UsdShade.Shader.Get(st, f"/root/_materials/{name}/Shader")
            sh.CreateInput("diffuse_texture", Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(texture))
        return mat

    return material


def write_usdz(out, parts, hulls, g, uv=None, label_png=None):
    looks = {
        "model": surface("Paint", g["PAINT"], LABEL_TEX if uv is not None else None),
        "inside": surface("Inside", dict(rgb=(0.20, 0.20, 0.21), rough=0.5, metal=1.0)),
    }
    bare = ("bottom", "top", "lid", "flap", "tab")
    for k in bare:
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
        if name == "model" and uv is not None:
            UsdGeom.PrimvarsAPI(mesh).CreatePrimvar(
                "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.faceVarying
            ).Set(Vt.Vec2fArray.FromNumpy(uv.astype(np.float32)))
        key = "Aluminium" if name in bare else name
        if key not in bound:
            bound[key] = make(st)
        UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(bound[key])
    pc.add_collision(st, hulls)
    st.Save()
    zip_writer = getattr(Sdf, "ZipFileWriter", None) or Usd.ZipFileWriter
    zw = zip_writer.CreateNew(str(out))
    zw.AddFile(str(tmp), "object.usdc")
    if label_png is not None:
        zw.AddFile(str(label_png), LABEL_TEX)
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
        f"{w['name'][0]} with its ring-pull raised and the teardrop pour opening open",
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
        if k in ("lid", "flap", "tab") or k.startswith("decal"):
            pts, faces = v if not k.startswith("decal") else v[0]
            assert mp.signed_volume(pts, faces) > 0, f"{k} has inward-facing winding"
    hulls = can.hulls()
    n_cav, n_open = can.check(hulls)
    return can, parts, hulls, (n_cav, n_open)


def area(parts):
    pts, faces = parts["model"]
    return sum(
        np.linalg.norm(np.cross(pts[f[i]] - pts[f[0]], pts[f[i + 1]] - pts[f[0]])) / 2
        for f in faces
        for i in range(1, len(f) - 1)
    )


def save(rigid_dir, idx, can, parts, hulls, g, mass, label_png=None):
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
    uv = body_uv(can, *parts["model"]) if label_png is not None else None
    write_usdz(d / "object.usdz", moved, hulls, g, uv, label_png)
    faces = [x for _, fs in geo for x in fs]
    meta = pc.metadata(uid, dict(pts=every + shift, faces=faces, contact_r=0.78 * g["R"]), mass, shift)
    meta["physics"].pop("friction", None)  # no measured friction: runtime defaults apply
    meta["visual"] = {"color_parts": ["model"]}  # recolour the painted body only
    if idx == FIRST:
        body = moved["model"][0]
        shell = np.concatenate([moved[k][0] for k in ("model", "bottom", "top")])
        meta["measurements_mm"] = {
            "直径": round(float(np.ptp(body[:, 0])) * 1000, 1),  # painted body
            "高度": round(float(np.ptp(shell[:, 2])) * 1000, 1),  # bottom to the top of the seam, no ring-pull
        }
    else:
        meta["variant_of"] = f"Rigid/{CATEGORY}/{FIRST:05d}"
    (d / "metadata.json").write_text(json.dumps(meta, indent=4, ensure_ascii=False))
    (d / "description.json").write_text(json.dumps(description(uid, g), indent=4))
    return d, meta


def main(rigid_dir):
    base = None
    label_png = soda_can_label.label(Path(tempfile.mkdtemp()) / "label.png")
    for k, over in enumerate([{}] + VARIANTS):
        g = {**COKE_ZERO, **over}
        can, parts, hulls, probes = build(g)
        a = area(parts)
        if k == 0:
            base, mass = a, g["MASS"]
        else:  # estimated: same wall per area as 00006
            mass = round(COKE_ZERO["MASS"] * a / base, 4)
        d, meta = save(rigid_dir, FIRST + k, can, parts, hulls, g, mass, label_png if g["TEXTURE"] else None)
        print(
            f"{d}: mass {mass} kg, size {meta['physics']['size']}, {len(hulls)} collision hulls "
            f"(max {max(len(h) for h in hulls)} points), probes clear: {probes[0]} cavity / {probes[1]} opening, "
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
