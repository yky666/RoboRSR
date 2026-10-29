#!/usr/bin/env python3
"""Perforated plastic tray basket (light blue PP) as a RoboDojo `Rigid` asset, plus generalization variants.

- basket/00001: Real2Sim platform registration real2sim/objects/Rigid/basket/00001 (spec.json, by 徐鹤鸣,
  2026-10-29 12:07:15): PP plastic, light blue, 178 g, 长 × 宽 × 高 338 × 258 × 84 mm (measured; 长/宽 are the outer
  extents including the rim flange, 高 is floor underside to rim top). Open rectangular tray, walls flaring outward,
  flat rim flange that overhangs further on the short ends and has downward tabs at the four corners, a rounded hand
  slot with an inner lip in each short wall, solid floor with faint ribs and a centre mark.
  Holes counted on the reference photos: long walls 4 rows x 20 columns (80 each); short walls 4 rows x 15 columns
  with the hand slot replacing the middle 5 columns of the top 2 rows (50 each). Hole size and pitch, wall drafts,
  flange overhangs, tabs and thicknesses are estimated from the photos.
- basket/00008-00013: generalization variants of 00001 (colour, material, size, shape), "variant_of" in metadata.
  basket/00000 and 00002-00007 belong to another generator; this script never writes them.

Usage:
    python scripts/objects/make_basket_blue.py [RIGID_DIR]    # default Assets/Object/Real2Sim/Rigid
    python scripts/objects/check_object.py Assets/Object/Real2Sim/Rigid/basket/00001
Writes RIGID_DIR/basket/<index>/{object.usdz, metadata.json, description.json}. Needs what make_pitcher.py needs, plus
shapely, trimesh and mapbox_earcut.

Frame: Z up, metres, origin at bbox centre; 长 along X (short walls with the hand slots at +-X), 宽 along Y.
USD:   /root (RigidBodyAPI) -> visual/model (walls with through-holes, floor, rim, skirt, tabs, slot lips; one mesh)
                            -> collision/hull_NN (solid wall pieces without holes, floor slab, rim pieces, tabs)
The wall is one strip unrolled around the perimeter (4 flat faces + 4 rounded corners). Holes and slots are cut from
the strip in 2D (along the wall x height), the strip is triangulated and mapped onto the outer and inner ruled
surfaces between the bottom and top plan outlines, and the hole sides join the two surfaces.
"""

import json
import os
from pathlib import Path
import sys
import tempfile
import traceback
import uuid

import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
from scipy.spatial import ConvexHull
from shapely.geometry import LineString, Point, Polygon, box
from shapely.ops import unary_union
import trimesh

sys.path.insert(0, str(Path(__file__).resolve().parent))
import make_pitcher as mp  # noqa: E402  (shared USD helpers; starts Kit when pxr needs it)
from make_pitcher import Sdf, Usd, UsdGeom, UsdPhysics, UsdShade  # noqa: E402
import make_plastic_cups as pc  # noqa: E402  (add_collision, metadata, clear_pp)

RIGID_DIR = Path(__file__).resolve().parents[2] / "Assets/Object/Real2Sim/Rigid"
CATEGORY = "basket"
REGISTERED = "Rigid/basket/00001"
HULL_MAX = 64  # PhysX GPU convex limit
FLAT_BAND = 0.0015  # collider points sit on the floor plane or at least this far above it (PhysX plane tolerance)
CAVITY_TOL = 0.0003  # max collider intrusion into the cavity
HOLE_SEGS = 5  # quarter-circle segments per hole (20 per round hole)
CORNER_STRIPS = 8  # wall corner facets
CORNER_HULLS = 4  # convex pieces per wall corner
SMOOTH_DEG = 35.0  # edges sharper than this are shaded flat

BLUE = dict(
    L=0.338,  # 长, measured: X extent including the end flanges
    W=0.258,  # 宽, measured: Y extent including the side flanges
    H=0.084,  # 高, measured: floor underside to rim top
    END_LIP=0.014,  # end flange overhang beyond the short-wall top, estimated (b1_1, b1_2)
    SIDE_LIP=0.005,  # side flange overhang beyond the long-wall top, estimated (b1_0, b1_1)
    R_WALL=0.014,  # plan corner radius of the wall outer surface, estimated
    R_RIM=0.016,  # plan corner radius of the rim outline, estimated (b1_1)
    DRAFT_END=10.0,  # short walls lean outward, degrees, estimated (b1_2: wall ends slope clearly)
    DRAFT_SIDE=4.0,  # long walls lean outward, degrees, estimated (b1_0: nearly vertical)
    T=0.002,  # wall thickness, estimated
    T_FLOOR=0.0025,  # floor thickness, estimated
    T_RIM=0.003,  # flange plate thickness, estimated
    SKIRT=0.007,  # downturned outer edge of the flange: height, estimated
    SKIRT_T=0.002,  # and thickness, estimated
    TAB=(0.026, 0.012, 0.002),  # corner tabs under the end flanges: drop below rim top, width, thickness, estimated
    HOLE=("round", 0.009, 0.009),  # hole shape, width, height; 0.6 x pitch on the photos, estimated
    ROWS=(0.0185, 0.0345, 0.0505, 0.0665),  # hole row centre heights (4 rows: counted; heights estimated)
    COLS_SIDE=20,  # holes per row on a long wall, counted (b1_2)
    COLS_END=15,  # holes per row on a short wall, counted (b1_0); the slot replaces 5 of them in the top 2 rows
    EDGE=0.006,  # outermost hole centre to the end of the flat wall face (rows fan out with the draft), estimated
    SLOT=(0.068, 2, 0.006),  # hand slot width, hole rows it spans (from the top), corner radius, estimated (b1_0)
    SLOT_LIP=(0.002, 0.004),  # slot frame width and depth into the cavity, estimated
    FLOOR_HOLES=None,  # (diameter, pitch) for a perforated floor
    RIBS=0.010,  # faint floor ribs across the width: spacing, estimated (b1_0)
    MARK=0.006,  # centre mark diameter, estimated (b1_0)
    COLOR=(0.13, 0.17, 0.31),  # light periwinkle blue, linear; the floor renders ~ sRGB 160, 172, 200 like the photos
    ROUGH=0.28,  # glossy
    LOOK="pp",
    MASS=0.178,  # measured
    WORDS=dict(
        colour=["light blue", "periwinkle blue", "pale blue"],
        material=["plastic", "PP plastic", "glossy polypropylene"],
        hole="round",
    ),
)


def rows(h, n, top_gap=0.0175, bottom=0.0185):
    """n evenly spaced hole rows between `bottom` and h - top_gap (the registered basket's margins)."""
    return tuple(round(float(z), 5) for z in np.linspace(bottom, h - top_gap, n))


VARIANTS = {  # index: changes to BLUE; sizes, colours and materials are invented (variants are not measured)
    8: dict(  # deeper mint basket, 5 rows
        H=0.110, ROWS=rows(0.110, 5), COLOR=(0.10, 0.25, 0.16),
        WORDS=dict(colour=["mint green", "pastel green", "light green"],
                   material=["plastic", "PP plastic", "glossy polypropylene"], hole="round"),
    ),
    9: dict(  # shallow pink tray, 3 rows
        H=0.060, ROWS=rows(0.060, 3), COLOR=(0.38, 0.12, 0.18), TAB=(0.020, 0.012, 0.002),
        WORDS=dict(colour=["pink", "rose pink", "pastel pink"],
                   material=["plastic", "PP plastic", "glossy polypropylene"], hole="round"),
    ),
    10: dict(  # square grey basket
        L=0.276, W=0.270, END_LIP=0.008, COLS_SIDE=16, COLS_END=16, COLOR=(0.12, 0.125, 0.135),
        WORDS=dict(colour=["grey", "cool grey", "medium grey"],
                   material=["plastic", "PP plastic", "glossy polypropylene"], hole="round"),
    ),
    11: dict(  # smaller yellow basket with tall oval slots
        L=0.300, W=0.220, H=0.080, HOLE=("oval", 0.007, 0.016), ROWS=rows(0.080, 3, 0.021, 0.022),
        COLS_SIDE=22, COLS_END=15, SLOT=(0.064, 1, 0.006), COLOR=(0.45, 0.30, 0.02),
        WORDS=dict(colour=["yellow", "sunny yellow", "golden yellow"],
                   material=["plastic", "PP plastic", "glossy polypropylene"], hole="oval slot"),
    ),
    12: dict(  # matte navy basket with a perforated floor
        FLOOR_HOLES=(0.007, 0.016), RIBS=None, MARK=None, COLOR=(0.015, 0.03, 0.09), ROUGH=0.6,
        WORDS=dict(colour=["navy blue", "dark blue", "deep navy"],
                   material=["plastic", "matte PP plastic", "matte polypropylene"], hole="round"),
    ),
    13: dict(  # small frosted clear basket with square holes
        L=0.250, W=0.180, H=0.070, HOLE=("square", 0.008, 0.008), ROWS=rows(0.070, 3), COLS_SIDE=15,
        COLS_END=10, SLOT=(0.054, 2, 0.005), COLOR=(0.92, 0.94, 0.96), LOOK="clear",
        WORDS=dict(colour=["clear", "frosted clear", "translucent white"],
                   material=["plastic", "translucent PP", "frosted polypropylene"], hole="square"),
    ),
}  # fmt: skip


def basket_params(idx):
    return BLUE if idx == 1 else dict(BLUE, **VARIANTS[idx])


def plan(g, z, off=0.0):
    """Wall outer surface plan at height z (scalar or array), grown by off: half sizes (a, b), corner radius."""
    a = g["L"] / 2 - g["END_LIP"] - (g["H"] - z) * np.tan(np.radians(g["DRAFT_END"])) + off
    b = g["W"] / 2 - g["SIDE_LIP"] - (g["H"] - z) * np.tan(np.radians(g["DRAFT_SIDE"])) + off
    return a, b, g["R_WALL"] + off


def face_half(g, z, k):
    """Half-length of the flat part of wall face k (0: +X, 1: +Y, 2: -X, 3: -Y); the same on both wall surfaces."""
    a, b, r = plan(g, z)
    return (b - r) if k % 2 == 0 else (a - r)


def rot(k, x, y):
    """Rotate (x, y) by k quarter turns."""
    c, s = [(1, 0), (0, 1), (-1, 0), (0, -1)][k % 4]
    return c * x - s * y, s * x + c * y


def strip_bounds(g):
    """Unrolled wall strip: u-bounds of the 8 segments (face +X, corner, face +Y, corner, ...)."""
    widths = []
    for k in range(4):
        widths += [2 * face_half(g, g["H"] / 2, k), g["R_WALL"] * np.pi / 2]
    return np.r_[0.0, np.cumsum(widths)]


def wall_points(g, ub, u, z, off):
    """Strip coordinates (u, z) -> 3D points on the wall surface grown by off (0 outer, -T inner)."""
    u, z = np.asarray(u, float), np.asarray(z, float)
    seg = np.clip(np.searchsorted(ub, u, side="right") - 1, 0, 7)
    t = (u - ub[seg]) / (ub[seg + 1] - ub[seg])
    a, b, r = plan(g, z, off)
    x, y = np.zeros_like(u), np.zeros_like(u)
    for i in range(8):
        m = seg == i
        if not m.any():
            continue
        k = i // 2
        d0, d1 = (a[m], b[m]) if k % 2 == 0 else (b[m], a[m])
        if i % 2 == 0:  # flat face: (normal distance, s) rotated into place
            x[m], y[m] = rot(k, d0, (2 * t[m] - 1) * (d1 - r))
        else:  # corner after face k
            cx, cy = rot(k, d0 - r, d1 - r)
            th = (k + t[m]) * np.pi / 2
            x[m], y[m] = cx + r * np.cos(th), cy + r * np.sin(th)
    return np.c_[x, y, z]


def to_strip(g, ub, k, pts):
    """Wall-face coordinates (s along the face, z) of face k -> strip coordinates (u, z)."""
    pts = np.asarray(pts, float)
    h = face_half(g, pts[:, 1], k)
    u = ub[2 * k] + (pts[:, 0] / (2 * h) + 0.5) * (ub[2 * k + 1] - ub[2 * k])
    return np.c_[u, pts[:, 1]]


def hole_shape(g, s, z):
    kind, w, h = g["HOLE"]
    if kind == "round":
        return Point(s, z).buffer(w / 2, quad_segs=HOLE_SEGS)
    if kind == "oval":  # vertical stadium
        return LineString([(s, z - (h - w) / 2), (s, z + (h - w) / 2)]).buffer(w / 2, quad_segs=HOLE_SEGS)
    rc = 0.15 * w  # rounded square
    return box(s - w / 2 + rc, z - h / 2 + rc, s + w / 2 - rc, z + h / 2 - rc).buffer(rc, quad_segs=2)


def slot_shape(g):
    """Hand slot outline on a short wall (s, z), spanning the top SLOT[1] hole rows."""
    w, n, rc = g["SLOT"]
    hh = g["HOLE"][2] / 2 + 0.0005
    z0, z1 = g["ROWS"][-n] - hh, g["ROWS"][-1] + hh
    return box(-w / 2 + rc, z0 + rc, w / 2 - rc, z1 - rc).buffer(rc, quad_segs=4)


def wall_cutouts(g):
    """Per face: the list of cut-out outlines in (s, z) (holes, then the slot on the short walls)."""
    slot = slot_shape(g)
    cuts = {}
    for k in range(4):
        n = g["COLS_END"] if k % 2 == 0 else g["COLS_SIDE"]
        shapes = []
        for z in g["ROWS"]:
            h = face_half(g, z, k)
            pitch = (2 * h - 2 * g["EDGE"]) / (n - 1)
            for j in range(n):
                hole = hole_shape(g, (j - (n - 1) / 2) * pitch, z)
                if k % 2 == 0 and hole.buffer(0.0015).intersects(slot):
                    continue
                shapes.append(hole)
        if k % 2 == 0:
            shapes.append(slot)
        cuts[k] = shapes
    return cuts


def triangulate(poly):
    """Earcut triangulation of a shapely (multi)polygon -> (2D vertices, CCW faces)."""
    vs, fs, n = [], [], 0
    for p in getattr(poly, "geoms", [poly]):
        if p.area < 1e-12:
            continue
        v, f = trimesh.creation.triangulate_polygon(p, engine="earcut")
        vs.append(v)
        fs.append(f + n)
        n += len(v)
    v, f = np.concatenate(vs), np.concatenate(fs)
    e1, e2 = v[f[:, 1]] - v[f[:, 0]], v[f[:, 2]] - v[f[:, 0]]
    cw = e1[:, 0] * e2[:, 1] - e1[:, 1] * e2[:, 0] < 0
    f[cw] = f[cw][:, ::-1]
    return v, f


def weld(v, f, key, tol):
    _, first, inv = np.unique(np.round(key / tol).astype(np.int64), axis=0, return_index=True, return_inverse=True)
    return v[first], inv.reshape(-1)[f]


def build_wall(g):
    """Perforated wall shell: strip triangulated in 2D, mapped to the outer and inner surfaces, hole sides joined."""
    ub = strip_bounds(g)
    cuts = wall_cutouts(g)
    holes = [to_strip(g, ub, k, np.asarray(c.exterior.coords)[:-1]) for k, cs in cuts.items() for c in cs]
    strip = Polygon([(0, 0), (ub[-1], 0), (ub[-1], g["H"]), (0, g["H"])], holes)
    assert strip.is_valid, "wall cut-outs overlap each other or the wall edges"
    corners = [np.linspace(ub[i], ub[i + 1], CORNER_STRIPS + 1) for i in range(1, 8, 2)]
    cuts_u = np.unique(np.concatenate([ub, *corners]))
    vs, fs, n = [], [], 0
    for u0, u1 in zip(cuts_u[:-1], cuts_u[1:]):
        v, f = triangulate(strip.intersection(box(u0, -1, u1, g["H"] + 1)))
        vs.append(v)
        fs.append(f + n)
        n += len(v)
    uv, tri = weld(np.concatenate(vs), np.concatenate(fs), np.concatenate(vs), 1e-9)
    outer = wall_points(g, ub, uv[:, 0], uv[:, 1], 0.0)
    inner = wall_points(g, ub, uv[:, 0], uv[:, 1], -g["T"])
    # weld in 3D: closes the strip seam at u = 0 / u = end
    _, first, inv = np.unique(np.round(outer / 1e-8).astype(np.int64), axis=0, return_index=True, return_inverse=True)
    tri = inv.reshape(-1)[tri]
    outer, inner = outer[first], inner[first]
    tri = tri[(tri[:, 0] != tri[:, 1]) & (tri[:, 1] != tri[:, 2]) & (tri[:, 0] != tri[:, 2])]
    e = np.concatenate([tri[:, [0, 1]], tri[:, [1, 2]], tri[:, [2, 0]]])
    _, ix, cnt = np.unique(np.sort(e, axis=1), axis=0, return_index=True, return_counts=True)
    border = e[ix[cnt == 1]]  # directed as in the outer triangles: holes, slots, top and bottom edges
    nv = len(outer)
    a, b = border[:, 0], border[:, 1]
    faces = np.concatenate([tri, tri[:, ::-1] + nv, np.c_[b, a, a + nv], np.c_[b, a + nv, b + nv]])
    return np.concatenate([outer, inner]), faces, ub, cuts


def rrect(a, b, r, segs=8):
    return box(-(a - r), -(b - r), a - r, b - r).buffer(r, quad_segs=segs)


def extrude(poly, h, z0=0.0):
    """Prism over a shapely (multi)polygon with holes. trimesh's extrude_polygon leaves earcut caps with mixed
    winding, and RTX culls the flipped triangles of single-sided meshes."""
    v2, f2 = triangulate(poly)
    k = len(v2)
    e = np.concatenate([f2[:, [0, 1]], f2[:, [1, 2]], f2[:, [2, 0]]])
    _, ix, cnt = np.unique(np.sort(e, axis=1), axis=0, return_index=True, return_counts=True)
    a, b = e[ix[cnt == 1]].T  # outline edges, interior on the left
    v = np.r_[np.c_[v2, np.full(k, z0)], np.c_[v2, np.full(k, z0 + h)]]
    return v, np.concatenate([f2[:, ::-1], f2 + k, np.c_[a, b, b + k], np.c_[a, b + k, a + k]])


def place(part, mat, off):
    """Linear map + offset; keeps faces outward-facing."""
    mat = np.asarray(mat, float)
    v, f = part
    return v @ mat.T + off, (f if np.linalg.det(mat) > 0 else f[:, ::-1])


def build_basket(g):
    wall_v, wall_f, ub, cuts = build_wall(g)
    parts = {"wall": (wall_v, wall_f)}
    a0, b0, r0 = plan(g, 0.0, -g["T"] / 2)
    floor = rrect(a0, b0, r0)
    if g["FLOOR_HOLES"]:
        d, p = g["FLOOR_HOLES"]
        ai, bi, _ = plan(g, g["T_FLOOR"], -g["T"])
        nx, ny = int((ai - 0.012) // p), int((bi - 0.012) // p)
        floor = floor.difference(
            unary_union([Point(i * p, j * p).buffer(d / 2, quad_segs=HOLE_SEGS)
                         for i in range(-nx, nx + 1) for j in range(-ny, ny + 1)])
        )  # fmt: skip
    parts["floor"] = extrude(floor, g["T_FLOOR"])
    deco = []
    if g["RIBS"]:
        ai, bi, _ = plan(g, g["T_FLOOR"], -g["T"])
        n = int((ai - 0.012) // g["RIBS"])
        deco += [
            box(i * g["RIBS"] - 0.0005, -(bi - 0.008), i * g["RIBS"] + 0.0005, bi - 0.008) for i in range(-n, n + 1)
        ]
    if g["MARK"]:
        deco.append(Point(0, 0).buffer(g["MARK"] / 2, quad_segs=6))
    if deco:
        dv, df, n = [], [], 0
        for p in getattr(unary_union(deco), "geoms", [unary_union(deco)]):
            v, f = extrude(p, 0.0003, g["T_FLOOR"] - 0.0001)
            dv.append(v)
            df.append(f + n)
            n += len(v)
        parts["floor_ribs"] = (np.concatenate(dv), np.concatenate(df))
    H, L2, W2 = g["H"], g["L"] / 2, g["W"] / 2
    rim_out = rrect(L2, W2, g["R_RIM"])
    at, bt, rt = plan(g, H, -g["T"] / 2)
    parts["rim"] = extrude(rim_out.difference(rrect(at, bt, rt)), g["T_RIM"], H - g["T_RIM"])
    st = g["SKIRT_T"]
    parts["skirt"] = extrude(rim_out.difference(rrect(L2 - st, W2 - st, g["R_RIM"] - st)), g["SKIRT"], H - g["SKIRT"])
    drop, tw, tt = g["TAB"]
    x1 = L2 - g["R_RIM"] + 0.004
    zt, zb = H - g["T_RIM"], H - drop
    tab = Polygon([(x1 - tw, zt), (x1, zt), (x1, zb + 0.006), (x1 - 0.006, zb), (x1 - tw, zb)])
    tab_v, tab_f = extrude(tab, tt)
    y0 = W2 - st - tt + 0.0003
    for sx in (1, -1):
        for sy in (1, -1):
            parts[f"tab_{sx}{sy}"] = place((tab_v, tab_f), [[sx, 0, 0], [0, 0, sy], [0, 1, 0]], (0, sy * y0, 0))
    lip_w, lip_d = g["SLOT_LIP"]
    slot = slot_shape(g)
    lip = extrude(slot.buffer(lip_w, quad_segs=4).difference(slot), lip_d + 0.0005)
    ai0, _, _ = plan(g, 0.0, -g["T"])
    slope = np.tan(np.radians(g["DRAFT_END"]))
    for sx in (1, -1):  # (s, z, w) -> wall plane of the short wall at +-X, w into the cavity
        parts[f"slot_lip_{sx}"] = place(lip, [[0, sx * slope, -sx], [sx, 0, 0], [0, 1, 0]], (sx * (ai0 + 0.0005), 0, 0))
    for name, (v, f) in parts.items():
        assert mp.signed_volume(v, f) > 0, f"{name} has inward-facing winding"
    hulls = collider(g)
    return dict(parts=parts, hulls=hulls, cuts=cuts, contact_r=float(min(plan(g, 0.0)[:2])))


def collider(g):
    """Solid wall pieces (no holes), floor slab, rim pieces and tabs; open top, nothing in the cavity."""
    H, T, L2, W2 = g["H"], g["T"], g["L"] / 2, g["W"] / 2
    hulls = []
    zs = np.array([0.0, H])
    for k in range(4):  # flat wall faces: outer and inner plane, full height
        pts = []
        for z in zs:
            h = face_half(g, z, k)
            for off in (0.0, -T):
                a, b, _ = plan(g, z, off)
                d = a if k % 2 == 0 else b
                pts += [rot(k, d, -h), rot(k, d, h)]
        hulls.append(np.c_[np.array(pts), np.repeat(zs, 4)])
    dth = np.pi / 2 / CORNER_HULLS
    for c in range(4):  # rounded wall corners; inner chords pushed out so they stay outside the cavity
        for j in range(CORNER_HULLS):
            th0 = c * np.pi / 2 + j * dth
            pts = []
            for z in zs:
                a, b, r = plan(g, z)
                d0, d1 = (a, b) if c % 2 == 0 else (b, a)
                cx, cy = rot(c, d0 - r, d1 - r)
                for th in (th0, th0 + dth / 2, th0 + dth):
                    pts.append((cx + r * np.cos(th), cy + r * np.sin(th), z))
                ri = (r - T) / np.cos(dth / 2)
                for th in (th0, th0 + dth):
                    pts.append((cx + ri * np.cos(th), cy + ri * np.sin(th), z))
            hulls.append(np.array(pts))
    a0, b0, r0 = plan(g, 0.0, -T / 2)
    ring = np.asarray(rrect(a0, b0, r0, 4).exterior.coords)[:-1]
    hulls.append(np.r_[np.c_[ring, np.zeros(len(ring))], np.c_[ring, np.full(len(ring), g["T_FLOOR"])]])
    zr = (H - g["SKIRT"], H)  # rim: from the wall inner surface at the top out to the flange edge, skirt depth
    ai, bi, ri = plan(g, H, -T)
    for k in range(4):
        h = face_half(g, H, k)
        d_in, d_out = (ai, L2) if k % 2 == 0 else (bi, W2)
        pts = [(*rot(k, d, s), z) for d in (d_in, d_out) for s in (-h, h) for z in zr]
        hulls.append(np.array(pts))
    rr = g["R_RIM"]
    for c in range(4):
        d0, d1 = (ai, bi) if c % 2 == 0 else (bi, ai)
        e0, e1 = (L2, W2) if c % 2 == 0 else (W2, L2)
        cx, cy = rot(c, d0 - ri, d1 - ri)
        ox, oy = rot(c, e0 - rr, e1 - rr)
        for j in range(CORNER_HULLS):
            th0 = c * np.pi / 2 + j * dth
            ths = (th0, th0 + dth / 2, th0 + dth)
            rin = ri / np.cos(dth / 2)
            pts = [(cx + rin * np.cos(t), cy + rin * np.sin(t), z) for t in (ths[0], ths[2]) for z in zr]
            pts += [(ox + rr * np.cos(t), oy + rr * np.sin(t), z) for t in ths for z in zr]
            hulls.append(np.array(pts))
    drop, tw, tt = g["TAB"]
    x1 = L2 - g["R_RIM"] + 0.004
    y0 = W2 - g["SKIRT_T"] - tt + 0.0003
    for sx in (1, -1):
        for sy in (1, -1):
            pts = [
                (sx * x, sy * y, z) for x in (x1 - tw, x1) for y in (y0, y0 + tt) for z in (H - drop, H - g["T_RIM"])
            ]
            hulls.append(np.array(pts))
    return hulls


def self_check(g, c):
    """Collider: point limits, flat floor plane, nothing inside the cavity. Returns the number of probe points."""
    for h in c["hulls"]:
        assert len(h) <= HULL_MAX, f"hull has {len(h)} points"
        z = h[:, 2]
        assert np.all((np.abs(z) < 1e-9) | (z >= FLAT_BAND)), (
            "collider point within the PhysX plane tolerance of the floor"
        )
    ub = strip_bounds(g)
    probes = []
    for z in np.linspace(g["T_FLOOR"] + CAVITY_TOL, g["H"], 14):
        u = np.linspace(0, ub[-1], 900, endpoint=False)
        probes.append(wall_points(g, ub, u, np.full_like(u, z), -g["T"] - CAVITY_TOL))
        a, b, _ = plan(g, z, -g["T"] - 0.01)
        gx, gy = np.meshgrid(np.linspace(-a, a, 15), np.linspace(-b, b, 11))
        probes.append(np.c_[gx.ravel(), gy.ravel(), np.full(gx.size, z)])
    probes = np.concatenate(probes)
    for i, h in enumerate(c["hulls"]):
        eq = ConvexHull(h).equations
        inside = (probes @ eq[:, :3].T + eq[:, 3]).max(1) < 0
        assert not inside.any(), f"collision hull {i} intrudes into the cavity at {probes[inside][0]}"
    return len(probes)


def shading(v, f):
    """Split vertices along edges sharper than SMOOTH_DEG; angle-weighted normals on the rest."""
    m = trimesh.Trimesh(v, f, process=False)
    fn = m.face_normals
    adj, edges = m.face_adjacency, m.face_adjacency_edges
    smooth = m.face_adjacency_angles < np.radians(SMOOTH_DEG)
    adj, edges = adj[smooth], edges[smooth]
    corner = np.arange(3 * len(f)).reshape(-1, 3)

    def cid(face, vert):
        return corner[face, np.argmax(f[face] == vert[:, None], axis=1)]

    rows_, cols_ = [], []
    for e in (0, 1):
        rows_.append(cid(adj[:, 0], edges[:, e]))
        cols_.append(cid(adj[:, 1], edges[:, e]))
    rows_, cols_ = np.concatenate(rows_), np.concatenate(cols_)
    # corners of the same vertex in the same face are one node already; join corners across smooth edges
    graph = coo_matrix((np.ones(len(rows_)), (rows_, cols_)), shape=(3 * len(f),) * 2)
    _, label = connected_components(graph, directed=False)
    # a corner's label must not mix different original vertices
    _, label = np.unique(np.c_[label, f.ravel()], axis=0, return_inverse=True)
    label = label.reshape(-1)
    ang = trimesh.triangles.angles(m.triangles).ravel()
    nrm = np.zeros((label.max() + 1, 3))
    np.add.at(nrm, label, fn.repeat(3, axis=0) * ang[:, None])
    nrm /= np.linalg.norm(nrm, axis=1, keepdims=True) + 1e-12
    pts = np.zeros((label.max() + 1, 3))
    pts[label] = v[f.ravel()]
    return pts, label.reshape(-1, 3), nrm


def material_fn(g):
    if g["LOOK"] == "clear":
        return lambda st: pc.clear_pp(st, g["COLOR"])
    inputs = {
        "diffuse_color_constant": g["COLOR"],
        "reflection_roughness_constant": g["ROUGH"],
        "metallic_constant": 0.0,
    }
    return lambda st: mp.add_material(st, "/root/_materials/PP", "OmniPBR", inputs, g["COLOR"], 1.0, g["ROUGH"])


def write_usdz(out, model, hulls, material):
    tmp = Path(tempfile.mkdtemp()) / "object.usdc"
    st = Usd.Stage.CreateNew(str(tmp))
    UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(st, 1.0)
    root = mp.add_xform(st, "/root")
    st.SetDefaultPrim(root.GetPrim())
    UsdPhysics.RigidBodyAPI.Apply(root.GetPrim())
    mp.add_xform(st, "/root/visual")
    pts, faces, nrm = model
    mesh = mp.add_mesh(st, "/root/visual/model", pts, faces.tolist(), nrm)
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(material(st))
    pc.add_collision(st, hulls)
    st.Save()
    zip_writer = getattr(Sdf, "ZipFileWriter", None) or Usd.ZipFileWriter
    zw = zip_writer.CreateNew(str(out))
    zw.AddFile(str(tmp), "object.usdc")
    zw.Save()


def description(uid, g):
    w = g["WORDS"]
    L, W, H = (round(g[k] * 100) for k in ("L", "W", "H"))
    square = abs(g["L"] - g["W"]) < 0.01
    tray = "square" if square else "rectangular"
    hole = w["hole"]
    floor = "perforated" if g["FLOOR_HOLES"] else "solid"
    caption = {
        "color": w["colour"],
        "material": w["material"],
        "size": [f"about {L} cm long", f"{L} x {W} cm", f"{H} cm deep"],
        "name": ["basket", "storage basket", f"perforated {tray} plastic basket"],
        "shape": [f"open {tray} tray", f"walls with rows of {hole} holes", "hand slots in both short ends"],
        "function": ["holding objects", "storing and sorting small items", "being carried by its hand slots"],
    }
    desc = [
        f"{w['colour'][0]} {w['material'][1]} storage basket shaped like an open {tray} tray",
        f"its walls are perforated with {len(g['ROWS'])} rows of {hole} holes and the floor is {floor}",
        "a rounded hand slot is cut into the top of each short end below a flat rim",
        f"about {L} cm long, {W} cm wide and {H} cm deep",
        "light shallow basket that can be lifted by its rim or hand slots to carry small objects",
    ]
    return {"uuid": uid, "caption": caption, "description": desc}


def shell_area(c):
    return sum(trimesh.Trimesh(v, f, process=False).area for k, (v, f) in c["parts"].items() if k != "floor_ribs")


def save(rigid_dir, idx, g, c, mass):
    every = np.concatenate([v for v, _ in c["parts"].values()])
    shift = -(every.min(0) + every.max(0)) / 2
    vs, fs, n = [], [], 0
    for v, f in c["parts"].values():
        vs.append(v + shift)
        fs.append(f + n)
        n += len(v)
    model = shading(np.concatenate(vs), np.concatenate(fs))
    hulls = [h + shift for h in c["hulls"]]
    d = Path(rigid_dir) / CATEGORY / f"{idx:05d}"
    d.mkdir(parents=True, exist_ok=True)
    uid = str(uuid.uuid5(uuid.NAMESPACE_URL, f"robodojo/rigid/{CATEGORY}/{idx}"))
    write_usdz(d / "object.usdz", model, hulls, material_fn(g))
    pts = every + shift
    meta = pc.metadata(uid, dict(pts=model[0], faces=model[1], contact_r=c["contact_r"]), round(mass, 4), shift)
    meta["physics"].pop("friction", None)  # no measured friction: runtime defaults apply
    ext = np.ptp(pts, axis=0) * 1000  # all visual parts: 长/宽 include the rim flange
    meta["measurements_mm"] = {k: round(float(ext[i]), 1) for k, i in (("长", 0), ("宽", 1), ("高", 2))}
    if idx != 1:
        meta = {"variant_of": REGISTERED, **meta}
    (d / "metadata.json").write_text(json.dumps(meta, indent=4, ensure_ascii=False))
    (d / "description.json").write_text(json.dumps(description(uid, g), indent=4))
    return d, meta, len(model[1])


def main(rigid_dir):
    base_area = None
    for idx in [1, *VARIANTS]:
        g = basket_params(idx)
        c = build_basket(g)
        n_probe = self_check(g, c)
        counts = [sum(1 for s in c["cuts"][k] if s.area < 0.0005) for k in range(4)]
        if idx == 1:
            assert counts == [50, 80, 50, 80], f"hole counts {counts} differ from the photos"
            base_area = shell_area(c)
        mass = BLUE["MASS"] * shell_area(c) / base_area  # variants: same wall stock, scaled by shell area (estimated)
        d, meta, n_faces = save(rigid_dir, idx, g, c, mass)
        assert n_faces < 300_000, f"{n_faces} faces"
        print(
            f"{d}: measurements_mm {meta['measurements_mm']}, mass {meta['physics']['mass']} kg, holes per wall "
            f"(+X, +Y, -X, -Y) {counts}, {n_faces} faces, {len(c['hulls'])} collision hulls, {n_probe} cavity probes"
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
