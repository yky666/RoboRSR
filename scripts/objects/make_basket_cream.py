#!/usr/bin/env python3
"""Cream perforated storage basket (PP, grey handle grips) as RoboDojo `Rigid` assets, plus generalization variants.

- basket/00000: Real2Sim platform registration real2sim/objects/Rigid/basket/00000 (spec.json, by 徐鹤鸣,
  2026-10-29 11:59:13): PP plastic, cream (黄白色), 266 g, 长 × 宽 × 高 358 × 253 × 96 mm (measured), "holes distributed
  unevenly". Open rectangular tray basket with rounded corners and walls flaring outward, a rolled lip, a small foot
  step at the bottom, a hand slot under a grey grip insert on each short end. Wall holes only in patches at the
  corners (the long walls are solid in the middle, the short walls solid under the slot); floor holes in a rounded
  field with a solid patch in the middle. Hole sizes, pitches and the field outline were read off the three photos
  against the measured rim size (estimated); see the constants.
- basket/00002-00007: generalization variants (`variant_of` basket/00000; colour, material, size, shape and hole
  pattern changed, mass scaled by shell area). basket/00001 and 00008-00013 belong to another basket family and are
  not written here.

Usage:
    python scripts/objects/make_basket_cream.py [RIGID_DIR]    # default Assets/Object/Real2Sim/Rigid
    python scripts/objects/check_object.py Assets/Object/Real2Sim/Rigid/basket/00000
Writes RIGID_DIR/basket/<index>/{object.usdz, metadata.json, description.json}. Needs what make_pitcher.py needs, plus
manifold3d (booleans for the through-holes).

Frame: Z up, metres, origin at bbox centre. 长 along X (grips on the ±X short ends), 宽 along Y.
USD:   /root (RigidBodyAPI) -> visual/model (cream body, real through-holes), visual/grips (grey grip inserts)
                            -> collision/hull_NN (floor plate, wall staves, rim pieces; solid, open top, <= 64 points)
The body is built from convex hulls of tapered rounded-rectangle rings (foot, wall, lip) minus the cavity, the recess
under the raised floor plate, the holes (cylinders along the wall normal) and the hand slots. It stands on the foot.
"""

import json
import os
from pathlib import Path
import sys
import tempfile
import traceback
import uuid

import manifold3d as m3d
import numpy as np
from scipy.spatial import ConvexHull

sys.path.insert(0, str(Path(__file__).resolve().parent))
import make_pitcher as mp  # noqa: E402  (shared USD helpers; starts Kit when pxr needs it)
from make_pitcher import Sdf, Usd, UsdGeom, UsdPhysics, UsdShade, Vt  # noqa: E402
import make_plastic_cups as pc  # noqa: E402  (add_collision, metadata)

RIGID_DIR = Path(__file__).resolve().parents[2] / "Assets/Object/Real2Sim/Rigid"
HULL_MAX = 64  # PhysX GPU convex limit
CAVITY_TOL = 0.0003  # cavity probe sits this far inside the inner wall surface
CORNER_N = 12  # arc points per plan corner of the visual rings
HOLE_SEG = 16  # segments per hole

BASKET = dict(
    L=0.358,  # 长, measured: outer length over the lip at the rim
    W=0.253,  # 宽, measured: outer width over the lip at the rim
    H=0.096,  # 高, measured: table to rim top (the grips stand 0.8 mm proud of the rim, not counted)
    BOT_L=0.308, BOT_W=0.228,  # outer wall size just above the foot, estimated from the flare in b0_1 / b0_2
    R_TOP=0.026, R_BOT=0.022,  # plan corner radius of the outer wall at the rim / bottom, estimated from b0_0
    T=0.0022, TF=0.0022,  # wall and floor thickness, estimated
    FZ=0.003,  # floor plate raised off the table on the wall foot (the floor holes look dark in all photos), estimated
    LIP=0.0035, LIP_H=0.007,  # rolled lip: outward overhang and height, estimated
    FOOT_H=0.008, FOOT_IN=0.0012,  # inset band at the bottom of the outer wall (the step in b0_1 / b0_2), estimated
    SLOT_W=0.066, SLOT_H=0.020, SLOT_R=0.006,  # hand slot under the grip, estimated from b0_1 (scale from the rim)
    GRIP=True, GRIP_L=0.074, GRIP_H=0.016,  # grey grip insert over the slot, estimated from b0_0 / b0_1
    WALL_D=0.0060, WALL_P=0.0115, ROW_P=0.0116,  # wall holes: diameter, pitch along a row, row pitch (staggered)
    ROW_Z0=0.0125, ROW_TOP=0.014,  # lowest row height, gap from the highest row to the rim top
    QL_TOP=0.087, QL_BOT=0.055, FADE=0.022,  # long-wall patch reach from the corner (top / bottom row), ragged edge
    SLOT_CLEAR=0.009,  # short walls: solid band each side of the slot, full height
    FLOOR_P=0.012, NCOL=24, NROW=15,  # floor hole grid (aligned, square pitch), counted in b0_0
    FLOOR_D_IN=0.0070, FLOOR_D_OUT=0.0046,  # floor holes shrink from the centre to the edge of the field (b0_0)
    BLOB=(0.012, 0.0, 0.038, 0.027),  # solid patch in the middle of the field: ellipse centre x, y and semi-axes
    LONE=((-0.024, 0.017), (-0.012, -0.019), (0.014, -0.017)),  # isolated holes inside that patch (b0_0)
    LONE_D=0.0045,
    SEED=0,
    MASS=0.266,  # measured
)  # fmt: skip
# field outline in b0_0: half-span of a hole row (fraction of the field half-length) against |row| / field half-width
FIELD_V = (0.0, 0.2, 0.35, 0.57, 0.67, 0.78, 0.86, 1.0)
FIELD_U = (1.0, 1.0, 0.89, 0.73, 0.68, 0.62, 0.53, 0.45)

CREAM = (0.80, 0.68, 0.41)  # linear; sRGB ~ (231, 216, 172), hue of the 黄白色 body in the photos
GRIP_GREY = (0.16, 0.155, 0.17)  # linear; sRGB ~ (111, 109, 115)

VARIANTS = {  # index: (overrides, body rgb, body roughness, grip rgb, words)
    2: (dict(SEED=2), (0.42, 0.58, 0.76), 0.45, GRIP_GREY,
        dict(color="powder blue", material="PP plastic", size="medium", shape="rectangular", look="powder blue")),
    3: (dict(H=0.130, QL_TOP=0.095, QL_BOT=0.050, SEED=3), (0.68, 0.68, 0.66), 0.45, (0.12, 0.12, 0.13),
        dict(color="light grey", material="PP plastic", size="deep", shape="rectangular", look="deep light grey")),
    4: (dict(H=0.060, QL_TOP=0.080, QL_BOT=0.060, SEED=4), (0.82, 0.52, 0.52), 0.45, GRIP_GREY,
        dict(color="blush pink", material="PP plastic", size="shallow", shape="tray-like", look="shallow pink")),
    5: (dict(L=0.280, W=0.280, BOT_L=0.250, BOT_W=0.250, NCOL=19, NROW=19, QL_TOP=0.090, QL_BOT=0.075,
             FLOOR_P=0.0115, BLOB=(0.0, 0.0, 0.030, 0.030), LONE=((0.0, 0.0),), SEED=5),
        (0.47, 0.74, 0.56), 0.45, GRIP_GREY,
        dict(color="mint green", material="PP plastic", size="medium", shape="square", look="square mint green")),
    6: (dict(GRIP=False, SEED=6), (0.06, 0.065, 0.07), 0.7, None,
        dict(color="charcoal", material="matte PP plastic", size="medium", shape="rectangular",
             look="charcoal matte")),
    7: (dict(L=0.270, W=0.190, H=0.075, BOT_L=0.244, BOT_W=0.175, R_TOP=0.020, R_BOT=0.017, NCOL=18, NROW=11,
             FLOOR_P=0.0115, QL_TOP=0.2, QL_BOT=0.2, FADE=0.0, SLOT_W=0.056, GRIP_L=0.064,
             BLOB=(0.0, 0.0, 0.0, 0.0), LONE=(), SEED=7),
        (0.86, 0.86, 0.84), 0.2, (0.03, 0.03, 0.03),
        dict(color="glossy white", material="glossy ABS plastic", size="small", shape="rectangular",
             look="small glossy white fully perforated")),
}  # fmt: skip


# ---------------------------------------------------------------- geometry
def rounded_rect(a, b, r, n=CORNER_N):
    """Closed CCW ring of half-sizes (a, b), corner radius r; n points per corner arc."""
    pts = []
    for k, (sx, sy) in enumerate(((1, 1), (-1, 1), (-1, -1), (1, -1))):
        t = np.linspace(k * np.pi / 2, (k + 1) * np.pi / 2, n)
        pts += list(zip(sx * (a - r) + r * np.cos(t), sy * (b - r) + r * np.sin(t)))
    return np.array(pts)


def sec(g, z, off=0.0):
    """(a, b, r) of the outer wall at height z, grown by `off` (negative: the inner surface)."""
    f = z / g["H"]
    a = g["BOT_L"] / 2 + (g["L"] / 2 - g["LIP"] - g["BOT_L"] / 2) * f
    b = g["BOT_W"] / 2 + (g["W"] / 2 - g["LIP"] - g["BOT_W"] / 2) * f
    r = g["R_BOT"] + (g["R_TOP"] - g["R_BOT"]) * f
    return a + off, b + off, max(r + off, 0.002)


def ring3(g, z, off=0.0, n=CORNER_N):
    return np.c_[rounded_rect(*sec(g, z, off), n=n), np.full(4 * n, z)]


def hull(*rings):
    return m3d.Manifold.hull_points(np.concatenate(rings))


def frame(n, p):
    """3x4 transform taking +Z to unit vector n, origin to p."""
    u = np.cross(n, (0, 0, 1) if abs(n[2]) < 0.9 else (1, 0, 0))
    u /= np.linalg.norm(u)
    return np.c_[u, np.cross(n, u), n, p]


def perimeter_point(g, z, corner, q):
    """Point and horizontal outward normal on the outer wall at height z, perimeter distance q from the 45 deg
    midpoint of `corner` (signs sx, sy); q > 0 runs onto the long wall (y = ±b), q < 0 onto the short wall."""
    a, b, r = sec(g, z)
    sx, sy = corner
    s_arc = r * np.pi / 4
    if abs(q) <= s_arc:
        phi = np.pi / 4 + q / r
        p, n = (a - r + r * np.cos(phi), b - r + r * np.sin(phi)), (np.cos(phi), np.sin(phi))
    elif q > 0:
        p, n = (a - r - (q - s_arc), b), (0.0, 1.0)
    else:
        p, n = (a, b - r - (-q - s_arc)), (1.0, 0.0)
    return np.array([sx * p[0], sy * p[1], z]), np.array([sx * n[0], sy * n[1], 0.0])


def wall_holes(g):
    """(centre, axis, diameter) of the wall holes: staggered rows in patches around the four corners."""
    rng = np.random.default_rng(g["SEED"])
    z1 = g["H"] - g["ROW_TOP"]
    rows = int(round((z1 - g["ROW_Z0"]) / g["ROW_P"])) + 1
    holes = []
    for corner in ((1, 1), (-1, 1), (-1, -1), (1, -1)):
        for k, z in enumerate(np.linspace(g["ROW_Z0"], z1, rows)):
            f = (z - g["ROW_Z0"]) / (z1 - g["ROW_Z0"])
            reach = g["QL_BOT"] + (g["QL_TOP"] - g["QL_BOT"]) * f
            for j in range(-30, 31):
                q = (j + 0.5 * (k % 2)) * g["WALL_P"]
                p, nh = perimeter_point(g, z, corner, q)
                cut = rng.random()  # drawn for every candidate so the pattern only depends on the seed
                if q > 0 and (q > reach - g["FADE"] * cut or abs(p[0]) < 0.4 * g["WALL_P"]):
                    continue  # long wall: patch fades out toward the middle, two corner patches never overlap
                if q <= 0 and (abs(p[1]) < g["SLOT_W"] / 2 + g["SLOT_CLEAR"] + g["WALL_D"] / 2 or corner[1] * p[1] < 0):
                    continue  # short wall: solid around and under the hand slot, each corner keeps to its half
                dp = (perimeter_point(g, z + 1e-4, corner, q)[0] - perimeter_point(g, z - 1e-4, corner, q)[0]) / 2e-4
                n = np.r_[nh[:2], -nh @ dp]
                holes.append((p, n / np.linalg.norm(n), g["WALL_D"]))
    return holes


def floor_holes(g):
    """(x, y, diameter) of the floor holes: aligned grid inside the field outline, minus the central solid patch."""
    fx, fy = (g["NCOL"] - 1) / 2 * g["FLOOR_P"], (g["NROW"] - 1) / 2 * g["FLOOR_P"]
    bx, by, ba, bb = g["BLOB"]
    out = []
    for i in range(g["NCOL"]):
        for j in range(g["NROW"]):
            x, y = (i - (g["NCOL"] - 1) / 2) * g["FLOOR_P"], (j - (g["NROW"] - 1) / 2) * g["FLOOR_P"]
            u, v = abs(x) / fx, abs(y) / fy
            span = np.interp(v, FIELD_V, FIELD_U)
            if u > span + 0.03:
                continue
            if ba > 0 and ((x - bx) / ba) ** 2 + ((y - by) / bb) ** 2 < 1:
                continue
            rho = min(1.0, max(u / span, v))
            out.append((x, y, g["FLOOR_D_IN"] - (g["FLOOR_D_IN"] - g["FLOOR_D_OUT"]) * rho**2))
    out += [(x, y, g["LONE_D"]) for x, y in g["LONE"]]
    return out


def build(g):
    H, T, TF, FZ = g["H"], g["T"], g["TF"], g["FZ"]
    a0, b0, _ = sec(g, 0.0, -g["FOOT_IN"])
    fx, fy = (g["NCOL"] - 1) / 2 * g["FLOOR_P"], (g["NROW"] - 1) / 2 * g["FLOOR_P"]
    ai, bi, _ = sec(g, FZ + TF, -T)
    assert fx + 0.008 < ai and fy + 0.008 < bi, "floor hole field does not fit inside the floor"

    foot = hull(ring3(g, 0.0, -g["FOOT_IN"]), ring3(g, g["FOOT_H"], -g["FOOT_IN"]))
    wall = hull(ring3(g, g["FOOT_H"]), ring3(g, H))
    lip = hull(ring3(g, H - g["LIP_H"], g["LIP"]), ring3(g, H, g["LIP"]))
    cavity = hull(ring3(g, FZ + TF, -T), ring3(g, H + 0.01, -T))
    under = hull(ring3(g, -0.01, -T), ring3(g, FZ, -T))  # recess under the raised floor plate
    body = m3d.Manifold.batch_boolean([foot, wall, lip], m3d.OpType.Add) - cavity - under

    cyl = []
    whs = wall_holes(g)
    for p, n, d in whs:
        c = m3d.Manifold.cylinder(4 * T + 0.004, d / 2, -1.0, HOLE_SEG, True)
        cyl.append(c.transform(frame(n, p - n * T / 2)))
    fhs = floor_holes(g)
    for x, y, d in fhs:
        c = m3d.Manifold.cylinder(TF + 0.004, d / 2, -1.0, HOLE_SEG, True)
        cyl.append(c.translate([x, y, FZ + TF / 2]))
    slot_z0, slot_z1 = H - g["GRIP_H"] - g["SLOT_H"], H - g["GRIP_H"] + 0.002  # top hides inside the grip
    sy, sr = g["SLOT_W"] / 2, g["SLOT_R"]
    yz = rounded_rect(sy, (slot_z1 - slot_z0) / 2, sr) + (0.0, (slot_z0 + slot_z1) / 2)
    a_top = sec(g, H)[0]
    for s in (1, -1):
        xs = (a_top - 0.03, a_top + 0.02)
        pts = np.array([(s * x, y, z) for x in xs for y, z in yz])
        cyl.append(m3d.Manifold.hull_points(pts))
    body = body - m3d.Manifold.batch_boolean(cyl, m3d.OpType.Add)
    parts = {"model": to_arrays(body)}

    if g["GRIP"]:
        gr = 0.003
        zb, zt = H - g["GRIP_H"], H + 0.0008
        xi, xo = sec(g, zb, -T)[0] - 0.0025, a_top + g["LIP"] + 0.0006
        grips = []
        for s in (1, -1):
            c = [(s * x, y, z) for x in (xi + gr, xo - gr) for y in (-g["GRIP_L"] / 2 + gr, g["GRIP_L"] / 2 - gr)
                 for z in (zb + gr, zt - gr)]  # fmt: skip
            box = m3d.Manifold.batch_hull([m3d.Manifold.sphere(gr, 12).translate(p) for p in c])
            studs = [
                m3d.Manifold.cylinder(0.0009, 0.001, -1.0, 10, False).translate([s * x, y, zt - 0.0003])
                for x in (xi + (xo - xi) / 3, xi + 2 * (xo - xi) / 3)
                for y in np.linspace(-g["GRIP_L"] / 2 + 0.006, g["GRIP_L"] / 2 - 0.006, 12)
            ]
            grips.append(m3d.Manifold.batch_boolean([box, *studs], m3d.OpType.Add))
        parts["grips"] = to_arrays(m3d.Manifold.compose(grips))

    for name, (pts, tri) in parts.items():
        assert mp.signed_volume(pts, tri) > 0, f"{name} has inward-facing winding"
    return dict(parts=parts, hulls=colliders(g), contact_r=min(a0, b0), n_wall=len(whs), n_floor=len(fhs))


def to_arrays(man):
    mesh = man.to_mesh()
    pts = np.asarray(mesh.vert_properties, dtype=float)[:, :3]
    return pts, np.asarray(mesh.tri_verts, dtype=np.int64)


# ---------------------------------------------------------------- collision
def stave_ring(g, z, off, phis_or_pts):
    """Points on the ring (a, b, r) = sec(z, off) for the stave endpoints; corner points are pushed out radially by
    1/cos(half stave angle) when `off` < 0 so the chord between them stays outside the cavity."""
    a, b, r = sec(g, z, off)
    out = []
    for kind, val, half in phis_or_pts:
        if kind == "corner":
            (sx, sy), phi = val
            rr = r / np.cos(half) if off < 0 else r
            out.append((sx * (a - r + rr * np.cos(phi)), sy * (b - r + rr * np.sin(phi)), z))
        else:  # straight-wall end: (sx, sy) of the corner, which wall
            (sx, sy), wall_kind = val
            out.append((sx * (a - r), sy * b, z) if wall_kind == "long" else (sx * a, sy * (b - r), z))
    return np.array(out)


def colliders(g, corner_staves=4):
    H, T, FT = g["H"], g["T"], g["FZ"] + g["TF"]
    hulls = [np.concatenate([ring3(g, g["FZ"], -T, n=6), ring3(g, FT, -T, n=6)])]  # floor plate, 48 points
    segs = []  # each stave: two endpoint descriptors
    corners = ((1, 1), (-1, 1), (-1, -1), (1, -1))
    for k, c in enumerate(corners):
        phis = np.linspace(k * np.pi / 2, (k + 1) * np.pi / 2, corner_staves + 1)
        half = np.pi / 4 / corner_staves
        for p0, p1 in zip(phis[:-1], phis[1:]):
            segs.append([("corner", (c, abs_phi(p0)), half), ("corner", (c, abs_phi(p1)), half)])
        nxt = corners[(k + 1) % 4]
        kind = "long" if k % 2 == 0 else "short"
        segs.append([("straight", (c, kind), 0.0), ("straight", (nxt, kind), 0.0)])
    for seg in segs:
        wall = [
            stave_ring(g, 0.0, -g["FOOT_IN"], seg),
            stave_ring(g, 0.0, -T, seg),
            stave_ring(g, H - g["LIP_H"], 0.0, seg),
            stave_ring(g, H - g["LIP_H"], -T, seg),
        ]
        rim = [
            stave_ring(g, H - g["LIP_H"], g["LIP"], seg),
            stave_ring(g, H, g["LIP"], seg),
            stave_ring(g, H - g["LIP_H"], -T, seg),
            stave_ring(g, H, -T, seg),
        ]
        hulls += [np.concatenate(wall), np.concatenate(rim)]
    return hulls


def abs_phi(phi):
    """Corner-local arc angle in [0, pi/2] (as in quadrant (1, 1)) of the global ring angle phi."""
    return float(np.arctan2(abs(np.sin(phi)), abs(np.cos(phi))))


def self_check(g, c):
    """Every hull within the GPU limit, flat bottom points exactly on z = 0 or >= 1.5 mm above, no hull reaching into
    the cavity (grid of probe points CAVITY_TOL inside the inner wall, above the floor)."""
    H, T, FT = g["H"], g["T"], g["FZ"] + g["TF"]
    probes = []
    for z in np.linspace(FT + 1e-4, H + 0.002, 40):
        a, b, r = sec(g, z, -T - CAVITY_TOL)
        ring = rounded_rect(a, b, r, n=24)
        xs, ys = np.meshgrid(np.linspace(-a, a, 60), np.linspace(-b, b, 45))
        grid = np.c_[xs.ravel(), ys.ravel()]
        dx, dy = np.maximum(np.abs(grid[:, 0]) - (a - r), 0), np.maximum(np.abs(grid[:, 1]) - (b - r), 0)
        grid = grid[dx**2 + dy**2 <= r * r]
        probes.append(np.c_[np.r_[grid, ring], np.full(len(grid) + len(ring), z)])
    probes = np.concatenate(probes)
    for h in c["hulls"]:
        assert len(h) <= HULL_MAX, f"collision hull has {len(h)} points"
        zlo = h[:, 2]
        assert np.all((np.abs(zlo) < 1e-9) | (zlo >= 0.0015)), "hull points within PhysX plane tolerance of the floor"
        eq = ConvexHull(h).equations
        inside = (probes @ eq[:, :3].T + eq[:, 3]).max(1) < -1e-7
        assert not inside.any(), f"collision hull intrudes the cavity at {probes[inside][0]}"
    return len(probes)


# ---------------------------------------------------------------- USD
def crease_normals(pts, tri, angle=35.0):
    """faceVarying normals: each corner averages the adjacent faces within `angle` of its own face."""
    fn = np.cross(pts[tri[:, 1]] - pts[tri[:, 0]], pts[tri[:, 2]] - pts[tri[:, 0]])
    un = fn / np.maximum(np.linalg.norm(fn, axis=1, keepdims=True), 1e-20)
    cv, cf = tri.reshape(-1), np.repeat(np.arange(len(tri)), 3)
    order = np.argsort(cv, kind="stable")
    starts = np.r_[0, np.flatnonzero(np.diff(cv[order])) + 1]
    counts = np.diff(np.r_[starts, len(cv)])
    grp = np.repeat(np.arange(len(counts)), counts)
    reps = counts[grp]
    a = np.repeat(np.arange(len(cv)), reps)
    b = starts[grp[a]] + np.arange(len(a)) - np.repeat(np.cumsum(reps) - reps, reps)
    fa, fb = cf[order[a]], cf[order[b]]
    keep = np.einsum("ij,ij->i", un[fa], un[fb]) > np.cos(np.radians(angle))
    tgt = order[a][keep]
    acc = np.stack([np.bincount(tgt, fn[fb[keep], k], minlength=len(cv)) for k in range(3)], 1)
    return acc / np.maximum(np.linalg.norm(acc, axis=1, keepdims=True), 1e-20)


def plastic(path, rgb, roughness):
    def material(st):
        inputs = {"diffuse_color_constant": rgb, "reflection_roughness_constant": roughness, "metallic_constant": 0.0}
        return mp.add_material(st, path, "OmniPBR", inputs, rgb, 1.0, roughness)

    return material


def write_usdz(out, c, materials):
    tmp = Path(tempfile.mkdtemp()) / "object.usdc"
    st = Usd.Stage.CreateNew(str(tmp))
    UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(st, 1.0)
    root = mp.add_xform(st, "/root")
    st.SetDefaultPrim(root.GetPrim())
    UsdPhysics.RigidBodyAPI.Apply(root.GetPrim())
    mp.add_xform(st, "/root/visual")
    for name, (pts, tri) in c["parts"].items():  # model first: loaders read the first Mesh
        mesh = mp.add_mesh(st, f"/root/visual/{name}", pts, tri.tolist())
        mesh.CreateNormalsAttr(Vt.Vec3fArray.FromNumpy(crease_normals(pts, tri).astype(np.float32)))
        mesh.SetNormalsInterpolation(UsdGeom.Tokens.faceVarying)
        UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(materials[name](st))
    pc.add_collision(st, c["hulls"])
    st.Save()
    zip_writer = getattr(Sdf, "ZipFileWriter", None) or Usd.ZipFileWriter
    zw = zip_writer.CreateNew(str(out))
    zw.AddFile(str(tmp), "object.usdc")
    zw.Save()


# ---------------------------------------------------------------- metadata
def mesh_area(pts, tri):
    return float(
        np.linalg.norm(np.cross(pts[tri[:, 1]] - pts[tri[:, 0]], pts[tri[:, 2]] - pts[tri[:, 0]]), axis=1).sum()
    )


def description(uid, g, words, grip):
    L, W, H = (round(g[k] * 100) for k in ("L", "W", "H"))
    col, mat, size, shape = words["color"], words["material"], words["size"], words["shape"]
    handle = "grey grip inserts over the hand slots" if grip else "open hand slots on both ends"
    caption = {
        "color": [col, f"{col} body", f"{col} with {'grey grips' if grip else 'no grip inserts'}"],
        "material": [mat, "injection-moulded plastic", "thin perforated plastic"],
        "size": [size, f"about {L} x {W} cm", f"about {H} cm tall"],
        "name": ["basket", "storage basket", "perforated storage basket"],
        "shape": [shape, "open rectangular tray with rounded corners", "flared walls with a rolled rim"],
        "function": ["holding small items", "storing and carrying things", "draining or airing its contents"],
    }
    desc = [
        f"{col} {mat} storage basket with an open top",
        f"{shape} basket with rounded corners, walls flaring outward and a thin rolled rim",
        "small round holes in patches near the corners of the walls and in a large field on the floor",
        f"hand slots cut into both short ends, with {handle}",
        f"light basket about {L} cm long, {W} cm wide and {H} cm tall",
    ]
    look = words["look"]
    names = {
        "en": ["basket", "storage basket", f"{col} storage basket", f"{look} perforated storage basket",
               f"{look} perforated storage basket with {handle}"],
        "zh": ["篮子", "收纳篮", f"{ZH[col]}收纳篮", f"{ZH[look]}镂空收纳篮",
               f"带{'灰色把手' if grip else '把手开口'}的{ZH[look]}镂空收纳篮"],
    }  # fmt: skip
    return {"uuid": uid, "caption": caption, "description": desc, "names": names}


ZH = {
    "cream": "米黄色",
    "powder blue": "粉蓝色",
    "light grey": "浅灰色",
    "blush pink": "浅粉色",
    "mint green": "薄荷绿",
    "charcoal": "炭黑色",
    "glossy white": "亮白色",
    "deep light grey": "深款浅灰色",
    "shallow pink": "浅款粉色",
    "square mint green": "方形薄荷绿",
    "charcoal matte": "哑光炭黑色",
    "small glossy white fully perforated": "小号满孔亮白色",
}


def save(rigid_dir, idx, g, c, materials, mass, words, variant_of=None):
    every = np.concatenate([p for p, _ in c["parts"].values()])
    shift = -(every.min(0) + every.max(0)) / 2
    parts = {k: (p + shift, f) for k, (p, f) in c["parts"].items()}
    c = dict(c, parts=parts, hulls=[h + shift for h in c["hulls"]])
    d = Path(rigid_dir) / "basket" / f"{idx:05d}"
    d.mkdir(parents=True, exist_ok=True)
    uid = str(uuid.uuid5(uuid.NAMESPACE_URL, f"robodojo/rigid/basket/{idx}"))
    write_usdz(d / "object.usdz", c, materials)
    faces = [f for _, fs in parts.values() for f in fs]
    meta = pc.metadata(uid, dict(pts=every + shift, faces=faces, contact_r=c["contact_r"]), round(mass, 4), shift)
    meta["physics"].pop("friction", None)  # no measured friction: runtime defaults apply
    ext = np.ptp(parts["model"][0], axis=0) * 1000  # cream body only, lip included, grips not
    meta["measurements_mm"] = {k: round(float(ext[i]), 1) for k, i in (("长", 0), ("宽", 1), ("高", 2))}
    if variant_of:
        meta["variant_of"] = variant_of
    (d / "metadata.json").write_text(json.dumps(meta, indent=4, ensure_ascii=False))
    (d / "description.json").write_text(
        json.dumps(description(uid, g, words, "grips" in parts), indent=4, ensure_ascii=False)
    )
    return d, meta


def main(rigid_dir):
    jobs = [(0, BASKET, CREAM, 0.45, GRIP_GREY, dict(color="cream", material="PP plastic", size="medium",
                                                     shape="rectangular", look="cream"))]  # fmt: skip
    jobs += [(i, dict(BASKET, **ov), rgb, rough, grip, w) for i, (ov, rgb, rough, grip, w) in VARIANTS.items()]
    area0 = None
    for idx, g, rgb, rough, grip_rgb, words in jobs:
        c = build(g)
        probes = self_check(g, c)
        area = mesh_area(*c["parts"]["model"])
        area0 = area0 or area
        mats = {"model": plastic("/root/_materials/BodyPP", rgb, rough)}
        if grip_rgb is not None and "grips" in c["parts"]:
            mats["grips"] = plastic("/root/_materials/GripPP", grip_rgb, 0.6)
        mass = BASKET["MASS"] * area / area0  # same wall thickness: mass follows shell area (estimated)
        d, meta = save(rigid_dir, idx, g, c, mats, mass, words, None if idx == 0 else "Rigid/basket/00000")
        nf = sum(len(f) for _, f in c["parts"].values())
        print(
            f"{d}: size {meta['physics']['size']}, measurements_mm {meta['measurements_mm']}, "
            f"mass {meta['physics']['mass']}, {c['n_wall']} wall + {c['n_floor']} floor holes, {nf} faces, "
            f"{len(c['hulls'])} hulls (max {max(map(len, c['hulls']))} pts), {probes} cavity probes clear",
            flush=True,  # APP.close() exits without flushing
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
