#!/usr/bin/env python3
"""iPhone 17 Pro Max (screen off) as RoboDojo `Rigid` assets, one per finish.

- iphone_17_pro_max/00000: cosmic orange. Not a registered real object: dimensions and mass from Apple's tech specs
  (support.apple.com/125091; mainland China model 231 g) and the dimensional drawing Apple publishes for accessory
  makers (developer.apple.com/accessories/dimensional-drawings, iPhone 17 Pro Max, 2025-09-09). Body 77.98 x 163.43 x
  8.75 mm,
  plan corners through the drawing's corner points, camera plateau 2.55 mm proud, lenses 1.88 mm above it, lens, flash,
  sensor and microphone positions and button positions from the drawing (all "官方"). Estimated: edge radii, plateau
  outline below the lenses, back glass panel inset, finish colours (sampled from Apple Newsroom product photos).
  Not modelled: Apple logo, speaker and microphone holes, antenna bands, SIM tray seam.
- iphone_17_pro_max/00001, 00002: deep blue and silver, generalization variants of 00000 (colour only).

Usage:
    python scripts/objects/make_iphone_17_pro_max.py [RIGID_DIR]    # default Assets/Object/Real2Sim/Rigid
    python scripts/objects/check_object.py Assets/Object/Real2Sim/Rigid/iphone_17_pro_max/0000{0,1,2} \
        --image-dir real2sim/objects
Writes RIGID_DIR/iphone_17_pro_max/<index>/{object.usdz, metadata.json, description.json}. Needs what make_pitcher.py
needs.

Frame: Z up out of the screen, metres, origin at bbox centre. Width (宽) along X, height (高) along Y with the camera at
+Y, the camera on the +X half seen from the back; action and volume buttons on -X, side button and camera control on
+X. Lying screen up, the phone rests on its lenses and the bottom of the back, tilted ~1.8 deg like the real one.
USD:   /root (RigidBodyAPI) -> visual/model (aluminium frame and back), visual/<part> (front glass, plateau, lenses)
                            -> collision/hull_NN (body, plateau, one per lens; each <= 64 points)
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
CATEGORY = "iphone_17_pro_max"
N = 256  # points per plan outline
MM = 0.001

W, L, T = 77.98 * MM, 163.43 * MM, 8.75 * MM  # official: product width, length, thickness
C, NC = 19.43 * MM, 3.17  # official corner: tangent 19.43 mm from each edge; exponent fitted to the drawing's points
EB = 1.3 * MM  # back edge radius, estimated
EF_H, EF_V = 2.61 * MM, 1.8 * MM  # front edge: official 2.61 mm from the housing to the flat top; height estimated
GLASS_EDGE = 1.2 * MM  # official: cover glass 75.58 mm wide in a 77.98 mm housing
PLATEAU_H, LENS_H = 2.55 * MM, 1.88 * MM  # official: back plate to plateau, plateau to camera glass
PLATEAU_LEN, PLATEAU_R, PLATEAU_FIL = 46.0 * MM, 11.96 * MM, 1.0 * MM  # length from the top, lower corners: drawing
LENS_D = 16.20 * MM  # official, 3x
LENSES = [(14.37, 14.37), (14.37, 33.61), (32.36, 23.99)]  # official: (from the camera-side edge, from the top), mm
FLASH, SENSOR, MIC = (64.16, 13.82, 6.80), (64.16, 34.16, 6.65), (64.16, 23.99, 1.15)  # official: x, y, diameter
ISLAND = (7.91, 20.76, 6.07)  # centre from the top (official), width (official), height (iOS 37 pt at 460 ppi), mm
PANEL_INSET, PANEL_TOP, PANEL_R = 3.4 * MM, 48.5 * MM, 14.0 * MM  # back glass panel, estimated from product photos
BUTTON_PROUD, BUTTON_T = 0.45 * MM, 2.66 * MM  # official: buttons stand 0.45 mm proud, 2.66 mm across
BUTTONS = {  # official: centre from the top and length, mm; side -1 = action/volume side
    "action": (-1, 34.28, 6.9), "volume_up": (-1, 48.43, 11.2), "volume_down": (-1, 62.63, 11.2),
    "side": (1, 55.53, 17.7), "camera_control": (1, 111.82, 17.1),
}  # fmt: skip
USB_C = (9.0 * MM, 3.12 * MM)  # official recommended connector keep-out on the bottom face
MASS = 0.231  # official, mainland China model

FINISHES = [  # colour word, aluminium (linear RGB, metallic), back glass panel (linear RGB)
    # the matte dielectric panel renders brighter than the metallic aluminium of the same colour: darker panel values
    ("cosmic orange", (0.62, 0.15, 0.03), 0.75, (0.45, 0.10, 0.018)),
    ("deep blue", (0.035, 0.045, 0.09), 0.75, (0.022, 0.028, 0.06)),
    ("silver", (0.62, 0.62, 0.62), 0.85, (0.70, 0.70, 0.68)),
]
FIXED = {  # rgb, roughness, metallic
    "front": ((0.004, 0.004, 0.005), 0.04, 0.0),
    "island": ((0.0, 0.0, 0.0), 0.5, 0.0),
    "lens_glass": ((0.01, 0.012, 0.02), 0.03, 0.0),
    "flash": ((0.75, 0.72, 0.62), 0.35, 0.0),
    "sensor": ((0.02, 0.02, 0.02), 0.2, 0.0),
    "mic": ((0.0, 0.0, 0.0), 0.8, 0.0),
    "camera_control": ((0.05, 0.05, 0.055), 0.1, 0.0),
    "port": ((0.01, 0.01, 0.01), 0.8, 0.0),
}
BODY_PARTS = ("model", "plateau", "lens_ring", "buttons")
HULL_MAX = 64
FLAT_BAND = 0.0015  # collider points stay on a flat face or at least this far from it (PhysX plane tolerance)


def outline(hw, top, bot, ct, nt, cb, nb, n=N):
    """Closed CCW plan outline, x in [-hw, hw], y in [bot, top]: top corners superellipse (extent ct, exponent nt),
    bottom corners (cb, nb; nb = 2 is circular); n points uniform by arc length from (hw, top - ct)."""
    t = np.linspace(0, np.pi / 2, 96)

    def corner(cx, cy, c, ne, sx, sy):
        return np.c_[cx + sx * c * np.cos(t) ** (2 / ne), cy + sy * c * np.sin(t) ** (2 / ne)]

    poly = np.vstack(
        [
            corner(hw - ct, top - ct, ct, nt, 1, 1),
            corner(-hw + ct, top - ct, ct, nt, -1, 1)[::-1],
            corner(-hw + cb, bot + cb, cb, nb, -1, -1),
            corner(hw - cb, bot + cb, cb, nb, 1, -1)[::-1],
        ]
    )
    closed = np.vstack([poly, poly[:1]])
    s = np.r_[0, np.cumsum(np.linalg.norm(np.diff(closed, axis=0), axis=1))]
    u = np.linspace(0, s[-1], n, endpoint=False)
    return np.c_[np.interp(u, s, closed[:, 0]), np.interp(u, s, closed[:, 1])]


def body_ring(d):
    return outline(W / 2 - d, L / 2 - d, -L / 2 + d, C - d, NC, C - d, NC)


def build_body():
    """Aluminium frame and back up to the cover glass edge, and the black cover glass above it."""
    back = [(EB - EB * np.sin(a), EB - EB * np.cos(a)) for a in np.linspace(0, np.pi / 2, 7)]  # (inset, z)
    sides = [(0.0, T - EF_V)]
    phi_glass = np.arccos(1 - GLASS_EDGE / EF_H)
    phis = np.r_[np.linspace(0, phi_glass, 5), np.linspace(phi_glass, np.pi / 2, 9)[1:]]
    front = [(EF_H * (1 - np.cos(p)), T - EF_V + EF_V * np.sin(p)) for p in phis]
    k = 5  # front rings up to and including the glass edge
    frame = [(0.0, 0.0)] + [(body_ring(d), z) for d, z in back + sides + front[:k]]
    glass = [(body_ring(d), z) for d, z in front[k - 1 :]] + [(0.0, T)]
    return pc.loft(frame), pc.loft(glass)


def build_plateau():
    hw, top, bot = W / 2 - EB, L / 2 - EB, L / 2 - PLATEAU_LEN
    yc = (top + bot) / 2

    def ring(d):
        return outline(hw - d, top - yc - d, bot - yc + d, C - EB - d, NC, PLATEAU_R - d, 2.0)

    fillet = [
        (PLATEAU_FIL * (1 - np.sin(a)), -PLATEAU_H + PLATEAU_FIL * (1 - np.cos(a)))
        for a in np.linspace(0, np.pi / 2, 6)
    ]
    prof = [(0.0, -PLATEAU_H)] + [(ring(d), z) for d, z in fillet] + [(ring(0.0), 0.3 * MM), (0.0, 0.3 * MM)]
    pts, faces = pc.loft(prof)
    return pts + [0.0, yc, 0.0], faces


def back_lathe(profile, x_mm, y_mm):
    """Surface of revolution [(r, u)] with u out of the back, centred at a drawing position (mm from the camera-side
    edge and from the top). Rotated 180 deg about X (u -> -z), which keeps the winding."""
    pts, faces = pc.loft(profile, pc.unit_section(2.0))
    pts = pts * [1.0, -1.0, -1.0]
    return pts + [W / 2 - x_mm * MM, L / 2 - y_mm * MM, 0.0], faces


def merge(parts):
    pts, faces = [], []
    for p, f in parts:
        base = sum(len(x) for x in pts)
        pts.append(p)
        faces += [[i + base for i in face] for face in f]
    return np.concatenate(pts), faces


def stadium(length, width, n=12):
    """Convex CCW stadium centred at the origin, long axis along the first coordinate."""
    r, h = width / 2, length / 2 - width / 2
    a = np.linspace(-np.pi / 2, np.pi / 2, n)
    right = np.c_[h + r * np.cos(a), r * np.sin(a)]
    return [tuple(p) for p in np.vstack([right, -right])]


def side_prism(poly, c0, c1, frame):
    """mp.prism over a polygon in a local (a, b) plane extruded along c, then rotated into the phone frame."""
    pts, faces = mp.prism(poly, c0, c1)
    a, b, c = pts.T
    rot = {"side": np.c_[c, a, b], "bottom": np.c_[a, -c, b]}[frame]  # both proper rotations
    return rot, faces


def build():
    model, front = build_body()
    plateau = build_plateau()
    u0, u1 = PLATEAU_H - 0.2 * MM, PLATEAU_H + LENS_H
    r = LENS_D / 2
    ring_prof = [(0.0, u0), (r, u0), (r, u1 - 0.4 * MM), (r - 0.4 * MM, u1), (r - 1.65 * MM, u1)]
    ring_prof += [(r - 1.9 * MM, u1 - 0.25 * MM), (0.0, u1 - 0.25 * MM)]
    glass_prof = [(0.0, u1 - 0.28 * MM), (r - 1.88 * MM, u1 - 0.28 * MM), (r - 1.88 * MM, u1 - 0.17 * MM)]
    glass_prof += [(0.0, u1 - 0.17 * MM)]
    lens_ring = merge([back_lathe(ring_prof, x, y) for x, y in LENSES])
    lens_glass = merge([back_lathe(glass_prof, x, y) for x, y in LENSES])

    def disc(x, y, dia, proud=0.07 * MM):
        rr = dia * MM / 2
        return back_lathe(
            [
                (0.0, PLATEAU_H - 0.1 * MM),
                (rr, PLATEAU_H - 0.1 * MM),
                (rr, PLATEAU_H + proud),
                (0.0, PLATEAU_H + proud),
            ],
            x,
            y,
        )

    parts = {"model": model, "front": front, "plateau": plateau, "lens_ring": lens_ring, "lens_glass": lens_glass}
    parts["flash"], parts["sensor"] = disc(*FLASH), disc(*SENSOR)
    parts["mic"] = disc(*MIC, proud=0.01 * MM)
    yi, wi, hi = ISLAND
    isl = [(x * MM, L / 2 - yi * MM + y * MM) for x, y in stadium(wi, hi)]
    parts["island"] = mp.prism(isl, T - 0.03 * MM, T + 0.02 * MM)
    panel = outline(
        W / 2 - PANEL_INSET, L / 2 - PANEL_TOP, -L / 2 + PANEL_INSET, PANEL_R, 2.0, C - PANEL_INSET, NC, 128
    )
    parts["back_panel"] = mp.prism([tuple(p) for p in panel], -0.05 * MM, 0.05 * MM)
    buttons, control = [], None
    for name, (side, y, length) in BUTTONS.items():
        poly = [(L / 2 - y * MM + a, T / 2 + b) for a, b in stadium(length * MM, BUTTON_T)]
        out = W / 2 + (0.03 * MM if name == "camera_control" else BUTTON_PROUD)
        c0, c1 = (W / 2 - 0.6 * MM, out) if side > 0 else (-out, -W / 2 + 0.6 * MM)
        prism = side_prism(poly, c0, c1, "side")
        if name == "camera_control":
            control = prism
        else:
            buttons.append(prism)
    parts["buttons"], parts["camera_control"] = merge(buttons), control
    port = [(a, T / 2 + b) for a, b in stadium(*USB_C)]
    parts["port"] = side_prism(port, L / 2 - 0.3 * MM, L / 2 + 0.02 * MM, "bottom")

    assert mp.signed_volume(*model) + mp.signed_volume(*front) > 0, "body has inward-facing winding"
    for name in (
        "plateau",
        "lens_ring",
        "lens_glass",
        "flash",
        "sensor",
        "mic",
        "island",
        "back_panel",
        "buttons",
        "camera_control",
        "port",
    ):
        assert mp.signed_volume(*parts[name]) > 0, f"{name} has inward-facing winding"

    body_pts = np.concatenate([model[0], front[0]])
    hulls = [hull_points(body_pts, (0.0, T)), hull_points(plateau[0], (-PLATEAU_H, 0.3 * MM))]
    sec = pc.unit_section(2.0)[:: pc.N_THETA // 24]
    for x, y in LENSES:
        cx, cy = W / 2 - x * MM, L / 2 - y * MM
        hulls.append(np.array([(cx + r * px, cy - r * py, -u) for u in (u0, u1) for px, py in sec]))
    return parts, hulls


def hull_points(pts, flat_z):
    """<= HULL_MAX points of one convex collider: the corners of its flat faces at flat_z, then greedily the point
    farthest outside the current hull, skipping points just off a flat face (as make_airpods.hull_points)."""
    hv = pts[ConvexHull(pts).vertices]
    chosen = []
    for z in flat_z:
        flat = np.flatnonzero(np.abs(hv[:, 2] - z) < 1e-7)
        chosen += [int(flat[np.argmax(hv[flat, :2] @ d)]) for d in ((1, 1), (-1, 1), (-1, -1), (1, -1))]
    chosen = list(dict.fromkeys(chosen))
    near = np.zeros(len(hv), bool)
    for z in flat_z:
        d = np.abs(hv[:, 2] - z)
        near |= (d > 1e-7) & (d < FLAT_BAND)
    while len(chosen) < HULL_MAX:
        eq = ConvexHull(hv[chosen]).equations
        out = (hv @ eq[:, :3].T + eq[:, 3]).max(1)
        out[near] = -np.inf
        out[chosen] = -np.inf
        if out.max() <= 1e-7:
            break
        chosen.append(int(np.argmax(out)))
    return hv[chosen]


def self_check(parts, hulls):
    body = np.concatenate([parts["model"][0], parts["front"][0]])
    gaps = []
    for pts, h in ((body, hulls[0]), (parts["plateau"][0], hulls[1])):
        eq = ConvexHull(h).equations
        gaps.append(float((pts @ eq[:, :3].T + eq[:, 3]).max(1).max()))
    assert all(len(h) <= HULL_MAX for h in hulls), "collision hull over the PhysX GPU limit"
    assert max(gaps) < 0.0015, f"collision hull too coarse: {max(gaps) * 1000:.2f} mm"
    return gaps


def surface(name, rgb, rough, metal):
    def material(st):
        inputs = {"diffuse_color_constant": rgb, "reflection_roughness_constant": rough, "metallic_constant": metal}
        return mp.add_material(st, f"/root/_materials/{name}", "OmniPBR", inputs, rgb, 1.0, rough)

    return material


def write_usdz(out, parts, hulls, finish):
    _, alu, metal, panel = finish
    looks = {k: surface(k, *v) for k, v in FIXED.items()}
    for k in BODY_PARTS:
        looks[k] = surface("Aluminium", alu, 0.35, metal)
    looks["back_panel"] = surface("BackGlass", panel, 0.6, 0.0)
    tmp = Path(tempfile.mkdtemp()) / "object.usdc"
    st = Usd.Stage.CreateNew(str(tmp))
    UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(st, 1.0)
    root = mp.add_xform(st, "/root")
    st.SetDefaultPrim(root.GetPrim())
    UsdPhysics.RigidBodyAPI.Apply(root.GetPrim())
    mp.add_xform(st, "/root/visual")
    bound = {}
    for name, (pts, faces) in parts.items():  # model first: loaders read the first Mesh
        mesh = mp.add_mesh(st, f"/root/visual/{name}", pts, faces, mp.vertex_normals(pts, faces))
        key = "Aluminium" if name in BODY_PARTS else name
        if key not in bound:
            bound[key] = looks[name](st)
        UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(bound[key])
    pc.add_collision(st, hulls)
    st.Save()
    zip_writer = getattr(Sdf, "ZipFileWriter", None) or Usd.ZipFileWriter
    zw = zip_writer.CreateNew(str(out))
    zw.AddFile(str(tmp), "object.usdc")
    zw.Save()


def description(uid, color):
    caption = {
        "color": [f"{color} aluminium", "black screen", f"{color} back"],
        "material": ["aluminium", "glass", "aluminium unibody with glass front and back"],
        "size": ["about 16 cm tall", "about 8 cm wide", "under 1 cm thick"],
        "name": ["iPhone 17 Pro Max", "smartphone", "mobile phone"],
        "shape": ["flat rounded rectangle", "full-width camera plateau", "three camera lenses"],
        "function": ["making calls", "taking photos", "being picked up and placed"],
    }
    desc = [
        f"{color} iPhone 17 Pro Max smartphone lying with its black screen up",
        f"large flat {color} phone with a full-width camera plateau and three lenses",
        "smartphone with a dark glass screen, rounded corners and buttons on both sides",
        f"{color} aluminium mobile phone about 16 cm tall and 8 cm wide",
        "thin phone picked up by its edges",
    ]
    return {"uuid": uid, "caption": caption, "description": desc}


def save(rigid_dir, idx, parts, hulls, finish):
    every = np.concatenate([p for p, _ in parts.values()])
    shift = -(every.min(0) + every.max(0)) / 2
    parts = {k: (p + shift, f) for k, (p, f) in parts.items()}
    hulls = [h + shift for h in hulls]
    d = Path(rigid_dir) / CATEGORY / f"{idx:05d}"
    d.mkdir(parents=True, exist_ok=True)
    uid = str(uuid.uuid5(uuid.NAMESPACE_URL, f"robodojo/rigid/{CATEGORY}/{idx}"))
    write_usdz(d / "object.usdz", parts, hulls, finish)
    faces = [f for _, fs in parts.values() for f in fs]
    meta = pc.metadata(uid, dict(pts=every + shift, faces=faces, contact_r=0.03), MASS, shift)
    del meta["physics"]["friction"]  # no measured friction: runtime defaults apply
    meta["visual"] = {"color_parts": list(BODY_PARTS)}  # recolour the aluminium only, the screen stays black
    body = np.concatenate([parts["model"][0], parts["front"][0]])
    ext = np.ptp(body, axis=0) * 1000  # housing without plateau, lenses and buttons
    if idx == 0:
        meta["measurements_mm"] = {
            "高": round(float(ext[1]), 1),
            "宽": round(float(ext[0]), 1),
            "厚": round(float(ext[2]), 2),
        }
    else:
        meta["variant_of"] = f"Rigid/{CATEGORY}/00000"
    (d / "metadata.json").write_text(json.dumps(meta, indent=4, ensure_ascii=False))
    (d / "description.json").write_text(json.dumps(description(uid, finish[0]), indent=4))
    return d, meta, ext


def main(rigid_dir):
    parts, hulls = build()
    gaps = self_check(parts, hulls)
    for idx, finish in enumerate(FINISHES):
        d, meta, ext = save(rigid_dir, idx, parts, hulls, finish)
        print(
            f"{d}: {finish[0]}, size {meta['physics']['size']}, housing {ext.round(2)} mm, {len(hulls)} hulls "
            f"({[len(h) for h in hulls]} points, max gap {max(gaps) * 1000:.2f} mm), "
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
