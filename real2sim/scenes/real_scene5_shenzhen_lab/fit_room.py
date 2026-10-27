"""Fit the room around the desk (walls, structural columns, ceiling levels, furniture) to the 2026-10-27 room photos
reference/IMG_4948-4955.jpg, in the desk frame of fit_photo.py. Writes fit_room.json for build_scene.py.

The eight photos were shot from one spot near (-1.2, -4.4) turning on the spot (the fitted eyes of six of them agree
within 0.5 m); IMG_4951 (elevators) is taken elsewhere and only read by eye. By then the desk had been pulled off the
wall and moved ~0.3 m towards the cabinet, so the desk is NOT used here; the scale comes from fixtures that did not
move: the cabinet (registered 75 x 45 x 120 cm, position from fit_photo.py), the three wall outlets left of the old
desk position (back-projected in IMG_4914 with its fitted camera) and the wall jog edge in IMG_4913. Focal length
fixed to fit_photo.py's 773.6 px (same phone, 1024 px). Pixels were read on zoomed crops; ~2-4 px.

Stages (each later stage holds the earlier results fixed):
1. joint: IMG_4955 (back wall square-on), IMG_4948 (NE corner), IMG_4949 (desk row) and the fixed IMG_4913 camera ->
   soffit along the back wall (underside, depth; its fascia is flush with the column faces), perforated ceiling
   height, NE column, east wall, desk row front edge and north end;
2. IMG_4950 from the desk row legs (5 modules, the last one at the railing) -> south soffit edge;
3. IMG_4954 from the back wall soffit fixtures (back-projected in IMG_4955) -> NW column / end of the back wall;
4. IMG_4953 (west wall square-on; eye prior = the shared spot) -> west wall;
5. IMG_4952 from three fixtures on the west wall (fire alarm, sign, switch; from IMG_4953) -> SW corner, end of the
   elevator wall.
Then pixels are back-projected onto known planes (`MEASURE`). Run: see PLAN.md.
"""

import json
from pathlib import Path

import cv2
import numpy as np
from scipy.optimize import least_squares

HERE = Path(__file__).resolve().parent
OLD = json.loads((HERE / "fit_photo.json").read_text())
F = OLD["focal_px"]
WALL_Y = OLD["fixed"]["wall_y"]
CAB = dict(x0=OLD["dims"]["cab_x0"], w=OLD["fixed"]["cab_w"], d=OLD["fixed"]["cab_d"], h=OLD["fixed"]["cab_h"])
OUTLET_X = (-1.905, -1.429, -0.950)  # back-wall outlets, IMG_4914 pixels (119,394) (204,403) (296,414) on y = WALL_Y
MODULE = 1.33  # desk row module pitch, from the IMG_4949 legs at y -0.70, -3.33, -4.66 (stage 1 camera)
N_MODULES = 5  # IMG_4949 shows modules 1-4, IMG_4950 the last two with the person at the last one
SPOT = (-1.3, -4.5, 1.83)  # shared camera spot (stages 1-3 eyes), prior for IMG_4953
# name: (prior, sigma). w = back-wall soffit depth = column depth (flush, see module docstring)
P1 = dict(
    oz=(0.36, 0.03),  # outlet centre height, IMG_4914
    z_s=(2.8, 0.5),  # soffit underside
    z_p=(2.95, 0.5),  # perforated ceiling
    x_jog=(2.0, 0.3),  # NE column west face
    d_col=(0.4, 0.3),  # column depth from the back wall (= soffit depth)
    x_E=(2.8, 0.6),  # east wall
    desk_dx=(0.65, 0.1),  # desk row front edge, distance from the east wall
    desk_yn=(-0.7, 0.5),  # desk row north end
)


def model(p):
    yb = WALL_Y - p["d_col"]
    x0, x1, y0, h = CAB["x0"], CAB["x0"] + CAB["w"], WALL_Y - CAB["d"], CAB["h"]
    xf = p["x_E"] - p["desk_dx"]
    return {
        "cab_tfl": (x0, y0, h), "cab_tfr": (x1, y0, h), "cab_bfl": (x0, y0, 0), "cab_bfr": (x1, y0, 0),
        "cab_fl_lo": (x0, y0, 0), "cab_fr_lo": (x1, y0, 0), "cab_bl_hi": (x0, WALL_Y, h), "cab_bl_lo": (x0, WALL_Y, 0),
        **{f"out{i}": (x, WALL_Y, p["oz"]) for i, x in enumerate(OUTLET_X)},
        "floor_a": (-3, WALL_Y, 0), "floor_b": (1, WALL_Y, 0),
        "ceil_a": (-3, WALL_Y, p["z_s"]), "ceil_b": (1, WALL_Y, p["z_s"]),
        "fasc_lo_a": (-3, yb, p["z_s"]), "fasc_lo_b": (1, yb, p["z_s"]),
        "fasc_hi_a": (-3, yb, p["z_p"]), "fasc_hi_b": (1, yb, p["z_p"]),
        "col_lo": (p["x_jog"], yb, 0), "col_hi": (p["x_jog"], yb, 2.5),  # NE column, convex corner
        "colE_lo": (p["x_E"], yb, 0), "colE_hi": (p["x_E"], yb, 2.5),  # column face meets the east wall
        "colt_a": (p["x_jog"], yb, p["z_p"]), "colt_b": (p["x_E"], yb, p["z_p"]),  # column face top
        "desk_a": (xf, 0, 0.75), "desk_b": (xf, -6, 0.75), "desk_n": (xf, p["desk_yn"], 0.75),
    }  # fmt: skip


# stage 1 pixels. "lines": (pixel on the model edge a-b, a, b)
OBS1 = {
    "IMG_4955.jpg": dict(
        start=((0.3, -4.4, 1.4), (0.3, 0.3, 1.3)),
        points={"out0": (445, 572), "out1": (523, 572), "out2": (601, 572)},
        lines=[
            ((100, 620), "floor_a", "floor_b"), ((650, 620), "floor_a", "floor_b"),
            ((967.3, 480), "cab_bl_hi", "cab_bl_lo"), ((968, 570), "cab_bl_hi", "cab_bl_lo"),
            ((100, 150), "ceil_a", "ceil_b"), ((700, 150), "ceil_a", "ceil_b"),
            ((100, 98), "fasc_lo_a", "fasc_lo_b"), ((700, 98), "fasc_lo_a", "fasc_lo_b"),
            ((100, 79), "fasc_hi_a", "fasc_hi_b"), ((700, 79), "fasc_hi_a", "fasc_hi_b"),
        ],
    ),
    "IMG_4948.jpg": dict(
        start=((-1.5, -3.5, 1.5), (1.5, 0.0, 1.0)),
        points={"cab_tfl": (350, 346.7), "cab_tfr": (487.5, 346.7)},
        lines=[
            ((430, 527), "cab_bfl", "cab_bfr"), ((350, 450), "cab_tfl", "cab_fl_lo"),
            ((346, 420), "cab_bl_hi", "cab_bl_lo"), ((487.5, 450), "cab_tfr", "cab_fr_lo"),
            ((0, 578), "floor_a", "floor_b"), ((330, 530), "floor_a", "floor_b"),
            ((556, 40), "col_lo", "col_hi"), ((555, 160), "col_lo", "col_hi"),
            ((556, 29.5), "colt_a", "colt_b"), ((688, 45), "colt_a", "colt_b"),
            ((693, 60), "colE_lo", "colE_hi"), ((697, 150), "colE_lo", "colE_hi"),
        ],
    ),
    "IMG_4949.jpg": dict(
        start=((0.5, -3.0, 1.4), (3.2, -3.0, 1.0)),
        points={"cab_tfr": (60, 316), "desk_n": (232, 370)},
        lines=[
            ((108, 30), "col_lo", "col_hi"), ((110, 8), "colt_a", "colt_b"), ((231, 40), "colt_a", "colt_b"),
            ((231, 60), "colE_lo", "colE_hi"), ((268, 373), "desk_a", "desk_b"), ((1018, 442), "desk_a", "desk_b"),
        ],
    ),
}  # fmt: skip
FIXED1 = {"IMG_4913.jpeg": dict(lines=[((1019, 112), "col_lo", "col_hi"), ((973, 425), "col_lo", "col_hi")])}


def K(size=(1024, 768)):
    return np.array([[F, 0, size[0] / 2], [0, F, size[1] / 2], [0, 0, 1.0]])


def project(cam, pts, size=(1024, 768)):
    px, _ = cv2.projectPoints(np.array(pts, np.float64).reshape(-1, 3), cam[0], cam[1], K(size), None)
    return px.reshape(-1, 2)


def line_err(cam, a, b, px, size=(1024, 768)):
    pa, pb = project(cam, [a, b], size)
    n = np.array([pa[1] - pb[1], pb[0] - pa[0]])
    return float(np.dot(np.asarray(px) - pa, n / np.linalg.norm(n)))


def start_pose(eye, target):
    eye, fwd = np.array(eye, float), np.subtract(target, eye)
    fwd /= np.linalg.norm(fwd)
    right = np.cross(fwd, [0, 0, 1])
    right /= np.linalg.norm(right)
    rmat = np.stack([right, np.cross(fwd, right), fwd])
    return [*cv2.Rodrigues(rmat)[0].ravel(), *(-rmat @ eye)]


def eye_of(cam):
    return -cv2.Rodrigues(np.asarray(cam[0]))[0].T @ np.asarray(cam[1])


def on_plane(cam, px, normal, c, size=(1024, 768)):
    """Where the ray through pixel px meets the plane normal . x = c."""
    rmat = cv2.Rodrigues(np.asarray(cam[0]))[0]
    eye = -rmat.T @ np.asarray(cam[1])
    d = rmat.T @ np.linalg.inv(K(size)) @ np.array([px[0], px[1], 1.0])
    normal = np.asarray(normal, float)
    return eye + (c - normal @ eye) / (normal @ d) * d


def solve(residual, x0, labels):
    """least squares, then per-observation pixel residuals (labels: names of the pixel residual entries)."""
    sol = least_squares(residual, x0, loss="soft_l1", f_scale=3.0, max_nfev=20000)
    r = residual(sol.x)[: len(labels)] * 2  # residuals are scaled by a 2 px sigma
    return sol.x, {k: round(abs(float(v)), 1) for k, v in zip(labels, r)}


def stage1():
    names, photos = list(P1), list(OBS1)
    old = {n: (np.array(OLD[n]["rvec"]), np.array(OLD[n]["tvec"])) for n in FIXED1}
    labels = []
    for n, o in [*OBS1.items(), *FIXED1.items()]:
        labels += [f"{n}:{k}:{ax}" for k in o.get("points", {}) for ax in "xy"]
        labels += [f"{n}:{a}-{b}@{px[0]:.0f}" for px, a, b in o["lines"]]

    def unpack(x):
        p = dict(zip(names, x[: len(names)]))
        cams = {n: (x[len(names) + 6 * i : len(names) + 6 * i + 3], x[len(names) + 6 * i + 3 : len(names) + 6 * i + 6])
                for i, n in enumerate(photos)}  # fmt: skip
        return p, cams

    def residual(x):
        p, cams = unpack(x)
        pts, r = model(p), []
        for n, o in [*OBS1.items(), *FIXED1.items()]:
            cam, size = (cams[n], (1024, 768)) if n in cams else (old[n], tuple(OLD[n]["size"]))
            for k, px in o.get("points", {}).items():
                r += list((project(cam, [pts[k]], size)[0] - px) / 2)
            r += [line_err(cam, pts[a], pts[b], px, size) / 2 for px, a, b in o["lines"]]
        return np.array(r + [(p[k] - m) / s for k, (m, s) in P1.items()])

    x0 = [m for m, _ in P1.values()] + sum((start_pose(*OBS1[n]["start"]) for n in photos), [])
    x, err = solve(residual, x0, labels)
    p, cams = unpack(x)
    return {k: float(v) for k, v in p.items()}, cams, err


def fit_camera(start, terms, prior=None):
    """One camera from known points / lines. terms: [("pt", xyz, px) | ("line", a, b, px)]; prior: (eye, sigma)."""
    labels = [f"{t[0]}{i}:{ax}" for i, t in enumerate(terms) for ax in ("xy" if t[0] == "pt" else "d")]

    def residual(x):
        cam, r = (x[:3], x[3:6]), []
        for t in terms:
            if t[0] == "pt":
                r += list((project(cam, [t[1]])[0] - t[2]) / 2)
            else:
                r.append(line_err(cam, t[1], t[2], t[3]) / 2)
        if prior:
            r += list((eye_of(cam) - prior[0]) / prior[1])
        return np.array(r)

    x, err = solve(residual, start_pose(*start), labels)
    return (x[:3], x[3:6]), err


def main():
    p, cams, err = stage1()
    errs = {"stage1": err}
    zs, zp, ycol = p["z_s"], p["z_p"], WALL_Y - p["d_col"]
    xf = p["x_E"] - p["desk_dx"]
    up = (0, 0, 1)

    # stage 2: IMG_4950 from the desk legs (front edge pixels at the legs of modules 4, 5 and the south end)
    def front(x):
        return 520 + (485 - 520) * (x - 59) / (353 - 59)

    legs = {62: 3, 353: 4, 556: 5}
    terms = [("pt", (xf, p["desk_yn"] - MODULE * k, 0.75), (u, front(u))) for u, k in legs.items()]
    terms += [("line", (p["x_E"], 0, zp), (p["x_E"], -9, zp), px) for px in ((100, 28), (500, 105))]
    cam2, errs["IMG_4950.jpg"] = fit_camera(((0.5, -3.5, 1.5), (3, -6.5, 1.2)), terms)
    cams["IMG_4950.jpg"] = cam2
    y_sf = on_plane(cam2, (555, 117), (1, 0, 0), p["x_E"])[1]  # south soffit fascia meets the east wall

    # stage 3: IMG_4954 from the back-wall soffit fixtures, back-projected in IMG_4955
    c55 = cams["IMG_4955.jpg"]
    fixtures = {
        k: on_plane(c55, px, up, zs) for k, px in {"smoke": (150, 130), "dl": (240, 125), "dome": (325, 132)}.items()
    }
    terms = [("pt", fixtures[k], px) for k, px in {"smoke": (775, 91), "dl": (862, 74), "dome": (960, 66)}.items()]
    terms += [("line", (-9, WALL_Y, 0), (3, WALL_Y, 0), px) for px in ((500, 490), (700, 535), (950, 590))]
    terms += [("line", (-9, ycol, zp), (3, ycol, zp), px) for px in ((450, 127), (783, 66))]
    cams["IMG_4954.jpg"], errs["IMG_4954.jpg"] = fit_camera(((-1, -4.4, 1.7), (-4, 0, 1.2)), terms)
    x_nw = on_plane(cams["IMG_4954.jpg"], (400, 470), up, 0)[0]  # back wall meets the NW column, at the floor

    # stage 4: IMG_4953 (west wall square-on), eye prior at the shared spot, west wall x as a 7th unknown
    def residual4(x):
        cam, xw = (x[:3], x[3:6]), x[6]
        r = [line_err(cam, (xw, -12, 0), (xw, 3, 0), px) / 2 for px in ((20, 464), (600, 487))]
        r += [line_err(cam, (xw, -12, zp), (xw, 3, zp), px) / 2 for px in ((300, 156), (1000, 152))]
        return np.array(r + list((eye_of(cam) - SPOT) / np.array([0.4, 0.4, 0.1])))

    x, errs["IMG_4953.jpg"] = solve(residual4, start_pose((-1.3, -4.5, 1.8), (-8, -4.5, 1.2)) + [-8.0], list("abcd"))
    cams["IMG_4953.jpg"], x_w = (x[:3], x[3:6]), float(x[6])
    c53 = cams["IMG_4953.jpg"]
    west = {
        k: on_plane(c53, px, (1, 0, 0), x_w)
        for k, px in {"fire": (215, 252), "sign": (218, 338), "switch": (105, 318)}.items()
    }

    # stage 5: IMG_4952 from the three west-wall fixtures
    terms = [
        ("pt", west[k], px) for k, px in {"fire": (946.5, 248.5), "sign": (943, 340), "switch": (831.5, 318.5)}.items()
    ]
    terms += [("line", (x_w, -12, 0), (x_w, 0, 0), px) for px in ((760, 447), (1000, 492))]
    terms += [("line", (x_w, -12, zs), (x_w, -7.3, zs), px) for px in ((760, 190), (1000, 154))]
    cams["IMG_4952.jpg"], errs["IMG_4952.jpg"] = fit_camera(((-2, -6, 1.6), (-8, -9, 1.2)), terms)
    c52 = cams["IMG_4952.jpg"]
    y_s = float(on_plane(cam2, (680, 300), (1, 0, 0), p["x_E"])[1] + on_plane(c52, (653, 508), up, 0)[1]) / 2

    def m(photo, px, normal, c):
        return [round(float(v), 3) for v in on_plane(cams[photo], px, normal, c)]

    # back-projections onto known planes: (photo, pixel, plane normal, plane offset)
    measure = {
        "sw_corner": m("IMG_4952.jpg", (718.5, 437.5), (1, 0, 0), x_w),  # west wall meets the south-west wall
        "s_wall_end": m("IMG_4952.jpg", (653, 508), up, 0),  # west end of the elevator wall, at the floor
        "kt3_east": m("IMG_4952.jpg", (160, 300), (0, 1, 0), y_s),  # KT3 surround, its east (image right) edge
        "kt4_west": m("IMG_4950.jpg", (840, 300), (0, 1, 0), y_s),  # KT4 surround, its west (image left) edge
        "se_corner": m("IMG_4950.jpg", (680, 300), (0, 1, 0), y_s),
        "rail_e": [m("IMG_4950.jpg", px, up, 0) for px in ((617, 578), (710, 620), (838, 670), (875, 690))],
        "rail_w": [m("IMG_4952.jpg", px, up, 0) for px in ((388, 667), (340, 622), (300, 585), (212, 718))],
        "turnstile_post": m("IMG_4952.jpg", (175, 755), up, 0),
        "downlights_n": [m("IMG_4955.jpg", px, up, zs) for px in ((240, 125), (660, 135), (1015, 130))]
        + [m("IMG_4954.jpg", px, up, zs) for px in ((612, 120), (421, 153))],
        "led_strips": [m("IMG_4955.jpg", px, up, zp) for px in ((62, 2), (130, 42), (895, 2), (845, 50))]
        + [m("IMG_4953.jpg", px, up, zp) for px in ((210, 98), (1000, 77))],
        "slot_diffuser": [m("IMG_4955.jpg", px, up, zp) for px in ((548, 10), (552, 70))],
        "shelf_front": [m("IMG_4953.jpg", px, up, 0) for px in ((640, 505), (820, 510))],
        "shelf_top": [m("IMG_4953.jpg", px, (1, 0, 0), x_w + 0.5) for px in ((642, 277), (828, 277))],
        "shelf_beam": [m("IMG_4953.jpg", (700, y), (1, 0, 0), x_w + 0.5) for y in (330, 383, 432, 485)],
        "desk_legs": [m("IMG_4949.jpg", (u, 373 + (442 - 373) * (u - 268) / 750), up, 0.75) for u in (249, 607, 891)],
        "lift_desk": [m("IMG_4954.jpg", px, up, 0.73) for px in ((345, 399), (507, 407), (470, 415))],
        "chairs": {
            "desk1": m("IMG_4948.jpg", (595, 560), up, 0), "desk2": m("IMG_4949.jpg", (420, 560), up, 0),
            "desk3": m("IMG_4949.jpg", (715, 585), up, 0), "desk5": m("IMG_4950.jpg", (420, 625), up, 0),
            "lift_desk": m("IMG_4954.jpg", (340, 515), up, 0),
        },
    }  # fmt: skip
    out = dict(
        params={k: round(v, 3) for k, v in p.items()}
        | dict(
            y_soffit_s=round(float(y_sf), 3),
            x_nw=round(float(x_nw), 3),
            x_W=round(float(x_w), 3),
            y_S=round(y_s, 3),
            module=MODULE,
            n_modules=N_MODULES,
        ),  # fmt: skip
        measure=measure,
        cameras={
            n: dict(
                eye=[round(float(v), 3) for v in eye_of(c)],
                rvec=[float(v) for v in c[0]],
                tvec=[float(v) for v in c[1]],
            )
            for n, c in cams.items()
        },  # fmt: skip
        residual_px=errs,
    )
    (HERE / "fit_room.json").write_text(json.dumps(out, indent=1), encoding="utf-8")
    print(json.dumps(out["params"], indent=1))
    print({n: c["eye"] for n, c in out["cameras"].items()})
    print({n: max(e.values()) for n, e in errs.items()})


if __name__ == "__main__":
    main()
