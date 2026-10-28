"""Fit the islands in front of the counter (four phone photos, reference/island_{front,right,left,end}.jpg) to
hand-picked pixels: the four camera poses, a shared focal length, the island sizes and the water-dispenser bays.

Scene frame (build_scene.py): Z up, metres, origin on the floor under the countertop centre, wall at y = depth / 2,
+Y into the wall. Two islands stand in the aisle, both stadium-shaped in plan with their long side along X:
- the bar: a narrow white top on a bronze stadium pedestal at its left end; its right end rests on a bronze drum
  standing on the green island's top;
- the green island: lower, a white top over a fluted green base on a bronze plinth, inset from the top's edge.

The cameras are anchored on the counter fitted by fit_photo.py (poster corners, shelf and countertop edges, the alcove's
column edges), so the islands share its scale (the microwave width; nothing here was tape-measured). The dispenser bays
are fitted on the right cabinet wall's plane: island_right sees them square on, photo2 (fit_photo.py) only very
obliquely. "points" are corners; "lines" are pixels on a model edge whose ends are hidden; "outlines" are pixels on the
visible edge of one part of a stadium outline (its rim, the slab's lower edge or the floor contact; "x0" / "x1" the end
arcs, "y0" / "y1" the aisle / wall sides), matched to the nearest point of that part's projection; "extremes" are the
leftmost / rightmost pixel of a part's silhouette. Prints the fit and per-pixel errors; writes fit_island.json, read by
build_scene.py.
"""

import json
from pathlib import Path

import cv2
import numpy as np
from scipy.optimize import least_squares

HERE = Path(__file__).resolve().parent
FP = json.loads((HERE / "fit_photo.json").read_text())
F = FP["dims"]
DY = F["depth"] / 2 - 0.30  # fit_photo.py frame -> scene frame
HL, WALL_Y, TOP = F["length"] / 2, F["depth"] / 2, F["top_z"]
RIGHT_FRONT = F["right_front"] + DY
SHELF_FRONT = WALL_Y - F["shelf_depth"]
POSTER_Y = WALL_Y - 0.0125

ANCHORS = {
    "poster_tl": (F["poster_x0"], POSTER_Y, F["poster_z1"]),
    "poster_tr": (F["poster_x1"], POSTER_Y, F["poster_z1"]),
    "poster_bl": (F["poster_x0"], POSTER_Y, F["poster_z0"]),
    "poster_br": (F["poster_x1"], POSTER_Y, F["poster_z0"]),
    "counter_fl": (-HL, -WALL_Y, TOP),
    "counter_fr": (HL, -WALL_Y, TOP),
    "faucet_base": (F["faucet_x"], F["sink_cy"] + DY + F["sink_sy"] / 2 + 0.035, TOP),
}
ANCHOR_LINES = {  # name: (point, direction)
    "shelf_upper": ((0, SHELF_FRONT, F["shelf_z2"]), (1, 0, 0)),
    "shelf_lower": ((0, SHELF_FRONT, F["shelf_z1"]), (1, 0, 0)),
    "counter_front": ((0, -WALL_Y, TOP), (1, 0, 0)),
    "alcove_left_back": ((-HL, WALL_Y, 0), (0, 0, 1)),
    "alcove_right_back": ((HL, WALL_Y, 0), (0, 0, 1)),
    "alcove_right_front": ((HL, RIGHT_FRONT, 0), (0, 0, 1)),
}

DIMS = {  # name: (prior, sigma); loose, they only hold unobserved directions
    "bar_x0": (-1.6, 1.0),
    "bar_x1": (0.9, 1.0),
    "bar_y0": (-1.9, 0.8),
    "bar_y1": (-1.3, 0.8),
    "bar_z": (1.05, 0.15),
    "bar_t": (0.04, 0.02),
    "ped_x0": (-1.5, 1.0),
    "ped_x1": (-0.6, 1.0),
    "ped_w": (0.45, 0.2),
    "drum_x0": (0.0, 1.0),
    "drum_x1": (0.7, 1.0),
    "drum_w": (0.5, 0.2),
    "green_x0": (-0.1, 1.0),
    "green_x1": (3.8, 1.0),
    "green_y0": (-2.3, 0.8),
    "green_y1": (-1.5, 0.8),
    "green_z": (0.9, 0.15),
    "green_t": (0.05, 0.03),
    "green_inset": (0.08, 0.05),
    "disp_x0": (2.97, 0.6),
    "disp_x1": (3.40, 0.6),
    "disp_x2": (3.45, 0.6),
    "disp_x3": (3.85, 0.6),
    "disp_z0": (0.78, 0.3),
    "disp_z1": (1.36, 0.3),
    "focal": (FP["focal_px"], 150.0),
}

# Pixels picked on 2-3x zoomed, gridded crops. Starting poses: camera centre and a point it looks at (by eye).
PHOTOS = {
    "island_front.jpg": {
        "size": (1024, 768),
        "start": ((0.2, -3.6, 1.15), (0.0, 0.0, 0.9)),
        "points": {"poster_tl": (330, 95), "poster_tr": (407, 99), "poster_bl": (331, 212), "poster_br": (406, 220)},
        "lines": {
            "shelf_upper": [(148, 207), (430, 220)],
            "shelf_lower": [(148, 262), (430, 272)],
            "alcove_left_back": [(100, 60), (100, 340)],
            "alcove_right_back": [(441, 120), (441, 340)],
        },
        "outlines": {
            "bar_top:x1": [(802, 382)],
            "bar_bottom:y0": [(10, 426), (300, 414), (750, 393)],
            "drum_bottom:x0": [(605, 420)],
            "drum_bottom:y0": [(680, 425)],
            "drum_bottom:x1": [(760, 420)],
            "green_top:x0": [(560, 410)],
            "green_bottom:y0": [(690, 457), (1020, 437)],
            "green_floor:y0": [(760, 600), (1020, 532)],
            "green_floor:x0": [(600, 585)],
            "ped_floor:y0": [(10, 707), (150, 684)],
            "ped_floor:x1": [(300, 660)],
        },
    },
    "island_right.jpg": {
        "size": (1024, 768),
        "start": ((3.0, -3.3, 1.3), (0.5, 0.0, 1.0)),
        "points": {"poster_tl": (400, 230), "poster_bl": (404, 333)},
        "lines": {
            "shelf_upper": [(280, 342), (450, 341)],
            "shelf_lower": [(280, 387), (450, 386)],
            "alcove_left_back": [(267, 300), (268, 450)],
            "alcove_right_front": [(462, 240), (463, 450)],
        },
        "dispensers": {
            "b0_tl": (689, 381),
            "b0_tr": (751, 381),
            "b0_bl": (691, 473),
            "b0_br": (752, 472),
            "b1_tl": (777, 377),
            "b1_tr": (845, 377),
            "b1_bl": (778, 474),
            "b1_br": (846, 475),
        },
        "outlines": {
            "bar_top:x1": [(439, 477)],
            "bar_bottom:y0": [(225, 502)],
            "bar_bottom:x1": [(430, 490)],
            "drum_bottom:x0": [(230, 523)],
            "drum_bottom:y0": [(320, 527)],
            "drum_bottom:x1": [(403, 522)],
            "bar_top:y1": [(100, 469), (350, 466)],
            "green_top:x0": [(168, 542)],
            "green_top:y1": [(440, 504), (734, 514), (940, 520)],
            "green_bottom:x0": [(205, 569)],
            "green_bottom:y0": [(450, 587), (1000, 645)],
        },
    },
    "island_left.jpg": {
        "size": (768, 1024),
        "start": ((-2.6, -2.3, 1.5), (1.0, -0.8, 0.8)),
        "points": {
            "poster_tl": (167, 255),
            "poster_tr": (230, 262),
            "poster_bl": (172, 390),
            "poster_br": (230, 395),
            "counter_fl": (45, 587),
            "counter_fr": (317, 530),
            "faucet_base": (141, 551),
        },
        "lines": {
            "shelf_upper": [(35, 395), (257, 405)],
            "shelf_lower": [(40, 459), (257, 450)],
            "alcove_right_front": [(319, 290), (318, 520)],
            "alcove_right_back": [(253, 265), (253, 585)],
        },
        "outlines": {
            "bar_bottom:x0": [(259, 654), (365, 663), (490, 654)],
            "ped_floor:x0": [(321, 945), (365, 955), (420, 955)],
            "ped_floor:y0": [(465, 880), (503, 818)],
        },
        "extremes": {"bar_top:x0": ("umin", 250)},
    },
    "island_end.jpg": {
        "size": (1024, 768),
        "start": ((5.0, -2.5, 1.5), (0.0, -1.0, 0.9)),
        "points": {"counter_fr": (552, 376)},
        "lines": {
            "shelf_upper": [(479, 261), (554, 263)],
            "shelf_lower": [(479, 293), (554, 294)],
            "counter_front": [(432, 347), (549, 375)],
            "alcove_right_front": [(557, 160), (557, 340)],
        },
        "outlines": {
            "bar_top:x1": [(399, 358)],
            "bar_bottom:x1": [(393, 367)],
            "drum_bottom:y0": [(278, 390)],
            "drum_bottom:x1": [(320, 393), (366, 390)],
            "green_top:x0": [(217, 383)],
            "green_top:y0": [(313, 440), (400, 483)],
            "green_top:x1": [(580, 503), (693, 483), (673, 460)],
            "green_bottom:y0": [(313, 453), (400, 497)],
            "green_bottom:x1": [(480, 515), (580, 517), (680, 503)],
            "green_floor:y0": [(245, 501), (301, 576), (376, 676)],
            "green_floor:x1": [(426, 717), (501, 745), (570, 742), (620, 726), (651, 695)],
        },
    },
}


def stadium(x0, x1, y0, y1, z, n=24):
    """Outline of a stadium (half-circle ends of radius (y1 - y0) / 2) in the plane z, by part: "x0" / "x1" the left /
    right end arcs, "y0" / "y1" the straight aisle / wall sides; polylines."""
    r = (y1 - y0) / 2
    yc = (y0 + y1) / 2
    xa, xb = x0 + r, max(x1 - r, x0 + r)
    a = np.linspace(-np.pi / 2, np.pi / 2, n)
    parts = {
        "x1": np.c_[xb + r * np.cos(a), yc + r * np.sin(a)],
        "x0": np.c_[xa - r * np.cos(a), yc - r * np.sin(a)],
        "y0": np.array([[xa, y0], [xb, y0]]),
        "y1": np.array([[xa, y1], [xb, y1]]),
    }
    return {k: np.c_[v, np.full(len(v), z)] for k, v in parts.items()}


def outlines(d):
    yc = (d["bar_y0"] + d["bar_y1"]) / 2
    gi = d["green_inset"]
    bar = (d["bar_x0"], d["bar_x1"], d["bar_y0"], d["bar_y1"])
    green = (d["green_x0"], d["green_x1"], d["green_y0"], d["green_y1"])
    return {
        "bar_top": stadium(*bar, d["bar_z"]),
        "bar_bottom": stadium(*bar, d["bar_z"] - d["bar_t"]),
        "ped_floor": stadium(d["ped_x0"], d["ped_x1"], yc - d["ped_w"] / 2, yc + d["ped_w"] / 2, 0.0),
        "drum_bottom": stadium(d["drum_x0"], d["drum_x1"], yc - d["drum_w"] / 2, yc + d["drum_w"] / 2, d["green_z"]),
        "green_top": stadium(*green, d["green_z"]),
        "green_bottom": stadium(*green, d["green_z"] - d["green_t"]),
        "green_floor": stadium(green[0] + gi, green[1] - gi, green[2] + gi, green[3] - gi, 0.0),
    }


def dispensers(d):
    out = {}
    for i, (xa, xb) in enumerate(((d["disp_x0"], d["disp_x1"]), (d["disp_x2"], d["disp_x3"]))):
        for key, x, z in (("tl", xa, "disp_z1"), ("tr", xb, "disp_z1"), ("bl", xa, "disp_z0"), ("br", xb, "disp_z0")):
            out[f"b{i}_{key}"] = (x, RIGHT_FRONT, d[z])
    return out


def intrinsics(d, size):
    return np.array([[d["focal"], 0, size[0] / 2], [0, d["focal"], size[1] / 2], [0, 0, 1]])


def project(k, pose, pts):
    pts = np.asarray(pts, float).reshape(-1, 3)
    return cv2.projectPoints(pts, pose[:3], pose[3:], k, None)[0].reshape(-1, 2)


def poly_dist(uv, px):
    """Distance from pixel `px` to the polyline `uv` (only the part in front of the camera is passed)."""
    a, b = uv[:-1], uv[1:]
    ab = b - a
    t = np.clip(np.einsum("ij,ij->i", np.subtract(px, a), ab) / np.maximum(np.einsum("ij,ij->i", ab, ab), 1e-9), 0, 1)
    return float(np.min(np.linalg.norm(a + t[:, None] * ab - px, axis=1)))


def line_dist(k, pose, p0, direction, px):
    a, b = project(k, pose, [p0, np.add(p0, np.multiply(direction, 5.0))])
    n = np.array([b[1] - a[1], a[0] - b[0]]) / np.linalg.norm(b - a)
    return float(np.dot(np.subtract(px, a), n))


def unpack(x):
    d = dict(zip(DIMS, x[: len(DIMS)]))
    poses = {name: x[len(DIMS) + 6 * i : len(DIMS) + 6 * i + 6] for i, name in enumerate(PHOTOS)}
    return d, poses


def residuals(x, report=None, anchors_only=False):
    d, poses = unpack(x)
    res = [(d[k] - v) / s for k, (v, s) in DIMS.items()]
    outl, disp = outlines(d), dispensers(d)
    for name, obs in PHOTOS.items():
        k, pose = intrinsics(d, obs["size"]), poses[name]
        points = list(obs["points"].items())
        if not anchors_only:
            points += list(obs.get("dispensers", {}).items())
        for key, px in points:
            e = project(k, pose, [ANCHORS[key] if key in ANCHORS else disp[key]])[0] - px
            res += list(e)
            if report is not None:
                report.append((name, key, float(np.hypot(*e))))
        for key, pxs in obs["lines"].items():
            for px in pxs:
                e = line_dist(k, pose, *ANCHOR_LINES[key], px)
                res.append(e)
                if report is not None:
                    report.append((name, key, abs(e)))
        for key, pxs in obs["outlines"].items() if not anchors_only else ():
            shape, part = key.split(":")
            uv = project(k, pose, outl[shape][part])
            for px in pxs:
                e = poly_dist(uv, px)
                res.append(e)
                if report is not None:
                    report.append((name, key, e))
        for key, (which, value) in obs.get("extremes", {}).items() if not anchors_only else ():
            shape, part = key.split(":")
            uv = project(k, pose, outl[shape][part])
            e = {"umin": uv[:, 0].min, "umax": uv[:, 0].max}[which]() - value
            res.append(e)
            if report is not None:
                report.append((name, f"{key} {which}", abs(e)))
    return np.array(res)


def start_pose(eye, target):
    eye, target = np.asarray(eye, float), np.asarray(target, float)
    f = target - eye
    f /= np.linalg.norm(f)
    r = np.cross(f, [0, 0, 1])
    r /= np.linalg.norm(r)
    rot = np.stack([r, np.cross(f, r), f])  # camera x right, y down, z forward
    return np.r_[cv2.Rodrigues(rot)[0].ravel(), -rot @ eye]


def main():
    x0 = np.r_[[v for v, _ in DIMS.values()], *[start_pose(*obs["start"]) for obs in PHOTOS.values()]]
    # the cameras on the counter's anchors first, then everything together
    x0 = least_squares(residuals, x0, kwargs={"anchors_only": True}, loss="soft_l1", f_scale=3.0, max_nfev=200).x
    fit = least_squares(residuals, x0, loss="soft_l1", f_scale=3.0, max_nfev=400)
    d, poses = unpack(fit.x)
    report = []
    residuals(fit.x, report)
    for name, key, e in report:
        print(f"  {name:17s} {key:20s} {e:5.1f} px")
    cams = {}
    for name, pose in poses.items():
        rot = cv2.Rodrigues(pose[:3])[0]
        centre = -rot.T @ pose[3:]
        cams[name] = {
            "rvec": pose[:3].round(5).tolist(),
            "tvec": pose[3:].round(4).tolist(),
            "centre": centre.round(3).tolist(),
        }
        print(f"{name}: camera at {centre.round(2)}")
    for k, v in d.items():
        print(f"{k:12s} {v:7.3f}")
    out = {
        "focal_px": round(d["focal"], 1),
        "dims": {k: round(float(v), 3) for k, v in d.items() if k != "focal"},
        "cameras": cams,
        "max_px": round(max(e for _, _, e in report), 1),
    }
    (HERE / "fit_island.json").write_text(json.dumps(out, indent=2) + "\n")


if __name__ == "__main__":
    main()
