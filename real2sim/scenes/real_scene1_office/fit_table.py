"""Fit the task table to its edges in the photos through the SfM cameras, and check it on the video frames.

The cameras come from reference/cameras_scene.json (SfM, room frame, metric at 0.151 m per SfM unit; the table was put
at the origin, axis-aligned, 1.20 x 0.62 x 0.68 m by priors). Each side of the tabletop (straight part, rounded corners
left out) is projected at its outline, the slab's lower edge where that side faces the camera and its top edge where it
faces away; at 9 points along it the photo's edge is the strongest bright-to-dark step along the outward normal within
+-30 px. Sides whose points disagree (cups, the tissue pack, motion blur) are dropped, and a view counts only if the
tabletop centre projects into it (v0040 shows just the left end at its border, the end face in shadow, so the
bright-to-dark step there is the top edge, not the outline). The phone photos p0-p4 fit the
table's length, width and top height, and its pose in the room frame (x, y, yaw), by least squares on the
point-to-edge distances, re-detecting the edges four times; the video frames are left out and give the out-of-sample
overlay: per frame the largest median offset of its kept sides. Writes fit_table.json. build_scene.py takes the sizes
(TABLE); the pose stays here: the room is placed at ROOM_SHIFT by design, not as photographed. Not the robot-camera
overlay of scripts/scenes/overlay.py.

Usage (numpy, scipy, opencv): python fit_table.py
"""

import json
from pathlib import Path

import cv2
import numpy as np
from scipy.optimize import least_squares

HERE = Path(__file__).resolve().parent
CAMS = json.loads((HERE / "reference" / "cameras_scene.json").read_text())
PRIOR = dict(length=1.20, width=0.62, top_z=0.68)
THICK = 0.025
VIEWS = sorted(p.stem for p in (HERE / "reference").glob("*.jpg") if p.stem in CAMS)
FIT = [v for v in VIEWS if v.startswith("p")]
CHECK = [v for v in VIEWS if v.startswith("v")]
SEARCH, TS = 30, np.linspace(0.2, 0.8, 9)
_GRAY = {}


def project(cam, points):
    X = np.asarray(points, float) @ np.array(cam["R_cam_from_scene"]).T + cam["t_cam_from_scene"]
    K = np.array([[cam["fx"], 0, cam["cx"]], [0, cam["fy"], cam["cy"]], [0, 0, 1]])
    uv = X @ K.T
    return uv[:, :2] / uv[:, 2:]


def outline(view, p):
    """{side: (A, B)} 3D ends of each side's outline for p = (x, y, yaw, length, width, top_z)."""
    x, y, yaw, length, width, top = p
    rot = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
    hx, hy = length / 2, width / 2
    eye = np.array(CAMS[view]["center_scene"][:2])
    out = {}
    for side, (a, b, n) in {
        "front": ((-hx, -hy), (hx, -hy), (0, -1)),  # robot side
        "right": ((hx, -hy), (hx, hy), (1, 0)),
        "back": ((hx, hy), (-hx, hy), (0, 1)),
        "left": ((-hx, hy), (-hx, -hy), (-1, 0)),
    }.items():
        a, b, n = rot @ a + (x, y), rot @ b + (x, y), rot @ n
        z = top - THICK if np.dot(n, eye - (a + b) / 2) > 0 else top
        out[side] = np.r_[a, z], np.r_[b, z]
    return out


def normal(cam, A, B, centre):
    pa, pb = project(cam, [A, B])
    d = (pb - pa) / np.linalg.norm(pb - pa)
    n = np.array([-d[1], d[0]])
    return pa, (-n if np.dot(centre - pa, n) > 0 else n)


def detect(view, p):
    """{side: [photo pixels on that side's edge]} found around the outline of p."""
    cam = CAMS[view]
    if view not in _GRAY:
        img = cv2.imread(str(HERE / "reference" / f"{view}.jpg"), cv2.IMREAD_GRAYSCALE).astype(np.float32)
        _GRAY[view] = cv2.GaussianBlur(img, (5, 5), 1.2)
    gray = _GRAY[view]
    h, w = gray.shape
    centre = project(cam, [(p[0], p[1], p[5])])[0]
    steps = np.arange(-SEARCH, SEARCH + 0.5, 0.5)
    out = {}
    for side, (A, B) in outline(view, p).items():
        _, n = normal(cam, A, B, centre)
        hits = []
        for q0 in project(cam, [A + t * (B - A) for t in TS]):
            if SEARCH < q0[0] < w - SEARCH and SEARCH < q0[1] < h - SEARCH:
                q = (q0 + steps[:, None] * n).astype(np.float32)
                profile = cv2.remap(gray, q[:, :1], q[:, 1:], cv2.INTER_LINEAR).ravel()
                hits.append((steps[np.argmax(-np.gradient(profile))], q0))
        if len(hits) < 4:
            continue
        offs = np.array([o for o, _ in hits])
        q1, q3 = np.percentile(offs, [25, 75])
        if q3 - q1 < 6:
            out[side] = [q0 + o * n for o, q0 in hits if q1 - 3 <= o <= q3 + 3]
    return out


def offsets(view, p, found):
    """{side: [signed px of each found pixel from p's outline]}, positive = outside it."""
    cam = CAMS[view]
    centre = project(cam, [(p[0], p[1], p[5])])[0]
    out = {}
    for side, (A, B) in outline(view, p).items():
        if side in found:
            pa, n = normal(cam, A, B, centre)
            out[side] = [float(np.dot(q - pa, n)) for q in found[side]]
    return out


def framed(view, p):
    """Whether the tabletop centre projects into the frame; else only an end of the table shows at the border."""
    cam = CAMS[view]
    u, v = project(cam, [(p[0], p[1], p[5])])[0]
    return 0 <= u < cam["width"] and 0 <= v < cam["height"]


def overlay(views, p):
    """{view: {"sides_px": {side: median offset}, "max_px": ...}} for views with a kept side; views that do not frame
    the table get "unframed": true and do not count."""
    out = {}
    for v in views:
        sides = {s: round(float(np.median(e)), 1) for s, e in offsets(v, p, detect(v, p)).items()}
        if sides:
            out[v] = dict(sides_px=sides, max_px=max(abs(e) for e in sides.values()))
            if not framed(v, p):
                out[v]["unframed"] = True
    return out


def main():
    prior = np.array([0.0, 0.0, 0.0, PRIOR["length"], PRIOR["width"], PRIOR["top_z"]])
    p = prior.copy()
    for _ in range(4):
        found = {v: detect(v, p) for v in FIT}
        p = least_squares(
            lambda x: np.concatenate([e for v in FIT for e in offsets(v, x, found[v]).values()]),
            p,
            loss="soft_l1",
            f_scale=2.0,
        ).x
    out = dict(
        table=dict(length=round(p[3], 3), width=round(p[4], 3), top_z=round(p[5], 3)),
        pose_in_room=dict(x=round(p[0], 4), y=round(p[1], 4), yaw_deg=round(float(np.degrees(p[2])), 2)),
        fit_views=overlay(FIT, p),
        check_views=overlay(CHECK, p),
        check_views_prior=overlay(CHECK, prior),
    )
    framed_views = [r for r in out["check_views"].values() if not r.get("unframed")]
    out["check_framed_views"], out["check_max_px"] = len(framed_views), max(r["max_px"] for r in framed_views)
    (HERE / "fit_table.json").write_text(json.dumps(out, indent=1) + "\n")
    print(json.dumps(out, indent=1))


if __name__ == "__main__":
    main()
