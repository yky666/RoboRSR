"""Photo overlay of the task table: how far the twin's tabletop edges land from the table's edges in the photos.

The phone photos and video frames have SfM cameras (reference/cameras_scene.json, room frame, table at the origin;
the 10-29 ROOM_SHIFT moved the room, not the table). Each top edge of the 1.20 x 0.62 m tabletop (straight part, rounded
corners left out) is projected; at 7 points along it the image edge is the strongest bright-to-dark step along the
outward normal within +-25 px. An edge's offset is the median of its points, kept only when they agree (spread < 4 px;
cups and the tissue pack break some); a view's max_px is its largest kept offset. These cameras were not fitted to the
table, so the offsets are out-of-sample. Not the robot-camera overlay of scripts/scenes/overlay.py.

Usage (numpy, opencv): python overlay_photos.py [--crops crops.jpg]
"""

import argparse
import json
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
VIEWS = ["p0_25a883", "p1_2a4974", "p2_6dda6d", "p4_cf220a", "v0160", "v0260", "v0400"]
SEARCH, SPREAD = 25, 4.0
TS = np.linspace(0.2, 0.8, 7)


def project(cam, points):
    X = np.asarray(points, float) @ np.array(cam["R_cam_from_scene"]).T + cam["t_cam_from_scene"]
    K = np.array([[cam["fx"], 0, cam["cx"]], [0, cam["fy"], cam["cy"]], [0, 0, 1]])
    uv = X @ K.T
    return uv[:, :2] / uv[:, 2:]


def edge_offsets(cam, gray, table):
    """{edge: [offset px of each point]}, positive = the photo's edge lies outside the projected one."""
    hx, hy, z = table["length"] / 2, table["width"] / 2, table["top_z"]
    corners = {"fl": (-hx, -hy), "fr": (hx, -hy), "br": (hx, hy), "bl": (-hx, hy)}  # robot side is -y
    edges = {"front": ("fl", "fr"), "right": ("fr", "br"), "back": ("br", "bl"), "left": ("bl", "fl")}
    centre = project(cam, [(0, 0, z)])[0]
    h, w = gray.shape
    out = {}
    for name, (a, b) in edges.items():
        A, B = np.array([*corners[a], z]), np.array([*corners[b], z])
        pa, pb = project(cam, [A, B])
        d = (pb - pa) / np.linalg.norm(pb - pa)
        n = np.array([-d[1], d[0]])
        n = -n if np.dot(centre - pa, n) > 0 else n
        steps = np.arange(-SEARCH, SEARCH + 0.5, 0.5)
        offs = []
        for p in project(cam, [A + t * (B - A) for t in TS]):
            if not (SEARCH < p[0] < w - SEARCH and SEARCH < p[1] < h - SEARCH):
                continue
            q = (p + steps[:, None] * n).astype(np.float32)
            profile = cv2.remap(gray, q[:, :1], q[:, 1:], cv2.INTER_LINEAR).ravel()
            offs.append(float(steps[np.argmax(-np.gradient(profile))]))
        out[name] = offs
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--crops", type=Path, help="write the projected outline on each view here")
    args = ap.parse_args()
    cams = json.loads((HERE / "reference" / "cameras_scene.json").read_text())
    table = json.loads((HERE / "usd" / "build_params.json").read_text())["table"]
    report, sheets = {}, []
    for view in VIEWS:
        cam = cams[view]
        img = cv2.imread(str(HERE / "reference" / f"{view}.jpg"))
        gray = cv2.GaussianBlur(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY).astype(np.float32), (5, 5), 1.2)
        kept = {}
        for edge, offs in edge_offsets(cam, gray, table).items():
            q1, med, q3 = np.percentile(offs, [25, 50, 75]) if len(offs) >= 3 else (0, None, 99)
            if med is not None and q3 - q1 < SPREAD:
                kept[edge] = round(float(med), 1)
        report[view] = dict(edges_px=kept, max_px=max((abs(v) for v in kept.values()), default=None))
        print(view, json.dumps(report[view]))
        if args.crops:
            hx, hy, z = table["length"] / 2, table["width"] / 2, table["top_z"]
            ring = project(cam, [(-hx, -hy, z), (hx, -hy, z), (hx, hy, z), (-hx, hy, z)]).astype(np.int32)
            cv2.polylines(img, [ring], True, (0, 0, 255), 2, cv2.LINE_AA)
            sheets.append(cv2.resize(img, (960, round(img.shape[0] * 960 / img.shape[1]))))
    if args.crops:
        cv2.imwrite(str(args.crops), np.vstack(sheets), [cv2.IMWRITE_JPEG_QUALITY, 85])
    worst = [r["max_px"] for r in report.values() if r["max_px"] is not None]
    print(f"{len(worst)} views, max {max(worst)} px")


if __name__ == "__main__":
    main()
