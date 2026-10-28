"""Paint texture of soda_can/00006 unwrapped from its reference photos (numpy, scipy, PIL only).

Per photo: the can's silhouette lines and the front arcs of the paint top (first red pixel under the seam) and bottom
give an ellipse per paint height t in [0, 1]; texel (alpha, t) is read at x = cx(t) + a(t) sin(alpha - cam),
y = yc(t) + b(t) cos(alpha - cam). cam is the photo's azimuth in the can frame (alpha CCW from +X), set from the
printed content (logo front at -Y, nutrition panel and barcode at 120-160 deg). Views are blended with cos^4 weights
after matching their paint red; glare (bright, unsaturated) is down-weighted. The blend is then posterized to the
can's four flat inks (red, black, yellow, white), which removes the photos' shading, glare and blur halos; the side
near +X that no photo faces is filled by mirroring its neighbours. Row 0 of the image is the paint top,
u = alpha / 2pi. The same design on the web (cans.ch ref 125284, regular-Coke twin of this Coke Zero can, see
$RUN/web/sources.txt) was only used to check the layout: its images are lower resolution than these photos.

    python scripts/objects/soda_can_label.py OUT.png    # preview the texture
"""

from pathlib import Path
import sys

import numpy as np
from PIL import Image
from scipy import ndimage

REF = Path(__file__).resolve().parents[2] / "real2sim/objects/Rigid/soda_can/00006/reference"
VIEWS = {  # photo: (azimuth of the camera in deg, region of interest x0, y0, x1, y1 in pixels)
    "4ad465c2034b1b4d5c4c098abe624b9c-1.jpg": (270.0, (280, 330, 710, 1080)),  # front: logo, 无糖, trophy
    "99abdcea69ea3316b7fe65428201d6a5.jpg": (190.0, (260, 230, 700, 970)),  # nutrition panel and logo
    "d90100d8f491e3ee472173d0c27d5bbe-1.jpg": (121.0, (140, 250, 680, 1120)),  # back: nutrition, barcode
}
SIZE = (2048, 1552)  # texture px: circumference 181 mm x painted height 138 mm


def silhouette(im, roi):
    hsv = np.asarray(im.convert("HSV")).astype(float) / 255
    h, s, v = hsv[..., 0], hsv[..., 1], hsv[..., 2]
    bg = ((s < 0.2) & (v > 0.45)) | ((h > 0.5) & (h < 0.75) & (s > 0.15))
    x0, y0, x1, y1 = roi
    m = np.zeros(bg.shape, bool)
    m[y0:y1, x0:x1] = ~bg[y0:y1, x0:x1]
    m = ndimage.binary_opening(m, iterations=2)
    lab, n = ndimage.label(m)
    m = lab == 1 + int(np.argmax(ndimage.sum(m, lab, range(1, n + 1))))
    m = ndimage.binary_fill_holes(ndimage.binary_closing(m, iterations=6))
    red = ((h < 0.06) | (h > 0.94)) & (s > 0.45) & (v > 0.25)
    return m, red


def ellipses(m, red):
    """Silhouette lines and the paint top / bottom front arcs -> (cx, a)(y), (yc, b) at the top and bottom."""
    ys = np.nonzero(m.any(1))[0]
    span = ys[-1] - ys[0]
    yy = ys[(ys > ys[0] + 0.15 * span) & (ys < ys[-1] - 0.12 * span)]
    left = np.polyfit(yy, [np.argmax(m[y]) for y in yy], 1)
    right = np.polyfit(yy, [m.shape[1] - 1 - np.argmax(m[y, ::-1]) for y in yy], 1)

    def ca(y):
        xl, xr = np.polyval(left, y), np.polyval(right, y)
        return (xl + xr) / 2, (xr - xl) / 2

    cx0, a0 = ca(ys[len(ys) // 2])
    top, bot = [], []
    for x in range(int(cx0 - 0.6 * a0), int(cx0 + 0.6 * a0)):
        col = np.nonzero(m[:, x])[0]
        if not len(col):
            continue
        r = np.nonzero(red[col[0] : col[0] + int(0.3 * span), x])[0]
        if len(r):
            top.append((x, col[0] + r[0]))
        bot.append((x, col[-1]))

    def arc(pts):
        pts = np.array(pts, float)
        for _ in range(3):
            cx, a = ca(pts[:, 1].min())
            A = np.c_[np.ones(len(pts)), np.sqrt(np.clip(1 - ((pts[:, 0] - cx) / a) ** 2, 0, 1))]
            sol = np.linalg.lstsq(A, pts[:, 1], rcond=None)[0]
            res = np.abs(pts[:, 1] - A @ sol)
            pts = pts[res < max(3.0, 2.5 * np.median(res))]
        return sol

    return ca, arc(top), arc(bot)


def sample(im, ca, top, bot, cam):
    tw, th = SIZE
    alpha = (np.arange(tw) + 0.5) / tw * 2 * np.pi
    t = (np.arange(th) + 0.5) / th
    yc, b = bot[0] + t * (top[0] - bot[0]), bot[1] + t * (top[1] - bot[1])
    cx, a = ca(yc)
    phi = alpha[None, :] - cam
    X, Y = cx[:, None] + a[:, None] * np.sin(phi), yc[:, None] + b[:, None] * np.cos(phi)
    out = np.stack([ndimage.map_coordinates(im[..., k], [Y, X], order=1, mode="nearest") for k in range(3)], -1)
    w = np.clip((np.cos(phi) - 0.04) / 0.96, 0, 1) ** 4 * np.ones_like(X)
    return out[::-1], w[::-1]


def hsv_of(img):
    return np.asarray(Image.fromarray((np.clip(img, 0, 1) * 255).astype(np.uint8)).convert("HSV")).astype(float) / 255


def label(out):
    """Write the unwrapped paint texture to `out` (PNG)."""
    tw, th = SIZE
    tex, wsum, ref = np.zeros((th, tw, 3)), np.zeros((th, tw)), None
    for name, (cam, roi) in VIEWS.items():
        im = Image.open(REF / name).convert("RGB")
        ca, top, bot = ellipses(*silhouette(im, roi))
        img, w = sample(np.asarray(im).astype(float) / 255, ca, top, bot, np.radians(cam))
        hsv = hsv_of(img)
        redm = ((hsv[..., 0] < 0.05) | (hsv[..., 0] > 0.95)) & (hsv[..., 1] > 0.6) & (w > 0.1)
        med = np.median(img[redm], axis=0)
        ref = med if ref is None else ref
        img = np.clip(img * ref / med, 0, 1)
        w = w * np.exp(-6 * np.clip(hsv[..., 2] - hsv[..., 1] - 0.25, 0, None))
        tex += img * w[..., None]
        wsum += w
    tex /= np.maximum(wsum, 1e-9)[..., None]
    hole = wsum < 2e-3
    cols = np.arange(tw)
    for r in range(th):
        ok = ~hole[r]
        if ok.any() and not ok.all():
            xs = np.r_[cols[ok] - tw, cols[ok], cols[ok] + tw]
            for k in range(3):
                tex[r, hole[r], k] = np.interp(cols[hole[r]], xs, np.tile(tex[r, ok, k], 3))
    soft = np.stack([ndimage.gaussian_filter(tex[..., k], (14, 3), mode="wrap") for k in range(3)], -1)
    hm = ndimage.gaussian_filter(hole.astype(float), 8, mode="wrap")[..., None]
    tex = tex * (1 - hm) + soft * hm
    tex = flatten(tex)
    hsv = hsv_of(tex)
    yellow = ((hsv[..., 0] > 0.08) & (hsv[..., 0] < 0.2) & (hsv[..., 1] > 0.3))[..., None]
    unseen = hm * np.where(yellow & (np.arange(th) >= 0.62 * th)[:, None, None], 0.0, 1.0)  # unseen side: paint red
    unseen[: int(0.07 * th)] = 1.0  # neck above the FIFA text: plain red (the photos only show dark shading there)
    tex = tex * (1 - unseen) + np.array(INKS["red"]) * unseen
    Image.fromarray((np.clip(tex, 0, 1) * 255).astype(np.uint8)).save(out)
    return out


INKS = {"red": (0.72, 0.03, 0.06), "black": (0.06, 0.05, 0.06), "yellow": (0.95, 0.76, 0.06)}


def flatten(tex):
    """Snap confidently red (incl. dark-shaded and pink-glare red), yellow and black pixels to the flat inks; white
    print and the anti-aliased edges keep the photo pixels."""
    hsv = hsv_of(tex)
    h, s, v = hsv[..., 0], hsv[..., 1], hsv[..., 2]
    redh = (h < 0.05) | (h > 0.93)
    neck = (np.arange(len(h)) < 0.12 * len(h))[:, None]
    masks = {
        "red": redh & (((s > 0.6) & (v > 0.35)) | (neck & (s > 0.12) & (v > 0.6))),  # neck: pink glare streaks
        "yellow": (h > 0.08) & (h < 0.2) & (s > 0.5) & (v > 0.55),
    }
    for name, m in masks.items():
        m = ndimage.gaussian_filter(m.astype(float), 0.7)[..., None]
        tex = tex * (1 - m) + np.array(INKS[name]) * m
    return tex


if __name__ == "__main__":
    label(sys.argv[1])
