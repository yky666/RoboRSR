"""Label artwork for make_plastic_bottle.py (numpy + PIL only, so it can be previewed without pxr).

draw_label: wrap-around label (00000-00005). The image is the label unrolled: u (left to right) runs counter-clockwise
around the bottle seen from above, which is left to right for a viewer outside; v runs bottom to top. Bottom `share`
of the height is the label's main colour, the rest its second colour. Front panel: picture, brand, Latin brand,
product and volume; back panel: composition table, barcode, fine print and a QR code.

Square tea bottles (00006-00012) carry three separate labels, each image as seen from outside (left to right, top to
bottom): a front panel, a back panel and a neck band. photo_panel / photo_band rectify them from the registration
photos (00006); front_panel / back_panel / neck_band draw generalized ones (00007-00012). two_sided appends the
label's reverse, which shows through the clear bottle.

Everything drawn is generic or fictional except the registered twins' brand words (00000, 00006).
"""

import os
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFilter, ImageFont

# (file name, face index) per role; Noto CJK collections hold JP, KR, SC, TC, HK faces, SC is index 2
FONTS = {
    "brand": [("NotoSerifCJK-Bold.ttc", 2), ("Songti.ttc", 0), ("STHeiti Medium.ttc", 0)],
    "bold": [("NotoSansCJK-Bold.ttc", 2), ("STHeiti Medium.ttc", 0)],
    "regular": [("NotoSansCJK-Regular.ttc", 2), ("STHeiti Light.ttc", 0)],
}
FONT_DIRS = [
    os.environ.get("CJK_FONT_DIR"),
    "/usr/share/fonts/opentype/noto",
    "/System/Library/Fonts",
    "/System/Library/Fonts/Supplemental",
]
WATER_TABLE = [
    ("偏硅酸", "≥1.8 mg/L"),
    ("钙", "≥400 μg/L"),
    ("钾", "≥35 μg/L"),
    ("钠", "≥80 μg/L"),
    ("镁", "≥50 μg/L"),
    ("pH值 (25℃)", "7.3±0.5"),
]
NUTRITION_TABLE = [("能量", "180 kJ"), ("蛋白质", "0 g"), ("脂肪", "0 g"), ("碳水化合物", "10.6 g"), ("钠", "20 mg")]
FINE_PRINT = [
    "配料：{ingredients}",
    "保质期：{shelf}",
    "贮存条件：避免阳光直射，常温保存",
    "生产日期：见瓶盖",
    "执行标准：{standard}",
    "产地：中国",
    "开启后请尽快饮用",
    "客服热线：400-000-0000",
]


def font_path(role):
    for d in filter(None, FONT_DIRS):
        for name, index in FONTS[role]:
            p = Path(d) / name
            if p.exists():
                return str(p), index
    raise FileNotFoundError(
        f"no CJK font for '{role}' ({[n for n, _ in FONTS[role]]}) in {FONT_DIRS}: set CJK_FONT_DIR, e.g. to the "
        "fonts-noto-cjk files unpacked with `apt-get download fonts-noto-cjk && dpkg -x fonts-noto-cjk_*.deb x`"
    )


def font(role, px):
    path, index = font_path(role)
    return ImageFont.truetype(path, max(int(px), 6), index=index)


def srgb(linear):
    return tuple(int(round(255 * min(max(c, 0.0), 1.0) ** (1 / 2.2))) for c in linear)


def fit_text(draw, text, role, box_w, box_h, spacing=0.0):
    """Largest font of `role` whose text (with letter spacing in em) fits box_w x box_h."""
    px = box_h
    while px > 6:
        f = font(role, px)
        w = text_width(draw, text, f, spacing)
        b = draw.textbbox((0, 0), text, font=f)
        if w <= box_w and b[3] - b[1] <= box_h:
            return f
        px *= 0.93
    return font(role, 6)


def text_width(draw, text, f, spacing):
    return sum(draw.textlength(ch, font=f) for ch in text) + spacing * f.size * (len(text) - 1)


def draw_text(draw, text, f, cx, top, fill, spacing=0.0, anchor="c"):
    """Draw `text` with letter spacing; cx is the centre (anchor c) or left edge (anchor l); top is the ink top."""
    w = text_width(draw, text, f, spacing)
    x = cx - w / 2 if anchor == "c" else cx
    y = top - draw.textbbox((0, 0), text, font=f)[1]
    for ch in text:
        draw.text((x, y), ch, font=f, fill=fill)
        x += draw.textlength(ch, font=f) + spacing * f.size


def ridge(rng, n, base, amp, rough):
    x = np.linspace(0, 1, n)
    y = np.zeros(n)
    for k in range(1, 7):
        y += amp / k**rough * np.sin(2 * np.pi * (k * x * rng.uniform(0.6, 1.4) + rng.uniform(0, 1)))
    return base - np.abs(y)


def picture(kind, w, h, main, accent, rng):
    """Front-panel picture, RGB w x h, in the label's main colour with accent highlights."""
    img = Image.new("RGB", (w, h))
    px = np.zeros((h, w, 3), np.float32)
    yy = np.linspace(0, 1, h)[:, None, None]
    if kind == "landscape":  # sky, three mountain ridges, a lake with their reflection
        px[:] = (1 - yy) * np.array([150, 196, 232]) + yy * np.array([236, 244, 250])
        img = Image.fromarray(px.astype(np.uint8))
        d = ImageDraw.Draw(img)
        shore = int(0.68 * h)
        layers = [(0.42, 0.20, (122, 150, 176)), (0.52, 0.16, (70, 110, 120)), (0.62, 0.10, (32, 78, 64))]
        ridges = []
        for base, amp, col in layers:
            y = ridge(rng, w, base * h, amp * h, 1.2)
            ridges.append((y, col))
            d.polygon([(0, shore)] + [(i, float(v)) for i, v in enumerate(y)] + [(w - 1, shore)], fill=col)
        lake = np.array(img).astype(np.float32)
        refl = lake[max(0, 2 * shore - h) : shore][::-1]
        refl = refl[: h - shore]
        water = np.linspace(1, 0.55, h - shore)[:, None, None]
        lake[shore:] = refl * 0.55 * water + np.array([120, 170, 200]) * (1 - 0.55 * water)
        img = Image.fromarray(np.clip(lake, 0, 255).astype(np.uint8)).filter(ImageFilter.GaussianBlur(0.6))
        d = ImageDraw.Draw(img)
        for _ in range(w // 3):  # ripples
            x, y = rng.uniform(0, w), rng.uniform(shore + 2, h)
            d.line([(x, y), (x + rng.uniform(4, 14), y)], fill=(210, 230, 240), width=1)
        return img
    hi = np.array(main) * 0.35 + 255 * 0.65
    px[:] = (1 - yy) * hi + yy * np.array(main)
    img = Image.fromarray(px.astype(np.uint8))
    d = ImageDraw.Draw(img, "RGBA")
    if kind == "bubbles":
        for _ in range(40):
            r = rng.uniform(0.02, 0.12) * h
            x, y = rng.uniform(0, w), rng.uniform(0, h)
            d.ellipse([x - r, y - r, x + r, y + r], outline=(255, 255, 255, 200), width=max(2, int(r / 6)))
            d.ellipse([x - r * 0.5, y - r * 0.6, x - r * 0.2, y - r * 0.3], fill=(255, 255, 255, 180))
    elif kind == "fruit":  # orange slices
        for _ in range(4):
            r = rng.uniform(0.22, 0.34) * h
            x, y = rng.uniform(0.15, 0.85) * w, rng.uniform(0.3, 0.8) * h
            d.ellipse([x - r, y - r, x + r, y + r], fill=(240, 150, 30, 255), outline=(250, 230, 160, 255), width=6)
            for a in np.linspace(0, 2 * np.pi, 10, endpoint=False):
                d.line([(x, y), (x + 0.85 * r * np.cos(a), y + 0.85 * r * np.sin(a))], fill=(252, 220, 140), width=3)
            d.ellipse([x - 0.12 * r, y - 0.12 * r, x + 0.12 * r, y + 0.12 * r], fill=(252, 230, 170, 255))
    elif kind == "leaves":  # tea leaves: almond shapes with a midrib, in greens over the gradient
        for _ in range(12):
            length, width = rng.uniform(0.30, 0.55) * h, rng.uniform(0.10, 0.16) * h
            x, y, a = rng.uniform(0.1, 0.9) * w, rng.uniform(0.2, 0.9) * h, rng.uniform(0, np.pi)
            t = np.linspace(-1, 1, 24)
            u, v = t * length / 2, np.sqrt(1 - t * t) * width / 2
            ca, sa = np.cos(a), np.sin(a)
            outline = [(x + p * ca - q * sa, y + p * sa + q * ca) for p, q in zip(u, v)]
            outline += [(x + p * ca + q * sa, y + p * sa - q * ca) for p, q in zip(u[::-1], v[::-1])]
            green = (int(rng.uniform(40, 90)), int(rng.uniform(120, 170)), int(rng.uniform(50, 80)), 230)
            d.polygon(outline, fill=green)
            d.line([outline[0], outline[23]], fill=(220, 240, 200, 200), width=2)
    elif kind == "stripes":
        for k in range(-6, 12):
            x0 = k * w / 6
            col = (*accent, 200) if k % 2 else (255, 255, 255, 90)
            d.polygon([(x0, h), (x0 + w / 12, h), (x0 + w / 12 + h * 0.6, 0), (x0 + h * 0.6, 0)], fill=col)
    return img


def barcode(draw, x0, y0, w, h, digits, ink, rng):
    bits = [1, 0, 1] + list(rng.integers(0, 2, 42)) + [0, 1, 0, 1, 0] + list(rng.integers(0, 2, 42)) + [1, 0, 1]
    bw = w / len(bits)
    for i, b in enumerate(bits):
        if b:
            draw.rectangle([x0 + i * bw, y0, x0 + (i + 1) * bw - 0.2, y0 + h * 0.8], fill=ink)
    f = fit_text(draw, digits, "regular", w, h * 0.17)
    draw_text(draw, digits, f, x0 + w / 2, y0 + h * 0.83, ink)


def qr(draw, x0, y0, s, ink, rng, n=25):
    m = rng.integers(0, 2, (n, n))
    for fx, fy in ((0, 0), (n - 7, 0), (0, n - 7)):  # finder patterns
        m[fy : fy + 7, fx : fx + 7] = 1
        m[fy + 1 : fy + 6, fx + 1 : fx + 6] = 0
        m[fy + 2 : fy + 5, fx + 2 : fx + 5] = 1
    c = s / n
    for j, i in zip(*np.nonzero(m)):
        draw.rectangle([x0 + i * c, y0 + j * c, x0 + (i + 1) * c - 0.5, y0 + (j + 1) * c - 0.5], fill=ink)


def draw_label(art, main, second, share, size, seed):
    """Label image (PIL RGB, size = (w, h)). main / second: linear RGB of the bottom / top colour bands."""
    w, h = size
    rng = np.random.default_rng(seed)
    c_main, c_second = srgb(main), srgb(second)
    img = Image.new("RGB", size, c_second)
    d = ImageDraw.Draw(img)
    split = int(round(h * (1 - share)))
    d.rectangle([0, split, w, h], fill=c_main)
    trim = srgb(art.get("trim", main))
    d.rectangle([0, 0, w, int(0.022 * h)], fill=trim)
    d.rectangle([0, split - int(0.012 * h), w, split], fill=trim)
    low, top = h - split, split

    # front panel, centred at u = 0.38
    fx0, fx1 = 0.20 * w, 0.56 * w
    cx = (fx0 + fx1) / 2
    pic_w, pic_h = int(0.15 * w), int(top * 0.80 + low * 0.30)
    pic = picture(art["picture"], pic_w, pic_h, c_main, c_second, rng)
    px0, py0 = int(0.035 * w), int(0.06 * h)
    d.rectangle([px0 - 6, py0 - 6, px0 + pic_w + 6, py0 + pic_h + 6], fill=(250, 250, 248))
    img.paste(pic, (px0, py0))
    f = fit_text(d, art["brand"], "brand", (fx1 - fx0) * 0.92, low * 0.46, spacing=0.06)
    draw_text(d, art["brand"], f, cx, split + low * 0.10, c_second, spacing=0.06)
    f = fit_text(d, art["latin"], "bold", (fx1 - fx0) * 0.80, low * 0.11, spacing=0.25)
    draw_text(d, art["latin"], f, cx, split + low * 0.64, c_second, spacing=0.25)
    f = fit_text(d, art["tagline"], "regular", (fx1 - fx0) * 0.70, low * 0.08)
    draw_text(d, art["tagline"], f, cx, split + low * 0.82, c_second)
    f = fit_text(d, art["product"], "bold", (fx1 - fx0) * 0.50, top * 0.30, spacing=0.12)
    draw_text(d, art["product"], f, fx0 + 0.01 * (fx1 - fx0), top * 0.28, c_main, spacing=0.12, anchor="l")
    f = fit_text(d, art["volume"], "bold", (fx1 - fx0) * 0.30, top * 0.30)
    draw_text(d, art["volume"], f, fx1 - text_width(d, art["volume"], f, 0), top * 0.28, c_main, anchor="l")
    f = fit_text(d, art["note"], "regular", (fx1 - fx0) * 0.9, top * 0.12)
    draw_text(d, art["note"], f, cx, top * 0.72, c_main)

    # back panel: composition table and barcode on the top band, fine print and QR code on the bottom band
    bx0, bx1 = 0.62 * w, 0.98 * w
    tx1 = bx0 + 0.21 * w
    rows = WATER_TABLE if art["table"] == "water" else NUTRITION_TABLE
    title = "饮用天然水物质成分表" if art["table"] == "water" else "营养成分表 (每100mL)"
    ink = c_main if sum(c_main) < 600 else (40, 40, 40)
    f = fit_text(d, title, "bold", tx1 - bx0, top * 0.13)
    draw_text(d, title, f, bx0, top * 0.10, ink, anchor="l")
    rh = top * 0.66 / len(rows)
    fr = font("regular", rh * 0.62)
    for i, (k, v) in enumerate(rows):
        y = top * 0.28 + i * rh
        d.line([(bx0, y), (tx1, y)], fill=ink, width=1)
        draw_text(d, k, fr, bx0 + 4, y + rh * 0.18, ink, anchor="l")
        draw_text(d, v, fr, tx1 - text_width(d, v, fr, 0) - 4, y + rh * 0.18, ink, anchor="l")
    d.rectangle([tx1 + 0.02 * w, top * 0.12, bx1, top * 0.88], fill=(255, 255, 255))
    barcode(d, tx1 + 0.03 * w, top * 0.18, bx1 - tx1 - 0.04 * w, top * 0.66, art["ean"], (15, 15, 15), rng)
    qs, m = min(low * 0.55, 0.10 * w), 0.006 * w
    lines = [line.format(**art["facts"]) for line in FINE_PRINT]
    longest = max(lines, key=lambda s: text_width(d, s, font("regular", 100), 0))
    fp = fit_text(d, longest, "regular", bx1 - qs - 3 * m - bx0, low * 0.065)
    for i, line in enumerate(lines):
        draw_text(d, line, fp, bx0, split + low * (0.08 + 0.105 * i), c_second, anchor="l")
    qy = split + (low - qs) / 2
    d.rectangle([bx1 - qs - 2 * m, qy - m, bx1, qy + qs + m], fill=(255,) * 3)
    qr(d, bx1 - qs - m, qy, qs, (15, 15, 15), rng)
    return img


# Square tea bottles (00006-00012): front panel, back panel and neck band, each drawn as seen from outside.
TEA_TABLE = [("能量", "0 kJ"), ("蛋白质", "0 g"), ("脂肪", "0 g"), ("碳水化合物", "0 g"), ("钠", "{sodium}")]
TEA_FINE_PRINT = [
    "配料：{ingredients}",
    "产品类型：{kind}",
    "保质期：{shelf}",
    "贮存条件：避免阳光直射，常温保存",
    "执行标准：{standard}",
    "生产日期：见瓶身",
    "开启后请尽快饮用，不宜久置",
    "客服热线：400-000-0000",
]


def homography(src, dst):
    """3 x 3 matrix mapping the four points src [(x, y)] onto dst."""
    a = []
    for (x, y), (u, v) in zip(src, dst):
        a += [[x, y, 1, 0, 0, 0, -u * x, -u * y], [0, 0, 0, x, y, 1, -v * x, -v * y]]
    h = np.linalg.solve(np.array(a, float), np.array(dst, float).reshape(-1))
    return np.r_[h, 1.0].reshape(3, 3)


def sample(photo, x, y):
    """Bilinear RGB (float 0-255) of a PIL image at pixel coordinates x, y (arrays of one shape)."""
    a = np.asarray(photo.convert("RGB"), np.float32)
    x = np.clip(np.asarray(x) - 0.5, 0, a.shape[1] - 1.001)
    y = np.clip(np.asarray(y) - 0.5, 0, a.shape[0] - 1.001)
    x0, y0 = np.floor(x).astype(int), np.floor(y).astype(int)
    fx, fy = (x - x0)[..., None], (y - y0)[..., None]
    top = a[y0, x0] * (1 - fx) + a[y0, x0 + 1] * fx
    return top * (1 - fy) + (a[y0 + 1, x0] * (1 - fx) + a[y0 + 1, x0 + 1] * fx) * fy


def photo_panel(photo, quad, size):
    """Label on a flat face, rectified from a photo: quad is its corners (top left, top right, bottom right, bottom
    left) in photo pixels. Returns float RGB (h, w, 3)."""
    w, h = size
    hm = homography([(0, 0), (w, 0), (w, h), (0, h)], quad)
    u, v = np.meshgrid(np.arange(w) + 0.5, np.arange(h) + 0.5)
    p = hm @ np.stack([u.ravel(), v.ravel(), np.ones(u.size)])
    return sample(photo, (p[0] / p[2]).reshape(h, w), (p[1] / p[2]).reshape(h, w))


def photo_band(photo, cx, rx, top, bottom, phi, seen, size):
    """Label on a vertical cylinder, unrolled from a photo taken square-on to its centre line: a point at angle a
    (0 = facing the camera, + = image right) is at x = cx + rx sin(a); the label's top and bottom edges are ellipse arcs
    with y = (y at a = 0, y at a = +-90 deg). phi: (a0, a1) radians spanned by the output; columns beyond +-seen (too
    oblique in the photo) get the row's median colour. Returns float RGB (h, w, 3)."""
    w, h = size
    a = np.linspace(phi[0], phi[1], w)
    v = (np.arange(h) + 0.5) / h
    yt = top[1] + (top[0] - top[1]) * np.cos(a)
    yb = bottom[1] + (bottom[0] - bottom[1]) * np.cos(a)
    y = yt[None] + v[:, None] * (yb - yt)[None]
    rgb = sample(photo, np.broadcast_to(cx + rx * np.sin(np.clip(a, -seen, seen))[None], y.shape), y)
    out = np.abs(a) > seen
    rgb[:, out] = np.median(rgb[:, ~out], axis=1)[:, None]
    return rgb


def even_light(rgb, background, target):
    """Divide out the photo's smooth lighting: per channel, fit a quadratic in (x, y) to the `background` pixels (bool
    mask) and rescale so that they come out as `target` (sRGB). Returns a PIL image."""
    h, w, _ = rgb.shape
    y, x = np.mgrid[0:h, 0:w] / np.array([h, w], float)[:, None, None]
    basis = np.stack([np.ones_like(x), x, y, x * x, x * y, y * y], -1)
    out = np.empty_like(rgb)
    for k in range(3):
        coef = np.linalg.lstsq(basis[background], rgb[..., k][background], rcond=None)[0]
        out[..., k] = rgb[..., k] / np.maximum(basis @ coef, 1.0) * target[k]
    return Image.fromarray(np.clip(out, 0, 255).astype(np.uint8))


def two_sided(img, backing, show):
    """The label image next to its reverse (right half): the backing colour with `show` of the print showing through.
    The reverse uses the same (u, v) as the front, so it reads mirrored from inside the bottle, as the real one does."""
    w, h = img.size
    a = np.asarray(img, np.float32)
    rev = (1 - show) * np.array(backing, np.float32) + show * a
    out = Image.new("RGB", (2 * w, h))
    out.paste(img, (0, 0))
    out.paste(Image.fromarray(rev.astype(np.uint8)), (w, 0))
    return out


def front_panel(art, size, seed):
    """Front label (w x h, portrait): product band on top, the brand set vertically, a picture and the tagline."""
    w, h = size
    rng = np.random.default_rng(seed)
    paper, ink, accent = srgb(art["paper"]), srgb(art["ink"]), srgb(art["accent"])
    img = Image.new("RGB", size, paper)
    d = ImageDraw.Draw(img)
    band = int(0.14 * h)
    d.rectangle([0, 0, w, band], fill=accent)
    f = fit_text(d, art["product"], "bold", 0.60 * w, 0.055 * h, spacing=0.08)
    draw_text(d, art["product"], f, 0.06 * w, 0.022 * h, paper, spacing=0.08, anchor="l")
    f = fit_text(d, art["volume"], "bold", 0.26 * w, 0.045 * h)
    draw_text(d, art["volume"], f, 0.94 * w - text_width(d, art["volume"], f, 0), 0.028 * h, paper, anchor="l")
    f = fit_text(d, art["note"], "regular", 0.88 * w, 0.032 * h, spacing=0.1)
    draw_text(d, art["note"], f, w / 2, 0.092 * h, paper, spacing=0.1)
    n = len(art["brand"])
    cell = min(0.40 * h / n, 0.62 * w)
    fb = font("brand", cell * 0.92)
    for i, ch in enumerate(art["brand"]):
        draw_text(d, ch, fb, w / 2, 0.17 * h + i * cell, ink)
    y = 0.17 * h + n * cell + 0.01 * h
    f = fit_text(d, art["latin"], "bold", 0.80 * w, 0.03 * h, spacing=0.2)
    draw_text(d, art["latin"], f, w / 2, y, ink, spacing=0.2)
    pw, ph = int(0.84 * w), int(0.90 * h - y - 0.06 * h)
    pic = picture(art["picture"], pw, ph, srgb(art["accent"]), paper, rng)
    px, py = int((w - pw) / 2), int(y + 0.05 * h)
    d.rectangle([px - 3, py - 3, px + pw + 3, py + ph + 3], fill=accent)
    img.paste(pic, (px, py))
    f = fit_text(d, art["tagline"], "regular", 0.84 * w, 0.032 * h, spacing=0.1)
    draw_text(d, art["tagline"], f, w / 2, 0.935 * h, ink, spacing=0.1)
    return img


def back_panel(art, size, seed):
    """Back label (w x h, portrait): claim band, nutrition table, fine print, brand mark and barcode."""
    w, h = size
    rng = np.random.default_rng(seed + 1000)
    paper, ink, accent = srgb(art["paper"]), srgb(art["ink"]), srgb(art["accent"])
    img = Image.new("RGB", size, (246, 246, 242))
    d = ImageDraw.Draw(img)
    m = 0.05 * w
    f = fit_text(d, art["brand"] + art["product"], "bold", w - 2 * m, 0.035 * h)
    draw_text(d, art["brand"] + art["product"], f, w / 2, 0.025 * h, ink)
    d.rectangle([m, 0.075 * h, w - m, 0.13 * h], fill=accent)
    f = fit_text(d, art["claim"], "bold", w - 3 * m, 0.04 * h)
    draw_text(d, art["claim"], f, w / 2, 0.083 * h, paper)
    title = "营养成分表"
    f = fit_text(d, title, "bold", w - 2 * m, 0.028 * h)
    draw_text(d, title, f, w / 2, 0.145 * h, ink)
    rows = [("项目", "每100毫升")] + [(k, v.format(**art["facts"])) for k, v in TEA_TABLE]
    rh, y0 = 0.032 * h, 0.185 * h
    fr = font("regular", rh * 0.62)
    for i, (k, v) in enumerate(rows):
        y = y0 + i * rh
        d.line([(m, y), (w - m, y)], fill=ink, width=1)
        draw_text(d, k, fr, m + 4, y + rh * 0.2, ink, anchor="l")
        draw_text(d, v, fr, w - m - text_width(d, v, fr, 0) - 4, y + rh * 0.2, ink, anchor="l")
    y = y0 + len(rows) * rh
    d.rectangle([m, y0, w - m, y], outline=ink, width=1)
    lines = [line.format(**art["facts"]) for line in TEA_FINE_PRINT]
    longest = max(lines, key=lambda s: text_width(d, s, font("regular", 100), 0))
    fp = fit_text(d, longest, "regular", w - 2 * m, 0.024 * h)
    for i, line in enumerate(lines):
        draw_text(d, line, fp, m, y + 0.025 * h + i * 0.034 * h, ink, anchor="l")
    y += 0.025 * h + len(lines) * 0.034 * h + 0.01 * h
    f = fit_text(d, art["brand"], "brand", 0.4 * w, 0.035 * h)
    bw = text_width(d, art["brand"], f, 0) + 0.06 * w
    d.rectangle([(w - bw) / 2, y, (w + bw) / 2, y + 0.05 * h], fill=accent)
    draw_text(d, art["brand"], f, w / 2, y + 0.008 * h, paper)
    barcode(d, 0.16 * w, 0.83 * h, 0.68 * w, 0.14 * h, art["ean"], (15, 15, 15), rng)
    return img


def neck_band(art, size, seed):
    """Neck label (w x h, landscape; its centre faces the front): brand, a line of small print, product and volume."""
    w, h = size
    paper, accent = srgb(art["paper"]), srgb(art["accent"])
    img = Image.new("RGB", size, accent)
    d = ImageDraw.Draw(img)
    for y in (0.07 * h, 0.93 * h):  # dotted rules
        for x in np.arange(0.005 * w, w, 0.012 * w):
            d.ellipse([x - 0.012 * h, y - 0.012 * h, x + 0.012 * h, y + 0.012 * h], fill=paper)
    f = fit_text(d, art["brand"], "brand", 0.34 * w, 0.40 * h, spacing=0.08)
    draw_text(d, art["brand"], f, w / 2, 0.14 * h, paper, spacing=0.08)
    f = fit_text(d, art["tagline"], "regular", 0.40 * w, 0.10 * h, spacing=0.15)
    draw_text(d, art["tagline"], f, w / 2, 0.60 * h, paper, spacing=0.15)
    line = f"{art['product']}  净含量{art['volume']}"
    f = fit_text(d, line, "bold", 0.42 * w, 0.13 * h)
    draw_text(d, line, f, w / 2, 0.75 * h, paper)
    return img
