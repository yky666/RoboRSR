"""Wrap-around label artwork for make_plastic_bottle.py (numpy + PIL only, so it can be previewed without pxr).

The image is the label unrolled: u (left to right) runs counter-clockwise around the bottle seen from above, which is
left to right for a viewer outside; v runs bottom to top. Bottom `share` of the height is the label's main colour, the
rest its second colour. Front panel: picture, brand, Latin brand, product and volume; back panel: composition table,
barcode, fine print and a QR code. The 00000 twin has its own layout traced from photos (nongfu_label). Everything
printed is generic or fictional except the 00000 twin's brand words and emblem.
"""

import os
from pathlib import Path

import numpy as np
from PIL import Image, ImageChops, ImageDraw, ImageFilter, ImageFont

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


def draw_text(draw, text, f, cx, top, fill, spacing=0.0, anchor="c", stroke=0):
    """Draw `text` with letter spacing; cx is the centre (anchor c) or left edge (anchor l); top is the ink top.
    stroke (px) thickens the glyphs."""
    w = text_width(draw, text, f, spacing)
    x = cx - w / 2 if anchor == "c" else cx
    y = top - draw.textbbox((0, 0), text, font=f)[1]
    for ch in text:
        draw.text((x, y), ch, font=f, fill=fill, stroke_width=stroke, stroke_fill=fill)
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


def paste_text(img, text, role, box, fill, spacing=0.0, weight=0.04):
    """Text whose ink is scaled to fill box (x0, y0, x1, y1) exactly, so it can be condensed like a printed logo;
    weight (em) thickens the strokes toward the heavy faces of the real print."""
    x0, y0, x1, y1 = (int(round(v)) for v in box)
    f = font(role, 3 * (y1 - y0))
    w = int(text_width(ImageDraw.Draw(img), text, f, spacing)) + 2 * f.size
    mask = Image.new("L", (w, 2 * f.size))
    draw_text(ImageDraw.Draw(mask), text, f, f.size // 2, f.size // 2, 255, spacing, "l", int(weight * f.size))
    mask = mask.crop(mask.getbbox()).resize((x1 - x0, y1 - y0), Image.LANCZOS)
    img.paste(Image.new("RGB", mask.size, fill), (x0, y0), mask)


def mountain_logo(w, h, rng):
    """农夫山泉 front emblem (RGBA w x h), traced from the product photo: a dark green peak over its striped
    reflection, teal side mountains with snow streaks and their faint reflections, two birds."""
    img = Image.new("RGBA", (w, h))
    d = ImageDraw.Draw(img)
    teal, mid, dark, lit = (98, 182, 162, 255), (38, 134, 100, 255), (6, 66, 36, 255), (22, 104, 60, 255)
    wl = 0.58 * h  # water line
    for (x0, xa, ya, x1), col in (((0.05, 0.26, 0.24, 0.42), teal), ((0.57, 0.71, 0.24, 0.93), mid)):
        d.polygon([(x0 * w, wl), (xa * w, ya * h), (x1 * w, wl)], fill=col)
        for k in range(3):  # thin snow streaks down from the ridge
            s = 0.18 + 0.22 * k
            xs, ys = xa + (x1 - xa) * s * 0.5, ya + (0.58 - ya) * s * 0.5
            d.line([(xs * w, ys * h), ((xs - 0.02) * w, (ys + 0.16) * h)], fill=(255, 255, 255, 200),
                   width=max(1, int(0.008 * w)))  # fmt: skip
        for k in range(5):  # side reflections
            y = wl + (0.035 + 0.045 * k) * h
            half = (x1 - x0) / 2 * (1 - 0.15 * k) * w
            cx = (x0 + x1) / 2 * w + rng.uniform(-0.01, 0.01) * w
            d.rectangle([cx - half, y, cx + half, y + 0.018 * h], fill=(*col[:3], 150 - 25 * k))
    d.polygon([(0.24 * w, wl), (0.47 * w, 0.07 * h), (0.72 * w, wl)], fill=dark)
    d.polygon([(0.24 * w, wl), (0.47 * w, 0.07 * h), (0.43 * w, wl)], fill=lit)
    for k in range(8):  # the peak's reflection narrows to a drop
        y = wl + (0.03 + 0.048 * k) * h
        half = 0.24 * (1 - k / 8.5) ** 1.3 * w
        cx = 0.48 * w + rng.uniform(-0.012, 0.012) * w
        d.rectangle([cx - half, y, cx + half, y + 0.026 * h], fill=dark)
    for bx, by, s in ((0.62, 0.16, 0.030), (0.69, 0.11, 0.036)):
        d.line([((bx - s) * w, (by - s) * h), (bx * w, by * h), ((bx + 0.6 * s) * w, (by - 1.2 * s) * h)],
               fill=mid, width=max(2, int(0.012 * w)))  # fmt: skip
    return img


def nongfu_label(art, main, second, share, size, seed):
    """Layout of the 00000 twin, after the photos: going right from the front (brand under the mountain emblem, the
    red band dipping at the front) come a vertical barcode over fine print, a white photo panel the full label
    height, then the composition table over a QR code. Drawn with the front at u 0.5, rolled to u 0.38."""
    w, h = size
    rng = np.random.default_rng(seed)
    red, white, line = srgb(main), srgb(second), (18, 58, 34)
    img = Image.new("RGB", size, white)
    d = ImageDraw.Draw(img)
    x = np.arange(w)
    bump = np.clip(np.abs(x / w - 0.5) / art["dip_w"], 0, 1)
    split = h * (1 - share) + art["dip"] * h * (1 + np.cos(np.pi * bump)) / 2
    d.polygon([(0, h)] + list(zip(x, split)) + [(w - 1, h)], fill=red)
    top, low = h * (1 - share), share * h  # band heights at the sides

    # front, centred at u 0.5
    lw, lh = int(0.23 * w), int(0.34 * h)
    logo = mountain_logo(lw, lh, rng)
    img.paste(logo, (int(0.5 * w - lw / 2), int(0.085 * h)), logo)
    bw = 0.28 * w
    paste_text(img, art["brand"], "bold", (0.5 * w - bw / 2, 0.555 * h, 0.5 * w + bw / 2, 0.694 * h), white, 0.08, 0.03)
    f = font("regular", 0.022 * h)
    d.text((0.5 * w + bw / 2 + 0.003 * w, 0.548 * h), "®", font=f, fill=white)
    paste_text(img, art["latin"], "bold", (0.5 * w - bw / 2, 0.711 * h, 0.5 * w + bw / 2, 0.775 * h), white, 0.10)
    paste_text(img, f"{art['product']}  净含量{art['volume']}", "bold",
               (0.5 * w - bw / 2, 0.799 * h, 0.5 * w + bw / 2, 0.856 * h), white, weight=0.015)  # fmt: skip

    # barcode (bars run around the bottle, digits upright along it) over fine print
    bx0, bx1 = 0.70 * w, 0.83 * w
    bar = Image.new("RGB", (int(0.30 * h), int(0.08 * w)), (255, 255, 255))
    barcode(ImageDraw.Draw(bar), 0, 0, bar.width, bar.height, art["ean"], (15, 15, 15), rng)
    img.paste(bar.rotate(90, expand=True), (int(bx0 + 0.01 * w), int(0.08 * h)))
    lines = [s.format(**art["facts"]) for s in FINE_PRINT]
    longest = max(lines, key=lambda s: text_width(d, s, font("regular", 100), 0))
    fp = fit_text(d, longest, "regular", bx1 - bx0, 0.032 * h)
    for i, s in enumerate(lines):
        draw_text(d, s, fp, bx0, top + low * 0.12 + i * 0.046 * h, white, anchor="l")

    # photo panel, white the full label height
    px0, px1 = 0.85 * w, 0.985 * w
    d.rectangle([px0, 0, px1, h], fill=white)
    pic = picture("landscape", int(px1 - px0 - 0.012 * w), int(0.40 * h), red, white, rng)
    img.paste(pic, (int(px0 + 0.006 * w), int(0.05 * h)))
    for i, (s, role, k) in enumerate(((art["source"], "bold", 0.05), (art["tagline"], "regular", 0.035))):
        f = fit_text(d, s, role, px1 - px0 - 0.012 * w, k * h)
        draw_text(d, s, f, (px0 + px1) / 2, (0.49 + 0.07 * i) * h, line)
    for i in range(6):  # small print under the photo
        y = (0.64 + 0.045 * i) * h
        d.rectangle([px0 + 0.008 * w, y, px1 - (0.008 + rng.uniform(0, 0.04)) * w, y + 0.014 * h], fill=(90, 90, 90))

    # composition table over a QR code and two lines on the red band
    tx0, tx1 = 0.04 * w, 0.29 * w
    ink = red
    f = fit_text(d, "饮用天然水物质成分表", "bold", tx1 - tx0, top * 0.11)
    draw_text(d, "饮用天然水物质成分表", f, tx0, top * 0.10, ink, anchor="l")
    f = fit_text(d, "每100mL含量 (μg/100mL)", "regular", (tx1 - tx0) * 0.7, top * 0.07)
    draw_text(d, "每100mL含量 (μg/100mL)", f, tx0, top * 0.25, ink, anchor="l")
    rh = top * 0.56 / len(WATER_TABLE)
    fr = font("regular", rh * 0.62)
    for i, (k, v) in enumerate(WATER_TABLE):
        y = top * 0.36 + i * rh
        d.line([(tx0, y), (tx1, y)], fill=ink, width=2)
        draw_text(d, k, fr, tx0 + 4, y + rh * 0.18, ink, anchor="l")
        draw_text(d, v, fr, tx1 - text_width(d, v, fr, 0) - 4, y + rh * 0.18, ink, anchor="l")
    qs, m = 0.40 * low, 0.006 * w
    qx, qy = tx0 + m, top + (low - qs) / 2
    d.rectangle([qx - m, qy - m, qx + qs + m, qy + qs + m], fill=(255, 255, 255))
    qr(d, qx, qy, qs, (15, 15, 15), rng)
    for i, s in enumerate((art["note"], art["tagline"], "请勿重复灌装使用")):
        f = fit_text(d, s, "regular", tx1 - qx - qs - 3 * m, low * 0.08)
        draw_text(d, s, f, qx + qs + 2 * m, top + low * (0.25 + 0.16 * i), white, anchor="l")

    d.rectangle([0, 0, w, 0.017 * h], fill=line)
    d.rectangle([0, h - 0.022 * h, w, h], fill=line)
    return ImageChops.offset(img, int(-0.12 * w), 0)


def draw_label(art, main, second, share, size, seed):
    """Label image (PIL RGB, size = (w, h)). main / second: linear RGB of the bottom / top colour bands."""
    if art.get("layout") == "nongfu":
        return nongfu_label(art, main, second, share, size, seed)
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
