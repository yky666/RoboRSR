"""One row per base asset: base + its variants, from real2sim/objects/Rigid/<cat>/<idx>/preview/sim.png.

Usage: python make_sheets.py BUILD_REPORT_JSON REAL2SIM_OBJECTS OUT_DIR [cat ...]
"""

import json
from pathlib import Path
import sys

from PIL import Image, ImageDraw

report = json.loads(Path(sys.argv[1]).read_text(encoding="utf-8"))
r2s, out = Path(sys.argv[2]), Path(sys.argv[3])
cats = set(sys.argv[4:])
out.mkdir(parents=True, exist_ok=True)
W, H = 320, 240
for row in report:
    if "error" in row:
        continue
    cat = row["id"].split("/")[1]
    if cats and cat not in cats:
        continue
    ids = [row["id"]] + [v["id"] for v in row["variants"]]
    labels = ["base"] + [
        f"{v['params']['color']} {v['params']['material']}\n{v['params']['shape'][0]} {v['params']['scale_cross_long']}"
        for v in row["variants"]
    ]
    canvas = Image.new("RGB", (W * len(ids), H + 34), "white")
    d = ImageDraw.Draw(canvas)
    for i, (aid, lab) in enumerate(zip(ids, labels)):
        p = r2s / aid / "preview" / "sim.png"
        if p.exists():
            canvas.paste(Image.open(p).convert("RGB").resize((W, H)), (i * W, 34))
        d.text((i * W + 4, 2), f"{aid.split('/', 1)[1]}  {lab}", fill="black")
    canvas.save(out / f"{row['id'].replace('/', '_')}.png")
    print("sheet", row["id"])
