"""Render review media directly from same-frame simulation PNG and tactile packets."""

import argparse
import hashlib
import json
from pathlib import Path
import sys

import av
import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
from env.robot_manager.aurora_contract import CHANNELS  # noqa: E402
from env.robot_manager.tactile_bridge import XenseTactileBridge  # noqa: E402


def make(recording, tactile, out):
    out.mkdir(parents=True, exist_ok=False)
    frames = [json.loads(x) for x in (recording / "frames.jsonl").read_text().splitlines()]
    scene = json.loads((recording / "scene_schema.json").read_text())
    bridge = XenseTactileBridge(tactile) if tactile else None
    sha = hashlib.sha256((recording / "frames.jsonl").read_bytes()).hexdigest()
    if bridge and json.loads((tactile / "report.json").read_text())["source_frames_sha256"] != sha:
        raise ValueError("Different physics and tactile source")
    font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 18)
    large = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 24)
    outputs = {}
    for name, size in [
        ("overview", (640, 480)),
        ("left", (320, 240)),
        ("right", (320, 240)),
        ("combined", (1920, 1080)),
    ]:
        writer = av.open(str(out / (name + ".mp4")), "w", options={"movflags": "+faststart"})
        stream = writer.add_stream("libx264", rate=scene["fps"])
        stream.width, stream.height = size
        stream.pix_fmt = "yuv420p"
        stream.options = {"crf": "20", "preset": "fast"}
        outputs[name] = (writer, stream)
    for frame in frames:
        fid = frame["frame_id"]
        images = {n: Image.open(recording / n / f"{fid:05d}.png").convert("RGB") for n in ["overview", "left", "right"]}
        canvas = Image.new("RGB", (1920, 1080), (17, 22, 30))
        canvas.paste(images["overview"].resize((960, 720)), (0, 0))
        draw = ImageDraw.Draw(canvas)
        for i, n in enumerate(["left", "right"]):
            canvas.paste(images[n].resize((480, 360)), (i * 480, 720))
        draw.rectangle((0, 0, 960, 80), fill=(15, 20, 26))
        draw.text((15, 8), "G1-D dual Aurora Lite | gravity pick / transport / place", font=large, fill="white")
        draw.text(
            (15, 45),
            f"{frame['phase']} | frame {fid:03d} | t={fid / scene['fps']:.1f}s | SIMULATION",
            font=font,
            fill="#b7d8ed",
        )
        if bridge:
            pkt = bridge.observation(fid, frame["simulation_time_s"])
            for row, key in enumerate(["fem_rgb", "simple_rgb"]):
                y = row * 540
                draw.text((975, y + 8), key + " | independent process / no force feedback", font=large, fill="white")
                for c, ch in enumerate(CHANNELS):
                    x = 960 + c * 240
                    draw.text(
                        (x + 8, y + 45), ch.replace("positive", "+").replace("negative", "-"), font=font, fill="#b7d8ed"
                    )
                    canvas.paste(Image.fromarray(pkt[key][c]).resize((230, 403)), (x + 5, y + 72))
                    depth = -pkt["depth_mm"][c].min()
                    force = np.linalg.norm(pkt["native_normal_force_w_n"][c])
                    draw.text((x + 8, y + 485), f"{depth:.3f}mm / {force:.2f}N", font=font, fill="white")
        else:
            draw.text((990, 300), "Tactile export NOT accepted for this run.", font=large, fill="#ffbc82")
            draw.text((990, 350), "See failure log; no zero-image substitution.", font=large, fill="white")
        images["combined"] = canvas
        if fid == min(70, len(frames) - 1):
            canvas.save(out / "poster.png")
        for n, img in images.items():
            writer, stream = outputs[n]
            for packet in stream.encode(av.VideoFrame.from_image(img)):
                writer.mux(packet)
    for writer, stream in outputs.values():
        for packet in stream.encode():
            writer.mux(packet)
        writer.close()
    (out / "provenance.json").write_text(
        json.dumps(
            dict(
                frames=len(frames),
                source_frames_sha256=sha,
                media="original same-state PNG; no state reconstruction or physics replay",
                tactile=bool(bridge),
                fps=scene["fps"],
                hardware_connected=False,
            ),
            indent=2,
        )
        + "\n"
    )


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--recording", type=Path, required=True)
    p.add_argument("--tactile", type=Path)
    p.add_argument("--out", type=Path, required=True)
    a = p.parse_args()
    make(a.recording, a.tactile, a.out)
