"""Audit matching physical/tactile frames and make a simulation-only review video."""

import argparse
import hashlib
import json
from pathlib import Path
import shutil
import sys

import av
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
from env.robot_manager.aurora_contract import CHANNELS  # noqa: E402
from env.robot_manager.tactile_bridge import XenseTactileBridge  # noqa: E402


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument(
        "--native", type=Path, required=True, help="folder containing recording/, videos and render_clipped_v2/"
    )
    p.add_argument("--tactile", type=Path, required=True)
    p.add_argument("--out", type=Path, required=True)
    a = p.parse_args()
    a.out.mkdir(parents=True, exist_ok=False)
    raw = a.native / "recording"
    frames = [json.loads(x) for x in (raw / "frames.jsonl").read_text().splitlines()]
    tactile = [json.loads(x) for x in (a.tactile / "metrics.jsonl").read_text().splitlines()]
    source_sha = hashlib.sha256((raw / "frames.jsonl").read_bytes()).hexdigest()
    contract = json.loads((a.tactile / "report.json").read_text())
    render_path = a.native / "render_provenance.json"
    if not render_path.is_file():
        render_path = a.native / "render_clipped_v2/render_provenance.json"
    render = json.loads(render_path.read_text())
    if source_sha != contract["source_frames_sha256"] or source_sha != render["source_frames_sha256"]:
        raise ValueError("Physical/tactile/render recordings have different source hashes")
    if len(frames) != len(tactile) or len(frames) != render["frames"]:
        raise ValueError("Frame count mismatch")
    bridge = XenseTactileBridge(a.tactile)
    phases = {}
    for f in frames:
        phases.setdefault(f["phase"], f["frame_id"])
    gauges = []
    for width in (30, 50):
        hold = [f for f in frames if f["phase"] == f"gauge_{width}_hold"]
        for side in ("left", "right"):
            name = f"Gauge{width}_{side}"
            positions = np.array([f["objects"][name]["pose_wxyz"][:3] for f in hold])
            f = hold[-1]
            pos = np.array(f["pad_world"][side + "_positive"])
            neg = np.array(f["pad_world"][side + "_negative"])
            gap = float(np.dot(neg[:3, 3] - pos[:3, 3], pos[:3, 2]))
            gauges.append(
                dict(
                    width_mm=width,
                    arm=side,
                    measured_gap_mm=gap * 1000,
                    max_hold_drift_mm=float(np.linalg.norm(positions - positions[0], axis=1).max() * 1000),
                    load_n=0.02 * 9.81,
                    hold_seconds=len(hold) / 10,
                    normal_contact_n={
                        ch: float(np.linalg.norm(f["native_contact_normal_force_w_n"][ch]))
                        for ch in CHANNELS
                        if ch.startswith(side)
                    },
                    within_candidate_gap_tolerance=abs(gap - width / 1000) < 0.0005,
                )
            )
    slip = []
    baseline = next(f for f in reversed(frames) if f["phase"] == "gauge_30_hold")
    end = next(f for f in reversed(frames) if f["phase"] == "gauge_30_slip")
    for side in ("left", "right"):
        n = "Gauge30_" + side
        distance = float(
            np.linalg.norm(np.array(end["objects"][n]["pose_wxyz"][:3]) - baseline["objects"][n]["pose_wxyz"][:3])
        )
        slip.append(
            dict(
                arm=side,
                applied_tangent_load_n=5.0,
                mass_kg=0.02,
                displacement_m=distance,
                detached_under_load=distance > 0.05,
            )
        )
    residual = {ch: tactile[-1]["channels"][ch]["fem_node_force_array_norm"] for ch in CHANNELS}
    checks = dict(
        frame_association=True,
        finite_outputs=contract["all_outputs_finite"],
        native_contacts_all_channels=all(
            np.linalg.norm(f["native_contact_normal_force_w_n"][ch]) > 0
            for ch in CHANNELS
            for f in [next(x for x in frames if x["phase"] == "flat_hold")]
        ),
        known_width_candidate_check=all(g["within_candidate_gap_tolerance"] for g in gauges),
        load_induced_slip_observed=all(x["detached_under_load"] for x in slip),
        fem_unloaded_residual_below_1e_3=all(v < 1e-3 for v in residual.values()),
    )
    report = dict(
        scope="simulation integration and controlled fixture checks; not hardware or general manipulation acceptance",
        checks=checks,
        frames=len(frames),
        fps=10,
        source_frames_sha256=source_sha,
        recorded_simulation_duration_s=frames[-1]["simulation_time_s"] - frames[0]["simulation_time_s"] + 0.1,
        gauges=gauges,
        slip=slip,
        fem_unloaded_array_norm=residual,
        render_max_pad_matrix_error=max(x["max_pad_matrix_difference"] for x in render["checks"]),
        real_mount_verified=False,
        real_material_calibrated=False,
        real_tactile_calibrated=False,
        realtime_30hz_verified=False,
        physics_force_feedback_from_fem=False,
        hardware_connected=False,
        inherited_full_body_controls_and_collision_preserved=True,
    )
    (a.out / "summary.json").write_text(json.dumps(report, indent=2) + "\n")
    time = np.array([f["simulation_time_s"] for f in frames])
    time -= time[0]
    fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True, constrained_layout=True)
    for ch in CHANNELS:
        axes[0].plot(time, [-f["channels"][ch]["min_depth_mm"] for f in tactile], label=ch)
        axes[1].plot(time, [np.linalg.norm(f["native_contact_normal_force_w_n"][ch]) for f in frames], label=ch)
        axes[2].plot(time, [f["channels"][ch]["fem_node_force_array_norm"] for f in tactile], label=ch)
    for ax, label in zip(axes, ["Indentation (mm)", "PhysX normal force (N)", "FEM array norm (vendor units)"]):
        ax.set_ylabel(label)
        ax.grid(alpha=0.25)
    axes[0].legend(ncol=2, fontsize=9)
    axes[-1].set_xlabel("Recorded simulation time (s)")
    fig.suptitle("Four G1-WS pads | candidate simulation | FEM force not fed back to physics")
    fig.savefig(a.out / "contact_curves.png", dpi=150)
    plt.close(fig)
    decoded = {}
    for camera in ("overview", "left", "right"):
        path = a.native / (camera + ".mp4")
        with av.open(str(path)) as source:
            decoded[camera] = [x.to_image() for x in source.decode(video=0)]
        if len(decoded[camera]) != len(frames):
            raise ValueError("Video length mismatch")
        shutil.copyfile(path, a.out / (camera + ".mp4"))
    fontpath = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
    font = ImageFont.truetype(fontpath, 18)
    large = ImageFont.truetype(fontpath, 24)
    video = av.open(str(a.out / "g1d_four_pad_fusion.mp4"), "w", options={"movflags": "+faststart"})
    stream = video.add_stream("libx264", rate=10)
    stream.width = 1920
    stream.height = 1080
    stream.pix_fmt = "yuv420p"
    stream.options = {"crf": "20", "preset": "fast"}
    for f, m in zip(frames, tactile):
        packet = bridge.observation(f["frame_id"], f["simulation_time_s"])
        canvas = Image.new("RGB", (1920, 1080), (17, 22, 30))
        draw = ImageDraw.Draw(canvas)
        canvas.paste(decoded["overview"][f["frame_id"]], (0, 0))
        for i, side in enumerate(("left", "right")):
            canvas.paste(decoded[side][f["frame_id"]].resize((480, 360)), (i * 480, 720))
            draw.rectangle((i * 480, 720, i * 480 + 480, 748), fill=(15, 20, 26))
            draw.text((i * 480 + 10, 724), side.upper() + " gripper | saved PhysX state", font=font, fill="white")
        draw.rectangle((0, 0, 960, 75), fill=(15, 20, 26))
        draw.text((15, 8), "G1-D + dual Aurora Lite | SIMULATION ONLY", font=large, fill="white")
        draw.text(
            (15, 44),
            f"Frame {f['frame_id']:03d}  t={time[f['frame_id']]:.1f}s  {f['phase']}",
            font=font,
            fill="#b7d8ed",
        )
        for row, (key, title) in enumerate(
            (("fem_rgb", "FEM + markers | one-way estimate"), ("simple_rgb", "SimpleSensor | depth-based tactile RGB"))
        ):
            y = row * 540
            draw.text((975, y + 8), title, font=large, fill="white")
            for c, ch in enumerate(CHANNELS):
                x = 960 + c * 240
                draw.text(
                    (x + 8, y + 43), ch.replace("positive", "+").replace("negative", "-"), font=font, fill="#b7d8ed"
                )
                canvas.paste(Image.fromarray(packet[key][c]).resize((230, 403)), (x + 5, y + 72))
                values = m["channels"][ch]
                native = np.linalg.norm(f["native_contact_normal_force_w_n"][ch])
                draw.text((x + 8, y + 481), f"depth {-values['min_depth_mm']:.3f} mm", font=font, fill="white")
                draw.text((x + 8, y + 505), f"PhysX {native:.2f} N", font=font, fill="#b7d8ed")
        if f["frame_id"] == 90:
            canvas.save(a.out / "poster.png")
        frame = av.VideoFrame.from_image(canvas)
        for packet_out in stream.encode(frame):
            video.mux(packet_out)
    for packet_out in stream.encode():
        video.mux(packet_out)
    video.close()
    template = Path(__file__).with_name("fusion_review_template.html").read_text()
    (a.out / "index.html").write_text(template.replace("PHASES", json.dumps(phases)), encoding="utf-8")
    print(json.dumps(dict(checks=checks, out=str(a.out))), flush=True)


if __name__ == "__main__":
    main()
