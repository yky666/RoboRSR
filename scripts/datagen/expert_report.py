"""Report on a batch of scripted-expert episodes: the feedback of one iteration on an expert (rsi/AGENTS.md R0.3).

Run with h5py, numpy and opencv (the Isaac env interpreter works, nothing is launched), on collect.py outputs:
    python scripts/datagen/expert_report.py output/x5_probe/stack_bowls output/x5_probe/stack_bowls_b \
        --out output/x5_probe/report [--baseline output/x5_probe/report_prev/report.json] [--sheets 6]
Per episode: the outcome (accepted; rejected at the start; the expert's failure, numbers stripped so that the same
failure groups across layouts; or the task checks still failing at the end, from the plugin's final_checks), the
largest per-action joint and link6 steps of each arm against the official demos' maxima, and the recorded objects
flung or off the table (privileged/<name>_poses). Writes report.json (the summary, with a 95% Wilson interval on the
acceptance rate, and every episode) and a cam_head contact sheet per failed episode under sheets/, and prints the
summary as markdown, against --baseline when given.
"""

import argparse
import json
import math
from pathlib import Path
import re
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

# official stack_bowls X5 demos (100 episodes, 25 Hz): largest per-action joint step 0.155 rad, link6 step 0.037 m
JOINT_STEP_RAD, LINK6_STEP_M = 0.16, 0.04
FLUNG_M_S = 1.0  # an object faster than this was knocked away, not carried
OFF_TABLE_Z = 0.69  # m: the X5 tabletop is at 0.74
SHEET_FRAMES, SHEET_WIDTH = 6, 320


def wilson(k, n, z=1.96):
    if not n:
        return [0.0, 0.0]
    p, d = k / n, 1 + z * z / n
    c, h = p + z * z / (2 * n), z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n))
    return [round((c - h) / d, 3), round((c + h) / d, 3)]


def outcome(q):
    if q.get("accepted"):
        return "accepted"
    if q.get("reason"):
        return f"rejected: {q['reason']}"
    if q.get("failure"):
        return "expert: " + " ".join(re.sub(r"\[[^\]]*\]|-?\d+(\.\d+)?", "", q["failure"]).split())
    failing = [name for name, holds in q.get("final_checks") or [] if not holds]
    return "task checks: " + (", ".join(failing) if failing else "held too briefly")


def motion(f, fps):
    """Largest per-action step of each arm's joints and link6 (executed state), and the flags they raise."""
    stats, flags = {}, []
    for side in ("left", "right"):
        q = f[f"state/{side}_arm_joint_states"][()]
        p = f[f"state/{side}_ee_poses"][()][:, :3]
        joint = float(np.abs(np.diff(q, axis=0)).max(initial=0.0))
        link6 = float(np.linalg.norm(np.diff(p, axis=0), axis=1).max(initial=0.0))
        stats[side] = dict(joint_step_rad=round(joint, 4), link6_step_m=round(link6, 4))
        flags += [f"{side} joint step"] * (joint > JOINT_STEP_RAD) + [f"{side} link6 step"] * (link6 > LINK6_STEP_M)
    for key in f.get("privileged", {}):
        poses = f[f"privileged/{key}"][()]
        if not key.endswith("_poses") or poses.ndim != 3 or len(poses) < 2:
            continue
        speed = np.linalg.norm(np.diff(poses[:, :, :3], axis=0), axis=2).max(0) * fps
        name = key.removesuffix("_poses")
        stats[name] = dict(max_speed_m_s=speed.round(3).tolist(), min_z_m=poses[:, :, 2].min(0).round(3).tolist())
        flags += [f"{name} flung"] * bool((speed > FLUNG_M_S).any())
        flags += [f"{name} off the table"] * bool((poses[:, :, 2].min(0) < OFF_TABLE_Z).any())
    return stats, flags


def contact_sheet(f, path):
    """cam_head at SHEET_FRAMES evenly spaced frames (the last included) in one row."""
    import cv2

    from XPolicyLab.utils.process_data import decode_image_bit

    colors = f["vision/cam_head/colors"]
    tiles = []
    for i in np.linspace(0, len(colors) - 1, SHEET_FRAMES).round().astype(int):
        rgb = decode_image_bit(colors[i])
        tile = cv2.resize(rgb, (SHEET_WIDTH, round(rgb.shape[0] * SHEET_WIDTH / rgb.shape[1])))
        cv2.putText(tile, str(i), (6, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
        tiles.append(tile)
    path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(path), np.hstack(tiles)[..., ::-1])


def episode(sidecar, sheets_left, out):
    import h5py

    q = json.loads(sidecar.read_text())
    row = dict(
        run=sidecar.parent.name,
        layout=f"{q.get('layout_seed')}/{q.get('layout_id')}",
        outcome=outcome(q),
        frames=q.get("frames", 0),
    )
    row.update({k: q[k] for k in ("failure", "final_checks") if q.get(k)})
    data = sidecar.with_suffix(".hdf5")
    if row["frames"] and data.exists():
        with h5py.File(data, "r") as f:
            row["motion"], row["flags"] = motion(f, q.get("fps", 25))
            if row["outcome"] != "accepted" and sheets_left > 0:
                sheet = out / "sheets" / f"{sidecar.parent.name}_{sidecar.stem}.jpg"
                contact_sheet(f, sheet)
                row["sheet"] = str(sheet)
    return row


def summarize(rows):
    accepted = [r for r in rows if r["outcome"] == "accepted"]
    outcomes, flags = {}, {}
    for r in rows:
        outcomes[r["outcome"]] = outcomes.get(r["outcome"], 0) + 1
        for flag in r.get("flags", []):
            flags[flag] = flags.get(flag, 0) + 1
    frames = [r["frames"] for r in accepted]
    return dict(
        episodes=len(rows),
        accepted=len(accepted),
        rate=round(len(accepted) / len(rows), 3) if rows else 0.0,
        rate_95=wilson(len(accepted), len(rows)),
        accepted_frames=[min(frames), int(np.median(frames)), max(frames)] if frames else None,
        outcomes=dict(sorted(outcomes.items(), key=lambda kv: -kv[1])),
        flags=dict(sorted(flags.items(), key=lambda kv: -kv[1])),
    )


def markdown(summary, rows, baseline=None):
    s, b = summary, baseline or {}
    was = f" (baseline {b['accepted']}/{b['episodes']} = {b['rate']:.0%})" if b else ""
    lines = [
        f"accepted {s['accepted']}/{s['episodes']} = {s['rate']:.0%}, 95% CI {s['rate_95']}{was}",
        f"accepted frames min/median/max: {s['accepted_frames']}",
        "",
        "| outcome | n | baseline |",
        "| :-- | --: | --: |",
    ]
    for name in dict.fromkeys([*s["outcomes"], *b.get("outcomes", {})]):
        before = b["outcomes"].get(name, 0) if b else ""
        lines.append(f"| {name} | {s['outcomes'].get(name, 0)} | {before} |")
    if s["flags"]:
        lines += ["", "flags: " + ", ".join(f"{k} x{v}" for k, v in s["flags"].items())]
    lines += ["", "| run | layout | frames | outcome | flags | sheet |", "| :-- | :-- | --: | :-- | :-- | :-- |"]
    for r in rows:
        lines.append(
            f"| {r['run']} | {r['layout']} | {r['frames']} | {r['outcome']} | {', '.join(r.get('flags', []))} "
            f"| {r.get('sheet', '')} |"
        )
    return "\n".join(lines)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("runs", nargs="+", type=Path, help="collect.py --output folders")
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--baseline", type=Path, help="report.json of the previous iteration")
    parser.add_argument("--sheets", type=int, default=6, help="contact sheets for at most this many failures")
    args = parser.parse_args(argv)
    sidecars = sorted(p for run in args.runs for p in run.glob("episode_*.json"))
    if not sidecars:
        raise SystemExit(f"no episode_*.json in {[str(r) for r in args.runs]}")
    rows = []
    for sidecar in sidecars:
        rows.append(episode(sidecar, args.sheets - sum("sheet" in r for r in rows), args.out))
    summary = summarize(rows)
    args.out.mkdir(parents=True, exist_ok=True)
    report = dict(runs=[str(r) for r in args.runs], summary=summary, episodes=rows)
    (args.out / "report.json").write_text(json.dumps(report, indent=2) + "\n")
    baseline = json.loads(args.baseline.read_text())["summary"] if args.baseline else None
    print(markdown(summary, rows, baseline))


if __name__ == "__main__":
    main()
