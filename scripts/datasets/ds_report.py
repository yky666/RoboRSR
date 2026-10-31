"""Sum ds_scan.py units into one summary per dataset: episodes, frames, hours and how each was obtained.

    ds_report.py [NAME...]       default: every dataset under OUT; writes OUT/NAME/summary.json and prints a table
A dataset NAME may have companions NAME.mp4 (fps of its HDF5 episodes) and NAME.unpacked / NAME.hdf5 (loose files
next to its archives); they are summed into NAME.
"""

from collections import Counter
from fractions import Fraction
import glob
import json
import os
import re
import sys

OUT = os.environ.get("DS_SCAN_OUT", "/mnt/data/datasets/_stats")
LOG_DIR = os.path.join(OUT, "_logs")  # queue scripts and their logs, from every machine that scanned
EPISODE_PARQUET = re.compile(r"(^|/)data/(chunk-\d+/)?episode_\d+\.parquet$")
PART = re.compile(r'^s (dir|archive) ("([^"]+)"|(\S+))')
SUFFIXES = (".mp4", ".unpacked", ".hdf5")
KNOWN_GAPS = {"BiPlay": "TFRecord 主体还没有解析"}  # parts no queue scans yet: never complete


def base_of(part):
    for suf in SUFFIXES:
        if part.endswith(suf):
            return part[: -len(suf)]
    return part


def queue_state():
    """Per part: units planned (largest announced), whether a queue finished it; parts listed per dataset."""
    parts, planned, ended = {}, {}, set()
    for sh in glob.glob(os.path.join(LOG_DIR, "*.sh")):
        for line in open(sh, errors="replace"):
            m = PART.match(line.strip())
            if m:
                name = m[3] or m[4]
                parts.setdefault(base_of(name), set()).add(name)
    for log in glob.glob(os.path.join(LOG_DIR, "*.log")):
        for line in open(log, errors="replace"):
            m = re.match(r"^(.+?): (\d+) units", line)
            if m:
                planned[m[1]] = max(planned.get(m[1], 0), int(m[2]))
            m = re.match(r"^### \S+ end (.+)$", line.strip())
            if m:
                rest = m[1]
                if rest.startswith("dir "):  # "end dir NAME MODE ROOT" (queue A) or "end dir NAME MODE"
                    rest = re.sub(r"^dir ", "", rest)
                    rest = re.sub(r" (hdf5|mp4|mcap|bag|parquet)( \S+)?$", "", rest)
                ended.add(rest)
    return parts, planned, ended


def rate(s):
    try:
        return float(Fraction(s)) if s and s != "0/0" else None
    except (ValueError, ZeroDivisionError):
        return None


def load(name):
    return [json.load(open(p)) for p in sorted(glob.glob(os.path.join(OUT, glob.escape(name), "units", "*.json")))]


def info_fps(name, unit_key):
    """fps of the LeRobot meta/info.json files kept from one archive unit."""
    tag = re.sub(r"[^\w.-]+", "_", unit_key.strip("/"))[-180:]
    out = []
    for p in glob.glob(
        os.path.join(OUT, glob.escape(name), "meta", glob.escape(tag), "**", "info.json"), recursive=True
    ):
        try:
            d = json.load(open(p))
            out.append((d.get("fps"), d.get("total_episodes"), d.get("total_frames")))
        except ValueError:
            pass
    return out


def summarize(name):
    s = {
        "name": name,
        "episodes": 0,
        "frames": 0,
        "hours": 0.0,
        "hours_known_frames": 0,
        "fps": Counter(),
        "how": set(),
        "units_done": 0,
        "errors": 0,
        "unknown_fps_frames": 0,
    }
    parts, plan, ended = queue_state()
    mine = parts.get(name, {name})
    s["units_planned"] = sum(plan.get(p, 0) for p in mine)
    s["parts_pending"] = sorted(p for p in mine if p not in ended)
    s["failed_units"] = sum(len(glob.glob(os.path.join(OUT, glob.escape(p), "units", "*.failed"))) for p in mine)
    comp_fps = Counter()
    for r in load(name + ".mp4"):  # companion videos: fps for this dataset's HDF5 frames
        for row in r.get("rows", []):
            if rate(row.get("rate")):
                comp_fps[rate(row["rate"])] += 1
    s["units_done"] += len(load(name + ".mp4"))
    for part in [name, name + ".unpacked", name + ".hdf5"]:
        for r in load(part):
            s["units_done"] += 1
            s["errors"] += len(r.get("errors", []))
            if "mode" in r:  # dir unit
                rows, mode = r["rows"], r["mode"]
                if mode in ("mcap", "bag"):
                    s["episodes"] += len(rows)
                    s["hours"] += sum(max(0.0, x["t1"] - x["t0"]) for x in rows) / 3600
                    s["how"].add(f"{mode}: 每个文件一条，时长=结束-开始时间戳")
                elif mode == "mp4":
                    s["episodes"] += len(rows)
                    s["hours"] += sum(float(x["duration"]) for x in rows if x.get("duration")) / 3600
                    for x in rows:
                        if rate(x.get("rate")):
                            s["fps"][rate(x["rate"])] += 1
                    s["how"].add("mp4: 每个视频一条，时长=容器时长")
                elif mode == "parquet":
                    s["episodes"] += len(rows)
                    s["frames"] += sum(x["rows"] for x in rows)
                    for x in rows:
                        if x.get("dt"):
                            f = round(1 / x["dt"], 3)
                            s["fps"][f] += 1
                            s["hours"] += x["rows"] / f / 3600
                            s["hours_known_frames"] += x["rows"]
                    s["how"].add("parquet: 每个文件一条，时长=行数×时间戳步长")
                elif mode == "hdf5":
                    for x in rows:
                        s["episodes"] += x["demos"] or 1
                        s["frames"] += x["frames"]
                    s["how"].add("hdf5: 每个文件一条（robomimic 格式按 demo 数），帧数=主数据集第一维")
                continue
            # archive unit
            eps = [p for p in r["parquet"] if EPISODE_PARQUET.search(p["name"])]
            infos = info_fps(part, r["unit"])
            fps_meta = next((f for f, _, _ in infos if f), None)
            if eps:
                fr = sum(p["rows"] for p in eps)
                s["episodes"] += len(eps)
                s["frames"] += fr
                f = fps_meta or next((round(1 / p["dt"], 3) for p in eps if p.get("dt")), None)
                if f:
                    s["fps"][f] += 1
                    s["hours"] += fr / f / 3600
                    s["hours_known_frames"] += fr
                else:
                    s["unknown_fps_frames"] += fr
                s["how"].add(
                    "LeRobot 压缩包：每个 episode parquet 一条，帧数=行数，fps 取 meta/info.json（否则时间戳步长）"
                )
            elif r["hdf5"]:
                f = next((rate(m.get("rate")) for m in r["mp4"] if rate(m.get("rate"))), None)
                for x in r["hdf5"]:
                    s["episodes"] += x["demos"] or 1
                    s["frames"] += x["frames"]
                    if f:
                        s["hours"] += x["frames"] / f / 3600
                        s["hours_known_frames"] += x["frames"]
                    else:
                        s["unknown_fps_frames"] += x["frames"]
                if f:
                    s["fps"][f] += 1
                s["how"].add("HDF5 压缩包：每个文件一条，帧数=主数据集第一维，fps 取同包视频")
            elif r["kinds"].get(".npy"):
                s["episodes"] += r["kinds"][".npy"]
                s["unknown_fps_frames"] += 1
                s["how"].add("npy 标注包：每个 episode 一个 npy，源视频帧率不一，时长留空")
            else:
                s["how"].add(f"压缩包内容：{dict(r['kinds'])}")
    if s["frames"] and s["hours_known_frames"] < s["frames"] and comp_fps:
        f = comp_fps.most_common(1)[0][0]  # HDF5 frames counted in dir mode, fps from the companion videos
        s["hours"] += (s["frames"] - s["hours_known_frames"]) / f / 3600
        s["hours_known_frames"] = s["frames"]
        s["fps"][f] += sum(comp_fps.values())
        s["how"].add(f"fps={f:g} 取自同数据集视频（{sum(comp_fps.values())} 个，{len(comp_fps)} 种帧率）")
    s["fps"] = {str(k): v for k, v in s["fps"].most_common(5)}
    s["how"] = sorted(s["how"])
    s["hours"] = round(s["hours"], 4)
    s["gap"] = KNOWN_GAPS.get(name, "")
    s["complete"] = (
        bool(s["units_planned"])
        and s["units_done"] >= s["units_planned"]
        and not s["parts_pending"]
        and not s["failed_units"]
        and not s["gap"]
    )
    return s


def main():
    names = sys.argv[1:] or sorted(
        {
            base_of(os.path.basename(p))
            for p in glob.glob(os.path.join(OUT, "*"))
            if os.path.isdir(p) and not os.path.basename(p).startswith("_")
        }
    )
    for n in names:
        s = summarize(n)
        with open(
            os.path.join(OUT, n, "summary.json") if os.path.isdir(os.path.join(OUT, n)) else os.devnull, "w"
        ) as f:
            json.dump(s, f, ensure_ascii=False, indent=1)
        print(
            f"{n[:30]:30} units {s['units_done']:>5}/{s['units_planned']:<5} {'DONE' if s['complete'] else '    '} "
            f"eps {s['episodes']:>9} frames {s['frames']:>11} hours {s['hours']:>10.2f} "
            f"fps {s['fps']} err {s['errors']}"
        )


if __name__ == "__main__":
    main()
