"""Write ds_report.py summaries into the platform's dataset inventory (a copy; publish it with r2s publish).

    ds_apply.py INVENTORY_IN SUMMARIES_DIR INVENTORY_OUT
Only complete scans fill a version's numbers; a scan still running only adds a progress line to its note. Robot
trajectories (real / sim / mixed) fill the counted values; human video only its hours; other kinds (navigation,
motion capture, point tracking) only the raw record. Each write names how it was counted.
"""

import datetime
import glob
import json
import os
import re
import sys

ASSET_ONLY = {"LIBERO-plus", "RoboCasa"}  # their local archives hold scene / model assets, no demonstrations
TODAY = datetime.date.today().isoformat()
TAG = re.compile(r"【本地扫描[^】]*】")  # replaced on every sync, so later results refresh it


def main(src, sums, dst):
    inv = json.load(open(src))
    by_name = {g["name"]: g for g in inv["groups"]}
    changed = []
    for p in sorted(glob.glob(os.path.join(sums, "*", "summary.json"))):
        s = json.load(open(p))
        g = by_name.get(s["name"])
        if not g:
            continue
        v = max(g["versions"], key=lambda x: x["gb"] or 0)  # the local copy that was scanned
        note = TAG.sub("", v.get("note") or "")
        if s["name"] in ASSET_ONLY and s["complete"]:
            if v["state"] != "仅资产（非轨迹）":
                v["state"] = "仅资产（非轨迹）"
                v["note"] = note + f"【本地扫描 {TODAY}：压缩包里只有场景 / 模型资产，没有示范轨迹】"
                changed.append((s["name"], "asset-only"))
            continue
        if not s["complete"]:
            v["note"] = note + (
                f"【本地扫描 {TODAY} 进行中：已读 {s['units_done']}/{s['units_planned']} 个单元，"
                f"{s['episodes']} 条、{s['hours']:.1f} h，完成前不计入】"
            )
            changed.append((s["name"], f"progress {s['units_done']}/{s['units_planned']}"))
            continue
        if not s["episodes"]:
            continue
        hours = round(s["hours"], 6) if s["hours"] and s["hours_known_frames"] >= s["frames"] else None
        how = "；".join(s["how"])
        v["note"] = note + (
            f"【本地扫描 {TODAY}：{s['units_done']} 个单元全部读完（压缩包按流解压，不落盘），"
            f"{s['episodes']} 条、{s['frames']} 帧{f'、{hours:.2f} h' if hours else '，帧率未知时长留空'}；{how}"
            f"；fps 分布 {s['fps']}；读取错误 {s['errors']} 处】"
        )
        v["episodes"] = s["episodes"]
        if hours is not None:
            v["hours"] = hours
        if v["source"] in ("real", "sim", "mixed"):
            v["selectedEpisodes"] = s["episodes"]
            v["selectedHours"] = hours
        elif v["source"] == "ego" and hours is not None:
            v["selectedHours"] = hours
        changed.append((s["name"], f"{s['episodes']} eps, {hours} h, source {v['source']}"))
    inv["date"] = TODAY
    json.dump(inv, open(dst, "w"), ensure_ascii=False, indent=1)
    for c in changed:
        print(*c)


if __name__ == "__main__":
    main(*sys.argv[1:])
