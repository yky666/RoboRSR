"""rsi_round.py: an exam plan from round-0 levels on the real rsi/subjects.yaml and task configs, and the manifest,
hold-pose and gate bookkeeping on synthetic seed folders (no Isaac)."""

import json
from pathlib import Path
import sys
import tempfile

sys.path.insert(0, str(Path(__file__).resolve().parent))
import rsi_round as R  # noqa: E402

SUBJECTS = R.load_subjects()
FAMILIES = [f for f in SUBJECTS["families"] if f != "tool"]
LEVELS = {
    "levels": {f: dict.fromkeys(SUBJECTS["axes"], "L1") for f in FAMILIES},
    "capability": {"family": "tool", "tasks": ["push_T", "sweep_blocks"], "episodes_per_task": 10, "level": "L0"},
}


def write(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data))


def test_plan():
    p = R.plan(LEVELS, 1, 250, SUBJECTS)
    exam = [j for j in p["jobs"] if j["role"] != "capability"]
    assert sum(j["episodes"] for j in exam) == 250 and p["exploration"]["share"] == 1.0  # round 1: all new
    by = {}
    for j in exam:
        key = (j["role"], j["axis"], j["level"])
        by.setdefault(key, {}).setdefault(j["family"], 0)
        by[key][j["family"]] += j["episodes"]
    for axis in SUBJECTS["axes"]:
        frontier, up = by[("frontier", axis, "L1")], by[("up", axis, "L2")]
        assert sum(frontier.values()) == 30 and sum(up.values()) == 12
        assert max(frontier.values()) - min(frontier.values()) <= 1  # families pooled, spread evenly
    assert "symbolic" not in by[("frontier", "instances", "L1")]  # the digits are the answer: nothing to swap
    assert p["not_examinable"] == [
        {"family": "symbolic", "axis": "instances", "reason": R.not_examinable("arrange_largest_number", "instances")}
    ]
    anchors = {j["task"]: j["episodes"] for j in exam if j["role"] == "anchor"}
    assert set(anchors) == {t for f in FAMILIES for t in SUBJECTS["families"][f]} and set(anchors.values()) == {4}
    place = {}
    for j in exam:
        if j["family"] == "place" and j["role"] != "anchor":
            place[j["task"]] = place.get(j["task"], 0) + j["episodes"]
    assert len(place) == 4 and max(place.values()) - min(place.values()) <= 1  # tasks of a family share it evenly
    cap = [j for j in p["jobs"] if j["role"] == "capability"]
    assert [(j["task"], j["episodes"], j["seed"]) for j in cap] == [("push_T", 10, 5000), ("sweep_blocks", 10, 5000)]
    seeds = {(j["axis"], j["level"]): j["seed"] for j in exam}
    assert len(set(seeds.values())) == len(seeds) and all(5100 <= s < 5200 for s in seeds.values())


def fake_round(p):
    out = Path(tempfile.mkdtemp())
    for j in p["jobs"]:
        profile = {"camera": {"ori_deg": 2}} if j["axis"] == "camera" else {}
        write(out / j["profile"], profile)
        folder = out / "layouts" / R.env_cfg_of(j["task"], profile) / str(j["seed"])
        names = [f"{j['task']}_{i}.json" for i in range(j["episodes"])]
        for name in names:
            write(folder / name, {"Table": {"default": "material_0122"}})
        write(folder / f"report_{j['task']}.json", {"layouts": dict.fromkeys(names, {})})
        write(folder / "split.json", {"split": p["split"]})
    return out


def test_manifest_hold_and_gate():
    p = R.plan(LEVELS, 1, 250, SUBJECTS)
    out = fake_round(p)
    m = R.build_manifest(p, out)
    assert len(m["layouts"]) == 270 and sum(e["capability"] for e in m["layouts"]) == 20
    cam = [e for e in m["layouts"] if e["axis"] == "camera"]
    assert {e["env_cfg"] for e in cam} >= {"arx_x5_rsi_stack_bowls_cam2", "arx_x5_rsi_fold_clothes_cam2"}
    write(out / "manifest.json", m)
    report = R.check(out / "manifest.json")
    assert report["passed"] and report["frontier"]["clutter/L1"] == 30
    assert report["counts"]["share"] == {"frontier": 0.6, "up": 0.24, "anchor": 0.16}
    assets = Path(tempfile.mkdtemp())
    seed, index = R.hold_folder(out / "manifest.json", assets)
    assert seed == 8901 and (assets / "Eval_Layout/RoboDojo/arx_x5/8901/make_toast_0.json").is_file()
    results = Path(tempfile.mkdtemp())
    for task, entries in index.items():
        run = range(1, len(entries)) if task == "make_toast" else range(len(entries))  # layout 0 unstable
        details = {str(n): {"layout_id": k, "success": False, "score": 0.0} for n, k in enumerate(run)}
        write(results / f"RoboDojo/{task}/OpenWAM/arx_x5/{seed}_rsihold/t0/_result.json", {"details": details})
    report = R.check(out / "manifest.json", results)
    assert report["gate"]["hold_ran"] and not report["gate"]["hold_stable"]
    assert report["hold"]["make_toast"]["unstable"] == [index["make_toast"][0]]
    assert not any(r["unstable"] for t, r in report["hold"].items() if t != "make_toast")
    rerun = {"0": {"layout_id": 0, "success": False, "score": 0.0}}  # a second run on the rest (--layout_ids)
    write(results / f"RoboDojo/make_toast/OpenWAM/arx_x5/{seed}_rsihold/t1/_result.json", {"details": rerun})
    assert R.check(out / "manifest.json", results)["passed"]
    history = R.plan(LEVELS, 2, 250, SUBJECTS, [out / "manifest.json"])
    assert history["exploration"]["share"] == 0.0 and history["jobs"][0]["seed"] // 100 == 52  # nothing new left
