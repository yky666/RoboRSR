"""rsi_round.py: an exam plan from round-0 levels on the real rsi/subjects.yaml and task configs, and the manifest,
top-up, hold-pose and gate bookkeeping on synthetic seed folders (no Isaac)."""

import json
from pathlib import Path
import sys
import tempfile

sys.path.insert(0, str(Path(__file__).resolve().parent))
import rsi_round as R  # noqa: E402

SUBJECTS = R.load_subjects()
FAMILIES = [f for f in SUBJECTS["families"] if f != "tool"]
TASKS = {t for f in FAMILIES for t in SUBJECTS["families"][f]}
GAPS = ["sort_nesting_dolls_by_size", "arrange_largest_number"]
LEVELS = {
    "levels": {f: dict.fromkeys(SUBJECTS["axes"], "L1") for f in FAMILIES},
    "capability": {"family": "tool", "tasks": ["push_T", "sweep_blocks"], "episodes_per_task": 10, "level": "L0"},
}


def write(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data))


def tally(jobs):
    by = {}
    for j in jobs:
        key = (j["role"], j["axis"], j["level"])
        by.setdefault(key, {}).setdefault(j["family"], 0)
        by[key][j["family"]] += j["episodes"]
    return by


def test_plan():
    p = R.plan(LEVELS, 1, 250, SUBJECTS)
    exam = [j for j in p["jobs"] if j["role"] != "capability"]
    assert p["exploration"]["share"] == 1.0  # round 1: all new
    by = tally(exam)
    for axis in SUBJECTS["axes"]:
        frontier, up = by[("frontier", axis, "L1")], by[("up", axis, "L2")]
        assert sum(frontier.values()) == 30 and sum(up.values()) == 12
        assert max(frontier.values()) - min(frontier.values()) <= 1  # families pooled, spread evenly
    assert "symbolic" not in by[("frontier", "instances", "L1")]  # the digits are the answer: nothing to swap
    assert p["not_examinable"] == [
        {"family": "symbolic", "axis": "instances", "reason": R.not_examinable("arrange_largest_number", "instances")}
    ]
    anchors = {j["task"]: j["episodes"] for j in exam if j["role"] == "anchor"}
    assert set(anchors) == TASKS and set(anchors.values()) == {R.MIN_ANCHORS}  # v0.7: >= 10 per task
    place = {}
    for j in exam:
        if j["family"] == "place" and j["role"] != "anchor":
            place[j["task"]] = place.get(j["task"], 0) + j["episodes"]
    assert len(place) == 4 and max(place.values()) - min(place.values()) <= 1  # tasks of a family share it evenly
    cap = [j for j in p["jobs"] if j["role"] == "capability"]
    assert [(j["task"], j["episodes"], j["seed"]) for j in cap] == [("push_T", 10, 5000), ("sweep_blocks", 10, 5000)]
    seeds = {(j["axis"], j["level"]): j["seed"] for j in exam}
    assert len(set(seeds.values())) == len(seeds) and all(5100 <= s < 5200 for s in seeds.values())


def test_plan_capability_gaps():
    """Gap tasks keep their anchors and get no frontier; the axes keep 30 frontier episodes without them."""
    p = R.plan(LEVELS, 2, 250, SUBJECTS, capability_tasks=GAPS)
    exam = [j for j in p["jobs"] if j["role"] != "capability"]
    assert not [j for j in exam if j["task"] in GAPS and j["role"] != "anchor"]
    assert {j["task"] for j in exam if j["role"] == "anchor"} == TASKS
    by = tally(exam)
    for axis in SUBJECTS["axes"]:
        assert sum(by[("frontier", axis, "L1")].values()) == 30 and "symbolic" not in by[("frontier", axis, "L1")]
    assert {"family": "symbolic", "axis": "clutter", "reason": "能力缺口任务"} in p["not_examinable"]
    assert p["capability_tasks"] == sorted(GAPS) and p["min_anchors"] == 10


def fake_round(p, out=None):
    out = out or Path(tempfile.mkdtemp())
    for j in p["jobs"]:
        profile = {"camera": {"ori_deg": 2}} if j["axis"] == "camera" else {}
        write(out / j["profile"], profile)
        folder = out / "layouts" / R.env_cfg_of(j["task"], profile) / str(j["seed"])
        names = [f"{j['task']}_{i}.json" for i in range(j["episodes"])]
        for name in names:
            write(folder / name, {"Table": {"default": "material_0122"}})
        write(folder / f"report_{j['task']}.json", {"layouts": dict.fromkeys(names, {})})
        write(folder / "split.json", {"split": p["split"]})
    write(out / "plan.json", p)
    m = R.build_manifest(p, out)
    write(out / "manifest.json", m)
    return out, m


def test_manifest_hold_and_gate():
    out, m = fake_round(R.plan(LEVELS, 1, 250, SUBJECTS))
    assert len(m["layouts"]) == 210 + 100 + 20 and sum(e["capability"] for e in m["layouts"]) == 20
    cam = [e for e in m["layouts"] if e["axis"] == "camera"]
    assert {e["env_cfg"] for e in cam} >= {"arx_x5_rsi_stack_bowls_cam2", "arx_x5_rsi_fold_clothes_cam2"}
    report = R.check(out / "manifest.json")
    assert report["passed"] and report["frontier"]["clutter/L1"] == 30
    assets = Path(tempfile.mkdtemp())
    seed, index = R.hold_folder(out / "manifest.json", assets)
    assert seed == 8901 and (assets / "Eval_Layout/RoboDojo/arx_x5/8901/make_toast_0.json").is_file()
    results = Path(tempfile.mkdtemp())
    for task, entries in index.items():
        run = range(1, min(len(entries), 25)) if task == "make_toast" else range(len(entries))  # 0 unstable
        details = {str(n): {"layout_id": k, "success": False, "score": 0.0} for n, k in enumerate(run)}
        write(results / f"RoboDojo/{task}/OpenWAM/arx_x5/{seed}_rsihold/t0/_result.json", {"details": details})
    report = R.check(out / "manifest.json", results)
    assert report["gate"]["hold_ran"] and not report["gate"]["hold_stable"]
    assert report["hold"]["make_toast"]["unstable"][0] == index["make_toast"][0]
    assert not any(r["unstable"] for t, r in report["hold"].items() if t != "make_toast")
    rest = range(25, len(index["make_toast"]))  # a run stops at eval_nums: the rest goes in a second one
    rerun = {str(n): {"layout_id": k, "success": False, "score": 0.0} for n, k in enumerate([0, *rest])}
    write(results / f"RoboDojo/make_toast/OpenWAM/arx_x5/{seed}_rsihold/t1/_result.json", {"details": rerun})
    assert R.check(out / "manifest.json", results)["passed"]
    history = R.plan(LEVELS, 2, 250, SUBJECTS, [out / "manifest.json"])
    assert history["exploration"]["share"] == 0.0 and history["jobs"][0]["seed"] // 100 == 52  # nothing new left


def test_topup():
    """Lost layouts and gap tasks leave axes short; the top-up fills them on the families with the fewest."""
    out, m = fake_round(R.plan(LEVELS, 1, 250, SUBJECTS))
    lost = [("make_toast", 5111, 0), ("make_toast", 5100, 0), ("make_toast", 5100, 3)]
    frontier = {}
    for e in m["layouts"]:
        scored = (e["task"], e["seed"], e["layout_id"]) not in lost
        if e["role"] == "frontier" and e["task"] not in GAPS and scored:
            frontier[e["axis"]] = frontier.get(e["axis"], 0) + 1
    need = {a: 30 - n for a, n in frontier.items()}
    assert need["clutter"] > 5  # symbolic's 5 and sort's share of place
    assert not R.check(_gapped(out, m))["gate"]["frontier_min"]
    p = R.topup(out / "manifest.json", {"make_toast": 2}, need, SUBJECTS, GAPS, lost)
    assert not [j for j in p["jobs"] if j["task"] in GAPS]
    assert sum(j["episodes"] for j in p["jobs"] if j["role"] == "frontier") == sum(need.values())
    seeds = {j["seed"] for j in p["jobs"]}
    assert not seeds & {e["seed"] for e in m["layouts"]} and all(5160 <= s < 5200 for s in seeds)
    top = out / "topup"
    _, tm = fake_round(p, top)
    assert tm["topup_of"]["unscored"][0] == {"task": "make_toast", "seed": 5100, "layout_id": 0}
    report = R.check(top / "manifest.json")
    assert report["passed"] and report["gate"]["new_seed_folders"]
    assert report["frontier"] == {f"{a}/L1": 30 for a in SUBJECTS["axes"]}  # gap tasks left out
    assert report["counts"]["anchors"]["make_toast"] == 10
    for axis in SUBJECTS["axes"]:  # exam and top-up together: families even again
        per = {}
        for e in R.scored_layouts(tm):
            if e["role"] == "frontier" and e["axis"] == axis and e["task"] not in GAPS:
                per[e["family"]] = per.get(e["family"], 0) + 1
        assert "symbolic" not in per and max(per.values()) - min(per.values()) <= 1, (axis, per)


def _gapped(out, m):
    """The exam's manifest with the gap tasks declared, as a v0.7 check would count it."""
    path = out / "manifest_gapped.json"
    write(path, m | {"capability_tasks": GAPS, "min_anchors": 10})
    return path
