"""One RSI round's layouts in bulk (rsi/AGENTS.md sections 4 and 5.3): a plan, its profiles and seed folders, the
manifest, and the G1 checks of an exam.

    python scripts/datagen/rsi_round.py plan --levels levels_0.json --round 1 --out <run>/exam/rounds/r1
    python scripts/datagen/rsi_round.py generate --plan <r1>/plan.json --assets Assets
    python scripts/datagen/rsi_round.py hold-folder --manifest <r1>/manifest.json --assets Assets
    python scripts/datagen/rsi_round.py check --manifest <r1>/manifest.json --hold-results eval_result
    python scripts/datagen/rsi_round.py topup --manifest <r1>/manifest.json --anchors make_toast=8 \
        --frontier camera=6 --capability-tasks arrange_largest_number --unscored make_toast:5111:0 --out <r1>/topup

plan allocates an exam (rsi/AGENTS.md v0.7): at every axis ~60% of the episodes at each family's frontier level and
~25% one level up, spread evenly over families and, inside a family, over its tasks, never on a capability-gap task, so
each axis keeps its frontier minimum without them; every task of the exam, gap tasks included (a gap is judged on the
anchors), gets at least MIN_ANCHORS L0 anchors; the fixed capability tasks get an L0 set of their own. topup plans
what an exam still needs after some layouts went unscored, into new seed folders, and check counts the exam and its
top-up together. generate takes any plan of jobs (task, axis,
level, episodes, seed), so the data side can use it too: one profile per subject (rsi_profile.compose), one
x5_layouts.py seed folder per subject, and manifest.json with one entry per layout. The seed folders are written under
<plan dir>/layouts/<env cfg>/<seed> and linked into the Assets/ overlay the engine reads. hold-folder links every
layout of a manifest into one folder per round for a hold-pose run (scripts/datagen/rsi_hold.py); check reads that
run's results, audits the assets and counts the gate.
"""

import argparse
import hashlib
import json
from pathlib import Path
import shutil
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT), str(ROOT / "scripts" / "datagen")]
import rsi_assets  # noqa: E402
import rsi_profile  # noqa: E402

SHARES = {"frontier": 0.6, "up": 0.25}
MIN_FRONTIER = 30  # episodes per axis at its frontier level, families pooled (the scorecard's publication floor)
MIN_ANCHORS = 10  # L0 episodes per exam task: a ratio of means divides by the task's L0 mean
TOPUP_SEED = 60  # + round base: a top-up's seed folders, after the exam's base + 10 * axis + level
SPLIT = {"exam": "val", "train": "train"}
SEED_BASE = {"exam": 5000, "train": 1000}  # + 100 * round; the capability set keeps the bare base
HOLD_SEED = 8900  # + round: the exam pool's hold-pose check folder, never examined


def load_subjects():
    import yaml

    return yaml.safe_load(rsi_profile.SUBJECTS.read_text(encoding="utf-8"))


def level_up(level):
    return f"L{min(int(level[1:]) + 1, 3)}"


def seed_of(pool, round_, axes, axis, level):
    """Seed folder of a subject: base + 100 * round, + 10 * (axis number) + level; the L0 anchors take the base."""
    base = SEED_BASE[pool] + 100 * round_
    return base if axis is None else base + 10 * (list(axes).index(axis) + 1) + int(level[1:])


def profile_name(axis, level):
    return "L0" if axis is None else f"{axis}_{level}"


def not_examinable(task, axis):
    """Why a task cannot show a level of `axis` above L0, or None. Instances need a labelled task object with a
    second instance to swap in (arrange_largest_number's digits carry no label: they are the answer)."""
    if axis != "instances":
        return None
    import x5_layouts

    labels = x5_layouts.pools(x5_layouts.load_yaml(x5_layouts.CONFIG_DIR / f"{task}.yml"))[0]
    if any(len(indices) > 1 for cats in labels.values() for _, _, indices in cats):
        return None
    return "任务配置里没有可换实例的带标签物体"


def spread(n, keys, start=0):
    """n split as evenly as possible over keys, the remainder going round-robin from keys[start]."""
    out = {k: n // len(keys) for k in keys}
    for j in range(n % len(keys)):
        out[keys[(start + j) % len(keys)]] += 1
    return out


def examined(manifests):
    """(family, axis, level) combinations earlier exams already held; anchors count as axis None."""
    seen = set()
    for path in manifests:
        for e in json.loads(Path(path).read_text(encoding="utf-8"))["layouts"]:
            if e["role"] != "capability":
                seen.add((e["family"], e["axis"], e["level"]))
    return seen


def plan(
    levels,
    round_,
    budget,
    subjects,
    history=(),
    pool="exam",
    skip=not_examinable,
    capability_tasks=(),
    min_anchors=MIN_ANCHORS,
):
    """Jobs of an exam from the referee's levels_<r> (levels: {family: {axis: level}}, capability: {...}); the
    capability-gap tasks get anchors only."""
    axes, families, gap = list(subjects["axes"]), subjects["families"], set(capability_tasks)
    capability = levels.get("capability") or {}
    exam_families = [f for f in families if f in levels["levels"] and f != capability.get("family")]
    jobs, blocked, cursor = {}, [], dict.fromkeys(exam_families, 0)

    def add(task, family, axis, level, role, n):
        if n <= 0:
            return
        key = (task, axis, level)
        job = jobs.setdefault(
            key,
            {
                "task": task,
                "family": family,
                "axis": axis,
                "level": level,
                "role": role,
                "episodes": 0,
                "seed": seed_of(pool, round_, axes, axis, level),
                "profile": f"profiles/{profile_name(axis, level)}.json",
            },
        )
        job["episodes"] += n

    per_axis = {
        "frontier": max(MIN_FRONTIER, round(budget * SHARES["frontier"] / len(axes))),
        "up": round(budget * SHARES["up"] / len(axes)),
    }
    for a, axis in enumerate(axes):
        open_tasks = {}
        for family in exam_families:
            reasons = {t: "能力缺口任务" if t in gap else skip(t, axis) for t in families[family]}
            if ok := [t for t, why in reasons.items() if not why]:
                open_tasks[family] = ok
            else:
                blocked.append({"family": family, "axis": axis, "reason": "；".join(sorted(set(reasons.values())))})
        for role, n in per_axis.items():
            for family, k in spread(n, list(open_tasks), a).items():
                frontier = levels["levels"][family][axis]
                level = frontier if role == "frontier" else level_up(frontier)
                ts = open_tasks[family]
                for task, m in spread(k, ts, cursor[family] % len(ts)).items():
                    add(task, family, axis, level, role, m)
                cursor[family] += k
    for family in exam_families:
        for task in families[family]:
            add(task, family, None, "L0", "anchor", min_anchors)
    exam_jobs = list(jobs.values())
    for task in capability.get("tasks") or []:
        exam_jobs.append(
            {
                "task": task,
                "family": capability["family"],
                "axis": None,
                "level": capability.get("level", "L0"),
                "role": "capability",
                "episodes": capability.get("episodes_per_task", 10),
                "seed": SEED_BASE[pool],
                "profile": "profiles/L0.json",
            }
        )
    return {
        "round": round_,
        "pool": pool,
        "split": SPLIT[pool],
        "budget": {"episodes": budget, "shares": SHARES, "min_frontier_per_axis": MIN_FRONTIER},
        "min_anchors": min_anchors,
        "capability_tasks": sorted(gap),
        "not_examinable": blocked,
        "capability": {
            "tasks": capability.get("tasks") or [],
            "seed": SEED_BASE[pool],
            "fixed": True,
            "note": "同一组 L0 布局，每轮都考、不重新生成；成绩前后轮成对比较",
        },
        "exploration": exploration(exam_jobs, history),
        "jobs": exam_jobs,
    }


def exploration(jobs, history):
    before = examined(history)
    total = sum(j["episodes"] for j in jobs if j["role"] != "capability")
    new = sum(
        j["episodes"] for j in jobs if j["role"] != "capability" and (j["family"], j["axis"], j["level"]) not in before
    )
    return {
        "history": [str(p) for p in history],
        "examined_before": sorted(map(list, before), key=str),
        "new_episodes": new,
        "episodes": total,
        "share": round(new / total, 3) if total else 0.0,
    }


def topup(manifest_path, anchors, frontier, subjects, capability_tasks=(), unscored=()):
    """Plan of what an exam still needs once some of its layouts went unscored: `anchors` {task: n} more L0 anchors,
    `frontier` {axis: n} more episodes at each family's frontier level of that axis, one at a time to the family, then
    the non-gap task, with the fewest scored ones. unscored: (task, seed, layout id) of the exam's lost layouts."""
    manifest_path = Path(manifest_path).resolve()
    m = json.loads(manifest_path.read_text(encoding="utf-8"))
    base_plan = json.loads((manifest_path.parent / m["plan"]).read_text(encoding="utf-8"))
    axes, families, gap = list(subjects["axes"]), subjects["families"], set(capability_tasks)
    family_of = {t: f for f, ts in families.items() for t in ts}
    lost = {(t, int(s), int(i)) for t, s, i in unscored}
    scored = [e for e in m["layouts"] if not e["capability"] and (e["task"], e["seed"], e["layout_id"]) not in lost]
    base = SEED_BASE[m["pool"]] + 100 * m["round"]
    free = iter(s for s in range(base + TOPUP_SEED, base + 100) if s not in {e["seed"] for e in m["layouts"]})
    seeds, jobs = {}, {}

    def add(task, axis, level, role):
        if (axis, level) not in seeds:
            seeds[(axis, level)] = next(free)
        job = jobs.setdefault(
            (task, axis, level),
            {
                "task": task,
                "family": family_of[task],
                "axis": axis,
                "level": level,
                "role": role,
                "episodes": 0,
                "seed": seeds[(axis, level)],
                "profile": f"profiles/{profile_name(axis, level)}.json",
            },
        )
        job["episodes"] += 1

    for task, n in anchors.items():
        for _ in range(n):
            add(task, None, "L0", "anchor")
    for axis in [a for a in axes if frontier.get(a)]:
        rows = [e for e in m["layouts"] if e["role"] == "frontier" and e["axis"] == axis and e["task"] not in gap]
        level = {e["family"]: e["level"] for e in rows}
        have = {(e["family"], e["task"]): 0 for e in rows}
        for e in scored:
            if (e["family"], e["task"]) in have and e["role"] == "frontier" and e["axis"] == axis:
                have[(e["family"], e["task"])] += 1
        tasks = {f: [t for t in families[f] if (f, t) in have] for f in families if f in level}
        for _ in range(frontier[axis]):
            fam = min(tasks, key=lambda f: (sum(have[(f, t)] for t in tasks[f]), list(families).index(f)))
            task = min(tasks[fam], key=lambda t: (have[(fam, t)], families[fam].index(t)))
            have[(fam, task)] += 1
            add(task, axis, level[fam], "frontier")
    history = base_plan["exploration"]["history"]
    return {
        "round": m["round"],
        "pool": m["pool"],
        "split": m["split"],
        "budget": {"topup": {"anchors": anchors, "frontier": frontier}, "min_frontier_per_axis": MIN_FRONTIER},
        "min_anchors": MIN_ANCHORS,
        "capability_tasks": sorted(gap),
        "topup_of": {
            "manifest": str(manifest_path),
            "unscored": [{"task": t, "seed": s, "layout_id": i} for t, s, i in sorted(lost)],
        },
        "hold_seed": HOLD_SEED + m["round"] + 10,
        "not_examinable": [],
        "capability": base_plan["capability"] | {"tasks": [], "note": "能力缺口固定集合在原考卷里，补考不含"},
        "exploration": exploration(list(jobs.values()), history),
        "jobs": list(jobs.values()),
    }


def env_cfg_of(task, profile):
    import x5_layouts

    return x5_layouts.env_cfg_name(task, profile.get("camera"))


def generate(plan_path, assets):
    """Profiles, seed folders and the manifest of a plan; seed folders that already have the task are kept."""
    import x5_layouts

    plan_path, assets = Path(plan_path).resolve(), Path(assets).resolve()
    out, p = plan_path.parent, json.loads(plan_path.read_text(encoding="utf-8"))
    subjects, opts = load_subjects(), rsi_profile.options(assets, p["pool"])
    profiles = {}
    for job in p["jobs"]:
        if job["profile"] not in profiles:
            levels = {} if job["axis"] is None else {job["axis"]: job["level"]}
            profile = rsi_profile.compose(subjects, levels, opts)
            if error := rsi_profile.check(profile, opts):
                raise SystemExit(f"{job['profile']}: {error}")
            (out / job["profile"]).parent.mkdir(parents=True, exist_ok=True)
            (out / job["profile"]).write_text(json.dumps(profile, ensure_ascii=False, indent=1), encoding="utf-8")
            profiles[job["profile"]] = profile
    for job in p["jobs"]:
        profile, task = profiles[job["profile"]], job["task"]
        env_cfg = env_cfg_of(task, profile)
        folder = out / "layouts" / env_cfg / str(job["seed"])
        link = assets / "Eval_Layout" / "RoboDojo" / env_cfg / str(job["seed"])
        folder.mkdir(parents=True, exist_ok=True)
        if not link.is_symlink() and not link.exists():
            link.parent.mkdir(parents=True, exist_ok=True)
            link.symlink_to(folder)
        elif link.resolve() != folder:
            raise SystemExit(f"{link} is not this round's folder {folder}")
        if not (folder / f"report_{task}.json").is_file():
            x5_layouts.main(
                [
                    *("--task", task, "--seed", str(job["seed"]), "--count", str(job["episodes"])),
                    *("--assets", str(assets), "--profile", str(out / job["profile"]), "--split", p["split"]),
                ]
            )
        if env_cfg != "arx_x5":
            (out / "env_cfg" / "camera").mkdir(parents=True, exist_ok=True)
            shutil.copy(ROOT / "env_cfg" / f"{env_cfg}.yml", out / "env_cfg")
            shutil.copy(
                ROOT / "env_cfg" / "camera" / f"{env_cfg.removeprefix('arx_x5_')}.yml", out / "env_cfg" / "camera"
            )
    manifest = build_manifest(p, out)
    (out / "manifest.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=1), encoding="utf-8")
    print(f"{len(manifest['layouts'])} layouts in {len(manifest['jobs'])} jobs -> {out / 'manifest.json'}")
    return manifest


def build_manifest(p, out):
    before = {tuple(c) for c in p["exploration"]["examined_before"]}
    jobs, layouts = [], []
    for job in p["jobs"]:
        profile = json.loads((out / job["profile"]).read_text(encoding="utf-8"))
        env_cfg, task = env_cfg_of(job["task"], profile), job["task"]
        folder = out / "layouts" / env_cfg / str(job["seed"])
        report = json.loads((folder / f"report_{task}.json").read_text(encoding="utf-8"))
        names = sorted(report["layouts"], key=lambda n: int(Path(n).stem.rsplit("_", 1)[1]))
        jobs.append(
            {
                "task": task,
                "env_cfg": env_cfg,
                "seed": job["seed"],
                "episodes": len(names),
                "planned": job["episodes"],
                "role": job["role"],
                "folder": str(folder.relative_to(out)),
            }
        )
        for name in names:
            layouts.append(
                {
                    "task": task,
                    "family": job["family"],
                    "axis": job["axis"],
                    "level": job["level"],
                    "role": job["role"],
                    "capability": job["role"] == "capability",
                    "seed": job["seed"],
                    "env_cfg": env_cfg,
                    "layout": name,
                    "layout_id": int(Path(name).stem.rsplit("_", 1)[1]),
                    "new_combination": job["role"] != "capability"
                    and (job["family"], job["axis"], job["level"]) not in before,
                    "redrawn": name in (report.get("redrawn") or {}),
                }
            )
    return {
        "round": p["round"],
        "pool": p["pool"],
        "split": p["split"],
        "plan": "plan.json",
        **{k: p[k] for k in ("min_anchors", "capability_tasks", "topup_of", "hold_seed") if k in p},
        "capability": p["capability"]
        | ({"folder": f"layouts/arx_x5/{p['capability']['seed']}"} if p["capability"]["tasks"] else {}),
        "env_cfg_files": "env_cfg/ (copy into the code checkout's env_cfg/ for the camera subjects)",
        "jobs": jobs,
        "layouts": layouts,
    }


def hold_folder(manifest_path, assets):
    """One arx_x5 seed folder linking every layout of the manifest per task, renumbered, for a hold-pose run."""
    manifest_path, assets = Path(manifest_path).resolve(), Path(assets).resolve()
    out, m = manifest_path.parent, json.loads(manifest_path.read_text(encoding="utf-8"))
    seed = m.get("hold_seed") or HOLD_SEED + m["round"]
    folder = out / "hold" / str(seed)
    shutil.rmtree(folder, ignore_errors=True)
    folder.mkdir(parents=True)
    index = {}
    for e in m["layouts"]:
        k = len(index.setdefault(e["task"], []))
        (folder / f"{e['task']}_{k}.json").symlink_to(out / "layouts" / e["env_cfg"] / str(e["seed"]) / e["layout"])
        index[e["task"]].append({"seed": e["seed"], "env_cfg": e["env_cfg"], "layout": e["layout"]})
    (folder / "split.json").write_text(json.dumps({"split": "val", "purpose": "hold-pose check"}), encoding="utf-8")
    (out / "hold" / "map.json").write_text(json.dumps({"seed": seed, "tasks": index}, indent=1), encoding="utf-8")
    link = assets / "Eval_Layout" / "RoboDojo" / "arx_x5" / str(seed)
    if link.is_symlink():
        link.unlink()
    link.parent.mkdir(parents=True, exist_ok=True)
    link.symlink_to(folder)
    print(f"hold folder {seed}: " + ", ".join(f"{t} {len(v)}" for t, v in index.items()))
    return seed, index


def hold_results(results_root, hold_map):
    """{task: {"run": n, "unstable": [map entries], "solved": [map entries], "results": paths}} from every
    _result.json of a task's hold runs (eval_result/RoboDojo/<task>/<policy>/arx_x5/<seed>_<info>/<run id>/), the
    latest run deciding a layout: a run stops at the task's eval_nums, so a long task takes several (--layout_ids)."""
    seed, found = hold_map["seed"], {}
    for task, entries in hold_map["tasks"].items():
        runs = sorted(
            Path(results_root).glob(f"RoboDojo/{task}/*/arx_x5/{seed}_*/*/_result.json"),
            key=lambda q: q.stat().st_mtime,
        )
        if not runs:
            found[task] = None
            continue
        done = {}
        for run in runs:
            details = json.loads(run.read_text(encoding="utf-8")).get("details") or {}
            done |= {int(d["layout_id"]): d for d in details.values()}
        found[task] = {
            "results": [str(r) for r in runs],
            "run": len(done),
            "unstable": [entries[k] for k in range(len(entries)) if k not in done],
            "solved": [entries[k] for k, d in sorted(done.items()) if d.get("success")],
        }
    return found


def scored_layouts(m):
    """Layouts a manifest's gate counts: its own, plus for a top-up the exam's minus the unscored ones."""
    if not (of := m.get("topup_of")):
        return m["layouts"]
    lost = {(u["task"], u["seed"], u["layout_id"]) for u in of["unscored"]}
    base = json.loads(Path(of["manifest"]).read_text(encoding="utf-8"))["layouts"]
    return [e for e in base if (e["task"], e["seed"], e["layout_id"]) not in lost] + m["layouts"]


def counts(manifest):
    """Episode counts the gate looks at; capability_tasks are left out of the frontier."""
    exam = [e for e in manifest["layouts"] if not e["capability"]]
    gap, low = set(manifest.get("capability_tasks") or []), manifest.get("min_anchors", 1)
    total = len(exam)
    by = {}
    for e in exam:
        by.setdefault(e["role"], {}).setdefault(f"{e['axis']}/{e['level']}", {}).setdefault(e["family"], 0)
        by[e["role"]][f"{e['axis']}/{e['level']}"][e["family"]] += 1
    anchors, frontier = {}, {}
    for e in exam:
        if e["role"] == "anchor":
            anchors[e["task"]] = anchors.get(e["task"], 0) + 1
        if e["role"] == "frontier":
            key = f"{e['axis']}/{e['level']}"
            frontier[key] = frontier.get(key, 0) + (e["task"] not in gap)
    tasks = sorted({e["task"] for e in exam})
    return {
        "episodes": total,
        "capability": {
            t: sum(e["task"] == t for e in manifest["layouts"] if e["capability"])
            for t in sorted({e["task"] for e in manifest["layouts"] if e["capability"]})
        },
        "share": {r: round(sum(sum(v.values()) for v in d.values()) / total, 3) for r, d in by.items()},
        "by_role": by,
        "frontier_without_gaps": frontier,
        "anchors": anchors,
        "tasks_short_of_anchors": [t for t in tasks if anchors.get(t, 0) < low],
        "new_share": round(sum(e["new_combination"] for e in exam) / total, 3) if total else 0.0,
    }


def check(manifest_path, results_root=None):
    manifest_path = Path(manifest_path).resolve()
    out, m = manifest_path.parent, json.loads(manifest_path.read_text(encoding="utf-8"))
    pools = rsi_assets.load()
    folders = sorted({(e["env_cfg"], e["seed"]) for e in m["layouts"]})
    audit = {f"{c}/{s}": rsi_assets.audit(out / "layouts" / c / str(s), pools, m["pool"]) for c, s in folders}
    splits = {
        f"{c}/{s}": json.loads((out / "layouts" / c / str(s) / "split.json").read_text(encoding="utf-8"))["split"]
        for c, s in folders
    }
    low, high = (SEED_BASE[m["pool"]], SEED_BASE[m["pool"]] + 3999)
    c = counts(m | {"layouts": scored_layouts(m)})
    frontier = c["frontier_without_gaps"]
    gate = {
        "audit": not any(audit.values()),
        "split": all(v == m["split"] for v in splits.values()),
        "seeds": all(low <= s <= high for _, s in folders),
        "frontier_min": all(n >= MIN_FRONTIER for n in frontier.values())
        and {k.split("/")[0] for k in frontier} == set(load_subjects()["axes"]),
        "anchors_every_task": not c["tasks_short_of_anchors"],
        "new_share": c["new_share"] >= 0.3,
    }
    if of := m.get("topup_of"):
        base = json.loads(Path(of["manifest"]).read_text(encoding="utf-8"))["layouts"]
        gate["new_seed_folders"] = not {s for _, s in folders} & {e["seed"] for e in base}
    hold = None
    if results_root:
        hold_map = json.loads((out / "hold" / "map.json").read_text(encoding="utf-8"))
        hold = hold_results(results_root, hold_map)
        gate["hold_ran"] = all(r is not None and r["run"] + len(r["unstable"]) > 0 for r in hold.values())
        gate["hold_stable"] = gate["hold_ran"] and not any(r["unstable"] or r["solved"] for r in hold.values())
    report = {
        "gate": gate,
        "passed": all(gate.values()),
        "counts": c,
        "frontier": frontier,
        "audit": audit,
        "split": splits,
        "hold": hold,
    }
    digest = hashlib.sha256(manifest_path.read_bytes()).hexdigest()[:12]
    (out / "g1.json").write_text(json.dumps(report | {"manifest_sha": digest}, ensure_ascii=False, indent=1), "utf-8")
    print(json.dumps({"gate": gate, "frontier": frontier, "share": c["share"]}, ensure_ascii=False))
    return report


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    s = sub.add_parser("plan")
    s.add_argument("--levels", type=Path, required=True, help="the referee's levels_<r>.json")
    s.add_argument("--round", type=int, required=True)
    s.add_argument("--budget", type=int, default=250, help="exam episodes, capability tasks not counted")
    s.add_argument("--history", type=Path, nargs="*", default=[], help="manifest.json of earlier rounds")
    s.add_argument("--capability-tasks", default="", help="comma-separated capability-gap tasks: anchors only")
    s.add_argument("--min-anchors", type=int, default=MIN_ANCHORS)
    s.add_argument("--out", type=Path, required=True)
    t = sub.add_parser("topup")
    t.add_argument("--manifest", type=Path, required=True, help="the exam's manifest.json")
    t.add_argument("--anchors", default="", help="task=n,... more L0 anchors")
    t.add_argument("--frontier", default="", help="axis=n,... more frontier episodes")
    t.add_argument("--capability-tasks", default="", help="comma-separated capability-gap tasks: no frontier")
    t.add_argument("--unscored", default="", help="task:seed:layout id,... of the exam's unscored layouts")
    t.add_argument("--out", type=Path, required=True)
    g = sub.add_parser("generate")
    g.add_argument("--plan", type=Path, required=True)
    g.add_argument("--assets", type=Path, required=True)
    h = sub.add_parser("hold-folder")
    h.add_argument("--manifest", type=Path, required=True)
    h.add_argument("--assets", type=Path, required=True)
    c = sub.add_parser("check")
    c.add_argument("--manifest", type=Path, required=True)
    c.add_argument("--hold-results", type=Path, help="eval_result/ root of the hold-pose run")
    args = ap.parse_args(argv)

    def counted(text):
        return {k: int(v) for k, v in (item.split("=") for item in text.split(",") if item)}

    if args.cmd in ("plan", "topup"):
        gap = [t for t in args.capability_tasks.split(",") if t]
        if args.cmd == "plan":
            levels = json.loads(args.levels.read_text(encoding="utf-8"))
            p = plan(
                levels,
                args.round,
                args.budget,
                load_subjects(),
                args.history,
                capability_tasks=gap,
                min_anchors=args.min_anchors,
            )
            p["levels"] = {"file": str(args.levels), "sha256": hashlib.sha256(args.levels.read_bytes()).hexdigest()}
        else:
            lost = [u.split(":") for u in args.unscored.split(",") if u]
            p = topup(args.manifest, counted(args.anchors), counted(args.frontier), load_subjects(), gap, lost)
        args.out.mkdir(parents=True, exist_ok=True)
        (args.out / "plan.json").write_text(json.dumps(p, ensure_ascii=False, indent=1), encoding="utf-8")
        print(f"{len(p['jobs'])} jobs, {p['exploration']['episodes']} exam episodes -> {args.out / 'plan.json'}")
    elif args.cmd == "generate":
        generate(args.plan, args.assets)
    elif args.cmd == "hold-folder":
        hold_folder(args.manifest, args.assets)
    else:
        raise SystemExit(0 if check(args.manifest, args.hold_results)["passed"] else 1)


if __name__ == "__main__":
    main()
