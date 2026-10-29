"""Judge one RSI exam (rsi/AGENTS.md 5.3 step 3): scores, retention, level changes and the two parties' outputs.

    python scripts/datagen/rsi_scorecard.py --manifest referee/rounds/r1/manifest.json --eval-root run/eval_result \
        --run-id exam_r1 --levels referee/levels_0.json --round 1 --model "OpenWAM-α" --out referee

Deterministic: the same results give the same files (bootstrap with a fixed seed). An episode's retention is its
score over its task's L0 anchor mean in the same exam; a subject's retention rho is the mean over its episodes,
families pooled. Tasks whose L0 mean is below CAPABILITY are capability gaps: reported, never in rho. The frontier
level of an axis moves up when rho's 95% lower bound >= UP, down when the upper bound < DOWN, by one level, inside
L1..L3; the pooled decision applies to every family on that axis. Writes
- out/rounds/r<N>/scores.json: every number, referee only;
- out/out/data/scorecard_r<N>.json: banded (score per 5 points, rho per 0.1), subjects with n >= MIN_N only;
- out/out/exam/results_r<N>/episodes.json and out/levels_<N>.json (+ a copy for the exam side).
"""

import argparse
from collections import defaultdict
import json
import math
from pathlib import Path
import random

CAPABILITY, UP, DOWN, MIN_N = 10.0, 0.7, 0.3, 30
LEVELS = ["L0", "L1", "L2", "L3"]


def bootstrap_rho(eps, anchors, n=2000, seed=0):
    """95% interval of rho resampling both the subject's episodes and each task's L0 anchors (4 a task are few);
    a resampled anchor mean is floored at CAPABILITY so the ratio stays bounded."""
    rng = random.Random(seed)
    means = []
    for _ in range(n):
        l0 = {t: max(sum(rng.choices(a, k=len(a))) / len(a), CAPABILITY) for t, a in sorted(anchors.items())}
        means.append(sum(s / l0[t] for t, s in rng.choices(eps, k=len(eps))) / len(eps))
    means.sort()
    return means[int(0.025 * n)], means[int(0.975 * n) - 1]


def band(x, width):
    lo = math.floor(x / width + 1e-9) * width
    return f"{lo:g}-{lo + width:g}" if width >= 1 else f"{lo:.1f}-{lo + width:.1f}"


def episodes(manifest, eval_root, run_id):
    """Manifest layouts joined with their scores (0-100); a layout without a result is left out and listed."""
    scores = {}
    for f in Path(eval_root).glob(f"RoboDojo/*/**/{run_id}_*/_result.json"):
        task = f.relative_to(eval_root).parts[1]
        seed = int(f.parent.name.rsplit("_", 1)[1])
        for d in json.loads(f.read_text())["details"].values():
            scores[(task, seed, int(d["layout_id"]))] = 100 * d["score"]
    out, missing = [], []
    for lay in manifest["layouts"]:
        key = (lay["task"], lay["seed"], lay["layout_id"])
        (out.append({**lay, "score": scores[key]}) if key in scores else missing.append(key))
    return out, missing


def judge(eps, levels):
    anchors = defaultdict(list)
    for e in eps:
        if e["level"] == "L0" and not e["capability"]:
            anchors[e["task"]].append(e["score"])
    l0 = {t: sum(v) / len(v) for t, v in anchors.items()}
    gaps = sorted(t for t, m in l0.items() if m < CAPABILITY)

    subjects = defaultdict(list)
    for e in eps:
        if e["axis"] and e["task"] in l0 and e["task"] not in gaps:
            subjects[(e["axis"], e["level"])].append((e["task"], e["score"]))
    rows = []
    for (axis, level), v in sorted(subjects.items()):
        rho = sum(s / l0[t] for t, s in v) / len(v)
        lo, hi = bootstrap_rho(v, {t: anchors[t] for t in {t for t, _ in v}})
        rows.append({
            "axis": axis, "level": level, "n": len(v), "score": sum(s for _, s in v) / len(v),
            "rho": rho, "rho_ci95": [lo, hi],
        })  # fmt: skip

    new = json.loads(json.dumps(levels["levels"]))
    moves = {}
    for axis in sorted({r["axis"] for r in rows}):
        fronts = {lv[axis] for lv in new.values() if axis in lv}
        for front in fronts:
            row = next((r for r in rows if r["axis"] == axis and r["level"] == front), None)
            if row is None or row["n"] < MIN_N:
                continue
            i = LEVELS.index(front)
            step = 1 if row["rho_ci95"][0] >= UP else -1 if row["rho_ci95"][1] < DOWN else 0
            to = LEVELS[min(max(i + step, 1), len(LEVELS) - 1)]
            moves[f"{axis}/{front}"] = to
            for lv in new.values():
                if lv.get(axis) == front:
                    lv[axis] = to

    capability = defaultdict(list)
    for e in eps:
        if e["capability"] or e["task"] in gaps:
            if e["level"] == "L0":
                capability[e["task"]].append(e["score"])
    return {
        "l0": l0, "capability_gaps": gaps, "subjects": rows, "moves": moves, "levels": new,
        "capability": {t: {"n": len(v), "score": sum(v) / len(v)} for t, v in sorted(capability.items())},
    }  # fmt: skip


def scorecard(j, rnd, model):
    return {
        "round": rnd,
        "model": model,
        "subjects": [
            {"axis": r["axis"], "level": int(r["level"][1]), "n": r["n"], "score": band(r["score"], 5),
             "retention": band(r["rho"], 0.1)}
            for r in j["subjects"] if r["n"] >= MIN_N
        ],
        "capability": [{"task": t, "n": c["n"], "score": band(c["score"], 5)} for t, c in j["capability"].items()],
        "sealed_probe": None,
        "audit": {"accepted": None, "rejected_batches": []},
    }  # fmt: skip


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--manifest", required=True)
    ap.add_argument("--eval-root", required=True)
    ap.add_argument("--run-id", required=True)
    ap.add_argument("--levels", required=True, help="levels_<N-1>.json")
    ap.add_argument("--round", type=int, required=True)
    ap.add_argument("--model", required=True)
    ap.add_argument("--out", required=True, help="the run's referee/ dir")
    a = ap.parse_args(argv)

    manifest = json.loads(Path(a.manifest).read_text())
    levels = json.loads(Path(a.levels).read_text())
    eps, missing = episodes(manifest, a.eval_root, a.run_id)
    j = judge(eps, levels)
    out, r = Path(a.out), a.round
    full = {"round": r, "model": a.model, "episodes": len(eps), "missing": missing, **j}
    for path, data in (
        (out / "rounds" / f"r{r}" / "scores.json", full),
        (out / "out" / "data" / f"scorecard_r{r}.json", scorecard(j, r, a.model)),
        (out / "out" / "exam" / f"results_r{r}" / "episodes.json", eps),
        (out / f"levels_{r}.json", {"round": r, "levels": j["levels"], "capability": levels.get("capability")}),
        (out / "out" / "exam" / f"levels_{r}.json", {"round": r, "levels": j["levels"]}),
    ):
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(data, indent=1, ensure_ascii=False) + "\n")
    for row in j["subjects"]:
        lo, hi = row["rho_ci95"]
        head = f"{row['axis']:10s} {row['level']} n={row['n']:3d} score {row['score']:5.1f}"
        print(f"{head} rho {row['rho']:.2f} [{lo:.2f}, {hi:.2f}]")
    print(f"capability gaps {j['capability_gaps']}; moves {j['moves']}; missing {len(missing)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
