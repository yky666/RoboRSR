"""RSI round 0 start evaluation (rsi/AGENTS.md R0.5): the base model on the first layouts of the official eval set,
checked task by task against the leaderboard. The same episodes are the calibration and the RSI starting point.

    bash scripts/robodojo.sh benchmark ... --tasks-file rsi/start_eval_tasks.txt --eval-num 10 --run-id <id>
    python scripts/datagen/rsi_start.py --eval-root eval_result --run-id <id> --out referee/rounds/r0/start.json

A task passes when the leaderboard score lies within the bootstrap 95% interval of our mean episode score, widened by
--margin points (10 episodes of a task the model never solves give a zero-width interval). The leaderboard averages
the three official sets with native episode counts; we run set 0 only, so small offsets are expected.
"""

import argparse
import json
from pathlib import Path
import random
import sys

ROOT = Path(__file__).resolve().parents[2]
LEADERBOARD = ROOT / "rsi" / "baseline" / "openwam_alpha_leaderboard.json"


def bootstrap(xs, n=2000, seed=0):
    rng = random.Random(seed)
    means = sorted(sum(rng.choices(xs, k=len(xs))) / len(xs) for _ in range(n))
    return means[int(0.025 * n)], means[int(0.975 * n) - 1]


def collect(eval_root, run_id):
    """{task: [episode score 0-100]} from the benchmark's <run_id>_<task>/_result.json files."""
    out = {}
    for f in sorted(Path(eval_root).glob("**/_result.json")):
        name = f.parent.name
        if name.startswith(run_id + "_"):
            details = json.loads(f.read_text())["details"]
            out[name[len(run_id) + 1 :]] = [100 * d["score"] for d in details.values()]
    return out


def compare(scores, leaderboard, episodes, margin, max_fail):
    rows = {}
    for task, xs in sorted(scores.items()):
        lb = leaderboard[task]["score"]
        lo, hi = bootstrap(xs)
        rows[task] = {
            "n": len(xs),
            "score": round(sum(xs) / len(xs), 2),
            "ci95": [round(lo, 2), round(hi, 2)],
            "leaderboard": lb,
            "complete": len(xs) == episodes,
            "pass": lo - margin <= lb <= hi + margin,
        }
    failed = [t for t, r in rows.items() if not (r["pass"] and r["complete"])]
    pooled = {k: round(sum(r[k] for r in rows.values()) / len(rows), 2) for k in ("score", "leaderboard")}
    return {
        "tasks": rows,
        "pooled": pooled,
        "failed": failed,
        "passed": len(failed) <= max_fail,
    }


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--eval-root", default=str(ROOT / "eval_result"))
    ap.add_argument("--run-id", required=True)
    ap.add_argument("--tasks-file", default=str(ROOT / "rsi" / "start_eval_tasks.txt"))
    ap.add_argument("--leaderboard", default=str(LEADERBOARD))
    ap.add_argument("--episodes", type=int, default=10)
    ap.add_argument("--margin", type=float, default=5.0, help="points added to each side of the interval")
    ap.add_argument("--max-fail", type=int, default=2, help="tasks allowed outside before the chain is suspect")
    ap.add_argument("--out")
    a = ap.parse_args(argv)

    tasks = Path(a.tasks_file).read_text().split()
    scores = collect(a.eval_root, a.run_id)
    missing = [t for t in tasks if t not in scores]
    if missing:
        sys.exit(f"no _result.json for run {a.run_id}: {', '.join(missing)}")
    leaderboard = json.loads(Path(a.leaderboard).read_text())["tasks"]
    report = compare({t: scores[t] for t in tasks}, leaderboard, a.episodes, a.margin, a.max_fail)
    report.update(run_id=a.run_id, episodes=a.episodes, margin=a.margin)

    for task, r in report["tasks"].items():
        flag = "" if r["pass"] and r["complete"] else "  <-- " + ("incomplete" if not r["complete"] else "off")
        lo, hi = r["ci95"]
        print(f"{task:36s} {r['score']:6.1f} [{lo:5.1f}, {hi:5.1f}]  lb {r['leaderboard']:6.2f}{flag}")
    p = report["pooled"]
    print(f"pooled {p['score']} vs leaderboard {p['leaderboard']}; off: {report['failed']}; passed={report['passed']}")
    if a.out:
        Path(a.out).parent.mkdir(parents=True, exist_ok=True)
        Path(a.out).write_text(json.dumps(report, indent=1) + "\n")
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
