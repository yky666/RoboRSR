"""Run an RSI exam's jobs on one model (rsi/AGENTS.md 5.3 step 2, gate G2): the referee's ops tool.

    python scripts/datagen/rsi_exam_run.py --manifest ../rounds/r1/manifest.json --run-id exam_r1 \
        --lanes 0:3,1:4,2:5 --ops ../ops/ops.jsonl --env1 fold_clothes

One policy server per task and lane, the task's jobs (seed folder x env config) as clients against it. A job whose
result dir gets no new file for --stall-min minutes is killed (Isaac ignores SIGTERM) and rerun whole with the same
run id, which resumes it; at most --retries reruns. Every dispatch, stall and rerun is a line in ops.jsonl. Tasks in
--env1 run with one parallel env (cloth diverges with ten), through <env cfg>_env1 copies of their env configs.
G2 at the end: every job exited 0 and its _result.json has eval_time == the planned episodes.
"""

import argparse
from collections import defaultdict
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import threading
import time

ROOT = Path(__file__).resolve().parents[2]
POLICY = dict(dir="XPolicyLab/policy/OpenWAM", ckpt="OpenWAM-Alpha-Sim-RoboDojo", action="ee")
_ops_lock = threading.Lock()


def log_op(path, **row):
    with _ops_lock, open(path, "a") as f:
        f.write(json.dumps({"time": time.strftime("%Y-%m-%dT%H:%M:%S"), **row}) + "\n")


def assign(jobs, lanes, env1):
    """{lane: [task, ...]}, greedy on cost: one Isaac start + one batch per job, env1 tasks one batch per episode."""
    cost = defaultdict(float)
    for j in jobs:
        cost[j["task"]] += 1 + (j["episodes"] if j["task"] in env1 else 1)
    load = {lane: 0.0 for lane in lanes}
    out = {lane: [] for lane in lanes}
    for task in sorted(cost, key=cost.get, reverse=True):
        lane = min(load, key=load.get)
        out[lane].append(task)
        load[lane] += cost[task]
    return out


def make_env1(env_cfg, assets):
    """<env cfg>_env1: same config on sim_config_env1 (num_envs 1), sharing the layout folders."""
    name = f"{env_cfg}_env1"
    sim = ROOT / "env_cfg" / "sim"
    if not (sim / "sim_config_env1.yml").exists():
        text = (sim / "sim_config.yml").read_text()
        assert "  num_envs: 10\n" in text
        (sim / "sim_config_env1.yml").write_text(text.replace("  num_envs: 10\n", "  num_envs: 1\n"))
    src = (ROOT / "env_cfg" / f"{env_cfg}.yml").read_text()
    text = src.replace(f"config_name: {env_cfg}\n", f"config_name: {name}\n").replace(
        "  sim: sim_config\n", "  sim: sim_config_env1\n"
    )
    assert text != src
    (ROOT / "env_cfg" / f"{name}.yml").write_text(text)
    link = Path(assets) / "Eval_Layout" / "RoboDojo" / name
    if not link.exists():
        link.symlink_to(env_cfg)
    return name


def result_dir(task, run_id):
    found = sorted((ROOT / "eval_result" / "RoboDojo" / task).glob(f"**/{run_id}"))
    return found[0] if found else None


def newest_mtime(d):
    times = [p.stat().st_mtime for p in d.rglob("*") if p.is_file()] if d else []
    return max(times, default=0.0)


def kill_group(proc):
    try:
        os.killpg(proc.pid, signal.SIGKILL)
    except ProcessLookupError:
        pass
    proc.wait()


def wait_port(port, timeout):
    end = time.time() + timeout
    while time.time() < end:
        with socket.socket() as s:
            if s.connect_ex(("127.0.0.1", port)) == 0:
                return True
        time.sleep(10)
    return False


def run_job(job, lane, port, a, logs):
    env_cfg = job["run_env_cfg"]
    rid = f"{a.run_id}_{job['task']}_{env_cfg}_{job['seed']}"
    cmd = [
        "bash", "scripts/robodojo.sh", "client", "--task", job["task"], "--policy-dir", POLICY["dir"],
        "--policy-host", "127.0.0.1", "--policy-port", str(port), "--eval-num", str(job["episodes"]),
        "--env-cfg", env_cfg, "--seed", str(job["seed"]), "--env-gpu", str(lane[1]),
        "--ckpt", POLICY["ckpt"], "--action-type", POLICY["action"],
    ]  # fmt: skip
    for attempt in range(a.retries + 1):
        log_op(a.ops, job=rid, action="dispatch" if attempt == 0 else "rerun", lane=list(lane), attempt=attempt)
        with open(logs / f"{rid}.log", "a") as f:
            proc = subprocess.Popen(
                cmd, cwd=ROOT, stdout=f, stderr=subprocess.STDOUT, start_new_session=True,
                env={**os.environ, "ROBODOJO_RUN_ID": rid},
            )  # fmt: skip
            start = time.time()
            while proc.poll() is None:
                time.sleep(60)
                if time.time() - max(start, newest_mtime(result_dir(job["task"], rid))) > a.stall_min * 60:
                    log_op(a.ops, job=rid, action="stall_kill", minutes=a.stall_min)
                    kill_group(proc)
                    break
        if proc.returncode == 0:
            break
    rdir = result_dir(job["task"], rid)
    return {"job": rid, "rc": proc.returncode, "dir": str(rdir or ""), "attempts": attempt + 1}


def run_lane(lane, port, tasks, jobs, a, logs, results):
    for task in tasks:
        server = subprocess.Popen(
            [
                "bash", "scripts/robodojo.sh", "server", "--policy-dir", POLICY["dir"], "--task", task,
                "--ckpt", POLICY["ckpt"], "--policy-env", a.policy_env, "--action-type", POLICY["action"],
                "--policy-gpu", str(lane[0]), "--policy-port", str(port), "--bind-host", "127.0.0.1",
            ],
            cwd=ROOT, stdout=open(logs / f"server_{task}.log", "w"), stderr=subprocess.STDOUT, start_new_session=True,
        )  # fmt: skip
        log_op(a.ops, job=f"server_{task}", action="dispatch", lane=list(lane), port=port)
        if not wait_port(port, 900):
            log_op(a.ops, job=f"server_{task}", action="server_timeout")
            kill_group(server)
            results.extend({"job": f"{task}:{j['seed']}", "rc": "no_server"} for j in jobs if j["task"] == task)
            continue
        for job in (j for j in jobs if j["task"] == task):
            results.append({**run_job(job, lane, port, a, logs), "planned": job["episodes"]})
        kill_group(server)
        log_op(a.ops, job=f"server_{task}", action="stopped")


def gate(results):
    """G2: exit 0 and eval_time == planned for every job."""
    bad = []
    for r in results:
        f = Path(r["dir"]) / "_result.json" if r.get("dir") else None
        done = json.loads(f.read_text())["eval_time"] if f and f.exists() else 0
        r["eval_time"] = done
        if r["rc"] != 0 or done != r["planned"]:
            bad.append(r["job"])
    return bad


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--manifest", required=True)
    ap.add_argument("--run-id", required=True)
    ap.add_argument("--lanes", required=True, help="policy_gpu:env_gpu,... one lane each")
    ap.add_argument("--ops", required=True)
    ap.add_argument("--policy-env", default="/root/miniconda3/envs/openwam-bench")
    ap.add_argument("--assets", default=str(ROOT / "Assets"))
    ap.add_argument("--env1", default="", help="comma-separated tasks run with one parallel env")
    ap.add_argument("--stall-min", type=float, default=30)
    ap.add_argument("--retries", type=int, default=3)
    ap.add_argument("--base-port", type=int, default=18600)
    ap.add_argument("--out")
    a = ap.parse_args(argv)

    jobs = json.loads(Path(a.manifest).read_text())["jobs"]
    env1 = set(filter(None, a.env1.split(",")))
    for j in jobs:
        j["run_env_cfg"] = make_env1(j["env_cfg"], a.assets) if j["task"] in env1 else j["env_cfg"]
    lanes = [tuple(int(x) for x in s.split(":")) for s in a.lanes.split(",")]
    plan = assign(jobs, lanes, env1)
    logs = ROOT / "logs" / a.run_id
    logs.mkdir(parents=True, exist_ok=True)
    Path(a.ops).parent.mkdir(parents=True, exist_ok=True)
    log_op(a.ops, job=a.run_id, action="start", plan={f"{p}:{e}": t for (p, e), t in plan.items()})

    results = []
    threads = [
        threading.Thread(target=run_lane, args=(lane, a.base_port + i, plan[lane], jobs, a, logs, results))
        for i, lane in enumerate(lanes)
    ]
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    bad = gate(results)
    report = {"run_id": a.run_id, "jobs": len(results), "failed": bad, "passed": not bad, "results": results}
    log_op(a.ops, job=a.run_id, action="end", passed=not bad, failed=bad)
    Path(a.out or logs / "g2.json").write_text(json.dumps(report, indent=1) + "\n")
    print(f"G2 {'passed' if not bad else 'failed'}: {len(results)} jobs, failed {bad}")
    return 0 if not bad else 1


if __name__ == "__main__":
    raise SystemExit(main())
