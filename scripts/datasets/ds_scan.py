"""Count episodes, frames and hours of local datasets, inside archives too, without unpacking them to disk.

    ds_scan.py archive NAME FILE...   one unit per archive; split parts (*.tar.gz.NN, *.tar.part-NNNN, *.tar-aa) joined
    ds_scan.py dir NAME MODE ROOT     MODE: hdf5 | mcap | bag | mp4 | parquet; files found under ROOT
Sum the units with ds_report.py.

Archives are read as one stream (gzip / zstd decompressed in memory). Per member: name and size go to a listing;
parquet members give their row count and timestamp range; HDF5 members are written to a scratch file just long enough
to read their dataset shapes; a few MP4 per unit are probed for fps and frame count; LeRobot meta files are kept.
Results go to OUT/NAME/units/*.json (one per unit, so a rerun skips finished units). Nothing is written next to the
datasets themselves.
"""

from collections import Counter, defaultdict
from concurrent.futures import ProcessPoolExecutor, as_completed
import glob
import gzip
import io
import json
import os
import re
import shutil
import subprocess
import sys
import tarfile
import time
import zipfile

OUT = os.environ.get("DS_SCAN_OUT", "/mnt/data/datasets/_stats")
SCRATCH = os.environ.get("DS_SCAN_TMP", "/root/scan_tmp")
WORKERS = int(os.environ.get("DS_SCAN_WORKERS", "12"))
MP4_PROBES = 3
KEEP_META = re.compile(
    r"(^|/)(meta/[^/]+\.(json|jsonl)|info\.json|dataset_info\.json|episodes\.jsonl|tasks\.jsonl|stats\.json)$"
)
SPLIT = re.compile(r"^(.*?)(\.tar\.gz\.\d+|\.tar\.part-\d+|\.tar-[a-z]{2}|\.tgz\.\d+|\.zip\.\d+)$")


def unit_name(path):
    return re.sub(r"[^\w.-]+", "_", path.strip("/"))[-180:]


def group_units(files):
    """Split parts of one archive become one unit, in order."""
    units = defaultdict(list)
    for f in sorted(files):
        m = SPLIT.match(f)
        units[m[1] + ".SPLIT" if m else f].append(f)
    return units


def opener(parts):
    """A byte stream of the archive's tar content, and the process feeding it (or None)."""
    first = parts[0]
    gz = ".tar.gz" in first or first.endswith(".tgz") or re.search(r"\.tgz\.\d+$", first)
    zst = ".tar.zst" in first or first.endswith(".tzst")
    if len(parts) == 1 and not (gz or zst):
        return open(first, "rb"), None
    cat = subprocess.Popen(["cat", *parts], stdout=subprocess.PIPE, bufsize=1 << 20)
    if not (gz or zst):
        return cat.stdout, cat
    dec = subprocess.Popen(
        ["zstd", "-dcq"] if zst else ["gzip", "-dc"], stdin=cat.stdout, stdout=subprocess.PIPE, bufsize=1 << 20
    )
    cat.stdout.close()
    return dec.stdout, dec


def h5_summary(path):
    import h5py

    lens, attrs = Counter(), {}
    with h5py.File(path, "r") as f:
        for k, v in f.attrs.items():
            attrs[k] = str(v)[:80]

        def visit(name, obj):
            if isinstance(obj, h5py.Dataset) and obj.shape:
                lens[f"{name}:{obj.shape[0]}"] += 1
            elif isinstance(obj, h5py.Group):
                for k in ("num_samples", "fps", "frequency", "control_freq"):
                    if k in obj.attrs:
                        attrs[f"{name}@{k}"] = str(obj.attrs[k])

        f.visititems(visit)
    # frames of one episode: the most common first-axis length among its datasets
    by_len = Counter()
    for key, n in lens.items():
        by_len[int(key.rsplit(":", 1)[1])] += n
    demos = sum(1 for k in lens if re.match(r"data/demo_\d+/actions:", k))
    frames = (
        sum(int(k.rsplit(":", 1)[1]) for k in lens if re.match(r"data/demo_\d+/actions:", k))
        if demos
        else (by_len.most_common(1)[0][0] if by_len else 0)
    )
    return {"frames": frames, "demos": demos, "keys": sorted(lens)[:40], "attrs": dict(list(attrs.items())[:20])}


def probe_mp4(path):
    out = subprocess.run(
        [
            "ffprobe",
            "-v",
            "error",
            "-select_streams",
            "v:0",
            "-show_entries",
            "stream=r_frame_rate,nb_frames:format=duration",
            "-of",
            "json",
            path,
        ],
        capture_output=True,
        text=True,
        timeout=120,
    ).stdout
    try:
        j = json.loads(out)
        s = (j.get("streams") or [{}])[0]
        return {
            "rate": s.get("r_frame_rate"),
            "frames": s.get("nb_frames"),
            "duration": j.get("format", {}).get("duration"),
        }
    except ValueError:
        return {"error": out[:80]}


def parquet_summary(data):
    import pyarrow.parquet as pq

    pf = pq.ParquetFile(io.BytesIO(data))
    rec = {"rows": pf.metadata.num_rows}
    if "timestamp" in pf.schema_arrow.names:
        t = pf.read(columns=["timestamp"])["timestamp"].to_pylist()
        if len(t) > 1:
            rec.update(t0=t[0], t1=t[-1], dt=round(t[1] - t[0], 6))
    return rec


def new_unit():
    return {
        "kinds": Counter(),
        "bytes": Counter(),
        "parquet": [],
        "hdf5": [],
        "mp4": [],
        "npy": [],
        "meta": [],
        "errors": [],
    }


def take(rec, name, size, read, keep_dir, scratch):
    """Look at one member: `read()` returns its bytes (members are read in order, once)."""
    base = name.rsplit("/", 1)[-1].lower()
    ext = next(
        (e for e in (".parquet", ".hdf5", ".h5", ".mp4", ".npy", ".json", ".jsonl") if base.endswith(e)), "other"
    )
    rec["kinds"][ext] += 1
    rec["bytes"][ext] += size
    try:
        if ext == ".parquet" and size < (2 << 30) and "/.cache/" not in name:
            rec["parquet"].append({"name": name, **parquet_summary(read())})
        elif ext in (".hdf5", ".h5") and size < (40 << 30):
            tmp = os.path.join(scratch, f"{os.getpid()}.h5")
            with open(tmp, "wb") as f:
                f.write(read())
            rec["hdf5"].append({"name": name, **h5_summary(tmp)})
            os.unlink(tmp)
        elif ext == ".mp4" and len(rec["mp4"]) < MP4_PROBES and size < (4 << 30):
            tmp = os.path.join(scratch, f"{os.getpid()}.mp4")
            with open(tmp, "wb") as f:
                f.write(read())
            rec["mp4"].append({"name": name, **probe_mp4(tmp)})
            os.unlink(tmp)
        elif ext == ".npy" and len(rec["npy"]) < 5 and size < (64 << 20):
            import numpy as np

            a = np.load(io.BytesIO(read()), allow_pickle=True)
            rec["npy"].append(
                {
                    "name": name,
                    "shape": list(a.shape),
                    "dtype": str(a.dtype),
                    "sample": str(a.item() if a.shape == () else a[:2])[:400],
                }
            )
        elif KEEP_META.search(name) and size < (64 << 20) and "/.cache/" not in name:
            dest = os.path.join(keep_dir, name.strip("/"))
            os.makedirs(os.path.dirname(dest), exist_ok=True)
            with open(dest, "wb") as f:
                f.write(read())
            rec["meta"].append(name)
    except Exception as e:  # one bad member must not lose the rest of the archive
        rec["errors"].append(f"{name}: {type(e).__name__}: {str(e)[:120]}")


def scan_archive(name, key, parts):
    out_dir = os.path.join(OUT, name)
    done = os.path.join(out_dir, "units", unit_name(key) + ".json")
    if os.path.exists(done):
        return key, "skip"
    keep_dir = os.path.join(out_dir, "meta", unit_name(key))
    scratch = os.path.join(SCRATCH, f"{os.getpid()}")
    os.makedirs(scratch, exist_ok=True)
    os.makedirs(os.path.dirname(done), exist_ok=True)
    rec, t0 = new_unit(), time.time()
    listing = gzip.open(os.path.join(out_dir, "units", unit_name(key) + ".list.tsv.gz"), "wt")
    try:
        if parts[0].endswith(".zip"):
            with zipfile.ZipFile(parts[0]) as zf:
                for zi in zf.infolist():
                    if zi.is_dir():
                        continue
                    listing.write(f"{zi.filename}\t{zi.file_size}\n")
                    take(rec, zi.filename, zi.file_size, lambda zi=zi: zf.read(zi), keep_dir, scratch)
        else:
            stream, proc = opener(parts)
            with tarfile.open(fileobj=stream, mode="r|") as tf:
                for m in tf:
                    if not m.isfile():
                        continue
                    listing.write(f"{m.name}\t{m.size}\n")
                    take(rec, m.name, m.size, lambda m=m: tf.extractfile(m).read(), keep_dir, scratch)
            if proc:
                proc.wait()
                if proc.returncode:
                    rec["errors"].append(f"decompressor exit {proc.returncode}")
    except Exception as e:
        rec["errors"].append(f"stream: {type(e).__name__}: {str(e)[:200]}")
    listing.close()
    shutil.rmtree(scratch, ignore_errors=True)
    rec.update(unit=key, parts=parts, seconds=round(time.time() - t0), in_bytes=sum(os.path.getsize(p) for p in parts))
    rec["kinds"], rec["bytes"] = dict(rec["kinds"]), dict(rec["bytes"])
    tmp = done + ".tmp"
    with open(tmp, "w") as f:
        json.dump(rec, f, ensure_ascii=False)
    os.replace(tmp, done)
    return key, f"{rec['seconds']}s, {len(rec['errors'])} errors"


def scan_file(name, mode, path):
    """dir mode: one result per file, collected per parent directory into units of up to 500 files."""
    if mode == "hdf5":
        return {"name": path, **h5_summary(path)}
    if mode == "mp4":
        return {"name": path, **probe_mp4(path)}
    if mode == "parquet":
        with open(path, "rb") as f:
            return {"name": path, **parquet_summary(f.read())}
    if mode == "mcap":
        sys.path.insert(0, "/root/scanlib")
        from mcap.reader import make_reader

        with open(path, "rb") as f:
            st = make_reader(f).get_summary().statistics
        return {
            "name": path,
            "messages": st.message_count,
            "t0": st.message_start_time / 1e9,
            "t1": st.message_end_time / 1e9,
            "channels": dict(st.channel_message_counts),
        }
    if mode == "bag":
        sys.path.insert(0, "/root/scanlib")
        from rosbags.rosbag1 import Reader

        with Reader(path) as r:
            return {
                "name": path,
                "messages": r.message_count,
                "t0": r.start_time / 1e9,
                "t1": r.end_time / 1e9,
                "topics": {c.topic: c.msgcount for c in r.connections},
            }
    raise ValueError(mode)


def scan_dir_unit(name, mode, key, files):
    out_dir = os.path.join(OUT, name, "units")
    done = os.path.join(out_dir, unit_name(key) + ".json")
    if os.path.exists(done):
        return key, "skip"
    os.makedirs(out_dir, exist_ok=True)
    t0, rows, errors = time.time(), [], []
    for p in files:
        try:
            rows.append(scan_file(name, mode, p))
        except Exception as e:
            errors.append(f"{p}: {type(e).__name__}: {str(e)[:120]}")
    rec = {
        "unit": key,
        "mode": mode,
        "files": len(files),
        "rows": rows,
        "errors": errors,
        "seconds": round(time.time() - t0),
    }
    with open(done + ".tmp", "w") as f:
        json.dump(rec, f, ensure_ascii=False)
    os.replace(done + ".tmp", done)
    return key, f"{rec['seconds']}s, {len(errors)} errors"


def run(jobs):
    with ProcessPoolExecutor(WORKERS) as ex:
        futs = [ex.submit(*j) for j in jobs]
        for fu in as_completed(futs):
            try:
                key, msg = fu.result()
                print(time.strftime("%H:%M:%S"), key, msg, flush=True)
            except Exception as e:
                print(time.strftime("%H:%M:%S"), "FAILED", type(e).__name__, e, flush=True)


def main():
    cmd, name = sys.argv[1], sys.argv[2]
    if cmd == "archive":
        files = [f for pat in sys.argv[3:] for f in glob.glob(pat, recursive=True)]
        units = group_units(files)
        print(f"{name}: {len(units)} units from {len(files)} files", flush=True)
        run([(scan_archive, name, k, v) for k, v in sorted(units.items())])
    elif cmd == "dir":
        mode, root = sys.argv[3], sys.argv[4]
        suffix = {
            "hdf5": (".hdf5", ".h5"),
            "mp4": (".mp4",),
            "parquet": (".parquet",),
            "mcap": (".mcap",),
            "bag": (".bag",),
        }[mode]
        groups = defaultdict(list)
        for dp, ds, fs in os.walk(root):
            ds[:] = [d for d in ds if not d.startswith(".")]
            for f in fs:
                if f.lower().endswith(suffix):
                    groups[dp].append(os.path.join(dp, f))
        jobs = []
        for dp, fs in sorted(groups.items()):
            fs.sort()
            for i in range(0, len(fs), 500):
                jobs.append((scan_dir_unit, name, mode, f"{os.path.relpath(dp, root)}#{i // 500}", fs[i : i + 500]))
        print(f"{name}: {len(jobs)} units, {sum(len(v) for v in groups.values())} files", flush=True)
        run(jobs)


if __name__ == "__main__":
    main()
