"""Verify then apply an explicitly selected delivery overlay; never operates Git.

Copy this helper to the package root as APPLY_TO_REPO.py. Without --apply it
only checks hashes. An existing file is accepted only if it matches the stated
base, the previous reviewed release, or the final file exactly.
"""

import argparse
import hashlib
import json
from pathlib import Path
import shutil


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--target", type=Path, required=True)
    parser.add_argument("--apply", action="store_true")
    args = parser.parse_args()
    source = Path(__file__).resolve().parent
    target = args.target.resolve()
    if not target.is_dir():
        raise SystemExit("Target directory does not exist")
    records = json.loads((source / "apply_manifest.json").read_text())["files"]
    pending = []
    for record in records:
        relative = Path(record["path"])
        src = (source / "repo_overlay" / relative).resolve()
        dst = (target / relative).resolve()
        if not src.is_relative_to(source / "repo_overlay") or not dst.is_relative_to(target):
            raise SystemExit("Manifest escapes package or target")
        if sha(src) != record["after_sha256"]:
            raise SystemExit(f"Package hash mismatch: {relative}")
        if dst.exists() and not dst.is_file():
            raise SystemExit(f"Existing path is not a file: {relative}")
        current = sha(dst) if dst.is_file() else None
        if current == record["after_sha256"]:
            continue
        if current not in record["allowed_previous_sha256"]:
            raise SystemExit(f"Existing content differs; refusing overwrite: {relative}")
        pending.append((src, dst))
    if args.apply:
        for src, dst in pending:
            dst.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(src, dst)
        for record in records:
            if sha(target / record["path"]) != record["after_sha256"]:
                raise SystemExit("Post-apply hash mismatch")
    print(
        json.dumps(
            dict(
                mode="applied" if args.apply else "verified only",
                files_to_apply=len(pending),
                checked_files=len(records),
                git_modified_by_helper=False,
            )
        )
    )


if __name__ == "__main__":
    main()
