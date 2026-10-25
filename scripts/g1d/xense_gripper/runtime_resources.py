"""Verify a separately provisioned vendor runtime without opening devices."""

import argparse
import hashlib
import json
from pathlib import Path
import platform
import sys
import zipfile


def install_archive(manifest, archive, destination):
    """Provision the original vendor archive privately; never extract arbitrary paths."""
    data = json.loads(manifest.read_text())
    digest = hashlib.sha256()
    with archive.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    if digest.hexdigest() not in {entry["sha256"] for entry in data["archives"]}:
        raise ValueError("Vendor archive digest is not in the verified manifest")
    expected = {entry["path"]: entry for entry in data["files"]}
    destination.mkdir(parents=True, exist_ok=False)
    with zipfile.ZipFile(archive) as source:
        for member in source.infolist():
            if "pip_prebundle/" not in member.filename or member.is_dir():
                continue
            relative = member.filename.split("pip_prebundle/", 1)[1]
            if relative not in expected:
                continue
            if Path(relative).is_absolute() or ".." in Path(relative).parts:
                raise ValueError("Unsafe archive path")
            payload = source.read(member)
            if hashlib.sha256(payload).hexdigest() != expected[relative]["sha256"]:
                raise ValueError("Archive member digest mismatch: " + relative)
            target = destination / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(payload)
    return verify(manifest, destination)


def verify(manifest, runtime):
    data = json.loads(manifest.read_text())
    failed = []
    for entry in data["files"]:
        p = runtime / entry["path"]
        if (
            not p.is_file()
            or p.stat().st_size != entry["bytes"]
            or hashlib.sha256(p.read_bytes()).hexdigest() != entry["sha256"]
        ):
            failed.append(entry["path"])
    return dict(
        passed=not failed,
        verified_files=len(data["files"]),
        failed=failed,
        required_python="3.10",
        current_python=platform.python_version(),
        abi_compatible=sys.version_info[:2] == (3, 10),
        hardware_connected=False,
        redistribution_authorized=data["public_redistribution_authorized"],
    )


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--manifest", type=Path, required=True)
    p.add_argument("--runtime", type=Path, required=True)
    p.add_argument("--out", type=Path, required=True)
    p.add_argument("--install-archive", type=Path, help="Original licensed vendor ZIP; --runtime must not exist")
    a = p.parse_args()
    r = (
        install_archive(a.manifest, a.install_archive, a.runtime)
        if a.install_archive
        else verify(a.manifest, a.runtime)
    )
    a.out.write_text(json.dumps(r, indent=2) + "\n")
    print(json.dumps({k: v for k, v in r.items() if k != "failed"}))
    sys.exit(0 if r["passed"] and r["abi_compatible"] else 1)
