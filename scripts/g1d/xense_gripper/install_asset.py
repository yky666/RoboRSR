"""Install the frozen robot into a private asset library; never overwrite a shared body."""

import argparse
import hashlib
import json
from pathlib import Path
import shutil

ROOT = Path(__file__).resolve().parents[3]


def install(source, assets):
    destination = assets / "Robots/g1d_aurora_lite_ws"
    if destination.exists():
        raise FileExistsError(f"Refusing to overwrite existing robot: {destination}")
    for relative, expected in json.loads((source / "sha256.json").read_text()).items():
        path = (source / relative).resolve()
        if not path.is_relative_to(source.resolve()):
            raise ValueError("Source manifest escapes model directory")
        if hashlib.sha256(path.read_bytes()).hexdigest() != expected:
            raise ValueError(f"Frozen asset checksum mismatch: {relative}")
    shutil.copytree(source, destination)
    print(destination)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=ROOT / "real2sim/robot/g1d_aurora_lite_ws")
    parser.add_argument(
        "--assets", type=Path, required=True, help="Use an isolated asset library, not a shared symlink"
    )
    args = parser.parse_args()
    install(args.source.resolve(), args.assets.resolve())


if __name__ == "__main__":
    main()
