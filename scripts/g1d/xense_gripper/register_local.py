"""Import this body's reviewed registration into an explicitly selected local platform database."""

import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "src/real2sim_platform"))
from server import Store, scan  # noqa: E402


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--db", type=Path, required=True)
    parser.add_argument("--assets", type=Path, default=ROOT / "Assets")
    args = parser.parse_args()
    body = "g1d_aurora_lite_ws"
    doc = json.loads((ROOT / f"real2sim/embodiments/{body}/registration.json").read_text())
    store = Store(args.db)
    current = next((item for item in store.list("embodiment") if item["id"] == body), {})
    if current.get("reg"):
        raise ValueError("Existing user registration will not be overwritten")
    registration = {k: v for k, v in doc.items() if k not in ("embodiment", "owner", "updated_by", "updated_at")}
    store.update(
        "embodiment", body, {"registered": True, "reg": registration, "owner": doc["owner"]["username"]}, "CAD 本地导入"
    )
    scan(store, args.assets.resolve(), [])
    print(f"Imported {body}; all calibration items remain pending")


if __name__ == "__main__":
    main()
