"""Freeze read-only local source contributions into this dedicated robot asset.

The MuJoCo candidate is retained only as an independent reference; its placeholder
inertias and collision exclusions are never imported into the full G1-D asset.
Vendor binaries and FEM/calibration data are inventoried by hash, not copied.
"""

import argparse
import hashlib
import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--local-gripper", type=Path, required=True)
    parser.add_argument("--local-audit", type=Path, required=True)
    parser.add_argument("--flange-dir", type=Path)
    parser.add_argument("--vendor-xensim", type=Path, required=True)
    args = parser.parse_args()
    repo = Path(__file__).resolve().parents[3]
    dest = repo / "real2sim/grippers/aurora_fusion"
    dest.mkdir(exist_ok=False)
    records = []

    def copy(source, relative, role):
        target = dest / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(source, target)
        records.append(dict(path=relative, sha256=digest(target), bytes=target.stat().st_size, role=role))

    local = args.local_gripper / "v2_kinematics"
    model = ET.parse(local / "model.xml").getroot()
    copy(local / "model.xml", "mujoco_reference/model.xml", "independent kinematic reference only")
    for mesh in model.findall("asset/mesh"):
        relative = mesh.get("file")
        copy(local / relative, "mujoco_reference/" + relative, "reference visual/contact geometry")
    for name in ("assembly.json", "kinematics_check.json"):
        copy(local / name, "mujoco_reference/" + name, "inherited evidence; not rerun evidence")
    for name in ("AUDIT.md", "alignment_audit.json", "XENSE_PACKAGE_AUDIT.md"):
        copy(args.local_audit / name, "inherited_audit/" + name, "inherited source audit; not calibration")
    if args.flange_dir:
        for name in ("universal_flange.STEP", "universal_flange.pdf"):
            copy(args.flange_dir / name, "mount_reference/" + name, "unmounted geometry; G1-D mating transform unknown")
    vendor = []
    for name in ("g1-ws_table.npz", "g1-ws.npz"):
        path = args.vendor_xensim / "assets/fem" / name
        vendor.append(dict(name=name, sha256=digest(path), bytes=path.stat().st_size, bundled=False))
    (dest / "source_manifest.json").write_text(
        json.dumps(
            dict(
                schema="aurora_fusion_sources.v1",
                files=records,
                vendor_resources=vendor,
                imported_into_robot=False,
                authoritative_robot="g1d_aurora_lite_ws",
                hardware_connected=False,
                mount_transform_verified=False,
            ),
            indent=2,
            ensure_ascii=False,
        )
        + "\n"
    )


if __name__ == "__main__":
    main()
