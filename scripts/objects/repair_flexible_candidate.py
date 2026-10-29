"""Copy an existing research asset, compute named measurements and opt in to reset.

Never edits a shared asset. USD geometry and measured inputs stay unchanged.
"""

import argparse
import hashlib
import json
from pathlib import Path
import shutil

import numpy as np


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--source", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    a = p.parse_args()
    dest = a.output
    shutil.copytree(a.source, dest)
    meta = json.loads((dest / "metadata.json").read_text())
    before = hashlib.sha256((dest / "object.usdz").read_bytes()).hexdigest()
    old = meta.get("measurements_mm", {})
    kind = meta.get("asset_id", "") or (
        "Garment/long_zip_bag/" + dest.name if "long_zip_bag" in str(a.source) else "Articulation/cable/" + dest.name
    )
    if kind.startswith("Articulation/cable/"):
        mesh = np.load(dest / "backends/newton/mesh.npz")
        points = mesh["points"]
        faces = mesh["faces"]
        fingerprint = hashlib.sha256(
            np.asarray(points, dtype="<f8").tobytes() + np.asarray(faces, dtype="<i4").tobytes()
        ).hexdigest()
        if fingerprint != meta["deformable"]["mesh_sha256"]:
            raise ValueError("Canonical mesh fingerprint mismatch")
        size = np.ptp(points, axis=0)
        if not np.allclose(size, meta["physics"]["size"], atol=1e-8):
            raise ValueError("Proxy/canonical envelope differs")
        measurements = {"长度": float(size[0] * 1000), "直径": float(max(size[1:]) * 1000)}
        runtime = json.loads((dest / "runtime_config.json").read_text())
        runtime.setdefault("physics", {})["reset_full_state"] = True
        (dest / "runtime_config.json").write_text(json.dumps(runtime, indent=2) + "\n")
        definition = (
            "Canonical flat-ended Newton tube mesh; PhysX proxy metadata envelope cross-checked; "
            "printing/collision excluded"
        )
    elif kind.startswith("Garment/long_zip_bag/"):
        mesh = np.load(dest / "mesh.npz")
        points = mesh["points"]
        faces = mesh["faces"]
        groups = mesh["groups"]
        body = points[np.unique(faces[groups == 0])]
        diameter = 0.0
        for x in np.unique(np.round(body[:, 0], 12)):
            ring = body[abs(body[:, 0] - x) < 1e-9]
            diameter = max(diameter, float(np.linalg.norm(ring[:, None, 1:] - ring[None, :, 1:], axis=-1).max()))
        measurements = {"长度": float(np.ptp(body[:, 0]) * 1000), "直径": diameter * 1000}
        definition = (
            "Body-only axial span; maximum body cross-section chord over all axial rows; handles/buckles excluded"
        )
    else:
        raise ValueError("Only the audited cable and zip-bag candidates are supported")
    meta["legacy_measurements_mm"] = old
    meta["measurements_mm"] = measurements
    (dest / "metadata.json").write_text(json.dumps(meta, ensure_ascii=False, indent=2) + "\n")
    audit = dict(
        source=str(a.source),
        object_sha256=before,
        geometry_unchanged=True,
        previous_measurements_mm=old,
        measurements_mm=measurements,
        definition=definition,
        scope="Output annotation/runtime repair only; no registration or material calibration change",
    )
    (dest / "review_patch.json").write_text(json.dumps(audit, ensure_ascii=False, indent=2) + "\n")
    assert hashlib.sha256((dest / "object.usdz").read_bytes()).hexdigest() == before
    print(json.dumps(audit, ensure_ascii=False), flush=True)


if __name__ == "__main__":
    main()
