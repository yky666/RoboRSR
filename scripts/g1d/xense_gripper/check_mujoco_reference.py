"""Native MuJoCo verification of the frozen standalone reference convention."""

import argparse
import json
from pathlib import Path
import sys

import mujoco

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
from env.robot_manager.aurora_contract import mjcf_closure_positions  # noqa: E402


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--out", type=Path, required=True)
    a = p.parse_args()
    model = mujoco.MjModel.from_xml_path(str(ROOT / "real2sim/grippers/aurora_fusion/mujoco_reference/model.xml"))
    state = mujoco.MjData(model)
    rows = []
    for gap in (0.0, 0.0425, 0.085):
        state.qpos[:] = mjcf_closure_positions(gap)
        mujoco.mj_forward(model, state)
        pts = {
            side: state.site_xpos[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, side + "_contact_center")].tolist()
            for side in ("left", "right")
        }
        measured = pts["left"][0] - pts["right"][0]
        if abs(measured - gap) > 1e-9:
            raise ValueError("MuJoCo reference gap mismatch")
        rows.append(dict(opening_m=gap, measured_site_gap_m=measured, site_native_cad_m=pts))
    a.out.write_text(
        json.dumps(
            dict(status="passed", engine=mujoco.__version__, rows=rows, actuators=model.nu, reference_only=True),
            indent=2,
        )
        + "\n"
    )


if __name__ == "__main__":
    main()
