"""Optional deformable registration and evidence contract, independent of a solver."""

import hashlib
import json
import math

FAMILIES = {"cloth", "garment", "rope", "bag"}
MEASUREMENTS = {"thickness_mm", "areal_density_g_m2", "diameter_mm", "linear_density_g_m"}
COMMON_CHECKS = (
    "geometry_mass",
    "deformation",
    "contact",
    "self_collision",
    "reset_replay",
    "shutdown",
    "material_fit",
)


def validate(profile):
    """Return a registration error without converting estimates into measurements."""
    if profile is None:
        return None
    if not isinstance(profile, dict):
        return "柔性建模信息必须是对象"
    if profile.get("family", "") not in (*FAMILIES, ""):
        return "柔性类别必须是布料、衣服、绳子或布袋"
    if profile.get("backend", "") not in ("", "auto", "physx", "newton"):
        return "柔性后端必须是 auto、physx 或 newton"
    for key in ("rest_state", "structure", "test_data"):
        if key in profile and (not isinstance(profile[key], str) or len(profile[key]) > 10000):
            return f"{key} 必须是长度不超过 10000 的文本"
    values = profile.get("measurements", {})
    if not isinstance(values, dict) or set(values) - MEASUREMENTS:
        return "柔性测量字段无效"
    for name, item in values.items():
        if not isinstance(item, dict):
            return f"{name} 必须包含数值、来源和依据"
        value, source, evidence = item.get("value"), item.get("source", ""), item.get("evidence", "")
        if value is not None and (
            isinstance(value, bool) or not isinstance(value, int | float) or not math.isfinite(value) or value <= 0
        ):
            return f"{name} 必须是正的有限数值"
        if source not in ("", "unmeasured", "estimated", "measured") or not isinstance(evidence, str):
            return f"{name} 来源或依据无效"
        if source == "measured" and (value is None or not evidence.strip()):
            return f"{name} 标为实测前，请填写数值和测量记录依据"
    return None


def registration_fingerprint(profile, mass_g=None, dimensions_mm=None):
    value = {"deformable": profile, "mass_g": mass_g, "dimensions_mm": dimensions_mm}
    return hashlib.sha256(json.dumps(value, sort_keys=True, ensure_ascii=False).encode()).hexdigest()


def acceptance(profile, report, mesh_sha256, mass_g=None, dimensions_mm=None):
    """Fail closed: a generic drop test or software baseline cannot accept a real flexible asset."""
    if not isinstance(profile, dict) or profile.get("family") not in tuple(FAMILIES):
        return None
    required = list(COMMON_CHECKS)
    if profile["family"] in {"garment", "bag"}:
        required.append("seams_openings")
    if profile["family"] == "rope":
        required.append("bend_twist")
    report = report if isinstance(report, dict) else {}
    checks = report.get("checks") if isinstance(report.get("checks"), dict) else {}
    rows = {}
    for name in required:
        item = checks.get(name)
        valid = isinstance(item, dict) and isinstance(item.get("evidence"), str) and item["evidence"].strip()
        rows[name] = item["status"] if valid and item.get("status") in ("pass", "fail") else "pending"
    identity_ok = bool(
        isinstance(mesh_sha256, str)
        and len(mesh_sha256) == 64
        and all(c in "0123456789abcdef" for c in mesh_sha256)
        and report.get("mesh_sha256") == mesh_sha256
    )
    registration_sha256 = registration_fingerprint(profile, mass_g, dimensions_mm)
    registration_ok = report.get("registration_sha256") == registration_sha256
    calibrated = report.get("material_calibrated") is True
    if not calibrated and rows["material_fit"] == "pass":
        rows["material_fit"] = "pending"
    status = "fail" if "fail" in rows.values() else "pending"
    if (
        not validate(profile)
        and identity_ok
        and registration_ok
        and calibrated
        and all(v == "pass" for v in rows.values())
    ):
        status = "pass"
    return {
        "status": status,
        "family": profile["family"],
        "checks": rows,
        "mesh_matches": identity_ok,
        "registration_matches": registration_ok,
        "registration_sha256": registration_sha256,
        "material_calibrated": calibrated,
    }
