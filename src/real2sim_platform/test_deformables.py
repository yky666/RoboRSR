"""Checks of the deformable registration and acceptance contract (no solver needed)."""

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))
import deformables


def test_validate():
    assert deformables.validate(None) is None
    assert deformables.validate({"family": "cloth", "backend": "physx"}) is None
    assert deformables.validate([]) == "柔性建模信息必须是对象"
    assert "柔性类别" in deformables.validate({"family": "jelly"})
    assert "柔性后端" in deformables.validate({"family": "rope", "backend": "mujoco"})
    assert deformables.validate({"measurements": {"weight": {"value": 1}}}) == "柔性测量字段无效"
    for bad in (0, -1, True, float("nan")):
        assert "正的有限数值" in deformables.validate({"measurements": {"thickness_mm": {"value": bad}}})
    measured_blank = {"measurements": {"thickness_mm": {"value": 0.4, "source": "measured", "evidence": " "}}}
    assert "测量记录依据" in deformables.validate(measured_blank)  # an estimate cannot be called measured
    estimated = {"measurements": {"thickness_mm": {"value": 0.4, "source": "estimated", "evidence": ""}}}
    assert deformables.validate(estimated) is None


def test_acceptance_fails_closed():
    profile = {"family": "rope"}
    assert deformables.acceptance({"family": "unknown"}, {}, "0" * 64) is None
    checks = ["geometry_mass", "deformation", "contact", "self_collision", "reset_state", "shutdown", "material_fit"]
    empty = deformables.acceptance(profile, {}, "0" * 64)
    assert empty["status"] == "pending" and set(empty["checks"]) == {*checks, "bend_twist"}  # rope adds bend/twist
    assert set(deformables.acceptance({"family": "bag"}, {}, "0" * 64)["checks"]) >= {"seams_openings"}
    for family in ("shell", "volume"):  # a paper cup or a plush is accepted on squashing and springing back
        assert deformables.validate({"family": family}) is None
        assert "compression_recovery" in deformables.acceptance({"family": family}, {}, "0" * 64)["checks"]

    mesh = "ab" * 32
    report = {
        "mesh_sha256": mesh,
        "registration_sha256": deformables.registration_fingerprint(profile, 12.0, "长 300"),
        "material_calibrated": True,
        "checks": {name: {"status": "pass", "evidence": "视频与日志"} for name in [*checks, "bend_twist"]},
    }
    assert deformables.acceptance(profile, report, mesh, 12.0, "长 300")["status"] == "pass"
    assert deformables.acceptance(profile, report, mesh, 13.0, "长 300")["status"] == "pending"  # registration changed
    assert deformables.acceptance(profile, report, "cd" * 32, 12.0, "长 300")["status"] == "pending"  # other mesh
    uncalibrated = deformables.acceptance(profile, {**report, "material_calibrated": False}, mesh, 12.0, "长 300")
    assert uncalibrated["checks"]["material_fit"] == "pending" and uncalibrated["status"] == "pending"
    no_evidence = {**report, "checks": {**report["checks"], "contact": {"status": "pass", "evidence": ""}}}
    assert deformables.acceptance(profile, no_evidence, mesh, 12.0, "长 300")["checks"]["contact"] == "pending"
    failed = {**report, "checks": {**report["checks"], "shutdown": {"status": "fail", "evidence": "崩溃日志"}}}
    assert deformables.acceptance(profile, failed, mesh, 12.0, "长 300")["status"] == "fail"


def test_replay_diagnostic_does_not_replace_state_reset():
    profile = {"family": "volume"}
    report = {
        "checks": {
            k: {"status": "pass", "evidence": "report.json"}
            for k in (*deformables.COMMON_CHECKS, "compression_recovery")
        },
        "mesh_sha256": "a" * 64,
        "material_calibrated": True,
        "registration_sha256": deformables.registration_fingerprint(profile),
        "diagnostics": {"replay": {"max_node_error_m": 0.03308, "acceptance_threshold": None}},
    }
    result = deformables.acceptance(profile, report, "a" * 64)
    assert result["status"] == "pass"
    assert result["diagnostics"]["replay"]["max_node_error_m"] == 0.03308
    report["checks"]["reset_state"]["status"] = "fail"
    assert deformables.acceptance(profile, report, "a" * 64)["status"] == "fail"


def test_flexible_prompt_protocol_is_versioned_without_invalidating_rigid_inputs():
    import object_registration as reg

    rigid = {"id": "Rigid/test/00000", "auto": {}}
    flexible = {"id": "Dynamic/test/00000", "auto": {}, "deformable": {"family": "volume"}}
    assert reg.version(rigid) == reg.VERSION
    assert reg.version(flexible) != reg.VERSION
    text = reg.prompt(flexible)
    assert "reset_state" in text and "diagnostics.replay" in text
    assert "deformable_assets/README.md" in text


def test_failed_report_for_another_mesh_or_registration_is_not_current_failure():
    profile = {"family": "rope"}
    mesh = "ab" * 32
    report = {
        "mesh_sha256": mesh,
        "registration_sha256": deformables.registration_fingerprint(profile, 100, "3 m"),
        "checks": {"reset_state": {"status": "fail", "evidence": "candidate reset report"}},
    }
    current = deformables.acceptance(profile, report, mesh, 100, "3 m")
    assert current["status"] == "fail"
    other = deformables.acceptance(profile, report, "cd" * 32, 100, "3 m")
    assert other["status"] == "pending" and other["evidence_status"] == "fail"
    assert other["failed_checks"] == ["reset_state"]
    changed = deformables.acceptance(profile, report, mesh, 80, "0.8 m")
    assert changed["status"] == "pending" and not changed["registration_matches"]
