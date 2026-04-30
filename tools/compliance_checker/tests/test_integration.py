"""Integration tests – full assets run through the complete checker stack.

These tests verify that the orchestration layer (checker.py + registry.py)
works end-to-end and that cross-section interactions behave correctly.
"""

import json

import pytest
from compliance_checker import ComplianceChecker, Severity
from compliance_checker.registry import build_checks

from .conftest import make_stage

# ------------------------------------------------------------------ #
# A fully compliant minimal asset                                      #
# ------------------------------------------------------------------ #

_COMPLIANT_USDA = """
#usda 1.0
(
    defaultPrim = "Robot"
    upAxis = "Z"
    metersPerUnit = 1
    kilogramsPerUnit = 1
    timeCodesPerSecond = 1
)

def Xform "Robot" (
    prepend apiSchemas = ["RosContextAPI"]
    kind = "component"
    assetInfo = {
        string identifier = "acme.simple_robot"
        string version = "1.0.0"
    }
) {
    string ros:context:namespace = "simple_robot"
    string ros:context:parent_frame = "world"

    def PhysicsRevoluteJoint "elbow" {
        float physics:lowerLimit = -1.57
        float physics:upperLimit = 1.57
        custom string ros:joint:name = "elbow_joint"
    }

    def Xform "Interfaces" {
        def Xform "JointStatePublisher" (
            prepend apiSchemas = ["RosTopicAPI"]
        ) {
            token ros:topic:role = "publisher"
            string ros:topic:name = "joint_states"
            string ros:topic:type = "sensor_msgs/msg/JointState"
            double ros:topic:publish_rate = 50.0
        }

        def Xform "CmdVelSubscriber" (
            prepend apiSchemas = ["RosTopicAPI"]
        ) {
            token ros:topic:role = "subscription"
            string ros:topic:name = "cmd_vel"
            string ros:topic:type = "geometry_msgs/msg/Twist"
        }
    }
}
"""


class TestCompliantAsset:
    def test_compliant_asset_zero_errors(self):
        stage = make_stage(_COMPLIANT_USDA)
        checks = build_checks(include_export=False)
        report = ComplianceChecker(stage, checks).run()
        assert not report.has_errors(), (
            f"Expected 0 errors but got {len(report.errors)}:\n"
            + "\n".join(f"  [{v.check_id}] {v.message}" for v in report.errors)
        )

    def test_compliant_asset_passes_ci_gate(self):
        stage = make_stage(_COMPLIANT_USDA)
        checks = build_checks(include_export=False)
        report = ComplianceChecker(stage, checks).run()
        assert report.passed(fail_on=Severity.ERROR)


# ------------------------------------------------------------------ #
# Section filtering                                                     #
# ------------------------------------------------------------------ #


class TestSectionFiltering:
    def test_section_filter_limits_checks(self):
        """When sections=['1.1'], only §1.1 checks run."""
        stage = make_stage("""
#usda 1.0
def PhysicsRevoluteJoint "j" {}
""")
        # Only §1.1 checks
        checks = build_checks(sections=["1.1"])
        report = ComplianceChecker(stage, checks).run()
        found_ids = {v.check_id for v in report.violations}
        # 1.3.1 joint limits should NOT appear because §1.3 was filtered out
        assert "1.3.1" not in found_ids
        # But §1.1 violations (missing metersPerUnit etc.) may appear
        assert found_ids.issubset(
            {"1.1.1", "1.1.2", "1.1.3", "1.1.4", "1.1.5", "1.1.6", "1.1.7"}
        )

    def test_section_2_filter_skips_physics(self):
        stage = make_stage("""
#usda 1.0
def PhysicsRevoluteJoint "j" {}
""")
        checks = build_checks(sections=["2"])
        report = ComplianceChecker(stage, checks).run()
        found_ids = {v.check_id for v in report.violations}
        assert not any(cid.startswith("1.") for cid in found_ids)


# ------------------------------------------------------------------ #
# Export checks are opt-in                                             #
# ------------------------------------------------------------------ #


class TestExportOptIn:
    def test_export_checks_excluded_by_default(self):
        """§3 checks must not run unless include_export=True."""
        stage = make_stage("""
#usda 1.0
def Material "Mat" {}
""")
        checks = build_checks(include_export=False)
        report = ComplianceChecker(stage, checks).run()
        assert not any(v.check_id.startswith("3.") for v in report.violations)

    def test_export_checks_included_with_flag(self):
        """§3 checks must run when include_export=True."""
        stage = make_stage("""
#usda 1.0
def Material "Mat" {}
""")
        checks = build_checks(include_export=True)
        report = ComplianceChecker(stage, checks).run()
        assert any(v.check_id.startswith("3.") for v in report.violations)


# ------------------------------------------------------------------ #
# Report output                                                         #
# ------------------------------------------------------------------ #


class TestReportOutput:
    def test_json_output_is_valid(self):
        stage = make_stage("""
#usda 1.0
def Xform "Robot" {}
""")
        checks = build_checks()
        report = ComplianceChecker(stage, checks).run()
        data = json.loads(report.to_json())
        assert "violations" in data
        assert "summary" in data
        assert isinstance(data["violations"], list)

    def test_report_by_section_groups_correctly(self):
        stage = make_stage("""
#usda 1.0
def PhysicsRevoluteJoint "j" {}
""")
        checks = build_checks()
        report = ComplianceChecker(stage, checks).run()
        by_section = report.by_section()
        # §1.3 violations for the missing joint limits should appear
        assert "1.3" in by_section or any(k.startswith("1.") for k in by_section)

    def test_severity_filtering(self):
        stage = make_stage(_COMPLIANT_USDA)
        checks = build_checks()
        report = ComplianceChecker(stage, checks).run()
        assert all(v.severity == Severity.ERROR for v in report.errors)
        assert all(v.severity == Severity.WARNING for v in report.warnings)

    def test_passed_true_when_no_errors(self):
        stage = make_stage(_COMPLIANT_USDA)
        checks = build_checks(include_export=False)
        report = ComplianceChecker(stage, checks).run()
        assert report.passed(fail_on=Severity.ERROR)

    def test_passed_false_when_errors_present(self):
        stage = make_stage('#usda 1.0\ndef PhysicsRevoluteJoint "j" {}')
        checks = build_checks()
        report = ComplianceChecker(stage, checks).run()
        assert not report.passed(fail_on=Severity.ERROR)


# ------------------------------------------------------------------ #
# Checker robustness                                                    #
# ------------------------------------------------------------------ #


class TestCheckerRobustness:
    def test_empty_stage_no_crash(self):
        stage = make_stage("#usda 1.0\n")
        checks = build_checks(include_export=True)
        report = ComplianceChecker(stage, checks).run()
        # No internal_error violations from crashes
        internal = [v for v in report.violations if "internal_error" in v.check_id]
        assert not internal, "Checker crashed on empty stage:\n" + "\n".join(
            f"  {v.message}" for v in internal
        )

    def test_from_path_on_file(self, tmp_path):
        """ComplianceChecker.from_path() must accept a real file path."""
        f = tmp_path / "test.usda"
        f.write_text('#usda 1.0\ndef Xform "Robot" {}\n')
        checker = ComplianceChecker.from_path(str(f))
        report = checker.run()
        assert isinstance(report.violations, list)

    def test_from_path_nonexistent_raises(self, tmp_path):
        with pytest.raises(FileNotFoundError):
            ComplianceChecker.from_path(str(tmp_path / "does_not_exist.usda"))
