"""Integration tests – full assets run through the complete checker stack."""

import json

import pytest
from pxr import UsdValidation

from .conftest import has, make_stage, run_keyword

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
        errors = run_keyword(stage, "rep0158")
        error_errors = [
            e for e in errors
            if str(e.GetType()) == "UsdValidation.ValidationErrorType.Error"
        ]
        assert not error_errors, (
            f"Expected 0 errors but got {len(error_errors)}:\n"
            + "\n".join(f"  [{e.GetName()}] {e.GetMessage()}" for e in error_errors)
        )

    def test_compliant_asset_passes_ci_gate(self):
        stage = make_stage(_COMPLIANT_USDA)
        errors = run_keyword(stage, "rep0158")
        error_errors = [
            e for e in errors
            if str(e.GetType()) == "UsdValidation.ValidationErrorType.Error"
        ]
        assert len(error_errors) == 0


# ------------------------------------------------------------------ #
# Section filtering                                                     #
# ------------------------------------------------------------------ #


class TestSectionFiltering:
    def test_section_filter_limits_checks(self):
        """When keyword is rep0158:1.1, only §1.1 checks run."""
        stage = make_stage("""
#usda 1.0
def PhysicsRevoluteJoint "j" {}
""")
        errors = run_keyword(stage, "rep0158:1.1")
        found_ids = {e.GetName() for e in errors}
        assert "1.3.1" not in found_ids
        assert found_ids.issubset(
            {"1.1.1", "1.1.2", "1.1.3", "1.1.4", "1.1.5", "1.1.6", "1.1.7"}
        )

    def test_section_2_filter_skips_physics(self):
        stage = make_stage("""
#usda 1.0
def PhysicsRevoluteJoint "j" {}
""")
        errors = run_keyword(stage, "rep0158:2.1")
        found_ids = {e.GetName() for e in errors}
        assert not any(cid.startswith("1.") for cid in found_ids)


# ------------------------------------------------------------------ #
# Export checks are opt-in                                             #
# ------------------------------------------------------------------ #


class TestExportOptIn:
    def test_export_checks_excluded_from_core(self):
        """§3 checks must not run when querying core keywords."""
        stage = make_stage("""
#usda 1.0
def Material "Mat" {}
""")
        errors = run_keyword(stage, "rep0158:1.1")
        assert not any(e.GetName().startswith("3.") for e in errors)

    def test_export_checks_included_with_keyword(self):
        """§3 checks must run when querying export keyword."""
        stage = make_stage("""
#usda 1.0
def Material "Mat" {}
""")
        errors = run_keyword(stage, "rep0158:3.1")
        assert any(e.GetName().startswith("3.") for e in errors)


# ------------------------------------------------------------------ #
# Report output                                                         #
# ------------------------------------------------------------------ #


class TestReportOutput:
    def test_json_output_is_valid(self):
        from compliance_checker.report import errors_to_json

        stage = make_stage("""
#usda 1.0
def Xform "Robot" {}
""")
        errors = run_keyword(stage, "rep0158")
        json_str = errors_to_json("test.usda", errors)
        data = json.loads(json_str)
        assert "violations" in data
        assert "summary" in data
        assert isinstance(data["violations"], list)


# ------------------------------------------------------------------ #
# Checker robustness                                                    #
# ------------------------------------------------------------------ #


class TestCheckerRobustness:
    def test_empty_stage_no_crash(self):
        stage = make_stage("#usda 1.0\n")
        errors = run_keyword(stage, "rep0158")
        assert isinstance(errors, list)

    def test_from_path_on_file(self, tmp_path):
        """ValidationContext must accept a real stage."""
        f = tmp_path / "test.usda"
        f.write_text('#usda 1.0\ndef Xform "Robot" {}\n')
        stage = UsdValidation  # just verify import works
        from pxr import Usd
        s = Usd.Stage.Open(str(f))
        errors = run_keyword(s, "rep0158:1.1")
        assert isinstance(errors, list)
