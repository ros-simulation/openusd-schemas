"""Shared fixtures and helpers for the REP-0158 compliance checker test suite.

The run_validators() helper provides a unified interface that maps validator
names to check classes. This allows tests to be written against the
UsdValidation naming convention while still running on the current framework.
"""

from __future__ import annotations

import pytest
from pxr import Sdf, Usd

from compliance_checker.checker import ComplianceChecker
from compliance_checker.checks.s1_1_units import CoordinateSystemCheck
from compliance_checker.checks.s1_2_structure import (
    AssetManagementCheck,
    CompositionModelCheck,
    InheritsSpecializesCheck,
    LayerEncodingCheck,
    ParallelSimulationInstancingCheck,
    PathConventionCheck,
    PayloadKinematicTopologyCheck,
    VariantDefaultCheck,
)
from compliance_checker.checks.s1_3_physics import (
    ArticulationRootCheck,
    CollisionGeometryAuthoringCheck,
    CollisionMaterialCheck,
    InstanceablePhysicsCheck,
    JointLimitsCheck,
    MassPropertiesCheck,
    MimicJointCheck,
)
from compliance_checker.checks.s2_ros import (
    CameraOpticalFrameCheck,
    RosActionCheck,
    RosContextPlacementCheck,
    RosFrameAPICheck,
    RosFrameAttributesCheck,
    RosInterfacePlacementCheck,
    RosInterfaceStructureCheck,
    RosJointNameCheck,
    RosServiceCheck,
    RosTopicCheck,
)
from compliance_checker.checks.s3_export import (
    GeometryConstraintsCheck,
    LightingPortabilityCheck,
    MaterialPortabilityCheck,
    TextureBakingCheck,
    TextureFormatCheck,
)
from compliance_checker.checks.s4_extended_physics import (
    ExtendedPhysicsActuatorCheck,
    ExtendedPhysicsMimicCheck,
    ExtendedPhysicsPositionClampingCheck,
)

_VALIDATOR_MAP: dict[str, type] = {
    "rep0158:CoordinateSystem": CoordinateSystemCheck,
    "rep0158:AssetManagement": AssetManagementCheck,
    "rep0158:PathConvention": PathConventionCheck,
    "rep0158:CompositionModel": CompositionModelCheck,
    "rep0158:VariantDefault": VariantDefaultCheck,
    "rep0158:InheritsSpecializes": InheritsSpecializesCheck,
    "rep0158:PayloadKinematicTopology": PayloadKinematicTopologyCheck,
    "rep0158:LayerEncoding": LayerEncodingCheck,
    "rep0158:ParallelSimulationInstancing": ParallelSimulationInstancingCheck,
    "rep0158:JointLimits": JointLimitsCheck,
    "rep0158:ArticulationRoot": ArticulationRootCheck,
    "rep0158:MassProperties": MassPropertiesCheck,
    "rep0158:CollisionMaterial": CollisionMaterialCheck,
    "rep0158:CollisionGeometryAuthoring": CollisionGeometryAuthoringCheck,
    "rep0158:MimicJoint": MimicJointCheck,
    "rep0158:InstanceablePhysics": InstanceablePhysicsCheck,
    "rep0158:RosContextPlacement": RosContextPlacementCheck,
    "rep0158:RosInterfacePlacement": RosInterfacePlacementCheck,
    "rep0158:RosInterfaceStructure": RosInterfaceStructureCheck,
    "rep0158:RosTopic": RosTopicCheck,
    "rep0158:RosService": RosServiceCheck,
    "rep0158:RosAction": RosActionCheck,
    "rep0158:RosFrameAPI": RosFrameAPICheck,
    "rep0158:RosFrameAttributes": RosFrameAttributesCheck,
    "rep0158:CameraOpticalFrame": CameraOpticalFrameCheck,
    "rep0158:RosJointName": RosJointNameCheck,
    "rep0158:MaterialPortability": MaterialPortabilityCheck,
    "rep0158:TextureFormat": TextureFormatCheck,
    "rep0158:TextureBaking": TextureBakingCheck,
    "rep0158:GeometryConstraints": GeometryConstraintsCheck,
    "rep0158:LightingPortability": LightingPortabilityCheck,
    "rep0158:ExtendedPhysicsMimic": ExtendedPhysicsMimicCheck,
    "rep0158:ExtendedPhysicsActuator": ExtendedPhysicsActuatorCheck,
    "rep0158:ExtendedPhysicsPositionClamping": ExtendedPhysicsPositionClampingCheck,
}


class _ErrorAdapter:
    """Adapts a Violation to the UsdValidation.ValidationError interface."""

    def __init__(self, violation):
        self._v = violation

    def GetName(self) -> str:
        return self._v.check_id

    def GetMessage(self) -> str:
        msg = self._v.message
        if self._v.suggestion:
            msg = f"{msg} Suggestion: {self._v.suggestion}"
        return msg

    def GetType(self):
        return self._v.severity

    def GetSites(self):
        return [_SiteAdapter(self._v.prim_path)]

    def GetIdentifier(self) -> str:
        return f"rep0158:{self._v.section}.{self._v.check_id}"


class _SiteAdapter:
    """Minimal shim so tests can call GetPrimPath() on error sites."""

    def __init__(self, prim_path: str):
        self._path = prim_path

    def GetPrimPath(self):
        return self._path

    def IsPrim(self):
        return True

    def IsProperty(self):
        return False


# ------------------------------------------------------------------ #
# Stage factory                                                         #
# ------------------------------------------------------------------ #


def make_stage(usda: str) -> Usd.Stage:
    """Return an in-memory USD stage loaded from a USDA string."""
    layer = Sdf.Layer.CreateAnonymous(".usda")
    layer.ImportFromString(usda)
    return Usd.Stage.Open(layer)


@pytest.fixture
def tmp_usda(tmp_path):
    """Fixture: write USDA content to a temp file, return path string."""

    def _write(content: str, name: str = "test.usda") -> str:
        p = tmp_path / name
        p.write_text(content)
        return str(p)

    return _write


# ------------------------------------------------------------------ #
# Check runner helpers                                                  #
# ------------------------------------------------------------------ #


def run_validators(
    stage: Usd.Stage,
    *validator_names: str,
) -> list[_ErrorAdapter]:
    """Run named validators against *stage*, return adapted error objects.

    Maps validator names (e.g. "rep0158:CoordinateSystem") to the
    corresponding BaseCheck class and runs them via the existing framework.
    Returns _ErrorAdapter objects that expose the UsdValidation API
    (GetName, GetMessage, GetType, GetSites).
    """
    check_classes = [_VALIDATOR_MAP[n] for n in validator_names]
    checks = [cls() for cls in check_classes]
    violations = ComplianceChecker(stage, checks).run().violations
    return [_ErrorAdapter(v) for v in violations]


def run_keyword(
    stage: Usd.Stage,
    keyword: str,
) -> list[_ErrorAdapter]:
    """Run all validators matching a keyword prefix."""
    from compliance_checker.registry import build_checks

    if keyword == "rep0158":
        checks = build_checks(include_export=True, include_extended=True)
    elif keyword.startswith("rep0158:"):
        section = keyword[len("rep0158:"):]
        checks = build_checks(
            include_export=True,
            include_extended=True,
            sections=[section],
        )
    else:
        checks = build_checks()

    violations = ComplianceChecker(stage, checks).run().violations
    return [_ErrorAdapter(v) for v in violations]


def ids(errors: list[_ErrorAdapter]) -> set[str]:
    """Return the set of error names from an error list."""
    return {e.GetName() for e in errors}


def has(errors: list[_ErrorAdapter], error_name: str) -> bool:
    """Return True when at least one error carries *error_name*."""
    return error_name in ids(errors)


def none_with(errors: list[_ErrorAdapter], error_name: str) -> bool:
    """Return True when no error carries *error_name*."""
    return error_name not in ids(errors)
