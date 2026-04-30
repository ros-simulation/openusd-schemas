"""Section 1.1 – Coordinate Systems & Units."""

from __future__ import annotations

from pxr import Gf, Usd, UsdGeom, UsdPhysics

from .base import (
    ErrorType,
    TimeRange,
    ValidationError,
    _error,
    _prim_site,
    _stage_site,
    register_stage_validator,
)

_ROTATE_OP_KEYWORDS = ("rotate", "orient")
_JOINT_TYPES = {
    "PhysicsRevoluteJoint",
    "PhysicsPrismaticJoint",
    "PhysicsFixedJoint",
    "PhysicsSphericalJoint",
    "PhysicsDistanceJoint",
    "PhysicsJoint",
}


def _check_meters_per_unit(stage: Usd.Stage) -> list[ValidationError]:
    value = UsdGeom.GetStageMetersPerUnit(stage)
    if value != 1.0:
        return [_error(
            "1.1.1",
            ErrorType.Error,
            _stage_site(stage),
            f"metersPerUnit is {value!r}; must be 1.0 (meters) per REP §1.1. "
            "All linear dimensions must be expressed in meters.",
            "Call UsdGeom.SetStageMetersPerUnit(stage, 1.0) on the root layer.",
        )]
    return []


def _check_kilograms_per_unit(stage: Usd.Stage) -> list[ValidationError]:
    value = stage.GetMetadata("kilogramsPerUnit")
    if value is not None and value != 1.0:
        return [_error(
            "1.1.2",
            ErrorType.Error,
            _stage_site(stage),
            f"kilogramsPerUnit is {value!r}; must be 1.0 per REP §1.1. "
            "All mass values must be expressed in kilograms.",
            "Author `kilogramsPerUnit = 1` in the root layer metadata block.",
        )]
    return []


def _check_up_axis(stage: Usd.Stage) -> list[ValidationError]:
    up_axis = UsdGeom.GetStageUpAxis(stage)
    if up_axis != UsdGeom.Tokens.z:
        return [_error(
            "1.1.3",
            ErrorType.Error,
            _stage_site(stage),
            f"upAxis is {up_axis!r}; must be 'Z' (Z-up, X-forward, Y-left) per REP §1.1 "
            "and REP 103.",
            "Call UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z).",
        )]
    return []


def _check_time_codes_per_second(stage: Usd.Stage) -> list[ValidationError]:
    value = stage.GetTimeCodesPerSecond()
    if value != 1.0:
        return [_error(
            "1.1.5",
            ErrorType.Error,
            _stage_site(stage),
            f"timeCodesPerSecond is {value!r}; must be 1.0 per REP §1.1. "
            "One USD time code must equal one second for interoperable assets.",
            "Author `timeCodesPerSecond = 1` in the root layer metadata block.",
        )]
    return []


def _check_root_rotation(stage: Usd.Stage) -> list[ValidationError]:
    default_prim = stage.GetDefaultPrim()
    if not (default_prim and default_prim.IsValid()):
        return []
    xformable = UsdGeom.Xformable(default_prim)
    if not xformable:
        return []
    errors: list[ValidationError] = []
    for op in xformable.GetOrderedXformOps():
        op_name = op.GetOpName()
        if any(k in op_name.lower() for k in _ROTATE_OP_KEYWORDS):
            errors.append(_error(
                "1.1.4",
                ErrorType.Warn,
                _prim_site(stage, str(default_prim.GetPath())),
                f"Root prim has rotation xformOp '{op_name}'. "
                "Geometry must not rely on root-node rotations to align to Z-up; "
                "points/normals should be baked at source level.",
                "Apply the rotation to the mesh data itself (freeze transforms) "
                "and remove the xformOp from the root prim.",
            ))
    return errors


def _is_kinematic_prim(prim: Usd.Prim) -> bool:
    return (
        prim.HasAPI(UsdPhysics.RigidBodyAPI) or prim.GetTypeName() in _JOINT_TYPES
    )


def _is_identity_scale(value: object) -> bool:
    if isinstance(value, (Gf.Vec3d, Gf.Vec3f, Gf.Vec3h)):
        return value == Gf.Vec3d(1.0, 1.0, 1.0)
    if isinstance(value, tuple) and len(value) == 3:
        return value == (1.0, 1.0, 1.0)
    return False


def _check_kinematic_transform_ops(stage: Usd.Stage) -> list[ValidationError]:
    errors: list[ValidationError] = []
    for prim in stage.TraverseAll():
        if not _is_kinematic_prim(prim):
            continue
        xformable = UsdGeom.Xformable(prim)
        if not xformable:
            continue

        ops = xformable.GetOrderedXformOps()
        if not ops:
            continue

        op_names = [op.GetOpName() for op in ops]
        lowered = [name.lower() for name in op_names]

        has_matrix = any("xformop:transform" in name for name in lowered)
        has_euler = any(
            "xformop:rotate" in name and "orient" not in name for name in lowered
        )
        has_translate = sum("xformop:translate" in name for name in lowered)
        has_orient = sum("xformop:orient" in name for name in lowered)

        if (
            has_matrix
            or has_euler
            or has_translate != 1
            or has_orient != 1
            or len(ops) != 2
        ):
            errors.append(_error(
                "1.1.6",
                ErrorType.Error,
                _prim_site(stage, str(prim.GetPath())),
                f"Kinematic prim '{prim.GetPath()}' must use a minimal transform stack: "
                "exactly one xformOp:translate and one xformOp:orient. "
                f"Found xformOpOrder={op_names!r}.",
                "Replace matrix/Euler ops with a translate+orient pair and keep only those "
                "two operations in xformOpOrder.",
            ))
    return errors


def _check_kinematic_scale(stage: Usd.Stage) -> list[ValidationError]:
    errors: list[ValidationError] = []
    for prim in stage.TraverseAll():
        if not _is_kinematic_prim(prim):
            continue
        xformable = UsdGeom.Xformable(prim)
        if not xformable:
            continue
        for op in xformable.GetOrderedXformOps():
            if "xformop:scale" not in op.GetOpName().lower():
                continue
            value = op.Get()
            if value is None:
                continue
            if not _is_identity_scale(value):
                errors.append(_error(
                    "1.1.7",
                    ErrorType.Error,
                    _prim_site(stage, str(prim.GetPath())),
                    f"Kinematic prim '{prim.GetPath()}' has non-identity scale "
                    f"{value!r}. Non-identity scale on rigid bodies and joints is "
                    "prohibited per REP §1.1.",
                    "Remove scale from the kinematic prim, or push geometric scaling "
                    "to leaf visual/collision geometry prims only.",
                ))
    return errors


def _validate_coordinate_system(
    stage: Usd.Stage, timeRange: TimeRange,
) -> list[ValidationError]:
    errors: list[ValidationError] = []
    errors.extend(_check_meters_per_unit(stage))
    errors.extend(_check_kilograms_per_unit(stage))
    errors.extend(_check_up_axis(stage))
    errors.extend(_check_time_codes_per_second(stage))
    errors.extend(_check_root_rotation(stage))
    errors.extend(_check_kinematic_transform_ops(stage))
    errors.extend(_check_kinematic_scale(stage))
    return errors


register_stage_validator(
    "CoordinateSystem",
    _validate_coordinate_system,
    doc="REP §1.1: units, axis conventions, and kinematic transform constraints.",
    section="1.1",
)
