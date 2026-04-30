"""Section 1.1 – Coordinate Systems & Units."""

from __future__ import annotations

from pxr import Gf, Usd, UsdGeom, UsdPhysics

from .base import ErrorType, TimeRange, _error, _prim_site, _stage_site, register_stage_validator
from ._tokens import (
    KINEMATIC_NON_IDENTITY_SCALE,
    KINEMATIC_TRANSFORM_OPS,
    ROOT_HAS_ROTATION,
    WRONG_KILOGRAMS_PER_UNIT,
    WRONG_METERS_PER_UNIT,
    WRONG_TIME_CODES_PER_SECOND,
    WRONG_UP_AXIS,
)

_ROTATE_OP_KEYWORDS = ("rotate", "orient")
_JOINT_TYPES = {
    "PhysicsRevoluteJoint", "PhysicsPrismaticJoint", "PhysicsFixedJoint",
    "PhysicsSphericalJoint", "PhysicsDistanceJoint", "PhysicsJoint",
}


def _is_kinematic_prim(prim: Usd.Prim) -> bool:
    return prim.HasAPI(UsdPhysics.RigidBodyAPI) or prim.GetTypeName() in _JOINT_TYPES


def _is_identity_scale(value: object) -> bool:
    if isinstance(value, (Gf.Vec3d, Gf.Vec3f, Gf.Vec3h)):
        return value == Gf.Vec3d(1.0, 1.0, 1.0)
    if isinstance(value, tuple) and len(value) == 3:
        return value == (1.0, 1.0, 1.0)
    return False


def _validate_coordinate_system(stage: Usd.Stage, timeRange: TimeRange):
    value = UsdGeom.GetStageMetersPerUnit(stage)
    if value != 1.0:
        yield _error(
            WRONG_METERS_PER_UNIT, ErrorType.Error, _stage_site(stage),
            f"metersPerUnit is {value!r}; must be 1.0 (meters) per REP §1.1. "
            "All linear dimensions must be expressed in meters.",
            "Call UsdGeom.SetStageMetersPerUnit(stage, 1.0) on the root layer.",
        )

    value = stage.GetMetadata("kilogramsPerUnit")
    if value is not None and value != 1.0:
        yield _error(
            WRONG_KILOGRAMS_PER_UNIT, ErrorType.Error, _stage_site(stage),
            f"kilogramsPerUnit is {value!r}; must be 1.0 per REP §1.1. "
            "All mass values must be expressed in kilograms.",
            "Author `kilogramsPerUnit = 1` in the root layer metadata block.",
        )

    up_axis = UsdGeom.GetStageUpAxis(stage)
    if up_axis != UsdGeom.Tokens.z:
        yield _error(
            WRONG_UP_AXIS, ErrorType.Error, _stage_site(stage),
            f"upAxis is {up_axis!r}; must be 'Z' (Z-up, X-forward, Y-left) per REP §1.1 "
            "and REP 103.",
            "Call UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z).",
        )

    value = stage.GetTimeCodesPerSecond()
    if value != 1.0:
        yield _error(
            WRONG_TIME_CODES_PER_SECOND, ErrorType.Error, _stage_site(stage),
            f"timeCodesPerSecond is {value!r}; must be 1.0 per REP §1.1. "
            "One USD time code must equal one second for interoperable assets.",
            "Author `timeCodesPerSecond = 1` in the root layer metadata block.",
        )

    default_prim = stage.GetDefaultPrim()
    if default_prim and default_prim.IsValid():
        xformable = UsdGeom.Xformable(default_prim)
        if xformable:
            pp = str(default_prim.GetPath())
            for op in xformable.GetOrderedXformOps():
                op_name = op.GetOpName()
                if any(k in op_name.lower() for k in _ROTATE_OP_KEYWORDS):
                    yield _error(
                        ROOT_HAS_ROTATION, ErrorType.Warn, _prim_site(stage, pp),
                        f"Root prim has rotation xformOp '{op_name}'. "
                        "Geometry must not rely on root-node rotations to align to Z-up; "
                        "points/normals should be baked at source level.",
                        "Apply the rotation to the mesh data itself (freeze transforms) "
                        "and remove the xformOp from the root prim.",
                    )

    for prim in stage.TraverseAll():
        if not _is_kinematic_prim(prim):
            continue
        xformable = UsdGeom.Xformable(prim)
        if not xformable:
            continue
        ops = xformable.GetOrderedXformOps()
        if not ops:
            continue
        pp = str(prim.GetPath())
        op_names = [op.GetOpName() for op in ops]
        lowered = [name.lower() for name in op_names]
        has_matrix = any("xformop:transform" in n for n in lowered)
        has_euler = any("xformop:rotate" in n and "orient" not in n for n in lowered)
        has_translate = sum("xformop:translate" in n for n in lowered)
        has_orient = sum("xformop:orient" in n for n in lowered)
        if has_matrix or has_euler or has_translate != 1 or has_orient != 1 or len(ops) != 2:
            yield _error(
                KINEMATIC_TRANSFORM_OPS, ErrorType.Error, _prim_site(stage, pp),
                f"Kinematic prim '{pp}' must use a minimal transform stack: "
                f"exactly one xformOp:translate and one xformOp:orient. Found xformOpOrder={op_names!r}.",
                "Replace matrix/Euler ops with a translate+orient pair and keep only those "
                "two operations in xformOpOrder.",
            )
        for op in ops:
            if "xformop:scale" not in op.GetOpName().lower():
                continue
            value = op.Get()
            if value is not None and not _is_identity_scale(value):
                yield _error(
                    KINEMATIC_NON_IDENTITY_SCALE, ErrorType.Error, _prim_site(stage, pp),
                    f"Kinematic prim '{pp}' has non-identity scale {value!r}. "
                    "Non-identity scale on rigid bodies and joints is prohibited per REP §1.1.",
                    "Remove scale from the kinematic prim, or push geometric scaling "
                    "to leaf visual/collision geometry prims only.",
                )


register_stage_validator(
    "CoordinateSystem", _validate_coordinate_system,
    doc="REP §1.1: units, axis conventions, and kinematic transform constraints.",
    section="1.1",
)
