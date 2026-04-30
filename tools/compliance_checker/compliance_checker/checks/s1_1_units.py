"""Section 1.1 – Coordinate Systems & Units."""

from __future__ import annotations

from typing import Iterator

from pxr import Gf, Usd, UsdGeom, UsdPhysics

from ..report import Severity, Violation
from .base import BaseCheck

_STAGE = "/"

_ROTATE_OP_KEYWORDS = ("rotate", "orient")
_JOINT_TYPES = {
    "PhysicsRevoluteJoint",
    "PhysicsPrismaticJoint",
    "PhysicsFixedJoint",
    "PhysicsSphericalJoint",
    "PhysicsDistanceJoint",
    "PhysicsJoint",
}


class CoordinateSystemCheck(BaseCheck):
    """REP §1.1: units, axis conventions, and kinematic transform constraints."""

    section = "1.1"

    def run(self, stage: Usd.Stage) -> Iterator[Violation]:
        yield from self._check_meters_per_unit(stage)
        yield from self._check_kilograms_per_unit(stage)
        yield from self._check_up_axis(stage)
        yield from self._check_time_codes_per_second(stage)
        yield from self._check_root_rotation(stage)
        yield from self._check_kinematic_transform_ops(stage)
        yield from self._check_kinematic_scale(stage)

    # ------------------------------------------------------------------ #
    # Individual rules                                                      #
    # ------------------------------------------------------------------ #

    def _check_meters_per_unit(self, stage: Usd.Stage) -> Iterator[Violation]:
        value = UsdGeom.GetStageMetersPerUnit(stage)
        if value != 1.0:
            yield Violation(
                check_id="1.1.1",
                severity=Severity.ERROR,
                prim_path=_STAGE,
                section=self.section,
                message=(
                    f"metersPerUnit is {value!r}; must be 1.0 (meters) per REP §1.1. "
                    "All linear dimensions must be expressed in meters."
                ),
                suggestion="Call UsdGeom.SetStageMetersPerUnit(stage, 1.0) on the root layer.",
            )

    def _check_kilograms_per_unit(self, stage: Usd.Stage) -> Iterator[Violation]:
        # UsdPhysics registers kilogramsPerUnit with a schema default of 1.0, so
        # GetMetadata always returns a float (never None). Only flag explicit wrong values.
        value = stage.GetMetadata("kilogramsPerUnit")
        if value is not None and value != 1.0:
            yield Violation(
                check_id="1.1.2",
                severity=Severity.ERROR,
                prim_path=_STAGE,
                section=self.section,
                message=(
                    f"kilogramsPerUnit is {value!r}; must be 1.0 per REP §1.1. "
                    "All mass values must be expressed in kilograms."
                ),
                suggestion="Author `kilogramsPerUnit = 1` in the root layer metadata block.",
            )

    def _check_up_axis(self, stage: Usd.Stage) -> Iterator[Violation]:
        up_axis = UsdGeom.GetStageUpAxis(stage)
        if up_axis != UsdGeom.Tokens.z:
            yield Violation(
                check_id="1.1.3",
                severity=Severity.ERROR,
                prim_path=_STAGE,
                section=self.section,
                message=(
                    f"upAxis is {up_axis!r}; must be 'Z' (Z-up, X-forward, Y-left) per REP §1.1 "
                    "and REP 103."
                ),
                suggestion="Call UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z).",
            )

    def _check_time_codes_per_second(self, stage: Usd.Stage) -> Iterator[Violation]:
        value = stage.GetTimeCodesPerSecond()
        if value != 1.0:
            yield Violation(
                check_id="1.1.5",
                severity=Severity.ERROR,
                prim_path=_STAGE,
                section=self.section,
                message=(
                    f"timeCodesPerSecond is {value!r}; must be 1.0 per REP §1.1. "
                    "One USD time code must equal one second for interoperable assets."
                ),
                suggestion="Author `timeCodesPerSecond = 1` in the root layer metadata block.",
            )

    def _check_root_rotation(self, stage: Usd.Stage) -> Iterator[Violation]:
        default_prim = stage.GetDefaultPrim()
        if not (default_prim and default_prim.IsValid()):
            return
        xformable = UsdGeom.Xformable(default_prim)
        if not xformable:
            return
        for op in xformable.GetOrderedXformOps():
            op_name = op.GetOpName()
            if any(k in op_name.lower() for k in _ROTATE_OP_KEYWORDS):
                yield Violation(
                    check_id="1.1.4",
                    severity=Severity.WARNING,
                    prim_path=str(default_prim.GetPath()),
                    section=self.section,
                    message=(
                        f"Root prim has rotation xformOp '{op_name}'. "
                        "Geometry must not rely on root-node rotations to align to Z-up; "
                        "points/normals should be baked at source level."
                    ),
                    suggestion=(
                        "Apply the rotation to the mesh data itself (freeze transforms) "
                        "and remove the xformOp from the root prim."
                    ),
                )

    def _check_kinematic_transform_ops(self, stage: Usd.Stage) -> Iterator[Violation]:
        for prim in stage.TraverseAll():
            if not self._is_kinematic_prim(prim):
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
                yield Violation(
                    check_id="1.1.6",
                    severity=Severity.ERROR,
                    prim_path=str(prim.GetPath()),
                    section=self.section,
                    message=(
                        f"Kinematic prim '{prim.GetPath()}' must use a minimal transform stack: "
                        "exactly one xformOp:translate and one xformOp:orient. "
                        f"Found xformOpOrder={op_names!r}."
                    ),
                    suggestion=(
                        "Replace matrix/Euler ops with a translate+orient pair and keep only those "
                        "two operations in xformOpOrder."
                    ),
                )

    def _check_kinematic_scale(self, stage: Usd.Stage) -> Iterator[Violation]:
        for prim in stage.TraverseAll():
            if not self._is_kinematic_prim(prim):
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
                if not self._is_identity_scale(value):
                    yield Violation(
                        check_id="1.1.7",
                        severity=Severity.ERROR,
                        prim_path=str(prim.GetPath()),
                        section=self.section,
                        message=(
                            f"Kinematic prim '{prim.GetPath()}' has non-identity scale "
                            f"{value!r}. Non-identity scale on rigid bodies and joints is "
                            "prohibited per REP §1.1."
                        ),
                        suggestion=(
                            "Remove scale from the kinematic prim, or push geometric scaling "
                            "to leaf visual/collision geometry prims only."
                        ),
                    )

    def _is_kinematic_prim(self, prim: Usd.Prim) -> bool:
        return (
            prim.HasAPI(UsdPhysics.RigidBodyAPI) or prim.GetTypeName() in _JOINT_TYPES
        )

    def _is_identity_scale(self, value: object) -> bool:
        if isinstance(value, (Gf.Vec3d, Gf.Vec3f, Gf.Vec3h)):
            return value == Gf.Vec3d(1.0, 1.0, 1.0)
        if isinstance(value, tuple) and len(value) == 3:
            return value == (1.0, 1.0, 1.0)
        return False
