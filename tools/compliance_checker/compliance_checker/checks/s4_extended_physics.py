"""Section 4.2 – Extended Physics Schema checks."""

from __future__ import annotations

from collections import defaultdict

from pxr import Usd

from .base import ErrorType, TimeRange, _error, _prim_site, register_stage_validator
from ._tokens import (
    ACTUATOR_INVALID_TARGETS,
    CLAMPING_LOOKUP_TABLE,
    MIMIC_CYCLE,
    MIMIC_INVALID_RELATIONSHIP,
    MIMIC_WRONG_JOINT_TYPE,
)

_ALLOWED_JOINT_TYPES = {"PhysicsRevoluteJoint", "PhysicsPrismaticJoint"}


def _applied(prim: Usd.Prim) -> set[str]:
    list_op = prim.GetMetadata("apiSchemas")
    if list_op is None:
        return set()
    return set(list_op.GetAppliedItems())


def _check_cycle_free_graph(mimic_graph: dict[str, list[str]], stage: Usd.Stage):
    visiting: set[str] = set()
    visited: set[str] = set()

    def dfs(node: str, stack: list[str]):
        if node in visiting:
            cycle = stack[stack.index(node):] + [node]
            return [_error(
                MIMIC_CYCLE, ErrorType.Error, _prim_site(stage, node),
                f"ExtendedPhysicsMimicAPI relationships must form a DAG. "
                f"Detected cycle: {' -> '.join(cycle)}.",
                "Break the mimic cycle by removing one coupling edge.",
            )]
        if node in visited:
            return []
        visiting.add(node)
        stack.append(node)
        errors = []
        for target in mimic_graph.get(node, []):
            errors.extend(dfs(target, stack))
        stack.pop()
        visiting.remove(node)
        visited.add(node)
        return errors

    for node in mimic_graph:
        yield from dfs(node, [])


def _validate_extended_physics_mimic(stage: Usd.Stage, timeRange: TimeRange):
    mimic_graph: dict[str, list[str]] = defaultdict(list)
    for prim in stage.TraverseAll():
        if "ExtendedPhysicsMimicAPI" not in _applied(prim):
            continue
        pp = str(prim.GetPath())
        if prim.GetTypeName() not in _ALLOWED_JOINT_TYPES:
            yield _error(
                MIMIC_WRONG_JOINT_TYPE, ErrorType.Warn, _prim_site(stage, pp),
                f"ExtendedPhysicsMimicAPI is applied to '{pp}' (type: '{prim.GetTypeName()}'). "
                "Must only be applied to PhysicsRevoluteJoint or PhysicsPrismaticJoint per REP §4.2.2.",
                "Remove ExtendedPhysicsMimicAPI or change the joint type.",
            )
            continue
        rel = prim.GetRelationship("ext_physics:mimic:joint")
        if not rel.IsValid() or not rel.GetTargets():
            yield _error(
                MIMIC_INVALID_RELATIONSHIP, ErrorType.Error, _prim_site(stage, pp),
                f"ExtendedPhysicsMimicAPI on '{pp}' is missing required relationship "
                "'ext_physics:mimic:joint'. The leader joint must be specified per REP §4.2.2.",
                "Author `rel ext_physics:mimic:joint = </path/to/source_joint>`.",
            )
            continue
        targets = rel.GetTargets()
        if len(targets) != 1:
            yield _error(
                MIMIC_INVALID_RELATIONSHIP, ErrorType.Error, _prim_site(stage, pp),
                f"'ext_physics:mimic:joint' on '{pp}' must target exactly one "
                f"source joint; found {len(targets)} targets.",
                "Keep exactly one relationship target.",
            )
            continue
        target = targets[0]
        source = stage.GetPrimAtPath(target)
        if not source or not source.IsValid():
            yield _error(
                MIMIC_INVALID_RELATIONSHIP, ErrorType.Error, _prim_site(stage, pp),
                f"'ext_physics:mimic:joint' on '{pp}' targets '{target}', "
                "which does not exist in the composed stage.",
                "Point to an existing revolute/prismatic joint prim.",
            )
            continue
        if source.GetTypeName() not in _ALLOWED_JOINT_TYPES:
            yield _error(
                MIMIC_INVALID_RELATIONSHIP, ErrorType.Error, _prim_site(stage, pp),
                f"'ext_physics:mimic:joint' on '{pp}' targets '{target}' "
                f"(type '{source.GetTypeName()}'). Source must be revolute or prismatic per REP §4.2.2.",
                "Target a PhysicsRevoluteJoint or PhysicsPrismaticJoint.",
            )
            continue
        mimic_graph[pp].append(str(target))
    yield from _check_cycle_free_graph(mimic_graph, stage)


def _validate_extended_physics_actuator(stage: Usd.Stage, timeRange: TimeRange):
    for prim in stage.TraverseAll():
        if "ExtendedPhysicsActuatorAPI" not in _applied(prim):
            continue
        pp = str(prim.GetPath())
        rel = prim.GetRelationship("ext_physics:actuator:targets")
        if not rel.IsValid() or not rel.GetTargets():
            yield _error(
                ACTUATOR_INVALID_TARGETS, ErrorType.Error, _prim_site(stage, pp),
                f"ExtendedPhysicsActuatorAPI on '{pp}' is missing required relationship "
                "'ext_physics:actuator:targets'. The target joint must be specified per REP §4.2.2.",
                "Author `rel ext_physics:actuator:targets = </path/to/joint>`.",
            )
            continue
        for target in rel.GetTargets():
            target_prim = stage.GetPrimAtPath(target)
            if not target_prim or not target_prim.IsValid():
                yield _error(
                    ACTUATOR_INVALID_TARGETS, ErrorType.Error, _prim_site(stage, pp),
                    f"'ext_physics:actuator:targets' on '{pp}' targets '{target}', "
                    "which does not exist in the composed stage.",
                    "Point to an existing PhysicsRevoluteJoint or PhysicsPrismaticJoint.",
                )
                continue
            if target_prim.GetTypeName() not in _ALLOWED_JOINT_TYPES:
                yield _error(
                    ACTUATOR_INVALID_TARGETS, ErrorType.Error, _prim_site(stage, pp),
                    f"'ext_physics:actuator:targets' on '{pp}' targets '{target}' "
                    f"(type '{target_prim.GetTypeName()}'). Only PhysicsRevoluteJoint or "
                    "PhysicsPrismaticJoint are supported per REP §4.2.2.",
                    "Target a PhysicsRevoluteJoint or PhysicsPrismaticJoint.",
                )


def _validate_extended_physics_position_clamping(stage: Usd.Stage, timeRange: TimeRange):
    for prim in stage.TraverseAll():
        if "ExtendedPhysicsPositionBasedClampingAPI" not in _applied(prim):
            continue
        pp = str(prim.GetPath())
        pos_attr = prim.GetAttribute("ext_physics:clamp_position:lookupPositions")
        eff_attr = prim.GetAttribute("ext_physics:clamp_position:lookupEfforts")
        positions = pos_attr.Get() if pos_attr.IsValid() else None
        efforts = eff_attr.Get() if eff_attr.IsValid() else None
        if positions is None and efforts is None:
            continue
        positions = list(positions) if positions is not None else []
        efforts = list(efforts) if efforts is not None else []
        if len(positions) != len(efforts):
            yield _error(
                CLAMPING_LOOKUP_TABLE, ErrorType.Error, _prim_site(stage, pp),
                f"ExtendedPhysicsPositionBasedClampingAPI on '{pp}' has "
                f"lookupPositions length {len(positions)} but lookupEfforts length "
                f"{len(efforts)}. Both arrays must be the same length per REP §4.2.2.",
                "Ensure lookupPositions and lookupEfforts have the same number of elements.",
            )
        if len(positions) >= 2:
            for i in range(len(positions) - 1):
                if positions[i] >= positions[i + 1]:
                    yield _error(
                        CLAMPING_LOOKUP_TABLE, ErrorType.Error, _prim_site(stage, pp),
                        f"ExtendedPhysicsPositionBasedClampingAPI on '{pp}' has "
                        f"non-monotonically-increasing lookupPositions at index {i} "
                        f"(values {positions[i]} >= {positions[i + 1]}). "
                        "Positions must be monotonically increasing per REP §4.2.2.",
                        "Sort lookupPositions in strictly ascending order.",
                    )
                    break


register_stage_validator(
    "ExtendedPhysicsMimic", _validate_extended_physics_mimic,
    doc="REP §4.2.2: ExtendedPhysicsMimicAPI on revolute/prismatic joints with valid DAG.",
    keywords=["rep0158:extended"], section="4.2",
)
register_stage_validator(
    "ExtendedPhysicsActuator", _validate_extended_physics_actuator,
    doc="REP §4.2.2: ExtendedPhysicsActuatorAPI targets must point to valid joints.",
    keywords=["rep0158:extended"], section="4.2",
)
register_stage_validator(
    "ExtendedPhysicsPositionClamping", _validate_extended_physics_position_clamping,
    doc="REP §4.2.2: Lookup table must have equal-length monotonic arrays.",
    keywords=["rep0158:extended"], section="4.2",
)
