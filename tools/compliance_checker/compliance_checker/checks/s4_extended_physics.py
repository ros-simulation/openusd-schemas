"""Section 4.2 – Extended Physics Schema checks."""

from __future__ import annotations

from collections import defaultdict

from pxr import Usd

from .base import (
    ErrorType,
    TimeRange,
    ValidationError,
    _error,
    _prim_site,
    register_stage_validator,
)


def _applied(prim: Usd.Prim) -> set[str]:
    list_op = prim.GetMetadata("apiSchemas")
    if list_op is None:
        return set()
    return set(list_op.GetAppliedItems())


_MIMIC_ALLOWED_TYPES = {"PhysicsRevoluteJoint", "PhysicsPrismaticJoint"}


def _check_cycle_free_graph(
    mimic_graph: dict[str, list[str]], stage: Usd.Stage
) -> list[ValidationError]:
    errors: list[ValidationError] = []
    visiting: set[str] = set()
    visited: set[str] = set()

    def dfs(node: str, stack: list[str]) -> None:
        if node in visiting:
            cycle = stack[stack.index(node):] + [node]
            errors.append(_error(
                "4.2.3",
                ErrorType.Error,
                _prim_site(stage, node),
                (
                    "ExtendedPhysicsMimicAPI relationships must form a DAG. "
                    f"Detected cycle: {' -> '.join(cycle)}."
                ),
                suggestion="Break the mimic cycle by removing one coupling edge.",
            ))
            return
        if node in visited:
            return
        visiting.add(node)
        stack.append(node)
        for target in mimic_graph.get(node, []):
            dfs(target, stack)
        stack.pop()
        visiting.remove(node)
        visited.add(node)

    for node in mimic_graph:
        dfs(node, [])

    return errors


def _validate_extended_physics_mimic(
    stage: Usd.Stage, timeRange: TimeRange
) -> list[ValidationError]:
    errors: list[ValidationError] = []
    mimic_graph: dict[str, list[str]] = defaultdict(list)

    for prim in stage.TraverseAll():
        if "ExtendedPhysicsMimicAPI" not in _applied(prim):
            continue
        type_name = prim.GetTypeName()
        prim_path = str(prim.GetPath())

        if type_name not in _MIMIC_ALLOWED_TYPES:
            errors.append(_error(
                "4.2.1",
                ErrorType.Warn,
                _prim_site(stage, prim_path),
                (
                    f"ExtendedPhysicsMimicAPI is applied to '{prim_path}' (type: '{type_name}'). "
                    "ExtendedPhysicsMimicAPI must only be applied to PhysicsRevoluteJoint or "
                    "PhysicsPrismaticJoint per REP §4.2.2."
                ),
                suggestion=(
                    "Remove ExtendedPhysicsMimicAPI from this prim or change the joint type "
                    "to PhysicsRevoluteJoint / PhysicsPrismaticJoint."
                ),
            ))
            continue

        rel = prim.GetRelationship("ext_physics:mimic:joint")
        if not rel.IsValid() or not rel.GetTargets():
            errors.append(_error(
                "4.2.2",
                ErrorType.Error,
                _prim_site(stage, prim_path),
                (
                    f"ExtendedPhysicsMimicAPI on '{prim_path}' is missing required "
                    "relationship 'ext_physics:mimic:joint'. "
                    "The leader joint must be specified per REP §4.2.2."
                ),
                suggestion=(
                    "Author `rel ext_physics:mimic:joint = </path/to/source_joint>` "
                    "targeting the source revolute/prismatic joint."
                ),
            ))
            continue

        targets = rel.GetTargets()
        if len(targets) != 1:
            errors.append(_error(
                "4.2.2",
                ErrorType.Error,
                _prim_site(stage, prim_path),
                (
                    f"'ext_physics:mimic:joint' on '{prim_path}' must target exactly one "
                    f"source joint; found {len(targets)} targets."
                ),
                suggestion="Keep exactly one relationship target.",
            ))
            continue

        target = targets[0]
        source = stage.GetPrimAtPath(target)
        if not source or not source.IsValid():
            errors.append(_error(
                "4.2.2",
                ErrorType.Error,
                _prim_site(stage, prim_path),
                (
                    f"'ext_physics:mimic:joint' on '{prim_path}' targets '{target}', "
                    "which does not exist in the composed stage."
                ),
                suggestion="Point to an existing revolute/prismatic joint prim.",
            ))
            continue

        if source.GetTypeName() not in _MIMIC_ALLOWED_TYPES:
            errors.append(_error(
                "4.2.2",
                ErrorType.Error,
                _prim_site(stage, prim_path),
                (
                    f"'ext_physics:mimic:joint' on '{prim_path}' targets '{target}' "
                    f"(type '{source.GetTypeName()}'). "
                    "Source joint must be revolute or prismatic per REP §4.2.2."
                ),
                suggestion="Target a PhysicsRevoluteJoint or PhysicsPrismaticJoint.",
            ))
            continue

        mimic_graph[prim_path].append(str(target))

    errors.extend(_check_cycle_free_graph(mimic_graph, stage))
    return errors


_ACTUATOR_ALLOWED_TYPES = {"PhysicsRevoluteJoint", "PhysicsPrismaticJoint"}


def _validate_extended_physics_actuator(
    stage: Usd.Stage, timeRange: TimeRange
) -> list[ValidationError]:
    errors: list[ValidationError] = []
    for prim in stage.TraverseAll():
        if "ExtendedPhysicsActuatorAPI" not in _applied(prim):
            continue
        prim_path = str(prim.GetPath())
        rel = prim.GetRelationship("ext_physics:actuator:targets")
        if not rel.IsValid() or not rel.GetTargets():
            errors.append(_error(
                "4.2.4",
                ErrorType.Error,
                _prim_site(stage, prim_path),
                (
                    f"ExtendedPhysicsActuatorAPI on '{prim_path}' is missing required "
                    "relationship 'ext_physics:actuator:targets'. "
                    "The target joint must be specified per REP §4.2.2."
                ),
                suggestion=(
                    "Author `rel ext_physics:actuator:targets = </path/to/joint>` "
                    "targeting a PhysicsRevoluteJoint or PhysicsPrismaticJoint."
                ),
            ))
            continue

        for target in rel.GetTargets():
            target_prim = stage.GetPrimAtPath(target)
            if not target_prim or not target_prim.IsValid():
                errors.append(_error(
                    "4.2.4",
                    ErrorType.Error,
                    _prim_site(stage, prim_path),
                    (
                        f"'ext_physics:actuator:targets' on '{prim_path}' targets '{target}', "
                        "which does not exist in the composed stage."
                    ),
                    suggestion="Point to an existing PhysicsRevoluteJoint or PhysicsPrismaticJoint.",
                ))
                continue

            type_name = target_prim.GetTypeName()
            if type_name not in _ACTUATOR_ALLOWED_TYPES:
                errors.append(_error(
                    "4.2.4",
                    ErrorType.Error,
                    _prim_site(stage, prim_path),
                    (
                        f"'ext_physics:actuator:targets' on '{prim_path}' targets '{target}' "
                        f"(type '{type_name}'). Only PhysicsRevoluteJoint or "
                        "PhysicsPrismaticJoint are supported per REP §4.2.2. "
                        "Multi-DOF joints are not supported."
                    ),
                    suggestion=(
                        "Target a PhysicsRevoluteJoint or PhysicsPrismaticJoint. "
                        "Multi-DOF PhysicsJoints are not supported by ExtendedPhysicsActuatorAPI."
                    ),
                ))
    return errors


def _validate_extended_physics_position_clamping(
    stage: Usd.Stage, timeRange: TimeRange
) -> list[ValidationError]:
    errors: list[ValidationError] = []
    for prim in stage.TraverseAll():
        if "ExtendedPhysicsPositionBasedClampingAPI" not in _applied(prim):
            continue
        prim_path = str(prim.GetPath())
        pos_attr = prim.GetAttribute("ext_physics:clamp_position:lookupPositions")
        eff_attr = prim.GetAttribute("ext_physics:clamp_position:lookupEfforts")

        positions = pos_attr.Get() if pos_attr.IsValid() else None
        efforts = eff_attr.Get() if eff_attr.IsValid() else None

        if positions is None and efforts is None:
            continue

        positions = list(positions) if positions is not None else []
        efforts = list(efforts) if efforts is not None else []

        if len(positions) != len(efforts):
            errors.append(_error(
                "4.2.5",
                ErrorType.Error,
                _prim_site(stage, prim_path),
                (
                    f"ExtendedPhysicsPositionBasedClampingAPI on '{prim_path}' has "
                    f"lookupPositions length {len(positions)} but lookupEfforts length "
                    f"{len(efforts)}. Both arrays must be the same length per REP §4.2.2."
                ),
                suggestion=(
                    "Ensure ext_physics:clamp_position:lookupPositions and "
                    "ext_physics:clamp_position:lookupEfforts have the same number of elements."
                ),
            ))

        if len(positions) >= 2:
            for i in range(len(positions) - 1):
                if positions[i] >= positions[i + 1]:
                    errors.append(_error(
                        "4.2.5",
                        ErrorType.Error,
                        _prim_site(stage, prim_path),
                        (
                            f"ExtendedPhysicsPositionBasedClampingAPI on '{prim_path}' has "
                            f"non-monotonically-increasing lookupPositions at index {i} "
                            f"(values {positions[i]} >= {positions[i + 1]}). "
                            "Positions must be monotonically increasing per REP §4.2.2."
                        ),
                        suggestion=(
                            "Sort ext_physics:clamp_position:lookupPositions in strictly "
                            "ascending order."
                        ),
                    ))
                    break
    return errors


# ------------------------------------------------------------------ #
# Registration                                                          #
# ------------------------------------------------------------------ #

register_stage_validator(
    "ExtendedPhysicsMimic",
    _validate_extended_physics_mimic,
    doc="REP §4.2.2: ExtendedPhysicsMimicAPI must only be on revolute/prismatic joints, with valid DAG.",
    keywords=["rep0158:extended"],
    section="4.2",
)

register_stage_validator(
    "ExtendedPhysicsActuator",
    _validate_extended_physics_actuator,
    doc="REP §4.2.2: ExtendedPhysicsActuatorAPI must have authored targets pointing to valid joints.",
    keywords=["rep0158:extended"],
    section="4.2",
)

register_stage_validator(
    "ExtendedPhysicsPositionClamping",
    _validate_extended_physics_position_clamping,
    doc="REP §4.2.2: ExtendedPhysicsPositionBasedClampingAPI lookup table must have equal-length monotonic arrays.",
    keywords=["rep0158:extended"],
    section="4.2",
)
