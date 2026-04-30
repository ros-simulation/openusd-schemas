"""Section 4.2 – Extended Physics Schema checks."""

from __future__ import annotations

from collections import defaultdict
from typing import Iterator

from pxr import Usd

from ..report import Severity, Violation
from .base import BaseCheck


def _applied(prim: Usd.Prim) -> set[str]:
    list_op = prim.GetMetadata("apiSchemas")
    if list_op is None:
        return set()
    return set(list_op.GetAppliedItems())


class ExtendedPhysicsMimicCheck(BaseCheck):
    """REP §4.2.2: ExtendedPhysicsMimicAPI must only be on revolute/prismatic joints, with valid DAG."""

    section = "4.2"

    _ALLOWED_TYPES = {"PhysicsRevoluteJoint", "PhysicsPrismaticJoint"}

    def run(self, stage: Usd.Stage) -> Iterator[Violation]:
        mimic_graph: dict[str, list[str]] = defaultdict(list)
        for prim in stage.TraverseAll():
            if "ExtendedPhysicsMimicAPI" not in _applied(prim):
                continue
            type_name = prim.GetTypeName()
            prim_path = str(prim.GetPath())

            if type_name not in self._ALLOWED_TYPES:
                yield Violation(
                    check_id="4.2.1",
                    severity=Severity.WARNING,
                    prim_path=prim_path,
                    section=self.section,
                    message=(
                        f"ExtendedPhysicsMimicAPI is applied to '{prim_path}' (type: '{type_name}'). "
                        "ExtendedPhysicsMimicAPI must only be applied to PhysicsRevoluteJoint or "
                        "PhysicsPrismaticJoint per REP §4.2.2."
                    ),
                    suggestion=(
                        "Remove ExtendedPhysicsMimicAPI from this prim or change the joint type "
                        "to PhysicsRevoluteJoint / PhysicsPrismaticJoint."
                    ),
                )
                continue

            rel = prim.GetRelationship("ext_physics:mimic:joint")
            if not rel.IsValid() or not rel.GetTargets():
                yield Violation(
                    check_id="4.2.2",
                    severity=Severity.ERROR,
                    prim_path=prim_path,
                    section=self.section,
                    message=(
                        f"ExtendedPhysicsMimicAPI on '{prim_path}' is missing required "
                        "relationship 'ext_physics:mimic:joint'. "
                        "The leader joint must be specified per REP §4.2.2."
                    ),
                    suggestion=(
                        "Author `rel ext_physics:mimic:joint = </path/to/source_joint>` "
                        "targeting the source revolute/prismatic joint."
                    ),
                )
                continue

            targets = rel.GetTargets()
            if len(targets) != 1:
                yield Violation(
                    check_id="4.2.2",
                    severity=Severity.ERROR,
                    prim_path=prim_path,
                    section=self.section,
                    message=(
                        f"'ext_physics:mimic:joint' on '{prim_path}' must target exactly one "
                        f"source joint; found {len(targets)} targets."
                    ),
                    suggestion="Keep exactly one relationship target.",
                )
                continue

            target = targets[0]
            source = stage.GetPrimAtPath(target)
            if not source or not source.IsValid():
                yield Violation(
                    check_id="4.2.2",
                    severity=Severity.ERROR,
                    prim_path=prim_path,
                    section=self.section,
                    message=(
                        f"'ext_physics:mimic:joint' on '{prim_path}' targets '{target}', "
                        "which does not exist in the composed stage."
                    ),
                    suggestion="Point to an existing revolute/prismatic joint prim.",
                )
                continue

            if source.GetTypeName() not in self._ALLOWED_TYPES:
                yield Violation(
                    check_id="4.2.2",
                    severity=Severity.ERROR,
                    prim_path=prim_path,
                    section=self.section,
                    message=(
                        f"'ext_physics:mimic:joint' on '{prim_path}' targets '{target}' "
                        f"(type '{source.GetTypeName()}'). "
                        "Source joint must be revolute or prismatic per REP §4.2.2."
                    ),
                    suggestion="Target a PhysicsRevoluteJoint or PhysicsPrismaticJoint.",
                )
                continue

            mimic_graph[prim_path].append(str(target))

        yield from self._check_cycle_free_graph(mimic_graph)

    def _check_cycle_free_graph(
        self, mimic_graph: dict[str, list[str]]
    ) -> Iterator[Violation]:
        visiting: set[str] = set()
        visited: set[str] = set()

        def dfs(node: str, stack: list[str]) -> Iterator[Violation]:
            if node in visiting:
                cycle = stack[stack.index(node) :] + [node]
                yield Violation(
                    check_id="4.2.3",
                    severity=Severity.ERROR,
                    prim_path=node,
                    section=self.section,
                    message=(
                        "ExtendedPhysicsMimicAPI relationships must form a DAG. "
                        f"Detected cycle: {' -> '.join(cycle)}."
                    ),
                    suggestion="Break the mimic cycle by removing one coupling edge.",
                )
                return
            if node in visited:
                return
            visiting.add(node)
            stack.append(node)
            for target in mimic_graph.get(node, []):
                yield from dfs(target, stack)
            stack.pop()
            visiting.remove(node)
            visited.add(node)

        for node in mimic_graph:
            yield from dfs(node, [])


class ExtendedPhysicsActuatorCheck(BaseCheck):
    """REP §4.2.2: ExtendedPhysicsActuatorAPI must have authored targets pointing to valid joints."""

    section = "4.2"

    _ALLOWED_TYPES = {"PhysicsRevoluteJoint", "PhysicsPrismaticJoint"}

    def run(self, stage: Usd.Stage) -> Iterator[Violation]:
        for prim in stage.TraverseAll():
            if "ExtendedPhysicsActuatorAPI" not in _applied(prim):
                continue
            yield from self._check_actuator(prim, stage)

    def _check_actuator(self, prim: Usd.Prim, stage: Usd.Stage) -> Iterator[Violation]:
        prim_path = str(prim.GetPath())
        rel = prim.GetRelationship("ext_physics:actuator:targets")
        if not rel.IsValid() or not rel.GetTargets():
            yield Violation(
                check_id="4.2.4",
                severity=Severity.ERROR,
                prim_path=prim_path,
                section=self.section,
                message=(
                    f"ExtendedPhysicsActuatorAPI on '{prim_path}' is missing required "
                    "relationship 'ext_physics:actuator:targets'. "
                    "The target joint must be specified per REP §4.2.2."
                ),
                suggestion=(
                    "Author `rel ext_physics:actuator:targets = </path/to/joint>` "
                    "targeting a PhysicsRevoluteJoint or PhysicsPrismaticJoint."
                ),
            )
            return

        for target in rel.GetTargets():
            target_prim = stage.GetPrimAtPath(target)
            if not target_prim or not target_prim.IsValid():
                yield Violation(
                    check_id="4.2.4",
                    severity=Severity.ERROR,
                    prim_path=prim_path,
                    section=self.section,
                    message=(
                        f"'ext_physics:actuator:targets' on '{prim_path}' targets '{target}', "
                        "which does not exist in the composed stage."
                    ),
                    suggestion="Point to an existing PhysicsRevoluteJoint or PhysicsPrismaticJoint.",
                )
                continue

            type_name = target_prim.GetTypeName()
            if type_name not in self._ALLOWED_TYPES:
                yield Violation(
                    check_id="4.2.4",
                    severity=Severity.ERROR,
                    prim_path=prim_path,
                    section=self.section,
                    message=(
                        f"'ext_physics:actuator:targets' on '{prim_path}' targets '{target}' "
                        f"(type '{type_name}'). Only PhysicsRevoluteJoint or "
                        "PhysicsPrismaticJoint are supported per REP §4.2.2. "
                        "Multi-DOF joints are not supported."
                    ),
                    suggestion=(
                        "Target a PhysicsRevoluteJoint or PhysicsPrismaticJoint. "
                        "Multi-DOF PhysicsJoints are not supported by ExtendedPhysicsActuatorAPI."
                    ),
                )


class ExtendedPhysicsPositionClampingCheck(BaseCheck):
    """REP §4.2.2: ExtendedPhysicsPositionBasedClampingAPI lookup table must have equal-length monotonic arrays."""

    section = "4.2"

    def run(self, stage: Usd.Stage) -> Iterator[Violation]:
        for prim in stage.TraverseAll():
            if "ExtendedPhysicsPositionBasedClampingAPI" not in _applied(prim):
                continue
            yield from self._check_lookup_table(prim)

    def _check_lookup_table(self, prim: Usd.Prim) -> Iterator[Violation]:
        prim_path = str(prim.GetPath())
        pos_attr = prim.GetAttribute("ext_physics:clamp_position:lookupPositions")
        eff_attr = prim.GetAttribute("ext_physics:clamp_position:lookupEfforts")

        positions = pos_attr.Get() if pos_attr.IsValid() else None
        efforts = eff_attr.Get() if eff_attr.IsValid() else None

        if positions is None and efforts is None:
            return

        positions = list(positions) if positions is not None else []
        efforts = list(efforts) if efforts is not None else []

        if len(positions) != len(efforts):
            yield Violation(
                check_id="4.2.5",
                severity=Severity.ERROR,
                prim_path=prim_path,
                section=self.section,
                message=(
                    f"ExtendedPhysicsPositionBasedClampingAPI on '{prim_path}' has "
                    f"lookupPositions length {len(positions)} but lookupEfforts length "
                    f"{len(efforts)}. Both arrays must be the same length per REP §4.2.2."
                ),
                suggestion=(
                    "Ensure ext_physics:clamp_position:lookupPositions and "
                    "ext_physics:clamp_position:lookupEfforts have the same number of elements."
                ),
            )

        if len(positions) >= 2:
            for i in range(len(positions) - 1):
                if positions[i] >= positions[i + 1]:
                    yield Violation(
                        check_id="4.2.5",
                        severity=Severity.ERROR,
                        prim_path=prim_path,
                        section=self.section,
                        message=(
                            f"ExtendedPhysicsPositionBasedClampingAPI on '{prim_path}' has "
                            f"non-monotonically-increasing lookupPositions at index {i} "
                            f"(values {positions[i]} >= {positions[i + 1]}). "
                            "Positions must be monotonically increasing per REP §4.2.2."
                        ),
                        suggestion=(
                            "Sort ext_physics:clamp_position:lookupPositions in strictly "
                            "ascending order."
                        ),
                    )
                    break
