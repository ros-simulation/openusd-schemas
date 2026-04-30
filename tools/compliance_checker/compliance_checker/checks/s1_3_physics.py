"""Section 1.3 – Physics checks."""

from __future__ import annotations

from collections import defaultdict
from typing import Iterator

from pxr import Usd, UsdGeom, UsdPhysics, UsdShade

from ..report import Severity, Violation
from .base import BaseCheck


def _has_api(prim: Usd.Prim, api_type) -> bool:
    return prim.HasAPI(api_type)


def _applied(prim: Usd.Prim) -> set[str]:
    """Return the set of applied API schema names, including unregistered custom schemas.

    ``prim.GetAppliedSchemas()`` silently drops schemas not present in the schema registry,
    which excludes our custom Ros*API schemas when the plugin is not loaded.
    Reading the composed ``apiSchemas`` metadata directly avoids this filter.
    """
    list_op = prim.GetMetadata("apiSchemas")
    if list_op is None:
        return set()
    return set(list_op.GetAppliedItems())


class JointLimitsCheck(BaseCheck):
    """REP §1.3: Non-continuous joints must author explicit limit attributes."""

    section = "1.3"

    _JOINT_TYPES = {
        "PhysicsRevoluteJoint": UsdPhysics.RevoluteJoint,
        "PhysicsPrismaticJoint": UsdPhysics.PrismaticJoint,
    }

    def run(self, stage: Usd.Stage) -> Iterator[Violation]:
        for prim in stage.TraverseAll():
            type_name = prim.GetTypeName()
            if type_name not in self._JOINT_TYPES:
                continue
            yield from self._check_limits(prim)

    def _check_limits(self, prim: Usd.Prim) -> Iterator[Violation]:
        # UsdPhysics defaults are -inf / +inf (meaning no limit). Check IsAuthored()
        # rather than Get() == None, because the default is a valid float, not None.
        prim_path = str(prim.GetPath())
        lower = prim.GetAttribute("physics:lowerLimit")
        upper = prim.GetAttribute("physics:upperLimit")

        if not lower.IsAuthored():
            yield Violation(
                check_id="1.3.1",
                severity=Severity.ERROR,
                prim_path=prim_path,
                section=self.section,
                message=(
                    f"Joint '{prim_path}' ({prim.GetTypeName()}) is missing "
                    "'physics:lowerLimit'. Non-continuous joints must author explicit limits "
                    "per REP §1.3."
                ),
                suggestion="Author `float physics:lowerLimit = <value>` on this joint prim.",
            )

        if not upper.IsAuthored():
            yield Violation(
                check_id="1.3.1",
                severity=Severity.ERROR,
                prim_path=prim_path,
                section=self.section,
                message=(
                    f"Joint '{prim_path}' ({prim.GetTypeName()}) is missing "
                    "'physics:upperLimit'. Non-continuous joints must author explicit limits "
                    "per REP §1.3."
                ),
                suggestion="Author `float physics:upperLimit = <value>` on this joint prim.",
            )


class ArticulationRootCheck(BaseCheck):
    """REP §1.3: At most one ArticulationRootAPI per connected kinematic tree."""

    section = "1.3"

    def run(self, stage: Usd.Stage) -> Iterator[Violation]:
        roots = [
            prim
            for prim in stage.TraverseAll()
            if _has_api(prim, UsdPhysics.ArticulationRootAPI)
        ]
        if len(roots) > 1:
            paths = ", ".join(str(p.GetPath()) for p in roots)
            yield Violation(
                check_id="1.3.2",
                severity=Severity.WARNING,
                prim_path="/",
                section=self.section,
                message=(
                    f"Stage contains {len(roots)} ArticulationRootAPI prims: [{paths}]. "
                    "There must be at most one per connected kinematic tree per REP §1.3. "
                    "Multiple roots in the same tree will fracture reduced-coordinate solvers."
                ),
                suggestion=(
                    "When composing a modular gripper/arm into a larger tree, use the "
                    "list-edit op `delete apiSchemas = ['PhysicsArticulationRootAPI']` "
                    "to prune nested roots in the composing stage."
                ),
            )


class MassPropertiesCheck(BaseCheck):
    """REP §1.3: Dynamic bodies must have mass > 0; no zero-mass hack."""

    section = "1.3"

    def run(self, stage: Usd.Stage) -> Iterator[Violation]:
        for prim in stage.TraverseAll():
            if not _has_api(prim, UsdPhysics.RigidBodyAPI):
                continue
            yield from self._check_mass(prim)

    def _check_mass(self, prim: Usd.Prim) -> Iterator[Violation]:
        if not _has_api(prim, UsdPhysics.MassAPI):
            return
        mass_attr = prim.GetAttribute("physics:mass")
        if not mass_attr.IsValid():
            return
        mass = mass_attr.Get()
        if mass is not None and mass <= 0.0:
            yield Violation(
                check_id="1.3.3",
                severity=Severity.WARNING,
                prim_path=str(prim.GetPath()),
                section=self.section,
                message=(
                    f"Prim '{prim.GetPath()}' has PhysicsRigidBodyAPI with non-positive "
                    f"physics:mass = {mass}. Dynamic bodies must define strictly positive "
                    "mass per REP §1.3."
                ),
                suggestion=(
                    "For static environments: omit PhysicsRigidBodyAPI (keep only CollisionAPI). "
                    "For robot anchors: set a valid mass > 0 and anchor via UsdPhysicsFixedJoint "
                    "with an empty body0 relationship. "
                    "For kinematic bodies: set physics:kinematicEnabled = true."
                ),
            )


class CollisionMaterialCheck(BaseCheck):
    """REP §1.3.4: Collision geometry must bind a physics material with friction/restitution."""

    section = "1.3"

    _REQUIRED_ATTRS = (
        "physics:staticFriction",
        "physics:dynamicFriction",
        "physics:restitution",
    )

    def run(self, stage: Usd.Stage) -> Iterator[Violation]:
        for prim in stage.TraverseAll():
            if not _has_api(prim, UsdPhysics.CollisionAPI):
                continue
            yield from self._check_physics_material_binding(prim, stage)

    def _check_physics_material_binding(
        self, prim: Usd.Prim, stage: Usd.Stage
    ) -> Iterator[Violation]:
        prim_path = str(prim.GetPath())
        binding_api = UsdShade.MaterialBindingAPI(prim)
        # Check for a physics-purpose binding
        direct = binding_api.GetDirectBinding("physics")
        if not direct.GetMaterialPath():
            yield Violation(
                check_id="1.3.5",
                severity=Severity.WARNING,
                prim_path=prim_path,
                section=self.section,
                message=(
                    f"Collision geometry '{prim_path}' has no physics material binding "
                    "(material:binding:physics). Deterministic contact dynamics require a "
                    "UsdPhysicsMaterialAPI material bound with the physics purpose per REP §1.3.4."
                ),
                suggestion=(
                    "Create a UsdShadeMaterial with UsdPhysicsMaterialAPI and bind it via "
                    "`UsdShade.MaterialBindingAPI(prim).Bind(mat, purpose='physics')`."
                ),
            )
            return

        # Material exists – verify it defines the required attributes
        mat_path = direct.GetMaterialPath()
        mat_prim = stage.GetPrimAtPath(mat_path)
        if not mat_prim.IsValid():
            return

        for attr_name in self._REQUIRED_ATTRS:
            attr = mat_prim.GetAttribute(attr_name)
            # UsdPhysics gives these schema defaults (0.0), so use IsAuthored()
            if not attr.IsAuthored():
                yield Violation(
                    check_id="1.3.5",
                    severity=Severity.WARNING,
                    prim_path=str(mat_path),
                    section=self.section,
                    message=(
                        f"Physics material '{mat_path}' bound to '{prim_path}' is missing "
                        f"'{attr_name}'. All three contact physics attributes must be defined "
                        "per REP §1.3.4."
                    ),
                    suggestion=f"Author `float {attr_name} = <value>` on the material prim.",
                )


class CollisionGeometryAuthoringCheck(BaseCheck):
    """REP §1.3.1: Collision geometry should use guide purpose and none approximation."""

    section = "1.3"

    def run(self, stage: Usd.Stage) -> Iterator[Violation]:
        for prim in stage.TraverseAll():
            if not _has_api(prim, UsdPhysics.CollisionAPI):
                continue
            yield from self._check_purpose(prim)
            yield from self._check_approximation(prim)

    def _check_purpose(self, prim: Usd.Prim) -> Iterator[Violation]:
        imageable = UsdGeom.Imageable(prim)
        if not imageable:
            return
        purpose_attr = imageable.GetPurposeAttr()
        purpose = purpose_attr.Get() if purpose_attr.IsValid() else None
        if purpose != UsdGeom.Tokens.guide:
            yield Violation(
                check_id="1.3.4",
                severity=Severity.WARNING,
                prim_path=str(prim.GetPath()),
                section=self.section,
                message=(
                    f"Collision geometry '{prim.GetPath()}' has purpose={purpose!r}. "
                    "Collision geometry should explicitly set purpose='guide' per REP §1.3.1."
                ),
                suggestion=(
                    "Author `token purpose = 'guide'` on collision geometry prims."
                ),
            )

    def _check_approximation(self, prim: Usd.Prim) -> Iterator[Violation]:
        approximation_attr = prim.GetAttribute("physics:approximation")
        approximation = (
            approximation_attr.Get()
            if approximation_attr and approximation_attr.IsValid()
            else None
        )
        if approximation != "none":
            yield Violation(
                check_id="1.3.4",
                severity=Severity.WARNING,
                prim_path=str(prim.GetPath()),
                section=self.section,
                message=(
                    f"Collision geometry '{prim.GetPath()}' has physics:approximation="
                    f"{approximation!r}. Collision geometry should explicitly set "
                    "physics:approximation='none' per REP §1.3.1."
                ),
                suggestion=(
                    "Author `token physics:approximation = 'none'` on collision geometry prims."
                ),
            )


class MimicJointCheck(BaseCheck):
    """REP §1.3: MimicJointAPI is deprecated; assets must use ExtendedPhysicsMimicAPI."""

    section = "1.3"

    _ALLOWED_TYPES = {"PhysicsRevoluteJoint", "PhysicsPrismaticJoint"}

    def run(self, stage: Usd.Stage) -> Iterator[Violation]:
        mimic_graph: dict[str, list[str]] = defaultdict(list)
        for prim in stage.TraverseAll():
            if "MimicJointAPI" not in _applied(prim):
                continue
            prim_path = str(prim.GetPath())
            yield Violation(
                check_id="1.3.9",
                severity=Severity.WARNING,
                prim_path=prim_path,
                section=self.section,
                message=(
                    f"Prim '{prim_path}' uses deprecated 'MimicJointAPI'. "
                    "REP §1.3 now requires 'ExtendedPhysicsMimicAPI' (ext_physics:mimic:*) "
                    "for mimic joint coupling. MimicJointAPI has been removed from the spec."
                ),
                suggestion=(
                    "Replace MimicJointAPI with ExtendedPhysicsMimicAPI and rename "
                    "'mimic:joint', 'mimic:multiplier', 'mimic:offset' relationships/attributes "
                    "to 'ext_physics:mimic:joint', 'ext_physics:mimic:multiplier', "
                    "'ext_physics:mimic:offset'."
                ),
            )
            type_name = prim.GetTypeName()
            prim_path = str(prim.GetPath())
            if type_name not in self._ALLOWED_TYPES:
                yield Violation(
                    check_id="1.3.7",
                    severity=Severity.WARNING,
                    prim_path=prim_path,
                    section=self.section,
                    message=(
                        f"MimicJointAPI is applied to '{prim_path}' (type: '{type_name}'). "
                        "MimicJointAPI must only be applied to PhysicsRevoluteJoint or "
                        "PhysicsPrismaticJoint per REP §1.3."
                    ),
                    suggestion=(
                        "Remove MimicJointAPI from this prim or change the joint type to "
                        "PhysicsRevoluteJoint / PhysicsPrismaticJoint."
                    ),
                )
                continue
            rel = prim.GetRelationship("mimic:joint")
            if not rel.IsValid():
                yield Violation(
                    check_id="1.3.7",
                    severity=Severity.ERROR,
                    prim_path=prim_path,
                    section=self.section,
                    message=(
                        f"MimicJointAPI on '{prim_path}' is missing required relationship "
                        "'mimic:joint'."
                    ),
                    suggestion=(
                        "Author `rel mimic:joint = </path/to/source_joint>` targeting the "
                        "source revolute/prismatic joint."
                    ),
                )
                continue
            targets = rel.GetTargets()
            if len(targets) != 1:
                yield Violation(
                    check_id="1.3.7",
                    severity=Severity.ERROR,
                    prim_path=prim_path,
                    section=self.section,
                    message=(
                        f"'mimic:joint' on '{prim_path}' must target exactly one source joint; "
                        f"found {len(targets)} targets."
                    ),
                    suggestion="Keep exactly one relationship target.",
                )
                continue
            target = targets[0]
            source = stage.GetPrimAtPath(target)
            if not source or not source.IsValid():
                yield Violation(
                    check_id="1.3.7",
                    severity=Severity.ERROR,
                    prim_path=prim_path,
                    section=self.section,
                    message=(
                        f"'mimic:joint' on '{prim_path}' targets '{target}', which does not "
                        "exist in the composed stage."
                    ),
                    suggestion="Point to an existing revolute/prismatic joint prim.",
                )
                continue
            if source.GetTypeName() not in self._ALLOWED_TYPES:
                yield Violation(
                    check_id="1.3.7",
                    severity=Severity.ERROR,
                    prim_path=prim_path,
                    section=self.section,
                    message=(
                        f"'mimic:joint' on '{prim_path}' targets '{target}' "
                        f"(type '{source.GetTypeName()}'). Source joint must be revolute or prismatic."
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
                    check_id="1.3.7",
                    severity=Severity.ERROR,
                    prim_path=node,
                    section=self.section,
                    message=(
                        "MimicJointAPI relationships must form a DAG. "
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


class InstanceablePhysicsCheck(BaseCheck):
    """REP §1.2.6: instanceable=true must not be set on physics/ROS prims."""

    section = "1.3"

    _FORBIDDEN_APIS = {
        "PhysicsRigidBodyAPI",
        "RosContextAPI",
        "RosTopicAPI",
        "RosServiceAPI",
        "RosActionAPI",
    }
    _FORBIDDEN_TYPES = {
        "PhysicsRevoluteJoint",
        "PhysicsPrismaticJoint",
        "PhysicsFixedJoint",
        "PhysicsSphericalJoint",
        "PhysicsDistanceJoint",
        "PhysicsPrismaticJoint",
        "PhysicsJoint",
    }

    def run(self, stage: Usd.Stage) -> Iterator[Violation]:
        for prim in stage.TraverseAll():
            if not prim.GetMetadata("instanceable"):
                continue
            applied = _applied(prim)
            forbidden_apis = applied & self._FORBIDDEN_APIS
            forbidden_type = prim.GetTypeName() in self._FORBIDDEN_TYPES
            if forbidden_apis or forbidden_type:
                reason = (
                    f"applied schemas: {forbidden_apis}"
                    if forbidden_apis
                    else f"type: {prim.GetTypeName()}"
                )
                yield Violation(
                    check_id="1.3.8",
                    severity=Severity.ERROR,
                    prim_path=str(prim.GetPath()),
                    section=self.section,
                    message=(
                        f"Prim '{prim.GetPath()}' has instanceable=true but carries physics "
                        f"or ROS schemas ({reason}). Instance proxies obscure child prims from "
                        "relationship targeting, breaking joints and ROS interfaces per REP §3.5."
                    ),
                    suggestion=(
                        "Only set instanceable=true on leaf visual/collision geometry. "
                        "Move physics and ROS schemas to a non-instanceable parent."
                    ),
                )
