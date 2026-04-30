"""Section 1.3 – Physics checks."""

from __future__ import annotations

from collections import defaultdict

from pxr import Usd, UsdGeom, UsdPhysics, UsdShade

from .base import (
    ErrorType,
    TimeRange,
    _error,
    _prim_site,
    _stage_site,
    register_stage_validator,
)


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


# ------------------------------------------------------------------ #
# JointLimits                                                          #
# ------------------------------------------------------------------ #


def _check_joint_limits(stage: Usd.Stage, timeRange: TimeRange) -> list:
    errors = []
    for prim in stage.TraverseAll():
        type_name = prim.GetTypeName()
        if type_name not in (
            "PhysicsRevoluteJoint",
            "PhysicsPrismaticJoint",
        ):
            continue
        prim_path = str(prim.GetPath())
        lower = prim.GetAttribute("physics:lowerLimit")
        upper = prim.GetAttribute("physics:upperLimit")

        if not lower.IsAuthored():
            errors.append(
                _error(
                    "1.3.1",
                    ErrorType.Error,
                    _prim_site(stage, prim_path),
                    (
                        f"Joint '{prim_path}' ({type_name}) is missing "
                        "'physics:lowerLimit'. Non-continuous joints must author explicit limits "
                        "per REP §1.3."
                    ),
                    suggestion="Author `float physics:lowerLimit = <value>` on this joint prim.",
                )
            )

        if not upper.IsAuthored():
            errors.append(
                _error(
                    "1.3.1",
                    ErrorType.Error,
                    _prim_site(stage, prim_path),
                    (
                        f"Joint '{prim_path}' ({type_name}) is missing "
                        "'physics:upperLimit'. Non-continuous joints must author explicit limits "
                        "per REP §1.3."
                    ),
                    suggestion="Author `float physics:upperLimit = <value>` on this joint prim.",
                )
            )
    return errors


register_stage_validator(
    "JointLimits",
    _check_joint_limits,
    doc="REP §1.3: Non-continuous joints must author explicit limit attributes.",
    section="1.3",
)


# ------------------------------------------------------------------ #
# ArticulationRoot                                                     #
# ------------------------------------------------------------------ #


def _check_articulation_root(stage: Usd.Stage, timeRange: TimeRange) -> list:
    errors = []
    roots = [
        prim
        for prim in stage.TraverseAll()
        if _has_api(prim, UsdPhysics.ArticulationRootAPI)
    ]
    if len(roots) > 1:
        paths = ", ".join(str(p.GetPath()) for p in roots)
        errors.append(
            _error(
                "1.3.2",
                ErrorType.Warn,
                _stage_site(stage),
                (
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
        )
    return errors


register_stage_validator(
    "ArticulationRoot",
    _check_articulation_root,
    doc="REP §1.3: At most one ArticulationRootAPI per connected kinematic tree.",
    section="1.3",
)


# ------------------------------------------------------------------ #
# MassProperties                                                       #
# ------------------------------------------------------------------ #


def _check_mass_properties(stage: Usd.Stage, timeRange: TimeRange) -> list:
    errors = []
    for prim in stage.TraverseAll():
        if not _has_api(prim, UsdPhysics.RigidBodyAPI):
            continue
        if not _has_api(prim, UsdPhysics.MassAPI):
            continue
        mass_attr = prim.GetAttribute("physics:mass")
        if not mass_attr.IsValid():
            continue
        mass = mass_attr.Get()
        if mass is not None and mass <= 0.0:
            errors.append(
                _error(
                    "1.3.3",
                    ErrorType.Warn,
                    _prim_site(stage, str(prim.GetPath())),
                    (
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
            )
    return errors


register_stage_validator(
    "MassProperties",
    _check_mass_properties,
    doc="REP §1.3: Dynamic bodies must have mass > 0; no zero-mass hack.",
    section="1.3",
)


# ------------------------------------------------------------------ #
# CollisionMaterial                                                    #
# ------------------------------------------------------------------ #

_CM_REQUIRED_ATTRS = (
    "physics:staticFriction",
    "physics:dynamicFriction",
    "physics:restitution",
)


def _check_collision_material(stage: Usd.Stage, timeRange: TimeRange) -> list:
    errors = []
    for prim in stage.TraverseAll():
        if not _has_api(prim, UsdPhysics.CollisionAPI):
            continue
        prim_path = str(prim.GetPath())
        binding_api = UsdShade.MaterialBindingAPI(prim)
        direct = binding_api.GetDirectBinding("physics")
        if not direct.GetMaterialPath():
            errors.append(
                _error(
                    "1.3.5",
                    ErrorType.Warn,
                    _prim_site(stage, prim_path),
                    (
                        f"Collision geometry '{prim_path}' has no physics material binding "
                        "(material:binding:physics). Deterministic contact dynamics require a "
                        "UsdPhysicsMaterialAPI material bound with the physics purpose per REP §1.3.4."
                    ),
                    suggestion=(
                        "Create a UsdShadeMaterial with UsdPhysicsMaterialAPI and bind it via "
                        "`UsdShade.MaterialBindingAPI(prim).Bind(mat, purpose='physics')`."
                    ),
                )
            )
            continue

        mat_path = direct.GetMaterialPath()
        mat_prim = stage.GetPrimAtPath(mat_path)
        if not mat_prim.IsValid():
            continue

        for attr_name in _CM_REQUIRED_ATTRS:
            attr = mat_prim.GetAttribute(attr_name)
            if not attr.IsAuthored():
                errors.append(
                    _error(
                        "1.3.5",
                        ErrorType.Warn,
                        _prim_site(stage, str(mat_path)),
                        (
                            f"Physics material '{mat_path}' bound to '{prim_path}' is missing "
                            f"'{attr_name}'. All three contact physics attributes must be defined "
                            "per REP §1.3.4."
                        ),
                        suggestion=f"Author `float {attr_name} = <value>` on the material prim.",
                    )
                )
    return errors


register_stage_validator(
    "CollisionMaterial",
    _check_collision_material,
    doc="REP §1.3.4: Collision geometry must bind a physics material with friction/restitution.",
    section="1.3",
)


# ------------------------------------------------------------------ #
# CollisionGeometryAuthoring                                           #
# ------------------------------------------------------------------ #


def _check_collision_geometry_authoring(
    stage: Usd.Stage, timeRange: TimeRange
) -> list:
    errors = []
    for prim in stage.TraverseAll():
        if not _has_api(prim, UsdPhysics.CollisionAPI):
            continue
        prim_path = str(prim.GetPath())

        # Check purpose
        imageable = UsdGeom.Imageable(prim)
        if imageable:
            purpose_attr = imageable.GetPurposeAttr()
            purpose = purpose_attr.Get() if purpose_attr.IsValid() else None
            if purpose != UsdGeom.Tokens.guide:
                errors.append(
                    _error(
                        "1.3.4",
                        ErrorType.Warn,
                        _prim_site(stage, prim_path),
                        (
                            f"Collision geometry '{prim_path}' has purpose={purpose!r}. "
                            "Collision geometry should explicitly set purpose='guide' per REP §1.3.1."
                        ),
                        suggestion="Author `token purpose = 'guide'` on collision geometry prims.",
                    )
                )

        # Check approximation
        approximation_attr = prim.GetAttribute("physics:approximation")
        approximation = (
            approximation_attr.Get()
            if approximation_attr and approximation_attr.IsValid()
            else None
        )
        if approximation != "none":
            errors.append(
                _error(
                    "1.3.4",
                    ErrorType.Warn,
                    _prim_site(stage, prim_path),
                    (
                        f"Collision geometry '{prim_path}' has physics:approximation="
                        f"{approximation!r}. Collision geometry should explicitly set "
                        "physics:approximation='none' per REP §1.3.1."
                    ),
                    suggestion=(
                        "Author `token physics:approximation = 'none'` on collision geometry prims."
                    ),
                )
            )
    return errors


register_stage_validator(
    "CollisionGeometryAuthoring",
    _check_collision_geometry_authoring,
    doc="REP §1.3.1: Collision geometry should use guide purpose and none approximation.",
    section="1.3",
)


# ------------------------------------------------------------------ #
# MimicJoint                                                           #
# ------------------------------------------------------------------ #

_MIMIC_ALLOWED_TYPES = {"PhysicsRevoluteJoint", "PhysicsPrismaticJoint"}


def _check_mimic_cycle_free_graph(mimic_graph: dict[str, list[str]], stage: Usd.Stage) -> list:
    errors = []
    visiting: set[str] = set()
    visited: set[str] = set()

    def dfs(node: str, stack: list[str]) -> None:
        if node in visiting:
            cycle = stack[stack.index(node):] + [node]
            errors.append(
                _error(
                    "1.3.7",
                    ErrorType.Error,
                    _prim_site(stage, node),
                    (
                        "MimicJointAPI relationships must form a DAG. "
                        f"Detected cycle: {' -> '.join(cycle)}."
                    ),
                    suggestion="Break the mimic cycle by removing one coupling edge.",
                )
            )
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


def _check_mimic_joint(stage: Usd.Stage, timeRange: TimeRange) -> list:
    errors = []
    mimic_graph: dict[str, list[str]] = defaultdict(list)

    for prim in stage.TraverseAll():
        if "MimicJointAPI" not in _applied(prim):
            continue
        prim_path = str(prim.GetPath())
        errors.append(
            _error(
                "1.3.9",
                ErrorType.Warn,
                _prim_site(stage, prim_path),
                (
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
        )
        type_name = prim.GetTypeName()
        if type_name not in _MIMIC_ALLOWED_TYPES:
            errors.append(
                _error(
                    "1.3.7",
                    ErrorType.Warn,
                    _prim_site(stage, prim_path),
                    (
                        f"MimicJointAPI is applied to '{prim_path}' (type: '{type_name}'). "
                        "MimicJointAPI must only be applied to PhysicsRevoluteJoint or "
                        "PhysicsPrismaticJoint per REP §1.3."
                    ),
                    suggestion=(
                        "Remove MimicJointAPI from this prim or change the joint type to "
                        "PhysicsRevoluteJoint / PhysicsPrismaticJoint."
                    ),
                )
            )
            continue
        rel = prim.GetRelationship("mimic:joint")
        if not rel.IsValid():
            errors.append(
                _error(
                    "1.3.7",
                    ErrorType.Error,
                    _prim_site(stage, prim_path),
                    (
                        f"MimicJointAPI on '{prim_path}' is missing required relationship "
                        "'mimic:joint'."
                    ),
                    suggestion=(
                        "Author `rel mimic:joint = </path/to/source_joint>` targeting the "
                        "source revolute/prismatic joint."
                    ),
                )
            )
            continue
        targets = rel.GetTargets()
        if len(targets) != 1:
            errors.append(
                _error(
                    "1.3.7",
                    ErrorType.Error,
                    _prim_site(stage, prim_path),
                    (
                        f"'mimic:joint' on '{prim_path}' must target exactly one source joint; "
                        f"found {len(targets)} targets."
                    ),
                    suggestion="Keep exactly one relationship target.",
                )
            )
            continue
        target = targets[0]
        source = stage.GetPrimAtPath(target)
        if not source or not source.IsValid():
            errors.append(
                _error(
                    "1.3.7",
                    ErrorType.Error,
                    _prim_site(stage, prim_path),
                    (
                        f"'mimic:joint' on '{prim_path}' targets '{target}', which does not "
                        "exist in the composed stage."
                    ),
                    suggestion="Point to an existing revolute/prismatic joint prim.",
                )
            )
            continue
        if source.GetTypeName() not in _MIMIC_ALLOWED_TYPES:
            errors.append(
                _error(
                    "1.3.7",
                    ErrorType.Error,
                    _prim_site(stage, prim_path),
                    (
                        f"'mimic:joint' on '{prim_path}' targets '{target}' "
                        f"(type '{source.GetTypeName()}'). Source joint must be revolute or prismatic."
                    ),
                    suggestion="Target a PhysicsRevoluteJoint or PhysicsPrismaticJoint.",
                )
            )
            continue
        mimic_graph[prim_path].append(str(target))

    errors.extend(_check_mimic_cycle_free_graph(mimic_graph, stage))
    return errors


register_stage_validator(
    "MimicJoint",
    _check_mimic_joint,
    doc="REP §1.3: MimicJointAPI is deprecated; assets must use ExtendedPhysicsMimicAPI.",
    section="1.3",
)


# ------------------------------------------------------------------ #
# InstanceablePhysics                                                  #
# ------------------------------------------------------------------ #

_IP_FORBIDDEN_APIS = {
    "PhysicsRigidBodyAPI",
    "RosContextAPI",
    "RosTopicAPI",
    "RosServiceAPI",
    "RosActionAPI",
}
_IP_FORBIDDEN_TYPES = {
    "PhysicsRevoluteJoint",
    "PhysicsPrismaticJoint",
    "PhysicsFixedJoint",
    "PhysicsSphericalJoint",
    "PhysicsDistanceJoint",
    "PhysicsJoint",
}


def _check_instanceable_physics(stage: Usd.Stage, timeRange: TimeRange) -> list:
    errors = []
    for prim in stage.TraverseAll():
        if not prim.GetMetadata("instanceable"):
            continue
        applied_schemas = _applied(prim)
        forbidden_apis = applied_schemas & _IP_FORBIDDEN_APIS
        forbidden_type = prim.GetTypeName() in _IP_FORBIDDEN_TYPES
        if forbidden_apis or forbidden_type:
            reason = (
                f"applied schemas: {forbidden_apis}"
                if forbidden_apis
                else f"type: {prim.GetTypeName()}"
            )
            errors.append(
                _error(
                    "1.3.8",
                    ErrorType.Error,
                    _prim_site(stage, str(prim.GetPath())),
                    (
                        f"Prim '{prim.GetPath()}' has instanceable=true but carries physics "
                        f"or ROS schemas ({reason}). Instance proxies obscure child prims from "
                        "relationship targeting, breaking joints and ROS interfaces per REP §3.5."
                    ),
                    suggestion=(
                        "Only set instanceable=true on leaf visual/collision geometry. "
                        "Move physics and ROS schemas to a non-instanceable parent."
                    ),
                )
            )
    return errors


register_stage_validator(
    "InstanceablePhysics",
    _check_instanceable_physics,
    doc="REP §1.2.6: instanceable=true must not be set on physics/ROS prims.",
    section="1.3",
)
