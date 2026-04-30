"""Section 1.3 – Physics checks."""

from __future__ import annotations

from collections import defaultdict

from pxr import Usd, UsdGeom, UsdPhysics, UsdShade

from .base import (
    ErrorType, TimeRange, _error, _prim_site, _site, _stage_site,
    register_prim_validator, register_stage_validator,
)
from ._tokens import (
    COLLISION_GEOMETRY_AUTHORING,
    COLLISION_MISSING_MATERIAL,
    INSTANCEABLE_PHYSICS,
    MIMIC_JOINT_CONSTRAINT,
    MIMIC_JOINT_DEPRECATED,
    MISSING_JOINT_LIMITS,
    MULTIPLE_ARTICULATION_ROOTS,
    NON_POSITIVE_MASS,
)


def _applied(prim: Usd.Prim) -> set[str]:
    """Return applied API schema names, including unregistered custom schemas."""
    list_op = prim.GetMetadata("apiSchemas")
    if list_op is None:
        return set()
    return set(list_op.GetAppliedItems())


# -- Prim-level validators ------------------------------------------------

def _check_joint_limits(prim: Usd.Prim, timeRange: TimeRange):
    pp = str(prim.GetPath())
    type_name = prim.GetTypeName()
    if not prim.GetAttribute("physics:lowerLimit").IsAuthored():
        yield _error(
            MISSING_JOINT_LIMITS, ErrorType.Error, _site(prim),
            f"Joint '{pp}' ({type_name}) is missing 'physics:lowerLimit'. "
            "Non-continuous joints must author explicit limits per REP §1.3.",
            "Author `float physics:lowerLimit = <value>` on this joint prim.",
        )
    if not prim.GetAttribute("physics:upperLimit").IsAuthored():
        yield _error(
            MISSING_JOINT_LIMITS, ErrorType.Error, _site(prim),
            f"Joint '{pp}' ({type_name}) is missing 'physics:upperLimit'. "
            "Non-continuous joints must author explicit limits per REP §1.3.",
            "Author `float physics:upperLimit = <value>` on this joint prim.",
        )


register_prim_validator(
    "JointLimits", _check_joint_limits,
    doc="REP §1.3: Non-continuous joints must author explicit limit attributes.",
    section="1.3",
    schema_types=["UsdPhysicsRevoluteJoint", "UsdPhysicsPrismaticJoint"],
)


def _check_mass_properties(prim: Usd.Prim, timeRange: TimeRange):
    if not prim.HasAPI(UsdPhysics.MassAPI):
        return
    mass_attr = prim.GetAttribute("physics:mass")
    if not mass_attr.IsValid():
        return
    mass = mass_attr.Get()
    if mass is not None and mass <= 0.0:
        pp = str(prim.GetPath())
        yield _error(
            NON_POSITIVE_MASS, ErrorType.Warn, _site(prim),
            f"Prim '{pp}' has PhysicsRigidBodyAPI with non-positive "
            f"physics:mass = {mass}. Dynamic bodies must define strictly positive "
            "mass per REP §1.3.",
            "For static environments: omit PhysicsRigidBodyAPI (keep only CollisionAPI). "
            "For robot anchors: set a valid mass > 0 and anchor via UsdPhysicsFixedJoint "
            "with an empty body0 relationship. "
            "For kinematic bodies: set physics:kinematicEnabled = true.",
        )


register_prim_validator(
    "MassProperties", _check_mass_properties,
    doc="REP §1.3: Dynamic bodies must have mass > 0; no zero-mass hack.",
    section="1.3",
    schema_types=["UsdPhysicsRigidBodyAPI"],
)


_CM_REQUIRED_ATTRS = (
    "physics:staticFriction",
    "physics:dynamicFriction",
    "physics:restitution",
)


def _check_collision_material(prim: Usd.Prim, timeRange: TimeRange):
    pp = str(prim.GetPath())
    binding_api = UsdShade.MaterialBindingAPI(prim)
    direct = binding_api.GetDirectBinding("physics")
    if not direct.GetMaterialPath():
        yield _error(
            COLLISION_MISSING_MATERIAL, ErrorType.Warn, _site(prim),
            f"Collision geometry '{pp}' has no physics material binding "
            "(material:binding:physics). Deterministic contact dynamics require a "
            "UsdPhysicsMaterialAPI material bound with the physics purpose per REP §1.3.4.",
            "Create a UsdShadeMaterial with UsdPhysicsMaterialAPI and bind it via "
            "`UsdShade.MaterialBindingAPI(prim).Bind(mat, purpose='physics')`.",
        )
        return
    mat_path = direct.GetMaterialPath()
    mat_prim = prim.GetStage().GetPrimAtPath(mat_path)
    if not mat_prim.IsValid():
        return
    for attr_name in _CM_REQUIRED_ATTRS:
        if not mat_prim.GetAttribute(attr_name).IsAuthored():
            yield _error(
                COLLISION_MISSING_MATERIAL, ErrorType.Warn,
                _site(prim),
                f"Physics material '{mat_path}' bound to '{pp}' is missing "
                f"'{attr_name}'. All three contact physics attributes must be defined "
                "per REP §1.3.4.",
                f"Author `float {attr_name} = <value>` on the material prim.",
            )


register_prim_validator(
    "CollisionMaterial", _check_collision_material,
    doc="REP §1.3.4: Collision geometry must bind a physics material with friction/restitution.",
    section="1.3",
    schema_types=["UsdPhysicsCollisionAPI"],
)


def _check_collision_geometry_authoring(prim: Usd.Prim, timeRange: TimeRange):
    pp = str(prim.GetPath())
    imageable = UsdGeom.Imageable(prim)
    if imageable:
        purpose_attr = imageable.GetPurposeAttr()
        purpose = purpose_attr.Get() if purpose_attr.IsValid() else None
        if purpose != UsdGeom.Tokens.guide:
            yield _error(
                COLLISION_GEOMETRY_AUTHORING, ErrorType.Warn, _site(prim),
                f"Collision geometry '{pp}' has purpose={purpose!r}. "
                "Collision geometry should explicitly set purpose='guide' per REP §1.3.1.",
                "Author `token purpose = 'guide'` on collision geometry prims.",
            )
    approx_attr = prim.GetAttribute("physics:approximation")
    approx = approx_attr.Get() if approx_attr and approx_attr.IsValid() else None
    if approx != "none":
        yield _error(
            COLLISION_GEOMETRY_AUTHORING, ErrorType.Warn, _site(prim),
            f"Collision geometry '{pp}' has physics:approximation={approx!r}. "
            "Collision geometry should explicitly set "
            "physics:approximation='none' per REP §1.3.1.",
            "Author `token physics:approximation = 'none'` on collision geometry prims.",
        )


register_prim_validator(
    "CollisionGeometryAuthoring", _check_collision_geometry_authoring,
    doc="REP §1.3.1: Collision geometry should use guide purpose and none approximation.",
    section="1.3",
    schema_types=["UsdPhysicsCollisionAPI"],
)


# -- Stage-level validators (need cross-prim context) ---------------------

def _check_articulation_root(stage: Usd.Stage, timeRange: TimeRange):
    roots = [p for p in stage.TraverseAll() if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
    if len(roots) > 1:
        paths = ", ".join(str(p.GetPath()) for p in roots)
        yield _error(
            MULTIPLE_ARTICULATION_ROOTS, ErrorType.Warn, _stage_site(stage),
            f"Stage contains {len(roots)} ArticulationRootAPI prims: [{paths}]. "
            "There must be at most one per connected kinematic tree per REP §1.3. "
            "Multiple roots in the same tree will fracture reduced-coordinate solvers.",
            "When composing a modular gripper/arm into a larger tree, use the "
            "list-edit op `delete apiSchemas = ['PhysicsArticulationRootAPI']` "
            "to prune nested roots in the composing stage.",
        )


register_stage_validator(
    "ArticulationRoot", _check_articulation_root,
    doc="REP §1.3: At most one ArticulationRootAPI per connected kinematic tree.",
    section="1.3",
)


_MIMIC_ALLOWED_TYPES = {"PhysicsRevoluteJoint", "PhysicsPrismaticJoint"}


def _check_mimic_joint(stage: Usd.Stage, timeRange: TimeRange):
    mimic_graph: dict[str, list[str]] = defaultdict(list)
    for prim in stage.TraverseAll():
        if "MimicJointAPI" not in _applied(prim):
            continue
        pp = str(prim.GetPath())
        yield _error(
            MIMIC_JOINT_DEPRECATED, ErrorType.Warn, _prim_site(stage, pp),
            f"Prim '{pp}' uses deprecated 'MimicJointAPI'. "
            "REP §1.3 now requires 'ExtendedPhysicsMimicAPI' (ext_physics:mimic:*) "
            "for mimic joint coupling. MimicJointAPI has been removed from the spec.",
            "Replace MimicJointAPI with ExtendedPhysicsMimicAPI and rename "
            "'mimic:joint', 'mimic:multiplier', 'mimic:offset' relationships/attributes "
            "to 'ext_physics:mimic:joint', 'ext_physics:mimic:multiplier', "
            "'ext_physics:mimic:offset'.",
        )
        type_name = prim.GetTypeName()
        if type_name not in _MIMIC_ALLOWED_TYPES:
            yield _error(
                MIMIC_JOINT_CONSTRAINT, ErrorType.Warn, _prim_site(stage, pp),
                f"MimicJointAPI is applied to '{pp}' (type: '{type_name}'). "
                "MimicJointAPI must only be applied to PhysicsRevoluteJoint or "
                "PhysicsPrismaticJoint per REP §1.3.",
                "Remove MimicJointAPI from this prim or change the joint type to "
                "PhysicsRevoluteJoint / PhysicsPrismaticJoint.",
            )
            continue
        rel = prim.GetRelationship("mimic:joint")
        if not rel.IsValid():
            yield _error(
                MIMIC_JOINT_CONSTRAINT, ErrorType.Error, _prim_site(stage, pp),
                f"MimicJointAPI on '{pp}' is missing required relationship 'mimic:joint'.",
                "Author `rel mimic:joint = </path/to/source_joint>` targeting the "
                "source revolute/prismatic joint.",
            )
            continue
        targets = rel.GetTargets()
        if len(targets) != 1:
            yield _error(
                MIMIC_JOINT_CONSTRAINT, ErrorType.Error, _prim_site(stage, pp),
                f"'mimic:joint' on '{pp}' must target exactly one source joint; "
                f"found {len(targets)} targets.",
                "Keep exactly one relationship target.",
            )
            continue
        target = targets[0]
        source = stage.GetPrimAtPath(target)
        if not source or not source.IsValid():
            yield _error(
                MIMIC_JOINT_CONSTRAINT, ErrorType.Error, _prim_site(stage, pp),
                f"'mimic:joint' on '{pp}' targets '{target}', which does not "
                "exist in the composed stage.",
                "Point to an existing revolute/prismatic joint prim.",
            )
            continue
        if source.GetTypeName() not in _MIMIC_ALLOWED_TYPES:
            yield _error(
                MIMIC_JOINT_CONSTRAINT, ErrorType.Error, _prim_site(stage, pp),
                f"'mimic:joint' on '{pp}' targets '{target}' "
                f"(type '{source.GetTypeName()}'). Source joint must be revolute or prismatic.",
                "Target a PhysicsRevoluteJoint or PhysicsPrismaticJoint.",
            )
            continue
        mimic_graph[pp].append(str(target))
    yield from _check_mimic_cycle_free(mimic_graph, stage)


def _check_mimic_cycle_free(graph, stage):
    visiting: set[str] = set()
    visited: set[str] = set()

    def dfs(node, stack):
        if node in visiting:
            cycle = stack[stack.index(node):] + [node]
            return [_error(
                MIMIC_JOINT_CONSTRAINT, ErrorType.Error, _prim_site(stage, node),
                "MimicJointAPI relationships must form a DAG. "
                f"Detected cycle: {' -> '.join(cycle)}.",
                "Break the mimic cycle by removing one coupling edge.",
            )]
        if node in visited:
            return []
        visiting.add(node)
        stack.append(node)
        errors = []
        for target in graph.get(node, []):
            errors.extend(dfs(target, stack))
        stack.pop()
        visiting.remove(node)
        visited.add(node)
        return errors

    for node in graph:
        yield from dfs(node, [])


register_stage_validator(
    "MimicJoint", _check_mimic_joint,
    doc="REP §1.3: MimicJointAPI is deprecated; assets must use ExtendedPhysicsMimicAPI.",
    section="1.3",
)


_IP_FORBIDDEN_APIS = {
    "PhysicsRigidBodyAPI", "RosContextAPI", "RosTopicAPI",
    "RosServiceAPI", "RosActionAPI",
}
_IP_FORBIDDEN_TYPES = {
    "PhysicsRevoluteJoint", "PhysicsPrismaticJoint", "PhysicsFixedJoint",
    "PhysicsSphericalJoint", "PhysicsDistanceJoint", "PhysicsJoint",
}


def _check_instanceable_physics(stage: Usd.Stage, timeRange: TimeRange):
    for prim in stage.TraverseAll():
        if not prim.GetMetadata("instanceable"):
            continue
        applied_schemas = _applied(prim)
        forbidden_apis = applied_schemas & _IP_FORBIDDEN_APIS
        forbidden_type = prim.GetTypeName() in _IP_FORBIDDEN_TYPES
        if forbidden_apis or forbidden_type:
            pp = str(prim.GetPath())
            reason = (f"applied schemas: {forbidden_apis}" if forbidden_apis
                      else f"type: {prim.GetTypeName()}")
            yield _error(
                INSTANCEABLE_PHYSICS, ErrorType.Error, _prim_site(stage, pp),
                f"Prim '{pp}' has instanceable=true but carries physics "
                f"or ROS schemas ({reason}). Instance proxies obscure child prims from "
                "relationship targeting, breaking joints and ROS interfaces per REP §3.5.",
                "Only set instanceable=true on leaf visual/collision geometry. "
                "Move physics and ROS schemas to a non-instanceable parent.",
            )


register_stage_validator(
    "InstanceablePhysics", _check_instanceable_physics,
    doc="REP §1.2.6: instanceable=true must not be set on physics/ROS prims.",
    section="1.3",
)
