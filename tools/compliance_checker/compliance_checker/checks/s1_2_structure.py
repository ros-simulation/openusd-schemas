"""Section 1.2 – Asset Structure & Composition."""

from __future__ import annotations

from pxr import Kind, Sdf, Usd, UsdPhysics

from .base import (
    ErrorType,
    TimeRange,
    ValidationError,
    _error,
    _prim_site,
    _stage_site,
    register_stage_validator,
)
from ._tokens import (
    ABSOLUTE_OR_PROPRIETARY_PATH,
    CUSTOM_COMPOSITION_ATTR,
    HEAVY_LAYER_ASCII,
    INHERITS_SPECIALIZES_ARC,
    MISSING_ASSET_IDENTIFIER,
    MISSING_ASSET_VERSION,
    MISSING_DEFAULT_PRIM,
    NESTED_COMPONENT,
    PAYLOAD_GATES_KINEMATIC,
    POINT_INSTANCER_PHYSICS,
    SCHEMA_LAYER_BINARY,
    VARIANT_NO_DEFAULT,
)

_FORBIDDEN_SCHEMES = ("omniverse://",)
_PREFAB_ATTR_HINTS = ("prefabpath", "assetref", "assetpath", "spawnpath", "modelpath")
_KINEMATIC_JOINT_TYPES = {
    "PhysicsRevoluteJoint", "PhysicsPrismaticJoint", "PhysicsFixedJoint",
    "PhysicsSphericalJoint", "PhysicsDistanceJoint", "PhysicsJoint",
}
_PHYSICS_ROS_APIS = {
    "PhysicsRigidBodyAPI", "PhysicsArticulationRootAPI",
    "RosContextAPI", "RosTopicAPI", "RosServiceAPI", "RosActionAPI",
}


def _applied(prim: Usd.Prim) -> set[str]:
    list_op = prim.GetMetadata("apiSchemas")
    if list_op is None:
        return set()
    return set(list_op.GetAppliedItems())


def _check_asset_management(
    stage: Usd.Stage, timeRange: TimeRange,
) -> list[ValidationError]:
    errors: list[ValidationError] = []
    default_prim = stage.GetDefaultPrim()
    if not (default_prim and default_prim.IsValid()):
        return [_error(
            MISSING_DEFAULT_PRIM, ErrorType.Warn, _stage_site(stage),
            "defaultPrim metadata is not set on the root layer. "
            "Referencing this asset without an explicit prim path is undefined behaviour "
            "and the Payload pattern breaks silently.",
            "Set stage.SetDefaultPrim(prim) or author "
            '`defaultPrim = "<PrimName>"` in the root layer header.',
        )]
    prim_site = _prim_site(stage, str(default_prim.GetPath()))
    asset_info = default_prim.GetMetadata("assetInfo") or {}
    if "identifier" not in asset_info:
        errors.append(_error(
            MISSING_ASSET_IDENTIFIER, ErrorType.Warn, prim_site,
            "defaultPrim is missing assetInfo:identifier. "
            "A unique, stable identifier (URI or canonical name) is required per REP §1.2.5.",
            'Add `string assetInfo:identifier = "<uri-or-name>"` '
            "to the defaultPrim's assetInfo dictionary.",
        ))
    if "version" not in asset_info:
        errors.append(_error(
            MISSING_ASSET_VERSION, ErrorType.Warn, prim_site,
            "defaultPrim is missing assetInfo:version. "
            "A version string (e.g. '1.0.0') is required per REP §1.2.5.",
            'Add `string assetInfo:version = "1.0.0"` '
            "to the defaultPrim's assetInfo dictionary.",
        ))
    return errors


register_stage_validator(
    "AssetManagement", _check_asset_management,
    doc="REP §1.2.5: defaultPrim and assetInfo requirements.", section="1.2",
)


def _check_path_convention(
    stage: Usd.Stage, timeRange: TimeRange,
) -> list[ValidationError]:
    errors: list[ValidationError] = []
    for layer in stage.GetUsedLayers():
        layer_id = layer.identifier
        for ref_path in layer.GetExternalReferences():
            if not ref_path:
                continue
            if ref_path.startswith("/") and not any(
                ref_path.startswith(s) for s in _FORBIDDEN_SCHEMES
            ):
                errors.append(_error(
                    ABSOLUTE_OR_PROPRIETARY_PATH, ErrorType.Error, _stage_site(stage),
                    f"Absolute path '{ref_path}' found in layer '{layer_id}'. "
                    "Internal references must use relative paths per REP §1.2.5.",
                    "Convert to a relative path (e.g. './geo/mesh.usdc').",
                ))
            for scheme in _FORBIDDEN_SCHEMES:
                if ref_path.lower().startswith(scheme):
                    errors.append(_error(
                        ABSOLUTE_OR_PROPRIETARY_PATH, ErrorType.Error, _stage_site(stage),
                        f"Proprietary URI scheme detected: '{ref_path}' in layer '{layer_id}'. "
                        "Absolute paths and proprietary schemes (e.g. omniverse://) "
                        "are strictly prohibited per REP §1.2.5.",
                        "Replace with a relative path or a package:// URI (ROS-specific layers only).",
                    ))
                    break
    for prim in stage.TraverseAll():
        for attr in prim.GetAuthoredAttributes():
            if not attr.GetMetadata("custom"):
                continue
            if attr.GetTypeName() != Sdf.ValueTypeNames.String:
                continue
            if any(hint in attr.GetName().lower() for hint in _PREFAB_ATTR_HINTS):
                errors.append(_error(
                    CUSTOM_COMPOSITION_ATTR, ErrorType.Error, _prim_site(stage, str(prim.GetPath())),
                    f"Custom string attribute '{attr.GetName()}' may be used for "
                    "dynamic composition or asset loading, which is prohibited per REP §1.2.5. "
                    "Asset composition must use native USD references or payloads only.",
                    "Remove the custom attribute and use a standard USD reference "
                    "or payload composition arc instead.",
                ))
    return errors


register_stage_validator(
    "PathConvention", _check_path_convention,
    doc="REP §1.2.5: internal references must use relative paths; no proprietary schemes.",
    section="1.2",
)


def _check_composition_model(
    stage: Usd.Stage, timeRange: TimeRange,
) -> list[ValidationError]:
    errors: list[ValidationError] = []
    for prim in stage.TraverseAll():
        if Usd.ModelAPI(prim).GetKind() != Kind.Tokens.component:
            continue
        for descendant in Usd.PrimRange(prim):
            if descendant == prim:
                continue
            if Usd.ModelAPI(descendant).GetKind() == Kind.Tokens.component:
                errors.append(_error(
                    NESTED_COMPONENT, ErrorType.Warn, _prim_site(stage, str(descendant.GetPath())),
                    f"Prim '{descendant.GetPath()}' has kind='component' but is a "
                    f"descendant of '{prim.GetPath()}' which is also kind='component'. "
                    "A component must not contain another component per REP §1.2.2.",
                    "Use kind='subcomponent' for nested organisational prims inside a component.",
                ))
    return errors


register_stage_validator(
    "CompositionModel", _check_composition_model,
    doc="REP §1.2.2: kind hierarchy – component must not contain component.", section="1.2",
)


def _check_variant_default(
    stage: Usd.Stage, timeRange: TimeRange,
) -> list[ValidationError]:
    errors: list[ValidationError] = []
    for prim in stage.TraverseAll():
        for vs_name in prim.GetVariantSets().GetNames():
            if not prim.GetVariantSets().GetVariantSet(vs_name).GetVariantSelection():
                errors.append(_error(
                    VARIANT_NO_DEFAULT, ErrorType.Warn, _prim_site(stage, str(prim.GetPath())),
                    f"VariantSet '{vs_name}' on prim '{prim.GetPath()}' has no default "
                    "variant selection. Loading this asset without explicit overrides "
                    "will resolve to an indeterminate state.",
                    f'Author a default selection: `variantSets.{vs_name} = "<default-variant>"`.',
                ))
    return errors


register_stage_validator(
    "VariantDefault", _check_variant_default,
    doc="REP §1.2.4: every VariantSet must author a default variant selection.", section="1.2",
)


def _check_inherits_specializes(
    stage: Usd.Stage, timeRange: TimeRange,
) -> list[ValidationError]:
    errors: list[ValidationError] = []
    for prim in stage.TraverseAll():
        pp = str(prim.GetPath())
        for path in prim.GetInherits().GetAllDirectInherits():
            errors.append(_error(
                INHERITS_SPECIALIZES_ARC, ErrorType.Warn, _prim_site(stage, pp),
                f"Prim '{pp}' uses an Inherits arc targeting '{path}'. "
                "Verify that the target class is defined within this asset's own layer "
                "stack; external class dependencies break portability (REP §1.2.3).",
                "If the class is external, define it within the asset or "
                "replace the inherit with a Reference/Payload.",
            ))
        seen: set[str] = set()
        for prim_spec in prim.GetPrimStack():
            for path in prim_spec.specializesList.GetAddedOrExplicitItems():
                path_str = str(path)
                if path_str in seen:
                    continue
                seen.add(path_str)
                errors.append(_error(
                    INHERITS_SPECIALIZES_ARC, ErrorType.Warn, _prim_site(stage, pp),
                    f"Prim '{pp}' uses a Specializes arc targeting '{path}'. "
                    "Verify that the target class is defined within this asset's own layer "
                    "stack; external class dependencies break portability (REP §1.2.3).",
                    "If the class is external, define it within the asset or "
                    "replace the specializes arc with a Reference/Payload.",
                ))
    return errors


register_stage_validator(
    "InheritsSpecializes", _check_inherits_specializes,
    doc="REP §1.2.3: flag Inherits/Specializes arcs for manual verification.", section="1.2",
)


def _check_payload_kinematic_topology(
    stage: Usd.Stage, timeRange: TimeRange,
) -> list[ValidationError]:
    errors: list[ValidationError] = []
    payload_roots: list[Usd.Prim] = []
    for prim in stage.TraverseAll():
        for prim_spec in prim.GetPrimStack():
            if prim_spec.payloadList.GetAddedOrExplicitItems():
                payload_roots.append(prim)
                break
    for root in payload_roots:
        applied = _applied(root)
        forbidden = applied & _PHYSICS_ROS_APIS
        if root.GetTypeName() in _KINEMATIC_JOINT_TYPES or forbidden:
            errors.append(_error(
                PAYLOAD_GATES_KINEMATIC, ErrorType.Error, _prim_site(stage, str(root.GetPath())),
                f"Payload root '{root.GetPath()}' carries kinematic/ROS schemas "
                f"(type={root.GetTypeName()}, apis={sorted(forbidden)}). "
                "Payloads must not gate joints, rigid bodies, or Ros*API topology per REP §1.2.3.",
                "Keep kinematic and ROS interface prims in the always-loaded graph; "
                "payload only nested visual/material heavy data.",
            ))
    return errors


register_stage_validator(
    "PayloadKinematicTopology", _check_payload_kinematic_topology,
    doc="REP §1.2.3: payloads must not gate kinematic/ROS topology prims.", section="1.2",
)


def _layer_has_api_schemas(prim_spec: Sdf.PrimSpec) -> bool:
    list_op = prim_spec.GetInfo("apiSchemas")
    if list_op and list_op.GetAppliedItems():
        return True
    return any(_layer_has_api_schemas(c) for c in prim_spec.nameChildren)


def _layer_has_heavy_geometry(prim_spec: Sdf.PrimSpec) -> bool:
    for attr_name in ("points", "faceVertexCounts"):
        attr = prim_spec.properties.get(attr_name)
        if attr is not None and hasattr(attr, "default") and attr.default is not None:
            if hasattr(attr.default, "__len__") and len(attr.default) > 50:
                return True
    return any(_layer_has_heavy_geometry(c) for c in prim_spec.nameChildren)


def _check_layer_encoding(
    stage: Usd.Stage, timeRange: TimeRange,
) -> list[ValidationError]:
    errors: list[ValidationError] = []
    for layer in stage.GetUsedLayers():
        identifier = layer.identifier
        if identifier.startswith("anon:"):
            continue
        if "." not in identifier.rsplit("/", 1)[-1]:
            continue
        ext = identifier.rsplit(".", 1)[-1].lower()
        if ext not in ("usda", "usdc", "usd"):
            continue
        has_schemas = _layer_has_api_schemas(layer.pseudoRoot)
        has_heavy = _layer_has_heavy_geometry(layer.pseudoRoot)
        if has_schemas and ext == "usdc":
            errors.append(_error(
                SCHEMA_LAYER_BINARY, ErrorType.Warn, _stage_site(stage),
                f"Layer '{identifier}' contains API schemas but uses binary .usdc encoding. "
                "Schema- and relationship-bearing layers must be authored as ASCII (.usda) "
                "per REP §1.2.1.",
                "Rename or re-export the layer as .usda (ASCII).",
            ))
        if has_heavy and ext == "usda":
            errors.append(_error(
                HEAVY_LAYER_ASCII, ErrorType.Warn, _stage_site(stage),
                f"Layer '{identifier}' contains heavy geometry data but uses ASCII .usda encoding. "
                "Heavy-data layers should use binary Crate encoding (.usdc) for performance "
                "per REP §1.2.1.",
                "Re-export the geometry layer as .usdc (binary Crate).",
            ))
    return errors


register_stage_validator(
    "LayerEncoding", _check_layer_encoding,
    doc="REP §1.2.1: schema/relationship-bearing layers must use .usda; heavy-data layers must use .usdc.",
    section="1.2",
)


def _contains_forbidden_semantics(root: Usd.Prim) -> bool:
    for prim in Usd.PrimRange(root):
        if prim.GetTypeName() in _KINEMATIC_JOINT_TYPES:
            return True
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            return True
        if _applied(prim) & _PHYSICS_ROS_APIS:
            return True
    return False


def _check_parallel_simulation_instancing(
    stage: Usd.Stage, timeRange: TimeRange,
) -> list[ValidationError]:
    errors: list[ValidationError] = []
    for prim in stage.TraverseAll():
        if prim.GetTypeName() != "PointInstancer":
            continue
        rel = prim.GetRelationship("prototypes")
        if not rel.IsValid():
            continue
        for target in rel.GetTargets():
            proto = stage.GetPrimAtPath(target)
            if not (proto and proto.IsValid()):
                continue
            if _contains_forbidden_semantics(proto):
                errors.append(_error(
                    POINT_INSTANCER_PHYSICS, ErrorType.Error, _prim_site(stage, str(prim.GetPath())),
                    f"PointInstancer '{prim.GetPath()}' references prototype '{target}' "
                    "that contains physics/ROS semantics. REP §1.2.6 forbids using USD "
                    "instancing to clone articulated physics assets for massive arrays.",
                    "Use PointInstancer only for atomic leaf geometry; delegate parallel "
                    "environment replication to simulator runtime APIs.",
                ))
    return errors


register_stage_validator(
    "ParallelSimulationInstancing", _check_parallel_simulation_instancing,
    doc="REP §1.2.6: point instancing must not replicate physics-enabled assets.",
    section="1.2",
)
