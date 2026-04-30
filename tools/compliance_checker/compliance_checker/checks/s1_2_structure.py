"""Section 1.2 – Asset Structure & Composition."""

from __future__ import annotations

from typing import TYPE_CHECKING

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

if TYPE_CHECKING:
    pass

# Forbidden URI schemes for asset paths (§1.2.5)
_FORBIDDEN_SCHEMES = ("omniverse://",)

# Heuristic: attribute names that suggest dynamic composition via strings (§1.2.5 / 1.2.9)
_PREFAB_ATTR_HINTS = ("prefabpath", "assetref", "assetpath", "spawnpath", "modelpath")
_KINEMATIC_JOINT_TYPES = {
    "PhysicsRevoluteJoint",
    "PhysicsPrismaticJoint",
    "PhysicsFixedJoint",
    "PhysicsSphericalJoint",
    "PhysicsDistanceJoint",
    "PhysicsJoint",
}


def _applied(prim: Usd.Prim) -> set[str]:
    list_op = prim.GetMetadata("apiSchemas")
    if list_op is None:
        return set()
    return set(list_op.GetAppliedItems())


# ------------------------------------------------------------------ #
# AssetManagement                                                       #
# ------------------------------------------------------------------ #


def _check_asset_management(
    stage: Usd.Stage, timeRange: TimeRange
) -> list[ValidationError]:
    errors: list[ValidationError] = []
    default_prim = stage.GetDefaultPrim()
    if not (default_prim and default_prim.IsValid()):
        errors.append(
            _error(
                "1.2.1",
                ErrorType.Warn,
                _stage_site(stage),
                "defaultPrim metadata is not set on the root layer. "
                "Referencing this asset without an explicit prim path is undefined behaviour "
                "and the Payload pattern breaks silently.",
                suggestion=(
                    "Set stage.SetDefaultPrim(prim) or author "
                    '`defaultPrim = "<PrimName>"` in the root layer header.'
                ),
            )
        )
        return errors

    asset_info = default_prim.GetMetadata("assetInfo") or {}

    if "identifier" not in asset_info:
        errors.append(
            _error(
                "1.2.2",
                ErrorType.Warn,
                _prim_site(stage, str(default_prim.GetPath())),
                "defaultPrim is missing assetInfo:identifier. "
                "A unique, stable identifier (URI or canonical name) is required per REP §1.2.5.",
                suggestion=(
                    'Add `string assetInfo:identifier = "<uri-or-name>"` '
                    "to the defaultPrim's assetInfo dictionary."
                ),
            )
        )

    if "version" not in asset_info:
        errors.append(
            _error(
                "1.2.3",
                ErrorType.Warn,
                _prim_site(stage, str(default_prim.GetPath())),
                "defaultPrim is missing assetInfo:version. "
                "A version string (e.g. '1.0.0') is required per REP §1.2.5.",
                suggestion=(
                    'Add `string assetInfo:version = "1.0.0"` '
                    "to the defaultPrim's assetInfo dictionary."
                ),
            )
        )

    return errors


register_stage_validator(
    "AssetManagement",
    _check_asset_management,
    doc="REP §1.2.5: defaultPrim and assetInfo requirements.",
    section="1.2",
)


# ------------------------------------------------------------------ #
# PathConvention                                                        #
# ------------------------------------------------------------------ #


def _check_path_convention(
    stage: Usd.Stage, timeRange: TimeRange
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
                errors.append(
                    _error(
                        "1.2.4",
                        ErrorType.Error,
                        _stage_site(stage),
                        f"Absolute path '{ref_path}' found in layer '{layer_id}'. "
                        "Internal references must use relative paths per REP §1.2.5.",
                        suggestion="Convert to a relative path (e.g. './geo/mesh.usdc').",
                    )
                )

            for scheme in _FORBIDDEN_SCHEMES:
                if ref_path.lower().startswith(scheme):
                    errors.append(
                        _error(
                            "1.2.4",
                            ErrorType.Error,
                            _stage_site(stage),
                            f"Proprietary URI scheme detected: '{ref_path}' "
                            f"in layer '{layer_id}'. "
                            "Absolute paths and proprietary schemes (e.g. omniverse://) "
                            "are strictly prohibited per REP §1.2.5.",
                            suggestion=(
                                "Replace with a relative path or a package:// URI "
                                "(ROS-specific layers only)."
                            ),
                        )
                    )
                    break

    for prim in stage.TraverseAll():
        for attr in prim.GetAuthoredAttributes():
            if not attr.GetMetadata("custom"):
                continue
            if attr.GetTypeName() != Sdf.ValueTypeNames.String:
                continue
            name_lower = attr.GetName().lower()
            if any(hint in name_lower for hint in _PREFAB_ATTR_HINTS):
                errors.append(
                    _error(
                        "1.2.9",
                        ErrorType.Error,
                        _prim_site(stage, str(prim.GetPath())),
                        f"Custom string attribute '{attr.GetName()}' may be used for "
                        "dynamic composition or asset loading, which is prohibited per REP §1.2.5. "
                        "Asset composition must use native USD references or payloads only.",
                        suggestion=(
                            "Remove the custom attribute and use a standard USD reference "
                            "or payload composition arc instead."
                        ),
                    )
                )

    return errors


register_stage_validator(
    "PathConvention",
    _check_path_convention,
    doc="REP §1.2.5: internal references must use relative paths; no proprietary schemes.",
    section="1.2",
)


# ------------------------------------------------------------------ #
# CompositionModel                                                      #
# ------------------------------------------------------------------ #


def _check_composition_model(
    stage: Usd.Stage, timeRange: TimeRange
) -> list[ValidationError]:
    errors: list[ValidationError] = []

    for prim in stage.TraverseAll():
        model = Usd.ModelAPI(prim)
        kind = model.GetKind()
        if kind != Kind.Tokens.component:
            continue
        for descendant in Usd.PrimRange(prim):
            if descendant == prim:
                continue
            desc_kind = Usd.ModelAPI(descendant).GetKind()
            if desc_kind == Kind.Tokens.component:
                errors.append(
                    _error(
                        "1.2.6",
                        ErrorType.Warn,
                        _prim_site(stage, str(descendant.GetPath())),
                        f"Prim '{descendant.GetPath()}' has kind='component' but is a "
                        f"descendant of '{prim.GetPath()}' which is also kind='component'. "
                        "A component must not contain another component per REP §1.2.2.",
                        suggestion=(
                            "Use kind='subcomponent' for nested organisational prims "
                            "inside a component."
                        ),
                    )
                )

    return errors


register_stage_validator(
    "CompositionModel",
    _check_composition_model,
    doc="REP §1.2.2: kind hierarchy – component must not contain component.",
    section="1.2",
)


# ------------------------------------------------------------------ #
# VariantDefault                                                        #
# ------------------------------------------------------------------ #


def _check_variant_default(
    stage: Usd.Stage, timeRange: TimeRange
) -> list[ValidationError]:
    errors: list[ValidationError] = []

    for prim in stage.TraverseAll():
        vs_sets = prim.GetVariantSets()
        for vs_name in vs_sets.GetNames():
            vs = vs_sets.GetVariantSet(vs_name)
            selection = vs.GetVariantSelection()
            if not selection:
                errors.append(
                    _error(
                        "1.2.7",
                        ErrorType.Warn,
                        _prim_site(stage, str(prim.GetPath())),
                        f"VariantSet '{vs_name}' on prim '{prim.GetPath()}' has no default "
                        "variant selection. Loading this asset without explicit overrides "
                        "will resolve to an indeterminate state.",
                        suggestion=(
                            f"Author a default selection: "
                            f'`variantSets.{vs_name} = "<default-variant>"`.'
                        ),
                    )
                )

    return errors


register_stage_validator(
    "VariantDefault",
    _check_variant_default,
    doc="REP §1.2.4: every VariantSet must author a default variant selection.",
    section="1.2",
)


# ------------------------------------------------------------------ #
# InheritsSpecializes                                                   #
# ------------------------------------------------------------------ #


def _check_inherits_specializes(
    stage: Usd.Stage, timeRange: TimeRange
) -> list[ValidationError]:
    errors: list[ValidationError] = []

    for prim in stage.TraverseAll():
        inherits = prim.GetInherits().GetAllDirectInherits()
        for path in inherits:
            errors.append(
                _error(
                    "1.2.8",
                    ErrorType.Warn,
                    _prim_site(stage, str(prim.GetPath())),
                    f"Prim '{prim.GetPath()}' uses an Inherits arc targeting '{path}'. "
                    "Verify that the target class is defined within this asset's own layer "
                    "stack; external class dependencies break portability (REP §1.2.3).",
                    suggestion=(
                        "If the class is external, define it within the asset or "
                        "replace the inherit with a Reference/Payload."
                    ),
                )
            )

        seen_spec_paths: set[str] = set()
        for prim_spec in prim.GetPrimStack():
            for path in prim_spec.specializesList.GetAddedOrExplicitItems():
                path_str = str(path)
                if path_str in seen_spec_paths:
                    continue
                seen_spec_paths.add(path_str)
                errors.append(
                    _error(
                        "1.2.8",
                        ErrorType.Warn,
                        _prim_site(stage, str(prim.GetPath())),
                        f"Prim '{prim.GetPath()}' uses a Specializes arc targeting '{path}'. "
                        "Verify that the target class is defined within this asset's own layer "
                        "stack; external class dependencies break portability (REP §1.2.3).",
                        suggestion=(
                            "If the class is external, define it within the asset or "
                            "replace the specializes arc with a Reference/Payload."
                        ),
                    )
                )

    return errors


register_stage_validator(
    "InheritsSpecializes",
    _check_inherits_specializes,
    doc="REP §1.2.3: flag Inherits/Specializes arcs for manual verification.",
    section="1.2",
)


# ------------------------------------------------------------------ #
# PayloadKinematicTopology                                              #
# ------------------------------------------------------------------ #

_PAYLOAD_FORBIDDEN_APIS = {
    "PhysicsRigidBodyAPI",
    "RosContextAPI",
    "RosTopicAPI",
    "RosServiceAPI",
    "RosActionAPI",
}


def _check_payload_kinematic_topology(
    stage: Usd.Stage, timeRange: TimeRange
) -> list[ValidationError]:
    errors: list[ValidationError] = []

    payload_roots: list[Usd.Prim] = []
    for prim in stage.TraverseAll():
        for prim_spec in prim.GetPrimStack():
            if prim_spec.payloadList.GetAddedOrExplicitItems():
                payload_roots.append(prim)
                break

    for root in payload_roots:
        applied_schemas = _applied(root)
        if root.GetTypeName() in _KINEMATIC_JOINT_TYPES or (
            applied_schemas & _PAYLOAD_FORBIDDEN_APIS
        ):
            errors.append(
                _error(
                    "1.2.10",
                    ErrorType.Error,
                    _prim_site(stage, str(root.GetPath())),
                    f"Payload root '{root.GetPath()}' carries kinematic/ROS schemas "
                    f"(type={root.GetTypeName()}, apis={sorted(applied_schemas & _PAYLOAD_FORBIDDEN_APIS)}). "
                    "Payloads must not gate joints, rigid bodies, or Ros*API topology per REP §1.2.3.",
                    suggestion=(
                        "Keep kinematic and ROS interface prims in the always-loaded graph; "
                        "payload only nested visual/material heavy data."
                    ),
                )
            )

    return errors


register_stage_validator(
    "PayloadKinematicTopology",
    _check_payload_kinematic_topology,
    doc="REP §1.2.3: payloads must not gate kinematic/ROS topology prims.",
    section="1.2",
)


# ------------------------------------------------------------------ #
# LayerEncoding                                                         #
# ------------------------------------------------------------------ #


def _layer_has_api_schemas(prim_spec: Sdf.PrimSpec) -> bool:
    list_op = prim_spec.GetInfo("apiSchemas")
    if list_op and list_op.GetAppliedItems():
        return True
    for child in prim_spec.nameChildren:
        if _layer_has_api_schemas(child):
            return True
    return False


def _layer_has_heavy_geometry(prim_spec: Sdf.PrimSpec) -> bool:
    for attr_name in ("points", "faceVertexCounts"):
        attr = prim_spec.properties.get(attr_name)
        if (
            attr is not None
            and hasattr(attr, "default")
            and attr.default is not None
        ):
            val = attr.default
            if hasattr(val, "__len__") and len(val) > 50:
                return True
    for child in prim_spec.nameChildren:
        if _layer_has_heavy_geometry(child):
            return True
    return False


def _check_layer_encoding(
    stage: Usd.Stage, timeRange: TimeRange
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
            errors.append(
                _error(
                    "1.2.12",
                    ErrorType.Warn,
                    _stage_site(stage),
                    f"Layer '{identifier}' contains API schemas but uses binary .usdc encoding. "
                    "Schema- and relationship-bearing layers must be authored as ASCII (.usda) "
                    "per REP §1.2.1.",
                    suggestion="Rename or re-export the layer as .usda (ASCII).",
                )
            )

        if has_heavy and ext == "usda":
            errors.append(
                _error(
                    "1.2.13",
                    ErrorType.Warn,
                    _stage_site(stage),
                    f"Layer '{identifier}' contains heavy geometry data but uses ASCII .usda encoding. "
                    "Heavy-data layers should use binary Crate encoding (.usdc) for performance "
                    "per REP §1.2.1.",
                    suggestion="Re-export the geometry layer as .usdc (binary Crate).",
                )
            )

    return errors


register_stage_validator(
    "LayerEncoding",
    _check_layer_encoding,
    doc="REP §1.2.1: schema/relationship-bearing layers must use .usda; heavy-data layers must use .usdc.",
    section="1.2",
)


# ------------------------------------------------------------------ #
# ParallelSimulationInstancing                                          #
# ------------------------------------------------------------------ #

_INSTANCING_FORBIDDEN_APIS = {
    "PhysicsRigidBodyAPI",
    "PhysicsArticulationRootAPI",
    "RosContextAPI",
    "RosTopicAPI",
    "RosServiceAPI",
    "RosActionAPI",
}


def _contains_forbidden_semantics(root: Usd.Prim) -> bool:
    for prim in Usd.PrimRange(root):
        if prim.GetTypeName() in _KINEMATIC_JOINT_TYPES:
            return True
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            return True
        applied_schemas = _applied(prim)
        if applied_schemas & _INSTANCING_FORBIDDEN_APIS:
            return True
    return False


def _check_parallel_simulation_instancing(
    stage: Usd.Stage, timeRange: TimeRange
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
                errors.append(
                    _error(
                        "1.2.11",
                        ErrorType.Error,
                        _prim_site(stage, str(prim.GetPath())),
                        f"PointInstancer '{prim.GetPath()}' references prototype '{target}' "
                        "that contains physics/ROS semantics. REP §1.2.6 forbids using USD "
                        "instancing to clone articulated physics assets for massive arrays.",
                        suggestion=(
                            "Use PointInstancer only for atomic leaf geometry; delegate parallel "
                            "environment replication to simulator runtime APIs."
                        ),
                    )
                )

    return errors


register_stage_validator(
    "ParallelSimulationInstancing",
    _check_parallel_simulation_instancing,
    doc="REP §1.2.6: point instancing must not replicate physics-enabled assets.",
    section="1.2",
)
