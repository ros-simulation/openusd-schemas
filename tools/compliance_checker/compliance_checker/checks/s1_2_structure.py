"""Section 1.2 – Asset Structure & Composition."""

from __future__ import annotations

from typing import Iterator

from pxr import Kind, Sdf, Usd, UsdPhysics

from ..report import Severity, Violation
from .base import BaseCheck

_STAGE = "/"

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


class AssetManagementCheck(BaseCheck):
    """REP §1.2.5: defaultPrim and assetInfo requirements."""

    section = "1.2"

    def run(self, stage: Usd.Stage) -> Iterator[Violation]:
        yield from self._check_default_prim(stage)

    def _check_default_prim(self, stage: Usd.Stage) -> Iterator[Violation]:
        default_prim = stage.GetDefaultPrim()
        if not (default_prim and default_prim.IsValid()):
            yield Violation(
                check_id="1.2.1",
                severity=Severity.WARNING,
                prim_path=_STAGE,
                section=self.section,
                message=(
                    "defaultPrim metadata is not set on the root layer. "
                    "Referencing this asset without an explicit prim path is undefined behaviour "
                    "and the Payload pattern breaks silently."
                ),
                suggestion=(
                    "Set stage.SetDefaultPrim(prim) or author "
                    '`defaultPrim = "<PrimName>"` in the root layer header.'
                ),
            )
            return

        asset_info = default_prim.GetMetadata("assetInfo") or {}

        if "identifier" not in asset_info:
            yield Violation(
                check_id="1.2.2",
                severity=Severity.WARNING,
                prim_path=str(default_prim.GetPath()),
                section=self.section,
                message=(
                    "defaultPrim is missing assetInfo:identifier. "
                    "A unique, stable identifier (URI or canonical name) is required per REP §1.2.5."
                ),
                suggestion=(
                    'Add `string assetInfo:identifier = "<uri-or-name>"` '
                    "to the defaultPrim's assetInfo dictionary."
                ),
            )

        if "version" not in asset_info:
            yield Violation(
                check_id="1.2.3",
                severity=Severity.WARNING,
                prim_path=str(default_prim.GetPath()),
                section=self.section,
                message=(
                    "defaultPrim is missing assetInfo:version. "
                    "A version string (e.g. '1.0.0') is required per REP §1.2.5."
                ),
                suggestion=(
                    'Add `string assetInfo:version = "1.0.0"` '
                    "to the defaultPrim's assetInfo dictionary."
                ),
            )


class PathConventionCheck(BaseCheck):
    """REP §1.2.5: internal references must use relative paths; no proprietary schemes."""

    section = "1.2"

    def run(self, stage: Usd.Stage) -> Iterator[Violation]:
        yield from self._check_external_references(stage)
        yield from self._check_custom_composition_attributes(stage)

    def _check_external_references(self, stage: Usd.Stage) -> Iterator[Violation]:
        for layer in stage.GetUsedLayers():
            layer_id = layer.identifier
            for ref_path in layer.GetExternalReferences():
                if not ref_path:
                    continue

                # Absolute filesystem paths
                if ref_path.startswith("/") and not any(
                    ref_path.startswith(s) for s in _FORBIDDEN_SCHEMES
                ):
                    yield Violation(
                        check_id="1.2.4",
                        severity=Severity.ERROR,
                        prim_path=layer_id,
                        section=self.section,
                        message=(
                            f"Absolute path '{ref_path}' found in layer '{layer_id}'. "
                            "Internal references must use relative paths per REP §1.2.5."
                        ),
                        suggestion="Convert to a relative path (e.g. './geo/mesh.usdc').",
                    )

                # Forbidden URI schemes
                for scheme in _FORBIDDEN_SCHEMES:
                    if ref_path.lower().startswith(scheme):
                        yield Violation(
                            check_id="1.2.4",
                            severity=Severity.ERROR,
                            prim_path=layer_id,
                            section=self.section,
                            message=(
                                f"Proprietary URI scheme detected: '{ref_path}' "
                                f"in layer '{layer_id}'. "
                                "Absolute paths and proprietary schemes (e.g. omniverse://) "
                                "are strictly prohibited per REP §1.2.5."
                            ),
                            suggestion=(
                                "Replace with a relative path or a package:// URI "
                                "(ROS-specific layers only)."
                            ),
                        )
                        break

    def _check_custom_composition_attributes(
        self, stage: Usd.Stage
    ) -> Iterator[Violation]:
        """Heuristic: flag custom string attributes whose name suggests dynamic loading."""
        for prim in stage.TraverseAll():
            for attr in prim.GetAuthoredAttributes():
                if not attr.GetMetadata("custom"):
                    continue
                if attr.GetTypeName() != Sdf.ValueTypeNames.String:
                    continue
                name_lower = attr.GetName().lower()
                if any(hint in name_lower for hint in _PREFAB_ATTR_HINTS):
                    yield Violation(
                        check_id="1.2.9",
                        severity=Severity.ERROR,
                        prim_path=str(prim.GetPath()),
                        section=self.section,
                        message=(
                            f"Custom string attribute '{attr.GetName()}' may be used for "
                            "dynamic composition or asset loading, which is prohibited per REP §1.2.5. "
                            "Asset composition must use native USD references or payloads only."
                        ),
                        suggestion=(
                            "Remove the custom attribute and use a standard USD reference "
                            "or payload composition arc instead."
                        ),
                    )


class CompositionModelCheck(BaseCheck):
    """REP §1.2.2: kind hierarchy – component must not contain component."""

    section = "1.2"

    def run(self, stage: Usd.Stage) -> Iterator[Violation]:
        yield from self._check_component_nesting(stage)

    def _check_component_nesting(self, stage: Usd.Stage) -> Iterator[Violation]:
        for prim in stage.TraverseAll():
            model = Usd.ModelAPI(prim)
            kind = model.GetKind()
            if kind != Kind.Tokens.component:
                continue
            # Walk all descendants (recursive) for nested components
            for descendant in Usd.PrimRange(prim):
                if descendant == prim:
                    continue
                desc_kind = Usd.ModelAPI(descendant).GetKind()
                if desc_kind == Kind.Tokens.component:
                    yield Violation(
                        check_id="1.2.6",
                        severity=Severity.WARNING,
                        prim_path=str(descendant.GetPath()),
                        section=self.section,
                        message=(
                            f"Prim '{descendant.GetPath()}' has kind='component' but is a "
                            f"descendant of '{prim.GetPath()}' which is also kind='component'. "
                            "A component must not contain another component per REP §1.2.2."
                        ),
                        suggestion=(
                            "Use kind='subcomponent' for nested organisational prims "
                            "inside a component."
                        ),
                    )


class VariantDefaultCheck(BaseCheck):
    """REP §1.2.4: every VariantSet must author a default variant selection."""

    section = "1.2"

    def run(self, stage: Usd.Stage) -> Iterator[Violation]:
        for prim in stage.TraverseAll():
            vs_sets = prim.GetVariantSets()
            for vs_name in vs_sets.GetNames():
                vs = vs_sets.GetVariantSet(vs_name)
                selection = vs.GetVariantSelection()
                if not selection:
                    yield Violation(
                        check_id="1.2.7",
                        severity=Severity.WARNING,
                        prim_path=str(prim.GetPath()),
                        section=self.section,
                        message=(
                            f"VariantSet '{vs_name}' on prim '{prim.GetPath()}' has no default "
                            "variant selection. Loading this asset without explicit overrides "
                            "will resolve to an indeterminate state."
                        ),
                        suggestion=(
                            f"Author a default selection: "
                            f'`variantSets.{vs_name} = "<default-variant>"`.'
                        ),
                    )


class InheritsSpecializesCheck(BaseCheck):
    """REP §1.2.3: flag Inherits/Specializes arcs for manual verification."""

    section = "1.2"

    def run(self, stage: Usd.Stage) -> Iterator[Violation]:
        for prim in stage.TraverseAll():
            inherits = prim.GetInherits().GetAllDirectInherits()
            for path in inherits:
                yield Violation(
                    check_id="1.2.8",
                    severity=Severity.WARNING,
                    prim_path=str(prim.GetPath()),
                    section=self.section,
                    message=(
                        f"Prim '{prim.GetPath()}' uses an Inherits arc targeting '{path}'. "
                        "Verify that the target class is defined within this asset's own layer "
                        "stack; external class dependencies break portability (REP §1.2.3)."
                    ),
                    suggestion=(
                        "If the class is external, define it within the asset or "
                        "replace the inherit with a Reference/Payload."
                    ),
                )

            # UsdSpecializes has no GetAllDirectSpecializes(); read via the prim stack instead.
            seen_spec_paths: set[str] = set()
            for prim_spec in prim.GetPrimStack():
                for path in prim_spec.specializesList.GetAddedOrExplicitItems():
                    path_str = str(path)
                    if path_str in seen_spec_paths:
                        continue
                    seen_spec_paths.add(path_str)
                    yield Violation(
                        check_id="1.2.8",
                        severity=Severity.WARNING,
                        prim_path=str(prim.GetPath()),
                        section=self.section,
                        message=(
                            f"Prim '{prim.GetPath()}' uses a Specializes arc targeting '{path}'. "
                            "Verify that the target class is defined within this asset's own layer "
                            "stack; external class dependencies break portability (REP §1.2.3)."
                        ),
                        suggestion=(
                            "If the class is external, define it within the asset or "
                            "replace the specializes arc with a Reference/Payload."
                        ),
                    )


class PayloadKinematicTopologyCheck(BaseCheck):
    """REP §1.2.3: payloads must not gate kinematic/ROS topology prims."""

    section = "1.2"

    _FORBIDDEN_APIS = {
        "PhysicsRigidBodyAPI",
        "RosContextAPI",
        "RosTopicAPI",
        "RosServiceAPI",
        "RosActionAPI",
    }

    def run(self, stage: Usd.Stage) -> Iterator[Violation]:
        payload_roots: list[Usd.Prim] = []
        for prim in stage.TraverseAll():
            for prim_spec in prim.GetPrimStack():
                if prim_spec.payloadList.GetAddedOrExplicitItems():
                    payload_roots.append(prim)
                    break

        for root in payload_roots:
            applied = self._applied(root)
            if root.GetTypeName() in _KINEMATIC_JOINT_TYPES or (
                applied & self._FORBIDDEN_APIS
            ):
                yield Violation(
                    check_id="1.2.10",
                    severity=Severity.ERROR,
                    prim_path=str(root.GetPath()),
                    section=self.section,
                    message=(
                        f"Payload root '{root.GetPath()}' carries kinematic/ROS schemas "
                        f"(type={root.GetTypeName()}, apis={sorted(applied & self._FORBIDDEN_APIS)}). "
                        "Payloads must not gate joints, rigid bodies, or Ros*API topology per REP §1.2.3."
                    ),
                    suggestion=(
                        "Keep kinematic and ROS interface prims in the always-loaded graph; "
                        "payload only nested visual/material heavy data."
                    ),
                )

    def _applied(self, prim: Usd.Prim) -> set[str]:
        list_op = prim.GetMetadata("apiSchemas")
        if list_op is None:
            return set()
        return set(list_op.GetAppliedItems())


class LayerEncodingCheck(BaseCheck):
    """REP §1.2.1: schema/relationship-bearing layers must use .usda; heavy-data layers must use .usdc."""

    section = "1.2"

    def run(self, stage: Usd.Stage) -> Iterator[Violation]:
        for layer in stage.GetUsedLayers():
            identifier = layer.identifier
            if identifier.startswith("anon:"):
                continue
            if "." not in identifier.rsplit("/", 1)[-1]:
                continue
            ext = identifier.rsplit(".", 1)[-1].lower()
            if ext not in ("usda", "usdc", "usd"):
                continue

            has_schemas = self._layer_has_api_schemas(layer)
            has_heavy = self._layer_has_heavy_geometry(layer)

            if has_schemas and ext == "usdc":
                yield Violation(
                    check_id="1.2.12",
                    severity=Severity.WARNING,
                    prim_path=identifier,
                    section=self.section,
                    message=(
                        f"Layer '{identifier}' contains API schemas but uses binary .usdc encoding. "
                        "Schema- and relationship-bearing layers must be authored as ASCII (.usda) "
                        "per REP §1.2.1."
                    ),
                    suggestion="Rename or re-export the layer as .usda (ASCII).",
                )

            if has_heavy and ext == "usda":
                yield Violation(
                    check_id="1.2.13",
                    severity=Severity.WARNING,
                    prim_path=identifier,
                    section=self.section,
                    message=(
                        f"Layer '{identifier}' contains heavy geometry data but uses ASCII .usda encoding. "
                        "Heavy-data layers should use binary Crate encoding (.usdc) for performance "
                        "per REP §1.2.1."
                    ),
                    suggestion="Re-export the geometry layer as .usdc (binary Crate).",
                )

    def _layer_has_api_schemas(self, layer: Sdf.Layer) -> bool:
        return self._traverse_for_schemas(layer.pseudoRoot)

    def _traverse_for_schemas(self, prim_spec: Sdf.PrimSpec) -> bool:
        list_op = prim_spec.GetInfo("apiSchemas")
        if list_op and list_op.GetAppliedItems():
            return True
        for child in prim_spec.nameChildren:
            if self._traverse_for_schemas(child):
                return True
        return False

    def _layer_has_heavy_geometry(self, layer: Sdf.Layer) -> bool:
        return self._traverse_for_geometry(layer.pseudoRoot)

    def _traverse_for_geometry(self, prim_spec: Sdf.PrimSpec) -> bool:
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
            if self._traverse_for_geometry(child):
                return True
        return False


class ParallelSimulationInstancingCheck(BaseCheck):
    """REP §1.2.6: point instancing must not replicate physics-enabled assets."""

    section = "1.2"

    _FORBIDDEN_APIS = {
        "PhysicsRigidBodyAPI",
        "PhysicsArticulationRootAPI",
        "RosContextAPI",
        "RosTopicAPI",
        "RosServiceAPI",
        "RosActionAPI",
    }

    def run(self, stage: Usd.Stage) -> Iterator[Violation]:
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
                if self._contains_forbidden_semantics(proto):
                    yield Violation(
                        check_id="1.2.11",
                        severity=Severity.ERROR,
                        prim_path=str(prim.GetPath()),
                        section=self.section,
                        message=(
                            f"PointInstancer '{prim.GetPath()}' references prototype '{target}' "
                            "that contains physics/ROS semantics. REP §1.2.6 forbids using USD "
                            "instancing to clone articulated physics assets for massive arrays."
                        ),
                        suggestion=(
                            "Use PointInstancer only for atomic leaf geometry; delegate parallel "
                            "environment replication to simulator runtime APIs."
                        ),
                    )

    def _contains_forbidden_semantics(self, root: Usd.Prim) -> bool:
        for prim in Usd.PrimRange(root):
            if prim.GetTypeName() in _KINEMATIC_JOINT_TYPES:
                return True
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                return True
            if self._applied(prim) & self._FORBIDDEN_APIS:
                return True
        return False

    def _applied(self, prim: Usd.Prim) -> set[str]:
        list_op = prim.GetMetadata("apiSchemas")
        if list_op is None:
            return set()
        return set(list_op.GetAppliedItems())
