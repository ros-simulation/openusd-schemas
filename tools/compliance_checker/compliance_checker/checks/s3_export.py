"""Section 3 – Export & Conversion checks (opt-in, potentially slow)."""

from __future__ import annotations

import os
import re

from pxr import Usd, UsdGeom, UsdShade

from .base import (
    ErrorType,
    TimeRange,
    ValidationError,
    _error,
    _prim_site,
    register_stage_validator,
)

# ------------------------------------------------------------------ #
# Constants                                                             #
# ------------------------------------------------------------------ #

_UDIM_RE = re.compile(r"\.\d{4}\b|<UDIM>|<%d>|UDIM", re.IGNORECASE)

_FORBIDDEN_EXTENSIONS = {".exr", ".tiff", ".tif", ".hdr"}
_DATA_MAP_JPEG_EXTENSIONS = {".jpg", ".jpeg"}
_DATA_MAP_HINTS = ("normal", "metallic", "roughness", "orm", "occlusion")

_PROCEDURAL_NODE_HINTS = ("noise", "math", "mix", "blend", "remap", "fractal")


def _applied(prim: Usd.Prim) -> set[str]:
    list_op = prim.GetMetadata("apiSchemas")
    if list_op is None:
        return set()
    return set(list_op.GetAppliedItems())


# ------------------------------------------------------------------ #
# Validators                                                            #
# ------------------------------------------------------------------ #


def _validate_material_portability(
    stage: Usd.Stage, timeRange: TimeRange
) -> list[ValidationError]:
    errors: list[ValidationError] = []
    for prim in stage.TraverseAll():
        if prim.GetTypeName() != "Material":
            continue
        mat = UsdShade.Material(prim)
        if not mat:
            continue
        prim_path = str(prim.GetPath())

        if not mat.GetSurfaceOutput():
            errors.append(_error(
                "3.1.1",
                ErrorType.Warn,
                _prim_site(stage, prim_path),
                (
                    f"Material '{prim_path}' has no connected 'outputs:surface' terminal. "
                    "A UsdPreviewSurface wired to the universal surface output is required "
                    "for glTF conversion per REP §3.1."
                ),
                suggestion=(
                    "Wire a UsdPreviewSurface shader to 'outputs:surface' inside the material. "
                    "Proprietary shaders (MDL, OSL) should use renderer-specific terminals only."
                ),
            ))
            continue

        sources, _ = mat.GetSurfaceOutput().GetConnectedSources()
        if not sources:
            errors.append(_error(
                "3.1.1",
                ErrorType.Warn,
                _prim_site(stage, prim_path),
                (
                    f"Material '{prim_path}' 'outputs:surface' terminal has no connected shader. "
                    "A UsdPreviewSurface must be wired to the universal surface output per REP §3.1."
                ),
                suggestion=(
                    "Wire a UsdPreviewSurface shader to 'outputs:surface' inside the material."
                ),
            ))
            continue

        for src in sources:
            shader_prim = src.source.GetPrim()
            shader = UsdShade.Shader(shader_prim)
            if shader:
                shader_id = shader.GetShaderId()
                if shader_id and "UsdPreviewSurface" not in shader_id:
                    errors.append(_error(
                        "3.1.1",
                        ErrorType.Warn,
                        _prim_site(stage, prim_path),
                        (
                            f"Material '{prim_path}' surface output is connected to a "
                            f"'{shader_id}' shader, not UsdPreviewSurface. "
                            "Proprietary shaders must not replace the universal surface output "
                            "per REP §3.1."
                        ),
                        suggestion=(
                            "Replace or supplement with a UsdPreviewSurface shader wired to "
                            "'outputs:surface'. Keep the proprietary shader on a "
                            "renderer-specific terminal (e.g. 'outputs:mdl:surface')."
                        ),
                    ))
    return errors


def _is_dome_light_texture(shader_prim: Usd.Prim) -> bool:
    parent = shader_prim.GetParent()
    while parent and parent.IsValid():
        if parent.GetTypeName() == "DomeLight":
            return True
        parent = parent.GetParent()
    return False


def _validate_texture_format(
    stage: Usd.Stage, timeRange: TimeRange
) -> list[ValidationError]:
    errors: list[ValidationError] = []
    for prim in stage.TraverseAll():
        if prim.GetTypeName() != "Shader":
            continue
        shader = UsdShade.Shader(prim)
        if not shader:
            continue
        shader_id = shader.GetShaderId()
        if not shader_id or "UsdUVTexture" not in shader_id:
            continue

        prim_path = str(prim.GetPath())
        file_input = shader.GetInput("file")
        if not file_input:
            continue
        asset_path = file_input.Get()
        if not asset_path:
            continue

        path_str = (
            str(asset_path.path) if hasattr(asset_path, "path") else str(asset_path)
        )
        if not path_str:
            continue

        if _UDIM_RE.search(path_str):
            errors.append(_error(
                "3.1.2",
                ErrorType.Error,
                _prim_site(stage, prim_path),
                (
                    f"Texture path '{path_str}' on '{prim_path}' contains a UDIM tile pattern. "
                    "Multi-tile UV mapping (UDIMs) is unsupported by glTF 2.0 and many real-time "
                    "engines; must not be used per REP §3.1."
                ),
                suggestion=(
                    "Pack all UVs into the [0,1] space using texture atlasing. "
                    "If multiple textures are required, split geometry with UsdGeomSubset "
                    "and assign separate materials."
                ),
            ))

        _, ext = os.path.splitext(path_str.lower())
        if ext in _FORBIDDEN_EXTENSIONS:
            is_dome_light = _is_dome_light_texture(prim)
            if ext == ".hdr" and is_dome_light:
                continue
            errors.append(_error(
                "3.2.1",
                ErrorType.Error,
                _prim_site(stage, prim_path),
                (
                    f"Texture '{path_str}' on '{prim_path}' uses forbidden format '{ext}'. "
                    "EXR, TIFF, and other HDR/DCC formats must not be used for surface maps "
                    "in distributed assets per REP §3.2. "
                    "(HDR is only permitted for UsdLuxDomeLight environment maps.)"
                ),
                suggestion=(
                    "Convert to PNG (for data maps: normal, metallic, roughness, ORM) or "
                    "JPEG (for color maps without alpha)."
                ),
            ))
        if ext in _DATA_MAP_JPEG_EXTENSIONS and any(
            hint in path_str.lower() for hint in _DATA_MAP_HINTS
        ):
            errors.append(_error(
                "3.2.2",
                ErrorType.Error,
                _prim_site(stage, prim_path),
                (
                    f"Data map texture '{path_str}' on '{prim_path}' uses JPEG. "
                    "Normal/metallic/roughness/ORM maps must use lossless PNG per REP §3.2."
                ),
                suggestion="Convert the data map to PNG.",
            ))
    return errors


def _is_collision_geometry(prim: Usd.Prim) -> bool:
    purpose_attr = prim.GetAttribute("purpose")
    if purpose_attr.IsValid():
        purpose = purpose_attr.Get()
        if purpose == "guide":
            return True
    parent = prim.GetParent()
    if parent and parent.IsValid():
        parent_name = parent.GetName().lower()
        if "collision" in parent_name or "collider" in parent_name:
            return True
    return False


def _validate_geometry_constraints(
    stage: Usd.Stage, timeRange: TimeRange
) -> list[ValidationError]:
    errors: list[ValidationError] = []
    for prim in stage.TraverseAll():
        if prim.GetTypeName() != "Mesh":
            continue
        mesh = UsdGeom.Mesh(prim)
        if not mesh:
            continue
        prim_path = str(prim.GetPath())

        # Orientation check
        orientation_attr = mesh.GetOrientationAttr()
        if orientation_attr.IsValid():
            orientation = orientation_attr.Get()
            if orientation is not None and str(orientation) == "leftHanded":
                errors.append(_error(
                    "3.4.2",
                    ErrorType.Warn,
                    _prim_site(stage, prim_path),
                    (
                        f"Mesh '{prim_path}' uses orientation='leftHanded'. "
                        "Meshes must use 'rightHanded' (CCW winding) to align with glTF 2.0 "
                        "per REP §3.4."
                    ),
                    suggestion=(
                        "Remove the orientation attribute (rightHanded is the OpenUSD default) "
                        "or correct the vertex winding order."
                    ),
                ))

        # Double-sided check
        ds_attr = mesh.GetDoubleSidedAttr()
        if ds_attr.IsValid() and ds_attr.Get():
            errors.append(_error(
                "3.4.3",
                ErrorType.Warn,
                _prim_site(stage, prim_path),
                (
                    f"Mesh '{prim_path}' has doubleSided=true. "
                    "Assets must not rely on doubleSided to mask incorrect winding "
                    "per REP §3.4. Fix the underlying face orientation instead."
                ),
                suggestion=(
                    "Correct the vertex winding order in your DCC tool and set "
                    "doubleSided=false."
                ),
            ))

        # Triangulation check (collision geometry only)
        if _is_collision_geometry(prim):
            face_counts_attr = mesh.GetFaceVertexCountsAttr()
            if face_counts_attr.IsValid():
                counts = face_counts_attr.Get()
                if counts is not None:
                    non_tri = [c for c in counts if c != 3]
                    if non_tri:
                        errors.append(_error(
                            "3.4.1",
                            ErrorType.Error,
                            _prim_site(stage, prim_path),
                            (
                                f"Collision mesh '{prim_path}' contains {len(non_tri)} non-triangular "
                                f"face(s) (e.g. polygon with {non_tri[0]} vertices). "
                                "Collision meshes must be explicitly triangulated per REP §3.4."
                            ),
                            suggestion=(
                                "Triangulate the mesh in your DCC tool or run a USD triangulation step "
                                "at export time."
                            ),
                        ))
    return errors


def _validate_texture_baking(
    stage: Usd.Stage, timeRange: TimeRange
) -> list[ValidationError]:
    errors: list[ValidationError] = []
    for prim in stage.TraverseAll():
        if prim.GetTypeName() != "Shader":
            continue
        shader = UsdShade.Shader(prim)
        if not shader:
            continue
        shader_id = (shader.GetShaderId() or "").lower()
        if any(hint in shader_id for hint in _PROCEDURAL_NODE_HINTS):
            errors.append(_error(
                "3.3.1",
                ErrorType.Warn,
                _prim_site(stage, str(prim.GetPath())),
                (
                    f"Shader '{prim.GetPath()}' uses procedural node '{shader.GetShaderId()}'. "
                    "Procedural texture graphs are not interoperable and should be baked "
                    "to image-backed textures or mesh primvars per REP §3.3."
                ),
                suggestion=(
                    "Bake this procedural network to PNG/JPEG textures (via UsdUVTexture) "
                    "or baked primvars before distribution."
                ),
            ))
    return errors


def _validate_lighting_portability(
    stage: Usd.Stage, timeRange: TimeRange
) -> list[ValidationError]:
    _AREA_LIGHT_TYPES = {"RectLight", "CylinderLight"}
    errors: list[ValidationError] = []
    for prim in stage.TraverseAll():
        if prim.GetTypeName() not in _AREA_LIGHT_TYPES:
            continue
        errors.append(_error(
            "3.6.1",
            ErrorType.Warn,
            _prim_site(stage, str(prim.GetPath())),
            (
                f"Light '{prim.GetPath()}' is type '{prim.GetTypeName()}'. "
                "Complex area lights are not universally supported and should be "
                "avoided for interoperable assets per REP §3.6."
            ),
            suggestion=(
                "Prefer punctual lights (UsdLuxDistantLight, UsdLuxSphereLight, "
                "or SphereLight + UsdLuxShapingAPI for spot behavior)."
            ),
        ))
    return errors


# ------------------------------------------------------------------ #
# Registration                                                          #
# ------------------------------------------------------------------ #

register_stage_validator(
    "MaterialPortability",
    _validate_material_portability,
    doc="REP §3.1: Materials must use UsdPreviewSurface as the normative surface.",
    keywords=["rep0158:export"],
    section="3.1",
)

register_stage_validator(
    "TextureFormat",
    _validate_texture_format,
    doc="REP §3.1 / §3.2: Validate texture file formats; flag UDIM tiles and forbidden formats.",
    keywords=["rep0158:export"],
    section="3.2",
)

register_stage_validator(
    "GeometryConstraints",
    _validate_geometry_constraints,
    doc="REP §3.4: Collision meshes must be triangulated; meshes must be rightHanded.",
    keywords=["rep0158:export"],
    section="3.4",
)

register_stage_validator(
    "TextureBaking",
    _validate_texture_baking,
    doc="REP §3.3: Procedural texture graphs should be baked to image/primvar data.",
    keywords=["rep0158:export"],
    section="3.3",
)

register_stage_validator(
    "LightingPortability",
    _validate_lighting_portability,
    doc="REP §3.6: avoid complex area lights for interoperable assets.",
    keywords=["rep0158:export"],
    section="3.6",
)
