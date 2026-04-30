"""Section 3 – Export & Conversion checks (opt-in, potentially slow)."""

from __future__ import annotations

import os
import re
from typing import Iterator

from pxr import Usd, UsdGeom, UsdShade

from ..report import Severity, Violation
from .base import BaseCheck

# ------------------------------------------------------------------ #
# Constants                                                             #
# ------------------------------------------------------------------ #

_UDIM_RE = re.compile(r"\.\d{4}\b|<UDIM>|<%d>|UDIM", re.IGNORECASE)

_FORBIDDEN_EXTENSIONS = {".exr", ".tiff", ".tif", ".hdr"}
_DATA_MAP_JPEG_EXTENSIONS = {".jpg", ".jpeg"}
# HDR is allowed for dome lights (checked separately)
_DATA_MAP_HINTS = ("normal", "metallic", "roughness", "orm", "occlusion")

# Procedural / math shader node type hints (non-exhaustive heuristic)
_PROCEDURAL_NODE_HINTS = ("noise", "math", "mix", "blend", "remap", "fractal")


def _applied(prim: Usd.Prim) -> set[str]:
    """Return all applied API schema names, including unregistered custom schemas."""
    list_op = prim.GetMetadata("apiSchemas")
    if list_op is None:
        return set()
    return set(list_op.GetAppliedItems())


class MaterialPortabilityCheck(BaseCheck):
    """REP §3.1: Materials must use UsdPreviewSurface as the normative surface."""

    section = "3.1"

    def run(self, stage: Usd.Stage) -> Iterator[Violation]:
        for prim in stage.TraverseAll():
            if prim.GetTypeName() != "Material":
                continue
            mat = UsdShade.Material(prim)
            if not mat:
                continue
            yield from self._check_preview_surface(prim, mat)

    def _check_preview_surface(
        self, prim: Usd.Prim, mat: UsdShade.Material
    ) -> Iterator[Violation]:
        prim_path = str(prim.GetPath())
        # Check that the universal surface output is connected
        if not mat.GetSurfaceOutput():
            yield Violation(
                check_id="3.1.1",
                severity=Severity.WARNING,
                prim_path=prim_path,
                section=self.section,
                message=(
                    f"Material '{prim_path}' has no connected 'outputs:surface' terminal. "
                    "A UsdPreviewSurface wired to the universal surface output is required "
                    "for glTF conversion per REP §3.1."
                ),
                suggestion=(
                    "Wire a UsdPreviewSurface shader to 'outputs:surface' inside the material. "
                    "Proprietary shaders (MDL, OSL) should use renderer-specific terminals only."
                ),
            )
            return

        # GetConnectedSources() returns (sources_list, invalid_sources_list).
        sources, _ = mat.GetSurfaceOutput().GetConnectedSources()
        if not sources:
            yield Violation(
                check_id="3.1.1",
                severity=Severity.WARNING,
                prim_path=prim_path,
                section=self.section,
                message=(
                    f"Material '{prim_path}' 'outputs:surface' terminal has no connected shader. "
                    "A UsdPreviewSurface must be wired to the universal surface output per REP §3.1."
                ),
                suggestion=(
                    "Wire a UsdPreviewSurface shader to 'outputs:surface' inside the material."
                ),
            )
            return

        # Walk the connected shader and check it's a UsdPreviewSurface.
        for src in sources:
            shader_prim = src.source.GetPrim()
            shader = UsdShade.Shader(shader_prim)
            if shader:
                shader_id = shader.GetShaderId()
                if shader_id and "UsdPreviewSurface" not in shader_id:
                    yield Violation(
                        check_id="3.1.1",
                        severity=Severity.WARNING,
                        prim_path=prim_path,
                        section=self.section,
                        message=(
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
                    )


class TextureFormatCheck(BaseCheck):
    """REP §3.1 / §3.2: Validate texture file formats; flag UDIM tiles and forbidden formats."""

    section = "3.2"

    def run(self, stage: Usd.Stage) -> Iterator[Violation]:
        for prim in stage.TraverseAll():
            if prim.GetTypeName() != "Shader":
                continue
            shader = UsdShade.Shader(prim)
            if not shader:
                continue
            shader_id = shader.GetShaderId()
            if not shader_id or "UsdUVTexture" not in shader_id:
                continue
            yield from self._check_texture_file(prim, shader, stage)

    def _check_texture_file(
        self, prim: Usd.Prim, shader: UsdShade.Shader, stage: Usd.Stage
    ) -> Iterator[Violation]:
        prim_path = str(prim.GetPath())
        file_input = shader.GetInput("file")
        if not file_input:
            return
        asset_path = file_input.Get()
        if not asset_path:
            return

        path_str = (
            str(asset_path.path) if hasattr(asset_path, "path") else str(asset_path)
        )
        if not path_str:
            return

        # UDIM check (§3.1)
        if _UDIM_RE.search(path_str):
            yield Violation(
                check_id="3.1.2",
                severity=Severity.ERROR,
                prim_path=prim_path,
                section="3.1",
                message=(
                    f"Texture path '{path_str}' on '{prim_path}' contains a UDIM tile pattern. "
                    "Multi-tile UV mapping (UDIMs) is unsupported by glTF 2.0 and many real-time "
                    "engines; must not be used per REP §3.1."
                ),
                suggestion=(
                    "Pack all UVs into the [0,1] space using texture atlasing. "
                    "If multiple textures are required, split geometry with UsdGeomSubset "
                    "and assign separate materials."
                ),
            )

        # Forbidden format check (§3.2)
        _, ext = os.path.splitext(path_str.lower())
        if ext in _FORBIDDEN_EXTENSIONS:
            # HDR is allowed for dome lights
            is_dome_light = self._is_dome_light_texture(prim, stage)
            if ext == ".hdr" and is_dome_light:
                return
            yield Violation(
                check_id="3.2.1",
                severity=Severity.ERROR,
                prim_path=prim_path,
                section=self.section,
                message=(
                    f"Texture '{path_str}' on '{prim_path}' uses forbidden format '{ext}'. "
                    "EXR, TIFF, and other HDR/DCC formats must not be used for surface maps "
                    "in distributed assets per REP §3.2. "
                    "(HDR is only permitted for UsdLuxDomeLight environment maps.)"
                ),
                suggestion=(
                    "Convert to PNG (for data maps: normal, metallic, roughness, ORM) or "
                    "JPEG (for color maps without alpha)."
                ),
            )
        if ext in _DATA_MAP_JPEG_EXTENSIONS and any(
            hint in path_str.lower() for hint in _DATA_MAP_HINTS
        ):
            yield Violation(
                check_id="3.2.2",
                severity=Severity.ERROR,
                prim_path=prim_path,
                section=self.section,
                message=(
                    f"Data map texture '{path_str}' on '{prim_path}' uses JPEG. "
                    "Normal/metallic/roughness/ORM maps must use lossless PNG per REP §3.2."
                ),
                suggestion="Convert the data map to PNG.",
            )

    def _is_dome_light_texture(self, shader_prim: Usd.Prim, stage: Usd.Stage) -> bool:
        """Heuristic: check if this shader feeds a DomeLight."""
        parent = shader_prim.GetParent()
        while parent and parent.IsValid():
            if parent.GetTypeName() == "DomeLight":
                return True
            parent = parent.GetParent()
        return False


class GeometryConstraintsCheck(BaseCheck):
    """REP §3.4: Collision meshes must be triangulated; meshes must be rightHanded."""

    section = "3.4"

    def run(self, stage: Usd.Stage) -> Iterator[Violation]:
        for prim in stage.TraverseAll():
            if prim.GetTypeName() != "Mesh":
                continue
            mesh = UsdGeom.Mesh(prim)
            if not mesh:
                continue
            yield from self._check_orientation(prim, mesh)
            yield from self._check_double_sided(prim, mesh)
            # Only check triangulation for collision geometry
            if self._is_collision_geometry(prim):
                yield from self._check_triangulation(prim, mesh)

    def _is_collision_geometry(self, prim: Usd.Prim) -> bool:
        # Heuristic: prim purpose is 'guide' (collision), or parent scope is named 'collision'
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

    def _check_triangulation(
        self, prim: Usd.Prim, mesh: UsdGeom.Mesh
    ) -> Iterator[Violation]:
        face_counts_attr = mesh.GetFaceVertexCountsAttr()
        if not face_counts_attr.IsValid():
            return
        counts = face_counts_attr.Get()
        if counts is None:
            return
        non_tri = [c for c in counts if c != 3]
        if non_tri:
            yield Violation(
                check_id="3.4.1",
                severity=Severity.ERROR,
                prim_path=str(prim.GetPath()),
                section=self.section,
                message=(
                    f"Collision mesh '{prim.GetPath()}' contains {len(non_tri)} non-triangular "
                    f"face(s) (e.g. polygon with {non_tri[0]} vertices). "
                    "Collision meshes must be explicitly triangulated per REP §3.4."
                ),
                suggestion=(
                    "Triangulate the mesh in your DCC tool or run a USD triangulation step "
                    "at export time."
                ),
            )

    def _check_orientation(
        self, prim: Usd.Prim, mesh: UsdGeom.Mesh
    ) -> Iterator[Violation]:
        orientation_attr = mesh.GetOrientationAttr()
        if not orientation_attr.IsValid():
            return
        orientation = orientation_attr.Get()
        if orientation is not None and str(orientation) == "leftHanded":
            yield Violation(
                check_id="3.4.2",
                severity=Severity.WARNING,
                prim_path=str(prim.GetPath()),
                section=self.section,
                message=(
                    f"Mesh '{prim.GetPath()}' uses orientation='leftHanded'. "
                    "Meshes must use 'rightHanded' (CCW winding) to align with glTF 2.0 "
                    "per REP §3.4."
                ),
                suggestion=(
                    "Remove the orientation attribute (rightHanded is the OpenUSD default) "
                    "or correct the vertex winding order."
                ),
            )

    def _check_double_sided(
        self, prim: Usd.Prim, mesh: UsdGeom.Mesh
    ) -> Iterator[Violation]:
        ds_attr = mesh.GetDoubleSidedAttr()
        if not ds_attr.IsValid():
            return
        if ds_attr.Get():
            yield Violation(
                check_id="3.4.3",
                severity=Severity.WARNING,
                prim_path=str(prim.GetPath()),
                section=self.section,
                message=(
                    f"Mesh '{prim.GetPath()}' has doubleSided=true. "
                    "Assets must not rely on doubleSided to mask incorrect winding "
                    "per REP §3.4. Fix the underlying face orientation instead."
                ),
                suggestion=(
                    "Correct the vertex winding order in your DCC tool and set "
                    "doubleSided=false."
                ),
            )


class TextureBakingCheck(BaseCheck):
    """REP §3.3: Procedural texture graphs should be baked to image/primvar data."""

    section = "3.3"

    def run(self, stage: Usd.Stage) -> Iterator[Violation]:
        for prim in stage.TraverseAll():
            if prim.GetTypeName() != "Shader":
                continue
            shader = UsdShade.Shader(prim)
            if not shader:
                continue
            shader_id = (shader.GetShaderId() or "").lower()
            if any(hint in shader_id for hint in _PROCEDURAL_NODE_HINTS):
                yield Violation(
                    check_id="3.3.1",
                    severity=Severity.WARNING,
                    prim_path=str(prim.GetPath()),
                    section=self.section,
                    message=(
                        f"Shader '{prim.GetPath()}' uses procedural node '{shader.GetShaderId()}'. "
                        "Procedural texture graphs are not interoperable and should be baked "
                        "to image-backed textures or mesh primvars per REP §3.3."
                    ),
                    suggestion=(
                        "Bake this procedural network to PNG/JPEG textures (via UsdUVTexture) "
                        "or baked primvars before distribution."
                    ),
                )


class LightingPortabilityCheck(BaseCheck):
    """REP §3.6: avoid complex area lights for interoperable assets."""

    section = "3.6"

    _AREA_LIGHT_TYPES = {"RectLight", "CylinderLight"}

    def run(self, stage: Usd.Stage) -> Iterator[Violation]:
        for prim in stage.TraverseAll():
            if prim.GetTypeName() not in self._AREA_LIGHT_TYPES:
                continue
            yield Violation(
                check_id="3.6.1",
                severity=Severity.WARNING,
                prim_path=str(prim.GetPath()),
                section=self.section,
                message=(
                    f"Light '{prim.GetPath()}' is type '{prim.GetTypeName()}'. "
                    "Complex area lights are not universally supported and should be "
                    "avoided for interoperable assets per REP §3.6."
                ),
                suggestion=(
                    "Prefer punctual lights (UsdLuxDistantLight, UsdLuxSphereLight, "
                    "or SphereLight + UsdLuxShapingAPI for spot behavior)."
                ),
            )
