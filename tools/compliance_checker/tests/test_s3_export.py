"""Tests for §3 – Export & Conversion (opt-in checks).

Check IDs covered:
  3.1.1  Material must have UsdPreviewSurface wired to outputs:surface
  3.1.2  Texture path must not contain UDIM tile patterns
  3.2.1  Texture file format must not be EXR, TIFF, or HDR (except dome lights)
  3.2.2  Data map textures (normal/roughness/metallic/ORM) must not use JPEG
  3.3.1  Procedural texture shader graphs should be baked
  3.4.1  Collision meshes must be fully triangulated
  3.4.2  Mesh orientation must not be leftHanded
  3.4.3  doubleSided must not be used to mask incorrect winding
"""

import pytest
from compliance_checker.checks._tokens import (
    COLLISION_NOT_TRIANGULATED,
    COMPLEX_AREA_LIGHT,
    DATA_MAP_JPEG,
    DOUBLE_SIDED_MESH,
    FORBIDDEN_TEXTURE_FORMAT,
    LEFT_HANDED_ORIENTATION,
    MISSING_PREVIEW_SURFACE,
    PROCEDURAL_SHADER,
    UDIM_TEXTURE,
)

from .conftest import has, make_stage, none_with, run_validators

# ------------------------------------------------------------------ #
# §3.1.1 – Material must use UsdPreviewSurface                         #
# ------------------------------------------------------------------ #


class TestMaterialPortability:
    def test_material_no_surface_output_gives_warning(self):
        stage = make_stage("""
#usda 1.0
def Material "Mat" {}
""")
        assert has(run_validators(stage, "rep0158:MaterialPortability"), MISSING_PREVIEW_SURFACE)

    def test_material_with_preview_surface_no_violation(self):
        stage = make_stage("""
#usda 1.0
def Material "Mat" {
    token outputs:surface.connect = </Mat/Surf.outputs:surface>

    def Shader "Surf" {
        uniform token info:id = "UsdPreviewSurface"
        float inputs:roughness = 0.5
        float inputs:metallic = 0.0
        token outputs:surface
    }
}
""")
        assert none_with(run_validators(stage, "rep0158:MaterialPortability"), MISSING_PREVIEW_SURFACE)

    def test_material_with_mdl_shader_only_gives_warning(self):
        """Proprietary shader on the universal terminal without UsdPreviewSurface."""
        stage = make_stage("""
#usda 1.0
def Material "Mat" {
    token outputs:surface.connect = </Mat/MDLSurf.outputs:out>

    def Shader "MDLSurf" {
        uniform token info:id = "mdlMaterial"
        token outputs:out
    }
}
""")
        assert has(run_validators(stage, "rep0158:MaterialPortability"), MISSING_PREVIEW_SURFACE)


# ------------------------------------------------------------------ #
# §3.1.2 – UDIM tile patterns                                          #
# ------------------------------------------------------------------ #


class TestUDIM:
    @pytest.mark.parametrize(
        "udim_path",
        [
            "./textures/albedo.1001.png",
            "./textures/albedo.<UDIM>.png",
            "./textures/normal_<UDIM>.png",
        ],
    )
    def test_udim_pattern_in_texture_gives_error(self, udim_path):
        stage = make_stage(f"""
#usda 1.0
def Shader "Tex" {{
    uniform token info:id = "UsdUVTexture"
    asset inputs:file = @{udim_path}@
    float3 outputs:rgb
}}
""")
        assert has(run_validators(stage, "rep0158:TextureFormat"), UDIM_TEXTURE)

    def test_non_udim_texture_no_violation(self):
        stage = make_stage("""
#usda 1.0
def Shader "Tex" {
    uniform token info:id = "UsdUVTexture"
    asset inputs:file = @./textures/albedo.png@
    float3 outputs:rgb
}
""")
        assert none_with(run_validators(stage, "rep0158:TextureFormat"), UDIM_TEXTURE)


# ------------------------------------------------------------------ #
# §3.2.1 – Forbidden texture formats                                   #
# ------------------------------------------------------------------ #


class TestTextureFormat:
    @pytest.mark.parametrize("ext", [".exr", ".tiff", ".tif"])
    def test_forbidden_texture_extension_gives_error(self, ext):
        stage = make_stage(f"""
#usda 1.0
def Shader "Tex" {{
    uniform token info:id = "UsdUVTexture"
    asset inputs:file = @./textures/normal_map{ext}@
    float3 outputs:rgb
}}
""")
        assert has(run_validators(stage, "rep0158:TextureFormat"), FORBIDDEN_TEXTURE_FORMAT)

    @pytest.mark.parametrize("ext", [".png", ".jpg", ".jpeg"])
    def test_permitted_texture_extensions_no_violation(self, ext):
        stage = make_stage(f"""
#usda 1.0
def Shader "Tex" {{
    uniform token info:id = "UsdUVTexture"
    asset inputs:file = @./textures/albedo{ext}@
    float3 outputs:rgb
}}
""")
        assert none_with(run_validators(stage, "rep0158:TextureFormat"), FORBIDDEN_TEXTURE_FORMAT)

    def test_non_uvtexture_shader_not_checked(self):
        """Shaders other than UsdUVTexture are out of scope for format checks."""
        stage = make_stage("""
#usda 1.0
def Shader "Surf" {
    uniform token info:id = "UsdPreviewSurface"
    token outputs:surface
}
""")
        assert none_with(run_validators(stage, "rep0158:TextureFormat"), FORBIDDEN_TEXTURE_FORMAT)

    def test_data_map_with_jpeg_gives_error(self):
        stage = make_stage("""
#usda 1.0
def Shader "Tex" {
    uniform token info:id = "UsdUVTexture"
    asset inputs:file = @./textures/normal_map.jpg@
    float3 outputs:rgb
}
""")
        assert has(run_validators(stage, "rep0158:TextureFormat"), DATA_MAP_JPEG)

    def test_data_map_with_png_no_violation(self):
        stage = make_stage("""
#usda 1.0
def Shader "Tex" {
    uniform token info:id = "UsdUVTexture"
    asset inputs:file = @./textures/roughness.png@
    float outputs:r
}
""")
        assert none_with(run_validators(stage, "rep0158:TextureFormat"), DATA_MAP_JPEG)


class TestTextureBaking:
    def test_procedural_noise_shader_gives_warning(self):
        stage = make_stage("""
#usda 1.0
def Shader "Noise" {
    uniform token info:id = "UsdNoise2d"
    float outputs:result
}
""")
        assert has(run_validators(stage, "rep0158:TextureBaking"), PROCEDURAL_SHADER)

    def test_uvtexture_shader_no_violation(self):
        stage = make_stage("""
#usda 1.0
def Shader "Tex" {
    uniform token info:id = "UsdUVTexture"
    asset inputs:file = @./textures/albedo.png@
    float3 outputs:rgb
}
""")
        assert none_with(run_validators(stage, "rep0158:TextureBaking"), PROCEDURAL_SHADER)


# ------------------------------------------------------------------ #
# §3.4.1 – Collision mesh triangulation                                #
# ------------------------------------------------------------------ #


class TestGeometryConstraints:
    def test_quad_collision_mesh_gives_error(self):
        """A collision mesh with quads (faceVertexCounts entry = 4) must be triangulated."""
        stage = make_stage("""
#usda 1.0
def Mesh "Collider" (
    prepend apiSchemas = ["PhysicsCollisionAPI"]
) {
    token purpose = "guide"
    int[] faceVertexCounts = [4, 4]
    int[] faceVertexIndices = [0, 1, 2, 3, 4, 5, 6, 7]
    point3f[] points = [
        (0,0,0),(1,0,0),(1,1,0),(0,1,0),
        (0,0,1),(1,0,1),(1,1,1),(0,1,1)
    ]
}
""")
        assert has(run_validators(stage, "rep0158:GeometryConstraints"), COLLISION_NOT_TRIANGULATED)

    def test_triangulated_collision_mesh_no_violation(self):
        stage = make_stage("""
#usda 1.0
def Mesh "Collider" (
    prepend apiSchemas = ["PhysicsCollisionAPI"]
) {
    token purpose = "guide"
    int[] faceVertexCounts = [3, 3, 3, 3]
    int[] faceVertexIndices = [0,1,2, 0,2,3, 0,3,4, 0,4,5]
    point3f[] points = [
        (0,0,0),(1,0,0),(1,1,0),(0,1,0),(0,0,1),(1,0,1)
    ]
}
""")
        assert none_with(run_validators(stage, "rep0158:GeometryConstraints"), COLLISION_NOT_TRIANGULATED)

    def test_ngon_visual_mesh_not_checked(self):
        """Visual meshes (no CollisionAPI, no 'guide' purpose) are not checked for triangulation."""
        stage = make_stage("""
#usda 1.0
def Mesh "Visual" {
    int[] faceVertexCounts = [4, 4]
    int[] faceVertexIndices = [0, 1, 2, 3, 4, 5, 6, 7]
    point3f[] points = [
        (0,0,0),(1,0,0),(1,1,0),(0,1,0),
        (0,0,1),(1,0,1),(1,1,1),(0,1,1)
    ]
}
""")
        assert none_with(run_validators(stage, "rep0158:GeometryConstraints"), COLLISION_NOT_TRIANGULATED)

    def test_collision_mesh_named_collision_scope_not_triangulated_gives_error(self):
        """Heuristic: parent named 'collision' also marks mesh as collision geometry."""
        stage = make_stage("""
#usda 1.0
def Xform "link" {
    def Xform "collision" {
        def Mesh "shape" {
            int[] faceVertexCounts = [4]
            int[] faceVertexIndices = [0, 1, 2, 3]
            point3f[] points = [(0,0,0),(1,0,0),(1,1,0),(0,1,0)]
        }
    }
}
""")
        assert has(run_validators(stage, "rep0158:GeometryConstraints"), COLLISION_NOT_TRIANGULATED)


# ------------------------------------------------------------------ #
# §3.4.2 – Mesh orientation                                            #
# ------------------------------------------------------------------ #


class TestMeshOrientation:
    def test_left_handed_mesh_gives_warning(self):
        stage = make_stage("""
#usda 1.0
def Mesh "Visual" {
    uniform token orientation = "leftHanded"
    int[] faceVertexCounts = [3]
    int[] faceVertexIndices = [0, 1, 2]
    point3f[] points = [(0,0,0),(1,0,0),(0,1,0)]
}
""")
        assert has(run_validators(stage, "rep0158:GeometryConstraints"), LEFT_HANDED_ORIENTATION)

    def test_right_handed_mesh_no_violation(self):
        stage = make_stage("""
#usda 1.0
def Mesh "Visual" {
    uniform token orientation = "rightHanded"
    int[] faceVertexCounts = [3]
    int[] faceVertexIndices = [0, 1, 2]
    point3f[] points = [(0,0,0),(1,0,0),(0,1,0)]
}
""")
        assert none_with(run_validators(stage, "rep0158:GeometryConstraints"), LEFT_HANDED_ORIENTATION)

    def test_default_orientation_no_violation(self):
        """OpenUSD default is rightHanded; no authored orientation must not be flagged."""
        stage = make_stage("""
#usda 1.0
def Mesh "Visual" {
    int[] faceVertexCounts = [3]
    int[] faceVertexIndices = [0, 1, 2]
    point3f[] points = [(0,0,0),(1,0,0),(0,1,0)]
}
""")
        assert none_with(run_validators(stage, "rep0158:GeometryConstraints"), LEFT_HANDED_ORIENTATION)


# ------------------------------------------------------------------ #
# §3.4.3 – doubleSided                                                 #
# ------------------------------------------------------------------ #


class TestDoubleSided:
    def test_double_sided_mesh_gives_warning(self):
        stage = make_stage("""
#usda 1.0
def Mesh "Visual" {
    uniform bool doubleSided = true
    int[] faceVertexCounts = [3]
    int[] faceVertexIndices = [0, 1, 2]
    point3f[] points = [(0,0,0),(1,0,0),(0,1,0)]
}
""")
        assert has(run_validators(stage, "rep0158:GeometryConstraints"), DOUBLE_SIDED_MESH)

    def test_single_sided_mesh_no_violation(self):
        stage = make_stage("""
#usda 1.0
def Mesh "Visual" {
    uniform bool doubleSided = false
    int[] faceVertexCounts = [3]
    int[] faceVertexIndices = [0, 1, 2]
    point3f[] points = [(0,0,0),(1,0,0),(0,1,0)]
}
""")
        assert none_with(run_validators(stage, "rep0158:GeometryConstraints"), DOUBLE_SIDED_MESH)


# ------------------------------------------------------------------ #
# §3.6 – Lighting portability                                          #
# ------------------------------------------------------------------ #


class TestLightingPortability:
    def test_rect_light_gives_warning(self):
        stage = make_stage("""
#usda 1.0
def RectLight "AreaLight" {}
""")
        assert has(run_validators(stage, "rep0158:LightingPortability"), COMPLEX_AREA_LIGHT)

    def test_cylinder_light_gives_warning(self):
        stage = make_stage("""
#usda 1.0
def CylinderLight "TubeLight" {}
""")
        assert has(run_validators(stage, "rep0158:LightingPortability"), COMPLEX_AREA_LIGHT)

    def test_punctual_light_no_violation(self):
        stage = make_stage("""
#usda 1.0
def SphereLight "PointLike" {}
def DistantLight "Sun" {}
""")
        assert none_with(run_validators(stage, "rep0158:LightingPortability"), COMPLEX_AREA_LIGHT)
