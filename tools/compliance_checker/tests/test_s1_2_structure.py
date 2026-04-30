"""Tests for §1.2 – Asset Structure & Composition.

Check IDs covered:
  1.2.1  defaultPrim must be set
  1.2.2  defaultPrim must carry assetInfo:identifier
  1.2.3  defaultPrim must carry assetInfo:version
  1.2.4  No absolute or omniverse:// paths in external references
  1.2.6  component must not contain component
  1.2.7  VariantSet must have a default selection
  1.2.8  Inherits/Specializes arcs flagged for verification
  1.2.9  Custom string prefabPath-like attributes prohibited
  1.2.12 Schema/relationship-bearing layers must use .usda encoding
  1.2.13 Heavy-data layers should use .usdc encoding
"""

import os

from compliance_checker.checks.s1_2_structure import (
    AssetManagementCheck,
    CompositionModelCheck,
    InheritsSpecializesCheck,
    LayerEncodingCheck,
    ParallelSimulationInstancingCheck,
    PathConventionCheck,
    PayloadKinematicTopologyCheck,
    VariantDefaultCheck,
)
from pxr import Sdf, Usd

from .conftest import has, make_stage, none_with, run_check

# ------------------------------------------------------------------ #
# §1.2.1 – defaultPrim                                                 #
# ------------------------------------------------------------------ #


class TestDefaultPrim:
    def test_missing_default_prim_gives_warning(self):
        stage = make_stage("""
#usda 1.0
def Xform "Robot" {}
""")
        assert has(run_check(stage, AssetManagementCheck), "1.2.1")

    def test_set_default_prim_clears_warning(self):
        stage = make_stage("""
#usda 1.0
(
    defaultPrim = "Robot"
)
def Xform "Robot"
(
    assetInfo = {
        string identifier = "my.robot"
        string version = "1.0.0"
    }
) {}
""")
        assert none_with(run_check(stage, AssetManagementCheck), "1.2.1")


# ------------------------------------------------------------------ #
# §1.2.2 / §1.2.3 – assetInfo on defaultPrim                          #
# ------------------------------------------------------------------ #


class TestAssetInfo:
    def test_missing_identifier_gives_warning(self):
        stage = make_stage("""
#usda 1.0
(
    defaultPrim = "Robot"
)
def Xform "Robot"
(
    assetInfo = {
        string version = "1.0.0"
    }
) {}
""")
        assert has(run_check(stage, AssetManagementCheck), "1.2.2")

    def test_missing_version_gives_warning(self):
        stage = make_stage("""
#usda 1.0
(
    defaultPrim = "Robot"
)
def Xform "Robot"
(
    assetInfo = {
        string identifier = "my.robot"
    }
) {}
""")
        assert has(run_check(stage, AssetManagementCheck), "1.2.3")

    def test_complete_asset_info_no_violation(self):
        stage = make_stage("""
#usda 1.0
(
    defaultPrim = "Robot"
)
def Xform "Robot"
(
    assetInfo = {
        string identifier = "acme.robot.arm"
        string version = "2.1.0"
    }
) {}
""")
        v = run_check(stage, AssetManagementCheck)
        assert none_with(v, "1.2.2")
        assert none_with(v, "1.2.3")


# ------------------------------------------------------------------ #
# §1.2.4 – path conventions                                            #
# ------------------------------------------------------------------ #


class TestPathConventions:
    def test_absolute_path_in_reference_gives_error(self, tmp_path):
        target = tmp_path / "geo.usda"
        target.write_text("#usda 1.0\n")
        main = tmp_path / "main.usda"
        # Use an absolute path reference
        main.write_text(f"""#usda 1.0
def Xform "Robot" (
    references = @{target}@
) {{}}
""")
        from pxr import Usd

        stage = Usd.Stage.Open(str(main))
        assert has(run_check(stage, PathConventionCheck), "1.2.4")

    def test_relative_path_in_reference_no_violation(self, tmp_path):
        target = tmp_path / "geo.usda"
        target.write_text("#usda 1.0\n")
        main = tmp_path / "main.usda"
        main.write_text("""#usda 1.0
def Xform "Robot" (
    references = @./geo.usda@
) {}
""")
        from pxr import Usd

        stage = Usd.Stage.Open(str(main))
        assert none_with(run_check(stage, PathConventionCheck), "1.2.4")

    def test_omniverse_uri_gives_error(self, tmp_path):
        main = tmp_path / "main.usda"
        main.write_text("""#usda 1.0
def Xform "Robot" (
    references = @omniverse://my-server/asset.usd@
) {}
""")
        from pxr import Usd

        stage = Usd.Stage.Open(str(main))
        assert has(run_check(stage, PathConventionCheck), "1.2.4")

    def test_custom_prefab_path_attribute_gives_error(self):
        stage = make_stage("""
#usda 1.0
def Xform "Robot" {
    custom string mysim:prefabPath = "robot_model.usd"
}
""")
        assert has(run_check(stage, PathConventionCheck), "1.2.9")

    def test_custom_asset_ref_attribute_gives_error(self):
        stage = make_stage("""
#usda 1.0
def Xform "Robot" {
    custom string vendor:assetRef = "model.usda"
}
""")
        assert has(run_check(stage, PathConventionCheck), "1.2.9")

    def test_regular_custom_string_attribute_no_violation(self):
        """A custom string attribute that does not suggest dynamic loading is fine."""
        stage = make_stage("""
#usda 1.0
def Xform "Robot" {
    custom string robot:description = "A simple robot arm"
}
""")
        assert none_with(run_check(stage, PathConventionCheck), "1.2.9")


# ------------------------------------------------------------------ #
# §1.2.6 – component nesting                                           #
# ------------------------------------------------------------------ #


class TestComponentNesting:
    def test_component_inside_component_gives_warning(self):
        stage = make_stage("""
#usda 1.0
def Xform "Robot" (
    kind = "component"
) {
    def Xform "Gripper" (
        kind = "component"
    ) {}
}
""")
        assert has(run_check(stage, CompositionModelCheck), "1.2.6")

    def test_component_inside_assembly_no_violation(self):
        stage = make_stage("""
#usda 1.0
def Xform "Scene" (
    kind = "assembly"
) {
    def Xform "Robot" (
        kind = "component"
    ) {}
}
""")
        assert none_with(run_check(stage, CompositionModelCheck), "1.2.6")

    def test_subcomponent_inside_component_no_violation(self):
        stage = make_stage("""
#usda 1.0
def Xform "Robot" (
    kind = "component"
) {
    def Xform "Gripper" (
        kind = "subcomponent"
    ) {}
}
""")
        assert none_with(run_check(stage, CompositionModelCheck), "1.2.6")

    def test_deeply_nested_component_gives_warning(self):
        """A component two levels deep inside another component is still a violation."""
        stage = make_stage("""
#usda 1.0
def Xform "Robot" (
    kind = "component"
) {
    def Xform "Arm" (
        kind = "subcomponent"
    ) {
        def Xform "Hand" (
            kind = "component"
        ) {}
    }
}
""")
        assert has(run_check(stage, CompositionModelCheck), "1.2.6")


# ------------------------------------------------------------------ #
# §1.2.7 – VariantSet default selection                                #
# ------------------------------------------------------------------ #


class TestVariantDefault:
    def test_variant_set_no_selection_gives_warning(self):
        stage = make_stage("""
#usda 1.0
def Xform "Robot" (
    prepend variantSets = "end_effector"
) {
    variantSet "end_effector" = {
        "gripper" {}
        "suction_cup" {}
    }
}
""")
        assert has(run_check(stage, VariantDefaultCheck), "1.2.7")

    def test_variant_set_with_selection_no_violation(self):
        stage = make_stage("""
#usda 1.0
def Xform "Robot" (
    variants = {
        string end_effector = "gripper"
    }
    prepend variantSets = "end_effector"
) {
    variantSet "end_effector" = {
        "gripper" {
        }
        "suction_cup" {
        }
    }
}
""")
        assert none_with(run_check(stage, VariantDefaultCheck), "1.2.7")


# ------------------------------------------------------------------ #
# §1.2.8 – Inherits / Specializes arcs                                 #
# ------------------------------------------------------------------ #


class TestInheritsSpecializes:
    def test_inherit_arc_gives_warning(self):
        stage = make_stage("""
#usda 1.0
class "BaseRobot" {}

def Xform "Robot" (
    inherits = </BaseRobot>
) {}
""")
        assert has(run_check(stage, InheritsSpecializesCheck), "1.2.8")

    def test_specializes_arc_gives_warning(self):
        stage = make_stage("""
#usda 1.0
class "BaseRobot" {}

def Xform "Robot" (
    specializes = </BaseRobot>
) {}
""")
        assert has(run_check(stage, InheritsSpecializesCheck), "1.2.8")

    def test_no_arcs_no_violation(self):
        stage = make_stage("""
#usda 1.0
def Xform "Robot" {}
""")
        assert none_with(run_check(stage, InheritsSpecializesCheck), "1.2.8")


# ------------------------------------------------------------------ #
# §1.2.3 – Payload pattern constraints                                 #
# ------------------------------------------------------------------ #


class TestPayloadKinematicTopology:
    def test_payload_root_with_rigid_body_gives_error(self):
        stage = make_stage("""
#usda 1.0
def Xform "Robot" (
    payload = @./body.usda@
    prepend apiSchemas = ["PhysicsRigidBodyAPI"]
) {}
""")
        assert has(run_check(stage, PayloadKinematicTopologyCheck), "1.2.10")

    def test_payload_root_with_ros_context_gives_error(self):
        stage = make_stage("""
#usda 1.0
def Xform "Robot" (
    payload = @./body.usda@
    prepend apiSchemas = ["RosContextAPI"]
) {
    string ros:context:namespace = "robot_1"
}
""")
        assert has(run_check(stage, PayloadKinematicTopologyCheck), "1.2.10")

    def test_non_payload_rigid_body_no_violation(self):
        stage = make_stage("""
#usda 1.0
def Xform "Robot" (
    prepend apiSchemas = ["PhysicsRigidBodyAPI"]
) {}
""")
        assert none_with(run_check(stage, PayloadKinematicTopologyCheck), "1.2.10")


# ------------------------------------------------------------------ #
# §1.2.6 – Parallel simulation and instancing                          #
# ------------------------------------------------------------------ #


class TestParallelSimulationInstancing:
    def test_point_instancer_with_physics_prototype_gives_error(self):
        stage = make_stage("""
#usda 1.0
def PointInstancer "envs" {
    rel prototypes = [</Prototypes/RobotProto>]
    point3f[] positions = [(0,0,0), (2,0,0)]
    quath[] orientations = [(1,0,0,0), (1,0,0,0)]
    int[] protoIndices = [0, 0]
}
def Scope "Prototypes" {
    def Xform "RobotProto" (
        prepend apiSchemas = ["PhysicsRigidBodyAPI"]
    ) {}
}
""")
        assert has(run_check(stage, ParallelSimulationInstancingCheck), "1.2.11")

    def test_point_instancer_with_visual_only_prototype_no_violation(self):
        stage = make_stage("""
#usda 1.0
def PointInstancer "clutter" {
    rel prototypes = [</Prototypes/Bolt>]
    point3f[] positions = [(0,0,0), (1,0,0)]
    quath[] orientations = [(1,0,0,0), (1,0,0,0)]
    int[] protoIndices = [0, 0]
}
def Scope "Prototypes" {
    def Mesh "Bolt" {
        int[] faceVertexCounts = [3]
        int[] faceVertexIndices = [0,1,2]
        point3f[] points = [(0,0,0), (1,0,0), (0,1,0)]
    }
}
""")
        assert none_with(run_check(stage, ParallelSimulationInstancingCheck), "1.2.11")


# ------------------------------------------------------------------ #
# §1.2.12 / §1.2.13 – Layer encoding                                  #
# ------------------------------------------------------------------ #


def _write_layer(tmp_path, filename: str, content: str) -> str:
    path = os.path.join(str(tmp_path), filename)
    with open(path, "w") as f:
        f.write(content)
    return path


class TestLayerEncoding:
    def test_anonymous_layer_skipped(self):
        """Anonymous in-memory layers are never flagged."""
        stage = make_stage("""
#usda 1.0
def Xform "Robot" (
    prepend apiSchemas = ["PhysicsRigidBodyAPI"]
) {}
""")
        v = run_check(stage, LayerEncodingCheck)
        assert none_with(v, "1.2.12")
        assert none_with(v, "1.2.13")

    def test_usdc_layer_with_api_schemas_gives_warning(self, tmp_path):
        """A .usdc layer that contains API schemas should warn (use .usda instead)."""
        usdc_path = os.path.join(str(tmp_path), "ros.usdc")
        layer = Sdf.Layer.CreateNew(usdc_path)
        prim_spec = Sdf.PrimSpec(layer, "RosInterface", Sdf.SpecifierDef, "Xform")
        prim_spec.SetInfo(
            "apiSchemas",
            Sdf.TokenListOp.CreateExplicit(["RosTopicAPI"]),
        )
        layer.Save()

        stage = Usd.Stage.Open(usdc_path)
        v = run_check(stage, LayerEncodingCheck)
        assert has(v, "1.2.12")

    def test_usda_layer_with_api_schemas_no_violation(self, tmp_path):
        """A .usda layer with API schemas is correct – no warning."""
        usda_path = _write_layer(
            tmp_path,
            "ros.usda",
            """#usda 1.0
def Xform "RosInterface" (
    prepend apiSchemas = ["RosTopicAPI"]
) {}
""",
        )
        stage = Usd.Stage.Open(usda_path)
        assert none_with(run_check(stage, LayerEncodingCheck), "1.2.12")

    def test_usda_layer_with_heavy_geometry_gives_warning(self, tmp_path):
        """A .usda layer containing large mesh data should warn (use .usdc instead)."""
        points = ", ".join(f"({i},0,0)" for i in range(60))
        face_counts = ", ".join(["3"] * 20)
        face_indices = ", ".join(str(i) for i in range(60))
        usda_path = _write_layer(
            tmp_path,
            "geometries.usda",
            f"""#usda 1.0
def Mesh "BigMesh" {{
    int[] faceVertexCounts = [{face_counts}]
    int[] faceVertexIndices = [{face_indices}]
    point3f[] points = [{points}]
}}
""",
        )
        stage = Usd.Stage.Open(usda_path)
        assert has(run_check(stage, LayerEncodingCheck), "1.2.13")

    def test_usdc_layer_with_only_geometry_no_schema_warning(self, tmp_path):
        """A .usdc layer with pure geometry (no schemas) should not trigger 1.2.12."""
        usdc_path = os.path.join(str(tmp_path), "geometries.usdc")
        layer = Sdf.Layer.CreateNew(usdc_path)
        prim_spec = Sdf.PrimSpec(layer, "BigMesh", Sdf.SpecifierDef, "Mesh")
        points_attr = Sdf.AttributeSpec(
            prim_spec,
            "points",
            Sdf.ValueTypeNames.Point3fArray,
        )
        from pxr import Gf, Vt

        points_attr.default = Vt.Vec3fArray([Gf.Vec3f(i, 0, 0) for i in range(60)])
        layer.Save()

        stage = Usd.Stage.Open(usdc_path)
        assert none_with(run_check(stage, LayerEncodingCheck), "1.2.12")
