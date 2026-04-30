"""Tests for §1.3 – Physics.

Check IDs covered:
  1.3.1  Non-continuous joints must author explicit physics:lowerLimit and physics:upperLimit
  1.3.2  At most one PhysicsArticulationRootAPI per stage
  1.3.3  PhysicsRigidBodyAPI prim must not have physics:mass = 0
  1.3.5  Collision geometry must have physics-purpose material binding with all three friction attrs
  1.3.7  MimicJointAPI must only be applied to revolute or prismatic joints
  1.3.8  Prims with physics/ROS schemas must not be instanceable
"""

from compliance_checker.checks._tokens import (
    COLLISION_GEOMETRY_AUTHORING,
    COLLISION_MISSING_MATERIAL,
    INSTANCEABLE_PHYSICS,
    MIMIC_JOINT_CONSTRAINT,
    MIMIC_JOINT_DEPRECATED,
    MISSING_JOINT_LIMITS,
    MULTIPLE_ARTICULATION_ROOTS,
    NON_POSITIVE_MASS,
)

from .conftest import has, make_stage, none_with, run_validators

# ------------------------------------------------------------------ #
# §1.3.1 – Joint limits                                                #
# ------------------------------------------------------------------ #


class TestJointLimits:
    def test_revolute_joint_missing_both_limits_gives_errors(self):
        stage = make_stage("""
#usda 1.0
def PhysicsRevoluteJoint "elbow_joint" {}
""")
        v = run_validators(stage, "rep0158:JointLimits")
        assert sum(1 for x in v if x.GetName() == MISSING_JOINT_LIMITS) == 2

    def test_revolute_joint_missing_lower_limit_gives_error(self):
        stage = make_stage("""
#usda 1.0
def PhysicsRevoluteJoint "elbow_joint" {
    float physics:upperLimit = 3.14
}
""")
        v = run_validators(stage, "rep0158:JointLimits")
        assert has(v, MISSING_JOINT_LIMITS)
        assert any("lowerLimit" in x.GetMessage() for x in v)

    def test_revolute_joint_missing_upper_limit_gives_error(self):
        stage = make_stage("""
#usda 1.0
def PhysicsRevoluteJoint "elbow_joint" {
    float physics:lowerLimit = -3.14
}
""")
        v = run_validators(stage, "rep0158:JointLimits")
        assert has(v, MISSING_JOINT_LIMITS)
        assert any("upperLimit" in x.GetMessage() for x in v)

    def test_revolute_joint_with_both_limits_no_violation(self):
        stage = make_stage("""
#usda 1.0
def PhysicsRevoluteJoint "elbow_joint" {
    float physics:lowerLimit = -1.57
    float physics:upperLimit = 1.57
}
""")
        assert none_with(run_validators(stage, "rep0158:JointLimits"), MISSING_JOINT_LIMITS)

    def test_prismatic_joint_missing_limits_gives_errors(self):
        stage = make_stage("""
#usda 1.0
def PhysicsPrismaticJoint "slide_joint" {}
""")
        v = run_validators(stage, "rep0158:JointLimits")
        assert sum(1 for x in v if x.GetName() == MISSING_JOINT_LIMITS) == 2

    def test_prismatic_joint_with_limits_no_violation(self):
        stage = make_stage("""
#usda 1.0
def PhysicsPrismaticJoint "slide_joint" {
    float physics:lowerLimit = 0.0
    float physics:upperLimit = 0.5
}
""")
        assert none_with(run_validators(stage, "rep0158:JointLimits"), MISSING_JOINT_LIMITS)

    def test_fixed_joint_no_limits_required(self):
        """Fixed joints do not have positional degrees of freedom – no limit check."""
        stage = make_stage("""
#usda 1.0
def PhysicsFixedJoint "world_joint" {}
""")
        assert none_with(run_validators(stage, "rep0158:JointLimits"), MISSING_JOINT_LIMITS)


# ------------------------------------------------------------------ #
# §1.3.2 – ArticulationRoot count                                      #
# ------------------------------------------------------------------ #


class TestArticulationRoot:
    def test_single_root_no_violation(self):
        stage = make_stage("""
#usda 1.0
def Xform "Robot" (
    prepend apiSchemas = ["PhysicsArticulationRootAPI"]
) {}
""")
        assert none_with(run_validators(stage, "rep0158:ArticulationRoot"), MULTIPLE_ARTICULATION_ROOTS)

    def test_two_roots_gives_warning(self):
        stage = make_stage("""
#usda 1.0
def Xform "Robot" (
    prepend apiSchemas = ["PhysicsArticulationRootAPI"]
) {}

def Xform "Gripper" (
    prepend apiSchemas = ["PhysicsArticulationRootAPI"]
) {}
""")
        assert has(run_validators(stage, "rep0158:ArticulationRoot"), MULTIPLE_ARTICULATION_ROOTS)

    def test_no_articulation_root_no_violation(self):
        stage = make_stage("""
#usda 1.0
def Xform "Robot" {}
""")
        assert none_with(run_validators(stage, "rep0158:ArticulationRoot"), MULTIPLE_ARTICULATION_ROOTS)


# ------------------------------------------------------------------ #
# §1.3.3 – Mass properties (zero mass)                                 #
# ------------------------------------------------------------------ #


class TestMassProperties:
    def test_zero_mass_rigid_body_gives_warning(self):
        stage = make_stage("""
#usda 1.0
def Xform "Base" (
    prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI"]
) {
    float physics:mass = 0.0
}
""")
        assert has(run_validators(stage, "rep0158:MassProperties"), NON_POSITIVE_MASS)

    def test_positive_mass_rigid_body_no_violation(self):
        stage = make_stage("""
#usda 1.0
def Xform "Base" (
    prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI"]
) {
    float physics:mass = 5.0
}
""")
        assert none_with(run_validators(stage, "rep0158:MassProperties"), NON_POSITIVE_MASS)

    def test_rigid_body_without_mass_api_no_violation(self):
        """RigidBodyAPI without MassAPI – no mass to check."""
        stage = make_stage("""
#usda 1.0
def Xform "Base" (
    prepend apiSchemas = ["PhysicsRigidBodyAPI"]
) {}
""")
        assert none_with(run_validators(stage, "rep0158:MassProperties"), NON_POSITIVE_MASS)

    def test_non_zero_mass_various_values_no_violation(self):
        for mass in ("0.001", "100.0", "1e3"):
            stage = make_stage(f"""
#usda 1.0
def Xform "Base" (
    prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI"]
) {{
    float physics:mass = {mass}
}}
""")
            assert none_with(run_validators(stage, "rep0158:MassProperties"), NON_POSITIVE_MASS), (
                f"Unexpected violation for mass={mass}"
            )

    def test_negative_mass_gives_warning(self):
        stage = make_stage("""
#usda 1.0
def Xform "Base" (
    prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI"]
) {
    float physics:mass = -1.0
}
""")
        assert has(run_validators(stage, "rep0158:MassProperties"), NON_POSITIVE_MASS)


# ------------------------------------------------------------------ #
# §1.3.5 – Collision material binding                                  #
# ------------------------------------------------------------------ #


class TestCollisionMaterial:
    def test_collision_without_material_binding_gives_warning(self):
        stage = make_stage("""
#usda 1.0
def Mesh "Collider" (
    prepend apiSchemas = ["PhysicsCollisionAPI"]
) {
    int[] faceVertexCounts = [3]
    int[] faceVertexIndices = [0, 1, 2]
    point3f[] points = [(0,0,0), (1,0,0), (0,1,0)]
}
""")
        assert has(run_validators(stage, "rep0158:CollisionMaterial"), COLLISION_MISSING_MATERIAL)

    def test_collision_with_complete_material_no_violation(self):
        stage = make_stage("""
#usda 1.0
def Mesh "Collider" (
    prepend apiSchemas = ["PhysicsCollisionAPI"]
) {
    int[] faceVertexCounts = [3]
    int[] faceVertexIndices = [0, 1, 2]
    point3f[] points = [(0,0,0), (1,0,0), (0,1,0)]
    rel material:binding:physics = </PhysMat>
}

def Material "PhysMat" (
    prepend apiSchemas = ["PhysicsMaterialAPI"]
) {
    float physics:staticFriction = 0.5
    float physics:dynamicFriction = 0.3
    float physics:restitution = 0.0
}
""")
        assert none_with(run_validators(stage, "rep0158:CollisionMaterial"), COLLISION_MISSING_MATERIAL)

    def test_material_missing_restitution_gives_warning(self):
        stage = make_stage("""
#usda 1.0
def Mesh "Collider" (
    prepend apiSchemas = ["PhysicsCollisionAPI"]
) {
    int[] faceVertexCounts = [3]
    int[] faceVertexIndices = [0, 1, 2]
    point3f[] points = [(0,0,0), (1,0,0), (0,1,0)]
    rel material:binding:physics = </PhysMat>
}

def Material "PhysMat" (
    prepend apiSchemas = ["PhysicsMaterialAPI"]
) {
    float physics:staticFriction = 0.5
    float physics:dynamicFriction = 0.3
}
""")
        v = run_validators(stage, "rep0158:CollisionMaterial")
        assert has(v, COLLISION_MISSING_MATERIAL)
        assert any("restitution" in x.GetMessage() for x in v)


# ------------------------------------------------------------------ #
# §1.3.4 – Collision geometry authoring                               #
# ------------------------------------------------------------------ #


class TestCollisionGeometryAuthoring:
    def test_collision_without_guide_purpose_gives_warning(self):
        stage = make_stage("""
#usda 1.0
def Mesh "Collider" (
    prepend apiSchemas = ["PhysicsCollisionAPI"]
) {
    token physics:approximation = "none"
    int[] faceVertexCounts = [3]
    int[] faceVertexIndices = [0, 1, 2]
    point3f[] points = [(0,0,0), (1,0,0), (0,1,0)]
}
""")
        assert has(run_validators(stage, "rep0158:CollisionGeometryAuthoring"), COLLISION_GEOMETRY_AUTHORING)

    def test_collision_without_none_approximation_gives_warning(self):
        stage = make_stage("""
#usda 1.0
def Mesh "Collider" (
    prepend apiSchemas = ["PhysicsCollisionAPI"]
) {
    token purpose = "guide"
    token physics:approximation = "convexHull"
    int[] faceVertexCounts = [3]
    int[] faceVertexIndices = [0, 1, 2]
    point3f[] points = [(0,0,0), (1,0,0), (0,1,0)]
}
""")
        assert has(run_validators(stage, "rep0158:CollisionGeometryAuthoring"), COLLISION_GEOMETRY_AUTHORING)

    def test_collision_with_guide_and_none_no_violation(self):
        stage = make_stage("""
#usda 1.0
def Mesh "Collider" (
    prepend apiSchemas = ["PhysicsCollisionAPI"]
) {
    token purpose = "guide"
    token physics:approximation = "none"
    int[] faceVertexCounts = [3]
    int[] faceVertexIndices = [0, 1, 2]
    point3f[] points = [(0,0,0), (1,0,0), (0,1,0)]
}
""")
        assert none_with(run_validators(stage, "rep0158:CollisionGeometryAuthoring"), COLLISION_GEOMETRY_AUTHORING)

    def test_non_collision_mesh_not_flagged(self):
        stage = make_stage("""
#usda 1.0
def Mesh "Visual" {
    token purpose = "default"
    int[] faceVertexCounts = [3]
    int[] faceVertexIndices = [0, 1, 2]
    point3f[] points = [(0,0,0), (1,0,0), (0,1,0)]
}
""")
        assert none_with(run_validators(stage, "rep0158:CollisionGeometryAuthoring"), COLLISION_GEOMETRY_AUTHORING)


# ------------------------------------------------------------------ #
# §1.3.7 – MimicJointAPI type constraint                               #
# ------------------------------------------------------------------ #


class TestMimicJoint:
    def test_mimic_on_fixed_joint_gives_warning(self):
        stage = make_stage("""
#usda 1.0
def PhysicsFixedJoint "fixed" (
    prepend apiSchemas = ["MimicJointAPI"]
) {}
""")
        assert has(run_validators(stage, "rep0158:MimicJoint"), MIMIC_JOINT_CONSTRAINT)

    def test_mimic_on_revolute_no_violation(self):
        stage = make_stage("""
#usda 1.0
def PhysicsRevoluteJoint "source" {
    float physics:lowerLimit = -1.0
    float physics:upperLimit = 1.0
}
def PhysicsRevoluteJoint "finger" (
    prepend apiSchemas = ["MimicJointAPI"]
) {
    float physics:lowerLimit = -1.0
    float physics:upperLimit = 1.0
    rel mimic:joint = </source>
}
""")
        assert none_with(run_validators(stage, "rep0158:MimicJoint"), MIMIC_JOINT_CONSTRAINT)

    def test_mimic_on_prismatic_no_violation(self):
        stage = make_stage("""
#usda 1.0
def PhysicsPrismaticJoint "source" {
    float physics:lowerLimit = 0.0
    float physics:upperLimit = 0.1
}
def PhysicsPrismaticJoint "slide" (
    prepend apiSchemas = ["MimicJointAPI"]
) {
    float physics:lowerLimit = 0.0
    float physics:upperLimit = 0.1
    rel mimic:joint = </source>
}
""")
        assert none_with(run_validators(stage, "rep0158:MimicJoint"), MIMIC_JOINT_CONSTRAINT)

    def test_mimic_missing_relationship_gives_error(self):
        stage = make_stage("""
#usda 1.0
def PhysicsRevoluteJoint "source" {
    float physics:lowerLimit = -1.0
    float physics:upperLimit = 1.0
}
def PhysicsRevoluteJoint "follower" (
    prepend apiSchemas = ["MimicJointAPI"]
) {
    float physics:lowerLimit = -1.0
    float physics:upperLimit = 1.0
}
""")
        assert has(run_validators(stage, "rep0158:MimicJoint"), MIMIC_JOINT_CONSTRAINT)

    def test_mimic_cycle_gives_error(self):
        stage = make_stage("""
#usda 1.0
def PhysicsRevoluteJoint "joint_a" (
    prepend apiSchemas = ["MimicJointAPI"]
) {
    float physics:lowerLimit = -1.0
    float physics:upperLimit = 1.0
    rel mimic:joint = </joint_b>
}
def PhysicsRevoluteJoint "joint_b" (
    prepend apiSchemas = ["MimicJointAPI"]
) {
    float physics:lowerLimit = -1.0
    float physics:upperLimit = 1.0
    rel mimic:joint = </joint_a>
}
""")
        assert has(run_validators(stage, "rep0158:MimicJoint"), MIMIC_JOINT_CONSTRAINT)

    def test_mimic_joint_api_deprecated_gives_warning(self):
        """Any use of MimicJointAPI must produce a 1.3.9 deprecation warning."""
        stage = make_stage("""
#usda 1.0
def PhysicsRevoluteJoint "source" {
    float physics:lowerLimit = -1.0
    float physics:upperLimit = 1.0
}
def PhysicsRevoluteJoint "follower" (
    prepend apiSchemas = ["MimicJointAPI"]
) {
    float physics:lowerLimit = -1.0
    float physics:upperLimit = 1.0
    rel mimic:joint = </source>
}
""")
        assert has(run_validators(stage, "rep0158:MimicJoint"), MIMIC_JOINT_DEPRECATED)


# ------------------------------------------------------------------ #
# §1.3.8 – instanceable on physics/ROS prims                           #
# ------------------------------------------------------------------ #


class TestInstanceablePhysics:
    def test_rigid_body_prim_instanceable_gives_error(self):
        stage = make_stage("""
#usda 1.0
def Xform "Robot" (
    prepend apiSchemas = ["PhysicsRigidBodyAPI"]
    instanceable = true
) {}
""")
        assert has(run_validators(stage, "rep0158:InstanceablePhysics"), INSTANCEABLE_PHYSICS)

    def test_ros_topic_prim_instanceable_gives_error(self):
        stage = make_stage("""
#usda 1.0
def Xform "Interface" (
    prepend apiSchemas = ["RosTopicAPI"]
    instanceable = true
) {}
""")
        assert has(run_validators(stage, "rep0158:InstanceablePhysics"), INSTANCEABLE_PHYSICS)

    def test_joint_prim_instanceable_gives_error(self):
        stage = make_stage("""
#usda 1.0
def PhysicsRevoluteJoint "arm_joint" (
    instanceable = true
) {
    float physics:lowerLimit = -1.0
    float physics:upperLimit = 1.0
}
""")
        assert has(run_validators(stage, "rep0158:InstanceablePhysics"), INSTANCEABLE_PHYSICS)

    def test_visual_mesh_instanceable_no_violation(self):
        """Pure visual geometry with instanceable=true is permitted (§3.5)."""
        stage = make_stage("""
#usda 1.0
def Mesh "BoltMesh" (
    instanceable = true
) {
    int[] faceVertexCounts = [3]
    int[] faceVertexIndices = [0, 1, 2]
    point3f[] points = [(0,0,0), (1,0,0), (0,1,0)]
}
""")
        assert none_with(run_validators(stage, "rep0158:InstanceablePhysics"), INSTANCEABLE_PHYSICS)
