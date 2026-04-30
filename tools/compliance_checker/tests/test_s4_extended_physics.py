"""Tests for §4.2 – Extended Physics Schemas.

Check IDs covered:
  4.2.1  ExtendedPhysicsMimicAPI must only be on revolute or prismatic joints
  4.2.2  ExtendedPhysicsMimicAPI must have a valid ext_physics:mimic:joint relationship
  4.2.3  ExtendedPhysicsMimicAPI relationships must form a DAG (no cycles)
  4.2.4  ExtendedPhysicsActuatorAPI must have authored ext_physics:actuator:targets
  4.2.5  ExtendedPhysicsPositionBasedClampingAPI lookup table must be valid
"""

from .conftest import has, make_stage, none_with, run_validators

# ------------------------------------------------------------------ #
# §4.2.1 / 4.2.2 / 4.2.3 – ExtendedPhysicsMimicAPI                   #
# ------------------------------------------------------------------ #


class TestExtendedPhysicsMimic:
    def test_mimic_on_fixed_joint_gives_warning(self):
        stage = make_stage("""
#usda 1.0
def PhysicsFixedJoint "fixed" (
    prepend apiSchemas = ["ExtendedPhysicsMimicAPI"]
) {}
""")
        assert has(run_validators(stage, "rep0158:ExtendedPhysicsMimic"), "4.2.1")

    def test_mimic_on_revolute_with_valid_rel_no_violation(self):
        stage = make_stage("""
#usda 1.0
def PhysicsRevoluteJoint "source" {
    float physics:lowerLimit = -1.0
    float physics:upperLimit = 1.0
}
def PhysicsRevoluteJoint "follower" (
    prepend apiSchemas = ["ExtendedPhysicsMimicAPI"]
) {
    float physics:lowerLimit = -1.0
    float physics:upperLimit = 1.0
    rel ext_physics:mimic:joint = </source>
}
""")
        v = run_validators(stage, "rep0158:ExtendedPhysicsMimic")
        assert none_with(v, "4.2.1")
        assert none_with(v, "4.2.2")
        assert none_with(v, "4.2.3")

    def test_mimic_on_prismatic_with_valid_rel_no_violation(self):
        stage = make_stage("""
#usda 1.0
def PhysicsPrismaticJoint "source" {
    float physics:lowerLimit = 0.0
    float physics:upperLimit = 0.1
}
def PhysicsPrismaticJoint "slide" (
    prepend apiSchemas = ["ExtendedPhysicsMimicAPI"]
) {
    float physics:lowerLimit = 0.0
    float physics:upperLimit = 0.1
    rel ext_physics:mimic:joint = </source>
}
""")
        v = run_validators(stage, "rep0158:ExtendedPhysicsMimic")
        assert none_with(v, "4.2.1")
        assert none_with(v, "4.2.2")

    def test_mimic_missing_relationship_gives_error(self):
        stage = make_stage("""
#usda 1.0
def PhysicsRevoluteJoint "follower" (
    prepend apiSchemas = ["ExtendedPhysicsMimicAPI"]
) {
    float physics:lowerLimit = -1.0
    float physics:upperLimit = 1.0
}
""")
        assert has(run_validators(stage, "rep0158:ExtendedPhysicsMimic"), "4.2.2")

    def test_mimic_relationship_targeting_nonexistent_prim_gives_error(self):
        stage = make_stage("""
#usda 1.0
def PhysicsRevoluteJoint "follower" (
    prepend apiSchemas = ["ExtendedPhysicsMimicAPI"]
) {
    rel ext_physics:mimic:joint = </nonexistent>
}
""")
        assert has(run_validators(stage, "rep0158:ExtendedPhysicsMimic"), "4.2.2")

    def test_mimic_targeting_non_joint_gives_error(self):
        stage = make_stage("""
#usda 1.0
def Xform "notajoint" {}
def PhysicsRevoluteJoint "follower" (
    prepend apiSchemas = ["ExtendedPhysicsMimicAPI"]
) {
    rel ext_physics:mimic:joint = </notajoint>
}
""")
        assert has(run_validators(stage, "rep0158:ExtendedPhysicsMimic"), "4.2.2")

    def test_mimic_cycle_gives_error(self):
        stage = make_stage("""
#usda 1.0
def PhysicsRevoluteJoint "joint_a" (
    prepend apiSchemas = ["ExtendedPhysicsMimicAPI"]
) {
    float physics:lowerLimit = -1.0
    float physics:upperLimit = 1.0
    rel ext_physics:mimic:joint = </joint_b>
}
def PhysicsRevoluteJoint "joint_b" (
    prepend apiSchemas = ["ExtendedPhysicsMimicAPI"]
) {
    float physics:lowerLimit = -1.0
    float physics:upperLimit = 1.0
    rel ext_physics:mimic:joint = </joint_a>
}
""")
        assert has(run_validators(stage, "rep0158:ExtendedPhysicsMimic"), "4.2.3")

    def test_mimic_chain_no_cycle_no_violation(self):
        stage = make_stage("""
#usda 1.0
def PhysicsRevoluteJoint "root" {
    float physics:lowerLimit = -1.0
    float physics:upperLimit = 1.0
}
def PhysicsRevoluteJoint "mid" (
    prepend apiSchemas = ["ExtendedPhysicsMimicAPI"]
) {
    float physics:lowerLimit = -1.0
    float physics:upperLimit = 1.0
    rel ext_physics:mimic:joint = </root>
}
def PhysicsRevoluteJoint "leaf" (
    prepend apiSchemas = ["ExtendedPhysicsMimicAPI"]
) {
    float physics:lowerLimit = -1.0
    float physics:upperLimit = 1.0
    rel ext_physics:mimic:joint = </mid>
}
""")
        v = run_validators(stage, "rep0158:ExtendedPhysicsMimic")
        assert none_with(v, "4.2.3")


# ------------------------------------------------------------------ #
# §4.2.4 – ExtendedPhysicsActuatorAPI                                  #
# ------------------------------------------------------------------ #


class TestExtendedPhysicsActuator:
    def test_actuator_missing_targets_gives_error(self):
        stage = make_stage("""
#usda 1.0
def Xform "actuator" (
    prepend apiSchemas = ["ExtendedPhysicsActuatorAPI"]
) {}
""")
        assert has(run_validators(stage, "rep0158:ExtendedPhysicsActuator"), "4.2.4")

    def test_actuator_targeting_revolute_no_violation(self):
        stage = make_stage("""
#usda 1.0
def PhysicsRevoluteJoint "elbow" {
    float physics:lowerLimit = -1.57
    float physics:upperLimit = 1.57
}
def Xform "actuator" (
    prepend apiSchemas = ["ExtendedPhysicsActuatorAPI"]
) {
    rel ext_physics:actuator:targets = </elbow>
}
""")
        assert none_with(run_validators(stage, "rep0158:ExtendedPhysicsActuator"), "4.2.4")

    def test_actuator_targeting_prismatic_no_violation(self):
        stage = make_stage("""
#usda 1.0
def PhysicsPrismaticJoint "slide" {
    float physics:lowerLimit = 0.0
    float physics:upperLimit = 0.5
}
def Xform "actuator" (
    prepend apiSchemas = ["ExtendedPhysicsActuatorAPI"]
) {
    rel ext_physics:actuator:targets = </slide>
}
""")
        assert none_with(run_validators(stage, "rep0158:ExtendedPhysicsActuator"), "4.2.4")

    def test_actuator_targeting_nonexistent_prim_gives_error(self):
        stage = make_stage("""
#usda 1.0
def Xform "actuator" (
    prepend apiSchemas = ["ExtendedPhysicsActuatorAPI"]
) {
    rel ext_physics:actuator:targets = </ghost>
}
""")
        assert has(run_validators(stage, "rep0158:ExtendedPhysicsActuator"), "4.2.4")

    def test_actuator_targeting_fixed_joint_gives_error(self):
        stage = make_stage("""
#usda 1.0
def PhysicsFixedJoint "base" {}
def Xform "actuator" (
    prepend apiSchemas = ["ExtendedPhysicsActuatorAPI"]
) {
    rel ext_physics:actuator:targets = </base>
}
""")
        assert has(run_validators(stage, "rep0158:ExtendedPhysicsActuator"), "4.2.4")

    def test_actuator_targeting_xform_gives_error(self):
        stage = make_stage("""
#usda 1.0
def Xform "link" {}
def Xform "actuator" (
    prepend apiSchemas = ["ExtendedPhysicsActuatorAPI"]
) {
    rel ext_physics:actuator:targets = </link>
}
""")
        assert has(run_validators(stage, "rep0158:ExtendedPhysicsActuator"), "4.2.4")


# ------------------------------------------------------------------ #
# §4.2.5 – ExtendedPhysicsPositionBasedClampingAPI                     #
# ------------------------------------------------------------------ #


class TestExtendedPhysicsPositionClamping:
    def test_valid_lookup_table_no_violation(self):
        stage = make_stage("""
#usda 1.0
def Xform "clamp" (
    prepend apiSchemas = ["ExtendedPhysicsPositionBasedClampingAPI"]
) {
    float[] ext_physics:clamp_position:lookupPositions = [-1.0, 0.0, 1.0]
    float[] ext_physics:clamp_position:lookupEfforts = [10.0, 20.0, 10.0]
}
""")
        assert none_with(
            run_validators(stage, "rep0158:ExtendedPhysicsPositionClamping"), "4.2.5"
        )

    def test_mismatched_lengths_gives_error(self):
        stage = make_stage("""
#usda 1.0
def Xform "clamp" (
    prepend apiSchemas = ["ExtendedPhysicsPositionBasedClampingAPI"]
) {
    float[] ext_physics:clamp_position:lookupPositions = [-1.0, 0.0, 1.0]
    float[] ext_physics:clamp_position:lookupEfforts = [10.0, 20.0]
}
""")
        assert has(run_validators(stage, "rep0158:ExtendedPhysicsPositionClamping"), "4.2.5")

    def test_non_monotonic_positions_gives_error(self):
        stage = make_stage("""
#usda 1.0
def Xform "clamp" (
    prepend apiSchemas = ["ExtendedPhysicsPositionBasedClampingAPI"]
) {
    float[] ext_physics:clamp_position:lookupPositions = [-1.0, 1.0, 0.0]
    float[] ext_physics:clamp_position:lookupEfforts = [10.0, 20.0, 15.0]
}
""")
        assert has(run_validators(stage, "rep0158:ExtendedPhysicsPositionClamping"), "4.2.5")

    def test_duplicate_position_values_gives_error(self):
        stage = make_stage("""
#usda 1.0
def Xform "clamp" (
    prepend apiSchemas = ["ExtendedPhysicsPositionBasedClampingAPI"]
) {
    float[] ext_physics:clamp_position:lookupPositions = [0.0, 0.0, 1.0]
    float[] ext_physics:clamp_position:lookupEfforts = [10.0, 10.0, 5.0]
}
""")
        assert has(run_validators(stage, "rep0158:ExtendedPhysicsPositionClamping"), "4.2.5")

    def test_empty_arrays_no_violation(self):
        stage = make_stage("""
#usda 1.0
def Xform "clamp" (
    prepend apiSchemas = ["ExtendedPhysicsPositionBasedClampingAPI"]
) {}
""")
        assert none_with(
            run_validators(stage, "rep0158:ExtendedPhysicsPositionClamping"), "4.2.5"
        )

    def test_single_entry_no_violation(self):
        stage = make_stage("""
#usda 1.0
def Xform "clamp" (
    prepend apiSchemas = ["ExtendedPhysicsPositionBasedClampingAPI"]
) {
    float[] ext_physics:clamp_position:lookupPositions = [0.5]
    float[] ext_physics:clamp_position:lookupEfforts = [100.0]
}
""")
        assert none_with(
            run_validators(stage, "rep0158:ExtendedPhysicsPositionClamping"), "4.2.5"
        )
