"""Tests for §1.1 – Coordinate Systems & Units.

Check IDs covered:
  1.1.1  metersPerUnit must be 1.0
  1.1.2  kilogramsPerUnit must be set and equal 1.0
  1.1.3  upAxis must be "Z"
  1.1.4  root prim must not have rotation xformOps
  1.1.5  timeCodesPerSecond must be 1.0
  1.1.6  kinematic prim transform stack must be translate+orient only
  1.1.7  kinematic prim scale must be identity if authored
"""

from compliance_checker.checks._tokens import (
    KINEMATIC_NON_IDENTITY_SCALE,
    KINEMATIC_TRANSFORM_OPS,
    ROOT_HAS_ROTATION,
    WRONG_KILOGRAMS_PER_UNIT,
    WRONG_METERS_PER_UNIT,
    WRONG_TIME_CODES_PER_SECOND,
    WRONG_UP_AXIS,
)

from .conftest import has, make_stage, none_with, run_validators

_V = "rep0158:CoordinateSystem"

# ------------------------------------------------------------------ #
# §1.1.1 – metersPerUnit                                               #
# ------------------------------------------------------------------ #


class TestMetersPerUnit:
    def test_missing_gives_error(self):
        """Default metersPerUnit is 0.01 (centimetres) – must report 1.1.1."""
        stage = make_stage('#usda 1.0\ndef Xform "Root" {}')
        assert has(run_validators(stage, _V), WRONG_METERS_PER_UNIT)

    def test_wrong_value_gives_error(self):
        stage = make_stage("""
#usda 1.0
(
    metersPerUnit = 0.01
)
def Xform "Root" {}
""")
        assert has(run_validators(stage, _V), WRONG_METERS_PER_UNIT)

    def test_correct_value_no_violation(self):
        stage = make_stage("""
#usda 1.0
(
    metersPerUnit = 1
)
def Xform "Root" {}
""")
        assert none_with(run_validators(stage, _V), WRONG_METERS_PER_UNIT)


# ------------------------------------------------------------------ #
# §1.1.2 – kilogramsPerUnit                                            #
# ------------------------------------------------------------------ #


class TestKilogramsPerUnit:
    def test_explicitly_wrong_value_gives_error(self):
        """kilogramsPerUnit has a schema default of 1.0, so only explicit wrong values matter."""
        stage = make_stage("""
#usda 1.0
(
    kilogramsPerUnit = 0.001
)
def Xform "Root" {}
""")
        assert has(run_validators(stage, _V), WRONG_KILOGRAMS_PER_UNIT)

    def test_wrong_value_gives_error(self):
        stage = make_stage("""
#usda 1.0
(
    kilogramsPerUnit = 0.001
)
def Xform "Root" {}
""")
        assert has(run_validators(stage, _V), WRONG_KILOGRAMS_PER_UNIT)

    def test_correct_value_no_violation(self):
        stage = make_stage("""
#usda 1.0
(
    kilogramsPerUnit = 1
)
def Xform "Root" {}
""")
        assert none_with(run_validators(stage, _V), WRONG_KILOGRAMS_PER_UNIT)


# ------------------------------------------------------------------ #
# §1.1.3 – upAxis                                                      #
# ------------------------------------------------------------------ #


class TestUpAxis:
    def test_default_y_up_gives_error(self):
        """OpenUSD default is Y-up; must report 1.1.3."""
        stage = make_stage('#usda 1.0\ndef Xform "Root" {}')
        assert has(run_validators(stage, _V), WRONG_UP_AXIS)

    def test_explicit_y_gives_error(self):
        stage = make_stage("""
#usda 1.0
(
    upAxis = "Y"
)
def Xform "Root" {}
""")
        assert has(run_validators(stage, _V), WRONG_UP_AXIS)

    def test_z_up_no_violation(self):
        stage = make_stage("""
#usda 1.0
(
    upAxis = "Z"
)
def Xform "Root" {}
""")
        assert none_with(run_validators(stage, _V), WRONG_UP_AXIS)


# ------------------------------------------------------------------ #
# §1.1.4 – root rotation xformOp                                       #
# ------------------------------------------------------------------ #


class TestRootRotation:
    def test_rotate_x_on_default_prim_gives_warning(self):
        stage = make_stage("""
#usda 1.0
(
    defaultPrim = "Robot"
    upAxis = "Z"
    metersPerUnit = 1
    kilogramsPerUnit = 1
)
def Xform "Robot" {
    float xformOp:rotateX = -90
    uniform token[] xformOpOrder = ["xformOp:rotateX"]
}
""")
        assert has(run_validators(stage, _V), ROOT_HAS_ROTATION)

    def test_orient_op_on_default_prim_gives_warning(self):
        stage = make_stage("""
#usda 1.0
(
    defaultPrim = "Robot"
    upAxis = "Z"
    metersPerUnit = 1
    kilogramsPerUnit = 1
)
def Xform "Robot" {
    quatf xformOp:orient = (1, 0, 0, 0)
    uniform token[] xformOpOrder = ["xformOp:orient"]
}
""")
        assert has(run_validators(stage, _V), ROOT_HAS_ROTATION)

    def test_translate_only_no_violation(self):
        """A translate-only xformOp on the root must not trigger 1.1.4."""
        stage = make_stage("""
#usda 1.0
(
    defaultPrim = "Robot"
    upAxis = "Z"
    metersPerUnit = 1
    kilogramsPerUnit = 1
)
def Xform "Robot" {
    float3 xformOp:translate = (0, 0, 0)
    uniform token[] xformOpOrder = ["xformOp:translate"]
}
""")
        assert none_with(run_validators(stage, _V), ROOT_HAS_ROTATION)

    def test_no_xform_no_violation(self):
        stage = make_stage("""
#usda 1.0
(
    defaultPrim = "Robot"
    upAxis = "Z"
    metersPerUnit = 1
    kilogramsPerUnit = 1
)
def Xform "Robot" {}
""")
        assert none_with(run_validators(stage, _V), ROOT_HAS_ROTATION)

    def test_rotation_on_non_default_prim_not_flagged(self):
        """Only the defaultPrim root is checked; child rotations are fine."""
        stage = make_stage("""
#usda 1.0
(
    defaultPrim = "Robot"
    upAxis = "Z"
    metersPerUnit = 1
    kilogramsPerUnit = 1
)
def Xform "Robot" {
    def Xform "Child" {
        float xformOp:rotateX = -90
        uniform token[] xformOpOrder = ["xformOp:rotateX"]
    }
}
""")
        assert none_with(run_validators(stage, _V), ROOT_HAS_ROTATION)


class TestTimeCodesPerSecond:
    def test_default_time_codes_per_second_gives_error(self):
        stage = make_stage('#usda 1.0\ndef Xform "Root" {}')
        assert has(run_validators(stage, _V), WRONG_TIME_CODES_PER_SECOND)

    def test_time_codes_per_second_set_to_one_no_violation(self):
        stage = make_stage("""
#usda 1.0
(
    timeCodesPerSecond = 1
)
def Xform "Root" {}
""")
        assert none_with(run_validators(stage, _V), WRONG_TIME_CODES_PER_SECOND)


class TestKinematicTransformOps:
    def test_rigid_body_with_matrix_op_gives_error(self):
        stage = make_stage("""
#usda 1.0
def Xform "Body" (
    prepend apiSchemas = ["PhysicsRigidBodyAPI"]
) {
    matrix4d xformOp:transform = ((1,0,0,0), (0,1,0,0), (0,0,1,0), (0,0,0,1))
    uniform token[] xformOpOrder = ["xformOp:transform"]
}
""")
        assert has(run_validators(stage, _V), KINEMATIC_TRANSFORM_OPS)

    def test_rigid_body_with_translate_orient_only_no_violation(self):
        stage = make_stage("""
#usda 1.0
def Xform "Body" (
    prepend apiSchemas = ["PhysicsRigidBodyAPI"]
) {
    float3 xformOp:translate = (0, 0, 0)
    quatf xformOp:orient = (1, 0, 0, 0)
    uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:orient"]
}
""")
        assert none_with(run_validators(stage, _V), KINEMATIC_TRANSFORM_OPS)


class TestKinematicScale:
    def test_non_identity_scale_on_rigid_body_gives_error(self):
        stage = make_stage("""
#usda 1.0
def Xform "Body" (
    prepend apiSchemas = ["PhysicsRigidBodyAPI"]
) {
    float3 xformOp:translate = (0, 0, 0)
    quatf xformOp:orient = (1, 0, 0, 0)
    float3 xformOp:scale = (1, 2, 1)
    uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:orient", "xformOp:scale"]
}
""")
        assert has(run_validators(stage, _V), KINEMATIC_NON_IDENTITY_SCALE)

    def test_identity_scale_on_rigid_body_no_violation(self):
        stage = make_stage("""
#usda 1.0
def Xform "Body" (
    prepend apiSchemas = ["PhysicsRigidBodyAPI"]
) {
    float3 xformOp:translate = (0, 0, 0)
    quatf xformOp:orient = (1, 0, 0, 0)
    float3 xformOp:scale = (1, 1, 1)
    uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:orient", "xformOp:scale"]
}
""")
        assert none_with(run_validators(stage, _V), KINEMATIC_NON_IDENTITY_SCALE)
