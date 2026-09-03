"""Tests for §2 – ROS Integration Schemas.

Check IDs covered:
  2.1.1   RosContextAPI must not be inside a payload arc
  2.1.2   Nested RosContextAPI must not set ros:context:parent_frame
  2.1.3   ros:context:namespace must follow §2.1.1 namespace rules
  2.2.1   RosTopicAPI / RosServiceAPI / RosActionAPI must not be inside a payload arc
  2.3.1   ros:*:name values must follow ROS 2 naming rules
  2.4.1   RosTopicAPI role must be set and valid
  2.4.2   RosTopicAPI name must be set
  2.4.3   RosTopicAPI type must be set
  2.4.4   Publisher RosTopicAPI must set publish_rate
  2.4.5   QoS token values must be within allowed sets
  2.4.6   ros:topic:starts_enabled must be bool
  2.4.7   ros:topic:override_frame_id must be valid
  2.4.8   ros:topic:qos:match_publisher must be bool and subscription-only
  2.4.9   ros:topic:qos:depth must be positive integer for keep_last
  2.5.1   RosServiceAPI required attributes
  2.5.2   ros:service:starts_enabled must be bool
  2.6.1   RosActionAPI required attributes
  2.6.2   ros:action:starts_enabled must be bool
  2.7.1   RosFrameAPI + PhysicsRigidBodyAPI should not coexist
  2.8.1   Camera interface placement and optical-frame rotation
  2.9.1   /clock topic is prohibited
  2.9.2   simulation_interfaces types are prohibited
  2.10.1  UsdPhysicsJoint prims should carry ros:joint:name
"""

import pytest

from compliance_checker.checks._tokens import (
    CAMERA_OPTICAL_FRAME, CONTEXT_INSIDE_PAYLOAD, FRAME_ON_RIGID_BODY,
    INTERFACE_INSIDE_PAYLOAD, INTERFACE_ON_RIGID_BODY, INVALID_ACTION_STARTS_ENABLED,
    INVALID_CONTEXT_NAMESPACE, INVALID_FRAME_ID, INVALID_FRAME_STATIC,
    INVALID_MATCH_PUBLISHER, INVALID_OVERRIDE_FRAME_ID, INVALID_QOS_DEPTH,
    INVALID_QOS_TOKEN, INVALID_ROS_NAME, INVALID_SERVICE_STARTS_ENABLED,
    INVALID_STARTS_ENABLED, MISSING_ACTION_ATTR, MISSING_JOINT_NAME,
    MISSING_PUBLISH_RATE, MISSING_SERVICE_ATTR, MISSING_TOPIC_NAME,
    MISSING_TOPIC_ROLE, MISSING_TOPIC_TYPE, MULTIPLE_INTERFACES_PER_PRIM,
    NESTED_CONTEXT_PARENT_FRAME, PROHIBITED_INTERFACE_TYPE, PROHIBITED_TOPIC_NAME,
    SENSOR_NOT_DIRECT_XFORM_CHILD,
)

from .conftest import has, make_stage, none_with, run_validators

# ------------------------------------------------------------------ #
# Helpers for building common USDA fragments                           #
# ------------------------------------------------------------------ #

_FULL_TOPIC = """
    token ros:topic:role = "publisher"
    string ros:topic:name = "joint_states"
    string ros:topic:type = "sensor_msgs/msg/JointState"
    double ros:topic:publish_rate = 10.0
"""

_FULL_SERVICE = """
    token ros:service:role = "server"
    string ros:service:name = "set_led"
    string ros:service:type = "std_srvs/srv/SetBool"
"""

_FULL_ACTION = """
    token ros:action:role = "server"
    string ros:action:name = "follow_trajectory"
    string ros:action:type = "control_msgs/action/FollowJointTrajectory"
"""


# ------------------------------------------------------------------ #
# §2.1.1 – RosContextAPI placement (payload)                           #
# ------------------------------------------------------------------ #


class TestRosContextPlacement:
    def test_context_inside_payload_gives_warning(self):
        stage = make_stage("""
#usda 1.0
def Xform "Robot" (
    payload = @./body.usda@
) {
    def Xform "SensorHead" (
        prepend apiSchemas = ["RosContextAPI"]
    ) {
        string ros:context:namespace = "head"
    }
}
""")
        assert has(run_validators(stage, "usdRosValidators:RosContextPlacement"), CONTEXT_INSIDE_PAYLOAD)

    def test_context_outside_payload_no_violation(self):
        stage = make_stage("""
#usda 1.0
def Xform "Robot" (
    prepend apiSchemas = ["RosContextAPI"]
) {
    string ros:context:namespace = "my_robot"
}
""")
        assert none_with(run_validators(stage, "usdRosValidators:RosContextPlacement"), CONTEXT_INSIDE_PAYLOAD)

    def test_nested_context_with_parent_frame_gives_warning(self):
        stage = make_stage("""
#usda 1.0
def Xform "Robot" (
    prepend apiSchemas = ["RosContextAPI"]
) {
    string ros:context:namespace = "robot"

    def Xform "Arm" (
        prepend apiSchemas = ["RosContextAPI"]
    ) {
        string ros:context:namespace = "arm"
        string ros:context:parent_frame = "world"
    }
}
""")
        assert has(run_validators(stage, "usdRosValidators:RosContextPlacement"), NESTED_CONTEXT_PARENT_FRAME)

    def test_outermost_context_with_parent_frame_no_violation(self):
        stage = make_stage("""
#usda 1.0
def Xform "Robot" (
    prepend apiSchemas = ["RosContextAPI"]
) {
    string ros:context:namespace = "robot"
    string ros:context:parent_frame = "world"
}
""")
        assert none_with(run_validators(stage, "usdRosValidators:RosContextPlacement"), NESTED_CONTEXT_PARENT_FRAME)

    def test_invalid_namespace_with_runtime_substitution_gives_error(self):
        stage = make_stage("""
#usda 1.0
def Xform "Robot" (
    prepend apiSchemas = ["RosContextAPI"]
) {
    string ros:context:namespace = "robot_{id}"
}
""")
        assert has(run_validators(stage, "usdRosValidators:RosContextPlacement"), INVALID_CONTEXT_NAMESPACE)

    def test_absolute_namespace_is_allowed(self):
        stage = make_stage("""
#usda 1.0
def Xform "Robot" (
    prepend apiSchemas = ["RosContextAPI"]
) {
    string ros:context:namespace = "/robot_1"
}
""")
        assert none_with(run_validators(stage, "usdRosValidators:RosContextPlacement"), INVALID_CONTEXT_NAMESPACE)


# ------------------------------------------------------------------ #
# §2.2.1 – Interface prim placement (payload)                          #
# ------------------------------------------------------------------ #


class TestRosInterfacePlacement:
    def test_topic_inside_payload_gives_error(self):
        stage = make_stage(f"""
#usda 1.0
def Xform "Robot" (
    payload = @./body.usda@
) {{
    def Xform "Topic" (
        prepend apiSchemas = ["RosTopicAPI"]
    ) {{
        {_FULL_TOPIC}
    }}
}}
""")
        assert has(run_validators(stage, "usdRosValidators:RosInterfacePlacement"), INTERFACE_INSIDE_PAYLOAD)

    def test_service_inside_payload_gives_error(self):
        stage = make_stage(f"""
#usda 1.0
def Xform "Robot" (
    payload = @./body.usda@
) {{
    def Xform "Svc" (
        prepend apiSchemas = ["RosServiceAPI"]
    ) {{
        {_FULL_SERVICE}
    }}
}}
""")
        assert has(run_validators(stage, "usdRosValidators:RosInterfacePlacement"), INTERFACE_INSIDE_PAYLOAD)

    def test_interface_outside_payload_no_violation(self):
        stage = make_stage(f"""
#usda 1.0
def Xform "Robot" {{
    def Xform "Topic" (
        prepend apiSchemas = ["RosTopicAPI"]
    ) {{
        {_FULL_TOPIC}
    }}
}}
""")
        assert none_with(run_validators(stage, "usdRosValidators:RosInterfacePlacement"), INTERFACE_INSIDE_PAYLOAD)


# ------------------------------------------------------------------ #
# §2.2.2 / §2.2.3 – Interface structure rules                         #
# ------------------------------------------------------------------ #


class TestRosInterfaceStructure:
    def test_interface_on_rigid_body_prim_gives_error(self):
        stage = make_stage(f"""
#usda 1.0
def Xform "link" (
    prepend apiSchemas = ["PhysicsRigidBodyAPI", "RosTopicAPI"]
) {{
    {_FULL_TOPIC}
}}
""")
        assert has(run_validators(stage, "usdRosValidators:RosInterfaceStructure"), INTERFACE_ON_RIGID_BODY)

    def test_multiple_interfaces_on_single_prim_gives_error(self):
        stage = make_stage(f"""
#usda 1.0
def Xform "iface" (
    prepend apiSchemas = ["RosTopicAPI", "RosServiceAPI"]
) {{
    {_FULL_TOPIC}
    {_FULL_SERVICE}
}}
""")
        assert has(run_validators(stage, "usdRosValidators:RosInterfaceStructure"), MULTIPLE_INTERFACES_PER_PRIM)

    def test_sensor_interface_not_direct_child_of_link_gives_warning(self):
        stage = make_stage(f"""
#usda 1.0
def Xform "link" (
    prepend apiSchemas = ["PhysicsRigidBodyAPI"]
) {{
    def Xform "sensor" {{
        def Xform "image_topic" (
            prepend apiSchemas = ["RosTopicAPI"]
        ) {{
            {_FULL_TOPIC}
        }}
    }}
}}
""")
        assert has(run_validators(stage, "usdRosValidators:RosInterfaceStructure"), SENSOR_NOT_DIRECT_XFORM_CHILD)

    def test_sensor_interface_direct_child_xform_no_violation(self):
        stage = make_stage(f"""
#usda 1.0
def Xform "link" (
    prepend apiSchemas = ["PhysicsRigidBodyAPI"]
) {{
    def Xform "image_topic" (
        prepend apiSchemas = ["RosTopicAPI"]
    ) {{
        {_FULL_TOPIC}
    }}
}}
""")
        errors = run_validators(stage, "usdRosValidators:RosInterfaceStructure")
        assert none_with(errors, INTERFACE_ON_RIGID_BODY)
        assert none_with(errors, MULTIPLE_INTERFACES_PER_PRIM)
        assert none_with(errors, SENSOR_NOT_DIRECT_XFORM_CHILD)


# ------------------------------------------------------------------ #
# §2.3.1 – ROS naming rules                                            #
# ------------------------------------------------------------------ #


class TestRosNaming:
    @pytest.mark.parametrize(
        "bad_name",
        [
            "1bad_name",  # starts with digit
            "bad name",  # contains space
            "bad//name",  # consecutive slashes
            "bad-name",  # contains hyphen
        ],
    )
    def test_invalid_topic_name_gives_error(self, bad_name):
        stage = make_stage(f"""
#usda 1.0
def Xform "T" (
    prepend apiSchemas = ["RosTopicAPI"]
) {{
    token ros:topic:role = "publisher"
    string ros:topic:name = "{bad_name}"
    string ros:topic:type = "std_msgs/msg/String"
    double ros:topic:publish_rate = 1.0
}}
""")
        assert has(run_validators(stage, "usdRosValidators:RosTopic"), INVALID_ROS_NAME)

    @pytest.mark.parametrize(
        "good_name",
        [
            "joint_states",
            "/robot/joint_states",
            "cmd_vel",
            "_private_topic",
        ],
    )
    def test_valid_topic_name_no_violation(self, good_name):
        stage = make_stage(f"""
#usda 1.0
def Xform "T" (
    prepend apiSchemas = ["RosTopicAPI"]
) {{
    token ros:topic:role = "publisher"
    string ros:topic:name = "{good_name}"
    string ros:topic:type = "std_msgs/msg/String"
    double ros:topic:publish_rate = 1.0
}}
""")
        assert none_with(run_validators(stage, "usdRosValidators:RosTopic"), INVALID_ROS_NAME)


# ------------------------------------------------------------------ #
# §2.4 – RosTopicAPI                                                   #
# ------------------------------------------------------------------ #


class TestRosTopic:
    def test_missing_role_gives_error(self):
        stage = make_stage("""
#usda 1.0
def Xform "T" (
    prepend apiSchemas = ["RosTopicAPI"]
) {
    string ros:topic:name = "joint_states"
    string ros:topic:type = "sensor_msgs/msg/JointState"
    double ros:topic:publish_rate = 10.0
}
""")
        assert has(run_validators(stage, "usdRosValidators:RosTopic"), MISSING_TOPIC_ROLE)

    def test_invalid_role_gives_error(self):
        stage = make_stage("""
#usda 1.0
def Xform "T" (
    prepend apiSchemas = ["RosTopicAPI"]
) {
    token ros:topic:role = "broadcaster"
    string ros:topic:name = "joint_states"
    string ros:topic:type = "sensor_msgs/msg/JointState"
    double ros:topic:publish_rate = 10.0
}
""")
        assert has(run_validators(stage, "usdRosValidators:RosTopic"), MISSING_TOPIC_ROLE)

    def test_missing_name_gives_error(self):
        stage = make_stage("""
#usda 1.0
def Xform "T" (
    prepend apiSchemas = ["RosTopicAPI"]
) {
    token ros:topic:role = "publisher"
    string ros:topic:type = "sensor_msgs/msg/JointState"
    double ros:topic:publish_rate = 10.0
}
""")
        assert has(run_validators(stage, "usdRosValidators:RosTopic"), MISSING_TOPIC_NAME)

    def test_missing_type_gives_error(self):
        stage = make_stage("""
#usda 1.0
def Xform "T" (
    prepend apiSchemas = ["RosTopicAPI"]
) {
    token ros:topic:role = "publisher"
    string ros:topic:name = "joint_states"
    double ros:topic:publish_rate = 10.0
}
""")
        assert has(run_validators(stage, "usdRosValidators:RosTopic"), MISSING_TOPIC_TYPE)

    def test_publisher_missing_rate_gives_error(self):
        stage = make_stage("""
#usda 1.0
def Xform "T" (
    prepend apiSchemas = ["RosTopicAPI"]
) {
    token ros:topic:role = "publisher"
    string ros:topic:name = "joint_states"
    string ros:topic:type = "sensor_msgs/msg/JointState"
}
""")
        assert has(run_validators(stage, "usdRosValidators:RosTopic"), MISSING_PUBLISH_RATE)

    def test_subscription_without_rate_no_violation(self):
        """Subscriptions do not require publish_rate (§2.4)."""
        stage = make_stage("""
#usda 1.0
def Xform "T" (
    prepend apiSchemas = ["RosTopicAPI"]
) {
    token ros:topic:role = "subscription"
    string ros:topic:name = "cmd_vel"
    string ros:topic:type = "geometry_msgs/msg/Twist"
}
""")
        assert none_with(run_validators(stage, "usdRosValidators:RosTopic"), MISSING_PUBLISH_RATE)

    def test_complete_topic_no_violation(self):
        stage = make_stage(f"""
#usda 1.0
def Xform "T" (
    prepend apiSchemas = ["RosTopicAPI"]
) {{
    {_FULL_TOPIC}
}}
""")
        v = run_validators(stage, "usdRosValidators:RosTopic")
        for cid in (
            MISSING_TOPIC_ROLE,
            MISSING_TOPIC_NAME,
            MISSING_TOPIC_TYPE,
            MISSING_PUBLISH_RATE,
            INVALID_QOS_TOKEN,
            INVALID_STARTS_ENABLED,
            INVALID_OVERRIDE_FRAME_ID,
            INVALID_MATCH_PUBLISHER,
            INVALID_QOS_DEPTH,
            INVALID_ROS_NAME,
        ):
            assert none_with(v, cid), f"Unexpected violation {cid}"

    @pytest.mark.parametrize(
        "attr,value",
        [
            ("ros:topic:qos:reliability", "turbo"),
            ("ros:topic:qos:durability", "immortal"),
            ("ros:topic:qos:history", "discard"),
        ],
    )
    def test_invalid_qos_token_gives_warning(self, attr, value):
        stage = make_stage(f"""
#usda 1.0
def Xform "T" (
    prepend apiSchemas = ["RosTopicAPI"]
) {{
    {_FULL_TOPIC}
    token {attr} = "{value}"
}}
""")
        assert has(run_validators(stage, "usdRosValidators:RosTopic"), INVALID_QOS_TOKEN)

    @pytest.mark.parametrize(
        "attr,value",
        [
            ("ros:topic:qos:reliability", "reliable"),
            ("ros:topic:qos:durability", "transient_local"),
            ("ros:topic:qos:history", "keep_all"),
        ],
    )
    def test_valid_qos_tokens_no_violation(self, attr, value):
        stage = make_stage(f"""
#usda 1.0
def Xform "T" (
    prepend apiSchemas = ["RosTopicAPI"]
) {{
    {_FULL_TOPIC}
    token {attr} = "{value}"
}}
""")
        assert none_with(run_validators(stage, "usdRosValidators:RosTopic"), INVALID_QOS_TOKEN)

    def test_topic_starts_enabled_bool_no_violation(self):
        stage = make_stage(f"""
#usda 1.0
def Xform "T" (
    prepend apiSchemas = ["RosTopicAPI"]
) {{
    {_FULL_TOPIC}
    bool ros:topic:starts_enabled = false
}}
""")
        assert none_with(run_validators(stage, "usdRosValidators:RosTopic"), INVALID_STARTS_ENABLED)

    def test_topic_starts_enabled_wrong_type_gives_error(self):
        stage = make_stage(f"""
#usda 1.0
def Xform "T" (
    prepend apiSchemas = ["RosTopicAPI"]
) {{
    {_FULL_TOPIC}
    string ros:topic:starts_enabled = "nope"
}}
""")
        assert has(run_validators(stage, "usdRosValidators:RosTopic"), INVALID_STARTS_ENABLED)

    def test_topic_override_frame_id_valid_name_no_violation(self):
        stage = make_stage(f"""
#usda 1.0
def Xform "T" (
    prepend apiSchemas = ["RosTopicAPI"]
) {{
    {_FULL_TOPIC}
    string ros:topic:override_frame_id = "map"
}}
""")
        assert none_with(run_validators(stage, "usdRosValidators:RosTopic"), INVALID_OVERRIDE_FRAME_ID)

    def test_topic_override_frame_id_invalid_name_gives_warning(self):
        stage = make_stage(f"""
#usda 1.0
def Xform "T" (
    prepend apiSchemas = ["RosTopicAPI"]
) {{
    {_FULL_TOPIC}
    string ros:topic:override_frame_id = "bad frame"
}}
""")
        assert has(run_validators(stage, "usdRosValidators:RosTopic"), INVALID_OVERRIDE_FRAME_ID)

    def test_qos_match_publisher_true_on_publisher_gives_warning(self):
        stage = make_stage("""
#usda 1.0
def Xform "T" (
    prepend apiSchemas = ["RosTopicAPI"]
) {
    token ros:topic:role = "publisher"
    string ros:topic:name = "joint_states"
    string ros:topic:type = "sensor_msgs/msg/JointState"
    double ros:topic:publish_rate = 10
    bool ros:topic:qos:match_publisher = true
}
""")
        assert has(run_validators(stage, "usdRosValidators:RosTopic"), INVALID_MATCH_PUBLISHER)

    def test_qos_depth_zero_with_keep_last_gives_error(self):
        stage = make_stage("""
#usda 1.0
def Xform "T" (
    prepend apiSchemas = ["RosTopicAPI"]
) {
    token ros:topic:role = "publisher"
    string ros:topic:name = "joint_states"
    string ros:topic:type = "sensor_msgs/msg/JointState"
    double ros:topic:publish_rate = 10
    token ros:topic:qos:history = "keep_last"
    int ros:topic:qos:depth = 0
}
""")
        assert has(run_validators(stage, "usdRosValidators:RosTopic"), INVALID_QOS_DEPTH)

    def test_qos_depth_positive_with_keep_last_no_violation(self):
        stage = make_stage("""
#usda 1.0
def Xform "T" (
    prepend apiSchemas = ["RosTopicAPI"]
) {
    token ros:topic:role = "publisher"
    string ros:topic:name = "joint_states"
    string ros:topic:type = "sensor_msgs/msg/JointState"
    double ros:topic:publish_rate = 10
    token ros:topic:qos:history = "keep_last"
    int ros:topic:qos:depth = 10
}
""")
        assert none_with(run_validators(stage, "usdRosValidators:RosTopic"), INVALID_QOS_DEPTH)


# ------------------------------------------------------------------ #
# §2.9 – Prohibited interfaces                                         #
# ------------------------------------------------------------------ #


class TestProhibitedInterfaces:
    def test_clock_topic_gives_error(self):
        stage = make_stage("""
#usda 1.0
def Xform "Clock" (
    prepend apiSchemas = ["RosTopicAPI"]
) {
    token ros:topic:role = "publisher"
    string ros:topic:name = "/clock"
    string ros:topic:type = "rosgraph_msgs/msg/Clock"
    double ros:topic:publish_rate = 100.0
}
""")
        assert has(run_validators(stage, "usdRosValidators:RosTopic"), PROHIBITED_TOPIC_NAME)

    def test_simulation_interfaces_type_gives_error(self):
        stage = make_stage("""
#usda 1.0
def Xform "Spawn" (
    prepend apiSchemas = ["RosServiceAPI"]
) {
    token ros:service:role = "server"
    string ros:service:name = "spawn_entity"
    string ros:service:type = "simulation_interfaces/srv/SpawnEntity"
}
""")
        assert has(run_validators(stage, "usdRosValidators:RosService"), PROHIBITED_INTERFACE_TYPE)

    def test_rosgraph_clock_type_gives_error(self):
        stage = make_stage("""
#usda 1.0
def Xform "T" (
    prepend apiSchemas = ["RosTopicAPI"]
) {
    token ros:topic:role = "publisher"
    string ros:topic:name = "sim_time"
    string ros:topic:type = "rosgraph_msgs/msg/Clock"
    double ros:topic:publish_rate = 100.0
}
""")
        assert has(run_validators(stage, "usdRosValidators:RosTopic"), PROHIBITED_INTERFACE_TYPE)


# ------------------------------------------------------------------ #
# §2.5 – RosServiceAPI                                                 #
# ------------------------------------------------------------------ #


class TestRosService:
    def test_complete_service_no_violation(self):
        stage = make_stage(f"""
#usda 1.0
def Xform "Svc" (
    prepend apiSchemas = ["RosServiceAPI"]
) {{
    {_FULL_SERVICE}
}}
""")
        v = run_validators(stage, "usdRosValidators:RosService")
        for cid in (MISSING_SERVICE_ATTR, INVALID_SERVICE_STARTS_ENABLED, INVALID_ROS_NAME):
            assert none_with(v, cid), f"Unexpected violation {cid}"

    def test_missing_service_role_gives_error(self):
        stage = make_stage("""
#usda 1.0
def Xform "Svc" (
    prepend apiSchemas = ["RosServiceAPI"]
) {
    string ros:service:name = "set_led"
    string ros:service:type = "std_srvs/srv/SetBool"
}
""")
        assert has(run_validators(stage, "usdRosValidators:RosService"), MISSING_SERVICE_ATTR)

    def test_missing_service_name_gives_error(self):
        stage = make_stage("""
#usda 1.0
def Xform "Svc" (
    prepend apiSchemas = ["RosServiceAPI"]
) {
    token ros:service:role = "server"
    string ros:service:type = "std_srvs/srv/SetBool"
}
""")
        assert has(run_validators(stage, "usdRosValidators:RosService"), MISSING_SERVICE_ATTR)

    def test_invalid_service_name_gives_naming_error(self):
        stage = make_stage("""
#usda 1.0
def Xform "Svc" (
    prepend apiSchemas = ["RosServiceAPI"]
) {
    token ros:service:role = "server"
    string ros:service:name = "2bad_name"
    string ros:service:type = "std_srvs/srv/SetBool"
}
""")
        assert has(run_validators(stage, "usdRosValidators:RosService"), INVALID_ROS_NAME)

    def test_service_starts_enabled_bool_no_violation(self):
        stage = make_stage("""
#usda 1.0
def Xform "Svc" (
    prepend apiSchemas = ["RosServiceAPI"]
) {
    token ros:service:role = "server"
    string ros:service:name = "set_led"
    string ros:service:type = "std_srvs/srv/SetBool"
    bool ros:service:starts_enabled = false
}
""")
        assert none_with(run_validators(stage, "usdRosValidators:RosService"), INVALID_SERVICE_STARTS_ENABLED)

    def test_service_starts_enabled_wrong_type_gives_error(self):
        stage = make_stage("""
#usda 1.0
def Xform "Svc" (
    prepend apiSchemas = ["RosServiceAPI"]
) {
    token ros:service:role = "server"
    string ros:service:name = "set_led"
    string ros:service:type = "std_srvs/srv/SetBool"
    string ros:service:starts_enabled = "false"
}
""")
        assert has(run_validators(stage, "usdRosValidators:RosService"), INVALID_SERVICE_STARTS_ENABLED)


# ------------------------------------------------------------------ #
# §2.6 – RosActionAPI                                                  #
# ------------------------------------------------------------------ #


class TestRosAction:
    def test_complete_action_no_violation(self):
        stage = make_stage(f"""
#usda 1.0
def Xform "Act" (
    prepend apiSchemas = ["RosActionAPI"]
) {{
    {_FULL_ACTION}
}}
""")
        v = run_validators(stage, "usdRosValidators:RosAction")
        for cid in (MISSING_ACTION_ATTR, INVALID_ACTION_STARTS_ENABLED, INVALID_ROS_NAME):
            assert none_with(v, cid), f"Unexpected violation {cid}"

    def test_missing_action_type_gives_error(self):
        stage = make_stage("""
#usda 1.0
def Xform "Act" (
    prepend apiSchemas = ["RosActionAPI"]
) {
    token ros:action:role = "server"
    string ros:action:name = "follow_trajectory"
}
""")
        assert has(run_validators(stage, "usdRosValidators:RosAction"), MISSING_ACTION_ATTR)

    def test_invalid_action_role_gives_error(self):
        stage = make_stage("""
#usda 1.0
def Xform "Act" (
    prepend apiSchemas = ["RosActionAPI"]
) {
    token ros:action:role = "broker"
    string ros:action:name = "move_arm"
    string ros:action:type = "control_msgs/action/FollowJointTrajectory"
}
""")
        assert has(run_validators(stage, "usdRosValidators:RosAction"), MISSING_ACTION_ATTR)

    def test_action_starts_enabled_bool_no_violation(self):
        stage = make_stage("""
#usda 1.0
def Xform "Act" (
    prepend apiSchemas = ["RosActionAPI"]
) {
    token ros:action:role = "server"
    string ros:action:name = "follow_trajectory"
    string ros:action:type = "control_msgs/action/FollowJointTrajectory"
    bool ros:action:starts_enabled = false
}
""")
        assert none_with(run_validators(stage, "usdRosValidators:RosAction"), INVALID_ACTION_STARTS_ENABLED)

    def test_action_starts_enabled_wrong_type_gives_error(self):
        stage = make_stage("""
#usda 1.0
def Xform "Act" (
    prepend apiSchemas = ["RosActionAPI"]
) {
    token ros:action:role = "server"
    string ros:action:name = "follow_trajectory"
    string ros:action:type = "control_msgs/action/FollowJointTrajectory"
    token ros:action:starts_enabled = "false"
}
""")
        assert has(run_validators(stage, "usdRosValidators:RosAction"), INVALID_ACTION_STARTS_ENABLED)


# ------------------------------------------------------------------ #
# §2.7 – RosFrameAPI + PhysicsRigidBodyAPI                             #
# ------------------------------------------------------------------ #


class TestRosFrameAPI:
    def test_frame_api_with_rigid_body_gives_warning(self):
        stage = make_stage("""
#usda 1.0
def Xform "Link" (
    prepend apiSchemas = ["PhysicsRigidBodyAPI", "RosFrameAPI"]
) {}
""")
        assert has(run_validators(stage, "usdRosValidators:RosFrameAPI"), FRAME_ON_RIGID_BODY)

    def test_frame_api_without_rigid_body_no_violation(self):
        stage = make_stage("""
#usda 1.0
def Xform "camera_optical_frame" (
    prepend apiSchemas = ["RosFrameAPI"]
) {}
""")
        assert none_with(run_validators(stage, "usdRosValidators:RosFrameAPI"), FRAME_ON_RIGID_BODY)


class TestRosFrameAttributes:
    def test_invalid_frame_id_gives_warning(self):
        stage = make_stage("""
#usda 1.0
def Xform "frame" (
    prepend apiSchemas = ["RosFrameAPI"]
) {
    string ros:frame:id = "bad frame"
}
""")
        assert has(run_validators(stage, "usdRosValidators:RosFrameAttributes"), INVALID_FRAME_ID)

    def test_non_bool_frame_static_gives_error(self):
        stage = make_stage("""
#usda 1.0
def Xform "frame" (
    prepend apiSchemas = ["RosFrameAPI"]
) {
    token ros:frame:static = "true"
}
""")
        assert has(run_validators(stage, "usdRosValidators:RosFrameAttributes"), INVALID_FRAME_STATIC)

    def test_valid_frame_attributes_no_violation(self):
        stage = make_stage("""
#usda 1.0
def Xform "frame" (
    prepend apiSchemas = ["RosFrameAPI"]
) {
    string ros:frame:id = "camera_optical_frame"
    bool ros:frame:static = true
}
""")
        errors = run_validators(stage, "usdRosValidators:RosFrameAttributes")
        assert none_with(errors, INVALID_FRAME_ID)
        assert none_with(errors, INVALID_FRAME_STATIC)


# ------------------------------------------------------------------ #
# §2.8 – Camera optical frame                                          #
# ------------------------------------------------------------------ #


class TestCameraOpticalFrame:
    def test_topic_directly_on_camera_gives_warning(self):
        stage = make_stage(f"""
#usda 1.0
def Camera "rgb_camera" (
    prepend apiSchemas = ["RosTopicAPI"]
) {{
    {_FULL_TOPIC}
}}
""")
        assert has(run_validators(stage, "usdRosValidators:CameraOpticalFrame"), CAMERA_OPTICAL_FRAME)

    def test_topic_on_child_with_rotation_no_violation(self):
        stage = make_stage(f"""
#usda 1.0
def Camera "rgb_camera" {{
    def Xform "camera_optical_frame" (
        prepend apiSchemas = ["RosTopicAPI", "RosFrameAPI"]
    ) {{
        float xformOp:rotateX = 180
        uniform token[] xformOpOrder = ["xformOp:rotateX"]
        {_FULL_TOPIC}
    }}
}}
""")
        assert none_with(run_validators(stage, "usdRosValidators:CameraOpticalFrame"), CAMERA_OPTICAL_FRAME)

    def test_topic_on_child_without_rotation_gives_warning(self):
        stage = make_stage(f"""
#usda 1.0
def Camera "rgb_camera" {{
    def Xform "camera_optical_frame" (
        prepend apiSchemas = ["RosTopicAPI"]
    ) {{
        {_FULL_TOPIC}
    }}
}}
""")
        assert has(run_validators(stage, "usdRosValidators:CameraOpticalFrame"), CAMERA_OPTICAL_FRAME)

    def test_camera_without_ros_interface_no_violation(self):
        """A Camera prim with no ROS schemas is not flagged."""
        stage = make_stage("""
#usda 1.0
def Camera "rgb_camera" {}
""")
        assert none_with(run_validators(stage, "usdRosValidators:CameraOpticalFrame"), CAMERA_OPTICAL_FRAME)


# ------------------------------------------------------------------ #
# §2.10 – Custom joint name                                            #
# ------------------------------------------------------------------ #


class TestRosJointName:
    def test_revolute_joint_without_ros_name_gives_warning(self):
        stage = make_stage("""
#usda 1.0
def PhysicsRevoluteJoint "shoulder_pan_joint" {
    float physics:lowerLimit = -3.14
    float physics:upperLimit = 3.14
}
""")
        assert has(run_validators(stage, "usdRosValidators:RosJointName"), MISSING_JOINT_NAME)

    def test_revolute_joint_with_ros_name_no_violation(self):
        stage = make_stage("""
#usda 1.0
def PhysicsRevoluteJoint "shoulder_pan_joint" {
    float physics:lowerLimit = -3.14
    float physics:upperLimit = 3.14
    custom string ros:joint:name = "shoulder_pan_joint"
}
""")
        assert none_with(run_validators(stage, "usdRosValidators:RosJointName"), MISSING_JOINT_NAME)

    def test_fixed_joint_without_ros_name_gives_warning(self):
        stage = make_stage("""
#usda 1.0
def PhysicsFixedJoint "world_fixed" {}
""")
        assert has(run_validators(stage, "usdRosValidators:RosJointName"), MISSING_JOINT_NAME)

    def test_prismatic_joint_with_ros_name_no_violation(self):
        stage = make_stage("""
#usda 1.0
def PhysicsPrismaticJoint "slide" {
    float physics:lowerLimit = 0.0
    float physics:upperLimit = 0.5
    custom string ros:joint:name = "linear_actuator"
}
""")
        assert none_with(run_validators(stage, "usdRosValidators:RosJointName"), MISSING_JOINT_NAME)

    def test_non_joint_prim_not_flagged(self):
        """A plain Xform is not a joint and must never trigger 2.10.1."""
        stage = make_stage("""
#usda 1.0
def Xform "link" {}
""")
        assert none_with(run_validators(stage, "usdRosValidators:RosJointName"), MISSING_JOINT_NAME)
