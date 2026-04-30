"""Section 2 – ROS Integration Schema checks."""

from __future__ import annotations

import re

from pxr import Sdf, Usd, UsdGeom, UsdPhysics

from .base import (
    ErrorType, TimeRange, _error, _prim_site, _site as _base_site,
    register_prim_validator, register_stage_validator,
)
from ._tokens import (
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

_ROS_NAME_RE = re.compile(r"^/?[a-zA-Z_][a-zA-Z0-9_]*(?:/[a-zA-Z_][a-zA-Z0-9_]*)*$")

_PROHIBITED_TYPES = ("rosgraph_msgs/msg/Clock", "simulation_interfaces/")
_PROHIBITED_TOPIC_NAMES = ("/clock",)

_ROS_SCHEMAS = {"RosContextAPI", "RosTopicAPI", "RosServiceAPI", "RosActionAPI", "RosFrameAPI"}
_INTERFACE_SCHEMAS = {"RosTopicAPI", "RosServiceAPI", "RosActionAPI"}


def _applied(prim: Usd.Prim) -> set[str]:
    list_op = prim.GetMetadata("apiSchemas")
    if list_op is None:
        return set()
    return set(list_op.GetAppliedItems())


def _str_attr(prim: Usd.Prim, name: str) -> str | None:
    attr = prim.GetAttribute(name)
    return attr.Get() if attr.IsValid() else None


def _bool_attr(prim: Usd.Prim, name: str) -> bool | None:
    attr = prim.GetAttribute(name)
    return attr.Get() if attr.IsValid() else None


def _build_payload_roots(stage: Usd.Stage) -> set[Sdf.Path]:
    roots: set[Sdf.Path] = set()
    for prim in stage.TraverseAll():
        for prim_spec in prim.GetPrimStack():
            if prim_spec.payloadList.GetAddedOrExplicitItems():
                roots.add(prim.GetPath())
                break
    return roots


def _is_inside_payload(prim: Usd.Prim, payload_roots: set[Sdf.Path]) -> bool:
    parent = prim.GetParent()
    while parent and parent.IsValid():
        if parent.GetPath() in payload_roots:
            return True
        parent = parent.GetParent()
    return False


def _validate_ros_name(name: str) -> bool:
    return bool(_ROS_NAME_RE.match(name))


def _nearest_rigid_body_ancestor(prim: Usd.Prim) -> Usd.Prim | None:
    parent = prim.GetParent()
    while parent and parent.IsValid():
        if parent.HasAPI(UsdPhysics.RigidBodyAPI):
            return parent
        parent = parent.GetParent()
    return None


def _validate_context_namespace(name: str) -> bool:
    if "~" in name or "{" in name or "}" in name:
        return False
    if "//" in name:
        return False
    if name.startswith("/"):
        if name.endswith("/"):
            return False
        return _validate_ros_name(name)
    if name.startswith("/") or name.endswith("/"):
        return False
    return _validate_ros_name(name)


def _find_outermost_context(stage: Usd.Stage) -> Usd.Prim | None:
    for prim in stage.Traverse():
        if "RosContextAPI" in _applied(prim):
            return prim
    return None


def _site(stage, prim):
    return _prim_site(stage, str(prim.GetPath()))


# --- §2.1 RosContextPlacement ---

def _validate_ros_context_placement(stage: Usd.Stage, timeRange: TimeRange):
    payload_roots = _build_payload_roots(stage)
    outermost = _find_outermost_context(stage)
    for prim in stage.TraverseAll():
        applied = _applied(prim)
        if "RosContextAPI" not in applied:
            continue
        pp = str(prim.GetPath())
        if _is_inside_payload(prim, payload_roots):
            yield _error(CONTEXT_INSIDE_PAYLOAD, ErrorType.Warn, _prim_site(stage, pp),
                f"RosContextAPI prim '{pp}' is inside a payload arc. "
                "The namespace graph must be resolvable without loading heavy geometry; "
                "RosContextAPI prims must reside outside payloads per REP §2.1.",
                "Move the RosContextAPI prim above the payload boundary.")
        namespace = _str_attr(prim, "ros:context:namespace")
        if namespace and not _validate_context_namespace(namespace):
            yield _error(INVALID_CONTEXT_NAMESPACE, ErrorType.Error, _prim_site(stage, pp),
                f"RosContextAPI namespace '{namespace}' on '{pp}' "
                "violates §2.1.1 rules. Namespaces must be either composable "
                "(no leading/trailing slash) or absolute (leading slash), and "
                "must not use '~' or '{{}}' substitutions.",
                "Use a valid namespace such as 'robot_1', 'left_camera', or '/global_ns'.")
        if outermost and prim != outermost:
            pf_attr = prim.GetAttribute("ros:context:parent_frame")
            if pf_attr.IsValid() and pf_attr.Get() is not None:
                yield _error(NESTED_CONTEXT_PARENT_FRAME, ErrorType.Warn, _prim_site(stage, pp),
                    f"Nested RosContextAPI '{pp}' sets ros:context:parent_frame, which is "
                    "only valid on the outermost context in the stage per REP §2.1.1. "
                    "This attribute will be ignored.",
                    "Remove ros:context:parent_frame from all contexts except the top-most one.")

register_stage_validator("RosContextPlacement", _validate_ros_context_placement,
    doc="REP §2.1: RosContextAPI prims must reside outside payload arcs.", section="2.1")


# --- §2.2 RosInterfacePlacement ---

def _validate_ros_interface_placement(stage: Usd.Stage, timeRange: TimeRange):
    payload_roots = _build_payload_roots(stage)
    for prim in stage.TraverseAll():
        applied = _applied(prim)
        in_payload = _is_inside_payload(prim, payload_roots)
        for schema in _INTERFACE_SCHEMAS:
            if schema in applied and in_payload:
                pp = str(prim.GetPath())
                yield _error(INTERFACE_INSIDE_PAYLOAD, ErrorType.Error, _prim_site(stage, pp),
                    f"Prim '{pp}' has {schema} but is inside a payload arc. "
                    "Interface prims must reside in the lightweight, traversable "
                    "kinematic hierarchy (outside payloads) per REP §2.2.",
                    "Move the prim (or its schema) above the payload boundary, "
                    "following the ETL pattern from §1.2.1.")

register_stage_validator("RosInterfacePlacement", _validate_ros_interface_placement,
    doc="REP §2.2: Interface prims must reside outside payload arcs.", section="2.2")


# --- §2.2 RosInterfaceStructure ---

def _validate_ros_interface_structure(stage: Usd.Stage, timeRange: TimeRange):
    for prim in stage.TraverseAll():
        applied = _applied(prim)
        present = _INTERFACE_SCHEMAS & applied
        if not present:
            continue
        pp = str(prim.GetPath())
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            yield _error(INTERFACE_ON_RIGID_BODY, ErrorType.Error, _prim_site(stage, pp),
                f"Prim '{pp}' carries ROS interface schemas and PhysicsRigidBodyAPI. "
                "Interface schemas should be placed on dedicated logical prims rather "
                "than physical rigid-body prims per REP §2.2.",
                "Move RosTopicAPI/RosServiceAPI/RosActionAPI schemas to a dedicated "
                "child Xform (for sensor interfaces) or a logical interfaces scope.")
        if len(present) > 1:
            yield _error(MULTIPLE_INTERFACES_PER_PRIM, ErrorType.Error, _prim_site(stage, pp),
                f"Prim '{pp}' carries multiple interface schemas {sorted(present)}. "
                "Sensor interfaces must use one interface schema per prim per REP §2.2.",
                "Split interfaces across separate child prims (for example, one prim "
                "for RosTopicAPI image_raw and another prim for camera_info).")
        rigid_ancestor = _nearest_rigid_body_ancestor(prim)
        if rigid_ancestor:
            parent = prim.GetParent()
            is_direct_child = parent and parent.IsValid() and parent == rigid_ancestor
            if not is_direct_child or prim.GetTypeName() != "Xform":
                yield _error(SENSOR_NOT_DIRECT_XFORM_CHILD, ErrorType.Warn, _prim_site(stage, pp),
                    f"Sensor interface prim '{pp}' is under rigid body "
                    f"'{rigid_ancestor.GetPath()}', but sensor interfaces should be authored "
                    "on a direct child UsdGeomXform of the physical link per REP §2.2.",
                    "Place this interface on a direct child Xform under the rigid body link.")

register_stage_validator("RosInterfaceStructure", _validate_ros_interface_structure,
    doc="REP §2.2: interface prim structure for robot-wide and sensor interfaces.", section="2.2")


# --- §2.4 RosTopic ---

_ALLOWED_RELIABILITY = {"system_default", "reliable", "best_effort"}
_ALLOWED_DURABILITY = {"system_default", "transient_local", "volatile"}
_ALLOWED_HISTORY = {"system_default", "keep_last", "keep_all"}
_ALLOWED_TOPIC_ROLES = {"publisher", "subscription"}


def _check_topic(prim: Usd.Prim, timeRange: TimeRange):
    pp = str(prim.GetPath())
    site = _base_site(prim)
    role = _str_attr(prim, "ros:topic:role")
    name = _str_attr(prim, "ros:topic:name")
    type_ = _str_attr(prim, "ros:topic:type")

    if not role:
        yield _error(MISSING_TOPIC_ROLE, ErrorType.Error, site,
            f"RosTopicAPI prim '{pp}' is missing required 'ros:topic:role'.",
            'Set `token ros:topic:role = "publisher"` or `"subscription"`.')
    elif role not in _ALLOWED_TOPIC_ROLES:
        yield _error(MISSING_TOPIC_ROLE, ErrorType.Error, site,
            f"RosTopicAPI prim '{pp}' has invalid role '{role}'. "
            f"Allowed values: {sorted(_ALLOWED_TOPIC_ROLES)}.",
            "Use 'publisher' or 'subscription'.")

    if not name:
        yield _error(MISSING_TOPIC_NAME, ErrorType.Error, site,
            f"RosTopicAPI prim '{pp}' is missing required 'ros:topic:name'.",
            'Set `string ros:topic:name = "<topic_name>"`.')
    else:
        if not _validate_ros_name(name):
            yield _error(INVALID_ROS_NAME, ErrorType.Error, site,
                f"'ros:topic:name' value '{name}' on '{pp}' violates ROS 2 naming rules. "
                "Names must contain only alphanumeric characters, underscores, and "
                "forward slashes, and must not start with a number.",
                "Rename to a valid ROS 2 topic/service/action name "
                "(e.g. 'joint_states', '/robot_1/cmd_vel').")
        for prohibited in _PROHIBITED_TOPIC_NAMES:
            if name == prohibited or name.endswith(prohibited):
                yield _error(PROHIBITED_TOPIC_NAME, ErrorType.Error, site,
                    f"Topic name '{name}' is a prohibited simulator-level interface "
                    "(/clock). Assets must not include simulator-level interfaces per REP §2.9.",
                    "Remove this interface from the asset.")

    if not type_:
        yield _error(MISSING_TOPIC_TYPE, ErrorType.Error, site,
            f"RosTopicAPI prim '{pp}' is missing required 'ros:topic:type'.",
            'Set `string ros:topic:type = "<pkg>/msg/<Type>"`.')
    else:
        for prohibited in _PROHIBITED_TYPES:
            if type_.startswith(prohibited):
                yield _error(PROHIBITED_INTERFACE_TYPE, ErrorType.Error, site,
                    f"Interface type '{type_}' on '{pp}' is a prohibited simulator-level "
                    "interface. Assets must not include interfaces from "
                    "simulation_interfaces or rosgraph_msgs/Clock per REP §2.9.",
                    "Remove this interface from the asset.")

    if role == "publisher":
        rate_attr = prim.GetAttribute("ros:topic:publish_rate")
        if not rate_attr.IsValid() or rate_attr.Get() is None:
            yield _error(MISSING_PUBLISH_RATE, ErrorType.Error, site,
                f"Publisher RosTopicAPI prim '{pp}' is missing 'ros:topic:publish_rate'. "
                "Required for all publishers per REP §2.4.",
                "Set `double ros:topic:publish_rate = <Hz>`.")

    for attr_name, allowed in (
        ("ros:topic:qos:reliability", _ALLOWED_RELIABILITY),
        ("ros:topic:qos:durability", _ALLOWED_DURABILITY),
        ("ros:topic:qos:history", _ALLOWED_HISTORY),
    ):
        val = _str_attr(prim, attr_name)
        if val is not None and val not in allowed:
            yield _error(INVALID_QOS_TOKEN, ErrorType.Warn, site,
                f"QoS attribute '{attr_name}' on '{pp}' has value '{val}' "
                f"which is not in the allowed set {sorted(allowed)}.",
                f"Use one of: {sorted(allowed)}.")

    mp_attr = prim.GetAttribute("ros:topic:qos:match_publisher")
    if mp_attr.IsValid():
        mp_val = mp_attr.Get()
        if not isinstance(mp_val, bool):
            yield _error(INVALID_MATCH_PUBLISHER, ErrorType.Error, site,
                f"'ros:topic:qos:match_publisher' on '{pp}' must be a bool. "
                f"Got {type(mp_val).__name__}.",
                "Use `bool ros:topic:qos:match_publisher = true|false`.")
        elif mp_val and role == "publisher":
            yield _error(INVALID_MATCH_PUBLISHER, ErrorType.Warn, site,
                f"'ros:topic:qos:match_publisher' is true on publisher '{pp}'. "
                "This QoS option is only applicable to subscriptions per REP §2.4.",
                "Unset this attribute on publishers.")

    depth_attr = prim.GetAttribute("ros:topic:qos:depth")
    if depth_attr.IsValid():
        depth = depth_attr.Get()
        if not isinstance(depth, int):
            yield _error(INVALID_QOS_DEPTH, ErrorType.Error, site,
                f"'ros:topic:qos:depth' on '{pp}' must be an int. "
                f"Got {type(depth).__name__}.",
                "Use a positive integer depth.")
        else:
            history = _str_attr(prim, "ros:topic:qos:history")
            if history == "keep_last" and depth <= 0:
                yield _error(INVALID_QOS_DEPTH, ErrorType.Error, site,
                    f"'ros:topic:qos:depth' on '{pp}' is {depth}. "
                    "Depth must be > 0 when history is keep_last.",
                    "Set `ros:topic:qos:depth` to a positive integer.")

    se_attr = prim.GetAttribute("ros:topic:starts_enabled")
    if se_attr.IsValid():
        se_val = _bool_attr(prim, "ros:topic:starts_enabled")
        if not isinstance(se_val, bool):
            yield _error(INVALID_STARTS_ENABLED, ErrorType.Error, site,
                f"'ros:topic:starts_enabled' on '{pp}' must be a bool. "
                f"Got {type(se_val).__name__}.",
                "Use `bool ros:topic:starts_enabled = true|false`.")

    ofi_attr = prim.GetAttribute("ros:topic:override_frame_id")
    if ofi_attr.IsValid():
        ofi_val = _str_attr(prim, "ros:topic:override_frame_id")
        if ofi_val is not None:
            if not isinstance(ofi_val, str):
                yield _error(INVALID_OVERRIDE_FRAME_ID, ErrorType.Error, site,
                    f"'ros:topic:override_frame_id' on '{pp}' must be a string. "
                    f"Got {type(ofi_val).__name__}.",
                    'Use `string ros:topic:override_frame_id = "map"`.')
            elif ofi_val and not _validate_ros_name(ofi_val):
                yield _error(INVALID_OVERRIDE_FRAME_ID, ErrorType.Warn, site,
                    f"'ros:topic:override_frame_id' value '{ofi_val}' on '{pp}' "
                    "does not follow ROS naming rules.",
                    "Use a valid ROS frame name (e.g. 'map', 'earth', '/robot/base_link').")


register_prim_validator("RosTopic", _check_topic,
    doc="REP §2.4: Validate RosTopicAPI required attributes and QoS values.",
    section="2.4", schema_types=["UsdRosTopicAPI"])


# --- §2.5 RosService ---

_ALLOWED_SERVICE_ROLES = {"server", "client"}


def _check_service(prim: Usd.Prim, timeRange: TimeRange):
    pp = str(prim.GetPath())
    site = _base_site(prim)
    for attr_name, check_id, hint in (
        ("ros:service:role", MISSING_SERVICE_ATTR, "Use 'server' or 'client'."),
        ("ros:service:name", MISSING_SERVICE_ATTR, "Set a valid ROS 2 service name."),
        ("ros:service:type", MISSING_SERVICE_ATTR, 'Set `string ros:service:type = "<pkg>/srv/<Type>"`'),
    ):
        if not _str_attr(prim, attr_name):
            yield _error(check_id, ErrorType.Error, site,
                f"RosServiceAPI prim '{pp}' is missing required '{attr_name}'.", hint)

    role = _str_attr(prim, "ros:service:role")
    if role and role not in _ALLOWED_SERVICE_ROLES:
        yield _error(MISSING_SERVICE_ATTR, ErrorType.Error, site,
            f"RosServiceAPI prim '{pp}' has invalid role '{role}'. "
            f"Allowed: {sorted(_ALLOWED_SERVICE_ROLES)}.",
            "Use 'server' or 'client'.")

    name = _str_attr(prim, "ros:service:name")
    if name and not _validate_ros_name(name):
        yield _error(INVALID_ROS_NAME, ErrorType.Error, site,
            f"'ros:service:name' value '{name}' on '{pp}' violates ROS 2 naming rules.",
            "Use only alphanumeric characters, underscores, and forward slashes.")

    type_ = _str_attr(prim, "ros:service:type")
    if type_:
        for prohibited in _PROHIBITED_TYPES:
            if type_.startswith(prohibited):
                yield _error(PROHIBITED_INTERFACE_TYPE, ErrorType.Error, site,
                    f"Service type '{type_}' on '{pp}' is a prohibited simulator-level "
                    "interface. Assets must not include interfaces from "
                    "simulation_interfaces or rosgraph_msgs/Clock per REP §2.9.",
                    "Remove this interface from the asset.")
                break

    se_attr = prim.GetAttribute("ros:service:starts_enabled")
    if se_attr.IsValid():
        se_val = _bool_attr(prim, "ros:service:starts_enabled")
        if not isinstance(se_val, bool):
            yield _error(INVALID_SERVICE_STARTS_ENABLED, ErrorType.Error, site,
                f"'ros:service:starts_enabled' on '{pp}' must be a bool. "
                f"Got {type(se_val).__name__}.",
                "Use `bool ros:service:starts_enabled = true|false`.")


register_prim_validator("RosService", _check_service,
    doc="REP §2.5: Validate RosServiceAPI required attributes.",
    section="2.5", schema_types=["UsdRosServiceAPI"])


# --- §2.6 RosAction ---

_ALLOWED_ACTION_ROLES = {"server", "client"}


def _check_action(prim: Usd.Prim, timeRange: TimeRange):
    pp = str(prim.GetPath())
    site = _base_site(prim)
    for attr_name, check_id, hint in (
        ("ros:action:role", MISSING_ACTION_ATTR, "Use 'server' or 'client'."),
        ("ros:action:name", MISSING_ACTION_ATTR, "Set a valid ROS 2 action name."),
        ("ros:action:type", MISSING_ACTION_ATTR, 'Set `string ros:action:type = "<pkg>/action/<Type>"`'),
    ):
        if not _str_attr(prim, attr_name):
            yield _error(check_id, ErrorType.Error, site,
                f"RosActionAPI prim '{pp}' is missing required '{attr_name}'.", hint)

    role = _str_attr(prim, "ros:action:role")
    if role and role not in _ALLOWED_ACTION_ROLES:
        yield _error(MISSING_ACTION_ATTR, ErrorType.Error, site,
            f"RosActionAPI prim '{pp}' has invalid role '{role}'. "
            f"Allowed: {sorted(_ALLOWED_ACTION_ROLES)}.",
            "Use 'server' or 'client'.")

    name = _str_attr(prim, "ros:action:name")
    if name and not _validate_ros_name(name):
        yield _error(INVALID_ROS_NAME, ErrorType.Error, site,
            f"'ros:action:name' value '{name}' on '{pp}' violates ROS 2 naming rules.",
            "Use only alphanumeric characters, underscores, and forward slashes.")

    se_attr = prim.GetAttribute("ros:action:starts_enabled")
    if se_attr.IsValid():
        se_val = _bool_attr(prim, "ros:action:starts_enabled")
        if not isinstance(se_val, bool):
            yield _error(INVALID_ACTION_STARTS_ENABLED, ErrorType.Error, site,
                f"'ros:action:starts_enabled' on '{pp}' must be a bool. "
                f"Got {type(se_val).__name__}.",
                "Use `bool ros:action:starts_enabled = true|false`.")


register_prim_validator("RosAction", _check_action,
    doc="REP §2.6: Validate RosActionAPI required attributes.",
    section="2.6", schema_types=["UsdRosActionAPI"])


# --- §2.7 RosFrameAPI ---

def _check_ros_frame_api(prim: Usd.Prim, timeRange: TimeRange):
    if prim.HasAPI(UsdPhysics.RigidBodyAPI):
        yield _error(FRAME_ON_RIGID_BODY, ErrorType.Warn, _base_site(prim),
            f"Prim '{prim.GetPath()}' has both RosFrameAPI and PhysicsRigidBodyAPI. "
            "Physical links connected via joints receive implicit TF broadcasting; "
            "explicit RosFrameAPI is redundant and may cause duplicate frames.",
            "Remove RosFrameAPI from prims that already carry PhysicsRigidBodyAPI. "
            "Use RosFrameAPI only for non-physical dummy frames.")

register_prim_validator("RosFrameAPI", _check_ros_frame_api,
    doc="REP §2.7: RosFrameAPI should not duplicate implicit TF.",
    section="2.7", schema_types=["UsdRosFrameAPI"])


# --- §2.7 RosFrameAttributes ---

def _check_ros_frame_attributes(prim: Usd.Prim, timeRange: TimeRange):
    pp = str(prim.GetPath())
    site = _base_site(prim)

    fid_attr = prim.GetAttribute("ros:frame:id")
    if fid_attr.IsValid():
        fid_val = _str_attr(prim, "ros:frame:id")
        if fid_val is not None:
            if not isinstance(fid_val, str):
                yield _error(INVALID_FRAME_ID, ErrorType.Error, site,
                    f"'ros:frame:id' on '{pp}' must be a string. Got {type(fid_val).__name__}.",
                    'Use `string ros:frame:id = "camera_optical_frame"`.')
            elif fid_val and not _validate_ros_name(fid_val):
                yield _error(INVALID_FRAME_ID, ErrorType.Warn, site,
                    f"'ros:frame:id' value '{fid_val}' on '{pp}' does not follow ROS naming rules.",
                    "Use a valid TF frame name (e.g. 'base_link', 'camera_optical_frame').")

    fs_attr = prim.GetAttribute("ros:frame:static")
    if fs_attr.IsValid():
        fs_val = _bool_attr(prim, "ros:frame:static")
        if not isinstance(fs_val, bool):
            yield _error(INVALID_FRAME_STATIC, ErrorType.Error, site,
                f"'ros:frame:static' on '{pp}' must be a bool. Got {type(fs_val).__name__}.",
                "Use `bool ros:frame:static = true|false`.")

register_prim_validator("RosFrameAttributes", _check_ros_frame_attributes,
    doc="REP §2.7: Validate RosFrameAPI attribute types and frame-id naming.",
    section="2.7", schema_types=["UsdRosFrameAPI"])


# --- §2.8 CameraOpticalFrame ---

def _validate_camera_optical_frame(stage: Usd.Stage, timeRange: TimeRange):
    for prim in stage.TraverseAll():
        if prim.GetTypeName() != "Camera":
            continue
        if "RosTopicAPI" in _applied(prim):
            yield _error(CAMERA_OPTICAL_FRAME, ErrorType.Warn, _site(stage, prim),
                f"RosTopicAPI is applied directly to Camera prim '{prim.GetPath()}'. "
                "OpenUSD cameras face -Z; ROS optical frames must face +Z. "
                "Authors must create a child Xform rotated 180° around X and apply "
                "RosTopicAPI there per REP §2.8.",
                "Create a child UsdGeomXform (e.g. 'camera_optical_frame') rotated "
                "180° around local X and move all RosTopicAPI / RosFrameAPI schemas to it.")
            continue
        for child in prim.GetAllChildren():
            if "RosTopicAPI" not in _applied(child) and "RosFrameAPI" not in _applied(child):
                continue
            xformable = UsdGeom.Xformable(child)
            if not xformable:
                continue
            has_x_rotation = any(
                "rotatex" in op.GetOpName().lower() or "orient" in op.GetOpName().lower()
                for op in xformable.GetOrderedXformOps()
            )
            if not has_x_rotation:
                yield _error(CAMERA_OPTICAL_FRAME, ErrorType.Warn, _site(stage, child),
                    f"Camera child prim '{child.GetPath()}' carries ROS interface schemas "
                    "but has no detected X-axis rotation. The optical frame must be rotated "
                    "180° around its local X-axis to align OpenUSD (-Z forward) with ROS "
                    "(+Z forward) per REP §2.8.",
                    "Add `float xformOp:rotateX = 180` and include 'xformOp:rotateX' "
                    "in xformOpOrder on the optical frame prim.")

register_stage_validator("CameraOpticalFrame", _validate_camera_optical_frame,
    doc="REP §2.8: Camera ROS interfaces must live on an optical frame child.", section="2.8")


# --- §2.10 RosJointName ---

def _check_ros_joint_name(prim: Usd.Prim, timeRange: TimeRange):
    attr = prim.GetAttribute("ros:joint:name")
    if not attr.IsValid() or attr.Get() is None:
        yield _error(MISSING_JOINT_NAME, ErrorType.Warn, _base_site(prim),
            f"Joint prim '{prim.GetPath()}' ({prim.GetTypeName()}) is missing "
            "the 'ros:joint:name' custom property. Without it, simulators fall "
            "back to the prim name, which may not match robot descriptions or "
            "controller configurations per REP §2.10.",
            'Add `custom string ros:joint:name = "<joint_name>"` on this prim '
            "to ensure correct mapping in JointState messages and ros2_control.")

register_prim_validator("RosJointName", _check_ros_joint_name,
    doc="REP §2.10: All UsdPhysicsJoint prims should carry ros:joint:name.",
    section="2.10", schema_types=[
        "UsdPhysicsRevoluteJoint", "UsdPhysicsPrismaticJoint",
        "UsdPhysicsFixedJoint", "UsdPhysicsSphericalJoint",
        "UsdPhysicsDistanceJoint", "UsdPhysicsJoint",
    ])
