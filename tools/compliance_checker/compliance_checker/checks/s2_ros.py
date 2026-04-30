"""Section 2 – ROS Integration Schema checks."""

from __future__ import annotations

import re
from typing import TYPE_CHECKING

from pxr import Sdf, Usd, UsdGeom, UsdPhysics

from .base import (
    ErrorType,
    TimeRange,
    ValidationError,
    _error,
    _prim_site,
    register_stage_validator,
)

if TYPE_CHECKING:
    from pxr import UsdValidation

# ------------------------------------------------------------------ #
# Helpers                                                               #
# ------------------------------------------------------------------ #

# ROS 2 name segment: starts with letter/underscore, followed by alphanumeric/underscore
_ROS_NAME_RE = re.compile(r"^/?[a-zA-Z_][a-zA-Z0-9_]*(?:/[a-zA-Z_][a-zA-Z0-9_]*)*$")

_PROHIBITED_TYPES = (
    "rosgraph_msgs/msg/Clock",
    "simulation_interfaces/",
)

_PROHIBITED_TOPIC_NAMES = ("/clock",)

_ROS_SCHEMAS = {
    "RosContextAPI",
    "RosTopicAPI",
    "RosServiceAPI",
    "RosActionAPI",
    "RosFrameAPI",
}
_INTERFACE_SCHEMAS = {"RosTopicAPI", "RosServiceAPI", "RosActionAPI"}


def _applied(prim: Usd.Prim) -> set[str]:
    """Return all applied API schema names, including unregistered custom schemas."""
    list_op = prim.GetMetadata("apiSchemas")
    if list_op is None:
        return set()
    return set(list_op.GetAppliedItems())


def _str_attr(prim: Usd.Prim, name: str) -> str | None:
    attr = prim.GetAttribute(name)
    if not attr.IsValid():
        return None
    return attr.Get()


def _bool_attr(prim: Usd.Prim, name: str) -> bool | None:
    attr = prim.GetAttribute(name)
    if not attr.IsValid():
        return None
    return attr.Get()


def _build_payload_roots(stage: Usd.Stage) -> set[Sdf.Path]:
    """Return paths of prims that introduce payload arcs."""
    roots: set[Sdf.Path] = set()
    for prim in stage.TraverseAll():
        for prim_spec in prim.GetPrimStack():
            if prim_spec.payloadList.GetAddedOrExplicitItems():
                roots.add(prim.GetPath())
                break
    return roots


def _is_inside_payload(prim: Usd.Prim, payload_roots: set[Sdf.Path]) -> bool:
    """Return True if *prim* is a descendant (not root) of a payload-loading prim."""
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
    """Validate RosContextAPI namespace according to §2.1.1."""
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
    """Return the first prim (DFS root-to-leaf) with RosContextAPI."""
    for prim in stage.Traverse():
        if "RosContextAPI" in _applied(prim):
            return prim
    return None


# ------------------------------------------------------------------ #
# Validators                                                            #
# ------------------------------------------------------------------ #


def _validate_ros_context_placement(
    stage: Usd.Stage, timeRange: TimeRange
) -> list[ValidationError]:
    """REP §2.1: RosContextAPI prims must reside outside payload arcs."""
    errors: list[ValidationError] = []
    payload_roots = _build_payload_roots(stage)
    outermost = _find_outermost_context(stage)

    for prim in stage.TraverseAll():
        applied = _applied(prim)
        if "RosContextAPI" not in applied:
            continue
        prim_path = str(prim.GetPath())

        if _is_inside_payload(prim, payload_roots):
            errors.append(_error(
                "2.1.1",
                ErrorType.Warn,
                _prim_site(stage, prim_path),
                (
                    f"RosContextAPI prim '{prim_path}' is inside a payload arc. "
                    "The namespace graph must be resolvable without loading heavy geometry; "
                    "RosContextAPI prims must reside outside payloads per REP §2.1."
                ),
                "Move the RosContextAPI prim above the payload boundary.",
            ))

        namespace = _str_attr(prim, "ros:context:namespace")
        if namespace and not _validate_context_namespace(namespace):
            errors.append(_error(
                "2.1.3",
                ErrorType.Error,
                _prim_site(stage, prim_path),
                (
                    f"RosContextAPI namespace '{namespace}' on '{prim_path}' "
                    "violates §2.1.1 rules. Namespaces must be either composable "
                    "(no leading/trailing slash) or absolute (leading slash), and "
                    "must not use '~' or '{{}}' substitutions."
                ),
                (
                    "Use a valid namespace such as 'robot_1', 'left_camera', "
                    "or '/global_ns'."
                ),
            ))

        # parent_frame is only valid on the outermost context
        if outermost and prim != outermost:
            pf_attr = prim.GetAttribute("ros:context:parent_frame")
            if pf_attr.IsValid() and pf_attr.Get() is not None:
                errors.append(_error(
                    "2.1.2",
                    ErrorType.Warn,
                    _prim_site(stage, prim_path),
                    (
                        f"Nested RosContextAPI '{prim_path}' sets "
                        "ros:context:parent_frame, which is only valid on the outermost "
                        "context in the stage per REP §2.1.1. This attribute will be ignored."
                    ),
                    (
                        "Remove ros:context:parent_frame from all contexts except the "
                        "top-most one."
                    ),
                ))

    return errors


register_stage_validator(
    "RosContextPlacement",
    _validate_ros_context_placement,
    doc="REP §2.1: RosContextAPI prims must reside outside payload arcs.",
    section="2.1",
)


def _validate_ros_interface_placement(
    stage: Usd.Stage, timeRange: TimeRange
) -> list[ValidationError]:
    """REP §2.2: RosTopicAPI / RosServiceAPI / RosActionAPI must reside outside payload arcs."""
    errors: list[ValidationError] = []
    payload_roots = _build_payload_roots(stage)
    for prim in stage.TraverseAll():
        applied = _applied(prim)
        in_payload = _is_inside_payload(prim, payload_roots)
        for schema in _INTERFACE_SCHEMAS:
            if schema in applied and in_payload:
                prim_path = str(prim.GetPath())
                errors.append(_error(
                    "2.2.1",
                    ErrorType.Error,
                    _prim_site(stage, prim_path),
                    (
                        f"Prim '{prim_path}' has {schema} but is inside a payload arc. "
                        "Interface prims must reside in the lightweight, traversable "
                        "kinematic hierarchy (outside payloads) per REP §2.2."
                    ),
                    (
                        "Move the prim (or its schema) above the payload boundary, "
                        "following the ETL pattern from §1.2.1."
                    ),
                ))
    return errors


register_stage_validator(
    "RosInterfacePlacement",
    _validate_ros_interface_placement,
    doc="REP §2.2: RosTopicAPI / RosServiceAPI / RosActionAPI must reside outside payload arcs.",
    section="2.2",
)


# ------------------------------------------------------------------ #
# Interface structure helpers                                           #
# ------------------------------------------------------------------ #


def _check_non_physical_placement(
    stage: Usd.Stage, prim: Usd.Prim
) -> list[ValidationError]:
    errors: list[ValidationError] = []
    if prim.HasAPI(UsdPhysics.RigidBodyAPI):
        errors.append(_error(
            "2.2.2",
            ErrorType.Error,
            _prim_site(stage, str(prim.GetPath())),
            (
                f"Prim '{prim.GetPath()}' carries ROS interface schemas and "
                "PhysicsRigidBodyAPI. Interface schemas should be placed on "
                "dedicated logical prims rather than physical rigid-body prims per REP §2.2."
            ),
            (
                "Move RosTopicAPI/RosServiceAPI/RosActionAPI schemas to a dedicated "
                "child Xform (for sensor interfaces) or a logical interfaces scope."
            ),
        ))
    return errors


def _check_one_interface_per_prim(
    stage: Usd.Stage, prim: Usd.Prim, present: set[str]
) -> list[ValidationError]:
    errors: list[ValidationError] = []
    if len(present) > 1:
        errors.append(_error(
            "2.2.3",
            ErrorType.Error,
            _prim_site(stage, str(prim.GetPath())),
            (
                f"Prim '{prim.GetPath()}' carries multiple interface schemas "
                f"{sorted(present)}. Sensor interfaces must use one interface "
                "schema per prim per REP §2.2."
            ),
            (
                "Split interfaces across separate child prims (for example, one prim "
                "for RosTopicAPI image_raw and another prim for camera_info)."
            ),
        ))
    return errors


def _check_sensor_child_placement(
    stage: Usd.Stage, prim: Usd.Prim
) -> list[ValidationError]:
    errors: list[ValidationError] = []
    rigid_ancestor = _nearest_rigid_body_ancestor(prim)
    if not rigid_ancestor:
        return errors
    parent = prim.GetParent()
    is_direct_child = parent and parent.IsValid() and parent == rigid_ancestor
    is_xform = prim.GetTypeName() == "Xform"
    if not is_direct_child or not is_xform:
        errors.append(_error(
            "2.2.4",
            ErrorType.Warn,
            _prim_site(stage, str(prim.GetPath())),
            (
                f"Sensor interface prim '{prim.GetPath()}' is under rigid body "
                f"'{rigid_ancestor.GetPath()}', but sensor interfaces should be authored "
                "on a direct child UsdGeomXform of the physical link per REP §2.2."
            ),
            (
                "Place this interface on a direct child Xform under the rigid body link."
            ),
        ))
    return errors


def _validate_ros_interface_structure(
    stage: Usd.Stage, timeRange: TimeRange
) -> list[ValidationError]:
    """REP §2.2: interface prim structure for robot-wide and sensor interfaces."""
    errors: list[ValidationError] = []
    for prim in stage.TraverseAll():
        applied = _applied(prim)
        present = _INTERFACE_SCHEMAS & applied
        if not present:
            continue
        errors.extend(_check_non_physical_placement(stage, prim))
        errors.extend(_check_one_interface_per_prim(stage, prim, present))
        errors.extend(_check_sensor_child_placement(stage, prim))
    return errors


register_stage_validator(
    "RosInterfaceStructure",
    _validate_ros_interface_structure,
    doc="REP §2.2: interface prim structure for robot-wide and sensor interfaces.",
    section="2.2",
)


# ------------------------------------------------------------------ #
# Topic check helpers                                                   #
# ------------------------------------------------------------------ #

_ALLOWED_RELIABILITY = {"system_default", "reliable", "best_effort"}
_ALLOWED_DURABILITY = {"system_default", "transient_local", "volatile"}
_ALLOWED_HISTORY = {"system_default", "keep_last", "keep_all"}
_ALLOWED_TOPIC_ROLES = {"publisher", "subscription"}


def _check_ros_name(
    stage: Usd.Stage, prim_path: str, attr: str, name: str, check_id: str
) -> list[ValidationError]:
    errors: list[ValidationError] = []
    if not _validate_ros_name(name):
        errors.append(_error(
            check_id,
            ErrorType.Error,
            _prim_site(stage, prim_path),
            (
                f"'{attr}' value '{name}' on '{prim_path}' violates ROS 2 naming rules. "
                "Names must contain only alphanumeric characters, underscores, and "
                "forward slashes, and must not start with a number."
            ),
            (
                "Rename to a valid ROS 2 topic/service/action name "
                "(e.g. 'joint_states', '/robot_1/cmd_vel')."
            ),
        ))
    return errors


def _check_prohibited_names(
    stage: Usd.Stage, prim_path: str, name: str, type_: str | None
) -> list[ValidationError]:
    errors: list[ValidationError] = []
    for prohibited in _PROHIBITED_TOPIC_NAMES:
        if name == prohibited or name.endswith(prohibited):
            errors.append(_error(
                "2.9.1",
                ErrorType.Error,
                _prim_site(stage, prim_path),
                (
                    f"Topic name '{name}' is a prohibited simulator-level interface "
                    "(/clock). Assets must not include simulator-level interfaces per REP §2.9."
                ),
                "Remove this interface from the asset.",
            ))
    return errors


def _check_prohibited_types(
    stage: Usd.Stage, prim_path: str, type_: str
) -> list[ValidationError]:
    errors: list[ValidationError] = []
    for prohibited in _PROHIBITED_TYPES:
        if type_.startswith(prohibited):
            errors.append(_error(
                "2.9.2",
                ErrorType.Error,
                _prim_site(stage, prim_path),
                (
                    f"Interface type '{type_}' on '{prim_path}' is a prohibited "
                    "simulator-level interface. Assets must not include interfaces from "
                    "simulation_interfaces or rosgraph_msgs/Clock per REP §2.9."
                ),
                "Remove this interface from the asset.",
            ))
    return errors


def _check_topic_qos(
    stage: Usd.Stage, prim: Usd.Prim
) -> list[ValidationError]:
    errors: list[ValidationError] = []
    prim_path = str(prim.GetPath())

    def check_token(attr_name: str, allowed: set[str]) -> list[ValidationError]:
        inner: list[ValidationError] = []
        val = _str_attr(prim, attr_name)
        if val is not None and val not in allowed:
            inner.append(_error(
                "2.4.5",
                ErrorType.Warn,
                _prim_site(stage, prim_path),
                (
                    f"QoS attribute '{attr_name}' on '{prim_path}' has value '{val}' "
                    f"which is not in the allowed set {sorted(allowed)}."
                ),
                f"Use one of: {sorted(allowed)}.",
            ))
        return inner

    errors.extend(check_token("ros:topic:qos:reliability", _ALLOWED_RELIABILITY))
    errors.extend(check_token("ros:topic:qos:durability", _ALLOWED_DURABILITY))
    errors.extend(check_token("ros:topic:qos:history", _ALLOWED_HISTORY))
    errors.extend(_check_topic_qos_match_publisher(stage, prim))
    errors.extend(_check_topic_qos_depth(stage, prim))
    return errors


def _check_topic_qos_match_publisher(
    stage: Usd.Stage, prim: Usd.Prim
) -> list[ValidationError]:
    errors: list[ValidationError] = []
    prim_path = str(prim.GetPath())
    role = _str_attr(prim, "ros:topic:role")
    attr = prim.GetAttribute("ros:topic:qos:match_publisher")
    if not attr.IsValid():
        return errors
    value = attr.Get()
    if not isinstance(value, bool):
        errors.append(_error(
            "2.4.8",
            ErrorType.Error,
            _prim_site(stage, prim_path),
            (
                f"'ros:topic:qos:match_publisher' on '{prim_path}' must be a bool. "
                f"Got {type(value).__name__}."
            ),
            "Use `bool ros:topic:qos:match_publisher = true|false`.",
        ))
        return errors
    if value and role == "publisher":
        errors.append(_error(
            "2.4.8",
            ErrorType.Warn,
            _prim_site(stage, prim_path),
            (
                f"'ros:topic:qos:match_publisher' is true on publisher '{prim_path}'. "
                "This QoS option is only applicable to subscriptions per REP §2.4."
            ),
            "Unset this attribute on publishers.",
        ))
    return errors


def _check_topic_qos_depth(
    stage: Usd.Stage, prim: Usd.Prim
) -> list[ValidationError]:
    errors: list[ValidationError] = []
    prim_path = str(prim.GetPath())
    depth_attr = prim.GetAttribute("ros:topic:qos:depth")
    if not depth_attr.IsValid():
        return errors
    depth = depth_attr.Get()
    if not isinstance(depth, int):
        errors.append(_error(
            "2.4.9",
            ErrorType.Error,
            _prim_site(stage, prim_path),
            (
                f"'ros:topic:qos:depth' on '{prim_path}' must be an int. "
                f"Got {type(depth).__name__}."
            ),
            "Use a positive integer depth.",
        ))
        return errors
    history = _str_attr(prim, "ros:topic:qos:history")
    if history == "keep_last" and depth <= 0:
        errors.append(_error(
            "2.4.9",
            ErrorType.Error,
            _prim_site(stage, prim_path),
            (
                f"'ros:topic:qos:depth' on '{prim_path}' is {depth}. "
                "Depth must be > 0 when history is keep_last."
            ),
            "Set `ros:topic:qos:depth` to a positive integer.",
        ))
    return errors


def _check_topic_starts_enabled(
    stage: Usd.Stage, prim: Usd.Prim
) -> list[ValidationError]:
    errors: list[ValidationError] = []
    prim_path = str(prim.GetPath())
    attr = prim.GetAttribute("ros:topic:starts_enabled")
    if not attr.IsValid():
        return errors
    value = _bool_attr(prim, "ros:topic:starts_enabled")
    if not isinstance(value, bool):
        errors.append(_error(
            "2.4.6",
            ErrorType.Error,
            _prim_site(stage, prim_path),
            (
                f"'ros:topic:starts_enabled' on '{prim_path}' must be a bool. "
                f"Got {type(value).__name__}."
            ),
            "Use `bool ros:topic:starts_enabled = true|false`.",
        ))
    return errors


def _check_topic_override_frame_id(
    stage: Usd.Stage, prim: Usd.Prim
) -> list[ValidationError]:
    errors: list[ValidationError] = []
    prim_path = str(prim.GetPath())
    attr = prim.GetAttribute("ros:topic:override_frame_id")
    if not attr.IsValid():
        return errors
    value = _str_attr(prim, "ros:topic:override_frame_id")
    if value is None:
        return errors
    if not isinstance(value, str):
        errors.append(_error(
            "2.4.7",
            ErrorType.Error,
            _prim_site(stage, prim_path),
            (
                f"'ros:topic:override_frame_id' on '{prim_path}' must be a string. "
                f"Got {type(value).__name__}."
            ),
            'Use `string ros:topic:override_frame_id = "map"`.',
        ))
        return errors
    if value and not _validate_ros_name(value):
        errors.append(_error(
            "2.4.7",
            ErrorType.Warn,
            _prim_site(stage, prim_path),
            (
                f"'ros:topic:override_frame_id' value '{value}' on '{prim_path}' "
                "does not follow ROS naming rules."
            ),
            "Use a valid ROS frame name (e.g. 'map', 'earth', '/robot/base_link').",
        ))
    return errors


def _check_topic(
    stage: Usd.Stage, prim: Usd.Prim
) -> list[ValidationError]:
    errors: list[ValidationError] = []
    prim_path = str(prim.GetPath())

    role = _str_attr(prim, "ros:topic:role")
    name = _str_attr(prim, "ros:topic:name")
    type_ = _str_attr(prim, "ros:topic:type")

    # Required: role
    if not role:
        errors.append(_error(
            "2.4.1",
            ErrorType.Error,
            _prim_site(stage, prim_path),
            f"RosTopicAPI prim '{prim_path}' is missing required 'ros:topic:role'.",
            'Set `token ros:topic:role = "publisher"` or `"subscription"`.',
        ))
    elif role not in _ALLOWED_TOPIC_ROLES:
        errors.append(_error(
            "2.4.1",
            ErrorType.Error,
            _prim_site(stage, prim_path),
            (
                f"RosTopicAPI prim '{prim_path}' has invalid role '{role}'. "
                f"Allowed values: {sorted(_ALLOWED_TOPIC_ROLES)}."
            ),
            "Use 'publisher' or 'subscription'.",
        ))

    # Required: name
    if not name:
        errors.append(_error(
            "2.4.2",
            ErrorType.Error,
            _prim_site(stage, prim_path),
            f"RosTopicAPI prim '{prim_path}' is missing required 'ros:topic:name'.",
            'Set `string ros:topic:name = "<topic_name>"`.',
        ))
    else:
        errors.extend(_check_ros_name(stage, prim_path, "ros:topic:name", name, "2.3.1"))
        errors.extend(_check_prohibited_names(stage, prim_path, name, type_))

    # Required: type
    if not type_:
        errors.append(_error(
            "2.4.3",
            ErrorType.Error,
            _prim_site(stage, prim_path),
            f"RosTopicAPI prim '{prim_path}' is missing required 'ros:topic:type'.",
            'Set `string ros:topic:type = "<pkg>/msg/<Type>"`.',
        ))
    else:
        errors.extend(_check_prohibited_types(stage, prim_path, type_))

    # Required for publishers: publish_rate
    if role == "publisher":
        rate_attr = prim.GetAttribute("ros:topic:publish_rate")
        if not rate_attr.IsValid() or rate_attr.Get() is None:
            errors.append(_error(
                "2.4.4",
                ErrorType.Error,
                _prim_site(stage, prim_path),
                (
                    f"Publisher RosTopicAPI prim '{prim_path}' is missing "
                    "'ros:topic:publish_rate'. Required for all publishers per REP §2.4."
                ),
                "Set `double ros:topic:publish_rate = <Hz>`.",
            ))

    # QoS token validation
    errors.extend(_check_topic_qos(stage, prim))
    errors.extend(_check_topic_starts_enabled(stage, prim))
    errors.extend(_check_topic_override_frame_id(stage, prim))
    return errors


def _validate_ros_topic(
    stage: Usd.Stage, timeRange: TimeRange
) -> list[ValidationError]:
    """REP §2.4: Validate RosTopicAPI required attributes and QoS values."""
    errors: list[ValidationError] = []
    for prim in stage.TraverseAll():
        if "RosTopicAPI" not in _applied(prim):
            continue
        errors.extend(_check_topic(stage, prim))
    return errors


register_stage_validator(
    "RosTopic",
    _validate_ros_topic,
    doc="REP §2.4: Validate RosTopicAPI required attributes and QoS values.",
    section="2.4",
)


# ------------------------------------------------------------------ #
# Service check                                                         #
# ------------------------------------------------------------------ #

_ALLOWED_SERVICE_ROLES = {"server", "client"}


def _check_service_starts_enabled(
    stage: Usd.Stage, prim: Usd.Prim
) -> list[ValidationError]:
    errors: list[ValidationError] = []
    prim_path = str(prim.GetPath())
    attr = prim.GetAttribute("ros:service:starts_enabled")
    if not attr.IsValid():
        return errors
    value = _bool_attr(prim, "ros:service:starts_enabled")
    if not isinstance(value, bool):
        errors.append(_error(
            "2.5.2",
            ErrorType.Error,
            _prim_site(stage, prim_path),
            (
                f"'ros:service:starts_enabled' on '{prim_path}' must be a bool. "
                f"Got {type(value).__name__}."
            ),
            "Use `bool ros:service:starts_enabled = true|false`.",
        ))
    return errors


def _check_service(
    stage: Usd.Stage, prim: Usd.Prim
) -> list[ValidationError]:
    errors: list[ValidationError] = []
    prim_path = str(prim.GetPath())
    for attr_name, check_id, hint in (
        ("ros:service:role", "2.5.1", "Use 'server' or 'client'."),
        ("ros:service:name", "2.5.1", "Set a valid ROS 2 service name."),
        (
            "ros:service:type",
            "2.5.1",
            'Set `string ros:service:type = "<pkg>/srv/<Type>"`',
        ),
    ):
        val = _str_attr(prim, attr_name)
        if not val:
            errors.append(_error(
                check_id,
                ErrorType.Error,
                _prim_site(stage, prim_path),
                (
                    f"RosServiceAPI prim '{prim_path}' is missing required '{attr_name}'."
                ),
                hint,
            ))

    role = _str_attr(prim, "ros:service:role")
    if role and role not in _ALLOWED_SERVICE_ROLES:
        errors.append(_error(
            "2.5.1",
            ErrorType.Error,
            _prim_site(stage, prim_path),
            (
                f"RosServiceAPI prim '{prim_path}' has invalid role '{role}'. "
                f"Allowed: {sorted(_ALLOWED_SERVICE_ROLES)}."
            ),
            "Use 'server' or 'client'.",
        ))

    name = _str_attr(prim, "ros:service:name")
    if name and not _validate_ros_name(name):
        errors.append(_error(
            "2.3.1",
            ErrorType.Error,
            _prim_site(stage, prim_path),
            (
                f"'ros:service:name' value '{name}' on '{prim_path}' violates ROS 2 "
                "naming rules."
            ),
            "Use only alphanumeric characters, underscores, and forward slashes.",
        ))

    type_ = _str_attr(prim, "ros:service:type")
    if type_:
        for prohibited in _PROHIBITED_TYPES:
            if type_.startswith(prohibited):
                errors.append(_error(
                    "2.9.2",
                    ErrorType.Error,
                    _prim_site(stage, prim_path),
                    (
                        f"Service type '{type_}' on '{prim_path}' is a prohibited "
                        "simulator-level interface. Assets must not include interfaces from "
                        "simulation_interfaces or rosgraph_msgs/Clock per REP §2.9."
                    ),
                    "Remove this interface from the asset.",
                ))
                break
    errors.extend(_check_service_starts_enabled(stage, prim))
    return errors


def _validate_ros_service(
    stage: Usd.Stage, timeRange: TimeRange
) -> list[ValidationError]:
    """REP §2.5: Validate RosServiceAPI required attributes."""
    errors: list[ValidationError] = []
    for prim in stage.TraverseAll():
        if "RosServiceAPI" not in _applied(prim):
            continue
        errors.extend(_check_service(stage, prim))
    return errors


register_stage_validator(
    "RosService",
    _validate_ros_service,
    doc="REP §2.5: Validate RosServiceAPI required attributes.",
    section="2.5",
)


# ------------------------------------------------------------------ #
# Action check                                                          #
# ------------------------------------------------------------------ #

_ALLOWED_ACTION_ROLES = {"server", "client"}


def _check_action_starts_enabled(
    stage: Usd.Stage, prim: Usd.Prim
) -> list[ValidationError]:
    errors: list[ValidationError] = []
    prim_path = str(prim.GetPath())
    attr = prim.GetAttribute("ros:action:starts_enabled")
    if not attr.IsValid():
        return errors
    value = _bool_attr(prim, "ros:action:starts_enabled")
    if not isinstance(value, bool):
        errors.append(_error(
            "2.6.2",
            ErrorType.Error,
            _prim_site(stage, prim_path),
            (
                f"'ros:action:starts_enabled' on '{prim_path}' must be a bool. "
                f"Got {type(value).__name__}."
            ),
            "Use `bool ros:action:starts_enabled = true|false`.",
        ))
    return errors


def _check_action(
    stage: Usd.Stage, prim: Usd.Prim
) -> list[ValidationError]:
    errors: list[ValidationError] = []
    prim_path = str(prim.GetPath())
    for attr_name, check_id, hint in (
        ("ros:action:role", "2.6.1", "Use 'server' or 'client'."),
        ("ros:action:name", "2.6.1", "Set a valid ROS 2 action name."),
        (
            "ros:action:type",
            "2.6.1",
            'Set `string ros:action:type = "<pkg>/action/<Type>"`',
        ),
    ):
        val = _str_attr(prim, attr_name)
        if not val:
            errors.append(_error(
                check_id,
                ErrorType.Error,
                _prim_site(stage, prim_path),
                (
                    f"RosActionAPI prim '{prim_path}' is missing required '{attr_name}'."
                ),
                hint,
            ))

    role = _str_attr(prim, "ros:action:role")
    if role and role not in _ALLOWED_ACTION_ROLES:
        errors.append(_error(
            "2.6.1",
            ErrorType.Error,
            _prim_site(stage, prim_path),
            (
                f"RosActionAPI prim '{prim_path}' has invalid role '{role}'. "
                f"Allowed: {sorted(_ALLOWED_ACTION_ROLES)}."
            ),
            "Use 'server' or 'client'.",
        ))

    name = _str_attr(prim, "ros:action:name")
    if name and not _validate_ros_name(name):
        errors.append(_error(
            "2.3.1",
            ErrorType.Error,
            _prim_site(stage, prim_path),
            (
                f"'ros:action:name' value '{name}' on '{prim_path}' violates ROS 2 "
                "naming rules."
            ),
            "Use only alphanumeric characters, underscores, and forward slashes.",
        ))
    errors.extend(_check_action_starts_enabled(stage, prim))
    return errors


def _validate_ros_action(
    stage: Usd.Stage, timeRange: TimeRange
) -> list[ValidationError]:
    """REP §2.6: Validate RosActionAPI required attributes."""
    errors: list[ValidationError] = []
    for prim in stage.TraverseAll():
        if "RosActionAPI" not in _applied(prim):
            continue
        errors.extend(_check_action(stage, prim))
    return errors


register_stage_validator(
    "RosAction",
    _validate_ros_action,
    doc="REP §2.6: Validate RosActionAPI required attributes.",
    section="2.6",
)


# ------------------------------------------------------------------ #
# Frame checks                                                          #
# ------------------------------------------------------------------ #


def _validate_ros_frame_api(
    stage: Usd.Stage, timeRange: TimeRange
) -> list[ValidationError]:
    """REP §2.7: RosFrameAPI should not duplicate implicit TF from PhysicsRigidBodyAPI."""
    errors: list[ValidationError] = []
    for prim in stage.TraverseAll():
        applied = _applied(prim)
        if "RosFrameAPI" not in applied:
            continue
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            errors.append(_error(
                "2.7.1",
                ErrorType.Warn,
                _prim_site(stage, str(prim.GetPath())),
                (
                    f"Prim '{prim.GetPath()}' has both RosFrameAPI and PhysicsRigidBodyAPI. "
                    "Physical links connected via joints receive implicit TF broadcasting; "
                    "explicit RosFrameAPI is redundant and may cause duplicate frames."
                ),
                (
                    "Remove RosFrameAPI from prims that already carry PhysicsRigidBodyAPI. "
                    "Use RosFrameAPI only for non-physical dummy frames."
                ),
            ))
    return errors


register_stage_validator(
    "RosFrameAPI",
    _validate_ros_frame_api,
    doc="REP §2.7: RosFrameAPI should not duplicate implicit TF from PhysicsRigidBodyAPI.",
    section="2.7",
)


def _check_frame_id(
    stage: Usd.Stage, prim: Usd.Prim
) -> list[ValidationError]:
    errors: list[ValidationError] = []
    prim_path = str(prim.GetPath())
    attr = prim.GetAttribute("ros:frame:id")
    if not attr.IsValid():
        return errors
    value = _str_attr(prim, "ros:frame:id")
    if value is None:
        return errors
    if not isinstance(value, str):
        errors.append(_error(
            "2.7.2",
            ErrorType.Error,
            _prim_site(stage, prim_path),
            (
                f"'ros:frame:id' on '{prim_path}' must be a string. "
                f"Got {type(value).__name__}."
            ),
            'Use `string ros:frame:id = "camera_optical_frame"`.',
        ))
        return errors
    if value and not _validate_ros_name(value):
        errors.append(_error(
            "2.7.2",
            ErrorType.Warn,
            _prim_site(stage, prim_path),
            (
                f"'ros:frame:id' value '{value}' on '{prim_path}' does not "
                "follow ROS naming rules."
            ),
            "Use a valid TF frame name (e.g. 'base_link', 'camera_optical_frame').",
        ))
    return errors


def _check_frame_static(
    stage: Usd.Stage, prim: Usd.Prim
) -> list[ValidationError]:
    errors: list[ValidationError] = []
    prim_path = str(prim.GetPath())
    attr = prim.GetAttribute("ros:frame:static")
    if not attr.IsValid():
        return errors
    value = _bool_attr(prim, "ros:frame:static")
    if not isinstance(value, bool):
        errors.append(_error(
            "2.7.3",
            ErrorType.Error,
            _prim_site(stage, prim_path),
            (
                f"'ros:frame:static' on '{prim_path}' must be a bool. "
                f"Got {type(value).__name__}."
            ),
            "Use `bool ros:frame:static = true|false`.",
        ))
    return errors


def _validate_ros_frame_attributes(
    stage: Usd.Stage, timeRange: TimeRange
) -> list[ValidationError]:
    """REP §2.7: Validate RosFrameAPI attribute types and frame-id naming."""
    errors: list[ValidationError] = []
    for prim in stage.TraverseAll():
        if "RosFrameAPI" not in _applied(prim):
            continue
        errors.extend(_check_frame_id(stage, prim))
        errors.extend(_check_frame_static(stage, prim))
    return errors


register_stage_validator(
    "RosFrameAttributes",
    _validate_ros_frame_attributes,
    doc="REP §2.7: Validate RosFrameAPI attribute types and frame-id naming.",
    section="2.7",
)


# ------------------------------------------------------------------ #
# Camera optical frame check                                            #
# ------------------------------------------------------------------ #


def _check_optical_rotation(
    stage: Usd.Stage, prim: Usd.Prim
) -> list[ValidationError]:
    errors: list[ValidationError] = []
    xformable = UsdGeom.Xformable(prim)
    if not xformable:
        return errors
    has_x_rotation = False
    for op in xformable.GetOrderedXformOps():
        op_name = op.GetOpName().lower()
        if "rotatex" in op_name or "orient" in op_name:
            has_x_rotation = True
            break
    if not has_x_rotation:
        errors.append(_error(
            "2.8.1",
            ErrorType.Warn,
            _prim_site(stage, str(prim.GetPath())),
            (
                f"Camera child prim '{prim.GetPath()}' carries ROS interface schemas "
                "but has no detected X-axis rotation. The optical frame must be rotated "
                "180° around its local X-axis to align OpenUSD (-Z forward) with ROS "
                "(+Z forward) per REP §2.8."
            ),
            (
                "Add `float xformOp:rotateX = 180` and include 'xformOp:rotateX' "
                "in xformOpOrder on the optical frame prim."
            ),
        ))
    return errors


def _validate_camera_optical_frame(
    stage: Usd.Stage, timeRange: TimeRange
) -> list[ValidationError]:
    """REP §2.8: Camera ROS interfaces must live on an optical frame child, not the camera itself."""
    errors: list[ValidationError] = []
    for prim in stage.TraverseAll():
        if prim.GetTypeName() != "Camera":
            continue
        applied = _applied(prim)
        # If RosTopicAPI is applied directly to the camera, that's wrong
        if "RosTopicAPI" in applied:
            errors.append(_error(
                "2.8.1",
                ErrorType.Warn,
                _prim_site(stage, str(prim.GetPath())),
                (
                    f"RosTopicAPI is applied directly to Camera prim '{prim.GetPath()}'. "
                    "OpenUSD cameras face -Z; ROS optical frames must face +Z. "
                    "Authors must create a child Xform rotated 180° around X and apply "
                    "RosTopicAPI there per REP §2.8."
                ),
                (
                    "Create a child UsdGeomXform (e.g. 'camera_optical_frame') rotated "
                    "180° around local X and move all RosTopicAPI / RosFrameAPI schemas to it."
                ),
            ))
            continue

        # Check children for optical frame with RosTopicAPI
        for child in prim.GetAllChildren():
            if "RosTopicAPI" in _applied(child) or "RosFrameAPI" in _applied(child):
                errors.extend(_check_optical_rotation(stage, child))
    return errors


register_stage_validator(
    "CameraOpticalFrame",
    _validate_camera_optical_frame,
    doc="REP §2.8: Camera ROS interfaces must live on an optical frame child, not the camera itself.",
    section="2.8",
)


# ------------------------------------------------------------------ #
# Joint name check                                                      #
# ------------------------------------------------------------------ #

_JOINT_TYPES = {
    "PhysicsRevoluteJoint",
    "PhysicsPrismaticJoint",
    "PhysicsFixedJoint",
    "PhysicsSphericalJoint",
    "PhysicsDistanceJoint",
    "PhysicsJoint",
}


def _validate_ros_joint_name(
    stage: Usd.Stage, timeRange: TimeRange
) -> list[ValidationError]:
    """REP §2.10: All UsdPhysicsJoint prims should carry ros:joint:name."""
    errors: list[ValidationError] = []
    for prim in stage.TraverseAll():
        if prim.GetTypeName() not in _JOINT_TYPES:
            continue
        attr = prim.GetAttribute("ros:joint:name")
        if not attr.IsValid() or attr.Get() is None:
            errors.append(_error(
                "2.10.1",
                ErrorType.Warn,
                _prim_site(stage, str(prim.GetPath())),
                (
                    f"Joint prim '{prim.GetPath()}' ({prim.GetTypeName()}) is missing "
                    "the 'ros:joint:name' custom property. Without it, simulators fall "
                    "back to the prim name, which may not match robot descriptions or "
                    "controller configurations per REP §2.10."
                ),
                (
                    'Add `custom string ros:joint:name = "<joint_name>"` on this prim '
                    "to ensure correct mapping in JointState messages and ros2_control."
                ),
            ))
    return errors


register_stage_validator(
    "RosJointName",
    _validate_ros_joint_name,
    doc="REP §2.10: All UsdPhysicsJoint prims should carry ros:joint:name.",
    section="2.10",
)
