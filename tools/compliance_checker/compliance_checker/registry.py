"""Check registry: assembles built-in checks and loads extension plug-ins.

Extension packages register additional check classes via the entry-point
group ``"repXXXX.checks"``.  Each entry point must resolve to a
:class:`~checks.base.BaseCheck` subclass (not an instance).

Example pyproject.toml entry for an extension package::

    [project.entry-points."repXXXX.checks"]
    my_sensor = "my_sensor_pkg.checks:SensorCameraCheck"
"""

from __future__ import annotations

import importlib.metadata
import logging
from typing import TYPE_CHECKING

from .checks import (
    ArticulationRootCheck,
    AssetManagementCheck,
    CameraOpticalFrameCheck,
    CollisionGeometryAuthoringCheck,
    CollisionMaterialCheck,
    CompositionModelCheck,
    CoordinateSystemCheck,
    ExtendedPhysicsActuatorCheck,
    ExtendedPhysicsMimicCheck,
    ExtendedPhysicsPositionClampingCheck,
    GeometryConstraintsCheck,
    InheritsSpecializesCheck,
    InstanceablePhysicsCheck,
    JointLimitsCheck,
    LayerEncodingCheck,
    LightingPortabilityCheck,
    MassPropertiesCheck,
    MaterialPortabilityCheck,
    MimicJointCheck,
    ParallelSimulationInstancingCheck,
    PathConventionCheck,
    PayloadKinematicTopologyCheck,
    RosActionCheck,
    RosContextPlacementCheck,
    RosFrameAPICheck,
    RosFrameAttributesCheck,
    RosInterfacePlacementCheck,
    RosInterfaceStructureCheck,
    RosJointNameCheck,
    RosServiceCheck,
    RosTopicCheck,
    TextureBakingCheck,
    TextureFormatCheck,
    VariantDefaultCheck,
)

if TYPE_CHECKING:
    from .checks.base import BaseCheck

log = logging.getLogger(__name__)

# ------------------------------------------------------------------ #
# Built-in check catalogue                                              #
# ------------------------------------------------------------------ #

#: Checks that run by default — core REP sections §1 and §2 (base schemas only).
_CORE_CHECKS: list[type[BaseCheck]] = [
    # §1.1
    CoordinateSystemCheck,
    # §1.2
    AssetManagementCheck,
    PathConventionCheck,
    LayerEncodingCheck,
    CompositionModelCheck,
    VariantDefaultCheck,
    InheritsSpecializesCheck,
    PayloadKinematicTopologyCheck,
    ParallelSimulationInstancingCheck,
    # §1.3
    JointLimitsCheck,
    ArticulationRootCheck,
    MassPropertiesCheck,
    CollisionGeometryAuthoringCheck,
    CollisionMaterialCheck,
    MimicJointCheck,
    InstanceablePhysicsCheck,
    # §2 — core ROS schemas (RosContextAPI, RosTopicAPI, RosServiceAPI, RosActionAPI, RosFrameAPI)
    RosContextPlacementCheck,
    RosInterfacePlacementCheck,
    RosInterfaceStructureCheck,
    RosTopicCheck,
    RosServiceCheck,
    RosActionCheck,
    RosFrameAttributesCheck,
    RosFrameAPICheck,
    CameraOpticalFrameCheck,
    RosJointNameCheck,
]

#: Extended schema checks (§4) — opt-in via ``include_extended=True`` / ``--extended``.
#: Covers ExtendedPhysics* schemas and RosControl* extension schemas.
_EXTENDED_CHECKS: list[type[BaseCheck]] = [
    ExtendedPhysicsMimicCheck,
    ExtendedPhysicsActuatorCheck,
    ExtendedPhysicsPositionClampingCheck,
]

#: Export/conversion checks (§3) — opt-in via ``include_export=True`` / ``--full``.
_EXPORT_CHECKS: list[type[BaseCheck]] = [
    MaterialPortabilityCheck,
    TextureFormatCheck,
    TextureBakingCheck,
    GeometryConstraintsCheck,
    LightingPortabilityCheck,
]

_ENTRY_POINT_GROUP = "repXXXX.checks"


def _load_extension_checks() -> list[type[BaseCheck]]:
    """Load check classes registered via the ``repXXXX.checks`` entry-point group."""
    extra: list[type[BaseCheck]] = []
    try:
        eps = importlib.metadata.entry_points(group=_ENTRY_POINT_GROUP)
    except Exception as exc:  # noqa: BLE001
        log.warning("Failed to query entry-points for extensions: %s", exc)
        return extra

    for ep in eps:
        try:
            cls = ep.load()
            extra.append(cls)
            log.debug("Loaded extension check: %s from %s", cls.__name__, ep.value)
        except Exception as exc:  # noqa: BLE001
            log.warning("Failed to load extension check '%s': %s", ep.name, exc)

    return extra


def build_checks(
    include_export: bool = False,
    include_extended: bool = False,
    sections: list[str] | None = None,
    include_extensions: bool = True,
) -> list[BaseCheck]:
    """Return instantiated check objects ready to run.

    Args:
        include_export: When True, §3 export/conversion checks are included.
            These may be slow on large stages (full mesh/texture traversal).
        include_extended: When True, §4 extended schema checks are included
            (ExtendedPhysics*, RosControl* extension schemas).
        sections: Optional allowlist of section prefixes (e.g. ``["1.1", "2"]``).
            Only checks whose ``section`` starts with one of these strings are kept.
            ``None`` means all sections are included.
        include_extensions: When True (default), attempt to load plug-in checks
            registered via the ``repXXXX.checks`` entry-point group.

    Returns:
        A list of instantiated :class:`~checks.base.BaseCheck` objects.
    """
    check_types: list[type[BaseCheck]] = list(_CORE_CHECKS)

    if include_extended:
        check_types.extend(_EXTENDED_CHECKS)

    if include_export:
        check_types.extend(_EXPORT_CHECKS)

    if include_extensions:
        check_types.extend(_load_extension_checks())

    instances = [cls() for cls in check_types]

    if sections:
        instances = [
            c for c in instances if any(c.section.startswith(s) for s in sections)
        ]

    return instances
