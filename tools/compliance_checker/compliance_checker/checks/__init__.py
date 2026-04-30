"""Default check classes, grouped by REP section."""

from .s1_1_units import CoordinateSystemCheck
from .s1_2_structure import (
    AssetManagementCheck,
    CompositionModelCheck,
    InheritsSpecializesCheck,
    LayerEncodingCheck,
    ParallelSimulationInstancingCheck,
    PathConventionCheck,
    PayloadKinematicTopologyCheck,
    VariantDefaultCheck,
)
from .s1_3_physics import (
    ArticulationRootCheck,
    CollisionGeometryAuthoringCheck,
    CollisionMaterialCheck,
    InstanceablePhysicsCheck,
    JointLimitsCheck,
    MassPropertiesCheck,
    MimicJointCheck,
)
from .s2_ros import (
    CameraOpticalFrameCheck,
    RosActionCheck,
    RosContextPlacementCheck,
    RosFrameAPICheck,
    RosFrameAttributesCheck,
    RosInterfacePlacementCheck,
    RosInterfaceStructureCheck,
    RosJointNameCheck,
    RosServiceCheck,
    RosTopicCheck,
)
from .s3_export import (
    GeometryConstraintsCheck,
    LightingPortabilityCheck,
    MaterialPortabilityCheck,
    TextureBakingCheck,
    TextureFormatCheck,
)
from .s4_extended_physics import (
    ExtendedPhysicsActuatorCheck,
    ExtendedPhysicsMimicCheck,
    ExtendedPhysicsPositionClampingCheck,
)

__all__ = [
    "ArticulationRootCheck",
    "AssetManagementCheck",
    "CameraOpticalFrameCheck",
    "CollisionGeometryAuthoringCheck",
    "CollisionMaterialCheck",
    "CompositionModelCheck",
    "CoordinateSystemCheck",
    "ExtendedPhysicsActuatorCheck",
    "ExtendedPhysicsMimicCheck",
    "ExtendedPhysicsPositionClampingCheck",
    "GeometryConstraintsCheck",
    "InheritsSpecializesCheck",
    "InstanceablePhysicsCheck",
    "JointLimitsCheck",
    "LayerEncodingCheck",
    "LightingPortabilityCheck",
    "MassPropertiesCheck",
    "MaterialPortabilityCheck",
    "MimicJointCheck",
    "ParallelSimulationInstancingCheck",
    "PathConventionCheck",
    "PayloadKinematicTopologyCheck",
    "RosActionCheck",
    "RosContextPlacementCheck",
    "RosFrameAPICheck",
    "RosFrameAttributesCheck",
    "RosInterfacePlacementCheck",
    "RosInterfaceStructureCheck",
    "RosJointNameCheck",
    "RosServiceCheck",
    "RosTopicCheck",
    "TextureBakingCheck",
    "TextureFormatCheck",
    "VariantDefaultCheck",
]
