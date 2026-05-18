"""Entry point for the usdRosValidators plugin.

USD imports this module on first registry hit for any
``usdRosValidators:*`` validator. The imports below trigger each check
module's registration calls, binding implementations to the metadata
declared in ``core/ros/plugin/resource/plugInfo.json``.
"""

from compliance_checker.checks import (  # noqa: F401
    s1_1_units,
    s1_2_structure,
    s1_3_physics,
    s2_ros,
    s3_export,
)
