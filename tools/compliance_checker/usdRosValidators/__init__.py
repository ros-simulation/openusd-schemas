"""Entry point for the usdRosValidators plugin.

Registers the bundled plugin manifest (``plugInfo.json`` next to this
module) so the codeless ROS schema and validator metadata are discoverable,
then imports each check module to bind validator implementations to the
metadata via ``RegisterPluginPrimValidator`` / ``RegisterPluginStageValidator``.

USD imports this module on first registry hit for any ``usdRosValidators:*``
validator; running it directly from Python (e.g. via ``usd-check``) has the
same effect.
"""

from pathlib import Path

from pxr import Plug

Plug.Registry().RegisterPlugins(str(Path(__file__).parent))

from compliance_checker.checks import (  # noqa: F401, E402
    s1_1_units,
    s1_2_structure,
    s1_3_physics,
    s2_ros,
    s3_export,
)
