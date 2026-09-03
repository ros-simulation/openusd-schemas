"""REP-0158 OpenUSD Simulation Asset Compliance Checker.

Built on the UsdValidation framework. Importing this package eagerly
imports ``usdRosValidators`` so the bundled plugin manifest is registered
with ``Plug.Registry()`` before any code instantiates
``UsdValidation.ValidationRegistry`` (whose singleton constructor snapshots
plugin metadata once at first access).
"""

import usdRosValidators  # noqa: F401
