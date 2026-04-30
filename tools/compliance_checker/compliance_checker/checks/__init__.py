"""REP-0158 check modules.

Importing a check module registers its validators with the UsdValidation
registry. The CLI imports all modules via _ensure_checks_loaded().
"""
