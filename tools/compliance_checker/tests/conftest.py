"""Shared fixtures and helpers for the REP-0158 compliance checker test suite."""

from __future__ import annotations

from typing import TYPE_CHECKING

import pytest
from pxr import Sdf, Usd, UsdValidation

if TYPE_CHECKING:
    pass


import usdRosValidators  # noqa: F401


# ------------------------------------------------------------------ #
# Stage factory                                                         #
# ------------------------------------------------------------------ #


def make_stage(usda: str) -> Usd.Stage:
    """Return an in-memory USD stage loaded from a USDA string."""
    layer = Sdf.Layer.CreateAnonymous(".usda")
    layer.ImportFromString(usda)
    return Usd.Stage.Open(layer)


@pytest.fixture
def tmp_usda(tmp_path):
    """Fixture: write USDA content to a temp file, return path string."""

    def _write(content: str, name: str = "test.usda") -> str:
        p = tmp_path / name
        p.write_text(content)
        return str(p)

    return _write


# ------------------------------------------------------------------ #
# Check runner helpers                                                  #
# ------------------------------------------------------------------ #

_registry = UsdValidation.ValidationRegistry()


def run_validators(
    stage: Usd.Stage,
    *validator_names: str,
) -> list[UsdValidation.ValidationError]:
    """Run named validators against *stage*, return all errors."""
    validators = [_registry.GetOrLoadValidatorByName(n) for n in validator_names]
    ctx = UsdValidation.ValidationContext(validators)
    return list(ctx.Validate(stage))


def run_keyword(
    stage: Usd.Stage,
    keyword: str,
) -> list[UsdValidation.ValidationError]:
    """Run all validators matching *keyword* against *stage*."""
    metas = _registry.GetValidatorMetadataForKeyword(keyword)
    validators = [_registry.GetOrLoadValidatorByName(m.name) for m in metas]
    ctx = UsdValidation.ValidationContext(validators)
    return list(ctx.Validate(stage))


def ids(errors: list[UsdValidation.ValidationError]) -> set[str]:
    """Return the set of error names from an error list."""
    return {e.GetName() for e in errors}


def has(errors: list[UsdValidation.ValidationError], error_name: str) -> bool:
    """Return True when at least one error carries *error_name*."""
    return error_name in ids(errors)


def none_with(errors: list[UsdValidation.ValidationError], error_name: str) -> bool:
    """Return True when no error carries *error_name*."""
    return error_name not in ids(errors)
