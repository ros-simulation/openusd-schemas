"""Registration helpers for UsdValidation-based compliance checks."""

from __future__ import annotations

from typing import TYPE_CHECKING, Callable

from pxr import Sdf, UsdValidation

if TYPE_CHECKING:
    from pxr import Usd

_registry = UsdValidation.ValidationRegistry()

ErrorType = UsdValidation.ValidationErrorType
ValidationError = UsdValidation.ValidationError
ErrorSite = UsdValidation.ValidationErrorSite
TimeRange = UsdValidation.ValidationTimeRange

StageValidatorFn = Callable[
    ["Usd.Stage", TimeRange], list[ValidationError]
]


def _stage_site(stage: "Usd.Stage") -> list[ErrorSite]:
    return [ErrorSite(stage, Sdf.Path.absoluteRootPath)]


def _prim_site(stage: "Usd.Stage", prim_path: str) -> list[ErrorSite]:
    return [ErrorSite(stage, Sdf.Path(prim_path))]


def _layer_site(layer: "Sdf.Layer") -> list[ErrorSite]:
    return [ErrorSite(layer, Sdf.Path.absoluteRootPath)]


def _error(
    name: str,
    error_type: ErrorType,
    sites: list[ErrorSite],
    message: str,
    suggestion: str | None = None,
) -> ValidationError:
    msg = message
    if suggestion:
        msg = f"{message} Suggestion: {suggestion}"
    return ValidationError(name, error_type, sites, msg)


def _site(prim: "Usd.Prim") -> list[ErrorSite]:
    return [ErrorSite(prim.GetStage(), prim.GetPath())]


PrimValidatorFn = Callable[
    ["Usd.Prim", TimeRange], list[ValidationError]
]


def register_plugin_stage_validator(name: str, fn: StageValidatorFn) -> None:
    """Bind a plugin-declared stage validator implementation.

    Validator metadata (doc, keywords, schemaTypes) must already be
    declared under the ``usdRosValidators`` entry in
    ``core/ros/plugin/resource/plugInfo.json``.
    """
    _registry.RegisterPluginStageValidator(f"usdRosValidators:{name}", fn)


def register_plugin_prim_validator(name: str, fn: PrimValidatorFn) -> None:
    """Bind a plugin-declared prim validator implementation.

    See :func:`register_plugin_stage_validator` for the metadata contract.
    """
    _registry.RegisterPluginPrimValidator(f"usdRosValidators:{name}", fn)


def get_validators_for_keywords(
    keywords: list[str],
) -> list:
    validators = []
    for kw in keywords:
        metas = _registry.GetValidatorMetadataForKeyword(kw)
        for m in metas:
            v = _registry.GetOrLoadValidatorByName(m.name)
            if v not in validators:
                validators.append(v)
    return validators


def get_all_rep0158_validators() -> list:
    metas = _registry.GetValidatorMetadataForKeyword("rep0158")
    return [_registry.GetOrLoadValidatorByName(m.name) for m in metas]
