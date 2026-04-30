"""Registration helpers for UsdValidation-based compliance checks."""

from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING, Callable

from pxr import Plug, Sdf, UsdValidation

if TYPE_CHECKING:
    from pxr import Usd

# Register the codeless ROS schema plugin so that prim.HasAPI() and
# Usd.SchemaRegistry work for RosContextAPI, RosTopicAPI, etc.
_SCHEMA_PLUGIN = Path(__file__).resolve().parents[4] / "core" / "ros" / "plugin" / "resource"
if _SCHEMA_PLUGIN.is_dir():
    Plug.Registry().RegisterPlugins(str(_SCHEMA_PLUGIN))

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


def register_prim_validator(
    name: str,
    fn: PrimValidatorFn,
    *,
    doc: str = "",
    keywords: list[str] | None = None,
    section: str = "",
    schema_types: list[str] | None = None,
) -> None:
    kw = list(keywords or [])
    if section:
        kw.append(f"rep0158:{section}")
    kw.append("rep0158")
    metadata = UsdValidation.ValidatorMetadata(
        name=f"rep0158:{name}",
        doc=doc,
        keywords=kw,
        schemaTypes=schema_types or [],
    )
    _registry.RegisterPrimValidator(metadata, fn)


def register_stage_validator(
    name: str,
    fn: StageValidatorFn,
    *,
    doc: str = "",
    keywords: list[str] | None = None,
    section: str = "",
) -> None:
    kw = list(keywords or [])
    if section:
        kw.append(f"rep0158:{section}")
    kw.append("rep0158")
    metadata = UsdValidation.ValidatorMetadata(
        name=f"rep0158:{name}",
        doc=doc,
        keywords=kw,
    )
    _registry.RegisterStageValidator(metadata, fn)


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
