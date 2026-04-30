"""Adapter: convert UsdValidation errors to JSON/dict for CLI output."""

from __future__ import annotations

import json
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from pxr import UsdValidation

_ERROR_TYPE_LABEL = {
    "Error": "error",
    "Warn": "warning",
    "Info": "info",
}


def _severity_label(error: "UsdValidation.ValidationError") -> str:
    return _ERROR_TYPE_LABEL.get(str(error.GetType()).rsplit(".", 1)[-1], "info")


def _section_from_error(error: "UsdValidation.ValidationError") -> str:
    name = error.GetName()
    parts = name.split(".")
    if len(parts) >= 2 and parts[0].isdigit():
        return f"{parts[0]}.{parts[1]}"
    if "." in name:
        prefix = name.rsplit(".", 1)[0]
        return prefix
    return name


def _prim_path_from_error(error: "UsdValidation.ValidationError") -> str:
    sites = error.GetSites()
    if sites:
        site = sites[0]
        if site.IsProperty():
            prop = site.GetProperty()
            if prop and prop.IsValid():
                return str(prop.GetPath())
        if site.IsPrim():
            prim = site.GetPrim()
            if prim and prim.IsValid():
                return str(prim.GetPath())
    return "/"


def error_to_dict(error: "UsdValidation.ValidationError") -> dict:
    return {
        "check_id": error.GetName(),
        "severity": _severity_label(error),
        "section": _section_from_error(error),
        "prim_path": _prim_path_from_error(error),
        "message": error.GetMessage(),
        "validator": error.GetIdentifier(),
    }


def errors_to_report_dict(
    asset_path: str, errors: list["UsdValidation.ValidationError"]
) -> dict:
    n_errors = sum(1 for e in errors if _severity_label(e) == "error")
    n_warnings = sum(1 for e in errors if _severity_label(e) == "warning")
    n_infos = sum(1 for e in errors if _severity_label(e) == "info")
    return {
        "asset": asset_path,
        "summary": {
            "errors": n_errors,
            "warnings": n_warnings,
            "infos": n_infos,
            "total": len(errors),
        },
        "violations": [error_to_dict(e) for e in errors],
    }


def errors_to_json(
    asset_path: str,
    errors: list["UsdValidation.ValidationError"],
    indent: int = 2,
) -> str:
    return json.dumps(errors_to_report_dict(asset_path, errors), indent=indent)
