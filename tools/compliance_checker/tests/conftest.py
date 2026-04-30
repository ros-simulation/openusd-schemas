"""Shared fixtures and helpers for the REP-0158 compliance checker test suite."""

from __future__ import annotations

from typing import TYPE_CHECKING

import pytest
from pxr import Sdf, Usd

if TYPE_CHECKING:
    from compliance_checker.checks.base import BaseCheck
    from compliance_checker.report import Violation


# ------------------------------------------------------------------ #
# Stage factory                                                         #
# ------------------------------------------------------------------ #


def make_stage(usda: str) -> Usd.Stage:
    """Return an in-memory USD stage loaded from a USDA string.

    Uses an anonymous SdfLayer so no files are written to disk.
    The payload warning printed to stderr when a payload target does
    not exist is expected in tests that exercise payload detection.
    """
    layer = Sdf.Layer.CreateAnonymous(".usda")
    layer.ImportFromString(usda)
    return Usd.Stage.Open(layer)


@pytest.fixture
def tmp_usda(tmp_path):
    """Fixture: write USDA content to a temp file, return path string.

    Use this only when the check under test requires real file-system
    references (e.g. external reference path checks).
    """

    def _write(content: str, name: str = "test.usda") -> str:
        p = tmp_path / name
        p.write_text(content)
        return str(p)

    return _write


# ------------------------------------------------------------------ #
# Check runner helpers                                                  #
# ------------------------------------------------------------------ #


def run_check(
    stage: Usd.Stage,
    *check_classes: type[BaseCheck],
) -> list[Violation]:
    """Run one or more check classes against *stage*, return all violations."""
    from compliance_checker.checker import ComplianceChecker

    checks = [cls() for cls in check_classes]
    return ComplianceChecker(stage, checks).run().violations


def ids(violations: list[Violation]) -> set[str]:
    """Return the set of check_ids from a violation list."""
    return {v.check_id for v in violations}


def has(violations: list[Violation], check_id: str) -> bool:
    """Return True when at least one violation carries *check_id*."""
    return check_id in ids(violations)


def none_with(violations: list[Violation], check_id: str) -> bool:
    """Return True when no violation carries *check_id*."""
    return check_id not in ids(violations)
