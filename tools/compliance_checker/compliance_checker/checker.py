"""Core orchestration: open a USD stage and run checks against it."""

from __future__ import annotations

import logging
from typing import TYPE_CHECKING

from pxr import Usd

from .registry import build_checks
from .report import Report, Severity, Violation

if TYPE_CHECKING:
    from .checks.base import BaseCheck

log = logging.getLogger(__name__)


class ComplianceChecker:
    """Opens a USD stage and runs a suite of :class:`~checks.base.BaseCheck` instances.

    Usage::

        checker = ComplianceChecker.from_path("robot.usd", include_export=True)
        report = checker.run()
        print(report.to_json())
    """

    def __init__(self, stage: Usd.Stage, checks: list[BaseCheck]) -> None:
        self._stage = stage
        self._checks = checks

    # ------------------------------------------------------------------ #
    # Factory                                                               #
    # ------------------------------------------------------------------ #

    @classmethod
    def from_path(
        cls,
        asset_path: str,
        include_export: bool = False,
        include_extended: bool = False,
        sections: list[str] | None = None,
        include_extensions: bool = True,
    ) -> "ComplianceChecker":
        """Open *asset_path* with payloads loaded and prepare checks.

        Args:
            asset_path: Path to the USD file (usda / usdc / usd).
            include_export: Include §3 export/conversion checks (slower).
            include_extended: Include §4 extended schema checks
                (ExtendedPhysics*, RosControl* extensions).
            sections: Allowlist of section prefixes. ``None`` = all sections.
            include_extensions: Load extension check plug-ins.

        Raises:
            FileNotFoundError: If the USD stage cannot be opened.
        """
        try:
            stage = Usd.Stage.Open(asset_path, Usd.Stage.LoadAll)
        except Exception as exc:
            raise FileNotFoundError(
                f"Could not open USD stage: {asset_path!r}"
            ) from exc
        if not stage:
            raise FileNotFoundError(f"Could not open USD stage: {asset_path!r}")

        checks = build_checks(
            include_export=include_export,
            include_extended=include_extended,
            sections=sections,
            include_extensions=include_extensions,
        )
        return cls(stage, checks)

    # ------------------------------------------------------------------ #
    # Run                                                                   #
    # ------------------------------------------------------------------ #

    def run(self) -> Report:
        """Execute all checks and return a :class:`~report.Report`."""
        asset_path = self._stage.GetRootLayer().identifier
        report = Report(asset_path=asset_path)

        for check in self._checks:
            log.debug(
                "Running check %r (section %s)", type(check).__name__, check.section
            )
            try:
                for violation in check.run(self._stage):
                    report.violations.append(violation)
            except Exception as exc:  # noqa: BLE001
                log.error(
                    "Check %r raised an unexpected error: %s",
                    type(check).__name__,
                    exc,
                    exc_info=True,
                )
                # Add an internal error violation so it surfaces in the report
                report.violations.append(
                    Violation(
                        check_id=f"{check.section}.internal_error",
                        severity=Severity.ERROR,
                        prim_path="/",
                        section=check.section,
                        message=(
                            f"Internal error in check '{type(check).__name__}': {exc}. "
                            "Please report this as a bug."
                        ),
                    )
                )

        # Sort: errors first, then warnings, then info; stable within each group
        report.violations.sort(key=lambda v: (v.severity, v.section, v.prim_path))
        return report
