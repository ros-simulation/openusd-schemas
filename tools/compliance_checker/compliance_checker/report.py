"""Report data structures for the REP-XXXX compliance checker."""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional


class Severity(str, Enum):
    ERROR = "error"
    WARNING = "warning"
    INFO = "info"

    def __lt__(self, other: "Severity") -> bool:
        _order = {Severity.ERROR: 0, Severity.WARNING: 1, Severity.INFO: 2}
        return _order[self] < _order[other]


@dataclass
class Violation:
    check_id: str
    severity: Severity
    prim_path: str
    message: str
    section: str
    suggestion: Optional[str] = None

    def to_dict(self) -> dict:
        return {
            "check_id": self.check_id,
            "severity": self.severity.value,
            "section": self.section,
            "prim_path": self.prim_path,
            "message": self.message,
            "suggestion": self.suggestion,
        }


@dataclass
class Report:
    asset_path: str
    violations: list[Violation] = field(default_factory=list)

    # ------------------------------------------------------------------ #
    # Convenience filters                                                   #
    # ------------------------------------------------------------------ #

    @property
    def errors(self) -> list[Violation]:
        return [v for v in self.violations if v.severity == Severity.ERROR]

    @property
    def warnings(self) -> list[Violation]:
        return [v for v in self.violations if v.severity == Severity.WARNING]

    @property
    def infos(self) -> list[Violation]:
        return [v for v in self.violations if v.severity == Severity.INFO]

    def by_section(self) -> dict[str, list[Violation]]:
        """Group violations by section number."""
        result: dict[str, list[Violation]] = {}
        for v in self.violations:
            result.setdefault(v.section, []).append(v)
        return result

    # ------------------------------------------------------------------ #
    # Serialisation                                                         #
    # ------------------------------------------------------------------ #

    def to_dict(self) -> dict:
        return {
            "asset": self.asset_path,
            "summary": {
                "errors": len(self.errors),
                "warnings": len(self.warnings),
                "infos": len(self.infos),
                "total": len(self.violations),
            },
            "violations": [v.to_dict() for v in self.violations],
        }

    def to_json(self, indent: int = 2) -> str:
        return json.dumps(self.to_dict(), indent=indent)

    # ------------------------------------------------------------------ #
    # Pass / fail helpers                                                   #
    # ------------------------------------------------------------------ #

    def has_errors(self) -> bool:
        return bool(self.errors)

    def has_warnings(self) -> bool:
        return bool(self.warnings)

    def passed(self, fail_on: Severity = Severity.ERROR) -> bool:
        """Return True when no violation reaches or exceeds *fail_on* severity."""
        return not any(v.severity <= fail_on for v in self.violations)
