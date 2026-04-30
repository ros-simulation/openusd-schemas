"""Abstract base class for all compliance checks."""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import TYPE_CHECKING, Iterator

if TYPE_CHECKING:
    from pxr import Usd

    from ..report import Violation


class BaseCheck(ABC):
    """Each concrete check class covers one numbered section of the REP.

    Subclasses must:
    - Set the *section* class attribute (e.g. ``"1.1"``).
    - Implement :meth:`run`, yielding :class:`~report.Violation` instances.
    """

    section: str = ""

    @abstractmethod
    def run(self, stage: "Usd.Stage") -> Iterator["Violation"]:
        """Yield every non-conformance found in *stage*.

        The generator is allowed to be empty (no violations found).
        """
        ...

    def __repr__(self) -> str:
        return f"<{type(self).__name__} section={self.section!r}>"
