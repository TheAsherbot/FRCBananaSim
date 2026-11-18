"""
diagnostics.py: Diagnostic codes, severities, and issue reporting for FRCBananaExporter.
Matches diagnostic catalog in Block 4 Plan §7.
"""

from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Dict, List, Optional


class Severity(str, Enum):
    INFO = "INFO"
    WARN = "WARN"
    ERROR = "ERROR"


@dataclass
class Diagnostic:
    code: str
    severity: Severity
    message: str
    target: Optional[str] = None
    suggestion: Optional[str] = None
    details: Dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> Dict[str, Any]:
        d = {
            "code": self.code,
            "severity": self.severity.value,
            "message": self.message,
        }
        if self.target is not None:
            d["target"] = self.target
        if self.suggestion is not None:
            d["suggestion"] = self.suggestion
        if self.details:
            d["details"] = self.details
        return d


class DiagnosticCollector:
    """Collects diagnostics during CAD parsing, analysis, and validation."""

    def __init__(self):
        self._diagnostics: List[Diagnostic] = []

    def add(self, diag: Diagnostic) -> None:
        self._diagnostics.append(diag)

    def error(self, code: str, message: str, target: Optional[str] = None, suggestion: Optional[str] = None, **details) -> None:
        self.add(Diagnostic(code, Severity.ERROR, message, target, suggestion, details))

    def warn(self, code: str, message: str, target: Optional[str] = None, suggestion: Optional[str] = None, **details) -> None:
        self.add(Diagnostic(code, Severity.WARN, message, target, suggestion, details))

    def info(self, code: str, message: str, target: Optional[str] = None, suggestion: Optional[str] = None, **details) -> None:
        self.add(Diagnostic(code, Severity.INFO, message, target, suggestion, details))

    @property
    def diagnostics(self) -> List[Diagnostic]:
        return list(self._diagnostics)

    @property
    def has_errors(self) -> bool:
        return any(d.severity == Severity.ERROR for d in self._diagnostics)

    @property
    def has_warnings(self) -> bool:
        return any(d.severity == Severity.WARN for d in self._diagnostics)

    @property
    def error_count(self) -> int:
        return sum(1 for d in self._diagnostics if d.severity == Severity.ERROR)

    @property
    def warning_count(self) -> int:
        return sum(1 for d in self._diagnostics if d.severity == Severity.WARN)

    @property
    def info_count(self) -> int:
        return sum(1 for d in self._diagnostics if d.severity == Severity.INFO)

    def to_list(self) -> List[Dict[str, Any]]:
        return [d.to_dict() for d in self._diagnostics]
