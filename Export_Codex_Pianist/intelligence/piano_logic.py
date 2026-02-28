from __future__ import annotations

from dataclasses import dataclass
from typing import List


@dataclass
class PianoLogicEngine:
    """Placeholder for Codex-driven piano planning logic."""

    startup_sequence: List[str] = None

    def __post_init__(self) -> None:
        if self.startup_sequence is None:
            self.startup_sequence = ["C4", "E4", "G4"]

    def get_startup_notes(self) -> List[str]:
        return list(self.startup_sequence)

