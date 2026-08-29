"""Minimal configuration bridge expected by the transplanted PixEagle class."""

from __future__ import annotations

from copy import deepcopy
from typing import Any, Dict


class Parameters:
    """Process-local SmartTracker settings populated by the ROS adapter."""

    SmartTracker: Dict[str, Any] = {}

    @classmethod
    def configure_smart_tracker(cls, config: Dict[str, Any]) -> None:
        if not isinstance(config, dict):
            raise TypeError("SmartTracker configuration must be a dictionary")
        cls.SmartTracker = deepcopy(config)


__all__ = ["Parameters"]
