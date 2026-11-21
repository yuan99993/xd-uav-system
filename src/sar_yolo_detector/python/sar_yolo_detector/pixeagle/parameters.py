"""Minimal configuration bridge expected by the transplanted PixEagle class."""

from __future__ import annotations

from copy import deepcopy
from typing import Any, Dict


class Parameters:
    """Process-local SmartTracker settings populated by the ROS adapter."""

    SmartTracker: Dict[str, Any] = {}

    @staticmethod
    def for_lock_policy(config: Dict[str, Any], class_agnostic: bool) -> Dict[str, Any]:
        result=deepcopy(config)
        # None means all model outputs. [] means no outputs in both adapters.
        if class_agnostic:result['SMART_TRACKER_ALLOWED_CLASS_IDS']=None
        return result

    @classmethod
    def configure_smart_tracker(cls, config: Dict[str, Any]) -> None:
        if not isinstance(config, dict):
            raise TypeError("SmartTracker configuration must be a dictionary")
        cls.SmartTracker = deepcopy(config)


__all__ = ["Parameters"]
