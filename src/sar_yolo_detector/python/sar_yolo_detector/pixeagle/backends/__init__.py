"""Detection backend registry for the transplanted SmartTracker."""

import importlib
from typing import Optional

from .detection_backend import DetectionBackend, DevicePreference


AVAILABLE_BACKENDS = {
    "ultralytics": (
        "sar_yolo_detector.pixeagle.backends.ultralytics_backend",
        "UltralyticsBackend",
    ),
}


def create_backend(
    backend_name: str = "ultralytics", config: Optional[dict] = None
) -> DetectionBackend:
    normalized = str(backend_name or "").strip().lower()
    if normalized not in AVAILABLE_BACKENDS:
        choices = ", ".join(sorted(AVAILABLE_BACKENDS))
        raise ValueError(f"Unknown detection backend {backend_name!r}; available: {choices}")
    module_name, class_name = AVAILABLE_BACKENDS[normalized]
    module = importlib.import_module(module_name)
    return getattr(module, class_name)(config or {})


__all__ = [
    "AVAILABLE_BACKENDS",
    "DetectionBackend",
    "DevicePreference",
    "create_backend",
]
