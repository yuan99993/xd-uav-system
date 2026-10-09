"""Read-only ROS status derived from a successfully loaded backend runtime.

No model files are opened and no configured/expected digest is substituted for
the artifact digest recorded by the loader.
"""

import re
import json
from pathlib import Path

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue


def build_model_status(runtime, *, node_name, stamp, configured_model_path="",
                       startup_ready=False, startup_error="",
                       active_model_id="unknown", source_model_sha256="",
                       model_epoch=0, class_names=()):
    """Build a fresh snapshot; do not mutate runtime or call the backend."""
    path = str(runtime.get("model_path") or "")
    digest = str(runtime.get("artifact_sha256") or "").lower()
    digest_valid = re.fullmatch(r"[0-9a-f]{64}", digest) is not None
    if not digest_valid:
        digest = ""

    status = DiagnosticStatus()
    status.name = node_name + ": model_runtime"
    status.hardware_id = str(runtime.get("effective_device") or "unknown")
    if startup_error:
        state = "ERROR"
        status.level = DiagnosticStatus.ERROR
        status.message = "model warmup failed"
    elif not path:
        state = "UNAVAILABLE"
        status.level = DiagnosticStatus.ERROR
        status.message = "loaded model path unavailable"
    elif not startup_ready:
        state = "WARMING_UP"
        status.level = DiagnosticStatus.WARN
        status.message = "model loaded; warmup not complete"
    else:
        state = "READY"
        status.level = DiagnosticStatus.OK if digest_valid else DiagnosticStatus.WARN
        status.message = ("loaded model ready" if digest_valid else
                          "loaded model ready; artifact digest unavailable")

    values = {
        "schema_version": "1",
        "state": state,
        "configured_model_path": str(configured_model_path),
        "loaded_model_path": path,
        "loaded_model_name": Path(path).name if path else "",
        "loaded_artifact_sha256": digest,
        "artifact_id": "sha256:" + digest if digest else "",
        "artifact_digest_available": str(digest_valid).lower(),
        "backend": str(runtime.get("backend") or "unknown"),
        "model_format": str(runtime.get("model_format") or "unknown"),
        "requested_device": str(runtime.get("requested_device") or "unknown"),
        "effective_device": status.hardware_id,
        "fallback_occurred": str(bool(runtime.get("fallback_occurred", False))).lower(),
        "fallback_reason": str(runtime.get("fallback_reason") or ""),
        "startup_ready": str(bool(startup_ready and path and not startup_error)).lower(),
        "startup_error": str(startup_error),
        "active_model_id": str(active_model_id or "unknown"),
        "source_model_sha256": str(source_model_sha256 or "").lower(),
        "model_epoch": str(int(model_epoch)),
        "class_names": json.dumps(list(class_names)),
    }
    status.values = [KeyValue(key=key, value=value) for key, value in values.items()]
    message = DiagnosticArray()
    message.header.stamp = stamp
    message.status = [status]
    return message
