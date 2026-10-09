"""Bounded prepare/commit transaction; no model build in the online path.

Preparation runs on exactly one background thread. Only the inference worker
may take a prepared candidate, and only between complete batches. Registry
hashes are rechecked before deserializing an engine (files can change on disk).
"""
from copy import deepcopy
from pathlib import Path
import threading
import time

from .model_registry import _sha256


class SwitchError(RuntimeError):
    def __init__(self, code, message):
        super().__init__(message)
        self.code = code


class ModelSwitch:
    def __init__(self, registry, loader, memory, wake, failed,
                 reserve_bytes=256 * 1024**2, candidate_bytes=768 * 1024**2):
        self.registry = registry
        self.loader, self.memory = loader, memory
        self.wake, self.failed = wake, failed
        self.reserve_bytes = max(0, int(reserve_bytes))
        self.candidate_bytes = max(128 * 1024**2, int(candidate_bytes))
        self.lock = threading.Lock()
        self.pending = None
        self.ready = None
        self.closed = False
        self.thread = None
        self.deadline = 0.0

    def submit(self, command):
        with self.lock:
            if self.closed:
                raise SwitchError("UNAVAILABLE", "detector is shutting down")
            if self.pending is not None:
                raise SwitchError("BUSY", "another model is being prepared")
            if command.model_id not in self.registry:
                raise SwitchError("UNKNOWN_MODEL", "model_id is not registered")
            self.pending = deepcopy(command)
            self.deadline = time.monotonic() + 28.0
            self.thread = threading.Thread(target=self._prepare,
                args=(self.pending,), name="detector_model_prepare", daemon=True)
            self.thread.start()

    def _prepare(self, command):
        candidate = None
        try:
            item = self.registry[command.model_id]
            if not item.get("engine_prepared") or not Path(item["engine_path"]).is_file():
                raise SwitchError("NOT_PREPARED", "prebuild the registered FP16 engine offline")
            if (_sha256(item["engine_path"]) != item["engine_sha256"] or
                    _sha256(item["pt_path"]) != item["pt_sha256"] or
                    item["engine_source_model_sha256"] != item["pt_sha256"]):
                raise SwitchError("HASH_MISMATCH", "registered model artifact changed")
            free, _ = self.memory()
            required = max(self.candidate_bytes,
                           Path(item["engine_path"]).stat().st_size * 8)
            if free < self.reserve_bytes + required:
                raise SwitchError("INSUFFICIENT_MEMORY",
                    "free GPU memory %d < reserve+candidate %d bytes" %
                    (free, self.reserve_bytes + required))
            candidate = self.loader(item)
            if time.monotonic() > self.deadline:
                raise SwitchError("TIMEOUT", "candidate preparation exceeded 28 seconds")
            free, _ = self.memory()
            if free < self.reserve_bytes:
                raise SwitchError("INSUFFICIENT_MEMORY", "candidate consumes the GPU safety reserve")
            with self.lock:
                if self.closed:
                    raise SwitchError("UNAVAILABLE", "detector is shutting down")
                self.ready = (command, item, candidate)
            self.wake()
            return
        except Exception as error:
            if candidate is not None:
                candidate[0].unload_model()
            with self.lock:
                self.pending = None
            memory_failure = (type(error).__name__ == "OutOfMemoryError" or
                              "out of memory" in str(error).lower())
            self.failed(command, getattr(error, "code",
                "INSUFFICIENT_MEMORY" if memory_failure else "LOAD_FAILED"), str(error))

    def take_ready(self):
        with self.lock:
            result, self.ready = self.ready, None
            return result

    def complete(self):
        with self.lock:
            self.pending = None

    def close(self):
        with self.lock:
            self.closed = True
            result, self.ready = self.ready, None
        if result:
            result[2][0].unload_model()
        if self.thread is not None and self.thread is not threading.current_thread():
            self.thread.join(timeout=3.0)
