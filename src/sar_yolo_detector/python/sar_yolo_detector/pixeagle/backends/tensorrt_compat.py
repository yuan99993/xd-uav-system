"""Load NVIDIA TensorRT bindings across CUDA-suffixed wheel layouts."""

from __future__ import annotations

import importlib
import hashlib
import json
import os
import shutil
import sys
import tempfile
from contextlib import contextmanager
from types import ModuleType
from pathlib import Path
from typing import Iterator, Tuple


def import_tensorrt() -> ModuleType:
    """Return the TensorRT API and expose it under Ultralytics' expected name.

    NVIDIA's CUDA-specific bindings wheel exposes ``tensorrt_bindings`` while
    Ultralytics imports ``tensorrt``. Reuse the installed binding module rather
    than installing a second, multi-gigabyte TensorRT runtime into the venv.
    """
    try:
        return importlib.import_module("tensorrt")
    except ModuleNotFoundError as error:
        if error.name != "tensorrt":
            raise

    module = importlib.import_module("tensorrt_bindings")
    sys.modules["tensorrt"] = module
    return module


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


@contextmanager
def _cache_lock(path: Path) -> Iterator[None]:
    # ROS deployment targets are Linux. A process lock prevents two detector
    # processes from compiling the same model into the shared cache at once.
    import fcntl

    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a+b") as lock_file:
        fcntl.flock(lock_file.fileno(), fcntl.LOCK_EX)
        try:
            yield
        finally:
            fcntl.flock(lock_file.fileno(), fcntl.LOCK_UN)


@contextmanager
def builder_optimization_level(level: int) -> Iterator[None]:
    """Temporarily set TensorRT's builder level without patching site packages."""
    if not 0 <= int(level) <= 5:
        raise ValueError("TensorRT builder optimization level must be within 0..5")
    trt = import_tensorrt()
    original_builder = trt.Builder

    class _ConfiguredBuilder:
        def __init__(self, *args, **kwargs):
            self._builder = original_builder(*args, **kwargs)

        def __getattr__(self, name):
            return getattr(self._builder, name)

        def create_builder_config(self):
            config = self._builder.create_builder_config()
            if hasattr(config, "builder_optimization_level"):
                config.builder_optimization_level = int(level)
            return config

    trt.Builder = _ConfiguredBuilder
    try:
        yield
    finally:
        trt.Builder = original_builder


def prepare_fp16_engine(
    model_path: Path,
    cache_dir: Path,
    *,
    image_size: int = 640,
    workspace_gib: float = 0.5,
    optimization_level: int = 1,
    expected_model_sha256: str = "",
) -> Tuple[Path, str]:
    """Build/reuse a model- and host-specific TensorRT FP16 cache artifact."""
    source = model_path.expanduser().resolve()
    if not source.is_file() or source.suffix.lower() != ".pt":
        raise ValueError("automatic TensorRT conversion requires an existing .pt model")
    if image_size < 320 or image_size > 1536 or image_size % 32:
        raise ValueError("TensorRT image_size must be a multiple of 32 within 320..1536")
    if not 0.25 <= workspace_gib <= 4.0:
        raise ValueError("TensorRT workspace_gib must be within 0.25..4.0")
    if not 0 <= int(optimization_level) <= 5:
        raise ValueError("TensorRT optimization_level must be within 0..5")

    source_sha256 = sha256_file(source)
    expected = expected_model_sha256.strip().lower()
    if expected and source_sha256 != expected:
        raise ValueError(
            f"source model SHA-256 mismatch: expected {expected}, got {source_sha256}"
        )

    trt = import_tensorrt()
    import torch
    import ultralytics

    if not torch.cuda.is_available():
        raise RuntimeError("automatic TensorRT conversion requires an available CUDA GPU")
    device = torch.cuda.get_device_properties(0)
    capability = torch.cuda.get_device_capability(0)
    fingerprint_data = {
        "source_sha256": source_sha256,
        "tensorrt": str(getattr(trt, "__version__", "unknown")),
        "ultralytics": str(getattr(ultralytics, "__version__", "unknown")),
        "cuda": str(torch.version.cuda or "unknown"),
        "gpu": str(device.name),
        "compute_capability": "%d.%d" % capability,
        "image_size": int(image_size),
        "workspace_gib": float(workspace_gib),
        "builder_optimization_level": int(optimization_level),
        "precision": "fp16",
        "dynamic": False,
    }
    fingerprint = hashlib.sha256(
        json.dumps(fingerprint_data, sort_keys=True, separators=(",", ":")).encode("utf-8")
    ).hexdigest()
    root = cache_dir.expanduser().resolve()
    output_dir = root / fingerprint
    engine_path = output_dir / (source.stem + ".engine")
    manifest_path = output_dir / (source.stem + ".engine.json")
    lock_path = root / ".locks" / (fingerprint + ".lock")

    with _cache_lock(lock_path):
        if engine_path.is_file() and manifest_path.is_file():
            try:
                manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
                observed_engine_sha256 = sha256_file(engine_path)
                if (manifest.get("fingerprint") == fingerprint and
                        manifest.get("source_sha256") == source_sha256 and
                        manifest.get("engine_sha256") == observed_engine_sha256):
                    return engine_path, observed_engine_sha256
            except (OSError, ValueError, TypeError):
                pass

        output_dir.mkdir(parents=True, exist_ok=True)
        with tempfile.TemporaryDirectory(prefix="tensorrt-build-", dir=str(root)) as work:
            staged_model = Path(work) / source.name
            shutil.copy2(source, staged_model)
            from ultralytics import YOLO

            with builder_optimization_level(optimization_level):
                exported = YOLO(str(staged_model)).export(
                    format="engine",
                    half=True,
                    imgsz=int(image_size),
                    dynamic=False,
                    device=0,
                    workspace=float(workspace_gib),
                )
            staged_engine = Path(str(exported)).expanduser().resolve()
            if not staged_engine.is_file() or staged_engine.suffix.lower() != ".engine":
                raise RuntimeError("Ultralytics failed to create a TensorRT .engine artifact")
            # TemporaryDirectory and cache are on the same filesystem, making
            # replacement atomic while never modifying the original model.
            os.replace(staged_engine, engine_path)

        engine_sha256 = sha256_file(engine_path)
        manifest = dict(fingerprint_data)
        manifest.update({
            "fingerprint": fingerprint,
            "source_model": str(source),
            "source_sha256": source_sha256,
            "engine": str(engine_path),
            "engine_sha256": engine_sha256,
        })
        temporary_manifest = manifest_path.with_suffix(manifest_path.suffix + ".tmp")
        temporary_manifest.write_text(
            json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        os.replace(temporary_manifest, manifest_path)
        return engine_path, engine_sha256
