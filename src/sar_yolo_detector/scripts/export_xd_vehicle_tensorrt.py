#!/usr/bin/env python3
"""Export the existing YOLO weights to an explicit TensorRT FP16 engine.

This is an offline deployment helper.  It never changes the source ``.pt``
file and refuses to run when CUDA/TensorRT are not available, so a partially
exported artifact cannot silently enter the flight launch.
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
import sys
from pathlib import Path

_PACKAGE_PYTHON = Path(__file__).resolve().parents[1] / "python"
if _PACKAGE_PYTHON.is_dir():
    sys.path.insert(0, str(_PACKAGE_PYTHON))

from sar_yolo_detector.pixeagle.backends.tensorrt_compat import (
    builder_optimization_level,
    import_tensorrt,
    sha256_file,
)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", required=True, type=Path,
                        help="existing Ultralytics .pt model")
    parser.add_argument("--output", type=Path, default=None,
                        help="optional destination .engine path")
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--workspace", type=float, default=0.5,
                        help="TensorRT workspace in GiB")
    parser.add_argument("--builder-optimization-level", type=int, choices=range(0, 6),
                        default=1, help="TensorRT tactic-search level (0 builds fastest)")
    args = parser.parse_args()
    model_path = args.model.expanduser().resolve()
    if not model_path.is_file() or model_path.suffix.lower() != ".pt":
        raise SystemExit("--model must point to an existing .pt file")
    try:
        trt = import_tensorrt()
        import torch
        from ultralytics import YOLO
    except Exception as error:
        raise SystemExit("PyTorch, Ultralytics, and TensorRT are required: %s" % error)
    if not torch.cuda.is_available():
        raise SystemExit("CUDA is unavailable; export on a GPU host")
    print("tensorrt_version=%s" % getattr(trt, "__version__", "unknown"))
    source_sha256 = sha256_file(model_path)
    print("source_sha256=%s" % source_sha256)
    model = YOLO(str(model_path))
    with builder_optimization_level(args.builder_optimization_level):
        exported = model.export(
            format="engine",
            half=True,
            imgsz=int(args.imgsz),
            dynamic=False,
            device=0,
            workspace=float(args.workspace),
        )
    engine_path = Path(str(exported)).expanduser().resolve()
    if not engine_path.is_file() or engine_path.suffix.lower() != ".engine":
        raise SystemExit("Ultralytics did not produce a valid .engine artifact")
    if args.output is not None:
        destination = args.output.expanduser().resolve()
        if destination.suffix.lower() != ".engine":
            raise SystemExit("--output must end with .engine")
        destination.parent.mkdir(parents=True, exist_ok=True)
        if engine_path != destination:
            shutil.copy2(engine_path, destination)
        engine_path = destination
    engine_sha256 = sha256_file(engine_path)
    manifest_path = engine_path.with_suffix(engine_path.suffix + ".json")
    manifest = {
        "source_model": str(model_path),
        "source_sha256": source_sha256,
        "engine": str(engine_path),
        "engine_sha256": engine_sha256,
        "precision": "fp16",
        "dynamic": False,
        "image_size": int(args.imgsz),
        "workspace_gib": float(args.workspace),
        "builder_optimization_level": int(args.builder_optimization_level),
        "tensorrt_version": str(getattr(trt, "__version__", "unknown")),
    }
    temporary_manifest = manifest_path.with_suffix(manifest_path.suffix + ".tmp")
    temporary_manifest.write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    os.replace(temporary_manifest, manifest_path)
    print("engine=%s" % engine_path)
    print("engine_sha256=%s" % engine_sha256)
    print("engine_manifest=%s" % manifest_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
