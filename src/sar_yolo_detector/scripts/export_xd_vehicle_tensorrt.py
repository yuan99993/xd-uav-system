#!/usr/bin/env python3
"""Export the existing YOLO weights to an explicit TensorRT FP16 engine.

This is an offline deployment helper.  It never changes the source ``.pt``
file and refuses to run when CUDA/TensorRT are not available, so a partially
exported artifact cannot silently enter the flight launch.
"""

from __future__ import annotations

import argparse
import hashlib
import shutil
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", required=True, type=Path,
                        help="existing Ultralytics .pt model")
    parser.add_argument("--output", type=Path, default=None,
                        help="optional destination .engine path")
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--workspace", type=float, default=2.0,
                        help="TensorRT workspace in GiB")
    args = parser.parse_args()
    model_path = args.model.expanduser().resolve()
    if not model_path.is_file() or model_path.suffix.lower() != ".pt":
        raise SystemExit("--model must point to an existing .pt file")
    try:
        import torch
        from ultralytics import YOLO
    except Exception as error:
        raise SystemExit("Ultralytics and torch are required: %s" % error)
    if not torch.cuda.is_available():
        raise SystemExit("CUDA is unavailable; export on a GPU host")
    try:
        import tensorrt  # noqa: F401
    except Exception as error:
        raise SystemExit(
            "Python TensorRT is unavailable; install a TensorRT runtime matching CUDA: %s"
            % error
        )
    print("source_sha256=%s" % hashlib.sha256(model_path.read_bytes()).hexdigest())
    model = YOLO(str(model_path))
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
    print("engine=%s" % engine_path)
    print("engine_sha256=%s" % hashlib.sha256(engine_path.read_bytes()).hexdigest())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
