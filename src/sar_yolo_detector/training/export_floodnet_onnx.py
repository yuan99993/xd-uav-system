#!/usr/bin/env python3
"""Export the FloodNet SegFormer checkpoint for the deployed SAR frontend.

The training checkpoint is a small training-state dictionary (rather than a
Hugging Face ``save_pretrained`` directory).  This script restores the model,
exports the native SegFormer logits, and validates the graph with ONNX Runtime.

The exported graph has fixed bindings for the deployed 1024-pixel crop:

    pixel_values: [1, 3, 1024, 1024] float32 (RGB, normalized)
    logits:       [1, 10, 256, 256] float32 (quarter-resolution class logits)

Keeping the logits at the network's native quarter resolution avoids an
unnecessary 4x tensor in TensorRT.  The inference adapter should argmax the
ten channels and resize the resulting uint8 mask to the camera resolution
before publishing ``FloodMask``/``sensor_msgs/Image`` to the C++ region
extractor.  TensorRT's ``--fp16`` flag is used when building the engine; no
accuracy-changing preprocessing is hidden in this export.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Dict

import numpy as np
import torch
import torch.nn as nn
from transformers import SegformerForSemanticSegmentation


CLASS_NAMES = [
    "background", "building_flooded", "building_non_flooded",
    "road_flooded", "road_non_flooded", "water", "tree", "vehicle",
    "pool", "grass",
]


class SegformerLogits(nn.Module):
    """Make the Hugging Face output a plain tensor for ONNX/TensorRT."""

    def __init__(self, model: SegformerForSemanticSegmentation) -> None:
        super().__init__()
        self.model = model

    def forward(self, pixel_values: torch.Tensor) -> torch.Tensor:
        return self.model(pixel_values=pixel_values).logits


def restore(checkpoint: Path) -> SegformerLogits:
    model = SegformerForSemanticSegmentation.from_pretrained(
        "nvidia/mit-b0",
        num_labels=len(CLASS_NAMES),
        id2label={i: name for i, name in enumerate(CLASS_NAMES)},
        label2id={name: i for i, name in enumerate(CLASS_NAMES)},
        ignore_mismatched_sizes=True,
    )
    state = torch.load(checkpoint, map_location="cpu")
    if not isinstance(state, dict) or "model" not in state:
        raise ValueError(f"expected training checkpoint with a 'model' key: {checkpoint}")
    model.load_state_dict(state["model"], strict=True)
    model.eval()
    return SegformerLogits(model)


def update_metadata(checkpoint: Path, onnx_path: Path, input_size: int, opset: int) -> None:
    metadata = checkpoint.parent / "training_metadata.json"
    if not metadata.is_file():
        return
    record: Dict[str, object] = json.loads(metadata.read_text(encoding="utf-8"))
    record["onnx_exported"] = True
    record["onnx"] = onnx_path.name
    record["onnx_opset"] = opset
    record["onnx_input_shape"] = [1, 3, input_size, input_size]
    record["onnx_output_shape"] = [1, len(CLASS_NAMES), input_size // 4, input_size // 4]
    metadata.write_text(json.dumps(record, indent=2) + "\n", encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--input-size", type=int, default=1024)
    parser.add_argument("--opset", type=int, default=17)
    parser.add_argument("--skip-verify", action="store_true")
    args = parser.parse_args()
    if args.input_size % 32:
        raise SystemExit("--input-size must be divisible by 32")

    torch.set_grad_enabled(False)
    wrapper = restore(args.checkpoint)
    dummy = torch.zeros(1, 3, args.input_size, args.input_size, dtype=torch.float32)
    reference = wrapper(dummy).cpu().numpy()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    torch.onnx.export(
        wrapper,
        (dummy,),
        str(args.output),
        input_names=["pixel_values"],
        output_names=["logits"],
        opset_version=args.opset,
        do_constant_folding=True,
        dynamic_axes=None,
    )

    import onnx

    graph = onnx.load(str(args.output))
    onnx.checker.check_model(graph)
    if not args.skip_verify:
        import onnxruntime as ort

        session = ort.InferenceSession(str(args.output), providers=["CPUExecutionProvider"])
        actual = session.run(["logits"], {"pixel_values": dummy.numpy()})[0]
        if actual.shape != reference.shape:
            raise RuntimeError(f"ONNX shape mismatch: torch={reference.shape} onnx={actual.shape}")
        max_error = float(np.max(np.abs(actual - reference)))
        if not np.isfinite(max_error) or max_error > 2e-3:
            raise RuntimeError(f"ONNX numerical check failed: max_abs_error={max_error:.6g}")
        print(f"ONNX Runtime check passed: shape={actual.shape}, max_abs_error={max_error:.6g}")
    update_metadata(args.checkpoint, args.output, args.input_size, args.opset)
    print(f"Exported {args.output} ({args.output.stat().st_size / 1024 / 1024:.1f} MiB)")


if __name__ == "__main__":
    main()
