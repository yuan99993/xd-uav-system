#!/usr/bin/env python3
"""Export fixed-shape raw D-FINE logits/boxes for the runtime TRT adapter."""
import argparse
import sys
from pathlib import Path

import torch
from torch import nn


class RawDfine(nn.Module):
    def __init__(self, model):
        super().__init__()
        self.model = model

    def forward(self, images):
        output = self.model(images)
        if not isinstance(output, dict) or "pred_logits" not in output or "pred_boxes" not in output:
            raise RuntimeError("D-FINE model must return pred_logits and pred_boxes")
        return output["pred_logits"], output["pred_boxes"]


def main():
    package_root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--size", type=int, choices=(320, 512), required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--opset", type=int, default=17)
    args = parser.parse_args()
    sys.path.insert(0, str(package_root / "python"))
    from sar_yolo_detector.pixeagle.backends.dfine_runtime import load_bundle
    model, _, _ = load_bundle(args.manifest.resolve(), "cpu")
    model.encoder.eval_spatial_size = None
    model.decoder.eval_spatial_size = None
    model.eval().deploy()
    wrapper = RawDfine(model).eval()
    pixels = torch.zeros((1, 3, args.size, args.size), dtype=torch.float32)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    torch.onnx.export(wrapper, pixels, str(args.output), input_names=["images"],
        output_names=["logits", "boxes"], opset_version=args.opset,
        do_constant_folding=True, dynamic_axes=None)
    print(f"Exported {args.size}x{args.size} D-FINE raw outputs: {args.output}")


if __name__ == "__main__":
    main()
