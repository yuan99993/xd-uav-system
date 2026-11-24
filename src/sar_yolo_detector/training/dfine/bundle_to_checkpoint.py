#!/usr/bin/env python3
"""Convert an integrity-verified D-FINE safetensors bundle for fine-tuning."""
import argparse
import sys
from pathlib import Path

import torch


def main():
    package_root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--device", default="cpu")
    args = parser.parse_args()
    sys.path.insert(0, str(package_root / "python"))
    from sar_yolo_detector.pixeagle.backends.dfine_runtime import load_bundle
    model, _, _ = load_bundle(args.manifest.resolve(), args.device)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    torch.save({"model": {key: value.detach().cpu() for key, value in model.state_dict().items()}},
               args.output)
    print(f"Wrote D-FINE fine-tuning checkpoint: {args.output}")


if __name__ == "__main__":
    main()
