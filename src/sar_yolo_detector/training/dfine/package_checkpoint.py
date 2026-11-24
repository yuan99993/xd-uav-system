#!/usr/bin/env python3
"""Package a D-FINE training checkpoint as the runtime safetensors bundle."""
import argparse
import json
import sys
from pathlib import Path

import torch


def main():
    package_root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo", type=Path, default=package_root / "third_party" / "D-FINE")
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--destination", type=Path, required=True)
    parser.add_argument("--variant", choices=("dfine", "deim_dfine"), default="dfine")
    parser.add_argument("--preprocess-mode", choices=("stretch", "letterbox"), default="letterbox")
    args = parser.parse_args()
    repo = args.repo.resolve()
    if not (repo / "train.py").is_file():
        parser.error(f"D-FINE train.py not found under {repo}")
    sys.path.insert(0, str(repo))
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    import letterbox_resize  # noqa: F401 - registers profile transform
    from src.core import YAMLConfig
    sys.path.insert(0, str(package_root / "python"))
    from sar_yolo_detector.pixeagle.backends.dfine_runtime import export_bundle, digest
    cfg = YAMLConfig(str(args.config.resolve()))
    if "HGNetv2" in cfg.yaml_cfg:
        cfg.yaml_cfg["HGNetv2"]["pretrained"] = False
    checkpoint = torch.load(args.checkpoint.resolve(), map_location="cpu", weights_only=False)
    if "ema" in checkpoint and isinstance(checkpoint["ema"], dict) and "module" in checkpoint["ema"]:
        state = checkpoint["ema"]["module"]
    elif "model" in checkpoint:
        state = checkpoint["model"]
    else:
        raise ValueError("Checkpoint must contain ema.module or model state")
    incompatible = cfg.model.load_state_dict(state, strict=False)
    if incompatible.missing_keys or incompatible.unexpected_keys:
        raise ValueError("Checkpoint/model mismatch: " +
            f"missing={incompatible.missing_keys[:8]} unexpected={incompatible.unexpected_keys[:8]}")
    manifest = export_bundle(cfg.model, cfg, args.variant, args.destination.resolve(),
                             ["car", "tank", "drone"], args.preprocess_mode)
    manifest_data = json.loads(Path(manifest).read_text(encoding="utf-8"))
    manifest_data["preprocess_implementation_sha256"] = {
        "training_letterbox": digest(Path(__file__).with_name("letterbox_resize.py")),
        "runtime_preprocessor": digest(package_root / "python/sar_yolo_detector/pixeagle/backends/dfine_runtime.py"),
        "fused_cuda_preprocessor": digest(package_root / "python/sar_yolo_detector/pixeagle/backends/fused_preprocess.py"),
    }
    Path(manifest).write_text(json.dumps(manifest_data, indent=2) + "\n", encoding="utf-8")
    print(f"Packaged verified runtime model bundle: {manifest}")


if __name__ == "__main__":
    main()
