#!/usr/bin/env python3
"""Train, validate, and export one SAR detector profile with Ultralytics.

This script belongs to the offline training toolchain. The deployed ROS node
stays C++ and consumes only an ONNX file or a TensorRT engine.
"""

import argparse
import hashlib
import json
import os
import shutil
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import List


PROFILES = {
    "eo_person": {"names": ["person"], "imgsz": 640, "batch": 4,
                   "package_model_dir": "eo_person", "artifact_prefix": "eo_person",
                   "default_model": "yolo11n.pt"},
    # EO rescue-person fine tuning mixes roof/building victims with maritime
    # swimmers.  It starts from the NITC checkpoint at a low learning rate and
    # stops early when the balanced validation split no longer improves.
    # The 768-pixel input preserves more high-altitude small-target detail;
    # batch 8 remains within the local 4 GB GPU budget for YOLO11n.
    "eo_person_rescue_mix": {
        "names": ["person"], "imgsz": 768, "batch": 8,
        "package_model_dir": "eo_person", "artifact_prefix": "eo_person_rescue_mix",
        "default_model": "yolo11n.pt",
        "training_kwargs": {
            "optimizer": "AdamW", "lr0": 0.0005, "lrf": 0.1,
            "cos_lr": True, "patience": 6, "close_mosaic": 5,
        },
    },
    # Second EO rescue-person pass uses every video-disjoint SeaDronesSee
    # swimmer frame.  Sea test targets are predominantly sub-0.1%-area boxes,
    # so it trains at 1024 rather than relying on low-resolution augmentation.
    # Lower mosaic/scale augmentation avoids repeatedly shrinking those people.
    "eo_person_rescue_mix_small_targets": {
        "names": ["person"], "imgsz": 1024, "batch": 6,
        "package_model_dir": "eo_person", "artifact_prefix": "eo_person_rescue_mix_small_targets",
        "default_model": "yolo11n.pt",
        "training_kwargs": {
            "optimizer": "AdamW", "lr0": 0.00025, "lrf": 0.1,
            "cos_lr": True, "patience": 6, "mosaic": 0.5,
            "close_mosaic": 6, "scale": 0.2,
        },
    },
    "eo_visdrone": {"names": ["pedestrian", "people", "bicycle", "car", "van", "truck", "tricycle", "awning_tricycle", "bus", "motor"], "imgsz": 640, "batch": 2, "package_model_dir": "visdrone", "artifact_prefix": "visdrone", "default_model": "yolo11n.pt"},
    # SeaDronesSee v2 uses one ignored category (id 0) and five trainable
    # categories.  The converter remaps those five categories to contiguous
    # YOLO ids 0..4; keeping this order identical to the runtime YAML is
    # important for the TensorRT output binding and task-point filtering.
    "maritime_seadronessee": {"names": ["swimmer", "boat", "jetski", "life_saving_appliances", "buoy"], "imgsz": 640, "batch": 2, "package_model_dir": "maritime_person", "artifact_prefix": "maritime_person", "default_model": "yolo11n.pt"},
    "ir_hit_uav": {"names": ["person", "bicycle", "car", "other_vehicle"], "imgsz": 640, "batch": 2, "package_model_dir": "thermal_uav", "artifact_prefix": "thermal_uav", "default_model": "yolo11n.pt"},
    "wildfire_wit_uas": {"names": ["person", "car", "bicycle", "other_vehicle"], "imgsz": 640, "batch": 1, "package_model_dir": "wildfire_ir", "artifact_prefix": "wildfire_ir", "default_model": "yolo11n.pt"},
    # Kept only to reproduce the original three-class baseline.  Do not use it
    # for a new operational run: its validation split has no building sample.
    "wildfire_fireman_ir": {"names": ["fire_region", "smoke_region", "building"], "imgsz": 640, "batch": 8, "package_model_dir": "wildfire_ir", "artifact_prefix": "fireman_ir", "default_model": "yolo11n.pt"},
    # Thermal v2 starts with a medium-small detector rather than the nano
    # baseline.  HSV manipulations are intentionally zero: thermal intensity
    # is physical signal, not arbitrary colour.  The options can still be
    # explicitly overridden by a researcher through Ultralytics if justified.
    "wildfire_fireman_ir_fs": {
        "names": ["fire_region", "smoke_region"], "imgsz": 640, "batch": 2,
        "package_model_dir": "wildfire_ir", "artifact_prefix": "fireman_ir_fs",
        "default_model": "yolo11s.pt",
        "training_kwargs": {
            "optimizer": "AdamW", "lr0": 0.001, "lrf": 0.01, "cos_lr": True,
            "patience": 20, "close_mosaic": 10,
            "hsv_h": 0.0, "hsv_s": 0.0, "hsv_v": 0.0,
        },
    },
}


def read_names(dataset_yaml: Path) -> List[str]:
    names = []
    in_names = False
    for line in dataset_yaml.read_text(encoding="utf-8").splitlines():
        if line.strip() == "names:":
            in_names = True; continue
        if in_names and line.startswith("  ") and ":" in line:
            names.append(line.split(":", 1)[1].strip().strip("'\""))
        elif in_names and line and not line.startswith(" "):
            break
    return names


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def validation_metric(validation: dict, needle: str) -> float:
    matches = [float(value) for key, value in validation.items()
               if needle.lower() in key.lower() and isinstance(value, (int, float))]
    if not matches:
        raise ValueError(f"validation output has no metric matching {needle!r}: {sorted(validation)}")
    return matches[0]


def enforce_deployment_gates(arguments: argparse.Namespace, validation: dict,
                             event_metrics: Path) -> None:
    if arguments.min_map50 is None or arguments.min_recall is None:
        raise ValueError("--refresh-package-models requires explicit --min-map50 and --min-recall gates")
    map50 = validation_metric(validation, "mAP50(B)")
    recall = validation_metric(validation, "recall(B)")
    if map50 < arguments.min_map50 or recall < arguments.min_recall:
        raise ValueError(f"deployment gate failed: mAP50={map50:.4f}, recall={recall:.4f}")
    if arguments.profile == "wildfire_fireman_ir_fs" and event_metrics is None:
        raise ValueError("wildfire_fireman_ir_fs refresh requires --event-evaluation")
    if event_metrics is not None:
        metrics = json.loads(event_metrics.read_text(encoding="utf-8"))
        if arguments.min_event_recall is not None:
            recall_value = metrics.get("event_recall")
            if recall_value is None or float(recall_value) < arguments.min_event_recall:
                raise ValueError(f"event recall deployment gate failed: {recall_value}")
        if arguments.max_false_alarms_per_hour is not None:
            alarm_rate = metrics.get("false_alarm_events_per_hour")
            if alarm_rate is None or float(alarm_rate) > arguments.max_false_alarms_per_hour:
                raise ValueError(f"false-alarm deployment gate failed: {alarm_rate}")
    if arguments.profile == "wildfire_fireman_ir_fs" and (
            arguments.min_event_recall is None or
            arguments.max_false_alarms_per_hour is None):
        raise ValueError("wildfire v2 refresh requires explicit event recall and false-alarm gates")


def refresh_package_models(profile_name: str, profile: dict, best: Path,
                           onnx: Path, engine: Path, package_model_dir: Path,
                           validation: dict,
                           event_metrics: Path = None,
                           allow_missing_engine: bool = False) -> None:
    """Publish a completed training result into the runtime package.

    Files are first copied to a private staging directory.  A killed training
    process therefore leaves the previously deployed model untouched.  The
    stable names are what rescue_profile.launch uses on a clean checkout.
    """
    package_model_dir.mkdir(parents=True, exist_ok=True)
    prefix = profile["artifact_prefix"]
    artifacts = {
        f"{prefix}.pt": best,
        f"{prefix}.onnx": onnx,
    }
    if engine.is_file():
        artifacts[f"{prefix}_fp16.engine"] = engine
    elif not allow_missing_engine:
        raise FileNotFoundError(
            "runtime refresh requires a newly built TensorRT engine; use "
            "--allow-onnx-only-refresh only for an intentionally ONNX-only deployment")
    if event_metrics is not None and event_metrics.is_file():
        artifacts[f"{prefix}_event_metrics.json"] = event_metrics
    staging = package_model_dir / f".staging-{os.getpid()}"
    staging.mkdir()
    try:
        for name, source in artifacts.items():
            if not source.is_file():
                raise FileNotFoundError(f"training artifact does not exist: {source}")
            shutil.copy2(source, staging / name)
        artifact_manifest = {
            name: {"sha256": sha256_file(staging / name),
                   "size_bytes": (staging / name).stat().st_size}
            for name in artifacts
        }
        manifest = {
            "profile": profile_name,
            "class_names": profile["names"],
            "updated_at_utc": datetime.now(timezone.utc).isoformat(),
            "checkpoint": str(best),
            "onnx": str(onnx),
            "engine_updated": engine.is_file(),
            "engine": str(engine) if engine.is_file() else "",
            "validation": {key: float(value) for key, value in validation.items()
                           if isinstance(value, (int, float))},
            "artifacts": artifact_manifest,
            "release_id": datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ") +
                          "-" + artifact_manifest[f"{prefix}.onnx"]["sha256"][:12],
        }
        if event_metrics is not None and event_metrics.is_file():
            manifest["event_metrics"] = json.loads(event_metrics.read_text(encoding="utf-8"))
        (staging / "training_metadata.json").write_text(
            json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
        # Publish the self-consistent artifact set first and its manifest last.
        # Consumers verify the configured engine digest at process startup.
        for staged in sorted(staging.iterdir(),
                             key=lambda path: path.name == "training_metadata.json"):
            staged.replace(package_model_dir / staged.name)
    finally:
        # The directory should be empty after the atomic replacements.  Any
        # unexpected leftover is removed file-by-file, never the package dir.
        for leftover in staging.iterdir():
            leftover.unlink()
        staging.rmdir()


def run_event_evaluation(arguments: argparse.Namespace, best: Path) -> Path:
    """Run temporal evaluation before any optional runtime-model refresh."""
    event_manifest = arguments.event_manifest
    if event_manifest is None:
        event_manifest = arguments.data.parent / "manifest.jsonl"
    if not event_manifest.is_file():
        raise FileNotFoundError("event evaluation needs manifest.jsonl; pass --event-manifest")
    output = arguments.event_output or (best.parent / "event_metrics.json")
    evaluator = Path(__file__).with_name("evaluate_temporal_events.py")
    command = [sys.executable, str(evaluator), "--model", str(best),
               "--dataset", str(event_manifest.parent), "--split", "val",
               "--device", arguments.device, "--confidence", str(arguments.event_confidence),
               "--iou", str(arguments.event_iou), "--output", str(output)]
    if arguments.event_altitude_bins:
        command.extend(["--altitude-bins", arguments.event_altitude_bins])
    print("Evaluating event-level metrics: " + " ".join(command))
    subprocess.run(command, check=True)
    return output


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", choices=sorted(PROFILES), required=True)
    parser.add_argument("--data", type=Path, required=True, help="converted dataset.yaml")
    parser.add_argument("--model", default="", help="pretrained model or .pt checkpoint; profile default when omitted")
    parser.add_argument("--epochs", type=int, default=100)
    parser.add_argument("--imgsz", type=int)
    parser.add_argument("--batch", type=int)
    parser.add_argument("--device", default="0", help="CUDA device; use cpu only for a smoke test")
    parser.add_argument("--workers", type=int, default=8,
                        help="dataloader workers; use 0 only for debugging")
    parser.add_argument("--cache", choices=("none", "disk", "ram"), default="disk",
                        help="cache images for repeated epochs; disk is safest on a 4GB GPU")
    parser.add_argument("--project", type=Path, default=Path("runs/sar_yolo"))
    parser.add_argument("--name")
    parser.add_argument("--export-engine", action="store_true",
                        help="build TensorRT engine with the host trtexec after ONNX export")
    parser.add_argument("--trtexec", default="",
                        help="optional trtexec path (auto-detected when omitted)")
    parser.add_argument("--refresh-package-models", action="store_true",
                        help="atomically replace the profile's packaged runtime models")
    parser.add_argument("--package-model-dir", type=Path,
                        help="override package model directory used by refresh")
    parser.add_argument("--allow-onnx-only-refresh", action="store_true",
                        help="explicitly permit refresh without a new TensorRT engine")
    parser.add_argument("--min-map50", type=float,
                        help="required minimum validation mAP50 for runtime refresh")
    parser.add_argument("--min-recall", type=float,
                        help="required minimum validation recall for runtime refresh")
    parser.add_argument("--min-event-recall", type=float,
                        help="minimum temporal event recall for runtime refresh")
    parser.add_argument("--max-false-alarms-per-hour", type=float,
                        help="maximum temporal false-alarm event rate for runtime refresh")
    parser.add_argument("--event-evaluation", action="store_true",
                        help="evaluate event recall, false alarms, latency and altitude bins on validation")
    parser.add_argument("--event-manifest", type=Path,
                        help="converted manifest.jsonl; default is alongside --data")
    parser.add_argument("--event-output", type=Path,
                        help="event metric JSON; default is weights/event_metrics.json")
    parser.add_argument("--event-confidence", type=float, default=0.25)
    parser.add_argument("--event-iou", type=float, default=0.50)
    parser.add_argument("--event-altitude-bins", default="0,30,60,120")
    arguments = parser.parse_args()
    profile = PROFILES[arguments.profile]
    observed_names = read_names(arguments.data)
    if observed_names != profile["names"]:
        raise ValueError(f"Dataset class order {observed_names} does not match {arguments.profile}")
    try:
        from ultralytics import YOLO
    except ImportError as error:
        raise SystemExit("Install training/requirements.txt in a dedicated training venv") from error
    model = YOLO(arguments.model or profile.get("default_model", "yolo11n.pt"))
    training_kwargs = dict(profile.get("training_kwargs", {}))
    results = model.train(
        data=str(arguments.data), epochs=max(1, arguments.epochs),
        imgsz=arguments.imgsz or profile["imgsz"], batch=arguments.batch or profile["batch"],
        device=arguments.device, project=str(arguments.project),
        name=arguments.name or arguments.profile,
        cache=False if arguments.cache == "none" else arguments.cache,
        workers=max(0, arguments.workers),
        pretrained=True, deterministic=True, **training_kwargs)
    best = Path(results.save_dir) / "weights" / "best.pt"
    evaluator = YOLO(str(best)); validation_result = evaluator.val(
        data=str(arguments.data), device=arguments.device)
    validation = getattr(validation_result, "results_dict", {}) or {}
    event_metrics = run_event_evaluation(arguments, best) if arguments.event_evaluation else None
    evaluator.export(format="onnx", imgsz=arguments.imgsz or profile["imgsz"],
                     dynamic=False, simplify=True, half=False)
    if arguments.export_engine:
        trtexec = arguments.trtexec or shutil.which("trtexec")
        if not trtexec:
            raise SystemExit("--export-engine requires the system TensorRT trtexec binary")
        onnx = best.with_suffix(".onnx")
        engine = best.with_suffix(".engine")
        command = [trtexec, f"--onnx={onnx}", f"--saveEngine={engine}",
                   "--fp16", "--memPoolSize=workspace:1024"]
        print("Building TensorRT engine with host trtexec: " + " ".join(command))
        subprocess.run(command, check=True)
    if arguments.refresh_package_models:
        enforce_deployment_gates(arguments, validation, event_metrics)
        package_model_dir = arguments.package_model_dir
        if package_model_dir is None:
            package_model_dir = (Path(__file__).resolve().parents[1] / "models" /
                                 profile["package_model_dir"])
        refresh_package_models(arguments.profile, profile, best,
                               best.with_suffix(".onnx"), best.with_suffix(".engine"),
                               package_model_dir, validation, event_metrics,
                               arguments.allow_onnx_only_refresh)
        print(f"Package models refreshed atomically: {package_model_dir}")
    print(f"Training/export complete. Best checkpoint: {best}")


if __name__ == "__main__":
    main()
