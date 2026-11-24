# SAR profile training and export

These are offline data-preparation/training tools. They are deliberately not a
runtime dependency of `sar_yolo_detector`: flight-time image inference,
timestamps, diagnostics, and `vision_msgs/Detection2DArray` publication remain
in C++.

Create a separate environment on the GPU training host:

```bash
python3 -m venv .venv-sar-yolo
source .venv-sar-yolo/bin/activate
pip install -r src/sar_yolo_detector/training/requirements.txt
```

Do not use `src/PixEagle/.venv` as a deployment dependency. It is unrelated to
this package and may not exist on collaborator machines.

## 1. EO: VisDrone2019-DET

The converter retains its ten official drone-view categories and excludes only
ignored/other regions:

```bash
python3 src/sar_yolo_detector/training/prepare_visdrone.py \
  --source /data/VisDrone2019-DET --output /data/sar_yolo/visdrone
python3 src/sar_yolo_detector/training/train_profile.py \
  --profile eo_visdrone --data /data/sar_yolo/visdrone/dataset.yaml \
  --device 0 --epochs 100
```

## 2. Maritime EO: SeaDronesSee v2

Download the official train/validation images and matching COCO JSON labels,
then preserve the class contract through the supplied mapping:

```bash
python3 src/sar_yolo_detector/training/prepare_coco.py \
  --images train=/data/seadronessee/images/train \
  --images val=/data/seadronessee/images/val \
  --annotations train=/data/seadronessee/instances_train.json \
  --annotations val=/data/seadronessee/instances_val.json \
  --class-map src/sar_yolo_detector/training/mappings/seadronessee_v2.json \
  --output /data/sar_yolo/seadronessee
python3 src/sar_yolo_detector/training/train_profile.py \
  --profile maritime_seadronessee \
  --data /data/sar_yolo/seadronessee/dataset.yaml --device 0 --epochs 100
```

## 3. IR: HIT-UAV

The native HIT-UAV v1.2 JSON uses COCO-like `filename`/`annotation` keys; the
converter accepts those keys as well as canonical COCO `file_name`/`annotations`.
`dontcare` is intentionally absent from the class map and is ignored.

```bash
python3 src/sar_yolo_detector/training/prepare_coco.py \
  --images train=/data/hit_uav/images/train --images val=/data/hit_uav/images/val \
  --annotations train=/data/hit_uav/instances_train.json \
  --annotations val=/data/hit_uav/instances_val.json \
  --class-map src/sar_yolo_detector/training/mappings/hit_uav.json \
  --output /data/sar_yolo/hit_uav
python3 src/sar_yolo_detector/training/train_profile.py \
  --profile ir_hit_uav --data /data/sar_yolo/hit_uav/dataset.yaml \
  --device 0 --epochs 100
```

For the low-load validation, the native HIT-UAV archive was converted to a
workspace-local directory such as `/data/sar_yolo/hit_uav/converted`
(2029 train / 290 validation images), then trained with `--batch 1 --imgsz 640
--epochs 1`. The resulting TensorRT engine was built with the host TensorRT
10.1 `trtexec`, rather than installing a second pip TensorRT runtime. Future
runs can pass `--export-engine`; the training script invokes the host
`trtexec` directly after ONNX export (or pass `--trtexec /path/to/trtexec`).
Runtime refresh also requires a newly built engine and explicit
`--min-map50/--min-recall` acceptance gates. The staged artifact manifest
records every file's SHA-256; startup rejects a digest mismatch.

`conversion_report.json` is mandatory review material: it records image,
label, skipped, malformed, and invalid-box counts. Do not train if the report
has unexpected zero class counts.

## 6. D-FINE edge-truncation fine-tuning and Letterbox engines

`prepare_dfine_edge_dataset.py` builds a map-disjoint COCO dataset for the
car/tank/drone D-FINE class contract. It requires at least 300 distinct
edge-positive images and 300 edge boxes across at least two maps, keeps one
whole map out for validation, samples ordinary positive baseline frames at a
3:1 edge-to-baseline image ratio, and keeps explicitly audited negatives at
20% of the resulting sample count. The 8% edge band is configurable. Empty
COCO annotation lists are not assumed to be negatives: mark an image record
`verified_negative: true` only after reviewing it.

Each `--map-id` is a separate scene/map. Repeat `--map-id`, `--images` and
`--annotations` in matching order. The map used by `--validation-map` must
contain labeled edge targets and must not also appear as a training source.
Before running, copy/review the annotations and add `verified_negative: true`
to confirmed empty frames:

```bash
python3 src/sar_yolo_detector/training/prepare_dfine_edge_dataset.py \
  --map-id map_a --images /data/map_a/images --annotations /data/map_a/instances.json \
  --map-id map_b --images /data/map_b/images --annotations /data/map_b/instances.json \
  --map-id map_c --images /data/map_c/images --annotations /data/map_c/instances.json \
  --validation-map map_c --output /data/dfine_edge/dataset
```

The training profile under `training/dfine/` uses a D-FINE `LetterboxResize`
transform with gray 114 fill at 512×512, and runs the upstream trainer without
modifying vendored D-FINE source. Convert the existing integrity-checked model
bundle to the upstream tuning-checkpoint format, then fine-tune for ten epochs:

```bash
python3 src/sar_yolo_detector/training/dfine/bundle_to_checkpoint.py \
  --manifest src/sar_yolo_detector/models/dfine_s_v3_16e/manifest.json \
  --output /data/dfine_edge/dfine_v3_start.pth
# Set the dataset and output paths in training/dfine/edge_letterbox_s_10e.yml.
python3 src/sar_yolo_detector/training/dfine/run_letterbox_training.py -- \
  --config src/sar_yolo_detector/training/dfine/edge_letterbox_s_10e.yml \
  --tuning /data/dfine_edge/dfine_v3_start.pth --use-amp --seed 7 \
  --device cuda:0 --output-dir /data/dfine_edge/run_s_10e
```

Package the selected training checkpoint into a new versioned model directory;
do not replace the current bundle before validation. The manifest records the
preprocessing policy, and runtime refuses a Letterbox/stretch mismatch unless
the experimental override is explicitly enabled.

```bash
python3 src/sar_yolo_detector/training/dfine/package_checkpoint.py \
  --config src/sar_yolo_detector/training/dfine/edge_letterbox_s_10e.yml \
  --checkpoint /data/dfine_edge/run_s_10e/best.pth \
  --destination src/sar_yolo_detector/models/dfine_s_v3_edge_letterbox_10e \
  --preprocess-mode letterbox
python3 src/sar_yolo_detector/training/dfine/export_raw_onnx.py \
  --manifest src/sar_yolo_detector/models/dfine_s_v3_edge_letterbox_10e/manifest.json \
  --size 512 --output /data/dfine_edge/dfine_edge_512.onnx
python3 src/sar_yolo_detector/training/dfine/export_raw_onnx.py \
  --manifest src/sar_yolo_detector/models/dfine_s_v3_edge_letterbox_10e/manifest.json \
  --size 320 --output /data/dfine_edge/dfine_edge_320.onnx
trtexec --onnx=/data/dfine_edge/dfine_edge_512.onnx \
  --saveEngine=/data/dfine_edge/dfine_edge_512.engine \
  --shapes=images:1x3x512x512 --fp16
trtexec --onnx=/data/dfine_edge/dfine_edge_320.onnx \
  --saveEngine=/data/dfine_edge/dfine_edge_320.engine \
  --shapes=images:1x3x320x320 --fp16
```

The engine adapter requires bindings `images`, `logits` and `boxes`; keep these
names when exporting. Set the new model-manifest SHA, engine SHA values,
`SMART_TRACKER_DFINE_RESIZE_MODE: letterbox`, and engine paths together in a
separate deployment profile. Acceptance needs held-out edge recall, false
positive rate on the audited negatives, detector latency, and border-exit / ID
switch metrics before promotion.

The current workspace has the existing D-FINE safetensors model and ROS bag
reports, but no labeled multi-map COCO edge dataset or verified-negative
manifest. Therefore sample collection, ten-epoch fine-tuning, and 512/320
TensorRT engine generation remain unperformed; the scripts above prepare that
workflow and do not claim a retrained model or measured gain.

Use a CUDA/TensorRT-enabled virtual environment created on the training host.
A 4 GB GPU should start with
`--batch 1 --imgsz 640`; use short smoke runs before increasing epochs or
resolution. Full three-profile training should be scheduled separately to
avoid exhausting GPU memory.

## 4. FloodNet SegFormer-B0

FloodNet training is kept with the unified package under
`training/floodnet/`. Install its optional dependencies separately:

```bash
python3 -m venv .venv-sar-flood
source .venv-sar-flood/bin/activate
pip install -r src/sar_yolo_detector/training/floodnet/requirements.txt
```

Before training, edit the `dataset.archive`, `dataset.extraction_root` and
`output.root` values in the selected YAML profile to paths on the current
machine. The trainer does not assume a fixed catkin workspace location:

```bash
python3 src/sar_yolo_detector/training/floodnet/train_floodnet_segformer.py \
  --config src/sar_yolo_detector/training/floodnet/floodnet_segformer_b0_pilot_10e.yaml
```

The packaged FloodNet artifacts and the pilot assessment are documented in
`models/floodnet_segformer_b0/README.md` and
`training/floodnet/FLOODNET_TRAINING_RESULTS.md`.

## 5. Wildfire IR: FireMan thermal v2 (recommended)

The original 50-epoch FireMan run is a three-class conversion baseline only.
Its frames were not re-split by flight group, it used colour HSV augmentation
on thermal imagery, and the validation subset contained no `building` labels.
Do not overwrite it.  The v2 workflow makes a separate two-class
`fire_region`/`smoke_region` data set, keeps every video in one split, and
samples videos at 1--3 FPS.

First verify the source video's actual frame rate from the dataset metadata or
the original recording.  The command below uses 30 only as an example; it is
not a claim about every FireMan release.  `--dry-run` prints the selected
whole-video split before it writes any image links.

```bash
python3 src/sar_yolo_detector/training/prepare_fireman_multiclass.py \
  --source /data/fireman/extracted/Multiclass/Unimodal/thermal \
  --out /data/sar_yolo/fireman_thermal_v2 \
  --source-fps 30 --sample-fps 2 --dry-run
```

After reviewing the printed class counts and group allocation, rerun without
`--dry-run`.  The output contains `manifest.jsonl`, an auditable mapping from
each YOLO image to video/group/timestamp, and `group_metadata_template.csv`.
Fill `altitude_m` in that CSV when source telemetry is available and rerun with
`--group-metadata-csv` to enable altitude-stratified recall.

Curate cloud, fog, steam, sunset, hot roof, and vehicle-exhaust imagery before
the production run.  Put real paths in a copy of
`hard_negative_manifest.example.csv`; one `group_id` must represent one
continuous flight/sequence, not a unique image.  The converter assigns the
entire negative sequence to one split and creates empty YOLO labels:

```bash
python3 src/sar_yolo_detector/training/prepare_fireman_multiclass.py \
  --source /data/fireman/extracted/Multiclass/Unimodal/thermal \
  --out /data/sar_yolo/fireman_thermal_v2 \
  --source-fps <verified_fps> --sample-fps 2 \
  --negative-manifest /data/sar_yolo/hard_negatives.csv
```

The recommended comparison uses YOLO11s.  Its FireMan v2 profile disables
`hsv_h`, `hsv_s`, and `hsv_v` by default and uses a lower AdamW learning rate,
cosine schedule, and early stopping.  On the current 4 GB GPU, start at the
profile's batch 2; reduce to 1 if CUDA reports OOM.  Event evaluation is run
before an optional packaged-model refresh, so an mAP-only result cannot silently
be deployed.

```bash
python3 src/sar_yolo_detector/training/train_profile.py \
  --profile wildfire_fireman_ir_fs \
  --data /data/sar_yolo/fireman_thermal_v2/dataset.yaml \
  --device 0 --epochs 100 --event-evaluation \
  --project /data/sar_yolo/runs --name fireman_thermal_v2_yolo11s
```

`weights/event_metrics.json` records event recall, per-class event recall,
false-alarm *events* per observed hour (not raw box count), first correct
detection latency, and altitude-band recall when altitude metadata exists.
Review the file before adding the explicit deployment gates, for example:

```bash
  --export-engine --refresh-package-models \
  --min-map50 0.60 --min-recall 0.60 \
  --min-event-recall 0.80 --max-false-alarms-per-hour 2.0
```

The FireMan v2 profile refuses refresh unless event evaluation and both event
gates are present.

The packaged v2 runtime profile is deliberately separate:

```bash
roslaunch sar_yolo_detector rescue_profile.launch \
  profile:=wildfire_ir_fire_smoke inference_backend:=tensorrt
```

It expects `models/wildfire_ir/fireman_ir_fs_fp16.engine`, which is absent
until a validated v2 run is explicitly packaged. Task generation is still
off by default; after explicit mission enablement only `fire_region` may create
a candidate, while `smoke_region` stays visible to the operator/decision layer
but is never falsely ground-projected as a navigation target.

## 6. Wildfire IR: legacy FireMan Multiclass baseline

The FireMan archive contains RGB and thermal CVAT exports.  This legacy command
reproduces the original three-class baseline only.  It is not the preferred
production pipeline because the classes do not have complete validation data:

```bash
python3 src/sar_yolo_detector/training/prepare_fireman_multiclass.py \
  --source /data/fireman/extracted/Multiclass/Unimodal/thermal \
  --out /data/sar_yolo/fireman_thermal_legacy \
  --source-fps <verified_fps> --sample-fps 0 \
  --classes fire_region,smoke_region,building \
  --allow-incomplete-class-split
python3 src/sar_yolo_detector/training/train_profile.py \
  --profile wildfire_fireman_ir --data /data/sar_yolo/fireman_thermal_legacy/dataset.yaml \
  --device 0 --batch 8 --epochs 50 --export-engine
```

The 50-epoch baseline is packaged under `models/wildfire_ir/`.  Its validation
split has no `building` instances and smoke recall is currently insufficient,
so it must be retrained with a class-complete, video-disjoint validation split
before autonomous deployment. The legacy runtime launcher hard-blocks task
generation even if a global experimental override is supplied.

## 6. Wildfire IR: WIT-UAS

WIT-UAS is distributed through its own repository and downloader because it
contains ROS bags as well as labelled LWIR images. Convert its standard YOLO
labels to the class contract in `mappings/wit_uas.json` before calling
`train_profile.py`; the runtime profile admits `person` by default and keeps
vehicle classes available for situational awareness.

```bash
python3 src/sar_yolo_detector/training/train_profile.py \
  --profile wildfire_wit_uas --data /data/sar_yolo/wit_uas/dataset.yaml \
  --device 0 --batch 1 --epochs 1
```

After export, copy only the selected `best.onnx`/GPU-specific `.engine` and
matching configuration to deployment. The current Noetic OpenCV 4.2 cannot
load many current ONNX graphs; set `inference_backend: tensorrt` with the
engine built on the target GPU, or run `opencv_dnn` on a modern compatible
OpenCV build.
