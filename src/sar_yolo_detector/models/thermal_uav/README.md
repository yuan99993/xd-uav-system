# Packaged thermal_uav model

This directory contains the 30-epoch HIT-UAV training result shipped with
`sar_yolo_detector`, so another catkin workspace can run the detector without
retraining. The stable `thermal_uav.*` names are refreshed by
`train_profile.py --refresh-package-models` after a successful training run:

- `thermal_uav.onnx`: portable source graph (YOLO11n, four
  classes: `person`, `bicycle`, `car`, `other_vehicle`).
- `thermal_uav_fp16.engine`: ready-to-run FP16 TensorRT engine
  built with TensorRT 10.1 for NVIDIA SM86 (RTX 30-series, including the
  development RTX 3050).
- `thermal_uav.pt`: training checkpoint for later fine-tuning.

The `hit_uav_yolo11n_*` files are the original immutable smoke-run artifacts.

The packaged checkpoint was trained for 30 epochs on HIT-UAV v1.2 with
YOLO11n, 640x640 input and batch 1. On the 290-image validation split it
achieved precision 0.697, recall 0.598, mAP50 0.663 and mAP50-95 0.400;
the `person` class reached mAP50 0.856. These are offline validation results,
not a substitute for field or closed-loop flight validation. The engine is
GPU/TensorRT-version specific; on another architecture, build a new engine
from the packaged ONNX with that host's `trtexec`.

SHA-256 (30-epoch packaged result):

```text
43fe0095183236d90bbe46db033ed26037915fa41e518c735deed090a3f8a087  thermal_uav.onnx
e35ef56cc9d01a8db6fd5f6a6c56626e73c40f932f645cc48ccf1bc4a723f7ea  thermal_uav_fp16.engine
f0a98c8bcc6629c6be137352307716e58c0852a21b043e9b601fbf75d64eecfc  thermal_uav.pt
```
