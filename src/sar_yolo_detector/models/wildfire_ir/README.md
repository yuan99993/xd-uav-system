# Packaged FireMan thermal wildfire model

This directory contains the 50-epoch FireMan Multiclass thermal training
result.  The runtime profile is `wildfire_ir`; the stable artifacts are
`fireman_ir.pt`, `fireman_ir.onnx`, and the TensorRT 10.1 FP16 engine
`fireman_ir_fp16.engine`.

The model was trained with YOLO11n, 640x640 input, batch 8 and three classes:
`fire_region`, `smoke_region`, and `building`.  The FireMan thermal validation
split contains no `building` instances, so that class cannot be evaluated from
this run.  The published best checkpoint is selected by mAP50-95 and reached:

- overall precision 0.995, recall 0.278, mAP50 0.289, mAP50-95 0.224;
- `fire_region`: precision 0.990, recall 0.556, mAP50 0.560, mAP50-95 0.446;
- `smoke_region`: precision 1.000, recall 0.000, mAP50 0.019, mAP50-95 0.003.

This is a dataset/conversion baseline, not a production wildfire detector:
smoke recall is insufficient and the validation split is not class-balanced.
Retrain with a video-disjoint split before using it for autonomous task
generation.  The recommended v2 pipeline deliberately removes the sparse
`building` class, samples source videos at 1--3 FPS, disables thermal HSV
augmentation, supports curated hard negatives, and evaluates event-level recall
and false alarms.  It produces separate, opt-in artifacts named
`fireman_ir_fs.pt`, `fireman_ir_fs.onnx`, and `fireman_ir_fs_fp16.engine`.

The existing `wildfire_ir` runtime profile remains pinned to this legacy
three-class engine.  Do not replace it merely because a new training run
finished.  After reviewing `fireman_ir_fs_event_metrics.json`, launch the
separate `wildfire_ir_fire_smoke` profile; it creates task candidates only for
`fire_region` and keeps smoke as a non-navigation diagnostic cue.

TensorRT `trtexec` on the development RTX 3050 reported 4.459 ms mean
end-to-end latency, 3.650 ms mean GPU compute time and 217.7 qps.  The engine
is GPU/ TensorRT-version specific; rebuild it on another GPU architecture.

SHA-256:

```text
53abb905aa2fedb55ae8598c8b9750ac59ac47be03ad99b82aa86d90ca170234  fireman_ir.pt
a8a3af6487a6221699a0051dcde97079004ddd627011c95c8eca1a2b8a160a1f  fireman_ir.onnx
a798ae337f474cd49fd7f88b4a74d25af086a4148efeb33524d883ad2eb68410  fireman_ir_fp16.engine
```
