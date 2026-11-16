# FloodNet SegFormer-B0 pilot (retained artifact)

The trimmed runtime package no longer builds the FloodNet inference and region
extraction nodes. These files are kept as model artifacts and historical
training records; they are not used by the current SmartTracker Fusion launch.

This directory retains the FloodNet pilot artifacts from the former rescue
profile. The profile launch and its inference nodes were removed from the
trimmed runtime package.

The packaged checkpoint is a PyTorch research artifact:

- `floodnet_segformer_b0_pilot_10e_best.pt`
- FloodNet official split: 1,445 train / 450 validation / 448 test
- 10 epochs, 1024 px crop, effective batch 12
- best validation mIoU: 0.4843
- hazard-class mean IoU (building_flooded, road_flooded, water): 0.4735

## Deployment exports

The checkpoint has been exported and checked on this host:

- [ONNX graph](floodnet_segformer_b0_pilot_10e_best.onnx), opset 17,
  fixed input `pixel_values=[1,3,1024,1024]` and output
  `logits=[1,10,256,256]` (native SegFormer quarter-resolution logits).
- [TensorRT 10.1 FP16 engine](floodnet_segformer_b0_pilot_10e_fp16.engine),
  fixed input/output bindings and about 13.1 MiB on disk.
- [training_metadata.json](training_metadata.json) records the export and
  benchmark details.

The ONNX graph was checked with ONNX Runtime (CPU provider); its output shape
matched PyTorch and the maximum absolute error was `2.6e-5`. TensorRT
`trtexec` built the engine with a 2048 MiB workspace. On the RTX 3050, the
random-input benchmark measured about 46.99 ms GPU compute per 1024 crop
(p50 46.15 ms, p90 47.88 ms, 20.7 qps). This is an inference-only benchmark;
camera copy, preprocessing, argmax and mask resizing are not included, and
clock throttling explains occasional tail latency.

The former deployment path used `sar_segformer_inference` to preprocess RGB,
run FP16/FP32 TensorRT, produce a class mask, and preserve the capture
timestamp. That runtime path is not included in the trimmed package.
