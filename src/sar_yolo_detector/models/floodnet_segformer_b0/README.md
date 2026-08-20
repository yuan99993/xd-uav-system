# FloodNet SegFormer-B0 profile

This directory is the unified `sar_yolo_detector` copy of the FloodNet pilot
artifact. It is selected as `profile:=floodnet` from
`launch/rescue_profile.launch`.

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

Re-export after changing the checkpoint with the packaged helper:

```bash
VENV=/home/promise/mrs_test/.venv-sar-flood
CKPT=/home/promise/mrs_test/src/sar_yolo_detector/models/floodnet_segformer_b0/floodnet_segformer_b0_pilot_10e_best.pt
OUT=${CKPT%.pt}.onnx
$VENV/bin/python /home/promise/mrs_test/src/sar_yolo_detector/training/export_floodnet_onnx.py \
  --checkpoint "$CKPT" --output "$OUT" --input-size 1024
/usr/src/tensorrt/bin/trtexec --onnx="$OUT" \
  --saveEngine="${CKPT%.pt}_fp16.engine" --fp16 \
  --memPoolSize=workspace:2048 \
  --inputIOFormats=fp16:chw --outputIOFormats=fp16:chw
```
The package now includes `sar_segformer_inference`, which preprocesses RGB
using ImageNet mean/std, runs FP16/FP32 TensorRT, argmaxes the ten logits,
resizes the class mask with nearest-neighbour sampling, and preserves the
capture timestamp. `rescue_profile.launch profile:=floodnet` starts it together
with the region extractor. Topics are relative and resolve under the selected
aircraft namespace; neither node sends flight commands.
