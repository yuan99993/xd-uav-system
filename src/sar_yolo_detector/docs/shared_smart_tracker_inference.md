# Shared SmartTracker inference across UAV launches

`xd_smart_tracker_fusion_integration.launch` now starts a lightweight model
provider candidate alongside each vehicle tracker. Candidates use a local
file lock keyed by model SHA-256: the first one loads the model and exposes a
ROS inference service, while later launches reuse that provider. If its
launch exits, another active candidate takes over. Launch invocations and
existing model arguments do not need to change.

Each UAV still owns its own SmartTracker, target identity, gimbal outputs, and
annotated image topic. Only YOLO inference is shared. Requests use a latest
frame slot per source and are capped at 7 inference requests per second per
tracker; old queued requests are replaced or discarded when stale.

When the configured model is a static TensorRT engine with a valid sidecar
manifest and verified source `.pt` file, the provider attempts to create a
dynamic batch-2 FP16 engine in the TensorRT cache. The original model and
launch arguments remain unchanged. If export fails or the source model is
unavailable, the provider uses the configured engine with batch size 1 and
logs that detections are serialized. PyTorch `.pt` models use batch-2
inference directly.

The per-tracker `inference_max_fps`, `inference_frames_replaced`, and
`inference_rate_limited_frames` diagnostic fields help distinguish the
requested rate from delivered throughput. Shared weights reduce duplicate
model memory and GPU contexts; they do not guarantee a particular aggregate
FPS. The dynamic engine must be benchmarked with all aircraft running to
confirm whether the board sustains 7 detections per second per UAV.
