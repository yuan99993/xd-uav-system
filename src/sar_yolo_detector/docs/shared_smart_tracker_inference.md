# Shared SmartTracker inference across UAVs

`xd_smart_tracker_fusion_integration.launch` starts one elected provider for a
model SHA-256. Each UAV keeps its own tracker and sends image requests to that
provider. The default Orin deployment uses the dynamic batch-3 TensorRT engine,
allows a maximum batch of three, and waits up to 10 ms for compatible requests.
Override `maximum_batch_size` or `batch_wait_ms` on the launch command line to
adjust those limits.

The provider has one model worker. It groups compatible pending requests into
one Ultralytics `predict` call; it does not run three concurrent
calls through one TensorRT execution context. Requests must have matching image
shape, confidence, IoU, detection limit, and image-size override. Actual batch
size depends on requests arriving together, so `maximum_batch_size=3` does not
mean every inference uses three images.

The shared client already has each frame decoded for tracking. Before sending a
request, it shrinks the frame's long side to
`SMART_TRACKER_SHARED_INPUT_MAX_SIDE` (default: the configured
`SMART_TRACKER_INFERENCE_IMAGE_SIZE`, or 512 for the XD vehicle profile). The
client maps returned detections back to the original frame's coordinate
system. Set the shared max side to `0` to disable this
transfer/preprocessing optimization.

Every ten seconds the provider logs `shared YOLO perf` with recent request rate,
counts of batch-1, batch-2, and batch-3 calls, and average model-call duration. Each
tracker's diagnostics expose `inference_ms` for the full client request and
`shared_model_inference_ms` for the provider's model call. A large gap points to
queueing, serialization, or service transport overhead; a large model-call
duration points to inference/postprocessing cost.

The per-tracker `inference_max_fps` defaults to 20 in the integration launch.
Actual delivered inference rate can be lower when the shared provider is busy;
check `inference_completed`, `inference_frames_replaced`, and the provider's
batch distribution before raising the requested rate. A larger batch wait can
improve batch occupancy when several sources are active, at the cost of added
request latency.
