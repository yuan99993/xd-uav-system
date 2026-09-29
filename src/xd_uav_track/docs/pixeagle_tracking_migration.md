# PixEagle SmartTracker behavior in `xd_uav_track`

This is an algorithmic port into the ROS tracker core, not a runtime wrapper
around PixEagle. The detector and the task/follower ROS contracts remain owned
by their existing packages and messages.

| PixEagle behavior | ROS implementation |
| --- | --- |
| Stable lock identity distinct from changing detector IDs | `TrackState.track_id` remains the persistent tracker/global identity; `detector_track_id` and `detector_id_stable` retain upstream provenance. Per-camera managers remain isolated. |
| ID-first, predicted-IoU, distance, and appearance matching | `MultiTrackManager` combines gated source-ID evidence, Kalman-predicted box overlap/distance, an appearance prototype gallery, and global Hungarian assignment. Existing high/low-confidence passes are retained. |
| Best-vs-runner-up ambiguity hold | Near-tied viable assignments are withheld, marked through `association_method="ambiguous"`, and cannot seed replacement tracks for that frame. The node withholds ambiguous measurements from the controller. |
| Class-history tolerance and confidence aging | Recent class history permits bounded class flicker; confidence is smoothed on updates and decays with elapsed time during loss. |
| Normal coast, expanded search, and lost lifecycle | Kalman prediction, configurable time-based occlusion/removal windows, bounded search expansion, and frame-count fallback for legacy configurations. |
| Long-loss identity verification and tentative reacquisition | Configurable `aggressive` / `balanced` / `strict` policy, size and exit-edge gates, same-candidate consecutive confirmation, and no control measurement until confirmation. Public identity stays unchanged. |
| Duplicate/out-of-order timestamp protection | The source callback and `MultiTrackManager` reject duplicate or backward timestamps without updating track state; clock-reset handling remains source-local. Kalman prediction uses observation-time deltas. |
| Appearance quality and re-identification memory | The existing tracker-owned, class-profiled ONNX/deep/hybrid ReID frontend supplies ROI embeddings; the C++ manager maintains bounded, quality-gated prototypes. The frontend no longer writes an undeclared `appearance_quality` ROS field; low-quality ROIs carry no appearance vector and fall back to motion/metric evidence. |

The ROS `DetectionArray`, `DetectionCandidate`, `TrackState`, service, and
follower output schemas were not changed. Existing `sar_yolo_detector` YOLO
weights remain the detection authority; this migration does not load a second
detector or replace the train7 model.

PixEagle's configured `models/yolo26n.pt` is not present as a usable model
artifact in its checkout, and its configured default BoT-SORT path is not a
tank-trained detector or ReID model. The standalone video demonstration used a
manually initialized CSRT tracker, so it is not a measurement of the
SmartTracker YOLO/BoT-SORT pipeline. CSRT/KCF/Dlib UI plugins are not substituted
for this ROS multi-target detection pipeline. The vehicle ONNX ReID artifact
already in `xd_uav_track` is a generic vehicle descriptor, not a tank-domain
validated checkpoint; this code change makes no tank-ReID accuracy claim.

Configuration lives in `config/track.yaml`; the fixed-wing orbit overlay sets
an explicit 40-second identity retention window in
`config/fixedwing_metric_orbit.yaml`. A focused regression suite covers
ambiguity hold, timestamp rejection, time-based loss/removal, stable identity,
and appearance-based reacquisition.
