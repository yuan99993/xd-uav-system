# Lightweight ROS video replay

This launch exercises the production `sar_yolo_detector` → `xd_uav_track`
2-D detection/identity path without Gazebo, PX4, TF, or control outputs. It
accepts an MP4/AVI/MOV file or a directory of naturally numbered JPEG/PNG
frames. The two-source mode reads one frame from each input per tick and gives
the pair identical capture timestamps. It ends at the shorter input. Inputs
must already be synchronized; `start_frame_a/b` may align a known fixed
offset. The reader does not decode ahead or queue the whole recording.
Frames wider than 1280 px are resized before publishing by default; set
`max_width:=0` to keep original resolution. Scale ground-truth boxes by
`--gt-scale-a/b` (e.g. `0.333333` for 3840→1280) when evaluating resized video.
For a 30 FPS source on a 5 FPS inference budget use `fps:=5 frame_step:=6`;
this samples every sixth original frame instead of slowing all 30 FPS frames
down. Frame markers retain the **original 1-based frame number** for truth
matching. Both inputs must have the same source FPS for a shared `frame_step`.
Set `lockstep:=true source_fps:=30` for offline frame-complete scoring: the
next image pair is sent only after both source-local tracker snapshots arrive.
This mode preserves original capture-time deltas but is **not** a real-time
throughput test; use the default paced mode to measure drops and latency.

```bash
cd /home/promise/mrs_test
source devel/setup.bash
roslaunch xd_uav_track video_tracking_replay.launch \
  video_a:=/absolute/path/to/vehicle_sequence \
  fps:=8 max_frames:=240 \
  record_path:=/home/promise/mrs_test/logs/video_single.jsonl \
  show_overlay:=true
```

For two *independent, synchronized* views:

```bash
roslaunch xd_uav_track video_tracking_replay.launch \
  video_a:=/absolute/path/to/camera_A.mp4 \
  video_b:=/absolute/path/to/camera_B.mp4 \
  start_frame_b:=0 fps:=8 max_frames:=240 \
  record_path:=/home/promise/mrs_test/logs/video_pair.jsonl \
  show_overlay:=true
```

The default model is the existing `xd_vehicle_train7.pt`; no weights are
changed. Replay enables the configured vehicle ReID front end by default so
that an identity run covers the deployed association path; set
`reid_enabled:=false` for a detector-throughput A/B run. Its overlay uses a
960-pixel full-frame input to make small overhead vehicles easier to inspect,
while prediction ROI recovery stays at 640 pixels. The production detector
profile remains 640 by default; promote a higher resolution only after the
same recording meets the latency and recall budget. The overlay draws
confirmed, actually detected boxes only. Keep
`fps` below sustainable detector throughput (8 FPS is the conservative
default). `max_frames:=0` plays to EOF. Launch exits when replay ends. ROS
topics include `/<UAV_NAME>/track/detections`, `track/tracks_by_source`, and
`fixed_camera/tracking_overlay` (plus the gimbal equivalents with two inputs).

For a live wide-area deployment, retain the normal detector profile and pass
the high-resolution overlay explicitly (it does not alter the model):

```bash
roslaunch sar_yolo_detector xd_yolo_multi_source_detection_integration.launch \
  config:=$(rospack find sar_yolo_detector)/config/xd_vehicle_detection.yaml \
  overlay_config:=$(rospack find sar_yolo_detector)/config/xd_vehicle_detection_highres.yaml \
  roi_inference_image_size:=640
```

First compare this 960-pixel setting to the 640 baseline on the identical
recording. Keep it only if small-target recall improves without exceeding the
per-source latency budget; the shared node’s diagnostics publish EWMA inference
and ROI timings, queue drops, ReID timing and CUDA memory.

`video_tracking_record_node.py` writes detection and tracking snapshots with
frame number, source, boxes and public IDs. With MOT-format ground truth:

```bash
python3 src/xd_uav_track/scripts/evaluate_video_tracking.py \
  logs/video_single.jsonl --gt-a /path/to/gt.txt
```

For a paired dataset with *shared ground-truth identity numbers*, add `--gt-b`
and the second camera's MOT file. Use `--frame-offset-a/b N` when replay starts
at zero-based image index N (MOT frame N+1). This small evaluator reports recall,
observed ID switches and simultaneous cross-source ID agreement, not full
HOTA/IDF1. A missing truth file cannot be replaced by guessing IDs from
nearby boxes.

## Dataset selection and interpretation

The downloaded [multi-UAV vehicle sample](https://huggingface.co/datasets/jye9/Multi-Camera-Multi-Vehicle-Tracking-System)
provides three synchronized one-minute real UAV views. Two source MP4s are
available locally at `data/video_replay/uav1_1min.mp4` and
`data/video_replay/uav2_1min.mp4` (about 404 MiB together). A low-load
paired check is:

```bash
roslaunch xd_uav_track video_tracking_replay.launch \
  video_a:=/home/promise/mrs_test/data/video_replay/uav1_1min.mp4 \
  video_b:=/home/promise/mrs_test/data/video_replay/uav2_1min.mp4 \
  fps:=5 frame_step:=6 source_fps:=30 lockstep:=true max_frames:=20 \
  record_path:=/home/promise/mrs_test/logs/video_uav_pair_lockstep_20f.jsonl
```

The sample's CSV identities come from the authors' *own tracking pipeline*;
they are **not independent manually verified ground truth**. Use these
videos for visual/temporal and synchronization tests, not a certified IDF1
or ID-switch claim.

The [GRAM-RTM M-30 dataset](https://gram.web.uah.es/data/datasets/rtm/index.html)
is a real 800×480, 30 FPS traffic sequence with 7,520 frames, car/truck/van
annotations and unique vehicle IDs. Its images and annotations are meant for
**testing only**, not training or parameter tuning. The official server
currently returned HTTP 403 from this workstation, so it has **not** been
downloaded or evaluated here. Its XML annotations need conversion to MOT CSV
before using the small evaluator above. Another formal vehicle benchmark is
the [KITTI tracking dataset](https://www.cvlibs.net/datasets/kitti/eval_tracking.php),
but its full image archive is too large for this lightweight first pass.

Two arbitrary videos, or the same recording copied into both inputs, verify
topic/timestamp plumbing only. They do **not** prove cross-camera identity.
The current global-ID fusion requires trustworthy common-world position,
camera calibration and vehicle pose. Generic uncalibrated traffic videos have
none of these, so distinct per-camera IDs are the safe expected result. For a
real cross-camera acceptance test, use overlapping calibrated cameras with
shared ground-truth entity IDs and metric projection/pose inputs, or extend
the identity contract to a validated appearance-only cross-view mode. Do not
interpret matching numeric local IDs from independent sources as a merge.
