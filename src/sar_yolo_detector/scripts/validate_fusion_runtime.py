#!/usr/bin/env python3
"""Bounded real-model ROS plumbing probe. Not an accuracy/UE benchmark."""
import argparse
from collections import Counter
import json
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import CameraInfo, Image
from sar_yolo_detector.msg import SmartTrackerState, TrackedDetection2DArray, FusionDetection2DArray
from sar_yolo_detector.srv import SelectSmartTrack, SelectSmartTrackRequest, SwitchTrackingMode
from std_msgs.msg import String
from xd_uav_track.msg import DetectionArray


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--image", required=True)
    parser.add_argument("--namespace", default="fusion_probe")
    parser.add_argument("--duration", type=float, default=8.)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    if not 1 <= args.duration <= 60:
        parser.error("duration must be between 1 and 60 seconds")
    frame = cv2.imread(args.image)
    if frame is None:
        parser.error("image cannot be decoded")
    rospy.init_node("fusion_runtime_probe", anonymous=True)
    prefix = "/" + args.namespace.strip("/")
    node = prefix + "/sar_yolo_smart_tracker_smart"
    lock = threading.Lock()
    detections, states, evidence, bridge, selected_stream, typed, annotated = [], [], [], [], [], [], []

    def receive(destination):
        def callback(message):
            with lock:
                destination.append(message)
                if len(destination) > 500:
                    del destination[:100]
        return callback

    rospy.Subscriber(prefix + "/sar_yolo_detector/smart/tracked_detections", TrackedDetection2DArray, receive(detections), queue_size=2)
    rospy.Subscriber(prefix + "/sar_yolo_detector/smart/state", SmartTrackerState, receive(states), queue_size=2)
    rospy.Subscriber(node + "/fusion_evidence", String, receive(evidence), queue_size=2)
    rospy.Subscriber(prefix + "/detect/input/detections_2d", DetectionArray, receive(bridge), queue_size=2)
    rospy.Subscriber(prefix + "/sar_yolo_detector/smart/selected_tracked_detections", TrackedDetection2DArray, receive(selected_stream), queue_size=2)
    rospy.Subscriber(prefix + "/sar_yolo_detector/smart/tracked_detections/fusion", FusionDetection2DArray, receive(typed), queue_size=2)
    rospy.Subscriber(prefix + "/sar_yolo_detector/smart/annotated", Image, receive(annotated), queue_size=1)
    publisher = rospy.Publisher(prefix + "/camera/image_raw", Image, queue_size=1)
    camera_publisher = rospy.Publisher(prefix + "/down_camera/camera_info", CameraInfo, queue_size=1, latch=True)
    camera_info = CameraInfo(width=frame.shape[1], height=frame.shape[0])
    camera_publisher.publish(camera_info)
    stop = threading.Event()

    def publish():
        seq = 0
        while not stop.wait(.20) and not rospy.is_shutdown():
            seq += 1
            message = Image()
            message.header.seq = seq
            message.header.stamp = rospy.Time.now()
            message.header.frame_id = "fusion_probe/camera_optical"
            message.height, message.width = frame.shape[:2]
            message.encoding = "bgr8"
            message.step = message.width * 3
            message.data = frame.tobytes()
            publisher.publish(message)

    worker = threading.Thread(target=publish, daemon=True)
    worker.start()
    report = {"kind": "real_pt_cuda_ros_plumbing_not_accuracy", "image": args.image}
    try:
        rospy.wait_for_service(node + "/switch_tracking_mode", timeout=100.)
        switch = rospy.ServiceProxy(node + "/switch_tracking_mode", SwitchTrackingMode)
        results = {}
        for mode in ("smart_tracker", "feature_tracker", "color_tracker", "fusion_tracker"):
            response = switch(mode, "")
            results[mode] = response.success and response.active_mode == mode
        report["mode_switches"] = results
        # CUDA/torchvision kernels may initialize lazily on the first frame.
        # Keep the startup allowance bounded and separate from the test window.
        deadline = time.monotonic() + 45.
        choice = None
        while time.monotonic() < deadline and not rospy.is_shutdown():
            with lock:
                if (detections and detections[-1].detections and
                        0 <= (rospy.Time.now() - detections[-1].header.stamp).to_sec() < .60):
                    candidates = [d for d in detections[-1].detections if d.track_id >= 0 and d.track_id_is_stable and d.detection.results[0].score >= .5]
                    if candidates:
                        choice = max(candidates, key=lambda d: d.detection.results[0].score)
            if choice is not None:
                break
            stop.wait(.05)
        if choice is None:
            raise RuntimeError("No stable YOLO target above the fusion identity threshold 0.50")
        select = rospy.ServiceProxy(node + "/select", SelectSmartTrack)
        response = select(SelectSmartTrackRequest(use_target_id=True, target_id=choice.track_id))
        if not response.success:
            raise RuntimeError(response.message)
        report["selected_id"] = response.selected_target_id
        start = time.monotonic()
        while time.monotonic() - start < args.duration and not rospy.is_shutdown():
            stop.wait(.05)
        with lock:
            selected = [s for s in states if s.has_selection]
            records = [json.loads(m.data) for m in evidence]
            counts = Counter(r["source"] for r in records if r["selected_id"] == response.selected_target_id)
            report.update(source_counts=dict(counts), state_messages=len(selected), bridge_messages=len(bridge),
                          identity_unchanged=bool(selected) and all(s.selected_track_id == response.selected_target_id for s in selected),
                          actual_model=selected[-1].model_path if selected else "",
                          device=selected[-1].device if selected else "",
                          tracker=selected[-1].tracker_type if selected else "",
                          max_processing_ms=max((s.frame_processing_ms for s in selected), default=0),
                          duplicate_selected_frames=sum(sum(d.track_id == response.selected_target_id for d in m.detections) > 1 for m in detections),
                          processing_p95_ms=float(np.percentile([s.frame_processing_ms for s in selected], 95)) if selected else 0.,
                          selected_stream_valid=any(len(m.detections) == 1 and m.detections[0].track_id == response.selected_target_id for m in selected_stream),
                          actual_dimensions_valid=bool(typed) and all((m.image_width, m.image_height) == (frame.shape[1], frame.shape[0]) for m in typed),
                          annotated_dimensions_valid=bool(annotated) and all((m.width, m.height) == (frame.shape[1], frame.shape[0]) for m in annotated),
                          bridge_sources=sorted({c.measurement_source for m in bridge for c in m.candidates}))
        report["passed"] = all(results.values()) and counts["fused"] > 0 and report["identity_unchanged"] and report["duplicate_selected_frames"] == 0 and len(bridge) > 0 and report["selected_stream_valid"] and report["actual_dimensions_valid"] and report["annotated_dimensions_valid"]
    except Exception as exc:
        report.update(passed=False, error=str(exc))
        with lock:
            report.update(state_messages=len(states), detector_messages=len(detections),
                          max_detector_count=max((len(m.detections) for m in detections), default=0),
                          actual_model=states[-1].model_path if states else "",
                          device=states[-1].device if states else "")
    finally:
        stop.set()
        worker.join(timeout=2.)
        Path(args.output).write_text(json.dumps(report, indent=2), encoding="utf-8")
        print(json.dumps(report, indent=2))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
