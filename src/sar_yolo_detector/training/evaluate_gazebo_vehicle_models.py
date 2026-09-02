#!/usr/bin/env python3
"""Measure a SmartTracker vehicle model against Gazebo-projected ground truth.

The evaluator assumes that the target models are static and that the target
mesh is known.  It projects the eight corners of the hatchback mesh into the
camera image using the current Gazebo pose, camera calibration, and TF.  The
tracked SmartTracker output is then matched to those boxes at IoU 0.50.

This is deliberately a ROS-only evaluator: the model is selected by the
SmartTracker launch file, so the same executable is used for both old and new
model runs.
"""

from __future__ import annotations

import argparse
import json
import math
import statistics
import time
from pathlib import Path
from typing import Iterable, Optional

import numpy as np
import rospy
import tf2_ros
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Point
from sensor_msgs.msg import CameraInfo
from sar_yolo_detector.msg import TrackedDetection2DArray


HATCHBACK_RAW_BOUNDS = (
    -1.07023154,
    1.07023154,
    -2.34410250,
    1.65699948,
    -0.01134110,
    1.55655264,
)


def quaternion_matrix(q) -> np.ndarray:
    x, y, z, w = q
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-12:
        return np.eye(4)
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w), 0],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w), 0],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y), 0],
            [0, 0, 0, 1],
        ],
        dtype=float,
    )


def transform_pose(pose, points: np.ndarray) -> np.ndarray:
    matrix = quaternion_matrix(
        (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
    )
    matrix[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
    homogeneous = np.concatenate([points, np.ones((len(points), 1))], axis=1)
    return (matrix @ homogeneous.T).T[:, :3]


def transform_tf(transform, points: np.ndarray) -> np.ndarray:
    matrix = quaternion_matrix(
        (
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w,
        )
    )
    matrix[:3, 3] = [
        transform.transform.translation.x,
        transform.transform.translation.y,
        transform.transform.translation.z,
    ]
    homogeneous = np.concatenate([points, np.ones((len(points), 1))], axis=1)
    return (matrix @ homogeneous.T).T[:, :3]


def hatchback_corners() -> np.ndarray:
    xmin, xmax, ymin, ymax, zmin, zmax = HATCHBACK_RAW_BOUNDS
    raw = np.array(
        [
            [x, y, z]
            for x in (xmin, xmax)
            for y in (ymin, ymax)
            for z in (zmin, zmax)
        ],
        dtype=float,
    )
    # hatchback/model.sdf rotates the mesh link by +90 degrees around Z.
    link_rotation = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]], dtype=float)
    return (link_rotation @ raw.T).T


def project_box(
    model_pose,
    world_to_camera,
    camera: CameraInfo,
    local_corners: np.ndarray,
) -> Optional[np.ndarray]:
    world_points = transform_pose(model_pose, local_corners)
    camera_points = transform_tf(world_to_camera, world_points)
    if np.any(camera_points[:, 2] <= 1e-4):
        return None
    u = camera.K[0] * camera_points[:, 0] / camera_points[:, 2] + camera.K[2]
    v = camera.K[4] * camera_points[:, 1] / camera_points[:, 2] + camera.K[5]
    x1, x2 = float(np.min(u)), float(np.max(u))
    y1, y2 = float(np.min(v)), float(np.max(v))
    x1 = max(0.0, min(float(camera.width), x1))
    y1 = max(0.0, min(float(camera.height), y1))
    x2 = max(0.0, min(float(camera.width), x2))
    y2 = max(0.0, min(float(camera.height), y2))
    if x2 <= x1 or y2 <= y1:
        return None
    return np.array([x1, y1, x2, y2], dtype=float)


def iou(left: np.ndarray, right: np.ndarray) -> float:
    x1 = max(left[0], right[0])
    y1 = max(left[1], right[1])
    x2 = min(left[2], right[2])
    y2 = min(left[3], right[3])
    intersection = max(0.0, x2 - x1) * max(0.0, y2 - y1)
    area_left = max(0.0, left[2] - left[0]) * max(0.0, left[3] - left[1])
    area_right = max(0.0, right[2] - right[0]) * max(0.0, right[3] - right[1])
    return intersection / max(area_left + area_right - intersection, 1e-9)


def average_precision(frames: list[tuple[list[np.ndarray], list[tuple[float, np.ndarray]]]]) -> float:
    total_gt = sum(len(gt) for gt, _ in frames)
    if total_gt == 0:
        return 0.0
    indexed = [
        (frame_index, score, box)
        for frame_index, (_, predictions) in enumerate(frames)
        for score, box in predictions
    ]
    indexed.sort(key=lambda item: item[1], reverse=True)
    matched = [np.zeros(len(gt), dtype=bool) for gt, _ in frames]
    tp = np.zeros(len(indexed), dtype=float)
    fp = np.zeros(len(indexed), dtype=float)
    for index, (frame_index, _, box) in enumerate(indexed):
        gt = frames[frame_index][0]
        if gt:
            overlaps = [iou(box, target) for target in gt]
            best = int(np.argmax(overlaps))
            if overlaps[best] >= 0.5 and not matched[frame_index][best]:
                matched[frame_index][best] = True
                tp[index] = 1.0
                continue
        fp[index] = 1.0
    recall = np.cumsum(tp) / max(total_gt, 1)
    precision = np.cumsum(tp) / np.maximum(np.cumsum(tp + fp), 1e-9)
    levels = np.linspace(0.0, 1.0, 101)
    return float(
        np.mean(
            [precision[recall >= level].max() if np.any(recall >= level) else 0.0 for level in levels]
        )
    )


def threshold_metrics(
    frames: list[tuple[list[np.ndarray], list[tuple[float, np.ndarray]]]], threshold: float
) -> dict[str, float]:
    matched = [np.zeros(len(gt), dtype=bool) for gt, _ in frames]
    tp = fp = 0
    for frame_index, (gt, predictions) in enumerate(frames):
        for score, box in sorted(predictions, reverse=True):
            if score < threshold:
                continue
            overlaps = [iou(box, target) for target in gt]
            if overlaps:
                best = int(np.argmax(overlaps))
                if overlaps[best] >= 0.5 and not matched[frame_index][best]:
                    matched[frame_index][best] = True
                    tp += 1
                    continue
            fp += 1
    total_gt = sum(len(gt) for gt, _ in frames)
    fn = total_gt - tp
    precision = tp / max(tp + fp, 1)
    recall = tp / max(tp + fn, 1)
    f1 = 2 * precision * recall / max(precision + recall, 1e-9)
    return {
        "precision": precision,
        "recall": recall,
        "f1": f1,
        "tp": tp,
        "fp": fp,
        "fn": fn,
    }


class GazeboEvaluator:
    def __init__(self, args: argparse.Namespace):
        self.args = args
        self.models: Optional[ModelStates] = None
        self.camera: Optional[CameraInfo] = None
        self.frames: list[tuple[list[np.ndarray], list[tuple[float, np.ndarray]]]] = []
        self.received = 0
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(20.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.local_corners = hatchback_corners()
        rospy.Subscriber(args.model_states_topic, ModelStates, self.models_callback, queue_size=1)
        rospy.Subscriber(args.camera_info_topic, CameraInfo, self.camera_callback, queue_size=1)
        rospy.Subscriber(args.detections_topic, TrackedDetection2DArray, self.detections_callback, queue_size=5)

    def models_callback(self, message: ModelStates) -> None:
        self.models = message

    def camera_callback(self, message: CameraInfo) -> None:
        self.camera = message

    def ground_truth(self) -> list[np.ndarray]:
        if self.models is None or self.camera is None:
            return []
        try:
            transform = self.tf_buffer.lookup_transform(
                self.args.camera_frame, self.args.world_frame, rospy.Time(0), rospy.Duration(0.2)
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return []
        targets = []
        for index, name in enumerate(self.models.name):
            if not name.startswith(self.args.target_prefix):
                continue
            box = project_box(self.models.pose[index], transform, self.camera, self.local_corners)
            if box is not None:
                targets.append(box)
        return targets

    def detections_callback(self, message: TrackedDetection2DArray) -> None:
        ground_truth = self.ground_truth()
        if not ground_truth:
            return
        predictions: list[tuple[float, np.ndarray]] = []
        for tracked in message.detections:
            if not tracked.detection.results:
                continue
            hypothesis = tracked.detection.results[0]
            if int(hypothesis.id) != self.args.class_id:
                continue
            bbox = tracked.detection.bbox
            box = np.array(
                [
                    bbox.center.x - bbox.size_x / 2.0,
                    bbox.center.y - bbox.size_y / 2.0,
                    bbox.center.x + bbox.size_x / 2.0,
                    bbox.center.y + bbox.size_y / 2.0,
                ],
                dtype=float,
            )
            predictions.append((float(hypothesis.score), box))
        self.frames.append((ground_truth, predictions))
        self.received += 1

    def run(self) -> dict[str, object]:
        deadline = time.monotonic() + self.args.duration
        rate = rospy.Rate(20.0)
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            rate.sleep()
        return {
            "protocol": {
                "world": self.args.world,
                "target_prefix": self.args.target_prefix,
                "target_model": "hatchback",
                "class_id": self.args.class_id,
                "class_name": self.args.class_name,
                "iou_threshold": 0.5,
                "deployment_confidence": self.args.deployment_confidence,
                "duration_sec": self.args.duration,
                "camera_frame": self.args.camera_frame,
            },
            "frames": self.received,
            "ground_truth_boxes": sum(len(gt) for gt, _ in self.frames),
            "predictions": sum(len(predictions) for _, predictions in self.frames),
            "ap50": average_precision(self.frames),
            "at_confidence": {
                str(threshold): threshold_metrics(self.frames, threshold)
                for threshold in (0.30, self.args.deployment_confidence)
            },
        }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--world", default="grass_plane.world")
    parser.add_argument("--duration", type=float, default=20.0)
    parser.add_argument("--class-id", type=int, required=True)
    parser.add_argument("--class-name", required=True)
    parser.add_argument("--deployment-confidence", type=float, default=0.60)
    parser.add_argument("--target-prefix", default="gazebo_eval_hatchback_")
    parser.add_argument("--model-states-topic", default="/gazebo/model_states")
    parser.add_argument("--camera-info-topic", default="/uav1/bluefox_optflow/camera_info")
    parser.add_argument("--detections-topic", default="/uav1/sar_yolo_detector/coco/tracked_detections")
    parser.add_argument("--camera-frame", default="uav1/bluefox_optflow_optical")
    parser.add_argument("--world-frame", default="world")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rospy.init_node("gazebo_vehicle_model_evaluator", anonymous=True)
    evaluator = GazeboEvaluator(args)
    result = evaluator.run()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
