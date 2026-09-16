#!/usr/bin/env python3
"""Simple HSV red-box detector for one or more XD UAV camera streams."""

import argparse
import sys
from typing import Dict, Iterable, List, Optional

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

from xd_uav_track.msg import DetectionArray, DetectionCandidate


def _normalise_uav_names(raw_names: Iterable[str]) -> List[str]:
    """Return unique namespace names without leading/trailing slashes."""

    result = []
    for raw_name in raw_names:
        # A ROS parameter is often supplied as "uav1,uav2" while argparse
        # naturally produces ["uav1", "uav2"]. Accept both forms.
        for item in str(raw_name).replace(",", " ").split():
            name = item.strip().strip("/")
            if name and name not in result:
                result.append(name)
    return result


def _format_topic(template: str, uav_name: str) -> str:
    try:
        topic = str(template).format(uav=uav_name)
    except (KeyError, ValueError) as error:
        raise ValueError(f"invalid topic template '{template}': {error}") from error
    if not topic:
        raise ValueError("topic template produced an empty topic")
    return topic


def _named_topic_overrides(raw_values: Optional[Iterable[str]]) -> Dict[str, str]:
    """Parse repeatable NAME=TOPIC values used by mixed sensor fleets."""

    result: Dict[str, str] = {}
    for raw_value in raw_values or []:
        name, separator, topic = str(raw_value).partition("=")
        name = name.strip().strip("/")
        topic = topic.strip()
        if not separator or not name or not topic:
            raise ValueError(
                f"invalid --image-topic '{raw_value}'; expected NAME=TOPIC"
            )
        if name in result:
            raise ValueError(f"duplicate --image-topic override for {name}")
        result[name] = topic
    return result


def _command_line() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Detect red boxes for multiple UAV camera streams in one process.",
        formatter_class=argparse.RawTextHelpFormatter,
        epilog=(
            "Examples:\n"
            "  python3 red_box_detector.py --uavs uav1 uav2\n"
            "  python3 red_box_detector.py --uavs uav1 uav2 "
            "--image-topic uav1=/uav1/down_camera/image_raw\n"
            "  python3 red_box_detector.py --uavs uav1 uav2 uav3 --no-debug-image\n\n"
            "Default topics for each UAV:\n"
            "  /{uav}/camera/image_raw\n"
            "  /{uav}/detect/input/detections_2d\n"
            "  /{uav}/track/red_detector/debug_image"
        ),
    )
    parser.add_argument(
        "--uavs",
        nargs="+",
        metavar="NAME",
        help="UAV namespaces (default ROS param ~uavs, otherwise uav1 uav2)",
    )
    parser.add_argument(
        "--image-template",
        help="input topic template containing {uav}",
    )
    parser.add_argument(
        "--image-topic",
        action="append",
        metavar="NAME=TOPIC",
        help=(
            "per-UAV image override; repeat as needed, for example "
            "uav1=/uav1/down_camera/image_raw"
        ),
    )
    parser.add_argument(
        "--detections-template",
        help="output DetectionArray topic template containing {uav}",
    )
    parser.add_argument(
        "--debug-template",
        help="debug-image topic template containing {uav}",
    )
    parser.add_argument(
        "--no-debug-image",
        action="store_true",
        help="disable all debug-image publishers",
    )
    # Remove ROS remapping arguments while retaining ordinary argparse flags.
    return parser.parse_args(rospy.myargv(argv=sys.argv)[1:])


class MultiUavRedBoxDetector:
    """Apply one common HSV configuration independently to several UAVs."""

    def __init__(self, arguments: argparse.Namespace):
        self._bridge = CvBridge()
        configured_uavs = arguments.uavs
        if configured_uavs is None:
            configured_uavs = rospy.get_param("~uavs", ["uav1", "uav2"])
            if isinstance(configured_uavs, str):
                configured_uavs = [configured_uavs]
        self._uavs = _normalise_uav_names(configured_uavs)
        if not self._uavs:
            raise ValueError("at least one UAV name is required")

        self._image_template = arguments.image_template or rospy.get_param(
            "~image_topic_template", "/{uav}/camera/image_raw"
        )
        self._image_topic_overrides = _named_topic_overrides(arguments.image_topic)
        unknown_overrides = set(self._image_topic_overrides) - set(self._uavs)
        if unknown_overrides:
            raise ValueError(
                "--image-topic contains UAVs not listed by --uavs: "
                + ", ".join(sorted(unknown_overrides))
            )
        self._detections_template = arguments.detections_template or rospy.get_param(
            "~detections_topic_template", "/{uav}/detect/input/detections_2d"
        )
        self._debug_template = arguments.debug_template or rospy.get_param(
            "~debug_image_topic_template", "/{uav}/track/red_detector/debug_image"
        )
        self._publish_debug_image = bool(
            rospy.get_param("~publish_debug_image", True)
        ) and not arguments.no_debug_image
        self._show_window = bool(rospy.get_param("~show_window", False))

        # OpenCV HSV hue uses [0, 179]. Red straddles the two ends.
        self._hue_low_1 = self._bounded_param("~hue_low_1", 0, 0, 179)
        self._hue_high_1 = self._bounded_param("~hue_high_1", 12, 0, 179)
        self._hue_low_2 = self._bounded_param("~hue_low_2", 168, 0, 179)
        self._hue_high_2 = self._bounded_param("~hue_high_2", 179, 0, 179)
        self._saturation_min = self._bounded_param(
            "~saturation_min", 100, 0, 255
        )
        self._value_min = self._bounded_param("~value_min", 60, 0, 255)
        self._minimum_area_px = max(
            1.0, float(rospy.get_param("~minimum_area_px", 400.0))
        )
        self._minimum_width_px = max(
            1, int(rospy.get_param("~minimum_width_px", 8))
        )
        self._minimum_height_px = max(
            1, int(rospy.get_param("~minimum_height_px", 8))
        )
        self._class_id = int(rospy.get_param("~class_id", 0))
        self._maximum_targets = max(
            0, int(rospy.get_param("~maximum_targets", 0))
        )
        self._confidence = min(
            1.0, max(0.0, float(rospy.get_param("~confidence", 1.0)))
        )

        kernel_size = max(1, int(rospy.get_param("~morphology_kernel", 5)))
        if kernel_size % 2 == 0:
            kernel_size += 1
        self._kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (kernel_size, kernel_size)
        )
        self._morphology_iterations = max(
            0, int(rospy.get_param("~morphology_iterations", 1))
        )

        self._detection_publishers: Dict[str, rospy.Publisher] = {}
        self._debug_publishers: Dict[str, rospy.Publisher] = {}
        self._image_sources: Dict[str, str] = {}
        self._subscribers: List[rospy.Subscriber] = []
        self._previous_target_counts = {name: 0 for name in self._uavs}
        for uav_name in self._uavs:
            image_topic = self._image_topic_overrides.get(
                uav_name, _format_topic(self._image_template, uav_name)
            )
            self._image_sources[uav_name] = (
                "down_rgb" if "/down_camera/" in image_topic else "front_rgb"
            )
            detections_topic = _format_topic(self._detections_template, uav_name)
            debug_topic = _format_topic(self._debug_template, uav_name)
            self._detection_publishers[uav_name] = rospy.Publisher(
                detections_topic, DetectionArray, queue_size=1
            )
            if self._publish_debug_image:
                self._debug_publishers[uav_name] = rospy.Publisher(
                    debug_topic, Image, queue_size=1
                )
            self._subscribers.append(
                rospy.Subscriber(
                    image_topic,
                    Image,
                    lambda message, name=uav_name: self._image_callback(name, message),
                    queue_size=1,
                    buff_size=2 ** 24,
                    tcp_nodelay=True,
                )
            )
            rospy.loginfo(
                "[red_box_detector/%s] image=%s detections=%s debug=%s",
                uav_name,
                image_topic,
                detections_topic,
                debug_topic if self._publish_debug_image else "disabled",
            )

        rospy.on_shutdown(self._on_shutdown)
        rospy.loginfo(
            "[red_box_detector] multi-UAV detector ready: %s",
            ", ".join(self._uavs),
        )

    @staticmethod
    def _bounded_param(name: str, default: int, minimum: int, maximum: int) -> int:
        return min(maximum, max(minimum, int(rospy.get_param(name, default))))

    def _make_red_mask(self, bgr_image: np.ndarray) -> np.ndarray:
        hsv_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
        lower_red_1 = np.array(
            [self._hue_low_1, self._saturation_min, self._value_min],
            dtype=np.uint8,
        )
        upper_red_1 = np.array([self._hue_high_1, 255, 255], dtype=np.uint8)
        lower_red_2 = np.array(
            [self._hue_low_2, self._saturation_min, self._value_min],
            dtype=np.uint8,
        )
        upper_red_2 = np.array([self._hue_high_2, 255, 255], dtype=np.uint8)
        mask = cv2.bitwise_or(
            cv2.inRange(hsv_image, lower_red_1, upper_red_1),
            cv2.inRange(hsv_image, lower_red_2, upper_red_2),
        )
        if self._morphology_iterations > 0:
            mask = cv2.morphologyEx(
                mask,
                cv2.MORPH_OPEN,
                self._kernel,
                iterations=self._morphology_iterations,
            )
            mask = cv2.morphologyEx(
                mask,
                cv2.MORPH_CLOSE,
                self._kernel,
                iterations=self._morphology_iterations,
            )
        return mask

    def _valid_boxes(self, mask: np.ndarray):
        contour_result = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        contours = contour_result[-2]
        valid = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < self._minimum_area_px:
                continue
            x, y, width, height = cv2.boundingRect(contour)
            if width < self._minimum_width_px or height < self._minimum_height_px:
                continue
            valid.append((area, x, y, width, height))
        valid.sort(key=lambda item: item[0], reverse=True)
        if self._maximum_targets > 0:
            valid = valid[: self._maximum_targets]
        return [item[1:] for item in valid]

    def _publish_boxes(self, uav_name: str, image_message: Image, width, height, boxes):
        message = DetectionArray()
        message.header = image_message.header
        message.image_width = width
        message.image_height = height
        message.image_source = self._image_sources[uav_name]
        message.sensor_id = image_message.header.frame_id
        message.detector_name = "red_box_detector"
        message.model_version = "hsv_v3_multi_uav"
        candidates = []
        for x, y, box_width, box_height in boxes:
            candidate = DetectionCandidate()
            candidate.track_id = -1
            candidate.class_id = self._class_id
            candidate.track_id_is_stable = False
            candidate.bbox = [x, y, x + box_width, y + box_height]
            candidate.has_bbox = True
            candidate.confidence = self._confidence
            candidates.append(candidate)
        message.candidates = candidates
        self._detection_publishers[uav_name].publish(message)

    def _publish_debug(self, uav_name: str, image_message: Image, bgr_image, boxes):
        debug_publisher: Optional[rospy.Publisher] = self._debug_publishers.get(uav_name)
        if debug_publisher is None and not self._show_window:
            return
        debug_image = bgr_image.copy()
        for index, (x, y, width, height) in enumerate(boxes, start=1):
            cv2.rectangle(debug_image, (x, y), (x + width, y + height), (0, 255, 0), 2)
            cv2.putText(
                debug_image,
                f"{uav_name} red target {index}",
                (x, max(20, y - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )
        if debug_publisher is not None:
            try:
                debug_message = self._bridge.cv2_to_imgmsg(debug_image, encoding="bgr8")
                debug_message.header = image_message.header
                debug_publisher.publish(debug_message)
            except CvBridgeError as error:
                rospy.logwarn_throttle(
                    2.0, "[red_box_detector/%s] debug conversion failed: %s", uav_name, error
                )
        if self._show_window:
            cv2.imshow(f"red_box_detector/{uav_name}", debug_image)
            cv2.waitKey(1)

    def _image_callback(self, uav_name: str, image_message: Image) -> None:
        try:
            bgr_image = self._bridge.imgmsg_to_cv2(
                image_message, desired_encoding="bgr8"
            )
        except CvBridgeError as error:
            rospy.logerr_throttle(
                2.0, "[red_box_detector/%s] image conversion failed: %s", uav_name, error
            )
            return
        if bgr_image is None or bgr_image.size == 0:
            rospy.logwarn_throttle(
                2.0, "[red_box_detector/%s] received empty image", uav_name
            )
            return
        image_height, image_width = bgr_image.shape[:2]
        boxes = self._valid_boxes(self._make_red_mask(bgr_image))
        self._publish_boxes(uav_name, image_message, image_width, image_height, boxes)
        self._publish_debug(uav_name, image_message, bgr_image, boxes)

        target_count = len(boxes)
        if target_count != self._previous_target_counts[uav_name]:
            if target_count > 0:
                rospy.loginfo(
                    "[red_box_detector/%s] visible red targets: %d",
                    uav_name,
                    target_count,
                )
            else:
                rospy.loginfo("[red_box_detector/%s] all targets lost", uav_name)
            self._previous_target_counts[uav_name] = target_count

    def _on_shutdown(self) -> None:
        if self._show_window:
            cv2.destroyAllWindows()


def main() -> int:
    arguments = _command_line()
    rospy.init_node("red_box_detector_multi")
    try:
        MultiUavRedBoxDetector(arguments)
    except ValueError as error:
        rospy.logfatal("[red_box_detector] %s", error)
        return 2
    rospy.spin()
    return 0


if __name__ == "__main__":
    sys.exit(main())
