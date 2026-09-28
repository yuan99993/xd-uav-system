#!/usr/bin/env python3
"""Detect the largest red region and publish the canonical 2D detection array."""

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

from xd_uav_track.msg import DetectionArray, DetectionCandidate


class RedBoxDetector:
    """HSV red-segmentation node for the simulated front RGB camera."""

    def __init__(self):
        self._bridge = CvBridge()

        self._image_topic = rospy.get_param(
            "~image_topic", "/uav1/camera/image_raw"
        )
        self._detections_topic = rospy.get_param(
            "~detections_topic", "/uav1/detect/input/detections_2d"
        )
        self._debug_image_topic = rospy.get_param(
            "~debug_image_topic", "/uav1/track/red_detector/debug_image"
        )
        self._publish_debug_image = rospy.get_param(
            "~publish_debug_image", True
        )
        self._show_window = rospy.get_param("~show_window", False)

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

        self._detections_publisher = rospy.Publisher(
            self._detections_topic, DetectionArray, queue_size=1
        )
        self._debug_publisher = None
        if self._publish_debug_image:
            self._debug_publisher = rospy.Publisher(
                self._debug_image_topic, Image, queue_size=1
            )

        self._previous_target_count = 0
        self._image_subscriber = rospy.Subscriber(
            self._image_topic,
            Image,
            self._image_callback,
            queue_size=1,
            buff_size=2 ** 24,
            tcp_nodelay=True,
        )

        rospy.on_shutdown(self._on_shutdown)
        rospy.loginfo(
            "[red_box_detector] image=%s, detections=%s, debug=%s",
            self._image_topic,
            self._detections_topic,
            self._debug_image_topic if self._publish_debug_image else "disabled",
        )

    @staticmethod
    def _bounded_param(name, default, minimum, maximum):
        return min(maximum, max(minimum, int(rospy.get_param(name, default))))

    def _make_red_mask(self, bgr_image):
        hsv_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
        lower_red_1 = np.array(
            [self._hue_low_1, self._saturation_min, self._value_min],
            dtype=np.uint8,
        )
        upper_red_1 = np.array(
            [self._hue_high_1, 255, 255], dtype=np.uint8
        )
        lower_red_2 = np.array(
            [self._hue_low_2, self._saturation_min, self._value_min],
            dtype=np.uint8,
        )
        upper_red_2 = np.array(
            [self._hue_high_2, 255, 255], dtype=np.uint8
        )

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

    def _valid_boxes(self, mask):
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
            if (
                width < self._minimum_width_px
                or height < self._minimum_height_px
            ):
                continue
            valid.append((area, x, y, width, height))

        # Large, clear regions are published first. An optional limit protects
        # the downstream tracker from receiving excessive red-noise blobs.
        valid.sort(key=lambda item: item[0], reverse=True)
        if self._maximum_targets > 0:
            valid = valid[: self._maximum_targets]
        return [item[1:] for item in valid]

    def _publish_boxes(self, image_message, width, height, detected_boxes):
        message = DetectionArray()
        message.header = image_message.header
        message.image_width = width
        message.image_height = height
        message.image_source = "front_rgb"
        message.sensor_id = image_message.header.frame_id
        message.detector_name = "red_box_detector"
        message.model_version = "hsv_v2_multi"
        candidates = []
        for detected_box in detected_boxes:
            x, y, box_width, box_height = detected_box
            candidate = DetectionCandidate()
            # This node performs detection, not temporal tracking. Let
            # xd_uav_track associate these candidates and assign stable IDs.
            candidate.track_id = -1
            candidate.class_id = self._class_id
            candidate.track_id_is_stable = False
            candidate.bbox = [x, y, x + box_width, y + box_height]
            candidate.has_bbox = True
            candidate.confidence = self._confidence
            candidates.append(candidate)
        message.candidates = candidates

        self._detections_publisher.publish(message)

    def _publish_debug(self, image_message, bgr_image, detected_boxes):
        if self._debug_publisher is None and not self._show_window:
            return

        debug_image = bgr_image.copy()
        for index, detected_box in enumerate(detected_boxes, start=1):
            x, y, width, height = detected_box
            cv2.rectangle(
                debug_image, (x, y), (x + width, y + height), (0, 255, 0), 2
            )
            cv2.putText(
                debug_image,
                "red target {}".format(index),
                (x, max(20, y - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )

        if self._debug_publisher is not None:
            try:
                debug_message = self._bridge.cv2_to_imgmsg(
                    debug_image, encoding="bgr8"
                )
                debug_message.header = image_message.header
                self._debug_publisher.publish(debug_message)
            except CvBridgeError as error:
                rospy.logwarn_throttle(
                    2.0, "[red_box_detector] debug conversion failed: %s", error
                )

        if self._show_window:
            cv2.imshow("red_box_detector", debug_image)
            cv2.waitKey(1)

    def _image_callback(self, image_message):
        try:
            bgr_image = self._bridge.imgmsg_to_cv2(
                image_message, desired_encoding="bgr8"
            )
        except CvBridgeError as error:
            rospy.logerr_throttle(
                2.0, "[red_box_detector] image conversion failed: %s", error
            )
            return

        if bgr_image is None or bgr_image.size == 0:
            rospy.logwarn_throttle(2.0, "[red_box_detector] received empty image")
            return

        image_height, image_width = bgr_image.shape[:2]
        mask = self._make_red_mask(bgr_image)
        detected_boxes = self._valid_boxes(mask)
        self._publish_boxes(
            image_message, image_width, image_height, detected_boxes
        )
        self._publish_debug(image_message, bgr_image, detected_boxes)

        target_count = len(detected_boxes)
        if target_count != self._previous_target_count:
            if target_count > 0:
                rospy.loginfo(
                    "[red_box_detector] visible red targets: %d", target_count
                )
            else:
                rospy.loginfo("[red_box_detector] all targets lost")
            self._previous_target_count = target_count

    def _on_shutdown(self):
        if self._show_window:
            cv2.destroyAllWindows()


def main():
    rospy.init_node("red_box_detector")
    RedBoxDetector()
    rospy.spin()


if __name__ == "__main__":
    main()
