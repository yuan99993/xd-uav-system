#!/usr/bin/env python3
"""Detect the largest red region in a ROS image and publish its bounding box."""

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

from xd_uav_track.msg import BoundingBox


class RedBoxDetector:
    """HSV red-segmentation node for the simulated front RGB camera."""

    def __init__(self):
        self._bridge = CvBridge()

        self._image_topic = rospy.get_param(
            "~image_topic", "/uav1/camera/image_raw"
        )
        self._bounding_box_topic = rospy.get_param(
            "~bounding_box_topic", "/uav1/track/bounding_box"
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
        self._track_id = int(rospy.get_param("~track_id", 1))
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

        self._box_publisher = rospy.Publisher(
            self._bounding_box_topic, BoundingBox, queue_size=1
        )
        self._debug_publisher = None
        if self._publish_debug_image:
            self._debug_publisher = rospy.Publisher(
                self._debug_image_topic, Image, queue_size=1
            )

        self._had_target = False
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
            "[red_box_detector] image=%s, bounding_box=%s, debug=%s",
            self._image_topic,
            self._bounding_box_topic,
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

    def _largest_valid_box(self, mask):
        contour_result = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        contours = contour_result[-2]
        if not contours:
            return None

        contour = max(contours, key=cv2.contourArea)
        if cv2.contourArea(contour) < self._minimum_area_px:
            return None

        x, y, width, height = cv2.boundingRect(contour)
        if width < self._minimum_width_px or height < self._minimum_height_px:
            return None
        return x, y, width, height

    def _publish_box(self, image_message, width, height, detected_box):
        message = BoundingBox()
        message.header = image_message.header
        message.image_width = width
        message.image_height = height
        message.track_id = self._track_id
        message.confidence = self._confidence

        if detected_box is None:
            message.valid = False
        else:
            x, y, box_width, box_height = detected_box
            message.x_min = float(x)
            message.y_min = float(y)
            message.x_max = float(x + box_width)
            message.y_max = float(y + box_height)
            message.valid = True

        self._box_publisher.publish(message)

    def _publish_debug(self, image_message, bgr_image, detected_box):
        if self._debug_publisher is None and not self._show_window:
            return

        debug_image = bgr_image.copy()
        if detected_box is not None:
            x, y, width, height = detected_box
            cv2.rectangle(
                debug_image, (x, y), (x + width, y + height), (0, 255, 0), 2
            )
            cv2.putText(
                debug_image,
                "red target",
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
        detected_box = self._largest_valid_box(mask)
        self._publish_box(
            image_message, image_width, image_height, detected_box
        )
        self._publish_debug(image_message, bgr_image, detected_box)

        has_target = detected_box is not None
        if has_target != self._had_target:
            if has_target:
                x, y, width, height = detected_box
                rospy.loginfo(
                    "[red_box_detector] target acquired: x=%d y=%d w=%d h=%d",
                    x,
                    y,
                    width,
                    height,
                )
            else:
                rospy.loginfo("[red_box_detector] target lost")
            self._had_target = has_target

    def _on_shutdown(self):
        if self._show_window:
            cv2.destroyAllWindows()


def main():
    rospy.init_node("red_box_detector")
    RedBoxDetector()
    rospy.spin()


if __name__ == "__main__":
    main()
