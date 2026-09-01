#!/usr/bin/env python3
import threading

import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

from gm_control.msg import BoundingBox2D


def _create_tracker(name):
    name = name.lower()
    factories = {
        "csrt": getattr(cv2, "TrackerCSRT_create", None),
        "kcf": getattr(cv2, "TrackerKCF_create", None),
        "mil": getattr(cv2, "TrackerMIL_create", None),
    }

    if name == "auto":
        for key in ("csrt", "kcf", "mil"):
            if factories[key] is not None:
                return factories[key](), key
        return None, "template"

    factory = factories.get(name)
    if factory is None:
        return None, "template"
    return factory(), name


def _clamp_box(x, y, w, h, image_width, image_height):
    x = max(0.0, min(float(x), float(image_width - 1)))
    y = max(0.0, min(float(y), float(image_height - 1)))
    w = max(1.0, min(float(w), float(image_width) - x))
    h = max(1.0, min(float(h), float(image_height) - y))
    return x, y, w, h


class BBoxTrackerNode:
    def __init__(self):
        self.lock = threading.RLock()
        self.bridge = CvBridge()

        self.tracker_type = rospy.get_param("~tracker/tracker_type", rospy.get_param("~tracker_type", "feature_tracker"))
        self.algorithm = rospy.get_param(
            "~tracker/feature_tracker/tracker_algorithm",
            rospy.get_param("~tracker_algorithm", "csrt"),
        )
        self.min_confidence = rospy.get_param("~min_confidence", 0.2)
        self.template_threshold = rospy.get_param(
            "~tracker/feature_tracker/template_match_threshold",
            rospy.get_param("~template_match_threshold", 0.45),
        )
        self.publish_debug_log = rospy.get_param("~publish_debug_log", True)
        self.safety_enabled = rospy.get_param(
            "~tracker/feature_tracker/tracker_safety/enabled",
            rospy.get_param("~tracker_safety/enabled", True),
        )
        self.max_center_jump_norm = rospy.get_param(
            "~tracker/feature_tracker/tracker_safety/max_center_jump_norm",
            rospy.get_param("~tracker_safety/max_center_jump_norm", 0.35),
        )
        self.min_area_ratio = rospy.get_param(
            "~tracker/feature_tracker/tracker_safety/min_area_ratio",
            rospy.get_param("~tracker_safety/min_area_ratio", 0.25),
        )
        self.max_area_ratio = rospy.get_param(
            "~tracker/feature_tracker/tracker_safety/max_area_ratio",
            rospy.get_param("~tracker_safety/max_area_ratio", 4.0),
        )
        self.edge_margin_px = rospy.get_param(
            "~tracker/feature_tracker/tracker_safety/edge_margin_px",
            rospy.get_param("~tracker_safety/edge_margin_px", 3.0),
        )
        self.max_fail_frames = rospy.get_param(
            "~tracker/feature_tracker/tracker_safety/max_fail_frames",
            rospy.get_param("~tracker_safety/max_fail_frames", 3),
        )

        self.color_name = rospy.get_param("~tracker/color_tracker/color", "red").lower()
        self.color_min_area_px = rospy.get_param("~tracker/color_tracker/min_area_px", 80.0)
        self.color_morph_kernel = int(rospy.get_param("~tracker/color_tracker/morph_kernel", 5))
        self.color_min_width_px = rospy.get_param("~tracker/color_tracker/min_width_px", 4.0)
        self.color_min_height_px = rospy.get_param("~tracker/color_tracker/min_height_px", 4.0)

        image_topic = rospy.get_param("~image_topic", "/camera/image_raw")
        initial_bbox_topic = rospy.get_param("~initial_bbox_topic", "/gm_control/initial_bbox")
        output_bbox_topic = rospy.get_param("~output_bbox_topic", "/gm_control/target_bbox")

        self.tracker = None
        self.tracker_name = None
        self.pending_init_box = None
        self.active = False
        self.template = None
        self.last_bbox = None
        self.fail_frames = 0
        self.frame_count = 0

        self.bbox_pub = rospy.Publisher(output_bbox_topic, BoundingBox2D, queue_size=5)
        self.image_sub = rospy.Subscriber(image_topic, Image, self._image_callback, queue_size=1)
        self.init_sub = rospy.Subscriber(initial_bbox_topic, BoundingBox2D, self._initial_bbox_callback, queue_size=1)

        rospy.loginfo(
            "bbox tracker started: image=%s initial=%s output=%s type=%s algorithm=%s color=%s",
            image_topic,
            initial_bbox_topic,
            output_bbox_topic,
            self.tracker_type,
            self.algorithm,
            self.color_name,
        )

    def _initial_bbox_callback(self, msg):
        if not msg.valid:
            with self.lock:
                self.active = False
                self.pending_init_box = None
                self.tracker = None
                self.template = None
                self.last_bbox = None
                self.fail_frames = 0
            rospy.loginfo("bbox tracker deactivated by invalid initial bbox")
            return

        with self.lock:
            self.pending_init_box = msg

    def _image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
            rospy.logwarn_throttle(2.0, "failed to convert image for bbox tracking: %s", exc)
            return

        if self.tracker_type == "color_tracker":
            bbox, confidence, valid = self._update_color_tracker(frame)
        else:
            with self.lock:
                pending = self.pending_init_box
                self.pending_init_box = None

            if pending is not None:
                self._initialize_tracker(frame, pending, msg.header)

            bbox, confidence, valid = self._update_tracker(frame)

        self._publish_bbox(msg.header, bbox, confidence, valid)

    def _update_color_tracker(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = self._make_color_mask(hsv)

        kernel_size = max(1, int(self.color_morph_kernel))
        if kernel_size > 1:
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours_info = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = contours_info[0] if len(contours_info) == 2 else contours_info[1]
        if not contours:
            return None, 0.0, False

        best = None
        best_area = 0.0
        for contour in contours:
            area = float(cv2.contourArea(contour))
            if area < self.color_min_area_px:
                continue
            x, y, w, h = cv2.boundingRect(contour)
            if w < self.color_min_width_px or h < self.color_min_height_px:
                continue
            if area > best_area:
                best_area = area
                best = (float(x), float(y), float(w), float(h))

        if best is None:
            return None, 0.0, False

        image_area = float(frame.shape[0] * frame.shape[1])
        confidence = min(1.0, best_area / max(1.0, image_area * 0.02))
        return best, confidence, True

    def _make_color_mask(self, hsv):
        if self.color_name == "red":
            lower1 = (0, 80, 50)
            upper1 = (12, 255, 255)
            lower2 = (168, 80, 50)
            upper2 = (180, 255, 255)
            return cv2.bitwise_or(cv2.inRange(hsv, lower1, upper1), cv2.inRange(hsv, lower2, upper2))
        if self.color_name == "green":
            return cv2.inRange(hsv, (35, 60, 40), (90, 255, 255))
        if self.color_name == "blue":
            return cv2.inRange(hsv, (90, 60, 40), (130, 255, 255))
        if self.color_name == "white":
            return cv2.inRange(hsv, (0, 0, 180), (180, 60, 255))
        if self.color_name == "black":
            return cv2.inRange(hsv, (0, 0, 0), (180, 255, 60))

        rospy.logwarn_throttle(2.0, "unknown color_tracker color '%s', falling back to red", self.color_name)
        return cv2.bitwise_or(
            cv2.inRange(hsv, (0, 80, 50), (12, 255, 255)),
            cv2.inRange(hsv, (168, 80, 50), (180, 255, 255)),
        )

    def _initialize_tracker(self, frame, bbox_msg, header):
        height, width = frame.shape[:2]
        x, y, w, h = _clamp_box(bbox_msg.x, bbox_msg.y, bbox_msg.width, bbox_msg.height, width, height)

        tracker, tracker_name = _create_tracker(self.algorithm)
        template = None
        active = False

        if tracker is not None:
            try:
                active = bool(tracker.init(frame, (x, y, w, h)))
            except Exception as exc:
                rospy.logwarn("OpenCV tracker init failed, falling back to template tracker: %s", exc)
                tracker = None
                tracker_name = "template"

        if tracker is None:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            x_i, y_i, w_i, h_i = int(x), int(y), int(w), int(h)
            template = gray[y_i : y_i + h_i, x_i : x_i + w_i].copy()
            active = template.size > 0
            tracker_name = "template"

        with self.lock:
            self.tracker = tracker
            self.tracker_name = tracker_name
            self.template = template
            self.last_bbox = (x, y, w, h)
            self.fail_frames = 0
            self.active = active

        rospy.loginfo(
            "bbox tracker initialized: algorithm=%s bbox=(%.1f, %.1f, %.1f, %.1f)",
            tracker_name,
            x,
            y,
            w,
            h,
        )

    def _update_tracker(self, frame):
        with self.lock:
            active = self.active
            tracker = self.tracker
            tracker_name = self.tracker_name
            template = self.template
            last_bbox = self.last_bbox

        if not active or last_bbox is None:
            return None, 0.0, False

        if tracker_name == "template":
            bbox, confidence, valid = self._update_template_tracker(frame, template, last_bbox)
        else:
            try:
                ok, bbox = tracker.update(frame)
            except Exception as exc:
                rospy.logwarn_throttle(2.0, "OpenCV tracker update failed: %s", exc)
                ok, bbox = False, last_bbox
            confidence = 1.0 if ok else 0.0
            valid = bool(ok)

        if valid:
            height, width = frame.shape[:2]
            bbox = _clamp_box(bbox[0], bbox[1], bbox[2], bbox[3], width, height)
            valid, reason = self._validate_bbox(bbox, last_bbox, width, height)
            if not valid:
                confidence = 0.0
                rospy.logwarn_throttle(1.0, "bbox tracker safety rejected target: %s", reason)

        with self.lock:
            if valid:
                self.last_bbox = bbox
                self.fail_frames = 0
                self.active = True
            else:
                self.fail_frames += 1
                if self.fail_frames >= self.max_fail_frames:
                    self.active = False
                    self.tracker = None
                    self.template = None
                    rospy.logwarn(
                        "bbox tracker stopped after %d consecutive invalid frames; publish /gm_control/initial_bbox again",
                        self.fail_frames,
                    )

        return bbox, confidence, valid

    def _validate_bbox(self, bbox, previous_bbox, image_width, image_height):
        if not self.safety_enabled or previous_bbox is None:
            return True, "ok"

        x, y, w, h = bbox
        px, py, pw, ph = previous_bbox

        if self.edge_margin_px > 0.0:
            margin = float(self.edge_margin_px)
            if (
                x <= margin
                or y <= margin
                or x + w >= image_width - margin
                or y + h >= image_height - margin
            ):
                return False, "bbox_too_close_to_image_edge"

        prev_area = max(1.0, pw * ph)
        area_ratio = (w * h) / prev_area
        if area_ratio < self.min_area_ratio:
            return False, "bbox_area_too_small ratio={:.2f}".format(area_ratio)
        if area_ratio > self.max_area_ratio:
            return False, "bbox_area_too_large ratio={:.2f}".format(area_ratio)

        cx = x + w * 0.5
        cy = y + h * 0.5
        pcx = px + pw * 0.5
        pcy = py + ph * 0.5
        dx = (cx - pcx) / max(1.0, float(image_width))
        dy = (cy - pcy) / max(1.0, float(image_height))
        center_jump = (dx * dx + dy * dy) ** 0.5
        if center_jump > self.max_center_jump_norm:
            return False, "bbox_center_jump_too_large jump={:.2f}".format(center_jump)

        return True, "ok"

    def _update_template_tracker(self, frame, template, last_bbox):
        if template is None or template.size == 0:
            return last_bbox, 0.0, False

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        th, tw = template.shape[:2]
        if gray.shape[0] < th or gray.shape[1] < tw:
            return last_bbox, 0.0, False

        result = cv2.matchTemplate(gray, template, cv2.TM_CCOEFF_NORMED)
        _, max_val, _, max_loc = cv2.minMaxLoc(result)
        x, y = max_loc
        bbox = (float(x), float(y), float(tw), float(th))
        return bbox, float(max_val), max_val >= self.template_threshold

    def _publish_bbox(self, header, bbox, confidence, valid):
        msg = BoundingBox2D()
        msg.header = header
        msg.valid = bool(valid and confidence >= self.min_confidence)
        msg.confidence = float(confidence)
        msg.target_id = self.color_name if self.tracker_type == "color_tracker" else (self.tracker_name or "")

        if bbox is not None:
            msg.x = float(bbox[0])
            msg.y = float(bbox[1])
            msg.width = float(bbox[2])
            msg.height = float(bbox[3])

        self.bbox_pub.publish(msg)


def main():
    rospy.init_node("bbox_tracker")
    BBoxTrackerNode()
    rospy.spin()


if __name__ == "__main__":
    main()
