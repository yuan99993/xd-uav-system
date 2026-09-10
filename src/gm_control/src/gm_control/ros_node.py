import threading
from dataclasses import replace

import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from dynamic_reconfigure.server import Server as DynamicReconfigureServer
from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import Image
from gm_control.srv import StartGimbalTracking, StartGimbalTrackingResponse

from gm_control.adapters.base import GimbalCommandData, GimbalStateData
from gm_control.cfg import GimbalPidConfig
from gm_control.controller import (
    ControllerConfig,
    ImageGimbalController,
    ImageSize,
    TargetBox,
)
from gm_control.msg import BoundingBox2D, GimbalCommand, GimbalState


def _make_config_from_ros_params() -> ControllerConfig:
    return ControllerConfig(
        control_mode=rospy.get_param("~control_mode", "rate").lower(),
        yaw_kp=rospy.get_param("~pid/yaw/p", rospy.get_param("~yaw_kp", 35.0)),
        yaw_ki=rospy.get_param("~pid/yaw/i", 0.0),
        yaw_kd=rospy.get_param("~pid/yaw/d", rospy.get_param("~yaw_kd", 0.0)),
        pitch_kp=rospy.get_param("~pid/pitch/p", rospy.get_param("~pitch_kp", 25.0)),
        pitch_ki=rospy.get_param("~pid/pitch/i", 0.0),
        pitch_kd=rospy.get_param("~pid/pitch/d", rospy.get_param("~pitch_kd", 0.0)),
        yaw_integral_limit=rospy.get_param("~pid/yaw/integral_limit", 1.0),
        pitch_integral_limit=rospy.get_param("~pid/pitch/integral_limit", 1.0),
        yaw_sign=rospy.get_param("~yaw_sign", 1.0),
        pitch_sign=rospy.get_param("~pitch_sign", 1.0),
        max_yaw_rate_deg_s=rospy.get_param("~max_yaw_rate_deg_s", 80.0),
        max_pitch_rate_deg_s=rospy.get_param("~max_pitch_rate_deg_s", 60.0),
        max_yaw_angle_deg=rospy.get_param("~max_yaw_angle_deg", 45.0),
        max_pitch_angle_deg=rospy.get_param("~max_pitch_angle_deg", 30.0),
        deadzone_x=rospy.get_param("~deadzone_x", 0.02),
        deadzone_y=rospy.get_param("~deadzone_y", 0.02),
        target_timeout_s=rospy.get_param("~target_lost/timeout_s", rospy.get_param("~target_timeout_s", 0.3)),
        target_lost_action=rospy.get_param("~target_lost/action", "stop"),
        target_lost_hold_time_s=rospy.get_param("~target_lost/hold_time_s", 0.2),
        target_lost_search_yaw_rate_deg_s=rospy.get_param("~target_lost/search_yaw_rate_deg_s", 15.0),
        target_lost_search_pitch_rate_deg_s=rospy.get_param("~target_lost/search_pitch_rate_deg_s", 0.0),
        target_lost_back_to_init_yaw_deg=rospy.get_param("~target_lost/back_to_init_yaw_deg", 0.0),
        target_lost_back_to_init_pitch_deg=rospy.get_param("~target_lost/back_to_init_pitch_deg", 0.0),
        target_lost_back_to_init_roll_deg=rospy.get_param("~target_lost/back_to_init_roll_deg", 0.0),
        min_confidence=rospy.get_param("~min_confidence", 0.0),
        smoothing_enabled=rospy.get_param("~smoothing/enabled", False),
        smoothing_alpha=rospy.get_param("~smoothing/alpha", 0.3),
        max_rate_change_deg_s2=rospy.get_param("~smoothing/max_rate_change_deg_s2", 200.0),
    )


def _dynamic_config_from_controller_config(config: ControllerConfig):
    return {
        "yaw_kp": config.yaw_kp,
        "yaw_ki": config.yaw_ki,
        "yaw_kd": config.yaw_kd,
        "yaw_integral_limit": config.yaw_integral_limit,
        "pitch_kp": config.pitch_kp,
        "pitch_ki": config.pitch_ki,
        "pitch_kd": config.pitch_kd,
        "pitch_integral_limit": config.pitch_integral_limit,
    }


class GimbalImageControllerNode:
    def __init__(self):
        self.lock = threading.RLock()
        self.image_size = None
        self.latest_image_msg = None
        self.target_box = None
        self.gimbal_state = None
        self.tracking_enabled = bool(rospy.get_param("~tracking_enabled_at_startup", False))
        self.last_stamp = rospy.Time.now()
        self.bridge = CvBridge()

        initial_config = _make_config_from_ros_params()
        self.controller = ImageGimbalController(initial_config)
        self.control_rate = rospy.get_param("~control_rate", 30.0)
        self.publish_debug_image = rospy.get_param("~publish_debug_image", True)
        self.dynamic_reconfigure_server = DynamicReconfigureServer(
            GimbalPidConfig,
            self._dynamic_reconfigure_callback,
        )
        self.dynamic_reconfigure_server.update_configuration(
            _dynamic_config_from_controller_config(initial_config)
        )

        image_topic = rospy.get_param("~image_topic", "/camera/image_raw")
        bbox_topic = rospy.get_param("~bbox_topic", "/gm_control/target_bbox")
        command_topic = rospy.get_param("~command_topic", "/gm_control/gimbal_cmd")
        state_topic = rospy.get_param("~state_topic", "/gm_control/gimbal_state")
        error_topic = rospy.get_param("~error_topic", "/gm_control/image_error")
        debug_image_topic = rospy.get_param("~debug_image_topic", "/gm_control/debug_image")

        self.command_pub = rospy.Publisher(command_topic, GimbalCommand, queue_size=10)
        self.error_pub = rospy.Publisher(error_topic, Vector3Stamped, queue_size=10)
        self.debug_image_pub = rospy.Publisher(debug_image_topic, Image, queue_size=1)

        self.image_sub = rospy.Subscriber(image_topic, Image, self._image_callback, queue_size=1)
        self.bbox_sub = rospy.Subscriber(bbox_topic, BoundingBox2D, self._bbox_callback, queue_size=5)
        self.state_sub = rospy.Subscriber(state_topic, GimbalState, self._state_callback, queue_size=5)
        start_service = rospy.get_param("~start_service", "gm_control/start_tracking")
        self.start_service = rospy.Service(
            start_service,
            StartGimbalTracking,
            self._start_tracking_callback,
        )

        rospy.loginfo("gm_control image controller started")
        rospy.loginfo(
            "image_topic=%s bbox_topic=%s command_topic=%s state_topic=%s "
            "start_service=%s tracking_enabled=%s",
            image_topic,
            bbox_topic,
            command_topic,
            state_topic,
            rospy.resolve_name(start_service),
            self.tracking_enabled,
        )

    def _start_tracking_callback(self, request):
        with self.lock:
            self.tracking_enabled = bool(request.start)
            # Do not carry integral/derivative state across a manual start.
            self.controller.reset()
            active = self.tracking_enabled

        if active:
            message = "gimbal image tracking enabled"
            rospy.loginfo(message)
        else:
            message = "gimbal image tracking disabled; holding current gimbal target"
            rospy.loginfo(message)

        return StartGimbalTrackingResponse(
            success=True,
            active=active,
            message=message,
        )

    def _dynamic_reconfigure_callback(self, config, _level):
        with self.lock:
            old_config = self.controller.config
            new_config = replace(
                old_config,
                yaw_kp=config.yaw_kp,
                yaw_ki=config.yaw_ki,
                yaw_kd=config.yaw_kd,
                yaw_integral_limit=config.yaw_integral_limit,
                pitch_kp=config.pitch_kp,
                pitch_ki=config.pitch_ki,
                pitch_kd=config.pitch_kd,
                pitch_integral_limit=config.pitch_integral_limit,
            )
            self.controller.config = new_config
            if (
                old_config.yaw_ki != new_config.yaw_ki
                or old_config.pitch_ki != new_config.pitch_ki
                or old_config.yaw_integral_limit != new_config.yaw_integral_limit
                or old_config.pitch_integral_limit != new_config.pitch_integral_limit
            ):
                self.controller.integral_x = 0.0
                self.controller.integral_y = 0.0

        return config

    def _image_callback(self, msg):
        with self.lock:
            self.image_size = ImageSize(width=msg.width, height=msg.height)
            self.latest_image_msg = msg
            self.last_stamp = msg.header.stamp if msg.header.stamp else rospy.Time.now()

    def _bbox_callback(self, msg):
        with self.lock:
            self.target_box = TargetBox(
                valid=msg.valid,
                x=msg.x,
                y=msg.y,
                width=msg.width,
                height=msg.height,
                confidence=msg.confidence,
            )
            self.last_stamp = msg.header.stamp if msg.header.stamp else rospy.Time.now()

    def _state_callback(self, msg):
        with self.lock:
            self.gimbal_state = GimbalStateData(
                valid=msg.valid,
                yaw_deg=msg.yaw_deg,
                pitch_deg=msg.pitch_deg,
                roll_deg=msg.roll_deg,
                yaw_rate_deg_s=msg.yaw_rate_deg_s,
                pitch_rate_deg_s=msg.pitch_rate_deg_s,
                roll_rate_deg_s=msg.roll_rate_deg_s,
            )

    def _to_ros_command(self, data: GimbalCommandData, stamp):
        msg = GimbalCommand()
        msg.header.stamp = stamp
        msg.header.frame_id = "gimbal"
        msg.mode = GimbalCommand.MODE_RATE if data.mode == "rate" else GimbalCommand.MODE_ANGLE
        msg.valid = data.valid
        msg.yaw_rate_deg_s = data.yaw_rate_deg_s
        msg.pitch_rate_deg_s = data.pitch_rate_deg_s
        msg.roll_rate_deg_s = data.roll_rate_deg_s
        msg.yaw_deg = data.yaw_deg
        msg.pitch_deg = data.pitch_deg
        msg.roll_deg = data.roll_deg
        return msg

    def _publish_debug_image(self, image_msg, target, command, error_x, error_y, gimbal_state):
        if not self.publish_debug_image or image_msg is None:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
            rospy.logwarn_throttle(2.0, "failed to convert debug image: %s", exc)
            return

        height, width = frame.shape[:2]
        image_center = (int(width * 0.5), int(height * 0.5))

        cv2.drawMarker(
            frame,
            image_center,
            (0, 255, 255),
            markerType=cv2.MARKER_CROSS,
            markerSize=14,
            thickness=1,
        )

        if target is not None and target.valid:
            x1 = int(target.x)
            y1 = int(target.y)
            x2 = int(target.x + target.width)
            y2 = int(target.y + target.height)
            cx, cy = target.center
            target_center = (int(cx), int(cy))

            color = (0, 255, 0) if command.valid else (0, 165, 255)
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 1)
            cv2.circle(frame, target_center, 2, (0, 0, 255), -1)
            cv2.line(frame, image_center, target_center, (255, 0, 0), 1)

            lines = [
                "err x={:.3f} y={:.3f}".format(error_x, error_y),
            ]
            if command.mode == "angle":
                lines.append("yaw={:.1f} pitch={:.1f} deg".format(command.yaw_deg, command.pitch_deg))
            else:
                lines.append(
                    "yaw={:.1f} pitch={:.1f} deg/s".format(
                        command.yaw_rate_deg_s,
                        command.pitch_rate_deg_s,
                    )
                )
            if gimbal_state is not None and gimbal_state.valid:
                lines.append("state y={:.1f} p={:.1f}".format(gimbal_state.yaw_deg, gimbal_state.pitch_deg))
        else:
            lines = ["no valid target"]

        for index, text in enumerate(lines):
            cv2.putText(
                frame,
                text,
                (6, 14 + index * 14),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.38,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )

        debug_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        debug_msg.header = image_msg.header
        self.debug_image_pub.publish(debug_msg)

    def spin(self):
        rate = rospy.Rate(self.control_rate)
        while not rospy.is_shutdown():
            with self.lock:
                target = self.target_box
                image_size = self.image_size
                image_msg = self.latest_image_msg
                gimbal_state = self.gimbal_state
                stamp = self.last_stamp
                tracking_enabled = self.tracking_enabled

            if tracking_enabled:
                command, (error_x, error_y) = self.controller.update(
                    target=target,
                    image_size=image_size,
                    gimbal_state=gimbal_state,
                    now=rospy.get_time(),
                )
            else:
                command = GimbalCommandData(
                    mode=self.controller.config.control_mode,
                    valid=False,
                )
                error_x, error_y = 0.0, 0.0

            self.command_pub.publish(self._to_ros_command(command, stamp))

            err = Vector3Stamped()
            err.header.stamp = stamp
            err.header.frame_id = "image"
            err.vector.x = error_x
            err.vector.y = error_y
            err.vector.z = 1.0 if command.valid else 0.0
            self.error_pub.publish(err)
            self._publish_debug_image(image_msg, target, command, error_x, error_y, gimbal_state)

            rate.sleep()


def main():
    rospy.init_node("gimbal_image_controller")
    node = GimbalImageControllerNode()
    node.spin()
