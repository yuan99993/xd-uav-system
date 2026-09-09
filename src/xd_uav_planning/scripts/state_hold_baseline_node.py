#!/usr/bin/env python3
"""Publish a non-canonical hold reference used only for owner switch checks."""

import math

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import PositionTarget
import tf2_ros
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose
from xd_uav_controller.msg import ControlState


class StateHoldBaseline:
    def __init__(self):
        self._timeout = float(rospy.get_param("~state_timeout", 0.20))
        self._frame = rospy.get_param("~common_frame", "world").strip("/")
        self._last_state = None
        self._buffer = tf2_ros.Buffer(cache_time=rospy.Duration(5.0))
        self._listener = tf2_ros.TransformListener(self._buffer)
        self._subscriber = rospy.Subscriber(
            rospy.get_param("~state_topic", "control_manager/state"),
            ControlState, self._callback, queue_size=10)
        self._publisher = rospy.Publisher(
            rospy.get_param("~output_topic", "integration/switch_baseline"),
            PositionTarget, queue_size=1)
        self._timer = rospy.Timer(rospy.Duration(0.05), self._timer_callback)

    @staticmethod
    def _finite(values):
        return all(math.isfinite(value) for value in values)

    def _callback(self, message):
        values = (message.position_odom.x, message.position_odom.y,
                  message.position_odom.z, message.velocity_odom.x,
                  message.velocity_odom.y, message.velocity_odom.z)
        if (not message.state_valid or not message.localization_valid or
                not message.odometry_fresh or
                message.header.stamp.to_sec() <= 0.0 or
                not self._finite(values)):
            self._last_state = None
            return
        position = message.position_odom
        source_frame = message.header.frame_id.strip("/")
        if source_frame != self._frame:
            try:
                transform = self._buffer.lookup_transform(
                    self._frame, message.header.frame_id,
                    message.header.stamp, rospy.Duration(0.05))
                pose = PoseStamped()
                pose.header = message.header
                pose.pose.position = message.position_odom
                pose.pose.orientation = message.orientation_odom_body
                position = do_transform_pose(pose, transform).pose.position
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException):
                self._last_state = None
                return
        self._last_state = (message.header.stamp, position)

    def _timer_callback(self, _event):
        state = self._last_state
        if state is None:
            return
        stamp, position = state
        age = (rospy.Time.now() - stamp).to_sec()
        if age < -0.02 or age > self._timeout:
            return
        baseline = PositionTarget()
        baseline.header.stamp = rospy.Time.now()
        baseline.header.frame_id = self._frame
        baseline.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        # Match the controller's idle reference contract: hold the measured
        # position with zero velocity/acceleration, while leaving heading free.
        baseline.type_mask = (PositionTarget.IGNORE_YAW |
                              PositionTarget.IGNORE_YAW_RATE)
        baseline.position = position
        self._publisher.publish(baseline)


def main():
    rospy.init_node("state_hold_baseline")
    StateHoldBaseline()
    rospy.spin()


if __name__ == "__main__":
    main()
