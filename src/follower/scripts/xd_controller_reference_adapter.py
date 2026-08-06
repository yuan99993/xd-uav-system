#!/usr/bin/env python3
"""Adapt follower body-velocity output to xd_uav_controller references."""

import math
import threading
import time

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from follower.msg import FollowerCommand


class XdControllerReferenceAdapter:
    """Integrate follower output into a continuous inertial MPC reference."""

    def __init__(self):
        self.lock = threading.Lock()
        self.state = None
        self.state_time = 0.0
        self.command = None
        self.command_time = 0.0
        self.reference_position = None
        self.reference_yaw = 0.0
        self.last_publish_time = time.monotonic()

        self.publish_rate = max(2.0, rospy.get_param('~publish_rate', 30.0))
        self.state_timeout = max(0.05, rospy.get_param('~state_timeout', 0.5))
        self.command_timeout = max(0.05, rospy.get_param('~command_timeout', 0.25))
        command_topic = rospy.get_param('~command_topic', '/follower_node/follower_command')
        state_topic = rospy.get_param('~state_topic', '/uav1/state_estimator/main/odom')
        reference_topic = rospy.get_param('~reference_topic', '/uav1/control/reference/odom')

        self.reference_pub = rospy.Publisher(reference_topic, Odometry, queue_size=10)
        rospy.Subscriber(command_topic, FollowerCommand, self.command_callback, queue_size=10)
        rospy.Subscriber(state_topic, Odometry, self.state_callback, queue_size=10)
        rospy.Timer(rospy.Duration(1.0 / self.publish_rate), self.timer_callback)
        rospy.loginfo('[FollowerXdAdapter] %s -> %s', command_topic, reference_topic)

    def command_callback(self, message):
        with self.lock:
            self.command = message
            self.command_time = time.monotonic()

    def state_callback(self, message):
        with self.lock:
            self.state = message
            self.state_time = time.monotonic()

    def timer_callback(self, _event):
        now = time.monotonic()
        dt = min(0.2, max(0.0, now - self.last_publish_time))
        self.last_publish_time = now
        with self.lock:
            state = self.state
            command = self.command
            state_fresh = now - self.state_time <= self.state_timeout
            command_fresh = now - self.command_time <= self.command_timeout
        if state is None or not state_fresh or not state.header.frame_id:
            rospy.logwarn_throttle(2.0, '[FollowerXdAdapter] Waiting for fresh estimator odometry')
            return

        q = state.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        valid = command is not None and command_fresh and command.command_valid
        forward = command.velocity_forward if valid else 0.0
        right = command.velocity_right if valid else 0.0
        down = command.velocity_down if valid else 0.0
        yaw_rate = math.radians(command.yaw_rate_deg_s) if valid else 0.0

        # Follower uses body FLU: right/down map to negative ENU y/z.
        vx = math.cos(yaw) * forward + math.sin(yaw) * right
        vy = math.sin(yaw) * forward - math.cos(yaw) * right
        vz = -down
        if self.reference_position is None:
            p = state.pose.pose.position
            self.reference_position = [p.x, p.y, p.z]
            self.reference_yaw = yaw
        self.reference_position[0] += vx * dt
        self.reference_position[1] += vy * dt
        self.reference_position[2] += vz * dt
        self.reference_yaw += yaw_rate * dt

        reference = Odometry()
        reference.header.stamp = rospy.Time.now()
        reference.header.frame_id = state.header.frame_id
        reference.child_frame_id = state.header.frame_id
        reference.pose.pose.position.x = self.reference_position[0]
        reference.pose.pose.position.y = self.reference_position[1]
        reference.pose.pose.position.z = self.reference_position[2]
        rotation = quaternion_from_euler(0.0, 0.0, self.reference_yaw)
        reference.pose.pose.orientation.x = rotation[0]
        reference.pose.pose.orientation.y = rotation[1]
        reference.pose.pose.orientation.z = rotation[2]
        reference.pose.pose.orientation.w = rotation[3]
        reference.twist.twist.linear.x = vx
        reference.twist.twist.linear.y = vy
        reference.twist.twist.linear.z = vz
        reference.twist.twist.angular.z = yaw_rate
        self.reference_pub.publish(reference)


if __name__ == '__main__':
    rospy.init_node('follower_xd_controller_adapter')
    XdControllerReferenceAdapter()
    rospy.spin()
