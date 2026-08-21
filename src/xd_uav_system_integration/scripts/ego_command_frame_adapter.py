#!/usr/bin/env python3
"""Relabel EGO commands when its hard-coded `world` means local odom."""

import copy

import rospy
from quadrotor_msgs.msg import PositionCommand


class CommandFrameAdapter:
    def __init__(self):
        self._frame_id = rospy.get_param("~frame_id")
        self._publisher = rospy.Publisher(
            rospy.get_param("~output_topic", "ego/position_command"),
            PositionCommand, queue_size=50)
        rospy.Subscriber(rospy.get_param("~input_topic", "ego/raw_position_command"),
                         PositionCommand, self._callback, queue_size=50)

    def _callback(self, message):
        adapted = copy.deepcopy(message)
        adapted.header.frame_id = self._frame_id
        self._publisher.publish(adapted)


if __name__ == "__main__":
    rospy.init_node("ego_command_frame_adapter")
    CommandFrameAdapter()
    rospy.spin()
