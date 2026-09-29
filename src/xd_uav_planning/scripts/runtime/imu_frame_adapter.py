#!/usr/bin/env python3
"""Adapt only the IMU frame label for a physically coincident body frame."""

import copy

import rospy
from sensor_msgs.msg import Imu


class ImuFrameAdapter:
    def __init__(self):
        self._frame_id = rospy.get_param("~frame_id")
        self._publisher = rospy.Publisher(
            rospy.get_param("~output_topic", "integration/imu_base_link"),
            Imu, queue_size=20)
        rospy.Subscriber(rospy.get_param("~input_topic", "mavros/imu/data"),
                         Imu, self._callback, queue_size=20)

    def _callback(self, message):
        if rospy.is_shutdown():
            return
        adapted = copy.deepcopy(message)
        adapted.header.frame_id = self._frame_id
        try:
            self._publisher.publish(adapted)
        except rospy.ROSException:
            if not rospy.is_shutdown():
                raise


if __name__ == "__main__":
    rospy.init_node("imu_frame_adapter")
    ImuFrameAdapter()
    rospy.spin()
