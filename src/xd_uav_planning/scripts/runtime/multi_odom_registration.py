#!/usr/bin/env python3
"""Publish a coherent batch of Gazebo spawn-to-local-odom registrations."""

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped


def main():
    rospy.init_node("multi_odom_registration")
    parent = rospy.get_param("~parent_frame", "world").strip("/")
    registrations = rospy.get_param("~registrations")
    transforms = []
    for item in registrations:
        child = str(item["child_frame"]).strip("/")
        xyz = item["translation_m"]
        if not parent or not child or len(xyz) != 3:
            raise rospy.ROSInitException("invalid odom registration")
        transform = TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = parent
        transform.child_frame_id = child
        transform.transform.translation.x = float(xyz[0])
        transform.transform.translation.y = float(xyz[1])
        transform.transform.translation.z = float(xyz[2])
        transform.transform.rotation.w = 1.0
        transforms.append(transform)
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    broadcaster.sendTransform(transforms)
    rospy.loginfo("published %d spawn-to-odom registrations", len(transforms))
    rospy.spin()


if __name__ == "__main__":
    main()
