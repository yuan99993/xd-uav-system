#!/usr/bin/env python3
"""Publish a coherent batch of Gazebo spawn-to-local-odom registrations."""

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Bool


def main():
    rospy.init_node("multi_odom_registration")
    parent = rospy.get_param("~parent_frame", "world").strip("/")
    registrations = rospy.get_param("~registrations")
    transforms = []
    validity_publishers = []
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
        validity_topic = str(item.get("validity_topic", "")).strip()
        if validity_topic:
            validity_publishers.append(rospy.Publisher(
                validity_topic, Bool, queue_size=1, latch=True))
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    broadcaster.sendTransform(transforms)
    rospy.loginfo("published %d spawn-to-odom registrations", len(transforms))
    valid = Bool(data=True)
    for publisher in validity_publishers:
        publisher.publish(valid)
    if not validity_publishers:
        rospy.spin()
        return

    rate_hz = float(rospy.get_param("~alignment_valid_rate_hz", 10.0))
    if rate_hz <= 0.0:
        raise rospy.ROSInitException("alignment_valid_rate_hz must be positive")
    rate = rospy.Rate(rate_hz)
    while not rospy.is_shutdown():
        for publisher in validity_publishers:
            publisher.publish(valid)
        rate.sleep()


if __name__ == "__main__":
    main()
