#!/usr/bin/env python3
"""Publish a repeatable wall point cloud for the EGO avoidance demo."""

import rospy
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header


def samples(start, stop, step):
    count = int(round((stop - start) / step))
    return [start + index * step for index in range(count + 1)]


def main():
    rospy.init_node("deterministic_obstacle_cloud")
    frame = rospy.get_param("~frame_id", "world")
    resolution = float(rospy.get_param("~resolution", 0.10))
    wall_x = float(rospy.get_param("~wall_x", 3.0))
    half_width = float(rospy.get_param("~half_width", 1.0))
    height = float(rospy.get_param("~height", 2.2))
    thickness = float(rospy.get_param("~thickness", 0.30))
    topic = rospy.get_param("~output_topic", "/map_generator/global_cloud")
    publisher = rospy.Publisher(topic, PointCloud2, queue_size=1, latch=True)
    points = [(x, y, z)
              for x in samples(wall_x - thickness / 2.0,
                               wall_x + thickness / 2.0, resolution)
              for y in samples(-half_width, half_width, resolution)
              for z in samples(0.0, height, resolution)]
    rate = rospy.Rate(float(rospy.get_param("~rate", 2.0)))
    while not rospy.is_shutdown():
        header = Header(stamp=rospy.Time.now(), frame_id=frame)
        publisher.publish(point_cloud2.create_cloud_xyz32(header, points))
        rate.sleep()


if __name__ == "__main__":
    main()
