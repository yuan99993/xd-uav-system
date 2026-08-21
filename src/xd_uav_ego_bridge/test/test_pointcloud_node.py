#!/usr/bin/env python3

import math
import unittest

import rospy
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, Header


class PointCloudNodeTest(unittest.TestCase):
    def setUp(self):
        self._clouds = []
        self._health = []
        self._publisher = rospy.Publisher(
            "/uav1/sensor/cloud", PointCloud2, queue_size=2)
        self._cloud_sub = rospy.Subscriber(
            "/uav1/ego/cloud_world", PointCloud2, self._clouds.append)
        self._health_sub = rospy.Subscriber(
            "/uav1/ego/sensing/healthy", Bool,
            lambda message: self._health.append(bool(message.data)))

    @staticmethod
    def _wait(predicate, timeout=4.0):
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        rate = rospy.Rate(50)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            if predicate():
                return True
            rate.sleep()
        return False

    def test_transform_finite_filter_and_stale_health(self):
        self.assertTrue(self._wait(lambda: self._publisher.get_num_connections() > 0))
        header = Header(stamp=rospy.Time.now(), frame_id="uav1/os")
        message = point_cloud2.create_cloud_xyz32(
            header, [(1.0, 0.0, 0.0), (math.nan, 0.0, 0.0)])
        for _ in range(3):
            message.header.stamp = rospy.Time.now()
            self._publisher.publish(message)
            rospy.sleep(0.03)
        self.assertTrue(self._wait(lambda: self._clouds and True in self._health))
        cloud = self._clouds[-1]
        self.assertEqual(cloud.header.frame_id, "world")
        points = list(point_cloud2.read_points(
            cloud, field_names=("x", "y", "z"), skip_nans=False))
        self.assertEqual(len(points), 1)
        self.assertAlmostEqual(points[0][0], 2.0, places=4)
        self.assertAlmostEqual(points[0][1], 2.0, places=4)
        self.assertAlmostEqual(points[0][2], 3.0, places=4)
        self.assertTrue(self._wait(
            lambda: self._health and self._health[-1] is False, timeout=1.0))


if __name__ == "__main__":
    rospy.init_node("test_pointcloud_node")
    import rostest
    rostest.rosrun("xd_uav_ego_bridge", "test_pointcloud_node",
                   PointCloudNodeTest)
