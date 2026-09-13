#!/usr/bin/env python3
"""Verify that Path mode plans early around a lidar-visible front face."""

import threading
import time
import unittest

import rospy
import rostest
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from traj_utils.msg import Bspline


class RouteObstacleTest(unittest.TestCase):
    def setUp(self):
        self._lock = threading.Lock()
        self._bspline = None
        self._path_publish_time = None
        self._odom_pub = rospy.Publisher(
            "/drone_0_route_obstacle/odom", Odometry, queue_size=10)
        self._cloud_pub = rospy.Publisher(
            "/drone_0_route_obstacle/cloud", PointCloud2, queue_size=2)
        self._path_pub = rospy.Publisher(
            "/route_obstacle/reference_path", Path, queue_size=1, latch=True)
        rospy.Subscriber("/drone_0_planning/bspline", Bspline,
                         self._bspline_callback, queue_size=5)
        self._odom_timer = rospy.Timer(rospy.Duration(0.02), self._publish_odom)
        self._cloud_timer = rospy.Timer(rospy.Duration(0.10), self._publish_cloud)

    def tearDown(self):
        self._odom_timer.shutdown()
        self._cloud_timer.shutdown()

    def _bspline_callback(self, message):
        with self._lock:
            if self._bspline is None:
                self._bspline = message

    def _publish_odom(self, _event):
        message = Odometry()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = "world"
        message.child_frame_id = "uav1/base_link"
        message.pose.pose.position.z = 1.5
        message.pose.pose.orientation.w = 1.0
        self._odom_pub.publish(message)

    def _publish_cloud(self, _event):
        # Only a front surface is available, matching what a lidar first sees
        # when approaching a tall pillar/wall. It spans floor to ceiling so a
        # valid local solution has to pass an observed side edge.
        points = []
        for y_index in range(-6, 7):
            for z_index in range(0, 16):
                points.append((3.0, y_index * 0.2, z_index * 0.2))
        header = Header(stamp=rospy.Time.now(), frame_id="world")
        self._cloud_pub.publish(pc2.create_cloud_xyz32(header, points))

    @staticmethod
    def _path_message():
        message = Path()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = "world"
        for x in (0.0, 6.0):
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x = x
            pose.pose.position.z = 1.5
            pose.pose.orientation.w = 1.0
            message.poses.append(pose)
        return message

    def test_front_face_produces_fast_side_detour(self):
        deadline = time.monotonic() + 6.0
        while (not rospy.is_shutdown() and time.monotonic() < deadline and
               (self._path_pub.get_num_connections() == 0 or
                self._cloud_pub.get_num_connections() == 0 or
                self._odom_pub.get_num_connections() == 0)):
            rospy.sleep(0.02)
        self.assertGreater(self._path_pub.get_num_connections(), 0)
        # Let several cloud frames populate the direct point-cloud map before
        # publishing the allocator route.
        rospy.sleep(0.5)
        self._path_publish_time = time.monotonic()
        self._path_pub.publish(self._path_message())

        deadline = time.monotonic() + 5.0
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            with self._lock:
                message = self._bspline
            if message is not None:
                break
            rospy.sleep(0.01)
        self.assertIsNotNone(message, "EGO did not publish a route B-spline")
        self.assertLess(time.monotonic() - self._path_publish_time, 1.0,
                        "local obstacle planning was not responsive")
        self.assertGreaterEqual(len(message.pos_pts), 7)

        # First/last points remain tied to the requested x-axis route, while
        # at least one free control point must go around the visible side.
        maximum_lateral = max(abs(point.y) for point in message.pos_pts)
        self.assertGreater(maximum_lateral, 1.25,
                           "planned spline did not pass the obstacle side")
        self.assertAlmostEqual(message.pos_pts[-1].x, 6.0, delta=0.35)
        self.assertAlmostEqual(message.pos_pts[-1].y, 0.0, delta=0.35)


if __name__ == "__main__":
    rospy.init_node("test_ego_route_obstacle")
    rostest.rosrun("xd_uav_planning", "ego_route_obstacle",
                   RouteObstacleTest)
