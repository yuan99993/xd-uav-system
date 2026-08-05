#!/usr/bin/env python3

import threading
import time
import unittest

import rospy
from geographic_msgs.msg import GeoPointStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool


class QuarantineRecoveryTest(unittest.TestCase):

    def setUp(self):
        self._running = True
        self._z = 2.0
        self._odom_pub = rospy.Publisher(
            "/uav1/mavros/local_position/odom", Odometry, queue_size=20
        )
        self._imu_pub = rospy.Publisher(
            "/uav1/mavros/imu/data", Imu, queue_size=20
        )
        self._origin_pub = rospy.Publisher(
            "/uav1/mavros/global_position/gp_origin",
            GeoPointStamped,
            queue_size=2,
            latch=True,
        )
        self._thread = threading.Thread(target=self._publish_loop, daemon=True)
        self._thread.start()

    def tearDown(self):
        self._running = False
        self._thread.join(timeout=1.0)

    def _publish_loop(self):
        rate = rospy.Rate(40)
        while self._running and not rospy.is_shutdown():
            now = rospy.Time.now()
            origin = GeoPointStamped()
            origin.header.stamp = now
            origin.header.frame_id = "earth"
            origin.position.latitude = 47.397742
            origin.position.longitude = 8.545594
            origin.position.altitude = 535.0
            self._origin_pub.publish(origin)

            imu = Imu()
            imu.header.stamp = now
            imu.header.frame_id = "uav1/base_link"
            imu.orientation.w = 1.0
            imu.linear_acceleration.z = 9.80665
            self._imu_pub.publish(imu)

            odom = Odometry()
            odom.header.stamp = now
            odom.header.frame_id = "uav1/odom"
            odom.child_frame_id = "uav1/base_link"
            odom.pose.pose.position.z = self._z
            odom.pose.pose.orientation.w = 1.0
            for index in (0, 7, 14, 35):
                odom.pose.covariance[index] = 0.02
                odom.twist.covariance[index] = 0.02
            self._odom_pub.publish(odom)
            rate.sleep()

    def _wait_bool(self, topic, expected, timeout=7.0):
        deadline = time.time() + timeout
        while time.time() < deadline and not rospy.is_shutdown():
            try:
                if rospy.wait_for_message(topic, Bool, timeout=0.25).data == expected:
                    return
            except rospy.ROSException:
                pass
        self.fail("{} did not become {}".format(topic, expected))

    def test_stable_samples_reacquire_after_quarantine(self):
        valid_topic = "/uav1/state_estimator/state_valid"
        self._wait_bool(valid_topic, True)

        # A persistent, valid step exceeds the normal innovation gate and
        # reproduces the old quarantine/re-quarantine recovery deadlock.
        self._z = 10.0
        self._wait_bool(valid_topic, False)
        self._wait_bool(valid_topic, True, timeout=8.0)

        estimate = rospy.wait_for_message(
            "/uav1/state_estimator/main/odom", Odometry, timeout=2.0
        )
        self.assertAlmostEqual(estimate.pose.pose.position.z, 10.0, delta=0.5)

        # One isolated spike must not satisfy the consecutive recovery policy.
        self._z = 30.0
        rospy.sleep(0.03)
        self._z = 10.0
        rospy.sleep(0.5)
        self.assertTrue(rospy.wait_for_message(valid_topic, Bool, timeout=1.0).data)


if __name__ == "__main__":
    rospy.init_node("quarantine_recovery_test")
    import rostest

    rostest.rosrun(
        "xd_uav_state_estimators",
        "quarantine_recovery_test",
        QuarantineRecoveryTest,
    )
