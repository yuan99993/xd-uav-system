#!/usr/bin/env python3

import math
import threading
import time
import unittest

import rospy
import tf2_ros
from geographic_msgs.msg import GeoPointStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool

from xd_uav_state_estimators.msg import EstimatorStatus, PositionXY
from xd_uav_state_estimators.srv import SwitchLocalizationSource


class ThreePackageIntegration(unittest.TestCase):

    def setUp(self):
        self._running = True
        self._publish_fastlio = False
        self._publish_mavros = True
        self._motion_start = rospy.Time.now()
        self._publishers = {
            "mavros": rospy.Publisher(
                "/uav1/mavros/local_position/odom", Odometry, queue_size=10
            ),
            "fastlio": rospy.Publisher(
                "/uav1/fastlio/Odometry", Odometry, queue_size=10
            ),
            "imu": rospy.Publisher("/uav1/mavros/imu/data", Imu, queue_size=20),
            "origin": rospy.Publisher(
                "/uav1/mavros/global_position/gp_origin",
                GeoPointStamped,
                queue_size=2,
                latch=True,
            ),
        }
        self._thread = threading.Thread(target=self._publish_loop)
        self._thread.daemon = True
        self._thread.start()

    def tearDown(self):
        self._running = False
        self._thread.join(timeout=1.0)

    def _odometry(self, parent, child, x, velocity_x=0.0):
        message = Odometry()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = parent
        message.child_frame_id = child
        message.pose.pose.position.x = x
        message.pose.pose.position.z = 2.0
        message.pose.pose.orientation.w = 1.0
        message.twist.twist.linear.x = velocity_x
        for index in (0, 7, 14, 35):
            message.pose.covariance[index] = 0.02
            message.twist.covariance[index] = 0.02
        return message

    def _publish_loop(self):
        rate = rospy.Rate(30)
        while self._running and not rospy.is_shutdown():
            now = rospy.Time.now()
            elapsed = max(0.0, (now - self._motion_start).to_sec())
            origin = GeoPointStamped()
            origin.header.stamp = now
            origin.header.frame_id = "earth"
            origin.position.latitude = 47.397742
            origin.position.longitude = 8.545594
            origin.position.altitude = 535.273915610798
            self._publishers["origin"].publish(origin)

            imu = Imu()
            imu.header.stamp = now
            imu.header.frame_id = "uav1/base_link"
            imu.orientation.w = 1.0
            imu.linear_acceleration.z = 9.80665
            self._publishers["imu"].publish(imu)

            if self._publish_mavros:
                self._publishers["mavros"].publish(
                    self._odometry(
                        "uav1/odom",
                        "uav1/base_link",
                        2.0 * elapsed,
                        velocity_x=2.0,
                    )
                )
            if self._publish_fastlio:
                self._publishers["fastlio"].publish(
                    self._odometry(
                        "uav1/fastlio_origin",
                        "uav1/lidar_imu_link",
                        50.0 + 2.0 * elapsed,
                        # Fast-LIO来源没有配置速度修正，用它验证切源不能清零main速度。
                        velocity_x=0.0,
                    )
                )
            rate.sleep()

    def _wait_for(self, predicate, timeout=7.0):
        deadline = time.time() + timeout
        while time.time() < deadline and not rospy.is_shutdown():
            try:
                value = predicate()
                if value:
                    return value
            except (rospy.ROSException, rospy.ServiceException):
                pass
            rospy.sleep(0.05)
        self.fail("condition not met before timeout")

    def test_estimation_switch_and_tf_ownership(self):
        correction = self._wait_for(
            lambda: rospy.wait_for_message(
                "/uav1/state_estimator_inputs/mavros/position_xy",
                PositionXY,
                timeout=0.2,
            )
        )
        self.assertEqual(correction.header.frame_id, "uav1/mavros_origin")
        self.assertEqual(correction.child_frame_id, "uav1/base_link")

        main = self._wait_for(
            lambda: rospy.wait_for_message(
                "/uav1/state_estimator/main/odom", Odometry, timeout=0.2
            )
        )
        self.assertEqual(main.header.frame_id, "uav1/odom")
        self.assertEqual(main.child_frame_id, "uav1/base_link")

        self._publish_fastlio = True
        self._wait_for(
            lambda: rospy.wait_for_message(
                "/uav1/state_estimator/sources/fastlio/valid",
                Bool,
                timeout=0.2,
            ).data
        )

        rospy.wait_for_service("/uav1/state_estimator/switch_source", timeout=5.0)
        switch = rospy.ServiceProxy(
            "/uav1/state_estimator/switch_source", SwitchLocalizationSource
        )
        before = self._wait_for(
            lambda: (
                message
                if abs(
                    (message := rospy.wait_for_message(
                        "/uav1/state_estimator/main/odom",
                        Odometry,
                        timeout=0.2,
                    )).twist.twist.linear.x
                    - 2.0
                )
                < 0.3
                else None
            )
        )
        response = switch("fastlio")
        self.assertTrue(response.success, response.message)
        self.assertEqual(response.active_source, "fastlio")
        after = rospy.wait_for_message(
            "/uav1/state_estimator/main/odom", Odometry, timeout=2.0
        )
        displacement = math.sqrt(
            (after.pose.pose.position.x - before.pose.pose.position.x) ** 2
            + (after.pose.pose.position.y - before.pose.pose.position.y) ** 2
            + (after.pose.pose.position.z - before.pose.pose.position.z) ** 2
        )
        self.assertLess(displacement, 0.5)
        velocity_jump = abs(
            after.twist.twist.linear.x - before.twist.twist.linear.x
        )
        self.assertLess(
            velocity_jump,
            0.4,
            "切换到没有速度修正的Fast-LIO后，main速度发生了跳变",
        )

        buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(buffer)
        transform = self._wait_for(
            lambda: buffer.lookup_transform(
                "world", "uav1/base_link", rospy.Time(0), rospy.Duration(0.2)
            )
        )
        self.assertEqual(transform.header.frame_id, "world")
        self.assertEqual(transform.child_frame_id, "uav1/base_link")
        self.assertAlmostEqual(transform.transform.translation.z, 2.0, delta=0.3)

        local_alignment = self._wait_for(
            lambda: buffer.lookup_transform(
                "uav1/local_origin",
                "uav1/odom",
                rospy.Time(0),
                rospy.Duration(0.2),
            )
        )
        self.assertEqual(local_alignment.child_frame_id, "uav1/odom")
        self.assertAlmostEqual(local_alignment.transform.translation.x, 0.0, delta=1e-6)
        self.assertAlmostEqual(local_alignment.transform.translation.y, 0.0, delta=1e-6)
        self.assertAlmostEqual(local_alignment.transform.translation.z, 0.0, delta=1e-6)

        mavros_origin = self._wait_for(
            lambda: buffer.lookup_transform(
                "uav1/odom",
                "uav1/mavros_origin",
                rospy.Time(0),
                rospy.Duration(0.2),
            )
        )
        self.assertEqual(mavros_origin.child_frame_id, "uav1/mavros_origin")

        mavros_estimate = self._wait_for(
            lambda: buffer.lookup_transform(
                "uav1/odom",
                "uav1/mavros_estimated_base_link",
                rospy.Time(0),
                rospy.Duration(0.2),
            )
        )
        self.assertEqual(
            mavros_estimate.child_frame_id,
            "uav1/mavros_estimated_base_link",
        )

        fastlio_origin = self._wait_for(
            lambda: buffer.lookup_transform(
                "uav1/odom",
                "uav1/fastlio_origin",
                rospy.Time(0),
                rospy.Duration(0.2),
            )
        )
        self.assertEqual(fastlio_origin.child_frame_id, "uav1/fastlio_origin")

        fastlio_estimate = self._wait_for(
            lambda: buffer.lookup_transform(
                "uav1/odom",
                "uav1/fastlio_estimated_base_link",
                rospy.Time(0),
                rospy.Duration(0.2),
            )
        )
        self.assertEqual(
            fastlio_estimate.child_frame_id,
            "uav1/fastlio_estimated_base_link",
        )

        world_odom = self._wait_for(
            lambda: rospy.wait_for_message(
                "/uav1/state_estimator/main/frames/world/odom",
                Odometry,
                timeout=0.2,
            )
        )
        self.assertEqual(world_odom.header.frame_id, "world")

        fastlio_world = self._wait_for(
            lambda: rospy.wait_for_message(
                "/uav1/state_estimator/sources/fastlio/frames/world/odom",
                Odometry,
                timeout=0.2,
            )
        )
        self.assertEqual(fastlio_world.header.frame_id, "world")

        response = switch("auto")
        self.assertTrue(response.success)
        self._publish_mavros = False
        status = self._wait_for(
            lambda: (
                message
                if (
                    (message := rospy.wait_for_message(
                        "/uav1/state_estimator/status",
                        EstimatorStatus,
                        timeout=0.2,
                    )).active_source
                    == "fastlio"
                )
                else None
            ),
            timeout=5.0,
        )
        self.assertTrue(status.state_valid)


if __name__ == "__main__":
    rospy.init_node("test_three_package_integration")
    import rostest

    rostest.rosrun(
        "xd_uav_state_estimators",
        "three_package_integration",
        ThreePackageIntegration,
    )
