#!/usr/bin/env python3

import math
import threading
import time
import unittest

import rospy
import tf2_ros
from geographic_msgs.msg import GeoPointStamped
from geometry_msgs.msg import AccelWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from xd_uav_state_estimators.msg import (
    EstimatorStatus,
    PositionXY,
    PositionZ,
    VelocityXY,
    YawRate,
)
from xd_uav_state_estimators.srv import SwitchLocalizationSource


class ThreePackageIntegration(unittest.TestCase):

    def setUp(self):
        self._running = True
        self._publish_fastlio = False
        self._publish_mavros = True
        self._motion_start = rospy.Time.now()
        self._odom_roll = 0.0
        self._odom_pitch = 0.0
        self._odom_yaw = 0.0
        self._body_rate = (0.0, 0.0, 0.0)
        self._linear_velocity_variance = (0.02, 0.02, 0.02)
        self._angular_velocity_variance = (0.02, 0.02, 0.02)
        self._imu_acceleration_x = 0.0
        self._mavros_position_z = 2.0
        self._fastlio_position_z = 2.0
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
        rospy.wait_for_service("/uav1/state_estimator/reset", timeout=5.0)
        reset = rospy.ServiceProxy("/uav1/state_estimator/reset", Trigger)
        response = reset()
        self.assertTrue(response.success, response.message)

    def tearDown(self):
        self._running = False
        self._thread.join(timeout=1.0)

    def _odometry(self, parent, child, x, position_z, velocity_x=0.0):
        message = Odometry()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = parent
        message.child_frame_id = child
        message.pose.pose.position.x = x
        message.pose.pose.position.z = position_z
        half_roll = 0.5 * self._odom_roll
        half_pitch = 0.5 * self._odom_pitch
        half_yaw = 0.5 * self._odom_yaw
        cr = math.cos(half_roll)
        sr = math.sin(half_roll)
        cp = math.cos(half_pitch)
        sp = math.sin(half_pitch)
        cy = math.cos(half_yaw)
        sy = math.sin(half_yaw)
        message.pose.pose.orientation.x = sr * cp * cy - cr * sp * sy
        message.pose.pose.orientation.y = cr * sp * cy + sr * cp * sy
        message.pose.pose.orientation.z = cr * cp * sy - sr * sp * cy
        message.pose.pose.orientation.w = cr * cp * cy + sr * sp * sy
        message.twist.twist.linear.x = velocity_x
        (
            message.twist.twist.angular.x,
            message.twist.twist.angular.y,
            message.twist.twist.angular.z,
        ) = self._body_rate
        for axis in range(6):
            message.pose.covariance[axis * 7] = 0.02
        for axis, variance in enumerate(self._linear_velocity_variance):
            message.twist.covariance[axis * 7] = variance
        for axis, variance in enumerate(self._angular_velocity_variance):
            message.twist.covariance[(axis + 3) * 7] = variance
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
            imu.linear_acceleration.x = self._imu_acceleration_x
            imu.linear_acceleration.z = 9.80665
            self._publishers["imu"].publish(imu)

            if self._publish_mavros:
                self._publishers["mavros"].publish(
                    self._odometry(
                        "ignored/input_origin",
                        "ignored/input_reference",
                        2.0 * elapsed,
                        self._mavros_position_z,
                        velocity_x=2.0,
                    )
                )
            if self._publish_fastlio:
                self._publishers["fastlio"].publish(
                    self._odometry(
                        "ignored/input_origin",
                        "ignored/input_reference",
                        50.0 + 2.0 * elapsed,
                        self._fastlio_position_z,
                        # Fast-LIO的Odometry同时提供其自身ESKF速度。
                        velocity_x=2.0,
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

    def test_adapter_yaw_rate_and_velocity_covariance(self):
        self._odom_roll = math.pi / 3.0
        self._odom_yaw = math.pi / 2.0
        self._body_rate = (0.0, 2.0, 1.0)
        self._linear_velocity_variance = (1.0, 4.0, 4.0)
        self._angular_velocity_variance = (0.2, 0.4, 0.1)

        expected_yaw_rate = (
            math.sin(self._odom_roll) * self._body_rate[1]
            + math.cos(self._odom_roll) * self._body_rate[2]
        )
        yaw_rate = self._wait_for(
            lambda: (
                message
                if abs(
                    (message := rospy.wait_for_message(
                        "/uav1/state_estimator_inputs/mavros/yaw_rate",
                        YawRate,
                        timeout=0.2,
                    )).yaw_rate
                    - expected_yaw_rate
                )
                < 1e-3
                else None
            )
        )
        self.assertTrue(yaw_rate.valid)
        self.assertEqual(yaw_rate.header.frame_id, "uav1/mavros_origin")
        self.assertEqual(yaw_rate.child_frame_id, "uav1/base_link")
        self.assertAlmostEqual(yaw_rate.variance, 0.325, delta=1e-3)

        velocity = self._wait_for(
            lambda: rospy.wait_for_message(
                "/uav1/state_estimator_inputs/mavros/velocity_xy",
                VelocityXY,
                timeout=0.2,
            )
        )
        # Twist原本在base_link表达，旋转到父坐标系后，90度yaw会交换X/Y方差。
        self.assertAlmostEqual(velocity.covariance[0], 4.0, delta=1e-3)
        self.assertAlmostEqual(velocity.covariance[3], 1.0, delta=1e-3)

        # Fast-LIO参考点相对base_link有0.1m杆臂，角度和角速度方差必须传播到
        # base_link的位置、线速度方差中。
        self._odom_roll = 0.0
        self._odom_yaw = 0.0
        self._linear_velocity_variance = (1.0, 1.0, 1.0)
        self._angular_velocity_variance = (2.0, 2.0, 2.0)
        self._publish_fastlio = True
        fastlio_position = self._wait_for(
            lambda: rospy.wait_for_message(
                "/uav1/state_estimator_inputs/fastlio/position_xy",
                PositionXY,
                timeout=0.2,
            )
        )
        fastlio_velocity = self._wait_for(
            lambda: rospy.wait_for_message(
                "/uav1/state_estimator_inputs/fastlio/velocity_xy",
                VelocityXY,
                timeout=0.2,
            )
        )
        fastlio_height = self._wait_for(
            lambda: rospy.wait_for_message(
                "/uav1/state_estimator_inputs/fastlio/position_z",
                PositionZ,
                timeout=0.2,
            )
        )
        self.assertEqual(
            fastlio_position.header.frame_id, "uav1/fastlio_origin"
        )
        self.assertEqual(
            fastlio_position.child_frame_id, "uav1/base_link"
        )
        self.assertAlmostEqual(fastlio_height.z, 1.9, delta=1e-3)
        self.assertAlmostEqual(
            fastlio_position.covariance[0], 0.0202, delta=1e-4
        )
        self.assertAlmostEqual(
            fastlio_position.covariance[3], 0.0202, delta=1e-4
        )
        self.assertAlmostEqual(
            fastlio_velocity.covariance[0], 1.02, delta=1e-3
        )
        self.assertAlmostEqual(
            fastlio_velocity.covariance[3], 1.02, delta=1e-3
        )

    def test_inactive_source_auto_aligns_on_connection_and_reconnect(self):
        # 主源在5m处工作时，一个刚启动的相对定位源应在首次完整接入后立即
        # 建立来源原点，不需要先把它切成主源。
        self._publish_fastlio = False
        self._mavros_position_z = 5.0
        reset = rospy.ServiceProxy("/uav1/state_estimator/reset", Trigger)
        response = reset()
        self.assertTrue(response.success, response.message)

        self._wait_for(
            lambda: (
                message
                if abs(
                    (message := rospy.wait_for_message(
                        "/uav1/state_estimator/main/odom",
                        Odometry,
                        timeout=0.2,
                    )).pose.pose.position.z
                    - 5.0
                )
                < 0.15
                else None
            )
        )
        self._wait_for(
            lambda: rospy.wait_for_message(
                "/uav1/state_estimator/sources/mavros/valid",
                Bool,
                timeout=0.2,
            ).data
        )
        rospy.wait_for_service("/uav1/state_estimator/switch_source", timeout=2.0)
        select_source = rospy.ServiceProxy(
            "/uav1/state_estimator/switch_source", SwitchLocalizationSource
        )
        response = select_source("mavros")
        self.assertTrue(response.success, response.message)

        # Fast-LIO输入描述lidar_imu_link；传感器初始z为0时，adapter输出的
        # base_link为-0.1m，所以odom->来源原点应约为5.1m。
        self._fastlio_position_z = 0.0
        self._publish_fastlio = True
        first_alignment = self._wait_for(
            lambda: (
                message
                if abs(
                    (message := rospy.wait_for_message(
                        "/uav1/state_estimator/sources/fastlio/alignment",
                        TransformStamped,
                        timeout=0.2,
                    )).transform.translation.z
                    - 5.1
                )
                < 0.15
                else None
            )
        )
        self.assertEqual(first_alignment.header.frame_id, "uav1/odom")
        self.assertEqual(first_alignment.child_frame_id, "uav1/fastlio_origin")
        self._wait_for(
            lambda: rospy.wait_for_message(
                "/uav1/state_estimator/sources/fastlio/valid",
                Bool,
                timeout=0.2,
            ).data
        )
        status = rospy.wait_for_message(
            "/uav1/state_estimator/status", EstimatorStatus, timeout=1.0
        )
        self.assertEqual(status.active_source, "mavros")

        # 同一定位会话内让来源自身产生一小段位移，再执行切源。来源原点只能由
        # 会话接入决定，不能为了让main无跳变而在切源时被重新定义。
        # 用连续运动而不是单周期0.6m阶跃推进来源位置；后者应当被NIS安全门
        # 识别为异常值，不能为了切源测试而放宽真实飞行使用的创新门限。
        for step in range(1, 31):
            self._fastlio_position_z = 0.02 * step
            rospy.sleep(0.035)
        self._wait_for(
            lambda: (
                message
                if abs(
                    (message := rospy.wait_for_message(
                        "/uav1/state_estimator/sources/fastlio/odom",
                        Odometry,
                        timeout=0.2,
                    )).pose.pose.position.z
                    - 0.5
                )
                < 0.08
                else None
            )
        )
        alignment_before_switch = rospy.wait_for_message(
            "/uav1/state_estimator/sources/fastlio/alignment",
            TransformStamped,
            timeout=1.0,
        )
        main_before_switch = rospy.wait_for_message(
            "/uav1/state_estimator/main/odom", Odometry, timeout=1.0
        )
        response = select_source("fastlio")
        self.assertTrue(response.success, response.message)
        alignment_after_switch = rospy.wait_for_message(
            "/uav1/state_estimator/sources/fastlio/alignment",
            TransformStamped,
            timeout=1.0,
        )
        self.assertAlmostEqual(
            alignment_after_switch.transform.translation.x,
            alignment_before_switch.transform.translation.x,
            delta=1e-9,
        )
        self.assertAlmostEqual(
            alignment_after_switch.transform.translation.y,
            alignment_before_switch.transform.translation.y,
            delta=1e-9,
        )
        self.assertAlmostEqual(
            alignment_after_switch.transform.translation.z,
            alignment_before_switch.transform.translation.z,
            delta=1e-9,
        )
        for component in ("x", "y", "z", "w"):
            self.assertAlmostEqual(
                getattr(alignment_after_switch.transform.rotation, component),
                getattr(alignment_before_switch.transform.rotation, component),
                delta=1e-9,
            )
        self._wait_for(
            lambda: (
                message
                if (message := rospy.wait_for_message(
                    "/uav1/state_estimator/status",
                    EstimatorStatus,
                    timeout=0.2,
                )).active_source
                == "fastlio"
                else None
            )
        )
        rospy.sleep(0.20)
        main_after_switch = rospy.wait_for_message(
            "/uav1/state_estimator/main/odom", Odometry, timeout=1.0
        )
        self.assertAlmostEqual(
            main_after_switch.pose.pose.position.z,
            main_before_switch.pose.pose.position.z,
            delta=0.05,
        )
        # 普通切源的坐标连续偏移必须在整个活动期间保留，不能在数秒内衰减
        # 后把main慢慢拖向新来源的原始高度。
        rospy.sleep(1.0)
        main_one_second_later = rospy.wait_for_message(
            "/uav1/state_estimator/main/odom", Odometry, timeout=1.0
        )
        self.assertAlmostEqual(
            main_one_second_later.pose.pose.position.z,
            main_before_switch.pose.pose.position.z,
            delta=0.08,
        )
        status_one_second_later = rospy.wait_for_message(
            "/uav1/state_estimator/status", EstimatorStatus, timeout=1.0
        )
        self.assertEqual(
            status_one_second_later.active_source,
            "fastlio",
            "健康来源之间存在差异时不应覆盖用户的手动选源",
        )
        response = select_source("mavros")
        self.assertTrue(response.success, response.message)
        self._wait_for(
            lambda: (
                message
                if (message := rospy.wait_for_message(
                    "/uav1/state_estimator/status",
                    EstimatorStatus,
                    timeout=0.2,
                )).active_source
                == "mavros"
                else None
            )
        )

        # 任意align_on_activation来源中断超过其required超时后，都应被视为
        # 一个已经结束的定位会话。重新接入必须用全新的一组必需修正重新对齐。
        self._publish_fastlio = False
        self._wait_for(
            lambda: not rospy.wait_for_message(
                "/uav1/state_estimator/sources/fastlio/valid",
                Bool,
                timeout=0.2,
            ).data
        )
        rospy.sleep(0.15)

        # 用不同的局部初值模拟同类定位节点重启后建立了新的内部原点。
        # sensor z=0.5 -> base_link z=0.4，故新对齐量应为5.0-0.4=4.6。
        self._fastlio_position_z = 0.5
        self._publish_fastlio = True
        second_alignment = self._wait_for(
            lambda: (
                message
                if abs(
                    (message := rospy.wait_for_message(
                        "/uav1/state_estimator/sources/fastlio/alignment",
                        TransformStamped,
                        timeout=0.2,
                    )).transform.translation.z
                    - 4.6
                )
                < 0.15
                else None
            )
        )
        self.assertGreater(
            abs(
                second_alignment.transform.translation.z
                - first_alignment.transform.translation.z
            ),
            0.3,
        )
        status = rospy.wait_for_message(
            "/uav1/state_estimator/status", EstimatorStatus, timeout=1.0
        )
        self.assertEqual(status.active_source, "mavros")

    def test_imu_drives_dead_reckoning(self):
        before = self._wait_for(
            lambda: rospy.wait_for_message(
                "/uav1/state_estimator/main/odom", Odometry, timeout=0.2
            )
        )
        self._publish_mavros = False
        self._imu_acceleration_x = 4.0
        rospy.sleep(0.30)
        after = rospy.wait_for_message(
            "/uav1/state_estimator/main/odom", Odometry, timeout=1.0
        )
        self.assertGreater(
            after.twist.twist.linear.x - before.twist.twist.linear.x,
            0.25,
            "定位修正停止后，IMU加速度没有继续推进main速度",
        )

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

        acceleration = self._wait_for(
            lambda: rospy.wait_for_message(
                "/uav1/state_estimator/main/acceleration",
                AccelWithCovarianceStamped,
                timeout=0.2,
            )
        )
        self.assertEqual(acceleration.header.frame_id, "uav1/odom")
        self.assertLess(
            abs((acceleration.header.stamp - main.header.stamp).to_sec()), 0.2
        )
        self.assertTrue(math.isfinite(acceleration.accel.accel.linear.x))
        self.assertTrue(math.isfinite(acceleration.accel.accel.linear.y))
        self.assertTrue(math.isfinite(acceleration.accel.accel.linear.z))
        self.assertTrue(math.isfinite(acceleration.accel.accel.angular.x))
        self.assertTrue(math.isfinite(acceleration.accel.accel.angular.y))
        self.assertTrue(math.isfinite(acceleration.accel.accel.angular.z))
        self.assertGreaterEqual(acceleration.accel.covariance[0], 0.0)
        self.assertGreaterEqual(acceleration.accel.covariance[7], 0.0)
        self.assertGreaterEqual(acceleration.accel.covariance[14], 0.0)
        self.assertGreaterEqual(acceleration.accel.covariance[21], 0.0)
        self.assertGreaterEqual(acceleration.accel.covariance[28], 0.0)
        self.assertGreaterEqual(acceleration.accel.covariance[35], 0.0)

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
