#!/usr/bin/env python3

import math
import unittest

import rospy
import rostest
import tf2_ros
from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool
from xd_uav_state_estimator.msg import EstimatorStatus
from xd_uav_state_estimator.srv import SwitchLocalizationSource


class MultiSourceTest(unittest.TestCase):

    def setUp(self):
        self.primary_publisher = rospy.Publisher(
            "/uav_test/primary_odom", Odometry, queue_size=10
        )
        self.fallback_publisher = rospy.Publisher(
            "/uav_test/fallback_odom", Odometry, queue_size=10
        )
        self.imu_publisher = rospy.Publisher(
            "/uav_test/imu_in", Imu, queue_size=50
        )
        self.main_output = None
        self.primary_output = None
        self.fallback_output = None
        self.primary_valid = False
        self.fallback_valid = False
        self.active_source = ""
        self.localization_valid = False
        self.state_valid = False
        self.estimator_state = EstimatorStatus.WAITING
        self.delayed_corrections = 0
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.main_output_subscriber = rospy.Subscriber(
            "/uav_test/state_estimator/main/odom",
            Odometry,
            self.main_output_callback,
        )
        self.primary_output_subscriber = rospy.Subscriber(
            "/uav_test/state_estimator/sources/primary/odom",
            Odometry,
            self.primary_output_callback,
        )
        self.fallback_output_subscriber = rospy.Subscriber(
            "/uav_test/state_estimator/sources/fallback/odom",
            Odometry,
            self.fallback_output_callback,
        )
        self.primary_valid_subscriber = rospy.Subscriber(
            "/uav_test/state_estimator/sources/primary/valid",
            Bool,
            self.primary_valid_callback,
        )
        self.fallback_valid_subscriber = rospy.Subscriber(
            "/uav_test/state_estimator/sources/fallback/valid",
            Bool,
            self.fallback_valid_callback,
        )
        self.diagnostics_subscriber = rospy.Subscriber(
            "/uav_test/state_estimator/diagnostics",
            DiagnosticArray,
            self.diagnostics_callback,
        )
        self.validity_subscriber = rospy.Subscriber(
            "/uav_test/state_estimator/localization_valid",
            Bool,
            self.validity_callback,
        )
        self.state_validity_subscriber = rospy.Subscriber(
            "/uav_test/state_estimator/state_valid",
            Bool,
            self.state_validity_callback,
        )
        self.status_subscriber = rospy.Subscriber(
            "/uav_test/state_estimator/status",
            EstimatorStatus,
            self.status_callback,
        )
        rospy.wait_for_service(
            "/uav_test/state_estimator/switch_source", timeout=5.0
        )
        self.switch_source = rospy.ServiceProxy(
            "/uav_test/state_estimator/switch_source",
            SwitchLocalizationSource,
        )

    def main_output_callback(self, message):
        self.main_output = message

    def primary_output_callback(self, message):
        self.primary_output = message

    def fallback_output_callback(self, message):
        self.fallback_output = message

    def primary_valid_callback(self, message):
        self.primary_valid = message.data

    def fallback_valid_callback(self, message):
        self.fallback_valid = message.data

    def diagnostics_callback(self, message):
        for status in message.status:
            if status.name != "uav_test/state_estimator":
                continue
            values = {entry.key: entry.value for entry in status.values}
            self.active_source = values.get("active_localization_source", "")
            self.delayed_corrections = int(
                values.get("delayed_corrections", "0")
            )

    def validity_callback(self, message):
        self.localization_valid = message.data

    def state_validity_callback(self, message):
        self.state_valid = message.data

    def status_callback(self, message):
        self.estimator_state = message.state

    @staticmethod
    def odometry(x_position, child_frame, parent_frame="uav_test/odom"):
        message = Odometry()
        message.header.stamp = rospy.Time.now() - rospy.Duration(0.04)
        message.header.frame_id = parent_frame
        message.child_frame_id = child_frame
        message.pose.pose.position.x = x_position
        message.pose.pose.orientation.w = 1.0
        return message

    @staticmethod
    def imu():
        message = Imu()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = "uav_test/base_link"
        message.orientation.w = 1.0
        message.linear_acceleration.z = 9.80665
        return message

    def publish_for(self, duration, publish_primary, primary_position=1.0):
        end_time = rospy.Time.now() + rospy.Duration(duration)
        rate = rospy.Rate(50)
        while not rospy.is_shutdown() and rospy.Time.now() < end_time:
            self.fallback_publisher.publish(
                self.odometry(
                    10.0, "fallback_sensor", "fallback_origin"
                )
            )
            if publish_primary:
                self.primary_publisher.publish(
                    self.odometry(primary_position, "uav_test/base_link")
                )
            self.imu_publisher.publish(self.imu())
            rate.sleep()

    def test_primary_fallback_switching(self):
        connection_deadline = rospy.Time.now() + rospy.Duration(5.0)
        rate = rospy.Rate(50)
        while not rospy.is_shutdown() and rospy.Time.now() < connection_deadline:
            if self.fallback_publisher.get_num_connections() > 0:
                break
            rate.sleep()
        self.assertGreater(self.fallback_publisher.get_num_connections(), 0)
        published_topics = {
            name for name, _ in rospy.get_published_topics()
        }
        self.assertNotIn(
            "/uav_test/state_estimator/odom", published_topics
        )
        self.assertIn(
            "/uav_test/state_estimator/main/odom", published_topics
        )

        # 主定位源尚未出现时，备用源应当能够初始化估计器。
        self.publish_for(0.8, publish_primary=False)
        self.assertEqual(self.active_source, "fallback")
        self.assertIsNotNone(self.main_output)
        self.assertIsNotNone(self.fallback_output)
        self.assertIsNone(self.primary_output)
        self.assertTrue(self.fallback_valid)
        self.assertFalse(self.primary_valid)
        self.assertEqual(self.main_output.header.frame_id, "uav_test/odom")
        self.assertEqual(
            self.main_output.child_frame_id, "uav_test/base_link"
        )
        self.assertTrue(
            math.isfinite(self.main_output.pose.pose.position.x)
        )
        self.assertGreater(self.main_output.pose.pose.position.x, 9.7)
        self.assertLess(self.main_output.pose.pose.position.x, 9.9)
        self.assertTrue(self.localization_valid)
        self.assertTrue(self.state_valid)
        self.assertTrue(
            self.tf_buffer.can_transform(
                "uav_test/odom",
                "fallback_origin",
                rospy.Time(0),
                rospy.Duration(1.0),
            )
        )

        # 主定位源恢复后，应按照角色和优先级自动接管。
        self.publish_for(0.8, publish_primary=True)
        self.assertEqual(self.active_source, "primary")
        self.assertIsNotNone(self.primary_output)
        self.assertTrue(self.primary_valid)

        # 服务指定备用源后，应保持手动选择，不被健康主源的优先级抢回。
        response = self.switch_source(source_name="fallback")
        self.assertTrue(response.success)
        self.assertFalse(response.automatic)
        self.assertEqual(response.requested_source, "fallback")
        self.publish_for(0.5, publish_primary=True)
        self.assertEqual(self.active_source, "fallback")

        # auto恢复按角色和优先级选择，健康主源应重新接管。
        response = self.switch_source(source_name="auto")
        self.assertTrue(response.success)
        self.assertTrue(response.automatic)
        self.publish_for(0.5, publish_primary=True)
        self.assertEqual(self.active_source, "primary")

        # 不存在的源必须拒绝，不能改变当前主源。
        response = self.switch_source(source_name="does_not_exist")
        self.assertFalse(response.success)
        self.publish_for(0.2, publish_primary=True)
        self.assertEqual(self.active_source, "primary")

        # 主源连续输出异常跳变时应被隔离，并自动切换到仍然可靠的备用源。
        self.publish_for(0.8, publish_primary=True, primary_position=100.0)
        self.assertEqual(self.active_source, "fallback")

        # 主源停止后继续发送备用源，隔离结束也不应选择不存在的主源。
        self.publish_for(0.8, publish_primary=False)
        self.assertEqual(self.active_source, "fallback")
        self.assertGreater(self.delayed_corrections, 0)

        # 主源隔离结束后必须经过连续稳定样本，随后才能重新接管。
        self.publish_for(1.0, publish_primary=True, primary_position=1.0)
        self.assertEqual(self.active_source, "primary")

        # 恢复后的主源再次停止时，仍应回退到备用源。
        self.publish_for(0.8, publish_primary=False)
        self.assertEqual(self.active_source, "fallback")

        # 所有定位源停止后，估计器必须明确发布定位无效状态。
        rospy.sleep(0.7)
        self.assertFalse(self.localization_valid)
        self.assertFalse(self.state_valid)
        self.assertEqual(self.estimator_state, EstimatorStatus.LOST)


if __name__ == "__main__":
    rospy.init_node("test_multi_source")
    rostest.rosrun(
        "xd_uav_state_estimator", "test_multi_source", MultiSourceTest
    )
