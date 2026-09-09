#!/usr/bin/env python3

import threading
import time
import unittest

import rospy
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool


class MavrosOdomRecoveryGateTest(unittest.TestCase):
    def setUp(self):
        self._running = True
        self._z = 2.0
        self._armed = False
        self._guarded = []
        self._state_pub = rospy.Publisher("/uav1/mavros/state", State, queue_size=5)
        self._odom_pub = rospy.Publisher(
            "/uav1/mavros/local_position/odom", Odometry, queue_size=20)
        self._imu_pub = rospy.Publisher("/uav1/mavros/imu/data", Imu, queue_size=20)
        self._guarded_sub = rospy.Subscriber(
            "/uav1/integration/mavros_odom_guarded", Odometry, self._guarded.append)
        self._thread = threading.Thread(target=self._publish_loop, daemon=True)
        self._thread.start()

    def tearDown(self):
        self._running = False
        self._thread.join(timeout=1.0)

    def _publish_loop(self):
        rate = rospy.Rate(40)
        while self._running and not rospy.is_shutdown():
            now = rospy.Time.now()
            self._state_pub.publish(State(connected=True, armed=self._armed, mode="MANUAL"))
            imu = Imu()
            imu.header.stamp = now
            imu.header.frame_id = "uav1/fcu"
            imu.orientation.w = 1.0
            imu.linear_acceleration.z = 9.80665
            self._imu_pub.publish(imu)
            odom = Odometry()
            odom.header.stamp = now
            odom.header.frame_id = "uav1/mavros_origin"
            odom.child_frame_id = "uav1/fcu"
            odom.pose.pose.position.z = self._z
            odom.pose.pose.orientation.w = 1.0
            for index in (0, 7, 14, 35):
                odom.pose.covariance[index] = 0.02
                odom.twist.covariance[index] = 0.02
            self._odom_pub.publish(odom)
            rate.sleep()

    @staticmethod
    def _wait(predicate, timeout=7.0):
        deadline = time.time() + timeout
        while time.time() < deadline and not rospy.is_shutdown():
            if predicate():
                return True
            rospy.sleep(0.02)
        return False

    @staticmethod
    def _main_z():
        try:
            message = rospy.wait_for_message(
                "/uav1/state_estimator/main/odom", Odometry, timeout=0.3)
            return message.pose.pose.position.z
        except rospy.ROSException:
            return None

    def test_disarmed_reseed_and_armed_fail_closed(self):
        self.assertTrue(self._wait(lambda: abs((self._main_z() or 0.0) - 2.0) < 0.5))
        self._z = 10.0
        self.assertTrue(self._wait(lambda: abs((self._main_z() or 0.0) - 10.0) < 0.5),
                        "disarmed persistent step did not recover through reseed")
        self.assertTrue(rospy.wait_for_message(
            "/uav1/integration/mavros_odom_gate/healthy", Bool, timeout=2.0).data)

        self._armed = True
        rospy.sleep(0.2)
        self._z = 20.0
        self.assertTrue(self._wait(lambda: rospy.wait_for_message(
            "/uav1/integration/mavros_odom_gate/healthy", Bool, timeout=0.3).data is False))
        count = len(self._guarded)
        rospy.sleep(0.6)
        self.assertLessEqual(len(self._guarded) - count, 1,
                             "guard forwarded a sustained jump while armed")
        self._armed = False
        self.assertTrue(self._wait(lambda: abs((self._main_z() or 0.0) - 20.0) < 0.5),
                        "disarmed recovery after armed fail-closed did not reseed")


if __name__ == "__main__":
    rospy.init_node("test_mavros_odom_recovery_gate")
    import rostest
    rostest.rosrun("xd_uav_planning", "mavros_odom_recovery_gate",
                   MavrosOdomRecoveryGateTest)
