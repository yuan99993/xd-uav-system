#!/usr/bin/env python3
import math
import time
import unittest

import rospy
import rostest
from std_srvs.srv import SetBool

from follower.msg import FollowerCommand
from xd_uav_controller.msg import ControlCommand


class AttitudeAdapterTest(unittest.TestCase):
    def setUp(self):
        self.mc_selected = None
        self.fw_selected = None
        self.mc_follower_pub = rospy.Publisher(
            '/adapter_test/mc/follower', FollowerCommand, queue_size=10)
        self.fw_follower_pub = rospy.Publisher(
            '/adapter_test/fw/follower', FollowerCommand, queue_size=10)
        self.mc_base_pub = rospy.Publisher(
            '/adapter_test/mc/base', ControlCommand, queue_size=10)
        rospy.Subscriber('/adapter_test/mc/selected', ControlCommand,
                         lambda msg: setattr(self, 'mc_selected', msg))
        rospy.Subscriber('/adapter_test/fw/selected', ControlCommand,
                         lambda msg: setattr(self, 'fw_selected', msg))
        deadline = time.monotonic() + 5.0
        while (self.mc_follower_pub.get_num_connections() == 0 or
               self.fw_follower_pub.get_num_connections() == 0) and \
                time.monotonic() < deadline:
            time.sleep(0.02)
        rospy.wait_for_service('/mc_adapter_test/enable_follower', timeout=5.0)
        self.enable_mc = rospy.ServiceProxy(
            '/mc_adapter_test/enable_follower', SetBool)

    @staticmethod
    def follower(profile, roll, pitch, yaw, thrust):
        message = FollowerCommand()
        message.header.stamp = rospy.Time.now()
        message.control_mode = 'attitude_rate'
        message.follower_profile = profile
        message.profile_supported = True
        message.command_valid = True
        message.control_authorized = True
        message.platform_ready = True
        message.roll_rate_deg_s = roll
        message.pitch_rate_deg_s = pitch
        message.yaw_rate_deg_s = yaw
        message.thrust = thrust
        return message

    def wait_for(self, predicate, timeout=2.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return True
            time.sleep(0.02)
        return False

    def test_conversion_timeout_and_takeoff_priority(self):
        mc = self.follower('mc_attitude_rate', 30.0, -20.0, 10.0, 0.65)
        # The production default is fail-closed: follower data alone cannot
        # capture the output before the operator explicitly enables handover.
        for _ in range(5):
            self.mc_follower_pub.publish(mc)
            time.sleep(0.02)
        self.assertTrue(self.wait_for(
            lambda: self.mc_selected is not None and
                    not self.mc_selected.valid))

        response = self.enable_mc(True)
        self.assertTrue(response.success)
        # Enabling clears the cached frame; publish a fresh command afterward.
        for _ in range(5):
            mc.header.stamp = rospy.Time.now()
            self.mc_follower_pub.publish(mc)
            time.sleep(0.02)
        self.assertTrue(self.wait_for(
            lambda: self.mc_selected is not None and self.mc_selected.valid))
        self.assertEqual(self.mc_selected.vehicle_type,
                         ControlCommand.VEHICLE_MULTIROTOR)
        self.assertAlmostEqual(self.mc_selected.body_rate.x,
                               math.radians(30.0), places=4)
        self.assertAlmostEqual(self.mc_selected.body_rate.y,
                               math.radians(-20.0), places=4)
        self.assertAlmostEqual(self.mc_selected.body_rate.z,
                               math.radians(10.0), places=4)
        self.assertAlmostEqual(self.mc_selected.thrust, 0.65, places=4)

        # A fresh takeoff command must preempt even a valid follower command.
        base = ControlCommand()
        base.header.stamp = rospy.Time.now()
        base.vehicle_type = ControlCommand.VEHICLE_MULTIROTOR
        base.valid = True
        base.takeoff_active = True
        base.thrust = 0.77
        base.controller = 'xd_takeoff'
        for _ in range(5):
            self.mc_follower_pub.publish(mc)
            self.mc_base_pub.publish(base)
            time.sleep(0.02)
        self.assertTrue(self.wait_for(
            lambda: self.mc_selected.controller == 'xd_takeoff'))
        self.assertAlmostEqual(self.mc_selected.thrust, 0.77, places=4)

        fw = self.follower('fw_attitude_rate', -15.0, 5.0, -4.0, 0.30)
        for _ in range(5):
            self.fw_follower_pub.publish(fw)
            time.sleep(0.02)
        self.assertTrue(self.wait_for(
            lambda: self.fw_selected is not None and self.fw_selected.valid))
        self.assertEqual(self.fw_selected.vehicle_type,
                         ControlCommand.VEHICLE_FIXEDWING)
        self.assertAlmostEqual(self.fw_selected.body_rate.x,
                               math.radians(-15.0), places=4)

        # With no base fallback configured, stale fixed-wing commands fail closed.
        self.assertTrue(self.wait_for(
            lambda: self.fw_selected is not None and
                    not self.fw_selected.valid, timeout=1.0))

        # A mismatched profile must also fail closed.
        wrong = self.follower('mc_attitude_rate', 0.0, 0.0, 0.0, 0.5)
        for _ in range(5):
            self.fw_follower_pub.publish(wrong)
            time.sleep(0.02)
        self.assertTrue(self.wait_for(
            lambda: self.fw_selected is not None and
                    not self.fw_selected.valid))


if __name__ == '__main__':
    rospy.init_node('test_attitude_adapter')
    rostest.rosrun('follower', 'attitude_adapter', AttitudeAdapterTest)
