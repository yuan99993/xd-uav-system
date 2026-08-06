#!/usr/bin/env python3
import unittest

import rospy
import rostest

from follower.srv import SetMode


class ProfileSwitchGuardTest(unittest.TestCase):
    def test_minimum_profile_dwell(self):
        rospy.wait_for_service('/profile_guard_test/set_profile', timeout=10.0)
        set_profile = rospy.ServiceProxy('/profile_guard_test/set_profile', SetMode)

        self.assertTrue(set_profile('mc_velocity_distance').success)
        blocked = set_profile('mc_velocity_position')
        self.assertFalse(blocked.success)
        self.assertIn('rate limited', blocked.message)

        rospy.sleep(0.60)
        self.assertTrue(set_profile('mc_velocity_position').success)


if __name__ == '__main__':
    rospy.init_node('test_profile_switch_guard')
    rostest.rosrun('follower', 'profile_switch_guard', ProfileSwitchGuardTest)
