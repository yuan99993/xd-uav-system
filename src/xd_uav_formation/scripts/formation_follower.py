#!/usr/bin/env python3
"""Standalone compatibility entry point for one formation follower."""

import rospy

from xd_uav_formation.runtime import FormationFollower


if __name__ == "__main__":
    rospy.init_node("formation_follower")
    FormationFollower()
    rospy.spin()
