#!/usr/bin/env python3
"""Wait until the control manager can safely accept a takeoff request.

The tmux test sessions start PX4, the estimator and the control manager in
parallel.  ``waitForTime`` only waits for ``/use_sim_time``; it does not wait
for the estimator's first valid, fresh fixed-wing state.  Calling the takeoff
service during that short startup window is therefore rejected by design.
"""

import argparse
import sys
import time

import rospy

from xd_uav_controller.msg import ControlState


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--uav", default="uav1")
    parser.add_argument("--timeout", type=float, default=120.0)
    parser.add_argument("--poll", type=float, default=0.5)
    return parser.parse_args(rospy.myargv()[1:])


def main():
    rospy.init_node("wait_for_uav_control_ready", anonymous=True)
    args = parse_args()
    topic = "/{}/control_manager/state".format(args.uav.strip("/"))
    # Use wall time for the watchdog.  A paused/reset simulation clock must
    # not make this helper wait forever during startup.
    deadline = time.monotonic() + max(0.0, args.timeout)

    while not rospy.is_shutdown():
        remaining = deadline - time.monotonic()
        if args.timeout > 0.0 and remaining <= 0.0:
            rospy.logerr("control manager did not become ready: %s", topic)
            return 1
        try:
            wait = min(max(0.1, args.poll), max(0.1, remaining)) \
                if args.timeout > 0.0 else max(0.1, args.poll)
            state = rospy.wait_for_message(topic, ControlState, timeout=wait)
        except rospy.ROSException:
            continue

        # The controller's takeoff callback requires these same validity
        # conditions.  For fixed-wing, state_valid already includes
        # airspeed_valid, but checking both makes the terminal diagnosis clear.
        if state.state_valid and state.odometry_fresh and state.imu_fresh \
                and state.airspeed_valid:
            rospy.loginfo(
                "control manager ready: state_valid=%s odometry=%s imu=%s airspeed=%s",
                state.state_valid,
                state.odometry_fresh,
                state.imu_fresh,
                state.airspeed_valid,
            )
            return 0

    return 1


if __name__ == "__main__":
    sys.exit(main())
