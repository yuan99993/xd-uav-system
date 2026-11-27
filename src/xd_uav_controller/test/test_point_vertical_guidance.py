#!/usr/bin/env python3
import time
import unittest

import rospy
import rostest
from mavros_msgs.msg import PositionTarget
from xd_uav_controller.msg import ControlCommand, ControlState


class PointVerticalInterfaceTest(unittest.TestCase):
    def test_point_orbit_and_velocity_masks(self):
        state_pub = rospy.Publisher('state', ControlState, queue_size=1)
        reference_pub = rospy.Publisher('reference_position_target', PositionTarget,
                                        queue_size=1)
        commands = []
        subscriber = rospy.Subscriber('command', ControlCommand,
                                      lambda msg: commands.append(msg), queue_size=20)
        state = ControlState()
        state.header.frame_id = 'uav1/odom'
        state.body_frame_id = 'uav1/base_link'
        state.airframe_type = rospy.get_param('~airframe_type_id', 1)
        state.vehicle_type = (ControlState.VEHICLE_FIXEDWING
                              if state.airframe_type == 1
                              else ControlState.VEHICLE_MULTIROTOR)
        state.flight_regime = 2
        state.position_odom.z = 100.0
        state.velocity_odom.x = 15.0
        state.orientation_odom_body.w = 1.0
        state.airspeed = 15.0
        state.groundspeed = 15.0
        state.state_valid = state.localization_valid = True
        state.odometry_fresh = state.imu_fresh = True
        state.acceleration_fresh = state.airspeed_valid = True
        state.stable = True

        def sample(reference, duration=0.8):
            commands.clear()
            end = time.monotonic() + duration
            while time.monotonic() < end:
                state.header.stamp = rospy.Time.now()
                state_pub.publish(state)
                reference.header.stamp = rospy.Time.now()
                reference_pub.publish(reference)
                time.sleep(0.02)
            valid = [c for c in commands if c.valid and
                     c.reference_source == 'position_target']
            self.assertTrue(valid, 'No valid position_target output')
            return valid[-1]

        reference = PositionTarget()
        reference.header.frame_id = 'uav1/odom'
        reference.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        reference.type_mask = (PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY |
                               PositionTarget.IGNORE_VZ | PositionTarget.IGNORE_AFX |
                               PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
                               PositionTarget.IGNORE_YAW | PositionTarget.IGNORE_YAW_RATE)
        reference.position.x = 1200.0
        reference.position.z = 5.0
        # Allow publishers/subscribers to connect before measuring.
        end = time.monotonic() + 3.0
        while time.monotonic() < end and (not state_pub.get_num_connections() or
                                         not reference_pub.get_num_connections()):
            time.sleep(0.02)
        first = sample(reference)
        self.assertLess(abs(first.body_rate.y), 0.4)
        self.assertLess(abs(first.body_rate.z), 0.01)
        second = sample(reference)
        self.assertGreater(second.body_rate.y, first.body_rate.y + 0.02,
                           'Repeated identical point must preserve the ramp state')

        # A nearby low target enters an orbit without rejecting the goal.
        reference.position.x = 100.0
        close = sample(reference)
        self.assertGreater(abs(close.body_rate.z), 0.05)

        # Same message and even a huge position error, but explicit velocity
        # fields: must bypass both point orbit and point rate limiting.
        reference.type_mask &= ~(PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY |
                                 PositionTarget.IGNORE_VZ)
        reference.velocity.x = 15.0
        reference.velocity.z = -8.0
        fast = sample(reference)
        self.assertGreater(fast.body_rate.y, 0.6)
        self.assertLess(abs(fast.body_rate.z), 0.01)

        # Enabled Vz=0 also bypasses the point mode. Ignore P for track's mask.
        reference.type_mask |= (PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY |
                                PositionTarget.IGNORE_PZ)
        reference.velocity.z = 0.0
        level = sample(reference)
        self.assertLess(abs(level.body_rate.y), 0.05)
        self.assertLess(abs(level.body_rate.z), 0.01)
        subscriber.unregister()


if __name__ == '__main__':
    rospy.init_node('test_point_vertical_guidance')
    rostest.rosrun('xd_uav_controller', 'point_vertical_guidance_interface',
                   PointVerticalInterfaceTest)
