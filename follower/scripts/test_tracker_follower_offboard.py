#!/usr/bin/env python3
"""Run a bounded Tracker -> Follower -> PX4 SITL offboard integration test."""

import argparse
import time

import rospy
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from std_srvs.srv import SetBool
from tracker.msg import ExternalInput


class TrackerFollowerOffboardTest:
    def __init__(self, duration):
        self.duration = max(1.0, duration)
        self.state = State()
        self.velocity_pub = rospy.Publisher(
            '/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=20)
        self.external_input_pub = rospy.Publisher(
            '/tracker_node/external_input', ExternalInput, queue_size=10)
        rospy.Subscriber('/mavros/state', State, self._state_callback, queue_size=10)

    def _state_callback(self, msg):
        self.state = msg

    @staticmethod
    def _zero_velocity_message():
        msg = TwistStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'base_link'
        return msg

    def _publish_zero_for(self, seconds):
        rate = rospy.Rate(20)
        end_time = time.monotonic() + seconds
        while not rospy.is_shutdown() and time.monotonic() < end_time:
            self.velocity_pub.publish(self._zero_velocity_message())
            rate.sleep()

    def _wait_for_mode_with_setpoints(self, mode, timeout):
        """Keep the OFFBOARD input stream alive while waiting for PX4 mode feedback."""
        rate = rospy.Rate(20)
        deadline = time.monotonic() + timeout
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            self.velocity_pub.publish(self._zero_velocity_message())
            if self.state.mode == mode:
                return
            rate.sleep()
        raise RuntimeError('PX4 did not enter {} mode'.format(mode))

    def _wait_for_armed_with_setpoints(self, timeout):
        """PX4 state telemetry can lag the arming service response by one update."""
        rate = rospy.Rate(20)
        deadline = time.monotonic() + timeout
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            self.velocity_pub.publish(self._zero_velocity_message())
            if self.state.armed:
                return
            rate.sleep()
        raise RuntimeError('PX4 is not armed after OFFBOARD setup')

    def _request_mode_with_setpoints(self, set_mode, mode, timeout):
        """Request a PX4 mode and wait for its asynchronous MAVROS state update."""
        response = set_mode(base_mode=0, custom_mode=mode)
        if not response.mode_sent:
            return False

        rate = rospy.Rate(20)
        deadline = time.monotonic() + timeout
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            self.velocity_pub.publish(self._zero_velocity_message())
            if self.state.mode == mode:
                return True
            rate.sleep()
        return False

    def _publish_external_input(self, command=''):
        msg = ExternalInput()
        msg.header.stamp = rospy.Time.now()
        msg.source = 'bounding_box'
        msg.command = command
        if command != 'stop_track':
            # Deliberately offset target: creates forward, vertical and yaw commands.
            msg.normalized_bbox = [0.30, 0.30, 0.10, 0.10]
            msg.has_normalized_bbox = True
            msg.confidence = 0.90
            msg.class_id = 0
        self.external_input_pub.publish(msg)

    def run(self):
        rospy.loginfo('Waiting for PX4/MAVROS connection...')
        deadline = time.monotonic() + 15.0
        while not rospy.is_shutdown() and not self.state.connected:
            if time.monotonic() > deadline:
                raise RuntimeError('Timed out waiting for MAVROS connection')
            rospy.sleep(0.1)

        rospy.wait_for_service('/mavros/cmd/arming', timeout=10.0)
        rospy.wait_for_service('/mavros/set_mode', timeout=10.0)
        rospy.wait_for_service('/follower_node/start', timeout=10.0)
        rospy.wait_for_service('/follower_node/stop', timeout=10.0)
        arm = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        set_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        follower_start = rospy.ServiceProxy('/follower_node/start', SetBool)
        follower_stop = rospy.ServiceProxy('/follower_node/stop', SetBool)

        try:
            rospy.loginfo('Pre-streaming zero velocity setpoints for OFFBOARD...')
            self._publish_zero_for(3.0)

            mode_response = set_mode(base_mode=0, custom_mode='OFFBOARD')
            if not mode_response.mode_sent:
                raise RuntimeError('PX4 OFFBOARD mode request was rejected')
            self._wait_for_mode_with_setpoints('OFFBOARD', 2.0)

            if not self.state.armed:
                arm_response = arm(value=True)
                if not arm_response.success:
                    raise RuntimeError('PX4 arming request was rejected')
            self._wait_for_armed_with_setpoints(3.0)

            follower_start(data=True)
            self._publish_external_input(command='start_track')
            rospy.loginfo('Running Tracker -> Follower velocity test for %.1f seconds...', self.duration)
            rate = rospy.Rate(10)
            end_time = time.monotonic() + self.duration
            while not rospy.is_shutdown() and time.monotonic() < end_time:
                self._publish_external_input()
                rate.sleep()
        finally:
            self._publish_external_input(command='stop_track')
            try:
                follower_stop(data=True)
            except rospy.ServiceException:
                pass
            self._publish_zero_for(2.0)
            try:
                for _ in range(3):
                    if self._request_mode_with_setpoints(set_mode, 'AUTO.LOITER', 3.0):
                        rospy.loginfo('Test stopped; PX4 is holding in AUTO.LOITER.')
                        break
                else:
                    rospy.logwarn('PX4 did not confirm AUTO.LOITER after the test.')
            except rospy.ServiceException:
                rospy.logwarn('Unable to request AUTO.LOITER; keep sending zero setpoints before stopping.')


def main():
    rospy.init_node('tracker_follower_offboard_test', anonymous=True)
    parser = argparse.ArgumentParser(description='Bounded Tracker/Follower PX4 SITL test.')
    parser.add_argument('--duration', type=float, default=8.0)
    args = parser.parse_args(rospy.myargv()[1:])
    TrackerFollowerOffboardTest(args.duration).run()


if __name__ == '__main__':
    try:
        main()
    except (rospy.ROSException, rospy.ServiceException, RuntimeError) as exc:
        rospy.logerr('Offboard integration test failed: %s', exc)
        raise
