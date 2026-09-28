#!/usr/bin/env python3

import threading
import unittest

import rospy
import rostest
from geometry_msgs.msg import Point
from traj_utils.msg import Bspline, MultiBsplines


def trajectory(drone_id, traj_id, start_time):
    message = Bspline()
    message.drone_id = drone_id
    message.order = 3
    message.traj_id = traj_id
    message.start_time = rospy.Time.from_sec(start_time)
    message.knots = [0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0]
    message.pos_pts = [Point(x=float(index)) for index in range(4)]
    return message


class SwarmHandoffRelayTest(unittest.TestCase):
    def setUp(self):
        self._lock = threading.Lock()
        self._messages = []
        self._raw = rospy.Publisher(
            "/test/swarm_raw", MultiBsplines, queue_size=1)
        self._broadcast = rospy.Publisher(
            "/test/broadcast", Bspline, queue_size=1)
        self._subscriber = rospy.Subscriber(
            "/test/swarm_reliable", MultiBsplines, self._callback,
            queue_size=20)

    def _callback(self, message):
        with self._lock:
            self._messages.append(message)

    @staticmethod
    def _wait(predicate, timeout=4.0):
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        rate = rospy.Rate(50)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            if predicate():
                return True
            rate.sleep()
        return False

    def _count(self):
        with self._lock:
            return len(self._messages)

    def test_latch_replay_validation_and_broadcast_refresh(self):
        self.assertTrue(self._wait(
            lambda: self._raw.get_num_connections() > 0 and
                    self._broadcast.get_num_connections() > 0))

        invalid = MultiBsplines(drone_id_from=1)
        invalid.traj = [trajectory(0, 1, 1.0)]
        self._raw.publish(invalid)
        rospy.sleep(0.25)
        self.assertEqual(0, self._count())

        valid = MultiBsplines(drone_id_from=1)
        valid.traj = [trajectory(0, 1, 1.0), trajectory(1, 2, 1.0)]
        self._raw.publish(valid)
        self.assertTrue(self._wait(lambda: self._count() >= 2),
                        "handoff was not replayed")

        updated = trajectory(0, 9, 2.0)
        self._broadcast.publish(updated)
        self.assertTrue(self._wait(
            lambda: any(message.traj[0].traj_id == 9
                        for message in self._messages)))

        late_messages = []
        late_subscriber = rospy.Subscriber(
            "/test/swarm_reliable", MultiBsplines, late_messages.append,
            queue_size=1)
        self.assertTrue(self._wait(
            lambda: late_messages and late_messages[-1].traj[0].traj_id == 9),
            "latched handoff was not delivered to a late subscriber")
        late_subscriber.unregister()


if __name__ == "__main__":
    rospy.init_node("test_swarm_handoff_relay")
    rostest.rosrun("xd_uav_planning", "swarm_handoff_relay",
                   SwarmHandoffRelayTest)
