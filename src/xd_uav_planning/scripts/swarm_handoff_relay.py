#!/usr/bin/env python3
"""Make the official EGO sequential swarm handoff reliable externally."""

import copy
import threading

import rospy
from traj_utils.msg import Bspline, MultiBsplines


class SwarmHandoffRelay:
    def __init__(self):
        self._lock = threading.Lock()
        self._expected_drone_id = int(rospy.get_param("~expected_drone_id"))
        self._latest = None
        self._broadcasts = {}

        input_topic = rospy.get_param("~input_topic")
        output_topic = rospy.get_param("~output_topic")
        broadcast_topic = rospy.get_param(
            "~broadcast_topic", "/broadcast_bspline")
        replay_period = float(rospy.get_param("~replay_period", 0.5))
        if self._expected_drone_id < 0:
            raise ValueError("expected_drone_id must be non-negative")
        if replay_period <= 0.0:
            raise ValueError("replay_period must be positive")

        self._publisher = rospy.Publisher(
            output_topic, MultiBsplines, queue_size=10, latch=True)
        self._raw_subscriber = rospy.Subscriber(
            input_topic, MultiBsplines, self._raw_callback, queue_size=10)
        self._broadcast_subscriber = rospy.Subscriber(
            broadcast_topic, Bspline, self._broadcast_callback, queue_size=100)
        self._timer = rospy.Timer(
            rospy.Duration(replay_period), self._replay_callback)

    @staticmethod
    def _trajectory_key(trajectory):
        return (trajectory.start_time.to_sec(), int(trajectory.traj_id))

    def _valid_chain(self, message):
        if message.drone_id_from != self._expected_drone_id:
            return False
        if len(message.traj) != self._expected_drone_id + 1:
            return False
        return all(trajectory.drone_id == index
                   for index, trajectory in enumerate(message.traj))

    def _publish_locked(self):
        if self._latest is not None:
            self._publisher.publish(copy.deepcopy(self._latest))

    def _raw_callback(self, message):
        if not self._valid_chain(message):
            rospy.logwarn_throttle(
                2.0, "rejecting malformed EGO swarm handoff for drone %d",
                self._expected_drone_id)
            return
        with self._lock:
            chain = copy.deepcopy(message)
            for drone_id, trajectory in self._broadcasts.items():
                current = chain.traj[drone_id]
                if self._trajectory_key(trajectory) > self._trajectory_key(current):
                    chain.traj[drone_id] = copy.deepcopy(trajectory)
            self._latest = chain
            self._publish_locked()

    def _broadcast_callback(self, message):
        drone_id = int(message.drone_id)
        if drone_id < 0 or drone_id > self._expected_drone_id:
            return
        with self._lock:
            previous = self._broadcasts.get(drone_id)
            if (previous is not None and
                    self._trajectory_key(message) <= self._trajectory_key(previous)):
                return
            self._broadcasts[drone_id] = copy.deepcopy(message)
            if self._latest is None:
                return
            current = self._latest.traj[drone_id]
            if self._trajectory_key(message) > self._trajectory_key(current):
                self._latest.traj[drone_id] = copy.deepcopy(message)
                self._publish_locked()

    def _replay_callback(self, _event):
        # Official EGO drops a predecessor message received before its first
        # odometry sample. Periodic replay guarantees delivery after odometry.
        with self._lock:
            self._publish_locked()


if __name__ == "__main__":
    rospy.init_node("swarm_handoff_relay")
    SwarmHandoffRelay()
    rospy.spin()
