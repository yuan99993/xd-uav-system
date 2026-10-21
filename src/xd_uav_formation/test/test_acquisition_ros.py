#!/usr/bin/env python3
"""Exercise every automatic gathering phase with a synthetic ROS team."""

import threading
import time
import unittest

import rospy
import rostest
from mavros_msgs.msg import State
from std_msgs.msg import String
from xd_uav_controller.msg import ControlState
from xd_uav_formation.srv import FormationControl

from xd_uav_formation.core import acquisition_slots, formation_offsets


class AcquisitionRosTest(unittest.TestCase):
    def setUp(self):
        self._fixedwing = rospy.get_param("~airframe", "multirotor") == "fixedwing"
        self._members = ("uav1", "uav2", "uav3")
        self._spacing = 40.0 if self._fixedwing else 10.0
        self._positions = [(0.0, 3.0 * index, 20.0)
                           for index in range(len(self._members))]
        self._lock = threading.Lock()
        self._running = True
        self._statuses = {}
        self._state_publishers = [rospy.Publisher(
            "/{}/control_manager/state".format(member), ControlState,
            queue_size=10) for member in self._members]
        self._mavros_publishers = [rospy.Publisher(
            "/{}/mavros/state".format(member), State,
            queue_size=10) for member in self._members]
        self._subscribers = [rospy.Subscriber(
            "/{}/formation_consensus/status".format(member), String,
            self._status_callback, callback_args=member)
            for member in self._members]
        self._thread = threading.Thread(target=self._publish_states)
        self._thread.daemon = True
        self._thread.start()

    def tearDown(self):
        self._running = False
        self._thread.join(timeout=2.0)

    def _status_callback(self, message, member):
        with self._lock:
            self._statuses[member] = message.data

    def _publish_states(self):
        while self._running and not rospy.is_shutdown():
            with self._lock:
                positions = tuple(self._positions)
            stamp = rospy.Time.now()
            for index, position in enumerate(positions):
                state = ControlState()
                state.header.stamp = stamp
                state.header.frame_id = "world"
                state.state_valid = True
                state.position_odom.x, state.position_odom.y, \
                    state.position_odom.z = position
                state.orientation_odom_body.w = 1.0
                state.vehicle_action = ControlState.ACTION_HOLD
                if self._fixedwing:
                    state.airframe_type = ControlState.AIRFRAME_FIXED_WING
                    state.vehicle_type = ControlState.VEHICLE_FIXEDWING
                    state.flight_regime = ControlState.REGIME_FORWARD_FLIGHT
                    state.velocity_odom.x = 15.0
                else:
                    state.airframe_type = ControlState.AIRFRAME_MULTIROTOR
                    state.vehicle_type = ControlState.VEHICLE_MULTIROTOR
                    state.flight_regime = ControlState.REGIME_HOVER
                self._state_publishers[index].publish(state)
                mavros = State()
                mavros.armed = True
                mavros.mode = "OFFBOARD"
                self._mavros_publishers[index].publish(mavros)
            time.sleep(0.05)

    def _until(self, expected, timeout=8.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and not rospy.is_shutdown():
            with self._lock:
                if all(self._statuses.get(member) == expected
                       for member in self._members):
                    return True
            time.sleep(0.03)
        return False

    def test_gather_then_follow(self):
        path = "/formation/set_enabled"
        rospy.wait_for_service(path, timeout=10.0)
        service = rospy.ServiceProxy(path, FormationControl)
        for _ in range(40):
            result = service(
                mode="consensus",
                members=list(self._members), enabled=True)
            if result.success:
                break
            time.sleep(0.05)
        self.assertTrue(result.success, result.message)
        self.assertTrue(self._until("active:acquiring_separating"))
        with self._lock:
            self._positions = [(x, y, 20.0 + index * 5.0)
                               for index, (x, y, _) in
                               enumerate(self._positions)]
        self.assertTrue(self._until("active:acquiring_gathering"))
        starts = [(0.0, 3.0 * index, 20.0)
                  for index in range(len(self._members))]
        goals = acquisition_slots(
            starts,
            formation_offsets("rectangle_compact", len(starts),
                              self._spacing), 0.0, 20.0)
        with self._lock:
            self._positions = [(goal[0], goal[1], 20.0 + index * 5.0)
                               for index, goal in enumerate(goals)]
        self.assertTrue(self._until("active:acquiring_merging"))
        with self._lock:
            self._positions = list(goals)
        self.assertTrue(self._until("active:following"))
        self.assertTrue(service(
            mode="consensus",
            members=list(self._members), enabled=False).success)


if __name__ == "__main__":
    rospy.init_node("acquisition_ros_test")
    rostest.rosrun("xd_uav_formation", "acquisition_ros_test",
                   AcquisitionRosTest)
