#!/usr/bin/env python3

import threading
import time
import unittest

import rospy
import rostest
from mavros_msgs.msg import PositionTarget, State
from std_msgs.msg import String
from std_srvs.srv import SetBool
from xd_uav_controller.msg import ControlState
from xd_uav_formation.srv import FormationControl


class ConsensusRosTest(unittest.TestCase):
    def setUp(self):
        self._airframe = rospy.get_param("~airframe", "multirotor")
        self._lock = threading.Lock()
        self._running = True
        self._peer_airframe_override = None
        self._statuses = {}
        self._references = {"uav1": 0, "uav2": 0}
        self._latest_velocity_x = {}
        self._uav2_x = 100.0
        self._state_pubs = {name: rospy.Publisher(
            "/{}/control_manager/state".format(name), ControlState,
            queue_size=10) for name in self._references}
        self._mavros_pubs = {name: rospy.Publisher(
            "/{}/mavros/state".format(name), State, queue_size=10)
            for name in self._references}
        self._subs = []
        for name in self._references:
            self._subs.append(rospy.Subscriber(
                "/{}/formation_consensus/status".format(name), String,
                self._on_status, callback_args=name))
            self._subs.append(rospy.Subscriber(
                "/{}/control/reference/setpoint".format(name), PositionTarget,
                self._on_reference, callback_args=name))
        self._external = rospy.Publisher(
            "/uav2/control/reference/setpoint", PositionTarget,
            queue_size=10)
        self._thread = threading.Thread(target=self._publish_states)
        self._thread.daemon = True
        self._thread.start()

    def tearDown(self):
        self._running = False
        self._thread.join(timeout=2.0)

    def _publish_states(self):
        while self._running and not rospy.is_shutdown():
            now = rospy.Time.now()
            for name, y in (("uav1", 40.0), ("uav2", -40.0)):
                state = ControlState()
                state.header.stamp = now
                state.header.frame_id = "world"
                state.state_valid = True
                if self._airframe == "multirotor":
                    state.airframe_type = ControlState.AIRFRAME_MULTIROTOR
                    state.vehicle_type = ControlState.VEHICLE_MULTIROTOR
                    state.flight_regime = ControlState.REGIME_HOVER
                else:
                    state.airframe_type = ControlState.AIRFRAME_FIXED_WING
                    state.vehicle_type = ControlState.VEHICLE_FIXEDWING
                    state.flight_regime = ControlState.REGIME_FORWARD_FLIGHT
                    state.velocity_odom.x = 15.0
                if name == "uav2" and self._peer_airframe_override is not None:
                    state.airframe_type = self._peer_airframe_override
                state.vehicle_action = ControlState.ACTION_HOLD
                state.position_odom.x = (self._uav2_x if name == "uav2"
                                         else 100.0)
                state.position_odom.y = y
                state.position_odom.z = 100.0
                state.orientation_odom_body.w = 1.0
                self._state_pubs[name].publish(state)
                mavros = State()
                mavros.armed = True
                mavros.mode = "OFFBOARD"
                self._mavros_pubs[name].publish(mavros)
            time.sleep(0.05)

    def _on_status(self, message, name):
        with self._lock:
            self._statuses[name] = message.data

    def _on_reference(self, message, name):
        expected_y = 40.0 if name == "uav1" else -40.0
        if (abs(message.position.y - expected_y) < 0.01 and
                message.header.frame_id == "world" and
                not (message.type_mask & PositionTarget.IGNORE_VX)):
            with self._lock:
                self._latest_velocity_x[name] = message.velocity.x
                if (abs(message.position.x - 120.0) < 0.01 and
                        (self._airframe == "multirotor" or
                         abs(message.velocity.x - 15.0) < 0.01)):
                    self._references[name] += 1

    def _until(self, predicate, timeout=5.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and not rospy.is_shutdown():
            with self._lock:
                if predicate():
                    return True
            time.sleep(0.02)
        return False

    def test_both_members_stream_and_preempt_together(self):
        services = {}
        for name in self._references:
            path = "/{}/formation_consensus/set_enabled".format(name)
            rospy.wait_for_service(path, timeout=10.0)
            services[name] = rospy.ServiceProxy(path, SetBool)
        self._peer_airframe_override = (
            ControlState.AIRFRAME_FIXED_WING
            if self._airframe == "multirotor" else
            ControlState.AIRFRAME_MULTIROTOR)
        time.sleep(0.15)
        mismatch = services["uav1"](True)
        self.assertFalse(mismatch.success)
        self.assertEqual(mismatch.message, "same_airframe_type_required")
        self._peer_airframe_override = None
        rospy.wait_for_service("/formation/set_enabled", timeout=5)
        team_service = rospy.ServiceProxy(
            "/formation/set_enabled", FormationControl)
        response = None
        for _ in range(40):
            response = team_service(
                mode="consensus",
                members=["uav1", "uav2"], enabled=True)
            if response.success:
                break
            time.sleep(0.05)
        self.assertTrue(response.success, response.message)
        self.assertTrue(self._until(lambda: all(
            count >= 3 for count in self._references.values())),
            "both members did not stream their slots")
        base_speed = 3.0 if self._airframe == "multirotor" else 15.0
        self._uav2_x = 80.0
        self.assertTrue(self._until(lambda:
            abs(self._latest_velocity_x.get("uav1", 0.0) -
                (base_speed - 2.0)) < 0.01 and
            abs(self._latest_velocity_x.get("uav2", 0.0) -
                (base_speed + 2.0)) < 0.01),
            "lagging member did not speed up while its neighbor slowed")
        self._uav2_x = 100.0
        self.assertTrue(self._until(lambda: all(
            abs(self._latest_velocity_x.get(name, 0.0) - base_speed) < 0.01
            for name in self._references)),
            "members did not return to cruise speed after convergence")
        external = PositionTarget()
        external.header.stamp = rospy.Time.now()
        external.header.frame_id = "world"
        external.position.x = 200.0
        for _ in range(3):
            self._external.publish(external)
            time.sleep(0.03)
        self.assertTrue(self._until(lambda: all(
            status.startswith("inactive:")
            for status in self._statuses.values()) and
            len(self._statuses) == 2), "team did not stop together")
        self.assertTrue(self._statuses["uav2"].startswith(
            "inactive:external_setpoint_from:"))
        self.assertTrue(self._statuses["uav1"].startswith(
            "inactive:team_cancel:"))


if __name__ == "__main__":
    rospy.init_node("consensus_ros_test")
    rostest.rosrun("xd_uav_formation", "consensus_ros_test",
                   ConsensusRosTest)
