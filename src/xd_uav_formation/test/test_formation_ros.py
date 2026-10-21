#!/usr/bin/env python3

import math
import threading
import time
import unittest

import rospy
import rostest
from mavros_msgs.msg import PositionTarget, State
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String
from std_srvs.srv import SetBool
from xd_uav_controller.msg import ControlState
from xd_uav_task_execute.msg import ExecuteTaskActionGoal
import tf2_ros


class FormationRosTest(unittest.TestCase):
    def setUp(self):
        self._airframe = rospy.get_param("~airframe", "multirotor")
        self._expected_x = float(rospy.get_param("~expected_x", 6.0))
        self._separate_frames = bool(rospy.get_param("~separate_frames", False))
        self._leader_airframe_override = None
        self._leader_regime_override = None
        self._lock = threading.Lock()
        self._status = ""
        self._formation_count = 0
        self._latest_velocity = None
        self._own_x = 0.0
        self._running = True
        self._own_state_pub = rospy.Publisher(
            "/uav2/control_manager/state", ControlState, queue_size=10)
        self._leader_state_pub = rospy.Publisher(
            "/uav1/control_manager/state", ControlState, queue_size=10)
        self._own_mavros_pub = rospy.Publisher(
            "/uav2/mavros/state", State, queue_size=10)
        self._leader_mavros_pub = rospy.Publisher(
            "/uav1/mavros/state", State, queue_size=10)
        self._external_pub = rospy.Publisher(
            "/uav2/control/reference/setpoint", PositionTarget, queue_size=10)
        self._preempt_pub = rospy.Publisher(
            "/uav2/formation/preempt", String, queue_size=10)
        self._task_goal_pub = rospy.Publisher(
            "/uav2/task_execute/execute/goal", ExecuteTaskActionGoal,
            queue_size=10)
        self._status_sub = rospy.Subscriber(
            "/uav2/formation_follower/status", String, self._on_status)
        self._reference_sub = rospy.Subscriber(
            "/uav2/control/reference/setpoint", PositionTarget,
            self._on_reference)
        self._publisher_thread = threading.Thread(target=self._publish_states)
        if self._separate_frames:
            self._tf_broadcaster = tf2_ros.StaticTransformBroadcaster()
            transform = TransformStamped()
            transform.header.stamp = rospy.Time.now()
            transform.header.frame_id = "uav2/odom"
            transform.child_frame_id = "uav1/odom"
            transform.transform.translation.x = 100.0
            transform.transform.rotation.w = 1.0
            self._tf_broadcaster.sendTransform(transform)
        self._publisher_thread.daemon = True
        self._publisher_thread.start()

    def tearDown(self):
        self._running = False
        self._publisher_thread.join(timeout=2.0)

    def _publish_states(self):
        while self._running and not rospy.is_shutdown():
            now = rospy.Time.now()
            for publisher, x in ((self._own_state_pub, self._own_x),
                                 (self._leader_state_pub, 10.0)):
                state = ControlState()
                state.header.stamp = now
                state.header.frame_id = (
                    "uav1/odom" if publisher is self._leader_state_pub
                    else "uav2/odom") if self._separate_frames else "world"
                state.state_valid = True
                if self._airframe == "multirotor":
                    state.vehicle_type = ControlState.VEHICLE_MULTIROTOR
                    state.airframe_type = ControlState.AIRFRAME_MULTIROTOR
                    state.flight_regime = ControlState.REGIME_HOVER
                else:
                    state.vehicle_type = ControlState.VEHICLE_FIXEDWING
                    state.airframe_type = (
                        ControlState.AIRFRAME_VTOL if self._airframe == "vtol"
                        else ControlState.AIRFRAME_FIXED_WING)
                    state.flight_regime = ControlState.REGIME_FORWARD_FLIGHT
                    state.velocity_odom.x = 15.0
                    bank = math.atan(1.5 / 9.8)
                    state.orientation_odom_body.x = math.sin(bank / 2.0)
                    state.orientation_odom_body.w = math.cos(bank / 2.0)
                if publisher is self._leader_state_pub:
                    if self._leader_airframe_override is not None:
                        state.airframe_type = self._leader_airframe_override
                    if self._leader_regime_override is not None:
                        state.flight_regime = self._leader_regime_override
                state.vehicle_action = ControlState.ACTION_HOLD
                state.position_odom.x = x
                if self._airframe == "multirotor":
                    state.orientation_odom_body.w = 1.0
                publisher.publish(state)
            mavros = State()
            mavros.armed = True
            mavros.mode = "OFFBOARD"
            self._own_mavros_pub.publish(mavros)
            self._leader_mavros_pub.publish(mavros)
            time.sleep(0.05)

    def _on_status(self, message):
        with self._lock:
            self._status = message.data

    def _on_reference(self, message):
        if abs(message.position.x - self._expected_x) < 0.01:
            with self._lock:
                self._formation_count += 1
                if not (message.type_mask & PositionTarget.IGNORE_VX):
                    self._latest_velocity = (message.velocity.x,
                                             message.velocity.y)

    def _until(self, predicate, timeout=5.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and not rospy.is_shutdown():
            with self._lock:
                if predicate():
                    return True
            time.sleep(0.02)
        return False

    def test_stream_yields_to_other_setpoint(self):
        rospy.wait_for_service(
            "/uav2/formation_follower/set_enabled", timeout=10.0)
        start = rospy.ServiceProxy(
            "/uav2/formation_follower/set_enabled", SetBool)
        self._leader_airframe_override = (
            ControlState.AIRFRAME_FIXED_WING
            if self._airframe == "multirotor"
            else ControlState.AIRFRAME_MULTIROTOR)
        time.sleep(0.15)
        mismatch = start(True)
        self.assertFalse(mismatch.success)
        self.assertEqual(mismatch.message, "same_airframe_type_required")
        self._leader_airframe_override = None
        response = None
        for _ in range(40):
            response = start(True)
            if response.success:
                break
            time.sleep(0.05)
        self.assertTrue(response.success, response.message)
        self.assertTrue(self._until(lambda: self._formation_count >= 3),
                        "formation did not stream its slot")
        base_x = 0.0 if self._airframe == "multirotor" else 15.0
        correction = max(-3.0, min(3.0, 0.25 * self._expected_x))
        expected_velocity_x = base_x + correction
        self.assertTrue(self._until(
            lambda: self._latest_velocity is not None and
            abs(self._latest_velocity[0] - expected_velocity_x) < 0.01),
            "slot error did not adjust the requested speed")
        self._own_x = self._expected_x
        self.assertTrue(self._until(
            lambda: self._latest_velocity is not None and
            abs(self._latest_velocity[0] - base_x) < 0.01),
            "speed did not return to feed-forward after catching up")
        with self._lock:
            self.assertEqual(self._status, "active:following")
            if self._airframe != "multirotor":
                self.assertAlmostEqual(self._latest_velocity[1], -8.0,
                                       places=2)

        external = PositionTarget()
        external.header.stamp = rospy.Time.now()
        external.header.frame_id = "world"
        external.position.x = 20.0
        for _ in range(3):
            self._external_pub.publish(external)
            time.sleep(0.03)
        self.assertTrue(self._until(
            lambda: self._status.startswith(
                "inactive:external_setpoint_from:")),
            "formation did not yield to a different publisher")
        with self._lock:
            count = self._formation_count
        time.sleep(0.25)
        with self._lock:
            self.assertEqual(self._formation_count, count)

    def test_flight_regime_change_ends_formation(self):
        rospy.wait_for_service(
            "/uav2/formation_follower/set_enabled", timeout=10.0)
        start = rospy.ServiceProxy(
            "/uav2/formation_follower/set_enabled", SetBool)
        response = None
        for _ in range(40):
            response = start(True)
            if response.success:
                break
            time.sleep(0.05)
        self.assertTrue(response.success, response.message)
        self.assertTrue(self._until(lambda: self._formation_count >= 2))
        self._leader_regime_override = (
            ControlState.REGIME_FORWARD_FLIGHT
            if self._airframe == "multirotor"
            else ControlState.REGIME_HOVER)
        self.assertTrue(self._until(lambda: self._status.startswith(
            "inactive:same_flight_regime_required")))

    def test_task_preempt_ends_formation_before_new_reference(self):
        rospy.wait_for_service(
            "/uav2/formation_follower/set_enabled", timeout=10.0)
        start = rospy.ServiceProxy(
            "/uav2/formation_follower/set_enabled", SetBool)
        response = None
        for _ in range(40):
            response = start(True)
            if response.success:
                break
            time.sleep(0.05)
        self.assertTrue(response.success, response.message)
        self.assertTrue(self._until(lambda: self._formation_count >= 2))
        self._preempt_pub.publish(String(data="new_task"))
        self.assertTrue(self._until(
            lambda: self._status == "inactive:preempt:new_task"))

    def test_execute_task_goal_ends_formation(self):
        rospy.wait_for_service(
            "/uav2/formation_follower/set_enabled", timeout=10.0)
        start = rospy.ServiceProxy(
            "/uav2/formation_follower/set_enabled", SetBool)
        response = None
        for _ in range(40):
            response = start(True)
            if response.success:
                break
            time.sleep(0.05)
        self.assertTrue(response.success, response.message)
        self.assertTrue(self._until(lambda: self._formation_count >= 2))
        goal = ExecuteTaskActionGoal()
        goal.header.stamp = rospy.Time.now()
        goal.goal.task_id = 42
        self._task_goal_pub.publish(goal)
        self.assertTrue(self._until(
            lambda: self._status == "inactive:task_goal:42"))


if __name__ == "__main__":
    rospy.init_node("formation_ros_test")
    rostest.rosrun("xd_uav_formation", "formation_ros_test", FormationRosTest)
