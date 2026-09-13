#!/usr/bin/env python3

import json
import math
import os
import threading
import unittest

import rosgraph
import rospy
import rostest
import sensor_msgs.point_cloud2 as pc2
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Point32, PoseStamped
from mavros_msgs.msg import PositionTarget
from quadrotor_msgs.msg import PositionCommand
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, Header
from traj_utils.msg import Bspline
from xd_uav_controller.msg import ControlState


class SingleChainTest(unittest.TestCase):
    def setUp(self):
        self.lock = threading.Lock()
        self.publish_state = True
        self.publish_cloud = True
        self.relay_command = True
        self.raw_command = None
        self.bspline = None
        self.candidate = None
        self.healthy = None
        self.diagnostics = None
        self.command_times = []
        self.candidate_times = []
        self.cloud_count = 0

        self.state_pub = rospy.Publisher(
            "/uav1/control_manager/state", ControlState, queue_size=20)
        self.cloud_pub = rospy.Publisher(
            "/drone_0_synthetic/cloud", PointCloud2, queue_size=2)
        self.command_pub = rospy.Publisher(
            "/e2/position_command", PositionCommand, queue_size=50)
        self.goal_pub = rospy.Publisher(
            "/move_base_simple/goal", PoseStamped, queue_size=1, latch=True)
        rospy.Subscriber("/e2/raw_position_command", PositionCommand,
                         self._command_callback, queue_size=50)
        rospy.Subscriber("/drone_0_planning/bspline", Bspline,
                         self._bspline_callback, queue_size=10)
        rospy.Subscriber("/uav1/ego/reference_candidate", PositionTarget,
                         self._candidate_callback, queue_size=50)
        rospy.Subscriber("/uav1/ego/bridge/healthy", Bool,
                         self._healthy_callback, queue_size=5)
        rospy.Subscriber("/uav1/ego/bridge/diagnostics", DiagnosticArray,
                         self._diagnostics_callback, queue_size=5)

        self.state_timer = rospy.Timer(rospy.Duration(0.01), self._state_timer)
        self.cloud_timer = rospy.Timer(rospy.Duration(0.1), self._cloud_timer)

    def tearDown(self):
        self.state_timer.shutdown()
        self.cloud_timer.shutdown()

    def _command_callback(self, message):
        with self.lock:
            self.raw_command = message
            self.command_times.append(rospy.Time.now().to_sec())
            relay = self.relay_command
        if relay:
            self.command_pub.publish(message)

    def _bspline_callback(self, message):
        with self.lock:
            self.bspline = message

    def _candidate_callback(self, message):
        with self.lock:
            self.candidate = message
            self.candidate_times.append(rospy.Time.now().to_sec())

    def _healthy_callback(self, message):
        with self.lock:
            self.healthy = bool(message.data)

    def _diagnostics_callback(self, message):
        with self.lock:
            self.diagnostics = message

    def _state_timer(self, _event):
        with self.lock:
            if not self.publish_state:
                return
            command = self.raw_command
        state = ControlState()
        state.header.stamp = rospy.Time.now()
        state.header.frame_id = "world"
        state.body_frame_id = "uav1/base_link"
        if command is None:
            state.position_odom.x = 0.0
            state.position_odom.y = 0.0
            state.position_odom.z = 1.5
            state.orientation_odom_body.w = 1.0
        else:
            state.position_odom = command.position
            state.velocity_odom = command.velocity
            state.orientation_odom_body.w = math.cos(command.yaw * 0.5)
            state.orientation_odom_body.z = math.sin(command.yaw * 0.5)
        state.state_valid = True
        state.localization_valid = True
        state.odometry_fresh = True
        self.state_pub.publish(state)

    def _cloud_timer(self, _event):
        with self.lock:
            if not self.publish_cloud:
                return
            self.cloud_count += 1
        header = Header(stamp=rospy.Time.now(), frame_id="world")
        # A deterministic column offset from the direct path. It exercises the
        # cloud input without making the software-chain test planner-fragile.
        points = []
        for z_index in range(3, 29):
            for angle_index in range(16):
                angle = 2.0 * math.pi * angle_index / 16.0
                points.append((3.0 + 0.35 * math.cos(angle),
                               1.5 + 0.35 * math.sin(angle),
                               z_index * 0.1))
        self.cloud_pub.publish(pc2.create_cloud_xyz32(header, points))

    def _wait_for(self, predicate, timeout, description):
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        rate = rospy.Rate(50)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            if predicate():
                return
            rate.sleep()
        self.fail("timeout waiting for " + description)

    def _healthy_is(self, expected):
        with self.lock:
            return self.healthy is expected

    def _command_x_greater_than(self, threshold):
        with self.lock:
            return (self.raw_command is not None and
                    self.raw_command.position.x > threshold)

    def test_chain_and_faults(self):
        self._wait_for(lambda: self.goal_pub.get_num_connections() > 0,
                       5.0, "manual EGO goal subscriber")
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "world"
        goal.pose.position.x = 6.0
        # A diagonal goal makes a fixed-yaw trajectory observable. The
        # traj_server must derive yaw from the B-spline velocity even when
        # this test deliberately configures time_forward=0.0 below.
        goal.pose.position.y = 6.0
        goal.pose.position.z = 2.0
        goal.pose.orientation.w = 1.0
        self.goal_pub.publish(goal)
        self._wait_for(lambda: self.bspline is not None, 15.0, "Bspline")
        self._wait_for(lambda: self._healthy_is(True), 8.0,
                       "healthy bridge")
        rospy.sleep(1.2)

        with self.lock:
            bspline = self.bspline
            command = self.raw_command
            candidate = self.candidate
            command_times = list(self.command_times)
            candidate_times = list(self.candidate_times)
            cloud_count = self.cloud_count

        self.assertEqual(0, bspline.drone_id)
        self.assertGreater(len(bspline.pos_pts), 3)
        self.assertGreater(len(bspline.knots), 3)
        self.assertIsNotNone(command)
        self.assertEqual("world", command.header.frame_id)
        self.assertEqual(PositionCommand.TRAJECTORY_STATUS_READY,
                         command.trajectory_flag)
        self.assertIsNotNone(candidate)
        self.assertEqual("world", candidate.header.frame_id)
        self.assertTrue(all(math.isfinite(value) for value in (
            candidate.position.x, candidate.position.y, candidate.position.z,
            candidate.velocity.x, candidate.velocity.y, candidate.velocity.z,
            candidate.acceleration_or_force.x,
            candidate.acceleration_or_force.y,
            candidate.acceleration_or_force.z,
            candidate.yaw, candidate.yaw_rate)))
        self.assertGreater(abs(candidate.yaw), 0.2)
        self.assertGreaterEqual(cloud_count, 5)

        # With the 20 m map and 6 m margin, x > 4 m forces a rolling-map
        # recenter. Verify that command production continues afterwards.
        self._wait_for(lambda: self._command_x_greater_than(4.2), 12.0,
                       "rolling-map recenter threshold")
        with self.lock:
            commands_before_recenter_settle = len(self.command_times)
        rospy.sleep(0.35)
        with self.lock:
            self.assertGreater(len(self.command_times),
                               commands_before_recenter_settle)

        command_window = command_times[-101:]
        candidate_window = candidate_times[-101:]
        self.assertGreaterEqual(len(command_window), 80)
        self.assertGreaterEqual(len(candidate_window), 80)
        command_hz = ((len(command_window) - 1) /
                      (command_window[-1] - command_window[0]))
        candidate_hz = ((len(candidate_window) - 1) /
                        (candidate_window[-1] - candidate_window[0]))
        self.assertGreater(command_hz, 80.0)
        self.assertGreater(candidate_hz, 80.0)

        # The live task goal altitude must reach EGO without being forced to
        # the upstream historical z=1 m value.
        endpoint_z = bspline.pos_pts[-1].z
        self.assertAlmostEqual(2.0, endpoint_z, delta=0.25)

        publications, subscriptions, _ = rosgraph.Master(
            rospy.get_name()).getSystemState()
        canonical = "/uav1/control/reference/setpoint"
        bridge = "/uav1/ego_bridge"
        self.assertNotIn(bridge, dict(publications).get(canonical, []))
        self.assertNotIn(bridge, dict(subscriptions).get(canonical, []))

        # Command interruption must fail closed.
        with self.lock:
            self.relay_command = False
        self._wait_for(lambda: self._healthy_is(False), 2.0,
                       "command timeout")
        with self.lock:
            command_fault_diag = self.diagnostics
            self.relay_command = True
        self._wait_for(lambda: self._healthy_is(True), 2.0,
                       "command recovery")

        # State interruption must also fail closed.
        with self.lock:
            self.publish_state = False
        self._wait_for(lambda: self._healthy_is(False), 2.0,
                       "state timeout")
        with self.lock:
            state_fault_diag = self.diagnostics
            self.publish_state = True
        self._wait_for(lambda: self._healthy_is(True), 2.0,
                       "state recovery")

        # The production bridge does not consume sensing. Record that cloud
        # interruption is independently observable but does not alter bridge
        # health; system integration must add sensing health before flight.
        with self.lock:
            before_cloud_stop = self.cloud_count
            self.publish_cloud = False
        rospy.sleep(0.4)
        with self.lock:
            after_cloud_stop = self.cloud_count
            healthy_without_cloud = self.healthy
        self.assertEqual(before_cloud_stop, after_cloud_stop)
        self.assertTrue(healthy_without_cloud)

        evidence = {
            "result": "pass",
            "frame": command.header.frame_id,
            "target_z": 2.0,
            "bspline_endpoint_z": endpoint_z,
            "bspline_drone_id": int(bspline.drone_id),
            "command_hz": command_hz,
            "candidate_hz": candidate_hz,
            "cloud_messages_before_stop": cloud_count,
            "command_fault": self._diagnostic_values(command_fault_diag),
            "state_fault": self._diagnostic_values(state_fault_diag),
            "bridge_healthy_without_cloud": healthy_without_cloud,
            "canonical_bridge_publisher": False,
            "canonical_bridge_subscriber": False,
            "limitations": [
                "synthetic ideal state following; no dynamics or flight",
                "bridge health does not include pointcloud freshness",
            ],
        }
        path = rospy.get_param("~evidence_path")
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w", encoding="utf-8") as stream:
            json.dump(evidence, stream, indent=2, sort_keys=True)

    @staticmethod
    def _diagnostic_values(message):
        if message is None or not message.status:
            return {}
        return {item.key: item.value for item in message.status[0].values}


if __name__ == "__main__":
    rospy.init_node("single_chain_test")
    rostest.rosrun(
        "xd_uav_planning", "single_chain_test", SingleChainTest)
