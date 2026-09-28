#!/usr/bin/env python3
"""Drive the low-load Gazebo visual tracking benchmark and publish its truth.

This is a test-scene coordinator, not a detector or a flight controller.  It
sets three visual target poses through Gazebo, continuously publishes their
public identities and ground-truth odometry, and points a side camera at the
primary tank.  The south-facing fixed camera is physically occluded by solid
walls during the documented 1/3/5/10 second hold intervals.
"""

import json
import math

import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


WORLD_FRAME = "world"
GROUND_Z = 0.0


def _smoothstep(value):
    value = max(0.0, min(1.0, value))
    return value * value * (3.0 - 2.0 * value)


def _mix(first, second, amount):
    amount = _smoothstep(amount)
    return (first[0] + (second[0] - first[0]) * amount,
            first[1] + (second[1] - first[1]) * amount)


def _quaternion_from_pitch_yaw(pitch, yaw):
    """Return Rz(yaw) * Ry(pitch), matching Gazebo's camera forward +X."""
    half_pitch = 0.5 * pitch
    half_yaw = 0.5 * yaw
    sy, cy = math.sin(half_pitch), math.cos(half_pitch)
    sz, cz = math.sin(half_yaw), math.cos(half_yaw)
    return (-sz * sy, cz * sy, sz * cy, cz * cy)


class BenchmarkDriver:
    """Owns scene timing, target truth, and the simulated gimbal attitude."""

    def __init__(self):
        self.rate_hz = float(rospy.get_param("~rate_hz", 20.0))
        if self.rate_hz < 2.0 or self.rate_hz > 60.0:
            raise ValueError("~rate_hz must be in [2, 60]")
        self.gimbal_anchor = tuple(rospy.get_param(
            "~gimbal_anchor", [25.0, 0.0, 20.0]))
        if len(self.gimbal_anchor) != 3:
            raise ValueError("~gimbal_anchor must contain [x, y, z]")
        self.gimbal_target_id = int(rospy.get_param("~gimbal_target_id", 101))
        # ModelState is intentionally used for deterministic benchmark paths;
        # it bypasses Gazebo contact resolution. Maintain visual clearance in
        # the driver so close targets test association rather than artefacts
        # where two solid models occupy the same space.
        self.minimum_tank_clearance_m = float(rospy.get_param(
            "~minimum_tank_clearance_m", 12.0))
        self.minimum_decoy_clearance_m = float(rospy.get_param(
            "~minimum_decoy_clearance_m", 14.0))
        if self.minimum_tank_clearance_m <= 0.0 or self.minimum_decoy_clearance_m <= 0.0:
            raise ValueError("visual-clearance distances must be positive")
        self.start_time = None
        self.last_time = None
        self.state_client = rospy.ServiceProxy("/gazebo/set_model_state",
                                               SetModelState, persistent=True)
        self.marker_pub = rospy.Publisher("/tracking_benchmark/ground_truth/markers",
                                          MarkerArray, queue_size=2)
        self.catalog_pub = rospy.Publisher("/tracking_benchmark/ground_truth/catalog",
                                           String, queue_size=1, latch=True)
        self.state_pub = rospy.Publisher("/tracking_benchmark/ground_truth/state_json",
                                         String, queue_size=2)
        self.phase_pub = rospy.Publisher("/tracking_benchmark/scenario_phase",
                                         String, queue_size=1, latch=True)
        self.targets = (
            {"id": 101, "model": "benchmark_tank_alpha", "class": "tank",
             "trajectory": self._alpha},
            {"id": 102, "model": "benchmark_tank_bravo", "class": "tank",
             "trajectory": self._bravo},
            {"id": 201, "model": "benchmark_armored_decoy",
             "class": "armored_decoy", "trajectory": self._decoy},
        )
        self.odom_pubs = {
            target["id"]: rospy.Publisher(
                "/tracking_benchmark/ground_truth/{}/odom".format(target["model"]),
                Odometry, queue_size=2)
            for target in self.targets
        }
        self.catalog_pub.publish(String(data=json.dumps({
            "schema_version": 1,
            "frame_id": WORLD_FRAME,
            "targets": [{"public_id": target["id"], "model": target["model"],
                         "class": target["class"]} for target in self.targets],
            "fixed_camera": "/tracking_benchmark/fixed_camera/image_raw",
            "gimbal_camera": "/tracking_benchmark/gimbal_camera/image_raw",
            "fixed_camera_occlusion_holds_sec": [1, 3, 5, 10],
        }, sort_keys=True)))

    @staticmethod
    def _alpha(time_sec):
        """64 s loop: circle, line, sharp turn, stop/restart, 1/3/5/10 s holds."""
        time_sec %= 64.0
        if time_sec < 12.0:
            angle = 2.0 * math.pi * time_sec / 12.0
            return (-13.0 + 4.5 * math.cos(angle),
                    -11.0 + 4.5 * math.sin(angle), "circle", False)
        if time_sec < 16.0:
            x, y = _mix((-8.5, -11.0), (-12.0, -4.0), (time_sec - 12.0) / 4.0)
            return x, y, "straight_line", False
        if time_sec < 17.0:
            x, y = _mix((-12.0, -4.0), (-12.0, 5.0), time_sec - 16.0)
            return x, y, "enter_occluder_1s", False
        if time_sec < 18.0:
            return -12.0, 5.0, "fixed_camera_occlusion_1s", True
        if time_sec < 19.0:
            x, y = _mix((-12.0, 5.0), (-12.0, -4.0), time_sec - 18.0)
            return x, y, "leave_occluder_1s", False
        if time_sec < 21.0:
            x, y = _mix((-12.0, -4.0), (-4.0, -4.0), (time_sec - 19.0) / 2.0)
            return x, y, "transit_to_occluder_3s", False
        if time_sec < 22.0:
            x, y = _mix((-4.0, -4.0), (-4.0, 5.0), time_sec - 21.0)
            return x, y, "enter_occluder_3s", False
        if time_sec < 25.0:
            return -4.0, 5.0, "fixed_camera_occlusion_3s", True
        if time_sec < 26.0:
            x, y = _mix((-4.0, 5.0), (-4.0, -4.0), time_sec - 25.0)
            return x, y, "leave_occluder_3s", False
        if time_sec < 28.0:
            x, y = _mix((-4.0, -4.0), (4.0, -4.0), (time_sec - 26.0) / 2.0)
            return x, y, "transit_to_occluder_5s", False
        if time_sec < 29.0:
            x, y = _mix((4.0, -4.0), (4.0, 5.0), time_sec - 28.0)
            return x, y, "enter_occluder_5s", False
        if time_sec < 34.0:
            return 4.0, 5.0, "fixed_camera_occlusion_5s", True
        if time_sec < 35.0:
            x, y = _mix((4.0, 5.0), (4.0, -4.0), time_sec - 34.0)
            return x, y, "leave_occluder_5s", False
        if time_sec < 37.0:
            x, y = _mix((4.0, -4.0), (13.0, -4.0), (time_sec - 35.0) / 2.0)
            return x, y, "transit_to_occluder_10s", False
        if time_sec < 38.0:
            x, y = _mix((13.0, -4.0), (13.0, 5.0), time_sec - 37.0)
            return x, y, "enter_occluder_10s", False
        if time_sec < 48.0:
            return 13.0, 5.0, "fixed_camera_occlusion_10s", True
        if time_sec < 49.0:
            x, y = _mix((13.0, 5.0), (13.0, -4.0), time_sec - 48.0)
            return x, y, "leave_occluder_10s", False
        if time_sec < 54.0:
            return 13.0, -4.0, "stop", False
        if time_sec < 55.5:
            x, y = _mix((13.0, -4.0), (13.0, -12.0), (time_sec - 54.0) / 1.5)
            return x, y, "sharp_turn_leg_1", False
        if time_sec < 57.0:
            x, y = _mix((13.0, -12.0), (7.0, -12.0), (time_sec - 55.5) / 1.5)
            return x, y, "sharp_turn_leg_2", False
        x, y = _mix((7.0, -12.0), (-8.5, -11.0), (time_sec - 57.0) / 7.0)
        return x, y, "return_to_circle", False

    @staticmethod
    def _bravo(time_sec):
        angle = 2.0 * math.pi * (time_sec % 28.0) / 28.0
        return (-3.0 + 9.5 * math.sin(angle),
                -9.0 + 4.2 * math.sin(2.0 * angle),
                "similar_tank_figure_eight", False)

    @staticmethod
    def _decoy(time_sec):
        time_sec %= 36.0
        if time_sec < 8.0:
            x, y = _mix((20.0, -14.0), (7.0, -14.0), time_sec / 8.0)
            return x, y, "decoy_approach", False
        if time_sec < 12.0:
            return 7.0, -14.0, "decoy_stop", False
        if time_sec < 16.0:
            x, y = _mix((7.0, -14.0), (7.0, -7.0), (time_sec - 12.0) / 4.0)
            return x, y, "decoy_turn", False
        if time_sec < 24.0:
            x, y = _mix((7.0, -7.0), (20.0, -7.0), (time_sec - 16.0) / 8.0)
            return x, y, "decoy_depart", False
        if time_sec < 28.0:
            return 20.0, -7.0, "decoy_stop", False
        x, y = _mix((20.0, -7.0), (20.0, -14.0), (time_sec - 28.0) / 8.0)
        return x, y, "decoy_return", False

    @staticmethod
    def _state(model_name, x, y, yaw, z=GROUND_Z):
        state = ModelState()
        state.model_name = model_name
        state.reference_frame = WORLD_FRAME
        state.pose = Pose()
        state.pose.position.x = x
        state.pose.position.y = y
        state.pose.position.z = z
        qx, qy, qz, qw = _quaternion_from_pitch_yaw(0.0, yaw)
        state.pose.orientation.x = qx
        state.pose.orientation.y = qy
        state.pose.orientation.z = qz
        state.pose.orientation.w = qw
        state.twist = Twist()
        return state

    @staticmethod
    def _heading(trajectory, time_sec, fallback=0.0):
        first = trajectory(time_sec)
        second = trajectory(time_sec + 0.05)
        dx, dy = second[0] - first[0], second[1] - first[1]
        return math.atan2(dy, dx) if math.hypot(dx, dy) > 0.02 else fallback

    @staticmethod
    def _separate_from(anchor, candidate, clearance, fallback_yaw):
        """Return candidate moved only when its visual envelopes overlap."""
        dx, dy = candidate[0] - anchor[0], candidate[1] - anchor[1]
        distance = math.hypot(dx, dy)
        if distance >= clearance:
            return candidate
        if distance < 1e-6:
            dx, dy = math.cos(fallback_yaw), math.sin(fallback_yaw)
            distance = 1.0
        scale = clearance / distance
        return (anchor[0] + dx * scale, anchor[1] + dy * scale,
                candidate[2], candidate[3])

    def _apply_visual_clearance(self, evaluated, elapsed):
        """Keep driven models apart because direct pose updates skip contacts."""
        alpha, bravo, decoy = evaluated
        alpha_heading = self._heading(self._alpha, elapsed)
        bravo_heading = self._heading(self._bravo, elapsed)
        # A correction away from bravo can otherwise put the decoy back inside
        # alpha's clearance disk.  A tiny bounded projection loop resolves all
        # three pairwise constraints deterministically without a physics step.
        # Sixteen projections are still negligible at the 20 Hz scene rate
        # and converge to millimetre-level separation for the worst crossing.
        for _ in range(16):
            bravo = self._separate_from(alpha, bravo,
                                        self.minimum_tank_clearance_m,
                                        alpha_heading + 0.5 * math.pi)
            decoy = self._separate_from(alpha, decoy,
                                        self.minimum_decoy_clearance_m,
                                        alpha_heading - 0.5 * math.pi)
            decoy = self._separate_from(bravo, decoy,
                                        self.minimum_decoy_clearance_m,
                                        bravo_heading - 0.5 * math.pi)
        return [alpha, bravo, decoy]

    def _publish_truth(self, stamp, elapsed, evaluated):
        markers = MarkerArray()
        rows = []
        for target, point in zip(self.targets, evaluated):
            x, y, phase, expected_occluded = point
            yaw = self._heading(target["trajectory"], elapsed)
            odometry = Odometry()
            odometry.header.stamp = stamp
            odometry.header.frame_id = WORLD_FRAME
            odometry.child_frame_id = target["model"] + "/base_link"
            odometry.pose.pose.position.x = x
            odometry.pose.pose.position.y = y
            odometry.pose.pose.position.z = GROUND_Z
            qx, qy, qz, qw = _quaternion_from_pitch_yaw(0.0, yaw)
            odometry.pose.pose.orientation.x = qx
            odometry.pose.pose.orientation.y = qy
            odometry.pose.pose.orientation.z = qz
            odometry.pose.pose.orientation.w = qw
            self.odom_pubs[target["id"]].publish(odometry)

            marker = Marker()
            marker.header.stamp = stamp
            marker.header.frame_id = WORLD_FRAME
            marker.ns = "tracking_benchmark_target_id"
            marker.id = target["id"]
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 4.0
            marker.pose.orientation.w = 1.0
            marker.scale.z = 1.0
            marker.color.r = 1.0 if target["id"] == self.gimbal_target_id else 0.85
            marker.color.g = 0.92
            marker.color.b = 0.20
            marker.color.a = 1.0
            marker.text = "ID {} ({})".format(target["id"], target["class"])
            markers.markers.append(marker)
            rows.append({"public_id": target["id"], "model": target["model"],
                         "class": target["class"], "position_world": [x, y, GROUND_Z],
                         "yaw_rad": yaw, "phase": phase,
                         "expected_fixed_camera_occluded": expected_occluded})
        self.marker_pub.publish(markers)
        self.state_pub.publish(String(data=json.dumps({
            "stamp": stamp.to_sec(), "elapsed_sec": elapsed, "targets": rows,
        }, sort_keys=True)))
        self.phase_pub.publish(String(data=evaluated[0][2]))

    def _point_gimbal(self, alpha_point):
        dx = alpha_point[0] - self.gimbal_anchor[0]
        dy = alpha_point[1] - self.gimbal_anchor[1]
        dz = alpha_point[2] - self.gimbal_anchor[2]
        horizontal = math.hypot(dx, dy)
        yaw = math.atan2(dy, dx)
        pitch = math.atan2(-dz, max(horizontal, 1e-6))
        state = self._state("benchmark_gimbal_camera", self.gimbal_anchor[0],
                            self.gimbal_anchor[1], yaw, self.gimbal_anchor[2])
        qx, qy, qz, qw = _quaternion_from_pitch_yaw(pitch, yaw)
        state.pose.orientation.x = qx
        state.pose.orientation.y = qy
        state.pose.orientation.z = qz
        state.pose.orientation.w = qw
        self.state_client(state)

    def run(self):
        while not rospy.is_shutdown():
            try:
                rospy.wait_for_service("/gazebo/set_model_state", timeout=2.0)
                break
            except rospy.ROSException:
                rospy.loginfo_throttle(5.0, "waiting for Gazebo set_model_state service")
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            stamp = rospy.Time.now()
            if stamp.is_zero():
                rate.sleep()
                continue
            now = stamp.to_sec()
            if self.start_time is None or (self.last_time is not None and now < self.last_time):
                self.start_time = now
            self.last_time = now
            elapsed = now - self.start_time
            evaluated = self._apply_visual_clearance(
                [target["trajectory"](elapsed) for target in self.targets], elapsed)
            try:
                for target, point in zip(self.targets, evaluated):
                    yaw = self._heading(target["trajectory"], elapsed)
                    self.state_client(self._state(target["model"], point[0], point[1], yaw))
                self._point_gimbal((evaluated[0][0], evaluated[0][1], 1.6))
            except rospy.ServiceException as error:
                rospy.logwarn_throttle(2.0, "Gazebo state update failed: %s", error)
            self._publish_truth(stamp, elapsed, evaluated)
            try:
                rate.sleep()
            except rospy.exceptions.ROSTimeMovedBackwardsException:
                self.start_time = None
                rate = rospy.Rate(self.rate_hz)
            except rospy.ROSInterruptException:
                # roslaunch is deliberately responsible for the Gazebo/UI
                # lifecycle; a normal shutdown is not a benchmark failure.
                break


def main():
    rospy.init_node("tracking_benchmark_driver")
    try:
        BenchmarkDriver().run()
    except ValueError as error:
        rospy.logfatal("invalid tracking benchmark configuration: %s", error)
        raise SystemExit(2)


if __name__ == "__main__":
    main()
