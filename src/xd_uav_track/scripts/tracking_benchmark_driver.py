#!/usr/bin/env python3
"""Drive the fixed-camera Gazebo benchmark tanks and truth feed."""

import json
import math
import os
import sys

import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import Pose, TransformStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import tf2_ros
import tf.transformations as transformations
from visualization_msgs.msg import Marker, MarkerArray

# catkin's devel relay sets __file__ to this source script, while an installed
# package resolves the ordinary sibling module from its lib directory.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from benchmark_trajectories import alpha as alpha_trajectory
from benchmark_trajectories import bravo as bravo_trajectory
from benchmark_trajectories import heading as trajectory_heading


WORLD_FRAME = "world"
GROUND_Z = 0.0


def _quaternion_from_pitch_yaw(pitch, yaw):
    """Return Rz(yaw) * Ry(pitch), matching Gazebo's camera forward +X."""
    half_pitch = 0.5 * pitch
    half_yaw = 0.5 * yaw
    sy, cy = math.sin(half_pitch), math.cos(half_pitch)
    sz, cz = math.sin(half_yaw), math.cos(half_yaw)
    return (-sz * sy, cz * sy, sz * cy, cz * cy)


class BenchmarkDriver:
    """Owns scene timing, tank poses, and their public truth records."""

    def __init__(self):
        self.rate_hz = float(rospy.get_param("~rate_hz", 20.0))
        if self.rate_hz < 2.0 or self.rate_hz > 60.0:
            raise ValueError("~rate_hz must be in [2, 60]")
        self.benchmark_namespace = str(rospy.get_param(
            "~benchmark_namespace", "tracking_benchmark")).strip("/")
        self.start_time = None
        self.last_time = None
        self.previous_loop_time = None
        self.last_yaw = {}
        self.state_client = rospy.ServiceProxy("/gazebo/set_model_state",
                                               SetModelState, persistent=True)
        self.tf_static_broadcaster = tf2_ros.StaticTransformBroadcaster()
        self._publish_static_frames()
        self.marker_pub = rospy.Publisher("/tracking_benchmark/ground_truth/markers",
                                          MarkerArray, queue_size=2)
        self.catalog_pub = rospy.Publisher("/tracking_benchmark/ground_truth/catalog",
                                           String, queue_size=1, latch=True)
        self.state_pub = rospy.Publisher("/tracking_benchmark/ground_truth/state_json",
                                         String, queue_size=2)
        self.phase_pub = rospy.Publisher("/tracking_benchmark/scenario_phase",
                                         String, queue_size=1, latch=True)
        self.vehicle_state_pub = rospy.Publisher(
            "/{}/state_estimator/benchmark/odom".format(
                self.benchmark_namespace), Odometry, queue_size=1)
        self.targets = (
            {"id": 101, "model": "benchmark_tank_alpha",
             "class": "tank", "trajectory": alpha_trajectory},
            {"id": 102, "model": "benchmark_tank_bravo",
             "class": "tank", "trajectory": bravo_trajectory},
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
            "scene_profile": "fixed_camera_two_tanks_no_occluders",
            "trajectory_model": "parallel_quintic_courses_16m_spacing",
            "speed_limit_mps": 3.6,
        }, sort_keys=True)))

    @staticmethod
    def _state(model_name, x, y, yaw, z=GROUND_Z, vx=0.0, vy=0.0):
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
        state.twist.linear.x = vx
        state.twist.linear.y = vy
        return state

    @staticmethod
    def _make_transform(parent, child, stamp, translation, quaternion):
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = parent
        transform.child_frame_id = child
        transform.transform.translation.x = translation[0]
        transform.transform.translation.y = translation[1]
        transform.transform.translation.z = translation[2]
        transform.transform.rotation.x = quaternion[0]
        transform.transform.rotation.y = quaternion[1]
        transform.transform.rotation.z = quaternion[2]
        transform.transform.rotation.w = quaternion[3]
        return transform

    def _publish_static_frames(self):
        fixed_link_q = transformations.quaternion_from_euler(
            0.0, math.radians(50.0), math.pi / 2.0)
        link_optical_q = transformations.quaternion_from_euler(
            -math.pi / 2.0, 0.0, -math.pi / 2.0)
        fixed_optical_q = transformations.quaternion_multiply(
            fixed_link_q, link_optical_q)
        stamp = rospy.Time(0)
        self.tf_static_broadcaster.sendTransform([
            self._make_transform(
                "world", "tracking_benchmark/metric_origin", stamp,
                (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
            self._make_transform(
                "world", "tracking_benchmark/fixed_camera_optical_frame", stamp,
                (0.0, -27.0, 29.0), fixed_optical_q),
        ])

    def _rate_limited_yaw(self, target, elapsed, dt):
        desired = trajectory_heading(target["trajectory"], elapsed,
                                     self.last_yaw.get(target["id"], 0.0))
        previous = self.last_yaw.get(target["id"])
        if previous is None:
            result = desired
        else:
            delta = math.atan2(math.sin(desired - previous),
                               math.cos(desired - previous))
            maximum_delta = 1.2 * max(0.0, min(0.2, dt))
            result = previous + max(-maximum_delta,
                                    min(maximum_delta, delta))
        self.last_yaw[target["id"]] = result
        return result

    @staticmethod
    def _velocity(trajectory, elapsed):
        half_step = 0.01
        before = trajectory(elapsed - half_step)
        after = trajectory(elapsed + half_step)
        return ((after[0] - before[0]) / (2.0 * half_step),
                (after[1] - before[1]) / (2.0 * half_step))

    def _publish_vehicle_state(self, stamp):
        odometry = Odometry()
        odometry.header.stamp = stamp
        odometry.header.frame_id = "world"
        odometry.child_frame_id = "tracking_benchmark/metric_origin"
        odometry.pose.pose.orientation.w = 1.0
        odometry.pose.covariance[0] = 1e-6
        odometry.pose.covariance[7] = 1e-6
        odometry.pose.covariance[14] = 1e-6
        odometry.pose.covariance[21] = 1e-6
        odometry.pose.covariance[28] = 1e-6
        odometry.pose.covariance[35] = 1e-6
        self.vehicle_state_pub.publish(odometry)

    def _publish_truth(self, stamp, elapsed, evaluated):
        markers = MarkerArray()
        rows = []
        for target, point in zip(self.targets, evaluated):
            x, y, phase, expected_occluded = point
            yaw = self.last_yaw.get(target["id"],
                                    trajectory_heading(target["trajectory"], elapsed))
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
            world_vx, world_vy = self._velocity(target["trajectory"], elapsed)
            odometry.twist.twist.linear.x = math.cos(yaw) * world_vx + math.sin(yaw) * world_vy
            odometry.twist.twist.linear.y = -math.sin(yaw) * world_vx + math.cos(yaw) * world_vy
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
            marker.color.r = 0.95
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
                self.last_yaw.clear()
                self.previous_loop_time = now
            self.last_time = now
            elapsed = now - self.start_time
            dt = max(0.0, min(0.2, now - self.previous_loop_time))
            self.previous_loop_time = now
            evaluated = [target["trajectory"](elapsed) for target in self.targets]
            try:
                for target, point in zip(self.targets, evaluated):
                    yaw = self._rate_limited_yaw(target, elapsed, dt)
                    vx, vy = self._velocity(target["trajectory"], elapsed)
                    self.state_client(self._state(target["model"], point[0],
                                                  point[1], yaw, vx=vx, vy=vy))
            except rospy.ServiceException as error:
                rospy.logwarn_throttle(2.0, "Gazebo state update failed: %s", error)
            self._publish_truth(stamp, elapsed, evaluated)
            self._publish_vehicle_state(stamp)
            try:
                rate.sleep()
            except rospy.exceptions.ROSTimeMovedBackwardsException:
                self.start_time = None
                self.previous_loop_time = None
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
