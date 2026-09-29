#!/usr/bin/env python3
"""Low-load moving-UAV search/follow scenario; control uses tracker output only."""

import json
import math
import threading

import rospy
import tf.transformations as transformations
import tf2_ros
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import Pose, TransformStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from xd_uav_track.msg import TrackStateArray


WORLD = "world"
BODY = "moving_uav_search/base_link"
CAMERA = "moving_uav_search/fixed_camera_optical_frame"
ALTITUDE_M = 28.0
GROUND_Z_M = 0.0


def yaw_quaternion(yaw):
    return transformations.quaternion_from_euler(0.0, 0.0, yaw)


def clamp_vector(x, y, maximum):
    magnitude = math.hypot(x, y)
    if magnitude > maximum > 0.0:
        scale = maximum / magnitude
        return x * scale, y * scale
    return x, y


class MovingSearchDriver:
    """Moves vehicles and a kinematic quadrotor, then follows visual metrics."""

    def __init__(self):
        self.rate_hz = float(rospy.get_param("~rate_hz", 20.0))
        self.altitude_m = float(rospy.get_param("~altitude_m", ALTITUDE_M))
        self.minimum_altitude_m = float(rospy.get_param("~minimum_altitude_m", 28.0))
        self.maximum_altitude_m = float(rospy.get_param("~maximum_altitude_m", 60.0))
        self.altitude_rate_mps = max(0.1, float(rospy.get_param("~altitude_rate_mps", 1.5)))
        self.expected_target_count = max(0, int(rospy.get_param("~expected_target_count", 0)))
        self.coverage_margin_m = max(0.0, float(rospy.get_param("~coverage_margin_m", 2.0)))
        self.camera_horizontal_fov_rad = float(
            rospy.get_param("~camera_horizontal_fov_rad", 1.22))
        self.camera_image_width = max(1, int(rospy.get_param("~camera_image_width", 640)))
        self.camera_image_height = max(1, int(rospy.get_param("~camera_image_height", 480)))
        self.search_cruise_speed_mps = max(0.2, float(rospy.get_param(
            "~search_cruise_speed_mps", 1.4)))
        self.coverage_transit_speed_mps = max(0.2, float(rospy.get_param(
            "~coverage_transit_speed_mps", 1.6)))
        self.follow_max_speed_mps = float(rospy.get_param(
            "~follow_max_speed_mps", 2.6))
        self.horizontal_acceleration_mps2 = float(rospy.get_param(
            "~horizontal_acceleration_mps2", 1.2))
        self.yaw_rate_limit_radps = float(rospy.get_param(
            "~yaw_rate_limit_radps", 0.4))
        self.follow_fov_margin = float(rospy.get_param(
            "~follow_fov_margin", 0.72))
        self.follow_preserve_other_targets = bool(rospy.get_param(
            "~follow_preserve_other_targets", True))
        self.follow_overview_altitude_m = float(rospy.get_param(
            "~follow_overview_altitude_m", 58.0))
        self.follow_overview_margin_m = float(rospy.get_param(
            "~follow_overview_margin_m", 2.0))
        self.follow_overview_fraction = float(rospy.get_param(
            "~follow_overview_fraction", 0.35))
        self.local_reacquire_sec = float(rospy.get_param(
            "~local_reacquire_sec", 12.0))
        self.local_reacquire_altitude_m = float(rospy.get_param(
            "~local_reacquire_altitude_m", 40.0))
        self.search_area_bounds = (
            float(rospy.get_param("~search_area/min_x_m", -34.0)),
            float(rospy.get_param("~search_area/max_x_m", 34.0)),
            float(rospy.get_param("~search_area/min_y_m", -18.0)),
            float(rospy.get_param("~search_area/max_y_m", 18.0)))
        if (not all(math.isfinite(value) for value in (
                self.altitude_m, self.minimum_altitude_m, self.maximum_altitude_m,
                self.altitude_rate_mps, self.coverage_margin_m,
                self.camera_horizontal_fov_rad, self.search_cruise_speed_mps,
                self.coverage_transit_speed_mps, self.follow_max_speed_mps,
                self.horizontal_acceleration_mps2, self.yaw_rate_limit_radps,
                self.follow_fov_margin, self.local_reacquire_sec,
                self.local_reacquire_altitude_m,
                self.follow_overview_altitude_m,
                self.follow_overview_margin_m,
                self.follow_overview_fraction, *self.search_area_bounds)) or
                self.minimum_altitude_m <= 0.0 or
                self.maximum_altitude_m < self.minimum_altitude_m or
                self.search_area_bounds[1] <= self.search_area_bounds[0] or
                self.search_area_bounds[3] <= self.search_area_bounds[2] or
                not 0.1 < self.camera_horizontal_fov_rad < 2.8 or
                self.follow_max_speed_mps <= 0.0 or
                self.horizontal_acceleration_mps2 <= 0.0 or
                self.yaw_rate_limit_radps <= 0.0 or
                not 0.3 <= self.follow_fov_margin <= 0.9 or
                self.local_reacquire_sec < 0.0 or
                self.follow_overview_altitude_m <= 0.0 or
                self.follow_overview_margin_m < 0.0 or
                not 0.0 <= self.follow_overview_fraction <= 1.0):
            raise ValueError("invalid global-view altitude/FOV/search-area configuration")
        self.altitude_m = max(self.minimum_altitude_m,
                              min(self.maximum_altitude_m, self.altitude_m))
        self.altitude_target_m = self.altitude_m
        self.local_reacquire_altitude_m = max(
            self.minimum_altitude_m,
            min(self.maximum_altitude_m, self.local_reacquire_altitude_m))
        self.follow_overview_altitude_m = max(
            self.minimum_altitude_m,
            min(self.maximum_altitude_m, self.follow_overview_altitude_m))
        self.uav_name = "moving_uav_search"
        self.drone_model = str(rospy.get_param("~drone_model", "search_quadrotor"))
        self.drone_xy = [-34.0, -21.0]
        self.drone_velocity = [0.0, 0.0]
        self.drone_yaw = 0.0
        self.search_progress = 0.0
        self.search_direction = 1.0
        self.search_path, self.search_cumulative, self.search_length = self._make_search_path()
        self.start_time = None
        self.last_loop_time = None
        self.last_stamp = rospy.Time()
        self.loop_count = 0
        self.pose_history = []
        self.pose_lock = threading.Lock()
        self.known_targets = {}
        self.target_state_lock = threading.Lock()
        self.target_memory_sec = max(2.0, float(rospy.get_param(
            "~target_memory_sec", 15.0)))
        self.follow_loss_coast_sec = max(0.0, float(rospy.get_param(
            "~follow_loss_coast_sec", 5.0)))

        self.vehicles = (
            {"id": 201, "name": "search_vehicle_alpha", "class": "vehicle",
             "lane_y": -16.0, "speed": 1.25, "phase_m": 0.0},
            {"id": 202, "name": "search_vehicle_bravo", "class": "vehicle",
             "lane_y": 0.0, "speed": 1.45, "phase_m": 60.0 + 2.0 * math.pi},
            {"id": 203, "name": "search_vehicle_charlie", "class": "vehicle",
             "lane_y": 16.0, "speed": 1.10, "phase_m": 35.0},
        )

        self.follow_track_id = None
        self.selection_mode = "all"
        self.manual_selection_active = False
        self.follow_mode_started_at = rospy.Time()
        self.reacquire_route_active = False
        self.last_visual_measurement = rospy.Time()
        self.target_world = None
        self.target_velocity_world = [0.0, 0.0]
        self.previous_target_world = None
        self.previous_target_stamp = rospy.Time()
        self.phase = "SEARCH"
        self.state_client = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState,
                                               persistent=True)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.tf_static_broadcaster = tf2_ros.StaticTransformBroadcaster()
        self._publish_static_camera_transform()

        ns = "/" + self.uav_name
        self.odom_pub = rospy.Publisher(ns + "/state_estimator/main/odom",
                                        Odometry, queue_size=1)
        self.truth_json_pub = rospy.Publisher(ns + "/ground_truth/state_json",
                                              String, queue_size=1)
        self.truth_catalog_pub = rospy.Publisher(ns + "/ground_truth/catalog",
                                                 String, queue_size=1, latch=True)
        self.truth_marker_pub = rospy.Publisher(ns + "/ground_truth/markers",
                                                MarkerArray, queue_size=1)
        self.phase_pub = rospy.Publisher(ns + "/scenario/phase", String,
                                         queue_size=1, latch=True)
        self.phase_pub.publish(String(data=self.phase))
        self.truth_catalog_pub.publish(String(data=json.dumps({
            "schema_version": 1,
            "world_frame": WORLD,
            "scene": "moving_uav_vehicle_search",
            "camera": "/{}/fixed_camera/image_raw".format(self.uav_name),
            "search_route": "four-lane rounded lawnmower",
            "uav_control_source": "xd_uav_track TrackStateArray only; no Gazebo truth",
            "vehicles": [{"public_id": v["id"], "model": v["name"],
                          "class": v["class"], "lane_y_m": v["lane_y"],
                          "speed_mps": v["speed"]} for v in self.vehicles],
        }, sort_keys=True)))
        self.track_sub = rospy.Subscriber(ns + "/track/tracks_by_source",
                                          TrackStateArray, self._track_callback,
                                          queue_size=2, tcp_nodelay=True)
        self.target_mode_sub = rospy.Subscriber(ns + "/track/target_mode", String,
                                                self._target_mode_callback,
                                                queue_size=1)

    @staticmethod
    def _make_search_path():
        """Build a lawnmower route with semicircular, tangent U-turns."""
        x_left, x_right = -34.0, 34.0
        lanes = [-21.0, -7.0, 7.0, 21.0]
        points = [(x_left, lanes[0])]
        for lane_index, y in enumerate(lanes):
            eastbound = lane_index % 2 == 0
            end_x = x_right if eastbound else x_left
            points.append((end_x, y))
            if lane_index + 1 >= len(lanes):
                continue
            next_y = lanes[lane_index + 1]
            radius = abs(next_y - y) * 0.5
            center_y = 0.5 * (y + next_y)
            center_x = end_x
            # All lanes progress from low to high y. At the right endpoint a
            # counter-clockwise half-circle gives the westbound tangent; at
            # the left endpoint the clockwise half-circle gives the eastbound
            # tangent. Both start at the current lane's low-y endpoint.
            start_angle = -0.5 * math.pi
            direction = 1.0 if eastbound else -1.0
            for sample in range(1, 33):
                angle = start_angle + direction * math.pi * sample / 32.0
                points.append((center_x + radius * math.cos(angle),
                               center_y + radius * math.sin(angle)))

        cumulative = [0.0]
        for first, second in zip(points, points[1:]):
            cumulative.append(cumulative[-1] + math.hypot(second[0] - first[0],
                                                          second[1] - first[1]))
        return points, cumulative, cumulative[-1]

    def _publish_static_camera_transform(self):
        # Optical x=right, y=down, z=forward. Camera forward points down (-body z).
        rotation = transformations.identity_matrix()
        rotation[0:3, 0:3] = ((0.0, -1.0, 0.0),
                              (-1.0, 0.0, 0.0),
                              (0.0, 0.0, -1.0))
        q = transformations.quaternion_from_matrix(rotation)
        transform = TransformStamped()
        transform.header.stamp = rospy.Time(0)
        transform.header.frame_id = BODY
        transform.child_frame_id = CAMERA
        transform.transform.translation.x = 0.10
        transform.transform.translation.z = -0.16
        transform.transform.rotation.x = q[0]
        transform.transform.rotation.y = q[1]
        transform.transform.rotation.z = q[2]
        transform.transform.rotation.w = q[3]
        self.tf_static_broadcaster.sendTransform(transform)

    def _path_at(self, distance):
        distance = max(0.0, min(self.search_length, distance))
        for index in range(len(self.search_cumulative) - 1):
            start_d = self.search_cumulative[index]
            end_d = self.search_cumulative[index + 1]
            if distance <= end_d or index == len(self.search_cumulative) - 2:
                segment_length = max(1e-9, end_d - start_d)
                ratio = max(0.0, min(1.0, (distance - start_d) / segment_length))
                a, b = self.search_path[index], self.search_path[index + 1]
                x = a[0] + ratio * (b[0] - a[0])
                y = a[1] + ratio * (b[1] - a[1])
                yaw = math.atan2(b[1] - a[1], b[0] - a[0])
                return x, y, yaw
        return self.search_path[-1][0], self.search_path[-1][1], 0.0

    def _closest_search_progress(self, x, y):
        best_distance_sq = float("inf")
        best_progress = 0.0
        for index in range(len(self.search_path) - 1):
            start = self.search_path[index]
            end = self.search_path[index + 1]
            dx, dy = end[0] - start[0], end[1] - start[1]
            length_sq = dx * dx + dy * dy
            if length_sq <= 1e-9:
                continue
            ratio = max(0.0, min(1.0, ((x - start[0]) * dx +
                                      (y - start[1]) * dy) / length_sq))
            px, py = start[0] + ratio * dx, start[1] + ratio * dy
            distance_sq = (x - px) ** 2 + (y - py) ** 2
            if distance_sq < best_distance_sq:
                best_distance_sq = distance_sq
                best_progress = self.search_cumulative[index] + math.sqrt(length_sq) * ratio
        return best_progress

    @staticmethod
    def _vehicle_state(vehicle, elapsed):
        half_length = 30.0
        turn_radius = 2.0
        speed = vehicle["speed"]
        y_center = vehicle["lane_y"]
        lower_straight = 2.0 * half_length
        turn_length = math.pi * turn_radius
        upper_start = lower_straight + turn_length
        left_turn_start = upper_start + lower_straight
        perimeter = left_turn_start + turn_length
        distance = (speed * elapsed + vehicle["phase_m"]) % perimeter

        if distance < lower_straight:
            x = -half_length + distance
            y = y_center - turn_radius
            vx, vy = speed, 0.0
        elif distance < upper_start:
            angle = -0.5 * math.pi + (distance - lower_straight) / turn_radius
            x = half_length + turn_radius * math.cos(angle)
            y = y_center + turn_radius * math.sin(angle)
            vx = -speed * math.sin(angle)
            vy = speed * math.cos(angle)
        elif distance < left_turn_start:
            along = distance - upper_start
            x = half_length - along
            y = y_center + turn_radius
            vx, vy = -speed, 0.0
        else:
            angle = 0.5 * math.pi + (distance - left_turn_start) / turn_radius
            x = -half_length + turn_radius * math.cos(angle)
            y = y_center + turn_radius * math.sin(angle)
            vx = -speed * math.sin(angle)
            vy = speed * math.cos(angle)
        yaw = math.atan2(vy, vx)
        return x, y, vx, vy, yaw

    def _pose_at(self, stamp):
        with self.pose_lock:
            if not self.pose_history:
                return None
            candidate = min(self.pose_history,
                            key=lambda item: abs((item[0] - stamp).to_sec()))
            if abs((candidate[0] - stamp).to_sec()) > 0.30:
                return None
            return candidate[1], candidate[2], candidate[3]

    def _track_callback(self, message):
        if message.image_source != "search_fixed":
            return
        self._update_known_targets(message)
        selected_tracks = [track for track in message.tracks if track.selected]
        self.manual_selection_active = (self.selection_mode == "follow" or
                                        bool(selected_tracks))
        if not selected_tracks:
            if self.follow_track_id is not None and self.selection_mode != "follow":
                rospy.loginfo("global multi-target mode enabled; clearing flight-follow target")
                self.follow_track_id = None
                self.last_visual_measurement = rospy.Time()
                self.target_world = None
                self.target_velocity_world = [0.0, 0.0]
                self.previous_target_world = None
                self.previous_target_stamp = rospy.Time()
            return
        target_track = selected_tracks[0]
        if target_track.track_id != self.follow_track_id:
            self.follow_track_id = target_track.track_id
            self.follow_mode_started_at = rospy.Time.now()
            self.reacquire_route_active = False
            self.last_visual_measurement = rospy.Time()
            self.target_world = None
            self.target_velocity_world = [0.0, 0.0]
            self.previous_target_world = None
            self.previous_target_stamp = rospy.Time()
            rospy.loginfo("flight-follow selected confirmed track_id=%d",
                          self.follow_track_id)
        if (target_track.lifecycle_state not in ("confirmed", "occluded") or
                not target_track.detected or target_track.predicted or
                not target_track.control_measurement_ready):
            return
        measurement = self._world_measurement(target_track, message)
        if measurement is None:
            return
        stamp, measured = measurement
        age = (rospy.Time.now() - stamp).to_sec()
        if stamp.is_zero() or age < -0.1 or age > 0.8 or (
                not self.previous_target_stamp.is_zero() and
                stamp <= self.previous_target_stamp):
            return
        self.reacquire_route_active = False
        if not stamp.is_zero() and not self.previous_target_stamp.is_zero():
            dt = (stamp - self.previous_target_stamp).to_sec()
            if 0.05 <= dt <= 1.0 and self.previous_target_world is not None:
                raw_vx = (measured[0] - self.previous_target_world[0]) / dt
                raw_vy = (measured[1] - self.previous_target_world[1]) / dt
                raw_vx, raw_vy = clamp_vector(raw_vx, raw_vy, 5.0)
                self.target_velocity_world[0] = 0.65 * self.target_velocity_world[0] + 0.35 * raw_vx
                self.target_velocity_world[1] = 0.65 * self.target_velocity_world[1] + 0.35 * raw_vy
            self.previous_target_world = measured
            self.previous_target_stamp = stamp
        elif not stamp.is_zero():
            self.previous_target_world = measured
            self.previous_target_stamp = stamp
        self.target_world = measured
        self.last_visual_measurement = stamp

    def _target_mode_callback(self, message):
        mode = str(message.data).strip().lower()
        if mode not in ("all", "global", "follow", "auto"):
            rospy.logwarn_throttle(5.0, "ignoring unknown target mode '%s'", mode)
            return
        previous_mode = self.selection_mode
        self.selection_mode = "all" if mode == "global" else mode
        self.manual_selection_active = self.selection_mode == "follow"
        if self.selection_mode == "follow" and previous_mode != "follow":
            self.follow_mode_started_at = rospy.Time.now()
        if self.selection_mode == "all":
            self.follow_track_id = None
            self.follow_mode_started_at = rospy.Time()
            self.reacquire_route_active = False
            self.last_visual_measurement = rospy.Time()
            self.target_world = None
            self.target_velocity_world = [0.0, 0.0]
            self.previous_target_world = None
            self.previous_target_stamp = rospy.Time()
        rospy.loginfo("moving-UAV target mode: %s", self.selection_mode)

    def _move_toward_xy(self, target, dt, maximum_speed, gain=0.8,
                        feedforward=(0.0, 0.0)):
        """Bound horizontal speed and vector acceleration through every mode."""
        if dt <= 0.0:
            return
        desired = clamp_vector(
            gain * (target[0] - self.drone_xy[0]) + feedforward[0],
            gain * (target[1] - self.drone_xy[1]) + feedforward[1],
            maximum_speed)
        change = clamp_vector(
            desired[0] - self.drone_velocity[0],
            desired[1] - self.drone_velocity[1],
            self.horizontal_acceleration_mps2 * dt)
        for axis in range(2):
            self.drone_velocity[axis] += change[axis]
            self.drone_xy[axis] += self.drone_velocity[axis] * dt

    def _vertical_fov_rad(self):
        return 2.0 * math.atan(
            math.tan(0.5 * self.camera_horizontal_fov_rad) *
            self.camera_image_height / float(self.camera_image_width))

    def _follow_altitude_target(self, lookahead_sec=1.0):
        """Keep the predicted vehicle inside the inner fixed-camera FOV."""
        dx = (self.target_world[0] + self.target_velocity_world[0] * lookahead_sec -
              self.drone_xy[0] - self.drone_velocity[0] * lookahead_sec)
        dy = (self.target_world[1] + self.target_velocity_world[1] * lookahead_sec -
              self.drone_xy[1] - self.drone_velocity[1] * lookahead_sec)
        cy, sy = math.cos(self.drone_yaw), math.sin(self.drone_yaw)
        forward = cy * dx + sy * dy
        lateral = -sy * dx + cy * dy
        required = max(
            abs(lateral) /
            (math.tan(0.5 * self.camera_horizontal_fov_rad) * self.follow_fov_margin),
            abs(forward) /
            (math.tan(0.5 * self._vertical_fov_rad()) * self.follow_fov_margin))
        return max(self.minimum_altitude_m,
                   min(self.maximum_altitude_m, required + 2.0))

    def _overview_follow_plan(self, predicted_target, stamp):
        """Bias toward the selected vehicle within a full-area FOV envelope."""
        if not self.follow_preserve_other_targets or self.expected_target_count <= 1:
            return None
        min_x, max_x, min_y, max_y = self.search_area_bounds
        if not (min_x - 2.0 <= predicted_target[0] <= max_x + 2.0 and
                min_y - 2.0 <= predicted_target[1] <= max_y + 2.0):
            return None
        plan = self._global_coverage_plan(stamp)
        if plan is None or not plan[6]:
            return None

        center_x, center_y = plan[:2]
        altitude = max(plan[3], self.follow_overview_altitude_m)
        cy, sy = math.cos(self.drone_yaw), math.sin(self.drone_yaw)
        corners = ((min_x, min_y), (min_x, max_y),
                   (max_x, min_y), (max_x, max_y))
        max_forward = max(abs(cy * (x - center_x) + sy * (y - center_y))
                          for x, y in corners)
        max_lateral = max(abs(-sy * (x - center_x) + cy * (y - center_y))
                          for x, y in corners)
        # Use current altitude while climbing; the vehicle cannot shift into
        # camera coverage that it has not reached yet.
        usable_altitude = min(self.altitude_m, altitude)
        forward_room = max(0.0, usable_altitude *
                           math.tan(0.5 * self._vertical_fov_rad()) -
                           max_forward - self.follow_overview_margin_m)
        lateral_room = max(0.0, usable_altitude *
                           math.tan(0.5 * self.camera_horizontal_fov_rad) -
                           max_lateral - self.follow_overview_margin_m)
        dx = predicted_target[0] - center_x
        dy = predicted_target[1] - center_y
        forward_bias = max(-forward_room, min(forward_room,
            self.follow_overview_fraction * (cy * dx + sy * dy)))
        lateral_bias = max(-lateral_room, min(lateral_room,
            self.follow_overview_fraction * (-sy * dx + cy * dy)))
        return ((center_x + cy * forward_bias - sy * lateral_bias,
                 center_y + sy * forward_bias + cy * lateral_bias), altitude)

    def _advance_search_route(self, dt):
        """Join and traverse a fallback sweep without jumping onto its path."""
        if not self.reacquire_route_active:
            self.search_progress = self._closest_search_progress(
                self.drone_xy[0], self.drone_xy[1])
            self.search_direction = (1.0 if self.search_progress <=
                                     0.5 * self.search_length else -1.0)
            self.reacquire_route_active = True
        x, y, _ = self._path_at(self.search_progress)
        if math.hypot(x - self.drone_xy[0], y - self.drone_xy[1]) <= 1.5:
            self.search_progress += (
                self.search_direction * self.search_cruise_speed_mps * dt)
            if self.search_progress >= self.search_length:
                self.search_progress = self.search_length
                self.search_direction = -1.0
            elif self.search_progress <= 0.0:
                self.search_progress = 0.0
                self.search_direction = 1.0
            x, y, _ = self._path_at(self.search_progress)
        self._move_toward_xy((x, y), dt, self.search_cruise_speed_mps)
        return self.drone_yaw

    def _advance_follow_reacquire_search(self, dt, stamp, loss_age):
        """Search locally first, then use a stable full-area camera viewpoint."""
        if (self.target_world is not None and
                loss_age <= self.follow_loss_coast_sec + self.local_reacquire_sec):
            self.phase = "FOLLOW_LOCAL_REACQUIRE"
            horizon = min(3.0, loss_age)
            anchor = [self.target_world[index] +
                      self.target_velocity_world[index] * horizon
                      for index in range(2)]
            overview = self._overview_follow_plan(anchor, stamp)
            if overview is not None:
                self.altitude_target_m = max(self.altitude_m, overview[1])
                self._move_toward_xy(overview[0], dt,
                                     self.coverage_transit_speed_mps, gain=0.45)
            else:
                self.altitude_target_m = max(self.altitude_m,
                                             self.local_reacquire_altitude_m)
                self._move_toward_xy(anchor, dt, self.search_cruise_speed_mps,
                                     gain=0.55)
            return self.drone_yaw

        plan = self._global_coverage_plan(stamp)
        if plan is not None and plan[6]:
            self.phase = "FOLLOW_REACQUIRE_SEARCH"
            self.reacquire_route_active = False
            return self._move_global_coverage(plan, dt)

        self.phase = "FOLLOW_REACQUIRE_SEARCH"
        self.altitude_target_m = max(self.altitude_m,
                                     self.local_reacquire_altitude_m)
        return self._advance_search_route(dt)

    def _update_known_targets(self, message):
        observations = []
        for track in message.tracks:
            if (track.lifecycle_state not in ("confirmed", "occluded") or
                    not track.detected or track.predicted or track.hit_count < 3):
                continue
            measurement = self._world_measurement(track, message)
            if measurement is not None:
                observations.append((track.track_id, measurement[0], measurement[1],
                                     int(track.hit_count)))
        with self.target_state_lock:
            for track_id, stamp, position, hit_count in observations:
                previous = self.known_targets.get(track_id)
                velocity = [0.0, 0.0]
                if previous is not None and not stamp.is_zero():
                    dt = (stamp - previous["stamp"]).to_sec()
                    if 0.05 <= dt <= 2.0:
                        raw = [(position[index] - previous["position"][index]) / dt
                               for index in range(2)]
                        raw = clamp_vector(raw[0], raw[1], 8.0)
                        velocity = [0.65 * previous["velocity"][index] +
                                    0.35 * raw[index] for index in range(2)]
                    else:
                        velocity = list(previous["velocity"])
                self.known_targets[track_id] = {
                    "position": list(position), "velocity": velocity, "stamp": stamp,
                    "hit_count": max(hit_count, previous["hit_count"]
                                     if previous is not None else 0)}

            now = rospy.Time.now()
            for track_id, state in list(self.known_targets.items()):
                age = (now - state["stamp"]).to_sec()
                if age < -0.5 or age > self.target_memory_sec:
                    del self.known_targets[track_id]

    @staticmethod
    def _angle_delta(target, current):
        return math.atan2(math.sin(target - current), math.cos(target - current))

    def _global_coverage_plan(self, stamp):
        if self.expected_target_count <= 0:
            return None
        with self.target_state_lock:
            states = list(self.known_targets.values())
        points = []
        visible_points = []
        for state in states:
            age = (stamp - state["stamp"]).to_sec()
            if age < -0.5 or age > self.target_memory_sec:
                continue
            horizon = max(0.0, min(1.0, age + 0.35))
            point = (state["position"][0] + state["velocity"][0] * horizon,
                     state["position"][1] + state["velocity"][1] * horizon)
            points.append(point)
            if age <= 0.75:
                visible_points.append(point)

        # The tracker supplies global IDs and CV-filtered positions. Predict
        # recent memories to a common time and merge only near-identical
        # fragments; require several detector hits before counting an identity.
        def unique_positions(values, separation_m):
            unique = []
            for point in values:
                if all(math.hypot(point[0] - other[0], point[1] - other[1]) >= separation_m
                       for other in unique):
                    unique.append(point)
            return unique

        unique_points = unique_positions(points, 6.5)
        visible_count = len(unique_positions(visible_points, 6.5))
        # Plan against the configured search area, not only against targets
        # already detected. This makes the global-view waypoint reachable
        # before the tracker has acquired every target. For a rigid nadir
        # camera the area center maximizes edge margin; a map-edge view would
        # require a controllable camera pitch or gimbal.
        min_x, max_x, min_y, max_y = self.search_area_bounds
        center_x = 0.5 * (min_x + max_x)
        center_y = 0.5 * (min_y + max_y)
        area_width = max_x - min_x
        area_height = max_y - min_y
        # Image-horizontal points along body -Y: yaw=pi/2 aligns it with X,
        # while yaw=0 aligns it with Y.
        desired_yaw = 0.5 * math.pi if area_width >= area_height else 0.0
        corners = ((min_x, min_y), (min_x, max_y),
                   (max_x, min_y), (max_x, max_y))
        forward, lateral = [], []
        cy, sy = math.cos(desired_yaw), math.sin(desired_yaw)
        for x, y in corners:
            dx, dy = x - center_x, y - center_y
            forward.append(cy * dx + sy * dy)
            lateral.append(-sy * dx + cy * dy)
        vertical_fov = self._vertical_fov_rad()
        required_altitude = max(
            (max(lateral) - min(lateral) + 2.0 * self.coverage_margin_m) /
            (2.0 * math.tan(0.5 * self.camera_horizontal_fov_rad)),
            (max(forward) - min(forward) + 2.0 * self.coverage_margin_m) /
            (2.0 * math.tan(0.5 * vertical_fov)))
        feasible = required_altitude <= self.maximum_altitude_m
        target_altitude = max(self.minimum_altitude_m,
                              min(self.maximum_altitude_m, required_altitude))
        return (center_x, center_y, desired_yaw, target_altitude,
                len(unique_points), visible_count, feasible, required_altitude)

    def _move_global_coverage(self, plan, dt):
        center_x, center_y, desired_yaw, target_altitude = plan[:4]
        self._move_toward_xy((center_x, center_y), dt,
                             self.coverage_transit_speed_mps)
        self.reacquire_route_active = False
        self.altitude_target_m = target_altitude
        if not plan[6]:
            rospy.logwarn_throttle(
                5.0, "configured search area needs %.1f m altitude; capped at %.1f m",
                plan[7], self.maximum_altitude_m)
        rospy.loginfo_throttle(
            5.0, "global search viewpoint=(%.1f, %.1f), observed targets %d/%d, "
            "altitude %.1f/%.1f m%s",
            center_x, center_y, plan[4], self.expected_target_count,
            self.altitude_m, target_altitude,
            " (FOV-limited)" if not plan[6] else "")
        return desired_yaw

    def _world_measurement(self, track, message):
        if (not track.detected or track.predicted or not track.range_valid or
                not track.has_relative_position_body):
            return None
        stamp = track.header.stamp if not track.header.stamp.is_zero() else message.header.stamp
        pose = self._pose_at(stamp)
        if pose is None:
            return None
        position, quaternion, _ = pose
        body_flu = [float(track.relative_position_body[0]),
                    -float(track.relative_position_body[1]),
                    -float(track.relative_position_body[2]), 0.0]
        rotation = transformations.quaternion_matrix(quaternion)
        world_offset = rotation.dot(body_flu)
        measured = [position[0] + world_offset[0], position[1] + world_offset[1]]
        if not all(math.isfinite(value) for value in measured):
            return None
        return stamp, measured

    @staticmethod
    def _model_state(name, x, y, z, yaw, vx=0.0, vy=0.0):
        state = ModelState()
        state.model_name = name
        state.reference_frame = WORLD
        state.pose = Pose()
        state.pose.position.x = x
        state.pose.position.y = y
        state.pose.position.z = z
        q = yaw_quaternion(yaw)
        state.pose.orientation.x, state.pose.orientation.y = q[0], q[1]
        state.pose.orientation.z, state.pose.orientation.w = q[2], q[3]
        state.twist = Twist()
        state.twist.linear.x = vx
        state.twist.linear.y = vy
        return state

    def _publish_uav_pose(self, stamp, vx, vy):
        q = yaw_quaternion(self.drone_yaw)
        state = self._model_state(self.drone_model, self.drone_xy[0], self.drone_xy[1],
                                  self.altitude_m, self.drone_yaw, vx, vy)
        try:
            response = self.state_client(state)
        except rospy.ServiceException as error:
            rospy.logwarn_throttle(2.0, "quadrotor Gazebo state update failed: %s", error)
            return False
        if not response.success:
            rospy.logwarn_throttle(2.0, "Gazebo rejected quadrotor pose: %s",
                                   response.status_message)
            return False

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = WORLD
        transform.child_frame_id = BODY
        transform.transform.translation.x = self.drone_xy[0]
        transform.transform.translation.y = self.drone_xy[1]
        transform.transform.translation.z = self.altitude_m
        transform.transform.rotation.x = q[0]
        transform.transform.rotation.y = q[1]
        transform.transform.rotation.z = q[2]
        transform.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(transform)

        odometry = Odometry()
        odometry.header.stamp = stamp
        odometry.header.frame_id = WORLD
        odometry.child_frame_id = BODY
        odometry.pose.pose.position.x = self.drone_xy[0]
        odometry.pose.pose.position.y = self.drone_xy[1]
        odometry.pose.pose.position.z = self.altitude_m
        odometry.pose.pose.orientation.x = q[0]
        odometry.pose.pose.orientation.y = q[1]
        odometry.pose.pose.orientation.z = q[2]
        odometry.pose.pose.orientation.w = q[3]
        odometry.twist.twist.linear.x = math.cos(self.drone_yaw) * vx + math.sin(self.drone_yaw) * vy
        odometry.twist.twist.linear.y = -math.sin(self.drone_yaw) * vx + math.cos(self.drone_yaw) * vy
        for axis in (0, 7, 14, 21, 28, 35):
            odometry.pose.covariance[axis] = 0.02
        self.odom_pub.publish(odometry)
        with self.pose_lock:
            self.pose_history.append((stamp, tuple(self.drone_xy), q, (vx, vy)))
            cutoff = rospy.Time.from_sec(max(0.0, stamp.to_sec() - 3.0))
            self.pose_history = [item for item in self.pose_history if item[0] >= cutoff]
        return True

    def _publish_truth(self, stamp, elapsed):
        markers = MarkerArray()
        rows = []
        for vehicle in self.vehicles:
            x, y, vx, vy, yaw = self._vehicle_state(vehicle, elapsed)
            marker = Marker()
            marker.header.stamp = stamp
            marker.header.frame_id = WORLD
            marker.ns = "moving_search_truth"
            marker.id = vehicle["id"]
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 4.2
            marker.pose.orientation.w = 1.0
            marker.scale.z = 0.9
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.08, 0.08, 0.08, 1.0
            marker.text = "GT {}".format(vehicle["id"])
            markers.markers.append(marker)
            rows.append({"public_id": vehicle["id"], "model": vehicle["name"],
                         "class": vehicle["class"], "position_world": [x, y, GROUND_Z_M],
                         "velocity_world": [vx, vy, 0.0], "yaw_rad": yaw})
        self.truth_marker_pub.publish(markers)
        self.truth_json_pub.publish(String(data=json.dumps({
            "stamp": stamp.to_sec(), "elapsed_sec": elapsed,
            "phase": self.phase, "locked_track_id": self.follow_track_id,
            "uav_position_world": [self.drone_xy[0], self.drone_xy[1], self.altitude_m],
            "vehicles": rows,
        }, sort_keys=True)))

    def run(self):
        while not rospy.is_shutdown():
            try:
                rospy.wait_for_service("/gazebo/set_model_state", timeout=2.0)
                break
            except rospy.ROSException:
                rospy.loginfo_throttle(5.0, "waiting for Gazebo set_model_state")

        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            stamp = rospy.Time.now()
            if stamp.is_zero():
                rate.sleep()
                continue
            if self.start_time is None or (not self.last_stamp.is_zero() and stamp < self.last_stamp):
                self.start_time = stamp
                self.last_loop_time = stamp
                self.search_progress = 0.0
                self.search_direction = 1.0
            elapsed = (stamp - self.start_time).to_sec()
            dt = max(0.0, min(0.10, (stamp - self.last_loop_time).to_sec()))
            self.last_loop_time = stamp
            self.last_stamp = stamp

            visual_age = ((stamp - self.last_visual_measurement).to_sec()
                          if not self.last_visual_measurement.is_zero()
                          else float("inf"))
            fresh_visual = 0.0 <= visual_age <= 1.8
            if self.follow_track_id is not None and fresh_visual and self.target_world is not None:
                self.phase = "FOLLOW"
                self.reacquire_route_active = False
                horizon = min(0.75, visual_age + 0.25)
                predicted_target = [
                    self.target_world[index] +
                    self.target_velocity_world[index] * horizon
                    for index in range(2)]
                overview = self._overview_follow_plan(predicted_target, stamp)
                if overview is not None:
                    self.altitude_target_m = overview[1]
                    self._move_toward_xy(
                        overview[0], dt, self.coverage_transit_speed_mps,
                        gain=0.45)
                else:
                    self.altitude_target_m = self._follow_altitude_target()
                    target_velocity = clamp_vector(
                        self.target_velocity_world[0],
                        self.target_velocity_world[1], 2.0)
                    self._move_toward_xy(
                        predicted_target, dt, self.follow_max_speed_mps,
                        gain=0.45, feedforward=target_velocity)
                # A quadrotor can translate while holding yaw. Keeping the
                # rigid nadir camera level in image coordinates reduces
                # apparent box motion during target turns.
                desired_yaw = self.drone_yaw
            elif self.selection_mode == "follow" and self.manual_selection_active:
                reference_stamp = (self.last_visual_measurement
                                   if not self.last_visual_measurement.is_zero()
                                   else self.follow_mode_started_at)
                loss_age = (max(0.0, (stamp - reference_stamp).to_sec())
                            if not reference_stamp.is_zero() else 0.0)
                if loss_age <= self.follow_loss_coast_sec:
                    self.phase = "FOLLOW_COAST"
                    self.altitude_target_m = max(self.altitude_m,
                                                 self.minimum_altitude_m)
                    if self.target_world is not None:
                        horizon = min(2.0, loss_age)
                        predicted_target = [
                            self.target_world[0] + self.target_velocity_world[0] * horizon,
                            self.target_world[1] + self.target_velocity_world[1] * horizon]
                        overview = self._overview_follow_plan(predicted_target, stamp)
                        if overview is not None:
                            self.altitude_target_m = max(self.altitude_m, overview[1])
                            self._move_toward_xy(
                                overview[0], dt, self.coverage_transit_speed_mps,
                                gain=0.45)
                        else:
                            self._move_toward_xy(
                                predicted_target, dt, self.search_cruise_speed_mps,
                                gain=0.55)
                    else:
                        self._move_toward_xy(
                            self.drone_xy, dt, self.search_cruise_speed_mps)
                    desired_yaw = self.drone_yaw
                else:
                    desired_yaw = self._advance_follow_reacquire_search(
                        dt, stamp, loss_age)
            elif not self.manual_selection_active and (
                    (coverage_plan := self._global_coverage_plan(stamp)) is not None):
                self.phase = "GLOBAL_COVERAGE"
                desired_yaw = self._move_global_coverage(coverage_plan, dt)
            else:
                if self.follow_track_id is not None and self.selection_mode != "follow":
                    rospy.loginfo("visual target lost; resuming search route")
                    self.follow_track_id = None
                    self.target_world = None
                    self.target_velocity_world = [0.0, 0.0]
                    self.previous_target_world = None
                    self.previous_target_stamp = rospy.Time()
                self.phase = "FOLLOW_LOST_SEARCH" if self.manual_selection_active else "SEARCH_ALL"
                self.altitude_target_m = self.minimum_altitude_m
                desired_yaw = self._advance_search_route(dt)

            altitude_step = self.altitude_rate_mps * dt
            altitude_error = self.altitude_target_m - self.altitude_m
            self.altitude_m += max(-altitude_step, min(altitude_step, altitude_error))

            yaw_delta = math.atan2(math.sin(desired_yaw - self.drone_yaw),
                                   math.cos(desired_yaw - self.drone_yaw))
            yaw_step = max(-self.yaw_rate_limit_radps * dt,
                           min(self.yaw_rate_limit_radps * dt, yaw_delta))
            self.drone_yaw = math.atan2(math.sin(self.drone_yaw + yaw_step),
                                        math.cos(self.drone_yaw + yaw_step))
            self._publish_uav_pose(stamp, self.drone_velocity[0], self.drone_velocity[1])

            self.loop_count += 1
            if self.loop_count % 2 == 0:
                for vehicle in self.vehicles:
                    x, y, vx, vy, yaw = self._vehicle_state(vehicle, elapsed)
                    try:
                        response = self.state_client(self._model_state(
                            vehicle["name"], x, y, GROUND_Z_M, yaw, vx, vy))
                        if not response.success:
                            rospy.logwarn_throttle(2.0, "Gazebo rejected %s pose: %s",
                                                   vehicle["name"], response.status_message)
                    except rospy.ServiceException as error:
                        rospy.logwarn_throttle(2.0, "vehicle Gazebo state update failed: %s", error)
            if self.loop_count % 4 == 0:
                self._publish_truth(stamp, elapsed)
                self.phase_pub.publish(String(data=self.phase))
            try:
                rate.sleep()
            except rospy.exceptions.ROSTimeMovedBackwardsException:
                self.start_time = None
                self.last_loop_time = None
            except rospy.ROSInterruptException:
                break


def main():
    rospy.init_node("moving_uav_search_driver")
    try:
        MovingSearchDriver().run()
    except (ValueError, rospy.ROSException) as error:
        rospy.logfatal("moving UAV search scenario failed: %s", error)
        raise


if __name__ == "__main__":
    main()
