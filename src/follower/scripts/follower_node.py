#!/usr/bin/env python3
"""
follower_node.py — Follower ROS Node

MRS-style ROS node wrapping FollowerCore.

Subscribes to:
  - /tracker_node/normalized_error (tracker/NormalizedError) — from Tracker
  - /mavros/local_position/pose (geometry_msgs/PoseStamped) — PX4 position
  - /mavros/imu/data (sensor_msgs/Imu) — PX4 attitude
  - /mavros/state (mavros_msgs/State) — PX4 flight mode
  - /mavros/altitude (mavros_msgs/Altitude) — PX4 altitude
  - /follower_node/controller_command (follower/ControllerCommand) — Controller → Follower

Publishes:
  - /follower_node/follower_command (follower/FollowerCommand) — control commands
  - /follower_node/follower_status (follower/FollowerStatus) — status/diagnostics
  - /follower_node/controller_feedback (follower/ControllerFeedback) — Follower → Controller
  - /mavros/setpoint_velocity/cmd_vel (geometry_msgs/TwistStamped) — PX4 velocity
  - /follower_node/velocity_command_marker (visualization_msgs/Marker) — RViz commanded velocity arrow
  - /follower_node/actual_velocity_marker (visualization_msgs/Marker) — RViz MAVROS feedback velocity arrow

Services:
  - /follower/start — start following
  - /follower/stop — stop following
  - /follower/emergency_stop — emergency stop
  - /follower/set_mode — switch lateral guidance mode
"""

import rospy
import ast
import time
import math
import threading
from typing import Optional

from std_msgs.msg import Header
from std_srvs.srv import SetBool, SetBoolResponse
from geometry_msgs.msg import Point, PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from sensor_msgs.msg import Imu
from mavros_msgs.msg import State, Altitude
from mrs_msgs.msg import VelocityReferenceStamped
from tracker.msg import NormalizedError
from follower.msg import FollowerCommand, FollowerStatus, ControllerFeedback, ControllerCommand
from follower.srv import SetMode, SetModeResponse

from follower.follower_core import FollowerCore, FollowerResult
from follower.px4_interface import PX4Interface, PX4Telemetry


def _get_pid_param(name, default):
    """Read a PID mapping whether roslaunch supplied YAML or a string."""
    value = rospy.get_param(name, default)
    if isinstance(value, str):
        try:
            value = ast.literal_eval(value)
        except (SyntaxError, ValueError):
            rospy.logwarn('[FollowerNode] Invalid PID parameter %s; using defaults', name)
            value = default
    if not isinstance(value, dict):
        rospy.logwarn('[FollowerNode] PID parameter %s is not a mapping; using defaults', name)
        value = default
    parsed = {}
    for key, fallback in default.items():
        try:
            parsed[key] = float(value.get(key, fallback))
        except (TypeError, ValueError):
            parsed[key] = fallback
    return parsed


class FollowerNode:
    """
    ROS node wrapping FollowerCore for ROS1-based target following.

    Bridges:
    - Tracker → FollowerCore (normalized error input)
    - PX4/MAVROS → FollowerCore (telemetry)
    - FollowerCore → PX4 Offboard (velocity commands)
    - FollowerCore ↔ Controller (bidirectional feedback port)
    """

    def __init__(self):
        rospy.init_node('follower_node', anonymous=False)

        # ── Parameters ───────────────────────────────────────────────────
        self.publish_rate = rospy.get_param('~publish_rate', 30.0)
        self.frame_width = rospy.get_param('~frame_width', 640)
        self.frame_height = rospy.get_param('~frame_height', 480)
        self.tracker_error_topic = rospy.get_param(
            '~tracker_error_topic', '/tracker_node/normalized_error')
        self.error_timeout_sec = max(0.01, float(rospy.get_param('~error_timeout_sec', 0.25)))
        self.mavros_namespace = rospy.get_param('~mavros_namespace', '/mavros').rstrip('/')
        self.enable_mavros_output = rospy.get_param('~enable_mavros_output', True)
        self.mavros_velocity_topic = rospy.get_param(
            '~mavros_velocity_topic', self.mavros_namespace + '/setpoint_velocity/cmd_vel')
        self.enable_mrs_velocity_reference = rospy.get_param(
            '~enable_mrs_velocity_reference', False)
        self.mrs_velocity_reference_topic = rospy.get_param(
            '~mrs_velocity_reference_topic', '/uav1/control_manager/velocity_reference_in')
        self.mrs_velocity_frame = rospy.get_param('~mrs_velocity_frame', 'uav1/fcu')
        self.visualization_frame = rospy.get_param('~visualization_frame', 'map')
        self.velocity_marker_scale = max(0.01, float(rospy.get_param('~velocity_marker_scale', 1.0)))
        self.velocity_marker_min_speed = max(0.0, float(rospy.get_param('~velocity_marker_min_speed', 0.02)))
        self.actual_velocity_marker_scale = max(0.01, float(rospy.get_param('~actual_velocity_marker_scale', 1.0)))
        self.actual_velocity_marker_min_speed = max(0.0, float(rospy.get_param('~actual_velocity_marker_min_speed', 0.02)))

        # Build config for FollowerCore
        config = {
            # Control mode
            'lateral_guidance_mode': rospy.get_param('~lateral_guidance_mode', 'coordinated_turn'),
            'enable_auto_mode_switching': rospy.get_param('~enable_auto_mode_switching', False),
            'mode_switch_velocity': rospy.get_param('~mode_switch_velocity', 3.0),
            'mode_switch_hysteresis': rospy.get_param('~mode_switch_hysteresis', 0.5),

            # Velocity limits
            'max_velocity_forward': rospy.get_param('~max_velocity_forward', 8.0),
            'max_velocity_lateral': rospy.get_param('~max_velocity_lateral', 3.0),
            'max_velocity_vertical': rospy.get_param('~max_velocity_vertical', 2.0),
            'max_velocity_magnitude': rospy.get_param('~max_velocity_magnitude', 15.0),
            'max_yaw_rate_deg_s': rospy.get_param('~max_yaw_rate_deg_s', 90.0),

            # Altitude
            'min_altitude': rospy.get_param('~min_altitude', 2.0),
            'max_altitude': rospy.get_param('~max_altitude', 120.0),

            # Forward velocity
            'initial_forward_velocity': rospy.get_param('~initial_forward_velocity', 0.0),
            'max_forward_velocity': rospy.get_param('~max_forward_velocity', 5.0),
            'forward_ramp_rate': rospy.get_param('~forward_ramp_rate', 0.5),
            'ramp_down_on_target_loss': rospy.get_param('~ramp_down_on_target_loss', True),
            'target_loss_stop_velocity': rospy.get_param('~target_loss_stop_velocity', 0.0),
            'forward_velocity_deadzone': rospy.get_param('~forward_velocity_deadzone', 0.01),
            'min_forward_velocity_threshold': rospy.get_param('~min_forward_velocity_threshold', 0.2),

            # Yaw smoothing
            'yaw_smoothing_enabled': rospy.get_param('~yaw_smoothing_enabled', True),
            'yaw_deadzone_deg_s': rospy.get_param('~yaw_deadzone_deg_s', 0.5),
            'yaw_max_rate_change_deg_s2': rospy.get_param('~yaw_max_rate_change_deg_s2', 90.0),
            'yaw_smoothing_alpha': rospy.get_param('~yaw_smoothing_alpha', 0.7),
            'yaw_speed_scaling_enabled': rospy.get_param('~yaw_speed_scaling_enabled', True),
            'yaw_min_speed_threshold': rospy.get_param('~yaw_min_speed_threshold', 0.5),
            'yaw_max_speed_threshold': rospy.get_param('~yaw_max_speed_threshold', 5.0),
            'yaw_low_speed_factor': rospy.get_param('~yaw_low_speed_factor', 0.5),

            # PID
            'pid_yaw': _get_pid_param('~pid_yaw', {'kp': 2.0, 'ki': 0.05, 'kd': 0.1}),
            'pid_right': _get_pid_param('~pid_right', {'kp': 1.5, 'ki': 0.02, 'kd': 0.05}),
            'pid_down': _get_pid_param('~pid_down', {'kp': 1.0, 'ki': 0.03, 'kd': 0.05}),

            # Adaptive
            'adaptive_dive_climb_enabled': rospy.get_param('~adaptive_dive_climb_enabled', False),
            'adaptive_smoothing_alpha': rospy.get_param('~adaptive_smoothing_alpha', 0.2),
            'adaptive_warmup_frames': rospy.get_param('~adaptive_warmup_frames', 10),
            'adaptive_rate_threshold': rospy.get_param('~adaptive_rate_threshold', 5.0),
            'adaptive_max_correction': rospy.get_param('~adaptive_max_correction', 1.0),
            'adaptive_correction_gain': rospy.get_param('~adaptive_correction_gain', 0.3),
            'adaptive_fwd_coupling_enabled': rospy.get_param('~adaptive_fwd_coupling_enabled', False),
            'pixel_to_rate_calibration': rospy.get_param('~pixel_to_rate_calibration', 0.05),

            # Pitch compensation
            'pitch_compensation_enabled': rospy.get_param('~pitch_compensation_enabled', False),
            'pitch_compensation_gain': rospy.get_param('~pitch_compensation_gain', 0.05),
            'pitch_smoothing_alpha': rospy.get_param('~pitch_smoothing_alpha', 0.7),
            'pitch_max_correction': rospy.get_param('~pitch_max_correction', 0.3),

            # Target loss
            'target_loss_coord_threshold': rospy.get_param('~target_loss_coord_threshold', 1.5),
            'target_loss_timeout': rospy.get_param('~target_loss_timeout', 3.0),

            # Smoothing
            'command_smoothing_enabled': rospy.get_param('~command_smoothing_enabled', True),
            'smoothing_factor': rospy.get_param('~smoothing_factor', 0.3),

            # Video
            'video_height_pixels': self.frame_height,
        }

        # ── Initialize FollowerCore ──────────────────────────────────────
        self.follower = FollowerCore(config)
        self.px4 = self.follower.px4  # Alias for readability

        # ── State ────────────────────────────────────────────────────────
        self.following_active = False
        self._last_error_msg: Optional[NormalizedError] = None
        self._last_error_time = 0.0
        self._last_compute_time = time.time()
        self._latest_pose: Optional[PoseStamped] = None
        self._latest_local_velocity: Optional[TwistStamped] = None
        self._lock = threading.Lock()

        # Controller feedback state
        self._controller_override_active = False
        self._controller_limits = None
        self._controller_override = None

        # ── Publishers ───────────────────────────────────────────────────
        self.cmd_pub = rospy.Publisher(
            '~follower_command', FollowerCommand, queue_size=10,
        )
        self.status_pub = rospy.Publisher(
            '~follower_status', FollowerStatus, queue_size=10,
        )
        self.controller_fb_pub = rospy.Publisher(
            '~controller_feedback', ControllerFeedback, queue_size=10,
        )
        self.velocity_viz_pub = rospy.Publisher(
            '~velocity_command_odom', Odometry, queue_size=10,
        )
        self.velocity_marker_pub = rospy.Publisher(
            '~velocity_command_marker', Marker, queue_size=10,
        )
        self.actual_velocity_marker_pub = rospy.Publisher(
            '~actual_velocity_marker', Marker, queue_size=10,
        )
        # Direct PX4/MAVROS output remains optional for SITL and standalone tests.
        self.px4_vel_pub = None
        if self.enable_mavros_output:
            self.px4_vel_pub = rospy.Publisher(
                self.mavros_velocity_topic, TwistStamped, queue_size=10,
            )
        self.mrs_velocity_pub = None
        if self.enable_mrs_velocity_reference:
            self.mrs_velocity_pub = rospy.Publisher(
                self.mrs_velocity_reference_topic, VelocityReferenceStamped, queue_size=10,
            )

        # ── Subscribers ──────────────────────────────────────────────────
        # Tracker
        self.error_sub = rospy.Subscriber(
            self.tracker_error_topic, NormalizedError,
            self._error_callback, queue_size=10,
        )

        # PX4 Telemetry
        self.imu_sub = rospy.Subscriber(
            self.mavros_namespace + '/imu/data', Imu,
            self._imu_callback, queue_size=10,
        )
        self.state_sub = rospy.Subscriber(
            self.mavros_namespace + '/state', State,
            self._state_callback, queue_size=10,
        )
        self.altitude_sub = rospy.Subscriber(
            self.mavros_namespace + '/altitude', Altitude,
            self._altitude_callback, queue_size=10,
        )
        self.pose_sub = rospy.Subscriber(
            self.mavros_namespace + '/local_position/pose', PoseStamped,
            self._pose_callback, queue_size=10,
        )
        self.velocity_sub = rospy.Subscriber(
            self.mavros_namespace + '/local_position/velocity_local', TwistStamped,
            self._velocity_callback, queue_size=10,
        )

        # Controller feedback (Controller → Follower, on separate topic)
        self.controller_sub = rospy.Subscriber(
            '~controller_command', ControllerCommand,
            self._controller_command_callback, queue_size=10,
        )

        # ── Timers ───────────────────────────────────────────────────────
        self._compute_timer = rospy.Timer(
            rospy.Duration(1.0 / self.publish_rate),
            self._compute_callback,
        )

        # ── Services ─────────────────────────────────────────────────────
        rospy.Service('~start', SetBool, self._srv_start)
        rospy.Service('~stop', SetBool, self._srv_stop)
        rospy.Service('~emergency_stop', SetBool, self._srv_emergency_stop)
        rospy.Service('~set_mode', SetMode, self._srv_set_mode)

        rospy.loginfo(
            "[FollowerNode] Initialized: publish_rate=%.1f Hz, mode=%s",
            self.publish_rate, config['lateral_guidance_mode'],
        )

    # ── Callbacks ────────────────────────────────────────────────────────

    def _error_callback(self, msg: NormalizedError):
        """Receive normalized error from Tracker."""
        with self._lock:
            self._last_error_msg = msg
            self._last_error_time = time.time()

    def _pose_callback(self, msg: PoseStamped):
        """Convert MAVROS local ENU position into the PX4 NED telemetry model."""
        position = msg.pose.position
        with self._lock:
            self._latest_pose = msg
        self.px4.update_position(position.x, position.y, -position.z)

    def _velocity_callback(self, msg: TwistStamped):
        """Convert MAVROS local ENU velocity into NED telemetry and retain it for RViz."""
        velocity = msg.twist.linear
        with self._lock:
            self._latest_local_velocity = msg
        self.px4.update_velocity_ned(velocity.x, velocity.y, -velocity.z)

    def _imu_callback(self, msg: Imu):
        """Receive IMU attitude from PX4."""
        from tf.transformations import euler_from_quaternion
        q = msg.orientation
        roll, pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.px4.update_attitude(
            roll_deg=math.degrees(roll), pitch_deg=math.degrees(pitch),
            yaw_deg=math.degrees(yaw), timestamp=msg.header.stamp.to_sec(),
        )

    def _state_callback(self, msg: State):
        """Receive flight mode from PX4."""
        self.px4.update_flight_mode(
            mode=msg.mode,
            armed=msg.armed,
            offboard=(msg.mode == "OFFBOARD"),
        )

    def _altitude_callback(self, msg: Altitude):
        """Receive altitude from PX4."""
        self.px4.update_altitude(
            alt_msl=msg.amsl,
            alt_rel=msg.relative,
        )

    def _controller_command_callback(self, msg: ControllerCommand):
        """Store downstream override and additional velocity bounds."""
        self._controller_override_active = bool(msg.override_active)
        if self._controller_override_active:
            self._controller_override = (
                msg.override_velocity_forward, msg.override_velocity_right,
                msg.override_velocity_down, msg.override_yaw_rate_deg_s,
            )
            self._controller_limits = list(msg.velocity_limits) if msg.has_velocity_limits else None
            rospy.loginfo_throttle(2.0, '[FollowerNode] Controller override: %s', msg.override_reason)
        else:
            self._controller_override = None
            self._controller_limits = list(msg.velocity_limits) if msg.has_velocity_limits else None

    # ── Compute Timer ────────────────────────────────────────────────────

    def _compute_callback(self, event):
        """Compute on fresh input, or actively converge to the loss-safe command."""
        if not self.following_active:
            self._publish_stop_command()
            self._publish_actual_velocity_marker()
            return
        current_time = time.time()
        with self._lock:
            error_msg = self._last_error_msg
            error_time = self._last_error_time
        stale = error_msg is None or current_time - error_time > self.error_timeout_sec
        if stale:
            rospy.logwarn_throttle(2.0, '[FollowerNode] Tracker error is unavailable or stale; stopping target corrections')
            error_x = error_y = 0.0
            error_valid = False
        else:
            error_x, error_y = error_msg.error_x, error_msg.error_y
            error_valid = bool(error_msg.error_valid)
        dt = current_time - self._last_compute_time
        self._last_compute_time = current_time
        if dt <= 0.0 or dt > 1.0:
            dt = 1.0 / self.publish_rate
        result = self.follower.compute(
            error_x=error_x, error_y=error_y, dt=dt, target_confidence=1.0,
            error_valid=error_valid, timestamp=current_time)
        self._apply_controller_constraints(result)
        self._publish_follower_command(result)
        self._publish_follower_status(result)
        self._publish_px4_velocity(result)
        self._publish_velocity_command_odometry(result)
        self._publish_velocity_command_marker(result)
        self._publish_actual_velocity_marker()
        self._publish_controller_feedback(result)

    def _apply_controller_constraints(self, result: FollowerResult):
        """Apply a trusted controller override, then reapply local safety bounds."""
        if self._controller_override_active and self._controller_override is not None:
            (result.velocity_forward, result.velocity_right, result.velocity_down,
             result.yaw_rate_deg_s) = self._controller_override
            result.command_valid = not result.emergency_stop_active
        if self._controller_limits is not None and len(self._controller_limits) == 6:
            vx_min, vx_max, vy_min, vy_max, vz_min, vz_max = self._controller_limits
            if vx_min <= vx_max and vy_min <= vy_max and vz_min <= vz_max:
                result.velocity_forward = max(vx_min, min(vx_max, result.velocity_forward))
                result.velocity_right = max(vy_min, min(vy_max, result.velocity_right))
                result.velocity_down = max(vz_min, min(vz_max, result.velocity_down))
            else:
                rospy.logwarn_throttle(2.0, '[FollowerNode] Ignoring invalid controller velocity limits')
        result.velocity_forward, _ = self.follower.safety.clamp_velocity_forward(result.velocity_forward)
        result.velocity_right, _ = self.follower.safety.clamp_velocity_lateral(result.velocity_right)
        result.velocity_down, _ = self.follower.safety.clamp_velocity_vertical(result.velocity_down)
        result.velocity_down = self.follower._enforce_altitude_envelope(result.velocity_down)
        result.velocity_forward, result.velocity_right, result.velocity_down =             self.follower.safety.clamp_command_magnitude(
                result.velocity_forward, result.velocity_right, result.velocity_down)
        yaw_rate, _ = self.follower.safety.clamp_yaw_rate(math.radians(result.yaw_rate_deg_s))
        result.yaw_rate_deg_s = math.degrees(yaw_rate)

    def _publish_follower_command(self, result: FollowerResult):
        """Publish FollowerCommand message."""
        msg = FollowerCommand()
        msg.header = Header(stamp=rospy.Time.now())

        msg.control_mode = "velocity_body"
        msg.lateral_guidance_mode = result.lateral_guidance_mode

        msg.velocity_forward = result.velocity_forward
        msg.velocity_right = result.velocity_right
        msg.velocity_down = result.velocity_down

        msg.yaw_rate_deg_s = result.yaw_rate_deg_s
        msg.yaw_rate_raw_deg_s = result.yaw_rate_raw_deg_s
        msg.yaw_smoothing_active = result.yaw_smoothing_active

        msg.command_valid = result.command_valid
        msg.target_visible = result.target_visible
        msg.emergency_stop_active = result.emergency_stop_active

        msg.adaptive_dive_climb_active = result.adaptive_active
        msg.adaptive_correction_down = result.adaptive_correction_down
        msg.adaptive_correction_fwd = result.adaptive_correction_fwd

        msg.pitch_compensation_active = result.pitch_compensation_active
        msg.pitch_compensation_value = result.pitch_compensation_value

        msg.target_error_x = result.target_error_x
        msg.target_error_y = result.target_error_y
        msg.target_lost = result.target_lost
        msg.target_loss_duration = result.target_loss_duration

        msg.pid_yaw_output = result.pid_yaw_output
        msg.pid_down_output = result.pid_down_output
        msg.pid_right_output = result.pid_right_output

        self.cmd_pub.publish(msg)

    def _publish_follower_status(self, result: FollowerResult):
        """Publish FollowerStatus message."""
        msg = FollowerStatus()
        msg.header = Header(stamp=rospy.Time.now())

        msg.following_active = self.following_active
        msg.target_lost = result.target_lost
        msg.emergency_stop_active = result.emergency_stop_active
        msg.control_mode = "velocity_body"
        msg.lateral_guidance_mode = result.lateral_guidance_mode

        msg.current_forward_velocity = result.current_forward_velocity
        msg.target_forward_velocity = self.follower.ramper.max_velocity
        msg.lateral_velocity = result.velocity_right
        msg.vertical_velocity = result.velocity_down
        msg.yaw_rate = result.yaw_rate_deg_s

        msg.target_error_x = result.target_error_x
        msg.target_error_y = result.target_error_y
        msg.target_loss_duration = result.target_loss_duration

        msg.adaptive_active = result.adaptive_active
        msg.vertical_rate_error = self.follower.adaptive.vertical_rate_error
        msg.smoothed_vertical_rate = self.follower.adaptive.smoothed_vertical_rate

        msg.pitch_compensation_active = result.pitch_compensation_active
        msg.pitch_compensation_value = result.pitch_compensation_value

        telem = self.px4.telemetry
        msg.altitude_current = telem.altitude_rel
        msg.altitude_safe = result.altitude_safe
        msg.altitude_violation_count = self.follower.altitude_violation_count

        msg.pid_yaw_output = result.pid_yaw_output
        msg.pid_down_output = result.pid_down_output
        msg.pid_right_output = result.pid_right_output

        msg.loop_actual_rate = self.publish_rate
        msg.update_count = result.update_count

        self.status_pub.publish(msg)

    def _publish_px4_velocity(self, result: FollowerResult):
        """Publish equivalent body-frame commands to enabled PX4 and MRS outputs."""
        if self.px4_vel_pub is not None:
            msg = TwistStamped()
            msg.header = Header(stamp=rospy.Time.now(), frame_id='base_link')
            msg.twist.linear.x = result.velocity_forward
            msg.twist.linear.y = -result.velocity_right
            msg.twist.linear.z = -result.velocity_down
            msg.twist.angular.z = math.radians(result.yaw_rate_deg_s)
            self.px4_vel_pub.publish(msg)
        if self.mrs_velocity_pub is not None:
            msg = VelocityReferenceStamped()
            msg.header = Header(stamp=rospy.Time.now(), frame_id=self.mrs_velocity_frame)
            msg.reference.velocity.x = result.velocity_forward
            msg.reference.velocity.y = -result.velocity_right
            msg.reference.velocity.z = -result.velocity_down
            msg.reference.heading_rate = math.radians(result.yaw_rate_deg_s)
            msg.reference.use_altitude = False
            msg.reference.use_heading = False
            msg.reference.use_heading_rate = True
            self.mrs_velocity_pub.publish(msg)

    def _publish_velocity_command_odometry(self, result: FollowerResult):
        """Publish a correctly typed RViz velocity visualization message."""
        msg = Odometry()
        msg.header = Header(stamp=rospy.Time.now(), frame_id=self.visualization_frame)
        msg.child_frame_id = 'follower_velocity_command'
        with self._lock:
            pose = self._latest_pose
        if pose is not None:
            msg.header.frame_id = pose.header.frame_id or self.visualization_frame
            msg.pose.pose = pose.pose
        else:
            msg.pose.pose.orientation.w = 1.0
        # Velocity is expressed in the aircraft body frame, matching MAVROS output.
        msg.twist.twist.linear.x = result.velocity_forward
        msg.twist.twist.linear.y = -result.velocity_right
        msg.twist.twist.linear.z = -result.velocity_down
        msg.twist.twist.angular.z = math.radians(result.yaw_rate_deg_s)
        self.velocity_viz_pub.publish(msg)

    def _publish_velocity_command_marker(self, result: FollowerResult):
        """Draw the commanded ENU velocity as an RViz arrow at the current pose."""
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.ns = 'follower_velocity_command'
        marker.id = 0
        marker.type = Marker.ARROW

        # MAVROS accepts body-frame FLU commands. Rotate horizontal components
        # into the local ENU frame before drawing a fixed-frame RViz marker.
        body_x = result.velocity_forward
        body_y = -result.velocity_right
        body_z = -result.velocity_down
        start_x = start_y = start_z = 0.0
        frame_id = self.visualization_frame
        with self._lock:
            pose = self._latest_pose
        if pose is not None:
            frame_id = pose.header.frame_id or frame_id
            start_x = pose.pose.position.x
            start_y = pose.pose.position.y
            start_z = pose.pose.position.z
            q = pose.pose.orientation
            from tf.transformations import euler_from_quaternion
            _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
            velocity_x = math.cos(yaw) * body_x - math.sin(yaw) * body_y
            velocity_y = math.sin(yaw) * body_x + math.cos(yaw) * body_y
        else:
            velocity_x = body_x
            velocity_y = body_y
        velocity_z = body_z
        marker.header.frame_id = frame_id

        speed = math.sqrt(velocity_x ** 2 + velocity_y ** 2 + velocity_z ** 2)
        if speed < self.velocity_marker_min_speed:
            marker.action = Marker.DELETE
            self.velocity_marker_pub.publish(marker)
            return

        marker.action = Marker.ADD
        marker.points = [
            Point(x=start_x, y=start_y, z=start_z),
            Point(
                x=start_x + velocity_x * self.velocity_marker_scale,
                y=start_y + velocity_y * self.velocity_marker_scale,
                z=start_z + velocity_z * self.velocity_marker_scale,
            ),
        ]
        marker.scale.x = 0.06
        marker.scale.y = 0.14
        marker.scale.z = 0.20
        marker.color.r = 1.0
        marker.color.g = 0.39
        marker.color.b = 0.0
        marker.color.a = 0.95
        marker.lifetime = rospy.Duration(0.25)
        self.velocity_marker_pub.publish(marker)

    def _publish_actual_velocity_marker(self):
        """Draw the latest MAVROS feedback velocity in the RViz fixed frame."""
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.ns = 'mavros_feedback_velocity'
        marker.id = 0
        marker.type = Marker.ARROW
        with self._lock:
            pose = self._latest_pose
            velocity_msg = self._latest_local_velocity

        if pose is None or velocity_msg is None:
            marker.action = Marker.DELETE
            self.actual_velocity_marker_pub.publish(marker)
            return

        frame_id = pose.header.frame_id or self.visualization_frame
        start = pose.pose.position
        velocity = velocity_msg.twist.linear
        velocity_x, velocity_y, velocity_z = velocity.x, velocity.y, velocity.z

        # Some MAVROS configurations label this feedback as base_link. Convert
        # body FLU values to the local fixed frame so the RViz arrow is spatially correct.
        if velocity_msg.header.frame_id.endswith('base_link'):
            q = pose.pose.orientation
            from tf.transformations import euler_from_quaternion
            _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
            velocity_x = math.cos(yaw) * velocity.x - math.sin(yaw) * velocity.y
            velocity_y = math.sin(yaw) * velocity.x + math.cos(yaw) * velocity.y

        marker.header.frame_id = frame_id
        speed = math.sqrt(velocity_x ** 2 + velocity_y ** 2 + velocity_z ** 2)
        if speed < self.actual_velocity_marker_min_speed:
            marker.action = Marker.DELETE
            self.actual_velocity_marker_pub.publish(marker)
            return

        marker.action = Marker.ADD
        marker.points = [
            Point(x=start.x, y=start.y, z=start.z),
            Point(
                x=start.x + velocity_x * self.actual_velocity_marker_scale,
                y=start.y + velocity_y * self.actual_velocity_marker_scale,
                z=start.z + velocity_z * self.actual_velocity_marker_scale,
            ),
        ]
        marker.scale.x = 0.05
        marker.scale.y = 0.12
        marker.scale.z = 0.18
        marker.color.r = 0.0
        marker.color.g = 0.85
        marker.color.b = 1.0
        marker.color.a = 0.95
        marker.lifetime = rospy.Duration(0.25)
        self.actual_velocity_marker_pub.publish(marker)

    def _publish_controller_feedback(self, result: FollowerResult):
        """Publish Follower→Controller feedback (suggestions + status)."""
        msg = ControllerFeedback()
        msg.header = Header(stamp=rospy.Time.now())

        # Suggestions for Controller
        msg.suggested_velocity_forward = result.suggested_velocity_forward
        msg.suggested_velocity_right = result.suggested_velocity_right
        msg.suggested_velocity_down = result.suggested_velocity_down
        msg.suggested_yaw_rate_deg_s = result.suggested_yaw_rate_deg_s
        msg.suggestion_valid = result.suggestion_valid

        # Current control mode
        msg.control_mode = "velocity_body"
        msg.lateral_guidance_mode = result.lateral_guidance_mode

        # Target status
        msg.target_error_x = result.target_error_x
        msg.target_error_y = result.target_error_y
        msg.target_visible = result.target_visible
        msg.emergency_stop_active = result.emergency_stop_active

        # Follower status
        msg.following_active = self.following_active
        msg.follower_confidence = (
            0.0 if result.target_lost else
            max(0.0, 1.0 - result.target_loss_duration / 3.0)
        )

        self.controller_fb_pub.publish(msg)

    def _publish_stop_command(self):
        """Publish zero body velocity on every enabled output path."""
        stop = FollowerResult()
        self._publish_px4_velocity(stop)
        self._publish_velocity_command_odometry(stop)
        self._publish_velocity_command_marker(stop)

    # ── Services ─────────────────────────────────────────────────────────

    def _srv_start(self, req):
        """Start following."""
        self.following_active = True
        self.follower.reset()
        rospy.loginfo("[FollowerNode] Following STARTED")
        return SetBoolResponse(success=True, message='Following started')

    def _srv_stop(self, req):
        """Stop following."""
        self.following_active = False
        self.follower.reset()
        rospy.loginfo("[FollowerNode] Following STOPPED")
        return SetBoolResponse(success=True, message='Following stopped')

    def _srv_emergency_stop(self, req):
        """Emergency stop."""
        active = req.data
        self.follower.set_emergency_stop(active)
        rospy.logwarn("[FollowerNode] EMERGENCY STOP: %s", "ACTIVE" if active else "RELEASED")
        return SetBoolResponse(success=True, message=f'Emergency stop {"active" if active else "released"}')

    def _srv_set_mode(self, req):
        """Set lateral guidance mode."""
        mode = req.mode
        self.follower.set_lateral_mode(mode)
        return SetModeResponse(success=True, message=f'Mode set to {mode}')

    def run(self):
        """Main run loop."""
        rospy.loginfo("[FollowerNode] Running...")
        rospy.spin()


def main():
    """Entry point."""
    try:
        node = FollowerNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr("[FollowerNode] Fatal error: %s", e)
        raise


if __name__ == '__main__':
    main()
