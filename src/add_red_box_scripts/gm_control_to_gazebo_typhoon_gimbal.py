#!/usr/bin/env python3
"""
Bridge namespaced gm_control messages to the Gazebo typhoon_h480 CGO3 gimbal.

This script is intentionally kept outside gm_control. It is only one backend
adapter for the current PX4/Gazebo test vehicle. Other gimbals can use different
adapters while keeping the gm_control image controller unchanged.
"""

import argparse
import math
import os
import threading
import time
from dataclasses import dataclass

# The native Gazebo gimbal controller uses MAVLink 2 messages.
os.environ.setdefault("MAVLINK20", "1")

import rospy
import tf2_ros
from gm_control.msg import BoundingBox2D, GimbalCommand, GimbalState
from gazebo_msgs.msg import ModelStates
from mavros_msgs.msg import State as MavrosState
from nav_msgs.msg import Odometry
from pymavlink import mavutil
from sensor_msgs.msg import Imu
from xd_uav_track.msg import DetectionArray

mavutil.set_dialect("common")


@dataclass
class LatestCommand:
    msg: GimbalCommand = None
    stamp: float = 0.0


def deg_to_rad(value: float) -> float:
    return math.radians(value)


def slew_towards(current: float, target: float, max_change: float) -> float:
    """Move current toward target without changing by more than max_change."""
    if max_change <= 0.0:
        return target
    delta = max(-max_change, min(max_change, target - current))
    return current + delta


def clamp(value: float, limit: float) -> float:
    """Symmetrically clamp value when limit is positive."""
    if limit <= 0.0:
        return value
    return max(-limit, min(limit, value))


def shortest_angle_deg(target: float, current: float) -> float:
    """Shortest signed angular error from current to target in degrees."""
    error = (target - current + 180.0) % 360.0 - 180.0
    return error


def rotate_vector_by_quaternion(vector, quaternion):
    """Rotate a three-vector by a ROS quaternion (x, y, z, w)."""
    vx, vy, vz = vector
    qx, qy, qz, qw = quaternion
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    return (
        vx + qw * tx + qy * tz - qz * ty,
        vy + qw * ty + qz * tx - qx * tz,
        vz + qw * tz + qx * ty - qy * tx,
    )


def line_of_sight_rates_deg(relative, relative_velocity):
    """Return inertial azimuth/elevation rates and range for one LOS vector."""
    rx, ry, rz = relative
    vx, vy, vz = relative_velocity
    horizontal_squared = rx * rx + ry * ry
    range_squared = horizontal_squared + rz * rz
    if range_squared <= 1.0e-6:
        return None

    horizontal = math.sqrt(horizontal_squared)
    # Azimuth is undefined directly above/below the target. Set only that
    # component to zero at the singularity; this is numerical handling, not an
    # aircraft/target collision constraint.
    yaw_rate = (
        (rx * vy - ry * vx) / horizontal_squared
        if horizontal_squared > 0.25
        else 0.0
    )
    if horizontal > 0.5:
        horizontal_rate = (rx * vx + ry * vy) / horizontal
        pitch_rate = (horizontal * vz - rz * horizontal_rate) / range_squared
    else:
        pitch_rate = 0.0
    return math.degrees(yaw_rate), math.degrees(pitch_rate), math.sqrt(range_squared)


def target_angles_from_body_vector(relative_body):
    """Return gimbal yaw/pitch angles for a nadir-zero fixed-wing camera.

    The fixed-wing SDF mounts the camera optical axis along body -Z at zero
    pitch.  Positive pitch rotates it toward body +X (the forward horizon),
    while yaw is the usual body +Z azimuth.  This is only used by the optional
    geometric pointing path; the legacy image-error path is unchanged.
    """
    bx, by, bz = relative_body
    horizontal = math.hypot(bx, by)
    distance = math.sqrt(bx * bx + by * by + bz * bz)
    if distance <= 1.0e-6:
        return None
    return (
        math.degrees(math.atan2(by, bx)),
        math.degrees(math.atan2(horizontal, -bz)),
        distance,
    )


def quaternion_from_euler(roll_rad: float, pitch_rad: float, yaw_rad: float):
    """Return a MAVLink quaternion in [w, x, y, z] order."""
    cr = math.cos(0.5 * roll_rad)
    sr = math.sin(0.5 * roll_rad)
    cp = math.cos(0.5 * pitch_rad)
    sp = math.sin(0.5 * pitch_rad)
    cy = math.cos(0.5 * yaw_rad)
    sy = math.sin(0.5 * yaw_rad)
    return [
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Apply gm_control/GimbalCommand to the Typhoon H480 Gazebo gimbal."
    )
    parser.add_argument("--command-topic", default="/uav1/gm_control/gimbal_cmd")
    parser.add_argument("--state-topic", default="/uav1/gm_control/gimbal_state")
    parser.add_argument("--frame-id", default="uav1/cgo3_gimbal")
    parser.add_argument("--rate", type=float, default=50.0)
    parser.add_argument("--command-timeout", type=float, default=0.5)
    parser.add_argument("--invalid-action", choices=["hold", "center"], default="hold")
    parser.add_argument(
        "--neutral-when-invalid",
        action="store_true",
        help=(
            "send MAVLink NEUTRAL before the first valid command so the native "
            "Gazebo gimbal applies no joint PID torque during takeoff"
        ),
    )
    parser.add_argument(
        "--direct-rate",
        action="store_true",
        help="send rate-mode commands as MAVLink angular velocities (for continuous yaw)",
    )
    parser.add_argument(
        "--body-rate-feedforward",
        action="store_true",
        help="compensate rate-mode commands for aircraft body angular velocity",
    )
    parser.add_argument(
        "--body-rate-topic",
        default="",
        help="sensor_msgs/Imu topic used by body-rate feedforward",
    )
    parser.add_argument("--body-rate-timeout", type=float, default=0.2)
    parser.add_argument("--body-rate-yaw-gain", type=float, default=1.0)
    parser.add_argument("--body-rate-pitch-gain", type=float, default=1.0)
    parser.add_argument("--body-rate-roll-gain", type=float, default=1.0)

    # Optional fixed-wing translational feedforward. The configured target is
    # expressed in target-frame and transformed through the existing world TF,
    # so the adapter does not duplicate the world manager's WGS84 conversion.
    parser.add_argument("--target-los-feedforward", action="store_true")
    parser.add_argument(
        "--target-position",
        type=float,
        nargs=3,
        metavar=("X", "Y", "Z"),
        help="static fallback target position; Gazebo model lookup takes priority",
    )
    parser.add_argument(
        "--target-frame",
        default="world",
        help="frame of the static --target-position fallback",
    )
    parser.add_argument(
        "--target-model",
        default="",
        help="exact Gazebo model name used as the live target position",
    )
    parser.add_argument(
        "--target-model-prefix",
        default="",
        help="Gazebo model-name prefix; the first PREFIX_N model is selected",
    )
    parser.add_argument(
        "--gazebo-model-states-topic",
        default="/gazebo/model_states",
        help="gazebo_msgs/ModelStates topic used for live target position",
    )
    parser.add_argument(
        "--target-model-frame",
        default="world",
        help="frame in which Gazebo model states are interpreted",
    )
    parser.add_argument("--target-model-timeout", type=float, default=1.0)
    parser.add_argument("--world-frame", default="world")
    parser.add_argument("--vehicle-odom-topic", default="")
    parser.add_argument("--vehicle-state-timeout", type=float, default=0.3)
    parser.add_argument(
        "--vehicle-state-topic",
        default="",
        help=(
            "optional mavros_msgs/State topic; when set, gimbal commands are "
            "blocked until the vehicle is armed"
        ),
    )
    parser.add_argument(
        "--gimbal-enable-altitude",
        type=float,
        default=0.0,
        help=(
            "hold the native gimbal neutral until world-frame odometry reaches "
            "this altitude in metres; zero disables the takeoff gate"
        ),
    )
    parser.add_argument(
        "--gimbal-flight-state-timeout",
        type=float,
        default=0.5,
        help="maximum age of the armed/altitude telemetry used by the takeoff gate",
    )
    parser.add_argument("--target-transform-timeout", type=float, default=0.05)
    parser.add_argument("--target-los-prediction-horizon", type=float, default=0.15)
    parser.add_argument("--target-los-filter-alpha", type=float, default=0.25)
    parser.add_argument("--vehicle-velocity-filter-alpha", type=float, default=0.25)
    parser.add_argument("--target-los-yaw-gain", type=float, default=1.0)
    parser.add_argument("--target-los-pitch-gain", type=float, default=1.0)
    parser.add_argument("--target-los-yaw-sign", type=float, default=1.0)
    parser.add_argument("--target-los-pitch-sign", type=float, default=1.0)
    parser.add_argument("--target-los-max-yaw-rate-deg-s", type=float, default=0.0)
    parser.add_argument("--target-los-max-pitch-rate-deg-s", type=float, default=0.0)
    parser.add_argument(
        "--target-pointing-feedforward",
        action="store_true",
        help=(
            "use the configured target world position and vehicle attitude to "
            "add an absolute geometric pointing correction"
        ),
    )
    parser.add_argument(
        "--target-pointing-yaw-gain",
        type=float,
        default=2.0,
        help="geometric yaw error gain in 1/s",
    )
    parser.add_argument(
        "--target-pointing-pitch-gain",
        type=float,
        default=2.0,
        help="geometric pitch error gain in 1/s",
    )
    parser.add_argument(
        "--target-pointing-yaw-rate-gain",
        type=float,
        default=1.0,
        help="gain applied to the measured body-frame target yaw rate",
    )
    parser.add_argument(
        "--target-pointing-pitch-rate-gain",
        type=float,
        default=1.0,
        help="gain applied to the measured body-frame target pitch rate",
    )
    parser.add_argument(
        "--target-pointing-max-yaw-rate-deg-s",
        type=float,
        default=0.0,
        help="optional clamp for the geometric yaw correction",
    )
    parser.add_argument(
        "--target-pointing-max-pitch-rate-deg-s",
        type=float,
        default=0.0,
        help="optional clamp for the geometric pitch correction",
    )
    parser.add_argument("--max-output-yaw-rate-deg-s", type=float, default=0.0)
    parser.add_argument("--max-output-pitch-rate-deg-s", type=float, default=0.0)
    parser.add_argument("--max-output-roll-rate-deg-s", type=float, default=0.0)

    # Optional external detector adapter. When configured, this script converts
    # the repository's existing DetectionArray directly to gm_control's existing
    # BoundingBox2D input. gm_control's own bbox_tracker is then unnecessary.
    parser.add_argument("--detections-topic", default="")
    parser.add_argument("--bbox-topic", default="")
    parser.add_argument("--detection-timeout", type=float, default=0.5)
    parser.add_argument("--detection-min-confidence", type=float, default=0.1)
    parser.add_argument("--detection-class-id", type=int, default=-1)
    parser.add_argument("--detection-target-id", type=int, default=-1)

    parser.add_argument("--gimbal-host", default="127.0.0.1")
    parser.add_argument("--gimbal-port", type=int, required=True,
                        help="fixed UDP port of the native Gazebo gimbal controller")

    parser.add_argument("--initial-yaw-deg", type=float, default=0.0)
    parser.add_argument("--initial-pitch-deg", type=float, default=0.0)
    parser.add_argument("--initial-roll-deg", type=float, default=0.0)

    parser.add_argument("--yaw-sign", type=float, default=1.0)
    parser.add_argument("--pitch-sign", type=float, default=1.0)
    parser.add_argument("--roll-sign", type=float, default=1.0)

    parser.add_argument("--yaw-offset-deg", type=float, default=0.0)
    parser.add_argument("--pitch-offset-deg", type=float, default=0.0)
    parser.add_argument("--roll-offset-deg", type=float, default=0.0)
    parser.add_argument(
        "--yaw-lock",
        action="store_true",
        help=(
            "send a yaw-lock command to the native Gazebo gimbal; use this "
            "for fixed-wing payloads to avoid vehicle-yaw target jumps"
        ),
    )
    parser.add_argument(
        "--yaw-lock-offset-deg",
        type=float,
        default=-90.0,
        help=(
            "MAVLink yaw quaternion offset used with --yaw-lock. The native "
            "Gazebo plugin adds +90 degrees in yaw-lock mode, so the default "
            "-90 degrees cancels that convention"
        ),
    )

    # Optional command shaping. Zero keeps the original immediate-response
    # behavior, so existing multirotor sessions are not changed implicitly.
    parser.add_argument("--max-yaw-accel-deg-s2", type=float, default=0.0)
    parser.add_argument("--max-pitch-accel-deg-s2", type=float, default=0.0)
    parser.add_argument("--max-roll-accel-deg-s2", type=float, default=0.0)
    parser.add_argument("--max-yaw-slew-deg-s", type=float, default=0.0)
    parser.add_argument("--max-pitch-slew-deg-s", type=float, default=0.0)
    parser.add_argument("--max-roll-slew-deg-s", type=float, default=0.0)
    parser.add_argument(
        "--max-integration-dt",
        type=float,
        default=0.0,
        help="cap rate integration time after a delayed loop; zero disables the cap",
    )

    parser.add_argument("--log-interval", type=float, default=1.0)
    return parser.parse_args()


class CommandBuffer:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._latest = LatestCommand()

    def callback(self, msg: GimbalCommand) -> None:
        with self._lock:
            self._latest = LatestCommand(msg=msg, stamp=time.time())

    def get(self) -> LatestCommand:
        with self._lock:
            return self._latest


@dataclass
class LatestBodyRate:
    roll_deg_s: float = 0.0
    pitch_deg_s: float = 0.0
    yaw_deg_s: float = 0.0
    stamp: float = 0.0


class BodyRateBuffer:
    """Keep stabilized aircraft angular rates for optional feedforward.

    ``sensor_msgs/Imu.angular_velocity`` is a body-frame angular velocity,
    not a set of roll/pitch/yaw Euler-angle rates.  In a banked fixed-wing
    turn, for example, a large body ``q`` component can be caused almost
    entirely by yawing about the world vertical axis.  The gimbal axes are
    stabilized Euler axes, so use the attitude quaternion to perform the
    body-rate -> ZYX Euler-rate conversion before exposing the values.
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._latest = LatestBodyRate()

    def callback(self, msg: Imu) -> None:
        p = float(msg.angular_velocity.x)
        q = float(msg.angular_velocity.y)
        r = float(msg.angular_velocity.z)
        if not all(math.isfinite(value) for value in (p, q, r)):
            return

        # ROS IMU orientation is already in the FLU/ENU convention used by
        # the adapter.  Fall back to the raw gyro if the estimator has not
        # published a valid quaternion yet.
        qw = float(msg.orientation.w)
        qx = float(msg.orientation.x)
        qy = float(msg.orientation.y)
        qz = float(msg.orientation.z)
        norm = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
        if norm > 1.0e-6 and all(math.isfinite(v) for v in (qw, qx, qy, qz)):
            qw /= norm
            qx /= norm
            qy /= norm
            qz /= norm
            roll = math.atan2(
                2.0 * (qw * qx + qy * qz),
                1.0 - 2.0 * (qx * qx + qy * qy),
            )
            pitch_arg = max(
                -1.0,
                min(1.0, 2.0 * (qw * qy - qz * qx)),
            )
            pitch = math.asin(pitch_arg)
            c_pitch = math.cos(pitch)
            if abs(c_pitch) > 1.0e-4:
                # ZYX Euler-rate mapping: [p q r] -> [roll_dot pitch_dot yaw_dot].
                roll_rate = p + math.tan(pitch) * (
                    q * math.sin(roll) + r * math.cos(roll)
                )
                pitch_rate = q * math.cos(roll) - r * math.sin(roll)
                yaw_rate = (
                    q * math.sin(roll) + r * math.cos(roll)
                ) / c_pitch
            else:
                roll_rate, pitch_rate, yaw_rate = p, q, r
        else:
            roll_rate, pitch_rate, yaw_rate = p, q, r

        with self._lock:
            self._latest = LatestBodyRate(
                roll_deg_s=math.degrees(roll_rate),
                pitch_deg_s=math.degrees(pitch_rate),
                yaw_deg_s=math.degrees(yaw_rate),
                stamp=time.time(),
            )

    def get(self, now: float, timeout: float) -> LatestBodyRate:
        with self._lock:
            latest = self._latest
        if latest.stamp <= 0.0 or now - latest.stamp > max(0.0, timeout):
            return LatestBodyRate()
        return latest


@dataclass
class LatestFlightState:
    armed: bool = False
    altitude_m: float = float("nan")
    state_receipt_time: float = 0.0
    odom_receipt_time: float = 0.0


class FlightReadinessBuffer:
    """Gate gimbal torque until a fixed-wing vehicle is safely airborne.

    gm_control can publish a valid search/tracking rate as soon as its ROS
    services are called.  Forwarding that command while the plane is still on
    the ground lets the payload accelerate and feed reaction torque into the
    airframe during the most sensitive part of takeoff.  This buffer is an
    optional adapter-side gate: it is inactive unless a state topic or a
    positive altitude threshold is supplied, so ordinary multirotor sessions
    retain their previous behaviour.
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._latest = LatestFlightState()

    def state_callback(self, msg: MavrosState) -> None:
        with self._lock:
            latest = self._latest
            self._latest = LatestFlightState(
                armed=bool(msg.armed),
                altitude_m=latest.altitude_m,
                state_receipt_time=time.time(),
                odom_receipt_time=latest.odom_receipt_time,
            )

    def odom_callback(self, msg: Odometry) -> None:
        altitude = float(msg.pose.pose.position.z)
        if not math.isfinite(altitude):
            return
        with self._lock:
            latest = self._latest
            self._latest = LatestFlightState(
                armed=latest.armed,
                altitude_m=altitude,
                state_receipt_time=latest.state_receipt_time,
                odom_receipt_time=time.time(),
            )

    def get(self) -> LatestFlightState:
        with self._lock:
            latest = self._latest
            return LatestFlightState(
                armed=latest.armed,
                altitude_m=latest.altitude_m,
                state_receipt_time=latest.state_receipt_time,
                odom_receipt_time=latest.odom_receipt_time,
            )

    def ready(self, now: float, args: argparse.Namespace) -> bool:
        """Return whether native gimbal torque may be enabled this cycle."""
        requires_state = bool(args.vehicle_state_topic)
        altitude_threshold = max(0.0, float(args.gimbal_enable_altitude))
        if not requires_state and altitude_threshold <= 0.0:
            return True

        latest = self.get()
        timeout = max(0.0, float(args.gimbal_flight_state_timeout))
        if requires_state:
            if latest.state_receipt_time <= 0.0:
                return False
            if timeout > 0.0 and now - latest.state_receipt_time > timeout:
                return False
            if not latest.armed:
                return False

        if altitude_threshold > 0.0:
            if latest.odom_receipt_time <= 0.0:
                return False
            if timeout > 0.0 and now - latest.odom_receipt_time > timeout:
                return False
            if not math.isfinite(latest.altitude_m):
                return False
            if latest.altitude_m < altitude_threshold:
                return False
        return True

    def summary(self) -> LatestFlightState:
        return self.get()


@dataclass
class LatestVehicleState:
    position: tuple = (0.0, 0.0, 0.0)
    velocity: tuple = (0.0, 0.0, 0.0)
    # ROS pose orientation, quaternion [x, y, z, w], body expressed in world.
    orientation: tuple = (0.0, 0.0, 0.0, 1.0)
    receipt_time: float = 0.0
    sample_time: float = 0.0


class WorldVehicleStateBuffer:
    """Estimate world velocity from consecutive world-frame positions.

    The estimator's republished Odometry pose is transformed to world, while
    older versions may leave the twist in its source frame. Position
    differentiation avoids silently mixing those frames in LOS feedforward.
    """

    def __init__(self, world_frame: str, velocity_alpha: float) -> None:
        self._lock = threading.Lock()
        self._latest = LatestVehicleState()
        self._world_frame = world_frame.strip("/")
        self._velocity_alpha = max(0.0, min(1.0, velocity_alpha))
        self._warned_frame = False

    def callback(self, msg: Odometry) -> None:
        frame = str(msg.header.frame_id).strip("/")
        if frame and frame != self._world_frame:
            if not self._warned_frame:
                rospy.logwarn(
                    "target LOS feedforward ignores odometry frame '%s'; expected '%s'",
                    frame,
                    self._world_frame,
                )
                self._warned_frame = True
            return

        position = (
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
            float(msg.pose.pose.position.z),
        )
        if not all(math.isfinite(value) for value in position):
            return
        orientation = (
            float(msg.pose.pose.orientation.x),
            float(msg.pose.pose.orientation.y),
            float(msg.pose.pose.orientation.z),
            float(msg.pose.pose.orientation.w),
        )
        if not all(math.isfinite(value) for value in orientation):
            orientation = (0.0, 0.0, 0.0, 1.0)
        orientation_norm = math.sqrt(sum(value * value for value in orientation))
        if orientation_norm <= 1.0e-6:
            orientation = (0.0, 0.0, 0.0, 1.0)
        else:
            orientation = tuple(value / orientation_norm for value in orientation)
        sample_time = (
            msg.header.stamp.to_sec()
            if msg.header.stamp != rospy.Time()
            else rospy.Time.now().to_sec()
        )
        receipt_time = time.time()
        with self._lock:
            previous = self._latest
            velocity = previous.velocity
            dt = sample_time - previous.sample_time
            if previous.sample_time > 0.0 and 1.0e-3 <= dt <= 1.0:
                raw_velocity = tuple(
                    (position[index] - previous.position[index]) / dt
                    for index in range(3)
                )
                alpha = self._velocity_alpha
                velocity = tuple(
                    previous.velocity[index]
                    + alpha * (raw_velocity[index] - previous.velocity[index])
                    for index in range(3)
                )
            elif previous.sample_time <= 0.0 or dt < 0.0 or dt > 1.0:
                velocity = (0.0, 0.0, 0.0)
            self._latest = LatestVehicleState(
                position=position,
                velocity=velocity,
                orientation=orientation,
                receipt_time=receipt_time,
                sample_time=sample_time,
            )

    def get(self) -> LatestVehicleState:
        with self._lock:
            latest = self._latest
            return LatestVehicleState(
                position=latest.position,
                velocity=latest.velocity,
                orientation=latest.orientation,
                receipt_time=latest.receipt_time,
                sample_time=latest.sample_time,
            )


@dataclass
class LosFeedforward:
    valid: bool = False
    yaw_rate_deg_s: float = 0.0
    pitch_rate_deg_s: float = 0.0
    distance_m: float = 0.0


@dataclass
class TargetPointing:
    valid: bool = False
    yaw_deg: float = 0.0
    pitch_deg: float = 0.0
    distance_m: float = 0.0
    # Derivative of the desired body-frame angles.  Unlike the inertial LOS
    # rate above, this already contains the aircraft attitude motion and the
    # translational motion of the vehicle relative to the fixed target.
    yaw_rate_deg_s: float = 0.0
    pitch_rate_deg_s: float = 0.0


@dataclass
class LatestGazeboTarget:
    position: tuple = (0.0, 0.0, 0.0)
    model_name: str = ""
    receipt_time: float = 0.0


class GazeboTargetBuffer:
    """Track the generated box pose directly from Gazebo ModelStates.

    ``spawn_red_boxes.py`` creates models named ``<prefix>_<index>`` and the
    Gazebo service does not publish the input coordinates as a ROS message.
    ModelStates is therefore the authoritative live position source for this
    adapter.  Selecting by exact name is preferred; a prefix selects the
    lowest numbered matching model for the single-target fixed-wing test.
    """

    def __init__(self, args: argparse.Namespace) -> None:
        self._lock = threading.Lock()
        self._latest = LatestGazeboTarget()
        self.model_name = args.target_model.strip()
        self.model_prefix = args.target_model_prefix.strip()
        self.timeout = max(0.0, float(args.target_model_timeout))
        self._last_wait_log = 0.0
        rospy.Subscriber(
            args.gazebo_model_states_topic,
            ModelStates,
            self.callback,
            queue_size=1,
        )

    def callback(self, message: ModelStates) -> None:
        names = list(message.name)
        selected_index = None
        if self.model_name:
            try:
                selected_index = names.index(self.model_name)
            except ValueError:
                selected_index = None
        elif self.model_prefix:
            candidates = [
                (index, name)
                for index, name in enumerate(names)
                if name.startswith(self.model_prefix + "_")
            ]
            if candidates:
                # Natural ordering keeps PREFIX_1 ahead of PREFIX_10.
                selected_index = min(
                    candidates,
                    key=lambda item: (
                        int(item[1][len(self.model_prefix) + 1 :])
                        if item[1][len(self.model_prefix) + 1 :].isdigit()
                        else 2**31,
                        item[1],
                    ),
                )[0]
        if selected_index is None or selected_index >= len(message.pose):
            return
        pose = message.pose[selected_index].position
        position = (float(pose.x), float(pose.y), float(pose.z))
        if not all(math.isfinite(value) for value in position):
            return
        with self._lock:
            self._latest = LatestGazeboTarget(
                position=position,
                model_name=names[selected_index],
                receipt_time=time.time(),
            )

    def get(self, now: float):
        with self._lock:
            latest = self._latest
        if (
            latest.receipt_time <= 0.0
            or (self.timeout > 0.0 and now - latest.receipt_time > self.timeout)
        ):
            return None
        return latest


class TargetLosFeedforward:
    """Compute target line-of-sight guidance from a live or static target."""

    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.world_frame = args.world_frame.strip("/")
        self.target_frame = args.target_frame.strip("/")
        self.target_model_frame = args.target_model_frame.strip("/")
        self.target_position = (
            tuple(float(value) for value in args.target_position)
            if args.target_position is not None
            else None
        )
        self.target_world = None
        self._last_model_name = ""
        self.gazebo_target = (
            GazeboTargetBuffer(args)
            if args.target_model or args.target_model_prefix
            else None
        )
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.vehicle_buffer = WorldVehicleStateBuffer(
            self.world_frame, args.vehicle_velocity_filter_alpha
        )
        self._filtered_yaw_rate = 0.0
        self._filtered_pitch_rate = 0.0
        self._last_pointing = None
        self._last_pointing_time = 0.0
        self._filtered_pointing_yaw_rate = 0.0
        self._filtered_pointing_pitch_rate = 0.0
        self._last_wait_log = 0.0
        rospy.Subscriber(
            args.vehicle_odom_topic,
            Odometry,
            self.vehicle_buffer.callback,
            queue_size=1,
        )

    def _resolve_target_world(self, now: float):
        # A live Gazebo model always takes priority over a static fallback.
        # ModelStates poses are already expressed in Gazebo's world frame; the
        # test launch therefore requires --target-model-frame to match the
        # configured world frame instead of attempting an unsafe implicit TF
        # conversion.
        if self.gazebo_target is not None:
            latest = self.gazebo_target.get(now)
            if latest is not None:
                if self.target_model_frame != self.world_frame:
                    if now - self._last_wait_log >= 2.0:
                        rospy.logwarn(
                            "Gazebo target model '%s' is in frame '%s'; expected world frame '%s'",
                            latest.model_name,
                            self.target_model_frame,
                            self.world_frame,
                        )
                        self._last_wait_log = now
                    return None
                if latest.model_name != self._last_model_name:
                    rospy.loginfo(
                        "target LOS position from Gazebo model '%s': %s (%.3f, %.3f, %.3f)",
                        latest.model_name,
                        self.world_frame,
                        latest.position[0],
                        latest.position[1],
                        latest.position[2],
                    )
                    self._last_model_name = latest.model_name
                self.target_world = latest.position
                return self.target_world
            if self.target_position is None:
                if now - self._last_wait_log >= 2.0:
                    selector = self.args.target_model or (self.args.target_model_prefix + "_N")
                    rospy.logwarn(
                        "waiting for Gazebo target model '%s' on %s",
                        selector,
                        self.args.gazebo_model_states_topic,
                    )
                    self._last_wait_log = now
                return None

        if self.target_position is None:
            return None
        if self.target_world is not None:
            return self.target_world
        if self.target_frame == self.world_frame:
            self.target_world = self.target_position
        else:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.world_frame,
                    self.target_frame,
                    rospy.Time(0),
                    rospy.Duration(max(0.0, self.args.target_transform_timeout)),
                )
            except (
                tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException,
            ) as error:
                now = time.time()
                if now - self._last_wait_log >= 2.0:
                    rospy.logwarn(
                        "target LOS feedforward waits for TF %s <- %s: %s",
                        self.world_frame,
                        self.target_frame,
                        error,
                    )
                    self._last_wait_log = now
                return None
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            rotated = rotate_vector_by_quaternion(
                self.target_position,
                (rotation.x, rotation.y, rotation.z, rotation.w),
            )
            self.target_world = (
                float(translation.x) + rotated[0],
                float(translation.y) + rotated[1],
                float(translation.z) + rotated[2],
            )
        rospy.loginfo(
            "target LOS position: source=%s (%.3f, %.3f, %.3f) -> %s (%.3f, %.3f, %.3f)",
            self.target_frame,
            self.target_position[0],
            self.target_position[1],
            self.target_position[2],
            self.world_frame,
            self.target_world[0],
            self.target_world[1],
            self.target_world[2],
        )
        return self.target_world

    def get(self, now: float) -> LosFeedforward:
        target = self._resolve_target_world(now)
        state = self.vehicle_buffer.get()
        if (
            target is None
            or state.receipt_time <= 0.0
            or now - state.receipt_time > max(0.0, self.args.vehicle_state_timeout)
        ):
            return LosFeedforward()

        horizon = max(0.0, self.args.target_los_prediction_horizon)
        predicted_position = tuple(
            state.position[index] + horizon * state.velocity[index]
            for index in range(3)
        )
        relative = tuple(
            target[index] - predicted_position[index] for index in range(3)
        )
        relative_velocity = tuple(-value for value in state.velocity)
        rates = line_of_sight_rates_deg(relative, relative_velocity)
        if rates is None:
            return LosFeedforward()
        yaw_rate_deg_s, pitch_rate_deg_s, distance_m = rates
        yaw_rate_deg_s = clamp(
            yaw_rate_deg_s, self.args.target_los_max_yaw_rate_deg_s
        )
        pitch_rate_deg_s = clamp(
            pitch_rate_deg_s, self.args.target_los_max_pitch_rate_deg_s
        )
        alpha = max(0.0, min(1.0, self.args.target_los_filter_alpha))
        self._filtered_yaw_rate += alpha * (
            yaw_rate_deg_s - self._filtered_yaw_rate
        )
        self._filtered_pitch_rate += alpha * (
            pitch_rate_deg_s - self._filtered_pitch_rate
        )
        return LosFeedforward(
            valid=True,
            yaw_rate_deg_s=self._filtered_yaw_rate,
            pitch_rate_deg_s=self._filtered_pitch_rate,
            distance_m=distance_m,
        )

    def get_pointing(self, now: float) -> TargetPointing:
        """Return absolute target angles in the fixed-wing vehicle frame.

        This is deliberately separate from ``get()``: the existing LOS rate
        feedforward remains available on its own, while this optional path can
        provide a geometric catch-up term when image detections arrive late or
        disappear for a few frames.
        """
        target = self._resolve_target_world(now)
        state = self.vehicle_buffer.get()
        if (
            target is None
            or state.receipt_time <= 0.0
            or now - state.receipt_time > max(0.0, self.args.vehicle_state_timeout)
        ):
            return TargetPointing()

        # Use the same short prediction horizon as LOS feedforward.  Image
        # transport, detector inference and ROS scheduling together can easily
        # consume 100--150 ms on a fast fixed-wing pass; pointing at the
        # current body pose alone therefore trails the target by that amount.
        horizon = max(0.0, self.args.target_los_prediction_horizon)
        predicted_position = tuple(
            state.position[index] + horizon * state.velocity[index]
            for index in range(3)
        )
        relative_world = tuple(
            target[index] - predicted_position[index] for index in range(3)
        )
        qx, qy, qz, qw = state.orientation
        # Odometry stores the body orientation in the world frame.  The
        # conjugate therefore rotates an inertial/world vector into body FLU.
        relative_body = rotate_vector_by_quaternion(
            relative_world, (-qx, -qy, -qz, qw)
        )
        angles = target_angles_from_body_vector(relative_body)
        if angles is None:
            return TargetPointing()
        yaw_deg, pitch_deg, distance_m = angles

        # Differentiate the *body-frame* pointing angles.  This is the rate
        # the gimbal setpoint itself must move at in yaw-lock mode; it includes
        # aircraft yaw/pitch/roll and the apparent motion caused by forward
        # flight.  Using the body-frame angle derivative avoids adding the
        # same body-rate compensation a second time below.
        yaw_rate_deg_s = 0.0
        pitch_rate_deg_s = 0.0
        sample_time = state.sample_time if state.sample_time > 0.0 else now
        if self._last_pointing is not None:
            dt = sample_time - self._last_pointing_time
            if 1.0e-3 <= dt <= 1.0:
                previous_yaw, previous_pitch = self._last_pointing
                yaw_rate_deg_s = shortest_angle_deg(yaw_deg, previous_yaw) / dt
                pitch_rate_deg_s = (pitch_deg - previous_pitch) / dt
        self._last_pointing = (yaw_deg, pitch_deg)
        self._last_pointing_time = sample_time
        alpha = max(0.0, min(1.0, self.args.target_los_filter_alpha))
        self._filtered_pointing_yaw_rate += alpha * (
            yaw_rate_deg_s - self._filtered_pointing_yaw_rate
        )
        self._filtered_pointing_pitch_rate += alpha * (
            pitch_rate_deg_s - self._filtered_pointing_pitch_rate
        )
        return TargetPointing(
            valid=True,
            yaw_deg=yaw_deg,
            pitch_deg=pitch_deg,
            distance_m=distance_m,
            yaw_rate_deg_s=self._filtered_pointing_yaw_rate,
            pitch_rate_deg_s=self._filtered_pointing_pitch_rate,
        )


class ExternalDetectionBridge:
    """Select one external DetectionArray candidate and publish the existing GM bbox."""

    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self._lock = threading.Lock()
        self._selected_id = None
        self._last_valid_time = 0.0
        self._invalid_published = True
        self.publisher = rospy.Publisher(
            args.bbox_topic, BoundingBox2D, queue_size=5
        )
        rospy.Subscriber(
            args.detections_topic,
            DetectionArray,
            self.callback,
            queue_size=1,
        )
        rospy.loginfo(
            "external detector bridge enabled: detections=%s bbox=%s",
            args.detections_topic,
            args.bbox_topic,
        )

    @staticmethod
    def _candidate_bbox(candidate, message):
        if candidate.has_bbox:
            x_min, y_min, x_max, y_max = (float(value) for value in candidate.bbox)
            return x_min, y_min, x_max - x_min, y_max - y_min
        if (
            candidate.has_normalized_bbox
            and message.image_width > 0
            and message.image_height > 0
        ):
            center_x, center_y, width, height = (
                float(value) for value in candidate.normalized_bbox
            )
            width *= float(message.image_width)
            height *= float(message.image_height)
            return (
                center_x * float(message.image_width) - 0.5 * width,
                center_y * float(message.image_height) - 0.5 * height,
                width,
                height,
            )
        return None

    def callback(self, message: DetectionArray) -> None:
        now = time.time()
        candidates = []
        for candidate in message.candidates:
            if (
                candidate.confidence < self.args.detection_min_confidence
                or (
                    self.args.detection_class_id >= 0
                    and candidate.class_id != self.args.detection_class_id
                )
                or (
                    self.args.detection_target_id >= 0
                    and candidate.track_id != self.args.detection_target_id
                )
            ):
                continue
            bbox = self._candidate_bbox(candidate, message)
            if bbox is None or not all(math.isfinite(value) for value in bbox):
                continue
            if bbox[2] <= 0.0 or bbox[3] <= 0.0:
                continue
            candidates.append((candidate, bbox))

        with self._lock:
            selected = next(
                (
                    item
                    for item in candidates
                    if self._selected_id is not None
                    and item[0].track_id == self._selected_id
                ),
                None,
            )
            if selected is None:
                # Do not jump to another object during a brief detector gap.
                if (
                    self._selected_id is not None
                    and now - self._last_valid_time <= self.args.detection_timeout
                ):
                    return
                selected = max(
                    candidates,
                    key=lambda item: float(item[0].confidence),
                    default=None,
                )
            if selected is None:
                return
            candidate, bbox = selected
            self._selected_id = (
                int(candidate.track_id) if candidate.track_id_is_stable else None
            )
            self._last_valid_time = now
            self._invalid_published = False

        output = BoundingBox2D()
        output.header = message.header
        if output.header.stamp == rospy.Time():
            output.header.stamp = rospy.Time.now()
        output.valid = True
        output.x, output.y, output.width, output.height = bbox
        output.confidence = float(candidate.confidence)
        output.target_id = str(candidate.track_id)
        self.publisher.publish(output)

    def tick(self, now: float) -> None:
        with self._lock:
            if (
                self._invalid_published
                or self._last_valid_time <= 0.0
                or now - self._last_valid_time <= self.args.detection_timeout
            ):
                return
            self._invalid_published = True
            self._selected_id = None
        output = BoundingBox2D()
        output.header.stamp = rospy.Time.now()
        output.valid = False
        self.publisher.publish(output)


class NativeGimbalSender:
    """Send angle setpoints to the native Gazebo gimbal PID controller."""

    def __init__(self, args: argparse.Namespace) -> None:
        self.link = mavutil.mavlink_connection(
            "udpout:{0}:{1}".format(args.gimbal_host, args.gimbal_port),
            source_system=255,
            source_component=mavutil.mavlink.MAV_COMP_ID_ONBOARD_COMPUTER,
        )
        # Roll and pitch are treated as lock-frame angles by the native plugin.
        # Keep yaw follow/body mode for the legacy Typhoon path. Fixed-wing
        # launches opt into yaw lock because the native plugin otherwise adds
        # vehicleYawRad to the setpoint; the first autopilot attitude update
        # can then create a large gimbal step and reaction torque at takeoff.
        self.flags = (
            mavutil.mavlink.GIMBAL_DEVICE_FLAGS_ROLL_LOCK
            | mavutil.mavlink.GIMBAL_DEVICE_FLAGS_PITCH_LOCK
        )
        if args.yaw_lock:
            self.flags |= mavutil.mavlink.GIMBAL_DEVICE_FLAGS_YAW_LOCK
        self.yaw_lock = bool(args.yaw_lock)
        self.yaw_lock_offset_deg = float(args.yaw_lock_offset_deg)
        rospy.loginfo(
            "Native Gazebo gimbal MAVLink endpoint: udp://%s:%d yaw_lock=%s",
            args.gimbal_host,
            args.gimbal_port,
            self.yaw_lock,
        )

    def send_angles(self, yaw_deg: float, pitch_deg: float, roll_deg: float) -> None:
        # The native plugin internally applies the Typhoon joint-axis signs:
        # physical roll = q.roll, physical pitch = -q.pitch, physical yaw = -q.yaw
        # (for yaw-follow mode). In yaw-lock mode the native plugin adds +90
        # degrees to its decoded yaw setpoint, so pre-compensate that fixed
        # convention offset. This keeps logical yaw=0 at the current physical
        # zero while removing the vehicleYawRad coupling.
        yaw_quaternion_deg = yaw_deg + (
            self.yaw_lock_offset_deg if self.yaw_lock else 0.0
        )
        q = quaternion_from_euler(
            deg_to_rad(roll_deg),
            -deg_to_rad(pitch_deg),
            -deg_to_rad(yaw_quaternion_deg),
        )
        self.link.mav.gimbal_device_set_attitude_send(
            0,
            0,
            self.flags,
            q,
            float("nan"),
            float("nan"),
            float("nan"),
        )

    def send_rates(self, yaw_rate_deg_s: float, pitch_rate_deg_s: float,
                   roll_rate_deg_s: float) -> None:
        """Send body-frame angular rates without an Euler/quaternion angle.

        The Gazebo plugin integrates finite angular_velocity fields while
        retaining its unwrapped internal setpoint.  Sending NaN for q is
        intentional: it prevents the plugin from re-normalizing yaw at the
        +/-180 degree quaternion seam.
        """
        self.link.mav.gimbal_device_set_attitude_send(
            0,
            0,
            self.flags,
            [float("nan")] * 4,
            deg_to_rad(roll_rate_deg_s),
            -deg_to_rad(pitch_rate_deg_s),
            -deg_to_rad(yaw_rate_deg_s),
        )

    def send_neutral(self) -> None:
        """Tell the native Gazebo plugin to release all gimbal PID torques."""
        self.link.mav.gimbal_device_set_attitude_send(
            0,
            0,
            self.flags | mavutil.mavlink.GIMBAL_DEVICE_FLAGS_NEUTRAL,
            quaternion_from_euler(0.0, 0.0, 0.0),
            float("nan"),
            float("nan"),
            float("nan"),
        )


class TyphoonGimbalAdapter:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.command_buffer = CommandBuffer()
        self.body_rate_buffer = BodyRateBuffer()
        self.flight_state_buffer = FlightReadinessBuffer()
        self.target_los = None
        self.external_detection_bridge = None
        # Logical angles use the same coordinate convention as /gm_control/gimbal_cmd.
        # yaw_sign/pitch_sign/roll_sign are applied only when converting to Gazebo
        # physical joint positions. This keeps /gm_control/gimbal_state compatible
        # with the controller's current_angle + delta_angle logic.
        self.yaw_deg = args.initial_yaw_deg
        self.pitch_deg = args.initial_pitch_deg
        self.roll_deg = args.initial_roll_deg
        self.yaw_rate_deg_s = 0.0
        self.pitch_rate_deg_s = 0.0
        self.roll_rate_deg_s = 0.0
        self._has_sent_valid_command = False
        # Rate transport is integrated by the native plugin from its last
        # absolute setpoint.  Re-anchor once after NEUTRAL or angle mode so a
        # yaw-lock convention offset cannot become a hidden jump.
        self._rate_mode_anchored = False
        self.last_update_time = time.time()
        self.last_log_time = 0.0

        self.state_pub = rospy.Publisher(args.state_topic, GimbalState, queue_size=10)
        rospy.Subscriber(args.command_topic, GimbalCommand, self.command_buffer.callback, queue_size=1)
        if args.vehicle_state_topic:
            rospy.Subscriber(
                args.vehicle_state_topic,
                MavrosState,
                self.flight_state_buffer.state_callback,
                queue_size=1,
            )
        if args.gimbal_enable_altitude > 0.0:
            if args.vehicle_odom_topic:
                rospy.Subscriber(
                    args.vehicle_odom_topic,
                    Odometry,
                    self.flight_state_buffer.odom_callback,
                    queue_size=1,
                )
            else:
                rospy.logwarn(
                    "--gimbal-enable-altitude requires --vehicle-odom-topic; "
                    "altitude gate will remain closed"
                )
        if args.vehicle_state_topic or args.gimbal_enable_altitude > 0.0:
            rospy.loginfo(
                "gimbal takeoff gate enabled: armed_topic=%s altitude=%.1fm timeout=%.2fs",
                args.vehicle_state_topic or "<disabled>",
                args.gimbal_enable_altitude,
                args.gimbal_flight_state_timeout,
            )
        if args.body_rate_feedforward:
            if args.body_rate_topic:
                rospy.Subscriber(args.body_rate_topic, Imu, self.body_rate_buffer.callback, queue_size=1)
                rospy.loginfo(
                    "Gimbal body-rate feedforward enabled: topic=%s gains=(yaw %.2f pitch %.2f roll %.2f)",
                    args.body_rate_topic,
                    args.body_rate_yaw_gain,
                    args.body_rate_pitch_gain,
                    args.body_rate_roll_gain,
                )
            else:
                rospy.logwarn("body-rate feedforward requested without --body-rate-topic; disabled")
        if args.target_los_feedforward or args.target_pointing_feedforward:
            target_source_configured = bool(
                args.target_position is not None
                or args.target_model
                or args.target_model_prefix
            )
            if not target_source_configured or not args.vehicle_odom_topic:
                rospy.logwarn(
                    "target pointing/LOS feedforward requires a target model "
                    "or --target-position plus --vehicle-odom-topic; disabled"
                )
            else:
                self.target_los = TargetLosFeedforward(args)
                rospy.loginfo(
                    "target geometry enabled: odometry=%s LOS=%s pointing=%s gains=(yaw %.2f pitch %.2f) horizon=%.2fs",
                    args.vehicle_odom_topic,
                    args.target_los_feedforward,
                    args.target_pointing_feedforward,
                    args.target_los_yaw_gain,
                    args.target_los_pitch_gain,
                    args.target_los_prediction_horizon,
                )
        if args.detections_topic or args.bbox_topic:
            if args.detections_topic and args.bbox_topic:
                self.external_detection_bridge = ExternalDetectionBridge(args)
            else:
                rospy.logwarn(
                    "external detector bridge requires both --detections-topic and --bbox-topic; disabled"
                )

        self.native_gimbal = NativeGimbalSender(args)
        rospy.loginfo(
            "gm_control native Gazebo gimbal adapter started: command_topic=%s state_topic=%s",
            args.command_topic,
            args.state_topic,
        )

    def update_angles_from_command(self, cmd: GimbalCommand, dt: float) -> None:
        if cmd.mode == GimbalCommand.MODE_ANGLE:
            prev_yaw = self.yaw_deg
            prev_pitch = self.pitch_deg
            prev_roll = self.roll_deg
            self.yaw_deg = slew_towards(
                self.yaw_deg,
                cmd.yaw_deg,
                max(0.0, self.args.max_yaw_slew_deg_s) * dt,
            )
            self.pitch_deg = slew_towards(
                self.pitch_deg,
                cmd.pitch_deg,
                max(0.0, self.args.max_pitch_slew_deg_s) * dt,
            )
            self.roll_deg = slew_towards(
                self.roll_deg,
                cmd.roll_deg,
                max(0.0, self.args.max_roll_slew_deg_s) * dt,
            )
            self.yaw_rate_deg_s = (self.yaw_deg - prev_yaw) / dt
            self.pitch_rate_deg_s = (self.pitch_deg - prev_pitch) / dt
            self.roll_rate_deg_s = (self.roll_deg - prev_roll) / dt
        else:
            self.yaw_rate_deg_s = slew_towards(
                self.yaw_rate_deg_s,
                cmd.yaw_rate_deg_s,
                max(0.0, self.args.max_yaw_accel_deg_s2) * dt,
            )
            self.pitch_rate_deg_s = slew_towards(
                self.pitch_rate_deg_s,
                cmd.pitch_rate_deg_s,
                max(0.0, self.args.max_pitch_accel_deg_s2) * dt,
            )
            self.roll_rate_deg_s = slew_towards(
                self.roll_rate_deg_s,
                cmd.roll_rate_deg_s,
                max(0.0, self.args.max_roll_accel_deg_s2) * dt,
            )
            self.yaw_deg += self.yaw_rate_deg_s * dt
            self.pitch_deg += self.pitch_rate_deg_s * dt
            self.roll_deg += self.roll_rate_deg_s * dt

        # Do not impose an application-level angle limit here. The command
        # remains an unbounded logical target; any remaining limit is owned by
        # the Gazebo joint definition itself.

    def target_pointing_logical_angles(self, pointing: TargetPointing):
        """Convert physical geometric angles to the adapter's logical frame."""
        yaw_sign = self.args.yaw_sign if abs(self.args.yaw_sign) > 1.0e-6 else 1.0
        pitch_sign = (
            self.args.pitch_sign if abs(self.args.pitch_sign) > 1.0e-6 else 1.0
        )
        return (
            (pointing.yaw_deg - self.args.yaw_offset_deg) / yaw_sign,
            (pointing.pitch_deg - self.args.pitch_offset_deg) / pitch_sign,
        )

    def step(self) -> None:
        now = time.time()
        dt = max(1e-3, now - self.last_update_time)
        self.last_update_time = now
        if self.args.max_integration_dt > 0.0:
            dt = min(dt, self.args.max_integration_dt)

        latest = self.command_buffer.get()
        if self.external_detection_bridge is not None:
            self.external_detection_bridge.tick(now)
        stale = latest.msg is None or (now - latest.stamp) > self.args.command_timeout
        command_valid = (not stale) and latest.msg.valid
        # A command can be valid from gm_control's point of view while the
        # aircraft is still on the runway.  Never let that command reach the
        # native PID/rate integrator until the optional flight gate is open.
        flight_ready = self.flight_state_buffer.ready(now, self.args)
        valid = command_valid and flight_ready

        if not flight_ready:
            # Reset the adapter-side integrator as well as the native plugin.
            # This prevents a search rate accumulated before takeoff from
            # becoming a large step when the gate opens.
            self.yaw_deg = self.args.initial_yaw_deg
            self.pitch_deg = self.args.initial_pitch_deg
            self.roll_deg = self.args.initial_roll_deg
            self.yaw_rate_deg_s = 0.0
            self.pitch_rate_deg_s = 0.0
            self.roll_rate_deg_s = 0.0
        elif valid:
            self.update_angles_from_command(latest.msg, dt)
        elif self.args.invalid_action == "center":
            prev_yaw = self.yaw_deg
            prev_pitch = self.pitch_deg
            prev_roll = self.roll_deg
            self.yaw_deg = slew_towards(
                self.yaw_deg,
                0.0,
                max(0.0, self.args.max_yaw_slew_deg_s) * dt,
            )
            self.pitch_deg = slew_towards(
                self.pitch_deg,
                0.0,
                max(0.0, self.args.max_pitch_slew_deg_s) * dt,
            )
            self.roll_deg = slew_towards(
                self.roll_deg,
                0.0,
                max(0.0, self.args.max_roll_slew_deg_s) * dt,
            )
            self.yaw_rate_deg_s = (self.yaw_deg - prev_yaw) / dt
            self.pitch_rate_deg_s = (self.pitch_deg - prev_pitch) / dt
            self.roll_rate_deg_s = (self.roll_deg - prev_roll) / dt
        else:
            self.yaw_rate_deg_s = 0.0
            self.pitch_rate_deg_s = 0.0
            self.roll_rate_deg_s = 0.0

        yaw_out = self.args.yaw_sign * self.yaw_deg + self.args.yaw_offset_deg
        pitch_out = self.args.pitch_sign * self.pitch_deg + self.args.pitch_offset_deg
        roll_out = self.args.roll_sign * self.roll_deg + self.args.roll_offset_deg
        body_rate = self.body_rate_buffer.get(now, self.args.body_rate_timeout)
        los_feedforward = (
            self.target_los.get(now)
            if self.target_los is not None and self.args.target_los_feedforward
            else LosFeedforward()
        )
        target_pointing = (
            self.target_los.get_pointing(now)
            if self.target_los is not None and self.args.target_pointing_feedforward
            else TargetPointing()
        )
        pointing_yaw_correction = 0.0
        pointing_pitch_correction = 0.0
        pointing_yaw_target = 0.0
        pointing_pitch_target = 0.0
        if target_pointing.valid:
            pointing_yaw_target, pointing_pitch_target = (
                self.target_pointing_logical_angles(target_pointing)
            )
            pointing_yaw_correction = (
                self.args.target_pointing_yaw_gain
                * shortest_angle_deg(pointing_yaw_target, self.yaw_deg)
            )
            pointing_pitch_correction = (
                self.args.target_pointing_pitch_gain
                * (pointing_pitch_target - self.pitch_deg)
            )
            pointing_yaw_correction = clamp(
                pointing_yaw_correction,
                self.args.target_pointing_max_yaw_rate_deg_s,
            )
            pointing_pitch_correction = clamp(
                pointing_pitch_correction,
                self.args.target_pointing_max_pitch_rate_deg_s,
            )

        # Rate-mode tracking goes through angular_velocity fields. Sending the
        # current logical angle as a quaternion would fold yaw at +/-180
        # degrees. Angle mode is retained for smooth return-to-init commands.
        sent_rate = (float("nan"), float("nan"), float("nan"))
        sent_mode = "angle"
        try:
            if not flight_ready:
                # Explicit NEUTRAL disables the native plugin's joint PID. It
                # is deliberately unconditional here, even without
                # --neutral-when-invalid, because the flight gate itself is a
                # safety contract requested by the fixed-wing launch.
                self.native_gimbal.send_neutral()
                sent_mode = "takeoff-neutral"
                self._has_sent_valid_command = False
                self._rate_mode_anchored = False
            elif self.args.direct_rate and valid and latest.msg.mode == GimbalCommand.MODE_RATE:
                # Convert logical command rates to the same physical frame as
                # send_angles(). NativeGimbalSender applies the MAVLink/PX4
                # pitch and yaw axis signs when packing the rate fields.
                yaw_rate_out = self.args.yaw_sign * self.yaw_rate_deg_s
                pitch_rate_out = self.args.pitch_sign * self.pitch_rate_deg_s
                roll_rate_out = self.args.roll_sign * self.roll_rate_deg_s
                # When geometric pointing is valid, its body-frame angle
                # derivative already contains the aircraft attitude motion.
                # Applying the raw IMU compensation on top would count that
                # motion twice and make the camera oscillate past the target.
                if self.args.body_rate_feedforward and not target_pointing.valid:
                    # Compensate the aircraft rotation in the native gimbal
                    # frame.  Yaw is a +Z joint, and the native plugin adds
                    # vehicleYawRad in yaw-follow mode, so subtracting the
                    # vehicle yaw rate cancels that motion.  The fixed-wing
                    # pitch and roll joints use -Y/-X axes; a positive MAVLink
                    # rate therefore produces a negative joint rate on those
                    # axes.  Their compensation has the opposite algebraic
                    # sign.  Keep this mapping here (rather than changing
                    # ROS messages or the ordinary multirotor path) because
                    # it is only active when --body-rate-feedforward is set.
                    yaw_rate_out -= self.args.body_rate_yaw_gain * body_rate.yaw_deg_s
                    pitch_rate_out += self.args.body_rate_pitch_gain * body_rate.pitch_deg_s
                    roll_rate_out += self.args.body_rate_roll_gain * body_rate.roll_deg_s
                if los_feedforward.valid:
                    # The image PID corrects residual pointing error. This term
                    # supplies the inertial LOS motion caused by fixed-wing
                    # translation, which an image-only loop sees only after the
                    # target has already moved away from the centre.
                    yaw_rate_out += (
                        (-1.0 if self.args.target_los_yaw_sign < 0.0 else 1.0)
                        * self.args.target_los_yaw_gain
                        * los_feedforward.yaw_rate_deg_s
                    )
                    pitch_rate_out += (
                        (-1.0 if self.args.target_los_pitch_sign < 0.0 else 1.0)
                        * self.args.target_los_pitch_gain
                        * los_feedforward.pitch_rate_deg_s
                    )
                if target_pointing.valid:
                    # GM's image PID is a residual correction around the
                    # geometric target.  Add the absolute position error in
                    # the same logical frame so a fast fixed-wing pass can
                    # catch up before the bbox reaches the image edge.
                    yaw_rate_out += self.args.yaw_sign * pointing_yaw_correction
                    pitch_rate_out += self.args.pitch_sign * pointing_pitch_correction
                    # The derivative was computed from consecutive target
                    # angles in the body frame, so it is already in logical
                    # gimbal coordinates and does not need the IMU term above.
                    yaw_rate_out += self.args.yaw_sign * (
                        self.args.target_pointing_yaw_rate_gain
                        * target_pointing.yaw_rate_deg_s
                    )
                    pitch_rate_out += self.args.pitch_sign * (
                        self.args.target_pointing_pitch_rate_gain
                        * target_pointing.pitch_rate_deg_s
                    )
                yaw_rate_out = clamp(
                    yaw_rate_out, self.args.max_output_yaw_rate_deg_s
                )
                pitch_rate_out = clamp(
                    pitch_rate_out, self.args.max_output_pitch_rate_deg_s
                )
                roll_rate_out = clamp(
                    roll_rate_out, self.args.max_output_roll_rate_deg_s
                )
                if not self._rate_mode_anchored:
                    # Establish the native plugin's internal absolute
                    # setpoint with the same yaw-lock offset used by
                    # send_angles().  The next cycle can safely switch to
                    # NaN-quaternion rate transport.
                    self.native_gimbal.send_angles(yaw_out, pitch_out, roll_out)
                    sent_mode = "rate-anchor"
                    self._rate_mode_anchored = True
                else:
                    sent_rate = (yaw_rate_out, pitch_rate_out, roll_rate_out)
                    sent_mode = "rate"
                    self.native_gimbal.send_rates(
                        yaw_rate_out,
                        pitch_rate_out,
                        roll_rate_out,
                    )
                    # Keep the adapter-side unwrapped state synchronized with
                    # the rate actually sent to Gazebo.  The geometric/LOS/body
                    # terms are added after ``update_angles_from_command``;
                    # omitting them would make the next cycle see the old
                    # angle and apply the same correction forever.
                    yaw_sign = self.args.yaw_sign if abs(self.args.yaw_sign) > 1.0e-6 else 1.0
                    pitch_sign = self.args.pitch_sign if abs(self.args.pitch_sign) > 1.0e-6 else 1.0
                    roll_sign = self.args.roll_sign if abs(self.args.roll_sign) > 1.0e-6 else 1.0
                    self.yaw_deg += (yaw_rate_out / yaw_sign - self.yaw_rate_deg_s) * dt
                    self.pitch_deg += (pitch_rate_out / pitch_sign - self.pitch_rate_deg_s) * dt
                    self.roll_deg += (roll_rate_out / roll_sign - self.roll_rate_deg_s) * dt
                    self.yaw_rate_deg_s = yaw_rate_out / yaw_sign
                    self.pitch_rate_deg_s = pitch_rate_out / pitch_sign
                    self.roll_rate_deg_s = roll_rate_out / roll_sign
                self._has_sent_valid_command = True
            elif valid:
                self.native_gimbal.send_angles(yaw_out, pitch_out, roll_out)
                self._has_sent_valid_command = True
                self._rate_mode_anchored = False
            elif target_pointing.valid:
                # A known world target is still useful during a detector gap:
                # point back to its predicted body-frame direction instead of
                # starting an unbounded blind search.
                self.yaw_deg = slew_towards(
                    self.yaw_deg,
                    pointing_yaw_target,
                    max(0.0, self.args.max_yaw_slew_deg_s) * dt,
                )
                self.pitch_deg = slew_towards(
                    self.pitch_deg,
                    pointing_pitch_target,
                    max(0.0, self.args.max_pitch_slew_deg_s) * dt,
                )
                self.yaw_rate_deg_s = 0.0
                self.pitch_rate_deg_s = 0.0
                self.roll_rate_deg_s = 0.0
                yaw_out = self.args.yaw_sign * self.yaw_deg + self.args.yaw_offset_deg
                pitch_out = self.args.pitch_sign * self.pitch_deg + self.args.pitch_offset_deg
                roll_out = self.args.roll_sign * self.roll_deg + self.args.roll_offset_deg
                self.native_gimbal.send_angles(yaw_out, pitch_out, roll_out)
                sent_mode = "position"
                self._has_sent_valid_command = True
                self._rate_mode_anchored = False
            elif self.args.neutral_when_invalid and not self._has_sent_valid_command:
                # A zero-rate command is not neutral for the native plugin:
                # it keeps integrating its previous setpoint.  Use the
                # explicit MAVLink NEUTRAL flag until GM produces its first
                # valid command, keeping the aircraft payload torque-free.
                self.native_gimbal.send_neutral()
                sent_mode = "neutral"
                self._rate_mode_anchored = False
            elif self.args.invalid_action == "center":
                self.native_gimbal.send_angles(yaw_out, pitch_out, roll_out)
                self._rate_mode_anchored = False
            elif self.args.direct_rate:
                # An invalid command means that GM is idle/stopped, not that
                # the native Gazebo joint controller should remain in its
                # rate-integration mode.  A zero rate leaves that plugin's
                # internal accumulated setpoint active.  During fixed-wing
                # takeoff even a small body-yaw disturbance can then make the
                # lightweight gimbal chase a stale setpoint and feed its PID
                # reaction torque back into the airframe.  Re-anchor all three
                # axes with the current absolute target, matching the original
                # pre-direct-rate idle behavior.  Quaternion wrapping is safe
                # here because the plugin uses shortest-angular-distance for
                # an angle target; continuous rate mode remains in use while
                # an actual MODE_RATE tracking/search command is valid.
                self.native_gimbal.send_angles(yaw_out, pitch_out, roll_out)
                self._rate_mode_anchored = False
            else:
                # Preserve the legacy angle-based behavior for Typhoon and
                # other finite-yaw gimbals using this adapter.
                self.native_gimbal.send_angles(yaw_out, pitch_out, roll_out)
                self._rate_mode_anchored = False
        except (OSError, AttributeError, TypeError, ValueError) as exc:
            if now - self.last_log_time > self.args.log_interval:
                rospy.logwarn("native gimbal MAVLink send error: %s", exc)
                self.last_log_time = now

        if now - self.last_log_time > self.args.log_interval:
            rospy.loginfo(
                "Gimbal adapter: valid=%s stale=%s cmd_frame=(%.2f, %.2f, %.2f) "
                "gazebo=(%.2f, %.2f, %.2f) deg "
                "los_ff=(valid=%s yaw=%.2f pitch=%.2f range=%.1fm) "
                "pointing=(valid=%s yaw=%.2f pitch=%.2f rate=(%.2f,%.2f) range=%.1fm) "
                "transport=%s sent_rate=(%.2f, %.2f, %.2f)deg/s "
                "flight_ready=%s armed=%s altitude=%.2fm",
                valid,
                stale,
                self.yaw_deg,
                self.pitch_deg,
                self.roll_deg,
                yaw_out,
                pitch_out,
                roll_out,
                los_feedforward.valid,
                los_feedforward.yaw_rate_deg_s,
                los_feedforward.pitch_rate_deg_s,
                los_feedforward.distance_m,
                target_pointing.valid,
                target_pointing.yaw_deg,
                target_pointing.pitch_deg,
                target_pointing.yaw_rate_deg_s,
                target_pointing.pitch_rate_deg_s,
                target_pointing.distance_m,
                sent_mode,
                sent_rate[0],
                sent_rate[1],
                sent_rate[2],
                flight_ready,
                self.flight_state_buffer.summary().armed,
                self.flight_state_buffer.summary().altitude_m,
            )
            self.last_log_time = now

        self.publish_state(valid=valid)

    def publish_state(self, valid: bool) -> None:
        msg = GimbalState()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.args.frame_id
        msg.valid = valid
        msg.yaw_deg = self.yaw_deg
        msg.pitch_deg = self.pitch_deg
        msg.roll_deg = self.roll_deg
        msg.yaw_rate_deg_s = self.yaw_rate_deg_s
        msg.pitch_rate_deg_s = self.pitch_rate_deg_s
        msg.roll_rate_deg_s = self.roll_rate_deg_s
        self.state_pub.publish(msg)

    def spin(self) -> None:
        rate = rospy.Rate(self.args.rate)
        while not rospy.is_shutdown():
            self.step()
            rate.sleep()


def main() -> None:
    args = parse_args()
    rospy.init_node("gm_control_to_gazebo_typhoon_gimbal", anonymous=True)
    adapter = TyphoonGimbalAdapter(args)
    adapter.spin()


if __name__ == "__main__":
    main()
