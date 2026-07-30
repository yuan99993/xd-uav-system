"""
PX4Interface — PX4-Autopilot / MAVROS 接口适配器。

Provides a unified interface for:
- Reading PX4 telemetry (attitude, altitude, velocity, flight mode)
  via MAVROS topics
- Publishing Offboard control commands via MAVROS setpoints
- Gazebo SITL compatibility

Supports both MAVROS (ROS1 topics) and can be extended for MAVSDK.
"""

from dataclasses import dataclass, field
from typing import Optional, Tuple
import logging

logger = logging.getLogger(__name__)


@dataclass
class PX4Telemetry:
    """
    Unified PX4 telemetry snapshot.

    Populated from MAVROS topics or MAVSDK callbacks.
    All angles in degrees, positions in meters, velocities in m/s.
    """
    timestamp: float = 0.0

    # Attitude
    roll_deg: float = 0.0
    pitch_deg: float = 0.0
    yaw_deg: float = 0.0

    # Position (NED)
    pos_x: float = 0.0    # North
    pos_y: float = 0.0    # East
    pos_z: float = 0.0    # Down (negative altitude)

    # Altitude
    altitude_msl: float = 0.0    # MSL altitude (m)
    altitude_rel: float = 0.0    # Relative altitude (m)

    # Velocity (NED)
    vel_n: float = 0.0    # North velocity
    vel_e: float = 0.0    # East velocity
    vel_d: float = 0.0    # Down velocity

    # Body velocity
    vel_body_x: float = 0.0   # Forward
    vel_body_y: float = 0.0   # Right
    vel_body_z: float = 0.0   # Down

    # Flight mode
    flight_mode: str = ""
    armed: bool = False
    offboard_active: bool = False

    # Validity flags
    attitude_valid: bool = False
    position_valid: bool = False
    velocity_valid: bool = False
    altitude_valid: bool = False


@dataclass
class PX4OffboardCommand:
    """
    Offboard control command for PX4.

    Corresponds to MAVSDK offboard.set_velocity_body() or
    MAVROS mavros/setpoint_raw/attitude.
    """
    timestamp: float = 0.0

    # Body velocity (m/s, body frame)
    velocity_forward: float = 0.0
    velocity_right: float = 0.0
    velocity_down: float = 0.0

    # Yaw rate (deg/s)
    yaw_rate_deg_s: float = 0.0

    # Yaw setpoint (degrees, 0=North)
    yaw_setpoint_deg: float = 0.0

    # Control mode
    control_mode: str = "velocity_body"  # velocity_body | attitude_rate

    # Validity
    command_valid: bool = False
    offboard_mode_requested: bool = False


class PX4Interface:
    """
    PX4-Autopilot interface adapter for ROS1/MAVROS.

    Manages:
    - Telemetry subscriptions (MAVROS topics)
    - Offboard command publication (MAVROS setpoints)
    - Mode switching (Offboard enable/disable)
    - Gazebo SITL compatibility

    ROS1 Topic Mapping:
    - Sub: /mavros/local_position/pose (geometry_msgs/PoseStamped)
    - Sub: /mavros/local_position/velocity_local (geometry_msgs/TwistStamped)
    - Sub: /mavros/imu/data (sensor_msgs/Imu)
    - Sub: /mavros/state (mavros_msgs/State)
    - Sub: /mavros/altitude (mavros_msgs/Altitude)
    - Pub: /mavros/setpoint_raw/local (mavros_msgs/PositionTarget)
    - Pub: /mavros/setpoint_velocity/cmd_vel (geometry_msgs/TwistStamped)
    """

    def __init__(self):
        """Initialize PX4 interface."""
        self._telemetry = PX4Telemetry()
        self._last_command = PX4OffboardCommand()

        logger.info("[PX4Interface] Initialized (MAVROS adapter)")

    # ── Telemetry ────────────────────────────────────────────────────────

    def update_attitude(self, roll_deg: float, pitch_deg: float, yaw_deg: float,
                        timestamp: float = 0.0):
        """Update attitude from IMU/attitude data."""
        self._telemetry.roll_deg = roll_deg
        self._telemetry.pitch_deg = pitch_deg
        self._telemetry.yaw_deg = yaw_deg
        self._telemetry.attitude_valid = True
        self._telemetry.timestamp = timestamp

    def update_position(self, x: float, y: float, z: float):
        """Update NED position."""
        self._telemetry.pos_x = x
        self._telemetry.pos_y = y
        self._telemetry.pos_z = z
        self._telemetry.position_valid = True

    def update_altitude(self, alt_msl: float, alt_rel: float):
        """Update altitude."""
        self._telemetry.altitude_msl = alt_msl
        self._telemetry.altitude_rel = alt_rel
        self._telemetry.altitude_valid = True

    def update_velocity_ned(self, vn: float, ve: float, vd: float):
        """Update NED velocity."""
        self._telemetry.vel_n = vn
        self._telemetry.vel_e = ve
        self._telemetry.vel_d = vd
        self._telemetry.velocity_valid = True

    def update_flight_mode(self, mode: str, armed: bool, offboard: bool):
        """Update flight mode."""
        self._telemetry.flight_mode = mode
        self._telemetry.armed = armed
        self._telemetry.offboard_active = offboard

    @property
    def telemetry(self) -> PX4Telemetry:
        """Get current telemetry snapshot."""
        return self._telemetry

    # ── Command Construction ─────────────────────────────────────────────

    def build_velocity_body_command(
        self,
        v_forward: float,
        v_right: float,
        v_down: float,
        yaw_rate_deg_s: float,
        yaw_setpoint_deg: float = 0.0,
    ) -> PX4OffboardCommand:
        """
        Build a body-velocity offboard command.

        Args:
            v_forward: Forward velocity (m/s, body-x)
            v_right: Right velocity (m/s, body-y)
            v_down: Down velocity (m/s, body-z, positive=down)
            yaw_rate_deg_s: Yaw angular rate (deg/s)
            yaw_setpoint_deg: Yaw setpoint (degrees)

        Returns:
            PX4OffboardCommand
        """
        cmd = PX4OffboardCommand(
            timestamp=self._telemetry.timestamp,
            velocity_forward=v_forward,
            velocity_right=v_right,
            velocity_down=v_down,
            yaw_rate_deg_s=yaw_rate_deg_s,
            yaw_setpoint_deg=yaw_setpoint_deg,
            control_mode="velocity_body",
            command_valid=True,
            offboard_mode_requested=True,
        )
        self._last_command = cmd
        return cmd

    def build_attitude_rate_command(
        self,
        roll_rate: float,
        pitch_rate: float,
        yaw_rate_deg_s: float,
        thrust: float,
    ) -> PX4OffboardCommand:
        """
        Build an attitude-rate offboard command.

        Args:
            roll_rate: Roll rate (rad/s)
            pitch_rate: Pitch rate (rad/s)
            yaw_rate_deg_s: Yaw rate (deg/s)
            thrust: Normalized thrust [0, 1]

        Returns:
            PX4OffboardCommand
        """
        cmd = PX4OffboardCommand(
            timestamp=self._telemetry.timestamp,
            yaw_rate_deg_s=yaw_rate_deg_s,
            velocity_down=thrust,  # Repurposed for thrust
            control_mode="attitude_rate",
            command_valid=True,
            offboard_mode_requested=True,
        )
        self._last_command = cmd
        return cmd

    def build_stop_command(self) -> PX4OffboardCommand:
        """Build a zero-velocity stop command."""
        cmd = PX4OffboardCommand(
            timestamp=self._telemetry.timestamp,
            control_mode="velocity_body",
            command_valid=True,
            offboard_mode_requested=True,
        )
        self._last_command = cmd
        return cmd

    @property
    def last_command(self) -> PX4OffboardCommand:
        """Get last built command."""
        return self._last_command
