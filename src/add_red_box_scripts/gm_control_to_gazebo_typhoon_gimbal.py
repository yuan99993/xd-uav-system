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
from gm_control.msg import GimbalCommand, GimbalState
from pymavlink import mavutil

mavutil.set_dialect("common")


@dataclass
class LatestCommand:
    msg: GimbalCommand = None
    stamp: float = 0.0


def deg_to_rad(value: float) -> float:
    return math.radians(value)


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


class NativeGimbalSender:
    """Send angle setpoints to the native Gazebo gimbal PID controller."""

    def __init__(self, args: argparse.Namespace) -> None:
        self.link = mavutil.mavlink_connection(
            "udpout:{0}:{1}".format(args.gimbal_host, args.gimbal_port),
            source_system=255,
            source_component=mavutil.mavlink.MAV_COMP_ID_ONBOARD_COMPUTER,
        )
        # Roll and pitch are treated as lock-frame angles by the native plugin.
        # Yaw is left in follow/body frame, matching the old ROS adapter's
        # relative-to-vehicle command convention.
        self.flags = (
            mavutil.mavlink.GIMBAL_DEVICE_FLAGS_ROLL_LOCK
            | mavutil.mavlink.GIMBAL_DEVICE_FLAGS_PITCH_LOCK
        )
        rospy.loginfo(
            "Native Gazebo gimbal MAVLink endpoint: udp://%s:%d",
            args.gimbal_host,
            args.gimbal_port,
        )

    def send_angles(self, yaw_deg: float, pitch_deg: float, roll_deg: float) -> None:
        # The native plugin internally applies the Typhoon joint-axis signs:
        # physical roll = q.roll, physical pitch = -q.pitch, physical yaw = -q.yaw
        # (for yaw-follow mode). Convert the old adapter's physical target into
        # the quaternion expected by GIMBAL_DEVICE_SET_ATTITUDE.
        q = quaternion_from_euler(
            deg_to_rad(roll_deg),
            -deg_to_rad(pitch_deg),
            -deg_to_rad(yaw_deg),
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


class TyphoonGimbalAdapter:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.command_buffer = CommandBuffer()
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
        self.last_update_time = time.time()
        self.last_log_time = 0.0

        self.state_pub = rospy.Publisher(args.state_topic, GimbalState, queue_size=10)
        rospy.Subscriber(args.command_topic, GimbalCommand, self.command_buffer.callback, queue_size=1)

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
            self.yaw_deg = cmd.yaw_deg
            self.pitch_deg = cmd.pitch_deg
            self.roll_deg = cmd.roll_deg
            self.yaw_rate_deg_s = (self.yaw_deg - prev_yaw) / dt
            self.pitch_rate_deg_s = (self.pitch_deg - prev_pitch) / dt
            self.roll_rate_deg_s = (self.roll_deg - prev_roll) / dt
        else:
            self.yaw_rate_deg_s = cmd.yaw_rate_deg_s
            self.pitch_rate_deg_s = cmd.pitch_rate_deg_s
            self.roll_rate_deg_s = cmd.roll_rate_deg_s
            self.yaw_deg += self.yaw_rate_deg_s * dt
            self.pitch_deg += self.pitch_rate_deg_s * dt
            self.roll_deg += self.roll_rate_deg_s * dt

        # Do not impose an application-level angle limit here. The command
        # remains an unbounded logical target; any remaining limit is owned by
        # the Gazebo joint definition itself.

    def step(self) -> None:
        now = time.time()
        dt = max(1e-3, now - self.last_update_time)
        self.last_update_time = now

        latest = self.command_buffer.get()
        stale = latest.msg is None or (now - latest.stamp) > self.args.command_timeout
        valid = (not stale) and latest.msg.valid

        if valid:
            self.update_angles_from_command(latest.msg, dt)
        elif self.args.invalid_action == "center":
            self.yaw_deg = 0.0
            self.pitch_deg = 0.0
            self.roll_deg = 0.0
            self.yaw_rate_deg_s = 0.0
            self.pitch_rate_deg_s = 0.0
            self.roll_rate_deg_s = 0.0
        else:
            self.yaw_rate_deg_s = 0.0
            self.pitch_rate_deg_s = 0.0
            self.roll_rate_deg_s = 0.0

        yaw_out = self.args.yaw_sign * self.yaw_deg + self.args.yaw_offset_deg
        pitch_out = self.args.pitch_sign * self.pitch_deg + self.args.pitch_offset_deg
        roll_out = self.args.roll_sign * self.roll_deg + self.args.roll_offset_deg

        # Always send the current target, including when the target is invalid.
        # This preserves hold/center semantics while the native SDF PID applies
        # the force needed to counter gravity.
        try:
            self.native_gimbal.send_angles(yaw_out, pitch_out, roll_out)
        except (OSError, AttributeError, TypeError, ValueError) as exc:
            if now - self.last_log_time > self.args.log_interval:
                rospy.logwarn("native gimbal MAVLink send error: %s", exc)
                self.last_log_time = now

        if now - self.last_log_time > self.args.log_interval:
            rospy.loginfo(
                "Gimbal adapter: valid=%s stale=%s cmd_frame=(%.2f, %.2f, %.2f) gazebo=(%.2f, %.2f, %.2f) deg",
                valid,
                stale,
                self.yaw_deg,
                self.pitch_deg,
                self.roll_deg,
                yaw_out,
                pitch_out,
                roll_out,
            )
            self.last_log_time = now

        self.publish_state(valid=not stale)

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
