#!/usr/bin/env python3
"""
Bridge namespaced gm_control messages to the Gazebo typhoon_h480 CGO3 gimbal.

This script is intentionally kept outside gm_control. It is only one backend
adapter for the current PX4/Gazebo test vehicle. Other gimbals can use different
adapters while keeping the gm_control image controller unchanged.
"""

import argparse
import math
import threading
import time
from dataclasses import dataclass

import rospy
from gazebo_msgs.srv import SetModelConfiguration, SetModelConfigurationRequest
from gm_control.msg import GimbalCommand, GimbalState


@dataclass
class LatestCommand:
    msg: GimbalCommand = None
    stamp: float = 0.0


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def deg_to_rad(value: float) -> float:
    return math.radians(value)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Apply gm_control/GimbalCommand to the Typhoon H480 Gazebo gimbal."
    )
    parser.add_argument("--command-topic", default="/uav1/gm_control/gimbal_cmd")
    parser.add_argument("--state-topic", default="/uav1/gm_control/gimbal_state")
    # single_vehicle_spawn.launch appends the PX4 instance ID to the vehicle
    # name. The single namespaced launcher uses instance zero.
    parser.add_argument("--model", default="typhoon_h4800")
    parser.add_argument("--frame-id", default="uav1/cgo3_gimbal")
    parser.add_argument("--rate", type=float, default=50.0)
    parser.add_argument("--command-timeout", type=float, default=0.5)
    parser.add_argument("--invalid-action", choices=["hold", "center"], default="hold")

    parser.add_argument("--initial-yaw-deg", type=float, default=0.0)
    parser.add_argument("--initial-pitch-deg", type=float, default=0.0)
    parser.add_argument("--initial-roll-deg", type=float, default=0.0)

    parser.add_argument("--yaw-sign", type=float, default=1.0)
    parser.add_argument("--pitch-sign", type=float, default=1.0)
    parser.add_argument("--roll-sign", type=float, default=1.0)

    parser.add_argument("--yaw-offset-deg", type=float, default=0.0)
    parser.add_argument("--pitch-offset-deg", type=float, default=0.0)
    parser.add_argument("--roll-offset-deg", type=float, default=0.0)

    parser.add_argument("--max-yaw-deg", type=float, default=240.0)
    parser.add_argument("--max-pitch-deg", type=float, default=90.0)
    parser.add_argument("--max-roll-deg", type=float, default=45.0)

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

        rospy.loginfo("Waiting for /gazebo/set_model_configuration ...")
        rospy.wait_for_service("/gazebo/set_model_configuration")
        self.set_config = rospy.ServiceProxy("/gazebo/set_model_configuration", SetModelConfiguration)
        rospy.loginfo(
            "gm_control Gazebo gimbal adapter started: command_topic=%s state_topic=%s model=%s",
            args.command_topic,
            args.state_topic,
            args.model,
        )

    def make_request(self, yaw_deg: float, pitch_deg: float, roll_deg: float):
        req = SetModelConfigurationRequest()
        req.model_name = self.args.model
        req.urdf_param_name = ""
        req.joint_names = [
            "cgo3_vertical_arm_joint",
            "cgo3_horizontal_arm_joint",
            "cgo3_camera_joint",
        ]
        req.joint_positions = [
            deg_to_rad(yaw_deg),
            deg_to_rad(roll_deg),
            deg_to_rad(pitch_deg),
        ]
        return req

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

        self.yaw_deg = clamp(self.yaw_deg, -self.args.max_yaw_deg, self.args.max_yaw_deg)
        self.pitch_deg = clamp(self.pitch_deg, -self.args.max_pitch_deg, self.args.max_pitch_deg)
        self.roll_deg = clamp(self.roll_deg, -self.args.max_roll_deg, self.args.max_roll_deg)

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

        try:
            resp = self.set_config(self.make_request(yaw_out, pitch_out, roll_out))
            if not resp.success and now - self.last_log_time > self.args.log_interval:
                rospy.logwarn("set_model_configuration failed: %s", resp.status_message)
                self.last_log_time = now
        except rospy.ServiceException as exc:
            if now - self.last_log_time > self.args.log_interval:
                rospy.logwarn("set_model_configuration service error: %s", exc)
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
