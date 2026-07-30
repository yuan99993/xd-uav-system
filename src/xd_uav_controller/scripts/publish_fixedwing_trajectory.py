#!/usr/bin/env python3

"""Publish a flyable circular MultiDOF trajectory for fixed-wing testing."""

import math

import rospy
import tf2_geometry_msgs
import tf2_ros
from geometry_msgs.msg import (
    PoseStamped,
    Quaternion,
    Transform,
    Twist,
)
from trajectory_msgs.msg import (
    MultiDOFJointTrajectory,
    MultiDOFJointTrajectoryPoint,
)

from xd_uav_controller.msg import ControlState


def wrap_angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def quaternion_from_yaw(yaw):
    quaternion = Quaternion()
    quaternion.z = math.sin(0.5 * yaw)
    quaternion.w = math.cos(0.5 * yaw)
    return quaternion


def yaw_from_quaternion(quaternion):
    sin_yaw = 2.0 * (
        quaternion.w * quaternion.z
        + quaternion.x * quaternion.y
    )
    cos_yaw = 1.0 - 2.0 * (
        quaternion.y * quaternion.y
        + quaternion.z * quaternion.z
    )
    return math.atan2(sin_yaw, cos_yaw)


def parse_direction(value):
    text = str(value).strip().lower()
    if text in ("ccw", "left", "+1", "1"):
        return 1
    if text in ("cw", "right", "-1"):
        return -1
    raise ValueError(
        "~direction必须是ccw/left/+1或cw/right/-1"
    )


class FixedwingCircleTrajectoryPublisher:

    def __init__(self):
        self._uav_name = rospy.get_param(
            "~uav_name", "uav1"
        ).strip("/")
        self._state_topic = rospy.get_param(
            "~state_topic",
            "/{}/control_manager/state".format(
                self._uav_name
            ),
        )
        self._trajectory_topic = rospy.get_param(
            "~trajectory_topic",
            "/{}/control/reference/trajectory".format(
                self._uav_name
            ),
        )
        self._requested_frame = rospy.get_param(
            "~frame_id", ""
        ).strip("/")
        self._radius = float(
            rospy.get_param("~radius", 80.0)
        )
        self._airspeed = float(
            rospy.get_param("~airspeed", 15.0)
        )
        self._direction = parse_direction(
            rospy.get_param("~direction", "ccw")
        )
        self._point_dt = float(
            rospy.get_param("~point_dt", 0.5)
        )
        self._horizon_laps = float(
            rospy.get_param("~horizon_laps", 2.0)
        )
        self._relative_altitude = float(
            rospy.get_param("~relative_altitude", 0.0)
        )
        self._repeat = bool(
            rospy.get_param("~repeat", True)
        )
        self._start_delay = float(
            rospy.get_param("~start_delay", 0.5)
        )

        if self._radius < 5.0:
            raise ValueError("~radius必须不小于5米")
        if self._airspeed <= 0.0:
            raise ValueError("~airspeed必须大于0")
        if self._point_dt <= 0.05:
            raise ValueError("~point_dt必须大于0.05秒")
        if self._horizon_laps < 1.0:
            raise ValueError("~horizon_laps必须不小于1")

        self._tf_buffer = tf2_ros.Buffer(
            cache_time=rospy.Duration(10.0)
        )
        self._tf_listener = tf2_ros.TransformListener(
            self._tf_buffer
        )
        self._publisher = rospy.Publisher(
            self._trajectory_topic,
            MultiDOFJointTrajectory,
            queue_size=1,
            latch=True,
        )

    def wait_for_state(self):
        rospy.loginfo(
            "等待有效固定翼控制状态: %s",
            self._state_topic,
        )
        while not rospy.is_shutdown():
            try:
                state = rospy.wait_for_message(
                    self._state_topic,
                    ControlState,
                    timeout=2.0,
                )
            except rospy.ROSException:
                rospy.logwarn_throttle(
                    2.0, "尚未收到固定翼控制状态"
                )
                continue
            if (
                state.vehicle_type
                != ControlState.VEHICLE_FIXEDWING
            ):
                rospy.logwarn_throttle(
                    2.0, "当前ControlState不是固定翼"
                )
                continue
            if not state.state_valid or not state.stable:
                rospy.logwarn_throttle(
                    2.0, "固定翼控制状态尚未稳定"
                )
                continue
            if not state.airspeed_valid:
                rospy.logwarn_throttle(
                    2.0, "固定翼空速尚未有效"
                )
                continue
            return state
        raise rospy.ROSInterruptException()

    def initial_pose_in_trajectory_frame(self, state):
        source_pose = PoseStamped()
        source_pose.header = state.header
        source_pose.pose.position = state.position_odom
        source_pose.pose.orientation = (
            state.orientation_odom_body
        )
        frame_id = (
            self._requested_frame
            if self._requested_frame
            else state.header.frame_id.strip("/")
        )
        if frame_id == state.header.frame_id.strip("/"):
            source_pose.header.frame_id = frame_id
            return source_pose

        transform = self._tf_buffer.lookup_transform(
            frame_id,
            state.header.frame_id,
            rospy.Time(0),
            rospy.Duration(1.0),
        )
        return tf2_geometry_msgs.do_transform_pose(
            source_pose, transform
        )

    def build_trajectory(
        self,
        start_time,
        frame_id,
        body_frame_id,
        center_x,
        center_y,
        target_z,
        initial_angle,
    ):
        angular_speed = self._airspeed / self._radius
        lap_duration = 2.0 * math.pi / angular_speed
        duration = self._horizon_laps * lap_duration
        point_count = int(
            math.ceil(duration / self._point_dt)
        )

        trajectory = MultiDOFJointTrajectory()
        trajectory.header.stamp = start_time
        trajectory.header.frame_id = frame_id
        trajectory.joint_names = [body_frame_id]

        for index in range(point_count + 1):
            time_from_start = min(
                index * self._point_dt, duration
            )
            angle = (
                initial_angle
                + self._direction
                * angular_speed
                * time_from_start
            )
            course = wrap_angle(
                angle
                + self._direction * 0.5 * math.pi
            )

            transform = Transform()
            transform.translation.x = (
                center_x + self._radius * math.cos(angle)
            )
            transform.translation.y = (
                center_y + self._radius * math.sin(angle)
            )
            transform.translation.z = target_z
            transform.rotation = quaternion_from_yaw(course)

            velocity = Twist()
            velocity.linear.x = (
                -self._direction
                * self._airspeed
                * math.sin(angle)
            )
            velocity.linear.y = (
                self._direction
                * self._airspeed
                * math.cos(angle)
            )
            velocity.angular.z = (
                self._direction * angular_speed
            )

            acceleration = Twist()
            centripetal = (
                self._airspeed * self._airspeed / self._radius
            )
            acceleration.linear.x = (
                -centripetal * math.cos(angle)
            )
            acceleration.linear.y = (
                -centripetal * math.sin(angle)
            )

            point = MultiDOFJointTrajectoryPoint()
            point.transforms = [transform]
            point.velocities = [velocity]
            point.accelerations = [acceleration]
            point.time_from_start = rospy.Duration(
                time_from_start
            )
            trajectory.points.append(point)

        return trajectory, lap_duration

    def run(self):
        state = self.wait_for_state()
        initial_pose = self.initial_pose_in_trajectory_frame(
            state
        )
        frame_id = initial_pose.header.frame_id.strip("/")
        body_frame_id = state.body_frame_id.strip("/")
        course = (
            state.course
            if math.isfinite(state.course)
            and frame_id
            == state.header.frame_id.strip("/")
            else yaw_from_quaternion(
                initial_pose.pose.orientation
            )
        )
        initial_angle = wrap_angle(
            course - self._direction * 0.5 * math.pi
        )
        center_x = (
            initial_pose.pose.position.x
            - self._direction
            * self._radius
            * math.sin(course)
        )
        center_y = (
            initial_pose.pose.position.y
            + self._direction
            * self._radius
            * math.cos(course)
        )
        target_z = (
            initial_pose.pose.position.z
            + self._relative_altitude
        )

        start_time = (
            rospy.Time.now()
            + rospy.Duration(max(0.0, self._start_delay))
        )
        trajectory, lap_duration = self.build_trajectory(
            start_time,
            frame_id,
            body_frame_id,
            center_x,
            center_y,
            target_z,
            initial_angle,
        )
        self._publisher.publish(trajectory)
        rospy.loginfo(
            "已发布固定翼圆轨迹: topic=%s frame=%s "
            "center=(%.1f, %.1f) z=%.1f radius=%.1f "
            "airspeed=%.1f direction=%s points=%d",
            self._trajectory_topic,
            frame_id,
            center_x,
            center_y,
            target_z,
            self._radius,
            self._airspeed,
            "CCW" if self._direction > 0 else "CW",
            len(trajectory.points),
        )

        if not self._repeat:
            rospy.spin()
            return

        next_start = (
            start_time + rospy.Duration(lap_duration)
        )
        while not rospy.is_shutdown():
            remaining = (next_start - rospy.Time.now()).to_sec()
            if remaining > 0.0:
                rospy.sleep(min(remaining, 0.2))
                continue
            trajectory, _ = self.build_trajectory(
                next_start,
                frame_id,
                body_frame_id,
                center_x,
                center_y,
                target_z,
                initial_angle,
            )
            self._publisher.publish(trajectory)
            rospy.loginfo(
                "圆轨迹已连续续期，下一圈仍保持同一圆心"
            )
            next_start += rospy.Duration(lap_duration)


def main():
    rospy.init_node("publish_fixedwing_trajectory")
    try:
        FixedwingCircleTrajectoryPublisher().run()
    except (
        ValueError,
        rospy.ROSException,
        tf2_ros.TransformException,
    ) as error:
        rospy.logfatal("固定翼轨迹发布失败: %s", error)
        raise SystemExit(1)


if __name__ == "__main__":
    main()
