#!/usr/bin/env python3
"""Publish a deterministic metric detection for the real PX4/Gazebo SITL run.

This is deliberately a test input, not a detector replacement: PX4, Gazebo,
MAVROS, state estimation, the fixed-wing controller, and tracker remain real.
The generated target is a constant-radius body-frame orbit with valid timing,
source metadata, range, and covariance so the metric guidance path can be
checked without requiring a camera sensor in the stock PX4 plane model.
"""

import math

import rospy
from gazebo_msgs.msg import ModelStates
from xd_uav_track.msg import DetectionArray, DetectionCandidate


class TruthPose:
    def __init__(self):
        self.vehicle = None
        self.target = None

    def callback(self, message):
        try:
            vehicle_name = rospy.get_param("~vehicle_model", "plane_catapult")
            target_name = rospy.get_param("~target_model", "visual_target")
            vehicle_index = message.name.index(vehicle_name)
            target_index = message.name.index(target_name)
        except (ValueError, KeyError):
            return
        vehicle_pose = message.pose[vehicle_index]
        target_pose = message.pose[target_index]
        self.vehicle = vehicle_pose
        self.target = target_pose

    def relative_body_frd(self):
        if self.vehicle is None or self.target is None:
            return None
        vp = self.vehicle.position
        tp = self.target.position
        dx, dy, dz = (float(tp.x - vp.x), float(tp.y - vp.y),
                      float(tp.z - vp.z))
        q = self.vehicle.orientation
        # Rotation matrix body/FLU -> world/ENU.  Gazebo plane uses x-forward,
        # y-left, z-up; the tracker contract is body FRD, hence the two sign
        # changes after applying the inverse rotation.
        x, y, z, w = float(q.x), float(q.y), float(q.z), float(q.w)
        norm = math.sqrt(x * x + y * y + z * z + w * w)
        if norm < 1e-6:
            return None
        x, y, z, w = x / norm, y / norm, z / norm, w / norm
        r00 = 1.0 - 2.0 * (y * y + z * z)
        r01 = 2.0 * (x * y - z * w)
        r02 = 2.0 * (x * z + y * w)
        r10 = 2.0 * (x * y + z * w)
        r11 = 1.0 - 2.0 * (x * x + z * z)
        r12 = 2.0 * (y * z - x * w)
        r20 = 2.0 * (x * z - y * w)
        r21 = 2.0 * (y * z + x * w)
        r22 = 1.0 - 2.0 * (x * x + y * y)
        body_flu = (r00 * dx + r10 * dy + r20 * dz,
                    r01 * dx + r11 * dy + r21 * dz,
                    r02 * dx + r12 * dy + r22 * dz)
        return [body_flu[0], -body_flu[1], -body_flu[2]]


def main():
    rospy.init_node("sitl_metric_input")
    namespace = rospy.get_namespace().strip("/")
    uav_name = rospy.get_param("~uav_name", namespace or "uav1")
    topic = rospy.get_param("~topic", "/%s/detect/detections" % uav_name)
    rate_hz = float(rospy.get_param("~rate_hz", 20.0))
    radius_m = float(rospy.get_param("~radius_m", 80.0))
    down_m = float(rospy.get_param("~down_m", 40.0))
    truth_mode = bool(rospy.get_param("~truth_follow", False))
    truth_pose = TruthPose()
    if truth_mode:
        rospy.Subscriber("/gazebo/model_states", ModelStates,
                         truth_pose.callback, queue_size=1)
    publisher = rospy.Publisher(topic, DetectionArray, queue_size=10)
    rate = rospy.Rate(rate_hz)
    phase = 0.0
    sent = 0

    while not rospy.is_shutdown():
        now = rospy.Time.now()
        if now.is_zero():
            rate.sleep()
            continue

        message = DetectionArray()
        message.header.stamp = now
        message.header.frame_id = "%s/down_camera_optical_frame" % uav_name
        message.image_width = 640
        message.image_height = 480
        message.image_source = "fixed_down"
        message.sensor_id = "sitl_metric_source"
        message.detector_name = "sitl_metric_input"
        message.model_version = "synthetic-1"
        message.command = "start_track" if sent == 0 else ""

        candidate = DetectionCandidate()
        candidate.track_id = 1
        candidate.class_id = 0
        candidate.track_id_is_stable = True
        candidate.has_bbox = True
        candidate.bbox = [282, 202, 76, 76]
        candidate.has_normalized_bbox = True
        candidate.normalized_bbox = [0.5, 0.5, 0.12, 0.16]
        candidate.confidence = 0.95
        candidate.has_relative_position_body = True
        relative = truth_pose.relative_body_frd() if truth_mode else [
            radius_m * math.cos(phase),
            radius_m * math.sin(phase),
            down_m,
        ]
        if relative is None:
            # Do not emit a fabricated metric point before Gazebo has supplied
            # both poses; this keeps the strict freshness gate meaningful.
            rate.sleep()
            continue
        candidate.relative_position_body = relative
        candidate.has_relative_velocity_body = True
        candidate.relative_velocity_body = [0.0, 0.0, 0.0]
        candidate.position_covariance = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0,
        ]
        candidate.velocity_covariance = [
            0.25, 0.0, 0.0,
            0.0, 0.25, 0.0,
            0.0, 0.0, 0.25,
        ]
        candidate.range_valid = True
        message.candidates = [candidate]
        publisher.publish(message)
        sent += 1
        phase = (phase + 0.01) % (2.0 * math.pi)
        try:
            rate.sleep()
        except rospy.exceptions.ROSTimeMovedBackwardsException:
            # Gazebo resets /clock when a fresh PX4 instance is launched.
            # Recreate the Rate object so the deterministic test source can
            # continue without turning a simulator restart into a stale
            # observation failure.
            rate = rospy.Rate(rate_hz)


if __name__ == "__main__":
    main()
