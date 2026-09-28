#!/usr/bin/env python3
import math
import threading
import unittest

import rospy
import rostest
from geographic_msgs.msg import GeoPointStamped
from xd_uav_detect.msg import (GeodeticDetectionArray, WorldDetection,
                               WorldDetectionArray)


class GeoreferencerTest(unittest.TestCase):
    def setUp(self):
        self._condition = threading.Condition()
        self._outputs = {}
        self._publishers = {}
        for name in ("same", "rotated", "missing_tf", "topic_origin"):
            self._publishers[name] = rospy.Publisher(
                "/geodesy/{}/input".format(name), WorldDetectionArray,
                queue_size=2)
            rospy.Subscriber(
                "/geodesy/{}/output".format(name), GeodeticDetectionArray,
                self._callback, callback_args=name, queue_size=10)
        self._origin_publisher = rospy.Publisher(
            "/geodesy/origin", GeoPointStamped, queue_size=1, latch=True)
        deadline = rospy.Time.now() + rospy.Duration(5.0)
        while (not rospy.is_shutdown() and rospy.Time.now() < deadline and
               any(publisher.get_num_connections() == 0
                   for publisher in self._publishers.values())):
            rospy.sleep(0.05)

    def _callback(self, message, name):
        with self._condition:
            self._outputs[name] = message
            self._condition.notify_all()

    @staticmethod
    def _candidate(x=10.0, y=0.0, z=0.0, covariance=None,
                   sensor_id="lidar_camera", valid=True):
        candidate = WorldDetection()
        candidate.source_candidate_index = 4
        candidate.track_id = 9
        candidate.class_id = 2
        candidate.confidence = 0.75
        candidate.image_source = "test_camera"
        candidate.sensor_id = sensor_id
        candidate.detector_name = "fixture"
        candidate.model_version = "v1"
        candidate.position_valid = valid
        candidate.position_world.x = x
        candidate.position_world.y = y
        candidate.position_world.z = z
        candidate.position_covariance_world = covariance or [
            1.0, 0.0, 0.0,
            0.0, 2.0, 0.0,
            0.0, 0.0, 3.0]
        return candidate

    def _message(self, frame="map", stamp=None, candidates=None):
        message = WorldDetectionArray()
        message.header.stamp = stamp if stamp is not None else rospy.Time.now()
        message.header.frame_id = frame
        message.detections = candidates or [self._candidate()]
        return message

    def _publish_and_wait(self, name, message, timeout=3.0):
        with self._condition:
            self._outputs.pop(name, None)
        self._publishers[name].publish(message)
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        with self._condition:
            while name not in self._outputs and not rospy.is_shutdown():
                remaining = (deadline - rospy.Time.now()).to_sec()
                if remaining <= 0.0:
                    break
                self._condition.wait(min(remaining, 0.1))
            self.assertIn(name, self._outputs, "no {} output".format(name))
            return self._outputs[name]

    def test_projection_metadata_and_backend_independence(self):
        candidates = [
            self._candidate(sensor_id="lidar_camera"),
            self._candidate(sensor_id="ground_plane"),
            self._candidate(sensor_id="gimbal_laser")]
        output = self._publish_and_wait(
            "same", self._message(candidates=candidates))
        self.assertEqual(output.header.frame_id, "map")
        self.assertEqual(len(output.detections), 3)
        for candidate, sensor_id in zip(
                output.detections,
                ("lidar_camera", "ground_plane", "gimbal_laser")):
            self.assertTrue(candidate.position_valid)
            self.assertEqual(candidate.source_candidate_index, 4)
            self.assertEqual(candidate.track_id, 9)
            self.assertEqual(candidate.sensor_id, sensor_id)
            self.assertAlmostEqual(candidate.position.latitude,
                                   47.397742999923459, places=10)
            self.assertAlmostEqual(candidate.position.longitude,
                                   8.545726465771809, places=10)
            self.assertAlmostEqual(candidate.position.altitude,
                                   123.4000078268, places=5)
            self.assertEqual(list(candidate.position_covariance_enu),
                             [1.0, 0.0, 0.0,
                              0.0, 2.0, 0.0,
                              0.0, 0.0, 3.0])

    def test_rotation_and_covariance(self):
        output = self._publish_and_wait(
            "rotated", self._message(frame="rotated_input"))
        candidate = output.detections[0]
        self.assertTrue(candidate.position_valid)
        self.assertAlmostEqual(candidate.position.latitude,
                               47.397832943627087, places=10)
        self.assertAlmostEqual(candidate.position.longitude,
                               8.545593999999996, places=10)
        covariance = candidate.position_covariance_enu
        self.assertAlmostEqual(covariance[0], 2.0, places=9)
        self.assertAlmostEqual(covariance[4], 1.0, places=9)
        self.assertAlmostEqual(covariance[8], 3.0, places=9)
        for row in range(3):
            for column in range(3):
                self.assertAlmostEqual(covariance[3 * row + column],
                                       covariance[3 * column + row], places=12)

    def test_fail_closed_inputs(self):
        zero_stamp = self._message(stamp=rospy.Time(0))
        self.assertFalse(self._publish_and_wait(
            "same", zero_stamp).detections[0].position_valid)

        missing_tf = self._message(frame="no_such_frame")
        self.assertFalse(self._publish_and_wait(
            "missing_tf", missing_tf).detections[0].position_valid)

        invalid = self._candidate(x=float("nan"))
        output = self._publish_and_wait(
            "same", self._message(candidates=[invalid]))
        self.assertFalse(output.detections[0].position_valid)
        self.assertEqual(output.detections[0].track_id, 9)

    def test_origin_topic_is_required_and_validated(self):
        message = self._message()
        self.assertFalse(self._publish_and_wait(
            "topic_origin", message).detections[0].position_valid)

        invalid_origin = GeoPointStamped()
        invalid_origin.position.latitude = math.nan
        invalid_origin.position.longitude = 8.545594
        invalid_origin.position.altitude = 123.4
        self._origin_publisher.publish(invalid_origin)
        rospy.sleep(0.1)
        self.assertFalse(self._publish_and_wait(
            "topic_origin", message).detections[0].position_valid)

        origin = GeoPointStamped()
        origin.position.latitude = 47.397743
        origin.position.longitude = 8.545594
        origin.position.altitude = 123.4
        self._origin_publisher.publish(origin)
        rospy.sleep(0.1)
        self.assertTrue(self._publish_and_wait(
            "topic_origin", message).detections[0].position_valid)


if __name__ == "__main__":
    rospy.init_node("world_detection_georeferencer_test")
    rostest.rosrun("xd_uav_detect", "world_detection_georeferencer",
                   GeoreferencerTest)
