#!/usr/bin/env python3
import threading
import time
import unittest

import rospy
import rostest
from sensor_msgs.msg import CameraInfo, Range
from xd_uav_detect.msg import WorldDetectionArray
from xd_uav_track.msg import DetectionArray, DetectionCandidate


class GimbalRangeNodeFailClosed(unittest.TestCase):
    def setUp(self):
        self._condition = threading.Condition()
        self._outputs = {}
        self._compat_outputs = {}
        self._latest_world = None
        self._detections_pub = rospy.Publisher(
            "/test/input", DetectionArray, queue_size=10)
        self._camera_pub = rospy.Publisher(
            "/test/camera_info", CameraInfo, queue_size=1, latch=True)
        self._range_pub = rospy.Publisher("/test/range", Range, queue_size=10)
        rospy.Subscriber("/test/output", DetectionArray,
                         self._output_callback, queue_size=10)
        rospy.Subscriber("/test/compat_output", DetectionArray,
                         self._compat_output_callback, queue_size=10)
        rospy.Subscriber("/test/world", WorldDetectionArray,
                         self._world_callback, queue_size=10)
        deadline = time.time() + 5.0
        while (self._detections_pub.get_num_connections() == 0 or
               self._camera_pub.get_num_connections() == 0 or
               self._range_pub.get_num_connections() == 0):
            if time.time() > deadline:
                self.fail("detect input subscribers did not connect")
            rospy.sleep(0.05)
        info = CameraInfo()
        info.header.frame_id = "test/camera_optical"
        info.width = 640
        info.height = 480
        info.K = [400.0, 0.0, 320.0, 0.0, 400.0, 240.0, 0.0, 0.0, 1.0]
        info.P = [400.0, 0.0, 320.0, 0.0,
                  0.0, 400.0, 240.0, 0.0,
                  0.0, 0.0, 1.0, 0.0]
        self._camera_pub.publish(info)
        rospy.sleep(0.2)

    def _output_callback(self, message):
        with self._condition:
            self._outputs[message.command] = message
            self._condition.notify_all()

    def _world_callback(self, message):
        with self._condition:
            self._latest_world = message
            self._condition.notify_all()

    def _compat_output_callback(self, message):
        with self._condition:
            self._compat_outputs[message.command] = message
            self._condition.notify_all()

    def _range(self, stamp, value=10.0):
        message = Range()
        message.header.stamp = stamp
        message.header.frame_id = "test/laser"
        message.radiation_type = Range.INFRARED
        message.field_of_view = 0.001
        message.min_range = 0.2
        message.max_range = 30.0
        message.range = value
        return message

    def _detections(self, seq, stamp, boxes):
        message = DetectionArray()
        message.header.seq = seq
        message.header.stamp = stamp
        message.header.frame_id = "test/camera_optical"
        message.command = str(seq)
        message.image_width = 640
        message.image_height = 480
        for coordinates in boxes:
            candidate = DetectionCandidate()
            candidate.has_bbox = True
            candidate.bbox = coordinates
            message.candidates.append(candidate)
        return message

    def _publish_case(self, seq, range_message, detection):
        with self._condition:
            self._latest_world = None
        if range_message is not None:
            self._range_pub.publish(range_message)
            rospy.sleep(0.05)
        deadline = time.time() + 4.0
        while time.time() < deadline:
            self._detections_pub.publish(detection)
            with self._condition:
                key = str(seq)
                if (key in self._outputs and key in self._compat_outputs and
                        self._latest_world is not None):
                    primary = self._outputs[key]
                    compatibility = self._compat_outputs[key]
                    # roscpp assigns Header.seq independently per publisher;
                    # all contract fields must otherwise remain identical.
                    self.assertEqual(primary.header.stamp,
                                     compatibility.header.stamp)
                    self.assertEqual(primary.header.frame_id,
                                     compatibility.header.frame_id)
                    self.assertEqual(primary.image_width,
                                     compatibility.image_width)
                    self.assertEqual(primary.image_height,
                                     compatibility.image_height)
                    self.assertEqual(primary.command, compatibility.command)
                    self.assertEqual(primary.image_source,
                                     compatibility.image_source)
                    self.assertEqual(primary.sensor_id,
                                     compatibility.sensor_id)
                    self.assertEqual(primary.detector_name,
                                     compatibility.detector_name)
                    self.assertEqual(primary.model_version,
                                     compatibility.model_version)
                    self.assertEqual(primary.candidates,
                                     compatibility.candidates)
                    return self._outputs[key], self._latest_world
                self._condition.wait(0.1)
        self.fail("timed out waiting for case {}".format(seq))

    def _assert_invalid(self, output, world):
        self.assertTrue(output.candidates)
        for candidate in output.candidates:
            self.assertFalse(candidate.range_valid, repr(candidate))
            self.assertFalse(candidate.has_relative_position_body)
        for candidate in world.detections:
            self.assertFalse(candidate.position_valid)

    def test_fail_closed_inputs_and_world_tf(self):
        center = [300, 220, 340, 260]
        now = rospy.Time.now()
        output, world = self._publish_case(
            1, self._range(now), self._detections(1, now, [center]))
        self.assertTrue(output.candidates[0].range_valid)
        self.assertAlmostEqual(
            output.candidates[0].relative_position_body[0], 10.0, places=4)
        self.assertFalse(world.detections[0].position_valid)

        output, world = self._publish_case(
            2, self._range(rospy.Time.now()),
            self._detections(2, rospy.Time(), [center]))
        self._assert_invalid(output, world)

        stamp = rospy.Time.now()
        output, world = self._publish_case(
            3, self._range(stamp),
            self._detections(3, stamp + rospy.Duration(1.0), [center]))
        self._assert_invalid(output, world)

        stamp = rospy.Time.now()
        output, world = self._publish_case(
            4, self._range(stamp, 30.0),
            self._detections(4, stamp, [center]))
        self._assert_invalid(output, world)

        stamp = rospy.Time.now()
        output, world = self._publish_case(
            5, self._range(stamp),
            self._detections(5, stamp, [[0, 0, 100, 100]]))
        self._assert_invalid(output, world)

        stamp = rospy.Time.now()
        output, world = self._publish_case(
            6, self._range(stamp),
            self._detections(6, stamp, [center, center]))
        self._assert_invalid(output, world)

        stamp = rospy.Time.now()
        missing_tf_range = self._range(stamp)
        missing_tf_range.header.frame_id = "test/missing_laser"
        output, world = self._publish_case(
            7, missing_tf_range, self._detections(7, stamp, [center]))
        self._assert_invalid(output, world)


if __name__ == "__main__":
    rospy.init_node("gimbal_range_node_test")
    rostest.rosrun("xd_uav_detect", "gimbal_range_node_fail_closed",
                   GimbalRangeNodeFailClosed)
