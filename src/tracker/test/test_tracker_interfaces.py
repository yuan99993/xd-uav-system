#!/usr/bin/env python3
import threading
import unittest

import rospy
import rostest

from sensor_msgs.msg import CameraInfo
from pod_msgs.msg import GimbalState
from tracker.msg import (DetectionArray, DetectionCandidate, NormalizedError,
                         TrackStateArray)
from tracker.srv import SelectTrack, SelectTrackRequest


class TrackerInterfacesTest(unittest.TestCase):
    def setUp(self):
        self._lock = threading.Lock()
        self._tracks = None
        self._error = None
        self._publisher = rospy.Publisher(
            '/tracker_interface_test/detection_candidates', DetectionArray,
            queue_size=1)
        self._subscriber = rospy.Subscriber(
            '/tracker_interface_test/track_states', TrackStateArray,
            self._tracks_callback, queue_size=1)
        self._error_subscriber = rospy.Subscriber(
            '/tracker_interface_test/normalized_error', NormalizedError,
            self._error_callback, queue_size=1)
        self._camera_info_publisher = rospy.Publisher(
            '/tracker_interface_test/camera_info', CameraInfo, queue_size=1,
            latch=True)
        self._gimbal_state_publisher = rospy.Publisher(
            '/tracker_interface_test/gimbal_state', GimbalState, queue_size=1,
            latch=True)

    def _tracks_callback(self, message):
        with self._lock:
            self._tracks = message

    def _error_callback(self, message):
        with self._lock:
            self._error = message

    def _wait(self, predicate, timeout=5.0):
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        rate = rospy.Rate(100)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            with self._lock:
                if predicate():
                    return True
            rate.sleep()
        return False

    @staticmethod
    def _candidate(track_id, center_x):
        result = DetectionCandidate()
        result.track_id = track_id
        result.track_id_is_stable = True
        result.class_id = 1
        result.has_normalized_bbox = True
        result.normalized_bbox = [center_x, 0.5, 0.2, 0.2]
        result.confidence = 0.9
        return result

    def test_track_list_and_explicit_selection(self):
        self.assertTrue(self._wait(
            lambda: self._publisher.get_num_connections() > 0))
        self.assertTrue(self._wait(
            lambda: self._camera_info_publisher.get_num_connections() > 0))
        self.assertTrue(self._wait(
            lambda: self._gimbal_state_publisher.get_num_connections() > 0))
        gimbal_state = GimbalState()
        gimbal_state.header.stamp = rospy.Time.now()
        gimbal_state.connected = True
        gimbal_state.stabilized = True
        gimbal_state.attitude_valid = True
        gimbal_state.parent_frame = 'base_link'
        gimbal_state.gimbal_frame = 'pod_gimbal_link'
        gimbal_state.optical_frame = 'eo_optical_frame'
        self._gimbal_state_publisher.publish(gimbal_state)
        camera_info = CameraInfo()
        camera_info.header.stamp = rospy.Time.now()
        camera_info.header.frame_id = 'eo_optical_frame'
        camera_info.width = 640
        camera_info.height = 480
        camera_info.K = [320.0, 0.0, 320.0,
                         0.0, 320.0, 240.0,
                         0.0, 0.0, 1.0]
        self._camera_info_publisher.publish(camera_info)
        message = DetectionArray()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = 'eo_optical_frame'
        message.candidates = [self._candidate(11, 0.3),
                              self._candidate(12, 0.7)]
        self._publisher.publish(message)
        self.assertTrue(self._wait(
            lambda: self._tracks is not None and len(self._tracks.tracks) == 2))

        rospy.wait_for_service('/tracker_interface_test/select_track', 5.0)
        select = rospy.ServiceProxy(
            '/tracker_interface_test/select_track', SelectTrack)
        request = SelectTrackRequest()
        request.target_id = 12
        request.start_tracking = True
        # image_source is the logical EO/IR stream name.  The optical TF frame
        # remains in DetectionArray.header.frame_id.
        request.image_source = 'eo'
        request.capture_timestamp = message.header.stamp
        response = select(request)
        self.assertTrue(response.success, response.message)
        self.assertEqual(12, response.selected_target_id)

        message.header.stamp = rospy.Time.now()
        self._publisher.publish(message)
        self.assertTrue(self._wait(lambda: any(
            track.track_id == 12 and track.selected for track in self._tracks.tracks)))
        self.assertTrue(self._wait(
            lambda: self._error is not None and self._error.has_angular_error))
        with self._lock:
            yaw_error = self._error.yaw_error_rad
        self.assertAlmostEqual(0.38051, yaw_error, delta=0.04)

        invalid = SelectTrackRequest()
        invalid.target_id = 999
        invalid.start_tracking = True
        invalid_response = select(invalid)
        self.assertFalse(invalid_response.success)


if __name__ == '__main__':
    rospy.init_node('tracker_interfaces_test')
    rostest.rosrun('tracker', 'tracker_interfaces', TrackerInterfacesTest)
