#!/usr/bin/env python3
"""FRD provenance and ages must describe real observations, not camera headers."""
from copy import deepcopy
import unittest
from unittest.mock import patch
import rospy
from std_msgs.msg import Header
from xd_uav_track.msg import TrackState, TrackStateArray
from test_perception_modes import manager_fixture, frame


def metric_track():
    item = TrackState(track_id=7, class_id=2, confidence=.9, detected=True,
        predicted=False, lifecycle_state="confirmed",control_measurement_ready=True,
        has_relative_position_body=True, range_valid=True,
        relative_position_body=[10., 3., -5.],position_frame="base_link_frd",
        position_stamp=rospy.Time(10),last_observation_stamp=rospy.Time(10),
        last_detection_stamp=rospy.Time(10),
        position_covariance=[.25,0,0,0,.5,0,0,0,.75],
        bbox=[50,40,86,64],normalized_bbox=[68/180,52/120,36/180,24/120])
    return item


class PositionMetadataTest(unittest.TestCase):
    def output(self, track, capture=10., now=10.1):
        manager = manager_fixture(); manager._mode = "detect"
        stamp = rospy.Time.from_sec(capture)
        manager._frames["fixed"].append((1, stamp, frame()))
        array = TrackStateArray(header=Header(stamp=stamp,frame_id="fixed_camera_optical"),
            image_source="fixed",tracks=[track])
        with patch.object(rospy.Time, "now", return_value=rospy.Time.from_sec(now)):
            manager._tracks_callback(array)
        return manager._targets_pub.messages[-1].targets[0]

    def test_real_position_is_frd_not_camera_and_age_includes_delivery_delay(self):
        output = self.output(metric_track())
        self.assertTrue(output.position_valid)
        self.assertEqual("base_link_frd",output.position_frame)
        self.assertEqual(rospy.Time(10),output.position_stamp)
        self.assertEqual([10.,3.,-5.],list(output.position_xyz))
        self.assertAlmostEqual(100., output.age_ms, places=2)

    def test_prediction_keeps_last_real_stamp_and_never_validates_old_position(self):
        track = metric_track(); track.detected=False;track.predicted=True
        track.lifecycle_state="occluded";track.control_measurement_ready=False
        output = self.output(track,10.2,10.25)
        self.assertEqual("prediction",output.measurement_source)
        self.assertFalse(output.position_valid)
        self.assertEqual(rospy.Time(10), output.position_stamp)
        self.assertEqual(rospy.Time(10), output.last_detection_stamp)
        self.assertAlmostEqual(250.,output.age_ms,places=2)
        delayed = self.output(track,10.2,11.0)
        self.assertEqual("stale",delayed.measurement_source,
                         "prediction deadline includes transport latency, not only frame gaps")

    def test_missing_future_stale_and_wrong_frame_metadata_fail_closed(self):
        original = metric_track()
        for field, value in (("position_stamp",rospy.Time(0)),
                             ("position_stamp",rospy.Time(11)),
                             ("position_frame","fixed_camera_optical"),
                             ("range_valid",False),
                             ("control_measurement_ready",False)):
            item=deepcopy(original);setattr(item,field,value)
            self.assertFalse(self.output(item).position_valid, field)
        self.assertFalse(self.output(original,now=12).position_valid)
        self.assertFalse(self.output(original,now=9).position_valid)

    def test_metric_hold_does_not_refresh_position_when_image_is_fresh(self):
        output = self.output(metric_track(),10.2,10.25)
        self.assertEqual(rospy.Time(10),output.position_stamp)
        self.assertFalse(output.position_valid)

    def test_nonfinite_or_indefinite_covariance_is_not_a_valid_measurement(self):
        for covariance in ([float("nan")]*9,[-1,0,0,0,1,0,0,0,1],
                           [1,2,0,2,1,0,0,0,1], [1,1,0,0,1,0,0,0,1]):
            item=metric_track();item.position_covariance=covariance
            self.assertFalse(self.output(item).position_valid)
        item=metric_track();item.relative_position_body=[float("inf"),0,0]
        self.assertFalse(self.output(item).position_valid)

    def test_lost_and_unknown_age_are_not_fresh_valid_positions(self):
        item=metric_track();item.detected=False;item.predicted=True;item.lifecycle_state="lost"
        output=self.output(item,10.3,10.4)
        self.assertEqual("stale",output.measurement_source)
        self.assertFalse(output.position_valid)
        self.assertAlmostEqual(400.,output.age_ms,places=2)
        item.last_observation_stamp=rospy.Time(0)
        item.last_detection_stamp=rospy.Time(0)
        output=self.output(item,10.3,10.4)
        self.assertEqual(-1.,output.age_ms)


if __name__ == "__main__":unittest.main()
