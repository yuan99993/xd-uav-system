#!/usr/bin/env python3
import io
import statistics
import time
import tracemalloc
import unittest
from copy import deepcopy
from unittest.mock import patch
import rospy
from std_msgs.msg import Header
from xd_uav_track.msg import TargetTrajectoryArray
from xd_uav_track.trajectory import TargetTrajectory
from test_perception_modes import manager_fixture, target


class TrajectoryTest(unittest.TestCase):
    def setUp(self):
        self.trail = TargetTrajectory()
        self.target = target()

    def add(self, seconds, source="fixed"):
        return self.trail.append(self.target, Header(stamp=rospy.Time.from_sec(seconds)), source, 1)

    def test_observed_predicted_loss_and_recovery_are_separate(self):
        self.add(10)
        self.target.measurement_source="prediction";self.target.lifecycle_state="occluded"
        self.target.position_valid=True
        self.add(10.2)
        self.assertTrue(self.trail.points[-1].predicted)
        self.assertFalse(self.trail.points[-1].position_valid)
        self.target.measurement_source="stale";self.target.lifecycle_state="lost"
        self.add(10.4)
        self.assertFalse(self.trail.active)
        self.assertEqual(2,len(self.trail.points))
        self.target.measurement_source="detector";self.target.lifecycle_state="confirmed"
        self.add(10.6)
        self.assertEqual(2,self.trail.points[-1].segment_id)
        self.assertEqual("loss",self.trail.points[-1].segment_reason)

    def test_sources_and_long_gap_break_pixel_segments(self):
        self.add(10);self.add(10.2,"gimbal")
        self.assertEqual("source_change",self.trail.points[-1].segment_reason)
        self.add(12,"gimbal")
        self.assertEqual("observation_gap",self.trail.points[-1].segment_reason)

    def test_duplicates_and_late_results_do_not_add_points_or_resurrect_lost(self):
        self.add(10);self.add(10)
        self.assertEqual(1,len(self.trail.points))
        self.target.lifecycle_state="lost"
        self.add(11)
        self.target.lifecycle_state="confirmed"
        self.assertFalse(self.add(10.5))
        self.assertEqual("LOST",self.trail.status)

    def test_identity_merge_keeps_path_and_new_selection_resets(self):
        self.add(10);self.trail.remap(7,31,"test-session")
        self.target.track_id=31;self.target.target_uid="test-session:31"
        self.add(10.2)
        self.assertEqual(2,len(self.trail.points))
        self.target.track_id=32;self.target.target_uid="test-session:32"
        self.add(10.4)
        self.assertEqual(1,len(self.trail.points))

    def test_time_and_capacity_bounds(self):
        for index in range(3000):self.add(1+index*.1)
        self.assertLessEqual(len(self.trail.points),600)
        self.assertLessEqual((self.trail.points[-1].capture_stamp-self.trail.points[0].capture_stamp).to_sec(),60)
        self.trail.expire(rospy.Time(400))
        self.assertEqual(0,len(self.trail.points))

    def test_new_selection_does_not_publish_previous_targets_active_path(self):
        self.add(10)
        replacement=target(32)
        self.trail.begin(replacement)
        self.assertEqual(32,self.trail.track_id)
        self.assertEqual(0,len(self.trail.points))
        self.assertFalse(self.trail.active)
        self.assertEqual("WAITING_OBSERVATION",self.trail.status)

    def test_mode_manager_publishes_and_closes_trajectory(self):
        manager=manager_fixture();manager._mode="track"
        manager._selected_uid=self.target.target_uid;manager._selected_source="fixed"
        manager._selected_track_id=7
        stamp=rospy.Time(10)
        manager._latest_targets["fixed"]=[deepcopy(self.target)]
        manager._latest_targets["fixed"][0].selected=True
        manager._latest_headers["fixed"]=Header(stamp=stamp)
        with patch.object(rospy.Time,"now",return_value=stamp):
            manager._publish_targets("fixed",Header(stamp=stamp),180,120)
            self.assertEqual(1,len(manager._trajectory_pub.messages[-1].points))
            manager._clear_selection()
        self.assertFalse(manager._trajectory_pub.messages[-1].active)
        self.assertEqual("INACTIVE",manager._trajectory_pub.messages[-1].status)

    def test_performance_and_memory_are_bounded_at_30fps(self):
        # Three simulated minutes, no ROS master or GPU. One point stream only.
        timings=[]
        tracemalloc.start()
        for index in range(5400):
            start=time.perf_counter()
            self.add(1+index/30.0)
            if index>3600:timings.append((time.perf_counter()-start)*1000)
        _,peak=tracemalloc.get_traced_memory();tracemalloc.stop()
        message=TargetTrajectoryArray(points=list(self.trail.points))
        serialization=[]
        for _ in range(20):
            output=io.BytesIO();start=time.perf_counter();message.serialize(output)
            serialization.append((time.perf_counter()-start)*1000)
        p95=sorted(timings)[int(len(timings)*.95)]
        serial_p95=sorted(serialization)[18]
        print("trajectory performance: append p95=%.3fms serialize p95=%.3fms peak=%.2fMiB bytes=%d points=%d" %
              (p95,serial_p95,peak/1048576,len(output.getvalue()),len(self.trail.points)))
        self.assertLess(p95,1.0)
        self.assertLess(serial_p95,20.0)
        self.assertLess(peak,4*1048576)
        self.assertLessEqual(len(self.trail.points),600)


if __name__=="__main__":unittest.main()
