import importlib.util
from pathlib import Path
import unittest
from copy import deepcopy
import rospy
from std_msgs.msg import Header
from xd_uav_track.msg import PerceptionTargetArray
from test_perception_modes import target

spec=importlib.util.spec_from_file_location('revision_probe',str(Path(__file__).with_name('measure_fusion_lock.py')))
module=importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class RevisionProbeTest(unittest.TestCase):
    def test_only_confirmed_overlapping_peer_is_counted(self):
        snapshot=PerceptionTargetArray(targets=[target(), target(8), target(9,70)])
        self.assertEqual(1, module.overlapping_confirmed_peers(snapshot,'test-session:7'))
        snapshot.targets[1].lifecycle_state='lost'
        self.assertEqual(0, module.overlapping_confirmed_peers(snapshot,'test-session:7'))
    def test_later_lost_revision_is_counted(self):
        probe=module.RevisionContinuity()
        snapshot=PerceptionTargetArray(header=Header(stamp=rospy.Time(10)),targets=[target()])
        probe.record(snapshot,'test-session:7',1)
        lost=deepcopy(snapshot)
        lost.targets[0].lifecycle_state='lost'; lost.targets[0].measurement_source='stale'
        probe.record(lost,'test-session:7',2)
        self.assertEqual(1,probe.report()['same_capture_downgrades'])
        self.assertEqual(1,probe.report()['final_missing_captures'])
        probe.record(snapshot,'test-session:7',3)
        self.assertEqual(1,probe.report()['missing_updates'])
        self.assertEqual(0,probe.report()['final_missing_captures'])

    def test_cache_is_bounded(self):
        probe=module.RevisionContinuity()
        for i in range(700):
            probe.record(PerceptionTargetArray(header=Header(stamp=rospy.Time(i+1))),'test-session:7')
        self.assertEqual(600,probe.report()['retained_captures'])
        self.assertEqual(32,len(probe.report()['first_missing_updates']))
