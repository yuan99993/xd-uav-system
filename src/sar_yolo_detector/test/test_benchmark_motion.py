import math
from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools/ue'))
from benchmark_motion import entity_position,BenchmarkClock
from benchmark_visibility import trace_blocked

class MotionTests(unittest.TestCase):
    def test_legacy_motion_is_unchanged(self):
        e=dict(position=[1,2,-3],velocity=[.6,-.4,.2])
        for t in [-1,0,5,40,100]:self.assertEqual(entity_position(e,t),[p+v*20*math.sin(t/20) for p,v in zip(e['position'],e['velocity'])])
    def test_configured_sine_is_bounded_and_has_declared_peak_speed(self):
        e=dict(position=[0,0,-15],velocity=[0,0,0],trajectory=dict(kind='sine',amplitude_m=[40,0,0],period_sec=2*math.pi*40/12))
        dt=.00001
        v=(entity_position(e,dt)[0]-entity_position(e,0)[0])/dt
        self.assertAlmostEqual(v,12,places=5)
        for i in range(1000):self.assertLessEqual(abs(entity_position(e,i*.1)[0]),40)
    def test_manual_clock_never_starts_before_command_and_can_hold_reset(self):
        c=BenchmarkClock(True);self.assertEqual(c.sample(100,now=50),0)
        c.command('start',now=50);self.assertEqual(c.sample(0,now=52),2)
        c.command('hold',now=53);self.assertEqual(c.sample(0,now=99),3)
        c.command('start',now=100);self.assertEqual(c.sample(0,now=102),5)
        c.command('reset',now=103);self.assertEqual(c.sample(200,now=200),0)
    def test_repeated_start_does_not_reset_motion(self):
        c=BenchmarkClock(True);c.command('start',now=10);c.command('start',now=12)
        self.assertEqual(c.sample(0,now=14),4)
    def test_unreal_optional_collision_outputs(self):
        self.assertFalse(trace_blocked(None));self.assertFalse(trace_blocked(False))
        self.assertFalse(trace_blocked((False,None)));self.assertTrue(trace_blocked((True,object())))
        self.assertTrue(trace_blocked(object()))
    def test_normal_clock_is_elapsed_not_test_wall_time(self):self.assertEqual(BenchmarkClock(False).sample(9,now=500),9)
    def test_bad_trajectory_is_rejected(self):
        for tr in [dict(kind='teleport'),dict(kind='sine',amplitude_m=[1,2,3],period_sec=0)]:
            with self.assertRaises(ValueError):entity_position(dict(position=[0,0,0],trajectory=tr),2)

    def test_step_events_are_absolute_offsets_and_reject_unordered_times(self):
        e=dict(position=[1,2,-3],trajectory=dict(kind='step_offsets',steps=[
            dict(at_sec=15,offset_m=[0,0,-200]),dict(at_sec=20,offset_m=[4,0,0])]))
        self.assertEqual(entity_position(e,14.99),[1,2,-3])
        self.assertEqual(entity_position(e,15),[1,2,-203])
        self.assertEqual(entity_position(e,20),[5,2,-3])
        e['trajectory']['steps'].reverse()
        with self.assertRaises(ValueError):entity_position(e,30)

if __name__=='__main__':unittest.main()
