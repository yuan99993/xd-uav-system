#!/usr/bin/env python3
import math
from pathlib import Path
import sys
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools/ue'))
from target_geometry import sightline_evidence,target_domain
from gimbal_selection import SelectionSightline

class GeometryTests(unittest.TestCase):
    def state(self,name='drone',stamp=100):
        return dict(stamp=stamp,frame_id='camera',track_id=1,tracker_type='fusion_tracker',
            class_id=2,class_name=name,has_selection=True,measurement_current=True,
            control_ready=True,prediction_only=False,stale=False,tentative=False,
            confidence=.9,center=[600,270])
    def test_all_domains_are_directions_not_positions(self):
        for name,domain in [('car','ground_candidate'),('tank','ground_candidate'),
                            ('drone','airborne'),('uav','airborne'),('','unknown'),('manual','unknown')]:
            e=sightline_evidence(self.state(name),[1,2,3],[5,6,-30])
            self.assertEqual(e['target_domain'],domain)
            self.assertAlmostEqual(np.linalg.norm(e['ray_world_unit']),1.)
            for flag in ['range_valid','target_position_valid','ground_projection_applied']:
                self.assertFalse(e[flag])
            self.assertNotIn('target_position_ned_m',e)
    def test_camera_height_never_becomes_aircraft_range(self):
        # A ray toward an airborne drone may also intersect terrain behind it.
        # Neither that intersection nor the bbox centre is a measured distance.
        for height in [-30,-300]:
            e=sightline_evidence(self.state(),[1,0,1],[0,0,height])
            self.assertEqual(e['metric_policy'],'independent_range_or_triangulation_required')
            self.assertFalse(e['range_valid'])
    def test_ground_class_does_not_prove_contact_or_known_surface(self):
        e=sightline_evidence(self.state('car'),[1,0,1],[0,0,-30])
        self.assertEqual(e['metric_policy'],'validated_ground_contact_and_surface_required')
        self.assertFalse(e['ground_projection_applied'])
    def test_invalid_rays_and_optional_origin(self):
        for ray in [[0,0,0],[math.nan,0,1],[1,2]]:
            with self.assertRaises(ValueError):sightline_evidence(self.state(),ray)
        self.assertIsNone(sightline_evidence(self.state(),[1,0,0],[math.inf,0,0])['camera_origin_ned_m'])
    def test_ground_air_identical_pixels_give_identical_servo_not_depth(self):
        values=[]
        for name in ['car','drone','']:
            s=SelectionSightline();s.capture(dict(capture_wall_start=100,width=960,height=540,
                horizontal_fov_deg=90,measured_camera_rotation_deg=dict(pitch=-45,yaw=30,roll=0),
                camera_position_ned_m=[0,0,-30]),'camera')
            v=s.observe(self.state(name),100.1);values.append(v)
            self.assertFalse(v['geometry']['range_valid'])
            self.assertFalse(v['geometry']['ground_projection_applied'])
            st=self.state(name,101);st['prediction_only']=True
            self.assertIsNone(s.observe(st,101.1))
        for v in values[1:]:
            self.assertEqual(v['yaw_world_deg'],values[0]['yaw_world_deg'])
            self.assertEqual(v['pitch_world_deg'],values[0]['pitch_world_deg'])
    def test_upward_air_sightline_is_accepted(self):
        s=SelectionSightline();s.capture(dict(capture_wall_start=100,width=960,height=540,
            horizontal_fov_deg=90,measured_camera_rotation_deg=dict(pitch=20,yaw=0,roll=0)), 'camera')
        v=s.observe(self.state(),100.1)
        self.assertGreater(v['pitch_world_deg'],0)
        self.assertLess(v['geometry']['ray_world_unit'][2],0)

if __name__=='__main__':unittest.main()
