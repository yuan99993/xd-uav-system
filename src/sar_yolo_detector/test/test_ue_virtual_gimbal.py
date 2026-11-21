#!/usr/bin/env python3
import json
import math
from pathlib import Path
import sys
import unittest
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools/ue'))
from virtual_gimbal import VirtualGimbal
from gimbal_selection import SelectionSightline
from geometry import unreal_camera_optical_rotation


class GimbalTests(unittest.TestCase):
    def test_rate_acceleration_and_command_delay(self):
        g=VirtualGimbal(dict(command_delay_sec=.1,max_rate_deg_s=6,max_accel_deg_s2=12,measurement_timeout_sec=5))
        g.step(0);g.command(1,0,40,-30)
        old=g.feedback()
        for i in range(1,401):
            now=i*.01;r=g.step(now)
            if now<=.1:self.assertAlmostEqual(r['yaw_deg'],0)
            self.assertLessEqual(abs(r['yaw_rate_deg_s']),6+1e-8)
            self.assertLessEqual(abs(r['yaw_rate_deg_s']-old['yaw_rate_deg_s']),12*.01+1e-8)
            self.assertLessEqual(abs(r['yaw_deg']-old['yaw_deg']),6*.01+1e-8)
            old=r
        self.assertGreater(r['yaw_deg'],15)

    def test_cancel_brakes_and_prediction_times_out(self):
        g=VirtualGimbal(dict(command_delay_sec=0,measurement_timeout_sec=.4))
        g.step(0);g.command(1,0,50,-30)
        for i in range(1,201):r=g.step(i*.01)
        self.assertEqual(r['mode'],'LOST')
        self.assertLess(abs(r['yaw_rate_deg_s']),.1)
        g.command(2,2);g.step(2.1)
        self.assertEqual(g.feedback()['mode'],'HOLD')
        self.assertFalse(g.command(1,2.1,60,-10))

    def test_hard_limits_and_stall(self):
        g=VirtualGimbal(dict(yaw_limits_deg=[-10,10],measurement_timeout_sec=5))
        g.step(0);g.command(1,0,80,80)
        for i in range(1,301):r=g.step(i*.01)
        self.assertLessEqual(r['yaw_deg'],10);self.assertLessEqual(r['pitch_deg'],45)
        self.assertTrue(r['saturated'])
        g=VirtualGimbal();g.step(0);g.command(1,0,100,-70)
        self.assertLess(abs(g.step(10)['yaw_deg']),1.)

    @staticmethod
    def state(stamp=100,u=600,v=270):
        return dict(stamp=stamp,frame_id='camera',track_id=1,tracker_type='fusion_tracker',
                    has_selection=True,measurement_current=True,control_ready=True,
                    prediction_only=False,stale=False,tentative=False,confidence=.8,center=[u,v])

    def test_capture_time_ray_and_signs(self):
        s=SelectionSightline()
        s.capture(dict(capture_wall_start=100,width=960,height=540,horizontal_fov_deg=90,
                       measured_camera_rotation_deg=dict(pitch=-45,yaw=30,roll=0)),'camera')
        r=s.observe(self.state(),100.2)
        self.assertGreater(r['yaw_world_deg'],30)
        self.assertLess(abs(r['pitch_world_deg']+45),3)
        self.assertIsNone(s.observe(self.state(),100.3))
        s2=SelectionSightline();s2.frames=s.frames
        r=s2.observe(self.state(v=400),100.2)
        self.assertLess(r['pitch_world_deg'],-45)

    def test_current_local_visual_is_accepted_without_fabricating_detector_evidence(self):
        from gimbal_selection import current_visual_measurement
        value=self.state();value['measurement_current']=False
        self.assertFalse(current_visual_measurement(value))
        for method in ['fusion_visual','manual_KCF','manual_color_roi_camshift']:
            value['association_method']=method
            self.assertTrue(current_visual_measurement(value))
            for key in ['prediction_only','stale','tentative']:
                bad=dict(value);bad[key]=True
                self.assertFalse(current_visual_measurement(bad))
            bad=dict(value);bad['control_ready']=False
            self.assertFalse(current_visual_measurement(bad))
        for method in ['fusion_prediction','fusion_lost','fusion_ambiguous','unknown']:
            value['association_method']=method
            self.assertFalse(current_visual_measurement(value))

    def test_prediction_stale_mismatch_and_cancel(self):
        s=SelectionSightline();s.capture(dict(capture_wall_start=100,width=960,height=540,horizontal_fov_deg=90,
                       measured_camera_rotation_deg=dict(pitch=-70,yaw=0,roll=0)),'camera')
        for key in ['prediction_only','stale','tentative']:
            state=self.state();state[key]=True;self.assertIsNone(s.observe(state,100.2))
        self.assertIsNone(s.observe(self.state(),103))
        state=self.state();state['frame_id']='other';self.assertIsNone(s.observe(state,100.2))
        self.assertIsNotNone(s.observe(self.state(),100.2))
        state=self.state(101);state['has_selection']=False
        self.assertTrue(s.observe(state,101)['cancel'])

    def test_body_yaw_compensation(self):
        g=VirtualGimbal(dict(command_delay_sec=0,measurement_timeout_sec=5))
        g.step(0);g.command(1,0,40,-70)
        for i in range(1,301):r=g.step(i*.01,body_yaw=40)
        self.assertLess(abs(r['yaw_deg']),.01)
        self.assertAlmostEqual(r['yaw_world_deg'],40,places=2)

    def test_visual_loop_speeds(self):
        # Ground truth only generates/evaluates pixels; actuator receives solely
        # captured pixels and measured camera orientation through SelectionSightline.
        results=[]
        for speed in [0.,2.,8.,20.]:
            g=VirtualGimbal(dict(home_pitch_deg=-20,command_delay_sec=.08))
            s=SelectionSightline();queue=[];sequence=0;errors=[];g.step(100)
            for i in range(1,2001):
                now=100+i*.01;true_yaw=-20+speed*(now-100)
                # Stay inside physical limits while changing direction smoothly.
                true_yaw=30*math.sin(speed/30*(now-100)) if speed else 20
                while queue and queue[0][0]<=now:
                    _,st=queue.pop(0);cmd=s.observe(st,now)
                    if cmd:
                        sequence+=1;g.command(sequence,now,cmd['yaw_world_deg'],cmd['pitch_world_deg'],now-st['stamp'])
                r=g.step(now)
                if i%20==0:
                    rotation=dict(pitch=r['pitch_deg'],yaw=r['yaw_world_deg'],roll=0)
                    ray=np.array([math.cos(math.radians(-20))*math.cos(math.radians(true_yaw)),
                                  math.cos(math.radians(-20))*math.sin(math.radians(true_yaw)), -math.sin(math.radians(-20))])
                    optical=unreal_camera_optical_rotation(rotation).T@ray
                    u,v=480+480*optical[0]/optical[2],270+480*optical[1]/optical[2]
                    s.capture(dict(capture_wall_start=now,width=960,height=540,horizontal_fov_deg=90,
                                   measured_camera_rotation_deg=rotation),'camera')
                    queue.append((now+.15,self.state(now,u,v)))
                    if i>400:errors.append(abs(u-480)/480)
            results.append(dict(max_target_rate_deg_s=speed,p95_center_error_norm=float(np.percentile(errors,95)),accepted=s.counts['accepted']))
            self.assertLess(np.percentile(errors,95),.35)
            self.assertGreater(s.counts['accepted'],70)
        print('Closed-loop synthetic image tests:',json.dumps(results))

if __name__=='__main__':unittest.main()
