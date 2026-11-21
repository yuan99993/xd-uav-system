import json
import math
from pathlib import Path
import sys
import tempfile
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools/ue'))
from virtual_zoom import FovProfile,VirtualZoom
from gimbal_selection import SelectionSightline
from ue_stream_client import camera_messages
from sar_yolo_detector.camera_motion import CameraMotion,rotation_homography


class VirtualZoomTest(unittest.TestCase):
    def test_default_fixed_lens_and_slew_acceleration_limits(self):
        fixed=FovProfile();zoom=VirtualZoom(fixed.config)
        for i in range(100):self.assertEqual(zoom.step(i*.01,fixed.target(i*.01)),90.)
        profile=FovProfile('zoom_test');zoom=VirtualZoom(profile.config);old=0.;minimum=100
        for i in range(3201):
            stamp=i*.01;value=zoom.step(stamp,profile.target(stamp));minimum=min(minimum,value)
            self.assertTrue(45<=value<=100);self.assertLessEqual(abs(zoom.axis.velocity),8+1e-8)
            if i:self.assertLessEqual(abs(zoom.axis.velocity-old),16*.01+1e-7)
            old=zoom.axis.velocity
        self.assertAlmostEqual(minimum,60.,delta=.05);self.assertAlmostEqual(value,90.,delta=.05)
    def test_profile_rejects_jump_outside_validated_optics(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'bad.json'
            for values in [[[0,30]],[[1,90]],[[0,90],[0,60]],[[0,90],[1,float('nan')]]]:
                path.write_text(json.dumps(dict(schedule=values)))
                with self.assertRaises(ValueError):FovProfile(str(path))
        with self.assertRaises(ValueError):VirtualZoom(dict(max_rate_deg_s=90))
        zoom=VirtualZoom();zoom.step(10,90)
        with self.assertRaises(ValueError):zoom.step(9,90)
    def test_same_world_sightline_uses_actual_zoom_of_each_capture(self):
        sightline=SelectionSightline();rays=[]
        for i,fov in enumerate([90.,60.,90.]):
            stamp=100+i*.1;fx=960/(2*math.tan(math.radians(fov)/2))
            sightline.capture(dict(width=960,height=540,capture_wall_start=stamp,horizontal_fov_deg=fov,
                measured_camera_rotation_deg=dict(pitch=-50,yaw=30,roll=0)),'camera')
            result=sightline.observe(dict(stamp=stamp,frame_id='camera',track_id=1,tracker_type='fusion',
                has_selection=True,measurement_current=False,control_ready=True,association_method='fusion_visual',
                prediction_only=False,stale=False,tentative=False,confidence=.9,center=[480+.2*fx,270+.1*fx]),stamp+.05)
            rays.append([result['yaw_world_deg'],result['pitch_world_deg']])
        np.testing.assert_allclose(rays,[rays[0]]*3,atol=1e-8)
    def test_ros_intrinsics_and_pose_keep_exact_capture_during_zoom(self):
        old=None
        for i,fov in enumerate([90.,89.,75.,60.]):
            meta=dict(frame_seq=i,capture_wall_start=100+i*.1,horizontal_fov_deg=fov,
                camera_position_ned_m=[0,0,-30],measured_camera_rotation_deg=dict(pitch=-70,yaw=0,roll=0))
            image,info,tf=camera_messages(meta,np.zeros((540,960,3),np.uint8),'camera','world')
            expected=960/(2*math.tan(math.radians(fov)/2));self.assertAlmostEqual(info.K[0],expected)
            self.assertEqual(image.header.stamp,info.header.stamp);self.assertEqual(info.header.stamp,tf.header.stamp)
            if old is not None:self.assertGreater(info.K[0],old)
            old=info.K[0]
    def test_slew_remains_inside_existing_camera_motion_scale_gate(self):
        zoom=VirtualZoom();previous=90.;zoom.step(0,45.)
        for i in range(1,100):
            current=zoom.step(i*.1,45.)
            def K(angle):
                f=480/math.tan(math.radians(angle)/2)
                return np.array([[f,0,480],[0,f,270],[0,0,1.]])
            matrix=rotation_homography(K(previous),np.eye(3),np.eye(3),current_intrinsics=K(current))
            self.assertTrue(CameraMotion.validate(matrix,(540,960)));previous=current

if __name__=='__main__':unittest.main()
