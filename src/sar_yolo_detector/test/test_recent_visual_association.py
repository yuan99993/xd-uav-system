import unittest
import numpy as np
from sar_yolo_detector.box_fusion import BoxFusion,Candidate,FusionResult
from test_box_fusion import LocalTracker,image
from sar_yolo_detector.camera_motion import transform_window
class RecentVisualReferenceTest(unittest.TestCase):
    def fixture(self):
        t=LocalTracker();f=BoxFusion(dict(latency_aware_association=True,strong_takeover_safety_enabled=True),tracker=t)
        f.initialize(image(),(60,60,40,40),10.,7,Candidate((60,60,40,40),0,.9,3,detector_stamp=10.))
        f.box=(100.,60.,40.,40.);f.stamp=f.last_observation_stamp=11.96
        f._camera_support=transform_window(f.box,np.eye(3))[1]
        f._detector_reference_box=(60.,60.,40.,40.);f._detector_reference_stamp=10.
        f._detector_velocity=np.zeros(2);f.velocity=np.array([10.,0.])
        f._velocity_history.extend((stamp,f.velocity.copy()) for stamp in (11.88,11.92,11.96))
        f.result=FusionResult(f.box,7,0,.9,'visual','verified',10.,3);t.box=f.box
        return f
    def test_fresh_verified_candidate_survives_prefilter_without_skipping_confirmation(self):
        f=self.fixture();frame=image((100,60,40,40))
        r=f.step(frame,12.,[Candidate((100,60,40,40),0,.9,3,detector_stamp=12.,selected_public_id=7)],camera_transform=np.eye(3),camera_motion_reliable=True)
        self.assertFalse(r.observed)
        self.assertEqual(r.reason,'reacquisition_confirmation')
        self.assertTrue(f.diagnostics['branches']['detector']['accepted'])
        self.assertTrue(f.diagnostics.get('recent_visual_association_reference'))
        self.assertEqual(r.public_id,7)
    def test_recent_visual_does_not_allow_far_neighbor_into_association(self):
        f=self.fixture();frame=image((100,60,40,40));frame[:,200:240]=image((200,60,40,40))[:,200:240]
        f.step(frame,12.,[Candidate((200,60,40,40),0,.9,3,detector_stamp=12.,selected_public_id=7)],camera_transform=np.eye(3),camera_motion_reliable=True)
        self.assertFalse(f.diagnostics.get('recent_visual_association_reference',False))
        self.assertEqual(f.diagnostics['strong_takeover_vetoes'].get('long_gap_identity_geometry_changed'),1)
