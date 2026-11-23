"""Camera transport preserves the learned template and independent search."""
import unittest
from unittest.mock import Mock, patch
import numpy as np
from sar_yolo_detector.manual_box_tracker import ManualBoxTracker

class NanoPoseSearchTest(unittest.TestCase):
    def test_opt_in_transport_does_not_warp_or_reinitialize(self):
        tracker=ManualBoxTracker({'feature_tracker':{'nano_pose_search_only':True}})
        tracker._active=True;tracker._last_bbox=(100.,80.,30.,20.)
        tracker._tracker_name='nano_trt'
        tracker._opencv_tracker=Mock();tracker._opencv_tracker.reposition_search.return_value=True
        tracker.initialize=Mock(side_effect=AssertionError('template must survive'))
        matrix=np.array([[1.,0.,3.],[0.,1.,2.],[0.,0.,1.]])
        with patch('cv2.warpPerspective',side_effect=AssertionError('no image warp')):
            self.assertTrue(tracker.compensate_motion(np.zeros((240,320,3),np.uint8),matrix))
        self.assertEqual(tracker._last_bbox,(103.,82.,30.,20.))
        self.assertEqual(tracker._pixel_fraction,(0.,0.,0.,0.))
        tracker._opencv_tracker.reposition_search.assert_called_once()

    def test_backend_transports_guard_anchor_and_preserves_template_and_hysteresis(self):
        from sar_yolo_detector.nano_tensorrt import NanoTensorRT
        backend=NanoTensorRT.__new__(NanoTensorRT)
        template=object();backend.template=template
        backend.guard_anchor_center=np.array([90.,70.]);backend.guard_anchor_size=np.array([20.,10.])
        backend.response_guard=Mock();guard=backend.response_guard
        matrix=np.array([[1.,0.,3.],[0.,1.,2.],[0.,0.,1.]])
        self.assertTrue(backend.reposition_search((103.,82.,30.,20.),matrix))
        np.testing.assert_allclose(backend.center,[118.,92.])
        np.testing.assert_allclose(backend.guard_anchor_center,[93.,72.])
        np.testing.assert_allclose(backend.guard_anchor_size,[20.,10.])
        self.assertIs(backend.template,template);guard.reset.assert_not_called()

if __name__=='__main__':unittest.main()
