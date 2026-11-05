"""D-FINE output-budget and cached-grid regression tests without model weights."""
import unittest
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np
try:
    import torch
except ImportError:
    torch = None


@unittest.skipIf(torch is None, 'D-FINE torch runtime unavailable in ROS system Python')
class DFinePostprocessTest(unittest.TestCase):
    def backend(self):
        from sar_yolo_detector.pixeagle.backends.dfine_backend import DFineBackend
        backend = DFineBackend.__new__(DFineBackend)
        backend._model = SimpleNamespace(model=SimpleNamespace(
            encoder=SimpleNamespace(eval_spatial_size=[640,640]),
            decoder=SimpleNamespace(eval_spatial_size=[640,640])), post=None,device='cpu',
            encoder_eval_spatial_size=[640,640],decoder_eval_spatial_size=[640,640])
        backend._allowed_class_ids = None
        backend._agnostic_nms = True
        backend._inference_image_size = 640
        return backend

    def output(self):
        return dict(boxes=torch.tensor([[10.,10.,30.,30.],[10.,10.,30.,30.],[55.,10.,75.,30.]]),
                    scores=torch.tensor([.95,.9,.8]),labels=torch.tensor([0,1,0]))

    def test_duplicate_queries_do_not_consume_distinct_target_budget(self):
        backend = self.backend()
        with patch('sar_yolo_detector.pixeagle.backends.dfine_runtime.infer',return_value=self.output()):
            _,rows = backend.detect(np.zeros((100,100,3),np.uint8),max_det=2)
        self.assertEqual(2,len(rows))
        self.assertEqual((55,10,75,30),rows[1].aabb_xyxy)

    def test_disallowed_classes_do_not_consume_output_budget(self):
        backend = self.backend(); backend._allowed_class_ids = {1}
        with patch('sar_yolo_detector.pixeagle.backends.dfine_runtime.infer',return_value=self.output()):
            _,rows = backend.detect(np.zeros((100,100,3),np.uint8),max_det=1)
        self.assertEqual(1,len(rows)); self.assertEqual(1,rows[0].class_id)

    def test_switching_back_to_640_restores_cached_grids(self):
        backend = self.backend()
        with patch('sar_yolo_detector.pixeagle.backends.dfine_runtime.infer',return_value=self.output()):
            backend.detect(np.zeros((100,100,3),np.uint8),image_size=960)
            self.assertIsNone(backend._model.model.encoder.eval_spatial_size)
            backend.detect(np.zeros((100,100,3),np.uint8),image_size=640)
        self.assertEqual([640,640],backend._model.model.encoder.eval_spatial_size)

    def test_decode_filters_nonfinite_and_clips_to_frame(self):
        from sar_yolo_detector.pixeagle.backends.dfine_runtime import normalized_rows
        result = dict(boxes=torch.tensor([[-2.,1.,30.,40.],[0.,0.,float('nan'),5.]]),
                      labels=torch.tensor([1,0]),scores=torch.tensor([.8,.9]))
        rows = normalized_rows(result,20,30,.1,30)
        self.assertEqual(1,len(rows)); self.assertEqual((0,1,20,30),rows[0][2])

    def test_repeated_capture_rejected_before_detector_or_botsort_runs(self):
        from sar_yolo_detector.pixeagle.backends.dfine_backend import DFineBackend
        backend=DFineBackend({'SMART_TRACKER_CLASS_CONSENSUS_ENABLED':True})
        backend._last_tracking_capture_stamp=2.
        with patch.object(backend,'detect') as infer:
            mode,rows=backend.detect_and_track(np.zeros((100,100,3),np.uint8),
                                             tracker_args={'_capture_stamp':2.})
        self.assertEqual(mode,'track');self.assertEqual(rows,[])
        infer.assert_not_called()

    def test_mode_reset_clears_semantic_history_and_capture_clock(self):
        from sar_yolo_detector.pixeagle.backends.dfine_backend import DFineBackend
        backend=DFineBackend({'SMART_TRACKER_CLASS_CONSENSUS_ENABLED':True})
        backend._class_consensus.process([dict(track_id=1,stable=True,label=0,score=.8,box=[0,0,20,20])],1.)
        backend._last_tracking_capture_stamp=1.
        backend.reset_tracking()
        self.assertEqual(backend._class_consensus.tracks,{})
        self.assertIsNone(backend._last_tracking_capture_stamp)


if __name__ == '__main__':
    unittest.main()
