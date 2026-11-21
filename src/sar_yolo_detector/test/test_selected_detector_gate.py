"""Selected D-FINE proposals must pass identity gates before receiving an ID."""
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np

from test_fusion_adapter import NODE
from sar_yolo_detector.pixeagle.detection_adapter import NormalizedDetection


class SelectedDetectorGateTest(unittest.TestCase):
    def test_motion_snapshot_recentres_and_expands_search_without_changing_identity(self):
        self.node._tracking_mode='fusion_tracker'
        self.frame=np.zeros((600,800,3),np.uint8)
        self.selection=(7,(300,250,30,30),0,((0,1),(2,)),np.ones(1),.7,.35,7,
            dict(velocity=(900.,0.),stamp=.9,last_detection_stamp=.9,lost=False))
        self.infer([])
        self.assertEqual((384,384),self.backend.detect.call_args.args[0].shape[:2])
        stages=[c for c in self.node._record_stage.call_args_list if c.args[0]=='focused_roi_inference']
        self.assertEqual([213,73],stages[-1].kwargs['offset'])

    def test_loose_proposal_nms_is_limited_to_fusion_mode(self):
        self.infer([self.detection()])
        self.assertEqual(.6,self.backend.detect.call_args.kwargs['iou'])
        self.node._tracking_mode='fusion_tracker'
        self.infer([self.detection()])
        self.assertEqual(.95,self.backend.detect.call_args.kwargs['iou'])

    def test_global_weak_hint_only_schedules_one_roi_and_never_renews_identity(self):
        self.node._tracking_mode='fusion_tracker'
        self.node._fusion.result.source='lost'
        self.node._fusion.last_detection_stamp=.5
        self.frame=np.zeros((600,800,3),np.uint8)
        self.selection=(7,(300,250,30,30),0,((0,1),(2,)),np.ones(1),.7,.35,7)
        self.infer([]);self.node._roi_misses=3
        weak=self.detection(box=(380,250,410,280));weak.confidence=.1
        result=self.infer([weak])
        self.assertEqual((380.,250.,30.,30.),self.node._roi_search_hint)
        self.assertEqual(-1,result[0].track_id)
        self.assertFalse(result[0].track_id_is_stable)
        self.assertEqual(.5,self.node._fusion.last_detection_stamp)
        self.assertEqual('lost',self.node._fusion.result.source)
        self.infer([])
        stages=[call for call in self.node._record_stage.call_args_list if call.args[0]=='focused_roi_inference']
        self.assertTrue(stages[-1].kwargs['search_hint_used'])
        self.assertEqual([267,137],stages[-1].kwargs['offset'])
        self.assertIsNone(self.node._roi_search_hint)
        self.infer([])
        self.assertEqual((600,800),self.backend.detect.call_args.args[0].shape[:2])

    def setUp(self):
        self.node = NODE.SmartTrackerNode.__new__(NODE.SmartTrackerNode)
        self.backend = SimpleNamespace(detect=Mock(), detect_and_track=Mock())
        self.node._tracker = SimpleNamespace(
            backend=self.backend, conf_threshold=.1, iou_threshold=.6, max_det=30)
        self.node._fusion = SimpleNamespace(margin=.12, config={}, min_detector_conf=.45,
                                           result=SimpleNamespace(source="visual"))
        self.node._record_stage = Mock()
        self.node._epoch = 0
        self.node._tracking_mode = "smart_tracker"
        self.frame = np.zeros((100, 100, 3), np.uint8)
        self.selection = (7, (20, 20, 30, 30), 0, ((0, 1), (2,)),
                          np.ones(1), .7, .35, 7)

    def test_search_hint_keeps_three_fresh_strong_confirmations_without_claiming_lock(self):
        self.node._tracking_mode='fusion_tracker'
        self.node._fusion.result.source='lost'
        self.node._fusion.last_detection_stamp=.5
        self.frame=np.zeros((600,800,3),np.uint8)
        self.selection=(7,(300,250,30,30),0,((0,1),(2,)),np.ones(1),.7,.35,7)
        self.infer([])
        self.node._roi_misses=3
        weak=self.detection(box=(380,250,410,280));weak.confidence=.1
        self.infer([weak])
        for _ in range(3):
            result=self.infer([self.detection(box=(113,113,143,143))])
            self.assertEqual((380.,250.,30.,30.),self.node._roi_search_hint)
            self.assertEqual(-1,result[0].track_id)
            self.assertEqual(.5,self.node._fusion.last_detection_stamp)
            self.assertEqual('lost',self.node._fusion.result.source)
        self.infer([self.detection(box=(113,113,143,143))])
        self.assertIsNone(self.node._roi_search_hint)

    def infer(self, proposals):
        self.backend.detect.return_value = ('detect', proposals)
        with patch.object(NODE, 'appearance', return_value=(np.ones(1), .9)):
            result = self.node._infer(self.frame, 1., self.selection)
        self.backend.detect_and_track.assert_not_called()
        return result

    def detection(self, class_id=0, box=(20, 20, 50, 50)):
        return NormalizedDetection(-1, class_id, .9, box, (35, 35),
                                   track_id_is_stable=False)

    def test_fusion_returns_raw_candidates_for_boxfusion_identity_gates(self):
        self.node._tracking_mode = "fusion_tracker"
        proposal = self.detection()
        result = self.infer([proposal])
        self.assertEqual(-1, result[0].track_id)
        self.assertFalse(result[0].track_id_is_stable)
        self.assertEqual(7, result[0].selected_public_id)

    def test_three_consecutive_misses_trigger_full_frame_search(self):
        self.frame = np.zeros((600, 800, 3), np.uint8)
        for _ in range(4):
            self.infer([])
        shapes = [call.args[0].shape[:2] for call in self.backend.detect.call_args_list]
        self.assertEqual([(256, 256)] * 3 + [(600, 800)], shapes)

    def test_new_selection_resets_crop_miss_counter(self):
        for _ in range(3):
            self.infer([])
        self.node._epoch += 1
        self.frame = np.zeros((600, 800, 3), np.uint8)
        self.infer([])
        self.assertEqual((256, 256), self.backend.detect.call_args.args[0].shape[:2])

    def test_one_compatible_proposal_maintains_existing_identity(self):
        proposal = self.detection()
        result = self.infer([proposal])
        self.assertEqual([7], [d.track_id for d in result])
        self.assertEqual(-1, proposal.track_id)

    def test_cross_family_proposal_rejected(self):
        self.assertEqual((), self.infer([self.detection(class_id=2)]))

    def test_nonoverlapping_proposal_rejected(self):
        self.assertEqual((), self.infer([self.detection(box=(60, 60, 90, 90))]))

    def test_ambiguous_proposals_rejected(self):
        self.assertEqual((), self.infer([self.detection(), self.detection(class_id=1)]))
