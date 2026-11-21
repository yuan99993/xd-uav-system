"""Exercise frame-loop ordering after a lost fallback without changing commit locks."""
import threading, unittest
from types import SimpleNamespace
from unittest.mock import Mock
import numpy as np
import rospy
from sar_yolo_detector.async_pipeline import LatestMailbox, InferencePacket
from sar_yolo_detector.box_fusion import BoxFusion, Candidate
from test_box_fusion import LocalTracker
from test_fusion_adapter import NODE

class LostDetectorDispatchTest(unittest.TestCase):
    def setup_node(self, result_epoch=3, stamp=10.3):
        rospy.rostime.set_rostime_initialized(True)
        node=NODE.SmartTrackerNode.__new__(NODE.SmartTrackerNode)
        frame=np.full((40,60,3),70,np.uint8)
        def message(t):return SimpleNamespace(header=SimpleNamespace(stamp=rospy.Time.from_sec(t),seq=1,frame_id='camera'),data=b'raw',height=40,width=60)
        image=message(stamp);original=message(10.28)
        packet=InferencePacket(original,frame,(),result_epoch,25.,'',99) if result_epoch is not None else None
        mailbox=SimpleNamespace(closed=False)
        class Frames:
            def get(self,timeout=0):mailbox.closed=True;return image,frame
        class Results:
            def __init__(self):self.waits=[]
            def get(self,timeout=0):
                self.waits.append(timeout)
                return packet
        node._image_mailbox=mailbox;node._decoded_mailbox=Frames();node._result_mailbox=Results()
        node._lock=threading.RLock();node._maximum_image_bytes=1000;node._epoch=3;node._model_busy=False
        node._tracking_mode='fusion_tracker';node._high_frequency_pixels=True
        node._capture_clock=SimpleNamespace(observe=lambda *a:SimpleNamespace(accepted=True,new_segment=False))
        node._detector_schedule=None;node._inference_mailbox=LatestMailbox()
        node._tracker=SimpleNamespace(backend=SimpleNamespace(backend_name='dfine'))
        node._fusion=BoxFusion(dict(strong_takeover_safety_enabled=True,detector_primary_processing=True,visual_fallback_gap_sec=.25),tracker=LocalTracker())
        node._fusion.initialize(frame,(10,10,15,15),10.,99,Candidate((10,10,15,15),0,.9,5,True,10.))
        node._last_capture_stamp=rospy.Time.from_sec(10.)
        node._process_image=Mock();node._record_stage=Mock()
        return node,packet,image

    def mark_lost(self, node):
        node._last_capture_stamp=rospy.Time.from_sec(10.29)
        node._fusion.stamp=10.29
        node._fusion._result(None,'lost','no_trusted_visual_observation',0.)

    def test_lost_old_packet_cannot_advance_the_clock(self):
        node,packet,image=self.setup_node();self.mark_lost(node);node._frame_loop()
        node._process_image.assert_not_called()
        self.assertEqual(rospy.Time.from_sec(10.29),node._last_capture_stamp)
        self.assertIn('stale_detector_after_loss_discarded',[c.args[0] for c in node._record_stage.call_args_list])

    def test_lost_without_detector_does_not_repeat_advancing_fallback(self):
        node,_,_=self.setup_node(None);self.mark_lost(node);node._frame_loop()
        node._process_image.assert_not_called()

    def test_original_newer_than_lost_is_processed_at_original_capture(self):
        node,packet,image=self.setup_node();self.mark_lost(node)
        node._last_capture_stamp=rospy.Time.from_sec(10.27);node._fusion.stamp=10.27
        node._frame_loop();call=node._process_image.call_args
        self.assertIs(packet.image,call.args[0]);self.assertIs(packet,call.kwargs['packet'])
        self.assertEqual(3,call.kwargs['expected_epoch'])

    def test_original_same_capture_can_correct_loss_without_visual_transport(self):
        node,packet,image=self.setup_node();self.mark_lost(node)
        node._last_capture_stamp=packet.image.header.stamp;node._fusion.stamp=10.28
        node._frame_loop();call=node._process_image.call_args
        self.assertIs(packet.image,call.args[0]);self.assertTrue(call.kwargs['result_only'])

    def test_old_epoch_cannot_restore_loss(self):
        node,_,_=self.setup_node(2);self.mark_lost(node);node._frame_loop()
        node._process_image.assert_not_called()

    def test_healthy_original_path_unchanged(self):
        node,packet,image=self.setup_node();node._frame_loop()
        self.assertIs(packet.image,node._process_image.call_args.args[0])
        self.assertEqual([0],node._result_mailbox.waits)

    def test_unobserved_prediction_also_discards_old_packet(self):
        node,_,_=self.setup_node();self.mark_lost(node)
        node._fusion._result(node._fusion.box,'prediction','no_trusted_visual_observation',0.)
        self.assertFalse(node._fusion.result.observed)
        node._frame_loop();node._process_image.assert_not_called()

    def test_unobserved_prediction_does_not_advance_without_packet(self):
        node,_,_=self.setup_node(None);self.mark_lost(node)
        node._fusion._result(node._fusion.box,'prediction','no_trusted_visual_observation',0.)
        node._frame_loop();node._process_image.assert_not_called()
