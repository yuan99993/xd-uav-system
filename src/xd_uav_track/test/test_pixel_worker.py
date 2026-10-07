"""Real batched LK, bounded asynchronous mailboxes and evidence semantics."""
import threading
import time
import unittest
from types import SimpleNamespace
from copy import deepcopy
from unittest.mock import patch
import cv2
import numpy as np
import rospy
from std_msgs.msg import Header
from xd_uav_track.msg import PerceptionTarget, DetectionArray, DetectionCandidate
from xd_uav_track.pixel_worker import LatestPixelWorker
from xd_uav_track.pixel_lock import flow_boxes


def image(shift=0):
    frame = np.zeros((160, 240, 3), np.uint8)
    patch = np.random.RandomState(7).randint(0, 255, (36, 40, 3), dtype=np.uint8)
    frame[40:76, 30+shift:70+shift] = patch
    frame[90:126, 140+shift:180+shift] = patch
    return frame


def target(identity, box):
    item = PerceptionTarget(track_id=identity, target_uid='session:'+str(identity),
        lifecycle_state='confirmed', measurement_source='detector', class_id=2,
        bbox_xyxy=list(box), last_detection_stamp=rospy.Time.from_sec(1.))
    return item


class PixelWorkerTests(unittest.TestCase):
    def test_seed_copies_only_retained_targets_and_keeps_priority(self):
        worker=LatestPixelWorker(['fixed'],lambda *args:None,maximum_tracks=2)
        try:
            targets=[target(i,(30,40,70,76)) for i in range(100)]
            for i,item in enumerate(targets): item.tracking_quality=i/100.
            targets[1].selected=True
            with patch('xd_uav_track.pixel_worker.deepcopy',wraps=deepcopy) as copy:
                worker.seed('fixed',Header(stamp=rospy.Time(1)),image(),targets,1)
                copied=[call.args[0] for call in copy.call_args_list if isinstance(call.args[0],PerceptionTarget)]
            self.assertEqual(len(copied),2)
            kept=worker.anchors['fixed'][2]
            self.assertEqual([item.track_id for item in kept],[1,99])
            targets[1].bbox_xyxy[0]=123
            self.assertEqual(kept[0].bbox_xyxy[0],30)
        finally: worker.close()
        worker.seed('fixed',Header(stamp=rospy.Time(1)),image(),targets,1)
        self.assertFalse(worker.anchors)

    def test_diagnostic_snapshot_is_bounded_during_updates(self):
        worker=LatestPixelWorker(['fixed'],lambda *args:None)
        try:
            for seq in range(160):
                worker.submit('fixed',Header(stamp=rospy.Time.from_sec(1.+seq*.1)),image(),1)
                snapshot=worker.snapshot()
                self.assertLessEqual(snapshot['pending_sources'],1)
                self.assertGreaterEqual(snapshot['processing_p95_ms'],0.)
            self.assertLessEqual(len(worker.processing_ms),128)
        finally: worker.close()

    def test_seed_uses_associated_measurement_not_smoothed_box(self):
        from test_perception_modes import manager_fixture
        manager=manager_fixture()
        manager._pixel_anchors={}
        submitted=[]
        manager._pixel_worker=SimpleNamespace(seed=lambda *args:submitted.append(args))
        h=Header(stamp=rospy.Time.from_sec(1.))
        manager._frames['fixed'].append((1,h.stamp,image()))
        manager._latest_headers['fixed']=h
        manager._latest_targets['fixed']=[target(7,(0,0,100,100))]
        candidate=DetectionCandidate(track_id=7,has_bbox=True,bbox=[30,40,70,76],
            measurement_source='detector')
        message=DetectionArray(header=h,image_source='fixed',image_width=240,
            image_height=160,model_epoch=1,candidates=[candidate])
        manager._pixel_anchor_callback(message)
        self.assertEqual(submitted[0][3][0].bbox_xyxy,[30,40,70,76])
        self.assertEqual(submitted[0][3][0].track_id,7)

    def test_scale_jump_requests_recovery(self):
        previous=cv2.cvtColor(image(),cv2.COLOR_BGR2GRAY)
        matrix=cv2.getRotationMatrix2D((50,58),0,1.4)
        current=cv2.warpAffine(previous,matrix,(240,160))
        rejected={}
        points=np.array([[[x,y]] for y in (5.,15.,25.) for x in (5.,15.,25.)],np.float32)
        original=[]
        def flow(first,second,points,guess,**kwargs):
            if not original:
                original.append(points.copy())
                moved=(points-np.array([50.,58.],np.float32))*1.4+np.array([50.,58.],np.float32)
            else:
                moved=original[0].copy()
            return moved,np.ones((len(points),1),np.uint8),None
        with patch('cv2.goodFeaturesToTrack',return_value=points), \
                patch('cv2.calcOpticalFlowPyrLK',side_effect=flow):
            result=flow_boxes(previous,current,{7:(30,40,70,76)},.1,rejections=rejected)
        self.assertNotIn(7,result)
        self.assertEqual(rejected.get(7),'scale_anomaly')

    def test_prediction_expires_and_never_becomes_observation(self):
        worker=LatestPixelWorker(['fixed'],lambda *args:None)
        try:
            h=Header(stamp=rospy.Time.from_sec(1.))
            worker._advance('fixed',h,image(),1,(h,image(),[target(7,(30,40,70,76))],1,0))
            worker._advance('fixed',Header(stamp=rospy.Time.from_sec(1.1)),image(4),1,None)
            with patch('xd_uav_track.pixel_worker.flow_boxes',return_value={}), \
                    patch('xd_uav_track.pixel_worker.color_box',return_value=None):
                result=worker._advance('fixed',Header(stamp=rospy.Time.from_sec(1.2)),image(),1,None)[0]
                self.assertEqual(result.measurement_source,'prediction')
                self.assertEqual(result.last_detection_stamp.to_sec(),1.)
                self.assertFalse(result.position_valid)
                result=worker._advance('fixed',Header(stamp=rospy.Time.from_sec(1.7)),image(),1,None)[0]
                self.assertEqual(result.measurement_source,'stale')
                self.assertEqual(result.bbox_xyxy,[0,0,0,0])
        finally:
            worker.close()

    def test_fusion_complete_pixel_scene_and_delayed_detector(self):
        from test_perception_modes import manager_fixture
        manager = manager_fixture()
        manager._closing = False
        manager._pixel_targets = {}
        manager._output_stamps = {}
        manager._pixel_worker = SimpleNamespace(generation=0)
        h = Header(seq=2,stamp=rospy.Time.from_sec(1.1))
        manager._frames['fixed'].append((2,h.stamp,image(4)))
        first, second = target(7,(34,40,74,76)), target(8,(144,90,184,126))
        for item in (first,second):
            item.measurement_source = 'visual_tracker'
        manager._pixel_callback('fixed',h,240,160,[first,second],1,0)
        self.assertEqual(len(manager._targets_pub.messages[-1].targets), 2)
        count = len(manager._targets_pub.messages)
        manager._publish_targets('fixed',Header(stamp=rospy.Time.from_sec(1.)),240,160)
        self.assertEqual(len(manager._targets_pub.messages), count)
        manager._pixel_callback('fixed',h,240,160,[first],1,99)
        self.assertEqual(len(manager._targets_pub.messages), count)

    def test_batched_two_targets(self):
        prev = cv2.cvtColor(image(), cv2.COLOR_BGR2GRAY)
        now = cv2.cvtColor(image(4), cv2.COLOR_BGR2GRAY)
        boxes = {1:(30,40,70,76), 2:(140,90,180,126)}
        observations = flow_boxes(prev, now, boxes, .1)
        self.assertEqual(set(observations), {1,2})
        for identity, (box, quality) in observations.items():
            self.assertAlmostEqual(box[0]-boxes[identity][0], 4, delta=1)
            self.assertGreater(quality, .5)

    def test_worker_does_not_refresh_detection_or_allocate_ids(self):
        outputs, ready = [], threading.Event()
        def publish(*args):
            outputs.append(deepcopy(args)); ready.set()
        worker = LatestPixelWorker(['fixed'], publish)
        try:
            header = Header(stamp=rospy.Time.from_sec(1.))
            worker.seed('fixed', header, image(), [target(7,(30,40,70,76))], 1)
            worker.submit('fixed', header, image(), 1)
            self.assertTrue(ready.wait(2)); ready.clear()
            worker.submit('fixed', Header(stamp=rospy.Time.from_sec(1.1)), image(4), 1)
            self.assertTrue(ready.wait(2))
            result = outputs[-1][4][0]
            self.assertEqual(result.track_id, 7)
            self.assertEqual(result.measurement_source, 'visual_tracker')
            self.assertEqual(result.last_detection_stamp.to_sec(), 1.)
            self.assertFalse(result.position_valid)
            self.assertAlmostEqual(result.bbox_xyxy[0], 34, delta=1)
        finally:
            worker.close()
        self.assertFalse(worker.thread.is_alive())

    def test_failed_flow_cannot_resurrect_old_box(self):
        worker = LatestPixelWorker(['fixed'], lambda *args:None)
        try:
            t = target(7,(30,40,70,76))
            h = Header(stamp=rospy.Time.from_sec(1.))
            anchor = (h,image(),[t],1,worker.generation)
            worker._advance('fixed', h,image(),1,anchor)
            output = worker._advance('fixed',Header(stamp=rospy.Time.from_sec(1.1)),
                np.zeros_like(image()),1,None)
            self.assertEqual(output[0].measurement_source, 'stale')
            self.assertFalse(worker.states['fixed']['tracks'])
        finally:
            worker.close()

    def test_mailbox_is_source_bounded(self):
        worker = LatestPixelWorker(['fixed','gimbal'], lambda *args:None)
        try:
            with worker.condition:
                for seq in range(100):
                    for source in ('fixed','gimbal','unknown'):
                        worker.submit(source, Header(seq=seq), image(),1)
                self.assertEqual(len(worker.pending), 2)
                self.assertEqual(worker.stats['received'], 200)
                self.assertEqual(worker.stats['dropped'], 198)
                worker.clear()
                self.assertFalse(worker.pending)
                self.assertFalse(worker.anchors)
                self.assertEqual(worker.generation, 1)
        finally:
            worker.close()


if __name__ == '__main__':
    unittest.main()
