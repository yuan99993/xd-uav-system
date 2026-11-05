"""No server/model startup: bounded web command validation and frame binding."""
import importlib.util
import base64
from collections import OrderedDict
from pathlib import Path
import threading
from types import SimpleNamespace
import unittest

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from std_msgs.msg import Header
from sar_yolo_detector.interactive_view import FramePair
from sar_yolo_detector.msg import SmartTrackerState

SPEC = importlib.util.spec_from_file_location('fusion_web', str(Path(__file__).resolve().parents[1]/'scripts/fusion_web_preview.py'))
WEB = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(WEB)


class Proxy:
    def __init__(self):
        self.requests = []

    def wait_for_service(self, timeout):
        pass

    def __call__(self, request):
        self.requests.append(request)
        return SimpleNamespace(success=True, selected_target_id=7, message='accepted')


class WebCommandTest(unittest.TestCase):
    def setUp(self):
        rospy.rostime.set_rostime_initialized(True)
        self.preview = WEB.Preview.__new__(WEB.Preview)
        self.preview.lock = threading.Lock()
        self.preview.commands = threading.Lock()
        now = rospy.Time.now()
        self.header = dict(seq=123, frame_id='fixed_camera', stamp=dict(secs=now.secs, nsecs=now.nsecs))
        self.preview.history = {'session:1': (self.header, 640, 480)}
        self.preview.max_age = .75
        self.preview.roi = Proxy()
        self.preview.manual_select = Proxy()
        self.preview.active_mode = 'fusion_tracker'
        self.preview.mode_epoch = 0
        self.preview.flight = {}
        self.messages = []
        self.preview.click = SimpleNamespace(get_num_connections=lambda: 1, publish=self.messages.append)

    def test_click_binds_served_frame(self):
        status, result = self.preview.command(dict(action='click', frame_token='session:1', point=[40, 50]))
        self.assertEqual(status, 202)
        self.assertTrue(result['sent'])
        self.assertEqual(self.messages[0].header.frame_id, 'fixed_camera')
        self.assertEqual(self.messages[0].header.seq, 123)

    def test_raw_and_stable_class_join_only_the_same_capture(self):
        from std_msgs.msg import String
        self.preview.labels=['car','tank','drone']
        self.preview.classification=OrderedDict()
        key=self.header['stamp']['secs']*1000000000+self.header['stamp']['nsecs']
        self.preview.targets=OrderedDict({key:[dict(id=5,name='car',class_id=0)]})
        self.preview.current=None
        import json
        payload=dict(schema_version=1,stamp=self.header['stamp'],objects=[dict(id=5,raw_class_id=1,
                     stable_class_id=0,label_source='temporal_consensus')])
        self.preview.classification_evidence(String(data=json.dumps(payload)))
        self.assertEqual(self.preview.targets[key][0]['raw_name'],'tank')
        self.assertEqual(self.preview.targets[key][0]['name'],'car')
        other=self.preview._attach_classification(key+1,[dict(id=5,name='car',class_id=0)])
        self.assertNotIn('raw_name',other[0])

    def test_fusion_click_on_a_displayed_box_gets_synchronous_selection_result(self):
        stamp = rospy.Time(self.header['stamp']['secs'], self.header['stamp']['nsecs']).to_nsec()
        self.preview.targets = {stamp: [dict(id=5, kind='detector_track',
                                           box=[30.,40.,40.,40.], confidence=.9)]}
        status, result = self.preview.command(dict(action='click',frame_token='session:1',point=[50,60]))
        self.assertEqual(status,200)
        self.assertTrue(result['success'])
        self.assertEqual(self.preview.roi.requests[0].header.stamp.to_nsec(),stamp)
        self.assertEqual(self.preview.roi.requests[0].roi.width,40)
        self.assertFalse(self.messages)

    def test_roi_and_clear_use_existing_service(self):
        status, result = self.preview.command(dict(action='roi', frame_token='session:1', roi=[40, 50, 20, 30]))
        self.assertEqual(status, 200)
        self.assertEqual(self.preview.roi.requests[0].roi.width, 20)
        self.assertEqual(self.preview.command(dict(action='clear'))[0], 200)
        self.assertTrue(self.preview.roi.requests[-1].clear_selection)

    def test_manual_roi_binds_the_displayed_frame(self):
        self.preview.active_mode = 'feature_tracker'
        status, result = self.preview.command(dict(action='roi', frame_token='session:1', roi=[40, 50, 20, 30]))
        self.assertEqual(status, 200)
        self.assertTrue(result['success'])
        request = self.preview.manual_select.requests[0]
        self.assertEqual(request.header.frame_id, 'fixed_camera')
        self.assertEqual(request.header.stamp.secs, self.header['stamp']['secs'])
        self.assertEqual(request.roi.width, 20)

    def test_frame_evidence_distinguishes_observed_and_predicted_lock(self):
        self.preview.frames = FramePair()
        state = SmartTrackerState()
        state.header.frame_id = self.header['frame_id']
        state.header.stamp = rospy.Time(self.header['stamp']['secs'], self.header['stamp']['nsecs'])
        state.has_selection = True
        state.tracking_active = True
        state.prediction_only = True
        state.measurement_current = False
        state.control_measurement_ready = False
        self.preview.state(state)
        evidence = next(iter(self.preview.frames.metadata.values()))
        self.assertTrue(evidence['tracking_active'])
        self.assertFalse(evidence['measurement_current'])
        self.assertFalse(evidence['control_target_valid'])

    def test_old_session_token_rejected(self):
        self.assertEqual(self.preview.command(dict(action='click', frame_token='old_session:1', point=[40, 50]))[0], 409)
        self.assertFalse(self.messages)

    def test_out_of_bounds_and_nonfinite_rejected(self):
        for point in ([-1, 30], [640, 30], [30, float('nan')], [True, 30]):
            self.assertEqual(self.preview.command(dict(action='click', frame_token='session:1', point=point))[0], 409)
        self.assertFalse(self.messages)

    def test_bad_roi_rejected(self):
        for box in ([10, 10, 2, 20], [630, 10, 20, 20], [10, 10, 20, -5]):
            self.assertEqual(self.preview.command(dict(action='roi', frame_token='session:1', roi=box))[0], 409)
        self.assertFalse(self.preview.roi.requests)

    def test_stale_rejected_without_latest_fallback(self):
        self.header['stamp'] = dict(secs=1, nsecs=0)
        self.assertEqual(self.preview.command(dict(action='click', frame_token='session:1', point=[40, 50]))[0], 409)
        self.assertFalse(self.messages)

    def test_busy_does_not_queue_another_command(self):
        self.preview.commands.acquire()
        try:
            status, result = self.preview.command(dict(action='clear'))
            self.assertEqual(status, 409)
            self.assertIn('BUSY', result['message'])
            self.assertFalse(self.preview.roi.requests)
        finally:
            self.preview.commands.release()

    def test_png_roundtrip_preserves_bgr_and_rgb_pixels(self):
        # Odd dimensions and fine colour texture expose resampling/codec loss.
        expected = np.random.RandomState(19).randint(0, 256, (47, 61, 3)).astype(np.uint8)
        for encoding in ('bgr8', 'rgb8'):
            preview = self.preview
            preview.frames = FramePair()
            preview.bridge = CvBridge()
            preview.targets = OrderedDict()
            preview.history = OrderedDict()
            preview.segment = None
            preview.session = 'lossless-test'
            preview.serial = 0
            preview.current = None
            source = expected if encoding == 'bgr8' else expected[:, :, ::-1].copy()
            image = preview.bridge.cv2_to_imgmsg(source, encoding=encoding)
            image.header = Header(seq=self.header['seq'], frame_id=self.header['frame_id'],
                                  stamp=rospy.Time(self.header['stamp']['secs'], self.header['stamp']['nsecs']))
            preview.frames.add_image(image)
            preview.frames.add_evidence(dict(capture_header=self.header, selected_id=-1))
            preview.encode(None)
            packet = preview.snapshot()
            raw = base64.b64decode(packet['png'])
            self.assertEqual(raw[:8], b'\x89PNG\r\n\x1a\n')
            decoded = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_COLOR)
            np.testing.assert_array_equal(decoded, expected)
            self.assertEqual((packet['width'], packet['height']), (61, 47))
            self.assertEqual(packet['image_mime'], 'image/png')
            self.assertNotIn('jpeg', packet)
            self.assertEqual(packet['capture_header'], self.header)

    def test_page_defaults_to_native_size_and_keeps_coordinate_mapping(self):
        html = (Path(__file__).resolve().parents[1]/'web/dist/index.html').read_text()
        self.assertIn('fit.checked=false;', html)
        self.assertIn('width:var(--native-width,640px)', html)
        self.assertIn('.video-viewport.fit canvas{width:100%}', html)
        self.assertIn('ctx.drawImage(bitmap,0,0)', html)
        self.assertIn('canvas.width/r.width', html)
        self.assertIn('canvas.height/r.height', html)
        self.assertIn("'data:image/png;base64,'+p.png", html)


if __name__ == '__main__':
    unittest.main()
