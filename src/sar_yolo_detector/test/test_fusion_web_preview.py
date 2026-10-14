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
        self.messages = []
        self.preview.click = SimpleNamespace(get_num_connections=lambda: 1, publish=self.messages.append)

    def test_click_binds_served_frame(self):
        status, result = self.preview.command(dict(action='click', frame_token='session:1', point=[40, 50]))
        self.assertEqual(status, 202)
        self.assertTrue(result['sent'])
        self.assertEqual(self.messages[0].header.frame_id, 'fixed_camera')
        self.assertEqual(self.messages[0].header.seq, 123)

    def test_roi_and_clear_use_existing_service(self):
        status, result = self.preview.command(dict(action='roi', frame_token='session:1', roi=[40, 50, 20, 30]))
        self.assertEqual(status, 200)
        self.assertEqual(self.preview.roi.requests[0].roi.width, 20)
        self.assertEqual(self.preview.command(dict(action='clear'))[0], 200)
        self.assertTrue(self.preview.roi.requests[-1].clear_selection)

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
