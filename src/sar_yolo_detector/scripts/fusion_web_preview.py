#!/usr/bin/env python3
"""Local-only, bounded lossless PNG preview and existing fusion interfaces."""
import base64
import copy
import json
import math
from collections import OrderedDict
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import threading
import time
import uuid
from urllib.parse import urlparse

import cv2
from cv_bridge import CvBridge
import rospkg
import rospy
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Image, RegionOfInterest
from std_msgs.msg import String
from sar_yolo_detector.interactive_view import FramePair
from sar_yolo_detector.msg import TrackedDetection2DArray
from sar_yolo_detector.srv import (
    SelectFusionROI,
    SelectFusionROIRequest,
    SelectFusionPoint,
    SelectFusionPointRequest,
)


class Preview:
    def __init__(self):
        self.frames = FramePair()
        self.lock = threading.Lock()
        self.commands = threading.Lock()
        self.targets = OrderedDict()
        self.history = OrderedDict()
        self.current = None
        self.serial = 0
        self.session = uuid.uuid4().hex
        self.segment = None
        self.bridge = CvBridge()
        self.max_age = float(rospy.get_param('~selection_max_age_sec', .75))
        self.labels = rospy.get_param('~class_names', ['car', 'ar-car', 'tank', 'm142'])
        self.click = rospy.Publisher(rospy.get_param('~select_pixel_topic'), PointStamped, queue_size=1)
        self.manual_roi = rospy.Publisher(rospy.get_param('~manual_roi_topic'), RegionOfInterest, queue_size=1)
        self.roi = rospy.ServiceProxy(rospy.get_param('~select_roi_service'), SelectFusionROI)
        self.manual_seed_roi = rospy.ServiceProxy(
            rospy.get_param('~select_manual_roi_service'), SelectFusionROI)
        self.point_selection = rospy.ServiceProxy(
            rospy.get_param('~select_point_service'), SelectFusionPoint)
        self.subscribers = [
            rospy.Subscriber(rospy.get_param('~image_topic'), Image, self.image, queue_size=1, buff_size=4*1024*1024),
            rospy.Subscriber(rospy.get_param('~evidence_topic'), String, self.evidence, queue_size=1),
            rospy.Subscriber(rospy.get_param('~targets_topic'), TrackedDetection2DArray, self.objects, queue_size=1)]
        self.timer = rospy.Timer(rospy.Duration(1./max(1., min(10., float(rospy.get_param('~fps', 5.))))), self.encode)

    def image(self, message):
        if len(message.data) <= 16*1024*1024:
            self.frames.add_image(message)

    def evidence(self, message):
        try:
            if len(message.data) <= 65536:
                self.frames.add_evidence(json.loads(message.data))
        except (KeyError, TypeError, ValueError):
            rospy.logwarn_throttle(5., 'Invalid fusion web metadata')

    def objects(self, message):
        if len(message.detections) > 256:
            return
        objects = []
        for item in message.detections:
            d = item.detection
            category = d.results[0].id if d.results else -1
            b = d.bbox
            if not all(math.isfinite(v) for v in (b.center.x, b.center.y, b.size_x, b.size_y)) or min(b.size_x, b.size_y) <= 0:
                continue
            objects.append(dict(id=item.track_id, class_id=category,
                name=self.labels[category] if 0 <= category < len(self.labels) else 'unknown',
                center=[b.center.x, b.center.y], box=[b.center.x-b.size_x/2, b.center.y-b.size_y/2, b.size_x, b.size_y]))
        with self.lock:
            self.targets[message.header.stamp.to_nsec()] = objects
            while len(self.targets) > 32:
                self.targets.popitem(last=False)

    def encode(self, event):
        pair = self.frames.take()
        if pair is None:
            return
        image, evidence = pair
        try:
            frame = self.bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')
            # Level 1 limits CPU work; PNG remains lossless at every level.
            ok, png = cv2.imencode('.png', frame, [cv2.IMWRITE_PNG_COMPRESSION, 1])
            if not ok:
                return
            with self.lock:
                segment = (evidence.get('pipeline_epoch'), evidence.get('capture_segment'))
                if self.segment is not None and segment != self.segment:
                    self.history.clear()
                    self.targets.clear()
                self.segment = segment
                self.serial += 1
                token = '{}:{}'.format(self.session, self.serial)
                self.current = dict(frame_token=token, capture_header=evidence['capture_header'],
                    width=image.width, height=image.height,
                    png=base64.b64encode(png).decode('ascii'), image_mime='image/png',
                    selected_id=evidence.get('selected_id', -1), source=evidence.get('source'),
                    reason=evidence.get('reason'), mode=evidence.get('mode'),
                    color_source=evidence.get('color_source'), color_name=evidence.get('color_name'),
                    objects=self.targets.get(image.header.stamp.to_nsec(), []),
                    stamp=image.header.stamp.to_sec(), received_wall=time.monotonic())
                self.history[token] = (copy.deepcopy(evidence['capture_header']), image.width,
                    image.height, evidence.get('mode', 'fusion_tracker'))
                while len(self.history) > 24:
                    self.history.popitem(last=False)
        except Exception as exc:
            rospy.logwarn_throttle(5., 'Web PNG conversion failed: %s', exc)

    def snapshot(self):
        with self.lock:
            if self.current is None:
                return dict(ready=False)
            output = dict(self.current)
        output['ready'] = True
        output['video_stale'] = time.monotonic()-output.pop('received_wall') > 2.
        output['age_ms'] = max(0., (rospy.Time.now().to_sec()-output['stamp'])*1000.)
        return output

    def command(self, body):
        if not self.commands.acquire(blocking=False):
            return 409, dict(success=False, message='BUSY: previous command is still running')
        try:
            action = body.get('action')
            if action not in ('click', 'roi', 'manual_roi', 'clear'):
                raise ValueError('Unknown command')
            with self.lock:
                current_mode = self.current.get('mode', 'fusion_tracker') if self.current else None
            request = SelectFusionROIRequest(clear_selection=action == 'clear')
            if action != 'clear':
                with self.lock:
                    original = self.history.get(str(body.get('frame_token')))
                if original is None:
                    raise ValueError('Displayed frame is no longer cached; refresh and retry')
                h, width, height, frame_mode = original
                if frame_mode != current_mode:
                    raise ValueError('Tracking mode changed; select on a new frame')
                request.header.seq = h['seq']
                request.header.frame_id = h['frame_id']
                request.header.stamp = rospy.Time(h['stamp']['secs'], h['stamp']['nsecs'])
                age = (rospy.Time.now()-request.header.stamp).to_sec()
                if age > self.max_age or age < -.02:
                    raise ValueError('Displayed frame is stale; use a quick click/drag on live video')
                values = body.get('point') if action == 'click' else body.get('roi')
                if not isinstance(values, list) or len(values) != (2 if action == 'click' else 4):
                    raise ValueError('Invalid pixel coordinates')
                if not all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in values):
                    raise ValueError('Non-finite coordinates')
                x, y = values[:2]
                if not 0 <= x < width or not 0 <= y < height:
                    raise ValueError('Point outside original image')
                if action == 'click':
                    if frame_mode == 'fusion_tracker':
                        point_request = SelectFusionPointRequest(header=request.header)
                        point_request.point.x, point_request.point.y = x, y
                        self.point_selection.wait_for_service(timeout=2.)
                        result = self.point_selection(point_request)
                        return (200 if result.success else 409), dict(
                            success=result.success,
                            selected_id=result.selected_target_id,
                            message=result.message,
                        )
                    if not self.click.get_num_connections():
                        raise ValueError('Click subscriber is not connected')
                    message = PointStamped(header=request.header)
                    message.point.x, message.point.y = x, y
                    self.click.publish(message)
                    return 202, dict(success=True, sent=True,
                        message='Click sent; verify the selected colour box' if frame_mode == 'color_tracker'
                        else 'Click sent; verify actual selected ID')
                w, h = values[2:]
                if w < 4 or h < 4 or x+w > width or y+h > height:
                    raise ValueError('ROI must be inside the image and at least 4x4 pixels')
                if frame_mode in ('feature_tracker', 'color_tracker'):
                    if not self.manual_roi.get_num_connections():
                        raise ValueError('Manual ROI subscriber is not connected')
                    self.manual_roi.publish(RegionOfInterest(x_offset=int(x), y_offset=int(y),
                        width=int(w), height=int(h)))
                    return 202, dict(success=True, sent=True,
                        message='Selected colour region queued for the next frame')
                if frame_mode != 'fusion_tracker':
                    raise ValueError('Selection is unavailable in this mode')
                request.roi.x_offset, request.roi.y_offset = int(x), int(y)
                request.roi.width, request.roi.height = int(w), int(h)
            elif current_mode in ('feature_tracker', 'color_tracker'):
                if not self.manual_roi.get_num_connections():
                    raise ValueError('Manual ROI subscriber is not connected')
                self.manual_roi.publish(RegionOfInterest())
                return 202, dict(success=True, sent=True, message='Manual target cleared')
            elif current_mode != 'fusion_tracker':
                raise ValueError('Selection is unavailable in this mode')
            service = self.manual_seed_roi if action == 'manual_roi' else self.roi
            service.wait_for_service(timeout=2.)
            result = service(request)
            return (200 if result.success else 409), dict(success=result.success,
                selected_id=result.selected_target_id, message=result.message)
        except (ValueError, TypeError, KeyError, rospy.ROSException, rospy.ServiceException) as exc:
            return 409, dict(success=False, message=str(exc))
        finally:
            self.commands.release()


class Server(ThreadingHTTPServer):
    daemon_threads = True
    request_queue_size = 8

    def __init__(self, address, handler):
        self.slots = threading.BoundedSemaphore(8)
        super().__init__(address, handler)

    def process_request(self, request, address):
        if self.slots.acquire(blocking=False):
            super().process_request(request, address)
        else:
            self.shutdown_request(request)

    def process_request_thread(self, request, address):
        try:
            request.settimeout(5.)
            super().process_request_thread(request, address)
        finally:
            self.slots.release()


def main():
    rospy.init_node('fusion_web_preview')
    cv2.setNumThreads(1)
    preview = Preview()
    html = (Path(rospkg.RosPack().get_path('sar_yolo_detector'))/'web/dist/index.html').read_bytes()

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def reply(self, status, data, content_type='application/json'):
            encoded = data if isinstance(data, bytes) else json.dumps(data, allow_nan=False).encode()
            self.send_response(status)
            self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', str(len(encoded)))
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.end_headers()
            try:
                self.wfile.write(encoded)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def do_GET(self):
            if self.path == '/':
                self.reply(200, html, 'text/html; charset=utf-8')
            elif self.path == '/api/frame':
                self.reply(200, preview.snapshot())
            else:
                self.reply(404, dict(message='Not found'))

        def do_POST(self):
            try:
                origin = self.headers.get('Origin')
                if origin and urlparse(origin).netloc != self.headers.get('Host'):
                    raise ValueError('Cross-origin command rejected')
                if self.path != '/api/command' or self.headers.get('Content-Type', '').split(';')[0] != 'application/json':
                    raise ValueError('JSON command required')
                length = int(self.headers.get('Content-Length', '0'))
                if not 0 < length <= 16384:
                    raise ValueError('Invalid command length')
                body = json.loads(self.rfile.read(length))
                if not isinstance(body, dict):
                    raise ValueError('JSON object required')
                status, result = preview.command(body)
                self.reply(status, result)
            except (ValueError, TypeError) as exc:
                self.reply(400, dict(success=False, message=str(exc)))

    host, port = rospy.get_param('~bind_address', '127.0.0.1'), int(rospy.get_param('~port', 8787))
    server = Server((host, port), Handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    rospy.loginfo('Fusion web preview: http://%s:%d (no second inference model)', host, port)
    try:
        rospy.spin()
    finally:
        preview.timer.shutdown()
        server.shutdown()
        server.server_close()


if __name__ == '__main__':
    main()
