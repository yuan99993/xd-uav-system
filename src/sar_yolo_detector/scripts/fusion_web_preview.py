#!/usr/bin/env python3
"""Local-only, bounded lossless PNG preview and existing fusion interfaces."""
import base64
import copy
import json
import io
import math
import os
import socket
from collections import OrderedDict
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import threading
import time
import uuid
from urllib.parse import urlparse,parse_qs

import numpy as np
import cv2
from cv_bridge import CvBridge
import rospkg
import rospy
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import CompressedImage, Image, RegionOfInterest
from std_msgs.msg import String
from sar_yolo_detector.interactive_view import FramePair
from sar_yolo_detector.msg import SmartTrackerState, TrackedDetection2DArray
from sar_yolo_detector.srv import (SelectFusionROI, SelectFusionROIRequest,
    SelectSmartTrack, SelectSmartTrackRequest, SwitchTrackingMode,
    SwitchTrackingModeRequest)
from sar_yolo_detector.web_console import WebConsoleMixin
from sar_yolo_detector.live_tuning import class_family


class Preview(WebConsoleMixin):
    def __init__(self):
        self.image_codec=str(rospy.get_param('~image_codec','png')).lower()
        if self.image_codec not in ('png','jpeg'):raise ValueError('image_codec must be png or jpeg')
        self.source_overlay=bool(rospy.get_param("~source_image_overlay",False))
        self.frames = FramePair(image_limit=16 if self.source_overlay else 1)
        self.lock = threading.Lock()
        self.commands = threading.Lock()
        self.targets = OrderedDict()
        self.classification = OrderedDict()
        self.history = OrderedDict()
        self.current = None
        self.serial = 0
        self.session = uuid.uuid4().hex
        self.segment = None
        self.active_mode = str(rospy.get_param('~initial_mode', 'smart_tracker'))
        self.mode_epoch = 0
        self.flight = {}
        self.bridge = CvBridge()
        self.max_age = float(rospy.get_param('~selection_max_age_sec', .75))
        raw_labels = rospy.get_param('~class_names', ['car', 'ar-car', 'tank', 'm142'])
        self.labels = raw_labels if isinstance(raw_labels, list) else [v.strip() for v in str(raw_labels).split(',')]
        self.click = rospy.Publisher(rospy.get_param('~select_pixel_topic'), PointStamped, queue_size=1)
        self.benchmark=(rospy.Publisher(rospy.get_param('~benchmark_topic','benchmark/command'),String,queue_size=1)
            if rospy.get_param('~enable_benchmark_controls',False) else None)
        self.manual_roi = rospy.Publisher(rospy.get_param('~manual_roi_topic'), RegionOfInterest, queue_size=1)
        roi_service=rospy.resolve_name(str(rospy.get_param('~select_roi_service')))
        tracker_namespace=roi_service.rsplit('/',1)[0]
        deadline=float(rospy.get_param(tracker_namespace+'/control_maximum_capture_age_sec',.5))
        if not math.isfinite(deadline):
            raise ValueError('Invalid tracker control capture deadline')
        self.control_max_age_ms=(deadline if deadline>0 else .5)*1000.
        self.roi = rospy.ServiceProxy(roi_service, SelectFusionROI)
        self.frame_roi = rospy.ServiceProxy(rospy.resolve_name(tracker_namespace+'/select_frame_roi'),
            __import__('sar_yolo_detector.srv',fromlist=['SelectFrameROI']).SelectFrameROI)
        self._tracker_namespace=tracker_namespace
        self._namespace=rospy.get_namespace().rstrip('/')
        self.manual_select = rospy.ServiceProxy(rospy.get_param('~select_manual_roi_service'), SelectFusionROI)
        self.select = rospy.ServiceProxy(rospy.get_param('~select_service'), SelectSmartTrack)
        self.switch = rospy.ServiceProxy(rospy.get_param('~mode_service'), SwitchTrackingMode)
        transport=str(rospy.get_param('~image_transport','raw'))
        if transport not in ('raw','compressed'):raise ValueError('image_transport must be raw or compressed')
        image_topic=rospy.get_param('~image_topic')+('/compressed' if transport=='compressed' else '')
        self.subscribers = [
            rospy.Subscriber(image_topic, CompressedImage if transport=='compressed' else Image, self.image, queue_size=1, buff_size=16*1024*1024, tcp_nodelay=True),
            rospy.Subscriber(rospy.get_param('~state_topic'), SmartTrackerState, self.state, queue_size=1),
            rospy.Subscriber(rospy.get_param('~mission_topic'), String, self.mission, queue_size=1),
            rospy.Subscriber(rospy.get_param('~targets_topic'), TrackedDetection2DArray, self.objects, queue_size=1)]
        classification_topic=rospy.get_param('~classification_topic','')
        if classification_topic:
            self.subscribers.append(rospy.Subscriber(classification_topic,String,self.classification_evidence,queue_size=1))
        self._console_init()
        fps=max(1.,min(10.,float(rospy.get_param('~fps',5.))))
        self.poll_interval_ms=int(round(1000./fps))
        self.timer = rospy.Timer(rospy.Duration(1./fps),self.encode)

    def image(self, message):
        if len(message.data) <= 16*1024*1024:
            self.frames.add_image(message)

    def state(self, message):
        h = message.header
        with self.lock:
            mode, epoch = self.active_mode, self.mode_epoch
        box = message.selected_bbox
        evidence = dict(capture_header=dict(seq=h.seq, frame_id=h.frame_id,
                stamp=dict(secs=h.stamp.secs, nsecs=h.stamp.nsecs)),
            overlay_capture_stamp=dict(secs=h.stamp.secs, nsecs=h.stamp.nsecs),
            mode=mode, selected_id=message.selected_track_id if message.has_selection else -1,
            source=message.association_method or 'none', reason=message.freshness_reason or '',
            selected_box=[box.center.x, box.center.y, box.size_x, box.size_y]
                if message.has_selection and message.geometry_type == 'aabb' else None,
            selected_box_xywh=[box.center.x-box.size_x/2.,box.center.y-box.size_y/2.,box.size_x,box.size_y]
                if message.has_selection and message.geometry_type == 'aabb' else None,
            confidence=float(message.confidence), backend=message.backend, device=message.device,
            measurement_current=bool(message.measurement_current),
            tracking_active=bool(message.tracking_active),
            control_target_valid=bool(message.control_measurement_ready),
            prediction_only=bool(message.prediction_only),tentative=bool(message.tentative),
            data_is_stale=bool(message.data_is_stale),frames_since_detection=int(message.frames_since_detection),
            selected_class_id=int(message.selected_class_id),selected_class_name=str(message.selected_class_name),
            selected_class_family=class_family(message.selected_class_name),
            model_path=message.model_path, processing_ms=float(message.frame_processing_ms),
            inference_ms=float(message.inference_ms),
            pipeline_epoch=epoch, capture_segment=epoch)
        # A delayed detector correction or explicit clear may refer to an
        # already displayed capture. Apply metadata immediately, without
        # resurrecting an older image or changing its selection token.
        if getattr(self,'source_overlay',False):
            with self.lock:
                if self.current is not None and self.current.get('capture_header')==evidence['capture_header']:
                    for key,value in evidence.items():
                        if key in self.current:self.current[key]=value
        try:
            self.frames.add_evidence(evidence)
        except (KeyError, TypeError, ValueError):
            rospy.logwarn_throttle(5., 'Invalid SmartTracker web metadata')

    def mission(self, message):
        try:
            value = json.loads(message.data)
            position = value.get('observer_ned_m')
            with self.lock:
                self.flight = dict(stage=str(value.get('stage', 'unknown')),
                    complete=bool(value.get('complete', False)),
                    elapsed_sec=value.get('elapsed_sec'), progress=value.get('progress'),
                    observer_ned_m=position if isinstance(position, list) and len(position) == 3 else None,
                    yaw_deg=value.get('yaw_deg'))
        except (TypeError, ValueError):
            rospy.logwarn_throttle(5., 'Invalid UE mission status')

    def objects(self, message):
        if len(message.detections) > 256:
            return
        objects = []
        for item in message.detections:
            d = item.detection
            category = d.results[0].id if d.results else -1
            track_id = int(item.track_id)
            is_manual = track_id >= 900000000
            b = d.bbox
            if not all(math.isfinite(v) for v in (b.center.x, b.center.y, b.size_x, b.size_y)) or min(b.size_x, b.size_y) <= 0:
                continue
            objects.append(dict(id=track_id, class_id=category,
                name='manual_roi' if is_manual else (self.labels[category] if 0 <= category < len(self.labels) else 'unknown'),
                kind='manual_track' if is_manual else 'detector_track',
                confidence=float(d.results[0].score) if d.results else 0.,
                center=[b.center.x, b.center.y], box=[b.center.x-b.size_x/2, b.center.y-b.size_y/2, b.size_x, b.size_y]))
        with self.lock:
            key=message.header.stamp.to_nsec()
            self.targets[key] = self._attach_classification(key,objects)
            if self.current is not None:
                stamp=self.current.get('capture_header',{}).get('stamp',{})
                if stamp.get('secs',0)*1000000000+stamp.get('nsecs',0)==key:self.current['objects']=self.targets[key]
            while len(self.targets) > 32:
                self.targets.popitem(last=False)

    def _attach_classification(self, key, objects):
        evidence=getattr(self,'classification',{}).get(key,{})
        output=[]
        for original in objects:
            row=dict(original)
            row['family']=class_family(row.get('name',''))
            item=evidence.get(row['id'])
            if item is not None:
                raw=int(item['raw_class_id'])
                row.update(raw_class_id=raw,raw_name=self.labels[raw] if 0<=raw<len(self.labels) else 'unknown',
                           label_source=item.get('label_source','detector'),confidence_kind='proposal_score')
            output.append(row)
        return output

    def classification_evidence(self, message):
        try:
            payload=json.loads(message.data)
            if payload.get('schema_version')!=1 or len(payload.get('objects',[]))>256:
                return
            stamp=payload['stamp']
            key=int(stamp['secs'])*1000000000+int(stamp['nsecs'])
            values={int(row['id']):row for row in payload['objects']}
            with self.lock:
                self.classification[key]=values
                while len(self.classification)>32:
                    self.classification.popitem(last=False)
                if key in self.targets:
                    self.targets[key]=self._attach_classification(key,self.targets[key])
                if self.current is not None:
                    current=self.current.get('capture_header',{}).get('stamp',{})
                    if (current.get('secs'),current.get('nsecs'))==(stamp['secs'],stamp['nsecs']):
                        self.current['objects']=self._attach_classification(key,self.current['objects'])
        except (KeyError,TypeError,ValueError):
            rospy.logwarn_throttle(5.,'Invalid class evidence')

    def encode(self, event):
        pair = self.frames.take()
        if pair is None:
            return
        image, evidence = pair
        try:
            is_compressed=isinstance(image,CompressedImage)
            codec=getattr(self,'image_codec','png')
            if is_compressed and codec=='jpeg' and 'jpeg' in image.format.lower():
                # The browser decodes these exact JPEG bytes. The ROS web
                # worker needs only validated dimensions for ROI coordinates.
                from PIL import Image as EncodedImage
                if image.data[:2]!=b'\xff\xd8' or image.data[-2:]!=b'\xff\xd9':raise ValueError('Incomplete JPEG preview')
                with EncodedImage.open(io.BytesIO(image.data)) as encoded:
                    width,height=encoded.size
                    if encoded.format!='JPEG' or min(width,height)<1 or width*height>16777216:
                        raise ValueError('Invalid JPEG preview geometry')
                ok,png=True,np.frombuffer(image.data,np.uint8)
            else:
                frame = (self.bridge.compressed_imgmsg_to_cv2(image,desired_encoding='bgr8') if is_compressed else
                    self.bridge.imgmsg_to_cv2(image,desired_encoding='bgr8'))
                height,width=frame.shape[:2]
                extension,options=('.jpg',[cv2.IMWRITE_JPEG_QUALITY,90]) if codec=='jpeg' else ('.png',[cv2.IMWRITE_PNG_COMPRESSION,1])
                ok,png=cv2.imencode(extension,frame,options)
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
                    source_image_overlay=getattr(self,"source_overlay",False),
                    image_capture_stamp=dict(secs=image.header.stamp.secs, nsecs=image.header.stamp.nsecs),
                    overlay_capture_stamp=evidence.get("overlay_capture_stamp"),
                    width=width, height=height,
                    _image_bytes=png.tobytes(), image_mime='image/jpeg' if codec=='jpeg' else 'image/png',
                    selected_id=evidence.get('selected_id', -1), source=evidence.get('source'),
                    reason=evidence.get('reason'), mode=evidence.get('mode'),
                    selected_box=evidence.get('selected_box'),
                    selected_box_xywh=evidence.get('selected_box_xywh'),
                    measurement_current=evidence.get('measurement_current', False),
                    tracking_active=evidence.get('tracking_active', False),
                    control_target_valid=evidence.get('control_target_valid', False),
                    prediction_only=evidence.get('prediction_only',False),tentative=evidence.get('tentative',False),
                    data_is_stale=evidence.get('data_is_stale',False),frames_since_detection=evidence.get('frames_since_detection',0),
                    selected_class_id=evidence.get('selected_class_id',-1),selected_class_name=evidence.get('selected_class_name',''),
                    selected_class_family=evidence.get('selected_class_family',''),
                    confidence=evidence.get('confidence', 0.), backend=evidence.get('backend'),
                    device=evidence.get('device'), processing_ms=evidence.get('processing_ms', 0.),
                    inference_ms=evidence.get('inference_ms', 0.),
                    objects=self.targets.get(image.header.stamp.to_nsec(), []),
                    flight=copy.deepcopy(self.flight),
                    stamp=image.header.stamp.to_sec(), received_wall=time.monotonic())
                self.history[token] = copy.deepcopy(evidence['capture_header']), width, height
                while len(self.history) > 24:
                    self.history.popitem(last=False)
        except Exception as exc:
            rospy.logwarn_throttle(5., 'Web PNG conversion failed: %s', exc)

    def snapshot(self, include_image=True, binary=False):
        with self.lock:
            if self.current is None:
                return dict(ready=False)
            output = dict(self.current)
        image_bytes=output.pop('_image_bytes',None)
        if binary:output['_image_bytes']=image_bytes
        elif include_image and image_bytes is not None:output['png']=base64.b64encode(image_bytes).decode('ascii')
        if not include_image:output.pop('png',None)
        output['ready'] = True
        output['benchmark_controls']=getattr(self,'benchmark',None) is not None
        output['poll_interval_ms']=getattr(self,'poll_interval_ms',200)
        output['age_ms'] = max(0., (rospy.Time.now().to_sec()-output['stamp'])*1000.)
        output['control_max_age_ms']=getattr(self,'control_max_age_ms',500.)
        output['video_stale'] = (time.monotonic()-output.pop('received_wall') > 2.
                                 or output['age_ms'] > self.max_age*1000.)
        # A once-current state does not remain a current control observation
        # while its paired display frame ages in a browser or transport queue.
        if output['video_stale'] or output['age_ms']>output['control_max_age_ms']:output['control_target_valid']=False
        return output

    def command(self, body):
        if not self.commands.acquire(blocking=False):
            return 409, dict(success=False, message='BUSY: previous command is still running')
        try:
            action = body.get('action')
            if action == 'benchmark':
                command=str(body.get('command',''))
                if command not in ('start','hold'):raise ValueError('Unknown scene motion command')
                publisher=getattr(self,'benchmark',None)
                if publisher is None or not publisher.get_num_connections():
                    return 409,dict(success=False,message='UE 场景运动控制尚未连接')
                publisher.publish(String(data=command))
                return 200,dict(success=True,message='目标运动已启动' if command=='start' else '目标运动已暂停')
            if action == 'mode':
                mode = str(body.get('mode', '')).strip().lower()
                allowed = {'smart_tracker', 'feature_tracker', 'color_tracker', 'fusion_tracker'}
                if mode not in allowed:
                    raise ValueError('Unknown tracking mode')
                request = SwitchTrackingModeRequest(mode=mode,
                    color_name=str(body.get('color_name', '')).strip().lower())
                self.switch.wait_for_service(timeout=2.)
                result = self.switch(request)
                if not result.success:
                    return 409, dict(success=False, message=result.message, active_mode=result.active_mode)
                with self.lock:
                    self.active_mode = result.active_mode
                    self.mode_epoch += 1
                if hasattr(self,'_console_lock'):
                    with self._console_lock:self._console_leases.clear()
                return 200, dict(success=True, active_mode=result.active_mode, message=result.message,
                    waiting_for_initial_roi=result.waiting_for_initial_roi)
            with self.lock:
                mode = self.active_mode
            if action == 'clear':
                if mode == 'fusion_tracker':
                    self.roi.wait_for_service(timeout=2.)
                    result = self.roi(SelectFusionROIRequest(clear_selection=True))
                else:
                    request = SelectSmartTrackRequest(clear_selection=True)
                    self.select.wait_for_service(timeout=2.)
                    result = self.select(request)
                return (200 if result.success else 409), dict(success=result.success,
                    selected_id=getattr(result, 'selected_target_id', -1), message=result.message)
            request = SelectFusionROIRequest(clear_selection=False)
            if action != 'clear':
                token=str(body.get('frame_token'))
                original=self._console_pinned_original(token,body.get('lease_id'))
                if original is None:
                    with self.lock:original=self.history.get(token)
                if original is None:
                    raise ValueError('Displayed frame is no longer cached; refresh and retry')
                h, width, height = original
                self._console_validate_selection(body,token,h)
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
                    if mode in ('feature_tracker', 'color_tracker'):
                        raise ValueError('Drag a box around the target to initialize this manual mode')
                    if mode == 'fusion_tracker':
                        with self.lock:
                            objects = list(getattr(self, 'targets', {}).get(request.header.stamp.to_nsec(), ()))
                        hits = [item for item in objects if item.get('kind') == 'detector_track'
                                and len(item.get('box', ())) == 4 and
                                item['box'][0] <= x <= item['box'][0]+item['box'][2] and
                                item['box'][1] <= y <= item['box'][1]+item['box'][3]]
                        if hits:
                            def distance(item):
                                left, top, bw, bh = item['box']
                                return math.hypot((x-left-bw/2.)/max(bw/2.,1.),
                                                  (y-top-bh/2.)/max(bh/2.,1.))
                            hits.sort(key=lambda item:(distance(item),-float(item.get('confidence',0.))))
                            if len(hits)>1 and (distance(hits[0])>.35 or
                                                distance(hits[1])<distance(hits[0])*1.7+.08):
                                raise ValueError('Click overlaps multiple target centers; click nearer one center')
                            left, top, bw, bh = hits[0]['box']
                            request.roi = RegionOfInterest(x_offset=max(0,int(left)), y_offset=max(0,int(top)),
                                width=min(width-max(0,int(left)),max(4,int(math.ceil(bw)))),
                                height=min(height-max(0,int(top)),max(4,int(math.ceil(bh)))))
                            self.roi.wait_for_service(timeout=2.)
                            result = self.roi(request)
                            return (200 if result.success else 409), dict(success=result.success,
                                selected_id=result.selected_target_id, message=result.message)
                    if not self.click.get_num_connections():
                        raise ValueError('Click subscriber is not connected')
                    message = PointStamped(header=request.header)
                    message.point.x, message.point.y = x, y
                    self.click.publish(message)
                    return 202, dict(success=True, sent=True, message='Click sent; verify actual selected ID')
                if action != 'roi':
                    raise ValueError('Unknown command')
                w, h = values[2:]
                if w < 4 or h < 4 or x+w > width or y+h > height:
                    raise ValueError('ROI must be inside the image and at least 4x4 pixels')
                request.roi.x_offset, request.roi.y_offset = int(x), int(y)
                if hasattr(self,'frame_roi'):
                    from sar_yolo_detector.srv import SelectFrameROIRequest
                    exact=SelectFrameROIRequest(header=request.header,roi=request.roi,clear_selection=False)
                    self.frame_roi.wait_for_service(timeout=2.)
                    result=self.frame_roi(exact)
                    return (200 if result.success else 409),dict(success=result.success,
                        selected_id=result.selected_target_id,message=result.message)
                if mode in ('feature_tracker','color_tracker'):
                    self.manual_select.wait_for_service(timeout=2.)
                    result=self.manual_select(request)
                    return (200 if result.success else 409),dict(success=result.success,selected_id=result.selected_target_id,message=result.message)
                if mode == 'smart_tracker':
                    request = SelectSmartTrackRequest(use_normalized_roi=True,
                        normalized_roi=[x/width, y/height, w/width, h/height])
                    self.select.wait_for_service(timeout=2.)
                    result = self.select(request)
                    return (200 if result.success else 409), dict(success=result.success,
                        selected_id=result.selected_target_id, message=result.message)
                request.roi.width, request.roi.height = int(w), int(h)
            self.roi.wait_for_service(timeout=2.)
            result = self.roi(request)
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
            request.setsockopt(socket.IPPROTO_TCP,socket.TCP_NODELAY,1)
            super().process_request_thread(request, address)
        finally:
            self.slots.release()


def main():
    rospy.init_node('fusion_web_preview')
    cv2.setNumThreads(1)
    preview = Preview()
    html = (Path(rospkg.RosPack().get_path('sar_yolo_detector'))/'web/dist/index.html').read_bytes()

    class Handler(BaseHTTPRequestHandler):
        protocol_version='HTTP/1.1'
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
            parsed=urlparse(self.path);path=parsed.path
            if path == '/':
                self.reply(200, html, 'text/html; charset=utf-8')
            elif path == '/api/frame':
                self.reply(200, preview.snapshot())
            elif path == '/api/image':
                packet=preview.snapshot(include_image=False,binary=True)
                data=packet.pop('_image_bytes',None)
                if data is None:
                    self.reply(503,dict(ready=False));return
                # The metadata may change after the image was displayed.
                # Return fresh metadata without retransmitting identical pixels.
                if self.headers.get('X-SAR-Displayed-Frame') == packet['frame_token']:
                    data=b''
                self.send_response(200)
                self.send_header('Content-Type',packet['image_mime'])
                self.send_header('Content-Length',str(len(data)))
                self.send_header('Cache-Control','no-store')
                self.send_header('X-SAR-Frame',base64.b64encode(json.dumps(packet,ensure_ascii=True,separators=(',',':')).encode()).decode('ascii'))
                self.end_headers()
                try:self.wfile.write(data)
                except (BrokenPipeError,ConnectionResetError):pass
            elif path == '/api/telemetry':
                self.reply(200, preview.snapshot(include_image=False))
            elif path == '/api/console':
                query=parse_qs(parsed.query);token=query.get('frame_token',[''])[0]
                lease=query.get('lease_id',[''])[0]
                self.reply(200,preview.console_snapshot(token or None,lease or None))
            else:
                self.reply(404, dict(message='Not found'))

        def do_POST(self):
            try:
                origin = self.headers.get('Origin')
                if origin and urlparse(origin).netloc != self.headers.get('Host'):
                    raise ValueError('Cross-origin command rejected')
                path=urlparse(self.path).path
                if path not in ('/api/command','/api/console/command') or self.headers.get('Content-Type', '').split(';')[0] != 'application/json':
                    raise ValueError('JSON command required')
                length = int(self.headers.get('Content-Length', '0'))
                if not 0 < length <= 16384:
                    raise ValueError('Invalid command length')
                body = json.loads(self.rfile.read(length))
                if not isinstance(body, dict):
                    raise ValueError('JSON object required')
                status, result = (preview.console_command(body) if path=='/api/console/command'
                                  else preview.command(body))
                self.reply(status, result)
            except (ValueError, TypeError) as exc:
                self.reply(400, dict(success=False, message=str(exc)))

    host, port = rospy.get_param('~bind_address', '127.0.0.1'), int(rospy.get_param('~port', 8787))
    hosts=[host]
    local_host=os.environ.get('SAR_LOCAL_WEB_HOST','')
    if host in ('127.0.0.1','localhost') and local_host and local_host not in hosts:hosts.append(local_host)
    servers=[]
    for address in hosts:
        server=Server((address,port),Handler)
        servers.append(server)
        threading.Thread(target=server.serve_forever,daemon=True).start()
    rospy.loginfo('Fusion web preview: http://%s:%d (no second inference model)', host, port)
    try:
        rospy.spin()
    finally:
        preview.timer.shutdown()
        for server in servers:
            server.shutdown()
            server.server_close()


if __name__ == '__main__':
    main()
