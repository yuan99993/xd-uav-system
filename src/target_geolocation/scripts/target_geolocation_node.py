#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""target_geolocation_node —— 由飞机 GPS + 云台姿态 + 测距解算目标 GPS 坐标。

数据流
------
    /uav1/mavros/global_position/global        NavSatFix    飞机 GPS（基准点）
    /uav1/mavlink/from  (#285)                 Mavlink      云台真实姿态（世界系四元数）
      └ 或 ~gimbal_att_topic                   QuaternionStamped / Imu / Float64MultiArray
                                              外部给定姿态；Float64MultiArray 走
                                              [yaw,pitch,roll] deg（见下）
    /uav1/mavros/global_position/compass_hdg   Float64      罗盘航向（机体系 yaw / 未锁 yaw 时必需）
    /uav1/mavros/distance_sensor/laser_1d      Range        测距仪：沿光轴的斜距
    /uav1/mavros/local_position/pose           PoseStamped  仅用于把结果画到 RViz（可选）

    -> /uav1/target/global_position            NavSatFix    目标经纬高（主输出）
    -> /uav1/target/geolocated                 GeolocatedTarget 坐标 + 框类别/置信度/track_id
    -> /uav1/target/local_enu                  PointStamped 相对飞机的 ENU 偏移
    -> /uav1/target/marker                     Marker       RViz 可视化
    -> /uav1/target/solution                   Float64MultiArray  11 维调试量

解算原理
--------
目标 = 飞机位置 + 斜距 × 光轴单位向量（在局部 ENU 切平面内相加，再投回 WGS84）。
光轴单位向量由云台姿态四元数给出，YAW_LOCK 未置位时用罗盘航向把机体系 yaw
抬到世界系；再叠加安装偏差(boresight)与目标在图像上的偏角。
细节见 src/target_geolocation/solver.py 的模块说明。
"""

import math
import os
import struct
import threading

import rospy
from geometry_msgs.msg import PointStamped, PoseStamped, QuaternionStamped
from mavros_msgs.msg import Mavlink
from sensor_msgs.msg import Imu, NavSatFix, Range
from std_msgs.msg import Float32, Float64, Float64MultiArray
from visualization_msgs.msg import Marker
from sar_yolo_detector.msg import TrackedDetection2DArray
from target_geolocation.msg import GeolocatedTarget
from xd_uav_track.msg import DetectionArray

from target_geolocation import geodesy, solver
from target_geolocation.solver import GIMBAL_DEVICE_FLAGS_YAW_LOCK

try:
    from vision_msgs.msg import BoundingBox2D
    _HAVE_VISION_MSGS = True
except ImportError:                                    # pragma: no cover
    _HAVE_VISION_MSGS = False

MSG_GIMBAL_DEVICE_ATTITUDE_STATUS = 285

# sensor_msgs/NavSatStatus
NAVSAT_STATUS_FIX = 0
# sensor_msgs/NavSatFix
COVARIANCE_TYPE_DIAGONAL_KNOWN = 1


def _join(ns, rel):
    """拼接命名空间与相对话题名，兼容 ns 为空/带尾斜杠的情况。"""
    if not rel:
        return ''
    if rel.startswith('/'):
        return rel
    if not ns:
        return '/' + rel
    return ns.rstrip('/') + '/' + rel


def _stamp_or_now(msg):
    """取消息时间戳；为 0（某些桥接不填）时退回当前时间，避免被误判为过期。"""
    stamp = msg.header.stamp
    if stamp.to_sec() <= 1e-3:
        return rospy.Time.now()
    return stamp


class TargetGeolocationNode(object):

    def __init__(self):
        rospy.init_node('target_geolocation')

        # ------------------------------------------------------- 话题与命名空间
        self.uav_ns = rospy.get_param('~uav_ns', '/uav1').rstrip('/')
        # 注意用 `or`：yaml 里留空的 "" 不能覆盖掉按 uav_ns 自动拼接的默认话题名
        self.out_prefix = (rospy.get_param('~output_prefix', '')
                           or _join(self.uav_ns, 'target'))

        self.gps_topic = (rospy.get_param('~gps_topic', '')
                          or _join(self.uav_ns, 'mavros/global_position/global'))
        self.heading_topic = (rospy.get_param('~heading_topic', '')
                              or _join(self.uav_ns, 'mavros/global_position/compass_hdg'))
        self.mavlink_topic = (rospy.get_param('~mavlink_topic', '')
                              or _join(self.uav_ns, 'mavlink/from'))
        self.range_topic = (rospy.get_param('~range_topic', '')
                            or _join(self.uav_ns, 'mavros/distance_sensor/laser_1d'))
        self.local_pose_topic = (rospy.get_param('~local_pose_topic', '')
                                 or _join(self.uav_ns, 'mavros/local_position/pose'))
        self.gimbal_att_topic = rospy.get_param('~gimbal_att_topic', '')
        self.att_msg_type = str(rospy.get_param('~att_msg_type', 'QuaternionStamped')).lower()
        self.det_topic = rospy.get_param('~det_topic', '')
        self.det_msg_type = str(rospy.get_param('~det_msg_type', 'BoundingBox2D')).lower()
        self.det_class_id = int(rospy.get_param('~det_class_id', -1))
        self.det_sensor_id = str(rospy.get_param('~det_sensor_id', 'gimbal_camera'))

        # -------- 欧拉角形式的云台姿态（如 /uav1/gimbal/attitude，[yaw,pitch,roll] deg）
        # yaw_is_body: 该 yaw 是相对机头(机体系)时置 True，世界方位 = yaw + 航向。
        #   /uav1/gimbal/attitude 实测为机体系：与世界系 #285 yaw 满足
        #   bridge_yaw = world_yaw - heading，故默认 True。
        self.gimbal_yaw_is_body = bool(rospy.get_param('~gimbal_yaw_is_body', True))
        self.gimbal_pitch_up_positive = bool(rospy.get_param('~gimbal_pitch_up_positive', True))

        # ------------------------------------------------------------- 解算参数
        self.boresight_rpy = (
            float(rospy.get_param('~boresight_roll_deg', 0.0)),
            float(rospy.get_param('~boresight_pitch_deg', 0.0)),
            float(rospy.get_param('~boresight_yaw_deg', 0.0)),
        )
        self.sensor_axis = str(rospy.get_param('~sensor_axis', 'x')).lower()
        if self.sensor_axis not in ('x', 'y', 'z'):
            rospy.logwarn('[GEO] sensor_axis=%s 非法，回退为 x', self.sensor_axis)
            self.sensor_axis = 'x'

        # ------------------------------------------------------------- 有效性门限
        self.publish_rate = float(rospy.get_param('~publish_rate', 10.0))
        self.max_input_age = float(rospy.get_param('~max_input_age', 0.5))
        self.max_skew = float(rospy.get_param('~max_skew', 0.20))
        self.min_range = float(rospy.get_param('~min_range', 0.5))
        self.max_range = float(rospy.get_param('~max_range', 3000.0))
        # 测距零偏补偿：实测激光常停在靶标"最近的表面"而非其中心，靶标有体积时
        # 会系统性地偏短（本仿真对 uav2 实测 -1.63 ~ -1.97 m）。补偿后靶点更贴近中心。
        self.range_bias = float(rospy.get_param('~range_bias', 0.0))
        self.require_downward = bool(rospy.get_param('~require_downward', True))
        self.max_ground_range = float(rospy.get_param('~max_ground_range', 5000.0))
        self.smooth_alpha = float(rospy.get_param('~smooth_alpha', 0.4))  # 0=关闭

        # --------------------------------------------------------- 图像偏角修正
        self.use_img_offset = bool(rospy.get_param('~use_img_offset', True))
        self.fov_x_deg = float(rospy.get_param('~fov_x_deg', 45.0))
        self.img_w = int(rospy.get_param('~image_width', 1280))
        self.img_h = int(rospy.get_param('~image_height', 720))
        self.max_img_offset_deg = float(rospy.get_param('~max_img_offset_deg', 20.0))
        self.det_timeout = float(rospy.get_param('~det_timeout', 0.3))

        # ------------------------------------------------------------ 输出设置
        self.marker_frame = rospy.get_param('~marker_frame', 'map')
        self.marker_scale = float(rospy.get_param('~marker_scale', 3.0))
        self.range_msg_type = str(rospy.get_param('~range_msg_type', 'Range'))

        # ------------------------------------------------------------ 共享状态
        self._lock = threading.Lock()
        self._gps = None       # dict(lat, lon, alt, stamp)
        self._range = None     # dict(value, stamp)
        self._gimbal = None    # dict(q, yaw_locked, stamp, src)
        self._heading = None   # dict(deg, stamp)
        self._local = None     # dict(x, y, z)
        self._det = None       # dict(u, v, stamp)
        self._ema = None       # 平滑用的 ENU 偏移
        self._last_valid = None

        # 输入版本号：每个回调自增，_tick 里据此跳过"输入没变"的重复解算
        self._rev = 0
        self._last_rev = -1
        self.publish_only_on_change = bool(
            rospy.get_param('~publish_only_on_change', True))
        self.enable_marker = bool(rospy.get_param('~enable_marker', True))

        # -------------------------------------------------------------- 订阅
        rospy.Subscriber(self.gps_topic, NavSatFix, self._gps_cb, queue_size=5)
        rospy.Subscriber(self.heading_topic, Float64, self._heading_cb, queue_size=5)
        rospy.Subscriber(self.local_pose_topic, PoseStamped, self._local_cb, queue_size=5)

        if self.gimbal_att_topic:
            if self.att_msg_type in ('imu', 'sensor_msgs/imu'):
                # AirSim / 飞控直接给 IMU 姿态（机体 -> 世界四元数）
                rospy.Subscriber(self.gimbal_att_topic, Imu, self._imu_att_cb, queue_size=20)
                rospy.loginfo('[GEO] 云台/机体姿态来源: Imu 话题 %s', self.gimbal_att_topic)
            elif self.att_msg_type in ('float64multiarray', 'euler', 'gimbal_euler',
                                       'std_msgs/float64multiarray'):
                # [yaw, pitch, roll] 单位 deg，yaw 默认机体系（见 ~gimbal_yaw_is_body）
                rospy.Subscriber(self.gimbal_att_topic, Float64MultiArray,
                                 self._gimbal_euler_cb, queue_size=20)
                rospy.loginfo('[GEO] 云台姿态来源: 欧拉角话题 %s  [yaw,pitch,roll] deg，'
                              'yaw_is_body=%s pitch_up_positive=%s',
                              self.gimbal_att_topic, self.gimbal_yaw_is_body,
                              self.gimbal_pitch_up_positive)
            else:
                rospy.Subscriber(self.gimbal_att_topic, QuaternionStamped,
                                 self._gimbal_att_cb, queue_size=20)
                rospy.loginfo('[GEO] 云台姿态来源: 话题 %s (按世界系四元数处理)',
                              self.gimbal_att_topic)
        else:
            # 直接按字节解 #285，不需要 pymavlink，也不再逐帧重建整帧
            rospy.Subscriber(self.mavlink_topic, Mavlink, self._mavlink_cb, queue_size=20)
            rospy.loginfo('[GEO] 云台姿态来源: %s 中的 #285（按 msgid 预过滤）',
                          self.mavlink_topic)

        if self.range_msg_type.lower() in ('float32', 'float', 'std_msgs/float32'):
            rospy.Subscriber(self.range_topic, Float32, self._range_float_cb, queue_size=10)
        else:
            rospy.Subscriber(self.range_topic, Range, self._range_cb, queue_size=10)

        if self.det_topic and self.det_msg_type in ('detectionarray', 'xd_uav_track/detectionarray'):
            rospy.Subscriber(self.det_topic, DetectionArray, self._det_array_cb, queue_size=5)
        elif self.det_topic and self.det_msg_type in (
                'trackeddetection2darray', 'sar_yolo_detector/trackeddetection2darray'):
            rospy.Subscriber(
                self.det_topic, TrackedDetection2DArray,
                self._tracked_det_array_cb, queue_size=5)
        elif self.det_topic and self.det_msg_type in ('boundingbox2d', 'vision_msgs/boundingbox2d'):
            if _HAVE_VISION_MSGS:
                rospy.Subscriber(self.det_topic, BoundingBox2D, self._det_cb, queue_size=5)
            else:
                rospy.logwarn('[GEO] 指定了 BoundingBox2D 但缺少 vision_msgs，忽略图像偏角')
        elif self.det_topic:
            rospy.logwarn('[GEO] det_msg_type=%s 不支持，忽略图像偏角', self.det_msg_type)

        # -------------------------------------------------------------- 发布
        self.pub_fix = rospy.Publisher(
            _join(self.out_prefix, 'global_position'), NavSatFix, queue_size=5)
        self.pub_geolocated = rospy.Publisher(
            _join(self.out_prefix, 'geolocated'), GeolocatedTarget, queue_size=5)
        self.pub_local = rospy.Publisher(
            _join(self.out_prefix, 'local_enu'), PointStamped, queue_size=5)
        self.pub_marker = rospy.Publisher(
            _join(self.out_prefix, 'marker'), Marker, queue_size=5)
        self.pub_solution = rospy.Publisher(
            _join(self.out_prefix, 'solution'), Float64MultiArray, queue_size=5)

        rospy.on_shutdown(self._on_shutdown)
        rospy.Timer(rospy.Duration(1.0 / max(self.publish_rate, 1.0)), self._tick)

        rospy.loginfo('[GEO] 就绪: gps=%s range=%s -> %s and %s', self.gps_topic,
                      self.range_topic,
                      _join(self.out_prefix, 'global_position'),
                      _join(self.out_prefix, 'geolocated'))
        rospy.loginfo('[GEO] boresight rpy=%s deg, sensor_axis=%s, 图像偏角修正=%s',
                      self.boresight_rpy, self.sensor_axis, self.use_img_offset)

    # =============================================================== 回调
    def _gps_cb(self, msg):
        if msg.status.status < NAVSAT_STATUS_FIX:
            return
        lat, lon = msg.latitude, msg.longitude
        if not (math.isfinite(lat) and math.isfinite(lon)) or (lat == 0.0 and lon == 0.0):
            return
        with self._lock:
            self._rev += 1
            self._gps = dict(lat=lat, lon=lon, alt=msg.altitude,
                             stamp=_stamp_or_now(msg))

    def _heading_cb(self, msg):
        if not math.isfinite(msg.data):
            return
        with self._lock:
            self._rev += 1
            self._heading = dict(deg=float(msg.data) % 360.0,
                                 stamp=rospy.Time.now())

    def _local_cb(self, msg):
        with self._lock:
            self._rev += 1
            self._local = dict(x=msg.pose.position.x, y=msg.pose.position.y,
                               z=msg.pose.position.z)

    def _range_cb(self, msg):
        value = float(msg.range)
        if not math.isfinite(value):
            return
        with self._lock:
            self._rev += 1
            self._range = dict(value=value, stamp=_stamp_or_now(msg))

    def _range_float_cb(self, msg):
        value = float(msg.data)
        if not math.isfinite(value):
            return
        with self._lock:
            self._rev += 1
            self._range = dict(value=value, stamp=rospy.Time.now())

    def _gimbal_att_cb(self, msg):
        """外部给定云台姿态（仿真或飞控已解算好的世界系四元数）。"""
        q_msg = msg.quaternion
        with self._lock:
            self._rev += 1
            self._gimbal = dict(q=(q_msg.w, q_msg.x, q_msg.y, q_msg.z),
                                yaw_locked=True, stamp=_stamp_or_now(msg),
                                src='topic')

    def _imu_att_cb(self, msg):
        """IMU 姿态（sensor_msgs/Imu）。机体固连传感器（无云台）时直接用它当光轴姿态。

        注意这一路是"机体 -> 世界"的完整姿态：Project AirSim / PX4 用 NED/FRD
        （X 前 Y 右 Z 下），airframe 的 yaw 已是世界系，等价于 YAW_LOCK 置位。
        若换成按 REP-103 发布 ENU/FLU 的驱动，需要把四元数转换后再喂进来。
        """
        q_msg = msg.orientation
        with self._lock:
            self._rev += 1
            self._gimbal = dict(q=(q_msg.w, q_msg.x, q_msg.y, q_msg.z),
                                yaw_locked=True, stamp=_stamp_or_now(msg),
                                src='imu')

    def _gimbal_euler_cb(self, msg):
        """欧拉角云台姿态，如 /uav1/gimbal/attitude = [yaw, pitch, roll]（deg）。

        暂存原始欧拉角；合成世界系四元数放到 _tick 里做，因为机体系 yaw 需要
        与当前航向配对（航向本身也在更新，放在这里配会用错时刻的航向）。
        """
        d = list(msg.data)
        if len(d) < 3:
            return
        yaw, pitch, roll = (float(d[0]), float(d[1]), float(d[2]))
        if not (math.isfinite(yaw) and math.isfinite(pitch) and math.isfinite(roll)):
            return
        with self._lock:
            self._rev += 1
            self._gimbal = dict(q=None,
                                euler=(yaw, pitch, roll),
                                yaw_locked=not self.gimbal_yaw_is_body,
                                stamp=rospy.Time.now(),
                                src='euler')

    def _euler_to_world_quat(self, euler, heading_deg):
        """[yaw, pitch, roll] deg + 航向 -> 云台世界系四元数 (w,x,y,z)，NED/FRD。

        约定（aeronautical / MAVLink）：
          yaw   0=北，顺时针为正（朝东 90°）
          pitch 抬头为正（上正）；下俯 19.85° => -19.85
          roll  右滚为正
        机体系 yaw 加航向即得世界 yaw；roll 在两条链路里都取原值（云台滚转很小）。
        """
        yaw, pitch, roll = euler
        if not self.gimbal_pitch_up_positive:
            pitch = -pitch                      # 输入是"下正"时翻回来
        yaw_world = yaw + heading_deg if self.gimbal_yaw_is_body else yaw
        yaw_world %= 360.0
        return solver.quat_from_euler_zyx(math.radians(roll),
                                          math.radians(pitch),
                                          math.radians(yaw_world))

    # ---------------------------------------------------- #285 反馈帧解析
    def _mavlink_cb(self, msg):
        """从 mavros_msgs/Mavlink 里取 #285 的云台姿态。

        性能要点（这条话题常跑到 200 Hz 以上、混杂几十种消息）：
          1. 先用 msg.msgid 过滤，非 #285 立即返回 —— 不做任何字节处理；
          2. 只在命中时才把 payload64 拼成字节，直接 struct 解四元数与 flags。
        早期实现是"每帧都重建整帧再交给 pymavlink 解析，最后才判 msgid"，
        单帧 0.85 us vs 现在 0.05 us，且不再依赖 pymavlink。
        """
        if (msg.msgid & 0xFFFFFF) != MSG_GIMBAL_DEVICE_ATTITUDE_STATUS:
            return
        try:
            payload = b''.join(struct.pack('<Q', w) for w in msg.payload64)
        except (TypeError, struct.error):
            return
        n = int(getattr(msg, 'len', len(payload)) or len(payload))
        if n < 24:
            return
        # GIMBAL_DEVICE_ATTITUDE_STATUS 前段布局（MAVLink 按类型排序，无填充）：
        #   time_boot_ms u32 | q[4] float (w,x,y,z) | target_system u8
        #   target_component u8 | flags u16 | failure_flags u32 | ...
        try:
            qw, qx, qy, qz = struct.unpack('<4f', payload[4:20])
            (flags,) = struct.unpack('<H', payload[22:24])
        except struct.error:
            return
        if math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz) < 0.5:
            return                                   # 云台未上报有效姿态
        with self._lock:
            self._gimbal = dict(q=(qw, qx, qy, qz),
                                yaw_locked=bool(flags & GIMBAL_DEVICE_FLAGS_YAW_LOCK),
                                stamp=_stamp_or_now(msg), src='mavlink')
            self._rev += 1

    def _det_cb(self, msg):
        with self._lock:
            self._rev += 1
            self._det = dict(u=msg.center.position.x, v=msg.center.position.y,
                             stamp=_stamp_or_now(msg),
                             class_id=self.det_class_id,
                             confidence=1.0,
                             track_id=-1,
                             track_id_is_stable=False,
                             sensor_id='bounding_box_2d')

    def _det_array_cb(self, msg):
        """Use the configured DetectionArray bbox center for off-axis correction."""
        width, height = int(msg.image_width), int(msg.image_height)
        if width <= 0 or height <= 0:
            with self._lock:
                self._rev += 1
                self._det = None
            return
        candidates = []
        for candidate in msg.candidates:
            class_id = int(candidate.class_id)
            if class_id < 0:
                continue
            if self.det_class_id >= 0 and class_id != self.det_class_id:
                continue
            if bool(candidate.has_normalized_bbox):
                cx = float(candidate.normalized_bbox[0]) * width
                cy = float(candidate.normalized_bbox[1]) * height
            elif bool(candidate.has_bbox):
                cx = 0.5 * (float(candidate.bbox[0]) + float(candidate.bbox[2]))
                cy = 0.5 * (float(candidate.bbox[1]) + float(candidate.bbox[3]))
            else:
                continue
            confidence = float(candidate.confidence)
            if not (math.isfinite(cx) and math.isfinite(cy) and math.isfinite(confidence)):
                continue
            candidates.append((confidence, cx, cy, candidate))
        if not candidates:
            with self._lock:
                self._rev += 1
                self._det = None
            return
        _, center_x, center_y, selected = max(
            candidates, key=lambda item: item[0])
        with self._lock:
            self._rev += 1
            self._det = dict(
                u=center_x,
                v=center_y,
                image_width=width,
                image_height=height,
                stamp=_stamp_or_now(msg),
                class_id=int(selected.class_id),
                confidence=float(selected.confidence),
                track_id=int(selected.track_id),
                track_id_is_stable=bool(selected.track_id_is_stable),
                sensor_id=str(msg.sensor_id),
            )

    def _tracked_det_array_cb(self, msg):
        """Use the currently selected smart-tracker box sent to gimbal control."""
        candidates = []
        for tracked in msg.detections:
            detection = tracked.detection
            results = list(detection.results)
            if results:
                hypothesis = max(results, key=lambda item: float(item.score))
                try:
                    class_id = int(hypothesis.id)
                except (TypeError, ValueError):
                    continue
                confidence = float(hypothesis.score)
            elif self.det_class_id >= 0:
                class_id = self.det_class_id
                confidence = 1.0
            else:
                continue
            if class_id < 0 or (self.det_class_id >= 0 and class_id != self.det_class_id):
                continue
            center_x = float(detection.bbox.center.x)
            center_y = float(detection.bbox.center.y)
            if not (math.isfinite(center_x) and math.isfinite(center_y)
                    and math.isfinite(confidence)):
                continue
            candidates.append((confidence, center_x, center_y, class_id, tracked))

        if not candidates:
            with self._lock:
                self._rev += 1
                self._det = None
            return
        confidence, center_x, center_y, class_id, selected = max(
            candidates, key=lambda item: item[0])
        provenance = getattr(msg, 'provenance', None)
        sensor_id = str(
            getattr(provenance, 'sensor_id', '') or self.det_sensor_id
        )
        with self._lock:
            self._rev += 1
            self._det = dict(
                u=center_x,
                v=center_y,
                image_width=self.img_w,
                image_height=self.img_h,
                stamp=_stamp_or_now(msg),
                class_id=class_id,
                confidence=confidence,
                track_id=int(selected.track_id),
                track_id_is_stable=bool(selected.track_id_is_stable),
                sensor_id=sensor_id,
            )

    # =============================================================== 主循环
    def _image_offset(self, now):
        """把目标在图像中的像素偏移折算成 (alpha, beta) 弧度偏角。"""
        if not self.use_img_offset:
            return (0.0, 0.0)
        with self._lock:
            det = self._det
        if det is None:
            return (0.0, 0.0)
        if abs((now - det['stamp']).to_sec()) > self.det_timeout:
            return (0.0, 0.0)

        image_width = int(det.get('image_width', self.img_w))
        image_height = int(det.get('image_height', self.img_h))
        if image_width <= 0 or image_height <= 0:
            return (0.0, 0.0)
        cx, cy = image_width * 0.5, image_height * 0.5
        tan_half_x = math.tan(math.radians(self.fov_x_deg) * 0.5)
        tan_half_y = tan_half_x * image_height / float(image_width)
        alpha = math.atan((det['u'] - cx) * tan_half_x / cx)      # 右偏为正
        beta = math.atan((det['v'] - cy) * tan_half_y / cy)       # 下偏为正

        limit = math.radians(self.max_img_offset_deg)
        if abs(alpha) > limit or abs(beta) > limit:
            # 目标离光轴太远时，测距仪打的点和目标很可能不是同一个东西
            rospy.logdebug_throttle(2.0, '[GEO] 图像偏角超限，忽略本次偏角修正')
            return (0.0, 0.0)
        return (alpha, beta)

    def _tick(self, _event):
        now = rospy.Time.now()
        with self._lock:
            # 输入没变就不重复解算/发布：定时器频率高于传感器频率时，这里能省掉
            # 大部分空转（例如 10 Hz 定时器 + 5 Hz 测距，可省一半）
            if self.publish_only_on_change and self._rev == self._last_rev:
                return
            self._last_rev = self._rev
            gps, rng, gimbal, heading = self._gps, self._range, self._gimbal, self._heading

        if gps is None or rng is None or gimbal is None:
            rospy.loginfo_throttle(
                3.0, '[GEO] 等待输入: gps=%s range=%s gimbal=%s',
                gps is not None, rng is not None, gimbal is not None)
            return

        # ---------------- 时间一致性：传感器必须"同时"、且都新鲜 ----------------
        stamps = [gps['stamp'], rng['stamp'], gimbal['stamp']]
        ages = [(now - s).to_sec() for s in stamps]
        if max(ages) > self.max_input_age:
            rospy.logwarn_throttle(2.0, '[GEO] 输入过期: gps=%.2fs range=%.2fs gimbal=%.2fs',
                                   ages[0], ages[1], ages[2])
            return
        if (max(stamps) - min(stamps)).to_sec() > self.max_skew:
            rospy.logwarn_throttle(
                2.0, '[GEO] 传感器时间不同步(>%.0f ms)，本次丢弃：'
                     '飞机在动时会把位移误差算成目标误差',
                self.max_skew * 1000.0)
            return

        # ---------------- YAW_LOCK 未置位时需要罗盘航向 ----------------
        if not gimbal['yaw_locked']:
            if heading is None:
                rospy.logwarn_throttle(2.0, '[GEO] 云台 yaw 未锁定，等待罗盘航向…')
                return
            if (now - heading['stamp']).to_sec() > self.max_input_age:
                rospy.logwarn_throttle(2.0, '[GEO] 罗盘航向过期')
                return

        # ---------------- 欧拉角来源：现场合成世界系四元数 ----------------
        q_gimbal = gimbal['q']
        yaw_locked = gimbal['yaw_locked']
        if q_gimbal is None:
            if heading is None:
                rospy.logwarn_throttle(2.0, '[GEO] 云台欧拉角为机体系，需要航向才能合成姿态')
                return
            try:
                q_gimbal = self._euler_to_world_quat(gimbal['euler'], heading['deg'])
            except Exception as exc:             # noqa: BLE001
                rospy.logwarn_throttle(2.0, '[GEO] 欧拉角转换失败: %s', exc)
                return
            yaw_locked = True                    # 已抬到世界系，解算器不要再叠加航向

        # ---------------- 测距合理性 ----------------
        slant = rng['value'] + self.range_bias
        if slant <= 0.0 or slant < self.min_range or slant > self.max_range:
            rospy.logwarn_throttle(2.0, '[GEO] 测距 %.2f m 超出 [%.2f, %.2f]，丢弃',
                                   slant, self.min_range, self.max_range)
            return

        alpha, beta = self._image_offset(now)

        try:
            res = solver.solve_target(
                gps['lat'], gps['lon'], gps['alt'],
                q_gimbal, slant,
                yaw_locked=yaw_locked,
                heading_deg=None if heading is None else heading['deg'],
                boresight_rpy_deg=self.boresight_rpy,
                img_offset_rad=(alpha, beta),
                sensor_axis=self.sensor_axis)
        except Exception as exc:                 # 数值异常（退化四元数等）
            rospy.logwarn_throttle(2.0, '[GEO] 解算失败: %s', exc)
            return

        if self.require_downward and res['looking_up']:
            rospy.logwarn_throttle(2.0, '[GEO] 光轴指向地平线以上(仰角 %.1f°)，'
                                        '不是地面目标，丢弃', res['elevation_deg'])
            return
        if res['ground_range'] > self.max_ground_range:
            rospy.logwarn_throttle(2.0, '[GEO] 水平距离 %.0f m 超限，丢弃',
                                   res['ground_range'])
            return

        # ---------------- 可选平滑（对 ENU 偏移做 EMA，抑制云台抖动） ----------------
        east, north, up = res['east'], res['north'], res['up']
        if self.smooth_alpha > 0.0:
            if self._ema is None:
                self._ema = (east, north, up)
            else:
                a = self.smooth_alpha
                self._ema = (a * east + (1 - a) * self._ema[0],
                             a * north + (1 - a) * self._ema[1],
                             a * up + (1 - a) * self._ema[2])
            east, north, up = self._ema
            lat, lon, alt = geodesy.enu_to_geodetic(
                gps['lat'], gps['lon'], gps['alt'], east, north, up)
        else:
            lat, lon, alt = res['lat'], res['lon'], res['alt']

        self._last_valid = now
        self._publish(res, lat, lon, alt, east, north, up, now)

        rospy.loginfo_throttle(
            2.0, '[GEO] 目标 lat=%.7f lon=%.7f alt=%.1f | 斜距=%.1fm 平距=%.1fm '
                 '方位=%.1f° 仰角=%.1f° yaw_lock=%s',
            lat, lon, alt, res['slant'], res['ground_range'],
            res['bearing_deg'], res['elevation_deg'], yaw_locked)

    # =============================================================== 发布
    def _publish(self, res, lat, lon, alt, east, north, up, stamp):
        fix = NavSatFix()
        fix.header.stamp = stamp
        fix.header.frame_id = 'target'
        fix.latitude = lat
        fix.longitude = lon
        fix.altitude = alt
        fix.status.status = NAVSAT_STATUS_FIX
        fix.status.service = 1                       # SERVICE_GPS
        fix.position_covariance_type = COVARIANCE_TYPE_DIAGONAL_KNOWN
        # 粗略标称协方差：横向误差主要由 L*角误差 决定，这里给量级参考
        sigma_h = max(1.0, 0.02 * abs(res['slant']))
        fix.position_covariance[0] = sigma_h * sigma_h
        fix.position_covariance[4] = sigma_h * sigma_h
        fix.position_covariance[8] = 2.0 * 2.0
        self.pub_fix.publish(fix)

        # Keep the box identity paired with the exact coordinate solution.
        # The plain NavSatFix remains available for existing consumers.
        with self._lock:
            detection = self._det
        if (detection is not None
                and 'class_id' in detection
                and int(detection['class_id']) >= 0
                and abs((stamp - detection['stamp']).to_sec()) <= self.det_timeout):
            geolocated = GeolocatedTarget()
            geolocated.fix = fix
            geolocated.class_id = int(detection['class_id'])
            geolocated.confidence = float(detection['confidence'])
            geolocated.track_id = int(detection['track_id'])
            geolocated.track_id_is_stable = bool(detection['track_id_is_stable'])
            geolocated.sensor_id = str(detection['sensor_id'])
            self.pub_geolocated.publish(geolocated)

        # 相对飞机（或本地坐标原点）的 ENU 位置，便于 RViz / 下游使用
        with self._lock:
            local = self._local
        if local is not None:
            px, py, pz = local['x'] + east, local['y'] + north, local['z'] + up
        else:
            px, py, pz = east, north, up

        pt = PointStamped()
        pt.header.stamp = stamp
        pt.header.frame_id = self.marker_frame
        pt.point.x, pt.point.y, pt.point.z = px, py, pz
        self.pub_local.publish(pt)

        if self.enable_marker:
            marker = Marker()
            marker.header.stamp = stamp
            marker.header.frame_id = self.marker_frame
            marker.ns = 'target'
            marker.id = 0
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position = pt.point
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = marker.scale.z = self.marker_scale
            marker.color.r = marker.color.g = 0.1
            marker.color.b = 0.1
            marker.color.a = 0.85
            marker.lifetime = rospy.Duration(1.0)
            self.pub_marker.publish(marker)

        arr = Float64MultiArray()
        arr.data = [lat, lon, alt,
                    east, north, up,
                    res['slant'], res['ground_range'],
                    res['bearing_deg'], res['elevation_deg'],
                    1.0 if res['looking_up'] else 0.0]
        self.pub_solution.publish(arr)

    def _on_shutdown(self):
        if self._last_valid is None:
            rospy.logwarn('[GEO] 退出：从未输出过有效解（检查话题/参数配置）')


def main():
    try:
        TargetGeolocationNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
