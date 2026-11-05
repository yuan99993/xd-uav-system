#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""test_node_dryrun —— 不装 ROS 也能"干跑"整个节点，验证话题/参数/消息字段/门限逻辑。

    python3 test/test_node_dryrun.py -v

做法：往 sys.modules 里注入一套最小 rospy / 消息桩，然后把节点脚本当模块导入，
按真实顺序喂 GPS、云台姿态、测距三路数据，调用一次 _tick()，检查发布出来的
NavSatFix 是否等于解析解。这能抓出话题名拼错、消息字段名写错、时间门限写错这类
只有跑起来才会暴露的问题（本机没有 ROS 环境，靠它替代 rosrun 冒烟测试）。
"""

import importlib.util
import math
import os
import struct
import sys
import types
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.join(HERE, '..')
sys.path.insert(0, os.path.join(PKG, 'src'))

from target_geolocation import geodesy, solver  # noqa: E402


# ============================================================ 最小 ROS 桩
class Duration(object):
    def __init__(self, secs=0.0):
        self._s = float(secs)

    def to_sec(self):
        return self._s


class Time(object):
    _clock = 1000.0

    def __init__(self, secs=None):
        self._s = Time._clock if secs is None else float(secs)

    @staticmethod
    def now():
        Time._clock += 0.05
        return Time(Time._clock)

    def to_sec(self):
        return self._s

    def __sub__(self, other):
        return Duration(self._s - other.to_sec())

    def __gt__(self, other):
        return self._s > other.to_sec()

    def __lt__(self, other):
        return self._s < other.to_sec()


class _Msg(object):
    def __init__(self, **kw):
        for k, v in kw.items():
            setattr(self, k, v)


class Header(object):
    def __init__(self, stamp=None, frame_id=''):
        self.stamp = stamp if stamp is not None else Time(0.0)
        self.frame_id = frame_id


class Point(object):
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x, self.y, self.z = x, y, z


class Quaternion(object):
    def __init__(self, x=0.0, y=0.0, z=0.0, w=1.0):
        self.x, self.y, self.z, self.w = x, y, z, w


class _Status(object):
    def __init__(self):
        self.status = -1
        self.service = 0


class NavSatFix(object):
    def __init__(self):
        self.header = Header()
        self.status = _Status()
        self.latitude = 0.0
        self.longitude = 0.0
        self.altitude = 0.0
        self.position_covariance = [0.0] * 9
        self.position_covariance_type = 0


class Range(object):
    INFRARED = 1

    def __init__(self):
        self.header = Header()
        self.radiation_type = 0
        self.field_of_view = 0.0
        self.min_range = 0.0
        self.max_range = 0.0
        self.range = 0.0


class QuaternionStamped(object):
    def __init__(self):
        self.header = Header()
        self.quaternion = Quaternion()


class Imu(object):
    def __init__(self):
        self.header = Header()
        self.orientation = Quaternion()


class PointStamped(object):
    def __init__(self):
        self.header = Header()
        self.point = Point()


class PoseStamped(object):
    def __init__(self):
        self.header = Header()
        self.pose = _Msg(position=Point(), orientation=Quaternion())


class Float32(object):
    def __init__(self, data=0.0):
        self.data = data


class Float64(Float32):
    pass


class Float64MultiArray(object):
    def __init__(self):
        self.data = []


class Marker(object):
    SPHERE, CUBE, ARROW, ADD, DELETE = 2, 1, 0, 0, 2

    def __init__(self):
        self.header = Header()
        self.ns = ''
        self.id = 0
        self.type = 0
        self.action = 0
        self.pose = _Msg(position=Point(), orientation=Quaternion())
        self.scale = Point(1.0, 1.0, 1.0)
        self.color = _Msg(r=0.0, g=0.0, b=0.0, a=1.0)
        self.lifetime = Duration(0.0)


class Mavlink(object):
    def __init__(self):
        self.header = Header()
        self.payload64 = []
        self.len = 0
        self.msgid = 0
        self.magic = 253
        self.incompat_flags = 0
        self.compat_flags = 0
        self.seq = 0
        self.sysid = 1
        self.compid = 1
        self.checksum = 0
        self.signature = []


class BoundingBox2D(object):
    def __init__(self):
        self.center = _Msg(position=Point())


class Publisher(object):
    def __init__(self, topic, msg_type, **kw):
        self.topic = topic
        self.msg_type = msg_type
        self.published = []

    def publish(self, msg):
        self.published.append(msg)


class Subscriber(object):
    REGISTRY = []

    def __init__(self, topic, msg_type, callback, **kw):
        self.topic = topic
        self.msg_type = msg_type
        self.callback = callback
        Subscriber.REGISTRY.append(self)


ROSPY_PARAMS = {}


def _install_ros_stubs():
    rospy = types.ModuleType('rospy')
    rospy.Time = Time
    rospy.Duration = Duration
    rospy.Publisher = Publisher
    rospy.Subscriber = Subscriber
    rospy.ROSInterruptException = type('ROSInterruptException', (Exception,), {})

    def _noop(*a, **k):
        return None

    for name in ('init_node', 'spin', 'on_shutdown', 'is_shutdown',
                 'loginfo', 'logwarn', 'logerr', 'logdebug',
                 'loginfo_throttle', 'logwarn_throttle', 'logerr_throttle',
                 'logdebug_throttle', 'signal_shutdown'):
        setattr(rospy, name, _noop)
    rospy.get_param = lambda name, default=None: ROSPY_PARAMS.get(name, default)
    rospy.set_param = lambda name, value: ROSPY_PARAMS.__setitem__(name, value)

    class _Timer(object):
        def __init__(self, duration, callback, **kw):
            self.duration = duration
            self.callback = callback

    rospy.Timer = _Timer
    sys.modules['rospy'] = rospy

    for name, classes in (
            ('sensor_msgs', {'NavSatFix': NavSatFix, 'Range': Range, 'Imu': Imu}),
            ('sensor_msgs.msg', {'NavSatFix': NavSatFix, 'Range': Range, 'Imu': Imu}),
            ('geometry_msgs', {}),
            ('geometry_msgs.msg', {'PointStamped': PointStamped,
                                   'PoseStamped': PoseStamped,
                                   'QuaternionStamped': QuaternionStamped}),
            ('std_msgs', {}),
            ('std_msgs.msg', {'Float32': Float32, 'Float64': Float64,
                              'Float64MultiArray': Float64MultiArray}),
            ('visualization_msgs', {}),
            ('visualization_msgs.msg', {'Marker': Marker}),
            ('mavros_msgs', {}),
            ('mavros_msgs.msg', {'Mavlink': Mavlink}),
            ('vision_msgs', {}),
            ('vision_msgs.msg', {'BoundingBox2D': BoundingBox2D}),
    ):
        mod = types.ModuleType(name)
        for cls_name, cls in classes.items():
            setattr(mod, cls_name, cls)
        sys.modules[name] = mod


def _load_node_module():
    path = os.path.join(PKG, 'scripts', 'target_geolocation_node.py')
    spec = importlib.util.spec_from_file_location('tgeo_node', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _make_msgs(uav, target, sigma_deg=0.0):
    """按几何构造三路输入消息（正向模型，与 solver 的符号约定互相独立）。"""
    lat, lon, alt = uav
    t_lat, t_lon, t_alt = target
    east, north = geodesy.geodetic_to_enu(lat, lon, t_lat, t_lon)
    up = t_alt - alt
    slant = math.sqrt(east * east + north * north + up * up)
    yaw = math.degrees(math.atan2(east, north)) + sigma_deg
    pitch = math.degrees(math.asin(up / slant))
    q = solver.quat_from_euler_zyx(0.0, math.radians(pitch), math.radians(yaw))

    stamp = Time(1000.0)
    gps = NavSatFix()
    gps.header.stamp = stamp
    gps.latitude, gps.longitude, gps.altitude = lat, lon, alt
    gps.status.status = 0
    rng = Range()
    rng.header.stamp = stamp
    rng.range = slant
    att = QuaternionStamped()
    att.header.stamp = stamp
    att.quaternion.w, att.quaternion.x, att.quaternion.y, att.quaternion.z = q
    return gps, rng, att, slant


UAV = (47.397742, 8.545594, 488.0)
OFFSET_E, OFFSET_N, OFFSET_U = 300.0 * math.sin(math.radians(60)), \
    300.0 * math.cos(math.radians(60)), -120.0
TARGET = geodesy.enu_to_geodetic(*UAV, OFFSET_E, OFFSET_N, OFFSET_U)


class TestNodeDryRun(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        _install_ros_stubs()
        cls.mod = _load_node_module()

    def setUp(self):
        Time._clock = 1000.0                     # 每个用例从同一时刻开始
        ROSPY_PARAMS.clear()
        ROSPY_PARAMS.update({
            '~gimbal_att_topic': '/sim/gimbal/attitude',
            '~range_topic': '/sim/range',
            '~use_img_offset': False,
            '~smooth_alpha': 0.0,
        })
        Subscriber.REGISTRY = []
        self.node = self.mod.TargetGeolocationNode()

    def test_topic_defaults_and_wiring(self):
        topics = [s.topic for s in Subscriber.REGISTRY]
        self.assertIn('/uav1/mavros/global_position/global', topics)
        self.assertIn('/uav1/mavros/global_position/compass_hdg', topics)
        self.assertIn('/sim/range', topics)
        self.assertIn('/sim/gimbal/attitude', topics)
        # 空字符串参数不能把输出前缀吃掉
        self.assertEqual(self.node.out_prefix, '/uav1/target')
        self.assertEqual(self.node.pub_fix.topic, '/uav1/target/global_position')

    def test_solution_matches_analytic(self):
        gps, rng, att, slant = _make_msgs(UAV, TARGET)
        self.node._gps_cb(gps)
        self.node._range_cb(rng)
        self.node._gimbal_att_cb(att)
        self.node._tick(None)

        self.assertEqual(len(self.node.pub_fix.published), 1, '未输出解')
        fix = self.node.pub_fix.published[-1]
        self.assertAlmostEqual(fix.latitude, TARGET[0], delta=1e-7)
        self.assertAlmostEqual(fix.longitude, TARGET[1], delta=1e-7)
        self.assertAlmostEqual(fix.altitude, TARGET[2], delta=1e-3)
        self.assertEqual(fix.status.status, 0)
        self.assertEqual(fix.position_covariance_type, 1)

        self.assertEqual(len(self.node.pub_solution.published[-1].data), 11)
        marker = self.node.pub_marker.published[-1]
        self.assertEqual(marker.header.frame_id, 'map')
        self.assertAlmostEqual(self.node.pub_solution.published[-1].data[5],
                               OFFSET_U, delta=1e-3)

    def test_range_out_of_bounds_is_rejected(self):
        gps, rng, att, _ = _make_msgs(UAV, TARGET)
        self.node._gps_cb(gps)
        self.node._gimbal_att_cb(att)
        rng.range = 1.0e6
        self.node._range_cb(rng)
        self.node._tick(None)
        self.assertEqual(len(self.node.pub_fix.published), 0)

    def test_missing_heading_with_unlocked_yaw(self):
        """YAW_LOCK=0 且没有罗盘航向 -> 必须拒绝出解，而不是用 0 航向硬算。"""
        gps, rng, att, _ = _make_msgs(UAV, TARGET)
        self.node._gps_cb(gps)
        self.node._range_cb(rng)
        self.node._gimbal_att_cb(att)
        self.node._gimbal['yaw_locked'] = False
        self.node._tick(None)
        self.assertEqual(len(self.node.pub_fix.published), 0)

        hdg = Float64()
        hdg.data = 30.0
        self.node._heading_cb(hdg)
        self.node._tick(None)
        self.assertEqual(len(self.node.pub_fix.published), 1)
        # 机体系 yaw + 航向 30° -> 目标方位应比原来多 30°
        self.assertAlmostEqual(self.node.pub_solution.published[-1].data[8], 90.0, delta=1e-3)

    def test_imu_attitude_source(self):
        """用 sensor_msgs/Imu 当姿态源（AirSim 这类无云台的机体）应给出同样结果。"""
        ROSPY_PARAMS['~att_msg_type'] = 'Imu'
        node = self.mod.TargetGeolocationNode()
        gps, rng, att, _ = _make_msgs(UAV, TARGET)
        imu = Imu()
        imu.header.stamp = att.header.stamp
        imu.orientation = att.quaternion
        node._gps_cb(gps)
        node._range_cb(rng)
        node._imu_att_cb(imu)
        node._tick(None)
        fix = node.pub_fix.published[-1]
        self.assertAlmostEqual(fix.latitude, TARGET[0], delta=1e-7)
        self.assertAlmostEqual(fix.longitude, TARGET[1], delta=1e-7)
        self.assertAlmostEqual(fix.altitude, TARGET[2], delta=1e-3)

    # ------------------------------ 欧拉角云台姿态：/uav1/gimbal/attitude 这条真实链路
    def _euler_setup(self, yaw_is_body=True, heading_deg=282.47):
        """按 /uav1/gimbal/attitude 的真实约定造输入。

        yaw_is_body=True 时话题里给的是 (世界方位 - 航向)，模拟实测得到的
        bridge_yaw = world_yaw - heading 关系；pitch 用"上正"。
        """
        ROSPY_PARAMS['~att_msg_type'] = 'Float64MultiArray'
        ROSPY_PARAMS['~gimbal_yaw_is_body'] = yaw_is_body
        node = self.mod.TargetGeolocationNode()

        gps, rng, att, _ = _make_msgs(UAV, TARGET)
        east, north = geodesy.geodetic_to_enu(UAV[0], UAV[1], TARGET[0], TARGET[1])
        world_yaw = math.degrees(math.atan2(east, north))
        up = TARGET[2] - UAV[2]
        slant = math.sqrt(east ** 2 + north ** 2 + up ** 2)
        elev = math.degrees(math.asin(up / slant))

        hdg = Float64(heading_deg)
        eul = Float64MultiArray()
        eul.data = [(world_yaw - heading_deg) % 360.0 if yaw_is_body else world_yaw,
                    elev, 0.0]

        node._gps_cb(gps)
        node._range_cb(rng)
        node._heading_cb(hdg)
        node._gimbal_euler_cb(eul)
        return node

    def test_euler_body_yaw_needs_heading(self):
        """[yaw(机体系), pitch(上正), roll] + 航向 应能复原目标。"""
        node = self._euler_setup(yaw_is_body=True, heading_deg=282.47)
        node._tick(None)
        fix = node.pub_fix.published[-1]
        self.assertAlmostEqual(fix.latitude, TARGET[0], delta=1e-7)
        self.assertAlmostEqual(fix.longitude, TARGET[1], delta=1e-7)
        self.assertAlmostEqual(fix.altitude, TARGET[2], delta=1e-3)

    def test_euler_world_yaw_ignores_heading(self):
        """~gimbal_yaw_is_body:=false 时 yaw 已是世界系，航向不能再叠加。"""
        node = self._euler_setup(yaw_is_body=False, heading_deg=282.47)
        node._tick(None)
        fix = node.pub_fix.published[-1]
        self.assertAlmostEqual(fix.latitude, TARGET[0], delta=1e-7)
        self.assertAlmostEqual(fix.longitude, TARGET[1], delta=1e-7)

    def test_euler_wrong_yaw_flag_shifts_bearing(self):
        """把机体系 yaw 误当世界系，目标会偏到另一个方位角（差 = 航向）。"""
        node = self._euler_setup(yaw_is_body=True, heading_deg=282.47)
        node.gimbal_yaw_is_body = False          # 故意用错约定
        node._tick(None)
        fix = node.pub_fix.published[-1]
        self.assertGreater(geodesy.horizontal_distance(
            TARGET[0], TARGET[1], fix.latitude, fix.longitude), 50.0)

    def test_euler_pitch_down_positive_flag(self):
        """~gimbal_pitch_up_positive:=false 时，下正输入应给出同一条光轴。"""
        ROSPY_PARAMS['~att_msg_type'] = 'Float64MultiArray'
        ROSPY_PARAMS['~gimbal_pitch_up_positive'] = False
        node = self.mod.TargetGeolocationNode()
        east, north = geodesy.geodetic_to_enu(UAV[0], UAV[1], TARGET[0], TARGET[1])
        up = TARGET[2] - UAV[2]
        slant = math.sqrt(east ** 2 + north ** 2 + up ** 2)
        world_yaw = math.degrees(math.atan2(east, north))
        elev = math.degrees(math.asin(up / slant))
        gps, rng, _, _ = _make_msgs(UAV, TARGET)
        eul = Float64MultiArray()
        eul.data = [(world_yaw - 10.0) % 360.0, -elev, 0.0]   # 下正 = -elev
        node._gps_cb(gps)
        node._range_cb(rng)
        node._heading_cb(Float64(10.0))
        node._gimbal_euler_cb(eul)
        node._tick(None)
        fix = node.pub_fix.published[-1]
        self.assertAlmostEqual(fix.latitude, TARGET[0], delta=1e-7)
        self.assertAlmostEqual(fix.longitude, TARGET[1], delta=1e-7)

    def test_range_bias_extends_range(self):
        """~range_bias 应把测距加上去（抵消"激光停在靶标表面"造成的偏短）。"""
        ROSPY_PARAMS['~range_bias'] = 2.0
        node = self.mod.TargetGeolocationNode()
        gps, rng, att, slant = _make_msgs(UAV, TARGET)
        rng.range = slant - 2.0                   # 模拟测得偏短 2 m
        node._gps_cb(gps)
        node._range_cb(rng)
        node._gimbal_att_cb(att)
        node._tick(None)
        fix = node.pub_fix.published[-1]
        self.assertAlmostEqual(fix.latitude, TARGET[0], delta=1e-7)
        self.assertAlmostEqual(fix.longitude, TARGET[1], delta=1e-7)

    # ------------------------------ #285 快路径：先按 msgid 过滤，再按字节解
    def _mk_mavlink(self, msgid, q, flags=0, payload_len=37):
        """造一条 mavros_msgs/Mavlink（#285 的布局：time u32 + q[4]f + ... + flags u16）。"""
        m = Mavlink()
        m.header.stamp = Time(1000.0)
        m.msgid = msgid
        m.len = payload_len
        body = struct.pack('<I', 1234) + struct.pack('<4f', *q)
        body += struct.pack('<BB', 1, 1) + struct.pack('<H', flags)
        body += b'\x00' * max(0, payload_len - len(body))
        body = body[:payload_len]
        # payload64 是 8 字节对齐的原始载荷
        pad = (-len(body)) % 8
        body += b'\x00' * pad
        m.payload64 = list(struct.unpack('<%dQ' % (len(body) // 8), body))
        return m

    def test_mavlink_285_fast_path(self):
        """直接喂 #285 原始帧，应解出与四元数源一致的结果。"""
        ROSPY_PARAMS['~gimbal_att_topic'] = ''
        node = self.mod.TargetGeolocationNode()
        gps, rng, att, _ = _make_msgs(UAV, TARGET)
        qa = att.quaternion
        m = self._mk_mavlink(285, (qa.w, qa.x, qa.y, qa.z), flags=16)
        node._gps_cb(gps)
        node._range_cb(rng)
        node._mavlink_cb(m)
        self.assertIsNotNone(node._gimbal)
        self.assertTrue(node._gimbal['yaw_locked'])
        node._tick(None)
        fix = node.pub_fix.published[-1]
        self.assertAlmostEqual(fix.latitude, TARGET[0], delta=1e-7)
        self.assertAlmostEqual(fix.longitude, TARGET[1], delta=1e-7)

    def test_mavlink_ignores_other_msgids(self):
        """非 #285 的帧必须原样跳过：不解码、不改状态、不报错。"""
        ROSPY_PARAMS['~gimbal_att_topic'] = ''
        node = self.mod.TargetGeolocationNode()
        for mid, ln in ((24, 30), (30, 28), (132, 16), (0, 0), (285 << 8, 40)):
            node._mavlink_cb(self._mk_mavlink(mid, (1, 0, 0, 0), payload_len=ln))
        self.assertIsNone(node._gimbal)

    def test_mavlink_short_payload_is_safe(self):
        """截断帧（MAVLink2 去尾零后只剩 20 字节）不能崩，只能丢弃。"""
        ROSPY_PARAMS['~gimbal_att_topic'] = ''
        node = self.mod.TargetGeolocationNode()
        node._mavlink_cb(self._mk_mavlink(285, (1, 0, 0, 0), payload_len=20))
        self.assertIsNone(node._gimbal)

    def test_stale_input_is_rejected(self):
        gps, rng, att, _ = _make_msgs(UAV, TARGET)
        self.node._gps_cb(gps)
        self.node._range_cb(rng)
        self.node._gimbal_att_cb(att)
        Time._clock += 5.0                       # 模拟 5 s 没数据
        self.node._tick(None)
        self.assertEqual(len(self.node.pub_fix.published), 0)


def _load_script(name):
    path = os.path.join(PKG, 'scripts', name)
    spec = importlib.util.spec_from_file_location('tgeo_' + name.split('.')[0], path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class TestSimSensorClosedLoop(unittest.TestCase):
    """把 sim_target_sensor 的正向输出直接喂给节点，检查能否复原它锚定的真值。"""

    @classmethod
    def setUpClass(cls):
        _install_ros_stubs()
        cls.node_mod = _load_node_module()
        cls.sim_mod = _load_script('sim_target_sensor.py')

    def test_closed_loop(self):
        Time._clock = 1000.0                     # 仿真传感器用 now() 打时间戳
        ROSPY_PARAMS.clear()
        ROSPY_PARAMS.update({
            '~truth_bearing_deg': 135.0,
            '~truth_distance_m': 250.0,
            '~truth_height_agl': 80.0,
            '~att_noise_deg': 0.0,
            '~range_noise_std': 0.0,
        })
        sensor = self.sim_mod.SimTargetSensor()
        gps = NavSatFix()
        gps.header.stamp = Time(Time._clock)     # 与传感器输出同一时刻，模拟真机
        gps.latitude, gps.longitude, gps.altitude = UAV
        gps.status.status = 0                    # NavSatStatus.FIX
        sensor._gps_cb(gps)
        sensor._tick(None)

        truth = sensor.pub_truth.published[-1]
        rng = sensor.pub_range.published[-1]
        att = sensor.pub_att.published[-1]

        ROSPY_PARAMS.clear()
        ROSPY_PARAMS.update({'~gimbal_att_topic': '/sim/gimbal/attitude',
                             '~use_img_offset': False, '~smooth_alpha': 0.0})
        node = self.node_mod.TargetGeolocationNode()
        node._gps_cb(gps)
        node._range_cb(rng)
        node._gimbal_att_cb(att)
        node._tick(None)

        fix = node.pub_fix.published[-1]
        self.assertAlmostEqual(fix.latitude, truth.latitude, delta=1e-7)
        self.assertAlmostEqual(fix.longitude, truth.longitude, delta=1e-7)
        self.assertAlmostEqual(fix.altitude, truth.altitude, delta=1e-3)
        # 真值锚点：250 m 距离、低 80 m
        self.assertAlmostEqual(node.pub_solution.published[-1].data[7], 250.0, delta=1e-3)
        self.assertAlmostEqual(node.pub_solution.published[-1].data[5], -80.0, delta=1e-6)


if __name__ == '__main__':
    unittest.main(verbosity=2)
