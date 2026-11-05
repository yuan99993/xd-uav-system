#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ros1_client —— 极简 ROS1 客户端（纯标准库），用于在没装 ROS 的机器上读远端话题。

原理：ROS1 的 TCPROS 是"订阅方主动连发布方"的单条 TCP 连接，所以只需要
  1) 问 master 要发布节点 URI（lookupNode）
  2) 调发布节点的 requestTopic 拿 (host, port)
  3) TCP 连上去，发连接头（callerid/topic/type/md5sum=*），之后就是
     4 字节长度 + 裸消息体 的流。

只实现了本项目需要的几种消息的反序列化（ROS1 序列化不加对齐填充）。
"""

import socket
import struct
import xmlrpc.client

CALLER_ID = '/geolocate_probe'


# --------------------------------------------------------------- 主题发现
class Master(object):
    def __init__(self, uri):
        self.uri = uri
        self.proxy = xmlrpc.client.ServerProxy(uri, allow_none=True)

    def topics(self):
        code, _msg, val = self.proxy.getSystemState(CALLER_ID)
        _c, _m, types = self.proxy.getTopicTypes(CALLER_ID)
        tmap = dict(types)
        out = {}
        for topic, nodes in val[0]:
            out[topic] = dict(type=tmap.get(topic, '?'), publishers=list(nodes))
        return out

    def publisher_uri(self, node_name):
        code, _msg, uri = self.proxy.lookupNode(CALLER_ID, node_name)
        if code != 1:
            raise RuntimeError('lookupNode(%s) 失败: %s' % (node_name, _msg))
        return uri

    def subscribe(self, topic, timeout=6.0, max_msgs=1, host_override=None):
        """订阅一个话题，返回解析后的消息列表（最多 max_msgs 条）。"""
        info = self.topics().get(topic)
        if not info:
            raise RuntimeError('话题不存在: %s' % topic)
        if not info['publishers']:
            raise RuntimeError('话题无发布者: %s' % topic)
        mtype = info['type']
        parser = _PARSERS.get(mtype)
        if parser is None:
            raise RuntimeError('未实现的消息类型: %s (%s)' % (mtype, topic))

        uri = self.publisher_uri(info['publishers'][0])
        pub = xmlrpc.client.ServerProxy(uri, allow_none=True)
        code, msg, proto = pub.requestTopic(CALLER_ID, topic, [['TCPROS']])
        if code != 1:
            raise RuntimeError('requestTopic 失败: %s' % msg)
        host, port = proto[1], proto[2]
        if host_override and host in ('127.0.0.1', 'localhost', '0.0.0.0'):
            host = host_override

        sock = socket.create_connection((host, port), timeout=timeout)
        try:
            _send_header(sock, {
                'callerid': CALLER_ID,
                'topic': topic,
                'type': mtype,
                'md5sum': '*',            # 通配，免去查 md5
                'tcp_nodelay': '0',
            })
            fields, payload = _read_header(sock)
            if 'error' in fields:
                raise RuntimeError('订阅被拒: %s' % fields['error'])

            msgs = []
            buf = b''
            deadline = _now() + timeout
            while len(msgs) < max_msgs and _now() < deadline:
                while len(buf) < 4:
                    chunk = sock.recv(65536)
                    if not chunk:
                        raise RuntimeError('连接被对端关闭')
                    buf += chunk
                (n,) = struct.unpack('<I', buf[:4])
                while len(buf) < 4 + n:
                    chunk = sock.recv(65536)
                    if not chunk:
                        raise RuntimeError('连接被对端关闭')
                    buf += chunk
                body = buf[4:4 + n]
                buf = buf[4 + n:]
                msg = parser(body)
                msg['_t'] = _now()      # 本地接收时刻，用于多路对齐（消息本身可能无 header）
                msgs.append(msg)
            return msgs
        finally:
            sock.close()


def _now():
    import time
    return time.time()


def _send_header(sock, fields):
    blob = b''
    for k, v in fields.items():
        item = ('%s=%s' % (k, v)).encode('utf-8')
        blob += struct.pack('<I', len(item)) + item
    sock.sendall(struct.pack('<I', len(blob)) + blob)


def _read_header(sock):
    raw = _recv_exact(sock, 4)
    (n,) = struct.unpack('<I', raw)
    blob = _recv_exact(sock, n)
    fields, off = {}, 0
    while off < len(blob):
        (ln,) = struct.unpack('<I', blob[off:off + 4])
        off += 4
        kv = blob[off:off + ln].decode('utf-8', 'replace')
        off += ln
        if '=' in kv:
            k, v = kv.split('=', 1)
            fields[k] = v
    return fields, blob


def _recv_exact(sock, n):
    buf = b''
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise RuntimeError('连接被对端关闭')
        buf += chunk
    return buf


# ------------------------------------------------------------------ 反序列化
class _Reader(object):
    def __init__(self, data):
        self.d = data
        self.o = 0

    def unpack(self, fmt, size):
        v = struct.unpack_from('<' + fmt, self.d, self.o)
        self.o += size
        return v

    def u8(self):
        return self.unpack('B', 1)[0]

    def i8(self):
        return self.unpack('b', 1)[0]

    def u16(self):
        return self.unpack('H', 2)[0]

    def u32(self):
        return self.unpack('I', 4)[0]

    def f32(self):
        return self.unpack('f', 4)[0]

    def f64(self):
        return self.unpack('d', 8)[0]

    def string(self):
        (n,) = self.unpack('I', 4)
        s = self.d[self.o:self.o + n].decode('utf-8', 'replace')
        self.o += n
        return s

    def header(self):
        seq = self.u32()
        sec = self.u32()
        nsec = self.u32()
        frame = self.string()
        return dict(seq=seq, stamp=sec + nsec * 1e-9, frame_id=frame)

    def floats(self, k):
        return list(self.unpack('%dd' % k, 8 * k))


def _parse_navsatfix(body):
    r = _Reader(body)
    h = r.header()
    status = r.i8()
    service = r.u16()
    lat = r.f64()
    lon = r.f64()
    alt = r.f64()
    cov = r.floats(9)
    cov_type = r.u8()
    return dict(kind='NavSatFix', header=h, status=status, service=service,
                lat=lat, lon=lon, alt=alt, cov=cov, cov_type=cov_type)


def _parse_range(body):
    r = _Reader(body)
    h = r.header()
    radiation = r.u8()
    fov = r.f32()
    min_r = r.f32()
    max_r = r.f32()
    value = r.f32()
    return dict(kind='Range', header=h, radiation_type=radiation, fov=fov,
                min_range=min_r, max_range=max_r, range=value)


def _parse_imu(body):
    r = _Reader(body)
    h = r.header()
    q = r.floats(4)                  # x, y, z, w
    r.floats(9)                      # orientation_covariance
    r.floats(3)                      # angular_velocity
    r.floats(9)                      # angular_velocity_covariance
    r.floats(3)                      # linear_acceleration
    r.floats(9)                      # linear_acceleration_covariance
    return dict(kind='Imu', header=h, q_xyzw=q)


def _parse_posestamped(body):
    r = _Reader(body)
    h = r.header()
    pos = r.floats(3)
    ori = r.floats(4)                # x, y, z, w
    return dict(kind='PoseStamped', header=h, pos=pos, q_xyzw=ori)


_PARSERS = {
    'sensor_msgs/NavSatFix': _parse_navsatfix,
    'sensor_msgs/Range': _parse_range,
    'sensor_msgs/Imu': _parse_imu,
    'geometry_msgs/PoseStamped': _parse_posestamped,
}


def _parse_pointcloud2(body):
    """PointCloud2 -> 只取出 x/y/z 三个 float32 字段，返回点列表。"""
    r = _Reader(body)
    h = r.header()
    height = r.u32()
    width = r.u32()
    nfields = r.u32()
    fields = []
    for _ in range(nfields):
        name = r.string()
        off = r.u32()
        dtype = r.u8()
        count = r.u32()
        fields.append(dict(name=name, offset=off, datatype=dtype, count=count))
    r.u8()                                        # is_bigendian
    point_step = r.u32()
    r.u32()                                       # row_step
    (nbytes,) = r.unpack('I', 4)
    data = r.d[r.o:r.o + nbytes]
    r.o += nbytes

    idx = {f['name']: f['offset'] for f in fields if f['name'] in ('x', 'y', 'z')}
    if len(idx) < 3:
        return dict(kind='PointCloud2', header=h, points=[])
    pts = []
    for i in range(height * width):
        base = i * point_step
        if base + point_step > len(data):
            break
        x, y, z = struct.unpack_from('<fff', data, base + idx['x'])
        pts.append((x, y, z))
    return dict(kind='PointCloud2', header=h, points=pts)


_PARSERS['sensor_msgs/PointCloud2'] = _parse_pointcloud2


def _parse_float64multiarray(body):
    r = _Reader(body)
    ndim = r.u32()
    dims = []
    for _ in range(ndim):
        label = r.string()
        size = r.u32()
        dims.append((label, size))
    data_off = r.u32()
    (n,) = r.unpack('I', 4)
    data = list(r.unpack('%dd' % n, 8 * n))
    return dict(kind='Float64MultiArray', dims=dims, data_offset=data_off, data=data)


def _parse_quaternion(body):
    r = _Reader(body)
    return dict(kind='Quaternion', q_xyzw=r.floats(4))


_PARSERS['std_msgs/Float64MultiArray'] = _parse_float64multiarray
_PARSERS['geometry_msgs/Quaternion'] = _parse_quaternion


def describe(uri, topic, caller_id=CALLER_ID):
    """连上发布方，返回连接头（含 message_definition），用于未知自定义消息的探查。"""
    m = Master(uri)
    info = m.topics()[topic]
    node = info['publishers'][0]
    pub = xmlrpc.client.ServerProxy(m.publisher_uri(node), allow_none=True)
    code, msg, proto = pub.requestTopic(caller_id, topic, [['TCPROS']])
    if code != 1:
        raise RuntimeError(msg)
    host = proto[1]
    if host in ('127.0.0.1', 'localhost', '0.0.0.0'):
        host = uri.split('//')[1].split(':')[0]
    sock = socket.create_connection((host, proto[2]), timeout=5.0)
    try:
        _send_header(sock, {'callerid': caller_id, 'topic': topic,
                            'type': info['type'], 'md5sum': '*', 'tcp_nodelay': '0'})
        fields, _ = _read_header(sock)
        return fields
    finally:
        sock.close()


def _parse_gimbal_state(body):
    r = _Reader(body)
    h = r.header()
    yaw, pitch, roll = r.floats(3)
    valid = r.u8()
    return dict(kind='GimbalState', header=h, yaw_rad=yaw, pitch_rad=pitch,
                roll_rad=roll, valid=bool(valid))


def _parse_debug_value(body):
    r = _Reader(body)
    h = r.header()
    key = r.string()
    val = r.f64()
    return dict(kind='DebugValue', header=h, key=key, value=val)


_PARSERS['xd_uav_track/GimbalState'] = _parse_gimbal_state
_PARSERS['mavros_msgs/DebugValue'] = _parse_debug_value


def _parse_float64(body):
    r = _Reader(body)
    return dict(kind='Float64', data=r.f64())


def _parse_float32(body):
    r = _Reader(body)
    return dict(kind='Float32', data=r.f32())


def _parse_vector3stamped(body):
    r = _Reader(body)
    h = r.header()
    return dict(kind='Vector3Stamped', header=h, v=r.floats(3))


_PARSERS['std_msgs/Float64'] = _parse_float64
_PARSERS['std_msgs/Float32'] = _parse_float32
_PARSERS['geometry_msgs/Vector3Stamped'] = _parse_vector3stamped


def _parse_mavlink(body):
    """mavros_msgs/Mavlink -> 只保留 msgid 与 payload 原始字节（不解帧）。

    注意字段顺序（Noetic 版）：header, framing_status, magic, len, incompat_flags,
    compat_flags, seq, sysid, compid, msgid, checksum, payload64[], signature[]。
    checksum 在 payload64 之前，且多一个 framing_status，容易漏。
    """
    r = _Reader(body)
    h = r.header()
    r.u8()                                   # framing_status
    magic = r.u8()
    length = r.u8()
    incompat = r.u8()
    compat = r.u8()
    seq = r.u8()
    sysid = r.u8()
    compid = r.u8()
    msgid = r.u32()
    r.u16()                                  # checksum
    (nwords,) = r.unpack('I', 4)
    words = r.unpack('%dQ' % nwords, 8 * nwords) if nwords else ()
    payload = b''.join(struct.pack('<Q', w) for w in words)[:length]
    return dict(kind='Mavlink', header=h, magic=magic, length=length,
                incompat=incompat, seq=seq, sysid=sysid, compid=compid,
                msgid=msgid, payload=payload)


_PARSERS['mavros_msgs/Mavlink'] = _parse_mavlink
