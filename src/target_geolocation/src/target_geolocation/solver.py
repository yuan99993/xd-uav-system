#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""目标地理定位解算核心（与 ROS 完全解耦，可在 PC 上直接单元测试）。

物理模型
--------
飞机在 P0(lat0, lon0, alt0)，云台光轴（激光测距仪/相机共轴）指向单位向量
d（ENU 系），测距仪给出沿光轴的斜距 L。目标位置：

    P_target = P0 + L * d      （在局部 ENU 切平面内平移，再投回 WGS84）

所以精度取决于三件事：
  1. 光轴方向 d 的角精度（云台姿态 + 安装偏差 + 图像偏角）；
  2. 斜距 L 的精度；
  3. 飞机自身 GPS 的精度。
横向误差 ≈ L * 角误差：L=300 m 时 1° 姿态误差 → 5.2 m，所以云台姿态必须用
实时反馈而不是指令值。

坐标与符号约定（与 PX4 / MAVLink 一致，务必先确认再上机）
--------------------------------------------------------
* 云台姿态来自 GIMBAL_DEVICE_ATTITUDE_STATUS(#285) 的 q = 机体系(FRD, x前 y右
  z下) -> 世界系(NED, x北 y东 z下) 的旋转四元数，顺序 [w,x,y,z]。
* #285 的 flags 含 GIMBAL_DEVICE_FLAGS_YAW_LOCK(16)：
    - 置位：q 的 yaw 已是相对正北的世界系角 —— 直接可用；
    - 未置位：q 的 yaw 是相对机头的角度 —— 必须叠加罗盘航向。
* 光轴默认取机体系 +X（云台"正前方"，镜头/激光朝外的那一轴）。pitch = -90°
  表示垂直下视，此时 +X 与 NED 的 +Z（向下）重合，与实机一致。
* yaw 采用罗盘航向习惯：0°=正北，90°=正东，顺时针为正。
"""

import math

from . import geodesy

GIMBAL_DEVICE_FLAGS_YAW_LOCK = 16

#: 若光轴不是机体系 +X，用固定旋转把它搬到 +X，便于统一按 (1, tanα, tanβ) 建模。
#: 'y'：绕 z 转 -90°，'z'：绕 y 转 +90°。
_SENSOR_AXIS_FIX = {
    'x': (1.0, 0.0, 0.0,
          0.0, 1.0, 0.0,
          0.0, 0.0, 1.0),
    'y': (0.0, 1.0, 0.0,
          -1.0, 0.0, 0.0,
          0.0, 0.0, 1.0),
    'z': (0.0, 0.0, 1.0,
          0.0, 1.0, 0.0,
          -1.0, 0.0, 0.0),
}


# ------------------------------------------------------------------ 基础数学
def _normalize(v):
    n = math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
    if n < 1e-12:
        raise ValueError('zero-length vector')
    return (v[0] / n, v[1] / n, v[2] / n)


def quat_to_rotmat(q):
    """四元数 [w,x,y,z] -> 3x3 旋转矩阵（按行展开的 9 元 tuple，active 旋转）。"""
    if len(q) != 4:
        raise ValueError('quaternion must be [w, x, y, z]')
    norm = math.sqrt(sum(v * v for v in q))
    if norm < 1e-9:
        raise ValueError('degenerate quaternion')
    w, x, y, z = [v / norm for v in q]
    return (
        1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y),
        2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x),
        2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y),
    )


def rpy_to_rotmat(roll_rad, pitch_rad, yaw_rad):
    """ZYX Tait-Bryan 欧拉角 -> 旋转矩阵（与 core_sim transform_utils 约定一致）。"""
    cr, sr = math.cos(roll_rad), math.sin(roll_rad)
    cp, sp = math.cos(pitch_rad), math.sin(pitch_rad)
    cy, sy = math.cos(yaw_rad), math.sin(yaw_rad)
    return (
        cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
        sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
        -sp, cp * sr, cp * cr,
    )


def rotz_rotmat(angle_rad):
    """绕 NED 的 +Z(向下) 旋转。正角度 = 北->东，即顺时针（罗盘方向）。"""
    c, s = math.cos(angle_rad), math.sin(angle_rad)
    return (c, -s, 0.0,
            s, c, 0.0,
            0.0, 0.0, 1.0)


def mat_mul(a, b):
    return (
        a[0] * b[0] + a[1] * b[3] + a[2] * b[6],
        a[0] * b[1] + a[1] * b[4] + a[2] * b[7],
        a[0] * b[2] + a[1] * b[5] + a[2] * b[8],
        a[3] * b[0] + a[4] * b[3] + a[5] * b[6],
        a[3] * b[1] + a[4] * b[4] + a[5] * b[7],
        a[3] * b[2] + a[4] * b[5] + a[5] * b[8],
        a[6] * b[0] + a[7] * b[3] + a[8] * b[6],
        a[6] * b[1] + a[7] * b[4] + a[8] * b[7],
        a[6] * b[2] + a[7] * b[5] + a[8] * b[8],
    )


def mat_vec(r, v):
    return (r[0] * v[0] + r[1] * v[1] + r[2] * v[2],
            r[3] * v[0] + r[4] * v[1] + r[5] * v[2],
            r[6] * v[0] + r[7] * v[1] + r[8] * v[2])


def rotmat_to_rpy_deg(r):
    """旋转矩阵 -> (roll, pitch, yaw)，单位度（仅用于日志诊断）。"""
    pitch = math.asin(max(-1.0, min(1.0, -r[6])))
    roll = math.atan2(r[7], r[8])
    yaw = math.atan2(r[3], r[0])
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def quat_from_euler_zyx(roll_rad, pitch_rad, yaw_rad):
    """欧拉角 -> 四元数 [w,x,y,z]（供仿真传感器构造 #285 姿态使用）。"""
    cr, sr = math.cos(roll_rad * 0.5), math.sin(roll_rad * 0.5)
    cp, sp = math.cos(pitch_rad * 0.5), math.sin(pitch_rad * 0.5)
    cy, sy = math.cos(yaw_rad * 0.5), math.sin(yaw_rad * 0.5)
    return (
        cy * cp * cr + sy * sp * sr,
        cy * cp * sr - sy * sp * cr,
        cy * sp * cr + sy * cp * sr,
        sy * cp * cr - cy * sp * sr,
    )


# ------------------------------------------------------------- 光轴方向求解
def los_unit_ned(q, yaw_locked=True, heading_deg=None,
                 boresight_rpy_deg=(0.0, 0.0, 0.0),
                 img_offset_rad=(0.0, 0.0), sensor_axis='x'):
    """求光轴（激视线）在世界系 NED 下的单位向量。

    :param q: #285 四元数 [w,x,y,z]（机体系 -> 世界系）
    :param yaw_locked: #285 的 YAW_LOCK 标志
    :param heading_deg: 飞机罗盘航向(度)，yaw 未锁定时使用
    :param boresight_rpy_deg: 光轴安装偏差（云台基准 -> 传感器实际指向）
    :param img_offset_rad: 目标相对光轴的图像偏角 (alpha, beta)：
                           alpha = 右偏为正(绕机体 +Z 向下)，beta = 下偏为正
    :param sensor_axis: 光轴对应的机体系轴，'x'/'y'/'z'
    """
    r_gimbal = quat_to_rotmat(q)                       # 机体系 FRD -> 世界 NED
    if not yaw_locked:
        if heading_deg is None:
            raise ValueError('yaw 未锁定(YAW_LOCK=0)时必须提供 heading_deg')
        # 机体相对 yaw 叠加罗盘航向 -> 世界系
        r_gimbal = mat_mul(rotz_rotmat(math.radians(heading_deg)), r_gimbal)

    r_b = rpy_to_rotmat(*(math.radians(a) for a in boresight_rpy_deg))
    r_fix = _SENSOR_AXIS_FIX[sensor_axis]
    r_world_sensor = mat_mul(mat_mul(r_gimbal, r_b), r_fix)   # 光轴基准系 -> 世界 NED

    alpha, beta = img_offset_rad
    # 光轴系内：x 前(基准)、y 右、z 下。图像 u 右偏 -> +y，v 下偏 -> +z
    ray = _normalize((1.0, math.tan(alpha), math.tan(beta)))
    return _normalize(mat_vec(r_world_sensor, ray))


def ned_to_enu(v):
    """NED (n,e,d) -> ENU (e,n,u) 分量顺序转换，返回 (east, north, up)。"""
    return (v[1], v[0], -v[2])


# ------------------------------------------------------------------ 主解算
def solve_target(uav_lat, uav_lon, uav_alt, q, slant_range,
                 yaw_locked=True, heading_deg=None,
                 boresight_rpy_deg=(0.0, 0.0, 0.0),
                 img_offset_rad=(0.0, 0.0), sensor_axis='x'):
    """根据飞机 GPS + 云台姿态 + 斜距，解算目标经纬高。

    :return: dict
        lat, lon, alt        目标 WGS84 坐标（alt 与输入同基准，mavros 为 AMSL）
        east, north, up      相对飞机位置的 ENU 偏移 [m]
        slant                使用的斜距 [m]
        ground_range         水平距离 [m]
        bearing_deg          飞机->目标 方位角（0=北，顺时针）
        elevation_deg        视线仰角（负值=俯视）
        looking_up           光轴是否指向地平线以上（此时不是地面目标，应丢弃）
    """
    ray_ned = los_unit_ned(q, yaw_locked=yaw_locked, heading_deg=heading_deg,
                           boresight_rpy_deg=boresight_rpy_deg,
                           img_offset_rad=img_offset_rad,
                           sensor_axis=sensor_axis)
    east, north, up = ned_to_enu(ray_ned)

    L = float(slant_range)
    de, dn, du = L * east, L * north, L * up

    lat, lon, alt = geodesy.enu_to_geodetic(uav_lat, uav_lon, uav_alt, de, dn, du)

    ground_range = math.hypot(de, dn)
    bearing = (math.degrees(math.atan2(de, dn)) + 360.0) % 360.0
    elevation = math.degrees(math.asin(max(-1.0, min(1.0, up))))

    return {
        'lat': lat, 'lon': lon, 'alt': alt,
        'east': de, 'north': dn, 'up': du,
        'slant': L,
        'ground_range': ground_range,
        'bearing_deg': bearing,
        'elevation_deg': elevation,
        'looking_up': up > 0.0,
    }


def solution_from_records(uav_record, gimbal_record, slant_range, cfg=None):
    """把"飞机位姿 + 云台姿态 + 测距"三份缓存记录拼起来求解（给 ROS 节点用）。

    :param uav_record: dict(lat, lon, alt)
    :param gimbal_record: dict(q, yaw_locked, heading_deg)
    :param cfg: dict(boresight_rpy_deg, img_offset_rad, sensor_axis)
    """
    cfg = cfg or {}
    return solve_target(
        uav_record['lat'], uav_record['lon'], uav_record['alt'],
        gimbal_record['q'], slant_range,
        yaw_locked=gimbal_record.get('yaw_locked', True),
        heading_deg=gimbal_record.get('heading_deg'),
        boresight_rpy_deg=cfg.get('boresight_rpy_deg', (0.0, 0.0, 0.0)),
        img_offset_rad=cfg.get('img_offset_rad', (0.0, 0.0)),
        sensor_axis=cfg.get('sensor_axis', 'x'),
    )
