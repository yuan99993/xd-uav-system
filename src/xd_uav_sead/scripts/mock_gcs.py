#!/usr/bin/env python3
"""
mock_gcs.py — 模拟地面站通过 ROS 话题向 SEAD 机载节点发送命令。
用法:
  rosrun xd_uav_sead mock_gcs.py _uav_name:=uav1 _cmd:=takeoff _alt:=120
  rosrun xd_uav_sead mock_gcs.py _uav_name:=uav1 _cmd:=waypoint _x:=100 _y:=50 _z:=80
  rosrun xd_uav_sead mock_gcs.py _uav_name:=uav1 _cmd:=sead_mission _targets_json:='[[100,0],[200,50],[300,100]]'
  rosrun xd_uav_sead mock_gcs.py _uav_name:=uav1 _cmd:=formation_config _shape:=VEE _spacing:=220
  rosrun xd_uav_sead mock_gcs.py _uav_name:=uav1 _cmd:=airspace_zone _zone_id:=1 _points_json:=[[0,0],[100,0],[100,100],[0,100]]
  rosrun xd_uav_sead mock_gcs.py _uav_name:=uav1 _cmd:=task_insert _x:=150 _y:=75
  rosrun xd_uav_sead mock_gcs.py _uav_name:=uav1 _cmd:=mission_abort
  rosrun xd_uav_sead mock_gcs.py _uav_name:=uav1 _cmd:=freq _freq:=2.0
  rosrun xd_uav_sead mock_gcs.py _uav_name:=uav1 _cmd:=origin _origin_id:=1
  rosrun xd_uav_sead mock_gcs.py _uav_name:=uav1 _cmd:=airspace_clear
"""
import json
import rospy
from std_msgs.msg import String


def _get_float(name, default):
    return rospy.get_param("~" + name, default)

def _get_str(name, default):
    return rospy.get_param("~" + name, default)

def _get_bool(name, default):
    return rospy.get_param("~" + name, default)

def _get_int(name, default):
    return rospy.get_param("~" + name, default)


MSG_MAP = {
    # (msg_id, builder_fn)
    "takeoff":         (3,  lambda: {"alt": _get_float("alt", 120.0)}),
    "arm":             (2,  lambda: {"armed": _get_bool("armed", True)}),
    "disarm":          (2,  lambda: {"armed": False}),
    "mode":            (1,  lambda: {"name": _get_str("mode", "GUIDED")}),
    "waypoint":        (5,  lambda: {
        "method": 0,
        "radius": _get_int("radius", 50),
        "target": [
            _get_float("x", 100.0),
            _get_float("y", 0.0),
            _get_float("z", 80.0),
        ],
    }),
    "info":            (44, lambda: {"text": _get_str("text", "mock GCS test")}),
    "freq":            (6,  lambda: {"freq": _get_float("freq", 2.0)}),
    "mission_abort":   (8,  lambda: {}),
    "origin":          (9,  lambda: {"origin_id": _get_int("origin_id", 0)}),
    "sead_mission":    (18, lambda: {
        "targets": json.loads(_get_str("targets_json", "[]")),
        "unknown_targets": [],
        "uav_type": _get_int("uav_type", 2),
        "velocity": _get_float("velocity", 20.0),
        "Rmin": _get_float("Rmin", 35.0),
        "waypoint_radius": _get_int("waypoint_radius", 50),
        "init_pos": [
            _get_float("init_x", 0.0),
            _get_float("init_y", 0.0),
            _get_float("init_z", 0.0),
        ],
        "end": [
            _get_float("end_x", 0.0),
            _get_float("end_y", 0.0),
            _get_float("end_z", 0.0),
        ],
    }),
    "task_insert":     (19, lambda: {
        "point": [_get_float("x", 100.0), _get_float("y", 100.0)],
        "task_type": _get_int("task_type", 0),
    }),
    "airspace_clear":  (20, lambda: {}),
    "airspace_zone":   (21, lambda: {
        "zone_id": _get_int("zone_id", 1),
        "blob": None,  # filled below
    }),
    "swarm":           (24, lambda: {
        "command": _get_str("swarm_cmd", "formation_config"),
        "params": {
            "shape": _get_str("shape", "TRAIL"),
            "enable": int(_get_bool("enable", True)),
            "leader_id": _get_int("leader_id", 1),
            "spacing": _get_float("spacing", 220.0),
            "standoff": _get_float("standoff", 5000.0),
            "safe_sep": _get_float("safe_sep", 140.0),
            "alt_step": _get_float("alt_step", 20.0),
        },
    }),
    "formation":       (26, lambda: {
        "point_id": _get_int("point_id", 1),
        "point": [
            _get_float("x", 0.0),
            _get_float("y", 0.0),
            _get_float("z", 120.0),
        ],
        "loiter_radius": _get_float("loiter_radius", 300.0),
    }),
}


rospy.init_node("mock_gcs", anonymous=True)
uav_name = rospy.get_param("~uav_name", "uav1")
cmd = rospy.get_param("~cmd", "info").lower()
cmd = {
    "formation_config": "swarm",
    "formation_point": "formation",
}.get(cmd, cmd)
topic = f"/{uav_name}/sead/command"
pub = rospy.Publisher(topic, String, queue_size=5)
rospy.sleep(0.3)

if cmd not in MSG_MAP:
    rospy.logwarn(f"[mock GCS] Unknown command: {cmd}, sending as info")
    msg_id, info = 44, {"text": f"unknown: {cmd}"}
else:
    msg_id, build_fn = MSG_MAP[cmd]
    info = build_fn()

# Special handling for airspace_zone: pack binary blob
if cmd == "airspace_zone":
    points_json = _get_str("points_json", "[[0,0],[100,0],[100,100],[0,100]]")
    zonedef_raw = {
        "zone_id": _get_int("zone_id", 1),
        "enabled": int(_get_bool("enabled", True)),
        "zone_type": _get_int("zone_type", 0),
        "level2d": _get_int("level2d", 0),
        "levelH": _get_int("levelH", 0),
        "minAlt": _get_float("minAlt", 0.0),
        "maxAlt": _get_float("maxAlt", 500.0),
        "vertices": json.loads(points_json),
    }
    info.update(zonedef_raw)
    info.pop("blob", None)

payload = {
    "msg_id": msg_id,
    "info": info,
    "ts": rospy.Time.now().to_sec(),
    "source": "mock_gcs",
}
pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))
rospy.loginfo(f"[mock GCS] ▶ {topic}  msg_id={msg_id} ({cmd})")
rospy.sleep(0.2)
