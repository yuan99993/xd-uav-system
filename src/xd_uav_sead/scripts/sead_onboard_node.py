#!/usr/bin/env python3

# === Hardware XBee import (original path, kept for real-hardware deployment) ===
try:
    from digi.xbee.devices import XBee64BitAddress, DigiMeshDevice, RemoteDigiMeshDevice
    from digi.xbee.exception import TimeoutException
    XBEE_HW_AVAILABLE = True
except ImportError:
    XBee64BitAddress = None
    DigiMeshDevice = None
    RemoteDigiMeshDevice = None
    class TimeoutException(Exception):
        """Fallback type used only when the Digi XBee SDK is unavailable."""

    XBEE_HW_AVAILABLE = False

# === ROS simulation bridge (new path, for PX4 SITL / Gazebo without XBee) ===
from xd_uav_sead.comms.rosbridge import SeadRosBridge
from xd_uav_sead.drone.drone import Drone, PositionTarget
from xd_uav_sead.comms.communication_info import (
    Armed,
    FrameType,
    Message_ID,
    Mode,
    WaypointMissionMethod,
    XBee_Devices,
    packet_processing,
    pathFollowingMethod,
)
from time import time
import numpy as np
from math import pi
from struct import unpack
import rospy
import multiprocessing as mp
import xd_uav_sead.planning.DPGA as DPGA
from xd_uav_sead.airspace.airspace_manager import AirspaceManager, ZoneDef
from xd_uav_sead.formation.formation_control import (
    FormationConfig,
    FormationController,
    FormationShape,
)
from xd_uav_sead.strike.simple_strike import SimpleStrikeManager

import os
import fcntl
import time as time_module
import subprocess
import signal
import json

launch_process = None   # 全局 launch 句柄
simple_strike_control_mode = os.environ.get(
    "SIMPLE_STRIKE_CONTROL_MODE",
    "swiftwing_vector",
).lower()
sead_control_mode = os.environ.get(
    "SEAD_CONTROL_MODE",
    "swiftwing_vector",
).lower()
sead_runtime_mode_env = os.environ.get(
    "SEAD_RUNTIME_MODE",
    "simple_strike",
).lower()
ga_time_interval = 0.5
ga_population_size = 100

if sead_control_mode in ["swiftwing", "speed"]:
    sead_control_mode = "swiftwing_vector"
elif sead_control_mode in ["position", "waypoint"]:
    sead_control_mode = "position_waypoint"
elif sead_control_mode not in [
    "swiftwing_vector",
    "position_waypoint",
    "swiftwing",
    "speed",
    "position",
    "waypoint",
]:
    rospy.logwarn(
        f"[SEAD] Unsupported SEAD_CONTROL_MODE={sead_control_mode}, fallback to swiftwing_vector"
    )
    sead_control_mode = "swiftwing_vector"

def start_roslaunch(uav_id, launch_pkg=None, launch_file=None):
    global launch_process
    if launch_process is not None and launch_process.poll() is None:
        print("[LAUNCH] Already running")
        return

    pkg = launch_pkg or rospy.get_param("~formation_launch_pkg", "")
    lfile = launch_file or rospy.get_param(
        "~formation_launch_file", ""
    )
    if not pkg or not lfile:
        rospy.logwarn(
            "[LAUNCH] external formation launch requested but "
            "~formation_launch_pkg/~formation_launch_file is not configured"
        )
        return
    try:
        print(f"[LAUNCH] Starting formation launch for UAV {uav_id}...")
        launch_process = subprocess.Popen(
            ["roslaunch", pkg, lfile, f"uav_id:={uav_id}"],
            preexec_fn=os.setsid,
        )
        print(f"[LAUNCH] Started with PID {launch_process.pid}")
    except Exception as e:
        print(f"[LAUNCH] Failed to start: {e}")


def stop_roslaunch():
    global launch_process
    if launch_process is None:
        return

    if launch_process.poll() is None:
        print("[LAUNCH] Stopping launch and all child nodes...")
        try:
            os.killpg(os.getpgid(launch_process.pid), signal.SIGTERM)
            launch_process.wait(timeout=5)
            print("[LAUNCH] Stopped cleanly")
        except Exception as e:
            print(f"[LAUNCH] Force killing: {e}")
            os.killpg(os.getpgid(launch_process.pid), signal.SIGKILL)
    else:
        print("[LAUNCH] Launch already stopped")

    launch_process = None


def activate_sead_mission(
    sead_info,
    old_task_allocation_process,
    pending_task_inserts,
    xbee,
    data,
    UAV,
    new_timer,
    gcs_address,
    u2u_address,
    airspace,
    uav_id,
    log_jsonl=None,
):
    global Mission

    stop_roslaunch()  # 停止编队 launch，SEAD 阶段只保留 SEAD 控制
    height = np.round(UAV.local_pose[2])
    waypoint_radius = sead_info[-1]
    target_count = len(sead_info[0]) if sead_info and len(sead_info) > 0 else 0
    Mission = Message_ID.SEAD_mission

    try:
        uav_config = sead_info[4]
        UAV.type = uav_config[0]
        UAV.v = uav_config[1]
        UAV.Rmin = uav_config[2]
    except Exception as ex:
        rospy.logwarn(f"[SEAD] UAV config apply failed: {ex}")

    try:
        if (
            old_task_allocation_process is not None
            and old_task_allocation_process.is_alive()
        ):
            old_task_allocation_process.terminate()
            old_task_allocation_process.join(timeout=1.0)
    except Exception as ex:
        rospy.logwarn(f"[SEAD] old GA process cleanup failed: {ex}")

    xbee.send_data_async(
        gcs_address,
        data.pack_record_time_packet(f"received SEAD mission", new_timer.t()),
    )
    xbee.send_data_async(
        gcs_address,
        data.pack_info_packet(f"SEAD target count = {target_count}"),
    )

    rospy.loginfo(f"[Main] Received SEAD Mission Command.")

    use_simple_strike = (
        sead_runtime_mode_env in ["simple_strike", "strike", "sync"]
        and target_count == 3
    )

    if use_simple_strike:
        runtime_message = (
            f"[SEAD] runtime=simple_strike, "
            f"SIMPLE_STRIKE_CONTROL_MODE={simple_strike_control_mode}"
        )
        xbee.send_data_async(
            gcs_address,
            data.pack_info_packet(runtime_message),
        )
        rospy.logwarn(runtime_message)
        simple_manager = SimpleStrikeManager(
            sead_info[0],
            sead_info[1],
            sead_info[3],
            uav_id,
            sead_info[4],
            log_jsonl=log_jsonl,
            airspace=airspace,
            control_mode=simple_strike_control_mode,
        )
        if getattr(simple_manager, "simple_strike_control_mode", "") == "swiftwing_vector":
            UAV.simple_strike_control_backend = "swiftwing_vector"
            offboard_source = "swiftwing_vector"
            backend_message = "SimpleStrike control backend -> SwiftWing vector"
        else:
            UAV.simple_strike_control_backend = "position_waypoint"
            offboard_source = "position"
            backend_message = "SimpleStrike control backend -> Position waypoint"
        if hasattr(UAV, "set_offboard_control_source"):
            UAV.set_offboard_control_source(offboard_source)
        else:
            UAV.offboard_control_source = offboard_source
        UAV.keepoffboard = None
        UAV.defaultoffboard = None
        xbee.send_data_async(
            gcs_address,
            data.pack_info_packet(backend_message),
        )
        if log_jsonl is not None:
            try:
                log_jsonl(
                    "simple_strike_control_mode",
                    simple_strike_control_mode=simple_manager.simple_strike_control_mode,
                    full_path_control_backend=simple_manager.full_path_control_backend,
                )
            except Exception:
                pass
        remote_map = {}
        try:
            remote_device_defs = [
                xb for xb in XBee_Devices if int(xb.name.replace("UAV", "")) != uav_id
            ]
            for xb, remote in zip(remote_device_defs, u2u_address):
                uid = int(xb.name.replace("UAV", ""))
                remote_map[uid] = remote
            simple_manager.set_remote_map(remote_map)
        except Exception as ex:
            rospy.logwarn(f"[SEAD] simple strike remote map setup failed: {ex}")
        if not UAV.mode == Mode.GUIDED.name:
            try:
                UAV.set_mode("OFFBOARD")
                if hasattr(UAV, "set_offboard_control_source"):
                    UAV.set_offboard_control_source(offboard_source)
                else:
                    UAV.offboard_control_source = offboard_source
                UAV.keepoffboard = None
                UAV.defaultoffboard = None
            except Exception as ex:
                rospy.logwarn(f"[SEAD] OFFBOARD switch failed: {ex}")
        return None, None, height, waypoint_radius, simple_manager, "simple_strike"

    runtime_message = (
        f"[SEAD] runtime=dpga_sead, SEAD_CONTROL_MODE={sead_control_mode}"
    )
    xbee.send_data_async(
        gcs_address,
        data.pack_info_packet(runtime_message),
    )
    rospy.logwarn(runtime_message)
    rospy.loginfo(f"[Main] Initializing Task Allocation Process...")

    taskAllocation2main, main2taskAllocation = mp.Queue(), mp.Queue()
    taskAllocationProcess = mp.Process(
        target=DPGA.task_allocation_process,
        args=(
            sead_info[0],
            ga_time_interval,
            ga_population_size,
            taskAllocation2main,
            main2taskAllocation,
        ),
    )

    rospy.loginfo(
        f"[Main] Task Allocation Process Started. Targets: {len(sead_info[0])}"
    )

    mainProcess = DPGA.main_process(
        sead_info[0],
        sead_info[1],
        sead_info[3],
        u2u_address,
        taskAllocation2main,
        main2taskAllocation,
        airspace=airspace,
        uav_id=uav_id,
        control_mode=sead_control_mode,
    )
    if not UAV.mode == Mode.GUIDED.name:
        try:
            UAV.set_mode("OFFBOARD")
        except Exception as ex:
            rospy.logwarn(f"[SEAD] OFFBOARD switch failed: {ex}")

    # 回放编队/等待期间收到的 Task_Insert，避免时序丢点
    try:
        if pending_task_inserts:
            if (
                not hasattr(mainProcess, "targets_set")
                or mainProcess.targets_set is None
            ):
                mainProcess.targets_set = []
            if (
                not hasattr(mainProcess, "new_targets")
                or mainProcess.new_targets is None
            ):
                mainProcess.new_targets = []

            added = 0
            for pt in pending_task_inserts:
                key = (round(pt[0], 3), round(pt[1], 3))
                if key not in [
                    (round(p[0], 3), round(p[1], 3))
                    for p in mainProcess.targets_set
                ]:
                    mainProcess.targets_set.append(pt)
                    mainProcess.new_targets.append(pt)
                    added += 1

            pending_task_inserts.clear()
            xbee.send_data_async(
                gcs_address,
                data.pack_info_packet(
                    f"Replayed {added} pending Task_Insert into SEAD."
                ),
            )
    except Exception as ex:
        xbee.send_data_async(
            gcs_address,
            data.pack_info_packet(f"Replay Task_Insert error: {ex}"),
        )

    try:
        if not taskAllocationProcess.is_alive():
            taskAllocationProcess.start()
    except Exception as ex:
        rospy.logerr(f"[SEAD] Failed to start GA process: {ex}")

    return mainProcess, taskAllocationProcess, height, waypoint_radius, None, "original"


# 在这样的/dev/ttyUSB*,/dev/ttyACM*设备中，自动查找符合id号的XBee设备
def find_xbee_by_id(target_id, baud=57600, scan_interval=2.0):
    """Scan serial ports and return (DigiMeshDevice, node_id, lock_file).

    Keeps the returned lock_file open to reserve the device for the caller.
    """
    if not XBEE_HW_AVAILABLE:
        raise RuntimeError("digi.xbee SDK not installed — cannot scan for XBee hardware. Use ~use_simulation:=true")
    import glob
    print(f"[XBee] Waiting for device with ID={target_id} ...")

    while True:
        ports = glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*")

        if not ports:
            print("[XBee] No serial devices found, retrying...")
            time_module.sleep(scan_interval)
            continue

        print(f"[XBee] Scanning ports: {ports}")

        for port in ports:
            lock_path = f"/tmp/xbee_lock_{os.path.basename(port)}"

            try:
                lock_file = open(lock_path, "w")
                fcntl.flock(lock_file, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except Exception:
                continue

            dev = None
            try:
                dev = DigiMeshDevice(port, baud)
                try:
                    dev.open(force_settings=True)
                except Exception as e:
                    try:
                        fcntl.flock(lock_file, fcntl.LOCK_UN)
                        lock_file.close()
                    except Exception:
                        pass
                    if dev is not None:
                        try:
                            dev.close()
                        except Exception:
                            pass
                    print(f"[XBee] Failed to open {port}: {e}")
                    continue

                try:
                    node_id_raw = dev.get_node_id()
                    node_id = int(node_id_raw) if str(node_id_raw).isdigit() else None
                except Exception:
                    node_id = None

                print(f"[XBee] {port} -> ID {node_id}")

                if node_id == target_id:
                    print(f"[XBee] Matched UAV ID {target_id} on {port}")
                    return dev, node_id, lock_file

                try:
                    dev.close()
                except Exception:
                    pass

            except Exception:
                try:
                    if dev is not None:
                        dev.close()
                except Exception:
                    pass

            try:
                fcntl.flock(lock_file, fcntl.LOCK_UN)
                lock_file.close()
            except Exception:
                pass

            time_module.sleep(0.2)

        print(f"[XBee] One scan round complete. Retry in {scan_interval}s...\n")
        time_module.sleep(scan_interval)


class Timer(object):
    def __init__(self):
        self.bias = None
        self.t1, self.t2, self.t3, self.t4 = None, None, None, None

    def t(self):
        return t() + self.bias

    def check_timer(self, interval, previous_send_time, delay=0):
        """
        sycronized communication (metrics: transmission frequency)
        """
        if (
            int((t() + self.bias + delay) * 10) % int(interval * 10) == 0
            and t() - previous_send_time >= interval / 1.01
        ):
            return True
        else:
            return False

    def check_deciTime(self, deciTime):
        if int((t() + self.bias) * 10 % 10) == deciTime:
            return True
        else:
            return False

    def check_period(self, period, previous_time):
        if t() - previous_time >= period:
            return True
        else:
            return False

    def time_synchronize_process(self, central_device, client, client_id):
        """
        Cristian algorithm: theta = (t2 - t1 + t3 - t4)/2
        """
        self.bias = None
        while not self.bias:
            print("time sycronization request")
            client.send_data_async(
                central_device,
                bytearray([Message_ID.Time_Synchromize.value, client_id]),
            )
            self.t1 = t()

            while t() - self.t1 < 5:
                try:
                    packet = client.read_data(1e-5)
                    if (
                        packet.data[0] == Message_ID.Time_Synchromize.value
                        and packet.data[1] == client_id
                    ):
                        self.t4 = packet.timestamp
                        t2, t3 = unpack("dd", packet.data[2:])
                        self.t2 = t2
                        self.t3 = t3
                        self.bias = (self.t2 - self.t1 + self.t3 - self.t4) / 2
                        print(
                            f"Bias: {self.bias}, t1:{self.t1}, t2:{self.t2}, t3:{self.t3}, t4:{self.t4}, delay{((self.t4 - self.t1) - (self.t3 - self.t2))/2}"
                        )
                        break
                except Exception:
                    continue


t = time  # 时间别名，Timer 类依赖

if __name__ == "__main__":
    # ==================== Initialization ====================
    # --- ROS connection ---
    rospy.init_node("drone", anonymous=True)

    # 从 param server 读配置（必须在 init_node 之后）
    uav_name = rospy.get_param("~uav_name", "uav0")
    uav_index = int(uav_name.replace("uav", ""))
    # Support both naming conventions: original uav0 is aircraft 1, while
    # MRS/PX4 commonly names the first aircraft uav1.
    default_uav_id = uav_index if uav_index > 0 else 1
    target_uav_id = int(rospy.get_param("~uav_id", default_uav_id))

    # 用 rospy params 覆盖模块级 os.environ 默认值（launch 文件的 param 设置从此生效）
    simple_strike_control_mode = rospy.get_param(
        "~simple_strike_control_mode",
        os.environ.get("SIMPLE_STRIKE_CONTROL_MODE", "swiftwing_vector"),
    ).lower()
    sead_control_mode = rospy.get_param(
        "~sead_control_mode",
        os.environ.get("SEAD_CONTROL_MODE", "swiftwing_vector"),
    ).lower()
    sead_runtime_mode_env = rospy.get_param(
        "~sead_runtime_mode",
        os.environ.get("SEAD_RUNTIME_MODE", "simple_strike"),
    ).lower()
    ga_time_interval = float(rospy.get_param("~ga/time_interval", 0.5))
    ga_population_size = int(rospy.get_param("~ga/population_size", 100))

    # re-validate after override
    if sead_control_mode in ["swiftwing", "speed"]:
        sead_control_mode = "swiftwing_vector"
    elif sead_control_mode in ["position", "waypoint"]:
        sead_control_mode = "position_waypoint"
    elif sead_control_mode not in [
        "swiftwing_vector",
        "position_waypoint",
    ]:
        rospy.logwarn(
            f"[SEAD] Unsupported SEAD_CONTROL_MODE={sead_control_mode}, fallback to swiftwing_vector"
        )
        sead_control_mode = "swiftwing_vector"

    # --- Communication init (XBee hardware or ROS simulation bridge) ---
    use_sim = rospy.get_param("~use_simulation", not XBEE_HW_AVAILABLE)

    if use_sim or not XBEE_HW_AVAILABLE:
        rospy.loginfo(f"[INIT] Using ROS simulation bridge (XBee hardware {'unavailable' if not XBEE_HW_AVAILABLE else 'disabled by param'})")
        comms = SeadRosBridge(uav_name, target_uav_id)
        xbee, uav_id, xbee_lock = comms, target_uav_id, None
        u2u_address = list(comms.u2u_address)
        gcs_address = comms.gcs_address
    else:
        rospy.loginfo("[INIT] Using real XBee hardware")
        # Preserve the original device discovery, node-ID match and file lock.
        xbee, uav_id, xbee_lock = find_xbee_by_id(
            target_uav_id,
            baud=int(rospy.get_param("~xbee_baud", 57600)),
        )
        u2u_address = [
            RemoteDigiMeshDevice(xbee, XBee64BitAddress.from_hex_string(xb.value))
            for xb in XBee_Devices
            if xb.name != f"UAV{uav_id}"
        ]
        gcs_address = RemoteDigiMeshDevice(
            xbee, XBee64BitAddress.from_hex_string("0013A2004105EB61")
        )
    mode_banner_lines = [
        "",
        "==================== CURRENT ONBOARD MODES ====================",
        f"UAV_NAME={uav_name}",
        f"UAV_ID={target_uav_id}",
        f"export SIMPLE_STRIKE_CONTROL_MODE={simple_strike_control_mode}",
        f"export SEAD_RUNTIME_MODE={sead_runtime_mode_env}",
        f"export SEAD_CONTROL_MODE={sead_control_mode}",
        "FORMATION_TRAIL_FOLLOW_MODE=leader_history_distance_back",
        "FORMATION_EXECUTION_MODE=position_waypoint_guide_to_waypoint",
        "FORMATION_TRANSITION=UNCHANGED_EXISTING_TRAIL_TO_VEE_LOGIC",
        "===============================================================",
        "",
    ]

    for line in mode_banner_lines:
        print(line, flush=True)

    rospy.logwarn(
        "[MODE] "
        f"SIMPLE_STRIKE_CONTROL_MODE={simple_strike_control_mode}, "
        f"SEAD_RUNTIME_MODE={sead_runtime_mode_env}, "
        f"SEAD_CONTROL_MODE={sead_control_mode}, "
        "FORMATION_TRAIL_FOLLOW_MODE=leader_history_distance_back, "
        "FORMATION_EXECUTION_MODE=position_waypoint_guide_to_waypoint, "
        "FORMATION_TRANSITION=UNCHANGED"
    )
    # 添加了输入参数，传入飞机名
    UAV = Drone(uav_name=uav_name)
    # onboard.py 接入 AirspaceManager，并对 GCS 回 Ack
    airspace = AirspaceManager()

    # --- Communication setting ---
    data = packet_processing(uav_id)
    xbee.send_data_async(
        gcs_address,
        data.pack_info_packet(
            f"[MODE] simple_strike={simple_strike_control_mode}, "
            f"sead_runtime={sead_runtime_mode_env}, "
            f"sead_control={sead_control_mode}, "
            "formation_trail_follow=leader_history_distance_back, "
            "formation_exec=position_waypoint_guide_to_waypoint, "
            "formation_transition=unchanged"
        ),
    )
    u2u_interval = 2
    u2g_interval = 0.5
    previous_u2u_time, previous_u2g_time = 0, 0

    # --- Time calibration ---
    new_timer = Timer()
    if use_sim or not XBEE_HW_AVAILABLE:
        new_timer.bias = 0.0
    else:
        new_timer.time_synchronize_process(gcs_address, xbee, uav_id)

    # --- Mission ---
    Mission = Message_ID.Default
    stop_mode = [Mode.LAND.name, Mode.RTL.name, Mode.LOITER.name]

    target, index, waypoint_radius = [], 0, 50  # waypoint_radius: m原值15
    completed = False
    previous_cmd_time = 0

    # --- SEAD runtime handles (needed for online replanning) ---
    mainProcess = None
    taskAllocationProcess = None
    simpleStrikeManager = None
    sead_runtime_mode = None
    pending_task_inserts = []  # list of [E,N]
    pending_sead_payload = None
    pending_sead_received_time = 0.0
    formation_config = FormationConfig(
        spacing=float(rospy.get_param("~formation/spacing", 220.0)),
        standoff_distance=float(
            rospy.get_param("~formation/standoff_distance", 5000.0)
        ),
        safe_separation=float(
            rospy.get_param("~formation/safe_separation", 140.0)
        ),
        altitude_step=float(rospy.get_param("~formation/altitude_step", 20.0)),
        broadcast_interval=float(
            rospy.get_param("~formation/broadcast_interval", 0.2)
        ),
        minimum_altitude=float(
            rospy.get_param("~formation/minimum_altitude", 80.0)
        ),
        static_hold=bool(rospy.get_param("~formation/static_hold", False)),
    )
    minimum_hold_altitude = float(rospy.get_param("~minimum_hold_altitude", 80.0))
    formation = FormationController(uav_id, config=formation_config)
    previous_formation_u2u = 0
    previous_formation_debug_time = 0.0
    formation_release_reported = False
    formation_hold_reported = False
    formation_shape_switched = False
    formation_in_hold = False
    pending_formation_point = None
    pending_formation_mode_request_time = 0.0

    logs_dir = os.path.join(os.getcwd(), "logs")
    os.makedirs(logs_dir, exist_ok=True)
    log_path = os.path.join(
        logs_dir,
        f"onboard_{uav_name}_{time_module.strftime('%Y%m%d_%H%M%S')}.jsonl",
    )
    log_file = open(log_path, "a", encoding="utf-8")

    def _jsonl_safe(value):
        if isinstance(value, np.ndarray):
            return value.tolist()
        if isinstance(value, np.generic):
            return value.item()
        if isinstance(value, dict):
            return {str(k): _jsonl_safe(v) for k, v in value.items()}
        if isinstance(value, (list, tuple)):
            return [_jsonl_safe(v) for v in value]
        if hasattr(value, "name") and hasattr(value, "value"):
            return value.name
        return value

    def log_jsonl(event_type, **kwargs):
        try:
            try:
                log_t = new_timer.t()
            except Exception:
                log_t = t()
            record = {
                "t": log_t,
                "uav_id": uav_id,
                "uav_name": uav_name,
                "mission": getattr(Mission, "name", str(Mission)),
                "local_pose": _jsonl_safe(getattr(UAV, "local_pose", [])),
                "event_type": event_type,
            }
            for key, value in kwargs.items():
                record[key] = _jsonl_safe(value)
            log_file.write(json.dumps(record, ensure_ascii=False) + "\n")
            log_file.flush()
        except Exception as ex:
            rospy.logwarn_throttle(2.0, f"[LOG] jsonl write failed: {ex}")

    # ==================== MainProgram ====================
    # Keep the idle path from busy-spinning.  Three onboard instances share the
    # simulator host with Gazebo/PX4/MAVROS, so yielding here is also part of
    # keeping their safety-critical state and service callbacks responsive.
    main_loop_rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        "receive data (U2U)(G2U)"
        try:
            drain_start = time()
            drain_count = 0
            while drain_count < 20 and (time() - drain_start) < 0.01:
                packet = xbee.read_data(timeout=1e-5)
                if not packet:
                    break
                drain_count += 1
                result = data.unpack_packet(packet.data)  # 解包
                if result is None:
                    continue  # 未识别的消息，跳过
                messageType, info = result
                print(messageType, info, new_timer.t())
                if (
                    messageType == Message_ID.Mode_Change
                ):  # 解包为模式改变则设置模式，同时模式改变时改变Mission状态，也可以做到停止任务
                    Mission = Message_ID.Default
                    if info.name == "GUIDED":
                        if (
                            Mission
                            not in [
                                Message_ID.Formation_Point,
                                Message_ID.Waypoints,
                                Message_ID.SEAD_mission,
                            ]
                            and pending_formation_point is None
                        ):
                            Mission = Message_ID.Default
                            formation.reset()
                        success = UAV.set_mode("OFFBOARD")
                    elif info.name == "LAND":
                        Mission = Message_ID.Default
                        pending_formation_point = None
                        pending_sead_payload = None
                        pending_sead_received_time = 0.0
                        formation_shape_switched = False
                        formation_in_hold = False
                        formation.reset()
                        success = UAV.set_mode("LAND")
                    elif info.name == "LOITER":
                        Mission = Message_ID.Default
                        pending_formation_point = None
                        pending_sead_payload = None
                        pending_sead_received_time = 0.0
                        formation_shape_switched = False
                        formation_in_hold = False
                        formation.reset()
                        success = UAV.set_mode("LOITER")
                    elif info.name == "RTL":
                        Mission = Message_ID.Default
                        pending_formation_point = None
                        pending_sead_payload = None
                        pending_sead_received_time = 0.0
                        formation_shape_switched = False
                        formation_in_hold = False
                        formation.reset()
                        success = UAV.set_mode("RTL")
                    else:
                        Mission = Message_ID.Default
                        pending_formation_point = None
                        pending_sead_payload = None
                        pending_sead_received_time = 0.0
                        formation_shape_switched = False
                        formation_in_hold = False
                        formation.reset()
                        rospy.loginfo(f"set_mode({info.name}) faild")
                elif messageType == Message_ID.info:
                    xbee.send_data_async(gcs_address, data.pack_info_packet(info))

                elif messageType == Message_ID.Arm:  # 解包为Armed则设置Armed状态
                    if info == Armed.armed:
                        if not UAV.armed:
                            UAV.set_arm()
                        else:
                            xbee.send_data_async(
                                gcs_address,
                                data.pack_info_packet(f"has already armed!"),
                            )
                    elif info == Armed.disarmed:
                        if UAV.armed:
                            UAV.set_disarm()
                        else:
                            xbee.send_data_async(
                                gcs_address,
                                data.pack_info_packet(f"has already disarmed!"),
                            )

                elif (
                    messageType == Message_ID.Time_Synchromize
                ):  # 解包为时间同步则进行时间同步
                    new_timer.bias = 0.0  # virtual sync

                elif messageType == Message_ID.Takeoff:  # 解包为Takeoff则进行Takeoff
                    success = UAV.takeoff(info)
                    # UAV.set_arm()
                    # success = UAV.takeoff(info)
                    if success:
                        xbee.send_data_async(
                            gcs_address,
                            data.pack_info_packet(f"perform Takeoff ({info}m)"),
                        )
                    else:
                        xbee.send_data_async(
                            gcs_address,
                            data.pack_info_packet(f"fail to Takeoff ({info}m)"),
                        )

                elif (
                    messageType == Message_ID.Comm_u2gFreq
                ):  # 解包为通信频率改变则改变通信频率
                    u2g_interval = 1 / info
                    xbee.send_data_async(
                        gcs_address, data.pack_info_packet(f"U2G frquency: {info}Hz")
                    )
                    # Preserve the original external-launch hook, but keep it
                    # opt-in so a frequency command cannot recursively spawn
                    # another onboard node.
                    if rospy.get_param("~start_external_formation_launch", False):
                        start_roslaunch(uav_index)
                elif (
                    messageType == Message_ID.Origin_Correction
                ):  # 解包为原点修正则进行原点修正？？？好像没有这个功能？
                    UAV.origin_correction(info)
                    xbee.send_data_async(
                        gcs_address, data.pack_info_packet(f"origin changed => {info}")
                    )

                elif (
                    messageType == Message_ID.Waypoints
                ):  # 航点任务，这里的主要是进行数据更新
                    method = info[0]
                    pending_formation_point = None
                    pending_sead_payload = None
                    pending_sead_received_time = 0.0
                    formation_shape_switched = False
                    formation_in_hold = False
                    simpleStrikeManager = None
                    sead_runtime_mode = None
                    formation.reset()
                    xbee.send_data_async(
                        gcs_address,
                        data.pack_info_packet(f"received Waypoints command"),
                    )
                    if (
                        not UAV.mode == Mode.GUIDED.name
                        and not (hasattr(UAV, 'local_pose') and UAV.local_pose[2] > 5.0)
                    ):  # 不是OFFBOARD且高度<5m才走起飞，已升空则跳过
                        success = UAV.set_mode("OFFBOARD")

                    if method == WaypointMissionMethod.guide_waypoint:  # 单点任务
                        Mission = Message_ID.Waypoints
                        waypoint_radius = info[1]
                        target = info[2]  # info[2] 是一个列表 [x, y, z, heading_deg]
                        target[2] = (
                            np.round(UAV.local_pose[2]) if target[2] == 0 else target[2]
                        )  # 如果目标高度为0，则使用当前高度（防止撞地）
                        completed = False

                    elif (
                        method == WaypointMissionMethod.guide_WPwithHeading
                    ):  # 带航向的单点任务
                        Mission = Message_ID.Waypoints
                        waypoint_radius = info[1]
                        target = info[2]  # info[2] 是一个列表 [x, y, z, heading_deg]
                        target[2] = (
                            np.round(UAV.local_pose[2]) if target[2] == 0 else target[2]
                        )  # 如果目标高度为0，则使用当前高度（防止撞地）
                        completed = False

                    elif (
                        method == WaypointMissionMethod.guide_waypoints
                    ):  # 多航点任务即VRP或者WPs
                        Mission = Message_ID.Waypoints
                        waypoint_radius = info[1]
                        target = info[2]
                        index = 0
                        completed = False

                    elif (
                        method == WaypointMissionMethod.CraigReynolds_Path_Following
                    ):  # 固定翼的另一种模式
                        Mission = Message_ID.Waypoints
                        waypoint_radius = info[1]
                        CRPF = info[2]
                        try:
                            path_pts = getattr(CRPF, "path", [])
                            crpf_count = len(path_pts) if path_pts is not None else 0
                            # print all CRPF path points for debugging
                            try:
                                if path_pts:
                                    rospy.loginfo(
                                        f"[CRPF] Path points (index: x, y, z):"
                                    )
                                    for i, pt in enumerate(path_pts):
                                        try:
                                            if len(pt) >= 3:
                                                x, y, z = (
                                                    float(pt[0]),
                                                    float(pt[1]),
                                                    float(pt[2]),
                                                )
                                            elif len(pt) == 2:
                                                x, y = float(pt[0]), float(pt[1])
                                                z = 0.0
                                            else:
                                                raise ValueError
                                            rospy.loginfo(
                                                f"[CRPF]   [{i:03d}] {x:.2f}, {y:.2f}, {z:.2f}"
                                            )
                                        except Exception:
                                            rospy.logwarn(
                                                f"[CRPF]   [{i:03d}] invalid point: {pt}"
                                            )
                                else:
                                    rospy.logwarn(
                                        "[CRPF] Path is empty, no points to print"
                                    )
                            except Exception as e:
                                rospy.logerr(f"[CRPF] Failed to print path points: {e}")
                        except Exception:
                            path_pts = []
                            crpf_count = 0

                        # 计算路径总长度（优先使用前三个分量，缺失则补0）
                        total_length = 0.0
                        try:
                            pts = path_pts if path_pts is not None else []
                            for i in range(1, len(pts)):
                                a_raw = pts[i - 1]
                                b_raw = pts[i]
                                try:
                                    a = (
                                        np.array(a_raw[:3], dtype=float)
                                        if len(a_raw) >= 3
                                        else np.array(
                                            list(a_raw[:2]) + [0.0], dtype=float
                                        )
                                    )
                                except Exception:
                                    a = np.array([0.0, 0.0, 0.0])
                                try:
                                    b = (
                                        np.array(b_raw[:3], dtype=float)
                                        if len(b_raw) >= 3
                                        else np.array(
                                            list(b_raw[:2]) + [0.0], dtype=float
                                        )
                                    )
                                except Exception:
                                    b = np.array([0.0, 0.0, 0.0])
                                total_length += np.linalg.norm(a - b)
                        except Exception:
                            total_length = 0.0

                        print(
                            f"[CRPF] received path with {crpf_count} points, method={getattr(getattr(CRPF,'method',None),'name',CRPF.method if hasattr(CRPF,'method') else 'UNKNOWN')} total_length={total_length:.2f}m",
                            flush=True,
                        )
                        try:
                            xbee.send_data_async(
                                gcs_address,
                                data.pack_info_packet(
                                    f"CRPF path count: {crpf_count}, length: {total_length:.2f}m"
                                ),
                            )
                        except Exception:
                            pass
                        UAV.v = info[-1][0]
                        UAV.Rmin = info[-1][1]
                        # log which CRPF method is selected for debugging
                        try:
                            method_name = (
                                CRPF.method.name
                                if hasattr(CRPF.method, "name")
                                else str(CRPF.method)
                            )
                        except Exception:
                            method_name = str(getattr(CRPF, "method", "UNKNOWN"))
                        rospy.loginfo(f"[CRPF] Using method: {method_name}")
                        index = 0
                        completed = False
                        pre_error = None
                        height = np.round(UAV.local_pose[2])

                elif messageType == Message_ID.Formation_Point:
                    rally_point = list(info["point"])
                    rally_point[2] = (
                        np.round(UAV.local_pose[2])
                        if len(rally_point) < 3 or rally_point[2] <= 0
                        else rally_point[2]
                    )
                    loiter_radius = float(info.get("loiter_radius", 300.0))

                    duplicate_pending = False
                    if pending_formation_point is not None:
                        pending_point = pending_formation_point.get("point", [])
                        duplicate_pending = (
                            len(pending_point) >= 3
                            and abs(float(pending_point[0]) - float(rally_point[0])) <= 1.0
                            and abs(float(pending_point[1]) - float(rally_point[1])) <= 1.0
                            and abs(float(pending_point[2]) - float(rally_point[2])) <= 1.0
                            and abs(
                                float(pending_formation_point.get("loiter_radius", 0.0))
                                - loiter_radius
                            )
                            <= 1.0
                        )

                    duplicate_active = (
                        Mission == Message_ID.Formation_Point
                        and formation.active
                        and formation.rally_point is not None
                        and abs(float(formation.rally_point[0]) - float(rally_point[0])) <= 1.0
                        and abs(float(formation.rally_point[1]) - float(rally_point[1])) <= 1.0
                        and abs(float(formation.rally_point[2]) - float(rally_point[2])) <= 1.0
                        and abs(float(formation.rally_loiter_radius) - loiter_radius) <= 1.0
                    )

                    if duplicate_pending or duplicate_active:
                        continue

                    pending_formation_point = {
                        "point": rally_point,
                        "loiter_radius": loiter_radius,
                    }
                    pending_formation_mode_request_time = 0.0
                    formation_hold_reported = False
                    formation_release_reported = False
                    formation_shape_switched = False
                    formation_in_hold = False
                    xbee.send_data_async(
                        gcs_address,
                        data.pack_info_packet(
                            f"Formation point queued: {rally_point[:2]}, loiter={loiter_radius:.0f}m"
                        ),
                    )
                    log_jsonl(
                        "formation_point_queued",
                        rally_point=rally_point,
                        loiter_radius=loiter_radius,
                    )

                elif messageType == Message_ID.Formation_State:
                    formation.update_remote_state(info)

                elif messageType == Message_ID.SimpleStrike_Assignment:
                    if (
                        Mission == Message_ID.SEAD_mission
                        and simpleStrikeManager is not None
                        and sead_runtime_mode == "simple_strike"
                    ):
                        simpleStrikeManager.apply_remote_assignment(
                            info, xbee=xbee, comm_info=data
                        )

                elif messageType == Message_ID.SimpleStrike_Assignment_Ack:
                    if (
                        Mission == Message_ID.SEAD_mission
                        and simpleStrikeManager is not None
                        and sead_runtime_mode == "simple_strike"
                    ):
                        simpleStrikeManager.apply_assignment_ack(info)

                elif messageType == Message_ID.SimpleStrike_ReleaseTime:
                    if (
                        Mission == Message_ID.SEAD_mission
                        and simpleStrikeManager is not None
                        and sead_runtime_mode == "simple_strike"
                    ):
                        simpleStrikeManager.apply_common_hit_time(
                            info, xbee=xbee, comm_info=data
                        )

                elif messageType == Message_ID.SimpleStrike_PathStatus:
                    if (
                        Mission == Message_ID.SEAD_mission
                        and simpleStrikeManager is not None
                        and sead_runtime_mode == "simple_strike"
                    ):
                        if simpleStrikeManager.apply_path_status(info):
                            try:
                                status = info.get("status", info)
                                rospy.logwarn_throttle(
                                    2.0,
                                    "[SEAD] received simple strike path status "
                                    f"from UAV{int(status.get('uav_id', -1))}",
                                )
                            except Exception:
                                pass

                elif messageType == Message_ID.Task_Insert:
                    # info expected: [E, N, task_type] (ENU meters)
                    try:
                        e_ins = float(info[0])
                        n_ins = float(info[1])
                        pt = [e_ins, n_ins]

                        # 1) 无条件缓存（避免时序丢点）
                        #    注意：浮点去重建议用 round，否则 1e-9 差异会认为不同点
                        key = (round(e_ins, 3), round(n_ins, 3))
                        if key not in [
                            (round(p[0], 3), round(p[1], 3))
                            for p in pending_task_inserts
                        ]:
                            pending_task_inserts.append(pt)
                            log_jsonl(
                                "task_insert_queued",
                                point=pt,
                                task_type=info[2] if len(info) > 2 else None,
                            )

                        # 2) 若 SEAD 已 active 且 mainProcess 存在，立即注入
                        if (
                            Mission == Message_ID.SEAD_mission
                            and mainProcess is not None
                        ):
                            if (
                                not hasattr(mainProcess, "targets_set")
                                or mainProcess.targets_set is None
                            ):
                                mainProcess.targets_set = []
                            if (
                                not hasattr(mainProcess, "new_targets")
                                or mainProcess.new_targets is None
                            ):
                                mainProcess.new_targets = []

                            # 去重注入
                            if key not in [
                                (round(p[0], 3), round(p[1], 3))
                                for p in mainProcess.targets_set
                            ]:
                                mainProcess.targets_set.append(pt)
                                mainProcess.new_targets.append(pt)
                                log_jsonl(
                                    "task_insert_accepted",
                                    point=pt,
                                    task_type=info[2] if len(info) > 2 else None,
                                )
                                xbee.send_data_async(
                                    gcs_address,
                                    data.pack_info_packet(
                                        f"Task_Insert accepted: ({e_ins:.1f},{n_ins:.1f})"
                                    ),
                                )
                            else:
                                xbee.send_data_async(
                                    gcs_address,
                                    data.pack_info_packet(
                                        "Task_Insert duplicated, ignored"
                                    ),
                                )
                        else:
                            # 没进入 SEAD 也没关系：已缓存，后面会回放
                            xbee.send_data_async(
                                gcs_address,
                                data.pack_info_packet(
                                    f"Task_Insert queued (SEAD not ready): ({e_ins:.1f},{n_ins:.1f})"
                                ),
                            )
                    except Exception as ex:
                        xbee.send_data_async(
                            gcs_address,
                            data.pack_info_packet(f"Task_Insert parse error: {ex}"),
                        )

                elif messageType == Message_ID.Airspace_Clear:
                    print(
                        f"[AIRSPACE][CLEAR] before={len(airspace.zones)} t={new_timer.t():.3f}",
                        flush=True,
                    )
                    airspace.clear()
                    print(
                        f"[AIRSPACE][CLEAR] after={len(airspace.zones)} t={new_timer.t():.3f}",
                        flush=True,
                    )

                    xbee.send_data_async(
                        gcs_address, data.pack_airspace_ack(zone_id=0, ok=1, err=0)
                    )
                    xbee.send_data_async(
                        gcs_address, data.pack_info_packet("Airspace cleared on UAV")
                    )
                elif messageType == Message_ID.Airspace_ZoneFrag:
                    # info: {"zone_id":.., "ok": None/True/False, "zonedef":..}
                    zid = info.get("zone_id", -1)

                    if info["ok"] is None:
                        # ✅ 关键：分片没收齐也要打印，否则你以为没收到
                        print(f"[AIRSPACE][RX] zid={zid} fragmenting...", flush=True)
                        # （可选）给 GCS 回一条 info，证明正在收
                        # xbee.send_data_async(gcs_address, data.pack_info_packet(f"[AIRSPACE] zid={zid} receiving frags..."))

                    elif info["ok"] is False:
                        print(
                            f"[AIRSPACE][RX][FAIL] zid={zid} err={info.get('error')}",
                            flush=True,
                        )
                        xbee.send_data_async(
                            gcs_address,
                            data.pack_airspace_ack(zone_id=zid, ok=0, err=1),
                        )
                        xbee.send_data_async(
                            gcs_address,
                            data.pack_info_packet(
                                f"ZoneDef parse fail: zone={zid}, err={info.get('error')}"
                            ),
                        )

                    else:
                        zd = info["zonedef"]
                        z = ZoneDef(
                            zone_id=zid,
                            enabled=zd["enabled"],
                            zone_type=zd["zone_type"],
                            level2d=zd["level2d"],
                            levelH=zd["levelH"],
                            minAlt=zd["minAlt"],
                            maxAlt=zd["maxAlt"],
                            vertices=zd["vertices"],
                        )

                        airspace.update_zone(z)

                        # ✅ 关键：打印“总数 + 顶点数 + first”
                        print(
                            f"[AIRSPACE][STORE] zid={zid} total={len(airspace.zones)} "
                            f"verts={len(z.vertices)} alt=({z.minAlt},{z.maxAlt}) "
                            f"first={z.vertices[0] if z.vertices else None}",
                            flush=True,
                        )

                        # ✅ 回 ACK（这是判定 COM 链路是否真正到机载的最强证据）
                        xbee.send_data_async(
                            gcs_address,
                            data.pack_airspace_ack(zone_id=zid, ok=1, err=0),
                        )
                        xbee.send_data_async(
                            gcs_address,
                            data.pack_info_packet(
                                f"[Airspace] zone={zid} stored. total={len(airspace.zones)}"
                            ),
                        )

                elif messageType == Message_ID.SEAD_mission:
                    formation_approach_active = (
                        (
                            Mission == Message_ID.Formation_Point
                            and formation.active
                            and not formation_in_hold
                        )
                        or pending_formation_point is not None
                    )
                    if formation_approach_active:
                        pending_sead_payload = info
                        pending_sead_received_time = new_timer.t()
                        log_jsonl(
                            "sead_queued",
                            target_count=len(info[0]) if info else 0,
                            unknown_target_count=len(info[1]) if len(info) > 1 else 0,
                            formation_in_hold=formation_in_hold,
                            queued_reason="formation_approach",
                        )
                        xbee.send_data_async(
                            gcs_address,
                            data.pack_info_packet(
                                "[SEAD] mission queued, waiting for formation hold"
                            ),
                        )
                        continue

                    if (
                        Mission == Message_ID.Formation_Point
                        and formation.active
                        and formation_in_hold
                    ):
                        xbee.send_data_async(
                            gcs_address,
                            data.pack_info_packet(
                                "[SEAD] manual confirm received at rally hold, starting SEAD"
                            ),
                        )
                        formation.reset()
                        previous_formation_u2u = 0
                        formation_release_reported = False
                        formation_hold_reported = False
                        formation_shape_switched = False
                        formation_in_hold = False
                        pending_sead_payload = None
                        pending_sead_received_time = 0.0
                        completed = False
                        (
                            mainProcess,
                            taskAllocationProcess,
                            height,
                            waypoint_radius,
                            simpleStrikeManager,
                            sead_runtime_mode,
                        ) = (
                            activate_sead_mission(
                                info,
                                taskAllocationProcess,
                                pending_task_inserts,
                                xbee,
                                data,
                                UAV,
                                new_timer,
                                gcs_address,
                                u2u_address,
                                airspace,
                                uav_id,
                                log_jsonl=log_jsonl,
                            )
                        )
                        log_jsonl(
                            "sead_activated",
                            from_queue=False,
                            activated_from_hold=True,
                            target_count=len(info[0]) if info else 0,
                            unknown_target_count=len(info[1]) if len(info) > 1 else 0,
                            waypoint_radius=waypoint_radius,
                            sead_runtime_mode=sead_runtime_mode,
                        )
                        continue

                    pending_sead_payload = None
                    pending_sead_received_time = 0.0
                    completed = False
                    formation.reset()
                    previous_formation_u2u = 0
                    formation_release_reported = False
                    formation_hold_reported = False
                    formation_shape_switched = False
                    formation_in_hold = False
                    xbee.send_data_async(
                        gcs_address,
                        data.pack_info_packet(
                            f"[SEAD] Mission received. Loaded zones={len(getattr(airspace, 'zones', {}))}"
                        ),
                    )
                    (
                        mainProcess,
                        taskAllocationProcess,
                        height,
                        waypoint_radius,
                        simpleStrikeManager,
                        sead_runtime_mode,
                    ) = (
                        activate_sead_mission(
                            info,
                            taskAllocationProcess,
                            pending_task_inserts,
                            xbee,
                            data,
                            UAV,
                            new_timer,
                            gcs_address,
                            u2u_address,
                            airspace,
                            uav_id,
                            log_jsonl=log_jsonl,
                        )
                    )
                    log_jsonl(
                        "sead_activated",
                        from_queue=False,
                        activated_from_hold=False,
                        target_count=len(info[0]) if info else 0,
                        unknown_target_count=len(info[1]) if len(info) > 1 else 0,
                        waypoint_radius=waypoint_radius,
                        sead_runtime_mode=sead_runtime_mode,
                    )
                    continue

                elif ( messageType == Message_ID.Swarm_Command ): # 解包为编队命令 sun
                    cmd = info.get("command", "")
                    params = info.get("params", {})
                    rospy.loginfo(f"[SWARM] Received command: {cmd} with params: {params}")
                    if cmd == "formation_config":
                        formation.apply_swarm_command(params)
                        shape_name = formation.config.shape.name
                    xbee.send_data_async(
                        gcs_address,
                        data.pack_info_packet(
                            f"Swarm command '{cmd}' applied: {shape_name}"
                            if cmd == "formation_config"
                            else f"Swarm command '{cmd}' received"
                        ),
                    )
                elif (
                    messageType == Message_ID.Mission_Abort
                ):  # 任务中止，仅变为Abort模式，并且让飞机降落
                    Mission = Message_ID.Mission_Abort
                    pending_formation_point = None
                    pending_sead_payload = None
                    pending_sead_received_time = 0.0
                    formation_shape_switched = False
                    formation_in_hold = False
                    formation.reset()
                    log_jsonl("mission_abort")
                    xbee.send_data_async(
                        gcs_address, data.pack_info_packet(f"mission stop and hold")
                    )
        except TimeoutException:
            # XBee receive timeout is normal flow control in hardware mode.
            pass
        except Exception as e:
            import traceback
            rospy.logerr_throttle(5.0, f"Main Loop Exception: {e}")
            traceback.print_exc()
            log_jsonl("main_loop_exception", error=str(e))

        if pending_formation_point is not None and UAV.armed:
            if UAV.mode == Mode.GUIDED.name:
                rally_point = list(pending_formation_point["point"])
                team_hint = [int(dev.name.replace("UAV", "")) for dev in XBee_Devices]
                formation.config.shape = FormationShape.TRAIL
                formation_shape_switched = False
                completed = False
                formation.start_rally(
                    rally_point,
                    pending_formation_point.get("loiter_radius", 300.0),
                    UAV.local_pose,
                    UAV.v,
                    rally_point[2],
                    team_ids=team_hint,
                    now=new_timer.t(),
                )
                Mission = Message_ID.Formation_Point
                previous_formation_u2u = 0
                previous_formation_debug_time = 0.0
                formation_release_reported = False
                formation_hold_reported = False
                formation_in_hold = False
                pending_formation_point = None
                xbee.send_data_async(
                    gcs_address,
                    data.pack_info_packet(
                        f"Formation mission activated: {rally_point[:2]}"
                    ),
                )
                log_jsonl(
                    "formation_activated",
                    rally_point=rally_point,
                    loiter_radius=formation.rally_loiter_radius,
                    shape=formation.config.shape.name,
                )
            elif t() - pending_formation_mode_request_time >= 5.0:
                pending_formation_mode_request_time = t()
                try:
                    UAV.set_mode("OFFBOARD")
                except Exception as ex:
                    rospy.logwarn(f"[Formation] OFFBOARD switch failed: {ex}")

        " Send data to GCS (U2G) "
        if new_timer.check_timer(
            u2g_interval, previous_u2g_time
        ):  # 数据发送的地方，回传飞机数据
            previous_u2g_time = t()
            try:
                send_packet = data.pack_u2g_packet_default(
                    Mission,
                    UAV.frame_type,
                    UAV.mode,
                    UAV.armed,
                    UAV.battery_perc,
                    new_timer.t(),
                    UAV.local_pose,
                    UAV.roll,
                    UAV.pitch,
                    UAV.yaw,
                    np.linalg.norm(UAV.local_velo),
                )
                xbee.send_data_async(gcs_address, send_packet)
            except Exception:
                pass

        " Mission program "
        if UAV.mode in ["GUIDED", "OFFBOARD"] and UAV.armed:  # OFFBOARD才执行后续的任务模式
            # 1. 如果有之前的任务目标，就在最后一个目标点盘旋
            currtnt_t = time()
            if (
                not getattr(UAV, "uses_external_control_manager", False)
                and currtnt_t - UAV.last_setpoint_time > 0.2
            ):
                if getattr(UAV, "offboard_control_source", "") == "swiftwing_vector":
                    if (
                        hasattr(UAV, "replay_last_swiftwing_vector_setpoint")
                        and UAV.replay_last_swiftwing_vector_setpoint()
                    ):
                        rospy.logwarn_throttle(
                            2.0,
                            "[OFFBOARD WATCHDOG] swiftwing_vector mode active, "
                            "replay vector setpoint",
                        )
                    else:
                        rospy.logwarn_throttle(
                            2.0,
                            "[OFFBOARD WATCHDOG] swiftwing_vector mode active but "
                            "no last vector setpoint; skip LOCAL_NED hold",
                        )
                elif UAV.keepoffboard is not None:
                    # 对于固定翼，持续发同一个点 = 盘旋
                    handshake_goal = PositionTarget()
                    handshake_goal.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
                    handshake_goal.type_mask = (
                        0b0000111111111000  # 忽略速度加速度，只控位置
                    )

                    handshake_goal.position.x = UAV.keepoffboard[0]
                    handshake_goal.position.y = UAV.keepoffboard[1]
                    handshake_goal.position.z = UAV.keepoffboard[2]
                    # 持续发送当前位置，让它原地盘旋
                    UAV.setpoint_pub.publish(handshake_goal)
                    UAV.last_setpoint_time = time()
                    rospy.loginfo("keepoffboard keepOFFBOARD not None")

                # 2. 如果刚起飞还没任务 (failsafe_hold_pos 为空)
                elif UAV.defaultoffboard is not None:
                    # 获取当前位置，并在当前位置盘旋
                    # 注意：z轴最好保持当前高度或安全高度

                    handshake_goal = PositionTarget()
                    handshake_goal.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
                    handshake_goal.type_mask = (
                        0b0000111111111000  # 忽略速度加速度，只控位置
                    )

                    handshake_goal.position.x = UAV.defaultoffboard[0]
                    handshake_goal.position.y = UAV.defaultoffboard[1]
                    handshake_goal.position.z = UAV.defaultoffboard[2]
                    # 持续发送当前位置，让它原地盘旋
                    UAV.setpoint_pub.publish(handshake_goal)
                    UAV.last_setpoint_time = time()
                    # rospy.loginfo("keepoffboard")
                else:
                    # 最后的保命：原地盘旋
                    UAV.guide_to_waypoint(
                        [
                            UAV.local_pose[0],
                            UAV.local_pose[1],
                            max(UAV.local_pose[2], minimum_hold_altitude),
                        ]
                    )
                    rospy.loginfo_throttle(2, "Watchdog: Emergency Hold at Current Pos")

        # if UAV.mode == "GUIDED":  # OFFBOARD才做后续的任务模式
        if Mission == Message_ID.Waypoints:  # 航点任务
            if new_timer.check_period(0.1, previous_cmd_time):
                previous_cmd_time = t()
                if (
                    method == WaypointMissionMethod.guide_waypoint
                ):  # 单航点任务，这个坐标系得对齐不然没办法判断飞机是否到目标点
                    UAV.guide_to_waypoint(target)
                    if (
                        np.linalg.norm(target - np.array(UAV.local_pose))
                        <= waypoint_radius
                        and not completed
                    ):
                        xbee.send_data_async(
                            gcs_address,
                            data.pack_info_packet(f"arrive at {target}"),
                        )
                        completed = True

                elif (
                    method == WaypointMissionMethod.guide_WPwithHeading
                ):  # 带航向的航点任务
                    UAV.guide_to_waypoint(target[:3], target[-1] * pi / 180)
                    if (
                        np.linalg.norm(target[:3] - np.array(UAV.local_pose))
                        <= waypoint_radius
                        and not completed
                    ):
                        xbee.send_data_async(
                            gcs_address,
                            data.pack_info_packet(
                                f"arrive at {target[:3]} with {target[-1]} deg"
                            ),
                        )
                        completed = True

                elif (
                    method == WaypointMissionMethod.guide_waypoints
                ):  # 多航点任务，VRP或者WPs
                    UAV.guide_to_waypoint(target[index])
                    if (
                        np.linalg.norm(target[index] - np.array(UAV.local_pose))
                        <= waypoint_radius
                        and not completed
                    ):
                        index += 1
                        xbee.send_data_async(
                            gcs_address, data.pack_info_packet(f"arrive at {index}")
                        )
                        if index == len(target):
                            index = -1
                            xbee.send_data_async(
                                gcs_address,
                                data.pack_info_packet(
                                    "Waypoints mission completed!"
                                ),
                            )
                            completed = True

                elif (
                    method == WaypointMissionMethod.CraigReynolds_Path_Following
                ):  # 另一种的路径跟随
                    if CRPF.method == pathFollowingMethod.path_following_position:
                        desirePoint, index, _ = CRPF.get_desirePoint(
                            UAV.v, UAV.local_pose[0], UAV.local_pose[1], UAV.yaw
                        )
                        UAV.guide_to_waypoint(
                            [desirePoint[0], desirePoint[1], CRPF.path[index][-1]]
                        )

                    elif (
                        CRPF.method
                        == pathFollowingMethod.path_following_position_yaw
                    ):
                        desirePoint, index, _ = CRPF.get_desirePoint(
                            UAV.v, UAV.local_pose[0], UAV.local_pose[1], UAV.yaw
                        )
                        UAV.guide_to_waypoint(
                            [desirePoint[0], desirePoint[1], CRPF.path[index][-1]],
                            arctan2(
                                desirePoint[1] - UAV.local_pose[1],
                                desirePoint[0] - UAV.local_pose[0],
                            ),
                        )

                    elif (
                        CRPF.method
                        == pathFollowingMethod.path_following_velocityLocal
                    ):
                        if not completed:
                            UAV.velocity_control(
                                CRPF.get_desireVelocity(
                                    UAV.v,
                                    UAV.local_pose[0],
                                    UAV.local_pose[1],
                                    UAV.local_velo[0],
                                    UAV.local_velo[1],
                                )[0]
                            )
                        else:
                            UAV.velocity_control(0, 0, 0)

                    elif (
                        CRPF.method
                        == pathFollowingMethod.dubinsPath_following_velocityBody_PID
                    ):
                        if UAV.frame_type == FrameType.Fixed_wing:
                            wp = CRPF.get_fixed_wing_waypoint(
                                UAV.local_pose[0], UAV.local_pose[1], min_dist=10.0
                            )
                            if wp:
                                UAV.guide_to_waypoint(wp)
                            else:
                                UAV.set_mode("LOITER")
                        else:
                            rospy.logwarn_throttle(
                                2.0,
                                "[Waypoints] velocityBody PID method is no longer supported; use guide_waypoints or position control",
                            )

                    if (
                        np.linalg.norm(
                            CRPF.path[-1][:2] - np.array(UAV.local_pose[:2])
                        )
                        <= waypoint_radius
                        and not completed
                    ):
                        xbee.send_data_async(
                            gcs_address,
                            data.pack_record_time_packet(
                                f"CraigReynolds Path Following {CRPF.method.name} mission completed!",
                                new_timer.t(),
                            ),
                        )
                        completed = True

        elif Mission == Message_ID.Formation_Point:
            if new_timer.check_period(0.1, previous_cmd_time):
                previous_cmd_time = t()
                team_hint = [int(dev.name.replace("UAV", "")) for dev in XBee_Devices]
                rally_cmd = formation.step_rally(
                    UAV.local_pose,
                    UAV.yaw,
                    new_timer.t(),
                    team_ids=team_hint,
                )
                if new_timer.t() - previous_formation_debug_time >= 1.0:
                    previous_formation_debug_time = new_timer.t()
                    log_jsonl(
                        "formation_rally_debug",
                        phase=rally_cmd.get("phase"),
                        slot_id=rally_cmd.get("slot_id"),
                        entry_distance=rally_cmd.get("entry_distance"),
                        holding=rally_cmd.get("holding"),
                        trail_follow_mode=rally_cmd.get("trail_follow_mode"),
                        trail_reference_id=rally_cmd.get("trail_reference_id"),
                        trail_slot_distance=rally_cmd.get("trail_slot_distance"),
                        waypoint=rally_cmd.get("waypoint"),
                        formation_shape=formation.config.shape.name,
                    )
                entry_distance = rally_cmd.get("entry_distance", None)
                switch_to_vee_distance = 600.0
                if (
                    not formation_shape_switched
                    and formation.rally_point is not None
                    and entry_distance is not None
                    and float(entry_distance) <= switch_to_vee_distance
                    and not rally_cmd.get("holding")
                ):
                    transition_start = formation.start_trail_to_vee_centered(
                        UAV.local_pose,
                        new_timer.t(),
                        team_ids=team_hint,
                        entry_distance=float(entry_distance),
                    )
                    if transition_start is not None:
                        formation_shape_switched = True
                        log_jsonl(
                            "formation_shape_switch",
                            from_shape="TRAIL",
                            to_shape="TRAIL_TO_VEE_CENTERED",
                            switch_basis="entry_distance",
                            entry_distance=float(entry_distance),
                            switch_to_vee_distance=switch_to_vee_distance,
                            spacing=formation.config.spacing,
                        )
                        log_jsonl(
                            "formation_transition_start",
                            **transition_start,
                        )
                        xbee.send_data_async(
                            gcs_address,
                            data.pack_info_packet(
                                "[Formation] shape switch: TRAIL -> VEE"
                            ),
                        )
                transition_info = rally_cmd.get("transition")
                if transition_info:
                    transition_event = transition_info.get("transition_event", "progress")
                    if transition_event == "complete":
                        log_jsonl(
                            "formation_transition_complete",
                            **transition_info,
                        )
                    elif transition_event == "progress":
                        log_jsonl(
                            "formation_transition_progress",
                            **transition_info,
                        )
                if new_timer.check_period(
                    formation.config.broadcast_interval, previous_formation_u2u
                ):
                    previous_formation_u2u = t()
                    try:
                        xbee.send_data_broadcast(
                            data.pack_formation_state_packet(
                                new_timer.t(),
                                UAV.local_pose,
                                UAV.local_velo,
                                UAV.yaw,
                                rally_cmd.get("phase", formation.phase.value),
                                rally_cmd.get(
                                    "slot_id", formation.get_slot_id(uav_id)
                                ),
                                common_target_time=0.0,
                            )
                        )
                    except Exception as ex:
                        rospy.logwarn_throttle(
                            2.0, f"[Formation][Rally] broadcast failed: {ex}"
                        )
                if rally_cmd.get("active"):
                    UAV.guide_to_waypoint(
                        rally_cmd["waypoint"], rally_cmd.get("yaw")
                    )
                    if rally_cmd.get("holding"):
                        formation_in_hold = True
                        if not formation_hold_reported:
                            formation_hold_reported = True
                            pending_wait_s = (
                                new_timer.t() - pending_sead_received_time
                                if pending_sead_payload is not None
                                and pending_sead_received_time
                                else 0.0
                            )
                            log_jsonl(
                                "formation_hold_waiting_sead",
                                loiter_radius=rally_cmd.get("loiter_radius", 0.0),
                                rally_point=formation.rally_point,
                                has_pending_sead=pending_sead_payload is not None,
                                pending_wait_s=pending_wait_s,
                            )
                            if pending_sead_payload is not None:
                                log_jsonl(
                                    "sead_queued",
                                    target_count=(
                                        len(pending_sead_payload[0])
                                        if pending_sead_payload
                                        else 0
                                    ),
                                    unknown_target_count=(
                                        len(pending_sead_payload[1])
                                        if pending_sead_payload
                                        and len(pending_sead_payload) > 1
                                        else 0
                                    ),
                                    formation_in_hold=formation_in_hold,
                                    queued_reason="formation_hold_wait_manual_confirm",
                                )
                                xbee.send_data_async(
                                    gcs_address,
                                    data.pack_info_packet(
                                        "[Formation] rally hold reached, queued SEAD is waiting for manual confirm"
                                    ),
                                )
                            else:
                                xbee.send_data_async(
                                    gcs_address,
                                    data.pack_info_packet(
                                        "[Formation] waiting at rally point for SEAD mission"
                                    ),
                                )

        elif Mission == Message_ID.SEAD_mission:  # 执行SEAD任务
            if not formation.active:
                if sead_runtime_mode == "simple_strike" and simpleStrikeManager is not None:
                    simpleStrikeManager.run(
                        xbee,
                        data,
                        UAV,
                        new_timer,
                        gcs_address,
                        height,
                        waypoint_radius,
                    )
                    completed = simpleStrikeManager.mission_flag
                elif mainProcess is not None and type(UAV) == Drone:
                    if UAV.frame_type == FrameType.Quad:
                        mainProcess.run_quadcopter(
                            xbee,
                            data,
                            UAV,
                            new_timer,
                            gcs_address,
                            height,
                            waypoint_radius,
                        )
                    elif UAV.frame_type == FrameType.Fixed_wing:
                        mainProcess.run_fixedWing(
                            xbee,
                            data,
                            UAV,
                            new_timer,
                            gcs_address,
                            height,
                            waypoint_radius,
                        )
                        completed = mainProcess.mission_flag
                elif mainProcess is not None:
                    mainProcess.run_simulation(
                        xbee, data, UAV, new_timer, gcs_address, waypoint_radius
                    )
            elif mainProcess is not None or simpleStrikeManager is not None:
                rospy.logwarn_throttle(
                    2.0, "[SEAD] formation still active, skip SEAD control"
                )

        elif Mission == Message_ID.Mission_Abort:
            if UAV.frame_type == FrameType.Quad:
                UAV.set_mode("LAND")
            elif UAV.frame_type == FrameType.Fixed_wing:
                UAV.set_mode("LOITER")

        " Mission cancal mechanism "
        if (
            UAV.mode in stop_mode and completed == True
        ):  # 如果无人机模式在stopmode中，且任务完成，则重新改变complete标志位，可以执行下一次任务
            Mission = Message_ID.Default
            completed = False
            pending_formation_point = None
            pending_sead_payload = None
            pending_sead_received_time = 0.0
            formation_shape_switched = False
            formation_in_hold = False
            simpleStrikeManager = None
            sead_runtime_mode = None
            formation.reset()
            if mainProcess is not None:
                mainProcess.mission_flag = False
        main_loop_rate.sleep()
    # 退出时，确保停止 roslaunch
    stop_roslaunch()
    xbee.send_data_async(gcs_address, data.pack_info_packet(f"rospy is shutdown!!"))
    try:
        log_file.close()
    except Exception:
        pass
