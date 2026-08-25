#!/usr/bin/env python3

import rospy
from mavros_msgs.msg import State, PositionTarget, HomePosition
from sensor_msgs.msg import Imu, BatteryState, NavSatFix
from nav_msgs.msg import Odometry
from mavros_msgs.srv import (
    CommandBool,
    CommandBoolRequest,
    CommandHome,
    CommandLong,
    CommandLongRequest,
    CommandTOL,
    ParamGet,
    ParamSet,
    ParamSetRequest,
    SetMode,
    SetModeRequest,
    StreamRate,
)
from math import *
import pymap3d as pm
from xd_uav_sead.comms.communication_info import *
import xd_uav_sead.planning.pathFollowing as pf
from std_srvs.srv import SetBool, Trigger
from xd_uav_controller.srv import Takeoff as ManagerTakeoff
from geometry_msgs.msg import Point, Vector3
from time import monotonic, sleep as wall_sleep, time


class Drone(object):
    def __init__(self, uav_name="uav0", message_rate=10):
        self.uav_name = uav_name
        self.ns_mavros = f"/{uav_name}/mavros"
        self.control_backend = rospy.get_param(
            "~control_backend", "direct_mavros"
        ).strip().lower()
        if self.control_backend not in ("direct_mavros", "xd_control_manager"):
            rospy.logwarn(
                f"[{uav_name}] unsupported control_backend={self.control_backend}; "
                "falling back to direct_mavros"
            )
            self.control_backend = "direct_mavros"
        self.uses_external_control_manager = (
            self.control_backend == "xd_control_manager"
        )
        self.headless_sitl_failsafe_bypass = bool(
            rospy.get_param("~headless_sitl_failsafe_bypass", False)
        )
        self.reference_frame = rospy.get_param(
            "~control_reference_frame", f"{uav_name}/local_origin"
        )
        self.shared_frame_enabled = bool(
            rospy.get_param("~shared_frame/enabled", False)
        )
        self.shared_frame_offset = [
            float(rospy.get_param("~shared_frame/offset_x", 0.0)),
            float(rospy.get_param("~shared_frame/offset_y", 0.0)),
            float(rospy.get_param("~shared_frame/offset_z", 0.0)),
        ]
        self.planning_reference_frame = rospy.get_param(
            "~shared_frame/frame_id", self.reference_frame
        ) if self.shared_frame_enabled else self.reference_frame
        # derive a UAV index from name (e.g. 'uav2' -> 2). Default 0 when not present.
        try:
            digits = "".join([c for c in uav_name if c.isdigit()])
            self.uav_index = int(digits) if digits != "" else 0
        except Exception:
            self.uav_index = 0
        # default altitude base (meters) + per-UAV offset (10m per index)
        self.default_altitude = 220 + (self.uav_index * 10)
        " Stream rate service "
        try:
            rospy.wait_for_service(f"{self.ns_mavros}/set_stream_rate", timeout=3.0)
            set_stream_rate = rospy.ServiceProxy(
                f"{self.ns_mavros}/set_stream_rate", StreamRate
            )
            set_stream_rate(message_rate=message_rate, on_off=1)
        except (rospy.ROSException, rospy.ServiceException) as exc:
            rospy.logwarn(f"[{uav_name}] set_stream_rate failed (non-fatal): {exc}")
        " Cooridnate system "
        self.origin_dict = {
            "0": rospy.get_param("~gps_origin_0_lat", 47.397742),
            "1": rospy.get_param("~gps_origin_1_lat", 22.9105162191694),
            "0_lon": rospy.get_param("~gps_origin_0_lon", 8.545594),
            "1_lon": rospy.get_param("~gps_origin_1_lon", 120.312446057796),
        }
        self.origin = [
            self.origin_dict["0"],
            self.origin_dict["0_lon"],
        ]
        " UAV info "
        self.armed = False
        self.mode = None
        self.control_local_pose = [0, 0, 0]
        self.local_pose, self.local_velo = [0, 0, 0], [0, 0, 0]
        self.gps_pose_lla = [0, 0, 0]
        self.roll, self.pitch, self.yaw = 0, 0, 0
        self.battery_volt, self.battery_perc = 0, 0
        self.keepoffboard = None  # 跑了任务之后，在最后一个任务点保持OFFBOARD模式
        self.defaultoffboard = None  # 没跑任务时，保持OFFBOARD模式
        self.altitude_ok = False
        self.transition_sent = False
        self.last_setpoint_time = time()
        self.home = [0, 0, 0]
        self.home_valid = False
        # Classify from the standard MAV_TYPE parameter.  Never guess a frame
        # type: sending multirotor controls to a fixed-wing vehicle is unsafe.
        self.frame_type = None  # 初始化为 None，只等 classifier 确认
        self._wait_for_supported_frame_type(
            float(rospy.get_param("~mav_type_timeout", 45.0))
        )
        rospy.loginfo(f"[Drone] frame_type = {self.frame_type}")
        rospy.Subscriber(f"{self.ns_mavros}/state", State, self.state_callback)
        rospy.Subscriber(f"{self.ns_mavros}/imu/data", Imu, self.imu_callback)
        rospy.Subscriber(
            f"{self.ns_mavros}/local_position/odom", Odometry, self.gps_enu_callback
        )
        rospy.Subscriber(
            f"{self.ns_mavros}/global_position/global", NavSatFix, self.gps_lla_callback
        )
        rospy.Subscriber(
            f"{self.ns_mavros}/home_position/home", HomePosition, self.home_callback
        )
        rospy.Subscriber(
            f"{self.ns_mavros}/battery", BatteryState, self.battery_callback
        )
        setpoint_topic = (
            rospy.get_param(
                "~control_setpoint_topic",
                f"/{self.uav_name}/control/reference/setpoint",
            )
            if self.uses_external_control_manager
            else f"{self.ns_mavros}/setpoint_raw/local"
        )
        self.setpoint_pub = rospy.Publisher(setpoint_topic, PositionTarget, queue_size=1)
        self.swiftwing_vector_pub = rospy.Publisher(
            f"/{self.uav_name}/control_signal/vector",
            Vector3,
            queue_size=1,
        )
        " => { Px, Py, Pz, Yaw } "
        self.position_cmd = PositionTarget()
        self.position_cmd.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        self.position_cmd.type_mask = 2552
        " => { Px, Py, Pz } "
        self.waypoint_cmd = PositionTarget()
        self.waypoint_cmd.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        self.waypoint_cmd.type_mask = 2552
        " => { Vx, Vy, Vz } "
        self.velocity_cmd = PositionTarget()
        self.velocity_cmd.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        self.velocity_cmd.type_mask = 3527
        " Prameters "
        self.v = 15
        self.Rmin = 50
        self.type = 2
        self.offboard_control_source = "position"
        self.last_swiftwing_vector_cmd = None
        self.last_swiftwing_vector_cmd_time = 0.0
        self.last_swiftwing_vector_debug_log_time = 0.0
        rospy.loginfo(
            f"[Drone] control_backend={self.control_backend}, "
            f"setpoint_topic={setpoint_topic}, reference_frame={self.reference_frame}, "
            f"shared_frame_enabled={self.shared_frame_enabled}, "
            f"shared_frame_offset={self.shared_frame_offset}"
        )

    def shared_to_control_waypoint(self, waypoint):
        values = [float(waypoint[0]), float(waypoint[1]), float(waypoint[2])]
        if not self.shared_frame_enabled:
            return values
        return [
            values[index] - self.shared_frame_offset[index]
            for index in range(3)
        ]

    def _publish_control_reference(self, message):
        if self.uses_external_control_manager:
            message.header.stamp = rospy.Time.now()
            message.header.frame_id = self.reference_frame
        self.setpoint_pub.publish(message)
        self.last_setpoint_time = time()

    def _call_manager_trigger(self, service_name):
        full_name = f"/{self.uav_name}/control_manager/{service_name}"
        try:
            rospy.wait_for_service(full_name, timeout=2.0)
            response = rospy.ServiceProxy(full_name, Trigger)()
            if not response.success:
                rospy.logwarn(f"[{self.uav_name}] {service_name} rejected: {response.message}")
            return bool(response.success)
        except (rospy.ROSException, rospy.ServiceException) as exc:
            rospy.logerr(f"[{self.uav_name}] {service_name} failed: {exc}")
            return False

    def enter_fail_closed_loiter(self):
        """Relinquish external control, then request PX4 fixed-wing loiter."""
        self.keepoffboard = None
        self.defaultoffboard = None
        if self.uses_external_control_manager:
            # The manager owns OFFBOARD.  Its public cancellation service must
            # complete before SEAD asks PX4 for AUTO.LOITER, preserving a single
            # control owner throughout the fail-closed transition.
            if not self._call_manager_trigger("cancel_offboard"):
                rospy.logerr(
                    f"[{self.uav_name}] cannot enter fail-closed loiter: "
                    "control manager retained OFFBOARD"
                )
                return False
        try:
            rospy.wait_for_service(f"{self.ns_mavros}/set_mode", timeout=2.0)
            response = rospy.ServiceProxy(
                f"{self.ns_mavros}/set_mode", SetMode
            )(custom_mode="AUTO.LOITER")
            if not response.mode_sent:
                rospy.logerr(f"[{self.uav_name}] PX4 rejected AUTO.LOITER")
            return bool(response.mode_sent)
        except (rospy.ROSException, rospy.ServiceException) as exc:
            rospy.logerr(f"[{self.uav_name}] AUTO.LOITER request failed: {exc}")
            return False

    def _handoff_external_mode(self, px4_mode):
        """Relinquish manager-owned OFFBOARD before selecting a PX4 mode."""
        self.keepoffboard = None
        self.defaultoffboard = None
        if not self._call_manager_trigger("cancel_offboard"):
            rospy.logerr(
                f"[{self.uav_name}] cannot hand control to {px4_mode}: "
                "control manager retained OFFBOARD"
            )
            return False
        try:
            rospy.wait_for_service(f"{self.ns_mavros}/set_mode", timeout=2.0)
            response = rospy.ServiceProxy(
                f"{self.ns_mavros}/set_mode", SetMode
            )(custom_mode=str(px4_mode))
            if not response.mode_sent:
                rospy.logerr(f"[{self.uav_name}] PX4 rejected {px4_mode}")
            return bool(response.mode_sent)
        except (rospy.ROSException, rospy.ServiceException) as exc:
            rospy.logerr(f"[{self.uav_name}] {px4_mode} request failed: {exc}")
            return False

    def state_callback(self, msg):
        self.armed = msg.armed
        px4_mode = msg.mode  # 获取原始 PX4 模式字符串

        # === PX4 到 APM 的模式映射表 ===
        if px4_mode == "OFFBOARD":
            self.mode = "GUIDED"  # 核心映射：外部控制

        elif px4_mode == "AUTO.TAKEOFF":
            self.mode = "AUTO"  # APM 的起飞通常包含在 AUTO 任务中

        elif px4_mode == "AUTO.LAND":
            self.mode = "LAND"  # 对应 Enum: LAND

        elif px4_mode == "AUTO.LOITER":
            self.mode = "LOITER"  # 对应 Enum: LOITER

        elif px4_mode == "AUTO.RTL":
            self.mode = "RTL"  # 对应 Enum: RTL
        else:
            self.mode = "POSHOLD"  # 其他模式则直接无用，保证能正常发包的模式就行

    def gps_lla_callback(self, msg):
        self.gps_pose_lla = [msg.latitude, msg.longitude, msg.altitude]

    def gps_enu_callback(self, msg):
        e_h, n_h, u_h = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        ]
        if self.uses_external_control_manager:
            # The estimator and control manager operate in the MAVROS local ENU
            # frame.  This path must not depend on MAVROS home_position, which is
            # intentionally blacklisted by the MRS simulation configuration.
            self.control_local_pose = [e_h, n_h, u_h]
            if self.shared_frame_enabled:
                self.local_pose = [
                    self.control_local_pose[index] + self.shared_frame_offset[index]
                    for index in range(3)
                ]
            else:
                self.local_pose = list(self.control_local_pose)
            self.local_velo = [
                msg.twist.twist.linear.x,
                msg.twist.twist.linear.y,
                msg.twist.twist.linear.z,
            ]
            return

        if not self.home_valid:
            return

        " Transform to the unified ENU coordinates of UAVs and GCS "
        x, y, z = pm.enu2ecef(e_h, n_h, u_h, self.home[0], self.home[1], self.home[2])
        e, n, u = pm.ecef2enu(x, y, z, self.origin[0], self.origin[1], 1418)
        self.local_pose = [e, n, u_h]
        self.local_velo = [
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z,
        ]

    def imu_callback(self, msg):
        self.imu_msg = msg
        self.roll, self.pitch, self.yaw = self.euler_from_quaternion(
            msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w
        )

    def euler_from_quaternion(self, x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = atan2(t3, t4)

        return roll_x, pitch_y, yaw_z  # in radians

    def home_callback(self, msg):
        self.home = [msg.geo.latitude, msg.geo.longitude, msg.geo.altitude]
        self.home_valid = isfinite(self.home[0]) and isfinite(self.home[1]) and (
            abs(self.home[0]) <= 90.0 and abs(self.home[1]) <= 180.0
        ) and not (self.home[0] == 0.0 and self.home[1] == 0.0)

    def battery_callback(self, msg):
        self.battery_volt = msg.voltage
        self.battery_perc = msg.percentage * 100

    def set_home_position(self, lat, lng, alt):
        rospy.wait_for_service(f"{self.ns_mavros}/cmd/set_home")
        try:
            cmd_setHome = rospy.ServiceProxy(
                f"{self.ns_mavros}/cmd/set_home", CommandHome
            )
            responce = cmd_setHome(
                current_gps=False, latitude=lat, longitude=lng, altitude=alt
            )
            print(responce.success)
        except rospy.ServiceException as e:
            print(e)

    def takeoff(self, alt):  # 起飞
        if self.uses_external_control_manager:
            service_name = f"/{self.uav_name}/control_manager/takeoff"
            try:
                if (
                    self.frame_type == FrameType.Fixed_wing
                    and self.headless_sitl_failsafe_bypass
                    and not self._configure_fixedwing_takeoff_safety()
                ):
                    rospy.logerr(
                        f"[{self.uav_name}] headless fixed-wing safety "
                        "configuration failed; manager takeoff not requested"
                    )
                    return False
                rospy.wait_for_service(service_name, timeout=2.0)
                response = rospy.ServiceProxy(service_name, ManagerTakeoff)(
                    altitude=float(alt)
                )
                if not response.success:
                    rospy.logwarn(
                        f"[{self.uav_name}] manager takeoff rejected: {response.message}"
                    )
                return bool(response.success)
            except (ValueError, rospy.ROSException, rospy.ServiceException) as exc:
                rospy.logerr(f"[{self.uav_name}] manager takeoff failed: {exc}")
                return False
        ns_mavros = self.ns_mavros
        if self.local_pose[2] < 15.0:
            rospy.loginfo("Fixed-wing: On ground. Setting launch params...")
            # self._set_px4_param("COM_OBL_ACT", value_int=0)
            # rospy.sleep(0.1)
            # self._set_px4_param("COM_OBL_RC_ACT", value_int=0)
            # rospy.sleep(0.1)
            # self._set_px4_param("COM_FAIL_ACT", value_int=0)
            # rospy.sleep(0.1)
            # self._set_px4_param("NAV_DLL_ACT", value_int=0)
            # rospy.sleep(0.1)
            # self._set_px4_param("COM_RCL_EXCEPT", value_int=4)
            # rospy.sleep(0.1)
            # self._set_px4_param("COM_RC_IN_MODE", value_int=1)

            rospy.loginfo("Fixed-wing: Switching to AUTO.TAKEOFF...")
            try:
                rospy.wait_for_service(f"{ns_mavros}/set_mode", timeout=2.0)
                set_mode_client = rospy.ServiceProxy(f"{ns_mavros}/set_mode", SetMode)
                set_mode_client(custom_mode="AUTO.TAKEOFF")

                rospy.wait_for_service(f"{ns_mavros}/cmd/arming", timeout=2.0)
                arm_client = rospy.ServiceProxy(f"{ns_mavros}/cmd/arming", CommandBool)
                arm_client(True)
            except Exception as e:
                rospy.logerr(f"Fixed-wing Takeoff Trigger Failed: {e}")
                return False

    def _set_px4_param(self, param_id, value_int=None, value_real=None):
        """辅助函数：设置PX4参数"""
        try:
            rospy.wait_for_service(f"{self.ns_mavros}/param/set", timeout=1.0)
            client = rospy.ServiceProxy(f"{self.ns_mavros}/param/set", ParamSet)
            req = ParamSetRequest()
            req.param_id = param_id
            if value_int is not None:
                req.value.integer = value_int
            if value_real is not None:
                req.value.real = value_real
            response = client(req)
            if not response.success:
                rospy.logerr(f"Param set rejected: {param_id}")
                return False
            return True
        except Exception as e:
            rospy.logwarn(f"Param set failed: {param_id} - {e}")
            return False

    def _configure_fixedwing_takeoff_safety(self):
        """Configure fixed-wing loss actions, with an explicit SITL-only bypass."""
        if self.headless_sitl_failsafe_bypass:
            rospy.logwarn(
                "Fixed-wing headless SITL bypass enabled: disabling RC/data-link "
                "loss actions and pinning the PX4 simulated battery at 100%; "
                "the official simulated airspeed sensor remains required"
            )
            settings = (
                ("COM_RC_IN_MODE", 4, None),
                # Ignore the absent RC link in both Hold (bit 1) and Offboard
                # (bit 2), otherwise PX4 replaces the final AUTO.LOITER with
                # the configured RC-loss action as soon as OFFBOARD ends.
                ("COM_RCL_EXCEPT", 6, None),
                ("NAV_DLL_ACT", 0, None),
                # PX4's battery_simulator defaults to a 60 s discharge interval.
                # Long SEAD routes therefore trigger a real PX4 battery failsafe
                # even though Gazebo has no finite fuel source.  Keep this
                # headless-SITL-only path deterministic without weakening the
                # battery protection used by real vehicles.
                ("SIM_BAT_DRAIN", None, 86400.0),
                ("SIM_BAT_MIN_PCT", None, 100.0),
            )
        else:
            settings = (
                ("NAV_RCL_ACT", 1, None),
                ("COM_RCL_EXCEPT", 4, None),
                ("COM_OF_LOSS_T", None, 5.0),
                ("COM_OBL_RC_ACT", 5, None),
            )
        for param_id, value_int, value_real in settings:
            if not self._set_px4_param(
                param_id, value_int=value_int, value_real=value_real
            ):
                return False
            rospy.sleep(0.2)
        return True

    def set_mode(self, mode):
        """
        超级模式切换接口：
        1. Fixed_wing:
        - OFFBOARD: 参数设置 -> 自动起飞 -> 等待离地 -> 发送OFFBOARD所需数据 -> 切OFFBOARD
        - LAND: 切 AUTO.LAND
        - LOITER: 切 AUTO.LOITER (盘旋)
        2. Quad (多旋翼):
        - OFFBOARD: 预热 setpoint → 切 OFFBOARD
        - LAND: 切 AUTO.LAND
        - LOITER: 切 AUTO.LOITER
        - TAKEOFF: AUTO.TAKEOFF
        """
        if self.uses_external_control_manager:
            requested = str(mode).upper()
            if requested in ("OFFBOARD", "GUIDED"):
                return self._call_manager_trigger("offboard")
            if requested in ("LAND", "AUTO.LAND"):
                self.keepoffboard = None
                return self._call_manager_trigger("land")
            if requested in ("LOITER", "AUTO.LOITER"):
                return self._handoff_external_mode("AUTO.LOITER")
            if requested in ("RTL", "AUTO.RTL"):
                return self._handoff_external_mode("AUTO.RTL")
            rospy.logwarn(
                f"[{self.uav_name}] mode {mode} has no public xd_control_manager "
                "equivalent; command rejected without bypassing the manager"
            )
            return False

        ns_mavros = self.ns_mavros
        if self.frame_type == FrameType.Quad:
            if mode == "OFFBOARD" or mode == "GUIDED":
                rospy.loginfo("Quad: Switching to OFFBOARD...")
                # 预热 setpoint 流（PX4 要求切 OFFBOARD 前有持续的 setpoint）
                for i in range(20):
                    if rospy.is_shutdown():
                        return False
                    sp = PositionTarget()
                    sp.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
                    sp.type_mask = 0b0000111111111000
                    sp.position.x = self.local_pose[0]
                    sp.position.y = self.local_pose[1]
                    sp.position.z = self.local_pose[2] if self.local_pose[2] > 1.0 else 10.0
                    self.setpoint_pub.publish(sp)
                    self.last_setpoint_time = time()
                    rospy.sleep(0.05)
                self.defaultoffboard = [
                    self.local_pose[0], self.local_pose[1],
                    max(self.local_pose[2], 10.0),
                ]
                try:
                    rospy.wait_for_service(f"{ns_mavros}/set_mode", timeout=2.0)
                    res = rospy.ServiceProxy(f"{ns_mavros}/set_mode", SetMode)(custom_mode="OFFBOARD")
                    if res.mode_sent:
                        rospy.loginfo("Quad: OFFBOARD success.")
                    else:
                        rospy.logwarn("Quad: OFFBOARD REJECTED by FCU.")
                    return res.mode_sent
                except Exception as e:
                    rospy.logerr(f"Quad OFFBOARD failed: {e}")
                    return False

            elif mode == "LAND" or mode == "AUTO.LAND":
                rospy.loginfo("Quad: Switching to AUTO.LAND...")
                self.keepoffboard = None
                try:
                    rospy.wait_for_service(f"{ns_mavros}/set_mode", timeout=2.0)
                    return rospy.ServiceProxy(f"{ns_mavros}/set_mode", SetMode)(custom_mode="AUTO.LAND").mode_sent
                except Exception as e:
                    rospy.logerr(f"Quad LAND failed: {e}")
                    return False

            elif mode == "LOITER" or mode == "AUTO.LOITER":
                rospy.loginfo("Quad: Switching to AUTO.LOITER...")
                try:
                    rospy.wait_for_service(f"{ns_mavros}/set_mode", timeout=2.0)
                    return rospy.ServiceProxy(f"{ns_mavros}/set_mode", SetMode)(custom_mode="AUTO.LOITER").mode_sent
                except Exception as e:
                    rospy.logerr(f"Quad LOITER failed: {e}")
                    return False

            elif mode == "TAKEOFF":
                rospy.loginfo("Quad: Switching to AUTO.TAKEOFF...")
                try:
                    rospy.wait_for_service(f"{ns_mavros}/set_mode", timeout=2.0)
                    return rospy.ServiceProxy(f"{ns_mavros}/set_mode", SetMode)(custom_mode="AUTO.TAKEOFF").mode_sent
                except Exception as e:
                    rospy.logerr(f"Quad TAKEOFF failed: {e}")
                    return False

            return True

        elif self.frame_type == FrameType.Fixed_wing:
            if mode == "TAKEOFF":
                rospy.loginfo("Fixed-wing: Switching to AUTO.TAKEOFF...")
                try:
                    rospy.wait_for_service(f"{ns_mavros}/set_mode", timeout=2.0)
                    set_mode_client = rospy.ServiceProxy(
                        f"{ns_mavros}/set_mode", SetMode
                    )
                    set_mode_client(custom_mode="AUTO.TAKEOFF")

                    rospy.wait_for_service(f"{ns_mavros}/cmd/arming", timeout=2.0)
                    arm_client = rospy.ServiceProxy(
                        f"{ns_mavros}/cmd/arming", CommandBool
                    )
                    arm_client(True)
                except Exception as e:
                    rospy.logerr(f"Fixed-wing Takeoff Trigger Failed: {e}")
                    return False

                rospy.loginfo("Fixed-wing: Climbing... Waiting for Alt > 15m")
                while self.local_pose[2] < 15.0 and not rospy.is_shutdown():
                    rospy.sleep(0.1)
                    self.altitude_ok = True
                rospy.loginfo("Fixed-wing: Airborne confirmed.")

                rospy.wait_for_service(f"{ns_mavros}/cmd/command", timeout=2.0)
                cmd_srv = rospy.ServiceProxy(f"{ns_mavros}/cmd/command", CommandLong)
                while self.altitude_ok and not self.transition_sent:
                    cmd = CommandLongRequest()
                    cmd.command = 3000
                    cmd.param1 = 4
                    if cmd_srv.call(cmd).success:
                        self.transition_sent = True
                        rospy.loginfo("VTOL transition sent")

            # --- A. 固定翼起飞 + OFFBOARD ---
            elif mode == "OFFBOARD":
                rospy.loginfo("Fixed-wing: Initiating Takeoff -> OFFBOARD sequence...")

                # 1. 如果还在地上 (<30m)，先执行起飞
                if self.local_pose[2] < 15.0:
                    rospy.loginfo("Fixed-wing: On ground. Setting launch params...")
                    if not self._configure_fixedwing_takeoff_safety():
                        rospy.logerr("Fixed-wing takeoff safety parameter setup failed")
                        return False

                    rospy.loginfo("Fixed-wing: Switching to AUTO.TAKEOFF...")
                    try:
                        rospy.wait_for_service(f"{ns_mavros}/set_mode", timeout=2.0)
                        set_mode_client = rospy.ServiceProxy(
                            f"{ns_mavros}/set_mode", SetMode
                        )
                        mode_response = set_mode_client(custom_mode="AUTO.TAKEOFF")
                        if not mode_response.mode_sent:
                            rospy.logerr("Fixed-wing AUTO.TAKEOFF mode rejected")
                            return False

                        rospy.wait_for_service(f"{ns_mavros}/cmd/arming", timeout=2.0)
                        arm_client = rospy.ServiceProxy(
                            f"{ns_mavros}/cmd/arming", CommandBool
                        )
                        arm_response = arm_client(True)
                        if not arm_response.success:
                            rospy.logerr(
                                "Fixed-wing arming rejected "
                                f"(result={arm_response.result})"
                            )
                            return False
                    except Exception as e:
                        rospy.logerr(f"Fixed-wing Takeoff Trigger Failed: {e}")
                        return False

                    rospy.loginfo("Fixed-wing: Climbing... Waiting for Alt > 15m")
                    while self.local_pose[2] < 15.0 and not rospy.is_shutdown():
                        rospy.sleep(0.1)
                        self.altitude_ok = True
                    rospy.loginfo("Fixed-wing: Airborne confirmed.")

                rospy.wait_for_service(f"{ns_mavros}/cmd/command", timeout=2.0)
                cmd_srv = rospy.ServiceProxy(f"{ns_mavros}/cmd/command", CommandLong)
                # while self.altitude_ok and not self.transition_sent:
                #     cmd = CommandLongRequest()
                #     cmd.command = 3000
                #     cmd.param1 = 4
                #     if cmd_srv.call(cmd).success:
                #         self.transition_sent = True
                #         rospy.loginfo("VTOL transition sent")

                # 2. 预热数据流
                rospy.loginfo(
                    "Fixed-wing: Pre-streaming RAW setpoints for handshake..."
                )

                # 构造一个临时的 Setpoint 消息，直接用当前的 local_pose
                # 这样最安全，告诉飞控：“我就想待在现在这个位置”
                handshake_goal = PositionTarget()
                handshake_goal.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
                handshake_goal.type_mask = (
                    0b0000111111111000  # 忽略速度加速度，只控位置
                )

                for i in range(50):  # 增加次数到 50 次 (约2.5秒)，给飞控更多反应时间
                    if rospy.is_shutdown():
                        break

                    # 直接使用当前的 local_pose，确保数据绝对有效
                    handshake_goal.position.x = self.local_pose[0]
                    handshake_goal.position.y = self.local_pose[1]
                    handshake_goal.position.z = 45
                    # handshake_goal.yaw = self.yaw # 保持当前航向

                    self.setpoint_pub.publish(handshake_goal)
                    self.last_setpoint_time = time()
                    rospy.sleep(0.05)  # 20Hz
                self.defaultoffboard = [
                    self.local_pose[0],
                    self.local_pose[1],
                    self.default_altitude,
                ]

                # 3. 切 OFFBOARD
                try:
                    rospy.wait_for_service(f"{ns_mavros}/set_mode", timeout=2.0)
                    set_mode_client = rospy.ServiceProxy(
                        f"{ns_mavros}/set_mode", SetMode
                    )

                    # 尝试切换
                    res = set_mode_client(custom_mode="OFFBOARD")

                    if res.mode_sent:
                        rospy.loginfo("Fixed-wing: Switched to OFFBOARD successfully.")
                    else:
                        # 【新增】必须加上这个，否则被拒绝了你都不知道
                        rospy.logwarn(
                            "Fixed-wing: OFFBOARD switch REJECTED by FCU! (Check Setpoints or Failsafe)"
                        )

                    return res.mode_sent
                except Exception as e:
                    rospy.logerr(f"Fixed-wing OFFBOARD switch failed: {e}")
                    return False

            # --- B. 固定翼降落 (LAND) ---
            elif mode == "LAND" or mode == "AUTO.LAND":
                rospy.loginfo("Fixed-wing: Switching to AUTO.LAND...")
                if self.mode != "LAND":
                    self.keepoffboard = None
                    try:
                        rospy.wait_for_service(f"{ns_mavros}/set_mode", timeout=2.0)
                        set_mode_client = rospy.ServiceProxy(
                            f"{ns_mavros}/set_mode", SetMode
                        )
                        res = set_mode_client(custom_mode="AUTO.RTL").mode_sent
                        rospy.sleep(1.0)
                        return res
                    except:
                        return False
                else:
                    return True

            # --- C. 固定翼盘旋 (LOITER) ---
            elif mode == "LOITER" or mode == "AUTO.LOITER":
                rospy.loginfo("Fixed-wing: Switching to AUTO.LOITER (Hold)...")
                if self.mode != "LOITER":
                    try:
                        rospy.wait_for_service(f"{ns_mavros}/set_mode", timeout=2.0)
                        set_mode_client = rospy.ServiceProxy(
                            f"{ns_mavros}/set_mode", SetMode
                        )
                        # PX4 固定翼盘旋/悬停模式
                        res = set_mode_client(custom_mode="AUTO.LOITER").mode_sent
                        rospy.sleep(1.0)
                        return res
                    except Exception as e:
                        rospy.logerr(f"Fixed-wing LOITER switch failed: {e}")
                        return False
                else:
                    return True

    def set_arm(self):
        """
        统一解锁接口：
        1. Fixed_wing: 使用原生 MAVROS cmd/arming
        """
        if self.uses_external_control_manager:
            rospy.logwarn(
                f"[{self.uav_name}] xd_control_manager intentionally has no public "
                "arm-only service; use takeoff instead"
            )
            return False
        ns_mavros = f"{self.ns_mavros}"

        if self.frame_type == FrameType.Fixed_wing:
            rospy.loginfo("Fixed-wing: Sending Arm command...")
            try:
                # 使用标准的 MAVROS 解锁服务
                rospy.wait_for_service(f"{ns_mavros}/cmd/arming", timeout=2.0)
                cmd_arming = rospy.ServiceProxy(f"{ns_mavros}/cmd/arming", CommandBool)

                res = cmd_arming(True)

                if res.success:
                    rospy.loginfo("Fixed-wing: Armed successfully.")
                else:
                    rospy.logwarn(f"Fixed-wing: Arming refused (result: {res.result})")
                return res.success

            except Exception as e:
                rospy.logerr(f"Fixed-wing Arming failed: {e}")
                return False

    def set_disarm(self):
        """
        统一上锁接口：
        1. Fixed_wing: 使用原生 MAVROS cmd/arming (False)
        """
        if self.uses_external_control_manager:
            rospy.logwarn(
                f"[{self.uav_name}] xd_control_manager intentionally has no public "
                "in-flight disarm service; use land instead"
            )
            return False
        # 命名空间构建
        ns_mavros = self.ns_mavros

        if self.frame_type == FrameType.Fixed_wing:
            rospy.loginfo("Fixed-wing: Disarming...")
            try:
                # 使用标准的 MAVROS 上锁服务
                rospy.wait_for_service(f"{ns_mavros}/cmd/arming", timeout=2.0)
                cmd_arming = rospy.ServiceProxy(f"{ns_mavros}/cmd/arming", CommandBool)

                # 发送 False 进行上锁
                res = cmd_arming(False)

                if res.success:
                    rospy.loginfo("Fixed-wing: Disarmed successfully.")
                else:
                    rospy.logwarn(f"Fixed-wing: Disarm refused (result: {res.result})")
                return res.success

            except Exception as e:
                rospy.logerr(f"Fixed-wing Disarm failed: {e}")
                return False

    def get_param(self, param_name):
        try:
            rospy.wait_for_service(f"{self.ns_mavros}/param/get", timeout=2.0)
            param_get = rospy.ServiceProxy(f"{self.ns_mavros}/param/get", ParamGet)
            responce = param_get(param_id=param_name)
            if responce.success:
                return responce.value
            else:
                return False
        except (rospy.ROSException, rospy.ServiceException):
            return False

    def uav_classifier(self):
        """Classify a supported airframe from the standard MAV_TYPE parameter."""
        mav_type_param = self.get_param("MAV_TYPE")
        if mav_type_param is None or mav_type_param is False:
            rospy.logwarn("[classifier] MAV_TYPE read failed; retrying")
            return None

        mav_type = int(mav_type_param.integer)
        rospy.loginfo(f"[classifier] MAV_TYPE = {mav_type}")
        if mav_type == 1:  # MAV_TYPE_FIXED_WING
            self.frame_type = FrameType.Fixed_wing
        elif mav_type in {2, 3, 4, 13, 14, 15, 20, 21}:
            # Quadrotor, coaxial, helicopter, hexa-, octo-, tri-, deca- and
            # dodecarotor all use the existing multirotor control path.
            self.frame_type = FrameType.Quad
        else:
            rospy.logerr(f"[classifier] unsupported MAV_TYPE = {mav_type}")
            return None

        rospy.loginfo(f"[classifier] => {self.frame_type}")
        return self.frame_type

    def _wait_for_supported_frame_type(self, timeout):
        """Wait for MAVROS parameter sync without depending on simulation time."""
        deadline = monotonic() + max(0.0, float(timeout))
        attempts = 0
        while not rospy.is_shutdown():
            attempts += 1
            if self.uav_classifier():
                self._classifier_attempts = attempts
                return self.frame_type
            remaining = deadline - monotonic()
            if remaining <= 0.0:
                break
            # Gazebo may not publish /clock yet, so rospy.sleep() can deadlock
            # or consume an unexpectedly short simulated interval at startup.
            wall_sleep(min(0.5, remaining))
        raise RuntimeError(
            "uav_classifier could not determine a supported MAV_TYPE "
            f"within {float(timeout):.1f}s ({attempts} attempts)"
        )



    def origin_correction(self, origin_id):
        oid = str(origin_id)
        self.origin = [
            self.origin_dict.get(oid, self.origin_dict.get("0", 47.397742)),
            self.origin_dict.get(f"{oid}_lon", self.origin_dict.get("0_lon", 8.545594)),
        ]

    def guide_to_waypoint(self, waypoint, yaw=None):
        "waypoint = [Px, Py, Pz]  with yaw angle"
        self.offboard_control_source = "position"
        if self.uses_external_control_manager:
            # Mission waypoints are local ENU coordinates in the same odom frame
            # used by xd_uav_state_estimators and xd_uav_control_manager.
            e, n, altitude = self.shared_to_control_waypoint(waypoint)
            if altitude <= 0.0:
                altitude = self.control_local_pose[2]
        else:
            if not self.home_valid:
                rospy.logerr_throttle(
                    2.0,
                    f"[{self.uav_name}] waypoint rejected: MAVROS home position "
                    "is not available",
                )
                return False
            x, y, z = pm.enu2ecef(
                waypoint[0], waypoint[1], waypoint[2],
                self.origin[0], self.origin[1], 1418,
            )
            e, n, u = pm.ecef2enu(
                x, y, z, self.home[0], self.home[1], self.home[2]
            )
            altitude = float(waypoint[2])
            # Preserve the original direct-MAVROS mission altitude convention.
            if altitude < 50.0:
                altitude = self.local_pose[2]
        self.position_cmd.position.x = e
        self.position_cmd.position.y = n
        self.position_cmd.position.z = altitude
        self.keepoffboard = [e, n, altitude]
        self.position_cmd.yaw = yaw if yaw is not None else self.yaw
        self._publish_control_reference(self.position_cmd)
        return True

    def set_offboard_control_source(self, source):
        self.offboard_control_source = str(source or "position")

    def velocity_control(self, velocity):
        """velocity = [Vx, Vy, Vz]"""
        self.offboard_control_source = "local_velocity"
        self.velocity_cmd.velocity.x = velocity[0]
        self.velocity_cmd.velocity.y = velocity[1]
        self.velocity_cmd.velocity.z = velocity[2]
        self._publish_control_reference(self.velocity_cmd)

    def swiftwing_vector_control(self, speed_mps, heading_rad, vz_mps=0.0):
        """
        Publish ENU velocity vector to SwiftWing vector_fw node.
        Only used by 3-target SimpleStrike.
        speed_mps: horizontal speed command
        heading_rad: ENU heading, atan2(North, East)
        vz_mps: vertical speed command, positive up
        """
        try:
            speed = float(speed_mps)
            heading = float(heading_rad)
            vz = float(vz_mps)
        except Exception as ex:
            rospy.logwarn(f"[SWIFTWING_VECTOR] invalid command: {ex}")
            return False

        msg = Vector3()
        msg.x = speed * cos(heading)
        msg.y = speed * sin(heading)
        msg.z = vz

        if self.uses_external_control_manager:
            self.velocity_cmd.velocity.x = msg.x
            self.velocity_cmd.velocity.y = msg.y
            self.velocity_cmd.velocity.z = msg.z
            self._publish_control_reference(self.velocity_cmd)
        else:
            self.swiftwing_vector_pub.publish(msg)
        self.offboard_control_source = "swiftwing_vector"
        self.keepoffboard = None
        self.defaultoffboard = None
        self.last_swiftwing_vector_cmd = (msg.x, msg.y, msg.z, speed, heading, vz)
        self.last_swiftwing_vector_cmd_time = time()
        self.last_setpoint_time = time()

        now = time()
        if now - self.last_swiftwing_vector_debug_log_time >= 1.0:
            self.last_swiftwing_vector_debug_log_time = now
            rospy.logwarn(
                f"[SWIFTWING_VECTOR_CMD] uav_name={self.uav_name}, "
                f"vx={msg.x:.2f}, vy={msg.y:.2f}, vz={msg.z:.2f}, "
                f"speed={speed:.2f}, heading={heading:.3f}, "
                f"source={self.offboard_control_source}"
            )
        return True

    def replay_last_swiftwing_vector_setpoint(self):
        if self.last_swiftwing_vector_cmd is None:
            return False
        vx, vy, vz, speed, heading, vz_cmd = self.last_swiftwing_vector_cmd
        msg = Vector3()
        msg.x = float(vx)
        msg.y = float(vy)
        msg.z = float(vz)
        if self.uses_external_control_manager:
            self.velocity_cmd.velocity.x = msg.x
            self.velocity_cmd.velocity.y = msg.y
            self.velocity_cmd.velocity.z = msg.z
            self._publish_control_reference(self.velocity_cmd)
        else:
            self.swiftwing_vector_pub.publish(msg)
        self.offboard_control_source = "swiftwing_vector"
        self.keepoffboard = None
        self.defaultoffboard = None
        self.last_setpoint_time = time()
        self.last_swiftwing_vector_cmd_time = time()
        return True

    # 新增加了速度和航向控制 sun
    def iteration(self, event):
        print("LLA: ", self.gps_pose_lla)
        print("local/pose:", self.local_pose, self.local_velo)
        print("yaw angle:", self.yaw, self.yaw * 180 / pi)
        print("roll angle:", self.roll, self.roll * 180 / pi)
        print("pitch angle:", self.pitch, self.pitch * 180 / pi)
        print(self.position_cmd.coordinate_frame, PositionTarget.FRAME_LOCAL_NED)
        print("....................................")
        print("bat:", self.battery_perc)
        print("bat:", self.battery_volt)
        print("....................................")
        print("home: ", self.home)
        print("armed", self.armed)
        print(self.frame_type)
        print("\n")


if __name__ == "__main__":
    rospy.init_node("drone", anonymous=True)
    uav_name = rospy.get_param("~uav_name", "uav0")
    uav_controll = Drone(uav_name=uav_name)

    rospy.Timer(rospy.Duration(0.5), uav_controll.iteration)
    rospy.spin()
