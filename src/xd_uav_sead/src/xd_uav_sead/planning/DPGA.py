import random
import numpy as np
import threading
import queue
import rospy
from matplotlib import pyplot as plt
import multiprocessing as mp
import dubins
from xd_uav_sead.planning.GA_SEAD_process import *
from xd_uav_sead.planning.GA_SEAD_process import _is_dubins_blocked, _find_best_bypass
from xd_uav_sead.comms.communication_info import *
import time
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import tf.transformations as tf_trans

def task_allocation_process(
    targets_sites, time_interval, pop_size, ga2control_queue, control2ga_queue
):
    ga_population, update = None, True
    sead_mission = GA_SEAD(targets_sites, pop_size)
    uavs = control2ga_queue.get()
    while True:
        "Optimization (for time_interval seconds)"
        solution, fitness_value, ga_population = (
            sead_mission.run_GA_time_period_version(
                time_interval, uavs, ga_population, update
            )
        )
        " Transmit the best solution and the corrsponding fitness value to the main process "
        ga2control_queue.put([fitness_value, solution])
        if not control2ga_queue.empty():
            "Obtain the information from the main process"
            uavs = control2ga_queue.get()
            update = True
            " Terminate the task allocation process "
            if uavs == [44]:
                break
        else:
            update = False


class UAV_Simulator(object):
    def __init__(
        self, uav_id, type, frame_type, velocity, Rmin, initial_position, base
    ):
        "Configuration of UAV"
        self.id = uav_id
        self.type = type
        self.v = velocity
        self.Rmin = Rmin
        self.omega_max = self.v / self.Rmin
        self.initial_position = initial_position
        self.base = base
        self.sencing_range = 50
        self.frame_type = frame_type
        " Altitude and position of UAV "
        self.local_pose = [initial_position[0], initial_position[1], 10]
        self.yaw = initial_position[2]
        self.local_velo = [0, 0, 0]
        self.yaw_rate = 0
        " Information for GCS "
        self.mode = Mode.GUIDED.name
        self.armed = False
        self.battery_perc = 100
        self.last_setpoint_time = time.time()

    def step(self, v_cmd, yaw_cmd, dt):
        """
        P controller for yaw rate and speed
                [ x(k+1)     ]   [ x(k) + v(k)cos(theta(k))dt ]
        state:  [ y(k+1)     ] = [ y(k) + v(k)sin(theta(k))dt ]
                [ theta(k+1) ]   [ theta(k) + utheta x dt     ]
                [ v(k+1)     ]   [ v(k) + us x dt             ]
        """
        self.local_pose[0] += self.local_velo[0] * np.cos(self.yaw) * dt
        self.local_pose[1] += self.local_velo[0] * np.sin(self.yaw) * dt
        self.yaw = pf.PlusMinusPi(self.yaw + self.yaw_rate * dt)
        self.local_velo[0] += 2 * (v_cmd - self.local_velo[0]) * dt
        self.yaw_rate = self.yaw_rate + 5 * (yaw_cmd - self.yaw_rate) * dt

    def set_mode(self, mode):
        self.mode = mode


class main_process(object):
    def __init__(
        self,
        targets_sites,
        unknown_targets,
        base_config,
        u2u_communication,
        ga2control_queue,
        control2ga_queue,
        airspace=None,
        uav_id=None,
        control_mode="swiftwing_vector",
    ):
        "SEAD mission"
        # 璁剧疆鏃犱汉鏈篒D鍜屽搴旂殑鍚嶇О銆乼opic
        self.uav_id = uav_id if uav_id is not None else 0
        self.uav_name = f"uav{self.uav_id}"
        self.viz_topic_name = f"/uav{self.uav_id}/sead/planned_path"
        control_mode = str(control_mode or "swiftwing_vector").lower()
        if control_mode in ["position", "position_waypoint", "waypoint"]:
            self.control_mode = "position_waypoint"
        elif control_mode in [
            "swiftwing",
            "swiftwing_vector",
            "speed",
            "speed_swiftwing",
        ]:
            self.control_mode = "swiftwing_vector"
        else:
            rospy.logwarn(
                f"[SEAD] Unsupported control_mode={control_mode}, fallback to swiftwing_vector"
            )
            self.control_mode = "swiftwing_vector"
        
        self.targets_set = targets_sites
        self.base = base_config
        " Communication port "
        self.u2u = u2u_communication
        self.ga2control_queue = ga2control_queue
        self.control2ga_queue = control2ga_queue
        " communication interval "
        self.T, self.T_comm = 2, 0.5
        " PID controller "
        self.pre_error = None
        self.Kp = 3
        self.Kd = 7
        " Predefine variables "
        self.packet, self.pos = [], []
        self.target = None
        self.fitness, self.best_solution = 1e-5, []
        self.terminated_tasks, self.new_targets = [], []
        self.previous_time_u2u, self.previous_time_control = 0, 0
        self.task_locking = False
        self.update = True
        self.into = False
        self.AT, self.NT = [], []
        self.back_to_base = False
        " Path following "
        # Historical method name kept for protocol compatibility.
        # Runtime control uses swiftwing_vector or position_waypoint.
        self.path_following = pf.CraigReynolds_Path_Following(
            pathFollowingMethod.dubinsPath_following_velocityBody_PID,
            5,
            [],
            3,
            self.Kp,
            self.Kd,
        )
        self.intial_windowIndex = 0
        self.path_update_flag = False # 璺緞鏇存柊鏍囧織浣?sun
        " Dynamic environments "
        self.unknown_targets_set = unknown_targets
        self.sencing_range = 50
        self.mission_flag = False
        self.airspace = airspace
        self.significant_event = True # 鏍囪鏄惁鍙戠敓浜嗛噸瑕佷簨浠讹紙濡傛柊浠诲姟鍒嗛厤锛夛紝浠ュ喅瀹氭槸鍚︽洿鏂拌矾寰?sun
        self.last_sent_ids = []  # 璁板綍涓婃鍙戦€佺殑浠诲姟鍒楄〃
        self.path_pub = rospy.Publisher(
            self.viz_topic_name, Path, queue_size=1, latch=True
        )
        rospy.loginfo(f"[DEBUG] Set UAV name to {self.uav_name}, topic to {self.viz_topic_name}")

    def reset(self):
        self.update = False
        self.path_update_flag = False
        self.significant_event = True
        self.last_sent_ids = []
        self.terminated_tasks = []
        self.new_targets = []
        self.task_locking = False
        self.into = False
        self.AT, self.NT = [], []
        self.back_to_base = False
        self.fitness, self.best_solution = 1e-5, []
        self.path_following.path = []
        self.target = None
        self.intial_windowIndex = 0

    # 鏂板鍔犵殑鍑芥暟 sun
    def build_ga_input_from_dict(self, uavs_dict, zones):
        uav_ids = sorted(uavs_dict.keys())

        ids = []
        types = []
        vs = []
        Rmins = []
        poses = []
        bases = []
        costs = []
        chromosomes = []

        # 猸?杩欓噷鏀规垚鎵佸钩缁撴瀯
        completed_all = []
        new_targets_all = []

        for uid in uav_ids:
            info = uavs_dict[uid]
            ids.append(uid)
            types.append(info["type"])
            vs.append(info["v"])
            Rmins.append(info["Rmin"])
            poses.append(info["pos"])
            bases.append(info["base"])
            costs.append(info["cost"])
            chromosomes.append(info["chromosome"])

            # 馃敶 鎵佸钩鍚堝苟锛岃€屼笉鏄?append 涓€涓?list
            completed_all.extend(info["completed_tasks"])
            new_targets_all.extend(info["new_targets"])

        uavs_info_list = [
            ids,               # 0
            types,             # 1
            vs,                # 2
            Rmins,             # 3
            poses,             # 4
            bases,             # 5
            costs,             # 6
            chromosomes,       # 7
            completed_all,     # 8  鉁?GA 鏈熸湜鐨勪竴缁寸粨鏋?
            new_targets_all    # 9  鉁?GA 鏈熸湜鐨勪竴缁寸粨鏋?
        ]

        return uavs_info_list + [zones]  # 10


    # [DPGA.py -> class main_process 鍐呴儴]

    def publish_rviz_path(self, path_points, height=50.0):
        """
        鍙戝竷璺緞鍒?Rviz
        path_points: [[x, y, theta], ...]
        """
        if not path_points:
            return

        ros_path = Path()
        ros_path.header.frame_id = "map"  # 纭繚 Rviz 鐨?Fixed Frame 璁剧疆涓?map
        ros_path.header.stamp = rospy.Time.now()

        for pt in path_points:
            pose = PoseStamped()
            pose.header = ros_path.header
            pose.pose.position.x = pt[0]
            pose.pose.position.y = pt[1]
            pose.pose.position.z = height

            # 灏嗚埅鍚戣 (yaw) 杞负鍥涘厓鏁?(缁昛杞存棆杞?
            # 濡傛灉娌℃湁 tf 搴擄紝鍙敤鍏紡:
            # qz = sin(yaw/2), qw = cos(yaw/2) (鍏朵綑涓?)
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = np.sin(pt[2] / 2.0)
            pose.pose.orientation.w = np.cos(pt[2] / 2.0)

            ros_path.poses.append(pose)

        self.path_pub.publish(ros_path)

    def generate_path(self, chromosome, id, v, Rmin):
        path_route, task_sequence_state = [], []
        # if chromosome and not self.back_to_base: 
        # 娉ㄩ噴鎺変笂闈㈢殑鍒ゆ柇鏉′欢锛屽鏋滀笉娉ㄩ噴鐨勮瘽褰撳畬鎴愭墍鏈変换鍔″悗鑸嚎涓嶅啀鏇存柊锛岄鏈烘棤娉曡繑鍥炲熀鍦?
        if chromosome:
            for p in range(len(chromosome[0])):
                if chromosome[3][p] == id:
                    assign_target = chromosome[1][p]
                    # 妫€鏌ヤ换鍔℃槸鍚﹀凡瀹屾垚 sun
                    if [assign_target, chromosome[2][p]] not in self.terminated_tasks:
                        assign_heading = chromosome[4][p] * 10
                        task_sequence_state.append(
                            [
                                self.targets_set[assign_target - 1][0],
                                self.targets_set[assign_target - 1][1],
                                assign_heading,
                                assign_target,
                                chromosome[2][p],
                            ]
                        )
                    else:
                        print(f"[DEBUG] UAV{id}: Skipping completed task {assign_target}")

            task_sequence_state.append(self.base)
            for state in task_sequence_state[:-1]:
                state[2] *= np.pi / 180
            dubins_path = dubins.shortest_path(
                self.pos, task_sequence_state[0][:3], Rmin
            )
            path_route.extend(dubins_path.sample_many(v / 5)[0])
            for p in range(len(task_sequence_state) - 1):
                sp = task_sequence_state[p][:3]
                gp = (
                    task_sequence_state[p + 1][:3]
                    if task_sequence_state[p][:3] != task_sequence_state[p + 1][:3]
                    else [
                        task_sequence_state[p + 1][0],
                        task_sequence_state[p + 1][1],
                        task_sequence_state[p + 1][2] - 1e-5,
                    ]
                )
                dubins_path = dubins.shortest_path(sp, gp, Rmin)
                # path_route.extend(dubins_path.sample_many(v / 10)[0][1:])
                path_route.extend(dubins_path.sample_many(v / 5)[0][1:])
            self.path_following.path, self.target = path_route, task_sequence_state
            " Initiate the index on path"
            self.intial_windowIndex = 0
            self.path_update_flag = True # 璺緞鏇存柊鏍囧織浣?sun

    def _drain_ga_queue(self):
        while not self.ga2control_queue.empty():
            self.fitness, self.best_solution = self.ga2control_queue.get()

    def _broadcast_self_state(self, xbee, comm_info, uav_ros, new_timer):
        if (
            new_timer.check_timer(self.T, self.previous_time_u2u, -0.1)
            and not self.back_to_base
        ):
            self.previous_time_u2u = time.time()
            self.packet, self.pos = comm_info.pack_SEAD_packet(
                uav_ros.type,
                uav_ros.v,
                uav_ros.Rmin,
                [uav_ros.local_pose[0], uav_ros.local_pose[1], uav_ros.yaw],
                self.base,
                self.task_locking,
                1 / self.fitness,
                self.best_solution,
                self.terminated_tasks,
                self.new_targets,
            )
            xbee.send_data_broadcast(self.packet)

    def _refresh_team_plan(self, comm_info, uav_ros, xbee=None, gcs=None):
        if not self.packet:
            return

        for uid, info in comm_info.uavs_info.items():
            self.AT.extend(info["completed_tasks"])
            self.NT.extend(info["new_targets"])

        for target_found in self.NT:
            if target_found not in self.targets_set:
                self.targets_set.append(target_found)

        if not any(info["fix"] for info in comm_info.uavs_info.values()):
            zones = (
                self.airspace.export_zones_for_planner()
                if self.airspace is not None
                else []
            )

            ga_input = self.build_ga_input_from_dict(comm_info.uavs_info, zones)
            self.control2ga_queue.put(ga_input)

            self.AT, self.NT = [], []

            if self.update:
                candidates = []
                for uid, info in comm_info.uavs_info.items():
                    chromosome = info.get("chromosome")
                    if info.get("cost") is not None and chromosome:
                        candidates.append((info["cost"], uid, chromosome))

                if candidates and self.significant_event:
                    best_entry = sorted(candidates, key=lambda x: (x[0], x[1]))[0][2]
                    self.generate_path(
                        best_entry, comm_info.uav_id, uav_ros.v, uav_ros.Rmin
                    )

                cur_h = (
                    uav_ros.local_pose[2]
                    if hasattr(uav_ros, "local_pose")
                    else 50.0
                )
                if self.path_following.path:
                    self.publish_rviz_path(self.path_following.path, height=cur_h)

                if xbee is not None and gcs is not None:
                    try:
                        assigned_tasks = []
                        if self.target:
                            for t in self.target:
                                if len(t) > 4:
                                    t_id = int(t[3])
                                    t_type = int(t[4])
                                    if t_id > 0:
                                        assigned_tasks.append(f"{t_id}-{t_type}")

                        if assigned_tasks:
                            msg_str = f"UAV{comm_info.uav_id} Plan: {assigned_tasks}"
                            if assigned_tasks != getattr(self, "last_sent_tasks", []):
                                pkt = comm_info.pack_info_packet(msg_str)
                                xbee.send_data_async(gcs, pkt)
                                rospy.loginfo(f"[XBee] Report Sent: {msg_str}")
                                self.last_sent_tasks = list(assigned_tasks)
                    except Exception as e:
                        rospy.logwarn(f"[XBee] Feedback Failed: {e}")

            self.update = True
        else:
            self.update = False

        self.packet = []
        comm_info.SEAD_info_clear()

    def plan_only(self, xbee, comm_info, uav_ros, new_timer, gcs=None):
        self._drain_ga_queue()
        self._broadcast_self_state(xbee, comm_info, uav_ros, new_timer)
        if new_timer.check_period(self.T_comm, self.previous_time_u2u) and self.packet:
            self._refresh_team_plan(comm_info, uav_ros, xbee=xbee, gcs=gcs)

    def estimate_sync_eta(self, uav_ros, task_types=(2, 3)):
        if self.mission_flag or not self.target:
            return None

        sync_target = None
        for task in self.target:
            if len(task) > 4 and int(task[4]) in task_types:
                sync_target = task
                break

        if sync_target is None:
            for task in self.target:
                if len(task) > 4:
                    sync_target = task
                    break

        if sync_target is None:
            return None

        distance = np.linalg.norm(
            [
                sync_target[0] - uav_ros.local_pose[0],
                sync_target[1] - uav_ros.local_pose[1],
            ]
        )
        distance += max(float(uav_ros.Rmin), 0.0) * np.pi * 0.5
        return distance / max(float(getattr(uav_ros, "v", 0.0)), 1.0)

    def run_quadcopter(
        self, xbee, comm_info, uav_ros, new_timer, gcs, height, waypoint_radius
    ):
        "<<<<<<<<<< Communication Layer >>>>>>>>>>"

        " Receive the solution from the task allocation process"
        self._drain_ga_queue()

        " Broadcast every T seceods"
        if (
            new_timer.check_timer(self.T, self.previous_time_u2u, -0.1)
            and not self.back_to_base
        ):
            self._broadcast_self_state(xbee, comm_info, uav_ros, new_timer)

        " Receive the information of UAVs after Tcomm seconds "
        if new_timer.check_period(self.T_comm, self.previous_time_u2u) and self.packet:
            for uid, info in comm_info.uavs_info.items():
                self.AT.extend(info["completed_tasks"])
                self.NT.extend(info["new_targets"])
            for target_found in self.NT:
                if target_found not in self.targets_set:
                    self.targets_set.append(target_found)

            " Check the task-locking mechanism is activate or not "
            if not any(info["fix"] for info in comm_info.uavs_info.values()):
                print(comm_info.uavs_info)
                " transmit the information of UAVs(packets received) to the task allocation process"

                zones = (
                    self.airspace.export_zones_for_planner()
                    if self.airspace is not None
                    else []
                )
                # ====== 鍏抽敭鏃ュ織锛氱‘璁?zones 宸茬粡鍑嗗濂藉苟鍗冲皢閫佸叆 GA ======
                z0 = zones[0] if zones else None
                z0_poly = z0.get("poly", []) if isinstance(z0, dict) else []
                print(
                    f"[DBG][toGA][quad] uavs_len={len(comm_info.uavs_info)} zones={len(zones)} "
                    f"z0_verts={len(z0_poly)} z0_first={(z0_poly[0] if z0_poly else None)}",
                    flush=True,
                )

                # self.control2ga_queue.put(comm_info.uavs_info + [zones])
                # 鏇挎崲涓婇潰鐨勪唬鐮佷负涓嬮潰鐨勪唬鐮?sun 杩欓噷鍙槸鍐欏湪杩欓噷锛屽苟娌℃湁娴嬭瘯
                ga_input = self.build_ga_input_from_dict(comm_info.uavs_info, zones)
                self.control2ga_queue.put(ga_input)

                self.AT, self.NT = [], []
                if self.update:
                    """
                    => Choose the solution with the lowest cost value and generate the path corresponding to the UAV ID.
                            (When it comes to the same fitness value, choose the solution with the smaller UAV ID)
                    """
                    candidates = []
                    for uid, info in comm_info.uavs_info.items():
                        chromosome = info.get("chromosome")
                        if info.get("cost") is not None and chromosome:
                            candidates.append((info["cost"], uid, chromosome))
                    if candidates:
                        best_entry = sorted(candidates, key=lambda x: (x[0], x[1]))[0][2]
                        self.generate_path(
                            best_entry,
                            comm_info.uav_id,
                            uav_ros.v,
                            uav_ros.Rmin,
                        )
                self.update = True
            else:
                self.update = False
            self.packet = []
            comm_info.SEAD_info_clear()

        " <<<<<<<<<< Control Layer >>>>>>>>>> "
        if self.path_following.path:
            "Identify the target is reached or not"
            if (
                np.linalg.norm(
                    [
                        self.target[0][0] - uav_ros.local_pose[0],
                        self.target[0][1] - uav_ros.local_pose[1],
                    ]
                )
                <= waypoint_radius
                and not self.into
            ):
                "Back to the base or not"
                if self.target[:-1] == []:
                    xbee.send_data_async(
                        gcs,
                        comm_info.pack_record_time_packet(
                            f"mission complete!", new_timer.t()
                        ),
                    )
                    " Shutdown the task allocation process "
                    self.control2ga_queue.put([44])
                self.into = True

            " 鍋氬畬浠诲嫏涔嬫浠?"
            if (
                np.linalg.norm(
                    [
                        self.target[0][0] - uav_ros.local_pose[0],
                        self.target[0][1] - uav_ros.local_pose[1],
                    ]
                )
                >= waypoint_radius
                and self.into
            ):
                if self.target[0][3:]:
                    self.terminated_tasks.append(self.target[0][3:])
                    xbee.send_data_async(
                        gcs,
                        comm_info.pack_record_time_packet(
                            f"task: {self.target[0][3:]} finished", new_timer.t()
                        ),
                    )
                    del self.target[0]
                    self.task_locking = False
                    self.into = False

            " Task-locking mechanism "
            if (
                np.linalg.norm(
                    [
                        self.target[0][0] - uav_ros.local_pose[0],
                        self.target[0][1] - uav_ros.local_pose[1],
                    ]
                )
                <= 2 * uav_ros.Rmin
            ):
                if self.target[0][3:]:
                    self.task_locking = True
                else:
                    self.back_to_base = True

            if new_timer.check_period(0.1, self.previous_time_control):
                desirePoint, self.intial_windowIndex, _, error_of_distance = (
                    self.path_following.get_desirePoint_withWindow(
                        uav_ros.v,
                        uav_ros.local_pose[0],
                        uav_ros.local_pose[1],
                        uav_ros.yaw,
                        self.intial_windowIndex,
                    )
                )
                if error_of_distance <= 0 and self.back_to_base:
                    uav_ros.set_mode("LOITER")
                else:
                    uav_ros.guide_to_waypoint([desirePoint[0], desirePoint[1], height])
                self.previous_time_control = time.time()

            " Unknown targets detection "
            for t in self.unknown_targets_set:
                if (
                    np.linalg.norm(
                        [uav_ros.local_pose[0] - t[0], uav_ros.local_pose[1] - t[1]]
                    )
                    <= self.sencing_range
                    and t not in self.targets_set
                    and uav_ros.type != 3
                ):
                    self.targets_set.append(t)
                    self.new_targets.append(t)
                    xbee.send_data_async(
                        gcs,
                        comm_info.pack_record_time_packet(
                            f"discover unknown target: {t}", new_timer.t()
                        ),
                    )

    def _follow_fixedwing_path(self, uav_ros, height):
        if not self.path_following.path:
            uav_ros.set_mode("LOITER")
            return

        mode = getattr(self, "control_mode", "swiftwing_vector")

        if mode == "position_waypoint":
            wp = self.path_following.get_fixed_wing_waypoint(
                uav_ros.local_pose[0],
                uav_ros.local_pose[1],
                min_dist=10.0,
                update=self.path_update_flag,
            )
            self.path_update_flag = False

            if wp:
                uav_ros.guide_to_waypoint(wp)
            else:
                uav_ros.set_mode("LOITER")
            return

        try:
            desirePoint, self.intial_windowIndex, _, _ = (
                self.path_following.get_desirePoint_withWindow(
                    uav_ros.v,
                    uav_ros.local_pose[0],
                    uav_ros.local_pose[1],
                    uav_ros.yaw,
                    self.intial_windowIndex,
                )
            )
        except Exception as ex:
            rospy.logwarn_throttle(
                1.0,
                f"[SEAD] SwiftWing path following failed to get desirePoint: {ex}",
            )
            return

        heading_cmd = np.arctan2(
            desirePoint[1] - uav_ros.local_pose[1],
            desirePoint[0] - uav_ros.local_pose[0],
        )

        v_cmd = float(getattr(uav_ros, "v", 20.0))
        vz_cmd = 0.3 * (float(height) - float(uav_ros.local_pose[2]))

        if hasattr(uav_ros, "swiftwing_vector_control"):
            ok = uav_ros.swiftwing_vector_control(v_cmd, heading_cmd, vz_cmd)
            if not ok:
                rospy.logwarn_throttle(
                    1.0,
                    "[SEAD] swiftwing_vector_control returned False",
                )
        else:
            rospy.logwarn_throttle(
                1.0,
                "[SEAD] uav_ros has no swiftwing_vector_control(), cannot run swiftwing_vector mode",
            )

    def run_fixedWing(
        self, xbee, comm_info, uav_ros, new_timer, gcs, height, waypoint_radius
    ):
        self._drain_ga_queue()

        # self.update = True  # 褰撴敹鍒版柊鐨?GA 瑙ｅ喅鏂规鏃讹紝鏍囪闇€瑕佹洿鏂拌矾寰?

        if (
            new_timer.check_timer(self.T, self.previous_time_u2u, -0.1)
            and not self.back_to_base
        ):
            self._broadcast_self_state(xbee, comm_info, uav_ros, new_timer)

        if new_timer.check_period(self.T_comm, self.previous_time_u2u) and self.packet:
            self._refresh_team_plan(comm_info, uav_ros, xbee=xbee, gcs=gcs)

        if self.path_following.path and not self.mission_flag:
            dist_to_target = np.linalg.norm(
                [
                    self.target[0][0] - uav_ros.local_pose[0],
                    self.target[0][1] - uav_ros.local_pose[1],
                ]
            )
            # print(f"Waypoint radius: {int(waypoint_radius)}") # 璋冭瘯鏃ュ織锛氭樉绀哄綋鍓嶇殑 waypoint_radius涓?
            arrival_dist = max(waypoint_radius, 5.0 * uav_ros.Rmin)
            if dist_to_target <= arrival_dist and not self.into:
                print(f"[DEBUG] UAV{getattr(self, 'uav_id', 'Unknown')}: Entering target area, dist={dist_to_target:.2f}, arrival_dist={arrival_dist:.2f}")
                print(f"[DEBUG] UAV{getattr(self, 'uav_id', 'Unknown')}: target={self.target[0] if self.target else 'None'}")
                print(f"[DEBUG] UAV{getattr(self, 'uav_id', 'Unknown')}: target length: {len(self.target)}, target[:-1]: {self.target[:-1]}")

                if self.target[:-1] == []:
                    print(f"[DEBUG] UAV{getattr(self, 'uav_id', 'Unknown')}: All targets completed, mission complete!")
                    xbee.send_data_async(
                        gcs,
                        comm_info.pack_record_time_packet(
                            f"mission complete!", new_timer.t()
                        ),
                    )
                    self.mission_flag = True
                    self.control2ga_queue.put([44])
                    # 澶嶄綅鍙橀噺浠ュ噯澶囦笅涓€杞换鍔?
                    self.reset()
                    # self.update = False
                    # 浠诲姟瀹屾垚鍒囨崲鍒版偓鍋滄ā寮?
                    uav_ros.set_mode("LOITER")
                self.into = True
            if dist_to_target >= arrival_dist and self.into:
                print(f"[DEBUG] UAV{getattr(self, 'uav_id', 'Unknown')}: dist_to_target={dist_to_target:.2f}, arrival_dist={arrival_dist:.2f}, into={self.into}")
                print(f"[DEBUG] UAV{getattr(self, 'uav_id', 'Unknown')}: target[0]={self.target[0] if self.target else 'None'}")
                if self.target[0][3:]:
                    print(f"[DEBUG] UAV{getattr(self, 'uav_id', 'Unknown')}: Finishing task {self.target[0][3:]}")
                    self.terminated_tasks.append(self.target[0][3:])
                    xbee.send_data_async(
                        gcs,
                        comm_info.pack_record_time_packet(
                            f"task: {self.target[0][3:]} finished", new_timer.t()
                        ),
                    )
                    del self.target[0]
                    self.task_locking = False
                    self.into = False
                else:
                    print(f"[DEBUG] UAV{getattr(self, 'uav_id', 'Unknown')}: No more targets or target[0][3:] is empty")
            if (
                np.linalg.norm(
                    [
                        self.target[0][0] - uav_ros.local_pose[0],
                        self.target[0][1] - uav_ros.local_pose[1],
                    ]
                )
                <= 30 * uav_ros.Rmin
            ):
                # 妫€鏌ユ槸鍚︽墍鏈変换鍔￠兘宸插畬鎴?
                total_tasks = len(self.targets_set)
                completed_tasks = len(self.terminated_tasks)
                all_tasks_completed = completed_tasks >= total_tasks
                
                if self.target[0][3:] and not all_tasks_completed:
                    # rospy.loginfo(f"[DEBUG] Task locking enabled for target {self.target[0][3:]}")
                    self.task_locking = True
                else:
                    # rospy.loginfo(f"[DEBUG] Going back to base (no more targets or all tasks completed)")
                    # print(f"[DEBUG] UAV{getattr(self, 'uav_id', 'Unknown')}: All tasks completed ({completed_tasks}/{total_tasks}), going back to base")
                    self.back_to_base = True

            if new_timer.check_period(0.02, self.previous_time_control):
                self._follow_fixedwing_path(uav_ros, height)
                self.previous_time_control = time.time()

            for t in self.unknown_targets_set:
                if (
                    np.linalg.norm(
                        [uav_ros.local_pose[0] - t[0], uav_ros.local_pose[1] - t[1]]
                    )
                    <= self.sencing_range
                    and t not in self.targets_set
                    and uav_ros.type != 3
                ):
                    self.targets_set.append(t)
                    self.new_targets.append(t)
                    xbee.send_data_async(
                        gcs,
                        comm_info.pack_record_time_packet(
                            f"discover unknown target: {t}", new_timer.t()
                        ),
                    )
            


    # def run_fixedWing(self, xbee, comm_info, uav_ros, new_timer, gcs, height, waypoint_radius):
    #     while not self.ga2control_queue.empty():
    #         self.fitness, self.best_solution = self.ga2control_queue.get()

    #     if new_timer.check_timer(self.T, self.previous_time_u2u, -0.1) and not self.back_to_base:
    #         self.previous_time_u2u = time.time()
    #         self.packet, self.pos = comm_info.pack_SEAD_packet(
    #             uav_ros.type, uav_ros.v, uav_ros.Rmin,
    #             [uav_ros.local_pose[0], uav_ros.local_pose[1], uav_ros.yaw],
    #             self.base, self.task_locking, 1/self.fitness, self.best_solution,
    #             self.terminated_tasks, self.new_targets
    #         )
    #         xbee.send_data_broadcast(self.packet)

    #     if new_timer.check_period(self.T_comm, self.previous_time_u2u) and self.packet:
    #         self.AT.extend(comm_info.uavs_info[8])
    #         self.NT.extend(comm_info.uavs_info[9])
    #         comm_info.uavs_info[8], comm_info.uavs_info[9] = self.AT, self.NT
    #         for target_found in self.NT:
    #             if target_found not in self.targets_set:
    #                 self.targets_set.append(target_found)

    #         if not any(comm_info.task_locking):
    #             zones = self.airspace.export_zones_for_planner() if self.airspace is not None else []
    #             self.control2ga_queue.put(comm_info.uavs_info + [zones])
    #             self.AT, self.NT = [], []

    #             if self.update:
    #                 # 鐢熸垚璺緞
    #                 best_entry = sorted(
    #                     sorted(zip(comm_info.uavs_info[0], comm_info.uavs_info[6], comm_info.uavs_info[7]), key=lambda x: x[0]),
    #                     key=lambda x: x[1]
    #                 )[0][-1]
    #                 self.generate_path(best_entry, comm_info.uav_id, uav_ros.v, uav_ros.Rmin)

    #                 # Rviz 鍙鍖?
    #                 cur_h = uav_ros.local_pose[2] if hasattr(uav_ros, 'local_pose') else 50.0
    #                 if self.path_following.path:
    #                     self.publish_rviz_path(self.path_following.path, height=cur_h)

    #                 # 鏂囨湰鍥炰紶
    #                 assigned_tasks = []
    #                 if self.target:
    #                     for t in self.target:
    #                         if len(t) > 4:
    #                             t_id = int(t[3])
    #                             t_type = int(t[4])
    #                             if t_id > 0:
    #                                 assigned_tasks.append(f"{t_id}-{t_type}")
    #                 if assigned_tasks and assigned_tasks != getattr(self, 'last_sent_tasks', []):
    #                     pkt = comm_info.pack_info_packet(f"UAV{comm_info.uav_id} Plan: {assigned_tasks}")
    #                     xbee.send_data_async(gcs, pkt)
    #                     self.last_sent_tasks = list(assigned_tasks)

    #             self.update = True
    #         else:
    #             self.update = False
    #         self.packet = []
    #         comm_info.SEAD_info_clear()

    #     # ===========================
    #     # 淇敼鐐?1锛氬垵濮嬪寲 GA 璺緞绱㈠紩
    #     # ===========================
    #     if not hasattr(self, "wp_idx"):
    #         self.wp_idx = 0

    #     # ===========================
    #     # 淇敼鐐?2锛氭寔缁彂甯冭矾寰勭偣
    #     # ===========================
    #     if self.path_following.path and self.wp_idx < len(self.path_following.path):
    #         target_wp = self.path_following.path[self.wp_idx]
    #         dx = target_wp[0] - uav_ros.local_pose[0]
    #         dy = target_wp[1] - uav_ros.local_pose[1]
    #         dz = height - uav_ros.local_pose[2]
    #         dist_to_target = (dx**2 + dy**2 + dz**2) ** 0.5
    #         arrival_dist = max(waypoint_radius, 3.0 * uav_ros.Rmin)

    #         # 鎸佺画鍙戝竷褰撳墠璺緞鐐?
    #         uav_ros.guide_to_waypoint([target_wp[0], target_wp[1], height])

    #         # 鍒拌揪鍒ゅ畾锛屽垏鎹㈠埌涓嬩竴涓偣
    #         if dist_to_target <= arrival_dist:
    #             self.wp_idx += 1

    #     # ===========================
    #     # 淇敼鐐?3锛氫繚鐣欏師鏈変换鍔￠攣瀹氶€昏緫
    #     # ===========================
    #     if self.target:
    #         if np.linalg.norm([self.target[0][0] - uav_ros.local_pose[0], self.target[0][1] - uav_ros.local_pose[1]]) <= 2 * uav_ros.Rmin:
    #             if self.target[0][3:]:
    #                 self.task_locking = True
    #             else:
    #                 self.back_to_base = True

    #     # ===========================
    #     # 淇敼鐐?4锛氭湭鐭ョ洰鏍囨娴嬩繚鐣?
    #     # ===========================
    #     for t in self.unknown_targets_set:
    #         if np.linalg.norm([uav_ros.local_pose[0] - t[0], uav_ros.local_pose[1] - t[1]]) <= self.sencing_range:
    #             if t not in self.targets_set and uav_ros.type != 3:
    #                 self.targets_set.append(t)
    #                 self.new_targets.append(t)
    #                 xbee.send_data_async(gcs, comm_info.pack_record_time_packet(f"discover unknown target: {t}", new_timer.t()))

    def run_simulation(self, xbee, comm_info, uav_ros, new_timer, gcs, waypoint_radius):
        while not self.ga2control_queue.empty():
            self.fitness, self.best_solution = self.ga2control_queue.get()

        if (
            new_timer.check_timer(self.T, self.previous_time_u2u, -0.1)
            and not self.back_to_base
        ):
            self.previous_time_u2u = time.time()
            self.packet, self.pos = comm_info.pack_SEAD_packet(
                uav_ros.type,
                uav_ros.v,
                uav_ros.Rmin,
                [uav_ros.local_pose[0], uav_ros.local_pose[1], uav_ros.yaw],
                self.base,
                self.task_locking,
                1 / self.fitness,
                self.best_solution,
                self.terminated_tasks,
                self.new_targets,
            )
            
            # for agent in self.u2u:
            #     xbee.send_data_async(agent, self.packet)
            xbee.send_data_broadcast(self.packet)
            # self.terminated_tasks, self.new_targets = [], []

        if new_timer.check_period(self.T_comm, self.previous_time_u2u) and self.packet:
            for uid, info in comm_info.uavs_info.items():
                self.AT.extend(info["completed_tasks"])
                self.NT.extend(info["new_targets"])

            # 鍘婚噸鍔犲叆鏈満鐩爣闆嗗悎
            for target_found in self.NT:
                if target_found not in self.targets_set:
                    self.targets_set.append(target_found)

            # if not any(comm_info.task_locking):
            if not any(info["fix"] for info in comm_info.uavs_info.values()):

                zones = (
                    self.airspace.export_zones_for_planner()
                    if self.airspace is not None
                    else []
                )

                # z0 = zones[0] if zones else None
                # z0_poly = z0.get("poly", []) if isinstance(z0, dict) else []
                # print(
                #     f"[DBG][toGA][sim] uavs_len={len(comm_info.uavs_info)} zones={len(zones)} "
                #     f"z0_verts={len(z0_poly)} z0_first={(z0_poly[0] if z0_poly else None)}",
                #     flush=True,
                # )

                # self.control2ga_queue.put(comm_info.uavs_info + [zones])
                # 鏇挎崲涓婇潰鐨勪唬鐮佷负涓嬮潰鐨勪唬鐮?sun 杩欓噷鍙槸鍐欏湪杩欓噷锛屽苟娌℃湁娴嬭瘯
                ga_input = self.build_ga_input_from_dict(comm_info.uavs_info, zones)
                self.control2ga_queue.put(ga_input)

                print(comm_info.uavs_info)
                self.AT, self.NT = [], []
                if self.update:
                    # ===== 閫夊嚭鍏ㄥ眬鏈€浼樿В =====
                    candidates = []
                    for uid, info in comm_info.uavs_info.items():
                        if info["cost"] is not None and info["chromosome"]:
                            candidates.append((info["cost"], uid, info["chromosome"]))

                    # 娣诲姞浜唖elf.significant_event鏍囧織浣嶏紝榛樿涓篢rue锛屽綋鍙戠敓閲嶈浜嬩欢锛堝鏂颁换鍔″垎閰嶏級鏃惰缃负True锛岃矾寰勭敓鎴愬悗閲嶇疆涓篎alse
                    if candidates and self.significant_event:
                        best_entry = sorted(candidates, key=lambda x: (x[0], x[1]))[0][2]
                        self.generate_path(best_entry, comm_info.uav_id, uav_ros.v, uav_ros.Rmin)

                        # ===== RViz 鏄剧ず =====
                        cur_h = (
                            uav_ros.local_pose[2]
                            if hasattr(uav_ros, "local_pose")
                            else 50.0
                        )
                        if self.path_following.path:
                            self.publish_rviz_path(self.path_following.path, height=cur_h)

                        # ===== 鏂囨湰鍥炰紶浠诲姟 =====
                        try:
                            assigned_tasks = []
                            if self.target:
                                for t in self.target:
                                    if len(t) > 4:
                                        t_id = int(t[3])
                                        t_type = int(t[4])
                                        if t_id > 0:
                                            assigned_tasks.append(f"{t_id}-{t_type}")

                            if assigned_tasks:
                                msg_str = f"UAV{comm_info.uav_id} Plan: {assigned_tasks}"
                                if assigned_tasks != getattr(self, "last_sent_tasks", []):
                                    pkt = comm_info.pack_info_packet(msg_str)
                                    xbee.send_data_async(gcs, pkt)
                                    rospy.loginfo(f"[XBee] Report Sent: {msg_str}")
                                    self.last_sent_tasks = list(assigned_tasks)

                        except Exception as e:
                            rospy.logwarn(f"[XBee] Feedback Failed: {e}")

                        # self.significant_event = False
                    # self.generate_path(
                    #     sorted(
                    #         sorted(
                    #             zip(
                    #                 comm_info.uavs_info[0],
                    #                 comm_info.uavs_info[6],
                    #                 comm_info.uavs_info[7],
                    #             ),
                    #             key=lambda x: x[0],
                    #         ),
                    #         key=lambda x: x[1],
                    #     )[0][-1],
                    #     comm_info.uav_id,
                    #     uav_ros.v,
                    #     uav_ros.Rmin,
                    # )
                self.update = True
            else:
                self.update = False
            self.packet = []
        comm_info.SEAD_info_clear()

        # 璋冭瘯璺緞鐢熸垚鐘舵€?
        # if self.path_following.path:
        #     print(f"[DEBUG] UAV{getattr(self, 'uav_id', 'Unknown')}: Path exists, length={len(self.path_following.path)}")
        # else:
        #     print(f"[DEBUG] UAV{getattr(self, 'uav_id', 'Unknown')}: No path exists, skipping path following")
        #     return  # 濡傛灉娌℃湁璺緞锛岀洿鎺ヨ繑鍥?

        if self.path_following.path:
            if (
                np.linalg.norm(
                    [
                        self.target[0][0] - uav_ros.local_pose[0],
                        self.target[0][1] - uav_ros.local_pose[1],
                    ]
                )
                <= 5 * waypoint_radius
                and not self.into
            ):
                if self.target[:-1] == []:
                    xbee.send_data_async(
                        gcs,
                        comm_info.pack_record_time_packet(
                            f"mission complete!", new_timer.t()
                        ),
                    )
                    self.control2ga_queue.put([44])
                self.into = True
            if (
                np.linalg.norm(
                    [
                        self.target[0][0] - uav_ros.local_pose[0],
                        self.target[0][1] - uav_ros.local_pose[1],
                    ]
                )
                >= 5 * waypoint_radius
                and self.into
            ):
                if self.target[0][3:]:
                    self.terminated_tasks.append(self.target[0][3:])
                    xbee.send_data_async(
                        gcs,
                        comm_info.pack_record_time_packet(
                            f"task: {self.target[0][3:]} finished", new_timer.t()
                        ),
                    )
                    del self.target[0]
                    self.task_locking = False
                    self.into = False
            if (
                np.linalg.norm(
                    [
                        self.target[0][0] - uav_ros.local_pose[0],
                        self.target[0][1] - uav_ros.local_pose[1],
                    ]
                )
                <= 20 * uav_ros.Rmin
            ):
                if self.target[0][3:]:
                    self.task_locking = True
                else:
                    self.back_to_base = True

            if new_timer.check_period(0.1, self.previous_time_control):
                desirePoint, self.intial_windowIndex, _, error_of_distance = (
                    self.path_following.get_desirePoint_withWindow(
                        uav_ros.v,
                        uav_ros.local_pose[0],
                        uav_ros.local_pose[1],
                        uav_ros.yaw,
                        self.intial_windowIndex,
                    )
                )
                u, self.pre_error = self.path_following.PID_control(
                    uav_ros.v,
                    uav_ros.Rmin,
                    uav_ros.local_pose,
                    uav_ros.yaw,
                    desirePoint,
                    self.pre_error,
                )
                if error_of_distance <= 0 and self.back_to_base:
                    target_V, u = 0, 0
                elif self.back_to_base:
                    pid_velo = 0.8 * np.linalg.norm(
                        [
                            self.base[0] - uav_ros.local_pose[0],
                            self.base[1] - uav_ros.local_pose[1],
                        ]
                    )
                    target_V = pid_velo if pid_velo < uav_ros.v else uav_ros.v
                else:
                    target_V = uav_ros.v
                dt = (
                    time.time() - self.previous_time_control
                    if not self.previous_time_control == 0
                    else 0
                )
                # 璋冭瘯椋炶杩囩▼
                # print(f"[DEBUG] UAV{getattr(self, 'uav_id', 'Unknown')}: pos=({uav_ros.local_pose[0]:.1f}, {uav_ros.local_pose[1]:.1f}, {uav_ros.local_pose[2]:.1f}), vel=({uav_ros.local_velo[0]:.2f}, {uav_ros.local_velo[1]:.2f}), yaw={uav_ros.yaw:.2f}, mode={uav_ros.mode}")
                uav_ros.step(target_V, u, dt)
                self.previous_time_control = time.time()

            for t in self.unknown_targets_set:
                if (
                    np.linalg.norm(
                        [uav_ros.local_pose[0] - t[0], uav_ros.local_pose[1] - t[1]]
                    )
                    <= self.sencing_range
                    and t not in self.targets_set
                    and uav_ros.type != 3
                ):
                    self.targets_set.append(t)
                    self.new_targets.append(t)
                    xbee.send_data_async(
                        gcs,
                        comm_info.pack_record_time_packet(
                            f"discover unknown target: {t}", new_timer.t()
                        ),
                    )
