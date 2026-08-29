"""Central ROS coordinator for scout coverage and worker rescue tasks."""

import threading
from math import atan2, cos, isfinite, sin, sqrt
from typing import Dict, List, Set, Tuple

import rospy
import tf2_ros
from geometry_msgs.msg import Point, Point32, PoseStamped
from mavros_msgs.msg import PositionTarget
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse
from visualization_msgs.msg import Marker, MarkerArray

from xd_uav_controller.msg import ControlState, PathStatus
from xd_uav_task_allocate.core.allocation import (
    TASK_COMPLETED,
    RescueTaskAllocator,
)
from xd_uav_task_allocate.core.coverage import (
    PlannedArea,
    SearchAreaDefinition,
    assign_areas,
    connect_fixedwing_paths,
    fixedwing_entry_path,
    fixedwing_lawnmower_path,
    lawnmower_path,
    path_length,
    verification_search_area,
)
from xd_uav_task_allocate.core.execution import (
    ArrivalDwellTracker,
    fixedwing_waypoint_reached,
    goal_heading,
    worker_approach_goal,
)
from xd_uav_task_allocate.core.geometry import (
    PoseHistory,
    PoseSample,
    RigidTransform,
    relative_frd_to_shared,
    rotate_covariance_frd_to_shared,
)
from xd_uav_task_allocate.core.registry import (
    TARGET_ASSIGNED,
    TARGET_CANDIDATE,
    TARGET_COMPLETED,
    TARGET_CONFIRMED,
    TARGET_EXECUTING,
    TARGET_STALE,
    TARGET_VERIFYING,
    GlobalTargetRegistry,
    TargetObservation,
)
from xd_uav_task_allocate.msg import (
    GlobalTarget,
    GlobalTargetArray,
    PlannerStatus,
    RescueTask,
    RescueTaskArray,
    SearchArea,
    SearchAreaArray,
)
from xd_uav_task_allocate.srv import (
    GetMissionState,
    GetMissionStateResponse,
    LoadSearchAreas,
    LoadSearchAreasResponse,
    SetVehicleEnabled,
    SetVehicleEnabledResponse,
    TargetCommand,
    TargetCommandResponse,
    TaskCommand,
    TaskCommandResponse,
    VehicleCommand,
    VehicleCommandResponse,
)
from xd_uav_track.msg import DetectionArray


MISSION_IDLE = "IDLE"
MISSION_LOADED = "LOADED"
MISSION_ACTIVE = "ACTIVE"
MISSION_PAUSED = "PAUSED"
MISSION_ABORTED = "ABORTED"
MISSION_COMPLETED = "COMPLETED"


class TaskAllocateCoordinator:
    """Owns global IDs and assignments; aircraft planners own collision avoidance."""

    def __init__(self):
        self._lock = threading.RLock()
        self.shared_frame = str(rospy.get_param("~mission/shared_frame", "world")).lstrip("/")
        if not self.shared_frame:
            raise ValueError("mission/shared_frame must not be empty")
        self.pose_sync_tolerance = float(
            rospy.get_param("~geolocation/maximum_pose_time_difference_sec", 0.15)
        )
        self.tf_lookup_timeout = float(
            rospy.get_param("~geolocation/tf_lookup_timeout_sec", 0.05)
        )
        self.search_area_altitude_tolerance = float(
            rospy.get_param("~search_area/maximum_transformed_altitude_variation_m", 0.20)
        )
        self.odometry_timeout = float(
            rospy.get_param("~geolocation/world_odometry_timeout_sec", 0.50)
        )
        self.health_timeout = float(
            rospy.get_param("~vehicle_health/timeout_sec", 0.50)
        )
        self.require_state_valid = bool(
            rospy.get_param("~vehicle_health/require_state_valid", True)
        )
        self.require_localization_valid = bool(
            rospy.get_param("~vehicle_health/require_localization_valid", True)
        )
        self.require_odometry_fresh = bool(
            rospy.get_param("~vehicle_health/require_odometry_fresh", True)
        )
        self.require_all_scouts_ready = bool(
            rospy.get_param("~execution/require_all_scouts_ready", True)
        )
        self.minimum_ready_scouts = max(
            1, int(rospy.get_param("~execution/minimum_ready_scouts", 1))
        )
        self.completion_grace_sec = max(
            0.0, float(rospy.get_param("~execution/completion_grace_sec", 2.0))
        )
        self.planner_backend = str(
            rospy.get_param("~planner/backend", "ego_swarm")
        ).strip()
        if self.planner_backend not in ("ego_swarm", "direct_controller_test"):
            raise ValueError(
                "planner/backend must be ego_swarm or direct_controller_test"
            )
        self.direct_face_goal = bool(
            rospy.get_param("~planner/direct_controller_test/face_goal", True)
        )
        self.direct_yaw_minimum_distance = max(
            0.0,
            float(
                rospy.get_param(
                    "~planner/direct_controller_test/yaw_minimum_distance_m", 0.1
                )
            ),
        )
        self.arrival_tracker = ArrivalDwellTracker(
            tolerance_m=rospy.get_param(
                "~planner/direct_controller_test/goal_tolerance_m", 1.0
            ),
            dwell_sec=rospy.get_param(
                "~planner/direct_controller_test/arrival_dwell_sec", 0.5
            ),
        )
        self.worker_arrival_tracker = ArrivalDwellTracker(
            tolerance_m=rospy.get_param(
                "~allocation/worker_approach/goal_tolerance_m", 0.5
            ),
            dwell_sec=rospy.get_param(
                "~allocation/worker_approach/arrival_dwell_sec", 1.0
            ),
        )
        self.worker_maximum_arrival_speed = max(
            0.0,
            float(
                rospy.get_param(
                    "~allocation/worker_approach/maximum_arrival_speed_mps",
                    0.35,
                )
            ),
        )
        self.worker_state_timeout = float(
            rospy.get_param("~allocation/worker_state_timeout_sec", 1.0)
        )
        self.worker_horizontal_standoff = max(
            0.0,
            float(
                rospy.get_param(
                    "~allocation/worker_approach/horizontal_standoff_m", 5.0
                )
            ),
        )
        self.planner_failure_retry_sec = float(
            rospy.get_param("~allocation/planner_failure_retry_sec", 5.0)
        )
        self.class_priorities = {
            int(key): int(value)
            for key, value in dict(rospy.get_param("~allocation/class_priorities", {})).items()
        }
        self.default_worker_vehicle_types = self._vehicle_type_list(
            rospy.get_param(
                "~allocation/default_worker_vehicle_types", ["multirotor"]
            )
        )
        self.class_worker_vehicle_types = {
            int(key): self._vehicle_type_list(value)
            for key, value in dict(
                rospy.get_param("~allocation/class_worker_vehicle_types", {})
            ).items()
        }
        self.task_duplicate_radius = max(
            0.0,
            float(
                rospy.get_param(
                    "~allocation/task_deduplication_radius_m", 5.0
                )
            ),
        )

        self.scout_configs = dict(rospy.get_param("~scouts", {}))
        self.worker_configs = dict(rospy.get_param("~workers", {}))
        configured_scout_types = {
            self._configured_vehicle_type(dict(config))
            for config in self.scout_configs.values()
        }
        hierarchical_mode = str(
            rospy.get_param("~hierarchical_search/mode", "")
        ).strip().lower()
        legacy_hierarchical_enabled = bool(
            rospy.get_param("~hierarchical_search/enabled", False)
        )
        self.hierarchical_search_enabled = self._resolve_hierarchical_search_mode(
            hierarchical_mode,
            configured_scout_types,
            legacy_hierarchical_enabled,
        )
        self.hierarchical_search_mode = (
            hierarchical_mode if hierarchical_mode else "legacy"
        )
        self.verification_dispatch_policy = str(
            rospy.get_param(
                "~hierarchical_search/verification/dispatch_policy",
                "immediate",
            )
        ).strip().lower()
        if self.verification_dispatch_policy not in (
            "after_coarse_complete",
            "immediate",
        ):
            raise ValueError(
                "hierarchical_search/verification/dispatch_policy must be "
                "after_coarse_complete or immediate"
            )
        self.verification_altitude = float(
            rospy.get_param("~hierarchical_search/verification/altitude_m", 4.0)
        )
        self.verification_lane_spacing = max(
            0.1,
            float(
                rospy.get_param(
                    "~hierarchical_search/verification/lane_spacing_m", 5.0
                )
            ),
        )
        self.verification_minimum_radius = max(
            0.1,
            float(
                rospy.get_param(
                    "~hierarchical_search/verification/minimum_radius_m", 20.0
                )
            ),
        )
        self.verification_maximum_radius = max(
            self.verification_minimum_radius,
            float(
                rospy.get_param(
                    "~hierarchical_search/verification/maximum_radius_m", 80.0
                )
            ),
        )
        self.verification_covariance_sigma = max(
            0.0,
            float(
                rospy.get_param(
                    "~hierarchical_search/verification/covariance_sigma", 3.0
                )
            ),
        )
        self.verification_radius_mode = str(
            rospy.get_param(
                "~hierarchical_search/verification/radius_mode", "covariance"
            )
        ).strip().lower()
        if self.verification_radius_mode not in ("covariance", "fixed"):
            raise ValueError(
                "hierarchical_search/verification/radius_mode must be "
                "covariance or fixed"
            )
        self.verification_fixed_radius = float(
            rospy.get_param(
                "~hierarchical_search/verification/fixed_radius_m", 30.0
            )
        )
        if self.verification_fixed_radius <= 0.0:
            raise ValueError(
                "hierarchical_search/verification/fixed_radius_m must be positive"
            )
        self.verification_maximum_concurrent_regions = max(
            1,
            int(
                rospy.get_param(
                    "~hierarchical_search/verification/maximum_concurrent_regions",
                    1,
                )
            ),
        )
        self.reject_after_verification_route = bool(
            rospy.get_param(
                "~hierarchical_search/verification/reject_after_route", True
            )
        )

        self.registry = GlobalTargetRegistry(
            association_radius_m=rospy.get_param("~target_registry/association_radius_m", 4.0),
            confirmation_hits=rospy.get_param("~target_registry/confirmation_hits", 5),
            confirmation_window_sec=rospy.get_param(
                "~target_registry/confirmation_window_sec", 2.0
            ),
            confirmation_minimum_span_sec=rospy.get_param(
                "~target_registry/confirmation_minimum_span_sec", 0.25
            ),
            confirmation_distinct_uavs=rospy.get_param(
                "~target_registry/confirmation_distinct_uavs", 2
            ),
            stale_timeout_sec=rospy.get_param("~target_registry/stale_timeout_sec", 15.0),
            confirmation_vehicle_types=(
                ("multirotor",)
                if self.hierarchical_search_enabled
                else ("multirotor", "fixedwing")
            ),
            association_covariance_sigma=rospy.get_param(
                "~target_registry/association_covariance_sigma", 3.0
            ),
            maximum_association_radius_m=rospy.get_param(
                "~target_registry/maximum_association_radius_m", 50.0
            ),
            cross_class_association_radius_m=rospy.get_param(
                "~target_registry/cross_class_association_radius_m", 3.0
            ),
            stable_track_maximum_jump_m=rospy.get_param(
                "~target_registry/stable_track_maximum_jump_m", 20.0
            ),
            class_confirmation_minimum_observations=rospy.get_param(
                "~target_registry/class_confirmation_minimum_observations", 3
            ),
            class_confirmation_minimum_ratio=rospy.get_param(
                "~target_registry/class_confirmation_minimum_ratio", 0.65
            ),
            maximum_confirmation_position_spread_m=rospy.get_param(
                "~target_registry/maximum_confirmation_position_spread_m", 3.0
            ),
            position_history_size=rospy.get_param(
                "~target_registry/position_history_size", 30
            ),
        )
        self.allocator = RescueTaskAllocator()

        tf_cache_sec = max(
            10.0,
            float(rospy.get_param("~geolocation/pose_history_sec", 5.0)) + 1.0,
        )
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(tf_cache_sec))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        all_configs = dict(self.scout_configs)
        all_configs.update(self.worker_configs)
        self.vehicle_types = {
            str(name): self._configured_vehicle_type(dict(config))
            for name, config in all_configs.items()
        }
        self.vehicle_backends = {
            str(name): self._configured_backend(dict(config))
            for name, config in all_configs.items()
        }
        self.direct_controller_test = any(
            backend == "direct_controller_test"
            for backend in self.vehicle_backends.values()
        )
        if self.direct_controller_test and not bool(
            rospy.get_param(
                "~planner/direct_controller_test/acknowledge_no_obstacle_avoidance",
                False,
            )
        ):
            raise ValueError(
                "any direct_controller_test vehicle requires explicit "
                "acknowledge_no_obstacle_avoidance=true"
            )
        if not self.scout_configs:
            rospy.logwarn("[task_allocate] no scouts configured")
        if not self.worker_configs:
            rospy.logwarn("[task_allocate] no workers configured")

        self.pose_histories: Dict[str, PoseHistory] = {}
        self.vehicle_world_positions: Dict[str, Tuple[float, Tuple[float, float, float]]] = {}
        self.vehicle_world_speeds: Dict[str, Tuple[float, float]] = {}
        self.vehicle_health: Dict[str, Tuple[float, bool]] = {}
        self.goal_publishers: Dict[str, rospy.Publisher] = {}
        self.direct_goal_publishers: Dict[str, rospy.Publisher] = {}
        self.direct_path_publishers: Dict[str, rospy.Publisher] = {}
        self.direct_status_publishers: Dict[str, rospy.Publisher] = {}
        self.path_publishers: Dict[str, rospy.Publisher] = {}
        self.routes: Dict[str, List[Tuple[float, float, float]]] = {}
        self.route_indices: Dict[str, int] = {}
        self.active_goals: Dict[str, Tuple[int, str, int]] = {}
        self.active_goal_points: Dict[str, Tuple[float, float, float]] = {}
        self.active_goal_origins: Dict[str, Tuple[float, float, float]] = {}
        self.active_trajectory_end_times: Dict[str, float] = {}
        self.active_trajectory_route_progress: Dict[
            str, List[Tuple[float, int]]
        ] = {}
        self.active_controller_paths: Set[str] = set()
        self.verification_pending: List[int] = []
        self.verification_target_by_scout: Dict[str, int] = {}
        self.verification_scout_by_target: Dict[int, str] = {}
        self.verification_plans: Dict[int, PlannedArea] = {}
        self.verification_region_by_target: Dict[int, int] = {}
        self.verification_targets_by_region: Dict[int, Set[int]] = {}
        self.verification_region_states: Dict[int, str] = {}
        self.worker_retry_after: Dict[str, float] = {}
        self.vehicle_operator_enabled = {
            str(name): True for name in all_configs
        }
        self.loaded_areas: List[PlannedArea] = []
        self.loaded_area_stamp = rospy.Time()
        self.mission_state = MISSION_IDLE
        self.mission_detail = "waiting for search areas"
        self.search_completed_at = 0.0
        self._next_goal_id = 1
        self._subscribers = []

        self.search_area_subscriber = rospy.Subscriber(
            str(rospy.get_param("~interfaces/input/search_areas", "/task_allocate/search_areas")),
            SearchAreaArray,
            self._search_areas_callback,
            queue_size=2,
        )
        self.targets_publisher = rospy.Publisher(
            str(rospy.get_param("~interfaces/output/global_targets", "/task_allocate/global_targets")),
            GlobalTargetArray,
            queue_size=2,
            latch=True,
        )
        self.tasks_publisher = rospy.Publisher(
            str(rospy.get_param("~interfaces/output/rescue_tasks", "/task_allocate/rescue_tasks")),
            RescueTaskArray,
            queue_size=2,
            latch=True,
        )
        self.mission_state_publisher = rospy.Publisher(
            str(
                rospy.get_param(
                    "~interfaces/output/mission_state",
                    "/task_allocate/mission_state",
                )
            ),
            String,
            queue_size=2,
            latch=True,
        )
        self.verification_areas_publisher = rospy.Publisher(
            str(
                rospy.get_param(
                    "~interfaces/output/verification_areas",
                    "/task_allocate/verification_areas",
                )
            ),
            SearchAreaArray,
            queue_size=2,
            latch=True,
        )
        self.verification_markers_publisher = rospy.Publisher(
            str(
                rospy.get_param(
                    "~interfaces/output/verification_markers",
                    "/task_allocate/verification_markers",
                )
            ),
            MarkerArray,
            queue_size=2,
            latch=True,
        )
        service_names = dict(rospy.get_param("~interfaces/services", {}))

        def service_name(key: str) -> str:
            return str(service_names.get(key, f"/task_allocate/{key}"))

        self.services = [
            rospy.Service(service_name("start"), Trigger, self._start_callback),
            rospy.Service(service_name("pause"), Trigger, self._pause_callback),
            rospy.Service(service_name("resume"), Trigger, self._resume_callback),
            rospy.Service(service_name("stop"), Trigger, self._stop_callback),
            rospy.Service(service_name("reset"), Trigger, self._reset_callback),
            rospy.Service(service_name("clear_all"), Trigger, self._clear_all_callback),
            rospy.Service(service_name("restart"), Trigger, self._restart_callback),
            rospy.Service(service_name("replan"), Trigger, self._replan_callback),
            rospy.Service(
                service_name("clear_results"), Trigger, self._clear_results_callback
            ),
            rospy.Service(
                service_name("cancel_all_tasks"),
                Trigger,
                self._cancel_all_tasks_callback,
            ),
            rospy.Service(
                service_name("load_search_areas"),
                LoadSearchAreas,
                self._load_search_areas_service_callback,
            ),
            rospy.Service(
                service_name("get_state"),
                GetMissionState,
                self._get_state_callback,
            ),
            rospy.Service(
                service_name("skip_waypoint"),
                VehicleCommand,
                self._skip_waypoint_callback,
            ),
            rospy.Service(
                service_name("set_vehicle_enabled"),
                SetVehicleEnabled,
                self._set_vehicle_enabled_callback,
            ),
            rospy.Service(
                service_name("cancel_task"), TaskCommand, self._cancel_task_callback
            ),
            rospy.Service(
                service_name("retry_task"), TaskCommand, self._retry_task_callback
            ),
            rospy.Service(
                service_name("reject_target"),
                TargetCommand,
                self._reject_target_callback,
            ),
            rospy.Service(
                service_name("retry_verification"),
                TargetCommand,
                self._retry_verification_callback,
            ),
        ]

        for name, config in self.scout_configs.items():
            self._configure_vehicle(str(name), dict(config), is_scout=True)
        for name, config in self.worker_configs.items():
            self.allocator.register_worker(
                str(name), self.vehicle_types[str(name)]
            )
            self._configure_vehicle(str(name), dict(config), is_scout=False)

        update_rate = max(1.0, float(rospy.get_param("~runtime/update_rate_hz", 5.0)))
        self.timer = rospy.Timer(rospy.Duration(1.0 / update_rate), self._timer_callback)
        self._publish_state()
        self._publish_mission_state()
        rospy.loginfo(
            "[task_allocate] ready: scouts=%s workers=%s shared_frame=%s; "
            "position=main world Odometry; vehicles=%s; backends=%s; "
            "hierarchical_search=%s(mode=%s)",
            sorted(self.scout_configs),
            sorted(self.worker_configs),
            self.shared_frame,
            self.vehicle_types,
            self.vehicle_backends,
            self.hierarchical_search_enabled,
            self.hierarchical_search_mode,
        )
        if self.direct_controller_test:
            rospy.logwarn(
                "[task_allocate] DIRECT CONTROLLER TEST MODE: goals bypass obstacle "
                "avoidance; use only in a verified clear test area"
            )

    @staticmethod
    def _topic(config: dict, key: str, fallback: str) -> str:
        return str(dict(config.get("topics", {})).get(key, fallback))

    @staticmethod
    def _vehicle_type_list(value) -> Tuple[str, ...]:
        values = [value] if isinstance(value, str) else list(value)
        normalized = tuple(sorted({str(item).strip().lower() for item in values}))
        if not normalized or any(
            item not in ("multirotor", "fixedwing") for item in normalized
        ):
            raise ValueError(
                "worker vehicle type lists must contain multirotor and/or fixedwing"
            )
        return normalized

    @staticmethod
    def _configured_vehicle_type(config: dict) -> str:
        vehicle_type = str(config.get("vehicle_type", "multirotor")).strip().lower()
        if vehicle_type not in ("multirotor", "fixedwing"):
            raise ValueError("vehicle_type must be multirotor or fixedwing")
        return vehicle_type

    @staticmethod
    def _resolve_hierarchical_search_mode(
        mode: str, scout_vehicle_types, legacy_enabled: bool = False
    ) -> bool:
        """Select reconnaissance flow from policy and configured scout types."""

        normalized = str(mode).strip().lower()
        if not normalized:
            return bool(legacy_enabled)
        if normalized == "auto":
            types = {str(value).strip().lower() for value in scout_vehicle_types}
            return "fixedwing" in types and "multirotor" in types
        if normalized == "hierarchical":
            return True
        if normalized == "single_stage":
            return False
        raise ValueError(
            "hierarchical_search/mode must be auto, hierarchical, or single_stage"
        )

    def _configured_backend(self, config: dict) -> str:
        execution = dict(config.get("execution", {}))
        backend = str(execution.get("backend", self.planner_backend)).strip()
        if backend not in ("ego_swarm", "direct_controller_test"):
            raise ValueError(
                "vehicle execution/backend must be ego_swarm or direct_controller_test"
            )
        return backend

    def _uses_direct_controller(self, vehicle: str) -> bool:
        return self.vehicle_backends.get(str(vehicle)) == "direct_controller_test"

    def _fixedwing_setting(self, vehicle: str, key: str, default: float) -> float:
        config = dict(
            dict(self.scout_configs).get(
                str(vehicle), dict(self.worker_configs).get(str(vehicle), {})
            )
        )
        fixedwing = dict(config.get("fixedwing", {}))
        return float(
            fixedwing.get(
                key,
                rospy.get_param(f"~planner/fixedwing/{key}", default),
            )
        )

    def _coverage_speed(self, vehicle: str) -> float:
        config = dict(
            dict(self.scout_configs).get(
                str(vehicle), dict(self.worker_configs).get(str(vehicle), {})
            )
        )
        coverage = dict(config.get("coverage", {}))
        vehicle_type = self.vehicle_types[str(vehicle)]
        default = 15.0 if vehicle_type == "fixedwing" else 4.0
        return max(
            0.1,
            float(
                coverage.get(
                    "nominal_speed_mps",
                    rospy.get_param(
                        f"~search_allocation/{vehicle_type}_nominal_speed_mps",
                        default,
                    ),
                )
            ),
        )

    @staticmethod
    def _quaternion_yaw(quaternion) -> float:
        x, y, z, w = [float(value) for value in quaternion[:4]]
        norm = sqrt(x * x + y * y + z * z + w * w)
        if norm <= 1e-9:
            raise ValueError("quaternion norm is zero")
        x, y, z, w = x / norm, y / norm, z / norm, w / norm
        return atan2(
            2.0 * (w * z + x * y),
            1.0 - 2.0 * (y * y + z * z),
        )

    @staticmethod
    def _health_topic(config: dict, fallback: str) -> str:
        health = dict(config.get("health", {}))
        return str(health.get("control_state_topic", fallback))

    @staticmethod
    def _odometry_topic(config: dict, fallback: str) -> str:
        localization = dict(config.get("localization", {}))
        return str(localization.get("world_odometry_topic", fallback))

    @staticmethod
    def _canonical_frame(frame_id: str) -> str:
        return str(frame_id).strip().lstrip("/")

    def _world_transform(self, source_frame: str, stamp: float) -> RigidTransform:
        source = self._canonical_frame(source_frame)
        if not source:
            raise ValueError("source frame is empty")
        if source == self.shared_frame:
            return RigidTransform()
        if stamp <= 0.0:
            lookup_stamp = rospy.Time(0)
        else:
            lookup_stamp = rospy.Time.from_sec(stamp)
        try:
            message = self.tf_buffer.lookup_transform(
                self.shared_frame,
                source,
                lookup_stamp,
                rospy.Duration(max(0.0, self.tf_lookup_timeout)),
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as error:
            raise ValueError(
                f"TF {self.shared_frame} <- {source} unavailable at {stamp:.6f}: {error}"
            )
        transform = message.transform
        return RigidTransform(
            translation=(
                float(transform.translation.x),
                float(transform.translation.y),
                float(transform.translation.z),
            ),
            rotation=(
                float(transform.rotation.x),
                float(transform.rotation.y),
                float(transform.rotation.z),
                float(transform.rotation.w),
            ),
        )

    def _configure_vehicle(self, name: str, config: dict, is_scout: bool) -> None:
        self.pose_histories[name] = PoseHistory(
            maximum_age_sec=rospy.get_param("~geolocation/pose_history_sec", 5.0)
        )
        if self._uses_direct_controller(name):
            self.direct_goal_publishers[name] = rospy.Publisher(
                self._topic(
                    config,
                    "controller_setpoint",
                    f"/{name}/control/reference/setpoint",
                ),
                PositionTarget,
                queue_size=2,
            )
            self.direct_status_publishers[name] = rospy.Publisher(
                self._topic(config, "planner_status", f"/{name}/planning/status"),
                PlannerStatus,
                queue_size=5,
                latch=True,
            )
            if self.vehicle_types[name] == "fixedwing":
                self.direct_path_publishers[name] = rospy.Publisher(
                    self._topic(
                        config,
                        "controller_path",
                        f"/{name}/control/reference/path",
                    ),
                    Path,
                    queue_size=1,
                    latch=False,
                )
                self._subscribers.append(
                    rospy.Subscriber(
                        self._topic(
                            config,
                            "controller_path_status",
                            f"/{name}/controller/path_status",
                        ),
                        PathStatus,
                        lambda message, vehicle=name: self._path_status_callback(
                            vehicle, message
                        ),
                        queue_size=10,
                    )
                )
        else:
            self.goal_publishers[name] = rospy.Publisher(
                self._topic(config, "planner_goal", f"/{name}/planning/goal"),
                PoseStamped,
                queue_size=2,
                latch=True,
            )
        self.path_publishers[name] = rospy.Publisher(
            self._topic(config, "mission_path", f"/{name}/planning/mission_path"),
            Path,
            queue_size=1,
            latch=True,
        )
        self._subscribers.append(
            rospy.Subscriber(
                self._odometry_topic(
                    config,
                    f"/{name}/state_estimator/main/frames/{self.shared_frame}/odom",
                ),
                Odometry,
                lambda message, vehicle=name, worker=not is_scout: self._odometry_callback(
                    vehicle, worker, message
                ),
                queue_size=50,
            )
        )
        self._subscribers.append(
            rospy.Subscriber(
                self._health_topic(config, f"/{name}/control_manager/state"),
                ControlState,
                lambda message, vehicle=name, worker=not is_scout: self._health_callback(
                    vehicle, worker, message
                ),
                queue_size=50,
            )
        )
        if not self._uses_direct_controller(name):
            self._subscribers.append(
                rospy.Subscriber(
                    self._topic(config, "planner_status", f"/{name}/planning/status"),
                    PlannerStatus,
                    lambda message, vehicle=name: self._planner_status_callback(
                        vehicle, message
                    ),
                    queue_size=10,
                )
            )
        if is_scout:
            self._subscribers.append(
                rospy.Subscriber(
                    self._topic(config, "detections", f"/{name}/track/detections"),
                    DetectionArray,
                    lambda message, vehicle=name: self._detection_callback(vehicle, message),
                    queue_size=10,
                )
            )

    def _vehicle_ready(self, name: str, now: float) -> bool:
        position = self.vehicle_world_positions.get(name)
        health = self.vehicle_health.get(name)
        execution_connected = not self._uses_direct_controller(name)
        if self._uses_direct_controller(name):
            execution_connected = bool(
                name in self.direct_goal_publishers
                and self.direct_goal_publishers[name].get_num_connections() > 0
            )
            if self.vehicle_types.get(name) == "fixedwing":
                execution_connected = bool(
                    execution_connected
                    and name in self.direct_path_publishers
                    and self.direct_path_publishers[name].get_num_connections() > 0
                )
        return bool(
            self.vehicle_operator_enabled.get(str(name), True)
            and position is not None
            and health is not None
            and health[1]
            and execution_connected
            and 0.0 <= now - position[0] <= self.odometry_timeout
            and 0.0 <= now - health[0] <= self.health_timeout
        )

    def _refresh_worker(self, name: str, now: float) -> None:
        position = self.vehicle_world_positions.get(name)
        if position is None:
            worker = self.allocator.workers.get(name)
            if worker is not None:
                worker.online = False
            return
        online = bool(
            self._vehicle_ready(name, now)
            and now >= self.worker_retry_after.get(name, 0.0)
        )
        if online:
            self.allocator.update_worker(name, position[1], now, online=True)
            self._dispatch_assignments()
        else:
            # Do not release a task on one invalid sample. Mark the worker
            # unavailable for new work; the timer releases an existing task
            # only after worker_state_timeout has elapsed continuously.
            worker = self.allocator.workers.get(name)
            if worker is not None:
                worker.online = False

    def _odometry_callback(self, name: str, is_worker: bool, message: Odometry) -> None:
        stamp = message.header.stamp.to_sec()
        if stamp <= 0.0:
            rospy.logwarn_throttle(
                2.0, "[task_allocate] %s world odometry has zero timestamp", name
            )
            return
        frame_id = self._canonical_frame(message.header.frame_id)
        if frame_id != self.shared_frame:
            rospy.logwarn_throttle(
                2.0,
                "[task_allocate] %s world odometry frame is '%s', expected '%s'",
                name,
                frame_id or "<empty>",
                self.shared_frame,
            )
            return
        pose = PoseSample(
            stamp=stamp,
            position_reference=(
                float(message.pose.pose.position.x),
                float(message.pose.pose.position.y),
                float(message.pose.pose.position.z),
            ),
            orientation_reference_body=(
                float(message.pose.pose.orientation.x),
                float(message.pose.pose.orientation.y),
                float(message.pose.pose.orientation.z),
                float(message.pose.pose.orientation.w),
            ),
            frame_id=frame_id,
        )
        try:
            world_position = RigidTransform().apply(pose.position_reference)
            # Also validate the orientation quaternion before buffering it.
            relative_frd_to_shared((0.0, 0.0, 0.0), pose)
        except ValueError as error:
            rospy.logwarn_throttle(
                2.0, "[task_allocate] %s invalid world odometry: %s", name, error
            )
            return
        with self._lock:
            self.pose_histories[name].add(pose)
            receive_time = rospy.Time.now().to_sec()
            self.vehicle_world_positions[name] = (receive_time, world_position)
            velocity = message.twist.twist.linear
            speed = sqrt(
                float(velocity.x) ** 2
                + float(velocity.y) ** 2
                + float(velocity.z) ** 2
            )
            if not isfinite(speed):
                rospy.logwarn_throttle(
                    2.0, "[task_allocate] %s world odometry speed is invalid", name
                )
                return
            self.vehicle_world_speeds[name] = (
                receive_time,
                speed,
            )
            if is_worker:
                self._refresh_worker(name, receive_time)

    def _health_callback(self, name: str, is_worker: bool, message: ControlState) -> None:
        stamp = message.header.stamp.to_sec()
        if stamp <= 0.0:
            rospy.logwarn_throttle(
                2.0, "[task_allocate] %s control state has zero timestamp", name
            )
            return
        expected_type = (
            ControlState.VEHICLE_FIXEDWING
            if self.vehicle_types[name] == "fixedwing"
            else ControlState.VEHICLE_MULTIROTOR
        )
        type_matches = int(message.vehicle_type) == int(expected_type)
        if not type_matches:
            rospy.logerr_throttle(
                2.0,
                "[task_allocate] %s ControlState vehicle_type=%d conflicts with configured %s",
                name,
                int(message.vehicle_type),
                self.vehicle_types[name],
            )
        healthy = bool(
            type_matches
            and (message.state_valid or not self.require_state_valid)
            and (message.localization_valid or not self.require_localization_valid)
            and (message.odometry_fresh or not self.require_odometry_fresh)
        )
        with self._lock:
            receive_time = rospy.Time.now().to_sec()
            self.vehicle_health[name] = (receive_time, healthy)
            if is_worker:
                self._refresh_worker(name, receive_time)

    def _publish_mission_state(self) -> None:
        self.mission_state_publisher.publish(String(data=self.mission_state))

    def _ready_scout_positions(self, now: float):
        positions = {}
        for name in self.scout_configs:
            position = self.vehicle_world_positions.get(name)
            if position is not None and self._vehicle_ready(name, now):
                positions[name] = position[1]
        return positions

    def _required_scout_count(self) -> int:
        configured = len(self.scout_configs)
        if self.require_all_scouts_ready:
            return configured
        return min(configured, self.minimum_ready_scouts)

    def _prepare_search_routes(self, scout_positions) -> None:
        coverage_positions = dict(scout_positions)
        if self.hierarchical_search_enabled:
            coverage_positions = {
                name: position
                for name, position in scout_positions.items()
                if self.vehicle_types[name] == "fixedwing"
            }
            if not coverage_positions:
                raise ValueError(
                    "hierarchical search requires at least one ready fixed-wing scout"
                )
            if not any(
                self.vehicle_types[name] == "multirotor"
                for name in scout_positions
            ):
                raise ValueError(
                    "hierarchical search requires at least one ready multirotor scout"
                )
        fixedwing_area_paths = {}
        vehicle_area_lengths = {}
        fixedwing_parameters = {}
        for scout in coverage_positions:
            if self.vehicle_types[scout] != "fixedwing":
                continue
            turn_radius = max(
                1.0,
                self._fixedwing_setting(scout, "minimum_turn_radius_m", 30.0),
            )
            waypoint_spacing = max(
                1.0,
                self._fixedwing_setting(
                    scout, "turn_waypoint_spacing_m", 15.0
                ),
            )
            straight_lead_distance = max(
                0.0,
                self._fixedwing_setting(
                    scout, "straight_lead_distance_m", turn_radius
                ),
            )
            fixedwing_parameters[scout] = (
                turn_radius,
                waypoint_spacing,
                straight_lead_distance,
            )
            for planned in self.loaded_areas:
                coarse_lane_spacing = self._fixedwing_setting(
                    scout, "coverage_lane_spacing_m", 0.0
                )
                coarse_altitude = self._fixedwing_setting(
                    scout, "coverage_altitude_m", 0.0
                )
                fixedwing_area = planned.area
                if coarse_lane_spacing > 0.0 or coarse_altitude > 0.0:
                    # A mixed fleet must not force the fixed-wing coarse scan
                    # to use the multirotor verification swath.  The override
                    # is the sensor's effective ground footprint at the
                    # fixed-wing search altitude. Zero keeps the area value.
                    fixedwing_area = SearchAreaDefinition(
                        area_id=planned.area.area_id,
                        boundary=planned.area.boundary,
                        altitude=(
                            coarse_altitude
                            if coarse_altitude > 0.0
                            else planned.area.altitude
                        ),
                        lane_spacing=(
                            coarse_lane_spacing
                            if coarse_lane_spacing > 0.0
                            else planned.area.lane_spacing
                        ),
                        priority=planned.area.priority,
                    )
                area_path = fixedwing_lawnmower_path(
                    fixedwing_area,
                    minimum_turn_radius=turn_radius,
                    turn_waypoint_spacing=waypoint_spacing,
                    straight_lead_distance=straight_lead_distance,
                )
                key = (scout, planned.area.area_id)
                fixedwing_area_paths[key] = area_path
                vehicle_area_lengths[key] = path_length(area_path)
        assignments = assign_areas(
            self.loaded_areas,
            coverage_positions,
            scout_speeds={
                scout: self._coverage_speed(scout) for scout in coverage_positions
            },
            vehicle_area_lengths=vehicle_area_lengths,
        )
        self.routes.clear()
        self.route_indices.clear()
        for scout in scout_positions:
            self.routes[scout] = []
            self.route_indices[scout] = 0
        for scout, areas in assignments.items():
            if self.vehicle_types[scout] == "fixedwing":
                turn_radius, waypoint_spacing, _ = fixedwing_parameters[scout]
                route = connect_fixedwing_paths(
                    [
                        fixedwing_area_paths[(scout, planned.area.area_id)]
                        for planned in areas
                    ],
                    minimum_turn_radius=turn_radius,
                    turn_waypoint_spacing=waypoint_spacing,
                )
                latest_pose = self.pose_histories[scout].latest()
                if latest_pose is not None:
                    route = fixedwing_entry_path(
                        coverage_positions[scout],
                        self._quaternion_yaw(
                            latest_pose.orientation_reference_body
                        ),
                        route,
                        minimum_turn_radius=turn_radius,
                        turn_waypoint_spacing=waypoint_spacing,
                    )
            else:
                route = [point for area in areas for point in area.path]
            self.routes[scout] = route
            self._publish_path(scout, route, self.loaded_area_stamp)
            rospy.loginfo(
                "[task_allocate] scout %s prepared areas=%s waypoints=%d",
                scout,
                [area.area.area_id for area in areas],
                len(route),
            )

    def _start_callback(self, _request) -> TriggerResponse:
        with self._lock:
            if self.mission_state != MISSION_LOADED or not self.loaded_areas:
                return TriggerResponse(
                    success=False,
                    message=f"mission is {self.mission_state}; load search areas first",
                )
            now = rospy.Time.now().to_sec()
            positions = self._ready_scout_positions(now)
            required = self._required_scout_count()
            if len(positions) < required:
                self.mission_detail = (
                    f"waiting for ready scouts: {len(positions)}/{required}; "
                    "call start again after takeoff"
                )
                self._publish_mission_state()
                return TriggerResponse(success=False, message=self.mission_detail)

            try:
                self._prepare_search_routes(positions)
            except ValueError as error:
                self.mission_detail = str(error)
                return TriggerResponse(success=False, message=self.mission_detail)
            if not any(self.routes.values()):
                self.mission_detail = "no executable route was generated"
                return TriggerResponse(success=False, message=self.mission_detail)
            self.mission_state = MISSION_ACTIVE
            self.mission_detail = "search execution started by operator"
            self.search_completed_at = 0.0
            self._publish_mission_state()
            for scout in sorted(self.routes):
                self._publish_next_scout_goal(scout)
            self._dispatch_assignments()
            return TriggerResponse(success=True, message=self.mission_detail)

    def _clear_route_state(self) -> None:
        for vehicle in list(self.path_publishers):
            self._publish_path(vehicle, [], rospy.Time.now())
        self.routes.clear()
        self.route_indices.clear()
        self.active_goals.clear()
        self.active_goal_points.clear()
        self.active_goal_origins.clear()
        self.active_trajectory_end_times.clear()
        self.active_trajectory_route_progress.clear()
        self.active_controller_paths.clear()
        self.verification_pending.clear()
        self.verification_target_by_scout.clear()
        self.verification_scout_by_target.clear()
        self.verification_plans.clear()
        self.verification_region_by_target.clear()
        self.verification_targets_by_region.clear()
        self.verification_region_states.clear()
        self._publish_verification_areas()
        self.arrival_tracker.reset_all()
        self.worker_arrival_tracker.reset_all()
        self.search_completed_at = 0.0

    def _clear_results_data(self) -> None:
        self.registry.clear(reset_ids=True)
        self.allocator.clear_tasks(reset_ids=True)
        self.worker_retry_after.clear()

    def _reset_mission_data(self, preserve_areas: bool) -> None:
        self._clear_route_state()
        self._clear_results_data()
        self._next_goal_id = 1
        if not preserve_areas:
            self.loaded_areas = []
            self.loaded_area_stamp = rospy.Time()
        self.mission_state = (
            MISSION_LOADED if self.loaded_areas else MISSION_IDLE
        )
        self.mission_detail = (
            "mission reset; search areas retained and ready to start"
            if self.loaded_areas
            else "mission cleared; waiting for search areas"
        )
        self._publish_mission_state()
        self._publish_state()

    def _publish_direct_hold(self, vehicle: str) -> bool:
        position = self.vehicle_world_positions.get(vehicle)
        if position is None or not self._uses_direct_controller(vehicle):
            return False
        if self.vehicle_types[vehicle] == "fixedwing":
            latest_pose = self.pose_histories[vehicle].latest()
            if latest_pose is None:
                return False
            course = self._quaternion_yaw(latest_pose.orientation_reference_body)
            speed = self._coverage_speed(vehicle)
            setpoint = PositionTarget()
            setpoint.header.stamp = rospy.Time.now()
            setpoint.header.frame_id = self.shared_frame
            setpoint.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
            setpoint.type_mask = (
                PositionTarget.IGNORE_VZ
                | PositionTarget.IGNORE_AFX
                | PositionTarget.IGNORE_AFY
                | PositionTarget.IGNORE_AFZ
                | PositionTarget.IGNORE_YAW
                | PositionTarget.IGNORE_YAW_RATE
            )
            setpoint.position.x = float(position[1][0])
            setpoint.position.y = float(position[1][1])
            setpoint.position.z = float(position[1][2])
            setpoint.velocity.x = speed * cos(course)
            setpoint.velocity.y = speed * sin(course)
            self.direct_goal_publishers[vehicle].publish(setpoint)
            self.active_trajectory_end_times.pop(vehicle, None)
            self.active_trajectory_route_progress.pop(vehicle, None)
            self.active_controller_paths.discard(vehicle)
            return True
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = self.shared_frame
        goal.pose.position.x = float(position[1][0])
        goal.pose.position.y = float(position[1][1])
        goal.pose.position.z = float(position[1][2])
        goal.pose.orientation.w = 1.0
        self._publish_direct_controller_goal(vehicle, goal)
        return True

    def _pause_active_vehicles(self) -> Tuple[bool, str]:
        unsupported = [
            vehicle
            for vehicle in self.active_goals
            if not self._uses_direct_controller(vehicle)
        ]
        if unsupported:
            return (
                False,
                "pause/stop requires a planner cancel adapter for: "
                + ", ".join(sorted(unsupported)),
            )
        for vehicle in list(self.active_goals):
            self._publish_direct_hold(vehicle)
        return True, ""

    def _republish_active_goal(self, vehicle: str) -> None:
        active = self.active_goals.get(vehicle)
        point = self.active_goal_points.get(vehicle)
        if active is None or point is None:
            return
        if (
            self._uses_direct_controller(vehicle)
            and self.vehicle_types.get(vehicle) == "fixedwing"
            and active[1] in ("search", "verification")
        ):
            self._clear_active_goal(vehicle)
            self._publish_next_scout_goal(vehicle)
            return
        goal = PoseStamped()
        goal.header.seq = int(active[0])
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = self.shared_frame
        goal.pose.position.x = float(point[0])
        goal.pose.position.y = float(point[1])
        goal.pose.position.z = float(point[2])
        goal.pose.orientation.w = 1.0
        if self._uses_direct_controller(vehicle):
            self._publish_direct_controller_goal(vehicle, goal)
        else:
            self.goal_publishers[vehicle].publish(goal)

    def _pause_callback(self, _request) -> TriggerResponse:
        with self._lock:
            if self.mission_state != MISSION_ACTIVE:
                return TriggerResponse(
                    False, f"mission is {self.mission_state}; only ACTIVE can pause"
                )
            success, detail = self._pause_active_vehicles()
            if not success:
                return TriggerResponse(False, detail)
            self.mission_state = MISSION_PAUSED
            self.mission_detail = "mission paused by operator; progress retained"
            self._publish_mission_state()
            return TriggerResponse(True, self.mission_detail)

    def _resume_callback(self, _request) -> TriggerResponse:
        with self._lock:
            if self.mission_state != MISSION_PAUSED:
                return TriggerResponse(
                    False, f"mission is {self.mission_state}; only PAUSED can resume"
                )
            self.mission_state = MISSION_ACTIVE
            self.mission_detail = "mission resumed by operator"
            self._publish_mission_state()
            for vehicle in list(self.active_goals):
                self._republish_active_goal(vehicle)
            for scout in sorted(self.routes):
                if (
                    scout not in self.active_goals
                    and self.route_indices.get(scout, 0)
                    < len(self.routes.get(scout, []))
                ):
                    self._publish_next_scout_goal(scout)
            self._dispatch_assignments()
            return TriggerResponse(True, self.mission_detail)

    def _stop_execution(self) -> Tuple[bool, str]:
        if self.mission_state not in (MISSION_ACTIVE, MISSION_PAUSED):
            return False, f"mission is {self.mission_state}; nothing is running"
        success, detail = self._pause_active_vehicles()
        if not success:
            return False, detail
        for vehicle, active in list(self.active_goals.items()):
            if active[1] == "rescue":
                self.allocator.cancel_task(
                    active[2], "mission stopped by operator"
                )
            self._clear_active_goal(vehicle)
        self.mission_state = MISSION_ABORTED
        self.mission_detail = "mission stopped by operator; areas and results retained"
        self._publish_mission_state()
        self._publish_state()
        return True, self.mission_detail

    def _stop_callback(self, _request) -> TriggerResponse:
        with self._lock:
            success, detail = self._stop_execution()
            return TriggerResponse(success, detail)

    def _reset_callback(self, _request) -> TriggerResponse:
        with self._lock:
            if self.mission_state in (MISSION_ACTIVE, MISSION_PAUSED):
                success, detail = self._stop_execution()
                if not success:
                    return TriggerResponse(False, detail)
            self._reset_mission_data(preserve_areas=True)
            return TriggerResponse(True, self.mission_detail)

    def _clear_all_callback(self, _request) -> TriggerResponse:
        with self._lock:
            if self.mission_state in (MISSION_ACTIVE, MISSION_PAUSED):
                success, detail = self._stop_execution()
                if not success:
                    return TriggerResponse(False, detail)
            self._reset_mission_data(preserve_areas=False)
            return TriggerResponse(True, self.mission_detail)

    def _restart_callback(self, request) -> TriggerResponse:
        with self._lock:
            if not self.loaded_areas:
                return TriggerResponse(False, "no search areas are loaded")
            if self.mission_state in (MISSION_ACTIVE, MISSION_PAUSED):
                success, detail = self._stop_execution()
                if not success:
                    return TriggerResponse(False, detail)
            self._reset_mission_data(preserve_areas=True)
            response = self._start_callback(request)
            if not response.success:
                response.message = "restart reset succeeded, but start failed: " + response.message
            return response

    def _replan_callback(self, _request) -> TriggerResponse:
        with self._lock:
            if self.mission_state not in (MISSION_ACTIVE, MISSION_PAUSED):
                return TriggerResponse(
                    False,
                    f"mission is {self.mission_state}; replan requires ACTIVE or PAUSED",
                )
            if self.verification_pending or self.verification_target_by_scout:
                return TriggerResponse(
                    False, "finish or reject active target verification before replanning"
                )
            was_active = self.mission_state == MISSION_ACTIVE
            if was_active:
                success, detail = self._pause_active_vehicles()
                if not success:
                    return TriggerResponse(False, detail)
            now = rospy.Time.now().to_sec()
            positions = self._ready_scout_positions(now)
            if len(positions) < self._required_scout_count():
                return TriggerResponse(
                    False, "not enough ready scouts to regenerate coverage routes"
                )
            for vehicle, active in list(self.active_goals.items()):
                if active[1] in ("search", "verification"):
                    self._clear_active_goal(vehicle)
            try:
                self._prepare_search_routes(positions)
            except ValueError as error:
                return TriggerResponse(False, str(error))
            self.search_completed_at = 0.0
            if was_active:
                for scout in sorted(self.routes):
                    self._publish_next_scout_goal(scout)
            self.mission_detail = (
                "coverage routes regenerated from current scout positions; "
                + ("execution continued" if was_active else "mission remains paused")
            )
            return TriggerResponse(True, self.mission_detail)

    def _clear_results_callback(self, _request) -> TriggerResponse:
        with self._lock:
            if self.mission_state in (MISSION_ACTIVE, MISSION_PAUSED):
                return TriggerResponse(
                    False, "stop or reset the mission before clearing results"
                )
            self._clear_results_data()
            self.verification_pending.clear()
            self.verification_target_by_scout.clear()
            self.verification_scout_by_target.clear()
            self.verification_plans.clear()
            self.verification_region_by_target.clear()
            self.verification_targets_by_region.clear()
            self.verification_region_states.clear()
            self._publish_verification_areas()
            self.mission_detail = "targets and rescue-task results cleared"
            self._publish_state()
            return TriggerResponse(True, self.mission_detail)

    def _cancel_all_tasks_callback(self, _request) -> TriggerResponse:
        with self._lock:
            unsupported = [
                vehicle
                for vehicle, active in self.active_goals.items()
                if active[1] == "rescue" and not self._uses_direct_controller(vehicle)
            ]
            if unsupported:
                return TriggerResponse(
                    False,
                    "cancel_all_tasks requires planner cancel adapters for: "
                    + ", ".join(sorted(unsupported)),
                )
            cancelled = 0
            for task in list(self.allocator.tasks.values()):
                if task.status == TASK_COMPLETED:
                    continue
                for vehicle, active in list(self.active_goals.items()):
                    if active[1] == "rescue" and active[2] == task.task_id:
                        self._publish_direct_hold(vehicle)
                        self._clear_active_goal(vehicle)
                if self.allocator.cancel_task(
                    task.task_id, "all rescue tasks cancelled by operator"
                ) is not None:
                    self.registry.set_status(task.target_id, TARGET_CONFIRMED)
                    cancelled += 1
            self._publish_state()
            return TriggerResponse(True, f"cancelled {cancelled} rescue task(s)")

    def _get_state_callback(self, _request) -> GetMissionStateResponse:
        with self._lock:
            route_progress = [
                f"{vehicle}:{self.route_indices.get(vehicle, 0)}/{len(route)}"
                for vehicle, route in sorted(self.routes.items())
            ]
            active_goals = [
                f"{vehicle}:goal={active[0]},kind={active[1]},object={active[2]}"
                for vehicle, active in sorted(self.active_goals.items())
            ]
            return GetMissionStateResponse(
                success=True,
                state=self.mission_state,
                detail=self.mission_detail,
                loaded_area_count=len(self.loaded_areas),
                target_count=len(self.registry.targets),
                pending_verification_count=(
                    len(self.verification_pending)
                    + len(self.verification_target_by_scout)
                ),
                task_count=len(self.allocator.tasks),
                route_progress=route_progress,
                active_goals=active_goals,
            )

    def _skip_waypoint_callback(self, request) -> VehicleCommandResponse:
        with self._lock:
            vehicle = str(request.vehicle_name).strip()
            if vehicle not in self.scout_configs:
                return VehicleCommandResponse(False, f"unknown scout: {vehicle}")
            if self.mission_state not in (MISSION_ACTIVE, MISSION_PAUSED):
                return VehicleCommandResponse(
                    False, f"mission is {self.mission_state}; no active route"
                )
            index = self.route_indices.get(vehicle, 0)
            route = self.routes.get(vehicle, [])
            if index >= len(route):
                return VehicleCommandResponse(False, f"{vehicle} route is complete")
            active = self.active_goals.get(vehicle)
            if active is not None and active[1] not in ("search", "verification"):
                return VehicleCommandResponse(False, f"{vehicle} is executing rescue work")
            if active is not None and not self._uses_direct_controller(vehicle):
                return VehicleCommandResponse(
                    False, f"{vehicle} planner has no cancel/skip adapter"
                )
            self._publish_direct_hold(vehicle)
            self._clear_active_goal(vehicle)
            self.route_indices[vehicle] = index + 1
            if self.mission_state == MISSION_ACTIVE:
                self._publish_next_scout_goal(vehicle)
            return VehicleCommandResponse(
                True, f"{vehicle} skipped route waypoint {index}"
            )

    def _set_vehicle_enabled_callback(
        self, request
    ) -> SetVehicleEnabledResponse:
        with self._lock:
            vehicle = str(request.vehicle_name).strip()
            if vehicle not in self.vehicle_types:
                return SetVehicleEnabledResponse(False, f"unknown vehicle: {vehicle}")
            enabled = bool(request.enabled)
            active = self.active_goals.get(vehicle)
            if (
                not enabled
                and active is not None
                and not self._uses_direct_controller(vehicle)
            ):
                return SetVehicleEnabledResponse(
                    False, f"{vehicle} planner has no cancel adapter"
                )
            self.vehicle_operator_enabled[vehicle] = enabled
            if not enabled and active is not None:
                self._publish_direct_hold(vehicle)
                if active[1] == "rescue":
                    task = self.allocator.release_worker(
                        vehicle, "vehicle disabled by operator; task released"
                    )
                    if task is not None:
                        self.registry.set_status(task.target_id, TARGET_CONFIRMED)
                self._clear_active_goal(vehicle)
            worker = self.allocator.workers.get(vehicle)
            if worker is not None and not enabled:
                worker.online = False
            if (
                enabled
                and self.mission_state == MISSION_ACTIVE
                and vehicle in self.scout_configs
                and vehicle not in self.active_goals
                and self.route_indices.get(vehicle, 0)
                < len(self.routes.get(vehicle, []))
            ):
                self._publish_next_scout_goal(vehicle)
            detail = f"{vehicle} operator eligibility set to {enabled}"
            return SetVehicleEnabledResponse(True, detail)

    def _cancel_task_callback(self, request) -> TaskCommandResponse:
        with self._lock:
            task = self.allocator.tasks.get(int(request.task_id))
            if task is None:
                return TaskCommandResponse(False, f"unknown task: {request.task_id}")
            unsupported = [
                vehicle
                for vehicle, active in self.active_goals.items()
                if active[1] == "rescue"
                and active[2] == task.task_id
                and not self._uses_direct_controller(vehicle)
            ]
            if unsupported:
                return TaskCommandResponse(
                    False, f"{unsupported[0]} planner has no cancel adapter"
                )
            for vehicle, active in list(self.active_goals.items()):
                if active[1] == "rescue" and active[2] == task.task_id:
                    self._publish_direct_hold(vehicle)
                    self._clear_active_goal(vehicle)
            cancelled = self.allocator.cancel_task(
                task.task_id, "rescue task cancelled by operator"
            )
            if cancelled is None:
                return TaskCommandResponse(False, "completed tasks cannot be cancelled")
            self.registry.set_status(task.target_id, TARGET_CONFIRMED)
            self._publish_state()
            return TaskCommandResponse(True, f"task {task.task_id} cancelled")

    def _retry_task_callback(self, request) -> TaskCommandResponse:
        with self._lock:
            task = self.allocator.retry_task(
                int(request.task_id), "rescue task queued again by operator"
            )
            if task is None:
                return TaskCommandResponse(
                    False, "unknown or completed task cannot be retried"
                )
            if task.target_id not in self.registry.targets:
                self.allocator.cancel_task(task.task_id, "linked target no longer exists")
                return TaskCommandResponse(False, "linked target no longer exists")
            self.registry.set_status(task.target_id, TARGET_CONFIRMED)
            self._dispatch_assignments()
            self._publish_state()
            return TaskCommandResponse(True, f"task {task.task_id} queued for allocation")

    def _reject_target_callback(self, request) -> TargetCommandResponse:
        with self._lock:
            target_id = int(request.target_id)
            if target_id not in self.registry.targets:
                return TargetCommandResponse(False, f"unknown target: {target_id}")
            unsupported = [
                vehicle
                for vehicle, active in self.active_goals.items()
                if (
                    (
                        active[1] == "rescue"
                        and self.allocator.tasks.get(active[2]) is not None
                        and self.allocator.tasks[active[2]].target_id == target_id
                    )
                )
                and not self._uses_direct_controller(vehicle)
            ]
            if unsupported:
                return TargetCommandResponse(
                    False,
                    "reject_target requires planner cancel adapters for: "
                    + ", ".join(sorted(unsupported)),
                )
            task = self.allocator.remove_target_task(target_id)
            if task is not None:
                for vehicle, active in list(self.active_goals.items()):
                    if active[1] == "rescue" and active[2] == task.task_id:
                        self._publish_direct_hold(vehicle)
                        self._clear_active_goal(vehicle)
            region_id = self.verification_region_by_target.pop(target_id, None)
            if region_id is not None:
                members = self.verification_targets_by_region.get(region_id, set())
                members.discard(target_id)
                if not members and region_id in self.verification_pending:
                    self.verification_pending = [
                        item for item in self.verification_pending if item != region_id
                    ]
                    self.verification_plans.pop(region_id, None)
                    self.verification_targets_by_region.pop(region_id, None)
                    self.verification_region_states.pop(region_id, None)
            self.registry.remove(target_id)
            self._publish_verification_areas()
            self._publish_state()
            return TargetCommandResponse(True, f"target {target_id} rejected and removed")

    def _retry_verification_callback(self, request) -> TargetCommandResponse:
        with self._lock:
            target_id = int(request.target_id)
            if not self.hierarchical_search_enabled:
                return TargetCommandResponse(False, "hierarchical search is disabled")
            if self.mission_state != MISSION_ACTIVE:
                return TargetCommandResponse(False, "mission must be ACTIVE")
            target = self.registry.targets.get(target_id)
            if target is None:
                return TargetCommandResponse(False, f"unknown target: {target_id}")
            region_id = self.verification_region_by_target.get(target_id)
            if region_id is not None and self.verification_region_states.get(
                region_id
            ) in ("pending", "active"):
                return TargetCommandResponse(False, "target verification is already pending")
            if target.status in (
                TARGET_ASSIGNED,
                TARGET_EXECUTING,
                TARGET_COMPLETED,
            ):
                return TargetCommandResponse(False, "target already has rescue progress")
            if region_id is None:
                self._queue_target_verification(target)
                self._publish_state()
                return TargetCommandResponse(
                    True, f"target {target_id} verification region queued"
                )
            self.registry.set_status(target_id, TARGET_VERIFYING)
            self.verification_region_states[region_id] = "pending"
            self.verification_pending.append(region_id)
            self._publish_verification_areas()
            self._assign_pending_verifications()
            self._publish_state()
            return TargetCommandResponse(True, f"target {target_id} verification queued")

    def _all_scout_routes_complete(self) -> bool:
        return bool(self.routes) and all(
            self.route_indices.get(name, 0) >= len(vehicle_route)
            for name, vehicle_route in self.routes.items()
        )

    def _coarse_search_routes_complete(self) -> bool:
        """Return whether every fixed-wing coarse route has finished."""

        fixedwing_scouts = [
            name
            for name in self.scout_configs
            if self.vehicle_types.get(name) == "fixedwing"
        ]
        return bool(fixedwing_scouts) and all(
            self.route_indices.get(name, 0) >= len(self.routes.get(name, []))
            for name in fixedwing_scouts
        )

    def _mark_search_complete_if_ready(self) -> None:
        if (
            self._all_scout_routes_complete()
            and not self.verification_pending
            and not self.verification_target_by_scout
            and self.search_completed_at <= 0.0
        ):
            self.search_completed_at = rospy.Time.now().to_sec()
            rospy.loginfo(
                "[task_allocate] coarse and verification routes complete; "
                "waiting %.1fs for final detections",
                self.completion_grace_sec,
            )

    def _build_verification_plan(self, target) -> PlannedArea:
        return verification_search_area(
            target_id=int(target.target_id),
            center=target.position,
            covariance=target.covariance,
            altitude=self.verification_altitude,
            lane_spacing=self.verification_lane_spacing,
            minimum_radius=self.verification_minimum_radius,
            maximum_radius=self.verification_maximum_radius,
            covariance_sigma=self.verification_covariance_sigma,
            fixed_radius=(
                self.verification_fixed_radius
                if self.verification_radius_mode == "fixed"
                else None
            ),
        )

    @staticmethod
    def _point_inside_verification_plan(point, planned: PlannedArea) -> bool:
        xs = [value[0] for value in planned.area.boundary]
        ys = [value[1] for value in planned.area.boundary]
        return (
            min(xs) <= float(point[0]) <= max(xs)
            and min(ys) <= float(point[1]) <= max(ys)
        )

    def _containing_verification_region(self, point):
        candidates = [
            region_id
            for region_id, planned in self.verification_plans.items()
            if self._point_inside_verification_plan(point, planned)
        ]
        if not candidates:
            return None
        return min(
            candidates,
            key=lambda region_id: (
                1
                if self.verification_region_states.get(region_id)
                in ("completed", "missed")
                else 0,
                self.verification_plans[region_id].length,
                region_id,
            ),
        )

    def _attach_target_to_verification_region(self, target, region_id: int) -> None:
        target_id = int(target.target_id)
        owner = int(region_id)
        self.verification_region_by_target[target_id] = owner
        self.verification_targets_by_region.setdefault(owner, set()).add(target_id)

    def _publish_verification_areas(self) -> None:
        if not hasattr(self, "verification_areas_publisher"):
            return
        stamp = rospy.Time.now()
        areas = SearchAreaArray()
        areas.header.stamp = stamp
        areas.header.frame_id = self.shared_frame
        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        colors = {
            "pending": (1.0, 0.75, 0.0, 0.85),
            "active": (0.1, 0.45, 1.0, 0.95),
            "completed": (0.1, 0.9, 0.2, 0.75),
            "missed": (1.0, 0.15, 0.1, 0.85),
        }
        for region_id in sorted(self.verification_plans):
            planned = self.verification_plans[region_id]
            area = SearchArea()
            area.area_id = int(region_id)
            area.altitude = float(planned.area.altitude)
            area.lane_spacing = float(planned.area.lane_spacing)
            area.priority = int(planned.area.priority)
            area.boundary.points = [
                Point32(x=float(x), y=float(y), z=float(planned.area.altitude))
                for x, y in planned.area.boundary
            ]
            areas.areas.append(area)

            state = self.verification_region_states.get(region_id, "pending")
            red, green, blue, alpha = colors.get(state, colors["pending"])
            marker = Marker()
            marker.header = areas.header
            marker.ns = "verification_areas"
            marker.id = int(region_id)
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.8
            marker.color.r = red
            marker.color.g = green
            marker.color.b = blue
            marker.color.a = alpha
            marker.points = [
                Point(x=float(x), y=float(y), z=float(planned.area.altitude))
                for x, y in planned.area.boundary
            ]
            marker.points.append(marker.points[0])
            markers.markers.append(marker)
        self.verification_areas_publisher.publish(areas)
        self.verification_markers_publisher.publish(markers)

    def _queue_target_verification(self, target) -> None:
        target_id = int(target.target_id)
        if target_id in self.verification_region_by_target:
            return
        try:
            planned = self._build_verification_plan(target)
        except ValueError as error:
            rospy.logerr(
                "[task_allocate] cannot build target %d verification area: %s",
                target_id,
                error,
            )
            self.registry.set_status(target_id, TARGET_STALE)
            return
        existing_region = self._containing_verification_region(target.position)
        if existing_region is not None:
            self._attach_target_to_verification_region(target, existing_region)
            if self.verification_region_states.get(existing_region) in (
                "completed",
                "missed",
            ):
                self.registry.set_status(target_id, TARGET_STALE)
            else:
                self.registry.set_status(target_id, TARGET_VERIFYING)
            rospy.loginfo(
                "[task_allocate] fixed-wing target %d reuses verification region %d",
                target_id,
                existing_region,
            )
            self._publish_verification_areas()
            return
        region_id = target_id
        self.verification_plans[region_id] = planned
        self.verification_region_states[region_id] = "pending"
        self._attach_target_to_verification_region(target, region_id)
        self.registry.set_status(target_id, TARGET_VERIFYING)
        self.verification_pending.append(region_id)
        self.search_completed_at = 0.0
        rospy.loginfo(
            "[task_allocate] fixed-wing coarse target %d published verification "
            "region %d at (%.2f, %.2f)",
            target_id,
            region_id,
            target.position[0],
            target.position[1],
        )
        self._publish_verification_areas()
        self._assign_pending_verifications()

    def _assign_pending_verifications(self) -> None:
        if not self.hierarchical_search_enabled or self.mission_state != MISSION_ACTIVE:
            return
        if (
            self.verification_dispatch_policy == "after_coarse_complete"
            and not self._coarse_search_routes_complete()
        ):
            return
        remaining_slots = (
            self.verification_maximum_concurrent_regions
            - len(self.verification_target_by_scout)
        )
        if remaining_slots <= 0:
            return
        now = rospy.Time.now().to_sec()
        available = [
            name
            for name in self.scout_configs
            if self.vehicle_types[name] == "multirotor"
            and name not in self.verification_target_by_scout
            and self._vehicle_ready(name, now)
            and self.route_indices.get(name, 0) >= len(self.routes.get(name, []))
            and name not in self.active_goals
        ]
        while self.verification_pending and available and remaining_slots > 0:
            region_id = self.verification_pending.pop(0)
            planned = self.verification_plans.get(region_id)
            if planned is None:
                continue
            members = self.verification_targets_by_region.get(region_id, set())
            # Confirmation may dispatch a worker immediately, but it must not
            # consume or cancel the local search region: another target can be
            # present elsewhere inside the same box. Only an empty region
            # (all members explicitly rejected) is safe to skip.
            if not any(
                self.registry.targets.get(target_id) is not None
                for target_id in members
            ):
                self.verification_region_states[region_id] = "missed"
                self._publish_verification_areas()
                continue
            center_x = sum(value[0] for value in planned.area.boundary) / len(
                planned.area.boundary
            )
            center_y = sum(value[1] for value in planned.area.boundary) / len(
                planned.area.boundary
            )
            scout = min(
                available,
                key=lambda name: (
                    (self.vehicle_world_positions[name][1][0] - center_x) ** 2
                    + (self.vehicle_world_positions[name][1][1] - center_y) ** 2,
                    name,
                ),
            )
            self.verification_target_by_scout[scout] = region_id
            self.verification_scout_by_target[region_id] = scout
            self.verification_region_states[region_id] = "active"
            self.routes[scout] = planned.path
            self.route_indices[scout] = 0
            self._publish_path(scout, planned.path, rospy.Time.now())
            rospy.loginfo(
                "[task_allocate] verification region %d -> %s, radius=%.1f m, "
                "altitude=%.1f m, waypoints=%d",
                region_id,
                scout,
                0.5 * (
                    planned.area.boundary[1][0] - planned.area.boundary[0][0]
                ),
                planned.area.altitude,
                len(planned.path),
            )
            self._publish_next_scout_goal(scout)
            available.remove(scout)
            remaining_slots -= 1
            self._publish_verification_areas()
        if self.verification_pending and not available:
            rospy.logwarn_throttle(
                5.0,
                "[task_allocate] %d target verification request(s) waiting for "
                "an idle healthy multirotor scout",
                len(self.verification_pending),
            )

    def _finish_target_verification(self, region_id: int) -> None:
        region_id = int(region_id)
        self.verification_pending = [
            item for item in self.verification_pending if item != region_id
        ]
        scout = self.verification_scout_by_target.pop(region_id, None)
        if scout is not None:
            self.verification_target_by_scout.pop(scout, None)
            active = self.active_goals.get(scout)
            if active is not None and active[1] == "verification":
                self._clear_active_goal(scout)
            self.routes[scout] = []
            self.route_indices[scout] = 0
            self._publish_path(scout, [], rospy.Time.now())
        for member_id in self.verification_targets_by_region.get(region_id, set()):
            member = self.registry.targets.get(member_id)
            if member is None or member.status != TARGET_VERIFYING:
                continue
            self.registry.set_status(
                member_id,
                TARGET_STALE if self.reject_after_verification_route else TARGET_CANDIDATE,
            )
        members = self.verification_targets_by_region.get(region_id, set())
        found = any(
            self.registry.targets.get(member_id) is not None
            and self.registry.targets[member_id].status
            in (TARGET_CONFIRMED, TARGET_ASSIGNED, TARGET_EXECUTING, TARGET_COMPLETED)
            for member_id in members
        )
        self.verification_region_states[region_id] = (
            "completed" if found else "missed"
        )
        rospy.loginfo(
            "[task_allocate] multirotor completed the full route for verification "
            "region %d; members=%s",
            region_id,
            sorted(self.verification_targets_by_region.get(region_id, set())),
        )
        self._publish_verification_areas()
        self._assign_pending_verifications()
        self._mark_search_complete_if_ready()

    def _detection_callback(self, scout: str, message: DetectionArray) -> None:
        stamp = message.header.stamp.to_sec()
        if stamp <= 0.0:
            rospy.logwarn_throttle(2.0, "[task_allocate] rejected zero-stamped detections")
            return
        with self._lock:
            if self.mission_state != MISSION_ACTIVE:
                return
            pose = self.pose_histories[scout].closest(stamp, self.pose_sync_tolerance)
            if pose is None:
                rospy.logwarn_throttle(
                    2.0,
                    "[task_allocate] %s detection has no time-aligned vehicle pose",
                    scout,
                )
                return
            changed = False
            for candidate in message.candidates:
                if not (candidate.range_valid and candidate.has_relative_position_body):
                    continue
                try:
                    position = relative_frd_to_shared(
                        candidate.relative_position_body,
                        pose,
                    )
                except ValueError as error:
                    rospy.logwarn_throttle(2.0, "[task_allocate] geolocation rejected: %s", error)
                    continue
                observation = TargetObservation(
                    source_uav=scout,
                    stamp=stamp,
                    class_id=int(candidate.class_id),
                    position=position,
                    confidence=float(candidate.confidence),
                    covariance=rotate_covariance_frd_to_shared(
                        candidate.position_covariance,
                        pose.orientation_reference_body,
                    ),
                    source_vehicle_type=self.vehicle_types[scout],
                    track_id=int(candidate.track_id),
                    track_id_is_stable=bool(candidate.track_id_is_stable),
                    sensor_id=str(message.sensor_id),
                )
                update = self.registry.observe(observation)
                changed = True
                if (
                    self.hierarchical_search_enabled
                    and self.vehicle_types[scout] == "multirotor"
                ):
                    region_id = self._containing_verification_region(
                        update.target.position
                    )
                    if region_id is not None:
                        self._attach_target_to_verification_region(
                            update.target, region_id
                        )
                if update.newly_confirmed:
                    priority = self.class_priorities.get(update.target.class_id, 0)
                    allowed_types = self.class_worker_vehicle_types.get(
                        update.target.class_id,
                        self.default_worker_vehicle_types,
                    )
                    task = self.allocator.ensure_task(
                        update.target,
                        priority=priority,
                        allowed_vehicle_types=allowed_types,
                        duplicate_radius_m=self.task_duplicate_radius,
                    )
                    if task.target_id != update.target.target_id:
                        rospy.logwarn(
                            "[task_allocate] target %d confirmed near existing "
                            "same-class task %d/target %d; duplicate task suppressed",
                            update.target.target_id,
                            task.task_id,
                            task.target_id,
                        )
                    else:
                        rospy.loginfo(
                            "[task_allocate] target %d confirmed at "
                            "(%.2f, %.2f), class=%d",
                            update.target.target_id,
                            update.target.position[0],
                            update.target.position[1],
                            update.target.class_id,
                        )
                elif (
                    self.hierarchical_search_enabled
                    and self.vehicle_types[scout] == "fixedwing"
                    and update.newly_evidence_ready
                ):
                    self._queue_target_verification(update.target)
            if changed:
                self._dispatch_assignments()
                self._publish_state()

    def _search_area_definition(
        self, message, world_from_area: RigidTransform
    ) -> SearchAreaDefinition:
        transformed = [
            world_from_area.apply((float(point.x), float(point.y), float(message.altitude)))
            for point in message.boundary.points
        ]
        altitudes = [point[2] for point in transformed]
        if altitudes and max(altitudes) - min(altitudes) > self.search_area_altitude_tolerance:
            raise ValueError(
                "area frame is tilted relative to world; one constant search altitude is not valid"
            )
        boundary = tuple((point[0], point[1]) for point in transformed)
        return SearchAreaDefinition(
            area_id=int(message.area_id),
            boundary=boundary,
            altitude=sum(altitudes) / len(altitudes) if altitudes else float(message.altitude),
            lane_spacing=float(message.lane_spacing),
            priority=int(message.priority),
        )

    def _search_areas_callback(self, message: SearchAreaArray) -> None:
        success, detail, _ = self._load_search_areas(message)
        if not success:
            rospy.logwarn("[task_allocate] rejected search areas: %s", detail)

    def _load_search_areas(self, message: SearchAreaArray):
        with self._lock:
            if self.mission_state in (MISSION_ACTIVE, MISSION_PAUSED):
                return (
                    False,
                    f"cannot replace search areas while mission is {self.mission_state}",
                    0,
                )
        area_frame = self._canonical_frame(message.header.frame_id)
        if not area_frame:
            return False, "search-area frame_id is empty", 0
        try:
            world_from_area = self._world_transform(
                area_frame, message.header.stamp.to_sec()
            )
        except ValueError as error:
            return False, str(error), 0
        planned = []
        errors = []
        for area_message in message.areas:
            try:
                area = self._search_area_definition(area_message, world_from_area)
                planned.append(PlannedArea(area=area, path=lawnmower_path(area)))
            except ValueError as error:
                errors.append(f"area {int(area_message.area_id)}: {error}")
        if not planned:
            detail = "; ".join(errors) if errors else "message contains no areas"
            return False, f"no valid search area: {detail}", 0
        with self._lock:
            if self.mission_state in (MISSION_ACTIVE, MISSION_PAUSED):
                return (
                    False,
                    f"mission became {self.mission_state} while loading areas",
                    0,
                )
            self._clear_route_state()
            self._clear_results_data()
            self.loaded_areas = planned
            self.loaded_area_stamp = (
                message.header.stamp
                if message.header.stamp.to_sec() > 0.0
                else rospy.Time.now()
            )
            self.mission_state = MISSION_LOADED
            self.mission_detail = (
                f"loaded {len(planned)} search area(s); waiting for takeoff and start"
            )
            self._publish_mission_state()
            self._publish_state()
            rospy.loginfo("[task_allocate] %s", self.mission_detail)
            if errors:
                rospy.logwarn(
                    "[task_allocate] loaded valid areas but rejected: %s",
                    "; ".join(errors),
                )
            return True, self.mission_detail, len(planned)

    def _load_search_areas_service_callback(
        self, request
    ) -> LoadSearchAreasResponse:
        success, detail, count = self._load_search_areas(request.search_areas)
        return LoadSearchAreasResponse(
            success=success,
            message=detail,
            loaded_area_count=count,
        )

    def _publish_path(self, vehicle: str, route: List[Tuple[float, float, float]], stamp) -> None:
        path = Path()
        path.header.stamp = stamp if stamp.to_sec() > 0.0 else rospy.Time.now()
        path.header.frame_id = self.shared_frame
        for point in route:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = point
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self.path_publishers[vehicle].publish(path)

    def _new_goal_id(self) -> int:
        goal_id = self._next_goal_id
        self._next_goal_id += 1
        return goal_id

    def _clear_active_goal(self, vehicle: str) -> None:
        self.active_goals.pop(vehicle, None)
        self.active_goal_points.pop(vehicle, None)
        self.active_goal_origins.pop(vehicle, None)
        self.active_trajectory_end_times.pop(vehicle, None)
        self.active_trajectory_route_progress.pop(vehicle, None)
        self.active_controller_paths.discard(vehicle)
        self.arrival_tracker.reset(vehicle)
        self.worker_arrival_tracker.reset(vehicle)

    def _publish_direct_status(
        self, vehicle: str, goal_id: int, state: int, detail: str
    ) -> None:
        message = PlannerStatus()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = self.shared_frame
        message.goal_id = int(goal_id)
        message.state = int(state)
        message.detail = str(detail)
        self.direct_status_publishers[vehicle].publish(message)

    def _fixedwing_route_velocity(self, vehicle: str):
        """Return the outgoing XY tangent for an active fixed-wing route point."""

        active = self.active_goals.get(vehicle)
        if active is None or active[1] not in ("search", "verification"):
            return None
        route = self.routes.get(vehicle, [])
        index = int(active[2])
        if index < 0 or index >= len(route):
            return None
        point = route[index]
        if index + 1 < len(route):
            adjacent = route[index + 1]
            dx = float(adjacent[0]) - float(point[0])
            dy = float(adjacent[1]) - float(point[1])
        elif index > 0:
            adjacent = route[index - 1]
            dx = float(point[0]) - float(adjacent[0])
            dy = float(point[1]) - float(adjacent[1])
        else:
            return None
        distance = sqrt(dx * dx + dy * dy)
        if distance <= 1e-6:
            return None
        speed = self._coverage_speed(vehicle)
        return (speed * dx / distance, speed * dy / distance)

    def _publish_direct_controller_goal(self, vehicle: str, goal: PoseStamped) -> None:
        setpoint = PositionTarget()
        setpoint.header = goal.header
        setpoint.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        setpoint.type_mask = (
            PositionTarget.IGNORE_VX
            | PositionTarget.IGNORE_VY
            | PositionTarget.IGNORE_VZ
            | PositionTarget.IGNORE_AFX
            | PositionTarget.IGNORE_AFY
            | PositionTarget.IGNORE_AFZ
            | PositionTarget.IGNORE_YAW
            | PositionTarget.IGNORE_YAW_RATE
        )
        # PZ always remains enabled. Worker goals have already captured a safe
        # approach altitude; ignoring PZ would disable vertical closed-loop
        # control rather than hold the current altitude.
        setpoint.position.x = goal.pose.position.x
        setpoint.position.y = goal.pose.position.y
        setpoint.position.z = goal.pose.position.z
        # A fixed wing cannot converge to and hold a succession of static
        # points.  Add the route tangent so the controller guides through each
        # point along the planned Dubins path instead of orbiting the point.
        route_velocity = (
            self._fixedwing_route_velocity(vehicle)
            if self.vehicle_types[vehicle] == "fixedwing"
            else None
        )
        if route_velocity is not None:
            setpoint.type_mask &= ~(
                PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY
            )
            setpoint.velocity.x, setpoint.velocity.y = route_velocity
        position = self.vehicle_world_positions.get(vehicle)
        if (
            self.direct_face_goal
            and self.vehicle_types[vehicle] == "multirotor"
            and position is not None
        ):
            yaw = goal_heading(
                position[1],
                (goal.pose.position.x, goal.pose.position.y, goal.pose.position.z),
                self.direct_yaw_minimum_distance,
            )
            if yaw is not None:
                setpoint.type_mask &= ~PositionTarget.IGNORE_YAW
                setpoint.yaw = yaw
        self.direct_goal_publishers[vehicle].publish(setpoint)

    def _refresh_fixedwing_direct_goals(self) -> None:
        """Keep tangent-bearing fixed-wing PositionTargets alive."""

        for vehicle, active in list(self.active_goals.items()):
            if (
                self.vehicle_types.get(vehicle) != "fixedwing"
                or not self._uses_direct_controller(vehicle)
                or active[1] not in ("search", "verification")
                or vehicle in self.active_controller_paths
            ):
                continue
            point = self.active_goal_points.get(vehicle)
            if point is None:
                continue
            goal = PoseStamped()
            goal.header.seq = int(active[0])
            goal.header.stamp = rospy.Time.now()
            goal.header.frame_id = self.shared_frame
            goal.pose.position.x = float(point[0])
            goal.pose.position.y = float(point[1])
            goal.pose.position.z = float(point[2])
            goal.pose.orientation.w = 1.0
            self._publish_direct_controller_goal(vehicle, goal)

    def _publish_fixedwing_route_path(
        self, vehicle: str, route, start_index: int, kind: str
    ) -> int:
        current = self.vehicle_world_positions.get(vehicle)
        remaining = list(route[int(start_index) :])
        if current is None or not remaining:
            raise ValueError("fixed-wing path needs current position and route")
        current_point = tuple(float(value) for value in current[1])
        first = remaining[0]
        separation = sqrt(
            (float(first[0]) - current_point[0]) ** 2
            + (float(first[1]) - current_point[1]) ** 2
            + (float(first[2]) - current_point[2]) ** 2
        )
        goal_id = self._new_goal_id()
        message = Path()
        message.header.seq = goal_id
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = self.shared_frame
        path_points = ([current_point] if separation > 1e-3 else []) + remaining
        for point in path_points:
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x = float(point[0])
            pose.pose.position.y = float(point[1])
            pose.pose.position.z = float(point[2])
            pose.pose.orientation.w = 1.0
            message.poses.append(pose)

        final_index = len(route) - 1
        self.active_goals[vehicle] = (goal_id, str(kind), final_index)
        self.active_goal_points[vehicle] = tuple(float(value) for value in route[-1])
        self.active_goal_origins[vehicle] = current_point
        self.active_controller_paths.add(vehicle)
        self.direct_path_publishers[vehicle].publish(message)
        self._publish_direct_status(
            vehicle,
            goal_id,
            PlannerStatus.ACTIVE,
            "complete fixed-wing geometric path active; no obstacle avoidance",
        )
        self._handle_goal_status(
            vehicle,
            goal_id,
            PlannerStatus.ACTIVE,
            "complete fixed-wing geometric path active; no obstacle avoidance",
        )
        rospy.loginfo(
            "[task_allocate] fixed-wing %s published complete path: "
            "route_points=%d nominal_speed=%.1fm/s",
            vehicle,
            len(remaining),
            self._coverage_speed(vehicle),
        )
        return goal_id

    def _publish_goal(self, vehicle: str, point, kind: str, object_id: int) -> int:
        goal_id = self._new_goal_id()
        goal = PoseStamped()
        goal.header.seq = goal_id
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = self.shared_frame
        goal.pose.position.x = float(point[0])
        goal.pose.position.y = float(point[1])
        goal.pose.position.z = float(point[2])
        goal.pose.orientation.w = 1.0
        self.active_goals[vehicle] = (goal_id, kind, int(object_id))
        self.active_goal_points[vehicle] = (
            float(point[0]),
            float(point[1]),
            float(point[2]),
        )
        current = self.vehicle_world_positions.get(vehicle)
        self.active_goal_origins[vehicle] = (
            current[1] if current is not None else self.active_goal_points[vehicle]
        )
        self.arrival_tracker.reset(vehicle)
        self.worker_arrival_tracker.reset(vehicle)
        if self._uses_direct_controller(vehicle):
            self._publish_direct_controller_goal(vehicle, goal)
            self._publish_direct_status(
                vehicle,
                goal_id,
                PlannerStatus.ACTIVE,
                "direct controller test goal active; no obstacle avoidance",
            )
            self._handle_goal_status(
                vehicle,
                goal_id,
                PlannerStatus.ACTIVE,
                "direct controller test goal active; no obstacle avoidance",
            )
        else:
            self.goal_publishers[vehicle].publish(goal)
        return goal_id

    def _publish_next_scout_goal(self, scout: str) -> None:
        if self.mission_state != MISSION_ACTIVE:
            return
        route = self.routes.get(scout, [])
        index = self.route_indices.get(scout, 0)
        if index >= len(route):
            self._clear_active_goal(scout)
            target_id = self.verification_target_by_scout.get(scout)
            if target_id is not None:
                self._finish_target_verification(target_id)
                return
            rospy.loginfo("[task_allocate] scout %s completed its coarse search route", scout)
            if (
                self.hierarchical_search_enabled
                and self.vehicle_types.get(scout) == "fixedwing"
            ):
                self._assign_pending_verifications()
            self._mark_search_complete_if_ready()
            return
        kind = "verification" if scout in self.verification_target_by_scout else "search"
        if (
            self._uses_direct_controller(scout)
            and self.vehicle_types.get(scout) == "fixedwing"
        ):
            try:
                self._publish_fixedwing_route_path(
                    scout, route, index, kind
                )
            except ValueError as error:
                rospy.logerr(
                    "[task_allocate] cannot publish fixed-wing path for %s: %s",
                    scout,
                    error,
                )
            return
        self._publish_goal(scout, route[index], kind, index)

    def _dispatch_assignments(self) -> None:
        if self.mission_state != MISSION_ACTIVE:
            return
        for task, worker in self.allocator.assign_pending():
            self.registry.set_status(task.target_id, TARGET_ASSIGNED)
            task.goal = list(
                worker_approach_goal(
                    worker.position,
                    task.target_position,
                    self.worker_horizontal_standoff,
                )
            )
            goal_id = self._publish_goal(worker.name, task.goal, "rescue", task.task_id)
            backend = self.vehicle_backends[worker.name]
            task.detail = f"{backend} goal {goal_id} published"
            rospy.loginfo(
                "[task_allocate] task %d target %d -> %s, backend=%s goal=%d "
                "approach=(%.2f, %.2f, %.2f)",
                task.task_id,
                task.target_id,
                worker.name,
                backend,
                goal_id,
                task.goal[0],
                task.goal[1],
                task.goal[2],
            )

    def _check_mission_completed(self, now: float) -> None:
        if self.mission_state != MISSION_ACTIVE or self.search_completed_at <= 0.0:
            return
        if self.verification_pending or self.verification_target_by_scout:
            return
        if now - self.search_completed_at < self.completion_grace_sec:
            return
        if any(task.status != TASK_COMPLETED for task in self.allocator.tasks.values()):
            return
        self.mission_state = MISSION_COMPLETED
        self.mission_detail = "all search routes and rescue tasks completed"
        self._publish_mission_state()
        rospy.loginfo("[task_allocate] %s", self.mission_detail)

    def _planner_status_callback(self, vehicle: str, message: PlannerStatus) -> None:
        with self._lock:
            self._handle_goal_status(
                vehicle,
                int(message.goal_id),
                int(message.state),
                str(message.detail),
            )

    def _path_status_callback(self, vehicle: str, message: PathStatus) -> None:
        """Translate controller path progress into the existing planner contract."""
        with self._lock:
            if vehicle not in self.active_controller_paths:
                return
            active = self.active_goals.get(vehicle)
            if active is None or int(message.path_id) != int(active[0]):
                return
            state = int(message.state)
            if state in (
                PathStatus.ACCEPTED,
                PathStatus.ACTIVE,
                PathStatus.REACQUIRING,
            ):
                planner_state = PlannerStatus.ACTIVE
            elif state == PathStatus.COMPLETED:
                planner_state = PlannerStatus.REACHED
            else:
                planner_state = PlannerStatus.FAILED
            detail = (
                f"controller path {float(message.progress) * 100.0:.1f}%: "
                f"{message.detail}"
            )
            self._publish_direct_status(
                vehicle, int(message.path_id), planner_state, detail
            )
            self._handle_goal_status(
                vehicle, int(message.path_id), planner_state, detail
            )

    def _handle_goal_status(
        self, vehicle: str, goal_id: int, state: int, detail: str
    ) -> None:
        active = self.active_goals.get(vehicle)
        if active is None or int(goal_id) != active[0]:
            return
        _, kind, object_id = active
        if kind in ("search", "verification"):
            if state == PlannerStatus.REACHED:
                self.route_indices[vehicle] = object_id + 1
                self._clear_active_goal(vehicle)
                self._publish_next_scout_goal(vehicle)
                self._check_mission_completed(rospy.Time.now().to_sec())
            elif state in (PlannerStatus.BLOCKED, PlannerStatus.FAILED):
                rospy.logerr(
                    "[task_allocate] scout %s route waypoint %d blocked: %s",
                    vehicle,
                    object_id,
                    detail,
                )
            return

        task = self.allocator.tasks.get(object_id)
        if task is None:
            return
        if state in (PlannerStatus.PLANNING, PlannerStatus.ACTIVE):
            if self.allocator.mark_executing(task.task_id):
                self.registry.set_status(task.target_id, TARGET_EXECUTING)
        elif state == PlannerStatus.REACHED:
            if self._uses_direct_controller(vehicle):
                # Freeze the worker at its measured completion pose instead of
                # leaving a stale approach command active while task state is
                # released or another task is allocated.
                self._publish_direct_hold(vehicle)
            completed = self.allocator.complete(task.task_id)
            if completed is not None:
                self.registry.set_status(completed.target_id, TARGET_COMPLETED)
            self._clear_active_goal(vehicle)
            self._dispatch_assignments()
            self._check_mission_completed(rospy.Time.now().to_sec())
        elif state in (PlannerStatus.BLOCKED, PlannerStatus.FAILED):
            self.allocator.release_worker(
                vehicle,
                f"execution backend failed: {detail}",
            )
            # Do not immediately feed the same failed goal back to the same
            # backend. A fresh valid ControlState re-enables this worker.
            worker = self.allocator.workers.get(vehicle)
            if worker is not None:
                worker.online = False
            self.worker_retry_after[vehicle] = (
                rospy.Time.now().to_sec() + self.planner_failure_retry_sec
            )
            self._clear_active_goal(vehicle)
            self.registry.set_status(task.target_id, TARGET_CONFIRMED)
            self._dispatch_assignments()
        self._publish_state()

    def _check_direct_goal_arrivals(self, now: float) -> None:
        if not self.direct_controller_test or self.mission_state != MISSION_ACTIVE:
            return
        reached = []
        for vehicle, active in list(self.active_goals.items()):
            if not self._uses_direct_controller(vehicle):
                continue
            # Fixed-wing path completion is reported by the controller from
            # actual along-track progress and endpoint geometry. Do not race
            # that feedback with the waypoint/dwell fallback below.
            if vehicle in getattr(self, "active_controller_paths", set()):
                continue
            progress = self.active_trajectory_route_progress.get(vehicle, [])
            for deadline, route_index in progress:
                if now >= deadline:
                    self.route_indices[vehicle] = max(
                        self.route_indices.get(vehicle, 0), route_index
                    )
            position = self.vehicle_world_positions.get(vehicle)
            goal = self.active_goal_points.get(vehicle)
            if (
                position is None
                or goal is None
                or not self._vehicle_ready(vehicle, now)
            ):
                self.arrival_tracker.reset(vehicle)
                self.worker_arrival_tracker.reset(vehicle)
                continue
            goal_id, kind, _ = active
            trajectory_end = self.active_trajectory_end_times.get(vehicle)
            if trajectory_end is not None:
                arrived = now >= trajectory_end
                if arrived and self.vehicle_types[vehicle] == "fixedwing":
                    route = self.routes.get(vehicle, [])
                    final_origin = (
                        route[-2]
                        if len(route) >= 2
                        else self.active_goal_origins.get(vehicle, position[1])
                    )
                    # The controller may slow its trajectory clock while it
                    # recaptures a turn. Nominal duration is therefore only a
                    # lower bound; require the final fly-through as well.
                    arrived = fixedwing_waypoint_reached(
                        position[1],
                        final_origin,
                        goal,
                        acceptance_radius_m=self._fixedwing_setting(
                            vehicle, "waypoint_acceptance_radius_m", 20.0
                        ),
                        altitude_tolerance_m=self._fixedwing_setting(
                            vehicle, "waypoint_altitude_tolerance_m", 10.0
                        ),
                        pass_cross_track_limit_m=self._fixedwing_setting(
                            vehicle, "pass_cross_track_limit_m", 40.0
                        ),
                    )
            elif self.vehicle_types[vehicle] == "fixedwing":
                origin = self.active_goal_origins.get(vehicle, position[1])
                arrived = fixedwing_waypoint_reached(
                    position[1],
                    origin,
                    goal,
                    acceptance_radius_m=self._fixedwing_setting(
                        vehicle, "waypoint_acceptance_radius_m", 20.0
                    ),
                    altitude_tolerance_m=self._fixedwing_setting(
                        vehicle, "waypoint_altitude_tolerance_m", 10.0
                    ),
                    pass_cross_track_limit_m=self._fixedwing_setting(
                        vehicle, "pass_cross_track_limit_m", 40.0
                    ),
                )
            else:
                tracker = (
                    self.worker_arrival_tracker
                    if kind == "rescue"
                    else self.arrival_tracker
                )
                speed = self.vehicle_world_speeds.get(vehicle)
                if (
                    kind == "rescue"
                    and (
                        speed is None
                        or now - speed[0] > self.odometry_timeout
                        or speed[1] > self.worker_maximum_arrival_speed
                    )
                ):
                    tracker.reset(vehicle)
                    arrived = False
                else:
                    arrived = tracker.update(
                        vehicle, position[1], goal, now, use_z=True
                    )
            if arrived:
                reached.append((vehicle, goal_id))
        for vehicle, goal_id in reached:
            detail = (
                "fixed-wing trajectory duration and final fly-through satisfied"
                if vehicle in self.active_trajectory_end_times
                else "fixed-wing waypoint acceptance or bounded fly-through satisfied"
                if self.vehicle_types[vehicle] == "fixedwing"
                else "world odometry remained inside direct-goal tolerance"
            )
            self._publish_direct_status(
                vehicle, goal_id, PlannerStatus.REACHED, detail
            )
            self._handle_goal_status(
                vehicle, goal_id, PlannerStatus.REACHED, detail
            )

    def _timer_callback(self, _event) -> None:
        with self._lock:
            now = rospy.Time.now().to_sec()
            if self.mission_state != MISSION_ACTIVE:
                # PAUSED deliberately freezes target expiry, route progress,
                # verification dispatch and worker task assignment. Other
                # inactive states likewise retain their final snapshot.
                self._publish_state()
                return
            self.registry.expire(now)
            released = self.allocator.mark_offline_workers(now, self.worker_state_timeout)
            for task in released:
                for vehicle, active in list(self.active_goals.items()):
                    if active[1] == "rescue" and active[2] == task.task_id:
                        self._clear_active_goal(vehicle)
                self.registry.set_status(task.target_id, TARGET_CONFIRMED)
            self._assign_pending_verifications()
            self._dispatch_assignments()
            self._refresh_fixedwing_direct_goals()
            self._check_direct_goal_arrivals(now)
            self._check_mission_completed(now)
            self._publish_state()

    def _publish_state(self) -> None:
        stamp = rospy.Time.now()
        targets = GlobalTargetArray()
        targets.header.stamp = stamp
        targets.header.frame_id = self.shared_frame
        for record in sorted(self.registry.targets.values(), key=lambda item: item.target_id):
            message = GlobalTarget()
            message.header = targets.header
            message.target_id = record.target_id
            message.class_id = record.class_id
            message.position.x, message.position.y, message.position.z = record.position
            message.position_covariance = record.covariance
            message.confidence = record.confidence
            message.observation_count = record.observation_count
            message.observer_uavs = sorted(record.observer_uavs)
            message.status = record.status
            targets.targets.append(message)
        self.targets_publisher.publish(targets)

        tasks = RescueTaskArray()
        tasks.header = targets.header
        for record in sorted(self.allocator.tasks.values(), key=lambda item: item.task_id):
            message = RescueTask()
            message.header = tasks.header
            message.task_id = record.task_id
            message.target_id = record.target_id
            message.class_id = record.class_id
            message.goal.x, message.goal.y, message.goal.z = record.goal
            message.priority = record.priority
            message.allowed_vehicle_types = list(record.allowed_vehicle_types)
            message.assigned_worker = record.assigned_worker
            message.status = record.status
            message.detail = record.detail
            tasks.tasks.append(message)
        self.tasks_publisher.publish(tasks)


def main() -> None:
    rospy.init_node("task_allocate_coordinator")
    TaskAllocateCoordinator()
    rospy.spin()
