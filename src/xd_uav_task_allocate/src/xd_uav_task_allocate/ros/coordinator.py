"""Central ROS coordinator for scout coverage and worker rescue tasks."""

import threading
from typing import Dict, List, Tuple

import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import PositionTarget
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse

from xd_uav_controller.msg import ControlState
from xd_uav_task_allocate.core.allocation import (
    TASK_ASSIGNED,
    TASK_COMPLETED,
    TASK_EXECUTING,
    RescueTaskAllocator,
)
from xd_uav_task_allocate.core.coverage import (
    PlannedArea,
    SearchAreaDefinition,
    assign_areas,
    lawnmower_path,
)
from xd_uav_task_allocate.core.execution import (
    ArrivalDwellTracker,
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
    TARGET_COMPLETED,
    TARGET_CONFIRMED,
    TARGET_EXECUTING,
    GlobalTargetRegistry,
    TargetObservation,
)
from xd_uav_task_allocate.msg import (
    GlobalTarget,
    GlobalTargetArray,
    PlannerStatus,
    RescueTask,
    RescueTaskArray,
    SearchAreaArray,
)
from xd_uav_track.msg import DetectionArray


MISSION_IDLE = "IDLE"
MISSION_LOADED = "LOADED"
MISSION_ACTIVE = "ACTIVE"
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
        self.direct_controller_test = self.planner_backend == "direct_controller_test"
        if self.direct_controller_test and not bool(
            rospy.get_param(
                "~planner/direct_controller_test/acknowledge_no_obstacle_avoidance",
                False,
            )
        ):
            raise ValueError(
                "direct_controller_test requires explicit "
                "acknowledge_no_obstacle_avoidance=true"
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
        self.worker_state_timeout = float(
            rospy.get_param("~allocation/worker_state_timeout_sec", 1.0)
        )
        self.worker_horizontal_standoff = max(
            0.0,
            float(
                rospy.get_param(
                    "~allocation/worker_approach/horizontal_standoff_m", 2.0
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
        )
        self.allocator = RescueTaskAllocator()

        tf_cache_sec = max(
            10.0,
            float(rospy.get_param("~geolocation/pose_history_sec", 5.0)) + 1.0,
        )
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(tf_cache_sec))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.scout_configs = dict(rospy.get_param("~scouts", {}))
        self.worker_configs = dict(rospy.get_param("~workers", {}))
        if not self.scout_configs:
            rospy.logwarn("[task_allocate] no scouts configured")
        if not self.worker_configs:
            rospy.logwarn("[task_allocate] no workers configured")

        self.pose_histories: Dict[str, PoseHistory] = {}
        self.vehicle_world_positions: Dict[str, Tuple[float, Tuple[float, float, float]]] = {}
        self.vehicle_health: Dict[str, Tuple[float, bool]] = {}
        self.goal_publishers: Dict[str, rospy.Publisher] = {}
        self.direct_goal_publishers: Dict[str, rospy.Publisher] = {}
        self.direct_status_publishers: Dict[str, rospy.Publisher] = {}
        self.path_publishers: Dict[str, rospy.Publisher] = {}
        self.routes: Dict[str, List[Tuple[float, float, float]]] = {}
        self.route_indices: Dict[str, int] = {}
        self.active_goals: Dict[str, Tuple[int, str, int]] = {}
        self.active_goal_points: Dict[str, Tuple[float, float, float]] = {}
        self.worker_retry_after: Dict[str, float] = {}
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
        self.start_service = rospy.Service(
            str(
                rospy.get_param(
                    "~interfaces/services/start",
                    "/task_allocate/start",
                )
            ),
            Trigger,
            self._start_callback,
        )

        for name, config in self.scout_configs.items():
            self._configure_vehicle(str(name), dict(config), is_scout=True)
        for name, config in self.worker_configs.items():
            self.allocator.register_worker(str(name))
            self._configure_vehicle(str(name), dict(config), is_scout=False)

        update_rate = max(1.0, float(rospy.get_param("~runtime/update_rate_hz", 5.0)))
        self.timer = rospy.Timer(rospy.Duration(1.0 / update_rate), self._timer_callback)
        self._publish_state()
        self._publish_mission_state()
        rospy.loginfo(
            "[task_allocate] ready: scouts=%s workers=%s shared_frame=%s; "
            "position=main world Odometry; backend=%s",
            sorted(self.scout_configs),
            sorted(self.worker_configs),
            self.shared_frame,
            self.planner_backend,
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
        if self.direct_controller_test:
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
        if not self.direct_controller_test:
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
        execution_connected = bool(
            not self.direct_controller_test
            or (
                name in self.direct_goal_publishers
                and self.direct_goal_publishers[name].get_num_connections() > 0
            )
        )
        return bool(
            position is not None
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
            if is_worker:
                self._refresh_worker(name, receive_time)

    def _health_callback(self, name: str, is_worker: bool, message: ControlState) -> None:
        stamp = message.header.stamp.to_sec()
        if stamp <= 0.0:
            rospy.logwarn_throttle(
                2.0, "[task_allocate] %s control state has zero timestamp", name
            )
            return
        healthy = bool(
            (message.state_valid or not self.require_state_valid)
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
        assignments = assign_areas(self.loaded_areas, scout_positions)
        self.routes.clear()
        self.route_indices.clear()
        for scout, areas in assignments.items():
            route = [point for area in areas for point in area.path]
            self.routes[scout] = route
            self.route_indices[scout] = 0
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

            self._prepare_search_routes(positions)
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
                )
                update = self.registry.observe(observation)
                changed = True
                if update.newly_confirmed:
                    priority = self.class_priorities.get(update.target.class_id, 0)
                    self.allocator.ensure_task(update.target, priority=priority)
                    rospy.loginfo(
                        "[task_allocate] target %d confirmed at (%.2f, %.2f), class=%d",
                        update.target.target_id,
                        update.target.position[0],
                        update.target.position[1],
                        update.target.class_id,
                    )
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
        with self._lock:
            if self.mission_state == MISSION_ACTIVE:
                rospy.logwarn(
                    "[task_allocate] rejected new search areas while mission is ACTIVE"
                )
                return
        area_frame = self._canonical_frame(message.header.frame_id)
        if not area_frame:
            rospy.logwarn("[task_allocate] rejected search areas with empty frame_id")
            return
        try:
            world_from_area = self._world_transform(
                area_frame, message.header.stamp.to_sec()
            )
        except ValueError as error:
            rospy.logwarn("[task_allocate] rejected search areas: %s", error)
            return
        planned = []
        for area_message in message.areas:
            try:
                area = self._search_area_definition(area_message, world_from_area)
                planned.append(PlannedArea(area=area, path=lawnmower_path(area)))
            except ValueError as error:
                rospy.logwarn(
                    "[task_allocate] rejected search area %d: %s",
                    area_message.area_id,
                    error,
                )
        if not planned:
            rospy.logwarn("[task_allocate] search-area message contains no valid area")
            return
        with self._lock:
            if self.mission_state == MISSION_ACTIVE:
                rospy.logwarn(
                    "[task_allocate] mission became ACTIVE while loading search areas"
                )
                return
            self.loaded_areas = planned
            self.loaded_area_stamp = (
                message.header.stamp
                if message.header.stamp.to_sec() > 0.0
                else rospy.Time.now()
            )
            self.routes.clear()
            self.route_indices.clear()
            self.active_goals.clear()
            self.active_goal_points.clear()
            self.arrival_tracker.reset_all()
            self.search_completed_at = 0.0
            self.mission_state = MISSION_LOADED
            self.mission_detail = (
                f"loaded {len(planned)} search area(s); waiting for takeoff and start"
            )
            self._publish_mission_state()
            rospy.loginfo("[task_allocate] %s", self.mission_detail)

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
        self.arrival_tracker.reset(vehicle)

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
        position = self.vehicle_world_positions.get(vehicle)
        if self.direct_face_goal and position is not None:
            yaw = goal_heading(
                position[1],
                (goal.pose.position.x, goal.pose.position.y, goal.pose.position.z),
                self.direct_yaw_minimum_distance,
            )
            if yaw is not None:
                setpoint.type_mask &= ~PositionTarget.IGNORE_YAW
                setpoint.yaw = yaw
        self.direct_goal_publishers[vehicle].publish(setpoint)

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
        self.arrival_tracker.reset(vehicle)
        if self.direct_controller_test:
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
            rospy.loginfo("[task_allocate] scout %s completed its search route", scout)
            if self.routes and all(
                self.route_indices.get(name, 0) >= len(vehicle_route)
                for name, vehicle_route in self.routes.items()
            ):
                if self.search_completed_at <= 0.0:
                    self.search_completed_at = rospy.Time.now().to_sec()
                    rospy.loginfo(
                        "[task_allocate] all scout routes complete; waiting %.1fs for final detections",
                        self.completion_grace_sec,
                    )
            return
        self._publish_goal(scout, route[index], "search", index)

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
            task.detail = f"{self.planner_backend} goal {goal_id} published"
            rospy.loginfo(
                "[task_allocate] task %d target %d -> %s, backend=%s goal=%d "
                "approach=(%.2f, %.2f, %.2f)",
                task.task_id,
                task.target_id,
                worker.name,
                self.planner_backend,
                goal_id,
                task.goal[0],
                task.goal[1],
                task.goal[2],
            )

    def _check_mission_completed(self, now: float) -> None:
        if self.mission_state != MISSION_ACTIVE or self.search_completed_at <= 0.0:
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

    def _handle_goal_status(
        self, vehicle: str, goal_id: int, state: int, detail: str
    ) -> None:
        active = self.active_goals.get(vehicle)
        if active is None or int(goal_id) != active[0]:
            return
        _, kind, object_id = active
        if kind == "search":
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
            position = self.vehicle_world_positions.get(vehicle)
            goal = self.active_goal_points.get(vehicle)
            if (
                position is None
                or goal is None
                or not self._vehicle_ready(vehicle, now)
            ):
                self.arrival_tracker.reset(vehicle)
                continue
            goal_id, _, _ = active
            if self.arrival_tracker.update(
                vehicle, position[1], goal, now, use_z=True
            ):
                reached.append((vehicle, goal_id))
        for vehicle, goal_id in reached:
            detail = "world odometry remained inside direct-goal tolerance"
            self._publish_direct_status(
                vehicle, goal_id, PlannerStatus.REACHED, detail
            )
            self._handle_goal_status(
                vehicle, goal_id, PlannerStatus.REACHED, detail
            )

    def _timer_callback(self, _event) -> None:
        with self._lock:
            now = rospy.Time.now().to_sec()
            self.registry.expire(now)
            released = self.allocator.mark_offline_workers(now, self.worker_state_timeout)
            for task in released:
                for vehicle, active in list(self.active_goals.items()):
                    if active[1] == "rescue" and active[2] == task.task_id:
                        self._clear_active_goal(vehicle)
                self.registry.set_status(task.target_id, TARGET_CONFIRMED)
            self._dispatch_assignments()
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
            message.assigned_worker = record.assigned_worker
            message.status = record.status
            message.detail = record.detail
            tasks.tasks.append(message)
        self.tasks_publisher.publish(tasks)


def main() -> None:
    rospy.init_node("task_allocate_coordinator")
    TaskAllocateCoordinator()
    rospy.spin()
