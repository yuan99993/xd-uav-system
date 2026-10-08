"""Fail-closed validation for releasing screened terrain paths to controllers."""


class GateNotReady(ValueError):
    """The current staged batch is not safe and complete enough to release."""


def _frame(value):
    return str(value).lstrip("/")


def path_goal_id(path):
    if path is None or len(path.poses) < 2:
        raise GateNotReady("staged_path_missing_or_too_short")
    goal_id = int(path.poses[0].header.seq)
    if goal_id <= 0:
        raise GateNotReady("staged_path_goal_missing")
    return goal_id


def releasable_paths(status, staged_paths, vehicle_names, common_frame="world"):
    """Return ``[(name, goal_id, path)]`` only for a coherent safe batch."""
    if status is None:
        raise GateNotReady("swarm_status_missing")
    if not status.analysis_ready:
        raise GateNotReady("swarm_analysis_not_ready")
    if not status.conflict_free:
        raise GateNotReady("swarm_conflict_detected")
    # TerrainConflict.SPATIAL == 1. Keep this pure module independent of
    # generated ROS messages. A monitor may explicitly mark spatial-only
    # records as warnings by publishing conflict_free=True; unknown and
    # spatiotemporal records still fail closed here.
    if any(int(getattr(conflict, "conflict_type", 0)) != 1
           for conflict in status.conflicts):
        raise GateNotReady("swarm_blocking_conflict_detected")
    if _frame(status.header.frame_id) != _frame(common_frame):
        raise GateNotReady("swarm_status_frame_mismatch")

    summaries = {item.uav_name: item for item in status.vehicles}
    output = []
    for name in vehicle_names:
        summary = summaries.get(name)
        if summary is None:
            raise GateNotReady("{}:summary_missing".format(name))
        if not summary.path_available:
            raise GateNotReady("{}:terrain_path_missing".format(name))
        path = staged_paths.get(name)
        goal_id = path_goal_id(path)
        if _frame(path.header.frame_id) != _frame(common_frame):
            raise GateNotReady("{}:staged_path_frame_mismatch".format(name))
        for pose in path.poses:
            if (_frame(pose.header.frame_id) != _frame(common_frame)
                    or int(pose.header.seq) != goal_id):
                raise GateNotReady("{}:staged_path_contract_mismatch".format(name))
        if int(summary.planner_status.goal_id) != goal_id:
            raise GateNotReady("{}:goal_id_mismatch".format(name))
        output.append((name, goal_id, path))
    return output
