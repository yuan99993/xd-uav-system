# xd-uavsystem-test agent entrypoint

Before analyzing or changing this repository, read and follow
`/home/promise/catkin_ws/AGENTS.md`. It contains the workspace directory map,
shared-repository modification boundaries, ROS/Python environment rules, and
simulation safety requirements.

Treat this repository's `CLAUDE.md` and `temp/` as historical task material,
not as authoritative current status. Re-verify any claims taken from them.

## Current-task navigation and repository boundaries

- `.codex-tmp/ACTIVE_TASK.md` is the only authoritative current task list,
  status, authorization gate, and next-action entrypoint. Read it before any
  task-specific design or implementation document.
- For the current EGO integration work, the normal editable implementation
  scope is limited to `src/xd_uav_ego_bridge/` and
  `src/xd_uav_system_integration/`, plus explicitly authorized task documents
  and `.codex-tmp/` evidence. Preserve unrelated working-tree changes.
- `src/ego-planner-swarm/` is an external nested Git workspace and is
  read-only unless the user separately authorizes a specific change. Never
  stage, commit, clean, or revert it from the parent repository.
- Existing `src/xd_uav_controller/`, `src/xd_uav_control_manager/`, estimator,
  MRS, PX4, Gazebo, QGC, and other maintained packages are read-only for this
  task unless `ACTIVE_TASK.md` records a new explicit authorization.
- SEAD integration is currently paused. Files under `src/xd_uav_sead/docs/`
  are historical/background material unless `ACTIVE_TASK.md` routes to one;
  do not infer the current work plan from their older E-stage sequences.
- The default current technical references are only
  `src/xd_uav_ego_bridge/docs/EGO_SWARM_TECHNICAL_GUIDE.md` and
  `src/xd_uav_system_integration/docs/COMPONENT_INTERFACE_CONTRACT.md`.

## Upstream-first and implementation quality

- For the current integration line, only modify `src/xd_uav_ego_bridge/`,
  `src/xd_uav_system_integration/`, and explicitly authorized task documents.
  Treat controller, manager, estimator, TF, FAST-LIO/Livox, EGO, MRS, PX4,
  Gazebo, tmux, and other maintained packages as read-only unless the user
  explicitly expands scope.
- Before concluding that another maintained package still has a bug, first
  check its source author's current remote target branch/version and compare
  the relevant code, configuration, and tests. Report remaining upstream
  defects and the smallest options to the user; do not silently patch the
  source package locally.
- Do not use frame-label relabeling, unverified identity TFs, absolute
  user-specific paths, copied private constants, or trial-and-error magic
  numbers as production fixes.
- Required constants must be named in a profile, include units, and have a
  traceable source such as an interface contract, vehicle/sensor calibration,
  simulation world, requirement, or repeatable measurement. Missing evidence
  fails closed.
- Experimental adapters must document assumptions, health/diagnostics,
  tests, and removal criteria. Do not propagate the current G4 demo fixtures
  into Fast-LIO, multi-vehicle, or real-flight paths.
