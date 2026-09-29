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
- For the current MRS-decoupling work, editable implementation scope is
  limited to `src/xd_uav_planning/`, `src/xd_uav_detect/`,
  `src/xd_uav_controller/`, `src/xd_uav_control_manager/`, their public
  documentation, and `.codex-tmp/` evidence. Preserve unrelated changes.
- `src/ego-planner-swarm/` is an external nested Git workspace and is
  read-only unless the user separately authorizes a specific change. Never
  stage, commit, clean, or revert it from the parent repository.
- Estimator, task allocation, MRS, PX4, Gazebo, QGC, and other maintained
  packages are read-only unless `ACTIVE_TASK.md` records explicit authority.
- Top-level `src/xd_uav_system_integration/` and `src/xd_uav_sead/` were
  intentionally removed. Do not recreate them or use old generated artifacts
  as current architecture references. The isolated
  `src/xd_uav_task_allocate/legacy/` tree belongs to task allocation and is
  outside this task.

## Upstream-first and implementation quality

- For the current integration line, only modify the packages named by the
  current-task scope above and explicitly authorized task documents. Treat
  estimator, task allocation, TF, FAST-LIO/Livox, EGO, MRS, PX4, Gazebo,
  tmux, and other maintained packages as read-only unless the user explicitly
  expands scope.
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
