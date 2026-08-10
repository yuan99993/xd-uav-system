# Follower profiles

> Production safety note: selecting a profile does not authorize aircraft
> motion. With the product defaults, acquire/renew `~manage_control_lease` and
> then call `~start`. See `IMPLEMENTATION.md` for backend and lease rules.

The C++ follower exposes the eight profile names used by PixEagle while keeping
the ROS/XD command path fail-closed.

Select a profile at startup with the private `follower_profile` parameter or at
runtime with:

```bash
rosservice call /follower_node/set_profile "mode: 'mc_velocity_position'"
```

## Velocity-body profiles

| Profile | Behavior | Tracker requirement |
|---|---|---|
| `mc_velocity_ground` | Independent forward/lateral PID ground guidance, mount-aware gimbal compensation, altitude-scaled error and guarded descent | angular error preferred; normalized image error fallback |
| `mc_velocity_distance` | Zero forward speed, lateral/vertical centering and yaw | normalized image error |
| `mc_velocity_position` | Zero forward/right speed, yaw and optional vertical centering | normalized image error |
| `mc_velocity_chase` | Forward ramp plus coordinated-turn or sideslip pursuit | normalized image error |
| `gm_velocity_chase` | PID pursuit driven by tracker line-of-sight angles | angular error required |
| `gm_velocity_vector` | Direct body velocity vector from line-of-sight angles | angular error required |

These profiles are accepted by `xd_controller_reference_adapter_node` and are
converted from forward/right/down body commands into XD odometry references.

## Attitude-rate profiles

| Profile | Output |
|---|---|
| `mc_attitude_rate` | altitude-held compensated thrust, yaw-gated pitch tracking, coordinated roll and target-loss hover |
| `fw_attitude_rate` | airspeed-adaptive L1 lateral guidance, TECS energy control, roll/load feedback, stall recovery and target-loss orbit |

Both are capability-gated. `mc_attitude_rate` requires
`allow_attitude_rate_profiles:=true`; `fw_attitude_rate` separately requires
`allow_fixed_wing_profile:=true`. The XD position-reference adapter always
rejects `attitude_rate` commands. A dedicated, vehicle-specific PX4 attitude
adapter and SITL acceptance are required before flight use.
The XD launch files also pass `vehicle_type` into the follower. When supplied,
the follower rejects a fixed-wing attitude profile on a multirotor (and the
reverse) before publishing it; the adapter performs the same check again.

The multicopter attitude profile captures the current altitude as its hold
setpoint (`mc_target_altitude_offset` defaults to zero), compensates thrust for
measured roll/pitch, and derives coordinated bank from measured ground speed.
The fixed-wing profile reads MAVROS `/mavros/vfr_hud` airspeed/groundspeed and
`/mavros/imu/data` roll feedback. Missing/stale airspeed falls back to fresh
ground speed and then cruise airspeed; stall recovery is only triggered by a
fresh airspeed measurement. After `target_loss_timeout`, the fixed-wing loss
strategy changes from straight/level cruise to a load-limited orbit.

## Safety and switching

Every profile shares target freshness/confidence checks, target-loss timeout,
altitude limits, emergency stop, controller-override limits, velocity/accel/jerk
feasibility, and PID anti-windup. Switching profiles resets PID and kinematic
history so a zero-forward profile cannot inherit motion from a chase profile.

The default remains `mc_velocity_chase`, so existing launch files and XD
controller communication retain their previous behavior.

### Profile cooperation and latency

Velocity-profile changes use a configurable smoothstep handover
(`profile_transition_duration_sec`, default 0.5 s). The old feasible command
is blended into the new profile before the common acceleration/jerk limiter,
so switching profiles does not create a first-frame velocity or yaw step.
Production configuration also enforces a minimum profile dwell time
(`profile_switch_min_interval_sec`, default 0.5 s); a service request inside
that window is rejected with a diagnostic reason instead of causing repeated
PID/history resets. Test configurations may set the dwell time to zero.
Attitude-rate profiles are never blended with each other because multicopter
and fixed-wing body-rate commands belong to different vehicle envelopes.

When the tracker still supplies valid normalized error but temporarily omits
`yaw_error_rad/pitch_error_rad`, the gimbal profiles cooperate with their
normalized-error equivalents instead of declaring an immediate target loss:

| Requested profile | Temporary effective profile |
|---|---|
| `gm_velocity_chase` | `mc_velocity_chase` |
| `gm_velocity_vector` | `mc_velocity_distance` |

The requested profile remains visible in `follower_profile`; diagnostics expose
the actual source through `effective_profile` and `profile_fallback_active`.
Once angular data returns, the requested GM algorithm resumes automatically.
Confidence acquisition/release hysteresis prevents command chatter around the
minimum confidence threshold.

### Metric relative-state control and uncertainty

`tracker/DetectionCandidate` may provide body-frame metric state
(`relative_position_body`, `relative_velocity_body`, and 3x3 covariance). The
tracker forwards this to `NormalizedError`/`TrackingOutput` with `range_valid`.
For `mc_velocity_chase` and `gm_velocity_chase`, the follower then uses:

```text
v = target_relative_velocity_feedforward
    - Kp * (desired_relative_position - measured_relative_position)
```

The bbox-size distance proxy remains the automatic fallback when `range_valid`
is false. Covariance is converted to a control scale: direct low-uncertainty
observations retain full authority, medium uncertainty is velocity/yaw limited,
ReID recovery ramps in over `reid_recovery_duration_sec`, and high uncertainty
fails closed with zero command.

The control path uses latest-only subscriber queues with TCP no-delay and runs
the follower/XD velocity adapter at 60 Hz by default. `FollowerStatus` reports
the measured loop rate, publication age, capture age, tracker-reported
inference latency, transport/queue latency, core compute time, and estimated
capture-to-command latency. When capture age exceeds
`tracker_latency_warning_sec` (default 120 ms), translational and body-rate
authority is reduced linearly; at `tracker_capture_timeout_sec` /
`tracker_latency_abort_sec` (default 250 ms), target control is rejected and
the profile's fail-safe output is used. Legacy tracker messages without a
`capture_timestamp` retain the publication-age behavior. Visualization
messages are only assembled when a visualization subscriber exists.

## PX4 attitude-rate SITL

Use the vehicle-specific launchers so the profile, controller type, tuning and
Gazebo assets cannot be mixed accidentally:

```bash
# Iris multicopter
roslaunch follower tracker_follower_xd_multirotor_attitude_sitl.launch

# PX4 plane; also selects the local aerodynamic and airspeed plugins
roslaunch follower tracker_follower_xd_fixedwing_attitude_sitl.launch
```

The command ownership path is intentionally single-writer:

```text
/follower_node/follower_command
  -> /follower_px4_attitude_rate_adapter
  -> /uav1/controller/selected_command
  -> /uav1/control_manager
  -> /uav1/mavros/setpoint_raw/attitude
```

`control_manager` is the only publisher to the MAVROS attitude setpoint.  The
adapter starts disabled, always gives takeoff/landing commands priority, checks
vehicle/profile/rate/thrust/freshness, and falls back to the XD controller.
After takeoff has completed and the follower is publishing a fresh valid frame,
enable the handover explicitly:

```bash
rosservice call /follower_node/manage_control_lease \
  "acquire: true
requester: 'operator_ui'
output_backend: 'command_only'
lease_duration_sec: 5.0"
rosservice call /follower_node/start "data: true"
rosservice call /follower_px4_attitude_rate_adapter/enable_follower "data: true"
```

Disable handover before stopping the follower or requesting landing:

```bash
rosservice call /follower_px4_attitude_rate_adapter/enable_follower "data: false"
rosservice call /follower_node/stop "data: true"
rosservice call /uav1/control_manager/land "{}"
```

The SITL launcher disables PX4 RC-loss interference for unattended testing and
selects Hold for OFFBOARD-loss behavior.  Those parameter changes are applied
only to the temporary SITL startup script; they do not change real-aircraft
parameters.
