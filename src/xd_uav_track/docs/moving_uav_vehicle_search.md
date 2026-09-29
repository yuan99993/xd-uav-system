# Moving-UAV vehicle search scene

This is a separate, low-load Gazebo Classic scenario. It does not modify the
static `tracking_benchmark` baseline.

## What it contains

- One kinematic quadrotor model with a rigid, downward-looking 640x480 camera.
- Three camouflaged vehicle models on parallel racetracks: straight sections
  have constant linear velocity and the ends use tangent 2 m-radius turns.
  Racetrack centerlines are 16 m apart, keeping at least 8 m between vehicle
  hulls at their closest approach.
- A four-lane lawnmower search route with rounded U-turns at 28 m initial
  altitude by default (`uav_altitude_m` is configurable in the demo launch).
  Search/reacquisition cruise is limited to 1.4 m/s and transit to the global
  observation point to 1.6 m/s by default; at 4 Hz this advances about 0.35 m
  per detector update.
- One shared train7 YOLO detector, the normal `xd_uav_detect` ground projector,
  and `xd_uav_track` with confirmed-only overlay display.
- Global multi-target search is the default: no best-quality track is silently
  selected for flight-follow, while all confirmed target boxes keep their
  identities on the overlay. From startup, the aircraft flies toward the
  center of the configured search-area rectangle, aligns the camera's wide
  image axis with the area's long axis, and climbs to the altitude needed to
  cover the entire rectangle within configured altitude limits. Tracker
  measurements report how many targets have been observed, but do not define
  coverage geometry; Gazebo target truth is never used for planning. With the
  rigid downward camera, the area center gives the widest coverage. A map-edge
  viewpoint would require camera tilt or a gimbal to look back across the area.
- An explicit target command switches to single-target `FOLLOW`; only that
  selected ID drives pursuit. Returning to `all` clears flight selection and
  resumes global coverage.
- During follow, horizontal speed and acceleration are capped at 2.6 m/s and
  1.2 m/s². The quadrotor holds yaw so the fixed nadir image does not rotate
  with every vehicle turn. Predicted camera FOV margin determines when it can
  descend from the wide-area observation altitude toward 28 m.
- If the selected target is lost, the UAV coasts for `follow_loss_coast_sec`
  (5 s by default), then climbs toward a 40 m local observation height for
  `local_reacquire_sec` (12 s). After that it returns smoothly to the area
  center and wide-area altitude. A rounded lawnmower sweep remains the fallback
  when a single view cannot cover the configured area. These transitions have
  bounded velocity and acceleration; they do not place the UAV directly onto
  a search route. The selected global ID is retained and only a confirmed
  identity match can resume `FOLLOW`.

The demo defaults to three expected targets, a 28 m minimum cruise altitude,
and a 60 m maximum altitude. When one target is selected, the UAV keeps a
58 m overview height and only biases a short distance toward it while the
configured search rectangle remains inside the fixed camera FOV. This avoids
losing the other vehicles just because one vehicle is selected. The same
overview constraint applies during coast and local reacquisition. Set
`follow_preserve_other_targets:=false` to use the close single-target chase
instead. The overview height, edge margin, and bias fraction are configurable
with `follow_overview_altitude_m`, `follow_overview_margin_m`, and
`follow_overview_fraction`. These can be changed with
`expected_target_count`, `minimum_altitude_m`, and `maximum_altitude_m` launch
arguments. The coast interval and search speeds are configurable with
`follow_loss_coast_sec`, `search_cruise_speed_mps`, and
`coverage_transit_speed_mps`.
The search rectangle is configurable with
`search_area_min_x_m`, `search_area_max_x_m`, `search_area_min_y_m`, and
`search_area_max_y_m`. Set `expected_target_count:=0` when the count is
unknown; the UAV then uses the lawnmower route instead of the fixed
global-coverage waypoint. Follow speed, horizontal acceleration, yaw rate,
camera FOV margin, and local reacquisition height/time are exposed as launch
arguments.

Select one visible global track for flight-follow:

```bash
rosservice call /moving_uav_search/track/set_target_mode \
  "{mode: 'follow', target_id: 2, image_source: 'search_fixed', capture_timestamp: {secs: 0, nsecs: 0}}"
```

Return to global multi-target search without selecting a vehicle:

```bash
rosservice call /moving_uav_search/track/set_target_mode \
  "{mode: 'all', target_id: -1, image_source: '', capture_timestamp: {secs: 0, nsecs: 0}}"
```

The Gazebo driver publishes commanded-path ground truth separately under
`/moving_uav_search/ground_truth/*`; it is for evaluation/display only. This is
not a PX4 flight-dynamics or flight-safety test: the vehicle's pose is driven
by a bounded kinematic model. Search/reacquisition cruise defaults to 1.4 m/s,
global-view transit is limited to 1.6 m/s, and selected-target pursuit is
capped at 2.6 m/s with 1.2 m/s² horizontal acceleration.

## Start

From the `mrs_test` ROS workspace after sourcing its setup file:

```bash
roslaunch xd_uav_track moving_uav_vehicle_search_demo.launch
```

This opens Gazebo and an image window showing confirmed tracker boxes. The
detector runs at 4 Hz while the simulated camera runs at 10 Hz. `gui:=false`
can be passed to run without the Gazebo window; `show_overlay:=false` suppresses
the image window. Set `enable_reid:=false` to reduce optional appearance
inference load.

Useful topics:

- `/moving_uav_search/fixed_camera/image_raw`
- `/moving_uav_search/fixed_camera/tracking_overlay`
- `/moving_uav_search/track/tracks_by_source`
- `/moving_uav_search/scenario/phase` (`GLOBAL_COVERAGE`, `FOLLOW`,
  `FOLLOW_COAST`, `FOLLOW_LOCAL_REACQUIRE`, or `FOLLOW_REACQUIRE_SEARCH`)
- `/moving_uav_search/ground_truth/state_json` (evaluation only)

The camera-to-body optical transform is fixed in
`moving_uav_search_driver.py`; world-to-body is broadcast at each kinematic
state update. Ground projection rejects missing/late capture-time transforms
instead of substituting the latest transform. Before replacing the simulated
camera with a real airframe, recalibrate that transform and camera intrinsics.
