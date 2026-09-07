# gm_control

`gm_control` is a ROS package for image-based gimbal control. Its first goal is
simple: keep a selected/tracked target near the center of the camera image.

The package is intentionally split into two layers:

1. `gimbal_image_controller_node.py`: image target box -> normalized image error
   -> yaw/pitch rate command.
2. gimbal adapter: command -> real backend such as Gazebo, MAVLink, serial, UDP,
   or a vendor SDK.

Only the first layer is implemented as a generic controller. The adapter boundary
is left open so different gimbals can be connected later without rewriting the
image-control logic.

## Connect to xd_uav_track

`xd_track_gimbal_bridge_node.py` connects the two packages without coupling the
generic tracker to a gimbal backend:

```text
/uavX/track/tracks (selected TrackState)
  -> /uavX/gm_control/target_bbox
  -> /uavX/gm_control/gimbal_cmd

/uavX/gm_control/gimbal_state (degrees)
  -> /uavX/track/gimbal_state (radians)
```

Start it next to an already running `xd_uav_track` node:

```bash
UAV_NAME=uav1 roslaunch gm_control xd_track_gimbal_control.launch \
  image_topic:=/uav1/camera/image_raw
```

The launch file intentionally does not start `bbox_tracker_node.py`: the selected
and filtered track from `xd_uav_track` is the single source of the gimbal target.
It only uses the custom `/uav1/gm_control/gimbal_cmd` and
`/uav1/gm_control/gimbal_state` boundary; the user-provided gimbal driver owns
those topics and no MAVROS mount-control interface is added here.

The bridge invalidates the bbox when the track stream times out. Direction signs
for driver-specific gimbal conventions are in `config/xd_track_bridge.yaml`.

## Topics

Input:

- `/camera/image_raw` (`sensor_msgs/Image`)
- `/gm_control/initial_bbox` (`gm_control/BoundingBox2D`)
- `/gm_control/target_bbox` (`gm_control/BoundingBox2D`)
- `/gm_control/gimbal_state` (`gm_control/GimbalState`, published by the
  user-provided gimbal driver)

Output:

- `/gm_control/gimbal_cmd` (`gm_control/GimbalCommand`, subscribed by the
  user-provided gimbal driver)
- `/gm_control/image_error` (`geometry_msgs/Vector3Stamped`)
- `/gm_control/debug_image` (`sensor_msgs/Image`)

`image_error.vector.x` is horizontal normalized error, `image_error.vector.y` is
vertical normalized error, and `image_error.vector.z` is `1` when the command is
valid.

## Build

```bash
cd /home/kzy/gm_control_ws
catkin_make
source devel/setup.bash
```

## Run

有外部检测时直接启动控制跟踪

```bash
roslaunch gm_control gm_control.launch image_topic:=/typhoon_h480/cgo3_camera/image_raw
```

Run the bbox tracker and gimbal image controller together,外部没有输入框时用这个产生一个跟踪框的效果:

```bash
roslaunch gm_control gm_tracking_control.launch image_topic:=/typhoon_h480/cgo3_camera/image_raw
```

Publish a quick initial target box for the tracker:

```bash
rostopic pub /gm_control/initial_bbox gm_control/BoundingBox2D "{valid: true, x: 200.0, y: 100.0, width: 40.0, height: 40.0, confidence: 1.0, target_id: test}" -1
```

Watch the command:

```bash
rostopic echo /gm_control/gimbal_cmd
```

View the debug image:

```bash
rqt_image_view /gm_control/debug_image
```

## Control Meaning

The controller computes:

```text
error_x = (target_center_x - image_width / 2) / (image_width / 2)
error_y = (target_center_y - image_height / 2) / (image_height / 2)
```

Then it generates:

```text
yaw_cmd   = yaw_sign   * (yaw_pid.p   * error_x + yaw_pid.i   * integral_x + yaw_pid.d   * d(error_x)/dt)
pitch_cmd = pitch_sign * (pitch_pid.p * error_y + pitch_pid.i * integral_y + pitch_pid.d * d(error_y)/dt)
```

If the gimbal turns the wrong way, flip `yaw_sign` or `pitch_sign` in
`config/gm_control.yaml`.

## Main Configuration

`control_mode` selects the command output style:

- `rate`: publish `yaw_rate_deg_s` and `pitch_rate_deg_s`.
- `angle`: publish `yaw_deg` and `pitch_deg` as image-error-based angle offsets.

`target_lost` controls what happens when the bbox disappears:

- `stop`: output an invalid command.
- `hold_last`: keep the previous command for `hold_time_s`.
- `search`: output a constant search yaw/pitch rate.
- `back_to_init`: output an angle command to return the gimbal to the configured
  initial yaw/pitch/roll.

`smoothing` filters command jumps caused by bbox noise.

`pid` contains the yaw and pitch controller gains. Keep `i: 0.0` at first unless
you specifically need to remove a steady-state offset.

The PID gains can also be tuned at runtime with `rqt_reconfigure`:

```bash
rosrun rqt_reconfigure rqt_reconfigure
```

Select `/gimbal_image_controller` and adjust `yaw_kp`, `yaw_ki`, `yaw_kd`,
`pitch_kp`, `pitch_ki`, `pitch_kd`, `yaw_integral_limit`, or
`pitch_integral_limit`.

`tracker_safety` rejects suspicious tracker output, such as a bbox that jumps too
far, changes size too much, or touches the image edge. When this happens the
tracker publishes an invalid bbox so the gimbal stops instead of chasing a
drifted target.

The bbox tracker has two modes:

- `tracker.tracker_type: color_tracker`: detect the configured color every frame
  and publish `/gm_control/target_bbox`. This does not need
  `/gm_control/initial_bbox`.
- `tracker.tracker_type: feature_tracker`: use OpenCV KCF/CSRT/MIL after one
  `/gm_control/initial_bbox` initialization.

For the Gazebo red target, start with:

```yaml
tracker:
  tracker_type: color_tracker
  color_tracker:
    color: red
```
