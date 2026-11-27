# Terrain navigation planner in xd-uav-system

This directory contains the ROS 1 terrain planner integrated into the
`xd-uav-system` catkin workspace. The original checkout at
`/home/dell/terrain-navigation` remains intact.

## Contents

- `planner_msgs`: planner status and route messages.
- `terrain_navigation`: terrain map, route, and path data structures.
- `terrain_planner`: OMPL/RRT* and Dubins fixed-wing planning core.
- `terrain_navigation_ros`: ROS planner node, MAVROS state inputs, topic-based
  planner commands, and global `nav_msgs/Path` output.
- `third_party`: source dependencies required by these packages (`grid_map`,
  `grid_map_geo`, `eigen_catkin`, and `catkin_simple`).
- `resources`: elevation/color rasters and Gazebo terrain files.
- `scripts/build_gazebo_terrain.py`: Gazebo terrain asset generator.

To change the ROS behavior and global path output, start with
`terrain_navigation_ros/src/terrain_planner.cpp` and
`terrain_navigation_ros/include/terrain_navigation_ros/terrain_planner.h`.
The search algorithm is under `terrain_planner/src/`; terrain sampling is
under `terrain_navigation/src/`.

## Build

Use your source-built MAVROS overlay, then build from the main workspace:

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
cd ~/xd-uav-system
catkin_make --pkg planner_msgs terrain_navigation terrain_planner terrain_navigation_ros grid_map_core grid_map_msgs grid_map_cv grid_map_ros grid_map_pcl grid_map_geo -j2
source devel/setup.bash
```

Other ROS packages and system libraries declared in the package manifests
must be available, including OMPL, GDAL, GeographicLib, and PCL. MAVROS comes
from the existing source-built workspace overlay.

## Planner-only launch

The planner launch starts one instance under the requested UAV namespace. It
relies on the shared world TF manager and that UAV's single-TF manager; it does
not start PX4, Gazebo, MAVROS, or create a competing world-to-map transform.

The planner keeps projected terrain calculations in a per-UAV
uavX/terrain_map frame, registers that frame in the existing world TF tree, and
converts the global route to the shared world frame. The controller can
transform the same path through world -> uavX/local_origin -> uavX/odom.

Start one planner per aircraft, with each output remapped to that aircraft's
controller path input:

    roslaunch terrain_navigation_ros planner_only.launch uav_name:=uav1 location:=davosdorf path_output_topic:=/uav1/control/reference/path
    roslaunch terrain_navigation_ros planner_only.launch uav_name:=uav2 location:=davosdorf path_output_topic:=/uav2/control/reference/path

Before launching, start the shared xd_uav_world_tf_manager and each aircraft's
MAVROS and xd_uav_single_tf_manager stack. Verify the TF chain with:

    rosrun tf tf_echo world uav1/odom

## Planner inputs

The planner accepts standard ROS topics under each UAV namespace. For example,
the goal is a `geometry_msgs/PointStamped` and its `header.frame_id` defines the
coordinate frame. The goal position is converted into the planner's DEM frame.
If `frame_id` is empty, the planner uses `planning_input_frame` (default:
`world`). Point coordinates are in meters; replace the example with the desired
world-frame goal.

```bash
rostopic pub --once /uav1/terrain_planner/goal geometry_msgs/PointStamped \
  '{header: {frame_id: world}, point: {x: 100.0, y: 100.0, z: 100.0}}'
rostopic pub --once /uav1/terrain_planner/plan std_msgs/Empty '{}'
```

`plan` uses the latest valid state from
`/uav1/state_estimator/main/odom` as the start. The estimator pose in
`uav1/odom` is transformed through TF into `uav1/local_origin`, reversed
through the `gp_origin` local Cartesian reference into WGS84, projected to
LV03, then offset by the DEM map origin. `/uav1/state_estimator/state_valid`
must be true.
There is no start-position input. The horizontal start remains the aircraft's
position; the planner checks its altitude against the safe terrain corridor.
The estimator topics, pose freshness timeout, geodetic-origin topic, turn
radius, terrain clearance, map altitude margin, startup-climb limits, planning
budget, and altitude-check setting are configured in
`terrain_navigation_ros/config/planner.yaml`. For
example, change
`estimator_pose_timeout: 5.0` to `estimator_pose_timeout: 3.0` to allow the
latest estimator message to be up to three seconds old. The current configured
value is five seconds. A different YAML file
can be selected with the launch argument `planner_config:=/path/to/planner.yaml`.

The goal's z value is ignored. At the goal, the planner chooses an altitude
above the DEM-derived minimum-clearance surface (`minimum_terrain_clearance`,
50 m by default). The global altitude ceiling is the highest valid elevation
in the loaded DEM plus `map_altitude_margin` (120 m by default). This keeps
the usable altitude range tied to each map instead of creating a separate
clearance band at every point.

The terminal orbit radius and direction are read from the running controller's
`/uav1/controller/fixedwing/modes/loiter/radius` and
`/uav1/controller/fixedwing/modes/loiter/direction` parameters (with the UAV
namespace substituted). The goal orbit must fit inside the DEM, and its
terrain-clearance altitude is computed along that orbit. `minimum_turn_radius`
in `planner.yaml` remains the minimum curvature radius for route turns and the
startup climbing turn; it is not a duplicate setting for the controller's
post-path waiting orbit.

The route starts at the estimator's current position and velocity. If the
aircraft is below the safe altitude needed to enter the route, the planner
first creates a gradual fixed-wing climbing turn and validates it against the
DEM. The transition uses `cruise_speed`, `maximum_climb_rate`, and
`maximum_climb_angle`. `startup_climb_minimum_clearance` (10 m by default)
checks the transition against the terrain directly below it; the main route
still uses the configured 50 m terrain-clearance surface. If the aircraft is
already below this startup clearance, the planner rejects the request rather
than claiming a safe climb.

After a route is available, publish `std_msgs/Empty` to
`/uav1/terrain_planner/execute_path` to activate the planner's internal route
following state. The planned route is also published as a latched
`nav_msgs/Path` with `header.frame_id` set to `world`, using the configured
`path_output_topic` so the controller connection remains unchanged.

Other inputs are:

- `/uav1/terrain_planner/planner_state` (`std_msgs/UInt8`): `1` hold, `2`
  navigate, `3` rollout, `4` abort, `5` return.
- `/uav1/terrain_planner/check_max_altitude` (`std_msgs/Bool`): toggle maximum
  altitude collision checks. Its initial value comes from the
  `check_max_altitude` setting in `planner.yaml`.

Select the terrain dataset with the launch-time `location` argument. It is
map configuration, not a runtime planner command.
