#!/bin/bash

WS=~/xd-uavsystem-test
export DISPLAY=:0
PID_FILE=/tmp/UE_roslaunch.pids

# 每次启动前清空旧记录
rm -f "$PID_FILE"
touch "$PID_FILE"

gnome-terminal --title="roscore" -- bash -c "
source /opt/ros/noetic/setup.bash
source $WS/devel/setup.bash

roslaunch xd_uav_state_estimators uav_localization_stack.launch UAV_NAME:=uav1 tf_config:=/home/nvidia/xd-uavsystem-test/src/xd_uav_single_tf_manager/config/multirotor_single_tf.yaml &
PID=\$!
echo \$PID >> "$PID_FILE"
wait \$PID

exec bash
"

sleep 2

gnome-terminal --title="mavros" -- bash -c "
source /opt/ros/noetic/setup.bash
source $WS/devel/setup.bash

roslaunch xd_uav_control_manager multirotor_system.launch UAV_NAME:=uav1 &
PID=\$!
echo \$PID >> "$PID_FILE"
wait \$PID

exec bash
"

sleep 2

gnome-terminal --title="controller" -- bash -c "
source /opt/ros/noetic/setup.bash
source $WS/devel/setup.bash

roslaunch xd_uav_world_tf_manager world_tf.launch &
PID=\$!
echo \$PID >> "$PID_FILE"
wait \$PID

exec bash
"

sleep 1

gnome-terminal --title="tf_manager" -- bash -c "
source .venv-sar/bin/activate
source /opt/ros/noetic/setup.bash
source $WS/devel/setup.bash

roslaunch sar_yolo_detector xd_smart_tracker_integration.launch UAV_NAME:=uav1 input_image_topic:=/Sim/SceneBasicDrone/robots/Drone1/sensors/DownCamera/scene_camera/image camera_info_topic:=/Sim/SceneBasicDrone/robots/Drone1/sensors/Chase/scene_camera/camera_info output_detections_topic:=detect/input/detections_2d image_source:=fixed_rgb sensor_id:=fixed_camera use_gpu:=false &
PID=\$!
echo \$PID >> "$PID_FILE"
wait \$PID

exec bash
"

sleep 1

gnome-terminal --title="setpoint" -- bash -c "
source /opt/ros/noetic/setup.bash
source $WS/devel/setup.bash

roslaunch xd_uav_task_allocate task_allocate.launch mission_config:=$(rospack find xd_uav_task_allocate)/config/mission.yaml scout_config:=$(rospack find xd_uav_task_allocate)/config/scouts_two_multi.yaml worker_config:=$(rospack find xd_uav_task_allocate)/config/workers_two_multi.yaml post_arrival_config:=$(rospack find xd_uav_task_allocate)/config/task_execution_four_multi.yaml &
PID=\$!
echo \$PID >> "$PID_FILE"
wait \$PID

exec bash
"



sleep 1

gnome-terminal --title="setpoint" -- bash -c "
source /opt/ros/noetic/setup.bash
source $WS/devel/setup.bash

roslaunch xd_uav_detect detect.launch UAV_NAME:=uav1 config:=/home/nvidia/xd-uavsystem-test/src/xd_uav_detect/config/camera_ground_plane.yaml &
PID=\$!
echo \$PID >> "$PID_FILE"
wait \$PID

exec bash
"

sleep 1

gnome-terminal --title="setpoint" -- bash -c "
source /opt/ros/noetic/setup.bash
source $WS/devel/setup.bash

roslaunch xd_uav_track track.launch UAV_NAME:=uav1 &
PID=\$!
echo \$PID >> "$PID_FILE"
wait \$PID

exec bash
"