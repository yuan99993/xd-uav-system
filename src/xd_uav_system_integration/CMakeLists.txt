cmake_minimum_required(VERSION 3.5)
project(xd_uav_system_integration)

find_package(catkin REQUIRED COMPONENTS
  diagnostic_msgs
  geometry_msgs
  gazebo_msgs
  geographic_msgs
  mavros_msgs
  nav_msgs
  quadrotor_msgs
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  tf2_geometry_msgs
  tf2_ros
  xd_uav_controller
  xd_uav_state_estimators
)

catkin_python_setup()

catkin_package(CATKIN_DEPENDS
  diagnostic_msgs
  geometry_msgs
  gazebo_msgs
  geographic_msgs
  mavros_msgs
  nav_msgs
  quadrotor_msgs
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  tf2_geometry_msgs
  tf2_ros
  xd_uav_controller
  xd_uav_state_estimators
)

catkin_install_python(PROGRAMS
  scripts/reference_mux_node.py
  scripts/health_gate_node.py
  scripts/reference_health_node.py
  scripts/profile_loader.py
  scripts/state_hold_baseline_node.py
  scripts/spawn_px4_vehicle_node.py
  scripts/select_owner_when_ready.py
  scripts/mavros_odom_recovery_gate.py
  scripts/auto_takeoff_when_ready.py
  scripts/deterministic_obstacle_cloud.py
  scripts/imu_frame_adapter.py
  scripts/rigid_body_imu_adapter.py
  scripts/ego_command_frame_adapter.py
  scripts/fastlio_shadow_gate.py
  scripts/gazebo_fastlio_world_alignment.py
  scripts/navsat_origin_provider.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS scripts/ego_obstacle_demo.sh scripts/ego_demo.sh
  scripts/multi_odom_registration.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_nosetests(test/test_integration_core.py)
  catkin_add_nosetests(test/test_profile.py)
  catkin_add_nosetests(test/test_portable_configuration.py)
  add_rostest(test/integration_nodes.test)
  add_rostest(test/mavros_odom_recovery_gate.test)
  add_rostest(test/fastlio_shadow_gate.test)
endif()
