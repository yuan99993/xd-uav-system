cmake_minimum_required(VERSION 3.5)
project(xd_uav_sead)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  mavros_msgs
  nav_msgs
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  tf
)

catkin_python_setup()

add_message_files(
  FILES
  NoFlyZone.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    message_runtime
    mavros_msgs
    nav_msgs
    rospy
    sensor_msgs
    std_msgs
    std_srvs
    tf
)

# Python 可执行脚本
catkin_install_python(PROGRAMS
  scripts/sead_onboard_node.py
  scripts/mock_gcs.py
  scripts/fixedwing_nofly_acceptance.py
  scripts/prepare_plane_sdf.py
  scripts/sead_validation_visualizer.py
  tools/xbee_send_formation_test.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_nosetests(test/test_core_functions.py)
  catkin_add_nosetests(test/test_dynamic_nofly.py)
  catkin_add_nosetests(test/test_rosbridge_protocol.py)
  add_rostest(test/dynamic_nofly_ros.test)
endif()
