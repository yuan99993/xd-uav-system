cmake_minimum_required(VERSION 3.5)
project(xd_uav_sead)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  nav_msgs
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  tf
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
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
  tools/xbee_send_formation_test.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_rosbridge_protocol.py)
endif()
