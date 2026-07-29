cmake_minimum_required(VERSION 3.0.2...3.28)
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
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Python 库模块 (devel 空间通过软链接，install 空间通过此处)
# Devel 软链接: devel/lib/python3/dist-packages/xd_uav_sead → src/xd_uav_sead/src/xd_uav_sead
install(DIRECTORY src/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# 在 devel 空间创建软链接 (catkin_make 每次 configure 时执行)
execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink
  ${CMAKE_CURRENT_SOURCE_DIR}/src/${PROJECT_NAME}
  ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}
  ERROR_QUIET
)
