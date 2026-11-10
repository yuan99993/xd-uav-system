cmake_minimum_required(VERSION 3.0.2)
project(target_geolocation)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  sar_yolo_detector
  sensor_msgs
  std_msgs
  xd_uav_track
)

# 必须调用：CATKIN_PACKAGE_BIN_DESTINATION / SHARE_DESTINATION 由它定义。
# 缺了它 catkin_install_python() 会报 "called without required DESTINATION argument"。
catkin_python_setup()

add_message_files(FILES GeolocatedTarget.msg)

generate_messages(DEPENDENCIES sensor_msgs std_msgs)

catkin_package(CATKIN_DEPENDS
  message_runtime
  sensor_msgs
  sar_yolo_detector
  std_msgs
  xd_uav_track
)

catkin_install_python(PROGRAMS
  scripts/target_geolocation_node.py
  scripts/sim_target_sensor.py
  scripts/check_accuracy.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY test
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  OPTIONAL
)
