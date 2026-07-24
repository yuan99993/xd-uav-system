#include <ros/ros.h>

#include <xd_uav_state_estimator/state_estimator_node.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "state_estimator");
  ros::NodeHandle node_handle;
  ros::NodeHandle private_node_handle("~");

  xd_uav_state_estimator::StateEstimatorNode estimator(node_handle, private_node_handle);
  ros::spin();
  return 0;
}
