#include <clocale>
#include <memory>
#include <stdexcept>
#include <string>

#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <xmlrpcpp/XmlRpcValue.h>

#include <xd_uav_state_estimators/odometry_adapter.hpp>

class OdometryAdapterManager {
 public:
  OdometryAdapterManager() : private_nh_("~") {
    private_nh_.param("uav_name", uav_name_, std::string("uav1"));

    XmlRpc::XmlRpcValue adapter_configs;
    if (!private_nh_.getParam("odometry_adapters", adapter_configs)) {
      throw std::runtime_error(
          "未找到odometry_adapters，请检查sources.yaml");
    }
    adapters_ = std::make_unique<
        xd_uav_state_estimators::OdometryAdapterCollection>(
        nh_, &tf_buffer_, uav_name_, adapter_configs);
    if (adapters_->requiresTransform()) {
      tf_listener_ = std::make_unique<tf2_ros::TransformListener>(tf_buffer_);
    }
  }

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  tf2_ros::Buffer tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<xd_uav_state_estimators::OdometryAdapterCollection>
      adapters_;
  std::string uav_name_;
};

int main(int argc, char** argv) {
  // rosconsole底层使用log4cxx；显式启用UTF-8，避免中文日志被转换成问号。
  std::setlocale(LC_ALL, "C.UTF-8");
  ros::init(argc, argv, "odometry_adapter_manager");
  try {
    OdometryAdapterManager manager;
    ros::spin();
  } catch (const std::exception& exception) {
    ROS_FATAL("[odometry_adapter_manager] 启动失败: %s", exception.what());
    return 1;
  }
  return 0;
}
