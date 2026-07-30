#include <algorithm>
#include <clocale>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <xmlrpcpp/XmlRpcValue.h>

#include <xd_uav_state_estimators/Heading.h>
#include <xd_uav_state_estimators/PositionXY.h>
#include <xd_uav_state_estimators/PositionZ.h>
#include <xd_uav_state_estimators/VelocityXY.h>
#include <xd_uav_state_estimators/VelocityZ.h>
#include <xd_uav_state_estimators/YawRate.h>

namespace {

double wrapAngle(const double value) {
  return std::atan2(std::sin(value), std::cos(value));
}

std::string trimSlashes(std::string value) {
  while (!value.empty() && value.front() == '/') {
    value.erase(value.begin());
  }
  while (!value.empty() && value.back() == '/') {
    value.pop_back();
  }
  return value;
}

double validVariance(const double value, const double fallback) {
  return std::isfinite(value) && value > 0.0 ? value : fallback;
}

bool normalize(tf2::Quaternion* quaternion) {
  if (!std::isfinite(quaternion->x()) || !std::isfinite(quaternion->y()) ||
      !std::isfinite(quaternion->z()) || !std::isfinite(quaternion->w()) ||
      quaternion->length2() < 1e-9) {
    return false;
  }
  quaternion->normalize();
  return true;
}

bool hasMember(const XmlRpc::XmlRpcValue& value, const std::string& key) {
  return value.getType() == XmlRpc::XmlRpcValue::TypeStruct &&
         value.hasMember(key);
}

std::string readString(const XmlRpc::XmlRpcValue& value,
                       const std::string& key,
                       const std::string& fallback,
                       const bool required = false) {
  if (!hasMember(value, key)) {
    if (required) {
      throw std::runtime_error("缺少字符串配置项'" + key + "'");
    }
    return fallback;
  }
  if (value[key].getType() != XmlRpc::XmlRpcValue::TypeString) {
    throw std::runtime_error("配置项'" + key + "'必须是字符串");
  }
  return static_cast<std::string>(value[key]);
}

double readNumber(const XmlRpc::XmlRpcValue& value,
                  const std::string& key,
                  const double fallback) {
  if (!hasMember(value, key)) {
    return fallback;
  }
  if (value[key].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
    return static_cast<double>(value[key]);
  }
  if (value[key].getType() == XmlRpc::XmlRpcValue::TypeInt) {
    return static_cast<int>(value[key]);
  }
  throw std::runtime_error("配置项'" + key + "'必须是数值");
}

std::string scopedFrame(const std::string& raw_frame,
                        const std::string& uav_name) {
  const std::string frame = trimSlashes(raw_frame);
  if (frame.empty() || frame == "world" || frame == "earth" ||
      frame.find('/') != std::string::npos) {
    return frame;
  }
  return uav_name + "/" + frame;
}

}  // 匿名命名空间

class OdometryAdapter {
 public:
  OdometryAdapter(const ros::NodeHandle& node_handle,
                  tf2_ros::Buffer* tf_buffer,
                  const std::string& uav_name,
                  const std::string& adapter_name,
                  const XmlRpc::XmlRpcValue& config)
      : nh_(node_handle),
        tf_buffer_(tf_buffer),
        adapter_name_(adapter_name),
        uav_name_(uav_name) {
    if (config.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
      throw std::runtime_error("适配器'" + adapter_name_ + "'的配置必须是映射");
    }
    if (adapter_name_.empty() ||
        adapter_name_.find('/') != std::string::npos) {
      throw std::runtime_error("适配器名称不能为空，也不能包含斜杠");
    }

    input_topic_ = readString(config, "input_topic", std::string(), true);
    output_namespace_ = "state_estimator_inputs/" + adapter_name_;
    body_frame_ = scopedFrame(
        readString(config, "body_frame", "base_link"), uav_name_);
    parent_frame_override_ = scopedFrame(
        readString(config, "parent_frame_override", std::string()), uav_name_);
    child_frame_override_ = scopedFrame(
        readString(config, "child_frame_override", std::string()), uav_name_);
    tf_timeout_ = readNumber(config, "tf_timeout", 0.03);
    max_input_delay_ = readNumber(config, "max_input_delay", 0.50);

    if (hasMember(config, "fallback_covariance")) {
      const XmlRpc::XmlRpcValue& covariance = config["fallback_covariance"];
      if (covariance.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
        throw std::runtime_error("适配器'" + adapter_name_ +
                                 "'的fallback_covariance必须是映射");
      }
      fallback_position_variance_ =
          readNumber(covariance, "position", 0.05);
      fallback_velocity_variance_ =
          readNumber(covariance, "velocity", 0.10);
      fallback_heading_variance_ =
          readNumber(covariance, "heading", 0.05);
      fallback_yaw_rate_variance_ =
          readNumber(covariance, "yaw_rate", 0.10);
    }

    if (input_topic_.empty() || body_frame_.empty()) {
      throw std::runtime_error("适配器'" + adapter_name_ +
                               "'的输入话题和机体坐标系不能为空");
    }
    if (tf_timeout_ < 0.0 || max_input_delay_ <= 0.0) {
      throw std::runtime_error("适配器'" + adapter_name_ +
                               "'的时间参数超出有效范围");
    }

    position_xy_publisher_ =
        nh_.advertise<xd_uav_state_estimators::PositionXY>(
            output_namespace_ + "/position_xy", 20);
    position_z_publisher_ =
        nh_.advertise<xd_uav_state_estimators::PositionZ>(
            output_namespace_ + "/position_z", 20);
    velocity_xy_publisher_ =
        nh_.advertise<xd_uav_state_estimators::VelocityXY>(
            output_namespace_ + "/velocity_xy", 20);
    velocity_z_publisher_ =
        nh_.advertise<xd_uav_state_estimators::VelocityZ>(
            output_namespace_ + "/velocity_z", 20);
    heading_publisher_ =
        nh_.advertise<xd_uav_state_estimators::Heading>(
            output_namespace_ + "/heading", 20);
    yaw_rate_publisher_ =
        nh_.advertise<xd_uav_state_estimators::YawRate>(
            output_namespace_ + "/yaw_rate", 20);
    odometry_subscriber_ =
        nh_.subscribe(input_topic_, 30, &OdometryAdapter::odometryCallback,
                      this);

    ROS_INFO("[odometry_adapter_manager] 已加载适配器'%s': %s -> %s/*",
             adapter_name_.c_str(), nh_.resolveName(input_topic_).c_str(),
             nh_.resolveName(output_namespace_).c_str());
  }

 private:
  void odometryCallback(const nav_msgs::Odometry::ConstPtr& message) {
    const ros::Time now = ros::Time::now();
    const ros::Time stamp =
        message->header.stamp.isZero() ? now : message->header.stamp;
    if ((now - stamp).toSec() > max_input_delay_ ||
        stamp > now + ros::Duration(0.05)) {
      ROS_WARN_STREAM_THROTTLE(
          2.0, "[odometry_adapter_manager/" << adapter_name_
                                             << "] 输入时间戳超出允许范围");
      return;
    }

    const std::string parent_frame =
        parent_frame_override_.empty()
            ? trimSlashes(message->header.frame_id)
            : parent_frame_override_;
    const std::string input_child =
        child_frame_override_.empty()
            ? trimSlashes(message->child_frame_id)
            : child_frame_override_;
    if (parent_frame.empty() || input_child.empty()) {
      ROS_WARN_STREAM_THROTTLE(
          2.0, "[odometry_adapter_manager/" << adapter_name_
                                             << "] 输入坐标系名称为空");
      return;
    }

    tf2::Quaternion parent_child_rotation;
    tf2::fromMsg(message->pose.pose.orientation, parent_child_rotation);
    const auto& position = message->pose.pose.position;
    const auto& linear = message->twist.twist.linear;
    const auto& angular = message->twist.twist.angular;
    if (!normalize(&parent_child_rotation) ||
        !std::isfinite(position.x) || !std::isfinite(position.y) ||
        !std::isfinite(position.z) || !std::isfinite(linear.x) ||
        !std::isfinite(linear.y) || !std::isfinite(linear.z) ||
        !std::isfinite(angular.x) || !std::isfinite(angular.y) ||
        !std::isfinite(angular.z)) {
      ROS_WARN_STREAM_THROTTLE(
          2.0, "[odometry_adapter_manager/" << adapter_name_
                                             << "] 输入包含非法数值");
      return;
    }

    const tf2::Transform parent_child(
        parent_child_rotation,
        tf2::Vector3(position.x, position.y, position.z));
    tf2::Transform child_body = tf2::Transform::getIdentity();
    if (input_child != body_frame_) {
      try {
        const geometry_msgs::TransformStamped transform =
            tf_buffer_->lookupTransform(
                input_child, body_frame_, stamp, ros::Duration(tf_timeout_));
        tf2::fromMsg(transform.transform, child_body);
      } catch (const tf2::TransformException& exception) {
        ROS_WARN_STREAM_THROTTLE(
            2.0, "[odometry_adapter_manager/"
                     << adapter_name_
                     << "] 无法把输入参考点转换到base_link: "
                     << exception.what());
        return;
      }
    }

    const tf2::Transform parent_body = parent_child * child_body;
    const tf2::Vector3 linear_child(linear.x, linear.y, linear.z);
    const tf2::Vector3 angular_child(angular.x, angular.y, angular.z);
    const tf2::Vector3 velocity_at_body =
        linear_child + angular_child.cross(child_body.getOrigin());
    const tf2::Vector3 velocity_parent =
        tf2::quatRotate(parent_child_rotation, velocity_at_body);
    const tf2::Vector3 angular_body =
        tf2::quatRotate(child_body.getRotation().inverse(), angular_child);

    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    tf2::Matrix3x3(parent_body.getRotation()).getRPY(roll, pitch, yaw);

    const double position_x_variance =
        validVariance(message->pose.covariance[0],
                      fallback_position_variance_);
    const double position_y_variance =
        validVariance(message->pose.covariance[7],
                      fallback_position_variance_);
    const double position_z_variance =
        validVariance(message->pose.covariance[14],
                      fallback_position_variance_);
    const double heading_variance =
        validVariance(message->pose.covariance[35],
                      fallback_heading_variance_);
    const double velocity_x_variance =
        validVariance(message->twist.covariance[0],
                      fallback_velocity_variance_);
    const double velocity_y_variance =
        validVariance(message->twist.covariance[7],
                      fallback_velocity_variance_);
    const double velocity_z_variance =
        validVariance(message->twist.covariance[14],
                      fallback_velocity_variance_);
    const double yaw_rate_variance =
        validVariance(message->twist.covariance[35],
                      fallback_yaw_rate_variance_);

    xd_uav_state_estimators::PositionXY position_xy;
    position_xy.header.stamp = stamp;
    position_xy.header.frame_id = parent_frame;
    position_xy.child_frame_id = body_frame_;
    position_xy.x = parent_body.getOrigin().x();
    position_xy.y = parent_body.getOrigin().y();
    position_xy.covariance[0] = position_x_variance;
    position_xy.covariance[1] = message->pose.covariance[1];
    position_xy.covariance[2] = message->pose.covariance[6];
    position_xy.covariance[3] = position_y_variance;
    position_xy.valid = true;
    position_xy_publisher_.publish(position_xy);

    xd_uav_state_estimators::PositionZ position_z;
    position_z.header = position_xy.header;
    position_z.child_frame_id = body_frame_;
    position_z.z = parent_body.getOrigin().z();
    position_z.variance = position_z_variance;
    position_z.valid = true;
    position_z_publisher_.publish(position_z);

    xd_uav_state_estimators::VelocityXY velocity_xy;
    velocity_xy.header = position_xy.header;
    velocity_xy.child_frame_id = body_frame_;
    velocity_xy.x = velocity_parent.x();
    velocity_xy.y = velocity_parent.y();
    velocity_xy.covariance[0] = velocity_x_variance;
    velocity_xy.covariance[1] = message->twist.covariance[1];
    velocity_xy.covariance[2] = message->twist.covariance[6];
    velocity_xy.covariance[3] = velocity_y_variance;
    velocity_xy.valid = true;
    velocity_xy_publisher_.publish(velocity_xy);

    xd_uav_state_estimators::VelocityZ velocity_z;
    velocity_z.header = position_xy.header;
    velocity_z.child_frame_id = body_frame_;
    velocity_z.z = velocity_parent.z();
    velocity_z.variance = velocity_z_variance;
    velocity_z.valid = true;
    velocity_z_publisher_.publish(velocity_z);

    xd_uav_state_estimators::Heading heading;
    heading.header = position_xy.header;
    heading.child_frame_id = body_frame_;
    heading.heading = wrapAngle(yaw);
    heading.variance = heading_variance;
    heading.valid = true;
    heading_publisher_.publish(heading);

    xd_uav_state_estimators::YawRate yaw_rate;
    yaw_rate.header.stamp = stamp;
    yaw_rate.header.frame_id = body_frame_;
    yaw_rate.child_frame_id = body_frame_;
    yaw_rate.yaw_rate = angular_body.z();
    yaw_rate.variance = yaw_rate_variance;
    yaw_rate.valid = true;
    yaw_rate_publisher_.publish(yaw_rate);
  }

  ros::NodeHandle nh_;
  tf2_ros::Buffer* tf_buffer_;
  ros::Subscriber odometry_subscriber_;
  ros::Publisher position_xy_publisher_;
  ros::Publisher position_z_publisher_;
  ros::Publisher velocity_xy_publisher_;
  ros::Publisher velocity_z_publisher_;
  ros::Publisher heading_publisher_;
  ros::Publisher yaw_rate_publisher_;

  std::string adapter_name_;
  std::string uav_name_;
  std::string input_topic_;
  std::string output_namespace_;
  std::string body_frame_;
  std::string parent_frame_override_;
  std::string child_frame_override_;
  double tf_timeout_{0.03};
  double max_input_delay_{0.50};
  double fallback_position_variance_{0.05};
  double fallback_velocity_variance_{0.10};
  double fallback_heading_variance_{0.05};
  double fallback_yaw_rate_variance_{0.10};
};

class OdometryAdapterManager {
 public:
  OdometryAdapterManager()
      : private_nh_("~"), tf_listener_(tf_buffer_) {
    private_nh_.param("uav_name", uav_name_, std::string("uav1"));
    uav_name_ = trimSlashes(uav_name_);
    if (uav_name_.empty()) {
      throw std::runtime_error("uav_name不能为空");
    }

    XmlRpc::XmlRpcValue adapter_configs;
    if (!private_nh_.getParam("odometry_adapters", adapter_configs)) {
      throw std::runtime_error(
          "未找到odometry_adapters，请检查sources.yaml");
    }
    if (adapter_configs.getType() != XmlRpc::XmlRpcValue::TypeStruct ||
        adapter_configs.size() == 0) {
      throw std::runtime_error("odometry_adapters必须是非空映射");
    }

    for (auto iterator = adapter_configs.begin();
         iterator != adapter_configs.end(); ++iterator) {
      adapters_.push_back(std::make_unique<OdometryAdapter>(
          nh_, &tf_buffer_, uav_name_, iterator->first, iterator->second));
    }
    ROS_INFO("[odometry_adapter_manager] 共加载%d个Odometry适配器",
             static_cast<int>(adapters_.size()));
  }

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::vector<std::unique_ptr<OdometryAdapter>> adapters_;
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
