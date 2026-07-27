#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <XmlRpcValue.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

namespace {

constexpr double kPi = 3.14159265358979323846;

double wrapAngle(const double value) {
  return std::atan2(std::sin(value), std::cos(value));
}

double interpolateAngle(const double from, const double to, const double alpha) {
  return wrapAngle(from + alpha * wrapAngle(to - from));
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

std::string xmlString(const XmlRpc::XmlRpcValue& value, const std::string& key,
                      const std::string& fallback = "") {
  if (value.getType() != XmlRpc::XmlRpcValue::TypeStruct || !value.hasMember(key) ||
      value[key].getType() != XmlRpc::XmlRpcValue::TypeString) {
    return fallback;
  }
  return static_cast<std::string>(value[key]);
}

bool xmlBool(const XmlRpc::XmlRpcValue& value, const std::string& key, const bool fallback) {
  if (value.getType() != XmlRpc::XmlRpcValue::TypeStruct || !value.hasMember(key) ||
      value[key].getType() != XmlRpc::XmlRpcValue::TypeBoolean) {
    return fallback;
  }
  return static_cast<bool>(value[key]);
}

double xmlNumber(const XmlRpc::XmlRpcValue& value) {
  if (value.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
    return static_cast<double>(value);
  }
  if (value.getType() == XmlRpc::XmlRpcValue::TypeInt) {
    return static_cast<int>(value);
  }
  throw std::runtime_error("expected a numeric YAML value");
}

std::array<double, 3> xmlVector3(const XmlRpc::XmlRpcValue& value,
                                const std::string& key) {
  if (value.getType() != XmlRpc::XmlRpcValue::TypeStruct || !value.hasMember(key) ||
      value[key].getType() != XmlRpc::XmlRpcValue::TypeArray ||
      value[key].size() != 3) {
    throw std::runtime_error(key + " must be a three-element list");
  }
  return {{xmlNumber(value[key][0]), xmlNumber(value[key][1]), xmlNumber(value[key][2])}};
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

struct DynamicRule {
  std::string name;
  std::string type;
  std::string topic;
  std::string parent_frame;
  std::string child_frame;
  bool invert{false};
  ros::Subscriber subscriber;
  ros::Time last_message;
  std::uint64_t published{0};
  std::uint64_t rejected{0};
};

}  // 匿名命名空间

class SingleTfManager {
 public:
  SingleTfManager() : private_nh_("~") {
    odom_body_transform_.setIdentity();
    local_body_transform_.setIdentity();
    private_nh_.param("uav_name", uav_name_, std::string("uav1"));
    uav_name_ = trimSlashes(uav_name_);
    private_nh_.param("publish_rate", publish_rate_, 30.0);
    private_nh_.param("input_timeout", input_timeout_, 1.0);
    loadLocalAlignment();
    loadStaticTransforms();
    loadDynamicTransforms();
    static_broadcaster_.sendTransform(static_transforms_);

    alignment_valid_publisher_ =
        private_nh_.advertise<std_msgs::Bool>("local_alignment_valid", 1, true);
    diagnostics_publisher_ =
        private_nh_.advertise<diagnostic_msgs::DiagnosticArray>("diagnostics", 2);
    timer_ = private_nh_.createTimer(
        ros::Duration(1.0 / std::max(1.0, publish_rate_)),
        &SingleTfManager::timerCallback, this);
    publishBoolean(alignment_valid_publisher_, false);
    ROS_INFO("[xd_uav_single_tf_manager] %s管理%zu个子坐标系",
             uav_name_.c_str(), child_owners_.size());
  }

 private:
  std::string scopedFrame(const std::string& configured) const {
    const std::string frame = trimSlashes(configured);
    if (frame.empty() || frame == "world" || frame == "earth" ||
        frame.find('/') != std::string::npos) {
      return frame;
    }
    return uav_name_ + "/" + frame;
  }

  void claimChild(const std::string& child, const std::string& owner) {
    if (child.empty()) {
      throw std::runtime_error(owner + " has an empty child frame");
    }
    const auto result = child_owners_.emplace(child, owner);
    if (!result.second) {
      throw std::runtime_error("child frame '" + child + "' is configured by both '" +
                               result.first->second + "' and '" + owner + "'");
    }
  }

  void loadLocalAlignment() {
    private_nh_.param("local_alignment/enabled", local_alignment_enabled_, true);
    if (!local_alignment_enabled_) {
      return;
    }
    std::string parent;
    std::string child;
    private_nh_.param("local_alignment/parent_frame", parent,
                      std::string("local_origin"));
    private_nh_.param("local_alignment/child_frame", child, std::string("odom"));
    local_parent_frame_ = scopedFrame(parent);
    local_child_frame_ = scopedFrame(child);
    claimChild(local_child_frame_, "local_alignment");

    private_nh_.param("local_alignment/fallback_identity",
                      fallback_identity_, true);
    private_nh_.param("local_alignment/correction_topic",
                      correction_topic_, std::string());
    private_nh_.param("local_alignment/odometry_topic",
                      odometry_topic_,
                      std::string("state_estimator/main/odom"));
    private_nh_.param("local_alignment/max_position_variance",
                      max_position_variance_, 100.0);
    private_nh_.param("local_alignment/position_filter_alpha",
                      position_filter_alpha_, 0.08);
    private_nh_.param("local_alignment/yaw_filter_alpha", yaw_filter_alpha_, 0.08);
    private_nh_.param("local_alignment/max_position_jump", max_position_jump_, 10.0);
    private_nh_.param("local_alignment/max_yaw_jump", max_yaw_jump_, kPi / 2.0);
    position_filter_alpha_ = std::clamp(position_filter_alpha_, 0.0, 1.0);
    yaw_filter_alpha_ = std::clamp(yaw_filter_alpha_, 0.0, 1.0);

    if (!correction_topic_.empty()) {
      odometry_subscriber_ = nh_.subscribe(
          odometry_topic_, 30,
          &SingleTfManager::odometryCallback, this);
      correction_subscriber_ = nh_.subscribe(
          correction_topic_, 20, &SingleTfManager::correctionCallback, this);
    }
  }

  void loadStaticTransforms() {
    XmlRpc::XmlRpcValue rules;
    if (!private_nh_.getParam("static_transforms", rules)) {
      return;
    }
    if (rules.getType() != XmlRpc::XmlRpcValue::TypeArray) {
      throw std::runtime_error("static_transforms must be a YAML list");
    }
    for (int index = 0; index < rules.size(); ++index) {
      if (!xmlBool(rules[index], "enabled", true)) {
        continue;
      }
      const std::string name =
          xmlString(rules[index], "name", "static_" + std::to_string(index));
      const std::string parent =
          scopedFrame(xmlString(rules[index], "parent_frame"));
      const std::string child =
          scopedFrame(xmlString(rules[index], "child_frame"));
      if (parent.empty()) {
        throw std::runtime_error(name + " has an empty parent frame");
      }
      claimChild(child, "static:" + name);
      const auto translation = xmlVector3(rules[index], "translation");
      const auto rotation_rpy = xmlVector3(rules[index], "rotation_rpy");
      geometry_msgs::TransformStamped transform;
      transform.header.stamp = ros::Time::now();
      transform.header.frame_id = parent;
      transform.child_frame_id = child;
      transform.transform.translation.x = translation[0];
      transform.transform.translation.y = translation[1];
      transform.transform.translation.z = translation[2];
      tf2::Quaternion quaternion;
      quaternion.setRPY(rotation_rpy[0], rotation_rpy[1], rotation_rpy[2]);
      transform.transform.rotation = tf2::toMsg(quaternion);
      static_transforms_.push_back(transform);
    }
  }

  void loadDynamicTransforms() {
    XmlRpc::XmlRpcValue rules;
    if (!private_nh_.getParam("dynamic_transforms", rules)) {
      return;
    }
    if (rules.getType() != XmlRpc::XmlRpcValue::TypeArray) {
      throw std::runtime_error("dynamic_transforms must be a YAML list");
    }
    for (int index = 0; index < rules.size(); ++index) {
      if (!xmlBool(rules[index], "enabled", true)) {
        continue;
      }
      auto rule = std::make_unique<DynamicRule>();
      rule->name = xmlString(rules[index], "name", "dynamic_" + std::to_string(index));
      rule->type = xmlString(rules[index], "type");
      rule->topic = xmlString(rules[index], "topic");
      rule->parent_frame =
          scopedFrame(xmlString(rules[index], "parent_frame"));
      rule->child_frame =
          scopedFrame(xmlString(rules[index], "child_frame"));
      rule->invert = xmlBool(rules[index], "invert", false);
      if (rule->topic.empty() || rule->parent_frame.empty() || rule->child_frame.empty()) {
        throw std::runtime_error("dynamic rule '" + rule->name +
                                 "' needs topic, parent_frame and child_frame");
      }
      claimChild(rule->child_frame, "dynamic:" + rule->name);
      DynamicRule* pointer = rule.get();
      if (rule->type == "odometry") {
        rule->subscriber = nh_.subscribe<nav_msgs::Odometry>(
            rule->topic, 20,
            [this, pointer](const nav_msgs::Odometry::ConstPtr& message) {
              odometryRuleCallback(pointer, message);
            });
      } else if (rule->type == "pose_stamped") {
        rule->subscriber = nh_.subscribe<geometry_msgs::PoseStamped>(
            rule->topic, 20,
            [this, pointer](const geometry_msgs::PoseStamped::ConstPtr& message) {
              poseRuleCallback(pointer, message);
            });
      } else if (rule->type == "transform_stamped") {
        rule->subscriber = nh_.subscribe<geometry_msgs::TransformStamped>(
            rule->topic, 20,
            [this, pointer](const geometry_msgs::TransformStamped::ConstPtr& message) {
              transformRuleCallback(pointer, message);
            });
      } else {
        throw std::runtime_error("unsupported dynamic transform type '" + rule->type + "'");
      }
      dynamic_rules_.push_back(std::move(rule));
    }
  }

  bool buildTransform(const std::string& parent, const std::string& child,
                      const ros::Time& stamp, const geometry_msgs::Vector3& translation,
                      const geometry_msgs::Quaternion& rotation, const bool invert,
                      geometry_msgs::TransformStamped* output) {
    tf2::Quaternion quaternion;
    tf2::fromMsg(rotation, quaternion);
    if (!normalize(&quaternion) || !std::isfinite(translation.x) ||
        !std::isfinite(translation.y) || !std::isfinite(translation.z)) {
      return false;
    }
    tf2::Transform transform(
        quaternion, tf2::Vector3(translation.x, translation.y, translation.z));
    if (invert) {
      transform = transform.inverse();
    }
    output->header.stamp = stamp.isZero() ? ros::Time::now() : stamp;
    output->header.frame_id = parent;
    output->child_frame_id = child;
    output->transform = tf2::toMsg(transform);
    return true;
  }

  void odometryCallback(const nav_msgs::Odometry::ConstPtr& message) {
    tf2::Quaternion rotation;
    tf2::fromMsg(message->pose.pose.orientation, rotation);
    if (!normalize(&rotation) ||
        !std::isfinite(message->pose.pose.position.x) ||
        !std::isfinite(message->pose.pose.position.y) ||
        !std::isfinite(message->pose.pose.position.z)) {
      ROS_WARN_THROTTLE(
          2.0,
          "[xd_uav_single_tf_manager] 主估计里程计位姿无效");
      return;
    }
    odom_body_transform_.setOrigin(
        tf2::Vector3(message->pose.pose.position.x,
                     message->pose.pose.position.y,
                     message->pose.pose.position.z));
    odom_body_transform_.setRotation(rotation);
    have_main_pose_ = true;
    last_main_receive_ = ros::Time::now();
    updateAlignment();
  }

  void correctionCallback(const nav_msgs::Odometry::ConstPtr& message) {
    tf2::Quaternion quaternion;
    tf2::fromMsg(message->pose.pose.orientation, quaternion);
    if (!normalize(&quaternion) || !std::isfinite(message->pose.pose.position.x) ||
        !std::isfinite(message->pose.pose.position.y) ||
        !std::isfinite(message->pose.pose.position.z)) {
      ROS_WARN_THROTTLE(
          2.0, "[xd_uav_single_tf_manager] 局部对齐修正无效");
      return;
    }

    double largest_variance = 0.0;
    for (const std::size_t index : {std::size_t{0}, std::size_t{7},
                                    std::size_t{14}}) {
      const double variance = message->pose.covariance[index];
      if (std::isfinite(variance) && variance > 0.0) {
        largest_variance = std::max(largest_variance, variance);
      }
    }
    if (largest_variance > max_position_variance_) {
      ROS_WARN_THROTTLE(
          2.0, "[xd_uav_single_tf_manager] 局部对齐修正协方差过大");
      return;
    }

    local_body_transform_.setOrigin(
        tf2::Vector3(message->pose.pose.position.x, message->pose.pose.position.y,
                     message->pose.pose.position.z));
    local_body_transform_.setRotation(quaternion);
    have_correction_pose_ = true;
    last_correction_receive_ = ros::Time::now();
    updateAlignment();
  }

  bool alignmentInputsFresh(const ros::Time& now) const {
    return have_main_pose_ && have_correction_pose_ &&
           (now - last_main_receive_).toSec() <= input_timeout_ &&
           (now - last_correction_receive_).toSec() <= input_timeout_;
  }

  void updateAlignment() {
    const ros::Time now = ros::Time::now();
    if (!alignmentInputsFresh(now)) {
      return;
    }

    // 已知local_origin -> base_link和odom -> base_link，
    // 反算local_origin -> odom，避免在本包内引入GPS或世界坐标逻辑。
    const tf2::Transform candidate =
        local_body_transform_ * odom_body_transform_.inverse();
    double roll = 0.0;
    double pitch = 0.0;
    double candidate_yaw = 0.0;
    tf2::Matrix3x3(candidate.getRotation()).getRPY(roll, pitch, candidate_yaw);
    candidate_yaw = wrapAngle(candidate_yaw);
    const std::array<double, 3> candidate_translation{{
        candidate.getOrigin().x(),
        candidate.getOrigin().y(),
        candidate.getOrigin().z()}};

    if (!alignment_initialized_) {
      alignment_translation_ = candidate_translation;
      alignment_yaw_ = candidate_yaw;
      alignment_initialized_ = true;
      return;
    }
    const double dx = candidate_translation[0] - alignment_translation_[0];
    const double dy = candidate_translation[1] - alignment_translation_[1];
    const double dz = candidate_translation[2] - alignment_translation_[2];
    if (std::sqrt(dx * dx + dy * dy + dz * dz) > max_position_jump_ ||
        std::abs(wrapAngle(candidate_yaw - alignment_yaw_)) > max_yaw_jump_) {
      ROS_WARN_THROTTLE(
          2.0, "[xd_uav_single_tf_manager] 拒绝局部对齐跳变");
      return;
    }
    for (std::size_t index = 0; index < alignment_translation_.size(); ++index) {
      alignment_translation_[index] +=
          position_filter_alpha_ *
          (candidate_translation[index] - alignment_translation_[index]);
    }
    alignment_yaw_ =
        interpolateAngle(alignment_yaw_, candidate_yaw, yaw_filter_alpha_);
  }

  void odometryRuleCallback(DynamicRule* rule,
                            const nav_msgs::Odometry::ConstPtr& message) {
    geometry_msgs::Vector3 translation;
    translation.x = message->pose.pose.position.x;
    translation.y = message->pose.pose.position.y;
    translation.z = message->pose.pose.position.z;
    publishRuleTransform(rule, message->header.stamp, translation,
                         message->pose.pose.orientation);
  }

  void poseRuleCallback(DynamicRule* rule,
                        const geometry_msgs::PoseStamped::ConstPtr& message) {
    geometry_msgs::Vector3 translation;
    translation.x = message->pose.position.x;
    translation.y = message->pose.position.y;
    translation.z = message->pose.position.z;
    publishRuleTransform(rule, message->header.stamp, translation,
                         message->pose.orientation);
  }

  void transformRuleCallback(
      DynamicRule* rule, const geometry_msgs::TransformStamped::ConstPtr& message) {
    publishRuleTransform(rule, message->header.stamp, message->transform.translation,
                         message->transform.rotation);
  }

  void publishRuleTransform(DynamicRule* rule, const ros::Time& stamp,
                            const geometry_msgs::Vector3& translation,
                            const geometry_msgs::Quaternion& rotation) {
    geometry_msgs::TransformStamped transform;
    if (!buildTransform(rule->parent_frame, rule->child_frame, stamp, translation,
                        rotation, rule->invert, &transform)) {
      rule->rejected++;
      return;
    }
    dynamic_broadcaster_.sendTransform(transform);
    rule->last_message = ros::Time::now();
    rule->published++;
  }

  void timerCallback(const ros::TimerEvent&) {
    const ros::Time now = ros::Time::now();
    const bool alignment_valid =
        local_alignment_enabled_ && alignment_initialized_ && alignmentInputsFresh(now);
    publishBoolean(alignment_valid_publisher_, alignment_valid);

    if (local_alignment_enabled_ &&
        (alignment_initialized_ || fallback_identity_)) {
      geometry_msgs::TransformStamped transform;
      transform.header.stamp = now;
      transform.header.frame_id = local_parent_frame_;
      transform.child_frame_id = local_child_frame_;
      transform.transform.translation.x =
          alignment_initialized_ ? alignment_translation_[0] : 0.0;
      transform.transform.translation.y =
          alignment_initialized_ ? alignment_translation_[1] : 0.0;
      transform.transform.translation.z =
          alignment_initialized_ ? alignment_translation_[2] : 0.0;
      tf2::Quaternion quaternion;
      quaternion.setRPY(0.0, 0.0,
                        alignment_initialized_ ? alignment_yaw_ : 0.0);
      transform.transform.rotation = tf2::toMsg(quaternion);
      dynamic_broadcaster_.sendTransform(transform);
    }
    publishDiagnostics(now, alignment_valid);
  }

  void publishDiagnostics(const ros::Time& now,
                          const bool alignment_valid) {
    if (!last_diagnostics_.isZero() &&
        (now - last_diagnostics_).toSec() < 0.5) {
      return;
    }
    last_diagnostics_ = now;
    diagnostic_msgs::DiagnosticArray array;
    array.header.stamp = now;
    diagnostic_msgs::DiagnosticStatus status;
    status.name = uav_name_ + "/single_tf_manager";
    status.hardware_id = uav_name_;
    const bool fallback_in_use =
        local_alignment_enabled_ && fallback_identity_ &&
        !alignment_initialized_;
    const bool local_chain_available =
        !local_alignment_enabled_ || alignment_valid ||
        fallback_in_use;
    status.level =
        local_chain_available
            ? diagnostic_msgs::DiagnosticStatus::OK
            : diagnostic_msgs::DiagnosticStatus::WARN;
    status.message =
        !local_alignment_enabled_
            ? "局部对齐层已禁用"
            : (alignment_valid
                   ? "局部对齐有效"
                   : (fallback_in_use ? "使用局部单位对齐"
                                      : "等待局部对齐修正"));
    addDiagnostic(&status, "local_alignment_valid", alignment_valid);
    addDiagnostic(&status, "local_alignment_identity_fallback",
                  fallback_in_use);
    addDiagnostic(&status, "owned_children",
                  static_cast<int>(child_owners_.size()));
    for (const auto& rule : dynamic_rules_) {
      addDiagnostic(&status, rule->name + "/published",
                    static_cast<int>(rule->published));
      addDiagnostic(&status, rule->name + "/rejected",
                    static_cast<int>(rule->rejected));
    }
    array.status.push_back(status);
    diagnostics_publisher_.publish(array);
  }

  static void addDiagnostic(diagnostic_msgs::DiagnosticStatus* status,
                            const std::string& key, const bool value) {
    diagnostic_msgs::KeyValue entry;
    entry.key = key;
    entry.value = value ? "true" : "false";
    status->values.push_back(entry);
  }

  static void addDiagnostic(diagnostic_msgs::DiagnosticStatus* status,
                            const std::string& key, const int value) {
    diagnostic_msgs::KeyValue entry;
    entry.key = key;
    entry.value = std::to_string(value);
    status->values.push_back(entry);
  }

  static void publishBoolean(const ros::Publisher& publisher, const bool value) {
    std_msgs::Bool message;
    message.data = value;
    publisher.publish(message);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  tf2_ros::TransformBroadcaster dynamic_broadcaster_;
  tf2_ros::StaticTransformBroadcaster static_broadcaster_;
  ros::Subscriber odometry_subscriber_;
  ros::Subscriber correction_subscriber_;
  ros::Publisher alignment_valid_publisher_;
  ros::Publisher diagnostics_publisher_;
  ros::Timer timer_;

  std::string uav_name_;
  std::string odometry_topic_;
  std::string local_parent_frame_;
  std::string local_child_frame_;
  std::string correction_topic_;
  std::unordered_map<std::string, std::string> child_owners_;
  std::vector<geometry_msgs::TransformStamped> static_transforms_;
  std::vector<std::unique_ptr<DynamicRule>> dynamic_rules_;
  tf2::Transform odom_body_transform_;
  tf2::Transform local_body_transform_;
  std::array<double, 3> alignment_translation_{{0.0, 0.0, 0.0}};
  ros::Time last_main_receive_;
  ros::Time last_correction_receive_;
  ros::Time last_diagnostics_;
  double alignment_yaw_{0.0};
  double publish_rate_{30.0};
  double input_timeout_{1.0};
  double max_position_variance_{100.0};
  double position_filter_alpha_{0.08};
  double yaw_filter_alpha_{0.08};
  double max_position_jump_{10.0};
  double max_yaw_jump_{kPi / 2.0};
  bool local_alignment_enabled_{true};
  bool fallback_identity_{true};
  bool have_main_pose_{false};
  bool have_correction_pose_{false};
  bool alignment_initialized_{false};
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "single_tf_manager");
  try {
    SingleTfManager manager;
    ros::spin();
  } catch (const std::exception& exception) {
    ROS_FATAL("[xd_uav_single_tf_manager] 启动失败: %s", exception.what());
    return 1;
  }
  return 0;
}
