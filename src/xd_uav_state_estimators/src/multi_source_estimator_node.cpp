#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <XmlRpcValue.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Dense>

#include <xd_uav_state_estimators/EstimatorStatus.h>
#include <xd_uav_state_estimators/SwitchLocalizationSource.h>

namespace {

constexpr double kPi = 3.14159265358979323846;

double wrapAngle(const double value) {
  return std::atan2(std::sin(value), std::cos(value));
}

bool finiteQuaternion(const geometry_msgs::Quaternion& quaternion) {
  return std::isfinite(quaternion.x) && std::isfinite(quaternion.y) &&
         std::isfinite(quaternion.z) && std::isfinite(quaternion.w);
}

bool normalizeQuaternion(const geometry_msgs::Quaternion& message, tf2::Quaternion* quaternion) {
  if (!finiteQuaternion(message)) {
    return false;
  }
  *quaternion = tf2::Quaternion(message.x, message.y, message.z, message.w);
  if (!std::isfinite(quaternion->length2()) || quaternion->length2() < 1e-9) {
    return false;
  }
  quaternion->normalize();
  return true;
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

std::string xmlString(const XmlRpc::XmlRpcValue& value, const std::string& key,
                      const std::string& fallback) {
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

double xmlDouble(const XmlRpc::XmlRpcValue& value, const std::string& key,
                 const double fallback) {
  if (value.getType() != XmlRpc::XmlRpcValue::TypeStruct || !value.hasMember(key)) {
    return fallback;
  }
  if (value[key].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
    return static_cast<double>(value[key]);
  }
  if (value[key].getType() == XmlRpc::XmlRpcValue::TypeInt) {
    return static_cast<int>(value[key]);
  }
  return fallback;
}

class AxisKalman {
 public:
  struct Parameters {
    double initial_position_variance{1.0};
    double initial_velocity_variance{1.0};
    double initial_acceleration_variance{10.0};
    double position_process_noise{0.05};
    double velocity_process_noise{0.5};
    double acceleration_process_noise{2.0};
  };

  AxisKalman() : AxisKalman(Parameters{}) {}

  explicit AxisKalman(Parameters parameters) : parameters_(parameters) {
    covariance_.setIdentity();
  }

  void initialize(const double position, const double velocity, const double acceleration) {
    state_ << position, velocity, acceleration;
    covariance_.setZero();
    covariance_(0, 0) = parameters_.initial_position_variance;
    covariance_(1, 1) = parameters_.initial_velocity_variance;
    covariance_(2, 2) = parameters_.initial_acceleration_variance;
    initialized_ = true;
  }

  void predict(const double dt) {
    if (!initialized_ || !std::isfinite(dt) || dt <= 0.0) {
      return;
    }
    Eigen::Matrix3d transition = Eigen::Matrix3d::Identity();
    transition(0, 1) = dt;
    transition(0, 2) = 0.5 * dt * dt;
    transition(1, 2) = dt;
    Eigen::Matrix3d process_noise = Eigen::Matrix3d::Zero();
    process_noise(0, 0) = parameters_.position_process_noise * dt;
    process_noise(1, 1) = parameters_.velocity_process_noise * dt;
    process_noise(2, 2) = parameters_.acceleration_process_noise * dt;
    state_ = transition * state_;
    covariance_ = transition * covariance_ * transition.transpose() + process_noise;
  }

  double innovation(const int index, const double measurement) const {
    return measurement - state_(index);
  }

  double innovationVariance(const int index, const double measurement_variance) const {
    return covariance_(index, index) + measurement_variance;
  }

  double nis(const int index, const double measurement, const double measurement_variance) const {
    const double variance = innovationVariance(index, measurement_variance);
    if (!std::isfinite(variance) || variance <= 1e-12) {
      return std::numeric_limits<double>::infinity();
    }
    const double residual = innovation(index, measurement);
    return residual * residual / variance;
  }

  void correct(const int index, const double measurement, const double measurement_variance) {
    if (!initialized_) {
      return;
    }
    Eigen::RowVector3d observation = Eigen::RowVector3d::Zero();
    observation(index) = 1.0;
    const double residual = measurement - (observation * state_)(0);
    const double innovation_variance =
        (observation * covariance_ * observation.transpose())(0, 0) + measurement_variance;
    if (!std::isfinite(innovation_variance) || innovation_variance <= 1e-12) {
      return;
    }
    const Eigen::Vector3d gain = covariance_ * observation.transpose() / innovation_variance;
    state_ += gain * residual;
    covariance_ =
        (Eigen::Matrix3d::Identity() - gain * observation) * covariance_;
    covariance_ = 0.5 * (covariance_ + covariance_.transpose());
  }

  double position() const { return state_(0); }
  double velocity() const { return state_(1); }
  double acceleration() const { return state_(2); }
  double variance(const int index) const { return covariance_(index, index); }
  bool initialized() const { return initialized_; }

 private:
  Parameters parameters_;
  Eigen::Vector3d state_{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d covariance_{Eigen::Matrix3d::Identity()};
  bool initialized_{false};
};

class YawKalman {
 public:
  void configure(const double yaw_process_noise, const double rate_process_noise) {
    yaw_process_noise_ = yaw_process_noise;
    rate_process_noise_ = rate_process_noise;
  }

  void initialize(const double yaw, const double rate) {
    state_ << wrapAngle(yaw), rate;
    covariance_.setZero();
    covariance_(0, 0) = 0.25;
    covariance_(1, 1) = 0.25;
    initialized_ = true;
  }

  void predict(const double dt) {
    if (!initialized_ || !std::isfinite(dt) || dt <= 0.0) {
      return;
    }
    Eigen::Matrix2d transition = Eigen::Matrix2d::Identity();
    transition(0, 1) = dt;
    Eigen::Matrix2d process_noise = Eigen::Matrix2d::Zero();
    process_noise(0, 0) = yaw_process_noise_ * dt;
    process_noise(1, 1) = rate_process_noise_ * dt;
    state_ = transition * state_;
    state_(0) = wrapAngle(state_(0));
    covariance_ = transition * covariance_ * transition.transpose() + process_noise;
  }

  double yawInnovation(const double yaw) const { return wrapAngle(yaw - state_(0)); }

  double yawNis(const double yaw, const double variance) const {
    const double innovation_variance = covariance_(0, 0) + variance;
    if (innovation_variance <= 1e-12) {
      return std::numeric_limits<double>::infinity();
    }
    const double residual = yawInnovation(yaw);
    return residual * residual / innovation_variance;
  }

  void correctYaw(const double yaw, const double variance) {
    correct(0, yawInnovation(yaw), variance);
    state_(0) = wrapAngle(state_(0));
  }

  void correctRate(const double rate, const double variance) {
    correct(1, rate - state_(1), variance);
  }

  double yaw() const { return state_(0); }
  double rate() const { return state_(1); }
  double variance(const int index) const { return covariance_(index, index); }
  bool initialized() const { return initialized_; }

 private:
  void correct(const int index, const double residual, const double variance) {
    Eigen::RowVector2d observation = Eigen::RowVector2d::Zero();
    observation(index) = 1.0;
    const double innovation_variance =
        (observation * covariance_ * observation.transpose())(0, 0) + variance;
    if (!std::isfinite(innovation_variance) || innovation_variance <= 1e-12) {
      return;
    }
    const Eigen::Vector2d gain = covariance_ * observation.transpose() / innovation_variance;
    state_ += gain * residual;
    covariance_ =
        (Eigen::Matrix2d::Identity() - gain * observation) * covariance_;
    covariance_ = 0.5 * (covariance_ + covariance_.transpose());
  }

  Eigen::Vector2d state_{Eigen::Vector2d::Zero()};
  Eigen::Matrix2d covariance_{Eigen::Matrix2d::Identity()};
  double yaw_process_noise_{0.02};
  double rate_process_noise_{0.2};
  bool initialized_{false};
};

struct SourceConfig {
  std::string name;
  std::string topic;
  std::string role{"fallback"};
  std::string alignment_mode{"align_on_activation"};
  int priority{0};
  bool enabled{true};
  bool use_position_xy{true};
  bool use_altitude{true};
  bool use_velocity_xy{true};
  bool use_vertical_velocity{true};
  bool use_heading{true};
  bool twist_in_body_frame{true};
  bool use_message_covariance{true};
  bool use_tf_for_child_frame{true};
  bool require_tf{true};
  double tf_timeout{0.02};
  double timeout{0.25};
  double position_xy_variance{0.01};
  double position_z_variance{0.05};
  double velocity_xy_variance{0.01};
  double velocity_z_variance{0.02};
  double heading_variance{0.02};
  double max_position_variance{100.0};
  double max_velocity_variance{100.0};
  double max_heading_variance{10.0};
  int max_consecutive_rejections{5};
  double quarantine_duration{1.0};
  int recovery_min_samples{20};
  double recovery_stable_time{0.5};
};

struct Measurement {
  ros::Time stamp;
  std::string parent_frame;
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
  tf2::Quaternion orientation{0.0, 0.0, 0.0, 1.0};
  Eigen::Vector3d angular_velocity_body{Eigen::Vector3d::Zero()};
  double yaw{0.0};
  double position_xy_variance{0.01};
  double position_z_variance{0.05};
  double velocity_xy_variance{0.01};
  double velocity_z_variance{0.02};
  double heading_variance{0.02};
};

struct Estimate {
  ros::Time stamp;
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
  Eigen::Vector3d acceleration{Eigen::Vector3d::Zero()};
  double yaw{0.0};
  double yaw_rate{0.0};
  double roll{0.0};
  double pitch{0.0};
  std::array<double, 3> position_variance{{1.0, 1.0, 1.0}};
  std::array<double, 3> velocity_variance{{1.0, 1.0, 1.0}};
  double yaw_variance{0.25};
};

struct FrameOutput {
  std::string name;
  std::string target_frame;
  std::string topic;
  double lookup_timeout{0.02};
  ros::Publisher publisher;
};

struct SourceRuntime {
  SourceRuntime(SourceConfig source_config, const AxisKalman::Parameters& parameters,
                const double yaw_process_noise, const double yaw_rate_process_noise)
      : config(std::move(source_config)),
        axis{{AxisKalman(parameters), AxisKalman(parameters), AxisKalman(parameters)}} {
    yaw_filter.configure(yaw_process_noise, yaw_rate_process_noise);
  }

  SourceConfig config;
  std::array<AxisKalman, 3> axis;
  YawKalman yaw_filter;
  ros::Subscriber subscriber;
  ros::Publisher odometry_publisher;
  ros::Publisher valid_publisher;
  ros::Publisher alignment_publisher;
  Measurement latest_raw;
  bool have_raw{false};
  bool initialized{false};
  bool alignment_initialized{false};
  double alignment_yaw{0.0};
  Eigen::Vector3d alignment_translation{Eigen::Vector3d::Zero()};
  ros::Time filter_stamp;
  ros::Time last_received;
  ros::Time last_accepted;
  ros::Time quarantine_until;
  ros::Time recovery_started;
  int consecutive_rejections{0};
  int recovery_samples{0};
  bool recovering{false};
  std::uint64_t received{0};
  std::uint64_t adapter_rejected{0};
  std::uint64_t correction_rejected{0};
  std::uint64_t accepted{0};
  double latest_roll{0.0};
  double latest_pitch{0.0};
};

}  // 匿名命名空间

class MultiSourceEstimatorNode {
 public:
  MultiSourceEstimatorNode()
      : private_nh_("~"), tf_listener_(tf_buffer_) {
    loadParameters();
    loadSources();
    loadFrameOutputs();

    imu_subscriber_ = nh_.subscribe("imu", 100, &MultiSourceEstimatorNode::imuCallback, this);
    main_odometry_publisher_ = private_nh_.advertise<nav_msgs::Odometry>("main/odom", 10);
    localization_valid_publisher_ =
        private_nh_.advertise<std_msgs::Bool>("localization_valid", 1, true);
    state_valid_publisher_ = private_nh_.advertise<std_msgs::Bool>("state_valid", 1, true);
    status_publisher_ =
        private_nh_.advertise<xd_uav_state_estimators::EstimatorStatus>("status", 2, true);
    diagnostics_publisher_ =
        private_nh_.advertise<diagnostic_msgs::DiagnosticArray>("diagnostics", 2);
    reset_server_ =
        private_nh_.advertiseService("reset", &MultiSourceEstimatorNode::resetCallback, this);
    switch_server_ = private_nh_.advertiseService(
        "switch_source", &MultiSourceEstimatorNode::switchSourceCallback, this);
    output_timer_ = private_nh_.createTimer(
        ros::Duration(1.0 / std::max(1.0, output_rate_)),
        &MultiSourceEstimatorNode::outputTimerCallback, this);
    diagnostics_timer_ = private_nh_.createTimer(
        ros::Duration(1.0 / std::max(0.2, diagnostics_rate_)),
        &MultiSourceEstimatorNode::diagnosticsTimerCallback, this);

    publishBoolean(localization_valid_publisher_, false);
    publishBoolean(state_valid_publisher_, false);
    ROS_INFO_STREAM("[xd_uav_state_estimators] initialized for " << uav_name_
                    << " with " << sources_.size()
                    << " source(s); this node does not publish TF");
  }

 private:
  void loadParameters() {
    private_nh_.param("uav_name", uav_name_, std::string("uav1"));
    uav_name_ = trimSlashes(uav_name_);
    private_nh_.param("odom_frame", odom_frame_, uav_name_ + "/odom");
    private_nh_.param("body_frame", body_frame_, uav_name_ + "/base_link");
    odom_frame_ = trimSlashes(odom_frame_);
    body_frame_ = trimSlashes(body_frame_);
    private_nh_.param("output_rate", output_rate_, 100.0);
    private_nh_.param("diagnostics_rate", diagnostics_rate_, 2.0);
    private_nh_.param("max_localization_delay", max_localization_delay_, 0.15);
    private_nh_.param("max_dead_reckoning_time", max_dead_reckoning_time_, 1.0);
    private_nh_.param("imu_timeout", imu_timeout_, 0.3);
    private_nh_.param("require_imu", require_imu_, false);
    private_nh_.param("output_twist_in_body_frame", output_twist_in_body_frame_, true);

    private_nh_.param("filter/initial_position_variance",
                      axis_parameters_.initial_position_variance, 1.0);
    private_nh_.param("filter/initial_velocity_variance",
                      axis_parameters_.initial_velocity_variance, 1.0);
    private_nh_.param("filter/initial_acceleration_variance",
                      axis_parameters_.initial_acceleration_variance, 10.0);
    private_nh_.param("filter/position_process_noise",
                      axis_parameters_.position_process_noise, 0.05);
    private_nh_.param("filter/velocity_process_noise",
                      axis_parameters_.velocity_process_noise, 0.5);
    private_nh_.param("filter/acceleration_process_noise",
                      axis_parameters_.acceleration_process_noise, 2.0);
    private_nh_.param("filter/yaw_process_noise", yaw_process_noise_, 0.02);
    private_nh_.param("filter/yaw_rate_process_noise", yaw_rate_process_noise_, 0.2);
    private_nh_.param("filter/imu_acceleration_variance", imu_acceleration_variance_, 0.5);
    private_nh_.param("filter/imu_yaw_rate_variance", imu_yaw_rate_variance_, 0.05);
    private_nh_.param("filter/remove_gravity", remove_gravity_, true);
    private_nh_.param("filter/gravity", gravity_, 9.80665);
    private_nh_.param("filter/acceleration_limit", acceleration_limit_, 30.0);

    private_nh_.param("innovation_gate/position_xy", position_xy_limit_, 5.0);
    private_nh_.param("innovation_gate/position_z", position_z_limit_, 3.0);
    private_nh_.param("innovation_gate/velocity_xy", velocity_xy_limit_, 5.0);
    private_nh_.param("innovation_gate/velocity_z", velocity_z_limit_, 3.0);
    private_nh_.param("innovation_gate/heading", heading_limit_, 1.57);
    private_nh_.param("innovation_gate/use_nis", use_nis_, true);
    private_nh_.param("innovation_gate/position_xy_nis", position_xy_nis_limit_, 9.21);
    private_nh_.param("innovation_gate/position_z_nis", position_z_nis_limit_, 6.63);
    private_nh_.param("innovation_gate/velocity_xy_nis", velocity_xy_nis_limit_, 9.21);
    private_nh_.param("innovation_gate/velocity_z_nis", velocity_z_nis_limit_, 6.63);
    private_nh_.param("innovation_gate/heading_nis", heading_nis_limit_, 6.63);

    private_nh_.param("source_switching/switch_back_to_primary", switch_back_to_primary_, true);
    private_nh_.param("source_switching/minimum_active_time", minimum_active_time_, 1.0);
    private_nh_.param("source_switching/cooldown", switch_cooldown_, 0.5);
  }

  void loadSources() {
    std::vector<std::string> names;
    if (!private_nh_.getParam("localization_sources", names) || names.empty()) {
      throw std::runtime_error("localization_sources must contain at least one source");
    }

    for (const std::string& name : names) {
      SourceConfig config;
      config.name = name;
      const std::string prefix = "sources/" + name + "/";
      private_nh_.param(prefix + "enabled", config.enabled, true);
      if (!config.enabled) {
        continue;
      }
      private_nh_.param(prefix + "topic", config.topic, std::string());
      private_nh_.param(prefix + "role", config.role, std::string("fallback"));
      private_nh_.param(prefix + "priority", config.priority, 0);
      private_nh_.param(prefix + "timeout", config.timeout, 0.25);
      private_nh_.param(prefix + "use_position_xy", config.use_position_xy, true);
      private_nh_.param(prefix + "use_altitude", config.use_altitude, true);
      private_nh_.param(prefix + "use_velocity_xy", config.use_velocity_xy, true);
      private_nh_.param(prefix + "use_vertical_velocity", config.use_vertical_velocity, true);
      private_nh_.param(prefix + "use_heading", config.use_heading, true);
      private_nh_.param(prefix + "twist_in_body_frame", config.twist_in_body_frame, true);
      private_nh_.param(prefix + "alignment/mode", config.alignment_mode,
                        std::string("align_on_activation"));
      private_nh_.param(prefix + "frame_transform/use_tf_for_child_frame",
                        config.use_tf_for_child_frame, true);
      private_nh_.param(prefix + "frame_transform/require_tf", config.require_tf, true);
      private_nh_.param(prefix + "frame_transform/timeout", config.tf_timeout, 0.02);
      private_nh_.param(prefix + "covariance/use_message",
                        config.use_message_covariance, true);
      private_nh_.param(prefix + "covariance/position_xy",
                        config.position_xy_variance, 0.01);
      private_nh_.param(prefix + "covariance/position_z",
                        config.position_z_variance, 0.05);
      private_nh_.param(prefix + "covariance/velocity_xy",
                        config.velocity_xy_variance, 0.01);
      private_nh_.param(prefix + "covariance/velocity_z",
                        config.velocity_z_variance, 0.02);
      private_nh_.param(prefix + "covariance/heading", config.heading_variance, 0.02);
      private_nh_.param(prefix + "reliability/max_position_variance",
                        config.max_position_variance, 100.0);
      private_nh_.param(prefix + "reliability/max_velocity_variance",
                        config.max_velocity_variance, 100.0);
      private_nh_.param(prefix + "reliability/max_heading_variance",
                        config.max_heading_variance, 10.0);
      private_nh_.param(prefix + "reliability/max_consecutive_rejections",
                        config.max_consecutive_rejections, 5);
      private_nh_.param(prefix + "reliability/quarantine_duration",
                        config.quarantine_duration, 1.0);
      private_nh_.param(prefix + "reliability/recovery_min_samples",
                        config.recovery_min_samples, 20);
      private_nh_.param(prefix + "reliability/recovery_stable_time",
                        config.recovery_stable_time, 0.5);
      if (config.topic.empty()) {
        throw std::runtime_error("source '" + name + "' has an empty topic");
      }

      auto source = std::make_unique<SourceRuntime>(
          config, axis_parameters_, yaw_process_noise_, yaw_rate_process_noise_);
      SourceRuntime* source_pointer = source.get();
      source->subscriber = nh_.subscribe<nav_msgs::Odometry>(
          config.topic, 30,
          [this, source_pointer](const nav_msgs::Odometry::ConstPtr& message) {
            sourceCallback(source_pointer, message);
          });
      source->odometry_publisher = private_nh_.advertise<nav_msgs::Odometry>(
          "sources/" + name + "/odom", 10);
      source->valid_publisher =
          private_nh_.advertise<std_msgs::Bool>("sources/" + name + "/valid", 1, true);
      source->alignment_publisher = private_nh_.advertise<geometry_msgs::TransformStamped>(
          "sources/" + name + "/alignment", 2, true);
      publishBoolean(source->valid_publisher, false);
      source_by_name_[name] = source_pointer;
      sources_.push_back(std::move(source));
      ROS_INFO_STREAM("[xd_uav_state_estimators] source '" << name << "' <- "
                      << nh_.resolveName(config.topic));
    }
  }

  void loadFrameOutputs() {
    XmlRpc::XmlRpcValue outputs;
    if (!private_nh_.getParam("frame_outputs", outputs)) {
      return;
    }
    if (outputs.getType() != XmlRpc::XmlRpcValue::TypeArray) {
      throw std::runtime_error("frame_outputs must be a YAML list");
    }
    for (int index = 0; index < outputs.size(); ++index) {
      if (!xmlBool(outputs[index], "enabled", true)) {
        continue;
      }
      FrameOutput output;
      output.name = xmlString(outputs[index], "name", "frame_" + std::to_string(index));
      output.target_frame = trimSlashes(xmlString(outputs[index], "target_frame", ""));
      output.topic = xmlString(outputs[index], "topic", "");
      output.lookup_timeout = xmlDouble(outputs[index], "lookup_timeout", 0.02);
      if (output.target_frame.empty() || output.topic.empty()) {
        throw std::runtime_error("enabled frame_outputs entries need target_frame and topic");
      }
      output.publisher = nh_.advertise<nav_msgs::Odometry>(output.topic, 5);
      frame_outputs_.push_back(output);
    }
  }

  bool convertMeasurement(SourceRuntime* source, const nav_msgs::Odometry& message,
                          Measurement* measurement, std::string* reason) {
    source->received++;
    const ros::Time now = ros::Time::now();
    measurement->stamp = message.header.stamp.isZero() ? now : message.header.stamp;
    if ((now - measurement->stamp).toSec() > max_localization_delay_ ||
        measurement->stamp > now + ros::Duration(0.05)) {
      *reason = "measurement timestamp outside allowed delay";
      return false;
    }
    measurement->parent_frame = trimSlashes(message.header.frame_id);
    const std::string child_frame = trimSlashes(message.child_frame_id);
    if (measurement->parent_frame.empty() || child_frame.empty()) {
      *reason = "empty frame_id or child_frame_id";
      return false;
    }

    tf2::Quaternion parent_child_rotation;
    if (!normalizeQuaternion(message.pose.pose.orientation, &parent_child_rotation)) {
      *reason = "invalid pose quaternion";
      return false;
    }
    const auto& position = message.pose.pose.position;
    if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
        !std::isfinite(position.z)) {
      *reason = "non-finite position";
      return false;
    }
    tf2::Transform parent_child(
        parent_child_rotation, tf2::Vector3(position.x, position.y, position.z));
    tf2::Transform child_body = tf2::Transform::getIdentity();

    if (child_frame != body_frame_ && source->config.use_tf_for_child_frame) {
      try {
        const geometry_msgs::TransformStamped transform = tf_buffer_.lookupTransform(
            child_frame, body_frame_, measurement->stamp,
            ros::Duration(source->config.tf_timeout));
        tf2::fromMsg(transform.transform, child_body);
      } catch (const tf2::TransformException& exception) {
        if (source->config.require_tf) {
          *reason = std::string("missing child-to-body TF: ") + exception.what();
          return false;
        }
        ROS_WARN_THROTTLE(2.0,
                          "[xd_uav_state_estimators] source '%s' assumes child '%s' is body '%s'",
                          source->config.name.c_str(), child_frame.c_str(), body_frame_.c_str());
      }
    } else if (child_frame != body_frame_ && source->config.require_tf) {
      *reason = "source child_frame differs from body_frame while TF conversion is disabled";
      return false;
    }

    const tf2::Transform parent_body = parent_child * child_body;
    measurement->position =
        Eigen::Vector3d(parent_body.getOrigin().x(), parent_body.getOrigin().y(),
                        parent_body.getOrigin().z());
    measurement->orientation = parent_body.getRotation();
    double roll = 0.0;
    double pitch = 0.0;
    tf2::Matrix3x3(measurement->orientation).getRPY(roll, pitch, measurement->yaw);

    const auto& linear = message.twist.twist.linear;
    const auto& angular = message.twist.twist.angular;
    if (!std::isfinite(linear.x) || !std::isfinite(linear.y) || !std::isfinite(linear.z) ||
        !std::isfinite(angular.x) || !std::isfinite(angular.y) ||
        !std::isfinite(angular.z)) {
      *reason = "non-finite twist";
      return false;
    }
    const tf2::Vector3 linear_input(linear.x, linear.y, linear.z);
    const tf2::Vector3 angular_input(angular.x, angular.y, angular.z);
    if (source->config.twist_in_body_frame) {
      const tf2::Vector3 velocity_at_body =
          linear_input + angular_input.cross(child_body.getOrigin());
      const tf2::Vector3 velocity_parent =
          tf2::quatRotate(parent_child_rotation, velocity_at_body);
      measurement->velocity =
          Eigen::Vector3d(velocity_parent.x(), velocity_parent.y(), velocity_parent.z());
      const tf2::Vector3 angular_body =
          tf2::quatRotate(child_body.getRotation().inverse(), angular_input);
      measurement->angular_velocity_body =
          Eigen::Vector3d(angular_body.x(), angular_body.y(), angular_body.z());
    } else {
      measurement->velocity = Eigen::Vector3d(linear.x, linear.y, linear.z);
      const tf2::Vector3 angular_body =
          tf2::quatRotate(parent_body.getRotation().inverse(), angular_input);
      measurement->angular_velocity_body =
          Eigen::Vector3d(angular_body.x(), angular_body.y(), angular_body.z());
    }

    const SourceConfig& config = source->config;
    measurement->position_xy_variance = config.position_xy_variance;
    measurement->position_z_variance = config.position_z_variance;
    measurement->velocity_xy_variance = config.velocity_xy_variance;
    measurement->velocity_z_variance = config.velocity_z_variance;
    measurement->heading_variance = config.heading_variance;
    if (config.use_message_covariance) {
      measurement->position_xy_variance =
          std::max(validVariance(message.pose.covariance[0], config.position_xy_variance),
                   validVariance(message.pose.covariance[7], config.position_xy_variance));
      measurement->position_z_variance =
          validVariance(message.pose.covariance[14], config.position_z_variance);
      measurement->heading_variance =
          validVariance(message.pose.covariance[35], config.heading_variance);
      measurement->velocity_xy_variance =
          std::max(validVariance(message.twist.covariance[0], config.velocity_xy_variance),
                   validVariance(message.twist.covariance[7], config.velocity_xy_variance));
      measurement->velocity_z_variance =
          validVariance(message.twist.covariance[14], config.velocity_z_variance);
    }
    if (measurement->position_xy_variance > config.max_position_variance ||
        measurement->position_z_variance > config.max_position_variance ||
        measurement->velocity_xy_variance > config.max_velocity_variance ||
        measurement->velocity_z_variance > config.max_velocity_variance ||
        measurement->heading_variance > config.max_heading_variance) {
      *reason = "measurement covariance exceeds source reliability limits";
      return false;
    }
    return true;
  }

  void sourceCallback(SourceRuntime* source, const nav_msgs::Odometry::ConstPtr& message) {
    Measurement measurement;
    std::string reason;
    if (!convertMeasurement(source, *message, &measurement, &reason)) {
      source->adapter_rejected++;
      ROS_WARN_THROTTLE(2.0, "[xd_uav_state_estimators] source '%s' rejected: %s",
                        source->config.name.c_str(), reason.c_str());
      return;
    }
    const ros::Time now = ros::Time::now();
    source->last_received = now;
    source->latest_raw = measurement;
    source->have_raw = true;

    if (now < source->quarantine_until) {
      return;
    }
    if (!source->quarantine_until.isZero() && !source->recovering) {
      source->recovering = true;
      source->recovery_started = now;
      source->recovery_samples = 0;
      source->alignment_initialized = false;
      source->initialized = false;
    }

    initializeAlignmentIfNeeded(source, measurement);
    Measurement aligned = applyAlignment(*source, measurement);
    if (!source->initialized) {
      initializeSource(source, aligned);
      registerAccepted(source, now);
      updateSelection(now);
      publishAlignment(*source, aligned.stamp);
      return;
    }

    const double dt = (aligned.stamp - source->filter_stamp).toSec();
    if (dt < -1e-4) {
      source->adapter_rejected++;
      ROS_WARN_THROTTLE(2.0, "[xd_uav_state_estimators] source '%s' timestamp moved backwards",
                        source->config.name.c_str());
      return;
    }
    predictSource(source, std::max(0.0, dt));
    source->filter_stamp = std::max(source->filter_stamp, aligned.stamp);

    if (!measurementPassesInnovationGate(*source, aligned, &reason)) {
      registerCorrectionRejection(source, now, reason);
      updateSelection(now);
      return;
    }
    correctSource(source, aligned);
    registerAccepted(source, now);
    publishAlignment(*source, aligned.stamp);
    updateSelection(now);
  }

  void initializeAlignmentIfNeeded(SourceRuntime* source, const Measurement& measurement) {
    if (source->alignment_initialized) {
      return;
    }
    if (source->config.alignment_mode == "identity" || active_source_.empty()) {
      source->alignment_yaw = 0.0;
      source->alignment_translation.setZero();
    } else {
      Estimate target;
      if (!currentMainEstimate(measurement.stamp, &target)) {
        source->alignment_yaw = 0.0;
        source->alignment_translation.setZero();
      } else {
        source->alignment_yaw = wrapAngle(target.yaw - measurement.yaw);
        const Eigen::Rotation2Dd rotation(source->alignment_yaw);
        const Eigen::Vector2d translated =
            target.position.head<2>() - rotation * measurement.position.head<2>();
        source->alignment_translation =
            Eigen::Vector3d(translated.x(), translated.y(),
                            target.position.z() - measurement.position.z());
      }
    }
    source->alignment_initialized = true;
  }

  Measurement applyAlignment(const SourceRuntime& source,
                             const Measurement& measurement) const {
    Measurement aligned = measurement;
    const Eigen::Rotation2Dd rotation(source.alignment_yaw);
    aligned.position.head<2>() =
        rotation * measurement.position.head<2>() +
        source.alignment_translation.head<2>();
    aligned.position.z() = measurement.position.z() + source.alignment_translation.z();
    aligned.velocity.head<2>() = rotation * measurement.velocity.head<2>();
    aligned.velocity.z() = measurement.velocity.z();
    aligned.yaw = wrapAngle(measurement.yaw + source.alignment_yaw);
    double roll = 0.0;
    double pitch = 0.0;
    double unused_yaw = 0.0;
    tf2::Matrix3x3(measurement.orientation).getRPY(roll, pitch, unused_yaw);
    aligned.orientation.setRPY(roll, pitch, aligned.yaw);
    return aligned;
  }

  void initializeSource(SourceRuntime* source, const Measurement& measurement) {
    for (int axis = 0; axis < 3; ++axis) {
      source->axis[axis].initialize(measurement.position(axis), measurement.velocity(axis), 0.0);
    }
    source->yaw_filter.initialize(measurement.yaw, measurement.angular_velocity_body.z());
    source->filter_stamp = measurement.stamp;
    tf2::Matrix3x3(measurement.orientation)
        .getRPY(source->latest_roll, source->latest_pitch, unused_yaw_);
    source->initialized = true;
  }

  void predictSource(SourceRuntime* source, const double dt) {
    if (dt <= 0.0) {
      return;
    }
    for (AxisKalman& axis : source->axis) {
      axis.predict(dt);
    }
    source->yaw_filter.predict(dt);
    if (imuAccelerationFresh()) {
      for (int axis = 0; axis < 3; ++axis) {
        source->axis[axis].correct(2, latest_acceleration_odom_(axis),
                                   imu_acceleration_variance_);
      }
    }
    if (imuRateFresh()) {
      source->yaw_filter.correctRate(latest_yaw_rate_, imu_yaw_rate_variance_);
    }
  }

  bool measurementPassesInnovationGate(const SourceRuntime& source,
                                       const Measurement& measurement,
                                       std::string* reason) const {
    if (source.config.use_position_xy) {
      const double dx = source.axis[0].innovation(0, measurement.position.x());
      const double dy = source.axis[1].innovation(0, measurement.position.y());
      const double nis =
          source.axis[0].nis(0, measurement.position.x(), measurement.position_xy_variance) +
          source.axis[1].nis(0, measurement.position.y(), measurement.position_xy_variance);
      if (std::hypot(dx, dy) > position_xy_limit_ ||
          (use_nis_ && nis > position_xy_nis_limit_)) {
        *reason = "horizontal position innovation";
        return false;
      }
    }
    if (source.config.use_altitude) {
      if (std::abs(source.axis[2].innovation(0, measurement.position.z())) > position_z_limit_ ||
          (use_nis_ && source.axis[2].nis(0, measurement.position.z(),
                                          measurement.position_z_variance) >
                           position_z_nis_limit_)) {
        *reason = "altitude innovation";
        return false;
      }
    }
    if (source.config.use_velocity_xy) {
      const double dx = source.axis[0].innovation(1, measurement.velocity.x());
      const double dy = source.axis[1].innovation(1, measurement.velocity.y());
      const double nis =
          source.axis[0].nis(1, measurement.velocity.x(), measurement.velocity_xy_variance) +
          source.axis[1].nis(1, measurement.velocity.y(), measurement.velocity_xy_variance);
      if (std::hypot(dx, dy) > velocity_xy_limit_ ||
          (use_nis_ && nis > velocity_xy_nis_limit_)) {
        *reason = "horizontal velocity innovation";
        return false;
      }
    }
    if (source.config.use_vertical_velocity) {
      if (std::abs(source.axis[2].innovation(1, measurement.velocity.z())) >
              velocity_z_limit_ ||
          (use_nis_ && source.axis[2].nis(1, measurement.velocity.z(),
                                          measurement.velocity_z_variance) >
                           velocity_z_nis_limit_)) {
        *reason = "vertical velocity innovation";
        return false;
      }
    }
    if (source.config.use_heading) {
      if (std::abs(source.yaw_filter.yawInnovation(measurement.yaw)) > heading_limit_ ||
          (use_nis_ &&
           source.yaw_filter.yawNis(measurement.yaw, measurement.heading_variance) >
               heading_nis_limit_)) {
        *reason = "heading innovation";
        return false;
      }
    }
    return true;
  }

  void correctSource(SourceRuntime* source, const Measurement& measurement) {
    if (source->config.use_position_xy) {
      source->axis[0].correct(0, measurement.position.x(), measurement.position_xy_variance);
      source->axis[1].correct(0, measurement.position.y(), measurement.position_xy_variance);
    }
    if (source->config.use_altitude) {
      source->axis[2].correct(0, measurement.position.z(), measurement.position_z_variance);
    }
    if (source->config.use_velocity_xy) {
      source->axis[0].correct(1, measurement.velocity.x(), measurement.velocity_xy_variance);
      source->axis[1].correct(1, measurement.velocity.y(), measurement.velocity_xy_variance);
    }
    if (source->config.use_vertical_velocity) {
      source->axis[2].correct(1, measurement.velocity.z(), measurement.velocity_z_variance);
    }
    if (source->config.use_heading) {
      source->yaw_filter.correctYaw(measurement.yaw, measurement.heading_variance);
    }
    source->yaw_filter.correctRate(measurement.angular_velocity_body.z(),
                                   imu_yaw_rate_variance_);
    tf2::Matrix3x3(measurement.orientation)
        .getRPY(source->latest_roll, source->latest_pitch, unused_yaw_);
  }

  void registerAccepted(SourceRuntime* source, const ros::Time& now) {
    source->accepted++;
    source->last_accepted = now;
    source->consecutive_rejections = 0;
    if (source->recovering) {
      source->recovery_samples++;
      if (source->recovery_samples >= source->config.recovery_min_samples &&
          (now - source->recovery_started).toSec() >=
              source->config.recovery_stable_time) {
        source->recovering = false;
        source->quarantine_until = ros::Time();
        ROS_INFO("[xd_uav_state_estimators] source '%s' recovered",
                 source->config.name.c_str());
      }
    }
  }

  void registerCorrectionRejection(SourceRuntime* source, const ros::Time& now,
                                   const std::string& reason) {
    source->correction_rejected++;
    source->consecutive_rejections++;
    if (source->consecutive_rejections >= source->config.max_consecutive_rejections) {
      source->quarantine_until = now + ros::Duration(source->config.quarantine_duration);
      source->recovering = false;
      source->recovery_samples = 0;
      ROS_WARN("[xd_uav_state_estimators] source '%s' quarantined: %s",
               source->config.name.c_str(), reason.c_str());
    } else {
      ROS_WARN_THROTTLE(1.0, "[xd_uav_state_estimators] source '%s' correction rejected: %s",
                        source->config.name.c_str(), reason.c_str());
    }
  }

  bool sourceHealthy(const SourceRuntime& source, const ros::Time& now) const {
    return source.initialized && !source.recovering && now >= source.quarantine_until &&
           !source.last_accepted.isZero() &&
           (now - source.last_accepted).toSec() <= source.config.timeout;
  }

  SourceRuntime* chooseDesiredSource(const ros::Time& now) const {
    if (!automatic_selection_ && !requested_source_.empty()) {
      const auto iterator = source_by_name_.find(requested_source_);
      if (iterator != source_by_name_.end() && sourceHealthy(*iterator->second, now)) {
        return iterator->second;
      }
    }
    std::vector<SourceRuntime*> candidates;
    for (const auto& source : sources_) {
      if (sourceHealthy(*source, now)) {
        candidates.push_back(source.get());
      }
    }
    if (candidates.empty()) {
      return nullptr;
    }
    std::sort(candidates.begin(), candidates.end(),
              [](const SourceRuntime* left, const SourceRuntime* right) {
                const int left_role = left->config.role == "primary" ? 0 : 1;
                const int right_role = right->config.role == "primary" ? 0 : 1;
                if (left_role != right_role) {
                  return left_role < right_role;
                }
                return left->config.priority < right->config.priority;
              });
    return candidates.front();
  }

  void updateSelection(const ros::Time& now) {
    SourceRuntime* desired = chooseDesiredSource(now);
    SourceRuntime* active =
        active_source_.empty() ? nullptr : source_by_name_.at(active_source_);
    if (desired == nullptr) {
      return;
    }
    if (active == nullptr || !sourceHealthy(*active, now)) {
      activateSource(desired, now);
      return;
    }
    if (desired == active) {
      return;
    }
    if (!automatic_selection_ && desired->config.name == requested_source_) {
      activateSource(desired, now);
      return;
    }
    if (!switch_back_to_primary_) {
      return;
    }
    if ((now - active_since_).toSec() < minimum_active_time_ ||
        (now - last_switch_time_).toSec() < switch_cooldown_) {
      return;
    }
    activateSource(desired, now);
  }

  void activateSource(SourceRuntime* source, const ros::Time& now) {
    if (source == nullptr || source->config.name == active_source_) {
      return;
    }
    Estimate target;
    const bool have_target = currentMainEstimate(now, &target);
    if (have_target && source->have_raw &&
        source->config.alignment_mode == "align_on_activation") {
      source->alignment_yaw = wrapAngle(target.yaw - source->latest_raw.yaw);
      const Eigen::Rotation2Dd rotation(source->alignment_yaw);
      const Eigen::Vector2d translation =
          target.position.head<2>() - rotation * source->latest_raw.position.head<2>();
      source->alignment_translation =
          Eigen::Vector3d(translation.x(), translation.y(),
                          target.position.z() - source->latest_raw.position.z());
      source->alignment_initialized = true;
      Measurement aligned = applyAlignment(*source, source->latest_raw);
      initializeSource(source, aligned);
      publishAlignment(*source, now);
    }
    active_source_ = source->config.name;
    active_since_ = now;
    last_switch_time_ = now;
    switch_count_++;
    ROS_INFO("[xd_uav_state_estimators] active source -> %s",
             active_source_.c_str());
  }

  bool estimateSourceAt(const SourceRuntime& source, const ros::Time& stamp,
                        Estimate* estimate) const {
    if (!source.initialized) {
      return false;
    }
    std::array<AxisKalman, 3> axis = source.axis;
    YawKalman yaw = source.yaw_filter;
    const double dt = std::max(0.0, (stamp - source.filter_stamp).toSec());
    for (AxisKalman& filter : axis) {
      filter.predict(dt);
    }
    yaw.predict(dt);
    if (imuAccelerationFresh()) {
      for (int index = 0; index < 3; ++index) {
        axis[index].correct(2, latest_acceleration_odom_(index),
                            imu_acceleration_variance_);
      }
    }
    if (imuRateFresh()) {
      yaw.correctRate(latest_yaw_rate_, imu_yaw_rate_variance_);
    }
    estimate->stamp = stamp;
    for (int index = 0; index < 3; ++index) {
      estimate->position(index) = axis[index].position();
      estimate->velocity(index) = axis[index].velocity();
      estimate->acceleration(index) = axis[index].acceleration();
      estimate->position_variance[index] = axis[index].variance(0);
      estimate->velocity_variance[index] = axis[index].variance(1);
    }
    estimate->yaw = yaw.yaw();
    estimate->yaw_rate = yaw.rate();
    estimate->yaw_variance = yaw.variance(0);
    estimate->roll = source.latest_roll;
    estimate->pitch = source.latest_pitch;
    return true;
  }

  bool currentMainEstimate(const ros::Time& stamp, Estimate* estimate) const {
    if (active_source_.empty()) {
      return false;
    }
    const auto iterator = source_by_name_.find(active_source_);
    return iterator != source_by_name_.end() &&
           estimateSourceAt(*iterator->second, stamp, estimate);
  }

  nav_msgs::Odometry estimateToOdometry(const Estimate& estimate) const {
    nav_msgs::Odometry message;
    message.header.stamp = estimate.stamp;
    message.header.frame_id = odom_frame_;
    message.child_frame_id = body_frame_;
    message.pose.pose.position.x = estimate.position.x();
    message.pose.pose.position.y = estimate.position.y();
    message.pose.pose.position.z = estimate.position.z();
    tf2::Quaternion orientation;
    orientation.setRPY(estimate.roll, estimate.pitch, estimate.yaw);
    message.pose.pose.orientation = tf2::toMsg(orientation);
    message.pose.covariance[0] = estimate.position_variance[0];
    message.pose.covariance[7] = estimate.position_variance[1];
    message.pose.covariance[14] = estimate.position_variance[2];
    message.pose.covariance[21] = 0.05;
    message.pose.covariance[28] = 0.05;
    message.pose.covariance[35] = estimate.yaw_variance;

    Eigen::Vector3d velocity = estimate.velocity;
    Eigen::Vector3d acceleration = estimate.acceleration;
    if (output_twist_in_body_frame_) {
      const tf2::Vector3 parent_velocity(velocity.x(), velocity.y(), velocity.z());
      const tf2::Vector3 body_velocity =
          tf2::quatRotate(orientation.inverse(), parent_velocity);
      velocity = Eigen::Vector3d(body_velocity.x(), body_velocity.y(), body_velocity.z());
      const tf2::Vector3 parent_acceleration(acceleration.x(), acceleration.y(),
                                             acceleration.z());
      const tf2::Vector3 body_acceleration =
          tf2::quatRotate(orientation.inverse(), parent_acceleration);
      acceleration = Eigen::Vector3d(body_acceleration.x(), body_acceleration.y(),
                                     body_acceleration.z());
    }
    message.twist.twist.linear.x = velocity.x();
    message.twist.twist.linear.y = velocity.y();
    message.twist.twist.linear.z = velocity.z();
    message.twist.twist.angular.z = estimate.yaw_rate;
    message.twist.covariance[0] = estimate.velocity_variance[0];
    message.twist.covariance[7] = estimate.velocity_variance[1];
    message.twist.covariance[14] = estimate.velocity_variance[2];
    message.twist.covariance[35] = 0.1;
    return message;
  }

  void imuCallback(const sensor_msgs::Imu::ConstPtr& message) {
    const auto& acceleration = message->linear_acceleration;
    const auto& angular_velocity = message->angular_velocity;
    if (!std::isfinite(acceleration.x) || !std::isfinite(acceleration.y) ||
        !std::isfinite(acceleration.z) || !std::isfinite(angular_velocity.z)) {
      return;
    }
    Eigen::Vector3d body_acceleration(acceleration.x, acceleration.y, acceleration.z);
    if (body_acceleration.norm() > acceleration_limit_) {
      ROS_WARN_THROTTLE(2.0, "[xd_uav_state_estimators] unreasonable IMU acceleration rejected");
      return;
    }
    Estimate main;
    if (currentMainEstimate(ros::Time::now(), &main)) {
      tf2::Quaternion orientation;
      orientation.setRPY(main.roll, main.pitch, main.yaw);
      const tf2::Vector3 rotated = tf2::quatRotate(
          orientation,
          tf2::Vector3(body_acceleration.x(), body_acceleration.y(),
                       body_acceleration.z()));
      latest_acceleration_odom_ = Eigen::Vector3d(rotated.x(), rotated.y(), rotated.z());
      if (remove_gravity_) {
        latest_acceleration_odom_.z() -= gravity_;
      }
      have_acceleration_ = true;
    }
    latest_yaw_rate_ = angular_velocity.z;
    last_imu_receive_ = ros::Time::now();
    have_imu_rate_ = true;
  }

  bool imuAccelerationFresh() const {
    return have_acceleration_ &&
           (ros::Time::now() - last_imu_receive_).toSec() <= imu_timeout_;
  }

  bool imuRateFresh() const {
    return have_imu_rate_ &&
           (ros::Time::now() - last_imu_receive_).toSec() <= imu_timeout_;
  }

  void outputTimerCallback(const ros::TimerEvent&) {
    const ros::Time now = ros::Time::now();
    updateSelection(now);
    for (const auto& source : sources_) {
      const bool valid = sourceHealthy(*source, now);
      publishBoolean(source->valid_publisher, valid);
      Estimate estimate;
      if (estimateSourceAt(*source, now, &estimate)) {
        source->odometry_publisher.publish(estimateToOdometry(estimate));
      }
    }

    const bool localization_valid =
        !active_source_.empty() && sourceHealthy(*source_by_name_.at(active_source_), now);
    bool state_valid = false;
    Estimate main;
    if (currentMainEstimate(now, &main)) {
      const SourceRuntime* active = source_by_name_.at(active_source_);
      const double dead_reckoning_age =
          active->last_accepted.isZero()
              ? std::numeric_limits<double>::infinity()
              : (now - active->last_accepted).toSec();
      state_valid = localization_valid || dead_reckoning_age <= max_dead_reckoning_time_;
      if (require_imu_ && !imuRateFresh()) {
        state_valid = false;
      }
      if (state_valid) {
        const nav_msgs::Odometry main_odometry = estimateToOdometry(main);
        main_odometry_publisher_.publish(main_odometry);
        latest_main_odometry_ = main_odometry;
        have_main_odometry_ = true;
        publishFrameOutputs(now);
      }
    }
    publishBoolean(localization_valid_publisher_, localization_valid);
    publishBoolean(state_valid_publisher_, state_valid);
    publishStatus(now, localization_valid, state_valid);
  }

  void publishFrameOutputs(const ros::Time& stamp) {
    if (!have_main_odometry_) {
      return;
    }
    for (FrameOutput& output : frame_outputs_) {
      try {
        const geometry_msgs::TransformStamped transform = tf_buffer_.lookupTransform(
            output.target_frame, body_frame_, ros::Time(0),
            ros::Duration(output.lookup_timeout));
        nav_msgs::Odometry message;
        message.header.stamp = stamp;
        message.header.frame_id = output.target_frame;
        message.child_frame_id = body_frame_;
        message.pose.pose.position.x = transform.transform.translation.x;
        message.pose.pose.position.y = transform.transform.translation.y;
        message.pose.pose.position.z = transform.transform.translation.z;
        message.pose.pose.orientation = transform.transform.rotation;
        message.pose.covariance = latest_main_odometry_.pose.covariance;
        message.twist = latest_main_odometry_.twist;
        output.publisher.publish(message);
      } catch (const tf2::TransformException& exception) {
        ROS_WARN_THROTTLE(2.0,
                          "[xd_uav_state_estimators] frame output '%s' unavailable: %s",
                          output.name.c_str(), exception.what());
      }
    }
  }

  void publishAlignment(const SourceRuntime& source, const ros::Time& stamp) {
    if (!source.alignment_initialized || source.latest_raw.parent_frame.empty()) {
      return;
    }
    geometry_msgs::TransformStamped message;
    message.header.stamp = stamp;
    message.header.frame_id = odom_frame_;
    message.child_frame_id = trimSlashes(source.latest_raw.parent_frame);
    message.transform.translation.x = source.alignment_translation.x();
    message.transform.translation.y = source.alignment_translation.y();
    message.transform.translation.z = source.alignment_translation.z();
    tf2::Quaternion rotation;
    rotation.setRPY(0.0, 0.0, source.alignment_yaw);
    message.transform.rotation = tf2::toMsg(rotation);
    source.alignment_publisher.publish(message);
  }

  void publishStatus(const ros::Time& now, const bool localization_valid,
                     const bool state_valid) {
    xd_uav_state_estimators::EstimatorStatus message;
    message.header.stamp = now;
    message.header.frame_id = odom_frame_;
    message.state_valid = state_valid;
    message.localization_valid = localization_valid;
    message.active_source = active_source_;
    message.requested_source = requested_source_;
    message.automatic_selection = automatic_selection_;
    message.source_switches = switch_count_;
    if (active_source_.empty()) {
      message.localization_age = -1.0;
      message.dead_reckoning_age = -1.0;
      message.state = xd_uav_state_estimators::EstimatorStatus::WAITING;
      message.state_name = "WAITING";
      message.failure_reason = "no healthy localization source";
    } else {
      const SourceRuntime* active = source_by_name_.at(active_source_);
      const double age = active->last_accepted.isZero()
                             ? -1.0
                             : (now - active->last_accepted).toSec();
      message.localization_age = age;
      message.dead_reckoning_age = localization_valid ? 0.0 : age;
      if (localization_valid && state_valid) {
        message.state = xd_uav_state_estimators::EstimatorStatus::RUNNING;
        message.state_name = "RUNNING";
      } else if (state_valid) {
        message.state = xd_uav_state_estimators::EstimatorStatus::DEAD_RECKONING;
        message.state_name = "DEAD_RECKONING";
        message.failure_reason = "active source is stale; IMU/model prediction only";
      } else {
        message.state = xd_uav_state_estimators::EstimatorStatus::LOST;
        message.state_name = "LOST";
        message.failure_reason = "no state safe for control";
      }
    }
    status_publisher_.publish(message);
  }

  void diagnosticsTimerCallback(const ros::TimerEvent&) {
    const ros::Time now = ros::Time::now();
    diagnostic_msgs::DiagnosticArray array;
    array.header.stamp = now;
    for (const auto& source : sources_) {
      diagnostic_msgs::DiagnosticStatus status;
      status.name = uav_name_ + "/state_estimator/source/" + source->config.name;
      status.hardware_id = source->config.name;
      const bool healthy = sourceHealthy(*source, now);
      status.level = healthy ? diagnostic_msgs::DiagnosticStatus::OK
                             : diagnostic_msgs::DiagnosticStatus::WARN;
      status.message =
          now < source->quarantine_until
              ? "quarantined"
              : (source->recovering ? "recovering" : (healthy ? "healthy" : "stale/waiting"));
      addDiagnostic(&status, "active",
                    source->config.name == active_source_ ? "true" : "false");
      addDiagnostic(&status, "topic", nh_.resolveName(source->config.topic));
      addDiagnostic(&status, "received", std::to_string(source->received));
      addDiagnostic(&status, "accepted", std::to_string(source->accepted));
      addDiagnostic(&status, "adapter_rejected",
                    std::to_string(source->adapter_rejected));
      addDiagnostic(&status, "correction_rejected",
                    std::to_string(source->correction_rejected));
      addDiagnostic(&status, "age",
                    source->last_accepted.isZero()
                        ? "-1"
                        : std::to_string((now - source->last_accepted).toSec()));
      array.status.push_back(status);
    }
    diagnostics_publisher_.publish(array);
  }

  static void addDiagnostic(diagnostic_msgs::DiagnosticStatus* status,
                            const std::string& key, const std::string& value) {
    diagnostic_msgs::KeyValue entry;
    entry.key = key;
    entry.value = value;
    status->values.push_back(entry);
  }

  static void publishBoolean(const ros::Publisher& publisher, const bool value) {
    std_msgs::Bool message;
    message.data = value;
    publisher.publish(message);
  }

  bool resetCallback(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& response) {
    for (auto& source : sources_) {
      source->initialized = false;
      source->alignment_initialized = false;
      source->consecutive_rejections = 0;
      source->quarantine_until = ros::Time();
      source->recovering = false;
      source->recovery_samples = 0;
    }
    active_source_.clear();
    have_main_odometry_ = false;
    response.success = true;
    response.message = "all source estimators reset";
    return true;
  }

  bool switchSourceCallback(
      xd_uav_state_estimators::SwitchLocalizationSource::Request& request,
      xd_uav_state_estimators::SwitchLocalizationSource::Response& response) {
    const std::string requested = request.source_name;
    if (requested.empty() || requested == "auto") {
      automatic_selection_ = true;
      requested_source_.clear();
      updateSelection(ros::Time::now());
      response.success = true;
      response.message = "automatic source selection enabled";
    } else {
      const auto iterator = source_by_name_.find(requested);
      if (iterator == source_by_name_.end()) {
        response.success = false;
        response.message = "unknown or disabled source: " + requested;
      } else if (!sourceHealthy(*iterator->second, ros::Time::now())) {
        response.success = false;
        response.message = "source is not healthy: " + requested;
      } else {
        automatic_selection_ = false;
        requested_source_ = requested;
        activateSource(iterator->second, ros::Time::now());
        response.success = true;
        response.message = "source selected: " + requested;
      }
    }
    response.active_source = active_source_;
    response.requested_source = requested_source_;
    response.automatic = automatic_selection_;
    return true;
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  ros::Subscriber imu_subscriber_;
  ros::Publisher main_odometry_publisher_;
  ros::Publisher localization_valid_publisher_;
  ros::Publisher state_valid_publisher_;
  ros::Publisher status_publisher_;
  ros::Publisher diagnostics_publisher_;
  ros::ServiceServer reset_server_;
  ros::ServiceServer switch_server_;
  ros::Timer output_timer_;
  ros::Timer diagnostics_timer_;

  std::vector<std::unique_ptr<SourceRuntime>> sources_;
  std::unordered_map<std::string, SourceRuntime*> source_by_name_;
  std::vector<FrameOutput> frame_outputs_;
  AxisKalman::Parameters axis_parameters_;
  std::string uav_name_;
  std::string odom_frame_;
  std::string body_frame_;
  std::string active_source_;
  std::string requested_source_;
  ros::Time active_since_;
  ros::Time last_switch_time_;
  ros::Time last_imu_receive_;
  nav_msgs::Odometry latest_main_odometry_;
  Eigen::Vector3d latest_acceleration_odom_{Eigen::Vector3d::Zero()};
  double latest_yaw_rate_{0.0};
  double unused_yaw_{0.0};
  double output_rate_{100.0};
  double diagnostics_rate_{2.0};
  double max_localization_delay_{0.15};
  double max_dead_reckoning_time_{1.0};
  double imu_timeout_{0.3};
  double yaw_process_noise_{0.02};
  double yaw_rate_process_noise_{0.2};
  double imu_acceleration_variance_{0.5};
  double imu_yaw_rate_variance_{0.05};
  double gravity_{9.80665};
  double acceleration_limit_{30.0};
  double position_xy_limit_{5.0};
  double position_z_limit_{3.0};
  double velocity_xy_limit_{5.0};
  double velocity_z_limit_{3.0};
  double heading_limit_{1.57};
  double position_xy_nis_limit_{9.21};
  double position_z_nis_limit_{6.63};
  double velocity_xy_nis_limit_{9.21};
  double velocity_z_nis_limit_{6.63};
  double heading_nis_limit_{6.63};
  double minimum_active_time_{1.0};
  double switch_cooldown_{0.5};
  bool require_imu_{false};
  bool remove_gravity_{true};
  bool output_twist_in_body_frame_{true};
  bool use_nis_{true};
  bool switch_back_to_primary_{true};
  bool automatic_selection_{true};
  bool have_acceleration_{false};
  bool have_imu_rate_{false};
  bool have_main_odometry_{false};
  std::uint64_t switch_count_{0};
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "state_estimator");
  try {
    MultiSourceEstimatorNode node;
    ros::spin();
  } catch (const std::exception& exception) {
    ROS_FATAL("[xd_uav_state_estimators] startup failed: %s", exception.what());
    return 1;
  }
  return 0;
}
