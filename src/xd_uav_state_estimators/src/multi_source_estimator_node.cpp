#include <algorithm>
#include <array>
#include <clocale>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <XmlRpcValue.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <geometry_msgs/AccelWithCovarianceStamped.h>
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
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Dense>

#include <xd_uav_state_estimators/EstimatorStatus.h>
#include <xd_uav_state_estimators/Heading.h>
#include <xd_uav_state_estimators/PositionXY.h>
#include <xd_uav_state_estimators/PositionZ.h>
#include <xd_uav_state_estimators/SwitchLocalizationSource.h>
#include <xd_uav_state_estimators/VelocityXY.h>
#include <xd_uav_state_estimators/VelocityZ.h>
#include <xd_uav_state_estimators/YawRate.h>

namespace {

constexpr double kPi = 3.14159265358979323846;

double wrapAngle(const double value) {
  return std::atan2(std::sin(value), std::cos(value));
}

bool eulerYawRateFromBodyRates(const double roll, const double pitch,
                               const Eigen::Vector3d& body_rate,
                               double* yaw_rate) {
  const double cosine_pitch = std::cos(pitch);
  if (!std::isfinite(roll) || !std::isfinite(pitch) ||
      !body_rate.allFinite() || std::abs(cosine_pitch) < 1e-3) {
    return false;
  }
  const double value =
      (std::sin(roll) * body_rate.y() +
       std::cos(roll) * body_rate.z()) /
      cosine_pitch;
  if (!std::isfinite(value)) {
    return false;
  }
  *yaw_rate = value;
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

bool normalizeQuaternion(const geometry_msgs::Quaternion& message,
                         tf2::Quaternion* quaternion) {
  if (!std::isfinite(message.x) || !std::isfinite(message.y) ||
      !std::isfinite(message.z) || !std::isfinite(message.w)) {
    return false;
  }
  *quaternion =
      tf2::Quaternion(message.x, message.y, message.z, message.w);
  if (quaternion->length2() < 1e-9) {
    return false;
  }
  quaternion->normalize();
  return true;
}

std::string xmlString(const XmlRpc::XmlRpcValue& value,
                      const std::string& key,
                      const std::string& fallback = "") {
  if (value.getType() != XmlRpc::XmlRpcValue::TypeStruct ||
      !value.hasMember(key) ||
      value[key].getType() != XmlRpc::XmlRpcValue::TypeString) {
    return fallback;
  }
  return static_cast<std::string>(value[key]);
}

bool xmlBool(const XmlRpc::XmlRpcValue& value, const std::string& key,
             const bool fallback) {
  if (value.getType() != XmlRpc::XmlRpcValue::TypeStruct ||
      !value.hasMember(key) ||
      value[key].getType() != XmlRpc::XmlRpcValue::TypeBoolean) {
    return fallback;
  }
  return static_cast<bool>(value[key]);
}

double xmlDouble(const XmlRpc::XmlRpcValue& value, const std::string& key,
                 const double fallback) {
  if (value.getType() != XmlRpc::XmlRpcValue::TypeStruct ||
      !value.hasMember(key)) {
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

std::string frameTopicName(const std::string& frame) {
  const std::string trimmed = trimSlashes(frame);
  const std::size_t separator = trimmed.find_last_of('/');
  return separator == std::string::npos ? trimmed
                                        : trimmed.substr(separator + 1);
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

  void initialize(const double position, const double velocity,
                  const double acceleration) {
    initialize(position, velocity, acceleration,
               parameters_.initial_position_variance,
               parameters_.initial_velocity_variance,
               parameters_.initial_acceleration_variance);
  }

  void initialize(const double position, const double velocity,
                  const double acceleration,
                  const double position_variance,
                  const double velocity_variance,
                  const double acceleration_variance) {
    state_ << position, velocity, acceleration;
    covariance_.setZero();
    covariance_(0, 0) =
        std::max(1e-9, position_variance);
    covariance_(1, 1) =
        std::max(1e-9, velocity_variance);
    covariance_(2, 2) =
        std::max(1e-9, acceleration_variance);
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
    covariance_ =
        transition * covariance_ * transition.transpose() + process_noise;
  }

  double innovation(const int index, const double measurement) const {
    return measurement - state_(index);
  }

  double nis(const int index, const double measurement,
             const double measurement_variance) const {
    const double variance =
        covariance_(index, index) + measurement_variance;
    if (!std::isfinite(variance) || variance <= 1e-12) {
      return std::numeric_limits<double>::infinity();
    }
    const double residual = innovation(index, measurement);
    return residual * residual / variance;
  }

  void correct(const int index, const double measurement,
               const double measurement_variance) {
    if (!initialized_) {
      return;
    }
    Eigen::RowVector3d observation = Eigen::RowVector3d::Zero();
    observation(index) = 1.0;
    const double residual = measurement - (observation * state_)(0);
    const double innovation_variance =
        (observation * covariance_ * observation.transpose())(0, 0) +
        measurement_variance;
    if (!std::isfinite(innovation_variance) ||
        innovation_variance <= 1e-12) {
      return;
    }
    const Eigen::Vector3d gain =
        covariance_ * observation.transpose() / innovation_variance;
    state_ += gain * residual;
    covariance_ =
        (Eigen::Matrix3d::Identity() - gain * observation) * covariance_;
    covariance_ = 0.5 * (covariance_ + covariance_.transpose());
  }

  double position() const { return state_(0); }
  double velocity() const { return state_(1); }
  double acceleration() const { return state_(2); }
  double variance(const int index) const {
    return covariance_(index, index);
  }

 private:
  Parameters parameters_;
  Eigen::Vector3d state_{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d covariance_{Eigen::Matrix3d::Identity()};
  bool initialized_{false};
};

class YawKalman {
 public:
  void configure(const double yaw_process_noise,
                 const double rate_process_noise) {
    yaw_process_noise_ = yaw_process_noise;
    rate_process_noise_ = rate_process_noise;
  }

  void initialize(const double yaw, const double rate) {
    initialize(yaw, rate, 0.25, 0.25);
  }

  void initialize(const double yaw, const double rate,
                  const double yaw_variance,
                  const double rate_variance) {
    state_ << wrapAngle(yaw), rate;
    covariance_.setZero();
    covariance_(0, 0) = std::max(1e-9, yaw_variance);
    covariance_(1, 1) = std::max(1e-9, rate_variance);
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
    covariance_ =
        transition * covariance_ * transition.transpose() + process_noise;
  }

  double yawInnovation(const double yaw) const {
    return wrapAngle(yaw - state_(0));
  }

  double yawNis(const double yaw, const double variance) const {
    const double innovation_variance = covariance_(0, 0) + variance;
    if (!std::isfinite(innovation_variance) ||
        innovation_variance <= 1e-12) {
      return std::numeric_limits<double>::infinity();
    }
    const double residual = yawInnovation(yaw);
    return residual * residual / innovation_variance;
  }

  double rateInnovation(const double rate) const {
    return rate - state_(1);
  }

  double rateNis(const double rate, const double variance) const {
    const double innovation_variance = covariance_(1, 1) + variance;
    if (!std::isfinite(innovation_variance) ||
        innovation_variance <= 1e-12) {
      return std::numeric_limits<double>::infinity();
    }
    const double residual = rateInnovation(rate);
    return residual * residual / innovation_variance;
  }

  void correctYaw(const double yaw, const double variance) {
    correct(0, yawInnovation(yaw), variance);
    state_(0) = wrapAngle(state_(0));
  }

  void correctRate(const double rate, const double variance) {
    correct(1, rateInnovation(rate), variance);
  }

  double yaw() const { return state_(0); }
  double rate() const { return state_(1); }
  double variance(const int index) const {
    return covariance_(index, index);
  }

 private:
  void correct(const int index, const double residual,
               const double variance) {
    Eigen::RowVector2d observation = Eigen::RowVector2d::Zero();
    observation(index) = 1.0;
    const double innovation_variance =
        (observation * covariance_ * observation.transpose())(0, 0) +
        variance;
    if (!std::isfinite(innovation_variance) ||
        innovation_variance <= 1e-12) {
      return;
    }
    const Eigen::Vector2d gain =
        covariance_ * observation.transpose() / innovation_variance;
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

enum class CorrectionKind {
  kPositionXY,
  kPositionZ,
  kVelocityXY,
  kVelocityZ,
  kHeading,
  kYawRate,
};

std::string correctionKindName(const CorrectionKind kind) {
  switch (kind) {
    case CorrectionKind::kPositionXY:
      return "position_xy";
    case CorrectionKind::kPositionZ:
      return "position_z";
    case CorrectionKind::kVelocityXY:
      return "velocity_xy";
    case CorrectionKind::kVelocityZ:
      return "velocity_z";
    case CorrectionKind::kHeading:
      return "heading";
    case CorrectionKind::kYawRate:
      return "yaw_rate";
  }
  return "unknown";
}

CorrectionKind parseCorrectionKind(const std::string& name) {
  if (name == "position_xy") {
    return CorrectionKind::kPositionXY;
  }
  if (name == "position_z") {
    return CorrectionKind::kPositionZ;
  }
  if (name == "velocity_xy") {
    return CorrectionKind::kVelocityXY;
  }
  if (name == "velocity_z") {
    return CorrectionKind::kVelocityZ;
  }
  if (name == "heading") {
    return CorrectionKind::kHeading;
  }
  if (name == "yaw_rate") {
    return CorrectionKind::kYawRate;
  }
  throw std::runtime_error("不支持的修正类型: " + name);
}

bool isPoseCorrection(const CorrectionKind kind) {
  return kind == CorrectionKind::kPositionXY ||
         kind == CorrectionKind::kPositionZ ||
         kind == CorrectionKind::kHeading;
}

struct CorrectionConfig {
  std::string name;
  std::string topic;
  // 由state_estimator_inputs/<provider>/...的话题结构自动识别。
  // provider只决定坐标标准化使用哪套来源原点对齐，不限制该修正最终
  // 参与哪个逻辑来源的滤波。
  std::string provider;
  CorrectionKind kind{CorrectionKind::kPositionXY};
  double timeout{0.25};
  bool required{true};
};

struct CorrectionRuntime {
  CorrectionConfig config;
  ros::Subscriber subscriber;
  ros::Time last_received;
  ros::Time last_accepted;
  ros::Time quarantine_until;
  ros::Time recovery_started;
  std::uint64_t received{0};
  std::uint64_t accepted{0};
  std::uint64_t rejected{0};
  std::uint64_t valid_session{0};
  int consecutive_rejections{0};
  int recovery_samples{0};
  bool recovering{false};
};

struct SourceConfig {
  std::string name;
  std::string alignment_mode{"align_on_activation"};
  int priority{0};
  double max_position_variance{100.0};
  double max_velocity_variance{100.0};
  double max_heading_variance{10.0};
  double max_yaw_rate_variance{10.0};
  int max_consecutive_rejections{5};
  double quarantine_duration{1.0};
  int recovery_min_samples{20};
  double recovery_stable_time{0.5};
  std::vector<std::string> republish_frames;
};

struct RawState {
  Eigen::Vector2d position_xy{Eigen::Vector2d::Zero()};
  Eigen::Vector2d velocity_xy{Eigen::Vector2d::Zero()};
  double position_z{0.0};
  double velocity_z{0.0};
  double heading{0.0};
  double yaw_rate{0.0};
  bool have_position_xy{false};
  bool have_position_z{false};
  bool have_velocity_xy{false};
  bool have_velocity_z{false};
  bool have_heading{false};
  bool have_yaw_rate{false};
};

struct Estimate {
  ros::Time stamp;
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
  Eigen::Vector3d acceleration{Eigen::Vector3d::Zero()};
  std::array<double, 3> position_variance{{1.0, 1.0, 1.0}};
  std::array<double, 3> velocity_variance{{1.0, 1.0, 1.0}};
  std::array<double, 3> acceleration_variance{{1.0, 1.0, 1.0}};
  double yaw{0.0};
  double yaw_rate{0.0};
  double yaw_variance{0.25};
  double yaw_rate_variance{0.25};
  double roll{0.0};
  double pitch{0.0};
};

struct FramePublisher {
  std::string target_frame;
  ros::Publisher publisher;
};

struct SourceRuntime {
  SourceRuntime(SourceConfig source_config,
                const AxisKalman::Parameters& axis_parameters,
                const double yaw_process_noise,
                const double yaw_rate_process_noise)
      : config(std::move(source_config)),
        axis{{AxisKalman(axis_parameters), AxisKalman(axis_parameters),
              AxisKalman(axis_parameters)}} {
    yaw_filter.configure(yaw_process_noise, yaw_rate_process_noise);
  }

  SourceConfig config;
  std::array<AxisKalman, 3> axis;
  YawKalman yaw_filter;
  std::vector<std::unique_ptr<CorrectionRuntime>> corrections;
  ros::Publisher odometry_publisher;
  ros::Publisher valid_publisher;
  ros::Publisher alignment_publisher;
  std::vector<FramePublisher> frame_publishers;
  // raw保存本定位提供者在自身原点下的原始状态，只用于建立/发布
  // provider-origin -> odom对齐；fused_raw保存已经标准化到odom的
  // 本来源输入，用于初始化该来源自己的完整滤波状态。
  RawState raw;
  RawState fused_raw;
  std::string raw_frame;
  ros::Time filter_stamp;
  ros::Time latest_raw_stamp;
  Eigen::Vector3d alignment_translation{Eigen::Vector3d::Zero()};
  double alignment_yaw{0.0};
  bool alignment_initialized{false};
  bool filters_initialized{false};
  bool required_corrections_were_connected{false};
  std::uint64_t session{1};
};

struct MainRuntime {
  MainRuntime(const AxisKalman::Parameters& axis_parameters,
              const double yaw_process_noise,
              const double yaw_rate_process_noise)
      : axis{{AxisKalman(axis_parameters), AxisKalman(axis_parameters),
              AxisKalman(axis_parameters)}} {
    yaw_filter.configure(yaw_process_noise, yaw_rate_process_noise);
  }

  std::array<AxisKalman, 3> axis;
  YawKalman yaw_filter;
  ros::Time filter_stamp;
  bool initialized{false};
};

struct CorrectionSample {
  CorrectionKind kind{CorrectionKind::kPositionXY};
  ros::Time stamp;
  std::string frame_id;
  std::string child_frame_id;
  Eigen::Vector2d vector{Eigen::Vector2d::Zero()};
  double scalar{0.0};
  double variance_a{0.01};
  double variance_b{0.01};
  bool valid{false};
};

}  // 匿名命名空间

class MultiSourceEstimatorNode {
 public:
  MultiSourceEstimatorNode()
      : private_nh_("~"), tf_listener_(tf_buffer_) {
    loadParameters();
    main_state_ = std::make_unique<MainRuntime>(
        axis_parameters_, yaw_process_noise_, yaw_rate_process_noise_);
    loadSources();

    imu_subscriber_ =
        nh_.subscribe("imu", 100, &MultiSourceEstimatorNode::imuCallback, this);
    main_odometry_publisher_ =
        private_nh_.advertise<nav_msgs::Odometry>("main/odom", 10);
    main_acceleration_publisher_ =
        private_nh_.advertise<geometry_msgs::AccelWithCovarianceStamped>(
            "main/acceleration", 10);
    localization_valid_publisher_ =
        private_nh_.advertise<std_msgs::Bool>("localization_valid", 1, true);
    state_valid_publisher_ =
        private_nh_.advertise<std_msgs::Bool>("state_valid", 1, true);
    status_publisher_ =
        private_nh_.advertise<xd_uav_state_estimators::EstimatorStatus>(
            "status", 5);
    diagnostics_publisher_ =
        private_nh_.advertise<diagnostic_msgs::DiagnosticArray>(
            "diagnostics", 5);
    reset_server_ = private_nh_.advertiseService(
        "reset", &MultiSourceEstimatorNode::resetCallback, this);
    switch_server_ = private_nh_.advertiseService(
        "switch_source", &MultiSourceEstimatorNode::switchSourceCallback,
        this);

    createMainFramePublishers();
    output_timer_ = private_nh_.createTimer(
        ros::Duration(1.0 / std::max(1.0, output_rate_)),
        &MultiSourceEstimatorNode::outputTimerCallback, this);
    diagnostics_timer_ = private_nh_.createTimer(
        ros::Duration(1.0 / std::max(0.2, diagnostics_rate_)),
        &MultiSourceEstimatorNode::diagnosticsTimerCallback, this);
    publishBoolean(localization_valid_publisher_, false);
    publishBoolean(state_valid_publisher_, false);

    ROS_INFO(
        "[xd_uav_state_estimators] %s已加载%zu个多修正定位源，并独占发布主输出TF",
        uav_name_.c_str(), sources_.size());
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

  void loadParameters() {
    private_nh_.param("uav_name", uav_name_, std::string("uav1"));
    uav_name_ = trimSlashes(uav_name_);
    private_nh_.param("odom_frame", odom_frame_, uav_name_ + "/odom");
    private_nh_.param("body_frame", body_frame_, uav_name_ + "/base_link");
    odom_frame_ = trimSlashes(odom_frame_);
    body_frame_ = trimSlashes(body_frame_);
    private_nh_.param("output_rate", output_rate_, 100.0);
    private_nh_.param("diagnostics_rate", diagnostics_rate_, 2.0);
    private_nh_.param("max_localization_delay", max_localization_delay_,
                      0.15);
    private_nh_.param("max_dead_reckoning_time", max_dead_reckoning_time_,
                      1.0);
    private_nh_.param("imu_timeout", imu_timeout_, 0.3);
    private_nh_.param("require_imu", require_imu_, false);
    private_nh_.param("frame_lookup_timeout", frame_lookup_timeout_, 0.03);

    private_nh_.param("filter/initial_position_variance",
                      axis_parameters_.initial_position_variance, 1.0);
    private_nh_.param("filter/initial_velocity_variance",
                      axis_parameters_.initial_velocity_variance, 1.0);
    private_nh_.param("filter/initial_acceleration_variance",
                      axis_parameters_.initial_acceleration_variance, 10.0);
    private_nh_.param("filter/position_process_noise",
                      axis_parameters_.position_process_noise, 0.05);
    private_nh_.param("filter/velocity_process_noise",
                      axis_parameters_.velocity_process_noise, 0.50);
    private_nh_.param("filter/acceleration_process_noise",
                      axis_parameters_.acceleration_process_noise, 2.0);
    private_nh_.param("filter/yaw_process_noise", yaw_process_noise_, 0.02);
    private_nh_.param("filter/yaw_rate_process_noise",
                      yaw_rate_process_noise_, 0.20);
    private_nh_.param("filter/imu_acceleration_variance",
                      imu_acceleration_variance_, 0.50);
    private_nh_.param("filter/imu_yaw_rate_variance",
                      imu_yaw_rate_variance_, 0.05);
    private_nh_.param("filter/angular_acceleration_time_constant",
                      angular_acceleration_time_constant_, 0.05);
    private_nh_.param("filter/angular_acceleration_variance",
                      angular_acceleration_variance_, 1.0);
    private_nh_.param("filter/angular_acceleration_limit",
                      angular_acceleration_limit_, 100.0);
    private_nh_.param("filter/remove_gravity", remove_gravity_, true);
    private_nh_.param("filter/gravity", gravity_, 9.80665);
    private_nh_.param("filter/acceleration_limit", acceleration_limit_, 30.0);

    private_nh_.param("innovation_gate/position_xy", position_xy_limit_,
                      5.0);
    private_nh_.param("innovation_gate/position_z", position_z_limit_, 3.0);
    private_nh_.param("innovation_gate/velocity_xy", velocity_xy_limit_,
                      5.0);
    private_nh_.param("innovation_gate/velocity_z", velocity_z_limit_, 3.0);
    private_nh_.param("innovation_gate/heading", heading_limit_, 1.57);
    private_nh_.param("innovation_gate/yaw_rate", yaw_rate_limit_, 3.0);
    private_nh_.param("innovation_gate/use_nis", use_nis_, true);
    private_nh_.param("innovation_gate/position_xy_nis",
                      position_xy_nis_limit_, 9.21);
    private_nh_.param("innovation_gate/position_z_nis",
                      position_z_nis_limit_, 6.63);
    private_nh_.param("innovation_gate/velocity_xy_nis",
                      velocity_xy_nis_limit_, 9.21);
    private_nh_.param("innovation_gate/velocity_z_nis",
                      velocity_z_nis_limit_, 6.63);
    private_nh_.param("innovation_gate/heading_nis",
                      heading_nis_limit_, 6.63);
    private_nh_.param("innovation_gate/yaw_rate_nis",
                      yaw_rate_nis_limit_, 6.63);

    private_nh_.param(
        "source_switching/switch_back_to_higher_priority",
        switch_back_to_higher_priority_, true);
    private_nh_.param("source_switching/minimum_active_time",
                      minimum_active_time_, 1.0);
    private_nh_.param("source_switching/cooldown", switch_cooldown_, 0.5);
    private_nh_.getParam("main_republish_in_frames",
                         main_republish_frames_);
  }

  std::string correctionTopic(const std::string& configured) const {
    if (configured.empty()) {
      return configured;
    }
    if (configured.front() == '/') {
      return configured;
    }
    const std::string trimmed = trimSlashes(configured);
    if (trimmed.rfind("state_estimator_inputs/", 0) == 0) {
      return trimmed;
    }
    return "state_estimator_inputs/" + trimmed;
  }

  void loadSources() {
    std::vector<std::string> names;
    if (!private_nh_.getParam("localization_sources", names) ||
        names.empty()) {
      throw std::runtime_error(
          "localization_sources至少需要包含一个来源");
    }
    for (const std::string& name : names) {
      if (source_by_name_.count(name) != 0) {
        throw std::runtime_error("定位源名称重复: " + name);
      }
      SourceConfig config;
      config.name = name;
      const std::string prefix = "sources/" + name + "/";
      private_nh_.param(prefix + "priority", config.priority, 0);
      private_nh_.param(prefix + "alignment/mode", config.alignment_mode,
                        std::string("align_on_activation"));
      if (config.alignment_mode != "align_on_activation" &&
          config.alignment_mode != "identity") {
        throw std::runtime_error(
            "来源'" + name + "'的alignment.mode无效");
      }
      private_nh_.param(prefix + "reliability/max_position_variance",
                        config.max_position_variance, 100.0);
      private_nh_.param(prefix + "reliability/max_velocity_variance",
                        config.max_velocity_variance, 100.0);
      private_nh_.param(prefix + "reliability/max_heading_variance",
                        config.max_heading_variance, 10.0);
      private_nh_.param(prefix + "reliability/max_yaw_rate_variance",
                        config.max_yaw_rate_variance, 10.0);
      private_nh_.param(prefix + "reliability/max_consecutive_rejections",
                        config.max_consecutive_rejections, 5);
      private_nh_.param(prefix + "reliability/quarantine_duration",
                        config.quarantine_duration, 1.0);
      private_nh_.param(prefix + "reliability/recovery_min_samples",
                        config.recovery_min_samples, 20);
      private_nh_.param(prefix + "reliability/recovery_stable_time",
                        config.recovery_stable_time, 0.5);
      private_nh_.getParam(prefix + "republish_in_frames",
                           config.republish_frames);

      auto source = std::make_unique<SourceRuntime>(
          config, axis_parameters_, yaw_process_noise_,
          yaw_rate_process_noise_);
      SourceRuntime* source_pointer = source.get();
      source->odometry_publisher =
          private_nh_.advertise<nav_msgs::Odometry>(
              "sources/" + name + "/odom", 10);
      source->valid_publisher =
          private_nh_.advertise<std_msgs::Bool>(
              "sources/" + name + "/valid", 1, true);
      source->alignment_publisher =
          private_nh_.advertise<geometry_msgs::TransformStamped>(
              "sources/" + name + "/alignment", 2, true);
      publishBoolean(source->valid_publisher, false);
      source_by_name_[name] = source_pointer;
      sources_.push_back(std::move(source));
    }

    // 先建立完整的来源表，再解析修正。这样任意来源都可以直接引用
    // 另一个已声明提供者的话题，不受YAML书写顺序影响。
    for (auto& source : sources_) {
      const std::string prefix = "sources/" + source->config.name + "/";
      loadCorrections(source.get(), prefix);
      createSourceFramePublishers(source.get());
    }
  }

  void loadCorrections(SourceRuntime* source, const std::string& prefix) {
    XmlRpc::XmlRpcValue rules;
    if (!private_nh_.getParam(prefix + "corrections", rules) ||
        rules.getType() != XmlRpc::XmlRpcValue::TypeArray ||
        rules.size() == 0) {
      throw std::runtime_error(
          "来源'" + source->config.name + "'必须配置corrections列表");
    }
    double default_timeout = 0.25;
    bool default_required = true;
    private_nh_.param(prefix + "correction_defaults/timeout",
                      default_timeout, 0.25);
    private_nh_.param(prefix + "correction_defaults/required",
                      default_required, true);
    std::unordered_map<std::string, int> kind_counts;

    for (int index = 0; index < rules.size(); ++index) {
      XmlRpc::XmlRpcValue& rule = rules[index];
      if (rule.getType() != XmlRpc::XmlRpcValue::TypeStruct ||
          rule.size() != 1) {
        throw std::runtime_error(
            "每条correction必须是只包含一个修正类型的映射");
      }
      auto iterator = rule.begin();
      const std::string kind_text = iterator->first;
      XmlRpc::XmlRpcValue& specification = iterator->second;
      CorrectionConfig config;
      config.kind = parseCorrectionKind(kind_text);
      config.timeout = default_timeout;
      config.required = default_required;
      if (specification.getType() == XmlRpc::XmlRpcValue::TypeString) {
        config.topic = static_cast<std::string>(specification);
      } else if (specification.getType() ==
                 XmlRpc::XmlRpcValue::TypeStruct) {
        config.topic = xmlString(specification, "topic");
        config.timeout =
            xmlDouble(specification, "timeout", default_timeout);
        config.required =
            xmlBool(specification, "required", default_required);
        config.name = xmlString(specification, "name");
      } else {
        throw std::runtime_error(
            "correction必须使用话题字符串或参数映射");
      }
      config.topic = correctionTopic(config.topic);
      if (config.topic.empty() || config.timeout <= 0.0) {
        throw std::runtime_error(
            "来源'" + source->config.name + "'存在无效correction参数");
      }
      std::string provider_path = trimSlashes(config.topic);
      const std::string input_prefix = "state_estimator_inputs/";
      const std::size_t input_prefix_position =
          provider_path.find(input_prefix);
      if (input_prefix_position != std::string::npos) {
        provider_path.erase(
            0, input_prefix_position + input_prefix.size());
      }
      const std::size_t separator = provider_path.find('/');
      const std::string candidate = provider_path.substr(0, separator);
      if (source_by_name_.count(candidate) != 0) {
        config.provider = candidate;
      }
      const int ordinal = ++kind_counts[kind_text];
      if (config.name.empty()) {
        config.name =
            ordinal == 1 ? kind_text
                         : kind_text + "_" + std::to_string(ordinal);
      }
      auto correction = std::make_unique<CorrectionRuntime>();
      correction->config = config;
      subscribeCorrection(source, correction.get());
      ROS_INFO(
          "[xd_uav_state_estimators] %s/%s <- %s，provider=%s，required=%s",
          source->config.name.c_str(), config.name.c_str(),
          nh_.resolveName(config.topic).c_str(),
          config.provider.empty() ? "standardized" : config.provider.c_str(),
          config.required ? "true" : "false");
      source->corrections.push_back(std::move(correction));
    }
  }

  void subscribeCorrection(SourceRuntime* source,
                           CorrectionRuntime* correction) {
    const std::string& topic = correction->config.topic;
    switch (correction->config.kind) {
      case CorrectionKind::kPositionXY:
        correction->subscriber =
            nh_.subscribe<xd_uav_state_estimators::PositionXY>(
                topic, 30,
                [this, source, correction](
                    const xd_uav_state_estimators::PositionXY::ConstPtr&
                        message) {
                  CorrectionSample sample;
                  sample.kind = CorrectionKind::kPositionXY;
                  sample.stamp = message->header.stamp;
                  sample.frame_id = trimSlashes(message->header.frame_id);
                  sample.child_frame_id =
                      trimSlashes(message->child_frame_id);
                  sample.vector = Eigen::Vector2d(message->x, message->y);
                  sample.variance_a = message->covariance[0];
                  sample.variance_b = message->covariance[3];
                  sample.valid = message->valid;
                  processCorrection(source, correction, sample);
                });
        break;
      case CorrectionKind::kPositionZ:
        correction->subscriber =
            nh_.subscribe<xd_uav_state_estimators::PositionZ>(
                topic, 30,
                [this, source, correction](
                    const xd_uav_state_estimators::PositionZ::ConstPtr&
                        message) {
                  CorrectionSample sample;
                  sample.kind = CorrectionKind::kPositionZ;
                  sample.stamp = message->header.stamp;
                  sample.frame_id = trimSlashes(message->header.frame_id);
                  sample.child_frame_id =
                      trimSlashes(message->child_frame_id);
                  sample.scalar = message->z;
                  sample.variance_a = message->variance;
                  sample.valid = message->valid;
                  processCorrection(source, correction, sample);
                });
        break;
      case CorrectionKind::kVelocityXY:
        correction->subscriber =
            nh_.subscribe<xd_uav_state_estimators::VelocityXY>(
                topic, 30,
                [this, source, correction](
                    const xd_uav_state_estimators::VelocityXY::ConstPtr&
                        message) {
                  CorrectionSample sample;
                  sample.kind = CorrectionKind::kVelocityXY;
                  sample.stamp = message->header.stamp;
                  sample.frame_id = trimSlashes(message->header.frame_id);
                  sample.child_frame_id =
                      trimSlashes(message->child_frame_id);
                  sample.vector = Eigen::Vector2d(message->x, message->y);
                  sample.variance_a = message->covariance[0];
                  sample.variance_b = message->covariance[3];
                  sample.valid = message->valid;
                  processCorrection(source, correction, sample);
                });
        break;
      case CorrectionKind::kVelocityZ:
        correction->subscriber =
            nh_.subscribe<xd_uav_state_estimators::VelocityZ>(
                topic, 30,
                [this, source, correction](
                    const xd_uav_state_estimators::VelocityZ::ConstPtr&
                        message) {
                  CorrectionSample sample;
                  sample.kind = CorrectionKind::kVelocityZ;
                  sample.stamp = message->header.stamp;
                  sample.frame_id = trimSlashes(message->header.frame_id);
                  sample.child_frame_id =
                      trimSlashes(message->child_frame_id);
                  sample.scalar = message->z;
                  sample.variance_a = message->variance;
                  sample.valid = message->valid;
                  processCorrection(source, correction, sample);
                });
        break;
      case CorrectionKind::kHeading:
        correction->subscriber =
            nh_.subscribe<xd_uav_state_estimators::Heading>(
                topic, 30,
                [this, source, correction](
                    const xd_uav_state_estimators::Heading::ConstPtr&
                        message) {
                  CorrectionSample sample;
                  sample.kind = CorrectionKind::kHeading;
                  sample.stamp = message->header.stamp;
                  sample.frame_id = trimSlashes(message->header.frame_id);
                  sample.child_frame_id =
                      trimSlashes(message->child_frame_id);
                  sample.scalar = wrapAngle(message->heading);
                  sample.variance_a = message->variance;
                  sample.valid = message->valid;
                  processCorrection(source, correction, sample);
                });
        break;
      case CorrectionKind::kYawRate:
        correction->subscriber =
            nh_.subscribe<xd_uav_state_estimators::YawRate>(
                topic, 30,
                [this, source, correction](
                    const xd_uav_state_estimators::YawRate::ConstPtr&
                        message) {
                  CorrectionSample sample;
                  sample.kind = CorrectionKind::kYawRate;
                  sample.stamp = message->header.stamp;
                  sample.frame_id = trimSlashes(message->header.frame_id);
                  sample.child_frame_id =
                      trimSlashes(message->child_frame_id);
                  sample.scalar = message->yaw_rate;
                  sample.variance_a = message->variance;
                  sample.valid = message->valid;
                  processCorrection(source, correction, sample);
                });
        break;
    }
  }

  bool sampleValid(const SourceRuntime& source,
                   const CorrectionSample& sample,
                   std::string* reason) const {
    const ros::Time now = ros::Time::now();
    if (!sample.valid) {
      *reason = "适配器标记数据无效";
      return false;
    }
    if (sample.stamp.isZero() ||
        (now - sample.stamp).toSec() > max_localization_delay_ ||
        sample.stamp > now + ros::Duration(0.05)) {
      *reason = "时间戳超出允许范围";
      return false;
    }
    if (sample.child_frame_id != body_frame_) {
      *reason = "child_frame_id不是标准base_link";
      return false;
    }
    if (!std::isfinite(sample.vector.x()) ||
        !std::isfinite(sample.vector.y()) ||
        !std::isfinite(sample.scalar) ||
        !std::isfinite(sample.variance_a) ||
        !std::isfinite(sample.variance_b) ||
        sample.variance_a <= 0.0 || sample.variance_b <= 0.0) {
      *reason = "数值或协方差非法";
      return false;
    }
    if (sample.kind != CorrectionKind::kYawRate &&
        sample.frame_id.empty()) {
      *reason = "参考坐标系为空";
      return false;
    }
    const double max_variance =
        sample.kind == CorrectionKind::kPositionXY ||
                sample.kind == CorrectionKind::kPositionZ
            ? source.config.max_position_variance
            : (sample.kind == CorrectionKind::kVelocityXY ||
                       sample.kind == CorrectionKind::kVelocityZ
                   ? source.config.max_velocity_variance
                   : (sample.kind == CorrectionKind::kHeading
                          ? source.config.max_heading_variance
                          : source.config.max_yaw_rate_variance));
    if (sample.variance_a > max_variance ||
        sample.variance_b > max_variance) {
      *reason = "协方差超过可靠性上限";
      return false;
    }
    return true;
  }

  void processCorrection(SourceRuntime* source,
                         CorrectionRuntime* correction,
                         const CorrectionSample& sample) {
    correction->received++;
    correction->last_received = ros::Time::now();
    if (correction->last_received < correction->quarantine_until) {
      return;
    }
    if (!correction->quarantine_until.isZero() &&
        !correction->recovering) {
      correction->recovering = true;
      correction->recovery_started = correction->last_received;
      correction->recovery_samples = 0;
    }

    std::string reason;
    if (!sampleValid(*source, sample, &reason)) {
      registerRejection(source, correction, reason);
      return;
    }
    const bool own_provider =
        correction->config.provider == source->config.name;
    if (own_provider) {
      if (isPoseCorrection(sample.kind)) {
        if (source->raw_frame.empty()) {
          source->raw_frame = sample.frame_id;
        } else if (sample.frame_id != source->raw_frame) {
          registerRejection(source, correction,
                            "同一提供者的位姿坐标系发生变化");
          return;
        }
      }
      if (!source->filters_initialized) {
        updateRawState(&source->raw, sample);
        source->latest_raw_stamp =
            std::max(source->latest_raw_stamp, sample.stamp);
      }
    }

    const bool alignment_was_initialized = source->alignment_initialized;
    initializeAlignmentIfReady(source, sample.stamp);
    if (!source->alignment_initialized) {
      return;
    }

    if (!alignment_was_initialized) {
      initializeFusedRawFromNative(source);
      publishAlignment(*source, sample.stamp);
    }

    CorrectionSample aligned;
    if (!standardizeSample(*source, *correction, sample, &aligned,
                           &reason)) {
      ROS_WARN_THROTTLE(
          1.0, "[xd_uav_state_estimators] 修正%s/%s等待标准化: %s",
          source->config.name.c_str(), correction->config.name.c_str(),
          reason.c_str());
      return;
    }
    if (!source->filters_initialized) {
      updateRawState(&source->fused_raw, aligned);
      correction->valid_session = source->session;
      registerAccepted(source, correction);
      if (!fusedRawReady(*source)) {
        return;
      }
      initializeFiltersFromFusedRaw(source, sample.stamp);
      if (source->config.name == active_source_) {
        // 活动来源的新会话已经直接对齐到当前main，因此旧会话的handover
        // 不能继续叠加。
        main_handover_translation_.setZero();
        main_handover_yaw_ = 0.0;
        main_handover_start_time_ = ros::Time();
        main_handover_rapid_recovery_ = false;
      }
      publishAlignment(*source, sample.stamp);
      updateSelection(ros::Time::now());
      return;
    }

    predictSourceTo(source, sample.stamp);
    if (!innovationAccepted(source->axis, source->yaw_filter,
                            aligned, &reason)) {
      registerRejection(source, correction, reason);
      updateSelection(ros::Time::now());
      return;
    }
    correctSource(source, aligned);
    if (own_provider) {
      updateRawState(&source->raw, sample);
      source->latest_raw_stamp =
          std::max(source->latest_raw_stamp, sample.stamp);
    }
    updateRawState(&source->fused_raw, aligned);
    correction->valid_session = source->session;
    registerAccepted(source, correction);
    publishAlignment(*source, sample.stamp);
    updateSelection(ros::Time::now());
  }

  void updateRawState(RawState* state,
                      const CorrectionSample& sample) {
    switch (sample.kind) {
      case CorrectionKind::kPositionXY:
        state->position_xy = sample.vector;
        state->have_position_xy = true;
        break;
      case CorrectionKind::kPositionZ:
        state->position_z = sample.scalar;
        state->have_position_z = true;
        break;
      case CorrectionKind::kVelocityXY:
        state->velocity_xy = sample.vector;
        state->have_velocity_xy = true;
        break;
      case CorrectionKind::kVelocityZ:
        state->velocity_z = sample.scalar;
        state->have_velocity_z = true;
        break;
      case CorrectionKind::kHeading:
        state->heading = wrapAngle(sample.scalar);
        state->have_heading = true;
        break;
      case CorrectionKind::kYawRate:
        state->yaw_rate = sample.scalar;
        state->have_yaw_rate = true;
        break;
    }
  }

  bool hasCorrectionKind(const SourceRuntime& source,
                         const CorrectionKind kind) const {
    return std::any_of(
        source.corrections.begin(), source.corrections.end(),
        [kind](const std::unique_ptr<CorrectionRuntime>& correction) {
          return correction->config.kind == kind;
        });
  }

  bool hasNativeCorrectionKind(const SourceRuntime& source,
                               const CorrectionKind kind) const {
    return std::any_of(
        source.corrections.begin(), source.corrections.end(),
        [&source, kind](const std::unique_ptr<CorrectionRuntime>& correction) {
          return correction->config.kind == kind &&
                 correction->config.provider == source.config.name;
        });
  }

  bool rawReadyForAlignment(const SourceRuntime& source) const {
    const bool position_xy_ready =
        !hasNativeCorrectionKind(source, CorrectionKind::kPositionXY) ||
        source.raw.have_position_xy;
    const bool position_z_ready =
        !hasNativeCorrectionKind(source, CorrectionKind::kPositionZ) ||
        source.raw.have_position_z;
    const bool heading_ready =
        !hasNativeCorrectionKind(source, CorrectionKind::kHeading) ||
        source.raw.have_heading;
    const bool has_native_pose =
        hasNativeCorrectionKind(source, CorrectionKind::kPositionXY) ||
        hasNativeCorrectionKind(source, CorrectionKind::kPositionZ) ||
        hasNativeCorrectionKind(source, CorrectionKind::kHeading);
    return position_xy_ready && position_z_ready && heading_ready &&
           (!source.raw_frame.empty() ||
            !has_native_pose);
  }

  void initializeAlignmentIfReady(SourceRuntime* source,
                                  const ros::Time& stamp) {
    if (source->alignment_initialized ||
        !rawReadyForAlignment(*source)) {
      return;
    }
    if (source->config.alignment_mode == "identity" ||
        source->raw_frame == odom_frame_ ||
        active_source_.empty()) {
      source->alignment_yaw = 0.0;
      source->alignment_translation.setZero();
      source->alignment_initialized = true;
      return;
    }
    Estimate target;
    if (!currentMainEstimate(stamp, &target)) {
      return;
    }
    calculateAlignmentToTarget(source, target);
  }

  void calculateAlignmentToTarget(SourceRuntime* source,
                                  const Estimate& target) {
    source->alignment_yaw =
        source->raw.have_heading
            ? wrapAngle(target.yaw - source->raw.heading)
            : 0.0;
    const Eigen::Rotation2Dd rotation(source->alignment_yaw);
    if (source->raw.have_position_xy) {
      const Eigen::Vector2d translated =
          target.position.head<2>() -
          rotation * source->raw.position_xy;
      source->alignment_translation.x() = translated.x();
      source->alignment_translation.y() = translated.y();
    } else {
      source->alignment_translation.x() = 0.0;
      source->alignment_translation.y() = 0.0;
    }
    source->alignment_translation.z() =
        source->raw.have_position_z
            ? target.position.z() - source->raw.position_z
            : 0.0;
    source->alignment_initialized = true;
  }

  void initializeFiltersFromFusedRaw(SourceRuntime* source,
                                     const ros::Time& stamp) {
    const RawState& state = source->fused_raw;
    Estimate main_seed;
    const bool have_main_seed = currentMainEstimate(stamp, &main_seed);
    const Eigen::Vector2d position =
        state.have_position_xy ? state.position_xy : Eigen::Vector2d::Zero();
    Eigen::Vector2d velocity = Eigen::Vector2d::Zero();
    if (state.have_velocity_xy) {
      velocity = state.velocity_xy;
    } else if (have_main_seed) {
      velocity = main_seed.velocity.head<2>();
    }
    const Eigen::Vector3d acceleration =
        have_main_seed ? main_seed.acceleration : Eigen::Vector3d::Zero();
    source->axis[0].initialize(position.x(), velocity.x(), acceleration.x());
    source->axis[1].initialize(position.y(), velocity.y(), acceleration.y());
    source->axis[2].initialize(
        state.have_position_z ? state.position_z : 0.0,
        state.have_velocity_z
            ? state.velocity_z
            : (have_main_seed ? main_seed.velocity.z() : 0.0),
        acceleration.z());
    source->yaw_filter.initialize(
        state.have_heading ? state.heading : 0.0,
        state.have_yaw_rate
            ? state.yaw_rate
            : (have_main_seed ? main_seed.yaw_rate : 0.0));
    source->filter_stamp = stamp;
    source->filters_initialized = true;
  }

  void initializeFusedRawFromNative(SourceRuntime* source) {
    const Eigen::Rotation2Dd rotation(source->alignment_yaw);
    RawState& output = source->fused_raw;
    const RawState& input = source->raw;
    if (input.have_position_xy) {
      output.position_xy = rotation * input.position_xy +
                           source->alignment_translation.head<2>();
      output.have_position_xy = true;
    }
    if (input.have_position_z) {
      output.position_z = input.position_z + source->alignment_translation.z();
      output.have_position_z = true;
    }
    if (input.have_velocity_xy) {
      output.velocity_xy = rotation * input.velocity_xy;
      output.have_velocity_xy = true;
    }
    if (input.have_velocity_z) {
      output.velocity_z = input.velocity_z;
      output.have_velocity_z = true;
    }
    if (input.have_heading) {
      output.heading = wrapAngle(input.heading + source->alignment_yaw);
      output.have_heading = true;
    }
    if (input.have_yaw_rate) {
      output.yaw_rate = input.yaw_rate;
      output.have_yaw_rate = true;
    }
  }

  bool fusedRawReady(const SourceRuntime& source) const {
    return std::all_of(
        source.corrections.begin(), source.corrections.end(),
        [&source](const std::unique_ptr<CorrectionRuntime>& correction) {
          return !correction->config.required ||
                 correction->valid_session == source.session;
        });
  }

  bool standardizeSample(const SourceRuntime& consumer,
                         const CorrectionRuntime& correction,
                         const CorrectionSample& sample,
                         CorrectionSample* standardized,
                         std::string* reason) const {
    if (sample.frame_id == odom_frame_ ||
        sample.kind == CorrectionKind::kYawRate) {
      *standardized = sample;
      standardized->frame_id = odom_frame_;
      return true;
    }
    if (sample.frame_id == body_frame_) {
      if (sample.kind == CorrectionKind::kVelocityZ) {
        *standardized = sample;
        standardized->frame_id = odom_frame_;
        return true;
      }
      if (sample.kind != CorrectionKind::kVelocityXY) {
        *reason = "只有速度允许在base_link中表达";
        return false;
      }
      *standardized = sample;
      const double yaw = consumer.filters_initialized
                             ? consumer.yaw_filter.yaw()
                             : (consumer.fused_raw.have_heading
                                    ? consumer.fused_raw.heading
                                    : 0.0);
      standardized->vector = Eigen::Rotation2Dd(yaw) * sample.vector;
      standardized->frame_id = odom_frame_;
      return true;
    }
    if (correction.config.provider.empty()) {
      *reason = "非标准odom输入的话题前缀未对应任何定位提供者";
      return false;
    }
    const auto provider_iterator =
        source_by_name_.find(correction.config.provider);
    if (provider_iterator == source_by_name_.end()) {
      *reason = "定位提供者不存在";
      return false;
    }
    const SourceRuntime& provider = *provider_iterator->second;
    if (!provider.alignment_initialized || provider.raw_frame.empty()) {
      *reason = "提供者" + correction.config.provider + "尚未完成原点对齐";
      return false;
    }
    if (sample.frame_id != provider.raw_frame) {
      *reason = "消息坐标系与提供者" + correction.config.provider +
                "的来源原点不一致";
      return false;
    }
    *standardized = alignSample(provider, sample);
    return true;
  }

  CorrectionSample alignSample(const SourceRuntime& source,
                               const CorrectionSample& sample) const {
    CorrectionSample aligned = sample;
    const Eigen::Rotation2Dd rotation(source.alignment_yaw);
    switch (sample.kind) {
      case CorrectionKind::kPositionXY:
        if (sample.frame_id != odom_frame_) {
          aligned.vector =
              rotation * sample.vector +
              source.alignment_translation.head<2>();
        }
        break;
      case CorrectionKind::kPositionZ:
        if (sample.frame_id != odom_frame_) {
          aligned.scalar += source.alignment_translation.z();
        }
        break;
      case CorrectionKind::kVelocityXY:
        if (sample.frame_id == body_frame_) {
          const Eigen::Rotation2Dd body_to_odom(
              source.yaw_filter.yaw());
          aligned.vector = body_to_odom * sample.vector;
        } else if (sample.frame_id != odom_frame_) {
          aligned.vector = rotation * sample.vector;
        }
        break;
      case CorrectionKind::kVelocityZ:
        break;
      case CorrectionKind::kHeading:
        if (sample.frame_id != odom_frame_) {
          aligned.scalar =
              wrapAngle(sample.scalar + source.alignment_yaw);
        }
        break;
      case CorrectionKind::kYawRate:
        break;
    }
    aligned.frame_id = odom_frame_;
    return aligned;
  }

  void predictSourceTo(SourceRuntime* source, const ros::Time& stamp) {
    if (!source->filters_initialized) {
      return;
    }
    const double dt = std::max(0.0, (stamp - source->filter_stamp).toSec());
    if (dt <= 0.0) {
      return;
    }
    for (AxisKalman& axis : source->axis) {
      axis.predict(dt);
    }
    source->yaw_filter.predict(dt);
    if (imuAccelerationFresh()) {
      for (int axis = 0; axis < 3; ++axis) {
        source->axis[axis].correct(
            2, latest_acceleration_odom_(axis),
            imu_acceleration_variance_);
      }
    }
    if (imuRateFresh()) {
      source->yaw_filter.correctRate(latest_yaw_rate_,
                                     imu_yaw_rate_variance_);
    }
    source->filter_stamp = stamp;
  }

  bool innovationAccepted(const std::array<AxisKalman, 3>& axis,
                          const YawKalman& yaw_filter,
                          const CorrectionSample& sample,
                          std::string* reason) const {
    switch (sample.kind) {
      case CorrectionKind::kPositionXY: {
        const double dx =
            axis[0].innovation(0, sample.vector.x());
        const double dy =
            axis[1].innovation(0, sample.vector.y());
        const double nis =
            axis[0].nis(0, sample.vector.x(), sample.variance_a) +
            axis[1].nis(0, sample.vector.y(), sample.variance_b);
        if (std::hypot(dx, dy) > position_xy_limit_ ||
            (use_nis_ && nis > position_xy_nis_limit_)) {
          *reason = "水平位置新息超限";
          return false;
        }
        break;
      }
      case CorrectionKind::kPositionZ:
        if (std::abs(axis[2].innovation(0, sample.scalar)) >
                position_z_limit_ ||
            (use_nis_ &&
             axis[2].nis(0, sample.scalar, sample.variance_a) >
                 position_z_nis_limit_)) {
          *reason = "高度新息超限";
          return false;
        }
        break;
      case CorrectionKind::kVelocityXY: {
        const double dx =
            axis[0].innovation(1, sample.vector.x());
        const double dy =
            axis[1].innovation(1, sample.vector.y());
        const double nis =
            axis[0].nis(1, sample.vector.x(), sample.variance_a) +
            axis[1].nis(1, sample.vector.y(), sample.variance_b);
        if (std::hypot(dx, dy) > velocity_xy_limit_ ||
            (use_nis_ && nis > velocity_xy_nis_limit_)) {
          *reason = "水平速度新息超限";
          return false;
        }
        break;
      }
      case CorrectionKind::kVelocityZ:
        if (std::abs(axis[2].innovation(1, sample.scalar)) >
                velocity_z_limit_ ||
            (use_nis_ &&
             axis[2].nis(1, sample.scalar, sample.variance_a) >
                 velocity_z_nis_limit_)) {
          *reason = "垂直速度新息超限";
          return false;
        }
        break;
      case CorrectionKind::kHeading:
        if (std::abs(yaw_filter.yawInnovation(sample.scalar)) >
                heading_limit_ ||
            (use_nis_ &&
             yaw_filter.yawNis(sample.scalar, sample.variance_a) >
                 heading_nis_limit_)) {
          *reason = "航向新息超限";
          return false;
        }
        break;
      case CorrectionKind::kYawRate:
        if (std::abs(yaw_filter.rateInnovation(sample.scalar)) >
                yaw_rate_limit_ ||
            (use_nis_ &&
             yaw_filter.rateNis(sample.scalar, sample.variance_a) >
                 yaw_rate_nis_limit_)) {
          *reason = "偏航角速度新息超限";
          return false;
        }
        break;
    }
    return true;
  }

  void correctSource(SourceRuntime* source,
                     const CorrectionSample& sample) {
    switch (sample.kind) {
      case CorrectionKind::kPositionXY:
        source->axis[0].correct(0, sample.vector.x(),
                                sample.variance_a);
        source->axis[1].correct(0, sample.vector.y(),
                                sample.variance_b);
        break;
      case CorrectionKind::kPositionZ:
        source->axis[2].correct(0, sample.scalar,
                                sample.variance_a);
        break;
      case CorrectionKind::kVelocityXY:
        source->axis[0].correct(1, sample.vector.x(),
                                sample.variance_a);
        source->axis[1].correct(1, sample.vector.y(),
                                sample.variance_b);
        break;
      case CorrectionKind::kVelocityZ:
        source->axis[2].correct(1, sample.scalar,
                                sample.variance_a);
        break;
      case CorrectionKind::kHeading:
        source->yaw_filter.correctYaw(sample.scalar,
                                      sample.variance_a);
        break;
      case CorrectionKind::kYawRate:
        source->yaw_filter.correctRate(sample.scalar,
                                       sample.variance_a);
        break;
    }
  }

  void initializeMainFromEstimate(const Estimate& estimate) {
    for (int index = 0; index < 3; ++index) {
      main_state_->axis[index].initialize(
          estimate.position(index), estimate.velocity(index),
          estimate.acceleration(index), estimate.position_variance[index],
          estimate.velocity_variance[index],
          estimate.acceleration_variance[index]);
    }
    main_state_->yaw_filter.initialize(
        estimate.yaw, estimate.yaw_rate, estimate.yaw_variance,
        estimate.yaw_rate_variance);
    main_state_->filter_stamp = estimate.stamp;
    main_state_->initialized = true;
  }

  void predictMainTo(const ros::Time& stamp) {
    if (!main_state_->initialized) {
      return;
    }
    const double dt =
        std::max(0.0, (stamp - main_state_->filter_stamp).toSec());
    if (dt <= 0.0) {
      return;
    }
    for (AxisKalman& axis : main_state_->axis) {
      axis.predict(dt);
    }
    main_state_->yaw_filter.predict(dt);
    main_state_->filter_stamp = stamp;
  }

  void fuseImuIntoMain(const ros::Time& stamp) {
    if (!main_state_->initialized ||
        (!last_main_imu_fusion_stamp_.isZero() &&
         stamp <= last_main_imu_fusion_stamp_)) {
      return;
    }
    if (stamp < main_state_->filter_stamp) {
      ROS_WARN_THROTTLE(
          1.0,
          "[xd_uav_state_estimators] 跳过早于main滤波时刻的IMU样本");
      return;
    }
    predictMainTo(stamp);
    for (int axis = 0; axis < 3; ++axis) {
      main_state_->axis[axis].correct(
          2, latest_acceleration_odom_(axis),
          imu_acceleration_variance_);
    }
    if (have_imu_rate_) {
      main_state_->yaw_filter.correctRate(
          latest_yaw_rate_, imu_yaw_rate_variance_);
    }
    last_main_imu_fusion_stamp_ = stamp;
  }

  double mainHandoverScale(const ros::Time& stamp) const {
    if (main_handover_start_time_.isZero()) {
      return 0.0;
    }
    // 正常切源时，handover表示“新来源连续坐标”到当前控制odom的固定
    // 坐标偏移。它必须在该来源保持活动期间一直存在；若逐渐衰减，main会在
    // 切换后数秒内重新靠向新来源的原始偏差，表现为飞机持续掉高或横向漂移。
    if (!main_handover_rapid_recovery_) {
      return 1.0;
    }
    const double elapsed =
        std::max(0.0, (stamp - main_handover_start_time_).toSec());
    // 只有持久分歧触发的安全回退才释放旧坐标偏移，使健康来源在短时间内
    // 恢复真实状态。该路径由rapid_recovery显式标识。
    return std::exp(-elapsed / 0.5);
  }

  Estimate applyMainHandover(const Estimate& source) const {
    Estimate handed_over = source;
    const double scale = mainHandoverScale(source.stamp);
    const double handover_yaw = scale * main_handover_yaw_;
    const Eigen::Vector3d handover_translation =
        scale * main_handover_translation_;
    const Eigen::Rotation2Dd rotation(handover_yaw);
    handed_over.position.head<2>() =
        rotation * source.position.head<2>() +
        handover_translation.head<2>();
    handed_over.position.z() += handover_translation.z();
    handed_over.velocity.head<2>() =
        rotation * source.velocity.head<2>();
    handed_over.acceleration.head<2>() =
        rotation * source.acceleration.head<2>();
    handed_over.yaw = wrapAngle(source.yaw + handover_yaw);

    const double cosine = std::cos(handover_yaw);
    const double sine = std::sin(handover_yaw);
    const auto rotate_variance =
        [cosine, sine](const std::array<double, 3>& input) {
          std::array<double, 3> output = input;
          output[0] = cosine * cosine * input[0] +
                      sine * sine * input[1];
          output[1] = sine * sine * input[0] +
                      cosine * cosine * input[1];
          return output;
        };
    handed_over.position_variance =
        rotate_variance(source.position_variance);
    handed_over.velocity_variance =
        rotate_variance(source.velocity_variance);
    handed_over.acceleration_variance =
        rotate_variance(source.acceleration_variance);
    return handed_over;
  }

  void registerAccepted(SourceRuntime* source,
                        CorrectionRuntime* correction) {
    const ros::Time now = ros::Time::now();
    correction->accepted++;
    correction->last_accepted = now;
    correction->consecutive_rejections = 0;
    if (correction->recovering) {
      correction->recovery_samples++;
      if (correction->recovery_samples >=
              source->config.recovery_min_samples &&
          (now - correction->recovery_started).toSec() >=
              source->config.recovery_stable_time) {
        correction->recovering = false;
        correction->quarantine_until = ros::Time();
        ROS_INFO(
            "[xd_uav_state_estimators] 修正%s/%s已恢复",
            source->config.name.c_str(),
            correction->config.name.c_str());
      }
    }
  }

  void registerRejection(SourceRuntime* source,
                         CorrectionRuntime* correction,
                         const std::string& reason) {
    correction->rejected++;
    correction->consecutive_rejections++;
    if (correction->consecutive_rejections >=
        source->config.max_consecutive_rejections) {
      correction->quarantine_until =
          ros::Time::now() +
          ros::Duration(source->config.quarantine_duration);
      correction->recovering = false;
      correction->recovery_samples = 0;
      ROS_WARN(
          "[xd_uav_state_estimators] 修正%s/%s已隔离: %s",
          source->config.name.c_str(),
          correction->config.name.c_str(), reason.c_str());
    } else {
      ROS_WARN_THROTTLE(
          1.0, "[xd_uav_state_estimators] 修正%s/%s被拒绝: %s",
          source->config.name.c_str(),
          correction->config.name.c_str(), reason.c_str());
    }
  }

  bool correctionHealthy(const CorrectionRuntime& correction,
                         const ros::Time& now) const {
    return !correction.last_accepted.isZero() &&
           now >= correction.quarantine_until &&
           !correction.recovering &&
           (now - correction.last_accepted).toSec() <=
               correction.config.timeout;
  }

  bool sourceHealthy(const SourceRuntime& source,
                     const ros::Time& now) const {
    if (!source.filters_initialized ||
        !source.alignment_initialized) {
      return false;
    }
    bool have_required = false;
    for (const auto& correction : source.corrections) {
      if (!correction->config.required) {
        continue;
      }
      have_required = true;
      if (!correctionHealthy(*correction, now)) {
        return false;
      }
    }
    return have_required;
  }

  std::string sourceHealthReason(const SourceRuntime& source,
                                 const ros::Time& now) const {
    if (!source.alignment_initialized) {
      return "来源原点尚未对齐";
    }
    if (!source.filters_initialized) {
      return "单源滤波器尚未初始化";
    }
    for (const auto& correction : source.corrections) {
      if (!correction->config.required ||
          correctionHealthy(*correction, now)) {
        continue;
      }
      if (correction->last_accepted.isZero()) {
        return correction->config.name + "尚无有效数据";
      }
      if (now < correction->quarantine_until) {
        return correction->config.name + "处于隔离期";
      }
      if (correction->recovering) {
        return correction->config.name + "处于恢复观察期";
      }
      return correction->config.name + "已超时";
    }
    return "没有必需修正";
  }

  bool requiredNativeCorrectionsConnected(const SourceRuntime& source,
                                          const ros::Time& now) const {
    bool have_required = false;
    for (const auto& correction : source.corrections) {
      if (!correction->config.required ||
          correction->config.provider != source.config.name) {
        continue;
      }
      have_required = true;
      if (correction->last_received.isZero() ||
          (now - correction->last_received).toSec() >
              correction->config.timeout) {
        return false;
      }
    }
    return have_required;
  }

  void prepareSourceForReconnect(SourceRuntime* source) {
    source->alignment_initialized = false;
    source->filters_initialized = false;
    source->raw = RawState();
    source->fused_raw = RawState();
    source->raw_frame.clear();
    source->filter_stamp = ros::Time();
    source->latest_raw_stamp = ros::Time();
    source->alignment_translation.setZero();
    source->alignment_yaw = 0.0;
    source->session++;
    if (source->session == 0) {
      source->session = 1;
      for (auto& correction : source->corrections) {
        correction->valid_session = 0;
      }
    }
    // last_accepted必须保留给main的dead-reckoning计时使用。上一次会话的
    // 原始数值已经由RawState清除；只有重连后重新收到全部必需位姿修正，
    // rawReadyForAlignment()才会允许建立新原点。
  }

  void updateSourceConnectionStates(const ros::Time& now) {
    for (auto& source : sources_) {
      const bool connected =
          requiredNativeCorrectionsConnected(*source, now);
      if (source->required_corrections_were_connected && !connected &&
          source->config.alignment_mode == "align_on_activation") {
        prepareSourceForReconnect(source.get());
        ROS_INFO(
            "[xd_uav_state_estimators] 来源%s的必需修正已中断，"
            "下次完整接入时将重新对齐来源原点",
            source->config.name.c_str());
      }
      source->required_corrections_were_connected = connected;
    }
  }

  double sourceLocalizationAge(const SourceRuntime& source,
                               const ros::Time& now) const {
    double age = 0.0;
    bool found = false;
    for (const auto& correction : source.corrections) {
      if (!correction->config.required ||
          correction->last_accepted.isZero()) {
        continue;
      }
      age = std::max(
          age, (now - correction->last_accepted).toSec());
      found = true;
    }
    return found ? age : std::numeric_limits<double>::infinity();
  }

  SourceRuntime* chooseDesiredSource(const ros::Time& now) const {
    if (!automatic_selection_ && !requested_source_.empty()) {
      const auto iterator =
          source_by_name_.find(requested_source_);
      if (iterator != source_by_name_.end() &&
          sourceHealthy(*iterator->second, now)) {
        return iterator->second;
      }
    }
    SourceRuntime* desired = nullptr;
    for (const auto& source : sources_) {
      if (!sourceHealthy(*source, now)) {
        continue;
      }
      if (desired == nullptr ||
          source->config.priority < desired->config.priority) {
        desired = source.get();
      }
    }
    return desired;
  }

  SourceRuntime* healthierHigherPrioritySource(
      const SourceRuntime& active, const ros::Time& now) const {
    SourceRuntime* reference = nullptr;
    for (const auto& source : sources_) {
      if (source.get() == &active ||
          source->config.priority >= active.config.priority ||
          !sourceHealthy(*source, now)) {
        continue;
      }
      if (reference == nullptr ||
          source->config.priority < reference->config.priority) {
        reference = source.get();
      }
    }
    return reference;
  }

  bool sourcesDisagree(const SourceRuntime& active,
                       const SourceRuntime& reference,
                       const ros::Time& now,
                       std::string* reason) const {
    Estimate active_estimate;
    Estimate reference_estimate;
    if (!estimateSourceAt(active, now, &active_estimate) ||
        !estimateSourceAt(reference, now, &reference_estimate)) {
      return false;
    }

    if (active.fused_raw.have_position_z &&
        reference.fused_raw.have_position_z) {
      const double difference = std::abs(
          active_estimate.position.z() - reference_estimate.position.z());
      // 跨来源安全检查关注控制实际会看到的绝对状态差，不能让某一来源
      // 瞬时增大的协方差反复抬高阈值、重置持续性计时。
      const double limit = 0.05 * position_z_limit_;
      if (difference > limit) {
        *reason = "z差=" + std::to_string(difference) +
                  "m，阈值=" + std::to_string(limit) + "m";
        return true;
      }
      const double velocity_difference = std::abs(
          active_estimate.velocity.z() - reference_estimate.velocity.z());
      const double velocity_limit = 0.05 * velocity_z_limit_;
      if (velocity_difference > velocity_limit) {
        *reason = "竖直速度差=" + std::to_string(velocity_difference) +
                  "m/s，阈值=" + std::to_string(velocity_limit) +
                  "m/s";
        return true;
      }
    }

    if (active.fused_raw.have_position_xy &&
        reference.fused_raw.have_position_xy) {
      const Eigen::Vector2d difference =
          active_estimate.position.head<2>() -
          reference_estimate.position.head<2>();
      const double limit = 0.05 * position_xy_limit_;
      if (difference.norm() > limit) {
        *reason = "水平差=" + std::to_string(difference.norm()) +
                  "m，阈值=" + std::to_string(limit) + "m";
        return true;
      }
      const Eigen::Vector2d velocity_difference =
          active_estimate.velocity.head<2>() -
          reference_estimate.velocity.head<2>();
      const double velocity_limit = 0.05 * velocity_xy_limit_;
      if (velocity_difference.norm() > velocity_limit) {
        *reason = "水平速度差=" +
                  std::to_string(velocity_difference.norm()) +
                  "m/s，阈值=" + std::to_string(velocity_limit) +
                  "m/s";
        return true;
      }
    }

    if (active.fused_raw.have_heading &&
        reference.fused_raw.have_heading) {
      const double difference = std::abs(wrapAngle(
          active_estimate.yaw - reference_estimate.yaw));
      const double limit = 0.05 * heading_limit_;
      if (difference > limit) {
        *reason = "航向差=" + std::to_string(difference) +
                  "rad，阈值=" + std::to_string(limit) + "rad";
        return true;
      }
    }
    return false;
  }

  void reportSourceDisagreement(const SourceRuntime& active,
                                const ros::Time& now) const {
    SourceRuntime* reference =
        healthierHigherPrioritySource(active, now);
    if (reference == nullptr) {
      return;
    }

    std::string reason;
    if (!sourcesDisagree(active, *reference, now, &reason)) {
      return;
    }
    // 来源间差异只用于诊断。不同定位系统可能具有不同的长期误差，不能仅凭
    // 与高优先级来源不一致就覆盖用户的手动选择。
    ROS_WARN_THROTTLE(
        1.0,
        "[xd_uav_state_estimators] 活动来源%s与健康高优先级来源%s"
        "存在分歧(%s)，保留当前选源，仅作诊断",
        active.config.name.c_str(), reference->config.name.c_str(),
        reason.c_str());
  }

  void updateSelection(const ros::Time& now) {
    SourceRuntime* desired = chooseDesiredSource(now);
    SourceRuntime* active =
        active_source_.empty()
            ? nullptr
            : source_by_name_.at(active_source_);
    if (desired == nullptr) {
      return;
    }
    if (active == nullptr) {
      activateSource(desired, now);
      return;
    }
    if (!sourceHealthy(*active, now)) {
      // 只有当前来源本身失效时才故障接管，并快速恢复到健康来源的真实状态。
      activateSource(desired, now, true);
      return;
    }
    reportSourceDisagreement(*active, now);
    if (desired == active) {
      return;
    }
    if (!automatic_selection_ &&
        desired->config.name == requested_source_) {
      activateSource(desired, now);
      return;
    }
    if (!switch_back_to_higher_priority_ ||
        desired->config.priority >= active->config.priority) {
      return;
    }
    if ((now - active_since_).toSec() < minimum_active_time_ ||
        (now - last_switch_time_).toSec() < switch_cooldown_) {
      return;
    }
    activateSource(desired, now);
  }

  void activateSource(SourceRuntime* source, const ros::Time& now,
                      const bool rapid_recovery = false) {
    if (source == nullptr ||
        source->config.name == active_source_) {
      return;
    }
    // 来源原点属于一次定位会话，只能在首次完整接入或断线重连时建立。
    // 切换main来源不能再次移动来源原点，否则同一个source frame会因为
    // 选源动作而改变物理含义，并带动整条TF分支跳变。
    Estimate source_estimate;
    if (!estimateSourceAt(*source, now, &source_estimate)) {
      ROS_WARN(
          "[xd_uav_state_estimators] 来源%s尚不能初始化main状态",
          source->config.name.c_str());
      return;
    }
    if (main_state_->initialized) {
      Estimate target;
      if (!currentMainEstimate(now, &target)) {
        ROS_WARN(
            "[xd_uav_state_estimators] 无法取得切源前main状态");
        return;
      }
      // main接管的是来源滤波器的完整估计，因此handover也必须用同一个
      // 完整估计建立，不能再按原始position/heading逐字段锚定。
      const Estimate& source_anchor = source_estimate;
      main_handover_yaw_ =
          wrapAngle(target.yaw - source_anchor.yaw);
      const Eigen::Rotation2Dd rotation(main_handover_yaw_);
      main_handover_translation_.head<2>() =
          target.position.head<2>() -
          rotation * source_anchor.position.head<2>();
      main_handover_translation_.z() =
          target.position.z() - source_anchor.position.z();
    } else {
      main_handover_translation_.setZero();
      main_handover_yaw_ = 0.0;
      initializeMainFromEstimate(source_estimate);
    }
    active_source_ = source->config.name;
    active_since_ = now;
    last_switch_time_ = now;
    main_handover_start_time_ = now;
    main_handover_rapid_recovery_ = rapid_recovery;
    switch_count_++;
    ROS_INFO(
        "[xd_uav_state_estimators] main定位源切换为%s，"
        "handover=[%.3f, %.3f, %.3f, yaw %.3f]",
        active_source_.c_str(), main_handover_translation_.x(),
        main_handover_translation_.y(), main_handover_translation_.z(),
        main_handover_yaw_);
  }

  bool estimateSourceAt(const SourceRuntime& source,
                        const ros::Time& stamp,
                        Estimate* estimate) const {
    if (!source.filters_initialized) {
      return false;
    }
    std::array<AxisKalman, 3> axis = source.axis;
    YawKalman yaw = source.yaw_filter;
    const double dt =
        std::max(0.0, (stamp - source.filter_stamp).toSec());
    for (AxisKalman& filter : axis) {
      filter.predict(dt);
    }
    yaw.predict(dt);
    estimate->stamp = stamp;
    for (int index = 0; index < 3; ++index) {
      estimate->position(index) = axis[index].position();
      estimate->velocity(index) = axis[index].velocity();
      estimate->acceleration(index) = axis[index].acceleration();
      estimate->position_variance[index] =
          axis[index].variance(0);
      estimate->velocity_variance[index] =
          axis[index].variance(1);
      estimate->acceleration_variance[index] =
          axis[index].variance(2);
    }
    estimate->yaw = yaw.yaw();
    estimate->yaw_rate = yaw.rate();
    estimate->yaw_variance = yaw.variance(0);
    estimate->yaw_rate_variance = yaw.variance(1);
    estimate->roll = latest_roll_;
    estimate->pitch = latest_pitch_;
    return true;
  }

  bool currentMainEstimate(const ros::Time& stamp,
                           Estimate* estimate) const {
    if (!main_state_->initialized || active_source_.empty()) {
      return false;
    }
    std::array<AxisKalman, 3> axis = main_state_->axis;
    YawKalman yaw = main_state_->yaw_filter;
    const double dt =
        std::max(0.0, (stamp - main_state_->filter_stamp).toSec());
    for (AxisKalman& filter : axis) {
      filter.predict(dt);
    }
    yaw.predict(dt);
    estimate->stamp = stamp;
    for (int index = 0; index < 3; ++index) {
      estimate->position(index) = axis[index].position();
      estimate->velocity(index) = axis[index].velocity();
      estimate->acceleration(index) = axis[index].acceleration();
      estimate->position_variance[index] = axis[index].variance(0);
      estimate->velocity_variance[index] = axis[index].variance(1);
      estimate->acceleration_variance[index] = axis[index].variance(2);
    }
    estimate->yaw = yaw.yaw();
    estimate->yaw_rate = yaw.rate();
    estimate->yaw_variance = yaw.variance(0);
    estimate->yaw_rate_variance = yaw.variance(1);
    estimate->roll = latest_roll_;
    estimate->pitch = latest_pitch_;
    return true;
  }

  nav_msgs::Odometry estimateToOdometry(
      const Estimate& estimate) const {
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

    const tf2::Vector3 parent_velocity(
        estimate.velocity.x(), estimate.velocity.y(),
        estimate.velocity.z());
    const tf2::Vector3 body_velocity =
        tf2::quatRotate(orientation.inverse(), parent_velocity);
    const Eigen::Vector3d velocity(
        body_velocity.x(), body_velocity.y(), body_velocity.z());
    message.twist.twist.linear.x = velocity.x();
    message.twist.twist.linear.y = velocity.y();
    message.twist.twist.linear.z = velocity.z();
    if (have_body_rate_sample_) {
      // Odometry.twist必须在child_frame_id表达；滤波器中的yaw_rate是欧拉
      // yaw导数，不能再冒充机体系Z轴角速度。
      message.twist.twist.angular.x = latest_body_rate_.x();
      message.twist.twist.angular.y = latest_body_rate_.y();
      message.twist.twist.angular.z = latest_body_rate_.z();
    }

    Eigen::Matrix3d parent_velocity_covariance = Eigen::Matrix3d::Zero();
    for (int axis = 0; axis < 3; ++axis) {
      parent_velocity_covariance(axis, axis) =
          estimate.velocity_variance[axis];
    }
    const tf2::Matrix3x3 body_parent_basis(orientation.inverse());
    Eigen::Matrix3d body_parent_rotation;
    for (int row = 0; row < 3; ++row) {
      for (int column = 0; column < 3; ++column) {
        body_parent_rotation(row, column) =
            body_parent_basis[row][column];
      }
    }
    const Eigen::Matrix3d body_velocity_covariance =
        body_parent_rotation * parent_velocity_covariance *
        body_parent_rotation.transpose();
    for (int row = 0; row < 3; ++row) {
      for (int column = 0; column < 3; ++column) {
        message.twist.covariance[row * 6 + column] =
            body_velocity_covariance(row, column);
      }
    }
    message.twist.covariance[21] = imu_yaw_rate_variance_;
    message.twist.covariance[28] = imu_yaw_rate_variance_;
    message.twist.covariance[35] = imu_yaw_rate_variance_;
    return message;
  }

  nav_msgs::Odometry sourceEstimateToOdometry(
      const SourceRuntime& source, const Estimate& estimate) const {
    nav_msgs::Odometry message = estimateToOdometry(estimate);
    if (source.raw_frame.empty() ||
        source.raw_frame == odom_frame_ ||
        !source.alignment_initialized) {
      return message;
    }

    // 来源滤波器内部工作在公共odom中。对外发布单来源结果时，
    // 使用该来源的对齐量反变换，恢复成“来源原点 -> base_link”。
    tf2::Quaternion odom_origin_rotation;
    odom_origin_rotation.setRPY(0.0, 0.0, source.alignment_yaw);
    const tf2::Transform odom_origin(
        odom_origin_rotation,
        tf2::Vector3(source.alignment_translation.x(),
                     source.alignment_translation.y(),
                     source.alignment_translation.z()));
    tf2::Transform odom_body;
    tf2::fromMsg(message.pose.pose, odom_body);
    const tf2::Transform origin_body =
        odom_origin.inverseTimes(odom_body);

    message.header.frame_id = source.raw_frame;
    message.pose.pose.position.x = origin_body.getOrigin().x();
    message.pose.pose.position.y = origin_body.getOrigin().y();
    message.pose.pose.position.z = origin_body.getOrigin().z();
    message.pose.pose.orientation =
        tf2::toMsg(origin_body.getRotation());

    // 位姿协方差原先表达在odom中，需要同步旋转到来源原点。
    Eigen::Matrix<double, 6, 6> covariance;
    for (int row = 0; row < 6; ++row) {
      for (int column = 0; column < 6; ++column) {
        covariance(row, column) =
            message.pose.covariance[row * 6 + column];
      }
    }
    const tf2::Matrix3x3 basis(odom_origin_rotation.inverse());
    Eigen::Matrix3d rotation;
    for (int row = 0; row < 3; ++row) {
      for (int column = 0; column < 3; ++column) {
        rotation(row, column) = basis[row][column];
      }
    }
    Eigen::Matrix<double, 6, 6> covariance_rotation =
        Eigen::Matrix<double, 6, 6>::Zero();
    covariance_rotation.block<3, 3>(0, 0) = rotation;
    covariance_rotation.block<3, 3>(3, 3) = rotation;
    covariance =
        covariance_rotation * covariance *
        covariance_rotation.transpose();
    for (int row = 0; row < 6; ++row) {
      for (int column = 0; column < 6; ++column) {
        message.pose.covariance[row * 6 + column] =
            covariance(row, column);
      }
    }
    return message;
  }

  geometry_msgs::AccelWithCovarianceStamped estimateToAcceleration(
      const Estimate& estimate) const {
    geometry_msgs::AccelWithCovarianceStamped message;
    message.header.stamp = estimate.stamp;
    message.header.frame_id = odom_frame_;
    message.accel.accel.linear.x = estimate.acceleration.x();
    message.accel.accel.linear.y = estimate.acceleration.y();
    message.accel.accel.linear.z = estimate.acceleration.z();
    message.accel.covariance[0] = estimate.acceleration_variance[0];
    message.accel.covariance[7] = estimate.acceleration_variance[1];
    message.accel.covariance[14] = estimate.acceleration_variance[2];
    tf2::Quaternion orientation;
    orientation.setRPY(estimate.roll, estimate.pitch, estimate.yaw);
    const tf2::Vector3 angular_acceleration_odom = tf2::quatRotate(
        orientation,
        tf2::Vector3(latest_angular_acceleration_body_.x(),
                     latest_angular_acceleration_body_.y(),
                     latest_angular_acceleration_body_.z()));
    message.accel.accel.angular.x = angular_acceleration_odom.x();
    message.accel.accel.angular.y = angular_acceleration_odom.y();
    message.accel.accel.angular.z = angular_acceleration_odom.z();
    message.accel.covariance[21] = angular_acceleration_variance_;
    message.accel.covariance[28] = angular_acceleration_variance_;
    message.accel.covariance[35] = angular_acceleration_variance_;
    return message;
  }

  void publishMainTransform(const nav_msgs::Odometry& odometry) {
    geometry_msgs::TransformStamped transform;
    transform.header = odometry.header;
    transform.child_frame_id = odometry.child_frame_id;
    transform.transform.translation.x =
        odometry.pose.pose.position.x;
    transform.transform.translation.y =
        odometry.pose.pose.position.y;
    transform.transform.translation.z =
        odometry.pose.pose.position.z;
    transform.transform.rotation = odometry.pose.pose.orientation;
    main_transform_broadcaster_.sendTransform(transform);
  }

  void imuCallback(const sensor_msgs::Imu::ConstPtr& message) {
    tf2::Quaternion orientation;
    if (!normalizeQuaternion(message->orientation, &orientation)) {
      return;
    }
    double yaw = 0.0;
    tf2::Matrix3x3(orientation).getRPY(
        latest_roll_, latest_pitch_, yaw);
    const auto& acceleration = message->linear_acceleration;
    const auto& angular_velocity = message->angular_velocity;
    if (!std::isfinite(acceleration.x) ||
        !std::isfinite(acceleration.y) ||
        !std::isfinite(acceleration.z) ||
        !std::isfinite(angular_velocity.x) ||
        !std::isfinite(angular_velocity.y) ||
        !std::isfinite(angular_velocity.z)) {
      return;
    }
    tf2::Vector3 acceleration_odom = tf2::quatRotate(
        orientation,
        tf2::Vector3(acceleration.x, acceleration.y,
                     acceleration.z));
    if (remove_gravity_) {
      acceleration_odom.setZ(acceleration_odom.z() - gravity_);
    }
    latest_acceleration_odom_ = Eigen::Vector3d(
        acceleration_odom.x(), acceleration_odom.y(),
        acceleration_odom.z());
    if (latest_acceleration_odom_.norm() > acceleration_limit_) {
      latest_acceleration_odom_ *=
          acceleration_limit_ / latest_acceleration_odom_.norm();
    }

    const ros::Time sample_stamp =
        message->header.stamp.isZero() ? ros::Time::now()
                                       : message->header.stamp;
    const Eigen::Vector3d body_rate(
        angular_velocity.x, angular_velocity.y, angular_velocity.z);
    if (have_body_rate_sample_) {
      const double dt = (sample_stamp - last_imu_sample_stamp_).toSec();
      if (dt > 1e-4 && dt <= imu_timeout_) {
        const Eigen::Vector3d raw_angular_acceleration =
            (body_rate - latest_body_rate_) / dt;
        Eigen::Vector3d limited_angular_acceleration =
            raw_angular_acceleration;
        const double angular_acceleration_limit =
            std::max(0.0, angular_acceleration_limit_);
        if (angular_acceleration_limit > 0.0 &&
            limited_angular_acceleration.norm() >
                angular_acceleration_limit) {
          limited_angular_acceleration *=
              angular_acceleration_limit /
              limited_angular_acceleration.norm();
        }
        const double time_constant =
            std::max(0.0, angular_acceleration_time_constant_);
        const double alpha =
            time_constant > 0.0 ? dt / (time_constant + dt) : 1.0;
        latest_angular_acceleration_body_ +=
            alpha * (limited_angular_acceleration -
                     latest_angular_acceleration_body_);
      } else if (dt < 0.0 || dt > imu_timeout_) {
        latest_angular_acceleration_body_.setZero();
      }
    }
    latest_body_rate_ = body_rate;
    last_imu_sample_stamp_ = sample_stamp;
    have_body_rate_sample_ = true;

    double euler_yaw_rate = 0.0;
    const bool yaw_rate_valid = eulerYawRateFromBodyRates(
        latest_roll_, latest_pitch_, body_rate, &euler_yaw_rate);
    if (yaw_rate_valid) {
      latest_yaw_rate_ = euler_yaw_rate;
    } else {
      ROS_WARN_THROTTLE(
          1.0,
          "[xd_uav_state_estimators] IMU姿态接近欧拉yaw奇异点，"
          "本次不融合航向角速度");
    }
    last_imu_receive_ = ros::Time::now();
    have_acceleration_ = true;
    have_imu_rate_ = yaw_rate_valid;
    // 活动来源健康时，main由该来源的完整状态原子同步。此时再次把MAVROS
    // IMU融合进main既会重复计算，也会因为传输延迟让IMU时间戳落在main
    // 当前时刻之前。只有定位来源失效、main进入短时惯性外推时才直接融合。
    const bool active_localization_healthy =
        !active_source_.empty() &&
        sourceHealthy(*source_by_name_.at(active_source_),
                      ros::Time::now());
    if (!active_localization_healthy) {
      fuseImuIntoMain(sample_stamp);
    }
  }

  bool imuAccelerationFresh() const {
    return have_acceleration_ &&
           (ros::Time::now() - last_imu_receive_).toSec() <=
               imu_timeout_;
  }

  bool imuRateFresh() const {
    return have_imu_rate_ &&
           (ros::Time::now() - last_imu_receive_).toSec() <=
               imu_timeout_;
  }

  void createSourceFramePublishers(SourceRuntime* source) {
    for (const std::string& configured :
         source->config.republish_frames) {
      const std::string target = scopedFrame(configured);
      if (target.empty()) {
        continue;
      }
      FramePublisher output;
      output.target_frame = target;
      output.publisher = private_nh_.advertise<nav_msgs::Odometry>(
          "sources/" + source->config.name + "/frames/" +
              frameTopicName(target) + "/odom",
          5);
      source->frame_publishers.push_back(output);
    }
  }

  void createMainFramePublishers() {
    for (const std::string& configured : main_republish_frames_) {
      const std::string target = scopedFrame(configured);
      if (target.empty()) {
        continue;
      }
      FramePublisher output;
      output.target_frame = target;
      output.publisher = private_nh_.advertise<nav_msgs::Odometry>(
          "main/frames/" + frameTopicName(target) + "/odom", 5);
      main_frame_publishers_.push_back(output);
    }
  }

  bool transformOdometry(const nav_msgs::Odometry& input,
                         const std::string& target_frame,
                         nav_msgs::Odometry* output) {
    const std::string input_frame =
        trimSlashes(input.header.frame_id);
    if (input_frame.empty()) {
      ROS_WARN_THROTTLE(
          2.0,
          "[xd_uav_state_estimators] 无法重发布frame_id为空的Odometry");
      return false;
    }
    tf2::Transform target_input = tf2::Transform::getIdentity();
    if (target_frame != input_frame) {
      try {
        const geometry_msgs::TransformStamped transform =
            tf_buffer_.lookupTransform(
                target_frame, input_frame, ros::Time(0),
                ros::Duration(frame_lookup_timeout_));
        tf2::fromMsg(transform.transform, target_input);
      } catch (const tf2::TransformException& exception) {
        ROS_WARN_THROTTLE(
            2.0,
            "[xd_uav_state_estimators] 无法重发布到坐标系%s: %s",
            target_frame.c_str(), exception.what());
        return false;
      }
    }
    tf2::Transform input_body;
    tf2::fromMsg(input.pose.pose, input_body);
    const tf2::Transform target_body = target_input * input_body;
    *output = input;
    output->header.frame_id = target_frame;
    output->pose.pose.position.x = target_body.getOrigin().x();
    output->pose.pose.position.y = target_body.getOrigin().y();
    output->pose.pose.position.z = target_body.getOrigin().z();
    output->pose.pose.orientation =
        tf2::toMsg(target_body.getRotation());

    Eigen::Matrix<double, 6, 6> covariance;
    for (int row = 0; row < 6; ++row) {
      for (int column = 0; column < 6; ++column) {
        covariance(row, column) =
            input.pose.covariance[row * 6 + column];
      }
    }
    const tf2::Matrix3x3 basis(target_input.getRotation());
    Eigen::Matrix3d rotation;
    for (int row = 0; row < 3; ++row) {
      for (int column = 0; column < 3; ++column) {
        rotation(row, column) = basis[row][column];
      }
    }
    Eigen::Matrix<double, 6, 6> covariance_rotation =
        Eigen::Matrix<double, 6, 6>::Zero();
    covariance_rotation.block<3, 3>(0, 0) = rotation;
    covariance_rotation.block<3, 3>(3, 3) = rotation;
    covariance =
        covariance_rotation * covariance *
        covariance_rotation.transpose();
    for (int row = 0; row < 6; ++row) {
      for (int column = 0; column < 6; ++column) {
        output->pose.covariance[row * 6 + column] =
            covariance(row, column);
      }
    }
    return true;
  }

  void publishFrameOutputs(const nav_msgs::Odometry& odometry,
                           std::vector<FramePublisher>* publishers) {
    for (FramePublisher& publisher : *publishers) {
      nav_msgs::Odometry transformed;
      if (transformOdometry(odometry, publisher.target_frame,
                            &transformed)) {
        publisher.publisher.publish(transformed);
      }
    }
  }

  void outputTimerCallback(const ros::TimerEvent&) {
    const ros::Time now = ros::Time::now();
    updateSourceConnectionStates(now);
    updateSelection(now);
    for (auto& source : sources_) {
      const bool valid = sourceHealthy(*source, now);
      publishBoolean(source->valid_publisher, valid);
      Estimate estimate;
      if (estimateSourceAt(*source, now, &estimate)) {
        const nav_msgs::Odometry odometry =
            sourceEstimateToOdometry(*source, estimate);
        source->odometry_publisher.publish(odometry);
        publishFrameOutputs(odometry, &source->frame_publishers);
      }
    }

    const bool localization_valid =
        !active_source_.empty() &&
        sourceHealthy(*source_by_name_.at(active_source_), now);
    if (localization_valid) {
      Estimate source_estimate;
      if (estimateSourceAt(*source_by_name_.at(active_source_), now,
                           &source_estimate)) {
        // main不再二次融合活动来源的各条修正。这里整包接管该来源已经
        // 完成滤波的状态，handover只负责保持控制坐标连续。
        initializeMainFromEstimate(applyMainHandover(source_estimate));
      }
    }
    bool state_valid = false;
    Estimate main;
    if (currentMainEstimate(now, &main)) {
      const double age = sourceLocalizationAge(
          *source_by_name_.at(active_source_), now);
      state_valid =
          localization_valid || age <= max_dead_reckoning_time_;
      if (require_imu_ && !imuRateFresh()) {
        state_valid = false;
      }
      if (state_valid) {
        const nav_msgs::Odometry main_odometry =
            estimateToOdometry(main);
        main_odometry_publisher_.publish(main_odometry);
        main_acceleration_publisher_.publish(
            estimateToAcceleration(main));
        publishMainTransform(main_odometry);
        publishFrameOutputs(main_odometry,
                            &main_frame_publishers_);
      }
    }
    publishBoolean(localization_valid_publisher_,
                   localization_valid);
    publishBoolean(state_valid_publisher_, state_valid);
    publishStatus(now, localization_valid, state_valid);
  }

  void publishAlignment(const SourceRuntime& source,
                        const ros::Time& stamp) {
    if (!source.alignment_initialized ||
        source.raw_frame.empty() ||
        source.raw_frame == odom_frame_) {
      return;
    }
    geometry_msgs::TransformStamped message;
    message.header.stamp = stamp;
    message.header.frame_id = odom_frame_;
    message.child_frame_id = source.raw_frame;
    message.transform.translation.x =
        source.alignment_translation.x();
    message.transform.translation.y =
        source.alignment_translation.y();
    message.transform.translation.z =
        source.alignment_translation.z();
    tf2::Quaternion rotation;
    rotation.setRPY(0.0, 0.0, source.alignment_yaw);
    message.transform.rotation = tf2::toMsg(rotation);
    source.alignment_publisher.publish(message);
  }

  void publishStatus(const ros::Time& now,
                     const bool localization_valid,
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
      message.state =
          xd_uav_state_estimators::EstimatorStatus::WAITING;
      message.state_name = "WAITING";
      message.failure_reason = "没有健康定位源";
    } else {
      const double age = sourceLocalizationAge(
          *source_by_name_.at(active_source_), now);
      message.localization_age = age;
      message.dead_reckoning_age =
          localization_valid ? 0.0 : age;
      if (localization_valid && state_valid) {
        message.state =
            xd_uav_state_estimators::EstimatorStatus::RUNNING;
        message.state_name = "RUNNING";
      } else if (state_valid) {
        message.state =
            xd_uav_state_estimators::EstimatorStatus::DEAD_RECKONING;
        message.state_name = "DEAD_RECKONING";
        message.failure_reason = "定位修正超时，正在短时预测";
      } else {
        message.state =
            xd_uav_state_estimators::EstimatorStatus::LOST;
        message.state_name = "LOST";
        message.failure_reason = "没有可安全用于控制的状态";
      }
    }
    status_publisher_.publish(message);
  }

  void diagnosticsTimerCallback(const ros::TimerEvent&) {
    const ros::Time now = ros::Time::now();
    diagnostic_msgs::DiagnosticArray array;
    array.header.stamp = now;
    for (const auto& source : sources_) {
      diagnostic_msgs::DiagnosticStatus source_status;
      source_status.name =
          uav_name_ + "/state_estimator/source/" +
          source->config.name;
      source_status.hardware_id = source->config.name;
      const bool healthy = sourceHealthy(*source, now);
      source_status.level =
          healthy ? diagnostic_msgs::DiagnosticStatus::OK
                  : diagnostic_msgs::DiagnosticStatus::WARN;
      source_status.message =
          healthy ? "健康" : "等待或存在必需修正异常";
      addDiagnostic(
          &source_status, "active",
          source->config.name == active_source_ ? "true" : "false");
      addDiagnostic(&source_status, "priority",
                    std::to_string(source->config.priority));
      addDiagnostic(
          &source_status, "alignment_translation",
          std::to_string(source->alignment_translation.x()) + "," +
              std::to_string(source->alignment_translation.y()) + "," +
              std::to_string(source->alignment_translation.z()));
      addDiagnostic(&source_status, "alignment_yaw",
                    std::to_string(source->alignment_yaw));
      addDiagnostic(
          &source_status, "alignment_initialized",
          source->alignment_initialized ? "true" : "false");
      array.status.push_back(source_status);

      for (const auto& correction : source->corrections) {
        diagnostic_msgs::DiagnosticStatus status;
        status.name =
            uav_name_ + "/state_estimator/correction/" +
            source->config.name + "/" + correction->config.name;
        status.hardware_id = source->config.name;
        const bool correction_healthy =
            correctionHealthy(*correction, now);
        status.level =
            correction_healthy
                ? diagnostic_msgs::DiagnosticStatus::OK
                : (correction->config.required
                       ? diagnostic_msgs::DiagnosticStatus::ERROR
                       : diagnostic_msgs::DiagnosticStatus::WARN);
        status.message =
            correction->recovering
                ? "恢复观察中"
                : (now < correction->quarantine_until
                       ? "已隔离"
                       : (correction_healthy ? "健康"
                                             : "超时或等待"));
        addDiagnostic(&status, "type",
                      correctionKindName(correction->config.kind));
        addDiagnostic(&status, "topic",
                      nh_.resolveName(correction->config.topic));
        addDiagnostic(&status, "required",
                      correction->config.required ? "true" : "false");
        addDiagnostic(&status, "received",
                      std::to_string(correction->received));
        addDiagnostic(&status, "accepted",
                      std::to_string(correction->accepted));
        addDiagnostic(&status, "rejected",
                      std::to_string(correction->rejected));
        addDiagnostic(
            &status, "age",
            correction->last_accepted.isZero()
                ? "-1"
                : std::to_string(
                      (now - correction->last_accepted).toSec()));
        array.status.push_back(status);
      }
    }
    diagnostics_publisher_.publish(array);
  }

  static void addDiagnostic(
      diagnostic_msgs::DiagnosticStatus* status,
      const std::string& key, const std::string& value) {
    diagnostic_msgs::KeyValue entry;
    entry.key = key;
    entry.value = value;
    status->values.push_back(entry);
  }

  static void publishBoolean(const ros::Publisher& publisher,
                             const bool value) {
    std_msgs::Bool message;
    message.data = value;
    publisher.publish(message);
  }

  bool resetCallback(std_srvs::Trigger::Request&,
                     std_srvs::Trigger::Response& response) {
    for (auto& source : sources_) {
      source->alignment_initialized = false;
      source->filters_initialized = false;
      source->raw = RawState();
      source->fused_raw = RawState();
      source->raw_frame.clear();
      source->required_corrections_were_connected = false;
      source->session++;
      if (source->session == 0) {
        source->session = 1;
        for (auto& correction : source->corrections) {
          correction->valid_session = 0;
        }
      }
      for (auto& correction : source->corrections) {
        correction->last_received = ros::Time();
        correction->last_accepted = ros::Time();
        correction->quarantine_until = ros::Time();
        correction->recovering = false;
        correction->consecutive_rejections = 0;
        correction->recovery_samples = 0;
      }
    }
    active_source_.clear();
    main_handover_translation_.setZero();
    main_handover_yaw_ = 0.0;
    main_handover_start_time_ = ros::Time();
    main_handover_rapid_recovery_ = false;
    main_state_->initialized = false;
    main_state_->filter_stamp = ros::Time();
    last_main_imu_fusion_stamp_ = ros::Time();
    response.success = true;
    response.message = "全部来源滤波器已重置";
    return true;
  }

  bool switchSourceCallback(
      xd_uav_state_estimators::SwitchLocalizationSource::Request&
          request,
      xd_uav_state_estimators::SwitchLocalizationSource::Response&
          response) {
    const std::string requested = request.source_name;
    if (requested.empty() || requested == "auto") {
      automatic_selection_ = true;
      requested_source_.clear();
      updateSelection(ros::Time::now());
      response.success = true;
      response.message = "已启用自动定位源选择";
    } else {
      const auto iterator = source_by_name_.find(requested);
      if (iterator == source_by_name_.end()) {
        response.success = false;
        response.message = "未知定位源: " + requested;
      } else if (!sourceHealthy(*iterator->second,
                                ros::Time::now())) {
        response.success = false;
        response.message = "定位源当前不健康: " + requested + "（" +
                           sourceHealthReason(*iterator->second,
                                              ros::Time::now()) +
                           "）";
      } else {
        automatic_selection_ = false;
        requested_source_ = requested;
        activateSource(iterator->second, ros::Time::now());
        response.success = true;
        response.message = "已指定定位源: " + requested;
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
  tf2_ros::TransformBroadcaster main_transform_broadcaster_;
  ros::Subscriber imu_subscriber_;
  ros::Publisher main_odometry_publisher_;
  ros::Publisher main_acceleration_publisher_;
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
  std::unique_ptr<MainRuntime> main_state_;
  std::vector<FramePublisher> main_frame_publishers_;
  std::vector<std::string> main_republish_frames_;
  AxisKalman::Parameters axis_parameters_;
  std::string uav_name_;
  std::string odom_frame_;
  std::string body_frame_;
  std::string active_source_;
  std::string requested_source_;
  ros::Time active_since_;
  ros::Time last_switch_time_;
  ros::Time last_imu_receive_;
  ros::Time last_imu_sample_stamp_;
  ros::Time last_main_imu_fusion_stamp_;
  ros::Time main_handover_start_time_;
  Eigen::Vector3d latest_acceleration_odom_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d main_handover_translation_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d latest_body_rate_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d latest_angular_acceleration_body_{
      Eigen::Vector3d::Zero()};
  double latest_yaw_rate_{0.0};
  double main_handover_yaw_{0.0};
  double latest_roll_{0.0};
  double latest_pitch_{0.0};
  double output_rate_{100.0};
  double diagnostics_rate_{2.0};
  double max_localization_delay_{0.15};
  double max_dead_reckoning_time_{1.0};
  double imu_timeout_{0.3};
  double yaw_process_noise_{0.02};
  double yaw_rate_process_noise_{0.2};
  double imu_acceleration_variance_{0.5};
  double imu_yaw_rate_variance_{0.05};
  double angular_acceleration_time_constant_{0.05};
  double angular_acceleration_variance_{1.0};
  double angular_acceleration_limit_{100.0};
  double gravity_{9.80665};
  double acceleration_limit_{30.0};
  double position_xy_limit_{5.0};
  double position_z_limit_{3.0};
  double velocity_xy_limit_{5.0};
  double velocity_z_limit_{3.0};
  double heading_limit_{1.57};
  double yaw_rate_limit_{3.0};
  double position_xy_nis_limit_{9.21};
  double position_z_nis_limit_{6.63};
  double velocity_xy_nis_limit_{9.21};
  double velocity_z_nis_limit_{6.63};
  double heading_nis_limit_{6.63};
  double yaw_rate_nis_limit_{6.63};
  double minimum_active_time_{1.0};
  double switch_cooldown_{0.5};
  double frame_lookup_timeout_{0.03};
  bool require_imu_{false};
  bool remove_gravity_{true};
  bool use_nis_{true};
  bool switch_back_to_higher_priority_{true};
  bool main_handover_rapid_recovery_{false};
  bool automatic_selection_{true};
  bool have_acceleration_{false};
  bool have_imu_rate_{false};
  bool have_body_rate_sample_{false};
  std::uint64_t switch_count_{0};
};

int main(int argc, char** argv) {
  // rosconsole底层使用log4cxx；显式启用UTF-8，避免中文日志被转换成问号。
  std::setlocale(LC_ALL, "C.UTF-8");
  ros::init(argc, argv, "state_estimator");
  try {
    MultiSourceEstimatorNode node;
    ros::spin();
  } catch (const std::exception& exception) {
    ROS_FATAL("[xd_uav_state_estimators] 启动失败: %s",
              exception.what());
    return 1;
  }
  return 0;
}
