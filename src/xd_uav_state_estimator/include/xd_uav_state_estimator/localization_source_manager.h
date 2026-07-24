#pragma once

#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Dense>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace xd_uav_state_estimator {

enum class SourceRole {
  kPrimary,
  kFallback,
};

enum class AlignmentMode {
  kIdentity,
  kAlignOnFirstMeasurement,
  kAlignOnActivation,
};

enum class SourceLifecycleState {
  kDisabled,
  kWaiting,
  kHealthy,
  kActive,
  kSuspect,
  kQuarantined,
  kRecovering,
};

struct LocalizationSourceDefaults {
  double timeout{0.25};
  bool twist_in_body_frame{true};
  double position_xy_variance{0.01};
  double position_z_variance{0.05};
  double velocity_xy_variance{0.01};
  double velocity_z_variance{0.02};
};

struct LocalizationSourceConfig {
  std::string name;
  std::string adapter{"odometry"};
  std::string topic;
  bool enabled{true};
  SourceRole role{SourceRole::kPrimary};
  int priority{0};
  bool use_position_xy{true};
  bool use_altitude{true};
  bool use_velocity_xy{true};
  bool use_vertical_velocity{true};
  bool use_heading{true};
  bool twist_in_body_frame{true};
  bool use_message_covariance{true};
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
  int alignment_samples{10};
  bool use_tf_for_child_frame{true};
  bool require_child_frame_tf{true};
  double tf_lookup_timeout{0.02};
  AlignmentMode alignment_mode{AlignmentMode::kIdentity};
};

struct LocalizationMeasurement {
  LocalizationSourceConfig config;
  ros::Time stamp;
  ros::Time receive_time;
  std::string parent_frame;
  std::string child_frame;
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
  geometry_msgs::Quaternion orientation;
  geometry_msgs::Vector3 angular_velocity;
  double position_xy_variance{0.01};
  double position_z_variance{0.05};
  double velocity_xy_variance{0.01};
  double velocity_z_variance{0.02};
  double heading_variance{0.02};
};

struct LocalizationDecision {
  bool accepted{false};
  bool active_source_changed{false};
  bool authoritative{false};
  std::string active_source;
  std::string reason;
};

struct LocalizationSourceStatus {
  LocalizationSourceConfig config;
  bool healthy{false};
  bool reliable{true};
  bool quarantined{false};
  bool active{false};
  double age{-1.0};
  ros::Time last_message_stamp;
  std::uint64_t received{0};
  std::uint64_t rejected{0};
  std::uint64_t accepted_corrections{0};
  std::uint64_t rejected_corrections{0};
  int consecutive_rejections{0};
  int recovery_samples{0};
  SourceLifecycleState lifecycle_state{SourceLifecycleState::kWaiting};
};

class LocalizationSourceAdapter {
public:
  using MeasurementCallback = std::function<void(const LocalizationMeasurement&)>;

  virtual ~LocalizationSourceAdapter() = default;
  virtual LocalizationSourceStatus status(const ros::Time& now) const = 0;
};

class OdometryAdapter final : public LocalizationSourceAdapter {
public:
  OdometryAdapter(ros::NodeHandle node_handle, LocalizationSourceConfig config,
                  std::string target_body_frame, tf2_ros::Buffer* tf_buffer,
                  MeasurementCallback callback);

  LocalizationSourceStatus status(const ros::Time& now) const override;

private:
  void callback(const nav_msgs::Odometry::ConstPtr& message);

  ros::NodeHandle node_handle_;
  LocalizationSourceConfig config_;
  std::string target_body_frame_;
  tf2_ros::Buffer* tf_buffer_{nullptr};
  MeasurementCallback measurement_callback_;
  ros::Subscriber subscriber_;
  ros::Time last_valid_receive_time_;
  ros::Time last_message_stamp_;
  std::uint64_t received_{0};
  std::uint64_t rejected_{0};
};

class LocalizationSourceManager {
public:
  using MeasurementCallback =
      std::function<void(const LocalizationMeasurement&, const LocalizationDecision&)>;

  LocalizationSourceManager(ros::NodeHandle node_handle,
                            ros::NodeHandle private_node_handle,
                            const LocalizationSourceDefaults& defaults,
                            MeasurementCallback callback);

  std::vector<LocalizationSourceStatus> statuses(const ros::Time& now) const;
  bool requestActiveSource(const std::string& source_name, const ros::Time& now,
                           std::string* message);
  void reportCorrectionResult(const std::string& source_name, bool accepted,
                              const ros::Time& now);
  bool localizationValid(const ros::Time& now) const;
  std::string activeSource() const;
  std::string requestedSource() const;
  bool automaticSelection() const;
  std::size_t enabledSourceCount() const;
  std::uint64_t switchCount() const;
  const std::vector<LocalizationSourceConfig>& configs() const;

private:
  void loadSources(const LocalizationSourceDefaults& defaults);
  void adapterCallback(const LocalizationMeasurement& measurement);
  LocalizationDecision decide(const LocalizationMeasurement& measurement,
                              const ros::Time& now);
  std::string selectHealthyPrimaryOrFallback(const ros::Time& now) const;
  bool sourceUsable(const std::string& name, const ros::Time& now) const;
  void updateRecovery(const std::string& name, const ros::Time& now);
  void setActiveSource(const std::string& name, const ros::Time& now);

  static SourceRole parseRole(const std::string& value);
  static AlignmentMode parseAlignmentMode(const std::string& value);

  ros::NodeHandle node_handle_;
  ros::NodeHandle private_node_handle_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::string body_frame_;
  MeasurementCallback measurement_callback_;
  bool switch_back_to_primary_{true};
  double minimum_active_time_{1.0};
  double switch_cooldown_{0.5};
  std::vector<LocalizationSourceConfig> configs_;
  std::vector<std::unique_ptr<LocalizationSourceAdapter>> adapters_;
  std::unordered_map<std::string, LocalizationSourceAdapter*> adapter_by_name_;
  struct ReliabilityState {
    int consecutive_rejections{0};
    ros::Time quarantine_until;
    ros::Time recovery_started;
    std::uint64_t accepted_corrections{0};
    std::uint64_t rejected_corrections{0};
    int recovery_samples{0};
    bool recovering{false};
  };
  std::unordered_map<std::string, ReliabilityState> reliability_by_name_;
  std::string active_source_;
  std::string requested_source_;
  bool selection_change_requested_{false};
  ros::Time active_since_;
  ros::Time last_switch_time_;
  std::uint64_t switch_count_{0};
};

std::string sourceRoleName(SourceRole role);
std::string alignmentModeName(AlignmentMode mode);
std::string sourceLifecycleStateName(SourceLifecycleState state);

}  // 命名空间 xd_uav_state_estimator
