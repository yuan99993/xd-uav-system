#include <xd_uav_state_estimator/localization_source_manager.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace xd_uav_state_estimator {

namespace {

double nonNegativeAge(const ros::Time& now, const ros::Time& stamp) {
  if (stamp.isZero() || now < stamp) {
    return 0.0;
  }
  return (now - stamp).toSec();
}

std::string lowerCase(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(),
                 [](const unsigned char character) {
                   return static_cast<char>(std::tolower(character));
                 });
  return value;
}

bool finite(const geometry_msgs::Point& value) {
  return std::isfinite(value.x) && std::isfinite(value.y) &&
         std::isfinite(value.z);
}

bool finite(const geometry_msgs::Vector3& value) {
  return std::isfinite(value.x) && std::isfinite(value.y) &&
         std::isfinite(value.z);
}

bool normalizeQuaternion(const geometry_msgs::Quaternion& message,
                         tf2::Quaternion* quaternion) {
  if (!std::isfinite(message.x) || !std::isfinite(message.y) ||
      !std::isfinite(message.z) || !std::isfinite(message.w)) {
    return false;
  }
  quaternion->setValue(message.x, message.y, message.z, message.w);
  if (!std::isfinite(quaternion->length2()) || quaternion->length2() < 1e-12) {
    return false;
  }
  quaternion->normalize();
  return true;
}

double covarianceValue(const boost::array<double, 36>& covariance,
                       const std::size_t index, const double fallback) {
  const double value = covariance[index];
  return std::isfinite(value) && value > 0.0 ? value : fallback;
}

int roleRank(const SourceRole role) {
  switch (role) {
    case SourceRole::kPrimary:
      return 0;
    case SourceRole::kFallback:
      return 1;
  }
  return 2;
}

}  // 匿名命名空间

OdometryAdapter::OdometryAdapter(ros::NodeHandle node_handle,
                                 LocalizationSourceConfig config,
                                 std::string target_body_frame,
                                 tf2_ros::Buffer* tf_buffer,
                                 MeasurementCallback callback)
    : node_handle_(std::move(node_handle)),
      config_(std::move(config)),
      target_body_frame_(std::move(target_body_frame)),
      tf_buffer_(tf_buffer),
      measurement_callback_(std::move(callback)) {
  ros::TransportHints transport_hints;
  transport_hints.tcpNoDelay();
  subscriber_ = node_handle_.subscribe(config_.topic, 50,
                                       &OdometryAdapter::callback, this,
                                       transport_hints);
}

void OdometryAdapter::callback(const nav_msgs::Odometry::ConstPtr& message) {
  ++received_;

  tf2::Quaternion orientation;
  if (!finite(message->pose.pose.position) ||
      !finite(message->twist.twist.linear) ||
      !finite(message->twist.twist.angular) ||
      !normalizeQuaternion(message->pose.pose.orientation, &orientation)) {
    ++rejected_;
    ROS_WARN_THROTTLE(1.0,
                      "[xd_uav_state_estimator] source '%s' rejected odometry with NaN/Inf or an invalid quaternion",
                      config_.name.c_str());
    return;
  }

  const ros::Time measurement_stamp = message->header.stamp.isZero()
                                          ? ros::Time::now()
                                          : message->header.stamp;
  tf2::Transform parent_to_child;
  tf2::fromMsg(message->pose.pose, parent_to_child);
  parent_to_child.setRotation(orientation);
  tf2::Transform child_to_body;
  child_to_body.setIdentity();
  const std::string message_child_frame = message->child_frame_id;
  const bool child_frame_matches = !message_child_frame.empty() &&
                                   message_child_frame == target_body_frame_;
  if (!child_frame_matches) {
    if (message_child_frame.empty() && config_.require_child_frame_tf) {
      ++rejected_;
      ROS_WARN_THROTTLE(1.0,
                        "[xd_uav_state_estimator] source '%s' has an empty child frame",
                        config_.name.c_str());
      return;
    }
    if (config_.use_tf_for_child_frame && !message_child_frame.empty()) {
      if (tf_buffer_ == nullptr) {
        ++rejected_;
        return;
      }
      try {
        const geometry_msgs::TransformStamped child_to_body_message =
            tf_buffer_->lookupTransform(
                message_child_frame, target_body_frame_, measurement_stamp,
                ros::Duration(config_.tf_lookup_timeout));
        tf2::fromMsg(child_to_body_message.transform, child_to_body);
      } catch (const tf2::TransformException& exception) {
        if (config_.require_child_frame_tf) {
          ++rejected_;
          ROS_WARN_THROTTLE(
              1.0,
              "[xd_uav_state_estimator] source '%s' cannot transform child frame '%s' to '%s': %s",
              config_.name.c_str(), message_child_frame.c_str(),
              target_body_frame_.c_str(), exception.what());
          return;
        }
        ROS_WARN_THROTTLE(
            2.0,
            "[xd_uav_state_estimator] source '%s' is treating child frame '%s' as body frame '%s'",
            config_.name.c_str(), message_child_frame.c_str(),
            target_body_frame_.c_str());
      }
    } else if (config_.require_child_frame_tf) {
      ++rejected_;
      ROS_WARN_THROTTLE(
          1.0,
          "[xd_uav_state_estimator] source '%s' child frame '%s' does not match '%s' and TF conversion is disabled",
          config_.name.c_str(), message_child_frame.c_str(),
          target_body_frame_.c_str());
      return;
    }
  }

  const tf2::Transform parent_to_body = parent_to_child * child_to_body;
  const tf2::Quaternion body_orientation = parent_to_body.getRotation();
  const tf2::Vector3 body_position = parent_to_body.getOrigin();
  const tf2::Vector3 linear_input(message->twist.twist.linear.x,
                                  message->twist.twist.linear.y,
                                  message->twist.twist.linear.z);
  const tf2::Vector3 angular_input(message->twist.twist.angular.x,
                                   message->twist.twist.angular.y,
                                   message->twist.twist.angular.z);
  tf2::Vector3 velocity_parent;
  tf2::Vector3 angular_velocity_body;
  if (config_.twist_in_body_frame) {
    const tf2::Vector3 velocity_body_in_child =
        linear_input + angular_input.cross(child_to_body.getOrigin());
    const tf2::Quaternion body_to_child_rotation =
        child_to_body.getRotation().inverse();
    const tf2::Vector3 velocity_body =
        tf2::quatRotate(body_to_child_rotation, velocity_body_in_child);
    angular_velocity_body =
        tf2::quatRotate(body_to_child_rotation, angular_input);
    velocity_parent = tf2::quatRotate(body_orientation, velocity_body);
  } else {
    const tf2::Vector3 lever_arm_parent =
        tf2::quatRotate(parent_to_child.getRotation(),
                        child_to_body.getOrigin());
    velocity_parent = linear_input + angular_input.cross(lever_arm_parent);
    angular_velocity_body =
        tf2::quatRotate(body_orientation.inverse(), angular_input);
  }

  LocalizationMeasurement measurement;
  measurement.config = config_;
  measurement.stamp = measurement_stamp;
  measurement.receive_time = ros::Time::now();
  measurement.parent_frame = message->header.frame_id;
  measurement.child_frame = target_body_frame_;
  measurement.position = Eigen::Vector3d(body_position.x(), body_position.y(),
                                         body_position.z());
  measurement.velocity = Eigen::Vector3d(velocity_parent.x(), velocity_parent.y(),
                                         velocity_parent.z());
  measurement.orientation = tf2::toMsg(body_orientation);
  measurement.angular_velocity.x = angular_velocity_body.x();
  measurement.angular_velocity.y = angular_velocity_body.y();
  measurement.angular_velocity.z = angular_velocity_body.z();

  measurement.position_xy_variance = config_.position_xy_variance;
  measurement.position_z_variance = config_.position_z_variance;
  measurement.velocity_xy_variance = config_.velocity_xy_variance;
  measurement.velocity_z_variance = config_.velocity_z_variance;
  measurement.heading_variance = config_.heading_variance;
  if (config_.use_message_covariance) {
    measurement.position_xy_variance = std::max(
        covarianceValue(message->pose.covariance, 0, config_.position_xy_variance),
        covarianceValue(message->pose.covariance, 7, config_.position_xy_variance));
    measurement.position_z_variance =
        covarianceValue(message->pose.covariance, 14, config_.position_z_variance);
    measurement.velocity_xy_variance = std::max(
        covarianceValue(message->twist.covariance, 0, config_.velocity_xy_variance),
        covarianceValue(message->twist.covariance, 7, config_.velocity_xy_variance));
    measurement.velocity_z_variance =
        covarianceValue(message->twist.covariance, 14, config_.velocity_z_variance);
    measurement.heading_variance =
        covarianceValue(message->pose.covariance, 35, config_.heading_variance);
  }

  const bool position_covariance_invalid =
      (config_.use_position_xy &&
       measurement.position_xy_variance > config_.max_position_variance) ||
      (config_.use_altitude &&
       measurement.position_z_variance > config_.max_position_variance);
  const bool velocity_covariance_invalid =
      (config_.use_velocity_xy &&
       measurement.velocity_xy_variance > config_.max_velocity_variance) ||
      (config_.use_vertical_velocity &&
       measurement.velocity_z_variance > config_.max_velocity_variance);
  const bool heading_covariance_invalid =
      config_.use_heading &&
      measurement.heading_variance > config_.max_heading_variance;
  if (position_covariance_invalid || velocity_covariance_invalid ||
      heading_covariance_invalid) {
    ++rejected_;
    ROS_WARN_THROTTLE(1.0,
                      "[xd_uav_state_estimator] source '%s' covariance exceeds configured limits",
                      config_.name.c_str());
    return;
  }

  last_valid_receive_time_ = measurement.receive_time;
  last_message_stamp_ = measurement.stamp;
  measurement_callback_(measurement);
}

LocalizationSourceStatus OdometryAdapter::status(const ros::Time& now) const {
  LocalizationSourceStatus result;
  result.config = config_;
  result.age = last_valid_receive_time_.isZero()
                   ? -1.0
                   : nonNegativeAge(now, last_valid_receive_time_);
  result.healthy = config_.enabled && result.age >= 0.0 &&
                   result.age <= config_.timeout;
  result.last_message_stamp = last_message_stamp_;
  result.received = received_;
  result.rejected = rejected_;
  return result;
}

LocalizationSourceManager::LocalizationSourceManager(
    ros::NodeHandle node_handle, ros::NodeHandle private_node_handle,
    const LocalizationSourceDefaults& defaults, MeasurementCallback callback)
    : node_handle_(std::move(node_handle)),
      private_node_handle_(std::move(private_node_handle)),
      tf_listener_(tf_buffer_),
      measurement_callback_(std::move(callback)) {
  private_node_handle_.param<std::string>("body_frame", body_frame_, "base_link");
  private_node_handle_.param("switch_back_to_primary", switch_back_to_primary_,
                             switch_back_to_primary_);
  private_node_handle_.param("source_switching/minimum_active_time",
                             minimum_active_time_, minimum_active_time_);
  private_node_handle_.param("source_switching/cooldown", switch_cooldown_,
                             switch_cooldown_);
  minimum_active_time_ = std::max(minimum_active_time_, 0.0);
  switch_cooldown_ = std::max(switch_cooldown_, 0.0);
  loadSources(defaults);

  for (auto& config : configs_) {
    reliability_by_name_.emplace(config.name, ReliabilityState{});
    if (!config.enabled) {
      continue;
    }
    if (config.adapter != "odometry") {
      ROS_ERROR("[xd_uav_state_estimator] source '%s' requested unsupported adapter '%s'",
                config.name.c_str(), config.adapter.c_str());
      config.enabled = false;
      continue;
    }

    auto adapter = std::make_unique<OdometryAdapter>(
        node_handle_, config, body_frame_, &tf_buffer_,
        [this](const LocalizationMeasurement& measurement) {
          adapterCallback(measurement);
        });
    adapter_by_name_[config.name] = adapter.get();
    adapters_.push_back(std::move(adapter));
    ROS_INFO("[xd_uav_state_estimator] loaded source '%s': topic='%s', role='%s'",
             config.name.c_str(), config.topic.c_str(),
             sourceRoleName(config.role).c_str());
  }

  if (adapters_.empty()) {
    throw std::runtime_error("no enabled localization source; check localization_sources");
  }
}

void LocalizationSourceManager::loadSources(
    const LocalizationSourceDefaults& defaults) {
  std::vector<std::string> source_names;
  if (!private_node_handle_.getParam("localization_sources", source_names) ||
      source_names.empty()) {
    source_names = {"mavros_local"};
    ROS_WARN("[xd_uav_state_estimator] localization_sources is missing; using legacy mavros_local default");
  }

  for (const std::string& name : source_names) {
    if (name.empty()) {
      throw std::invalid_argument("localization source name must not be empty");
    }
    if (std::any_of(configs_.begin(), configs_.end(),
                    [&name](const LocalizationSourceConfig& config) {
                      return config.name == name;
                    })) {
      throw std::invalid_argument("duplicate localization source name: " + name);
    }

    ros::NodeHandle source_node(private_node_handle_, "sources/" + name);
    LocalizationSourceConfig config;
    config.name = name;
    config.topic = name == "mavros_local" ? "mavros/local_position/odom"
                                           : name + "/odom";
    config.timeout = defaults.timeout;
    config.twist_in_body_frame = defaults.twist_in_body_frame;
    config.position_xy_variance = defaults.position_xy_variance;
    config.position_z_variance = defaults.position_z_variance;
    config.velocity_xy_variance = defaults.velocity_xy_variance;
    config.velocity_z_variance = defaults.velocity_z_variance;

    source_node.param("enabled", config.enabled, config.enabled);
    source_node.param("adapter", config.adapter, config.adapter);
    source_node.param("topic", config.topic, config.topic);
    source_node.param("priority", config.priority, config.priority);
    bool legacy_use_position = true;
    bool legacy_use_velocity = true;
    source_node.param("use_position", legacy_use_position, legacy_use_position);
    source_node.param("use_velocity", legacy_use_velocity, legacy_use_velocity);
    config.use_position_xy = legacy_use_position;
    config.use_altitude = legacy_use_position;
    config.use_velocity_xy = legacy_use_velocity;
    config.use_vertical_velocity = legacy_use_velocity;
    source_node.param("use_position_xy", config.use_position_xy,
                      config.use_position_xy);
    source_node.param("use_altitude", config.use_altitude, config.use_altitude);
    source_node.param("use_velocity_xy", config.use_velocity_xy,
                      config.use_velocity_xy);
    source_node.param("use_vertical_velocity", config.use_vertical_velocity,
                      config.use_vertical_velocity);
    source_node.param("use_heading", config.use_heading, config.use_heading);
    source_node.param("twist_in_body_frame", config.twist_in_body_frame,
                      config.twist_in_body_frame);
    source_node.param("timeout", config.timeout, config.timeout);
    source_node.param("covariance/use_message", config.use_message_covariance,
                      config.use_message_covariance);
    source_node.param("covariance/position_xy", config.position_xy_variance,
                      config.position_xy_variance);
    source_node.param("covariance/position_z", config.position_z_variance,
                      config.position_z_variance);
    source_node.param("covariance/velocity_xy", config.velocity_xy_variance,
                      config.velocity_xy_variance);
    source_node.param("covariance/velocity_z", config.velocity_z_variance,
                      config.velocity_z_variance);
    source_node.param("covariance/heading", config.heading_variance,
                      config.heading_variance);
    source_node.param("reliability/max_position_variance",
                      config.max_position_variance, config.max_position_variance);
    source_node.param("reliability/max_velocity_variance",
                      config.max_velocity_variance, config.max_velocity_variance);
    source_node.param("reliability/max_heading_variance",
                      config.max_heading_variance, config.max_heading_variance);
    source_node.param("reliability/max_consecutive_rejections",
                      config.max_consecutive_rejections,
                      config.max_consecutive_rejections);
    source_node.param("reliability/quarantine_duration",
                      config.quarantine_duration, config.quarantine_duration);
    source_node.param("reliability/recovery_min_samples",
                      config.recovery_min_samples, config.recovery_min_samples);
    source_node.param("reliability/recovery_stable_time",
                      config.recovery_stable_time, config.recovery_stable_time);
    source_node.param("alignment/samples", config.alignment_samples,
                      config.alignment_samples);
    source_node.param("frame_transform/use_tf_for_child_frame",
                      config.use_tf_for_child_frame,
                      config.use_tf_for_child_frame);
    source_node.param("frame_transform/require_tf",
                      config.require_child_frame_tf,
                      config.require_child_frame_tf);
    source_node.param("frame_transform/timeout",
                      config.tf_lookup_timeout, config.tf_lookup_timeout);

    std::string role = "primary";
    std::string alignment = "identity";
    source_node.param("role", role, role);
    source_node.param("alignment/mode", alignment, alignment);
    config.role = parseRole(role);
    config.alignment_mode = parseAlignmentMode(alignment);

    config.adapter = lowerCase(config.adapter);
    if (config.adapter == "nav_msgs/odometry") {
      config.adapter = "odometry";
    }
    config.timeout = std::max(config.timeout, 0.01);
    config.position_xy_variance = std::max(config.position_xy_variance, 1e-12);
    config.position_z_variance = std::max(config.position_z_variance, 1e-12);
    config.velocity_xy_variance = std::max(config.velocity_xy_variance, 1e-12);
    config.velocity_z_variance = std::max(config.velocity_z_variance, 1e-12);
    config.heading_variance = std::max(config.heading_variance, 1e-12);
    config.max_position_variance = std::max(config.max_position_variance, 1e-12);
    config.max_velocity_variance = std::max(config.max_velocity_variance, 1e-12);
    config.max_heading_variance = std::max(config.max_heading_variance, 1e-12);
    config.max_consecutive_rejections =
        std::max(config.max_consecutive_rejections, 1);
    config.quarantine_duration = std::max(config.quarantine_duration, 0.1);
    config.recovery_min_samples = std::max(config.recovery_min_samples, 1);
    config.recovery_stable_time = std::max(config.recovery_stable_time, 0.0);
    config.alignment_samples = std::max(config.alignment_samples, 1);
    config.tf_lookup_timeout = std::max(config.tf_lookup_timeout, 0.0);
    if (config.topic.empty()) {
      throw std::invalid_argument("localization source '" + name + "' has an empty topic");
    }
    if (!config.use_position_xy && !config.use_altitude &&
        !config.use_velocity_xy && !config.use_vertical_velocity &&
        !config.use_heading) {
      ROS_WARN("[xd_uav_state_estimator] source '%s' uses neither position nor velocity and cannot correct the state",
               name.c_str());
    }
    configs_.push_back(config);
  }
}

void LocalizationSourceManager::adapterCallback(
    const LocalizationMeasurement& measurement) {
  const ros::Time now = ros::Time::now();
  updateRecovery(measurement.config.name, now);
  const LocalizationDecision decision = decide(measurement, now);
  measurement_callback_(measurement, decision);
}

LocalizationDecision LocalizationSourceManager::decide(
    const LocalizationMeasurement& measurement, const ros::Time& now) {
  LocalizationDecision decision;

  const bool requested_source_usable =
      !requested_source_.empty() && sourceUsable(requested_source_, now);
  const std::string selected =
      requested_source_usable ? requested_source_
                               : selectHealthyPrimaryOrFallback(now);
  std::string guarded_selection = selected;
  const bool current_usable = !active_source_.empty() &&
                              sourceUsable(active_source_, now);
  const bool force_selection =
      requested_source_usable || selection_change_requested_;
  if (current_usable && selected != active_source_ && !force_selection) {
    const double active_duration = nonNegativeAge(now, active_since_);
    const double cooldown_age = nonNegativeAge(now, last_switch_time_);
    if (!switch_back_to_primary_ || active_duration < minimum_active_time_ ||
        cooldown_age < switch_cooldown_) {
      guarded_selection = active_source_;
    }
  }

  // 只在被选中的源自己的回调中完成切换。这样active_source_changed会和
  // 目标源的第一帧同时出现，align_on_activation才能正确建立连续坐标对齐。
  if (!guarded_selection.empty() && guarded_selection != active_source_ &&
      measurement.config.name != guarded_selection) {
    guarded_selection = active_source_;
  }

  decision.active_source_changed = guarded_selection != active_source_;
  if (decision.active_source_changed) {
    setActiveSource(guarded_selection, now);
  }
  if (selection_change_requested_) {
    const bool request_applied =
        requested_source_.empty() ? active_source_ == selected
                                  : active_source_ == requested_source_;
    if (request_applied) {
      selection_change_requested_ = false;
    }
  }
  decision.active_source = active_source_;
  decision.accepted = !active_source_.empty() &&
                      measurement.config.name == active_source_;
  decision.authoritative = decision.accepted;
  decision.reason =
      decision.accepted
          ? (requested_source_.empty() ? "active localization source"
                                       : "manually selected localization source")
          : "standby source";
  return decision;
}

std::string LocalizationSourceManager::selectHealthyPrimaryOrFallback(
    const ros::Time& now) const {
  if (!switch_back_to_primary_ && !active_source_.empty()) {
    if (sourceUsable(active_source_, now)) {
      return active_source_;
    }
  }

  const LocalizationSourceConfig* selected = nullptr;
  for (const auto& config : configs_) {
    if (!config.enabled) {
      continue;
    }
    const auto adapter = adapter_by_name_.find(config.name);
    if (adapter == adapter_by_name_.end() || !sourceUsable(config.name, now)) {
      continue;
    }
    if (selected == nullptr || roleRank(config.role) < roleRank(selected->role) ||
        (roleRank(config.role) == roleRank(selected->role) &&
         config.priority < selected->priority)) {
      selected = &config;
    }
  }
  return selected == nullptr ? std::string() : selected->name;
}

std::vector<LocalizationSourceStatus> LocalizationSourceManager::statuses(
    const ros::Time& now) const {
  std::vector<LocalizationSourceStatus> result;
  result.reserve(configs_.size());
  for (const auto& config : configs_) {
    const auto adapter = adapter_by_name_.find(config.name);
    LocalizationSourceStatus source_status;
    if (adapter != adapter_by_name_.end()) {
      source_status = adapter->second->status(now);
    } else {
      source_status.config = config;
    }
    const auto reliability = reliability_by_name_.find(config.name);
    if (reliability != reliability_by_name_.end()) {
      const ReliabilityState& state = reliability->second;
      source_status.quarantined = !state.quarantine_until.isZero() &&
                                  now < state.quarantine_until;
      source_status.reliable = !source_status.quarantined && !state.recovering;
      source_status.accepted_corrections = state.accepted_corrections;
      source_status.rejected_corrections = state.rejected_corrections;
      source_status.consecutive_rejections = state.consecutive_rejections;
      source_status.recovery_samples = state.recovery_samples;
    }
    source_status.active = config.name == active_source_ &&
                           source_status.healthy && source_status.reliable;
    if (!config.enabled) {
      source_status.lifecycle_state = SourceLifecycleState::kDisabled;
    } else if (source_status.quarantined) {
      source_status.lifecycle_state = SourceLifecycleState::kQuarantined;
    } else if (!source_status.reliable) {
      source_status.lifecycle_state = SourceLifecycleState::kRecovering;
    } else if (source_status.received == 0) {
      source_status.lifecycle_state = SourceLifecycleState::kWaiting;
    } else if (!source_status.healthy || source_status.consecutive_rejections > 0) {
      source_status.lifecycle_state = SourceLifecycleState::kSuspect;
    } else if (source_status.active) {
      source_status.lifecycle_state = SourceLifecycleState::kActive;
    } else {
      source_status.lifecycle_state = SourceLifecycleState::kHealthy;
    }
    result.push_back(source_status);
  }
  return result;
}

bool LocalizationSourceManager::requestActiveSource(
    const std::string& source_name, const ros::Time& now,
    std::string* message) {
  const std::string normalized = lowerCase(source_name);
  if (source_name.empty() || normalized == "auto") {
    requested_source_.clear();
    selection_change_requested_ = true;
    if (message != nullptr) {
      *message =
          "automatic source selection requested; it will apply on the next localization sample";
    }
    return true;
  }

  const auto config =
      std::find_if(configs_.begin(), configs_.end(),
                   [&source_name](const LocalizationSourceConfig& candidate) {
                     return candidate.name == source_name;
                   });
  if (config == configs_.end() || !config->enabled ||
      adapter_by_name_.find(source_name) == adapter_by_name_.end()) {
    if (message != nullptr) {
      *message = "unknown or disabled localization source: " + source_name;
    }
    return false;
  }
  if (!sourceUsable(source_name, now)) {
    if (message != nullptr) {
      *message = "localization source is not currently healthy and reliable: " +
                 source_name;
    }
    return false;
  }

  requested_source_ = source_name;
  selection_change_requested_ = true;
  if (message != nullptr) {
    *message = "source switch accepted; it will apply on the next valid sample from " +
               source_name;
  }
  return true;
}

void LocalizationSourceManager::reportCorrectionResult(
    const std::string& source_name, const bool accepted, const ros::Time& now) {
  const auto reliability = reliability_by_name_.find(source_name);
  if (reliability == reliability_by_name_.end()) {
    return;
  }

  ReliabilityState& state = reliability->second;
  if (accepted) {
    ++state.accepted_corrections;
    state.consecutive_rejections = 0;
    state.quarantine_until = ros::Time(0);
    state.recovery_started = ros::Time(0);
    state.recovery_samples = 0;
    state.recovering = false;
    return;
  }

  ++state.rejected_corrections;
  ++state.consecutive_rejections;
  const auto config = std::find_if(
      configs_.begin(), configs_.end(),
      [&source_name](const LocalizationSourceConfig& candidate) {
        return candidate.name == source_name;
      });
  if (config != configs_.end() &&
      state.consecutive_rejections >= config->max_consecutive_rejections) {
    state.quarantine_until = now + ros::Duration(config->quarantine_duration);
    state.recovery_started = ros::Time(0);
    state.recovery_samples = 0;
    state.recovering = true;
    state.consecutive_rejections = 0;
    ROS_WARN("[xd_uav_state_estimator] source '%s' quarantined for %.2f s after repeated rejected corrections",
             source_name.c_str(), config->quarantine_duration);
  }
}

bool LocalizationSourceManager::localizationValid(const ros::Time& now) const {
  if (active_source_.empty()) {
    return false;
  }
  return sourceUsable(active_source_, now);
}

bool LocalizationSourceManager::sourceUsable(const std::string& name,
                                              const ros::Time& now) const {
  const auto adapter = adapter_by_name_.find(name);
  if (adapter == adapter_by_name_.end() || !adapter->second->status(now).healthy) {
    return false;
  }
  const auto reliability = reliability_by_name_.find(name);
  if (reliability == reliability_by_name_.end()) {
    return true;
  }
  const ReliabilityState& state = reliability->second;
  return !state.recovering &&
         (state.quarantine_until.isZero() || now >= state.quarantine_until);
}

void LocalizationSourceManager::updateRecovery(const std::string& name,
                                               const ros::Time& now) {
  const auto reliability = reliability_by_name_.find(name);
  if (reliability == reliability_by_name_.end()) {
    return;
  }
  ReliabilityState& state = reliability->second;
  if (!state.recovering || now < state.quarantine_until) {
    return;
  }

  if (state.recovery_started.isZero()) {
    state.recovery_started = now;
    state.recovery_samples = 0;
  }
  ++state.recovery_samples;

  const auto config = std::find_if(
      configs_.begin(), configs_.end(),
      [&name](const LocalizationSourceConfig& candidate) {
        return candidate.name == name;
      });
  if (config == configs_.end()) {
    return;
  }
  const double recovery_age = nonNegativeAge(now, state.recovery_started);
  if (state.recovery_samples >= config->recovery_min_samples &&
      recovery_age >= config->recovery_stable_time) {
    state.recovering = false;
    state.quarantine_until = ros::Time(0);
    state.consecutive_rejections = 0;
    ROS_INFO("[xd_uav_state_estimator] source '%s' completed recovery with %d stable samples",
             name.c_str(), state.recovery_samples);
  }
}

void LocalizationSourceManager::setActiveSource(const std::string& name,
                                                 const ros::Time& now) {
  if (name == active_source_) {
    return;
  }
  active_source_ = name;
  active_since_ = now;
  last_switch_time_ = now;
  ++switch_count_;
}

std::string LocalizationSourceManager::activeSource() const {
  return active_source_;
}

std::string LocalizationSourceManager::requestedSource() const {
  return requested_source_;
}

bool LocalizationSourceManager::automaticSelection() const {
  return requested_source_.empty();
}

std::size_t LocalizationSourceManager::enabledSourceCount() const {
  return adapters_.size();
}

std::uint64_t LocalizationSourceManager::switchCount() const {
  return switch_count_;
}

const std::vector<LocalizationSourceConfig>&
LocalizationSourceManager::configs() const {
  return configs_;
}

SourceRole LocalizationSourceManager::parseRole(const std::string& value) {
  const std::string normalized = lowerCase(value);
  if (normalized == "primary") {
    return SourceRole::kPrimary;
  }
  if (normalized == "fallback") {
    return SourceRole::kFallback;
  }
  throw std::invalid_argument("unsupported localization source role: " + value);
}

AlignmentMode LocalizationSourceManager::parseAlignmentMode(
    const std::string& value) {
  const std::string normalized = lowerCase(value);
  if (normalized == "identity") {
    return AlignmentMode::kIdentity;
  }
  if (normalized == "align_on_first_measurement") {
    return AlignmentMode::kAlignOnFirstMeasurement;
  }
  if (normalized == "align_on_activation") {
    return AlignmentMode::kAlignOnActivation;
  }
  throw std::invalid_argument("unsupported localization alignment mode: " + value);
}

std::string sourceRoleName(const SourceRole role) {
  switch (role) {
    case SourceRole::kPrimary:
      return "primary";
    case SourceRole::kFallback:
      return "fallback";
  }
  return "unknown";
}

std::string alignmentModeName(const AlignmentMode mode) {
  switch (mode) {
    case AlignmentMode::kIdentity:
      return "identity";
    case AlignmentMode::kAlignOnFirstMeasurement:
      return "align_on_first_measurement";
    case AlignmentMode::kAlignOnActivation:
      return "align_on_activation";
  }
  return "unknown";
}

std::string sourceLifecycleStateName(const SourceLifecycleState state) {
  switch (state) {
    case SourceLifecycleState::kDisabled:
      return "disabled";
    case SourceLifecycleState::kWaiting:
      return "waiting";
    case SourceLifecycleState::kHealthy:
      return "healthy";
    case SourceLifecycleState::kActive:
      return "active";
    case SourceLifecycleState::kSuspect:
      return "suspect";
    case SourceLifecycleState::kQuarantined:
      return "quarantined";
    case SourceLifecycleState::kRecovering:
      return "recovering";
  }
  return "unknown";
}

}  // 命名空间 xd_uav_state_estimator
