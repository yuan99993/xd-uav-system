#include <xd_uav_state_estimator/state_estimator_node.h>

#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iterator>
#include <sstream>
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

std::string numberString(const double value, const int precision = 4) {
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(precision) << value;
  return stream.str();
}

std::string withoutBoundarySlashes(std::string value) {
  while (!value.empty() && value.front() == '/') {
    value.erase(value.begin());
  }
  while (!value.empty() && value.back() == '/') {
    value.pop_back();
  }
  return value;
}

std::string estimatorStateName(const std::uint8_t state) {
  switch (state) {
    case EstimatorStatus::WAITING:
      return "waiting";
    case EstimatorStatus::RUNNING:
      return "running";
    case EstimatorStatus::DEGRADED:
      return "degraded";
    case EstimatorStatus::DEAD_RECKONING:
      return "dead_reckoning";
    case EstimatorStatus::LOST:
      return "lost";
  }
  return "unknown";
}

double quaternionYaw(const tf2::Quaternion& orientation) {
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
  return yaw;
}

tf2::Quaternion quaternionWithYaw(const tf2::Quaternion& orientation,
                                  const double yaw) {
  double roll = 0.0;
  double pitch = 0.0;
  double unused_yaw = 0.0;
  tf2::Matrix3x3(orientation).getRPY(roll, pitch, unused_yaw);
  tf2::Quaternion result;
  result.setRPY(roll, pitch, yaw);
  result.normalize();
  return result;
}

double median(std::vector<double> values) {
  if (values.empty()) {
    return 0.0;
  }
  const std::size_t middle = values.size() / 2;
  std::nth_element(values.begin(), values.begin() + middle, values.end());
  if (values.size() % 2 != 0) {
    return values[middle];
  }
  const double upper = values[middle];
  std::nth_element(values.begin(), values.begin() + middle - 1, values.end());
  return 0.5 * (values[middle - 1] + upper);
}

}  // 匿名命名空间

StateEstimatorNode::StateEstimatorNode(ros::NodeHandle nh, ros::NodeHandle private_nh)
    : nh_(std::move(nh)), private_nh_(std::move(private_nh)) {
  loadParameters();

  ros::TransportHints transport_hints;
  transport_hints.tcpNoDelay();
  imu_subscriber_ = nh_.subscribe("imu_in", 100, &StateEstimatorNode::imuCallback, this, transport_hints);
  control_input_subscriber_ =
      nh_.subscribe("control_input_in", 20, &StateEstimatorNode::controlInputCallback, this, transport_hints);

  LocalizationSourceDefaults source_defaults;
  source_defaults.timeout = input_timeout_;
  source_defaults.twist_in_body_frame = odom_twist_in_body_frame_;
  source_defaults.position_xy_variance = position_xy_variance_;
  source_defaults.position_z_variance = position_z_variance_;
  source_defaults.velocity_xy_variance = velocity_xy_variance_;
  source_defaults.velocity_z_variance = velocity_z_variance_;
  localization_source_manager_ = std::make_unique<LocalizationSourceManager>(
      nh_, private_nh_, source_defaults,
      [this](const LocalizationMeasurement& measurement,
             const LocalizationDecision& decision) {
        localizationCallback(measurement, decision);
      });
  initializeSourceEstimators();

  main_odometry_publisher_ =
      private_nh_.advertise<nav_msgs::Odometry>("main/odom", 10);
  acceleration_publisher_ = private_nh_.advertise<geometry_msgs::AccelStamped>("acceleration", 10);
  imu_acceleration_publisher_ =
      private_nh_.advertise<geometry_msgs::AccelStamped>("imu_acceleration", 10);
  innovation_publisher_ = private_nh_.advertise<nav_msgs::Odometry>("innovation", 10);
  diagnostics_publisher_ = private_nh_.advertise<diagnostic_msgs::DiagnosticArray>("diagnostics", 10);
  localization_valid_publisher_ =
      private_nh_.advertise<std_msgs::Bool>("localization_valid", 1, true);
  state_valid_publisher_ =
      private_nh_.advertise<std_msgs::Bool>("state_valid", 1, true);
  estimator_status_publisher_ =
      private_nh_.advertise<EstimatorStatus>("status", 1, true);
  reset_server_ = private_nh_.advertiseService("reset", &StateEstimatorNode::resetCallback, this);
  switch_source_server_ = private_nh_.advertiseService(
      "switch_source", &StateEstimatorNode::switchSourceCallback, this);

  publish_timer_ = nh_.createTimer(ros::Duration(1.0 / output_rate_),
                                   &StateEstimatorNode::publishTimerCallback, this);
  diagnostics_timer_ = nh_.createTimer(ros::Duration(1.0 / diagnostics_rate_),
                                       &StateEstimatorNode::diagnosticsTimerCallback, this);

  ROS_INFO_STREAM("[xd_uav_state_estimator] initialized for " << uav_name_
                  << ", odom frame=" << odom_frame_ << ", body frame=" << body_frame_
                  << ", publish_tf=" << std::boolalpha << publish_tf_
                  << ", imu_prediction=" << use_imu_prediction_
                  << ", source_selection=automatic/service"
                  << ", localization_sources="
                  << localization_source_manager_->enabledSourceCount());
}

void StateEstimatorNode::loadParameters() {
  private_nh_.param<std::string>("uav_name", uav_name_, "uav1");
  private_nh_.param<std::string>("world_frame", world_frame_, "world");
  private_nh_.param<std::string>("map_frame", map_frame_, uav_name_ + "/local_origin");
  private_nh_.param<std::string>("odom_frame", odom_frame_, uav_name_ + "/odom");
  private_nh_.param<std::string>("body_frame", body_frame_, uav_name_ + "/base_link");

  uav_name_ = withoutBoundarySlashes(uav_name_);
  world_frame_ = withoutBoundarySlashes(world_frame_);
  map_frame_ = withoutBoundarySlashes(map_frame_);
  odom_frame_ = withoutBoundarySlashes(odom_frame_);
  body_frame_ = withoutBoundarySlashes(body_frame_);
  if (uav_name_.empty() || world_frame_.empty() || map_frame_.empty() ||
      odom_frame_.empty() || body_frame_.empty()) {
    throw std::invalid_argument("uav_name and TF frame parameters must not be empty");
  }

  private_nh_.param("output_rate", output_rate_, output_rate_);
  private_nh_.param("diagnostics_rate", diagnostics_rate_, diagnostics_rate_);
  private_nh_.param("input_timeout", input_timeout_, input_timeout_);
  active_localization_timeout_ = input_timeout_;
  private_nh_.param("imu_timeout", imu_timeout_, imu_timeout_);
  private_nh_.param("control_input_timeout", control_input_timeout_, control_input_timeout_);
  private_nh_.param("max_prediction_step", max_prediction_step_, max_prediction_step_);
  private_nh_.param("reset_gap", reset_gap_, reset_gap_);
  private_nh_.param("max_localization_delay", max_localization_delay_,
                    max_localization_delay_);
  private_nh_.param("delayed_measurement/enabled", delayed_measurement_enabled_,
                    delayed_measurement_enabled_);
  private_nh_.param("delayed_measurement/history_duration",
                    filter_history_duration_, filter_history_duration_);
  private_nh_.param("max_dead_reckoning_time", max_dead_reckoning_time_,
                    max_dead_reckoning_time_);
  private_nh_.param("odom_twist_in_body_frame", odom_twist_in_body_frame_, odom_twist_in_body_frame_);
  private_nh_.param("output_twist_in_body_frame", output_twist_in_body_frame_, output_twist_in_body_frame_);
  private_nh_.param("publish_tf", publish_tf_, publish_tf_);
  private_nh_.param("publish_source_origin_tf", publish_source_origin_tf_,
                    publish_source_origin_tf_);
  private_nh_.param("require_imu", require_imu_, require_imu_);
  private_nh_.param("imu_prediction/enabled", use_imu_prediction_, use_imu_prediction_);
  private_nh_.param("use_control_input", use_control_input_, use_control_input_);

  private_nh_.param("measurement_noise/position_xy", position_xy_variance_, position_xy_variance_);
  private_nh_.param("measurement_noise/position_z", position_z_variance_, position_z_variance_);
  private_nh_.param("measurement_noise/velocity_xy", velocity_xy_variance_, velocity_xy_variance_);
  private_nh_.param("measurement_noise/velocity_z", velocity_z_variance_, velocity_z_variance_);
  private_nh_.param("heading/measurement_noise", heading_variance_,
                    heading_variance_);
  private_nh_.param("heading/rate_measurement_noise", heading_rate_variance_,
                    heading_rate_variance_);
  private_nh_.param("innovation_limit/position_xy", position_xy_innovation_limit_, position_xy_innovation_limit_);
  private_nh_.param("innovation_limit/position_z", position_z_innovation_limit_, position_z_innovation_limit_);
  private_nh_.param("innovation_limit/velocity_xy", velocity_xy_innovation_limit_,
                    velocity_xy_innovation_limit_);
  private_nh_.param("innovation_limit/velocity_z", velocity_z_innovation_limit_,
                    velocity_z_innovation_limit_);
  private_nh_.param("heading/innovation_limit", heading_innovation_limit_,
                    heading_innovation_limit_);
  private_nh_.param("heading/use_imu_rate", use_imu_heading_rate_,
                    use_imu_heading_rate_);
  private_nh_.param("innovation_gate/use_mahalanobis", use_mahalanobis_gate_,
                    use_mahalanobis_gate_);
  private_nh_.param("innovation_gate/position_xy_nis",
                    position_xy_nis_limit_, position_xy_nis_limit_);
  private_nh_.param("innovation_gate/position_z_nis",
                    position_z_nis_limit_, position_z_nis_limit_);
  private_nh_.param("innovation_gate/velocity_xy_nis",
                    velocity_xy_nis_limit_, velocity_xy_nis_limit_);
  private_nh_.param("innovation_gate/velocity_z_nis",
                    velocity_z_nis_limit_, velocity_z_nis_limit_);
  private_nh_.param("innovation_gate/heading_nis",
                    heading_nis_limit_, heading_nis_limit_);

  output_rate_ = std::max(output_rate_, 1.0);
  diagnostics_rate_ = std::max(diagnostics_rate_, 0.2);
  max_prediction_step_ = std::clamp(max_prediction_step_, 0.001, 0.1);
  reset_gap_ = std::max(reset_gap_, max_prediction_step_);
  max_localization_delay_ = std::max(max_localization_delay_, 0.0);
  filter_history_duration_ = std::max(
      filter_history_duration_, max_localization_delay_ + max_prediction_step_);
  max_dead_reckoning_time_ = std::max(max_dead_reckoning_time_, 0.0);
  heading_variance_ = std::max(heading_variance_, 1e-12);
  heading_rate_variance_ = std::max(heading_rate_variance_, 1e-12);
  heading_innovation_limit_ = std::clamp(heading_innovation_limit_, 0.01, 3.14159265358979323846);
  position_xy_nis_limit_ = std::max(position_xy_nis_limit_, 0.1);
  position_z_nis_limit_ = std::max(position_z_nis_limit_, 0.1);
  velocity_xy_nis_limit_ = std::max(velocity_xy_nis_limit_, 0.1);
  velocity_z_nis_limit_ = std::max(velocity_z_nis_limit_, 0.1);
  heading_nis_limit_ = std::max(heading_nis_limit_, 0.1);

  private_nh_.param("initial_covariance/position",
                    filter_parameters_.initial_position_covariance,
                    filter_parameters_.initial_position_covariance);
  private_nh_.param("initial_covariance/velocity",
                    filter_parameters_.initial_velocity_covariance,
                    filter_parameters_.initial_velocity_covariance);
  private_nh_.param("initial_covariance/acceleration",
                    filter_parameters_.initial_acceleration_covariance,
                    filter_parameters_.initial_acceleration_covariance);
  private_nh_.param("process_noise/lateral/position",
                    filter_parameters_.lateral_process_position,
                    filter_parameters_.lateral_process_position);
  private_nh_.param("process_noise/lateral/velocity",
                    filter_parameters_.lateral_process_velocity,
                    filter_parameters_.lateral_process_velocity);
  private_nh_.param("process_noise/lateral/acceleration",
                    filter_parameters_.lateral_process_acceleration,
                    filter_parameters_.lateral_process_acceleration);
  private_nh_.param("process_noise/altitude/position",
                    filter_parameters_.altitude_process_position,
                    filter_parameters_.altitude_process_position);
  private_nh_.param("process_noise/altitude/velocity",
                    filter_parameters_.altitude_process_velocity,
                    filter_parameters_.altitude_process_velocity);
  private_nh_.param("process_noise/altitude/acceleration",
                    filter_parameters_.altitude_process_acceleration,
                    filter_parameters_.altitude_process_acceleration);
  private_nh_.param("control_input_coeff", filter_parameters_.input_coefficient,
                    filter_parameters_.input_coefficient);
  if (use_imu_prediction_) {
    private_nh_.param("imu_prediction/input_coefficient",
                      filter_parameters_.input_coefficient,
                      filter_parameters_.input_coefficient);
  }
  filters_ = std::make_unique<LinearFilters>(filter_parameters_);

  private_nh_.param("heading/initial_covariance",
                    heading_filter_parameters_.initial_heading_covariance,
                    heading_filter_parameters_.initial_heading_covariance);
  private_nh_.param("heading/initial_rate_covariance",
                    heading_filter_parameters_.initial_rate_covariance,
                    heading_filter_parameters_.initial_rate_covariance);
  private_nh_.param("heading/process_noise",
                    heading_filter_parameters_.heading_process_noise,
                    heading_filter_parameters_.heading_process_noise);
  private_nh_.param("heading/rate_process_noise",
                    heading_filter_parameters_.rate_process_noise,
                    heading_filter_parameters_.rate_process_noise);
  heading_filter_ =
      std::make_unique<HeadingFilter>(heading_filter_parameters_);

  ImuAccelerationParameters imu_parameters;
  private_nh_.param("imu_prediction/remove_gravity", imu_parameters.remove_gravity,
                    imu_parameters.remove_gravity);
  double gravity_magnitude = imu_parameters.gravity.z();
  private_nh_.param("imu_prediction/gravity_magnitude", gravity_magnitude,
                    gravity_magnitude);
  imu_parameters.gravity = Eigen::Vector3d(0.0, 0.0, gravity_magnitude);
  private_nh_.param("imu_prediction/acceleration_limit", imu_parameters.acceleration_limit,
                    imu_parameters.acceleration_limit);
  private_nh_.param("imu_prediction/estimate_bias", imu_parameters.estimate_bias,
                    imu_parameters.estimate_bias);
  int bias_samples = static_cast<int>(imu_parameters.bias_samples);
  private_nh_.param("imu_prediction/bias_samples", bias_samples, bias_samples);
  imu_parameters.bias_samples = static_cast<std::size_t>(std::max(bias_samples, 1));
  private_nh_.param("imu_prediction/stationary_velocity_threshold",
                    imu_parameters.stationary_velocity_threshold,
                    imu_parameters.stationary_velocity_threshold);
  private_nh_.param("imu_prediction/stationary_angular_velocity_threshold",
                    imu_parameters.stationary_angular_velocity_threshold,
                    imu_parameters.stationary_angular_velocity_threshold);
  private_nh_.param("imu_prediction/initial_bias/x", imu_parameters.initial_bias.x(),
                    imu_parameters.initial_bias.x());
  private_nh_.param("imu_prediction/initial_bias/y", imu_parameters.initial_bias.y(),
                    imu_parameters.initial_bias.y());
  private_nh_.param("imu_prediction/initial_bias/z", imu_parameters.initial_bias.z(),
                    imu_parameters.initial_bias.z());
  imu_acceleration_processor_ =
      std::make_unique<ImuAccelerationProcessor>(imu_parameters);
}

void StateEstimatorNode::initializeSourceEstimators() {
  for (const auto& config : localization_source_manager_->configs()) {
    if (!config.enabled) {
      continue;
    }

    auto estimator = std::make_unique<SourceEstimator>(
        config, filter_parameters_, heading_filter_parameters_);
    const std::string topic_prefix = "sources/" + config.name;
    estimator->odometry_publisher =
        private_nh_.advertise<nav_msgs::Odometry>(topic_prefix + "/odom", 10);
    estimator->valid_publisher =
        private_nh_.advertise<std_msgs::Bool>(topic_prefix + "/valid", 1, true);
    std_msgs::Bool initial_valid;
    initial_valid.data = false;
    estimator->valid_publisher.publish(initial_valid);
    source_estimators_.emplace(config.name, std::move(estimator));

    ROS_INFO("[xd_uav_state_estimator] source '%s' estimate output: ~/%s/odom",
             config.name.c_str(), topic_prefix.c_str());
  }
}

void StateEstimatorNode::updateSourceEstimator(
    const LocalizationMeasurement& measurement,
    const Eigen::Vector3d& position,
    const Eigen::Vector3d& velocity,
    const geometry_msgs::Quaternion& orientation_message) {
  const auto iterator = source_estimators_.find(measurement.config.name);
  if (iterator == source_estimators_.end()) {
    return;
  }
  SourceEstimator& estimator = *iterator->second;

  tf2::Quaternion orientation;
  if (!normalizeQuaternion(orientation_message, &orientation)) {
    return;
  }

  try {
    double dt = estimator.filters.initialized()
                    ? (measurement.stamp - estimator.filters.stamp()).toSec()
                    : 0.0;
    if (estimator.filters.initialized() &&
        (!std::isfinite(dt) || dt < -reset_gap_ || dt > reset_gap_)) {
      estimator.filters.reset();
      estimator.heading_filter.reset();
      dt = 0.0;
    }

    if (!estimator.filters.initialized()) {
      Eigen::Vector3d initial_position = Eigen::Vector3d::Zero();
      Eigen::Vector3d initial_velocity = Eigen::Vector3d::Zero();
      if (measurement.config.use_position_xy) {
        initial_position.head<2>() = position.head<2>();
      }
      if (measurement.config.use_altitude) {
        initial_position.z() = position.z();
      }
      if (measurement.config.use_velocity_xy) {
        initial_velocity.head<2>() = velocity.head<2>();
      }
      if (measurement.config.use_vertical_velocity) {
        initial_velocity.z() = velocity.z();
      }
      estimator.filters.initialize(initial_position, initial_velocity,
                                   measurement.stamp);

      tf2::Quaternion heading_orientation = orientation;
      double heading_rate = measurement.angular_velocity.z;
      if (!measurement.config.use_heading && have_imu_) {
        normalizeQuaternion(latest_imu_.orientation, &heading_orientation);
        heading_rate = latest_imu_.angular_velocity.z;
      }
      estimator.heading_filter.initialize(
          quaternionYaw(heading_orientation),
          std::isfinite(heading_rate) ? heading_rate : 0.0,
          measurement.stamp);
    } else if (dt > 0.0) {
      const Eigen::Vector3d acceleration =
          use_imu_prediction_ && imu_acceleration_valid_
              ? latest_imu_acceleration_odom_
              : Eigen::Vector3d::Zero();
      const bool acceleration_valid =
          use_imu_prediction_ && imu_acceleration_valid_;
      ros::Time prediction_stamp = estimator.filters.stamp();
      double remaining = dt;
      while (remaining > 1e-9) {
        const double step = std::min(remaining, max_prediction_step_);
        prediction_stamp += ros::Duration(step);
        estimator.filters.predict(step, acceleration, acceleration_valid,
                                  prediction_stamp);
        remaining -= step;
      }
      estimator.heading_filter.predict(measurement.stamp);
    }

    const ros::Time correction_stamp =
        estimator.filters.stamp() > measurement.stamp
            ? estimator.filters.stamp()
            : measurement.stamp;
    if (measurement.config.use_position_xy) {
      estimator.filters.correctLateralPosition(
          position.head<2>(), measurement.position_xy_variance,
          correction_stamp);
    }
    if (measurement.config.use_altitude) {
      estimator.filters.correctAltitudePosition(
          position.z(), measurement.position_z_variance, correction_stamp);
    }
    if (measurement.config.use_velocity_xy) {
      estimator.filters.correctLateralVelocity(
          velocity.head<2>(), measurement.velocity_xy_variance,
          correction_stamp);
    }
    if (measurement.config.use_vertical_velocity) {
      estimator.filters.correctVerticalVelocity(
          velocity.z(), measurement.velocity_z_variance, correction_stamp);
    }
    if (measurement.config.use_heading) {
      estimator.heading_filter.correctHeading(
          quaternionYaw(orientation), measurement.heading_variance,
          correction_stamp);
    }

    estimator.orientation = orientation_message;
    estimator.angular_velocity = measurement.angular_velocity;
    estimator.last_receive_time = measurement.receive_time;
    if (!estimator.filters.finite() || !estimator.heading_filter.finite()) {
      throw std::runtime_error("source filter generated non-finite state");
    }
    publishSourceEstimator(estimator);
  } catch (const std::exception& exception) {
    estimator.filters.reset();
    estimator.heading_filter.reset();
    estimator.last_receive_time = ros::Time(0);
    ROS_WARN("[xd_uav_state_estimator] source '%s' independent estimate reset: %s",
             measurement.config.name.c_str(), exception.what());
  }
}

void StateEstimatorNode::publishSourceEstimator(SourceEstimator& estimator) {
  if (!estimator.filters.initialized() ||
      !estimator.heading_filter.initialized()) {
    return;
  }

  geometry_msgs::Quaternion orientation_message = estimator.orientation;
  geometry_msgs::Vector3 angular_velocity = estimator.angular_velocity;
  if (have_imu_ &&
      nonNegativeAge(ros::Time::now(), last_imu_receive_time_) <= imu_timeout_) {
    orientation_message = latest_imu_.orientation;
    angular_velocity = latest_imu_.angular_velocity;
  }

  tf2::Quaternion orientation;
  if (!normalizeQuaternion(orientation_message, &orientation)) {
    return;
  }
  orientation =
      quaternionWithYaw(orientation, estimator.heading_filter.heading());
  angular_velocity.z = estimator.heading_filter.rate();

  const Eigen::Vector3d position = estimator.filters.position();
  const Eigen::Vector3d velocity_parent = estimator.filters.velocity();
  const Eigen::Vector3d velocity_output =
      output_twist_in_body_frame_
          ? rotateParentToBody(velocity_parent, orientation)
          : velocity_parent;

  nav_msgs::Odometry odometry;
  odometry.header.stamp = estimator.filters.stamp();
  odometry.header.frame_id = odom_frame_;
  odometry.child_frame_id = body_frame_;
  odometry.pose.pose.position.x = position.x();
  odometry.pose.pose.position.y = position.y();
  odometry.pose.pose.position.z = position.z();
  odometry.pose.pose.orientation = tf2::toMsg(orientation);
  odometry.twist.twist.linear = eigenToVector(velocity_output);
  odometry.twist.twist.angular = angular_velocity;

  const Eigen::Matrix3d position_covariance =
      estimator.filters.positionCovariance();
  odometry.pose.covariance[0] = position_covariance(0, 0);
  odometry.pose.covariance[7] = position_covariance(1, 1);
  odometry.pose.covariance[14] = position_covariance(2, 2);
  odometry.pose.covariance[35] =
      estimator.heading_filter.headingVariance();

  Eigen::Matrix3d velocity_covariance =
      estimator.filters.velocityCovariance();
  if (output_twist_in_body_frame_) {
    const tf2::Matrix3x3 rotation(orientation.inverse());
    Eigen::Matrix3d eigen_rotation;
    for (int row = 0; row < 3; ++row) {
      for (int column = 0; column < 3; ++column) {
        eigen_rotation(row, column) = rotation[row][column];
      }
    }
    velocity_covariance =
        eigen_rotation * velocity_covariance * eigen_rotation.transpose();
  }
  odometry.twist.covariance[0] = velocity_covariance(0, 0);
  odometry.twist.covariance[7] = velocity_covariance(1, 1);
  odometry.twist.covariance[14] = velocity_covariance(2, 2);
  odometry.twist.covariance[35] =
      estimator.heading_filter.rateVariance();

  estimator.odometry_publisher.publish(odometry);
  std_msgs::Bool valid;
  valid.data = true;
  estimator.valid_publisher.publish(valid);
}

void StateEstimatorNode::publishSourceValidity(const ros::Time& now) {
  for (auto& item : source_estimators_) {
    SourceEstimator& estimator = *item.second;
    const bool valid =
        estimator.filters.initialized() && estimator.filters.finite() &&
        estimator.heading_filter.initialized() &&
        estimator.heading_filter.finite() &&
        !estimator.last_receive_time.isZero() &&
        nonNegativeAge(now, estimator.last_receive_time) <=
            estimator.config.timeout;
    std_msgs::Bool valid_message;
    valid_message.data = valid;
    estimator.valid_publisher.publish(valid_message);
  }
}

void StateEstimatorNode::publishSourceOriginTransforms(
    const ros::Time& stamp) {
  if (!publish_source_origin_tf_) {
    return;
  }

  std::vector<geometry_msgs::TransformStamped> transforms;
  transforms.reserve(source_alignments_.size());
  for (const auto& item : source_alignments_) {
    const SourceAlignment& alignment = item.second;
    const std::string& source_origin = alignment.parent_frame;
    if (!alignment.initialized || source_origin.empty() ||
        source_origin == world_frame_ || source_origin == map_frame_ ||
        source_origin == odom_frame_ || source_origin == body_frame_) {
      continue;
    }

    geometry_msgs::TransformStamped transform;
    transform.header.stamp = stamp;
    transform.header.frame_id = odom_frame_;
    transform.child_frame_id = source_origin;
    transform.transform.translation.x = alignment.translation.x();
    transform.transform.translation.y = alignment.translation.y();
    transform.transform.translation.z = alignment.translation.z();
    transform.transform.rotation = tf2::toMsg(alignment.rotation);
    transforms.push_back(transform);
  }

  if (!transforms.empty()) {
    transform_broadcaster_.sendTransform(transforms);
  }
}

void StateEstimatorNode::localizationCallback(
    const LocalizationMeasurement& measurement,
    const LocalizationDecision& decision) {
  ++received_odometry_;

  std::scoped_lock lock(mutex_);

  double source_dt = 0.0;
  const auto previous_stamp = last_source_stamps_.find(measurement.config.name);
  if (previous_stamp != last_source_stamps_.end()) {
    source_dt = (measurement.stamp - previous_stamp->second).toSec();
  }
  last_source_stamps_[measurement.config.name] = measurement.stamp;

  Eigen::Vector3d position;
  Eigen::Vector3d velocity;
  geometry_msgs::Quaternion aligned_orientation;
  bool alignment_pending = false;
  if (!alignMeasurement(measurement, decision, &position, &velocity,
                        &aligned_orientation, &alignment_pending)) {
    if (alignment_pending) {
      ++ignored_localization_;
      if (decision.accepted) {
        healthy_ = false;
        health_message_ = "collecting localization alignment samples";
      }
      return;
    }
    ++rejected_odometry_;
    localization_source_manager_->reportCorrectionResult(
        measurement.config.name, false, ros::Time::now());
    if (decision.accepted) {
      healthy_ = false;
      health_message_ = "localization alignment failed";
    }
    return;
  }

  updateSourceEstimator(measurement, position, velocity, aligned_orientation);

  if (!decision.accepted) {
    ++ignored_localization_;
    return;
  }

  tf2::Quaternion orientation;
  if (!normalizeQuaternion(aligned_orientation, &orientation)) {
    ++rejected_odometry_;
    localization_source_manager_->reportCorrectionResult(
        measurement.config.name, false, ros::Time::now());
    return;
  }

  const double measured_heading = quaternionYaw(orientation);

  if (!filters_->initialized()) {
    const bool has_position = measurement.config.use_position_xy ||
                              measurement.config.use_altitude;
    if (!has_position || !decision.authoritative) {
      ++ignored_localization_;
      health_message_ = "waiting for an authoritative position source";
      return;
    }
    Eigen::Vector3d initial_position = Eigen::Vector3d::Zero();
    Eigen::Vector3d initial_velocity = Eigen::Vector3d::Zero();
    if (measurement.config.use_position_xy) {
      initial_position.head<2>() = position.head<2>();
    }
    if (measurement.config.use_altitude) {
      initial_position.z() = position.z();
    }
    if (measurement.config.use_velocity_xy) {
      initial_velocity.head<2>() = velocity.head<2>();
    }
    if (measurement.config.use_vertical_velocity) {
      initial_velocity.z() = velocity.z();
    }
    latest_odometry_velocity_odom_ = initial_velocity;
    last_odom_receive_time_ = measurement.receive_time;
    active_localization_timeout_ = measurement.config.timeout;
    have_odom_ = true;
    initializeFilter(initial_position, initial_velocity, measurement.stamp);
    odom_orientation_ = aligned_orientation;
    odom_angular_velocity_ = measurement.angular_velocity;
    initializeHeading(aligned_orientation, measurement.angular_velocity.z,
                      measurement.config.use_heading, measurement.stamp);
    localization_source_manager_->reportCorrectionResult(
        measurement.config.name, true, ros::Time::now());
    return;
  }

  if (source_dt < -reset_gap_ &&
      (measurement.config.use_position_xy || measurement.config.use_altitude) &&
      decision.authoritative) {
    ++reset_count_;
    Eigen::Vector3d initial_position = filters_->position();
    Eigen::Vector3d initial_velocity = filters_->velocity();
    if (measurement.config.use_position_xy) {
      initial_position.head<2>() = position.head<2>();
    }
    if (measurement.config.use_altitude) {
      initial_position.z() = position.z();
    }
    if (measurement.config.use_velocity_xy) {
      initial_velocity.head<2>() = velocity.head<2>();
    }
    if (measurement.config.use_vertical_velocity) {
      initial_velocity.z() = velocity.z();
    }
    ROS_WARN("[xd_uav_state_estimator] source '%s' timestamp moved backwards by %.3f s; reinitializing",
             measurement.config.name.c_str(), source_dt);
    latest_odometry_velocity_odom_ = initial_velocity;
    last_odom_receive_time_ = measurement.receive_time;
    active_localization_timeout_ = measurement.config.timeout;
    have_odom_ = true;
    initializeFilter(initial_position, initial_velocity, measurement.stamp);
    odom_orientation_ = aligned_orientation;
    odom_angular_velocity_ = measurement.angular_velocity;
    initializeHeading(aligned_orientation, measurement.angular_velocity.z,
                      measurement.config.use_heading, measurement.stamp);
    localization_source_manager_->reportCorrectionResult(
        measurement.config.name, true, ros::Time::now());
    return;
  }

  try {
    std::vector<PredictionRecord> replay_records;
    ros::Time replay_present_stamp;
    bool delayed_replay = false;
    double heading_for_correction = measured_heading;
    const double filter_dt = (measurement.stamp - filters_->stamp()).toSec();
    if (filter_dt < -max_localization_delay_) {
      ++ignored_localization_;
      ++delayed_rejection_count_;
      localization_source_manager_->reportCorrectionResult(
          measurement.config.name, false, ros::Time::now());
      ROS_WARN_THROTTLE(1.0,
                        "[xd_uav_state_estimator] ignoring source '%s' measurement delayed by %.3f s",
                        measurement.config.name.c_str(), -filter_dt);
      return;
    }
    if (filter_dt > reset_gap_) {
      if ((measurement.config.use_position_xy || measurement.config.use_altitude) &&
          decision.authoritative) {
        ++reset_count_;
        Eigen::Vector3d initial_position = filters_->position();
        Eigen::Vector3d initial_velocity = filters_->velocity();
        if (measurement.config.use_position_xy) {
          initial_position.head<2>() = position.head<2>();
        }
        if (measurement.config.use_altitude) {
          initial_position.z() = position.z();
        }
        if (measurement.config.use_velocity_xy) {
          initial_velocity.head<2>() = velocity.head<2>();
        }
        if (measurement.config.use_vertical_velocity) {
          initial_velocity.z() = velocity.z();
        }
        ROS_WARN("[xd_uav_state_estimator] prediction gap before localization correction is %.3f s; reinitializing",
                 filter_dt);
        latest_odometry_velocity_odom_ = initial_velocity;
        last_odom_receive_time_ = measurement.receive_time;
        active_localization_timeout_ = measurement.config.timeout;
        have_odom_ = true;
        initializeFilter(initial_position, initial_velocity, measurement.stamp);
        odom_orientation_ = aligned_orientation;
        odom_angular_velocity_ = measurement.angular_velocity;
        initializeHeading(aligned_orientation, measurement.angular_velocity.z,
                          measurement.config.use_heading, measurement.stamp);
        localization_source_manager_->reportCorrectionResult(
            measurement.config.name, true, ros::Time::now());
      } else {
        ++ignored_localization_;
        localization_source_manager_->reportCorrectionResult(
            measurement.config.name, false, ros::Time::now());
      }
      return;
    }
    if (filter_dt < -1e-6) {
      if (!delayed_measurement_enabled_ ||
          !rewindFilterTo(measurement.stamp, &replay_records,
                          &replay_present_stamp)) {
        ++ignored_localization_;
        ++delayed_rejection_count_;
        localization_source_manager_->reportCorrectionResult(
            measurement.config.name, false, ros::Time::now());
        ROS_WARN_THROTTLE(
            1.0,
            "[xd_uav_state_estimator] source '%s' delayed measurement is outside the available filter history",
            measurement.config.name.c_str());
        return;
      }
      delayed_replay = true;
      const double heading_delay =
          std::max((replay_present_stamp - measurement.stamp).toSec(), 0.0);
      heading_for_correction = HeadingFilter::wrapAngle(
          measured_heading + measurement.angular_velocity.z * heading_delay);
    } else if (filter_dt > 0.0) {
      bool acceleration_valid = false;
      std::string source;
      const Eigen::Vector3d acceleration =
          predictionAccelerationInOdom(measurement.stamp, orientation,
                                       &acceleration_valid, &source);
      predictTo(measurement.stamp, acceleration, acceleration_valid);
      prediction_source_ = acceleration_valid ? source : "model";
    }

    position_innovation_.setZero();
    velocity_innovation_.setZero();
    if (measurement.config.use_position_xy) {
      position_innovation_.head<2>() =
          position.head<2>() - filters_->position().head<2>();
    }
    if (measurement.config.use_altitude) {
      position_innovation_.z() = position.z() - filters_->position().z();
    }
    if (measurement.config.use_velocity_xy) {
      velocity_innovation_.head<2>() =
          velocity.head<2>() - filters_->velocity().head<2>();
    }
    if (measurement.config.use_vertical_velocity) {
      velocity_innovation_.z() = velocity.z() - filters_->velocity().z();
    }
    heading_innovation_ = 0.0;
    if (measurement.config.use_heading && heading_filter_->initialized()) {
      heading_filter_->predict(measurement.stamp);
      heading_innovation_ = HeadingFilter::wrapAngle(
          heading_for_correction - heading_filter_->heading());
    }

    position_xy_nis_ = 0.0;
    position_z_nis_ = 0.0;
    velocity_xy_nis_ = 0.0;
    velocity_z_nis_ = 0.0;
    heading_nis_ = 0.0;
    const Eigen::Matrix3d position_covariance = filters_->positionCovariance();
    const Eigen::Matrix3d velocity_covariance = filters_->velocityCovariance();
    if (measurement.config.use_position_xy) {
      for (int axis = 0; axis < 2; ++axis) {
        const double innovation_variance = std::max(
            position_covariance(axis, axis) +
                measurement.position_xy_variance,
            1e-12);
        position_xy_nis_ += position_innovation_(axis) *
                            position_innovation_(axis) /
                            innovation_variance;
      }
    }
    if (measurement.config.use_altitude) {
      const double innovation_variance = std::max(
          position_covariance(2, 2) + measurement.position_z_variance, 1e-12);
      position_z_nis_ = position_innovation_.z() * position_innovation_.z() /
                        innovation_variance;
    }
    if (measurement.config.use_velocity_xy) {
      for (int axis = 0; axis < 2; ++axis) {
        const double innovation_variance = std::max(
            velocity_covariance(axis, axis) +
                measurement.velocity_xy_variance,
            1e-12);
        velocity_xy_nis_ += velocity_innovation_(axis) *
                            velocity_innovation_(axis) /
                            innovation_variance;
      }
    }
    if (measurement.config.use_vertical_velocity) {
      const double innovation_variance = std::max(
          velocity_covariance(2, 2) + measurement.velocity_z_variance, 1e-12);
      velocity_z_nis_ = velocity_innovation_.z() * velocity_innovation_.z() /
                        innovation_variance;
    }
    if (measurement.config.use_heading && heading_filter_->initialized()) {
      const double innovation_variance = std::max(
          heading_filter_->headingVariance() + measurement.heading_variance,
          1e-12);
      heading_nis_ = heading_innovation_ * heading_innovation_ /
                     innovation_variance;
    }

    const bool absolute_outlier =
        (measurement.config.use_position_xy &&
         position_innovation_.head<2>().cwiseAbs().maxCoeff() >
             position_xy_innovation_limit_) ||
        (measurement.config.use_altitude &&
         std::abs(position_innovation_.z()) > position_z_innovation_limit_) ||
        (measurement.config.use_velocity_xy &&
         velocity_innovation_.head<2>().cwiseAbs().maxCoeff() >
             velocity_xy_innovation_limit_) ||
        (measurement.config.use_vertical_velocity &&
         std::abs(velocity_innovation_.z()) > velocity_z_innovation_limit_) ||
        (measurement.config.use_heading && heading_filter_->initialized() &&
         std::abs(heading_innovation_) > heading_innovation_limit_);
    const bool statistical_outlier = use_mahalanobis_gate_ &&
        ((measurement.config.use_position_xy &&
          position_xy_nis_ > position_xy_nis_limit_) ||
         (measurement.config.use_altitude &&
          position_z_nis_ > position_z_nis_limit_) ||
         (measurement.config.use_velocity_xy &&
          velocity_xy_nis_ > velocity_xy_nis_limit_) ||
         (measurement.config.use_vertical_velocity &&
          velocity_z_nis_ > velocity_z_nis_limit_) ||
         (measurement.config.use_heading && heading_filter_->initialized() &&
          heading_nis_ > heading_nis_limit_));
    const bool outlier = absolute_outlier || statistical_outlier;
    if (outlier) {
      ++rejected_odometry_;
      if (measurement.config.use_heading &&
          std::abs(heading_innovation_) > heading_innovation_limit_) {
        ++heading_rejection_count_;
      }
      localization_source_manager_->reportCorrectionResult(
          measurement.config.name, false, ros::Time::now());
      ROS_WARN_THROTTLE(1.0,
                        "[xd_uav_state_estimator] source '%s' rejected by innovation limits",
                        measurement.config.name.c_str());
      if (delayed_replay) {
        replayPredictions(replay_records, replay_present_stamp);
      }
      return;
    }

    // 延迟测量已经回退到其时间戳；普通测量则不能让滤波器时间戳倒退。
    const ros::Time correction_stamp = filters_->stamp() > measurement.stamp
                                           ? filters_->stamp()
                                           : measurement.stamp;
    if (measurement.config.use_position_xy) {
      filters_->correctLateralPosition(position.head<2>(),
                                       measurement.position_xy_variance,
                                       correction_stamp);
      ++correction_count_;
    }
    if (measurement.config.use_altitude) {
      filters_->correctAltitudePosition(position.z(),
                                        measurement.position_z_variance,
                                        correction_stamp);
      ++correction_count_;
    }
    if (measurement.config.use_velocity_xy) {
      filters_->correctLateralVelocity(velocity.head<2>(),
                                       measurement.velocity_xy_variance,
                                       correction_stamp);
      latest_odometry_velocity_odom_.head<2>() = velocity.head<2>();
      ++correction_count_;
    }
    if (measurement.config.use_vertical_velocity) {
      filters_->correctVerticalVelocity(velocity.z(),
                                        measurement.velocity_z_variance,
                                        correction_stamp);
      latest_odometry_velocity_odom_.z() = velocity.z();
      ++correction_count_;
    }
    if (measurement.config.use_heading) {
      if (!heading_filter_->initialized()) {
        initializeHeading(aligned_orientation, measurement.angular_velocity.z,
                          true, correction_stamp);
      } else {
        heading_filter_->correctHeading(heading_for_correction,
                                        measurement.heading_variance,
                                        correction_stamp);
      }
      ++heading_correction_count_;
    }

    recordFilterSnapshot();
    if (delayed_replay) {
      replayPredictions(replay_records, replay_present_stamp);
      ++delayed_correction_count_;
    }

    if (!filters_->finite() || !heading_filter_->finite()) {
      throw std::runtime_error("filter generated non-finite state or covariance");
    }

    last_odom_stamp_ = filters_->stamp();
    if (decision.authoritative) {
      last_odom_receive_time_ = measurement.receive_time;
      active_localization_timeout_ = measurement.config.timeout;
      have_odom_ = true;
    }
    odom_orientation_ = aligned_orientation;
    odom_angular_velocity_ = measurement.angular_velocity;
    latest_odometry_velocity_odom_ = filters_->velocity();
    healthy_ = true;
    localization_source_manager_->reportCorrectionResult(
        measurement.config.name, true, ros::Time::now());
    if (use_imu_prediction_ && imu_acceleration_processor_->calibrating()) {
      health_message_ = "calibrating IMU acceleration bias";
    } else if (use_imu_prediction_ && imu_acceleration_valid_) {
      health_message_ = "running with IMU prediction";
    } else {
      health_message_ = "running with model prediction";
    }
  } catch (const std::exception& exception) {
    ++rejected_odometry_;
    ++reset_count_;
    healthy_ = false;
    health_message_ = exception.what();
    localization_source_manager_->reportCorrectionResult(
        measurement.config.name, false, ros::Time::now());
    ROS_ERROR("[xd_uav_state_estimator] source '%s' correction failed: %s",
              measurement.config.name.c_str(), exception.what());
    filters_->reset();
    heading_filter_->reset();
    prediction_history_.clear();
    filter_history_.clear();
    have_odom_ = false;
  }
}

bool StateEstimatorNode::alignMeasurement(
    const LocalizationMeasurement& measurement,
    const LocalizationDecision& decision, Eigen::Vector3d* position,
    Eigen::Vector3d* velocity, geometry_msgs::Quaternion* orientation,
    bool* alignment_pending) {
  if (position == nullptr || velocity == nullptr || orientation == nullptr ||
      alignment_pending == nullptr) {
    return false;
  }
  *alignment_pending = false;

  tf2::Quaternion source_orientation;
  if (!normalizeQuaternion(measurement.orientation, &source_orientation)) {
    return false;
  }

  SourceAlignment& alignment = source_alignments_[measurement.config.name];
  const std::string source_parent_frame =
      withoutBoundarySlashes(measurement.parent_frame);
  const bool parent_frame_changed =
      !alignment.parent_frame.empty() &&
      alignment.parent_frame != source_parent_frame;
  if (parent_frame_changed) {
    alignment.initialized = false;
    alignment.translation_candidates.clear();
    alignment.heading_candidates.clear();
  }
  alignment.parent_frame = source_parent_frame;
  const bool align_on_activation =
      measurement.config.alignment_mode == AlignmentMode::kAlignOnActivation &&
      decision.active_source_changed;
  if (align_on_activation) {
    alignment.initialized = false;
    alignment.translation_candidates.clear();
    alignment.heading_candidates.clear();
  }
  const bool needs_alignment = !alignment.initialized || align_on_activation;

  if (measurement.config.alignment_mode == AlignmentMode::kIdentity) {
    alignment.rotation = tf2::Quaternion(0.0, 0.0, 0.0, 1.0);
    alignment.translation.setZero();
    alignment.initialized = true;
    alignment.translation_candidates.clear();
    alignment.heading_candidates.clear();
    if (!measurement.parent_frame.empty() &&
        withoutBoundarySlashes(measurement.parent_frame) != odom_frame_) {
      ROS_WARN_THROTTLE(2.0,
                        "[xd_uav_state_estimator] source '%s' uses identity alignment but parent frame is '%s', expected '%s'",
                        measurement.config.name.c_str(),
                        measurement.parent_frame.c_str(), odom_frame_.c_str());
    }
  } else if (needs_alignment) {
    if (!filters_->initialized()) {
      alignment.rotation = tf2::Quaternion(0.0, 0.0, 0.0, 1.0);
      alignment.translation.setZero();
      alignment.initialized = true;
    } else {
      geometry_msgs::Quaternion current_orientation_message = odom_orientation_;
      if (have_imu_) {
        current_orientation_message = latest_imu_.orientation;
      }
      tf2::Quaternion current_orientation;
      if (!normalizeQuaternion(current_orientation_message, &current_orientation)) {
        return false;
      }
      const double current_heading = heading_filter_->initialized()
                                         ? heading_filter_->heading()
                                         : quaternionYaw(current_orientation);
      const double source_heading = quaternionYaw(source_orientation);
      const double heading_candidate =
          HeadingFilter::wrapAngle(current_heading - source_heading);
      tf2::Quaternion candidate_rotation;
      candidate_rotation.setRPY(0.0, 0.0, heading_candidate);
      candidate_rotation.normalize();
      const Eigen::Vector3d translation_candidate =
          filters_->position() -
          rotateBodyToParent(measurement.position, candidate_rotation);
      alignment.heading_candidates.push_back(heading_candidate);
      alignment.translation_candidates.push_back(translation_candidate);

      if (static_cast<int>(alignment.heading_candidates.size()) <
          measurement.config.alignment_samples) {
        *alignment_pending = true;
        return false;
      }

      double sine_sum = 0.0;
      double cosine_sum = 0.0;
      for (const double candidate : alignment.heading_candidates) {
        sine_sum += std::sin(candidate);
        cosine_sum += std::cos(candidate);
      }
      const double aligned_heading = std::atan2(sine_sum, cosine_sum);
      alignment.rotation.setRPY(0.0, 0.0, aligned_heading);
      alignment.rotation.normalize();

      for (int axis = 0; axis < 3; ++axis) {
        std::vector<double> axis_candidates;
        axis_candidates.reserve(alignment.translation_candidates.size());
        for (const auto& candidate : alignment.translation_candidates) {
          axis_candidates.push_back(candidate(axis));
        }
        alignment.translation(axis) = median(std::move(axis_candidates));
      }
      alignment.translation_candidates.clear();
      alignment.heading_candidates.clear();
      alignment.initialized = true;
    }
    ROS_INFO("[xd_uav_state_estimator] aligned source '%s' to frame '%s'",
             measurement.config.name.c_str(), odom_frame_.c_str());
  }

  if (!measurement.child_frame.empty() &&
      withoutBoundarySlashes(measurement.child_frame) != body_frame_) {
    ROS_WARN_THROTTLE(2.0,
                      "[xd_uav_state_estimator] source '%s' child frame is '%s'; treating it as body frame '%s'",
                      measurement.config.name.c_str(),
                      measurement.child_frame.c_str(), body_frame_.c_str());
  }

  *position = rotateBodyToParent(measurement.position, alignment.rotation) +
              alignment.translation;
  *velocity = rotateBodyToParent(measurement.velocity, alignment.rotation);
  tf2::Quaternion aligned_orientation = alignment.rotation * source_orientation;
  aligned_orientation.normalize();
  *orientation = tf2::toMsg(aligned_orientation);
  return position->allFinite() && velocity->allFinite();
}

void StateEstimatorNode::imuCallback(const sensor_msgs::Imu::ConstPtr& message) {
  ++received_imu_;
  tf2::Quaternion orientation;
  if (!normalizeQuaternion(message->orientation, &orientation) ||
      !finite(message->angular_velocity) || !finite(message->linear_acceleration)) {
    ++rejected_imu_;
    ROS_WARN_THROTTLE(1.0, "[xd_uav_state_estimator] rejecting invalid IMU message");
    return;
  }

  std::scoped_lock lock(mutex_);
  const ros::Time stamp = message->header.stamp.isZero() ? ros::Time::now()
                                                         : message->header.stamp;
  latest_imu_ = *message;
  latest_imu_.orientation = tf2::toMsg(orientation);
  last_imu_receive_time_ = ros::Time::now();
  have_imu_ = true;

  Eigen::Vector3d specific_force_odom = vectorToEigen(message->linear_acceleration);
  const std::string& frame = message->header.frame_id;
  if (frame.empty() || frame == body_frame_) {
    specific_force_odom = rotateBodyToParent(specific_force_odom, orientation);
  } else if (frame != odom_frame_ && frame != map_frame_ && frame != world_frame_) {
    ++rejected_imu_;
    imu_acceleration_valid_ = false;
    ROS_WARN_THROTTLE(1.0,
                      "[xd_uav_state_estimator] IMU frame '%s' is unsupported for acceleration "
                      "prediction; expected '%s' or '%s'",
                      frame.c_str(), body_frame_.c_str(), odom_frame_.c_str());
    return;
  }

  const Eigen::Vector3d angular_velocity = vectorToEigen(message->angular_velocity);
  if (have_odom_) {
    imu_acceleration_valid_ = imu_acceleration_processor_->update(
        specific_force_odom, latest_odometry_velocity_odom_, angular_velocity,
        &latest_imu_acceleration_odom_);

    if (!imu_acceleration_valid_ && !imu_acceleration_processor_->calibrating()) {
      ++rejected_imu_;
      ROS_WARN_THROTTLE(1.0,
                        "[xd_uav_state_estimator] rejecting IMU acceleration outside configured "
                        "limits");
    }
  }

  if (!last_imu_stamp_.isZero() && stamp < last_imu_stamp_) {
    imu_acceleration_valid_ = false;
    ROS_WARN_THROTTLE(1.0,
                      "[xd_uav_state_estimator] IMU timestamp moved backwards; waiting for "
                      "odometry synchronization");
  }
  last_imu_stamp_ = stamp;

  if (use_imu_heading_rate_ && heading_filter_->initialized()) {
    const double heading_dt = (stamp - heading_filter_->stamp()).toSec();
    if (heading_dt > reset_gap_) {
      heading_filter_->initialize(heading_filter_->heading(),
                                  message->angular_velocity.z, stamp);
    } else if (heading_dt >= -max_localization_delay_) {
      double rate_variance = heading_rate_variance_;
      if (message->angular_velocity_covariance[0] >= 0.0 &&
          std::isfinite(message->angular_velocity_covariance[8]) &&
          message->angular_velocity_covariance[8] > 0.0) {
        rate_variance = message->angular_velocity_covariance[8];
      }
      heading_filter_->correctRate(message->angular_velocity.z,
                                   rate_variance, stamp);
    }
  }

  if (!use_imu_prediction_ || !filters_->initialized() || !imu_acceleration_valid_) {
    return;
  }

  geometry_msgs::AccelStamped processed_acceleration;
  processed_acceleration.header.stamp = stamp;
  processed_acceleration.header.frame_id = odom_frame_;
  processed_acceleration.accel.linear = eigenToVector(latest_imu_acceleration_odom_);
  imu_acceleration_publisher_.publish(processed_acceleration);

  const double dt = (stamp - filters_->stamp()).toSec();
  if (dt > reset_gap_) {
    healthy_ = false;
    health_message_ = "IMU prediction time gap";
    ROS_WARN_THROTTLE(1.0,
                      "[xd_uav_state_estimator] IMU prediction gap is %.3f s; waiting for "
                      "odometry reinitialization", dt);
    return;
  }

  if (dt > 0.0 && predictTo(stamp, latest_imu_acceleration_odom_, true)) {
    ++imu_prediction_count_;
    prediction_source_ = "imu";
  }
}

void StateEstimatorNode::controlInputCallback(const geometry_msgs::AccelStamped::ConstPtr& message) {
  if (!finite(message->accel.linear) || !finite(message->accel.angular)) {
    ROS_WARN_THROTTLE(1.0, "[xd_uav_state_estimator] rejecting invalid control input");
    return;
  }
  std::scoped_lock lock(mutex_);
  latest_control_input_ = *message;
  last_control_receive_time_ = ros::Time::now();
  have_control_input_ = true;
}

void StateEstimatorNode::initializeFilter(const Eigen::Vector3d& position,
                                          const Eigen::Vector3d& velocity,
                                          const ros::Time& stamp) {
  filters_->initialize(position, velocity, stamp);
  prediction_history_.clear();
  filter_history_.clear();
  recordFilterSnapshot();
  last_odom_stamp_ = stamp;
  position_innovation_.setZero();
  velocity_innovation_.setZero();
  healthy_ = !require_imu_ || have_imu_;
  prediction_source_ = "model";
  if (!healthy_) {
    health_message_ = "waiting for IMU";
  } else if (use_imu_prediction_ && imu_acceleration_processor_->calibrating()) {
    health_message_ = "calibrating IMU acceleration bias";
  } else {
    health_message_ = "running";
  }
  ROS_INFO("[xd_uav_state_estimator] filter initialized at [%.3f, %.3f, %.3f]",
           position.x(), position.y(), position.z());
}

void StateEstimatorNode::initializeHeading(
    const geometry_msgs::Quaternion& orientation_message,
    const double angular_rate, const bool use_source_heading,
    const ros::Time& stamp) {
  tf2::Quaternion orientation;
  double initial_rate = std::isfinite(angular_rate) ? angular_rate : 0.0;
  bool orientation_valid = false;

  if (use_source_heading) {
    orientation_valid = normalizeQuaternion(orientation_message, &orientation);
  }
  if (!orientation_valid && have_imu_) {
    orientation_valid = normalizeQuaternion(latest_imu_.orientation, &orientation);
    if (std::isfinite(latest_imu_.angular_velocity.z)) {
      initial_rate = latest_imu_.angular_velocity.z;
    }
  }
  if (!orientation_valid) {
    orientation.setRPY(0.0, 0.0, 0.0);
  }
  heading_filter_->initialize(quaternionYaw(orientation), initial_rate, stamp);
  heading_innovation_ = 0.0;
}

bool StateEstimatorNode::predictTo(const ros::Time& stamp,
                                   const Eigen::Vector3d& acceleration,
                                   const bool acceleration_valid) {
  if (!filters_->initialized() || !acceleration.allFinite()) {
    return false;
  }

  double remaining = (stamp - filters_->stamp()).toSec();
  if (!std::isfinite(remaining) || remaining <= 0.0 || remaining > reset_gap_) {
    return false;
  }

  ros::Time prediction_stamp = filters_->stamp();
  while (remaining > 1e-9) {
    const double step = std::min(remaining, max_prediction_step_);
    const ros::Time step_start = prediction_stamp;
    prediction_stamp += ros::Duration(step);
    filters_->predict(step, acceleration, acceleration_valid, prediction_stamp);
    PredictionRecord record;
    record.start_stamp = step_start;
    record.end_stamp = prediction_stamp;
    record.acceleration = acceleration;
    record.acceleration_valid = acceleration_valid;
    prediction_history_.push_back(record);
    recordFilterSnapshot();
    remaining -= step;
    if (!replaying_prediction_history_) {
      ++prediction_count_;
    }
  }
  pruneFilterHistory();
  return true;
}

void StateEstimatorNode::recordFilterSnapshot() {
  if (!filters_->initialized()) {
    return;
  }
  FilterHistoryEntry entry;
  entry.stamp = filters_->stamp();
  entry.snapshot = filters_->snapshot();
  filter_history_.push_back(entry);
}

void StateEstimatorNode::pruneFilterHistory() {
  if (filter_history_.empty()) {
    return;
  }
  const ros::Time newest_stamp = filter_history_.back().stamp;
  const ros::Time cutoff = newest_stamp.toSec() > filter_history_duration_
                               ? newest_stamp - ros::Duration(filter_history_duration_)
                               : ros::Time(0);
  while (filter_history_.size() > 1 && filter_history_[1].stamp < cutoff) {
    filter_history_.pop_front();
  }
  const ros::Time oldest_snapshot = filter_history_.front().stamp;
  while (!prediction_history_.empty() &&
         prediction_history_.front().end_stamp <= oldest_snapshot) {
    prediction_history_.pop_front();
  }
}

bool StateEstimatorNode::rewindFilterTo(
    const ros::Time& stamp, std::vector<PredictionRecord>* replay_records,
    ros::Time* present_stamp) {
  if (replay_records == nullptr || present_stamp == nullptr ||
      filter_history_.empty() || stamp > filters_->stamp()) {
    return false;
  }

  auto selected = filter_history_.end();
  for (auto iterator = filter_history_.begin();
       iterator != filter_history_.end(); ++iterator) {
    if (iterator->stamp <= stamp) {
      selected = iterator;
    } else {
      break;
    }
  }
  if (selected == filter_history_.end()) {
    return false;
  }

  *present_stamp = filters_->stamp();
  replay_records->clear();
  for (const auto& record : prediction_history_) {
    if (record.end_stamp > selected->stamp &&
        record.start_stamp < *present_stamp) {
      replay_records->push_back(record);
    }
  }

  const ros::Time selected_stamp = selected->stamp;
  filters_->restore(selected->snapshot);
  filter_history_.erase(std::next(selected), filter_history_.end());
  while (!prediction_history_.empty() &&
         prediction_history_.back().end_stamp > selected_stamp) {
    prediction_history_.pop_back();
  }

  replaying_prediction_history_ = true;
  for (const auto& record : *replay_records) {
    if (record.end_stamp <= filters_->stamp()) {
      continue;
    }
    const ros::Time segment_end = record.end_stamp < stamp
                                      ? record.end_stamp
                                      : stamp;
    if (segment_end > filters_->stamp()) {
      predictTo(segment_end, record.acceleration,
                record.acceleration_valid);
    }
    if (segment_end >= stamp) {
      break;
    }
  }
  if (filters_->stamp() < stamp) {
    predictTo(stamp, Eigen::Vector3d::Zero(), false);
  }
  replaying_prediction_history_ = false;
  return std::abs((filters_->stamp() - stamp).toSec()) < 1e-6;
}

void StateEstimatorNode::replayPredictions(
    const std::vector<PredictionRecord>& records,
    const ros::Time& target_stamp) {
  replaying_prediction_history_ = true;
  for (const auto& record : records) {
    if (record.end_stamp <= filters_->stamp()) {
      continue;
    }
    const ros::Time segment_end = record.end_stamp < target_stamp
                                      ? record.end_stamp
                                      : target_stamp;
    if (segment_end > filters_->stamp()) {
      predictTo(segment_end, record.acceleration,
                record.acceleration_valid);
    }
    if (segment_end >= target_stamp) {
      break;
    }
  }
  if (filters_->stamp() < target_stamp) {
    predictTo(target_stamp, Eigen::Vector3d::Zero(), false);
  }
  replaying_prediction_history_ = false;
  pruneFilterHistory();
}

Eigen::Vector3d StateEstimatorNode::predictionAccelerationInOdom(
    const ros::Time& stamp, const tf2::Quaternion& body_orientation,
    bool* valid, std::string* source) const {
  *valid = false;
  *source = "model";

  if (use_imu_prediction_ && have_imu_ && imu_acceleration_valid_) {
    const double age = (stamp - last_imu_stamp_).toSec();
    if (age >= -0.05 && age <= imu_timeout_) {
      *valid = true;
      *source = "imu";
      return latest_imu_acceleration_odom_;
    }
  }

  bool control_valid = false;
  const Eigen::Vector3d control =
      controlAccelerationInOdom(stamp, body_orientation, &control_valid);
  if (control_valid) {
    *valid = true;
    *source = "control";
  }
  return control;
}

Eigen::Vector3d StateEstimatorNode::controlAccelerationInOdom(
    const ros::Time& stamp, const tf2::Quaternion& body_orientation, bool* valid) const {
  *valid = false;
  if (!use_control_input_ || !have_control_input_) {
    return Eigen::Vector3d::Zero();
  }

  const ros::Time control_stamp = latest_control_input_.header.stamp.isZero()
                                      ? last_control_receive_time_
                                      : latest_control_input_.header.stamp;
  const double age = (stamp - control_stamp).toSec();
  if (age < -0.05 || age > control_input_timeout_) {
    return Eigen::Vector3d::Zero();
  }

  Eigen::Vector3d acceleration = vectorToEigen(latest_control_input_.accel.linear);
  const std::string& frame = latest_control_input_.header.frame_id;
  if (frame.empty() || frame == body_frame_) {
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    tf2::Matrix3x3(body_orientation).getRPY(roll, pitch, yaw);
    const double x = std::cos(yaw) * acceleration.x() - std::sin(yaw) * acceleration.y();
    const double y = std::sin(yaw) * acceleration.x() + std::cos(yaw) * acceleration.y();
    acceleration.x() = x;
    acceleration.y() = y;
  } else if (frame != odom_frame_ && frame != map_frame_ && frame != world_frame_) {
    ROS_WARN_THROTTLE(1.0, "[xd_uav_state_estimator] unsupported control-input frame '%s'",
                      frame.c_str());
    return Eigen::Vector3d::Zero();
  }

  *valid = true;
  return acceleration;
}

void StateEstimatorNode::publishTimerCallback(const ros::TimerEvent& event) {
  (void)event;
  publishState();
}

void StateEstimatorNode::diagnosticsTimerCallback(const ros::TimerEvent& event) {
  (void)event;
  publishDiagnostics();
}

bool StateEstimatorNode::publishEstimatorStatus(
    const ros::Time& now, const bool localization_valid) {
  EstimatorStatus status;
  status.header.stamp = now;
  status.localization_valid = localization_valid;
  status.active_source = localization_source_manager_->activeSource();
  status.localization_age = have_odom_
                                ? nonNegativeAge(now, last_odom_receive_time_)
                                : -1.0;
  status.dead_reckoning_age = have_odom_
                                  ? std::max(status.localization_age -
                                                 active_localization_timeout_,
                                             0.0)
                                  : -1.0;
  status.source_switches = localization_source_manager_->switchCount();

  const bool filters_valid = filters_->initialized() && filters_->finite() &&
                             heading_filter_->initialized() &&
                             heading_filter_->finite();
  const bool imu_fresh = have_imu_ &&
                         nonNegativeAge(now, last_imu_receive_time_) <=
                             imu_timeout_;
  if (!filters_->initialized()) {
    status.state = EstimatorStatus::WAITING;
    status.failure_reason = "waiting for first localization measurement";
  } else if (!filters_valid) {
    status.state = EstimatorStatus::LOST;
    status.failure_reason = "non-finite estimator state";
  } else if (require_imu_ && !imu_fresh) {
    status.state = EstimatorStatus::LOST;
    status.failure_reason = "required IMU is unavailable";
  } else if (localization_valid) {
    if (!imu_fresh ||
        (use_imu_prediction_ && imu_acceleration_processor_->calibrating())) {
      status.state = EstimatorStatus::DEGRADED;
      status.failure_reason = !imu_fresh
                                  ? "IMU unavailable; using model prediction"
                                  : "IMU bias calibration is incomplete";
    } else {
      status.state = EstimatorStatus::RUNNING;
      status.failure_reason.clear();
    }
  } else if (have_odom_ && status.dead_reckoning_age >= 0.0 &&
             status.dead_reckoning_age <= max_dead_reckoning_time_) {
    status.state = EstimatorStatus::DEAD_RECKONING;
    status.failure_reason = "localization unavailable; prediction only";
  } else {
    status.state = EstimatorStatus::LOST;
    status.failure_reason = "localization loss exceeded prediction limit";
  }

  status.state_name = estimatorStateName(status.state);
  status.state_valid = status.state == EstimatorStatus::RUNNING ||
                       status.state == EstimatorStatus::DEGRADED ||
                       status.state == EstimatorStatus::DEAD_RECKONING;

  std_msgs::Bool localization_valid_message;
  localization_valid_message.data = status.localization_valid;
  localization_valid_publisher_.publish(localization_valid_message);
  std_msgs::Bool state_valid_message;
  state_valid_message.data = status.state_valid;
  state_valid_publisher_.publish(state_valid_message);
  estimator_status_publisher_.publish(status);
  return status.state_valid;
}

void StateEstimatorNode::publishState() {
  std::scoped_lock lock(mutex_);
  const ros::Time now = ros::Time::now();
  publishSourceValidity(now);
  const bool localization_valid =
      filters_->initialized() && filters_->finite() &&
      heading_filter_->initialized() && heading_filter_->finite() &&
      localization_source_manager_->localizationValid(now) && have_odom_ &&
      nonNegativeAge(now, last_odom_receive_time_) <=
          active_localization_timeout_;
  const bool state_valid = publishEstimatorStatus(now, localization_valid);
  if (!filters_->initialized() || !state_valid) {
    return;
  }

  if (!localization_valid) {
    healthy_ = false;
    health_message_ = "localization unavailable";
  }
  if (require_imu_ && (!have_imu_ || nonNegativeAge(now, last_imu_receive_time_) > imu_timeout_)) {
    healthy_ = false;
    health_message_ = "IMU timeout";
    return;
  }

  geometry_msgs::Quaternion orientation_message = odom_orientation_;
  geometry_msgs::Vector3 angular_velocity = odom_angular_velocity_;
  boost::array<double, 9> orientation_covariance{};
  if (have_imu_ && nonNegativeAge(now, last_imu_receive_time_) <= imu_timeout_) {
    orientation_message = latest_imu_.orientation;
    angular_velocity = latest_imu_.angular_velocity;
    orientation_covariance = latest_imu_.orientation_covariance;
  }

  tf2::Quaternion orientation;
  if (!normalizeQuaternion(orientation_message, &orientation)) {
    return;
  }
  if (heading_filter_->initialized()) {
    orientation = quaternionWithYaw(orientation, heading_filter_->heading());
    angular_velocity.z = heading_filter_->rate();
    orientation_covariance[8] = heading_filter_->headingVariance();
  }

  const Eigen::Vector3d position = filters_->position();
  const Eigen::Vector3d velocity_parent = filters_->velocity();
  const Eigen::Vector3d acceleration_parent = filters_->acceleration();
  const Eigen::Vector3d velocity_output = output_twist_in_body_frame_
                                              ? rotateParentToBody(velocity_parent, orientation)
                                              : velocity_parent;

  nav_msgs::Odometry odometry;
  odometry.header.stamp = filters_->stamp();
  odometry.header.frame_id = odom_frame_;
  odometry.child_frame_id = body_frame_;
  odometry.pose.pose.position.x = position.x();
  odometry.pose.pose.position.y = position.y();
  odometry.pose.pose.position.z = position.z();
  odometry.pose.pose.orientation = tf2::toMsg(orientation);
  odometry.twist.twist.linear = eigenToVector(velocity_output);
  odometry.twist.twist.angular = angular_velocity;

  const Eigen::Matrix3d position_covariance = filters_->positionCovariance();
  odometry.pose.covariance[0] = position_covariance(0, 0);
  odometry.pose.covariance[7] = position_covariance(1, 1);
  odometry.pose.covariance[14] = position_covariance(2, 2);
  odometry.pose.covariance[21] = orientation_covariance[0];
  odometry.pose.covariance[28] = orientation_covariance[4];
  odometry.pose.covariance[35] = orientation_covariance[8];

  Eigen::Matrix3d velocity_covariance = filters_->velocityCovariance();
  if (output_twist_in_body_frame_) {
    const tf2::Matrix3x3 rotation(orientation.inverse());
    Eigen::Matrix3d eigen_rotation;
    for (int row = 0; row < 3; ++row) {
      for (int column = 0; column < 3; ++column) {
        eigen_rotation(row, column) = rotation[row][column];
      }
    }
    velocity_covariance = eigen_rotation * velocity_covariance * eigen_rotation.transpose();
  }
  odometry.twist.covariance[0] = velocity_covariance(0, 0);
  odometry.twist.covariance[7] = velocity_covariance(1, 1);
  odometry.twist.covariance[14] = velocity_covariance(2, 2);
  if (have_imu_) {
    odometry.twist.covariance[21] = latest_imu_.angular_velocity_covariance[0];
    odometry.twist.covariance[28] = latest_imu_.angular_velocity_covariance[4];
    odometry.twist.covariance[35] = latest_imu_.angular_velocity_covariance[8];
  }

  geometry_msgs::AccelStamped acceleration;
  acceleration.header.stamp = filters_->stamp();
  acceleration.header.frame_id = odom_frame_;
  acceleration.accel.linear = eigenToVector(acceleration_parent);

  nav_msgs::Odometry innovation;
  innovation.header.stamp = filters_->stamp();
  innovation.header.frame_id = odom_frame_;
  innovation.child_frame_id = body_frame_;
  innovation.pose.pose.orientation.w = 1.0;
  innovation.pose.pose.position.x = position_innovation_.x();
  innovation.pose.pose.position.y = position_innovation_.y();
  innovation.pose.pose.position.z = position_innovation_.z();
  innovation.twist.twist.linear = eigenToVector(velocity_innovation_);

  main_odometry_publisher_.publish(odometry);
  acceleration_publisher_.publish(acceleration);
  innovation_publisher_.publish(innovation);

  if (publish_tf_) {
    geometry_msgs::TransformStamped transform;
    transform.header = odometry.header;
    transform.child_frame_id = body_frame_;
    transform.transform.translation.x = position.x();
    transform.transform.translation.y = position.y();
    transform.transform.translation.z = position.z();
    transform.transform.rotation = odometry.pose.pose.orientation;
    transform_broadcaster_.sendTransform(transform);
  }
  publishSourceOriginTransforms(odometry.header.stamp);
}

void StateEstimatorNode::publishDiagnostics() {
  std::scoped_lock lock(mutex_);
  const ros::Time now = ros::Time::now();
  const double odometry_age = have_odom_ ? nonNegativeAge(now, last_odom_receive_time_) : -1.0;
  const double imu_age = have_imu_ ? nonNegativeAge(now, last_imu_receive_time_) : -1.0;
  const bool localization_valid =
      filters_->initialized() && filters_->finite() &&
      heading_filter_->initialized() && heading_filter_->finite() &&
      localization_source_manager_->localizationValid(now) && have_odom_ &&
      odometry_age <= active_localization_timeout_;

  diagnostic_msgs::DiagnosticStatus status;
  status.name = uav_name_ + "/state_estimator";
  status.hardware_id = uav_name_;
  if (!filters_->initialized()) {
    status.level = diagnostic_msgs::DiagnosticStatus::WARN;
    status.message = "waiting for first odometry";
  } else if (!localization_valid) {
    status.level = diagnostic_msgs::DiagnosticStatus::ERROR;
    status.message = !filters_->finite() ? "non-finite filter state"
                                        : "localization unavailable";
  } else if (!have_imu_ || imu_age > imu_timeout_) {
    status.level = require_imu_ ? diagnostic_msgs::DiagnosticStatus::ERROR
                                : diagnostic_msgs::DiagnosticStatus::WARN;
    status.message = "IMU unavailable; using odometry attitude";
  } else if (use_imu_prediction_ && imu_acceleration_processor_->calibrating()) {
    status.level = diagnostic_msgs::DiagnosticStatus::WARN;
    status.message = "calibrating IMU acceleration bias";
  } else if (use_imu_prediction_ && !imu_acceleration_valid_) {
    status.level = diagnostic_msgs::DiagnosticStatus::WARN;
    status.message = "IMU acceleration unavailable; using model prediction";
  } else {
    status.level = healthy_ ? diagnostic_msgs::DiagnosticStatus::OK
                            : diagnostic_msgs::DiagnosticStatus::WARN;
    status.message = health_message_;
  }

  const auto add_value = [&status](const std::string& key, const std::string& value) {
    diagnostic_msgs::KeyValue entry;
    entry.key = key;
    entry.value = value;
    status.values.push_back(entry);
  };
  add_value("initialized", filters_->initialized() ? "true" : "false");
  add_value("localization_valid", localization_valid ? "true" : "false");
  add_value("odom_age_s", numberString(odometry_age));
  add_value("imu_age_s", numberString(imu_age));
  add_value("received_odometry", std::to_string(received_odometry_));
  add_value("rejected_odometry", std::to_string(rejected_odometry_));
  add_value("ignored_localization", std::to_string(ignored_localization_));
  add_value("source_selection",
            localization_source_manager_->automaticSelection() ? "automatic"
                                                                : "manual");
  add_value("requested_localization_source",
            localization_source_manager_->requestedSource());
  add_value("active_localization_source",
            localization_source_manager_->activeSource());
  add_value("active_localization_timeout_s",
            numberString(active_localization_timeout_));
  add_value("localization_source_switches",
            std::to_string(localization_source_manager_->switchCount()));
  add_value("received_imu", std::to_string(received_imu_));
  add_value("rejected_imu", std::to_string(rejected_imu_));
  add_value("imu_predictions", std::to_string(imu_prediction_count_));
  add_value("predictions", std::to_string(prediction_count_));
  add_value("corrections", std::to_string(correction_count_));
  add_value("heading_corrections", std::to_string(heading_correction_count_));
  add_value("heading_rejections", std::to_string(heading_rejection_count_));
  add_value("heading_initialized",
            heading_filter_->initialized() ? "true" : "false");
  if (heading_filter_->initialized()) {
    add_value("heading_rad", numberString(heading_filter_->heading()));
    add_value("heading_rate_rad_s", numberString(heading_filter_->rate()));
    add_value("heading_variance",
              numberString(heading_filter_->headingVariance()));
    add_value("heading_innovation_rad", numberString(heading_innovation_));
  }
  add_value("resets", std::to_string(reset_count_));
  add_value("world_frame", world_frame_);
  add_value("map_frame", map_frame_);
  add_value("odom_frame", odom_frame_);
  add_value("body_frame", body_frame_);
  add_value("prediction_source", prediction_source_);
  add_value("imu_prediction_enabled", use_imu_prediction_ ? "true" : "false");
  add_value("imu_acceleration_valid", imu_acceleration_valid_ ? "true" : "false");
  add_value("imu_bias_ready", imu_acceleration_processor_->biasReady() ? "true" : "false");
  add_value("imu_bias_samples",
            std::to_string(imu_acceleration_processor_->biasSampleCount()) + "/" +
                std::to_string(imu_acceleration_processor_->requiredBiasSamples()));
  const Eigen::Vector3d& imu_bias = imu_acceleration_processor_->bias();
  add_value("imu_bias_x", numberString(imu_bias.x()));
  add_value("imu_bias_y", numberString(imu_bias.y()));
  add_value("imu_bias_z", numberString(imu_bias.z()));
  add_value("control_input_enabled", use_control_input_ ? "true" : "false");
  add_value("publish_tf", publish_tf_ ? "true" : "false");
  add_value("publish_source_origin_tf",
            publish_source_origin_tf_ ? "true" : "false");
  add_value("position_xy_nis", numberString(position_xy_nis_));
  add_value("position_z_nis", numberString(position_z_nis_));
  add_value("velocity_xy_nis", numberString(velocity_xy_nis_));
  add_value("velocity_z_nis", numberString(velocity_z_nis_));
  add_value("heading_nis", numberString(heading_nis_));
  add_value("mahalanobis_gate_enabled",
            use_mahalanobis_gate_ ? "true" : "false");
  add_value("delayed_measurement_enabled",
            delayed_measurement_enabled_ ? "true" : "false");
  add_value("delayed_corrections",
            std::to_string(delayed_correction_count_));
  add_value("delayed_rejections",
            std::to_string(delayed_rejection_count_));
  add_value("filter_history_entries",
            std::to_string(filter_history_.size()));

  diagnostic_msgs::DiagnosticArray diagnostics;
  diagnostics.header.stamp = now;
  diagnostics.status.push_back(status);

  for (const auto& source : localization_source_manager_->statuses(now)) {
    diagnostic_msgs::DiagnosticStatus source_status;
    source_status.name = uav_name_ + "/state_estimator/source/" + source.config.name;
    source_status.hardware_id = uav_name_;
    if (!source.config.enabled) {
      source_status.level = diagnostic_msgs::DiagnosticStatus::OK;
      source_status.message = "disabled";
    } else if (source.quarantined) {
      source_status.level = diagnostic_msgs::DiagnosticStatus::WARN;
      source_status.message = "quarantined after rejected corrections";
    } else if (!source.reliable) {
      source_status.level = diagnostic_msgs::DiagnosticStatus::WARN;
      source_status.message = "waiting for stable recovery samples";
    } else if (!source.healthy) {
      source_status.level = diagnostic_msgs::DiagnosticStatus::WARN;
      source_status.message = source.received == 0 ? "waiting for data" : "timeout";
    } else {
      source_status.level = diagnostic_msgs::DiagnosticStatus::OK;
      source_status.message = source.active ? "active" : "standby";
    }

    const auto add_source_value =
        [&source_status](const std::string& key, const std::string& value) {
          diagnostic_msgs::KeyValue entry;
          entry.key = key;
          entry.value = value;
          source_status.values.push_back(entry);
        };
    add_source_value("topic", source.config.topic);
    add_source_value("adapter", source.config.adapter);
    add_source_value("role", sourceRoleName(source.config.role));
    add_source_value("priority", std::to_string(source.config.priority));
    add_source_value("active", source.active ? "true" : "false");
    add_source_value("healthy", source.healthy ? "true" : "false");
    add_source_value("reliable", source.reliable ? "true" : "false");
    add_source_value("quarantined", source.quarantined ? "true" : "false");
    add_source_value("lifecycle_state",
                     sourceLifecycleStateName(source.lifecycle_state));
    add_source_value("age_s", numberString(source.age));
    add_source_value("timeout_s", numberString(source.config.timeout));
    add_source_value("received", std::to_string(source.received));
    add_source_value("rejected", std::to_string(source.rejected));
    add_source_value("accepted_corrections",
                     std::to_string(source.accepted_corrections));
    add_source_value("rejected_corrections",
                     std::to_string(source.rejected_corrections));
    add_source_value("consecutive_rejections",
                     std::to_string(source.consecutive_rejections));
    add_source_value("recovery_samples",
                     std::to_string(source.recovery_samples));
    add_source_value("use_position_xy",
                     source.config.use_position_xy ? "true" : "false");
    add_source_value("use_altitude",
                     source.config.use_altitude ? "true" : "false");
    add_source_value("use_velocity_xy",
                     source.config.use_velocity_xy ? "true" : "false");
    add_source_value("use_vertical_velocity",
                     source.config.use_vertical_velocity ? "true" : "false");
    add_source_value("use_heading",
                     source.config.use_heading ? "true" : "false");
    add_source_value("alignment_mode",
                     alignmentModeName(source.config.alignment_mode));
    diagnostics.status.push_back(source_status);
  }
  diagnostics_publisher_.publish(diagnostics);
}

bool StateEstimatorNode::resetCallback(std_srvs::Trigger::Request& request,
                                       std_srvs::Trigger::Response& response) {
  (void)request;
  std::scoped_lock lock(mutex_);
  filters_->reset();
  heading_filter_->reset();
  prediction_history_.clear();
  filter_history_.clear();
  last_odom_stamp_ = ros::Time(0);
  last_odom_receive_time_ = ros::Time(0);
  last_source_stamps_.clear();
  source_alignments_.clear();
  for (auto& item : source_estimators_) {
    item.second->filters.reset();
    item.second->heading_filter.reset();
    item.second->last_receive_time = ros::Time(0);
    std_msgs::Bool valid;
    valid.data = false;
    item.second->valid_publisher.publish(valid);
  }
  have_odom_ = false;
  active_localization_timeout_ = input_timeout_;
  healthy_ = false;
  health_message_ = "waiting for odometry after reset";
  heading_innovation_ = 0.0;
  position_xy_nis_ = 0.0;
  position_z_nis_ = 0.0;
  velocity_xy_nis_ = 0.0;
  velocity_z_nis_ = 0.0;
  heading_nis_ = 0.0;
  ++reset_count_;
  response.success = true;
  response.message = "state estimator reset; next odometry initializes it";
  return true;
}

bool StateEstimatorNode::switchSourceCallback(
    SwitchLocalizationSource::Request& request,
    SwitchLocalizationSource::Response& response) {
  std::scoped_lock lock(mutex_);
  response.success = localization_source_manager_->requestActiveSource(
      request.source_name, ros::Time::now(), &response.message);
  response.active_source = localization_source_manager_->activeSource();
  response.requested_source =
      localization_source_manager_->requestedSource();
  response.automatic = localization_source_manager_->automaticSelection();
  if (response.success) {
    ROS_INFO("[xd_uav_state_estimator] %s", response.message.c_str());
  } else {
    ROS_WARN("[xd_uav_state_estimator] source switch rejected: %s",
             response.message.c_str());
  }
  return true;
}

bool StateEstimatorNode::finite(const double value) {
  return std::isfinite(value);
}

bool StateEstimatorNode::finite(const geometry_msgs::Vector3& vector) {
  return finite(vector.x) && finite(vector.y) && finite(vector.z);
}

bool StateEstimatorNode::finite(const geometry_msgs::Point& point) {
  return finite(point.x) && finite(point.y) && finite(point.z);
}

bool StateEstimatorNode::normalizeQuaternion(const geometry_msgs::Quaternion& message,
                                             tf2::Quaternion* quaternion) {
  if (!finite(message.x) || !finite(message.y) || !finite(message.z) || !finite(message.w)) {
    return false;
  }
  quaternion->setValue(message.x, message.y, message.z, message.w);
  const double norm_squared = quaternion->length2();
  if (!std::isfinite(norm_squared) || norm_squared < 1e-12) {
    return false;
  }
  quaternion->normalize();
  return true;
}

Eigen::Vector3d StateEstimatorNode::vectorToEigen(const geometry_msgs::Vector3& vector) {
  return {vector.x, vector.y, vector.z};
}

geometry_msgs::Vector3 StateEstimatorNode::eigenToVector(const Eigen::Vector3d& vector) {
  geometry_msgs::Vector3 message;
  message.x = vector.x();
  message.y = vector.y();
  message.z = vector.z();
  return message;
}

Eigen::Vector3d StateEstimatorNode::rotateBodyToParent(const Eigen::Vector3d& vector,
                                                       const tf2::Quaternion& orientation) {
  const tf2::Vector3 rotated = tf2::quatRotate(orientation, tf2::Vector3(vector.x(), vector.y(), vector.z()));
  return {rotated.x(), rotated.y(), rotated.z()};
}

Eigen::Vector3d StateEstimatorNode::rotateParentToBody(const Eigen::Vector3d& vector,
                                                       const tf2::Quaternion& orientation) {
  const tf2::Vector3 rotated = tf2::quatRotate(orientation.inverse(),
                                               tf2::Vector3(vector.x(), vector.y(), vector.z()));
  return {rotated.x(), rotated.y(), rotated.z()};
}

}  // 命名空间 xd_uav_state_estimator
