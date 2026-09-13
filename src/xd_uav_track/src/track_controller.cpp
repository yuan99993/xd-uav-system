#include <xd_uav_track/track_controller.hpp>

#include <algorithm>
#include <cmath>

namespace xd_uav_track {

namespace {

bool finiteMeasurement(const TargetMeasurement& value) {
  return std::isfinite(value.receive_time) &&
      std::isfinite(value.observation_time) && std::isfinite(value.x_min) &&
         std::isfinite(value.y_min) && std::isfinite(value.x_max) &&
         std::isfinite(value.y_max) && std::isfinite(value.confidence) &&
         std::isfinite(value.tracking_quality) &&
         std::all_of(value.state_covariance.begin(),
                     value.state_covariance.end(),
                     [](double v) { return std::isfinite(v); });
}

std::array<double, 3> bodyFrdToWorldEnu(
    const std::array<double, 3>& body_frd, const VehicleState& state) {
  // Body FRD -> body FLU, then rotate with the complete vehicle attitude.
  // A fixed-wing orbit can carry 30--35 deg of bank; using yaw only here
  // projects the target onto the wrong ground point and slowly drifts the
  // world-frame orbit centre away from the real target.
  const double forward = body_frd[0];
  const double left = -body_frd[1];
  const double up = -body_frd[2];
  const double cr = std::cos(state.roll);
  const double sr = std::sin(state.roll);
  const double cp = std::cos(state.pitch);
  const double sp = std::sin(state.pitch);
  const double cy = std::cos(state.yaw);
  const double sy = std::sin(state.yaw);
  return {{state.x + cy * cp * forward +
                    (cy * sp * sr - sy * cr) * left +
                    (cy * sp * cr + sy * sr) * up,
           state.y + sy * cp * forward +
                    (sy * sp * sr + cy * cr) * left +
                    (sy * sp * cr - cy * sr) * up,
           state.z - sp * forward + cp * sr * left + cp * cr * up}};
}

std::array<double, 3> worldEnuToBodyFrd(
    const std::array<double, 3>& world, const VehicleState& state) {
  const double dx = world[0] - state.x;
  const double dy = world[1] - state.y;
  const double dz = world[2] - state.z;
  const double cr = std::cos(state.roll);
  const double sr = std::sin(state.roll);
  const double cp = std::cos(state.pitch);
  const double sp = std::sin(state.pitch);
  const double cy = std::cos(state.yaw);
  const double sy = std::sin(state.yaw);
  // R(roll,pitch,yaw)^T * (world - vehicle), then FLU -> FRD.
  const double forward = cy * cp * dx + sy * cp * dy - sp * dz;
  const double left = (cy * sp * sr - sy * cr) * dx +
      (sy * sp * sr + cy * cr) * dy + cp * sr * dz;
  const double up = (cy * sp * cr + sy * sr) * dx +
      (sy * sp * cr - cy * sr) * dy + cp * cr * dz;
  return {{forward, -left, -up}};
}

std::array<double, 3> bodyFrdVectorToWorldEnu(
    const std::array<double, 3>& body_frd, const VehicleState& state) {
  const double forward = body_frd[0];
  const double left = -body_frd[1];
  const double up = -body_frd[2];
  const double cr = std::cos(state.roll);
  const double sr = std::sin(state.roll);
  const double cp = std::cos(state.pitch);
  const double sp = std::sin(state.pitch);
  const double cy = std::cos(state.yaw);
  const double sy = std::sin(state.yaw);
  return {{cy * cp * forward +
                (cy * sp * sr - sy * cr) * left +
                (cy * sp * cr + sy * sr) * up,
           sy * cp * forward +
                (sy * sp * sr + cy * cr) * left +
                (sy * sp * cr - cy * sr) * up,
           -sp * forward + cp * sr * left + cp * cr * up}};
}

std::array<double, 3> worldEnuVectorToBodyFrd(
    const std::array<double, 3>& world, const VehicleState& state) {
  const double cr = std::cos(state.roll);
  const double sr = std::sin(state.roll);
  const double cp = std::cos(state.pitch);
  const double sp = std::sin(state.pitch);
  const double cy = std::cos(state.yaw);
  const double sy = std::sin(state.yaw);
  const double forward = cy * cp * world[0] + sy * cp * world[1] -
      sp * world[2];
  const double left = (cy * sp * sr - sy * cr) * world[0] +
      (sy * sp * sr + cy * cr) * world[1] + cp * sr * world[2];
  const double up = (cy * sp * cr + sy * sr) * world[0] +
      (sy * sp * cr - cy * sr) * world[1] + cp * cr * world[2];
  return {{forward, -left, -up}};
}

}  // namespace

bool parseFollowerProfile(const std::string& value, FollowerProfile* profile) {
  if (profile == nullptr) return false;
  if (value == "mc_velocity_ground") {
    *profile = FollowerProfile::kVelocityGround;
    return true;
  }
  if (value == "mc_velocity_position") {
    *profile = FollowerProfile::kVelocityPosition;
    return true;
  }
  if (value == "mc_velocity_distance") {
    *profile = FollowerProfile::kVelocityDistance;
    return true;
  }
  if (value == "mc_velocity_chase") {
    *profile = FollowerProfile::kVelocityChase;
    return true;
  }
  if (value == "gm_velocity_chase") {
    *profile = FollowerProfile::kGimbalVelocityChase;
    return true;
  }
  if (value == "gm_velocity_vector") {
    *profile = FollowerProfile::kGimbalVelocityVector;
    return true;
  }
  if (value == "fw_velocity_vector") {
    *profile = FollowerProfile::kFixedWingVelocityVector;
    return true;
  }
  return false;
}

const char* followerProfileName(const FollowerProfile profile) {
  switch (profile) {
    case FollowerProfile::kVelocityGround: return "mc_velocity_ground";
    case FollowerProfile::kVelocityPosition: return "mc_velocity_position";
    case FollowerProfile::kVelocityDistance: return "mc_velocity_distance";
    case FollowerProfile::kVelocityChase: return "mc_velocity_chase";
    case FollowerProfile::kGimbalVelocityChase: return "gm_velocity_chase";
    case FollowerProfile::kGimbalVelocityVector: return "gm_velocity_vector";
    case FollowerProfile::kFixedWingVelocityVector:
      return "fw_velocity_vector";
  }
  return "mc_velocity_chase";
}

bool parseLateralGuidanceMode(const std::string& value,
                              LateralGuidanceMode* mode) {
  if (mode == nullptr) return false;
  if (value == "coordinated_turn") {
    *mode = LateralGuidanceMode::kCoordinatedTurn;
    return true;
  }
  if (value == "sideslip") {
    *mode = LateralGuidanceMode::kSideslip;
    return true;
  }
  return false;
}

const char* lateralGuidanceModeName(const LateralGuidanceMode mode) {
  return mode == LateralGuidanceMode::kSideslip ? "sideslip"
                                                : "coordinated_turn";
}

double TrackController::clamp(const double value, const double minimum,
                              const double maximum) {
  return std::max(minimum, std::min(maximum, value));
}

double TrackController::applyDeadband(const double value,
                                      const double threshold) {
  if (std::abs(value) <= threshold) return 0.0;
  return std::copysign(std::abs(value) - threshold, value);
}

double TrackController::moveToward(const double value, const double target,
                                   const double maximum_delta) {
  if (maximum_delta <= 0.0) return target;
  return value + clamp(target - value, -maximum_delta, maximum_delta);
}

void TrackController::Pid::configure(const PidConfig& requested,
                                     const double output_limit) {
  config_ = requested;
  config_.integral_limit = std::max(0.0, config_.integral_limit);
  output_limit_ = std::max(0.0, output_limit);
  reset();
}

double TrackController::Pid::update(const double error, const double dt) {
  if (!std::isfinite(error)) return 0.0;
  const double safe_dt = std::isfinite(dt) && dt > 1e-4 && dt <= 0.5
                             ? dt : 0.0;
  const double derivative = initialized_ && safe_dt > 0.0
                                ? (error - previous_error_) / safe_dt : 0.0;
  double candidate_integral = integral_;
  if (safe_dt > 0.0) {
    candidate_integral = TrackController::clamp(
        integral_ + config_.ki * error * safe_dt,
        -config_.integral_limit, config_.integral_limit);
  }
  const double unsaturated = config_.kp * error + candidate_integral +
                             config_.kd * derivative;
  const double output = TrackController::clamp(
      unsaturated, -output_limit_, output_limit_);

  // Conditional integration: do not accumulate while the error pushes farther
  // into saturation, but allow it to unwind immediately.
  const bool pushes_upper = unsaturated > output_limit_ && error > 0.0;
  const bool pushes_lower = unsaturated < -output_limit_ && error < 0.0;
  if (!pushes_upper && !pushes_lower) integral_ = candidate_integral;
  previous_error_ = error;
  initialized_ = true;
  return output;
}

void TrackController::Pid::reset() {
  integral_ = 0.0;
  previous_error_ = 0.0;
  initialized_ = false;
}

TrackController::TrackController(const TrackControllerConfig& requested)
    : config_(requested) {
  config_.input_timeout_sec = std::max(0.02, config_.input_timeout_sec);
  config_.minimum_confidence = clamp(config_.minimum_confidence, 0.0, 1.0);
  config_.confidence_hysteresis = clamp(
      config_.confidence_hysteresis, 0.0, config_.minimum_confidence);
  config_.position_filter_alpha = clamp(config_.position_filter_alpha, 0.0, 1.0);
  config_.velocity_filter_alpha = clamp(config_.velocity_filter_alpha, 0.0, 1.0);
  config_.target_x = clamp(config_.target_x, -1.0, 1.0);
  config_.target_y = clamp(config_.target_y, -1.0, 1.0);
  config_.horizontal_deadband = clamp(config_.horizontal_deadband, 0.0, 1.0);
  config_.vertical_deadband = clamp(config_.vertical_deadband, 0.0, 1.0);
  config_.chase_forward_velocity = std::max(0.0,
      config_.chase_forward_velocity);
  config_.initial_forward_velocity = std::max(0.0,
      config_.initial_forward_velocity);
  config_.target_loss_forward_velocity = std::max(0.0,
      config_.target_loss_forward_velocity);
  config_.forward_ramp_rate = std::max(0.0, config_.forward_ramp_rate);
  config_.target_loss_reset_sec = std::max(0.0,
      config_.target_loss_reset_sec);
  config_.maximum_forward_velocity = std::max(0.0,
      config_.maximum_forward_velocity);
  config_.maximum_reverse_velocity = std::max(0.0,
      config_.maximum_reverse_velocity);
  config_.maximum_lateral_velocity = std::max(0.0,
      config_.maximum_lateral_velocity);
  config_.maximum_vertical_velocity = std::max(0.0,
      config_.maximum_vertical_velocity);
  config_.maximum_yaw_rate = std::max(0.0, config_.maximum_yaw_rate);
  config_.velocity_ema_alpha = clamp(config_.velocity_ema_alpha, 0.0, 1.0);
  config_.yaw_deadzone = std::max(0.0, config_.yaw_deadzone);
  config_.yaw_max_acceleration = std::max(0.0,
      config_.yaw_max_acceleration);
  config_.yaw_ema_alpha = clamp(config_.yaw_ema_alpha, 0.0, 1.0);
  config_.yaw_min_speed = std::max(0.0, config_.yaw_min_speed);
  config_.yaw_max_speed = std::max(config_.yaw_min_speed + 1e-3,
      config_.yaw_max_speed);
  config_.yaw_low_speed_factor = clamp(config_.yaw_low_speed_factor, 0.0, 1.0);
  config_.gimbal_input_timeout_sec = std::max(0.02,
      config_.gimbal_input_timeout_sec);
  if (config_.gimbal_mount_type != "HORIZONTAL" &&
      config_.gimbal_mount_type != "VERTICAL" &&
      config_.gimbal_mount_type != "TILTED_45") {
    config_.gimbal_mount_type = "HORIZONTAL";
  }
  config_.gimbal_angle_filter_previous_weight = clamp(
      config_.gimbal_angle_filter_previous_weight, 0.0, 1.0);
  config_.gimbal_angle_deadzone = std::max(0.0,
      config_.gimbal_angle_deadzone);
  if (config_.gm_chase_forward_mode != "CONSTANT" &&
      config_.gm_chase_forward_mode != "PITCH_BASED") {
    config_.gm_chase_forward_mode = "CONSTANT";
  }
  config_.gm_chase_base_forward_speed = std::max(0.0,
      config_.gm_chase_base_forward_speed);
  config_.gm_chase_forward_acceleration = std::max(0.0,
      config_.gm_chase_forward_acceleration);
  config_.gm_chase_pitch_velocity_scaling = std::max(0.0,
      config_.gm_chase_pitch_velocity_scaling);
  config_.gm_chase_pitch_deadzone = std::max(0.0,
      config_.gm_chase_pitch_deadzone);
  config_.gm_chase_maximum_lateral_angle = std::max(1e-3,
      config_.gm_chase_maximum_lateral_angle);
  config_.gm_chase_maximum_vertical_angle = std::max(1e-3,
      config_.gm_chase_maximum_vertical_angle);
  config_.gm_vector_velocity = std::max(0.0,
      config_.gm_vector_velocity);
  config_.gm_vector_ramp_acceleration = std::max(0.0,
      config_.gm_vector_ramp_acceleration);
  config_.gm_vector_yaw_rate_gain = std::max(0.0,
      config_.gm_vector_yaw_rate_gain);
  config_.ground_altitude_factor = std::max(0.0,
      config_.ground_altitude_factor);
  config_.ground_maximum_descent_velocity = std::max(0.0,
      config_.ground_maximum_descent_velocity);
  config_.fw_commanded_airspeed = std::max(0.1,
      config_.fw_commanded_airspeed);
  config_.fw_maximum_course_offset = clamp(
      config_.fw_maximum_course_offset, 0.01, 1.40);
  config_.fw_maximum_climb_rate = std::max(0.0,
      config_.fw_maximum_climb_rate);
  config_.uncertainty_nominal_sigma = std::max(
      1e-6, config_.uncertainty_nominal_sigma);
  config_.uncertainty_slow_ratio = std::max(
      0.0, config_.uncertainty_slow_ratio);
  config_.uncertainty_abort_ratio = std::max(
      config_.uncertainty_slow_ratio + 1e-3,
      config_.uncertainty_abort_ratio);
  config_.uncertainty_minimum_scale = clamp(
      config_.uncertainty_minimum_scale, 0.0, 1.0);
  config_.uncertainty_yaw_scale = clamp(
      config_.uncertainty_yaw_scale, 0.0, 1.0);
  config_.reidentification_initial_scale = clamp(
      config_.reidentification_initial_scale, 0.0, 1.0);
  config_.reidentification_recovery_sec = std::max(
      0.0, config_.reidentification_recovery_sec);
  config_.relative_position_gain = std::max(
      0.0, config_.relative_position_gain);
  config_.relative_velocity_feedforward = std::max(
      0.0, config_.relative_velocity_feedforward);
  config_.relative_maximum_correction = std::max(
      0.0, config_.relative_maximum_correction);
  config_.chase_forward_velocity = std::min(
      config_.chase_forward_velocity, config_.maximum_forward_velocity);
  config_.initial_forward_velocity = std::min(
      config_.initial_forward_velocity, config_.maximum_forward_velocity);
  config_.target_loss_forward_velocity = std::min(
      config_.target_loss_forward_velocity, config_.maximum_forward_velocity);

  lateral_pid_.configure(config_.lateral_pid,
                         config_.maximum_lateral_velocity);
  vertical_pid_.configure(config_.vertical_pid,
                          config_.maximum_vertical_velocity);
  yaw_pid_.configure(config_.yaw_pid, config_.maximum_yaw_rate);
  ground_forward_pid_.configure(config_.ground_forward_pid,
                                config_.maximum_forward_velocity);
  fw_course_pid_.configure(config_.fw_course_pid,
                           config_.fw_maximum_course_offset);
  fw_climb_rate_pid_.configure(config_.fw_climb_rate_pid,
                               config_.fw_maximum_climb_rate);
  target_guidance_.reset(new TargetGuidance(config_.target_guidance));
  forward_velocity_ = config_.initial_forward_velocity;
  gm_chase_forward_velocity_ = 0.0;
  gm_vector_velocity_ = 0.0;
}

void TrackController::setVehicleState(const VehicleState& state) {
  if (!state.valid || !std::isfinite(state.roll) ||
      !std::isfinite(state.pitch) || !std::isfinite(state.altitude)) {
    vehicle_state_.valid = false;
    return;
  }
  VehicleState sanitized = state;
  if (sanitized.pose_valid &&
      (!std::isfinite(sanitized.x) || !std::isfinite(sanitized.y) ||
       !std::isfinite(sanitized.z) || !std::isfinite(sanitized.yaw) ||
       !std::isfinite(sanitized.receive_time) ||
       !std::isfinite(sanitized.observation_time))) {
    sanitized.pose_valid = false;
  }
  vehicle_state_ = sanitized;
  if (!sanitized.pose_valid) return;
  const double stamp = sanitized.observation_time > 0.0
      ? sanitized.observation_time : sanitized.receive_time;
  if (!std::isfinite(stamp) || stamp <= 0.0) return;
  if (!vehicle_state_history_.empty() &&
      stamp < (vehicle_state_history_.back().observation_time > 0.0
          ? vehicle_state_history_.back().observation_time
          : vehicle_state_history_.back().receive_time)) {
    // A restarted odometry clock must not corrupt a usable capture-time pose
    // history. The newest state remains available for legacy metric input.
    return;
  }
  vehicle_state_history_.push_back(sanitized);
  const double keep_sec = std::max(2.0,
      config_.target_guidance.world_filter_pose_history_sec);
  while (vehicle_state_history_.size() > 2) {
    const VehicleState& oldest = vehicle_state_history_.front();
    const double oldest_stamp = oldest.observation_time > 0.0
        ? oldest.observation_time : oldest.receive_time;
    if (stamp - oldest_stamp <= keep_sec) break;
    vehicle_state_history_.pop_front();
  }
}

void TrackController::setGimbalState(const GimbalStateData& state) {
  if (!state.valid || !std::isfinite(state.receive_time) ||
      !std::isfinite(state.yaw) || !std::isfinite(state.pitch) ||
      !std::isfinite(state.roll)) {
    gimbal_state_.valid = false;
    return;
  }
  gimbal_state_ = state;
}

bool TrackController::vehicleStateAtObservationTime(
    const double stamp, VehicleState* state) const {
  if (state == nullptr || !vehicle_state_.valid || !vehicle_state_.pose_valid) {
    return false;
  }
  if (!std::isfinite(stamp) || stamp <= 0.0 || vehicle_state_history_.empty()) {
    *state = vehicle_state_;
    return true;
  }
  const double maximum_gap = std::max(0.02,
      config_.target_guidance.world_filter_pose_history_sec);
  auto state_stamp = [](const VehicleState& value) {
    return value.observation_time > 0.0 ? value.observation_time
                                        : value.receive_time;
  };
  auto upper = std::lower_bound(
      vehicle_state_history_.begin(), vehicle_state_history_.end(), stamp,
      [&](const VehicleState& value, const double value_stamp) {
        return state_stamp(value) < value_stamp;
      });
  if (upper == vehicle_state_history_.begin()) {
    if (std::abs(state_stamp(*upper) - stamp) > maximum_gap) return false;
    *state = *upper;
    return true;
  }
  if (upper == vehicle_state_history_.end()) {
    const VehicleState& newest = vehicle_state_history_.back();
    if (std::abs(state_stamp(newest) - stamp) > maximum_gap) return false;
    *state = newest;
    return true;
  }
  const VehicleState& after = *upper;
  const VehicleState& before = *std::prev(upper);
  const double before_stamp = state_stamp(before);
  const double after_stamp = state_stamp(after);
  if (stamp - before_stamp > maximum_gap || after_stamp - stamp > maximum_gap ||
      after_stamp <= before_stamp) {
    return false;
  }
  const double ratio = clamp((stamp - before_stamp) /
      (after_stamp - before_stamp), 0.0, 1.0);
  *state = before;
  state->roll = before.roll + ratio * (after.roll - before.roll);
  state->pitch = before.pitch + ratio * (after.pitch - before.pitch);
  const double yaw_delta = std::atan2(std::sin(after.yaw - before.yaw),
                                      std::cos(after.yaw - before.yaw));
  state->yaw = before.yaw + ratio * yaw_delta;
  state->x = before.x + ratio * (after.x - before.x);
  state->y = before.y + ratio * (after.y - before.y);
  state->z = before.z + ratio * (after.z - before.z);
  state->altitude = before.altitude + ratio * (after.altitude - before.altitude);
  state->observation_time = stamp;
  state->pose_valid = true;
  state->valid = true;
  return true;
}

bool TrackController::metricObservationFromMeasurement(
    const TargetMeasurement& incoming, MetricObservation* observation,
    std::string* rejection_reason) const {
  auto reject = [&](const std::string& reason) {
    if (rejection_reason != nullptr) *rejection_reason = reason;
    return false;
  };
  if (observation == nullptr || !incoming.has_relative_position_body ||
      !incoming.range_valid) {
    return reject("metric position/range is unavailable");
  }
  const double timestamp = incoming.observation_time > 0.0 &&
          std::isfinite(incoming.observation_time)
      ? incoming.observation_time : incoming.receive_time;
  VehicleState capture_state;
  if (!vehicleStateAtObservationTime(timestamp, &capture_state)) {
    return reject("vehicle pose is unavailable at metric capture time");
  }
  observation->world_position = bodyFrdToWorldEnu(
      incoming.relative_position_body, capture_state);
  if (!std::all_of(observation->world_position.begin(),
                   observation->world_position.end(),
                   [](double value) { return std::isfinite(value); })) {
    return reject("metric world position is non-finite");
  }
  observation->sigma_m = incoming.position_sigma_m > 1e-3 &&
          std::isfinite(incoming.position_sigma_m)
      ? incoming.position_sigma_m : 1.0;
  observation->world_velocity = incoming.has_relative_velocity_body
      ? bodyFrdVectorToWorldEnu(incoming.relative_velocity_body, capture_state)
      : std::array<double, 3>{{0.0, 0.0, 0.0}};
  observation->velocity_valid = incoming.has_relative_velocity_body &&
      std::all_of(observation->world_velocity.begin(),
                  observation->world_velocity.end(),
                  [](double value) { return std::isfinite(value); });
  observation->stamp = timestamp;
  observation->class_id = incoming.class_id;
  observation->image_source = incoming.image_source;
  return true;
}

void TrackController::updateMetricStateFromWorldFilter() {
  const bool use_imm = config_.target_guidance.world_filter_model == "imm";
  last_metric_world_position_ = {{0.0, 0.0, 0.0}};
  last_metric_world_velocity_ = {{0.0, 0.0, 0.0}};
  double variance = 0.0;
  for (std::size_t model_index = 0;
       model_index < kWorldFilterModelCount; ++model_index) {
    const double weight = use_imm ? world_filter_probabilities_[model_index]
                                  : (model_index == 1 ? 1.0 : 0.0);
    for (std::size_t axis = 0; axis < 3; ++axis) {
      last_metric_world_position_[axis] +=
          weight * world_filter_models_[model_index].position[axis];
      last_metric_world_velocity_[axis] +=
          weight * world_filter_models_[model_index].velocity[axis];
      variance += weight * world_filter_models_[model_index].p_position[axis];
    }
  }
  last_metric_world_sigma_m_ = std::sqrt(std::max(1e-9, variance / 3.0));
  last_metric_world_velocity_valid_ = true;
  have_metric_world_state_ = true;
}

bool TrackController::applyMetricObservation(
    const MetricObservation& observation, const bool enforce_gate,
    std::string* rejection_reason) {
  auto reject = [&](const std::string& reason) {
    if (rejection_reason != nullptr) *rejection_reason = reason;
    return false;
  };
  const TargetGuidanceConfig& guidance = config_.target_guidance;
  const bool filter_enabled = guidance.world_filter_enabled ||
      guidance.multi_source_fusion_enabled || guidance.world_filter_oosm_enabled ||
      guidance.world_filter_model == "imm";

  // Keep the legacy direct metric path available for low-compute profiles.
  if (!filter_enabled) {
    if (have_metric_world_state_ && observation.stamp <= metric_world_time_) {
      return reject("metric timestamp is out of order");
    }
    if (have_metric_world_state_ && guidance.world_filter_max_innovation_m > 0.0) {
      const double innovation = std::sqrt(
          (observation.world_position[0] - last_metric_world_position_[0]) *
              (observation.world_position[0] - last_metric_world_position_[0]) +
          (observation.world_position[1] - last_metric_world_position_[1]) *
              (observation.world_position[1] - last_metric_world_position_[1]) +
          (observation.world_position[2] - last_metric_world_position_[2]) *
              (observation.world_position[2] - last_metric_world_position_[2]));
      if (!std::isfinite(innovation) ||
          innovation > guidance.world_filter_max_innovation_m) {
        return reject("metric innovation exceeds configured gate");
      }
    }
    const double dt = have_metric_world_state_
        ? observation.stamp - metric_world_time_ : 0.0;
    if (observation.velocity_valid) {
      last_metric_world_velocity_ = observation.world_velocity;
      last_metric_world_velocity_valid_ = true;
    } else if (have_metric_world_state_ && dt > 1e-3 && dt <= 2.0) {
      for (std::size_t i = 0; i < 3; ++i) {
        last_metric_world_velocity_[i] =
            (observation.world_position[i] - last_metric_world_position_[i]) / dt;
      }
      last_metric_world_velocity_valid_ = true;
    }
    last_metric_world_position_ = observation.world_position;
    metric_world_time_ = observation.stamp;
    last_metric_world_sigma_m_ = observation.sigma_m;
    have_metric_world_state_ = true;
    return true;
  }

  const bool use_imm = guidance.world_filter_model == "imm";
  const double model_accel[kWorldFilterModelCount] = {
      std::max(0.0, guidance.world_filter_imm_static_accel_stddev_mps2),
      std::max(0.0, guidance.world_filter_process_accel_stddev_mps2),
      std::max(guidance.world_filter_imm_static_accel_stddev_mps2,
               guidance.world_filter_imm_maneuver_accel_stddev_mps2)};
  const double minimum_probability = clamp(
      guidance.world_filter_imm_min_probability, 0.0, 0.30);

  auto initialize_model = [&](WorldFilterModel* model,
                              const WorldMotionModel motion_model) {
    model->initialized = true;
    model->motion_model = motion_model;
    model->stamp = observation.stamp;
    model->position = observation.world_position;
    model->velocity = observation.velocity_valid ? observation.world_velocity
                                     : std::array<double, 3>{{0.0, 0.0, 0.0}};
    if (motion_model == WorldMotionModel::kStationary) {
      model->velocity = {{0.0, 0.0, 0.0}};
    }
    for (std::size_t i = 0; i < 3; ++i) {
      model->p_position[i] = observation.sigma_m * observation.sigma_m;
      model->p_position_velocity[i] = 0.0;
      model->p_velocity[i] = observation.velocity_valid ? 4.0 : 100.0;
    }
    model->turn_rate_radps = 0.0;
    model->p_turn_rate = 0.25;
  };
  auto predict_model = [&](WorldFilterModel* model, double dt,
                           double accel_sigma) {
    if (model == nullptr || !model->initialized) return;
    dt = std::max(0.0, std::min(dt, 2.0));
    const double accel_var = accel_sigma * accel_sigma;
    const double dt2 = dt * dt;
    const double dt3 = dt2 * dt;
    const double dt4 = dt2 * dt2;
    if (model->motion_model == WorldMotionModel::kCoordinatedTurn &&
        std::abs(model->turn_rate_radps) > 1e-4) {
      const double turn = model->turn_rate_radps;
      const double angle = turn * dt;
      const double sine = std::sin(angle);
      const double cosine = std::cos(angle);
      const double vx = model->velocity[0];
      const double vy = model->velocity[1];
      model->position[0] += (sine * vx - (1.0 - cosine) * vy) / turn;
      model->position[1] += ((1.0 - cosine) * vx + sine * vy) / turn;
      model->velocity[0] = cosine * vx - sine * vy;
      model->velocity[1] = sine * vx + cosine * vy;
    } else if (model->motion_model != WorldMotionModel::kStationary) {
      model->position[0] += dt * model->velocity[0];
      model->position[1] += dt * model->velocity[1];
    }
    if (model->motion_model != WorldMotionModel::kStationary) {
      model->position[2] += dt * model->velocity[2];
    } else {
      const double decay = std::exp(-4.0 * dt);
      for (std::size_t i = 0; i < 3; ++i) model->velocity[i] *= decay;
    }
    for (std::size_t i = 0; i < 3; ++i) {
      const double p00 = model->p_position[i];
      const double p01 = model->p_position_velocity[i];
      const double p11 = model->p_velocity[i];
      const double motion_dt = model->motion_model == WorldMotionModel::kStationary
          ? 0.0 : dt;
      model->p_position[i] = p00 + 2.0 * motion_dt * p01 + motion_dt * motion_dt * p11 +
          0.25 * dt4 * accel_var;
      model->p_position_velocity[i] = p01 + motion_dt * p11 +
          0.5 * dt3 * accel_var;
      model->p_velocity[i] = p11 + dt2 * accel_var;
    }
    if (model->motion_model == WorldMotionModel::kCoordinatedTurn) {
      model->p_turn_rate += dt * guidance.world_filter_imm_turn_rate_stddev_radps2 *
          guidance.world_filter_imm_turn_rate_stddev_radps2;
    }
    model->stamp += dt;
  };
  auto update_model = [&](WorldFilterModel* model, const double prediction_dt) {
    double log_likelihood = 0.0;
    if (model == nullptr || !model->initialized) return log_likelihood;
    for (std::size_t i = 0; i < 3; ++i) {
      const double prior_p01 = model->p_position_velocity[i];
      const double innovation = observation.world_position[i] - model->position[i];
      const double innovation_variance = std::max(
          1e-9, model->p_position[i] + observation.sigma_m * observation.sigma_m);
      const double kp = model->p_position[i] / innovation_variance;
      const double kv = prior_p01 / innovation_variance;
      model->position[i] += kp * innovation;
      model->velocity[i] += kv * innovation;
      model->p_position[i] = std::max(1e-9,
          (1.0 - kp) * model->p_position[i]);
      model->p_position_velocity[i] = (1.0 - kp) * prior_p01;
      model->p_velocity[i] = std::max(1e-9,
          model->p_velocity[i] - kv * prior_p01);
      log_likelihood += -0.5 *
          (std::log(2.0 * 3.14159265358979323846 * innovation_variance) +
           innovation * innovation / innovation_variance);
    }
    if (observation.velocity_valid &&
        model->motion_model != WorldMotionModel::kStationary) {
      constexpr double kVelocityVariance = 4.0;
      const double previous_heading = std::atan2(model->velocity[1],
                                                 model->velocity[0]);
      for (std::size_t i = 0; i < 3; ++i) {
        const double variance = std::max(1e-9, model->p_velocity[i] +
                                         kVelocityVariance);
        const double gain = model->p_velocity[i] / variance;
        model->velocity[i] += gain *
            (observation.world_velocity[i] - model->velocity[i]);
        model->p_velocity[i] = std::max(1e-9,
            (1.0 - gain) * model->p_velocity[i]);
      }
      if (model->motion_model == WorldMotionModel::kCoordinatedTurn &&
          model->stamp > 0.0) {
        const double heading = std::atan2(model->velocity[1], model->velocity[0]);
        const double speed = std::hypot(model->velocity[0], model->velocity[1]);
        if (speed > 0.3) {
          // Heading difference is an angle.  Convert it to rad/s before it
          // is used by the coordinated-turn predictor; treating the angle as
          // a rate made the model depend on camera FPS.
          const double inferred = std::atan2(std::sin(heading - previous_heading),
                                             std::cos(heading - previous_heading)) /
              std::max(1e-3, prediction_dt);
          model->turn_rate_radps = clamp(0.75 * model->turn_rate_radps +
              0.25 * inferred, -guidance.world_filter_imm_max_turn_rate_radps,
              guidance.world_filter_imm_max_turn_rate_radps);
        }
      }
    }
    return log_likelihood;
  };
  auto fused_position = [&]() {
    std::array<double, 3> result{{0.0, 0.0, 0.0}};
    for (std::size_t model_index = 0;
         model_index < kWorldFilterModelCount; ++model_index) {
      const double weight = use_imm ? world_filter_probabilities_[model_index]
                                    : (model_index == 1 ? 1.0 : 0.0);
      for (std::size_t i = 0; i < 3; ++i) {
        result[i] += weight * world_filter_models_[model_index].position[i];
      }
    }
    return result;
  };
  auto fused_velocity = [&]() {
    std::array<double, 3> result{{0.0, 0.0, 0.0}};
    for (std::size_t model_index = 0;
         model_index < kWorldFilterModelCount; ++model_index) {
      const double weight = use_imm ? world_filter_probabilities_[model_index]
                                    : (model_index == 1 ? 1.0 : 0.0);
      for (std::size_t i = 0; i < 3; ++i) {
        result[i] += weight * world_filter_models_[model_index].velocity[i];
      }
    }
    return result;
  };
  auto update_probabilities = [&](const std::array<double,
                                      kWorldFilterModelCount>& likelihoods) {
    if (!use_imm) {
      world_filter_probabilities_ = {{0.0, 1.0, 0.0}};
      return;
    }
    const double maximum = *std::max_element(likelihoods.begin(), likelihoods.end());
    double total = 0.0;
    for (std::size_t i = 0; i < kWorldFilterModelCount; ++i) {
      world_filter_probabilities_[i] = std::max(minimum_probability,
          world_filter_probabilities_[i] * std::exp(likelihoods[i] - maximum));
      total += world_filter_probabilities_[i];
    }
    for (double& probability : world_filter_probabilities_) probability /= total;
  };

  if (!have_metric_world_state_ || !world_filter_models_[0].initialized) {
    initialize_model(&world_filter_models_[0], WorldMotionModel::kStationary);
    initialize_model(&world_filter_models_[1], WorldMotionModel::kConstantVelocity);
    initialize_model(&world_filter_models_[2], WorldMotionModel::kCoordinatedTurn);
    world_filter_probabilities_ = use_imm
        ? std::array<double, kWorldFilterModelCount>{{1.0 / 3.0, 1.0 / 3.0,
                                                       1.0 / 3.0}}
        : std::array<double, kWorldFilterModelCount>{{0.0, 1.0, 0.0}};
    last_metric_world_position_ = observation.world_position;
    last_metric_world_velocity_ = observation.velocity_valid ? observation.world_velocity
                                                 : std::array<double, 3>{{0.0, 0.0, 0.0}};
    last_metric_world_velocity_valid_ = observation.velocity_valid;
    last_metric_world_sigma_m_ = observation.sigma_m;
    metric_world_time_ = observation.stamp;
    have_metric_world_state_ = true;
    return true;
  }
  if (observation.stamp < metric_world_time_ - 1e-9) {
    return reject("metric observations must be replayed in timestamp order");
  }
  const double dt = std::max(0.0, observation.stamp - metric_world_time_);
  if (use_imm && dt > 0.0) {
    const std::array<double, kWorldFilterModelCount> persistence{{
        guidance.world_filter_imm_static_persistence,
        guidance.world_filter_imm_cv_persistence,
        guidance.world_filter_imm_turn_persistence}};
    std::array<WorldFilterModel, kWorldFilterModelCount> mixed;
    std::array<double, kWorldFilterModelCount> predicted_probability{{0.0, 0.0, 0.0}};
    for (std::size_t destination = 0; destination < kWorldFilterModelCount;
         ++destination) {
      for (std::size_t source = 0; source < kWorldFilterModelCount; ++source) {
        const double transition = source == destination ? persistence[source]
            : (1.0 - persistence[source]) /
                static_cast<double>(kWorldFilterModelCount - 1);
        predicted_probability[destination] +=
            world_filter_probabilities_[source] * transition;
      }
      mixed[destination] = world_filter_models_[destination];
      for (std::size_t axis = 0; axis < 3; ++axis) {
        mixed[destination].position[axis] = 0.0;
        mixed[destination].velocity[axis] = 0.0;
      }
      mixed[destination].turn_rate_radps = 0.0;
      for (std::size_t source = 0; source < kWorldFilterModelCount; ++source) {
        const double transition = source == destination ? persistence[source]
            : (1.0 - persistence[source]) /
                static_cast<double>(kWorldFilterModelCount - 1);
        const double weight = world_filter_probabilities_[source] * transition /
            std::max(1e-12, predicted_probability[destination]);
        for (std::size_t axis = 0; axis < 3; ++axis) {
          mixed[destination].position[axis] +=
              weight * world_filter_models_[source].position[axis];
          mixed[destination].velocity[axis] +=
              weight * world_filter_models_[source].velocity[axis];
        }
        mixed[destination].turn_rate_radps +=
            weight * world_filter_models_[source].turn_rate_radps;
      }
      for (std::size_t axis = 0; axis < 3; ++axis) {
        mixed[destination].p_position[axis] = 0.0;
        mixed[destination].p_position_velocity[axis] = 0.0;
        mixed[destination].p_velocity[axis] = 0.0;
        for (std::size_t source = 0; source < kWorldFilterModelCount; ++source) {
          const double transition = source == destination ? persistence[source]
              : (1.0 - persistence[source]) /
                  static_cast<double>(kWorldFilterModelCount - 1);
          const double weight = world_filter_probabilities_[source] * transition /
              std::max(1e-12, predicted_probability[destination]);
          const double dp = world_filter_models_[source].position[axis] -
              mixed[destination].position[axis];
          const double dv = world_filter_models_[source].velocity[axis] -
              mixed[destination].velocity[axis];
          mixed[destination].p_position[axis] += weight *
              (world_filter_models_[source].p_position[axis] + dp * dp);
          mixed[destination].p_position_velocity[axis] += weight *
              (world_filter_models_[source].p_position_velocity[axis] + dp * dv);
          mixed[destination].p_velocity[axis] += weight *
              (world_filter_models_[source].p_velocity[axis] + dv * dv);
        }
      }
    }
    world_filter_models_ = mixed;
    world_filter_probabilities_ = predicted_probability;
  }
  for (std::size_t index = 0; index < kWorldFilterModelCount; ++index) {
    if (use_imm || index == 1) predict_model(&world_filter_models_[index], dt,
                                               model_accel[index]);
  }
  const std::array<double, 3> predicted = fused_position();
  double mahalanobis = 0.0;
  double euclidean_innovation = 0.0;
  for (std::size_t axis = 0; axis < 3; ++axis) {
    double predicted_variance = 0.0;
    for (std::size_t index = 0; index < kWorldFilterModelCount; ++index) {
      const double weight = use_imm ? world_filter_probabilities_[index]
                                    : (index == 1 ? 1.0 : 0.0);
      predicted_variance += weight * world_filter_models_[index].p_position[axis];
    }
    const double innovation = observation.world_position[axis] - predicted[axis];
    euclidean_innovation += innovation * innovation;
    mahalanobis += innovation * innovation / std::max(1e-6,
        predicted_variance + observation.sigma_m * observation.sigma_m);
  }
  if (enforce_gate && guidance.world_filter_max_innovation_m > 0.0 &&
      std::sqrt(euclidean_innovation) > guidance.world_filter_max_innovation_m) {
    return reject("metric innovation exceeds configured gate");
  }
  if (enforce_gate && guidance.world_filter_mahalanobis_gate > 0.0 &&
      mahalanobis > guidance.world_filter_mahalanobis_gate) {
    return reject("metric innovation exceeds Mahalanobis gate");
  }
  std::array<double, kWorldFilterModelCount> likelihoods{{0.0, 0.0, 0.0}};
  for (std::size_t index = 0; index < kWorldFilterModelCount; ++index) {
    if (use_imm || index == 1) {
      likelihoods[index] = update_model(&world_filter_models_[index], dt);
    }
  }
  update_probabilities(likelihoods);
  metric_world_time_ = observation.stamp;
  updateMetricStateFromWorldFilter();
  if (rejection_reason != nullptr) rejection_reason->clear();
  return true;
}

bool TrackController::integrateMetricMeasurement(
    const TargetMeasurement& incoming, std::array<double, 3>* filtered_world,
    std::string* rejection_reason) {
  auto reject = [&](const std::string& reason) {
    if (rejection_reason != nullptr) *rejection_reason = reason;
    return false;
  };
  if (filtered_world == nullptr) return reject("metric output is unavailable");
  MetricObservation observation;
  if (!metricObservationFromMeasurement(incoming, &observation, rejection_reason)) {
    return false;
  }
  const bool filter_enabled = config_.target_guidance.world_filter_enabled ||
      config_.target_guidance.multi_source_fusion_enabled ||
      config_.target_guidance.world_filter_oosm_enabled ||
      config_.target_guidance.world_filter_model == "imm";
  if (!filter_enabled) {
    if (!applyMetricObservation(observation, true, rejection_reason)) return false;
    *filtered_world = last_metric_world_position_;
    return true;
  }
  const WorldFilterState saved_state{have_metric_world_state_, metric_world_time_,
      world_filter_models_, world_filter_probabilities_};
  const auto saved_history = world_filter_history_;
  const WorldFilterState saved_baseline = world_filter_baseline_;
  const bool delayed = have_metric_world_state_ &&
      observation.stamp < metric_world_time_ - 1e-9;
  if (delayed && (!config_.target_guidance.world_filter_oosm_enabled ||
      metric_world_time_ - observation.stamp >
          config_.target_guidance.world_filter_oosm_window_sec ||
      (world_filter_baseline_.initialized &&
       observation.stamp <= world_filter_baseline_.stamp))) {
    return reject("metric timestamp is outside the OOSM replay window");
  }
  if (!delayed) {
    // Prediction, IMM mixing and innovation gating are implemented together
    // in applyMetricObservation().  Keep that operation transactional: an
    // outlier must not leave a predicted-but-uncommitted filter state behind
    // and then be predicted a second time by the next frame.
    if (!applyMetricObservation(observation, true, rejection_reason)) {
      have_metric_world_state_ = saved_state.initialized;
      metric_world_time_ = saved_state.stamp;
      world_filter_models_ = saved_state.models;
      world_filter_probabilities_ = saved_state.probabilities;
      world_filter_history_ = saved_history;
      world_filter_baseline_ = saved_baseline;
      return false;
    }
    WorldFilterHistory history;
    history.observation = observation;
    history.state_after = {have_metric_world_state_, metric_world_time_,
                           world_filter_models_, world_filter_probabilities_};
    world_filter_history_.push_back(history);
  } else {
    MetricObservation delayed_observation = observation;
    const double age = std::max(0.0, metric_world_time_ - observation.stamp);
    delayed_observation.sigma_m *= std::sqrt(std::max(1.0,
        config_.target_guidance.world_filter_oosm_covariance_inflation) *
        (1.0 + age));
    auto insertion = std::upper_bound(world_filter_history_.begin(),
                                      world_filter_history_.end(),
                                      delayed_observation.stamp,
        [](const double stamp, const WorldFilterHistory& history) {
          return stamp < history.observation.stamp;
        });
    WorldFilterHistory delayed_history;
    delayed_history.observation = delayed_observation;
    const std::size_t delayed_index = static_cast<std::size_t>(
        std::distance(world_filter_history_.begin(), insertion));
    world_filter_history_.insert(insertion, delayed_history);
    const WorldFilterState restore = delayed_index == 0
        ? world_filter_baseline_ : world_filter_history_[delayed_index - 1].state_after;
    have_metric_world_state_ = restore.initialized;
    metric_world_time_ = restore.stamp;
    world_filter_models_ = restore.models;
    world_filter_probabilities_ = restore.probabilities;
    bool replay_ok = true;
    std::string replay_reason;
    for (std::size_t index = delayed_index; index < world_filter_history_.size(); ++index) {
      const bool gate = index == delayed_index;
      if (!applyMetricObservation(world_filter_history_[index].observation, gate,
                                  &replay_reason)) {
        replay_ok = false;
        break;
      }
      world_filter_history_[index].state_after = {
          have_metric_world_state_, metric_world_time_, world_filter_models_,
          world_filter_probabilities_};
    }
    if (!replay_ok) {
      have_metric_world_state_ = saved_state.initialized;
      metric_world_time_ = saved_state.stamp;
      world_filter_models_ = saved_state.models;
      world_filter_probabilities_ = saved_state.probabilities;
      world_filter_history_ = saved_history;
      world_filter_baseline_ = saved_baseline;
      return reject(replay_reason.empty() ? "OOSM replay failed" : replay_reason);
    }
  }
  while (!world_filter_history_.empty() &&
         metric_world_time_ - world_filter_history_.front().observation.stamp >
             std::max(0.0, config_.target_guidance.world_filter_oosm_window_sec)) {
    world_filter_baseline_ = world_filter_history_.front().state_after;
    world_filter_history_.pop_front();
  }
  *filtered_world = last_metric_world_position_;
  if (rejection_reason != nullptr) rejection_reason->clear();
  return true;
}

bool TrackController::updateMetricMeasurement(
    const TargetMeasurement& incoming, std::string* rejection_reason) {
  if (!finiteMeasurement(incoming)) {
    if (rejection_reason != nullptr) *rejection_reason =
        "metric measurement contains non-finite values";
    return false;
  }
  if (incoming.receive_time < 0.0 || incoming.observation_time < 0.0) {
    if (rejection_reason != nullptr) *rejection_reason =
        "metric measurement time is invalid";
    return false;
  }
  std::array<double, 3> filtered_world{{0.0, 0.0, 0.0}};
  if (!integrateMetricMeasurement(incoming, &filtered_world,
                                  rejection_reason)) {
    return false;
  }
  // A standby source contributes to the same coast state as the active
  // source. Re-express the fused world estimate in the current body frame so
  // a subsequent image gap does not fall back to an older source position.
  if (vehicle_state_.valid && vehicle_state_.pose_valid) {
    last_metric_position_frd_ = worldEnuToBodyFrd(
        last_metric_world_position_, vehicle_state_);
    last_metric_velocity_frd_ = worldEnuVectorToBodyFrd(
        last_metric_world_velocity_, vehicle_state_);
    last_metric_velocity_valid_ = last_metric_world_velocity_valid_;
    last_metric_sigma_m_ = last_metric_world_sigma_m_;
  } else {
    last_metric_position_frd_ = incoming.relative_position_body;
    last_metric_velocity_frd_ = incoming.relative_velocity_body;
    last_metric_velocity_valid_ = incoming.has_relative_velocity_body;
    last_metric_sigma_m_ = incoming.position_sigma_m > 1e-3 &&
            std::isfinite(incoming.position_sigma_m)
        ? incoming.position_sigma_m : 1.0;
  }
  have_metric_state_ = true;
  metric_state_time_ = incoming.receive_time;
  return true;
}

bool TrackController::metricMeasurementCompatible(
    const TargetMeasurement& incoming, const double maximum_distance_m,
    std::string* rejection_reason) const {
  auto reject = [&](const std::string& reason) {
    if (rejection_reason != nullptr) *rejection_reason = reason;
    return false;
  };
  if (maximum_distance_m <= 0.0 || !have_metric_world_state_) return true;
  MetricObservation observation;
  if (!metricObservationFromMeasurement(incoming, &observation,
                                        rejection_reason)) {
    return false;
  }
  std::array<double, 3> predicted = last_metric_world_position_;
  const double dt = observation.stamp - metric_world_time_;
  if (last_metric_world_velocity_valid_ && std::isfinite(dt) &&
      std::abs(dt) <= std::max(2.0,
          config_.target_guidance.multi_source_max_age_sec)) {
    for (std::size_t axis = 0; axis < 3; ++axis) {
      predicted[axis] += last_metric_world_velocity_[axis] * dt;
    }
  }
  double squared = 0.0;
  for (std::size_t axis = 0; axis < 3; ++axis) {
    const double error = observation.world_position[axis] - predicted[axis];
    squared += error * error;
  }
  const double gate = maximum_distance_m + std::max(0.0, observation.sigma_m) +
      std::max(0.0, last_metric_world_sigma_m_);
  if (!std::isfinite(squared) || std::sqrt(squared) > gate) {
    return reject("cross-source metric position is outside the identity gate");
  }
  return true;
}

bool TrackController::updateMeasurement(
    const TargetMeasurement& incoming, std::string* rejection_reason) {
  auto reject = [&](const std::string& reason) {
    if (rejection_reason != nullptr) *rejection_reason = reason;
    return false;
  };
  if (!finiteMeasurement(incoming)) {
    return reject("target box contains non-finite values");
  }
  if (incoming.image_width == 0 || incoming.image_height == 0) {
    return reject("image dimensions must be non-zero");
  }
  if (incoming.receive_time < 0.0) {
    return reject("target box receive time is invalid");
  }
  if (incoming.observation_time < 0.0) {
    return reject("target observation time is invalid");
  }

  const double confidence_threshold = confidence_locked_
      ? config_.minimum_confidence - config_.confidence_hysteresis
      : config_.minimum_confidence;
  if (incoming.confidence < confidence_threshold) {
    return reject("target confidence is below the tracking threshold");
  }

  const double width = static_cast<double>(incoming.image_width);
  const double height = static_cast<double>(incoming.image_height);
  const double x_min = clamp(incoming.x_min / width, 0.0, 1.0);
  const double y_min = clamp(incoming.y_min / height, 0.0, 1.0);
  const double x_max = clamp(incoming.x_max / width, 0.0, 1.0);
  const double y_max = clamp(incoming.y_max / height, 0.0, 1.0);
  if (x_max <= x_min || y_max <= y_min) {
    return reject("target box has zero or negative area");
  }

  const double center_x = x_min + x_max - 1.0;
  const double center_y = y_min + y_max - 1.0;
  const double size_ratio = std::sqrt((x_max - x_min) * (y_max - y_min));
  const bool target_changed = filter_initialized_ && incoming.track_id >= 0 &&
      filtered_track_id_ >= 0 && incoming.track_id != filtered_track_id_;
  const double measurement_dt = incoming.receive_time - previous_measurement_time_;
  if (!filter_initialized_ || target_changed || measurement_dt <= 0.0 ||
      measurement_dt > config_.target_loss_reset_sec) {
    filtered_center_x_ = center_x;
    filtered_center_y_ = center_y;
    filtered_velocity_x_ = 0.0;
    filtered_velocity_y_ = 0.0;
    filtered_size_ratio_ = size_ratio;
    filter_initialized_ = true;
    resetFollowerState();
  } else {
    const double old_x = filtered_center_x_;
    const double old_y = filtered_center_y_;
    const double alpha = config_.position_filter_alpha;
    filtered_center_x_ += alpha * (center_x - filtered_center_x_);
    filtered_center_y_ += alpha * (center_y - filtered_center_y_);
    filtered_size_ratio_ += alpha * (size_ratio - filtered_size_ratio_);
    if (measurement_dt > 1e-4) {
      const double raw_vx = (filtered_center_x_ - old_x) / measurement_dt;
      const double raw_vy = (filtered_center_y_ - old_y) / measurement_dt;
      const double velocity_alpha = config_.velocity_filter_alpha;
      filtered_velocity_x_ += velocity_alpha *
          (raw_vx - filtered_velocity_x_);
      filtered_velocity_y_ += velocity_alpha *
          (raw_vy - filtered_velocity_y_);
    }
  }

  measurement_ = incoming;
  measurement_.tracking_quality = clamp(incoming.tracking_quality, 0.0, 1.0);
  last_metric_measurement_accepted_ = false;
  bool metric_accepted = incoming.has_relative_position_body &&
      incoming.range_valid;
  std::array<double, 3> filtered_world_position{{0.0, 0.0, 0.0}};
  bool filtered_world_valid = false;
  if (metric_accepted && vehicle_state_.valid && vehicle_state_.pose_valid) {
    std::string metric_reason;
    metric_accepted = integrateMetricMeasurement(
        incoming, &filtered_world_position, &metric_reason);
    filtered_world_valid = metric_accepted;
  }
  if (metric_accepted && incoming.has_relative_position_body && incoming.range_valid) {
    last_metric_measurement_accepted_ = true;
    have_metric_state_ = true;
    metric_state_time_ = incoming.receive_time;
    last_metric_position_frd_ = incoming.relative_position_body;
    last_metric_velocity_valid_ = incoming.has_relative_velocity_body;
    if (last_metric_velocity_valid_) {
      last_metric_velocity_frd_ = incoming.relative_velocity_body;
    } else {
      last_metric_velocity_frd_ = {{0.0, 0.0, 0.0}};
    }
    last_metric_sigma_m_ = incoming.position_sigma_m > 1e-3 &&
            std::isfinite(incoming.position_sigma_m)
        ? incoming.position_sigma_m : 1.0;
    if (filtered_world_valid) {
      const std::array<double, 3>& world = filtered_world_position;
      if (last_metric_world_velocity_valid_ &&
          config_.target_guidance.world_filter_max_speed_mps > 0.0) {
        const double speed = std::sqrt(
            last_metric_world_velocity_[0] * last_metric_world_velocity_[0] +
            last_metric_world_velocity_[1] * last_metric_world_velocity_[1] +
            last_metric_world_velocity_[2] * last_metric_world_velocity_[2]);
        if (speed > config_.target_guidance.world_filter_max_speed_mps) {
          const double scale = config_.target_guidance.world_filter_max_speed_mps /
              std::max(1e-6, speed);
          for (double& value : last_metric_world_velocity_) value *= scale;
        }
      }
      if (config_.target_guidance.world_filter_enabled) {
        measurement_.relative_position_body = worldEnuToBodyFrd(
            last_metric_world_position_, vehicle_state_);
        last_metric_position_frd_ = measurement_.relative_position_body;
        last_metric_velocity_frd_ = worldEnuVectorToBodyFrd(
            last_metric_world_velocity_, vehicle_state_);
        last_metric_velocity_valid_ = last_metric_world_velocity_valid_;
        last_metric_sigma_m_ = last_metric_world_sigma_m_;
      }
    }
  } else if (incoming.has_relative_position_body && incoming.range_valid) {
    // Keep the image track, but never expose a rejected metric innovation to
    // guidance.  The previous world state remains available for bounded coast.
    measurement_.has_relative_position_body = false;
    measurement_.has_relative_velocity_body = false;
    measurement_.range_valid = false;
  }
  if (incoming.reidentification_match) {
    reidentification_time_ = incoming.receive_time;
  }
  previous_measurement_time_ = incoming.receive_time;
  filtered_track_id_ = incoming.track_id;
  have_measurement_ = true;
  ever_acquired_ = true;
  confidence_locked_ = true;
  loss_active_ = false;
  loss_controllers_reset_ = false;
  cleared_reason_.clear();
  if (rejection_reason != nullptr) rejection_reason->clear();
  return true;
}

bool TrackController::lastMetricMeasurementAccepted() const {
  return last_metric_measurement_accepted_;
}

TrackVelocity TrackController::baseOutput(const double now) const {
  TrackVelocity output;
  output.profile = followerProfileName(config_.profile);
  LateralGuidanceMode active_guidance = config_.lateral_guidance;
  if (config_.profile == FollowerProfile::kGimbalVelocityChase) {
    active_guidance = config_.gm_chase_lateral_guidance;
  } else if (config_.profile == FollowerProfile::kGimbalVelocityVector) {
    active_guidance = config_.gm_vector_lateral_guidance;
  } else if (config_.profile == FollowerProfile::kFixedWingVelocityVector) {
    output.lateral_guidance_mode = "velocity_vector";
  }
  if (output.lateral_guidance_mode.empty()) {
    output.lateral_guidance_mode = lateralGuidanceModeName(active_guidance);
  }
  output.use_yaw_rate =
      config_.profile != FollowerProfile::kFixedWingVelocityVector;
  output.release_reference_on_invalid =
      config_.profile == FollowerProfile::kFixedWingVelocityVector;
  output.track_id = filtered_track_id_;
  output.center_x = filtered_center_x_;
  output.center_y = filtered_center_y_;
  output.center_velocity_x = filtered_velocity_x_;
  output.center_velocity_y = filtered_velocity_y_;
  output.target_size_ratio = filtered_size_ratio_;
  output.target_predicted = measurement_.predicted;
  output.tracking_quality = measurement_.tracking_quality;
  output.association_method = measurement_.association_method;
  if (have_measurement_) {
    output.input_age_sec = std::max(0.0, now - measurement_.receive_time);
  }
  return output;
}

bool TrackController::filteredGimbalAngles(const double now, double* yaw,
                                           double* pitch, double* roll) {
  if (yaw == nullptr || pitch == nullptr || roll == nullptr ||
      !gimbal_state_.valid || now - gimbal_state_.receive_time < -1e-6 ||
      now - gimbal_state_.receive_time > config_.gimbal_input_timeout_sec) {
    return false;
  }
  if (gimbal_state_.status_valid) {
    if (gimbal_state_.status_receive_time > 0.0 &&
        (now - gimbal_state_.status_receive_time < -1e-6 ||
         now - gimbal_state_.status_receive_time >
             config_.gimbal_input_timeout_sec)) {
      return false;
    }
    if (!gimbal_state_.healthy || !gimbal_state_.tracking_active ||
        !gimbal_state_.control_authority_available) {
      return false;
    }
  }
  if (!gimbal_filter_initialized_) {
    filtered_gimbal_yaw_ = gimbal_state_.yaw;
    filtered_gimbal_pitch_ = gimbal_state_.pitch;
    filtered_gimbal_roll_ = gimbal_state_.roll;
    gimbal_filter_initialized_ = true;
  } else {
    const double previous = config_.gimbal_angle_filter_previous_weight;
    const double current = 1.0 - previous;
    filtered_gimbal_yaw_ = previous * filtered_gimbal_yaw_ +
                           current * gimbal_state_.yaw;
    filtered_gimbal_pitch_ = previous * filtered_gimbal_pitch_ +
                             current * gimbal_state_.pitch;
    filtered_gimbal_roll_ = previous * filtered_gimbal_roll_ +
                            current * gimbal_state_.roll;
  }
  *yaw = (config_.gimbal_invert_yaw ? -filtered_gimbal_yaw_
                                    : filtered_gimbal_yaw_) +
         config_.gimbal_yaw_offset;
  *pitch = (config_.gimbal_invert_pitch ? -filtered_gimbal_pitch_
                                        : filtered_gimbal_pitch_) +
           config_.gimbal_pitch_offset;
  *roll = (config_.gimbal_invert_roll ? -filtered_gimbal_roll_
                                      : filtered_gimbal_roll_) +
          config_.gimbal_roll_offset;
  const double neutral_pitch = config_.gimbal_mount_type == "VERTICAL"
      ? 1.5707963267948966
      : (config_.gimbal_mount_type == "TILTED_45"
             ? 0.7853981633974483 : 0.0);
  if (std::abs(*yaw) < config_.gimbal_angle_deadzone) *yaw = 0.0;
  if (std::abs(*roll) < config_.gimbal_angle_deadzone) *roll = 0.0;
  if (std::abs(*pitch - neutral_pitch) < config_.gimbal_angle_deadzone) {
    *pitch = neutral_pitch;
  }
  return true;
}

void TrackController::gimbalToBodyVector(
    const double yaw, const double pitch, const double roll,
    double* forward, double* right, double* down) const {
  if (config_.gimbal_mount_type == "VERTICAL") {
    const double adjusted_pitch = pitch - 1.5707963267948966;
    *forward = std::cos(yaw) * std::cos(adjusted_pitch);
    *right = -std::sin(roll);
    *down = std::sin(adjusted_pitch);
  } else {
    const double adjusted_pitch = config_.gimbal_mount_type == "TILTED_45"
        ? pitch - 0.7853981633974483 : pitch;
    *forward = std::cos(adjusted_pitch) * std::cos(yaw);
    *right = std::sin(yaw) * std::cos(adjusted_pitch);
    *down = std::sin(adjusted_pitch);
  }
  const double norm = std::sqrt(*forward * *forward + *right * *right +
                                *down * *down);
  if (norm > 1e-9) {
    *forward /= norm;
    *right /= norm;
    *down /= norm;
  }
}

double TrackController::smoothYaw(const double raw_yaw, const double dt,
                                  const double forward_speed) {
  if (!config_.yaw_smoothing_enabled) return raw_yaw;
  double target = applyDeadband(raw_yaw, config_.yaw_deadzone);
  if (config_.yaw_speed_scaling_enabled) {
    double factor = 1.0;
    const double speed = std::abs(forward_speed);
    if (speed <= config_.yaw_min_speed) {
      factor = config_.yaw_low_speed_factor;
    } else if (speed < config_.yaw_max_speed) {
      const double ratio = (speed - config_.yaw_min_speed) /
          (config_.yaw_max_speed - config_.yaw_min_speed);
      factor = config_.yaw_low_speed_factor +
          ratio * (1.0 - config_.yaw_low_speed_factor);
    }
    target *= factor;
  }
  yaw_rate_limited_ = moveToward(
      yaw_rate_limited_, target, config_.yaw_max_acceleration * dt);
  smoothed_yaw_ += config_.yaw_ema_alpha *
      (yaw_rate_limited_ - smoothed_yaw_);
  return smoothed_yaw_;
}

TrackVelocity TrackController::compute(const double now) {
  TrackVelocity output = baseOutput(std::isfinite(now) ? now : 0.0);
  if (!std::isfinite(now) || now < 0.0) {
    output.tracking_state = "invalid_time";
    output.invalid_reason = "control time is invalid";
    return output;
  }

  double dt = previous_compute_time_ > 0.0 ? now - previous_compute_time_ : 0.0;
  if (!std::isfinite(dt) || dt < 0.0 || dt > 0.5) dt = 0.0;
  previous_compute_time_ = now;

  bool fresh = have_measurement_;
  double measurement_age = 0.0;
  if (fresh) {
    measurement_age = now - measurement_.receive_time;
    if (measurement_age < -1e-6) {
      fresh = false;
      cleared_reason_ = "target box time is in the future";
    } else if (measurement_age > config_.input_timeout_sec) {
      fresh = false;
      cleared_reason_ = "target box is stale";
    }
  }

  // Optional metric guidance is evaluated before the legacy loss path.  This
  // lets a fixed-wing keep the captured inertial orbit during a bounded
  // detector gap while preserving the existing TrackVelocity interface.
  const bool guidance_enabled = target_guidance_ != nullptr &&
      config_.target_guidance.enabled &&
      config_.target_guidance.mode != TargetGuidanceMode::kLegacyVisual;
  if (guidance_enabled && (have_measurement_ || have_metric_state_)) {
    const bool metric_mode_requested = config_.target_guidance.mode ==
            TargetGuidanceMode::kMetricPursuit ||
        config_.target_guidance.mode == TargetGuidanceMode::kMetricOrbit;
    const bool metric_observation = fresh &&
        measurement_.has_relative_position_body && measurement_.range_valid;
    const double metric_elapsed = have_metric_state_
        ? std::max(0.0, now - metric_state_time_) : 0.0;
    const double coast_limit = config_.input_timeout_sec +
        config_.target_guidance.target_loss_coast_sec +
        config_.target_guidance.target_loss_orbit_sec +
        (config_.target_guidance.target_loss_center_hold_enabled
             ? config_.target_guidance.target_loss_center_hold_sec : 0.0);
    const bool metric_coast = !metric_observation && have_metric_state_ &&
        metric_elapsed <= coast_limit && metric_elapsed >= -1e-6;
    if (metric_observation || metric_coast || (fresh && metric_mode_requested)) {
      TargetGuidanceInput input;
      input.now = now;
      input.dt = dt;
      // ``now`` is intentionally wall-clock based: control watchdogs must
      // continue working when /clock stalls or is reset.  The capture stamp
      // is still used by the inertial filter/OOSM path, but it may be in a
      // different epoch (Gazebo simulation time versus wall time), so it
      // must never be used for this freshness comparison.
      input.observation_time = metric_coast ? now : measurement_.receive_time;
      input.image_error_x = filtered_center_x_ - config_.target_x;
      input.image_error_y = filtered_center_y_ - config_.target_y;
      input.metric_valid = metric_observation || metric_coast;
      input.range_valid = metric_observation || metric_coast;
      input.position_sigma_m = !(metric_observation || metric_coast) ? 0.0
          : metric_coast
          ? last_metric_sigma_m_ +
              ((metric_elapsed <= config_.target_guidance.target_loss_coast_sec)
                   ? config_.target_guidance.prediction_process_sigma_mps
                   : config_.target_guidance.target_loss_hold_process_sigma_mps) *
                  std::max(0.0, metric_elapsed)
          : (measurement_.position_sigma_m > 1e-3
                 ? measurement_.position_sigma_m : last_metric_sigma_m_);
      input.relative_position_body_frd = metric_coast
          ? last_metric_position_frd_
          : (metric_observation ? measurement_.relative_position_body
                                 : std::array<double, 3>{{0.0, 0.0, 0.0}});
      input.relative_velocity_valid = metric_coast
          ? last_metric_velocity_valid_
          : (metric_observation && measurement_.has_relative_velocity_body);
      input.relative_velocity_body_frd = metric_coast
          ? last_metric_velocity_frd_
          : (metric_observation ? measurement_.relative_velocity_body
                                 : std::array<double, 3>{{0.0, 0.0, 0.0}});
      if (metric_coast && input.relative_velocity_valid) {
        for (std::size_t i = 0; i < 3; ++i) {
          input.relative_position_body_frd[i] +=
              input.relative_velocity_body_frd[i] * metric_elapsed;
        }
      }
      const bool pose_fresh = vehicle_state_.valid &&
          vehicle_state_.pose_valid &&
          (vehicle_state_.receive_time <= 0.0 ||
           now - vehicle_state_.receive_time <=
               config_.target_guidance.inertial_coast_state_timeout_sec);
      if (metric_coast && have_metric_world_state_ && pose_fresh) {
        std::array<double, 3> world = last_metric_world_position_;
        if (last_metric_world_velocity_valid_) {
          for (std::size_t i = 0; i < 3; ++i) {
            world[i] += last_metric_world_velocity_[i] * metric_elapsed;
          }
        }
        input.relative_position_body_frd = worldEnuToBodyFrd(
            world, vehicle_state_);
        if (last_metric_world_velocity_valid_) {
          input.relative_velocity_body_frd = worldEnuVectorToBodyFrd(
              last_metric_world_velocity_, vehicle_state_);
          input.relative_velocity_valid = true;
        }
      }
      const std::string source = measurement_.image_source;
      input.source_is_gimbal = source.find("gimbal") != std::string::npos ||
          source.find("lrf") != std::string::npos ||
          source.find("pod") != std::string::npos;
      input.source_is_fixed_camera = !input.source_is_gimbal;
      input.vehicle_altitude_valid = vehicle_state_.valid;
      input.vehicle_altitude_m = vehicle_state_.altitude;
      input.vehicle_roll = vehicle_state_.roll;
      input.vehicle_pitch = vehicle_state_.pitch;
      const double guidance_speed = config_.target_guidance.commanded_speed > 0.1
          ? config_.target_guidance.commanded_speed
          : config_.fw_commanded_airspeed;
      input.commanded_speed = config_.profile ==
              FollowerProfile::kFixedWingVelocityVector
          ? guidance_speed : std::min(guidance_speed,
                                      config_.maximum_forward_velocity);
      double gimbal_yaw = 0.0;
      double gimbal_pitch = 0.0;
      double gimbal_roll = 0.0;
      if (input.source_is_gimbal && filteredGimbalAngles(
              now, &gimbal_yaw, &gimbal_pitch, &gimbal_roll)) {
        input.bearing_valid = true;
        input.bearing_yaw = gimbal_yaw;
        input.bearing_pitch = gimbal_pitch;
      }
      if (gimbal_state_.status_valid && input.source_is_gimbal) {
        input.gimbal_fov_valid = gimbal_state_.healthy &&
            gimbal_state_.tracking_active &&
            gimbal_state_.control_authority_available &&
            gimbal_state_.horizontal_fov_rad > 0.0 &&
            gimbal_state_.vertical_fov_rad > 0.0;
        input.gimbal_horizontal_fov_rad = gimbal_state_.horizontal_fov_rad;
        input.gimbal_vertical_fov_rad = gimbal_state_.vertical_fov_rad;
        input.gimbal_zoom_ratio = gimbal_state_.zoom_ratio;
      }
      TargetGuidanceOutput guidance = target_guidance_->update(input);
      if (gimbal_state_.status_valid && input.source_is_gimbal &&
          (!gimbal_state_.healthy || !gimbal_state_.tracking_active ||
           !gimbal_state_.control_authority_available)) {
        guidance.active = false;
        guidance.invalid_reason =
            "gimbal status reports unhealthy, unlocked or no control authority";
        guidance.state = "gimbal_unavailable";
      }
      if (guidance.active) {
        output.forward = guidance.forward;
        output.left = guidance.left;
        output.up = guidance.up;
        if (config_.profile != FollowerProfile::kFixedWingVelocityVector) {
          output.forward = clamp(output.forward,
                                 -config_.maximum_reverse_velocity,
                                 config_.maximum_forward_velocity);
          output.left = clamp(output.left, -config_.maximum_lateral_velocity,
                              config_.maximum_lateral_velocity);
          output.up = clamp(output.up, -config_.maximum_vertical_velocity,
                            config_.maximum_vertical_velocity);
        }
        output.yaw_rate = config_.profile ==
                FollowerProfile::kFixedWingVelocityVector
            ? (config_.target_guidance.publish_course_rate_feedforward
                   ? guidance.course_rate : 0.0)
            : guidance.course_rate;
        output.use_yaw_rate = config_.profile !=
            FollowerProfile::kFixedWingVelocityVector ||
            config_.target_guidance.publish_course_rate_feedforward;
        // The fixed-wing controller can combine a tangent velocity with a
        // position anchor.  Anchoring the reference on the requested orbit
        // circumference prevents a velocity-only handoff from cutting across
        // the target during entry, while keeping the public ROS messages
        // unchanged.  The anchor is recomputed in the same odometry frame as
        // the vehicle state, so it remains valid through yaw changes.
        if (config_.target_guidance.publish_position_reference &&
            config_.profile == FollowerProfile::kFixedWingVelocityVector &&
            have_metric_world_state_ && vehicle_state_.pose_valid &&
            vehicle_state_.valid) {
          // Publish the circumference anchor during both pursuit and orbit.
          // Waiting until orbit_active lets a fast fixed-wing cross the target
          // before the first position reference is accepted; the resulting
          // capture overshoot is then counted as an out-of-tolerance radius
          // interval.  The target world estimate is already gated by the
          // metric filter, so using it here is safe before capture as well.
          const double dx = vehicle_state_.x - last_metric_world_position_[0];
          const double dy = vehicle_state_.y - last_metric_world_position_[1];
          const double distance = std::hypot(dx, dy);
          const double anchor_radius = guidance.effective_radius_m > 1.0
              ? guidance.effective_radius_m
              : config_.target_guidance.orbit_radius_m;
          if (std::isfinite(distance) && distance > 1e-3 &&
              std::isfinite(anchor_radius)) {
            output.position_reference_valid = true;
            output.position_reference[0] = last_metric_world_position_[0] +
                anchor_radius * dx / distance;
            output.position_reference[1] = last_metric_world_position_[1] +
                anchor_radius * dy / distance;
            // The metric target depth is not an altitude measurement.  When
            // a fixed-wing entry altitude is configured, use that odometry-Z
            // setpoint for the whole orbit instead of chasing the current
            // height (which would make the altitude loop blind to a climb).
            // A negative value preserves the deployment-safe current-height
            // fallback for vehicles without a calibrated altitude reference.
            output.position_reference[2] =
                config_.target_guidance.fixed_wing_entry_altitude_reference_m >= 0.0
                    ? config_.target_guidance.fixed_wing_entry_altitude_reference_m
                    : vehicle_state_.z;
          }
        }
        if (!config_.enable_vertical_control) output.up = 0.0;
        output.release_reference_on_invalid = false;
        output.valid = true;
        output.target_visible = fresh && !measurement_.predicted;
        output.target_predicted = !fresh || measurement_.predicted;
        output.tracking_state = guidance.state;
        if (metric_coast) {
          const double coast_age = std::max(0.0, metric_elapsed -
              config_.input_timeout_sec);
          if (coast_age <= config_.target_guidance.target_loss_coast_sec) {
            output.tracking_state = "coast";
          } else if (coast_age <= config_.target_guidance.target_loss_coast_sec +
                     config_.target_guidance.target_loss_orbit_sec) {
            output.tracking_state = "orbit_coast";
          } else {
            output.tracking_state = "center_hold";
          }
          output.target_loss_duration_sec = coast_age;
        }
        output.invalid_reason.clear();
        return output;
      }
      if (metric_mode_requested && !guidance.invalid_reason.empty()) {
        output.valid = false;
        output.target_visible = false;
        output.tracking_state = guidance.state;
        output.invalid_reason = guidance.invalid_reason;
        output.release_reference_on_invalid =
            config_.profile == FollowerProfile::kFixedWingVelocityVector;
        return output;
      }
    }
  }

  if (!fresh) {
    output.target_predicted = false;
    if (!ever_acquired_) {
      output.tracking_state = "waiting_for_target";
      output.invalid_reason = cleared_reason_.empty()
          ? "target box has not been received" : cleared_reason_;
      return output;
    }
    if (!loss_active_) {
      loss_active_ = true;
      loss_start_time_ = now;
      lateral_pid_.reset();
      vertical_pid_.reset();
      yaw_pid_.reset();
      ground_forward_pid_.reset();
      fw_course_pid_.reset();
      fw_climb_rate_pid_.reset();
      smoothed_left_ = 0.0;
      smoothed_up_ = 0.0;
      yaw_rate_limited_ = 0.0;
      smoothed_yaw_ = 0.0;
    }
    output.target_loss_duration_sec = std::max(0.0, now - loss_start_time_);
    output.tracking_state = "target_lost";
    output.invalid_reason = cleared_reason_.empty()
        ? "target is unavailable" : cleared_reason_;
    if (config_.profile == FollowerProfile::kFixedWingVelocityVector) {
      // A fixed-wing cannot execute a zero-velocity stop. Mark the command
      // invalid so the node releases the stream and the downstream controller
      // transitions to its configured reference-timeout loiter.
      output.valid = false;
      if (!loss_controllers_reset_ &&
          output.target_loss_duration_sec >= config_.target_loss_reset_sec) {
        filtered_velocity_x_ = 0.0;
        filtered_velocity_y_ = 0.0;
        confidence_locked_ = false;
        loss_controllers_reset_ = true;
      }
      return output;
    }
    if (config_.profile == FollowerProfile::kVelocityChase) {
      forward_velocity_ = moveToward(
          forward_velocity_, config_.target_loss_forward_velocity,
          config_.forward_ramp_rate * dt);
    } else {
      forward_velocity_ = 0.0;
    }
    output.forward = forward_velocity_;
    output.valid = true;  // Intentional deceleration/zero command after acquisition.
    if (!loss_controllers_reset_ &&
        output.target_loss_duration_sec >= config_.target_loss_reset_sec) {
      filtered_velocity_x_ = 0.0;
      filtered_velocity_y_ = 0.0;
      confidence_locked_ = false;
      loss_controllers_reset_ = true;
    }
    return output;
  }

  loss_active_ = false;
  loss_controllers_reset_ = false;
  output.target_visible = true;
  output.valid = true;
  output.target_predicted = measurement_.predicted;
  output.tracking_state = measurement_.predicted ? "predicting" : "tracking";
  output.error_x = clamp(filtered_center_x_ - config_.target_x, -2.0, 2.0);
  output.error_y = clamp(filtered_center_y_ - config_.target_y, -2.0, 2.0);
  const double error_x = applyDeadband(
      output.error_x, config_.horizontal_deadband);
  const double error_y = applyDeadband(
      output.error_y, config_.vertical_deadband);

  double safety_scale = 1.0;
  if (config_.uncertainty_control_enabled) {
    const double sigma = std::sqrt(std::max(
        0.0, std::max(measurement_.state_covariance[0],
                      measurement_.state_covariance[1])));
    const double ratio = sigma / config_.uncertainty_nominal_sigma;
    if (ratio >= config_.uncertainty_abort_ratio) {
      output.valid = false;
      output.tracking_state = "uncertainty_abort";
      output.invalid_reason = "track covariance exceeds the abort limit";
      output.uncertainty_scale = 0.0;
      return output;
    }
    if (ratio > config_.uncertainty_slow_ratio) {
      safety_scale = std::max(config_.uncertainty_minimum_scale,
                              1.0 / std::max(1.0, ratio));
      output.uncertainty_limited = true;
    }
  }
  if (reidentification_time_ >= 0.0 &&
      config_.reidentification_recovery_sec > 0.0) {
    const double progress = clamp(
        (now - reidentification_time_) /
            config_.reidentification_recovery_sec,
        0.0, 1.0);
    const double reid_scale = config_.reidentification_initial_scale +
        progress * (1.0 - config_.reidentification_initial_scale);
    safety_scale = std::min(safety_scale, reid_scale);
    output.uncertainty_limited = safety_scale < 1.0;
  }
  output.uncertainty_scale = safety_scale;

  double desired_forward = 0.0;
  double desired_left = 0.0;
  double desired_up = 0.0;
  double desired_yaw = 0.0;
  if (config_.enable_vertical_control &&
      config_.profile != FollowerProfile::kVelocityGround &&
      config_.profile != FollowerProfile::kGimbalVelocityChase &&
      config_.profile != FollowerProfile::kGimbalVelocityVector &&
      config_.profile != FollowerProfile::kFixedWingVelocityVector) {
    desired_up = vertical_pid_.update(-error_y, dt);
  } else {
    vertical_pid_.reset();
  }

  switch (config_.profile) {
    case FollowerProfile::kVelocityGround: {
      double ground_x = error_x;
      double ground_y = error_y;
      if (config_.ground_attitude_compensation_enabled &&
          vehicle_state_.valid) {
        ground_x += config_.ground_base_adjustment_x * vehicle_state_.roll;
        ground_y -= config_.ground_base_adjustment_y * vehicle_state_.pitch;
      }
      if (config_.ground_altitude_scaling_enabled && vehicle_state_.valid) {
        const double altitude = std::max(0.0, vehicle_state_.altitude);
        ground_x *= 1.0 + config_.ground_base_adjustment_x /
            (1.0 + config_.ground_altitude_factor * altitude);
        ground_y *= 1.0 + config_.ground_base_adjustment_y /
            (1.0 + config_.ground_altitude_factor * altitude);
      }
      // For the downward optical frame, image +Y points toward body -X.
      // Therefore a target below the image centre requires negative body-FLU
      // forward velocity; using +ground_y makes the error diverge.
      desired_forward = ground_forward_pid_.update(-ground_y, dt);
      desired_left = lateral_pid_.update(-ground_x, dt);
      if (config_.ground_descend_to_target && vehicle_state_.valid &&
          vehicle_state_.altitude > config_.ground_target_altitude) {
        desired_up = -std::min(
            config_.ground_maximum_descent_velocity,
            std::max(0.0, vehicle_state_.altitude -
                              config_.ground_target_altitude));
      } else {
        desired_up = 0.0;
      }
      yaw_pid_.reset();
      break;
    }
    case FollowerProfile::kVelocityPosition:
      desired_yaw = yaw_pid_.update(-error_x, dt);
      lateral_pid_.reset();
      break;
    case FollowerProfile::kVelocityDistance:
      desired_left = lateral_pid_.update(-error_x, dt);
      if (config_.distance_enable_yaw) {
        desired_yaw = yaw_pid_.update(-error_x, dt);
      } else {
        yaw_pid_.reset();
      }
      break;
    case FollowerProfile::kVelocityChase:
      desired_forward = config_.chase_forward_velocity;
      if (config_.lateral_guidance == LateralGuidanceMode::kCoordinatedTurn) {
        desired_yaw = yaw_pid_.update(-error_x, dt);
        lateral_pid_.reset();
      } else {
        desired_left = lateral_pid_.update(-error_x, dt);
        yaw_pid_.reset();
      }
      break;
    case FollowerProfile::kGimbalVelocityChase: {
      double gimbal_yaw = 0.0;
      double gimbal_pitch = 0.0;
      double gimbal_roll = 0.0;
      if (!filteredGimbalAngles(now, &gimbal_yaw, &gimbal_pitch,
                                &gimbal_roll)) {
        output.valid = false;
        output.target_visible = false;
        output.tracking_state = "waiting_for_gimbal_state";
        output.invalid_reason =
            "gm_velocity_chase requires a fresh valid gimbal state";
        return output;
      }
      double unit_forward = 0.0;
      double unit_right = 0.0;
      double unit_down = 0.0;
      gimbalToBodyVector(gimbal_yaw, gimbal_pitch, gimbal_roll,
                         &unit_forward, &unit_right, &unit_down);
      const double horizontal = std::hypot(unit_forward, unit_right);
      const double lateral_error = clamp(
          std::atan2(unit_right, std::max(1e-9, unit_forward)) /
              config_.gm_chase_maximum_lateral_angle,
          -1.0, 1.0);
      const double vertical_error = clamp(
          std::atan2(unit_down, std::max(1e-9, horizontal)) /
              config_.gm_chase_maximum_vertical_angle,
          -1.0, 1.0);
      double target_forward = std::min(config_.gm_chase_base_forward_speed,
                                       config_.maximum_forward_velocity);
      if (config_.gm_chase_forward_mode == "PITCH_BASED") {
        const double neutral_pitch = config_.gimbal_mount_type == "VERTICAL"
            ? 1.5707963267948966
            : (config_.gimbal_mount_type == "TILTED_45"
                   ? 0.7853981633974483 : 0.0);
        const double pitch_error = std::abs(gimbal_pitch - neutral_pitch);
        target_forward = pitch_error > config_.gm_chase_pitch_deadzone
            ? std::min(config_.maximum_forward_velocity,
                       pitch_error * 57.29577951308232 *
                           config_.gm_chase_pitch_velocity_scaling)
            : 0.0;
      }
      gm_chase_forward_velocity_ = moveToward(
          gm_chase_forward_velocity_, target_forward,
          config_.gm_chase_forward_acceleration * dt);
      desired_forward = gm_chase_forward_velocity_;
      desired_up = vertical_pid_.update(-vertical_error, dt);
      if (config_.gm_chase_lateral_guidance ==
          LateralGuidanceMode::kCoordinatedTurn) {
        desired_yaw = yaw_pid_.update(-lateral_error, dt);
        lateral_pid_.reset();
      } else {
        desired_left = lateral_pid_.update(-lateral_error, dt);
        yaw_pid_.reset();
      }
      break;
    }
    case FollowerProfile::kGimbalVelocityVector: {
      double gimbal_yaw = 0.0;
      double gimbal_pitch = 0.0;
      double gimbal_roll = 0.0;
      if (!filteredGimbalAngles(now, &gimbal_yaw, &gimbal_pitch,
                                &gimbal_roll)) {
        output.valid = false;
        output.target_visible = false;
        output.tracking_state = "waiting_for_gimbal_state";
        output.invalid_reason =
            "gm_velocity_vector requires a fresh valid gimbal state";
        return output;
      }
      gm_vector_velocity_ = moveToward(
          gm_vector_velocity_,
          std::min(config_.gm_vector_velocity,
                   config_.maximum_forward_velocity),
          config_.gm_vector_ramp_acceleration * dt);
      double unit_forward = 0.0;
      double unit_right = 0.0;
      double unit_down = 0.0;
      gimbalToBodyVector(gimbal_yaw, gimbal_pitch, gimbal_roll,
                         &unit_forward, &unit_right, &unit_down);
      if (!config_.gm_vector_enable_vertical_control) {
        const double horizontal = std::hypot(unit_forward, unit_right);
        if (horizontal > 1e-9) {
          unit_forward /= horizontal;
          unit_right /= horizontal;
        }
        unit_down = 0.0;
      }
      desired_forward = std::max(0.0,
          gm_vector_velocity_ * unit_forward);
      desired_up = -gm_vector_velocity_ * unit_down;
      if (config_.gm_vector_lateral_guidance ==
          LateralGuidanceMode::kCoordinatedTurn) {
        desired_left = 0.0;
        desired_yaw = clamp(-config_.gm_vector_yaw_rate_gain * gimbal_yaw,
                            -config_.maximum_yaw_rate,
                            config_.maximum_yaw_rate);
      } else {
        desired_left = -gm_vector_velocity_ * unit_right;
        desired_yaw = 0.0;
      }
      lateral_pid_.reset();
      vertical_pid_.reset();
      yaw_pid_.reset();
      break;
    }
    case FollowerProfile::kFixedWingVelocityVector: {
      // The horizontal vector direction is the requested course relative to
      // current heading. Its magnitude is the requested airspeed. Positive
      // image x is right, while body-FLU y is left, hence the negative error.
      const double course_offset =
          safety_scale * fw_course_pid_.update(-error_x, dt);
      const double airspeed = config_.fw_commanded_airspeed;
      output.forward = airspeed * std::cos(course_offset);
      output.left = airspeed * std::sin(course_offset);
      output.up = config_.enable_vertical_control
          ? fw_climb_rate_pid_.update(-error_y, dt) : 0.0;
      output.up *= safety_scale;
      output.yaw_rate = 0.0;
      output.use_yaw_rate = false;
      return output;
    }
  }

  if (config_.profile == FollowerProfile::kVelocityGround) {
    const double alpha = config_.velocity_smoothing_enabled
        ? config_.velocity_ema_alpha : 1.0;
    forward_velocity_ += alpha * (desired_forward - forward_velocity_);
  } else if (config_.profile == FollowerProfile::kGimbalVelocityChase ||
             config_.profile == FollowerProfile::kGimbalVelocityVector) {
    forward_velocity_ = desired_forward;
  } else {
    forward_velocity_ = moveToward(
        forward_velocity_, desired_forward, config_.forward_ramp_rate * dt);
  }
  if (config_.velocity_smoothing_enabled) {
    smoothed_left_ += config_.velocity_ema_alpha *
        (desired_left - smoothed_left_);
    smoothed_up_ += config_.velocity_ema_alpha *
        (desired_up - smoothed_up_);
  } else {
    smoothed_left_ = desired_left;
    smoothed_up_ = desired_up;
  }
  const double minimum_forward = config_.profile ==
          FollowerProfile::kVelocityGround
      ? -config_.maximum_reverse_velocity : 0.0;
  output.forward = clamp(forward_velocity_, minimum_forward,
                         config_.maximum_forward_velocity);
  output.left = clamp(smoothed_left_, -config_.maximum_lateral_velocity,
                      config_.maximum_lateral_velocity);
  output.up = clamp(smoothed_up_, -config_.maximum_vertical_velocity,
                    config_.maximum_vertical_velocity);
  output.yaw_rate = clamp(smoothYaw(desired_yaw, dt, output.forward),
                          -config_.maximum_yaw_rate,
                          config_.maximum_yaw_rate);
  if (config_.relative_state_control_enabled &&
      (config_.profile == FollowerProfile::kVelocityChase ||
       config_.profile == FollowerProfile::kGimbalVelocityChase) &&
      measurement_.range_valid && measurement_.has_relative_position_body) {
    const auto& position = measurement_.relative_position_body;
    const auto& velocity = measurement_.relative_velocity_body;
    const double ff_forward = measurement_.has_relative_velocity_body
        ? config_.relative_velocity_feedforward * velocity[0] : 0.0;
    const double ff_left = measurement_.has_relative_velocity_body
        ? -config_.relative_velocity_feedforward * velocity[1] : 0.0;
    const double ff_up = measurement_.has_relative_velocity_body
        ? -config_.relative_velocity_feedforward * velocity[2] : 0.0;
    const double limit = config_.relative_maximum_correction;
    output.forward += clamp(
        ff_forward + config_.relative_position_gain *
            (position[0] - config_.relative_target_forward),
        -limit, limit);
    output.left += clamp(
        ff_left - config_.relative_position_gain * position[1],
        -limit, limit);
    output.up += clamp(
        ff_up - config_.relative_position_gain *
            (position[2] - config_.relative_target_down),
        -limit, limit);
    output.forward = clamp(output.forward, 0.0,
                           config_.maximum_forward_velocity);
    output.left = clamp(output.left, -config_.maximum_lateral_velocity,
                        config_.maximum_lateral_velocity);
    output.up = clamp(output.up, -config_.maximum_vertical_velocity,
                      config_.maximum_vertical_velocity);
    output.relative_state_active = true;
  }
  output.forward *= safety_scale;
  output.left *= safety_scale;
  output.up *= safety_scale;
  output.yaw_rate *= safety_scale * config_.uncertainty_yaw_scale +
      (1.0 - config_.uncertainty_yaw_scale);
  output.invalid_reason.clear();
  return output;
}

void TrackController::clearMeasurement(const std::string& reason) {
  have_measurement_ = false;
  confidence_locked_ = false;
  cleared_reason_ = reason;
}

bool TrackController::setProfile(const FollowerProfile profile) {
  if (profile == config_.profile) return false;
  config_.profile = profile;
  resetFollowerState();
  return true;
}

FollowerProfile TrackController::profile() const {
  return config_.profile;
}

void TrackController::resetFollowerState() {
  lateral_pid_.reset();
  vertical_pid_.reset();
  yaw_pid_.reset();
  ground_forward_pid_.reset();
  fw_course_pid_.reset();
  fw_climb_rate_pid_.reset();
  forward_velocity_ = config_.initial_forward_velocity;
  smoothed_left_ = 0.0;
  smoothed_up_ = 0.0;
  yaw_rate_limited_ = 0.0;
  smoothed_yaw_ = 0.0;
  gm_chase_forward_velocity_ = 0.0;
  gm_vector_velocity_ = 0.0;
  previous_compute_time_ = 0.0;
}

void TrackController::reset() {
  have_measurement_ = false;
  ever_acquired_ = false;
  confidence_locked_ = false;
  filter_initialized_ = false;
  loss_active_ = false;
  loss_controllers_reset_ = false;
  measurement_ = TargetMeasurement();
  filtered_center_x_ = 0.0;
  filtered_center_y_ = 0.0;
  filtered_velocity_x_ = 0.0;
  filtered_velocity_y_ = 0.0;
  filtered_size_ratio_ = 0.0;
  previous_measurement_time_ = 0.0;
  filtered_track_id_ = -1;
  reidentification_time_ = -1.0;
  have_metric_state_ = false;
  last_metric_measurement_accepted_ = false;
  metric_state_time_ = 0.0;
  last_metric_position_frd_ = {{0.0, 0.0, 0.0}};
  last_metric_velocity_frd_ = {{0.0, 0.0, 0.0}};
  last_metric_velocity_valid_ = false;
  last_metric_sigma_m_ = 1.0;
  have_metric_world_state_ = false;
  metric_world_time_ = 0.0;
  last_metric_world_position_ = {{0.0, 0.0, 0.0}};
  last_metric_world_velocity_ = {{0.0, 0.0, 0.0}};
  last_metric_world_velocity_valid_ = false;
  last_metric_world_sigma_m_ = 1.0;
  world_filter_models_ = {{WorldFilterModel(), WorldFilterModel(),
                           WorldFilterModel()}};
  world_filter_probabilities_ = {{1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0}};
  world_filter_baseline_ = WorldFilterState();
  world_filter_history_.clear();
  vehicle_state_history_.clear();
  if (target_guidance_) target_guidance_->reset();
  gimbal_state_ = GimbalStateData();
  gimbal_filter_initialized_ = false;
  filtered_gimbal_yaw_ = 0.0;
  filtered_gimbal_pitch_ = 0.0;
  filtered_gimbal_roll_ = 0.0;
  loss_start_time_ = 0.0;
  cleared_reason_ = "target box has not been received";
  resetFollowerState();
}

}  // namespace xd_uav_track
