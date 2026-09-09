#include <xd_uav_track/track_controller.hpp>

#include <algorithm>
#include <cmath>

namespace xd_uav_track {

namespace {

bool finiteMeasurement(const TargetMeasurement& value) {
  return std::isfinite(value.receive_time) && std::isfinite(value.x_min) &&
         std::isfinite(value.y_min) && std::isfinite(value.x_max) &&
         std::isfinite(value.y_max) && std::isfinite(value.confidence) &&
         std::isfinite(value.tracking_quality) &&
         std::all_of(value.state_covariance.begin(),
                     value.state_covariance.end(),
                     [](double v) { return std::isfinite(v); });
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
  vehicle_state_ = state;
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
  if (fresh) {
    const double age = now - measurement_.receive_time;
    if (age < -1e-6) {
      fresh = false;
      cleared_reason_ = "target box time is in the future";
    } else if (age > config_.input_timeout_sec) {
      fresh = false;
      cleared_reason_ = "target box is stale";
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
