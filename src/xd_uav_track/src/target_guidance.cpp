#include <xd_uav_track/target_guidance.hpp>

#include <algorithm>
#include <cmath>
#include <Eigen/Geometry>

namespace xd_uav_track {
namespace {

constexpr double kPi = 3.14159265358979323846;

struct CameraProjection {
  bool visible{false};
  // Normalized clearance to the configured usable image boundary. Positive
  // means visible; a larger value gives more tolerance to model/servo error.
  double margin{-1e9};
};

CameraProjection predictCameraProjection(
    const TargetGuidanceConfig& config,
    const TargetGuidanceInput& input,
    const Eigen::Vector3d& target_body_flu,
    const double speed,
    const double yaw_rate,
    const double future_roll) {
  CameraProjection result;
  const double horizon = std::max(0.0, config.fov_prediction_sec);

  // Work in a local frame whose current yaw is zero. The current body attitude
  // first places the measured body-relative target in that frame. The future
  // body pose then follows a constant-rate horizontal arc and a coordinated
  // bank. This avoids the old straight-line subtraction/LOS rotation shortcut,
  // which double-counted part of a turn and ignored pitch/bank camera motion.
  const Eigen::Matrix3d current_body_to_local =
      (Eigen::AngleAxisd(input.vehicle_pitch, Eigen::Vector3d::UnitY()) *
       Eigen::AngleAxisd(input.vehicle_roll, Eigen::Vector3d::UnitX()))
          .toRotationMatrix();
  const double yaw_delta = yaw_rate * horizon;
  Eigen::Vector3d displacement(speed * horizon, 0.0, 0.0);
  if (std::abs(yaw_rate) > 1e-6) {
    displacement.x() = speed / yaw_rate * std::sin(yaw_delta);
    displacement.y() = speed / yaw_rate * (1.0 - std::cos(yaw_delta));
  }
  const Eigen::Matrix3d future_body_to_local =
      (Eigen::AngleAxisd(yaw_delta, Eigen::Vector3d::UnitZ()) *
       Eigen::AngleAxisd(input.vehicle_pitch, Eigen::Vector3d::UnitY()) *
       Eigen::AngleAxisd(future_roll, Eigen::Vector3d::UnitX()))
          .toRotationMatrix();
  Eigen::Vector3d future_body = future_body_to_local.transpose() *
      (current_body_to_local * target_body_flu - displacement);
  future_body -= Eigen::Vector3d(config.camera_translation[0],
                                 config.camera_translation[1],
                                 config.camera_translation[2]);

  const auto& rpy = config.camera_rpy;
  const Eigen::Matrix3d optical_to_body =
      (Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitZ()) *
       Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY()) *
       Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitX()))
          .toRotationMatrix();
  const Eigen::Vector3d optical = optical_to_body.transpose() * future_body;
  if (!optical.allFinite() || optical.z() <= 1e-3) return result;

  const double u = config.camera_fx * optical.x() / optical.z() +
      config.camera_cx;
  const double v = config.camera_fy * optical.y() / optical.z() +
      config.camera_cy;
  const double border_x = config.fixed_camera_fov_margin_ratio *
      config.camera_width;
  const double border_y = config.fixed_camera_fov_margin_ratio *
      config.camera_height;
  const double usable_width = std::max(1.0, config.camera_width - 2.0 * border_x);
  const double usable_height = std::max(1.0, config.camera_height - 2.0 * border_y);
  result.margin = std::min(
      std::min((u - border_x) / usable_width,
               (config.camera_width - border_x - u) / usable_width),
      std::min((v - border_y) / usable_height,
               (config.camera_height - border_y - v) / usable_height));
  result.visible = result.margin >= 0.0;
  return result;
}

}  // namespace

bool parseTargetGuidanceMode(const std::string& value,
                             TargetGuidanceMode* mode) {
  if (mode == nullptr) return false;
  if (value == "legacy_visual" || value == "direct_visual") {
    *mode = TargetGuidanceMode::kLegacyVisual;
    return true;
  }
  if (value == "metric_pursuit") {
    *mode = TargetGuidanceMode::kMetricPursuit;
    return true;
  }
  if (value == "metric_orbit" || value == "standoff_orbit") {
    *mode = TargetGuidanceMode::kMetricOrbit;
    return true;
  }
  if (value == "auto") {
    *mode = TargetGuidanceMode::kAuto;
    return true;
  }
  return false;
}

const char* targetGuidanceModeName(const TargetGuidanceMode mode) {
  switch (mode) {
    case TargetGuidanceMode::kLegacyVisual: return "legacy_visual";
    case TargetGuidanceMode::kMetricPursuit: return "metric_pursuit";
    case TargetGuidanceMode::kMetricOrbit: return "metric_orbit";
    case TargetGuidanceMode::kAuto: return "auto";
  }
  return "legacy_visual";
}

bool parseOrbitDirection(const std::string& value,
                         OrbitDirection* direction) {
  if (direction == nullptr) return false;
  if (value == "auto") {
    *direction = OrbitDirection::kAuto;
    return true;
  }
  if (value == "cw" || value == "clockwise") {
    *direction = OrbitDirection::kClockwise;
    return true;
  }
  if (value == "ccw" || value == "counter_clockwise") {
    *direction = OrbitDirection::kCounterClockwise;
    return true;
  }
  return false;
}

const char* orbitDirectionName(const OrbitDirection direction) {
  switch (direction) {
    case OrbitDirection::kAuto: return "auto";
    case OrbitDirection::kClockwise: return "cw";
    case OrbitDirection::kCounterClockwise: return "ccw";
  }
  return "auto";
}

TargetGuidance::TargetGuidance(const TargetGuidanceConfig& requested)
    : config_(requested) {
  config_.metric_timeout_sec = std::max(0.02, config_.metric_timeout_sec);
  config_.maximum_position_sigma_m =
      std::max(0.0, config_.maximum_position_sigma_m);
  config_.orbit_radius_m = std::max(1.0, config_.orbit_radius_m);
  config_.orbit_capture_hysteresis_m =
      std::max(0.0, config_.orbit_capture_hysteresis_m);
  config_.minimum_turn_radius_m =
      std::max(1.0, config_.minimum_turn_radius_m);
  config_.radial_gain = std::max(0.0, config_.radial_gain);
  config_.radial_damping = std::max(0.0, config_.radial_damping);
  config_.maximum_course_offset = clamp(
      config_.maximum_course_offset, 0.01, kPi - 0.01);
  config_.maximum_course_rate_radps = std::max(
      0.0, config_.maximum_course_rate_radps);
  config_.commanded_speed = std::max(0.1, config_.commanded_speed);
  config_.maximum_lateral_speed = std::max(0.0,
                                            config_.maximum_lateral_speed);
  config_.maximum_vertical_speed = std::max(0.0,
                                             config_.maximum_vertical_speed);
  config_.fixed_camera_fov_margin_ratio = clamp(
      config_.fixed_camera_fov_margin_ratio, 0.0, 0.49);
  config_.dynamic_gimbal_fov_margin_ratio = clamp(
      config_.dynamic_gimbal_fov_margin_ratio, 0.0, 0.49);
  config_.dynamic_gimbal_min_fov_rad = std::max(
      0.01, config_.dynamic_gimbal_min_fov_rad);
  config_.dynamic_gimbal_zoom_reference_ratio = std::max(
      0.0, config_.dynamic_gimbal_zoom_reference_ratio);
  config_.dynamic_gimbal_zoom_margin_gain = std::max(
      0.0, config_.dynamic_gimbal_zoom_margin_gain);
  config_.fov_prediction_sec = std::max(0.0, config_.fov_prediction_sec);
  config_.prediction_process_sigma_mps =
      std::max(0.0, config_.prediction_process_sigma_mps);
  config_.target_loss_hold_process_sigma_mps = std::max(
      0.0, config_.target_loss_hold_process_sigma_mps);
  config_.target_loss_center_hold_sec = std::max(
      0.0, config_.target_loss_center_hold_sec);
  config_.target_loss_center_hold_process_sigma_mps = std::max(
      0.0, config_.target_loss_center_hold_process_sigma_mps);
  config_.fixed_wing_airspeed_mps = std::max(0.1,
                                              config_.fixed_wing_airspeed_mps);
  config_.fixed_wing_maximum_bank_angle_deg = clamp(
      config_.fixed_wing_maximum_bank_angle_deg, 1.0, 89.0);
  config_.gravity_mps2 = std::max(0.1, config_.gravity_mps2);
  config_.fixed_wing_altitude_gain = std::max(
      0.0, config_.fixed_wing_altitude_gain);
  if (!std::isfinite(config_.fixed_wing_entry_altitude_reference_m)) {
    config_.fixed_wing_entry_altitude_reference_m = -1.0;
  }
  config_.target_loss_coast_sec = std::max(0.0,
                                            config_.target_loss_coast_sec);
  config_.target_loss_orbit_sec = std::max(
      config_.target_loss_coast_sec, config_.target_loss_orbit_sec);
  config_.inertial_coast_state_timeout_sec = std::max(
      0.0, config_.inertial_coast_state_timeout_sec);
  config_.world_filter_position_alpha = clamp(
      config_.world_filter_position_alpha, 0.0, 1.0);
  config_.world_filter_max_innovation_m = std::max(
      0.0, config_.world_filter_max_innovation_m);
  config_.world_filter_max_speed_mps = std::max(
      0.0, config_.world_filter_max_speed_mps);
  config_.world_filter_process_accel_stddev_mps2 = std::max(
      0.0, config_.world_filter_process_accel_stddev_mps2);
  config_.world_filter_mahalanobis_gate = std::max(
      0.0, config_.world_filter_mahalanobis_gate);
  config_.world_filter_imm_static_accel_stddev_mps2 = std::max(
      0.0, config_.world_filter_imm_static_accel_stddev_mps2);
  config_.world_filter_imm_maneuver_accel_stddev_mps2 = std::max(
      config_.world_filter_imm_static_accel_stddev_mps2,
      config_.world_filter_imm_maneuver_accel_stddev_mps2);
  config_.world_filter_imm_min_probability = clamp(
      config_.world_filter_imm_min_probability, 0.0, 0.49);
  config_.world_filter_imm_turn_rate_stddev_radps2 = std::max(
      0.0, config_.world_filter_imm_turn_rate_stddev_radps2);
  config_.world_filter_imm_static_persistence = clamp(
      config_.world_filter_imm_static_persistence, 0.0, 0.999);
  config_.world_filter_imm_cv_persistence = clamp(
      config_.world_filter_imm_cv_persistence, 0.0, 0.999);
  config_.world_filter_imm_turn_persistence = clamp(
      config_.world_filter_imm_turn_persistence, 0.0, 0.999);
  config_.world_filter_imm_max_turn_rate_radps = std::max(
      0.0, config_.world_filter_imm_max_turn_rate_radps);
  config_.world_filter_oosm_window_sec = std::max(
      0.0, config_.world_filter_oosm_window_sec);
  config_.world_filter_oosm_covariance_inflation = std::max(
      1.0, config_.world_filter_oosm_covariance_inflation);
  config_.world_filter_pose_history_sec = std::max(
      0.0, config_.world_filter_pose_history_sec);
  config_.multi_source_max_age_sec = std::max(
      0.0, config_.multi_source_max_age_sec);
  if (config_.observation_policy != "image_only" &&
      config_.observation_policy != "metric_required" &&
      config_.observation_policy != "metric_preferred") {
    config_.observation_policy = "metric_preferred";
  }
}

double TargetGuidance::clamp(const double value, const double minimum,
                             const double maximum) {
  return std::max(minimum, std::min(maximum, value));
}

double TargetGuidance::wrapAngle(double angle) {
  while (angle > kPi) angle -= 2.0 * kPi;
  while (angle < -kPi) angle += 2.0 * kPi;
  return angle;
}

double TargetGuidance::vectorNorm(const double x, const double y) {
  return std::hypot(x, y);
}

bool TargetGuidance::finiteInput(const TargetGuidanceInput& input) {
  const auto finite = [](const double value) { return std::isfinite(value); };
  if (!finite(input.now) || !finite(input.dt) ||
      !finite(input.observation_time) ||
      !finite(input.image_error_x) || !finite(input.image_error_y) ||
      !finite(input.position_sigma_m) || !finite(input.commanded_speed) ||
      !finite(input.bearing_yaw) || !finite(input.bearing_pitch) ||
      !finite(input.vehicle_altitude_m) || !finite(input.gimbal_yaw_return) ||
      !finite(input.gimbal_pitch_return) || !finite(input.vehicle_roll) ||
      !finite(input.vehicle_pitch) ||
      !finite(input.gimbal_horizontal_fov_rad) ||
      !finite(input.gimbal_vertical_fov_rad) ||
      !finite(input.gimbal_zoom_ratio)) {
    return false;
  }
  for (const double value : input.relative_position_body_frd) {
    if (!finite(value)) return false;
  }
  for (const double value : input.relative_velocity_body_frd) {
    if (!finite(value)) return false;
  }
  return true;
}

TargetGuidanceOutput TargetGuidance::update(
    const TargetGuidanceInput& input) {
  TargetGuidanceOutput output;
  output.mode = targetGuidanceModeName(config_.mode);
  if (!config_.enabled || config_.mode == TargetGuidanceMode::kLegacyVisual) {
    output.active = false;
    output.state = "visual";
    return output;
  }
  if (!finiteInput(input) || input.now < 0.0) {
    output.invalid_reason = "target guidance input is non-finite";
    output.state = "invalid";
    return output;
  }

  // Do not disable the course-rate limiter when ROS/Gazebo briefly stalls.
  // A bounded nominal step preserves a continuous command on reacquisition;
  // the controller's stale-input policy remains responsible for releasing
  // the stream after the configured orbit timeout.
  const double dt = input.dt > 1e-4
      ? std::min(input.dt, 0.5) : 1.0 / 30.0;
  const bool metric_allowed = config_.observation_policy != "image_only";
  const bool position_finite = metric_allowed && input.metric_valid &&
      input.range_valid;
  const double position_sigma = std::max(0.0, input.position_sigma_m);
  const bool metric_usable = position_finite &&
      (config_.maximum_position_sigma_m <= 0.0 ||
       position_sigma <= config_.maximum_position_sigma_m);
  const double metric_age = input.now - input.observation_time;
  const bool metric_fresh = metric_usable && metric_age >= -1e-3 &&
      metric_age <= config_.metric_timeout_sec;
  output.range_accepted = metric_usable;

  const bool metric_required = config_.observation_policy == "metric_required";
  const bool use_metric = config_.mode == TargetGuidanceMode::kMetricPursuit ||
      config_.mode == TargetGuidanceMode::kMetricOrbit ||
      config_.mode == TargetGuidanceMode::kAuto;
  const bool metric_mode_requires_observation =
      config_.mode == TargetGuidanceMode::kMetricPursuit ||
      config_.mode == TargetGuidanceMode::kMetricOrbit;
  if ((metric_required || metric_mode_requires_observation) && !metric_fresh) {
    output.invalid_reason = "metric target observation is stale or invalid";
    output.state = "metric_required";
    return output;
  }
  if (config_.mode == TargetGuidanceMode::kAuto && !metric_fresh &&
      config_.observation_policy != "image_only" && input.bearing_valid) {
    // A gimbal can keep the target in its own image while the airframe still
    // needs a course correction. Use the LOS yaw as a bearing-only fallback;
    // no radial or altitude command is inferred without a range.
    output.active = true;
    output.state = "bearing_only";
    output.metric_active = false;
    output.range_accepted = false;
    const double requested_bearing_course = clamp(
        -input.bearing_yaw, -config_.maximum_course_offset,
        config_.maximum_course_offset);
    output.course_offset = requested_bearing_course;
    const double bearing_dt = input.dt > 1e-4 ? std::min(input.dt, 0.5)
                                              : 1.0 / 30.0;
    if (config_.maximum_course_rate_radps > 0.0 && bearing_dt > 0.0) {
      const double previous_course = course_offset_initialized_
          ? previous_course_offset_ : 0.0;
      const double delta = wrapAngle(requested_bearing_course - previous_course);
      const double limited_delta = clamp(
          delta, -config_.maximum_course_rate_radps * bearing_dt,
          config_.maximum_course_rate_radps * bearing_dt);
      output.course_offset = clamp(
          wrapAngle(previous_course + limited_delta),
          -config_.maximum_course_offset, config_.maximum_course_offset);
      output.course_rate = limited_delta / bearing_dt;
    }
    const double speed = input.commanded_speed > 0.1
        ? input.commanded_speed : config_.commanded_speed;
    output.forward = speed * std::cos(output.course_offset);
    output.left = speed * std::sin(output.course_offset);
    output.up = 0.0;
    output.invalid_reason.clear();
    previous_course_offset_ = output.course_offset;
    course_offset_initialized_ = true;
    previous_time_ = input.now;
    return output;
  }
  if (!use_metric || !metric_fresh) {
    // The caller keeps the existing PID image path. Returning inactive is
    // intentional: it makes auto mode degrade safely to the legacy behavior.
    output.active = false;
    output.state = "visual_fallback";
    return output;
  }

  // DetectionCandidate is body-FRD. Convert to the controller's body-FLU
  // convention before building a velocity vector.
  const double target_forward = input.relative_position_body_frd[0];
  const double target_left = -input.relative_position_body_frd[1];
  const double target_up = -input.relative_position_body_frd[2];
  const double horizontal_range = vectorNorm(target_forward, target_left);
  if (horizontal_range < 1e-3) {
    output.invalid_reason = "metric target is too close to define a course";
    output.state = "metric_degenerate";
    return output;
  }

  const double requested_speed = input.commanded_speed > 0.1
      ? input.commanded_speed : config_.commanded_speed;
  double kinematic_radius = 0.0;
  if (config_.fixed_wing_kinematics_enabled) {
    const double bank = config_.fixed_wing_maximum_bank_angle_deg * kPi / 180.0;
    kinematic_radius = config_.fixed_wing_airspeed_mps *
        config_.fixed_wing_airspeed_mps /
        (config_.gravity_mps2 * std::tan(bank));
  }
  const double radius = std::max({config_.orbit_radius_m,
                                  config_.minimum_turn_radius_m,
                                  kinematic_radius});
  output.effective_radius_m = radius;
  output.radial_error_m = horizontal_range - radius;

  double desired_forward = target_forward / horizontal_range * requested_speed;
  double desired_left = target_left / horizontal_range * requested_speed;
  double desired_up = 0.0;
  if (config_.fixed_wing_entry_altitude_hold_enabled) {
    // Hold the aircraft's entry altitude, not the target-relative down
    // component.  A ground target can legitimately be far below the aircraft
    // and must never turn that geometric range into an unsolicited descent.
    if (input.vehicle_altitude_valid) {
      if (!entry_altitude_initialized_) {
        entry_altitude_m_ =
            config_.fixed_wing_entry_altitude_reference_m >= 0.0
                ? config_.fixed_wing_entry_altitude_reference_m
                : input.vehicle_altitude_m +
                      config_.fixed_wing_entry_altitude_offset_m;
        entry_altitude_initialized_ = true;
      }
      desired_up = config_.fixed_wing_altitude_gain *
          (entry_altitude_m_ - input.vehicle_altitude_m);
    }
  } else if (config_.metric_vertical_control) {
    desired_up = target_up / horizontal_range * requested_speed * 0.15;
  }
  output.state = "pursuit";
  output.metric_active = true;

  bool fov_limited = config_.fixed_camera_fov_constraint_enabled &&
      input.source_is_fixed_camera && !input.source_is_gimbal &&
      (std::abs(input.image_error_x) >=
           1.0 - config_.fixed_camera_fov_margin_ratio ||
       std::abs(input.image_error_y) >=
           1.0 - config_.fixed_camera_fov_margin_ratio);
  bool gimbal_fov_edge = false;
  if (config_.dynamic_gimbal_fov_enabled && input.source_is_gimbal) {
    if (!input.gimbal_fov_valid ||
        input.gimbal_horizontal_fov_rad < config_.dynamic_gimbal_min_fov_rad ||
        input.gimbal_vertical_fov_rad < config_.dynamic_gimbal_min_fov_rad) {
      output.state = "gimbal_fov_unavailable";
      output.invalid_reason =
          "dynamic gimbal FOV requires a fresh calibrated horizontal/vertical FOV";
      return output;
    }
    // Normalized detector coordinates already include current zoom.  Increase
    // the protected image band as zoom narrows the optical FOV, leaving room
    // for payload servo latency and a delayed detector frame.
    const double zoom_margin = std::max(
        0.0, input.gimbal_zoom_ratio -
            config_.dynamic_gimbal_zoom_reference_ratio) *
        config_.dynamic_gimbal_zoom_margin_gain;
    const double margin = clamp(config_.dynamic_gimbal_fov_margin_ratio +
        zoom_margin, 0.0, 0.49);
    gimbal_fov_edge = std::abs(input.image_error_x) >= 1.0 - margin ||
        std::abs(input.image_error_y) >= 1.0 - margin;
    if (gimbal_fov_edge) output.state = "gimbal_fov_edge";
  }
  if (config_.predictive_fov_enabled && input.source_is_fixed_camera &&
      !input.source_is_gimbal) {
    if (!(config_.camera_fx > 0.0 && config_.camera_fy > 0.0 &&
          config_.camera_width > 0.0 && config_.camera_height > 0.0)) {
      output.state = "invalid_camera_calibration";
      output.invalid_reason = "predictive FOV requires positive camera intrinsics";
      return output;
    }
    const double turn = config_.gravity_mps2 * std::tan(clamp(input.vehicle_roll,
        -1.2, 1.2)) / std::max(0.1, requested_speed);
    const CameraProjection projection = predictCameraProjection(
        config_, input, Eigen::Vector3d(target_forward, target_left, target_up),
        requested_speed, turn, input.vehicle_roll);
    fov_limited = fov_limited || !projection.visible;
  }
  if (config_.mode == TargetGuidanceMode::kMetricOrbit ||
      config_.mode == TargetGuidanceMode::kAuto) {
    if (fov_limited &&
        !(orbit_active_ && config_.fixed_camera_preserve_orbit_on_fov_loss)) {
      // Before orbit capture, a target at the image boundary should be
      // recaptured by pursuit.  After capture, callers may explicitly keep
      // the inertial tangent orbit through a short FOV excursion; otherwise
      // this branch preserves the legacy reacquisition behaviour.
      orbit_active_ = false;
      output.state = "fov_reacquire";
    }
    if (!fov_limited && !orbit_active_ && horizontal_range <= radius +
                                      config_.orbit_capture_hysteresis_m) {
      orbit_active_ = true;
    }
    if (orbit_active_ &&
        (!fov_limited || config_.fixed_camera_preserve_orbit_on_fov_loss)) {
      if (!orbit_direction_initialized_) {
        if (config_.orbit_direction == OrbitDirection::kClockwise) {
          orbit_sign_ = -1;
        } else if (config_.orbit_direction ==
                   OrbitDirection::kCounterClockwise) {
          orbit_sign_ = 1;
        } else {
          bool selected_by_visibility = false;
          if (config_.fixed_camera_visibility_direction_selection_enabled &&
              config_.predictive_fov_enabled && input.source_is_fixed_camera &&
              !input.source_is_gimbal) {
            const double bank_magnitude = std::atan(
                requested_speed * requested_speed /
                std::max(1e-3, config_.gravity_mps2 * radius));
            const Eigen::Vector3d target(target_forward, target_left, target_up);
            const CameraProjection clockwise = predictCameraProjection(
                config_, input, target, requested_speed,
                -requested_speed / radius, -bank_magnitude);
            const CameraProjection counter_clockwise = predictCameraProjection(
                config_, input, target, requested_speed,
                requested_speed / radius, bank_magnitude);
            if (clockwise.visible != counter_clockwise.visible ||
                std::abs(clockwise.margin - counter_clockwise.margin) > 1e-6) {
              orbit_sign_ = counter_clockwise.margin > clockwise.margin ? 1 : -1;
              selected_by_visibility = true;
            }
          }
          if (!selected_by_visibility) {
            // Deterministic fallback for uncalibrated/tied projections. The
            // sign is latched, avoiding rapid direction changes near center.
            orbit_sign_ = input.image_error_x >= 0.0 ? -1 : 1;
          }
        }
        orbit_direction_initialized_ = true;
      }
      const double radial_forward = target_forward / horizontal_range;
      const double radial_left = target_left / horizontal_range;
      const double tangent_forward = -static_cast<double>(orbit_sign_) *
          radial_left;
      const double tangent_left = static_cast<double>(orbit_sign_) *
          radial_forward;
      const double radial_velocity = dt > 0.0
          ? (output.radial_error_m - last_radial_error_) / dt : 0.0;
      // relative_position_body points from the aircraft toward the target, so
      // this radial unit vector is inward.  A positive (outside) range error
      // therefore needs a positive correction along that vector; an inside
      // error naturally produces an outward (negative) correction.
      // The radial error is e = range - orbit_radius while a positive
      // correction points inward (toward the target).  Hence e_dot = -u_r;
      // adding D*e_dot supplies physical damping, whereas subtracting it
      // would reinforce the inward/outward oscillation.
      const double radial_correction = config_.radial_gain *
          output.radial_error_m + config_.radial_damping * radial_velocity;
      desired_forward = requested_speed * tangent_forward +
          radial_correction * radial_forward;
      desired_left = requested_speed * tangent_left +
          radial_correction * radial_left;
      output.state = "orbit";
      output.orbit_active = true;
      output.course_rate = static_cast<double>(orbit_sign_) * requested_speed /
          std::max(1.0, radius);
    }
  }
  // Preserve the edge condition in diagnostics even when the inertial orbit
  // remains active. Previously the orbit branch overwrote this state, making
  // a zoom-induced early-recenter request invisible to the caller.
  if (gimbal_fov_edge && output.state == "orbit") {
    output.state = "gimbal_fov_edge";
  }

  if (config_.target_velocity_feedforward && input.relative_velocity_valid) {
    desired_forward += input.relative_velocity_body_frd[0];
    desired_left -= input.relative_velocity_body_frd[1];
    desired_up -= input.relative_velocity_body_frd[2];
  }
  if (input.source_is_gimbal) {
    // Positive gimbal yaw is to the right.  A right-side soft-limit bias asks
    // the fixed-wing course to move right as well, bringing the LOS back
    // toward the gimbal centre.  Pitch is applied only when vertical metric
    // control is explicitly enabled; altitude-hold remains authoritative.
    desired_left -= requested_speed * clamp(input.gimbal_yaw_return, -1.0, 1.0);
    if (!config_.fixed_wing_entry_altitude_hold_enabled &&
        config_.metric_vertical_control) {
      desired_up -= requested_speed *
          clamp(input.gimbal_pitch_return, -1.0, 1.0);
    }
    if (gimbal_fov_edge) {
      // Softly bias the aircraft toward the live image centre. The gimbal's
      // own angular soft-limit bias remains active as a second, independent
      // guard and no hard snap is introduced in the course command.
      desired_left -= requested_speed * 0.35 *
          clamp(input.image_error_x, -1.0, 1.0);
      if (config_.metric_vertical_control &&
          !config_.fixed_wing_entry_altitude_hold_enabled) {
        desired_up -= requested_speed * 0.15 *
            clamp(input.image_error_y, -1.0, 1.0);
      }
    }
  }
  const double requested_course = clamp(wrapAngle(
      std::atan2(desired_left, desired_forward)),
      -config_.maximum_course_offset, config_.maximum_course_offset);
  if (config_.wind_feasibility_enabled) {
    if (!input.wind_valid || !std::isfinite(input.wind_forward) || !std::isfinite(input.wind_left)) {
      output.state = "wind_unavailable";
      output.invalid_reason = "wind feasibility requires a fresh wind estimate";
      return output;
    }
    const double crosswind = -input.wind_forward * std::sin(requested_course) +
        input.wind_left * std::cos(requested_course);
    const double alongwind = input.wind_forward * std::cos(requested_course) +
        input.wind_left * std::sin(requested_course);
    const double airspeed = config_.fixed_wing_airspeed_mps;
    if (std::abs(crosswind) >= airspeed ||
        std::sqrt(std::max(0.0, airspeed * airspeed - crosswind * crosswind)) + alongwind <= 0.0) {
      output.state = "wind_infeasible";
      output.invalid_reason = "requested ground course is infeasible at configured airspeed";
      return output;
    }
  }
  output.course_offset = requested_course;
  if (config_.maximum_course_rate_radps > 0.0 && dt > 0.0) {
    // Start from a neutral course on the first metric command as well. The
    // previous implementation bypassed the limiter until the second sample,
    // allowing a visual->metric handover to create a one-frame course jump.
    const double previous_course = course_offset_initialized_
        ? previous_course_offset_ : 0.0;
    const double delta = wrapAngle(requested_course - previous_course);
    const double limited_delta = clamp(
        delta, -config_.maximum_course_rate_radps * dt,
        config_.maximum_course_rate_radps * dt);
    output.course_offset = clamp(
        wrapAngle(previous_course + limited_delta),
        -config_.maximum_course_offset, config_.maximum_course_offset);
    output.course_rate = limited_delta / dt;
  } else if (dt > 0.0 && course_offset_initialized_) {
    output.course_rate = wrapAngle(output.course_offset -
                                   previous_course_offset_) / dt;
  }
  // Keep the commanded horizontal speed constant. This is especially
  // important for fixed-wing output, where reducing the vector magnitude
  // would unintentionally become an airspeed command.
  const double speed = std::max(0.1, requested_speed);
  output.forward = speed * std::cos(output.course_offset);
  output.left = speed * std::sin(output.course_offset);
  output.up = clamp(desired_up, -config_.maximum_vertical_speed,
                    config_.maximum_vertical_speed);
  output.active = true;
  output.invalid_reason.clear();
  previous_course_offset_ = output.course_offset;
  course_offset_initialized_ = true;
  last_radial_error_ = output.radial_error_m;
  previous_time_ = input.now;
  return output;
}

void TargetGuidance::reset() {
  orbit_sign_ = 1;
  orbit_direction_initialized_ = false;
  orbit_active_ = false;
  last_radial_error_ = 0.0;
  previous_time_ = 0.0;
  previous_course_offset_ = 0.0;
  course_offset_initialized_ = false;
  entry_altitude_initialized_ = false;
  entry_altitude_m_ = 0.0;
}

}  // namespace xd_uav_track
