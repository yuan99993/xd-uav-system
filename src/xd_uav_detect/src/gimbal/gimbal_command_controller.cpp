#include "xd_uav_detect/gimbal/gimbal_command_controller.hpp"

#include <algorithm>
#include <cmath>

namespace xd_uav_detect {

GimbalCommandController::GimbalCommandController(
    const GimbalControlConfig& requested)
    : config_(requested) {
  if (config_.minimum_yaw_rad > config_.maximum_yaw_rad) {
    std::swap(config_.minimum_yaw_rad, config_.maximum_yaw_rad);
  }
  if (config_.minimum_pitch_rad > config_.maximum_pitch_rad) {
    std::swap(config_.minimum_pitch_rad, config_.maximum_pitch_rad);
  }
  config_.maximum_yaw_rate_rad_s =
      std::max(0.0, config_.maximum_yaw_rate_rad_s);
  config_.maximum_pitch_rate_rad_s =
      std::max(0.0, config_.maximum_pitch_rate_rad_s);
  config_.command_timeout_sec = std::max(0.0, config_.command_timeout_sec);
  config_.home_yaw_rad = clamp(config_.home_yaw_rad,
                               config_.minimum_yaw_rad,
                               config_.maximum_yaw_rad);
  config_.home_pitch_rad = clamp(config_.home_pitch_rad,
                                 config_.minimum_pitch_rad,
                                 config_.maximum_pitch_rad);
}

void GimbalCommandController::setJointState(
    const double yaw_rad, const double pitch_rad,
    const double yaw_velocity_rad_s, const double pitch_velocity_rad_s,
    const double now_sec) {
  if (!std::isfinite(yaw_rad) || !std::isfinite(pitch_rad) ||
      !std::isfinite(yaw_velocity_rad_s) ||
      !std::isfinite(pitch_velocity_rad_s) || !std::isfinite(now_sec)) {
    return;
  }
  measured_yaw_rad_ = yaw_rad;
  measured_pitch_rad_ = pitch_rad;
  measured_yaw_velocity_rad_s_ = yaw_velocity_rad_s;
  measured_pitch_velocity_rad_s_ = pitch_velocity_rad_s;
  if (!have_joint_state_) {
    commanded_yaw_rad_ = clamp(yaw_rad, config_.minimum_yaw_rad,
                               config_.maximum_yaw_rad);
    commanded_pitch_rad_ = clamp(pitch_rad, config_.minimum_pitch_rad,
                                 config_.maximum_pitch_rad);
    if (!have_command_) {
      requested_yaw_rad_ = commanded_yaw_rad_;
      requested_pitch_rad_ = commanded_pitch_rad_;
    }
    last_update_sec_ = now_sec;
  }
  have_joint_state_ = true;
}

bool GimbalCommandController::acceptCommand(
    const unsigned char mode, const double yaw_rad, const double pitch_rad,
    const double yaw_rate_rad_s, const double pitch_rate_rad_s,
    const double now_sec, std::string* rejection_reason) {
  if (!std::isfinite(yaw_rad) || !std::isfinite(pitch_rad) ||
      !std::isfinite(yaw_rate_rad_s) || !std::isfinite(pitch_rate_rad_s) ||
      !std::isfinite(now_sec)) {
    if (rejection_reason != nullptr) *rejection_reason = "non_finite_command";
    return false;
  }
  if (mode != kPosition && mode != kVelocity && mode != kHome) {
    if (rejection_reason != nullptr) *rejection_reason = "unsupported_mode";
    return false;
  }

  active_mode_ = mode;
  if (mode == kHome) {
    requested_yaw_rad_ = config_.home_yaw_rad;
    requested_pitch_rad_ = config_.home_pitch_rad;
    requested_yaw_rate_rad_s_ = config_.maximum_yaw_rate_rad_s;
    requested_pitch_rate_rad_s_ = config_.maximum_pitch_rate_rad_s;
  } else if (mode == kPosition) {
    requested_yaw_rad_ = clamp(yaw_rad, config_.minimum_yaw_rad,
                               config_.maximum_yaw_rad);
    requested_pitch_rad_ = clamp(pitch_rad, config_.minimum_pitch_rad,
                                 config_.maximum_pitch_rad);
    requested_yaw_rate_rad_s_ = yaw_rate_rad_s > 0.0
        ? std::min(yaw_rate_rad_s, config_.maximum_yaw_rate_rad_s)
        : config_.maximum_yaw_rate_rad_s;
    requested_pitch_rate_rad_s_ = pitch_rate_rad_s > 0.0
        ? std::min(pitch_rate_rad_s, config_.maximum_pitch_rate_rad_s)
        : config_.maximum_pitch_rate_rad_s;
  } else {
    requested_yaw_rate_rad_s_ = clamp(
        yaw_rate_rad_s, -config_.maximum_yaw_rate_rad_s,
        config_.maximum_yaw_rate_rad_s);
    requested_pitch_rate_rad_s_ = clamp(
        pitch_rate_rad_s, -config_.maximum_pitch_rate_rad_s,
        config_.maximum_pitch_rate_rad_s);
  }
  last_command_sec_ = now_sec;
  have_command_ = true;
  if (rejection_reason != nullptr) rejection_reason->clear();
  return true;
}

GimbalControlOutput GimbalCommandController::update(const double now_sec) {
  GimbalControlOutput output;
  output.joint_state_valid = have_joint_state_;
  output.active_mode = active_mode_;
  output.yaw_rad = measured_yaw_rad_;
  output.pitch_rad = measured_pitch_rad_;
  output.yaw_velocity_rad_s = measured_yaw_velocity_rad_s_;
  output.pitch_velocity_rad_s = measured_pitch_velocity_rad_s_;
  if (!have_joint_state_ || !std::isfinite(now_sec)) return output;

  const double dt = std::max(0.0, std::min(0.2, now_sec - last_update_sec_));
  last_update_sec_ = now_sec;
  const bool fresh = have_command_ &&
      now_sec >= last_command_sec_ &&
      now_sec - last_command_sec_ <= config_.command_timeout_sec;
  const bool position_goal_active =
      have_command_ && active_mode_ != kVelocity;
  const bool control_active = fresh || position_goal_active;

  if (fresh && active_mode_ == kVelocity) {
    requested_yaw_rad_ = clamp(
        commanded_yaw_rad_ + requested_yaw_rate_rad_s_ * dt,
        config_.minimum_yaw_rad, config_.maximum_yaw_rad);
    requested_pitch_rad_ = clamp(
        commanded_pitch_rad_ + requested_pitch_rate_rad_s_ * dt,
        config_.minimum_pitch_rad, config_.maximum_pitch_rad);
  }

  if (control_active) {
    const double yaw_rate = active_mode_ == kVelocity
        ? config_.maximum_yaw_rate_rad_s : requested_yaw_rate_rad_s_;
    const double pitch_rate = active_mode_ == kVelocity
        ? config_.maximum_pitch_rate_rad_s : requested_pitch_rate_rad_s_;
    commanded_yaw_rad_ = approach(commanded_yaw_rad_, requested_yaw_rad_,
                                  yaw_rate * dt);
    commanded_pitch_rad_ = approach(commanded_pitch_rad_,
                                    requested_pitch_rad_, pitch_rate * dt);
  }

  output.output_valid = true;
  output.command_valid = control_active;
  output.commanded_yaw_rad = commanded_yaw_rad_;
  output.commanded_pitch_rad = commanded_pitch_rad_;
  output.status = !have_command_ ? "waiting_for_command"
      : (control_active ? "active" : "command_timeout_hold");
  return output;
}

double GimbalCommandController::clamp(
    const double value, const double minimum, const double maximum) {
  return std::max(minimum, std::min(maximum, value));
}

double GimbalCommandController::approach(
    const double current, const double target, const double maximum_step) {
  if (maximum_step <= 0.0) return current;
  return current + clamp(target - current, -maximum_step, maximum_step);
}

}  // namespace xd_uav_detect
