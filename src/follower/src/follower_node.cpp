#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>

#include <XmlRpcValue.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <follower/ControllerCommand.h>
#include <follower/ControllerFeedback.h>
#include <follower/FollowerCommand.h>
#include <follower/FollowerStatus.h>
#include <follower/ManageControlLease.h>
#include <follower/SetMode.h>
#include <follower/control_guard.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/Altitude.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/VFR_HUD.h>
#include <mrs_msgs/VelocityReferenceStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_srvs/SetBool.h>
#include <tf/transform_datatypes.h>
#include <tracker/NormalizedError.h>
#include <visualization_msgs/Marker.h>

#include <pod_msgs/GimbalState.h>

namespace {
constexpr double kPi = 3.14159265358979323846;

double clamp(const double value, const double low, const double high) {
  return std::max(low, std::min(high, value));
}
double sign(const double value) {
  return value > 0.0 ? 1.0 : (value < 0.0 ? -1.0 : 0.0);
}

struct Gains {
  double kp, ki, kd;
};

Gains loadGains(const ros::NodeHandle& nh, const std::string& name,
                const Gains defaults) {
  XmlRpc::XmlRpcValue map;
  if (!nh.getParam(name, map) ||
      map.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
    return defaults;
  }
  const auto read = [&](const char* key, const double fallback) {
    if (!map.hasMember(key)) return fallback;
    try {
      return map[key].getType() == XmlRpc::XmlRpcValue::TypeInt
                 ? static_cast<double>(static_cast<int>(map[key]))
                 : static_cast<double>(map[key]);
    } catch (...) {
      return fallback;
    }
  };
  return {read("kp", defaults.kp), read("ki", defaults.ki),
          read("kd", defaults.kd)};
}

class Pid {
 public:
  void configure(const Gains gains, const double low, const double high,
                 const bool pom, const double anti_windup_gain) {
    gains_ = gains;
    low_ = low;
    high_ = high;
    pom_ = pom;
    anti_windup_gain_ = std::max(0.0, anti_windup_gain);
  }
  double update(const double input, double dt) {
    if (dt <= 0.0) dt = 1.0;
    const bool first_sample = first_;
    if (first_) {
      previous_input_ = input;
      first_ = false;
    }
    const double error = pom_ ? -previous_input_ : -input;
    if (first_sample) previous_error_ = error;
    const double p = pom_ ? -gains_.kp * (input - previous_input_)
                          : gains_.kp * error;
    const double candidate_integral = integral_ + gains_.ki * error * dt;
    const double d = first_sample ? 0.0 :
        (pom_ ? -gains_.kd * (input - previous_input_) / dt
              : gains_.kd * (error - previous_error_) / dt);
    const double raw_output = p + candidate_integral + d;
    const double output = clamp(raw_output, low_, high_);
    // Standard back-calculation: the saturated-minus-raw residual drives the
    // integrator back into its controllable range without changing PID gains.
    integral_ = candidate_integral +
                anti_windup_gain_ * (output - raw_output) * dt;
    previous_input_ = input;
    previous_error_ = error;
    previous_output_ = output;
    return output;
  }
  void reset() {
    integral_ = previous_input_ = previous_error_ = previous_output_ = 0.0;
    first_ = true;
  }

 private:
  Gains gains_{0.0, 0.0, 0.0};
  double low_{-1.0}, high_{1.0};
  bool pom_{false}, first_{true};
  double anti_windup_gain_{0.1};
  double integral_{0.0}, previous_input_{0.0}, previous_error_{0.0},
      previous_output_{0.0};
};

struct Output {
  double forward{0.0}, right{0.0}, down{0.0}, yaw{0.0}, raw_yaw{0.0};
  double roll_rate{0.0}, pitch_rate{0.0}, thrust{0.0};
  double pid_yaw{0.0}, pid_right{0.0}, pid_down{0.0};
  double error_x{0.0}, error_y{0.0}, loss_duration{0.0};
  double adaptive_down{0.0}, adaptive_forward{0.0};
  double vertical_rate_error{0.0}, smoothed_vertical_rate{0.0};
  double pitch_correction{0.0}, current_forward{0.0};
  double tracking_confidence{0.0};
  double relative_position_sigma{0.0}, relative_velocity_sigma{0.0};
  double uncertainty_scale{1.0};
  bool metric_state_active{false};
  bool uncertainty_limited{false};
  bool uncertainty_abort{false};
  double input_age_ms{0.0}, compute_time_ms{0.0};
  double tracker_capture_age_ms{0.0};
  double tracker_inference_latency_ms{0.0};
  double tracker_transport_latency_ms{0.0};
  bool latency_limited{false}, latency_abort{false};
  bool valid{false}, visible{false}, lost{false}, emergency{false}, grace{false};
  bool adaptive{false}, pitch_active{false}, yaw_smoothing{false};
  bool fw_loss_maneuver{false}, fw_stall_recovery{false};
  bool altitude_safe{true};
  std::string mode{"coordinated_turn"};
  std::string profile{"mc_velocity_chase"};
  std::string effective_profile{"mc_velocity_chase"};
  std::string previous_profile;
  std::string control_mode{"velocity_body"};
  bool profile_fallback_active{false};
  bool profile_transition_active{false};
  bool profile_supported{true};
  std::string profile_rejection_reason;
  std::string control_state{"DISABLED"};
  std::string invalid_reason{"follower is disabled"};
  std::string output_backend{"command_only"};
  bool control_authorized{false};
  bool platform_ready{false};
  bool input_received{false};
  bool capture_timestamp_valid{false};
  ros::Time source_capture_timestamp;
  uint32_t sequence{0};
};

class FollowerCore {
 public:
  explicit FollowerCore(const ros::NodeHandle& nh) {
    nh.param("follower_profile", profile_, std::string("mc_velocity_chase"));
    nh.param("vehicle_type", vehicle_type_, std::string(""));
    nh.param("lateral_guidance_mode", mode_, std::string("coordinated_turn"));
    nh.param("enable_auto_mode_switching", auto_switch_, false);
    nh.param("mode_switch_velocity", switch_velocity_, 3.0);
    nh.param("mode_switch_hysteresis", switch_hysteresis_, 0.5);
    nh.param("max_velocity_forward", max_forward_, 8.0);
    nh.param("max_velocity_lateral", max_lateral_, 3.0);
    nh.param("max_velocity_vertical", max_vertical_, 2.0);
    nh.param("max_velocity_magnitude", max_magnitude_, 15.0);
    nh.param("max_yaw_rate_deg_s", max_yaw_, 90.0);
    // Keep the compiled fallback aligned with the SITL configuration.  A real
    // flight launch should explicitly override this with its mission limit.
    nh.param("min_altitude", min_altitude_, 1.0);
    nh.param("max_altitude", max_altitude_, 120.0);
    nh.param("altitude_timeout_sec", altitude_timeout_, 0.5);
    nh.param("altitude_fail_safe", altitude_fail_safe_, true);
    nh.param("initial_forward_velocity", forward_, 0.0);
    nh.param("max_forward_velocity", target_forward_, 5.0);
    nh.param("forward_ramp_rate", ramp_rate_, 0.5);
    nh.param("ramp_down_on_target_loss", ramp_down_, true);
    nh.param("target_loss_stop_velocity", loss_forward_, 0.0);
    nh.param("forward_velocity_deadzone", velocity_deadzone_, 0.01);
    nh.param("target_loss_coord_threshold", loss_threshold_, 1.5);
    nh.param("target_loss_timeout", loss_timeout_, 3.0);
    nh.param("command_smoothing_enabled", smooth_velocity_, true);
    nh.param("smoothing_factor", smoothing_factor_, 0.3);
    nh.param("pid_proportional_on_measurement", pom_, false);
    nh.param("pid_anti_windup_gain", anti_windup_gain_, 0.1);
    nh.param("min_tracking_confidence", min_tracking_confidence_, 0.5);
    nh.param("target_confidence_hysteresis", confidence_hysteresis_, 0.08);
    nh.param("enable_profile_sensor_fallback", profile_sensor_fallback_, true);
    nh.param("enable_profile_transition_blending", transition_blending_, true);
    nh.param("profile_transition_duration_sec", transition_duration_, 0.5);
    nh.param("profile_switch_min_interval_sec", profile_switch_min_interval_, 0.0);
    confidence_hysteresis_ = clamp(confidence_hysteresis_, 0.0,
                                   min_tracking_confidence_);
    transition_duration_ = std::max(0.0, transition_duration_);
    profile_switch_min_interval_ = std::max(0.0, profile_switch_min_interval_);
    nh.param("yaw_smoothing_enabled", smooth_yaw_enabled_, true);
    nh.param("yaw_deadzone_deg_s", yaw_deadzone_, 0.5);
    nh.param("yaw_max_rate_change_deg_s2", yaw_acceleration_, 90.0);
    nh.param("yaw_feasibility_max_jerk_deg_s3", yaw_feasible_jerk_, 360.0);
    nh.param("yaw_smoothing_alpha", yaw_alpha_, 0.7);
    nh.param("yaw_speed_scaling_enabled", yaw_speed_scaling_, true);
    nh.param("yaw_min_speed_threshold", yaw_min_speed_, 0.5);
    nh.param("yaw_max_speed_threshold", yaw_max_speed_, 5.0);
    nh.param("yaw_low_speed_factor", yaw_low_factor_, 0.5);
    nh.param("feasibility_max_vel", feasible_velocity_, 8.0);
    nh.param("feasibility_max_acc", feasible_acceleration_, 5.0);
    nh.param("feasibility_max_jerk", feasible_jerk_, 20.0);
    nh.param("pitch_compensation_enabled", pitch_enabled_, false);
    nh.param("pitch_compensation_gain", pitch_gain_, 0.05);
    nh.param("pitch_smoothing_alpha", pitch_alpha_, 0.7);
    nh.param("pitch_min_velocity", pitch_min_velocity_, 1.0);
    nh.param("pitch_deadband", pitch_deadband_, 2.0);
    nh.param("pitch_max_angle", pitch_max_angle_, 45.0);
    nh.param("pitch_max_correction", pitch_max_correction_, 0.3);
    nh.param("adaptive_dive_climb_enabled", adaptive_enabled_, false);
    nh.param("adaptive_smoothing_alpha", adaptive_alpha_, 0.2);
    nh.param("adaptive_warmup_frames", adaptive_warmup_, 10);
    nh.param("adaptive_rate_threshold", adaptive_threshold_, 5.0);
    nh.param("adaptive_max_correction", adaptive_max_, 1.0);
    nh.param("adaptive_correction_gain", adaptive_gain_, 0.3);
    nh.param("adaptive_min_confidence", adaptive_confidence_, 0.6);
    nh.param("adaptive_fwd_coupling_enabled", adaptive_forward_enabled_, false);
    nh.param("adaptive_fwd_coupling_gain", adaptive_forward_gain_, 0.1);
    nh.param("pixel_to_rate_calibration", pixel_rate_, 0.05);
    nh.param("video_height_pixels", video_height_, 480.0);
    nh.param("ground_descent_velocity", ground_descent_velocity_, 0.25);
    nh.param("ground_min_descent_height", ground_min_descent_height_, 2.0);
    nh.param("ground_altitude_scaling_enabled", ground_altitude_scaling_, true);
    nh.param("ground_altitude_factor", ground_altitude_factor_, 0.005);
    nh.param("ground_base_adjustment_x", ground_base_adjustment_x_, 0.2);
    nh.param("ground_base_adjustment_y", ground_base_adjustment_y_, 0.2);
    nh.param("ground_min_error_scale", ground_min_error_scale_, 1.0);
    nh.param("ground_max_error_scale", ground_max_error_scale_, 1.5);
    nh.param("gm_vector_speed", gm_vector_speed_, 3.0);
    nh.param("enable_relative_state_control", relative_state_enabled_, true);
    nh.param("relative_target_distance_m", relative_target_distance_, 5.0);
    nh.param("relative_target_down_m", relative_target_down_, 0.0);
    nh.param("relative_position_kp", relative_position_kp_, 0.4);
    nh.param("relative_velocity_feedforward_gain",
             relative_velocity_ff_gain_, 0.8);
    nh.param("relative_max_correction", relative_max_correction_, 3.0);
    nh.param("uncertainty_control_enabled", uncertainty_control_enabled_, true);
    nh.param("uncertainty_nominal_sigma_m", uncertainty_nominal_sigma_m_, 0.75);
    nh.param("uncertainty_nominal_sigma_px", uncertainty_nominal_sigma_px_, 40.0);
    nh.param("uncertainty_slow_sigma_ratio", uncertainty_slow_sigma_ratio_, 1.0);
    nh.param("uncertainty_abort_sigma_ratio", uncertainty_abort_sigma_ratio_, 2.5);
    nh.param("uncertainty_min_control_scale", uncertainty_min_control_scale_, 0.25);
    nh.param("uncertainty_yaw_scale", uncertainty_yaw_scale_, 0.65);
    nh.param("reid_initial_control_scale", reid_initial_scale_, 0.35);
    nh.param("reid_recovery_duration_sec", reid_recovery_duration_, 0.6);
    nh.param("gimbal_mount_type", gimbal_mount_type_, std::string("HORIZONTAL"));
    nh.param("gimbal_mount_yaw_offset_deg", gimbal_yaw_offset_deg_, 0.0);
    nh.param("gimbal_mount_pitch_offset_deg", gimbal_pitch_offset_deg_, 0.0);
    nh.param("gimbal_mount_roll_offset_deg", gimbal_roll_offset_deg_, 0.0);
    nh.param("gimbal_invert_yaw", gimbal_invert_yaw_, false);
    nh.param("gimbal_invert_pitch", gimbal_invert_pitch_, false);
    nh.param("gimbal_invert_roll", gimbal_invert_roll_, false);
    nh.param("gimbal_angle_smoothing_alpha", gimbal_angle_alpha_, 0.7);
    nh.param("gimbal_angle_deadzone_deg", gimbal_angle_deadzone_deg_, 0.25);
    nh.param("gm_vector_enable_altitude_control", gm_vector_altitude_control_, true);
    nh.param("gm_chase_forward_mode", gm_chase_forward_mode_, std::string("CONSTANT"));
    nh.param("gm_chase_base_forward_speed", gm_chase_base_speed_, 2.0);
    nh.param("gm_chase_forward_acceleration", gm_chase_acceleration_, 2.0);
    nh.param("gm_chase_pitch_velocity_scaling", gm_chase_pitch_scaling_, 0.15);
    nh.param("gm_chase_pitch_deadzone_deg", gm_chase_pitch_deadzone_deg_, 2.0);
    nh.param("gm_chase_max_lateral_angle_deg", gm_chase_max_lateral_deg_, 90.0);
    nh.param("gm_chase_max_vertical_angle_deg", gm_chase_max_vertical_deg_, 90.0);
    nh.param("allow_attitude_rate_profiles", allow_attitude_rate_profiles_, false);
    nh.param("allow_fixed_wing_profile", allow_fixed_wing_profile_, false);
    nh.param("mc_hover_thrust", mc_hover_thrust_, 0.5);
    nh.param("mc_min_thrust", mc_min_thrust_, 0.1);
    nh.param("mc_max_thrust", mc_max_thrust_, 0.9);
    nh.param("mc_max_roll_rate_deg_s", mc_max_roll_rate_, 45.0);
    nh.param("mc_max_pitch_rate_deg_s", mc_max_pitch_rate_, 45.0);
    nh.param("mc_altitude_hold_enabled", mc_altitude_hold_enabled_, true);
    nh.param("mc_target_altitude_offset", mc_target_altitude_offset_, 0.0);
    nh.param("mc_altitude_thrust_min_correction", mc_altitude_thrust_min_, -0.3);
    nh.param("mc_altitude_thrust_max_correction", mc_altitude_thrust_max_, 0.3);
    nh.param("mc_thrust_compensation_enabled", mc_thrust_compensation_, true);
    nh.param("mc_thrust_compensation_gain", mc_thrust_compensation_gain_, 1.0);
    nh.param("mc_coordinated_roll_enabled", mc_coordinated_roll_, true);
    nh.param("mc_turn_coordination_gain", mc_turn_coordination_gain_, 1.0);
    nh.param("mc_max_bank_angle_deg", mc_max_bank_angle_, 35.0);
    nh.param("mc_yaw_error_gating_enabled", mc_yaw_gating_, true);
    nh.param("mc_yaw_error_gate", mc_yaw_gate_, 0.35);
    nh.param("fw_cruise_thrust", fw_cruise_thrust_, 0.6);
    nh.param("fw_max_roll_rate_deg_s", fw_max_roll_rate_, 35.0);
    nh.param("fw_max_pitch_rate_deg_s", fw_max_pitch_rate_, 25.0);
    nh.param("fw_min_airspeed", fw_min_airspeed_, 12.0);
    nh.param("fw_cruise_airspeed", fw_cruise_airspeed_, 18.0);
    nh.param("fw_max_airspeed", fw_max_airspeed_, 30.0);
    nh.param("fw_airspeed_timeout_sec", fw_airspeed_timeout_, 0.5);
    nh.param("fw_max_bank_angle_deg", fw_max_bank_angle_, 35.0);
    nh.param("fw_max_load_factor", fw_max_load_factor_, 2.5);
    nh.param("fw_l1_distance", fw_l1_distance_, 50.0);
    nh.param("fw_l1_damping", fw_l1_damping_, 0.75);
    nh.param("fw_l1_adaptive", fw_l1_adaptive_, true);
    nh.param("fw_l1_min_distance", fw_l1_min_distance_, 20.0);
    nh.param("fw_l1_max_distance", fw_l1_max_distance_, 100.0);
    nh.param("fw_l1_lateral_scale", fw_l1_lateral_scale_, 50.0);
    nh.param("fw_tecs_enabled", fw_tecs_enabled_, true);
    nh.param("fw_tecs_time_constant", fw_tecs_time_constant_, 5.0);
    nh.param("fw_tecs_pitch_damping", fw_tecs_pitch_damping_, 0.15);
    nh.param("fw_tecs_throttle_damping", fw_tecs_throttle_damping_, 0.05);
    nh.param("fw_tecs_spe_weight", fw_tecs_spe_weight_, 1.0);
    nh.param("fw_tecs_altitude_scale", fw_tecs_altitude_scale_, 20.0);
    nh.param("fw_tecs_max_integral", fw_tecs_max_integral_, 50.0);
    nh.param("fw_min_thrust", fw_min_thrust_, 0.2);
    nh.param("fw_max_thrust", fw_max_thrust_, 1.0);
    nh.param("fw_thrust_slew_rate", fw_thrust_slew_rate_, 0.5);
    nh.param("fw_stall_protection_enabled", fw_stall_protection_, true);
    nh.param("fw_stall_margin_buffer", fw_stall_margin_, 3.0);
    nh.param("fw_stall_recovery_pitch_rate_deg_s", fw_stall_recovery_pitch_, -5.0);
    nh.param("fw_stall_recovery_thrust", fw_stall_recovery_thrust_, 1.0);
    nh.param("fw_orbit_on_target_loss", fw_orbit_on_loss_, true);
    nh.param("fw_orbit_radius", fw_orbit_radius_, 100.0);
    nh.param("fw_orbit_direction", fw_orbit_direction_, 1.0);
    yaw_pid_.configure(loadGains(nh, "pid_yaw", {2.0, 0.05, 0.1}),
                       -max_yaw_ * kPi / 180.0, max_yaw_ * kPi / 180.0, pom_, anti_windup_gain_);
    right_pid_.configure(loadGains(nh, "pid_right", {1.5, 0.02, 0.05}),
                         -max_lateral_, max_lateral_, pom_, anti_windup_gain_);
    down_pid_.configure(loadGains(nh, "pid_down", {1.0, 0.03, 0.05}),
                        -max_vertical_, max_vertical_, pom_, anti_windup_gain_);
    ground_forward_pid_.configure(
        loadGains(nh, "pid_ground_forward", {2.0, 0.03, 0.05}),
        0.0, max_forward_, pom_, anti_windup_gain_);
    mc_altitude_pid_.configure(
        loadGains(nh, "pid_mc_attitude_altitude", {0.08, 0.01, 0.03}),
        mc_altitude_thrust_min_, mc_altitude_thrust_max_, false,
        anti_windup_gain_);
    mc_roll_pid_.configure(
        loadGains(nh, "pid_mc_coordinated_roll", {2.0, 0.02, 0.05}),
        -mc_max_roll_rate_, mc_max_roll_rate_, false, anti_windup_gain_);
    fw_bank_pid_.configure(
        loadGains(nh, "pid_fw_bank", {2.0, 0.01, 0.05}),
        -fw_max_roll_rate_, fw_max_roll_rate_, false, anti_windup_gain_);
    if (!isKnownProfile(profile_)) {
      ROS_WARN("[FollowerCore] Unknown follower_profile '%s'; using mc_velocity_chase",
               profile_.c_str());
      profile_ = "mc_velocity_chase";
    }
  }

  void setPitch(const double pitch) {
    if (!std::isfinite(pitch)) return;
    pitch_ = clamp(pitch, -pitch_max_angle_, pitch_max_angle_);
    smoothed_pitch_ = pitch_initialized_
                          ? pitch_alpha_ * pitch_ +
                                (1.0 - pitch_alpha_) * smoothed_pitch_
                          : pitch_;
    pitch_initialized_ = true;
  }
  void setAttitude(const double roll_deg, const double pitch_deg,
                   const double yaw_deg) {
    if (std::isfinite(roll_deg)) roll_deg_ = roll_deg;
    if (std::isfinite(yaw_deg)) yaw_deg_ = yaw_deg;
    setPitch(pitch_deg);
    attitude_valid_ = std::isfinite(roll_deg) && std::isfinite(pitch_deg) &&
                      std::isfinite(yaw_deg);
  }
  void setGroundSpeed(const double speed) {
    if (std::isfinite(speed) && speed >= 0.0) {
      ground_speed_ = speed;
      ground_speed_valid_ = true;
    }
  }
  void setAirspeed(const double speed, const double now) {
    if (std::isfinite(speed) && speed >= 0.0) {
      airspeed_ = speed;
      airspeed_time_ = now;
      airspeed_valid_ = true;
    }
  }
  void setAltitude(const double altitude, const double now) {
    altitude_ = altitude;
    altitude_valid_ = std::isfinite(altitude);
    if (altitude_valid_) altitude_time_ = now;
  }
  void setLocalAltitude(const double altitude, const double now) {
    local_altitude_ = altitude;
    local_altitude_valid_ = std::isfinite(altitude);
    if (local_altitude_valid_) local_altitude_time_ = now;
  }
  bool setMode(const std::string& mode, const double now) {
    if (mode != "coordinated_turn" && mode != "sideslip") return false;
    if (mode != mode_) {
      mode_ = mode;
      last_switch_ = now;
      resetControllers();
      yaw_feasibility_time_ = 0.0;
      previous_feasible_yaw_ = previous_yaw_acceleration_ = 0.0;
      yaw_feasible_initialized_ = false;
      gimbal_angles_initialized_ = false;
      gm_vector_current_speed_ = gm_chase_current_speed_ = 0.0;
      mc_altitude_target_initialized_ = false;
      resetFixedWingState();
    }
    return true;
  }
  bool setProfile(const std::string& profile, const double now,
                  std::string* rejection = nullptr) {
    if (!isKnownProfile(profile)) {
      if (rejection) *rejection = "unknown profile";
      return false;
    }
    if (profile == "fw_attitude_rate" && !allow_fixed_wing_profile_) {
      if (rejection) {
        *rejection = "fixed-wing profile disabled; authorize a fixed-wing attitude adapter first";
      }
      return false;
    }
    if (profile == "fw_attitude_rate" && !vehicle_type_.empty() &&
        vehicle_type_ != "fixedwing") {
      if (rejection) {
        *rejection = "fixed-wing profile rejected for vehicle_type='" +
                     vehicle_type_ + "'";
      }
      return false;
    }
    if (profile == "mc_attitude_rate" && !allow_attitude_rate_profiles_) {
      if (rejection) {
        *rejection = "attitude-rate profiles are disabled; enable a dedicated PX4 attitude adapter first";
      }
      return false;
    }
    if (profile == "mc_attitude_rate" && !vehicle_type_.empty() &&
        vehicle_type_ != "multirotor") {
      if (rejection) {
        *rejection = "multirotor attitude profile rejected for vehicle_type='" +
                     vehicle_type_ + "'";
      }
      return false;
    }
    if (profile != profile_ && profile_switch_min_interval_ > 0.0 &&
        have_profile_switch_ &&
        now - last_profile_switch_ < profile_switch_min_interval_) {
      if (rejection) {
        const double remaining = profile_switch_min_interval_ -
                                  std::max(0.0, now - last_profile_switch_);
        *rejection = "profile switch rate limited; wait " +
                     std::to_string(remaining) + " s";
      }
      return false;
    }
    if (profile != profile_) {
      const std::string old_profile = profile_;
      transition_active_ = transition_blending_ && transition_duration_ > 0.0 &&
          have_last_output_ && last_output_.valid &&
          !isAttitudeProfile(old_profile) && !isAttitudeProfile(profile);
      if (transition_active_) {
        transition_from_ = last_output_;
        transition_from_profile_ = old_profile;
        transition_start_ = now;
      } else {
        transition_from_profile_.clear();
      }
      profile_ = profile;
      last_switch_ = now;
      last_profile_switch_ = now;
      have_profile_switch_ = true;
      resetControllers();
      forward_ = 0.0;
      velocity_history_.clear();
      acceleration_history_.clear();
      feasibility_time_ = 0.0;
      yaw_feasibility_time_ = 0.0;
      previous_feasible_yaw_ = previous_yaw_acceleration_ = 0.0;
      yaw_feasible_initialized_ = false;
      gimbal_angles_initialized_ = false;
      gm_vector_current_speed_ = gm_chase_current_speed_ = 0.0;
      mc_altitude_target_initialized_ = false;
      resetFixedWingState();
    }
    return true;
  }
  void emergency(const bool active) {
    emergency_ = active;
    if (active) {
      resetControllers();
      forward_ = 0.0;
    }
  }
  void reset() {
    resetControllers();
    forward_ = 0.0;
    // Emergency state is deliberately sticky. It can only be released by the
    // dedicated emergency_stop service, never by stop/start/profile reset.
    lost_ = false;
    loss_start_ = loss_duration_ = 0.0;
    sequence_ = 0;
    velocity_history_.clear();
    acceleration_history_.clear();
    adaptive_history_.clear();
    adaptive_count_ = 0;
    adaptive_rate_ = 0.0;
    feasibility_time_ = 0.0;
    yaw_feasibility_time_ = 0.0;
    yaw_feasible_initialized_ = false;
    gimbal_angles_initialized_ = false;
    gm_vector_current_speed_ = 0.0;
    gm_chase_current_speed_ = 0.0;
    reid_recovery_active_ = false;
    reid_recovery_start_ = 0.0;
    mc_altitude_target_initialized_ = false;
    resetFixedWingState();
    target_confidence_locked_ = false;
    transition_active_ = false;
    transition_from_profile_.clear();
    have_last_output_ = false;
  }

  Output compute(const double ex, const double ey, const double size_error,
                 const double los_yaw_rad, const double los_pitch_rad,
                 const bool angular_error_valid,
                 const bool valid,
                 const double confidence, const bool range_valid,
                 const std::array<double, 3>& relative_position,
                 const std::array<double, 3>& relative_velocity,
                 const std::array<double, 9>& position_covariance,
                 const std::array<double, 9>& velocity_covariance,
                 const std::array<double, 4>& tracker_state_covariance,
                 const std::string& association_method,
                 const std::string& fusion_status,
                 const double now, double dt) {
    if (dt <= 0.0 || dt > 1.0) dt = 1.0 / 30.0;
    Output out;
    out.sequence = ++sequence_;
    out.error_x = ex;
    out.error_y = ey;
    out.tracking_confidence = clamp(confidence, 0.0, 1.0);
    const bool metric_state_valid = relative_state_enabled_ && range_valid &&
        std::all_of(relative_position.begin(), relative_position.end(),
                    [](const double value) { return std::isfinite(value); }) &&
        std::all_of(relative_velocity.begin(), relative_velocity.end(),
                    [](const double value) { return std::isfinite(value); });
    out.metric_state_active = metric_state_valid;
    if (metric_state_valid) {
      double position_variance = 0.0, velocity_variance = 0.0;
      for (int i : {0, 4, 8}) {
        position_variance = std::max(position_variance,
                                     std::max(0.0, position_covariance[i]));
        velocity_variance = std::max(velocity_variance,
                                     std::max(0.0, velocity_covariance[i]));
      }
      out.relative_position_sigma = std::sqrt(position_variance);
      out.relative_velocity_sigma = std::sqrt(velocity_variance);
    }
    const bool predicted_sample = association_method == "predicted" ||
        association_method == "lost" || fusion_status == "PREDICTED" ||
        fusion_status == "LOST";
    double state_sigma = 0.0;
    if (metric_state_valid) {
      state_sigma = out.relative_position_sigma;
    } else {
      state_sigma = std::sqrt(std::max({0.0,
          static_cast<double>(tracker_state_covariance[0]),
          static_cast<double>(tracker_state_covariance[1])}));
    }
    const double sigma_reference = metric_state_valid
        ? std::max(1e-3, uncertainty_nominal_sigma_m_)
        : std::max(1e-3, uncertainty_nominal_sigma_px_);
    const double sigma_ratio = state_sigma / sigma_reference;
    const bool uncertainty_abort = uncertainty_control_enabled_ &&
        sigma_ratio >= uncertainty_abort_sigma_ratio_;
    double uncertainty_scale = 1.0;
    if (uncertainty_control_enabled_ && sigma_ratio > uncertainty_slow_sigma_ratio_) {
      uncertainty_scale = clamp(1.0 / std::max(1.0, sigma_ratio),
                                uncertainty_min_control_scale_, 1.0);
    }
    const bool appearance_sample = association_method == "appearance";
    if (appearance_sample) {
      if (!reid_recovery_active_) {
        reid_recovery_active_ = true;
        reid_recovery_start_ = now;
      }
      const double progress = clamp(
          (now - reid_recovery_start_) /
              std::max(1e-3, reid_recovery_duration_), 0.0, 1.0);
      uncertainty_scale = std::min(
          uncertainty_scale,
          reid_initial_scale_ + progress * (1.0 - reid_initial_scale_));
    } else if (association_method == "id" || association_method == "spatial") {
      reid_recovery_active_ = false;
    }
    out.uncertainty_scale = uncertainty_scale;
    out.uncertainty_limited = uncertainty_scale < 0.999 || uncertainty_abort;
    out.uncertainty_abort = uncertainty_abort;
    out.profile = profile_;
    std::string effective_profile = profile_;
    const bool normalized_observation_valid = valid && std::isfinite(ex) &&
        std::isfinite(ey) && std::abs(ex) <= loss_threshold_ &&
        std::abs(ey) <= loss_threshold_;
    if (profile_sensor_fallback_ && normalized_observation_valid &&
        !angular_error_valid) {
      if (profile_ == "gm_velocity_chase") {
        effective_profile = "mc_velocity_chase";
      } else if (profile_ == "gm_velocity_vector") {
        effective_profile = "mc_velocity_distance";
      }
    }
    out.effective_profile = effective_profile;
    out.profile_fallback_active = effective_profile != profile_;
    out.control_mode = isAttitudeProfile(profile_) ? "attitude_rate" : "velocity_body";
    const bool vehicle_profile_supported = vehicle_type_.empty() ||
        (profile_ != "fw_attitude_rate" || vehicle_type_ == "fixedwing") &&
        (profile_ != "mc_attitude_rate" || vehicle_type_ == "multirotor");
    out.profile_supported = vehicle_profile_supported &&
        (profile_ == "fw_attitude_rate" ? allow_fixed_wing_profile_ :
        (profile_ == "mc_attitude_rate" ? allow_attitude_rate_profiles_ : true));
    if (!out.profile_supported) {
      out.profile_rejection_reason = vehicle_profile_supported
          ? "attitude-rate output disabled by capability gate"
          : "profile rejected by follower vehicle_type capability gate";
    }
    if (profile_ == "mc_attitude_rate") out.thrust = mc_hover_thrust_;
    if (profile_ == "fw_attitude_rate") out.thrust = fw_cruise_thrust_;
    out.emergency = emergency_;
    const bool requires_angles = effective_profile == "gm_velocity_chase" ||
                                 effective_profile == "gm_velocity_vector";
    const bool uses_mount_angles = requires_angles ||
                                   effective_profile == "mc_velocity_ground";
    const double confidence_threshold = target_confidence_locked_
        ? std::max(0.0, min_tracking_confidence_ - confidence_hysteresis_)
        : min_tracking_confidence_;
    bool target_valid =
        normalized_observation_valid && confidence >= confidence_threshold &&
        (!requires_angles || angular_error_valid) &&
        std::isfinite(ex) && std::isfinite(ey) && !predicted_sample &&
        !uncertainty_abort;
    target_confidence_locked_ = target_valid;
    if (target_valid) {
      lost_ = false;
      loss_start_ = loss_duration_ = 0.0;
    } else if (!lost_) {
      lost_ = true;
      loss_start_ = now;
    } else {
      loss_duration_ = now - loss_start_;
    }
    out.visible = target_valid;
    out.lost = lost_;
    out.loss_duration = loss_duration_;
    out.grace = lost_ && loss_duration_ <= loss_timeout_;

    const double desired =
        emergency_ ? 0.0
                   : (lost_ && ramp_down_ ? loss_forward_ : target_forward_);
    const double difference = desired - forward_;
    forward_ += std::abs(difference) < velocity_deadzone_
                    ? difference
                    : clamp(difference, -ramp_rate_ * dt, ramp_rate_ * dt);
    forward_ = clamp(forward_, 0.0, target_forward_);
    out.current_forward = forward_;
    if (auto_switch_ && now - last_switch_ >= 2.0) {
      if (mode_ == "sideslip" &&
          forward_ >= switch_velocity_ + switch_hysteresis_) {
        setMode("coordinated_turn", now);
      } else if (mode_ != "sideslip" &&
                 forward_ <= switch_velocity_ - switch_hysteresis_) {
        setMode("sideslip", now);
      }
    }
    out.mode = mode_;

    double corrected_y = ey;
    if (pitch_enabled_ && pitch_initialized_ &&
        std::abs(smoothed_pitch_) >= pitch_deadband_ &&
        std::abs(forward_) >= pitch_min_velocity_) {
      out.pitch_correction =
          clamp(-pitch_gain_ * smoothed_pitch_ * forward_,
                -pitch_max_correction_, pitch_max_correction_);
      out.pitch_active = true;
      corrected_y -= out.pitch_correction;
    }

    double right = 0.0, down = 0.0, yaw = 0.0;
    double forward = forward_;
    double gimbal_yaw = 0.0, gimbal_pitch = 0.0, gimbal_roll = 0.0;
    if (target_valid && angular_error_valid && uses_mount_angles) {
      filteredGimbalAngles(los_yaw_rad, los_pitch_rad,
                           gimbal_yaw, gimbal_pitch, gimbal_roll);
    }
    const auto yaw_control = [&](const double horizontal_error) {
      out.pid_yaw = yaw_pid_.update(-horizontal_error, dt);
      out.raw_yaw = out.pid_yaw * 180.0 / kPi;
      out.yaw_smoothing = smooth_yaw_enabled_;
      return smoothYaw(out.raw_yaw, dt, forward_);
    };
    const auto right_control = [&](const double horizontal_error) {
      out.pid_right = right_pid_.update(-horizontal_error, dt);
      double value = out.pid_right;
      if (smooth_velocity_) {
        smoothed_right_ = smoothing_factor_ * smoothed_right_ +
                          (1.0 - smoothing_factor_) * value;
        value = smoothed_right_;
      }
      return value;
    };
    const auto down_control = [&](const double vertical_error) {
      out.pid_down = down_pid_.update(-vertical_error, dt);
      double value = out.pid_down;
      if (smooth_velocity_) {
        smoothed_down_ = smoothing_factor_ * smoothed_down_ +
                         (1.0 - smoothing_factor_) * value;
        value = smoothed_down_;
      }
      return value;
    };
    if (!target_valid && !isAttitudeProfile(effective_profile)) {
      resetControllers();
    } else if (effective_profile == "mc_velocity_ground") {
      // Prefer the calibrated camera LOS when tracker angular output exists;
      // normalized pixels remain a backwards-compatible fallback.
      double ground_x = ex;
      double ground_y = corrected_y;
      if (angular_error_valid) {
        gimbalControlErrors(gimbal_yaw, gimbal_pitch, gimbal_roll,
                            ground_x, ground_y);
      }
      const double active_altitude = altitude_valid_ ? altitude_ : local_altitude_;
      if (ground_altitude_scaling_ &&
          (altitude_valid_ || local_altitude_valid_) && active_altitude >= 0.0) {
        const double scale_x = clamp(
            1.0 + ground_base_adjustment_x_ /
                      (1.0 + ground_altitude_factor_ * active_altitude),
            ground_min_error_scale_, ground_max_error_scale_);
        const double scale_y = clamp(
            1.0 + ground_base_adjustment_y_ /
                      (1.0 + ground_altitude_factor_ * active_altitude),
            ground_min_error_scale_, ground_max_error_scale_);
        ground_x *= scale_x;
        ground_y *= scale_y;
      }
      out.pid_right = right_pid_.update(-ground_x, dt);
      right = out.pid_right;
      out.pid_down = 0.0;
      forward = ground_forward_pid_.update(ground_y, dt);
      if (smooth_velocity_) {
        smoothed_right_ = smoothing_factor_ * smoothed_right_ +
                          (1.0 - smoothing_factor_) * right;
        right = smoothed_right_;
      }
      down = active_altitude > ground_min_descent_height_
                 ? ground_descent_velocity_ : 0.0;
      yaw = 0.0;
    } else if (effective_profile == "mc_velocity_distance") {
      forward = 0.0;
      right = right_control(ex);
      down = down_control(corrected_y);
      yaw = yaw_control(ex);
    } else if (effective_profile == "mc_velocity_position") {
      forward = 0.0;
      right = 0.0;
      down = down_control(corrected_y);
      yaw = yaw_control(ex);
    } else if (effective_profile == "mc_velocity_chase") {
      down = down_control(corrected_y);
      if (mode_ == "coordinated_turn") yaw = yaw_control(ex);
      else right = right_control(ex);
    } else if (effective_profile == "gm_velocity_chase") {
      double lateral_error = 0.0, vertical_error = 0.0;
      gimbalControlErrors(gimbal_yaw, gimbal_pitch, gimbal_roll,
                          lateral_error, vertical_error);
      forward = gmChaseForwardVelocity(gimbal_pitch, dt);
      down = down_control(vertical_error);
      if (mode_ == "coordinated_turn") yaw = yaw_control(lateral_error);
      else right = right_control(lateral_error);
    } else if (effective_profile == "gm_velocity_vector") {
      gm_vector_current_speed_ += clamp(
          gm_vector_speed_ - gm_vector_current_speed_, -ramp_rate_ * dt,
          ramp_rate_ * dt);
      double unit_forward = 0.0, unit_right = 0.0, unit_down = 0.0;
      gimbalToBodyVector(gimbal_yaw, gimbal_pitch, gimbal_roll,
                         unit_forward, unit_right, unit_down);
      forward = std::max(0.0, gm_vector_current_speed_ * unit_forward);
      right = gm_vector_current_speed_ * unit_right;
      down = gm_vector_altitude_control_
                 ? gm_vector_current_speed_ * unit_down : 0.0;
      if (mode_ == "coordinated_turn") {
        // Use the true filtered line-of-sight yaw, converted only for the PID
        // input contract; vector geometry itself remains in radians.
        out.pid_yaw = yaw_pid_.update(-clamp(
            std::tan(gimbal_yaw) / std::max(1e-6, std::tan(0.5 * kPi / 3.0)),
            -1.0, 1.0), dt);
        out.raw_yaw = out.pid_yaw * 180.0 / kPi;
        yaw = smoothYaw(out.raw_yaw, dt, forward);
        out.yaw_smoothing = smooth_yaw_enabled_;
        right = 0.0;
      }
    } else if (effective_profile == "mc_attitude_rate") {
      forward = right = down = 0.0;
      out.pitch_rate = clamp(-corrected_y * mc_max_pitch_rate_, -mc_max_pitch_rate_, mc_max_pitch_rate_);
      yaw = yaw_control(ex);
      if (mc_yaw_gating_ && std::abs(ex) > mc_yaw_gate_) {
        out.pitch_rate = 0.0;
      }
      if (mc_coordinated_roll_) {
        const double target_bank = clamp(
            std::atan((yaw * kPi / 180.0) *
                      std::max(ground_speed_, 1.0) / 9.81) * 180.0 / kPi,
            -mc_max_bank_angle_, mc_max_bank_angle_);
        out.roll_rate = mc_roll_pid_.update(
            (roll_deg_ - target_bank) * mc_turn_coordination_gain_, dt);
      }
    } else if (effective_profile == "fw_attitude_rate") {
      forward = right = down = 0.0;
      const double speed = currentFixedWingAirspeed(now);
      yaw = fwL1Guidance(ex, speed, dt);
      const double target_bank = fwCoordinatedBank(yaw, speed);
      out.roll_rate = fw_bank_pid_.update(roll_deg_ - target_bank, dt);
      fwTecs(-corrected_y * fw_tecs_altitude_scale_, speed, dt,
             out.pitch_rate, out.thrust);
    }
    if (metric_state_valid &&
        (effective_profile == "mc_velocity_chase" ||
         effective_profile == "gm_velocity_chase")) {
      // Body-frame metric state takes precedence over bbox-size for chase
      // distance.  Relative velocity supplies feed-forward, while position
      // error closes the formation offset; safety/feasibility remains later.
      const double forward_error = relative_target_distance_ -
                                   relative_position[0];
      const double right_error = -relative_position[1];
      const double down_error = relative_target_down_ - relative_position[2];
      forward = clamp(relative_velocity_ff_gain_ * relative_velocity[0] -
                          relative_position_kp_ * forward_error,
                      0.0, relative_max_correction_);
      right = clamp(relative_velocity_ff_gain_ * relative_velocity[1] -
                        relative_position_kp_ * right_error,
                    -relative_max_correction_, relative_max_correction_);
      down = clamp(relative_velocity_ff_gain_ * relative_velocity[2] -
                       relative_position_kp_ * down_error,
                   -relative_max_correction_, relative_max_correction_);
    }
    if (target_valid && uncertainty_scale < 0.999 &&
        !isAttitudeProfile(effective_profile)) {
      forward *= uncertainty_scale;
      right *= uncertainty_scale;
      down *= uncertainty_scale;
      yaw *= uncertainty_yaw_scale_ * uncertainty_scale;
    }
    if (uncertainty_abort) {
      // High covariance is a fail-closed condition. Do not let the ordinary
      // target-loss grace ramp keep stale forward motion alive.
      forward_ = 0.0;
      forward = right = down = yaw = 0.0;
      out.grace = false;
    }
    if (profile_ == "mc_attitude_rate") {
      out.thrust = mcAttitudeThrust(dt);
      if (!target_valid) {
        // Explicit target-loss HOVER state: no stale tracking rates.  Thrust
        // remains under altitude hold during the bounded command grace period.
        out.roll_rate = out.pitch_rate = 0.0;
        yaw = 0.0;
      }
    }
    if (profile_ == "fw_attitude_rate") {
      const bool airspeed_fresh = airspeed_valid_ &&
          now - airspeed_time_ <= fw_airspeed_timeout_;
      const double speed = currentFixedWingAirspeed(now);
      if (fw_stall_protection_ && airspeed_fresh &&
          speed < fw_min_airspeed_) {
        out.roll_rate = 0.0;
        out.pitch_rate = clamp(fw_stall_recovery_pitch_,
                               -fw_max_pitch_rate_, fw_max_pitch_rate_);
        yaw = 0.0;
        out.thrust = clamp(fw_stall_recovery_thrust_,
                           fw_min_thrust_, fw_max_thrust_);
        out.fw_stall_recovery = true;
      } else if (!target_valid) {
        out.roll_rate = out.pitch_rate = yaw = 0.0;
        out.thrust = fw_cruise_thrust_;
        if (fw_orbit_on_loss_ && loss_duration_ > loss_timeout_) {
          const double direction = fw_orbit_direction_ < 0.0 ? -1.0 : 1.0;
          yaw = direction * clamp(speed / std::max(10.0, fw_orbit_radius_) *
                                       180.0 / kPi,
                                   0.0, max_yaw_);
          const double orbit_bank = fwCoordinatedBank(yaw, speed);
          out.roll_rate = fw_bank_pid_.update(roll_deg_ - orbit_bank, dt);
          out.fw_loss_maneuver = true;
        }
      }
    }
    if (target_valid && adaptive_enabled_ &&
        confidence >= adaptive_confidence_) {
      adaptive(forward, down, ey, now, out);
    }
    // Safety/feasibility is applied after any external override in finalize().
    const bool altitude_known = altitude_valid_ || local_altitude_valid_;
    const double active_altitude = altitude_valid_ ? altitude_ : local_altitude_;
    out.altitude_safe = !altitude_known ||
                        (active_altitude >= min_altitude_ &&
                         active_altitude <= max_altitude_);
    if (!out.altitude_safe) ++altitude_violations_;
    if (emergency_) {
      forward = right = down = yaw = 0.0;
      out.roll_rate = out.pitch_rate = 0.0;
    }
    // During a bounded loss grace interval only a decelerating forward command
    // is retained; lateral, vertical and yaw control never use stale pixels.
    if (!target_valid && !out.fw_loss_maneuver) right = down = yaw = 0.0;
    out.forward = forward;
    out.right = right;
    out.down = down;
    out.yaw = yaw;
    out.valid = (target_valid || out.grace || out.fw_loss_maneuver ||
                 out.fw_stall_recovery) &&
                !emergency_ && out.profile_supported;
    return out;
  }

  void constrain(Output& out, const follower::ControllerCommand& command) {
    const bool finite_override = std::isfinite(command.override_velocity_forward) &&
        std::isfinite(command.override_velocity_right) &&
        std::isfinite(command.override_velocity_down) &&
        std::isfinite(command.override_yaw_rate_deg_s);
    if (command.override_active && finite_override) {
      out.forward = command.override_velocity_forward;
      out.right = command.override_velocity_right;
      out.down = command.override_velocity_down;
      out.yaw = command.override_yaw_rate_deg_s;
      out.valid = !emergency_ && out.profile_supported;
    }
    if (command.has_velocity_limits &&
        std::all_of(command.velocity_limits.begin(), command.velocity_limits.end(),
                    [](const float value) { return std::isfinite(value); }) &&
        command.velocity_limits[0] <= command.velocity_limits[1] &&
        command.velocity_limits[2] <= command.velocity_limits[3] &&
        command.velocity_limits[4] <= command.velocity_limits[5]) {
      out.forward = clamp(out.forward, command.velocity_limits[0],
                          command.velocity_limits[1]);
      out.right = clamp(out.right, command.velocity_limits[2],
                        command.velocity_limits[3]);
      out.down = clamp(out.down, command.velocity_limits[4],
                       command.velocity_limits[5]);
    }
  }

  // Apply a monotonic authority reduction when the tracker sample is old but
  // still inside the control timeout.  Scaling only the translational and
  // body-rate terms preserves the attitude-profile thrust/altitude hold while
  // preventing stale pixels from commanding a full-speed maneuver.
  void limitForTrackerLatency(Output& out, const double scale) const {
    const double s = clamp(scale, 0.0, 1.0);
    out.forward *= s;
    out.right *= s;
    out.down *= s;
    out.yaw *= s;
    out.roll_rate *= s;
    out.pitch_rate *= s;
    out.pid_yaw *= s;
    out.pid_right *= s;
    out.pid_down *= s;
  }

  void finalize(Output& out, const double now) {
    const bool altitude_fresh =
        (altitude_valid_ && now - altitude_time_ <= altitude_timeout_) ||
        (local_altitude_valid_ && now - local_altitude_time_ <= altitude_timeout_);
    if (altitude_fail_safe_ && !altitude_fresh) {
      out.forward = out.right = out.down = out.yaw = 0.0;
      out.roll_rate = out.pitch_rate = 0.0;
      out.valid = false;
      out.altitude_safe = false;
      transition_active_ = false;
      last_output_ = out;
      have_last_output_ = true;
      return;
    }
    if (out.emergency) out.forward = out.right = out.down = out.yaw = 0.0;
    if (out.uncertainty_abort) {
      out.forward = out.right = out.down = out.yaw = 0.0;
      out.roll_rate = out.pitch_rate = 0.0;
      out.valid = false;
      velocity_history_.clear();
      acceleration_history_.clear();
      last_output_ = out;
      have_last_output_ = true;
      return;
    }
    out.forward = clamp(out.forward, 0.0, max_forward_);
    out.right = clamp(out.right, -max_lateral_, max_lateral_);
    out.down = clamp(out.down, -max_vertical_, max_vertical_);
    out.yaw = clamp(out.yaw, -max_yaw_, max_yaw_);
    applyProfileTransition(out, now);
    magnitude(out.forward, out.right, out.down, max_magnitude_);
    feasible(out.forward, out.right, out.down, now);
    feasibleYaw(out.yaw, now);
    if (out.fw_stall_recovery) out.yaw = 0.0;
    if (isAttitudeProfile(out.profile) && !out.visible &&
        !out.fw_loss_maneuver && !out.fw_stall_recovery) {
      out.roll_rate = out.pitch_rate = out.yaw = 0.0;
    }
    const double active_altitude = altitude_valid_ ? altitude_ : local_altitude_;
    if (altitude_fresh) {
      out.altitude_safe = active_altitude >= min_altitude_ &&
                          active_altitude <= max_altitude_;
      if (active_altitude <= min_altitude_) out.down = std::min(0.0, out.down);
      if (active_altitude >= max_altitude_) out.down = std::max(0.0, out.down);
    }
    last_output_ = out;
    have_last_output_ = true;
  }

  double targetForward() const { return target_forward_; }
  const std::string& profile() const { return profile_; }
  double altitude() const { return altitude_valid_ ? altitude_ : (local_altitude_valid_ ? local_altitude_ : 0.0); }
  double pitch() const { return pitch_; }
  int altitudeViolations() const { return altitude_violations_; }
  bool emergencyActive() const { return emergency_; }

 private:
  static bool isKnownProfile(const std::string& profile) {
    static const std::array<const char*, 8> profiles{{
        "mc_velocity_ground", "mc_velocity_distance",
        "mc_velocity_position", "mc_velocity_chase",
        "gm_velocity_chase", "gm_velocity_vector",
        "mc_attitude_rate", "fw_attitude_rate"}};
    return std::any_of(profiles.begin(), profiles.end(),
                       [&](const char* value) { return profile == value; });
  }
  static bool isAttitudeProfile(const std::string& profile) {
    return profile == "mc_attitude_rate" || profile == "fw_attitude_rate";
  }
  void applyProfileTransition(Output& out, const double now) {
    if (!transition_active_) return;
    out.previous_profile = transition_from_profile_;
    if (!out.valid || !out.visible || out.emergency ||
        isAttitudeProfile(out.profile)) {
      transition_active_ = false;
      return;
    }
    const double linear = clamp(
        (now - transition_start_) / std::max(1e-3, transition_duration_),
        0.0, 1.0);
    const double alpha = linear * linear * (3.0 - 2.0 * linear);
    const double previous = 1.0 - alpha;
    out.forward = previous * transition_from_.forward + alpha * out.forward;
    out.right = previous * transition_from_.right + alpha * out.right;
    out.down = previous * transition_from_.down + alpha * out.down;
    out.yaw = previous * transition_from_.yaw + alpha * out.yaw;
    out.current_forward = previous * transition_from_.current_forward +
                          alpha * out.current_forward;
    out.profile_transition_active = linear < 1.0;
    if (linear >= 1.0) transition_active_ = false;
  }
  void filteredGimbalAngles(const double yaw, const double pitch,
                            double& filtered_yaw, double& filtered_pitch,
                            double& filtered_roll) {
    const double alpha = clamp(gimbal_angle_alpha_, 0.0, 1.0);
    if (!gimbal_angles_initialized_) {
      filtered_gimbal_yaw_ = yaw;
      filtered_gimbal_pitch_ = pitch;
      filtered_gimbal_roll_ = 0.0;
      gimbal_angles_initialized_ = true;
    } else {
      filtered_gimbal_yaw_ = alpha * filtered_gimbal_yaw_ + (1.0 - alpha) * yaw;
      filtered_gimbal_pitch_ = alpha * filtered_gimbal_pitch_ + (1.0 - alpha) * pitch;
      filtered_gimbal_roll_ = alpha * filtered_gimbal_roll_;
    }
    const double deadzone = gimbal_angle_deadzone_deg_ * kPi / 180.0;
    filtered_yaw = std::abs(filtered_gimbal_yaw_) < deadzone
                       ? 0.0 : filtered_gimbal_yaw_;
    filtered_pitch = std::abs(filtered_gimbal_pitch_) < deadzone
                         ? 0.0 : filtered_gimbal_pitch_;
    filtered_roll = std::abs(filtered_gimbal_roll_) < deadzone
                        ? 0.0 : filtered_gimbal_roll_;
    filtered_yaw = (gimbal_invert_yaw_ ? -filtered_yaw : filtered_yaw) +
                   gimbal_yaw_offset_deg_ * kPi / 180.0;
    filtered_pitch = (gimbal_invert_pitch_ ? -filtered_pitch : filtered_pitch) +
                     gimbal_pitch_offset_deg_ * kPi / 180.0;
    filtered_roll = (gimbal_invert_roll_ ? -filtered_roll : filtered_roll) +
                    gimbal_roll_offset_deg_ * kPi / 180.0;
  }
  void gimbalToBodyVector(const double yaw, const double pitch,
                          const double roll, double& forward, double& right,
                          double& down) const {
    // Tracker angles are line-of-sight errors relative to the camera optical
    // axis, not absolute servo angles. Apply the physical boresight pitch here.
    const double mount_pitch = gimbal_mount_type_ == "VERTICAL"
                                   ? 0.5 * kPi
                                   : (gimbal_mount_type_ == "TILTED_45"
                                          ? 0.25 * kPi : 0.0);
    const double body_pitch = pitch + mount_pitch;
    forward = std::cos(body_pitch) * std::cos(yaw);
    const double unrolled_right = std::sin(yaw) * std::cos(body_pitch);
    const double unrolled_down = std::sin(body_pitch);
    right = std::cos(roll) * unrolled_right -
            std::sin(roll) * unrolled_down;
    down = std::sin(roll) * unrolled_right +
           std::cos(roll) * unrolled_down;
    const double norm = std::sqrt(forward * forward + right * right + down * down);
    if (norm > 1e-9) {
      forward /= norm;
      right /= norm;
      down /= norm;
    }
  }
  void gimbalControlErrors(const double yaw, const double pitch,
                           const double roll, double& lateral,
                           double& vertical) const {
    double forward = 0.0, right = 0.0, down = 0.0;
    gimbalToBodyVector(yaw, pitch, roll, forward, right, down);
    const double horizontal = std::sqrt(forward * forward + right * right);
    const double lateral_angle = std::atan2(right, std::max(1e-9, forward));
    const double vertical_angle = std::atan2(down, std::max(1e-9, horizontal));
    lateral = clamp(lateral_angle /
                        std::max(1e-6, gm_chase_max_lateral_deg_ * kPi / 180.0),
                    -1.0, 1.0);
    vertical = clamp(vertical_angle /
                         std::max(1e-6, gm_chase_max_vertical_deg_ * kPi / 180.0),
                     -1.0, 1.0);
  }
  double gmChaseForwardVelocity(const double pitch, const double dt) {
    double target = clamp(gm_chase_base_speed_, 0.0, target_forward_);
    if (gm_chase_forward_mode_ == "PITCH_BASED") {
      double pitch_error_deg = pitch * 180.0 / kPi;
      if (gimbal_mount_type_ == "VERTICAL") pitch_error_deg += 90.0;
      target = std::abs(pitch_error_deg) > gm_chase_pitch_deadzone_deg_
                   ? std::min(target_forward_,
                              std::abs(pitch_error_deg) * gm_chase_pitch_scaling_)
                   : 0.0;
    }
    const double max_change = std::max(0.0, gm_chase_acceleration_) * dt;
    gm_chase_current_speed_ += clamp(target - gm_chase_current_speed_,
                                     -max_change, max_change);
    return clamp(gm_chase_current_speed_, 0.0, target_forward_);
  }
  static void magnitude(double& x, double& y, double& z, const double max) {
    const double norm = std::sqrt(x * x + y * y + z * z);
    if (norm > max && norm > 0.0) {
      const double scale = max / norm;
      x *= scale;
      y *= scale;
      z *= scale;
    }
  }
  void resetControllers() {
    yaw_pid_.reset();
    right_pid_.reset();
    down_pid_.reset();
    ground_forward_pid_.reset();
    mc_altitude_pid_.reset();
    mc_roll_pid_.reset();
    fw_bank_pid_.reset();
    previous_yaw_ = filtered_yaw_ = smoothed_right_ = smoothed_down_ = 0.0;
  }
  double mcAttitudeThrust(const double dt) {
    double thrust = mc_hover_thrust_;
    const bool altitude_known = altitude_valid_ || local_altitude_valid_;
    const double active_altitude = altitude_valid_ ? altitude_ : local_altitude_;
    if (mc_altitude_hold_enabled_ && altitude_known) {
      if (!mc_altitude_target_initialized_) {
        mc_altitude_target_ = active_altitude + mc_target_altitude_offset_;
        mc_altitude_target_initialized_ = true;
        mc_altitude_pid_.reset();
      }
      // Pid has a zero setpoint contract; measurement=current-target yields
      // positive thrust when the aircraft is below the captured altitude.
      thrust += mc_altitude_pid_.update(
          active_altitude - mc_altitude_target_, dt);
    }
    if (mc_thrust_compensation_ && attitude_valid_) {
      const double vertical_factor = std::cos(roll_deg_ * kPi / 180.0) *
                                     std::cos(pitch_ * kPi / 180.0);
      if (vertical_factor > 0.5) {
        thrust += mc_thrust_compensation_gain_ * mc_hover_thrust_ *
                  (1.0 / vertical_factor - 1.0);
      }
    }
    return clamp(thrust, mc_min_thrust_, mc_max_thrust_);
  }
  double currentFixedWingAirspeed(const double now) const {
    if (airspeed_valid_ && now - airspeed_time_ <= fw_airspeed_timeout_) {
      return clamp(airspeed_, 0.0, fw_max_airspeed_ * 1.5);
    }
    if (ground_speed_valid_) return clamp(ground_speed_, fw_min_airspeed_,
                                          fw_max_airspeed_);
    return fw_cruise_airspeed_;
  }
  double fwL1Guidance(const double cross_track_error, const double speed,
                      const double dt) {
    const double speed_ratio = clamp(
        (speed - fw_min_airspeed_) /
            std::max(1e-3, fw_max_airspeed_ - fw_min_airspeed_),
        0.0, 1.0);
    const double l1 = fw_l1_adaptive_
        ? fw_l1_min_distance_ + speed_ratio *
              (fw_l1_max_distance_ - fw_l1_min_distance_)
        : fw_l1_distance_;
    const double rate = fw_l1_initialized_
        ? (cross_track_error - fw_last_cross_track_) / std::max(dt, 1e-3)
        : 0.0;
    fw_l1_initialized_ = true;
    fw_last_cross_track_ = cross_track_error;
    const double safe_speed = std::max(speed, fw_min_airspeed_);
    const double damping = 2.0 * fw_l1_damping_ * fw_l1_damping_;
    double lateral_acceleration =
        damping * safe_speed * safe_speed / std::max(1.0, l1) *
        (cross_track_error * fw_l1_lateral_scale_ / std::max(1.0, l1));
    lateral_acceleration += fw_l1_damping_ * rate * fw_l1_lateral_scale_;
    return clamp(lateral_acceleration / safe_speed * 180.0 / kPi,
                 -max_yaw_, max_yaw_);
  }
  double fwCoordinatedBank(const double yaw_rate_deg_s,
                           const double speed) const {
    double bank = std::atan((yaw_rate_deg_s * kPi / 180.0) *
                            std::max(speed, fw_min_airspeed_) / 9.81) *
                  180.0 / kPi;
    double load_limited_bank = 0.0;
    if (fw_max_load_factor_ > 1.0) {
      load_limited_bank = std::acos(1.0 / fw_max_load_factor_) * 180.0 / kPi;
    }
    const double limit = std::min(fw_max_bank_angle_, load_limited_bank);
    return clamp(bank, -limit, limit);
  }
  void fwTecs(const double altitude_error, const double speed,
              const double dt, double& pitch_rate, double& thrust) {
    if (!fw_tecs_enabled_) {
      pitch_rate = clamp(0.5 * altitude_error,
                         -fw_max_pitch_rate_, fw_max_pitch_rate_);
      thrust = fw_cruise_thrust_;
      return;
    }
    const double spe_error = fw_tecs_spe_weight_ * altitude_error;
    const double ske_error =
        (fw_cruise_airspeed_ * fw_cruise_airspeed_ - speed * speed) /
        (2.0 * 9.81);
    const double ste_error = spe_error + ske_error;
    const double seb_error = spe_error - ske_error;
    fw_ste_integral_ = clamp(fw_ste_integral_ + ste_error * dt,
                             -fw_tecs_max_integral_, fw_tecs_max_integral_);
    fw_seb_integral_ = clamp(fw_seb_integral_ + seb_error * dt,
                             -fw_tecs_max_integral_, fw_tecs_max_integral_);
    const double seb_rate = fw_tecs_initialized_
        ? (seb_error - fw_previous_seb_error_) / std::max(dt, 1e-3) : 0.0;
    fw_tecs_initialized_ = true;
    fw_previous_seb_error_ = seb_error;
    pitch_rate = clamp(
        (seb_error / fw_tecs_time_constant_ +
         fw_tecs_pitch_damping_ * seb_rate + 0.01 * fw_seb_integral_) * 10.0,
        -fw_max_pitch_rate_, fw_max_pitch_rate_);
    const double requested = clamp(
        fw_cruise_thrust_ + ste_error / fw_tecs_time_constant_ +
            fw_tecs_throttle_damping_ * fw_ste_integral_,
        fw_min_thrust_, fw_max_thrust_);
    const double max_change = fw_thrust_slew_rate_ * std::max(dt, 1e-3);
    fw_last_thrust_ += clamp(requested - fw_last_thrust_,
                             -max_change, max_change);
    thrust = clamp(fw_last_thrust_, fw_min_thrust_, fw_max_thrust_);
  }
  void resetFixedWingState() {
    fw_l1_initialized_ = fw_tecs_initialized_ = false;
    fw_last_cross_track_ = fw_previous_seb_error_ = 0.0;
    fw_ste_integral_ = fw_seb_integral_ = 0.0;
    fw_last_thrust_ = fw_cruise_thrust_;
  }
  void feasibleYaw(double& yaw, const double now) {
    const double dt = yaw_feasibility_time_ > 0.0
                          ? clamp(now - yaw_feasibility_time_, 1e-3, 0.1)
                          : 1.0 / 30.0;
    yaw_feasibility_time_ = now;
    if (!yaw_feasible_initialized_) yaw_feasible_initialized_ = true;
    double acceleration = (yaw - previous_feasible_yaw_) / dt;
    acceleration = clamp(acceleration, -yaw_acceleration_, yaw_acceleration_);
    const double jerk = (acceleration - previous_yaw_acceleration_) / dt;
    if (std::abs(jerk) > yaw_feasible_jerk_) {
      acceleration = previous_yaw_acceleration_ +
          clamp(jerk, -yaw_feasible_jerk_, yaw_feasible_jerk_) * dt;
    }
    yaw = previous_feasible_yaw_ + acceleration * dt;
    previous_feasible_yaw_ = yaw;
    previous_yaw_acceleration_ = acceleration;
  }
  double smoothYaw(double value, const double dt, const double speed) {
    if (!smooth_yaw_enabled_) return value;
    value = std::abs(value) < yaw_deadzone_
                ? 0.0
                : sign(value) * (std::abs(value) - yaw_deadzone_);
    if (yaw_speed_scaling_) {
      double factor = 1.0;
      if (speed <= yaw_min_speed_) factor = yaw_low_factor_;
      else if (speed < yaw_max_speed_) {
        const double t =
            (speed - yaw_min_speed_) / (yaw_max_speed_ - yaw_min_speed_);
        factor = yaw_low_factor_ + t * (1.0 - yaw_low_factor_);
      }
      value *= factor;
    }
    previous_yaw_ += clamp(value - previous_yaw_,
                           -yaw_acceleration_ * dt, yaw_acceleration_ * dt);
    filtered_yaw_ =
        yaw_alpha_ * previous_yaw_ + (1.0 - yaw_alpha_) * filtered_yaw_;
    return filtered_yaw_;
  }
  void feasible(double& x, double& y, double& z, const double now) {
    magnitude(x, y, z, feasible_velocity_);
    const double dt = feasibility_time_ > 0.0 ? now - feasibility_time_ : 0.033;
    feasibility_time_ = now;
    if (dt > 0.0 && !velocity_history_.empty()) {
      const auto previous = velocity_history_.back();
      std::array<double, 3> acceleration{
          (x - previous[0]) / dt, (y - previous[1]) / dt,
          (z - previous[2]) / dt};
      double norm = std::sqrt(acceleration[0] * acceleration[0] +
                              acceleration[1] * acceleration[1] +
                              acceleration[2] * acceleration[2]);
      if (norm > feasible_acceleration_) {
        const double scale = feasible_acceleration_ / norm;
        for (auto& value : acceleration) value *= scale;
        x = previous[0] + acceleration[0] * dt;
        y = previous[1] + acceleration[1] * dt;
        z = previous[2] + acceleration[2] * dt;
      }
      if (!acceleration_history_.empty()) {
        const auto previous_acceleration = acceleration_history_.back();
        std::array<double, 3> jerk{
            (acceleration[0] - previous_acceleration[0]) / dt,
            (acceleration[1] - previous_acceleration[1]) / dt,
            (acceleration[2] - previous_acceleration[2]) / dt};
        norm = std::sqrt(jerk[0] * jerk[0] + jerk[1] * jerk[1] +
                         jerk[2] * jerk[2]);
        if (norm > feasible_jerk_) {
          const double scale = feasible_jerk_ / norm;
          for (auto& value : jerk) value *= scale;
          for (size_t i = 0; i < 3; ++i) {
            acceleration[i] =
                previous_acceleration[i] + jerk[i] * dt;
          }
          x = previous[0] + acceleration[0] * dt;
          y = previous[1] + acceleration[1] * dt;
          z = previous[2] + acceleration[2] * dt;
        }
      }
      acceleration_history_.push_back(acceleration);
      if (acceleration_history_.size() > 5) acceleration_history_.pop_front();
    }
    velocity_history_.push_back({x, y, z});
    if (velocity_history_.size() > 5) velocity_history_.pop_front();
  }
  void adaptive(double& forward, double& down, const double y,
                const double now, Output& out) {
    adaptive_history_.push_back({now, y});
    if (adaptive_history_.size() > 30) adaptive_history_.pop_front();
    if (++adaptive_count_ < adaptive_warmup_ ||
        adaptive_history_.size() < 2) return;
    const size_t index =
        adaptive_history_.size() > 5 ? adaptive_history_.size() - 5 : 0;
    const auto first = adaptive_history_[index];
    const auto last = adaptive_history_.back();
    const double dt = last.first - first.first;
    if (dt > 0.001) {
      const double rate = (last.second - first.second) / dt * video_height_;
      adaptive_rate_ =
          adaptive_alpha_ * rate + (1.0 - adaptive_alpha_) * adaptive_rate_;
    }
    const double error = adaptive_rate_ - down * pixel_rate_ * video_height_;
    out.adaptive = true;
    out.vertical_rate_error = error;
    out.smoothed_vertical_rate = adaptive_rate_;
    if (std::abs(error) < adaptive_threshold_) return;
    double correction =
        clamp((error - sign(error) * adaptive_threshold_) * adaptive_gain_,
              -adaptive_max_, adaptive_max_);
    if (mode_ == "sideslip") correction *= 0.5;
    out.adaptive_down = correction * 0.01;
    out.adaptive_forward =
        adaptive_forward_enabled_
            ? -correction * adaptive_forward_gain_ * 0.01
            : 0.0;
    down += out.adaptive_down;
    forward += out.adaptive_forward;
  }

  std::string mode_, profile_, vehicle_type_;
  bool auto_switch_{false}, ramp_down_{true}, pom_{false};
  double switch_velocity_{3.0}, switch_hysteresis_{0.5}, last_switch_{0.0};
  double max_forward_{8.0}, max_lateral_{3.0}, max_vertical_{2.0};
  double max_magnitude_{15.0}, max_yaw_{90.0};
  double min_altitude_{1.0}, max_altitude_{120.0}, altitude_{0.0}, local_altitude_{0.0};
  double altitude_timeout_{0.5}, altitude_time_{0.0}, local_altitude_time_{0.0};
  bool altitude_valid_{false}, local_altitude_valid_{false}, altitude_fail_safe_{true};
  int altitude_violations_{0};
  double forward_{0.0}, target_forward_{5.0}, ramp_rate_{0.5};
  double loss_forward_{0.0}, velocity_deadzone_{0.01};
  double loss_threshold_{1.5}, loss_timeout_{3.0};
  bool lost_{false}, emergency_{false};
  double loss_start_{0.0}, loss_duration_{0.0};
  bool smooth_velocity_{true};
  double smoothing_factor_{0.3}, smoothed_right_{0.0}, smoothed_down_{0.0};
  Pid yaw_pid_, right_pid_, down_pid_, ground_forward_pid_;
  Pid mc_altitude_pid_, mc_roll_pid_, fw_bank_pid_;
  bool smooth_yaw_enabled_{true}, yaw_speed_scaling_{true};
  double yaw_deadzone_{0.5}, yaw_acceleration_{90.0}, yaw_alpha_{0.7};
  double yaw_min_speed_{0.5}, yaw_max_speed_{5.0}, yaw_low_factor_{0.5};
  double previous_yaw_{0.0}, filtered_yaw_{0.0};
  double yaw_feasible_jerk_{360.0}, yaw_feasibility_time_{0.0};
  double previous_feasible_yaw_{0.0}, previous_yaw_acceleration_{0.0};
  bool yaw_feasible_initialized_{false};
  double feasible_velocity_{8.0}, feasible_acceleration_{5.0};
  double feasible_jerk_{20.0};
  double feasibility_time_{0.0};
  std::deque<std::array<double, 3>> velocity_history_, acceleration_history_;
  bool pitch_enabled_{false}, pitch_initialized_{false};
  double pitch_gain_{0.05}, pitch_alpha_{0.7}, pitch_min_velocity_{1.0};
  double pitch_deadband_{2.0}, pitch_max_angle_{45.0};
  double pitch_max_correction_{0.3}, pitch_{0.0}, smoothed_pitch_{0.0};
  bool adaptive_enabled_{false}, adaptive_forward_enabled_{false};
  double adaptive_alpha_{0.2}, adaptive_threshold_{5.0}, adaptive_max_{1.0};
  double adaptive_gain_{0.3}, adaptive_confidence_{0.6};
  double adaptive_forward_gain_{0.1}, pixel_rate_{0.05}, video_height_{480.0};
  double anti_windup_gain_{0.1}, min_tracking_confidence_{0.5};
  double confidence_hysteresis_{0.08};
  bool target_confidence_locked_{false}, profile_sensor_fallback_{true};
  bool transition_blending_{true}, transition_active_{false};
  double transition_duration_{0.5}, transition_start_{0.0};
  double profile_switch_min_interval_{0.0}, last_profile_switch_{0.0};
  bool have_profile_switch_{false};
  std::string transition_from_profile_;
  Output transition_from_, last_output_;
  bool have_last_output_{false};
  double ground_descent_velocity_{0.25}, ground_min_descent_height_{2.0};
  bool ground_altitude_scaling_{true};
  double ground_altitude_factor_{0.005};
  double ground_base_adjustment_x_{0.2}, ground_base_adjustment_y_{0.2};
  double ground_min_error_scale_{1.0}, ground_max_error_scale_{1.5};
  double gm_vector_speed_{3.0};
  bool relative_state_enabled_{true};
  double relative_target_distance_{5.0}, relative_target_down_{0.0};
  double relative_position_kp_{0.4}, relative_velocity_ff_gain_{0.8};
  double relative_max_correction_{3.0};
  bool uncertainty_control_enabled_{true};
  double uncertainty_nominal_sigma_m_{0.75}, uncertainty_nominal_sigma_px_{40.0};
  double uncertainty_slow_sigma_ratio_{1.0}, uncertainty_abort_sigma_ratio_{2.5};
  double uncertainty_min_control_scale_{0.25}, uncertainty_yaw_scale_{0.65};
  double reid_initial_scale_{0.35}, reid_recovery_duration_{0.6};
  bool reid_recovery_active_{false};
  double reid_recovery_start_{0.0};
  double gm_vector_current_speed_{0.0};
  std::string gm_chase_forward_mode_{"CONSTANT"};
  double gm_chase_base_speed_{2.0}, gm_chase_acceleration_{2.0};
  double gm_chase_pitch_scaling_{0.15}, gm_chase_pitch_deadzone_deg_{2.0};
  double gm_chase_max_lateral_deg_{90.0}, gm_chase_max_vertical_deg_{90.0};
  double gm_chase_current_speed_{0.0};
  std::string gimbal_mount_type_{"HORIZONTAL"};
  double gimbal_yaw_offset_deg_{0.0}, gimbal_pitch_offset_deg_{0.0};
  double gimbal_roll_offset_deg_{0.0}, gimbal_angle_alpha_{0.7};
  double gimbal_angle_deadzone_deg_{0.25};
  bool gimbal_invert_yaw_{false}, gimbal_invert_pitch_{false};
  bool gimbal_invert_roll_{false}, gm_vector_altitude_control_{true};
  bool gimbal_angles_initialized_{false};
  double filtered_gimbal_yaw_{0.0}, filtered_gimbal_pitch_{0.0};
  double filtered_gimbal_roll_{0.0};
  bool allow_attitude_rate_profiles_{false}, allow_fixed_wing_profile_{false};
  double mc_hover_thrust_{0.5}, mc_min_thrust_{0.1}, mc_max_thrust_{0.9};
  double mc_max_roll_rate_{45.0}, mc_max_pitch_rate_{45.0};
  bool mc_altitude_hold_enabled_{true}, mc_thrust_compensation_{true};
  bool mc_coordinated_roll_{true}, mc_yaw_gating_{true};
  bool mc_altitude_target_initialized_{false};
  double mc_target_altitude_offset_{0.0}, mc_altitude_thrust_min_{-0.3};
  double mc_altitude_thrust_max_{0.3}, mc_thrust_compensation_gain_{1.0};
  double mc_turn_coordination_gain_{1.0}, mc_max_bank_angle_{35.0};
  double mc_yaw_gate_{0.35}, mc_altitude_target_{0.0};
  double roll_deg_{0.0}, yaw_deg_{0.0}, ground_speed_{0.0};
  bool attitude_valid_{false}, ground_speed_valid_{false};
  double fw_cruise_thrust_{0.6}, fw_max_roll_rate_{35.0}, fw_max_pitch_rate_{25.0};
  double fw_min_airspeed_{12.0}, fw_cruise_airspeed_{18.0};
  double fw_max_airspeed_{30.0}, fw_airspeed_timeout_{0.5};
  double fw_max_bank_angle_{35.0}, fw_max_load_factor_{2.5};
  double fw_l1_distance_{50.0}, fw_l1_damping_{0.75};
  bool fw_l1_adaptive_{true}, fw_tecs_enabled_{true};
  double fw_l1_min_distance_{20.0}, fw_l1_max_distance_{100.0};
  double fw_l1_lateral_scale_{50.0}, fw_tecs_time_constant_{5.0};
  double fw_tecs_pitch_damping_{0.15}, fw_tecs_throttle_damping_{0.05};
  double fw_tecs_spe_weight_{1.0}, fw_tecs_altitude_scale_{20.0};
  double fw_tecs_max_integral_{50.0}, fw_min_thrust_{0.2};
  double fw_max_thrust_{1.0}, fw_thrust_slew_rate_{0.5};
  bool fw_stall_protection_{true}, fw_orbit_on_loss_{true};
  double fw_stall_margin_{3.0}, fw_stall_recovery_pitch_{-5.0};
  double fw_stall_recovery_thrust_{1.0}, fw_orbit_radius_{100.0};
  double fw_orbit_direction_{1.0}, airspeed_{0.0}, airspeed_time_{0.0};
  bool airspeed_valid_{false}, fw_l1_initialized_{false};
  bool fw_tecs_initialized_{false};
  double fw_last_cross_track_{0.0}, fw_ste_integral_{0.0};
  double fw_seb_integral_{0.0}, fw_previous_seb_error_{0.0};
  double fw_last_thrust_{0.6};
  int adaptive_warmup_{10}, adaptive_count_{0};
  double adaptive_rate_{0.0};
  std::deque<std::pair<double, double>> adaptive_history_;
  uint32_t sequence_{0};
};

class FollowerNode {
 public:
  FollowerNode() : private_nh_("~"), core_(private_nh_) {
    private_nh_.param("publish_rate", rate_, 60.0);
    private_nh_.param("error_timeout_sec", timeout_, 0.25);
    private_nh_.param("controller_command_timeout_sec", controller_timeout_, 0.25);
    private_nh_.param("tracker_capture_timeout_sec", capture_timeout_, timeout_);
    private_nh_.param("tracker_latency_warning_sec", latency_warning_,
                      std::min(0.12, 0.5 * capture_timeout_));
    private_nh_.param("tracker_latency_abort_sec", latency_abort_,
                      capture_timeout_);
    capture_timeout_ = std::max(0.05, capture_timeout_);
    latency_warning_ = clamp(latency_warning_, 0.0, capture_timeout_);
    latency_abort_ = std::max(latency_warning_ + 1e-3, latency_abort_);
    private_nh_.param("tracker_error_topic", error_topic_,
                      std::string("/tracker_node/normalized_error"));
    private_nh_.param("gimbal_state_topic", gimbal_state_topic_,
                      std::string("/pod/gimbal/state"));
    private_nh_.param("gimbal_state_timeout_sec", gimbal_state_timeout_sec_, 0.15);
    private_nh_.param("require_gimbal_state_for_control",
                      require_gimbal_state_for_control_, false);
    gimbal_state_timeout_sec_ = std::max(0.02, gimbal_state_timeout_sec_);
    private_nh_.param("use_angular_error", use_angular_error_, true);
    private_nh_.param("use_tracker_control_confidence",
                      use_tracker_control_confidence_, true);
    private_nh_.param("require_capture_timestamp",
                      require_capture_timestamp_, false);
    private_nh_.param("tracker_future_tolerance_sec",
                      future_tolerance_sec_, 0.02);
    private_nh_.param("reject_out_of_order_capture",
                      reject_out_of_order_capture_, true);
    future_tolerance_sec_ = clamp(future_tolerance_sec_, 0.0, 1.0);
    private_nh_.param("camera_fov_horizontal_deg", camera_fov_horizontal_deg_, 60.0);
    private_nh_.param("camera_fov_vertical_deg", camera_fov_vertical_deg_, 45.0);
    private_nh_.param("mavros_namespace", mavros_ns_, std::string("/mavros"));
    while (mavros_ns_.size() > 1 && mavros_ns_.back() == '/') mavros_ns_.pop_back();
    bool legacy_mavros_output = true;
    bool legacy_mrs_output = false;
    private_nh_.param("enable_mavros_output", legacy_mavros_output, true);
    private_nh_.param("mavros_velocity_topic", mavros_topic_,
                      mavros_ns_ + "/setpoint_velocity/cmd_vel");
    private_nh_.param("enable_mrs_velocity_reference", legacy_mrs_output, false);
    private_nh_.param("mrs_velocity_reference_topic", mrs_topic_,
                      std::string("/uav1/control_manager/velocity_reference_in"));
    private_nh_.param("mrs_velocity_frame", mrs_frame_, std::string("uav1/fcu"));
    private_nh_.param("mavros_body_velocity_topic", mavros_raw_topic_,
                      mavros_ns_ + "/setpoint_raw/local");

    std::string configured_backend;
    private_nh_.param("control_output_backend", configured_backend,
                      std::string(""));
    if (configured_backend.empty()) {
      if (legacy_mavros_output && legacy_mrs_output) {
        configured_backend = "invalid";
        ROS_ERROR("[FollowerNode] MAVROS and MRS outputs cannot both be enabled");
      } else if (legacy_mavros_output) {
        configured_backend = "mavros_twist_legacy";
      } else if (legacy_mrs_output) {
        configured_backend = "mrs_velocity";
      } else {
        configured_backend = "command_only";
      }
    }
    follower::ControlGuardConfig guard_config;
    guard_config.output_backend = follower::parseOutputBackend(configured_backend);
    private_nh_.param("require_control_lease", guard_config.require_lease, false);
    private_nh_.param("minimum_control_lease_sec",
                      guard_config.minimum_lease_sec, 0.10);
    private_nh_.param("maximum_control_lease_sec",
                      guard_config.maximum_lease_sec, 30.0);
    control_guard_.reset(new follower::ControlGuard(guard_config));
    output_backend_ = control_guard_->outputBackend();
    output_backend_name_ = follower::outputBackendName(output_backend_);
    mavros_output_ = output_backend_ == follower::OutputBackend::kMavrosBody ||
                     output_backend_ == follower::OutputBackend::kMavrosTwistLegacy;
    mrs_output_ = output_backend_ == follower::OutputBackend::kMrsVelocity;

    private_nh_.param("flight_state_timeout_sec", flight_state_timeout_, 0.5);
    private_nh_.param("require_mavros_connected", require_mavros_connected_, true);
    private_nh_.param("require_mavros_armed", require_mavros_armed_, true);
    private_nh_.param("required_mavros_mode", required_mavros_mode_,
                      std::string("OFFBOARD"));
    private_nh_.param("require_controller_ready_for_mrs",
                      require_controller_ready_for_mrs_, true);
    private_nh_.param("trusted_controller_name", trusted_controller_name_,
                      std::string(""));
    flight_state_timeout_ = std::max(0.05, flight_state_timeout_);
    command_pub_ = private_nh_.advertise<follower::FollowerCommand>("follower_command", 10);
    status_pub_ = private_nh_.advertise<follower::FollowerStatus>("follower_status", 10);
    feedback_pub_ = private_nh_.advertise<follower::ControllerFeedback>("controller_feedback", 10);
    odom_pub_ = private_nh_.advertise<nav_msgs::Odometry>("velocity_command_odom", 10);
    marker_pub_ = private_nh_.advertise<visualization_msgs::Marker>("velocity_command_marker", 10);
    actual_marker_pub_ = private_nh_.advertise<visualization_msgs::Marker>("actual_velocity_marker", 10);
    if (output_backend_ == follower::OutputBackend::kMavrosTwistLegacy) {
      mavros_pub_ = nh_.advertise<geometry_msgs::TwistStamped>(mavros_topic_, 10);
    }
    if (output_backend_ == follower::OutputBackend::kMavrosBody) {
      mavros_raw_pub_ = nh_.advertise<mavros_msgs::PositionTarget>(
          mavros_raw_topic_, 10);
    }
    if (mrs_output_) mrs_pub_ = nh_.advertise<mrs_msgs::VelocityReferenceStamped>(mrs_topic_, 10);
    const ros::TransportHints low_latency_transport =
        ros::TransportHints().tcpNoDelay();
    // Control consumes only the newest sample; deep subscriber queues turn a
    // temporary CPU spike into stale commands and are therefore unsafe here.
    error_sub_ = nh_.subscribe(error_topic_, 1, &FollowerNode::errorCallback,
                               this, low_latency_transport);
    gimbal_state_sub_ = nh_.subscribe(
        gimbal_state_topic_, 1, &FollowerNode::gimbalStateCallback, this,
        low_latency_transport);
    imu_sub_ = nh_.subscribe(mavros_ns_ + "/imu/data", 1,
                             &FollowerNode::imuCallback, this,
                             low_latency_transport);
    altitude_sub_ = nh_.subscribe(mavros_ns_ + "/altitude", 1,
                                  &FollowerNode::altitudeCallback, this,
                                  low_latency_transport);
    pose_sub_ = nh_.subscribe(mavros_ns_ + "/local_position/pose", 1,
                              &FollowerNode::poseCallback, this,
                              low_latency_transport);
    velocity_sub_ = nh_.subscribe(
        mavros_ns_ + "/local_position/velocity_local", 1,
        &FollowerNode::velocityCallback, this, low_latency_transport);
    vfr_hud_sub_ = nh_.subscribe(mavros_ns_ + "/vfr_hud", 1,
                                 &FollowerNode::vfrHudCallback, this,
                                 low_latency_transport);
    state_sub_ = nh_.subscribe(mavros_ns_ + "/state", 1,
                               &FollowerNode::stateCallback, this,
                               low_latency_transport);
    controller_sub_ = private_nh_.subscribe(
        "controller_command", 1, &FollowerNode::controllerCallback, this,
        low_latency_transport);
    start_srv_ = private_nh_.advertiseService("start", &FollowerNode::start, this);
    stop_srv_ = private_nh_.advertiseService("stop", &FollowerNode::stop, this);
    emergency_srv_ = private_nh_.advertiseService("emergency_stop", &FollowerNode::emergency, this);
    mode_srv_ = private_nh_.advertiseService("set_mode", &FollowerNode::mode, this);
    profile_srv_ = private_nh_.advertiseService(
        "set_profile", &FollowerNode::profile, this);
    lease_srv_ = private_nh_.advertiseService(
        "manage_control_lease", &FollowerNode::manageLease, this);
    timer_ = nh_.createTimer(ros::Duration(1.0 / std::max(2.0, rate_)),
                             &FollowerNode::tick, this);
    previous_time_ = ros::WallTime::now();
    updater_.setHardwareID("aircraft_follow_controller");
    updater_.add("follower_control", this, &FollowerNode::diagnostics);
    if (!control_guard_->configurationValid()) {
      ROS_ERROR("[FollowerNode] Invalid/conflicting output backend; all actuator output is blocked");
    }
    ROS_INFO("[FollowerNode] C++ implementation started at %.1f Hz, backend=%s, lease=%s",
             rate_, output_backend_name_.c_str(),
             control_guard_->requireLease() ? "required" : "compatibility");
  }

 private:
  void errorCallback(const tracker::NormalizedError::ConstPtr& msg) {
    const bool finite = std::isfinite(msg->error_x) &&
        std::isfinite(msg->error_y) && std::isfinite(msg->error_size) &&
        std::isfinite(msg->confidence) &&
        std::isfinite(msg->control_confidence) &&
        std::isfinite(msg->inference_latency_ms);
    if (!finite) {
      ++rejected_input_count_;
      ROS_WARN_THROTTLE(2.0,
                        "[FollowerNode] Rejected non-finite tracker input");
      return;
    }
    const ros::Time ros_now = ros::Time::now();
    if (!msg->capture_timestamp.isZero() && !ros_now.isZero() &&
        msg->capture_timestamp >
            ros_now + ros::Duration(future_tolerance_sec_)) {
      ++rejected_input_count_;
      ROS_WARN_THROTTLE(2.0,
                        "[FollowerNode] Rejected future capture timestamp");
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (reject_out_of_order_capture_ &&
        !msg->capture_timestamp.isZero() &&
        !last_capture_timestamp_.isZero() &&
        msg->capture_timestamp < last_capture_timestamp_) {
      ++rejected_input_count_;
      ROS_WARN_THROTTLE(2.0,
                        "[FollowerNode] Rejected out-of-order capture timestamp");
      return;
    }
    error_ = *msg;
    error_time_ = ros::WallTime::now();
    have_error_ = true;
    if (!msg->capture_timestamp.isZero()) {
      last_capture_timestamp_ = msg->capture_timestamp;
    }
  }
  // In the pod profile, pod_tracker_adapter already converts the camera LOS
  // into body coordinates.  Direct GimbalState is therefore a safety gate,
  // not a second rotation that could double-compensate the command.
  void gimbalStateCallback(const pod_msgs::GimbalState::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    gimbal_state_ = *msg;
    gimbal_state_time_ = ros::WallTime::now();
    have_gimbal_state_ = true;
  }
  void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    double roll, pitch, yaw;
    tf::Matrix3x3(tf::Quaternion(msg->orientation.x, msg->orientation.y,
                                msg->orientation.z, msg->orientation.w))
        .getRPY(roll, pitch, yaw);
    core_.setAttitude(roll * 180.0 / kPi, pitch * 180.0 / kPi,
                      yaw * 180.0 / kPi);
  }
  void altitudeCallback(const mavros_msgs::Altitude::ConstPtr& msg) {
    core_.setAltitude(msg->relative, ros::WallTime::now().toSec());
  }
  void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    pose_ = *msg;
    have_pose_ = true;
    core_.setLocalAltitude(msg->pose.position.z, ros::WallTime::now().toSec());
  }
  void velocityCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    velocity_ = *msg;
    have_velocity_ = true;
    core_.setGroundSpeed(std::sqrt(
        msg->twist.linear.x * msg->twist.linear.x +
        msg->twist.linear.y * msg->twist.linear.y +
        msg->twist.linear.z * msg->twist.linear.z));
  }
  void vfrHudCallback(const mavros_msgs::VFR_HUD::ConstPtr& msg) {
    core_.setAirspeed(msg->airspeed, ros::WallTime::now().toSec());
    core_.setGroundSpeed(msg->groundspeed);
  }
  void stateCallback(const mavros_msgs::State::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    flight_state_ = *msg;
    flight_state_time_ = ros::WallTime::now();
    have_flight_state_ = true;
  }
  void controllerCallback(const follower::ControllerCommand::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    controller_ = *msg;
    have_controller_ = true;
    controller_time_ = ros::WallTime::now();
  }
  bool start(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res) {
    if (!req.data) {
      res.success = false;
      res.message = "Start service requires data=true";
      return true;
    }
    if (!control_guard_->configurationValid()) {
      res.success = false;
      res.message = "Output backend configuration is invalid";
      return true;
    }
    if (core_.emergencyActive()) {
      res.success = false;
      res.message = "Release emergency stop before starting";
      return true;
    }
    if (!control_guard_->authorized(ros::WallTime::now().toSec())) {
      res.success = false;
      res.message = "A valid aircraft-follow control lease is required";
      return true;
    }
    active_ = true;
    core_.reset();
    res.success = true;
    res.message = "Following started";
    return true;
  }
  bool stop(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res) {
    if (!req.data) {
      res.success = false;
      res.message = "Stop service requires data=true";
      return true;
    }
    active_ = false;
    core_.reset();
    res.success = true;
    res.message = "Following stopped";
    return true;
  }
  bool emergency(std_srvs::SetBool::Request& req,
                 std_srvs::SetBool::Response& res) {
    core_.emergency(req.data);
    if (req.data) {
      active_ = false;
      core_.reset();
    }
    res.success = true;
    res.message = req.data ? "Emergency stop active" : "Emergency stop released";
    return true;
  }
  bool manageLease(follower::ManageControlLease::Request& req,
                   follower::ManageControlLease::Response& res) {
    const double wall_now = ros::WallTime::now().toSec();
    const follower::LeaseResult result = req.acquire
        ? control_guard_->acquire(req.requester, req.output_backend,
                                  req.lease_duration_sec, wall_now)
        : control_guard_->release(req.requester, wall_now);
    if (!req.acquire && result.success) {
      active_ = false;
      core_.reset();
    }
    res.success = result.success;
    res.message = result.message;
    res.active_requester = result.requester;
    res.active_output_backend =
        follower::outputBackendName(result.output_backend);
    const double remaining = std::max(0.0, result.expires_at - wall_now);
    res.expires_at = remaining > 0.0
        ? ros::Time::now() + ros::Duration(remaining) : ros::Time();
    return true;
  }

  bool platformReady(const bool controller_fresh,
                     const follower::ControllerCommand& controller,
                     const bool flight_state_fresh,
                     const mavros_msgs::State& flight_state,
                     std::string* reason) const {
    if (reason == nullptr) return false;
    reason->clear();
    if (!control_guard_->configurationValid()) {
      *reason = "invalid or conflicting output backend";
      return false;
    }
    if (output_backend_ == follower::OutputBackend::kCommandOnly) return true;
    if (output_backend_ == follower::OutputBackend::kMavrosBody ||
        output_backend_ == follower::OutputBackend::kMavrosTwistLegacy) {
      if (!flight_state_fresh) {
        *reason = "MAVROS flight state is stale or unavailable";
        return false;
      }
      if (require_mavros_connected_ && !flight_state.connected) {
        *reason = "MAVROS is disconnected";
        return false;
      }
      if (require_mavros_armed_ && !flight_state.armed) {
        *reason = "aircraft is not armed";
        return false;
      }
      if (!required_mavros_mode_.empty() &&
          flight_state.mode != required_mavros_mode_) {
        *reason = "flight mode is not " + required_mavros_mode_;
        return false;
      }
      return true;
    }
    if (output_backend_ == follower::OutputBackend::kMrsVelocity &&
        require_controller_ready_for_mrs_) {
      if (!controller_fresh || !controller.controller_ready) {
        *reason = "MRS/controller readiness is stale or false";
        return false;
      }
      if (!trusted_controller_name_.empty() &&
          controller.controller_name != trusted_controller_name_) {
        *reason = "controller identity is not trusted";
        return false;
      }
    }
    return true;
  }
  bool mode(follower::SetMode::Request& req, follower::SetMode::Response& res) {
    res.success = core_.setMode(req.mode, ros::WallTime::now().toSec());
    res.message = res.success ? "Mode set to " + req.mode : "Unknown mode";
    return true;
  }
  bool profile(follower::SetMode::Request& req,
               follower::SetMode::Response& res) {
    std::string rejection;
    res.success = core_.setProfile(req.mode, ros::WallTime::now().toSec(),
                                   &rejection);
    res.message = res.success ? "Follower profile set to " + req.mode
                              : rejection;
    return true;
  }
  void diagnostics(diagnostic_updater::DiagnosticStatusWrapper& status) {
    int level = diagnostic_msgs::DiagnosticStatus::OK;
    std::string summary = last_control_state_.empty()
        ? "Follower initializing" : last_control_state_;
    if (!control_guard_->configurationValid() ||
        last_control_state_ == "EMERGENCY_STOP" ||
        last_control_state_ == "BLOCKED_PLATFORM" ||
        last_control_state_ == "LATENCY_ABORT" ||
        last_control_state_ == "UNCERTAINTY_ABORT") {
      level = diagnostic_msgs::DiagnosticStatus::ERROR;
    } else if (last_control_state_ != "FOLLOWING" &&
               last_control_state_ != "DISABLED") {
      level = diagnostic_msgs::DiagnosticStatus::WARN;
    }
    status.summary(level, summary);
    status.add("active", active_);
    status.add("command_valid", last_command_valid_);
    status.add("control_state", last_control_state_);
    status.add("invalid_reason", last_invalid_reason_);
    status.add("output_backend", output_backend_name_);
    status.add("control_authorized", last_control_authorized_);
    status.add("platform_ready", last_platform_ready_);
    status.add("lease_required", control_guard_->requireLease());
    status.add("lease_requester", control_guard_->requester());
    status.add("lease_remaining_sec",
               control_guard_->remaining(ros::WallTime::now().toSec()));
    status.add("rejected_inputs",
               static_cast<long long>(rejected_input_count_));
    status.add("rejected_outputs",
               static_cast<long long>(rejected_output_count_));
    status.add("lease_expirations",
               static_cast<long long>(lease_expiration_count_));
    status.add("loop_rate_hz", loop_actual_rate_);
    status.add("gimbal_state_topic", gimbal_state_topic_);
    status.add("gimbal_state_required", require_gimbal_state_for_control_);
    status.add("gimbal_state_fresh", have_gimbal_state_ &&
        (ros::WallTime::now() - gimbal_state_time_).toSec() <= gimbal_state_timeout_sec_);
  }

  void tick(const ros::TimerEvent&) {
    const ros::WallTime now = ros::WallTime::now();
    const double dt = (now - previous_time_).toSec();
    previous_time_ = now;
    if (dt > 1e-4 && dt < 1.0) {
      const double instantaneous_rate = 1.0 / dt;
      loop_actual_rate_ = loop_rate_initialized_
          ? 0.9 * loop_actual_rate_ + 0.1 * instantaneous_rate
          : instantaneous_rate;
      loop_rate_initialized_ = true;
    }
    tracker::NormalizedError error;
    follower::ControllerCommand controller;
    mavros_msgs::State flight_state;
    pod_msgs::GimbalState gimbal_state;
    bool arrival_fresh, fresh, controller_fresh, gimbal_state_fresh;
    bool flight_state_fresh;
    double input_age_sec = timeout_ + 1.0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      arrival_fresh = have_error_ && (now - error_time_).toSec() <= timeout_;
      fresh = arrival_fresh;
      if (have_error_) input_age_sec = (now - error_time_).toSec();
      error = error_;
      controller = controller_;
      controller_fresh = have_controller_ &&
          (now - controller_time_).toSec() <= controller_timeout_;
      flight_state = flight_state_;
      flight_state_fresh = have_flight_state_ &&
          (now - flight_state_time_).toSec() <= flight_state_timeout_;
      gimbal_state = gimbal_state_;
      gimbal_state_fresh = have_gimbal_state_ &&
          (now - gimbal_state_time_).toSec() <= gimbal_state_timeout_sec_;
    }
    const bool gimbal_state_ready = gimbal_state_fresh && gimbal_state.connected &&
        gimbal_state.stabilized && gimbal_state.attitude_valid &&
        !gimbal_state.limit_active && gimbal_state.fault_code.empty();
    const double wall_now = now.toSec();
    const bool lease_expired = control_guard_->leaseExpired(wall_now);
    if (lease_expired && active_) {
      active_ = false;
      core_.reset();
      ++lease_expiration_count_;
    }
    const bool control_authorized = control_guard_->authorized(wall_now);
    std::string platform_reason;
    const bool platform_ready = platformReady(
        controller_fresh, controller, flight_state_fresh, flight_state,
        &platform_reason);
    Output out;
    const ros::Time ros_now = ros::Time::now();
    if (arrival_fresh && !error.header.stamp.isZero() && !ros_now.isZero()) {
      const double stamped_age = (ros_now - error.header.stamp).toSec();
      if (stamped_age >= 0.0 && stamped_age <= 3600.0) {
        input_age_sec = stamped_age;
      }
    }
    // A tracker message is normally stamped at publication time for ROS
    // transport compatibility, while capture_timestamp identifies the actual
    // image sample.  Use both: input_age_ms remains the legacy publication
    // age, while capture_age drives the safety decision and diagnostics.
    bool capture_stamp_valid = false;
    double capture_age_sec = std::max(0.0, input_age_sec);
    bool capture_in_future = false;
    if (arrival_fresh && !error.capture_timestamp.isZero() && !ros_now.isZero()) {
      const double age = (ros_now - error.capture_timestamp).toSec();
      if (age >= -future_tolerance_sec_ && age <= 3600.0) {
        capture_stamp_valid = true;
        capture_age_sec = std::max(0.0, age);
      } else if (age < -future_tolerance_sec_) {
        capture_in_future = true;
      }
    }
    const double safe_capture_limit =
        std::min(capture_timeout_, latency_abort_);
    const bool latency_abort = capture_stamp_valid &&
        capture_age_sec > safe_capture_limit;
    const bool capture_required_but_missing =
        require_capture_timestamp_ && !capture_stamp_valid;
    fresh = arrival_fresh && !latency_abort &&
            !capture_required_but_missing && !capture_in_future;
    if (require_gimbal_state_for_control_ && !gimbal_state_ready) fresh = false;
    const bool latency_limited = capture_stamp_valid &&
        capture_age_sec > latency_warning_ && !latency_abort;
    const double latency_scale = latency_limited
        ? clamp((safe_capture_limit - capture_age_sec) /
                    std::max(1e-3, safe_capture_limit - latency_warning_),
                0.0, 1.0)
        : 1.0;
    const double inference_latency_ms =
        std::isfinite(error.inference_latency_ms)
            ? std::max(0.0, static_cast<double>(error.inference_latency_ms))
            : 0.0;
    const double transport_latency_ms = capture_stamp_valid
        ? std::max(0.0, capture_age_sec * 1000.0 - inference_latency_ms)
        : 0.0;
    const ros::WallTime compute_start = ros::WallTime::now();
    std::array<double, 3> relative_position{0.0, 0.0, 0.0};
    std::array<double, 3> relative_velocity{0.0, 0.0, 0.0};
    std::array<double, 9> position_covariance{0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                               0.0, 0.0, 0.0};
    std::array<double, 9> velocity_covariance{0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                               0.0, 0.0, 0.0};
    std::array<double, 4> tracker_state_covariance{0.0, 0.0, 0.0, 0.0};
    for (std::size_t i = 0; i < 3; ++i) {
      relative_position[i] = error.target_relative_position_body[i];
      relative_velocity[i] = error.target_relative_velocity_body[i];
    }
    for (std::size_t i = 0; i < 9; ++i) {
      position_covariance[i] = error.position_covariance[i];
      velocity_covariance[i] = error.velocity_covariance[i];
    }
    for (std::size_t i = 0; i < 4; ++i) {
      tracker_state_covariance[i] = error.state_covariance[i];
    }
    if (active_) {
      double error_x = fresh ? error.error_x : 0.0;
      double error_y = fresh ? error.error_y : 0.0;
      if (fresh && use_angular_error_ && error.has_angular_error &&
          std::isfinite(error.yaw_error_rad) && std::isfinite(error.pitch_error_rad)) {
        const double horizontal_half_fov =
            0.5 * camera_fov_horizontal_deg_ * kPi / 180.0;
        const double vertical_half_fov =
            0.5 * camera_fov_vertical_deg_ * kPi / 180.0;
        error_x = clamp(std::tan(error.yaw_error_rad) /
                            std::max(1e-6, std::tan(horizontal_half_fov)), -1.0, 1.0);
        error_y = clamp(std::tan(error.pitch_error_rad) /
                            std::max(1e-6, std::tan(vertical_half_fov)), -1.0, 1.0);
      }
      // SmartTracker marks tentative, predicted, and failed recovery samples
      // explicitly.  Empty association_method means an older tracker message,
      // so retain the established error_valid/confidence contract.
      const bool smart_tracker_sample = !error.association_method.empty();
      const bool control_ready = !smart_tracker_sample ||
          error.control_measurement_ready;
      const double control_confidence =
          use_tracker_control_confidence_ && smart_tracker_sample
              ? error.control_confidence : error.confidence;
      out = core_.compute(error_x, error_y,
                          fresh ? error.error_size : 0.0,
                          fresh ? error.yaw_error_rad : 0.0,
                          fresh ? error.pitch_error_rad : 0.0,
                          fresh && error.has_angular_error,
                          fresh && error.error_valid && control_ready,
                          fresh ? control_confidence : 0.0,
                          fresh && error.range_valid,
                          relative_position, relative_velocity,
                          position_covariance, velocity_covariance,
                          tracker_state_covariance,
                          error.association_method, error.fusion_status,
                          now.toSec(), dt);
      // A controller override is still subject to the profile capability
      // gate and must not revive a sample rejected by the capture-time safety
      // check.  It is constrained normally for fresh/legacy inputs.
      if (controller_fresh && !latency_abort) core_.constrain(out, controller);
      if (latency_limited) core_.limitForTrackerLatency(out, latency_scale);
      core_.finalize(out, now.toSec());
    }
    out.input_age_ms = std::max(0.0, input_age_sec * 1000.0);
    out.tracker_capture_age_ms = capture_stamp_valid
        ? std::max(0.0, capture_age_sec * 1000.0) : out.input_age_ms;
    out.tracker_inference_latency_ms = inference_latency_ms;
    out.tracker_transport_latency_ms = transport_latency_ms;
    out.latency_limited = latency_limited;
    out.latency_abort = latency_abort;
    out.output_backend = output_backend_name_;
    out.emergency = core_.emergencyActive();
    out.control_authorized = control_authorized;
    out.platform_ready = platform_ready;
    out.input_received = have_error_;
    out.capture_timestamp_valid = capture_stamp_valid;
    out.source_capture_timestamp = capture_stamp_valid
        ? error.capture_timestamp : ros::Time();

    if (core_.emergencyActive()) {
      out.control_state = "EMERGENCY_STOP";
      out.invalid_reason = "emergency stop is active";
    } else if (lease_expired) {
      out.control_state = "LEASE_EXPIRED";
      out.invalid_reason = "aircraft-follow control lease expired";
    } else if (!active_) {
      out.control_state = control_authorized ? "DISABLED" : "WAITING_FOR_LEASE";
      out.invalid_reason = control_authorized
          ? "follower is disabled" : "control lease is missing or expired";
    } else if (!control_authorized) {
      out.control_state = "WAITING_FOR_LEASE";
      out.invalid_reason = "control lease is missing or expired";
    } else if (!platform_ready) {
      out.control_state = "BLOCKED_PLATFORM";
      out.invalid_reason = platform_reason;
    } else if (require_gimbal_state_for_control_ && !gimbal_state_ready) {
      out.control_state = "GIMBAL_UNAVAILABLE";
      out.invalid_reason = "direct GimbalState is stale, limited, faulted, or unstabilized";
    } else if (!arrival_fresh) {
      out.control_state = "WAITING_FOR_INPUT";
      out.invalid_reason = "tracker input is stale or unavailable";
    } else if (capture_in_future) {
      out.control_state = "INPUT_REJECTED";
      out.invalid_reason = "tracker capture timestamp is in the future";
    } else if (capture_required_but_missing) {
      out.control_state = "INPUT_REJECTED";
      out.invalid_reason = "valid tracker capture timestamp is required";
    } else if (latency_abort) {
      out.control_state = "LATENCY_ABORT";
      out.invalid_reason = "capture-to-control latency exceeds abort limit";
    } else if (!out.profile_supported) {
      out.control_state = "PROFILE_REJECTED";
      out.invalid_reason = out.profile_rejection_reason;
    } else if (out.uncertainty_abort) {
      out.control_state = "UNCERTAINTY_ABORT";
      out.invalid_reason = "tracker uncertainty exceeds abort limit";
    } else if (out.lost) {
      out.control_state = "TARGET_LOST";
      out.invalid_reason = out.valid
          ? "bounded target-loss behavior is active" : "target measurement is unavailable";
    } else if (out.latency_limited || out.uncertainty_limited) {
      out.control_state = "DEGRADED";
      out.invalid_reason.clear();
    } else if (out.valid) {
      out.control_state = "FOLLOWING";
      out.invalid_reason.clear();
    } else {
      out.control_state = "HOLD";
      out.invalid_reason = "control core did not produce a valid command";
    }

    const bool input_time_gate = arrival_fresh && !capture_in_future &&
        !capture_required_but_missing && !latency_abort &&
        (!require_gimbal_state_for_control_ || gimbal_state_ready);
    if (!control_authorized || !platform_ready || !active_ ||
        !input_time_gate) {
      out.valid = false;
    }
    if (!out.valid && out.control_state != "TARGET_LOST") {
      ++rejected_output_count_;
    }
    out.compute_time_ms =
        (ros::WallTime::now() - compute_start).toSec() * 1000.0;
    publish(out);
    updater_.update();
  }
  void publish(const Output& out) {
    const ros::Time stamp = ros::Time::now();
    const bool command_allowed = active_ && out.valid &&
                                 out.control_authorized && out.platform_ready;
    const double safe_forward = command_allowed ? out.forward : 0.0;
    const double safe_right = command_allowed ? out.right : 0.0;
    const double safe_down = command_allowed ? out.down : 0.0;
    const double safe_yaw = command_allowed ? out.yaw : 0.0;
    const double safe_roll_rate = command_allowed ? out.roll_rate : 0.0;
    const double safe_pitch_rate = command_allowed ? out.pitch_rate : 0.0;
    const double safe_thrust = command_allowed ? out.thrust : 0.0;
    last_control_state_ = out.control_state;
    last_invalid_reason_ = out.invalid_reason;
    last_command_valid_ = command_allowed;
    last_control_authorized_ = out.control_authorized;
    last_platform_ready_ = out.platform_ready;
    follower::FollowerCommand cmd;
    cmd.header.stamp = stamp;
    cmd.control_mode = out.control_mode;
    cmd.lateral_guidance_mode = out.mode;
    cmd.follower_profile = out.profile;
    cmd.profile_supported = out.profile_supported;
    cmd.profile_rejection_reason = out.profile_rejection_reason;
    cmd.velocity_forward = safe_forward;
    cmd.velocity_right = safe_right;
    cmd.velocity_down = safe_down;
    cmd.yaw_rate_deg_s = safe_yaw;
    cmd.yaw_rate_raw_deg_s = out.raw_yaw;
    cmd.yaw_smoothing_active = out.yaw_smoothing;
    cmd.roll_rate_deg_s = safe_roll_rate;
    cmd.pitch_rate_deg_s = safe_pitch_rate;
    cmd.thrust = safe_thrust;
    cmd.command_valid = command_allowed;
    cmd.target_visible = out.visible;
    cmd.emergency_stop_active = out.emergency;
    cmd.adaptive_dive_climb_active = out.adaptive;
    cmd.adaptive_correction_down = out.adaptive_down;
    cmd.adaptive_correction_fwd = out.adaptive_forward;
    cmd.pitch_compensation_active = out.pitch_active;
    cmd.pitch_compensation_value = out.pitch_correction;
    cmd.target_error_x = out.error_x;
    cmd.target_error_y = out.error_y;
    cmd.target_lost = out.lost;
    cmd.target_loss_duration = out.loss_duration;
    cmd.pid_yaw_output = out.pid_yaw;
    cmd.pid_down_output = out.pid_down;
    cmd.pid_right_output = out.pid_right;
    cmd.metric_state_active = out.metric_state_active;
    cmd.uncertainty_scale = out.uncertainty_scale;
    cmd.uncertainty_limited = out.uncertainty_limited;
    cmd.uncertainty_abort = out.uncertainty_abort;
    cmd.command_sequence = ++command_sequence_;
    cmd.source_capture_timestamp = out.source_capture_timestamp;
    cmd.control_state = out.control_state;
    cmd.invalid_reason = out.invalid_reason;
    cmd.output_backend = out.output_backend;
    cmd.control_authorized = out.control_authorized;
    cmd.platform_ready = out.platform_ready;
    command_pub_.publish(cmd);

    follower::FollowerStatus status;
    status.header.stamp = stamp;
    status.following_active = active_;
    status.target_lost = out.lost;
    status.emergency_stop_active = out.emergency;
    status.control_mode = out.control_mode;
    status.lateral_guidance_mode = out.mode;
    status.follower_profile = out.profile;
    status.effective_profile = out.effective_profile;
    status.previous_profile = out.previous_profile;
    status.profile_fallback_active = out.profile_fallback_active;
    status.profile_transition_active = out.profile_transition_active;
    status.profile_supported = out.profile_supported;
    status.profile_rejection_reason = out.profile_rejection_reason;
    status.current_forward_velocity = out.current_forward;
    status.target_forward_velocity = core_.targetForward();
    status.lateral_velocity = out.right;
    status.vertical_velocity = out.down;
    status.yaw_rate = out.yaw;
    status.roll_rate = out.roll_rate;
    status.pitch_rate = out.pitch_rate;
    status.thrust = out.thrust;
    status.target_error_x = out.error_x;
    status.target_error_y = out.error_y;
    status.target_confidence = out.tracking_confidence;
    status.target_loss_duration = out.loss_duration;
    status.adaptive_active = out.adaptive;
    status.vertical_rate_error = out.vertical_rate_error;
    status.smoothed_vertical_rate = out.smoothed_vertical_rate;
    status.pitch_compensation_active = out.pitch_active;
    status.current_pitch_angle = core_.pitch();
    status.pitch_compensation_value = out.pitch_correction;
    status.altitude_current = core_.altitude();
    status.altitude_safe = out.altitude_safe;
    status.altitude_violation_count = core_.altitudeViolations();
    status.pid_yaw_output = out.pid_yaw;
    status.pid_down_output = out.pid_down;
    status.pid_right_output = out.pid_right;
    status.command_rate_hz = rate_;
    status.loop_actual_rate = loop_actual_rate_;
    status.input_age_ms = out.input_age_ms;
    status.compute_time_ms = out.compute_time_ms;
    status.end_to_end_latency_ms = out.tracker_capture_age_ms +
                                   out.compute_time_ms;
    status.update_count = out.sequence;
    status.tracker_capture_age_ms = out.tracker_capture_age_ms;
    status.tracker_inference_latency_ms = out.tracker_inference_latency_ms;
    status.tracker_transport_latency_ms = out.tracker_transport_latency_ms;
    status.latency_limited = out.latency_limited;
    status.latency_abort = out.latency_abort;
    status.relative_position_sigma = out.relative_position_sigma;
    status.relative_velocity_sigma = out.relative_velocity_sigma;
    status.uncertainty_scale = out.uncertainty_scale;
    status.metric_state_active = out.metric_state_active;
    status.uncertainty_limited = out.uncertainty_limited;
    status.uncertainty_abort = out.uncertainty_abort;
    status.control_state = out.control_state;
    status.invalid_reason = out.invalid_reason;
    status.output_backend = out.output_backend;
    status.control_authorized = out.control_authorized;
    status.platform_ready = out.platform_ready;
    status.input_received = out.input_received;
    status.capture_timestamp_valid = out.capture_timestamp_valid;
    status.lease_requester = control_guard_->leaseActive(
        ros::WallTime::now().toSec()) ? control_guard_->requester() : "";
    status.lease_remaining_sec = static_cast<float>(
        control_guard_->remaining(ros::WallTime::now().toSec()));
    status.rejected_input_count = rejected_input_count_;
    status.rejected_output_count = rejected_output_count_;
    status_pub_.publish(status);

    follower::ControllerFeedback feedback;
    feedback.header.stamp = stamp;
    feedback.suggested_velocity_forward = out.forward;
    feedback.suggested_velocity_right = out.right;
    feedback.suggested_velocity_down = out.down;
    feedback.suggested_yaw_rate_deg_s = out.yaw;
    feedback.suggestion_valid = cmd.command_valid;
    feedback.control_mode = out.control_mode;
    feedback.lateral_guidance_mode = out.mode;
    feedback.follower_profile = out.profile;
    feedback.profile_supported = out.profile_supported;
    feedback.target_error_x = out.error_x;
    feedback.target_error_y = out.error_y;
    feedback.target_visible = out.visible;
    feedback.emergency_stop_active = out.emergency;
    feedback.following_active = active_;
    feedback.follower_confidence = out.tracking_confidence;
    feedback_pub_.publish(feedback);

    geometry_msgs::TwistStamped velocity;
    velocity.header.stamp = stamp;
    velocity.header.frame_id = "base_link";
    velocity.twist.linear.x = safe_forward;
    velocity.twist.linear.y = -safe_right;
    velocity.twist.linear.z = -safe_down;
    velocity.twist.angular.z = safe_yaw * kPi / 180.0;
    const bool actuator_path_enabled = out.control_authorized &&
                                       out.platform_ready && active_;
    if (actuator_path_enabled &&
        output_backend_ == follower::OutputBackend::kMavrosTwistLegacy &&
        out.control_mode == "velocity_body") {
      mavros_pub_.publish(velocity);
    }
    if (actuator_path_enabled &&
        output_backend_ == follower::OutputBackend::kMavrosBody &&
        out.control_mode == "velocity_body") {
      mavros_msgs::PositionTarget target;
      target.header.stamp = stamp;
      target.header.frame_id = "base_link_frd";
      target.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
      target.type_mask =
          mavros_msgs::PositionTarget::IGNORE_PX |
          mavros_msgs::PositionTarget::IGNORE_PY |
          mavros_msgs::PositionTarget::IGNORE_PZ |
          mavros_msgs::PositionTarget::IGNORE_AFX |
          mavros_msgs::PositionTarget::IGNORE_AFY |
          mavros_msgs::PositionTarget::IGNORE_AFZ |
          mavros_msgs::PositionTarget::IGNORE_YAW;
      target.velocity.x = safe_forward;
      target.velocity.y = safe_right;
      target.velocity.z = safe_down;
      target.yaw_rate = safe_yaw * kPi / 180.0;
      mavros_raw_pub_.publish(target);
    }
    if (actuator_path_enabled && mrs_output_ &&
        out.control_mode == "velocity_body") {
      mrs_msgs::VelocityReferenceStamped reference;
      reference.header.stamp = stamp;
      reference.header.frame_id = mrs_frame_;
      reference.reference.velocity.x = safe_forward;
      reference.reference.velocity.y = -safe_right;
      reference.reference.velocity.z = -safe_down;
      reference.reference.heading_rate = safe_yaw * kPi / 180.0;
      reference.reference.use_altitude = false;
      reference.reference.use_heading = false;
      reference.reference.use_heading_rate = true;
      mrs_pub_.publish(reference);
    }
    const bool need_odom = odom_pub_.getNumSubscribers() > 0;
    const bool need_command_marker = marker_pub_.getNumSubscribers() > 0;
    const bool need_actual_marker = actual_marker_pub_.getNumSubscribers() > 0;
    if (need_odom || need_command_marker || need_actual_marker) {
      nav_msgs::Odometry odom;
      odom.header.stamp = stamp;
      odom.header.frame_id = "map";
      odom.child_frame_id = "follower_velocity_command";
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (have_pose_) {
          odom.header.frame_id = pose_.header.frame_id;
          odom.pose.pose = pose_.pose;
        } else {
          odom.pose.pose.orientation.w = 1.0;
        }
      }
      odom.twist.twist = velocity.twist;
      if (need_odom) odom_pub_.publish(odom);
      if (need_command_marker || need_actual_marker) {
        visualization_msgs::Marker marker;
        marker.header = odom.header;
        marker.id = 0;
        marker.action = visualization_msgs::Marker::DELETE;
        if (need_command_marker) {
          marker.ns = "follower_velocity_command";
          marker_pub_.publish(marker);
        }
        if (need_actual_marker) {
          marker.ns = "mavros_feedback_velocity";
          actual_marker_pub_.publish(marker);
        }
      }
    }
  }

  ros::NodeHandle nh_, private_nh_;
  FollowerCore core_;
  std::unique_ptr<follower::ControlGuard> control_guard_;
  double rate_{60.0}, timeout_{0.25}, controller_timeout_{0.25};
  double capture_timeout_{0.25}, latency_warning_{0.12}, latency_abort_{0.25};
  double future_tolerance_sec_{0.02}, flight_state_timeout_{0.5};
  double loop_actual_rate_{0.0};
  bool loop_rate_initialized_{false};
  bool use_angular_error_{true}, use_tracker_control_confidence_{true};
  bool require_capture_timestamp_{false}, reject_out_of_order_capture_{true};
  bool require_mavros_connected_{true}, require_mavros_armed_{true};
  bool require_controller_ready_for_mrs_{true};
  double camera_fov_horizontal_deg_{60.0}, camera_fov_vertical_deg_{45.0};
  std::string error_topic_, mavros_ns_, mavros_topic_, mavros_raw_topic_;
  std::string gimbal_state_topic_{"/pod/gimbal/state"};
  std::string mrs_topic_, mrs_frame_, required_mavros_mode_;
  std::string trusted_controller_name_, output_backend_name_;
  follower::OutputBackend output_backend_{follower::OutputBackend::kInvalid};
  bool mavros_output_{true}, mrs_output_{false}, active_{false};
  ros::Publisher command_pub_, status_pub_, feedback_pub_, odom_pub_;
  ros::Publisher marker_pub_, actual_marker_pub_, mavros_pub_, mavros_raw_pub_;
  ros::Publisher mrs_pub_;
  ros::Subscriber error_sub_, imu_sub_, altitude_sub_, pose_sub_, velocity_sub_;
  ros::Subscriber gimbal_state_sub_;
  ros::Subscriber vfr_hud_sub_;
  ros::Subscriber state_sub_, controller_sub_;
  ros::ServiceServer start_srv_, stop_srv_, emergency_srv_, mode_srv_, profile_srv_;
  ros::ServiceServer lease_srv_;
  ros::Timer timer_;
  diagnostic_updater::Updater updater_;
  std::mutex mutex_;
  tracker::NormalizedError error_;
  pod_msgs::GimbalState gimbal_state_;
  follower::ControllerCommand controller_;
  mavros_msgs::State flight_state_;
  geometry_msgs::PoseStamped pose_;
  geometry_msgs::TwistStamped velocity_;
  bool have_error_{false}, have_controller_{false};
  bool have_gimbal_state_{false};
  bool have_flight_state_{false};
  bool have_pose_{false}, have_velocity_{false};
  ros::WallTime error_time_, controller_time_, flight_state_time_, gimbal_state_time_, previous_time_;
  ros::Time last_capture_timestamp_;
  std::uint64_t command_sequence_{0};
  std::uint64_t rejected_input_count_{0}, rejected_output_count_{0};
  std::uint64_t lease_expiration_count_{0};
  std::string last_control_state_{"DISABLED"};
  std::string last_invalid_reason_{"follower is disabled"};
  bool last_command_valid_{false}, last_control_authorized_{false};
  bool last_platform_ready_{false};
  double gimbal_state_timeout_sec_{0.15};
  bool require_gimbal_state_for_control_{false};
};
}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "follower_node");
  FollowerNode node;
  // A single callback queue keeps controller state updates deterministic and
  // avoids concurrent mutation of PID/filter state.
  ros::spin();
  return 0;
}
