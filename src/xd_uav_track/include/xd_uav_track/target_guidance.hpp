#pragma once

#include <array>
#include <string>

namespace xd_uav_track {

// This is deliberately independent of a vehicle profile.  A multicopter or
// fixed-wing follower may select the same target guidance policy; the profile
// only decides how the resulting velocity vector is consumed downstream.
enum class TargetGuidanceMode {
  kLegacyVisual,
  kMetricPursuit,
  kMetricOrbit,
  kAuto,
};

enum class OrbitDirection {
  kAuto = 0,
  kClockwise = -1,
  kCounterClockwise = 1,
};

struct TargetGuidanceConfig {
  bool enabled{false};
  TargetGuidanceMode mode{TargetGuidanceMode::kLegacyVisual};
  std::string observation_policy{"metric_preferred"};
  double metric_timeout_sec{0.30};
  double maximum_position_sigma_m{12.0};

  double orbit_radius_m{80.0};
  OrbitDirection orbit_direction{OrbitDirection::kAuto};
  double orbit_capture_hysteresis_m{8.0};
  double minimum_turn_radius_m{45.0};
  double radial_gain{0.30};
  double radial_damping{0.65};
  bool target_velocity_feedforward{true};
  bool publish_course_rate_feedforward{false};
  // Expose the inertial target centre to the fixed-wing position adapter.
  // This is an internal TrackVelocity field; ROS output messages remain
  // unchanged and velocity-only profiles keep their legacy behavior.
  bool publish_position_reference{false};

  // The fixed-wing vector adapter uses this as its maximum allowable course
  // offset.  Multirotor users can leave it at the default and consume the
  // complete vector directly.
  double maximum_course_offset{0.70};
  // Rate limit applied to the requested course offset. Zero disables the
  // limiter; fixed-wing profiles normally leave it enabled.
  double maximum_course_rate_radps{0.35};
  double commanded_speed{15.0};
  double maximum_lateral_speed{2.0};
  double maximum_vertical_speed{1.5};
  bool metric_vertical_control{false};
  bool wind_feasibility_enabled{false};

  // Optional image-plane guard for a fixed forward camera. Error coordinates
  // are normalized to [-1, 1]; when the target enters the margin, orbit mode
  // temporarily becomes metric pursuit so the airframe reacquires visibility.
  bool fixed_camera_fov_constraint_enabled{false};
  double fixed_camera_fov_margin_ratio{0.10};
  // Once a metric orbit has been captured, a temporary fixed-camera edge
  // excursion must not replace the inertial orbit by a radial pursuit.  The
  // target may be outside the image for a few frames; TrackController's
  // bounded coast prediction keeps the tangent command alive.
  bool fixed_camera_preserve_orbit_on_fov_loss{false};
  // In automatic orbit-direction mode, project both feasible coordinated
  // turns through the calibrated camera and latch the direction with the
  // larger future image margin. This is especially useful for a rigid
  // downward/oblique camera, where bank angle moves the target substantially.
  bool fixed_camera_visibility_direction_selection_enabled{false};
  // Optional runtime FOV contract for a gimbal source.  The detector keeps
  // normalized image coordinates, while this gate uses the live GimbalStatus
  // FOV/zoom to trigger an early recenter before the target is clipped.
  bool dynamic_gimbal_fov_enabled{false};
  double dynamic_gimbal_fov_margin_ratio{0.10};
  double dynamic_gimbal_min_fov_rad{0.10};
  // Optional zoom-dependent guard. A zoom ratio at or below the reference
  // contributes no extra margin; higher ratios linearly enlarge the protected
  // band by this gain. The live GimbalStatus FOV remains authoritative.
  double dynamic_gimbal_zoom_reference_ratio{1.0};
  double dynamic_gimbal_zoom_margin_gain{0.05};
  bool predictive_fov_enabled{false};
  double camera_fx{0.0}, camera_fy{0.0}, camera_cx{0.0}, camera_cy{0.0};
  double camera_width{640.0}, camera_height{480.0};
  // Optical camera -> body FLU rotation and camera origin in body FLU.
  std::array<double, 3> camera_rpy{{0.0, 0.0, 0.0}};
  std::array<double, 3> camera_translation{{0.0, 0.0, 0.0}};
  double fov_prediction_sec{0.5};
  // If enabled, derive the minimum feasible orbit radius from airspeed and
  // bank angle: R >= V^2 / (g tan(phi)).
  bool fixed_wing_kinematics_enabled{false};
  double fixed_wing_airspeed_mps{15.0};
  double fixed_wing_maximum_bank_angle_deg{35.0};
  double gravity_mps2{9.81};
  bool fixed_wing_entry_altitude_hold_enabled{false};
  // Offset from the aircraft altitude at the moment metric entry guidance is
  // first accepted.  This replaces the old target-relative-down heuristic:
  // target depth is not an aircraft altitude measurement.
  double fixed_wing_entry_altitude_offset_m{0.0};
  // Optional local-frame altitude reference. A non-negative value is used
  // instead of the first metric capture altitude. This prevents a target
  // first detected during catapult climb from permanently latching a low,
  // camera-FOV-limiting orbit. Negative retains legacy entry-altitude latching.
  double fixed_wing_entry_altitude_reference_m{-1.0};
  // Kept for YAML/API compatibility with older configurations.  It is no
  // longer used to command fixed-wing altitude.
  double fixed_wing_entry_target_down_m{0.0};
  double fixed_wing_altitude_gain{0.35};

  // A short metric coast period is useful for a moving target, but a stale
  // measurement must never be silently treated as a fresh range.
  double target_loss_coast_sec{1.0};
  double target_loss_orbit_sec{5.0};
  double prediction_process_sigma_mps{1.0};
  // Lower process noise for the longer orbit-hold interval. This keeps a
  // bounded coast usable for a complete fixed-wing turn.
  double target_loss_hold_process_sigma_mps{0.20};
  // Optional bounded phase after target_loss_orbit_sec. The target position
  // is frozen in the inertial frame and the tracker remains the orbit owner,
  // preventing the downstream timeout loiter from inventing a new centre at
  // the aircraft position. It is disabled by default and requires valid,
  // fresh vehicle poses throughout the hold.
  bool target_loss_center_hold_enabled{false};
  double target_loss_center_hold_sec{0.0};
  double target_loss_center_hold_process_sigma_mps{0.05};
  // Freshness bound for the aircraft pose used to re-express an inertial
  // coast. This is intentionally independent from detector freshness: a
  // camera callback may stall briefly while odometry remains safe to use.
  double inertial_coast_state_timeout_sec{0.30};

  // Optional world-frame CV filter knobs.  The filter is deliberately light
  // weight (no extra ROS message or thread) and is used to gate/smooth metric
  // reacquisition before it can move a captured orbit centre.
  bool world_filter_enabled{false};
  std::string world_filter_model{"cv"};
  double world_filter_position_alpha{0.30};
  double world_filter_max_innovation_m{18.0};
  double world_filter_max_speed_mps{40.0};
  double world_filter_process_accel_stddev_mps2{1.5};
  double world_filter_mahalanobis_gate{16.27};
  double world_filter_imm_static_accel_stddev_mps2{0.20};
  double world_filter_imm_maneuver_accel_stddev_mps2{4.0};
  double world_filter_imm_min_probability{0.02};
  // Three-model IMM tuning.  The models are stationary, constant-velocity
  // and coordinated-turn respectively.  The values below are persistence
  // probabilities for the Markov transition matrix diagonal.
  double world_filter_imm_turn_rate_stddev_radps2{0.35};
  double world_filter_imm_static_persistence{0.96};
  double world_filter_imm_cv_persistence{0.92};
  double world_filter_imm_turn_persistence{0.90};
  double world_filter_imm_max_turn_rate_radps{1.20};
  bool world_filter_oosm_enabled{false};
  double world_filter_oosm_window_sec{0.50};
  double world_filter_oosm_covariance_inflation{4.0};
  // Vehicle poses are retained in the same capture-time clock domain so a
  // delayed body-frame measurement is transformed using the aircraft pose at
  // exposure, not the pose at callback arrival.
  double world_filter_pose_history_sec{2.0};
  bool multi_source_fusion_enabled{false};
  double multi_source_max_age_sec{0.50};
};

struct TargetGuidanceInput {
  double now{0.0};
  double dt{0.0};
  double observation_time{0.0};
  double image_error_x{0.0};
  double image_error_y{0.0};
  bool metric_valid{false};
  bool range_valid{false};
  std::array<double, 3> relative_position_body_frd{{0.0, 0.0, 0.0}};
  bool relative_velocity_valid{false};
  std::array<double, 3> relative_velocity_body_frd{{0.0, 0.0, 0.0}};
  // Optional LOS from a stabilized gimbal. +yaw is right and is converted to
  // body-FLU left by the guidance implementation. It remains useful when a
  // laser range measurement is temporarily unavailable.
  bool bearing_valid{false};
  double bearing_yaw{0.0};
  double bearing_pitch{0.0};
  double position_sigma_m{0.0};
  double commanded_speed{0.0};
  bool vehicle_altitude_valid{false};
  double vehicle_altitude_m{0.0};
  double vehicle_roll{0.0};
  double vehicle_pitch{0.0};
  bool wind_valid{false};
  double wind_forward{0.0}, wind_left{0.0};
  // Source-dependent guidance is intentionally internal metadata.  It lets a
  // fixed camera enforce its image FOV while a gimbal source uses soft-limit
  // recentering instead of the fixed-camera reacquisition rule.
  bool source_is_fixed_camera{true};
  bool source_is_gimbal{false};
  double gimbal_yaw_return{0.0};
  double gimbal_pitch_return{0.0};
  bool gimbal_fov_valid{false};
  double gimbal_horizontal_fov_rad{0.0};
  double gimbal_vertical_fov_rad{0.0};
  double gimbal_zoom_ratio{0.0};
};

struct TargetGuidanceOutput {
  bool active{false};
  bool metric_active{false};
  bool orbit_active{false};
  bool range_accepted{false};
  double forward{0.0};
  double left{0.0};
  double up{0.0};
  double course_rate{0.0};
  double course_offset{0.0};
  double effective_radius_m{0.0};
  double radial_error_m{0.0};
  std::string mode{"legacy_visual"};
  std::string state{"visual"};
  std::string invalid_reason;
};

bool parseTargetGuidanceMode(const std::string& value,
                             TargetGuidanceMode* mode);
const char* targetGuidanceModeName(TargetGuidanceMode mode);

bool parseOrbitDirection(const std::string& value, OrbitDirection* direction);
const char* orbitDirectionName(OrbitDirection direction);

// Reusable high-level guidance core. It only produces a body-FLU velocity
// vector and never commands roll, throttle or attitude directly.
class TargetGuidance {
 public:
  explicit TargetGuidance(const TargetGuidanceConfig& config =
                              TargetGuidanceConfig());

  TargetGuidanceOutput update(const TargetGuidanceInput& input);
  void reset();

 private:
  static double clamp(double value, double minimum, double maximum);
  static double wrapAngle(double angle);
  static double vectorNorm(double x, double y);
  static bool finiteInput(const TargetGuidanceInput& input);

  TargetGuidanceConfig config_;
  int orbit_sign_{1};
  bool orbit_direction_initialized_{false};
  bool orbit_active_{false};
  double last_radial_error_{0.0};
  double previous_time_{0.0};
  double previous_course_offset_{0.0};
  bool course_offset_initialized_{false};
  bool entry_altitude_initialized_{false};
  double entry_altitude_m_{0.0};
};

}  // namespace xd_uav_track
