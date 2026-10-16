#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <deque>
#include <stdexcept>
#include <memory>
#include <map>
#include <mutex>
#include <set>
#include <string>
#include <vector>

#include <geometry_msgs/TwistStamped.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <mavros_msgs/PositionTarget.h>
#include <nav_msgs/Odometry.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <boost/bind.hpp>
#include <std_msgs/String.h>
#include <std_srvs/SetBool.h>
#include <xd_uav_track/DetectionArray.h>
#include <xd_uav_track/FollowerCommand.h>
#include <xd_uav_track/GimbalStatus.h>
#include <xd_uav_track/GimbalState.h>
#include <xd_uav_track/MetricTarget.h>
#include <xd_uav_track/ManageVisualIdentity.h>
#include <xd_uav_track/SelectTrack.h>
#include <xd_uav_track/SetTargetMode.h>
#include <xd_uav_track/SetProfile.h>
#include <xd_uav_track/StartTracker.h>
#include <xd_uav_track/TargetPixelArray.h>
#include <xd_uav_track/TrackStateArray.h>
#include <xd_uav_track/TrackStatus.h>
#include <xd_uav_track/multi_track_manager.hpp>
#include <xd_uav_track/tracklet_stitcher.hpp>
#include <xd_uav_track/track_controller.hpp>
#include <xmlrpcpp/XmlRpcValue.h>

namespace {

void eulerFromQuaternion(const geometry_msgs::Quaternion& quaternion,
                         double* roll, double* pitch, double* yaw) {
  const double sin_roll = 2.0 *
      (quaternion.w * quaternion.x + quaternion.y * quaternion.z);
  const double cos_roll = 1.0 - 2.0 *
      (quaternion.x * quaternion.x + quaternion.y * quaternion.y);
  *roll = std::atan2(sin_roll, cos_roll);
  const double sin_pitch = 2.0 *
      (quaternion.w * quaternion.y - quaternion.z * quaternion.x);
  *pitch = std::abs(sin_pitch) >= 1.0
      ? std::copysign(1.5707963267948966, sin_pitch) : std::asin(sin_pitch);
  const double sin_yaw = 2.0 *
      (quaternion.w * quaternion.z + quaternion.x * quaternion.y);
  const double cos_yaw = 1.0 - 2.0 *
      (quaternion.y * quaternion.y + quaternion.z * quaternion.z);
  *yaw = std::atan2(sin_yaw, cos_yaw);
}

void loadPid(const ros::NodeHandle& nh, const std::string& prefix,
             xd_uav_track::PidConfig* pid) {
  nh.param(prefix + "/kp", pid->kp, pid->kp);
  nh.param(prefix + "/ki", pid->ki, pid->ki);
  nh.param(prefix + "/kd", pid->kd, pid->kd);
  nh.param(prefix + "/integral_limit", pid->integral_limit,
           pid->integral_limit);
}

bool loadDoubleArray(const ros::NodeHandle& nh, const std::string& key,
                     std::array<double, 3>* value) {
  if (value == nullptr) return false;
  XmlRpc::XmlRpcValue raw;
  if (!nh.getParam(key, raw) || raw.getType() != XmlRpc::XmlRpcValue::TypeArray ||
      raw.size() != 3) return false;
  for (int i = 0; i < 3; ++i) {
    if (raw[i].getType() != XmlRpc::XmlRpcValue::TypeDouble &&
        raw[i].getType() != XmlRpc::XmlRpcValue::TypeInt) return false;
    (*value)[static_cast<std::size_t>(i)] =
        raw[i].getType() == XmlRpc::XmlRpcValue::TypeDouble
            ? static_cast<double>(raw[i])
            : static_cast<int>(raw[i]);
  }
  return true;
}

void loadTargetGuidance(const ros::NodeHandle& nh,
                        xd_uav_track::TargetGuidanceConfig* config) {
  if (config == nullptr) return;
  const std::string p = "follower/target_guidance/";
  nh.param(p + "enabled", config->enabled, config->enabled);
  std::string mode = xd_uav_track::targetGuidanceModeName(config->mode);
  nh.param(p + "mode", mode, mode);
  if (!xd_uav_track::parseTargetGuidanceMode(mode, &config->mode)) {
    ROS_WARN("[xd_uav_track] unsupported target guidance mode '%s'; using legacy_visual",
             mode.c_str());
    config->mode = xd_uav_track::TargetGuidanceMode::kLegacyVisual;
  }
  nh.param(p + "observation/policy", config->observation_policy,
           config->observation_policy);
  nh.param(p + "observation/metric_timeout_sec", config->metric_timeout_sec,
           config->metric_timeout_sec);
  nh.param(p + "observation/max_position_sigma_m", config->maximum_position_sigma_m,
           config->maximum_position_sigma_m);
  nh.param(p + "orbit/radius_m", config->orbit_radius_m, config->orbit_radius_m);
  std::string direction = xd_uav_track::orbitDirectionName(config->orbit_direction);
  nh.param(p + "orbit/direction", direction, direction);
  if (!xd_uav_track::parseOrbitDirection(direction, &config->orbit_direction)) {
    ROS_WARN("[xd_uav_track] unsupported orbit direction '%s'; using auto",
             direction.c_str());
    config->orbit_direction = xd_uav_track::OrbitDirection::kAuto;
  }
  nh.param(p + "orbit/capture_hysteresis_m", config->orbit_capture_hysteresis_m,
           config->orbit_capture_hysteresis_m);
  nh.param(p + "orbit/minimum_turn_radius_m", config->minimum_turn_radius_m,
           config->minimum_turn_radius_m);
  nh.param(p + "orbit/radial_gain", config->radial_gain, config->radial_gain);
  nh.param(p + "orbit/radial_damping", config->radial_damping,
           config->radial_damping);
  nh.param(p + "orbit/target_velocity_feedforward",
           config->target_velocity_feedforward,
           config->target_velocity_feedforward);
  nh.param(p + "orbit/publish_course_rate_feedforward",
           config->publish_course_rate_feedforward,
           config->publish_course_rate_feedforward);
  nh.param(p + "orbit/publish_position_reference",
           config->publish_position_reference, config->publish_position_reference);
  nh.param(p + "maximum_course_offset", config->maximum_course_offset,
           config->maximum_course_offset);
  nh.param(p + "commanded_speed", config->commanded_speed,
           config->commanded_speed);
  nh.param(p + "maximum_lateral_speed", config->maximum_lateral_speed,
           config->maximum_lateral_speed);
  nh.param(p + "maximum_vertical_speed", config->maximum_vertical_speed,
           config->maximum_vertical_speed);
  nh.param(p + "fixed_camera/fov_constraint_enabled",
           config->fixed_camera_fov_constraint_enabled,
           config->fixed_camera_fov_constraint_enabled);
  nh.param(p + "fixed_camera/fov_margin_ratio", config->fixed_camera_fov_margin_ratio,
           config->fixed_camera_fov_margin_ratio);
  nh.param(p + "fixed_camera/preserve_orbit_on_fov_loss",
           config->fixed_camera_preserve_orbit_on_fov_loss,
           config->fixed_camera_preserve_orbit_on_fov_loss);
  nh.param(p + "fixed_camera/visibility_direction_selection_enabled",
           config->fixed_camera_visibility_direction_selection_enabled,
           config->fixed_camera_visibility_direction_selection_enabled);
  nh.param(p + "gimbal/dynamic_fov_enabled", config->dynamic_gimbal_fov_enabled,
           config->dynamic_gimbal_fov_enabled);
  nh.param(p + "gimbal/dynamic_fov_margin_ratio",
           config->dynamic_gimbal_fov_margin_ratio,
           config->dynamic_gimbal_fov_margin_ratio);
  nh.param(p + "gimbal/min_fov_rad", config->dynamic_gimbal_min_fov_rad,
           config->dynamic_gimbal_min_fov_rad);
  nh.param(p + "gimbal/zoom_reference_ratio", config->dynamic_gimbal_zoom_reference_ratio,
           config->dynamic_gimbal_zoom_reference_ratio);
  nh.param(p + "gimbal/zoom_margin_gain", config->dynamic_gimbal_zoom_margin_gain,
           config->dynamic_gimbal_zoom_margin_gain);
  nh.param(p + "predictive_fov/enabled", config->predictive_fov_enabled,
           config->predictive_fov_enabled);
  nh.param(p + "predictive_fov/fx", config->camera_fx, config->camera_fx);
  nh.param(p + "predictive_fov/fy", config->camera_fy, config->camera_fy);
  nh.param(p + "predictive_fov/cx", config->camera_cx, config->camera_cx);
  nh.param(p + "predictive_fov/cy", config->camera_cy, config->camera_cy);
  nh.param(p + "predictive_fov/width", config->camera_width, config->camera_width);
  nh.param(p + "predictive_fov/height", config->camera_height, config->camera_height);
  nh.param(p + "predictive_fov/horizon_sec", config->fov_prediction_sec,
           config->fov_prediction_sec);
  loadDoubleArray(nh, p + "predictive_fov/optical_to_body_rpy", &config->camera_rpy);
  loadDoubleArray(nh, p + "predictive_fov/translation", &config->camera_translation);
  nh.param(p + "fixed_wing/kinematics_enabled", config->fixed_wing_kinematics_enabled,
           config->fixed_wing_kinematics_enabled);
  nh.param(p + "fixed_wing/airspeed_mps", config->fixed_wing_airspeed_mps,
           config->fixed_wing_airspeed_mps);
  nh.param(p + "fixed_wing/maximum_bank_angle_deg",
           config->fixed_wing_maximum_bank_angle_deg,
           config->fixed_wing_maximum_bank_angle_deg);
  nh.param(p + "fixed_wing/gravity_mps2", config->gravity_mps2, config->gravity_mps2);
  nh.param(p + "fixed_wing/entry_altitude_hold_enabled",
           config->fixed_wing_entry_altitude_hold_enabled,
           config->fixed_wing_entry_altitude_hold_enabled);
  nh.param(p + "fixed_wing/entry_altitude_offset_m",
           config->fixed_wing_entry_altitude_offset_m,
           config->fixed_wing_entry_altitude_offset_m);
  nh.param(p + "fixed_wing/entry_altitude_reference_m",
           config->fixed_wing_entry_altitude_reference_m,
           config->fixed_wing_entry_altitude_reference_m);
  nh.param(p + "fixed_wing/entry_target_down_m", config->fixed_wing_entry_target_down_m,
           config->fixed_wing_entry_target_down_m);
  nh.param(p + "fixed_wing/altitude_gain", config->fixed_wing_altitude_gain,
           config->fixed_wing_altitude_gain);
  nh.param(p + "maximum_course_rate_radps", config->maximum_course_rate_radps,
           config->maximum_course_rate_radps);
  nh.param(p + "target_loss/coast_time_sec", config->target_loss_coast_sec,
           config->target_loss_coast_sec);
  nh.param(p + "target_loss/orbit_time_sec", config->target_loss_orbit_sec,
           config->target_loss_orbit_sec);
  nh.param(p + "target_loss/process_sigma_mps", config->prediction_process_sigma_mps,
           config->prediction_process_sigma_mps);
  nh.param(p + "target_loss/hold_process_sigma_mps",
           config->target_loss_hold_process_sigma_mps,
           config->target_loss_hold_process_sigma_mps);
  nh.param(p + "target_loss/center_hold_enabled",
           config->target_loss_center_hold_enabled,
           config->target_loss_center_hold_enabled);
  nh.param(p + "target_loss/center_hold_time_sec",
           config->target_loss_center_hold_sec,
           config->target_loss_center_hold_sec);
  nh.param(p + "target_loss/center_hold_process_sigma_mps",
           config->target_loss_center_hold_process_sigma_mps,
           config->target_loss_center_hold_process_sigma_mps);
  nh.param(p + "target_loss/inertial_coast_state_timeout_sec",
           config->inertial_coast_state_timeout_sec,
           config->inertial_coast_state_timeout_sec);
  nh.param(p + "metric_velocity_estimation/world_filter_enabled",
           config->world_filter_enabled, config->world_filter_enabled);
  nh.param(p + "metric_velocity_estimation/world_filter_model",
           config->world_filter_model, config->world_filter_model);
  nh.param(p + "metric_velocity_estimation/position_filter_alpha",
           config->world_filter_position_alpha,
           config->world_filter_position_alpha);
  nh.param(p + "metric_velocity_estimation/position_maximum_innovation_m",
           config->world_filter_max_innovation_m,
           config->world_filter_max_innovation_m);
  nh.param(p + "metric_velocity_estimation/maximum_speed_mps",
           config->world_filter_max_speed_mps, config->world_filter_max_speed_mps);
  nh.param(p + "metric_velocity_estimation/world_filter_process_accel_stddev_mps2",
           config->world_filter_process_accel_stddev_mps2,
           config->world_filter_process_accel_stddev_mps2);
  nh.param(p + "metric_velocity_estimation/world_filter_mahalanobis_gate",
           config->world_filter_mahalanobis_gate,
           config->world_filter_mahalanobis_gate);
  nh.param(p + "metric_velocity_estimation/world_filter_imm_static_accel_stddev_mps2",
           config->world_filter_imm_static_accel_stddev_mps2,
           config->world_filter_imm_static_accel_stddev_mps2);
  nh.param(p + "metric_velocity_estimation/world_filter_imm_maneuver_accel_stddev_mps2",
           config->world_filter_imm_maneuver_accel_stddev_mps2,
           config->world_filter_imm_maneuver_accel_stddev_mps2);
  nh.param(p + "metric_velocity_estimation/world_filter_imm_min_probability",
           config->world_filter_imm_min_probability,
           config->world_filter_imm_min_probability);
  nh.param(p + "metric_velocity_estimation/world_filter_imm_turn_rate_stddev_radps2",
           config->world_filter_imm_turn_rate_stddev_radps2,
           config->world_filter_imm_turn_rate_stddev_radps2);
  nh.param(p + "metric_velocity_estimation/world_filter_imm_static_persistence",
           config->world_filter_imm_static_persistence,
           config->world_filter_imm_static_persistence);
  nh.param(p + "metric_velocity_estimation/world_filter_imm_cv_persistence",
           config->world_filter_imm_cv_persistence,
           config->world_filter_imm_cv_persistence);
  nh.param(p + "metric_velocity_estimation/world_filter_imm_turn_persistence",
           config->world_filter_imm_turn_persistence,
           config->world_filter_imm_turn_persistence);
  nh.param(p + "metric_velocity_estimation/world_filter_imm_max_turn_rate_radps",
           config->world_filter_imm_max_turn_rate_radps,
           config->world_filter_imm_max_turn_rate_radps);
  nh.param(p + "metric_velocity_estimation/world_filter_oosm_enabled",
           config->world_filter_oosm_enabled, config->world_filter_oosm_enabled);
  nh.param(p + "metric_velocity_estimation/world_filter_oosm_window_sec",
           config->world_filter_oosm_window_sec,
           config->world_filter_oosm_window_sec);
  nh.param(p + "metric_velocity_estimation/world_filter_oosm_covariance_inflation",
           config->world_filter_oosm_covariance_inflation,
           config->world_filter_oosm_covariance_inflation);
  nh.param(p + "metric_velocity_estimation/world_filter_pose_history_sec",
           config->world_filter_pose_history_sec,
           config->world_filter_pose_history_sec);
  nh.param(p + "metric_velocity_estimation/multi_source_fusion_enabled",
           config->multi_source_fusion_enabled,
           config->multi_source_fusion_enabled);
  nh.param(p + "metric_velocity_estimation/multi_source_max_age_sec",
           config->multi_source_max_age_sec, config->multi_source_max_age_sec);
  if (config->world_filter_enabled && config->world_filter_model != "cv" &&
      config->world_filter_model != "imm") {
    ROS_WARN("[xd_uav_track] unsupported world_filter_model='%s'; using CV",
             config->world_filter_model.c_str());
  }
  if (config->world_filter_oosm_enabled) {
    ROS_INFO("[xd_uav_track] OOSM replay enabled for a %.2fs window; older frames remain rejected",
             config->world_filter_oosm_window_sec);
  }
}

}  // namespace

class XdUavTrackNode {
 public:
  XdUavTrackNode() : private_nh_("~") {
    xd_uav_track::TrackControllerConfig config;
    private_nh_.param("tracker/input_timeout_sec", config.input_timeout_sec,
                      config.input_timeout_sec);
    private_nh_.param("tracker/minimum_confidence", config.minimum_confidence,
                      config.minimum_confidence);
    private_nh_.param("tracker/confidence_hysteresis",
                      config.confidence_hysteresis,
                      config.confidence_hysteresis);
    private_nh_.param("tracker/position_filter_alpha",
                      config.position_filter_alpha,
                      config.position_filter_alpha);
    private_nh_.param("tracker/velocity_filter_alpha",
                      config.velocity_filter_alpha,
                      config.velocity_filter_alpha);
    private_nh_.param("tracker/target_x", config.target_x, config.target_x);
    private_nh_.param("tracker/target_y", config.target_y, config.target_y);

    std::string profile_name{xd_uav_track::followerProfileName(config.profile)};
    private_nh_.param("follower/profile", profile_name, profile_name);
    if (profile_name == "fw_attitude_rate") {
      ROS_WARN("[xd_uav_track] legacy profile 'fw_attitude_rate' was removed; "
               "using 'fw_velocity_vector' (velocity-only fixed-wing guidance)");
      profile_name = "fw_velocity_vector";
    }
    if (!xd_uav_track::parseFollowerProfile(profile_name, &config.profile)) {
      ROS_WARN("[xd_uav_track] unsupported profile '%s'; using mc_velocity_chase",
               profile_name.c_str());
      config.profile = xd_uav_track::FollowerProfile::kVelocityChase;
    }
    std::string guidance_name{
        xd_uav_track::lateralGuidanceModeName(config.lateral_guidance)};
    private_nh_.param("follower/lateral_guidance_mode", guidance_name,
                      guidance_name);
    if (!xd_uav_track::parseLateralGuidanceMode(
            guidance_name, &config.lateral_guidance)) {
      ROS_WARN("[xd_uav_track] unsupported lateral guidance '%s'; using coordinated_turn",
               guidance_name.c_str());
      config.lateral_guidance =
          xd_uav_track::LateralGuidanceMode::kCoordinatedTurn;
    }
    lateral_guidance_name_ =
        xd_uav_track::lateralGuidanceModeName(config.lateral_guidance);
    std::string gm_chase_guidance{"coordinated_turn"};
    std::string gm_vector_guidance{"sideslip"};
    private_nh_.param("follower/gm_velocity_chase/lateral_guidance_mode",
                      gm_chase_guidance, gm_chase_guidance);
    private_nh_.param("follower/gm_velocity_vector/lateral_guidance_mode",
                      gm_vector_guidance, gm_vector_guidance);
    if (!xd_uav_track::parseLateralGuidanceMode(
            gm_chase_guidance, &config.gm_chase_lateral_guidance)) {
      config.gm_chase_lateral_guidance =
          xd_uav_track::LateralGuidanceMode::kCoordinatedTurn;
    }
    if (!xd_uav_track::parseLateralGuidanceMode(
            gm_vector_guidance, &config.gm_vector_lateral_guidance)) {
      config.gm_vector_lateral_guidance =
          xd_uav_track::LateralGuidanceMode::kSideslip;
    }
    private_nh_.param("follower/enable_vertical_control",
                      config.enable_vertical_control,
                      config.enable_vertical_control);
    private_nh_.param("follower/distance_enable_yaw",
                      config.distance_enable_yaw,
                      config.distance_enable_yaw);
    private_nh_.param("follower/horizontal_deadband",
                      config.horizontal_deadband,
                      config.horizontal_deadband);
    private_nh_.param("follower/vertical_deadband",
                      config.vertical_deadband,
                      config.vertical_deadband);
    loadPid(private_nh_, "follower/pid/lateral", &config.lateral_pid);
    loadPid(private_nh_, "follower/pid/vertical", &config.vertical_pid);
    loadPid(private_nh_, "follower/pid/yaw", &config.yaw_pid);
    loadPid(private_nh_, "follower/pid/ground_forward",
            &config.ground_forward_pid);
    private_nh_.param("follower/ground/altitude_scaling_enabled",
                      config.ground_altitude_scaling_enabled,
                      config.ground_altitude_scaling_enabled);
    private_nh_.param("follower/ground/attitude_compensation_enabled",
                      config.ground_attitude_compensation_enabled,
                      config.ground_attitude_compensation_enabled);
    private_nh_.param("follower/ground/base_adjustment_x",
                      config.ground_base_adjustment_x,
                      config.ground_base_adjustment_x);
    private_nh_.param("follower/ground/base_adjustment_y",
                      config.ground_base_adjustment_y,
                      config.ground_base_adjustment_y);
    private_nh_.param("follower/ground/altitude_factor",
                      config.ground_altitude_factor,
                      config.ground_altitude_factor);
    private_nh_.param("follower/ground/descend_to_target",
                      config.ground_descend_to_target,
                      config.ground_descend_to_target);
    private_nh_.param("follower/ground/target_altitude",
                      config.ground_target_altitude,
                      config.ground_target_altitude);
    private_nh_.param("follower/ground/maximum_descent_velocity",
                      config.ground_maximum_descent_velocity,
                      config.ground_maximum_descent_velocity);

    private_nh_.param("follower/chase/initial_forward_velocity",
                      config.initial_forward_velocity,
                      config.initial_forward_velocity);
    private_nh_.param("follower/chase/forward_velocity",
                      config.chase_forward_velocity,
                      config.chase_forward_velocity);
    private_nh_.param("follower/chase/forward_ramp_rate",
                      config.forward_ramp_rate,
                      config.forward_ramp_rate);
    private_nh_.param("follower/target_loss/forward_velocity",
                      config.target_loss_forward_velocity,
                      config.target_loss_forward_velocity);
    private_nh_.param("follower/target_loss/reset_after_sec",
                      config.target_loss_reset_sec,
                      config.target_loss_reset_sec);
    private_nh_.param("follower/limits/maximum_forward_velocity",
                      config.maximum_forward_velocity,
                      config.maximum_forward_velocity);
    private_nh_.param("follower/limits/maximum_reverse_velocity",
                      config.maximum_reverse_velocity,
                      config.maximum_reverse_velocity);
    private_nh_.param("follower/limits/maximum_lateral_velocity",
                      config.maximum_lateral_velocity,
                      config.maximum_lateral_velocity);
    private_nh_.param("follower/limits/maximum_vertical_velocity",
                      config.maximum_vertical_velocity,
                      config.maximum_vertical_velocity);
    private_nh_.param("follower/limits/maximum_yaw_rate",
                      config.maximum_yaw_rate,
                      config.maximum_yaw_rate);
    private_nh_.param("follower/smoothing/velocity/enabled",
                      config.velocity_smoothing_enabled,
                      config.velocity_smoothing_enabled);
    private_nh_.param("follower/smoothing/velocity/ema_alpha",
                      config.velocity_ema_alpha,
                      config.velocity_ema_alpha);
    private_nh_.param("follower/smoothing/yaw/enabled",
                      config.yaw_smoothing_enabled,
                      config.yaw_smoothing_enabled);
    private_nh_.param("follower/smoothing/yaw/deadzone",
                      config.yaw_deadzone, config.yaw_deadzone);
    private_nh_.param("follower/smoothing/yaw/maximum_acceleration",
                      config.yaw_max_acceleration,
                      config.yaw_max_acceleration);
    private_nh_.param("follower/smoothing/yaw/ema_alpha",
                      config.yaw_ema_alpha, config.yaw_ema_alpha);
    private_nh_.param("follower/smoothing/yaw/speed_scaling_enabled",
                      config.yaw_speed_scaling_enabled,
                      config.yaw_speed_scaling_enabled);
    private_nh_.param("follower/smoothing/yaw/minimum_speed",
                      config.yaw_min_speed, config.yaw_min_speed);
    private_nh_.param("follower/smoothing/yaw/maximum_speed",
                      config.yaw_max_speed, config.yaw_max_speed);
    private_nh_.param("follower/smoothing/yaw/low_speed_factor",
                      config.yaw_low_speed_factor,
                      config.yaw_low_speed_factor);

    private_nh_.param("follower/gimbal/input_timeout_sec",
                      config.gimbal_input_timeout_sec,
                      config.gimbal_input_timeout_sec);
    private_nh_.param("follower/gimbal/mount_type",
                      config.gimbal_mount_type,
                      config.gimbal_mount_type);
    private_nh_.param("follower/gimbal/angle_filter_previous_weight",
                      config.gimbal_angle_filter_previous_weight,
                      config.gimbal_angle_filter_previous_weight);
    private_nh_.param("follower/gimbal/angle_deadzone",
                      config.gimbal_angle_deadzone,
                      config.gimbal_angle_deadzone);
    private_nh_.param("follower/gimbal/yaw_offset",
                      config.gimbal_yaw_offset, config.gimbal_yaw_offset);
    private_nh_.param("follower/gimbal/pitch_offset",
                      config.gimbal_pitch_offset,
                      config.gimbal_pitch_offset);
    private_nh_.param("follower/gimbal/roll_offset",
                      config.gimbal_roll_offset, config.gimbal_roll_offset);
    private_nh_.param("follower/gimbal/invert_yaw",
                      config.gimbal_invert_yaw,
                      config.gimbal_invert_yaw);
    private_nh_.param("follower/gimbal/invert_pitch",
                      config.gimbal_invert_pitch,
                      config.gimbal_invert_pitch);
    private_nh_.param("follower/gimbal/invert_roll",
                      config.gimbal_invert_roll,
                      config.gimbal_invert_roll);
    private_nh_.param("follower/gm_velocity_chase/forward_mode",
                      config.gm_chase_forward_mode,
                      config.gm_chase_forward_mode);
    private_nh_.param("follower/gm_velocity_chase/base_forward_speed",
                      config.gm_chase_base_forward_speed,
                      config.gm_chase_base_forward_speed);
    private_nh_.param("follower/gm_velocity_chase/forward_acceleration",
                      config.gm_chase_forward_acceleration,
                      config.gm_chase_forward_acceleration);
    private_nh_.param("follower/gm_velocity_chase/pitch_velocity_scaling",
                      config.gm_chase_pitch_velocity_scaling,
                      config.gm_chase_pitch_velocity_scaling);
    private_nh_.param("follower/gm_velocity_chase/pitch_deadzone",
                      config.gm_chase_pitch_deadzone,
                      config.gm_chase_pitch_deadzone);
    private_nh_.param("follower/gm_velocity_chase/maximum_lateral_angle",
                      config.gm_chase_maximum_lateral_angle,
                      config.gm_chase_maximum_lateral_angle);
    private_nh_.param("follower/gm_velocity_chase/maximum_vertical_angle",
                      config.gm_chase_maximum_vertical_angle,
                      config.gm_chase_maximum_vertical_angle);
    private_nh_.param("follower/gm_velocity_vector/velocity",
                      config.gm_vector_velocity,
                      config.gm_vector_velocity);
    private_nh_.param("follower/gm_velocity_vector/ramp_acceleration",
                      config.gm_vector_ramp_acceleration,
                      config.gm_vector_ramp_acceleration);
    private_nh_.param("follower/gm_velocity_vector/enable_vertical_control",
                      config.gm_vector_enable_vertical_control,
                      config.gm_vector_enable_vertical_control);
    private_nh_.param("follower/gm_velocity_vector/yaw_rate_gain",
                      config.gm_vector_yaw_rate_gain,
                      config.gm_vector_yaw_rate_gain);

    loadPid(private_nh_, "follower/fw_velocity_vector/pid/course",
            &config.fw_course_pid);
    loadPid(private_nh_, "follower/fw_velocity_vector/pid/climb_rate",
            &config.fw_climb_rate_pid);
    private_nh_.param("follower/fw_velocity_vector/airspeed",
                      config.fw_commanded_airspeed,
                      config.fw_commanded_airspeed);
    private_nh_.param("follower/fw_velocity_vector/maximum_course_offset",
                      config.fw_maximum_course_offset,
                      config.fw_maximum_course_offset);
    private_nh_.param("follower/fw_velocity_vector/maximum_climb_rate",
                      config.fw_maximum_climb_rate,
                      config.fw_maximum_climb_rate);
    loadPid(private_nh_, "follower/gmfw_velocity_vector/pid/course",
            &config.gmfw_course_pid);
    loadPid(private_nh_, "follower/gmfw_velocity_vector/pid/climb_rate",
            &config.gmfw_climb_rate_pid);
    private_nh_.param("follower/gmfw_velocity_vector/airspeed",
                      config.gmfw_commanded_airspeed,
                      config.gmfw_commanded_airspeed);
    private_nh_.param("follower/gmfw_velocity_vector/maximum_course_offset",
                      config.gmfw_maximum_course_offset,
                      config.gmfw_maximum_course_offset);
    private_nh_.param("follower/gmfw_velocity_vector/maximum_climb_rate",
                      config.gmfw_maximum_climb_rate,
                      config.gmfw_maximum_climb_rate);
    private_nh_.param("follower/gmfw_velocity_vector/gimbal_horizontal_fov_rad",
                      config.gmfw_horizontal_fov_rad,
                      config.gmfw_horizontal_fov_rad);
    private_nh_.param("follower/gmfw_velocity_vector/gimbal_vertical_fov_rad",
                      config.gmfw_vertical_fov_rad,
                      config.gmfw_vertical_fov_rad);
    private_nh_.param("follower/gmfw_velocity_vector/gimbal_image_alignment_rad",
                      config.gmfw_image_alignment_rad,
                      config.gmfw_image_alignment_rad);
    // Allow the registered metric profiles to carry their own fixed-wing
    // limits while retaining fw_velocity_vector as the common fallback.
    if (xd_uav_track::isMetricFixedWingProfile(config.profile)) {
      const std::string profile_prefix = std::string("follower/") +
          xd_uav_track::followerProfileName(config.profile);
      loadPid(private_nh_, profile_prefix + "/pid/course",
              &config.fw_course_pid);
      loadPid(private_nh_, profile_prefix + "/pid/climb_rate",
              &config.fw_climb_rate_pid);
      private_nh_.param(profile_prefix + "/airspeed",
                        config.fw_commanded_airspeed,
                        config.fw_commanded_airspeed);
      private_nh_.param(profile_prefix + "/maximum_course_offset",
                        config.fw_maximum_course_offset,
                        config.fw_maximum_course_offset);
      private_nh_.param(profile_prefix + "/maximum_climb_rate",
                        config.fw_maximum_climb_rate,
                        config.fw_maximum_climb_rate);
    }
    private_nh_.param("safety/uncertainty/enabled", config.uncertainty_control_enabled,
                      config.uncertainty_control_enabled);
    private_nh_.param("safety/uncertainty/nominal_sigma_normalized",
                      config.uncertainty_nominal_sigma,
                      config.uncertainty_nominal_sigma);
    private_nh_.param("safety/uncertainty/slow_ratio", config.uncertainty_slow_ratio,
                      config.uncertainty_slow_ratio);
    private_nh_.param("safety/uncertainty/abort_ratio", config.uncertainty_abort_ratio,
                      config.uncertainty_abort_ratio);
    private_nh_.param("safety/uncertainty/minimum_scale",
                      config.uncertainty_minimum_scale,
                      config.uncertainty_minimum_scale);
    private_nh_.param("safety/uncertainty/yaw_scale", config.uncertainty_yaw_scale,
                      config.uncertainty_yaw_scale);
    private_nh_.param("safety/reidentification/initial_scale",
                      config.reidentification_initial_scale,
                      config.reidentification_initial_scale);
    private_nh_.param("safety/reidentification/recovery_sec",
                      config.reidentification_recovery_sec,
                      config.reidentification_recovery_sec);
    private_nh_.param("follower/relative_state/enabled",
                      config.relative_state_control_enabled,
                      config.relative_state_control_enabled);
    private_nh_.param("follower/relative_state/target_forward_m",
                      config.relative_target_forward,
                      config.relative_target_forward);
    private_nh_.param("follower/relative_state/target_down_m",
                      config.relative_target_down, config.relative_target_down);
    private_nh_.param("follower/relative_state/position_gain",
                      config.relative_position_gain,
                      config.relative_position_gain);
    private_nh_.param("follower/relative_state/velocity_feedforward",
                      config.relative_velocity_feedforward,
                      config.relative_velocity_feedforward);
    private_nh_.param("follower/relative_state/maximum_correction_mps",
                      config.relative_maximum_correction,
                      config.relative_maximum_correction);
    loadTargetGuidance(private_nh_, &config.target_guidance);
    if (xd_uav_track::isMetricFixedWingProfile(config.profile)) {
      config.target_guidance.enabled = true;
      config.target_guidance.mode =
          xd_uav_track::targetGuidanceModeForFollowerProfile(config.profile);
    }
    gimbal_timeout_sec_ = config.gimbal_input_timeout_sec;
    multi_source_fusion_enabled_ = config.target_guidance.multi_source_fusion_enabled;
    controller_.reset(new xd_uav_track::TrackController(config));
    requested_profile_ = config.profile;

    xd_uav_track::MultiTrackConfig multi_config;
    private_nh_.getParam("tracker/association/compatible_class_ids",
                         multi_config.compatible_class_ids);
    tracklet_stitch_config_.compatible_class_ids =
        multi_config.compatible_class_ids;
    private_nh_.param("tracker/selection/auto_select", auto_select_track_, true);
    std::string default_target_mode{"auto"};
    private_nh_.param("tracker/selection/default_mode", default_target_mode,
                      default_target_mode);
    if (default_target_mode == "all" || default_target_mode == "global") {
      auto_selection_suppressed_ = true;
      target_selection_mode_ = "all";
    } else if (default_target_mode == "follow") {
      auto_selection_suppressed_ = true;
      target_selection_mode_ = "follow";
    } else if (default_target_mode != "auto") {
      ROS_WARN("[xd_uav_track] unknown default target mode '%s'; using auto",
               default_target_mode.c_str());
    }
    private_nh_.param("tracker/image_source",
                      accepted_image_source_, std::string());
    private_nh_.getParam("tracker/sources/allowed", allowed_image_sources_);
    private_nh_.param("tracker/sources/maximum_count", maximum_sources_, 8);
    private_nh_.param("tracker/sources/idle_ttl_sec", source_idle_ttl_sec_, 30.0);
    if (maximum_sources_ < 1 || maximum_sources_ > 32 ||
        !std::isfinite(source_idle_ttl_sec_) || source_idle_ttl_sec_ < 1.0)
      throw std::invalid_argument("invalid tracker source count/idle TTL");
    private_nh_.param("tracker/source_policy", source_policy_,
                      std::string("auto_handover"));
    private_nh_.param("tracker/source_aware", source_identity_gate_enabled_,
                      true);
    private_nh_.param("tracker/global_identity/enabled",
                      global_identity_merge_enabled_, true);
    private_nh_.param("tracker/global_identity/cross_source_metric_enabled",
                      tracklet_stitch_config_.cross_source_metric_enabled,
                      tracklet_stitch_config_.cross_source_metric_enabled);
    private_nh_.param("tracker/global_identity/cross_source_max_time_delta_sec",
                      tracklet_stitch_config_.cross_source_max_time_delta_sec,
                      tracklet_stitch_config_.cross_source_max_time_delta_sec);
    private_nh_.param("tracker/global_identity/cross_source_max_distance_m",
                      tracklet_stitch_config_.cross_source_max_distance_m,
                      tracklet_stitch_config_.cross_source_max_distance_m);
    private_nh_.param(
        "tracker/global_identity/cross_source_max_position_variance_m2",
        tracklet_stitch_config_.cross_source_max_position_variance_m2,
        tracklet_stitch_config_.cross_source_max_position_variance_m2);
    private_nh_.param(
        "tracker/global_identity/cross_source_fusion_max_time_delta_sec",
        cross_source_fusion_max_time_delta_sec_, 0.05);
    private_nh_.param("tracker/tracklet_stitching/enabled",
                      tracklet_stitch_config_.enabled,
                      tracklet_stitch_config_.enabled);
    private_nh_.param("tracker/tracklet_stitching/archive_ttl_sec",
                      tracklet_stitch_config_.archive_ttl_sec,
                      tracklet_stitch_config_.archive_ttl_sec);
    private_nh_.param(
        "tracker/tracklet_stitching/locked_identity_archive_ttl_sec",
        tracklet_stitch_config_.locked_identity_archive_ttl_sec,
        tracklet_stitch_config_.locked_identity_archive_ttl_sec);
    private_nh_.param(
        "tracker/tracklet_stitching/locked_identity_min_appearance_cosine",
        tracklet_stitch_config_.locked_identity_min_appearance_cosine,
        tracklet_stitch_config_.locked_identity_min_appearance_cosine);
    private_nh_.param("tracker/tracklet_stitching/maximum_distance_m",
                      tracklet_stitch_config_.maximum_distance_m,
                      tracklet_stitch_config_.maximum_distance_m);
    private_nh_.param("tracker/tracklet_stitching/maximum_speed_mps",
                      tracklet_stitch_config_.maximum_speed_mps,
                      tracklet_stitch_config_.maximum_speed_mps);
    private_nh_.param("tracker/tracklet_stitching/maximum_velocity_delta_mps",
                      tracklet_stitch_config_.maximum_velocity_delta_mps,
                      tracklet_stitch_config_.maximum_velocity_delta_mps);
    private_nh_.param("tracker/tracklet_stitching/mahalanobis_gate_sq",
                      tracklet_stitch_config_.mahalanobis_gate_sq,
                      tracklet_stitch_config_.mahalanobis_gate_sq);
    private_nh_.param("tracker/tracklet_stitching/process_noise_m2_per_s2",
                      tracklet_stitch_config_.process_noise_m2_per_s2,
                      tracklet_stitch_config_.process_noise_m2_per_s2);
    private_nh_.param("tracker/tracklet_stitching/minimum_position_variance_m2",
                      tracklet_stitch_config_.minimum_position_variance_m2,
                      tracklet_stitch_config_.minimum_position_variance_m2);
    private_nh_.param("tracker/tracklet_stitching/minimum_appearance_cosine",
                      tracklet_stitch_config_.minimum_appearance_cosine,
                      tracklet_stitch_config_.minimum_appearance_cosine);
    private_nh_.param("tracker/tracklet_stitching/motion_only_max_gap_sec",
                      tracklet_stitch_config_.motion_only_max_gap_sec,
                      tracklet_stitch_config_.motion_only_max_gap_sec);
    private_nh_.param("tracker/tracklet_stitching/motion_only_max_residual_m",
                      tracklet_stitch_config_.motion_only_max_residual_m,
                      tracklet_stitch_config_.motion_only_max_residual_m);
    private_nh_.param(
        "tracker/tracklet_stitching/motion_only_max_velocity_delta_mps",
        tracklet_stitch_config_.motion_only_max_velocity_delta_mps,
        tracklet_stitch_config_.motion_only_max_velocity_delta_mps);
    private_nh_.param("tracker/tracklet_stitching/minimum_score",
                      tracklet_stitch_config_.minimum_score,
                      tracklet_stitch_config_.minimum_score);
    private_nh_.param("tracker/tracklet_stitching/minimum_score_margin",
                      tracklet_stitch_config_.minimum_score_margin,
                      tracklet_stitch_config_.minimum_score_margin);
    private_nh_.param("tracker/tracklet_stitching/confirmation_frames",
                      tracklet_stitch_confirmation_frames_, 3);
    private_nh_.param("tracker/tracklet_stitching/maximum_confirmation_gap_sec",
                      tracklet_stitch_max_confirmation_gap_sec_, 0.50);
    private_nh_.param("tracker/tracklet_stitching/maximum_identities",
                      tracklet_stitch_maximum_identities_, 512);
    tracklet_stitch_config_.archive_ttl_sec = std::max(
        0.0, tracklet_stitch_config_.archive_ttl_sec);
    tracklet_stitch_config_.locked_identity_archive_ttl_sec = std::max(
        tracklet_stitch_config_.archive_ttl_sec,
        tracklet_stitch_config_.locked_identity_archive_ttl_sec);
    tracklet_stitch_config_.maximum_distance_m = std::max(
        0.1, tracklet_stitch_config_.maximum_distance_m);
    tracklet_stitch_config_.maximum_speed_mps = std::max(
        0.1, tracklet_stitch_config_.maximum_speed_mps);
    tracklet_stitch_config_.maximum_velocity_delta_mps = std::max(
        0.1, tracklet_stitch_config_.maximum_velocity_delta_mps);
    tracklet_stitch_config_.mahalanobis_gate_sq = std::max(
        0.1, tracklet_stitch_config_.mahalanobis_gate_sq);
    tracklet_stitch_config_.process_noise_m2_per_s2 = std::max(
        0.0, tracklet_stitch_config_.process_noise_m2_per_s2);
    tracklet_stitch_config_.minimum_position_variance_m2 = std::max(
        1e-4, tracklet_stitch_config_.minimum_position_variance_m2);
    tracklet_stitch_config_.minimum_appearance_cosine = std::max(-1.0,
        std::min(1.0, tracklet_stitch_config_.minimum_appearance_cosine));
    tracklet_stitch_config_.motion_only_max_gap_sec = std::max(
        0.0, std::min(tracklet_stitch_config_.archive_ttl_sec,
                      tracklet_stitch_config_.motion_only_max_gap_sec));
    tracklet_stitch_config_.motion_only_max_residual_m = std::max(
        0.1, std::min(tracklet_stitch_config_.maximum_distance_m,
                      tracklet_stitch_config_.motion_only_max_residual_m));
    tracklet_stitch_config_.motion_only_max_velocity_delta_mps = std::max(
        0.1, std::min(tracklet_stitch_config_.maximum_velocity_delta_mps,
                      tracklet_stitch_config_.motion_only_max_velocity_delta_mps));
    tracklet_stitch_config_.locked_identity_min_appearance_cosine = std::max(
        tracklet_stitch_config_.minimum_appearance_cosine,
        std::min(1.0,
                 tracklet_stitch_config_.locked_identity_min_appearance_cosine));
    tracklet_stitch_config_.minimum_score = std::max(0.0,
        std::min(1.0, tracklet_stitch_config_.minimum_score));
    tracklet_stitch_config_.minimum_score_margin = std::max(0.0,
        std::min(1.0, tracklet_stitch_config_.minimum_score_margin));
    tracklet_stitch_config_.cross_source_max_time_delta_sec = std::max(
        0.0, tracklet_stitch_config_.cross_source_max_time_delta_sec);
    tracklet_stitch_config_.cross_source_max_distance_m = std::max(
        0.1, tracklet_stitch_config_.cross_source_max_distance_m);
    tracklet_stitch_config_.cross_source_max_position_variance_m2 = std::max(
        1e-4, tracklet_stitch_config_.cross_source_max_position_variance_m2);
    cross_source_fusion_max_time_delta_sec_ = std::max(
        0.0, std::min(tracklet_stitch_config_.cross_source_max_time_delta_sec,
                      cross_source_fusion_max_time_delta_sec_));
    tracklet_stitch_confirmation_frames_ = std::max(
        2, std::min(10, tracklet_stitch_confirmation_frames_));
    tracklet_stitch_max_confirmation_gap_sec_ = std::max(
        0.05, tracklet_stitch_max_confirmation_gap_sec_);
    tracklet_stitch_maximum_identities_ = std::max(
        16, tracklet_stitch_maximum_identities_);
    private_nh_.param("tracker/source_cross_class_reject",
                      source_cross_class_reject_, true);
    private_nh_.param("tracker/source_identity_max_age_sec",
                      source_identity_max_age_sec_,
                      config.target_guidance.multi_source_max_age_sec);
    private_nh_.param("tracker/source_handover_distance_m",
                      source_identity_max_distance_m_, 25.0);
    private_nh_.param("tracker/source_handover_timeout_sec",
                      source_handover_timeout_sec_, 0.75);
    private_nh_.param("tracker/selection/locked_target_release_timeout_sec",
                      locked_target_release_timeout_sec_, 60.0);
    private_nh_.param("tracker/select_capture_tolerance_sec",
                      select_capture_tolerance_sec_, 0.10);
    private_nh_.param("tracker/reject_out_of_order_timestamps",
                      reject_out_of_order_timestamps_, true);
    private_nh_.param("tracker/source_clock_reset_jump_sec",
                      source_clock_reset_jump_sec_, 5.0);
    private_nh_.param("tracker/source_quality_handover/enabled",
                      source_quality_handover_enabled_, false);
    private_nh_.param("tracker/source_quality_handover/min_active_quality",
                      source_min_active_quality_, 0.25);
    private_nh_.param("tracker/source_quality_handover/min_incoming_quality",
                      source_min_incoming_quality_, 0.55);
    private_nh_.param("tracker/source_quality_handover/quality_margin",
                      source_quality_margin_, 0.10);
    private_nh_.param("tracker/source_quality_handover/cooldown_sec",
                      source_handover_cooldown_sec_, 1.0);
    private_nh_.param("tracker/lifecycle/confirmation_hits",
                      multi_config.confirmation_hits, multi_config.confirmation_hits);
    private_nh_.param("tracker/lifecycle/occlusion_frames",
                      multi_config.occlusion_frames, multi_config.occlusion_frames);
    private_nh_.param("tracker/lifecycle/removal_frames",
                      multi_config.removal_frames, multi_config.removal_frames);
    private_nh_.param("tracker/lifecycle/reference_fps",
                      multi_config.reference_fps, multi_config.reference_fps);
    private_nh_.param("tracker/lifecycle/occlusion_timeout_sec",
                      multi_config.occlusion_timeout_sec,
                      multi_config.occlusion_timeout_sec);
    private_nh_.param("tracker/lifecycle/removal_timeout_sec",
                      multi_config.removal_timeout_sec,
                      multi_config.removal_timeout_sec);
    private_nh_.param("tracker/lifecycle/reacquisition_after_sec",
                      multi_config.reacquisition_after_sec,
                      multi_config.reacquisition_after_sec);
    private_nh_.param("tracker/lifecycle/reacquisition_confirm_hits",
                      multi_config.reacquisition_confirm_hits,
                      multi_config.reacquisition_confirm_hits);
    private_nh_.param("tracker/lifecycle/reacquisition_mode",
                      multi_config.reacquisition_mode,
                      multi_config.reacquisition_mode);
    private_nh_.param("tracker/lifecycle/maximum_size_change_ratio",
                      multi_config.maximum_size_change_ratio,
                      multi_config.maximum_size_change_ratio);
    private_nh_.param("tracker/lifecycle/exit_edge_margin",
                      multi_config.exit_edge_margin,
                      multi_config.exit_edge_margin);
    private_nh_.param("tracker/limits/maximum_tracks",
                      multi_config.maximum_tracks, multi_config.maximum_tracks);
    private_nh_.param("tracker/limits/maximum_embedding_dimension",
                      multi_config.maximum_embedding_dimension,
                      multi_config.maximum_embedding_dimension);
    private_nh_.param("tracker/confidence/new_track",
                      multi_config.minimum_new_track_confidence,
                      multi_config.minimum_new_track_confidence);
    private_nh_.param("tracker/confidence/update",
                      multi_config.minimum_update_confidence,
                      multi_config.minimum_update_confidence);
    private_nh_.param("tracker/association/iou_threshold",
                      multi_config.association_iou_threshold,
                      multi_config.association_iou_threshold);
    private_nh_.param("tracker/association/center_distance",
                      multi_config.association_center_distance,
                      multi_config.association_center_distance);
    private_nh_.param("tracker/association/ambiguity_margin",
                      multi_config.association_ambiguity_margin,
                      multi_config.association_ambiguity_margin);
    private_nh_.param("tracker/association/metric_relative/enabled",
                      multi_config.metric_relative_association_enabled,
                      multi_config.metric_relative_association_enabled);
    private_nh_.param("tracker/association/metric_relative/maximum_distance_m",
                      multi_config.metric_relative_max_distance_m,
                      multi_config.metric_relative_max_distance_m);
    private_nh_.param("tracker/association/metric_relative/score_weight",
                      multi_config.metric_relative_score_weight,
                      multi_config.metric_relative_score_weight);
    private_nh_.param("tracker/association/metric_relative/smoothing_alpha",
                      multi_config.metric_relative_smoothing_alpha,
                      multi_config.metric_relative_smoothing_alpha);
    private_nh_.param("tracker/association/metric_relative/maximum_speed_mps",
                      multi_config.metric_relative_max_speed_mps,
                      multi_config.metric_relative_max_speed_mps);
    private_nh_.param("tracker/association/metric_relative/jump_tolerance_m",
                      multi_config.metric_relative_jump_tolerance_m,
                      multi_config.metric_relative_jump_tolerance_m);
    private_nh_.param("tracker/association/flexible_class_matching",
                      multi_config.flexible_class_matching,
                      multi_config.flexible_class_matching);
    private_nh_.param("tracker/association/class_history_size",
                      multi_config.class_history_size,
                      multi_config.class_history_size);
    private_nh_.param("tracker/association/search_expansion_rate_per_sec",
                      multi_config.search_expansion_rate_per_sec,
                      multi_config.search_expansion_rate_per_sec);
    private_nh_.param("tracker/association/search_expansion_cap",
                      multi_config.search_expansion_cap,
                      multi_config.search_expansion_cap);
    private_nh_.param("tracker/association/lenient_iou_scale",
                      multi_config.lenient_iou_scale,
                      multi_config.lenient_iou_scale);
    private_nh_.param("tracker/association/lenient_iou_floor",
                      multi_config.lenient_iou_floor,
                      multi_config.lenient_iou_floor);
    private_nh_.param("tracker/association/appearance_distance_gate_factor",
                      multi_config.appearance_distance_gate_factor,
                      multi_config.appearance_distance_gate_factor);
    private_nh_.param("tracker/association/appearance_cosine",
                      multi_config.appearance_minimum_cosine,
                      multi_config.appearance_minimum_cosine);
    private_nh_.param("tracker/association/high_confidence_threshold",
                      multi_config.high_confidence_threshold,
                      multi_config.high_confidence_threshold);
    private_nh_.param("tracker/association/appearance_gallery_size",
                      multi_config.appearance_gallery_size,
                      multi_config.appearance_gallery_size);
    private_nh_.param("tracker/association/confidence_smoothing_alpha",
                      multi_config.confidence_smoothing_alpha,
                      multi_config.confidence_smoothing_alpha);
    private_nh_.param("tracker/association/confidence_decay_rate",
                      multi_config.confidence_decay_rate,
                      multi_config.confidence_decay_rate);
    private_nh_.param("tracker/kalman/process_noise",
                      multi_config.process_noise, multi_config.process_noise);
    private_nh_.param("tracker/kalman/measurement_noise",
                      multi_config.measurement_noise, multi_config.measurement_noise);
    private_nh_.param("frames/body", body_frame_, std::string("base_link"));
    const auto separator = body_frame_.find_last_of('/');
    const std::string frd_default = separator == std::string::npos ? "base_link_frd" :
        body_frame_.substr(0, separator + 1) + "base_link_frd";
    private_nh_.param("frames/body_frd", multi_config.position_frame, frd_default);
    if (multi_config.position_frame.empty())
      throw std::invalid_argument("frames/body_frd must name the FRD position frame");
    multi_config_ = multi_config;

    private_nh_.param("runtime/publish_rate_hz", publish_rate_, 30.0);
    private_nh_.param("runtime/state_timeout_sec", state_timeout_sec_, 0.30);
    private_nh_.param("runtime/publish_control_reference",
                      publish_control_reference_, true);
    private_nh_.param("tracker/enabled_at_startup", tracker_active_, false);
    private_nh_.param("runtime/perception_only", perception_only_, false);
    if (perception_only_) {
      // A startup-only boundary, not a dynamically writable flight enable.
      // Conflicting YAML and legacy start commands cannot bypass it.
      publish_control_reference_ = false;
      tracker_active_ = false;
    }
    private_nh_.param("safety/profile_switch/minimum_interval_sec",
                      profile_switch_min_interval_sec_, 0.50);
    private_nh_.param("safety/profile_switch/blend_duration_sec",
                      profile_blend_duration_sec_, 0.50);
    private_nh_.param("safety/gimbal_fallback/enabled",
                      gimbal_fallback_enabled_, true);
    private_nh_.param("safety/emergency_stop_at_startup",
                      emergency_stop_active_, false);
    private_nh_.param("frames/body", body_frame_, std::string("base_link"));
    // Launch owns UAV namespace and interface wiring. These relative defaults
    // are only a namespace-neutral fallback for direct rosrun use.
    std::string state_topic{"state_estimator/main/odom"};
    std::string detections_topic{"track/detections"};
    std::string tracks_topic{"track/tracks"};
    std::string tracks_by_source_topic{"track/tracks_by_source"};
    std::string target_pixels_topic{"track/target_pixels_by_source"};
    std::string body_velocity_topic{"track/velocity_body"};
    std::string follower_command_topic{"track/command"};
    std::string gimbal_state_topic{"track/gimbal_state"};
    std::string gimbal_status_topic{"track/gimbal_status"};
    std::string metric_target_topic{"track/metric_target"};
    std::string status_topic{"track/status"};
    std::string target_mode_topic{"track/target_mode"};
    std::string reference_topic{"control/reference/setpoint"};
    private_nh_.param("interfaces/input/detections", detections_topic,
                      detections_topic);
    private_nh_.param("interfaces/output/tracks", tracks_topic, tracks_topic);
    private_nh_.param("interfaces/output/tracks_by_source", tracks_by_source_topic,
                      tracks_by_source_topic);
    private_nh_.param("interfaces/output/target_pixels_by_source",
                      target_pixels_topic, target_pixels_topic);
    private_nh_.param("interfaces/output/target_mode", target_mode_topic,
                      target_mode_topic);
    private_nh_.param("interfaces/output/body_velocity", body_velocity_topic,
                      body_velocity_topic);
    private_nh_.param("interfaces/output/follower_command", follower_command_topic,
                      follower_command_topic);
    private_nh_.param("interfaces/input/gimbal_state", gimbal_state_topic,
                      gimbal_state_topic);
    private_nh_.param("interfaces/input/gimbal_status", gimbal_status_topic,
                      gimbal_status_topic);
    private_nh_.param("interfaces/input/metric_target", metric_target_topic,
                      metric_target_topic);
    private_nh_.param("interfaces/output/status", status_topic, status_topic);
    private_nh_.param("interfaces/input/vehicle_state", state_topic, state_topic);
    private_nh_.param("interfaces/output/control_reference", reference_topic,
                      reference_topic);

    private_nh_.setCallbackQueue(&control_queue_);
    ros::SubscribeOptions detections_options =
        ros::SubscribeOptions::create<xd_uav_track::DetectionArray>(
            detections_topic, 1,
            boost::bind(&XdUavTrackNode::detectionsCallback, this, _1),
            ros::VoidConstPtr(), &perception_queue_);
    detections_subscriber_ = nh_.subscribe(detections_options);
    ros::SubscribeOptions state_options =
        ros::SubscribeOptions::create<nav_msgs::Odometry>(
            state_topic, 1,
            boost::bind(&XdUavTrackNode::stateCallback, this, _1),
            ros::VoidConstPtr(), &control_queue_);
    state_subscriber_ = nh_.subscribe(state_options);
    ros::SubscribeOptions gimbal_options =
        ros::SubscribeOptions::create<xd_uav_track::GimbalState>(
            gimbal_state_topic, 1,
            boost::bind(&XdUavTrackNode::gimbalStateCallback, this, _1),
            ros::VoidConstPtr(), &perception_queue_);
    gimbal_state_subscriber_ = nh_.subscribe(gimbal_options);
    ros::SubscribeOptions gimbal_status_options =
        ros::SubscribeOptions::create<xd_uav_track::GimbalStatus>(
            gimbal_status_topic, 1,
            boost::bind(&XdUavTrackNode::gimbalStatusCallback, this, _1),
            ros::VoidConstPtr(), &perception_queue_);
    gimbal_status_subscriber_ = nh_.subscribe(gimbal_status_options);
    if (!perception_only_ && !metric_target_topic.empty()) {
      ros::SubscribeOptions metric_target_options =
          ros::SubscribeOptions::create<xd_uav_track::MetricTarget>(
              metric_target_topic, 5,
              boost::bind(&XdUavTrackNode::metricTargetCallback, this, _1),
              ros::VoidConstPtr(), &control_queue_);
      metric_target_subscriber_ = nh_.subscribe(metric_target_options);
    }
    if (!perception_only_) {
      body_velocity_publisher_ = nh_.advertise<geometry_msgs::TwistStamped>(
          body_velocity_topic, 10);
      follower_command_publisher_ = nh_.advertise<xd_uav_track::FollowerCommand>(
          follower_command_topic, 10);
    }
    status_publisher_ = nh_.advertise<xd_uav_track::TrackStatus>(
        status_topic, 10);
    if (perception_only_)
      runtime_diagnostics_publisher_ = nh_.advertise<diagnostic_msgs::DiagnosticArray>(
          "track/runtime_diagnostics", 1);
    tracks_publisher_ = nh_.advertise<xd_uav_track::TrackStateArray>(
        tracks_topic, 10);
    tracks_by_source_publisher_ = nh_.advertise<xd_uav_track::TrackStateArray>(
        tracks_by_source_topic, 10);
    observations_publisher_ = nh_.advertise<xd_uav_track::DetectionArray>(
        "track/internal/observations_by_source",1);
    target_pixels_publisher_ = nh_.advertise<xd_uav_track::TargetPixelArray>(
        target_pixels_topic, 10);
    visual_tracks_publisher_ = nh_.advertise<xd_uav_track::TrackStateArray>(
        "track/visual_tracks_by_source", 10);
    visual_identity_service_ = private_nh_.advertiseService(
        "manage_visual_identity", &XdUavTrackNode::manageVisualIdentity, this);
    target_mode_publisher_ = nh_.advertise<std_msgs::String>(target_mode_topic, 1, true);
    std_msgs::String initial_target_mode;
    initial_target_mode.data = target_selection_mode_;
    target_mode_publisher_.publish(initial_target_mode);
    if (publish_control_reference_) {
      reference_publisher_ = nh_.advertise<mavros_msgs::PositionTarget>(
          reference_topic, 10);
    }
    profile_service_ = private_nh_.advertiseService(
        "set_profile", &XdUavTrackNode::setProfile, this);
    start_tracker_service_ = private_nh_.advertiseService(
        "start_tracker", &XdUavTrackNode::startTracker, this);
    select_track_service_ = private_nh_.advertiseService(
        "select_track", &XdUavTrackNode::selectTrack, this);
    set_target_mode_service_ = private_nh_.advertiseService(
        "set_target_mode", &XdUavTrackNode::setTargetMode, this);
    emergency_stop_service_ = private_nh_.advertiseService(
        "emergency_stop", &XdUavTrackNode::emergencyStop, this);
    ros::TimerOptions timer_options(
        ros::Duration(1.0 / std::max(2.0, publish_rate_)),
        boost::bind(&XdUavTrackNode::timerCallback, this, _1),
        &control_queue_, false);
    timer_ = nh_.createTimer(timer_options);
    perception_spinner_.reset(new ros::AsyncSpinner(1, &perception_queue_));
    control_spinner_.reset(new ros::AsyncSpinner(1, &control_queue_));
    perception_spinner_->start();
    control_spinner_->start();

    ROS_INFO("[xd_uav_track] runtime=%s at %.1f Hz; profile=%s; guidance=%s; reference=%s; tracking=%s",
             perception_only_ ? "perception_only" : "tracker + follower",
             publish_rate_,
             xd_uav_track::followerProfileName(controller_->profile()),
             lateral_guidance_name_.c_str(),
             publish_control_reference_ ? "enabled" : "disabled",
             tracker_active_ ? "active" : "waiting for StartTracker");
  }

 private:
  bool sourceAllowed(const std::string& source) const {
    return !source.empty() && source.size() <= 128U &&
        (accepted_image_source_.empty() || source == accepted_image_source_) &&
        (allowed_image_sources_.empty() ||
         std::find(allowed_image_sources_.begin(), allowed_image_sources_.end(),
                   source) != allowed_image_sources_.end());
  }

  void pruneSources(const ros::WallTime& now) {
    // Maintenance has its own bounded cadence, not a full archive scan on
    // every image/control tick. TTL cleanup jitter is at most 0.5 s plus the
    // configured timer period; association itself remains per observation.
    if (!last_storage_prune_.isZero() && (now - last_storage_prune_).toSec() < 0.5) return;
    last_storage_prune_ = now;
    for (auto it = source_last_receive_.begin(); it != source_last_receive_.end();) {
      if ((now - it->second).toSec() <= source_idle_ttl_sec_) { ++it; continue; }
      const auto source = it->first;
      clearSourceGlobalIds(source);
      source_trackers_.erase(source);
      source_last_stamp_.erase(source);
      latest_tracks_by_source_.erase(source);
      identity_detection_history_.erase(source);
      if (active_source_ == source) { active_source_.clear(); latest_tracks_ = {}; }
      it = source_last_receive_.erase(it);
    }
    pruneVisualIdentities();
    // Even a disconnected camera must not pin its archived entities forever.
    std::set<int> mapped_ids;
    for (const auto& entry : global_id_by_local_) mapped_ids.insert(entry.second);
    for (auto it = global_entities_.begin(); it != global_entities_.end();) {
      const double ttl = it->first == locked_global_id_
          ? tracklet_stitch_config_.locked_identity_archive_ttl_sec
          : tracklet_stitch_config_.archive_ttl_sec;
      if (!mapped_ids.count(it->first) && !visual_identities_.count(it->first) &&
          !it->second.last_seen_wall.isZero() &&
          (now - it->second.last_seen_wall).toSec() > ttl) {
        if (locked_global_id_ == it->first) locked_global_id_ = -1;
        it = global_entities_.erase(it);
      } else ++it;
    }
  }

  static bool captureStampFresh(const ros::Time& capture_stamp,
                                const ros::Time& now,
                                const double timeout_sec) {
    // A zero stamp is retained for legacy angle-only drivers.  Proper ROS
    // producers carry capture time, which lets us reject a delayed replay
    // even though its local transport arrival is recent.
    if (capture_stamp.isZero() || now.isZero()) return true;
    const double age = (now - capture_stamp).toSec();
    return std::isfinite(age) && age >= -0.05 &&
        age <= std::max(0.02, timeout_sec);
  }

  xd_uav_track::MultiTrackManager* trackerForSource(
      const std::string& source) {
    auto it = source_trackers_.find(source);
    if (it == source_trackers_.end()) {
      it = source_trackers_.emplace(
          source, std::unique_ptr<xd_uav_track::MultiTrackManager>(
                      new xd_uav_track::MultiTrackManager(multi_config_))).first;
    }
    return it->second.get();
  }

  struct GlobalEntity {
    int class_id{-1};
    std::string source;
    int local_track_id{-1};
    ros::Time stamp;
    ros::Time world_stamp;
    ros::WallTime last_seen_wall;
    std::array<double, 3> world{{0.0, 0.0, 0.0}};
    std::array<double, 3> world_variance{{1.0, 1.0, 1.0}};
    std::array<double, 3> world_velocity{{0.0, 0.0, 0.0}};
    bool has_world{false};
    bool has_world_velocity{false};
    bool established{false};
    bool provisional{true};
    bool stitch_ambiguous{false};
    int stitch_search_frames{0};
    std::uint32_t metric_observations{0};
    std::vector<float> appearance;
    std::vector<std::vector<float>> appearance_gallery;
  };

  struct PendingTrackletStitch {
    int identity_id{-1};
    int consecutive_hits{0};
    ros::Time last_stamp;
    double last_score{0.0};
  };

  struct VisualIdentity {
    std::string owner;
    std::string key;
    std::string source;
    xd_uav_track::TrackState latest;
    std::deque<xd_uav_track::TrackState> history;
    std::vector<float> signature;
    ros::WallTime updated;
    int candidate{-1};
    int confirmations{0};
    ros::Time confirmation_stamp;
    ros::Time bind_started_stamp;
    bool attached_detector{false};
    ros::Time detector_stamp;
  };

  static double boxIou(const xd_uav_track::TrackState& a,
                       const xd_uav_track::TrackState& b) {
    const double intersection = std::max(0, std::min(a.bbox[2], b.bbox[2])-
        std::max(a.bbox[0], b.bbox[0])) * static_cast<double>(std::max(0,
        std::min(a.bbox[3], b.bbox[3])-std::max(a.bbox[1], b.bbox[1])));
    const double aa = std::max(0, a.bbox[2]-a.bbox[0]) *
        static_cast<double>(std::max(0, a.bbox[3]-a.bbox[1]));
    const double ab = std::max(0, b.bbox[2]-b.bbox[0]) *
        static_cast<double>(std::max(0, b.bbox[3]-b.bbox[1]));
    return intersection / std::max(1.0, aa+ab-intersection);
  }

  void pruneVisualIdentities() {
    const auto now = ros::WallTime::now();
    for (auto it = visual_identities_.begin(); it != visual_identities_.end();) {
      if ((now-it->second.updated).toSec() >
          std::max(1.0, tracklet_stitch_config_.archive_ttl_sec))
        it = visual_identities_.erase(it);
      else ++it;
    }
  }

  void publishVisualTracks(const std::string& source,
                          const std_msgs::Header& header) {
    xd_uav_track::TrackStateArray output;
    output.model_epoch = model_epoch_;
    output.class_names = model_class_names_;
    output.header = header;
    output.image_source = source;
    for (const auto& entry : visual_identities_) {
      if (entry.second.source != source || entry.second.attached_detector) continue;
      auto track = entry.second.latest;
      // Evidence belongs to one capture. Old boxes are status, never new measurements.
      if (track.header.stamp != header.stamp) {
        track.lifecycle_state = "lost";
        track.detected = false;
        track.predicted = true;
        track.control_measurement_ready = false;
      }
      output.tracks.push_back(track);
    }
    visual_tracks_publisher_.publish(output);
  }

  bool manageVisualIdentity(xd_uav_track::ManageVisualIdentity::Request& request,
                            xd_uav_track::ManageVisualIdentity::Response& response) {
    std::lock_guard<std::mutex> lock(mutex_);
    pruneVisualIdentities();
    response.global_id = -1;
    const auto fail = [&](const std::string& reason) {
      if (request.action == "bind" && reason != "duplicate bind timestamp") {
        const auto owned = visual_identities_.find(request.global_id);
        if (owned != visual_identities_.end() && owned->second.owner == request.owner_session &&
            owned->second.source == request.image_source) {
          owned->second.confirmations = 0;
          owned->second.candidate = -1;
        }
      }
      response.success = false; response.message = reason; return true;
    };
    const bool needs_observation = request.action != "detach" &&
        request.action != "reset_bind" && request.action != "retire";
    if (request.owner_session.empty() || request.owner_session.size() > 128U ||
        request.image_source.empty() || request.image_source.size() > 128U ||
        request.registration_key.size() > 128U ||
        request.image_signature.size() > 512U || (needs_observation && (
        request.observation.header.stamp.isZero() ||
        request.observation.bbox[0] < 0 || request.observation.bbox[1] < 0 ||
        request.observation.bbox[2] > 100000 || request.observation.bbox[3] > 100000 ||
        request.observation.bbox[2] <= request.observation.bbox[0] ||
        request.observation.bbox[3] <= request.observation.bbox[1])))
      return fail("invalid visual identity observation");
    if (!sourceAllowed(request.image_source))
      return fail("source is not allowed");
    int id = request.global_id;
    if (request.action == "attach") {
      // Adopt an existing public entity; selecting it must not allocate an ID.
      const auto existing = visual_identities_.find(id);
      if (existing != visual_identities_.end()) {
        if (existing->second.owner != request.owner_session ||
            existing->second.source != request.image_source)
          return fail("visual identity ownership mismatch");
        response.success = true; response.global_id = id;
        response.message = "already attached"; return true;
      }
      const auto history = identity_detection_history_.find(request.image_source);
      const xd_uav_track::TrackState* anchor = nullptr;
      if (history != identity_detection_history_.end())
        for (const auto& snapshot : history->second)
          if (snapshot.header.stamp == request.observation.header.stamp)
            for (const auto& track : snapshot.tracks)
              if (track.track_id == id && track.detected && !track.predicted &&
                  track.lifecycle_state == "confirmed" && track.confidence >= 0.5F &&
                  boxIou(track, request.observation) >= 0.5) anchor = &track;
      if (!anchor || !global_entities_.count(id)) return fail("confirmed capture anchor missing");
      if (visual_identities_.size() >= 32U || request.image_signature.empty())
        return fail("visual capacity or appearance unavailable");
      double energy = 0.0;
      for (const auto value : request.image_signature) {
        if (!std::isfinite(value)) return fail("invalid signature");
        energy += value * value;
      }
      if (energy < 1e-9) return fail("invalid signature");
      VisualIdentity identity;
      identity.owner = request.owner_session; identity.source = request.image_source;
      identity.signature = request.image_signature;
      identity.attached_detector = true;
      identity.detector_stamp = request.observation.header.stamp;
      identity.updated = ros::WallTime::now();
      visual_identities_[id] = identity;
    }
    if (request.action == "register") {
      if (request.registration_key.empty()) return fail("registration_key required");
      for (const auto& entry : visual_identities_)
        if (entry.second.owner == request.owner_session &&
            entry.second.key == request.registration_key) {
          response.success = true; response.global_id = entry.first;
          response.message = "already registered"; return true;
        }
      if (visual_identities_.size() >= 32U ||
          global_entities_.size() >= static_cast<std::size_t>(tracklet_stitch_maximum_identities_))
        return fail("identity capacity reached");
      if (request.image_signature.size() > 512U) return fail("signature too large");
      for (const auto value : request.image_signature)
        if (!std::isfinite(value)) return fail("invalid signature");
      id = next_global_id_++;
      VisualIdentity identity;
      identity.owner = request.owner_session;
      identity.key = request.registration_key;
      identity.source = request.image_source;
      identity.updated = ros::WallTime::now();
      identity.signature = request.image_signature;
      visual_identities_[id] = identity;
      GlobalEntity entity;
      entity.source = request.image_source;
      entity.stamp = request.observation.header.stamp;
      entity.last_seen_wall = identity.updated;
      entity.established = true;
      entity.provisional = false;
      global_entities_[id] = entity;
    }
    auto it = visual_identities_.find(id);
    if (it == visual_identities_.end() || it->second.owner != request.owner_session ||
        it->second.source != request.image_source)
      return fail("visual identity ownership mismatch");
    auto& identity = it->second;
    response.global_id = id;
    if (request.action == "retire") {
      // End an unselected ROI observer without deleting its public archive.
      identity.latest.lifecycle_state = "lost";
      identity.latest.detected = false; identity.latest.predicted = true;
      identity.latest.tracking_quality = 0.0F;
      identity.latest.control_measurement_ready = false;
      identity.candidate = -1; identity.confirmations = 0;
      publishVisualTracks(identity.source, identity.latest.header);
      response.success = true; response.message = "visual observer retired; identity retained";
      return true;
    }
    if (request.action == "detach") {
      if (!identity.attached_detector) return fail("registered visual identity cannot detach");
      visual_identities_.erase(it);
      response.success = true; response.message = "selected observer detached"; return true;
    }
    if (request.action == "reset_bind") {
      identity.candidate = -1;
      identity.confirmations = 0;
      response.success = true; response.message = "binding evidence interrupted";
      return true;
    }
    if (request.action == "bind") {
      const ros::Time stamp = request.observation.header.stamp;
      const auto source_history = identity_detection_history_.find(request.image_source);
      if (source_history == identity_detection_history_.end()) return fail("detector history missing");
      const xd_uav_track::TrackStateArray* snapshot = nullptr;
      for (const auto& item : source_history->second)
        if (item.header.stamp == stamp) snapshot = &item;
      const xd_uav_track::TrackState* visual = nullptr;
      for (const auto& item : identity.history)
        if (item.header.stamp == stamp) visual = &item;
      if (!snapshot || !visual || visual->lifecycle_state != "confirmed")
        return fail("same capture evidence missing");
      const xd_uav_track::TrackState* candidate = nullptr;
      double first = 0.0, second = 0.0;
      int best_id = -1;
      for (const auto& track : snapshot->tracks) {
        if (!track.detected || track.predicted || track.confidence < 0.5F ||
            track.lifecycle_state != "confirmed") continue;
        const double overlap = boxIou(*visual, track);
        if (overlap > first) { second = first; first = overlap; best_id = track.track_id; }
        else second = std::max(second, overlap);
        if (track.track_id == request.candidate_global_id) candidate = &track;
      }
      if (!candidate || best_id != request.candidate_global_id || first < 0.5 || first-second < 0.15) {
        identity.confirmations = 0;
        return fail("ambiguous or inconsistent detector geometry");
      }
      if (identity.attached_detector && request.candidate_global_id != id) {
        // Simultaneously measured objects are not fragments of one entity.
        for (const auto& track : snapshot->tracks)
          if (track.track_id == id && track.detected && !track.predicted &&
              track.lifecycle_state == "confirmed") {
            identity.confirmations = 0;
            return fail("original entity is simultaneously visible");
          }
        const auto previous = global_entities_.find(id);
        const auto next = global_entities_.find(request.candidate_global_id);
        if (previous == global_entities_.end() || next == global_entities_.end())
          return fail("entity evidence expired");
        const auto& before = previous->second;
        const auto& after = next->second;
        const auto& compatible = tracklet_stitch_config_.compatible_class_ids;
        const bool same_family =
            std::find(compatible.begin(), compatible.end(), before.class_id) != compatible.end() &&
            std::find(compatible.begin(), compatible.end(), after.class_id) != compatible.end();
        if (before.class_id >= 0 && after.class_id >= 0 &&
            before.class_id != after.class_id && !same_family)
          return fail("entity class conflict");
        if (before.has_world && after.has_world) {
          const double dt = (after.world_stamp-before.world_stamp).toSec();
          if (dt < 0.0 || dt > tracklet_stitch_config_.archive_ttl_sec)
            return fail("world evidence stale");
          double squared = 0.0, displacement = 0.0, velocity_delta = 0.0, innovation = 0.0;
          for (int axis=0; axis<3; ++axis) {
            const double delta = after.world[axis]-before.world[axis]-
                (before.has_world_velocity ? before.world_velocity[axis]*dt : 0.0);
            squared += delta*delta;
            const double raw_delta = after.world[axis]-before.world[axis];
            displacement += raw_delta*raw_delta;
            const double dv = after.world_velocity[axis]-before.world_velocity[axis];
            velocity_delta += dv*dv;
            innovation += delta*delta / std::max(tracklet_stitch_config_.minimum_position_variance_m2,
                before.world_variance[axis]+after.world_variance[axis]+
                tracklet_stitch_config_.process_noise_m2_per_s2*dt*dt);
          }
          if (!std::isfinite(innovation) ||
              std::sqrt(squared) > tracklet_stitch_config_.maximum_distance_m ||
              innovation > tracklet_stitch_config_.mahalanobis_gate_sq ||
              (dt > 0.05 && std::sqrt(displacement)/dt > tracklet_stitch_config_.maximum_speed_mps) ||
              (before.has_world_velocity && after.has_world_velocity &&
               std::sqrt(velocity_delta) > tracklet_stitch_config_.maximum_velocity_delta_mps)) {
            ROS_WARN_STREAM_THROTTLE(2.0, "[xd_uav_track] visual rebind metric rejection id="
                << id << " candidate=" << request.candidate_global_id << " dt=" << dt
                << " residual_m=" << std::sqrt(squared) << " innovation=" << innovation
                << " displacement_m=" << std::sqrt(displacement)
                << " velocity_delta_mps=" << std::sqrt(velocity_delta)
                << " before=(" << before.world[0] << "," << before.world[1] << "," << before.world[2]
                << ") after=(" << after.world[0] << "," << after.world[1] << "," << after.world[2] << ")");
            return fail("world innovation conflict");
          }
        } else if ((stamp-identity.detector_stamp).toSec() > 1.5 &&
            !(identity.candidate == request.candidate_global_id && identity.confirmations > 0 &&
              (identity.bind_started_stamp-identity.detector_stamp).toSec() <= 1.5 &&
              (stamp-identity.bind_started_stamp).toSec() <= 1.5 &&
              (stamp-identity.confirmation_stamp).toSec() <= tracklet_stitch_max_confirmation_gap_sec_+1e-6)) {
          // Weak image evidence can bridge a short, continuous interruption,
          // never a long disappearance without a common metric frame.
          return fail("metric evidence required after long interruption");
        }
      }
      if (identity.signature.empty() || request.image_signature.size() != identity.signature.size())
        return fail("appearance evidence unavailable");
      double dot = 0.0, aa = 0.0, bb = 0.0;
      for (std::size_t k=0;k<identity.signature.size();++k) {
        if (!std::isfinite(request.image_signature[k]) || !std::isfinite(identity.signature[k]))
          return fail("invalid appearance evidence");
        dot += request.image_signature[k]*identity.signature[k];
        aa += identity.signature[k]*identity.signature[k];
        bb += request.image_signature[k]*request.image_signature[k];
      }
      if (dot / std::max(1e-9, std::sqrt(aa*bb)) < 0.55) {
        identity.confirmations = 0;
        return fail("appearance conflict");
      }
      if (stamp <= identity.confirmation_stamp) return fail("duplicate bind timestamp");
      if (identity.candidate != request.candidate_global_id ||
          (stamp-identity.confirmation_stamp).toSec() > 1.5)
        identity.confirmations = 0;
      if (identity.confirmations == 0) identity.bind_started_stamp = stamp;
      identity.candidate = request.candidate_global_id;
      identity.confirmation_stamp = stamp;
      if (++identity.confirmations < 3) {
        response.success = true; response.message = "bind pending"; return true;
      }
      const int old = request.candidate_global_id;
      if (old != id) {
        // A second manually registered entity cannot be collapsed by ROI overlap.
        if (visual_identities_.count(old)) return fail("candidate owns a separate visual identity");
        auto entity = global_entities_.find(old);
        if (entity == global_entities_.end()) return fail("candidate entity expired");
        global_entities_[id] = entity->second;
        global_entities_.erase(old);
        for (auto& mapping : global_id_by_local_)
          if (mapping.second == old) mapping.second = id;
        if (locked_global_id_ == old) locked_global_id_ = id;
        for (auto& pending : pending_tracklet_stitches_)
          if (pending.second.identity_id == old) pending.second.identity_id = id;
        for (auto& source : latest_tracks_by_source_) {
          for (auto& track : source.second.tracks)
            if (track.track_id == old) track.track_id = id;
          tracks_by_source_publisher_.publish(source.second);
        }
        for (auto& track : latest_tracks_.tracks)
          if (track.track_id == old) track.track_id = id;
      }
      if (identity.latest.class_id < 0) identity.latest.class_id = candidate->class_id;
      if (identity.attached_detector) identity.detector_stamp = stamp;
      response.success = true; response.bound = true;
      response.message = "detector track bound to visual global identity";
      publishVisualTracks(identity.source, identity.latest.header);
      return true;
    }
    if (request.action != "register" && request.action != "update" && request.action != "attach")
      return fail("invalid action");
    if (request.action == "update" && request.observation.header.stamp <= identity.latest.header.stamp)
      return fail("out of order visual observation");
    identity.latest = request.observation;
    const auto known_entity = global_entities_.find(id);
    if (known_entity != global_entities_.end() && known_entity->second.class_id >= 0)
      identity.latest.class_id = known_entity->second.class_id;
    identity.latest.track_id = id;
    identity.latest.association_method = "visual_identity";
    identity.latest.control_measurement_ready = false;
    if (identity.latest.detected && !identity.latest.predicted)
      identity.latest.last_observation_stamp = identity.latest.header.stamp;
    else if (!identity.history.empty())
      identity.latest.last_observation_stamp = identity.history.back().last_observation_stamp;
    // Image-only SOT cannot manufacture metric frame/time provenance.
    identity.latest.position_frame.clear();
    identity.latest.position_stamp = ros::Time();
    identity.latest.has_relative_position_body = false;
    identity.latest.range_valid = false;
    identity.updated = ros::WallTime::now();
    if (identity.attached_detector) {
      const auto history = identity_detection_history_.find(identity.source);
      if (history != identity_detection_history_.end())
        for (const auto& snapshot : history->second)
          if (snapshot.header.stamp == request.observation.header.stamp)
            for (const auto& track : snapshot.tracks)
              if (track.track_id == id && track.detected && !track.predicted &&
                  track.lifecycle_state == "confirmed") {
                identity.detector_stamp = snapshot.header.stamp;
                if (boxIou(track, request.observation) >= 0.5 &&
                    request.image_signature.size() == identity.signature.size()) {
                  bool finite = true;
                  double dot = 0.0, aa = 0.0, bb = 0.0;
                  for (std::size_t k=0; k<identity.signature.size(); ++k) {
                    const double value = request.image_signature[k];
                    finite = finite && std::isfinite(value);
                    dot += value*identity.signature[k]; aa += value*value;
                    bb += identity.signature[k]*identity.signature[k];
                  }
                  if (finite && dot/std::max(1e-9, std::sqrt(aa*bb)) >= 0.55)
                    identity.signature = request.image_signature;
                }
              }
    }
    identity.history.push_back(identity.latest);
    while (identity.history.size() > 32U) identity.history.pop_front();
    auto& entity = global_entities_[id];
    entity.established = true; entity.provisional = false;
    entity.last_seen_wall = identity.updated;
    // Do not refresh world_stamp or deep ReID using image-only observations.
    publishVisualTracks(identity.source, identity.latest.header);
    response.success = true; response.message = "visual identity updated";
    return true;
  }

  bool captureVehiclePose(const ros::Time& stamp, std::array<double, 3>* position,
                          geometry_msgs::Quaternion* orientation) const {
    xd_uav_track::VehicleState capture;
    if (stamp.isZero() || !controller_->vehicleStateAtObservationTime(stamp.toSec(), &capture) ||
        std::abs(capture.observation_time-stamp.toSec()) > source_identity_max_age_sec_)
      return false;
    *position = {{capture.x, capture.y, capture.z}};
    const double cr=std::cos(capture.roll*.5), sr=std::sin(capture.roll*.5);
    const double cp=std::cos(capture.pitch*.5), sp=std::sin(capture.pitch*.5);
    const double cy=std::cos(capture.yaw*.5), sy=std::sin(capture.yaw*.5);
    orientation->x=sr*cp*cy-cr*sp*sy; orientation->y=cr*sp*cy+sr*cp*sy;
    orientation->z=cr*cp*sy-sr*sp*cy; orientation->w=cr*cp*cy+sr*sp*sy;
    return true;
  }

  bool trackWorldPosition(const xd_uav_track::TrackState& track,
                          const ros::Time& stamp,
                          std::array<double, 3>* result) const {
    if (!track.detected || track.predicted || !track.range_valid ||
        !track.has_relative_position_body ||
        (!track.position_stamp.isZero() && track.position_stamp != stamp) ||
        !have_state_ || stamp.isZero())
      return false;
    std::array<double, 3> capture_position;
    geometry_msgs::Quaternion capture_orientation;
    if (!captureVehiclePose(stamp, &capture_position, &capture_orientation)) return false;
    for (int axis = 0; axis < 3; ++axis)
      if (!std::isfinite(track.relative_position_body[axis])) return false;
    for (int axis : {0, 4, 8})
      if (!std::isfinite(track.position_covariance[axis]) ||
          track.position_covariance[axis] < 0.0F ||
          track.position_covariance[axis] >
              source_identity_max_distance_m_ * source_identity_max_distance_m_)
        return false;
    // Detector metric coordinates are FRD; ROS odometry orientation rotates
    // FLU body vectors into its world frame.
    const double x = track.relative_position_body[0];
    const double y = -track.relative_position_body[1];
    const double z = -track.relative_position_body[2];
    const auto& q = capture_orientation;
    const double xx = q.x * q.x, yy = q.y * q.y, zz = q.z * q.z;
    (*result)[0] = capture_position[0] +
        (1.0 - 2.0 * (yy + zz)) * x + 2.0 * (q.x * q.y - q.w * q.z) * y +
        2.0 * (q.x * q.z + q.w * q.y) * z;
    (*result)[1] = capture_position[1] +
        2.0 * (q.x * q.y + q.w * q.z) * x + (1.0 - 2.0 * (xx + zz)) * y +
        2.0 * (q.y * q.z - q.w * q.x) * z;
    (*result)[2] = capture_position[2] +
        2.0 * (q.x * q.z - q.w * q.y) * x +
        2.0 * (q.y * q.z + q.w * q.x) * y + (1.0 - 2.0 * (xx + yy)) * z;
    return std::isfinite((*result)[0]) && std::isfinite((*result)[1]) &&
           std::isfinite((*result)[2]);
  }

  bool trackWorldPositionVariance(
      const xd_uav_track::TrackState& track,
      const ros::Time& stamp,
      std::array<double, 3>* result) const {
    if (result == nullptr) return false;
    for (const float value : track.position_covariance)
      if (!std::isfinite(static_cast<double>(value))) return false;
    std::array<double, 3> capture_position;
    geometry_msgs::Quaternion capture_orientation;
    if (!captureVehiclePose(stamp, &capture_position, &capture_orientation)) return false;
    const auto& q = capture_orientation;
    const double xx = q.x * q.x, yy = q.y * q.y, zz = q.z * q.z;
    const double xy = q.x * q.y, xz = q.x * q.z, yz = q.y * q.z;
    const double wx = q.w * q.x, wy = q.w * q.y, wz = q.w * q.z;
    double rotation[3][3] = {
        {1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)},
        {2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)},
        {2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)}};
    // FRD -> FLU before applying the vehicle attitude.
    for (int row = 0; row < 3; ++row) {
      rotation[row][1] *= -1.0;
      rotation[row][2] *= -1.0;
    }
    double covariance[3][3];
    for (int row = 0; row < 3; ++row)
      for (int column = 0; column < 3; ++column)
        covariance[row][column] = track.position_covariance[row * 3 + column];
    for (int axis = 0; axis < 3; ++axis) {
      double variance = 0.0;
      for (int row = 0; row < 3; ++row)
        for (int column = 0; column < 3; ++column)
          variance += rotation[axis][row] * covariance[row][column] *
                      rotation[axis][column];
      if (!std::isfinite(variance) || variance < -1e-5) return false;
      (*result)[axis] = std::max(0.0, variance);
    }
    return true;
  }

  void assignGlobalIds(xd_uav_track::ManagedDetectionFrame* frame,
                       const std::string& source) {
    pruneVisualIdentities();
    const ros::Time stamp = frame->tracks.header.stamp;
    std::set<int> present_local_ids;
    std::map<int, bool> currently_detected;
    for (const auto& track : frame->tracks.tracks) {
      present_local_ids.insert(track.track_id);
      currently_detected[track.track_id] = track.detected && !track.predicted;
    }

    const auto update_entity = [&](GlobalEntity* entity,
                                   const xd_uav_track::TrackState& track,
                                   const int local_id,
                                   const bool has_world,
                                   const std::array<double, 3>& world,
                                   const std::vector<float>* appearance) {
      if (entity == nullptr) return;
      if (entity->class_id < 0 && track.lifecycle_state == "confirmed")
        entity->class_id = track.class_id;
      if (track.detected && !track.predicted) {
        // Entity observation provenance is intentionally separate from
        // active_source_: this is the latest metric contributor, while the
        // latter is the only source allowed to drive follower control.
        const std::string previous_metric_source = entity->source;
        entity->source = source;
        entity->local_track_id = local_id;
        entity->stamp = stamp;
        entity->last_seen_wall = ros::WallTime::now();
        if (has_world) {
          std::array<double, 3> measured_variance{{1.0, 1.0, 1.0}};
          const bool measured_variance_valid = trackWorldPositionVariance(
              track, stamp, &measured_variance);
          std::array<double, 3> fused_world = world;
          std::array<double, 3> fused_variance = measured_variance_valid
              ? measured_variance : std::array<double, 3>{{1.0, 1.0, 1.0}};
          const double dt = entity->world_stamp.isZero() || stamp.isZero()
              ? 0.0 : (stamp - entity->world_stamp).toSec();
          // Two calibrated cameras that observe the same entity nearly at
          // the same capture time can reduce center jitter by covariance
          // weighting.  Never blend delayed measurements: that would smear
          // a moving vehicle and incorrectly look like a better estimate.
          const bool synchronized_other_source = entity->has_world &&
              previous_metric_source != source && std::abs(dt) <=
                  cross_source_fusion_max_time_delta_sec_;
          if (synchronized_other_source) {
            double squared_distance = 0.0;
            for (std::size_t axis = 0; axis < world.size(); ++axis) {
              const double delta = world[axis] - entity->world[axis];
              squared_distance += delta * delta;
            }
            const bool position_consistent = std::isfinite(squared_distance) &&
                std::sqrt(squared_distance) <=
                    tracklet_stitch_config_.cross_source_max_distance_m;
            if (position_consistent && measured_variance_valid) {
              for (std::size_t axis = 0; axis < world.size(); ++axis) {
                const double old_variance = std::max(
                    1e-4, entity->world_variance[axis]);
                const double new_variance = std::max(
                    1e-4, measured_variance[axis]);
                if (!std::isfinite(old_variance) ||
                    !std::isfinite(new_variance) ||
                    old_variance > tracklet_stitch_config_.cross_source_max_position_variance_m2 ||
                    new_variance > tracklet_stitch_config_.cross_source_max_position_variance_m2) {
                  fused_world = world;
                  fused_variance = measured_variance;
                  break;
                }
                const double old_weight = 1.0 / old_variance;
                const double new_weight = 1.0 / new_variance;
                fused_world[axis] = (entity->world[axis] * old_weight +
                                     world[axis] * new_weight) /
                    (old_weight + new_weight);
                fused_variance[axis] = 1.0 / (old_weight + new_weight);
              }
            }
          }
          if (entity->has_world && dt > 0.05 && dt <= 5.0) {
            std::array<double, 3> measured_velocity{{0.0, 0.0, 0.0}};
            double speed_sq = 0.0;
            for (std::size_t axis = 0; axis < world.size(); ++axis) {
              measured_velocity[axis] =
                  (fused_world[axis] - entity->world[axis]) / dt;
              speed_sq += measured_velocity[axis] * measured_velocity[axis];
            }
            const double speed = std::sqrt(speed_sq);
            if (std::isfinite(speed) && speed <=
                1.5 * tracklet_stitch_config_.maximum_speed_mps) {
              if (entity->has_world_velocity) {
                for (std::size_t axis = 0; axis < world.size(); ++axis)
                  entity->world_velocity[axis] = 0.65 * entity->world_velocity[axis] +
                      0.35 * measured_velocity[axis];
              } else {
                entity->world_velocity = measured_velocity;
                entity->has_world_velocity = true;
              }
            }
          }
          entity->world = fused_world;
          entity->world_variance = fused_variance;
          entity->world_stamp = stamp;
          entity->has_world = true;
          ++entity->metric_observations;
        }
        if (appearance != nullptr && !appearance->empty()) {
          entity->appearance = *appearance;
          entity->appearance_gallery.push_back(*appearance);
          while (entity->appearance_gallery.size() > 8U)
            entity->appearance_gallery.erase(entity->appearance_gallery.begin());
        }
        if (track.lifecycle_state == "confirmed") {
          entity->established = true;
        }
      }
    };

    for (auto& track : frame->tracks.tracks) {
      const int local_id = track.track_id;
      const auto key = std::make_pair(source, local_id);
      auto mapping = global_id_by_local_.find(key);
      // A stable upstream/local numeric ID can be reused after its manager
      // track was removed between source frames. Do not inherit its old
      // global identity solely because the integer was recycled; force the
      // new generation through the normal tracklet gates.
      if (mapping != global_id_by_local_.end() && track.age_frames <= 1 &&
          track.association_method == "new") {
        const auto old_identity = global_entities_.find(mapping->second);
        if (old_identity != global_entities_.end() &&
            old_identity->second.source == source &&
            old_identity->second.local_track_id == local_id) {
          global_id_by_local_.erase(mapping);
          pending_tracklet_stitches_.erase(key);
          mapping = global_id_by_local_.end();
          ROS_DEBUG("[xd_uav_track] local id reused; requiring tracklet gates "
                    "source=%s local_id=%d", source.c_str(), local_id);
        }
      }
      std::array<double, 3> world;
      const bool has_world = trackWorldPosition(track, stamp, &world);
      std::array<double, 3> world_variance{{1.0, 1.0, 1.0}};
      const bool has_world_variance = has_world &&
          trackWorldPositionVariance(track, stamp, &world_variance);
      const std::vector<float>* appearance = nullptr;
      for (const auto& candidate : frame->candidates.candidates)
        if (candidate.track_id == local_id &&
            !candidate.appearance_embedding.empty()) {
          appearance = &candidate.appearance_embedding;
          break;
        }
      int global_id = mapping == global_id_by_local_.end() ? -1 : mapping->second;
      if (global_id < 0) {
        if (global_entities_.size() >= static_cast<std::size_t>(tracklet_stitch_maximum_identities_)) {
          auto oldest = global_entities_.end();
          for (auto candidate = global_entities_.begin(); candidate != global_entities_.end(); ++candidate) {
            if (candidate->first == locked_global_id_ || visual_identities_.count(candidate->first)) continue;
            bool mapped = false;
            for (const auto& entry : global_id_by_local_)
              if (entry.second == candidate->first) { mapped = true; break; }
            if (!mapped && (oldest == global_entities_.end() ||
                candidate->second.last_seen_wall < oldest->second.last_seen_wall)) oldest = candidate;
          }
          if (oldest != global_entities_.end()) global_entities_.erase(oldest);
          else {
            // Never alias a live entity or grow past the hard limit. An
            // unassigned local track may be admitted on a later capture.
            track.track_id = -1;
            ROS_WARN_THROTTLE(2.0, "[xd_uav_track] global identity capacity reached");
            continue;
          }
        }
        global_id = next_global_id_++;
        global_id_by_local_[key] = global_id;
        global_entities_.emplace(global_id, GlobalEntity());
      }
      auto entity_it = global_entities_.find(global_id);
      if (entity_it == global_entities_.end())
        entity_it = global_entities_.emplace(global_id, GlobalEntity()).first;
      update_entity(&entity_it->second, track, local_id, has_world, world,
                    appearance);

      const bool is_new_identity = entity_it->second.provisional;
      if (is_new_identity && track.lifecycle_state == "confirmed" &&
          !tracklet_stitch_config_.enabled) {
        entity_it->second.provisional = false;
      }
      // A first confirmed image may precede usable odometry/metric localization.
      // Keep it eligible for a later calibrated stitch instead of permanently
      // committing an independent identity on that first missing-world frame.
      // The image track is still published; no metric evidence is invented.
      if (tracklet_stitch_config_.enabled && is_new_identity &&
          track.detected && !track.predicted &&
          track.lifecycle_state == "confirmed" && has_world) {
        xd_uav_track::TrackletObservation observation;
        observation.class_id = track.class_id;
        observation.source = source;
        observation.stamp_sec = stamp.toSec();
        observation.confirmed = true;
        observation.has_world_position = has_world;
        observation.world_position = world;
        observation.position_variance = has_world_variance
            ? world_variance : std::array<double, 3>{{1.0, 1.0, 1.0}};
        observation.has_world_velocity = entity_it->second.has_world_velocity;
        observation.world_velocity = entity_it->second.world_velocity;
        if (appearance != nullptr) observation.appearance = *appearance;

        std::vector<xd_uav_track::TrackletIdentitySnapshot> identities;
        identities.reserve(global_entities_.size());
        bool waiting_for_lost_same_source_identity = false;
        for (const auto& candidate_entry : global_entities_) {
          const int candidate_id = candidate_entry.first;
          const auto& candidate = candidate_entry.second;
          if (candidate_id == global_id || candidate.provisional ||
              (!global_identity_merge_enabled_ &&
               !tracklet_stitch_config_.cross_source_metric_enabled &&
               candidate.source != source))
            continue;
          bool active_in_source = false;
          for (const auto& local_mapping : global_id_by_local_) {
            if (local_mapping.first.first == source &&
                local_mapping.second == candidate_id &&
                present_local_ids.count(local_mapping.first.second) != 0) {
              active_in_source = true;
              const auto observed = currently_detected.find(
                  local_mapping.first.second);
              if (candidate.class_id == track.class_id &&
                  observed != currently_detected.end() && !observed->second)
                waiting_for_lost_same_source_identity = true;
              break;
            }
          }
          xd_uav_track::TrackletIdentitySnapshot snapshot;
          snapshot.identity_id = candidate_id;
          snapshot.class_id = candidate.class_id;
          snapshot.source = candidate.source;
          snapshot.last_seen_sec = candidate.world_stamp.toSec();
          snapshot.locked_target = candidate_id == locked_global_id_;
          snapshot.established = candidate.established;
          snapshot.active_in_source = active_in_source;
          snapshot.has_world_position = candidate.has_world;
          snapshot.has_world_velocity = candidate.has_world_velocity;
          snapshot.world_position = candidate.world;
          snapshot.position_variance = candidate.world_variance;
          snapshot.world_velocity = candidate.world_velocity;
          snapshot.appearance_prototypes = candidate.appearance_gallery;
          identities.push_back(std::move(snapshot));
        }
        const auto match = xd_uav_track::findTrackletStitchMatch(
            observation, identities, tracklet_stitch_config_);
        auto pending = pending_tracklet_stitches_.find(key);
        if (match.identity_id >= 0) {
          xd_uav_track::TrackletStitchConfirmationState previous;
          if (pending != pending_tracklet_stitches_.end()) {
            previous.identity_id = pending->second.identity_id;
            previous.consecutive_hits = pending->second.consecutive_hits;
            previous.last_stamp_sec = pending->second.last_stamp.toSec();
            previous.last_score = pending->second.last_score;
          }
          const auto confirmed = xd_uav_track::advanceTrackletStitchConfirmation(
              previous, match, stamp.toSec(),
              tracklet_stitch_max_confirmation_gap_sec_);
          PendingTrackletStitch hypothesis;
          hypothesis.identity_id = confirmed.identity_id;
          hypothesis.consecutive_hits = confirmed.consecutive_hits;
          hypothesis.last_stamp = ros::Time(confirmed.last_stamp_sec);
          hypothesis.last_score = confirmed.last_score;
          pending_tracklet_stitches_[key] = hypothesis;
          if (hypothesis.consecutive_hits >= tracklet_stitch_confirmation_frames_) {
            const int provisional_id = global_id;
            global_id = match.identity_id;
            global_id_by_local_[key] = global_id;
            global_entities_.erase(provisional_id);
            auto committed_entity = global_entities_.find(global_id);
            if (committed_entity != global_entities_.end()) {
              update_entity(&committed_entity->second, track, local_id, has_world,
                            world, appearance);
              committed_entity->second.provisional = false;
              committed_entity->second.established = true;
            }
            pending_tracklet_stitches_.erase(key);
            track.association_method = "tracklet_stitch";
            track.reidentification_match = true;
            ROS_INFO("[xd_uav_track] stitched source=%s local_id=%d to global_id=%d "
                     "score=%.3f margin=%.3f confirmations=%d",
                     source.c_str(), local_id, global_id, match.score,
                     match.margin, hypothesis.consecutive_hits);
          } else {
            track.association_method = "stitch_pending";
            track.lifecycle_state = "tentative";
            track.control_measurement_ready = false;
          }
        } else {
          pending_tracklet_stitches_.erase(key);
          if (match.ambiguous) {
            entity_it->second.stitch_ambiguous = true;
            track.association_method = "stitch_ambiguous";
            track.lifecycle_state = "tentative";
            track.control_measurement_ready = false;
          } else if (!entity_it->second.stitch_ambiguous &&
                     !waiting_for_lost_same_source_identity) {
            ++entity_it->second.stitch_search_frames;
            if (entity_it->second.stitch_search_frames >=
                tracklet_stitch_confirmation_frames_)
              entity_it->second.provisional = false;
          }
        }
      } else if (is_new_identity && track.detected &&
                 track.association_method == "stitch_pending") {
        track.association_method = "new";
      }
      if (!track.detected) pending_tracklet_stitches_.erase(key);
      track.track_id = global_id;
      const auto canonical = global_entities_.find(global_id);
      if (canonical != global_entities_.end() && canonical->second.class_id >= 0)
        track.class_id = canonical->second.class_id;
    }
    auto& tracks = frame->tracks.tracks;
    tracks.erase(std::remove_if(tracks.begin(), tracks.end(),
        [](const xd_uav_track::TrackState& track) { return track.track_id < 0; }), tracks.end());
    for (auto it = global_id_by_local_.begin(); it != global_id_by_local_.end();) {
      if (it->first.first == source &&
          present_local_ids.count(it->first.second) == 0) {
        pending_tracklet_stitches_.erase(it->first);
        it = global_id_by_local_.erase(it);
      } else ++it;
    }
    // Expire archived identity prototypes and pending hypotheses by capture
    // time; retain an actively mapped or locked identity.
    for (auto it = global_entities_.begin(); it != global_entities_.end();) {
      bool active = it->first == locked_global_id_ || visual_identities_.count(it->first) != 0;
      for (const auto& mapping : global_id_by_local_)
        if (mapping.second == it->first) {
          active = true;
          break;
        }
      const double age = it->second.last_seen_wall.isZero()
          ? 0.0 : (ros::WallTime::now() - it->second.last_seen_wall).toSec();
      if (!active && age > tracklet_stitch_config_.archive_ttl_sec)
        it = global_entities_.erase(it);
      else ++it;
    }
    // Keep identity and hypothesis storage bounded even with noisy source
    // churn or a long-running process.
    if (global_entities_.size() > static_cast<std::size_t>(
            tracklet_stitch_maximum_identities_)) {
      for (auto it = global_entities_.begin(); it != global_entities_.end();) {
        bool active = it->first == locked_global_id_ || visual_identities_.count(it->first) != 0;
        for (const auto& mapping : global_id_by_local_)
          if (mapping.second == it->first) {
            active = true;
            break;
          }
        if (!active)
          it = global_entities_.erase(it);
        else ++it;
        if (global_entities_.size() <= static_cast<std::size_t>(
                tracklet_stitch_maximum_identities_)) break;
      }
    }
  }

  void clearSourceGlobalIds(const std::string& source) {
    for (auto it = global_id_by_local_.begin(); it != global_id_by_local_.end();) {
      if (it->first.first == source) {
        pending_tracklet_stitches_.erase(it->first);
        it = global_id_by_local_.erase(it);
      }
      else ++it;
    }
  }

  bool metricSourceIdentityCompatible(
      const xd_uav_track::TargetMeasurement& measurement,
      const std::string& source, std::string* reason) const {
    if (!source_identity_gate_enabled_ || !metric_identity_valid_ ||
        source == metric_identity_source_) return true;
    const double age = std::abs(measurement.observation_time -
                                metric_identity_time_);
    // An old identity anchor must not permanently prevent an explicit
    // re-acquisition. Spatial consistency is still enforced by the shared
    // world-filter innovation gate in TrackController.
    if (!std::isfinite(age) || age > source_identity_max_age_sec_) return true;
    if (source_cross_class_reject_ && metric_identity_class_id_ >= 0 &&
        measurement.class_id >= 0 &&
        measurement.class_id != metric_identity_class_id_) {
      const auto& compatible = tracklet_stitch_config_.compatible_class_ids;
      const bool same_vehicle_family =
          std::find(compatible.begin(), compatible.end(),
                    metric_identity_class_id_) != compatible.end() &&
          std::find(compatible.begin(), compatible.end(),
                    measurement.class_id) != compatible.end();
      if (!same_vehicle_family) {
        if (reason != nullptr) *reason =
            "cross-source class conflicts with the fresh selected target";
        return false;
      }
    }
    return controller_->metricMeasurementCompatible(
        measurement, source_identity_max_distance_m_, reason);
  }

  void detectionsCallback(
      const xd_uav_track::DetectionArray::ConstPtr& message) {
    std::lock_guard<std::mutex> lock(mutex_);
    const std::string source = message->image_source.empty()
        ? (message->header.frame_id.empty() ? std::string("default")
                                            : message->header.frame_id)
        : message->image_source;
    if (!sourceAllowed(source)) return;
    pruneSources(ros::WallTime::now());
    if (!source_trackers_.count(source) &&
        source_trackers_.size() >= static_cast<std::size_t>(maximum_sources_)) {
      ROS_WARN_THROTTLE(2.0, "[xd_uav_track] source capacity reached; rejecting %s",
                        source.c_str());
      return;
    }
    if (message->model_epoch && message->model_epoch < model_epoch_) return;
    if (message->model_epoch > model_epoch_) {
      model_epoch_ = message->model_epoch;
      model_class_names_ = message->class_names;
      for (auto& manager : source_trackers_) manager.second->invalidateClasses();
      for (auto& entity : global_entities_) {
        entity.second.class_id = -1;
        entity.second.appearance.clear();
        entity.second.appearance_gallery.clear();
      }
      for (auto& identity : visual_identities_) {
        identity.second.latest.class_id = -1;
        identity.second.history.clear();
      }
      latest_tracks_by_source_.clear();
      identity_detection_history_.clear();
      pending_tracklet_stitches_.clear();
      metric_identity_class_id_ = -1;
    }
    const ros::Time stamp = message->header.stamp;
    auto stamp_it = source_last_stamp_.find(source);
    if (reject_out_of_order_timestamps_ && !stamp.isZero() &&
        stamp_it != source_last_stamp_.end() && stamp <= stamp_it->second) {
      // A backwards jump larger than the configured reset threshold denotes a
      // restarted sensor clock; clear only that source instead of poisoning
      // the other camera's track history.
      const double jump = stamp < stamp_it->second
          ? (stamp_it->second - stamp).toSec() : 0.0;
      if (stamp == stamp_it->second || jump <= source_clock_reset_jump_sec_) return;
      auto tracker_it = source_trackers_.find(source);
      if (tracker_it != source_trackers_.end()) tracker_it->second->reset();
      clearSourceGlobalIds(source);
    }
    const unsigned int image_width = message->image_width;
    const unsigned int image_height = message->image_height;
    if (image_width == 0 || image_height == 0) {
      ROS_WARN_THROTTLE(2.0, "[xd_uav_track] DetectionArray image size is zero");
      return;
    }
    xd_uav_track::MultiTrackManager* tracker = trackerForSource(source);
    if (message->command == "reset") {
      tracker->reset();
      clearSourceGlobalIds(source);
      locked_global_id_ = -1;
      controller_->reset();
      metric_identity_valid_ = false;
    } else if (message->command == "start_track" && !perception_only_) {
      tracker_active_ = true;
    } else if (message->command == "stop_track") {
      tracker_active_ = false;
      locked_global_id_ = -1;
      controller_->reset();
      metric_identity_valid_ = false;
    }

    auto frame = tracker->update(
        *message, static_cast<int>(image_width),
        static_cast<int>(image_height), source);
    if (!frame.accepted) {
      ROS_WARN_THROTTLE(2.0, "[xd_uav_track] rejected duplicate/out-of-order source frame: %s",
                        source.c_str());
      return;
    }
    if (!stamp.isZero()) source_last_stamp_[source] = stamp;
    assignGlobalIds(&frame, source);
    source_last_receive_[source] = ros::WallTime::now();
    const std::string previous_active_source = active_source_;
    if (active_source_.empty()) active_source_ = source;
    const ros::WallTime wall_now = ros::WallTime::now();
    if (locked_global_id_ >= 0 && locked_target_release_timeout_sec_ > 0.0 &&
        !locked_target_last_seen_.isZero() &&
        (wall_now - locked_target_last_seen_).toSec() >
            locked_target_release_timeout_sec_) locked_global_id_ = -1;
    auto active_receive = source_last_receive_.find(active_source_);
    const bool active_stale = active_receive == source_last_receive_.end() ||
        (wall_now - active_receive->second).toSec() > source_handover_timeout_sec_;
    const bool manual_policy = source_policy_ == "manual" ||
        source_policy_ == "fixed";
    const bool fixed_first_policy = source_policy_ == "fixed_first";
    if (!manual_policy && active_stale && locked_global_id_ < 0)
      active_source_ = source;
    if (fixed_first_policy && locked_global_id_ < 0 &&
        source.find("fixed") != std::string::npos) {
      active_source_ = source;
    }
    if (!manual_policy && !fixed_first_policy && locked_global_id_ < 0 &&
        source != active_source_ &&
        source_quality_handover_enabled_ &&
        (wall_now - last_source_switch_).toSec() >= source_handover_cooldown_sec_) {
      const auto best_quality = [](const xd_uav_track::TrackStateArray& tracks) {
        float best = 0.0F;
        for (const auto& track : tracks.tracks) {
          if (track.lifecycle_state == "confirmed" ||
              track.lifecycle_state == "occluded") {
            best = std::max(best, track.tracking_quality);
          }
        }
        return best;
      };
      const float incoming_quality = best_quality(frame.tracks);
      const auto current_it = latest_tracks_by_source_.find(active_source_);
      const float active_quality = current_it == latest_tracks_by_source_.end()
          ? 0.0F : best_quality(current_it->second);
      if (incoming_quality >= source_min_incoming_quality_ &&
          active_quality <= source_min_active_quality_ &&
          incoming_quality >= active_quality + source_quality_margin_) {
        active_source_ = source;
        last_source_switch_ = wall_now;
      }
    }
    if (source == active_source_ && previous_active_source != active_source_ &&
        active_receive != source_last_receive_.end()) {
      // A source that has just been selected becomes the handover reference.
      last_source_switch_ = wall_now;
    }
    if ((manual_policy || fixed_first_policy) && active_source_.empty()) {
      active_source_ = source;
    }
    int selected_id = tracker->selectedTrackId();
    if (locked_global_id_ >= 0) {
      selected_id = -1;
      for (const auto& track : frame.tracks.tracks)
        if (track.track_id == locked_global_id_) {
          for (const auto& mapping : global_id_by_local_)
            if (mapping.first.first == source && mapping.second == locked_global_id_) {
              selected_id = mapping.first.second;
              break;
            }
          break;
        }
      tracker->setSelectedTrackId(selected_id);
    }
    if (!auto_selection_suppressed_ && selected_id < 0 && locked_global_id_ < 0 &&
        source == active_source_ && auto_select_track_) {
      float best_quality = -1.0F;
      for (const auto& track : frame.tracks.tracks) {
        if (track.lifecycle_state == "confirmed" &&
            track.tracking_quality > best_quality) {
          best_quality = track.tracking_quality;
          for (const auto& mapping : global_id_by_local_)
            if (mapping.first.first == source && mapping.second == track.track_id) {
              selected_id = mapping.first.second;
              break;
            }
        }
      }
      if (selected_id >= 0) tracker->setSelectedTrackId(selected_id);
    }

    const xd_uav_track::TrackState* selected = nullptr;
    for (auto& track : frame.tracks.tracks) {
      const auto mapping = global_id_by_local_.find(std::make_pair(source, selected_id));
      track.selected = mapping != global_id_by_local_.end() &&
          track.track_id == mapping->second;
      if (track.selected &&
          (track.lifecycle_state == "confirmed" ||
           track.lifecycle_state == "occluded")) {
        selected = &track;
      }
    }
    if (selected && selected->control_measurement_ready &&
        locked_global_id_ < 0) {
      locked_global_id_ = selected->track_id;
      auto identity = global_entities_.find(locked_global_id_);
      if (identity != global_entities_.end())
        identity->second.provisional = false;
    }
    if (selected && selected->detected && !selected->predicted &&
        selected->control_measurement_ready)
      locked_target_last_seen_ = wall_now;
    if (selected && selected->detected && !selected->predicted &&
        selected->control_measurement_ready &&
        source != active_source_ && locked_global_id_ >= 0 &&
        !manual_policy &&
        (active_stale || (source_quality_handover_enabled_ &&
         selected->tracking_quality >= source_min_incoming_quality_ &&
         (wall_now - last_source_switch_).toSec() >= source_handover_cooldown_sec_))) {
      active_source_ = source;
      last_source_switch_ = wall_now;
    }
    latest_tracks_by_source_[source] = frame.tracks;
    if (!identity_detection_history_.count(source) && identity_detection_history_.size() >= 16U) {
      auto oldest = std::min_element(identity_detection_history_.begin(), identity_detection_history_.end(),
          [](const auto& a, const auto& b) {
            return a.second.back().header.stamp < b.second.back().header.stamp;
          });
      identity_detection_history_.erase(oldest);
    }
    auto& identity_history = identity_detection_history_[source];
    auto identity_snapshot = frame.tracks;
    // Identity confirmation evaluates actual same-capture YOLO evidence,
    // not the EMA confidence or Kalman-smoothed rectangle used for display.
    // Otherwise a strong current detection can remain below 0.50 for several
    // frames, and a weak one can inherit a previously high EMA confidence.
    for (auto& track : identity_snapshot.tracks) {
      if (!track.detected || track.predicted) continue;
      track.confidence = 0.0F;
      for (const auto& candidate : frame.candidates.candidates) {
        if (!candidate.measurement_source.empty() && candidate.measurement_source != "detector") continue;
        const auto mapping = global_id_by_local_.find(std::make_pair(source,candidate.track_id));
        if (mapping == global_id_by_local_.end() || mapping->second != track.track_id) continue;
        track.confidence = candidate.confidence;
        if (candidate.has_bbox) track.bbox = candidate.bbox;
        if (candidate.has_normalized_bbox) track.normalized_bbox = candidate.normalized_bbox;
        break;
      }
    }
    identity_history.push_back(std::move(identity_snapshot));
    while (identity_history.size() > 32U) identity_history.pop_front();
    tracks_by_source_publisher_.publish(frame.tracks);
    if (observations_publisher_.getNumSubscribers() > 0) {
      auto observations = frame.candidates;
      observations.image_source=source;
      observations.image_width=image_width;
      observations.image_height=image_height;
      // Internal image anchors carry the association's public entity ID, but
      // retain measured geometry rather than the smoothed/predicted box.
      for (auto& candidate : observations.candidates) {
        const auto identity = global_id_by_local_.find(std::make_pair(source,candidate.track_id));
        candidate.track_id = identity == global_id_by_local_.end() ? -1 : identity->second;
      }
      observations_publisher_.publish(observations);
    }
    xd_uav_track::TargetPixelArray target_pixels;
    target_pixels.header = frame.tracks.header;
    target_pixels.image_source = source;
    target_pixels.image_width = image_width;
    target_pixels.image_height = image_height;
    for (const auto& track : frame.tracks.tracks) {
      if (track.lifecycle_state != "confirmed" &&
          track.lifecycle_state != "occluded") continue;
      const int x_min = std::max(0, std::min(static_cast<int>(image_width) - 1,
                                             track.bbox[0]));
      const int y_min = std::max(0, std::min(static_cast<int>(image_height) - 1,
                                             track.bbox[1]));
      const int x_max = std::max(0, std::min(static_cast<int>(image_width),
                                             track.bbox[2]));
      const int y_max = std::max(0, std::min(static_cast<int>(image_height),
                                             track.bbox[3]));
      if (x_max <= x_min || y_max <= y_min) continue;
      xd_uav_track::TargetPixel point;
      point.track_id = track.track_id;
      point.class_id = track.class_id;
      point.confidence = track.confidence;
      point.x = 0.5f * static_cast<float>(x_min + x_max);
      point.y = 0.5f * static_cast<float>(y_min + y_max);
      point.bbox[0] = x_min;
      point.bbox[1] = y_min;
      point.bbox[2] = x_max;
      point.bbox[3] = y_max;
      point.lifecycle_state = track.lifecycle_state;
      point.detected = track.detected;
      point.predicted = track.predicted;
      point.selected = track.selected;
      target_pixels.targets.push_back(point);
    }
    target_pixels_publisher_.publish(target_pixels);
    if (source == active_source_) {
      latest_tracks_ = frame.tracks;
      tracks_publisher_.publish(frame.tracks);
    }
    if (!tracker_active_) return;
    // A standby camera cannot invalidate the active camera's measurement.
    if (source != active_source_ && selected == nullptr) return;
    if (selected == nullptr) {
      controller_->clearMeasurement("no confirmed selected track");
      return;
    }
    if (selected->association_method == "ambiguous") {
      controller_->clearMeasurement("selected target association is ambiguous");
      return;
    }
    if (selected->detected && !selected->control_measurement_ready) {
      controller_->clearMeasurement("selected target reacquisition is tentative");
      return;
    }

    xd_uav_track::TargetMeasurement measurement;
    measurement.receive_time = ros::WallTime::now().toSec();
    measurement.observation_time = stamp.isZero()
        ? measurement.receive_time : stamp.toSec();
    measurement.image_source = source;
    measurement.image_width = image_width;
    measurement.image_height = image_height;
    measurement.x_min = selected->bbox[0];
    measurement.y_min = selected->bbox[1];
    measurement.x_max = selected->bbox[2];
    measurement.y_max = selected->bbox[3];
    measurement.confidence = selected->confidence;
    measurement.track_id = selected->track_id;
    measurement.class_id = selected->class_id;
    measurement.predicted = selected->predicted;
    measurement.reidentification_match = selected->reidentification_match;
    measurement.tracking_quality = selected->tracking_quality;
    measurement.association_method = selected->association_method;
    for (std::size_t i = 0; i < 4; ++i) {
      measurement.state_covariance[i] = selected->state_covariance[i];
    }
    measurement.has_relative_position_body =
        selected->has_relative_position_body;
    measurement.has_relative_velocity_body =
        selected->has_relative_velocity_body;
    measurement.range_valid = selected->range_valid;
    measurement.position_sigma_m = 0.0;
    if (selected->range_valid && selected->has_relative_position_body) {
      const double covariance_max = std::max({
          static_cast<double>(selected->position_covariance[0]),
          static_cast<double>(selected->position_covariance[4]),
          static_cast<double>(selected->position_covariance[8]), 0.0});
      measurement.position_sigma_m = std::sqrt(covariance_max);
    }
    for (std::size_t i = 0; i < 3; ++i) {
      measurement.relative_position_body[i] =
          selected->relative_position_body[i];
      measurement.relative_velocity_body[i] =
          selected->relative_velocity_body[i];
    }
    if (source != active_source_) {
      if (multi_source_fusion_enabled_ && measurement.range_valid &&
          measurement.has_relative_position_body) {
        std::string identity_reason;
        if (!metricSourceIdentityCompatible(measurement, source,
                                            &identity_reason)) {
          ROS_WARN_THROTTLE(2.0,
                            "[xd_uav_track] standby source rejected: %s",
                            identity_reason.c_str());
          return;
        }
        std::string metric_reason;
        if (!controller_->updateMetricMeasurement(measurement, &metric_reason)) {
          ROS_WARN_THROTTLE(2.0,
                            "[xd_uav_track] standby metric source rejected: %s",
                            metric_reason.c_str());
        }
      }
      return;
    }
    if (measurement.range_valid && measurement.has_relative_position_body) {
      std::string identity_reason;
      if (!metricSourceIdentityCompatible(measurement, source,
                                          &identity_reason)) {
        controller_->clearMeasurement(identity_reason);
        ROS_WARN_THROTTLE(2.0,
                          "[xd_uav_track] active source rejected: %s",
                          identity_reason.c_str());
        return;
      }
    }
    std::string reason;
    if (!controller_->updateMeasurement(measurement, &reason)) {
      controller_->clearMeasurement(reason);
      ROS_WARN_THROTTLE(2.0, "[xd_uav_track] rejected selected track: %s",
                        reason.c_str());
    } else if (measurement.range_valid && measurement.has_relative_position_body &&
               controller_->lastMetricMeasurementAccepted()) {
      metric_identity_valid_ = true;
      metric_identity_class_id_ = measurement.class_id;
      metric_identity_source_ = source;
      metric_identity_time_ = measurement.observation_time;
    }
  }

  void stateCallback(const nav_msgs::Odometry::ConstPtr& message) {
    if (message->header.frame_id.empty()) return;
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    eulerFromQuaternion(message->pose.pose.orientation, &roll, &pitch, &yaw);
    if (!std::isfinite(roll) || !std::isfinite(pitch) ||
        !std::isfinite(yaw)) return;
    std::lock_guard<std::mutex> lock(mutex_);
    const ros::WallTime now = ros::WallTime::now();
    state_yaw_ = yaw;
    vehicle_position_ = {{message->pose.pose.position.x,
                          message->pose.pose.position.y,
                          message->pose.pose.position.z}};
    vehicle_orientation_ = message->pose.pose.orientation;
    vehicle_stamp_ = message->header.stamp;
    state_frame_ = message->header.frame_id;
    state_receive_time_ = now;
    have_state_ = true;
    xd_uav_track::VehicleState state;
    state.roll = roll;
    state.pitch = pitch;
    state.altitude = message->pose.pose.position.z;
    state.x = message->pose.pose.position.x;
    state.y = message->pose.pose.position.y;
    state.z = message->pose.pose.position.z;
    state.yaw = yaw;
    state.receive_time = now.toSec();
    state.observation_time = message->header.stamp.isZero()
        ? state.receive_time : message->header.stamp.toSec();
    state.pose_valid = true;
    state.valid = true;
    controller_->setVehicleState(state);
  }

  void metricTargetCallback(
      const xd_uav_track::MetricTarget::ConstPtr& message) {
    if (message == nullptr) return;
    std::lock_guard<std::mutex> lock(mutex_);
    const auto canonical_frame = [](const std::string& frame) {
      return frame.empty() ? frame
                           : (frame.front() == '/' ? frame.substr(1) : frame);
    };
    if (!message->header.frame_id.empty() && !state_frame_.empty() &&
        canonical_frame(message->header.frame_id) !=
            canonical_frame(state_frame_)) {
      ROS_WARN_THROTTLE(2.0,
                        "[xd_uav_track] metric target frame '%s' does not "
                        "match odometry frame '%s'; target rejected",
                        message->header.frame_id.c_str(), state_frame_.c_str());
      return;
    }
    const auto finite = [](double value) { return std::isfinite(value); };
    const auto& p = message->pose.pose.position;
    const auto& v = message->velocity.twist.linear;
    std::array<double, 3> world_position{{p.x, p.y, p.z}};
    std::array<double, 3> world_velocity{{v.x, v.y, v.z}};
    if (!std::all_of(world_position.begin(), world_position.end(), finite)) {
      ROS_WARN_THROTTLE(2.0,
                        "[xd_uav_track] metric target position is non-finite");
      return;
    }
    const bool velocity_valid = message->has_velocity &&
        std::all_of(world_velocity.begin(), world_velocity.end(), finite);
    if (message->has_velocity && !velocity_valid) {
      ROS_WARN_THROTTLE(2.0,
                        "[xd_uav_track] metric target velocity is non-finite");
      return;
    }
    if (!message->valid) return;
    const double receive_time = ros::WallTime::now().toSec();
    const double observation_time = message->header.stamp.isZero()
        ? receive_time : message->header.stamp.toSec();
    double sigma = 0.0;
    const double covariance_max = std::max({
        static_cast<double>(message->pose.covariance[0]),
        static_cast<double>(message->pose.covariance[7]),
        static_cast<double>(message->pose.covariance[14]), 0.0});
    if (std::isfinite(covariance_max) && covariance_max > 0.0) {
      sigma = std::sqrt(covariance_max);
    }
    std::string reason;
    if (!controller_->updateMetricWorldTarget(
            message->target_id, receive_time, observation_time, world_position,
            velocity_valid, world_velocity, sigma,
            message->source.empty() ? std::string("metric_target")
                                     : message->source,
            &reason)) {
      ROS_WARN_THROTTLE(2.0,
                        "[xd_uav_track] metric world target rejected: %s",
                        reason.c_str());
    }
  }

  void gimbalStateCallback(
      const xd_uav_track::GimbalState::ConstPtr& message) {
    std::lock_guard<std::mutex> lock(mutex_);
    // Preserve the most recent health/authority/FOV status. Angle and status
    // messages commonly run at different rates; rebuilding the struct here
    // used to erase a valid status on every angle update.
    const ros::WallTime now = ros::WallTime::now();
    const bool capture_fresh = captureStampFresh(
        message->header.stamp, ros::Time::now(), gimbal_timeout_sec_);
    if (!capture_fresh) {
      ROS_WARN_THROTTLE(2.0,
                        "[xd_uav_track] rejecting delayed/future gimbal angle sample");
    }
    gimbal_state_data_.receive_time = now.toSec();
    gimbal_state_data_.yaw = message->yaw_rad;
    gimbal_state_data_.pitch = message->pitch_rad;
    gimbal_state_data_.roll = message->roll_rad;
    gimbal_state_data_.valid = message->valid && capture_fresh;
    controller_->setGimbalState(gimbal_state_data_);
    gimbal_state_receive_time_ = now;
    have_gimbal_state_ = message->valid && capture_fresh;
  }

  void gimbalStatusCallback(
      const xd_uav_track::GimbalStatus::ConstPtr& message) {
    std::lock_guard<std::mutex> lock(mutex_);
    const ros::WallTime now = ros::WallTime::now();
    // Once a status message has arrived, keep the status contract active even
    // when its valid bit is false. Treating invalid status as "no status"
    // would let a legacy angle frame bypass the safety gate.
    const bool capture_fresh = captureStampFresh(
        message->header.stamp, ros::Time::now(), gimbal_timeout_sec_);
    if (!capture_fresh) {
      ROS_WARN_THROTTLE(2.0,
                        "[xd_uav_track] rejecting delayed/future gimbal status sample");
    }
    gimbal_state_data_.status_valid = true;
    gimbal_state_data_.healthy = capture_fresh && message->valid && message->healthy;
    gimbal_state_data_.tracking_active = message->tracking_active;
    gimbal_state_data_.range_valid = message->range_valid;
    gimbal_state_data_.control_authority_available =
        message->control_authority_available;
    gimbal_state_data_.zoom_ratio = message->zoom_ratio;
    gimbal_state_data_.horizontal_fov_rad = message->horizontal_fov_rad;
    gimbal_state_data_.vertical_fov_rad = message->vertical_fov_rad;
    gimbal_state_data_.range_quality = message->range_quality;
    gimbal_state_data_.status_receive_time = now.toSec();
    controller_->setGimbalState(gimbal_state_data_);
    gimbal_status_receive_time_ = now;
  }

  bool setProfile(xd_uav_track::SetProfile::Request& request,
                  xd_uav_track::SetProfile::Response& response) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (perception_only_) {
      response.success = false;
      response.active_profile = xd_uav_track::followerProfileName(controller_->profile());
      response.message = "PERCEPTION_ONLY: flight guidance is disabled";
      return true;
    }
    std::string requested_name = request.profile;
    const bool legacy_fixed_wing = requested_name == "fw_attitude_rate";
    if (legacy_fixed_wing) {
      requested_name = "fw_velocity_vector";
    }
    xd_uav_track::FollowerProfile requested_profile;
    if (!xd_uav_track::parseFollowerProfile(requested_name,
                                             &requested_profile)) {
      response.success = false;
      response.active_profile =
          xd_uav_track::followerProfileName(controller_->profile());
      response.message = "supported profiles: mc_velocity_ground, "
          "mc_velocity_position, mc_velocity_distance, mc_velocity_chase, "
          "gm_velocity_chase, gm_velocity_vector, fw_velocity_vector, "
          "gmfw_velocity_vector, "
          "fw_metric_pursuit, fw_metric_orbit";
      return true;
    }
    const ros::WallTime now = ros::WallTime::now();
    if (!last_profile_change_.isZero() &&
        (now - last_profile_change_).toSec() <
            profile_switch_min_interval_sec_ &&
        requested_profile != requested_profile_) {
      response.success = false;
      response.active_profile =
          xd_uav_track::followerProfileName(requested_profile_);
      response.message = "profile switch rejected: minimum interval not elapsed";
      return true;
    }
    const auto previous_profile = controller_->profile();
    requested_profile_ = requested_profile;
    const bool changed = controller_->setProfile(requested_profile);
    if (changed) {
      last_profile_change_ = now;
      const bool fixed_wing_transition =
          xd_uav_track::isFixedWingProfile(previous_profile) ||
          xd_uav_track::isFixedWingProfile(requested_profile);
      blend_active_ = have_last_command_ && last_command_.valid &&
          profile_blend_duration_sec_ > 0.0 && !fixed_wing_transition;
      blend_start_ = now;
      blend_source_ = last_command_;
    }
    response.success = true;
    response.active_profile =
        xd_uav_track::followerProfileName(controller_->profile());
    if (legacy_fixed_wing) {
      response.message = changed
          ? "legacy fw_attitude_rate mapped to fw_velocity_vector"
          : "fw_velocity_vector already active; fw_attitude_rate is removed";
    } else {
      response.message = changed ? "follower profile changed"
                                 : "follower profile already active";
    }
    ROS_INFO("[xd_uav_track] %s: %s", response.message.c_str(),
             response.active_profile.c_str());
    return true;
  }

  bool startTracker(xd_uav_track::StartTracker::Request& request,
                    xd_uav_track::StartTracker::Response& response) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (perception_only_) {
      response.success = !request.start;
      response.active = false;
      response.message = "PERCEPTION_ONLY: use perception/target/select for visual locking";
      return true;
    }
    const bool changed = tracker_active_ != request.start;
    if (changed) {
      tracker_active_ = request.start;
      locked_global_id_ = -1;
      // Never reuse a box, filter state or PID integral from before a start or
      // stop transition. A continuous detector supplies a fresh box next.
      controller_->reset();
      metric_identity_valid_ = false;
    }

    response.success = true;
    response.active = tracker_active_;
    if (changed) {
      response.message = tracker_active_ ? "tracker started"
                                         : "tracker stopped";
    } else {
      response.message = tracker_active_ ? "tracker already active"
                                         : "tracker already stopped";
    }
    ROS_INFO("[xd_uav_track] %s", response.message.c_str());
    return true;
  }

  bool selectTrack(xd_uav_track::SelectTrack::Request& request,
                   xd_uav_track::SelectTrack::Response& response) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (perception_only_ && request.start_tracking) {
      response.success = false;
      response.selected_target_id = -1;
      response.message = "PERCEPTION_ONLY: selection cannot enable flight guidance";
      return true;
    }
    const std::string source = request.image_source.empty()
        ? active_source_ : request.image_source;
    auto tracks_it = latest_tracks_by_source_.find(source);
    if (tracks_it == latest_tracks_by_source_.end()) {
      response.success = false;
      response.message = "requested image source has no track frame";
      response.selected_target_id = -1;
      return true;
    }
    const auto& source_tracks = tracks_it->second;
    if (!request.capture_timestamp.isZero()) {
      if (source_tracks.header.stamp.isZero() ||
          std::abs((source_tracks.header.stamp -
                    request.capture_timestamp).toSec()) >
              select_capture_tolerance_sec_) {
        response.success = false;
        response.message = "requested capture timestamp is outside source tolerance";
        response.selected_target_id = -1;
        return true;
      }
    }
    auto tracker_it = source_trackers_.find(source);
    if (tracker_it == source_trackers_.end()) {
      response.success = false;
      response.message = "requested image source tracker is unavailable";
      response.selected_target_id = -1;
      return true;
    }
    xd_uav_track::MultiTrackManager* tracker = tracker_it->second.get();
    int target_id = request.target_id;
    if (request.use_normalized_roi) {
      const double rx1 = request.normalized_roi[0];
      const double ry1 = request.normalized_roi[1];
      const double rx2 = rx1 + request.normalized_roi[2];
      const double ry2 = ry1 + request.normalized_roi[3];
      double best_iou = 0.0;
      target_id = -1;
      for (const auto& track : source_tracks.tracks) {
        const double x1 = track.normalized_bbox[0] -
            0.5 * track.normalized_bbox[2];
        const double y1 = track.normalized_bbox[1] -
            0.5 * track.normalized_bbox[3];
        const double x2 = x1 + track.normalized_bbox[2];
        const double y2 = y1 + track.normalized_bbox[3];
        const double intersection = std::max(0.0, std::min(rx2, x2) -
            std::max(rx1, x1)) * std::max(0.0, std::min(ry2, y2) -
            std::max(ry1, y1));
        const double union_area = request.normalized_roi[2] *
            request.normalized_roi[3] + track.normalized_bbox[2] *
            track.normalized_bbox[3] - intersection;
        const double iou = union_area > 1e-9 ? intersection / union_area : 0.0;
        if (iou > best_iou) {
          best_iou = iou;
          target_id = track.track_id;
        }
      }
    }
    bool found = false;
    for (const auto& track : source_tracks.tracks) {
      if (track.track_id == target_id) found = true;
    }
    if (!found) {
      response.success = false;
      response.message = "requested track is not active";
      response.selected_target_id = tracker->selectedTrackId();
      return true;
    }
    int local_target_id = -1;
    for (const auto& mapping : global_id_by_local_)
      if (mapping.first.first == source && mapping.second == target_id) {
        local_target_id = mapping.first.second;
        break;
      }
    if (local_target_id < 0) {
      response.success = false;
      response.message = "global target has no local track in requested source";
      response.selected_target_id = -1;
      return true;
    }
    tracker->setSelectedTrackId(local_target_id);
    auto_selection_suppressed_ = true;
    target_selection_mode_ = "follow";
    std_msgs::String follow_mode_message;
    follow_mode_message.data = target_selection_mode_;
    target_mode_publisher_.publish(follow_mode_message);
    locked_global_id_ = target_id;
    auto selected_identity = global_entities_.find(target_id);
    if (selected_identity != global_entities_.end())
      selected_identity->second.provisional = false;
    locked_target_last_seen_ = ros::WallTime::now();
    active_source_ = source;
    latest_tracks_ = source_tracks;
    tracker_active_ = request.start_tracking;
    controller_->reset();
    metric_identity_valid_ = false;
    response.success = true;
    response.message = tracker_active_ ? "track selected and tracking started"
                                       : "track selected; tracker remains stopped";
    response.selected_target_id = target_id;
    return true;
  }

  bool setTargetMode(xd_uav_track::SetTargetMode::Request& request,
                     xd_uav_track::SetTargetMode::Response& response) {
    if (request.mode == "follow") {
      xd_uav_track::SelectTrack::Request select_request;
      xd_uav_track::SelectTrack::Response select_response;
      select_request.target_id = request.target_id;
      select_request.start_tracking = true;
      select_request.use_normalized_roi = false;
      select_request.image_source = request.image_source;
      select_request.capture_timestamp = request.capture_timestamp;
      selectTrack(select_request, select_response);
      response.success = select_response.success;
      response.message = select_response.message;
      response.selected_target_id = select_response.selected_target_id;
      return true;
    }
    if (request.mode != "all" && request.mode != "global") {
      response.success = false;
      response.message = "mode must be 'all' or 'follow'";
      response.selected_target_id = -1;
      return true;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    auto_selection_suppressed_ = true;
    target_selection_mode_ = "all";
    std_msgs::String all_mode_message;
    all_mode_message.data = target_selection_mode_;
    target_mode_publisher_.publish(all_mode_message);
    locked_global_id_ = -1;
    locked_target_last_seen_ = ros::WallTime();
    for (auto& entry : source_trackers_)
      entry.second->setSelectedTrackId(-1);
    for (auto& entry : latest_tracks_by_source_) {
      for (auto& track : entry.second.tracks) track.selected = false;
      if (entry.first == active_source_) {
        latest_tracks_ = entry.second;
        tracks_publisher_.publish(entry.second);
      }
      tracks_by_source_publisher_.publish(entry.second);
    }
    controller_->reset();
    metric_identity_valid_ = false;
    response.success = true;
    response.message = "global multi-target search enabled; no target is selected for flight follow";
    response.selected_target_id = -1;
    return true;
  }

  bool emergencyStop(std_srvs::SetBool::Request& request,
                     std_srvs::SetBool::Response& response) {
    std::lock_guard<std::mutex> lock(mutex_);
    emergency_stop_active_ = request.data;
    if (emergency_stop_active_) {
      controller_->reset();
      metric_identity_valid_ = false;
    }
    response.success = true;
    response.message = emergency_stop_active_ ? "emergency stop enabled"
                                               : "emergency stop released";
    ROS_WARN("[xd_uav_track] %s", response.message.c_str());
    return true;
  }

  void publishReference(const xd_uav_track::TrackVelocity& command,
                        const bool state_valid, const ros::Time& stamp,
                        bool* published) {
    *published = false;
    if (!publish_control_reference_ || !state_valid) return;
    if (!command.valid && command.release_reference_on_invalid) {
      // Do not command a fixed-wing to zero velocity. Stop refreshing the
      // stream and let xd_uav_controller's reference timeout enter loiter.
      reference_owned_ = false;
      return;
    }
    if (!command.valid && !reference_owned_) return;

    const double forward = command.valid ? command.forward : 0.0;
    const double left = command.valid ? command.left : 0.0;
    const double up = command.valid ? command.up : 0.0;
    const double yaw_rate = command.valid ? command.yaw_rate : 0.0;
    mavros_msgs::PositionTarget reference;
    reference.header.stamp = stamp;
    reference.header.frame_id = state_frame_;
    // xd_uav_controller requires this MAVROS enum but consumes ROS ENU values.
    reference.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    reference.type_mask =
        mavros_msgs::PositionTarget::IGNORE_PX |
        mavros_msgs::PositionTarget::IGNORE_PY |
        mavros_msgs::PositionTarget::IGNORE_PZ |
        mavros_msgs::PositionTarget::IGNORE_AFX |
        mavros_msgs::PositionTarget::IGNORE_AFY |
        mavros_msgs::PositionTarget::IGNORE_AFZ |
        mavros_msgs::PositionTarget::IGNORE_YAW;
    if (command.position_reference_valid) {
      reference.type_mask &= static_cast<uint16_t>(
          ~(mavros_msgs::PositionTarget::IGNORE_PX |
            mavros_msgs::PositionTarget::IGNORE_PY |
            mavros_msgs::PositionTarget::IGNORE_PZ));
      reference.position.x = command.position_reference[0];
      reference.position.y = command.position_reference[1];
      reference.position.z = command.position_reference[2];
    }
    if (!command.use_yaw_rate) {
      reference.type_mask |= mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
    }
    reference.velocity.x =
        std::cos(state_yaw_) * forward - std::sin(state_yaw_) * left;
    reference.velocity.y =
        std::sin(state_yaw_) * forward + std::cos(state_yaw_) * left;
    reference.velocity.z = up;
    reference.yaw_rate = command.use_yaw_rate ? yaw_rate : 0.0;
    reference_publisher_.publish(reference);
    // An invalid command after owning the reference publishes exactly one
    // zero reference, then releases ownership.
    reference_owned_ = command.valid;
    *published = true;
  }

  void timerCallback(const ros::TimerEvent&) {
    std::lock_guard<std::mutex> lock(mutex_);
    const ros::WallTime wall_now = ros::WallTime::now();
    const ros::Time stamp = ros::Time::now();
    pruneSources(wall_now);
    if (perception_only_) {
      xd_uav_track::TrackStatus status;
      status.header.stamp = stamp;
      status.track_id = -1;
      status.tracking_state = "perception_only";
      status.invalid_reason = "flight guidance disabled; see perception/state for visual lock state";
      status_publisher_.publish(status);
      diagnostic_msgs::DiagnosticArray diagnostics;
      diagnostics.header.stamp = stamp;
      diagnostic_msgs::DiagnosticStatus runtime;
      runtime.name = ros::this_node::getName() + "/perception_storage";
      runtime.level = diagnostic_msgs::DiagnosticStatus::OK;
      runtime.message = "perception_only; flight outputs disabled";
      const auto add_count = [&](const std::string& key, std::size_t count) {
        diagnostic_msgs::KeyValue value;
        value.key = key; value.value = std::to_string(count);
        runtime.values.push_back(value);
      };
      add_count("sources", source_trackers_.size());
      add_count("maximum_sources", maximum_sources_);
      add_count("global_entities", global_entities_.size());
      add_count("maximum_global_entities", tracklet_stitch_maximum_identities_);
      add_count("local_mappings", global_id_by_local_.size());
      add_count("visual_identities", visual_identities_.size());
      add_count("pending_stitches", pending_tracklet_stitches_.size());
      diagnostics.status.push_back(runtime);
      runtime_diagnostics_publisher_.publish(diagnostics);
      return;  // No controller computation, velocity, follower or reference publication.
    }
    const bool state_valid = have_state_ &&
        (wall_now - state_receive_time_).toSec() <= state_timeout_sec_;
    const bool gimbal_state_valid = have_gimbal_state_ &&
        (wall_now - gimbal_state_receive_time_).toSec() <=
            gimbal_timeout_sec_ &&
        (!gimbal_state_data_.status_valid ||
         (wall_now - gimbal_status_receive_time_).toSec() <=
             gimbal_timeout_sec_) &&
        (!gimbal_state_data_.status_valid ||
         (gimbal_state_data_.healthy && gimbal_state_data_.tracking_active &&
          gimbal_state_data_.control_authority_available));
    xd_uav_track::FollowerProfile effective_profile = requested_profile_;
    gimbal_fallback_active_ = false;
    if (gimbal_fallback_enabled_ && !gimbal_state_valid) {
      if (requested_profile_ ==
          xd_uav_track::FollowerProfile::kGimbalVelocityChase) {
        effective_profile = xd_uav_track::FollowerProfile::kVelocityChase;
        gimbal_fallback_active_ = true;
      } else if (requested_profile_ ==
                 xd_uav_track::FollowerProfile::kGimbalVelocityVector) {
        effective_profile = xd_uav_track::FollowerProfile::kVelocityDistance;
        gimbal_fallback_active_ = true;
      }
    }
    if (controller_->profile() != effective_profile) {
      const auto old_effective_profile = controller_->profile();
      controller_->setProfile(effective_profile);
      const bool fixed_wing_transition =
          xd_uav_track::isFixedWingProfile(old_effective_profile) ||
          xd_uav_track::isFixedWingProfile(effective_profile);
      if (have_last_command_ && last_command_.valid &&
          profile_blend_duration_sec_ > 0.0 && !fixed_wing_transition) {
        blend_active_ = true;
        blend_start_ = wall_now;
        blend_source_ = last_command_;
      }
    }
    xd_uav_track::TrackVelocity command =
        controller_->compute(wall_now.toSec());
    if (blend_active_ && command.valid && blend_source_.valid) {
      const double ratio = std::max(0.0, std::min(1.0,
          (wall_now - blend_start_).toSec() /
              std::max(1e-6, profile_blend_duration_sec_)));
      command.forward = blend_source_.forward +
          ratio * (command.forward - blend_source_.forward);
      command.left = blend_source_.left +
          ratio * (command.left - blend_source_.left);
      command.up = blend_source_.up +
          ratio * (command.up - blend_source_.up);
      command.yaw_rate = blend_source_.yaw_rate +
          ratio * (command.yaw_rate - blend_source_.yaw_rate);
      if (ratio >= 1.0) blend_active_ = false;
    }
    if (!tracker_active_) {
      command.valid = false;
      command.target_visible = false;
      command.track_id = -1;
      command.forward = 0.0;
      command.left = 0.0;
      command.up = 0.0;
      command.yaw_rate = 0.0;
      command.tracking_state = "stopped";
      command.invalid_reason = "tracker is waiting for StartTracker";
    }
    if (emergency_stop_active_) {
      command.valid = false;
      command.target_visible = false;
      command.forward = 0.0;
      command.left = 0.0;
      command.up = 0.0;
      command.yaw_rate = 0.0;
      command.release_reference_on_invalid =
          xd_uav_track::isFixedWingProfile(requested_profile_);
      command.tracking_state = "emergency_stop";
      command.invalid_reason = "emergency stop service is active";
    }

    geometry_msgs::TwistStamped body_velocity;
    body_velocity.header.stamp = stamp;
    body_velocity.header.frame_id = body_frame_;
    if (command.valid) {
      body_velocity.twist.linear.x = command.forward;
      body_velocity.twist.linear.y = command.left;
      body_velocity.twist.linear.z = command.up;
      body_velocity.twist.angular.z = command.yaw_rate;
    }
    body_velocity_publisher_.publish(body_velocity);

    xd_uav_track::FollowerCommand follower_command;
    follower_command.header.stamp = stamp;
    follower_command.header.frame_id = body_frame_;
    follower_command.follower_profile = command.profile;
    follower_command.command_valid = command.valid;
    follower_command.target_visible = command.target_visible;
    follower_command.target_predicted = command.target_predicted;
    follower_command.uncertainty_limited = command.uncertainty_limited;
    follower_command.relative_state_active = command.relative_state_active;
    follower_command.metric_target_valid = command.metric_target_valid;
    follower_command.metric_active = command.metric_active;
    follower_command.orbit_active = command.orbit_active;
    follower_command.velocity_body.x = command.forward;
    follower_command.velocity_body.y = command.left;
    follower_command.velocity_body.z = command.up;
    follower_command.yaw_rate = command.yaw_rate;
    follower_command.metric_radial_error_m = command.metric_radial_error_m;
    follower_command.metric_effective_radius_m =
        command.metric_effective_radius_m;
    follower_command.yaw_rate_enabled = command.use_yaw_rate;
    follower_command.tracking_state = command.tracking_state;
    follower_command.tracking_quality = command.tracking_quality;
    follower_command.uncertainty_scale = command.uncertainty_scale;
    follower_command.association_method = command.association_method;
    follower_command.invalid_reason = command.invalid_reason;
    follower_command_publisher_.publish(follower_command);

    bool reference_published = false;
    publishReference(command, state_valid, stamp, &reference_published);

    xd_uav_track::TrackStatus status;
    status.header.stamp = stamp;
    status.tracker_active = tracker_active_;
    status.target_visible = command.target_visible;
    status.target_predicted = command.target_predicted;
    status.command_valid = command.valid;
    status.state_valid = state_valid;
    status.gimbal_state_valid = gimbal_state_valid;
    status.control_reference_published = reference_published;
    status.uncertainty_limited = command.uncertainty_limited;
    status.relative_state_active = command.relative_state_active;
    status.metric_target_valid = command.metric_target_valid;
    status.metric_active = command.metric_active;
    status.orbit_active = command.orbit_active;
    status.gimbal_fallback_active = gimbal_fallback_active_;
    status.emergency_stop_active = emergency_stop_active_;
    status.track_id = command.track_id;
    status.follower_profile = command.profile;
    status.requested_profile =
        xd_uav_track::followerProfileName(requested_profile_);
    status.lateral_guidance_mode = command.lateral_guidance_mode;
    status.tracking_state = command.tracking_state;
    status.error_x = command.error_x;
    status.error_y = command.error_y;
    status.center_x = command.center_x;
    status.center_y = command.center_y;
    status.center_velocity_x = command.center_velocity_x;
    status.center_velocity_y = command.center_velocity_y;
    status.target_size_ratio = command.target_size_ratio;
    status.input_age_sec = command.input_age_sec;
    status.target_loss_duration_sec = command.target_loss_duration_sec;
    status.tracking_quality = command.tracking_quality;
    status.uncertainty_scale = command.uncertainty_scale;
    status.command_forward = command.forward;
    status.command_left = command.left;
    status.command_up = command.up;
    status.command_yaw_rate = command.yaw_rate;
    status.metric_radial_error_m = command.metric_radial_error_m;
    status.metric_effective_radius_m = command.metric_effective_radius_m;
    status.association_method = command.association_method;
    status.invalid_reason = command.invalid_reason;
    if (command.valid && publish_control_reference_ && !state_valid) {
      status.invalid_reason =
          "body command is valid but estimator odometry is stale";
    }
    status_publisher_.publish(status);
    last_command_ = command;
    have_last_command_ = command.valid;
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber detections_subscriber_;
  ros::Subscriber state_subscriber_;
  ros::Subscriber gimbal_state_subscriber_;
  ros::Subscriber gimbal_status_subscriber_;
  ros::Subscriber metric_target_subscriber_;
  ros::CallbackQueue perception_queue_;
  ros::CallbackQueue control_queue_;
  std::unique_ptr<ros::AsyncSpinner> perception_spinner_;
  std::unique_ptr<ros::AsyncSpinner> control_spinner_;
  ros::Publisher body_velocity_publisher_;
  ros::Publisher follower_command_publisher_;
  ros::Publisher reference_publisher_;
  ros::Publisher status_publisher_;
  ros::Publisher runtime_diagnostics_publisher_;
  ros::Publisher tracks_publisher_;
  ros::Publisher tracks_by_source_publisher_;
  ros::Publisher target_pixels_publisher_;
  ros::Publisher observations_publisher_;
  ros::Publisher target_mode_publisher_;
  ros::Publisher visual_tracks_publisher_;
  ros::ServiceServer visual_identity_service_;
  std::map<int, VisualIdentity> visual_identities_;
  std::map<std::string, std::deque<xd_uav_track::TrackStateArray>> identity_detection_history_;
  ros::ServiceServer profile_service_;
  ros::ServiceServer start_tracker_service_;
  ros::ServiceServer select_track_service_;
  ros::ServiceServer set_target_mode_service_;
  ros::ServiceServer emergency_stop_service_;
  ros::Timer timer_;
  std::unique_ptr<xd_uav_track::TrackController> controller_;
  xd_uav_track::MultiTrackConfig multi_config_;
  std::map<std::string, std::unique_ptr<xd_uav_track::MultiTrackManager>>
      source_trackers_;
  std::map<std::string, xd_uav_track::TrackStateArray>
      latest_tracks_by_source_;
  std::map<std::string, ros::WallTime> source_last_receive_;
  std::map<std::string, ros::Time> source_last_stamp_;
  std::map<std::pair<std::string, int>, int> global_id_by_local_;
  std::map<std::pair<std::string, int>, PendingTrackletStitch>
      pending_tracklet_stitches_;
  std::map<int, GlobalEntity> global_entities_;
  xd_uav_track::TrackletStitchConfig tracklet_stitch_config_;
  int tracklet_stitch_confirmation_frames_{3};
  double tracklet_stitch_max_confirmation_gap_sec_{0.50};
  int tracklet_stitch_maximum_identities_{512};
  int next_global_id_{1};
  int locked_global_id_{-1};
  bool global_identity_merge_enabled_{true};
  ros::WallTime locked_target_last_seen_;
  double locked_target_release_timeout_sec_{60.0};
  double cross_source_fusion_max_time_delta_sec_{0.05};
  std::array<double, 3> vehicle_position_{{0.0, 0.0, 0.0}};
  geometry_msgs::Quaternion vehicle_orientation_;
  ros::Time vehicle_stamp_;
  std::mutex mutex_;
  bool publish_control_reference_{true};
  bool perception_only_{false};
  std::vector<std::string> allowed_image_sources_;
  int maximum_sources_{8};
  double source_idle_ttl_sec_{30.0};
  ros::WallTime last_storage_prune_;
  bool tracker_active_{false};
  bool have_state_{false};
  bool have_gimbal_state_{false};
  bool reference_owned_{false};
  bool auto_select_track_{true};
  bool auto_selection_suppressed_{false};
  std::string target_selection_mode_{"auto"};
  bool gimbal_fallback_enabled_{true};
  bool gimbal_fallback_active_{false};
  bool emergency_stop_active_{false};
  bool blend_active_{false};
  bool have_last_command_{false};
  double publish_rate_{30.0};
  double state_timeout_sec_{0.30};
  double gimbal_timeout_sec_{0.20};
  double profile_switch_min_interval_sec_{0.50};
  double profile_blend_duration_sec_{0.50};
  double state_yaw_{0.0};
  ros::WallTime state_receive_time_;
  ros::WallTime gimbal_state_receive_time_;
  ros::WallTime gimbal_status_receive_time_;
  xd_uav_track::GimbalStateData gimbal_state_data_;
  ros::WallTime last_profile_change_;
  ros::WallTime blend_start_;
  xd_uav_track::FollowerProfile requested_profile_{
      xd_uav_track::FollowerProfile::kVelocityChase};
  xd_uav_track::TrackVelocity last_command_;
  xd_uav_track::TrackVelocity blend_source_;
  xd_uav_track::TrackStateArray latest_tracks_;
  std::string state_frame_;
  std::string body_frame_{"base_link"};
  std::string lateral_guidance_name_{"coordinated_turn"};
  std::string accepted_image_source_;
  std::string source_policy_{"auto_handover"};
  std::string active_source_;
  double source_handover_timeout_sec_{0.75};
  double select_capture_tolerance_sec_{0.10};
  double source_clock_reset_jump_sec_{5.0};
  bool source_quality_handover_enabled_{false};
  double source_min_active_quality_{0.25};
  double source_min_incoming_quality_{0.55};
  double source_quality_margin_{0.10};
  double source_handover_cooldown_sec_{1.0};
  ros::WallTime last_source_switch_;
  bool reject_out_of_order_timestamps_{true};
  bool multi_source_fusion_enabled_{false};
  bool source_identity_gate_enabled_{true};
  bool source_cross_class_reject_{true};
  double source_identity_max_age_sec_{0.50};
  double source_identity_max_distance_m_{25.0};
  bool metric_identity_valid_{false};
  int metric_identity_class_id_{-1};
  std::uint64_t model_epoch_{0};
  std::vector<std::string> model_class_names_;
  std::string metric_identity_source_;
  double metric_identity_time_{0.0};
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "xd_uav_track");
  XdUavTrackNode node;
  ros::waitForShutdown();
  return 0;
}
