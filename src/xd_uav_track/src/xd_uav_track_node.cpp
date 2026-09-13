#include <algorithm>
#include <cmath>
#include <memory>
#include <map>
#include <mutex>
#include <string>

#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/PositionTarget.h>
#include <nav_msgs/Odometry.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <boost/bind.hpp>
#include <std_srvs/SetBool.h>
#include <xd_uav_track/DetectionArray.h>
#include <xd_uav_track/FollowerCommand.h>
#include <xd_uav_track/GimbalStatus.h>
#include <xd_uav_track/GimbalState.h>
#include <xd_uav_track/SelectTrack.h>
#include <xd_uav_track/SetProfile.h>
#include <xd_uav_track/StartTracker.h>
#include <xd_uav_track/TrackStateArray.h>
#include <xd_uav_track/TrackStatus.h>
#include <xd_uav_track/multi_track_manager.hpp>
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
    gimbal_timeout_sec_ = config.gimbal_input_timeout_sec;
    multi_source_fusion_enabled_ = config.target_guidance.multi_source_fusion_enabled;
    controller_.reset(new xd_uav_track::TrackController(config));
    requested_profile_ = config.profile;

    xd_uav_track::MultiTrackConfig multi_config;
    private_nh_.param("tracker/selection/auto_select", auto_select_track_, true);
    private_nh_.param("tracker/image_source",
                      accepted_image_source_, std::string());
    private_nh_.param("tracker/source_policy", source_policy_,
                      std::string("auto_handover"));
    private_nh_.param("tracker/source_aware", source_identity_gate_enabled_,
                      true);
    private_nh_.param("tracker/source_cross_class_reject",
                      source_cross_class_reject_, true);
    private_nh_.param("tracker/source_identity_max_age_sec",
                      source_identity_max_age_sec_,
                      config.target_guidance.multi_source_max_age_sec);
    private_nh_.param("tracker/source_handover_distance_m",
                      source_identity_max_distance_m_, 25.0);
    private_nh_.param("tracker/source_handover_timeout_sec",
                      source_handover_timeout_sec_, 0.75);
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
    private_nh_.param("tracker/association/appearance_cosine",
                      multi_config.appearance_minimum_cosine,
                      multi_config.appearance_minimum_cosine);
    private_nh_.param("tracker/kalman/process_noise",
                      multi_config.process_noise, multi_config.process_noise);
    private_nh_.param("tracker/kalman/measurement_noise",
                      multi_config.measurement_noise, multi_config.measurement_noise);
    multi_config_ = multi_config;

    private_nh_.param("runtime/publish_rate_hz", publish_rate_, 30.0);
    private_nh_.param("runtime/state_timeout_sec", state_timeout_sec_, 0.30);
    private_nh_.param("runtime/publish_control_reference",
                      publish_control_reference_, true);
    private_nh_.param("tracker/enabled_at_startup", tracker_active_, false);
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
    std::string body_velocity_topic{"track/velocity_body"};
    std::string follower_command_topic{"track/command"};
    std::string gimbal_state_topic{"track/gimbal_state"};
    std::string gimbal_status_topic{"track/gimbal_status"};
    std::string status_topic{"track/status"};
    std::string reference_topic{"control/reference/setpoint"};
    private_nh_.param("interfaces/input/detections", detections_topic,
                      detections_topic);
    private_nh_.param("interfaces/output/tracks", tracks_topic, tracks_topic);
    private_nh_.param("interfaces/output/body_velocity", body_velocity_topic,
                      body_velocity_topic);
    private_nh_.param("interfaces/output/follower_command", follower_command_topic,
                      follower_command_topic);
    private_nh_.param("interfaces/input/gimbal_state", gimbal_state_topic,
                      gimbal_state_topic);
    private_nh_.param("interfaces/input/gimbal_status", gimbal_status_topic,
                      gimbal_status_topic);
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
    body_velocity_publisher_ = nh_.advertise<geometry_msgs::TwistStamped>(
        body_velocity_topic, 10);
    follower_command_publisher_ = nh_.advertise<xd_uav_track::FollowerCommand>(
        follower_command_topic, 10);
    status_publisher_ = nh_.advertise<xd_uav_track::TrackStatus>(
        status_topic, 10);
    tracks_publisher_ = nh_.advertise<xd_uav_track::TrackStateArray>(
        tracks_topic, 10);
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

    ROS_INFO("[xd_uav_track] tracker + follower started at %.1f Hz; profile=%s; guidance=%s; reference=%s; tracking=%s",
             publish_rate_,
             xd_uav_track::followerProfileName(controller_->profile()),
             lateral_guidance_name_.c_str(),
             publish_control_reference_ ? "enabled" : "disabled",
             tracker_active_ ? "active" : "waiting for StartTracker");
  }

 private:
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
      if (reason != nullptr) *reason =
          "cross-source class conflicts with the fresh selected target";
      return false;
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
    if (!accepted_image_source_.empty() && source != accepted_image_source_) return;
    const ros::Time stamp = message->header.stamp;
    auto stamp_it = source_last_stamp_.find(source);
    if (reject_out_of_order_timestamps_ && !stamp.isZero() &&
        stamp_it != source_last_stamp_.end() && stamp < stamp_it->second) {
      // A backwards jump larger than the configured reset threshold denotes a
      // restarted sensor clock; clear only that source instead of poisoning
      // the other camera's track history.
      const double jump = (stamp_it->second - stamp).toSec();
      if (jump <= source_clock_reset_jump_sec_) return;
      auto tracker_it = source_trackers_.find(source);
      if (tracker_it != source_trackers_.end()) tracker_it->second->reset();
    }
    if (!stamp.isZero()) source_last_stamp_[source] = stamp;
    const unsigned int image_width = message->image_width;
    const unsigned int image_height = message->image_height;
    if (image_width == 0 || image_height == 0) {
      ROS_WARN_THROTTLE(2.0, "[xd_uav_track] DetectionArray image size is zero");
      return;
    }
    xd_uav_track::MultiTrackManager* tracker = trackerForSource(source);
    if (message->command == "reset") {
      tracker->reset();
      controller_->reset();
      metric_identity_valid_ = false;
    } else if (message->command == "start_track") {
      tracker_active_ = true;
    } else if (message->command == "stop_track") {
      tracker_active_ = false;
      controller_->reset();
      metric_identity_valid_ = false;
    }

    auto frame = tracker->update(
        *message, static_cast<int>(image_width),
        static_cast<int>(image_height), source);
    latest_tracks_by_source_[source] = frame.tracks;
    source_last_receive_[source] = ros::WallTime::now();
    const std::string previous_active_source = active_source_;
    if (active_source_.empty()) active_source_ = source;
    const ros::WallTime wall_now = ros::WallTime::now();
    auto active_receive = source_last_receive_.find(active_source_);
    const bool active_stale = active_receive == source_last_receive_.end() ||
        (wall_now - active_receive->second).toSec() > source_handover_timeout_sec_;
    const bool manual_policy = source_policy_ == "manual" ||
        source_policy_ == "fixed";
    const bool fixed_first_policy = source_policy_ == "fixed_first";
    if (!manual_policy && active_stale) active_source_ = source;
    if (fixed_first_policy && source.find("fixed") != std::string::npos) {
      active_source_ = source;
    }
    if (!manual_policy && !fixed_first_policy && source != active_source_ &&
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
    if (selected_id < 0 && auto_select_track_) {
      float best_quality = -1.0F;
      for (const auto& track : frame.tracks.tracks) {
        if (track.lifecycle_state == "confirmed" &&
            track.tracking_quality > best_quality) {
          best_quality = track.tracking_quality;
          selected_id = track.track_id;
        }
      }
      if (selected_id >= 0) tracker->setSelectedTrackId(selected_id);
    }

    const xd_uav_track::TrackState* selected = nullptr;
    for (auto& track : frame.tracks.tracks) {
      track.selected = track.track_id == selected_id;
      if (track.selected &&
          (track.lifecycle_state == "confirmed" ||
           track.lifecycle_state == "occluded")) {
        selected = &track;
      }
    }
    if (source == active_source_) {
      latest_tracks_ = frame.tracks;
      tracks_publisher_.publish(frame.tracks);
    }
    if (!tracker_active_) return;
    if (selected == nullptr) {
      controller_->clearMeasurement("no confirmed selected track");
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
          "gm_velocity_chase, gm_velocity_vector, fw_velocity_vector";
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
          previous_profile == xd_uav_track::FollowerProfile::kFixedWingVelocityVector ||
          requested_profile == xd_uav_track::FollowerProfile::kFixedWingVelocityVector;
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
    const bool changed = tracker_active_ != request.start;
    if (changed) {
      tracker_active_ = request.start;
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
    tracker->setSelectedTrackId(target_id);
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
          old_effective_profile ==
              xd_uav_track::FollowerProfile::kFixedWingVelocityVector ||
          effective_profile ==
              xd_uav_track::FollowerProfile::kFixedWingVelocityVector;
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
      command.release_reference_on_invalid = requested_profile_ ==
          xd_uav_track::FollowerProfile::kFixedWingVelocityVector;
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
    follower_command.velocity_body.x = command.forward;
    follower_command.velocity_body.y = command.left;
    follower_command.velocity_body.z = command.up;
    follower_command.yaw_rate = command.yaw_rate;
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
  ros::CallbackQueue perception_queue_;
  ros::CallbackQueue control_queue_;
  std::unique_ptr<ros::AsyncSpinner> perception_spinner_;
  std::unique_ptr<ros::AsyncSpinner> control_spinner_;
  ros::Publisher body_velocity_publisher_;
  ros::Publisher follower_command_publisher_;
  ros::Publisher reference_publisher_;
  ros::Publisher status_publisher_;
  ros::Publisher tracks_publisher_;
  ros::ServiceServer profile_service_;
  ros::ServiceServer start_tracker_service_;
  ros::ServiceServer select_track_service_;
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
  std::mutex mutex_;
  bool publish_control_reference_{true};
  bool tracker_active_{false};
  bool have_state_{false};
  bool have_gimbal_state_{false};
  bool reference_owned_{false};
  bool auto_select_track_{true};
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
  std::string metric_identity_source_;
  double metric_identity_time_{0.0};
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "xd_uav_track");
  XdUavTrackNode node;
  ros::waitForShutdown();
  return 0;
}
