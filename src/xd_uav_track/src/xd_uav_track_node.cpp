#include <algorithm>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>

#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/PositionTarget.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_srvs/SetBool.h>
#include <xd_uav_track/BoundingBox.h>
#include <xd_uav_track/DetectionArray.h>
#include <xd_uav_track/FollowerCommand.h>
#include <xd_uav_track/GimbalState.h>
#include <xd_uav_track/SelectTrack.h>
#include <xd_uav_track/SetProfile.h>
#include <xd_uav_track/StartTracker.h>
#include <xd_uav_track/TrackStateArray.h>
#include <xd_uav_track/TrackStatus.h>
#include <xd_uav_track/multi_track_manager.hpp>
#include <xd_uav_track/track_controller.hpp>

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
    gimbal_timeout_sec_ = config.gimbal_input_timeout_sec;
    controller_.reset(new xd_uav_track::TrackController(config));
    requested_profile_ = config.profile;

    xd_uav_track::MultiTrackConfig multi_config;
    private_nh_.param("tracker/multi_target/enabled", multi_target_enabled_, false);
    private_nh_.param("tracker/multi_target/auto_select", auto_select_track_, true);
    private_nh_.param("tracker/image_geometry/use_camera_info_fallback",
                      use_camera_info_fallback_, true);
    private_nh_.param("tracker/multi_target/image_source",
                      accepted_image_source_, std::string());
    private_nh_.param("tracker/multi_target/lifecycle/confirmation_hits",
                      multi_config.confirmation_hits, multi_config.confirmation_hits);
    private_nh_.param("tracker/multi_target/lifecycle/occlusion_frames",
                      multi_config.occlusion_frames, multi_config.occlusion_frames);
    private_nh_.param("tracker/multi_target/lifecycle/removal_frames",
                      multi_config.removal_frames, multi_config.removal_frames);
    private_nh_.param("tracker/multi_target/limits/maximum_tracks",
                      multi_config.maximum_tracks, multi_config.maximum_tracks);
    private_nh_.param("tracker/multi_target/limits/maximum_embedding_dimension",
                      multi_config.maximum_embedding_dimension,
                      multi_config.maximum_embedding_dimension);
    private_nh_.param("tracker/multi_target/confidence/new_track",
                      multi_config.minimum_new_track_confidence,
                      multi_config.minimum_new_track_confidence);
    private_nh_.param("tracker/multi_target/confidence/update",
                      multi_config.minimum_update_confidence,
                      multi_config.minimum_update_confidence);
    private_nh_.param("tracker/multi_target/association/iou_threshold",
                      multi_config.association_iou_threshold,
                      multi_config.association_iou_threshold);
    private_nh_.param("tracker/multi_target/association/center_distance",
                      multi_config.association_center_distance,
                      multi_config.association_center_distance);
    private_nh_.param("tracker/multi_target/association/appearance_cosine",
                      multi_config.appearance_minimum_cosine,
                      multi_config.appearance_minimum_cosine);
    private_nh_.param("tracker/multi_target/kalman/process_noise",
                      multi_config.process_noise, multi_config.process_noise);
    private_nh_.param("tracker/multi_target/kalman/measurement_noise",
                      multi_config.measurement_noise, multi_config.measurement_noise);
    multi_tracker_.reset(new xd_uav_track::MultiTrackManager(multi_config));

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
    std::string uav_name{"uav1"};
    private_nh_.param("vehicle/uav_name", uav_name, uav_name);
    private_nh_.param("vehicle/body_frame", body_frame_, uav_name + "/base_link");
    std::string state_topic = "/" + uav_name +
        "/state_estimator/main/odom";
    std::string bounding_box_topic = "/" + uav_name +
        "/track/bounding_box";
    std::string detections_topic = "/" + uav_name + "/track/detections";
    std::string camera_info_topic = "/" + uav_name + "/camera/camera_info";
    std::string tracks_topic = "/" + uav_name + "/track/tracks";
    std::string body_velocity_topic = "/" + uav_name +
        "/track/velocity_body";
    std::string follower_command_topic = "/" + uav_name +
        "/track/command";
    std::string gimbal_state_topic = "/" + uav_name +
        "/track/gimbal_state";
    std::string status_topic = "/" + uav_name + "/track/status";
    std::string reference_topic = "/" + uav_name +
        "/control/reference/setpoint";
    private_nh_.param("interfaces/input/bounding_box", bounding_box_topic,
                      bounding_box_topic);
    private_nh_.param("interfaces/input/detections", detections_topic,
                      detections_topic);
    private_nh_.param("interfaces/input/camera_info", camera_info_topic,
                      camera_info_topic);
    private_nh_.param("interfaces/output/tracks", tracks_topic, tracks_topic);
    private_nh_.param("interfaces/output/body_velocity", body_velocity_topic,
                      body_velocity_topic);
    private_nh_.param("interfaces/output/follower_command", follower_command_topic,
                      follower_command_topic);
    private_nh_.param("interfaces/input/gimbal_state", gimbal_state_topic,
                      gimbal_state_topic);
    private_nh_.param("interfaces/output/status", status_topic, status_topic);
    private_nh_.param("interfaces/input/vehicle_state", state_topic, state_topic);
    private_nh_.param("interfaces/output/control_reference", reference_topic,
                      reference_topic);

    box_subscriber_ = nh_.subscribe(
        bounding_box_topic, 1, &XdUavTrackNode::boxCallback, this,
        ros::TransportHints().tcpNoDelay());
    detections_subscriber_ = nh_.subscribe(
        detections_topic, 1, &XdUavTrackNode::detectionsCallback, this,
        ros::TransportHints().tcpNoDelay());
    camera_info_subscriber_ = nh_.subscribe(
        camera_info_topic, 1, &XdUavTrackNode::cameraInfoCallback, this,
        ros::TransportHints().tcpNoDelay());
    state_subscriber_ = nh_.subscribe(
        state_topic, 1, &XdUavTrackNode::stateCallback, this,
        ros::TransportHints().tcpNoDelay());
    gimbal_state_subscriber_ = nh_.subscribe(
        gimbal_state_topic, 1, &XdUavTrackNode::gimbalStateCallback, this,
        ros::TransportHints().tcpNoDelay());
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
    timer_ = nh_.createTimer(
        ros::Duration(1.0 / std::max(2.0, publish_rate_)),
        &XdUavTrackNode::timerCallback, this);

    ROS_INFO("[xd_uav_track] tracker + follower started at %.1f Hz; profile=%s; guidance=%s; reference=%s; tracking=%s",
             publish_rate_,
             xd_uav_track::followerProfileName(controller_->profile()),
             lateral_guidance_name_.c_str(),
             publish_control_reference_ ? "enabled" : "disabled",
             tracker_active_ ? "active" : "waiting for StartTracker");
  }

 private:
  void boxCallback(const xd_uav_track::BoundingBox::ConstPtr& message) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (multi_target_enabled_) return;
    if (!tracker_active_) return;
    if (!message->valid) {
      controller_->clearMeasurement("upstream marked target box invalid");
      return;
    }
    xd_uav_track::BoundingBoxMeasurement measurement;
    measurement.receive_time = ros::WallTime::now().toSec();
    measurement.image_width = message->image_width > 0 ? message->image_width
        : (use_camera_info_fallback_ ? camera_width_ : 0);
    measurement.image_height = message->image_height > 0 ? message->image_height
        : (use_camera_info_fallback_ ? camera_height_ : 0);
    measurement.x_min = message->x_min;
    measurement.y_min = message->y_min;
    measurement.x_max = message->x_max;
    measurement.y_max = message->y_max;
    measurement.confidence = message->confidence;
    measurement.track_id = message->track_id;
    std::string reason;
    if (!controller_->updateMeasurement(measurement, &reason)) {
      controller_->clearMeasurement(reason);
      ROS_WARN_THROTTLE(2.0, "[xd_uav_track] rejected target box: %s",
                        reason.c_str());
    }
  }

  void detectionsCallback(
      const xd_uav_track::DetectionArray::ConstPtr& message) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!multi_target_enabled_) return;
    if (!accepted_image_source_.empty() &&
        message->image_source != accepted_image_source_) return;
    const unsigned int image_width = message->image_width > 0
        ? message->image_width : (use_camera_info_fallback_ ? camera_width_ : 0);
    const unsigned int image_height = message->image_height > 0
        ? message->image_height : (use_camera_info_fallback_ ? camera_height_ : 0);
    if (image_width == 0 || image_height == 0) {
      ROS_WARN_THROTTLE(2.0, "[xd_uav_track] DetectionArray image size is zero");
      return;
    }
    if (message->command == "reset") {
      multi_tracker_->reset();
      controller_->reset();
    } else if (message->command == "start_track") {
      tracker_active_ = true;
    } else if (message->command == "stop_track") {
      tracker_active_ = false;
      controller_->reset();
    }

    auto frame = multi_tracker_->update(
        *message, static_cast<int>(image_width),
        static_cast<int>(image_height));
    int selected_id = multi_tracker_->selectedTrackId();
    if (selected_id < 0 && auto_select_track_) {
      float best_quality = -1.0F;
      for (const auto& track : frame.tracks.tracks) {
        if (track.lifecycle_state == "confirmed" &&
            track.tracking_quality > best_quality) {
          best_quality = track.tracking_quality;
          selected_id = track.track_id;
        }
      }
      if (selected_id >= 0) multi_tracker_->setSelectedTrackId(selected_id);
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
    latest_tracks_ = frame.tracks;
    tracks_publisher_.publish(frame.tracks);
    if (!tracker_active_) return;
    if (selected == nullptr) {
      controller_->clearMeasurement("no confirmed selected track");
      return;
    }

    xd_uav_track::BoundingBoxMeasurement measurement;
    measurement.receive_time = ros::WallTime::now().toSec();
    measurement.image_width = image_width;
    measurement.image_height = image_height;
    measurement.x_min = selected->bbox[0];
    measurement.y_min = selected->bbox[1];
    measurement.x_max = selected->bbox[2];
    measurement.y_max = selected->bbox[3];
    measurement.confidence = selected->confidence;
    measurement.track_id = selected->track_id;
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
    for (std::size_t i = 0; i < 3; ++i) {
      measurement.relative_position_body[i] =
          selected->relative_position_body[i];
      measurement.relative_velocity_body[i] =
          selected->relative_velocity_body[i];
    }
    std::string reason;
    if (!controller_->updateMeasurement(measurement, &reason)) {
      controller_->clearMeasurement(reason);
      ROS_WARN_THROTTLE(2.0, "[xd_uav_track] rejected selected track: %s",
                        reason.c_str());
    }
  }

  void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& message) {
    if (message->width == 0 || message->height == 0) return;
    std::lock_guard<std::mutex> lock(mutex_);
    camera_width_ = message->width;
    camera_height_ = message->height;
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
    state.valid = true;
    controller_->setVehicleState(state);
  }

  void gimbalStateCallback(
      const xd_uav_track::GimbalState::ConstPtr& message) {
    std::lock_guard<std::mutex> lock(mutex_);
    xd_uav_track::GimbalStateData state;
    state.receive_time = ros::WallTime::now().toSec();
    state.yaw = message->yaw_rad;
    state.pitch = message->pitch_rad;
    state.roll = message->roll_rad;
    state.valid = message->valid;
    controller_->setGimbalState(state);
    gimbal_state_receive_time_ = ros::WallTime::now();
    have_gimbal_state_ = message->valid;
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
    if (!multi_target_enabled_) {
      response.success = false;
      response.message = "multi-target tracking is disabled in track.yaml";
      response.selected_target_id = -1;
      return true;
    }
    int target_id = request.target_id;
    if (request.use_normalized_roi) {
      const double rx1 = request.normalized_roi[0];
      const double ry1 = request.normalized_roi[1];
      const double rx2 = rx1 + request.normalized_roi[2];
      const double ry2 = ry1 + request.normalized_roi[3];
      double best_iou = 0.0;
      target_id = -1;
      for (const auto& track : latest_tracks_.tracks) {
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
    for (const auto& track : latest_tracks_.tracks) {
      if (track.track_id == target_id) found = true;
    }
    if (!found) {
      response.success = false;
      response.message = "requested track is not active";
      response.selected_target_id = multi_tracker_->selectedTrackId();
      return true;
    }
    multi_tracker_->setSelectedTrackId(target_id);
    tracker_active_ = request.start_tracking;
    controller_->reset();
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
    if (emergency_stop_active_) controller_->reset();
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
            gimbal_timeout_sec_;
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
    status.multi_target_enabled = multi_target_enabled_;
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
  ros::Subscriber box_subscriber_;
  ros::Subscriber detections_subscriber_;
  ros::Subscriber camera_info_subscriber_;
  ros::Subscriber state_subscriber_;
  ros::Subscriber gimbal_state_subscriber_;
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
  std::unique_ptr<xd_uav_track::MultiTrackManager> multi_tracker_;
  std::mutex mutex_;
  bool publish_control_reference_{true};
  bool tracker_active_{false};
  bool have_state_{false};
  bool have_gimbal_state_{false};
  bool reference_owned_{false};
  bool multi_target_enabled_{false};
  bool auto_select_track_{true};
  bool use_camera_info_fallback_{true};
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
  unsigned int camera_width_{0};
  unsigned int camera_height_{0};
  ros::WallTime state_receive_time_;
  ros::WallTime gimbal_state_receive_time_;
  ros::WallTime last_profile_change_;
  ros::WallTime blend_start_;
  xd_uav_track::FollowerProfile requested_profile_{
      xd_uav_track::FollowerProfile::kVelocityChase};
  xd_uav_track::TrackVelocity last_command_;
  xd_uav_track::TrackVelocity blend_source_;
  xd_uav_track::TrackStateArray latest_tracks_;
  std::string state_frame_;
  std::string body_frame_{"uav1/fcu"};
  std::string lateral_guidance_name_{"coordinated_turn"};
  std::string accepted_image_source_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "xd_uav_track");
  XdUavTrackNode node;
  ros::spin();
  return 0;
}
