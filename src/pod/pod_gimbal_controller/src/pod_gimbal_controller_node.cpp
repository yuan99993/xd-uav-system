#include <algorithm>
#include <cmath>
#include <string>

#include <diagnostic_updater/diagnostic_updater.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tracker/NormalizedError.h>

#include <pod_msgs/GimbalCommand.h>
#include <pod_msgs/GimbalState.h>

namespace {
constexpr double kPi = 3.14159265358979323846;
double clamp(const double value, const double low, const double high) {
  return std::max(low, std::min(high, value));
}
}

class PodGimbalController {
 public:
  PodGimbalController() : private_nh_("~") {
    private_nh_.param("normalized_error_topic", error_topic_,
                      std::string("/tracker_node/normalized_error"));
    private_nh_.param("gimbal_state_topic", state_topic_,
                      std::string("/pod/gimbal/state"));
    private_nh_.param("mission_command_topic", command_topic_,
                      std::string("/pod/gimbal/mission_command"));
    private_nh_.param("body_frame", body_frame_, std::string("base_link"));
    private_nh_.param("gimbal_frame", gimbal_frame_, std::string("pod_gimbal_link"));
    private_nh_.param("optical_frame", optical_frame_,
                      std::string("pod_camera_optical_frame"));
    private_nh_.param("publish_dynamic_tf", publish_dynamic_tf_, true);
    private_nh_.param("tracking_enabled", tracking_enabled_, false);
    private_nh_.param("require_control_measurement_ready", require_measurement_, true);
    private_nh_.param("state_timeout_sec", state_timeout_sec_, 0.15);
    private_nh_.param("error_timeout_sec", error_timeout_sec_, 0.20);
    private_nh_.param("publish_rate", publish_rate_, 50.0);
    private_nh_.param("yaw_rate_gain", yaw_gain_, 1.2);
    private_nh_.param("pitch_rate_gain", pitch_gain_, 1.2);
    private_nh_.param("max_yaw_rate_deg_s", max_yaw_rate_, 30.0);
    private_nh_.param("max_pitch_rate_deg_s", max_pitch_rate_, 25.0);
    private_nh_.param("camera_mount_roll_deg", camera_mount_roll_deg_, 0.0);
    private_nh_.param("camera_mount_pitch_deg", camera_mount_pitch_deg_, 0.0);
    private_nh_.param("camera_mount_yaw_deg", camera_mount_yaw_deg_, 0.0);
    state_timeout_sec_ = std::max(0.02, state_timeout_sec_);
    error_timeout_sec_ = std::max(0.02, error_timeout_sec_);
    publish_rate_ = std::max(2.0, publish_rate_);

    command_pub_ = nh_.advertise<pod_msgs::GimbalCommand>(command_topic_, 10);
    error_sub_ = nh_.subscribe(error_topic_, 10, &PodGimbalController::errorCallback,
                               this, ros::TransportHints().tcpNoDelay());
    state_sub_ = nh_.subscribe(state_topic_, 10, &PodGimbalController::stateCallback,
                               this, ros::TransportHints().tcpNoDelay());
    tracking_service_ = private_nh_.advertiseService(
        "set_tracking_enabled", &PodGimbalController::setTrackingEnabled, this);
    timer_ = nh_.createTimer(ros::Duration(1.0 / publish_rate_),
                              &PodGimbalController::timerCallback, this);
    updater_.setHardwareID("pod_gimbal_visual_servo");
    updater_.add("visual_servo", this, &PodGimbalController::diagnostics);
  }

 private:
  void errorCallback(const tracker::NormalizedError::ConstPtr& message) {
    error_ = *message;
    error_time_ = ros::WallTime::now();
    have_error_ = true;
  }

  void stateCallback(const pod_msgs::GimbalState::ConstPtr& message) {
    state_ = *message;
    state_time_ = ros::WallTime::now();
    have_state_ = true;
  }

  bool setTrackingEnabled(std_srvs::SetBool::Request& request,
                          std_srvs::SetBool::Response& response) {
    tracking_enabled_ = request.data;
    response.success = true;
    response.message = tracking_enabled_ ? "Gimbal visual servo enabled"
                                         : "Gimbal visual servo disabled";
    return true;
  }

  void broadcastTf(const ros::Time& stamp) {
    if (!publish_dynamic_tf_ || !have_state_ || !state_.attitude_valid) return;
    const double roll = state_.attitude_deg.x * kPi / 180.0;
    const double pitch = state_.attitude_deg.y * kPi / 180.0;
    const double yaw = state_.attitude_deg.z * kPi / 180.0;
    geometry_msgs::TransformStamped gimbal_transform;
    gimbal_transform.header.stamp = stamp;
    gimbal_transform.header.frame_id = state_.parent_frame.empty()
        ? body_frame_ : state_.parent_frame;
    gimbal_transform.child_frame_id = state_.gimbal_frame.empty()
        ? gimbal_frame_ : state_.gimbal_frame;
    tf2::Quaternion gimbal_orientation;
    gimbal_orientation.setRPY(roll, pitch, yaw);
    gimbal_transform.transform.rotation = tf2::toMsg(gimbal_orientation);
    tf_broadcaster_.sendTransform(gimbal_transform);

    geometry_msgs::TransformStamped optical_transform;
    optical_transform.header.stamp = stamp;
    optical_transform.header.frame_id = gimbal_transform.child_frame_id;
    optical_transform.child_frame_id = state_.optical_frame.empty()
        ? optical_frame_ : state_.optical_frame;
    tf2::Quaternion mount_orientation;
    mount_orientation.setRPY(camera_mount_roll_deg_ * kPi / 180.0,
                             camera_mount_pitch_deg_ * kPi / 180.0,
                             camera_mount_yaw_deg_ * kPi / 180.0);
    optical_transform.transform.rotation = tf2::toMsg(mount_orientation);
    tf_broadcaster_.sendTransform(optical_transform);
  }

  void timerCallback(const ros::TimerEvent&) {
    const ros::WallTime now = ros::WallTime::now();
    const bool state_fresh = have_state_ &&
        (now - state_time_).toSec() <= state_timeout_sec_ && state_.connected &&
        state_.attitude_valid && !state_.limit_active && state_.fault_code.empty();
    const bool error_fresh = have_error_ &&
        (now - error_time_).toSec() <= error_timeout_sec_ && error_.error_valid &&
        error_.target_visible && !error_.is_estimated &&
        (!require_measurement_ || error_.control_measurement_ready);
    broadcastTf(ros::Time::now());

    pod_msgs::GimbalCommand command;
    command.header.stamp = ros::Time::now();
    command.source = "pod_gimbal_controller";
    command.mode = "hold";
    command.focus_auto = true;
    if (tracking_enabled_ && state_fresh && error_fresh) {
      const double yaw_error_deg = error_.has_angular_error
          ? error_.yaw_error_rad * 180.0 / kPi : error_.error_x * 30.0;
      const double pitch_error_deg = error_.has_angular_error
          ? error_.pitch_error_rad * 180.0 / kPi : error_.error_y * 22.5;
      command.mode = "rate";
      command.target_rate_deg_s.z = clamp(yaw_gain_ * yaw_error_deg,
                                           -max_yaw_rate_, max_yaw_rate_);
      command.target_rate_deg_s.y = clamp(pitch_gain_ * pitch_error_deg,
                                           -max_pitch_rate_, max_pitch_rate_);
      active_ = true;
      reason_ = "tracking measurement";
    } else {
      active_ = false;
      if (!tracking_enabled_) reason_ = "tracking disabled";
      else if (!state_fresh) reason_ = "gimbal state unavailable or unsafe";
      else reason_ = "tracker measurement unavailable";
    }
    command_pub_.publish(command);
    updater_.update();
  }

  void diagnostics(diagnostic_updater::DiagnosticStatusWrapper& status) {
    const int level = active_ ? diagnostic_msgs::DiagnosticStatus::OK
                              : diagnostic_msgs::DiagnosticStatus::WARN;
    status.summary(level, reason_);
    status.add("tracking_enabled", tracking_enabled_);
    status.add("active", active_);
    status.add("body_frame", body_frame_);
    status.add("gimbal_frame", gimbal_frame_);
    status.add("optical_frame", optical_frame_);
  }

  ros::NodeHandle nh_, private_nh_;
  ros::Publisher command_pub_;
  ros::Subscriber error_sub_, state_sub_;
  ros::ServiceServer tracking_service_;
  ros::Timer timer_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  diagnostic_updater::Updater updater_;
  tracker::NormalizedError error_;
  pod_msgs::GimbalState state_;
  ros::WallTime error_time_, state_time_;
  bool have_error_{false}, have_state_{false}, tracking_enabled_{false};
  bool active_{false}, publish_dynamic_tf_{true}, require_measurement_{true};
  double state_timeout_sec_{0.15}, error_timeout_sec_{0.20}, publish_rate_{50.0};
  double yaw_gain_{1.2}, pitch_gain_{1.2}, max_yaw_rate_{30.0}, max_pitch_rate_{25.0};
  double camera_mount_roll_deg_{0.0}, camera_mount_pitch_deg_{0.0}, camera_mount_yaw_deg_{0.0};
  std::string error_topic_, state_topic_, command_topic_, body_frame_, gimbal_frame_, optical_frame_;
  std::string reason_{"initializing"};
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "pod_gimbal_controller");
  PodGimbalController node;
  ros::spin();
  return 0;
}
