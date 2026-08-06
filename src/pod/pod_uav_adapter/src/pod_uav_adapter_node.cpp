#include <algorithm>
#include <cmath>
#include <mutex>
#include <string>

#include <diagnostic_updater/diagnostic_updater.h>
#include <follower/FollowerCommand.h>
#include <follower/FollowerStatus.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mrs_msgs/VelocityReferenceStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/BatteryState.h>
#include <std_srvs/Trigger.h>
#include <tf/transform_datatypes.h>

#include <pod_msgs/UavControlExecution.h>
#include <pod_msgs/UavState.h>

namespace {
constexpr double kPi = 3.14159265358979323846;
double clamp(const double value, const double low, const double high) {
  return std::max(low, std::min(high, value));
}
}

class PodUavAdapter {
 public:
  PodUavAdapter() : private_nh_("~") {
    private_nh_.param("control_backend", backend_, std::string("none"));
    nh_.param("/pod/uav_control_backend", backend_, backend_);
    private_nh_.param("follower_command_topic", command_topic_,
                      std::string("/follower_node/follower_command"));
    private_nh_.param("follower_status_topic", follower_status_topic_,
                      std::string("/follower_node/follower_status"));
    private_nh_.param("uav_state_topic", uav_state_topic_, std::string("/pod/uav/state"));
    private_nh_.param("execution_topic", execution_topic_,
                      std::string("/pod/uav/control_execution"));
    private_nh_.param("mavros_state_topic", mavros_state_topic_,
                      std::string("/mavros/state"));
    private_nh_.param("odom_topic", odom_topic_,
                      std::string("/uav1/state_estimator/main/odom"));
    private_nh_.param("battery_topic", battery_topic_,
                      std::string("/mavros/battery"));
    private_nh_.param("mavros_setpoint_topic", mavros_setpoint_topic_,
                      std::string("/mavros/setpoint_raw/local"));
    private_nh_.param("mavros_set_mode_service", mavros_set_mode_service_,
                      std::string("/mavros/set_mode"));
    private_nh_.param("required_mavros_mode", required_mavros_mode_,
                      std::string("OFFBOARD"));
    private_nh_.param("mrs_velocity_reference_topic", mrs_topic_,
                      std::string("/uav1/control_manager/velocity_reference_in"));
    private_nh_.param("mrs_velocity_frame", mrs_frame_, std::string("uav1/fcu"));
    private_nh_.param("xd_reference_topic", xd_reference_topic_,
                      std::string("/uav1/control/reference/odom"));
    private_nh_.param("command_timeout_sec", command_timeout_sec_, 0.25);
    private_nh_.param("state_timeout_sec", state_timeout_sec_, 0.5);
    private_nh_.param("publish_rate", publish_rate_, 50.0);
    private_nh_.param("prestream_count", prestream_count_, 30);
    private_nh_.param("prestream_rate_hz", prestream_rate_hz_, 30.0);
    command_timeout_sec_ = std::max(0.05, command_timeout_sec_);
    state_timeout_sec_ = std::max(0.05, state_timeout_sec_);
    publish_rate_ = std::max(2.0, publish_rate_);
    prestream_count_ = std::max(1, prestream_count_);
    prestream_rate_hz_ = std::max(2.0, prestream_rate_hz_);

    state_pub_ = nh_.advertise<pod_msgs::UavState>(uav_state_topic_, 10, true);
    execution_pub_ = nh_.advertise<pod_msgs::UavControlExecution>(execution_topic_, 20);
    if (backend_ == "mavros_direct") {
      mavros_setpoint_pub_ = nh_.advertise<mavros_msgs::PositionTarget>(
          mavros_setpoint_topic_, 20);
    } else if (backend_ == "mrs") {
      mrs_pub_ = nh_.advertise<mrs_msgs::VelocityReferenceStamped>(mrs_topic_, 20);
    } else if (backend_ == "xd") {
      xd_reference_pub_ = nh_.advertise<nav_msgs::Odometry>(xd_reference_topic_, 20);
    }
    command_sub_ = nh_.subscribe(command_topic_, 10, &PodUavAdapter::commandCallback,
                                 this, ros::TransportHints().tcpNoDelay());
    follower_status_sub_ = nh_.subscribe(follower_status_topic_, 10,
                                          &PodUavAdapter::followerStatusCallback, this);
    mavros_state_sub_ = nh_.subscribe(mavros_state_topic_, 10,
                                      &PodUavAdapter::mavrosStateCallback, this);
    odom_sub_ = nh_.subscribe(odom_topic_, 10, &PodUavAdapter::odomCallback, this);
    battery_sub_ = nh_.subscribe(battery_topic_, 10, &PodUavAdapter::batteryCallback, this);
    mavros_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>(mavros_set_mode_service_);
    prepare_service_ = private_nh_.advertiseService("prepare_follow",
                                                     &PodUavAdapter::prepareFollow, this);
    timer_ = nh_.createTimer(ros::Duration(1.0 / publish_rate_),
                              &PodUavAdapter::timerCallback, this);
    updater_.setHardwareID("pod_uav_adapter");
    updater_.add("uav_control", this, &PodUavAdapter::diagnostics);
    reason_ = backend_ == "none" ? "aircraft control disabled" : "waiting for telemetry";
  }

 private:
  void commandCallback(const follower::FollowerCommand::ConstPtr& message) {
    std::lock_guard<std::mutex> lock(mutex_);
    command_ = *message;
    command_time_ = ros::WallTime::now();
    have_command_ = true;
  }
  void followerStatusCallback(const follower::FollowerStatus::ConstPtr& message) {
    std::lock_guard<std::mutex> lock(mutex_);
    follower_status_ = *message;
    follower_status_time_ = ros::WallTime::now();
    have_follower_status_ = true;
  }
  void mavrosStateCallback(const mavros_msgs::State::ConstPtr& message) {
    std::lock_guard<std::mutex> lock(mutex_);
    mavros_state_ = *message;
    mavros_state_time_ = ros::WallTime::now();
    have_mavros_state_ = true;
  }
  void odomCallback(const nav_msgs::Odometry::ConstPtr& message) {
    std::lock_guard<std::mutex> lock(mutex_);
    odom_ = *message;
    odom_time_ = ros::WallTime::now();
    have_odom_ = true;
  }
  void batteryCallback(const sensor_msgs::BatteryState::ConstPtr& message) {
    std::lock_guard<std::mutex> lock(mutex_);
    battery_ = *message;
    battery_time_ = ros::WallTime::now();
    have_battery_ = true;
  }

  bool fresh(const ros::WallTime& time) const {
    return !time.isZero() && (ros::WallTime::now() - time).toSec() <= state_timeout_sec_;
  }
  bool mavrosReady() const {
    return have_mavros_state_ && fresh(mavros_state_time_) && mavros_state_.connected &&
        mavros_state_.armed && mavros_state_.mode == required_mavros_mode_;
  }
  bool commandValid() const {
    return have_command_ && (ros::WallTime::now() - command_time_).toSec() <=
        command_timeout_sec_ && command_.command_valid && command_.profile_supported &&
        command_.control_authorized && command_.platform_ready &&
        command_.control_mode == "velocity_body";
  }

  void publishMavrosSetpoint(const follower::FollowerCommand& command) {
    mavros_msgs::PositionTarget target;
    target.header.stamp = ros::Time::now();
    target.header.frame_id = "base_link_frd";
    target.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
    target.type_mask = mavros_msgs::PositionTarget::IGNORE_PX |
        mavros_msgs::PositionTarget::IGNORE_PY |
        mavros_msgs::PositionTarget::IGNORE_PZ |
        mavros_msgs::PositionTarget::IGNORE_AFX |
        mavros_msgs::PositionTarget::IGNORE_AFY |
        mavros_msgs::PositionTarget::IGNORE_AFZ |
        mavros_msgs::PositionTarget::IGNORE_YAW;
    target.velocity.x = command.velocity_forward;
    target.velocity.y = command.velocity_right;
    target.velocity.z = command.velocity_down;
    target.yaw_rate = command.yaw_rate_deg_s * kPi / 180.0;
    mavros_setpoint_pub_.publish(target);
  }

  void publishMrsSetpoint(const follower::FollowerCommand& command) {
    mrs_msgs::VelocityReferenceStamped reference;
    reference.header.stamp = ros::Time::now();
    reference.header.frame_id = mrs_frame_;
    reference.reference.velocity.x = command.velocity_forward;
    reference.reference.velocity.y = -command.velocity_right;
    reference.reference.velocity.z = -command.velocity_down;
    reference.reference.heading_rate = command.yaw_rate_deg_s * kPi / 180.0;
    reference.reference.use_altitude = false;
    reference.reference.use_heading = false;
    reference.reference.use_heading_rate = true;
    mrs_pub_.publish(reference);
  }

  bool publishXdSetpoint(const follower::FollowerCommand& command, const double dt) {
    if (!have_odom_ || !fresh(odom_time_) || odom_.header.frame_id.empty()) return false;
    const double yaw = tf::getYaw(odom_.pose.pose.orientation);
    const double vx = std::cos(yaw) * command.velocity_forward +
        std::sin(yaw) * command.velocity_right;
    const double vy = std::sin(yaw) * command.velocity_forward -
        std::cos(yaw) * command.velocity_right;
    const double vz = -command.velocity_down;
    if (!have_xd_reference_) {
      xd_x_ = odom_.pose.pose.position.x;
      xd_y_ = odom_.pose.pose.position.y;
      xd_z_ = odom_.pose.pose.position.z;
      xd_yaw_ = yaw;
      have_xd_reference_ = true;
    }
    xd_x_ += vx * dt;
    xd_y_ += vy * dt;
    xd_z_ += vz * dt;
    xd_yaw_ = std::atan2(std::sin(xd_yaw_ + command.yaw_rate_deg_s * kPi / 180.0 * dt),
                          std::cos(xd_yaw_ + command.yaw_rate_deg_s * kPi / 180.0 * dt));
    nav_msgs::Odometry reference;
    reference.header.stamp = ros::Time::now();
    reference.header.frame_id = odom_.header.frame_id;
    reference.child_frame_id = odom_.header.frame_id;
    reference.pose.pose.position.x = xd_x_;
    reference.pose.pose.position.y = xd_y_;
    reference.pose.pose.position.z = xd_z_;
    reference.pose.pose.orientation = tf::createQuaternionMsgFromYaw(xd_yaw_);
    reference.twist.twist.linear.x = vx;
    reference.twist.twist.linear.y = vy;
    reference.twist.twist.linear.z = vz;
    reference.twist.twist.angular.z = command.yaw_rate_deg_s * kPi / 180.0;
    xd_reference_pub_.publish(reference);
    return true;
  }

  bool prepareFollow(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& response) {
    if (backend_ != "mavros_direct") {
      response.success = backend_ == "xd" || backend_ == "mrs";
      response.message = response.success ? "backend does not require MAVROS mode handover"
                                         : "no UAV control backend is selected";
      return true;
    }
    mavros_msgs::State state;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!have_mavros_state_ || !fresh(mavros_state_time_) || !mavros_state_.connected ||
          !mavros_state_.armed) {
        response.success = false;
        response.message = "MAVROS must be connected and armed before follow handover";
        return true;
      }
      state = mavros_state_;
    }
    follower::FollowerCommand zero;
    for (int index = 0; index < prestream_count_ && ros::ok(); ++index) {
      publishMavrosSetpoint(zero);
      ros::Duration(1.0 / prestream_rate_hz_).sleep();
    }
    mavros_msgs::SetMode set_mode;
    set_mode.request.custom_mode = required_mavros_mode_;
    if (!mavros_mode_client_.waitForExistence(ros::Duration(1.0)) ||
        !mavros_mode_client_.call(set_mode) || !set_mode.response.mode_sent) {
      response.success = false;
      response.message = "MAVROS rejected requested control mode";
      return true;
    }
    response.success = true;
    response.message = "MAVROS prestream completed and mode request sent";
    return true;
  }

  void timerCallback(const ros::TimerEvent& event) {
    const double dt = std::min(0.2, std::max(0.0, (event.current_real - event.last_real).toSec()));
    follower::FollowerCommand command;
    bool valid = false;
    bool backend_ready = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      valid = commandValid();
      if (valid) command = command_;
      backend_ready = backend_ == "mavros_direct" ? mavrosReady()
          : backend_ == "xd" ? have_odom_ && fresh(odom_time_)
          : backend_ == "mrs" ? true : false;
    }
    pod_msgs::UavControlExecution execution;
    execution.header.stamp = ros::Time::now();
    execution.backend = backend_;
    execution.safety_gate_passed = valid && backend_ready;
    execution.accepted = false;
    execution.executing = false;
    execution.reason = "aircraft control disabled";
    if (valid) {
      execution.source_command_sequence = command.command_sequence;
      execution.source_capture_timestamp = command.source_capture_timestamp;
      execution.command_age_ms = static_cast<float>(
          (ros::WallTime::now() - command_time_).toSec() * 1000.0);
    }
    follower::FollowerCommand applied = valid ? command : follower::FollowerCommand();
    if (backend_ready && (valid || has_taken_control_)) {
      bool published = false;
      if (backend_ == "mavros_direct") {
        publishMavrosSetpoint(applied);
        published = true;
      } else if (backend_ == "mrs") {
        publishMrsSetpoint(applied);
        published = true;
      } else if (backend_ == "xd") {
        published = publishXdSetpoint(applied, dt);
      }
      has_taken_control_ = has_taken_control_ || published;
      execution.accepted = valid && published;
      execution.executing = valid && published;
      execution.failsafe_active = published && !valid;
      execution.applied_velocity_body.x = applied.velocity_forward;
      execution.applied_velocity_body.y = applied.velocity_right;
      execution.applied_velocity_body.z = applied.velocity_down;
      execution.applied_yaw_rate_deg_s = applied.yaw_rate_deg_s;
      execution.reason = execution.executing ? "setpoint published" :
          (execution.failsafe_active ? "safe zero setpoint published" : "backend rejected command");
    } else if (backend_ != "none") {
      execution.reason = valid ? "backend telemetry or mode is not ready"
                               : "waiting for a valid follower command";
    }
    reason_ = execution.reason;
    execution_pub_.publish(execution);
    publishUavState();
    updater_.update();
  }

  void publishUavState() {
    pod_msgs::UavState output;
    output.header.stamp = ros::Time::now();
    std::lock_guard<std::mutex> lock(mutex_);
    const bool backend_telemetry_ready = (backend_ == "xd" || backend_ == "mrs") &&
        have_odom_ && fresh(odom_time_);
    output.connected = (have_mavros_state_ && fresh(mavros_state_time_) &&
                        mavros_state_.connected) || backend_telemetry_ready;
    output.flight_mode = have_mavros_state_ ? mavros_state_.mode : "";
    output.armed = (have_mavros_state_ && mavros_state_.armed) || backend_telemetry_ready;
    output.offboard_active = backend_ == "mavros_direct" ? mavrosReady()
        : backend_ == "xd" || backend_ == "mrs";
    output.localization_valid = have_odom_ && fresh(odom_time_) && !odom_.header.frame_id.empty();
    if (output.localization_valid) {
      output.header = odom_.header;
      output.pose = odom_.pose.pose;
      output.twist = odom_.twist.twist;
    }
    output.battery_percent = have_battery_ && fresh(battery_time_)
        ? battery_.percentage : -1.0F;
    output.control_backend = backend_;
    state_pub_.publish(output);
  }

  void diagnostics(diagnostic_updater::DiagnosticStatusWrapper& status) {
    const int level = backend_ == "none" ? diagnostic_msgs::DiagnosticStatus::WARN
        : reason_ == "setpoint published" ? diagnostic_msgs::DiagnosticStatus::OK
        : diagnostic_msgs::DiagnosticStatus::WARN;
    status.summary(level, reason_);
    status.add("backend", backend_);
    status.add("mavros_ready", mavrosReady());
    status.add("has_taken_control", has_taken_control_);
  }

  ros::NodeHandle nh_, private_nh_;
  ros::Publisher state_pub_, execution_pub_, mavros_setpoint_pub_, mrs_pub_, xd_reference_pub_;
  ros::Subscriber command_sub_, follower_status_sub_, mavros_state_sub_, odom_sub_, battery_sub_;
  ros::ServiceClient mavros_mode_client_;
  ros::ServiceServer prepare_service_;
  ros::Timer timer_;
  diagnostic_updater::Updater updater_;
  mutable std::mutex mutex_;
  follower::FollowerCommand command_;
  follower::FollowerStatus follower_status_;
  mavros_msgs::State mavros_state_;
  nav_msgs::Odometry odom_;
  sensor_msgs::BatteryState battery_;
  ros::WallTime command_time_, follower_status_time_, mavros_state_time_, odom_time_, battery_time_;
  bool have_command_{false}, have_follower_status_{false}, have_mavros_state_{false};
  bool have_odom_{false}, have_battery_{false}, has_taken_control_{false}, have_xd_reference_{false};
  double xd_x_{0.0}, xd_y_{0.0}, xd_z_{0.0}, xd_yaw_{0.0};
  double command_timeout_sec_{0.25}, state_timeout_sec_{0.5}, publish_rate_{50.0};
  int prestream_count_{30};
  double prestream_rate_hz_{30.0};
  std::string backend_, command_topic_, follower_status_topic_, uav_state_topic_, execution_topic_;
  std::string mavros_state_topic_, odom_topic_, battery_topic_, mavros_setpoint_topic_, mavros_set_mode_service_;
  std::string required_mavros_mode_, mrs_topic_, mrs_frame_, xd_reference_topic_, reason_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "pod_uav_adapter");
  PodUavAdapter node;
  ros::spin();
  return 0;
}
