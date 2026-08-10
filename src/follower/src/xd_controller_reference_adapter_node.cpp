#include <algorithm>
#include <cmath>
#include <mutex>
#include <string>

#include <follower/FollowerCommand.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>

namespace {

double wrapAngle(const double angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}

}  // namespace

class XdControllerReferenceAdapter {
 public:
  XdControllerReferenceAdapter() : private_nh_("~") {
    private_nh_.param("publish_rate", publish_rate_, 60.0);
    private_nh_.param("state_timeout", state_timeout_, 0.5);
    private_nh_.param("command_timeout", command_timeout_, 0.25);
    publish_rate_ = std::max(2.0, publish_rate_);
    state_timeout_ = std::max(0.05, state_timeout_);
    command_timeout_ = std::max(0.05, command_timeout_);

    std::string command_topic{"/follower_node/follower_command"};
    std::string state_topic{"/uav1/state_estimator/main/odom"};
    std::string reference_topic{"/uav1/control/reference/odom"};
    private_nh_.param("command_topic", command_topic, command_topic);
    private_nh_.param("state_topic", state_topic, state_topic);
    private_nh_.param("reference_topic", reference_topic, reference_topic);

    reference_publisher_ =
        nh_.advertise<nav_msgs::Odometry>(reference_topic, 10);
    const ros::TransportHints low_latency_transport =
        ros::TransportHints().tcpNoDelay();
    command_subscriber_ = nh_.subscribe(
        command_topic, 1,
        &XdControllerReferenceAdapter::commandCallback, this,
        low_latency_transport);
    state_subscriber_ = nh_.subscribe(
        state_topic, 1,
        &XdControllerReferenceAdapter::stateCallback, this,
        low_latency_transport);
    timer_ = nh_.createTimer(
        ros::Duration(1.0 / publish_rate_),
        &XdControllerReferenceAdapter::timerCallback, this);
    last_publish_wall_time_ = ros::WallTime::now();

    ROS_INFO_STREAM("[FollowerXdAdapter] C++ adapter: " << command_topic
                    << " -> " << reference_topic);
  }

 private:
  void commandCallback(const follower::FollowerCommand::ConstPtr& message) {
    std::lock_guard<std::mutex> lock(mutex_);
    command_ = *message;
    have_command_ = true;
    command_wall_time_ = ros::WallTime::now();
  }

  void stateCallback(const nav_msgs::Odometry::ConstPtr& message) {
    std::lock_guard<std::mutex> lock(mutex_);
    state_ = *message;
    have_state_ = true;
    state_wall_time_ = ros::WallTime::now();
  }

  void timerCallback(const ros::TimerEvent&) {
    const ros::WallTime now = ros::WallTime::now();
    const double dt = std::min(
        0.2, std::max(0.0, (now - last_publish_wall_time_).toSec()));
    last_publish_wall_time_ = now;

    nav_msgs::Odometry state;
    follower::FollowerCommand command;
    bool have_state = false;
    bool command_valid = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      have_state = have_state_ &&
                   (now - state_wall_time_).toSec() <= state_timeout_;
      if (have_state) {
        state = state_;
      }
      command_valid =
          have_command_ &&
          (now - command_wall_time_).toSec() <= command_timeout_ &&
          command_.command_valid && command_.profile_supported &&
          command_.control_authorized && command_.platform_ready &&
          command_.control_mode == "velocity_body";
      if (have_command_ && command_.command_valid &&
          command_.control_mode != "velocity_body") {
        ROS_ERROR_THROTTLE(2.0,
            "[FollowerXdAdapter] Rejecting non-velocity profile '%s' (%s)",
            command_.follower_profile.c_str(), command_.control_mode.c_str());
      }
      if (command_valid) {
        command = command_;
      }
    }

    if (!have_state || state.header.frame_id.empty()) {
      ROS_WARN_THROTTLE(
          2.0, "[FollowerXdAdapter] Waiting for fresh estimator odometry");
      return;
    }

    const double yaw = tf::getYaw(state.pose.pose.orientation);
    double forward = 0.0;
    double right = 0.0;
    double down = 0.0;
    double yaw_rate = 0.0;

    // Do not claim the controller reference topic before the follower has
    // produced its first valid command. This leaves the controller's internal
    // takeoff reference untouched. After the adapter has taken ownership,
    // however, it must keep a fresh stationary reference on target loss or
    // follower stop; otherwise the controller rejects the stale point
    // reference after reference_timeout and the control manager enters
    // failsafe.
    if (!command_valid && !have_reference_) {
      command_active_ = false;
      return;
    }
    if (command_valid) {
      forward = command.velocity_forward;
      right = command.velocity_right;
      down = command.velocity_down;
      yaw_rate = command.yaw_rate_deg_s * M_PI / 180.0;
    }

    // Follower uses forward/right/down. The XD reference is ENU odometry.
    const double vx =
        std::cos(yaw) * forward + std::sin(yaw) * right;
    const double vy =
        std::sin(yaw) * forward - std::cos(yaw) * right;
    const double vz = -down;

    if (!have_reference_ || command_active_ != command_valid) {
      reference_x_ = state.pose.pose.position.x;
      reference_y_ = state.pose.pose.position.y;
      reference_z_ = state.pose.pose.position.z;
      reference_yaw_ = yaw;
      have_reference_ = true;
    }
    command_active_ = command_valid;
    reference_x_ += vx * dt;
    reference_y_ += vy * dt;
    reference_z_ += vz * dt;
    reference_yaw_ = wrapAngle(reference_yaw_ + yaw_rate * dt);

    nav_msgs::Odometry reference;
    reference.header.stamp = ros::Time::now();
    reference.header.frame_id = state.header.frame_id;
    reference.child_frame_id = state.header.frame_id;
    reference.pose.pose.position.x = reference_x_;
    reference.pose.pose.position.y = reference_y_;
    reference.pose.pose.position.z = reference_z_;
    reference.pose.pose.orientation =
        tf::createQuaternionMsgFromYaw(reference_yaw_);
    reference.twist.twist.linear.x = vx;
    reference.twist.twist.linear.y = vy;
    reference.twist.twist.linear.z = vz;
    reference.twist.twist.angular.z = yaw_rate;
    reference_publisher_.publish(reference);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Publisher reference_publisher_;
  ros::Subscriber command_subscriber_;
  ros::Subscriber state_subscriber_;
  ros::Timer timer_;

  std::mutex mutex_;
  nav_msgs::Odometry state_;
  follower::FollowerCommand command_;
  bool have_state_{false};
  bool have_command_{false};
  ros::WallTime state_wall_time_;
  ros::WallTime command_wall_time_;
  ros::WallTime last_publish_wall_time_;

  double publish_rate_{30.0};
  double state_timeout_{0.5};
  double command_timeout_{0.25};
  bool have_reference_{false};
  bool command_active_{false};
  double reference_x_{0.0};
  double reference_y_{0.0};
  double reference_z_{0.0};
  double reference_yaw_{0.0};
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "follower_xd_controller_adapter");
  XdControllerReferenceAdapter adapter;
  ros::spin();
  return 0;
}
