#include <algorithm>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include <xd_uav_detect/GimbalCommand.h>
#include <xd_uav_detect/GimbalState.h>
#include "xd_uav_detect/gimbal/gimbal_command_controller.hpp"

namespace {

class GimbalControlNode {
 public:
  GimbalControlNode() : private_nh_("~") {
    xd_uav_detect::GimbalControlConfig config;
    private_nh_.param("limits/minimum_yaw_rad", config.minimum_yaw_rad,
                      config.minimum_yaw_rad);
    private_nh_.param("limits/maximum_yaw_rad", config.maximum_yaw_rad,
                      config.maximum_yaw_rad);
    private_nh_.param("limits/minimum_pitch_rad", config.minimum_pitch_rad,
                      config.minimum_pitch_rad);
    private_nh_.param("limits/maximum_pitch_rad", config.maximum_pitch_rad,
                      config.maximum_pitch_rad);
    private_nh_.param("limits/maximum_yaw_rate_rad_s",
                      config.maximum_yaw_rate_rad_s,
                      config.maximum_yaw_rate_rad_s);
    private_nh_.param("limits/maximum_pitch_rate_rad_s",
                      config.maximum_pitch_rate_rad_s,
                      config.maximum_pitch_rate_rad_s);
    private_nh_.param("command_timeout_sec", config.command_timeout_sec,
                      config.command_timeout_sec);
    private_nh_.param("home/yaw_rad", config.home_yaw_rad,
                      config.home_yaw_rad);
    private_nh_.param("home/pitch_rad", config.home_pitch_rad,
                      config.home_pitch_rad);
    private_nh_.param("joints/yaw", yaw_joint_name_,
                      std::string("gimbal_yaw_joint"));
    private_nh_.param("joints/pitch", pitch_joint_name_,
                      std::string("gimbal_pitch_joint"));
    private_nh_.param("state_frame", state_frame_,
                      std::string("base_link"));
    private_nh_.param("trajectory_reference_link", reference_link_,
                      std::string("world"));
    private_nh_.param("control_rate_hz", control_rate_hz_, 50.0);
    control_rate_hz_ = std::max(1.0, control_rate_hz_);
    controller_.reset(new xd_uav_detect::GimbalCommandController(config));

    std::string command_topic{"gimbal/command"};
    std::string state_topic{"gimbal/state"};
    std::string joint_states_topic{"joint_states"};
    std::string trajectory_topic{"gimbal/set_joint_trajectory"};
    private_nh_.param("interfaces/command", command_topic, command_topic);
    private_nh_.param("interfaces/state", state_topic, state_topic);
    private_nh_.param("interfaces/joint_states", joint_states_topic,
                      joint_states_topic);
    private_nh_.param("interfaces/joint_trajectory", trajectory_topic,
                      trajectory_topic);

    command_subscriber_ = nh_.subscribe(
        command_topic, 10, &GimbalControlNode::commandCallback, this,
        ros::TransportHints().tcpNoDelay());
    joint_state_subscriber_ = nh_.subscribe(
        joint_states_topic, 10, &GimbalControlNode::jointStateCallback, this,
        ros::TransportHints().tcpNoDelay());
    trajectory_publisher_ =
        nh_.advertise<trajectory_msgs::JointTrajectory>(trajectory_topic, 10);
    state_publisher_ = nh_.advertise<xd_uav_detect::GimbalState>(state_topic, 10);
    timer_ = nh_.createTimer(ros::Duration(1.0 / control_rate_hz_),
                             &GimbalControlNode::timerCallback, this);

    ROS_INFO("[gimbal_control] command=%s state=%s joints=%s trajectory=%s",
             nh_.resolveName(command_topic).c_str(),
             nh_.resolveName(state_topic).c_str(),
             nh_.resolveName(joint_states_topic).c_str(),
             nh_.resolveName(trajectory_topic).c_str());
  }

 private:
  void commandCallback(const xd_uav_detect::GimbalCommand::ConstPtr& message) {
    std::string rejection;
    std::lock_guard<std::mutex> lock(mutex_);
    if (!controller_->acceptCommand(
            message->mode, message->yaw, message->pitch,
            message->yaw_rate, message->pitch_rate,
            ros::Time::now().toSec(), &rejection)) {
      ROS_WARN_THROTTLE(1.0, "[gimbal_control] rejected command: %s",
                        rejection.c_str());
    }
  }

  void jointStateCallback(const sensor_msgs::JointState::ConstPtr& message) {
    const auto yaw = std::find(message->name.begin(), message->name.end(),
                               yaw_joint_name_);
    const auto pitch = std::find(message->name.begin(), message->name.end(),
                                 pitch_joint_name_);
    if (yaw == message->name.end() || pitch == message->name.end()) return;
    const std::size_t yaw_index =
        static_cast<std::size_t>(std::distance(message->name.begin(), yaw));
    const std::size_t pitch_index =
        static_cast<std::size_t>(std::distance(message->name.begin(), pitch));
    if (yaw_index >= message->position.size() ||
        pitch_index >= message->position.size()) return;
    const double yaw_velocity = yaw_index < message->velocity.size()
        ? message->velocity[yaw_index] : 0.0;
    const double pitch_velocity = pitch_index < message->velocity.size()
        ? message->velocity[pitch_index] : 0.0;
    std::lock_guard<std::mutex> lock(mutex_);
    controller_->setJointState(
        message->position[yaw_index], message->position[pitch_index],
        yaw_velocity, pitch_velocity, ros::Time::now().toSec());
  }

  void timerCallback(const ros::TimerEvent&) {
    xd_uav_detect::GimbalControlOutput output;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      output = controller_->update(ros::Time::now().toSec());
    }
    const ros::Time stamp = ros::Time::now();
    const bool trajectory_changed = !have_published_trajectory_ ||
        std::abs(output.commanded_yaw_rad - last_published_yaw_rad_) > 1e-6 ||
        std::abs(output.commanded_pitch_rad - last_published_pitch_rad_) > 1e-6;
    if (output.output_valid && trajectory_changed) {
      trajectory_msgs::JointTrajectory trajectory;
      trajectory.header.stamp = stamp;
      trajectory.header.frame_id = reference_link_;
      trajectory.joint_names = {yaw_joint_name_, pitch_joint_name_};
      trajectory_msgs::JointTrajectoryPoint point;
      point.positions = {output.commanded_yaw_rad,
                         output.commanded_pitch_rad};
      point.time_from_start = ros::Duration(1.0 / control_rate_hz_);
      trajectory.points.push_back(point);
      trajectory_publisher_.publish(trajectory);
      have_published_trajectory_ = true;
      last_published_yaw_rad_ = output.commanded_yaw_rad;
      last_published_pitch_rad_ = output.commanded_pitch_rad;
    }

    xd_uav_detect::GimbalState state;
    state.header.stamp = stamp;
    state.header.frame_id = state_frame_;
    state.yaw = output.yaw_rad;
    state.pitch = output.pitch_rad;
    state.yaw_velocity = output.yaw_velocity_rad_s;
    state.pitch_velocity = output.pitch_velocity_rad_s;
    state.commanded_yaw = output.commanded_yaw_rad;
    state.commanded_pitch = output.commanded_pitch_rad;
    state.active_mode = output.active_mode;
    state.joint_state_valid = output.joint_state_valid;
    state.command_valid = output.command_valid;
    state.status = output.status;
    state_publisher_.publish(state);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  std::unique_ptr<xd_uav_detect::GimbalCommandController> controller_;
  std::mutex mutex_;
  std::string yaw_joint_name_;
  std::string pitch_joint_name_;
  std::string state_frame_;
  std::string reference_link_;
  double control_rate_hz_{50.0};
  bool have_published_trajectory_{false};
  double last_published_yaw_rad_{0.0};
  double last_published_pitch_rad_{0.0};
  ros::Subscriber command_subscriber_;
  ros::Subscriber joint_state_subscriber_;
  ros::Publisher trajectory_publisher_;
  ros::Publisher state_publisher_;
  ros::Timer timer_;
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "gimbal_control");
  try {
    GimbalControlNode node;
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL("[gimbal_control] startup failed: %s", error.what());
    return 1;
  }
  return 0;
}
