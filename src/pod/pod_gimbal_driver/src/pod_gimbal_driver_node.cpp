#include <algorithm>
#include <cstdint>
#include <string>

#include <diagnostic_updater/diagnostic_updater.h>
#include <pod_msgs/GimbalCommand.h>
#include <pod_msgs/GimbalState.h>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>

class PodGimbalDriver {
 public:
  PodGimbalDriver() : private_nh_("~") {
    private_nh_.param("vendor_state_topic", vendor_state_topic_,
                      std::string("/pod/vendor/gimbal/state"));
    private_nh_.param("vendor_command_topic", vendor_command_topic_,
                      std::string("/pod/vendor/gimbal/command"));
    private_nh_.param("state_topic", state_topic_, std::string("/pod/gimbal/state"));
    private_nh_.param("command_topic", command_topic_, std::string("/pod/gimbal/command"));
    private_nh_.param("parent_frame", parent_frame_, std::string("base_link"));
    private_nh_.param("gimbal_frame", gimbal_frame_, std::string("pod_gimbal_link"));
    private_nh_.param("optical_frame", optical_frame_, std::string("pod_camera_optical_frame"));
    private_nh_.param("state_timeout_sec", state_timeout_sec_, 0.25);
    state_timeout_sec_ = std::max(0.05, state_timeout_sec_);
    state_pub_ = nh_.advertise<pod_msgs::GimbalState>(state_topic_, 10, true);
    vendor_command_pub_ = nh_.advertise<pod_msgs::GimbalCommand>(vendor_command_topic_, 10);
    vendor_state_sub_ = nh_.subscribe(vendor_state_topic_, 10,
                                      &PodGimbalDriver::stateCallback, this,
                                      ros::TransportHints().tcpNoDelay());
    command_sub_ = nh_.subscribe(command_topic_, 10,
                                 &PodGimbalDriver::commandCallback, this,
                                 ros::TransportHints().tcpNoDelay());
    recenter_srv_ = nh_.advertiseService("/pod/gimbal/recenter",
                                         &PodGimbalDriver::recenter, this);
    updater_.setHardwareID("pod_gimbal_driver_bridge");
    updater_.add("gimbal", this, &PodGimbalDriver::diagnostics);
  }

 private:
  void stateCallback(const pod_msgs::GimbalStateConstPtr& message) {
    pod_msgs::GimbalState output = *message;
    if (output.header.stamp.isZero()) output.header.stamp = ros::Time::now();
    if (output.parent_frame.empty()) output.parent_frame = parent_frame_;
    if (output.gimbal_frame.empty()) output.gimbal_frame = gimbal_frame_;
    if (output.optical_frame.empty()) output.optical_frame = optical_frame_;
    last_state_ = output;
    last_state_time_ = ros::WallTime::now();
    have_state_ = true;
    state_pub_.publish(output);
  }
  void commandCallback(const pod_msgs::GimbalCommandConstPtr& message) {
    // The only product input is the arbiter's final command.  A vendor bridge
    // can remap vendor_command_topic_ to CAN, serial, MAVLink, or SDK code.
    vendor_command_pub_.publish(message);
    ++command_count_;
  }
  bool recenter(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& response) {
    pod_msgs::GimbalCommand command;
    command.header.stamp = ros::Time::now();
    command.source = "pod_gimbal_driver_recenter";
    command.mode = "position";
    command.recenter = true;
    command.focus_auto = true;
    vendor_command_pub_.publish(command);
    response.success = true;
    response.message = "recenter command forwarded to vendor bridge";
    return true;
  }
  void diagnostics(diagnostic_updater::DiagnosticStatusWrapper& status) {
    const bool fresh = have_state_ &&
        (ros::WallTime::now() - last_state_time_).toSec() <= state_timeout_sec_;
    const bool healthy = fresh && last_state_.connected && last_state_.attitude_valid &&
        last_state_.fault_code.empty();
    status.summary(healthy ? diagnostic_msgs::DiagnosticStatus::OK
                           : diagnostic_msgs::DiagnosticStatus::WARN,
                   healthy ? "vendor gimbal state forwarding" : "gimbal state unavailable or unsafe");
    status.add("vendor_state_topic", vendor_state_topic_);
    status.add("vendor_command_topic", vendor_command_topic_);
    status.add("state_fresh", fresh);
    status.add("commands_forwarded", static_cast<long long>(command_count_));
  }
  ros::NodeHandle nh_, private_nh_;
  ros::Publisher state_pub_, vendor_command_pub_;
  ros::Subscriber vendor_state_sub_, command_sub_;
  ros::ServiceServer recenter_srv_;
  diagnostic_updater::Updater updater_;
  std::string vendor_state_topic_, vendor_command_topic_, state_topic_, command_topic_;
  std::string parent_frame_, gimbal_frame_, optical_frame_;
  double state_timeout_sec_{0.25};
  pod_msgs::GimbalState last_state_;
  ros::WallTime last_state_time_;
  bool have_state_{false};
  std::uint64_t command_count_{0};
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "pod_gimbal_driver");
  PodGimbalDriver node;
  ros::spin();
  return 0;
}
