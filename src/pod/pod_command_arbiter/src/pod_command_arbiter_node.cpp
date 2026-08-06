#include <algorithm>
#include <string>

#include <diagnostic_updater/diagnostic_updater.h>
#include <pod_msgs/ControlLease.h>
#include <pod_msgs/GimbalCommand.h>
#include <pod_msgs/GimbalState.h>
#include <pod_msgs/ManageGimbalLease.h>
#include <ros/ros.h>
#include <std_srvs/SetBool.h>

class PodCommandArbiter {
 public:
  PodCommandArbiter() : private_nh_("~") {
    private_nh_.param("manual_command_topic", manual_topic_,
                      std::string("/pod/gimbal/manual_command"));
    private_nh_.param("mission_command_topic", mission_topic_,
                      std::string("/pod/gimbal/mission_command"));
    private_nh_.param("safety_command_topic", safety_topic_,
                      std::string("/pod/gimbal/safety_command"));
    private_nh_.param("state_topic", state_topic_, std::string("/pod/gimbal/state"));
    private_nh_.param("command_topic", command_topic_, std::string("/pod/gimbal/command"));
    private_nh_.param("lease_topic", lease_topic_, std::string("/pod/gimbal/control_lease"));
    private_nh_.param("command_timeout_sec", command_timeout_sec_, 0.20);
    private_nh_.param("state_timeout_sec", state_timeout_sec_, 0.25);
    private_nh_.param("publish_rate", publish_rate_, 50.0);
    command_timeout_sec_ = std::max(0.02, command_timeout_sec_);
    state_timeout_sec_ = std::max(0.02, state_timeout_sec_);
    publish_rate_ = std::max(2.0, publish_rate_);
    command_pub_ = nh_.advertise<pod_msgs::GimbalCommand>(command_topic_, 10);
    lease_pub_ = nh_.advertise<pod_msgs::ControlLease>(lease_topic_, 10, true);
    manual_sub_ = nh_.subscribe(manual_topic_, 10, &PodCommandArbiter::manualCallback, this);
    mission_sub_ = nh_.subscribe(mission_topic_, 10, &PodCommandArbiter::missionCallback, this);
    safety_sub_ = nh_.subscribe(safety_topic_, 10, &PodCommandArbiter::safetyCallback, this);
    state_sub_ = nh_.subscribe(state_topic_, 10, &PodCommandArbiter::stateCallback, this);
    lease_srv_ = nh_.advertiseService("/pod/gimbal/manage_control_lease",
                                      &PodCommandArbiter::manageLease, this);
    emergency_srv_ = nh_.advertiseService("/pod/gimbal/emergency_stop",
                                          &PodCommandArbiter::emergencyStop, this);
    timer_ = nh_.createTimer(ros::Duration(1.0 / publish_rate_),
                             &PodCommandArbiter::tick, this);
    updater_.setHardwareID("pod_gimbal_command_arbiter");
    updater_.add("arbitration", this, &PodCommandArbiter::diagnostics);
  }

 private:
  void manualCallback(const pod_msgs::GimbalCommandConstPtr& message) {
    manual_command_ = *message; manual_time_ = ros::WallTime::now();
  }
  void missionCallback(const pod_msgs::GimbalCommandConstPtr& message) {
    mission_command_ = *message; mission_time_ = ros::WallTime::now();
  }
  void safetyCallback(const pod_msgs::GimbalCommandConstPtr& message) {
    safety_command_ = *message; safety_time_ = ros::WallTime::now();
    if (message->emergency_stop) emergency_active_ = true;
  }
  void stateCallback(const pod_msgs::GimbalStateConstPtr& message) {
    state_ = *message; state_time_ = ros::WallTime::now(); have_state_ = true;
  }
  bool fresh(const ros::WallTime& time, const double timeout) const {
    return !time.isZero() && (ros::WallTime::now() - time).toSec() <= timeout;
  }
  bool leaseActive() const { return lease_.active && ros::WallTime::now() < lease_expiry_; }
  pod_msgs::ControlLease leaseMessage() const {
    pod_msgs::ControlLease output = lease_;
    output.header.stamp = ros::Time::now();
    output.active = leaseActive();
    output.ttl_sec = output.active ? static_cast<float>((lease_expiry_ - ros::WallTime::now()).toSec()) : 0.0F;
    if (!output.active) output.reason = "no active gimbal command lease";
    return output;
  }
  bool manageLease(pod_msgs::ManageGimbalLease::Request& request,
                   pod_msgs::ManageGimbalLease::Response& response) {
    if (request.requester.empty() || request.resource != "gimbal") {
      response.success = false;
      response.message = "requester and resource=gimbal are required";
      response.lease = leaseMessage();
      return true;
    }
    if (!request.acquire) {
      if (leaseActive() && lease_.owner != request.requester) {
        response.success = false;
        response.message = "only current lease owner may release";
      } else {
        lease_.active = false;
        lease_.reason = "released by " + request.requester;
        response.success = true;
        response.message = lease_.reason;
      }
      response.lease = leaseMessage();
      return true;
    }
    const float ttl = std::max(0.10F, std::min(30.0F, request.ttl_sec));
    if (leaseActive() && lease_.owner != request.requester && request.priority < lease_priority_) {
      response.success = false;
      response.message = "higher-priority active gimbal lease is held by " + lease_.owner;
      response.lease = leaseMessage();
      return true;
    }
    lease_.owner = request.requester;
    lease_.resource = "gimbal";
    lease_.sequence = ++lease_sequence_;
    lease_.ttl_sec = ttl;
    lease_.active = true;
    lease_.reason = "granted";
    lease_priority_ = request.priority;
    lease_expiry_ = ros::WallTime::now() + ros::WallDuration(ttl);
    response.success = true;
    response.message = "gimbal command lease granted";
    response.lease = leaseMessage();
    return true;
  }
  bool emergencyStop(std_srvs::SetBool::Request& request,
                     std_srvs::SetBool::Response& response) {
    emergency_active_ = request.data;
    response.success = true;
    response.message = emergency_active_ ? "gimbal emergency hold enabled" : "gimbal emergency hold cleared";
    return true;
  }
  pod_msgs::GimbalCommand holdCommand(const std::string& reason) const {
    pod_msgs::GimbalCommand output;
    output.header.stamp = ros::Time::now();
    output.source = "pod_command_arbiter";
    output.mode = "hold";
    output.focus_auto = true;
    if (emergency_active_) output.emergency_stop = true;
    (void)reason;
    return output;
  }
  void tick(const ros::TimerEvent&) {
    const bool state_ready = have_state_ && fresh(state_time_, state_timeout_sec_) &&
        state_.connected && state_.attitude_valid && !state_.limit_active && state_.fault_code.empty();
    pod_msgs::GimbalCommand selected = holdCommand("no valid command");
    active_source_ = "hold";
    if (emergency_active_) {
      selected = holdCommand("emergency");
      active_source_ = "emergency";
    } else if (fresh(safety_time_, command_timeout_sec_)) {
      selected = safety_command_;
      active_source_ = "safety";
    } else if (state_ready && leaseActive()) {
      if (lease_.owner == manual_command_.source && fresh(manual_time_, command_timeout_sec_)) {
        selected = manual_command_;
        active_source_ = "manual:" + lease_.owner;
      } else if (lease_.owner == mission_command_.source && fresh(mission_time_, command_timeout_sec_)) {
        selected = mission_command_;
        active_source_ = "mission:" + lease_.owner;
      } else {
        active_source_ = "lease_waiting_for_owner_command";
      }
    } else if (!state_ready) {
      active_source_ = "hold_gimbal_state_unavailable";
    }
    selected.header.stamp = ros::Time::now();
    command_pub_.publish(selected);
    lease_pub_.publish(leaseMessage());
    updater_.update();
  }
  void diagnostics(diagnostic_updater::DiagnosticStatusWrapper& status) {
    const bool active = leaseActive();
    status.summary(emergency_active_ ? diagnostic_msgs::DiagnosticStatus::ERROR
                                     : diagnostic_msgs::DiagnosticStatus::OK,
                   emergency_active_ ? "emergency gimbal hold" : active_source_);
    status.add("active_source", active_source_);
    status.add("lease_active", active);
    status.add("lease_owner", active ? lease_.owner : "");
    status.add("lease_priority", static_cast<int>(lease_priority_));
    status.add("emergency_active", emergency_active_);
  }
  ros::NodeHandle nh_, private_nh_;
  ros::Publisher command_pub_, lease_pub_;
  ros::Subscriber manual_sub_, mission_sub_, safety_sub_, state_sub_;
  ros::ServiceServer lease_srv_, emergency_srv_;
  ros::Timer timer_;
  diagnostic_updater::Updater updater_;
  pod_msgs::GimbalCommand manual_command_, mission_command_, safety_command_;
  pod_msgs::GimbalState state_;
  pod_msgs::ControlLease lease_;
  ros::WallTime manual_time_, mission_time_, safety_time_, state_time_, lease_expiry_;
  std::string manual_topic_, mission_topic_, safety_topic_, state_topic_, command_topic_, lease_topic_;
  std::string active_source_{"initializing"};
  double command_timeout_sec_{0.20}, state_timeout_sec_{0.25}, publish_rate_{50.0};
  bool have_state_{false}, emergency_active_{false};
  uint8_t lease_priority_{0};
  uint32_t lease_sequence_{0};
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "pod_command_arbiter");
  PodCommandArbiter node;
  ros::spin();
  return 0;
}
