#include <algorithm>
#include <boost/bind.hpp>
#include <memory>
#include <mutex>
#include <string>

#include <actionlib/server/simple_action_server.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <follower/FollowerStatus.h>
#include <follower/ManageControlLease.h>
#include <pod_msgs/SelectTarget.h>
#include <pod_msgs/SelectedTarget.h>
#include <pod_msgs/GimbalState.h>
#include <pod_msgs/ManageGimbalLease.h>
#include <pod_msgs/MissionState.h>
#include <pod_msgs/TaskEvent.h>
#include <pod_msgs/TrackTargetAction.h>
#include <pod_msgs/UavControlExecution.h>
#include <pod_msgs/UavState.h>
#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>

class PodMissionManager {
 public:
  PodMissionManager()
      : private_nh_("~") {
    private_nh_.param("selected_target_topic", selected_target_topic_,
                      std::string("/pod/target/selected"));
    private_nh_.param("gimbal_state_topic", gimbal_state_topic_,
                      std::string("/pod/gimbal/state"));
    private_nh_.param("uav_state_topic", uav_state_topic_, std::string("/pod/uav/state"));
    private_nh_.param("uav_execution_topic", execution_topic_,
                      std::string("/pod/uav/control_execution"));
    private_nh_.param("uav_prepare_follow_service", uav_prepare_service_,
                      std::string("/pod_uav_adapter/prepare_follow"));
    private_nh_.param("follower_status_topic", follower_status_topic_,
                      std::string("/follower_node/follower_status"));
    private_nh_.param("mission_state_topic", mission_state_topic_,
                      std::string("/pod/mission/state"));
    private_nh_.param("event_topic", event_topic_, std::string("/pod/events"));
    private_nh_.param("track_action_name", track_action_name_,
                      std::string("/pod/mission/track_target"));
    private_nh_.param("reset_service_name", reset_service_name_,
                      std::string("/pod/mission/reset"));
    private_nh_.param("select_target_service", select_target_service_,
                      std::string("/pod/mission/select_target"));
    private_nh_.param("gimbal_tracking_service", gimbal_tracking_service_,
                      std::string("/pod_gimbal_controller/set_tracking_enabled"));
    private_nh_.param("gimbal_lease_service", gimbal_lease_service_,
                      std::string("/pod/gimbal/manage_control_lease"));
    private_nh_.param("gimbal_lease_requester", gimbal_lease_requester_,
                      std::string("pod_gimbal_controller"));
    private_nh_.param("gimbal_lease_priority", gimbal_lease_priority_, 100);
    private_nh_.param("gimbal_lease_duration_sec", gimbal_lease_duration_sec_, 1.0F);
    private_nh_.param("require_gimbal_command_lease", require_gimbal_command_lease_, false);
    private_nh_.param("follower_lease_service", follower_lease_service_,
                      std::string("/follower_node/manage_control_lease"));
    private_nh_.param("follower_start_service", follower_start_service_,
                      std::string("/follower_node/start"));
    private_nh_.param("follower_stop_service", follower_stop_service_,
                      std::string("/follower_node/stop"));
    private_nh_.param("follower_output_backend", follower_backend_,
                      std::string("command_only"));
    private_nh_.param("lease_requester", lease_requester_,
                      std::string("pod_mission_manager"));
    private_nh_.param("lease_duration_sec", lease_duration_sec_, 1.0F);
    private_nh_.param("lease_renew_interval_sec", lease_renew_interval_sec_, 0.35);
    private_nh_.param("target_loss_abort_sec", target_loss_abort_sec_, 2.0);
    private_nh_.param("state_timeout_sec", state_timeout_sec_, 0.5);
    private_nh_.param("uav_prepare_timeout_sec", uav_prepare_timeout_sec_, 3.0);
    private_nh_.param("require_gimbal_ready", require_gimbal_ready_, true);
    private_nh_.param("require_uav_localization", require_uav_localization_, true);
    private_nh_.param("require_execution_ack", require_execution_ack_, true);
    lease_duration_sec_ = std::max(0.2F, lease_duration_sec_);
    lease_renew_interval_sec_ = std::max(0.05, lease_renew_interval_sec_);

    mission_pub_ = nh_.advertise<pod_msgs::MissionState>(mission_state_topic_, 10, true);
    event_pub_ = nh_.advertise<pod_msgs::TaskEvent>(event_topic_, 20);
    selected_sub_ = nh_.subscribe(selected_target_topic_, 10,
                                  &PodMissionManager::selectedCallback, this);
    gimbal_sub_ = nh_.subscribe(gimbal_state_topic_, 10,
                                &PodMissionManager::gimbalCallback, this);
    uav_sub_ = nh_.subscribe(uav_state_topic_, 10, &PodMissionManager::uavCallback, this);
    execution_sub_ = nh_.subscribe(execution_topic_, 10,
                                   &PodMissionManager::executionCallback, this);
    follower_sub_ = nh_.subscribe(follower_status_topic_, 10,
                                  &PodMissionManager::followerCallback, this);
    select_client_ = nh_.serviceClient<pod_msgs::SelectTarget>(select_target_service_);
    gimbal_tracking_client_ = nh_.serviceClient<std_srvs::SetBool>(gimbal_tracking_service_);
    gimbal_lease_client_ = nh_.serviceClient<pod_msgs::ManageGimbalLease>(
        gimbal_lease_service_);
    follower_lease_client_ = nh_.serviceClient<follower::ManageControlLease>(
        follower_lease_service_);
    follower_start_client_ = nh_.serviceClient<std_srvs::SetBool>(follower_start_service_);
    follower_stop_client_ = nh_.serviceClient<std_srvs::SetBool>(follower_stop_service_);
    uav_prepare_client_ = nh_.serviceClient<std_srvs::Trigger>(uav_prepare_service_);
    reset_service_ = nh_.advertiseService(reset_service_name_, &PodMissionManager::reset, this);
    state_timer_ = nh_.createTimer(ros::Duration(0.1), &PodMissionManager::stateTimer, this);
    updater_.setHardwareID("pod_mission_manager");
    updater_.add("mission", this, &PodMissionManager::diagnostics);
    setState("READY", "no active task");
    track_server_.reset(new actionlib::SimpleActionServer<pod_msgs::TrackTargetAction>(
        nh_, track_action_name_, boost::bind(&PodMissionManager::executeTrack, this, _1), false));
    track_server_->start();
  }

 private:
  void selectedCallback(const pod_msgs::SelectedTarget::ConstPtr& message) {
    std::lock_guard<std::mutex> lock(mutex_);
    selected_ = *message;
    selected_time_ = ros::WallTime::now();
    have_selected_ = true;
  }
  void gimbalCallback(const pod_msgs::GimbalState::ConstPtr& message) {
    std::lock_guard<std::mutex> lock(mutex_);
    gimbal_ = *message;
    gimbal_time_ = ros::WallTime::now();
    have_gimbal_ = true;
  }
  void uavCallback(const pod_msgs::UavState::ConstPtr& message) {
    std::lock_guard<std::mutex> lock(mutex_);
    uav_ = *message;
    uav_time_ = ros::WallTime::now();
    have_uav_ = true;
  }
  void executionCallback(const pod_msgs::UavControlExecution::ConstPtr& message) {
    std::lock_guard<std::mutex> lock(mutex_);
    execution_ = *message;
    execution_time_ = ros::WallTime::now();
    have_execution_ = true;
  }
  void followerCallback(const follower::FollowerStatus::ConstPtr& message) {
    std::lock_guard<std::mutex> lock(mutex_);
    follower_ = *message;
    follower_time_ = ros::WallTime::now();
    have_follower_ = true;
  }

  bool fresh(const ros::WallTime& timestamp) const {
    return !timestamp.isZero() &&
        (ros::WallTime::now() - timestamp).toSec() <= state_timeout_sec_;
  }

  void event(const std::string& level, const std::string& event_type,
             const std::string& message, const int target_id) {
    pod_msgs::TaskEvent output;
    output.header.stamp = ros::Time::now();
    output.level = level;
    output.source = "pod_mission_manager";
    output.event_type = event_type;
    output.message = message;
    output.target_id = target_id;
    event_pub_.publish(output);
  }

  void setState(const std::string& state, const std::string& reason) {
    std::lock_guard<std::mutex> lock(mutex_);
    state_name_ = state;
    state_reason_ = reason;
  }

  void publishState() {
    pod_msgs::MissionState output;
    output.header.stamp = ros::Time::now();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      output.state = state_name_;
      output.active_mode = active_mode_;
      output.selected_target_id = have_selected_ ? selected_.target_id : -1;
      output.gimbal_control_enabled = gimbal_tracking_enabled_;
      output.aircraft_follow_enabled = aircraft_follow_enabled_;
      output.control_authorized = have_follower_ && fresh(follower_time_) &&
          follower_.control_authorized;
      output.aircraft_command_executing = have_execution_ && fresh(execution_time_) &&
          execution_.accepted && execution_.executing;
      output.aircraft_follow_state = aircraft_follow_state_;
      output.reason = state_reason_;
    }
    mission_pub_.publish(output);
  }

  void stateTimer(const ros::TimerEvent&) {
    publishState();
    updater_.update();
  }

  bool setGimbalTracking(const bool enable) {
    if (enable && require_gimbal_command_lease_ && !manageGimbalLease(true)) {
      return false;
    }
    std_srvs::SetBool service;
    service.request.data = enable;
    if (!gimbal_tracking_client_.waitForExistence(ros::Duration(1.0)) ||
        !gimbal_tracking_client_.call(service) || !service.response.success) {
      if (enable && require_gimbal_command_lease_) manageGimbalLease(false);
      return false;
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      gimbal_tracking_enabled_ = enable;
    }
    if (!enable && require_gimbal_command_lease_) manageGimbalLease(false);
    return true;
  }

  bool manageGimbalLease(const bool acquire) {
    pod_msgs::ManageGimbalLease service;
    service.request.acquire = acquire;
    service.request.requester = gimbal_lease_requester_;
    service.request.resource = "gimbal";
    service.request.priority = static_cast<uint8_t>(std::max(0, std::min(255, gimbal_lease_priority_)));
    service.request.ttl_sec = acquire ? std::max(0.1F, gimbal_lease_duration_sec_) : 0.0F;
    return gimbal_lease_client_.waitForExistence(ros::Duration(1.0)) &&
        gimbal_lease_client_.call(service) && service.response.success;
  }

  bool renewFollowerLease() {
    follower::ManageControlLease service;
    service.request.acquire = true;
    service.request.requester = lease_requester_;
    service.request.output_backend = follower_backend_;
    service.request.lease_duration_sec = lease_duration_sec_;
    return follower_lease_client_.waitForExistence(ros::Duration(1.0)) &&
        follower_lease_client_.call(service) && service.response.success;
  }

  void stopFollowerAndRelease() {
    std_srvs::SetBool stop;
    stop.request.data = true;
    if (follower_stop_client_.exists()) follower_stop_client_.call(stop);
    follower::ManageControlLease release;
    release.request.acquire = false;
    release.request.requester = lease_requester_;
    release.request.output_backend = follower_backend_;
    release.request.lease_duration_sec = 0.0F;
    if (follower_lease_client_.exists()) follower_lease_client_.call(release);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      aircraft_follow_enabled_ = false;
      aircraft_follow_state_ = "DISABLED";
    }
  }

  bool startFollower(std::string& reason) {
    if (!renewFollowerLease()) {
      reason = "follower control lease was denied";
      return false;
    }
    std_srvs::SetBool start;
    start.request.data = true;
    if (!follower_start_client_.waitForExistence(ros::Duration(1.0)) ||
        !follower_start_client_.call(start) || !start.response.success) {
      stopFollowerAndRelease();
      reason = "follower start was rejected";
      return false;
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      aircraft_follow_enabled_ = true;
      aircraft_follow_state_ = "ARMED_WAITING_FOR_COMMAND";
    }
    return true;
  }

  bool platformReadyForAircraft(std::string& reason) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!have_uav_ || !fresh(uav_time_) || !uav_.connected) {
      reason = "UAV telemetry is unavailable";
      return false;
    }
    if (!uav_.armed || !uav_.offboard_active) {
      reason = "UAV is not armed in the selected control mode";
      return false;
    }
    if (require_uav_localization_ && !uav_.localization_valid) {
      reason = "UAV localization is not valid";
      return false;
    }
    if (uav_.control_backend == "none" || uav_.control_backend.empty()) {
      reason = "no UAV control backend is selected";
      return false;
    }
    return true;
  }

  bool prepareUavForFollow(std::string& reason) {
    std_srvs::Trigger prepare;
    if (!uav_prepare_client_.waitForExistence(ros::Duration(1.0)) ||
        !uav_prepare_client_.call(prepare) || !prepare.response.success) {
      reason = "UAV follow handover preparation failed";
      return false;
    }
    const ros::WallTime deadline = ros::WallTime::now() +
        ros::WallDuration(std::max(0.1, uav_prepare_timeout_sec_));
    while (ros::ok() && ros::WallTime::now() < deadline) {
      if (platformReadyForAircraft(reason)) return true;
      ros::WallDuration(0.05).sleep();
    }
    reason = "UAV control backend did not become ready after handover";
    return false;
  }

  bool safeTrackingMeasurement(const int target_id, std::string& reason) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!have_selected_ || !fresh(selected_time_) || !selected_.selected ||
        selected_.target_id != target_id || !selected_.tracking_active) {
      reason = "selected target is not actively tracked";
      return false;
    }
    if (!selected_.control_measurement_ready) {
      reason = "selected target is prediction-only or unconfirmed";
      return false;
    }
    if (require_gimbal_ready_ && (!have_gimbal_ || !fresh(gimbal_time_) ||
        !gimbal_.connected || !gimbal_.stabilized || !gimbal_.attitude_valid ||
        gimbal_.limit_active || !gimbal_.fault_code.empty())) {
      reason = "gimbal feedback is not ready for closed-loop tracking";
      return false;
    }
    return true;
  }

  bool reset(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& response) {
    stopFollowerAndRelease();
    setGimbalTracking(false);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      active_mode_.clear();
    }
    setState("READY", "mission reset");
    response.success = true;
    response.message = "mission reset";
    return true;
  }

  void executeTrack(const pod_msgs::TrackTargetGoalConstPtr& goal) {
    pod_msgs::TrackTargetResult result;
    pod_msgs::TrackTargetFeedback feedback;
    const int target_id = goal->target_id;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      active_mode_ = "TRACK_TARGET";
    }
    setState("ACQUIRE", "selecting requested target");
    event("INFO", "TRACK_REQUESTED", "track action accepted", target_id);

    pod_msgs::SelectTarget select;
    select.request.target_id = target_id;
    select.request.start_tracking = true;
    if (!select_client_.waitForExistence(ros::Duration(1.0)) ||
        !select_client_.call(select) || !select.response.success) {
      result.success = false;
      result.message = "target selection failed";
      setState("READY", result.message);
      event("ERROR", "TRACK_REJECTED", result.message, target_id);
      track_server_->setAborted(result, result.message);
      return;
    }
    if (goal->enable_gimbal_tracking && !setGimbalTracking(true)) {
      result.success = false;
      result.message = "gimbal visual-servo service is unavailable";
      setState("FAULT", result.message);
      track_server_->setAborted(result, result.message);
      return;
    }
    std::string reason;
    if (goal->enable_aircraft_follow) {
      if (!prepareUavForFollow(reason) || !startFollower(reason)) {
        setGimbalTracking(false);
        result.success = false;
        result.message = reason;
        setState("READY", reason);
        event("ERROR", "AIRCRAFT_FOLLOW_REJECTED", reason, target_id);
        track_server_->setAborted(result, reason);
        return;
      }
    }

    setState("TRACKING", "tracking active");
    ros::WallTime lost_since;
    ros::WallTime next_renew = ros::WallTime::now();
    ros::WallTime next_gimbal_renew = ros::WallTime::now();
    ros::Rate rate(10.0);
    while (ros::ok()) {
      if (track_server_->isPreemptRequested()) {
        result.success = true;
        result.message = "tracking preempted";
        break;
      }
      bool measurement_ok = safeTrackingMeasurement(target_id, reason);
      feedback.target_visible = measurement_ok;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        feedback.tracking_confidence = have_selected_ ? selected_.confidence : 0.0F;
      }
      track_server_->publishFeedback(feedback);
      if (!measurement_ok) {
        if (lost_since.isZero()) lost_since = ros::WallTime::now();
        setState("LOST", reason);
        if ((ros::WallTime::now() - lost_since).toSec() > target_loss_abort_sec_) {
          result.success = false;
          result.message = "target lost beyond configured timeout: " + reason;
          break;
        }
      } else {
        lost_since = ros::WallTime();
        setState("TRACKING", "tracking measurement ready");
      }
      if (goal->enable_aircraft_follow) {
        if (ros::WallTime::now() >= next_renew) {
          if (!renewFollowerLease()) {
            result.success = false;
            result.message = "follower lease renewal failed";
            break;
          }
          next_renew = ros::WallTime::now() + ros::WallDuration(lease_renew_interval_sec_);
        }
        bool execution_ok = false;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          execution_ok = have_execution_ && fresh(execution_time_) &&
              execution_.accepted && execution_.executing;
        }
        if (require_execution_ack_ && measurement_ok && !execution_ok) {
          std::lock_guard<std::mutex> lock(mutex_);
          aircraft_follow_state_ = "WAITING_FOR_EXECUTION_ACK";
        } else {
          std::lock_guard<std::mutex> lock(mutex_);
          aircraft_follow_state_ = execution_ok ? "EXECUTING" : "DEGRADED";
        }
      }
      if (goal->enable_gimbal_tracking && require_gimbal_command_lease_ &&
          ros::WallTime::now() >= next_gimbal_renew) {
        if (!manageGimbalLease(true)) {
          result.success = false;
          result.message = "gimbal command lease renewal failed";
          break;
        }
        next_gimbal_renew = ros::WallTime::now() + ros::WallDuration(
            std::max(0.05, std::min(0.35, 0.5 * static_cast<double>(gimbal_lease_duration_sec_))));
      }
      rate.sleep();
    }
    const bool preempted = track_server_->isPreemptRequested();
    stopFollowerAndRelease();
    setGimbalTracking(false);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      active_mode_.clear();
    }
    setState("READY", result.message.empty() ? "tracking stopped" : result.message);
    event(result.success ? "INFO" : "WARN", "TRACK_FINISHED", result.message, target_id);
    if (preempted) track_server_->setPreempted(result, result.message);
    else if (result.success) track_server_->setSucceeded(result, result.message);
    else track_server_->setAborted(result, result.message);
  }

  void diagnostics(diagnostic_updater::DiagnosticStatusWrapper& status) {
    std::lock_guard<std::mutex> lock(mutex_);
    const int level = state_name_ == "FAULT" ? diagnostic_msgs::DiagnosticStatus::ERROR
        : state_name_ == "LOST" ? diagnostic_msgs::DiagnosticStatus::WARN
        : diagnostic_msgs::DiagnosticStatus::OK;
    status.summary(level, state_reason_);
    status.add("state", state_name_);
    status.add("mode", active_mode_);
    status.add("aircraft_follow", aircraft_follow_enabled_);
    status.add("aircraft_follow_state", aircraft_follow_state_);
    status.add("gimbal_tracking", gimbal_tracking_enabled_);
  }

  ros::NodeHandle nh_, private_nh_;
  std::unique_ptr<actionlib::SimpleActionServer<pod_msgs::TrackTargetAction>> track_server_;
  ros::Publisher mission_pub_, event_pub_;
  ros::Subscriber selected_sub_, gimbal_sub_, uav_sub_, execution_sub_, follower_sub_;
  ros::ServiceClient select_client_, gimbal_tracking_client_, gimbal_lease_client_, follower_lease_client_;
  ros::ServiceClient follower_start_client_, follower_stop_client_, uav_prepare_client_;
  ros::ServiceServer reset_service_;
  ros::Timer state_timer_;
  diagnostic_updater::Updater updater_;
  mutable std::mutex mutex_;
  pod_msgs::SelectedTarget selected_;
  pod_msgs::GimbalState gimbal_;
  pod_msgs::UavState uav_;
  pod_msgs::UavControlExecution execution_;
  follower::FollowerStatus follower_;
  ros::WallTime selected_time_, gimbal_time_, uav_time_, execution_time_, follower_time_;
  bool have_selected_{false}, have_gimbal_{false}, have_uav_{false};
  bool have_execution_{false}, have_follower_{false}, gimbal_tracking_enabled_{false};
  bool aircraft_follow_enabled_{false};
  bool require_gimbal_ready_{true}, require_uav_localization_{true}, require_execution_ack_{true};
  bool require_gimbal_command_lease_{false};
  float lease_duration_sec_{1.0F};
  float gimbal_lease_duration_sec_{1.0F};
  int gimbal_lease_priority_{100};
  double lease_renew_interval_sec_{0.35}, target_loss_abort_sec_{2.0}, state_timeout_sec_{0.5};
  double uav_prepare_timeout_sec_{3.0};
  std::string selected_target_topic_, gimbal_state_topic_, uav_state_topic_, execution_topic_, uav_prepare_service_;
  std::string track_action_name_, reset_service_name_;
  std::string follower_status_topic_, mission_state_topic_, event_topic_, select_target_service_;
  std::string gimbal_tracking_service_, gimbal_lease_service_, gimbal_lease_requester_;
  std::string follower_lease_service_, follower_start_service_, follower_stop_service_;
  std::string follower_backend_, lease_requester_, state_name_{"INITIALIZING"};
  std::string state_reason_{"initializing"}, active_mode_, aircraft_follow_state_{"DISABLED"};
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "pod_mission_manager");
  PodMissionManager node;
  // The action callback contains the task lifecycle loop.  A multi-threaded
  // spinner keeps target, gimbal, UAV and execution-feedback subscriptions
  // live while that loop is running.
  ros::AsyncSpinner spinner(4);
  spinner.start();
  ros::waitForShutdown();
  return 0;
}
