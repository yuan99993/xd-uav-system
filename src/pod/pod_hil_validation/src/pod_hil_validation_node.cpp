#include <string>

#include <diagnostic_updater/diagnostic_updater.h>
#include <pod_msgs/GimbalState.h>
#include <pod_msgs/UavControlExecution.h>
#include <pod_msgs/UavState.h>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <tracker/DetectionArray.h>

class PodHilValidation {
 public:
  PodHilValidation() : private_nh_("~") {
    private_nh_.param("gimbal_state_topic", gimbal_topic_, std::string("/pod/gimbal/state"));
    private_nh_.param("detections_topic", detections_topic_, std::string("/pod/perception/detections"));
    private_nh_.param("uav_state_topic", uav_topic_, std::string("/pod/uav/state"));
    private_nh_.param("uav_execution_topic", execution_topic_, std::string("/pod/uav/control_execution"));
    private_nh_.param("expected_parent_frame", parent_frame_, std::string("base_link"));
    private_nh_.param("expected_gimbal_frame", gimbal_frame_, std::string("pod_gimbal_link"));
    private_nh_.param("expected_optical_frame", optical_frame_, std::string("pod_camera_optical_frame"));
    private_nh_.param("sample_timeout_sec", timeout_, 0.5);
    private_nh_.param("require_uav", require_uav_, false);
    private_nh_.param("require_detections", require_detections_, true);
    gimbal_sub_ = nh_.subscribe(gimbal_topic_, 10, &PodHilValidation::gimbalCallback, this);
    detections_sub_ = nh_.subscribe(detections_topic_, 10, &PodHilValidation::detectionsCallback, this);
    uav_sub_ = nh_.subscribe(uav_topic_, 10, &PodHilValidation::uavCallback, this);
    execution_sub_ = nh_.subscribe(execution_topic_, 10, &PodHilValidation::executionCallback, this);
    preflight_srv_ = nh_.advertiseService("/pod/hil/run_preflight",
                                          &PodHilValidation::preflight, this);
    updater_.setHardwareID("pod_hil_validation");
    updater_.add("preflight", this, &PodHilValidation::diagnostics);
  }
 private:
  bool fresh(const ros::WallTime& time) const {
    return !time.isZero() && (ros::WallTime::now() - time).toSec() <= timeout_;
  }
  void gimbalCallback(const pod_msgs::GimbalStateConstPtr& message) {
    gimbal_ = *message; gimbal_time_ = ros::WallTime::now();
  }
  void detectionsCallback(const tracker::DetectionArrayConstPtr& message) {
    detections_ = *message; detections_time_ = ros::WallTime::now();
  }
  void uavCallback(const pod_msgs::UavStateConstPtr& message) {
    uav_ = *message; uav_time_ = ros::WallTime::now();
  }
  void executionCallback(const pod_msgs::UavControlExecutionConstPtr& message) {
    execution_ = *message; execution_time_ = ros::WallTime::now();
  }
  bool check(std::string* reason) const {
    if (!fresh(gimbal_time_) || !gimbal_.connected || !gimbal_.stabilized ||
        !gimbal_.attitude_valid || gimbal_.limit_active || !gimbal_.fault_code.empty()) {
      *reason = "GimbalState is stale or reports unsafe hardware";
      return false;
    }
    if (gimbal_.parent_frame != parent_frame_ || gimbal_.gimbal_frame != gimbal_frame_ ||
        gimbal_.optical_frame != optical_frame_) {
      *reason = "GimbalState frame names do not match HIL configuration";
      return false;
    }
    if (require_detections_ && (!fresh(detections_time_) || detections_.header.stamp.isZero() ||
        detections_.image_source.empty() || detections_.sensor_id.empty())) {
      *reason = "detection stream lacks fresh capture time or EO/IR provenance";
      return false;
    }
    if (require_uav_ && (!fresh(uav_time_) || !uav_.connected || !uav_.localization_valid ||
        uav_.control_backend.empty() || uav_.control_backend == "none")) {
      *reason = "UAV state is not ready for HIL control validation";
      return false;
    }
    *reason = "HIL preflight passed";
    return true;
  }
  bool preflight(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& response) {
    response.success = check(&response.message);
    return true;
  }
  void diagnostics(diagnostic_updater::DiagnosticStatusWrapper& status) {
    std::string reason;
    const bool ready = check(&reason);
    status.summary(ready ? diagnostic_msgs::DiagnosticStatus::OK
                         : diagnostic_msgs::DiagnosticStatus::WARN, reason);
    status.add("gimbal_fresh", fresh(gimbal_time_));
    status.add("detections_fresh", fresh(detections_time_));
    status.add("uav_fresh", fresh(uav_time_));
    status.add("execution_fresh", fresh(execution_time_));
    status.add("last_execution_reason", execution_.reason);
  }
  ros::NodeHandle nh_, private_nh_;
  ros::Subscriber gimbal_sub_, detections_sub_, uav_sub_, execution_sub_;
  ros::ServiceServer preflight_srv_;
  diagnostic_updater::Updater updater_;
  pod_msgs::GimbalState gimbal_;
  tracker::DetectionArray detections_;
  pod_msgs::UavState uav_;
  pod_msgs::UavControlExecution execution_;
  ros::WallTime gimbal_time_, detections_time_, uav_time_, execution_time_;
  std::string gimbal_topic_, detections_topic_, uav_topic_, execution_topic_;
  std::string parent_frame_, gimbal_frame_, optical_frame_;
  double timeout_{0.5};
  bool require_uav_{false}, require_detections_{true};
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "pod_hil_validation");
  PodHilValidation node;
  ros::spin();
  return 0;
}
