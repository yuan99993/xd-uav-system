#include <cmath>
#include <cstdint>
#include <string>

#include <diagnostic_updater/diagnostic_updater.h>
#include <pod_msgs/GimbalState.h>
#include <ros/ros.h>
#include <tracker/DetectionArray.h>

class PodSim {
 public:
  PodSim() : private_nh_("~") {
    private_nh_.param("gimbal_state_topic", gimbal_topic_,
                      std::string("/pod/vendor/gimbal/state"));
    private_nh_.param("detections_topic", detections_topic_,
                      std::string("/pod/vendor/detections"));
    private_nh_.param("publish_rate", rate_, 20.0);
    private_nh_.param("image_source", image_source_, std::string("eo"));
    private_nh_.param("sensor_id", sensor_id_, std::string("sim_eo"));
    private_nh_.param("detector_name", detector_name_, std::string("pod_sim"));
    private_nh_.param("model_version", model_version_, std::string("sim-1"));
    rate_ = std::max(1.0, rate_);
    gimbal_pub_ = nh_.advertise<pod_msgs::GimbalState>(gimbal_topic_, 10);
    detections_pub_ = nh_.advertise<tracker::DetectionArray>(detections_topic_, 10);
    timer_ = nh_.createTimer(ros::Duration(1.0 / rate_), &PodSim::tick, this);
    updater_.setHardwareID("pod_sim");
    updater_.add("simulation", this, &PodSim::diagnostics);
  }
 private:
  void tick(const ros::TimerEvent&) {
    const ros::Time now = ros::Time::now();
    const double phase = now.toSec();
    pod_msgs::GimbalState state;
    state.header.stamp = now;
    state.attitude_deg.z = 12.0 * std::sin(0.2 * phase);
    state.attitude_deg.y = -5.0 * std::sin(0.15 * phase);
    state.connected = true;
    state.stabilized = true;
    state.attitude_valid = true;
    state.mode = "stabilize";
    state.parent_frame = "base_link";
    state.gimbal_frame = "pod_gimbal_link";
    state.optical_frame = "pod_camera_optical_frame";
    gimbal_pub_.publish(state);

    tracker::DetectionArray detections;
    detections.header.stamp = now;
    detections.header.frame_id = state.optical_frame;
    detections.image_source = image_source_;
    detections.sensor_id = sensor_id_;
    detections.detector_name = detector_name_;
    detections.model_version = model_version_;
    tracker::DetectionCandidate target;
    target.track_id = 1;
    target.track_id_is_stable = true;
    target.class_id = 0;
    target.confidence = 0.92F;
    const int center_x = static_cast<int>(320.0 + 180.0 * std::sin(0.35 * phase));
    const int center_y = static_cast<int>(240.0 + 90.0 * std::sin(0.21 * phase));
    target.bbox = {center_x - 30, center_y - 20, center_x + 30, center_y + 20};
    target.has_bbox = true;
    target.normalized_bbox = {static_cast<float>(center_x / 640.0),
                              static_cast<float>(center_y / 480.0), 60.0F / 640.0F,
                              40.0F / 480.0F};
    target.has_normalized_bbox = true;
    detections.candidates.push_back(target);
    detections_pub_.publish(detections);
    ++frames_;
  }
  void diagnostics(diagnostic_updater::DiagnosticStatusWrapper& status) {
    status.summary(diagnostic_msgs::DiagnosticStatus::OK, "deterministic gimbal and target simulation active");
    status.add("frames", static_cast<long long>(frames_));
    status.add("image_source", image_source_);
  }
  ros::NodeHandle nh_, private_nh_;
  ros::Publisher gimbal_pub_, detections_pub_;
  ros::Timer timer_;
  diagnostic_updater::Updater updater_;
  std::string gimbal_topic_, detections_topic_, image_source_, sensor_id_, detector_name_, model_version_;
  double rate_{20.0};
  std::uint64_t frames_{0};
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "pod_sim");
  PodSim node;
  ros::spin();
  return 0;
}
