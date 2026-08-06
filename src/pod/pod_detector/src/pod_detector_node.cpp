#include <cmath>
#include <cstdint>
#include <string>

#include <diagnostic_updater/diagnostic_updater.h>
#include <ros/ros.h>
#include <tracker/DetectionArray.h>

class PodDetector {
 public:
  PodDetector() : private_nh_("~") {
    private_nh_.param("vendor_detections_topic", vendor_topic_,
                      std::string("/pod/vendor/detections"));
    private_nh_.param("detections_topic", product_topic_,
                      std::string("/pod/perception/detections"));
    private_nh_.param("image_source", image_source_, std::string("eo"));
    private_nh_.param("sensor_id", sensor_id_, std::string("eo_primary"));
    private_nh_.param("detector_name", detector_name_, std::string("external_detector"));
    private_nh_.param("model_version", model_version_, std::string("unknown"));
    private_nh_.param("allow_vendor_provenance", allow_vendor_provenance_, true);
    product_pub_ = nh_.advertise<tracker::DetectionArray>(product_topic_, 10);
    vendor_sub_ = nh_.subscribe(vendor_topic_, 10, &PodDetector::callback, this,
                                ros::TransportHints().tcpNoDelay());
    updater_.setHardwareID("pod_detector_bridge");
    updater_.add("detector", this, &PodDetector::diagnostics);
  }

 private:
  void callback(const tracker::DetectionArrayConstPtr& message) {
    if (message->header.stamp.isZero()) {
      ++rejected_count_;
      ROS_WARN_THROTTLE(2.0, "[PodDetector] Rejected detection frame without capture timestamp");
      return;
    }
    tracker::DetectionArray output = *message;
    if (!allow_vendor_provenance_ || output.image_source.empty()) output.image_source = image_source_;
    if (!allow_vendor_provenance_ || output.sensor_id.empty()) output.sensor_id = sensor_id_;
    if (!allow_vendor_provenance_ || output.detector_name.empty()) output.detector_name = detector_name_;
    if (!allow_vendor_provenance_ || output.model_version.empty()) output.model_version = model_version_;
    for (const auto& candidate : output.candidates) {
      if (!std::isfinite(candidate.confidence)) {
        ++rejected_count_;
        ROS_WARN_THROTTLE(2.0, "[PodDetector] Rejected detection frame with non-finite confidence");
        return;
      }
    }
    last_time_ = ros::WallTime::now();
    last_count_ = output.candidates.size();
    ++published_count_;
    product_pub_.publish(output);
  }
  void diagnostics(diagnostic_updater::DiagnosticStatusWrapper& status) {
    const bool fresh = !last_time_.isZero() &&
        (ros::WallTime::now() - last_time_).toSec() <= 1.0;
    status.summary(fresh ? diagnostic_msgs::DiagnosticStatus::OK
                         : diagnostic_msgs::DiagnosticStatus::WARN,
                   fresh ? "detector frames normalized" : "waiting for vendor detections");
    status.add("product_topic", product_topic_);
    status.add("image_source", image_source_);
    status.add("sensor_id", sensor_id_);
    status.add("detector_name", detector_name_);
    status.add("model_version", model_version_);
    status.add("last_detection_count", static_cast<int>(last_count_));
    status.add("published_frames", static_cast<long long>(published_count_));
    status.add("rejected_frames", static_cast<long long>(rejected_count_));
  }
  ros::NodeHandle nh_, private_nh_;
  ros::Publisher product_pub_;
  ros::Subscriber vendor_sub_;
  diagnostic_updater::Updater updater_;
  std::string vendor_topic_, product_topic_, image_source_, sensor_id_, detector_name_, model_version_;
  bool allow_vendor_provenance_{true};
  ros::WallTime last_time_;
  std::size_t last_count_{0};
  std::uint64_t published_count_{0}, rejected_count_{0};
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "pod_detector");
  PodDetector node;
  ros::spin();
  return 0;
}
