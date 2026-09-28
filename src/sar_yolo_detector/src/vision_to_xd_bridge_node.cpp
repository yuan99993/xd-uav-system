#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>

#include <ros/ros.h>
#include <sar_yolo_detector/TrackedDetection2DArray.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/Header.h>
#include <vision_msgs/Detection2DArray.h>
#include <xd_uav_track/DetectionArray.h>
#include <xd_uav_track/DetectionCandidate.h>

namespace {

class VisionToXdBridge {
public:
  VisionToXdBridge() : private_node_("~") {
    private_node_.param("input_detections_topic", input_topic_,
                        std::string("sar_yolo_detector/detections"));
    private_node_.param("input_type", input_type_, std::string("vision"));
    private_node_.param("output_detections_topic", output_topic_,
                        std::string("detect/input/detections_2d"));
    private_node_.param("camera_info_topic", camera_info_topic_,
                        std::string("camera/camera_info"));
    int configured_width = 0;
    int configured_height = 0;
    private_node_.param("image_width", configured_width, 0);
    private_node_.param("image_height", configured_height, 0);
    image_width_ = static_cast<unsigned int>(std::max(0, configured_width));
    image_height_ = static_cast<unsigned int>(std::max(0, configured_height));
    private_node_.param("image_source", image_source_, std::string("eo"));
    private_node_.param("sensor_id", sensor_id_,
                        std::string("camera_primary"));
    private_node_.param("detector_name", detector_name_,
                        std::string("sar_yolo_detector"));
    private_node_.param("model_version", model_version_,
                        std::string("unknown"));

    publisher_ =
        node_.advertise<xd_uav_track::DetectionArray>(output_topic_, 5);
    if (input_type_ == "vision") {
      subscriber_ = node_.subscribe(
          input_topic_, 5, &VisionToXdBridge::detectionsCallback, this,
          ros::TransportHints().tcpNoDelay());
    } else if (input_type_ == "tracked") {
      subscriber_ = node_.subscribe(
          input_topic_, 5, &VisionToXdBridge::trackedDetectionsCallback, this,
          ros::TransportHints().tcpNoDelay());
    } else {
      throw std::invalid_argument("input_type must be 'vision' or 'tracked'");
    }
    if (!camera_info_topic_.empty()) {
      camera_info_subscriber_ = node_.subscribe(
          camera_info_topic_, 1, &VisionToXdBridge::cameraInfoCallback, this,
          ros::TransportHints().tcpNoDelay());
    }

    ROS_INFO_STREAM("[sar_vision_to_xd_bridge] " << input_topic_ << " ("
                    << input_type_ << ") -> " << output_topic_
                    << "; CameraInfo="
                    << (camera_info_topic_.empty() ? "disabled"
                                                   : camera_info_topic_));
  }

private:
  void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr &message) {
    if (message->width == 0 || message->height == 0)
      return;
    image_width_ = message->width;
    image_height_ = message->height;
  }

  static bool finiteBox(const vision_msgs::BoundingBox2D &box) {
    return std::isfinite(box.center.x) && std::isfinite(box.center.y) &&
           std::isfinite(box.size_x) && std::isfinite(box.size_y) &&
           box.size_x > 0.0 && box.size_y > 0.0;
  }

  static int32_t boundedInt(const double value) {
    const double lower = static_cast<double>(std::numeric_limits<int32_t>::min());
    const double upper = static_cast<double>(std::numeric_limits<int32_t>::max());
    return static_cast<int32_t>(std::max(lower, std::min(upper, value)));
  }

  xd_uav_track::DetectionArray outputFor(const std_msgs::Header &header) const {
    xd_uav_track::DetectionArray output;
    output.header = header;
    output.image_width = image_width_;
    output.image_height = image_height_;
    output.image_source = image_source_;
    output.sensor_id = sensor_id_;
    output.detector_name = detector_name_;
    output.model_version = model_version_;
    return output;
  }

  bool appendDetection(const vision_msgs::Detection2D &detection,
                       const int64_t source_track_id,
                       const bool source_track_id_is_stable,
                       xd_uav_track::DetectionArray *output) const {
    if (output == nullptr || !finiteBox(detection.bbox) ||
        detection.results.empty()) {
      return false;
    }

    const auto best = std::max_element(
        detection.results.begin(), detection.results.end(),
        [](const vision_msgs::ObjectHypothesisWithPose &left,
           const vision_msgs::ObjectHypothesisWithPose &right) {
          return left.score < right.score;
        });
    if (best == detection.results.end() || !std::isfinite(best->score) ||
        best->id < std::numeric_limits<int32_t>::min() ||
        best->id > std::numeric_limits<int32_t>::max()) {
      return false;
    }

    double x_min = detection.bbox.center.x - 0.5 * detection.bbox.size_x;
    double y_min = detection.bbox.center.y - 0.5 * detection.bbox.size_y;
    double x_max = detection.bbox.center.x + 0.5 * detection.bbox.size_x;
    double y_max = detection.bbox.center.y + 0.5 * detection.bbox.size_y;
    if (output->image_width > 0) {
      x_min = std::max(0.0, std::min<double>(output->image_width, x_min));
      x_max = std::max(0.0, std::min<double>(output->image_width, x_max));
    }
    if (output->image_height > 0) {
      y_min = std::max(0.0, std::min<double>(output->image_height, y_min));
      y_max = std::max(0.0, std::min<double>(output->image_height, y_max));
    }
    if (x_max <= x_min || y_max <= y_min)
      return false;

    xd_uav_track::DetectionCandidate candidate;
    if (source_track_id_is_stable && source_track_id >= 0 &&
        source_track_id <= std::numeric_limits<int32_t>::max()) {
      candidate.track_id = static_cast<int32_t>(source_track_id);
      candidate.track_id_is_stable = true;
    } else {
      candidate.track_id = -1;
      candidate.track_id_is_stable = false;
    }
    candidate.class_id = static_cast<int32_t>(best->id);
    candidate.confidence = static_cast<float>(best->score);
    candidate.bbox[0] = boundedInt(std::floor(x_min));
    candidate.bbox[1] = boundedInt(std::floor(y_min));
    candidate.bbox[2] = boundedInt(std::ceil(x_max));
    candidate.bbox[3] = boundedInt(std::ceil(y_max));
    candidate.has_bbox = true;
    candidate.has_normalized_bbox = false;
    candidate.has_relative_position_body = false;
    candidate.has_relative_velocity_body = false;
    candidate.range_valid = false;
    output->candidates.push_back(candidate);
    return true;
  }

  void warnRejected(const std::size_t rejected) const {
    if (rejected == 0)
      return;
    ROS_WARN_STREAM_THROTTLE(
        2.0, "[sar_vision_to_xd_bridge] rejected " << rejected
                                                     << " malformed detections");
  }

  void detectionsCallback(
      const vision_msgs::Detection2DArray::ConstPtr &message) {
    xd_uav_track::DetectionArray output = outputFor(message->header);
    output.candidates.reserve(message->detections.size());
    std::size_t rejected = 0;
    for (const auto &detection : message->detections) {
      if (!appendDetection(detection, -1, false, &output))
        ++rejected;
    }
    warnRejected(rejected);
    publisher_.publish(output);
  }

  void trackedDetectionsCallback(
      const sar_yolo_detector::TrackedDetection2DArray::ConstPtr &message) {
    xd_uav_track::DetectionArray output = outputFor(message->header);
    if (!message->provenance.sensor_id.empty())
      output.sensor_id = message->provenance.sensor_id;
    if (!message->provenance.model_version.empty())
      output.model_version = message->provenance.model_version;
    output.candidates.reserve(message->detections.size());
    std::size_t rejected = 0;
    for (const auto &tracked : message->detections) {
      if (!appendDetection(tracked.detection, tracked.track_id,
                           tracked.track_id_is_stable, &output)) {
        ++rejected;
      }
    }
    warnRejected(rejected);
    publisher_.publish(output);
  }

  ros::NodeHandle node_;
  ros::NodeHandle private_node_;
  ros::Publisher publisher_;
  ros::Subscriber subscriber_;
  ros::Subscriber camera_info_subscriber_;
  std::string input_topic_;
  std::string input_type_;
  std::string output_topic_;
  std::string camera_info_topic_;
  std::string image_source_;
  std::string sensor_id_;
  std::string detector_name_;
  std::string model_version_;
  unsigned int image_width_{0};
  unsigned int image_height_{0};
};

} // namespace

int main(int argc, char **argv) {
  ros::init(argc, argv, "sar_vision_to_xd_bridge");
  VisionToXdBridge bridge;
  ros::spin();
  return 0;
}
