#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <iostream>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <XmlRpcValue.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <opencv2/core.hpp>
#include <ros/ros.h>
#include <vision_msgs/Detection2D.h>
#include <vision_msgs/Detection2DArray.h>
#include <vision_msgs/ObjectHypothesisWithPose.h>
#include <vision_msgs/VisionInfo.h>

#include <sar_yolo_detector/FusionDetectionProvenance.h>
#include <sar_yolo_detector/FusionProvenanceArray.h>
#include <sar_yolo_detector/linear_assignment.hpp>

namespace {

double clamp(const double value, const double low, const double high) {
  return std::max(low, std::min(high, value));
}

double iou(const cv::Rect2d &left, const cv::Rect2d &right) {
  const cv::Rect2d intersection = left & right;
  const double union_area = left.area() + right.area() - intersection.area();
  return union_area <= 0.0 ? 0.0 : intersection.area() / union_area;
}

bool contains(const std::vector<int> &values, const int value) {
  return std::find(values.begin(), values.end(), value) != values.end();
}

struct Detection {
  cv::Rect2d box;
  float confidence{0.0F};
  std::uint32_t source_index{0U};
};

struct PendingFrame {
  vision_msgs::Detection2DArray::ConstPtr message;
  ros::WallTime received;
};

double numericXmlValue(const XmlRpc::XmlRpcValue &value,
                       const std::string &parameter_name) {
  if (value.getType() == XmlRpc::XmlRpcValue::TypeInt)
    return static_cast<int>(value);
  if (value.getType() == XmlRpc::XmlRpcValue::TypeDouble)
    return static_cast<double>(value);
  throw std::invalid_argument(parameter_name + " must contain numeric values");
}

std::vector<int> integerArrayParameter(ros::NodeHandle &node,
                                       const std::string &parameter_name) {
  XmlRpc::XmlRpcValue value;
  if (!node.getParam(parameter_name, value) ||
      value.getType() != XmlRpc::XmlRpcValue::TypeArray)
    return {};
  std::vector<int> result;
  result.reserve(value.size());
  for (int index = 0; index < value.size(); ++index)
    result.push_back(static_cast<int>(numericXmlValue(value[index], parameter_name)));
  return result;
}

std::vector<double> numericArrayParameter(ros::NodeHandle &node,
                                          const std::string &parameter_name) {
  XmlRpc::XmlRpcValue value;
  if (!node.getParam(parameter_name, value) ||
      value.getType() != XmlRpc::XmlRpcValue::TypeArray)
    return {};
  std::vector<double> result;
  result.reserve(value.size());
  for (int index = 0; index < value.size(); ++index)
    result.push_back(numericXmlValue(value[index], parameter_name));
  return result;
}

class LateFusionNode {
public:
  LateFusionNode() : private_node_("~") {
    private_node_.param("primary_detections_topic", primary_topic_,
                        std::string("sar_yolo_detector/eo/detections"));
    private_node_.param("secondary_detections_topic", secondary_topic_,
                        std::string("sar_yolo_detector/ir/detections"));
    private_node_.param("fused_detections_topic", output_topic_,
                        std::string("sar_yolo_detector/fused/detections"));
    private_node_.param("fusion_provenance_topic", provenance_topic_,
                        output_topic_ + "/provenance");
    private_node_.param("primary_frame_id", primary_frame_id_,
                        std::string("eo_camera_optical"));
    private_node_.param("secondary_frame_id", secondary_frame_id_,
                        std::string("ir_camera_optical"));
    private_node_.param("sync_tolerance_sec", sync_tolerance_sec_, 0.08);
    private_node_.param("late_fusion_hold_sec", hold_sec_, 0.10);
    private_node_.param("matching_iou_threshold", matching_iou_threshold_, 0.30);
    private_node_.param("secondary_only_confidence_scale",
                        secondary_only_scale_, 0.80);
    private_node_.param("registration_verified", registration_verified_, false);
    private_node_.param("registration_mode", registration_mode_,
                        std::string("homography"));
    private_node_.param("calibration_version", calibration_version_,
                        std::string());
    private_node_.param("require_versioned_calibration",
                        require_versioned_calibration_, true);
    private_node_.param("require_matching_frames", require_matching_frames_,
                        true);
    private_node_.param("fusion_queue_size", fusion_queue_size_, 10);
    private_node_.param("confidence_fusion_mode", confidence_fusion_mode_,
                        std::string("weighted_mean"));
    private_node_.param("person_class_id", output_class_id_, 0);
    private_node_.param("primary_width", primary_width_, 0);
    private_node_.param("primary_height", primary_height_, 0);
    primary_human_ids_ =
        integerArrayParameter(private_node_, "primary_human_class_ids");
    secondary_human_ids_ =
        integerArrayParameter(private_node_, "secondary_human_class_ids");
    const std::vector<double> homography_values =
        numericArrayParameter(private_node_, "secondary_to_primary_homography");
    if (primary_human_ids_.empty()) primary_human_ids_ = {0};
    if (secondary_human_ids_.empty()) secondary_human_ids_ = {0};
    if (!registration_verified_) {
      throw std::invalid_argument(
          "EO/IR late fusion requires ~registration_verified:=true after "
          "camera calibration; it will not assume pixel alignment");
    }
    if (registration_mode_ != "homography") {
      throw std::invalid_argument(
          "This image-space node only supports registration_mode=homography; "
          "use planning-facing localized candidates for world-coordinate fusion");
    }
    if (require_versioned_calibration_ && calibration_version_.empty()) {
      throw std::invalid_argument(
          "EO/IR fusion requires a non-empty calibration_version");
    }
    if (homography_values.size() != 9) {
      throw std::invalid_argument(
          "~secondary_to_primary_homography must contain 9 calibrated values");
    }
    homography_ = cv::Mat(3, 3, CV_64F);
    for (int index = 0; index < 9; ++index) {
      if (!std::isfinite(homography_values[index]))
        throw std::invalid_argument("Homography contains a non-finite value");
      homography_.at<double>(index / 3, index % 3) = homography_values[index];
    }
    if (std::abs(cv::determinant(homography_)) < 1e-12) {
      throw std::invalid_argument("Secondary-to-primary homography is singular");
    }
    sync_tolerance_sec_ = clamp(sync_tolerance_sec_, 0.005, 1.0);
    hold_sec_ = clamp(hold_sec_, sync_tolerance_sec_, 1.0);
    matching_iou_threshold_ = clamp(matching_iou_threshold_, 0.0, 1.0);
    secondary_only_scale_ = clamp(secondary_only_scale_, 0.0, 1.0);
    fusion_queue_size_ = std::max(2, fusion_queue_size_);
    if (confidence_fusion_mode_ != "weighted_mean" &&
        confidence_fusion_mode_ != "independent" &&
        confidence_fusion_mode_ != "maximum") {
      throw std::invalid_argument(
          "confidence_fusion_mode must be weighted_mean, maximum, or independent");
    }
    publisher_ = node_.advertise<vision_msgs::Detection2DArray>(output_topic_, 5);
    provenance_pub_ =
        node_.advertise<sar_yolo_detector::FusionProvenanceArray>(
            provenance_topic_, 5);
    vision_info_pub_ = node_.advertise<vision_msgs::VisionInfo>(
        output_topic_ + "/vision_info", 1, true);
    primary_sub_ = node_.subscribe(primary_topic_, 3, &LateFusionNode::primaryCallback,
                                   this, ros::TransportHints().tcpNoDelay());
    secondary_sub_ = node_.subscribe(secondary_topic_, 3,
                                     &LateFusionNode::secondaryCallback, this,
                                     ros::TransportHints().tcpNoDelay());
    timer_ = node_.createWallTimer(ros::WallDuration(0.02),
                                   &LateFusionNode::timerCallback, this);
    updater_.setHardwareID("sar_eo_ir_late_fusion");
    updater_.add("eo_ir_late_fusion", this, &LateFusionNode::diagnostics);
    vision_msgs::VisionInfo info;
    info.header.stamp = ros::Time::now();
    info.header.frame_id = primary_frame_id_;
    info.method = "calibrated_eo_ir_late_fusion";
    info.database_location = "rosparam://" + private_node_.getNamespace() +
                             "/secondary_to_primary_homography";
    info.database_version = 1;
    vision_info_pub_.publish(info);
    ROS_INFO_STREAM("[SarLateFusion] EO=" << primary_topic_ << " IR="
                    << secondary_topic_ << " output=" << output_topic_);
  }

private:
  bool validFrame(const vision_msgs::Detection2DArray::ConstPtr &message,
                  const bool primary) {
    if (message->header.stamp.isZero()) {
      ++rejected_frames_;
      return false;
    }
    ros::Time &last_stamp = primary ? last_primary_stamp_ : last_secondary_stamp_;
    if (!last_stamp.isZero() && message->header.stamp <= last_stamp) {
      ++rejected_frames_;
      return false;
    }
    const std::string &expected = primary ? primary_frame_id_ : secondary_frame_id_;
    if (require_matching_frames_ && !expected.empty() &&
        message->header.frame_id != expected) {
      ++rejected_frames_;
      return false;
    }
    last_stamp = message->header.stamp;
    return true;
  }

  void enqueue(std::deque<PendingFrame> *queue,
               const vision_msgs::Detection2DArray::ConstPtr &message,
               const bool primary) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!validFrame(message, primary)) {
      ROS_WARN_STREAM_THROTTLE(
          2.0, "[SarLateFusion] Rejected "
                   << (primary ? "primary" : "secondary")
                   << " frame: timestamp/frame contract violation");
      return;
    }
    queue->push_back({message, ros::WallTime::now()});
    if (primary)
      ++primary_messages_;
    else
      ++secondary_messages_;
    while (queue->size() > static_cast<std::size_t>(fusion_queue_size_)) {
      const PendingFrame overflow = queue->front();
      queue->pop_front();
      if (primary)
        publish(overflow.message, vision_msgs::Detection2DArray::ConstPtr(),
                false);
      else
        publish(vision_msgs::Detection2DArray::ConstPtr(), overflow.message,
                false);
      ++queue_overflow_count_;
    }
    flushLocked(false);
  }

  void primaryCallback(const vision_msgs::Detection2DArray::ConstPtr &message) {
    enqueue(&primary_queue_, message, true);
  }

  void secondaryCallback(const vision_msgs::Detection2DArray::ConstPtr &message) {
    enqueue(&secondary_queue_, message, false);
  }

  void timerCallback(const ros::WallTimerEvent &) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      flushLocked(true);
    }
    // diagnostics() takes the same mutex to snapshot counters.  Do not call
    // Updater::update() while the fusion state lock is held, otherwise its
    // callback self-deadlocks on the first timer tick.
    updater_.update();
  }

  std::vector<Detection> humanDetections(
      const vision_msgs::Detection2DArray &message,
      const std::vector<int> &human_ids, const bool transform_secondary) const {
    std::vector<Detection> output;
    for (std::size_t source_index = 0;
         source_index < message.detections.size(); ++source_index) {
      const auto &message_detection = message.detections[source_index];
      if (!std::isfinite(message_detection.bbox.center.x) ||
          !std::isfinite(message_detection.bbox.center.y) ||
          !std::isfinite(message_detection.bbox.size_x) ||
          !std::isfinite(message_detection.bbox.size_y) ||
          message_detection.bbox.size_x <= 0.0 || message_detection.bbox.size_y <= 0.0) {
        continue;
      }
      float confidence = -1.0F;
      for (const auto &hypothesis : message_detection.results) {
        if (contains(human_ids, static_cast<int>(hypothesis.id)) &&
            std::isfinite(hypothesis.score)) {
          confidence = std::max(confidence,
                                static_cast<float>(hypothesis.score));
        }
      }
      if (confidence < 0.0F)
        continue;
      cv::Rect2d box(message_detection.bbox.center.x -
                         0.5 * message_detection.bbox.size_x,
                     message_detection.bbox.center.y -
                         0.5 * message_detection.bbox.size_y,
                     message_detection.bbox.size_x, message_detection.bbox.size_y);
      if (transform_secondary) {
        std::vector<cv::Point2d> source = {
            {box.x, box.y}, {box.x + box.width, box.y},
            {box.x + box.width, box.y + box.height}, {box.x, box.y + box.height}};
        std::vector<cv::Point2d> projected;
        cv::perspectiveTransform(source, projected, homography_);
        if (projected.size() != 4)
          continue;
        const auto x_bounds = std::minmax_element(
            projected.begin(), projected.end(),
            [](const cv::Point2d &left, const cv::Point2d &right) {
              return left.x < right.x;
            });
        const auto y_bounds = std::minmax_element(
            projected.begin(), projected.end(),
            [](const cv::Point2d &left, const cv::Point2d &right) {
              return left.y < right.y;
            });
        box = cv::Rect2d(x_bounds.first->x, y_bounds.first->y,
                         x_bounds.second->x - x_bounds.first->x,
                         y_bounds.second->y - y_bounds.first->y);
      }
      if (primary_width_ > 0 && primary_height_ > 0) {
        const double x1 = clamp(box.x, 0.0, primary_width_);
        const double y1 = clamp(box.y, 0.0, primary_height_);
        const double x2 = clamp(box.x + box.width, 0.0, primary_width_);
        const double y2 = clamp(box.y + box.height, 0.0, primary_height_);
        box = cv::Rect2d(x1, y1, x2 - x1, y2 - y1);
      }
      if (box.width > 0.0 && box.height > 0.0)
        output.push_back({box, static_cast<float>(clamp(confidence, 0.0, 1.0)),
                          static_cast<std::uint32_t>(source_index)});
    }
    return output;
  }

  void publish(const vision_msgs::Detection2DArray::ConstPtr &primary,
               const vision_msgs::Detection2DArray::ConstPtr &secondary,
               const bool paired) {
    const auto &header_source = primary ? primary->header : secondary->header;
    vision_msgs::Detection2DArray output;
    output.header = header_source;
    output.header.frame_id = primary_frame_id_.empty() ? header_source.frame_id
                                                        : primary_frame_id_;
    std::vector<Detection> primary_detections =
        primary ? humanDetections(*primary, primary_human_ids_, false)
                : std::vector<Detection>();
    std::vector<Detection> secondary_detections =
        secondary ? humanDetections(*secondary, secondary_human_ids_, true)
                  : std::vector<Detection>();
    std::vector<std::vector<double>> costs(
        primary_detections.size(),
        std::vector<double>(secondary_detections.size(),
                            std::numeric_limits<double>::infinity()));
    for (std::size_t primary_index = 0;
         primary_index < primary_detections.size(); ++primary_index) {
      for (std::size_t secondary_index = 0;
           secondary_index < secondary_detections.size(); ++secondary_index) {
        const double overlap = iou(primary_detections[primary_index].box,
                                   secondary_detections[secondary_index].box);
        if (overlap >= matching_iou_threshold_)
          costs[primary_index][secondary_index] = 1.0 - overlap;
      }
    }
    const std::vector<int> assignments =
        sar_yolo_detector::minimumCostAssignment(costs, 1.0);
    std::vector<bool> secondary_used(secondary_detections.size(), false);
    std::vector<Detection> fused;
    sar_yolo_detector::FusionProvenanceArray provenance;
    provenance.header = output.header;
    provenance.sequence = next_provenance_sequence_++;
    for (std::size_t primary_index = 0;
         primary_index < primary_detections.size(); ++primary_index) {
      const Detection &eo = primary_detections[primary_index];
      const int best = assignments.empty() ? -1 : assignments[primary_index];
      sar_yolo_detector::FusionDetectionProvenance source;
      source.output_index = static_cast<std::uint32_t>(fused.size());
      source.has_primary = true;
      source.primary_index = eo.source_index;
      source.primary_stamp = primary->header.stamp;
      source.primary_confidence = eo.confidence;
      source.registration_mode = registration_mode_;
      source.calibration_version = calibration_version_;
      if (best < 0) {
        fused.push_back(eo);
        provenance.detections.push_back(source);
        continue;
      }
      secondary_used[best] = true;
      const Detection &ir = secondary_detections[best];
      source.has_secondary = true;
      source.secondary_index = ir.source_index;
      source.secondary_stamp = secondary->header.stamp;
      source.secondary_confidence = ir.confidence;
      const double total = std::max(1e-6, static_cast<double>(eo.confidence + ir.confidence));
      float fused_confidence = 0.5F * (eo.confidence + ir.confidence);
      if (confidence_fusion_mode_ == "maximum") {
        fused_confidence = std::max(eo.confidence, ir.confidence);
      } else if (confidence_fusion_mode_ == "independent") {
        fused_confidence = static_cast<float>(
            1.0 - (1.0 - eo.confidence) * (1.0 - ir.confidence));
      }
      fused.push_back({cv::Rect2d((eo.box.x * eo.confidence + ir.box.x * ir.confidence) / total,
                                  (eo.box.y * eo.confidence + ir.box.y * ir.confidence) / total,
                                  (eo.box.width * eo.confidence + ir.box.width * ir.confidence) / total,
                                  (eo.box.height * eo.confidence + ir.box.height * ir.confidence) / total),
                       fused_confidence,
                       eo.source_index});
      provenance.detections.push_back(source);
    }
    for (std::size_t index = 0; index < secondary_detections.size(); ++index) {
      if (!secondary_used[index]) {
        Detection standalone = secondary_detections[index];
        standalone.confidence *= secondary_only_scale_;
        sar_yolo_detector::FusionDetectionProvenance source;
        source.output_index = static_cast<std::uint32_t>(fused.size());
        source.has_secondary = true;
        source.secondary_index = standalone.source_index;
        source.secondary_stamp = secondary->header.stamp;
        source.secondary_confidence = secondary_detections[index].confidence;
        source.registration_mode = registration_mode_;
        source.calibration_version = calibration_version_;
        fused.push_back(standalone);
        provenance.detections.push_back(source);
      }
    }
    for (const Detection &detection : fused) {
      vision_msgs::Detection2D message;
      message.header = output.header;
      message.bbox.center.x = detection.box.x + 0.5 * detection.box.width;
      message.bbox.center.y = detection.box.y + 0.5 * detection.box.height;
      message.bbox.size_x = detection.box.width;
      message.bbox.size_y = detection.box.height;
      vision_msgs::ObjectHypothesisWithPose result;
      result.id = output_class_id_;
      result.score = detection.confidence;
      message.results.push_back(result);
      output.detections.push_back(message);
    }
    publisher_.publish(output);
    provenance_pub_.publish(provenance);
    ++published_frames_;
    published_detections_ += output.detections.size();
    if (paired) ++paired_frames_;
    else if (primary) ++primary_only_frames_;
    else ++secondary_only_frames_;
  }

  void flushLocked(const bool allow_timeout) {
    bool progressed = true;
    while (progressed) {
      progressed = false;
      if (!primary_queue_.empty() && !secondary_queue_.empty()) {
        const ros::Time primary_stamp =
            primary_queue_.front().message->header.stamp;
        const ros::Time secondary_stamp =
            secondary_queue_.front().message->header.stamp;
        const double signed_delta = (primary_stamp - secondary_stamp).toSec();
        if (std::abs(signed_delta) <= sync_tolerance_sec_) {
          publish(primary_queue_.front().message,
                  secondary_queue_.front().message, true);
          primary_queue_.pop_front();
          secondary_queue_.pop_front();
          progressed = true;
          continue;
        }

        // Timestamps are required to be strictly increasing on each input.
        // Therefore an older queue front can no longer match a future frame.
        if (signed_delta < 0.0) {
          publish(primary_queue_.front().message,
                  vision_msgs::Detection2DArray::ConstPtr(), false);
          primary_queue_.pop_front();
        } else {
          publish(vision_msgs::Detection2DArray::ConstPtr(),
                  secondary_queue_.front().message, false);
          secondary_queue_.pop_front();
        }
        ++timing_miss_count_;
        progressed = true;
        continue;
      }

      if (!allow_timeout)
        continue;
      const ros::WallTime now = ros::WallTime::now();
      if (!primary_queue_.empty() &&
          (now - primary_queue_.front().received).toSec() >= hold_sec_) {
        publish(primary_queue_.front().message,
                vision_msgs::Detection2DArray::ConstPtr(), false);
        primary_queue_.pop_front();
        ++timing_miss_count_;
        progressed = true;
      }
      if (!secondary_queue_.empty() &&
          (now - secondary_queue_.front().received).toSec() >= hold_sec_) {
        publish(vision_msgs::Detection2DArray::ConstPtr(),
                secondary_queue_.front().message, false);
        secondary_queue_.pop_front();
        ++timing_miss_count_;
        progressed = true;
      }
    }
  }

  void diagnostics(diagnostic_updater::DiagnosticStatusWrapper &status) {
    std::lock_guard<std::mutex> lock(mutex_);
    const int level = timing_miss_count_ > paired_frames_
        ? diagnostic_msgs::DiagnosticStatus::WARN
        : diagnostic_msgs::DiagnosticStatus::OK;
    status.summary(level, level == diagnostic_msgs::DiagnosticStatus::OK
                              ? "Calibrated EO/IR late fusion running"
                              : "EO/IR timing mismatch; publishing unimodal fallback");
    status.add("registration_verified", registration_verified_);
    status.add("registration_mode", registration_mode_);
    status.add("calibration_version", calibration_version_);
    status.add("confidence_fusion_mode", confidence_fusion_mode_);
    status.add("sync_tolerance_sec", sync_tolerance_sec_);
    status.add("late_fusion_hold_sec", hold_sec_);
    status.add("primary_queue_depth", static_cast<int>(primary_queue_.size()));
    status.add("secondary_queue_depth", static_cast<int>(secondary_queue_.size()));
    status.add("queue_overflows", static_cast<long long>(queue_overflow_count_));
    status.add("rejected_frames", static_cast<long long>(rejected_frames_));
    status.add("primary_messages", static_cast<long long>(primary_messages_));
    status.add("secondary_messages", static_cast<long long>(secondary_messages_));
    status.add("paired_frames", static_cast<long long>(paired_frames_));
    status.add("primary_only_frames", static_cast<long long>(primary_only_frames_));
    status.add("secondary_only_frames", static_cast<long long>(secondary_only_frames_));
    status.add("timing_misses", static_cast<long long>(timing_miss_count_));
    status.add("published_frames", static_cast<long long>(published_frames_));
    status.add("published_detections", static_cast<long long>(published_detections_));
  }

  ros::NodeHandle node_, private_node_;
  ros::Publisher publisher_, vision_info_pub_, provenance_pub_;
  ros::Subscriber primary_sub_, secondary_sub_;
  ros::WallTimer timer_;
  diagnostic_updater::Updater updater_;
  std::mutex mutex_;
  std::deque<PendingFrame> primary_queue_, secondary_queue_;
  cv::Mat homography_;
  std::string primary_topic_, secondary_topic_, output_topic_, provenance_topic_;
  std::string primary_frame_id_, secondary_frame_id_, registration_mode_;
  std::string calibration_version_, confidence_fusion_mode_;
  std::vector<int> primary_human_ids_, secondary_human_ids_;
  double sync_tolerance_sec_{0.08}, hold_sec_{0.10}, matching_iou_threshold_{0.30};
  double secondary_only_scale_{0.80};
  bool registration_verified_{false}, require_versioned_calibration_{true};
  bool require_matching_frames_{true};
  int output_class_id_{0}, primary_width_{0}, primary_height_{0};
  int fusion_queue_size_{10};
  ros::Time last_primary_stamp_, last_secondary_stamp_;
  std::uint64_t primary_messages_{0}, secondary_messages_{0}, paired_frames_{0};
  std::uint64_t primary_only_frames_{0}, secondary_only_frames_{0}, timing_miss_count_{0};
  std::uint64_t published_frames_{0}, published_detections_{0};
  std::uint64_t queue_overflow_count_{0}, rejected_frames_{0};
  std::uint64_t next_provenance_sequence_{1};
};

}  // namespace

int main(int argc, char **argv) {
  ros::init(argc, argv, "sar_detection_late_fusion");
  try {
    LateFusionNode node;
    ros::spin();
  } catch (const std::exception &exception) {
    std::cerr << "[SarLateFusion] Startup failed: " << exception.what()
              << std::endl;
    ROS_FATAL_STREAM("[SarLateFusion] Startup failed: " << exception.what());
    return 1;
  }
  return 0;
}
