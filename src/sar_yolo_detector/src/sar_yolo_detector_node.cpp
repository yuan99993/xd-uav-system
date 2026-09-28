#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <limits>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <XmlRpcValue.h>
#include <cv_bridge/cv_bridge.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <image_transport/image_transport.h>
#include <opencv2/dnn/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Float32.h>
#include <vision_msgs/Detection2D.h>
#include <vision_msgs/Detection2DArray.h>
#include <vision_msgs/ObjectHypothesisWithPose.h>
#include <vision_msgs/VisionInfo.h>

#include <sar_yolo_detector/inference_backend.hpp>
#include <sar_yolo_detector/model_integrity.hpp>
#include <sar_yolo_detector/runtime_identity.hpp>
#include <sar_yolo_detector/ThermalImageInfo.h>
#include <sar_yolo_detector/yolo_postprocessor.hpp>

namespace sar_yolo_detector {
namespace {

double millisecondsSince(const std::chrono::steady_clock::time_point &start) {
  return std::chrono::duration<double, std::milli>(
             std::chrono::steady_clock::now() - start)
      .count();
}

double intersectionOverUnion(const cv::Rect &left, const cv::Rect &right) {
  const cv::Rect intersection = left & right;
  const double union_area = static_cast<double>(left.area()) + right.area() -
                            static_cast<double>(intersection.area());
  return union_area <= 0.0 ? 0.0 : intersection.area() / union_area;
}

void mergeRoiDetections(std::vector<Detection> *full_frame,
                        std::vector<Detection> roi_detections,
                        const double suppression_iou) {
  if (full_frame == nullptr)
    return;
  // A recheck is a recovery/confirmation pass for the selected ROI, not a
  // second detector stream.  When the full frame already has detections, do
  // not append a spatially unrelated ROI box: that can create competing
  // same-class candidates and destabilize selected-track association.
  const bool full_frame_was_empty = full_frame->empty();
  for (const Detection &candidate : roi_detections) {
    bool merged = false;
    for (Detection &existing : *full_frame) {
      if (existing.class_id == candidate.class_id &&
          intersectionOverUnion(existing.box, candidate.box) >= suppression_iou) {
        if (candidate.confidence > existing.confidence)
          existing = candidate;
        merged = true;
        break;
      }
    }
    if (!merged && full_frame_was_empty)
      full_frame->push_back(candidate);
  }
  std::sort(full_frame->begin(), full_frame->end(),
            [](const Detection &left, const Detection &right) {
              return left.confidence > right.confidence;
            });
}

double intersectionOverMinimumArea(const cv::Rect &left,
                                   const cv::Rect &right) {
  const cv::Rect intersection = left & right;
  const double minimum_area = std::min(left.area(), right.area());
  return minimum_area <= 0.0 ? 0.0 : intersection.area() / minimum_area;
}

void globallyMergeDetections(std::vector<Detection> *detections,
                             const double overlap_threshold,
                             const int maximum_detections) {
  if (detections == nullptr)
    return;
  std::sort(detections->begin(), detections->end(),
            [](const Detection &left, const Detection &right) {
              return left.confidence > right.confidence;
            });
  std::vector<Detection> kept;
  kept.reserve(std::min<std::size_t>(
      detections->size(), static_cast<std::size_t>(maximum_detections)));
  for (const Detection &candidate : *detections) {
    bool duplicate = false;
    for (const Detection &existing : kept) {
      if (candidate.class_id == existing.class_id &&
          (intersectionOverUnion(candidate.box, existing.box) >=
               overlap_threshold ||
           intersectionOverMinimumArea(candidate.box, existing.box) >=
               std::max(0.80, overlap_threshold))) {
        duplicate = true;
        break;
      }
    }
    if (!duplicate)
      kept.push_back(candidate);
    if (static_cast<int>(kept.size()) >= maximum_detections)
      break;
  }
  *detections = std::move(kept);
}

std::vector<int> tileOrigins(const int length, const int tile_length,
                             const double overlap_ratio) {
  if (tile_length >= length)
    return {0};
  const int stride = std::max(
      1, static_cast<int>(std::round(tile_length * (1.0 - overlap_ratio))));
  std::vector<int> origins;
  for (int origin = 0; origin + tile_length < length; origin += stride)
    origins.push_back(origin);
  const int final_origin = length - tile_length;
  if (origins.empty() || origins.back() != final_origin)
    origins.push_back(final_origin);
  return origins;
}

struct PreparedImage {
  cv::Mat bgr;
  bool thermal{false};
  bool radiometric{false};
  double lower_raw{0.0};
  double upper_raw{255.0};
  bool bad_pixel_corrected{false};
  bool clahe_applied{false};
};

std::vector<std::string> stringArrayParameter(ros::NodeHandle &node,
                                              const std::string &name) {
  XmlRpc::XmlRpcValue value;
  if (!node.getParam(name, value) ||
      value.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    return {};
  }
  std::vector<std::string> result;
  result.reserve(value.size());
  for (int index = 0; index < value.size(); ++index) {
    if (value[index].getType() != XmlRpc::XmlRpcValue::TypeString) {
      throw std::invalid_argument("Parameter " + name +
                                  " must be a string array");
    }
    result.push_back(static_cast<std::string>(value[index]));
  }
  return result;
}

std::vector<int> integerArrayParameter(ros::NodeHandle &node,
                                       const std::string &name) {
  XmlRpc::XmlRpcValue value;
  if (!node.getParam(name, value) ||
      value.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    return {};
  }
  std::vector<int> result;
  result.reserve(value.size());
  for (int index = 0; index < value.size(); ++index) {
    if (value[index].getType() != XmlRpc::XmlRpcValue::TypeInt) {
      throw std::invalid_argument("Parameter " + name +
                                  " must be an integer array");
    }
    result.push_back(static_cast<int>(value[index]));
  }
  return result;
}

double numberValue(const XmlRpc::XmlRpcValue &value,
                   const std::string &parameter_name) {
  if (value.getType() == XmlRpc::XmlRpcValue::TypeInt) {
    return static_cast<int>(value);
  }
  if (value.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
    return static_cast<double>(value);
  }
  throw std::invalid_argument("Parameter " + parameter_name +
                              " must contain numeric values");
}

std::vector<TestDetection> testDetectionsParameter(ros::NodeHandle &node,
                                                   const std::string &name) {
  XmlRpc::XmlRpcValue value;
  if (!node.getParam(name, value) ||
      value.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    return {};
  }
  std::vector<TestDetection> result;
  result.reserve(value.size());
  for (int index = 0; index < value.size(); ++index) {
    if (value[index].getType() != XmlRpc::XmlRpcValue::TypeArray ||
        value[index].size() != 6) {
      throw std::invalid_argument(
          "Each " + name +
          " entry must be [cx, cy, width, height, confidence, class_id]");
    }
    const XmlRpc::XmlRpcValue &entry = value[index];
    TestDetection detection;
    detection.center_x = static_cast<float>(numberValue(entry[0], name));
    detection.center_y = static_cast<float>(numberValue(entry[1], name));
    detection.width = static_cast<float>(numberValue(entry[2], name));
    detection.height = static_cast<float>(numberValue(entry[3], name));
    detection.confidence = static_cast<float>(numberValue(entry[4], name));
    detection.class_id = static_cast<int>(numberValue(entry[5], name));
    result.push_back(detection);
  }
  return result;
}

cv::Scalar colorForClass(const int class_id) {
  const int seed = std::max(0, class_id) + 1;
  return cv::Scalar((37 * seed) % 205 + 50, (17 * seed) % 205 + 50,
                    (29 * seed) % 205 + 50);
}

} // namespace

class SarYoloDetectorNode {
public:
  SarYoloDetectorNode() : private_node_("~"), image_transport_(node_) {
    loadParameters();
    loadInferenceBackend();

    detections_publisher_ =
        node_.advertise<vision_msgs::Detection2DArray>(detections_topic_, 5);
    vision_info_publisher_ =
        node_.advertise<vision_msgs::VisionInfo>(vision_info_topic_, 1, true);
    thermal_info_publisher_ =
        node_.advertise<sar_yolo_detector::ThermalImageInfo>(
            thermal_info_topic_, 2);
    if (publish_debug_image_) {
      debug_publisher_ = image_transport_.advertise(debug_image_topic_, 1);
    }
    image_subscriber_ = image_transport_.subscribe(
        input_image_topic_, 1, &SarYoloDetectorNode::imageCallback, this,
        image_transport::TransportHints("raw",
                                        ros::TransportHints().tcpNoDelay()));
    if (roi_recheck_enabled_) {
      roi_recheck_subscriber_ = node_.subscribe(
          roi_recheck_topic_, 1, &SarYoloDetectorNode::roiRecheckCallback,
          this, ros::TransportHints().tcpNoDelay());
    }
    if (tiling_policy_ == "height") {
      height_subscriber_ = node_.subscribe(
          height_topic_, 3, &SarYoloDetectorNode::heightCallback, this,
          ros::TransportHints().tcpNoDelay());
    } else if (tiling_policy_ == "gsd") {
      gsd_subscriber_ = node_.subscribe(
          gsd_topic_, 3, &SarYoloDetectorNode::gsdCallback, this,
          ros::TransportHints().tcpNoDelay());
    }

    diagnostics_.setHardwareID("sar_yolo_" + sensor_id_);
    diagnostics_.add("yolo_detector", this,
                     &SarYoloDetectorNode::diagnosticCallback);
    diagnostic_timer_ =
        node_.createTimer(ros::Duration(1.0),
                          &SarYoloDetectorNode::diagnosticTimerCallback, this);
    publishVisionInfo();
    worker_ = std::thread(&SarYoloDetectorNode::workerLoop, this);

    ROS_INFO_STREAM("[SarYoloDetector] Ready: model="
                    << model_path_ << " input=" << input_image_topic_
                    << " output=" << detections_topic_
                    << " backend=" << inference_backend_->name() << " ("
                    << inference_backend_->details() << ")");
  }

  ~SarYoloDetectorNode() {
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      stop_requested_ = true;
    }
    queue_condition_.notify_all();
    if (worker_.joinable())
      worker_.join();
  }

private:
  void loadParameters() {
    private_node_.param("model_path", model_path_, std::string());
    private_node_.param("model_version", model_version_,
                        std::string("unknown"));
    private_node_.param("model_sha256", model_sha256_, std::string());
    private_node_.param("require_model_sha256", require_model_sha256_, false);
    private_node_.param("rescue_profile", rescue_profile_,
                        std::string("generic"));
    private_node_.param("input_image_topic", input_image_topic_,
                        std::string("camera/image_raw"));
    private_node_.param("detections_topic", detections_topic_,
                        std::string("sar_yolo_detector/detections"));
    private_node_.param("vision_info_topic", vision_info_topic_,
                        std::string("sar_yolo_detector/vision_info"));
    private_node_.param("debug_image_topic", debug_image_topic_,
                        std::string("sar_yolo_detector/debug_image"));
    private_node_.param("thermal_info_topic", thermal_info_topic_,
                        std::string("sar_yolo_detector/thermal_info"));
    private_node_.param("image_source", image_source_, std::string("eo"));
    private_node_.param("sensor_id", sensor_id_, std::string("camera_primary"));
    private_node_.param("detector_name", detector_name_,
                        std::string("sar_yolo_opencv_dnn"));
    private_node_.param("inference_backend", inference_backend_name_,
                        std::string());
    private_node_.param("opencv_target", opencv_target_, std::string());
    // dnn_backend is retained as a backward-compatible alias for the old
    // OpenCV-only node configuration.
    private_node_.param("dnn_backend", legacy_dnn_backend_,
                        std::string("opencv_cpu"));
    private_node_.param("engine_path", engine_path_, std::string());
    private_node_.param("network_config_path", network_config_path_,
                        std::string());
    private_node_.param("darknet_coordinates_normalized",
                        darknet_coordinates_normalized_, true);
    private_node_.param("roi_recheck_enabled", roi_recheck_enabled_, false);
    private_node_.param("roi_recheck_topic", roi_recheck_topic_,
                        std::string("tracker_node/roi_recheck_request"));
    private_node_.param("roi_recheck_interval_sec", roi_recheck_interval_sec_,
                        0.4);
    private_node_.param("roi_recheck_context_scale", roi_recheck_context_scale_,
                        1.8);
    private_node_.param("roi_recheck_max_capture_delta_sec",
                        roi_recheck_max_capture_delta_sec_, 0.35);
    private_node_.param("roi_recheck_merge_iou", roi_recheck_merge_iou_, 0.55);
    private_node_.param("model_layout", model_layout_name_,
                        std::string("auto"));
    private_node_.param("input_width", postprocessor_config_.input_width, 640);
    private_node_.param("input_height", postprocessor_config_.input_height,
                        640);
    private_node_.param("confidence_threshold",
                        postprocessor_config_.confidence_threshold, 0.25F);
    private_node_.param("nms_threshold", postprocessor_config_.nms_threshold,
                        0.45F);
    private_node_.param("minimum_box_size_px",
                        postprocessor_config_.minimum_box_size_px, 2.0F);
    private_node_.param("maximum_detections",
                        postprocessor_config_.maximum_detections, 300);
    private_node_.param("class_agnostic_nms",
                        postprocessor_config_.class_agnostic_nms, false);
    private_node_.param("publish_debug_image", publish_debug_image_, false);
    private_node_.param("require_capture_timestamp", require_capture_timestamp_,
                        true);
    private_node_.param("reject_out_of_order", reject_out_of_order_, true);
    private_node_.param("future_timestamp_tolerance_sec",
                        future_timestamp_tolerance_sec_, 0.02);
    private_node_.param("maximum_capture_age_sec", maximum_capture_age_sec_,
                        0.50);
    private_node_.param("stale_input_warning_sec", stale_input_warning_sec_,
                        1.0);
    private_node_.param("swap_rb", swap_rb_, true);
    private_node_.param("input_mode", input_mode_, std::string("color"));
    private_node_.param("thermal_normalization", thermal_normalization_,
                        std::string("percentile"));
    private_node_.param("thermal_lower_percentile",
                        thermal_lower_percentile_, 1.0);
    private_node_.param("thermal_upper_percentile",
                        thermal_upper_percentile_, 99.0);
    private_node_.param("thermal_fixed_min", thermal_fixed_min_, 0.0);
    private_node_.param("thermal_fixed_max", thermal_fixed_max_, 65535.0);
    private_node_.param("thermal_bad_pixel_correction",
                        thermal_bad_pixel_correction_, false);
    private_node_.param("thermal_clahe_enabled", thermal_clahe_enabled_, false);
    private_node_.param("thermal_clahe_clip_limit",
                        thermal_clahe_clip_limit_, 2.0);
    private_node_.param("temperature_scale", temperature_scale_, 1.0);
    private_node_.param("temperature_offset", temperature_offset_, 0.0);
    private_node_.param("temperature_unit", temperature_unit_,
                        std::string("raw"));
    private_node_.param("tiling_enabled", tiling_enabled_, false);
    private_node_.param("tiling_policy", tiling_policy_,
                        std::string("always"));
    private_node_.param("height_topic", height_topic_,
                        std::string("altimeter/range"));
    private_node_.param("gsd_topic", gsd_topic_,
                        std::string("perception/gsd_m_per_px"));
    private_node_.param("tiling_min_height_m", tiling_min_height_m_, 40.0);
    private_node_.param("tiling_min_gsd_m_per_px",
                        tiling_min_gsd_m_per_px_, 0.05);
    private_node_.param("target_inference_gsd_m_per_px",
                        target_inference_gsd_m_per_px_, 0.03);
    private_node_.param("tiling_metric_timeout_sec",
                        tiling_metric_timeout_sec_, 1.0);
    private_node_.param("tile_width_px", tile_width_px_,
                        postprocessor_config_.input_width);
    private_node_.param("tile_height_px", tile_height_px_,
                        postprocessor_config_.input_height);
    private_node_.param("tile_overlap_ratio", tile_overlap_ratio_, 0.20);
    private_node_.param("tile_include_full_frame", tile_include_full_frame_,
                        true);
    private_node_.param("tile_merge_overlap", tile_merge_overlap_, 0.45);
    private_node_.param("database_version", database_version_, 1);
    postprocessor_config_.class_names =
        stringArrayParameter(private_node_, "class_names");
    postprocessor_config_.allowed_class_ids =
        integerArrayParameter(private_node_, "allowed_class_ids");
    test_detections_ =
        testDetectionsParameter(private_node_, "test_detections");
    postprocessor_config_.model_layout = parseModelLayout(model_layout_name_);
    class_names_ = postprocessor_config_.class_names;
    if (class_names_.empty()) {
      throw std::invalid_argument(
          "~class_names must match the ONNX output classes");
    }
    future_timestamp_tolerance_sec_ =
        std::max(0.0, future_timestamp_tolerance_sec_);
    maximum_capture_age_sec_ = std::max(0.0, maximum_capture_age_sec_);
    stale_input_warning_sec_ = std::max(0.1, stale_input_warning_sec_);
    roi_recheck_interval_sec_ = std::max(0.05, roi_recheck_interval_sec_);
    roi_recheck_context_scale_ = std::max(1.0, roi_recheck_context_scale_);
    roi_recheck_max_capture_delta_sec_ =
        std::max(0.01, roi_recheck_max_capture_delta_sec_);
    roi_recheck_merge_iou_ = std::max(0.0, std::min(1.0, roi_recheck_merge_iou_));
    if (input_mode_ != "color" && input_mode_ != "thermal_auto")
      throw std::invalid_argument("input_mode must be color or thermal_auto");
    if (thermal_normalization_ != "percentile" &&
        thermal_normalization_ != "fixed") {
      throw std::invalid_argument(
          "thermal_normalization must be percentile or fixed");
    }
    thermal_lower_percentile_ =
        std::max(0.0, std::min(99.0, thermal_lower_percentile_));
    thermal_upper_percentile_ =
        std::max(thermal_lower_percentile_ + 0.1,
                 std::min(100.0, thermal_upper_percentile_));
    if (thermal_fixed_max_ <= thermal_fixed_min_)
      throw std::invalid_argument("thermal_fixed_max must exceed thermal_fixed_min");
    tile_width_px_ = std::max(32, tile_width_px_);
    tile_height_px_ = std::max(32, tile_height_px_);
    tile_overlap_ratio_ = std::max(0.0, std::min(0.80, tile_overlap_ratio_));
    tile_merge_overlap_ = std::max(0.0, std::min(1.0, tile_merge_overlap_));
    if (tiling_policy_ != "always" && tiling_policy_ != "height" &&
        tiling_policy_ != "gsd") {
      throw std::invalid_argument(
          "tiling_policy must be always, height, or gsd");
    }
    tiling_min_height_m_ = std::max(0.0, tiling_min_height_m_);
    tiling_min_gsd_m_per_px_ = std::max(0.0, tiling_min_gsd_m_per_px_);
    target_inference_gsd_m_per_px_ =
        std::max(1.0e-5, target_inference_gsd_m_per_px_);
    tiling_metric_timeout_sec_ = std::max(0.05, tiling_metric_timeout_sec_);
    identity_ = loadPerceptionIdentity(&private_node_, sensor_id_,
                                       rescue_profile_, "IMAGE_PIXELS");
    // Keep legacy parameter names and the planning-facing identity consistent.
    identity_.profile = rescue_profile_;
    identity_.model_version = model_version_;
    identity_.model_sha256 = model_sha256_;
    postprocessor_.reset(new YoloPostprocessor(postprocessor_config_));
  }

  void loadInferenceBackend() {
    if (inference_backend_name_.empty()) {
      inference_backend_name_ = "opencv_dnn";
    }
    if (opencv_target_.empty())
      opencv_target_ = legacy_dnn_backend_;
    if (inference_backend_name_ != "test") {
      const std::string artifact = inference_backend_name_ == "tensorrt"
                                       ? engine_path_
                                       : model_path_;
      verifyModelArtifact(artifact, model_sha256_, require_model_sha256_);
    } else if (require_model_sha256_) {
      throw std::invalid_argument(
          "require_model_sha256 cannot be used with the synthetic test backend");
    }
    InferenceBackendConfig config;
    config.backend = inference_backend_name_;
    config.model_path = model_path_;
    config.network_config_path = network_config_path_;
    config.engine_path = engine_path_;
    config.opencv_target = opencv_target_;
    config.darknet_coordinates_normalized = darknet_coordinates_normalized_;
    config.class_count = static_cast<int>(class_names_.size());
    config.yolov5_layout =
        postprocessor_config_.model_layout == ModelLayout::kYoloV5;
    config.test_detections = test_detections_;
    config.tensorrt_output_binding_names =
        stringArrayParameter(private_node_, "tensorrt_output_binding_names");
    inference_backend_ = createInferenceBackend(config);
  }

  void publishVisionInfo() {
    vision_msgs::VisionInfo message;
    message.header.stamp = ros::Time::now();
    message.header.frame_id = image_source_;
    message.method = rescue_profile_ + "/" + detector_name_ + ":" +
                     model_version_;
    message.database_location =
        "rosparam://" + private_node_.getNamespace() + "/class_names";
    message.database_version = database_version_;
    vision_info_publisher_.publish(message);
  }

  bool timestampAccepted(const sensor_msgs::ImageConstPtr &message) {
    const ros::Time now = ros::Time::now();
    if (message->header.stamp.isZero()) {
      if (!require_capture_timestamp_)
        return true;
      rejectFrame("missing capture timestamp");
      return false;
    }
    if (!now.isZero()) {
      const double age = (now - message->header.stamp).toSec();
      if (age < -future_timestamp_tolerance_sec_) {
        rejectFrame("capture timestamp is in the future");
        return false;
      }
      if (maximum_capture_age_sec_ > 0.0 && age > maximum_capture_age_sec_) {
        rejectFrame("capture timestamp is stale");
        return false;
      }
    }
    std::lock_guard<std::mutex> lock(queue_mutex_);
    if (reject_out_of_order_ && !last_received_stamp_.isZero() &&
        message->header.stamp <= last_received_stamp_) {
      rejectFrameLocked("out-of-order capture timestamp");
      return false;
    }
    last_received_stamp_ = message->header.stamp;
    return true;
  }

  void imageCallback(const sensor_msgs::ImageConstPtr &message) {
    {
      std::lock_guard<std::mutex> lock(metrics_mutex_);
      ++received_frames_;
      last_input_wall_time_ = ros::WallTime::now();
    }
    if (message->width == 0 || message->height == 0 || message->data.empty()) {
      rejectFrame("empty image");
      return;
    }
    if (!timestampAccepted(message))
      return;
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      if (pending_image_) {
        std::lock_guard<std::mutex> metrics_lock(metrics_mutex_);
        ++dropped_frames_;
      }
      pending_image_ = message;
    }
    queue_condition_.notify_one();
  }

  void roiRecheckCallback(const vision_msgs::Detection2D::ConstPtr &message) {
    if (!roi_recheck_enabled_ || message->header.stamp.isZero() ||
        !std::isfinite(message->bbox.center.x) ||
        !std::isfinite(message->bbox.center.y) ||
        !std::isfinite(message->bbox.size_x) ||
        !std::isfinite(message->bbox.size_y) || message->bbox.size_x <= 0.0 ||
        message->bbox.size_y <= 0.0) {
      std::lock_guard<std::mutex> lock(metrics_mutex_);
      ++roi_recheck_rejected_requests_;
      return;
    }
    std::lock_guard<std::mutex> lock(roi_mutex_);
    roi_request_stamp_ = message->header.stamp;
    roi_request_frame_id_ = message->header.frame_id;
    roi_request_box_ = cv::Rect2d(
        message->bbox.center.x - 0.5 * message->bbox.size_x,
        message->bbox.center.y - 0.5 * message->bbox.size_y,
        message->bbox.size_x, message->bbox.size_y);
    have_roi_request_ = true;
    std::lock_guard<std::mutex> metrics_lock(metrics_mutex_);
    ++roi_recheck_requests_;
  }

  void heightCallback(const sensor_msgs::Range::ConstPtr &message) {
    if (!std::isfinite(message->range) || message->range <= 0.0F)
      return;
    std::lock_guard<std::mutex> lock(metrics_mutex_);
    latest_height_m_ = message->range;
    last_height_wall_ = ros::WallTime::now();
  }

  void gsdCallback(const std_msgs::Float32::ConstPtr &message) {
    if (!std::isfinite(message->data) || message->data <= 0.0F)
      return;
    std::lock_guard<std::mutex> lock(metrics_mutex_);
    latest_gsd_m_per_px_ = message->data;
    last_gsd_wall_ = ros::WallTime::now();
  }

  bool roiForImage(const sensor_msgs::ImageConstPtr &message,
                   const cv::Size &image_size, cv::Rect *roi) {
    if (!roi_recheck_enabled_ || roi == nullptr)
      return false;
    std::lock_guard<std::mutex> lock(roi_mutex_);
    if (!have_roi_request_ ||
        (!roi_request_frame_id_.empty() && !message->header.frame_id.empty() &&
         roi_request_frame_id_ != message->header.frame_id) ||
        std::abs((message->header.stamp - roi_request_stamp_).toSec()) >
            roi_recheck_max_capture_delta_sec_ ||
        (!last_roi_recheck_wall_.isZero() &&
         (ros::WallTime::now() - last_roi_recheck_wall_).toSec() <
             roi_recheck_interval_sec_)) {
      return false;
    }
    const double width = roi_request_box_.width * roi_recheck_context_scale_;
    const double height = roi_request_box_.height * roi_recheck_context_scale_;
    const double center_x = roi_request_box_.x + 0.5 * roi_request_box_.width;
    const double center_y = roi_request_box_.y + 0.5 * roi_request_box_.height;
    const int x1 = std::max(0, static_cast<int>(std::floor(center_x - width / 2.0)));
    const int y1 = std::max(0, static_cast<int>(std::floor(center_y - height / 2.0)));
    const int x2 = std::min(image_size.width,
                            static_cast<int>(std::ceil(center_x + width / 2.0)));
    const int y2 = std::min(image_size.height,
                            static_cast<int>(std::ceil(center_y + height / 2.0)));
    if (x2 <= x1 || y2 <= y1)
      return false;
    *roi = cv::Rect(x1, y1, x2 - x1, y2 - y1);
    last_roi_recheck_wall_ = ros::WallTime::now();
    return true;
  }

  void rejectFrame(const std::string &reason) {
    std::lock_guard<std::mutex> lock(metrics_mutex_);
    ++rejected_frames_;
    last_error_ = reason;
    ROS_WARN_STREAM_THROTTLE(2.0,
                             "[SarYoloDetector] Rejected frame: " << reason);
  }

  void rejectFrameLocked(const std::string &reason) {
    std::lock_guard<std::mutex> lock(metrics_mutex_);
    ++rejected_frames_;
    last_error_ = reason;
    ROS_WARN_STREAM_THROTTLE(2.0,
                             "[SarYoloDetector] Rejected frame: " << reason);
  }

  void workerLoop() {
    while (ros::ok()) {
      sensor_msgs::ImageConstPtr message;
      {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        queue_condition_.wait(lock, [this] {
          return stop_requested_ || static_cast<bool>(pending_image_);
        });
        if (stop_requested_)
          return;
        message = pending_image_;
        pending_image_.reset();
      }
      processImage(message);
    }
  }

  PreparedImage prepareImage(const sensor_msgs::ImageConstPtr &message) const {
    PreparedImage prepared;
    if (input_mode_ == "color") {
      prepared.bgr = cv_bridge::toCvCopy(
                         message, sensor_msgs::image_encodings::BGR8)
                         ->image;
      return prepared;
    }

    const cv_bridge::CvImageConstPtr native = cv_bridge::toCvShare(message);
    if (native->image.channels() != 1) {
      throw std::runtime_error(
          "thermal_auto requires a one-channel MONO8/MONO16/Y16 image");
    }
    cv::Mat raw_float;
    native->image.convertTo(raw_float, CV_32F);
    if (!raw_float.isContinuous())
      raw_float = raw_float.clone();
    prepared.thermal = true;
    prepared.radiometric = native->image.depth() == CV_16U ||
                           native->image.depth() == CV_16S ||
                           native->image.depth() == CV_32F;

    double lower = thermal_fixed_min_;
    double upper = thermal_fixed_max_;
    if (thermal_normalization_ == "percentile") {
      const std::size_t total = raw_float.total();
      const std::size_t stride = std::max<std::size_t>(1, total / 100000U);
      std::vector<float> samples;
      samples.reserve(total / stride + 1U);
      const float *pixels = raw_float.ptr<float>();
      for (std::size_t index = 0; index < total; index += stride) {
        if (std::isfinite(pixels[index]))
          samples.push_back(pixels[index]);
      }
      if (samples.empty())
        throw std::runtime_error("thermal image contains no finite samples");
      std::sort(samples.begin(), samples.end());
      const auto percentile = [&samples](const double value) {
        const std::size_t index = static_cast<std::size_t>(std::round(
            (value / 100.0) * static_cast<double>(samples.size() - 1U)));
        return static_cast<double>(samples[std::min(index, samples.size() - 1U)]);
      };
      lower = percentile(thermal_lower_percentile_);
      upper = percentile(thermal_upper_percentile_);
    }
    if (!std::isfinite(lower) || !std::isfinite(upper) || upper <= lower)
      throw std::runtime_error("thermal normalization range is degenerate");
    prepared.lower_raw = lower;
    prepared.upper_raw = upper;

    cv::Mat clipped;
    cv::max(raw_float, lower, clipped);
    cv::min(clipped, upper, clipped);
    cv::Mat gray8;
    clipped.convertTo(gray8, CV_8U, 255.0 / (upper - lower),
                      -255.0 * lower / (upper - lower));
    if (thermal_bad_pixel_correction_) {
      cv::medianBlur(gray8, gray8, 3);
      prepared.bad_pixel_corrected = true;
    }
    if (thermal_clahe_enabled_) {
      cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(
          std::max(0.1, thermal_clahe_clip_limit_), cv::Size(8, 8));
      clahe->apply(gray8, gray8);
      prepared.clahe_applied = true;
    }
    cv::cvtColor(gray8, prepared.bgr, cv::COLOR_GRAY2BGR);
    return prepared;
  }

  std::vector<Detection> inferRegion(const cv::Mat &image,
                                     const cv::Point &offset,
                                     double *preprocess_ms,
                                     double *inference_ms,
                                     double *postprocess_ms) {
    const auto preprocess_start = std::chrono::steady_clock::now();
    LetterboxTransform transform;
    const cv::Mat letterboxed = letterboxImage(
        image, postprocessor_config_.input_width,
        postprocessor_config_.input_height, &transform);
    const cv::Mat blob = cv::dnn::blobFromImage(
        letterboxed, 1.0 / 255.0,
        cv::Size(postprocessor_config_.input_width,
                 postprocessor_config_.input_height),
        cv::Scalar(), swap_rb_, false, CV_32F);
    *preprocess_ms += millisecondsSince(preprocess_start);

    const auto inference_start = std::chrono::steady_clock::now();
    const std::vector<cv::Mat> outputs = inference_backend_->infer(blob);
    *inference_ms += millisecondsSince(inference_start);

    const auto postprocess_start = std::chrono::steady_clock::now();
    std::vector<Detection> detections =
        postprocessor_->decode(outputs, transform, image.size());
    for (Detection &detection : detections) {
      detection.box.x += offset.x;
      detection.box.y += offset.y;
    }
    *postprocess_ms += millisecondsSince(postprocess_start);
    return detections;
  }

  void publishThermalInfo(const sensor_msgs::ImageConstPtr &image,
                          const PreparedImage &prepared) {
    if (!prepared.thermal)
      return;
    sar_yolo_detector::ThermalImageInfo info;
    info.header = image->header;
    info.provenance = identity_;
    info.raw_encoding = image->encoding;
    info.radiometric_input = prepared.radiometric;
    info.temperature_unit = temperature_unit_;
    info.temperature_scale = temperature_scale_;
    info.temperature_offset = temperature_offset_;
    info.normalization_mode = thermal_normalization_;
    info.lower_raw_value = prepared.lower_raw;
    info.upper_raw_value = prepared.upper_raw;
    info.bad_pixel_correction_applied = prepared.bad_pixel_corrected;
    info.clahe_applied = prepared.clahe_applied;
    thermal_info_publisher_.publish(info);
  }

  cv::Size selectedTileSize(const cv::Size &source_size,
                            bool *metric_available) {
    *metric_available = true;
    if (!tiling_enabled_)
      return source_size;
    double scale = 1.0;
    if (tiling_policy_ != "always") {
      std::lock_guard<std::mutex> lock(metrics_mutex_);
      const ros::WallTime now = ros::WallTime::now();
      if (tiling_policy_ == "height") {
        const bool valid = !last_height_wall_.isZero() &&
            (now - last_height_wall_).toSec() <= tiling_metric_timeout_sec_;
        *metric_available = valid;
        if (!valid || latest_height_m_ < tiling_min_height_m_)
          return source_size;
        scale = tiling_min_height_m_ > 0.0
                    ? tiling_min_height_m_ / latest_height_m_
                    : 1.0;
      } else {
        const bool valid = !last_gsd_wall_.isZero() &&
            (now - last_gsd_wall_).toSec() <= tiling_metric_timeout_sec_;
        *metric_available = valid;
        if (!valid || latest_gsd_m_per_px_ < tiling_min_gsd_m_per_px_)
          return source_size;
        const double desired_width =
            target_inference_gsd_m_per_px_ *
            postprocessor_config_.input_width / latest_gsd_m_per_px_;
        scale = desired_width / tile_width_px_;
      }
    }
    const int width = std::min(
        source_size.width,
        std::max(128, static_cast<int>(std::round(tile_width_px_ * scale))));
    const int height = std::min(
        source_size.height,
        std::max(128, static_cast<int>(std::round(tile_height_px_ * scale))));
    return cv::Size(width, height);
  }

  void processImage(const sensor_msgs::ImageConstPtr &message) {
    try {
      const auto total_start = std::chrono::steady_clock::now();
      const PreparedImage prepared = prepareImage(message);
      const cv::Mat &source = prepared.bgr;
      double preprocess_ms = 0.0;
      double inference_ms = 0.0;
      double postprocess_ms = 0.0;
      int inference_region_count = 0;
      std::vector<Detection> detections;
      bool tiling_metric_available = true;
      const cv::Size selected_tile_size =
          selectedTileSize(source.size(), &tiling_metric_available);
      const bool use_tiles = tiling_enabled_ &&
          (source.cols > selected_tile_size.width ||
           source.rows > selected_tile_size.height);
      if (!use_tiles || tile_include_full_frame_) {
        std::vector<Detection> full = inferRegion(
            source, cv::Point(0, 0), &preprocess_ms, &inference_ms,
            &postprocess_ms);
        detections.insert(detections.end(), full.begin(), full.end());
        ++inference_region_count;
      }
      if (use_tiles) {
        const int tile_width = selected_tile_size.width;
        const int tile_height = selected_tile_size.height;
        const std::vector<int> xs =
            tileOrigins(source.cols, tile_width, tile_overlap_ratio_);
        const std::vector<int> ys =
            tileOrigins(source.rows, tile_height, tile_overlap_ratio_);
        for (const int y : ys) {
          for (const int x : xs) {
            const cv::Rect tile(x, y, tile_width, tile_height);
            std::vector<Detection> local = inferRegion(
                source(tile), tile.tl(), &preprocess_ms, &inference_ms,
                &postprocess_ms);
            detections.insert(detections.end(), local.begin(), local.end());
            ++inference_region_count;
          }
        }
        globallyMergeDetections(&detections, tile_merge_overlap_,
                                postprocessor_config_.maximum_detections);
      }
      cv::Rect roi;
      double roi_inference_ms = 0.0;
      bool roi_recheck_used = false;
      if (roiForImage(message, source.size(), &roi)) {
        const cv::Mat roi_image = source(roi);
        double roi_preprocess_ms = 0.0;
        double roi_postprocess_ms = 0.0;
        const auto roi_inference_start = std::chrono::steady_clock::now();
        std::vector<Detection> roi_detections = inferRegion(
            roi_image, roi.tl(), &roi_preprocess_ms, &roi_inference_ms,
            &roi_postprocess_ms);
        roi_inference_ms = millisecondsSince(roi_inference_start);
        mergeRoiDetections(&detections, std::move(roi_detections),
                           roi_recheck_merge_iou_);
        roi_recheck_used = true;
      }

      // The capture timestamp is the control-time reference. A frame that was
      // fresh when queued but became stale while waiting/inferencing must not
      // be published into Tracker's bounded OOSM history.
      const ros::Time publish_time = ros::Time::now();
      if (maximum_capture_age_sec_ > 0.0 && !message->header.stamp.isZero() &&
          !publish_time.isZero() && publish_time > message->header.stamp &&
          (publish_time - message->header.stamp).toSec() >
              maximum_capture_age_sec_) {
        rejectFrame("frame became stale before detection publication");
        return;
      }

      publishDetections(message, detections);
      publishThermalInfo(message, prepared);
      if (publish_debug_image_ && debug_publisher_.getNumSubscribers() > 0) {
        publishDebugImage(message, source, detections);
      }
      const double total_ms = millisecondsSince(total_start);
      double capture_to_publish_ms = -1.0;
      const ros::Time now = publish_time;
      if (!message->header.stamp.isZero() && !now.isZero()) {
        capture_to_publish_ms =
            std::max(0.0, 1000.0 * (now - message->header.stamp).toSec());
      }
      {
        std::lock_guard<std::mutex> lock(metrics_mutex_);
        ++published_frames_;
        last_detection_count_ = detections.size();
        last_preprocess_ms_ = preprocess_ms;
        last_inference_ms_ = inference_ms;
        last_postprocess_ms_ = postprocess_ms;
        last_total_ms_ = total_ms;
        last_capture_to_publish_ms_ = capture_to_publish_ms;
        last_roi_recheck_used_ = roi_recheck_used;
        last_roi_inference_ms_ = roi_inference_ms;
        last_inference_region_count_ = inference_region_count;
        last_tiling_metric_available_ = tiling_metric_available;
        last_tile_width_px_ = selected_tile_size.width;
        last_tile_height_px_ = selected_tile_size.height;
        last_input_encoding_ = message->encoding;
        last_thermal_lower_raw_ = prepared.lower_raw;
        last_thermal_upper_raw_ = prepared.upper_raw;
        if (roi_recheck_used)
          ++roi_recheck_inferences_;
        last_publish_wall_time_ = ros::WallTime::now();
        last_error_.clear();
      }
    } catch (const cv::Exception &exception) {
      inferenceFailure(exception.what());
    } catch (const std::exception &exception) {
      inferenceFailure(exception.what());
    }
  }

  void inferenceFailure(const std::string &reason) {
    std::lock_guard<std::mutex> lock(metrics_mutex_);
    ++inference_failures_;
    last_error_ = reason;
    ROS_ERROR_STREAM_THROTTLE(2.0,
                              "[SarYoloDetector] Inference failed: " << reason);
  }

  void publishDetections(const sensor_msgs::ImageConstPtr &image,
                         const std::vector<Detection> &detections) {
    vision_msgs::Detection2DArray output;
    output.header = image->header;
    output.detections.reserve(detections.size());
    for (const Detection &detection : detections) {
      vision_msgs::Detection2D message;
      message.header = image->header;
      message.bbox.center.x = detection.box.x + 0.5 * detection.box.width;
      message.bbox.center.y = detection.box.y + 0.5 * detection.box.height;
      message.bbox.center.theta = 0.0;
      message.bbox.size_x = detection.box.width;
      message.bbox.size_y = detection.box.height;
      vision_msgs::ObjectHypothesisWithPose hypothesis;
      hypothesis.id = detection.class_id;
      hypothesis.score = detection.confidence;
      message.results.push_back(hypothesis);
      output.detections.push_back(message);
    }
    detections_publisher_.publish(output);
  }

  void publishDebugImage(const sensor_msgs::ImageConstPtr &image,
                         const cv::Mat &source,
                         const std::vector<Detection> &detections) {
    cv::Mat annotated = source.clone();
    for (const Detection &detection : detections) {
      const cv::Scalar color = colorForClass(detection.class_id);
      cv::rectangle(annotated, detection.box, color, 2, cv::LINE_AA);
      std::ostringstream label;
      if (detection.class_id >= 0 &&
          detection.class_id < static_cast<int>(class_names_.size())) {
        label << class_names_[detection.class_id];
      } else {
        label << "class_" << detection.class_id;
      }
      label << ' ' << std::fixed << std::setprecision(2)
            << detection.confidence;
      const int text_y = std::max(15, detection.box.y - 5);
      cv::putText(annotated, label.str(), cv::Point(detection.box.x, text_y),
                  cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv::LINE_AA);
    }
    debug_publisher_.publish(
        cv_bridge::CvImage(image->header, sensor_msgs::image_encodings::BGR8,
                           annotated)
            .toImageMsg());
  }

  void diagnosticTimerCallback(const ros::TimerEvent &) {
    diagnostics_.update();
  }

  void diagnosticCallback(diagnostic_updater::DiagnosticStatusWrapper &status) {
    std::lock_guard<std::mutex> lock(metrics_mutex_);
    const double input_age =
        last_input_wall_time_.isZero()
            ? -1.0
            : (ros::WallTime::now() - last_input_wall_time_).toSec();
    if (inference_failures_ > 0 && !last_error_.empty()) {
      status.summary(diagnostic_msgs::DiagnosticStatus::ERROR,
                     "YOLO inference failure: " + last_error_);
    } else if (received_frames_ == 0) {
      status.summary(diagnostic_msgs::DiagnosticStatus::WARN,
                     "Waiting for camera images");
    } else if (input_age > stale_input_warning_sec_) {
      status.summary(diagnostic_msgs::DiagnosticStatus::WARN,
                     "Camera stream is stale");
    } else {
      status.summary(diagnostic_msgs::DiagnosticStatus::OK,
                     "YOLO detector running");
    }
    status.add("model_path", model_path_);
    status.add("model_version", model_version_);
    status.add("model_sha256", model_sha256_);
    status.add("model_hash_required", require_model_sha256_);
    status.add("rescue_profile", rescue_profile_);
    status.add("detector_name", detector_name_);
    status.add("image_source", image_source_);
    status.add("sensor_id", sensor_id_);
    status.add("inference_backend", inference_backend_->name());
    status.add("inference_backend_details", inference_backend_->details());
    status.add("tensorrt_available", tensorRtBackendAvailable());
    status.add("tensorrt_availability", tensorRtBackendAvailabilityMessage());
    status.add("input_topic", input_image_topic_);
    status.add("input_mode", input_mode_);
    status.add("last_input_encoding", last_input_encoding_);
    status.add("detections_topic", detections_topic_);
    status.add("received_frames", static_cast<long long>(received_frames_));
    status.add("published_frames", static_cast<long long>(published_frames_));
    status.add("dropped_pending_frames",
               static_cast<long long>(dropped_frames_));
    status.add("rejected_frames", static_cast<long long>(rejected_frames_));
    status.add("inference_failures",
               static_cast<long long>(inference_failures_));
    status.add("last_detection_count", static_cast<int>(last_detection_count_));
    status.add("preprocess_ms", last_preprocess_ms_);
    status.add("inference_ms", last_inference_ms_);
    status.add("postprocess_ms", last_postprocess_ms_);
    status.add("detector_total_ms", last_total_ms_);
    status.add("capture_to_publish_ms", last_capture_to_publish_ms_);
    status.add("roi_recheck_enabled", roi_recheck_enabled_);
    status.add("roi_recheck_requests",
               static_cast<long long>(roi_recheck_requests_));
    status.add("roi_recheck_rejected_requests",
               static_cast<long long>(roi_recheck_rejected_requests_));
    status.add("roi_recheck_inferences",
               static_cast<long long>(roi_recheck_inferences_));
    status.add("last_roi_recheck_used", last_roi_recheck_used_);
    status.add("last_roi_inference_ms", last_roi_inference_ms_);
    status.add("tiling_enabled", tiling_enabled_);
    status.add("tiling_policy", tiling_policy_);
    status.add("tiling_metric_available", last_tiling_metric_available_);
    status.add("selected_tile_width_px", last_tile_width_px_);
    status.add("selected_tile_height_px", last_tile_height_px_);
    status.add("latest_height_m", latest_height_m_);
    status.add("latest_gsd_m_per_px", latest_gsd_m_per_px_);
    status.add("last_inference_region_count", last_inference_region_count_);
    status.add("thermal_normalization", thermal_normalization_);
    status.add("last_thermal_lower_raw", last_thermal_lower_raw_);
    status.add("last_thermal_upper_raw", last_thermal_upper_raw_);
    status.add("input_age_sec", input_age);
    status.add("last_error", last_error_);
  }

  ros::NodeHandle node_;
  ros::NodeHandle private_node_;
  image_transport::ImageTransport image_transport_;
  image_transport::Subscriber image_subscriber_;
  ros::Subscriber roi_recheck_subscriber_;
  ros::Subscriber height_subscriber_, gsd_subscriber_;
  image_transport::Publisher debug_publisher_;
  ros::Publisher detections_publisher_;
  ros::Publisher vision_info_publisher_;
  ros::Publisher thermal_info_publisher_;
  diagnostic_updater::Updater diagnostics_;
  ros::Timer diagnostic_timer_;

  std::string model_path_;
  std::string model_version_;
  std::string model_sha256_;
  std::string rescue_profile_;
  std::string input_image_topic_;
  std::string detections_topic_;
  std::string vision_info_topic_;
  std::string debug_image_topic_;
  std::string thermal_info_topic_;
  std::string image_source_;
  std::string sensor_id_;
  std::string detector_name_;
  std::string inference_backend_name_;
  std::string legacy_dnn_backend_;
  std::string opencv_target_;
  std::string engine_path_;
  std::string network_config_path_;
  std::string model_layout_name_;
  std::string input_mode_;
  std::string thermal_normalization_;
  std::string temperature_unit_;
  std::string tiling_policy_, height_topic_, gsd_topic_;
  std::vector<std::string> class_names_;
  std::vector<TestDetection> test_detections_;
  PostprocessorConfig postprocessor_config_;
  std::unique_ptr<YoloPostprocessor> postprocessor_;
  std::unique_ptr<InferenceBackend> inference_backend_;
  PerceptionIdentity identity_;

  bool publish_debug_image_{false};
  bool require_model_sha256_{false};
  bool require_capture_timestamp_{true};
  bool reject_out_of_order_{true};
  bool swap_rb_{true};
  bool darknet_coordinates_normalized_{true};
  bool roi_recheck_enabled_{false};
  bool thermal_bad_pixel_correction_{false};
  bool thermal_clahe_enabled_{false};
  bool tiling_enabled_{false};
  bool tile_include_full_frame_{true};
  int database_version_{1};
  int tile_width_px_{640};
  int tile_height_px_{640};
  double future_timestamp_tolerance_sec_{0.02};
  double maximum_capture_age_sec_{0.50};
  double stale_input_warning_sec_{1.0};
  double roi_recheck_interval_sec_{0.4};
  double roi_recheck_context_scale_{1.8};
  double roi_recheck_max_capture_delta_sec_{0.35};
  double roi_recheck_merge_iou_{0.55};
  double thermal_lower_percentile_{1.0};
  double thermal_upper_percentile_{99.0};
  double thermal_fixed_min_{0.0};
  double thermal_fixed_max_{65535.0};
  double thermal_clahe_clip_limit_{2.0};
  double temperature_scale_{1.0};
  double temperature_offset_{0.0};
  double tile_overlap_ratio_{0.20};
  double tile_merge_overlap_{0.45};
  double tiling_min_height_m_{40.0};
  double tiling_min_gsd_m_per_px_{0.05};
  double target_inference_gsd_m_per_px_{0.03};
  double tiling_metric_timeout_sec_{1.0};
  std::string roi_recheck_topic_{"tracker_node/roi_recheck_request"};

  std::mutex queue_mutex_;
  std::condition_variable queue_condition_;
  sensor_msgs::ImageConstPtr pending_image_;
  ros::Time last_received_stamp_;
  bool stop_requested_{false};
  std::thread worker_;

  std::mutex roi_mutex_;
  cv::Rect2d roi_request_box_;
  ros::Time roi_request_stamp_;
  std::string roi_request_frame_id_;
  ros::WallTime last_roi_recheck_wall_;
  bool have_roi_request_{false};

  std::mutex metrics_mutex_;
  std::uint64_t received_frames_{0};
  std::uint64_t published_frames_{0};
  std::uint64_t dropped_frames_{0};
  std::uint64_t rejected_frames_{0};
  std::uint64_t inference_failures_{0};
  std::size_t last_detection_count_{0};
  double last_preprocess_ms_{0.0};
  double last_inference_ms_{0.0};
  double last_postprocess_ms_{0.0};
  double last_total_ms_{0.0};
  double last_capture_to_publish_ms_{-1.0};
  bool last_roi_recheck_used_{false};
  double last_roi_inference_ms_{0.0};
  int last_inference_region_count_{0};
  int last_tile_width_px_{0}, last_tile_height_px_{0};
  bool last_tiling_metric_available_{true};
  double latest_height_m_{-1.0}, latest_gsd_m_per_px_{-1.0};
  ros::WallTime last_height_wall_, last_gsd_wall_;
  double last_thermal_lower_raw_{0.0};
  double last_thermal_upper_raw_{0.0};
  std::uint64_t roi_recheck_requests_{0};
  std::uint64_t roi_recheck_rejected_requests_{0};
  std::uint64_t roi_recheck_inferences_{0};
  ros::WallTime last_input_wall_time_;
  ros::WallTime last_publish_wall_time_;
  std::string last_error_;
  std::string last_input_encoding_;
};

} // namespace sar_yolo_detector

int main(int argc, char **argv) {
  ros::init(argc, argv, "sar_yolo_detector");
  try {
    sar_yolo_detector::SarYoloDetectorNode node;
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::waitForShutdown();
  } catch (const std::exception &exception) {
    std::cerr << "[SarYoloDetector] Startup failed: " << exception.what()
              << std::endl;
    ROS_FATAL_STREAM("[SarYoloDetector] Startup failed: " << exception.what());
    return 1;
  }
  return 0;
}
