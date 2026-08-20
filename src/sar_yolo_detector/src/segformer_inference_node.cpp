#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <XmlRpcValue.h>
#include <cv_bridge/cv_bridge.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <sar_yolo_detector/inference_backend.hpp>
#include <sar_yolo_detector/model_integrity.hpp>
#include <sar_yolo_detector/runtime_identity.hpp>

namespace sar_yolo_detector {
namespace {

double elapsedMilliseconds(
    const std::chrono::steady_clock::time_point &start) {
  return std::chrono::duration<double, std::milli>(
             std::chrono::steady_clock::now() - start)
      .count();
}

std::vector<double> numberArray(ros::NodeHandle *node, const std::string &name,
                                const std::vector<double> &defaults) {
  XmlRpc::XmlRpcValue value;
  if (!node->getParam(name, value))
    return defaults;
  if (value.getType() != XmlRpc::XmlRpcValue::TypeArray)
    throw std::invalid_argument(name + " must be a numeric array");
  std::vector<double> result;
  result.reserve(value.size());
  for (int index = 0; index < value.size(); ++index) {
    if (value[index].getType() == XmlRpc::XmlRpcValue::TypeInt)
      result.push_back(static_cast<int>(value[index]));
    else if (value[index].getType() == XmlRpc::XmlRpcValue::TypeDouble)
      result.push_back(static_cast<double>(value[index]));
    else
      throw std::invalid_argument(name + " must contain only numbers");
  }
  return result;
}

std::vector<std::string> stringArray(ros::NodeHandle *node,
                                     const std::string &name) {
  XmlRpc::XmlRpcValue value;
  if (!node->getParam(name, value))
    return {};
  if (value.getType() != XmlRpc::XmlRpcValue::TypeArray)
    throw std::invalid_argument(name + " must be a string array");
  std::vector<std::string> result;
  result.reserve(value.size());
  for (int index = 0; index < value.size(); ++index) {
    if (value[index].getType() != XmlRpc::XmlRpcValue::TypeString)
      throw std::invalid_argument(name + " must contain only strings");
    result.push_back(static_cast<std::string>(value[index]));
  }
  return result;
}

} // namespace

class SegformerInferenceNode {
public:
  SegformerInferenceNode()
      : private_node_("~"), image_transport_(node_) {
    loadParameters();
    InferenceBackendConfig backend_config;
    backend_config.backend = inference_backend_;
    backend_config.model_path = model_path_;
    backend_config.engine_path = engine_path_;
    backend_config.opencv_target = opencv_target_;
    backend_config.tensorrt_output_binding_names = output_binding_names_;
    verifyModelArtifact(inference_backend_ == "tensorrt" ? engine_path_
                                                          : model_path_,
                        model_sha256_, require_model_sha256_);
    backend_ = createInferenceBackend(backend_config);

    mask_publisher_ = image_transport_.advertise(output_mask_topic_, 1);
    image_subscriber_ = image_transport_.subscribe(
        input_image_topic_, 1, &SegformerInferenceNode::imageCallback, this,
        image_transport::TransportHints(
            "raw", ros::TransportHints().tcpNoDelay()));
    diagnostics_.setHardwareID("sar_segformer_" + identity_.uav_id + "_" +
                               identity_.sensor_id);
    diagnostics_.add("segformer_inference", this,
                     &SegformerInferenceNode::diagnosticCallback);
    diagnostic_timer_ = node_.createWallTimer(
        ros::WallDuration(1.0),
        &SegformerInferenceNode::diagnosticTimerCallback, this);
    worker_ = std::thread(&SegformerInferenceNode::workerLoop, this);
    ROS_INFO_STREAM("[SarSegFormer] Ready: " << input_image_topic_ << " -> "
                    << output_mask_topic_ << " backend=" << backend_->name()
                    << " input=" << input_width_ << 'x' << input_height_);
  }

  ~SegformerInferenceNode() {
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
    private_node_.param("input_image_topic", input_image_topic_,
                        std::string("camera/eo/image_raw"));
    private_node_.param("output_mask_topic", output_mask_topic_,
                        std::string("sar_yolo_detector/floodnet/class_mask"));
    private_node_.param("inference_backend", inference_backend_,
                        std::string("tensorrt"));
    private_node_.param("engine_path", engine_path_, std::string());
    private_node_.param("model_path", model_path_, std::string());
    private_node_.param("model_sha256", model_sha256_, std::string());
    private_node_.param("require_model_sha256", require_model_sha256_, true);
    private_node_.param("opencv_target", opencv_target_,
                        std::string("opencv_cpu"));
    private_node_.param("input_width", input_width_, 1024);
    private_node_.param("input_height", input_height_, 1024);
    private_node_.param("class_count", class_count_, 10);
    private_node_.param("maximum_capture_age_sec", maximum_capture_age_sec_,
                        1.0);
    private_node_.param("future_timestamp_tolerance_sec",
                        future_timestamp_tolerance_sec_, 0.02);
    private_node_.param("require_capture_timestamp",
                        require_capture_timestamp_, true);
    private_node_.param("reject_out_of_order", reject_out_of_order_, true);
    image_mean_ = numberArray(&private_node_, "image_mean",
                              {0.485, 0.456, 0.406});
    image_std_ = numberArray(&private_node_, "image_std",
                             {0.229, 0.224, 0.225});
    output_binding_names_ =
        stringArray(&private_node_, "tensorrt_output_binding_names");
    if (input_width_ <= 0 || input_height_ <= 0 || class_count_ <= 1)
      throw std::invalid_argument("SegFormer dimensions/classes are invalid");
    if (image_mean_.size() != 3U || image_std_.size() != 3U ||
        std::any_of(image_std_.begin(), image_std_.end(),
                    [](const double value) { return value <= 0.0; })) {
      throw std::invalid_argument("image_mean/image_std must contain 3 channels");
    }
    maximum_capture_age_sec_ = std::max(0.0, maximum_capture_age_sec_);
    future_timestamp_tolerance_sec_ =
        std::max(0.0, future_timestamp_tolerance_sec_);
    identity_ = loadPerceptionIdentity(&private_node_, "floodnet_eo_primary",
                                       "floodnet_segformer_b0", "CAMERA_RAY");
  }

  bool timestampAccepted(const sensor_msgs::Image &message) {
    if (message.header.stamp.isZero()) {
      if (!require_capture_timestamp_)
        return true;
      reject("missing capture timestamp");
      return false;
    }
    const ros::Time now = ros::Time::now();
    if (!now.isZero()) {
      const double age = (now - message.header.stamp).toSec();
      if (age < -future_timestamp_tolerance_sec_) {
        reject("capture timestamp is in the future");
        return false;
      }
      if (maximum_capture_age_sec_ > 0.0 && age > maximum_capture_age_sec_) {
        reject("capture timestamp is stale");
        return false;
      }
    }
    std::lock_guard<std::mutex> lock(queue_mutex_);
    if (reject_out_of_order_ && !last_received_stamp_.isZero() &&
        message.header.stamp <= last_received_stamp_) {
      rejectLocked("out-of-order capture timestamp");
      return false;
    }
    last_received_stamp_ = message.header.stamp;
    return true;
  }

  void imageCallback(const sensor_msgs::ImageConstPtr &message) {
    {
      std::lock_guard<std::mutex> lock(metrics_mutex_);
      ++received_frames_;
      last_input_wall_ = ros::WallTime::now();
    }
    if (message->data.empty() || message->width == 0U || message->height == 0U) {
      reject("empty image");
      return;
    }
    if (!timestampAccepted(*message))
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

  void reject(const std::string &reason) {
    std::lock_guard<std::mutex> lock(metrics_mutex_);
    ++rejected_frames_;
    last_error_ = reason;
    ROS_WARN_STREAM_THROTTLE(2.0, "[SarSegFormer] Rejected frame: " << reason);
  }

  void rejectLocked(const std::string &reason) { reject(reason); }

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
      process(message);
    }
  }

  cv::Mat makeInputBlob(const cv::Mat &bgr) const {
    cv::Mat resized, rgb, float_rgb;
    cv::resize(bgr, resized, cv::Size(input_width_, input_height_), 0.0, 0.0,
               cv::INTER_LINEAR);
    cv::cvtColor(resized, rgb, cv::COLOR_BGR2RGB);
    rgb.convertTo(float_rgb, CV_32FC3, 1.0 / 255.0);
    int dimensions[4] = {1, 3, input_height_, input_width_};
    cv::Mat blob(4, dimensions, CV_32F);
    std::vector<cv::Mat> channels;
    cv::split(float_rgb, channels);
    const std::size_t plane =
        static_cast<std::size_t>(input_width_) * input_height_;
    float *destination = blob.ptr<float>();
    for (int channel = 0; channel < 3; ++channel) {
      cv::Mat normalized =
          (channels[channel] - image_mean_[channel]) / image_std_[channel];
      std::memcpy(destination + static_cast<std::size_t>(channel) * plane,
                  normalized.ptr<float>(), plane * sizeof(float));
    }
    return blob;
  }

  cv::Mat decodeMask(const std::vector<cv::Mat> &outputs,
                     const cv::Size &source_size) const {
    if (outputs.empty())
      throw std::runtime_error("SegFormer produced no output tensor");
    const cv::Mat *logits = nullptr;
    for (const cv::Mat &candidate : outputs) {
      if (!candidate.empty() && candidate.dims == 4 && candidate.size[0] == 1 &&
          candidate.size[1] == class_count_) {
        logits = &candidate;
        break;
      }
    }
    if (logits == nullptr)
      throw std::runtime_error(
          "SegFormer output must be NCHW [1,class_count,height,width]");
    const int height = logits->size[2];
    const int width = logits->size[3];
    const std::size_t plane = static_cast<std::size_t>(height) * width;
    const float *values = logits->ptr<float>();
    cv::Mat native_mask(height, width, CV_8UC1);
    for (std::size_t pixel = 0; pixel < plane; ++pixel) {
      int best_class = 0;
      float best_score = values[pixel];
      if (!std::isfinite(best_score))
        throw std::runtime_error("SegFormer output contains NaN/Inf");
      for (int class_id = 1; class_id < class_count_; ++class_id) {
        const float score = values[static_cast<std::size_t>(class_id) * plane +
                                   pixel];
        if (!std::isfinite(score))
          throw std::runtime_error("SegFormer output contains NaN/Inf");
        if (score > best_score) {
          best_score = score;
          best_class = class_id;
        }
      }
      native_mask.data[pixel] = static_cast<std::uint8_t>(best_class);
    }
    cv::Mat mask;
    cv::resize(native_mask, mask, source_size, 0.0, 0.0, cv::INTER_NEAREST);
    return mask;
  }

  void process(const sensor_msgs::ImageConstPtr &message) {
    try {
      const auto total_start = std::chrono::steady_clock::now();
      const cv_bridge::CvImageConstPtr image =
          cv_bridge::toCvShare(message, sensor_msgs::image_encodings::BGR8);
      const auto preprocess_start = std::chrono::steady_clock::now();
      const cv::Mat blob = makeInputBlob(image->image);
      const double preprocess_ms = elapsedMilliseconds(preprocess_start);
      const auto inference_start = std::chrono::steady_clock::now();
      const std::vector<cv::Mat> outputs = backend_->infer(blob);
      const double inference_ms = elapsedMilliseconds(inference_start);
      const auto postprocess_start = std::chrono::steady_clock::now();
      const cv::Mat mask = decodeMask(outputs, image->image.size());
      const double postprocess_ms = elapsedMilliseconds(postprocess_start);

      const ros::Time now = ros::Time::now();
      if (maximum_capture_age_sec_ > 0.0 && !now.isZero() &&
          !message->header.stamp.isZero() && now > message->header.stamp &&
          (now - message->header.stamp).toSec() > maximum_capture_age_sec_) {
        reject("frame became stale before mask publication");
        return;
      }
      mask_publisher_.publish(
          cv_bridge::CvImage(message->header,
                             sensor_msgs::image_encodings::MONO8, mask)
              .toImageMsg());
      std::lock_guard<std::mutex> lock(metrics_mutex_);
      ++published_frames_;
      last_preprocess_ms_ = preprocess_ms;
      last_inference_ms_ = inference_ms;
      last_postprocess_ms_ = postprocess_ms;
      last_total_ms_ = elapsedMilliseconds(total_start);
      last_output_wall_ = ros::WallTime::now();
      last_error_.clear();
    } catch (const std::exception &exception) {
      std::lock_guard<std::mutex> lock(metrics_mutex_);
      ++inference_failures_;
      last_error_ = exception.what();
      ROS_ERROR_STREAM_THROTTLE(2.0,
                                "[SarSegFormer] Inference failed: "
                                    << exception.what());
    }
  }

  void diagnosticTimerCallback(const ros::WallTimerEvent &) {
    diagnostics_.update();
  }

  void diagnosticCallback(
      diagnostic_updater::DiagnosticStatusWrapper &status) {
    std::lock_guard<std::mutex> lock(metrics_mutex_);
    const double input_age =
        last_input_wall_.isZero()
            ? -1.0
            : (ros::WallTime::now() - last_input_wall_).toSec();
    const double output_age =
        last_output_wall_.isZero()
            ? -1.0
            : (ros::WallTime::now() - last_output_wall_).toSec();
    if (!last_error_.empty() && inference_failures_ > 0U) {
      status.summary(diagnostic_msgs::DiagnosticStatus::ERROR,
                     "SegFormer inference failure: " + last_error_);
    } else if (received_frames_ == 0U) {
      status.summary(diagnostic_msgs::DiagnosticStatus::WARN,
                     "Waiting for EO images");
    } else if (output_age < 0.0 || output_age > 2.0) {
      status.summary(diagnostic_msgs::DiagnosticStatus::WARN,
                     "SegFormer output is stale");
    } else {
      status.summary(diagnostic_msgs::DiagnosticStatus::OK,
                     "SegFormer inference running");
    }
    status.add("mission_id", identity_.mission_id);
    status.add("uav_id", identity_.uav_id);
    status.add("session_uuid", identity_.session_uuid);
    status.add("profile", identity_.profile);
    status.add("model_version", identity_.model_version);
    status.add("model_sha256", identity_.model_sha256);
    status.add("model_hash_required", require_model_sha256_);
    status.add("backend", backend_->name());
    status.add("backend_details", backend_->details());
    status.add("received_frames", static_cast<long long>(received_frames_));
    status.add("published_frames", static_cast<long long>(published_frames_));
    status.add("dropped_frames", static_cast<long long>(dropped_frames_));
    status.add("rejected_frames", static_cast<long long>(rejected_frames_));
    status.add("inference_failures",
               static_cast<long long>(inference_failures_));
    status.add("preprocess_ms", last_preprocess_ms_);
    status.add("inference_ms", last_inference_ms_);
    status.add("postprocess_ms", last_postprocess_ms_);
    status.add("total_ms", last_total_ms_);
    status.add("input_age_sec", input_age);
    status.add("output_age_sec", output_age);
    status.add("last_error", last_error_);
  }

  ros::NodeHandle node_, private_node_;
  image_transport::ImageTransport image_transport_;
  image_transport::Subscriber image_subscriber_;
  image_transport::Publisher mask_publisher_;
  diagnostic_updater::Updater diagnostics_;
  ros::WallTimer diagnostic_timer_;
  std::unique_ptr<InferenceBackend> backend_;
  PerceptionIdentity identity_;
  std::string input_image_topic_, output_mask_topic_, inference_backend_;
  std::string engine_path_, model_path_, model_sha256_, opencv_target_;
  std::vector<std::string> output_binding_names_;
  std::vector<double> image_mean_, image_std_;
  int input_width_{1024}, input_height_{1024}, class_count_{10};
  double maximum_capture_age_sec_{1.0};
  double future_timestamp_tolerance_sec_{0.02};
  bool require_capture_timestamp_{true}, reject_out_of_order_{true};
  bool require_model_sha256_{true};

  std::mutex queue_mutex_, metrics_mutex_;
  std::condition_variable queue_condition_;
  sensor_msgs::ImageConstPtr pending_image_;
  ros::Time last_received_stamp_;
  bool stop_requested_{false};
  std::thread worker_;
  std::uint64_t received_frames_{0}, published_frames_{0}, dropped_frames_{0};
  std::uint64_t rejected_frames_{0}, inference_failures_{0};
  ros::WallTime last_input_wall_, last_output_wall_;
  double last_preprocess_ms_{0.0}, last_inference_ms_{0.0};
  double last_postprocess_ms_{0.0}, last_total_ms_{0.0};
  std::string last_error_;
};

} // namespace sar_yolo_detector

int main(int argc, char **argv) {
  ros::init(argc, argv, "sar_segformer_inference");
  try {
    sar_yolo_detector::SegformerInferenceNode node;
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::waitForShutdown();
  } catch (const std::exception &exception) {
    std::cerr << "[SarSegFormer] Startup failed: " << exception.what()
              << std::endl;
    ROS_FATAL_STREAM("[SarSegFormer] Startup failed: " << exception.what());
    return 1;
  }
  return 0;
}
