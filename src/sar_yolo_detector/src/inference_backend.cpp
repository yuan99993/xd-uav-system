#include <sar_yolo_detector/inference_backend.hpp>

#include <algorithm>
#include <atomic>
#include <cctype>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <memory>
#include <stdexcept>
#include <string>
#include <sstream>
#include <utility>

#include <opencv2/core/version.hpp>
#include <opencv2/dnn/dnn.hpp>

namespace sar_yolo_detector {

std::unique_ptr<InferenceBackend>
createTensorRtBackend(const InferenceBackendConfig &config);

namespace {

std::string lower(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(),
                 [](const unsigned char character) {
                   return static_cast<char>(std::tolower(character));
                 });
  return value;
}

bool readableFile(const std::string &path) {
  return !path.empty() && std::ifstream(path).good();
}

class OpenCvDnnBackend final : public InferenceBackend {
public:
  explicit OpenCvDnnBackend(const InferenceBackendConfig &config)
      : target_(config.opencv_target) {
    if (!readableFile(config.model_path)) {
      throw std::invalid_argument(
          "OpenCV DNN backend requires a readable ONNX model_path");
    }
    network_ = cv::dnn::readNetFromONNX(config.model_path);
    if (network_.empty()) {
      throw std::runtime_error("OpenCV DNN returned an empty network");
    }
    if (target_ == "opencv_cpu") {
      network_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
      network_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
    } else if (target_ == "opencv_opencl") {
      network_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
      network_.setPreferableTarget(cv::dnn::DNN_TARGET_OPENCL_FP16);
    } else if (target_ == "opencv_cuda") {
#if CV_VERSION_MAJOR >= 4
      network_.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
      network_.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA_FP16);
#else
      throw std::runtime_error("opencv_cuda requires OpenCV 4 with CUDA DNN");
#endif
    } else {
      throw std::invalid_argument("Unsupported opencv_target: " + target_);
    }
    output_layer_names_ = network_.getUnconnectedOutLayersNames();
    if (output_layer_names_.empty()) {
      throw std::runtime_error("ONNX model has no discoverable output layer");
    }
  }

  std::string name() const override { return "opencv_dnn"; }

  std::string details() const override { return target_; }

  std::vector<cv::Mat> infer(const cv::Mat &input_blob) override {
    if (input_blob.empty())
      throw std::invalid_argument("Inference blob is empty");
    network_.setInput(input_blob);
    std::vector<cv::Mat> outputs;
    network_.forward(outputs, output_layer_names_);
    return outputs;
  }

private:
  cv::dnn::Net network_;
  std::vector<std::string> output_layer_names_;
  std::string target_;
};

// OpenCV 4.2 is the system DNN runtime on this ROS Noetic image.  It cannot
// import several current ONNX graphs, but it does reliably support the
// Darknet representation.  Keeping this separate from the ONNX backend makes
// the compatibility choice explicit rather than silently falling back.
class OpenCvDarknetBackend final : public InferenceBackend {
public:
  explicit OpenCvDarknetBackend(const InferenceBackendConfig &config)
      : target_(config.opencv_target),
        coordinates_normalized_(config.darknet_coordinates_normalized),
        class_count_(config.class_count) {
    if (!readableFile(config.network_config_path) ||
        !readableFile(config.model_path)) {
      throw std::invalid_argument(
          "OpenCV Darknet backend requires readable network_config_path and "
          "model_path (.weights)");
    }
    network_ =
        cv::dnn::readNetFromDarknet(config.network_config_path, config.model_path);
    if (network_.empty()) {
      throw std::runtime_error("OpenCV DNN returned an empty Darknet network");
    }
    if (target_ == "opencv_cpu") {
      network_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
      network_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
    } else if (target_ == "opencv_opencl") {
      network_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
      network_.setPreferableTarget(cv::dnn::DNN_TARGET_OPENCL_FP16);
    } else if (target_ == "opencv_cuda") {
#if CV_VERSION_MAJOR >= 4
      network_.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
      network_.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA_FP16);
#else
      throw std::runtime_error("opencv_cuda requires OpenCV 4 with CUDA DNN");
#endif
    } else {
      throw std::invalid_argument("Unsupported opencv_target: " + target_);
    }
    output_layer_names_ = network_.getUnconnectedOutLayersNames();
    if (output_layer_names_.empty()) {
      throw std::runtime_error("Darknet model has no discoverable output layer");
    }
  }

  std::string name() const override { return "opencv_darknet"; }

  std::string details() const override {
    std::ostringstream stream;
    stream << target_
           << (coordinates_normalized_ ? ":normalized_xywh" : ":pixel_xywh")
           << ":last_raw_obj=" << std::scientific << std::setprecision(3)
           << last_max_objectness_.load() << ":last_raw_joint="
           << last_max_joint_score_.load() << ":last_raw_class="
           << last_max_class_score_.load() << ":last_output="
           << last_output_dims_.load() << "D/" << last_output_rows_.load()
           << "x" << last_output_cols_.load() << ":last_raw_abs="
           << last_max_absolute_value_.load();
    return stream.str();
  }

  std::vector<cv::Mat> infer(const cv::Mat &input_blob) override {
    if (input_blob.empty() || input_blob.dims != 4) {
      throw std::invalid_argument("Darknet inference requires a NCHW image blob");
    }
    network_.setInput(input_blob);
    std::vector<cv::Mat> outputs;
    network_.forward(outputs, output_layer_names_);
    if (!coordinates_normalized_) {
      return outputs;
    }
    const float input_width = static_cast<float>(input_blob.size[3]);
    const float input_height = static_cast<float>(input_blob.size[2]);
    float max_objectness = 0.0F;
    float max_joint_score = 0.0F;
    float max_class_score = 0.0F;
    float max_absolute_value = 0.0F;
    for (cv::Mat &output : outputs) {
      last_output_dims_.store(output.dims);
      last_output_rows_.store(output.rows);
      last_output_cols_.store(output.cols);
      if (output.empty() || output.dims != 2 || output.cols < 5) {
        throw std::runtime_error("Unexpected Darknet detection output shape");
      }
      if (output.type() != CV_32F) {
        output.convertTo(output, CV_32F);
      }
      if (!output.isContinuous()) {
        output = output.clone();
      }
      for (int row_index = 0; row_index < output.rows; ++row_index) {
        float *row = output.ptr<float>(row_index);
        for (int column_index = 0; column_index < output.cols; ++column_index) {
          max_absolute_value =
              std::max(max_absolute_value, std::abs(row[column_index]));
        }
        if (class_count_ > 0 && output.cols >= class_count_ + 5) {
          const float objectness = row[4];
          float max_class_probability = 0.0F;
          for (int class_index = 0; class_index < class_count_; ++class_index) {
            max_class_probability =
                std::max(max_class_probability, row[5 + class_index]);
          }
          max_objectness = std::max(max_objectness, objectness);
          max_class_score =
              std::max(max_class_score, max_class_probability);
          max_joint_score =
              std::max(max_joint_score, objectness * max_class_probability);
        }
        row[0] *= input_width;
        row[1] *= input_height;
        row[2] *= input_width;
        row[3] *= input_height;
      }
    }
    last_max_objectness_.store(max_objectness);
    last_max_joint_score_.store(max_joint_score);
    last_max_class_score_.store(max_class_score);
    last_max_absolute_value_.store(max_absolute_value);
    return outputs;
  }

private:
  cv::dnn::Net network_;
  std::vector<std::string> output_layer_names_;
  std::string target_;
  bool coordinates_normalized_{true};
  int class_count_{0};
  std::atomic<float> last_max_objectness_{0.0F};
  std::atomic<float> last_max_joint_score_{0.0F};
  std::atomic<float> last_max_class_score_{0.0F};
  std::atomic<int> last_output_dims_{0};
  std::atomic<int> last_output_rows_{0};
  std::atomic<int> last_output_cols_{0};
  std::atomic<float> last_max_absolute_value_{0.0F};
};

class DeterministicTestBackend final : public InferenceBackend {
public:
  explicit DeterministicTestBackend(const InferenceBackendConfig &config)
      : class_count_(config.class_count), yolov5_layout_(config.yolov5_layout),
        detections_(config.test_detections) {
    if (class_count_ <= 0) {
      throw std::invalid_argument(
          "Test backend requires a positive class_count");
    }
    if (detections_.empty()) {
      TestDetection default_detection;
      default_detection.center_x = 320.0F;
      default_detection.center_y = 320.0F;
      default_detection.width = 140.0F;
      default_detection.height = 220.0F;
      default_detection.confidence = 0.95F;
      default_detection.class_id = 0;
      detections_.push_back(default_detection);
    }
    for (const TestDetection &detection : detections_) {
      if (detection.class_id < 0 || detection.class_id >= class_count_ ||
          detection.width <= 0.0F || detection.height <= 0.0F ||
          detection.confidence < 0.0F || detection.confidence > 1.0F) {
        throw std::invalid_argument("Invalid test_detections entry");
      }
    }
  }

  std::string name() const override { return "test"; }

  std::string details() const override {
    return "deterministic_raw_yolo_output";
  }

  std::vector<cv::Mat> infer(const cv::Mat &input_blob) override {
    if (input_blob.empty())
      throw std::invalid_argument("Inference blob is empty");
    const int feature_count = class_count_ + (yolov5_layout_ ? 5 : 4);
    int dimensions[3] = {1, static_cast<int>(detections_.size()),
                         feature_count};
    cv::Mat output(3, dimensions, CV_32F, cv::Scalar::all(0));
    float *values = output.ptr<float>();
    for (std::size_t index = 0; index < detections_.size(); ++index) {
      const TestDetection &detection = detections_[index];
      float *row = values + index * feature_count;
      row[0] = detection.center_x;
      row[1] = detection.center_y;
      row[2] = detection.width;
      row[3] = detection.height;
      const int class_offset = yolov5_layout_ ? 5 : 4;
      if (yolov5_layout_)
        row[4] = detection.confidence;
      row[class_offset + detection.class_id] =
          yolov5_layout_ ? 1.0F : detection.confidence;
    }
    return {output};
  }

private:
  int class_count_{0};
  bool yolov5_layout_{false};
  std::vector<TestDetection> detections_;
};

} // namespace

std::unique_ptr<InferenceBackend>
createInferenceBackend(const InferenceBackendConfig &config) {
  const std::string backend = lower(config.backend);
  if (backend == "opencv_dnn" || backend == "opencv") {
    return std::unique_ptr<InferenceBackend>(new OpenCvDnnBackend(config));
  }
  if (backend == "opencv_darknet" || backend == "darknet") {
    return std::unique_ptr<InferenceBackend>(new OpenCvDarknetBackend(config));
  }
  if (backend == "test" || backend == "mock") {
    return std::unique_ptr<InferenceBackend>(
        new DeterministicTestBackend(config));
  }
  if (backend == "tensorrt" || backend == "trt") {
    return createTensorRtBackend(config);
  }
  throw std::invalid_argument("Unsupported inference_backend: " +
                              config.backend);
}

} // namespace sar_yolo_detector
