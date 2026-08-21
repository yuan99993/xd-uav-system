#pragma once

#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace sar_yolo_detector {

struct TestDetection {
  float center_x{0.0F};
  float center_y{0.0F};
  float width{0.0F};
  float height{0.0F};
  float confidence{0.0F};
  int class_id{-1};
};

struct InferenceBackendConfig {
  std::string backend{"opencv_dnn"};
  std::string model_path;
  std::string network_config_path;
  std::string engine_path;
  std::string opencv_target{"opencv_cpu"};
  bool darknet_coordinates_normalized{true};
  int class_count{0};
  bool yolov5_layout{false};
  std::vector<std::string> tensorrt_output_binding_names;
  std::vector<TestDetection> test_detections;
};

class InferenceBackend {
public:
  virtual ~InferenceBackend() = default;

  virtual std::string name() const = 0;
  virtual std::string details() const = 0;
  virtual std::vector<cv::Mat> infer(const cv::Mat &input_blob) = 0;
};

std::unique_ptr<InferenceBackend>
createInferenceBackend(const InferenceBackendConfig &config);

bool tensorRtBackendAvailable();
std::string tensorRtBackendAvailabilityMessage();

} // namespace sar_yolo_detector
