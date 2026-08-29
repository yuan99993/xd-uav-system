#pragma once

#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace sar_yolo_detector {

enum class ModelLayout {
  kAuto,
  kYoloV5,
  kUltralytics,
};

struct LetterboxTransform {
  double scale{1.0};
  int pad_x{0};
  int pad_y{0};
  int input_width{0};
  int input_height{0};
};

struct Detection {
  int class_id{-1};
  float confidence{0.0F};
  cv::Rect box;
};

struct PostprocessorConfig {
  int input_width{640};
  int input_height{640};
  float confidence_threshold{0.25F};
  float nms_threshold{0.45F};
  float minimum_box_size_px{2.0F};
  int maximum_detections{300};
  bool class_agnostic_nms{false};
  ModelLayout model_layout{ModelLayout::kAuto};
  std::vector<std::string> class_names;
  std::vector<int> allowed_class_ids;
};

cv::Mat letterboxImage(const cv::Mat &image, int input_width, int input_height,
                       LetterboxTransform *transform);

ModelLayout parseModelLayout(const std::string &value);

class YoloPostprocessor {
public:
  explicit YoloPostprocessor(PostprocessorConfig config);

  std::vector<Detection> decode(const std::vector<cv::Mat> &outputs,
                                const LetterboxTransform &transform,
                                const cv::Size &original_size) const;

private:
  cv::Mat predictionRows(const cv::Mat &output) const;
  bool classAllowed(int class_id) const;

  PostprocessorConfig config_;
};

} // namespace sar_yolo_detector
