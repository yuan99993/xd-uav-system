#include <sar_yolo_detector/yolo_postprocessor.hpp>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <map>
#include <stdexcept>
#include <utility>

#include <opencv2/dnn/dnn.hpp>
#include <opencv2/imgproc.hpp>

namespace sar_yolo_detector {
namespace {

std::string lower(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(),
                 [](const unsigned char character) {
                   return static_cast<char>(std::tolower(character));
                 });
  return value;
}

bool finiteRow(const float *row, const int count) {
  for (int index = 0; index < count; ++index) {
    if (!std::isfinite(row[index]))
      return false;
  }
  return true;
}

cv::Rect clippedBox(const float center_x, const float center_y,
                    const float width, const float height,
                    const LetterboxTransform &transform,
                    const cv::Size &original_size) {
  const double scale = std::max(1e-9, transform.scale);
  const double left = (center_x - 0.5 * width - transform.pad_x) / scale;
  const double top = (center_y - 0.5 * height - transform.pad_y) / scale;
  const double right = (center_x + 0.5 * width - transform.pad_x) / scale;
  const double bottom = (center_y + 0.5 * height - transform.pad_y) / scale;

  const int x1 = std::max(
      0, std::min(original_size.width, static_cast<int>(std::floor(left))));
  const int y1 = std::max(
      0, std::min(original_size.height, static_cast<int>(std::floor(top))));
  const int x2 = std::max(
      0, std::min(original_size.width, static_cast<int>(std::ceil(right))));
  const int y2 = std::max(
      0, std::min(original_size.height, static_cast<int>(std::ceil(bottom))));
  return cv::Rect(x1, y1, std::max(0, x2 - x1), std::max(0, y2 - y1));
}

} // namespace

cv::Mat letterboxImage(const cv::Mat &image, const int input_width,
                       const int input_height,
                       LetterboxTransform *const transform) {
  if (image.empty())
    throw std::invalid_argument("Cannot letterbox an empty image");
  if (input_width <= 0 || input_height <= 0) {
    throw std::invalid_argument("YOLO input dimensions must be positive");
  }
  const double scale = std::min(static_cast<double>(input_width) / image.cols,
                                static_cast<double>(input_height) / image.rows);
  const int resized_width =
      std::max(1, static_cast<int>(std::round(image.cols * scale)));
  const int resized_height =
      std::max(1, static_cast<int>(std::round(image.rows * scale)));
  const int horizontal_padding = input_width - resized_width;
  const int vertical_padding = input_height - resized_height;
  const int left = horizontal_padding / 2;
  const int top = vertical_padding / 2;

  cv::Mat resized;
  cv::resize(image, resized, cv::Size(resized_width, resized_height), 0.0, 0.0,
             cv::INTER_LINEAR);
  cv::Mat output(input_height, input_width, image.type(),
                 cv::Scalar(114, 114, 114));
  resized.copyTo(output(cv::Rect(left, top, resized_width, resized_height)));
  if (transform != nullptr) {
    transform->scale = scale;
    transform->pad_x = left;
    transform->pad_y = top;
    transform->input_width = input_width;
    transform->input_height = input_height;
  }
  return output;
}

ModelLayout parseModelLayout(const std::string &value) {
  const std::string normalized = lower(value);
  if (normalized.empty() || normalized == "auto")
    return ModelLayout::kAuto;
  if (normalized == "yolov5" || normalized == "v5")
    return ModelLayout::kYoloV5;
  if (normalized == "ultralytics" || normalized == "yolov8" ||
      normalized == "yolo11") {
    return ModelLayout::kUltralytics;
  }
  throw std::invalid_argument("Unknown model_layout: " + value);
}

YoloPostprocessor::YoloPostprocessor(PostprocessorConfig config)
    : config_(std::move(config)) {
  if (config_.class_names.empty()) {
    throw std::invalid_argument(
        "class_names must match the exported YOLO model");
  }
  if (config_.confidence_threshold < 0.0F ||
      config_.confidence_threshold > 1.0F || config_.nms_threshold < 0.0F ||
      config_.nms_threshold > 1.0F) {
    throw std::invalid_argument(
        "Confidence and NMS thresholds must be in [0, 1]");
  }
  config_.maximum_detections = std::max(1, config_.maximum_detections);
  config_.minimum_box_size_px = std::max(0.0F, config_.minimum_box_size_px);
  std::sort(config_.allowed_class_ids.begin(), config_.allowed_class_ids.end());
  config_.allowed_class_ids.erase(std::unique(config_.allowed_class_ids.begin(),
                                              config_.allowed_class_ids.end()),
                                  config_.allowed_class_ids.end());
}

cv::Mat YoloPostprocessor::predictionRows(const cv::Mat &output) const {
  if (output.empty())
    return cv::Mat();
  cv::Mat float_output;
  if (output.type() == CV_32F) {
    float_output = output;
  } else {
    output.convertTo(float_output, CV_32F);
  }
  if (!float_output.isContinuous())
    float_output = float_output.clone();

  const int class_count = static_cast<int>(config_.class_names.size());
  const int v5_features = class_count + 5;
  const int ultralytics_features = class_count + 4;
  if (float_output.dims == 2)
    return float_output.clone();

  if (float_output.dims == 3 && float_output.size[0] == 1) {
    const int first = float_output.size[1];
    const int second = float_output.size[2];
    cv::Mat view(first, second, CV_32F, float_output.ptr<float>());
    if (first == v5_features || first == ultralytics_features) {
      return view.t();
    }
    if (second == v5_features || second == ultralytics_features) {
      return view.clone();
    }
  }

  const int last_dimension = float_output.size[float_output.dims - 1];
  if (last_dimension == v5_features || last_dimension == ultralytics_features) {
    const int row_count =
        static_cast<int>(float_output.total() / last_dimension);
    return cv::Mat(row_count, last_dimension, CV_32F, float_output.ptr<float>())
        .clone();
  }
  throw std::runtime_error("Unsupported YOLO output layout; export a raw "
                           "detect ONNX model without embedded NMS");
}

bool YoloPostprocessor::classAllowed(const int class_id) const {
  return config_.allowed_class_ids.empty() ||
         std::binary_search(config_.allowed_class_ids.begin(),
                            config_.allowed_class_ids.end(), class_id);
}

std::vector<Detection>
YoloPostprocessor::decode(const std::vector<cv::Mat> &outputs,
                          const LetterboxTransform &transform,
                          const cv::Size &original_size) const {
  if (original_size.width <= 0 || original_size.height <= 0)
    return {};
  std::vector<cv::Mat> row_blocks;
  row_blocks.reserve(outputs.size());
  for (const cv::Mat &output : outputs) {
    cv::Mat rows = predictionRows(output);
    if (!rows.empty())
      row_blocks.push_back(rows);
  }
  if (row_blocks.empty())
    return {};
  cv::Mat predictions;
  cv::vconcat(row_blocks, predictions);

  const int class_count = static_cast<int>(config_.class_names.size());
  const int v5_features = class_count + 5;
  const int ultralytics_features = class_count + 4;
  ModelLayout layout = config_.model_layout;
  if (layout == ModelLayout::kAuto) {
    if (predictions.cols == v5_features)
      layout = ModelLayout::kYoloV5;
    else if (predictions.cols == ultralytics_features) {
      layout = ModelLayout::kUltralytics;
    } else {
      throw std::runtime_error(
          "YOLO output class count does not match class_names");
    }
  }
  const int class_offset = layout == ModelLayout::kYoloV5 ? 5 : 4;
  const int expected_features = class_offset + class_count;
  if (predictions.cols != expected_features) {
    throw std::runtime_error(
        "Configured model_layout does not match ONNX output");
  }

  std::vector<cv::Rect> boxes;
  std::vector<float> confidences;
  std::vector<int> class_ids;
  boxes.reserve(predictions.rows);
  confidences.reserve(predictions.rows);
  class_ids.reserve(predictions.rows);
  for (int row_index = 0; row_index < predictions.rows; ++row_index) {
    const float *const row = predictions.ptr<float>(row_index);
    if (!finiteRow(row, expected_features))
      continue;
    const float objectness = layout == ModelLayout::kYoloV5 ? row[4] : 1.0F;
    if (objectness <= 0.0F)
      continue;
    int best_class = -1;
    float best_class_score = 0.0F;
    for (int class_id = 0; class_id < class_count; ++class_id) {
      const float score = row[class_offset + class_id];
      if (score > best_class_score) {
        best_class_score = score;
        best_class = class_id;
      }
    }
    const float confidence = objectness * best_class_score;
    if (best_class < 0 || confidence < config_.confidence_threshold ||
        !classAllowed(best_class)) {
      continue;
    }
    const cv::Rect box =
        clippedBox(row[0], row[1], row[2], row[3], transform, original_size);
    if (box.width < config_.minimum_box_size_px ||
        box.height < config_.minimum_box_size_px) {
      continue;
    }
    boxes.push_back(box);
    confidences.push_back(confidence);
    class_ids.push_back(best_class);
  }

  std::vector<int> kept_indices;
  if (config_.class_agnostic_nms) {
    cv::dnn::NMSBoxes(boxes, confidences, config_.confidence_threshold,
                      config_.nms_threshold, kept_indices);
  } else {
    std::map<int, std::vector<int>> indices_by_class;
    for (std::size_t index = 0; index < class_ids.size(); ++index) {
      indices_by_class[class_ids[index]].push_back(static_cast<int>(index));
    }
    for (const auto &entry : indices_by_class) {
      std::vector<cv::Rect> class_boxes;
      std::vector<float> class_confidences;
      class_boxes.reserve(entry.second.size());
      class_confidences.reserve(entry.second.size());
      for (const int index : entry.second) {
        class_boxes.push_back(boxes[index]);
        class_confidences.push_back(confidences[index]);
      }
      std::vector<int> class_kept;
      cv::dnn::NMSBoxes(class_boxes, class_confidences,
                        config_.confidence_threshold, config_.nms_threshold,
                        class_kept);
      for (const int local_index : class_kept) {
        kept_indices.push_back(entry.second[local_index]);
      }
    }
  }

  std::sort(kept_indices.begin(), kept_indices.end(),
            [&confidences](const int left, const int right) {
              if (confidences[left] == confidences[right])
                return left < right;
              return confidences[left] > confidences[right];
            });
  if (static_cast<int>(kept_indices.size()) > config_.maximum_detections) {
    kept_indices.resize(config_.maximum_detections);
  }
  std::vector<Detection> detections;
  detections.reserve(kept_indices.size());
  for (const int index : kept_indices) {
    detections.push_back({class_ids[index], confidences[index], boxes[index]});
  }
  return detections;
}

} // namespace sar_yolo_detector
