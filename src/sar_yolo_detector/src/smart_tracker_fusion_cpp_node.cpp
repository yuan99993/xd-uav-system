#include <sar_yolo_detector/FusionDetection2D.h>
#include <sar_yolo_detector/FusionDetection2DArray.h>
#include <sar_yolo_detector/GetSharedModelInfo.h>
#include <sar_yolo_detector/NormalizedDetection.h>
#include <sar_yolo_detector/PerceptionIdentity.h>
#include <sar_yolo_detector/RunSharedInference.h>
#include <sar_yolo_detector/SelectFusionPoint.h>
#include <sar_yolo_detector/SelectFusionROI.h>
#include <sar_yolo_detector/SelectSmartTrack.h>
#include <sar_yolo_detector/SmartTrackerState.h>
#include <sar_yolo_detector/SwitchSmartTrackerModel.h>
#include <sar_yolo_detector/SwitchTrackingMode.h>
#include <sar_yolo_detector/TrackedDetection2D.h>
#include <sar_yolo_detector/TrackedDetection2DArray.h>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/RegionOfInterest.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_srvs/SetBool.h>
#include <vision_msgs/Detection2D.h>
#include <vision_msgs/ObjectHypothesisWithPose.h>
#include <xd_uav_track/DetectionArray.h>
#include <xd_uav_track/GimbalState.h>
#include <xd_uav_track/TrackStatus.h>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/video/tracking.hpp>
#include <ros/ros.h>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <numeric>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>
#include <boost/function.hpp>

namespace {

using SteadyClock = std::chrono::steady_clock;

struct Box {
  double x{0.0};
  double y{0.0};
  double w{0.0};
  double h{0.0};
};

struct Detection {
  int64_t id{-1};
  int class_id{-1};
  double confidence{0.0};
  Box box;
  bool stable{false};
  std::string geometry{"aabb"};
  std::array<double, 5> obb{};
  bool has_rotation{false};
  double rotation_deg{0.0};
  double flow_quality{0.0};
  double flow_scale_ratio{1.0};
  std::vector<float> polygon;
  double detector_stamp{0.0};
  std::string source{"detector"};
  bool selected_identity_reacquire{false};
};

struct TrackResult {
  Box box;
  int64_t public_id{-1};
  int64_t local_id{-1};
  int class_id{-1};
  double confidence{0.0};
  double detector_stamp{0.0};
  std::string source{"idle"};
  std::string reason{"no_selection"};
  bool observed{false};
  bool has_box{false};
};

struct Snapshot {
  sensor_msgs::ImageConstPtr image;
  cv::Mat frame;
  std::vector<Detection> detections;
  Box selected_box;
  bool has_selected_box;
};

double clamp01(double value) {
  return std::max(0.0, std::min(1.0, value));
}

bool finiteBox(const Box &box) {
  return std::isfinite(box.x) && std::isfinite(box.y) &&
         std::isfinite(box.w) && std::isfinite(box.h);
}

bool validBox(const Box &box, const cv::Size &size) {
  return finiteBox(box) && box.w >= 4.0 && box.h >= 4.0 && box.x >= 0.0 &&
         box.y >= 0.0 && box.x + box.w <= size.width + 0.01 &&
         box.y + box.h <= size.height + 0.01;
}

bool boxToPixelRoi(const Box &box, const cv::Size &size, cv::Rect *roi) {
  if (roi == nullptr || size.width <= 0 || size.height <= 0 ||
      !validBox(box, size)) return false;

  const double width = static_cast<double>(size.width);
  const double height = static_cast<double>(size.height);
  const double left_edge = std::max(0.0, std::min(width, box.x));
  const double top_edge = std::max(0.0, std::min(height, box.y));
  const double right_edge = std::max(0.0, std::min(width, box.x + box.w));
  const double bottom_edge = std::max(0.0, std::min(height, box.y + box.h));

  const int left = static_cast<int>(std::floor(left_edge));
  const int top = static_cast<int>(std::floor(top_edge));
  const int right = static_cast<int>(std::ceil(right_edge));
  const int bottom = static_cast<int>(std::ceil(bottom_edge));
  if (right <= left || bottom <= top) return false;

  *roi = cv::Rect(left, top, right - left, bottom - top);
  return true;
}

double boxIou(const Box &a, const Box &b) {
  const double left = std::max(a.x, b.x);
  const double top = std::max(a.y, b.y);
  const double right = std::min(a.x + a.w, b.x + b.w);
  const double bottom = std::min(a.y + a.h, b.y + b.h);
  const double area = std::max(0.0, right - left) * std::max(0.0, bottom - top);
  return area / std::max(1.0, a.w * a.h + b.w * b.h - area);
}

bool duplicateTargetBoxes(const Box &a, int class_a, const Box &b, int class_b) {
  const double iou = boxIou(a, b);
  if (iou >= 0.72) return true;
  if (class_a != class_b) return false;

  const double left = std::max(a.x, b.x);
  const double top = std::max(a.y, b.y);
  const double right = std::min(a.x + a.w, b.x + b.w);
  const double bottom = std::min(a.y + a.h, b.y + b.h);
  const double intersection = std::max(0.0, right - left) *
      std::max(0.0, bottom - top);
  const double smaller_area = std::min(a.w * a.h, b.w * b.h);
  const double contained_overlap = intersection / std::max(1.0, smaller_area);
  const double scale = std::max(8.0,
      std::hypot(std::max(a.w, b.w), std::max(a.h, b.h)));
  const cv::Point2d a_center(a.x + a.w * 0.5, a.y + a.h * 0.5);
  const cv::Point2d b_center(b.x + b.w * 0.5, b.y + b.h * 0.5);
  const double center_distance = cv::norm(a_center - b_center) / scale;
  if (contained_overlap >= 0.75 && center_distance <= 0.22) return true;

  const double area_ratio = a.w * a.h / std::max(1.0, b.w * b.h);
  return area_ratio >= 0.80 && area_ratio <= 1.25 && center_distance <= 0.15;
}

cv::Point2d center(const Box &box) {
  return {box.x + box.w * 0.5, box.y + box.h * 0.5};
}

Box clipBox(const Box &box, const cv::Size &size) {
  const double left = std::max(0.0, box.x);
  const double top = std::max(0.0, box.y);
  const double right = std::min(static_cast<double>(size.width), box.x + box.w);
  const double bottom = std::min(static_cast<double>(size.height), box.y + box.h);
  return {left, top, std::max(0.0, right - left), std::max(0.0, bottom - top)};
}

Box transformBox(const Box &box, const cv::Mat &matrix) {
  if (matrix.empty()) return box;
  std::vector<cv::Point2f> corners{{static_cast<float>(box.x), static_cast<float>(box.y)},
      {static_cast<float>(box.x + box.w), static_cast<float>(box.y)},
      {static_cast<float>(box.x + box.w), static_cast<float>(box.y + box.h)},
      {static_cast<float>(box.x), static_cast<float>(box.y + box.h)}};
  std::vector<cv::Point2f> projected;
  cv::perspectiveTransform(corners, projected, matrix);
  if (projected.size() != 4) return {};
  float left = projected[0].x, right = left, top = projected[0].y, bottom = top;
  for (const auto &point : projected) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y)) return {};
    left = std::min(left, point.x); right = std::max(right, point.x);
    top = std::min(top, point.y); bottom = std::max(bottom, point.y);
  }
  return {left, top, right - left, bottom - top};
}

std::string lower(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(),
                 [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
  return value;
}

bool hasConfiguredColor(const std::string &value) {
  const std::string color = lower(value);
  return color == "red" || color == "green" || color == "blue" ||
      color == "white" || color == "black";
}

std::string jsonEscape(const std::string &value) {
  std::ostringstream escaped;
  for (unsigned char ch : value) {
    switch (ch) {
      case '"': escaped << "\\\""; break;
      case '\\': escaped << "\\\\"; break;
      case '\b': escaped << "\\b"; break;
      case '\f': escaped << "\\f"; break;
      case '\n': escaped << "\\n"; break;
      case '\r': escaped << "\\r"; break;
      case '\t': escaped << "\\t"; break;
      default:
        if (ch < 0x20) {
          const char hex[] = "0123456789abcdef";
          escaped << "\\u00" << hex[(ch >> 4) & 0x0f] << hex[ch & 0x0f];
        } else {
          escaped << static_cast<char>(ch);
        }
    }
  }
  return escaped.str();
}

cv::Mat imageToBgr(const sensor_msgs::Image &message) {
  if (message.width == 0 || message.height == 0) {
    throw std::runtime_error("image dimensions must be positive");
  }
  const std::string encoding = lower(message.encoding);
  int channels = 0;
  int conversion = -1;
  if (encoding == "bgr8") channels = 3;
  else if (encoding == "rgb8") { channels = 3; conversion = cv::COLOR_RGB2BGR; }
  else if (encoding == "bgra8") { channels = 4; conversion = cv::COLOR_BGRA2BGR; }
  else if (encoding == "rgba8") { channels = 4; conversion = cv::COLOR_RGBA2BGR; }
  else if (encoding == "mono8" || encoding == "8uc1") {
    channels = 1;
    conversion = cv::COLOR_GRAY2BGR;
  }
  if (channels > 0) {
    const size_t minimum_step = static_cast<size_t>(message.width) * channels;
    const size_t step = message.step;
    if (step < minimum_step || message.data.size() < step * message.height) {
      throw std::runtime_error("image buffer is smaller than its encoding and dimensions");
    }
    cv::Mat rows(static_cast<int>(message.height), static_cast<int>(step), CV_8UC1,
                 const_cast<uint8_t *>(message.data.data()));
    cv::Mat packed = rows.colRange(0, static_cast<int>(minimum_step));
    cv::Mat decoded(static_cast<int>(message.height), static_cast<int>(message.width),
                    CV_MAKETYPE(CV_8U, channels), packed.data,
                    static_cast<size_t>(packed.step));
    if (conversion < 0) return decoded;
    cv::Mat bgr;
    cv::cvtColor(decoded, bgr, conversion);
    return bgr;
  }
  if (encoding == "mono16" || encoding == "16uc1" || encoding == "y16") {
    const size_t minimum_step = static_cast<size_t>(message.width) * 2;
    if (message.step < minimum_step || message.step % 2 != 0 ||
        message.data.size() < static_cast<size_t>(message.step) * message.height) {
      throw std::runtime_error("invalid 16-bit image buffer");
    }
    const int type = CV_16UC1;
    cv::Mat gray16(static_cast<int>(message.height), static_cast<int>(message.width), type);
    for (uint32_t y = 0; y < message.height; ++y) {
      const uint8_t *row = message.data.data() + static_cast<size_t>(y) * message.step;
      uint16_t *dst = gray16.ptr<uint16_t>(static_cast<int>(y));
      for (uint32_t x = 0; x < message.width; ++x) {
        const uint8_t a = row[2 * x], b = row[2 * x + 1];
        dst[x] = message.is_bigendian
                     ? static_cast<uint16_t>((a << 8) | b)
                     : static_cast<uint16_t>((b << 8) | a);
      }
    }
    cv::Mat gray8;
    cv::normalize(gray16, gray8, 0, 255, cv::NORM_MINMAX, CV_8U);
    cv::Mat bgr;
    cv::cvtColor(gray8, bgr, cv::COLOR_GRAY2BGR);
    return bgr;
  }
  throw std::runtime_error("unsupported sensor_msgs/Image encoding: " + encoding);
}

sensor_msgs::Image toRosImage(const cv::Mat &frame, const std_msgs::Header &header) {
  if (frame.empty() || frame.type() != CV_8UC3) {
    throw std::runtime_error("annotated frame must be 8-bit BGR");
  }
  cv::Mat packed = frame.isContinuous() ? frame : frame.clone();
  sensor_msgs::Image message;
  message.header = header;
  message.height = static_cast<uint32_t>(packed.rows);
  message.width = static_cast<uint32_t>(packed.cols);
  message.encoding = "bgr8";
  message.is_bigendian = 0;
  message.step = static_cast<uint32_t>(packed.cols * 3);
  message.data.assign(packed.datastart, packed.dataend);
  return message;
}

std::vector<float> appearanceFeature(const cv::Mat &frame, const Box &box,
                                    double *quality) {
  *quality = 0.0;
  cv::Rect roi;
  if (!boxToPixelRoi(box, frame.size(), &roi)) return {};
  cv::Mat small, gray, hsv, mask, hist;
  cv::resize(frame(roi), small, cv::Size(48, 48));
  cv::cvtColor(small, gray, cv::COLOR_BGR2GRAY);
  cv::Scalar mean, deviation;
  cv::meanStdDev(gray, mean, deviation);
  *quality = std::min(1.0, deviation[0] / 18.0) *
             std::min(1.0, std::min(box.w, box.h) / 24.0);
  cv::cvtColor(small, hsv, cv::COLOR_BGR2HSV);
  cv::inRange(hsv, cv::Scalar(0, 40, 20), cv::Scalar(179, 255, 255), mask);
  if (cv::countNonZero(mask) < std::max(16, static_cast<int>(0.10 * mask.total()))) {
    mask.release();
  }
  int channels[] = {0, 1};
  int sizes[] = {16, 8};
  float hue[] = {0.0f, 180.0f};
  float saturation[] = {0.0f, 256.0f};
  const float *ranges[] = {hue, saturation};
  cv::calcHist(&hsv, 1, channels, mask, hist, 2, sizes, ranges, true, false);
  const double norm = cv::norm(hist, cv::NORM_L2);
  if (norm > 1e-9) hist /= norm;
  std::vector<float> result(hist.total());
  std::copy(hist.ptr<float>(), hist.ptr<float>() + hist.total(), result.begin());
  return result;
}

cv::Mat grayTemplate(const cv::Mat &frame, const Box &box) {
  cv::Rect roi;
  if (!boxToPixelRoi(box, frame.size(), &roi)) return {};
  cv::Mat gray;
  cv::cvtColor(frame(roi), gray, cv::COLOR_BGR2GRAY);
  return gray.clone();
}

bool matchGrayTemplate(const cv::Mat &frame_gray, const cv::Mat &reference,
                       const Box &predicted, double absence_sec, bool global_search,
                       bool multi_scale,
                       Box *matched_box, double *matched_score) {
  if (frame_gray.empty() || reference.empty() || !matched_box || !matched_score) return false;

  const double coordinate_scale = global_search && std::max(reference.cols, reference.rows) >= 48
      ? 0.5 : 1.0;
  cv::Mat search_image;
  if (coordinate_scale < 1.0) {
    cv::resize(frame_gray, search_image, cv::Size(), coordinate_scale, coordinate_scale,
               cv::INTER_AREA);
  } else {
    search_image = frame_gray;
  }
  cv::Mat search_template;
  if (coordinate_scale < 1.0) {
    cv::resize(reference, search_template, cv::Size(), coordinate_scale, coordinate_scale,
               cv::INTER_AREA);
  } else {
    search_template = reference;
  }
  if (search_template.cols < 4 || search_template.rows < 4) return false;

  cv::Rect search_roi(0, 0, search_image.cols, search_image.rows);
  if (!global_search) {
    const double expansion = 0.5 + std::min(2.0, std::max(0.0, absence_sec) * 0.6);
    const double margin_x = std::max(24.0, predicted.w * expansion);
    const double margin_y = std::max(24.0, predicted.h * expansion);
    const double scale = coordinate_scale;
    const int left = std::max(0, static_cast<int>(std::floor((predicted.x + predicted.w * 0.5 - margin_x) * scale)));
    const int top = std::max(0, static_cast<int>(std::floor((predicted.y + predicted.h * 0.5 - margin_y) * scale)));
    const int right = std::min(search_image.cols,
        static_cast<int>(std::ceil((predicted.x + predicted.w * 0.5 + margin_x) * scale)));
    const int bottom = std::min(search_image.rows,
        static_cast<int>(std::ceil((predicted.y + predicted.h * 0.5 + margin_y) * scale)));
    if (right <= left || bottom <= top) return false;
    search_roi = cv::Rect(left, top, right - left, bottom - top);
  }
  if (search_roi.width < search_template.cols || search_roi.height < search_template.rows) return false;

  const cv::Mat search = search_image(search_roi);
  const std::array<double, 5> scales{{0.60, 0.78, 1.0, 1.28, 1.65}};
  double best_score = -1.0;
  Box best_box;
  for (size_t i = 0; i < scales.size(); ++i) {
    if (!multi_scale && i != 2) continue;
    const double factor = scales[i];
    const int width = std::max(6, static_cast<int>(std::lround(search_template.cols * factor)));
    const int height = std::max(6, static_cast<int>(std::lround(search_template.rows * factor)));
    if (width > search.cols || height > search.rows) continue;
    cv::Mat scaled_template;
    cv::resize(search_template, scaled_template, cv::Size(width, height), 0.0, 0.0,
               factor < 1.0 ? cv::INTER_AREA : cv::INTER_LINEAR);
    cv::Scalar template_mean, template_stddev;
    cv::meanStdDev(scaled_template, template_mean, template_stddev);
    const bool low_texture = template_stddev[0] < 2.0;
    const int method = low_texture ? cv::TM_SQDIFF_NORMED : cv::TM_CCOEFF_NORMED;
    cv::Mat response;
    cv::matchTemplate(search, scaled_template, response, method);
    double score = -1.0, minimum = 0.0;
    cv::Point location, minimum_location;
    cv::minMaxLoc(response, &minimum, &score, &minimum_location, &location);
    if (low_texture) {
      score = 1.0 - minimum;
      location = minimum_location;
    }
    if (!std::isfinite(score) || score <= best_score) continue;
    best_score = score;
    best_box = {(search_roi.x + location.x) / coordinate_scale,
                (search_roi.y + location.y) / coordinate_scale,
                width / coordinate_scale, height / coordinate_scale};
  }
  if (best_score < 0.0) return false;
  *matched_box = best_box;
  *matched_score = best_score;
  return true;
}

double cosineSimilarity(const std::vector<float> &a, const std::vector<float> &b) {
  if (a.size() != b.size() || a.empty()) return 0.0;
  return std::inner_product(a.begin(), a.end(), b.begin(), 0.0);
}

bool validateHomography(const cv::Mat &matrix, const cv::Size &size) {
  if (matrix.rows != 3 || matrix.cols != 3 || !cv::checkRange(matrix)) return false;
  std::vector<cv::Point2f> points{{0, 0}, {static_cast<float>(size.width), 0},
      {static_cast<float>(size.width), static_cast<float>(size.height)},
      {0, static_cast<float>(size.height)},
      {static_cast<float>(size.width) / 2, static_cast<float>(size.height) / 2}};
  std::vector<cv::Point2f> projected;
  for (const auto &point : points) {
    const double denominator = matrix.at<double>(2, 0) * point.x +
        matrix.at<double>(2, 1) * point.y + matrix.at<double>(2, 2);
    if (!std::isfinite(denominator) || denominator <= 0.2) return false;
  }
  cv::perspectiveTransform(points, projected, matrix);
  for (size_t i = 0; i < points.size(); ++i) {
    if (!std::isfinite(projected[i].x) || !std::isfinite(projected[i].y)) return false;
    if (i < 4 && cv::norm(projected[i] - points[i]) >
        0.3 * std::hypot(size.width, size.height)) return false;
  }
  std::vector<cv::Point> polygon;
  for (size_t i = 0; i < 4; ++i) polygon.emplace_back(projected[i]);
  const double area = std::abs(cv::contourArea(polygon)) /
                      std::max(1.0, static_cast<double>(size.area()));
  return area >= 0.65 && area <= 1.5;
}

cv::Mat rotationFromQuaternion(const geometry_msgs::Quaternion &input) {
  const double norm = std::sqrt(input.x * input.x + input.y * input.y +
                                input.z * input.z + input.w * input.w);
  if (!std::isfinite(norm) || norm < 0.9 || norm > 1.1) return {};
  const double x = input.x / norm, y = input.y / norm;
  const double z = input.z / norm, w = input.w / norm;
  return (cv::Mat_<double>(3, 3) <<
      1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w),
      2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w),
      2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y));
}

class CameraMotion {
 public:
  explicit CameraMotion(const YAML::Node &config) {
    enabled_ = config["enabled"] ? config["enabled"].as<bool>() : true;
    max_features_ = config["max_features"] ? config["max_features"].as<int>() : 160;
    min_inliers_ = config["minimum_inliers"] ? config["minimum_inliers"].as<int>() : 12;
    max_gap_ = config["max_gap_sec"] ? config["max_gap_sec"].as<double>() : 0.75;
  }

  void reset() { gray_.release(); stamp_ = 0.0; history_.clear(); source_ = "unavailable"; }
  bool enabled() const { return enabled_; }
  const std::string &source() const { return source_; }

  cv::Mat update(const cv::Mat &frame, double stamp, const std::vector<Box> &boxes,
                 const cv::Mat &pose = cv::Mat()) {
    cv::Mat transform;
    source_ = "unavailable";
    if (!enabled_ || boxes.empty()) {
      gray_.release();
      stamp_ = stamp;
      history_.clear();
      return transform;
    }
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    const double previous_stamp = stamp_;
    if (!gray_.empty() && gray_.size() == gray.size() && stamp > previous_stamp &&
        stamp - previous_stamp <= max_gap_) {
      cv::Mat background = estimate(gray_, gray, boxes, pose);
      if (!background.empty()) {
        transform = background;
        source_ = pose.empty() ? "background" : "pose_seeded_background";
      } else if (!pose.empty() && validateHomography(pose, gray.size())) {
        transform = pose;
        source_ = "pose_rotation_only";
      }
      history_.push_back({previous_stamp, stamp, transform.clone()});
      while (history_.size() > 32) history_.pop_front();
    } else {
      history_.clear();
    }
    gray_ = gray;
    stamp_ = stamp;
    return transform;
  }

  cv::Mat between(double first, double last) const {
    if (first == last) return cv::Mat::eye(3, 3, CV_64F);
    cv::Mat result = cv::Mat::eye(3, 3, CV_64F);
    double cursor = first;
    for (const auto &item : history_) {
      if (std::abs(item.first - cursor) < 1e-6) {
        if (item.matrix.empty()) return {};
        result = item.matrix * result;
        cursor = item.second;
        if (std::abs(cursor - last) < 1e-6) return result;
      }
    }
    return {};
  }

 private:
  cv::Mat estimate(const cv::Mat &previous, const cv::Mat &current,
                   const std::vector<Box> &boxes, const cv::Mat &pose) const {
    const double scale = std::min(1.0, 320.0 / previous.cols);
    cv::Mat old_small, new_small;
    cv::resize(previous, old_small, cv::Size(), scale, scale, cv::INTER_AREA);
    cv::resize(current, new_small, old_small.size(), 0.0, 0.0, cv::INTER_AREA);
    cv::Mat mask(old_small.size(), CV_8UC1, cv::Scalar(255));
    for (const Box &box : boxes) {
      const int margin = std::max(6, static_cast<int>(0.15 * std::max(box.w, box.h)));
      cv::rectangle(mask,
          cv::Point(std::max(0, static_cast<int>((box.x - margin) * scale)),
                    std::max(0, static_cast<int>((box.y - margin) * scale))),
          cv::Point(std::min(mask.cols - 1, static_cast<int>((box.x + box.w + margin) * scale)),
                    std::min(mask.rows - 1, static_cast<int>((box.y + box.h + margin) * scale))),
          cv::Scalar(0), cv::FILLED);
    }
    cv::Mat initial;
    if (!pose.empty() && validateHomography(pose, previous.size())) {
      const cv::Mat scaling = (cv::Mat_<double>(3, 3) << scale, 0, 0, 0, scale, 0, 0, 0, 1);
      const cv::Mat small_pose = scaling * pose * scaling.inv();
      std::vector<cv::Point2f> points, moved, back;
      cv::goodFeaturesToTrack(old_small, points, max_features_, 0.02, 8.0, mask);
      if (static_cast<int>(points.size()) >= min_inliers_) {
        std::vector<cv::Point2f> projected;
        cv::perspectiveTransform(points, projected, small_pose);
        initial = cv::Mat(projected).reshape(2, static_cast<int>(projected.size()));
        // Continue below with the same points; the pose is only an LK seed.
        return estimateWithPoints(previous, old_small, new_small, points, boxes, scale, small_pose);
      }
    }
    std::vector<cv::Point2f> points, moved, back;
    cv::goodFeaturesToTrack(old_small, points, max_features_, 0.02, 8.0, mask);
    if (static_cast<int>(points.size()) < min_inliers_) return {};
    return estimateWithPoints(previous, old_small, new_small, points, boxes, scale, cv::Mat());
  }

  cv::Mat estimateWithPoints(const cv::Mat &previous, const cv::Mat &old_small,
      const cv::Mat &new_small, const std::vector<cv::Point2f> &points,
      const std::vector<Box> &, double scale, const cv::Mat &initial_transform) const {
    std::vector<cv::Point2f> moved, back, initial_points;
    if (!initial_transform.empty()) cv::perspectiveTransform(points, initial_points, initial_transform);
    std::vector<uint8_t> forward_status, backward_status;
    std::vector<float> errors;
    cv::calcOpticalFlowPyrLK(old_small, new_small, points, moved, forward_status, errors,
        cv::Size(21, 21), 3, cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 30, 0.01),
        initial_points.empty() ? 0 : cv::OPTFLOW_USE_INITIAL_FLOW, 1e-4);
    if (moved.empty()) return {};
    cv::calcOpticalFlowPyrLK(new_small, old_small, moved, back, backward_status, errors,
                             cv::Size(21, 21), 3);
    std::vector<cv::Point2f> before, after;
    for (size_t i = 0; i < points.size(); ++i) {
      if (i >= backward_status.size() || !forward_status[i] || !backward_status[i] ||
          cv::norm(back[i] - points[i]) > 1.5 ||
          !std::isfinite(moved[i].x) || !std::isfinite(moved[i].y)) continue;
      before.push_back(points[i]);
      after.push_back(moved[i]);
    }
    if (static_cast<int>(before.size()) < min_inliers_) return {};
    cv::Mat inliers, affine = cv::estimateAffinePartial2D(before, after, inliers,
        cv::RANSAC, 2.0, 500, 0.99);
    if (affine.empty() || inliers.empty()) return {};
    int count = 0;
    std::vector<cv::Point2f> inlier_points;
    for (int i = 0; i < inliers.rows; ++i) count += inliers.at<uint8_t>(i, 0) != 0;
    if (count < min_inliers_ || count / static_cast<double>(before.size()) < 0.65) return {};
    for (int i = 0; i < inliers.rows; ++i) {
      if (inliers.at<uint8_t>(i, 0)) inlier_points.push_back(before[static_cast<size_t>(i)]);
    }
    if (inlier_points.size() < static_cast<size_t>(min_inliers_)) return {};
    double min_x = inlier_points.front().x, max_x = min_x;
    double min_y = inlier_points.front().y, max_y = min_y;
    for (const auto &point : inlier_points) {
      min_x = std::min(min_x, static_cast<double>(point.x));
      max_x = std::max(max_x, static_cast<double>(point.x));
      min_y = std::min(min_y, static_cast<double>(point.y));
      max_y = std::max(max_y, static_cast<double>(point.y));
    }
    if (max_x - min_x < old_small.cols * 0.25 || max_y - min_y < old_small.rows * 0.25) return {};
    cv::Mat homography = cv::Mat::eye(3, 3, CV_64F);
    affine.copyTo(homography(cv::Rect(0, 0, 3, 2)));
    const cv::Mat scaling = (cv::Mat_<double>(3, 3) << scale, 0, 0, 0, scale, 0, 0, 0, 1);
    const cv::Mat inverse = scaling.inv();
    homography = inverse * homography * scaling;
    return validateHomography(homography, previous.size()) ? homography : cv::Mat();
  }

  struct MotionStep { double first; double second; cv::Mat matrix; };
  bool enabled_{true};
  int max_features_{160};
  int min_inliers_{12};
  double max_gap_{0.75};
  cv::Mat gray_;
  double stamp_{0.0};
  std::deque<MotionStep> history_;
  std::string source_{"unavailable"};
};

class TemplateDetections {
 public:
  void reset() { tracks_.clear(); used_track_ids_.clear(); }

  std::vector<Detection> update(const cv::Mat &frame, double stamp,
      const std::vector<Detection> &detections, bool detector_updated,
      const cv::Mat &camera_transform, double track_memory_sec,
      double coast_hold_sec, double match_threshold, double update_confidence,
      double template_refresh_sec,
      double appearance_gate,
      double quality_gate, double max_area_ratio, double multi_scale_period_sec,
      int64_t selected_track_id) {
    used_track_ids_.clear();
    if (frame.empty()) return {};
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

    pruneDuplicateTracks(selected_track_id);
    std::unordered_map<int64_t, Detection> fresh_by_id;
    if (detector_updated) {
      for (const auto &detection : detections) {
        if (!detection.stable || detection.id < 0 || !validBox(detection.box, frame.size())) continue;
        int64_t track_id = detection.id;
        auto found = tracks_.find(track_id);
        if (found == tracks_.end()) {
          double best_score = -std::numeric_limits<double>::infinity();
          int64_t best_id = -1;
          double detection_quality = 0.0;
          const auto detection_feature = appearanceFeature(frame, detection.box, &detection_quality);
          const bool small_detection = std::max(detection.box.w, detection.box.h) < 20.0;
          for (const auto &entry : tracks_) {
            if (entry.first == track_id || used_track_ids_.count(entry.first)) continue;
            const bool class_changed =
                entry.second.detection.class_id != detection.class_id;
            const Box &old_box = entry.second.box;
            const double overlap = boxIou(old_box, detection.box);
            const double area_ratio = detection.box.w * detection.box.h /
                std::max(1.0, old_box.w * old_box.h);
            if (class_changed && overlap < 0.60) continue;
            const double distance = cv::norm(center(old_box) - center(detection.box)) /
                std::max(8.0, std::hypot(std::max(old_box.w, detection.box.w),
                                        std::max(old_box.h, detection.box.h)));
            const double similarity = entry.second.appearance.empty()
                ? 0.0 : cosineSimilarity(detection_feature, entry.second.appearance);
            const double left = std::max(old_box.x, detection.box.x);
            const double top = std::max(old_box.y, detection.box.y);
            const double right = std::min(old_box.x + old_box.w, detection.box.x + detection.box.w);
            const double bottom = std::min(old_box.y + old_box.h, detection.box.y + detection.box.h);
            const double intersection = std::max(0.0, right - left) *
                std::max(0.0, bottom - top);
            const double contained_overlap = intersection /
                std::max(1.0, std::min(old_box.w * old_box.h,
                                       detection.box.w * detection.box.h));
            const bool small_track = std::max(old_box.w, old_box.h) < 20.0;
            const bool overlap_match = area_ratio >= 0.45 && area_ratio <= 2.2 &&
                overlap >= (class_changed ? 0.60 :
                    (small_detection ? 0.35 : 0.25)) &&
                (similarity >= appearance_gate - 0.15 || small_detection);
            const bool scale_change_match = !class_changed && contained_overlap >= 0.70 &&
                distance <= 0.30 &&
                (similarity >= appearance_gate - 0.15 || small_detection || small_track);
            const bool appearance_match = !class_changed && distance <= 0.30 &&
                similarity >= appearance_gate;
            if (!overlap_match && !scale_change_match && !appearance_match) continue;
            const double selected_bonus = entry.first == selected_track_id ? 1.0 : 0.0;
            const double score = selected_bonus + overlap + 0.25 * similarity - 0.1 * distance;
            if (score > best_score) {
              best_score = score;
              best_id = entry.first;
            }
          }
          if (best_id >= 0) {
            track_id = best_id;
            found = tracks_.find(track_id);
          }
        }

        Detection assigned = detection;
        assigned.id = track_id;
        used_track_ids_.insert(track_id);
        fresh_by_id[track_id] = assigned;
        if (found != tracks_.end()) continue;

        if (tracks_.size() >= kMaximumTracks) {
          auto oldest = tracks_.end();
          for (auto it = tracks_.begin(); it != tracks_.end(); ++it) {
            if (it->first == selected_track_id) continue;
            if (oldest == tracks_.end() ||
                it->second.detection.detector_stamp < oldest->second.detection.detector_stamp)
              oldest = it;
          }
          if (oldest != tracks_.end()) tracks_.erase(oldest);
        }
        TemplateTrack track;
        track.detection = assigned;
        track.box = assigned.box;
        track.gray_template = grayTemplate(frame, assigned.box);
        track.appearance = appearanceFeature(frame, assigned.box, &track.quality);
        track.last_update_stamp = stamp;
        track.last_observation_stamp = stamp;
        track.last_template_update_stamp = stamp;
        track.last_multi_scale_stamp = stamp;
        tracks_.emplace(track_id, std::move(track));
      }
    }

    for (auto it = tracks_.begin(); it != tracks_.end();) {
      const double age = it->first == selected_track_id
          ? stamp - it->second.last_observation_stamp
          : stamp - it->second.detection.detector_stamp;
      if (age > track_memory_sec) it = tracks_.erase(it);
      else ++it;
    }

    std::vector<Detection> output;
    output.reserve(tracks_.size());
    for (auto &entry : tracks_) {
      TemplateTrack &track = entry.second;
      const double dt = std::max(0.0, stamp - track.last_update_stamp);
      if (validateHomography(camera_transform, frame.size())) {
        track.box = transformBox(track.box, camera_transform);
      }
      Box predicted = track.box;
      const double prediction_dt = std::min(0.5, dt);
      predicted.x += track.velocity.x * prediction_dt;
      predicted.y += track.velocity.y * prediction_dt;
      predicted = clipBox(predicted, frame.size());

      const Detection *fresh_detection = nullptr;
      const auto fresh = fresh_by_id.find(entry.first);
      if (fresh != fresh_by_id.end()) fresh_detection = &fresh->second;
      const double lost_for = std::max(0.0, stamp - track.last_observation_stamp);
      const bool multi_scale_due = track.last_multi_scale_stamp <= 0.0 ||
          stamp - track.last_multi_scale_stamp >= multi_scale_period_sec;

      Box matched_box;
      double match_score = 0.0;
      bool template_matched = !track.gray_template.empty() &&
          matchGrayTemplate(gray, track.gray_template, predicted, lost_for, false,
              multi_scale_due, &matched_box, &match_score);
      auto validate_match = [&](const Box &candidate, double score, const Box &reference) {
        if (!validBox(candidate, frame.size()) ||
            score < match_threshold || !validBox(reference, frame.size())) return false;
        const double ratio = candidate.w * candidate.h /
            std::max(1.0, reference.w * reference.h);
        const double distance = cv::norm(center(candidate) - center(reference)) /
            std::max(8.0, std::hypot(reference.w, reference.h));
        if (ratio < 1.0 / max_area_ratio || ratio > max_area_ratio ||
            distance > 1.4 + std::min(3.0, lost_for * 0.8)) return false;
        double quality = 0.0;
        const auto feature = appearanceFeature(frame, candidate, &quality);
        const double similarity = track.appearance.empty()
            ? 1.0 : cosineSimilarity(feature, track.appearance);
        const bool small_patch = std::max(candidate.w, candidate.h) < 20.0;
        return (quality >= quality_gate || small_patch) &&
            similarity >= appearance_gate - (small_patch ? 0.10 : 0.0);
      };
      if (template_matched && !validate_match(matched_box, match_score, predicted))
        template_matched = false;

      if (multi_scale_due && !track.gray_template.empty())
        track.last_multi_scale_stamp = stamp;
      if (!template_matched && fresh_detection && !track.gray_template.empty()) {
        Box detector_match;
        double detector_score = 0.0;
        if (matchGrayTemplate(gray, track.gray_template, fresh_detection->box, lost_for,
                false, true, &detector_match, &detector_score) &&
            validate_match(detector_match, detector_score, fresh_detection->box)) {
          matched_box = detector_match;
          match_score = detector_score;
          template_matched = true;
          track.last_multi_scale_stamp = stamp;
        }
      }

      bool detector_fallback = false;
      if (fresh_detection && fresh_detection->confidence >= update_confidence) {
        double detector_quality = 0.0;
        const auto detector_feature = appearanceFeature(frame, fresh_detection->box, &detector_quality);
        const double detector_similarity = track.appearance.empty()
            ? 1.0 : cosineSimilarity(detector_feature, track.appearance);
        const bool small_detector_box = std::max(fresh_detection->box.w,
            fresh_detection->box.h) < 20.0;
        if ((detector_quality >= quality_gate || small_detector_box) &&
            detector_similarity >= appearance_gate - (small_detector_box ? 0.10 : 0.0)) {
          if (!template_matched) {
            matched_box = fresh_detection->box;
            match_score = std::max(match_threshold, std::min(0.95, fresh_detection->confidence));
            template_matched = true;
            detector_fallback = true;
          } else {
            const double separation = cv::norm(center(matched_box) - center(fresh_detection->box)) /
                std::max(8.0, std::hypot(matched_box.w, matched_box.h));
            if (separation <= 0.75 || boxIou(matched_box, fresh_detection->box) >= 0.10) {
              matched_box = smoothBox(matched_box, fresh_detection->box, 0.25);
            }
          }
          if (stamp - track.last_template_update_stamp >= template_refresh_sec) {
            cv::Mat current_template = grayTemplate(frame, fresh_detection->box);
            if (!current_template.empty()) {
              if (track.gray_template.empty()) track.gray_template = current_template;
              else {
                cv::Mat aligned;
                cv::resize(current_template, aligned, track.gray_template.size());
                cv::addWeighted(track.gray_template, 0.95, aligned, 0.05, 0.0,
                                track.gray_template);
              }
              if (track.appearance.size() == detector_feature.size()) {
                for (size_t i = 0; i < detector_feature.size(); ++i)
                  track.appearance[i] = 0.95f * track.appearance[i] + 0.05f * detector_feature[i];
              }
              track.last_template_update_stamp = stamp;
            }
          }
        }
      }

      if (template_matched) {
        const double previous_area = std::max(1.0, track.box.w * track.box.h);
        if (dt > 1e-3) {
          const cv::Point2d measured_velocity =
              (center(matched_box) - center(track.box)) / dt;
          track.velocity = 0.75 * track.velocity + 0.25 * measured_velocity;
        }
        track.box = clipBox(matched_box, frame.size());
        track.last_observation_stamp = stamp;
        track.quality = detector_fallback ? std::max(match_threshold, match_score) : match_score;
        track.scale_ratio = track.box.w * track.box.h /
            previous_area;
      } else {
        track.box = predicted;
      }
      track.last_update_stamp = stamp;
      if (fresh_detection) {
        track.detection = *fresh_detection;
      }
      if (stamp - track.last_observation_stamp > coast_hold_sec ||
          !validBox(track.box, frame.size())) continue;

      Detection tracked = track.detection;
      tracked.box = track.box;
      tracked.source = template_matched ? "template_tracker" : "template_prediction";
      tracked.flow_quality = template_matched ? track.quality : 0.0;
      tracked.flow_scale_ratio = track.scale_ratio;
      tracked.geometry = "aabb";
      tracked.has_rotation = false;
      if (!template_matched) tracked.confidence = std::min(0.25, tracked.confidence);
      output.push_back(std::move(tracked));
    }
    used_track_ids_.clear();
    pruneDuplicateTracks(selected_track_id);
    return suppressDuplicateOutput(std::move(output), selected_track_id);
  }

 private:
  struct TemplateTrack {
    Detection detection;
    Box box;
    cv::Mat gray_template;
    std::vector<float> appearance;
    cv::Point2d velocity{0.0, 0.0};
    double quality{0.0};
    double scale_ratio{1.0};
    double last_update_stamp{0.0};
    double last_observation_stamp{0.0};
    double last_template_update_stamp{0.0};
    double last_multi_scale_stamp{0.0};
  };

  static Box smoothBox(const Box &prior, const Box &measurement, double gain) {
    gain = clamp01(gain);
    return {prior.x + gain * (measurement.x - prior.x),
            prior.y + gain * (measurement.y - prior.y),
            prior.w + gain * (measurement.w - prior.w),
            prior.h + gain * (measurement.h - prior.h)};
  }

  static bool duplicateBoxes(const Box &a, int class_a,
                             const Box &b, int class_b) {
    return duplicateTargetBoxes(a, class_a, b, class_b);
  }

  bool preferredTrack(int64_t a_id, const TemplateTrack &a,
                      int64_t b_id, const TemplateTrack &b,
                      int64_t selected_track_id) const {
    if (a_id == b_id) return false;
    if (a_id == selected_track_id || b_id == selected_track_id)
      return a_id == selected_track_id;
    if (a.detection.detector_stamp != b.detection.detector_stamp)
      return a.detection.detector_stamp > b.detection.detector_stamp;
    if (a.quality != b.quality) return a.quality > b.quality;
    if (a.last_observation_stamp != b.last_observation_stamp)
      return a.last_observation_stamp > b.last_observation_stamp;
    return a_id < b_id;
  }

  void pruneDuplicateTracks(int64_t selected_track_id) {
    std::vector<int64_t> ids;
    ids.reserve(tracks_.size());
    for (const auto &entry : tracks_) ids.push_back(entry.first);
    std::sort(ids.begin(), ids.end(), [this, selected_track_id](int64_t a, int64_t b) {
      return preferredTrack(a, tracks_.at(a), b, tracks_.at(b), selected_track_id);
    });
    std::vector<int64_t> kept;
    for (int64_t id : ids) {
      const auto current = tracks_.find(id);
      if (current == tracks_.end()) continue;
      bool duplicate = false;
      for (int64_t kept_id : kept) {
        const auto other = tracks_.find(kept_id);
        if (other != tracks_.end() &&
            duplicateBoxes(current->second.box, current->second.detection.class_id,
                           other->second.box, other->second.detection.class_id)) {
          duplicate = true;
          break;
        }
      }
      if (duplicate && id != selected_track_id) tracks_.erase(current);
      else kept.push_back(id);
    }
  }

  std::vector<Detection> suppressDuplicateOutput(std::vector<Detection> output,
                                                  int64_t selected_track_id) const {
    std::sort(output.begin(), output.end(), [selected_track_id](const Detection &a,
                                                               const Detection &b) {
      if (a.id == b.id) return false;
      if (a.id == selected_track_id || b.id == selected_track_id)
        return a.id == selected_track_id;
      if (a.detector_stamp != b.detector_stamp) return a.detector_stamp > b.detector_stamp;
      if (a.flow_quality != b.flow_quality) return a.flow_quality > b.flow_quality;
      return a.confidence > b.confidence;
    });
    std::vector<Detection> unique;
    unique.reserve(output.size());
    for (auto &detection : output) {
      const auto duplicate = std::find_if(unique.begin(), unique.end(),
          [&detection](const Detection &kept) {
            return duplicateBoxes(detection.box, detection.class_id,
                                  kept.box, kept.class_id);
          });
      if (duplicate == unique.end()) unique.push_back(std::move(detection));
    }
    return unique;
  }

  static constexpr size_t kMaximumTracks = 30;
  std::unordered_map<int64_t, TemplateTrack> tracks_;
  std::unordered_set<int64_t> used_track_ids_;
};

class SmartTrackerFusionCpp {
 public:
  SmartTrackerFusionCpp() : nh_(), pnh_("~"), tf_listener_(tf_buffer_) {
    cv::setNumThreads(1);
    loadConfig();
    createRosApi();
  }

  ~SmartTrackerFusionCpp() { stopWorker(); }

  void run() {
    startWorker();
    ros::waitForShutdown();
    stopWorker();
  }

 private:
  struct InferenceJob {
    cv::Mat frame;
    std_msgs::Header header;
    uint64_t mode_epoch{0};
  };

  struct InferenceResult {
    uint64_t generation{0};
    uint64_t mode_epoch{0};
    double stamp{0.0};
    double inference_ms{0.0};
    std::vector<Detection> detections;
  };

  void loadConfig();
  void createRosApi();
  void startWorker();
  void stopWorker();
  void imageCallback(const sensor_msgs::ImageConstPtr &image);
  void processLoop();
  void inferenceLoop();
  void enqueueInference(const cv::Mat &frame, const std_msgs::Header &header,
                        uint64_t mode_epoch);
  bool takeInferenceResult(double frame_stamp, uint64_t mode_epoch,
                           InferenceResult *result);
  void process(const sensor_msgs::ImageConstPtr &image);
  bool requestDetections(const cv::Mat &frame, const std_msgs::Header &header,
                         std::vector<Detection> *detections);
  void publishOutputs(const sensor_msgs::ImageConstPtr &image, const cv::Mat &frame,
                      const std::vector<Detection> &detections, bool fresh,
                      double inference_ms, double processing_ms);
  bool selectFromSnapshot(const Snapshot &snapshot, const Box &requested,
                          bool snap_to_detection, bool manual_seed,
                          int64_t *selected_id, std::string *error);
  bool selectConfiguredColorCandidate(const Snapshot &snapshot,
                                      const Detection &candidate,
                                      const std::string &reason,
                                      bool auto_selection = false);
  void resetSelection();
  bool selectService(sar_yolo_detector::SelectSmartTrack::Request &request,
                     sar_yolo_detector::SelectSmartTrack::Response &response);
  bool switchMode(sar_yolo_detector::SwitchTrackingMode::Request &request,
                  sar_yolo_detector::SwitchTrackingMode::Response &response);
  bool switchModel(sar_yolo_detector::SwitchSmartTrackerModel::Request &request,
                   sar_yolo_detector::SwitchSmartTrackerModel::Response &response);
  bool selectRoi(sar_yolo_detector::SelectFusionROI::Request &request,
                 sar_yolo_detector::SelectFusionROI::Response &response,
                 bool manual_seed);
  bool selectPoint(sar_yolo_detector::SelectFusionPoint::Request &request,
                   sar_yolo_detector::SelectFusionPoint::Response &response);
  bool autoSelect(std_srvs::SetBool::Request &request,
                  std_srvs::SetBool::Response &response);
  void pointCallback(const geometry_msgs::PointStampedConstPtr &point);
  void manualRoiCallback(const sensor_msgs::RegionOfInterestConstPtr &roi);
  void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &message);
  void metricTargetStatusCallback(const std_msgs::StringConstPtr &message);
  cv::Mat poseRotation(const sensor_msgs::Image &message);
  void phaseDetectionCallback(const xd_uav_track::DetectionArrayConstPtr &message);
  void phaseStatusCallback(const xd_uav_track::TrackStatusConstPtr &message);
  void phaseGimbalCallback(const xd_uav_track::GimbalStateConstPtr &message);
  bool fusionRoiService(sar_yolo_detector::SelectFusionROI::Request &request,
                        sar_yolo_detector::SelectFusionROI::Response &response) {
    return selectRoi(request, response, false);
  }
  bool manualRoiService(sar_yolo_detector::SelectFusionROI::Request &request,
                        sar_yolo_detector::SelectFusionROI::Response &response) {
    return selectRoi(request, response, true);
  }
  bool trackingPhase();
  void drawOverlay(cv::Mat &frame, const std::vector<Detection> &detections);
  void publishEmptySelection(const std_msgs::Header &header);
  bool initializeColorFromRoi(const Snapshot &snapshot, const Box &roi);
  void processColor(const cv::Mat &frame, double stamp,
                    std::vector<Detection> &detections);
  void processFusion(const cv::Mat &frame, double stamp,
                     const std::vector<Detection> &detections,
                     bool detector_updated, const cv::Mat &camera_transform,
                     const Detection *visual_observation);
  void updateAutoSelection(const std::vector<Detection> &detections, const cv::Mat &frame,
                           const sensor_msgs::ImageConstPtr &image);
  std::string className(int class_id) const;
  std::string updateLossJournal(const TrackResult &selected, double stamp);

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  ros::Subscriber image_sub_;
  ros::Subscriber point_sub_;
  ros::Subscriber roi_sub_;
  ros::Subscriber camera_info_sub_;
  ros::Subscriber metric_target_status_sub_;
  ros::Subscriber phase_detection_sub_;
  ros::Subscriber phase_status_sub_;
  ros::Subscriber phase_gimbal_sub_;
  ros::Publisher tracked_pub_;
  ros::Publisher fusion_pub_;
  ros::Publisher observations_pub_;
  ros::Publisher evidence_pub_;
  ros::Publisher loss_pub_;
  ros::Publisher state_pub_;
  ros::Publisher annotated_pub_;
  ros::Publisher selected_pub_;
  ros::Publisher auto_status_pub_;
  ros::ServiceClient info_client_;
  ros::ServiceClient inference_client_;
  ros::ServiceServer select_srv_;
  ros::ServiceServer mode_srv_;
  ros::ServiceServer model_srv_;
  ros::ServiceServer fusion_roi_srv_;
  ros::ServiceServer manual_roi_srv_;
  ros::ServiceServer fusion_point_srv_;
  ros::ServiceServer auto_srv_;

  std::mutex image_mutex_;
  std::condition_variable image_cv_;
  sensor_msgs::ImageConstPtr pending_image_;
  std::thread worker_;
  std::mutex inference_mutex_;
  std::condition_variable inference_cv_;
  InferenceJob pending_inference_job_;
  bool has_pending_inference_job_{false};
  std::thread inference_worker_;
  std::mutex inference_result_mutex_;
  InferenceResult latest_inference_result_;
  uint64_t consumed_inference_generation_{0};
  uint64_t next_inference_generation_{0};
  std::mutex inference_client_mutex_;
  std::atomic<bool> stopping_{false};
  std::mutex state_mutex_;
  mutable std::mutex labels_mutex_;
  std::mutex phase_mutex_;

  std::string model_path_;
  std::string model_key_;
  std::string inference_base_;
  std::string source_id_;
  std::string input_topic_;
  std::string camera_info_topic_;
  std::string metric_target_status_topic_;
  std::string motion_fixed_frame_{"world"};
  std::string annotated_topic_;
  std::string tracked_topic_;
  std::string fusion_topic_;
  std::string observations_topic_;
  std::string state_topic_;
  std::string selected_topic_;
  std::string select_pixel_topic_;
  std::string auto_service_name_;
  std::string auto_status_topic_;
  std::string manual_roi_topic_;
  std::string mode_{"fusion_tracker"};
  bool shared_inference_enabled_{false};
  uint64_t mode_epoch_{0};
  std::string color_name_;
  std::string model_version_;
  std::string mission_id_;
  std::string uav_id_;
  std::string session_uuid_;
  std::string sensor_id_;
  std::string profile_;
  std::string model_sha256_;
  std::string calibration_version_;
  std::string coordinate_contract_;
  std::unordered_map<int, std::string> labels_;
  std::unordered_map<int, int> class_group_by_id_;
  std::vector<std::vector<int>> class_groups_;
  YAML::Node fusion_config_;

  double confidence_threshold_{0.10};
  double iou_threshold_{0.60};
  double inference_max_fps_{20.0};
  double annotated_max_fps_{40.0};
  double maximum_age_sec_{2.0};
  double future_tolerance_sec_{0.02};
  double last_inference_wall_{0.0};
  double last_annotated_wall_{0.0};
  double last_capture_stamp_{0.0};
  int inference_image_size_{512};
  int max_detections_{30};
  bool publish_annotated_{true};
  bool use_gpu_{true};
  bool phase_overlay_enabled_{false};
  std::string phase_detection_topic_;
  std::string phase_status_topic_;
  std::string phase_gimbal_topic_;
  double phase_detection_timeout_{0.5};
  double phase_status_timeout_{0.5};
  double phase_gimbal_timeout_{0.5};
  double phase_min_duration_{1.0};
  double phase_enter_error_{0.05};
  double phase_exit_error_{0.40};
  double phase_enter_yaw_{0.08};
  double phase_exit_yaw_{0.18};
  bool phase_has_detection_{false};
  bool phase_tracker_active_{false};
  bool phase_target_visible_{false};
  bool phase_command_valid_{false};
  bool phase_gimbal_valid_{false};
  bool phase_status_gimbal_valid_{false};
  bool phase_attacking_{false};
  bool phase_gimbal_yaw_inverted_{false};
  bool phase_gimbal_filter_initialized_{false};
  double phase_error_x_{0.0};
  double phase_error_y_{0.0};
  double phase_gimbal_yaw_{0.0};
  double phase_filtered_gimbal_yaw_{0.0};
  double phase_gimbal_yaw_offset_{0.0};
  double phase_gimbal_filter_previous_weight_{0.7};
  double phase_gimbal_angle_deadzone_{0.0};
  double phase_detection_wall_{0.0};
  double phase_status_wall_{0.0};
  double phase_gimbal_wall_{0.0};
  double phase_tracking_started_{0.0};
  std::string phase_profile_;
  std::string phase_text_{"UNIDENTIFICATION"};

  cv::Mat latest_frame_;
  cv::Mat previous_frame_;
  std_msgs::Header latest_header_;
  std::vector<Detection> latest_detections_;
  std::vector<Detection> last_detector_detections_;
  double last_detector_stamp_{0.0};
  std::deque<Snapshot> snapshots_;
  size_t snapshot_bytes_{0};
  int snapshot_max_frames_{24};
  size_t snapshot_max_bytes_{25165824};
  double snapshot_max_age_{0.75};

  cv::Ptr<cv::TrackerKCF> kcf_;
  // FusionTracker keeps an adaptive appearance patch for long visual searches.
  // It can follow visual matches or YOLO boxes overlapping the locked target.
  cv::Mat target_template_gray_;
  double last_template_update_stamp_{0.0};
  bool selected_{false};
  bool manual_seed_{false};
  bool manual_active_{false};
  Box selected_box_;
  Box manual_seed_box_;
  int64_t selected_id_{-1};
  int64_t selected_local_id_{-1};
  int selected_class_id_{-1};
  int64_t next_id_{900000001};
  int64_t next_detector_id_{1};
  double selected_confidence_{0.0};
  double selected_stamp_{0.0};
  double last_fusion_stamp_{0.0};
  double last_observation_stamp_{0.0};
  double last_detector_for_selection_{0.0};
  std::vector<float> appearance_anchor_;
  double anchor_area_{0.0};
  cv::Point2d velocity_{0.0, 0.0};
  TrackResult result_;
  int pending_reacquire_id_{-1};
  int pending_reacquire_count_{0};
  double pending_reacquire_stamp_{0.0};
  Box pending_reacquire_box_;
  int confirmation_frames_{3};
  double visual_hold_sec_{1.5};
  double prediction_hold_sec_{0.35};
  double detector_id_memory_sec_{1.5};
  double reacquire_detector_gain_{0.35};
  double reacquire_hold_sec_{8.0};
  double identity_ttl_sec_{60.0};
  double max_frame_gap_sec_{0.75};
  double appearance_gate_{0.62};
  double quality_gate_{0.30};
  double long_appearance_gate_{0.76};
  double long_quality_gate_{0.50};
  double long_score_gate_{0.65};
  double ambiguity_margin_{0.12};
  double center_gate_sizes_{1.6};
  double max_area_ratio_{3.0};
  double detector_weight_{0.70};
  double detector_authority_{0.85};
  double min_detector_confidence_{0.50};
  double min_maintenance_confidence_{0.10};
  double local_template_refresh_sec_{0.40};
  double template_match_threshold_{0.48};
  double long_template_match_threshold_{0.64};
  double template_update_confidence_{0.70};
  double template_search_period_sec_{0.20};
  double last_template_search_stamp_{0.0};
  int64_t last_template_candidate_trigger_id_{-1};
  double pending_box_stamp_{0.0};
  bool motion_continuous_{true};
  bool long_recovery_{false};
  bool manual_seed_without_yolo_{false};
  bool manual_seed_visual_without_yolo_{true};
  bool long_motion_required_{true};
  double motion_horizon_sec_{2.0};
  Box manual_seed_region_;
  CameraMotion camera_motion_{YAML::Node(YAML::NodeType::Map)};
  cv::Mat camera_info_k_;
  sensor_msgs::CameraInfo camera_info_;
  TemplateDetections template_detections_;
  cv::Mat previous_rotation_;
  std::string camera_frame_id_;
  bool camera_info_valid_{false};
  bool metric_target_status_received_{false};
  bool metric_target_valid_{false};
  double processed_frames_{0.0};
  double inference_failures_{0.0};
  std::string last_error_;
  bool auto_select_enabled_{false};
  bool auto_selected_{false};
  double auto_selected_missing_since_{0.0};
  int auto_select_stable_frames_{2};
  int auto_select_count_{0};
  int64_t auto_select_candidate_id_{-1};
  int auto_select_candidate_class_{-1};
  double auto_select_min_confidence_{0.0};
  double auto_select_max_gap_{1.0};
  double auto_select_lost_hold_sec_{8.0};
  double auto_select_last_stamp_{0.0};
  Box auto_select_box_;
  bool manual_enabled_{true};
  bool color_waiting_roi_{false};
  bool color_initialized_{false};
  std::vector<float> color_histogram_;
  cv::Mat color_mask_;
  Box pending_roi_;
  bool has_pending_roi_{false};
  bool model_info_loaded_{false};
  std::vector<int> allowed_class_ids_;
  int color_class_id_{2};
  int64_t manual_track_id_{900000000};
  double manual_min_confidence_{0.0};
  int minimum_color_area_{80};
  bool auto_select_pub_status_{false};
  bool auto_select_candidate_stable_{false};
  std::deque<std::string> loss_context_;
  std::deque<std::string> recent_loss_events_;
  std::deque<std::string> active_loss_after_;
  bool loss_active_{false};
  int64_t loss_event_selected_id_{-1};
  int64_t loss_event_last_local_id_{-1};
  uint64_t loss_event_count_{0};
  uint64_t loss_recovered_count_{0};
  uint64_t loss_interrupted_count_{0};
  uint64_t loss_published_revision_{0};
  double loss_event_start_stamp_{0.0};
  double loss_event_last_stamp_{0.0};
  double last_loss_publish_wall_{0.0};
  std::string loss_first_reason_;
  std::string loss_last_reason_;
  uint64_t loss_summary_event_id_{0};
  std::string loss_summary_reason_;
  double loss_summary_duration_{0.0};
  double loss_finalized_stamp_{0.0};
};

void SmartTrackerFusionCpp::loadConfig() {
  pnh_.param<std::string>("model_path", model_path_, "");
  pnh_.param<std::string>("model_sha256", model_sha256_, "");
  pnh_.param<std::string>("input_image_topic", input_topic_, "camera/image_raw");
  pnh_.param<std::string>("camera_info_topic", camera_info_topic_, "");
  pnh_.param<std::string>("metric_target_status_topic", metric_target_status_topic_,
                          "track/metric_target_status");
  pnh_.param<std::string>("camera_motion_fixed_frame", motion_fixed_frame_, "world");
  pnh_.param<std::string>("annotated_image_topic", annotated_topic_,
                          "sar_yolo_detector/smart/annotated");
  pnh_.param<std::string>("tracked_detections_topic", tracked_topic_,
                          "sar_yolo_detector/smart/tracked_detections");
  pnh_.param<std::string>("fusion_detections_topic", fusion_topic_,
                          tracked_topic_ + "/fusion");
  pnh_.param<std::string>("fusion_observations_topic", observations_topic_,
                          tracked_topic_ + "/observations");
  pnh_.param<std::string>("state_topic", state_topic_, "sar_yolo_detector/smart/state");
  pnh_.param<std::string>("selected_detections_topic", selected_topic_,
                          "sar_yolo_detector/smart/selected_tracked_detections");
  pnh_.param<std::string>("select_pixel_topic", select_pixel_topic_,
                          "sar_yolo_detector/smart/select_pixel");
  pnh_.param<std::string>("auto_select_service", auto_service_name_,
                          "sar_yolo_detector/smart/auto_select");
  pnh_.param<std::string>("auto_select_status_topic", auto_status_topic_,
                          "sar_yolo_detector/smart/auto_select_status");
  pnh_.param<std::string>("mission_id", mission_id_, "mission_unset");
  pnh_.param<std::string>("uav_id", uav_id_, "uav_unset");
  pnh_.param<std::string>("session_uuid", session_uuid_, "");
  pnh_.param<std::string>("sensor_id", sensor_id_, "camera_primary");
  pnh_.param<std::string>("profile", profile_, "generic");
  pnh_.param<std::string>("model_version", model_version_, "configured_model");
  pnh_.param<std::string>("calibration_version", calibration_version_, "unversioned");
  pnh_.param<std::string>("coordinate_contract", coordinate_contract_, "CAMERA_RAY");
  pnh_.param<std::string>("ManualBoxTracker/initial_roi_topic", manual_roi_topic_,
                          "sar_yolo_detector/manual/initial_roi");
  pnh_.param("publish_annotated_image", publish_annotated_, true);
  pnh_.param("SmartTracker/SMART_TRACKER_USE_GPU", use_gpu_, true);
  pnh_.param("SmartTracker/SMART_TRACKER_CONFIDENCE_THRESHOLD", confidence_threshold_, 0.10);
  pnh_.param("SmartTracker/SMART_TRACKER_IOU_THRESHOLD", iou_threshold_, 0.60);
  pnh_.param("SmartTracker/SMART_TRACKER_MAX_DETECTIONS", max_detections_, 30);
  pnh_.param("SmartTracker/SMART_TRACKER_INFERENCE_IMAGE_SIZE", inference_image_size_, 512);
  pnh_.param("inference_max_fps", inference_max_fps_, 20.0);
  pnh_.param("annotated_max_fps", annotated_max_fps_, 40.0);
  pnh_.param("maximum_capture_age_sec", maximum_age_sec_, 2.0);
  pnh_.param("future_timestamp_tolerance_sec", future_tolerance_sec_, 0.02);
  pnh_.param("Default_Tracker", mode_, std::string("FusionTracker"));
  pnh_.param("shared_inference", shared_inference_enabled_, false);
  mode_ = lower(mode_);
  if (mode_ == "fusiontracker" || mode_ == "fusion") mode_ = "fusion_tracker";
  if (mode_ == "featuretracker" || mode_ == "manualboxtracker" || mode_ == "manual") mode_ = "feature_tracker";
  if (mode_ == "colortracker" || mode_ == "colour_tracker") mode_ = "color_tracker";
  if (mode_ != "fusion_tracker" && mode_ != "feature_tracker" && mode_ != "color_tracker") {
    throw std::runtime_error("Default_Tracker must be FusionTracker, FeatureTracker, or ColorTracker");
  }
  if (!shared_inference_enabled_) {
    throw std::runtime_error(
        "The C++ SmartTracker requires shared_inference:=true and a running shared YOLO provider");
  }

  if (model_path_.empty()) throw std::runtime_error("model_path is required");
  model_key_ = lower(model_sha256_);
  if (model_key_.size() != 64) throw std::runtime_error("model_sha256 must contain a 64-character model key");
  inference_base_ = "/sar_yolo/shared_inference/m_" + model_key_;
  std::string configured_base;
  pnh_.param<std::string>("SmartTracker/SMART_TRACKER_SHARED_SERVICE_BASE", configured_base, "");
  if (!configured_base.empty()) inference_base_ = "/" + configured_base;
  source_id_ = uav_id_ + "/" + sensor_id_ + "/" + ros::this_node::getName();
  confidence_threshold_ = clamp01(confidence_threshold_);
  iou_threshold_ = clamp01(iou_threshold_);
  inference_max_fps_ = std::max(0.0, inference_max_fps_);
  annotated_max_fps_ = std::max(0.0, annotated_max_fps_);
  inference_image_size_ = std::max(320, std::min(1536, inference_image_size_));
  max_detections_ = std::max(1, std::min(100, max_detections_));

  std::vector<int> allowed_classes;
  if (pnh_.getParam("SmartTracker/SMART_TRACKER_ALLOWED_CLASS_IDS", allowed_classes)) {
    allowed_class_ids_ = allowed_classes;
    std::vector<int> group;
    for (int class_id : allowed_classes) if (class_id >= 0) group.push_back(class_id);
    if (!group.empty()) class_groups_.push_back(group);
  }
  XmlRpc::XmlRpcValue groups;
  if (pnh_.getParam("FusionTracker/class_groups", groups) && groups.getType() == XmlRpc::XmlRpcValue::TypeArray) {
    class_groups_.clear();
    for (int i = 0; i < groups.size(); ++i) {
      if (groups[i].getType() != XmlRpc::XmlRpcValue::TypeArray) continue;
      std::vector<int> group;
      for (int j = 0; j < groups[i].size(); ++j) {
        if (groups[i][j].getType() == XmlRpc::XmlRpcValue::TypeInt) group.push_back(static_cast<int>(groups[i][j]));
      }
      if (!group.empty()) class_groups_.push_back(group);
    }
  }
  for (size_t i = 0; i < class_groups_.size(); ++i) {
    for (int id : class_groups_[i]) class_group_by_id_[id] = static_cast<int>(i);
  }

  pnh_.param("FusionTracker/visual_hold_sec", visual_hold_sec_, 1.5);
  pnh_.param("FusionTracker/prediction_hold_sec", prediction_hold_sec_, 0.35);
  pnh_.param("FusionTracker/detector_id_memory_sec", detector_id_memory_sec_, 1.5);
  pnh_.param("FusionTracker/reacquire_detector_gain", reacquire_detector_gain_, 0.35);
  pnh_.param("FusionTracker/reacquire_hold_sec", reacquire_hold_sec_, 8.0);
  pnh_.param("FusionTracker/identity_memory_ttl_sec", identity_ttl_sec_, 60.0);
  pnh_.param("FusionTracker/manual_seed_visual_without_yolo", manual_seed_visual_without_yolo_, true);
  pnh_.param("FusionTracker/long_reacquire_require_camera_motion", long_motion_required_, true);
  pnh_.param("FusionTracker/reacquire_motion_horizon_sec", motion_horizon_sec_, 2.0);
  pnh_.param("FusionTracker/max_frame_gap_sec", max_frame_gap_sec_, 0.75);
  pnh_.param("FusionTracker/appearance_gate", appearance_gate_, 0.62);
  pnh_.param("FusionTracker/quality_gate", quality_gate_, 0.30);
  pnh_.param("FusionTracker/long_reacquire_appearance_gate", long_appearance_gate_, 0.76);
  pnh_.param("FusionTracker/long_reacquire_quality_gate", long_quality_gate_, 0.50);
  pnh_.param("FusionTracker/long_reacquire_minimum_score", long_score_gate_, 0.65);
  pnh_.param("FusionTracker/ambiguity_margin", ambiguity_margin_, 0.12);
  pnh_.param("FusionTracker/center_gate_sizes", center_gate_sizes_, 1.6);
  pnh_.param("FusionTracker/max_area_ratio", max_area_ratio_, 3.0);
  pnh_.param("FusionTracker/detector_weight", detector_weight_, 0.70);
  pnh_.param("FusionTracker/detector_authority_confidence", detector_authority_, 0.85);
  pnh_.param("FusionTracker/minimum_detector_confidence", min_detector_confidence_, 0.50);
  pnh_.param("FusionTracker/minimum_maintenance_confidence", min_maintenance_confidence_, 0.10);
  pnh_.param("FusionTracker/local_template_refresh_sec", local_template_refresh_sec_, 0.40);
  pnh_.param("FusionTracker/template_match_threshold", template_match_threshold_, 0.48);
  pnh_.param("FusionTracker/long_template_match_threshold", long_template_match_threshold_, 0.64);
  pnh_.param("FusionTracker/template_update_confidence", template_update_confidence_, 0.70);
  pnh_.param("FusionTracker/template_search_period_sec", template_search_period_sec_, 0.20);
  pnh_.param("FusionTracker/reacquire_confirm_frames", confirmation_frames_, 3);
  detector_id_memory_sec_ = std::max(0.0, detector_id_memory_sec_);
  reacquire_detector_gain_ = clamp01(reacquire_detector_gain_);
  template_match_threshold_ = clamp01(template_match_threshold_);
  long_template_match_threshold_ = std::max(template_match_threshold_,
                                             clamp01(long_template_match_threshold_));
  template_update_confidence_ = clamp01(template_update_confidence_);
  template_search_period_sec_ = std::max(0.05, template_search_period_sec_);
  pnh_.param("FusionTracker/selection_max_frames", snapshot_max_frames_, 24);
  int snapshot_bytes = 25165824;
  pnh_.param("FusionTracker/selection_max_bytes", snapshot_bytes, snapshot_bytes);
  snapshot_max_bytes_ = static_cast<size_t>(std::max(1024, snapshot_bytes));
  pnh_.param("FusionTracker/selection_max_age_sec", snapshot_max_age_, 0.75);
  if (identity_ttl_sec_ < 0.0 ||
      (identity_ttl_sec_ > 0.0 &&
       (identity_ttl_sec_ <= reacquire_hold_sec_ || identity_ttl_sec_ > 300.0))) {
    throw std::runtime_error("FusionTracker identity_memory_ttl_sec must be 0 (until reselection) or exceed reacquire_hold_sec and be <= 300");
  }
  if (min_maintenance_confidence_ > min_detector_confidence_) {
    throw std::runtime_error("FusionTracker minimum_maintenance_confidence must not exceed minimum_detector_confidence");
  }
  YAML::Node motion_config(YAML::NodeType::Map);
  bool motion_enabled = true;
  int motion_features = 160, motion_inliers = 12;
  double motion_gap = 0.75;
  pnh_.param("FusionTracker/camera_motion/enabled", motion_enabled, motion_enabled);
  pnh_.param("FusionTracker/camera_motion/max_features", motion_features, motion_features);
  pnh_.param("FusionTracker/camera_motion/minimum_inliers", motion_inliers, motion_inliers);
  pnh_.param("FusionTracker/camera_motion/max_gap_sec", motion_gap, motion_gap);
  motion_config["enabled"] = motion_enabled;
  motion_config["max_features"] = motion_features;
  motion_config["minimum_inliers"] = motion_inliers;
  motion_config["max_gap_sec"] = motion_gap;
  camera_motion_ = CameraMotion(motion_config);

  pnh_.param("ManualBoxTracker/enabled", manual_enabled_, true);
  pnh_.param("ManualBoxTracker/class_id", color_class_id_, 2);
  int manual_track_id = 900000000;
  pnh_.param("ManualBoxTracker/track_id", manual_track_id, manual_track_id);
  manual_track_id_ = manual_track_id;
  pnh_.param("ManualBoxTracker/minimum_confidence", manual_min_confidence_, 0.0);
  pnh_.param("ManualBoxTracker/color_tracker/min_area_px", minimum_color_area_, 80);
  pnh_.param<std::string>("ManualBoxTracker/color_tracker/color", color_name_, "red");
  pnh_.param("AutoSelect/enabled_at_startup", auto_select_enabled_, false);
  pnh_.param("AutoSelect/stable_frames", auto_select_stable_frames_, 2);
  pnh_.param("AutoSelect/minimum_confidence", auto_select_min_confidence_, 0.0);
  pnh_.param("AutoSelect/max_frame_gap_sec", auto_select_max_gap_, 1.0);
  pnh_.param("AutoSelect/lost_hold_sec", auto_select_lost_hold_sec_, reacquire_hold_sec_);
  if (!std::isfinite(auto_select_lost_hold_sec_) || auto_select_lost_hold_sec_ <= 0.0 ||
      auto_select_lost_hold_sec_ > 300.0) {
    throw std::runtime_error("AutoSelect lost_hold_sec must be > 0 and <= 300 seconds");
  }
  color_waiting_roi_ = mode_ == "color_tracker" && !hasConfiguredColor(color_name_);

  pnh_.param("PhaseOverlay/enabled", phase_overlay_enabled_, false);
  pnh_.param<std::string>("PhaseOverlay/input_detections_topic", phase_detection_topic_, "track/selected_detections");
  pnh_.param<std::string>("PhaseOverlay/status_topic", phase_status_topic_, "track/status");
  pnh_.param<std::string>("PhaseOverlay/gimbal_state_topic", phase_gimbal_topic_, "track/gimbal_state");
  pnh_.param("PhaseOverlay/detection_timeout_sec", phase_detection_timeout_, 0.5);
  pnh_.param("PhaseOverlay/status_timeout_sec", phase_status_timeout_, 0.5);
  pnh_.param("PhaseOverlay/gimbal_state_timeout_sec", phase_gimbal_timeout_, 0.5);
  pnh_.param("PhaseOverlay/attacking_min_tracking_duration_sec", phase_min_duration_, 1.0);
  pnh_.param("PhaseOverlay/attacking_error_threshold", phase_enter_error_, 0.05);
  pnh_.param("PhaseOverlay/attacking_error_exit_threshold", phase_exit_error_, 0.40);
  pnh_.param("PhaseOverlay/attacking_gimbal_yaw_threshold_rad", phase_enter_yaw_, 0.08);
  pnh_.param("PhaseOverlay/attacking_gimbal_yaw_exit_threshold_rad", phase_exit_yaw_, 0.18);
  pnh_.param("phase_gimbal_yaw_inverted", phase_gimbal_yaw_inverted_, false);
  pnh_.param("phase_gimbal_yaw_offset_rad", phase_gimbal_yaw_offset_, 0.0);
  pnh_.param("phase_gimbal_filter_previous_weight", phase_gimbal_filter_previous_weight_, 0.7);
  pnh_.param("phase_gimbal_angle_deadzone_rad", phase_gimbal_angle_deadzone_, 0.0);
  phase_gimbal_filter_previous_weight_ = std::max(0.0, std::min(0.99, phase_gimbal_filter_previous_weight_));
  phase_gimbal_angle_deadzone_ = std::max(0.0, phase_gimbal_angle_deadzone_);

  std::ostringstream id;
  id << "smart_" << std::hex << std::hash<std::string>{}(std::to_string(SteadyClock::now().time_since_epoch().count()));
  if (session_uuid_.empty() || session_uuid_ == "smart_tracker_unset") session_uuid_ = id.str();
  ROS_INFO_STREAM("C++ SmartTracker configured: model=" << model_path_
      << " backend=shared_remote mode=" << mode_ << " inference_max_fps=" << inference_max_fps_);
}

void SmartTrackerFusionCpp::createRosApi() {
  tracked_pub_ = nh_.advertise<sar_yolo_detector::TrackedDetection2DArray>(tracked_topic_, 3);
  fusion_pub_ = nh_.advertise<sar_yolo_detector::FusionDetection2DArray>(fusion_topic_, 1);
  observations_pub_ = nh_.advertise<sar_yolo_detector::FusionDetection2DArray>(observations_topic_, 1);
  evidence_pub_ = pnh_.advertise<std_msgs::String>("fusion_evidence", 1);
  loss_pub_ = pnh_.advertise<std_msgs::String>("fusion_loss_history", 1, true);
  state_pub_ = nh_.advertise<sar_yolo_detector::SmartTrackerState>(state_topic_, 3);
  if (publish_annotated_) annotated_pub_ = nh_.advertise<sensor_msgs::Image>(annotated_topic_, 1);
  selected_pub_ = nh_.advertise<sar_yolo_detector::TrackedDetection2DArray>(selected_topic_, 3);
  auto_status_pub_ = nh_.advertise<std_msgs::Bool>(auto_status_topic_, 1, true);

  image_sub_ = nh_.subscribe<sensor_msgs::Image>(input_topic_, 1,
      &SmartTrackerFusionCpp::imageCallback, this, ros::TransportHints().tcpNoDelay());
  point_sub_ = nh_.subscribe<geometry_msgs::PointStamped>(select_pixel_topic_, 1,
      &SmartTrackerFusionCpp::pointCallback, this, ros::TransportHints().tcpNoDelay());
  if (!manual_roi_topic_.empty()) {
    roi_sub_ = nh_.subscribe<sensor_msgs::RegionOfInterest>(manual_roi_topic_, 1,
        &SmartTrackerFusionCpp::manualRoiCallback, this);
  }
  if (!camera_info_topic_.empty()) {
    camera_info_sub_ = nh_.subscribe<sensor_msgs::CameraInfo>(camera_info_topic_, 1,
        &SmartTrackerFusionCpp::cameraInfoCallback, this);
  }
  if (!metric_target_status_topic_.empty()) {
    metric_target_status_sub_ = nh_.subscribe<std_msgs::String>(
        metric_target_status_topic_, 1,
        &SmartTrackerFusionCpp::metricTargetStatusCallback, this);
  }
  if (phase_overlay_enabled_) {
    phase_detection_sub_ = nh_.subscribe<xd_uav_track::DetectionArray>(phase_detection_topic_, 1,
        &SmartTrackerFusionCpp::phaseDetectionCallback, this);
    phase_status_sub_ = nh_.subscribe<xd_uav_track::TrackStatus>(phase_status_topic_, 1,
        &SmartTrackerFusionCpp::phaseStatusCallback, this);
    phase_gimbal_sub_ = nh_.subscribe<xd_uav_track::GimbalState>(phase_gimbal_topic_, 1,
        &SmartTrackerFusionCpp::phaseGimbalCallback, this);
  }

  select_srv_ = pnh_.advertiseService("select", &SmartTrackerFusionCpp::selectService, this);
  mode_srv_ = pnh_.advertiseService("switch_tracking_mode", &SmartTrackerFusionCpp::switchMode, this);
  model_srv_ = pnh_.advertiseService("switch_model", &SmartTrackerFusionCpp::switchModel, this);
  fusion_roi_srv_ = pnh_.advertiseService("select_fusion_roi",
      &SmartTrackerFusionCpp::fusionRoiService, this);
  manual_roi_srv_ = pnh_.advertiseService("select_fusion_manual_roi",
      &SmartTrackerFusionCpp::manualRoiService, this);
  fusion_point_srv_ = pnh_.advertiseService("select_fusion_point", &SmartTrackerFusionCpp::selectPoint, this);
  auto_srv_ = nh_.advertiseService(auto_service_name_, &SmartTrackerFusionCpp::autoSelect, this);
  std_msgs::Bool auto_status;
  auto_status.data = auto_select_enabled_;
  auto_status_pub_.publish(auto_status);

  // Reuse the TCPROS connection on the inference worker to avoid a service
  // handshake for every frame. The worker resets it after a transport failure
  // so a newly elected model provider can be discovered.
  info_client_ = nh_.serviceClient<sar_yolo_detector::GetSharedModelInfo>(inference_base_ + "/info", false);
  inference_client_ = nh_.serviceClient<sar_yolo_detector::RunSharedInference>(inference_base_ + "/detect", true);
  ROS_INFO_STREAM("C++ SmartTracker ROS API ready; image=" << input_topic_ << " annotated=" << annotated_topic_
      << " inference=" << inference_base_
      << " metric_target_status=" << metric_target_status_topic_);
}

void SmartTrackerFusionCpp::startWorker() {
  stopping_ = false;
  worker_ = std::thread(&SmartTrackerFusionCpp::processLoop, this);
  inference_worker_ = std::thread(&SmartTrackerFusionCpp::inferenceLoop, this);
}

void SmartTrackerFusionCpp::stopWorker() {
  stopping_ = true;
  image_cv_.notify_all();
  inference_cv_.notify_all();
  if (worker_.joinable()) worker_.join();
  if (inference_worker_.joinable()) inference_worker_.join();
}

void SmartTrackerFusionCpp::imageCallback(const sensor_msgs::ImageConstPtr &image) {
  {
    std::lock_guard<std::mutex> lock(image_mutex_);
    pending_image_ = image;
  }
  image_cv_.notify_one();
}

std::string SmartTrackerFusionCpp::className(int class_id) const {
  std::lock_guard<std::mutex> lock(labels_mutex_);
  const auto found = labels_.find(class_id);
  return found == labels_.end() ? "unknown" : found->second;
}

std::string SmartTrackerFusionCpp::updateLossJournal(const TrackResult &selected, double stamp) {
  auto quote = [](const std::string &value) {
    std::ostringstream escaped;
    escaped << '"';
    for (unsigned char ch : value) {
      if (ch == '"' || ch == '\\') escaped << '\\' << ch;
      else if (ch == '\n') escaped << "\\n";
      else if (ch == '\r') escaped << "\\r";
      else if (ch == '\t') escaped << "\\t";
      else if (ch >= 0x20) escaped << ch;
    }
    escaped << '"';
    return escaped.str();
  };
  auto sample = [&](const TrackResult &value, double time) {
    std::ostringstream json;
    json << "{\"stamp\":" << time << ",\"selected_id\":" << value.public_id
         << ",\"local_id\":" << value.local_id << ",\"source\":" << quote(value.source)
         << ",\"reason\":" << quote(value.reason) << ",\"observed\":"
         << (value.observed ? "true" : "false") << '}';
    return json.str();
  };
  auto appendEvent = [&](bool recovered, const std::string &end_reason) {
    std::ostringstream event;
    event << "{\"event_id\":" << loss_summary_event_id_
          << ",\"selected_id\":" << loss_event_selected_id_
          << ",\"start_stamp\":" << loss_event_start_stamp_
          << ",\"first_reason\":" << quote(loss_first_reason_)
          << ",\"last_stamp\":" << loss_event_last_stamp_
          << ",\"last_reason\":" << quote(loss_last_reason_)
          << ",\"duration_sec\":" << loss_summary_duration_
          << ",\"recovered\":" << (recovered ? "true" : "false")
          << ",\"end_reason\":" << quote(end_reason)
          << ",\"recovered_local_id\":" << (recovered ? loss_event_last_local_id_ : -1)
          << ",\"before\":[";
    for (size_t i = 0; i < loss_context_.size(); ++i) {
      if (i) event << ',';
      event << loss_context_[i];
    }
    event << "],\"after\":[";
    for (size_t i = 0; i < active_loss_after_.size(); ++i) {
      if (i) event << ',';
      event << active_loss_after_[i];
    }
    event << "]}";
    recent_loss_events_.push_back(event.str());
    while (recent_loss_events_.size() > 32) recent_loss_events_.pop_front();
    loss_summary_reason_ = loss_first_reason_;
    loss_finalized_stamp_ = loss_event_last_stamp_;
    loss_active_ = false;
    active_loss_after_.clear();
  };

  bool revision_changed = false;
  if (selected.public_id != loss_event_selected_id_) {
    if (loss_active_) {
      loss_summary_duration_ = std::max(0.0, loss_event_last_stamp_ - loss_event_start_stamp_);
      appendEvent(false, "selection_or_timeline_changed");
      ++loss_interrupted_count_;
      revision_changed = true;
    }
    loss_context_.clear();
    loss_event_selected_id_ = selected.public_id;
  }
  const std::string current = sample(selected, stamp);
  if (selected.public_id >= 0) {
    if (!selected.observed && !loss_active_) {
      ++loss_event_count_;
      loss_summary_event_id_ = loss_event_count_;
      loss_event_selected_id_ = selected.public_id;
      loss_event_start_stamp_ = stamp;
      loss_event_last_stamp_ = stamp;
      loss_event_last_local_id_ = selected.local_id;
      loss_first_reason_ = selected.reason;
      loss_last_reason_ = selected.reason;
      loss_active_ = true;
      active_loss_after_.clear();
      revision_changed = true;
    }
    if (loss_active_) {
      loss_event_last_stamp_ = stamp;
      loss_event_last_local_id_ = selected.local_id;
      loss_last_reason_ = selected.reason;
      loss_summary_duration_ = std::max(0.0, stamp - loss_event_start_stamp_);
      if (active_loss_after_.size() < 12) active_loss_after_.push_back(current);
      if (selected.observed) {
        appendEvent(true, "trusted_observation");
        ++loss_recovered_count_;
        revision_changed = true;
      }
    }
  }
  loss_context_.push_back(current);
  while (loss_context_.size() > 12) loss_context_.pop_front();

  const double wall_now = std::chrono::duration<double>(SteadyClock::now().time_since_epoch()).count();
  const uint64_t revision = loss_event_count_ + loss_recovered_count_ + loss_interrupted_count_;
  if ((revision_changed || (loss_active_ && wall_now - last_loss_publish_wall_ >= 1.0)) &&
      (revision != loss_published_revision_ || loss_active_)) {
    std::ostringstream snapshot;
    snapshot << "{\"loss_events\":" << loss_event_count_
        << ",\"recovered_events\":" << loss_recovered_count_
        << ",\"interrupted_events\":" << loss_interrupted_count_
        << ",\"active\":";
    if (loss_active_) {
      snapshot << "{\"event_id\":" << loss_summary_event_id_
          << ",\"selected_id\":" << loss_event_selected_id_
          << ",\"start_stamp\":" << loss_event_start_stamp_
          << ",\"first_reason\":" << quote(loss_first_reason_)
          << ",\"last_stamp\":" << loss_event_last_stamp_
          << ",\"last_reason\":" << quote(loss_last_reason_)
          << ",\"duration_sec\":" << loss_summary_duration_
          << ",\"recovered\":false,\"before\":[";
      for (size_t i = 0; i < loss_context_.size(); ++i) {
        if (i) snapshot << ',';
        snapshot << loss_context_[i];
      }
      snapshot << "],\"after\":[";
      for (size_t i = 0; i < active_loss_after_.size(); ++i) {
        if (i) snapshot << ',';
        snapshot << active_loss_after_[i];
      }
      snapshot << "]}";
    } else snapshot << "null";
    snapshot << ",\"recent_events\":[";
    const size_t begin = recent_loss_events_.size() > 2 ? recent_loss_events_.size() - 2 : 0;
    for (size_t i = begin; i < recent_loss_events_.size(); ++i) {
      if (i > begin) snapshot << ',';
      snapshot << recent_loss_events_[i];
    }
    snapshot << "],\"finalized_through_stamp\":" << loss_finalized_stamp_ << '}';
    std_msgs::String message;
    message.data = snapshot.str();
    loss_pub_.publish(message);
    loss_published_revision_ = revision;
    last_loss_publish_wall_ = wall_now;
  }

  std::ostringstream summary;
  summary << "{\"loss_events\":" << loss_event_count_
      << ",\"recovered_events\":" << loss_recovered_count_
      << ",\"active\":" << (loss_active_ ? "true" : "false")
      << ",\"event_id\":" << (loss_summary_event_id_ ? std::to_string(loss_summary_event_id_) : "null")
      << ",\"first_reason\":" << (loss_summary_event_id_ ? quote(loss_active_ ? loss_first_reason_ : loss_summary_reason_) : "null")
      << ",\"duration_sec\":" << loss_summary_duration_
      << ",\"finalized_through_stamp\":" << loss_finalized_stamp_ << '}';
  return summary.str();
}

void SmartTrackerFusionCpp::processLoop() {
  while (ros::ok() && !stopping_) {
    sensor_msgs::ImageConstPtr image;
    {
      std::unique_lock<std::mutex> lock(image_mutex_);
      image_cv_.wait(lock, [this]() { return stopping_ || pending_image_; });
      if (stopping_) break;
      image.swap(pending_image_);
    }
    if (!image) continue;
    try {
      process(image);
    } catch (const std::exception &error) {
      std::lock_guard<std::mutex> lock(state_mutex_);
      last_error_ = error.what();
      ROS_ERROR_THROTTLE(2.0, "C++ SmartTracker frame failed: %s", error.what());
    }
  }
}

void SmartTrackerFusionCpp::enqueueInference(const cv::Mat &frame,
    const std_msgs::Header &header, uint64_t mode_epoch) {
  if (frame.empty()) return;
  {
    std::lock_guard<std::mutex> lock(inference_mutex_);
    // Keep only the newest waiting frame. The active service call cannot be
    // interrupted, so queuing older frames would only increase result age.
    pending_inference_job_.frame = frame;
    pending_inference_job_.header = header;
    pending_inference_job_.mode_epoch = mode_epoch;
    has_pending_inference_job_ = true;
  }
  inference_cv_.notify_one();
}

bool SmartTrackerFusionCpp::takeInferenceResult(double frame_stamp,
    uint64_t mode_epoch, InferenceResult *result) {
  if (result == nullptr) return false;
  std::lock_guard<std::mutex> lock(inference_result_mutex_);
  if (latest_inference_result_.generation == 0 ||
      latest_inference_result_.generation <= consumed_inference_generation_) return false;

  const InferenceResult &latest = latest_inference_result_;
  if (latest.mode_epoch != mode_epoch) {
    consumed_inference_generation_ = latest.generation;
    return false;
  }
  // Do not apply a detection to a frame that predates the inference input.
  // The next camera frame will consume it once its timestamp catches up.
  if (latest.stamp > frame_stamp) return false;
  if (maximum_age_sec_ > 0.0 && frame_stamp - latest.stamp > maximum_age_sec_) {
    consumed_inference_generation_ = latest.generation;
    return false;
  }

  *result = latest;
  consumed_inference_generation_ = latest.generation;
  return true;
}

void SmartTrackerFusionCpp::inferenceLoop() {
  while (ros::ok() && !stopping_) {
    InferenceJob job;
    {
      std::unique_lock<std::mutex> lock(inference_mutex_);
      inference_cv_.wait(lock, [this]() { return stopping_ || has_pending_inference_job_; });
      if (stopping_) break;
      job = std::move(pending_inference_job_);
      pending_inference_job_ = InferenceJob();
      has_pending_inference_job_ = false;
    }
    if (job.frame.empty()) continue;

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (mode_ != "fusion_tracker" || mode_epoch_ != job.mode_epoch) continue;
    }

    try {
      std::vector<Detection> detections;
      double inference_ms = 0.0;
      const auto started = SteadyClock::now();
      const bool succeeded = requestDetections(job.frame, job.header, &detections);
      if (!succeeded) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (mode_epoch_ == job.mode_epoch) ++inference_failures_;
        continue;
      }
      inference_ms = std::chrono::duration<double, std::milli>(
          SteadyClock::now() - started).count();

      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (mode_ != "fusion_tracker" || mode_epoch_ != job.mode_epoch) continue;
      }
      {
        std::lock_guard<std::mutex> lock(inference_result_mutex_);
        latest_inference_result_.generation = ++next_inference_generation_;
        latest_inference_result_.mode_epoch = job.mode_epoch;
        latest_inference_result_.stamp = job.header.stamp.toSec();
        latest_inference_result_.inference_ms = inference_ms;
        latest_inference_result_.detections = std::move(detections);
      }
    } catch (const std::exception &error) {
      std::lock_guard<std::mutex> lock(state_mutex_);
      last_error_ = error.what();
      if (mode_epoch_ == job.mode_epoch) ++inference_failures_;
      ROS_ERROR_THROTTLE(2.0, "C++ SmartTracker inference worker failed: %s", error.what());
    }
  }
}

bool SmartTrackerFusionCpp::requestDetections(const cv::Mat &frame,
    const std_msgs::Header &header, std::vector<Detection> *detections) {
  if (frame.empty() || detections == nullptr) return false;
  if (!model_info_loaded_) {
    if (info_client_.waitForExistence(ros::Duration(0.25))) {
      sar_yolo_detector::GetSharedModelInfo info;
      info.request.model_key = model_key_;
      if (info_client_.call(info) && info.response.success) {
        try {
          YAML::Node labels = YAML::Load(info.response.labels_json);
          std::unordered_map<int, std::string> parsed_labels;
          if (labels.IsMap()) {
            for (const auto &entry : labels) {
              parsed_labels[std::stoi(entry.first.as<std::string>())] =
                  entry.second.as<std::string>();
            }
          }
          std::lock_guard<std::mutex> lock(labels_mutex_);
          for (auto &entry : parsed_labels) labels_[entry.first] = std::move(entry.second);
        } catch (const std::exception &error) {
          ROS_WARN_STREAM_THROTTLE(5.0, "Could not parse shared-model labels: " << error.what());
        }
        model_info_loaded_ = true;
      }
    }
  }
  bool service_exists = false;
  {
    std::lock_guard<std::mutex> lock(inference_client_mutex_);
    service_exists = inference_client_.waitForExistence(ros::Duration(0.5));
  }
  if (!service_exists) {
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      last_error_ = "shared TensorRT inference service is unavailable";
    }
    ROS_WARN_STREAM_THROTTLE(5.0,
        "Shared YOLO inference request skipped: service is unavailable at "
        << inference_base_ << "/detect");
    return false;
  }

  const int source_width = frame.cols;
  const int source_height = frame.rows;
  cv::Mat input = frame;
  const int max_side = inference_image_size_;
  if (std::max(frame.cols, frame.rows) > max_side) {
    const double scale = max_side / static_cast<double>(std::max(frame.cols, frame.rows));
    const cv::Size size(std::max(1, static_cast<int>(std::lround(frame.cols * scale))),
                        std::max(1, static_cast<int>(std::lround(frame.rows * scale))));
    cv::resize(frame, input, size, 0.0, 0.0, cv::INTER_AREA);
  }
  sar_yolo_detector::RunSharedInference call;
  call.request.model_key = model_key_;
  call.request.source_id = source_id_;
  call.request.image.header = header;
  call.request.image.height = input.rows;
  call.request.image.width = input.cols;
  call.request.image.encoding = "bgr8";
  call.request.image.is_bigendian = 0;
  call.request.image.step = static_cast<uint32_t>(input.cols * 3);
  cv::Mat packed = input.isContinuous() ? input : input.clone();
  call.request.image.data.assign(packed.datastart, packed.dataend);
  call.request.confidence = static_cast<float>(confidence_threshold_);
  call.request.iou = static_cast<float>(iou_threshold_);
  call.request.max_detections = static_cast<uint32_t>(max_detections_);
  call.request.image_size = inference_image_size_;
  const auto started = SteadyClock::now();
  bool service_call_succeeded = false;
  {
    std::lock_guard<std::mutex> lock(inference_client_mutex_);
    service_call_succeeded = inference_client_.call(call);
  }
  if (!service_call_succeeded || !call.response.success) {
    const std::string message = !service_call_succeeded
        ? "ROS service transport call failed at " + inference_base_ + "/detect"
        : (call.response.message.empty()
            ? "shared TensorRT provider rejected the inference request without a reason"
            : call.response.message);
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      last_error_ = message;
    }
    if (!service_call_succeeded ||
        message.find("model key mismatch") != std::string::npos) {
      std::lock_guard<std::mutex> lock(inference_client_mutex_);
      inference_client_.shutdown();
      inference_client_ = nh_.serviceClient<sar_yolo_detector::RunSharedInference>(
          inference_base_ + "/detect", true);
    }
    ROS_WARN_STREAM_THROTTLE(5.0,
        "Shared YOLO inference request failed: " << message);
    return false;
  }
  const double service_ms = std::chrono::duration<double, std::milli>(SteadyClock::now() - started).count();
  const double scale_x = source_width / static_cast<double>(input.cols);
  const double scale_y = source_height / static_cast<double>(input.rows);
  const size_t raw_detection_count = call.response.detections.size();
  size_t class_filtered_count = 0;
  detections->clear();
  detections->reserve(call.response.detections.size());
  for (const auto &raw : call.response.detections) {
    if (!allowed_class_ids_.empty() &&
        std::find(allowed_class_ids_.begin(), allowed_class_ids_.end(), raw.class_id) == allowed_class_ids_.end()) {
      ++class_filtered_count;
      continue;
    }
    Detection detection;
    detection.id = raw.track_id;
    detection.class_id = raw.class_id;
    detection.confidence = raw.confidence;
    detection.box = {raw.aabb_xyxy[0] * scale_x, raw.aabb_xyxy[1] * scale_y,
        (raw.aabb_xyxy[2] - raw.aabb_xyxy[0]) * scale_x,
        (raw.aabb_xyxy[3] - raw.aabb_xyxy[1]) * scale_y};
    detection.stable = raw.track_id_is_stable;
    detection.geometry = raw.geometry_type;
    detection.has_rotation = raw.has_rotation;
    detection.rotation_deg = raw.rotation_deg;
    detection.detector_stamp = header.stamp.toSec();
    for (size_t i = 0; i < detection.obb.size(); ++i) detection.obb[i] = raw.obb_xywhr[i];
    detection.obb[0] *= scale_x;
    detection.obb[1] *= scale_y;
    detection.obb[2] *= scale_x;
    detection.obb[3] *= scale_y;
    detection.polygon.reserve(raw.polygon_xy.size());
    for (size_t i = 0; i + 1 < raw.polygon_xy.size(); i += 2) {
      detection.polygon.push_back(raw.polygon_xy[i] * scale_x);
      detection.polygon.push_back(raw.polygon_xy[i + 1] * scale_y);
    }
    if (validBox(detection.box, frame.size())) detections->push_back(std::move(detection));
  }
  ROS_INFO_STREAM_THROTTLE(5.0,
      "Shared YOLO inference succeeded: model_results=" << raw_detection_count
      << " accepted=" << detections->size()
      << " class_filtered=" << class_filtered_count
      << " service_ms=" << std::fixed << std::setprecision(1) << service_ms
      << " provider_ms=" << call.response.inference_ms);
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    last_error_.clear();
  }
  return true;
}

void SmartTrackerFusionCpp::process(const sensor_msgs::ImageConstPtr &image) {
  if (!image) return;
  const double stamp = image->header.stamp.toSec();
  if (!std::isfinite(stamp) || stamp <= 0.0) {
    ROS_WARN_THROTTLE(5.0, "SmartTracker skipped image with invalid/zero header stamp");
    return;
  }
  const double now_ros = ros::Time::now().toSec();
  if (now_ros > 0.0) {
    const double age = now_ros - stamp;
    if (age < -future_tolerance_sec_) {
      ROS_WARN_STREAM_THROTTLE(5.0, "SmartTracker skipped future-dated image: age="
          << std::fixed << std::setprecision(3) << age << " s");
      return;
    }
    if (maximum_age_sec_ > 0.0 && age > maximum_age_sec_) {
      ROS_WARN_STREAM_THROTTLE(5.0, "SmartTracker skipped stale image: age="
          << std::fixed << std::setprecision(3) << age << " s, limit="
          << maximum_age_sec_ << " s");
      return;
    }
  }
  cv::Mat frame = imageToBgr(*image);
  const double wall_now = std::chrono::duration<double>(SteadyClock::now().time_since_epoch()).count();
  bool run_inference = false;
  std::string frame_mode;
  uint64_t frame_mode_epoch = 0;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (stamp <= last_capture_stamp_) {
      if (last_capture_stamp_ - stamp < 0.5) {
        ROS_WARN_THROTTLE(5.0, "SmartTracker skipped duplicate/out-of-order image timestamp");
        return;
      }
      resetSelection();
      template_detections_.reset();
      snapshots_.clear();
      snapshot_bytes_ = 0;
      camera_motion_.reset();
      latest_frame_.release();
      previous_frame_.release();
      last_detector_detections_.clear();
      last_detector_stamp_ = 0.0;
      last_capture_stamp_ = 0.0;
      last_inference_wall_ = 0.0;
      ++mode_epoch_;
      ROS_WARN("Capture timestamp moved backwards; starting a new tracker segment");
    }
    if (!latest_frame_.empty() && (latest_frame_.size() != frame.size() ||
                                   latest_header_.frame_id != image->header.frame_id)) {
      resetSelection();
      template_detections_.reset();
      snapshots_.clear();
      snapshot_bytes_ = 0;
      camera_motion_.reset();
      last_detector_detections_.clear();
      last_detector_stamp_ = 0.0;
      previous_frame_.release();
      ++mode_epoch_;
    }
    last_capture_stamp_ = stamp;
    frame_mode = mode_;
    frame_mode_epoch = mode_epoch_;
    run_inference = frame_mode == "fusion_tracker" &&
        (inference_max_fps_ <= 0.0 || last_inference_wall_ == 0.0 ||
         wall_now - last_inference_wall_ >= 1.0 / inference_max_fps_);
    if (run_inference) last_inference_wall_ = wall_now;
  }

  if (run_inference) enqueueInference(frame, image->header, frame_mode_epoch);

  std::vector<Detection> new_detections;
  double inference_ms = 0.0;
  double inference_result_stamp = 0.0;
  InferenceResult inference_result;
  const bool inference_succeeded = takeInferenceResult(stamp, frame_mode_epoch,
                                                         &inference_result);
  if (inference_succeeded) {
    new_detections = std::move(inference_result.detections);
    inference_ms = inference_result.inference_ms;
    inference_result_stamp = inference_result.stamp;
  }

  const auto processing_started = SteadyClock::now();
  std::vector<Detection> current_detections;
  bool fresh = false;
  cv::Mat motion;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    const bool inference_result_current = mode_ == frame_mode && mode_epoch_ == frame_mode_epoch;
    if (!inference_result_current) {
      new_detections.clear();
      inference_ms = 0.0;
    }
    if (inference_succeeded && inference_result_current) {
      // The shared C++ TensorRT provider returns frame-local negative IDs.
      // Associate new YOLO boxes to the latest detector frame. For the selected
      // target only, also retain a short ID memory across successful empty
      // detector frames. The memory is used only for ID association; empty
      // detector results still clear active boxes and never refresh their age.
      std::vector<bool> previous_used(last_detector_detections_.size(), false);
      bool selected_identity_reused = false;
      for (auto &detection : new_detections) {
        if (detection.stable && detection.id >= 0) continue;
        size_t best_index = last_detector_detections_.size();
        double best_iou = 0.15;
        for (size_t i = 0; i < last_detector_detections_.size(); ++i) {
          const Detection &previous = last_detector_detections_[i];
          if (previous_used[i] || !previous.stable || previous.id < 0 ||
              previous.class_id != detection.class_id) continue;
          const double overlap = boxIou(previous.box, detection.box);
          if (overlap > best_iou) {
            best_iou = overlap;
            best_index = i;
          }
        }
        if (best_index < last_detector_detections_.size()) {
          detection.id = last_detector_detections_[best_index].id;
          previous_used[best_index] = true;
        } else {
          bool matched_selected_memory = false;
          const double selected_detection_age = inference_result_stamp - last_detector_for_selection_;
          if (!selected_ && !selected_identity_reused && selected_local_id_ >= 0 &&
              last_detector_for_selection_ > 0.0 && selected_detection_age >= 0.0 &&
              selected_detection_age <= detector_id_memory_sec_ &&
              detection.confidence >= min_maintenance_confidence_) {
            bool same_class = selected_class_id_ < 0 || detection.class_id == selected_class_id_;
            if (!same_class) {
              const auto selected_group = class_group_by_id_.find(selected_class_id_);
              const auto detection_group = class_group_by_id_.find(detection.class_id);
              same_class = selected_group != class_group_by_id_.end() &&
                  detection_group != class_group_by_id_.end() &&
                  selected_group->second == detection_group->second;
            }

            const double prediction_dt = std::min(max_frame_gap_sec_,
                std::max(0.0, inference_result_stamp - last_observation_stamp_));
            Box selected_prediction = selected_box_;
            selected_prediction.x += velocity_.x * prediction_dt;
            selected_prediction.y += velocity_.y * prediction_dt;
            const double area_ratio = detection.box.w * detection.box.h /
                std::max(1.0, selected_prediction.w * selected_prediction.h);
            const double center_distance = cv::norm(center(detection.box) - center(selected_prediction)) /
                std::max(8.0, std::hypot(selected_prediction.w, selected_prediction.h));
            const bool geometry_ok = same_class && validBox(selected_prediction, frame.size()) &&
                area_ratio >= 1.0 / max_area_ratio_ && area_ratio <= max_area_ratio_ &&
                center_distance <= center_gate_sizes_ && validBox(detection.box, frame.size());
            if (geometry_ok) {
              double quality = 0.0;
              const auto feature = appearanceFeature(frame, detection.box, &quality);
              const double similarity = appearance_anchor_.empty()
                  ? 1.0 : cosineSimilarity(feature, appearance_anchor_);
              if (quality >= quality_gate_ && similarity >= appearance_gate_) {
                detection.id = selected_local_id_;
                detection.selected_identity_reacquire = true;
                matched_selected_memory = true;
                selected_identity_reused = true;
              }
            }
          }
          if (!matched_selected_memory) detection.id = next_detector_id_++;
        }
        detection.stable = true;
      }
      last_detector_detections_ = std::move(new_detections);
      last_detector_stamp_ = inference_result_stamp;
      fresh = true;
    }
    if (mode_ == "fusion_tracker") {
      current_detections = last_detector_detections_;
      for (auto &detection : current_detections) detection.detector_stamp = last_detector_stamp_;
      std::vector<Box> motion_boxes;
      for (const auto &detection : current_detections) motion_boxes.push_back(detection.box);
      if (selected_ && result_.has_box) motion_boxes.push_back(result_.box);
      const cv::Mat pose = motion_boxes.empty() ? cv::Mat() : poseRotation(*image);
      motion = camera_motion_.update(frame, stamp, motion_boxes, pose);
    } else {
      current_detections.clear();
      motion.release();
    }

    if (selected_ && mode_ == "fusion_tracker" && !motion.empty() &&
        validateHomography(motion, frame.size())) {
      const cv::Point2d old_center = center(selected_box_);
      std::vector<cv::Point2f> velocity_points{{static_cast<float>(old_center.x), static_cast<float>(old_center.y)},
          {static_cast<float>(old_center.x + velocity_.x * 0.01),
           static_cast<float>(old_center.y + velocity_.y * 0.01)}};
      std::vector<cv::Point2f> projected_velocity;
      cv::perspectiveTransform(velocity_points, projected_velocity, motion);
      if (projected_velocity.size() == 2) {
        velocity_ = cv::Point2d((projected_velocity[1].x - projected_velocity[0].x) / 0.01,
                                (projected_velocity[1].y - projected_velocity[0].y) / 0.01);
      }
      const double old_area = std::max(1.0, selected_box_.w * selected_box_.h);
      std::vector<cv::Point2f> old_corners{{static_cast<float>(selected_box_.x), static_cast<float>(selected_box_.y)},
          {static_cast<float>(selected_box_.x + selected_box_.w), static_cast<float>(selected_box_.y)},
          {static_cast<float>(selected_box_.x + selected_box_.w), static_cast<float>(selected_box_.y + selected_box_.h)},
          {static_cast<float>(selected_box_.x), static_cast<float>(selected_box_.y + selected_box_.h)}};
      std::vector<cv::Point2f> projected_corners;
      cv::perspectiveTransform(old_corners, projected_corners, motion);
      if (projected_corners.size() == 4) {
        anchor_area_ *= std::abs(cv::contourArea(projected_corners)) / old_area;
      }
      const Box transformed = transformBox(selected_box_, motion);
      if (validBox(transformed, frame.size())) {
        if (pending_reacquire_count_ > 0)
          pending_reacquire_box_ = transformBox(pending_reacquire_box_, motion);
        selected_box_ = transformed;
      }
    }

    // A YOLO box may refresh the selected visual template only when it
    // overlaps the locked template box in the same image frame. It never
    // replaces the selected result box or adds another overlay box.
    if (fresh && selected_ && mode_ == "fusion_tracker" &&
        !target_template_gray_.empty() && !current_detections.empty() &&
        stamp - last_template_update_stamp_ >= local_template_refresh_sec_) {
      const cv::Mat *detector_frame = nullptr;
      Box locked_box_at_detection = selected_box_;
      if (std::abs(inference_result_stamp - stamp) <= 1e-6) {
        detector_frame = &frame;
      } else {
        for (const auto &snapshot : snapshots_) {
          if (snapshot.image &&
              std::abs(snapshot.image->header.stamp.toSec() - inference_result_stamp) <= 1e-6) {
            detector_frame = &snapshot.frame;
            if (snapshot.has_selected_box) locked_box_at_detection = snapshot.selected_box;
            break;
          }
        }
      }
      if (detector_frame) {
        const Detection *overlapping_detection = nullptr;
        double greatest_overlap = 0.0;
        for (const auto &detection : current_detections) {
          const double overlap = boxIou(detection.box, locked_box_at_detection);
          if (overlap > greatest_overlap) {
            greatest_overlap = overlap;
            overlapping_detection = &detection;
          }
        }
        if (overlapping_detection) {
          cv::Mat detector_template = grayTemplate(*detector_frame, overlapping_detection->box);
          if (!detector_template.empty()) {
            // Let the selected template's native size follow the overlapping
            // detector box gradually, preventing scale drift from shrinking
            // the tracked box while keeping each update visually smooth.
            const double size_gain = 0.25;
            const cv::Size updated_size(
                std::max(4, static_cast<int>(std::lround(target_template_gray_.cols +
                    size_gain * (detector_template.cols - target_template_gray_.cols)))),
                std::max(4, static_cast<int>(std::lround(target_template_gray_.rows +
                    size_gain * (detector_template.rows - target_template_gray_.rows)))));
            cv::Mat previous_template, aligned_detection, updated_template;
            cv::resize(target_template_gray_, previous_template, updated_size);
            cv::resize(detector_template, aligned_detection, updated_size);
            cv::addWeighted(previous_template, 0.75, aligned_detection, 0.25,
                            0.0, updated_template);
            target_template_gray_ = std::move(updated_template);

            double quality = 0.0;
            const std::vector<float> detection_feature =
                appearanceFeature(*detector_frame, overlapping_detection->box, &quality);
            if (detection_feature.size() == appearance_anchor_.size()) {
              for (size_t i = 0; i < appearance_anchor_.size(); ++i)
                appearance_anchor_[i] = 0.80f * appearance_anchor_[i] +
                    0.20f * detection_feature[i];
              const double norm = std::sqrt(std::inner_product(appearance_anchor_.begin(),
                  appearance_anchor_.end(), appearance_anchor_.begin(), 0.0));
              if (norm > 1e-9)
                for (float &item : appearance_anchor_)
                  item = static_cast<float>(item / norm);
            }
            last_template_update_stamp_ = stamp;
          }
        }
      }
    }

    if (mode_ == "fusion_tracker") {
      const std::vector<Detection> no_yolo_updates;
      processFusion(frame, stamp, no_yolo_updates, false, motion, nullptr);
    } else if (mode_ == "feature_tracker" && has_pending_roi_) {
      has_pending_roi_ = false;
      resetSelection();
      selected_ = true;
      manual_active_ = true;
      manual_seed_ = false;
      selected_id_ = manual_track_id_;
      selected_local_id_ = -1;
      selected_class_id_ = color_class_id_;
      selected_box_ = pending_roi_;
      selected_confidence_ = 0.5;
      selected_stamp_ = stamp;
      last_fusion_stamp_ = stamp;
      last_observation_stamp_ = stamp;
      double quality = 0.0;
      appearance_anchor_ = appearanceFeature(frame, selected_box_, &quality);
      anchor_area_ = selected_box_.w * selected_box_.h;
      try {
        kcf_ = cv::TrackerKCF::create();
        kcf_->init(frame, cv::Rect(static_cast<int>(std::lround(selected_box_.x)),
            static_cast<int>(std::lround(selected_box_.y)), static_cast<int>(std::lround(selected_box_.w)),
            static_cast<int>(std::lround(selected_box_.h))));
        result_ = {selected_box_, selected_id_, -1, selected_class_id_, selected_confidence_,
            0.0, "visual", "initialized", true, true};
      } catch (const cv::Exception &) {
        resetSelection();
        result_.source = "lost";
        result_.reason = "feature_tracker_initialization_failed";
      }
    } else if (mode_ == "feature_tracker" && manual_active_ && kcf_) {
      cv::Rect tracked(static_cast<int>(std::lround(selected_box_.x)), static_cast<int>(std::lround(selected_box_.y)),
                       static_cast<int>(std::lround(selected_box_.w)), static_cast<int>(std::lround(selected_box_.h)));
      bool ok = false;
      try { ok = kcf_->update(frame, tracked); } catch (const cv::Exception &) { ok = false; }
      if (ok) {
        selected_box_ = clipBox({static_cast<double>(tracked.x), static_cast<double>(tracked.y),
                                 static_cast<double>(tracked.width), static_cast<double>(tracked.height)}, frame.size());
        result_ = {selected_box_, selected_id_, selected_local_id_, selected_class_id_, selected_confidence_,
                   last_detector_for_selection_, "visual", "feature_tracker", true, true};
      } else {
        result_.source = "lost";
        result_.reason = "feature_tracker_failed";
        result_.has_box = false;
        result_.observed = false;
      }
    } else if (mode_ == "color_tracker") {
      processColor(frame, stamp, current_detections);
    } else {
      result_ = TrackResult();
    }

    const bool auto_target_unavailable = !result_.has_box &&
        (result_.source == "lost" || result_.source == "ambiguous");
    const bool auto_loss_monitoring = auto_select_enabled_ && auto_selected_ && selected_ &&
        (mode_ == "fusion_tracker" || mode_ == "color_tracker");
    if (auto_loss_monitoring && auto_target_unavailable) {
      if (auto_selected_missing_since_ <= 0.0) auto_selected_missing_since_ = stamp;
      const double missing_duration = stamp - auto_selected_missing_since_;
      if (missing_duration >= auto_select_lost_hold_sec_) {
        ROS_WARN_STREAM("C++ SmartTracker auto-selected target has had no valid box for "
            << std::fixed << std::setprecision(1) << missing_duration
            << " s; releasing its lock and resuming automatic selection");
        resetSelection();
        result_.source = "lost";
        result_.reason = "auto_select_target_lost";
        auto_select_count_ = 0;
        auto_select_candidate_id_ = -1;
        auto_select_candidate_class_ = -1;
        auto_select_candidate_stable_ = false;
      }
    } else {
      auto_selected_missing_since_ = 0.0;
    }

    latest_frame_ = frame;
    previous_frame_ = frame;
    latest_header_ = image->header;
    latest_detections_ = current_detections;
    snapshots_.push_back({image, frame, current_detections, selected_box_, selected_});
    snapshot_bytes_ += static_cast<size_t>(frame.total() * frame.elemSize());
    while (!snapshots_.empty() && (snapshots_.size() > static_cast<size_t>(snapshot_max_frames_) ||
           image->header.stamp.toSec() - snapshots_.front().image->header.stamp.toSec() > snapshot_max_age_ ||
           snapshot_bytes_ > snapshot_max_bytes_)) {
      snapshot_bytes_ -= static_cast<size_t>(snapshots_.front().frame.total() * snapshots_.front().frame.elemSize());
      snapshots_.pop_front();
    }
    if ((fresh || mode_ == "color_tracker") && auto_select_enabled_ && !selected_) {
      if (mode_ == "fusion_tracker" && fresh) {
        const std::vector<Detection> fresh_candidates = last_detector_detections_;
        updateAutoSelection(fresh_candidates, frame, image);
      } else {
        updateAutoSelection(current_detections, frame, image);
      }
    }
  }

  const double processing_ms = std::chrono::duration<double, std::milli>(SteadyClock::now() - processing_started).count();
  publishOutputs(image, frame, current_detections, fresh, inference_ms, processing_ms);
}

void SmartTrackerFusionCpp::processFusion(const cv::Mat &frame, double stamp,
    const std::vector<Detection> &detections, bool detector_updated,
    const cv::Mat &camera_transform, const Detection *visual_observation) {
  result_.public_id = selected_id_;
  result_.local_id = selected_local_id_;
  result_.class_id = selected_class_id_;
  result_.detector_stamp = last_detector_for_selection_;
  if (!selected_) {
    result_ = TrackResult();
    return;
  }

  const double step = stamp - last_fusion_stamp_;
  if (!std::isfinite(step) || step <= 0.0) {
    result_.has_box = false;
    result_.observed = false;
    result_.source = "lost";
    result_.reason = "non_increasing_capture";
    result_.confidence = 0.0;
    return;
  }
  last_fusion_stamp_ = stamp;
  const double absence = std::max(0.0, stamp - last_observation_stamp_);
  long_recovery_ = absence > reacquire_hold_sec_;
  if (identity_ttl_sec_ > 0.0 && absence > identity_ttl_sec_) {
    pending_reacquire_id_ = -1;
    pending_reacquire_count_ = 0;
    appearance_anchor_.clear();
    result_.has_box = false;
    result_.observed = false;
    result_.source = "lost";
    result_.reason = "identity_memory_expired";
    result_.confidence = 0.0;
    return;
  }
  const bool motion_valid = validateHomography(camera_transform, frame.size());
  const bool motion_reliable = motion_valid &&
      (camera_motion_.source() == "background" || camera_motion_.source() == "pose_seeded_background");
  if (!motion_reliable || step > max_frame_gap_sec_) motion_continuous_ = false;

  Box predicted = selected_box_;
  const double horizon = std::min(absence, motion_horizon_sec_);
  predicted.x += velocity_.x * horizon;
  predicted.y += velocity_.y * horizon;
  const auto smooth_box = [](const Box &prior, const Box &measurement, double gain) {
    gain = clamp01(gain);
    return Box{prior.x + gain * (measurement.x - prior.x),
               prior.y + gain * (measurement.y - prior.y),
               prior.w + gain * (measurement.w - prior.w),
               prior.h + gain * (measurement.h - prior.h)};
  };
  const auto smooth_template_box = [](const Box &prior, const Box &measurement) {
    // Template scale estimates can jump between pyramid levels. Smooth size
    // more strongly than center so zoom changes remain responsive without
    // making the displayed box pulse from frame to frame.
    const double center_gain = 0.60;
    const double size_gain = 0.35;
    const cv::Point2d prior_center = center(prior);
    const cv::Point2d measured_center = center(measurement);
    const double cx = prior_center.x + center_gain * (measured_center.x - prior_center.x);
    const double cy = prior_center.y + center_gain * (measured_center.y - prior_center.y);
    const double width = prior.w + size_gain * (measurement.w - prior.w);
    const double height = prior.h + size_gain * (measurement.h - prior.h);
    return Box{cx - width * 0.5, cy - height * 0.5, width, height};
  };
  if (step > max_frame_gap_sec_ || long_recovery_) {
    if (step > max_frame_gap_sec_) {
      pending_reacquire_id_ = -1;
      pending_reacquire_count_ = 0;
    }
  }

  Box visual;
  bool visual_valid = false;
  double visual_quality = 0.0;
  double visual_template_quality = 0.0;
  double visual_template_similarity = 0.0;
  std::vector<float> visual_feature;
  // In fusion mode, only the selected target owns a template. Keep its local
  // search active every frame; expand to a global multi-scale search only
  // periodically after a prolonged loss.
  Box template_match_box;
  double template_match_score = 0.0;
  bool template_visual_valid = false;

  // If YOLO briefly returns the target under a new tracker ID, use that box as
  // a local search center for one template verification attempt.
  const Detection *rebound_template_candidate = nullptr;
  if (detector_updated) {
    for (const auto &candidate : detections) {
      if (!candidate.stable || candidate.id < 0 || candidate.id == selected_local_id_ ||
          !std::isfinite(candidate.confidence) || candidate.confidence < min_detector_confidence_ ||
          candidate.detector_stamp <= last_detector_for_selection_ || candidate.detector_stamp > stamp ||
          !validBox(candidate.box, frame.size())) continue;
      if (selected_class_id_ >= 0 && candidate.class_id != selected_class_id_) {
        const auto first = class_group_by_id_.find(selected_class_id_);
        const auto second = class_group_by_id_.find(candidate.class_id);
        if (first == class_group_by_id_.end() || second == class_group_by_id_.end() ||
            first->second != second->second) continue;
      }
      if (candidate.id != last_template_candidate_trigger_id_) {
        rebound_template_candidate = &candidate;
        break;
      }
    }
  }
  const bool multi_scale_due = last_template_search_stamp_ <= 0.0 ||
      stamp - last_template_search_stamp_ >= template_search_period_sec_;
  const bool primary_template_search_due = true;
  const double template_observation_ratio = visual_observation
      ? visual_observation->box.w * visual_observation->box.h /
          std::max(1.0, predicted.w * predicted.h) : 0.0;
  const double template_observation_distance = visual_observation
      ? cv::norm(center(visual_observation->box) - center(predicted)) /
          std::max(8.0, std::hypot(predicted.w, predicted.h)) : 0.0;
  const bool has_template_tracker_observation = visual_observation &&
      visual_observation->source == "template_tracker" && visual_observation->stable &&
      selected_local_id_ >= 0 && visual_observation->id == selected_local_id_ &&
      validBox(visual_observation->box, frame.size()) &&
      visual_observation->flow_quality >= template_match_threshold_ &&
      template_observation_ratio >= 1.0 / max_area_ratio_ &&
      template_observation_ratio <= max_area_ratio_ &&
      template_observation_distance <= center_gate_sizes_;
  cv::Mat frame_gray;
  if (!target_template_gray_.empty() && primary_template_search_due &&
      !has_template_tracker_observation)
    cv::cvtColor(frame, frame_gray, cv::COLOR_BGR2GRAY);

  const auto run_template_match = [&](const Box &search_center, bool global_search,
                                      bool multi_scale) {
    if (frame_gray.empty()) return false;
    Box match_box;
    double match_score = 0.0;
    if (!matchGrayTemplate(frame_gray, target_template_gray_, search_center, absence,
                           global_search, multi_scale, &match_box, &match_score)) return false;
    double template_quality = 0.0;
    auto template_feature = appearanceFeature(frame, match_box, &template_quality);
    const double template_similarity = appearance_anchor_.empty()
        ? 1.0 : cosineSimilarity(template_feature, appearance_anchor_);
    const Box &geometry_reference = search_center;
    const double area_ratio = match_box.w * match_box.h /
        std::max(1.0, geometry_reference.w * geometry_reference.h);
    const double center_distance = cv::norm(center(match_box) - center(geometry_reference)) /
        std::max(8.0, std::hypot(geometry_reference.w, geometry_reference.h));
    const double adaptive_center_gate = center_gate_sizes_ +
        std::min(2.0, absence * 0.20);
    const double score_gate = long_recovery_
        ? long_template_match_threshold_ : template_match_threshold_;
    const bool local_dynamic_match = !long_recovery_ &&
        absence <= max_frame_gap_sec_ &&
        match_score >= std::max(0.62, template_match_threshold_ + 0.14) &&
        center_distance <= std::min(0.75, adaptive_center_gate) &&
        area_ratio >= 1.0 / std::min(max_area_ratio_, 2.5) &&
        area_ratio <= std::min(max_area_ratio_, 2.5) &&
        template_similarity >= std::max(0.48, appearance_gate_ - 0.14);
    const bool geometry_ok = validBox(match_box, frame.size()) &&
        area_ratio >= 1.0 / max_area_ratio_ && area_ratio <= max_area_ratio_ &&
        (global_search || center_distance <= adaptive_center_gate);
    const bool appearance_ok = long_recovery_
        ? (template_similarity >= long_appearance_gate_ || match_score >= 0.84)
        : (template_similarity >= appearance_gate_ || local_dynamic_match);
    const bool quality_ok = template_quality >= quality_gate_ ||
        (!long_recovery_ && match_score >= 0.72 &&
         template_quality >= quality_gate_ * 0.60);
    if (match_score < score_gate || !geometry_ok || !appearance_ok ||
        !quality_ok) return false;
    template_match_box = match_box;
    template_match_score = match_score;
    template_visual_valid = true;
    visual_template_quality = template_quality;
    visual_template_similarity = template_similarity;
    visual = match_box;
    visual_quality = std::min(match_score, std::max(template_quality, quality_gate_));
    visual_feature = std::move(template_feature);
    visual_valid = true;
    return true;
  };

  if (!target_template_gray_.empty() && primary_template_search_due &&
      !has_template_tracker_observation) {
    const bool global_search = long_recovery_ && multi_scale_due && !rebound_template_candidate;
    const Box &search_center = rebound_template_candidate
        ? rebound_template_candidate->box : predicted;
    const bool multi_scale_search = multi_scale_due;
    if (multi_scale_search) last_template_search_stamp_ = stamp;
    run_template_match(search_center, global_search, multi_scale_search);
  }

  // A one-frame YOLO candidate may be far outside the current local search
  // window. Check its box directly against the saved target template once.
  if (!template_visual_valid && rebound_template_candidate &&
      !target_template_gray_.empty() && !has_template_tracker_observation) {
    if (frame_gray.empty()) cv::cvtColor(frame, frame_gray, cv::COLOR_BGR2GRAY);
    last_template_candidate_trigger_id_ = rebound_template_candidate->id;
    last_template_search_stamp_ = stamp;
    run_template_match(rebound_template_candidate->box, false, true);
  }

  const bool template_tracker_observation = visual_observation &&
      visual_observation->source == "template_tracker";
  if (visual_observation && (template_tracker_observation ||
      (!long_recovery_ && step <= max_frame_gap_sec_)) &&
      selected_local_id_ >= 0 && visual_observation->stable &&
      visual_observation->id == selected_local_id_ &&
      visual_observation->detector_stamp > 0.0 && visual_observation->detector_stamp <= stamp &&
      (template_tracker_observation ||
       stamp - visual_observation->detector_stamp <= max_frame_gap_sec_) &&
      visual_observation->flow_quality >=
          (template_tracker_observation ? template_match_threshold_ : 0.60) &&
      validBox(visual_observation->box, frame.size())) {
    const double distance = cv::norm(center(visual_observation->box) - center(predicted)) /
        std::max(8.0, std::hypot(predicted.w, predicted.h));
    const double ratio = visual_observation->flow_scale_ratio;
    bool flow_valid = distance <= center_gate_sizes_ &&
        ratio >= 1.0 / max_area_ratio_ && ratio <= max_area_ratio_;
    double flow_quality = 0.0;
    std::vector<float> flow_feature;
    if (flow_valid) {
      flow_feature = appearanceFeature(frame, visual_observation->box, &flow_quality);
      const bool small_patch = std::max(visual_observation->box.w,
          visual_observation->box.h) < 20.0;
      const double similarity = appearance_anchor_.empty()
          ? 1.0 : cosineSimilarity(flow_feature, appearance_anchor_);
      flow_valid = (flow_quality >= quality_gate_ || (template_tracker_observation && small_patch)) &&
          similarity >= appearance_gate_ - (template_tracker_observation && small_patch ? 0.10 : 0.0);
    }
    if (flow_valid) {
      if (visual_valid && boxIou(visual, visual_observation->box) < 0.15) {
        if (!template_visual_valid) visual_valid = false;
      } else {
        if (template_tracker_observation) {
          template_visual_valid = true;
          template_match_box = visual_observation->box;
          template_match_score = visual_observation->flow_quality;
        }
        visual = visual_observation->box;
        visual_quality = std::min(flow_quality, visual_observation->flow_quality);
        visual_feature = std::move(flow_feature);
        visual_valid = true;
      }
    }
  }

  std::vector<std::pair<double, const Detection *>> ranked;
  for (const auto &candidate : detections) {
    if (!detector_updated || candidate.detector_stamp <= last_detector_for_selection_ ||
        candidate.detector_stamp > stamp || !std::isfinite(candidate.confidence) ||
        candidate.confidence < min_maintenance_confidence_ || candidate.confidence > 1.0 ||
        !candidate.stable || candidate.id < 0 || !validBox(candidate.box, frame.size())) continue;
    bool template_support = false;
    if (template_visual_valid) {
      const double template_area_ratio = candidate.box.w * candidate.box.h /
          std::max(1.0, template_match_box.w * template_match_box.h);
      const double template_center_distance =
          cv::norm(center(candidate.box) - center(template_match_box)) /
          std::max(8.0, std::hypot(template_match_box.w, template_match_box.h));
      template_support = template_area_ratio >= 1.0 / max_area_ratio_ &&
          template_area_ratio <= max_area_ratio_ &&
          (boxIou(candidate.box, template_match_box) >= 0.10 ||
           template_center_distance <= 0.55) &&
          template_match_score >= (long_recovery_
              ? long_template_match_threshold_ : template_match_threshold_);
    }
    // A weak detection for the already selected stable ID can maintain the
    // track if it passes the same spatial and appearance gates below. Keep
    // the higher confidence threshold for acquiring or rebinding identities.
    const bool maintains_selected_id = candidate.id == selected_local_id_ &&
        selected_local_id_ >= 0;
    if (candidate.confidence < min_detector_confidence_ && !maintains_selected_id &&
        !template_support) continue;
    if (selected_class_id_ >= 0 && candidate.class_id != selected_class_id_) {
      const auto first = class_group_by_id_.find(selected_class_id_);
      const auto second = class_group_by_id_.find(candidate.class_id);
      if (first == class_group_by_id_.end() || second == class_group_by_id_.end() ||
          first->second != second->second) continue;
    }
    if (manual_seed_without_yolo_) {
      const cv::Point2d point = center(candidate.box);
      const bool inside = (point.x >= manual_seed_region_.x &&
          point.x <= manual_seed_region_.x + manual_seed_region_.w &&
          point.y >= manual_seed_region_.y &&
          point.y <= manual_seed_region_.y + manual_seed_region_.h) ||
          boxIou(candidate.box, manual_seed_region_) >= 0.15 ||
          (pending_reacquire_count_ > 0 && candidate.id == pending_reacquire_id_);
      if (!inside) continue;
    } else {
      const Box &reference = template_support ? template_match_box : predicted;
      const double ratio = candidate.box.w * candidate.box.h /
          std::max(1.0, reference.w * reference.h);
      const double distance = cv::norm(center(candidate.box) - center(reference)) /
          std::max(8.0, std::hypot(reference.w, reference.h));
      if (ratio < 1.0 / max_area_ratio_ || ratio > max_area_ratio_ ||
          distance > (template_support ? 0.75 : center_gate_sizes_)) continue;
    }
    double quality = 0.0;
    const auto feature = appearanceFeature(frame, candidate.box, &quality);
    const double similarity = appearance_anchor_.empty() ? 0.0 :
        cosineSimilarity(feature, appearance_anchor_);
    const double required_quality = long_recovery_ && !template_support
        ? std::max(quality_gate_, long_quality_gate_) : quality_gate_;
    const double required_similarity = long_recovery_ && !template_support
        ? std::max(appearance_gate_, long_appearance_gate_) : appearance_gate_;
    if (quality < required_quality || similarity < required_similarity) continue;
    if (long_recovery_ && long_motion_required_ && !motion_continuous_ && !template_support) continue;
    const Box &ranking_reference = template_support ? template_match_box : predicted;
    const double distance = cv::norm(center(candidate.box) - center(ranking_reference));
    const double scale = std::max(8.0, std::hypot(ranking_reference.w, ranking_reference.h));
    const double score = 0.45 * std::max(0.0, 1.0 - distance / scale) +
        0.25 * boxIou(candidate.box, ranking_reference) + 0.30 * similarity;
    if (long_recovery_ && score < long_score_gate_ && !template_support) continue;
    ranked.emplace_back(score, &candidate);
  }
  std::sort(ranked.begin(), ranked.end(), [](const auto &a, const auto &b) {
    return a.first > b.first;
  });
  if (ranked.size() > 1 && ranked[0].first - ranked[1].first < ambiguity_margin_) {
    pending_reacquire_id_ = -1;
    pending_reacquire_count_ = 0;
    if (template_visual_valid) {
      const Box continued = smooth_box(predicted, visual, 0.65);
      const double dt = stamp - last_observation_stamp_;
      if (dt > 0.0) {
        velocity_ = 0.8 * velocity_ +
            0.2 * (center(continued) - center(selected_box_)) / dt;
      }
      selected_box_ = continued;
      last_observation_stamp_ = stamp;
      result_.box = continued;
      result_.has_box = true;
      result_.observed = true;
      result_.source = "visual";
      result_.reason = "template_match_competing_detections";
      result_.confidence = std::min({0.65, template_match_score, selected_confidence_});
      motion_continuous_ = true;
      return;
    }
    result_.has_box = false;
    result_.observed = false;
    result_.source = "ambiguous";
    result_.reason = "competing_detections";
    result_.confidence = 0.0;
    return;
  }
  const Detection *match = ranked.empty() ? nullptr : ranked.front().second;
  const bool template_support_match = match && template_visual_valid &&
      template_match_score >= (long_recovery_
          ? long_template_match_threshold_ : template_match_threshold_) &&
      (boxIou(match->box, template_match_box) >= 0.10 ||
       cv::norm(center(match->box) - center(template_match_box)) /
           std::max(8.0, std::hypot(template_match_box.w, template_match_box.h)) <= 0.55);
  if (match && selected_local_id_ >= 0 && match->id != selected_local_id_ &&
      std::any_of(detections.begin(), detections.end(), [this](const Detection &item) {
        return item.id == selected_local_id_;
      })) {
    if (template_visual_valid) {
      const Box continued = smooth_box(predicted, visual, 0.65);
      const double dt = stamp - last_observation_stamp_;
      if (dt > 0.0) {
        velocity_ = 0.8 * velocity_ +
            0.2 * (center(continued) - center(selected_box_)) / dt;
      }
      selected_box_ = continued;
      last_observation_stamp_ = stamp;
      result_.box = continued;
      result_.has_box = true;
      result_.observed = true;
      result_.source = "visual";
      result_.reason = "template_match_identity_conflict";
      result_.confidence = std::min({0.65, template_match_score, selected_confidence_});
      motion_continuous_ = true;
      return;
    }
    result_.has_box = false;
    result_.observed = false;
    result_.source = "ambiguous";
    result_.reason = "old_and_new_tracks_simultaneously_visible";
    result_.confidence = 0.0;
    return;
  }
  const bool recent_same_identity = match && match->id == selected_local_id_ &&
      match->confidence >= min_detector_confidence_ && ranked.front().first >= 0.65 &&
      boxIou(match->box, predicted) >= 0.30 && absence <= max_frame_gap_sec_ &&
      stamp - last_detector_for_selection_ <= max_frame_gap_sec_ && result_.source != "ambiguous";
  const bool rebound = match && (selected_local_id_ < 0 || match->id != selected_local_id_ ||
      match->selected_identity_reacquire || pending_reacquire_count_ > 0 || long_recovery_ ||
      step > max_frame_gap_sec_ ||
      ((result_.source == "lost" || result_.source == "ambiguous" || result_.source == "prediction") &&
       !recent_same_identity));

  if (rebound) {
    const double measured = match->detector_stamp;
    const double pending_dt = std::max(0.0, measured - pending_reacquire_stamp_);
    const cv::Point2d pending_expected = center(pending_reacquire_box_) + velocity_ * pending_dt;
    const bool pending_jump = pending_reacquire_id_ == match->id &&
        pending_reacquire_count_ > 0 && cv::norm(center(match->box) - pending_expected) >
        2.0 + center_gate_sizes_ * std::hypot(match->box.w, match->box.h) *
            std::min(1.0, pending_dt / max_frame_gap_sec_);
    if (pending_reacquire_id_ != match->id || measured - pending_reacquire_stamp_ > max_frame_gap_sec_ ||
        step > max_frame_gap_sec_) {
      pending_reacquire_count_ = 0;
    }
    if (pending_jump) pending_reacquire_count_ = 0;
    pending_reacquire_id_ = static_cast<int>(match->id);
    if (measured > pending_reacquire_stamp_) {
      ++pending_reacquire_count_;
      pending_reacquire_stamp_ = measured;
      pending_reacquire_box_ = match->box;
    }
    // A same-frame template match is independent visual confirmation of a
    // one-frame YOLO rebound, so do not require three separate detector hits.
    if (template_support_match) pending_reacquire_count_ = confirmation_frames_;
    if (pending_reacquire_count_ < confirmation_frames_) {
      const bool seed_hold = manual_seed_without_yolo_ && manual_seed_visual_without_yolo_;
      const bool visual_hold_allowed = visual_valid &&
          (template_visual_valid || seed_hold ||
           stamp - std::max(last_detector_for_selection_, selected_stamp_) <= visual_hold_sec_);
      // Identity confirmation and control-box continuity are separate jobs:
      // the candidate has already passed class, motion, size, and appearance
      // gates above. Keep a valid template box on the output while YOLO confirms,
      // even if the detector rectangle itself has a different shape/IoU.
      if (visual_hold_allowed) {
        result_.box = visual;
        result_.has_box = true;
        result_.observed = true;
        result_.source = "visual";
        result_.reason = "reacquisition_confirmation_visual_hold";
        result_.confidence = std::min({0.65, visual_quality, selected_confidence_});
      } else {
        result_.has_box = false;
        result_.observed = false;
        result_.source = "ambiguous";
        result_.reason = "reacquisition_confirmation";
        result_.confidence = 0.0;
      }
      return;
    }
  } else if (!match && (detector_updated || stamp - pending_reacquire_stamp_ > max_frame_gap_sec_)) {
    pending_reacquire_id_ = -1;
    pending_reacquire_count_ = 0;
  }

  if (match) {
    manual_seed_without_yolo_ = false;
    const bool agrees = visual_valid && boxIou(visual, match->box) >= 0.30;
    Box measurement = match->box;
    std::string source = "yolo";
    if (agrees) {
      const double weight = std::max(0.5, detector_weight_ * match->confidence /
          std::max(0.01, detector_weight_ * match->confidence +
          (1.0 - detector_weight_) * visual_quality));
      measurement = {weight * match->box.x + (1.0 - weight) * visual.x,
                     weight * match->box.y + (1.0 - weight) * visual.y,
                     weight * match->box.w + (1.0 - weight) * visual.w,
                     weight * match->box.h + (1.0 - weight) * visual.h};
      source = "fused";
    }
    // Do not let a confirmed identity rebound snap the control box to one YOLO
    // rectangle. Blend the re-acquired detector box into the template/predicted box
    // over several frames; ordinary same-ID corrections retain their existing
    // confidence-dependent gain.
    const double authority_span = std::max(0.01, detector_authority_ - min_detector_confidence_);
    const double confidence_above_maintenance = clamp01(
        (match->confidence - min_detector_confidence_) / authority_span);
    const double detector_gain = rebound ? reacquire_detector_gain_ :
        std::max(0.28, std::min(0.62,
            0.28 + 0.28 * confidence_above_maintenance +
            (match->confidence >= detector_authority_ ? 0.06 : 0.0)));
    const Box fused = smooth_box(predicted, measurement, detector_gain);
    const bool preserve_visual_motion = rebound && visual_valid && !long_recovery_ &&
        stamp - std::max(last_detector_for_selection_, selected_stamp_) <= visual_hold_sec_;
    if (rebound && !preserve_visual_motion) {
      velocity_ = cv::Point2d(0.0, 0.0);
      motion_continuous_ = true;
    } else {
      const double observation_dt = stamp - last_observation_stamp_;
      if (observation_dt > 0.0) {
        const Box &velocity_box = preserve_visual_motion ? visual : fused;
        const cv::Point2d measured_velocity = (center(velocity_box) - center(selected_box_)) / observation_dt;
        velocity_ = 0.8 * velocity_ + 0.2 * measured_velocity;
      }
    }
    selected_local_id_ = match->id;
    selected_class_id_ = selected_class_id_ < 0 ? match->class_id : selected_class_id_;
    selected_box_ = fused;
    selected_confidence_ = match->confidence;
    last_detector_for_selection_ = match->detector_stamp;
    last_observation_stamp_ = stamp;
    pending_reacquire_id_ = -1;
    pending_reacquire_count_ = 0;
    double quality = 0.0;
    const auto feature = appearanceFeature(frame, match->box, &quality);
    const double template_similarity = appearance_anchor_.empty()
        ? 1.0 : cosineSimilarity(feature, appearance_anchor_);
    if (match->detector_stamp == stamp &&
        match->confidence >= template_update_confidence_ &&
        quality >= quality_gate_ && template_similarity >= appearance_gate_ &&
        stamp - last_template_update_stamp_ >= local_template_refresh_sec_) {
      cv::Mat current_template = grayTemplate(frame, match->box);
      if (!current_template.empty()) {
        if (target_template_gray_.empty()) {
          target_template_gray_ = current_template;
        } else {
          cv::Mat aligned_template;
          cv::resize(current_template, aligned_template, target_template_gray_.size());
          cv::addWeighted(target_template_gray_, 0.95, aligned_template, 0.05,
                          0.0, target_template_gray_);
        }
        last_template_update_stamp_ = stamp;
      }
    }
    if (quality >= 0.65 && match->confidence >= min_detector_confidence_ && match->detector_stamp == stamp) {
      if (appearance_anchor_.size() == feature.size()) {
        for (size_t i = 0; i < feature.size(); ++i) appearance_anchor_[i] = 0.95f * appearance_anchor_[i] + 0.05f * feature[i];
        const double norm = std::sqrt(std::inner_product(appearance_anchor_.begin(), appearance_anchor_.end(),
            appearance_anchor_.begin(), 0.0));
        if (norm > 1e-9) for (float &item : appearance_anchor_) item = static_cast<float>(item / norm);
      } else appearance_anchor_ = feature;
      anchor_area_ = 0.95 * anchor_area_ + 0.05 * match->box.w * match->box.h;
    }
    result_.box = fused;
    result_.has_box = true;
    result_.observed = true;
    result_.source = source;
    result_.reason = agrees ? "compatible_observations" : source;
    result_.confidence = match->confidence;
    result_.detector_stamp = match->detector_stamp;
  } else if (visual_valid && (template_visual_valid ||
      (manual_seed_without_yolo_ && manual_seed_visual_without_yolo_) ||
      stamp - std::max(last_detector_for_selection_, selected_stamp_) <= visual_hold_sec_)) {
    const Box filtered_visual = template_visual_valid
        ? smooth_template_box(predicted, visual) : smooth_box(predicted, visual, 0.65);
    const double dt = stamp - last_observation_stamp_;
    if (dt > 0.0) {
      const cv::Point2d measured = (center(filtered_visual) - center(selected_box_)) / dt;
      velocity_ = 0.8 * velocity_ + 0.2 * measured;
    }
    selected_box_ = filtered_visual;
    last_observation_stamp_ = stamp;
    const double template_adaptation_score =
        std::max(0.62, template_match_threshold_ + 0.14);
    const double template_adaptation_similarity = std::max(0.48, appearance_gate_ - 0.14);
    if (template_visual_valid && !target_template_gray_.empty() &&
        stamp - last_template_update_stamp_ >= local_template_refresh_sec_ &&
        template_match_score >= template_adaptation_score &&
        (visual_template_quality >= quality_gate_ ||
         (template_match_score >= 0.72 &&
          visual_template_quality >= quality_gate_ * 0.60)) &&
        visual_template_similarity >= template_adaptation_similarity) {
      cv::Mat current_template = grayTemplate(frame, visual);
      if (!current_template.empty()) {
        cv::Mat aligned_template;
        cv::resize(current_template, aligned_template, target_template_gray_.size());
        const double confidence = clamp01((template_match_score - template_adaptation_score) /
            std::max(1e-3, 1.0 - template_adaptation_score));
        const double adaptation_weight = 0.06 + 0.10 * confidence;
        cv::addWeighted(target_template_gray_, 1.0 - adaptation_weight,
                        aligned_template, adaptation_weight,
                        0.0, target_template_gray_);
        last_template_update_stamp_ = stamp;
      }
    }
    result_.box = filtered_visual;
    result_.has_box = true;
    result_.observed = true;
    result_.source = "visual";
    result_.reason = template_visual_valid
        ? "validated_template_match" : "validated_local_tracker_observation";
    result_.confidence = std::min({0.65, visual_quality, selected_confidence_});
  } else if (absence <= prediction_hold_sec_ && validBox(predicted, frame.size())) {
    result_.box = predicted;
    result_.has_box = true;
    result_.observed = false;
    result_.source = "prediction";
    result_.reason = "no_trusted_visual_observation";
    result_.confidence = std::min(0.25, selected_confidence_);
  } else {
    result_.has_box = false;
    result_.observed = false;
    result_.source = "lost";
    result_.reason = long_recovery_ ? "identity_memory_waiting" : "no_trusted_visual_observation";
    result_.confidence = 0.0;
  }
  result_.public_id = selected_id_;
  result_.local_id = selected_local_id_;
  result_.class_id = selected_class_id_;
  if (result_.source == "fused" || result_.source == "yolo" || result_.source == "visual") {
    result_.detector_stamp = last_detector_for_selection_;
  }
  if (result_.observed) motion_continuous_ = true;
}

void SmartTrackerFusionCpp::processColor(const cv::Mat &frame, double stamp,
    std::vector<Detection> &detections) {
  result_ = TrackResult();
  detections.clear();
  const std::string name = lower(color_name_);
  const bool configured_color = hasConfiguredColor(name);
  if (color_waiting_roi_ || (!color_initialized_ && !configured_color)) {
    result_.reason = "waiting_for_initial_roi";
    return;
  }
  cv::Mat hsv, mask;
  cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
  if (name == "red") {
    cv::Mat first, second;
    cv::inRange(hsv, cv::Scalar(0, 80, 50), cv::Scalar(12, 255, 255), first);
    cv::inRange(hsv, cv::Scalar(168, 80, 50), cv::Scalar(180, 255, 255), second);
    cv::bitwise_or(first, second, mask);
  } else if (name == "green") {
    cv::inRange(hsv, cv::Scalar(35, 60, 35), cv::Scalar(90, 255, 255), mask);
  } else if (name == "blue") {
    cv::inRange(hsv, cv::Scalar(90, 60, 35), cv::Scalar(135, 255, 255), mask);
  } else if (name == "white") {
    cv::inRange(hsv, cv::Scalar(0, 0, 180), cv::Scalar(180, 80, 255), mask);
  } else if (name == "black") {
    cv::inRange(hsv, cv::Scalar(0, 0, 0), cv::Scalar(180, 255, 65), mask);
  } else if (color_initialized_ && !color_histogram_.empty()) {
    int channels[] = {0, 1};
    int sizes[] = {16, 8};
    float hue[] = {0.0f, 180.0f}, saturation[] = {0.0f, 256.0f};
    const float *ranges[] = {hue, saturation};
    cv::Mat hist(2, sizes, CV_32F, color_histogram_.data());
    cv::calcBackProject(&hsv, 1, channels, hist, mask, ranges, 1.0);
    cv::threshold(mask, mask, 35, 255, cv::THRESH_BINARY);
  } else {
    return;
  }
  const int kernel_size = 5;
  cv::morphologyEx(mask, mask, cv::MORPH_OPEN,
      cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(kernel_size, kernel_size)));
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  const bool follow_selection = selected_ && validBox(selected_box_, frame.size());
  const cv::Point2d anchor = follow_selection ? center(selected_box_) : cv::Point2d();
  const double max_jump = follow_selection
      ? std::min(0.35 * std::hypot(frame.cols, frame.rows),
                 std::max(48.0, 4.0 * std::hypot(selected_box_.w, selected_box_.h)))
      : std::numeric_limits<double>::infinity();
  double best_score = follow_selection ? std::numeric_limits<double>::infinity() : -1.0;
  Box best;
  size_t best_index = 0;
  for (const auto &contour : contours) {
    const cv::Rect rect = cv::boundingRect(contour);
    if (rect.area() < minimum_color_area_ || rect.width < 4 || rect.height < 4) continue;
    const Box candidate_box{static_cast<double>(rect.x), static_cast<double>(rect.y),
                            static_cast<double>(rect.width), static_cast<double>(rect.height)};
    double score = cv::contourArea(contour);
    bool eligible_to_follow_selected = true;
    if (follow_selection) {
      const double distance = cv::norm(center(candidate_box) - anchor);
      if (distance > max_jump) {
        // Keep this component in the full-frame color candidates. The jump
        // gate applies only to maintaining the selected identity, as in the
        // Python ColorTracker path.
        eligible_to_follow_selected = false;
      } else if (configured_color) {
        // Python ColorTracker follows the nearest colour component to the UI
        // selected box and rejects components that jump too far.
        score = distance;
      } else if (eligible_to_follow_selected) {
        // ROI-learned color mode is a single-target tracker. Prefer the
        // component overlapping the sampled ROI, then the nearest component.
        score = -boxIou(candidate_box, selected_box_) * 1.0e6 + distance;
      }
    }
    Detection detection;
    detection.id = static_cast<int64_t>(detections.size() + 1);
    detection.class_id = color_class_id_;
    detection.confidence = 0.5;
    detection.box = candidate_box;
    detection.stable = false;
    detection.source = "color_detector";
    detection.detector_stamp = stamp;
    detections.push_back(std::move(detection));
    if (eligible_to_follow_selected &&
        ((follow_selection && score < best_score) || (!follow_selection && score > best_score))) {
      best_score = score;
      best = candidate_box;
      best_index = detections.size() - 1;
    }
  }
  const bool found = validBox(best, frame.size()) &&
      (follow_selection ? best_score < std::numeric_limits<double>::infinity() : best_score >= 0.0);
  if (!found) {
    if (!configured_color) detections.clear();
    if (selected_) {
      result_.public_id = selected_id_;
      result_.local_id = selected_local_id_;
      result_.class_id = selected_class_id_;
      result_.source = "lost";
      result_.reason = "color_not_found";
    }
    return;
  }
  if (!selected_) {
    result_.reason = "color_candidates_waiting_for_selection";
    return;
  }
  selected_box_ = best;
  selected_confidence_ = detections[best_index].confidence;
  last_observation_stamp_ = stamp;
  detections[best_index].source = "color_detector_candidate";
  if (!configured_color) {
    Detection selected_detection = detections[best_index];
    detections.assign(1, selected_detection);
  }
  result_ = {best, selected_id_, selected_local_id_, selected_class_id_, selected_confidence_,
             0.0, "visual", "color_tracker", true, true};
}

bool SmartTrackerFusionCpp::selectConfiguredColorCandidate(const Snapshot &snapshot,
    const Detection &candidate, const std::string &reason, bool auto_selection) {
  if (!snapshot.image || mode_ != "color_tracker" || !hasConfiguredColor(color_name_) ||
      !validBox(candidate.box, snapshot.frame.size())) return false;

  const bool was_auto_selecting = auto_select_enabled_;
  if (!auto_selection) auto_select_enabled_ = false;
  auto_select_count_ = 0;
  auto_select_candidate_id_ = -1;
  resetSelection();
  selected_ = true;
  manual_active_ = true;
  selected_id_ = manual_track_id_;
  selected_local_id_ = -1;
  selected_class_id_ = candidate.class_id;
  selected_box_ = candidate.box;
  selected_confidence_ = candidate.confidence;
  selected_stamp_ = snapshot.image->header.stamp.toSec();
  last_observation_stamp_ = selected_stamp_;
  result_ = {selected_box_, selected_id_, selected_local_id_, selected_class_id_,
             selected_confidence_, 0.0, "visual", reason, true, true};
  if (was_auto_selecting && !auto_selection) {
    auto_select_pub_status_ = false;
    std_msgs::Bool status;
    status.data = false;
    auto_status_pub_.publish(status);
  }
  return true;
}

bool SmartTrackerFusionCpp::initializeColorFromRoi(const Snapshot &snapshot, const Box &roi) {
  if (!snapshot.image || !validBox(roi, snapshot.frame.size())) return false;
  const int x = std::max(0, static_cast<int>(std::floor(roi.x)));
  const int y = std::max(0, static_cast<int>(std::floor(roi.y)));
  const int right = std::min(snapshot.frame.cols, static_cast<int>(std::ceil(roi.x + roi.w)));
  const int bottom = std::min(snapshot.frame.rows, static_cast<int>(std::ceil(roi.y + roi.h)));
  if (right - x < 4 || bottom - y < 4) return false;

  cv::Mat hsv;
  cv::cvtColor(snapshot.frame(cv::Rect(x, y, right - x, bottom - y)), hsv, cv::COLOR_BGR2HSV);
  int channels[] = {0, 1};
  int sizes[] = {16, 8};
  float hue[] = {0.0f, 180.0f}, saturation[] = {0.0f, 256.0f};
  const float *ranges[] = {hue, saturation};
  cv::Mat histogram;
  cv::calcHist(&hsv, 1, channels, cv::Mat(), histogram, 2, sizes, ranges, true, false);
  double max_bin = 0.0;
  cv::minMaxLoc(histogram, nullptr, &max_bin);
  if (!(max_bin > 1e-9)) return false;
  // calcBackProject writes an 8-bit mask and processColor thresholds it at
  // 35, so keep histogram bins in the same 0..255 range. L2-normalizing to
  // values below 1 made every backprojection pixel fall below that threshold.
  histogram *= 255.0 / max_bin;

  const bool was_auto_selecting = auto_select_enabled_;
  resetSelection();
  color_histogram_.assign(histogram.ptr<float>(), histogram.ptr<float>() + histogram.total());
  color_initialized_ = true;
  color_waiting_roi_ = false;
  color_name_ = "custom";
  auto_select_enabled_ = false;
  auto_select_count_ = 0;
  selected_ = true;
  manual_active_ = true;
  manual_seed_ = false;
  selected_id_ = manual_track_id_;
  selected_local_id_ = -1;
  selected_class_id_ = color_class_id_;
  selected_box_ = roi;
  selected_confidence_ = 0.5;
  selected_stamp_ = snapshot.image->header.stamp.toSec();
  last_observation_stamp_ = selected_stamp_;
  result_ = {roi, selected_id_, selected_local_id_, selected_class_id_, selected_confidence_,
             0.0, "visual", "color_roi_selected", true, true};
  if (was_auto_selecting) {
    std_msgs::Bool status;
    status.data = false;
    auto_status_pub_.publish(status);
  }
  return true;
}

void SmartTrackerFusionCpp::updateAutoSelection(const std::vector<Detection> &detections,
    const cv::Mat &frame, const sensor_msgs::ImageConstPtr &image) {
  const auto clear_candidate = [this]() {
    auto_select_count_ = 0;
    auto_select_candidate_id_ = -1;
    auto_select_candidate_class_ = -1;
    auto_select_candidate_stable_ = false;
    auto_select_last_stamp_ = 0.0;
    auto_select_box_ = Box();
  };
  if (!auto_select_enabled_ || !image) {
    clear_candidate();
    return;
  }

  const double stamp = image->header.stamp.toSec();
  if (!std::isfinite(stamp) || stamp <= 0.0) {
    clear_candidate();
    return;
  }
  if (detections.empty()) {
    if (auto_select_last_stamp_ <= 0.0 || stamp <= auto_select_last_stamp_ ||
        stamp - auto_select_last_stamp_ > auto_select_max_gap_)
      clear_candidate();
    return;
  }

  const Detection *winner = nullptr;
  const Detection *pending_candidate = nullptr;
  for (const auto &detection : detections) {
    if (detection.confidence < auto_select_min_confidence_ || !validBox(detection.box, frame.size())) continue;
    if (winner == nullptr || detection.confidence > winner->confidence ||
        (detection.confidence == winner->confidence && detection.box.w * detection.box.h > winner->box.w * winner->box.h)) {
      winner = &detection;
    }
    const bool same_pending_identity = auto_select_candidate_class_ == detection.class_id &&
        ((auto_select_candidate_stable_ && detection.stable &&
          auto_select_candidate_id_ == detection.id) ||
         (!auto_select_candidate_stable_ && !detection.stable &&
          boxIou(auto_select_box_, detection.box) >= 0.30));
    if (auto_select_count_ > 0 && same_pending_identity &&
        (pending_candidate == nullptr || detection.confidence > pending_candidate->confidence)) {
      pending_candidate = &detection;
    }
  }
  // Keep confirming the same target even if another object's confidence is
  // briefly higher in a later detector result.
  if (pending_candidate != nullptr) winner = pending_candidate;
  if (winner == nullptr) {
    if (auto_select_last_stamp_ <= 0.0 || stamp <= auto_select_last_stamp_ ||
        stamp - auto_select_last_stamp_ > auto_select_max_gap_)
      clear_candidate();
    return;
  }

  const double area_ratio = winner->box.w * winner->box.h /
      std::max(1.0, auto_select_box_.w * auto_select_box_.h);
  const double center_distance = cv::norm(center(auto_select_box_) - center(winner->box)) /
      std::max(8.0, std::hypot(std::max(auto_select_box_.w, winner->box.w),
                              std::max(auto_select_box_.h, winner->box.h)));
  const bool stable_id_match = auto_select_candidate_stable_ && winner->stable &&
      auto_select_candidate_id_ == winner->id;
  const bool geometry_match = area_ratio >= 0.25 && area_ratio <= 4.0 &&
      center_distance <= 0.60 &&
      (boxIou(auto_select_box_, winner->box) >= 0.10 ||
       duplicateTargetBoxes(auto_select_box_, winner->class_id,
                            winner->box, winner->class_id));
  const bool same = auto_select_count_ > 0 &&
      auto_select_candidate_class_ == winner->class_id &&
      (stable_id_match || geometry_match) &&
      stamp - auto_select_last_stamp_ > 0.0 &&
      stamp - auto_select_last_stamp_ <= auto_select_max_gap_;
  auto_select_count_ = same ? auto_select_count_ + 1 : 1;
  auto_select_candidate_id_ = winner->id;
  auto_select_candidate_class_ = winner->class_id;
  auto_select_candidate_stable_ = winner->stable;
  auto_select_box_ = winner->box;
  auto_select_last_stamp_ = stamp;
  if (auto_select_count_ < auto_select_stable_frames_) return;
  Snapshot snapshot{image, frame, detections, selected_box_, selected_};
  bool selected = false;
  if (mode_ == "color_tracker" && hasConfiguredColor(color_name_)) {
    selected = selectConfiguredColorCandidate(snapshot, *winner, "color_auto_selected", true);
  } else {
    int64_t id = -1;
    std::string error;
    selected = selectFromSnapshot(snapshot, winner->box, true, false, &id, &error);
  }
  if (selected) {
    auto_selected_ = true;
    ROS_INFO_STREAM("C++ SmartTracker auto-selected detection id=" << winner->id
        << " stable_frames=" << auto_select_count_ << "; auto-selection remains armed");
  }
  auto_select_count_ = 0;
  auto_select_candidate_id_ = -1;
}

bool SmartTrackerFusionCpp::trackingPhase() {
  std::lock_guard<std::mutex> phase_lock(phase_mutex_);
  const double now = std::chrono::duration<double>(SteadyClock::now().time_since_epoch()).count();
  if (!phase_has_detection_ || now - phase_detection_wall_ > phase_detection_timeout_) {
    phase_attacking_ = false;
    phase_tracking_started_ = 0.0;
    phase_text_ = "UNIDENTIFICATION";
    return false;
  }
  if (phase_status_wall_ <= 0.0 || now - phase_status_wall_ > phase_status_timeout_ || !phase_tracker_active_) {
    phase_attacking_ = false;
    phase_tracking_started_ = 0.0;
    phase_text_ = "LOCKING";
    return false;
  }
  if (!phase_target_visible_ || !phase_command_valid_) {
    phase_attacking_ = false;
    phase_tracking_started_ = 0.0;
    phase_text_ = "TRACKING";
    return false;
  }
  if (phase_tracking_started_ <= 0.0) phase_tracking_started_ = now;
  if (now - phase_tracking_started_ < phase_min_duration_) {
    phase_attacking_ = false;
    phase_text_ = "TRACKING";
    return false;
  }
  const bool gimbal_profile = phase_profile_ == "gm_velocity_chase" ||
      phase_profile_ == "gm_velocity_vector" || phase_profile_ == "gmfw_velocity_vector";
  double error = std::max(std::abs(phase_error_x_), std::abs(phase_error_y_));
  double enter = phase_enter_error_, exit = phase_exit_error_;
  if (gimbal_profile) {
    if (!phase_status_gimbal_valid_ || !phase_gimbal_valid_ ||
        now - phase_gimbal_wall_ > phase_gimbal_timeout_) {
      phase_attacking_ = false;
      phase_tracking_started_ = 0.0;
      phase_text_ = "TRACKING";
      return false;
    }
    error = std::abs(phase_gimbal_yaw_);
    enter = phase_enter_yaw_;
    exit = phase_exit_yaw_;
  }
  if (phase_attacking_) {
    if (error > exit) phase_attacking_ = false;
  } else if (error <= enter) {
    phase_attacking_ = true;
  }
  phase_text_ = phase_attacking_ ? "ATTACKING" : "TRACKING";
  return phase_attacking_;
}

void SmartTrackerFusionCpp::drawOverlay(cv::Mat &frame, const std::vector<Detection> &detections) {
  std::lock_guard<std::mutex> state_lock(state_mutex_);
  const bool only_selected_fusion_target = selected_ && mode_ == "fusion_tracker";
  const cv::Scalar selected_color(0, 255, 255);
  const auto draw_selected_reticle = [&frame, &selected_color](const Box &box) {
    const int left = static_cast<int>(std::lround(box.x));
    const int top = static_cast<int>(std::lround(box.y));
    const int right = static_cast<int>(std::lround(box.x + box.w));
    const int bottom = static_cast<int>(std::lround(box.y + box.h));
    const int span = std::max(1, std::min(right - left, bottom - top));
    const int corner = std::max(3, std::min(24, span / 4));
    const int thickness = 3;

    cv::line(frame, {left, top}, {left + corner, top}, selected_color, thickness, cv::LINE_AA);
    cv::line(frame, {left, top}, {left, top + corner}, selected_color, thickness, cv::LINE_AA);
    cv::line(frame, {right, top}, {right - corner, top}, selected_color, thickness, cv::LINE_AA);
    cv::line(frame, {right, top}, {right, top + corner}, selected_color, thickness, cv::LINE_AA);
    cv::line(frame, {left, bottom}, {left + corner, bottom}, selected_color, thickness, cv::LINE_AA);
    cv::line(frame, {left, bottom}, {left, bottom - corner}, selected_color, thickness, cv::LINE_AA);
    cv::line(frame, {right, bottom}, {right - corner, bottom}, selected_color, thickness, cv::LINE_AA);
    cv::line(frame, {right, bottom}, {right, bottom - corner}, selected_color, thickness, cv::LINE_AA);

    const int center_x = (left + right) / 2;
    const int center_y = (top + bottom) / 2;
    const int cross_arm = std::max(4, std::min(14, span / 10));
    cv::line(frame, {center_x - cross_arm, center_y}, {center_x + cross_arm, center_y},
             selected_color, thickness, cv::LINE_AA);
    cv::line(frame, {center_x, center_y - cross_arm}, {center_x, center_y + cross_arm},
             selected_color, thickness, cv::LINE_AA);
  };
  const Detection *selected_detection = nullptr;
  if (!only_selected_fusion_target) {
    for (const auto &detection : detections) {
      const bool selected = selected_ && detection.id == selected_local_id_ && detection.stable;
      if (selected) {
        selected_detection = &detection;
        continue;
      }
      const cv::Scalar color(255, 190, 0);
      const cv::Point first(static_cast<int>(std::lround(detection.box.x)), static_cast<int>(std::lround(detection.box.y)));
      const cv::Point second(static_cast<int>(std::lround(detection.box.x + detection.box.w)),
                             static_cast<int>(std::lround(detection.box.y + detection.box.h)));
      if (detection.polygon.size() >= 6) {
        std::vector<cv::Point> polygon;
        for (size_t i = 0; i + 1 < detection.polygon.size(); i += 2)
          polygon.emplace_back(static_cast<int>(std::lround(detection.polygon[i])),
                               static_cast<int>(std::lround(detection.polygon[i + 1])));
        cv::polylines(frame, polygon, true, color, 2, cv::LINE_AA);
      } else {
        cv::rectangle(frame, first, second, color, 2, cv::LINE_AA);
      }
      std::ostringstream label;
      label << className(detection.class_id) << " ID " << detection.id;
      cv::putText(frame, label.str(), cv::Point(first.x, std::max(18, first.y - 5)),
                  cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv::LINE_AA);
    }
  }
  if (result_.has_box && result_.public_id >= 0 && result_.source != "idle") {
    const cv::Point first(static_cast<int>(std::lround(result_.box.x)), static_cast<int>(std::lround(result_.box.y)));
    draw_selected_reticle(result_.box);
    std::ostringstream label;
    label << className(result_.class_id) << " ID " << result_.public_id;
    cv::putText(frame, label.str(), cv::Point(first.x, std::max(18, first.y - 5)),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, selected_color, 1, cv::LINE_AA);
  } else if (!only_selected_fusion_target && selected_detection != nullptr) {
    const cv::Point first(static_cast<int>(std::lround(selected_detection->box.x)),
                          static_cast<int>(std::lround(selected_detection->box.y)));
    draw_selected_reticle(selected_detection->box);
    std::ostringstream label;
    label << className(selected_detection->class_id) << " ID " << selected_detection->id;
    cv::putText(frame, label.str(), cv::Point(first.x, std::max(18, first.y - 5)),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, selected_color, 1, cv::LINE_AA);
  } else if (selected_ && (result_.source == "lost" || result_.source == "ambiguous" || result_.source == "prediction")) {
    cv::putText(frame, lower(result_.source), cv::Point(12, 25), cv::FONT_HERSHEY_SIMPLEX,
                0.6, cv::Scalar(0, 180, 255), 1, cv::LINE_AA);
  }

  const std::string metric_status_text = !metric_target_status_received_
      ? "valid: n/a" : (metric_target_valid_ ? "valid: true" : "valid: false");
  int metric_baseline = 0;
  const double metric_font_scale = 0.9;
  const int metric_thickness = 2;
  const cv::Size metric_text_size = cv::getTextSize(metric_status_text,
      cv::FONT_HERSHEY_SIMPLEX, metric_font_scale, metric_thickness, &metric_baseline);
  const int metric_x = std::max(6, frame.cols - metric_text_size.width - 22);
  const int metric_y = 8;
  const cv::Scalar metric_color = metric_target_valid_
      ? cv::Scalar(0, 220, 0) : cv::Scalar(0, 80, 255);
  cv::rectangle(frame,
      cv::Point(metric_x, metric_y),
      cv::Point(metric_x + metric_text_size.width + 16,
                metric_y + metric_text_size.height + metric_baseline + 14),
      cv::Scalar(20, 20, 20), cv::FILLED);
  cv::putText(frame, metric_status_text,
      cv::Point(metric_x + 8, metric_y + metric_text_size.height + 6),
      cv::FONT_HERSHEY_SIMPLEX, metric_font_scale, metric_color,
      metric_thickness, cv::LINE_AA);

  if (phase_overlay_enabled_) {
    trackingPhase();
    const std::map<std::string, cv::Scalar> colors{{"UNIDENTIFICATION", {96, 96, 96}},
        {"LOCKING", {0, 165, 255}}, {"TRACKING", {0, 210, 0}}, {"ATTACKING", {0, 0, 255}}};
    const std::string text = phase_text_.empty() ? "UNIDENTIFICATION" : phase_text_;
    int baseline = 0;
    const cv::Size size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.9, 2, &baseline);
    const cv::Scalar color = colors.at(text);
    cv::rectangle(frame, cv::Point(8, 8), cv::Point(32 + size.width, 32 + size.height + baseline), color, cv::FILLED);
    cv::putText(frame, text, cv::Point(20, 20 + size.height), cv::FONT_HERSHEY_SIMPLEX,
                0.9, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
  }
}

void SmartTrackerFusionCpp::metricTargetStatusCallback(
    const std_msgs::StringConstPtr &message) {
  if (!message) return;
  try {
    const YAML::Node status = YAML::Load(message->data);
    if (!status.IsMap() || !status["valid"] || !status["valid"].IsScalar()) {
      throw std::runtime_error("metric target status has no scalar 'valid' field");
    }
    const bool valid = status["valid"].as<bool>();
    std::lock_guard<std::mutex> lock(state_mutex_);
    metric_target_valid_ = valid;
    metric_target_status_received_ = true;
  } catch (const std::exception &error) {
    ROS_WARN_STREAM_THROTTLE(5.0,
        "Could not parse metric target status from " << metric_target_status_topic_
        << ": " << error.what());
  }
}

void SmartTrackerFusionCpp::phaseDetectionCallback(const xd_uav_track::DetectionArrayConstPtr &message) {
  std::lock_guard<std::mutex> lock(phase_mutex_);
  phase_has_detection_ = false;
  for (const auto &candidate : message->candidates) {
    if (candidate.has_bbox && candidate.bbox[2] > candidate.bbox[0] && candidate.bbox[3] > candidate.bbox[1]) {
      phase_has_detection_ = true;
      break;
    }
  }
  phase_detection_wall_ = std::chrono::duration<double>(SteadyClock::now().time_since_epoch()).count();
}

void SmartTrackerFusionCpp::phaseStatusCallback(const xd_uav_track::TrackStatusConstPtr &message) {
  std::lock_guard<std::mutex> lock(phase_mutex_);
  if (message->follower_profile != phase_profile_) {
    phase_attacking_ = false;
    phase_tracking_started_ = 0.0;
  }
  phase_profile_ = message->follower_profile;
  phase_tracker_active_ = message->tracker_active;
  phase_target_visible_ = message->target_visible;
  phase_command_valid_ = message->command_valid;
  phase_status_gimbal_valid_ = message->gimbal_state_valid;
  phase_error_x_ = message->error_x;
  phase_error_y_ = message->error_y;
  phase_status_wall_ = std::chrono::duration<double>(SteadyClock::now().time_since_epoch()).count();
}

void SmartTrackerFusionCpp::phaseGimbalCallback(const xd_uav_track::GimbalStateConstPtr &message) {
  std::lock_guard<std::mutex> lock(phase_mutex_);
  phase_gimbal_valid_ = message->valid && std::isfinite(message->yaw_rad);
  phase_gimbal_wall_ = std::chrono::duration<double>(SteadyClock::now().time_since_epoch()).count();
  if (!phase_gimbal_valid_) {
    phase_gimbal_filter_initialized_ = false;
    return;
  }
  const double yaw = phase_gimbal_yaw_inverted_ ? -message->yaw_rad : message->yaw_rad;
  if (!phase_gimbal_filter_initialized_) {
    phase_filtered_gimbal_yaw_ = yaw;
    phase_gimbal_filter_initialized_ = true;
  } else {
    phase_filtered_gimbal_yaw_ = phase_gimbal_filter_previous_weight_ * phase_filtered_gimbal_yaw_ +
        (1.0 - phase_gimbal_filter_previous_weight_) * yaw;
  }
  phase_gimbal_yaw_ = phase_filtered_gimbal_yaw_ + phase_gimbal_yaw_offset_;
  if (std::abs(phase_gimbal_yaw_) < phase_gimbal_angle_deadzone_) phase_gimbal_yaw_ = 0.0;
}

void SmartTrackerFusionCpp::publishOutputs(const sensor_msgs::ImageConstPtr &image,
    const cv::Mat &frame, const std::vector<Detection> &detections, bool fresh,
    double inference_ms, double processing_ms) {
  if (!image || frame.empty()) return;
  TrackResult selected;
  bool has_selection = false;
  bool manual_active = false;
  std::string mode;
  std::string error;
  std::string phase;
  int class_id = -1;
  int64_t public_id = -1;
  int64_t local_id = -1;
  double selected_stamp = 0.0;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    selected = result_;
    has_selection = selected_;
    manual_active = manual_active_;
    mode = mode_;
    error = last_error_;
    phase = phase_text_;
    class_id = selected_class_id_;
    public_id = selected_id_;
    local_id = selected_local_id_;
    selected_stamp = last_detector_for_selection_;
  }

  // FusionTracker publishes YOLO candidates; ColorTracker publishes color
  // candidates. FeatureTracker only publishes a manually selected target.
  std::vector<Detection> output =
      (mode == "fusion_tracker" || mode == "color_tracker")
      ? detections : std::vector<Detection>{};
  const bool selected_observed = has_selection && selected.observed && selected.has_box;
  const bool selected_coasting = has_selection && selected.has_box &&
      selected.source == "prediction";
  const bool selected_publishable = selected_observed || selected_coasting;
  if (selected_publishable) {
    output.erase(std::remove_if(output.begin(), output.end(), [local_id, mode, &selected](const Detection &detection) {
      return (local_id >= 0 && detection.id == local_id) ||
          duplicateTargetBoxes(detection.box, detection.class_id,
                               selected.box, selected.class_id) ||
          (mode == "color_tracker" && boxIou(detection.box, selected.box) >= 0.50);
    }), output.end());
    Detection target;
    target.id = selected.public_id;
    target.class_id = selected.class_id >= 0 ? selected.class_id : class_id;
    target.confidence = selected.confidence;
    target.box = selected.box;
    target.stable = true;
    target.geometry = "aabb";
    target.detector_stamp = selected.detector_stamp > 0.0 ? selected.detector_stamp : selected_stamp;
    target.source = selected.source == "visual" ? "visual_tracker" :
        (selected.source == "prediction" ? "prediction" : "detector");
    output.push_back(target);
  }

  sar_yolo_detector::PerceptionIdentity provenance;
  provenance.mission_id = mission_id_;
  provenance.uav_id = uav_id_;
  provenance.session_uuid = session_uuid_;
  provenance.sensor_id = sensor_id_;
  provenance.profile = profile_;
  provenance.model_version = model_version_;
  provenance.model_sha256 = model_sha256_;
  provenance.calibration_version = calibration_version_;
  provenance.coordinate_contract = coordinate_contract_;

  sar_yolo_detector::TrackedDetection2DArray tracked_array;
  tracked_array.header = image->header;
  tracked_array.provenance = provenance;
  sar_yolo_detector::FusionDetection2DArray fusion_array;
  fusion_array.header = image->header;
  fusion_array.provenance = provenance;
  fusion_array.image_width = image->width;
  fusion_array.image_height = image->height;

  auto append_detection = [&](const Detection &item,
                              sar_yolo_detector::TrackedDetection2DArray *tracked,
                              sar_yolo_detector::FusionDetection2DArray *fusion) {
    sar_yolo_detector::TrackedDetection2D output_item;
    output_item.track_id = item.id;
    output_item.track_id_is_stable = item.stable;
    output_item.detection.header = image->header;
    output_item.detection.bbox.center.x = item.box.x + item.box.w * 0.5;
    output_item.detection.bbox.center.y = item.box.y + item.box.h * 0.5;
    output_item.detection.bbox.center.theta = item.has_rotation ? item.rotation_deg * CV_PI / 180.0 : 0.0;
    output_item.detection.bbox.size_x = std::max(0.0, item.box.w);
    output_item.detection.bbox.size_y = std::max(0.0, item.box.h);
    vision_msgs::ObjectHypothesisWithPose hypothesis;
    hypothesis.id = item.class_id;
    hypothesis.score = clamp01(item.confidence);
    output_item.detection.results.push_back(hypothesis);
    tracked->detections.push_back(output_item);

    sar_yolo_detector::FusionDetection2D fused;
    fused.target = output_item;
    fused.measurement_source = item.source.empty() ? "detector" : item.source;
    fused.last_detector_stamp = item.detector_stamp > 0.0 ? ros::Time(item.detector_stamp) : image->header.stamp;
    fusion->detections.push_back(fused);
  };

  for (const auto &item : output) append_detection(item, &tracked_array, &fusion_array);
  tracked_pub_.publish(tracked_array);
  if (mode == "fusion_tracker") fusion_pub_.publish(fusion_array);

  if (mode != "fusion_tracker" || fresh) {
    sar_yolo_detector::FusionDetection2DArray observations;
    observations.header = image->header;
    observations.provenance = provenance;
    observations.image_width = image->width;
    observations.image_height = image->height;
    const auto &observation_items = mode == "fusion_tracker" ? detections : output;
    for (const auto &item : observation_items) {
      if (mode == "fusion_tracker" &&
          std::abs(item.detector_stamp - image->header.stamp.toSec()) > 1e-6) continue;
      sar_yolo_detector::FusionDetection2D entry;
      sar_yolo_detector::TrackedDetection2D target;
      target.track_id = item.id;
      target.track_id_is_stable = item.stable;
      target.detection.header = image->header;
      target.detection.bbox.center.x = item.box.x + item.box.w * 0.5;
      target.detection.bbox.center.y = item.box.y + item.box.h * 0.5;
      target.detection.bbox.center.theta = item.has_rotation ? item.rotation_deg * CV_PI / 180.0 : 0.0;
      target.detection.bbox.size_x = item.box.w;
      target.detection.bbox.size_y = item.box.h;
      vision_msgs::ObjectHypothesisWithPose hypothesis;
      hypothesis.id = item.class_id;
      hypothesis.score = clamp01(item.confidence);
      target.detection.results.push_back(hypothesis);
      entry.target = target;
      entry.measurement_source = mode == "fusion_tracker"
          ? "detector" : (item.source.empty() ? "visual_tracker" : item.source);
      entry.last_detector_stamp = mode == "fusion_tracker" && item.detector_stamp > 0.0
          ? ros::Time(item.detector_stamp) : ros::Time(0);
      observations.detections.push_back(entry);
    }
    observations_pub_.publish(observations);
  }

  sar_yolo_detector::TrackedDetection2DArray selected_array;
  selected_array.header = image->header;
  selected_array.provenance = provenance;
  if (selected_publishable) {
    auto found = std::find_if(output.begin(), output.end(), [public_id](const Detection &detection) {
      return detection.id == public_id;
    });
    if (found != output.end()) {
      sar_yolo_detector::TrackedDetection2DArray one;
      sar_yolo_detector::FusionDetection2DArray unused;
      append_detection(*found, &one, &unused);
      if (!one.detections.empty()) selected_array.detections.push_back(one.detections.front());
    }
  }
  selected_pub_.publish(selected_array);

  sar_yolo_detector::SmartTrackerState state;
  state.header = image->header;
  state.tracking_active = has_selection;
  state.has_selection = has_selection;
  state.selected_track_id = has_selection ? selected.public_id : -1;
  state.selected_class_id = has_selection ? selected.class_id : -1;
  state.selected_class_name = has_selection ? className(selected.class_id) : "unknown";
  state.confidence = static_cast<float>(has_selection ? selected.confidence : 0.0);
  if (selected.has_box) {
    state.selected_bbox.center.x = selected.box.x + selected.box.w * 0.5;
    state.selected_bbox.center.y = selected.box.y + selected.box.h * 0.5;
    state.selected_bbox.center.theta = 0.0;
    state.selected_bbox.size_x = selected.box.w;
    state.selected_bbox.size_y = selected.box.h;
    state.geometry_type = "aabb";
  } else {
    state.geometry_type = "none";
  }
  state.measurement_current = selected.source == "yolo" || selected.source == "fused";
  state.prediction_only = selected.source == "prediction";
  state.tentative = selected.source == "ambiguous";
  state.data_is_stale = selected.source == "lost" || selected.source == "ambiguous";
  state.control_measurement_ready = selected.observed;
  state.association_method = mode == "fusion_tracker"
      ? "fusion_" + selected.source : "manual_" + mode;
  state.freshness_reason = selected.reason;
  state.tracker_type = mode == "fusion_tracker" ? "YOLO+Template" : mode;
  state.backend = mode == "fusion_tracker" ? "shared_remote" : "opencv";
  state.device = mode == "fusion_tracker" && use_gpu_ ? "cuda" : "cpu";
  state.model_path = mode == "fusion_tracker" ? model_path_ : "";
  state.detection_count = static_cast<uint32_t>(output.size());
  state.frame_processing_ms = static_cast<float>(processing_ms);
  if (selected.detector_stamp > 0.0 && image->header.stamp.toSec() >= selected.detector_stamp) {
    state.frames_since_detection = static_cast<uint32_t>(std::max(0.0,
        (image->header.stamp.toSec() - selected.detector_stamp) * std::max(1.0, inference_max_fps_)));
  }
  state_pub_.publish(state);

  const std::string loss_summary = updateLossJournal(selected, image->header.stamp.toSec());
  std::ostringstream evidence;
  evidence << "{\"stamp\":" << image->header.stamp.toSec()
      << ",\"frame_seq\":" << image->header.seq
      << ",\"mode\":\"" << mode << "\",\"selected_id\":" << selected.public_id
      << ",\"local_id\":" << selected.local_id << ",\"source\":\"" << selected.source
      << "\",\"reason\":\"" << selected.reason << "\",\"observed\":"
      << (selected.observed ? "true" : "false")
      << ",\"detection_count\":" << output.size()
      << ",\"detector_updated\":" << (fresh ? "true" : "false")
      << ",\"inference_error\":\"" << jsonEscape(error) << "\""
      << ",\"inference_ms\":" << inference_ms
      << ",\"processing_ms\":" << processing_ms
      << ",\"camera_motion_source\":\"" << camera_motion_.source() << "\""
      << ",\"loss_event\":" << loss_summary << '}';
  std_msgs::String evidence_message;
  evidence_message.data = evidence.str();
  evidence_pub_.publish(evidence_message);

  if (publish_annotated_ && annotated_pub_ && annotated_pub_.getNumSubscribers() > 0) {
    const double wall_now = std::chrono::duration<double>(SteadyClock::now().time_since_epoch()).count();
    if (annotated_max_fps_ <= 0.0 || last_annotated_wall_ == 0.0 ||
        wall_now - last_annotated_wall_ >= 1.0 / annotated_max_fps_) {
      cv::Mat annotated = frame.clone();
      drawOverlay(annotated, output);
      annotated_pub_.publish(toRosImage(annotated, image->header));
      last_annotated_wall_ = wall_now;
    }
  }
  (void)manual_active;
}

void SmartTrackerFusionCpp::publishEmptySelection(const std_msgs::Header &header) {
  sar_yolo_detector::TrackedDetection2DArray message;
  message.header = header;
  message.provenance.mission_id = mission_id_;
  message.provenance.uav_id = uav_id_;
  message.provenance.session_uuid = session_uuid_;
  message.provenance.sensor_id = sensor_id_;
  message.provenance.profile = profile_;
  message.provenance.model_version = model_version_;
  message.provenance.model_sha256 = model_sha256_;
  message.provenance.calibration_version = calibration_version_;
  message.provenance.coordinate_contract = coordinate_contract_;
  selected_pub_.publish(message);
}

void SmartTrackerFusionCpp::resetSelection() {
  selected_ = false;
  auto_selected_ = false;
  auto_selected_missing_since_ = 0.0;
  manual_active_ = false;
  manual_seed_ = false;
  kcf_.release();
  target_template_gray_.release();
  last_template_update_stamp_ = 0.0;
  last_template_search_stamp_ = 0.0;
  last_template_candidate_trigger_id_ = -1;
  selected_box_ = Box();
  selected_id_ = -1;
  selected_local_id_ = -1;
  selected_class_id_ = -1;
  selected_confidence_ = 0.0;
  selected_stamp_ = 0.0;
  last_observation_stamp_ = 0.0;
  last_fusion_stamp_ = 0.0;
  last_detector_for_selection_ = 0.0;
  manual_seed_without_yolo_ = false;
  manual_seed_region_ = Box();
  motion_continuous_ = true;
  long_recovery_ = false;
  appearance_anchor_.clear();
  anchor_area_ = 0.0;
  velocity_ = cv::Point2d(0.0, 0.0);
  pending_reacquire_id_ = -1;
  pending_reacquire_count_ = 0;
  pending_reacquire_stamp_ = 0.0;
  result_ = TrackResult();
  color_initialized_ = false;
  color_histogram_.clear();
  color_mask_.release();
}

bool SmartTrackerFusionCpp::selectFromSnapshot(const Snapshot &snapshot, const Box &requested,
    bool snap_to_detection, bool manual_seed, int64_t *selected_id, std::string *error) {
  if (!snapshot.image || !validBox(requested, snapshot.frame.size())) {
    if (error) *error = "ROI must lie inside the original image and be at least 4x4 pixels";
    return false;
  }
  Box chosen = requested;
  const Detection *candidate = nullptr;
  if (snap_to_detection) {
    std::vector<const Detection *> matches;
    for (const auto &item : snapshot.detections) {
      const cv::Point2d point = center(item.box);
      const bool center_inside = point.x >= requested.x && point.x <= requested.x + requested.w &&
          point.y >= requested.y && point.y <= requested.y + requested.h;
      if (boxIou(item.box, requested) >= 0.15 || center_inside) matches.push_back(&item);
    }
    if (matches.size() > 1) {
      if (error) *error = "ROI contains multiple targets; select a tighter ROI";
      return false;
    }
    if (!matches.empty()) candidate = matches.front();
  }
  if (candidate) chosen = candidate->box;
  if (!manual_enabled_) {
    if (error) *error = "ManualBoxTracker is disabled";
    return false;
  }
  if (next_id_ > 2147483647LL) {
    if (error) *error = "Fusion identity range exhausted; restart with a new session";
    return false;
  }
  resetSelection();
  template_detections_.reset();
  ++mode_epoch_;
  last_inference_wall_ = 0.0;
  selected_ = true;
  manual_active_ = true;
  manual_seed_ = manual_seed && candidate == nullptr;
  manual_seed_without_yolo_ = manual_seed_;
  manual_seed_region_ = manual_seed_ ? chosen : Box();
  selected_local_id_ = candidate ? candidate->id : -1;
  selected_class_id_ = candidate ? candidate->class_id : color_class_id_;
  selected_id_ = candidate && candidate->stable && candidate->id >= 0
      ? candidate->id : next_id_++;
  selected_box_ = chosen;
  selected_confidence_ = candidate ? candidate->confidence : 0.0;
  selected_stamp_ = snapshot.image->header.stamp.toSec();
  last_fusion_stamp_ = selected_stamp_;
  last_observation_stamp_ = selected_stamp_;
  last_detector_for_selection_ = candidate ? candidate->detector_stamp : 0.0;
  manual_seed_box_ = chosen;
  double quality = 0.0;
  appearance_anchor_ = appearanceFeature(snapshot.frame, chosen, &quality);
  anchor_area_ = chosen.w * chosen.h;
  target_template_gray_ = grayTemplate(snapshot.frame, chosen);
  last_template_update_stamp_ = selected_stamp_;
  if (mode_ == "feature_tracker") {
    try {
      kcf_ = cv::TrackerKCF::create();
      if (!kcf_) throw std::runtime_error("OpenCV KCF tracker is unavailable");
      const cv::Rect roi(static_cast<int>(std::lround(chosen.x)),
                         static_cast<int>(std::lround(chosen.y)),
                         static_cast<int>(std::lround(chosen.w)),
                         static_cast<int>(std::lround(chosen.h)));
      kcf_->init(snapshot.frame, roi);
    } catch (const cv::Exception &exception) {
      resetSelection();
      if (error) *error = std::string("could not initialize KCF tracker: ") + exception.what();
      return false;
    } catch (const std::exception &exception) {
      resetSelection();
      if (error) *error = std::string("could not initialize KCF tracker: ") + exception.what();
      return false;
    }
  }
  if (candidate) {
    result_ = {chosen, selected_id_, selected_local_id_, selected_class_id_,
               selected_confidence_, last_detector_for_selection_, "yolo", "initialized", true, true};
  } else {
    const std::string source = quality >= quality_gate_ ? "visual" : "lost";
    result_ = {chosen, selected_id_, -1, selected_class_id_, std::min(0.65, quality),
               0.0, source, "initialized", source == "visual", true};
  }
  if (selected_id) *selected_id = selected_id_;
  if (error) error->clear();
  return true;
}

bool SmartTrackerFusionCpp::selectService(sar_yolo_detector::SelectSmartTrack::Request &request,
    sar_yolo_detector::SelectSmartTrack::Response &response) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (request.clear_selection) {
    resetSelection();
    publishEmptySelection(latest_header_);
    response.success = true;
    response.message = "Selection cleared";
    response.selected_target_id = -1;
    response.tracking_active = false;
    response.measurement_current = false;
    return true;
  }
  if (mode_ != "fusion_tracker") {
    response.message = "Detection selection by ID/ROI is only available in FusionTracker";
    response.selected_target_id = -1;
    return true;
  }
  if (snapshots_.empty()) {
    response.message = "No original image is available";
    response.selected_target_id = -1;
    return true;
  }
  const Snapshot &snapshot = snapshots_.back();
  const double now = ros::Time::now().toSec();
  const double stamp = snapshot.image->header.stamp.toSec();
  if (now > 0.0 && (now - stamp > snapshot_max_age_ || stamp - now > future_tolerance_sec_)) {
    response.message = "Latest selection image is stale";
    response.selected_target_id = -1;
    return true;
  }
  Box requested;
  bool snap = true;
  if (request.use_target_id) {
    const auto found = std::find_if(snapshot.detections.begin(), snapshot.detections.end(),
        [&request](const Detection &item) { return item.id == request.target_id; });
    if (found == snapshot.detections.end()) {
      response.message = "Target ID is not present in the latest detector frame";
      response.selected_target_id = -1;
      return true;
    }
    requested = found->box;
  } else if (request.use_normalized_roi) {
    requested = {request.normalized_roi[0] * snapshot.frame.cols,
                 request.normalized_roi[1] * snapshot.frame.rows,
                 request.normalized_roi[2] * snapshot.frame.cols,
                 request.normalized_roi[3] * snapshot.frame.rows};
  } else {
    const double x = request.use_normalized_point ? request.point_x * snapshot.frame.cols : request.point_x;
    const double y = request.use_normalized_point ? request.point_y * snapshot.frame.rows : request.point_y;
    const Detection *winner = nullptr;
    double best = std::numeric_limits<double>::infinity();
    for (const auto &item : snapshot.detections) {
      const double dx = std::max({item.box.x - x, 0.0, x - item.box.x - item.box.w});
      const double dy = std::max({item.box.y - y, 0.0, y - item.box.y - item.box.h});
      const double distance = dx * dx + dy * dy;
      if (distance < best) { best = distance; winner = &item; }
    }
    if (winner == nullptr) {
      response.message = "Click does not identify a detected target";
      response.selected_target_id = -1;
      return true;
    }
    requested = winner->box;
  }
  int64_t selected_id = -1;
  std::string error;
  response.success = selectFromSnapshot(snapshot, requested, snap, false, &selected_id, &error);
  response.message = response.success ? "Fusion initialized on latest frame" : error;
  response.tracking_active = response.success;
  response.selected_target_id = response.success ? selected_id : -1;
  response.measurement_current = response.success && result_.source == "yolo";
  return true;
}

bool SmartTrackerFusionCpp::switchMode(sar_yolo_detector::SwitchTrackingMode::Request &request,
    sar_yolo_detector::SwitchTrackingMode::Response &response) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  std::string requested = lower(request.mode);
  if (requested == "fusion" || requested == "fusiontracker") requested = "fusion_tracker";
  if (requested == "feature" || requested == "featuretracker" || requested == "manual") requested = "feature_tracker";
  if (requested == "color" || requested == "colour" || requested == "colortracker" || requested == "colour_tracker") requested = "color_tracker";
  if (requested != "fusion_tracker" && requested != "feature_tracker" && requested != "color_tracker") {
    response.success = false;
    response.message = "mode must be fusion_tracker, feature_tracker, or color_tracker";
    response.active_mode = mode_;
    return true;
  }
  bool inference_service_available = true;
  if (requested == "fusion_tracker") {
    std::lock_guard<std::mutex> client_lock(inference_client_mutex_);
    inference_service_available = inference_client_.waitForExistence(ros::Duration(0.25));
  }
  if (requested == "fusion_tracker" && !inference_service_available) {
    response.success = false;
    response.active_mode = mode_;
    response.message = "Cannot enter fusion_tracker: shared YOLO inference service is unavailable at " +
        inference_base_ + "/detect";
    return true;
  }
  if (requested != "fusion_tracker" && !manual_enabled_) {
    response.success = false;
    response.message = "ManualBoxTracker is disabled";
    response.active_mode = mode_;
    return true;
  }
  std::string requested_color = lower(request.color_name);
  if (requested == "color_tracker" && !requested_color.empty() && requested_color != "red" &&
      requested_color != "green" && requested_color != "blue" && requested_color != "white" &&
      requested_color != "black" && requested_color != "custom") {
    response.success = false;
    response.message = "color_tracker/color must be red, green, blue, white, black, or custom";
    response.active_mode = mode_;
    return true;
  }
  if (requested == mode_ && (requested != "color_tracker" || requested_color.empty() || requested_color == color_name_)) {
    response.success = true;
    response.message = "Mode already active; existing target retained";
    response.active_mode = mode_;
    response.waiting_for_initial_roi = requested == "color_tracker" && color_waiting_roi_;
    response.color_source = hasConfiguredColor(color_name_) ? "configured" : "roi";
    response.color_name = color_name_;
    return true;
  }
  // An omitted color_name keeps the configured color; callers can request
  // "custom" to enter ROI-learning mode explicitly.
  if (requested == "color_tracker" && !requested_color.empty()) color_name_ = requested_color;
  mode_ = requested;
  ++mode_epoch_;
  resetSelection();
  has_pending_roi_ = false;
  template_detections_.reset();
  camera_motion_.reset();
  last_detector_detections_.clear();
  latest_detections_.clear();
  last_detector_stamp_ = 0.0;
  last_detector_for_selection_ = 0.0;
  last_inference_wall_ = 0.0;
  last_error_.clear();
  snapshots_.clear();
  snapshot_bytes_ = 0;
  auto_select_count_ = 0;
  auto_select_candidate_id_ = -1;
  if (requested != "fusion_tracker" && auto_select_enabled_) {
    auto_select_enabled_ = false;
    std_msgs::Bool status;
    status.data = false;
    auto_status_pub_.publish(status);
  }
  // A configured named color is searched over the full image immediately.
  // Empty/custom color settings wait for an ROI to learn a histogram.
  color_waiting_roi_ = requested == "color_tracker" && !hasConfiguredColor(color_name_);
  response.success = true;
  response.active_mode = mode_;
  response.waiting_for_initial_roi = requested == "feature_tracker" || color_waiting_roi_;
  response.color_source = hasConfiguredColor(color_name_) ? "configured" : "roi";
  response.color_name = color_name_;
  if (requested == "fusion_tracker") {
    response.message = "Switched to YOLO/feature fusion; select by ID, point or frame-bound ROI";
  } else if (requested == "color_tracker") {
    response.message = response.waiting_for_initial_roi
        ? "Switched to color_tracker; waiting for initial ROI; YOLO inference paused"
        : "Switched to configured color tracker; YOLO inference paused";
  } else {
    response.message = "Switched to feature_tracker; YOLO inference paused";
  }
  ROS_INFO_STREAM("Tracking mode switched to " << mode_
      << (mode_ == "fusion_tracker"
          ? "; shared YOLO inference resumed"
          : "; shared YOLO inference paused; model server remains loaded"));
  return true;
}

bool SmartTrackerFusionCpp::switchModel(sar_yolo_detector::SwitchSmartTrackerModel::Request &request,
    sar_yolo_detector::SwitchSmartTrackerModel::Response &response) {
  const std::string requested = request.model_path.empty() ? model_path_ : request.model_path;
  response.effective_device = use_gpu_ ? "cuda" : "cpu";
  response.effective_model_path = model_path_;
  {
    std::lock_guard<std::mutex> lock(labels_mutex_);
    response.class_count = static_cast<uint32_t>(labels_.size());
  }
  if (requested == model_path_ || requested == model_key_) {
    response.success = true;
    response.message = "Shared TensorRT model is already active";
  } else {
    response.success = false;
    response.message = "The shared TensorRT engine is immutable while this launch is active; change model_path for all UAVs together";
  }
  return true;
}

bool SmartTrackerFusionCpp::selectRoi(sar_yolo_detector::SelectFusionROI::Request &request,
    sar_yolo_detector::SelectFusionROI::Response &response, bool manual_seed) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (request.clear_selection) {
    resetSelection();
    publishEmptySelection(request.header);
    response.success = true;
    response.message = "Fusion selection cleared";
    response.selected_target_id = -1;
    return true;
  }
  auto found = std::find_if(snapshots_.begin(), snapshots_.end(), [&request](const Snapshot &item) {
    return item.image && item.image->header.stamp == request.header.stamp &&
           item.image->header.frame_id == request.header.frame_id;
  });
  if (found == snapshots_.end() || request.header.stamp.isZero()) {
    response.message = "Original image frame is not cached; refresh the displayed image";
    return true;
  }
  const double age = (ros::Time::now() - request.header.stamp).toSec();
  if (age > snapshot_max_age_ || age < -future_tolerance_sec_) {
    response.message = "Original image is stale or in the future";
    return true;
  }
  const Box roi{static_cast<double>(request.roi.x_offset), static_cast<double>(request.roi.y_offset),
                static_cast<double>(request.roi.width), static_cast<double>(request.roi.height)};
  if (mode_ == "color_tracker") {
    if (hasConfiguredColor(color_name_)) {
      const double roi_area = roi.w * roi.h;
      const Detection *winner = nullptr;
      double best_overlap_ratio = -1.0;
      double best_overlap = 0.0;
      for (const auto &candidate : found->detections) {
        const double left = std::max(roi.x, candidate.box.x);
        const double top = std::max(roi.y, candidate.box.y);
        const double right = std::min(roi.x + roi.w, candidate.box.x + candidate.box.w);
        const double bottom = std::min(roi.y + roi.h, candidate.box.y + candidate.box.h);
        const double overlap = std::max(0.0, right - left) * std::max(0.0, bottom - top);
        if (overlap <= 0.0) continue;
        const double box_area = candidate.box.w * candidate.box.h;
        const double ratio = overlap / std::max(1.0, std::min(roi_area, box_area));
        if (ratio > best_overlap_ratio ||
            (ratio == best_overlap_ratio && overlap > best_overlap)) {
          winner = &candidate;
          best_overlap_ratio = ratio;
          best_overlap = overlap;
        }
      }
      response.success = winner && selectConfiguredColorCandidate(*found, *winner, "color_roi_selected");
      response.message = response.success
          ? "Configured-color candidate selected by ROI"
          : "ROI did not overlap a detected color box";
      response.selected_target_id = response.success ? manual_track_id_ : -1;
      return true;
    }
    response.success = initializeColorFromRoi(*found, roi);
    response.message = response.success
        ? "ColorTracker initialized from the selected ROI; YOLO inference remains paused"
        : "ColorTracker ROI is invalid or has no usable color histogram";
    response.selected_target_id = response.success ? manual_track_id_ : -1;
    return true;
  }
  int64_t selected_id = -1;
  std::string error;
  response.success = selectFromSnapshot(*found, roi, !manual_seed, manual_seed, &selected_id, &error);
  response.message = response.success ? (manual_seed
      ? "Template initialized from the requested manual ROI; YOLO will validate and correct nearby matches"
      : "Fusion initialized from the requested original frame") : error;
  response.selected_target_id = response.success ? selected_id : -1;
  return true;
}

bool SmartTrackerFusionCpp::selectPoint(sar_yolo_detector::SelectFusionPoint::Request &request,
    sar_yolo_detector::SelectFusionPoint::Response &response) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  auto found = std::find_if(snapshots_.begin(), snapshots_.end(), [&request](const Snapshot &item) {
    return item.image && item.image->header.stamp == request.header.stamp &&
           item.image->header.frame_id == request.header.frame_id;
  });
  if (found == snapshots_.end() || request.header.stamp.isZero()) {
    response.message = "Original image frame is not cached; refresh the displayed image";
    return true;
  }
  const double age = (ros::Time::now() - request.header.stamp).toSec();
  if (age > snapshot_max_age_ || age < -future_tolerance_sec_) {
    response.message = "Original image frame is stale or in the future";
    return true;
  }
  const double x = request.point.x, y = request.point.y;
  if (!std::isfinite(x) || !std::isfinite(y) || x < 0 || y < 0 || x >= found->frame.cols || y >= found->frame.rows) {
    response.message = "Point lies outside the original image";
    return true;
  }
  std::vector<const Detection *> matches;
  for (const auto &item : found->detections) {
    if (x >= item.box.x && x <= item.box.x + item.box.w && y >= item.box.y && y <= item.box.y + item.box.h)
      matches.push_back(&item);
  }
  if (matches.size() != 1) {
    response.message = matches.empty() ? "Point must identify exactly one YOLO detection"
                                       : "Point overlaps multiple detections; choose an unambiguous point";
    return true;
  }
  int64_t selected_id = -1;
  std::string error;
  response.success = selectFromSnapshot(*found, matches.front()->box, true, false, &selected_id, &error);
  response.selected_target_id = response.success ? selected_id : -1;
  response.message = response.success ? "Fusion reinitialized from the selected YOLO detection" : error;
  return true;
}

bool SmartTrackerFusionCpp::autoSelect(std_srvs::SetBool::Request &request,
    std_srvs::SetBool::Response &response) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  const bool color_search_ready = mode_ == "color_tracker" &&
      (hasConfiguredColor(color_name_) || color_initialized_);
  const bool supported = mode_ == "fusion_tracker" || color_search_ready;
  if (request.data && !supported) {
    auto_select_enabled_ = false;
    response.success = false;
    response.message = "Auto selection requires FusionTracker or a configured color/initialized ColorTracker ROI";
  } else {
    auto_select_enabled_ = request.data;
    if (mode_ == "color_tracker") {
      color_waiting_roi_ = !color_initialized_ && !hasConfiguredColor(color_name_);
    }
    response.success = true;
    response.message = request.data
        ? (mode_ == "color_tracker"
            ? (hasConfiguredColor(color_name_)
                ? "Configured-color auto selection armed; it resumes after target loss"
                : "ROI-learned color auto selection armed; it resumes after target loss")
            : "Auto target selection armed; it resumes after target loss")
        : "Auto target selection stopped";
  }
  auto_select_count_ = 0;
  auto_select_candidate_id_ = -1;
  std_msgs::Bool status;
  status.data = auto_select_enabled_;
  auto_status_pub_.publish(status);
  return true;
}

void SmartTrackerFusionCpp::pointCallback(const geometry_msgs::PointStampedConstPtr &point) {
  if (!point) return;
  std::lock_guard<std::mutex> lock(state_mutex_);
  const double x = point->point.x, y = point->point.y;
  if (!std::isfinite(x) || !std::isfinite(y)) return;
  if (x < 0.0 || y < 0.0) {
    resetSelection();
    publishEmptySelection(latest_header_);
    return;
  }
  if (snapshots_.empty()) return;
  const Snapshot &snapshot = snapshots_.back();
  if (x >= snapshot.frame.cols || y >= snapshot.frame.rows) return;
  if (mode_ == "fusion_tracker") {
    const Detection *winner = nullptr;
    double best = std::numeric_limits<double>::infinity();
    for (const auto &item : snapshot.detections) {
      const double dx = std::max({item.box.x - x, 0.0, x - item.box.x - item.box.w});
      const double dy = std::max({item.box.y - y, 0.0, y - item.box.y - item.box.h});
      const double distance = dx * dx + dy * dy;
      if (distance < best) { best = distance; winner = &item; }
    }
    if (!winner) return;
    int64_t id = -1;
    std::string error;
    selectFromSnapshot(snapshot, winner->box, true, false, &id, &error);
  } else if (mode_ == "color_tracker" && hasConfiguredColor(color_name_)) {
    const Detection *winner = nullptr;
    double best_distance = std::numeric_limits<double>::infinity();
    double best_center_distance = std::numeric_limits<double>::infinity();
    double best_confidence = -1.0;
    for (const auto &candidate : snapshot.detections) {
      const double dx = std::max({candidate.box.x - x, 0.0,
                                  x - candidate.box.x - candidate.box.w});
      const double dy = std::max({candidate.box.y - y, 0.0,
                                  y - candidate.box.y - candidate.box.h});
      const double distance = dx * dx + dy * dy;
      const cv::Point2d candidate_center = center(candidate.box);
      const double center_distance = std::pow(candidate_center.x - x, 2.0) +
          std::pow(candidate_center.y - y, 2.0);
      if (distance < best_distance ||
          (distance == best_distance && center_distance < best_center_distance) ||
          (distance == best_distance && center_distance == best_center_distance &&
           candidate.confidence > best_confidence)) {
        winner = &candidate;
        best_distance = distance;
        best_center_distance = center_distance;
        best_confidence = candidate.confidence;
      }
    }
    if (winner) {
      selectConfiguredColorCandidate(snapshot, *winner, "color_pixel_selected");
    } else {
      resetSelection();
      publishEmptySelection(snapshot.image->header);
      ROS_WARN_THROTTLE(2.0, "Ignoring color target click: no matching color boxes");
    }
  } else {
    const int side = std::min({20, snapshot.frame.cols, snapshot.frame.rows});
    const int x0 = std::max(0, std::min(snapshot.frame.cols - side, static_cast<int>(std::lround(x)) - side / 2));
    const int y0 = std::max(0, std::min(snapshot.frame.rows - side, static_cast<int>(std::lround(y)) - side / 2));
    const Box roi{static_cast<double>(x0), static_cast<double>(y0),
                  static_cast<double>(side), static_cast<double>(side)};
    if (mode_ == "color_tracker") {
      if (!initializeColorFromRoi(snapshot, roi)) {
        ROS_WARN_THROTTLE(2.0, "ColorTracker click ROI is too small or has no usable color histogram");
      }
    } else {
      pending_roi_ = roi;
      has_pending_roi_ = true;
    }
  }
}

void SmartTrackerFusionCpp::manualRoiCallback(const sensor_msgs::RegionOfInterestConstPtr &roi) {
  if (!roi) return;
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (snapshots_.empty()) return;
  const Snapshot &snapshot = snapshots_.back();
  if (mode_ == "color_tracker" && (roi->width == 0 || roi->height == 0)) {
    resetSelection();
    color_waiting_roi_ = !hasConfiguredColor(color_name_);
    publishEmptySelection(snapshot.image ? snapshot.image->header : latest_header_);
    return;
  }
  const Box box{static_cast<double>(roi->x_offset), static_cast<double>(roi->y_offset),
                static_cast<double>(roi->width), static_cast<double>(roi->height)};
  if (!validBox(box, snapshot.frame.size())) return;
  if (mode_ == "color_tracker") {
    if (hasConfiguredColor(color_name_)) {
      const double roi_area = box.w * box.h;
      const Detection *winner = nullptr;
      double best_overlap_ratio = -1.0;
      double best_overlap = 0.0;
      for (const auto &candidate : snapshot.detections) {
        const double left = std::max(box.x, candidate.box.x);
        const double top = std::max(box.y, candidate.box.y);
        const double right = std::min(box.x + box.w, candidate.box.x + candidate.box.w);
        const double bottom = std::min(box.y + box.h, candidate.box.y + candidate.box.h);
        const double overlap = std::max(0.0, right - left) * std::max(0.0, bottom - top);
        if (overlap <= 0.0) continue;
        const double box_area = candidate.box.w * candidate.box.h;
        const double ratio = overlap / std::max(1.0, std::min(roi_area, box_area));
        if (ratio > best_overlap_ratio ||
            (ratio == best_overlap_ratio && overlap > best_overlap)) {
          winner = &candidate;
          best_overlap_ratio = ratio;
          best_overlap = overlap;
        }
      }
      if (!winner || !selectConfiguredColorCandidate(snapshot, *winner, "color_roi_selected")) {
        ROS_WARN_THROTTLE(2.0, "Color ROI did not overlap any detected color box");
      }
      return;
    }
    if (!initializeColorFromRoi(snapshot, box)) {
      ROS_WARN_THROTTLE(2.0, "ColorTracker ROI has no usable color histogram");
    }
    return;
  }
  int64_t id = -1;
  std::string error;
  selectFromSnapshot(snapshot, box, mode_ == "fusion_tracker", true, &id, &error);
}

void SmartTrackerFusionCpp::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &message) {
  if (!message) return;
  std::lock_guard<std::mutex> lock(state_mutex_);
  camera_info_ = *message;
  cv::Mat k(3, 3, CV_64F);
  for (int i = 0; i < 9; ++i) k.at<double>(i / 3, i % 3) = message->K[i];
  camera_info_k_ = k;
  camera_frame_id_ = message->header.frame_id;
  camera_info_valid_ = k.at<double>(0, 0) > 0 && k.at<double>(1, 1) > 0;
}

cv::Mat SmartTrackerFusionCpp::poseRotation(const sensor_msgs::Image &message) {
  if (!camera_info_valid_ || latest_header_.stamp.isZero() ||
      latest_header_.frame_id != message.header.frame_id ||
      camera_frame_id_ != message.header.frame_id ||
      camera_info_.width != message.width || camera_info_.height != message.height) return {};
  if (!camera_info_.header.stamp.isZero() &&
      std::abs((message.header.stamp - camera_info_.header.stamp).toSec()) > 0.5) return {};
  for (double distortion : camera_info_.D) if (std::abs(distortion) > 1e-8) return {};
  try {
    const auto previous = tf_buffer_.lookupTransform(motion_fixed_frame_, message.header.frame_id,
        latest_header_.stamp, ros::Duration(0.0));
    const auto current = tf_buffer_.lookupTransform(motion_fixed_frame_, message.header.frame_id,
        message.header.stamp, ros::Duration(0.0));
    const cv::Mat previous_rotation = rotationFromQuaternion(previous.transform.rotation);
    const cv::Mat current_rotation = rotationFromQuaternion(current.transform.rotation);
    if (previous_rotation.empty() || current_rotation.empty()) return {};
    cv::Mat homography = camera_info_k_ * current_rotation.t() * previous_rotation * camera_info_k_.inv();
    if (std::abs(homography.at<double>(2, 2)) < 1e-9) return {};
    homography /= homography.at<double>(2, 2);
    return validateHomography(homography, cv::Size(message.width, message.height)) ? homography : cv::Mat();
  } catch (const tf2::TransformException &) {
    return {};
  }
}

}  // namespace

int main(int argc, char **argv) {
  ros::init(argc, argv, "sar_yolo_smart_tracker_cpp");
  try {
    SmartTrackerFusionCpp node;
    ros::AsyncSpinner spinner(4);
    spinner.start();
    node.run();
  } catch (const std::exception &error) {
    ROS_FATAL_STREAM("C++ SmartTracker stopped: " << error.what());
    return 1;
  }
  return 0;
}

