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

double boxIou(const Box &a, const Box &b) {
  const double left = std::max(a.x, b.x);
  const double top = std::max(a.y, b.y);
  const double right = std::min(a.x + a.w, b.x + b.w);
  const double bottom = std::min(a.y + a.h, b.y + b.h);
  const double area = std::max(0.0, right - left) * std::max(0.0, bottom - top);
  return area / std::max(1.0, a.w * a.h + b.w * b.h - area);
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
  if (!validBox(box, frame.size())) return {};
  const cv::Rect roi(static_cast<int>(box.x), static_cast<int>(box.y),
                     static_cast<int>(box.w), static_cast<int>(box.h));
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

class FlowDetections {
 public:
  void reset() { tracks_.clear(); previous_gray_.release(); previous_stamp_ = 0.0; }

  std::vector<Detection> update(const cv::Mat &frame, double stamp,
      const std::vector<Detection> &detections, bool detector_updated,
      const cv::Mat &camera_transform, double max_age) {
    if (detector_updated) {
      tracks_.clear();
      for (const auto &detection : detections) {
        if (tracks_.size() >= 30) break;
        if (!detection.stable || detection.id < 0 || !validBox(detection.box, frame.size())) continue;
        FlowTrack track;
        track.detection = detection;
        track.box = detection.box;
        track.seed_area = std::max(1.0, detection.box.w * detection.box.h);
        tracks_.push_back(std::move(track));
      }
      previous_gray_.release();
      previous_stamp_ = stamp;
      if (tracks_.empty()) return detections;
      cv::cvtColor(frame, previous_gray_, cv::COLOR_BGR2GRAY);
      for (auto &track : tracks_) seed(previous_gray_, track);
      return detections;
    }
    if (tracks_.empty()) {
      previous_gray_.release();
      previous_stamp_ = stamp;
      // If this detector frame could not seed LK tracks, keep the latest
      // detector boxes visible between rate-limited inference calls until they expire.
      std::vector<Detection> cached;
      for (const auto &detection : detections) {
        if (detection.detector_stamp > 0.0 && stamp >= detection.detector_stamp &&
            stamp - detection.detector_stamp <= max_age &&
            validBox(detection.box, frame.size())) {
          cached.push_back(detection);
        }
      }
      return cached;
    }
    const double dt = stamp - previous_stamp_;
    if (dt <= 0.0 || dt > max_age || previous_gray_.empty()) {
      tracks_.clear();
      previous_gray_.release();
      previous_stamp_ = stamp;
      return {};
    }
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    advance(gray, camera_transform);
    previous_gray_ = gray;
    previous_stamp_ = stamp;
    if (tracks_.empty()) {
      // Keep the last YOLO boxes visible on the frame where LK loses its
      // points; they remain eligible until the detector age limit expires.
      std::vector<Detection> cached;
      for (const auto &detection : detections) {
        if (detection.detector_stamp > 0.0 && stamp >= detection.detector_stamp &&
            stamp - detection.detector_stamp <= max_age &&
            validBox(detection.box, frame.size())) {
          cached.push_back(detection);
        }
      }
      return cached;
    }
    std::vector<Detection> output;
    for (const auto &track : tracks_) {
      if (stamp - track.detection.detector_stamp > max_age) continue;
      Detection detection = track.detection;
      detection.box = track.box;
      detection.source = "visual_tracker";
      detection.confidence = std::min(0.65, detection.confidence);
      detection.flow_quality = track.quality;
      detection.flow_scale_ratio = track.area_ratio;
      detection.geometry = "aabb";
      detection.has_rotation = false;
      output.push_back(std::move(detection));
    }
    return output;
  }

 private:
  struct FlowTrack {
    Detection detection;
    Box box;
    std::vector<cv::Point2f> points;
    std::vector<cv::Point2f> corners;
    double seed_area{1.0};
    double quality{0.0};
    double area_ratio{1.0};
  };

  static void seed(const cv::Mat &gray, FlowTrack &track) {
    const cv::Rect roi(static_cast<int>(track.box.x), static_cast<int>(track.box.y),
        static_cast<int>(track.box.w), static_cast<int>(track.box.h));
    cv::Mat crop = gray(roi);
    std::vector<cv::Point2f> points;
    cv::goodFeaturesToTrack(crop, points, 16, 0.01, 3.0);
    for (auto &point : points) point += cv::Point2f(static_cast<float>(roi.x), static_cast<float>(roi.y));
    track.points = std::move(points);
    const float x = static_cast<float>(track.box.x), y = static_cast<float>(track.box.y);
    const float right = static_cast<float>(track.box.x + track.box.w);
    const float bottom = static_cast<float>(track.box.y + track.box.h);
    track.corners = {{x, y}, {right, y}, {right, bottom}, {x, bottom}};
    if (track.points.size() >= 6) {
      const cv::RotatedRect rotated = cv::minAreaRect(track.points);
      cv::Point2f fitted[4];
      rotated.points(fitted);
      std::vector<cv::Point2f> contour(fitted, fitted + 4);
      const double fitted_area = std::abs(cv::contourArea(contour));
      if (fitted_area >= 16.0) {
        track.corners = std::move(contour);
        track.seed_area = std::max(track.seed_area, fitted_area);
      }
    }
  }

  void advance(const cv::Mat &gray, const cv::Mat &camera_transform) {
    std::vector<size_t> groups;
    std::vector<cv::Point2f> before;
    std::vector<size_t> offsets;
    for (size_t i = 0; i < tracks_.size(); ++i) {
      if (tracks_[i].points.size() < 4) continue;
      groups.push_back(i);
      offsets.push_back(before.size());
      before.insert(before.end(), tracks_[i].points.begin(), tracks_[i].points.end());
    }
    if (groups.empty()) { tracks_.clear(); return; }

    const double scale = std::min(1.0, 480.0 / gray.cols);
    cv::Mat old_small = previous_gray_, new_small = gray;
    if (scale < 1.0) {
      const cv::Size size(std::max(1, static_cast<int>(std::lround(gray.cols * scale))),
                          std::max(1, static_cast<int>(std::lround(gray.rows * scale))));
      cv::resize(previous_gray_, old_small, size, 0.0, 0.0, cv::INTER_AREA);
      cv::resize(gray, new_small, size, 0.0, 0.0, cv::INTER_AREA);
    }
    const bool have_motion = validateHomography(camera_transform, gray.size());
    std::vector<cv::Point2f> initial = before;
    if (have_motion) cv::perspectiveTransform(before, initial, camera_transform);
    std::vector<cv::Point2f> before_small = before, initial_small = initial;
    for (auto &point : before_small) point *= static_cast<float>(scale);
    for (auto &point : initial_small) point *= static_cast<float>(scale);
    std::vector<cv::Point2f> after_small, back_small;
    std::vector<uint8_t> forward_status, backward_status;
    std::vector<float> errors;
    try {
      cv::calcOpticalFlowPyrLK(old_small, new_small, before_small, after_small,
          forward_status, errors, cv::Size(21, 21), 2,
          cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 30, 0.01),
          have_motion ? cv::OPTFLOW_USE_INITIAL_FLOW : 0);
      if (after_small.empty()) { tracks_.clear(); return; }
      cv::calcOpticalFlowPyrLK(new_small, old_small, after_small, back_small,
          backward_status, errors, cv::Size(21, 21), 2,
          cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 30, 0.01));
    } catch (const cv::Exception &) { tracks_.clear(); return; }

    std::vector<uint8_t> accepted(before.size(), 0);
    std::vector<cv::Point2f> after(before.size());
    for (size_t i = 0; i < before.size(); ++i) {
      if (i >= forward_status.size() || i >= backward_status.size() ||
          !forward_status[i] || !backward_status[i]) continue;
      const cv::Point2f back = back_small[i] * static_cast<float>(1.0 / scale);
      after[i] = after_small[i] * static_cast<float>(1.0 / scale);
      if (cv::norm(back - before[i]) <= 1.5 && std::isfinite(after[i].x) && std::isfinite(after[i].y))
        accepted[i] = 1;
    }
    std::vector<FlowTrack> kept;
    for (size_t g = 0; g < groups.size(); ++g) {
      FlowTrack track = tracks_[groups[g]];
      const size_t start = offsets[g], count = track.points.size();
      std::vector<cv::Point2f> old_points, new_points, expected;
      for (size_t i = 0; i < count; ++i) {
        const size_t k = start + i;
        if (!accepted[k]) continue;
        old_points.push_back(before[k]);
        new_points.push_back(after[k]);
        expected.push_back(have_motion ? initial[k] : before[k]);
      }
      if (new_points.size() < std::max<size_t>(4, (count + 1) / 2)) continue;
      cv::Mat affine, inliers;
      try { affine = cv::estimateAffinePartial2D(expected, new_points, inliers,
          cv::RANSAC, 2.0, 100, 0.99); }
      catch (const cv::Exception &) { affine.release(); }
      cv::Mat object_motion = cv::Mat::eye(3, 3, CV_64F);
      std::vector<uint8_t> support(new_points.size(), 1);
      if (!affine.empty() && !inliers.empty()) {
        int inlier_count = 0;
        std::vector<cv::Point2f> supported;
        for (int i = 0; i < inliers.rows; ++i) {
          if (inliers.at<uint8_t>(i, 0)) {
            ++inlier_count;
            supported.push_back(expected[static_cast<size_t>(i)]);
          } else support[static_cast<size_t>(i)] = 0;
        }
        const double scale_change = std::hypot(affine.at<double>(0, 0), affine.at<double>(1, 0));
        const double angle = std::abs(std::atan2(affine.at<double>(1, 0), affine.at<double>(0, 0)));
        double spread_x = 0.0, spread_y = 0.0;
        if (!supported.empty()) {
          auto minmax_x = std::minmax_element(supported.begin(), supported.end(),
              [](const cv::Point2f &a, const cv::Point2f &b) { return a.x < b.x; });
          auto minmax_y = std::minmax_element(supported.begin(), supported.end(),
              [](const cv::Point2f &a, const cv::Point2f &b) { return a.y < b.y; });
          spread_x = minmax_x.second->x - minmax_x.first->x;
          spread_y = minmax_y.second->y - minmax_y.first->y;
        }
        if (inlier_count >= std::max(4, static_cast<int>(std::ceil(0.6 * new_points.size()))) &&
            std::min(spread_x, spread_y) >= 3.0 && scale_change >= 0.80 && scale_change <= 1.25 &&
            angle <= 35.0 * CV_PI / 180.0) {
          affine.copyTo(object_motion(cv::Rect(0, 0, 3, 2)));
        } else {
          std::vector<double> dx, dy;
          for (size_t i = 0; i < expected.size(); ++i) {
            dx.push_back(new_points[i].x - expected[i].x);
            dy.push_back(new_points[i].y - expected[i].y);
          }
          auto median = [](std::vector<double> &values) {
            const size_t mid = values.size() / 2;
            std::nth_element(values.begin(), values.begin() + mid, values.end());
            return values[mid];
          };
          object_motion.at<double>(0, 2) = median(dx);
          object_motion.at<double>(1, 2) = median(dy);
          std::fill(support.begin(), support.end(), 1);
        }
      }
      std::vector<cv::Point2f> moved_expected;
      cv::perspectiveTransform(expected, moved_expected, object_motion);
      std::vector<double> residuals;
      for (size_t i = 0; i < new_points.size(); ++i) if (support[i])
        residuals.push_back(cv::norm(moved_expected[i] - new_points[i]));
      if (residuals.empty()) continue;
      std::sort(residuals.begin(), residuals.end());
      const double median_residual = residuals[residuals.size() / 2];
      if (median_residual > 3.0) continue;
      cv::Mat total_motion = object_motion;
      if (have_motion) total_motion = object_motion * camera_transform;
      std::vector<cv::Point2f> corners;
      cv::perspectiveTransform(track.corners, corners, total_motion);
      if (corners.size() != 4) continue;
      float left = corners[0].x, right = left, top = corners[0].y, bottom = top;
      for (const auto &point : corners) {
        if (!std::isfinite(point.x) || !std::isfinite(point.y)) { left = right = top = bottom = 0; break; }
        left = std::min(left, point.x); right = std::max(right, point.x);
        top = std::min(top, point.y); bottom = std::max(bottom, point.y);
      }
      const Box clipped{std::max(0.0, static_cast<double>(left)),
          std::max(0.0, static_cast<double>(top)),
          std::min(static_cast<double>(gray.cols), static_cast<double>(right)) - std::max(0.0, static_cast<double>(left)),
          std::min(static_cast<double>(gray.rows), static_cast<double>(bottom)) - std::max(0.0, static_cast<double>(top))};
      const double full_area = std::max(0.0, static_cast<double>(right-left)) * std::max(0.0, static_cast<double>(bottom-top));
      if (!validBox(clipped, gray.size()) || clipped.w * clipped.h < 0.25 * std::max(1.0, full_area)) continue;
      std::vector<cv::Point2f> inside;
      for (size_t i = 0; i < new_points.size(); ++i) {
        if (support[i] && new_points[i].x >= 0 && new_points[i].x < gray.cols &&
            new_points[i].y >= 0 && new_points[i].y < gray.rows) inside.push_back(new_points[i]);
      }
      if (inside.size() < 4) continue;
      track.box = clipped;
      track.corners = std::move(corners);
      track.points = std::move(inside);
      track.area_ratio = std::abs(cv::contourArea(track.corners)) / std::max(1.0, track.seed_area);
      track.quality = static_cast<double>(track.points.size()) / std::max<size_t>(1, new_points.size()) /
          (1.0 + median_residual / 3.0);
      kept.push_back(std::move(track));
    }
    tracks_ = std::move(kept);
  }

  std::vector<FlowTrack> tracks_;
  cv::Mat previous_gray_;
  double previous_stamp_{0.0};
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
  void loadConfig();
  void createRosApi();
  void startWorker();
  void stopWorker();
  void imageCallback(const sensor_msgs::ImageConstPtr &image);
  void processLoop();
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
                                      const std::string &reason);
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
  std::atomic<bool> stopping_{false};
  std::mutex state_mutex_;
  std::mutex phase_mutex_;

  std::string model_path_;
  std::string model_key_;
  std::string inference_base_;
  std::string source_id_;
  std::string input_topic_;
  std::string camera_info_topic_;
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
  int kcf_reseed_count_{0};
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
  double appearance_gate_{0.70};
  double quality_gate_{0.35};
  double long_appearance_gate_{0.85};
  double long_quality_gate_{0.65};
  double long_score_gate_{0.65};
  double ambiguity_margin_{0.12};
  double center_gate_sizes_{1.4};
  double max_area_ratio_{2.5};
  double detector_weight_{0.70};
  double detector_authority_{0.85};
  double min_detector_confidence_{0.50};
  double min_maintenance_confidence_{0.10};
  double local_template_refresh_sec_{0.75};
  double last_local_seed_stamp_{0.0};
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
  FlowDetections flow_detections_;
  cv::Mat previous_rotation_;
  std::string camera_frame_id_;
  bool camera_info_valid_{false};
  double processed_frames_{0.0};
  double inference_failures_{0.0};
  std::string last_error_;
  bool auto_select_enabled_{false};
  int auto_select_stable_frames_{5};
  int auto_select_count_{0};
  int auto_select_candidate_id_{-1};
  int auto_select_candidate_class_{-1};
  double auto_select_min_confidence_{0.0};
  double auto_select_max_gap_{1.0};
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
  pnh_.param("FusionTracker/appearance_gate", appearance_gate_, 0.70);
  pnh_.param("FusionTracker/quality_gate", quality_gate_, 0.35);
  pnh_.param("FusionTracker/long_reacquire_appearance_gate", long_appearance_gate_, 0.85);
  pnh_.param("FusionTracker/long_reacquire_quality_gate", long_quality_gate_, 0.65);
  pnh_.param("FusionTracker/long_reacquire_minimum_score", long_score_gate_, 0.65);
  pnh_.param("FusionTracker/ambiguity_margin", ambiguity_margin_, 0.12);
  pnh_.param("FusionTracker/center_gate_sizes", center_gate_sizes_, 1.4);
  pnh_.param("FusionTracker/max_area_ratio", max_area_ratio_, 2.5);
  pnh_.param("FusionTracker/detector_weight", detector_weight_, 0.70);
  pnh_.param("FusionTracker/detector_authority_confidence", detector_authority_, 0.85);
  pnh_.param("FusionTracker/minimum_detector_confidence", min_detector_confidence_, 0.50);
  pnh_.param("FusionTracker/minimum_maintenance_confidence", min_maintenance_confidence_, 0.10);
  pnh_.param("FusionTracker/local_template_refresh_sec", local_template_refresh_sec_, 0.75);
  pnh_.param("FusionTracker/reacquire_confirm_frames", confirmation_frames_, 3);
  detector_id_memory_sec_ = std::max(0.0, detector_id_memory_sec_);
  reacquire_detector_gain_ = clamp01(reacquire_detector_gain_);
  pnh_.param("FusionTracker/selection_max_frames", snapshot_max_frames_, 24);
  int snapshot_bytes = 25165824;
  pnh_.param("FusionTracker/selection_max_bytes", snapshot_bytes, snapshot_bytes);
  snapshot_max_bytes_ = static_cast<size_t>(std::max(1024, snapshot_bytes));
  pnh_.param("FusionTracker/selection_max_age_sec", snapshot_max_age_, 0.75);
  if (identity_ttl_sec_ <= reacquire_hold_sec_ || identity_ttl_sec_ > 300.0) {
    throw std::runtime_error("FusionTracker identity_memory_ttl_sec must exceed reacquire_hold_sec and be <= 300");
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
  pnh_.param("AutoSelect/stable_frames", auto_select_stable_frames_, 5);
  pnh_.param("AutoSelect/minimum_confidence", auto_select_min_confidence_, 0.0);
  pnh_.param("AutoSelect/max_frame_gap_sec", auto_select_max_gap_, 1.0);
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

  // The model server is elected from several standby nodes. Do not retain a
  // TCPROS connection to a provider that may have released the model lock.
  info_client_ = nh_.serviceClient<sar_yolo_detector::GetSharedModelInfo>(inference_base_ + "/info", false);
  inference_client_ = nh_.serviceClient<sar_yolo_detector::RunSharedInference>(inference_base_ + "/detect", false);
  ROS_INFO_STREAM("C++ SmartTracker ROS API ready; image=" << input_topic_ << " annotated=" << annotated_topic_
      << " inference=" << inference_base_);
}

void SmartTrackerFusionCpp::startWorker() {
  stopping_ = false;
  worker_ = std::thread(&SmartTrackerFusionCpp::processLoop, this);
}

void SmartTrackerFusionCpp::stopWorker() {
  stopping_ = true;
  image_cv_.notify_all();
  if (worker_.joinable()) worker_.join();
}

void SmartTrackerFusionCpp::imageCallback(const sensor_msgs::ImageConstPtr &image) {
  {
    std::lock_guard<std::mutex> lock(image_mutex_);
    pending_image_ = image;
  }
  image_cv_.notify_one();
}

std::string SmartTrackerFusionCpp::className(int class_id) const {
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
          if (labels.IsMap()) {
            for (const auto &entry : labels) {
              labels_[std::stoi(entry.first.as<std::string>())] = entry.second.as<std::string>();
            }
          }
        } catch (const std::exception &error) {
          ROS_WARN_STREAM_THROTTLE(5.0, "Could not parse shared-model labels: " << error.what());
        }
        model_info_loaded_ = true;
      }
    }
  }
  if (!inference_client_.waitForExistence(ros::Duration(0.5))) {
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
  const bool service_call_succeeded = inference_client_.call(call);
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
      << " service_ms=" << std::fixed << std::setprecision(1) << service_ms);
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
      flow_detections_.reset();
      snapshots_.clear();
      snapshot_bytes_ = 0;
      camera_motion_.reset();
      latest_frame_.release();
      previous_frame_.release();
      last_detector_detections_.clear();
      last_detector_stamp_ = 0.0;
      last_capture_stamp_ = 0.0;
      last_inference_wall_ = 0.0;
      ROS_WARN("Capture timestamp moved backwards; starting a new tracker segment");
    }
    if (!latest_frame_.empty() && (latest_frame_.size() != frame.size() ||
                                   latest_header_.frame_id != image->header.frame_id)) {
      resetSelection();
      flow_detections_.reset();
      snapshots_.clear();
      snapshot_bytes_ = 0;
      camera_motion_.reset();
      last_detector_detections_.clear();
      last_detector_stamp_ = 0.0;
      previous_frame_.release();
    }
    last_capture_stamp_ = stamp;
    frame_mode = mode_;
    frame_mode_epoch = mode_epoch_;
    run_inference = frame_mode == "fusion_tracker" &&
        (inference_max_fps_ <= 0.0 || last_inference_wall_ == 0.0 ||
         wall_now - last_inference_wall_ >= 1.0 / inference_max_fps_);
    if (run_inference) last_inference_wall_ = wall_now;
  }

  std::vector<Detection> new_detections;
  double inference_ms = 0.0;
  bool inference_succeeded = false;
  if (run_inference) {
    const auto started = SteadyClock::now();
    if (requestDetections(frame, image->header, &new_detections)) {
      inference_succeeded = true;
      inference_ms = std::chrono::duration<double, std::milli>(SteadyClock::now() - started).count();
    } else {
      std::lock_guard<std::mutex> lock(state_mutex_);
      ++inference_failures_;
    }
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
    if (run_inference && inference_result_current && inference_succeeded) {
      // The shared C++ TensorRT provider returns frame-local negative IDs.
      // Associate each new YOLO box to the previous detector frame so Fusion
      // and KCF can keep a stable local identity across inference updates.
      std::vector<bool> previous_used(last_detector_detections_.size(), false);
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
          detection.id = next_detector_id_++;
        }
        detection.stable = true;
      }
      last_detector_detections_ = std::move(new_detections);
      last_detector_stamp_ = stamp;
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
      current_detections = flow_detections_.update(frame, stamp, current_detections,
          fresh, motion, max_frame_gap_sec_);
    } else {
      current_detections.clear();
      motion.release();
    }

    if (selected_ && kcf_ && !motion.empty() && !previous_frame_.empty() &&
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
        const cv::Point2d new_center = center(transformed);
        const double center_shift = cv::norm(new_center - old_center);
        const double width_change = std::abs(transformed.w - selected_box_.w) /
            std::max(1.0, selected_box_.w);
        const double height_change = std::abs(transformed.h - selected_box_.h) /
            std::max(1.0, selected_box_.h);
        // KCF already updates on the current frame. Re-warping the entire
        // previous frame and rebuilding the filter for subpixel/near-static
        // motion adds CPU cost without a meaningful correction.
        constexpr double kMinCompensationShiftPx = 0.75;
        constexpr double kMinCompensationScaleChange = 0.005;
        const bool compensation_needed = center_shift >= kMinCompensationShiftPx ||
            width_change >= kMinCompensationScaleChange ||
            height_change >= kMinCompensationScaleChange;
        if (compensation_needed) {
          if (pending_reacquire_count_ > 0) pending_reacquire_box_ = transformBox(pending_reacquire_box_, motion);
          cv::Mat warped_previous;
          cv::warpPerspective(previous_frame_, warped_previous, motion, frame.size());
          selected_box_ = transformed;
          try {
            kcf_ = cv::TrackerKCF::create();
            const cv::Rect roi(static_cast<int>(std::lround(transformed.x)),
                static_cast<int>(std::lround(transformed.y)),
                static_cast<int>(std::lround(transformed.w)),
                static_cast<int>(std::lround(transformed.h)));
            kcf_->init(warped_previous, roi);
          } catch (const cv::Exception &) {
            kcf_.release();
          }
        }
      }
    }

    if (mode_ == "fusion_tracker") {
      const Detection *flow_observation = nullptr;
      if (selected_local_id_ >= 0) {
        const auto found = std::find_if(current_detections.begin(), current_detections.end(),
            [this](const Detection &detection) {
              return detection.id == selected_local_id_ && detection.stable &&
                  detection.source == "visual_tracker";
            });
        if (found != current_detections.end()) flow_observation = &*found;
      }
      processFusion(frame, stamp, current_detections, fresh, motion, flow_observation);
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

    latest_frame_ = frame;
    previous_frame_ = frame;
    latest_header_ = image->header;
    latest_detections_ = current_detections;
    snapshots_.push_back({image, frame, current_detections});
    snapshot_bytes_ += static_cast<size_t>(frame.total() * frame.elemSize());
    while (!snapshots_.empty() && (snapshots_.size() > static_cast<size_t>(snapshot_max_frames_) ||
           image->header.stamp.toSec() - snapshots_.front().image->header.stamp.toSec() > snapshot_max_age_ ||
           snapshot_bytes_ > snapshot_max_bytes_)) {
      snapshot_bytes_ -= static_cast<size_t>(snapshots_.front().frame.total() * snapshots_.front().frame.elemSize());
      snapshots_.pop_front();
    }
    if ((fresh || mode_ == "color_tracker") && auto_select_enabled_ && !selected_)
      updateAutoSelection(current_detections, frame, image);
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
  if (absence > identity_ttl_sec_) {
    kcf_.release();
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
  if (step > max_frame_gap_sec_ || long_recovery_) {
    kcf_.release();
    if (step > max_frame_gap_sec_) {
      pending_reacquire_id_ = -1;
      pending_reacquire_count_ = 0;
    }
  }

  Box visual;
  bool visual_valid = false;
  double visual_quality = 0.0;
  std::vector<float> visual_feature;
  if (kcf_ && step <= max_frame_gap_sec_ && !long_recovery_) {
    cv::Rect tracked(static_cast<int>(std::lround(selected_box_.x)),
        static_cast<int>(std::lround(selected_box_.y)),
        static_cast<int>(std::lround(selected_box_.w)),
        static_cast<int>(std::lround(selected_box_.h)));
    try { visual_valid = kcf_->update(frame, tracked); }
    catch (const cv::Exception &) { visual_valid = false; }
    if (!visual_valid) kcf_.release();
    if (visual_valid) {
      visual = clipBox({static_cast<double>(tracked.x), static_cast<double>(tracked.y),
                        static_cast<double>(tracked.width), static_cast<double>(tracked.height)}, frame.size());
      visual_feature = appearanceFeature(frame, visual, &visual_quality);
      const double ratio = visual.w * visual.h / std::max(1.0, selected_box_.w * selected_box_.h);
      const double distance = cv::norm(center(visual) - center(predicted)) /
          std::max(8.0, std::hypot(predicted.w, predicted.h));
      const bool compatible = ratio >= 1.0 / max_area_ratio_ && ratio <= max_area_ratio_ &&
          distance <= center_gate_sizes_ && validBox(visual, frame.size());
      const bool appearance_ok = appearance_anchor_.empty() ||
          cosineSimilarity(visual_feature, appearance_anchor_) >= appearance_gate_;
      if (!compatible || visual_quality < quality_gate_ || !appearance_ok) visual_valid = false;
    }
  }

  if (visual_observation && !long_recovery_ && step <= max_frame_gap_sec_ &&
      selected_local_id_ >= 0 && visual_observation->stable &&
      visual_observation->id == selected_local_id_ &&
      visual_observation->detector_stamp > 0.0 && visual_observation->detector_stamp <= stamp &&
      stamp - visual_observation->detector_stamp <= max_frame_gap_sec_ &&
      visual_observation->flow_quality >= 0.60 &&
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
      flow_valid = flow_quality >= quality_gate_ &&
          (appearance_anchor_.empty() || cosineSimilarity(flow_feature, appearance_anchor_) >= appearance_gate_);
    }
    if (flow_valid) {
      if (visual_valid && boxIou(visual, visual_observation->box) < 0.15) {
        visual_valid = false;
      } else {
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
    // A weak detection for the already selected stable ID can maintain the
    // track if it passes the same spatial and appearance gates below. Keep
    // the higher confidence threshold for acquiring or rebinding identities.
    const bool maintains_selected_id = candidate.id == selected_local_id_ &&
        selected_local_id_ >= 0;
    if (candidate.confidence < min_detector_confidence_ && !maintains_selected_id) continue;
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
      const double ratio = candidate.box.w * candidate.box.h /
          std::max(1.0, predicted.w * predicted.h);
      const double distance = cv::norm(center(candidate.box) - center(predicted)) /
          std::max(8.0, std::hypot(predicted.w, predicted.h));
      if (ratio < 1.0 / max_area_ratio_ || ratio > max_area_ratio_ ||
          distance > center_gate_sizes_) continue;
    }
    double quality = 0.0;
    const auto feature = appearanceFeature(frame, candidate.box, &quality);
    const double similarity = appearance_anchor_.empty() ? 0.0 :
        cosineSimilarity(feature, appearance_anchor_);
    const double required_quality = long_recovery_ ? std::max(quality_gate_, long_quality_gate_) : quality_gate_;
    const double required_similarity = long_recovery_ ? std::max(appearance_gate_, long_appearance_gate_) : appearance_gate_;
    if (quality < required_quality || similarity < required_similarity) continue;
    if (long_recovery_ && long_motion_required_ && !motion_continuous_) continue;
    const double distance = cv::norm(center(candidate.box) - center(predicted));
    const double scale = std::max(8.0, std::hypot(predicted.w, predicted.h));
    const double score = 0.45 * std::max(0.0, 1.0 - distance / scale) +
        0.25 * boxIou(candidate.box, predicted) + 0.30 * similarity;
    if (long_recovery_ && score < long_score_gate_) continue;
    ranked.emplace_back(score, &candidate);
  }
  std::sort(ranked.begin(), ranked.end(), [](const auto &a, const auto &b) {
    return a.first > b.first;
  });
  if (ranked.size() > 1 && ranked[0].first - ranked[1].first < ambiguity_margin_) {
    pending_reacquire_id_ = -1;
    pending_reacquire_count_ = 0;
    result_.has_box = false;
    result_.observed = false;
    result_.source = "ambiguous";
    result_.reason = "competing_detections";
    result_.confidence = 0.0;
    return;
  }
  const Detection *match = ranked.empty() ? nullptr : ranked.front().second;
  if (match && selected_local_id_ >= 0 && match->id != selected_local_id_ &&
      std::any_of(detections.begin(), detections.end(), [this](const Detection &item) {
        return item.id == selected_local_id_;
      })) {
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
      pending_reacquire_count_ > 0 || long_recovery_ || step > max_frame_gap_sec_ ||
      ((result_.source == "lost" || result_.source == "ambiguous" || result_.source == "prediction") &&
       !recent_same_identity));

  if (rebound) {
    const double measured = match->detector_stamp;
    const bool pending_jump = pending_reacquire_id_ == match->id &&
        pending_reacquire_count_ > 0 && cv::norm(center(match->box) - center(pending_reacquire_box_)) >
        2.0 + center_gate_sizes_ * std::hypot(match->box.w, match->box.h) *
            std::min(1.0, std::max(0.0, measured - pending_reacquire_stamp_) / max_frame_gap_sec_);
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
    if (pending_reacquire_count_ < confirmation_frames_) {
      const bool seed_hold = manual_seed_without_yolo_ && manual_seed_visual_without_yolo_;
      const bool agrees = visual_valid && stamp - std::max(last_detector_for_selection_, selected_stamp_) <= visual_hold_sec_ &&
          boxIou(visual, match->box) >= 0.30;
      if (visual_valid && (seed_hold || agrees)) {
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
    if (agrees) kcf_reseed_count_ = 0;
    else if (detector_updated) ++kcf_reseed_count_;
    // Do not let a single detector update move the published target box
    // directly. The constant-velocity prediction absorbs small detector jitter;
    // a confirmed identity rebound starts at the new measurement immediately.
    const double authority_span = std::max(0.01, detector_authority_ - min_detector_confidence_);
    const double confidence_above_maintenance = clamp01(
        (match->confidence - min_detector_confidence_) / authority_span);
    const double detector_gain = rebound ? 1.0 :
        std::max(0.28, std::min(0.62,
            0.28 + 0.28 * confidence_above_maintenance +
            (match->confidence >= detector_authority_ ? 0.06 : 0.0)));
    const Box fused = smooth_box(predicted, measurement, detector_gain);
    const bool refresh = agrees && stamp - last_local_seed_stamp_ >= local_template_refresh_sec_ &&
        match->confidence >= min_detector_confidence_ && match->detector_stamp == stamp;
    const bool reseed_kcf = rebound || !kcf_ || refresh || kcf_reseed_count_ >= 2;
    if (reseed_kcf) {
      try {
        kcf_ = cv::TrackerKCF::create();
        kcf_->init(frame, cv::Rect(static_cast<int>(std::lround(fused.x)),
            static_cast<int>(std::lround(fused.y)), static_cast<int>(std::lround(fused.w)),
            static_cast<int>(std::lround(fused.h))));
        last_local_seed_stamp_ = stamp;
        kcf_reseed_count_ = 0;
      } catch (const cv::Exception &) { kcf_.release(); }
    }
    if (rebound) {
      velocity_ = cv::Point2d(0.0, 0.0);
      motion_continuous_ = true;
    } else {
      const double observation_dt = stamp - last_observation_stamp_;
      if (observation_dt > 0.0) {
        const cv::Point2d measured_velocity = (center(fused) - center(selected_box_)) / observation_dt;
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
  } else if (visual_valid && ((manual_seed_without_yolo_ && manual_seed_visual_without_yolo_) ||
      stamp - std::max(last_detector_for_selection_, selected_stamp_) <= visual_hold_sec_)) {
    const Box filtered_visual = smooth_box(predicted, visual, 0.65);
    const double dt = stamp - last_observation_stamp_;
    if (dt > 0.0) {
      const cv::Point2d measured = (center(filtered_visual) - center(selected_box_)) / dt;
      velocity_ = 0.8 * velocity_ + 0.2 * measured;
    }
    selected_box_ = filtered_visual;
    last_observation_stamp_ = stamp;
    if (visual_quality >= 0.65 && visual_feature.size() == appearance_anchor_.size()) {
      for (size_t i = 0; i < visual_feature.size(); ++i)
        appearance_anchor_[i] = 0.95f * appearance_anchor_[i] + 0.05f * visual_feature[i];
    }
    result_.box = filtered_visual;
    result_.has_box = true;
    result_.observed = true;
    result_.source = "visual";
    result_.reason = "validated_local_tracker_observation";
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
    const Detection &candidate, const std::string &reason) {
  if (!snapshot.image || mode_ != "color_tracker" || !hasConfiguredColor(color_name_) ||
      !validBox(candidate.box, snapshot.frame.size())) return false;

  const bool was_auto_selecting = auto_select_enabled_;
  auto_select_enabled_ = false;
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
  if (was_auto_selecting) {
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
  if (!auto_select_enabled_ || detections.empty() || !image) {
    auto_select_count_ = 0;
    auto_select_candidate_id_ = -1;
    return;
  }
  const Detection *winner = nullptr;
  for (const auto &detection : detections) {
    if (detection.confidence < auto_select_min_confidence_ || !validBox(detection.box, frame.size())) continue;
    if (winner == nullptr || detection.confidence > winner->confidence ||
        (detection.confidence == winner->confidence && detection.box.w * detection.box.h > winner->box.w * winner->box.h)) {
      winner = &detection;
    }
  }
  if (winner == nullptr) {
    auto_select_count_ = 0;
    auto_select_candidate_id_ = -1;
    return;
  }
  const double stamp = image->header.stamp.toSec();
  const bool same_identity = (auto_select_candidate_stable_ || winner->stable)
      ? (auto_select_candidate_stable_ && winner->stable && auto_select_candidate_id_ == winner->id)
      : (boxIou(auto_select_box_, winner->box) >= 0.30);
  const bool same = auto_select_candidate_class_ == winner->class_id && same_identity &&
      boxIou(auto_select_box_, winner->box) >= 0.30 &&
      stamp - auto_select_last_stamp_ > 0.0 && stamp - auto_select_last_stamp_ <= auto_select_max_gap_;
  auto_select_count_ = same ? auto_select_count_ + 1 : 1;
  auto_select_candidate_id_ = static_cast<int>(winner->id);
  auto_select_candidate_class_ = winner->class_id;
  auto_select_candidate_stable_ = winner->stable;
  auto_select_box_ = winner->box;
  auto_select_last_stamp_ = stamp;
  if (auto_select_count_ < auto_select_stable_frames_) return;
  Snapshot snapshot{image, frame, detections};
  bool selected = false;
  if (mode_ == "color_tracker" && hasConfiguredColor(color_name_)) {
    selected = selectConfiguredColorCandidate(snapshot, *winner, "color_auto_selected");
  } else {
    int64_t id = -1;
    std::string error;
    selected = selectFromSnapshot(snapshot, winner->box, true, false, &id, &error);
  }
  if (selected) {
    auto_select_enabled_ = false;
    auto_select_pub_status_ = false;
    std_msgs::Bool status;
    status.data = false;
    auto_status_pub_.publish(status);
    ROS_INFO_STREAM("C++ SmartTracker auto-selected detection id=" << winner->id << " stable_frames=" << auto_select_count_);
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
  for (const auto &detection : detections) {
    const bool selected = selected_ && detection.id == selected_local_id_ && detection.stable;
    const cv::Scalar color = selected ? cv::Scalar(0, 255, 100) : cv::Scalar(255, 190, 0);
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
  if (result_.has_box && result_.public_id >= 0 && result_.source != "idle") {
    const cv::Point first(static_cast<int>(std::lround(result_.box.x)), static_cast<int>(std::lround(result_.box.y)));
    const cv::Point second(static_cast<int>(std::lround(result_.box.x + result_.box.w)),
                           static_cast<int>(std::lround(result_.box.y + result_.box.h)));
    cv::rectangle(frame, first, second, cv::Scalar(0, 255, 100), 2, cv::LINE_AA);
    std::ostringstream label;
    label << className(result_.class_id) << " ID " << result_.public_id;
    cv::putText(frame, label.str(), cv::Point(first.x, std::max(18, first.y - 5)),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 100), 1, cv::LINE_AA);
  } else if (selected_ && (result_.source == "lost" || result_.source == "ambiguous" || result_.source == "prediction")) {
    cv::putText(frame, lower(result_.source), cv::Point(12, 25), cv::FONT_HERSHEY_SIMPLEX,
                0.6, cv::Scalar(0, 180, 255), 1, cv::LINE_AA);
  }
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
  state.tracker_type = mode == "fusion_tracker" ? "YOLO+KCF" : mode;
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
  manual_active_ = false;
  manual_seed_ = false;
  kcf_.release();
  kcf_reseed_count_ = 0;
  selected_box_ = Box();
  selected_id_ = -1;
  selected_local_id_ = -1;
  selected_class_id_ = -1;
  selected_confidence_ = 0.0;
  selected_stamp_ = 0.0;
  last_observation_stamp_ = 0.0;
  last_fusion_stamp_ = 0.0;
  last_detector_for_selection_ = 0.0;
  last_local_seed_stamp_ = 0.0;
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
  last_local_seed_stamp_ = selected_stamp_;
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
  if (requested == "fusion_tracker" &&
      !inference_client_.waitForExistence(ros::Duration(0.25))) {
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
  flow_detections_.reset();
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
  response.class_count = static_cast<uint32_t>(labels_.size());
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
      ? "KCF initialized from the requested manual ROI; YOLO will validate and correct nearby matches"
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
                ? "Whole-image search for the configured color started"
                : "Whole-image search from the ROI-learned color histogram started")
            : "Auto target selection started; waiting for stable detections")
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

