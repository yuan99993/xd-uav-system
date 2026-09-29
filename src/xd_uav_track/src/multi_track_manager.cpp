#include <xd_uav_track/multi_track_manager.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <deque>
#include <limits>
#include <map>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <ros/ros.h>

namespace xd_uav_track {
namespace {

double clampValue(const double value, const double low, const double high) {
  return std::max(low, std::min(high, value));
}

bool finiteFloat(const float value) {
  return std::isfinite(static_cast<double>(value));
}

struct NormalizedBox {
  double cx{0.0};
  double cy{0.0};
  double width{0.0};
  double height{0.0};
};

bool normalizedBox(const xd_uav_track::DetectionCandidate& candidate,
                   const int image_width, const int image_height,
                   NormalizedBox* result) {
  if (result == nullptr || image_width <= 0 || image_height <= 0 ||
      !finiteFloat(candidate.confidence)) {
    return false;
  }
  NormalizedBox box;
  if (candidate.has_normalized_bbox) {
    for (const float value : candidate.normalized_bbox) {
      if (!finiteFloat(value)) return false;
    }
    box.cx = candidate.normalized_bbox[0];
    box.cy = candidate.normalized_bbox[1];
    box.width = candidate.normalized_bbox[2];
    box.height = candidate.normalized_bbox[3];
  } else if (candidate.has_bbox) {
    const int x1 = candidate.bbox[0];
    const int y1 = candidate.bbox[1];
    const int x2 = candidate.bbox[2];
    const int y2 = candidate.bbox[3];
    if (x2 <= x1 || y2 <= y1) return false;
    box.cx = 0.5 * static_cast<double>(x1 + x2) / image_width;
    box.cy = 0.5 * static_cast<double>(y1 + y2) / image_height;
    box.width = static_cast<double>(x2 - x1) / image_width;
    box.height = static_cast<double>(y2 - y1) / image_height;
  } else {
    return false;
  }
  if (!std::isfinite(box.cx) || !std::isfinite(box.cy) ||
      !std::isfinite(box.width) || !std::isfinite(box.height) ||
      box.width <= 0.0 || box.height <= 0.0 || box.width > 1.5 ||
      box.height > 1.5 || box.cx < -0.25 || box.cx > 1.25 ||
      box.cy < -0.25 || box.cy > 1.25) {
    return false;
  }
  box.width = clampValue(box.width, 1.0 / image_width, 1.0);
  box.height = clampValue(box.height, 1.0 / image_height, 1.0);
  box.cx = clampValue(box.cx, 0.5 * box.width, 1.0 - 0.5 * box.width);
  box.cy = clampValue(box.cy, 0.5 * box.height, 1.0 - 0.5 * box.height);
  *result = box;
  return true;
}

double intersectionOverUnion(const NormalizedBox& lhs,
                             const NormalizedBox& rhs) {
  const double lhs_x1 = lhs.cx - 0.5 * lhs.width;
  const double lhs_y1 = lhs.cy - 0.5 * lhs.height;
  const double lhs_x2 = lhs.cx + 0.5 * lhs.width;
  const double lhs_y2 = lhs.cy + 0.5 * lhs.height;
  const double rhs_x1 = rhs.cx - 0.5 * rhs.width;
  const double rhs_y1 = rhs.cy - 0.5 * rhs.height;
  const double rhs_x2 = rhs.cx + 0.5 * rhs.width;
  const double rhs_y2 = rhs.cy + 0.5 * rhs.height;
  const double intersection =
      std::max(0.0, std::min(lhs_x2, rhs_x2) - std::max(lhs_x1, rhs_x1)) *
      std::max(0.0, std::min(lhs_y2, rhs_y2) - std::max(lhs_y1, rhs_y1));
  const double area = lhs.width * lhs.height + rhs.width * rhs.height - intersection;
  return area > 1e-12 ? intersection / area : 0.0;
}

double normalizedCenterDistance(const NormalizedBox& lhs,
                                const NormalizedBox& rhs) {
  const double scale = std::max(1e-6, std::hypot(lhs.width, lhs.height));
  return std::hypot(lhs.cx - rhs.cx, lhs.cy - rhs.cy) / scale;
}

double cosineSimilarity(const std::vector<float>& lhs,
                        const std::vector<float>& rhs) {
  if (lhs.empty() || lhs.size() != rhs.size()) return -1.0;
  double dot = 0.0;
  double lhs_norm = 0.0;
  double rhs_norm = 0.0;
  for (std::size_t index = 0; index < lhs.size(); ++index) {
    if (!finiteFloat(lhs[index]) || !finiteFloat(rhs[index])) return -1.0;
    dot += static_cast<double>(lhs[index]) * rhs[index];
    lhs_norm += static_cast<double>(lhs[index]) * lhs[index];
    rhs_norm += static_cast<double>(rhs[index]) * rhs[index];
  }
  if (lhs_norm <= 1e-12 || rhs_norm <= 1e-12) return -1.0;
  return dot / std::sqrt(lhs_norm * rhs_norm);
}

// Rectangular maximum-weight assignment. Invalid edges have a very negative
// score; dummy columns let a detection remain unmatched without stealing a
// viable track from another detection.
std::vector<int> globalAssignment(const std::vector<std::vector<double>>& scores) {
  if (scores.empty()) return {};
  const int rows = static_cast<int>(scores.size());
  const int real_columns = static_cast<int>(scores.front().size());
  const int columns = real_columns + rows;
  std::vector<double> u(rows + 1), v(columns + 1);
  std::vector<int> p(columns + 1), way(columns + 1);
  for (int row = 1; row <= rows; ++row) {
    p[0] = row;
    int column = 0;
    std::vector<double> minv(columns + 1, std::numeric_limits<double>::infinity());
    std::vector<bool> used(columns + 1, false);
    do {
      used[column] = true;
      const int current = p[column];
      double delta = std::numeric_limits<double>::infinity();
      int next = 0;
      for (int candidate = 1; candidate <= columns; ++candidate) {
        if (used[candidate]) continue;
        const double score = candidate <= real_columns
            ? scores[current - 1][candidate - 1] : 0.0;
        const double cost = -score - u[current] - v[candidate];
        if (cost < minv[candidate]) {
          minv[candidate] = cost;
          way[candidate] = column;
        }
        if (minv[candidate] < delta) {
          delta = minv[candidate];
          next = candidate;
        }
      }
      for (int candidate = 0; candidate <= columns; ++candidate) {
        if (used[candidate]) {
          u[p[candidate]] += delta;
          v[candidate] -= delta;
        } else {
          minv[candidate] -= delta;
        }
      }
      column = next;
    } while (p[column] != 0);
    do {
      const int previous = way[column];
      p[column] = p[previous];
      column = previous;
    } while (column != 0);
  }
  std::vector<int> result(rows, -1);
  for (int column = 1; column <= real_columns; ++column) {
    if (p[column] > 0 && scores[p[column] - 1][column - 1] > 0.0)
      result[p[column] - 1] = column - 1;
  }
  return result;
}

class BoxKalmanFilter {
 public:
  void initialize(const NormalizedBox& box, const MultiTrackConfig& config) {
    state_.setZero();
    state_ << box.cx, box.cy, std::log(box.width), std::log(box.height),
        0.0, 0.0, 0.0, 0.0;
    covariance_.setIdentity();
    covariance_.diagonal() << 0.01, 0.01, 0.02, 0.02,
        0.25, 0.25, 0.10, 0.10;
    process_noise_ = std::max(1e-6, config.process_noise);
    measurement_noise_ = std::max(1e-6, config.measurement_noise);
  }

  void predict(const double requested_dt) {
    double remaining = clampValue(requested_dt, 1e-3, 5.0);
    while (remaining > 1e-9) {
      const double dt = std::min(0.5, remaining);
      predictStep(dt);
      remaining -= dt;
    }
  }

  void predictStep(const double dt) {
    Eigen::Matrix<double, 8, 8> transition =
        Eigen::Matrix<double, 8, 8>::Identity();
    for (int index = 0; index < 4; ++index) transition(index, index + 4) = dt;
    Eigen::Matrix<double, 8, 8> process =
        Eigen::Matrix<double, 8, 8>::Zero();
    const double dt2 = dt * dt;
    const double dt3 = dt2 * dt;
    const double dt4 = dt2 * dt2;
    for (int index = 0; index < 4; ++index) {
      const int velocity = index + 4;
      const double scale = (index < 2 ? 0.01 : 0.002) * process_noise_;
      process(index, index) = 0.25 * dt4 * scale;
      process(index, velocity) = process(velocity, index) = 0.5 * dt3 * scale;
      process(velocity, velocity) = dt2 * scale;
    }
    state_ = transition * state_;
    covariance_ = transition * covariance_ * transition.transpose() + process;
  }

  void update(const NormalizedBox& box, const double confidence) {
    Eigen::Matrix<double, 4, 8> observation;
    observation.setZero();
    observation(0, 0) = observation(1, 1) = 1.0;
    observation(2, 2) = observation(3, 3) = 1.0;
    Eigen::Matrix<double, 4, 1> measurement;
    measurement << box.cx, box.cy, std::log(box.width), std::log(box.height);
    Eigen::Matrix4d noise = Eigen::Matrix4d::Identity();
    const double confidence_scale = 1.0 / std::max(0.05, confidence);
    noise.diagonal() << 0.0025, 0.0025, 0.01, 0.01;
    noise *= measurement_noise_ * confidence_scale;
    const Eigen::Matrix<double, 4, 1> residual = measurement - observation * state_;
    const Eigen::Matrix4d innovation =
        observation * covariance_ * observation.transpose() + noise;
    const Eigen::Matrix<double, 8, 4> gain = covariance_ * observation.transpose() *
        innovation.ldlt().solve(Eigen::Matrix4d::Identity());
    state_ += gain * residual;
    const Eigen::Matrix<double, 8, 8> identity =
        Eigen::Matrix<double, 8, 8>::Identity();
    const auto correction = identity - gain * observation;
    covariance_ = correction * covariance_ * correction.transpose() +
                  gain * noise * gain.transpose();
  }

  NormalizedBox box() const {
    NormalizedBox result;
    result.cx = clampValue(state_(0), 0.0, 1.0);
    result.cy = clampValue(state_(1), 0.0, 1.0);
    result.width = std::exp(clampValue(state_(2), std::log(1e-4), std::log(1.0)));
    result.height = std::exp(clampValue(state_(3), std::log(1e-4), std::log(1.0)));
    return result;
  }

  int outsideEdge() const {
    const double width = std::exp(clampValue(
        state_(2), std::log(1e-4), std::log(1.0)));
    const double height = std::exp(clampValue(
        state_(3), std::log(1e-4), std::log(1.0)));
    const double half_width = 0.5 * width;
    const double half_height = 0.5 * height;
    if (state_(0) + half_width < 0.0) return 1;  // left
    if (state_(0) - half_width > 1.0) return 2;  // right
    if (state_(1) + half_height < 0.0) return 3; // top
    if (state_(1) - half_height > 1.0) return 4; // bottom
    return 0;
  }

  std::array<float, 2> velocity(const int width, const int height) const {
    return {static_cast<float>(state_(4) * width),
            static_cast<float>(state_(5) * height)};
  }

  std::array<float, 4> covariance() const {
    return {static_cast<float>(std::max(0.0, covariance_(0, 0))),
            static_cast<float>(std::max(0.0, covariance_(1, 1))),
            static_cast<float>(std::max(0.0, covariance_(2, 2))),
            static_cast<float>(std::max(0.0, covariance_(3, 3)))};
  }

 private:
  Eigen::Matrix<double, 8, 1> state_{Eigen::Matrix<double, 8, 1>::Zero()};
  Eigen::Matrix<double, 8, 8> covariance_{Eigen::Matrix<double, 8, 8>::Identity()};
  double process_noise_{1.0};
  double measurement_noise_{1.0};
};

struct PreparedDetection {
  xd_uav_track::DetectionCandidate message;
  NormalizedBox box;
  std::size_t original_index{0};
};

bool metricRelativeXY(const xd_uav_track::DetectionCandidate& candidate,
                      std::array<double, 2>* position) {
  if (position == nullptr || !candidate.range_valid ||
      !candidate.has_relative_position_body) return false;
  const double x = candidate.relative_position_body[0];
  const double y = candidate.relative_position_body[1];
  if (!std::isfinite(x) || !std::isfinite(y)) return false;
  *position = {{x, y}};
  return true;
}

bool compatibleClass(const int first, const int second,
                     const std::vector<int>& family) {
  if (first == second) return true;
  return first >= 0 && second >= 0 &&
      std::find(family.begin(), family.end(), first) != family.end() &&
      std::find(family.begin(), family.end(), second) != family.end();
}

bool sameReacquisitionCandidate(const NormalizedBox& previous_box,
                                const xd_uav_track::DetectionCandidate& previous,
                                const PreparedDetection& current,
                                const double minimum_iou,
                                const std::vector<int>& compatible_classes) {
  if (previous.class_id >= 0 && current.message.class_id >= 0 &&
      !compatibleClass(previous.class_id, current.message.class_id,
                       compatible_classes)) return false;
  if (previous.track_id_is_stable && current.message.track_id_is_stable &&
      previous.track_id >= 0 && current.message.track_id >= 0)
    return previous.track_id == current.message.track_id;
  return intersectionOverUnion(previous_box, current.box) >= minimum_iou ||
      normalizedCenterDistance(previous_box, current.box) <= 0.75;
}

struct PersistentTrack {
  int public_id{-1};
  int detector_track_id{-1};
  bool detector_id_stable{false};
  int class_id{-1};
  bool class_locked{false};
  std::map<int, double> class_evidence;
  double confidence{0.0};
  BoxKalmanFilter filter;
  xd_uav_track::DetectionCandidate latest_detection;
  std::array<double, 2> metric_relative_xy{{0.0, 0.0}};
  bool has_metric_relative_xy{false};
  std_msgs::Header latest_header;
  std::vector<float> appearance;
  std::deque<std::vector<float>> appearance_gallery;
  std::deque<int> class_history;
  std::uint32_t age{1};
  std::uint32_t hits{1};
  std::uint32_t missing{0};
  double missing_seconds{0.0};
  bool detected{true};
  bool reidentified{false};
  bool reacquisition_pending{false};
  std::uint32_t reacquisition_hits{0};
  bool have_reacquisition_candidate{false};
  NormalizedBox reacquisition_box;
  xd_uav_track::DetectionCandidate reacquisition_candidate;
  int exit_edge{0};
  std::string association{"new"};
  ros::Time last_stamp;
};

}  // namespace

struct MultiTrackManager::Impl {
  explicit Impl(MultiTrackConfig requested) : config(std::move(requested)) {
    time_based_lifecycle = config.occlusion_timeout_sec > 0.0 ||
        config.removal_timeout_sec > 0.0;
    config.confirmation_hits = std::max(1, config.confirmation_hits);
    config.occlusion_frames = std::max(0, config.occlusion_frames);
    config.removal_frames = std::max(config.occlusion_frames + 1,
                                     config.removal_frames);
    config.reference_fps = clampValue(config.reference_fps, 1.0, 120.0);
    config.occlusion_timeout_sec = config.occlusion_timeout_sec > 0.0
        ? config.occlusion_timeout_sec
        : static_cast<double>(config.occlusion_frames) / config.reference_fps;
    config.removal_timeout_sec = config.removal_timeout_sec > 0.0
        ? config.removal_timeout_sec
        : static_cast<double>(config.removal_frames + 1) / config.reference_fps;
    config.removal_timeout_sec = std::max(
        config.occlusion_timeout_sec + 1.0 / config.reference_fps,
        config.removal_timeout_sec);
    config.reacquisition_after_sec = config.reacquisition_after_sec > 0.0
        ? config.reacquisition_after_sec : config.occlusion_timeout_sec;
    config.reacquisition_confirm_hits =
        std::max(1, config.reacquisition_confirm_hits);
    if (config.reacquisition_mode != "aggressive" &&
        config.reacquisition_mode != "strict")
      config.reacquisition_mode = "balanced";
    config.maximum_size_change_ratio = std::max(
        1.0, config.maximum_size_change_ratio);
    config.exit_edge_margin = clampValue(config.exit_edge_margin, 0.0, 0.45);
    config.search_expansion_rate_per_sec = std::max(
        0.0, config.search_expansion_rate_per_sec);
    config.search_expansion_cap = std::max(1.0, config.search_expansion_cap);
    config.lenient_iou_scale = clampValue(config.lenient_iou_scale, 0.0, 1.0);
    config.lenient_iou_floor = clampValue(config.lenient_iou_floor, 0.0, 1.0);
    config.appearance_distance_gate_factor = std::max(
        0.0, config.appearance_distance_gate_factor);
    config.maximum_tracks = std::max(1, config.maximum_tracks);
    config.maximum_embedding_dimension =
        std::max(0, config.maximum_embedding_dimension);
    config.minimum_new_track_confidence =
        clampValue(config.minimum_new_track_confidence, 0.0, 1.0);
    config.minimum_update_confidence =
        clampValue(config.minimum_update_confidence, 0.0, 1.0);
    config.association_iou_threshold =
        clampValue(config.association_iou_threshold, 0.0, 1.0);
    config.association_center_distance =
        std::max(0.0, config.association_center_distance);
    config.association_ambiguity_margin =
        std::max(0.0, config.association_ambiguity_margin);
    config.metric_relative_max_distance_m =
        clampValue(config.metric_relative_max_distance_m, 1.0, 100.0);
    config.metric_relative_score_weight =
        clampValue(config.metric_relative_score_weight, 0.0, 5.0);
    config.metric_relative_smoothing_alpha =
        clampValue(config.metric_relative_smoothing_alpha, 0.01, 1.0);
    config.metric_relative_max_speed_mps =
        clampValue(config.metric_relative_max_speed_mps, 0.1, 100.0);
    config.metric_relative_jump_tolerance_m =
        clampValue(config.metric_relative_jump_tolerance_m, 0.0, 100.0);
    config.class_history_size = std::max(1, std::min(64,
        config.class_history_size));
    config.confidence_smoothing_alpha = clampValue(
        config.confidence_smoothing_alpha, 0.0, 0.99);
    config.confidence_decay_rate = clampValue(
        config.confidence_decay_rate, 0.0, 1.0);
    config.appearance_minimum_cosine =
        clampValue(config.appearance_minimum_cosine, -1.0, 1.0);
    config.high_confidence_threshold = clampValue(
        config.high_confidence_threshold, config.minimum_update_confidence, 1.0);
    config.appearance_gallery_size = std::max(1, std::min(16, config.appearance_gallery_size));
  }

  int allocateId(const PreparedDetection& detection) {
    if (detection.message.track_id_is_stable && detection.message.track_id >= 0 &&
        tracks.count(detection.message.track_id) == 0) {
      return detection.message.track_id;
    }
    while (tracks.count(next_generated_id) != 0 || next_generated_id < 0) {
      if (next_generated_id == std::numeric_limits<int>::max()) {
        next_generated_id = 1000000000;
      } else {
        ++next_generated_id;
      }
    }
    return next_generated_id++;
  }

  double dtFor(const PersistentTrack& track, const ros::Time& stamp,
               const double frame_dt) const {
    if (stamp.isZero() || track.last_stamp.isZero() || stamp <= track.last_stamp) {
      return frame_dt;
    }
    return clampValue((stamp - track.last_stamp).toSec(), 1e-3, 5.0);
  }

  MultiTrackConfig config;
  bool time_based_lifecycle{false};
  std::map<int, PersistentTrack> tracks;
  int next_generated_id{1000000000};
  int selected_track_id{-1};
  ros::Time last_frame_stamp;
  MultiTrackStatistics stats;
};

MultiTrackManager::MultiTrackManager(const MultiTrackConfig& config)
    : impl_(new Impl(config)) {}

MultiTrackManager::~MultiTrackManager() = default;
MultiTrackManager::MultiTrackManager(MultiTrackManager&&) noexcept = default;
MultiTrackManager& MultiTrackManager::operator=(MultiTrackManager&&) noexcept = default;

ManagedDetectionFrame MultiTrackManager::update(
    const xd_uav_track::DetectionArray& detections, const int image_width,
    const int image_height) {
  const std::string source = detections.image_source.empty()
      ? detections.header.frame_id : detections.image_source;
  return update(detections, image_width, image_height, source);
}

ManagedDetectionFrame MultiTrackManager::update(
    const xd_uav_track::DetectionArray& detections, const int image_width,
    const int image_height, const std::string& image_source) {
  ManagedDetectionFrame output;
  output.candidates.header = detections.header;
  output.candidates.command = detections.command;
  output.tracks.header = detections.header;
  output.tracks.image_source = image_source;
  ++impl_->stats.input_frames;

  const ros::Time frame_stamp = detections.header.stamp.isZero()
      ? ros::Time::now() : detections.header.stamp;
  double frame_dt = 1.0 / impl_->config.reference_fps;
  if (!frame_stamp.isZero() && !impl_->last_frame_stamp.isZero()) {
    if (frame_stamp <= impl_->last_frame_stamp) {
      output.accepted = false;
      return output;
    }
    frame_dt = (frame_stamp - impl_->last_frame_stamp).toSec();
  }
  if (!frame_stamp.isZero()) impl_->last_frame_stamp = frame_stamp;

  std::vector<PreparedDetection> prepared;
  prepared.reserve(detections.candidates.size());
  for (std::size_t index = 0; index < detections.candidates.size(); ++index) {
    const auto& raw = detections.candidates[index];
    NormalizedBox box;
    const bool embedding_valid = raw.appearance_embedding.size() <=
        static_cast<std::size_t>(impl_->config.maximum_embedding_dimension);
    if (!embedding_valid || !normalizedBox(raw, image_width, image_height, &box) ||
        raw.confidence < impl_->config.minimum_update_confidence) {
      ++impl_->stats.rejected_detections;
      continue;
    }
    PreparedDetection item;
    item.message = raw;
    item.box = box;
    item.original_index = index;
    prepared.push_back(std::move(item));
  }

  std::set<int> tracks_missing_before_frame;
  for (auto iterator = impl_->tracks.begin(); iterator != impl_->tracks.end();) {
    auto& track = iterator->second;
    if (track.missing > 0) tracks_missing_before_frame.insert(iterator->first);
    track.filter.predict(impl_->dtFor(track, frame_stamp, frame_dt));
    ++track.age;
    ++track.missing;
    track.missing_seconds += std::max(0.0, frame_dt);
    track.detected = false;
    track.reidentified = false;
    track.association = "predicted";
    const int predicted_exit_edge = track.filter.outsideEdge();
    if (predicted_exit_edge != 0) track.exit_edge = predicted_exit_edge;
    const bool expired = impl_->time_based_lifecycle
        ? track.missing_seconds > impl_->config.removal_timeout_sec
        : track.missing > static_cast<std::uint32_t>(
              impl_->config.removal_frames);
    if (expired) {
      if (impl_->selected_track_id == iterator->first)
        impl_->selected_track_id = -1;
      iterator = impl_->tracks.erase(iterator);
      ++impl_->stats.removed_tracks;
      continue;
    }
    ++iterator;
  }

  std::set<int> used_tracks;
  std::set<int> ambiguous_tracks;
  std::vector<bool> ambiguous_detections(prepared.size(), false);
  std::vector<int> assigned_ids(prepared.size(), -1);
  std::vector<std::string> assigned_methods(prepared.size());

  // ByteTrack-style stages: confident observations get first use of the
  // global assignment; weak observations may update, but never create, IDs.
  for (int stage = 0; stage < 2; ++stage) {
    std::vector<std::size_t> detection_indices;
    std::vector<int> track_ids;
    for (std::size_t index = 0; index < prepared.size(); ++index) {
      if (assigned_ids[index] < 0 &&
          (prepared[index].message.confidence >= impl_->config.high_confidence_threshold) ==
              (stage == 0)) detection_indices.push_back(index);
    }
    for (const auto& entry : impl_->tracks)
      if (used_tracks.count(entry.first) == 0) track_ids.push_back(entry.first);
    if (detection_indices.empty() || track_ids.empty()) continue;
    std::vector<std::vector<double>> scores(
        detection_indices.size(), std::vector<double>(track_ids.size(), -1e6));
    std::vector<std::vector<std::string>> methods(
        detection_indices.size(), std::vector<std::string>(track_ids.size()));
    for (std::size_t row = 0; row < detection_indices.size(); ++row) {
      const auto& detection = prepared[detection_indices[row]];
      for (std::size_t column = 0; column < track_ids.size(); ++column) {
        const auto& track = impl_->tracks.at(track_ids[column]);
        if (track.class_id >= 0 && detection.message.class_id >= 0 &&
            track.class_id != detection.message.class_id) {
          const bool recent_class_match = impl_->config.flexible_class_matching &&
              std::find(track.class_history.begin(), track.class_history.end(),
                        detection.message.class_id) != track.class_history.end();
          const bool configured_family_match =
              impl_->config.flexible_class_matching &&
              compatibleClass(track.class_id, detection.message.class_id,
                              impl_->config.compatible_class_ids);
          if (!recent_class_match && !configured_family_match) continue;
        }
        const NormalizedBox predicted = track.filter.box();
        const double overlap = intersectionOverUnion(predicted, detection.box);
        const double distance = normalizedCenterDistance(predicted, detection.box);
        double appearance = cosineSimilarity(
            track.appearance, detection.message.appearance_embedding);
        for (const auto& prototype : track.appearance_gallery)
          appearance = std::max(appearance, cosineSimilarity(
              prototype, detection.message.appearance_embedding));
        const bool appearance_match =
            appearance >= impl_->config.appearance_minimum_cosine;
        const bool same_detector_id = detection.message.track_id_is_stable &&
            detection.message.track_id >= 0 && track.detector_id_stable &&
            track.detector_track_id == detection.message.track_id;
        double metric_affinity = 0.0;
        std::array<double, 2> observed_metric;
        if (impl_->config.metric_relative_association_enabled &&
            track.has_metric_relative_xy &&
            metricRelativeXY(detection.message, &observed_metric)) {
          const double metric_distance = std::hypot(
              observed_metric[0] - track.metric_relative_xy[0],
              observed_metric[1] - track.metric_relative_xy[1]);
          if (metric_distance > impl_->config.metric_relative_max_distance_m)
            continue;
          metric_affinity = impl_->config.metric_relative_score_weight *
              (1.0 - metric_distance /
               impl_->config.metric_relative_max_distance_m);
        }
        const bool recovering_after_loss =
            tracks_missing_before_frame.count(track_ids[column]) != 0 &&
            track.missing_seconds >= impl_->config.reacquisition_after_sec;
        if (recovering_after_loss && predicted.width * predicted.height > 1e-8) {
          const double area_ratio = detection.box.width * detection.box.height /
              (predicted.width * predicted.height);
          if (area_ratio < 1.0 / impl_->config.maximum_size_change_ratio ||
              area_ratio > impl_->config.maximum_size_change_ratio) continue;
          if (track.exit_edge != 0) {
            const double margin = impl_->config.exit_edge_margin;
            const bool returned_near_exit = track.exit_edge == 1
                ? detection.box.cx <= margin
                : track.exit_edge == 2 ? detection.box.cx >= 1.0 - margin
                : track.exit_edge == 3 ? detection.box.cy <= margin
                                       : detection.box.cy >= 1.0 - margin;
            if (!returned_near_exit) continue;
          }
          if (impl_->config.reacquisition_mode == "strict" &&
              !appearance_match) continue;
          if (impl_->config.reacquisition_mode == "balanced" &&
              !same_detector_id && !appearance_match) continue;
        }
        const double search_multiplier = tracks_missing_before_frame.count(
            track_ids[column]) == 0 ? 1.0 : std::min(
                impl_->config.search_expansion_cap,
                1.0 + track.missing_seconds *
                    impl_->config.search_expansion_rate_per_sec);
        const double center_gate = impl_->config.association_center_distance *
            search_multiplier;
        const double iou_gate = tracks_missing_before_frame.count(
            track_ids[column]) == 0 ? impl_->config.association_iou_threshold
            : std::max(impl_->config.lenient_iou_floor,
                impl_->config.association_iou_threshold *
                    impl_->config.lenient_iou_scale);
        const double appearance_gate = impl_->config.appearance_distance_gate_factor *
            search_multiplier;
        if (overlap < iou_gate && distance > center_gate &&
            !(appearance_match && distance <= appearance_gate)) continue;
        scores[row][column] = 1.0 + 2.0 * overlap - 0.15 * distance +
            (appearance_match ? 0.35 * appearance : 0.0) + metric_affinity;
        if (same_detector_id) scores[row][column] += 1.0;
        methods[row][column] = same_detector_id ? "detector_id" :
            (appearance_match && overlap < 0.5 ? "appearance" : "spatial");
      }
    }

    // Preserve PixEagle's fail-closed identity behavior: a global optimizer
    // must not turn a near-tie into a confident ID switch. Ambiguous rows and
    // candidates competing equally for one track are withheld from both
    // association and new-track creation for this frame.
    std::vector<bool> ambiguous_rows(detection_indices.size(), false);
    const double ambiguity_margin = impl_->config.association_ambiguity_margin;
    if (ambiguity_margin > 0.0) {
      for (std::size_t row = 0; row < scores.size(); ++row) {
        std::vector<std::pair<double, int>> ranked;
        for (std::size_t column = 0; column < track_ids.size(); ++column)
          if (scores[row][column] > 0.0)
            ranked.emplace_back(scores[row][column], static_cast<int>(column));
        std::sort(ranked.begin(), ranked.end(),
                  [](const auto& lhs, const auto& rhs) {
                    return lhs.first > rhs.first;
                  });
        if (ranked.size() >= 2 &&
            ranked[0].first - ranked[1].first < ambiguity_margin) {
          ambiguous_rows[row] = true;
          ambiguous_tracks.insert(track_ids[ranked[0].second]);
          ambiguous_tracks.insert(track_ids[ranked[1].second]);
        }
      }
      for (std::size_t column = 0; column < track_ids.size(); ++column) {
        std::vector<std::pair<double, int>> ranked;
        for (std::size_t row = 0; row < scores.size(); ++row)
          if (scores[row][column] > 0.0)
            ranked.emplace_back(scores[row][column], static_cast<int>(row));
        std::sort(ranked.begin(), ranked.end(),
                  [](const auto& lhs, const auto& rhs) {
                    return lhs.first > rhs.first;
                  });
        if (ranked.size() >= 2 &&
            ranked[0].first - ranked[1].first < ambiguity_margin) {
          ambiguous_tracks.insert(track_ids[column]);
          ambiguous_rows[ranked[0].second] = true;
          ambiguous_rows[ranked[1].second] = true;
        }
      }
      for (std::size_t row = 0; row < ambiguous_rows.size(); ++row) {
        if (!ambiguous_rows[row]) continue;
        ambiguous_detections[detection_indices[row]] = true;
        for (std::size_t column = 0; column < track_ids.size(); ++column)
          if (scores[row][column] > 0.0)
            ambiguous_tracks.insert(track_ids[column]);
        std::fill(scores[row].begin(), scores[row].end(), -1e6);
      }
    }

    const auto assignment = globalAssignment(scores);
    for (std::size_t row = 0; row < assignment.size(); ++row) {
      const int column = assignment[row];
      if (column < 0 || ambiguous_rows[row]) continue;
      const std::size_t index = detection_indices[row];
      assigned_ids[index] = track_ids[column];
      assigned_methods[index] = methods[row][column];
      used_tracks.insert(track_ids[column]);
    }
  }

  // Create bounded new tracks for the unmatched detections.
  for (std::size_t index = 0; index < prepared.size(); ++index) {
    if (assigned_ids[index] >= 0 || ambiguous_detections[index] ||
        prepared[index].message.confidence < std::max(
            impl_->config.minimum_new_track_confidence,
            impl_->config.high_confidence_threshold) ||
        impl_->tracks.size() >=
            static_cast<std::size_t>(impl_->config.maximum_tracks)) {
      continue;
    }
    PersistentTrack track;
    track.public_id = impl_->allocateId(prepared[index]);
    // A detector may omit its ID on an occasional frame.  Once a stable
    // upstream identity has been observed, retain it so that a later stable
    // observation can still take the deterministic association path.
    if (prepared[index].message.track_id_is_stable &&
        prepared[index].message.track_id >= 0) {
      track.detector_track_id = prepared[index].message.track_id;
      track.detector_id_stable = true;
    } else if (!track.detector_id_stable) {
      track.detector_track_id = prepared[index].message.track_id;
    }
    track.class_id = prepared[index].message.class_id;
    track.class_locked = impl_->config.confirmation_hits <= 1 &&
                         track.class_id >= 0;
    if (!track.class_locked && track.class_id >= 0)
      track.class_evidence[track.class_id] =
          clampValue(prepared[index].message.confidence, 0.0, 1.0);
    track.class_history.push_back(track.class_id);
    track.confidence = clampValue(prepared[index].message.confidence, 0.0, 1.0);
    track.filter.initialize(prepared[index].box, impl_->config);
    track.latest_detection = prepared[index].message;
    track.has_metric_relative_xy = metricRelativeXY(
        prepared[index].message, &track.metric_relative_xy);
    track.latest_header = detections.header;
    track.appearance = prepared[index].message.appearance_embedding;
    track.last_stamp = frame_stamp;
    const int public_id = track.public_id;
    impl_->tracks.emplace(public_id, std::move(track));
    assigned_ids[index] = public_id;
    assigned_methods[index] = "new";
    used_tracks.insert(public_id);
    ++impl_->stats.created_tracks;
  }

  // Apply every accepted measurement exactly once and rewrite its ID to the
  // persistent public ID consumed by the selected-target tracker.
  for (std::size_t index = 0; index < prepared.size(); ++index) {
    const int assigned_id = assigned_ids[index];
    if (assigned_id < 0) {
      ++impl_->stats.rejected_detections;
      continue;
    }
    auto track_it = impl_->tracks.find(assigned_id);
    if (track_it == impl_->tracks.end()) continue;
    auto& track = track_it->second;
    const bool existing_track = assigned_methods[index] != "new";
    const bool long_gap_reacquisition = existing_track &&
        tracks_missing_before_frame.count(assigned_id) != 0 &&
        track.missing_seconds >= impl_->config.reacquisition_after_sec;
    if (existing_track) {
      track.filter.update(prepared[index].box,
                          clampValue(prepared[index].message.confidence, 0.0, 1.0));
      ++track.hits;
    }
    track.missing = 0;
    track.missing_seconds = 0.0;
    track.detected = true;
    track.reidentified = assigned_methods[index] == "appearance";
    if (long_gap_reacquisition &&
        impl_->config.reacquisition_mode != "aggressive") {
      track.reacquisition_pending = true;
      track.reacquisition_hits = 0;
      track.have_reacquisition_candidate = false;
    }
    if (track.reacquisition_pending) {
      const bool same_candidate = track.have_reacquisition_candidate &&
          sameReacquisitionCandidate(track.reacquisition_box,
              track.reacquisition_candidate, prepared[index], 0.05,
              impl_->config.compatible_class_ids);
      track.reacquisition_hits = same_candidate
          ? track.reacquisition_hits + 1 : 1;
      track.reacquisition_box = prepared[index].box;
      track.reacquisition_candidate = prepared[index].message;
      track.have_reacquisition_candidate = true;
      if (track.reacquisition_hits >= static_cast<std::uint32_t>(
              impl_->config.reacquisition_confirm_hits)) {
        track.reacquisition_pending = false;
        track.reacquisition_hits = 0;
        track.have_reacquisition_candidate = false;
      }
    }
    track.association = track.reacquisition_pending ? "reacquiring" :
        assigned_methods[index];
    const double measurement_confidence = clampValue(
        prepared[index].message.confidence, 0.0, 1.0);
    track.confidence = impl_->config.confidence_smoothing_alpha * track.confidence +
        (1.0 - impl_->config.confidence_smoothing_alpha) * measurement_confidence;
    const bool provisional_class_bridge = track.class_id >= 0 &&
        prepared[index].message.class_id >= 0 &&
        track.class_id != prepared[index].message.class_id;
    if (!track.class_locked && prepared[index].message.class_id >= 0) {
      if (existing_track)
        track.class_evidence[prepared[index].message.class_id] +=
            measurement_confidence;
      const auto strongest = std::max_element(
          track.class_evidence.begin(), track.class_evidence.end(),
          [](const auto& lhs, const auto& rhs) {
            return lhs.second < rhs.second;
          });
      if (strongest != track.class_evidence.end())
        track.class_id = strongest->first;
      if (track.hits >= static_cast<std::uint32_t>(
              impl_->config.confirmation_hits)) {
        track.class_locked = true;
        track.class_evidence.clear();
      }
    }
    if (prepared[index].message.class_id >= 0 &&
        (track.class_history.empty() || track.class_history.back() !=
            prepared[index].message.class_id))
      track.class_history.push_back(prepared[index].message.class_id);
    while (track.class_history.size() > static_cast<std::size_t>(
               impl_->config.class_history_size))
      track.class_history.pop_front();
    if (prepared[index].message.track_id_is_stable &&
        prepared[index].message.track_id >= 0) {
      track.detector_track_id = prepared[index].message.track_id;
      track.detector_id_stable = true;
    } else if (!track.detector_id_stable) {
      track.detector_track_id = prepared[index].message.track_id;
    }
    if (!prepared[index].message.appearance_embedding.empty()) {
      track.appearance = prepared[index].message.appearance_embedding;
      const double box_pixels = prepared[index].box.width * image_width *
                                prepared[index].box.height * image_height;
      if (prepared[index].message.confidence >= impl_->config.high_confidence_threshold &&
          box_pixels >= 400.0) {
        track.appearance_gallery.push_back(track.appearance);
        while (track.appearance_gallery.size() >
               static_cast<std::size_t>(impl_->config.appearance_gallery_size))
          track.appearance_gallery.pop_front();
      }
    }
    bool metric_outlier_held = false;
    std::array<double, 2> observed_metric;
    if (metricRelativeXY(prepared[index].message, &observed_metric)) {
      if (!track.has_metric_relative_xy) {
        track.metric_relative_xy = observed_metric;
        track.has_metric_relative_xy = true;
      } else {
        const double jump = std::hypot(
            observed_metric[0] - track.metric_relative_xy[0],
            observed_metric[1] - track.metric_relative_xy[1]);
        const double elapsed = std::max(
            0.0, (frame_stamp - track.last_stamp).toSec());
        const double physical_gate = impl_->config.metric_relative_jump_tolerance_m +
            impl_->config.metric_relative_max_speed_mps * elapsed;
        metric_outlier_held = impl_->config.metric_relative_association_enabled &&
            existing_track && jump > physical_gate;
        if (!metric_outlier_held) {
          const double alpha = impl_->config.metric_relative_smoothing_alpha;
          for (int axis = 0; axis < 2; ++axis)
            track.metric_relative_xy[axis] =
                (1.0 - alpha) * track.metric_relative_xy[axis] +
                alpha * observed_metric[axis];
        }
      }
    }
    track.latest_detection = prepared[index].message;
    if (metric_outlier_held) {
      // Retain the last physically plausible metric anchor, while keeping the
      // actual image box and its identity. Increase uncertainty so downstream
      // guidance cannot mistake this held position for a precise fresh range.
      track.latest_detection.relative_position_body[0] =
          static_cast<float>(track.metric_relative_xy[0]);
      track.latest_detection.relative_position_body[1] =
          static_cast<float>(track.metric_relative_xy[1]);
      const float uncertainty = static_cast<float>(
          impl_->config.metric_relative_jump_tolerance_m *
          impl_->config.metric_relative_jump_tolerance_m);
      track.latest_detection.position_covariance[0] = std::max(
          track.latest_detection.position_covariance[0], uncertainty);
      track.latest_detection.position_covariance[4] = std::max(
          track.latest_detection.position_covariance[4], uncertainty);
      track.association = "metric_outlier_hold";
    }
    track.latest_detection.track_id = assigned_id;
    track.latest_detection.track_id_is_stable = true;
    // YOLO still supplies geometry each frame; its viewpoint-dependent label
    // is not allowed to relabel a confirmed physical target downstream.
    track.latest_detection.class_id = track.class_id;
    track.latest_header = detections.header;
    track.last_stamp = frame_stamp;
    if (provisional_class_bridge) track.association = "class_bridge";
    track.exit_edge = 0;
    output.candidates.candidates.push_back(track.latest_detection);
    ++impl_->stats.accepted_detections;
  }

  for (const int track_id : ambiguous_tracks) {
    const auto track = impl_->tracks.find(track_id);
    if (track != impl_->tracks.end() && !track->second.detected)
      track->second.association = "ambiguous";
  }
  for (auto& entry : impl_->tracks) {
    auto& track = entry.second;
    if (!track.detected) {
      const double decay_rate = clampValue(impl_->config.confidence_decay_rate,
                                           0.0, 1.0);
      const double elapsed_reference_frames = frame_dt * impl_->config.reference_fps;
      track.confidence *= std::pow(1.0 - decay_rate,
                                   elapsed_reference_frames);
      if (track.reacquisition_pending) {
        track.reacquisition_hits = 0;
        track.have_reacquisition_candidate = false;
      }
    }
  }

  output.tracks.tracks.reserve(impl_->tracks.size());
  for (const auto& entry : impl_->tracks) {
    const auto& track = entry.second;
    const NormalizedBox box = track.filter.box();
    xd_uav_track::TrackState state;
    state.header = detections.header;
    state.track_id = track.public_id;
    state.detector_track_id = track.detector_track_id;
    state.detector_id_stable = track.detector_id_stable;
    state.class_id = track.class_id;
    state.confidence = static_cast<float>(clampValue(track.confidence, 0.0, 1.0));
    state.normalized_bbox = {static_cast<float>(box.cx), static_cast<float>(box.cy),
                             static_cast<float>(box.width),
                             static_cast<float>(box.height)};
    state.bbox = {
        static_cast<int>(std::lround((box.cx - 0.5 * box.width) * image_width)),
        static_cast<int>(std::lround((box.cy - 0.5 * box.height) * image_height)),
        static_cast<int>(std::lround((box.cx + 0.5 * box.width) * image_width)),
        static_cast<int>(std::lround((box.cy + 0.5 * box.height) * image_height))};
    const auto image_velocity = track.filter.velocity(image_width, image_height);
    const auto state_covariance = track.filter.covariance();
    for (std::size_t index = 0; index < image_velocity.size(); ++index) {
      state.image_velocity[index] = image_velocity[index];
    }
    for (std::size_t index = 0; index < state_covariance.size(); ++index) {
      state.state_covariance[index] = state_covariance[index];
    }
    state.age_frames = track.age;
    state.hit_count = track.hits;
    state.frames_since_detection = track.missing;
    if (track.hits < static_cast<std::uint32_t>(impl_->config.confirmation_hits)) {
      state.lifecycle_state = "tentative";
    } else if (track.missing == 0) {
      state.lifecycle_state = "confirmed";
    } else if (impl_->time_based_lifecycle
                   ? track.missing_seconds <= impl_->config.occlusion_timeout_sec
                   : track.missing <= static_cast<std::uint32_t>(
                         impl_->config.occlusion_frames)) {
      state.lifecycle_state = "occluded";
    } else {
      state.lifecycle_state = "lost";
    }
    state.detected = track.detected;
    state.predicted = !track.detected;
    state.reidentification_match = track.reidentified;
    state.selected = track.public_id == impl_->selected_track_id;
    state.control_measurement_ready = track.detected &&
        !track.reacquisition_pending &&
        state.lifecycle_state == "confirmed";
    const double missing_decay = std::max(0.0, 1.0 -
        track.missing_seconds / impl_->config.removal_timeout_sec);
    state.tracking_quality = static_cast<float>(
        clampValue(track.confidence * missing_decay, 0.0, 1.0));
    state.association_method = track.association;
    const auto& latest = track.latest_detection;
    state.has_relative_position_body = latest.has_relative_position_body;
    state.has_relative_velocity_body = latest.has_relative_velocity_body;
    state.range_valid = latest.range_valid && latest.has_relative_position_body;
    state.relative_position_body = latest.relative_position_body;
    state.relative_velocity_body = latest.relative_velocity_body;
    state.position_covariance = latest.position_covariance;
    state.velocity_covariance = latest.velocity_covariance;
    output.tracks.tracks.push_back(std::move(state));
  }
  impl_->stats.active_tracks = impl_->tracks.size();
  return output;
}

bool MultiTrackManager::latestCandidate(
    const int track_id, xd_uav_track::DetectionCandidate* candidate,
    std_msgs::Header* header) const {
  if (candidate == nullptr) return false;
  const auto iterator = impl_->tracks.find(track_id);
  if (iterator == impl_->tracks.end() || !iterator->second.detected) return false;
  *candidate = iterator->second.latest_detection;
  if (header != nullptr) *header = iterator->second.latest_header;
  return true;
}

void MultiTrackManager::setSelectedTrackId(const int track_id) {
  impl_->selected_track_id = impl_->tracks.count(track_id) == 0 ? -1 : track_id;
}

int MultiTrackManager::selectedTrackId() const {
  return impl_->selected_track_id;
}

void MultiTrackManager::reset() {
  impl_->tracks.clear();
  impl_->selected_track_id = -1;
  impl_->last_frame_stamp = ros::Time();
  impl_->stats.active_tracks = 0;
}

MultiTrackStatistics MultiTrackManager::statistics() const {
  MultiTrackStatistics result = impl_->stats;
  result.active_tracks = impl_->tracks.size();
  return result;
}

}  // namespace xd_uav_track
