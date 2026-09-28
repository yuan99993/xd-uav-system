#include <xd_uav_track/multi_track_manager.hpp>

#include <algorithm>
#include <array>
#include <cmath>
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
    const double dt = clampValue(requested_dt, 1e-3, 0.5);
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

struct PersistentTrack {
  int public_id{-1};
  int detector_track_id{-1};
  bool detector_id_stable{false};
  int class_id{-1};
  double confidence{0.0};
  BoxKalmanFilter filter;
  xd_uav_track::DetectionCandidate latest_detection;
  std_msgs::Header latest_header;
  std::vector<float> appearance;
  std::uint32_t age{1};
  std::uint32_t hits{1};
  std::uint32_t missing{0};
  bool detected{true};
  bool reidentified{false};
  std::string association{"new"};
  ros::Time last_stamp;
};

}  // namespace

struct MultiTrackManager::Impl {
  explicit Impl(MultiTrackConfig requested) : config(std::move(requested)) {
    config.confirmation_hits = std::max(1, config.confirmation_hits);
    config.occlusion_frames = std::max(0, config.occlusion_frames);
    config.removal_frames = std::max(config.occlusion_frames + 1,
                                     config.removal_frames);
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
    config.appearance_minimum_cosine =
        clampValue(config.appearance_minimum_cosine, -1.0, 1.0);
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

  double dtFor(const PersistentTrack& track, const ros::Time& stamp) const {
    if (stamp.isZero() || track.last_stamp.isZero() || stamp <= track.last_stamp) {
      return 1.0 / 30.0;
    }
    return clampValue((stamp - track.last_stamp).toSec(), 1e-3, 0.5);
  }

  MultiTrackConfig config;
  std::map<int, PersistentTrack> tracks;
  int next_generated_id{1000000000};
  int selected_track_id{-1};
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

  const ros::Time frame_stamp = detections.header.stamp.isZero()
      ? ros::Time::now() : detections.header.stamp;
  for (auto& entry : impl_->tracks) {
    entry.second.filter.predict(impl_->dtFor(entry.second, frame_stamp));
    ++entry.second.age;
    ++entry.second.missing;
    entry.second.detected = false;
    entry.second.reidentified = false;
    entry.second.association = "predicted";
  }

  std::set<int> used_tracks;
  std::vector<int> assigned_ids(prepared.size(), -1);
  std::vector<std::string> assigned_methods(prepared.size());

  // Stable detector IDs have first priority, but a duplicated ID in one frame
  // is deliberately not allowed to update the same state twice.
  for (std::size_t index = 0; index < prepared.size(); ++index) {
    const auto& detection = prepared[index];
    if (!detection.message.track_id_is_stable || detection.message.track_id < 0) continue;
    for (const auto& entry : impl_->tracks) {
      const auto& track = entry.second;
      if (used_tracks.count(entry.first) == 0 && track.detector_id_stable &&
          track.detector_track_id == detection.message.track_id) {
        assigned_ids[index] = entry.first;
        assigned_methods[index] = "detector_id";
        used_tracks.insert(entry.first);
        break;
      }
    }
  }

  // Remaining candidates use bounded geometry, with appearance as a
  // position-gated tie breaker. This cannot jump across the entire frame.
  for (std::size_t index = 0; index < prepared.size(); ++index) {
    if (assigned_ids[index] >= 0) continue;
    const auto& detection = prepared[index];
    int best_id = -1;
    double best_score = -std::numeric_limits<double>::infinity();
    std::string best_method;
    for (const auto& entry : impl_->tracks) {
      if (used_tracks.count(entry.first) != 0) continue;
      const auto& track = entry.second;
      if (track.class_id >= 0 && detection.message.class_id >= 0 &&
          track.class_id != detection.message.class_id) continue;
      const NormalizedBox predicted = track.filter.box();
      const double overlap = intersectionOverUnion(predicted, detection.box);
      const double distance = normalizedCenterDistance(predicted, detection.box);
      if (overlap < impl_->config.association_iou_threshold &&
          distance > impl_->config.association_center_distance) continue;
      const double appearance = cosineSimilarity(
          track.appearance, detection.message.appearance_embedding);
      const bool appearance_match =
          appearance >= impl_->config.appearance_minimum_cosine;
      const double score = 2.0 * overlap - 0.15 * distance +
                           (appearance_match ? 0.35 * appearance : 0.0);
      if (score > best_score) {
        best_score = score;
        best_id = entry.first;
        best_method = appearance_match && overlap < 0.5
                          ? "appearance" : "spatial";
      }
    }
    if (best_id >= 0) {
      assigned_ids[index] = best_id;
      assigned_methods[index] = best_method;
      used_tracks.insert(best_id);
    }
  }

  // Create bounded new tracks for the unmatched detections.
  for (std::size_t index = 0; index < prepared.size(); ++index) {
    if (assigned_ids[index] >= 0 ||
        prepared[index].message.confidence <
            impl_->config.minimum_new_track_confidence ||
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
    track.confidence = clampValue(prepared[index].message.confidence, 0.0, 1.0);
    track.filter.initialize(prepared[index].box, impl_->config);
    track.latest_detection = prepared[index].message;
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
    if (assigned_methods[index] != "new") {
      track.filter.update(prepared[index].box,
                          clampValue(prepared[index].message.confidence, 0.0, 1.0));
      ++track.hits;
    }
    track.missing = 0;
    track.detected = true;
    track.reidentified = assigned_methods[index] == "appearance";
    track.association = assigned_methods[index];
    track.confidence = 0.75 * clampValue(prepared[index].message.confidence, 0.0, 1.0) +
                       0.25 * track.confidence;
    track.class_id = prepared[index].message.class_id;
    if (prepared[index].message.track_id_is_stable &&
        prepared[index].message.track_id >= 0) {
      track.detector_track_id = prepared[index].message.track_id;
      track.detector_id_stable = true;
    } else if (!track.detector_id_stable) {
      track.detector_track_id = prepared[index].message.track_id;
    }
    if (!prepared[index].message.appearance_embedding.empty()) {
      track.appearance = prepared[index].message.appearance_embedding;
    }
    track.latest_detection = prepared[index].message;
    track.latest_detection.track_id = assigned_id;
    track.latest_detection.track_id_is_stable = true;
    track.latest_header = detections.header;
    track.last_stamp = frame_stamp;
    output.candidates.candidates.push_back(track.latest_detection);
    ++impl_->stats.accepted_detections;
  }

  for (auto iterator = impl_->tracks.begin(); iterator != impl_->tracks.end();) {
    if (iterator->second.missing >
        static_cast<std::uint32_t>(impl_->config.removal_frames)) {
      if (impl_->selected_track_id == iterator->first) {
        impl_->selected_track_id = -1;
      }
      iterator = impl_->tracks.erase(iterator);
      ++impl_->stats.removed_tracks;
    } else {
      ++iterator;
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
    } else if (track.missing <=
               static_cast<std::uint32_t>(impl_->config.occlusion_frames)) {
      state.lifecycle_state = "occluded";
    } else {
      state.lifecycle_state = "lost";
    }
    state.detected = track.detected;
    state.predicted = !track.detected;
    state.reidentification_match = track.reidentified;
    state.selected = track.public_id == impl_->selected_track_id;
    state.control_measurement_ready = track.detected &&
        state.lifecycle_state == "confirmed";
    const double missing_decay = std::max(0.0, 1.0 -
        static_cast<double>(track.missing) /
            std::max(1, impl_->config.removal_frames));
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
  impl_->stats.active_tracks = 0;
}

MultiTrackStatistics MultiTrackManager::statistics() const {
  MultiTrackStatistics result = impl_->stats;
  result.active_tracks = impl_->tracks.size();
  return result;
}

}  // namespace xd_uav_track
