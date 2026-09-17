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

double embeddingNorm(const std::vector<float>& embedding) {
  if (embedding.empty()) return -1.0;
  double squared_norm = 0.0;
  for (const float value : embedding) {
    if (!finiteFloat(value)) return -1.0;
    squared_norm += static_cast<double>(value) * value;
  }
  return squared_norm > 1e-12 ? std::sqrt(squared_norm) : -1.0;
}

double cosineSimilarityWithRhsNorm(const std::vector<float>& lhs,
                                   const std::vector<float>& rhs,
                                   const double rhs_norm) {
  // -2 is an out-of-domain sentinel; -1 is a valid cosine value.
  if (lhs.empty() || lhs.size() != rhs.size() || rhs_norm <= 0.0) return -2.0;
  double dot = 0.0;
  double lhs_squared_norm = 0.0;
  for (std::size_t index = 0; index < lhs.size(); ++index) {
    if (!finiteFloat(lhs[index]) || !finiteFloat(rhs[index])) return -2.0;
    dot += static_cast<double>(lhs[index]) * rhs[index];
    lhs_squared_norm += static_cast<double>(lhs[index]) * lhs[index];
  }
  if (lhs_squared_norm <= 1e-12) return -2.0;
  return dot / (std::sqrt(lhs_squared_norm) * rhs_norm);
}

double cosineSimilarity(const std::vector<float>& lhs,
                        const std::vector<float>& rhs) {
  return cosineSimilarityWithRhsNorm(lhs, rhs, embeddingNorm(rhs));
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

  void observationCentricReupdate(const NormalizedBox& previous,
                                  const NormalizedBox& current,
                                  const double dt, const double blend) {
    if (!(std::isfinite(dt) && dt > 1e-3)) return;
    const double clipped = clampValue(blend, 0.0, 1.0);
    const std::array<double, 4> prior{{previous.cx, previous.cy,
        std::log(previous.width), std::log(previous.height)}};
    const std::array<double, 4> observed{{current.cx, current.cy,
        std::log(current.width), std::log(current.height)}};
    for (int index = 0; index < 4; ++index) {
      const double velocity = (observed[index] - prior[index]) / dt;
      state_(index + 4) = (1.0 - clipped) * state_(index + 4) +
          clipped * velocity;
    }
  }

  void compensateCameraMotion(const std::array<double, 9>& homography) {
    const NormalizedBox before = box();
    const std::array<std::array<double, 2>, 4> corners{{
        {{before.cx - 0.5 * before.width, before.cy - 0.5 * before.height}},
        {{before.cx + 0.5 * before.width, before.cy - 0.5 * before.height}},
        {{before.cx + 0.5 * before.width, before.cy + 0.5 * before.height}},
        {{before.cx - 0.5 * before.width, before.cy + 0.5 * before.height}}}};
    double x_min = std::numeric_limits<double>::infinity();
    double y_min = std::numeric_limits<double>::infinity();
    double x_max = -std::numeric_limits<double>::infinity();
    double y_max = -std::numeric_limits<double>::infinity();
    for (const auto& point : corners) {
      const double denominator = homography[6] * point[0] +
          homography[7] * point[1] + homography[8];
      if (!std::isfinite(denominator) || std::abs(denominator) < 1e-8) return;
      const double x = (homography[0] * point[0] + homography[1] * point[1] +
                        homography[2]) / denominator;
      const double y = (homography[3] * point[0] + homography[4] * point[1] +
                        homography[5]) / denominator;
      if (!std::isfinite(x) || !std::isfinite(y)) return;
      x_min = std::min(x_min, x); y_min = std::min(y_min, y);
      x_max = std::max(x_max, x); y_max = std::max(y_max, y);
    }
    const double width = x_max - x_min;
    const double height = y_max - y_min;
    if (!(width > 1e-4 && height > 1e-4 && width < 4.0 && height < 4.0)) return;
    state_(0) = 0.5 * (x_min + x_max);
    state_(1) = 0.5 * (y_min + y_max);
    state_(2) = std::log(width);
    state_(3) = std::log(height);
    // The homography explains platform rotation, not target velocity. Leave a
    // damped residual velocity for true target motion and inflate position
    // uncertainty slightly for calibration/model error.
    state_.segment<4>(4) *= 0.75;
    covariance_.block<4, 4>(0, 0) *= 1.15;
  }

  double mahalanobisDistance(const NormalizedBox& box,
                             const double confidence) const {
    Eigen::Matrix<double, 4, 8> observation;
    observation.setZero();
    observation(0, 0) = observation(1, 1) = 1.0;
    observation(2, 2) = observation(3, 3) = 1.0;
    Eigen::Matrix<double, 4, 1> measurement;
    measurement << box.cx, box.cy, std::log(box.width), std::log(box.height);
    Eigen::Matrix4d noise = Eigen::Matrix4d::Identity();
    noise.diagonal() << 0.0025, 0.0025, 0.01, 0.01;
    noise *= measurement_noise_ / std::max(0.05, confidence);
    const Eigen::Matrix<double, 4, 1> residual = measurement - observation * state_;
    const Eigen::Matrix4d innovation =
        observation * covariance_ * observation.transpose() + noise;
    const double value = (residual.transpose() *
        innovation.ldlt().solve(residual))(0, 0);
    return std::isfinite(value) ? std::max(0.0, value)
                                : std::numeric_limits<double>::infinity();
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
  explicit PreparedDetection(
      const xd_uav_track::DetectionCandidate& source) : message(source) {}

  // The input DetectionArray remains alive for this entire update. Keeping a
  // reference avoids copying every appearance embedding before association.
  const xd_uav_track::DetectionCandidate& message;
  double appearance_norm{-1.0};
  double appearance_quality{0.0};
  NormalizedBox box;
  std::size_t original_index{0};
  std::string identity_label;
  std::string identity_source_type;
  double identity_confidence{0.0};
  ros::Time identity_stamp;
  bool memory_ambiguous{false};
  bool world_valid{false};
  std::array<double, 3> world_position{{0.0, 0.0, 0.0}};
  std::array<double, 3> world_velocity{{0.0, 0.0, 0.0}};
  bool world_velocity_valid{false};
  double world_sigma_m{1.0};
  ros::Time world_stamp;
};

struct PersistentTrack {
  int public_id{-1};
  int detector_track_id{-1};
  bool detector_id_stable{false};
  int class_id{-1};
  double confidence{0.0};
  BoxKalmanFilter filter;
  NormalizedBox last_observation_box;
  xd_uav_track::DetectionCandidate latest_detection;
  std_msgs::Header latest_header;
  std::vector<float> appearance;
  std::vector<std::vector<float>> appearance_gallery;
  int group_id{-1};
  std::string image_source;
  std::string stable_identity_label;
  std::map<std::string, int> identity_label_votes;
  ros::Time last_identity_hint_stamp;
  int reacquisition_hits_remaining{0};
  bool identity_ambiguous{false};
  bool world_valid{false};
  std::array<double, 3> world_position{{0.0, 0.0, 0.0}};
  std::array<double, 3> world_velocity{{0.0, 0.0, 0.0}};
  bool world_velocity_valid{false};
  double world_sigma_m{1.0};
  ros::Time world_stamp;
  std::uint32_t age{1};
  std::uint32_t hits{1};
  std::uint32_t missing{0};
  bool detected{true};
  bool reidentified{false};
  std::string association{"new"};
  ros::Time last_stamp;
  ros::Time last_detection_stamp;
};

struct GroupPrototype {
  int group_id{-1};
  int class_id{-1};
  double log_aspect{0.0};
  std::vector<float> appearance;
  std::string image_source;
  std::uint32_t samples{0};
  ros::Time last_stamp;
};

struct MemoryIdentity {
  int public_id{-1};
  int class_id{-1};
  int group_id{-1};
  NormalizedBox last_box;
  std::vector<std::vector<float>> appearance_gallery;
  xd_uav_track::DetectionCandidate latest_detection;
  std::string stable_identity_label;
  std::string image_source;
  double confidence{0.0};
  std::uint32_t hits{0};
  ros::Time last_detection_stamp;
  bool world_valid{false};
  std::array<double, 3> world_position{{0.0, 0.0, 0.0}};
  std::array<double, 3> world_velocity{{0.0, 0.0, 0.0}};
  bool world_velocity_valid{false};
  double world_sigma_m{1.0};
  ros::Time world_stamp;
};

enum class AssociationKind : std::uint8_t {
  kNone = 0,
  kGlobalSpatial,
  kAppearance,
  kGroup,
  kLongTerm,
  kGroupMemory,
  kNumber,
};

const char* associationName(const AssociationKind kind) {
  switch (kind) {
    case AssociationKind::kGlobalSpatial: return "global_spatial";
    case AssociationKind::kAppearance: return "appearance";
    case AssociationKind::kGroup: return "group_reid";
    case AssociationKind::kLongTerm: return "long_term_reid";
    case AssociationKind::kGroupMemory: return "group_memory_reid";
    case AssociationKind::kNumber: return "number_reid";
    case AssociationKind::kNone: break;
  }
  return "";
}

double gallerySimilarity(const PersistentTrack& track,
                         const std::vector<float>& embedding,
                         const double embedding_norm) {
  double best = cosineSimilarityWithRhsNorm(
      track.appearance, embedding, embedding_norm);
  for (const auto& stored : track.appearance_gallery) {
    best = std::max(best, cosineSimilarityWithRhsNorm(
        stored, embedding, embedding_norm));
  }
  return best;
}

double gallerySimilarity(const MemoryIdentity& identity,
                         const std::vector<float>& embedding,
                         const double embedding_norm) {
  double best = -2.0;
  for (const auto& stored : identity.appearance_gallery) {
    best = std::max(best, cosineSimilarityWithRhsNorm(
        stored, embedding, embedding_norm));
  }
  return best;
}

double aspectDistance(const NormalizedBox& lhs, const NormalizedBox& rhs) {
  const double lhs_aspect = lhs.width / std::max(1e-6, lhs.height);
  const double rhs_aspect = rhs.width / std::max(1e-6, rhs.height);
  return std::abs(std::log(lhs_aspect / rhs_aspect));
}

double hintIntersectionOverUnion(const TargetIdentityHint& hint,
                                const NormalizedBox& box) {
  NormalizedBox hint_box;
  hint_box.cx = hint.normalized_bbox[0];
  hint_box.cy = hint.normalized_bbox[1];
  hint_box.width = hint.normalized_bbox[2];
  hint_box.height = hint.normalized_bbox[3];
  if (hint_box.width <= 0.0 || hint_box.height <= 0.0) return 0.0;
  return intersectionOverUnion(hint_box, box);
}

// Rectangular Hungarian minimum-cost assignment. Invalid edges must use a
// value larger than kUnmatchedCost; they are never returned as matches.
std::vector<int> hungarianAssignment(const std::vector<std::vector<double>>& cost,
                                     const double unmatched_cost,
                                     const double invalid_cost) {
  const std::size_t rows = cost.size();
  const std::size_t cols = rows == 0 ? 0 : cost.front().size();
  // Add explicit dummy rows/columns even for a square matrix.  Without them
  // an all-invalid detection would be forced to consume a real track and
  // could prevent a later valid detection from receiving that track.
  const std::size_t size = rows + cols;
  std::vector<int> result(rows, -1);
  if (size == 0) return result;
  std::vector<double> u(size + 1), v(size + 1);
  std::vector<std::size_t> p(size + 1), way(size + 1);
  for (std::size_t i = 1; i <= size; ++i) {
    p[0] = i;
    std::size_t j0 = 0;
    std::vector<double> minv(size + 1, std::numeric_limits<double>::infinity());
    std::vector<bool> used(size + 1, false);
    do {
      used[j0] = true;
      const std::size_t i0 = p[j0];
      double delta = std::numeric_limits<double>::infinity();
      std::size_t j1 = 0;
      for (std::size_t j = 1; j <= size; ++j) {
        if (used[j]) continue;
        const double current = (i0 <= rows && j <= cols)
            ? cost[i0 - 1][j - 1] : unmatched_cost;
        const double bounded = std::isfinite(current) ? current : invalid_cost;
        const double candidate = bounded - u[i0] - v[j];
        if (candidate < minv[j]) {
          minv[j] = candidate;
          way[j] = j0;
        }
        if (minv[j] < delta) {
          delta = minv[j];
          j1 = j;
        }
      }
      for (std::size_t j = 0; j <= size; ++j) {
        if (used[j]) { u[p[j]] += delta; v[j] -= delta; }
        else { minv[j] -= delta; }
      }
      j0 = j1;
    } while (p[j0] != 0);
    do {
      const std::size_t j1 = way[j0];
      p[j0] = p[j1];
      j0 = j1;
    } while (j0 != 0);
  }
  for (std::size_t j = 1; j <= size; ++j) {
    if (p[j] != 0 && p[j] <= rows && j <= cols &&
        cost[p[j] - 1][j - 1] < unmatched_cost) {
      result[p[j] - 1] = static_cast<int>(j - 1);
    }
  }
  return result;
}

// Split a sparse bipartite graph into independent connected components before
// invoking the dense Hungarian solver. Camera scenes commonly contain many
// tracks, but motion/class gates leave only small local components; solving a
// single (detections + tracks)^3 padded matrix wastes most of the callback.
std::vector<int> sparseHungarianAssignment(
    const std::vector<std::vector<double>>& cost,
    const double unmatched_cost, const double invalid_cost) {
  const std::size_t rows = cost.size();
  const std::size_t cols = rows == 0 ? 0 : cost.front().size();
  std::vector<int> result(rows, -1);
  if (rows == 0 || cols == 0) return result;
  std::vector<std::vector<std::size_t>> row_edges(rows), col_edges(cols);
  for (std::size_t row = 0; row < rows; ++row) {
    for (std::size_t col = 0; col < cols; ++col) {
      if (cost[row][col] < unmatched_cost) {
        row_edges[row].push_back(col);
        col_edges[col].push_back(row);
      }
    }
  }
  std::vector<bool> visited_rows(rows, false), visited_cols(cols, false);
  for (std::size_t start = 0; start < rows; ++start) {
    if (visited_rows[start] || row_edges[start].empty()) continue;
    std::vector<std::size_t> component_rows, component_cols;
    std::deque<std::pair<bool, std::size_t>> queue;
    visited_rows[start] = true;
    queue.emplace_back(true, start);
    while (!queue.empty()) {
      const auto item = queue.front();
      queue.pop_front();
      if (item.first) {
        component_rows.push_back(item.second);
        for (const std::size_t col : row_edges[item.second]) {
          if (!visited_cols[col]) {
            visited_cols[col] = true;
            queue.emplace_back(false, col);
          }
        }
      } else {
        component_cols.push_back(item.second);
        for (const std::size_t row : col_edges[item.second]) {
          if (!visited_rows[row]) {
            visited_rows[row] = true;
            queue.emplace_back(true, row);
          }
        }
      }
    }
    std::vector<std::vector<double>> component_cost(
        component_rows.size(),
        std::vector<double>(component_cols.size(), invalid_cost));
    for (std::size_t row = 0; row < component_rows.size(); ++row) {
      for (std::size_t col = 0; col < component_cols.size(); ++col) {
        component_cost[row][col] =
            cost[component_rows[row]][component_cols[col]];
      }
    }
    const auto component_assignment = hungarianAssignment(
        component_cost, unmatched_cost, invalid_cost);
    for (std::size_t row = 0; row < component_assignment.size(); ++row) {
      if (component_assignment[row] >= 0) {
        result[component_rows[row]] = static_cast<int>(
            component_cols[static_cast<std::size_t>(component_assignment[row])]);
      }
    }
  }
  return result;
}

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
    config.high_confidence_threshold =
        clampValue(config.high_confidence_threshold, 0.0, 1.0);
    config.low_confidence_threshold = clampValue(
        config.low_confidence_threshold, 0.0, config.high_confidence_threshold);
    config.appearance_gallery_size = std::max(1, config.appearance_gallery_size);
    config.appearance_update_minimum_confidence = clampValue(
        config.appearance_update_minimum_confidence, 0.0, 1.0);
    config.appearance_minimum_quality = clampValue(
        config.appearance_minimum_quality, 0.0, 1.0);
    config.appearance_low_quality_weight = clampValue(
        config.appearance_low_quality_weight, 0.0, 1.0);
    config.observation_centric_velocity_blend = clampValue(
        config.observation_centric_velocity_blend, 0.0, 1.0);
    config.association_mahalanobis_gate =
        std::max(1e-3, config.association_mahalanobis_gate);
    config.metric_innovation_distance_m =
        std::max(0.0, config.metric_innovation_distance_m);
    config.group_appearance_minimum_cosine = clampValue(
        config.group_appearance_minimum_cosine, -1.0, 1.0);
    config.group_aspect_log_gate = std::max(1e-3, config.group_aspect_log_gate);
    config.maximum_groups = std::max(1, config.maximum_groups);
    config.group_trigger_minimum_features = std::max(
        1, std::min(5, config.group_trigger_minimum_features));
    config.group_trigger_score_margin = std::max(
        0.0, config.group_trigger_score_margin);
    config.short_occlusion_sec = std::max(0.0, config.short_occlusion_sec);
    config.long_association_minimum_margin = std::max(
        0.0, config.long_association_minimum_margin);
    config.long_term_memory_ttl_sec = std::max(
        0.0, config.long_term_memory_ttl_sec);
    config.long_term_memory_maximum_identities = std::max(
        1, config.long_term_memory_maximum_identities);
    config.long_term_memory_minimum_cosine = clampValue(
        config.long_term_memory_minimum_cosine, -1.0, 1.0);
    config.long_term_memory_minimum_margin = std::max(
        0.0, config.long_term_memory_minimum_margin);
    config.long_term_memory_maximum_cost = clampValue(
        config.long_term_memory_maximum_cost, 0.0, 1.0);
    config.long_term_reconfirmation_hits = std::max(
        2, config.long_term_reconfirmation_hits);
    config.identity_hint_minimum_confidence = clampValue(
        config.identity_hint_minimum_confidence, 0.0, 1.0);
    config.identity_hint_hard_confidence = clampValue(
        config.identity_hint_hard_confidence,
        config.identity_hint_minimum_confidence, 1.0);
    config.identity_hint_iou_gate = clampValue(
        config.identity_hint_iou_gate, 0.0, 1.0);
    config.identity_hint_confirmations = std::max(
        2, config.identity_hint_confirmations);
    config.world_innovation_gate_sigma = std::max(
        1.0, config.world_innovation_gate_sigma);
    config.world_process_noise_mps = std::max(
        0.0, config.world_process_noise_mps);
  }

  int allocateId(const PreparedDetection& detection) {
    if (detection.message.track_id_is_stable && detection.message.track_id >= 0 &&
        tracks.count(detection.message.track_id) == 0) {
      return detection.message.track_id;
    }
    while (tracks.count(next_generated_id) != 0 ||
           identity_memory.count(next_generated_id) != 0 ||
           next_generated_id < 0) {
      if (next_generated_id == std::numeric_limits<int>::max()) {
        next_generated_id = 1000000000;
      } else {
        ++next_generated_id;
      }
    }
    return next_generated_id++;
  }

  bool hardLabelConflict(const std::string& stable_label,
                         const PreparedDetection& detection) const {
    return !stable_label.empty() && !detection.identity_label.empty() &&
        stable_label != detection.identity_label &&
        detection.identity_confidence >= config.identity_hint_hard_confidence;
  }

  void updateIdentityLabel(PersistentTrack* track,
                           const PreparedDetection& detection) const {
    if (track == nullptr || detection.identity_label.empty() ||
        detection.identity_confidence < config.identity_hint_minimum_confidence) {
      return;
    }
    if (!detection.identity_stamp.isZero() &&
        !track->last_identity_hint_stamp.isZero() &&
        detection.identity_stamp <= track->last_identity_hint_stamp) return;
    if (!detection.identity_stamp.isZero()) {
      track->last_identity_hint_stamp = detection.identity_stamp;
    }
    // An established physical label is immutable during this track lifetime.
    // Contradictory samples are association evidence, never replacement data.
    if (!track->stable_identity_label.empty()) return;
    int& votes = track->identity_label_votes[detection.identity_label];
    ++votes;
    if (votes >= config.identity_hint_confirmations) {
      track->stable_identity_label = detection.identity_label;
    }
  }

  void updateAppearance(PersistentTrack* track,
                        const std::vector<float>& embedding) const {
    if (track == nullptr || embedding.empty()) return;
    track->appearance = embedding;
    std::size_t closest = track->appearance_gallery.size();
    double closest_similarity = -2.0;
    for (std::size_t i = 0; i < track->appearance_gallery.size(); ++i) {
      const double similarity = cosineSimilarity(
          track->appearance_gallery[i], embedding);
      if (similarity > closest_similarity) {
        closest_similarity = similarity;
        closest = i;
      }
    }
    // Repeated nearly-identical views refresh one prototype instead of
    // crowding out illumination/viewpoint diversity.
    if (closest < track->appearance_gallery.size() &&
        closest_similarity >= 0.985) {
      track->appearance_gallery[closest] = embedding;
      return;
    }
    track->appearance_gallery.push_back(embedding);
    while (track->appearance_gallery.size() >
           static_cast<std::size_t>(config.appearance_gallery_size)) {
      // Discard one member of the most redundant pair, preserving rare views
      // (shadow, zoom, fixed-camera and gimbal appearances) for long ReID.
      std::size_t remove_index = 0;
      double most_redundant = -2.0;
      for (std::size_t i = 0; i < track->appearance_gallery.size(); ++i) {
        for (std::size_t j = i + 1; j < track->appearance_gallery.size(); ++j) {
          const double similarity = cosineSimilarity(
              track->appearance_gallery[i], track->appearance_gallery[j]);
          if (similarity > most_redundant) {
            most_redundant = similarity;
            remove_index = i;
          }
        }
      }
      track->appearance_gallery.erase(
          track->appearance_gallery.begin() + remove_index);
    }
  }

  void updateWorld(PersistentTrack* track,
                   const PreparedDetection& detection) const {
    if (track == nullptr || !detection.world_valid) return;
    if (track->world_valid && !track->world_stamp.isZero() &&
        !detection.world_stamp.isZero() &&
        detection.world_stamp > track->world_stamp &&
        !detection.world_velocity_valid) {
      const double dt = (detection.world_stamp - track->world_stamp).toSec();
      if (dt > 1e-3 && dt < 2.0) {
        for (std::size_t axis = 0; axis < 3; ++axis) {
          const double measured = (detection.world_position[axis] -
                                   track->world_position[axis]) / dt;
          track->world_velocity[axis] = track->world_velocity_valid
              ? 0.7 * track->world_velocity[axis] + 0.3 * measured : measured;
        }
        track->world_velocity_valid = true;
      }
    }
    track->world_position = detection.world_position;
    if (detection.world_velocity_valid) {
      track->world_velocity = detection.world_velocity;
      track->world_velocity_valid = true;
    }
    track->world_sigma_m = std::max(0.05, detection.world_sigma_m);
    track->world_stamp = detection.world_stamp;
    track->world_valid = true;
  }

  bool groupCompatible(const GroupPrototype& group,
                       const PreparedDetection& detection,
                       const std::string& source) const {
    if (group.class_id >= 0 && detection.message.class_id >= 0 &&
        group.class_id != detection.message.class_id) return false;
    if (config.group_source_strict && !group.image_source.empty() &&
        !source.empty() && group.image_source != source) return false;
    const double log_aspect = std::log(detection.box.width /
                                      std::max(1e-6, detection.box.height));
    if (std::abs(group.log_aspect - log_aspect) >
        config.group_aspect_log_gate) return false;
    const double appearance = cosineSimilarityWithRhsNorm(
        group.appearance, detection.message.appearance_embedding,
        detection.appearance_norm);
    if (appearance >= -1.0 &&
        appearance < config.group_appearance_minimum_cosine) return false;
    return true;
  }

  int findOrCreateGroup(const PreparedDetection& detection,
                        const std::string& source,
                        const ros::Time& stamp) {
    if (!config.group_reid_enabled) return -1;
    int best_id = -1;
    double best_score = -std::numeric_limits<double>::infinity();
    const double log_aspect = std::log(detection.box.width /
                                      std::max(1e-6, detection.box.height));
    for (const auto& entry : groups) {
      const GroupPrototype& group = entry.second;
      if (!groupCompatible(group, detection, source)) continue;
      const double appearance = cosineSimilarityWithRhsNorm(
          group.appearance, detection.message.appearance_embedding,
          detection.appearance_norm);
      const double shape = 1.0 - clampValue(
          std::abs(group.log_aspect - log_aspect) /
              config.group_aspect_log_gate, 0.0, 1.0);
      const double score = 0.65 * (appearance >= -1.0 ? appearance : 0.5) +
                           0.35 * shape;
      if (score > best_score) { best_score = score; best_id = entry.first; }
    }
    if (best_id >= 0) return best_id;
    if (groups.size() >= static_cast<std::size_t>(config.maximum_groups)) {
      std::set<int> referenced;
      for (const auto& entry : tracks) referenced.insert(entry.second.group_id);
      for (const auto& entry : identity_memory)
        referenced.insert(entry.second.group_id);
      auto oldest = groups.end();
      for (auto it = groups.begin(); it != groups.end(); ++it) {
        if (referenced.count(it->first) != 0) continue;
        if (oldest == groups.end() ||
            it->second.last_stamp < oldest->second.last_stamp) oldest = it;
      }
      if (oldest != groups.end()) groups.erase(oldest);
      else return -1;
    }
    GroupPrototype group;
    group.group_id = next_group_id++;
    group.class_id = detection.message.class_id;
    group.log_aspect = log_aspect;
    group.appearance = detection.message.appearance_embedding;
    group.image_source = source;
    group.samples = 1;
    group.last_stamp = stamp;
    const int id = group.group_id;
    groups.emplace(id, std::move(group));
    return id;
  }

  void updateGroup(const PersistentTrack& track,
                   const PreparedDetection& detection,
                   const ros::Time& stamp) {
    if (track.group_id < 0) return;
    auto it = groups.find(track.group_id);
    if (it == groups.end()) return;
    GroupPrototype& group = it->second;
    const double alpha = 1.0 / std::min<std::uint32_t>(20, group.samples + 1);
    const double log_aspect = std::log(detection.box.width /
                                      std::max(1e-6, detection.box.height));
    group.log_aspect = (1.0 - alpha) * group.log_aspect + alpha * log_aspect;
    if (!detection.message.appearance_embedding.empty() &&
        detection.message.confidence >= config.appearance_update_minimum_confidence) {
      if (group.appearance.size() != detection.message.appearance_embedding.size()) {
        group.appearance = detection.message.appearance_embedding;
      } else {
        for (std::size_t i = 0; i < group.appearance.size(); ++i) {
          group.appearance[i] = static_cast<float>((1.0 - alpha) *
              group.appearance[i] + alpha * detection.message.appearance_embedding[i]);
        }
      }
    }
    group.samples++;
    group.last_stamp = stamp;
  }

  void remember(PersistentTrack* track) {
    if (track == nullptr) return;
    if (!config.long_term_memory_enabled ||
        track->identity_ambiguous ||
        track->hits < static_cast<std::uint32_t>(config.confirmation_hits)) return;
    MemoryIdentity memory;
    memory.public_id = track->public_id;
    memory.class_id = track->class_id;
    memory.group_id = track->group_id;
    memory.last_box = track->filter.box();
    memory.appearance_gallery = std::move(track->appearance_gallery);
    if (memory.appearance_gallery.empty() && !track->appearance.empty()) {
      memory.appearance_gallery.push_back(std::move(track->appearance));
    }
    memory.latest_detection = std::move(track->latest_detection);
    memory.stable_identity_label = std::move(track->stable_identity_label);
    memory.image_source = std::move(track->image_source);
    memory.confidence = track->confidence;
    memory.hits = track->hits;
    memory.last_detection_stamp = track->last_detection_stamp;
    memory.world_valid = track->world_valid;
    memory.world_position = track->world_position;
    memory.world_velocity = track->world_velocity;
    memory.world_velocity_valid = track->world_velocity_valid;
    memory.world_sigma_m = track->world_sigma_m;
    memory.world_stamp = track->world_stamp;
    identity_memory[memory.public_id] = std::move(memory);
    while (identity_memory.size() >
           static_cast<std::size_t>(config.long_term_memory_maximum_identities)) {
      // Confidence buys a limited grace period, while sufficiently old
      // memories are still evicted. This is safer than retaining either only
      // the newest low-quality clutter or only an ancient high score.
      auto oldest = std::min_element(identity_memory.begin(), identity_memory.end(),
          [](const auto& lhs, const auto& rhs) {
            const double lhs_score = lhs.second.last_detection_stamp.toSec() +
                                     30.0 * lhs.second.confidence;
            const double rhs_score = rhs.second.last_detection_stamp.toSec() +
                                     30.0 * rhs.second.confidence;
            return lhs_score < rhs_score;
          });
      if (oldest == identity_memory.end()) break;
      identity_memory.erase(oldest);
    }
  }

  void pruneMemory(const ros::Time& stamp) {
    if (stamp.isZero()) return;
    for (auto it = identity_memory.begin(); it != identity_memory.end();) {
      const double age = !it->second.last_detection_stamp.isZero() &&
          stamp >= it->second.last_detection_stamp
          ? (stamp - it->second.last_detection_stamp).toSec() : 0.0;
      if (age > config.long_term_memory_ttl_sec) it = identity_memory.erase(it);
      else ++it;
    }
  }

  double dtFor(const PersistentTrack& track, const ros::Time& stamp) const {
    if (stamp.isZero() || track.last_stamp.isZero() || stamp <= track.last_stamp) {
      return 1.0 / 30.0;
    }
    return clampValue((stamp - track.last_stamp).toSec(), 1e-3, 0.5);
  }

  MultiTrackConfig config;
  std::map<int, PersistentTrack> tracks;
  std::map<int, MemoryIdentity> identity_memory;
  std::map<int, GroupPrototype> groups;
  int next_generated_id{1000000000};
  int next_group_id{1};
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
  return update(detections, image_width, image_height, image_source, {});
}

ManagedDetectionFrame MultiTrackManager::update(
    const xd_uav_track::DetectionArray& detections, const int image_width,
    const int image_height, const std::string& image_source,
    const std::vector<TargetIdentityHint>& identity_hints) {
  return update(detections, image_width, image_height, image_source,
                identity_hints, {});
}

ManagedDetectionFrame MultiTrackManager::update(
    const xd_uav_track::DetectionArray& detections, const int image_width,
    const int image_height, const std::string& image_source,
    const std::vector<TargetIdentityHint>& identity_hints,
    const std::vector<TargetWorldObservation>& world_observations) {
  return update(detections, image_width, image_height, image_source,
                identity_hints, world_observations, {});
}

ManagedDetectionFrame MultiTrackManager::update(
    const xd_uav_track::DetectionArray& detections, const int image_width,
    const int image_height, const std::string& image_source,
    const std::vector<TargetIdentityHint>& identity_hints,
    const std::vector<TargetWorldObservation>& world_observations,
    const CameraMotionCompensation& camera_motion) {
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
    PreparedDetection item(raw);
    item.box = box;
    item.original_index = index;
    item.appearance_norm = embeddingNorm(raw.appearance_embedding);
    const double declared_quality = std::isfinite(raw.appearance_quality)
        ? clampValue(raw.appearance_quality, 0.0, 1.0) : 0.0;
    // Legacy producers have no quality field and publish its ROS default 0.
    // Preserve their established association behavior; quality-aware encoders
    // encode an explicit non-zero score (including 0.001 for unusable ROIs).
    item.appearance_quality = raw.appearance_embedding.empty() ? 0.0 :
        (declared_quality > 0.0 ? declared_quality : 1.0);
    prepared.push_back(std::move(item));
  }

  std::vector<const TargetWorldObservation*> world_by_candidate(
      detections.candidates.size(), nullptr);
  for (const auto& world : world_observations) {
    if (world.candidate_index < world_by_candidate.size()) {
      world_by_candidate[world.candidate_index] = &world;
    }
  }
  for (auto& detection : prepared) {
    const TargetWorldObservation* world =
        world_by_candidate[detection.original_index];
    if (world != nullptr && std::isfinite(world->sigma_m) &&
        world->sigma_m > 0.0) {
      bool finite = true;
      for (double value : world->position)
        finite = finite && std::isfinite(value);
      if (!finite) continue;
      detection.world_valid = true;
      detection.world_position = world->position;
      detection.world_velocity = world->velocity;
      detection.world_velocity_valid = world->velocity_valid &&
          std::all_of(world->velocity.begin(), world->velocity.end(),
                      [](double value) { return std::isfinite(value); });
      detection.world_sigma_m = world->sigma_m;
      detection.world_stamp = world->capture_stamp;
    }
  }

  // Attach every optional physical label to at most one detection. The
  // timestamp/source validation is performed by the ROS node; this layer only
  // performs class and image-space consistency checks.
  std::set<std::size_t> used_hints;
  for (auto& detection : prepared) {
    std::size_t best_hint = identity_hints.size();
    double best_overlap = impl_->config.identity_hint_iou_gate;
    for (std::size_t i = 0; i < identity_hints.size(); ++i) {
      const auto& hint = identity_hints[i];
      if (used_hints.count(i) != 0 || hint.identity_label.empty() ||
          !std::isfinite(hint.confidence) ||
          hint.confidence < impl_->config.identity_hint_minimum_confidence ||
          (hint.class_id >= 0 && detection.message.class_id >= 0 &&
           hint.class_id != detection.message.class_id)) continue;
      const double overlap = hintIntersectionOverUnion(hint, detection.box);
      if (overlap >= best_overlap) {
        best_overlap = overlap;
        best_hint = i;
      }
    }
    if (best_hint < identity_hints.size()) {
      const auto& hint = identity_hints[best_hint];
      detection.identity_label = hint.identity_label;
      detection.identity_source_type = hint.source_type;
      detection.identity_confidence = clampValue(hint.confidence, 0.0, 1.0);
      detection.identity_stamp = hint.capture_stamp.isZero()
          ? detections.header.stamp : hint.capture_stamp;
      used_hints.insert(best_hint);
    }
  }

  const ros::Time frame_stamp = detections.header.stamp.isZero()
      ? ros::Time::now() : detections.header.stamp;
  impl_->pruneMemory(frame_stamp);
  for (auto& entry : impl_->tracks) {
    entry.second.filter.predict(impl_->dtFor(entry.second, frame_stamp));
    if (camera_motion.valid) {
      entry.second.filter.compensateCameraMotion(
          camera_motion.normalized_homography);
    }
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
  std::map<int, int> stable_detector_index;
  for (const auto& entry : impl_->tracks) {
    if (entry.second.detector_id_stable &&
        entry.second.detector_track_id >= 0) {
      stable_detector_index.emplace(entry.second.detector_track_id, entry.first);
    }
  }
  for (std::size_t index = 0; index < prepared.size(); ++index) {
    const auto& detection = prepared[index];
    if (!detection.message.track_id_is_stable || detection.message.track_id < 0) continue;
    const auto stable = stable_detector_index.find(detection.message.track_id);
    if (stable == stable_detector_index.end() ||
        used_tracks.count(stable->second) != 0) continue;
    const auto track_it = impl_->tracks.find(stable->second);
    if (track_it == impl_->tracks.end()) continue;
    const auto& track = track_it->second;
    if ((track.class_id >= 0 && detection.message.class_id >= 0 &&
         track.class_id != detection.message.class_id) ||
        impl_->hardLabelConflict(track.stable_identity_label, detection)) continue;
    assigned_ids[index] = stable->second;
    assigned_methods[index] = "detector_id";
    used_tracks.insert(stable->second);
  }

  // A globally optimal assignment avoids the order-dependent ID swaps caused
  // by the old greedy loop.  High-confidence detections establish identities;
  // low-confidence detections may only sustain an unmatched existing track.
  const auto associate = [&](const std::vector<std::size_t>& detection_indices) {
    std::vector<int> track_ids;
    std::vector<const PersistentTrack*> track_refs;
    for (const auto& entry : impl_->tracks) {
      if (used_tracks.count(entry.first) == 0 &&
          !entry.second.identity_ambiguous) {
        track_ids.push_back(entry.first);
        track_refs.push_back(&entry.second);
      }
    }
    if (detection_indices.empty() || track_ids.empty()) return;
    constexpr double kUnmatchedCost = 1.0;
    constexpr double kInvalidCost = 1e6;
    std::vector<std::vector<double>> costs(
        detection_indices.size(), std::vector<double>(track_ids.size(), kInvalidCost));
    std::vector<std::vector<AssociationKind>> methods(
        detection_indices.size(), std::vector<AssociationKind>(
            track_ids.size(), AssociationKind::kNone));
    std::vector<std::vector<std::uint8_t>> coarse_matches(
        detection_indices.size(), std::vector<std::uint8_t>(track_ids.size(), 0));
    std::vector<std::vector<double>> group_costs(
        detection_indices.size(), std::vector<double>(track_ids.size(), 1.0));
    for (std::size_t row = 0; row < detection_indices.size(); ++row) {
      const auto& detection = prepared[detection_indices[row]];
      for (std::size_t column = 0; column < track_ids.size(); ++column) {
        const auto& track = *track_refs[column];
        if (track.class_id >= 0 && detection.message.class_id >= 0 &&
            track.class_id != detection.message.class_id) continue;
        if (impl_->hardLabelConflict(track.stable_identity_label, detection)) continue;
        const NormalizedBox predicted = track.filter.box();
        const double overlap = intersectionOverUnion(predicted, detection.box);
        const double distance = normalizedCenterDistance(predicted, detection.box);
        const double mahalanobis = track.filter.mahalanobisDistance(
            detection.box, clampValue(detection.message.confidence, 0.0, 1.0));
        if ((overlap < impl_->config.association_iou_threshold &&
             distance > impl_->config.association_center_distance) ||
            mahalanobis > impl_->config.association_mahalanobis_gate) {
          continue;
        }
        // Metric observations are an additional same-camera innovation gate.
        // Cross-camera/world consistency is deliberately evaluated in the
        // controller's world-state filter, where vehicle pose and timestamps
        // are available; appearance alone is never permitted to merge sources.
        const auto& latest = track.latest_detection;
        if (impl_->config.metric_innovation_distance_m > 0.0 &&
            latest.range_valid && latest.has_relative_position_body &&
            detection.message.range_valid &&
            detection.message.has_relative_position_body) {
          double squared_distance = 0.0;
          bool valid_metric = true;
          for (std::size_t axis = 0; axis < 3; ++axis) {
            const double delta = static_cast<double>(
                latest.relative_position_body[axis]) - static_cast<double>(
                detection.message.relative_position_body[axis]);
            if (!std::isfinite(delta)) { valid_metric = false; break; }
            squared_distance += delta * delta;
          }
          if (!valid_metric || std::sqrt(squared_distance) >
              impl_->config.metric_innovation_distance_m) {
            continue;
          }
        }
        const double appearance = gallerySimilarity(
            track, detection.message.appearance_embedding,
            detection.appearance_norm);
        const bool has_appearance = appearance >= -1.0;
        const double appearance_quality = detection.appearance_quality;
        const bool reliable_appearance = has_appearance &&
            appearance_quality >= impl_->config.appearance_minimum_quality;
        const bool appearance_match = reliable_appearance &&
            appearance >= impl_->config.appearance_minimum_cosine;
        const bool exact_physical_label = !track.stable_identity_label.empty() &&
            track.stable_identity_label == detection.identity_label &&
            detection.identity_confidence >=
                impl_->config.identity_hint_hard_confidence;
        // Appearance is a gated component: it cannot alone link an object
        // across an arbitrary image displacement, but resolves close crossings.
        const double geometric = 0.60 * (1.0 - overlap) + 0.40 *
            std::min(1.0, distance / std::max(1e-6,
                impl_->config.association_center_distance));
        // Low-quality embeddings converge toward a neutral cost rather than
        // being allowed to overwrite motion during glare, blur or tiny ROIs.
        const double raw_appearance_term = has_appearance ? 1.0 - appearance : 0.5;
        const double appearance_reliability = has_appearance
            ? impl_->config.appearance_low_quality_weight +
                (1.0 - impl_->config.appearance_low_quality_weight) *
                    appearance_quality : 0.0;
        const double appearance_term = appearance_reliability *
            raw_appearance_term + (1.0 - appearance_reliability) * 0.5;
        const double innovation = std::min(1.0, mahalanobis /
            impl_->config.association_mahalanobis_gate);
        const double unseen_sec = !frame_stamp.isZero() &&
            !track.last_detection_stamp.isZero() &&
            frame_stamp >= track.last_detection_stamp
            ? (frame_stamp - track.last_detection_stamp).toSec() : 0.0;
        double world_cost = 0.5;
        const bool have_world_pair = track.world_valid && detection.world_valid;
        if (have_world_pair) {
          std::array<double, 3> predicted_world = track.world_position;
          const double world_dt = !detection.world_stamp.isZero() &&
              !track.world_stamp.isZero()
              ? std::max(0.0, (detection.world_stamp - track.world_stamp).toSec())
              : unseen_sec;
          if (track.world_velocity_valid) {
            for (std::size_t axis = 0; axis < 3; ++axis)
              predicted_world[axis] += track.world_velocity[axis] * world_dt;
          }
          double squared = 0.0;
          for (std::size_t axis = 0; axis < 3; ++axis) {
            const double error = detection.world_position[axis] -
                                 predicted_world[axis];
            squared += error * error;
          }
          const double sigma = std::sqrt(
              track.world_sigma_m * track.world_sigma_m +
              detection.world_sigma_m * detection.world_sigma_m +
              std::pow(impl_->config.world_process_noise_mps * world_dt, 2));
          const double normalized_world_error = std::sqrt(squared) /
              std::max(0.10, sigma);
          if (!std::isfinite(normalized_world_error) ||
              normalized_world_error >
                  impl_->config.world_innovation_gate_sigma) continue;
          world_cost = clampValue(normalized_world_error /
              impl_->config.world_innovation_gate_sigma, 0.0, 1.0);
        }
        if (unseen_sec > impl_->config.short_occlusion_sec && reliable_appearance &&
            !appearance_match && !exact_physical_label) continue;
        if (unseen_sec <= 0.10) {
          costs[row][column] = 0.65 * geometric + 0.20 * appearance_term +
                               0.15 * innovation;
        } else if (unseen_sec <= impl_->config.short_occlusion_sec) {
          costs[row][column] = 0.42 * geometric + 0.33 * appearance_term +
                               0.25 * innovation;
        } else {
          costs[row][column] = 0.25 * geometric + 0.50 * appearance_term +
                               0.25 * innovation;
        }
        if (have_world_pair) {
          const double world_weight = unseen_sec <= 0.10 ? 0.15 :
              (unseen_sec <= impl_->config.short_occlusion_sec ? 0.25 : 0.40);
          costs[row][column] = (1.0 - world_weight) * costs[row][column] +
                               world_weight * world_cost;
        }
        int feature_matches = 0;
        if (track.class_id < 0 || detection.message.class_id < 0 ||
            track.class_id == detection.message.class_id) ++feature_matches;
        const double shape_distance = aspectDistance(predicted, detection.box);
        if (shape_distance <= impl_->config.group_aspect_log_gate)
          ++feature_matches;
        if (appearance >= impl_->config.group_appearance_minimum_cosine)
          ++feature_matches;
        if (!track.image_source.empty() && track.image_source == image_source)
          ++feature_matches;
        if (!track.stable_identity_label.empty() &&
            track.stable_identity_label == detection.identity_label)
          ++feature_matches;
        coarse_matches[row][column] = feature_matches;
        double group_penalty = 0.5;
        const auto group_it = impl_->groups.find(track.group_id);
        if (group_it != impl_->groups.end()) {
          const bool compatible = impl_->groupCompatible(
              group_it->second, detection, image_source);
          if (unseen_sec > impl_->config.short_occlusion_sec && !compatible &&
              !exact_physical_label) {
            costs[row][column] = kInvalidCost;
            continue;
          }
          group_penalty = compatible ? 0.0 : 1.0;
        }
        const double shape_penalty = clampValue(shape_distance /
            impl_->config.group_aspect_log_gate, 0.0, 1.0);
        double label_penalty = 0.5;
        if (!track.stable_identity_label.empty() &&
            !detection.identity_label.empty()) {
          label_penalty = track.stable_identity_label == detection.identity_label
              ? 0.0 : 1.0;
        }
        group_costs[row][column] = 0.45 * group_penalty +
            0.25 * shape_penalty + 0.30 * label_penalty;
        methods[row][column] = appearance_match && overlap < 0.5
            ? AssociationKind::kAppearance : AssociationKind::kGlobalSpatial;
      }
    }
    // Ordinary ReID remains the primary path. Group-ReID is activated only
    // for ambiguous rows or when multiple coarse attributes agree with more
    // than one identity, which keeps the common case inexpensive.
    if (impl_->config.group_reid_enabled) {
      for (std::size_t row = 0; row < costs.size(); ++row) {
        double best = std::numeric_limits<double>::infinity();
        double second = std::numeric_limits<double>::infinity();
        std::size_t valid_count = 0;
        int coarse_candidates = 0;
        for (std::size_t column = 0; column < costs[row].size(); ++column) {
          const double value = costs[row][column];
          if (value < kInvalidCost) {
            ++valid_count;
            if (value < best) {
              second = best;
              best = value;
            } else if (value < second) {
              second = value;
            }
          }
          if (coarse_matches[row][column] >=
              impl_->config.group_trigger_minimum_features) ++coarse_candidates;
        }
        const bool score_ambiguous = valid_count > 1 &&
            second - best <= impl_->config.group_trigger_score_margin;
        const bool group_triggered = score_ambiguous || coarse_candidates > 1;
        if (!group_triggered) continue;
        for (std::size_t column = 0; column < costs[row].size(); ++column) {
          if (costs[row][column] >= kInvalidCost) continue;
          costs[row][column] = 0.70 * costs[row][column] +
                               0.30 * group_costs[row][column];
          methods[row][column] = AssociationKind::kGroup;
        }
      }
    }
    const auto assignment = sparseHungarianAssignment(
        costs, kUnmatchedCost, kInvalidCost);
    for (std::size_t row = 0; row < assignment.size(); ++row) {
      const int column = assignment[row];
      if (column < 0 || costs[row][column] >= kUnmatchedCost) continue;
      const auto& track = impl_->tracks.at(
          track_ids[static_cast<std::size_t>(column)]);
      const double unseen_sec = !frame_stamp.isZero() &&
          !track.last_detection_stamp.isZero() && frame_stamp >= track.last_detection_stamp
          ? (frame_stamp - track.last_detection_stamp).toSec() : 0.0;
      if (unseen_sec > impl_->config.short_occlusion_sec) {
        double best = std::numeric_limits<double>::infinity();
        double second = std::numeric_limits<double>::infinity();
        std::size_t alternative_count = 0;
        for (double value : costs[row]) {
          if (value >= kUnmatchedCost) continue;
          ++alternative_count;
          if (value < best) {
            second = best;
            best = value;
          } else if (value < second) {
            second = value;
          }
        }
        if (alternative_count > 0 && costs[row][column] > best + 1e-9) {
          continue;
        }
        if (alternative_count > 1 && second - best <
                impl_->config.long_association_minimum_margin) {
          continue;
        }
      }
      const std::size_t detection_index = detection_indices[row];
      const int track_id = track_ids[static_cast<std::size_t>(column)];
      assigned_ids[detection_index] = track_id;
      assigned_methods[detection_index] = associationName(methods[row][column]);
      used_tracks.insert(track_id);
    }
  };
  std::vector<std::size_t> high_confidence;
  std::vector<std::size_t> low_confidence;
  for (std::size_t index = 0; index < prepared.size(); ++index) {
    if (assigned_ids[index] >= 0) continue;
    if (prepared[index].message.confidence >= impl_->config.high_confidence_threshold) {
      high_confidence.push_back(index);
    } else if (prepared[index].message.confidence >= impl_->config.low_confidence_threshold) {
      low_confidence.push_back(index);
    }
  }
  associate(high_confidence);
  associate(low_confidence);

  // Tracks that have left the real-time pool can still reclaim their public
  // ID from the bounded identity memory. This pass is intentionally stricter
  // than active association: appearance/physical identity needs an absolute
  // threshold and the winner must be separated from the runner-up.
  if (impl_->config.long_term_memory_enabled && !impl_->identity_memory.empty()) {
    std::vector<std::size_t> unmatched_high;
    for (const std::size_t index : high_confidence) {
      if (assigned_ids[index] < 0) unmatched_high.push_back(index);
    }
    std::vector<int> memory_ids;
    std::vector<const MemoryIdentity*> memory_refs;
    for (const auto& entry : impl_->identity_memory) {
      if (impl_->tracks.count(entry.first) == 0) {
        memory_ids.push_back(entry.first);
        memory_refs.push_back(&entry.second);
      }
    }
    constexpr double kInvalidMemoryCost = 1e6;
    const double unmatched_memory_cost = std::min(
        0.99, impl_->config.long_term_memory_maximum_cost + 1e-3);
    std::vector<std::vector<double>> memory_costs(
        unmatched_high.size(),
        std::vector<double>(memory_ids.size(), kInvalidMemoryCost));
    std::vector<std::vector<AssociationKind>> memory_methods(
        unmatched_high.size(), std::vector<AssociationKind>(
            memory_ids.size(), AssociationKind::kNone));
    for (std::size_t row = 0; row < unmatched_high.size(); ++row) {
      const auto& detection = prepared[unmatched_high[row]];
      for (std::size_t column = 0; column < memory_ids.size(); ++column) {
        const MemoryIdentity& memory = *memory_refs[column];
        if (memory.class_id >= 0 && detection.message.class_id >= 0 &&
            memory.class_id != detection.message.class_id) continue;
        if (impl_->hardLabelConflict(memory.stable_identity_label, detection)) continue;
        const bool label_matches = !memory.stable_identity_label.empty() &&
            memory.stable_identity_label == detection.identity_label &&
            detection.identity_confidence >=
                impl_->config.identity_hint_minimum_confidence;
        const bool exact_label = label_matches &&
            detection.identity_confidence >=
                impl_->config.identity_hint_hard_confidence;
        const double appearance = gallerySimilarity(
            memory, detection.message.appearance_embedding,
            detection.appearance_norm);
        if (!exact_label &&
            appearance < impl_->config.long_term_memory_minimum_cosine) continue;
        double group_penalty = 0.5;
        bool group_compatible = true;
        const auto group_it = impl_->groups.find(memory.group_id);
        if (group_it != impl_->groups.end()) {
          group_compatible = impl_->groupCompatible(
              group_it->second, detection, image_source);
          group_penalty = group_compatible ? 0.0 : 1.0;
        }
        if (!group_compatible && !exact_label) continue;
        const double shape = clampValue(aspectDistance(memory.last_box, detection.box) /
            impl_->config.group_aspect_log_gate, 0.0, 1.0);
        double metric = 0.5;
        const auto& old = memory.latest_detection;
        // Body-relative locations are comparable only for a fresh same-camera
        // memory. Longer/cross-camera matching is left to the node's inertial
        // world-state gate; stale body coordinates are never treated as world.
        const double memory_age = !frame_stamp.isZero() &&
            !memory.last_detection_stamp.isZero() &&
            frame_stamp >= memory.last_detection_stamp
            ? (frame_stamp - memory.last_detection_stamp).toSec() : 0.0;
        if (memory.world_valid && detection.world_valid) {
          std::array<double, 3> predicted = memory.world_position;
          const double world_dt = !detection.world_stamp.isZero() &&
              !memory.world_stamp.isZero()
              ? std::max(0.0,
                  (detection.world_stamp - memory.world_stamp).toSec())
              : memory_age;
          if (memory.world_velocity_valid) {
            for (std::size_t axis = 0; axis < 3; ++axis)
              predicted[axis] += memory.world_velocity[axis] * world_dt;
          }
          double squared = 0.0;
          for (std::size_t axis = 0; axis < 3; ++axis) {
            const double error = detection.world_position[axis] - predicted[axis];
            squared += error * error;
          }
          const double sigma = std::sqrt(
              memory.world_sigma_m * memory.world_sigma_m +
              detection.world_sigma_m * detection.world_sigma_m +
              std::pow(impl_->config.world_process_noise_mps * world_dt, 2));
          const double normalized = std::sqrt(squared) / std::max(0.10, sigma);
          if (!std::isfinite(normalized) || normalized >
              impl_->config.world_innovation_gate_sigma) continue;
          metric = clampValue(normalized /
              impl_->config.world_innovation_gate_sigma, 0.0, 1.0);
        } else if (memory.image_source == image_source &&
            memory_age <= impl_->config.short_occlusion_sec && old.range_valid &&
            old.has_relative_position_body && detection.message.range_valid &&
            detection.message.has_relative_position_body) {
          double squared = 0.0;
          bool finite = true;
          for (std::size_t axis = 0; axis < 3; ++axis) {
            const double delta = old.relative_position_body[axis] -
                                 detection.message.relative_position_body[axis];
            if (!std::isfinite(delta)) { finite = false; break; }
            squared += delta * delta;
          }
          if (!finite) continue;
          const double distance = std::sqrt(squared);
          if (impl_->config.metric_innovation_distance_m > 0.0 &&
              distance > impl_->config.metric_innovation_distance_m) continue;
          metric = clampValue(distance /
              std::max(1e-6, impl_->config.metric_innovation_distance_m), 0.0, 1.0);
        }
        const double appearance_cost = appearance >= -1.0
            ? 1.0 - appearance : 0.5;
        const double label_cost = label_matches ? 0.0 :
            (!memory.stable_identity_label.empty() &&
             !detection.identity_label.empty() ? 1.0 : 0.5);
        // Ordinary appearance is evaluated in parallel with coarse grouping.
        // Exact physical labels dominate but still require multi-frame control
        // reconfirmation after resurrection.
        memory_costs[row][column] = exact_label
            ? 0.10 * appearance_cost + 0.10 * shape + 0.10 * metric +
                  0.10 * group_penalty + 0.60 * label_cost
            : 0.48 * appearance_cost + 0.17 * shape + 0.20 * metric +
                  0.15 * group_penalty;
        memory_methods[row][column] = exact_label
            ? AssociationKind::kNumber : (group_penalty < 0.5
                ? AssociationKind::kGroupMemory : AssociationKind::kLongTerm);
      }
    }
    const auto memory_assignment = sparseHungarianAssignment(
        memory_costs, unmatched_memory_cost, kInvalidMemoryCost);
    std::set<int> used_memory;
    for (std::size_t row = 0; row < memory_assignment.size(); ++row) {
      const int column = memory_assignment[row];
      if (column < 0) continue;
      const double assigned_cost = memory_costs[row][column];
      if (assigned_cost > impl_->config.long_term_memory_maximum_cost) continue;
      double best = std::numeric_limits<double>::infinity();
      double second = std::numeric_limits<double>::infinity();
      std::size_t valid_count = 0;
      for (double value : memory_costs[row]) {
        if (value > impl_->config.long_term_memory_maximum_cost) continue;
        ++valid_count;
        if (value < best) {
          second = best;
          best = value;
        } else if (value < second) {
          second = value;
        }
      }
      if (valid_count > 1 && second - best <
          impl_->config.long_term_memory_minimum_margin) {
        prepared[unmatched_high[row]].memory_ambiguous = true;
        continue;
      }
      const int memory_id = memory_ids[static_cast<std::size_t>(column)];
      if (used_memory.count(memory_id) != 0) continue;
      const auto memory_it = impl_->identity_memory.find(memory_id);
      if (memory_it == impl_->identity_memory.end()) continue;
      MemoryIdentity memory = std::move(memory_it->second);
      const std::size_t detection_index = unmatched_high[row];
      PersistentTrack restored;
      restored.public_id = memory.public_id;
      restored.class_id = memory.class_id;
      restored.confidence = memory.confidence;
      restored.filter.initialize(prepared[detection_index].box, impl_->config);
      restored.latest_detection = prepared[detection_index].message;
      restored.latest_header = detections.header;
      restored.appearance_gallery = std::move(memory.appearance_gallery);
      restored.appearance = prepared[detection_index].message.appearance_embedding;
      restored.group_id = memory.group_id;
      restored.image_source = image_source;
      restored.stable_identity_label = std::move(memory.stable_identity_label);
      restored.world_valid = memory.world_valid;
      restored.world_position = memory.world_position;
      restored.world_velocity = memory.world_velocity;
      restored.world_velocity_valid = memory.world_velocity_valid;
      restored.world_sigma_m = memory.world_sigma_m;
      restored.world_stamp = memory.world_stamp;
      restored.hits = memory.hits;
      restored.reacquisition_hits_remaining =
          impl_->config.long_term_reconfirmation_hits - 1;
      restored.last_stamp = frame_stamp;
      restored.last_detection_stamp = frame_stamp;
      restored.association = associationName(memory_methods[row][column]);
      // Remove a local placeholder created while the memory candidates were
      // tied. It never owned a confirmed identity and must not survive next
      // to the recovered public track.
      for (auto pending = impl_->tracks.begin(); pending != impl_->tracks.end();) {
        if (pending->second.identity_ambiguous &&
            (pending->second.class_id < 0 || restored.class_id < 0 ||
             pending->second.class_id == restored.class_id) &&
            (intersectionOverUnion(pending->second.filter.box(),
                                   prepared[detection_index].box) >=
                 impl_->config.association_iou_threshold ||
             normalizedCenterDistance(pending->second.filter.box(),
                                      prepared[detection_index].box) <=
                 impl_->config.association_center_distance)) {
          pending = impl_->tracks.erase(pending);
        } else {
          ++pending;
        }
      }
      impl_->tracks.emplace(restored.public_id, std::move(restored));
      impl_->identity_memory.erase(memory_it);
      assigned_ids[detection_index] = memory_id;
      assigned_methods[detection_index] = associationName(
          memory_methods[row][column]);
      used_tracks.insert(memory_id);
      used_memory.insert(memory_id);
    }
  }

  // If memory candidates are still tied, preserve one local tentative
  // tracklet for image continuity. It is deliberately excluded from ordinary
  // association above, so every new frame gets another chance to resolve the
  // old identities instead of accidentally confirming the placeholder.
  {
    std::vector<std::size_t> ambiguous_detections;
    std::vector<int> ambiguous_tracks;
    for (std::size_t i = 0; i < prepared.size(); ++i) {
      if (assigned_ids[i] < 0 && prepared[i].memory_ambiguous)
        ambiguous_detections.push_back(i);
    }
    for (const auto& entry : impl_->tracks) {
      if (entry.second.identity_ambiguous) ambiguous_tracks.push_back(entry.first);
    }
    if (!ambiguous_detections.empty() && !ambiguous_tracks.empty()) {
      constexpr double kInvalid = 1e6;
      constexpr double kUnmatched = 1.0;
      std::vector<std::vector<double>> costs(
          ambiguous_detections.size(),
          std::vector<double>(ambiguous_tracks.size(), kInvalid));
      for (std::size_t row = 0; row < ambiguous_detections.size(); ++row) {
        const auto& detection = prepared[ambiguous_detections[row]];
        for (std::size_t column = 0; column < ambiguous_tracks.size(); ++column) {
          const auto& track = impl_->tracks.at(ambiguous_tracks[column]);
          if (track.class_id >= 0 && detection.message.class_id >= 0 &&
              track.class_id != detection.message.class_id) continue;
          const NormalizedBox predicted = track.filter.box();
          const double overlap = intersectionOverUnion(predicted, detection.box);
          const double distance = normalizedCenterDistance(predicted, detection.box);
          if (overlap < impl_->config.association_iou_threshold &&
              distance > impl_->config.association_center_distance) continue;
          costs[row][column] = 0.65 * (1.0 - overlap) + 0.35 * clampValue(
              distance / std::max(1e-6,
                  impl_->config.association_center_distance), 0.0, 1.0);
        }
      }
      const auto assignment = sparseHungarianAssignment(
          costs, kUnmatched, kInvalid);
      for (std::size_t row = 0; row < assignment.size(); ++row) {
        if (assignment[row] < 0 || costs[row][assignment[row]] >= kUnmatched) continue;
        const std::size_t detection_index = ambiguous_detections[row];
        const int track_id = ambiguous_tracks[assignment[row]];
        assigned_ids[detection_index] = track_id;
        assigned_methods[detection_index] = "identity_ambiguous";
        used_tracks.insert(track_id);
      }
    }
  }

  // Create bounded new tracks for the unmatched detections.
  for (std::size_t index = 0; index < prepared.size(); ++index) {
    if (assigned_ids[index] >= 0 ||
        prepared[index].message.confidence <
            std::max(impl_->config.minimum_new_track_confidence,
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
    track.confidence = clampValue(prepared[index].message.confidence, 0.0, 1.0);
    track.filter.initialize(prepared[index].box, impl_->config);
    track.last_observation_box = prepared[index].box;
    track.latest_detection = prepared[index].message;
    track.latest_header = detections.header;
    track.appearance = prepared[index].message.appearance_embedding;
    impl_->updateAppearance(&track, track.appearance);
    track.group_id = impl_->findOrCreateGroup(
        prepared[index], image_source, frame_stamp);
    track.image_source = image_source;
    track.identity_ambiguous = prepared[index].memory_ambiguous;
    impl_->updateWorld(&track, prepared[index]);
    track.last_stamp = frame_stamp;
    track.last_detection_stamp = frame_stamp;
    const int public_id = track.public_id;
    const bool identity_ambiguous = track.identity_ambiguous;
    impl_->tracks.emplace(public_id, std::move(track));
    assigned_ids[index] = public_id;
    assigned_methods[index] = identity_ambiguous
        ? "identity_ambiguous" : "new";
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
    const bool freshly_restored = assigned_methods[index] == "long_term_reid" ||
        assigned_methods[index] == "group_memory_reid" ||
        assigned_methods[index] == "number_reid";
    const bool freshly_created = assigned_methods[index] == "new" ||
        (assigned_methods[index] == "identity_ambiguous" && track.age == 1);
    if (!freshly_created && !freshly_restored) {
      track.filter.update(prepared[index].box,
                          clampValue(prepared[index].message.confidence, 0.0, 1.0));
      if (impl_->config.observation_centric_reupdate_enabled &&
          !frame_stamp.isZero() && !track.last_detection_stamp.isZero() &&
          frame_stamp > track.last_detection_stamp) {
        track.filter.observationCentricReupdate(track.last_observation_box,
            prepared[index].box, (frame_stamp - track.last_detection_stamp).toSec(),
            impl_->config.observation_centric_velocity_blend);
      }
      ++track.hits;
    }
    bool reacquisition_confirmed = false;
    if (!freshly_restored && track.reacquisition_hits_remaining > 0) {
      --track.reacquisition_hits_remaining;
      reacquisition_confirmed = track.reacquisition_hits_remaining == 0;
    }
    track.missing = 0;
    track.detected = true;
    track.reidentified = assigned_methods[index] == "appearance" ||
                         reacquisition_confirmed;
    track.association = track.reacquisition_hits_remaining > 0 &&
        !freshly_restored ? "reid_confirming" : assigned_methods[index];
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
    if (!prepared[index].message.appearance_embedding.empty() &&
        prepared[index].message.confidence >=
            impl_->config.appearance_update_minimum_confidence &&
        prepared[index].appearance_quality >=
            impl_->config.appearance_minimum_quality) {
      impl_->updateAppearance(&track,
                              prepared[index].message.appearance_embedding);
    }
    impl_->updateIdentityLabel(&track, prepared[index]);
    impl_->updateWorld(&track, prepared[index]);
    impl_->updateGroup(track, prepared[index], frame_stamp);
    track.latest_detection = prepared[index].message;
    track.latest_detection.track_id = assigned_id;
    track.latest_detection.track_id_is_stable = true;
    track.latest_header = detections.header;
    track.last_observation_box = prepared[index].box;
    track.last_stamp = frame_stamp;
    track.last_detection_stamp = frame_stamp;
    output.candidates.candidates.push_back(track.latest_detection);
    ++impl_->stats.accepted_detections;
  }

  for (auto iterator = impl_->tracks.begin(); iterator != impl_->tracks.end();) {
    const auto& track = iterator->second;
    const double unseen_sec = (!frame_stamp.isZero() && !track.last_detection_stamp.isZero() &&
        frame_stamp >= track.last_detection_stamp)
        ? (frame_stamp - track.last_detection_stamp).toSec() : 0.0;
    const bool timed_out = impl_->config.removal_timeout_sec >= 0.0
        ? unseen_sec > impl_->config.removal_timeout_sec
        : track.missing > static_cast<std::uint32_t>(impl_->config.removal_frames);
    if (timed_out) {
      impl_->remember(&iterator->second);
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
    const double unseen_sec = (!frame_stamp.isZero() && !track.last_detection_stamp.isZero() &&
        frame_stamp >= track.last_detection_stamp)
        ? (frame_stamp - track.last_detection_stamp).toSec() : 0.0;
    const bool occluded = impl_->config.occlusion_timeout_sec >= 0.0
        ? unseen_sec <= impl_->config.occlusion_timeout_sec
        : track.missing <= static_cast<std::uint32_t>(impl_->config.occlusion_frames);
    if (track.identity_ambiguous || track.reacquisition_hits_remaining > 0 ||
        track.hits < static_cast<std::uint32_t>(impl_->config.confirmation_hits)) {
      state.lifecycle_state = "tentative";
    } else if (track.missing == 0) {
      state.lifecycle_state = "confirmed";
    } else if (occluded) {
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

bool MultiTrackManager::identityLabel(const int track_id,
                                      std::string* label) const {
  if (label == nullptr) return false;
  const auto active = impl_->tracks.find(track_id);
  if (active != impl_->tracks.end() &&
      !active->second.stable_identity_label.empty()) {
    *label = active->second.stable_identity_label;
    return true;
  }
  const auto memory = impl_->identity_memory.find(track_id);
  if (memory != impl_->identity_memory.end() &&
      !memory->second.stable_identity_label.empty()) {
    *label = memory->second.stable_identity_label;
    return true;
  }
  return false;
}

bool MultiTrackManager::adoptPublicId(const int local_track_id,
                                      const int global_public_id) {
  if (global_public_id < 0) return false;
  auto source = impl_->tracks.find(local_track_id);
  if (source == impl_->tracks.end()) return false;
  if (local_track_id == global_public_id) return true;
  if (impl_->tracks.count(global_public_id) != 0) return false;
  impl_->identity_memory.erase(global_public_id);
  PersistentTrack adopted = std::move(source->second);
  impl_->tracks.erase(source);
  adopted.public_id = global_public_id;
  adopted.latest_detection.track_id = global_public_id;
  adopted.latest_detection.track_id_is_stable = true;
  impl_->tracks.emplace(global_public_id, std::move(adopted));
  if (impl_->selected_track_id == local_track_id) {
    impl_->selected_track_id = global_public_id;
  }
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
  impl_->identity_memory.clear();
  impl_->groups.clear();
  impl_->selected_track_id = -1;
  impl_->stats.active_tracks = 0;
}

MultiTrackStatistics MultiTrackManager::statistics() const {
  MultiTrackStatistics result = impl_->stats;
  result.active_tracks = impl_->tracks.size();
  return result;
}

}  // namespace xd_uav_track
