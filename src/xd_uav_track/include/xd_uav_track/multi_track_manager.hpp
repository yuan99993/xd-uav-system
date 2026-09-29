#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <std_msgs/Header.h>
#include <xd_uav_track/DetectionArray.h>
#include <xd_uav_track/DetectionCandidate.h>
#include <xd_uav_track/TrackStateArray.h>

namespace xd_uav_track {

struct MultiTrackConfig {
  int confirmation_hits{2};
  int occlusion_frames{5};
  int removal_frames{30};
  double reference_fps{30.0};
  double occlusion_timeout_sec{0.0};
  double removal_timeout_sec{0.0};
  double reacquisition_after_sec{0.0};
  int reacquisition_confirm_hits{3};
  std::string reacquisition_mode{"balanced"};
  double maximum_size_change_ratio{2.5};
  double exit_edge_margin{0.15};
  double search_expansion_rate_per_sec{6.0};
  double search_expansion_cap{2.0};
  double lenient_iou_scale{0.4};
  double lenient_iou_floor{0.10};
  double appearance_distance_gate_factor{3.0};
  int maximum_tracks{128};
  int maximum_embedding_dimension{2048};
  double minimum_new_track_confidence{0.20};
  double minimum_update_confidence{0.05};
  double association_iou_threshold{0.20};
  double association_center_distance{1.50};
  double association_ambiguity_margin{0.08};
  // Only enable for a source whose body-relative metric origin is stationary
  // (or externally motion-compensated). Moving-aircraft configurations keep
  // this off and use the world-state identity layer instead.
  bool metric_relative_association_enabled{false};
  double metric_relative_max_distance_m{10.0};
  double metric_relative_score_weight{1.2};
  double metric_relative_smoothing_alpha{0.25};
  double metric_relative_max_speed_mps{8.0};
  double metric_relative_jump_tolerance_m{2.0};
  bool flexible_class_matching{true};
  // Class IDs in this configured family may represent the same object across
  // viewpoint-dependent detector label flips. Geometry/ReID gates still apply.
  std::vector<int> compatible_class_ids;
  int class_history_size{10};
  double appearance_minimum_cosine{0.78};
  double high_confidence_threshold{0.50};
  int appearance_gallery_size{5};
  double confidence_smoothing_alpha{0.80};
  double confidence_decay_rate{0.05};
  double process_noise{1.0};
  double measurement_noise{1.0};
};

struct MultiTrackStatistics {
  std::uint64_t input_frames{0};
  std::uint64_t accepted_detections{0};
  std::uint64_t rejected_detections{0};
  std::uint64_t created_tracks{0};
  std::uint64_t removed_tracks{0};
  std::size_t active_tracks{0};
};

struct ManagedDetectionFrame {
  // False means the input timestamp was duplicate or out of order and the
  // manager state was left untouched.
  bool accepted{true};
  xd_uav_track::DetectionArray candidates;
  xd_uav_track::TrackStateArray tracks;
};

// Maintains an independent constant-velocity Kalman state and lifecycle for
// every visible target. The class has no publishers and keeps the tracking
// policy isolated so it can be reused by tests/nodelets or adapted behind a
// future transport wrapper.
class MultiTrackManager {
 public:
  explicit MultiTrackManager(const MultiTrackConfig& config = MultiTrackConfig());
  ~MultiTrackManager();
  MultiTrackManager(MultiTrackManager&&) noexcept;
  MultiTrackManager& operator=(MultiTrackManager&&) noexcept;

  MultiTrackManager(const MultiTrackManager&) = delete;
  MultiTrackManager& operator=(const MultiTrackManager&) = delete;

  ManagedDetectionFrame update(const xd_uav_track::DetectionArray& detections,
                               int image_width, int image_height);
  // Source-aware overload keeps independent ROS image streams observable
  // without changing the legacy three-argument API.
  ManagedDetectionFrame update(const xd_uav_track::DetectionArray& detections,
                               int image_width, int image_height,
                               const std::string& image_source);
  bool latestCandidate(int track_id, xd_uav_track::DetectionCandidate* candidate,
                       std_msgs::Header* header = nullptr) const;
  void setSelectedTrackId(int track_id);
  int selectedTrackId() const;
  void reset();
  MultiTrackStatistics statistics() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace xd_uav_track
