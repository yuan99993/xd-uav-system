#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <std_msgs/Header.h>
#include <ros/time.h>
#include <xd_uav_track/DetectionArray.h>
#include <xd_uav_track/DetectionCandidate.h>
#include <xd_uav_track/TrackStateArray.h>

namespace xd_uav_track {

struct MultiTrackConfig {
  int confirmation_hits{2};
  int occlusion_frames{5};
  int removal_frames{30};
  int maximum_tracks{128};
  int maximum_embedding_dimension{2048};
  double minimum_new_track_confidence{0.20};
  double minimum_update_confidence{0.05};
  double association_iou_threshold{0.20};
  double association_center_distance{1.50};
  double appearance_minimum_cosine{0.78};
  // ByteTrack-style association separates reliable detections from weak
  // detections. Weak detections may sustain an existing track but never
  // create a new public identity.
  double high_confidence_threshold{0.50};
  double low_confidence_threshold{0.10};
  int appearance_gallery_size{12};
  double appearance_update_minimum_confidence{0.60};
  double appearance_minimum_quality{0.35};
  // Quality is used as a soft association weight, never as a reason to drop
  // a geometrically valid detection.
  double appearance_low_quality_weight{0.20};
  // Image-space innovation is evaluated against the Kalman covariance.  It
  // makes large jumps fail even when their boxes happen to overlap.
  double association_mahalanobis_gate{16.0};
  double metric_innovation_distance_m{25.0};
  // Non-negative values make the lifecycle independent of camera FPS.  A
  // negative value retains the legacy frame-count behaviour.
  double occlusion_timeout_sec{-1.0};
  double removal_timeout_sec{-1.0};
  double process_noise{1.0};
  double measurement_noise{1.0};
  // OC-SORT observation-centric re-update is an opt-in A/B path. CV Kalman
  // remains the default for existing deployments.
  bool observation_centric_reupdate_enabled{false};
  double observation_centric_velocity_blend{0.50};

  // Group-ReID first narrows candidates by stable coarse attributes. The
  // group identifier is deliberately internal: public track IDs remain the
  // only identity exposed by TrackState/DetectionCandidate.
  bool group_reid_enabled{true};
  double group_appearance_minimum_cosine{0.55};
  double group_aspect_log_gate{0.55};
  bool group_source_strict{false};
  int maximum_groups{64};
  int group_trigger_minimum_features{2};
  double group_trigger_score_margin{0.12};

  // Active association changes its evidence balance as an observation ages.
  double short_occlusion_sec{1.0};
  double long_association_minimum_margin{0.08};

  // A long-term identity memory is separate from the active Kalman tracks.
  bool long_term_memory_enabled{true};
  double long_term_memory_ttl_sec{60.0};
  int long_term_memory_maximum_identities{256};
  double long_term_memory_minimum_cosine{0.84};
  double long_term_memory_minimum_margin{0.08};
  double long_term_memory_maximum_cost{0.55};
  int long_term_reconfirmation_hits{3};

  // Optional OCR/QR/AprilTag/operator labels. A stable label requires more
  // than one observation, so a single noisy recognition cannot overwrite an
  // identity. A high-confidence conflict is a hard association rejection.
  double identity_hint_minimum_confidence{0.55};
  double identity_hint_hard_confidence{0.90};
  double identity_hint_iou_gate{0.30};
  int identity_hint_confirmations{2};
  double world_innovation_gate_sigma{5.0};
  double world_process_noise_mps{2.0};
};

// Transport-neutral identity evidence attached to one detection frame. The
// node converts optional IdentityHintArray messages into this representation.
struct TargetIdentityHint {
  int class_id{-1};
  std::array<double, 4> normalized_bbox{{0.0, 0.0, 0.0, 0.0}};
  std::string identity_label;
  std::string source_type;
  double confidence{0.0};
  ros::Time capture_stamp;
};

struct TargetWorldObservation {
  std::size_t candidate_index{0};
  std::array<double, 3> position{{0.0, 0.0, 0.0}};
  std::array<double, 3> velocity{{0.0, 0.0, 0.0}};
  bool velocity_valid{false};
  double sigma_m{1.0};
  ros::Time capture_stamp;
};

// Rotation-only image homography obtained from synchronized UAV pose and
// calibrated camera intrinsics/extrinsics. It maps the previous normalized
// image plane into the current one. Translation is intentionally excluded:
// without a depth estimate it would fabricate parallax.
struct CameraMotionCompensation {
  bool valid{false};
  std::array<double, 9> normalized_homography{{1.0, 0.0, 0.0,
                                                 0.0, 1.0, 0.0,
                                                 0.0, 0.0, 1.0}};
  ros::Time previous_stamp;
  ros::Time current_stamp;
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
  ManagedDetectionFrame update(
      const xd_uav_track::DetectionArray& detections, int image_width,
      int image_height, const std::string& image_source,
      const std::vector<TargetIdentityHint>& identity_hints);
  ManagedDetectionFrame update(
      const xd_uav_track::DetectionArray& detections, int image_width,
      int image_height, const std::string& image_source,
      const std::vector<TargetIdentityHint>& identity_hints,
      const std::vector<TargetWorldObservation>& world_observations);
  ManagedDetectionFrame update(
      const xd_uav_track::DetectionArray& detections, int image_width,
      int image_height, const std::string& image_source,
      const std::vector<TargetIdentityHint>& identity_hints,
      const std::vector<TargetWorldObservation>& world_observations,
      const CameraMotionCompensation& camera_motion);
  bool latestCandidate(int track_id, xd_uav_track::DetectionCandidate* candidate,
                       std_msgs::Header* header = nullptr) const;
  // Returns a repeatedly-confirmed physical label from either the active
  // track or long-term memory. Unconfirmed OCR/QR samples are never exposed.
  bool identityLabel(int track_id, std::string* label) const;
  // Promote a source-local tracklet to a verified global public identity.
  // The caller must perform cross-source time/world/class gating first.
  bool adoptPublicId(int local_track_id, int global_public_id);
  void setSelectedTrackId(int track_id);
  int selectedTrackId() const;
  void reset();
  MultiTrackStatistics statistics() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace xd_uav_track
