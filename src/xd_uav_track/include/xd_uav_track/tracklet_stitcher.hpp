#pragma once

#include <array>
#include <string>
#include <vector>

namespace xd_uav_track {

struct TrackletIdentitySnapshot {
  int identity_id{-1};
  int class_id{-1};
  std::string source;
  double last_seen_sec{0.0};
  bool established{false};
  bool active_in_source{false};
  bool locked_target{false};
  bool has_world_position{false};
  bool has_world_velocity{false};
  std::array<double, 3> world_position{{0.0, 0.0, 0.0}};
  std::array<double, 3> position_variance{{1.0, 1.0, 1.0}};
  std::array<double, 3> world_velocity{{0.0, 0.0, 0.0}};
  std::vector<std::vector<float>> appearance_prototypes;
};

struct TrackletObservation {
  int class_id{-1};
  std::string source;
  double stamp_sec{0.0};
  bool confirmed{false};
  bool has_world_position{false};
  bool has_world_velocity{false};
  std::array<double, 3> world_position{{0.0, 0.0, 0.0}};
  std::array<double, 3> position_variance{{1.0, 1.0, 1.0}};
  std::array<double, 3> world_velocity{{0.0, 0.0, 0.0}};
  std::vector<float> appearance;
};

struct TrackletStitchConfig {
  bool enabled{true};
  // Cross-camera identity association is deliberately separate from normal
  // same-camera tracklet recovery. It is only safe with synchronized metric
  // world positions and bounded measurement uncertainty.
  bool cross_source_metric_enabled{false};
  double cross_source_max_time_delta_sec{0.35};
  double cross_source_max_distance_m{4.0};
  double cross_source_max_position_variance_m2{9.0};
  double archive_ttl_sec{30.0};
  // A manually locked identity can outlive the ordinary tracklet archive.
  // Beyond archive_ttl_sec, only a strong, unique appearance match may
  // restore its ID; stale linear motion is not trusted after a long gap.
  double locked_identity_archive_ttl_sec{600.0};
  double locked_identity_min_appearance_cosine{0.90};
  double maximum_distance_m{25.0};
  double maximum_speed_mps{25.0};
  double maximum_velocity_delta_mps{8.0};
  double mahalanobis_gate_sq{11.34};
  double process_noise_m2_per_s2{2.0};
  double minimum_position_variance_m2{0.25};
  double minimum_appearance_cosine{0.78};
  // A changed viewpoint may make a valid ReID descriptor disagree after a
  // turn. Permit motion-only recovery for a short gap, with tighter metric
  // and velocity gates than the ordinary appearance-supported path.
  double motion_only_max_gap_sec{15.0};
  double motion_only_max_residual_m{10.0};
  double motion_only_max_velocity_delta_mps{6.0};
  double minimum_score{0.45};
  double minimum_score_margin{0.15};
  // Optional same-source class family for viewpoint-dependent label changes.
  std::vector<int> compatible_class_ids;
};

struct TrackletStitchMatch {
  int identity_id{-1};
  bool ambiguous{false};
  double score{0.0};
  double margin{0.0};
  std::string reason{"no_candidate"};
};

struct TrackletStitchConfirmationState {
  int identity_id{-1};
  int consecutive_hits{0};
  double last_stamp_sec{0.0};
  double last_score{0.0};
};

// Selects a previously established identity for a new confirmed tracklet.
// Same-source candidates must be archived. Cross-source candidates may be
// active in the other camera, but require close-in-time, low-uncertainty
// metric world geometry; appearance is optional supporting evidence.
TrackletStitchMatch findTrackletStitchMatch(
    const TrackletObservation& observation,
    const std::vector<TrackletIdentitySnapshot>& identities,
    const TrackletStitchConfig& config);

TrackletStitchConfirmationState advanceTrackletStitchConfirmation(
    const TrackletStitchConfirmationState& previous,
    const TrackletStitchMatch& match, double stamp_sec,
    double maximum_gap_sec);

}  // namespace xd_uav_track
