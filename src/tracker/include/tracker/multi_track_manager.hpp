#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>

#include <std_msgs/Header.h>
#include <tracker/DetectionArray.h>
#include <tracker/DetectionCandidate.h>
#include <tracker/TrackStateArray.h>

namespace tracker {

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
  tracker::DetectionArray candidates;
  tracker::TrackStateArray tracks;
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

  ManagedDetectionFrame update(const tracker::DetectionArray& detections,
                               int image_width, int image_height);
  bool latestCandidate(int track_id, tracker::DetectionCandidate* candidate,
                       std_msgs::Header* header = nullptr) const;
  void setSelectedTrackId(int track_id);
  int selectedTrackId() const;
  void reset();
  MultiTrackStatistics statistics() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace tracker
