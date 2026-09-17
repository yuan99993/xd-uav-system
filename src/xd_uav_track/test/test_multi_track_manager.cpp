#include <gtest/gtest.h>

#include <chrono>

#include <xd_uav_track/DetectionArray.h>
#include <xd_uav_track/DetectionCandidate.h>
#include <xd_uav_track/multi_track_manager.hpp>

namespace {

xd_uav_track::DetectionCandidate candidate(const int id, const bool stable,
                                      const float cx, const float confidence) {
  xd_uav_track::DetectionCandidate result;
  result.track_id = id;
  result.track_id_is_stable = stable;
  result.class_id = 2;
  result.has_normalized_bbox = true;
  result.normalized_bbox = {cx, 0.5F, 0.2F, 0.2F};
  result.confidence = confidence;
  return result;
}

xd_uav_track::DetectionArray frame(const double stamp,
                              const std::vector<xd_uav_track::DetectionCandidate>& items) {
  xd_uav_track::DetectionArray result;
  result.header.stamp = ros::Time(stamp);
  result.header.frame_id = "eo_optical_frame";
  result.candidates = items;
  return result;
}

xd_uav_track::TargetIdentityHint identityHint(
    const float cx, const std::string& label, const double confidence = 0.95) {
  xd_uav_track::TargetIdentityHint result;
  result.class_id = 2;
  result.normalized_bbox = {{cx, 0.5, 0.2, 0.2}};
  result.identity_label = label;
  result.source_type = "ocr";
  result.confidence = confidence;
  return result;
}

xd_uav_track::TargetWorldObservation worldObservation(
    const double stamp, const double x) {
  xd_uav_track::TargetWorldObservation result;
  result.candidate_index = 0;
  result.position = {{x, 0.0, 0.0}};
  result.sigma_m = 1.0;
  result.capture_stamp = ros::Time(stamp);
  return result;
}

TEST(MultiTrackManager, PreservesStableDetectorIdsAndConfirmsTracks) {
  xd_uav_track::MultiTrackManager manager;
  auto first = manager.update(frame(1.0, {candidate(41, true, 0.25F, 0.9F),
                                          candidate(42, true, 0.75F, 0.8F)}),
                              640, 480);
  ASSERT_EQ(2U, first.tracks.tracks.size());
  EXPECT_EQ(41, first.tracks.tracks[0].track_id);
  EXPECT_EQ("tentative", first.tracks.tracks[0].lifecycle_state);

  auto second = manager.update(frame(1.04, {candidate(41, true, 0.26F, 0.9F),
                                            candidate(42, true, 0.74F, 0.8F)}),
                               640, 480);
  ASSERT_EQ(2U, second.tracks.tracks.size());
  EXPECT_EQ("confirmed", second.tracks.tracks[0].lifecycle_state);
  EXPECT_TRUE(second.tracks.tracks[0].control_measurement_ready);
}

TEST(MultiTrackManager, AssignsPersistentIdsToUnstableDetections) {
  xd_uav_track::MultiTrackManager manager;
  auto first = manager.update(frame(2.0, {candidate(-1, false, 0.4F, 0.8F)}),
                              640, 480);
  ASSERT_EQ(1U, first.tracks.tracks.size());
  const int persistent_id = first.tracks.tracks.front().track_id;
  EXPECT_GE(persistent_id, 1000000000);

  auto second = manager.update(frame(2.04, {candidate(-1, false, 0.42F, 0.8F)}),
                               640, 480);
  ASSERT_EQ(1U, second.tracks.tracks.size());
  EXPECT_EQ(persistent_id, second.tracks.tracks.front().track_id);
  ASSERT_EQ(1U, second.candidates.candidates.size());
  EXPECT_TRUE(second.candidates.candidates.front().track_id_is_stable);
  EXPECT_EQ(persistent_id, second.candidates.candidates.front().track_id);
}

TEST(MultiTrackManager, RetainsStableDetectorIdentityAcrossIdDropout) {
  xd_uav_track::MultiTrackManager manager;
  manager.update(frame(2.5, {candidate(73, true, 0.4F, 0.9F)}), 640, 480);

  auto dropout = manager.update(
      frame(2.54, {candidate(-1, false, 0.41F, 0.9F)}), 640, 480);
  ASSERT_EQ(1U, dropout.tracks.tracks.size());
  EXPECT_EQ(73, dropout.tracks.tracks.front().track_id);
  EXPECT_TRUE(dropout.tracks.tracks.front().detector_id_stable);
  EXPECT_EQ(73, dropout.tracks.tracks.front().detector_track_id);

  auto recovered = manager.update(
      frame(2.58, {candidate(73, true, 0.42F, 0.9F)}), 640, 480);
  ASSERT_EQ(1U, recovered.tracks.tracks.size());
  EXPECT_EQ(73, recovered.tracks.tracks.front().track_id);
  EXPECT_EQ("detector_id", recovered.tracks.tracks.front().association_method);
}

TEST(MultiTrackManager, PredictsThenRemovesLostTracks) {
  xd_uav_track::MultiTrackConfig config;
  config.confirmation_hits = 1;
  config.occlusion_frames = 1;
  config.removal_frames = 2;
  xd_uav_track::MultiTrackManager manager(config);
  manager.update(frame(3.0, {candidate(8, true, 0.5F, 0.9F)}), 640, 480);
  auto missing_once = manager.update(frame(3.04, {}), 640, 480);
  ASSERT_EQ(1U, missing_once.tracks.tracks.size());
  EXPECT_TRUE(missing_once.tracks.tracks.front().predicted);
  EXPECT_EQ("occluded", missing_once.tracks.tracks.front().lifecycle_state);
  manager.update(frame(3.08, {}), 640, 480);
  auto removed = manager.update(frame(3.12, {}), 640, 480);
  EXPECT_TRUE(removed.tracks.tracks.empty());
}

TEST(MultiTrackManager, ReturnsOnlyCurrentCandidatesForSelection) {
  xd_uav_track::MultiTrackManager manager;
  manager.update(frame(4.0, {candidate(9, true, 0.5F, 0.9F)}), 640, 480);
  xd_uav_track::DetectionCandidate selected;
  EXPECT_TRUE(manager.latestCandidate(9, &selected));
  manager.update(frame(4.04, {}), 640, 480);
  EXPECT_FALSE(manager.latestCandidate(9, &selected));
}

TEST(MultiTrackManager, AppliesValidatedCameraMotionOnlyWhenEnabled) {
  xd_uav_track::MultiTrackConfig config;
  config.confirmation_hits = 1;
  xd_uav_track::MultiTrackManager manager(config);
  manager.update(frame(4.5, {candidate(19, true, 0.50F, 0.9F)}), 640, 480);
  xd_uav_track::CameraMotionCompensation motion;
  motion.valid = true;
  // Normalized translation: previous image point moves right by 0.10 in the
  // current camera frame. This is the rotation-homography contract supplied
  // by xd_uav_track_node after pose/camera calibration.
  motion.normalized_homography = {{1.0, 0.0, 0.10,
                                    0.0, 1.0, 0.0,
                                    0.0, 0.0, 1.0}};
  const auto predicted = manager.update(frame(4.54, {}), 640, 480, "eo", {}, {}, motion);
  ASSERT_EQ(1U, predicted.tracks.tracks.size());
  EXPECT_GT(predicted.tracks.tracks.front().normalized_bbox[0], 0.58F);
}

TEST(MultiTrackManager, UsesAppearanceForReidentificationWithinMotionGate) {
  xd_uav_track::MultiTrackConfig config;
  config.confirmation_hits = 1;
  config.association_iou_threshold = 0.6;
  config.association_center_distance = 3.0;
  config.appearance_minimum_cosine = 0.8;
  xd_uav_track::MultiTrackManager manager(config);
  auto first_candidate = candidate(-1, false, 0.40F, 0.9F);
  first_candidate.appearance_embedding = {1.0F, 0.0F, 0.0F};
  const auto first = manager.update(frame(5.0, {first_candidate}), 640, 480);
  ASSERT_EQ(1U, first.tracks.tracks.size());
  const int persistent_id = first.tracks.tracks.front().track_id;

  auto recovered_candidate = candidate(-1, false, 0.48F, 0.9F);
  recovered_candidate.appearance_embedding = {0.99F, 0.01F, 0.0F};
  const auto recovered = manager.update(
      frame(5.04, {recovered_candidate}), 640, 480);
  ASSERT_EQ(1U, recovered.tracks.tracks.size());
  EXPECT_EQ(persistent_id, recovered.tracks.tracks.front().track_id);
  EXPECT_TRUE(recovered.tracks.tracks.front().reidentification_match);
  EXPECT_EQ("appearance", recovered.tracks.tracks.front().association_method);
}

TEST(MultiTrackManager, LowConfidenceDetectionCannotCreatePublicIdentity) {
  xd_uav_track::MultiTrackConfig config;
  config.confirmation_hits = 1;
  config.high_confidence_threshold = 0.50;
  config.low_confidence_threshold = 0.10;
  xd_uav_track::MultiTrackManager manager(config);
  const auto output = manager.update(
      frame(6.0, {candidate(-1, false, 0.50F, 0.20F)}), 640, 480);
  EXPECT_TRUE(output.tracks.tracks.empty());
  EXPECT_TRUE(output.candidates.candidates.empty());
}

TEST(MultiTrackManager, TimeLifecycleDoesNotDependOnFrameRate) {
  xd_uav_track::MultiTrackConfig config;
  config.confirmation_hits = 1;
  config.occlusion_timeout_sec = 0.10;
  config.removal_timeout_sec = 0.25;
  xd_uav_track::MultiTrackManager manager(config);
  manager.update(frame(7.0, {candidate(-1, false, 0.50F, 0.90F)}), 640, 480);
  const auto occluded = manager.update(frame(7.08, {}), 640, 480);
  ASSERT_EQ(1U, occluded.tracks.tracks.size());
  EXPECT_EQ("occluded", occluded.tracks.tracks.front().lifecycle_state);
  const auto lost = manager.update(frame(7.15, {}), 640, 480);
  ASSERT_EQ(1U, lost.tracks.tracks.size());
  EXPECT_EQ("lost", lost.tracks.tracks.front().lifecycle_state);
  const auto removed = manager.update(frame(7.30, {}), 640, 480);
  EXPECT_TRUE(removed.tracks.tracks.empty());
}

TEST(MultiTrackManager, AdoptsVerifiedGlobalIdentityWithoutNewTrack) {
  xd_uav_track::MultiTrackConfig config;
  config.confirmation_hits = 1;
  xd_uav_track::MultiTrackManager manager(config);
  const auto first = manager.update(
      frame(8.0, {candidate(-1, false, 0.50F, 0.90F)}), 640, 480);
  ASSERT_EQ(1U, first.tracks.tracks.size());
  const int local_id = first.tracks.tracks.front().track_id;
  ASSERT_TRUE(manager.adoptPublicId(local_id, 77));
  const auto second = manager.update(
      frame(8.04, {candidate(-1, false, 0.51F, 0.90F)}), 640, 480);
  ASSERT_EQ(1U, second.tracks.tracks.size());
  EXPECT_EQ(77, second.tracks.tracks.front().track_id);
  EXPECT_FALSE(manager.adoptPublicId(77, 77 - 100));
}

TEST(MultiTrackManager, LongTermMemoryRestoresIdOnlyAfterReconfirmation) {
  xd_uav_track::MultiTrackConfig config;
  config.confirmation_hits = 1;
  config.removal_timeout_sec = 0.10;
  config.long_term_memory_ttl_sec = 10.0;
  config.long_term_reconfirmation_hits = 3;
  xd_uav_track::MultiTrackManager manager(config);
  auto initial = candidate(-1, false, 0.25F, 0.95F);
  initial.appearance_embedding = {1.0F, 0.0F, 0.0F};
  const auto first = manager.update(frame(10.0, {initial}), 640, 480);
  ASSERT_EQ(1U, first.tracks.tracks.size());
  const int original_id = first.tracks.tracks.front().track_id;
  EXPECT_TRUE(manager.update(frame(10.2, {}), 640, 480).tracks.tracks.empty());

  auto returned = candidate(-1, false, 0.75F, 0.95F);
  returned.appearance_embedding = {0.999F, 0.001F, 0.0F};
  const auto reacquired = manager.update(frame(11.0, {returned}), 640, 480);
  ASSERT_EQ(1U, reacquired.tracks.tracks.size());
  EXPECT_EQ(original_id, reacquired.tracks.tracks.front().track_id);
  EXPECT_EQ("tentative", reacquired.tracks.tracks.front().lifecycle_state);
  EXPECT_FALSE(reacquired.tracks.tracks.front().control_measurement_ready);

  const auto confirming = manager.update(frame(11.04, {returned}), 640, 480);
  ASSERT_EQ(1U, confirming.tracks.tracks.size());
  EXPECT_EQ("tentative", confirming.tracks.tracks.front().lifecycle_state);
  const auto confirmed = manager.update(frame(11.08, {returned}), 640, 480);
  ASSERT_EQ(1U, confirmed.tracks.tracks.size());
  EXPECT_EQ("confirmed", confirmed.tracks.tracks.front().lifecycle_state);
  EXPECT_TRUE(confirmed.tracks.tracks.front().control_measurement_ready);
  EXPECT_TRUE(confirmed.tracks.tracks.front().reidentification_match);
}

TEST(MultiTrackManager, AmbiguousLongTermMatchDoesNotStealOldId) {
  xd_uav_track::MultiTrackConfig config;
  config.confirmation_hits = 1;
  config.removal_timeout_sec = 0.10;
  config.long_term_memory_minimum_cosine = 0.70;
  config.long_term_memory_minimum_margin = 0.08;
  xd_uav_track::MultiTrackManager manager(config);
  auto left = candidate(-1, false, 0.25F, 0.95F);
  auto right = candidate(-1, false, 0.75F, 0.95F);
  left.appearance_embedding = {1.0F, 0.0F, 0.0F};
  right.appearance_embedding = {0.8F, 0.6F, 0.0F};
  const auto first = manager.update(frame(20.0, {left, right}), 640, 480);
  ASSERT_EQ(2U, first.tracks.tracks.size());
  const int first_id = first.tracks.tracks[0].track_id;
  const int second_id = first.tracks.tracks[1].track_id;
  manager.update(frame(20.2, {}), 640, 480);

  auto ambiguous = candidate(-1, false, 0.50F, 0.95F);
  ambiguous.appearance_embedding = {0.95F, 0.31F, 0.0F};
  const auto result = manager.update(frame(21.0, {ambiguous}), 640, 480);
  ASSERT_EQ(1U, result.tracks.tracks.size());
  EXPECT_NE(first_id, result.tracks.tracks.front().track_id);
  EXPECT_NE(second_id, result.tracks.tracks.front().track_id);
  EXPECT_EQ("identity_ambiguous", result.tracks.tracks.front().association_method);
  EXPECT_EQ("tentative", result.tracks.tracks.front().lifecycle_state);
  EXPECT_FALSE(result.tracks.tracks.front().control_measurement_ready);
  const int pending_id = result.tracks.tracks.front().track_id;

  const auto still_tied = manager.update(frame(21.04, {ambiguous}), 640, 480);
  ASSERT_EQ(1U, still_tied.tracks.tracks.size());
  EXPECT_EQ(pending_id, still_tied.tracks.tracks.front().track_id);
  EXPECT_EQ("tentative", still_tied.tracks.tracks.front().lifecycle_state);
  EXPECT_FALSE(still_tied.tracks.tracks.front().control_measurement_ready);

  auto resolved = ambiguous;
  resolved.appearance_embedding = {1.0F, 0.0F, 0.0F};
  const auto recovered = manager.update(frame(21.08, {resolved}), 640, 480);
  ASSERT_EQ(1U, recovered.tracks.tracks.size());
  EXPECT_EQ(first_id, recovered.tracks.tracks.front().track_id);
  EXPECT_NE(pending_id, recovered.tracks.tracks.front().track_id);
  EXPECT_EQ("tentative", recovered.tracks.tracks.front().lifecycle_state);
}

TEST(MultiTrackManager, PhysicalLabelNeedsRepeatedEvidenceAndRejectsConflict) {
  xd_uav_track::MultiTrackConfig config;
  config.confirmation_hits = 1;
  config.removal_timeout_sec = 0.10;
  config.identity_hint_confirmations = 2;
  xd_uav_track::MultiTrackManager manager(config);
  const auto target = candidate(-1, false, 0.30F, 0.95F);
  const auto first = manager.update(frame(30.0, {target}), 640, 480,
      "fixed", {identityHint(0.30F, "TANK-07")});
  ASSERT_EQ(1U, first.tracks.tracks.size());
  const int original_id = first.tracks.tracks.front().track_id;
  manager.update(frame(30.04, {target}), 640, 480,
      "fixed", {identityHint(0.30F, "TANK-07")});
  manager.update(frame(30.2, {}), 640, 480, "fixed", {});

  auto elsewhere = candidate(-1, false, 0.70F, 0.95F);
  const auto conflict = manager.update(frame(31.0, {elsewhere}), 640, 480,
      "fixed", {identityHint(0.70F, "TANK-99")});
  ASSERT_EQ(1U, conflict.tracks.tracks.size());
  EXPECT_NE(original_id, conflict.tracks.tracks.front().track_id);
}

TEST(MultiTrackManager, ConfirmedPhysicalLabelCanRestoreWithoutAppearance) {
  xd_uav_track::MultiTrackConfig config;
  config.confirmation_hits = 1;
  config.removal_timeout_sec = 0.10;
  config.identity_hint_confirmations = 2;
  config.long_term_reconfirmation_hits = 2;
  xd_uav_track::MultiTrackManager manager(config);
  const auto target = candidate(-1, false, 0.30F, 0.95F);
  const auto first = manager.update(frame(35.0, {target}), 640, 480,
      "fixed", {identityHint(0.30F, "TANK-07")});
  ASSERT_EQ(1U, first.tracks.tracks.size());
  const int original_id = first.tracks.tracks.front().track_id;
  manager.update(frame(35.04, {target}), 640, 480,
      "fixed", {identityHint(0.30F, "TANK-07")});
  manager.update(frame(35.2, {}), 640, 480, "fixed", {});

  auto returned = candidate(-1, false, 0.70F, 0.95F);
  const auto recovered = manager.update(frame(36.0, {returned}), 640, 480,
      "fixed", {identityHint(0.70F, "TANK-07")});
  ASSERT_EQ(1U, recovered.tracks.tracks.size());
  EXPECT_EQ(original_id, recovered.tracks.tracks.front().track_id);
  EXPECT_EQ("number_reid", recovered.tracks.tracks.front().association_method);
  EXPECT_FALSE(recovered.tracks.tracks.front().control_measurement_ready);
}

TEST(MultiTrackManager, ReplayedIdentityHintCannotConfirmAStableLabel) {
  xd_uav_track::MultiTrackConfig config;
  config.confirmation_hits = 1;
  config.identity_hint_confirmations = 2;
  xd_uav_track::MultiTrackManager manager(config);
  const auto target = candidate(-1, false, 0.30F, 0.95F);
  auto repeated = identityHint(0.30F, "TANK-07");
  repeated.capture_stamp = ros::Time(37.0);
  const auto first = manager.update(frame(37.0, {target}), 640, 480,
      "fixed", {repeated});
  ASSERT_EQ(1U, first.tracks.tracks.size());
  const int id = first.tracks.tracks.front().track_id;
  manager.update(frame(37.04, {target}), 640, 480, "fixed", {repeated});
  std::string label;
  EXPECT_FALSE(manager.identityLabel(id, &label));
}

TEST(MultiTrackManager, GroupReidActivatesForSimilarCompetingTracks) {
  xd_uav_track::MultiTrackConfig config;
  config.confirmation_hits = 1;
  config.group_trigger_minimum_features = 2;
  xd_uav_track::MultiTrackManager manager(config);
  auto left = candidate(-1, false, 0.40F, 0.95F);
  auto right = candidate(-1, false, 0.60F, 0.95F);
  left.appearance_embedding = {1.0F, 0.0F};
  right.appearance_embedding = {0.999F, 0.01F};
  manager.update(frame(40.0, {left, right}), 640, 480, "fixed");
  const auto similar = manager.update(frame(40.04, {left, right}),
                                      640, 480, "fixed");
  ASSERT_EQ(2U, similar.tracks.tracks.size());
  EXPECT_EQ("group_reid", similar.tracks.tracks[0].association_method);
  EXPECT_EQ("group_reid", similar.tracks.tracks[1].association_method);
}

TEST(MultiTrackManager, LongTermReidRejectsWorldPositionOutlier) {
  xd_uav_track::MultiTrackConfig config;
  config.confirmation_hits = 1;
  config.removal_timeout_sec = 0.10;
  config.world_innovation_gate_sigma = 3.0;
  config.world_process_noise_mps = 0.1;
  xd_uav_track::MultiTrackManager manager(config);
  auto target = candidate(-1, false, 0.30F, 0.95F);
  target.appearance_embedding = {1.0F, 0.0F};
  const auto first = manager.update(frame(50.0, {target}), 640, 480,
      "fixed", {}, {worldObservation(50.0, 0.0)});
  ASSERT_EQ(1U, first.tracks.tracks.size());
  const int original_id = first.tracks.tracks.front().track_id;
  manager.update(frame(50.2, {}), 640, 480, "fixed", {}, {});

  const auto outlier = manager.update(frame(51.0, {target}), 640, 480,
      "fixed", {}, {worldObservation(51.0, 100.0)});
  ASSERT_EQ(1U, outlier.tracks.tracks.size());
  EXPECT_NE(original_id, outlier.tracks.tracks.front().track_id);
  EXPECT_EQ("new", outlier.tracks.tracks.front().association_method);
}

TEST(MultiTrackManager, SparseSixtyFourTargetRegression) {
  xd_uav_track::MultiTrackConfig config;
  config.confirmation_hits = 1;
  config.maximum_tracks = 80;
  xd_uav_track::MultiTrackManager manager(config);
  std::vector<xd_uav_track::DetectionCandidate> detections;
  detections.reserve(64);
  for (int i = 0; i < 64; ++i) {
    auto item = candidate(-1, false, 0.05F + 0.014F * i, 0.95F);
    item.normalized_bbox[2] = 0.006F;
    item.normalized_bbox[3] = 0.006F;
    detections.push_back(std::move(item));
  }
  const auto first = manager.update(frame(60.0, detections), 1920, 1080);
  ASSERT_EQ(64U, first.tracks.tracks.size());
  std::vector<int> ids;
  for (const auto& track : first.tracks.tracks) ids.push_back(track.track_id);
  const auto start = std::chrono::steady_clock::now();
  for (int iteration = 1; iteration <= 50; ++iteration) {
    const auto result = manager.update(
        frame(60.0 + 0.033 * iteration, detections), 1920, 1080);
    ASSERT_EQ(64U, result.tracks.tracks.size());
    for (std::size_t i = 0; i < ids.size(); ++i)
      EXPECT_EQ(ids[i], result.tracks.tracks[i].track_id);
  }
  const double elapsed_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - start).count();
  RecordProperty("64_tracks_50_frames_elapsed_ms", elapsed_ms);
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::Time::init();
  return RUN_ALL_TESTS();
}
