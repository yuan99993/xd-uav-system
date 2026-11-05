#include <gtest/gtest.h>

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

xd_uav_track::DetectionCandidate metricCandidate(
    const float cx, const float metric_y) {
  auto result = candidate(-1, false, cx, 0.9F);
  result.has_relative_position_body = true;
  result.range_valid = true;
  result.relative_position_body = {0.0F, metric_y, 0.0F};
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

TEST(MultiTrackManager, DuplicateSubclassBoxesCannotCreateTwoIdentities) {
  xd_uav_track::MultiTrackConfig config;
  config.compatible_class_ids = {0, 1};
  xd_uav_track::MultiTrackManager manager(config);
  auto car = candidate(41, true, .4F, .9F);
  car.class_id = 0;
  auto duplicate = candidate(42, true, .401F, .7F);
  duplicate.class_id = 1;
  auto result = manager.update(frame(1., {duplicate, car}), 640, 480);
  ASSERT_EQ(1U, result.tracks.tracks.size());
  EXPECT_EQ(41, result.tracks.tracks.front().detector_track_id);
  EXPECT_EQ(1U, manager.statistics().rejected_detections);
}

TEST(MultiTrackManager, DeduplicationPreservesAdjacentTargetsAndAirborneClass) {
  xd_uav_track::MultiTrackConfig config;
  config.compatible_class_ids = {0, 1};
  xd_uav_track::MultiTrackManager manager(config);
  auto car = candidate(41, true, .4F, .9F); car.class_id = 0;
  auto nearby = candidate(42, true, .5F, .8F); nearby.class_id = 0;
  auto airborne = candidate(43, true, .4F, .85F); airborne.class_id = 2;
  auto result = manager.update(frame(1., {car, nearby, airborne}), 640, 480);
  EXPECT_EQ(3U, result.tracks.tracks.size());
}

TEST(MultiTrackManager, VisualAssistancePreservesIdentityNotDetectorFreshness) {
  xd_uav_track::MultiTrackConfig config;
  config.compatible_class_ids = {0, 1, 2, 3};
  xd_uav_track::MultiTrackManager manager(config);
  auto detector = candidate(41, true, .25F, .9F);
  manager.update(frame(1.0, {detector}), 640, 480);
  manager.update(frame(1.1, {detector}), 640, 480);
  auto visual = detector;
  visual.measurement_source = "visual_tracker";
  visual.confidence = .45F;
  visual.class_id = 1;
  visual.has_relative_position_body = true;
  visual.range_valid = true;
  auto result = manager.update(frame(1.2, {visual}), 640, 480);
  ASSERT_EQ(1U, result.tracks.tracks.size());
  const auto& track = result.tracks.tracks.front();
  EXPECT_EQ(41, track.track_id);
  EXPECT_EQ(2, track.class_id);
  EXPECT_FALSE(track.detected);
  EXPECT_FALSE(track.predicted);
  EXPECT_FALSE(track.control_measurement_ready);
  EXPECT_FALSE(track.range_valid);
  EXPECT_NEAR(.9, track.confidence, 1e-5);
  EXPECT_NEAR(.45, track.tracking_quality, 1e-5);
  EXPECT_EQ("visual_tracker", track.measurement_source);
  EXPECT_EQ(ros::Time(1.1), track.last_detection_stamp);
  EXPECT_EQ(ros::Time(1.2), track.last_observation_stamp);
  xd_uav_track::MultiTrackManager empty;
  visual.confidence = .99F;
  EXPECT_TRUE(empty.update(frame(1.2, {visual}), 640, 480).tracks.tracks.empty());
}

TEST(MultiTrackManager, ModelContractInvalidationRetainsIdentityAndReconfirmsClass) {
  xd_uav_track::MultiTrackConfig config;
  config.confirmation_hits = 3;
  xd_uav_track::MultiTrackManager manager(config);
  auto item = candidate(-1, false, .4F, .9F);
  xd_uav_track::ManagedDetectionFrame latest;
  for (int i = 0; i < 3; ++i)
    latest = manager.update(frame(1.0 + i * .1, {item}), 640, 480);
  ASSERT_EQ(1U, latest.tracks.tracks.size());
  const int identity = latest.tracks.tracks[0].track_id;
  EXPECT_EQ(2, latest.tracks.tracks[0].class_id);
  manager.invalidateClasses();
  item.class_id = 1;
  for (int i = 0; i < 3; ++i) {
    auto input = frame(2.0 + i * .1, {item});
    input.model_epoch = 2; input.class_names = {"car", "tank"};
    latest = manager.update(input, 640, 480);
    ASSERT_EQ(1U, latest.tracks.tracks.size());
    EXPECT_EQ(identity, latest.tracks.tracks[0].track_id);
    EXPECT_EQ(2U, latest.tracks.model_epoch);
    EXPECT_EQ(i < 2 ? -1 : 1, latest.tracks.tracks[0].class_id);
  }
}

TEST(MultiTrackManager, KeepsIndependentFrdPositionAndObservationTimeDuringPrediction) {
  xd_uav_track::MultiTrackConfig config;
  config.confirmation_hits = 1;
  config.position_frame = "uav7/base_link_frd";
  xd_uav_track::MultiTrackManager manager(config);
  const auto observed = manager.update(frame(10.0, {metricCandidate(.4F, 3.F)}), 640, 480);
  ASSERT_EQ(1U, observed.tracks.tracks.size());
  const auto& measurement = observed.tracks.tracks[0];
  EXPECT_EQ("eo_optical_frame", measurement.header.frame_id);
  EXPECT_EQ("uav7/base_link_frd", measurement.position_frame);
  EXPECT_EQ(ros::Time(10.0), measurement.last_observation_stamp);
  EXPECT_EQ(ros::Time(10.0), measurement.position_stamp);
  const auto image_only = manager.update(frame(10.1, {candidate(-1, false, .4F, .9F)}), 640, 480);
  ASSERT_EQ(1U, image_only.tracks.tracks.size());
  EXPECT_EQ(ros::Time(10.1), image_only.tracks.tracks[0].last_observation_stamp);
  EXPECT_EQ(ros::Time(10.0), image_only.tracks.tracks[0].position_stamp);
  EXPECT_FALSE(image_only.tracks.tracks[0].range_valid);
  const auto predicted = manager.update(frame(10.2, {}), 640, 480);
  ASSERT_EQ(1U, predicted.tracks.tracks.size());
  EXPECT_TRUE(predicted.tracks.tracks[0].predicted);
  EXPECT_EQ(ros::Time(10.1), predicted.tracks.tracks[0].last_observation_stamp);
  EXPECT_EQ(ros::Time(10.0), predicted.tracks.tracks[0].position_stamp);
}

TEST(MultiTrackManager, MetricOutlierDoesNotRefreshRealPositionStamp) {
  xd_uav_track::MultiTrackConfig config;
  config.confirmation_hits = 1;
  config.metric_relative_association_enabled = true;
  config.metric_relative_max_distance_m = 100;
  config.metric_relative_max_speed_mps = .1;
  config.metric_relative_jump_tolerance_m = .1;
  xd_uav_track::MultiTrackManager manager(config);
  manager.update(frame(10.0, {metricCandidate(.4F, 1.F)}), 640, 480);
  const auto outlier = manager.update(frame(10.1, {metricCandidate(.4F, 8.F)}), 640, 480);
  ASSERT_EQ(1U, outlier.tracks.tracks.size());
  EXPECT_EQ("metric_outlier_hold", outlier.tracks.tracks[0].association_method);
  EXPECT_EQ(ros::Time(10.0), outlier.tracks.tracks[0].position_stamp);
  EXPECT_EQ(ros::Time(10.1), outlier.tracks.tracks[0].last_observation_stamp);
  EXPECT_FALSE(outlier.tracks.tracks[0].range_valid);
  ASSERT_EQ(1U, outlier.candidates.candidates.size());
  EXPECT_FALSE(outlier.candidates.candidates[0].range_valid);
}

TEST(MultiTrackManager, MetricGateDoesNotFreezeLegitimateMotionBehindSmoothing) {
  xd_uav_track::MultiTrackConfig config;
  config.confirmation_hits = 1;
  config.metric_relative_association_enabled = true;
  config.metric_relative_max_distance_m = 100;
  config.metric_relative_max_speed_mps = 6;
  config.metric_relative_jump_tolerance_m = .5;
  config.metric_relative_smoothing_alpha = .05;
  xd_uav_track::MultiTrackManager manager(config);
  int original = -1;
  for (int index = 0; index < 24; ++index) {
    const double stamp = 20.0 + .2 * index;
    const auto result = manager.update(frame(stamp,
        {metricCandidate(.4F, static_cast<float>(index))}),640,480);
    ASSERT_EQ(1U,result.tracks.tracks.size());
    const auto& track = result.tracks.tracks.front();
    if (index == 0) original = track.track_id;
    EXPECT_EQ(original,track.track_id);
    EXPECT_NE("metric_outlier_hold",track.association_method);
    EXPECT_TRUE(track.range_valid);
    EXPECT_EQ(ros::Time(stamp),track.position_stamp);
  }
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

TEST(MultiTrackManager, GlobalAssignmentAvoidsGreedyIdentitySwap) {
  xd_uav_track::MultiTrackConfig config;
  config.confirmation_hits = 1;
  xd_uav_track::MultiTrackManager manager(config);
  const auto initial = manager.update(
      frame(20.0, {candidate(-1, false, 0.40F, 0.9F),
                   candidate(-1, false, 0.60F, 0.9F)}), 640, 480);
  ASSERT_EQ(2U, initial.tracks.tracks.size());
  const int left_id = initial.candidates.candidates[0].track_id;
  const int right_id = initial.candidates.candidates[1].track_id;
  const auto crossed = manager.update(
      frame(20.04, {candidate(-1, false, 0.53F, 0.9F),
                    candidate(-1, false, 0.62F, 0.9F)}), 640, 480);
  ASSERT_EQ(2U, crossed.candidates.candidates.size());
  EXPECT_EQ(left_id, crossed.candidates.candidates[0].track_id);
  EXPECT_EQ(right_id, crossed.candidates.candidates[1].track_id);
}

TEST(MultiTrackManager, MetricAnchorPreventsOverlappingVehicleIdExchange) {
  xd_uav_track::MultiTrackConfig config;
  config.confirmation_hits = 1;
  config.metric_relative_association_enabled = true;
  config.metric_relative_max_distance_m = 10.0;
  config.metric_relative_score_weight = 1.2;
  xd_uav_track::MultiTrackManager manager(config);
  const auto initial = manager.update(
      frame(20.0, {metricCandidate(0.45F, -5.0F),
                   metricCandidate(0.55F, -21.0F)}), 640, 480);
  ASSERT_EQ(2U, initial.candidates.candidates.size());
  const int near_id = initial.candidates.candidates[0].track_id;
  const int far_id = initial.candidates.candidates[1].track_id;

  // In image space the detections exchange sides and overlap. A briefly
  // biased ground projection (far 21 m -> 14 m) must not drag its anchor onto
  // the near vehicle or let the next observation exchange public IDs.
  auto crossing = manager.update(
      frame(20.3, {metricCandidate(0.56F, -5.5F),
                   metricCandidate(0.44F, -14.0F)}), 640, 480);
  ASSERT_EQ(2U, crossing.candidates.candidates.size());
  EXPECT_EQ(near_id, crossing.candidates.candidates[0].track_id);
  EXPECT_EQ(far_id, crossing.candidates.candidates[1].track_id);
  EXPECT_EQ("metric_outlier_hold", crossing.tracks.tracks[1].association_method);
  EXPECT_NEAR(-21.0F,
              crossing.candidates.candidates[1].relative_position_body[1], 0.01F);

  auto recovered = manager.update(
      frame(20.6, {metricCandidate(0.55F, -6.0F),
                   metricCandidate(0.45F, -23.0F)}), 640, 480);
  ASSERT_EQ(2U, recovered.candidates.candidates.size());
  EXPECT_EQ(near_id, recovered.candidates.candidates[0].track_id);
  EXPECT_EQ(far_id, recovered.candidates.candidates[1].track_id);
}

TEST(MultiTrackManager, WeakDetectionUpdatesButNeverCreatesTrack) {
  xd_uav_track::MultiTrackConfig config;
  config.confirmation_hits = 1;
  config.minimum_update_confidence = 0.10;
  config.minimum_new_track_confidence = 0.50;
  config.high_confidence_threshold = 0.50;
  xd_uav_track::MultiTrackManager manager(config);
  const auto initial = manager.update(
      frame(21.0, {candidate(-1, false, 0.3F, 0.9F)}), 640, 480);
  ASSERT_EQ(1U, initial.tracks.tracks.size());
  const int id = initial.tracks.tracks[0].track_id;
  const auto next = manager.update(
      frame(21.04, {candidate(-1, false, 0.31F, 0.2F),
                    candidate(-1, false, 0.8F, 0.2F)}), 640, 480);
  ASSERT_EQ(1U, next.tracks.tracks.size());
  ASSERT_EQ(1U, next.candidates.candidates.size());
  EXPECT_EQ(id, next.candidates.candidates[0].track_id);
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

TEST(MultiTrackManager, RejectsDuplicateTimestampWithoutMutatingIdentity) {
  xd_uav_track::MultiTrackManager manager;
  const auto initial = manager.update(
      frame(40.0, {candidate(91, true, 0.35F, 0.9F)}), 640, 480);
  ASSERT_EQ(1U, initial.tracks.tracks.size());
  const int persistent_id = initial.tracks.tracks.front().track_id;

  const auto duplicate = manager.update(
      frame(40.0, {candidate(92, true, 0.8F, 0.99F)}), 640, 480);
  EXPECT_FALSE(duplicate.accepted);
  EXPECT_TRUE(duplicate.tracks.tracks.empty());
  EXPECT_TRUE(duplicate.candidates.candidates.empty());

  const auto next = manager.update(
      frame(40.04, {candidate(91, true, 0.36F, 0.9F)}), 640, 480);
  ASSERT_TRUE(next.accepted);
  ASSERT_EQ(1U, next.candidates.candidates.size());
  EXPECT_EQ(persistent_id, next.candidates.candidates.front().track_id);
}

TEST(MultiTrackManager, HoldsAmbiguousCrossingInsteadOfForcingAnIdentity) {
  xd_uav_track::MultiTrackConfig config;
  config.confirmation_hits = 1;
  config.association_ambiguity_margin = 0.08;
  xd_uav_track::MultiTrackManager manager(config);
  const auto initial = manager.update(
      frame(41.0, {candidate(101, true, 0.45F, 0.9F),
                   candidate(102, true, 0.55F, 0.9F)}), 640, 480);
  ASSERT_EQ(2U, initial.tracks.tracks.size());

  const auto crossing = manager.update(
      frame(41.04, {candidate(-1, false, 0.50F, 0.9F)}), 640, 480);
  EXPECT_TRUE(crossing.candidates.candidates.empty());
  ASSERT_EQ(2U, crossing.tracks.tracks.size());
  EXPECT_EQ("ambiguous", crossing.tracks.tracks[0].association_method);
  EXPECT_EQ("ambiguous", crossing.tracks.tracks[1].association_method);
  EXPECT_TRUE(crossing.tracks.tracks[0].predicted);
  EXPECT_FALSE(crossing.tracks.tracks[0].control_measurement_ready);
}

TEST(MultiTrackManager, KeepsPublicIdAndWaitsForConsecutiveReacquisition) {
  xd_uav_track::MultiTrackConfig config;
  config.confirmation_hits = 1;
  config.occlusion_timeout_sec = 0.10;
  config.removal_timeout_sec = 1.0;
  config.reacquisition_after_sec = 0.10;
  config.reacquisition_confirm_hits = 3;
  xd_uav_track::MultiTrackManager manager(config);
  const auto initial = manager.update(
      frame(42.0, {candidate(87, true, 0.40F, 0.9F)}), 640, 480);
  ASSERT_EQ(1U, initial.tracks.tracks.size());
  const int locked_id = initial.tracks.tracks.front().track_id;

  const auto lost = manager.update(frame(42.20, {}), 640, 480);
  ASSERT_EQ(1U, lost.tracks.tracks.size());
  EXPECT_EQ("lost", lost.tracks.tracks.front().lifecycle_state);

  const auto first = manager.update(
      frame(42.24, {candidate(87, true, 0.41F, 0.9F)}), 640, 480);
  ASSERT_EQ(1U, first.tracks.tracks.size());
  EXPECT_EQ(locked_id, first.tracks.tracks.front().track_id);
  EXPECT_EQ("reacquiring", first.tracks.tracks.front().association_method);
  EXPECT_FALSE(first.tracks.tracks.front().control_measurement_ready);

  const auto second = manager.update(
      frame(42.28, {candidate(87, true, 0.42F, 0.9F)}), 640, 480);
  ASSERT_EQ(1U, second.tracks.tracks.size());
  EXPECT_FALSE(second.tracks.tracks.front().control_measurement_ready);

  const auto third = manager.update(
      frame(42.32, {candidate(87, true, 0.43F, 0.9F)}), 640, 480);
  ASSERT_EQ(1U, third.tracks.tracks.size());
  EXPECT_EQ(locked_id, third.tracks.tracks.front().track_id);
  EXPECT_TRUE(third.tracks.tracks.front().control_measurement_ready);
}

TEST(MultiTrackManager, ReidentifiesChangedSourceIdWithAppearanceAfterLoss) {
  xd_uav_track::MultiTrackConfig config;
  config.confirmation_hits = 1;
  config.association_iou_threshold = 0.6;
  config.association_center_distance = 3.0;
  config.appearance_minimum_cosine = 0.8;
  config.occlusion_timeout_sec = 0.10;
  config.removal_timeout_sec = 1.0;
  config.reacquisition_after_sec = 0.10;
  config.reacquisition_confirm_hits = 3;
  xd_uav_track::MultiTrackManager manager(config);

  auto original = candidate(-1, false, 0.40F, 0.9F);
  original.appearance_embedding = {1.0F, 0.0F, 0.0F};
  const auto initial = manager.update(frame(44.0, {original}), 640, 480);
  ASSERT_EQ(1U, initial.tracks.tracks.size());
  const int locked_id = initial.tracks.tracks.front().track_id;
  manager.update(frame(44.20, {}), 640, 480);

  auto return_candidate = candidate(500, true, 0.50F, 0.9F);
  return_candidate.appearance_embedding = {0.99F, 0.01F, 0.0F};
  const auto first = manager.update(
      frame(44.24, {return_candidate}), 640, 480);
  ASSERT_EQ(1U, first.tracks.tracks.size());
  EXPECT_EQ(locked_id, first.tracks.tracks.front().track_id);
  EXPECT_TRUE(first.tracks.tracks.front().reidentification_match);
  EXPECT_FALSE(first.tracks.tracks.front().control_measurement_ready);

  auto confirmation = return_candidate;
  confirmation.normalized_bbox[0] = 0.51F;
  manager.update(frame(44.28, {confirmation}), 640, 480);
  confirmation.normalized_bbox[0] = 0.52F;
  const auto confirmed = manager.update(
      frame(44.32, {confirmation}), 640, 480);
  ASSERT_EQ(1U, confirmed.tracks.tracks.size());
  EXPECT_EQ(locked_id, confirmed.tracks.tracks.front().track_id);
  EXPECT_TRUE(confirmed.tracks.tracks.front().control_measurement_ready);
}

TEST(MultiTrackManager, UsesElapsedTimeForConfiguredOcclusionAndRemoval) {
  xd_uav_track::MultiTrackConfig config;
  config.confirmation_hits = 1;
  config.occlusion_timeout_sec = 0.10;
  config.removal_timeout_sec = 0.50;
  xd_uav_track::MultiTrackManager manager(config);
  manager.update(frame(43.0, {candidate(88, true, 0.5F, 0.9F)}), 640, 480);

  const auto lost = manager.update(frame(43.20, {}), 640, 480);
  ASSERT_EQ(1U, lost.tracks.tracks.size());
  EXPECT_EQ("lost", lost.tracks.tracks.front().lifecycle_state);
  const auto expired = manager.update(frame(43.60, {}), 640, 480);
  EXPECT_TRUE(expired.tracks.tracks.empty());
}

TEST(MultiTrackManager, KeepsIdentityAcrossYawAspectChangeAfterDetectorGap) {
  xd_uav_track::MultiTrackConfig config;
  config.confirmation_hits = 1;
  config.occlusion_timeout_sec = 0.20;
  config.removal_timeout_sec = 2.0;
  // The single-target detection-only benchmark has no appearance vector;
  // permit Kalman reacquisition throughout its configured local lifetime.
  config.reacquisition_after_sec = 2.0;
  config.association_center_distance = 2.0;
  xd_uav_track::MultiTrackManager manager(config);

  auto broadside = candidate(-1, false, 0.50F, 0.9F);
  broadside.normalized_bbox = {0.50F, 0.50F, 0.28F, 0.12F};
  const auto initial = manager.update(frame(46.0, {broadside}), 640, 480);
  ASSERT_EQ(1U, initial.tracks.tracks.size());
  const int identity = initial.tracks.tracks.front().track_id;

  manager.update(frame(46.2, {}), 640, 480);
  manager.update(frame(46.4, {}), 640, 480);
  auto end_on = candidate(-1, false, 0.51F, 0.85F);
  end_on.normalized_bbox = {0.51F, 0.50F, 0.12F, 0.28F};
  const auto reacquired = manager.update(frame(46.6, {end_on}), 640, 480);

  ASSERT_EQ(1U, reacquired.candidates.candidates.size());
  EXPECT_EQ(identity, reacquired.candidates.candidates.front().track_id);
  ASSERT_EQ(1U, reacquired.tracks.tracks.size());
  EXPECT_EQ("confirmed", reacquired.tracks.tracks.front().lifecycle_state);
}

TEST(MultiTrackManager, KeepsArmoredIdentityAcrossConfiguredClassFlip) {
  xd_uav_track::MultiTrackConfig config;
  config.confirmation_hits = 1;
  config.compatible_class_ids = {2, 3};
  xd_uav_track::MultiTrackManager manager(config);

  auto tank = candidate(-1, false, 0.50F, 0.9F);
  tank.class_id = 2;
  const auto first = manager.update(frame(47.0, {tank}), 640, 480);
  ASSERT_EQ(1U, first.tracks.tracks.size());
  const int identity = first.tracks.tracks.front().track_id;

  auto m142 = candidate(-1, false, 0.51F, 0.85F);
  m142.class_id = 3;
  const auto flipped = manager.update(frame(47.1, {m142}), 640, 480);
  ASSERT_EQ(1U, flipped.candidates.candidates.size());
  EXPECT_EQ(identity, flipped.candidates.candidates.front().track_id);
  EXPECT_EQ(2, flipped.candidates.candidates.front().class_id);
  EXPECT_EQ(2, flipped.tracks.tracks.front().class_id);
  EXPECT_EQ("class_bridge", flipped.tracks.tracks.front().association_method);
}

TEST(MultiTrackManager, ChoosesClassAtConfirmationAndKeepsItThroughLaterFlips) {
  xd_uav_track::MultiTrackConfig config;
  config.confirmation_hits = 2;
  config.compatible_class_ids = {0, 1, 2, 3};
  xd_uav_track::MultiTrackManager manager(config);

  auto first = candidate(-1, false, 0.50F, 0.55F);
  first.class_id = 2;
  const auto tentative = manager.update(frame(49.0, {first}), 640, 480);
  ASSERT_EQ(1U, tentative.tracks.tracks.size());
  EXPECT_EQ("tentative", tentative.tracks.tracks.front().lifecycle_state);
  const int identity = tentative.tracks.tracks.front().track_id;

  auto second = candidate(-1, false, 0.51F, 0.90F);
  second.class_id = 0;
  const auto confirmed = manager.update(frame(49.1, {second}), 640, 480);
  ASSERT_EQ(1U, confirmed.tracks.tracks.size());
  EXPECT_EQ(identity, confirmed.tracks.tracks.front().track_id);
  EXPECT_EQ("confirmed", confirmed.tracks.tracks.front().lifecycle_state);
  EXPECT_EQ(0, confirmed.tracks.tracks.front().class_id);
  EXPECT_EQ(0, confirmed.candidates.candidates.front().class_id);

  auto third = candidate(-1, false, 0.52F, 0.95F);
  third.class_id = 1;
  const auto later = manager.update(frame(49.2, {third}), 640, 480);
  ASSERT_EQ(1U, later.tracks.tracks.size());
  EXPECT_EQ(identity, later.tracks.tracks.front().track_id);
  EXPECT_EQ(0, later.tracks.tracks.front().class_id);
  EXPECT_EQ(0, later.candidates.candidates.front().class_id);
}

TEST(MultiTrackManager, TrustedUpstreamClassCorrectionKeepsIdentityAndIgnoresVisualClass) {
  xd_uav_track::MultiTrackConfig config;
  config.confirmation_hits = 1;
  config.compatible_class_ids = {0, 1};
  config.upstream_class_consensus = true;
  xd_uav_track::MultiTrackManager manager(config);
  auto observation = candidate(41, true, .4F, .9F);
  observation.class_id = 0;
  auto first = manager.update(frame(1., {observation}), 640, 480);
  ASSERT_EQ(1U, first.tracks.tracks.size());
  const auto identity = first.tracks.tracks.front().track_id;
  observation.class_id = 1;
  auto corrected = manager.update(frame(1.2, {observation}), 640, 480);
  ASSERT_EQ(1U, corrected.tracks.tracks.size());
  EXPECT_EQ(identity, corrected.tracks.tracks.front().track_id);
  EXPECT_EQ(1, corrected.tracks.tracks.front().class_id);
  EXPECT_EQ(1, corrected.candidates.candidates.front().class_id);
  observation.measurement_source = "visual_tracker";
  observation.class_id = 0;
  auto visual = manager.update(frame(1.4, {observation}), 640, 480);
  ASSERT_EQ(1U, visual.tracks.tracks.size());
  EXPECT_EQ(1, visual.tracks.tracks.front().class_id);
  EXPECT_EQ(identity, visual.tracks.tracks.front().track_id);
}

TEST(MultiTrackManager, UnstableIdsCannotOverrideConfirmedClassContract) {
  xd_uav_track::MultiTrackConfig config;
  config.confirmation_hits = 1;
  config.compatible_class_ids = {0, 1};
  config.upstream_class_consensus = true;
  xd_uav_track::MultiTrackManager manager(config);
  auto observation = candidate(-1, false, .4F, .9F);
  observation.class_id = 0;
  manager.update(frame(1., {observation}), 640, 480);
  observation.class_id = 1;
  auto result = manager.update(frame(1.2, {observation}), 640, 480);
  ASSERT_EQ(1U, result.tracks.tracks.size());
  EXPECT_EQ(0, result.tracks.tracks.front().class_id);
}

TEST(MultiTrackManager, ClassFlipDoesNotResetReacquisitionConfirmation) {
  xd_uav_track::MultiTrackConfig config;
  config.confirmation_hits = 1;
  config.compatible_class_ids = {0, 1, 2, 3};
  config.occlusion_timeout_sec = 0.10;
  config.removal_timeout_sec = 2.0;
  config.reacquisition_after_sec = 0.10;
  config.reacquisition_confirm_hits = 2;
  config.reacquisition_mode = "balanced";
  xd_uav_track::MultiTrackManager manager(config);

  auto original = candidate(-1, false, 0.50F, 0.90F);
  original.class_id = 2;
  original.appearance_embedding = {1.0F, 0.0F};
  const auto initial = manager.update(frame(50.0, {original}), 640, 480);
  const int identity = initial.tracks.tracks.front().track_id;
  manager.update(frame(50.2, {}), 640, 480);

  auto returning = candidate(-1, false, 0.51F, 0.85F);
  returning.class_id = 0;
  returning.appearance_embedding = {1.0F, 0.0F};
  const auto pending = manager.update(frame(50.25, {returning}), 640, 480);
  ASSERT_EQ(1U, pending.tracks.tracks.size());
  EXPECT_EQ(identity, pending.tracks.tracks.front().track_id);
  EXPECT_FALSE(pending.tracks.tracks.front().control_measurement_ready);

  returning.class_id = 1;
  returning.normalized_bbox[0] = 0.52F;
  const auto recovered = manager.update(frame(50.30, {returning}), 640, 480);
  ASSERT_EQ(1U, recovered.tracks.tracks.size());
  EXPECT_EQ(identity, recovered.tracks.tracks.front().track_id);
  EXPECT_EQ(2, recovered.tracks.tracks.front().class_id);
  EXPECT_TRUE(recovered.tracks.tracks.front().control_measurement_ready);
}

TEST(MultiTrackManager, NearbyArmoredVehiclesKeepSeparateIdsAcrossClassFlips) {
  xd_uav_track::MultiTrackConfig config;
  config.confirmation_hits = 1;
  config.compatible_class_ids = {0, 1, 2, 3};
  xd_uav_track::MultiTrackManager manager(config);

  auto left = candidate(-1, false, 0.40F, 0.90F);
  auto right = candidate(-1, false, 0.60F, 0.90F);
  right.class_id = 0;
  const auto first = manager.update(frame(48.0, {left, right}), 640, 480);
  ASSERT_EQ(2U, first.candidates.candidates.size());
  const int left_id = first.candidates.candidates[0].track_id;
  const int right_id = first.candidates.candidates[1].track_id;
  EXPECT_NE(left_id, right_id);

  left.class_id = 1;
  left.normalized_bbox[0] = 0.42F;
  right.class_id = 2;
  right.normalized_bbox[0] = 0.58F;
  const auto flipped = manager.update(frame(48.1, {right, left}), 640, 480);
  ASSERT_EQ(2U, flipped.candidates.candidates.size());
  EXPECT_EQ(right_id, flipped.candidates.candidates[0].track_id);
  EXPECT_EQ(left_id, flipped.candidates.candidates[1].track_id);
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::Time::init();
  return RUN_ALL_TESTS();
}
