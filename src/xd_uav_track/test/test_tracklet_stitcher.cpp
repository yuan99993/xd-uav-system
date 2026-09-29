#include <gtest/gtest.h>

#include <xd_uav_track/tracklet_stitcher.hpp>

namespace {

xd_uav_track::TrackletIdentitySnapshot identity(
    const int id, const double stamp, const double x, const double vx,
    const std::string& source, const std::vector<float>& appearance) {
  xd_uav_track::TrackletIdentitySnapshot result;
  result.identity_id = id;
  result.class_id = 2;
  result.source = source;
  result.last_seen_sec = stamp;
  result.established = true;
  result.active_in_source = false;
  result.has_world_position = true;
  result.has_world_velocity = true;
  result.world_position = {{x, 0.0, 0.0}};
  result.position_variance = {{0.1, 0.1, 0.1}};
  result.world_velocity = {{vx, 0.0, 0.0}};
  result.appearance_prototypes.push_back(appearance);
  return result;
}

xd_uav_track::TrackletObservation observation(
    const double stamp, const double x, const double vx,
    const std::string& source, const std::vector<float>& appearance) {
  xd_uav_track::TrackletObservation result;
  result.class_id = 2;
  result.source = source;
  result.stamp_sec = stamp;
  result.confirmed = true;
  result.has_world_position = true;
  result.has_world_velocity = true;
  result.world_position = {{x, 0.0, 0.0}};
  result.position_variance = {{0.1, 0.1, 0.1}};
  result.world_velocity = {{vx, 0.0, 0.0}};
  result.appearance = appearance;
  return result;
}

TEST(TrackletStitcher, PredictsInertialMotionInsteadOfUsingNearestPosition) {
  const auto moving = identity(21, 1.0, 0.0, 1.0, "fixed", {1.0F, 0.0F});
  const auto distractor = identity(22, 1.0, 3.0, -1.0, "fixed", {0.0F, 1.0F});
  const auto result = xd_uav_track::findTrackletStitchMatch(
      observation(4.0, 3.0, 1.0, "fixed", {0.99F, 0.01F}),
      {moving, distractor}, xd_uav_track::TrackletStitchConfig());
  EXPECT_EQ(21, result.identity_id);
  EXPECT_FALSE(result.ambiguous);
  EXPECT_EQ("matched", result.reason);
}

TEST(TrackletStitcher, LeavesNearTieAmbiguous) {
  const auto first = identity(31, 1.0, 0.0, 1.0, "fixed", {1.0F, 0.0F});
  const auto second = identity(32, 1.0, 0.0, 1.0, "fixed", {1.0F, 0.0F});
  const auto result = xd_uav_track::findTrackletStitchMatch(
      observation(2.0, 1.0, 1.0, "fixed", {1.0F, 0.0F}),
      {first, second}, xd_uav_track::TrackletStitchConfig());
  EXPECT_EQ(-1, result.identity_id);
  EXPECT_TRUE(result.ambiguous);
  EXPECT_EQ("best_second_margin_too_small", result.reason);
}

TEST(TrackletStitcher, RefusesPositionOnlyAndUncalibratedCrossSourceMatches) {
  auto old = identity(41, 1.0, 0.0, 0.0, "fixed", {});
  old.has_world_velocity = false;
  const auto same_source = xd_uav_track::findTrackletStitchMatch(
      observation(2.0, 0.0, 0.0, "fixed", {}), {old},
      xd_uav_track::TrackletStitchConfig());
  EXPECT_EQ(-1, same_source.identity_id);

  auto incoming = observation(2.0, 0.0, 0.0, "gimbal", {});
  const auto cross_source = xd_uav_track::findTrackletStitchMatch(
      incoming, {old}, xd_uav_track::TrackletStitchConfig());
  EXPECT_EQ(-1, cross_source.identity_id);
}

TEST(TrackletStitcher, MergesSynchronizedMetricCrossSourceWithoutReid) {
  auto old = identity(45, 10.0, 12.0, 0.0, "benchmark_fixed", {});
  old.has_world_velocity = false;
  auto incoming = observation(10.08, 12.25, 0.0, "benchmark_gimbal", {});
  incoming.has_world_velocity = false;
  auto config = xd_uav_track::TrackletStitchConfig();
  config.cross_source_metric_enabled = true;

  const auto result = xd_uav_track::findTrackletStitchMatch(
      incoming, {old}, config);
  EXPECT_EQ(45, result.identity_id);
  EXPECT_FALSE(result.ambiguous);
  EXPECT_EQ("matched", result.reason);
}

TEST(TrackletStitcher, AllowsVehicleSubtypeDifferenceAcrossMetricSources) {
  auto fixed = identity(49, 10.0, 12.0, 0.0, "benchmark_fixed", {});
  auto gimbal_measurement = observation(
      10.08, 12.10, 0.0, "benchmark_gimbal", {});
  fixed.class_id = 1;
  gimbal_measurement.class_id = 2;
  fixed.has_world_velocity = false;
  gimbal_measurement.has_world_velocity = false;
  auto config = xd_uav_track::TrackletStitchConfig();
  config.cross_source_metric_enabled = true;

  const auto result = xd_uav_track::findTrackletStitchMatch(
      gimbal_measurement, {fixed}, config);
  EXPECT_EQ(49, result.identity_id);
  EXPECT_FALSE(result.ambiguous);
  EXPECT_GT(result.score, config.minimum_score);
}

TEST(TrackletStitcher, RejectsStaleOrUncertainCrossSourceMetric) {
  auto old = identity(46, 10.0, 12.0, 0.0, "benchmark_fixed", {});
  old.has_world_velocity = false;
  auto incoming = observation(10.5, 12.0, 0.0, "benchmark_gimbal", {});
  incoming.has_world_velocity = false;
  auto config = xd_uav_track::TrackletStitchConfig();
  config.cross_source_metric_enabled = true;
  EXPECT_EQ(-1, xd_uav_track::findTrackletStitchMatch(
                    incoming, {old}, config).identity_id);

  incoming.stamp_sec = 10.05;
  incoming.position_variance = {{10.0, 0.1, 0.1}};
  EXPECT_EQ(-1, xd_uav_track::findTrackletStitchMatch(
                    incoming, {old}, config).identity_id);
}

TEST(TrackletStitcher, RecoversLongLostLockedIdentityOnlyWithStrongAppearance) {
  auto old = identity(61, 10.0, 0.0, 1.0, "fixed", {1.0F, 0.0F});
  old.locked_target = true;
  auto incoming = observation(310.0, 180.0, 0.0, "fixed",
                             {0.995F, 0.005F});
  auto config = xd_uav_track::TrackletStitchConfig();
  config.archive_ttl_sec = 40.0;
  config.locked_identity_archive_ttl_sec = 600.0;
  config.locked_identity_min_appearance_cosine = 0.90;

  const auto result = xd_uav_track::findTrackletStitchMatch(
      incoming, {old}, config);
  EXPECT_EQ(61, result.identity_id);
  EXPECT_FALSE(result.ambiguous);
  EXPECT_EQ("matched", result.reason);
}

TEST(TrackletStitcher, RecoversShortGapAfterVehicleTurnsDespiteAppearanceChange) {
  auto old = identity(65, 312.0, -24.7, -1.45, "fixed", {1.0F, 0.0F});
  auto incoming = observation(320.0, -30.0, 1.45, "fixed", {0.0F, 1.0F});
  auto config = xd_uav_track::TrackletStitchConfig();
  config.process_noise_m2_per_s2 = 2.0;

  const auto match = xd_uav_track::findTrackletStitchMatch(
      incoming, {old}, config);
  EXPECT_EQ(65, match.identity_id);
  EXPECT_FALSE(match.ambiguous);

  // A similarly looking vehicle on another lane must not inherit that ID.
  incoming.world_position[1] = 12.0;
  EXPECT_EQ(-1, xd_uav_track::findTrackletStitchMatch(
                    incoming, {old}, config).identity_id);
}

TEST(TrackletStitcher, DoesNotUseLongGapPositionOrWeakAppearanceForLockedIdentity) {
  auto old = identity(62, 10.0, 0.0, 1.0, "fixed", {1.0F, 0.0F});
  old.locked_target = true;
  auto incoming = observation(310.0, 180.0, 0.0, "fixed", {0.0F, 1.0F});
  auto config = xd_uav_track::TrackletStitchConfig();
  config.archive_ttl_sec = 40.0;
  config.locked_identity_archive_ttl_sec = 600.0;

  EXPECT_EQ(-1, xd_uav_track::findTrackletStitchMatch(
                    incoming, {old}, config).identity_id);
  incoming.appearance.clear();
  EXPECT_EQ(-1, xd_uav_track::findTrackletStitchMatch(
                    incoming, {old}, config).identity_id);
}

TEST(TrackletStitcher, KeepsLongGapLockedRecoveryAmbiguousForSimilarVehicles) {
  auto locked = identity(63, 10.0, 0.0, 1.0, "fixed", {1.0F, 0.0F});
  locked.locked_target = true;
  auto similar = identity(64, 10.0, 200.0, 0.0, "fixed", {1.0F, 0.0F});
  auto incoming = observation(310.0, -150.0, 0.0, "fixed",
                             {0.999F, 0.001F});
  auto config = xd_uav_track::TrackletStitchConfig();
  config.archive_ttl_sec = 40.0;
  config.locked_identity_archive_ttl_sec = 600.0;
  config.locked_identity_min_appearance_cosine = 0.90;

  const auto result = xd_uav_track::findTrackletStitchMatch(
      incoming, {locked, similar}, config);
  EXPECT_EQ(-1, result.identity_id);
  EXPECT_TRUE(result.ambiguous);
  EXPECT_EQ("best_second_margin_too_small", result.reason);
}

TEST(TrackletStitcher, RejectsAmbiguousCrossSourceWorldMatch) {
  auto first = identity(47, 10.0, 12.0, 0.0, "benchmark_fixed", {});
  auto second = identity(48, 10.0, 12.1, 0.0, "benchmark_fixed", {});
  first.has_world_velocity = false;
  second.has_world_velocity = false;
  auto incoming = observation(10.05, 12.05, 0.0, "benchmark_gimbal", {});
  incoming.has_world_velocity = false;
  auto config = xd_uav_track::TrackletStitchConfig();
  config.cross_source_metric_enabled = true;

  const auto result = xd_uav_track::findTrackletStitchMatch(
      incoming, {first, second}, config);
  EXPECT_EQ(-1, result.identity_id);
  EXPECT_TRUE(result.ambiguous);
  EXPECT_EQ("best_second_margin_too_small", result.reason);
}

TEST(TrackletStitcher, RejectsActiveSameSourceIdentityAndExpiredArchive) {
  auto active = identity(51, 1.0, 0.0, 1.0, "fixed", {1.0F, 0.0F});
  active.active_in_source = true;
  auto config = xd_uav_track::TrackletStitchConfig();
  const auto active_result = xd_uav_track::findTrackletStitchMatch(
      observation(2.0, 1.0, 1.0, "fixed", {1.0F, 0.0F}), {active}, config);
  EXPECT_EQ(-1, active_result.identity_id);

  active.active_in_source = false;
  config.archive_ttl_sec = 0.5;
  const auto expired_result = xd_uav_track::findTrackletStitchMatch(
      observation(2.0, 1.0, 1.0, "fixed", {1.0F, 0.0F}), {active}, config);
  EXPECT_EQ(-1, expired_result.identity_id);
}

TEST(TrackletStitcher, RejectsClassConflictAndUnconfirmedObservation) {
  auto old = identity(61, 1.0, 0.0, 1.0, "fixed", {1.0F, 0.0F});
  auto candidate = observation(2.0, 1.0, 1.0, "fixed", {1.0F, 0.0F});
  candidate.class_id = 3;
  EXPECT_EQ(-1, xd_uav_track::findTrackletStitchMatch(
      candidate, {old}, xd_uav_track::TrackletStitchConfig()).identity_id);
  auto compatible = xd_uav_track::TrackletStitchConfig();
  compatible.compatible_class_ids = {2, 3};
  const auto bridged = xd_uav_track::findTrackletStitchMatch(
      candidate, {old}, compatible);
  EXPECT_EQ(61, bridged.identity_id);
  candidate.class_id = 2;
  candidate.confirmed = false;
  EXPECT_EQ(-1, xd_uav_track::findTrackletStitchMatch(
      candidate, {old}, xd_uav_track::TrackletStitchConfig()).identity_id);
}

TEST(TrackletStitcher, RequiresConsecutiveSameIdentityConfirmation) {
  xd_uav_track::TrackletStitchMatch match;
  match.identity_id = 71;
  match.score = 0.9;
  auto state = xd_uav_track::advanceTrackletStitchConfirmation(
      {}, match, 5.0, 0.5);
  ASSERT_EQ(1, state.consecutive_hits);
  state = xd_uav_track::advanceTrackletStitchConfirmation(
      state, match, 5.1, 0.5);
  EXPECT_EQ(2, state.consecutive_hits);
  state = xd_uav_track::advanceTrackletStitchConfirmation(
      state, match, 5.7, 0.5);
  EXPECT_EQ(1, state.consecutive_hits);

  match.identity_id = 72;
  state = xd_uav_track::advanceTrackletStitchConfirmation(
      state, match, 5.8, 0.5);
  EXPECT_EQ(1, state.consecutive_hits);
  match.ambiguous = true;
  state = xd_uav_track::advanceTrackletStitchConfirmation(
      state, match, 5.9, 0.5);
  EXPECT_EQ(-1, state.identity_id);
  EXPECT_EQ(0, state.consecutive_hits);
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
