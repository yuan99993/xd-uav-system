#include <gtest/gtest.h>

#include <tracker/DetectionArray.h>
#include <tracker/DetectionCandidate.h>
#include <tracker/multi_track_manager.hpp>

namespace {

tracker::DetectionCandidate candidate(const int id, const bool stable,
                                      const float cx, const float confidence) {
  tracker::DetectionCandidate result;
  result.track_id = id;
  result.track_id_is_stable = stable;
  result.class_id = 2;
  result.has_normalized_bbox = true;
  result.normalized_bbox = {cx, 0.5F, 0.2F, 0.2F};
  result.confidence = confidence;
  return result;
}

tracker::DetectionArray frame(const double stamp,
                              const std::vector<tracker::DetectionCandidate>& items) {
  tracker::DetectionArray result;
  result.header.stamp = ros::Time(stamp);
  result.header.frame_id = "eo_optical_frame";
  result.candidates = items;
  return result;
}

TEST(MultiTrackManager, PreservesStableDetectorIdsAndConfirmsTracks) {
  tracker::MultiTrackManager manager;
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
  tracker::MultiTrackManager manager;
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
  tracker::MultiTrackManager manager;
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
  tracker::MultiTrackConfig config;
  config.confirmation_hits = 1;
  config.occlusion_frames = 1;
  config.removal_frames = 2;
  tracker::MultiTrackManager manager(config);
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
  tracker::MultiTrackManager manager;
  manager.update(frame(4.0, {candidate(9, true, 0.5F, 0.9F)}), 640, 480);
  tracker::DetectionCandidate selected;
  EXPECT_TRUE(manager.latestCandidate(9, &selected));
  manager.update(frame(4.04, {}), 640, 480);
  EXPECT_FALSE(manager.latestCandidate(9, &selected));
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::Time::init();
  return RUN_ALL_TESTS();
}
