#include <gtest/gtest.h>
#include <xd_uav_controller/point_vertical_guidance.h>

namespace {
using namespace xd_uav_controller;

UnifiedReference point() {
  UnifiedReference result;
  result.type = ReferenceType::kPositionTarget;
  result.use_position = {{true, true, true}};
  return result;
}

TEST(PointVerticalGuidance, ExplicitVelocityIncludingZeroBypassesGuidance) {
  EXPECT_TRUE(usesPointVerticalGuidance(point()));
  for (int axis = 0; axis < 3; ++axis) {
    auto reference = point();
    reference.use_velocity[axis] = true;
    EXPECT_FALSE(usesPointVerticalGuidance(reference));
    reference = point();
    reference.use_acceleration[axis] = true;
    EXPECT_FALSE(usesPointVerticalGuidance(reference));
  }
  auto reference = point();
  reference.path_reference = true;
  EXPECT_FALSE(usesPointVerticalGuidance(reference));
  reference = point();
  reference.trajectory_reference = true;
  EXPECT_FALSE(usesPointVerticalGuidance(reference));
  reference = point();
  reference.type = ReferenceType::kInternal;
  EXPECT_FALSE(usesPointVerticalGuidance(reference));
}

PointVerticalOutput update(PointVerticalGuidance& guidance,
                           const Eigen::Vector3d& position,
                           const Eigen::Vector3d& velocity,
                           const Eigen::Vector3d& goal, double airspeed = 15.0) {
  return guidance.update(PointVerticalConfig(), position, velocity, goal,
                         0.0, airspeed, 15.0, 10.0, 100.0, 1, 0.6, 1.0, 0.02);
}

TEST(PointVerticalGuidance, DistantPointDescendsAlongLegAndKeepsRateContinuous) {
  PointVerticalGuidance guidance;
  const Eigen::Vector3d goal(600, 0, 5);
  double previous_rate = 0.0;
  for (int i = 0; i < 250; ++i) {
    const auto output = update(guidance, {0, 0, 100}, {15, 0, 0}, goal);
    EXPECT_FALSE(output.orbit);
    EXPECT_LE(std::abs(output.climb_rate - previous_rate), 0.016001);
    previous_rate = output.climb_rate;
  }
  EXPECT_NEAR(-95.0 * 15.0 / 600.0, previous_rate, 1e-6);
  auto output = update(guidance, {300, 0, 52.5}, {15, 0, previous_rate}, goal);
  EXPECT_FALSE(output.orbit);
  EXPECT_NEAR(previous_rate, output.climb_rate, 1e-6);
  for (int i = 0; i < 250; ++i) {
    output = update(guidance, goal, {15, 0, 0}, goal);
  }
  EXPECT_NEAR(0.0, output.climb_rate, 1e-6);
}

TEST(PointVerticalGuidance, NearbyLowPointCapturesHeightInOrbitThenResumesGoal) {
  PointVerticalGuidance guidance;
  const Eigen::Vector3d goal(100, 0, 5);
  auto output = update(guidance, {0, 0, 100}, {15, 0, 0}, goal);
  EXPECT_TRUE(output.orbit);
  EXPECT_NEAR(0.0, output.course, 1e-6);  // tangent entry, no heading jump
  EXPECT_GT(output.course_rate, 0.0);
  for (int i = 0; i < 400; ++i) {
    output = update(guidance, {0, 0, 100}, {15, 0, -2}, goal);
    EXPECT_TRUE(output.orbit);
    EXPECT_GE(output.climb_rate, -15.0 * std::sin(0.209440) - 1e-6);
  }
  // Merely crossing the altitude with a high descent speed cannot exit orbit.
  output = update(guidance, {0, 0, 5}, {15, 0, -2}, goal);
  EXPECT_TRUE(output.orbit);
  for (int i = 0; i < 400; ++i) {
    output = update(guidance, {0, 0, 5}, {15, 0, 0}, goal);
  }
  EXPECT_FALSE(output.orbit);
  EXPECT_NEAR(0.0, output.climb_rate, 1e-6);
}

TEST(PointVerticalGuidance, HighGoalRespectsAirspeedAndResetReanchorsNewLeg) {
  PointVerticalGuidance guidance;
  auto output = PointVerticalOutput();
  for (int i = 0; i < 400; ++i) {
    output = update(guidance, {0, 0, 100}, {15, 0, 0}, {100, 0, 2000});
  }
  EXPECT_TRUE(output.orbit);
  EXPECT_LE(output.climb_rate, 15.0 * std::sin(0.174533) + 1e-6);
  guidance.reset();
  output = update(guidance, {100, 0, 100}, {15, 0, 0}, {1000, 0, 100});
  EXPECT_FALSE(output.orbit);
  EXPECT_NEAR(0.0, output.climb_rate, 1e-6);
}

TEST(PointVerticalGuidance, OrbitSettlesDespiteSmallVerticalTrackingBias) {
  PointVerticalGuidance guidance;
  Eigen::Vector3d position(0, 0, 100);
  Eigen::Vector3d velocity(15, 0, 0);
  PointVerticalOutput output;
  bool saw_orbit = false;
  // A simple lagged inner loop with -0.15 m/s steady bias checks that
  // capture converges instead of remaining in an endless height orbit.
  for (int i = 0; i < 9000; ++i) {
    output = update(guidance, position, velocity, {100, 0, 5});
    saw_orbit = saw_orbit || output.orbit;
    velocity.z() += 0.02 / 0.4 *
        (output.climb_rate - 0.15 - velocity.z());
    position.z() += 0.02 * velocity.z();
  }
  EXPECT_TRUE(saw_orbit);
  EXPECT_FALSE(output.orbit);
  EXPECT_NEAR(5.0, position.z(), 1.0);
  EXPECT_NEAR(0.0, velocity.z(), 0.1);
}
}  // namespace
