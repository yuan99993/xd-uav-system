#include <xd_uav_formation/core.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>

using namespace xd_uav_formation;

TEST(FormationCoreCpp, SlotRotationAndYawWrapMatchPythonCore) {
  const Slot slot = slotTarget({10.0, 20.0, 30.0}, M_PI / 2.0,
                               {-4.0, 2.0, 3.0}, M_PI);
  EXPECT_NEAR(slot[0], 8.0, 1e-12);
  EXPECT_NEAR(slot[1], 16.0, 1e-12);
  EXPECT_NEAR(slot[2], 33.0, 1e-12);
  EXPECT_NEAR(slot[3], -M_PI / 2.0, 1e-12);
}

TEST(FormationCoreCpp, SlotsAndCatchupMatchPythonCore) {
  const auto circle = formationOffsets("circle", 5, 20.0);
  ASSERT_EQ(circle.size(), 5u);
  EXPECT_NEAR(circle[0][1], 20.0, 1e-12);
  for (const auto& slot : circle) {
    EXPECT_GE(std::hypot(slot[0], slot[1]), 20.0 - 1e-9);
  }
  const auto grid = formationOffsets("rectangle_compact", 5, 20.0);
  EXPECT_EQ(grid[0], (Vec3{0.0, -20.0, 0.0}));
  const Vec3 caught_up = catchupVelocity(
      {15.0, 0.0, 0.0}, {10.0, 0.0, 0.0}, {0.0, 0.0, 0.0},
      0.25, 3.0, true, 24.0, true, 11.0);
  EXPECT_EQ(caught_up, (Vec3{17.5, 0.0, 0.0}));
}

TEST(FormationCoreCpp, SlotVelocityAndPathTurnRateMatchPythonCore) {
  const Vec3 velocity = slotVelocity(
      {15.0, 0.0, 0.0}, {0.0, 1.5, 0.0}, 0.0,
      {-80.0, 0.0, 0.0}, 0.15, true);
  EXPECT_NEAR(velocity[0], 15.0, 1e-12);
  EXPECT_NEAR(velocity[1], -8.0, 1e-12);

  const std::vector<Vec3> square{{0.0, 0.0, 20.0}, {100.0, 0.0, 20.0},
                                 {100.0, 100.0, 20.0}, {0.0, 100.0, 20.0}};
  const auto route = roundedPath(square, 20.0, true);
  ASSERT_GT(route.size(), 40u);
  double largest_turn = 0.0;
  bool has_positive_turn = false;
  for (const auto& point : route) {
    const auto result = pathReferenceState(point, route, 10.0, 0.0, true);
    largest_turn = std::max(largest_turn, std::abs(result.turn_rate));
    has_positive_turn = has_positive_turn || result.turn_rate > 0.0;
  }
  EXPECT_TRUE(has_positive_turn);
  EXPECT_LT(largest_turn, 1.0);
}

TEST(FormationCoreCpp, ConsensusPacingAndAcquisitionMatchPythonCore) {
  const auto targets = consensusSpeedTargets(
      20.0, {8.0, -1.6, -1.6}, {15.0, 20.0, 20.0}, 0.4, 4.0, 0.1);
  ASSERT_EQ(targets.size(), 3u);
  EXPECT_GT(targets[0], 15.0);
  EXPECT_LT(targets[1], 20.0);
  const auto neighbors = ringNeighbors({"uav1", "uav2"}, "uav1");
  ASSERT_EQ(neighbors.size(), 1u);
  EXPECT_EQ(neighbors[0], "uav2");

  const std::vector<Vec3> starts{{0.0, 0.0, 20.0}, {0.0, 3.0, 20.0},
                                 {0.0, 6.0, 20.0}};
  const auto slots = formationOffsets("rectangle_compact", 3, 10.0);
  const auto goals = acquisitionSlots(starts, slots, 0.4, 20.0);
  ASSERT_EQ(goals.size(), starts.size());
  for (std::size_t axis = 0; axis < 2; ++axis) {
    double start_sum = 0.0, goal_sum = 0.0;
    for (std::size_t i = 0; i < starts.size(); ++i) {
      start_sum += starts[i][axis];
      goal_sum += goals[i][axis];
    }
    EXPECT_NEAR(start_sum, goal_sum, 1e-9);
  }
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

