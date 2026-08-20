#include <cmath>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include <sar_yolo_detector/linear_assignment.hpp>

namespace sar_yolo_detector {
namespace {

TEST(LinearAssignment, UsesEveryRealColumnAtMostOnce) {
  const std::vector<std::vector<double>> costs = {
      {0.10, 0.20}, {0.11, 0.90}, {0.12, 0.13}};
  const std::vector<int> result = minimumCostAssignment(costs, 0.50);
  ASSERT_EQ(result.size(), 3U);
  EXPECT_EQ(result[0], 0);
  EXPECT_EQ(result[1], -1);
  EXPECT_EQ(result[2], 1);
}

TEST(LinearAssignment, InvalidEdgesUseUnmatchedDummy) {
  const double inf = std::numeric_limits<double>::infinity();
  const std::vector<std::vector<double>> costs = {{inf}, {0.25}};
  const std::vector<int> result = minimumCostAssignment(costs, 0.50);
  ASSERT_EQ(result.size(), 2U);
  EXPECT_EQ(result[0], -1);
  EXPECT_EQ(result[1], 0);
}

} // namespace
} // namespace sar_yolo_detector

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
