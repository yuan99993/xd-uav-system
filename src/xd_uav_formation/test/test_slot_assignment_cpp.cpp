#include <xd_uav_formation/runtime.hpp>
#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>

using namespace xd_uav_formation;

namespace {
FormationGroup group(const std::string& id,
                     const std::vector<std::string>& members) {
  FormationGroup result;
  result.id = id;
  result.members = members;
  result.nominal_size = members.size();
  return result;
}

const FormationGroup& findGroup(const FormationSlotAssignment& assignment,
                                const std::string& id) {
  for (const auto& entry : assignment.ordered_groups.groups)
    if (entry.id == id) return entry;
  throw std::runtime_error("missing group");
}
}  // namespace

TEST(SlotAssignment, SingletonGroupsChooseNearbySlotsAndKeepTheirIdentity) {
  FormationGroupPlan plan;
  plan.group_size = 1;
  plan.groups = {group("g1", {"u1"}), group("g2", {"u2"})};
  // The first line slot is y=+40. u1 starts on the opposite slot.
  const std::map<std::string, Vec3> positions{
      {"u1", {100, -40, 100}}, {"u2", {100, 40, 100}}};
  const auto result = assignFormationSlots(plan, {}, positions,
      "line", 80, 0, 2, 0, 100);
  ASSERT_EQ(result.ordered_groups.groups.size(), 2u);
  EXPECT_EQ(result.ordered_groups.groups[0].id, "g2");
  EXPECT_EQ(result.ordered_groups.groups[1].id, "g1");
  EXPECT_EQ(findGroup(result, "g1").members, (std::vector<std::string>{"u1"}));
  EXPECT_EQ(plan.groups[0].id, "g1");
  // Crossing positions during flight must not trigger another assignment.
  const auto held = assignFormationSlots(plan, result,
      {{"u1", {100, 40, 100}}, {"u2", {100, -40, 100}}},
      "line", 80, 0, 2, 0, 100);
  EXPECT_EQ(held.ordered_groups.groups[0].id, "g2");
}

TEST(SlotAssignment, RotationIsIncludedInDistanceCosts) {
  FormationGroupPlan plan;
  plan.group_size = 1;
  plan.groups = {group("g1", {"u1"}), group("g2", {"u2"})};
  const auto result = assignFormationSlots(plan, {},
      {{"u1", {40, 0, 100}}, {"u2", {-40, 0, 100}}},
      "line", 80, 0, 2, M_PI / 2, 100);
  EXPECT_EQ(result.ordered_groups.groups[0].id, "g2");
}

TEST(SlotAssignment, MultiAircraftGroupsOptimizeOuterAndInnerSlots) {
  FormationGroupPlan plan;
  plan.group_size = 2;
  plan.revision = 1;
  plan.groups = {group("g1", {"a", "b"}), group("g2", {"c", "d"})};
  // Outer centers are y=+100,-100; inner offsets are y=+10,-10.
  const std::map<std::string, Vec3> positions{
      {"a", {0, -110, 100}}, {"b", {0, -90, 100}},
      {"c", {0, 90, 100}}, {"d", {0, 110, 100}}};
  const auto result = assignFormationSlots(plan, {}, positions,
      "line", 20, 200, 2, 0, 100);
  EXPECT_EQ(result.ordered_groups.groups[0].id, "g2");
  EXPECT_EQ(findGroup(result, "g1").members, (std::vector<std::string>{"b", "a"}));
  EXPECT_EQ(findGroup(result, "g2").members, (std::vector<std::string>{"d", "c"}));

  auto reduced = plan;
  reduced.groups[0].members = {"b"};
  // Reverse the other group's positions to detect accidental reassignment.
  auto moved = positions;
  moved.erase("a");
  std::swap(moved["c"], moved["d"]);
  const auto after_dropout = assignFormationSlots(reduced, result, moved,
      "line", 20, 200, 2, 0, 100);
  EXPECT_EQ(after_dropout.ordered_groups.groups[0].id, "g2");
  EXPECT_EQ(findGroup(after_dropout, "g2").members, findGroup(result, "g2").members);
  EXPECT_EQ(findGroup(after_dropout, "g1").members, (std::vector<std::string>{"b"}));

  // Regrouping invalidates the cache, allowing fresh matching.
  auto regrouped = plan;
  ++regrouped.revision;
  auto regroup_positions = positions;
  for (auto& entry : regroup_positions) entry.second[1] *= -1;
  const auto fresh = assignFormationSlots(regrouped, result, regroup_positions,
      "line", 20, 200, 2, 0, 100);
  EXPECT_EQ(fresh.ordered_groups.groups[0].id, "g1");
}

TEST(SlotAssignment, DropoutReassignsSurvivorsWithinTheirOriginalGroup) {
  FormationGroupPlan plan;
  plan.group_size = 3;
  plan.groups = {group("g1", {"a", "b", "c"}), group("g2", {"d", "e", "f"})};
  std::map<std::string, Vec3> points{
      {"a", {0, 220, 100}}, {"b", {0, 200, 100}}, {"c", {0, 180, 100}},
      {"d", {0, -180, 100}}, {"e", {0, -200, 100}}, {"f", {0, -220, 100}}};
  const auto initial = assignFormationSlots(plan, {}, points,
      "line", 20, 400, 2, 0, 100);
  plan.groups[0].members = {"a", "c"};
  points.erase("b");
  std::swap(points["a"], points["c"]);
  std::swap(points["d"], points["f"]);
  const auto updated = assignFormationSlots(plan, initial, points,
      "line", 20, 400, 2, 0, 100);
  EXPECT_EQ(findGroup(updated, "g1").members, (std::vector<std::string>{"c", "a"}));
  EXPECT_EQ(findGroup(updated, "g2").members, findGroup(initial, "g2").members);
  EXPECT_EQ(findGroup(updated, "g1").nominal_size, 3u);
}

TEST(SlotAssignment, ShapeChangeRecomputesAndEmptyGroupsKeepTheirPlace) {
  FormationGroupPlan plan;
  plan.group_size = 1;
  plan.groups = {group("g1", {"u1"}), group("g2", {"u2"})};
  const std::map<std::string, Vec3> positions{
      {"u1", {40, -40, 100}}, {"u2", {-40, 40, 100}}};
  const auto line = assignFormationSlots(plan, {}, positions,
      "line", 80, 0, 2, 0, 100);
  const auto column = assignFormationSlots(plan, line, positions,
      "column", 80, 0, 2, 0, 100);
  EXPECT_EQ(line.ordered_groups.groups[0].id, "g2");
  EXPECT_EQ(column.ordered_groups.groups[0].id, "g1");
  auto reduced = plan;
  reduced.groups[1].members.clear();
  const auto dropout = assignFormationSlots(reduced, line,
      {{"u1", {-40, 40, 100}}}, "line", 80, 0, 2, 0, 100);
  EXPECT_EQ(dropout.ordered_groups.groups[0].id, "g2");
  EXPECT_TRUE(dropout.ordered_groups.groups[0].members.empty());
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
