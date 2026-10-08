#include <gtest/gtest.h>

#include <limits>

#include <Eigen/Dense>

#include "terrain_navigation/path.h"
#include "terrain_navigation/terrain_map.h"
#include "xd_uav_planning/terrain_path_validator.h"

namespace {

State state(double x, double y, double z) {
  State value;
  value.position = Eigen::Vector3d(x, y, z);
  value.velocity = Eigen::Vector3d(1.0, 0.0, 0.0);
  value.attitude = Eigen::Vector4d(1.0, 0.0, 0.0, 0.0);
  return value;
}

TerrainMap mapWithCorridor() {
  TerrainMap terrain;
  auto& map = terrain.getGridMap();
  map.setGeometry(grid_map::Length(100.0, 100.0), 1.0,
                  grid_map::Position::Zero());
  map.add("distance_surface", 10.0);
  map.add("max_elevation", 30.0);
  return terrain;
}

::Path basicPath() {
  ::Path path;
  PathSegment segment;
  segment.curvature = 0.01;
  segment.flightpath_angle = 0.1;
  segment.states = {state(-10.0, 0.0, 20.0), state(-5.0, 0.0, 20.0),
                    state(0.0, 0.0, 20.0), state(5.0, 0.0, 20.0),
                    state(10.0, 0.0, 20.0)};
  path.segments.push_back(segment);
  return path;
}

xd_uav_planning::TerrainPathValidation validate(
    ::Path& path, TerrainMap& terrain,
    const xd_uav_planning::TerrainPathLimits& limits = {}) {
  return xd_uav_planning::validateTerrainPath(
      path, terrain, Eigen::Vector3d(-10.0, 0.0, 20.0),
      Eigen::Vector3d(10.0, 0.0, 20.0), limits);
}

TEST(TerrainPathValidator, AcceptsSafeFixedWingPath) {
  auto terrain = mapWithCorridor();
  auto path = basicPath();
  const auto result = validate(path, terrain);
  EXPECT_TRUE(result.valid) << result.reason;
  EXPECT_EQ("ok", result.reason);
  EXPECT_EQ(5u, result.state_count);
  EXPECT_DOUBLE_EQ(10.0, result.minimum_lower_margin_m);
  EXPECT_DOUBLE_EQ(10.0, result.minimum_upper_margin_m);
}

TEST(TerrainPathValidator, RejectsLowerAndUpperSurfaceViolations) {
  auto terrain = mapWithCorridor();
  xd_uav_planning::TerrainPathLimits limits;
  limits.maximum_sample_gap_m = 20.0;
  auto lower_path = basicPath();
  lower_path.segments.front().states[2].position.z() = 9.0;
  EXPECT_EQ("terrain_clearance_violation",
            validate(lower_path, terrain, limits).reason);

  auto upper_path = basicPath();
  upper_path.segments.front().states[2].position.z() = 31.0;
  EXPECT_EQ("maximum_altitude_violation",
            validate(upper_path, terrain, limits).reason);
}

TEST(TerrainPathValidator, SamplesBetweenPlannerStates) {
  auto terrain = mapWithCorridor();
  auto path = basicPath();
  terrain.getGridMap().atPosition(
      "max_elevation", grid_map::Position(-2.0, 0.0)) = 15.0;
  EXPECT_EQ("maximum_altitude_violation", validate(path, terrain).reason);
}

TEST(TerrainPathValidator, RejectsDynamicsAndSamplingViolations) {
  auto terrain = mapWithCorridor();
  auto curvature_path = basicPath();
  curvature_path.segments.front().curvature = 0.02;
  EXPECT_EQ("minimum_turning_radius_violation",
            validate(curvature_path, terrain).reason);

  auto climb_path = basicPath();
  climb_path.segments.front().flightpath_angle = 0.2;
  EXPECT_EQ("climb_angle_violation", validate(climb_path, terrain).reason);

  auto gap_path = basicPath();
  xd_uav_planning::TerrainPathLimits strict;
  strict.maximum_sample_gap_m = 4.0;
  EXPECT_EQ("terrain_path_sample_gap_exceeded",
            validate(gap_path, terrain, strict).reason);
}

TEST(TerrainPathValidator, RejectsEndpointAndFiniteValueViolations) {
  auto terrain = mapWithCorridor();
  auto endpoint_path = basicPath();
  endpoint_path.segments.front().states.back().position.x() = 9.0;
  EXPECT_EQ("terrain_path_goal_mismatch",
            validate(endpoint_path, terrain).reason);

  auto finite_path = basicPath();
  finite_path.segments.front().states[1].position.x() =
      std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ("terrain_path_not_finite", validate(finite_path, terrain).reason);
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
