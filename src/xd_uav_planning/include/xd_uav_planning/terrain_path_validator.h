#ifndef XD_UAV_PLANNING_TERRAIN_PATH_VALIDATOR_H
#define XD_UAV_PLANNING_TERRAIN_PATH_VALIDATOR_H

#include <cstddef>
#include <string>

#include <Eigen/Dense>

#include "terrain_navigation/path.h"
#include "terrain_navigation/terrain_map.h"

namespace xd_uav_planning {

struct TerrainPathLimits {
  double minimum_turning_radius_m{66.66667};
  double maximum_climb_angle_rad{0.15};
  double validation_sample_step_m{1.0};
  double maximum_sample_gap_m{10.0};
  double endpoint_tolerance_m{0.01};
  double surface_tolerance_m{1e-3};
  std::size_t maximum_output_points{10000};
};

struct TerrainPathValidation {
  bool valid{false};
  std::string reason{"not_validated"};
  std::size_t state_count{0};
  double minimum_lower_margin_m{0.0};
  double minimum_upper_margin_m{0.0};
  double maximum_sample_gap_m{0.0};
  double maximum_abs_curvature_inv_m{0.0};
  double maximum_abs_climb_angle_rad{0.0};
};

TerrainPathValidation validateTerrainPath(
    const ::Path& path, TerrainMap& terrain_map,
    const Eigen::Vector3d& expected_start,
    const Eigen::Vector3d& expected_goal,
    const TerrainPathLimits& limits);

}  // namespace xd_uav_planning

#endif  // XD_UAV_PLANNING_TERRAIN_PATH_VALIDATOR_H
