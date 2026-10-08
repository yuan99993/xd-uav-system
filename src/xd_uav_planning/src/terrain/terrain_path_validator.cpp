#include "xd_uav_planning/terrain_path_validator.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace xd_uav_planning {
namespace {

bool finiteState(const State& state) {
  return state.position.allFinite() && state.velocity.allFinite() &&
         state.attitude.allFinite();
}

bool validatePosition(const Eigen::Vector3d& position, TerrainMap& terrain_map,
                      const TerrainPathLimits& limits,
                      TerrainPathValidation* result) {
  auto& map = terrain_map.getGridMap();
  const Eigen::Vector2d horizontal = position.head<2>();
  if (!map.isInside(horizontal)) {
    result->reason = "terrain_path_outside_map";
    return false;
  }
  const double lower = map.atPosition("distance_surface", horizontal);
  const double upper = map.atPosition("max_elevation", horizontal);
  if (!std::isfinite(lower) || !std::isfinite(upper)) {
    result->reason = "terrain_surface_not_finite";
    return false;
  }
  const double lower_margin = position.z() - lower;
  const double upper_margin = upper - position.z();
  result->minimum_lower_margin_m =
      std::min(result->minimum_lower_margin_m, lower_margin);
  result->minimum_upper_margin_m =
      std::min(result->minimum_upper_margin_m, upper_margin);
  if (lower_margin < -limits.surface_tolerance_m) {
    result->reason = "terrain_clearance_violation";
    return false;
  }
  if (upper_margin < -limits.surface_tolerance_m) {
    result->reason = "maximum_altitude_violation";
    return false;
  }
  return true;
}

bool validateEdge(const Eigen::Vector3d& start, const Eigen::Vector3d& end,
                  TerrainMap& terrain_map, const TerrainPathLimits& limits,
                  TerrainPathValidation* result) {
  const double distance = (end - start).norm();
  result->maximum_sample_gap_m =
      std::max(result->maximum_sample_gap_m, distance);
  if (!std::isfinite(distance)) {
    result->reason = "terrain_path_not_finite";
    return false;
  }
  if (distance > limits.maximum_sample_gap_m) {
    result->reason = "terrain_path_sample_gap_exceeded";
    return false;
  }
  const std::size_t samples = std::max<std::size_t>(
      1, static_cast<std::size_t>(
             std::ceil(distance / limits.validation_sample_step_m)));
  for (std::size_t index = 1; index <= samples; ++index) {
    const double progress = static_cast<double>(index) /
                            static_cast<double>(samples);
    if (!validatePosition(start + progress * (end - start), terrain_map,
                          limits, result)) {
      return false;
    }
  }
  return true;
}

}  // namespace

TerrainPathValidation validateTerrainPath(
    const ::Path& path, TerrainMap& terrain_map,
    const Eigen::Vector3d& expected_start,
    const Eigen::Vector3d& expected_goal,
    const TerrainPathLimits& limits) {
  TerrainPathValidation result;
  result.minimum_lower_margin_m = std::numeric_limits<double>::infinity();
  result.minimum_upper_margin_m = std::numeric_limits<double>::infinity();

  auto& map = terrain_map.getGridMap();
  if (!map.exists("distance_surface") || !map.exists("max_elevation")) {
    result.reason = "terrain_safety_layers_missing";
    return result;
  }
  if (!(limits.minimum_turning_radius_m > 0.0) ||
      !(limits.maximum_climb_angle_rad > 0.0) ||
      !(limits.validation_sample_step_m > 0.0) ||
      !(limits.maximum_sample_gap_m > 0.0) ||
      limits.maximum_output_points < 2) {
    result.reason = "terrain_validation_limits_invalid";
    return result;
  }
  if (path.segments.empty()) {
    result.reason = "terrain_path_empty";
    return result;
  }

  bool have_previous = false;
  Eigen::Vector3d previous = Eigen::Vector3d::Zero();
  Eigen::Vector3d actual_start = Eigen::Vector3d::Zero();
  Eigen::Vector3d actual_goal = Eigen::Vector3d::Zero();
  for (const auto& segment : path.segments) {
    if (segment.states.empty()) {
      result.reason = "terrain_path_segment_empty";
      return result;
    }
    if (!std::isfinite(segment.curvature) ||
        !std::isfinite(segment.flightpath_angle)) {
      result.reason = "terrain_path_dynamics_not_finite";
      return result;
    }
    result.maximum_abs_curvature_inv_m = std::max(
        result.maximum_abs_curvature_inv_m, std::abs(segment.curvature));
    result.maximum_abs_climb_angle_rad = std::max(
        result.maximum_abs_climb_angle_rad,
        std::abs(segment.flightpath_angle));
    if (std::abs(segment.curvature) >
        (1.0 / limits.minimum_turning_radius_m) + 1e-6) {
      result.reason = "minimum_turning_radius_violation";
      return result;
    }
    if (std::abs(segment.flightpath_angle) >
        limits.maximum_climb_angle_rad + 1e-6) {
      result.reason = "climb_angle_violation";
      return result;
    }

    for (const auto& state : segment.states) {
      if (!finiteState(state)) {
        result.reason = "terrain_path_not_finite";
        return result;
      }
      if (result.state_count == 0) {
        actual_start = state.position;
        if (!validatePosition(state.position, terrain_map, limits, &result)) {
          return result;
        }
      } else if (!validateEdge(previous, state.position, terrain_map, limits,
                               &result)) {
        return result;
      }
      previous = state.position;
      actual_goal = state.position;
      have_previous = true;
      ++result.state_count;
      if (result.state_count > limits.maximum_output_points) {
        result.reason = "terrain_path_too_many_points";
        return result;
      }
    }
  }

  if (!have_previous || result.state_count < 2) {
    result.reason = "terrain_path_too_short";
    return result;
  }
  if ((actual_start - expected_start).norm() > limits.endpoint_tolerance_m) {
    result.reason = "terrain_path_start_mismatch";
    return result;
  }
  if ((actual_goal - expected_goal).norm() > limits.endpoint_tolerance_m) {
    result.reason = "terrain_path_goal_mismatch";
    return result;
  }

  result.valid = true;
  result.reason = "ok";
  return result;
}

}  // namespace xd_uav_planning
