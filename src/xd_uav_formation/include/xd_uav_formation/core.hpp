#pragma once

#include <array>
#include <cstdint>
#include <cstddef>
#include <string>
#include <utility>
#include <vector>

namespace xd_uav_formation {

using Vec3 = std::array<double, 3>;
using Slot = std::array<double, 4>;

struct PathResult {
  Vec3 position;
  Vec3 velocity;
  double turn_rate;
};

struct TurnReference {
  double heading{0.0};
  double rate{0.0};
  double acceleration{0.0};
};

struct HorizontalMotion {
  Vec3 velocity{0.0, 0.0, 0.0};
  Vec3 acceleration{0.0, 0.0, 0.0};
  double course_rate{0.0};
};

struct FormationTurnPlan {
  double speed{0.0};
  double turn_rate{0.0};
  double lateral_velocity{0.0};
};

// Raise the common speed to protect the slowest slot before reducing turn
// rate. All nominal slots retain forward motion and respect the speed cap.
FormationTurnPlan planFormationTurn(const std::vector<Vec3>& offsets,
    double cruise_speed, double minimum_speed, double maximum_speed,
    double requested_rate, double lateral_velocity);

// Bounded angular acceleration prevents a turn-rate step from demanding an
// instantaneous change in the inner/outer aircraft's tangential speeds.
TurnReference advanceTurnReference(const TurnReference& previous,
    double desired_heading, double desired_rate, double dt,
    double maximum_rate, double maximum_acceleration);
HorizontalMotion slewHorizontalCorrection(const Vec3& previous,
    const Vec3& desired, double dt, double maximum_acceleration);
HorizontalMotion horizontalMotion(const Vec3& velocity, const Vec3& acceleration);
HorizontalMotion limitHorizontalMotion(const HorizontalMotion& motion,
    double minimum_speed, double maximum_speed, double fallback_heading);
HorizontalMotion turningSlotMotion(const Vec3& center_velocity,
    const Vec3& center_acceleration, double heading, const Vec3& offset,
    double turn_rate, double turn_acceleration);

Vec3 validateOffset(const Vec3& offset, double minimum_distance);
std::vector<Vec3> formationOffsets(const std::string& shape,
                                   std::size_t follower_count,
                                   double spacing);
Slot slotTarget(const Vec3& position, double yaw, const Vec3& offset,
                double yaw_offset = 0.0);
Vec3 slotVelocity(const Vec3& velocity, const Vec3& acceleration,
                  double heading, const Vec3& offset,
                  double max_turn_rate = 0.0, bool limit_turn_rate = false,
                  double turn_rate = 0.0, bool fixed_turn_rate = false);
Vec3 catchupVelocity(const Vec3& base_velocity, const Vec3& slot_position,
                     const Vec3& own_position, double gain,
                     double max_correction, bool limit_correction,
                     double max_horizontal_speed, bool limit_speed,
                     double min_horizontal_speed = 0.0);
Vec3 virtualCenter(const Vec3& position, double heading,
                   const Vec3& offset);
std::pair<Vec3, double> sharedCenter(const std::vector<Vec3>& centers);
std::vector<Vec3> acquisitionSlots(const std::vector<Vec3>& positions,
                                   const std::vector<Vec3>& offsets,
                                   double heading, double altitude);
// Hungarian assignment: result[i] is the unique slot for aircraft i. Minimize
// total horizontal distance; altitude is handled by the acquisition tiers.
std::vector<std::size_t> minimumDistanceAssignment(
    const std::vector<Vec3>& positions, const std::vector<Vec3>& slots);
std::vector<std::string> ringNeighbors(
    const std::vector<std::string>& members, const std::string& own_name);
std::vector<Vec3> roundedPath(const std::vector<Vec3>& waypoints,
                              double radius, bool loop);
std::pair<double, double> projectPathProgress(
    const Vec3& position, const std::vector<Vec3>& waypoints, bool loop);
PathResult pathReferenceState(const Vec3& center,
                              const std::vector<Vec3>& waypoints,
                              double speed, double lookahead, bool loop,
                              double progress_offset = 0.0);
Vec3 consensusVelocity(const Vec3& path_velocity,
                       const Vec3& desired_center,
                       const Vec3& own_center,
                       const std::vector<Vec3>& neighbor_centers,
                       double gain, double convergence_time,
                       double max_correction, bool limit_correction);
std::vector<double> consensusSpeedTargets(
    double path_speed, const std::vector<double>& progress_errors,
    const std::vector<double>& member_speeds, double gain,
    double max_speed_adjustment, double min_speed, double deadband = 1.0);
bool blockingAction(uint8_t action);

}  // namespace xd_uav_formation
