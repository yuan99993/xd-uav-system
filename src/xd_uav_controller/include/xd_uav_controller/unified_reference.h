#pragma once

#include <array>
#include <cstdint>
#include <string>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Header.h>

namespace xd_uav_controller {

using AxisMask = std::array<bool, 3>;

enum class ReferenceType : uint8_t {
  kUnknown = 0,
  kPositionTarget = 1,
  kTrajectory = 2,
  kPath = 3,
  kSimpleGoal = 4,
  kInternal = 5,
  kIdle = 6,
};

inline uint8_t referenceTypeValue(const ReferenceType type) {
  return static_cast<uint8_t>(type);
}

struct UnifiedReference {
  ReferenceType type{ReferenceType::kUnknown};
  std::string source;
  std_msgs::Header header;
  geometry_msgs::Point position;
  geometry_msgs::Vector3 velocity;
  geometry_msgs::Vector3 acceleration;
  geometry_msgs::Vector3 jerk;
  double yaw{0.0};
  double yaw_rate{0.0};
  AxisMask use_position{{false, false, false}};
  AxisMask use_velocity{{false, false, false}};
  AxisMask use_acceleration{{false, false, false}};
  bool use_jerk{false};
  bool use_yaw{false};
  bool use_yaw_rate{false};
  bool trajectory_reference{false};
  bool path_reference{false};
};

}  // namespace xd_uav_controller
