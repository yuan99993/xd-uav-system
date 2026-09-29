#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>

#include <mavros_msgs/PositionTarget.h>
#include <ros/time.h>

#include <xd_uav_controller/unified_reference.h>

namespace xd_uav_controller {

enum class SetpointOutputType : uint8_t {
  kRawAttitude = 0,
  kRawLocal = 1,
};

inline SetpointOutputType setpointOutputTypeForReference(
    const ReferenceType type,
    const std::string& position_target_type,
    const std::string& path_type,
    const std::string& trajectory_type,
    const std::string& internal_type = "raw_attitude") {
  const std::string* configured_type = nullptr;
  switch (type) {
    case ReferenceType::kPositionTarget:
    case ReferenceType::kSimpleGoal:
      configured_type = &position_target_type;
      break;
    case ReferenceType::kPath:
      configured_type = &path_type;
      break;
    case ReferenceType::kTrajectory:
      configured_type = &trajectory_type;
      break;
    case ReferenceType::kUnknown:
      return SetpointOutputType::kRawAttitude;
    case ReferenceType::kInternal:
    case ReferenceType::kIdle:
      configured_type = &internal_type;
      break;
  }
  return configured_type != nullptr && *configured_type == "raw_local"
             ? SetpointOutputType::kRawLocal
             : SetpointOutputType::kRawAttitude;
}

inline uint8_t setpointOutputTypeValue(const SetpointOutputType type) {
  return static_cast<uint8_t>(type);
}

inline bool makeRawLocalTarget(
    const UnifiedReference& reference,
    const ros::Time& stamp,
    mavros_msgs::PositionTarget* target,
    std::string* reason,
    const bool fixedwing_position_only = false) {
  if (target == nullptr || reason == nullptr) {
    return false;
  }
  if (reference.header.frame_id.empty()) {
    *reason = "raw_local参考的frame_id不能为空";
    return false;
  }

  const bool has_spatial_target =
      reference.use_position[0] || reference.use_position[1] ||
      reference.use_position[2] || reference.use_velocity[0] ||
      reference.use_velocity[1] || reference.use_velocity[2] ||
      reference.use_acceleration[0] ||
      reference.use_acceleration[1] ||
      reference.use_acceleration[2];
  if (!has_spatial_target) {
    *reason = "raw_local至少需要启用一个位置、速度或加速度轴";
    return false;
  }
  if (fixedwing_position_only &&
      !(reference.use_position[0] && reference.use_position[1] &&
        reference.use_position[2])) {
    *reason =
        "固定翼PX4 raw_local当前只支持完整XYZ位置目标";
    return false;
  }

  const auto finite_selected_point = [](const geometry_msgs::Point& value,
                                        const AxisMask& mask) {
    return (!mask[0] || std::isfinite(value.x)) &&
           (!mask[1] || std::isfinite(value.y)) &&
           (!mask[2] || std::isfinite(value.z));
  };
  const auto finite_selected_vector = [](const geometry_msgs::Vector3& value,
                                         const AxisMask& mask) {
    return (!mask[0] || std::isfinite(value.x)) &&
           (!mask[1] || std::isfinite(value.y)) &&
           (!mask[2] || std::isfinite(value.z));
  };
  if (!finite_selected_point(reference.position,
                             reference.use_position) ||
      !finite_selected_vector(reference.velocity,
                              reference.use_velocity) ||
      !finite_selected_vector(reference.acceleration,
                              reference.use_acceleration) ||
      (reference.use_yaw && !std::isfinite(reference.yaw)) ||
      (reference.use_yaw_rate &&
       !std::isfinite(reference.yaw_rate))) {
    *reason = "raw_local启用的目标字段包含非法数值";
    return false;
  }

  mavros_msgs::PositionTarget result;
  result.header = reference.header;
  result.header.stamp = stamp;
  result.coordinate_frame =
      mavros_msgs::PositionTarget::FRAME_LOCAL_NED;

  constexpr std::array<uint16_t, 3> position_bits{{
      mavros_msgs::PositionTarget::IGNORE_PX,
      mavros_msgs::PositionTarget::IGNORE_PY,
      mavros_msgs::PositionTarget::IGNORE_PZ}};
  constexpr std::array<uint16_t, 3> velocity_bits{{
      mavros_msgs::PositionTarget::IGNORE_VX,
      mavros_msgs::PositionTarget::IGNORE_VY,
      mavros_msgs::PositionTarget::IGNORE_VZ}};
  constexpr std::array<uint16_t, 3> acceleration_bits{{
      mavros_msgs::PositionTarget::IGNORE_AFX,
      mavros_msgs::PositionTarget::IGNORE_AFY,
      mavros_msgs::PositionTarget::IGNORE_AFZ}};
  // Match MAVROS setpoint_position/local for multirotor takeoff and hover:
  // send a position and heading target, leaving velocity/acceleration
  // control to PX4. The controller's internal takeoff reference also carries
  // feed-forward terms for raw-attitude backends; they must not leak into the
  // raw-local path when the requested behavior is a position hold.
  const bool position_yaw_hold =
      !fixedwing_position_only &&
      (reference.type == ReferenceType::kIdle ||
       (reference.type == ReferenceType::kInternal &&
        reference.source == "takeoff"));
  uint16_t type_mask = 0U;
  for (std::size_t axis = 0; axis < 3; ++axis) {
    if (!reference.use_position[axis]) {
      type_mask |= position_bits[axis];
    }
    if (position_yaw_hold || !reference.use_velocity[axis]) {
      type_mask |= velocity_bits[axis];
    }
    if (position_yaw_hold || !reference.use_acceleration[axis]) {
      type_mask |= acceleration_bits[axis];
    }
  }
  if (!reference.use_yaw) {
    type_mask |= mavros_msgs::PositionTarget::IGNORE_YAW;
  }
  if (!reference.use_yaw_rate) {
    type_mask |= mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
  }
  if (fixedwing_position_only) {
    type_mask = mavros_msgs::PositionTarget::IGNORE_VX |
                mavros_msgs::PositionTarget::IGNORE_VY |
                mavros_msgs::PositionTarget::IGNORE_VZ |
                mavros_msgs::PositionTarget::IGNORE_AFX |
                mavros_msgs::PositionTarget::IGNORE_AFY |
                mavros_msgs::PositionTarget::IGNORE_AFZ |
                mavros_msgs::PositionTarget::IGNORE_YAW |
                mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
  }
  result.type_mask = type_mask;
  result.position = reference.position;
  result.velocity = reference.velocity;
  result.acceleration_or_force = reference.acceleration;
  result.yaw = reference.yaw;
  result.yaw_rate = reference.yaw_rate;
  *target = result;
  reason->clear();
  return true;
}

}  // namespace xd_uav_controller
