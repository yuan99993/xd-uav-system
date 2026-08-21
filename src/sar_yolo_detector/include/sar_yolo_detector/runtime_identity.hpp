#pragma once

#include <chrono>
#include <cstdint>
#include <iomanip>
#include <random>
#include <sstream>
#include <string>

#include <ros/ros.h>

#include <sar_yolo_detector/PerceptionIdentity.h>

namespace sar_yolo_detector {

inline std::string generateSessionUuid(const std::string &uav_id) {
  std::random_device random_device;
  std::mt19937_64 generator(
      (static_cast<std::uint64_t>(random_device()) << 32U) ^
      static_cast<std::uint64_t>(
          std::chrono::high_resolution_clock::now().time_since_epoch().count()));
  std::uniform_int_distribution<std::uint64_t> distribution;
  const std::uint64_t high = distribution(generator);
  const std::uint64_t low = distribution(generator);
  std::ostringstream stream;
  stream << (uav_id.empty() ? "uav" : uav_id) << '-' << std::hex
         << std::setfill('0') << std::setw(16) << high << std::setw(16) << low;
  return stream.str();
}

inline PerceptionIdentity loadPerceptionIdentity(
    ros::NodeHandle *private_node, const std::string &default_sensor_id,
    const std::string &default_profile,
    const std::string &default_coordinate_contract) {
  PerceptionIdentity identity;
  private_node->param("mission_id", identity.mission_id,
                      std::string("mission_unset"));
  private_node->param("uav_id", identity.uav_id, std::string("uav_unset"));
  private_node->param("session_uuid", identity.session_uuid, std::string());
  private_node->param("sensor_id", identity.sensor_id, default_sensor_id);
  private_node->param("profile", identity.profile, default_profile);
  private_node->param("model_version", identity.model_version,
                      std::string("unknown"));
  private_node->param("model_sha256", identity.model_sha256, std::string());
  private_node->param("calibration_version", identity.calibration_version,
                      std::string("unversioned"));
  private_node->param("coordinate_contract", identity.coordinate_contract,
                      default_coordinate_contract);
  if (identity.session_uuid.empty())
    identity.session_uuid = generateSessionUuid(identity.uav_id);
  return identity;
}

inline std::string observationUuid(const PerceptionIdentity &identity,
                                   const std::uint32_t local_track_id) {
  return identity.mission_id + "/" + identity.uav_id + "/" +
         identity.session_uuid + "/" + std::to_string(local_track_id);
}

} // namespace sar_yolo_detector
