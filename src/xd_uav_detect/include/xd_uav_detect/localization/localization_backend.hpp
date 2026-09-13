#pragma once

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>

#include <xd_uav_detect/localization/gimbal_range_localization.hpp>
#include <xd_uav_detect/localization/ground_plane_projection.hpp>
#include <xd_uav_detect/localization/lidar_camera_fusion.hpp>

namespace xd_uav_detect {

enum class LocalizationBackendKind {
  kLidarCamera,
  kCameraGroundPlane,
  kGimbalLaserRange,
};

struct LocalizationRequirements {
  bool point_cloud{false};
  bool laser_range{false};
  bool world_from_camera{false};
};

struct LocalizationBackendConfig {
  FusionConfig lidar_camera;
  GroundPlaneConfig camera_ground_plane;
  GimbalRangeConfig gimbal_laser_range;
};

struct LocalizationInput {
  std::vector<PixelBox> boxes;
  CameraIntrinsics intrinsics;
  std::vector<LidarPoint> points;
  double range_m{0.0};
  Eigen::Matrix4d camera_from_sensor{Eigen::Matrix4d::Identity()};
  Eigen::Matrix4d body_from_sensor{Eigen::Matrix4d::Identity()};
  Eigen::Matrix4d world_from_camera{Eigen::Matrix4d::Identity()};
};

struct LocalizationEstimate {
  bool valid{false};
  Eigen::Vector3d position_body_flu{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d covariance_body_flu{Eigen::Matrix3d::Zero()};
};

class LocalizationBackend {
 public:
  virtual ~LocalizationBackend() = default;

  virtual LocalizationBackendKind kind() const = 0;
  virtual const char* name() const = 0;
  virtual LocalizationRequirements requirements() const = 0;
  virtual std::vector<LocalizationEstimate> localize(
      const LocalizationInput& input) const = 0;
};

// Accepts the three canonical method names and the historical aliases used by
// existing configurations. The returned string is always canonical.
std::string canonicalLocalizationMethod(const std::string& method,
                                        bool* deprecated_alias = nullptr);

std::unique_ptr<LocalizationBackend> makeLocalizationBackend(
    const std::string& method, const LocalizationBackendConfig& config,
    bool* deprecated_alias = nullptr);

}  // namespace xd_uav_detect
