#pragma once

#include <array>
#include <vector>

#include <Eigen/Core>

namespace xd_uav_detect {

struct FusionConfig {
  int minimum_cluster_points{3};
  double minimum_range_m{0.30};
  double maximum_range_m{120.0};
  double box_shrink_ratio{0.10};
  double depth_cluster_tolerance_m{0.75};
};

struct CameraIntrinsics {
  double fx{0.0};
  double fy{0.0};
  double cx{0.0};
  double cy{0.0};
  unsigned int width{0};
  unsigned int height{0};

  bool valid() const;
};

struct PixelBox {
  double x_min{0.0};
  double y_min{0.0};
  double x_max{0.0};
  double y_max{0.0};

  bool valid() const;
};

struct LidarPoint {
  Eigen::Vector3d position_lidar{Eigen::Vector3d::Zero()};
};

struct FusedPosition {
  bool valid{false};
  std::size_t supporting_points{0};
  Eigen::Vector3d position_body_flu{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d covariance_body_flu{Eigen::Matrix3d::Zero()};
};

class LidarCameraFusion {
 public:
  explicit LidarCameraFusion(const FusionConfig& config = FusionConfig());

  FusedPosition estimate(
      const std::vector<LidarPoint>& points,
      const PixelBox& box,
      const CameraIntrinsics& intrinsics,
      const Eigen::Matrix4d& camera_from_lidar,
      const Eigen::Matrix4d& body_from_lidar) const;

 private:
  FusionConfig config_;
};

}  // namespace xd_uav_detect
