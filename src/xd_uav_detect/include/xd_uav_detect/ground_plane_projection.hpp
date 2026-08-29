#pragma once

#include <Eigen/Core>

#include <xd_uav_detect/lidar_camera_fusion.hpp>

namespace xd_uav_detect {

// Parameters for locating a ground target by intersecting a calibrated camera
// ray with a horizontal plane in the configured world frame.
struct GroundPlaneConfig {
  double ground_plane_z_m{0.0};
  double minimum_range_m{1.0};
  double maximum_range_m{500.0};
  double minimum_ray_plane_angle_deg{5.0};
  double box_anchor_y_ratio{0.5};
  double pixel_stddev_px{2.0};
  double ground_height_stddev_m{1.0};
  double position_stddev_m{0.25};
};

struct GroundProjectionResult {
  bool valid{false};
  double ray_range_m{0.0};
  Eigen::Vector3d position_body_flu{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d covariance_body_flu{Eigen::Matrix3d::Zero()};
};

class GroundPlaneProjection {
 public:
  explicit GroundPlaneProjection(
      const GroundPlaneConfig& config = GroundPlaneConfig());

  // Both transforms must describe the same capture time and camera optical
  // frame. world_from_camera is used for the plane intersection;
  // body_from_camera returns the metric result in the ROS body FLU frame.
  GroundProjectionResult estimate(
      const PixelBox& box,
      const CameraIntrinsics& intrinsics,
      const Eigen::Matrix4d& world_from_camera,
      const Eigen::Matrix4d& body_from_camera) const;

 private:
  bool projectPixel(double u, double v, double ground_plane_z_m,
                    const CameraIntrinsics& intrinsics,
                    const Eigen::Matrix4d& world_from_camera,
                    const Eigen::Matrix4d& body_from_camera,
                    Eigen::Vector3d* position_body_flu,
                    double* ray_range_m) const;

  GroundPlaneConfig config_;
};

}  // namespace xd_uav_detect
