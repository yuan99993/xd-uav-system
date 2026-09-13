#pragma once

#include <cstddef>
#include <limits>
#include <vector>

#include <Eigen/Core>

#include <xd_uav_detect/localization/lidar_camera_fusion.hpp>

namespace xd_uav_detect {

struct GimbalRangeConfig {
  bool require_laser_axis_in_bbox{true};
  double bbox_gate_margin_px{4.0};
  double range_stddev_m{0.20};
  double angular_stddev_rad{0.003};
  double position_stddev_m{0.10};
};

struct GimbalRangeResult {
  bool valid{false};
  std::size_t selected_box_index{std::numeric_limits<std::size_t>::max()};
  Eigen::Vector2d laser_pixel{Eigen::Vector2d::Zero()};
  Eigen::Vector3d position_body_flu{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d covariance_body_flu{Eigen::Matrix3d::Zero()};
};

// Localizes the endpoint of a single-beam Range sensor. Both transforms use
// ROS frame conventions; the Range beam is the +X axis of laser_frame.
// Exactly one image box must cover the projected endpoint when gating is
// enabled; with gating disabled, exactly one valid box must exist.
class GimbalRangeLocalization {
 public:
  explicit GimbalRangeLocalization(const GimbalRangeConfig& config);

  GimbalRangeResult estimate(
      double range_m, const std::vector<PixelBox>& boxes,
      const CameraIntrinsics& intrinsics,
      const Eigen::Matrix4d& camera_from_laser,
      const Eigen::Matrix4d& body_from_laser) const;

 private:
  GimbalRangeConfig config_;
};

}  // namespace xd_uav_detect
