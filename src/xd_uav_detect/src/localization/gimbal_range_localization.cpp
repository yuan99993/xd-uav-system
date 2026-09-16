#include <xd_uav_detect/localization/gimbal_range_localization.hpp>

#include <algorithm>
#include <cmath>

namespace xd_uav_detect {

GimbalRangeLocalization::GimbalRangeLocalization(
    const GimbalRangeConfig& config)
    : config_(config) {
  config_.bbox_gate_margin_px = std::max(0.0, config_.bbox_gate_margin_px);
  config_.range_stddev_m = std::max(0.0, config_.range_stddev_m);
  config_.angular_stddev_rad = std::max(0.0, config_.angular_stddev_rad);
  config_.position_stddev_m = std::max(0.0, config_.position_stddev_m);
}

GimbalRangeResult GimbalRangeLocalization::estimate(
    const double range_m, const std::vector<PixelBox>& boxes,
    const CameraIntrinsics& intrinsics,
    const Eigen::Matrix4d& camera_from_laser,
    const Eigen::Matrix4d& body_from_laser) const {
  GimbalRangeResult result;
  if (!std::isfinite(range_m) || range_m <= 0.0 || boxes.empty() ||
      !intrinsics.valid() || !camera_from_laser.array().isFinite().all() ||
      !body_from_laser.array().isFinite().all()) {
    return result;
  }

  const Eigen::Vector4d endpoint_laser(range_m, 0.0, 0.0, 1.0);
  const Eigen::Vector4d endpoint_camera = camera_from_laser * endpoint_laser;
  if (!endpoint_camera.head<3>().array().isFinite().all() ||
      endpoint_camera.z() <= 1e-6) {
    return result;
  }
  result.laser_pixel.x() =
      intrinsics.fx * endpoint_camera.x() / endpoint_camera.z() + intrinsics.cx;
  result.laser_pixel.y() =
      intrinsics.fy * endpoint_camera.y() / endpoint_camera.z() + intrinsics.cy;
  if (!result.laser_pixel.array().isFinite().all()) return result;

  std::size_t match_count = 0;
  for (std::size_t index = 0; index < boxes.size(); ++index) {
    if (!boxes[index].valid()) continue;
    const double margin = config_.bbox_gate_margin_px;
    if (!config_.require_laser_axis_in_bbox ||
        (result.laser_pixel.x() >= boxes[index].x_min - margin &&
        result.laser_pixel.x() <= boxes[index].x_max + margin &&
        result.laser_pixel.y() >= boxes[index].y_min - margin &&
        result.laser_pixel.y() <= boxes[index].y_max + margin)) {
      result.selected_box_index = index;
      ++match_count;
    }
  }
  if (match_count != 1) return result;

  const Eigen::Vector4d endpoint_body = body_from_laser * endpoint_laser;
  result.position_body_flu = endpoint_body.head<3>();
  if (!result.position_body_flu.array().isFinite().all()) return result;

  const double transverse_stddev = std::hypot(
      range_m * config_.angular_stddev_rad, config_.position_stddev_m);
  const double radial_stddev = std::hypot(
      config_.range_stddev_m, config_.position_stddev_m);
  Eigen::Matrix3d covariance_laser = Eigen::Matrix3d::Zero();
  covariance_laser(0, 0) = radial_stddev * radial_stddev;
  covariance_laser(1, 1) = transverse_stddev * transverse_stddev;
  covariance_laser(2, 2) = transverse_stddev * transverse_stddev;
  const Eigen::Matrix3d body_from_laser_rotation =
      body_from_laser.block<3, 3>(0, 0);
  result.covariance_body_flu = body_from_laser_rotation * covariance_laser *
      body_from_laser_rotation.transpose();
  result.valid = result.covariance_body_flu.array().isFinite().all();
  return result;
}

}  // namespace xd_uav_detect
