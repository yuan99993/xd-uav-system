#include <xd_uav_detect/ground_plane_projection.hpp>

#include <algorithm>
#include <cmath>

namespace xd_uav_detect {

namespace {

constexpr double kPi = 3.14159265358979323846;

bool finiteMatrix(const Eigen::Matrix4d& matrix) {
  return matrix.array().isFinite().all();
}

}  // namespace

GroundPlaneProjection::GroundPlaneProjection(
    const GroundPlaneConfig& requested)
    : config_(requested) {
  config_.minimum_range_m = std::max(0.0, config_.minimum_range_m);
  config_.maximum_range_m = std::max(
      config_.minimum_range_m + 1e-3, config_.maximum_range_m);
  config_.minimum_ray_plane_angle_deg = std::max(
      0.1, std::min(89.0, config_.minimum_ray_plane_angle_deg));
  config_.box_anchor_y_ratio = std::max(
      0.0, std::min(1.0, config_.box_anchor_y_ratio));
  config_.pixel_stddev_px = std::max(0.0, config_.pixel_stddev_px);
  config_.ground_height_stddev_m = std::max(
      0.0, config_.ground_height_stddev_m);
  config_.position_stddev_m = std::max(0.0, config_.position_stddev_m);
}

bool GroundPlaneProjection::projectPixel(
    const double u, const double v, const double ground_plane_z_m,
    const CameraIntrinsics& intrinsics,
    const Eigen::Matrix4d& world_from_camera,
    const Eigen::Matrix4d& body_from_camera,
    Eigen::Vector3d* position_body_flu, double* ray_range_m) const {
  if (position_body_flu == nullptr || ray_range_m == nullptr ||
      !intrinsics.valid() || !finiteMatrix(world_from_camera) ||
      !finiteMatrix(body_from_camera) || !std::isfinite(u) ||
      !std::isfinite(v) || !std::isfinite(ground_plane_z_m)) {
    return false;
  }

  Eigen::Vector3d ray_camera(
      (u - intrinsics.cx) / intrinsics.fx,
      (v - intrinsics.cy) / intrinsics.fy, 1.0);
  if (!ray_camera.array().isFinite().all() || ray_camera.norm() <= 1e-9) {
    return false;
  }
  ray_camera.normalize();

  Eigen::Vector3d ray_world =
      world_from_camera.block<3, 3>(0, 0) * ray_camera;
  if (!ray_world.array().isFinite().all() || ray_world.norm() <= 1e-9) {
    return false;
  }
  ray_world.normalize();
  const double minimum_vertical_component = std::sin(
      config_.minimum_ray_plane_angle_deg * kPi / 180.0);
  if (std::abs(ray_world.z()) < minimum_vertical_component) return false;

  const Eigen::Vector3d origin_world =
      world_from_camera.block<3, 1>(0, 3);
  const double range =
      (ground_plane_z_m - origin_world.z()) / ray_world.z();
  if (!std::isfinite(range) || range < config_.minimum_range_m ||
      range > config_.maximum_range_m) {
    return false;  // Includes intersections behind the camera.
  }

  const Eigen::Vector4d target_camera(
      range * ray_camera.x(), range * ray_camera.y(),
      range * ray_camera.z(), 1.0);
  const Eigen::Vector4d target_body = body_from_camera * target_camera;
  if (!target_body.array().isFinite().all()) return false;
  *position_body_flu = target_body.head<3>();
  *ray_range_m = range;
  return position_body_flu->array().isFinite().all();
}

GroundProjectionResult GroundPlaneProjection::estimate(
    const PixelBox& box, const CameraIntrinsics& intrinsics,
    const Eigen::Matrix4d& world_from_camera,
    const Eigen::Matrix4d& body_from_camera) const {
  GroundProjectionResult result;
  if (!box.valid() || !intrinsics.valid()) return result;

  const double u = 0.5 * (box.x_min + box.x_max);
  const double v = box.y_min + config_.box_anchor_y_ratio *
      (box.y_max - box.y_min);
  if (u < 0.0 || v < 0.0 || u > intrinsics.width ||
      v > intrinsics.height) {
    return result;
  }
  if (!projectPixel(u, v, config_.ground_plane_z_m, intrinsics,
                    world_from_camera, body_from_camera,
                    &result.position_body_flu, &result.ray_range_m)) {
    return result;
  }

  result.covariance_body_flu = config_.position_stddev_m *
      config_.position_stddev_m * Eigen::Matrix3d::Identity();
  auto accumulateSymmetricPerturbation = [&](
      const double plus_u, const double plus_v, const double plus_z,
      const double minus_u, const double minus_v, const double minus_z) {
    Eigen::Vector3d plus;
    Eigen::Vector3d minus;
    double unused_range = 0.0;
    if (projectPixel(plus_u, plus_v, plus_z, intrinsics,
                     world_from_camera, body_from_camera,
                     &plus, &unused_range) &&
        projectPixel(minus_u, minus_v, minus_z, intrinsics,
                     world_from_camera, body_from_camera,
                     &minus, &unused_range)) {
      // The samples are +/- one configured standard deviation, so the
      // half-difference is already a one-sigma displacement vector.
      const Eigen::Vector3d displacement = 0.5 * (plus - minus);
      result.covariance_body_flu +=
          displacement * displacement.transpose();
    }
  };
  if (config_.pixel_stddev_px > 0.0) {
    accumulateSymmetricPerturbation(
        u + config_.pixel_stddev_px, v, config_.ground_plane_z_m,
        u - config_.pixel_stddev_px, v, config_.ground_plane_z_m);
    accumulateSymmetricPerturbation(
        u, v + config_.pixel_stddev_px, config_.ground_plane_z_m,
        u, v - config_.pixel_stddev_px, config_.ground_plane_z_m);
  }
  if (config_.ground_height_stddev_m > 0.0) {
    accumulateSymmetricPerturbation(
        u, v, config_.ground_plane_z_m + config_.ground_height_stddev_m,
        u, v, config_.ground_plane_z_m - config_.ground_height_stddev_m);
  }

  result.valid = result.position_body_flu.array().isFinite().all() &&
      result.covariance_body_flu.array().isFinite().all();
  return result;
}

}  // namespace xd_uav_detect
