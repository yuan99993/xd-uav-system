#include <xd_uav_detect/lidar_camera_fusion.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

namespace xd_uav_detect {

namespace {

struct ProjectedPoint {
  double depth{0.0};
  Eigen::Vector3d body{Eigen::Vector3d::Zero()};
};

double median(std::vector<double>* values) {
  if (values == nullptr || values->empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const std::size_t middle = values->size() / 2;
  std::nth_element(values->begin(), values->begin() + middle, values->end());
  double result = (*values)[middle];
  if (values->size() % 2 == 0) {
    const auto lower = std::max_element(values->begin(),
                                        values->begin() + middle);
    result = 0.5 * (result + *lower);
  }
  return result;
}

bool finiteMatrix(const Eigen::Matrix4d& matrix) {
  return matrix.array().isFinite().all();
}

}  // namespace

bool CameraIntrinsics::valid() const {
  return std::isfinite(fx) && std::isfinite(fy) && std::isfinite(cx) &&
      std::isfinite(cy) && fx > 0.0 && fy > 0.0 && width > 0 && height > 0;
}

bool PixelBox::valid() const {
  return std::isfinite(x_min) && std::isfinite(y_min) &&
      std::isfinite(x_max) && std::isfinite(y_max) &&
      x_max > x_min && y_max > y_min;
}

LidarCameraFusion::LidarCameraFusion(const FusionConfig& requested)
    : config_(requested) {
  config_.minimum_cluster_points = std::max(1, config_.minimum_cluster_points);
  config_.minimum_range_m = std::max(0.0, config_.minimum_range_m);
  config_.maximum_range_m = std::max(
      config_.minimum_range_m + 1e-3, config_.maximum_range_m);
  config_.box_shrink_ratio = std::max(
      0.0, std::min(0.45, config_.box_shrink_ratio));
  config_.depth_cluster_tolerance_m = std::max(
      0.01, config_.depth_cluster_tolerance_m);
}

FusedPosition LidarCameraFusion::estimate(
    const std::vector<LidarPoint>& points, const PixelBox& requested_box,
    const CameraIntrinsics& intrinsics,
    const Eigen::Matrix4d& camera_from_lidar,
    const Eigen::Matrix4d& body_from_lidar) const {
  FusedPosition result;
  if (!requested_box.valid() || !intrinsics.valid() ||
      !finiteMatrix(camera_from_lidar) || !finiteMatrix(body_from_lidar)) {
    return result;
  }

  const double shrink_x = config_.box_shrink_ratio *
      (requested_box.x_max - requested_box.x_min);
  const double shrink_y = config_.box_shrink_ratio *
      (requested_box.y_max - requested_box.y_min);
  const double x_min = requested_box.x_min + shrink_x;
  const double x_max = requested_box.x_max - shrink_x;
  const double y_min = requested_box.y_min + shrink_y;
  const double y_max = requested_box.y_max - shrink_y;

  std::vector<ProjectedPoint> inside;
  inside.reserve(points.size() / 8 + 1);
  for (const auto& point : points) {
    if (!point.position_lidar.array().isFinite().all()) continue;
    const double range = point.position_lidar.norm();
    if (range < config_.minimum_range_m || range > config_.maximum_range_m) {
      continue;
    }
    const Eigen::Vector4d homogeneous(
        point.position_lidar.x(), point.position_lidar.y(),
        point.position_lidar.z(), 1.0);
    const Eigen::Vector4d camera = camera_from_lidar * homogeneous;
    if (!camera.array().isFinite().all() || camera.z() <= 1e-4) continue;
    const double u = intrinsics.fx * camera.x() / camera.z() + intrinsics.cx;
    const double v = intrinsics.fy * camera.y() / camera.z() + intrinsics.cy;
    if (u < x_min || u > x_max || v < y_min || v > y_max) continue;
    const Eigen::Vector4d body = body_from_lidar * homogeneous;
    if (!body.array().isFinite().all()) continue;
    inside.push_back({camera.z(), body.head<3>()});
  }
  if (inside.size() < static_cast<std::size_t>(config_.minimum_cluster_points)) {
    return result;
  }

  std::sort(inside.begin(), inside.end(),
            [](const ProjectedPoint& lhs, const ProjectedPoint& rhs) {
              return lhs.depth < rhs.depth;
            });
  std::size_t cluster_begin = 0;
  std::size_t cluster_end = 0;
  bool found_cluster = false;
  for (std::size_t begin = 0; begin < inside.size();) {
    std::size_t end = begin + 1;
    while (end < inside.size() &&
           inside[end].depth - inside[end - 1].depth <=
               config_.depth_cluster_tolerance_m) {
      ++end;
    }
    if (end - begin >=
        static_cast<std::size_t>(config_.minimum_cluster_points)) {
      cluster_begin = begin;
      cluster_end = end;
      found_cluster = true;
      break;  // The nearest sufficiently populated depth cluster is foreground.
    }
    begin = end;
  }
  if (!found_cluster) return result;

  std::array<std::vector<double>, 3> axes;
  for (std::size_t index = cluster_begin; index < cluster_end; ++index) {
    for (int axis = 0; axis < 3; ++axis) {
      axes[axis].push_back(inside[index].body[axis]);
    }
  }
  for (int axis = 0; axis < 3; ++axis) {
    result.position_body_flu[axis] = median(&axes[axis]);
  }
  result.supporting_points = cluster_end - cluster_begin;
  if (!result.position_body_flu.array().isFinite().all()) return FusedPosition();

  if (result.supporting_points > 1) {
    for (std::size_t index = cluster_begin; index < cluster_end; ++index) {
      const Eigen::Vector3d residual =
          inside[index].body - result.position_body_flu;
      result.covariance_body_flu += residual * residual.transpose();
    }
    result.covariance_body_flu /=
        static_cast<double>(result.supporting_points - 1);
  } else {
    result.covariance_body_flu = 0.25 * Eigen::Matrix3d::Identity();
  }
  result.valid = result.covariance_body_flu.array().isFinite().all();
  return result;
}

}  // namespace xd_uav_detect
