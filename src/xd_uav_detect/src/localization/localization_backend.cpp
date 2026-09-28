#include <xd_uav_detect/localization/localization_backend.hpp>

#include <algorithm>
#include <cctype>
#include <stdexcept>
#include <utility>

namespace xd_uav_detect {
namespace {

class LidarCameraBackend final : public LocalizationBackend {
 public:
  explicit LidarCameraBackend(const FusionConfig& config) : fusion_(config) {}

  LocalizationBackendKind kind() const override {
    return LocalizationBackendKind::kLidarCamera;
  }
  const char* name() const override { return "lidar_camera"; }
  LocalizationRequirements requirements() const override {
    LocalizationRequirements result;
    result.point_cloud = true;
    return result;
  }
  std::vector<LocalizationEstimate> localize(
      const LocalizationInput& input) const override {
    std::vector<LocalizationEstimate> output(input.boxes.size());
    for (std::size_t index = 0; index < input.boxes.size(); ++index) {
      const auto estimate = fusion_.estimate(
          input.points, input.boxes[index], input.intrinsics,
          input.camera_from_sensor, input.body_from_sensor);
      output[index].valid = estimate.valid;
      output[index].position_body_flu = estimate.position_body_flu;
      output[index].covariance_body_flu = estimate.covariance_body_flu;
    }
    return output;
  }

 private:
  LidarCameraFusion fusion_;
};

class CameraGroundPlaneBackend final : public LocalizationBackend {
 public:
  explicit CameraGroundPlaneBackend(const GroundPlaneConfig& config)
      : projection_(config) {}

  LocalizationBackendKind kind() const override {
    return LocalizationBackendKind::kCameraGroundPlane;
  }
  const char* name() const override { return "camera_ground_plane"; }
  LocalizationRequirements requirements() const override {
    LocalizationRequirements result;
    result.world_from_camera = true;
    return result;
  }
  std::vector<LocalizationEstimate> localize(
      const LocalizationInput& input) const override {
    std::vector<LocalizationEstimate> output(input.boxes.size());
    for (std::size_t index = 0; index < input.boxes.size(); ++index) {
      const auto estimate = projection_.estimate(
          input.boxes[index], input.intrinsics, input.world_from_camera,
          input.body_from_sensor);
      output[index].valid = estimate.valid;
      output[index].position_body_flu = estimate.position_body_flu;
      output[index].covariance_body_flu = estimate.covariance_body_flu;
    }
    return output;
  }

 private:
  GroundPlaneProjection projection_;
};

class GimbalLaserRangeBackend final : public LocalizationBackend {
 public:
  explicit GimbalLaserRangeBackend(const GimbalRangeConfig& config)
      : localization_(config) {}

  LocalizationBackendKind kind() const override {
    return LocalizationBackendKind::kGimbalLaserRange;
  }
  const char* name() const override { return "gimbal_laser_range"; }
  LocalizationRequirements requirements() const override {
    LocalizationRequirements result;
    result.laser_range = true;
    return result;
  }
  std::vector<LocalizationEstimate> localize(
      const LocalizationInput& input) const override {
    std::vector<LocalizationEstimate> output(input.boxes.size());
    const auto estimate = localization_.estimate(
        input.range_m, input.boxes, input.intrinsics,
        input.camera_from_sensor, input.body_from_sensor);
    if (estimate.valid && estimate.selected_box_index < output.size()) {
      auto& selected = output[estimate.selected_box_index];
      selected.valid = true;
      selected.position_body_flu = estimate.position_body_flu;
      selected.covariance_body_flu = estimate.covariance_body_flu;
    }
    return output;
  }

 private:
  GimbalRangeLocalization localization_;
};

}  // namespace

std::string canonicalLocalizationMethod(const std::string& method,
                                        bool* deprecated_alias) {
  std::string normalized = method;
  std::transform(normalized.begin(), normalized.end(), normalized.begin(),
                 [](const unsigned char value) {
                   return static_cast<char>(std::tolower(value));
                 });
  bool deprecated = false;
  if (normalized == "multirotor") {
    normalized = "lidar_camera";
    deprecated = true;
  } else if (normalized == "ground_plane" || normalized == "fixedwing") {
    normalized = "camera_ground_plane";
    deprecated = true;
  }
  if (normalized != "lidar_camera" &&
      normalized != "camera_ground_plane" &&
      normalized != "gimbal_laser_range") {
    throw std::invalid_argument(
        "localization method must be lidar_camera, camera_ground_plane, or "
        "gimbal_laser_range");
  }
  if (deprecated_alias != nullptr) *deprecated_alias = deprecated;
  return normalized;
}

std::unique_ptr<LocalizationBackend> makeLocalizationBackend(
    const std::string& method, const LocalizationBackendConfig& config,
    bool* deprecated_alias) {
  const std::string canonical =
      canonicalLocalizationMethod(method, deprecated_alias);
  if (canonical == "lidar_camera") {
    return std::unique_ptr<LocalizationBackend>(
        new LidarCameraBackend(config.lidar_camera));
  }
  if (canonical == "camera_ground_plane") {
    return std::unique_ptr<LocalizationBackend>(
        new CameraGroundPlaneBackend(config.camera_ground_plane));
  }
  return std::unique_ptr<LocalizationBackend>(
      new GimbalLaserRangeBackend(config.gimbal_laser_range));
}

}  // namespace xd_uav_detect
