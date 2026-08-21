#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include <xd_uav_detect/ground_plane_projection.hpp>

namespace {

xd_uav_detect::CameraIntrinsics intrinsics() {
  xd_uav_detect::CameraIntrinsics value;
  value.fx = 100.0;
  value.fy = 100.0;
  value.cx = 320.0;
  value.cy = 240.0;
  value.width = 640;
  value.height = 480;
  return value;
}

xd_uav_detect::PixelBox boxAt(const double u, const double v) {
  xd_uav_detect::PixelBox box;
  box.x_min = u - 5.0;
  box.x_max = u + 5.0;
  box.y_min = v - 5.0;
  box.y_max = v + 5.0;
  return box;
}

Eigen::Matrix4d downwardCameraAt(const double altitude) {
  Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
  // Camera optical +Z points toward world -Z.
  transform.block<3, 3>(0, 0) =
      Eigen::AngleAxisd(3.14159265358979323846,
                        Eigen::Vector3d::UnitX()).toRotationMatrix();
  transform(2, 3) = altitude;
  return transform;
}

TEST(GroundPlaneProjection, CenterPixelIntersectsGroundBelowCamera) {
  xd_uav_detect::GroundPlaneConfig config;
  config.ground_plane_z_m = 0.0;
  config.position_stddev_m = 0.0;
  xd_uav_detect::GroundPlaneProjection projection(config);

  const auto result = projection.estimate(
      boxAt(320.0, 240.0), intrinsics(), downwardCameraAt(100.0),
      Eigen::Matrix4d::Identity());
  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(result.ray_range_m, 100.0, 1e-9);
  EXPECT_NEAR(result.position_body_flu.x(), 0.0, 1e-9);
  EXPECT_NEAR(result.position_body_flu.y(), 0.0, 1e-9);
  EXPECT_NEAR(result.position_body_flu.z(), 100.0, 1e-9);
  EXPECT_GT(result.covariance_body_flu.trace(), 0.0);
}

TEST(GroundPlaneProjection, OffCenterPixelProducesLateralOffset) {
  xd_uav_detect::GroundPlaneConfig config;
  config.ground_plane_z_m = 0.0;
  xd_uav_detect::GroundPlaneProjection projection(config);
  const auto result = projection.estimate(
      boxAt(330.0, 240.0), intrinsics(), downwardCameraAt(100.0),
      Eigen::Matrix4d::Identity());
  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(result.position_body_flu.x(), 10.0, 1e-9);
  EXPECT_NEAR(result.position_body_flu.y(), 0.0, 1e-9);
  EXPECT_NEAR(result.position_body_flu.z(), 100.0, 1e-9);
}

TEST(GroundPlaneProjection, RejectsPlaneBehindCamera) {
  xd_uav_detect::GroundPlaneProjection projection;
  Eigen::Matrix4d upward_camera = Eigen::Matrix4d::Identity();
  upward_camera(2, 3) = 100.0;
  const auto result = projection.estimate(
      boxAt(320.0, 240.0), intrinsics(), upward_camera,
      Eigen::Matrix4d::Identity());
  EXPECT_FALSE(result.valid);
}

TEST(GroundPlaneProjection, AppliesConfiguredAnchorWithinBox) {
  xd_uav_detect::GroundPlaneConfig config;
  config.box_anchor_y_ratio = 1.0;
  xd_uav_detect::GroundPlaneProjection projection(config);
  xd_uav_detect::PixelBox box;
  box.x_min = 310.0;
  box.x_max = 330.0;
  box.y_min = 220.0;
  box.y_max = 250.0;
  const auto result = projection.estimate(
      box, intrinsics(), downwardCameraAt(100.0),
      Eigen::Matrix4d::Identity());
  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(result.position_body_flu.y(), 10.0, 1e-9);
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
