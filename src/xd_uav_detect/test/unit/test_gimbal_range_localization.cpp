#include <gtest/gtest.h>

#include <cmath>

#include <Eigen/Geometry>

#include <xd_uav_detect/localization/gimbal_range_localization.hpp>

namespace {

xd_uav_detect::CameraIntrinsics intrinsics() {
  xd_uav_detect::CameraIntrinsics value;
  value.fx = 400.0;
  value.fy = 400.0;
  value.cx = 320.0;
  value.cy = 240.0;
  value.width = 640;
  value.height = 480;
  return value;
}

Eigen::Matrix4d opticalFromLaser() {
  Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
  transform.block<3, 3>(0, 0) <<
      0.0, -1.0, 0.0,
      0.0, 0.0, -1.0,
      1.0, 0.0, 0.0;
  return transform;
}

xd_uav_detect::PixelBox centerBox() {
  xd_uav_detect::PixelBox box;
  box.x_min = 300.0;
  box.y_min = 220.0;
  box.x_max = 340.0;
  box.y_max = 260.0;
  return box;
}

}  // namespace

TEST(GimbalRangeLocalization, ForwardBeamProducesBodyEndpointAndCovariance) {
  xd_uav_detect::GimbalRangeConfig config;
  config.range_stddev_m = 0.2;
  config.angular_stddev_rad = 0.01;
  config.position_stddev_m = 0.1;
  xd_uav_detect::GimbalRangeLocalization localizer(config);
  const auto result = localizer.estimate(
      50.0, {centerBox()}, intrinsics(), opticalFromLaser(),
      Eigen::Matrix4d::Identity());

  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.selected_box_index, 0u);
  EXPECT_NEAR(result.laser_pixel.x(), 320.0, 1e-9);
  EXPECT_NEAR(result.laser_pixel.y(), 240.0, 1e-9);
  EXPECT_NEAR(result.position_body_flu.x(), 50.0, 1e-9);
  EXPECT_NEAR(result.position_body_flu.y(), 0.0, 1e-9);
  EXPECT_NEAR(result.position_body_flu.z(), 0.0, 1e-9);
  EXPECT_NEAR(result.covariance_body_flu(0, 0), 0.05, 1e-9);
  EXPECT_NEAR(result.covariance_body_flu(1, 1), 0.26, 1e-9);
}

TEST(GimbalRangeLocalization, BodyRotationRotatesEndpoint) {
  xd_uav_detect::GimbalRangeLocalization localizer({});
  Eigen::Matrix4d body_from_laser = Eigen::Matrix4d::Identity();
  body_from_laser.block<3, 3>(0, 0) =
      Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const auto result = localizer.estimate(
      10.0, {centerBox()}, intrinsics(), opticalFromLaser(), body_from_laser);
  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(result.position_body_flu.x(), 0.0, 1e-9);
  EXPECT_NEAR(result.position_body_flu.y(), 10.0, 1e-9);
}

TEST(GimbalRangeLocalization, RejectsNoBoxAndAmbiguousBoxes) {
  xd_uav_detect::GimbalRangeLocalization localizer({});
  xd_uav_detect::PixelBox away = centerBox();
  away.x_min = 0.0;
  away.x_max = 100.0;
  EXPECT_FALSE(localizer.estimate(
      10.0, {away}, intrinsics(), opticalFromLaser(),
      Eigen::Matrix4d::Identity()).valid);
  EXPECT_FALSE(localizer.estimate(
      10.0, {centerBox(), centerBox()}, intrinsics(), opticalFromLaser(),
      Eigen::Matrix4d::Identity()).valid);
}

TEST(GimbalRangeLocalization, RejectsInvalidRangeAndBeamBehindCamera) {
  xd_uav_detect::GimbalRangeLocalization localizer({});
  EXPECT_FALSE(localizer.estimate(
      0.0, {centerBox()}, intrinsics(), opticalFromLaser(),
      Eigen::Matrix4d::Identity()).valid);
  EXPECT_FALSE(localizer.estimate(
      10.0, {centerBox()}, intrinsics(), Eigen::Matrix4d::Identity(),
      Eigen::Matrix4d::Identity()).valid);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
