#include <gtest/gtest.h>

#include <xd_uav_detect/lidar_camera_fusion.hpp>

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

xd_uav_detect::PixelBox centerBox() {
  xd_uav_detect::PixelBox box;
  box.x_min = 290.0;
  box.y_min = 210.0;
  box.x_max = 350.0;
  box.y_max = 270.0;
  return box;
}

TEST(LidarCameraFusion, ProjectsAndSelectsNearestForegroundCluster) {
  xd_uav_detect::FusionConfig config;
  config.minimum_cluster_points = 3;
  config.box_shrink_ratio = 0.0;
  config.depth_cluster_tolerance_m = 0.5;
  xd_uav_detect::LidarCameraFusion fusion(config);

  std::vector<xd_uav_detect::LidarPoint> points;
  // The test LiDAR frame is already the camera optical frame.
  points.push_back({Eigen::Vector3d(-0.10, 0.00, 5.0)});
  points.push_back({Eigen::Vector3d(0.00, 0.05, 5.1)});
  points.push_back({Eigen::Vector3d(0.10, -0.05, 4.9)});
  // Background points are inside the same 2D box but form a farther cluster.
  points.push_back({Eigen::Vector3d(-0.10, 0.00, 12.0)});
  points.push_back({Eigen::Vector3d(0.00, 0.05, 12.1)});
  points.push_back({Eigen::Vector3d(0.10, -0.05, 11.9)});

  const auto result = fusion.estimate(
      points, centerBox(), intrinsics(), Eigen::Matrix4d::Identity(),
      Eigen::Matrix4d::Identity());
  ASSERT_TRUE(result.valid);
  EXPECT_EQ(3U, result.supporting_points);
  EXPECT_NEAR(result.position_body_flu.x(), 0.0, 1e-9);
  EXPECT_NEAR(result.position_body_flu.y(), 0.0, 1e-9);
  EXPECT_NEAR(result.position_body_flu.z(), 5.0, 1e-9);
}

TEST(LidarCameraFusion, RejectsPointsOutsideTheDetectionBox) {
  xd_uav_detect::LidarCameraFusion fusion;
  std::vector<xd_uav_detect::LidarPoint> points = {
      {Eigen::Vector3d(5.0, 0.0, 5.0)},
      {Eigen::Vector3d(5.1, 0.0, 5.0)},
      {Eigen::Vector3d(4.9, 0.0, 5.0)}};
  const auto result = fusion.estimate(
      points, centerBox(), intrinsics(), Eigen::Matrix4d::Identity(),
      Eigen::Matrix4d::Identity());
  EXPECT_FALSE(result.valid);
}

TEST(LidarCameraFusion, AppliesConfiguredSensorTransforms) {
  xd_uav_detect::FusionConfig config;
  config.minimum_cluster_points = 1;
  config.box_shrink_ratio = 0.0;
  xd_uav_detect::LidarCameraFusion fusion(config);
  std::vector<xd_uav_detect::LidarPoint> points = {
      {Eigen::Vector3d(1.0, 2.0, 5.0)}};
  Eigen::Matrix4d camera_from_lidar = Eigen::Matrix4d::Identity();
  camera_from_lidar(0, 3) = -1.0;
  camera_from_lidar(1, 3) = -2.0;
  Eigen::Matrix4d body_from_lidar = Eigen::Matrix4d::Identity();
  body_from_lidar(0, 3) = 3.0;
  const auto result = fusion.estimate(
      points, centerBox(), intrinsics(), camera_from_lidar, body_from_lidar);
  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(result.position_body_flu.x(), 4.0, 1e-9);
  EXPECT_NEAR(result.position_body_flu.y(), 2.0, 1e-9);
  EXPECT_NEAR(result.position_body_flu.z(), 5.0, 1e-9);
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
