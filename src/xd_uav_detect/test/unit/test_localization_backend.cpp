#include <gtest/gtest.h>

#include <stdexcept>

#include <xd_uav_detect/localization/localization_backend.hpp>

TEST(LocalizationBackend, CanonicalNamesAndRequirements) {
  xd_uav_detect::LocalizationBackendConfig config;
  bool deprecated = false;

  auto lidar = xd_uav_detect::makeLocalizationBackend(
      "lidar_camera", config, &deprecated);
  EXPECT_FALSE(deprecated);
  EXPECT_EQ(std::string("lidar_camera"), lidar->name());
  EXPECT_TRUE(lidar->requirements().point_cloud);
  EXPECT_FALSE(lidar->requirements().laser_range);

  auto ground = xd_uav_detect::makeLocalizationBackend(
      "fixedwing", config, &deprecated);
  EXPECT_TRUE(deprecated);
  EXPECT_EQ(std::string("camera_ground_plane"), ground->name());
  EXPECT_TRUE(ground->requirements().world_from_camera);

  auto gimbal = xd_uav_detect::makeLocalizationBackend(
      "GIMBAL_LASER_RANGE", config, &deprecated);
  EXPECT_FALSE(deprecated);
  EXPECT_EQ(std::string("gimbal_laser_range"), gimbal->name());
  EXPECT_TRUE(gimbal->requirements().laser_range);
}

TEST(LocalizationBackend, RejectsUnknownMethod) {
  xd_uav_detect::LocalizationBackendConfig config;
  EXPECT_THROW(
      xd_uav_detect::makeLocalizationBackend("gps_follow", config),
      std::invalid_argument);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
