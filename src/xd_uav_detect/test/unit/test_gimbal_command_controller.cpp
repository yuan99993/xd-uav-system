#include <limits>
#include <string>

#include <gtest/gtest.h>

#include "xd_uav_detect/gimbal/gimbal_command_controller.hpp"

namespace {

xd_uav_detect::GimbalControlConfig config() {
  xd_uav_detect::GimbalControlConfig result;
  result.minimum_yaw_rad = -1.0;
  result.maximum_yaw_rad = 1.0;
  result.minimum_pitch_rad = -0.8;
  result.maximum_pitch_rad = 0.4;
  result.maximum_yaw_rate_rad_s = 1.0;
  result.maximum_pitch_rate_rad_s = 0.5;
  result.command_timeout_sec = 0.5;
  return result;
}

TEST(GimbalCommandController, WaitsForFiniteJointFeedback) {
  xd_uav_detect::GimbalCommandController controller(config());
  EXPECT_FALSE(controller.update(0.0).output_valid);
  controller.setJointState(
      std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0, 0.0, 0.0);
  EXPECT_FALSE(controller.update(0.1).joint_state_valid);
  controller.setJointState(0.2, -0.1, 0.0, 0.0, 0.1);
  const auto output = controller.update(0.2);
  EXPECT_TRUE(output.output_valid);
  EXPECT_TRUE(output.joint_state_valid);
  EXPECT_FALSE(output.command_valid);
  EXPECT_EQ("waiting_for_command", output.status);
}

TEST(GimbalCommandController, ClampsAndRateLimitsPosition) {
  xd_uav_detect::GimbalCommandController controller(config());
  controller.setJointState(0.0, 0.0, 0.0, 0.0, 0.0);
  std::string rejection;
  ASSERT_TRUE(controller.acceptCommand(
      xd_uav_detect::GimbalCommandController::kPosition,
      5.0, -5.0, 0.0, 0.0, 0.0, &rejection));
  auto output = controller.update(0.1);
  EXPECT_NEAR(0.1, output.commanded_yaw_rad, 1e-9);
  EXPECT_NEAR(-0.05, output.commanded_pitch_rad, 1e-9);
  output = controller.update(0.4);
  EXPECT_NEAR(0.3, output.commanded_yaw_rad, 1e-9);
  EXPECT_NEAR(-0.15, output.commanded_pitch_rad, 1e-9);
}

TEST(GimbalCommandController, VelocityStopsOnTimeout) {
  xd_uav_detect::GimbalCommandController controller(config());
  controller.setJointState(0.0, 0.0, 0.0, 0.0, 0.0);
  std::string rejection;
  ASSERT_TRUE(controller.acceptCommand(
      xd_uav_detect::GimbalCommandController::kVelocity,
      0.0, 0.0, 0.8, -0.4, 0.0, &rejection));
  auto output = controller.update(0.2);
  EXPECT_NEAR(0.16, output.commanded_yaw_rad, 1e-9);
  EXPECT_NEAR(-0.08, output.commanded_pitch_rad, 1e-9);
  const double held_yaw = output.commanded_yaw_rad;
  const double held_pitch = output.commanded_pitch_rad;
  output = controller.update(0.8);
  EXPECT_FALSE(output.command_valid);
  EXPECT_EQ("command_timeout_hold", output.status);
  EXPECT_DOUBLE_EQ(held_yaw, output.commanded_yaw_rad);
  EXPECT_DOUBLE_EQ(held_pitch, output.commanded_pitch_rad);
}

TEST(GimbalCommandController, RejectsUnsupportedAndNonFiniteCommands) {
  xd_uav_detect::GimbalCommandController controller(config());
  std::string rejection;
  EXPECT_FALSE(controller.acceptCommand(99, 0.0, 0.0, 0.0, 0.0,
                                        0.0, &rejection));
  EXPECT_EQ("unsupported_mode", rejection);
  EXPECT_FALSE(controller.acceptCommand(
      xd_uav_detect::GimbalCommandController::kPosition,
      std::numeric_limits<double>::infinity(), 0.0, 0.0, 0.0,
      0.0, &rejection));
  EXPECT_EQ("non_finite_command", rejection);
}

TEST(GimbalCommandController, PreservesPositionCommandSentBeforeFeedback) {
  xd_uav_detect::GimbalCommandController controller(config());
  std::string rejection;
  ASSERT_TRUE(controller.acceptCommand(
      xd_uav_detect::GimbalCommandController::kPosition,
      0.8, -0.4, 1.0, 0.5, 0.0, &rejection));
  controller.setJointState(0.0, 0.0, 0.0, 0.0, 0.1);
  const auto output = controller.update(0.3);
  EXPECT_TRUE(output.command_valid);
  EXPECT_NEAR(0.2, output.commanded_yaw_rad, 1e-9);
  EXPECT_NEAR(-0.1, output.commanded_pitch_rad, 1e-9);
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
