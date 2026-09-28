#include <gtest/gtest.h>

#include <cmath>

#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/ParamValue.h>

#include <xd_uav_control_manager/autopilot/arducopter_guided_backend.h>
#include <xd_uav_control_manager/autopilot/px4_offboard_backend.h>

namespace manager = xd_uav_control_manager;

namespace {

manager::ArduCopterGuidedBackend makeArduCopterBackend() {
  return manager::ArduCopterGuidedBackend(
      "GUIDED", "LOITER", "GUID_OPTIONS", 8,
      "MOT_THST_HOVER", 0.01, 245, 10.0);
}

void validateArduCopterParameters(
    manager::ArduCopterGuidedBackend* backend) {
  mavros_msgs::ParamValue guid_options;
  guid_options.integer = 8;
  mavros_msgs::ParamValue hover;
  hover.real = 0.39;
  std::string reason;
  ASSERT_TRUE(backend->validateParameter(
      "GUID_OPTIONS", guid_options, &reason));
  ASSERT_TRUE(backend->validateParameter(
      "MOT_THST_HOVER", hover, &reason));
}

}  // namespace

TEST(Px4OffboardBackend, PreservesBodyRateContractAndMask) {
  manager::Px4OffboardBackend backend("OFFBOARD", "POSCTL");
  xd_uav_controller::ControlCommand command;
  command.output_type = command.OUTPUT_BODY_RATE;
  command.body_rate_valid = true;
  command.body_rate.x = 0.1;
  command.body_rate.y = -0.2;
  command.body_rate.z = 0.3;
  command.thrust = 0.45;
  mavros_msgs::AttitudeTarget target;
  std::string reason;

  ASSERT_TRUE(backend.buildTarget(
      command, "base_link", ros::Time(3.0), &target, &reason));
  EXPECT_EQ(mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE,
            target.type_mask);
  EXPECT_DOUBLE_EQ(0.1, target.body_rate.x);
  EXPECT_FLOAT_EQ(0.45F, target.thrust);
  EXPECT_EQ("OFFBOARD", backend.activeMode());
  EXPECT_EQ("POSCTL", backend.exitMode());
}

TEST(ArduCopterGuidedBackend, UsesQuaternionAndIgnoresAllBodyRates) {
  auto backend = makeArduCopterBackend();
  validateArduCopterParameters(&backend);
  xd_uav_controller::ControlCommand command;
  command.output_type = command.OUTPUT_ATTITUDE;
  command.attitude_valid = true;
  command.attitude.z = std::sin(0.25);
  command.attitude.w = std::cos(0.25);
  command.thrust = 0.6;
  command.hover_throttle = 0.39;
  mavros_msgs::AttitudeTarget target;
  std::string reason;

  ASSERT_TRUE(backend.buildTarget(
      command, "base_link", ros::Time(4.0), &target, &reason));
  EXPECT_EQ(mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE |
                mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE |
                mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE,
            target.type_mask);
  EXPECT_NEAR(std::sin(0.25), target.orientation.z, 1e-12);
  EXPECT_NEAR(std::cos(0.25), target.orientation.w, 1e-12);
  EXPECT_FLOAT_EQ(0.6F, target.thrust);
  EXPECT_EQ((std::vector<std::string>{"GUID_OPTIONS", "MOT_THST_HOVER"}),
            backend.requiredParameters());
  const auto intervals = backend.requiredMessageIntervals();
  ASSERT_EQ(1U, intervals.size());
  EXPECT_EQ(245U, intervals.front().message_id);
  EXPECT_FLOAT_EQ(10.0F, intervals.front().rate_hz);
}

TEST(ArduCopterGuidedBackend, RejectsBodyRateAndUnsupportedAirframes) {
  auto backend = makeArduCopterBackend();
  xd_uav_controller::ControlCommand command;
  command.output_type = command.OUTPUT_BODY_RATE;
  command.body_rate_valid = true;
  command.thrust = 0.5;
  std::string reason;

  EXPECT_FALSE(backend.validateCommand(command, &reason));
  EXPECT_TRUE(backend.supportsAirframe(
      xd_uav_controller::AirframeType::kMultirotor));
  EXPECT_FALSE(backend.supportsAirframe(
      xd_uav_controller::AirframeType::kFixedWing));
  EXPECT_FALSE(backend.supportsAirframe(
      xd_uav_controller::AirframeType::kVtol));
}

TEST(ArduCopterGuidedBackend, RejectsCapabilityAndHoverMismatch) {
  auto backend = makeArduCopterBackend();
  mavros_msgs::ParamValue value;
  std::string reason;
  EXPECT_FALSE(backend.validateParameter("GUID_OPTIONS", value, &reason));
  value.integer = 8;
  EXPECT_TRUE(backend.validateParameter("GUID_OPTIONS", value, &reason));
  value.real = 0.39;
  EXPECT_TRUE(backend.validateParameter("MOT_THST_HOVER", value, &reason));

  xd_uav_controller::ControlCommand command;
  command.output_type = command.OUTPUT_ATTITUDE;
  command.attitude_valid = true;
  command.attitude.w = 1.0;
  command.thrust = 0.39;
  command.hover_throttle = 0.70;
  EXPECT_FALSE(backend.validateCommand(command, &reason));
  EXPECT_NE(std::string::npos, reason.find("MOT_THST_HOVER"));
}

TEST(AutopilotBackendFactory, RejectsUnknownAutopilot) {
  EXPECT_THROW(manager::makeAutopilotBackend(
                   "unknown", "ACTIVE", "EXIT", "P", 1,
                   "HOVER", 0.01, 245, 10.0),
               std::runtime_error);
}
