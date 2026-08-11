#include <gtest/gtest.h>

#include <cmath>
#include <string>

#include <xd_uav_track/track_controller.hpp>

namespace {

xd_uav_track::TargetMeasurement box(
    const double x_min, const double y_min,
    const double x_max, const double y_max,
    const double confidence = 1.0, const double time = 10.0,
    const int track_id = 7) {
  xd_uav_track::TargetMeasurement value;
  value.receive_time = time;
  value.image_width = 640;
  value.image_height = 480;
  value.x_min = x_min;
  value.y_min = y_min;
  value.x_max = x_max;
  value.y_max = y_max;
  value.confidence = confidence;
  value.track_id = track_id;
  return value;
}

xd_uav_track::TrackControllerConfig deterministicConfig() {
  xd_uav_track::TrackControllerConfig config;
  config.position_filter_alpha = 1.0;
  config.velocity_smoothing_enabled = false;
  config.yaw_smoothing_enabled = false;
  config.forward_ramp_rate = 0.0;
  config.lateral_pid.ki = config.lateral_pid.kd = 0.0;
  config.vertical_pid.ki = config.vertical_pid.kd = 0.0;
  config.yaw_pid.ki = config.yaw_pid.kd = 0.0;
  config.ground_forward_pid.ki = config.ground_forward_pid.kd = 0.0;
  config.fw_course_pid.ki = config.fw_course_pid.kd = 0.0;
  config.fw_climb_rate_pid.ki = config.fw_climb_rate_pid.kd = 0.0;
  return config;
}

xd_uav_track::VehicleState vehicleState() {
  xd_uav_track::VehicleState state;
  state.altitude = 20.0;
  state.valid = true;
  return state;
}

xd_uav_track::GimbalStateData gimbalState(
    const double yaw, const double pitch, const double roll = 0.0,
    const double time = 10.0) {
  xd_uav_track::GimbalStateData state;
  state.receive_time = time;
  state.yaw = yaw;
  state.pitch = pitch;
  state.roll = roll;
  state.valid = true;
  return state;
}

TEST(TrackController, NormalizesCenteredBoxToPixEagleCoordinates) {
  auto config = deterministicConfig();
  config.profile = xd_uav_track::FollowerProfile::kVelocityPosition;
  xd_uav_track::TrackController controller(config);
  ASSERT_TRUE(controller.updateMeasurement(box(256, 192, 384, 288)));
  const auto output = controller.compute(10.01);
  EXPECT_TRUE(output.valid);
  EXPECT_TRUE(output.target_visible);
  EXPECT_EQ(output.tracking_state, "tracking");
  EXPECT_NEAR(output.center_x, 0.0, 1e-9);
  EXPECT_NEAR(output.center_y, 0.0, 1e-9);
  EXPECT_NEAR(output.error_x, 0.0, 1e-9);
  EXPECT_NEAR(output.error_y, 0.0, 1e-9);
  EXPECT_NEAR(output.forward, 0.0, 1e-9);
  EXPECT_NEAR(output.left, 0.0, 1e-9);
  EXPECT_NEAR(output.up, 0.0, 1e-9);
  EXPECT_NEAR(output.yaw_rate, 0.0, 1e-9);
}

TEST(TrackController, PositionProfileUsesYawAndVerticalOnly) {
  auto config = deterministicConfig();
  config.profile = xd_uav_track::FollowerProfile::kVelocityPosition;
  xd_uav_track::TrackController controller(config);
  ASSERT_TRUE(controller.updateMeasurement(box(400, 300, 500, 380)));
  const auto output = controller.compute(10.01);
  EXPECT_DOUBLE_EQ(output.forward, 0.0);
  EXPECT_DOUBLE_EQ(output.left, 0.0);
  EXPECT_LT(output.up, 0.0);
  EXPECT_LT(output.yaw_rate, 0.0);
}

TEST(TrackController, GroundProfileUsesImageAxesAndOptionalDescent) {
  auto config = deterministicConfig();
  config.profile = xd_uav_track::FollowerProfile::kVelocityGround;
  config.ground_altitude_scaling_enabled = false;
  config.ground_descend_to_target = true;
  config.ground_target_altitude = 3.0;
  config.ground_maximum_descent_velocity = 0.4;
  xd_uav_track::TrackController controller(config);
  controller.setVehicleState(vehicleState());
  ASSERT_TRUE(controller.updateMeasurement(box(400, 300, 500, 380)));
  const auto output = controller.compute(10.01);
  EXPECT_EQ(output.profile, "mc_velocity_ground");
  EXPECT_GT(output.forward, 0.0);
  EXPECT_LT(output.left, 0.0);
  EXPECT_NEAR(output.up, -0.4, 1e-9);
  EXPECT_DOUBLE_EQ(output.yaw_rate, 0.0);
}

TEST(TrackController, DistanceProfileUsesLateralAndVerticalWithoutYaw) {
  auto config = deterministicConfig();
  config.profile = xd_uav_track::FollowerProfile::kVelocityDistance;
  config.distance_enable_yaw = false;
  xd_uav_track::TrackController controller(config);
  ASSERT_TRUE(controller.updateMeasurement(box(400, 300, 500, 380)));
  const auto output = controller.compute(10.01);
  EXPECT_DOUBLE_EQ(output.forward, 0.0);
  EXPECT_LT(output.left, 0.0);
  EXPECT_LT(output.up, 0.0);
  EXPECT_DOUBLE_EQ(output.yaw_rate, 0.0);
}

TEST(TrackController, ChaseProfileRampsForwardAndCoordinatesWithYaw) {
  auto config = deterministicConfig();
  config.profile = xd_uav_track::FollowerProfile::kVelocityChase;
  config.chase_forward_velocity = 2.5;
  config.lateral_guidance =
      xd_uav_track::LateralGuidanceMode::kCoordinatedTurn;
  xd_uav_track::TrackController controller(config);
  ASSERT_TRUE(controller.updateMeasurement(box(400, 180, 500, 300)));
  const auto output = controller.compute(10.01);
  EXPECT_NEAR(output.forward, 2.5, 1e-9);
  EXPECT_DOUBLE_EQ(output.left, 0.0);
  EXPECT_LT(output.yaw_rate, 0.0);
}

TEST(TrackController, ChaseSideslipUsesLateralWithoutYaw) {
  auto config = deterministicConfig();
  config.profile = xd_uav_track::FollowerProfile::kVelocityChase;
  config.lateral_guidance = xd_uav_track::LateralGuidanceMode::kSideslip;
  xd_uav_track::TrackController controller(config);
  ASSERT_TRUE(controller.updateMeasurement(box(400, 180, 500, 300)));
  const auto output = controller.compute(10.01);
  EXPECT_GT(output.forward, 0.0);
  EXPECT_LT(output.left, 0.0);
  EXPECT_DOUBLE_EQ(output.yaw_rate, 0.0);
}

TEST(TrackController, GimbalChaseUsesAnglesForPidPursuit) {
  auto config = deterministicConfig();
  config.profile = xd_uav_track::FollowerProfile::kGimbalVelocityChase;
  config.gimbal_angle_filter_previous_weight = 0.0;
  config.gimbal_angle_deadzone = 0.0;
  config.gm_chase_base_forward_speed = 2.0;
  config.gm_chase_forward_acceleration = 0.0;
  config.gm_chase_lateral_guidance =
      xd_uav_track::LateralGuidanceMode::kCoordinatedTurn;
  xd_uav_track::TrackController controller(config);
  controller.setGimbalState(gimbalState(0.30, 0.20));
  ASSERT_TRUE(controller.updateMeasurement(box(280, 200, 360, 280)));
  const auto output = controller.compute(10.01);
  EXPECT_TRUE(output.valid);
  EXPECT_EQ(output.profile, "gm_velocity_chase");
  EXPECT_EQ(output.lateral_guidance_mode, "coordinated_turn");
  EXPECT_GT(output.forward, 0.0);
  EXPECT_DOUBLE_EQ(output.left, 0.0);
  EXPECT_LT(output.up, 0.0);
  EXPECT_LT(output.yaw_rate, 0.0);
}

TEST(TrackController, GimbalVectorConvertsOpticalAxisToBodyVelocity) {
  auto config = deterministicConfig();
  config.profile = xd_uav_track::FollowerProfile::kGimbalVelocityVector;
  config.gimbal_angle_filter_previous_weight = 0.0;
  config.gimbal_angle_deadzone = 0.0;
  config.gm_vector_velocity = 2.0;
  config.gm_vector_ramp_acceleration = 0.0;
  config.gm_vector_enable_vertical_control = true;
  config.gm_vector_lateral_guidance =
      xd_uav_track::LateralGuidanceMode::kSideslip;
  xd_uav_track::TrackController controller(config);
  controller.setGimbalState(gimbalState(0.30, 0.20));
  ASSERT_TRUE(controller.updateMeasurement(box(280, 200, 360, 280)));
  const auto output = controller.compute(10.01);
  EXPECT_TRUE(output.valid);
  EXPECT_EQ(output.profile, "gm_velocity_vector");
  EXPECT_EQ(output.lateral_guidance_mode, "sideslip");
  EXPECT_GT(output.forward, 0.0);
  EXPECT_LT(output.left, 0.0);
  EXPECT_LT(output.up, 0.0);
  EXPECT_DOUBLE_EQ(output.yaw_rate, 0.0);
}

TEST(TrackController, GimbalProfilesRequireFreshGimbalState) {
  auto config = deterministicConfig();
  config.profile = xd_uav_track::FollowerProfile::kGimbalVelocityChase;
  xd_uav_track::TrackController controller(config);
  ASSERT_TRUE(controller.updateMeasurement(box(280, 200, 360, 280)));
  const auto output = controller.compute(10.01);
  EXPECT_FALSE(output.valid);
  EXPECT_EQ(output.tracking_state, "waiting_for_gimbal_state");
}

TEST(TrackController, FixedWingProfileProducesOnlyOneVelocityVector) {
  auto config = deterministicConfig();
  config.profile =
      xd_uav_track::FollowerProfile::kFixedWingVelocityVector;
  config.fw_commanded_airspeed = 15.0;
  config.fw_maximum_course_offset = 0.70;
  config.fw_maximum_climb_rate = 3.0;
  xd_uav_track::TrackController controller(config);
  ASSERT_TRUE(controller.updateMeasurement(box(380, 80, 480, 180)));
  const auto output = controller.compute(10.01);
  EXPECT_TRUE(output.valid);
  EXPECT_EQ(output.profile, "fw_velocity_vector");
  EXPECT_EQ(output.lateral_guidance_mode, "velocity_vector");
  EXPECT_GT(output.forward, 0.0);
  EXPECT_LT(output.left, 0.0);
  EXPECT_GT(output.up, 0.0);
  EXPECT_NEAR(std::hypot(output.forward, output.left),
              config.fw_commanded_airspeed, 1e-9);
  EXPECT_DOUBLE_EQ(output.yaw_rate, 0.0);
  EXPECT_FALSE(output.use_yaw_rate);
}

TEST(TrackController, FixedWingTargetLossReleasesVelocityReference) {
  auto config = deterministicConfig();
  config.profile =
      xd_uav_track::FollowerProfile::kFixedWingVelocityVector;
  config.input_timeout_sec = 0.20;
  xd_uav_track::TrackController controller(config);
  ASSERT_TRUE(controller.updateMeasurement(box(380, 80, 480, 180)));
  EXPECT_TRUE(controller.compute(10.01).valid);
  const auto lost = controller.compute(10.31);
  EXPECT_FALSE(lost.valid);
  EXPECT_FALSE(lost.target_visible);
  EXPECT_TRUE(lost.release_reference_on_invalid);
  EXPECT_EQ(lost.tracking_state, "target_lost");
  EXPECT_DOUBLE_EQ(lost.forward, 0.0);
  EXPECT_DOUBLE_EQ(lost.left, 0.0);
  EXPECT_DOUBLE_EQ(lost.up, 0.0);
}

TEST(TrackController, SupportsAllSevenVelocityProfiles) {
  const char* names[] = {
      "mc_velocity_ground", "mc_velocity_position",
      "mc_velocity_distance", "mc_velocity_chase",
      "gm_velocity_chase", "gm_velocity_vector",
      "fw_velocity_vector"};
  for (const char* name : names) {
    xd_uav_track::FollowerProfile profile;
    ASSERT_TRUE(xd_uav_track::parseFollowerProfile(name, &profile));
    EXPECT_STREQ(xd_uav_track::followerProfileName(profile), name);
  }
  xd_uav_track::FollowerProfile rejected;
  EXPECT_FALSE(xd_uav_track::parseFollowerProfile("unknown", &rejected));
}

TEST(TrackController, ConfidenceHysteresisKeepsAnAcquiredTarget) {
  auto config = deterministicConfig();
  config.minimum_confidence = 0.50;
  config.confidence_hysteresis = 0.10;
  xd_uav_track::TrackController controller(config);
  std::string reason;
  EXPECT_FALSE(controller.updateMeasurement(
      box(200, 150, 300, 250, 0.45), &reason));
  ASSERT_TRUE(controller.updateMeasurement(
      box(200, 150, 300, 250, 0.60), &reason));
  EXPECT_TRUE(controller.updateMeasurement(
      box(202, 150, 302, 250, 0.45, 10.05), &reason));
  EXPECT_FALSE(controller.updateMeasurement(
      box(204, 150, 304, 250, 0.39, 10.10), &reason));
}

TEST(TrackController, TargetLossStopsAxesAndRampsChaseForwardSpeed) {
  auto config = deterministicConfig();
  config.profile = xd_uav_track::FollowerProfile::kVelocityChase;
  config.input_timeout_sec = 0.20;
  config.initial_forward_velocity = 2.0;
  config.chase_forward_velocity = 2.0;
  config.forward_ramp_rate = 1.0;
  config.target_loss_forward_velocity = 0.0;
  xd_uav_track::TrackController controller(config);
  ASSERT_TRUE(controller.updateMeasurement(box(400, 300, 500, 380)));
  EXPECT_NEAR(controller.compute(10.01).forward, 2.0, 1e-9);
  const auto lost = controller.compute(10.31);
  EXPECT_TRUE(lost.valid);
  EXPECT_FALSE(lost.target_visible);
  EXPECT_EQ(lost.tracking_state, "target_lost");
  EXPECT_GT(lost.forward, 0.0);
  EXPECT_LT(lost.forward, 2.0);
  EXPECT_DOUBLE_EQ(lost.left, 0.0);
  EXPECT_DOUBLE_EQ(lost.up, 0.0);
  EXPECT_DOUBLE_EQ(lost.yaw_rate, 0.0);
  auto stopped = lost;
  for (int index = 1; index <= 20; ++index) {
    stopped = controller.compute(10.31 + 0.10 * index);
  }
  EXPECT_NEAR(stopped.forward, 0.0, 1e-9);
}

TEST(TrackController, RejectsMalformedBoxesAndStartsWithoutACommand) {
  xd_uav_track::TrackController controller{
      xd_uav_track::TrackControllerConfig()};
  EXPECT_FALSE(controller.compute(1.0).valid);
  std::string reason;
  EXPECT_FALSE(controller.updateMeasurement(box(100, 100, 100, 200), &reason));
  EXPECT_FALSE(reason.empty());
}

TEST(TrackController, RuntimeProfileSwitchResetsFollowerState) {
  auto config = deterministicConfig();
  config.profile = xd_uav_track::FollowerProfile::kVelocityChase;
  config.initial_forward_velocity = 0.0;
  config.chase_forward_velocity = 2.0;
  xd_uav_track::TrackController controller(config);
  ASSERT_TRUE(controller.updateMeasurement(box(400, 180, 500, 300)));
  EXPECT_GT(controller.compute(10.01).forward, 0.0);
  EXPECT_TRUE(controller.setProfile(
      xd_uav_track::FollowerProfile::kVelocityPosition));
  const auto position = controller.compute(10.02);
  EXPECT_EQ(position.profile, "mc_velocity_position");
  EXPECT_DOUBLE_EQ(position.forward, 0.0);
  EXPECT_DOUBLE_EQ(position.left, 0.0);
}

TEST(TrackController, PredictedTrackIsExplicitAndCovarianceLimitsSpeed) {
  auto config = deterministicConfig();
  config.profile = xd_uav_track::FollowerProfile::kVelocityChase;
  config.chase_forward_velocity = 4.0;
  config.uncertainty_nominal_sigma = 0.05;
  config.uncertainty_slow_ratio = 1.0;
  config.uncertainty_abort_ratio = 3.0;
  auto measurement = box(280, 200, 360, 280);
  measurement.predicted = true;
  measurement.tracking_quality = 0.6;
  measurement.association_method = "predicted";
  measurement.state_covariance = {{0.01, 0.01, 0.01, 0.01}};
  xd_uav_track::TrackController controller(config);
  ASSERT_TRUE(controller.updateMeasurement(measurement));
  const auto output = controller.compute(10.01);
  EXPECT_TRUE(output.valid);
  EXPECT_TRUE(output.target_predicted);
  EXPECT_EQ(output.tracking_state, "predicting");
  EXPECT_TRUE(output.uncertainty_limited);
  EXPECT_NEAR(output.uncertainty_scale, 0.5, 1e-9);
  EXPECT_NEAR(output.forward, 2.0, 1e-9);
  EXPECT_EQ(output.association_method, "predicted");
}

TEST(TrackController, ExcessiveCovarianceAbortsGuidance) {
  auto config = deterministicConfig();
  config.uncertainty_nominal_sigma = 0.05;
  config.uncertainty_abort_ratio = 2.0;
  auto measurement = box(280, 200, 360, 280);
  measurement.state_covariance = {{0.04, 0.04, 0.01, 0.01}};
  xd_uav_track::TrackController controller(config);
  ASSERT_TRUE(controller.updateMeasurement(measurement));
  const auto output = controller.compute(10.01);
  EXPECT_FALSE(output.valid);
  EXPECT_EQ(output.tracking_state, "uncertainty_abort");
  EXPECT_DOUBLE_EQ(output.uncertainty_scale, 0.0);
}

TEST(TrackController, RelativeStateAddsMetricChaseCorrection) {
  auto config = deterministicConfig();
  config.profile = xd_uav_track::FollowerProfile::kVelocityChase;
  config.chase_forward_velocity = 1.0;
  config.relative_target_forward = 8.0;
  config.relative_position_gain = 0.5;
  config.relative_velocity_feedforward = 1.0;
  auto measurement = box(280, 200, 360, 280);
  measurement.has_relative_position_body = true;
  measurement.has_relative_velocity_body = true;
  measurement.range_valid = true;
  measurement.relative_position_body = {{10.0, 1.0, -1.0}};
  measurement.relative_velocity_body = {{0.5, 0.0, 0.0}};
  xd_uav_track::TrackController controller(config);
  ASSERT_TRUE(controller.updateMeasurement(measurement));
  const auto output = controller.compute(10.01);
  EXPECT_TRUE(output.relative_state_active);
  EXPECT_NEAR(output.forward, 2.5, 1e-9);
  EXPECT_NEAR(output.left, -0.5, 1e-9);
  EXPECT_NEAR(output.up, 0.5, 1e-9);
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
