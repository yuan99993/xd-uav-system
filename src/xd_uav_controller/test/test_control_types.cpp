#include <gtest/gtest.h>

#include <xd_uav_controller/backend_resolver.h>
#include <xd_uav_controller/controller_backend.h>
#include <xd_uav_controller/command_contract.h>
#include <xd_uav_controller/control_types.h>
#include <xd_uav_controller/functional_controller_backends.h>
#include <xd_uav_controller/setpoint_output.h>
#include <xd_uav_controller/unified_reference.h>

namespace control = xd_uav_controller;

TEST(ControlTypes, ParsesLegacyAndNewAirframeNames) {
  EXPECT_EQ(control::parseAirframeType("multirotor"),
            control::AirframeType::kMultirotor);
  EXPECT_EQ(control::parseAirframeType("fixedwing"),
            control::AirframeType::kFixedWing);
  EXPECT_EQ(control::parseAirframeType("fixed_wing"),
            control::AirframeType::kFixedWing);
  EXPECT_EQ(control::parseAirframeType("vtol"),
            control::AirframeType::kVtol);
  EXPECT_EQ(control::parseAirframeType("tiltrotor"),
            control::AirframeType::kTiltrotor);
  EXPECT_EQ(control::parseAirframeType("invalid"),
            control::AirframeType::kUnknown);
}

TEST(ControlTypes, ValidatesAirframeAndRegimePairs) {
  EXPECT_TRUE(control::supportsRegime(
      control::AirframeType::kMultirotor,
      control::FlightRegime::kHover));
  EXPECT_FALSE(control::supportsRegime(
      control::AirframeType::kMultirotor,
      control::FlightRegime::kForwardFlight));
  EXPECT_TRUE(control::supportsRegime(
      control::AirframeType::kFixedWing,
      control::FlightRegime::kForwardFlight));
  EXPECT_TRUE(control::supportsRegime(
      control::AirframeType::kVtol,
      control::FlightRegime::kTransitionToForward));
  EXPECT_TRUE(control::supportsRegime(
      control::AirframeType::kTiltrotor,
      control::FlightRegime::kTransitionToHover));
}

TEST(ControlTypes, PreservesSourceBackendDuringTransitions) {
  EXPECT_EQ(control::BackendId::kMultirotor,
            control::backendForRegime(
                control::FlightRegime::kTransitionToForward,
                control::BackendId::kMultirotor));
  EXPECT_EQ(control::BackendId::kFixedWing,
            control::backendForRegime(
                control::FlightRegime::kTransitionToHover,
                control::BackendId::kFixedWing));
  EXPECT_EQ(control::BackendId::kNone,
            control::backendForRegime(
                control::FlightRegime::kTransitionToForward));
}

TEST(BackendResolver, RejectsIllegalAirframeRegimeCombination) {
  const control::BackendResolution resolution = control::resolveBackend(
      control::AirframeType::kMultirotor,
      control::FlightRegime::kForwardFlight,
      control::BackendId::kMultirotor);

  EXPECT_FALSE(resolution.valid);
  EXPECT_EQ(control::BackendId::kNone, resolution.backend);
  EXPECT_FALSE(resolution.reason.empty());
}

TEST(BackendResolver, EnforcesTheDeterministicTransitionSource) {
  const auto forward = control::resolveBackend(
      control::AirframeType::kVtol,
      control::FlightRegime::kTransitionToForward,
      control::BackendId::kFixedWing);
  const auto hover = control::resolveBackend(
      control::AirframeType::kTiltrotor,
      control::FlightRegime::kTransitionToHover,
      control::BackendId::kMultirotor);

  EXPECT_FALSE(forward.valid);
  EXPECT_EQ(control::BackendId::kNone, forward.backend);
  EXPECT_FALSE(hover.valid);
  EXPECT_EQ(control::BackendId::kNone, hover.backend);
}

TEST(BackendResolver, KeepsLastBackendOnUnknownWithoutSelectingANewOne) {
  const auto resolution = control::resolveBackend(
      control::AirframeType::kVtol,
      control::FlightRegime::kUnknown,
      control::BackendId::kFixedWing);

  EXPECT_FALSE(resolution.valid);
  EXPECT_EQ(control::BackendId::kNone, resolution.backend);
  EXPECT_EQ(control::BackendId::kFixedWing,
            resolution.last_stable_backend);
}

TEST(UnifiedReference, PreservesPerAxisMasksAndSourceMetadata) {
  control::UnifiedReference reference;
  reference.type = control::ReferenceType::kPositionTarget;
  reference.use_position = {{true, false, true}};
  reference.use_velocity = {{false, true, false}};
  reference.source = "position_target";
  reference.position.x = 4.0;
  reference.velocity.y = -2.0;

  EXPECT_TRUE(reference.use_position[0]);
  EXPECT_FALSE(reference.use_position[1]);
  EXPECT_TRUE(reference.use_position[2]);
  EXPECT_TRUE(reference.use_velocity[1]);
  EXPECT_EQ("position_target", reference.source);
  EXPECT_DOUBLE_EQ(4.0, reference.position.x);
  EXPECT_DOUBLE_EQ(-2.0, reference.velocity.y);
}

TEST(SetpointOutput, ConvertsMaskedReferenceToRawLocalTarget) {
  control::UnifiedReference reference;
  reference.type = control::ReferenceType::kTrajectory;
  reference.source = "trajectory";
  reference.header.frame_id = "uav2/odom";
  reference.position.x = 10.0;
  reference.position.z = 3.0;
  reference.velocity.y = 2.0;
  reference.acceleration.z = 0.5;
  reference.yaw = 1.2;
  reference.use_position = {{true, false, true}};
  reference.use_velocity = {{false, true, false}};
  reference.use_acceleration = {{false, false, true}};
  reference.use_yaw = true;
  reference.use_yaw_rate = false;

  mavros_msgs::PositionTarget target;
  std::string reason;
  const ros::Time stamp(123.0);
  ASSERT_TRUE(control::makeRawLocalTarget(
      reference, stamp, &target, &reason)) << reason;
  EXPECT_EQ("uav2/odom", target.header.frame_id);
  EXPECT_EQ(stamp, target.header.stamp);
  EXPECT_EQ(mavros_msgs::PositionTarget::FRAME_LOCAL_NED,
            target.coordinate_frame);
  EXPECT_DOUBLE_EQ(10.0, target.position.x);
  EXPECT_DOUBLE_EQ(2.0, target.velocity.y);
  EXPECT_DOUBLE_EQ(0.5, target.acceleration_or_force.z);
  EXPECT_FLOAT_EQ(1.2F, target.yaw);
  EXPECT_EQ(0U, target.type_mask &
                    mavros_msgs::PositionTarget::IGNORE_PX);
  EXPECT_NE(0U, target.type_mask &
                    mavros_msgs::PositionTarget::IGNORE_PY);
  EXPECT_NE(0U, target.type_mask &
                    mavros_msgs::PositionTarget::IGNORE_VX);
  EXPECT_EQ(0U, target.type_mask &
                    mavros_msgs::PositionTarget::IGNORE_VY);
  EXPECT_NE(0U, target.type_mask &
                    mavros_msgs::PositionTarget::IGNORE_YAW_RATE);
}

TEST(SetpointOutput, RestrictsFixedWingRawLocalToFullPositionTarget) {
  control::UnifiedReference reference;
  reference.header.frame_id = "uav2/odom";
  reference.use_position = {{true, true, true}};
  reference.use_velocity = {{true, true, true}};
  reference.use_acceleration = {{true, true, true}};
  reference.use_yaw = true;
  reference.use_yaw_rate = true;

  mavros_msgs::PositionTarget target;
  std::string reason;
  ASSERT_TRUE(control::makeRawLocalTarget(
      reference, ros::Time(123.0), &target, &reason, true)) << reason;
  EXPECT_EQ(
      mavros_msgs::PositionTarget::IGNORE_VX |
          mavros_msgs::PositionTarget::IGNORE_VY |
          mavros_msgs::PositionTarget::IGNORE_VZ |
          mavros_msgs::PositionTarget::IGNORE_AFX |
          mavros_msgs::PositionTarget::IGNORE_AFY |
          mavros_msgs::PositionTarget::IGNORE_AFZ |
          mavros_msgs::PositionTarget::IGNORE_YAW |
          mavros_msgs::PositionTarget::IGNORE_YAW_RATE,
      target.type_mask);

  reference.use_position[2] = false;
  EXPECT_FALSE(control::makeRawLocalTarget(
      reference, ros::Time(123.0), &target, &reason, true));
  EXPECT_NE(std::string::npos, reason.find("完整XYZ位置目标"));
}

TEST(SetpointOutput, InternalHoverTakeoffUsesPositionAndYawOnly) {
  control::UnifiedReference reference;
  reference.type = control::ReferenceType::kInternal;
  reference.source = "takeoff";
  reference.header.frame_id = "uav2/odom";
  reference.position.x = 1.0;
  reference.position.y = -2.0;
  reference.position.z = 8.0;
  reference.velocity = geometry_msgs::Vector3();
  reference.acceleration = geometry_msgs::Vector3();
  reference.use_position = {{true, true, true}};
  reference.use_velocity = {{true, true, true}};
  reference.use_acceleration = {{true, true, true}};
  reference.use_yaw = true;
  reference.yaw = 0.4;
  reference.use_yaw_rate = false;

  mavros_msgs::PositionTarget target;
  std::string reason;
  ASSERT_TRUE(control::makeRawLocalTarget(
      reference, ros::Time(123.0), &target, &reason)) << reason;
  EXPECT_EQ(
      mavros_msgs::PositionTarget::IGNORE_VX |
          mavros_msgs::PositionTarget::IGNORE_VY |
          mavros_msgs::PositionTarget::IGNORE_VZ |
          mavros_msgs::PositionTarget::IGNORE_AFX |
          mavros_msgs::PositionTarget::IGNORE_AFY |
          mavros_msgs::PositionTarget::IGNORE_AFZ |
          mavros_msgs::PositionTarget::IGNORE_YAW_RATE,
      target.type_mask);
  EXPECT_EQ(0U, target.type_mask &
                    mavros_msgs::PositionTarget::IGNORE_YAW);
  EXPECT_FLOAT_EQ(0.4F, target.yaw);
}

TEST(SetpointOutput, SelectsOutputIndependentlyByReferenceType) {
  EXPECT_EQ(control::SetpointOutputType::kRawLocal,
            control::setpointOutputTypeForReference(
                control::ReferenceType::kSimpleGoal,
                "raw_local", "raw_attitude", "raw_attitude"));
  EXPECT_EQ(control::SetpointOutputType::kRawAttitude,
            control::setpointOutputTypeForReference(
                control::ReferenceType::kPath,
                "raw_local", "raw_attitude", "raw_local"));
  EXPECT_EQ(control::SetpointOutputType::kRawLocal,
            control::setpointOutputTypeForReference(
                control::ReferenceType::kTrajectory,
                "raw_local", "raw_attitude", "raw_local"));
  EXPECT_EQ(control::SetpointOutputType::kRawAttitude,
            control::setpointOutputTypeForReference(
                control::ReferenceType::kInternal,
                "raw_local", "raw_local", "raw_local"));
  EXPECT_EQ(control::SetpointOutputType::kRawLocal,
            control::setpointOutputTypeForReference(
                control::ReferenceType::kInternal,
                "raw_attitude", "raw_attitude", "raw_attitude",
                "raw_local"));
  EXPECT_EQ(control::SetpointOutputType::kRawLocal,
            control::setpointOutputTypeForReference(
                control::ReferenceType::kIdle,
                "raw_attitude", "raw_attitude", "raw_attitude",
                "raw_local"));
}

TEST(ControllerBackendContract, MapsReferenceTypesToWireValues) {
  EXPECT_EQ(1U, control::referenceTypeValue(
                    control::ReferenceType::kPositionTarget));
  EXPECT_EQ(2U, control::referenceTypeValue(
                    control::ReferenceType::kTrajectory));
  EXPECT_EQ(3U,
            control::referenceTypeValue(control::ReferenceType::kPath));
  EXPECT_EQ(5U,
            control::referenceTypeValue(control::ReferenceType::kInternal));
  EXPECT_EQ(6U,
            control::referenceTypeValue(control::ReferenceType::kIdle));
}

TEST(ControllerBackendContract, LifecycleResetsStateAndRejectsInactiveUpdate) {
  int reset_count = 0;
  control::MultirotorControllerBackend backend(
      [](const control::ControlState&, const control::UnifiedReference&,
         const ros::Time&, double) {
        control::BackendOutput output;
        output.valid = true;
        return output;
      },
      [&reset_count]() { ++reset_count; });
  control::ControlState state;
  control::UnifiedReference reference;
  reference.type = control::ReferenceType::kIdle;

  EXPECT_FALSE(backend.update(state, reference, ros::Time(1.0), 0.01).valid);
  backend.onActivate(state, reference);
  EXPECT_EQ(1, reset_count);
  EXPECT_TRUE(backend.active());
  EXPECT_TRUE(backend.update(state, reference, ros::Time(1.0), 0.01).valid);
  backend.onDeactivate();
  EXPECT_FALSE(backend.active());
  EXPECT_FALSE(backend.update(state, reference, ros::Time(1.0), 0.01).valid);
}

TEST(CommandContract, RejectsOldRegimeAndActionGenerations) {
  control::ControlState state;
  state.airframe_type = state.AIRFRAME_VTOL;
  state.flight_regime = state.REGIME_HOVER;
  state.regime_generation = 4U;
  state.action_generation = 8U;
  control::ControlCommand command;
  command.airframe_type = state.airframe_type;
  command.flight_regime = state.flight_regime;
  command.regime_generation = state.regime_generation;
  command.action_generation = state.action_generation;
  command.active_backend = command.BACKEND_MULTIROTOR;
  std::string reason;

  EXPECT_TRUE(control::commandMatchesStateContract(
      state, command, control::BackendId::kMultirotor, &reason));
  --command.regime_generation;
  EXPECT_FALSE(control::commandMatchesStateContract(
      state, command, control::BackendId::kMultirotor, &reason));
  command.regime_generation = state.regime_generation;
  --command.action_generation;
  EXPECT_FALSE(control::commandMatchesStateContract(
      state, command, control::BackendId::kMultirotor, &reason));
}

TEST(CommandContract, AllowsSourceBackendDuringObservedVtolTransitions) {
  control::ControlState state;
  state.airframe_type = state.AIRFRAME_VTOL;
  state.regime_generation = 4U;
  state.action_generation = 8U;
  control::ControlCommand command;
  command.airframe_type = state.airframe_type;
  command.regime_generation = state.regime_generation;
  command.action_generation = state.action_generation;
  std::string reason;

  state.flight_regime = state.REGIME_TRANSITION_TO_FORWARD;
  command.flight_regime = command.REGIME_HOVER;
  command.active_backend = command.BACKEND_MULTIROTOR;
  EXPECT_TRUE(control::commandMatchesStateContract(
      state, command, control::BackendId::kMultirotor, &reason)) << reason;

  state.flight_regime = state.REGIME_TRANSITION_TO_HOVER;
  command.flight_regime = command.REGIME_FORWARD_FLIGHT;
  command.active_backend = command.BACKEND_FIXED_WING;
  EXPECT_TRUE(control::commandMatchesStateContract(
      state, command, control::BackendId::kFixedWing, &reason)) << reason;

  command.flight_regime = command.REGIME_HOVER;
  command.active_backend = command.BACKEND_MULTIROTOR;
  EXPECT_FALSE(control::commandMatchesStateContract(
      state, command, control::BackendId::kFixedWing, &reason));
  EXPECT_EQ("flight_regime mismatch", reason);
}

TEST(CommandContract, StillRejectsTransitionCommandsFromWrongGeneration) {
  control::ControlState state;
  state.airframe_type = state.AIRFRAME_VTOL;
  state.flight_regime = state.REGIME_TRANSITION_TO_FORWARD;
  state.regime_generation = 4U;
  state.action_generation = 8U;
  control::ControlCommand command;
  command.airframe_type = state.airframe_type;
  command.flight_regime = command.REGIME_HOVER;
  command.regime_generation = state.regime_generation - 1U;
  command.action_generation = state.action_generation;
  command.active_backend = command.BACKEND_MULTIROTOR;
  std::string reason;

  EXPECT_FALSE(control::commandMatchesStateContract(
      state, command, control::BackendId::kMultirotor, &reason));
  EXPECT_EQ("stale regime_generation", reason);
}
