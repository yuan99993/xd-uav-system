#include <gtest/gtest.h>

#include <xd_uav_controller/backend_resolver.h>
#include <xd_uav_controller/controller_backend.h>
#include <xd_uav_controller/command_contract.h>
#include <xd_uav_controller/control_types.h>
#include <xd_uav_controller/functional_controller_backends.h>
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
