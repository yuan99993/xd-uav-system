#include <gtest/gtest.h>

#include <xd_uav_control_manager/SetFlightRegime.h>
#include <xd_uav_controller/ControlCommand.h>
#include <xd_uav_controller/ControlState.h>
#include <xd_uav_controller/InternalCommand.h>

TEST(ControlInterfaces, CarriesAirframeRegimeAndGeneration) {
  xd_uav_controller::ControlState state;
  state.airframe_type = state.AIRFRAME_VTOL;
  state.flight_regime = state.REGIME_TRANSITION_TO_FORWARD;
  state.requested_regime = state.REQUESTED_FORWARD_FLIGHT;
  state.vehicle_action = state.ACTION_NAVIGATE;
  state.regime_generation = 7U;
  state.action_generation = 11U;
  state.transition_pending = true;

  xd_uav_controller::ControlCommand command;
  command.airframe_type = state.airframe_type;
  command.flight_regime = state.flight_regime;
  command.regime_generation = state.regime_generation;
  command.active_backend = command.BACKEND_MULTIROTOR;
  command.action_generation = state.action_generation;
  command.action_status = command.ACTION_ACTIVE;

  EXPECT_TRUE(state.transition_pending);
  EXPECT_EQ(command.regime_generation, 7U);
  EXPECT_EQ(command.action_generation, 11U);
}

TEST(ControlInterfaces, ExposesForwardAndHoverTransitionTargets) {
  xd_uav_control_manager::SetFlightRegime request;
  request.request.target_regime =
      xd_uav_control_manager::SetFlightRegime::Request::TARGET_FORWARD_FLIGHT;
  EXPECT_EQ(request.request.target_regime,
            xd_uav_control_manager::SetFlightRegime::Request::TARGET_FORWARD_FLIGHT);
}

TEST(ControlInterfaces, InternalActionCommandCarriesPhaseAndGeneration) {
  xd_uav_controller::InternalCommand command;
  command.request.command =
      xd_uav_controller::InternalCommand::Request::LAND_HOME;
  command.request.vehicle_action =
      xd_uav_controller::InternalCommand::Request::ACTION_RETURN_HOME;
  command.request.action_phase =
      xd_uav_controller::InternalCommand::Request::PHASE_REQUEST_HOVER;
  command.request.action_generation = 23U;
  command.request.return_home = true;

  EXPECT_EQ(23U, command.request.action_generation);
  EXPECT_TRUE(command.request.return_home);
  EXPECT_EQ(4U, command.request.action_phase);
}
