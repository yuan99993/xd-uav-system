#pragma once

#include <string>

#include <xd_uav_controller/ControlCommand.h>
#include <xd_uav_controller/ControlState.h>
#include <xd_uav_controller/control_types.h>

namespace xd_uav_controller {

inline bool isSourceRegimeForObservedTransition(
    const FlightRegime observed_regime,
    const FlightRegime command_regime,
    const AirframeType airframe,
    const BackendId expected_backend) {
  if (!isVtolAirframe(airframe)) {
    return false;
  }
  return
      (observed_regime == FlightRegime::kTransitionToForward &&
       command_regime == FlightRegime::kHover &&
       expected_backend == BackendId::kMultirotor) ||
      (observed_regime == FlightRegime::kTransitionToHover &&
       command_regime == FlightRegime::kForwardFlight &&
       expected_backend == BackendId::kFixedWing);
}

inline bool commandMatchesStateContract(const ControlState& state,
                                        const ControlCommand& command,
                                        const BackendId expected_backend,
                                        std::string* reason) {
  if (command.airframe_type != state.airframe_type) {
    *reason = "airframe_type mismatch";
    return false;
  }
  const auto state_regime =
      static_cast<FlightRegime>(state.flight_regime);
  const auto command_regime =
      static_cast<FlightRegime>(command.flight_regime);
  if (command.flight_regime != state.flight_regime &&
      !isSourceRegimeForObservedTransition(
          state_regime, command_regime,
          static_cast<AirframeType>(state.airframe_type),
          expected_backend)) {
    *reason = "flight_regime mismatch";
    return false;
  }
  if (command.regime_generation != state.regime_generation) {
    *reason = "stale regime_generation";
    return false;
  }
  if (command.action_generation != state.action_generation) {
    *reason = "stale action_generation";
    return false;
  }
  if (command.active_backend != static_cast<uint8_t>(expected_backend)) {
    *reason = "active_backend mismatch";
    return false;
  }
  reason->clear();
  return true;
}

}  // namespace xd_uav_controller
