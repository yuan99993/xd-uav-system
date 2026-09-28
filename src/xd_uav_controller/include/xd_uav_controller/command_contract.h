#pragma once

#include <string>

#include <xd_uav_controller/ControlCommand.h>
#include <xd_uav_controller/ControlState.h>
#include <xd_uav_controller/control_types.h>

namespace xd_uav_controller {

inline bool commandMatchesStateContract(const ControlState& state,
                                        const ControlCommand& command,
                                        const BackendId expected_backend,
                                        std::string* reason) {
  if (command.airframe_type != state.airframe_type) {
    *reason = "airframe_type mismatch";
    return false;
  }
  if (command.flight_regime != state.flight_regime) {
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
