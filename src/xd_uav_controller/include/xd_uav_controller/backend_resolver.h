#pragma once

#include <string>

#include <xd_uav_controller/control_types.h>

namespace xd_uav_controller {

struct BackendResolution {
  bool valid{false};
  BackendId backend{BackendId::kNone};
  BackendId last_stable_backend{BackendId::kNone};
  bool switches_backend{false};
  std::string reason;
};

inline BackendResolution resolveBackend(
    const AirframeType airframe, const FlightRegime regime,
    const BackendId last_stable_backend) {
  BackendResolution result;
  result.last_stable_backend = last_stable_backend;

  if (!supportsRegime(airframe, regime)) {
    result.reason = "illegal airframe and flight-regime combination";
    return result;
  }

  switch (regime) {
    case FlightRegime::kHover:
      result.backend = BackendId::kMultirotor;
      break;
    case FlightRegime::kForwardFlight:
      result.backend = BackendId::kFixedWing;
      break;
    case FlightRegime::kTransitionToForward:
      if (last_stable_backend != BackendId::kMultirotor) {
        result.reason =
            "transition-to-forward requires multirotor source backend";
        return result;
      }
      result.backend = BackendId::kMultirotor;
      break;
    case FlightRegime::kTransitionToHover:
      if (last_stable_backend != BackendId::kFixedWing) {
        result.reason =
            "transition-to-hover requires fixed-wing source backend";
        return result;
      }
      result.backend = BackendId::kFixedWing;
      break;
    case FlightRegime::kUnknown:
      result.reason = "observed flight regime is unknown";
      return result;
  }

  result.valid = true;
  result.switches_backend =
      last_stable_backend != BackendId::kNone &&
      result.backend != last_stable_backend;
  result.reason = "backend resolved";
  return result;
}

}  // namespace xd_uav_controller
