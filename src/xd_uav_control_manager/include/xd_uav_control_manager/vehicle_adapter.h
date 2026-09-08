#pragma once

#include <xd_uav_controller/control_types.h>

namespace xd_uav_control_manager {

struct VehicleCapabilities {
  bool supports_hover{false};
  bool supports_forward_flight{false};
  bool supports_transition{false};
  bool supports_vertical_takeoff{false};
  bool supports_vertical_landing{false};
  bool requires_airspeed_in_forward_flight{false};
};

class VehicleAdapter {
 public:
  virtual ~VehicleAdapter() = default;

  virtual xd_uav_controller::AirframeType airframeType() const = 0;
  virtual VehicleCapabilities capabilities() const = 0;

  virtual bool supportsRegime(
      const xd_uav_controller::FlightRegime regime) const {
    const VehicleCapabilities value = capabilities();
    switch (regime) {
      case xd_uav_controller::FlightRegime::kHover:
        return value.supports_hover;
      case xd_uav_controller::FlightRegime::kForwardFlight:
        return value.supports_forward_flight;
      case xd_uav_controller::FlightRegime::kTransitionToForward:
      case xd_uav_controller::FlightRegime::kTransitionToHover:
        return value.supports_transition;
      case xd_uav_controller::FlightRegime::kUnknown:
        return false;
    }
    return false;
  }

  virtual bool requiresAirspeed(
      const xd_uav_controller::FlightRegime regime) const {
    return capabilities().requires_airspeed_in_forward_flight &&
           (regime ==
                xd_uav_controller::FlightRegime::kForwardFlight ||
            regime ==
                xd_uav_controller::FlightRegime::kTransitionToForward ||
            regime ==
                xd_uav_controller::FlightRegime::kTransitionToHover);
  }
};

}  // namespace xd_uav_control_manager
