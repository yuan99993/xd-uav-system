#pragma once

#include <xd_uav_control_manager/vehicle_adapter.h>

namespace xd_uav_control_manager {

class FixedWingVehicleAdapter final : public VehicleAdapter {
 public:
  xd_uav_controller::AirframeType airframeType() const override {
    return xd_uav_controller::AirframeType::kFixedWing;
  }

  VehicleCapabilities capabilities() const override {
    VehicleCapabilities value;
    value.supports_forward_flight = true;
    value.requires_airspeed_in_forward_flight = true;
    return value;
  }
};

}  // namespace xd_uav_control_manager
