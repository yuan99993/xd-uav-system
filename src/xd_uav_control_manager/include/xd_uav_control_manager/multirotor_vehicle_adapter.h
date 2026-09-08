#pragma once

#include <xd_uav_control_manager/vehicle_adapter.h>

namespace xd_uav_control_manager {

class MultirotorVehicleAdapter final : public VehicleAdapter {
 public:
  xd_uav_controller::AirframeType airframeType() const override {
    return xd_uav_controller::AirframeType::kMultirotor;
  }

  VehicleCapabilities capabilities() const override {
    VehicleCapabilities value;
    value.supports_hover = true;
    value.supports_vertical_takeoff = true;
    value.supports_vertical_landing = true;
    return value;
  }
};

}  // namespace xd_uav_control_manager
