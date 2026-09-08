#pragma once

#include <cstdint>
#include <string>

#include <ros/time.h>

#include <xd_uav_controller/control_types.h>
#include <xd_uav_control_manager/vehicle_adapter.h>

namespace xd_uav_control_manager {

enum class TransitionResult : uint8_t {
  kNone = 0,
  kTransportFailure,
  kAckRejected,
  kAccepted,
  kCompleted,
  kTimedOut,
};

struct TransitionStatus {
  bool pending{false};
  bool service_accepted{false};
  bool timed_out{false};
  xd_uav_controller::RequestedRegime target{
      xd_uav_controller::RequestedRegime::kNone};
  xd_uav_controller::RequestedRegime last_target{
      xd_uav_controller::RequestedRegime::kNone};
  ros::Time started_at;
  TransitionResult last_result{TransitionResult::kNone};
  uint8_t raw_ack_result{0U};
  std::string detail;
};

class VtolVehicleAdapter : public VehicleAdapter {
 public:
  explicit VtolVehicleAdapter(xd_uav_controller::AirframeType airframe);

  xd_uav_controller::AirframeType airframeType() const override;
  VehicleCapabilities capabilities() const override;

  xd_uav_controller::FlightRegime observe(uint8_t px4_vtol_state,
                                               bool state_fresh);
  bool beginTransition(xd_uav_controller::RequestedRegime target,
                       const ros::Time& now, std::string* reason);
  void recordServiceAccepted(uint8_t result);
  void recordTransportFailure(const std::string& detail);
  void recordServiceFailure(uint8_t result, const std::string& detail);
  bool checkTimeout(const ros::Time& now, double timeout_seconds);

  xd_uav_controller::FlightRegime observedRegime() const;
  const TransitionStatus& transitionStatus() const;

 private:
  xd_uav_controller::AirframeType airframe_;
  xd_uav_controller::FlightRegime observed_regime_{
      xd_uav_controller::FlightRegime::kUnknown};
  TransitionStatus transition_status_;
};

}  // namespace xd_uav_control_manager
