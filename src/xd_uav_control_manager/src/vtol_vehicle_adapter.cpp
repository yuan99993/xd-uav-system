#include <xd_uav_control_manager/vtol_vehicle_adapter.h>

#include <mavros_msgs/ExtendedState.h>

namespace xd_uav_control_manager {
namespace control = xd_uav_controller;

VtolVehicleAdapter::VtolVehicleAdapter(const control::AirframeType airframe)
    : airframe_(airframe) {}

control::AirframeType VtolVehicleAdapter::airframeType() const {
  return airframe_;
}

VehicleCapabilities VtolVehicleAdapter::capabilities() const {
  VehicleCapabilities value;
  value.supports_hover = true;
  value.supports_forward_flight = true;
  value.supports_transition = true;
  value.supports_vertical_takeoff = true;
  value.supports_vertical_landing = true;
  value.requires_airspeed_in_forward_flight = true;
  return value;
}

control::FlightRegime VtolVehicleAdapter::observe(
    const uint8_t px4_vtol_state, const bool state_fresh) {
  if (!state_fresh || !control::isVtolAirframe(airframe_)) {
    observed_regime_ = control::FlightRegime::kUnknown;
    return observed_regime_;
  }

  switch (px4_vtol_state) {
    case mavros_msgs::ExtendedState::VTOL_STATE_MC:
      observed_regime_ = control::FlightRegime::kHover;
      break;
    case mavros_msgs::ExtendedState::VTOL_STATE_TRANSITION_TO_FW:
      observed_regime_ = control::FlightRegime::kTransitionToForward;
      break;
    case mavros_msgs::ExtendedState::VTOL_STATE_FW:
      observed_regime_ = control::FlightRegime::kForwardFlight;
      break;
    case mavros_msgs::ExtendedState::VTOL_STATE_TRANSITION_TO_MC:
      observed_regime_ = control::FlightRegime::kTransitionToHover;
      break;
    default:
      observed_regime_ = control::FlightRegime::kUnknown;
      break;
  }

  const bool reached_forward =
      transition_status_.target == control::RequestedRegime::kForwardFlight &&
      observed_regime_ == control::FlightRegime::kForwardFlight;
  const bool reached_hover =
      transition_status_.target == control::RequestedRegime::kHover &&
      observed_regime_ == control::FlightRegime::kHover;
  if (transition_status_.pending && (reached_forward || reached_hover)) {
    transition_status_.pending = false;
    transition_status_.last_target = transition_status_.target;
    transition_status_.target = control::RequestedRegime::kNone;
    transition_status_.last_result = TransitionResult::kCompleted;
    transition_status_.detail = "PX4 confirmed requested regime";
  }
  return observed_regime_;
}

bool VtolVehicleAdapter::beginTransition(const control::RequestedRegime target,
                                         const ros::Time& now,
                                         std::string* reason) {
  if (!control::isVtolAirframe(airframe_)) {
    *reason = "airframe does not support VTOL transition";
    return false;
  }
  if (target == control::RequestedRegime::kNone) {
    *reason = "transition target is none";
    return false;
  }
  if (transition_status_.pending) {
    if (transition_status_.target == target) {
      *reason = "identical transition is already pending";
      return true;
    }
    *reason = "opposite transition is pending";
    return false;
  }
  if ((target == control::RequestedRegime::kForwardFlight &&
       observed_regime_ == control::FlightRegime::kForwardFlight) ||
      (target == control::RequestedRegime::kHover &&
       observed_regime_ == control::FlightRegime::kHover)) {
    *reason = "requested regime is already active";
    return true;
  }
  const bool legal_source =
      (target == control::RequestedRegime::kForwardFlight &&
       observed_regime_ == control::FlightRegime::kHover) ||
      (target == control::RequestedRegime::kHover &&
       observed_regime_ == control::FlightRegime::kForwardFlight);
  if (!legal_source) {
    *reason = "observed regime does not permit requested transition";
    return false;
  }

  transition_status_ = TransitionStatus{};
  transition_status_.pending = true;
  transition_status_.target = target;
  transition_status_.started_at = now;
  transition_status_.detail = "transition request pending";
  *reason = transition_status_.detail;
  return true;
}

void VtolVehicleAdapter::recordServiceAccepted(const uint8_t result) {
  transition_status_.service_accepted = true;
  transition_status_.raw_ack_result = result;
  transition_status_.last_result = TransitionResult::kAccepted;
  transition_status_.detail = "MAVROS accepted transition request";
}

void VtolVehicleAdapter::recordTransportFailure(const std::string& detail) {
  transition_status_.pending = false;
  transition_status_.service_accepted = false;
  transition_status_.last_target = transition_status_.target;
  transition_status_.target = control::RequestedRegime::kNone;
  transition_status_.last_result = TransitionResult::kTransportFailure;
  transition_status_.detail = detail;
}

void VtolVehicleAdapter::recordServiceFailure(const uint8_t result,
                                              const std::string& detail) {
  transition_status_.pending = false;
  transition_status_.service_accepted = false;
  transition_status_.last_target = transition_status_.target;
  transition_status_.target = control::RequestedRegime::kNone;
  transition_status_.raw_ack_result = result;
  transition_status_.last_result = TransitionResult::kAckRejected;
  transition_status_.detail = detail;
}

bool VtolVehicleAdapter::checkTimeout(const ros::Time& now,
                                      const double timeout_seconds) {
  if (!transition_status_.pending || timeout_seconds <= 0.0 ||
      transition_status_.started_at.isZero()) {
    return false;
  }
  if ((now - transition_status_.started_at).toSec() <= timeout_seconds) {
    return false;
  }
  transition_status_.pending = false;
  transition_status_.timed_out = true;
  transition_status_.last_target = transition_status_.target;
  transition_status_.target = control::RequestedRegime::kNone;
  transition_status_.last_result = TransitionResult::kTimedOut;
  transition_status_.detail = "VTOL transition timed out";
  return true;
}

control::FlightRegime VtolVehicleAdapter::observedRegime() const {
  return observed_regime_;
}

const TransitionStatus& VtolVehicleAdapter::transitionStatus() const {
  return transition_status_;
}

}  // namespace xd_uav_control_manager
