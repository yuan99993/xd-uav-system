#pragma once

#include <string>

#include <xd_uav_controller/control_types.h>

namespace xd_uav_control_manager {

enum class VtolTakeoffPhase : uint8_t {
  kNone = 0,
  kRequestHover,
  kWaitHover,
  kReady,
  kStarted,
  kFailed,
};

class VtolTakeoffCoordinator {
 public:
  bool start(xd_uav_controller::FlightRegime regime,
             std::string* reason) {
    reset();
    if (regime == xd_uav_controller::FlightRegime::kHover) {
      phase_ = VtolTakeoffPhase::kReady;
      *reason = "hover confirmed; vertical takeoff may start";
      return true;
    }
    if (regime == xd_uav_controller::FlightRegime::kForwardFlight) {
      phase_ = VtolTakeoffPhase::kRequestHover;
      *reason = "forward-flight state; hover transition required";
      return true;
    }
    phase_ = VtolTakeoffPhase::kFailed;
    detail_ = "takeoff requires a stable hover or forward-flight regime";
    *reason = detail_;
    return false;
  }

  void transitionRequested() {
    if (phase_ == VtolTakeoffPhase::kRequestHover) {
      phase_ = VtolTakeoffPhase::kWaitHover;
    }
  }

  bool observe(xd_uav_controller::FlightRegime regime) {
    if ((phase_ == VtolTakeoffPhase::kRequestHover ||
         phase_ == VtolTakeoffPhase::kWaitHover) &&
        regime == xd_uav_controller::FlightRegime::kHover) {
      phase_ = VtolTakeoffPhase::kReady;
      detail_ = "PX4 confirmed hover; waiting for multirotor backend";
      return true;
    }
    return false;
  }

  void commandStarted() {
    if (phase_ == VtolTakeoffPhase::kReady) {
      phase_ = VtolTakeoffPhase::kStarted;
    }
  }

  void fail(const std::string& detail) {
    phase_ = VtolTakeoffPhase::kFailed;
    detail_ = detail;
  }

  void reset() {
    phase_ = VtolTakeoffPhase::kNone;
    detail_.clear();
  }

  bool waitingForHover() const {
    return phase_ == VtolTakeoffPhase::kRequestHover ||
           phase_ == VtolTakeoffPhase::kWaitHover ||
           phase_ == VtolTakeoffPhase::kReady;
  }

  VtolTakeoffPhase phase() const { return phase_; }
  const std::string& detail() const { return detail_; }

 private:
  VtolTakeoffPhase phase_{VtolTakeoffPhase::kNone};
  std::string detail_;
};

}  // namespace xd_uav_control_manager
