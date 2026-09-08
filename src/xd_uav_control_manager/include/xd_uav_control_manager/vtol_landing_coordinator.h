#pragma once

#include <string>

#include <xd_uav_controller/control_types.h>

namespace xd_uav_control_manager {

enum class VtolLandingPhase : uint8_t {
  kNone = 0,
  kRequestHover,
  kWaitHover,
  kVerticalDescent,
  kTouchdownConfirm,
  kDisarm,
  kComplete,
  kFailed,
};

class VtolLandingCoordinator {
 public:
  bool start(xd_uav_controller::FlightRegime regime,
             std::string* reason) {
    reset();
    if (regime == xd_uav_controller::FlightRegime::kHover) {
      phase_ = VtolLandingPhase::kVerticalDescent;
      *reason = "hover confirmed; vertical descent may start";
      return true;
    }
    if (regime == xd_uav_controller::FlightRegime::kForwardFlight) {
      phase_ = VtolLandingPhase::kRequestHover;
      *reason = "forward flight; hover transition required";
      return true;
    }
    phase_ = VtolLandingPhase::kFailed;
    detail_ = "landing requires a stable hover or forward-flight regime";
    *reason = detail_;
    return false;
  }

  void transitionRequested() {
    if (phase_ == VtolLandingPhase::kRequestHover) {
      phase_ = VtolLandingPhase::kWaitHover;
    }
  }

  bool observe(xd_uav_controller::FlightRegime regime) {
    if ((phase_ == VtolLandingPhase::kRequestHover ||
         phase_ == VtolLandingPhase::kWaitHover) &&
        regime == xd_uav_controller::FlightRegime::kHover) {
      phase_ = VtolLandingPhase::kVerticalDescent;
      detail_ = "PX4 confirmed hover; vertical descent may start";
      return true;
    }
    return false;
  }

  void touchdownCandidate() {
    if (phase_ == VtolLandingPhase::kVerticalDescent) {
      phase_ = VtolLandingPhase::kTouchdownConfirm;
    }
  }

  void touchdownConfirmed() {
    if (phase_ == VtolLandingPhase::kTouchdownConfirm) {
      phase_ = VtolLandingPhase::kDisarm;
    }
  }

  void complete() { phase_ = VtolLandingPhase::kComplete; }

  void fail(const std::string& detail) {
    phase_ = VtolLandingPhase::kFailed;
    detail_ = detail;
  }

  void reset() {
    phase_ = VtolLandingPhase::kNone;
    detail_.clear();
  }

  VtolLandingPhase phase() const { return phase_; }
  const std::string& detail() const { return detail_; }

 private:
  VtolLandingPhase phase_{VtolLandingPhase::kNone};
  std::string detail_;
};

}  // namespace xd_uav_control_manager
