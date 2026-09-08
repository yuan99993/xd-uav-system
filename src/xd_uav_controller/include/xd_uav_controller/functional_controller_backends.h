#pragma once

#include <functional>
#include <utility>

#include <xd_uav_controller/controller_backend.h>

namespace xd_uav_controller {

using BackendUpdate = std::function<BackendOutput(
    const ControlState&, const UnifiedReference&, const ros::Time&, double)>;
using BackendReset = std::function<void()>;

class FunctionalControllerBackend : public ControllerBackend {
 public:
  FunctionalControllerBackend(BackendId id, FlightRegime stable_regime,
                              BackendUpdate update, BackendReset reset)
      : id_(id), stable_regime_(stable_regime),
        update_(std::move(update)), reset_(std::move(reset)) {}

  BackendId id() const override { return id_; }
  bool configure(const ros::NodeHandle&) override { return true; }

  bool supportsRegime(const FlightRegime regime) const override {
    return regime == stable_regime_ ||
           (id_ == BackendId::kMultirotor &&
            regime == FlightRegime::kTransitionToForward) ||
           (id_ == BackendId::kFixedWing &&
            regime == FlightRegime::kTransitionToHover);
  }

  bool supportsReference(const ReferenceType type) const override {
    return type != ReferenceType::kUnknown;
  }

  void onActivate(const ControlState&, const UnifiedReference&) override {
    reset();
    active_ = true;
  }

  void onDeactivate() override { active_ = false; }

  void reset() override {
    if (reset_) {
      reset_();
    }
  }

  BackendOutput update(const ControlState& state,
                       const UnifiedReference& reference,
                       const ros::Time& now, const double dt) override {
    if (!active_) {
      BackendOutput output;
      output.reason = "controller backend is not active";
      return output;
    }
    return update_(state, reference, now, dt);
  }

  bool active() const { return active_; }

 private:
  BackendId id_;
  FlightRegime stable_regime_;
  BackendUpdate update_;
  BackendReset reset_;
  bool active_{false};
};

class MultirotorControllerBackend final : public FunctionalControllerBackend {
 public:
  MultirotorControllerBackend(BackendUpdate update, BackendReset reset)
      : FunctionalControllerBackend(BackendId::kMultirotor,
                                    FlightRegime::kHover,
                                    std::move(update), std::move(reset)) {}
};

class FixedWingControllerBackend final : public FunctionalControllerBackend {
 public:
  FixedWingControllerBackend(BackendUpdate update, BackendReset reset)
      : FunctionalControllerBackend(BackendId::kFixedWing,
                                    FlightRegime::kForwardFlight,
                                    std::move(update), std::move(reset)) {}
};

}  // namespace xd_uav_controller
