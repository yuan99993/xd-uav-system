#pragma once

#include <string>

#include <xd_uav_control_manager/autopilot/autopilot_backend.h>

namespace xd_uav_control_manager {

class Px4OffboardBackend final : public AutopilotBackend {
 public:
  Px4OffboardBackend(std::string active_mode, std::string exit_mode);

  std::string name() const override;
  const std::string& activeMode() const override;
  const std::string& exitMode() const override;
  bool supportsAirframe(
      xd_uav_controller::AirframeType airframe) const override;
  bool validateCommand(
      const xd_uav_controller::ControlCommand& command,
      std::string* reason) const override;
  bool buildTarget(
      const xd_uav_controller::ControlCommand& command,
      const std::string& body_frame, const ros::Time& stamp,
      mavros_msgs::AttitudeTarget* target,
      std::string* reason) const override;
  void buildTouchdownTarget(
      const std::string& body_frame, const ros::Time& stamp,
      mavros_msgs::AttitudeTarget* target) const override;

 private:
  std::string active_mode_;
  std::string exit_mode_;
};

}  // namespace xd_uav_control_manager
