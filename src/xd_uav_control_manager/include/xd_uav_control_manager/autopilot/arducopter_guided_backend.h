#pragma once

#include <cstdint>
#include <string>

#include <xd_uav_control_manager/autopilot/autopilot_backend.h>

namespace xd_uav_control_manager {

class ArduCopterGuidedBackend final : public AutopilotBackend {
 public:
  ArduCopterGuidedBackend(std::string active_mode, std::string exit_mode,
                         std::string required_parameter,
                         int64_t required_parameter_mask,
                         std::string hover_throttle_parameter,
                         double hover_throttle_tolerance,
                         uint32_t extended_state_message_id,
                         double extended_state_rate_hz);

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
  std::vector<std::string> requiredParameters() const override;
  bool validateParameter(
      const std::string& name,
      const mavros_msgs::ParamValue& value,
      std::string* reason) override;
  std::vector<MessageIntervalRequirement>
  requiredMessageIntervals() const override;

 private:
  std::string active_mode_;
  std::string exit_mode_;
  std::string required_parameter_;
  int64_t required_parameter_mask_{8};
  std::string hover_throttle_parameter_;
  double hover_throttle_tolerance_{0.01};
  uint32_t extended_state_message_id_{245U};
  double extended_state_rate_hz_{10.0};
  bool have_fcu_hover_throttle_{false};
  double fcu_hover_throttle_{0.0};
};

}  // namespace xd_uav_control_manager
