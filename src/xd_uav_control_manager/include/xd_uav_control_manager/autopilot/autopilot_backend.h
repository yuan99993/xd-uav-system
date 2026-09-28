#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/ParamValue.h>
#include <ros/node_handle.h>
#include <ros/time.h>

#include <xd_uav_controller/ControlCommand.h>
#include <xd_uav_controller/control_types.h>

namespace xd_uav_control_manager {

struct MessageIntervalRequirement {
  uint32_t message_id{0U};
  float rate_hz{0.0F};
};

class AutopilotBackend {
 public:
  virtual ~AutopilotBackend() = default;

  virtual std::string name() const = 0;
  virtual const std::string& activeMode() const = 0;
  virtual const std::string& exitMode() const = 0;
  virtual bool supportsAirframe(
      xd_uav_controller::AirframeType airframe) const = 0;
  virtual bool validateCommand(
      const xd_uav_controller::ControlCommand& command,
      std::string* reason) const = 0;
  virtual bool buildTarget(
      const xd_uav_controller::ControlCommand& command,
      const std::string& body_frame, const ros::Time& stamp,
      mavros_msgs::AttitudeTarget* target,
      std::string* reason) const = 0;
  virtual void buildTouchdownTarget(
      const std::string& body_frame, const ros::Time& stamp,
      mavros_msgs::AttitudeTarget* target) const = 0;

  virtual std::vector<std::string> requiredParameters() const {
    return {};
  }
  virtual bool validateParameter(
      const std::string&, const mavros_msgs::ParamValue&,
      std::string*) {
    return true;
  }
  virtual std::vector<MessageIntervalRequirement>
  requiredMessageIntervals() const {
    return {};
  }
};

std::unique_ptr<AutopilotBackend> makeAutopilotBackend(
    const std::string& type, const std::string& active_mode,
    const std::string& exit_mode,
    const std::string& required_parameter,
    int64_t required_parameter_mask,
    const std::string& hover_throttle_parameter,
    double hover_throttle_tolerance,
    uint32_t extended_state_message_id,
    double extended_state_rate_hz);

// Load and validate only the parameters used by the selected backend.  This
// keeps FCU-specific configuration out of the common safety state machine.
std::unique_ptr<AutopilotBackend> loadAutopilotBackend(
    const ros::NodeHandle& private_node_handle);

}  // namespace xd_uav_control_manager
