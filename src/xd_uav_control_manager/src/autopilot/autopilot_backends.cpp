#include <xd_uav_control_manager/autopilot/arducopter_guided_backend.h>
#include <xd_uav_control_manager/autopilot/px4_offboard_backend.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace xd_uav_control_manager {
namespace {

bool finiteVector(const geometry_msgs::Vector3& value) {
  return std::isfinite(value.x) && std::isfinite(value.y) &&
         std::isfinite(value.z);
}

bool normalizedQuaternion(const geometry_msgs::Quaternion& input,
                          geometry_msgs::Quaternion* output) {
  tf2::Quaternion quaternion;
  tf2::fromMsg(input, quaternion);
  if (!std::isfinite(quaternion.x()) ||
      !std::isfinite(quaternion.y()) ||
      !std::isfinite(quaternion.z()) ||
      !std::isfinite(quaternion.w()) || quaternion.length2() < 1e-9) {
    return false;
  }
  quaternion.normalize();
  *output = tf2::toMsg(quaternion);
  return true;
}

bool validThrust(const double thrust) {
  return std::isfinite(thrust) && thrust >= 0.0 && thrust <= 1.0;
}

std::string loadExitMode(const ros::NodeHandle& node_handle,
                         const std::string& fallback) {
  std::string exit_mode;
  if (node_handle.getParam("autopilot/exit_mode", exit_mode)) {
    return exit_mode;
  }
  if (node_handle.getParam("offboard/exit/cancel_mode", exit_mode)) {
    ROS_WARN(
        "[xd_uav_control_manager] 参数offboard/exit/cancel_mode已迁移为"
        "autopilot/exit_mode；本次仍兼容旧路径");
    return exit_mode;
  }
  if (node_handle.getParam("offboard/cancel_mode", exit_mode)) {
    ROS_WARN(
        "[xd_uav_control_manager] 参数offboard/cancel_mode已迁移为"
        "autopilot/exit_mode；本次仍兼容旧路径");
    return exit_mode;
  }
  return fallback;
}

void initializeTarget(const std::string& body_frame,
                      const ros::Time& stamp,
                      mavros_msgs::AttitudeTarget* target) {
  *target = mavros_msgs::AttitudeTarget();
  target->header.stamp = stamp;
  target->header.frame_id = body_frame;
  target->orientation.w = 1.0;
}

}  // namespace

Px4OffboardBackend::Px4OffboardBackend(std::string active_mode,
                                       std::string exit_mode)
    : active_mode_(std::move(active_mode)),
      exit_mode_(std::move(exit_mode)) {}

std::string Px4OffboardBackend::name() const { return "px4"; }
const std::string& Px4OffboardBackend::activeMode() const {
  return active_mode_;
}
const std::string& Px4OffboardBackend::exitMode() const {
  return exit_mode_;
}
bool Px4OffboardBackend::supportsAirframe(
    const xd_uav_controller::AirframeType) const {
  return true;
}
bool Px4OffboardBackend::validateCommand(
    const xd_uav_controller::ControlCommand& command,
    std::string* reason) const {
  if (command.output_type != command.OUTPUT_BODY_RATE ||
      !command.body_rate_valid) {
    *reason = "PX4 OFFBOARD要求有效body-rate输出";
    return false;
  }
  if (!finiteVector(command.body_rate) || !validThrust(command.thrust)) {
    *reason = "PX4 OFFBOARD控制量包含非法值";
    return false;
  }
  return true;
}
bool Px4OffboardBackend::buildTarget(
    const xd_uav_controller::ControlCommand& command,
    const std::string& body_frame, const ros::Time& stamp,
    mavros_msgs::AttitudeTarget* target, std::string* reason) const {
  if (!validateCommand(command, reason)) {
    return false;
  }
  initializeTarget(body_frame, stamp, target);
  target->type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;
  target->body_rate = command.body_rate;
  target->thrust = static_cast<float>(command.thrust);
  return true;
}
void Px4OffboardBackend::buildTouchdownTarget(
    const std::string& body_frame, const ros::Time& stamp,
    mavros_msgs::AttitudeTarget* target) const {
  initializeTarget(body_frame, stamp, target);
  target->type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;
}

ArduCopterGuidedBackend::ArduCopterGuidedBackend(
    std::string active_mode, std::string exit_mode,
    std::string required_parameter, const int64_t required_parameter_mask,
    std::string hover_throttle_parameter,
    const double hover_throttle_tolerance,
    const uint32_t extended_state_message_id,
    const double extended_state_rate_hz)
    : active_mode_(std::move(active_mode)),
      exit_mode_(std::move(exit_mode)),
      required_parameter_(std::move(required_parameter)),
      required_parameter_mask_(required_parameter_mask),
      hover_throttle_parameter_(std::move(hover_throttle_parameter)),
      hover_throttle_tolerance_(hover_throttle_tolerance),
      extended_state_message_id_(extended_state_message_id),
      extended_state_rate_hz_(extended_state_rate_hz) {}

std::string ArduCopterGuidedBackend::name() const { return "arducopter"; }
const std::string& ArduCopterGuidedBackend::activeMode() const {
  return active_mode_;
}
const std::string& ArduCopterGuidedBackend::exitMode() const {
  return exit_mode_;
}
bool ArduCopterGuidedBackend::supportsAirframe(
    const xd_uav_controller::AirframeType airframe) const {
  return airframe == xd_uav_controller::AirframeType::kMultirotor;
}
bool ArduCopterGuidedBackend::validateCommand(
    const xd_uav_controller::ControlCommand& command,
    std::string* reason) const {
  if (command.output_type != command.OUTPUT_ATTITUDE ||
      !command.attitude_valid) {
    *reason = "ArduCopter GUIDED要求有效quaternion attitude输出";
    return false;
  }
  geometry_msgs::Quaternion normalized;
  if (!normalizedQuaternion(command.attitude, &normalized) ||
      !validThrust(command.thrust)) {
    *reason = "ArduCopter GUIDED控制量包含非法值";
    return false;
  }
  if (!have_fcu_hover_throttle_) {
    *reason = "尚未读取ArduCopter悬停推力参数";
    return false;
  }
  if (!std::isfinite(command.hover_throttle) ||
      std::abs(command.hover_throttle - fcu_hover_throttle_) >
          hover_throttle_tolerance_) {
    *reason = "控制器hover_throttle与" + hover_throttle_parameter_ +
              "不一致(controller=" +
              std::to_string(command.hover_throttle) + ", fcu=" +
              std::to_string(fcu_hover_throttle_) + ")";
    return false;
  }
  return true;
}
bool ArduCopterGuidedBackend::buildTarget(
    const xd_uav_controller::ControlCommand& command,
    const std::string& body_frame, const ros::Time& stamp,
    mavros_msgs::AttitudeTarget* target, std::string* reason) const {
  if (!validateCommand(command, reason)) {
    return false;
  }
  initializeTarget(body_frame, stamp, target);
  target->type_mask = mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE |
                      mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE |
                      mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE;
  normalizedQuaternion(command.attitude, &target->orientation);
  target->thrust = static_cast<float>(command.thrust);
  return true;
}
void ArduCopterGuidedBackend::buildTouchdownTarget(
    const std::string& body_frame, const ros::Time& stamp,
    mavros_msgs::AttitudeTarget* target) const {
  initializeTarget(body_frame, stamp, target);
  target->type_mask = mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE |
                      mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE |
                      mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE;
}
std::vector<std::string> ArduCopterGuidedBackend::requiredParameters() const {
  return {required_parameter_, hover_throttle_parameter_};
}
bool ArduCopterGuidedBackend::validateParameter(
    const std::string& name, const mavros_msgs::ParamValue& value,
    std::string* reason) {
  if (name == required_parameter_) {
    if ((value.integer & required_parameter_mask_) !=
        required_parameter_mask_) {
      *reason = required_parameter_ + "必须启用raw-thrust位(mask=" +
                std::to_string(required_parameter_mask_) + ")";
      return false;
    }
    return true;
  }
  if (name == hover_throttle_parameter_) {
    if (!std::isfinite(value.real) || value.real <= 0.0 ||
        value.real > 1.0) {
      *reason = hover_throttle_parameter_ + "不在(0,1]内";
      return false;
    }
    fcu_hover_throttle_ = value.real;
    have_fcu_hover_throttle_ = true;
    return true;
  }
  *reason = "未知的ArduCopter能力参数: " + name;
  return false;
}
std::vector<MessageIntervalRequirement>
ArduCopterGuidedBackend::requiredMessageIntervals() const {
  return {{extended_state_message_id_,
           static_cast<float>(extended_state_rate_hz_)}};
}

std::unique_ptr<AutopilotBackend> makeAutopilotBackend(
    const std::string& type, const std::string& active_mode,
    const std::string& exit_mode,
    const std::string& required_parameter,
    const int64_t required_parameter_mask,
    const std::string& hover_throttle_parameter,
    const double hover_throttle_tolerance,
    const uint32_t extended_state_message_id,
    const double extended_state_rate_hz) {
  if (type == "px4") {
    return std::make_unique<Px4OffboardBackend>(active_mode, exit_mode);
  }
  if (type == "arducopter" || type == "apm") {
    return std::make_unique<ArduCopterGuidedBackend>(
        active_mode, exit_mode, required_parameter,
        required_parameter_mask, hover_throttle_parameter,
        hover_throttle_tolerance, extended_state_message_id,
        extended_state_rate_hz);
  }
  throw std::runtime_error("autopilot/type必须是px4或arducopter");
}

std::unique_ptr<AutopilotBackend> loadAutopilotBackend(
    const ros::NodeHandle& private_node_handle) {
  std::string type;
  private_node_handle.param("autopilot/type", type, std::string("px4"));

  if (type == "px4") {
    std::string active_mode;
    private_node_handle.param(
        "autopilot/active_mode", active_mode, std::string("OFFBOARD"));
    const std::string exit_mode =
        loadExitMode(private_node_handle, "POSCTL");
    if (active_mode.empty() || exit_mode.empty()) {
      throw std::runtime_error("PX4模式名不能为空");
    }
    return std::make_unique<Px4OffboardBackend>(active_mode, exit_mode);
  }

  if (type == "arducopter" || type == "apm") {
    std::string active_mode;
    std::string required_parameter;
    std::string hover_throttle_parameter;
    int required_parameter_mask = 8;
    int extended_state_message_id = 245;
    double hover_throttle_tolerance = 0.01;
    double extended_state_rate_hz = 10.0;
    private_node_handle.param(
        "autopilot/active_mode", active_mode, std::string("GUIDED"));
    const std::string exit_mode =
        loadExitMode(private_node_handle, "LOITER");
    private_node_handle.param(
        "autopilot/arducopter/required_parameter", required_parameter,
        std::string("GUID_OPTIONS"));
    private_node_handle.param(
        "autopilot/arducopter/raw_thrust_mask", required_parameter_mask, 8);
    private_node_handle.param(
        "autopilot/arducopter/hover_throttle_parameter",
        hover_throttle_parameter, std::string("MOT_THST_HOVER"));
    private_node_handle.param(
        "autopilot/arducopter/hover_throttle_tolerance",
        hover_throttle_tolerance, 0.01);
    private_node_handle.param(
        "autopilot/arducopter/extended_state_message_id",
        extended_state_message_id, 245);
    private_node_handle.param(
        "autopilot/arducopter/extended_state_rate_hz",
        extended_state_rate_hz, 10.0);
    if (active_mode.empty() || exit_mode.empty() ||
        required_parameter.empty() || required_parameter_mask <= 0 ||
        hover_throttle_parameter.empty() ||
        !std::isfinite(hover_throttle_tolerance) ||
        hover_throttle_tolerance < 0.0 || extended_state_message_id <= 0 ||
        !std::isfinite(extended_state_rate_hz) ||
        extended_state_rate_hz <= 0.0) {
      throw std::runtime_error("ArduCopter后端参数不在有效范围内");
    }
    return std::make_unique<ArduCopterGuidedBackend>(
        active_mode, exit_mode, required_parameter,
        required_parameter_mask, hover_throttle_parameter,
        hover_throttle_tolerance,
        static_cast<uint32_t>(extended_state_message_id),
        extended_state_rate_hz);
  }

  throw std::runtime_error("autopilot/type必须是px4或arducopter");
}

}  // namespace xd_uav_control_manager
