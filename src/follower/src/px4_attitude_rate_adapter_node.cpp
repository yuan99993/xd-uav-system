#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include <follower/FollowerCommand.h>
#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <xd_uav_controller/ControlCommand.h>

namespace {

constexpr double kPi = 3.14159265358979323846;

bool finiteVector(const geometry_msgs::Vector3& value) {
  return std::isfinite(value.x) && std::isfinite(value.y) &&
         std::isfinite(value.z);
}

std::array<double, 3> loadRateLimits(
    const ros::NodeHandle& node, const std::string& name,
    const std::array<double, 3>& fallback) {
  std::vector<double> values;
  if (!node.getParam(name, values) || values.size() != 3) return fallback;
  return {{std::abs(values[0]), std::abs(values[1]),
           std::abs(values[2])}};
}

}  // namespace

class Px4AttitudeRateAdapter {
 public:
  Px4AttitudeRateAdapter() : private_nh_("~") {
    private_nh_.param("vehicle_type", vehicle_type_,
                      std::string("multirotor"));
    if (vehicle_type_ != "multirotor" && vehicle_type_ != "fixedwing") {
      throw std::runtime_error(
          "vehicle_type must be multirotor or fixedwing");
    }
    vehicle_type_id_ = vehicle_type_ == "multirotor"
        ? xd_uav_controller::ControlCommand::VEHICLE_MULTIROTOR
        : xd_uav_controller::ControlCommand::VEHICLE_FIXEDWING;
    expected_profile_ = vehicle_type_ == "multirotor"
        ? "mc_attitude_rate" : "fw_attitude_rate";
    private_nh_.param("command_timeout_sec", command_timeout_, 0.20);
    private_nh_.param("base_command_timeout_sec", base_timeout_, 0.20);
    private_nh_.param("fallback_to_base_controller", fallback_to_base_, true);
    private_nh_.param("enabled_on_start", external_enabled_, false);
    const double default_min_thrust =
        vehicle_type_ == "fixedwing" ? 0.15 : 0.0;
    const double default_max_thrust =
        vehicle_type_ == "fixedwing" ? 0.90 : 1.0;
    private_nh_.param("min_thrust", min_thrust_, default_min_thrust);
    private_nh_.param("max_thrust", max_thrust_, default_max_thrust);
    double publish_rate = vehicle_type_ == "multirotor" ? 100.0 : 50.0;
    private_nh_.param("publish_rate", publish_rate, publish_rate);
    max_rates_ = loadRateLimits(
        private_nh_, "max_body_rate_rad_s",
        vehicle_type_ == "multirotor"
            ? std::array<double, 3>{{1.5, 1.5, 1.5}}
            : std::array<double, 3>{{0.35, 0.25, 0.20}});

    const ros::TransportHints low_latency_transport =
        ros::TransportHints().tcpNoDelay();
    follower_sub_ = nh_.subscribe(
        "follower_command", 1,
        &Px4AttitudeRateAdapter::followerCallback, this,
        low_latency_transport);
    base_sub_ = nh_.subscribe(
        "base_command", 1,
        &Px4AttitudeRateAdapter::baseCallback, this,
        low_latency_transport);
    selected_pub_ = nh_.advertise<xd_uav_controller::ControlCommand>(
        "selected_command", 20);
    enable_server_ = private_nh_.advertiseService(
        "enable_follower", &Px4AttitudeRateAdapter::enableCallback, this);
    timer_ = nh_.createWallTimer(
        ros::WallDuration(1.0 / std::max(2.0, publish_rate)),
        &Px4AttitudeRateAdapter::timerCallback, this);
    ROS_INFO(
        "[FollowerAttitudeAdapter] %s/%s -> XD ControlCommand (%.1fHz)",
        vehicle_type_.c_str(), expected_profile_.c_str(), publish_rate);
  }

 private:
  void followerCallback(const follower::FollowerCommand::ConstPtr& message) {
    std::lock_guard<std::mutex> lock(mutex_);
    follower_ = *message;
    follower_receive_ = ros::WallTime::now();
    have_follower_ = true;
  }

  void baseCallback(
      const xd_uav_controller::ControlCommand::ConstPtr& message) {
    std::lock_guard<std::mutex> lock(mutex_);
    base_ = *message;
    base_receive_ = ros::WallTime::now();
    have_base_ = true;
  }

  bool enableCallback(std_srvs::SetBool::Request& request,
                      std_srvs::SetBool::Response& response) {
    std::lock_guard<std::mutex> lock(mutex_);
    external_enabled_ = request.data;
    // Require a frame received after each transition.  This prevents a stale
    // pre-takeoff altitude setpoint from being selected at handover.
    have_follower_ = false;
    response.success = true;
    response.message = external_enabled_
        ? "follower attitude-rate handover armed; waiting for a fresh command"
        : "follower handover disabled; XD base controller owns the output";
    return true;
  }

  bool validFollower(const follower::FollowerCommand& command,
                     const double age, std::string* reason) const {
    if (age > command_timeout_) {
      *reason = "follower attitude command timeout";
      return false;
    }
    if (!command.command_valid || !command.profile_supported ||
        !command.control_authorized || !command.platform_ready) {
      *reason = "follower command invalid or capability-gated";
      return false;
    }
    if (command.control_mode != "attitude_rate" ||
        command.follower_profile != expected_profile_) {
      *reason = "follower profile/vehicle mismatch";
      return false;
    }
    const std::array<double, 3> rates{{
        command.roll_rate_deg_s * kPi / 180.0,
        command.pitch_rate_deg_s * kPi / 180.0,
        command.yaw_rate_deg_s * kPi / 180.0}};
    if (!std::isfinite(rates[0]) || !std::isfinite(rates[1]) ||
        !std::isfinite(rates[2]) || !std::isfinite(command.thrust)) {
      *reason = "follower command contains non-finite values";
      return false;
    }
    for (size_t axis = 0; axis < rates.size(); ++axis) {
      if (std::abs(rates[axis]) > max_rates_[axis]) {
        *reason = "follower body rate exceeds adapter safety limit";
        return false;
      }
    }
    if (command.thrust < min_thrust_ || command.thrust > max_thrust_) {
      *reason = "follower thrust exceeds adapter safety limit";
      return false;
    }
    return true;
  }

  bool validBase(const xd_uav_controller::ControlCommand& command,
                 const double age) const {
    return age <= base_timeout_ && command.valid &&
           command.vehicle_type == vehicle_type_id_ &&
           finiteVector(command.body_rate) &&
           std::isfinite(command.thrust) && command.thrust >= 0.0 &&
           command.thrust <= 1.0;
  }

  xd_uav_controller::ControlCommand convert(
      const follower::FollowerCommand& command) const {
    xd_uav_controller::ControlCommand output;
    output.header.stamp = ros::Time::now();
    output.vehicle_type = vehicle_type_id_;
    output.body_rate.x = command.roll_rate_deg_s * kPi / 180.0;
    output.body_rate.y = command.pitch_rate_deg_s * kPi / 180.0;
    output.body_rate.z = command.yaw_rate_deg_s * kPi / 180.0;
    output.thrust = command.thrust;
    output.valid = true;
    output.takeoff_active = false;
    output.landing_active = false;
    output.landing_touchdown = false;
    output.controller = "follower_px4_attitude_rate_adapter";
    return output;
  }

  void timerCallback(const ros::WallTimerEvent&) {
    follower::FollowerCommand follower;
    xd_uav_controller::ControlCommand base;
    bool have_follower = false, have_base = false;
    bool external_enabled = false;
    double follower_age = 0.0, base_age = 0.0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      follower = follower_;
      base = base_;
      have_follower = have_follower_;
      have_base = have_base_;
      external_enabled = external_enabled_;
      const ros::WallTime now = ros::WallTime::now();
      follower_age = have_follower ? (now - follower_receive_).toSec() : 1e9;
      base_age = have_base ? (now - base_receive_).toSec() : 1e9;
    }

    std::string follower_reason = "no follower command";
    const bool follower_valid = external_enabled && have_follower &&
        validFollower(follower, follower_age, &follower_reason);
    if (!external_enabled) follower_reason = "follower handover disabled";
    const bool base_valid = have_base && validBase(base, base_age);

    xd_uav_controller::ControlCommand selected;
    // Takeoff and landing always remain owned by the trajectory controller.
    if (base_valid && (base.takeoff_active || base.landing_active)) {
      selected = base;
      selected.header.stamp = ros::Time::now();
    } else if (follower_valid) {
      selected = convert(follower);
    } else if (fallback_to_base_ && base_valid) {
      selected = base;
      selected.header.stamp = ros::Time::now();
      selected.controller += "+follower_fallback";
    } else {
      selected.header.stamp = ros::Time::now();
      selected.vehicle_type = vehicle_type_id_;
      selected.valid = false;
      selected.controller = "follower_px4_attitude_rate_adapter";
      selected.rejection_reason = follower_reason;
      ROS_WARN_THROTTLE(1.0, "[FollowerAttitudeAdapter] %s",
                        follower_reason.c_str());
    }
    selected_pub_.publish(selected);
  }

  ros::NodeHandle nh_, private_nh_;
  ros::Subscriber follower_sub_, base_sub_;
  ros::Publisher selected_pub_;
  ros::ServiceServer enable_server_;
  ros::WallTimer timer_;
  std::mutex mutex_;
  follower::FollowerCommand follower_;
  xd_uav_controller::ControlCommand base_;
  ros::WallTime follower_receive_, base_receive_;
  bool have_follower_{false}, have_base_{false};
  std::string vehicle_type_, expected_profile_;
  uint8_t vehicle_type_id_{0};
  double command_timeout_{0.20}, base_timeout_{0.20};
  double min_thrust_{0.0}, max_thrust_{1.0};
  bool fallback_to_base_{true};
  bool external_enabled_{false};
  std::array<double, 3> max_rates_{{1.5, 1.5, 1.5}};
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "follower_px4_attitude_rate_adapter");
  try {
    Px4AttitudeRateAdapter node;
    ros::spin();
  } catch (const std::exception& exception) {
    ROS_FATAL("[FollowerAttitudeAdapter] %s", exception.what());
    return 1;
  }
  return 0;
}
