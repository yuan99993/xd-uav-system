#pragma once

#include <string>

namespace xd_uav_detect {

struct GimbalControlConfig {
  double minimum_yaw_rad{-3.14159};
  double maximum_yaw_rad{3.14159};
  double minimum_pitch_rad{-1.5708};
  double maximum_pitch_rad{1.5708};
  double maximum_yaw_rate_rad_s{1.0};
  double maximum_pitch_rate_rad_s{1.0};
  double command_timeout_sec{0.5};
  double home_yaw_rad{0.0};
  double home_pitch_rad{0.0};
};

struct GimbalControlOutput {
  bool output_valid{false};
  bool joint_state_valid{false};
  bool command_valid{false};
  unsigned char active_mode{0};
  double yaw_rad{0.0};
  double pitch_rad{0.0};
  double yaw_velocity_rad_s{0.0};
  double pitch_velocity_rad_s{0.0};
  double commanded_yaw_rad{0.0};
  double commanded_pitch_rad{0.0};
  std::string status{"waiting_for_joint_state"};
};

class GimbalCommandController {
 public:
  enum Mode : unsigned char {
    kPosition = 0,
    kVelocity = 1,
    kHome = 2,
  };

  explicit GimbalCommandController(const GimbalControlConfig& config);

  void setJointState(double yaw_rad, double pitch_rad,
                     double yaw_velocity_rad_s,
                     double pitch_velocity_rad_s, double now_sec);

  bool acceptCommand(unsigned char mode, double yaw_rad, double pitch_rad,
                     double yaw_rate_rad_s, double pitch_rate_rad_s,
                     double now_sec, std::string* rejection_reason);

  GimbalControlOutput update(double now_sec);

 private:
  static double clamp(double value, double minimum, double maximum);
  static double approach(double current, double target, double maximum_step);

  GimbalControlConfig config_;
  bool have_joint_state_{false};
  bool have_command_{false};
  unsigned char active_mode_{kPosition};
  double measured_yaw_rad_{0.0};
  double measured_pitch_rad_{0.0};
  double measured_yaw_velocity_rad_s_{0.0};
  double measured_pitch_velocity_rad_s_{0.0};
  double commanded_yaw_rad_{0.0};
  double commanded_pitch_rad_{0.0};
  double requested_yaw_rad_{0.0};
  double requested_pitch_rad_{0.0};
  double requested_yaw_rate_rad_s_{0.0};
  double requested_pitch_rate_rad_s_{0.0};
  double last_command_sec_{0.0};
  double last_update_sec_{0.0};
};

}  // namespace xd_uav_detect
