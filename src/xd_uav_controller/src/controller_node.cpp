#include <algorithm>
#include <array>
#include <clocale>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/PositionTarget.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <std_msgs/Bool.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include <xd_uav_controller/ControlCommand.h>
#include <xd_uav_controller/ControlState.h>
#include <xd_uav_controller/InternalCommand.h>
#include <xd_uav_controller/Takeoff.h>
#include <std_srvs/Trigger.h>

namespace {

constexpr double kPi = 3.14159265358979323846;

double clamp(const double value, const double minimum,
             const double maximum) {
  return std::max(minimum, std::min(maximum, value));
}

double wrapAngle(const double value) {
  return std::atan2(std::sin(value), std::cos(value));
}

bool finite(const geometry_msgs::Vector3& value) {
  return std::isfinite(value.x) && std::isfinite(value.y) &&
         std::isfinite(value.z);
}

bool finite(const geometry_msgs::Point& value) {
  return std::isfinite(value.x) && std::isfinite(value.y) &&
         std::isfinite(value.z);
}

using AxisMask = std::array<bool, 3>;

bool anyAxis(const AxisMask& mask) {
  return mask[0] || mask[1] || mask[2];
}

bool finiteSelected(
    const geometry_msgs::Point& value,
    const AxisMask& mask) {
  return (!mask[0] || std::isfinite(value.x)) &&
         (!mask[1] || std::isfinite(value.y)) &&
         (!mask[2] || std::isfinite(value.z));
}

bool finiteSelected(
    const geometry_msgs::Vector3& value,
    const AxisMask& mask) {
  return (!mask[0] || std::isfinite(value.x)) &&
         (!mask[1] || std::isfinite(value.y)) &&
         (!mask[2] || std::isfinite(value.z));
}

bool quaternionToMatrix(const geometry_msgs::Quaternion& message,
                        Eigen::Matrix3d* rotation) {
  tf2::Quaternion quaternion;
  tf2::fromMsg(message, quaternion);
  if (!std::isfinite(quaternion.x()) ||
      !std::isfinite(quaternion.y()) ||
      !std::isfinite(quaternion.z()) ||
      !std::isfinite(quaternion.w()) ||
      quaternion.length2() < 1e-9) {
    return false;
  }
  quaternion.normalize();
  tf2::Matrix3x3 matrix(quaternion);
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      (*rotation)(row, column) = matrix[row][column];
    }
  }
  return rotation->allFinite();
}

std::array<double, 3> loadVector3(
    const ros::NodeHandle& node_handle, const std::string& name,
    const std::array<double, 3>& fallback) {
  std::vector<double> values;
  if (!node_handle.getParam(name, values)) {
    return fallback;
  }
  if (values.size() != 3) {
    throw std::runtime_error(name + "必须包含3个数值");
  }
  return {{values[0], values[1], values[2]}};
}

std::array<double, 2> loadVector2(
    const ros::NodeHandle& node_handle, const std::string& name,
    const std::array<double, 2>& fallback) {
  std::vector<double> values;
  if (!node_handle.getParam(name, values)) {
    return fallback;
  }
  if (values.size() != 2) {
    throw std::runtime_error(name + "必须包含2个数值");
  }
  return {{values[0], values[1]}};
}

class FiniteHorizonAxisMpc {
 public:
  void configure(const int horizon, const double dt,
                 const std::array<double, 2>& q,
                 const std::array<double, 2>& terminal_q,
                 const double r) {
    horizon_ = std::max(2, horizon);
    dt_ = std::max(1e-3, dt);
    q_ = q;
    terminal_q_ = terminal_q;
    r_ = std::max(1e-6, r);
  }

  double acceleration(
      const Eigen::Vector2d& state,
      const Eigen::Vector2d& reference,
      const std::array<bool, 2>& controlled,
      const double acceleration_feedforward) const {
    Eigen::Matrix2d a;
    const double dt2 = dt_ * dt_;
    a << 1.0, dt_,
         0.0, 1.0;
    Eigen::Vector2d b(0.5 * dt2, dt_);

    Eigen::Matrix2d q = Eigen::Matrix2d::Zero();
    Eigen::Matrix2d p = Eigen::Matrix2d::Zero();
    for (int index = 0; index < 2; ++index) {
      if (controlled[index]) {
        q(index, index) = std::max(0.0, q_[index]);
        p(index, index) = std::max(0.0, terminal_q_[index]);
      }
    }

    Eigen::RowVector2d first_gain = Eigen::RowVector2d::Zero();
    for (int step = horizon_ - 1; step >= 0; --step) {
      const double denominator = r_ + (b.transpose() * p * b)(0, 0);
      const Eigen::RowVector2d gain =
          (b.transpose() * p * a) / denominator;
      p = q + a.transpose() * p * (a - b * gain);
      if (step == 0) {
        first_gain = gain;
      }
    }
    return -first_gain.dot(state - reference) +
           acceleration_feedforward;
  }

 private:
  int horizon_{26};
  double dt_{0.01};
  double r_{20.0};
  std::array<double, 2> q_{{500.0, 100.0}};
  std::array<double, 2> terminal_q_{{1000.0, 300.0}};
};

struct ControllerResult {
  Eigen::Vector3d body_rate{Eigen::Vector3d::Zero()};
  double thrust{0.0};
  bool valid{false};
  std::string reason;
};

struct Reference {
  std_msgs::Header header;
  geometry_msgs::Point position;
  geometry_msgs::Vector3 velocity;
  geometry_msgs::Vector3 acceleration;
  geometry_msgs::Vector3 jerk;
  double yaw{0.0};
  double yaw_rate{0.0};
  AxisMask use_position{{false, false, false}};
  AxisMask use_velocity{{false, false, false}};
  AxisMask use_acceleration{{false, false, false}};
  bool use_jerk{false};
  bool use_yaw{false};
  bool use_yaw_rate{false};
  bool trajectory_reference{false};
};

}  // namespace

class ControllerNode {
 public:
  ControllerNode()
      : private_nh_("~"), tf_listener_(tf_buffer_) {
    loadParameters();
    state_subscriber_ = nh_.subscribe(
        "state", 20, &ControllerNode::stateCallback, this);
    reference_position_target_subscriber_ = nh_.subscribe(
        "reference_position_target", 20,
        &ControllerNode::referencePositionTargetCallback, this);
    reference_position_target_publisher_ =
        nh_.advertise<mavros_msgs::PositionTarget>(
            "reference_position_target", 1, true);
    reference_trajectory_subscriber_ = nh_.subscribe(
        "reference_trajectory", 5,
        &ControllerNode::referenceTrajectoryCallback, this);
    simple_goal_subscriber_ = nh_.subscribe(
        "simple_goal", 5,
        &ControllerNode::simpleGoalCallback, this);
    local_alignment_valid_subscriber_ = nh_.subscribe(
        "local_alignment_valid", 5,
        &ControllerNode::localAlignmentValidCallback, this);
    reference_trajectory_path_publisher_ =
        nh_.advertise<nav_msgs::Path>(
            "reference_trajectory_path", 1, true);
    command_publisher_ =
        nh_.advertise<xd_uav_controller::ControlCommand>("command", 20);
    internal_command_server_ = private_nh_.advertiseService(
        "internal/command",
        &ControllerNode::internalCommandCallback, this);
    timer_ = private_nh_.createTimer(
        ros::Duration(1.0 / std::max(1.0, control_rate_)),
        &ControllerNode::timerCallback, this);
    ROS_INFO("[xd_uav_controller] 已加载%s控制器",
             vehicle_type_.c_str());
  }

 private:
  void loadParameters() {
    private_nh_.param("vehicle_type", vehicle_type_,
                      std::string("multirotor"));
    if (vehicle_type_ != "multirotor" &&
        vehicle_type_ != "fixedwing") {
      throw std::runtime_error(
          "vehicle_type必须是multirotor或fixedwing");
    }
    vehicle_type_id_ =
        vehicle_type_ == "multirotor"
            ? xd_uav_controller::ControlState::VEHICLE_MULTIROTOR
            : xd_uav_controller::ControlState::VEHICLE_FIXEDWING;
    private_nh_.param("control_rate", control_rate_,
                      vehicle_type_ == "multirotor" ? 100.0 : 50.0);
    private_nh_.param("state_timeout", state_timeout_, 0.20);
    private_nh_.param("reference_timeout", reference_timeout_, 0.50);
    private_nh_.param("reference_frames/transform_timeout",
                      reference_transform_timeout_, 0.03);
    private_nh_.param("reference_frames/max_transform_age",
                      maximum_transform_age_, 0.50);
    private_nh_.param("reference_frames/failure_grace_duration",
                      reference_transform_failure_grace_, 0.50);
    private_nh_.param("reference_frames/max_inertial_frame_tilt",
                      maximum_inertial_frame_tilt_, 0.10);
    private_nh_.param("reference_frames/require_global_alignment",
                      require_global_alignment_, true);
    private_nh_.param("reference_frames/global_alignment_timeout",
                      global_alignment_timeout_, 1.0);
    if (!private_nh_.getParam(
            "reference_frames/allowed",
            allowed_reference_frames_)) {
      allowed_reference_frames_ = {
          "local_origin", "/world", "/map"};
    }
    if (!private_nh_.getParam(
            "reference_frames/global_alignment_frames",
            global_alignment_frames_)) {
      global_alignment_frames_ = {
          "local_origin", "/world", "/map"};
    }
    private_nh_.param("simple_goal/use_message_z",
                      simple_goal_use_message_z_, false);

    if (vehicle_type_ == "multirotor") {
      private_nh_.param("multirotor/gravity", gravity_, 9.80665);
      private_nh_.param("multirotor/hover_throttle",
                        hover_throttle_, 0.5);
      private_nh_.param("multirotor/min_throttle",
                        minimum_throttle_, 0.05);
      private_nh_.param("multirotor/max_throttle",
                        maximum_throttle_, 0.9);
      private_nh_.param("multirotor/max_tilt", maximum_tilt_, 0.61);
      attitude_gain_ = loadVector3(
          private_nh_, "multirotor/attitude_gain",
          {{5.0, 5.0, 2.0}});
      maximum_body_rate_ = loadVector3(
          private_nh_, "multirotor/max_body_rate",
          {{3.0, 3.0, 2.0}});

      int horizon = 26;
      const double dt =
          1.0 / std::max(1.0, control_rate_);
      double r = 20.0;
      private_nh_.param("multirotor/mpc/horizon", horizon, 26);
      private_nh_.param("multirotor/mpc/r", r, 20.0);
      const auto q = loadVector2(
          private_nh_, "multirotor/mpc/q",
          {{500.0, 100.0}});
      const auto terminal_q = loadVector2(
          private_nh_, "multirotor/mpc/terminal_q",
          {{1000.0, 300.0}});
      mpc_.configure(horizon, dt, q, terminal_q, r);
      mpc_dt_ = dt;
      private_nh_.param("multirotor/mpc/max_velocity_xy",
                        max_velocity_xy_, 5.0);
      private_nh_.param("multirotor/mpc/max_velocity_z",
                        max_velocity_z_, 3.0);
      private_nh_.param("multirotor/mpc/max_acceleration_xy",
                        max_acceleration_xy_, 5.0);
      private_nh_.param("multirotor/mpc/max_acceleration_z",
                        max_acceleration_z_, 4.0);
      private_nh_.param("multirotor/mpc/max_jerk_xy",
                        max_jerk_xy_, 8.0);
      private_nh_.param("multirotor/mpc/max_jerk_z",
                        max_jerk_z_, 6.0);
      velocity_integral_gain_ = loadVector3(
          private_nh_,
          "multirotor/mpc/velocity_integral_gain",
          {{1.0, 1.0, 2.0}});
      velocity_integral_acceleration_limit_ = loadVector3(
          private_nh_,
          "multirotor/mpc/velocity_integral_acceleration_limit",
          {{1.0, 1.0, 1.0}});
      acceleration_feedback_gain_ = loadVector3(
          private_nh_,
          "multirotor/mpc/acceleration_feedback_gain",
          {{0.25, 0.25, 0.35}});
      acceleration_integral_gain_ = loadVector3(
          private_nh_,
          "multirotor/mpc/acceleration_integral_gain",
          {{0.20, 0.20, 0.40}});
      acceleration_integral_acceleration_limit_ = loadVector3(
          private_nh_,
          "multirotor/mpc/acceleration_integral_acceleration_limit",
          {{1.0, 1.0, 1.0}});
    } else {
      private_nh_.param("fixedwing/gravity", gravity_, 9.80665);
      private_nh_.param("fixedwing/cruise_airspeed",
                        cruise_airspeed_, 15.0);
      private_nh_.param("fixedwing/minimum_airspeed",
                        minimum_airspeed_, 11.0);
      private_nh_.param("fixedwing/maximum_airspeed",
                        maximum_airspeed_, 24.0);
      private_nh_.param("fixedwing/trim_throttle",
                        trim_throttle_, 0.25);
      private_nh_.param("fixedwing/min_throttle",
                        minimum_throttle_, 0.05);
      private_nh_.param("fixedwing/max_throttle",
                        maximum_throttle_, 1.0);
      private_nh_.param("fixedwing/airspeed_throttle_gain",
                        airspeed_throttle_gain_, 0.08);
      private_nh_.param("fixedwing/course_time_constant",
                        course_time_constant_, 3.0);
      private_nh_.param(
          "fixedwing/guidance/lookahead_distance",
          fixedwing_guidance_lookahead_distance_, 60.0);
      private_nh_.param(
          "fixedwing/guidance/adaptive_lookahead",
          fixedwing_adaptive_lookahead_, true);
      private_nh_.param(
          "fixedwing/guidance/minimum_lookahead_distance",
          fixedwing_minimum_lookahead_distance_, 30.0);
      private_nh_.param(
          "fixedwing/guidance/maximum_lookahead_distance",
          fixedwing_maximum_lookahead_distance_, 60.0);
      private_nh_.param(
          "fixedwing/guidance/turn_radius_gain",
          fixedwing_lookahead_turn_radius_gain_, 1.2);
      private_nh_.param(
          "fixedwing/guidance/lookahead_filter_time_constant",
          fixedwing_lookahead_filter_time_constant_, 0.50);
      private_nh_.param("fixedwing/altitude_time_constant",
                        altitude_time_constant_, 5.0);
      private_nh_.param("fixedwing/altitude_integral_gain",
                        altitude_integral_gain_, 0.02);
      private_nh_.param(
          "fixedwing/altitude_integral_climb_rate_limit",
          altitude_integral_climb_rate_limit_, 1.0);
      private_nh_.param("fixedwing/climb_rate_pitch_gain",
                        climb_rate_pitch_gain_, 0.06);
      private_nh_.param(
          "fixedwing/climb_rate_filter_time_constant",
          climb_rate_filter_time_constant_, 0.50);
      private_nh_.param(
          "fixedwing/vertical_acceleration_pitch_rate_gain",
          vertical_acceleration_pitch_rate_gain_, 1.0);
      private_nh_.param("fixedwing/climb_rate_throttle_gain",
                        climb_rate_throttle_gain_, 0.03);
      private_nh_.param("fixedwing/roll_time_constant",
                        roll_time_constant_, 0.45);
      private_nh_.param("fixedwing/pitch_time_constant",
                        pitch_time_constant_, 0.45);
      private_nh_.param("fixedwing/max_roll", max_roll_, 0.79);
      private_nh_.param("fixedwing/max_pitch", max_pitch_, 0.35);
      private_nh_.param("fixedwing/max_climb_rate",
                        max_climb_rate_, 4.0);
      maximum_body_rate_ = loadVector3(
          private_nh_, "fixedwing/max_body_rate",
          {{2.0, 1.5, 1.5}});
      private_nh_.param("fixedwing/loiter/radius",
                        fixedwing_loiter_radius_, 80.0);
      private_nh_.param("fixedwing/loiter/airspeed",
                        fixedwing_loiter_airspeed_, 15.0);
      private_nh_.param("fixedwing/loiter/direction",
                        fixedwing_loiter_direction_, 1);
      private_nh_.param("fixedwing/loiter/radial_gain",
                        fixedwing_loiter_radial_gain_, 1.0);
      private_nh_.param(
          "fixedwing/loiter/max_course_correction",
          fixedwing_loiter_max_course_correction_, 0.70);
      if (fixedwing_loiter_radius_ < 5.0) {
        throw std::runtime_error(
            "fixedwing/loiter/radius必须不小于5米");
      }
      if (fixedwing_guidance_lookahead_distance_ < 5.0) {
        throw std::runtime_error(
            "fixedwing/guidance/lookahead_distance"
            "必须不小于5米");
      }
      fixedwing_loiter_airspeed_ = clamp(
          fixedwing_loiter_airspeed_,
          minimum_airspeed_, maximum_airspeed_);
      fixedwing_loiter_direction_ =
          fixedwing_loiter_direction_ >= 0 ? 1 : -1;
      fixedwing_loiter_radial_gain_ =
          std::max(0.0, fixedwing_loiter_radial_gain_);
      fixedwing_loiter_max_course_correction_ = clamp(
          fixedwing_loiter_max_course_correction_,
          0.0, 0.5 * kPi);
    }

    private_nh_.param("takeoff/default_altitude",
                      takeoff_default_altitude_,
                      vehicle_type_ == "multirotor" ? 2.0 : 30.0);
    private_nh_.param("takeoff/max_velocity",
                      takeoff_max_velocity_, 0.8);
    private_nh_.param("takeoff/max_acceleration",
                      takeoff_max_acceleration_, 0.6);
    private_nh_.param("takeoff/position_tolerance",
                      takeoff_position_tolerance_, 0.15);
    private_nh_.param("takeoff/velocity_tolerance",
                      takeoff_velocity_tolerance_, 0.15);
    private_nh_.param("takeoff/takeoff_throttle",
                      fixedwing_takeoff_throttle_, 0.9);
    private_nh_.param("takeoff/rotate_airspeed",
                      fixedwing_rotate_airspeed_, 12.0);
    private_nh_.param("takeoff/climb_airspeed",
                      fixedwing_climb_airspeed_, 15.0);
    private_nh_.param("takeoff/climb_pitch",
                      fixedwing_climb_pitch_, 0.21);
    private_nh_.param("takeoff/altitude_tolerance",
                      fixedwing_altitude_tolerance_, 3.0);
    private_nh_.param("landing/descent_velocity",
                      landing_descent_velocity_, 0.35);
    private_nh_.param("landing/final_descent_velocity",
                      landing_final_descent_velocity_, 0.15);
    private_nh_.param("landing/slow_height",
                      landing_slow_height_, 0.7);
    private_nh_.param("landing/return_velocity",
                      landing_return_velocity_, 1.0);
    private_nh_.param("landing/position_tolerance",
                      landing_position_tolerance_, 0.20);
    private_nh_.param("landing/velocity_tolerance",
                      landing_velocity_tolerance_, 0.20);
    private_nh_.param("landing/touchdown_offset",
                      landing_touchdown_offset_, 0.15);
    private_nh_.param("landing/touchdown_height_tolerance",
                      landing_touchdown_height_tolerance_, 0.10);
    private_nh_.param("landing/touchdown_velocity_tolerance",
                      landing_touchdown_velocity_tolerance_, 0.20);
    private_nh_.param("landing/approach_distance",
                      fixedwing_landing_approach_distance_, 400.0);
    private_nh_.param("landing/approach_height",
                      fixedwing_landing_approach_height_, 30.0);
    private_nh_.param("landing/approach_airspeed",
                      fixedwing_landing_approach_airspeed_, 15.0);
    private_nh_.param("landing/approach_acceptance_radius",
                      fixedwing_landing_approach_acceptance_radius_,
                      50.0);
    private_nh_.param("landing/approach_altitude_tolerance",
                      fixedwing_landing_approach_altitude_tolerance_,
                      8.0);
    private_nh_.param("landing/approach_course_tolerance",
                      fixedwing_landing_approach_course_tolerance_,
                      0.52);
    private_nh_.param(
        "landing/approach_capture_turn_radius_factor",
        fixedwing_landing_approach_capture_turn_radius_factor_,
        2.0);
    private_nh_.param("landing/line_lookahead_distance",
                      fixedwing_landing_line_lookahead_distance_,
                      80.0);
    private_nh_.param("landing/max_course_correction",
                      fixedwing_landing_max_course_correction_,
                      0.70);
    private_nh_.param("landing/glide_slope_angle",
                      fixedwing_landing_glide_slope_angle_, 0.08);
    private_nh_.param("landing/local_touchdown_ahead",
                      fixedwing_landing_local_touchdown_ahead_, 400.0);
    private_nh_.param("landing/rollout_distance",
                      fixedwing_landing_rollout_distance_, 200.0);
    private_nh_.param("landing/max_roll",
                      fixedwing_landing_max_roll_, 0.35);
    private_nh_.param("landing/touchdown_height_tolerance",
                      fixedwing_landing_touchdown_height_tolerance_,
                      1.0);
    private_nh_.param("landing/touchdown_groundspeed",
                      fixedwing_landing_touchdown_groundspeed_, 2.0);
    private_nh_.param("landing/touchdown_vertical_speed",
                      fixedwing_landing_touchdown_vertical_speed_, 2.0);
    private_nh_.param("home/mode", home_mode_,
                      std::string("takeoff"));
    private_nh_.param("home/frame", home_frame_config_,
                      std::string("local_origin"));
    const auto fixed_home_position = loadVector3(
        private_nh_, "home/fixed_position",
        {{0.0, 0.0, 0.0}});
    fixed_home_position_ = Eigen::Vector3d(
        fixed_home_position[0],
        fixed_home_position[1],
        fixed_home_position[2]);
    private_nh_.param("home/fixed_yaw",
                      fixed_home_yaw_, 0.0);
    private_nh_.param("home/use_home_yaw",
                      use_home_yaw_, false);
    if (home_mode_ != "takeoff" &&
        home_mode_ != "fixed_local") {
      throw std::runtime_error(
          "home/mode必须是takeoff或fixed_local");
    }
    if (fixedwing_landing_approach_distance_ < 50.0 ||
        fixedwing_landing_approach_height_ <= 0.0 ||
        fixedwing_landing_approach_airspeed_ <= 0.0 ||
        fixedwing_landing_glide_slope_angle_ <= 0.0 ||
        fixedwing_landing_glide_slope_angle_ >= 0.35 ||
        fixedwing_landing_local_touchdown_ahead_ < 50.0 ||
        fixedwing_landing_rollout_distance_ < 50.0 ||
        fixedwing_landing_approach_capture_turn_radius_factor_ <
            1.0 ||
        fixedwing_landing_line_lookahead_distance_ < 10.0 ||
        fixedwing_landing_max_course_correction_ <= 0.0 ||
        fixedwing_landing_max_course_correction_ >= 1.4) {
      throw std::runtime_error(
          "固定翼landing参数不在安全范围内");
    }
    if (vehicle_type_ == "fixedwing" &&
        (fixedwing_guidance_lookahead_distance_ <= 0.0 ||
         fixedwing_minimum_lookahead_distance_ <= 0.0 ||
         fixedwing_maximum_lookahead_distance_ <
             fixedwing_minimum_lookahead_distance_ ||
         fixedwing_lookahead_turn_radius_gain_ <= 0.0 ||
         fixedwing_lookahead_filter_time_constant_ < 0.0 ||
         altitude_time_constant_ <= 0.0 ||
         altitude_integral_gain_ < 0.0 ||
         altitude_integral_climb_rate_limit_ < 0.0 ||
         climb_rate_pitch_gain_ < 0.0 ||
         climb_rate_filter_time_constant_ < 0.0 ||
         vertical_acceleration_pitch_rate_gain_ < 0.0 ||
         climb_rate_throttle_gain_ < 0.0 ||
         !std::isfinite(fixedwing_guidance_lookahead_distance_) ||
         !std::isfinite(fixedwing_minimum_lookahead_distance_) ||
         !std::isfinite(fixedwing_maximum_lookahead_distance_) ||
         !std::isfinite(fixedwing_lookahead_turn_radius_gain_) ||
         !std::isfinite(fixedwing_lookahead_filter_time_constant_) ||
         !std::isfinite(altitude_time_constant_) ||
         !std::isfinite(altitude_integral_gain_) ||
         !std::isfinite(altitude_integral_climb_rate_limit_) ||
         !std::isfinite(climb_rate_pitch_gain_) ||
         !std::isfinite(climb_rate_filter_time_constant_) ||
         !std::isfinite(vertical_acceleration_pitch_rate_gain_) ||
         !std::isfinite(climb_rate_throttle_gain_))) {
      throw std::runtime_error(
          "固定翼guidance/高度控制参数不在安全范围内");
    }
    fixedwing_landing_approach_airspeed_ = clamp(
        fixedwing_landing_approach_airspeed_,
        minimum_airspeed_, maximum_airspeed_);
    fixedwing_landing_max_roll_ = clamp(
        std::abs(fixedwing_landing_max_roll_),
        0.05, max_roll_);
  }

  static std::string canonicalFrame(const std::string& frame) {
    const std::size_t first = frame.find_first_not_of('/');
    return first == std::string::npos ? std::string()
                                     : frame.substr(first);
  }

  static std::string frameParent(const std::string& frame) {
    const std::string canonical = canonicalFrame(frame);
    const std::size_t separator = canonical.find_last_of('/');
    return separator == std::string::npos
               ? std::string()
               : canonical.substr(0, separator);
  }

  static std::string frameLeaf(const std::string& frame) {
    const std::string canonical = canonicalFrame(frame);
    const std::size_t separator = canonical.find_last_of('/');
    return separator == std::string::npos
               ? canonical
               : canonical.substr(separator + 1);
  }

  static std::string resolveConfiguredFrame(
      const std::string& configured,
      const std::string& control_frame) {
    const std::string canonical =
        canonicalFrame(configured);
    if (canonical.empty()) {
      return std::string();
    }
    if (configured.front() == '/' ||
        canonical.find('/') != std::string::npos) {
      return canonical;
    }
    const std::string parent =
        frameParent(control_frame);
    return parent.empty()
               ? canonical
               : parent + "/" + canonical;
  }

  bool matchesConfiguredFrame(
      const std::string& frame,
      const std::string& configured,
      const std::string& control_frame) const {
    const std::string canonical = canonicalFrame(frame);
    const std::string pattern = canonicalFrame(configured);
    if (canonical.empty() || pattern.empty()) {
      return false;
    }
    if (configured.front() == '/' ||
        pattern.find('/') != std::string::npos) {
      return canonical == pattern;
    }
    return frameLeaf(canonical) == pattern &&
           frameParent(canonical) ==
               frameParent(control_frame);
  }

  bool referenceFrameAllowed(
      const std::string& frame,
      const std::string& control_frame) const {
    if (canonicalFrame(frame) ==
        canonicalFrame(control_frame)) {
      return true;
    }
    for (const std::string& configured :
         allowed_reference_frames_) {
      if (matchesConfiguredFrame(
              frame, configured, control_frame)) {
        return true;
      }
    }
    return false;
  }

  bool globalAlignmentRequired(
      const std::string& frame,
      const std::string& control_frame) const {
    if (!require_global_alignment_ ||
        canonicalFrame(frame) ==
            canonicalFrame(control_frame)) {
      return false;
    }
    for (const std::string& configured :
         global_alignment_frames_) {
      if (matchesConfiguredFrame(
              frame, configured, control_frame)) {
        return true;
      }
    }
    return false;
  }

  bool globalAlignmentReady(
      const std::string& frame,
      const std::string& control_frame,
      std::string* reason) const {
    if (!globalAlignmentRequired(frame, control_frame)) {
      return true;
    }
    if (!have_local_alignment_valid_) {
      *reason = "尚未收到local_alignment_valid";
      return false;
    }
    if (!local_alignment_valid_) {
      *reason = "local/global参考坐标系对齐当前无效";
      return false;
    }
    if (global_alignment_timeout_ > 0.0 &&
        (ros::Time::now() -
         last_local_alignment_valid_receive_).toSec() >
            global_alignment_timeout_) {
      *reason = "local_alignment_valid消息超时";
      return false;
    }
    return true;
  }

  void localAlignmentValidCallback(
      const std_msgs::Bool::ConstPtr& message) {
    local_alignment_valid_ = message->data;
    have_local_alignment_valid_ = true;
    last_local_alignment_valid_receive_ = ros::Time::now();
  }

  bool lookupReferenceTransform(
      const std::string& target_frame,
      const std::string& source_frame,
      const double timeout,
      tf2::Transform* target_source,
      std::string* reason) {
    const std::string target = canonicalFrame(target_frame);
    const std::string source = canonicalFrame(source_frame);
    if (target.empty() || source.empty()) {
      *reason = "控制坐标系或参考坐标系为空";
      return false;
    }
    if (!referenceFrameAllowed(source, target)) {
      *reason = "参考坐标系" + source +
                "不在允许的惯性坐标系列表中";
      return false;
    }
    if (!globalAlignmentReady(source, target, reason)) {
      return false;
    }
    if (target == source) {
      *target_source = tf2::Transform::getIdentity();
      return true;
    }

    geometry_msgs::TransformStamped transform;
    try {
      transform = tf_buffer_.lookupTransform(
          target, source, ros::Time(0),
          ros::Duration(std::max(0.0, timeout)));
    } catch (const tf2::TransformException& exception) {
      *reason = "无法把" + source + "转换到" + target +
                ": " + exception.what();
      return false;
    }
    if (maximum_transform_age_ > 0.0 &&
        !transform.header.stamp.isZero()) {
      const double age =
          (ros::Time::now() - transform.header.stamp).toSec();
      if (!std::isfinite(age) || age < 0.0 ||
          age > maximum_transform_age_) {
        *reason = "参考坐标系TF已超时";
        return false;
      }
    }
    tf2::fromMsg(transform.transform, *target_source);
    if (target_source->getRotation().length2() < 1e-9) {
      *reason = "参考坐标系TF四元数无效";
      return false;
    }
    target_source->getRotation().normalize();
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    tf2::Matrix3x3(target_source->getRotation())
        .getRPY(roll, pitch, yaw);
    if (std::abs(roll) > maximum_inertial_frame_tilt_ ||
        std::abs(pitch) > maximum_inertial_frame_tilt_) {
      *reason = "参考坐标系与控制坐标系的重力方向不一致";
      return false;
    }
    return true;
  }

  static geometry_msgs::Vector3 rotateVector(
      const tf2::Transform& target_source,
      const geometry_msgs::Vector3& source) {
    const tf2::Vector3 input(source.x, source.y, source.z);
    const tf2::Vector3 output =
        tf2::quatRotate(target_source.getRotation(), input);
    geometry_msgs::Vector3 result;
    result.x = output.x();
    result.y = output.y();
    result.z = output.z();
    return result;
  }

  static bool axisMaskCompatibleWithRotation(
      const AxisMask& mask,
      const tf2::Transform& target_source) {
    const tf2::Matrix3x3 rotation(
        target_source.getRotation());
    constexpr double kAxisMixingTolerance = 1e-6;
    for (int target_axis = 0; target_axis < 3;
         ++target_axis) {
      for (int source_axis = 0; source_axis < 3;
           ++source_axis) {
        if (mask[target_axis] != mask[source_axis] &&
            std::abs(
                rotation[target_axis][source_axis]) >
                kAxisMixingTolerance) {
          return false;
        }
      }
    }
    return true;
  }

  bool transformReferenceToControl(
      const Reference& source_reference,
      const std::string& control_frame,
      const double timeout,
      Reference* control_reference,
      std::string* reason) {
    tf2::Transform control_source;
    if (!lookupReferenceTransform(
            control_frame,
            source_reference.header.frame_id,
            timeout, &control_source, reason)) {
      return false;
    }
    if (!axisMaskCompatibleWithRotation(
            source_reference.use_position,
            control_source) ||
        !axisMaskCompatibleWithRotation(
            source_reference.use_velocity,
            control_source) ||
        !axisMaskCompatibleWithRotation(
            source_reference.use_acceleration,
            control_source)) {
      *reason =
          "参考TF旋转会混合已启用与已忽略的控制轴；"
          "逐轴掩码应使用控制odom或轴方向一致的惯性frame";
      return false;
    }

    *control_reference = source_reference;
    control_reference->header.frame_id =
        canonicalFrame(control_frame);
    control_reference->header.stamp = ros::Time::now();
    if (anyAxis(source_reference.use_position)) {
      const tf2::Vector3 source_position(
          source_reference.position.x,
          source_reference.position.y,
          source_reference.position.z);
      const tf2::Vector3 control_position =
          control_source * source_position;
      control_reference->position.x = control_position.x();
      control_reference->position.y = control_position.y();
      control_reference->position.z = control_position.z();
    }

    if (source_reference.use_yaw) {
      tf2::Quaternion source_orientation;
      source_orientation.setRPY(
          0.0, 0.0, source_reference.yaw);
      tf2::Quaternion control_orientation =
          control_source.getRotation() *
          source_orientation;
      control_orientation.normalize();
      double roll = 0.0;
      double pitch = 0.0;
      tf2::Matrix3x3(control_orientation).getRPY(
          roll, pitch, control_reference->yaw);
      control_reference->yaw =
          wrapAngle(control_reference->yaw);
    }
    if (anyAxis(source_reference.use_velocity)) {
      control_reference->velocity = rotateVector(
          control_source, source_reference.velocity);
    }
    if (anyAxis(source_reference.use_acceleration)) {
      control_reference->acceleration = rotateVector(
          control_source, source_reference.acceleration);
    }
    if (source_reference.use_jerk) {
      control_reference->jerk = rotateVector(
          control_source, source_reference.jerk);
    }
    if (source_reference.use_yaw_rate) {
      geometry_msgs::Vector3 angular_velocity;
      angular_velocity.z = source_reference.yaw_rate;
      control_reference->yaw_rate =
          rotateVector(control_source, angular_velocity).z;
    }
    return true;
  }

  bool positionTargetToSourceReference(
      const mavros_msgs::PositionTarget& message,
      Reference* source_reference,
      std::string* reason) const {
    if (message.header.frame_id.empty()) {
      *reason = "PositionTarget.header.frame_id不能为空";
      return false;
    }
    if (message.coordinate_frame !=
        mavros_msgs::PositionTarget::FRAME_LOCAL_NED) {
      *reason =
          "当前只支持PositionTarget.FRAME_LOCAL_NED；"
          "字段按ROS ENU惯性坐标语义解释";
      return false;
    }

    constexpr std::array<uint16_t, 3> position_bits{{
        mavros_msgs::PositionTarget::IGNORE_PX,
        mavros_msgs::PositionTarget::IGNORE_PY,
        mavros_msgs::PositionTarget::IGNORE_PZ}};
    constexpr std::array<uint16_t, 3> velocity_bits{{
        mavros_msgs::PositionTarget::IGNORE_VX,
        mavros_msgs::PositionTarget::IGNORE_VY,
        mavros_msgs::PositionTarget::IGNORE_VZ}};
    constexpr std::array<uint16_t, 3> acceleration_bits{{
        mavros_msgs::PositionTarget::IGNORE_AFX,
        mavros_msgs::PositionTarget::IGNORE_AFY,
        mavros_msgs::PositionTarget::IGNORE_AFZ}};
    constexpr uint16_t position_group =
        position_bits[0] | position_bits[1] |
        position_bits[2];
    constexpr uint16_t velocity_group =
        velocity_bits[0] | velocity_bits[1] |
        velocity_bits[2];
    constexpr uint16_t acceleration_group =
        acceleration_bits[0] | acceleration_bits[1] |
        acceleration_bits[2];
    constexpr uint16_t known_mask =
        position_group | velocity_group |
        acceleration_group |
        mavros_msgs::PositionTarget::FORCE |
        mavros_msgs::PositionTarget::IGNORE_YAW |
        mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

    if ((message.type_mask &
         static_cast<uint16_t>(~known_mask)) != 0U) {
      *reason = "PositionTarget.type_mask包含未知位";
      return false;
    }
    if ((message.type_mask &
         mavros_msgs::PositionTarget::FORCE) != 0U) {
      *reason =
          "当前不支持PositionTarget.FORCE，只接受加速度";
      return false;
    }

    Reference reference;
    reference.header = message.header;
    reference.header.frame_id =
        canonicalFrame(message.header.frame_id);
    for (int axis = 0; axis < 3; ++axis) {
      reference.use_position[axis] =
          (message.type_mask & position_bits[axis]) == 0U;
      reference.use_velocity[axis] =
          (message.type_mask & velocity_bits[axis]) == 0U;
      reference.use_acceleration[axis] =
          (message.type_mask &
           acceleration_bits[axis]) == 0U;
    }
    reference.use_yaw =
        (message.type_mask &
         mavros_msgs::PositionTarget::IGNORE_YAW) == 0U;
    reference.use_yaw_rate =
        (message.type_mask &
         mavros_msgs::PositionTarget::IGNORE_YAW_RATE) == 0U;

    if (!anyAxis(reference.use_position) &&
        !anyAxis(reference.use_velocity) &&
        !anyAxis(reference.use_acceleration)) {
      *reason =
          "PositionTarget至少要启用一个位置、速度或加速度轴";
      return false;
    }
    if (!finiteSelected(
            message.position,
            reference.use_position)) {
      *reason = "启用的位置字段包含非法数值";
      return false;
    }
    if (!finiteSelected(
            message.velocity,
            reference.use_velocity)) {
      *reason = "启用的速度字段包含非法数值";
      return false;
    }
    if (!finiteSelected(
            message.acceleration_or_force,
            reference.use_acceleration)) {
      *reason = "启用的加速度字段包含非法数值";
      return false;
    }
    if (reference.use_yaw &&
        !std::isfinite(message.yaw)) {
      *reason = "启用的yaw字段包含非法数值";
      return false;
    }
    if (reference.use_yaw_rate &&
        !std::isfinite(message.yaw_rate)) {
      *reason = "启用的yaw_rate字段包含非法数值";
      return false;
    }

    reference.position.x =
        reference.use_position[0] ? message.position.x : 0.0;
    reference.position.y =
        reference.use_position[1] ? message.position.y : 0.0;
    reference.position.z =
        reference.use_position[2] ? message.position.z : 0.0;
    reference.velocity.x =
        reference.use_velocity[0] ? message.velocity.x : 0.0;
    reference.velocity.y =
        reference.use_velocity[1] ? message.velocity.y : 0.0;
    reference.velocity.z =
        reference.use_velocity[2] ? message.velocity.z : 0.0;
    reference.acceleration.x =
        reference.use_acceleration[0]
            ? message.acceleration_or_force.x
            : 0.0;
    reference.acceleration.y =
        reference.use_acceleration[1]
            ? message.acceleration_or_force.y
            : 0.0;
    reference.acceleration.z =
        reference.use_acceleration[2]
            ? message.acceleration_or_force.z
            : 0.0;
    if (reference.use_yaw) {
      reference.yaw = message.yaw;
    }
    if (reference.use_yaw_rate) {
      reference.yaw_rate = message.yaw_rate;
    }
    *source_reference = reference;
    return true;
  }

  bool positionTargetSupportedByVehicle(
      const Reference& reference,
      std::string* reason) const {
    if (vehicle_type_ != "fixedwing") {
      return true;
    }
    if (anyAxis(reference.use_acceleration)) {
      *reason =
          "固定翼PositionTarget暂不支持加速度或力控制；"
          "请掩码忽略AFX/AFY/AFZ，使用位置、速度、yaw"
          "或yaw_rate";
      return false;
    }
    return true;
  }

  void stateCallback(
      const xd_uav_controller::ControlState::ConstPtr& message) {
    state_ = *message;
    last_state_receive_ = ros::Time::now();
    have_state_ = true;
    if (vehicle_type_ == "multirotor" &&
        (!state_.state_valid ||
         !finite(state_.acceleration_odom))) {
      acceleration_command_initialized_ = false;
      velocity_integral_acceleration_state_.setZero();
      acceleration_feedback_integral_state_.setZero();
      acceleration_feedback_active_.fill(false);
    }
    if (!have_reference_ &&
        !internal_reference_active_ &&
        !idle_reference_active_ && state_.state_valid) {
      captureIdleReference();
    }
  }

  bool externalReferenceAllowed() const {
    if (landing_active_) {
      ROS_WARN_THROTTLE(
          1.0,
          "[xd_uav_controller] 正在降落，忽略外部控制参考");
      return false;
    }
    return true;
  }

  void activateExternalReference() {
    last_reference_receive_ = ros::Time::now();
    active_reference_transform_failure_since_ = ros::Time();
    have_normalized_reference_ = true;
    have_reference_ = true;
    have_reference_error_ = false;
    internal_reference_active_ = false;
    idle_reference_active_ = false;
    takeoff_active_ = false;
    fixedwing_loiter_active_ = false;
  }

  void resetFixedwingControlState() {
    fixedwing_altitude_integral_climb_rate_ = 0.0;
    filtered_climb_rate_ = 0.0;
    filtered_climb_rate_initialized_ = false;
    last_fixedwing_control_time_ = ros::Time();
    filtered_fixedwing_lookahead_distance_ =
        fixedwing_guidance_lookahead_distance_;
    fixedwing_lookahead_initialized_ = false;
  }

  void rejectExternalReference(const std::string& reason) {
    reference_input_error_ = reason;
    have_reference_error_ = true;
    ROS_ERROR_THROTTLE(
        1.0,
        "[xd_uav_controller] 拒绝新的外部参考，保持当前有效目标: %s",
        reason.c_str());
  }

  void referencePositionTargetCallback(
      const mavros_msgs::PositionTarget::ConstPtr& message) {
    if (!externalReferenceAllowed()) {
      return;
    }
    if (!have_state_ || !state_.state_valid) {
      rejectExternalReference(
          "收到PositionTarget参考时控制状态尚未有效");
      return;
    }

    Reference source;
    std::string reason;
    if (!positionTargetToSourceReference(
            *message, &source, &reason)) {
      rejectExternalReference(reason);
      return;
    }
    if (!positionTargetSupportedByVehicle(source, &reason)) {
      rejectExternalReference(reason);
      return;
    }
    Reference normalized;
    if (!transformReferenceToControl(
            source, state_.header.frame_id,
            reference_transform_timeout_,
            &normalized, &reason)) {
      rejectExternalReference(reason);
      return;
    }

    reference_source_ = source;
    reference_ = normalized;
    trajectory_active_ = false;
    point_reference_latched_ =
        anyAxis(source.use_position) &&
        !anyAxis(source.use_velocity) &&
        !anyAxis(source.use_acceleration) &&
        !source.use_yaw_rate;
    activateExternalReference();
  }

  double altitudeToKeepForSimpleGoal() const {
    if (internal_reference_active_ &&
        have_takeoff_origin_) {
      return takeoff_target_z_;
    }
    if (have_reference_ &&
        have_normalized_reference_ &&
        reference_.use_position[2]) {
      return reference_.position.z;
    }
    if (idle_reference_active_) {
      return idle_position_.z();
    }
    return state_.position_odom.z;
  }

  void simpleGoalCallback(
      const geometry_msgs::PoseStamped::ConstPtr& message) {
    if (!externalReferenceAllowed()) {
      return;
    }
    if (!have_state_ || !state_.state_valid) {
      rejectExternalReference(
          "收到simple goal时控制状态尚未有效");
      return;
    }
    if (message->header.frame_id.empty() ||
        !finite(message->pose.position)) {
      rejectExternalReference(
          "simple goal位置无效或frame_id为空");
      return;
    }
    Eigen::Matrix3d rotation;
    if (!quaternionToMatrix(
            message->pose.orientation, &rotation)) {
      rejectExternalReference(
          "simple goal姿态四元数无效");
      return;
    }

    tf2::Transform control_source;
    std::string reason;
    if (!lookupReferenceTransform(
            state_.header.frame_id,
            message->header.frame_id,
            reference_transform_timeout_,
            &control_source, &reason)) {
      rejectExternalReference(reason);
      return;
    }

    geometry_msgs::Point goal_position =
        message->pose.position;
    if (!simple_goal_use_message_z_) {
      const double altitude_to_keep =
          altitudeToKeepForSimpleGoal();
      const tf2::Vector3 control_position(
          state_.position_odom.x,
          state_.position_odom.y,
          altitude_to_keep);
      const tf2::Vector3 source_position =
          control_source.inverse() * control_position;
      goal_position.z = source_position.z();
    }

    mavros_msgs::PositionTarget reference;
    reference.header = message->header;
    reference.header.frame_id =
        canonicalFrame(message->header.frame_id);
    reference.header.stamp = ros::Time::now();
    reference.header.seq = ++simple_goal_sequence_counter_;
    reference.coordinate_frame =
        mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    reference.type_mask =
        mavros_msgs::PositionTarget::IGNORE_VX |
        mavros_msgs::PositionTarget::IGNORE_VY |
        mavros_msgs::PositionTarget::IGNORE_VZ |
        mavros_msgs::PositionTarget::IGNORE_AFX |
        mavros_msgs::PositionTarget::IGNORE_AFY |
        mavros_msgs::PositionTarget::IGNORE_AFZ |
        mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
    reference.position = goal_position;
    reference.yaw = std::atan2(rotation(1, 0), rotation(0, 0));

    Reference source;
    Reference normalized;
    if (!positionTargetToSourceReference(
            reference, &source, &reason) ||
        !positionTargetSupportedByVehicle(source, &reason) ||
        !transformReferenceToControl(
            source, state_.header.frame_id, 0.0,
            &normalized, &reason)) {
      rejectExternalReference(reason);
      return;
    }
    reference_position_target_publisher_.publish(reference);
    ROS_INFO(
        "[xd_uav_controller] simple goal已从%s适配到%s: "
        "x=%.2f y=%.2f z=%.2f yaw=%.2f",
        reference.header.frame_id.c_str(),
        normalized.header.frame_id.c_str(),
        normalized.position.x,
        normalized.position.y,
        normalized.position.z,
        normalized.yaw);
  }

  bool validateTrajectory(
      const trajectory_msgs::MultiDOFJointTrajectory& trajectory,
      std::string* reason) const {
    if (trajectory.header.frame_id.empty()) {
      *reason =
          "MultiDOFJointTrajectory.header.frame_id不能为空";
      return false;
    }
    if (trajectory.points.empty()) {
      *reason = "MultiDOFJointTrajectory.points不能为空";
      return false;
    }
    if (trajectory.joint_names.size() != 1) {
      *reason =
          "当前控制器要求joint_names中只包含一个机体";
      return false;
    }

    const bool has_velocity =
        !trajectory.points.front().velocities.empty();
    const bool has_acceleration =
        !trajectory.points.front().accelerations.empty();
    double previous_time = -1.0;
    for (std::size_t index = 0;
         index < trajectory.points.size(); ++index) {
      const auto& point = trajectory.points[index];
      if (point.transforms.size() != 1) {
        *reason = "每个轨迹点必须且只能包含一个transform";
        return false;
      }
      if ((!point.velocities.empty() &&
           point.velocities.size() != 1) ||
          (!point.accelerations.empty() &&
           point.accelerations.size() != 1)) {
        *reason =
            "每个轨迹点的velocity/acceleration必须为空或只有一个";
        return false;
      }
      if ((!point.velocities.empty()) != has_velocity ||
          (!point.accelerations.empty()) !=
              has_acceleration) {
        *reason =
            "所有轨迹点必须一致地提供或省略速度、加速度";
        return false;
      }
      Eigen::Matrix3d rotation;
      if (!finite(point.transforms.front().translation) ||
          !quaternionToMatrix(
              point.transforms.front().rotation, &rotation)) {
        *reason = "轨迹点位置或姿态无效";
        return false;
      }
      if (has_velocity &&
          (!finite(point.velocities.front().linear) ||
           !finite(point.velocities.front().angular))) {
        *reason = "轨迹点速度包含非法数值";
        return false;
      }
      if (has_acceleration &&
          (!finite(point.accelerations.front().linear) ||
           !finite(point.accelerations.front().angular))) {
        *reason = "轨迹点加速度包含非法数值";
        return false;
      }
      const double point_time = point.time_from_start.toSec();
      if (!std::isfinite(point_time) || point_time < 0.0 ||
          (index > 0 && point_time <= previous_time)) {
        *reason =
            "轨迹点time_from_start必须非负且严格递增";
        return false;
      }
      previous_time = point_time;
    }
    return true;
  }

  Reference trajectoryPointReference(
      const trajectory_msgs::MultiDOFJointTrajectoryPoint&
          point,
      const std::string& frame_id,
      const ros::Time& stamp) const {
    Reference reference;
    reference.header.stamp = stamp;
    reference.header.frame_id = canonicalFrame(frame_id);
    reference.trajectory_reference = true;
    const auto& transform = point.transforms.front();
    reference.position.x = transform.translation.x;
    reference.position.y = transform.translation.y;
    reference.position.z = transform.translation.z;
    reference.use_position = {{true, true, true}};
    reference.use_yaw = true;
    Eigen::Matrix3d rotation;
    quaternionToMatrix(transform.rotation, &rotation);
    reference.yaw =
        std::atan2(rotation(1, 0), rotation(0, 0));
    if (!point.velocities.empty()) {
      reference.velocity = point.velocities.front().linear;
      reference.yaw_rate =
          point.velocities.front().angular.z;
      reference.use_velocity = {{true, true, true}};
      reference.use_yaw_rate = true;
    }
    if (!point.accelerations.empty()) {
      reference.acceleration =
          point.accelerations.front().linear;
      reference.use_acceleration = {{true, true, true}};
    }
    return reference;
  }

  bool sampleTrajectory(const ros::Time& now,
                        Reference* reference,
                        std::string* reason) const {
    if (!trajectory_active_ ||
        reference_trajectory_.points.empty()) {
      *reason = "当前没有有效轨迹";
      return false;
    }
    const double elapsed =
        (now - reference_trajectory_start_).toSec();
    const auto& points = reference_trajectory_.points;
    if (elapsed <= points.front().time_from_start.toSec()) {
      *reference =
          trajectoryPointReference(
              points.front(),
              reference_trajectory_.header.frame_id, now);
      return true;
    }
    if (elapsed >= points.back().time_from_start.toSec()) {
      *reference =
          trajectoryPointReference(
              points.back(),
              reference_trajectory_.header.frame_id, now);
      return true;
    }

    std::size_t upper = 1;
    while (upper < points.size() &&
           points[upper].time_from_start.toSec() < elapsed) {
      ++upper;
    }
    const std::size_t lower = upper - 1;
    const double lower_time =
        points[lower].time_from_start.toSec();
    const double upper_time =
        points[upper].time_from_start.toSec();
    const double alpha =
        clamp((elapsed - lower_time) /
                  (upper_time - lower_time),
              0.0, 1.0);
    const Reference first =
        trajectoryPointReference(
            points[lower],
            reference_trajectory_.header.frame_id, now);
    const Reference second =
        trajectoryPointReference(
            points[upper],
            reference_trajectory_.header.frame_id, now);
    *reference = first;
    reference->header.stamp = now;
    reference->position.x =
        first.position.x +
        alpha * (second.position.x - first.position.x);
    reference->position.y =
        first.position.y +
        alpha * (second.position.y - first.position.y);
    reference->position.z =
        first.position.z +
        alpha * (second.position.z - first.position.z);
    reference->yaw = wrapAngle(
        first.yaw +
        alpha * wrapAngle(second.yaw - first.yaw));
    if (anyAxis(first.use_velocity) &&
        anyAxis(second.use_velocity)) {
      reference->velocity.x =
          first.velocity.x +
          alpha * (second.velocity.x - first.velocity.x);
      reference->velocity.y =
          first.velocity.y +
          alpha * (second.velocity.y - first.velocity.y);
      reference->velocity.z =
          first.velocity.z +
          alpha * (second.velocity.z - first.velocity.z);
      reference->yaw_rate =
          first.yaw_rate +
          alpha * (second.yaw_rate - first.yaw_rate);
    }
    if (anyAxis(first.use_acceleration) &&
        anyAxis(second.use_acceleration)) {
      reference->acceleration.x =
          first.acceleration.x +
          alpha *
              (second.acceleration.x -
               first.acceleration.x);
      reference->acceleration.y =
          first.acceleration.y +
          alpha *
              (second.acceleration.y -
               first.acceleration.y);
      reference->acceleration.z =
          first.acceleration.z +
          alpha *
              (second.acceleration.z -
               first.acceleration.z);
    }
    return true;
  }

  void publishTrajectoryPath(
      const trajectory_msgs::MultiDOFJointTrajectory&
          trajectory) {
    nav_msgs::Path path;
    path.header.stamp = ros::Time::now();
    path.header.frame_id =
        canonicalFrame(trajectory.header.frame_id);
    path.poses.reserve(trajectory.points.size());
    for (const auto& point : trajectory.points) {
      geometry_msgs::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x =
          point.transforms.front().translation.x;
      pose.pose.position.y =
          point.transforms.front().translation.y;
      pose.pose.position.z =
          point.transforms.front().translation.z;
      pose.pose.orientation =
          point.transforms.front().rotation;
      path.poses.push_back(pose);
    }
    reference_trajectory_path_publisher_.publish(path);
  }

  void referenceTrajectoryCallback(
      const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr&
          message) {
    if (!externalReferenceAllowed()) {
      return;
    }
    if (!have_state_ || !state_.state_valid) {
      rejectExternalReference(
          "收到轨迹参考时控制状态尚未有效");
      return;
    }
    std::string reason;
    if (!validateTrajectory(*message, &reason)) {
      rejectExternalReference(reason);
      return;
    }
    const Reference first =
        trajectoryPointReference(
            message->points.front(),
            message->header.frame_id,
            ros::Time::now());
    Reference normalized;
    if (!transformReferenceToControl(
            first, state_.header.frame_id,
            reference_transform_timeout_,
            &normalized, &reason)) {
      rejectExternalReference(reason);
      return;
    }
    reference_trajectory_ = *message;
    reference_ = normalized;
    reference_trajectory_start_ =
        message->header.stamp.isZero()
            ? ros::Time::now()
            : message->header.stamp;
    if (vehicle_type_ == "fixedwing") {
      resetFixedwingControlState();
    }
    publishTrajectoryPath(*message);
    trajectory_active_ = true;
    point_reference_latched_ = false;
    activateExternalReference();
  }

  void captureIdleReference() {
    idle_position_ = Eigen::Vector3d(
        state_.position_odom.x, state_.position_odom.y,
        state_.position_odom.z);
    idle_course_ = state_.course;
    Eigen::Matrix3d rotation;
    if (!std::isfinite(idle_course_) &&
        quaternionToMatrix(state_.orientation_odom_body, &rotation)) {
      idle_course_ =
          std::atan2(rotation(1, 0), rotation(0, 0));
    }
    if (!std::isfinite(idle_course_)) {
      idle_course_ = 0.0;
    }
    idle_airspeed_ =
        state_.airspeed_valid && state_.airspeed > 0.5
            ? state_.airspeed
            : cruise_airspeed_;
    idle_reference_active_ = true;
  }

  Reference makeIdleReference() const {
    Reference reference;
    reference.header.stamp = ros::Time::now();
    reference.header.frame_id = state_.header.frame_id;
    reference.position.x = idle_position_.x();
    reference.position.y = idle_position_.y();
    reference.position.z = idle_position_.z();
    reference.yaw = idle_course_;
    reference.use_position = {{true, true, true}};
    reference.use_velocity = {{true, true, true}};
    reference.use_yaw = true;
    if (vehicle_type_ == "multirotor") {
      reference.use_acceleration = {{true, true, true}};
    } else {
      reference.velocity.x =
          idle_airspeed_ * std::cos(idle_course_);
      reference.velocity.y =
          idle_airspeed_ * std::sin(idle_course_);
    }
    return reference;
  }

  bool internalCommandCallback(
      xd_uav_controller::InternalCommand::Request& request,
      xd_uav_controller::InternalCommand::Response& response) {
    switch (request.command) {
      case xd_uav_controller::InternalCommand::Request::TAKEOFF: {
        xd_uav_controller::Takeoff::Request takeoff_request;
        xd_uav_controller::Takeoff::Response takeoff_response;
        takeoff_request.altitude = request.altitude;
        takeoffCallback(takeoff_request, takeoff_response);
        response.success = takeoff_response.success;
        response.message = takeoff_response.message;
        return true;
      }
      case xd_uav_controller::InternalCommand::Request::LAND: {
        std_srvs::Trigger::Request land_request;
        std_srvs::Trigger::Response land_response;
        landCallback(land_request, land_response);
        response.success = land_response.success;
        response.message = land_response.message;
        return true;
      }
      case xd_uav_controller::InternalCommand::Request::LAND_HOME: {
        std_srvs::Trigger::Request land_request;
        std_srvs::Trigger::Response land_response;
        landHomeCallback(land_request, land_response);
        response.success = land_response.success;
        response.message = land_response.message;
        return true;
      }
      case xd_uav_controller::InternalCommand::Request::RESET: {
        std_srvs::Trigger::Request reset_request;
        std_srvs::Trigger::Response reset_response;
        resetCallback(reset_request, reset_response);
        response.success = reset_response.success;
        response.message = reset_response.message;
        return true;
      }
      default:
        response.success = false;
        response.message = "未知的控制器内部命令";
        return true;
    }
  }

  bool takeoffCallback(
      xd_uav_controller::Takeoff::Request& request,
      xd_uav_controller::Takeoff::Response& response) {
    if (!have_state_ || !state_.state_valid ||
        !state_.odometry_fresh || !state_.imu_fresh) {
      response.success = false;
      response.message = "当前控制状态无效，不能开始起飞";
      return true;
    }
    if (vehicle_type_ == "fixedwing" && !state_.airspeed_valid) {
      response.success = false;
      response.message = "固定翼空速无效，不能开始OFFBOARD起飞";
      return true;
    }
    if (landing_active_) {
      response.success = false;
      response.message = "正在执行降落，不能再次起飞";
      return true;
    }
    if (internal_reference_active_) {
      response.success = true;
      response.message =
          takeoff_active_
              ? "起飞已经在执行，保持原有目标"
              : "起飞已经完成，保持原有目标";
      return true;
    }
    const double altitude =
        request.altitude > 0.0 ? request.altitude
                               : takeoff_default_altitude_;
    takeoff_course_ = state_.course;
    if (!std::isfinite(takeoff_course_)) {
      Eigen::Matrix3d rotation;
      if (!quaternionToMatrix(state_.orientation_odom_body,
                              &rotation)) {
        response.success = false;
        response.message = "当前姿态无效";
        return true;
      }
      takeoff_course_ =
          std::atan2(rotation(1, 0), rotation(0, 0));
    }

    takeoff_origin_ = Eigen::Vector3d(
        state_.position_odom.x, state_.position_odom.y,
        state_.position_odom.z);
    have_takeoff_origin_ = true;
    takeoff_target_z_ = state_.position_odom.z + altitude;

    std::string home_message;
    if (home_mode_ == "takeoff") {
      recordTakeoffHome(takeoff_course_, &home_message);
    } else if (!initializeFixedHome(&home_message)) {
      response.success = false;
      response.message =
          "固定home配置无效: " + home_message;
      have_takeoff_origin_ = false;
      return true;
    }
    ROS_INFO("[xd_uav_controller] %s",
             home_message.c_str());

    internal_reference_active_ = true;
    idle_reference_active_ = false;
    if (vehicle_type_ == "multirotor") {
      // Start a takeoff from hover. The acceleration command is then
      // slew-limited by the configured jerk limits.
      acceleration_command_state_.setZero();
      acceleration_command_initialized_ = true;
    }
    takeoff_active_ = true;
    response.success = true;
    response.message = "起飞参考已建立";
    return true;
  }

  bool initializeFixedHome(std::string* reason) {
    if (have_home_reference_) {
      *reason = "保持已配置的固定home";
      return true;
    }
    if (!have_state_ || state_.header.frame_id.empty()) {
      *reason = "控制状态frame_id为空";
      return false;
    }
    const std::string home_frame = resolveConfiguredFrame(
        home_frame_config_, state_.header.frame_id);
    if (home_frame.empty()) {
      *reason = "home/frame不能为空";
      return false;
    }

    home_reference_source_ = Reference();
    home_reference_source_.header.stamp = ros::Time::now();
    home_reference_source_.header.frame_id = home_frame;
    home_reference_source_.position.x =
        fixed_home_position_.x();
    home_reference_source_.position.y =
        fixed_home_position_.y();
    home_reference_source_.position.z =
        fixed_home_position_.z();
    home_reference_source_.yaw =
        wrapAngle(fixed_home_yaw_);
    home_reference_source_.use_position =
        {{true, true, true}};
    home_reference_source_.use_yaw = true;
    have_home_reference_ = true;
    *reason = "固定home已载入: frame=" + home_frame;
    return true;
  }

  void recordTakeoffHome(
      const double control_course,
      std::string* message) {
    const std::string control_frame =
        canonicalFrame(state_.header.frame_id);
    const std::string requested_home_frame =
        resolveConfiguredFrame(
            home_frame_config_, control_frame);

    tf2::Transform control_home;
    std::string transform_reason;
    if (!requested_home_frame.empty() &&
        lookupReferenceTransform(
            control_frame, requested_home_frame,
            reference_transform_timeout_, &control_home,
            &transform_reason)) {
      const tf2::Vector3 control_position(
          state_.position_odom.x,
          state_.position_odom.y,
          state_.position_odom.z);
      const tf2::Vector3 home_position =
          control_home.inverse() * control_position;
      tf2::Quaternion control_orientation;
      control_orientation.setRPY(
          0.0, 0.0, control_course);
      tf2::Quaternion home_orientation =
          control_home.getRotation().inverse() *
          control_orientation;
      home_orientation.normalize();
      double home_roll = 0.0;
      double home_pitch = 0.0;
      double home_yaw = 0.0;
      tf2::Matrix3x3(home_orientation).getRPY(
          home_roll, home_pitch, home_yaw);

      home_reference_source_ = Reference();
      home_reference_source_.header.stamp =
          ros::Time::now();
      home_reference_source_.header.frame_id =
          requested_home_frame;
      home_reference_source_.position.x =
          home_position.x();
      home_reference_source_.position.y =
          home_position.y();
      home_reference_source_.position.z =
          home_position.z();
      home_reference_source_.yaw =
          wrapAngle(home_yaw);
      home_reference_source_.use_position =
          {{true, true, true}};
      home_reference_source_.use_yaw = true;
      have_home_reference_ = true;
      *message =
          "已在" + requested_home_frame +
          "中记录本次起飞home";
      return;
    }

    // A takeoff must not be blocked solely because global alignment is
    // temporarily unavailable. In that case the current continuous odom
    // is the only coordinate system in which the home can be established.
    home_reference_source_ = Reference();
    home_reference_source_.header.stamp =
        ros::Time::now();
    home_reference_source_.header.frame_id =
        control_frame;
    home_reference_source_.position.x =
        state_.position_odom.x;
    home_reference_source_.position.y =
        state_.position_odom.y;
    home_reference_source_.position.z =
        state_.position_odom.z;
    home_reference_source_.yaw =
        wrapAngle(control_course);
    home_reference_source_.use_position =
        {{true, true, true}};
    home_reference_source_.use_yaw = true;
    have_home_reference_ = true;
    *message =
        "无法在" + requested_home_frame +
        "中记录home，已退化为当前" + control_frame +
        "坐标: " + transform_reason;
    ROS_WARN("[xd_uav_controller] %s",
             message->c_str());
  }

  bool homeInControlFrame(
      const std::string& control_frame,
      const double timeout,
      Eigen::Vector3d* position,
      double* yaw,
      std::string* reason) {
    if (!have_home_reference_) {
      *reason = "home尚未建立";
      return false;
    }
    Reference normalized_home;
    if (!transformReferenceToControl(
            home_reference_source_, control_frame,
            timeout, &normalized_home, reason)) {
      return false;
    }
    *position = Eigen::Vector3d(
        normalized_home.position.x,
        normalized_home.position.y,
        normalized_home.position.z);
    *yaw = normalized_home.yaw;
    if (!position->allFinite() ||
        !std::isfinite(*yaw)) {
      *reason = "转换后的home包含非法数值";
      return false;
    }
    return true;
  }

  bool landCallback(
      std_srvs::Trigger::Request&,
      std_srvs::Trigger::Response& response) {
    return startLanding(false, response);
  }

  bool landHomeCallback(
      std_srvs::Trigger::Request&,
      std_srvs::Trigger::Response& response) {
    return startLanding(true, response);
  }

  bool startLanding(
      const bool return_home,
      std_srvs::Trigger::Response& response) {
    if (!have_state_ || !state_.state_valid ||
        !state_.odometry_fresh || !state_.imu_fresh) {
      response.success = false;
      response.message = "当前控制状态无效，不能开始降落";
      return true;
    }
    if (!have_takeoff_origin_) {
      response.success = false;
      response.message =
          "没有本次起飞的地面高度，拒绝自动降落";
      return true;
    }
    if (landing_active_) {
      response.success = true;
      response.message =
          landing_return_home_
              ? "返航降落已经在执行"
              : "原地降落已经在执行";
      return true;
    }

    const Eigen::Vector3d current_position(
        state_.position_odom.x, state_.position_odom.y,
        state_.position_odom.z);
    double home_course = state_.course;
    if (return_home) {
      std::string home_reason;
      if (home_mode_ == "fixed_local" &&
          !initializeFixedHome(&home_reason)) {
        response.success = false;
        response.message =
            "固定home配置无效: " + home_reason;
        return true;
      }
      if (!homeInControlFrame(
              state_.header.frame_id,
              reference_transform_timeout_,
              &landing_origin_, &home_course,
              &home_reason)) {
        response.success = false;
        response.message =
            "无法获得当前控制坐标系中的home: " +
            home_reason;
        return true;
      }
      landing_ground_z_ = landing_origin_.z();
    } else {
      landing_origin_ = current_position;
      landing_origin_.z() = takeoff_origin_.z();
      landing_ground_z_ = takeoff_origin_.z();
    }
    landing_setpoint_ = current_position;
    landing_target_z_ =
        landing_ground_z_ -
        std::abs(landing_touchdown_offset_);
    landing_course_ = state_.course;
    if (!std::isfinite(landing_course_)) {
      Eigen::Matrix3d rotation;
      if (!quaternionToMatrix(state_.orientation_odom_body,
                              &rotation)) {
        response.success = false;
        response.message = "当前姿态无效";
        return true;
      }
      landing_course_ =
          std::atan2(rotation(1, 0), rotation(0, 0));
    }
    if (return_home && use_home_yaw_) {
      landing_course_ = home_course;
    }

    if (vehicle_type_ == "fixedwing") {
      const Eigen::Vector2d landing_direction(
          std::cos(landing_course_),
          std::sin(landing_course_));
      if (!return_home) {
        const double current_height =
            std::max(0.0, current_position.z() -
                              landing_ground_z_);
        const double slope_distance =
            current_height /
            std::tan(fixedwing_landing_glide_slope_angle_);
        const double touchdown_ahead = std::max(
            fixedwing_landing_local_touchdown_ahead_,
            slope_distance);
        landing_origin_.head<2>() =
            current_position.head<2>() +
            touchdown_ahead * landing_direction;
        landing_origin_.z() = landing_ground_z_;
      }
      fixedwing_landing_approach_point_ =
          landing_origin_ -
          fixedwing_landing_approach_distance_ *
              Eigen::Vector3d(
                  landing_direction.x(),
                  landing_direction.y(), 0.0);
      fixedwing_landing_approach_point_.z() =
          landing_ground_z_ +
          fixedwing_landing_approach_height_;
      fixedwing_landing_phase_ =
          FixedwingLandingPhase::kApproach;
      fixedwing_landing_throttle_scale_ = 1.0;
      ROS_INFO(
          "[xd_uav_controller] 固定翼%s降落航线: "
          "approach=(%.1f, %.1f, %.1f) "
          "touchdown=(%.1f, %.1f, %.1f) course=%.2f",
          return_home ? "返航" : "当前航向",
          fixedwing_landing_approach_point_.x(),
          fixedwing_landing_approach_point_.y(),
          fixedwing_landing_approach_point_.z(),
          landing_origin_.x(), landing_origin_.y(),
          landing_origin_.z(), landing_course_);
    }

    landing_active_ = true;
    landing_touchdown_ = false;
    landing_return_home_ = return_home;
    if (vehicle_type_ == "multirotor") {
      landing_phase_ =
          return_home ? LandingPhase::kApproach
                      : LandingPhase::kDescent;
    }
    last_landing_update_ = ros::Time::now();
    takeoff_active_ = false;
    fixedwing_loiter_active_ = false;
    internal_reference_active_ = true;
    idle_reference_active_ = false;
    response.success = true;
    if (vehicle_type_ == "fixedwing") {
      response.message =
          return_home
              ? "固定翼返航进近与降落参考已建立"
              : "固定翼当前航向进近与降落参考已建立";
    } else {
      response.message =
          return_home
              ? "返航降落参考已建立"
              : "原地降落参考已建立";
    }
    return true;
  }

  bool resetCallback(
      std_srvs::Trigger::Request&,
      std_srvs::Trigger::Response& response) {
    takeoff_active_ = false;
    fixedwing_loiter_active_ = false;
    landing_active_ = false;
    landing_touchdown_ = false;
    landing_return_home_ = false;
    landing_phase_ = LandingPhase::kNone;
    fixedwing_landing_phase_ =
        FixedwingLandingPhase::kNone;
    internal_reference_active_ = false;
    idle_reference_active_ = false;
    have_reference_ = false;
    have_reference_error_ = false;
    have_normalized_reference_ = false;
    trajectory_active_ = false;
    point_reference_latched_ = false;
    have_takeoff_origin_ = false;
    active_reference_transform_failure_since_ = ros::Time();
    acceleration_command_initialized_ = false;
    velocity_integral_acceleration_state_.setZero();
    acceleration_feedback_integral_state_.setZero();
    acceleration_feedback_active_.fill(false);
    resetFixedwingControlState();
    if (have_state_ && state_.state_valid) {
      captureIdleReference();
    }
    response.success = true;
    response.message = "控制器内部起降状态已复位";
    return true;
  }

  double currentFixedwingCourse() const {
    if (std::isfinite(state_.course)) {
      return state_.course;
    }
    Eigen::Matrix3d rotation;
    if (quaternionToMatrix(
            state_.orientation_odom_body, &rotation)) {
      return std::atan2(rotation(1, 0), rotation(0, 0));
    }
    return takeoff_course_;
  }

  double fixedwingLandingNominalSinkRate() const {
    return std::min(
        fixedwing_landing_approach_airspeed_ *
            std::tan(fixedwing_landing_glide_slope_angle_),
        fixedwing_landing_touchdown_vertical_speed_);
  }

  double fixedwingLandingContactHeight() const {
    return std::min(
        fixedwing_landing_touchdown_height_tolerance_,
        std::abs(landing_touchdown_offset_));
  }

  double fixedwingLandingTransitionHeight() const {
    return std::max(
        fixedwing_landing_touchdown_height_tolerance_,
        fixedwingLandingNominalSinkRate() *
            std::max(0.1, altitude_time_constant_));
  }

  double fixedwingLandingFlareProgress(
      const double height_above_touchdown) const {
    const double contact_height =
        fixedwingLandingContactHeight();
    const double transition_height =
        fixedwingLandingTransitionHeight();
    return clamp(
        (transition_height - height_above_touchdown) /
            std::max(0.1, transition_height - contact_height),
        0.0, 1.0);
  }

  void startFixedwingLoiter(
      const double altitude,
      const std::string& reason) {
    const double course = currentFixedwingCourse();
    const double direction =
        static_cast<double>(fixedwing_loiter_direction_);

    // Put the circle centre to the left for a CCW orbit and to the
    // right for a CW orbit. The current position/course is therefore
    // tangent to the new circle and entering loiter does not require
    // an immediate heading discontinuity.
    fixedwing_loiter_center_.x() =
        state_.position_odom.x -
        direction * fixedwing_loiter_radius_ *
            std::sin(course);
    fixedwing_loiter_center_.y() =
        state_.position_odom.y +
        direction * fixedwing_loiter_radius_ *
            std::cos(course);
    fixedwing_loiter_center_.z() = altitude;
    fixedwing_loiter_active_ = true;
    takeoff_active_ = false;
    internal_reference_active_ = true;
    idle_reference_active_ = false;
    have_reference_ = false;
    have_reference_error_ = false;
    have_normalized_reference_ = false;
    trajectory_active_ = false;
    point_reference_latched_ = false;
    active_reference_transform_failure_since_ = ros::Time();
    resetFixedwingControlState();
    ROS_INFO(
        "[xd_uav_controller] 固定翼进入等待盘旋(%s): "
        "center=(%.1f, %.1f) altitude=%.1f radius=%.1f "
        "direction=%s",
        reason.c_str(),
        fixedwing_loiter_center_.x(),
        fixedwing_loiter_center_.y(),
        fixedwing_loiter_center_.z(),
        fixedwing_loiter_radius_,
        fixedwing_loiter_direction_ > 0 ? "CCW" : "CW");
  }

  Reference makeFixedwingLoiterReference() const {
    Reference reference;
    reference.header.stamp = ros::Time::now();
    reference.header.frame_id = state_.header.frame_id;
    reference.use_position = {{true, true, true}};
    reference.use_velocity = {{true, true, true}};
    reference.use_yaw = true;
    reference.use_yaw_rate = true;
    reference.position.x = fixedwing_loiter_center_.x();
    reference.position.y = fixedwing_loiter_center_.y();
    reference.position.z = fixedwing_loiter_center_.z();

    const double dx =
        state_.position_odom.x -
        fixedwing_loiter_center_.x();
    const double dy =
        state_.position_odom.y -
        fixedwing_loiter_center_.y();
    const double distance =
        std::max(1.0, std::hypot(dx, dy));
    const double radial_error =
        (distance - fixedwing_loiter_radius_) /
        fixedwing_loiter_radius_;
    const double correction = clamp(
        std::atan(
            fixedwing_loiter_radial_gain_ * radial_error),
        -fixedwing_loiter_max_course_correction_,
        fixedwing_loiter_max_course_correction_);
    const double direction =
        static_cast<double>(fixedwing_loiter_direction_);
    const double desired_course = wrapAngle(
        std::atan2(dy, dx) +
        direction * (0.5 * kPi + correction));
    reference.velocity.x =
        fixedwing_loiter_airspeed_ *
        std::cos(desired_course);
    reference.velocity.y =
        fixedwing_loiter_airspeed_ *
        std::sin(desired_course);
    reference.velocity.z = 0.0;
    reference.yaw = desired_course;
    reference.yaw_rate =
        direction * fixedwing_loiter_airspeed_ /
        fixedwing_loiter_radius_;
    return reference;
  }

  Reference makeTakeoffReference() {
    if (vehicle_type_ == "fixedwing" &&
        fixedwing_loiter_active_) {
      return makeFixedwingLoiterReference();
    }

    Reference reference;
    reference.header.stamp = ros::Time::now();
    reference.header.frame_id = state_.header.frame_id;
    reference.use_position = {{true, true, true}};
    reference.use_velocity = {{true, true, true}};
    reference.use_acceleration = {{true, true, true}};
    reference.use_yaw = true;
    reference.position.x = takeoff_origin_.x();
    reference.position.y = takeoff_origin_.y();
    reference.position.z = takeoff_target_z_;
    reference.yaw = takeoff_course_;

    if (vehicle_type_ == "multirotor") {
      const double error =
          takeoff_target_z_ - state_.position_odom.z;
      const double desired_velocity = clamp(
          error, -takeoff_max_velocity_, takeoff_max_velocity_);
      reference.velocity.z = desired_velocity;
      reference.acceleration.z = clamp(
          (desired_velocity - state_.velocity_odom.z) / 0.5,
          -takeoff_max_acceleration_, takeoff_max_acceleration_);
      if (std::abs(error) < takeoff_position_tolerance_ &&
          std::abs(state_.velocity_odom.z) <
              takeoff_velocity_tolerance_) {
        takeoff_active_ = false;
      }
    } else {
      reference.velocity.x =
          fixedwing_climb_airspeed_ * std::cos(takeoff_course_);
      reference.velocity.y =
          fixedwing_climb_airspeed_ * std::sin(takeoff_course_);
      reference.velocity.z = 0.0;
      if (takeoff_target_z_ - state_.position_odom.z <
          fixedwing_altitude_tolerance_) {
        startFixedwingLoiter(
            takeoff_target_z_, "达到起飞目标高度");
        return makeFixedwingLoiterReference();
      }
    }
    return reference;
  }

  Reference makeFixedwingLandingReference() {
    const ros::Time now = ros::Time::now();

    if (landing_return_home_) {
      Eigen::Vector3d latest_home;
      double latest_home_yaw = landing_course_;
      std::string home_reason;
      if (homeInControlFrame(
              state_.header.frame_id, 0.0,
              &latest_home, &latest_home_yaw,
              &home_reason)) {
        landing_origin_ = latest_home;
        landing_ground_z_ = latest_home.z();
        if (use_home_yaw_) {
          landing_course_ = latest_home_yaw;
        }
      } else {
        ROS_WARN_THROTTLE(
            1.0,
            "[xd_uav_controller] 固定翼land_home暂时无法按最新TF"
            "更新home，保持上一次有效转换: %s",
            home_reason.c_str());
      }
    }

    const Eigen::Vector2d landing_direction(
        std::cos(landing_course_),
        std::sin(landing_course_));
    fixedwing_landing_approach_point_ =
        landing_origin_ -
        fixedwing_landing_approach_distance_ *
            Eigen::Vector3d(
                landing_direction.x(),
                landing_direction.y(), 0.0);
    fixedwing_landing_approach_point_.z() =
        landing_ground_z_ +
        fixedwing_landing_approach_height_;

    const Eigen::Vector2d current_position(
        state_.position_odom.x, state_.position_odom.y);
    const Eigen::Vector2d touchdown_position =
        landing_origin_.head<2>();
    const Eigen::Vector2d approach_position =
        fixedwing_landing_approach_point_.head<2>();
    const double height_above_touchdown =
        std::max(0.0, state_.position_odom.z -
                          landing_ground_z_);
    const double distance_to_approach =
        (approach_position - current_position).norm();
    const double estimated_turn_radius =
        fixedwing_landing_approach_airspeed_ *
        fixedwing_landing_approach_airspeed_ /
        std::max(
            0.1,
            gravity_ *
                std::tan(fixedwing_landing_max_roll_));
    const double approach_capture_radius = std::max(
        fixedwing_landing_approach_acceptance_radius_,
        fixedwing_landing_approach_capture_turn_radius_factor_ *
            estimated_turn_radius);
    const double along_track_remaining = std::max(
        0.0,
        (touchdown_position - current_position)
            .dot(landing_direction));
    const Eigen::Vector2d left_normal(
        -landing_direction.y(), landing_direction.x());
    const double cross_track_error =
        left_normal.dot(
            current_position - touchdown_position);
    const double course_error = std::abs(wrapAngle(
        currentFixedwingCourse() - landing_course_));

    if (fixedwing_landing_phase_ ==
            FixedwingLandingPhase::kApproach &&
        distance_to_approach <= approach_capture_radius &&
        std::abs(state_.position_odom.z -
                 fixedwing_landing_approach_point_.z()) <=
            fixedwing_landing_approach_altitude_tolerance_ &&
        course_error <=
            fixedwing_landing_approach_course_tolerance_) {
      fixedwing_landing_phase_ =
          FixedwingLandingPhase::kGlideSlope;
      ROS_INFO(
          "[xd_uav_controller] 固定翼已对准进近航线，"
          "开始下滑(capture_radius=%.1fm)",
          approach_capture_radius);
    }

    if (fixedwing_landing_phase_ ==
            FixedwingLandingPhase::kGlideSlope &&
        height_above_touchdown <=
            fixedwing_landing_touchdown_height_tolerance_) {
      fixedwing_landing_phase_ =
          FixedwingLandingPhase::kRollout;
      ROS_INFO(
          "[xd_uav_controller] 固定翼进入近地滑跑引导，"
          "按离地高度连续收油");
    }

    Reference reference;
    reference.header.stamp = now;
    reference.header.frame_id = state_.header.frame_id;
    reference.use_position = {{true, true, true}};
    reference.use_velocity = {{true, true, true}};
    reference.use_yaw = true;
    reference.yaw = landing_course_;

    if (fixedwing_landing_phase_ ==
        FixedwingLandingPhase::kApproach) {
      const Eigen::Vector2d to_approach =
          approach_position - current_position;
      const double approach_course =
          to_approach.norm() > 1.0
              ? std::atan2(to_approach.y(),
                           to_approach.x())
              : landing_course_;
      reference.position.x =
          fixedwing_landing_approach_point_.x();
      reference.position.y =
          fixedwing_landing_approach_point_.y();
      reference.position.z =
          fixedwing_landing_approach_point_.z();
      reference.velocity.x =
          fixedwing_landing_approach_airspeed_ *
          std::cos(approach_course);
      reference.velocity.y =
          fixedwing_landing_approach_airspeed_ *
          std::sin(approach_course);
      reference.velocity.z = 0.0;
      reference.yaw = approach_course;
    } else if (fixedwing_landing_phase_ ==
               FixedwingLandingPhase::kGlideSlope) {
      const double course_correction = clamp(
          std::atan2(
              cross_track_error,
              fixedwing_landing_line_lookahead_distance_),
          -fixedwing_landing_max_course_correction_,
          fixedwing_landing_max_course_correction_);
      const double path_course = wrapAngle(
          landing_course_ - course_correction);
      const double desired_height = clamp(
          along_track_remaining *
              std::tan(fixedwing_landing_glide_slope_angle_),
          0.0,
          fixedwing_landing_approach_height_);
      const double nominal_sink_rate =
          fixedwingLandingNominalSinkRate();
      const double contact_height =
          fixedwingLandingContactHeight();
      const double transition_height =
          fixedwingLandingTransitionHeight();
      const double touchdown_sink_rate =
          0.5 * nominal_sink_rate;
      double sink_rate_limit = 0.0;
      if (height_above_touchdown > contact_height) {
        if (height_above_touchdown <= transition_height) {
          sink_rate_limit = std::max(
              touchdown_sink_rate,
              nominal_sink_rate *
                  height_above_touchdown /
                  transition_height);
        } else {
          const double path_capture_progress = clamp(
              height_above_touchdown / transition_height - 1.0,
              0.0, 1.0);
          sink_rate_limit =
              nominal_sink_rate +
              path_capture_progress *
                  (fixedwing_landing_touchdown_vertical_speed_ -
                   nominal_sink_rate);
        }
      }
      const double path_climb_rate =
          -nominal_sink_rate +
          (desired_height - height_above_touchdown) /
              std::max(0.1, altitude_time_constant_);
      reference.position.x = landing_origin_.x();
      reference.position.y = landing_origin_.y();
      reference.position.z = state_.position_odom.z;
      reference.use_position[2] = false;
      reference.velocity.x =
          fixedwing_landing_approach_airspeed_ *
          std::cos(path_course);
      reference.velocity.y =
          fixedwing_landing_approach_airspeed_ *
          std::sin(path_course);
      reference.velocity.z = clamp(
          path_climb_rate,
          -sink_rate_limit, sink_rate_limit);
      reference.yaw = path_course;
    } else {
      const double course_correction = clamp(
          std::atan2(
              cross_track_error,
              fixedwing_landing_line_lookahead_distance_),
          -fixedwing_landing_max_course_correction_,
          fixedwing_landing_max_course_correction_);
      const double path_course = wrapAngle(
          landing_course_ - course_correction);
      const Eigen::Vector2d rollout_target =
          touchdown_position +
          fixedwing_landing_rollout_distance_ *
              landing_direction;
      reference.position.x = rollout_target.x();
      reference.position.y = rollout_target.y();
      reference.position.z = state_.position_odom.z;
      reference.use_position[2] = false;
      reference.velocity.x =
          fixedwing_landing_approach_airspeed_ *
          std::cos(path_course);
      reference.velocity.y =
          fixedwing_landing_approach_airspeed_ *
          std::sin(path_course);
      const double contact_height =
          fixedwingLandingContactHeight();
      reference.velocity.z =
          height_above_touchdown <= contact_height
              ? 0.0
              : -0.5 * fixedwingLandingNominalSinkRate();
      reference.yaw = path_course;
    }

    if (!landing_touchdown_) {
      landing_touchdown_ =
          fixedwing_landing_phase_ ==
              FixedwingLandingPhase::kRollout &&
          height_above_touchdown <=
              fixedwing_landing_touchdown_height_tolerance_ &&
          state_.groundspeed <=
              fixedwing_landing_touchdown_groundspeed_ &&
          std::abs(state_.velocity_odom.z) <=
              fixedwing_landing_touchdown_vertical_speed_;
    }
    return reference;
  }

  Reference makeLandingReference() {
    if (vehicle_type_ == "fixedwing") {
      return makeFixedwingLandingReference();
    }

    const ros::Time now = ros::Time::now();
    double dt = 1.0 / std::max(1.0, control_rate_);
    if (!last_landing_update_.isZero()) {
      const double measured_dt =
          (now - last_landing_update_).toSec();
      if (measured_dt > 0.0) {
        dt = clamp(measured_dt, 1e-3, 0.05);
      }
    }
    last_landing_update_ = now;

    if (landing_return_home_) {
      Eigen::Vector3d latest_home;
      double latest_home_yaw = landing_course_;
      std::string home_reason;
      if (homeInControlFrame(
              state_.header.frame_id, 0.0,
              &latest_home, &latest_home_yaw,
              &home_reason)) {
        const double ground_shift =
            latest_home.z() - landing_ground_z_;
        landing_origin_ = latest_home;
        landing_ground_z_ = latest_home.z();
        landing_target_z_ =
            landing_ground_z_ -
            std::abs(landing_touchdown_offset_);
        landing_setpoint_.z() += ground_shift;
        if (use_home_yaw_) {
          landing_course_ = latest_home_yaw;
        }
      } else {
        ROS_WARN_THROTTLE(
            1.0,
            "[xd_uav_controller] land_home暂时无法按最新TF更新home，"
            "保持上一次有效转换: %s",
            home_reason.c_str());
      }
    }

    Reference reference;
    reference.header.stamp = now;
    reference.header.frame_id = state_.header.frame_id;
    reference.use_position = {{true, true, true}};
    reference.use_velocity = {{true, true, true}};
    reference.use_acceleration = {{true, true, true}};
    reference.use_yaw = true;
    if (landing_phase_ == LandingPhase::kApproach) {
      const Eigen::Vector2d target_xy =
          landing_origin_.head<2>();
      const Eigen::Vector2d setpoint_xy =
          landing_setpoint_.head<2>();
      const Eigen::Vector2d remaining =
          target_xy - setpoint_xy;
      const double return_velocity =
          std::abs(landing_return_velocity_);
      const double maximum_step =
          return_velocity * dt;
      if (remaining.norm() > maximum_step &&
          maximum_step > 0.0) {
        const Eigen::Vector2d direction =
            remaining.normalized();
        landing_setpoint_.head<2>() +=
            direction * maximum_step;
        reference.velocity.x =
            direction.x() * return_velocity;
        reference.velocity.y =
            direction.y() * return_velocity;
      } else {
        landing_setpoint_.head<2>() = target_xy;
      }

      const Eigen::Vector2d current_xy(
          state_.position_odom.x, state_.position_odom.y);
      const Eigen::Vector2d current_velocity_xy(
          state_.velocity_odom.x, state_.velocity_odom.y);
      if ((target_xy - current_xy).norm() <=
              landing_position_tolerance_ &&
          current_velocity_xy.norm() <=
              landing_velocity_tolerance_) {
        landing_phase_ = LandingPhase::kDescent;
        landing_setpoint_.head<2>() = target_xy;
      }
    }

    if (landing_phase_ == LandingPhase::kDescent) {
      landing_setpoint_.head<2>() =
          landing_origin_.head<2>();
      const double height =
          std::max(0.0, state_.position_odom.z -
                            landing_ground_z_);
      const double descent_velocity =
          height <= landing_slow_height_
              ? std::abs(landing_final_descent_velocity_)
              : std::abs(landing_descent_velocity_);
      landing_setpoint_.z() = std::max(
          landing_target_z_,
          landing_setpoint_.z() - descent_velocity * dt);
      reference.velocity.z = -descent_velocity;
    }

    const Eigen::Vector2d horizontal_error(
        state_.position_odom.x - landing_origin_.x(),
        state_.position_odom.y - landing_origin_.y());
    if (!landing_touchdown_) {
      landing_touchdown_ =
          landing_phase_ == LandingPhase::kDescent &&
          landing_setpoint_.z() <=
              landing_ground_z_ +
                  landing_touchdown_height_tolerance_ &&
          state_.position_odom.z <=
              landing_ground_z_ +
                  landing_touchdown_height_tolerance_ &&
          std::abs(state_.velocity_odom.z) <=
              landing_touchdown_velocity_tolerance_ &&
          horizontal_error.norm() <=
              2.0 * landing_position_tolerance_;
    }

    reference.position.x = landing_setpoint_.x();
    reference.position.y = landing_setpoint_.y();
    reference.position.z = landing_setpoint_.z();
    reference.yaw = landing_course_;
    return reference;
  }

  bool validReference(
      const Reference& reference,
      std::string* reason) const {
    if (vehicle_type_ == "multirotor" &&
        !anyAxis(reference.use_position) &&
        !anyAxis(reference.use_velocity) &&
        !anyAxis(reference.use_acceleration) &&
        !reference.use_jerk) {
      *reason = "四旋翼参考未启用任何平移控制量";
      return false;
    }
    if (vehicle_type_ == "fixedwing" &&
        !anyAxis(reference.use_position) &&
        !anyAxis(reference.use_velocity)) {
      *reason = "固定翼参考至少需要位置或速度";
      return false;
    }
    if (!finiteSelected(
            reference.position,
            reference.use_position)) {
      *reason = "位置参考包含非法数值";
      return false;
    }
    if (!finiteSelected(
            reference.velocity,
            reference.use_velocity)) {
      *reason = "速度参考包含非法数值";
      return false;
    }
    if (!finiteSelected(
            reference.acceleration,
            reference.use_acceleration)) {
      *reason = "加速度参考包含非法数值";
      return false;
    }
    if (reference.use_jerk && !finite(reference.jerk)) {
      *reason = "jerk参考包含非法数值";
      return false;
    }
    if (reference.use_yaw && !std::isfinite(reference.yaw)) {
      *reason = "yaw参考包含非法数值";
      return false;
    }
    return true;
  }

  ControllerResult multirotorControl(
      const Reference& reference) {
    ControllerResult result;
    bool acceleration_feedback_required = false;
    for (int axis = 0; axis < 3; ++axis) {
      acceleration_feedback_required =
          acceleration_feedback_required ||
          (reference.use_acceleration[axis] &&
           !reference.use_position[axis] &&
           !reference.use_velocity[axis]);
    }
    if (acceleration_feedback_required &&
        (!state_.acceleration_fresh ||
         !finite(state_.acceleration_odom))) {
      acceleration_feedback_integral_state_.setZero();
      acceleration_feedback_active_.fill(false);
      result.reason =
          "纯加速度控制要求有效且新鲜的acceleration_odom";
      return result;
    }

    Eigen::Matrix3d rotation;
    if (!quaternionToMatrix(state_.orientation_odom_body,
                            &rotation)) {
      result.reason = "当前姿态四元数无效";
      return result;
    }

    const Eigen::Vector3d position(
        state_.position_odom.x, state_.position_odom.y,
        state_.position_odom.z);
    const Eigen::Vector3d velocity(
        state_.velocity_odom.x, state_.velocity_odom.y,
        state_.velocity_odom.z);
    const Eigen::Vector3d measured_acceleration(
        state_.acceleration_odom.x,
        state_.acceleration_odom.y,
        state_.acceleration_odom.z);
    Eigen::Vector3d position_reference = position;
    Eigen::Vector3d velocity_reference =
        Eigen::Vector3d::Zero();
    const Eigen::Vector3d requested_position(
        reference.position.x, reference.position.y,
        reference.position.z);
    const Eigen::Vector3d requested_velocity(
        reference.velocity.x, reference.velocity.y,
        reference.velocity.z);
    const Eigen::Vector3d acceleration_reference(
        reference.acceleration.x, reference.acceleration.y,
        reference.acceleration.z);
    Eigen::Vector3d jerk_feedforward = Eigen::Vector3d::Zero();

    for (int axis = 0; axis < 3; ++axis) {
      if (reference.use_position[axis]) {
        position_reference(axis) =
            requested_position(axis);
      }
      if (reference.use_velocity[axis]) {
        velocity_reference(axis) =
            requested_velocity(axis);
      }
    }
    const double horizontal_speed =
        velocity_reference.head<2>().norm();
    if (horizontal_speed > max_velocity_xy_) {
      velocity_reference.head<2>() *=
          max_velocity_xy_ / horizontal_speed;
    }
    velocity_reference.z() = clamp(
        velocity_reference.z(), -max_velocity_z_,
        max_velocity_z_);
    if (reference.use_jerk) {
      jerk_feedforward = Eigen::Vector3d(
          reference.jerk.x, reference.jerk.y,
          reference.jerk.z);
    }

    if (!acceleration_command_initialized_) {
      acceleration_command_state_.setZero();
      acceleration_command_initialized_ = true;
    }
    Eigen::Vector3d desired_acceleration =
        Eigen::Vector3d::Zero();
    for (int axis = 0; axis < 3; ++axis) {
      const std::array<bool, 2> controlled{{
          reference.use_position[axis],
          reference.use_velocity[axis] ||
              reference.use_position[axis]}};
      const Eigen::Vector2d current(
          position(axis), velocity(axis));
      const Eigen::Vector2d desired(
          position_reference(axis), velocity_reference(axis));
      desired_acceleration(axis) = mpc_.acceleration(
          current, desired, controlled,
          reference.use_acceleration[axis]
              ? acceleration_reference(axis)
              : 0.0);
      if (reference.use_velocity[axis] &&
          !reference.use_position[axis]) {
        const double velocity_error =
            velocity_reference(axis) - velocity(axis);
        velocity_integral_acceleration_state_(axis) =
            clamp(
                velocity_integral_acceleration_state_(axis) +
                    velocity_integral_gain_[axis] *
                        velocity_error * mpc_dt_,
                -velocity_integral_acceleration_limit_[axis],
                velocity_integral_acceleration_limit_[axis]);
        desired_acceleration(axis) +=
            velocity_integral_acceleration_state_(axis);
      } else {
        velocity_integral_acceleration_state_(axis) = 0.0;
      }
      const bool pure_acceleration_axis =
          reference.use_acceleration[axis] &&
          !reference.use_position[axis] &&
          !reference.use_velocity[axis];
      if (pure_acceleration_axis) {
        const double feedback_gain =
            std::max(0.0, acceleration_feedback_gain_[axis]);
        const double integral_gain =
            std::max(0.0, acceleration_integral_gain_[axis]);
        const double integral_limit = std::max(
            0.0,
            acceleration_integral_acceleration_limit_[axis]);
        if (!acceleration_feedback_active_[axis]) {
          acceleration_feedback_integral_state_(axis) =
              clamp(
                  acceleration_command_state_(axis) -
                      acceleration_reference(axis),
                  -integral_limit, integral_limit);
          acceleration_feedback_active_[axis] = true;
        }
        const double acceleration_error =
            acceleration_reference(axis) -
            measured_acceleration(axis);
        acceleration_feedback_integral_state_(axis) =
            clamp(
                acceleration_feedback_integral_state_(axis) +
                    integral_gain * acceleration_error *
                        mpc_dt_,
                -integral_limit, integral_limit);
        desired_acceleration(axis) +=
            feedback_gain * acceleration_error +
            acceleration_feedback_integral_state_(axis);
      } else {
        acceleration_feedback_integral_state_(axis) = 0.0;
        acceleration_feedback_active_[axis] = false;
      }
      if (reference.use_jerk) {
        desired_acceleration(axis) +=
            jerk_feedforward(axis) * mpc_dt_;
      }
    }
    const double desired_horizontal =
        desired_acceleration.head<2>().norm();
    if (desired_horizontal > max_acceleration_xy_) {
      desired_acceleration.head<2>() *=
          max_acceleration_xy_ / desired_horizontal;
    }
    desired_acceleration.z() = clamp(
        desired_acceleration.z(), -max_acceleration_z_,
        max_acceleration_z_);

    Eigen::Vector2d horizontal_step =
        desired_acceleration.head<2>() -
        acceleration_command_state_.head<2>();
    const double maximum_horizontal_step =
        max_jerk_xy_ * mpc_dt_;
    if (horizontal_step.norm() > maximum_horizontal_step) {
      horizontal_step *=
          maximum_horizontal_step / horizontal_step.norm();
    }
    acceleration_command_state_.head<2>() += horizontal_step;
    acceleration_command_state_.z() += clamp(
        desired_acceleration.z() -
            acceleration_command_state_.z(),
        -max_jerk_z_ * mpc_dt_,
        max_jerk_z_ * mpc_dt_);
    const double horizontal_acceleration =
        acceleration_command_state_.head<2>().norm();
    if (horizontal_acceleration > max_acceleration_xy_) {
      acceleration_command_state_.head<2>() *=
          max_acceleration_xy_ / horizontal_acceleration;
    }
    acceleration_command_state_.z() = clamp(
        acceleration_command_state_.z(), -max_acceleration_z_,
        max_acceleration_z_);
    const Eigen::Vector3d acceleration_command =
        acceleration_command_state_;

    Eigen::Vector3d total_acceleration =
        acceleration_command +
        Eigen::Vector3d(0.0, 0.0, gravity_);
    total_acceleration.z() =
        std::max(0.1 * gravity_, total_acceleration.z());
    const double maximum_horizontal =
        total_acceleration.z() * std::tan(maximum_tilt_);
    if (total_acceleration.head<2>().norm() >
        maximum_horizontal) {
      total_acceleration.head<2>() *=
          maximum_horizontal /
          total_acceleration.head<2>().norm();
    }
    if (total_acceleration.norm() < 1e-6) {
      result.reason = "期望合力为零";
      return result;
    }

    const double desired_yaw =
        reference.use_yaw
            ? reference.yaw
            : std::atan2(rotation(1, 0), rotation(0, 0));
    const Eigen::Vector3d b3_desired =
        total_acceleration.normalized();
    const Eigen::Vector3d heading(
        std::cos(desired_yaw), std::sin(desired_yaw), 0.0);
    Eigen::Vector3d b2_desired = b3_desired.cross(heading);
    if (b2_desired.norm() < 1e-6) {
      result.reason = "无法构造期望姿态";
      return result;
    }
    b2_desired.normalize();
    const Eigen::Vector3d b1_desired =
        b2_desired.cross(b3_desired).normalized();
    Eigen::Matrix3d desired_rotation;
    desired_rotation.col(0) = b1_desired;
    desired_rotation.col(1) = b2_desired;
    desired_rotation.col(2) = b3_desired;

    const Eigen::Matrix3d error_matrix =
        0.5 * (desired_rotation.transpose() * rotation -
               rotation.transpose() * desired_rotation);
    const Eigen::Vector3d attitude_error(
        error_matrix(2, 1), error_matrix(0, 2),
        error_matrix(1, 0));
    for (int axis = 0; axis < 3; ++axis) {
      result.body_rate(axis) = clamp(
          -attitude_gain_[axis] * attitude_error(axis),
          -maximum_body_rate_[axis], maximum_body_rate_[axis]);
    }
    if (reference.use_yaw_rate) {
      result.body_rate.z() = clamp(
          result.body_rate.z() + reference.yaw_rate,
          -maximum_body_rate_[2], maximum_body_rate_[2]);
    }

    const double acceleration_along_current_z =
        total_acceleration.dot(rotation.col(2));
    result.thrust = clamp(
        hover_throttle_ * acceleration_along_current_z /
            gravity_,
        minimum_throttle_, maximum_throttle_);
    result.valid = result.body_rate.allFinite() &&
                   std::isfinite(result.thrust);
    if (!result.valid) {
      result.reason = "四旋翼控制输出包含非法数值";
    }
    return result;
  }

  ControllerResult fixedwingControl(
      const Reference& reference,
      const ros::Time& now) {
    ControllerResult result;
    if (!state_.airspeed_valid ||
        state_.airspeed < 0.0 ||
        !std::isfinite(state_.airspeed)) {
      result.reason = "固定翼空速无效";
      return result;
    }
    Eigen::Matrix3d rotation;
    if (!quaternionToMatrix(state_.orientation_odom_body,
                            &rotation)) {
      result.reason = "当前姿态四元数无效";
      return result;
    }
    tf2::Quaternion quaternion;
    tf2::fromMsg(state_.orientation_odom_body, quaternion);
    quaternion.normalize();
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);

    const double current_course =
        std::isfinite(state_.course) ? state_.course : yaw;
    double dt = 1.0 / std::max(1.0, control_rate_);
    if (!last_fixedwing_control_time_.isZero()) {
      const double measured_dt =
          (now - last_fixedwing_control_time_).toSec();
      if (measured_dt > 0.0 && std::isfinite(measured_dt)) {
        dt = clamp(measured_dt, 1e-3, 0.10);
      }
    }
    last_fixedwing_control_time_ = now;

    const bool external_point_reference =
        !reference.trajectory_reference &&
        !internal_reference_active_ &&
        !idle_reference_active_ &&
        !landing_active_;
    double desired_course =
        reference.use_yaw &&
                (reference.trajectory_reference ||
                 external_point_reference)
            ? reference.yaw
            : current_course;
    double desired_airspeed = cruise_airspeed_;
    bool curvature_rate_valid = false;
    double curvature_rate = 0.0;
    if (reference.trajectory_reference &&
        reference.use_velocity[0] &&
        reference.use_velocity[1] &&
        reference.use_acceleration[0] &&
        reference.use_acceleration[1]) {
      const double velocity_squared =
          reference.velocity.x * reference.velocity.x +
          reference.velocity.y * reference.velocity.y;
      if (velocity_squared > 0.25) {
        curvature_rate =
            (reference.velocity.x * reference.acceleration.y -
             reference.velocity.y * reference.acceleration.x) /
            velocity_squared;
        curvature_rate_valid = std::isfinite(curvature_rate);
      }
    }
    const bool explicit_trajectory_yaw_rate =
        reference.trajectory_reference &&
        reference.use_yaw_rate &&
        std::abs(reference.yaw_rate) > 1e-6;
    double course_rate_feedforward = 0.0;
    if (explicit_trajectory_yaw_rate) {
      course_rate_feedforward = reference.yaw_rate;
      if (curvature_rate_valid &&
          std::abs(reference.yaw_rate - curvature_rate) > 0.10) {
        ROS_WARN_THROTTLE(
            1.0,
            "[xd_uav_controller] 固定翼轨迹yaw_rate"
            "与速度/加速度曲率不一致"
            "(yaw_rate=%.3f curvature=%.3f)，"
            "优先使用显式yaw_rate",
            reference.yaw_rate, curvature_rate);
      }
    } else if (reference.trajectory_reference &&
               curvature_rate_valid) {
      course_rate_feedforward = curvature_rate;
    } else if ((external_point_reference ||
                fixedwing_loiter_active_) &&
               reference.use_yaw_rate) {
      course_rate_feedforward = reference.yaw_rate;
    }
    const bool use_external_horizontal_position =
        (reference.use_position[0] ||
         reference.use_position[1]) &&
        !internal_reference_active_ &&
        !idle_reference_active_;
    if (use_external_horizontal_position) {
      const double target_x =
          reference.use_position[0]
              ? reference.position.x
              : state_.position_odom.x;
      const double target_y =
          reference.use_position[1]
              ? reference.position.y
              : state_.position_odom.y;
      const double dx =
          target_x - state_.position_odom.x;
      const double dy =
          target_y - state_.position_odom.y;
      if (std::hypot(dx, dy) > 1.0) {
        desired_course = std::atan2(dy, dx);
      }
    }
    if (reference.use_velocity[0] ||
        reference.use_velocity[1]) {
      const double horizontal_speed =
          std::hypot(reference.velocity.x,
                     reference.velocity.y);
      if (horizontal_speed > 0.5) {
        const double velocity_course =
            std::atan2(reference.velocity.y,
                       reference.velocity.x);
        desired_course = velocity_course;
        desired_airspeed = clamp(
            horizontal_speed, minimum_airspeed_,
            maximum_airspeed_);
        if (use_external_horizontal_position) {
          double guidance_lookahead_distance =
              fixedwing_guidance_lookahead_distance_;
          if (reference.trajectory_reference &&
              fixedwing_adaptive_lookahead_) {
            double target_lookahead =
                fixedwing_maximum_lookahead_distance_;
            if (std::abs(course_rate_feedforward) > 1e-6) {
              const double turn_radius =
                  horizontal_speed /
                  std::abs(course_rate_feedforward);
              target_lookahead = clamp(
                  fixedwing_lookahead_turn_radius_gain_ *
                      turn_radius,
                  fixedwing_minimum_lookahead_distance_,
                  fixedwing_maximum_lookahead_distance_);
            }
            if (!fixedwing_lookahead_initialized_) {
              filtered_fixedwing_lookahead_distance_ =
                  target_lookahead;
              fixedwing_lookahead_initialized_ = true;
            } else {
              const double filter_alpha =
                  fixedwing_lookahead_filter_time_constant_ <=
                          1e-6
                      ? 1.0
                      : dt /
                            (fixedwing_lookahead_filter_time_constant_ +
                             dt);
              filtered_fixedwing_lookahead_distance_ +=
                  filter_alpha *
                  (target_lookahead -
                   filtered_fixedwing_lookahead_distance_);
            }
            guidance_lookahead_distance =
                filtered_fixedwing_lookahead_distance_;
          }
          // Track a virtual point in front of the sampled trajectory
          // position. The velocity supplies the path tangent while
          // the vector from the aircraft to this look-ahead point
          // closes the horizontal position error. This prevents the
          // velocity feed-forward from discarding cross-track
          // feedback and shifting the whole flown path.
          const double inverse_speed =
              1.0 / horizontal_speed;
          const double target_x =
              reference.use_position[0]
                  ? reference.position.x
                  : state_.position_odom.x;
          const double target_y =
              reference.use_position[1]
                  ? reference.position.y
                  : state_.position_odom.y;
          const double guidance_x =
              target_x +
              guidance_lookahead_distance *
                  reference.velocity.x * inverse_speed;
          const double guidance_y =
              target_y +
              guidance_lookahead_distance *
                  reference.velocity.y * inverse_speed;
          const double guidance_dx =
              guidance_x - state_.position_odom.x;
          const double guidance_dy =
              guidance_y - state_.position_odom.y;
          if (std::hypot(guidance_dx, guidance_dy) > 1.0) {
            desired_course =
                std::atan2(guidance_dy, guidance_dx);
          }
        }
      }
    }
    if (!std::isfinite(desired_course)) {
      desired_course = yaw;
    }

    const double airspeed =
        std::max(minimum_airspeed_, state_.airspeed);
    const double course_error =
        wrapAngle(desired_course - current_course);
    const double desired_course_rate = clamp(
        course_error / std::max(0.1, course_time_constant_) +
            course_rate_feedforward,
        -gravity_ * std::tan(max_roll_) / airspeed,
        gravity_ * std::tan(max_roll_) / airspeed);
    double desired_roll = clamp(
        -std::atan2(airspeed * desired_course_rate, gravity_),
        -max_roll_, max_roll_);

    double desired_climb_rate = 0.0;
    double altitude_error = 0.0;
    if (reference.use_position[2]) {
      altitude_error =
          reference.position.z - state_.position_odom.z;
      desired_climb_rate +=
          altitude_error / std::max(0.1, altitude_time_constant_);
    }
    if (reference.use_velocity[2]) {
      desired_climb_rate += reference.velocity.z;
    }

    // The integral output is expressed directly as a climb-rate correction.
    // It removes persistent altitude bias without changing trajectory vz
    // feed-forward. Disable it for takeoff/landing, where dedicated pitch
    // laws and saturation would otherwise cause wind-up.
    const bool altitude_integrator_enabled =
        reference.use_position[2] && !takeoff_active_ &&
        !landing_active_ && altitude_integral_gain_ > 0.0 &&
        altitude_integral_climb_rate_limit_ > 0.0;
    if (altitude_integrator_enabled) {
      const double integral_delta =
          altitude_integral_gain_ * altitude_error * dt;
      const double candidate = clamp(
          fixedwing_altitude_integral_climb_rate_ +
              integral_delta,
          -altitude_integral_climb_rate_limit_,
          altitude_integral_climb_rate_limit_);
      const double candidate_climb_rate =
          desired_climb_rate + candidate;
      const bool winds_up_high =
          candidate_climb_rate > max_climb_rate_ &&
          integral_delta > 0.0;
      const bool winds_up_low =
          candidate_climb_rate < -max_climb_rate_ &&
          integral_delta < 0.0;
      if (!winds_up_high && !winds_up_low) {
        fixedwing_altitude_integral_climb_rate_ = candidate;
      }
      desired_climb_rate +=
          fixedwing_altitude_integral_climb_rate_;
    } else {
      fixedwing_altitude_integral_climb_rate_ = 0.0;
    }
    desired_climb_rate = clamp(
        desired_climb_rate, -max_climb_rate_, max_climb_rate_);

    const double measured_climb_rate = state_.velocity_odom.z;
    if (!filtered_climb_rate_initialized_ ||
        !std::isfinite(filtered_climb_rate_)) {
      filtered_climb_rate_ = measured_climb_rate;
      filtered_climb_rate_initialized_ = true;
    } else {
      const double filter_alpha =
          climb_rate_filter_time_constant_ <= 1e-6
              ? 1.0
              : dt / (climb_rate_filter_time_constant_ + dt);
      filtered_climb_rate_ +=
          filter_alpha *
          (measured_climb_rate - filtered_climb_rate_);
    }
    const double climb_rate_error =
        desired_climb_rate - filtered_climb_rate_;
    double desired_pitch = clamp(
        -std::asin(clamp(desired_climb_rate / airspeed,
                         -0.95, 0.95)) -
            climb_rate_pitch_gain_ * climb_rate_error,
        -max_pitch_, max_pitch_);
    double pitch_rate_feedforward = 0.0;
    if (reference.use_acceleration[2]) {
      // With the repository's ROS FLU convention, positive inertial-z
      // acceleration requires a negative (nose-up) pitch rate. This term
      // anticipates the changing flight-path angle of a 3-D trajectory.
      pitch_rate_feedforward =
          -vertical_acceleration_pitch_rate_gain_ *
          reference.acceleration.z /
          std::max(minimum_airspeed_, airspeed);
    }
    double throttle = clamp(
        trim_throttle_ +
            airspeed_throttle_gain_ *
                (desired_airspeed - state_.airspeed) +
            climb_rate_throttle_gain_ * desired_climb_rate,
        minimum_throttle_, maximum_throttle_);

    if (takeoff_active_) {
      desired_roll = clamp(desired_roll, -0.25, 0.25);
      throttle = fixedwing_takeoff_throttle_;
      desired_pitch =
          state_.airspeed >= fixedwing_rotate_airspeed_
              ? -std::abs(fixedwing_climb_pitch_)
              : 0.0;
    }
    if (landing_active_) {
      desired_roll = clamp(
          desired_roll,
          -fixedwing_landing_max_roll_,
          fixedwing_landing_max_roll_);
      if (fixedwing_landing_phase_ ==
          FixedwingLandingPhase::kRollout) {
        desired_roll = clamp(desired_roll, -0.15, 0.15);
      }
      if (fixedwing_landing_phase_ !=
          FixedwingLandingPhase::kApproach) {
        const double height_above_touchdown = std::max(
            0.0, state_.position_odom.z - landing_ground_z_);
        const double contact_height =
            fixedwingLandingContactHeight();
        const double flare_progress =
            fixedwingLandingFlareProgress(
                height_above_touchdown);
        const double target_throttle_scale =
            height_above_touchdown <= contact_height
                ? 0.0
                : 1.0 - flare_progress;
        // Never restore throttle after a bounce or noisy height sample.
        fixedwing_landing_throttle_scale_ = std::min(
            fixedwing_landing_throttle_scale_,
            target_throttle_scale);
        throttle *= fixedwing_landing_throttle_scale_;
        // Blend the continuously computed descent attitude toward level
        // flight. This removes the nose-down contact without introducing
        // a fixed flare pitch or an instantaneous attitude step.
        desired_pitch *= 1.0 - flare_progress;
      }
      if (landing_touchdown_) {
        throttle = 0.0;
      }
    }

    result.body_rate.x() = clamp(
        (desired_roll - roll) /
            std::max(0.1, roll_time_constant_),
        -maximum_body_rate_[0], maximum_body_rate_[0]);
    result.body_rate.y() = clamp(
        (desired_pitch - pitch) /
                std::max(0.1, pitch_time_constant_) +
            pitch_rate_feedforward,
        -maximum_body_rate_[1], maximum_body_rate_[1]);
    result.body_rate.z() = clamp(
        -gravity_ * std::tan(desired_roll) / airspeed,
        -maximum_body_rate_[2], maximum_body_rate_[2]);
    const bool landing_allows_zero_throttle =
        landing_active_ &&
        fixedwing_landing_phase_ !=
            FixedwingLandingPhase::kApproach;
    result.thrust = clamp(
        throttle,
        landing_allows_zero_throttle ? 0.0
                                     : minimum_throttle_,
        maximum_throttle_);
    result.valid = result.body_rate.allFinite() &&
                   std::isfinite(result.thrust);
    if (!result.valid) {
      result.reason = "固定翼控制输出包含非法数值";
    }
    return result;
  }

  bool useTransformedReference(
      const bool transformed,
      const Reference& candidate,
      const std::string& transform_reason,
      const ros::Time& now,
      Reference* reference,
      std::string* reason) {
    if (transformed) {
      reference_ = candidate;
      have_normalized_reference_ = true;
      active_reference_transform_failure_since_ = ros::Time();
      *reference = candidate;
      return true;
    }
    if (active_reference_transform_failure_since_.isZero()) {
      active_reference_transform_failure_since_ = now;
    }
    const double failure_duration =
        (now -
         active_reference_transform_failure_since_).toSec();
    if (have_normalized_reference_ &&
        failure_duration <=
            reference_transform_failure_grace_) {
      *reference = reference_;
      reference->header.stamp = now;
      ROS_WARN_THROTTLE(
          1.0,
          "[xd_uav_controller] 参考TF短暂失效(%.2fs/%.2fs)，"
          "保持最后一个有效控制目标: %s",
          failure_duration,
          reference_transform_failure_grace_,
          transform_reason.c_str());
      return true;
    }
    *reason =
        "活动参考坐标系持续失效: " + transform_reason;
    return false;
  }

  bool resolvePointReference(
      const ros::Time& now,
      Reference* reference,
      std::string* reason) {
    Reference candidate;
    std::string transform_reason;
    const bool transformed =
        transformReferenceToControl(
            reference_source_,
            state_.header.frame_id, 0.0,
            &candidate, &transform_reason);
    return useTransformedReference(
        transformed, candidate, transform_reason,
        now, reference, reason);
  }

  bool resolveTrajectoryReference(
      const ros::Time& now,
      Reference* reference,
      std::string* reason) {
    if (vehicle_type_ == "fixedwing" &&
        trajectory_active_ &&
        !reference_trajectory_.points.empty()) {
      const double elapsed =
          (now - reference_trajectory_start_).toSec();
      const double trajectory_duration =
          reference_trajectory_.points.back()
              .time_from_start.toSec();
      if (elapsed >= trajectory_duration) {
        startFixedwingLoiter(
            state_.position_odom.z, "轨迹执行完成");
        *reference = makeFixedwingLoiterReference();
        return true;
      }
    }
    Reference source_reference;
    if (!sampleTrajectory(
            now, &source_reference, reason)) {
      return false;
    }
    Reference candidate;
    std::string transform_reason;
    const bool transformed =
        transformReferenceToControl(
            source_reference, state_.header.frame_id,
            0.0, &candidate, &transform_reason);
    return useTransformedReference(
        transformed, candidate, transform_reason,
        now, reference, reason);
  }

  void timerCallback(const ros::TimerEvent&) {
    const ros::Time now = ros::Time::now();
    xd_uav_controller::ControlCommand command;
    command.header.stamp = now;
    command.vehicle_type = vehicle_type_id_;
    command.controller =
        vehicle_type_ == "multirotor"
            ? "finite_horizon_mpc_so3"
            : (landing_active_
                   ? "fixedwing_course_energy_landing"
                   : fixedwing_loiter_active_
                   ? "fixedwing_course_energy_loiter"
                   : "fixedwing_course_energy");
    command.takeoff_active = takeoff_active_;
    command.landing_active = landing_active_;
    command.landing_touchdown = landing_touchdown_;

    if (!have_state_) {
      command.rejection_reason = "尚未收到控制状态";
      command_publisher_.publish(command);
      return;
    }
    command.header.frame_id = state_.body_frame_id;
    const double state_age =
        (now - last_state_receive_).toSec();
    if (state_age > state_timeout_ || !state_.state_valid ||
        !state_.odometry_fresh || !state_.imu_fresh) {
      acceleration_command_initialized_ = false;
      velocity_integral_acceleration_state_.setZero();
      acceleration_feedback_integral_state_.setZero();
      acceleration_feedback_active_.fill(false);
      resetFixedwingControlState();
      command.rejection_reason = "控制状态无效或超时";
      command_publisher_.publish(command);
      return;
    }
    if (state_.vehicle_type != vehicle_type_id_) {
      command.rejection_reason = "状态机型与控制器配置不一致";
      command_publisher_.publish(command);
      return;
    }

    Reference reference;
    if (landing_active_) {
      reference = makeLandingReference();
    } else if (internal_reference_active_) {
      reference = makeTakeoffReference();
    } else if (idle_reference_active_) {
      reference = makeIdleReference();
    } else if (have_reference_) {
      if (trajectory_active_) {
        std::string trajectory_reason;
        if (!resolveTrajectoryReference(
                now, &reference, &trajectory_reason)) {
          command.rejection_reason = trajectory_reason;
          command_publisher_.publish(command);
          return;
        }
      } else {
        if (!point_reference_latched_ &&
            (now - last_reference_receive_).toSec() >
                reference_timeout_) {
          if (vehicle_type_ == "fixedwing") {
            startFixedwingLoiter(
                state_.position_odom.z,
                "外部PositionTarget参考超时");
            reference = makeFixedwingLoiterReference();
          } else {
            have_reference_ = false;
            have_normalized_reference_ = false;
            point_reference_latched_ = false;
            captureIdleReference();
            reference = makeIdleReference();
            ROS_WARN_THROTTLE(
                1.0,
                "[xd_uav_controller] 流式PositionTarget超时，"
                "已捕获当前位置进入悬停");
          }
        } else {
          std::string point_reason;
          if (!resolvePointReference(
                  now, &reference, &point_reason)) {
            command.rejection_reason = point_reason;
            command_publisher_.publish(command);
            return;
          }
        }
      }
    } else if (have_reference_error_) {
      command.rejection_reason =
          "外部控制参考无效: " + reference_input_error_;
      command_publisher_.publish(command);
      return;
    } else {
      command.rejection_reason = "尚未收到控制参考";
      command_publisher_.publish(command);
      return;
    }

    std::string reference_reason;
    if (!validReference(reference, &reference_reason)) {
      command.rejection_reason = reference_reason;
      command_publisher_.publish(command);
      return;
    }
    if (!reference.header.frame_id.empty() &&
        canonicalFrame(reference.header.frame_id) !=
            canonicalFrame(state_.header.frame_id)) {
      command.rejection_reason =
          "控制参考坐标系与控制状态坐标系不一致";
      command_publisher_.publish(command);
      return;
    }
    const ControllerResult result =
        vehicle_type_ == "multirotor"
            ? multirotorControl(reference)
            : fixedwingControl(reference, now);
    command.body_rate.x = result.body_rate.x();
    command.body_rate.y = result.body_rate.y();
    command.body_rate.z = result.body_rate.z();
    command.thrust = result.thrust;
    command.valid = result.valid;
    command.rejection_reason = result.reason;
    if (vehicle_type_ == "fixedwing") {
      if (landing_active_) {
        command.controller =
            "fixedwing_course_energy_landing";
      } else if (fixedwing_loiter_active_) {
        command.controller =
            "fixedwing_course_energy_loiter";
      }
    }
    command.takeoff_active = takeoff_active_;
    command.landing_active = landing_active_;
    command.landing_touchdown = landing_touchdown_;
    command_publisher_.publish(command);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  ros::Subscriber state_subscriber_;
  ros::Subscriber reference_position_target_subscriber_;
  ros::Subscriber reference_trajectory_subscriber_;
  ros::Subscriber simple_goal_subscriber_;
  ros::Subscriber local_alignment_valid_subscriber_;
  ros::Publisher reference_position_target_publisher_;
  ros::Publisher reference_trajectory_path_publisher_;
  ros::Publisher command_publisher_;
  ros::ServiceServer internal_command_server_;
  ros::Timer timer_;

  xd_uav_controller::ControlState state_;
  Reference reference_;
  Reference reference_source_;
  trajectory_msgs::MultiDOFJointTrajectory
      reference_trajectory_;
  ros::Time last_state_receive_;
  ros::Time last_reference_receive_;
  ros::Time reference_trajectory_start_;
  ros::Time last_local_alignment_valid_receive_;
  ros::Time active_reference_transform_failure_since_;
  bool have_state_{false};
  bool have_reference_{false};
  bool have_reference_error_{false};
  bool have_normalized_reference_{false};
  bool have_local_alignment_valid_{false};
  bool local_alignment_valid_{false};
  bool trajectory_active_{false};
  bool point_reference_latched_{false};
  uint32_t simple_goal_sequence_counter_{0};
  bool internal_reference_active_{false};
  bool idle_reference_active_{false};
  bool takeoff_active_{false};
  bool fixedwing_loiter_active_{false};
  bool landing_active_{false};
  bool landing_return_home_{false};
  bool landing_touchdown_{false};
  bool have_takeoff_origin_{false};
  bool have_home_reference_{false};
  bool acceleration_command_initialized_{false};
  Reference home_reference_source_;
  Eigen::Vector3d takeoff_origin_{Eigen::Vector3d::Zero()};
  double takeoff_target_z_{0.0};
  double takeoff_course_{0.0};
  Eigen::Vector3d fixedwing_loiter_center_{
      Eigen::Vector3d::Zero()};
  Eigen::Vector3d landing_origin_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d landing_setpoint_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d fixedwing_landing_approach_point_{
      Eigen::Vector3d::Zero()};
  double landing_ground_z_{0.0};
  double landing_target_z_{0.0};
  double landing_course_{0.0};
  ros::Time last_landing_update_;
  Eigen::Vector3d acceleration_command_state_{
      Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity_integral_acceleration_state_{
      Eigen::Vector3d::Zero()};
  Eigen::Vector3d acceleration_feedback_integral_state_{
      Eigen::Vector3d::Zero()};
  std::array<bool, 3> acceleration_feedback_active_{{
      false, false, false}};
  Eigen::Vector3d idle_position_{Eigen::Vector3d::Zero()};
  double idle_course_{0.0};
  double idle_airspeed_{15.0};

  std::string vehicle_type_;
  uint8_t vehicle_type_id_{
      xd_uav_controller::ControlState::VEHICLE_MULTIROTOR};
  double control_rate_{100.0};
  double state_timeout_{0.2};
  double reference_timeout_{0.5};
  double reference_transform_timeout_{0.03};
  double maximum_transform_age_{0.5};
  double reference_transform_failure_grace_{0.5};
  double maximum_inertial_frame_tilt_{0.1};
  double global_alignment_timeout_{1.0};
  bool require_global_alignment_{true};
  std::vector<std::string> allowed_reference_frames_;
  std::vector<std::string> global_alignment_frames_;
  bool simple_goal_use_message_z_{false};
  std::string reference_input_error_;

  FiniteHorizonAxisMpc mpc_;
  double mpc_dt_{0.01};
  double gravity_{9.80665};
  double hover_throttle_{0.5};
  double minimum_throttle_{0.05};
  double maximum_throttle_{0.9};
  double maximum_tilt_{0.61};
  std::array<double, 3> attitude_gain_{{5.0, 5.0, 2.0}};
  std::array<double, 3> maximum_body_rate_{{3.0, 3.0, 2.0}};
  double max_velocity_xy_{5.0};
  double max_velocity_z_{3.0};
  double max_acceleration_xy_{5.0};
  double max_acceleration_z_{4.0};
  double max_jerk_xy_{8.0};
  double max_jerk_z_{6.0};
  std::array<double, 3> velocity_integral_gain_{{
      1.0, 1.0, 2.0}};
  std::array<double, 3>
      velocity_integral_acceleration_limit_{{
          1.0, 1.0, 1.0}};
  std::array<double, 3> acceleration_feedback_gain_{{
      0.25, 0.25, 0.35}};
  std::array<double, 3> acceleration_integral_gain_{{
      0.20, 0.20, 0.40}};
  std::array<double, 3>
      acceleration_integral_acceleration_limit_{{
          1.0, 1.0, 1.0}};

  double cruise_airspeed_{15.0};
  double minimum_airspeed_{11.0};
  double maximum_airspeed_{24.0};
  double trim_throttle_{0.25};
  double airspeed_throttle_gain_{0.08};
  double course_time_constant_{3.0};
  double fixedwing_guidance_lookahead_distance_{60.0};
  bool fixedwing_adaptive_lookahead_{true};
  double fixedwing_minimum_lookahead_distance_{30.0};
  double fixedwing_maximum_lookahead_distance_{60.0};
  double fixedwing_lookahead_turn_radius_gain_{1.2};
  double fixedwing_lookahead_filter_time_constant_{0.50};
  double filtered_fixedwing_lookahead_distance_{60.0};
  bool fixedwing_lookahead_initialized_{false};
  double altitude_time_constant_{5.0};
  double altitude_integral_gain_{0.02};
  double altitude_integral_climb_rate_limit_{1.0};
  double climb_rate_pitch_gain_{0.06};
  double climb_rate_filter_time_constant_{0.50};
  double vertical_acceleration_pitch_rate_gain_{1.0};
  double climb_rate_throttle_gain_{0.03};
  double fixedwing_altitude_integral_climb_rate_{0.0};
  double filtered_climb_rate_{0.0};
  bool filtered_climb_rate_initialized_{false};
  ros::Time last_fixedwing_control_time_;
  double roll_time_constant_{0.45};
  double pitch_time_constant_{0.45};
  double max_roll_{0.79};
  double max_pitch_{0.35};
  double max_climb_rate_{4.0};
  double fixedwing_loiter_radius_{80.0};
  double fixedwing_loiter_airspeed_{15.0};
  int fixedwing_loiter_direction_{1};
  double fixedwing_loiter_radial_gain_{1.0};
  double fixedwing_loiter_max_course_correction_{0.70};

  double takeoff_default_altitude_{2.0};
  double takeoff_max_velocity_{0.8};
  double takeoff_max_acceleration_{0.6};
  double takeoff_position_tolerance_{0.15};
  double takeoff_velocity_tolerance_{0.15};
  double fixedwing_takeoff_throttle_{0.9};
  double fixedwing_rotate_airspeed_{12.0};
  double fixedwing_climb_airspeed_{15.0};
  double fixedwing_climb_pitch_{0.21};
  double fixedwing_altitude_tolerance_{3.0};
  enum class LandingPhase {
    kNone,
    kApproach,
    kDescent,
  };
  enum class FixedwingLandingPhase {
    kNone,
    kApproach,
    kGlideSlope,
    kRollout,
  };
  LandingPhase landing_phase_{LandingPhase::kNone};
  FixedwingLandingPhase fixedwing_landing_phase_{
      FixedwingLandingPhase::kNone};
  double landing_descent_velocity_{0.35};
  double landing_final_descent_velocity_{0.15};
  double landing_slow_height_{0.7};
  double landing_return_velocity_{1.0};
  double landing_position_tolerance_{0.20};
  double landing_velocity_tolerance_{0.20};
  double landing_touchdown_offset_{0.15};
  double landing_touchdown_height_tolerance_{0.10};
  double landing_touchdown_velocity_tolerance_{0.20};
  double fixedwing_landing_approach_distance_{400.0};
  double fixedwing_landing_approach_height_{30.0};
  double fixedwing_landing_approach_airspeed_{15.0};
  double fixedwing_landing_approach_acceptance_radius_{50.0};
  double fixedwing_landing_approach_altitude_tolerance_{8.0};
  double fixedwing_landing_approach_course_tolerance_{0.52};
  double fixedwing_landing_approach_capture_turn_radius_factor_{2.0};
  double fixedwing_landing_line_lookahead_distance_{80.0};
  double fixedwing_landing_max_course_correction_{0.70};
  double fixedwing_landing_glide_slope_angle_{0.08};
  double fixedwing_landing_local_touchdown_ahead_{400.0};
  double fixedwing_landing_rollout_distance_{200.0};
  double fixedwing_landing_max_roll_{0.35};
  double fixedwing_landing_touchdown_height_tolerance_{1.0};
  double fixedwing_landing_touchdown_groundspeed_{2.0};
  double fixedwing_landing_touchdown_vertical_speed_{2.0};
  double fixedwing_landing_throttle_scale_{1.0};
  std::string home_mode_{"takeoff"};
  std::string home_frame_config_{"local_origin"};
  Eigen::Vector3d fixed_home_position_{
      Eigen::Vector3d::Zero()};
  double fixed_home_yaw_{0.0};
  bool use_home_yaw_{false};
};

int main(int argc, char** argv) {
  std::setlocale(LC_ALL, "");
  ros::init(argc, argv, "controller");
  try {
    ControllerNode node;
    ros::spin();
  } catch (const std::exception& exception) {
    ROS_FATAL("[xd_uav_controller] 启动失败: %s",
              exception.what());
    return 1;
  }
  return 0;
}
