#include <algorithm>
#include <array>
#include <clocale>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/PositionTarget.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/Range.h>
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
#include <xd_uav_controller/PathStatus.h>
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

template <typename T>
void loadParameterWithLegacy(
    const ros::NodeHandle& node_handle,
    const std::string& name,
    const std::string& legacy_name,
    T* value,
    const T& fallback) {
  *value = fallback;
  if (node_handle.getParam(name, *value)) {
    return;
  }
  if (node_handle.getParam(legacy_name, *value)) {
    ROS_WARN(
        "[xd_uav_controller] 参数%s已迁移为%s；"
        "本次仍兼容旧路径",
        legacy_name.c_str(), name.c_str());
  }
}

std::array<double, 3> loadVector3WithLegacy(
    const ros::NodeHandle& node_handle,
    const std::string& name,
    const std::string& legacy_name,
    const std::array<double, 3>& fallback) {
  std::vector<double> values;
  std::string loaded_name = name;
  if (!node_handle.getParam(name, values)) {
    loaded_name = legacy_name;
    if (!node_handle.getParam(legacy_name, values)) {
      return fallback;
    }
    ROS_WARN(
        "[xd_uav_controller] 参数%s已迁移为%s；"
        "本次仍兼容旧路径",
        legacy_name.c_str(), name.c_str());
  }
  if (values.size() != 3) {
    throw std::runtime_error(
        loaded_name + "必须包含3个数值");
  }
  return {{values[0], values[1], values[2]}};
}

std::array<double, 2> loadVector2WithLegacy(
    const ros::NodeHandle& node_handle,
    const std::string& name,
    const std::string& legacy_name,
    const std::array<double, 2>& fallback) {
  std::vector<double> values;
  std::string loaded_name = name;
  if (!node_handle.getParam(name, values)) {
    loaded_name = legacy_name;
    if (!node_handle.getParam(legacy_name, values)) {
      return fallback;
    }
    ROS_WARN(
        "[xd_uav_controller] 参数%s已迁移为%s；"
        "本次仍兼容旧路径",
        legacy_name.c_str(), name.c_str());
  }
  if (values.size() != 2) {
    throw std::runtime_error(
        loaded_name + "必须包含2个数值");
  }
  return {{values[0], values[1]}};
}

struct MultirotorVerticalLandingConfig {
  double descent_velocity{0.35};
  double final_descent_velocity{0.15};
  double slow_height{0.7};
  double touchdown_offset{0.15};
  double touchdown_height_tolerance{0.10};
  double touchdown_velocity_tolerance{0.20};
};

struct DistanceSensorLandingConfig
    : public MultirotorVerticalLandingConfig {
  double timeout{0.5};
  double velocity_filter_time_constant{0.30};
};

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
  bool path_reference{false};
};

// Canonical target consumed by the fixed-wing control law. Input adapters
// may start from a trajectory, a masked PositionTarget or an internal mode,
// but source-specific masks and geometry do not pass beyond this boundary.
struct FixedwingControlTarget {
  double course{0.0};
  double course_rate_feedforward{0.0};
  double airspeed{0.0};
  double altitude_error{0.0};
  double climb_rate_feedforward{0.0};
  double vertical_acceleration_feedforward{0.0};
  bool use_altitude_feedback{false};
  bool enable_course_integrator{false};
  bool enable_altitude_integrator{false};
};

}  // namespace

class ControllerNode {
 public:
  ControllerNode()
      : private_nh_("~"), tf_listener_(tf_buffer_) {
    loadParameters();
    state_subscriber_ = nh_.subscribe(
        "state", 20, &ControllerNode::stateCallback, this);
    distance_sensor_subscriber_ = nh_.subscribe(
        "distance_sensor", 20,
        &ControllerNode::distanceSensorCallback, this);
    reference_position_target_subscriber_ = nh_.subscribe(
        "reference_position_target", 20,
        &ControllerNode::referencePositionTargetCallback, this);
    reference_position_target_publisher_ =
        nh_.advertise<mavros_msgs::PositionTarget>(
            "reference_position_target", 1, true);
    reference_trajectory_subscriber_ = nh_.subscribe(
        "reference_trajectory", 5,
        &ControllerNode::referenceTrajectoryCallback, this);
    reference_path_subscriber_ = nh_.subscribe(
        "reference_path", 2,
        &ControllerNode::referencePathCallback, this);
    simple_goal_subscriber_ = nh_.subscribe(
        "simple_goal", 5,
        &ControllerNode::simpleGoalCallback, this);
    local_alignment_valid_subscriber_ = nh_.subscribe(
        "local_alignment_valid", 5,
        &ControllerNode::localAlignmentValidCallback, this);
    reference_trajectory_path_publisher_ =
        nh_.advertise<nav_msgs::Path>(
            "reference_trajectory_path", 1, true);
    path_status_publisher_ =
        nh_.advertise<xd_uav_controller::PathStatus>(
            "path_status", 10, true);
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
    loadParameterWithLegacy(
        private_nh_, "state_input/timeout", "state_timeout",
        &state_timeout_, 0.20);
    loadParameterWithLegacy(
        private_nh_, "reference_input/timeout", "reference_timeout",
        &reference_timeout_, 0.50);
    loadParameterWithLegacy(
        private_nh_, "reference_input/transform/timeout",
        "reference_frames/transform_timeout",
        &reference_transform_timeout_, 0.03);
    loadParameterWithLegacy(
        private_nh_, "reference_input/transform/max_age",
        "reference_frames/max_transform_age",
        &maximum_transform_age_, 0.50);
    loadParameterWithLegacy(
        private_nh_,
        "reference_input/transform/failure_grace_duration",
        "reference_frames/failure_grace_duration",
        &reference_transform_failure_grace_, 0.50);
    loadParameterWithLegacy(
        private_nh_,
        "reference_input/transform/max_inertial_frame_tilt",
        "reference_frames/max_inertial_frame_tilt",
        &maximum_inertial_frame_tilt_, 0.10);
    loadParameterWithLegacy(
        private_nh_, "reference_input/global_alignment/required",
        "reference_frames/require_global_alignment",
        &require_global_alignment_, true);
    loadParameterWithLegacy(
        private_nh_, "reference_input/global_alignment/timeout",
        "reference_frames/global_alignment_timeout",
        &global_alignment_timeout_, 1.0);
    loadParameterWithLegacy(
        private_nh_, "reference_input/allowed_frames",
        "reference_frames/allowed", &allowed_reference_frames_,
        std::vector<std::string>{"local_origin", "/world", "/map"});
    loadParameterWithLegacy(
        private_nh_, "reference_input/global_alignment/frames",
        "reference_frames/global_alignment_frames",
        &global_alignment_frames_,
        std::vector<std::string>{"local_origin", "/world", "/map"});
    loadParameterWithLegacy(
        private_nh_, "reference_input/simple_goal/use_message_z",
        "simple_goal/use_message_z", &simple_goal_use_message_z_,
        false);
    private_nh_.param(
        "reference_input/path/minimum_segment_length",
        path_minimum_segment_length_, 0.20);
    private_nh_.param(
        "reference_input/path/status_rate", path_status_rate_, 10.0);

    if (vehicle_type_ == "multirotor") {
      loadParameterWithLegacy(
          private_nh_, "multirotor/model/gravity",
          "multirotor/gravity", &gravity_, 9.80665);
      loadParameterWithLegacy(
          private_nh_, "multirotor/model/hover_throttle",
          "multirotor/hover_throttle", &hover_throttle_, 0.5);
      private_nh_.param(
          "multirotor/path_following/nominal_speed",
          multirotor_path_nominal_speed_, 3.0);
      private_nh_.param(
          "multirotor/path_following/lookahead_distance",
          multirotor_path_lookahead_distance_, 2.0);
      private_nh_.param(
          "multirotor/path_following/terminal_slowdown_distance",
          multirotor_path_terminal_slowdown_distance_, 4.0);
      private_nh_.param(
          "multirotor/path_following/completion_position_tolerance",
          multirotor_path_completion_position_tolerance_, 0.5);
      private_nh_.param(
          "multirotor/path_following/completion_speed_tolerance",
          multirotor_path_completion_speed_tolerance_, 0.3);
      private_nh_.param(
          "multirotor/path_following/reacquisition_distance",
          multirotor_path_reacquisition_distance_, 4.0);
      loadParameterWithLegacy(
          private_nh_, "multirotor/limits/minimum_throttle",
          "multirotor/min_throttle", &minimum_throttle_, 0.05);
      loadParameterWithLegacy(
          private_nh_, "multirotor/limits/maximum_throttle",
          "multirotor/max_throttle", &maximum_throttle_, 0.9);
      loadParameterWithLegacy(
          private_nh_, "multirotor/limits/maximum_tilt",
          "multirotor/max_tilt", &maximum_tilt_, 0.61);
      attitude_gain_ = loadVector3WithLegacy(
          private_nh_, "multirotor/attitude_control/gain",
          "multirotor/attitude_gain", {{5.0, 5.0, 2.0}});
      maximum_body_rate_ = loadVector3WithLegacy(
          private_nh_, "multirotor/limits/maximum_body_rate",
          "multirotor/max_body_rate", {{3.0, 3.0, 2.0}});

      int horizon = 26;
      const double dt =
          1.0 / std::max(1.0, control_rate_);
      double r = 20.0;
      loadParameterWithLegacy(
          private_nh_, "multirotor/position_control/mpc/horizon",
          "multirotor/mpc/horizon", &horizon, 26);
      loadParameterWithLegacy(
          private_nh_, "multirotor/position_control/mpc/r",
          "multirotor/mpc/r", &r, 20.0);
      const auto q = loadVector2WithLegacy(
          private_nh_, "multirotor/position_control/mpc/q",
          "multirotor/mpc/q", {{500.0, 100.0}});
      const auto terminal_q = loadVector2WithLegacy(
          private_nh_, "multirotor/position_control/mpc/terminal_q",
          "multirotor/mpc/terminal_q", {{1000.0, 300.0}});
      mpc_.configure(horizon, dt, q, terminal_q, r);
      mpc_dt_ = dt;
      loadParameterWithLegacy(
          private_nh_, "multirotor/limits/maximum_velocity_xy",
          "multirotor/mpc/max_velocity_xy", &max_velocity_xy_, 5.0);
      loadParameterWithLegacy(
          private_nh_, "multirotor/limits/maximum_velocity_z",
          "multirotor/mpc/max_velocity_z", &max_velocity_z_, 3.0);
      loadParameterWithLegacy(
          private_nh_, "multirotor/limits/maximum_acceleration_xy",
          "multirotor/mpc/max_acceleration_xy",
          &max_acceleration_xy_, 5.0);
      loadParameterWithLegacy(
          private_nh_, "multirotor/limits/maximum_acceleration_z",
          "multirotor/mpc/max_acceleration_z",
          &max_acceleration_z_, 4.0);
      loadParameterWithLegacy(
          private_nh_, "multirotor/limits/maximum_jerk_xy",
          "multirotor/mpc/max_jerk_xy", &max_jerk_xy_, 8.0);
      loadParameterWithLegacy(
          private_nh_, "multirotor/limits/maximum_jerk_z",
          "multirotor/mpc/max_jerk_z", &max_jerk_z_, 6.0);
      velocity_integral_gain_ = loadVector3WithLegacy(
          private_nh_,
          "multirotor/position_control/disturbance_rejection/"
          "velocity_integral_gain",
          "multirotor/mpc/velocity_integral_gain",
          {{1.0, 1.0, 2.0}});
      velocity_integral_acceleration_limit_ = loadVector3WithLegacy(
          private_nh_,
          "multirotor/position_control/disturbance_rejection/"
          "velocity_integral_acceleration_limit",
          "multirotor/mpc/velocity_integral_acceleration_limit",
          {{1.0, 1.0, 1.0}});
      position_integral_gain_ = loadVector3WithLegacy(
          private_nh_,
          "multirotor/position_control/disturbance_rejection/"
          "position_integral_gain",
          "multirotor/mpc/position_integral_gain",
          {{0.08, 0.08, 0.15}});
      position_integral_acceleration_limit_ = loadVector3WithLegacy(
          private_nh_,
          "multirotor/position_control/disturbance_rejection/"
          "position_integral_acceleration_limit",
          "multirotor/mpc/position_integral_acceleration_limit",
          {{0.60, 0.60, 0.80}});
      loadParameterWithLegacy(
          private_nh_,
          "multirotor/position_control/disturbance_rejection/"
          "anti_windup_gain",
          "multirotor/mpc/anti_windup_gain",
          &multirotor_anti_windup_gain_, 1.5);
      acceleration_feedback_gain_ = loadVector3WithLegacy(
          private_nh_,
          "multirotor/position_control/disturbance_rejection/"
          "acceleration_feedback_gain",
          "multirotor/mpc/acceleration_feedback_gain",
          {{0.25, 0.25, 0.35}});
      acceleration_integral_gain_ = loadVector3WithLegacy(
          private_nh_,
          "multirotor/position_control/disturbance_rejection/"
          "acceleration_integral_gain",
          "multirotor/mpc/acceleration_integral_gain",
          {{0.20, 0.20, 0.40}});
      acceleration_integral_acceleration_limit_ = loadVector3WithLegacy(
          private_nh_,
          "multirotor/position_control/disturbance_rejection/"
          "acceleration_integral_acceleration_limit",
          "multirotor/mpc/acceleration_integral_acceleration_limit",
          {{1.0, 1.0, 1.0}});
    } else {
      loadParameterWithLegacy(
          private_nh_, "fixedwing/model/gravity",
          "fixedwing/gravity", &gravity_, 9.80665);
      private_nh_.param(
          "fixedwing/path_following/curvature_estimation_distance",
          fixedwing_path_curvature_distance_, 10.0);
      private_nh_.param(
          "fixedwing/path_following/curvature_preview_time",
          fixedwing_path_curvature_preview_time_, 0.30);
      private_nh_.param(
          "fixedwing/path_following/completion_radius",
          fixedwing_path_completion_radius_, 20.0);
      private_nh_.param(
          "fixedwing/path_following/completion_altitude_tolerance",
          fixedwing_path_completion_altitude_tolerance_, 5.0);
      private_nh_.param(
          "fixedwing/path_following/reacquisition_distance",
          fixedwing_path_reacquisition_distance_, 50.0);
      loadParameterWithLegacy(
          private_nh_, "fixedwing/energy_control/cruise_airspeed",
          "fixedwing/cruise_airspeed", &cruise_airspeed_, 15.0);
      loadParameterWithLegacy(
          private_nh_, "fixedwing/energy_control/minimum_airspeed",
          "fixedwing/minimum_airspeed", &minimum_airspeed_, 11.0);
      loadParameterWithLegacy(
          private_nh_, "fixedwing/energy_control/maximum_airspeed",
          "fixedwing/maximum_airspeed", &maximum_airspeed_, 24.0);
      loadParameterWithLegacy(
          private_nh_, "fixedwing/energy_control/trim_throttle",
          "fixedwing/trim_throttle", &trim_throttle_, 0.25);
      loadParameterWithLegacy(
          private_nh_, "fixedwing/limits/minimum_throttle",
          "fixedwing/min_throttle", &minimum_throttle_, 0.05);
      loadParameterWithLegacy(
          private_nh_, "fixedwing/limits/maximum_throttle",
          "fixedwing/max_throttle", &maximum_throttle_, 1.0);
      loadParameterWithLegacy(
          private_nh_,
          "fixedwing/energy_control/airspeed_throttle_gain",
          "fixedwing/airspeed_throttle_gain",
          &airspeed_throttle_gain_, 0.08);
      loadParameterWithLegacy(
          private_nh_,
          "fixedwing/energy_control/climb_rate_throttle_gain",
          "fixedwing/climb_rate_throttle_gain",
          &climb_rate_throttle_gain_, 0.03);
      private_nh_.param(
          "fixedwing/energy_control/turn_load_factor_throttle_gain",
          turn_load_factor_throttle_gain_, 0.16);
      private_nh_.param(
          "fixedwing/energy_control/bank_airspeed_margin",
          bank_airspeed_margin_, 0.5);
      private_nh_.param(
          "fixedwing/energy_control/underspeed_hysteresis",
          underspeed_hysteresis_, 0.7);
      private_nh_.param(
          "fixedwing/energy_control/underspeed_max_roll",
          underspeed_max_roll_, 0.35);
      private_nh_.param(
          "fixedwing/energy_control/underspeed_max_nose_up_pitch",
          underspeed_max_nose_up_pitch_, 0.05);

      loadParameterWithLegacy(
          private_nh_,
          "fixedwing/reference_adapter/streaming_course_rate_feedforward",
          "fixedwing/guidance/streaming_course_rate_feedforward",
          &fixedwing_streaming_course_rate_feedforward_, true);
      loadParameterWithLegacy(
          private_nh_,
          "fixedwing/reference_adapter/course_rate_filter_time_constant",
          "fixedwing/guidance/course_rate_filter_time_constant",
          &fixedwing_course_rate_filter_time_constant_, 0.25);
      loadParameterWithLegacy(
          private_nh_,
          "fixedwing/reference_adapter/minimum_course_rate_speed",
          "fixedwing/guidance/minimum_course_rate_speed",
          &fixedwing_minimum_course_rate_speed_, 2.0);

      loadParameterWithLegacy(
          private_nh_, "fixedwing/path_guidance/lookahead_distance",
          "fixedwing/guidance/lookahead_distance",
          &fixedwing_guidance_lookahead_distance_, 60.0);
      loadParameterWithLegacy(
          private_nh_, "fixedwing/path_guidance/adaptive_lookahead",
          "fixedwing/guidance/adaptive_lookahead",
          &fixedwing_adaptive_lookahead_, true);
      loadParameterWithLegacy(
          private_nh_,
          "fixedwing/path_guidance/minimum_lookahead_distance",
          "fixedwing/guidance/minimum_lookahead_distance",
          &fixedwing_minimum_lookahead_distance_, 30.0);
      loadParameterWithLegacy(
          private_nh_,
          "fixedwing/path_guidance/maximum_lookahead_distance",
          "fixedwing/guidance/maximum_lookahead_distance",
          &fixedwing_maximum_lookahead_distance_, 60.0);
      loadParameterWithLegacy(
          private_nh_, "fixedwing/path_guidance/turn_radius_gain",
          "fixedwing/guidance/turn_radius_gain",
          &fixedwing_lookahead_turn_radius_gain_, 1.2);
      loadParameterWithLegacy(
          private_nh_,
          "fixedwing/path_guidance/lookahead_filter_time_constant",
          "fixedwing/guidance/lookahead_filter_time_constant",
          &fixedwing_lookahead_filter_time_constant_, 0.50);

      loadParameterWithLegacy(
          private_nh_,
          "fixedwing/lateral_control/course_time_constant",
          "fixedwing/course_time_constant",
          &course_time_constant_, 3.0);
      loadParameterWithLegacy(
          private_nh_,
          "fixedwing/lateral_control/course_integral_gain",
          "fixedwing/guidance/course_integral_gain",
          &fixedwing_course_integral_gain_, 0.08);
      loadParameterWithLegacy(
          private_nh_,
          "fixedwing/lateral_control/course_integral_rate_limit",
          "fixedwing/guidance/course_integral_rate_limit",
          &fixedwing_course_integral_rate_limit_, 0.25);
      loadParameterWithLegacy(
          private_nh_,
          "fixedwing/lateral_control/roll_time_constant",
          "fixedwing/roll_time_constant",
          &roll_time_constant_, 0.45);

      loadParameterWithLegacy(
          private_nh_,
          "fixedwing/vertical_control/altitude_time_constant",
          "fixedwing/altitude_time_constant",
          &altitude_time_constant_, 5.0);
      loadParameterWithLegacy(
          private_nh_,
          "fixedwing/vertical_control/altitude_integral_gain",
          "fixedwing/altitude_integral_gain",
          &altitude_integral_gain_, 0.02);
      loadParameterWithLegacy(
          private_nh_,
          "fixedwing/vertical_control/altitude_integral_climb_rate_limit",
          "fixedwing/altitude_integral_climb_rate_limit",
          &altitude_integral_climb_rate_limit_, 1.0);
      loadParameterWithLegacy(
          private_nh_,
          "fixedwing/vertical_control/climb_rate_pitch_gain",
          "fixedwing/climb_rate_pitch_gain",
          &climb_rate_pitch_gain_, 0.06);
      loadParameterWithLegacy(
          private_nh_,
          "fixedwing/vertical_control/climb_rate_filter_time_constant",
          "fixedwing/climb_rate_filter_time_constant",
          &climb_rate_filter_time_constant_, 0.50);
      loadParameterWithLegacy(
          private_nh_,
          "fixedwing/vertical_control/vertical_acceleration_pitch_rate_gain",
          "fixedwing/vertical_acceleration_pitch_rate_gain",
          &vertical_acceleration_pitch_rate_gain_, 1.0);
      loadParameterWithLegacy(
          private_nh_,
          "fixedwing/vertical_control/pitch_time_constant",
          "fixedwing/pitch_time_constant",
          &pitch_time_constant_, 0.45);
      private_nh_.param(
          "fixedwing/vertical_control/bank_load_factor_pitch_gain",
          bank_load_factor_pitch_gain_, 0.20);

      loadParameterWithLegacy(
          private_nh_, "fixedwing/limits/maximum_roll",
          "fixedwing/max_roll", &max_roll_, 0.79);
      loadParameterWithLegacy(
          private_nh_, "fixedwing/limits/maximum_pitch",
          "fixedwing/max_pitch", &max_pitch_, 0.35);
      loadParameterWithLegacy(
          private_nh_, "fixedwing/limits/maximum_climb_rate",
          "fixedwing/max_climb_rate", &max_climb_rate_, 4.0);
      maximum_body_rate_ = loadVector3WithLegacy(
          private_nh_, "fixedwing/limits/maximum_body_rate",
          "fixedwing/max_body_rate", {{2.0, 1.5, 1.5}});

      loadParameterWithLegacy(
          private_nh_, "fixedwing/modes/loiter/radius",
          "fixedwing/loiter/radius", &fixedwing_loiter_radius_, 80.0);
      loadParameterWithLegacy(
          private_nh_, "fixedwing/modes/loiter/airspeed",
          "fixedwing/loiter/airspeed",
          &fixedwing_loiter_airspeed_, 15.0);
      loadParameterWithLegacy(
          private_nh_, "fixedwing/modes/loiter/direction",
          "fixedwing/loiter/direction",
          &fixedwing_loiter_direction_, 1);
      loadParameterWithLegacy(
          private_nh_, "fixedwing/modes/loiter/radial_gain",
          "fixedwing/loiter/radial_gain",
          &fixedwing_loiter_radial_gain_, 1.0);
      loadParameterWithLegacy(
          private_nh_,
          "fixedwing/modes/loiter/max_course_correction",
          "fixedwing/loiter/max_course_correction",
          &fixedwing_loiter_max_course_correction_, 0.70);
      if (fixedwing_loiter_radius_ < 5.0) {
        throw std::runtime_error(
            "fixedwing/modes/loiter/radius必须不小于5米");
      }
      if (fixedwing_guidance_lookahead_distance_ < 5.0) {
        throw std::runtime_error(
            "fixedwing/path_guidance/lookahead_distance"
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
    private_nh_.param("landing/height_source",
                      landing_height_source_,
                      std::string("odom"));
    loadParameterWithLegacy(
        private_nh_, "landing/return_max_velocity",
        "landing/return_velocity", &landing_return_max_velocity_, 1.0);
    private_nh_.param("landing/position_tolerance",
                      landing_position_tolerance_, 0.20);
    private_nh_.param("landing/velocity_tolerance",
                      landing_velocity_tolerance_, 0.20);
    if (vehicle_type_ == "multirotor") {
      loadParameterWithLegacy(
          private_nh_, "landing/distance_sensor_cfg/timeout",
          "landing/distance_sensor_timeout",
          &distance_sensor_landing_config_.timeout, 0.5);
      loadParameterWithLegacy(
          private_nh_,
          "landing/distance_sensor_cfg/velocity_filter_time_constant",
          "landing/distance_sensor_velocity_filter_time_constant",
          &distance_sensor_landing_config_
               .velocity_filter_time_constant,
          0.30);
      loadParameterWithLegacy(
          private_nh_, "landing/distance_sensor_cfg/descent_velocity",
          "landing/descent_velocity",
          &distance_sensor_landing_config_.descent_velocity, 0.35);
      loadParameterWithLegacy(
          private_nh_,
          "landing/distance_sensor_cfg/final_descent_velocity",
          "landing/final_descent_velocity",
          &distance_sensor_landing_config_.final_descent_velocity,
          0.15);
      loadParameterWithLegacy(
          private_nh_, "landing/distance_sensor_cfg/slow_height",
          "landing/slow_height",
          &distance_sensor_landing_config_.slow_height, 0.7);
      loadParameterWithLegacy(
          private_nh_, "landing/distance_sensor_cfg/touchdown_offset",
          "landing/touchdown_offset",
          &distance_sensor_landing_config_.touchdown_offset, 0.15);
      loadParameterWithLegacy(
          private_nh_,
          "landing/distance_sensor_cfg/touchdown_height_tolerance",
          "landing/touchdown_height_tolerance",
          &distance_sensor_landing_config_
               .touchdown_height_tolerance,
          0.10);
      loadParameterWithLegacy(
          private_nh_,
          "landing/distance_sensor_cfg/touchdown_velocity_tolerance",
          "landing/touchdown_velocity_tolerance",
          &distance_sensor_landing_config_
               .touchdown_velocity_tolerance,
          0.20);

      loadParameterWithLegacy(
          private_nh_, "landing/odom_cfg/descent_velocity",
          "landing/descent_velocity",
          &odom_landing_config_.descent_velocity, 0.35);
      loadParameterWithLegacy(
          private_nh_, "landing/odom_cfg/final_descent_velocity",
          "landing/final_descent_velocity",
          &odom_landing_config_.final_descent_velocity, 0.15);
      loadParameterWithLegacy(
          private_nh_, "landing/odom_cfg/slow_height",
          "landing/slow_height",
          &odom_landing_config_.slow_height, 0.7);
      loadParameterWithLegacy(
          private_nh_, "landing/odom_cfg/touchdown_offset",
          "landing/touchdown_offset",
          &odom_landing_config_.touchdown_offset, 0.15);
      loadParameterWithLegacy(
          private_nh_,
          "landing/odom_cfg/touchdown_height_tolerance",
          "landing/touchdown_height_tolerance",
          &odom_landing_config_.touchdown_height_tolerance, 0.10);
      loadParameterWithLegacy(
          private_nh_,
          "landing/odom_cfg/touchdown_velocity_tolerance",
          "landing/touchdown_velocity_tolerance",
          &odom_landing_config_.touchdown_velocity_tolerance, 0.20);
    } else {
      private_nh_.param("landing/touchdown_offset",
                        landing_touchdown_offset_, 0.15);
    }
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
    if (landing_height_source_ != "odom" &&
        landing_height_source_ != "distance_sensor") {
      throw std::runtime_error(
          "landing/height_source必须是odom或distance_sensor");
    }
    if (vehicle_type_ == "fixedwing" &&
        landing_height_source_ != "odom") {
      throw std::runtime_error(
          "固定翼landing/height_source目前只支持odom");
    }
    const auto frames_valid = [](const std::vector<std::string>& frames) {
      return !frames.empty() &&
             std::all_of(
                 frames.begin(), frames.end(),
                 [](const std::string& frame) {
                   return !frame.empty();
                 });
    };
    if (!std::isfinite(control_rate_) || control_rate_ <= 0.0 ||
        !std::isfinite(state_timeout_) || state_timeout_ <= 0.0 ||
        !std::isfinite(reference_timeout_) ||
        reference_timeout_ <= 0.0 ||
        !std::isfinite(reference_transform_timeout_) ||
        reference_transform_timeout_ < 0.0 ||
        !std::isfinite(maximum_transform_age_) ||
        maximum_transform_age_ <= 0.0 ||
        !std::isfinite(reference_transform_failure_grace_) ||
        reference_transform_failure_grace_ < 0.0 ||
        !std::isfinite(maximum_inertial_frame_tilt_) ||
        maximum_inertial_frame_tilt_ < 0.0 ||
        maximum_inertial_frame_tilt_ >= 0.5 * kPi ||
        !std::isfinite(global_alignment_timeout_) ||
        global_alignment_timeout_ <= 0.0 ||
        !frames_valid(allowed_reference_frames_) ||
        (require_global_alignment_ &&
         !frames_valid(global_alignment_frames_))) {
      throw std::runtime_error(
          "公共状态输入或参考输入参数不在有效范围内");
    }
    if (home_frame_config_.empty() ||
        !fixed_home_position_.allFinite() ||
        !std::isfinite(fixed_home_yaw_)) {
      throw std::runtime_error("公共home配置包含空frame或非法数值");
    }
    if (vehicle_type_ == "multirotor") {
      const auto vertical_landing_config_valid = [](
          const MultirotorVerticalLandingConfig& config) {
        return std::isfinite(config.descent_velocity) &&
               config.descent_velocity > 0.0 &&
               std::isfinite(config.final_descent_velocity) &&
               config.final_descent_velocity > 0.0 &&
               std::isfinite(config.slow_height) &&
               config.slow_height > 0.0 &&
               std::isfinite(config.touchdown_offset) &&
               config.touchdown_offset >= 0.0 &&
               std::isfinite(config.touchdown_height_tolerance) &&
               config.touchdown_height_tolerance >= 0.0 &&
               std::isfinite(config.touchdown_velocity_tolerance) &&
               config.touchdown_velocity_tolerance >= 0.0;
      };
      const auto valid_nonnegative_vector = [](
          const std::array<double, 3>& values) {
        return std::all_of(
            values.begin(), values.end(),
            [](const double value) {
              return std::isfinite(value) && value >= 0.0;
            });
      };
      if (!vertical_landing_config_valid(odom_landing_config_) ||
          !vertical_landing_config_valid(
              distance_sensor_landing_config_) ||
          !std::isfinite(distance_sensor_landing_config_.timeout) ||
          distance_sensor_landing_config_.timeout <= 0.0 ||
          !std::isfinite(
              distance_sensor_landing_config_
                  .velocity_filter_time_constant) ||
          distance_sensor_landing_config_
                  .velocity_filter_time_constant <= 0.0 ||
          !std::isfinite(landing_return_max_velocity_) ||
          landing_return_max_velocity_ <= 0.0 ||
          !std::isfinite(landing_position_tolerance_) ||
          landing_position_tolerance_ < 0.0 ||
          !std::isfinite(landing_velocity_tolerance_) ||
          landing_velocity_tolerance_ < 0.0 ||
          !std::isfinite(control_rate_) || control_rate_ <= 0.0 ||
          !std::isfinite(hover_throttle_) || hover_throttle_ <= 0.0 ||
          hover_throttle_ > 1.0 ||
          !std::isfinite(minimum_throttle_) ||
          !std::isfinite(maximum_throttle_) ||
          minimum_throttle_ < 0.0 ||
          maximum_throttle_ <= minimum_throttle_ ||
          maximum_throttle_ > 1.0 ||
          !std::isfinite(maximum_tilt_) || maximum_tilt_ <= 0.0 ||
          maximum_tilt_ >= 0.5 * kPi ||
          !std::isfinite(max_velocity_xy_) || max_velocity_xy_ <= 0.0 ||
          !std::isfinite(max_velocity_z_) || max_velocity_z_ <= 0.0 ||
          !std::isfinite(max_acceleration_xy_) ||
          max_acceleration_xy_ <= 0.0 ||
          !std::isfinite(max_acceleration_z_) ||
          max_acceleration_z_ <= 0.0 ||
          !std::isfinite(max_jerk_xy_) || max_jerk_xy_ <= 0.0 ||
          !std::isfinite(max_jerk_z_) || max_jerk_z_ <= 0.0 ||
          !std::isfinite(multirotor_anti_windup_gain_) ||
          multirotor_anti_windup_gain_ < 0.0 ||
          !valid_nonnegative_vector(attitude_gain_) ||
          !valid_nonnegative_vector(maximum_body_rate_) ||
          !valid_nonnegative_vector(position_integral_gain_) ||
          !valid_nonnegative_vector(
              position_integral_acceleration_limit_) ||
          !valid_nonnegative_vector(velocity_integral_gain_) ||
          !valid_nonnegative_vector(
              velocity_integral_acceleration_limit_) ||
          !valid_nonnegative_vector(acceleration_feedback_gain_) ||
          !valid_nonnegative_vector(acceleration_integral_gain_) ||
          !valid_nonnegative_vector(
              acceleration_integral_acceleration_limit_)) {
        throw std::runtime_error(
            "multirotor控制、限制或抗积分饱和参数不在安全范围内");
      }
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
        (fixedwing_path_curvature_distance_ <= 0.0 ||
         fixedwing_path_curvature_preview_time_ < 0.0 ||
         fixedwing_guidance_lookahead_distance_ <= 0.0 ||
         fixedwing_minimum_lookahead_distance_ <= 0.0 ||
         fixedwing_maximum_lookahead_distance_ <
             fixedwing_minimum_lookahead_distance_ ||
         fixedwing_lookahead_turn_radius_gain_ <= 0.0 ||
         fixedwing_lookahead_filter_time_constant_ < 0.0 ||
         fixedwing_course_rate_filter_time_constant_ < 0.0 ||
         fixedwing_minimum_course_rate_speed_ < 0.0 ||
         fixedwing_course_integral_gain_ < 0.0 ||
         fixedwing_course_integral_rate_limit_ < 0.0 ||
         altitude_time_constant_ <= 0.0 ||
         altitude_integral_gain_ < 0.0 ||
         altitude_integral_climb_rate_limit_ < 0.0 ||
         climb_rate_pitch_gain_ < 0.0 ||
         climb_rate_filter_time_constant_ < 0.0 ||
         vertical_acceleration_pitch_rate_gain_ < 0.0 ||
         bank_load_factor_pitch_gain_ < 0.0 ||
         climb_rate_throttle_gain_ < 0.0 ||
         turn_load_factor_throttle_gain_ < 0.0 ||
         bank_airspeed_margin_ < 0.0 ||
         underspeed_hysteresis_ < 0.0 ||
         underspeed_max_roll_ <= 0.0 ||
         underspeed_max_roll_ > max_roll_ ||
         underspeed_max_nose_up_pitch_ < 0.0 ||
         underspeed_max_nose_up_pitch_ > max_pitch_ ||
         minimum_airspeed_ <= 0.0 ||
         maximum_airspeed_ <= minimum_airspeed_ ||
         max_roll_ <= 0.0 || max_roll_ >= 0.5 * kPi ||
         !std::isfinite(fixedwing_path_curvature_distance_) ||
         !std::isfinite(fixedwing_path_curvature_preview_time_) ||
         !std::isfinite(fixedwing_guidance_lookahead_distance_) ||
         !std::isfinite(fixedwing_minimum_lookahead_distance_) ||
         !std::isfinite(fixedwing_maximum_lookahead_distance_) ||
         !std::isfinite(fixedwing_lookahead_turn_radius_gain_) ||
         !std::isfinite(fixedwing_lookahead_filter_time_constant_) ||
         !std::isfinite(
             fixedwing_course_rate_filter_time_constant_) ||
         !std::isfinite(fixedwing_minimum_course_rate_speed_) ||
         !std::isfinite(fixedwing_course_integral_gain_) ||
         !std::isfinite(fixedwing_course_integral_rate_limit_) ||
         !std::isfinite(altitude_time_constant_) ||
         !std::isfinite(altitude_integral_gain_) ||
         !std::isfinite(altitude_integral_climb_rate_limit_) ||
         !std::isfinite(climb_rate_pitch_gain_) ||
         !std::isfinite(climb_rate_filter_time_constant_) ||
         !std::isfinite(vertical_acceleration_pitch_rate_gain_) ||
         !std::isfinite(bank_load_factor_pitch_gain_) ||
         !std::isfinite(climb_rate_throttle_gain_) ||
         !std::isfinite(turn_load_factor_throttle_gain_) ||
         !std::isfinite(bank_airspeed_margin_) ||
         !std::isfinite(underspeed_hysteresis_) ||
         !std::isfinite(underspeed_max_roll_) ||
         !std::isfinite(underspeed_max_nose_up_pitch_))) {
      throw std::runtime_error(
          "固定翼reference_adapter/path_guidance/控制参数"
          "不在安全范围内");
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
    if (!anyAxis(reference.use_position) &&
        !anyAxis(reference.use_velocity)) {
      *reason =
          "固定翼不支持纯加速度PositionTarget；"
          "加速度只能与位置或速度一起作为前馈";
      return false;
    }
    const bool any_horizontal_acceleration =
        reference.use_acceleration[0] ||
        reference.use_acceleration[1];
    if (any_horizontal_acceleration &&
        !(reference.use_acceleration[0] &&
          reference.use_acceleration[1] &&
          reference.use_velocity[0] &&
          reference.use_velocity[1])) {
      *reason =
          "固定翼水平加速度前馈要求同时启用VX、VY、AX、AY，"
          "用于统一计算路径曲率";
      return false;
    }
    if (reference.use_acceleration[2] &&
        !reference.use_position[2] &&
        !reference.use_velocity[2]) {
      *reason =
          "固定翼AZ前馈要求同时启用PZ或VZ";
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
      resetMultirotorControlState();
    }
    if (!have_reference_ &&
        !internal_reference_active_ &&
        !idle_reference_active_ && state_.state_valid) {
      captureIdleReference();
    }
  }

  void distanceSensorCallback(
      const sensor_msgs::Range::ConstPtr& message) {
    distance_sensor_ = *message;
    const double range = static_cast<double>(message->range);
    const double minimum = static_cast<double>(message->min_range);
    const double maximum = static_cast<double>(message->max_range);
    if (!std::isfinite(range) || !std::isfinite(minimum) ||
        !std::isfinite(maximum) || minimum < 0.0 ||
        maximum <= minimum || range < minimum - 1e-3 ||
        range > maximum + 1e-3) {
      return;
    }

    // Range is the slant distance along the body-fixed downward beam.  Use
    // its vertical component as AGL so a small landing attitude does not look
    // like an artificial climb.  If attitude is not available yet, retaining
    // the raw range is safer than manufacturing a correction.
    double height = range;
    Eigen::Matrix3d rotation;
    if (have_state_ && quaternionToMatrix(
                           state_.orientation_odom_body, &rotation)) {
      height *= std::abs(rotation(2, 2));
    }
    if (!std::isfinite(height)) {
      return;
    }

    const ros::Time now = ros::Time::now();
    if (!have_distance_sensor_ ||
        last_distance_sensor_sample_time_.isZero()) {
      filtered_distance_sensor_velocity_ = 0.0;
      distance_sensor_velocity_valid_ = false;
    } else {
      const double dt =
          (now - last_distance_sensor_sample_time_).toSec();
      if (std::isfinite(dt) && dt > 1e-4 &&
          dt <= distance_sensor_landing_config_.timeout) {
        const double raw_velocity =
            (height - distance_sensor_height_) / dt;
        const double alpha = clamp(
            dt / (distance_sensor_landing_config_
                      .velocity_filter_time_constant + dt),
            0.0, 1.0);
        filtered_distance_sensor_velocity_ +=
            alpha * (raw_velocity -
                     filtered_distance_sensor_velocity_);
        distance_sensor_velocity_valid_ =
            std::isfinite(filtered_distance_sensor_velocity_);
      } else {
        filtered_distance_sensor_velocity_ = 0.0;
        distance_sensor_velocity_valid_ = false;
      }
    }
    distance_sensor_height_ = height;
    last_distance_sensor_sample_time_ = now;
    last_distance_sensor_receive_ = now;
    have_distance_sensor_ = true;
  }

  bool distanceSensorHeight(
      const ros::Time& now, double* height,
      std::string* reason = nullptr,
      double* vertical_velocity = nullptr) const {
    const auto reject = [reason](const std::string& value) {
      if (reason != nullptr) {
        *reason = value;
      }
      return false;
    };
    if (!have_distance_sensor_) {
      return reject("尚未收到下视距离传感器数据");
    }
    const double age =
        (now - last_distance_sensor_receive_).toSec();
    if (!std::isfinite(age) || age < 0.0 ||
        age > distance_sensor_landing_config_.timeout) {
      return reject("下视距离传感器数据超时");
    }
    if (!std::isfinite(distance_sensor_height_)) {
      return reject("下视距离传感器高度无效");
    }
    if (vertical_velocity != nullptr) {
      if (!distance_sensor_velocity_valid_) {
        return reject("下视距离变化率尚未建立");
      }
      *vertical_velocity = filtered_distance_sensor_velocity_;
    }
    *height = distance_sensor_height_;
    return true;
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

  void resetMultirotorControlState() {
    acceleration_command_initialized_ = false;
    acceleration_command_state_.setZero();
    position_integral_acceleration_state_.setZero();
    velocity_integral_acceleration_state_.setZero();
    acceleration_feedback_integral_state_.setZero();
    acceleration_feedback_active_.fill(false);
    last_multirotor_control_time_ = ros::Time();
  }

  void resetFixedwingControlState() {
    fixedwing_altitude_integral_climb_rate_ = 0.0;
    filtered_climb_rate_ = 0.0;
    filtered_climb_rate_initialized_ = false;
    fixedwing_course_integral_rate_ = 0.0;
    filtered_fixedwing_setpoint_course_rate_ = 0.0;
    previous_fixedwing_setpoint_course_ = 0.0;
    fixedwing_setpoint_course_rate_initialized_ = false;
    last_fixedwing_setpoint_course_sample_time_ = ros::Time();
    fixedwing_setpoint_course_hold_ = 0.0;
    fixedwing_setpoint_course_hold_initialized_ = false;
    fixedwing_external_setpoint_mode_initialized_ = false;
    fixedwing_external_setpoint_mode_signature_ = 0U;
    fixedwing_underspeed_active_ = false;
    last_fixedwing_control_time_ = ros::Time();
    filtered_fixedwing_lookahead_distance_ =
        fixedwing_guidance_lookahead_distance_;
    fixedwing_lookahead_initialized_ = false;
  }

  static uint16_t fixedwingSetpointModeSignature(
      const Reference& reference) {
    uint16_t signature = 0U;
    for (int axis = 0; axis < 3; ++axis) {
      if (reference.use_position[axis]) {
        signature |= static_cast<uint16_t>(1U << axis);
      }
      if (reference.use_velocity[axis]) {
        signature |= static_cast<uint16_t>(1U << (axis + 3));
      }
      if (reference.use_acceleration[axis]) {
        signature |= static_cast<uint16_t>(1U << (axis + 6));
      }
    }
    if (reference.use_yaw) {
      signature |= static_cast<uint16_t>(1U << 9);
    }
    if (reference.use_yaw_rate) {
      signature |= static_cast<uint16_t>(1U << 10);
    }
    return signature;
  }

  void updateFixedwingExternalSetpointGuidance(
      const Reference& reference,
      const bool starting_new_mode) {
    if (vehicle_type_ != "fixedwing") {
      return;
    }

    const uint16_t signature =
        fixedwingSetpointModeSignature(reference);
    const bool mode_changed =
        starting_new_mode ||
        !fixedwing_external_setpoint_mode_initialized_ ||
        signature != fixedwing_external_setpoint_mode_signature_;
    if (mode_changed) {
      resetFixedwingControlState();
      fixedwing_external_setpoint_mode_initialized_ = true;
      fixedwing_external_setpoint_mode_signature_ = signature;
      fixedwing_setpoint_course_hold_ = currentFixedwingCourse();
      fixedwing_setpoint_course_hold_initialized_ =
          std::isfinite(fixedwing_setpoint_course_hold_);
    }

    const double horizontal_speed =
        std::hypot(reference.velocity.x, reference.velocity.y);
    const bool can_estimate_course_rate =
        fixedwing_streaming_course_rate_feedforward_ &&
        !reference.use_yaw_rate &&
        (reference.use_velocity[0] ||
         reference.use_velocity[1]) &&
        horizontal_speed >= fixedwing_minimum_course_rate_speed_;
    if (!can_estimate_course_rate) {
      filtered_fixedwing_setpoint_course_rate_ = 0.0;
      fixedwing_setpoint_course_rate_initialized_ = false;
      last_fixedwing_setpoint_course_sample_time_ = ros::Time();
      return;
    }

    const ros::Time now = ros::Time::now();
    const double velocity_course = std::atan2(
        reference.velocity.y, reference.velocity.x);
    if (!fixedwing_setpoint_course_rate_initialized_) {
      previous_fixedwing_setpoint_course_ = velocity_course;
      filtered_fixedwing_setpoint_course_rate_ = 0.0;
      last_fixedwing_setpoint_course_sample_time_ = now;
      fixedwing_setpoint_course_rate_initialized_ = true;
      return;
    }

    const double sample_dt =
        (now - last_fixedwing_setpoint_course_sample_time_).toSec();
    if (sample_dt <= 1e-3 || !std::isfinite(sample_dt) ||
        sample_dt > std::max(0.1, reference_timeout_)) {
      previous_fixedwing_setpoint_course_ = velocity_course;
      filtered_fixedwing_setpoint_course_rate_ = 0.0;
      last_fixedwing_setpoint_course_sample_time_ = now;
      return;
    }

    const double airspeed = std::max(
        minimum_airspeed_,
        state_.airspeed_valid ? state_.airspeed
                              : minimum_airspeed_);
    const double maximum_course_rate =
        gravity_ * std::tan(max_roll_) / airspeed;
    const double measured_course_rate = clamp(
        wrapAngle(
            velocity_course - previous_fixedwing_setpoint_course_) /
            sample_dt,
        -maximum_course_rate, maximum_course_rate);
    const double filter_alpha =
        fixedwing_course_rate_filter_time_constant_ <= 1e-6
            ? 1.0
            : sample_dt /
                  (fixedwing_course_rate_filter_time_constant_ +
                   sample_dt);
    filtered_fixedwing_setpoint_course_rate_ +=
        filter_alpha *
        (measured_course_rate -
         filtered_fixedwing_setpoint_course_rate_);
    previous_fixedwing_setpoint_course_ = velocity_course;
    last_fixedwing_setpoint_course_sample_time_ = now;
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

    const bool starting_new_mode =
        trajectory_active_ || internal_reference_active_ ||
        idle_reference_active_ || fixedwing_loiter_active_ ||
        !have_reference_;
    updateFixedwingExternalSetpointGuidance(
        normalized, starting_new_mode);

    reference_source_ = source;
    reference_ = normalized;
    path_active_ = false;
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

  void publishPathStatus(
      const uint8_t status, const std::string& detail,
      const bool force = false) {
    const ros::Time now = ros::Time::now();
    if (!force && path_status_rate_ > 0.0 &&
        !last_path_status_publish_.isZero() &&
        (now - last_path_status_publish_).toSec() <
            1.0 / path_status_rate_) {
      return;
    }
    xd_uav_controller::PathStatus message;
    message.header.stamp = now;
    message.header.frame_id = state_.header.frame_id;
    message.path_id = active_path_id_;
    message.state = status;
    message.current_segment =
        static_cast<uint32_t>(path_current_segment_);
    message.progress = path_total_length_ > 1e-6
                           ? static_cast<float>(
                                 path_progress_ / path_total_length_)
                           : 0.0F;
    message.cross_track_error =
        static_cast<float>(path_cross_track_error_);
    message.distance_to_end = static_cast<float>(
        std::max(0.0, path_total_length_ - path_progress_));
    message.detail = detail;
    path_status_publisher_.publish(message);
    last_path_status_publish_ = now;
  }

  void rejectPath(const uint32_t path_id, const std::string& detail) {
    const uint32_t previous_path_id = active_path_id_;
    active_path_id_ = path_id;
    publishPathStatus(
        xd_uav_controller::PathStatus::REJECTED, detail, true);
    active_path_id_ = previous_path_id;
  }

  bool samplePathAt(
      const double requested_distance, Eigen::Vector3d* point,
      Eigen::Vector3d* tangent, std::size_t* segment = nullptr) const {
    if (reference_path_points_.size() < 2 ||
        reference_path_lengths_.size() != reference_path_points_.size()) {
      return false;
    }
    const double distance = clamp(
        requested_distance, 0.0, reference_path_lengths_.back());
    auto upper = std::upper_bound(
        reference_path_lengths_.begin(), reference_path_lengths_.end(),
        distance);
    std::size_t index = upper == reference_path_lengths_.begin()
                            ? 0
                            : static_cast<std::size_t>(
                                  upper - reference_path_lengths_.begin() - 1);
    index = std::min(index, reference_path_points_.size() - 2);
    const Eigen::Vector3d delta =
        reference_path_points_[index + 1] - reference_path_points_[index];
    const double length = std::max(1e-9, delta.norm());
    const double alpha = clamp(
        (distance - reference_path_lengths_[index]) / length, 0.0, 1.0);
    *point = reference_path_points_[index] + alpha * delta;
    *tangent = delta / length;
    if (segment != nullptr) {
      *segment = index;
    }
    return true;
  }

  bool estimatePathCurvature(
      const double distance, const double window,
      double* curvature) const {
    if (window <= 1e-3 || curvature == nullptr) {
      return false;
    }
    const double before_distance =
        std::max(0.0, distance - 0.5 * window);
    const double after_distance =
        std::min(path_total_length_, distance + 0.5 * window);
    if (after_distance - before_distance <= 1e-3) {
      return false;
    }
    Eigen::Vector3d before_point;
    Eigen::Vector3d before_tangent;
    Eigen::Vector3d after_point;
    Eigen::Vector3d after_tangent;
    if (!samplePathAt(
            before_distance, &before_point, &before_tangent) ||
        !samplePathAt(
            after_distance, &after_point, &after_tangent)) {
      return false;
    }
    const double before_horizontal_norm =
        std::hypot(before_tangent.x(), before_tangent.y());
    const double after_horizontal_norm =
        std::hypot(after_tangent.x(), after_tangent.y());
    if (before_horizontal_norm < 1e-6 ||
        after_horizontal_norm < 1e-6) {
      return false;
    }
    const double heading_change = wrapAngle(
        std::atan2(after_tangent.y(), after_tangent.x()) -
        std::atan2(before_tangent.y(), before_tangent.x()));
    *curvature =
        heading_change / (after_distance - before_distance);
    return std::isfinite(*curvature);
  }

  bool projectOntoPath(const Eigen::Vector3d& position) {
    if (reference_path_points_.size() < 2) {
      return false;
    }
    std::size_t segment = std::min(
        path_current_segment_, reference_path_points_.size() - 2);
    double projection_ratio = 0.0;
    double projection_distance_squared = 0.0;
    double projection_progress = path_progress_;
    const auto project_segment = [&](const std::size_t index) {
      const Eigen::Vector3d delta =
          reference_path_points_[index + 1] - reference_path_points_[index];
      const double length_squared = delta.squaredNorm();
      if (length_squared < 1e-9) {
        return false;
      }
      projection_ratio = clamp(
          (position - reference_path_points_[index]).dot(delta) /
              length_squared,
          0.0, 1.0);
      const Eigen::Vector3d projection =
          reference_path_points_[index] + projection_ratio * delta;
      projection_distance_squared =
          (position - projection).squaredNorm();
      projection_progress = reference_path_lengths_[index] +
          projection_ratio * std::sqrt(length_squared);
      return true;
    };
    if (!project_segment(segment)) {
      return false;
    }

    // Path is an ordered geometric contract. Advance only after the aircraft
    // reaches or passes the active segment's terminal plane. This is independent
    // of path shape and avoids ambiguous nearest-point jumps at intersections,
    // parallel branches and routes that return close to an earlier point.
    constexpr double kSegmentEndRatio = 1.0 - 1e-6;
    while (projection_ratio >= kSegmentEndRatio &&
           segment + 2 < reference_path_points_.size()) {
      ++segment;
      if (!project_segment(segment)) {
        return false;
      }
    }
    path_progress_ = path_progress_initialized_
                         ? std::max(path_progress_, projection_progress)
                         : projection_progress;
    path_progress_initialized_ = true;
    path_current_segment_ = segment;
    path_cross_track_error_ =
        std::sqrt(projection_distance_squared);
    return true;
  }

  bool refreshPathTransform(const ros::Time& now, std::string* reason) {
    tf2::Transform control_source;
    std::string transform_reason;
    if (!lookupReferenceTransform(
            state_.header.frame_id, reference_path_frame_, 0.0,
            &control_source, &transform_reason)) {
      if (active_reference_transform_failure_since_.isZero()) {
        active_reference_transform_failure_since_ = now;
      }
      if (!reference_path_points_.empty() &&
          (now - active_reference_transform_failure_since_).toSec() <=
              reference_transform_failure_grace_) {
        ROS_WARN_THROTTLE(
            1.0,
            "[xd_uav_controller] 路径TF短暂失效，保持最后一次有效路径: %s",
            transform_reason.c_str());
        return true;
      }
      *reason = "活动路径坐标系持续失效: " + transform_reason;
      return false;
    }
    active_reference_transform_failure_since_ = ros::Time();
    reference_path_points_.clear();
    reference_path_points_.reserve(reference_path_source_points_.size());
    for (const auto& source_point : reference_path_source_points_) {
      const tf2::Vector3 source(
          source_point.x(), source_point.y(), source_point.z());
      const tf2::Vector3 transformed = control_source * source;
      reference_path_points_.emplace_back(
          transformed.x(), transformed.y(), transformed.z());
    }
    return true;
  }

  bool failActivePath(
      const std::string& reason, Reference* reference) {
    publishPathStatus(
        xd_uav_controller::PathStatus::FAILED, reason, true);
    path_active_ = false;
    if (vehicle_type_ == "fixedwing") {
      startFixedwingLoiter(
          state_.position_odom.z, "路径跟随失败");
      *reference = makeFixedwingLoiterReference();
    } else {
      have_reference_ = false;
      have_normalized_reference_ = false;
      point_reference_latched_ = false;
      resetMultirotorControlState();
      captureIdleReference();
      *reference = makeIdleReference();
    }
    ROS_ERROR("[xd_uav_controller] %s", reason.c_str());
    return true;
  }

  bool resolvePathReference(
      const ros::Time& now, Reference* reference,
      std::string* reason) {
    if (!refreshPathTransform(now, reason)) {
      return failActivePath(*reason, reference);
    }
    const Eigen::Vector3d position(
        state_.position_odom.x, state_.position_odom.y,
        state_.position_odom.z);
    if (!projectOntoPath(position)) {
      *reason = "无法把飞机位置投影到活动路径";
      return failActivePath(*reason, reference);
    }

    const Eigen::Vector3d endpoint = reference_path_points_.back();
    const Eigen::Vector3d endpoint_error = endpoint - position;
    const double horizontal_endpoint_error =
        std::hypot(endpoint_error.x(), endpoint_error.y());
    const double speed = std::sqrt(
        state_.velocity_odom.x * state_.velocity_odom.x +
        state_.velocity_odom.y * state_.velocity_odom.y +
        state_.velocity_odom.z * state_.velocity_odom.z);
    const double completion_progress_tolerance =
        vehicle_type_ == "multirotor"
            ? std::max(multirotor_path_lookahead_distance_,
                       multirotor_path_completion_position_tolerance_)
            : fixedwing_path_completion_radius_;
    const bool near_path_end =
        path_progress_ >=
        std::max(0.0, path_total_length_ - completion_progress_tolerance);
    const bool completed = vehicle_type_ == "multirotor"
                               ? near_path_end && endpoint_error.norm() <=
                                         multirotor_path_completion_position_tolerance_ &&
                                     speed <= multirotor_path_completion_speed_tolerance_
                               : near_path_end && horizontal_endpoint_error <=
                                         fixedwing_path_completion_radius_ &&
                                     std::abs(endpoint_error.z()) <=
                                         fixedwing_path_completion_altitude_tolerance_;

    if (completed) {
      path_progress_ = path_total_length_;
      publishPathStatus(
          xd_uav_controller::PathStatus::COMPLETED,
          "路径已按实际位置完成", true);
      path_active_ = false;
      if (vehicle_type_ == "fixedwing") {
        startFixedwingLoiter(endpoint.z(), "路径执行完成");
        *reference = makeFixedwingLoiterReference();
        return true;
      }
    }

    Eigen::Vector3d path_point;
    Eigen::Vector3d tangent;
    double reference_distance = path_progress_;
    if (vehicle_type_ == "multirotor") {
      reference_distance = std::min(
          path_total_length_,
          path_progress_ + multirotor_path_lookahead_distance_);
    }
    if (!samplePathAt(reference_distance, &path_point, &tangent)) {
      *reason = "无法在活动路径上采样参考";
      return failActivePath(*reason, reference);
    }

    *reference = Reference();
    reference->header.stamp = now;
    reference->header.frame_id = state_.header.frame_id;
    reference->path_reference = true;
    reference->position.x = path_point.x();
    reference->position.y = path_point.y();
    reference->position.z = path_point.z();
    reference->use_position = {{true, true, true}};

    double desired_speed = vehicle_type_ == "multirotor"
                               ? std::min(multirotor_path_nominal_speed_,
                                          max_velocity_xy_)
                               : cruise_airspeed_;
    const double remaining = std::max(
        0.0, path_total_length_ - path_progress_);
    if (vehicle_type_ == "multirotor") {
      desired_speed *= clamp(
          remaining /
              std::max(0.1, multirotor_path_terminal_slowdown_distance_),
          0.0, 1.0);
    }
    Eigen::Vector3d velocity = tangent * desired_speed;
    if (vehicle_type_ == "fixedwing") {
      const double horizontal_norm =
          std::hypot(tangent.x(), tangent.y());
      if (horizontal_norm < 1e-6) {
        *reason = "固定翼路径包含近似垂直的航段";
        return failActivePath(*reason, reference);
      }
      velocity.x() = desired_speed * tangent.x() / horizontal_norm;
      velocity.y() = desired_speed * tangent.y() / horizontal_norm;
      velocity.z() = clamp(
          desired_speed * tangent.z() / horizontal_norm,
          -max_climb_rate_, max_climb_rate_);
    }
    reference->velocity.x = velocity.x();
    reference->velocity.y = velocity.y();
    reference->velocity.z = velocity.z();
    reference->use_velocity = {{true, true, true}};
    reference->yaw = std::atan2(velocity.y(), velocity.x());
    reference->use_yaw = true;

    const double curvature_window =
        vehicle_type_ == "fixedwing"
            ? fixedwing_path_curvature_distance_
            : std::max(1.0, multirotor_path_lookahead_distance_);
    double curvature_distance = path_progress_;
    if (vehicle_type_ == "fixedwing") {
      // Keep lateral position guidance attached to the local path tangent.
      // Only curvature is previewed by the short distance needed to establish
      // bank. Using the full geometric lookahead as the path sample makes the
      // aircraft aim directly at the next bend and cut away from the current
      // straight tens of metres too early.
      const double horizontal_groundspeed = std::hypot(
          state_.velocity_odom.x, state_.velocity_odom.y);
      curvature_distance = std::min(
          path_total_length_,
          path_progress_ +
              horizontal_groundspeed *
                  fixedwing_path_curvature_preview_time_);
    }
    double curvature = 0.0;
    if (estimatePathCurvature(
            curvature_distance, curvature_window, &curvature)) {
      reference->yaw_rate =
          desired_speed * curvature;
      reference->use_yaw_rate = true;
      reference->acceleration.x =
          -reference->yaw_rate * velocity.y();
      reference->acceleration.y =
          reference->yaw_rate * velocity.x();
      reference->acceleration.z = 0.0;
      reference->use_acceleration = {{true, true, true}};
    }

    if (completed && vehicle_type_ == "multirotor") {
      reference->position.x = endpoint.x();
      reference->position.y = endpoint.y();
      reference->position.z = endpoint.z();
      reference->velocity = geometry_msgs::Vector3();
      reference->acceleration = geometry_msgs::Vector3();
      reference_source_ = *reference;
      reference_ = *reference;
      point_reference_latched_ = true;
      return true;
    }

    const double reacquisition_distance =
        vehicle_type_ == "multirotor"
            ? multirotor_path_reacquisition_distance_
            : fixedwing_path_reacquisition_distance_;
    publishPathStatus(
        path_cross_track_error_ > reacquisition_distance
            ? xd_uav_controller::PathStatus::REACQUIRING
            : xd_uav_controller::PathStatus::ACTIVE,
        path_cross_track_error_ > reacquisition_distance
            ? "正在重新捕获路径"
            : "路径跟随中");
    return true;
  }

  void referencePathCallback(const nav_msgs::Path::ConstPtr& message) {
    const uint32_t requested_id = message->header.seq != 0U
                                      ? message->header.seq
                                      : ++path_sequence_counter_;
    if (!externalReferenceAllowed() || !have_state_ ||
        !state_.state_valid) {
      rejectPath(requested_id, "当前状态不允许接受外部路径");
      return;
    }
    if (message->header.frame_id.empty() || message->poses.size() < 2) {
      rejectPath(requested_id, "Path必须包含frame_id和至少两个点");
      return;
    }
    tf2::Transform control_source;
    std::string reason;
    if (!lookupReferenceTransform(
            state_.header.frame_id, message->header.frame_id,
            reference_transform_timeout_, &control_source, &reason)) {
      rejectPath(requested_id, reason);
      return;
    }
    std::vector<Eigen::Vector3d> source_points;
    std::vector<Eigen::Vector3d> points;
    for (const auto& pose : message->poses) {
      if ((!pose.header.frame_id.empty() &&
           canonicalFrame(pose.header.frame_id) !=
               canonicalFrame(message->header.frame_id)) ||
          !finite(pose.pose.position)) {
        rejectPath(requested_id, "Path中的点坐标或frame_id无效");
        return;
      }
      const tf2::Vector3 source(
          pose.pose.position.x, pose.pose.position.y,
          pose.pose.position.z);
      const tf2::Vector3 transformed = control_source * source;
      const Eigen::Vector3d point(
          transformed.x(), transformed.y(), transformed.z());
      if (points.empty() ||
          (point - points.back()).norm() >= path_minimum_segment_length_) {
        source_points.emplace_back(source.x(), source.y(), source.z());
        points.push_back(point);
      }
    }
    if (points.size() < 2) {
      rejectPath(requested_id, "去除重复点后Path不足两个点");
      return;
    }
    reference_path_source_points_ = std::move(source_points);
    reference_path_points_ = std::move(points);
    reference_path_frame_ = canonicalFrame(message->header.frame_id);
    reference_path_lengths_.assign(reference_path_points_.size(), 0.0);
    for (std::size_t index = 1; index < reference_path_points_.size(); ++index) {
      reference_path_lengths_[index] = reference_path_lengths_[index - 1] +
          (reference_path_points_[index] -
           reference_path_points_[index - 1]).norm();
    }
    path_total_length_ = reference_path_lengths_.back();
    path_progress_ = 0.0;
    path_cross_track_error_ = 0.0;
    path_current_segment_ = 0;
    path_progress_initialized_ = false;
    active_path_id_ = requested_id;
    path_active_ = true;
    trajectory_active_ = false;
    point_reference_latched_ = false;
    if (vehicle_type_ == "fixedwing") {
      resetFixedwingControlState();
    } else {
      resetMultirotorControlState();
    }
    activateExternalReference();
    publishPathStatus(
        xd_uav_controller::PathStatus::ACCEPTED,
        "路径已接受", true);
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
    path_active_ = false;
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
      case xd_uav_controller::InternalCommand::Request::CANCEL_LANDING: {
        std_srvs::Trigger::Request cancel_request;
        std_srvs::Trigger::Response cancel_response;
        cancelLandingCallback(cancel_request, cancel_response);
        response.success = cancel_response.success;
        response.message = cancel_response.message;
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
      resetMultirotorControlState();
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
    double initial_distance_sensor_height = 0.0;
    if (vehicle_type_ == "multirotor" &&
        landing_height_source_ == "distance_sensor") {
      std::string reason;
      if (!distanceSensorHeight(
              ros::Time::now(), &initial_distance_sensor_height,
              &reason)) {
        response.success = false;
        response.message =
            "下视高度不可用，拒绝自动降落: " + reason;
        return true;
      }
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
    if (vehicle_type_ == "multirotor") {
      landing_target_z_ =
          landing_ground_z_ -
          std::abs(odom_landing_config_.touchdown_offset);
      landing_agl_setpoint_ = initial_distance_sensor_height;
      landing_agl_setpoint_initialized_ =
          landing_height_source_ == "distance_sensor";
      landing_return_velocity_command_.setZero();
    } else {
      landing_target_z_ =
          landing_ground_z_ -
          std::abs(landing_touchdown_offset_);
    }
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
      fixedwing_landing_guidance_course_ = landing_course_;
      fixedwing_landing_guidance_course_initialized_ = false;
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

  bool cancelLandingCallback(
      std_srvs::Trigger::Request&,
      std_srvs::Trigger::Response& response) {
    if (!landing_active_) {
      response.success = true;
      response.message = "当前没有正在执行的降落";
      return true;
    }
    if (!have_state_ || !state_.state_valid ||
        !state_.odometry_fresh || !state_.imu_fresh) {
      response.success = false;
      response.message =
          "当前控制状态无效，拒绝取消降落并恢复飞行推力";
      return true;
    }
    if (landing_touchdown_) {
      response.success = false;
      response.message = "已经检测到触地，不能取消降落";
      return true;
    }

    landing_active_ = false;
    landing_touchdown_ = false;
    landing_return_home_ = false;
    landing_phase_ = LandingPhase::kNone;
    landing_agl_setpoint_initialized_ = false;
    landing_return_velocity_command_.setZero();
    fixedwing_landing_phase_ =
        FixedwingLandingPhase::kNone;
    fixedwing_landing_guidance_course_ = 0.0;
    fixedwing_landing_guidance_course_initialized_ = false;
    fixedwing_landing_throttle_scale_ = 1.0;
    last_landing_update_ = ros::Time();
    takeoff_active_ = false;

    // A cancelled landing must not resume a stale point or trajectory.
    // Capture a fresh, continuous safe reference at the aircraft's current
    // state; a later external command can replace it normally.
    have_reference_ = false;
    have_reference_error_ = false;
    have_normalized_reference_ = false;
    path_active_ = false;
    trajectory_active_ = false;
    point_reference_latched_ = false;
    active_reference_transform_failure_since_ = ros::Time();

    if (vehicle_type_ == "fixedwing") {
      startFixedwingLoiter(
          state_.position_odom.z, "用户取消降落");
      response.message =
          "已取消固定翼降落，将从当前位置平滑进入等待盘旋";
    } else {
      internal_reference_active_ = false;
      fixedwing_loiter_active_ = false;
      idle_reference_active_ = false;
      resetMultirotorControlState();
      captureIdleReference();
      response.message =
          "已取消多旋翼降落，将在当前位置悬停";
    }
    response.success = true;
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
    landing_agl_setpoint_initialized_ = false;
    landing_return_velocity_command_.setZero();
    fixedwing_landing_phase_ =
        FixedwingLandingPhase::kNone;
    fixedwing_landing_guidance_course_ = 0.0;
    fixedwing_landing_guidance_course_initialized_ = false;
    internal_reference_active_ = false;
    idle_reference_active_ = false;
    have_reference_ = false;
    have_reference_error_ = false;
    have_normalized_reference_ = false;
    path_active_ = false;
    trajectory_active_ = false;
    point_reference_latched_ = false;
    have_takeoff_origin_ = false;
    active_reference_transform_failure_since_ = ros::Time();
    resetMultirotorControlState();
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
    path_active_ = false;
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
    double guidance_dt =
        1.0 / std::max(1.0, control_rate_);
    if (!last_landing_update_.isZero()) {
      const double measured_dt =
          (now - last_landing_update_).toSec();
      if (measured_dt > 0.0 && std::isfinite(measured_dt)) {
        guidance_dt = clamp(measured_dt, 1e-3, 0.10);
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
    const Eigen::Vector2d to_approach =
        approach_position - current_position;
    const double approach_course =
        to_approach.norm() > 1.0
            ? std::atan2(to_approach.y(), to_approach.x())
            : landing_course_;
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
    const double course_correction = clamp(
        std::atan2(
            cross_track_error,
            fixedwing_landing_line_lookahead_distance_),
        -fixedwing_landing_max_course_correction_,
        fixedwing_landing_max_course_correction_);
    const double path_course = wrapAngle(
        landing_course_ - course_correction);
    const double course_error = std::abs(wrapAngle(
        currentFixedwingCourse() - landing_course_));

    if (fixedwing_landing_phase_ ==
            FixedwingLandingPhase::kApproach &&
        distance_to_approach <= approach_capture_radius) {
      // Reconstruct a course target that produces the aircraft's current
      // bank. This preserves the established turn even if the bearing to
      // the static approach point has just flipped after passing it.
      double current_roll = 0.0;
      Eigen::Matrix3d landing_rotation;
      if (quaternionToMatrix(
              state_.orientation_odom_body,
              &landing_rotation)) {
        current_roll = std::atan2(
            landing_rotation(2, 1),
            landing_rotation(2, 2));
      }
      current_roll = clamp(
          current_roll,
          -fixedwing_landing_max_roll_,
          fixedwing_landing_max_roll_);
      const double bank_continuous_course_rate =
          -gravity_ * std::tan(current_roll) /
          std::max(
              minimum_airspeed_,
              fixedwing_landing_approach_airspeed_);
      fixedwing_landing_phase_ =
          FixedwingLandingPhase::kLineCapture;
      fixedwing_landing_guidance_course_ = wrapAngle(
          currentFixedwingCourse() +
          bank_continuous_course_rate *
              std::max(0.1, course_time_constant_));
      fixedwing_landing_guidance_course_initialized_ = true;
      ROS_INFO(
          "[xd_uav_controller] 固定翼进入进近直线捕获，"
          "保持进近高度(capture_radius=%.1fm)",
          approach_capture_radius);
    } else if (fixedwing_landing_phase_ ==
               FixedwingLandingPhase::kLineCapture &&
        std::abs(state_.position_odom.z -
                 fixedwing_landing_approach_point_.z()) <=
            fixedwing_landing_approach_altitude_tolerance_ &&
        course_error <=
            fixedwing_landing_approach_course_tolerance_ &&
        std::abs(wrapAngle(
            path_course -
            fixedwing_landing_guidance_course_)) <=
            fixedwing_landing_approach_course_tolerance_) {
      fixedwing_landing_phase_ =
          FixedwingLandingPhase::kGlideSlope;
      ROS_INFO(
          "[xd_uav_controller] 固定翼已对准进近航线，"
          "开始下滑(capture_radius=%.1fm)",
          approach_capture_radius);
    }
    if (fixedwing_landing_phase_ !=
        FixedwingLandingPhase::kApproach) {
      if (!fixedwing_landing_guidance_course_initialized_) {
        fixedwing_landing_guidance_course_ =
            currentFixedwingCourse();
        fixedwing_landing_guidance_course_initialized_ = true;
      }
      const double maximum_landing_course_rate =
          gravity_ * std::tan(fixedwing_landing_max_roll_) /
          std::max(
              minimum_airspeed_,
              fixedwing_landing_approach_airspeed_);
      const double course_step = clamp(
          wrapAngle(
              path_course -
              fixedwing_landing_guidance_course_),
          -maximum_landing_course_rate * guidance_dt,
          maximum_landing_course_rate * guidance_dt);
      fixedwing_landing_guidance_course_ = wrapAngle(
          fixedwing_landing_guidance_course_ + course_step);
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
               FixedwingLandingPhase::kLineCapture) {
      // Horizontal line capture and vertical descent are deliberately
      // separate. Hold approach altitude while the existing point-turn
      // course slews into the final-line course.
      reference.use_position = {{false, false, true}};
      reference.position.z =
          fixedwing_landing_approach_point_.z();
      reference.velocity.x =
          fixedwing_landing_approach_airspeed_ *
          std::cos(fixedwing_landing_guidance_course_);
      reference.velocity.y =
          fixedwing_landing_approach_airspeed_ *
          std::sin(fixedwing_landing_guidance_course_);
      reference.velocity.z = 0.0;
      reference.yaw = fixedwing_landing_guidance_course_;
    } else if (fixedwing_landing_phase_ ==
               FixedwingLandingPhase::kGlideSlope) {
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
      reference.use_position = {{false, false, false}};
      reference.velocity.x =
          fixedwing_landing_approach_airspeed_ *
          std::cos(fixedwing_landing_guidance_course_);
      reference.velocity.y =
          fixedwing_landing_approach_airspeed_ *
          std::sin(fixedwing_landing_guidance_course_);
      reference.velocity.z = clamp(
          path_climb_rate,
          -sink_rate_limit, sink_rate_limit);
      reference.yaw = fixedwing_landing_guidance_course_;
    } else {
      const Eigen::Vector2d rollout_target =
          touchdown_position +
          fixedwing_landing_rollout_distance_ *
              landing_direction;
      reference.use_position = {{true, true, false}};
      reference.position.x = rollout_target.x();
      reference.position.y = rollout_target.y();
      reference.velocity.x =
          fixedwing_landing_approach_airspeed_ *
          std::cos(fixedwing_landing_guidance_course_);
      reference.velocity.y =
          fixedwing_landing_approach_airspeed_ *
          std::sin(fixedwing_landing_guidance_course_);
      const double contact_height =
          fixedwingLandingContactHeight();
      reference.velocity.z =
          height_above_touchdown <= contact_height
              ? 0.0
              : -0.5 * fixedwingLandingNominalSinkRate();
      reference.yaw = fixedwing_landing_guidance_course_;
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
        if (vehicle_type_ == "multirotor") {
          landing_target_z_ =
              landing_ground_z_ -
              std::abs(odom_landing_config_.touchdown_offset);
        } else {
          landing_target_z_ =
              landing_ground_z_ -
              std::abs(landing_touchdown_offset_);
        }
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
      // Horizontal return is velocity-controlled.  A moving position target
      // ahead of the aircraft used to dominate the lower-weight velocity
      // objective in the MPC and kept pulling toward home while the velocity
      // profile was already asking to brake.
      reference.use_position[0] = false;
      reference.use_position[1] = false;
      const Eigen::Vector2d target_xy =
          landing_origin_.head<2>();
      const Eigen::Vector2d current_xy(
          state_.position_odom.x, state_.position_odom.y);
      const Eigen::Vector2d current_velocity_xy(
          state_.velocity_odom.x, state_.velocity_odom.y);
      const Eigen::Vector2d actual_remaining =
          target_xy - current_xy;
      const double actual_distance = actual_remaining.norm();

      // return_max_velocity is a ceiling, not a constant command.  Use only a
      // conservative fraction of the hard acceleration limit for trajectory
      // braking and include control/attitude response distance.  Solving
      //   d = v * response_time + v^2 / (2 * braking_acceleration)
      // for v starts braking well before home instead of assuming the hard
      // acceleration limit is available instantaneously.
      Eigen::Vector2d desired_return_velocity =
          Eigen::Vector2d::Zero();
      if (actual_distance > landing_position_tolerance_) {
        const double braking_distance = std::max(
            0.0, actual_distance - landing_position_tolerance_);
        const double braking_acceleration = std::max(
            0.20, 0.25 * max_acceleration_xy_);
        constexpr double kControlResponseTime = 0.35;
        const double response_velocity =
            braking_acceleration * kControlResponseTime;
        const double stopping_speed =
            -response_velocity + std::sqrt(
                response_velocity * response_velocity +
                2.0 * braking_acceleration * braking_distance);
        const double speed = std::min(
            std::abs(landing_return_max_velocity_), stopping_speed);
        desired_return_velocity =
            speed * actual_remaining / actual_distance;
      }

      const bool reducing_speed =
          desired_return_velocity.norm() + 1e-6 <
              landing_return_velocity_command_.norm() ||
          desired_return_velocity.dot(
              landing_return_velocity_command_) < 0.0;
      if (reducing_speed) {
        // Do not slew-limit braking twice.  The multirotor controller below
        // already enforces acceleration and jerk limits on the realizable
        // command; delaying the velocity reference here caused overshoot.
        landing_return_velocity_command_ =
            desired_return_velocity;
      } else {
        Eigen::Vector2d velocity_delta =
            desired_return_velocity -
            landing_return_velocity_command_;
        const double maximum_velocity_delta =
            max_acceleration_xy_ * dt;
        if (velocity_delta.norm() > maximum_velocity_delta &&
            maximum_velocity_delta > 0.0) {
          velocity_delta *=
              maximum_velocity_delta / velocity_delta.norm();
        }
        landing_return_velocity_command_ += velocity_delta;
      }
      reference.velocity.x =
          landing_return_velocity_command_.x();
      reference.velocity.y =
          landing_return_velocity_command_.y();

      // Feed the measured velocity tracking error forward as acceleration.
      // The inner controller remains the single owner of acceleration/jerk
      // limits, while this term makes braking depend on the aircraft's real
      // closing speed rather than assuming it followed the previous target.
      constexpr double kVelocityResponseTime = 0.35;
      Eigen::Vector2d return_acceleration =
          (landing_return_velocity_command_ - current_velocity_xy) /
          kVelocityResponseTime;
      if (return_acceleration.norm() > max_acceleration_xy_) {
        return_acceleration *=
            max_acceleration_xy_ / return_acceleration.norm();
      }
      reference.acceleration.x = return_acceleration.x();
      reference.acceleration.y = return_acceleration.y();
      landing_setpoint_.head<2>() = current_xy;

      if ((target_xy - current_xy).norm() <=
              landing_position_tolerance_ &&
          current_velocity_xy.norm() <=
              landing_velocity_tolerance_) {
        landing_phase_ = LandingPhase::kDescent;
        landing_setpoint_.head<2>() = target_xy;
        landing_return_velocity_command_.setZero();
        if (landing_height_source_ == "distance_sensor") {
          double height = 0.0;
          if (distanceSensorHeight(now, &height)) {
            landing_agl_setpoint_ = height;
            landing_agl_setpoint_initialized_ = true;
          } else {
            landing_agl_setpoint_initialized_ = false;
          }
        }
      }
    }

    if (landing_phase_ == LandingPhase::kDescent) {
      landing_setpoint_.head<2>() =
          landing_origin_.head<2>();
      if (landing_height_source_ == "distance_sensor") {
        const DistanceSensorLandingConfig& config =
            distance_sensor_landing_config_;
        double height = 0.0;
        std::string reason;
        if (distanceSensorHeight(now, &height, &reason)) {
          if (!landing_agl_setpoint_initialized_) {
            landing_agl_setpoint_ = height;
            landing_agl_setpoint_initialized_ = true;
          }
          const double descent_velocity =
              height <= config.slow_height
                  ? std::abs(config.final_descent_velocity)
                  : std::abs(config.descent_velocity);
          const double touchdown_height =
              std::abs(config.touchdown_offset);
          landing_agl_setpoint_ = std::max(
              touchdown_height,
              landing_agl_setpoint_ - descent_velocity * dt);

          // Form the vertical position error directly in AGL, then translate
          // only that error into the current odom frame.  An odom-Z offset or
          // slow drift therefore cannot change the requested clearance.
          landing_setpoint_.z() =
              state_.position_odom.z +
              (landing_agl_setpoint_ - height);
          reference.velocity.z =
              landing_agl_setpoint_ > touchdown_height + 1e-3
                  ? -descent_velocity
                  : 0.0;
        } else {
          landing_agl_setpoint_initialized_ = false;
          landing_setpoint_.z() = state_.position_odom.z;
          reference.velocity.z = 0.0;
          ROS_ERROR_THROTTLE(
              1.0,
              "[xd_uav_controller] 降落暂停Z下降: %s",
              reason.c_str());
        }
      } else {
        const MultirotorVerticalLandingConfig& config =
            odom_landing_config_;
        const double height =
            std::max(0.0, state_.position_odom.z -
                              landing_ground_z_);
        const double descent_velocity =
            height <= config.slow_height
                ? std::abs(config.final_descent_velocity)
                : std::abs(config.descent_velocity);
        landing_setpoint_.z() = std::max(
            landing_target_z_,
            landing_setpoint_.z() - descent_velocity * dt);
        reference.velocity.z = -descent_velocity;
      }
    }

    if (!landing_touchdown_) {
      // For land_home, kDescent can only be entered after the vehicle has
      // reached home within landing_position_tolerance_.  Do not require the
      // horizontal error to remain small at touchdown: ground contact can
      // prevent the vehicle from correcting a small descent drift, which
      // would otherwise keep hover thrust applied indefinitely and prevent
      // PX4 from reporting LANDED_STATE_ON_GROUND.
      if (landing_height_source_ == "distance_sensor") {
        double height = 0.0;
        double vertical_velocity = 0.0;
        const DistanceSensorLandingConfig& config =
            distance_sensor_landing_config_;
        landing_touchdown_ =
            landing_phase_ == LandingPhase::kDescent &&
            distanceSensorHeight(
                now, &height, nullptr, &vertical_velocity) &&
            height <= std::abs(config.touchdown_offset) +
                          config.touchdown_height_tolerance &&
            std::abs(vertical_velocity) <=
                config.touchdown_velocity_tolerance;
      } else {
        const MultirotorVerticalLandingConfig& config =
            odom_landing_config_;
        landing_touchdown_ =
            landing_phase_ == LandingPhase::kDescent &&
            landing_setpoint_.z() <=
                landing_ground_z_ +
                    config.touchdown_height_tolerance &&
            state_.position_odom.z <=
                landing_ground_z_ +
                    config.touchdown_height_tolerance &&
            std::abs(state_.velocity_odom.z) <=
                config.touchdown_velocity_tolerance;
      }
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

  void updateMultirotorIntegral(
      const double error, const double gain, const double limit,
      const double raw_acceleration,
      const double realizable_acceleration, const double dt,
      double* state) const {
    const double safe_gain = std::max(0.0, gain);
    const double safe_limit = std::max(0.0, limit);
    if (safe_gain <= 0.0 || safe_limit <= 0.0) {
      *state = 0.0;
      return;
    }

    const double integral_delta = safe_gain * error * dt;
    const double saturation_error =
        raw_acceleration - realizable_acceleration;
    const bool saturated = std::abs(saturation_error) > 1e-3;
    const bool pushes_further_into_saturation =
        saturated && integral_delta * saturation_error > 0.0;

    if (!pushes_further_into_saturation) {
      *state = clamp(*state + integral_delta,
                     -safe_limit, safe_limit);
      return;
    }

    // Clamping prevents new wind-up. If an existing integral has the same
    // sign as the unavailable acceleration, unwind only toward zero so a
    // large proportional command cannot create an opposite-sign integral.
    if (*state * saturation_error > 0.0 &&
        multirotor_anti_windup_gain_ > 0.0) {
      const double unwind = std::min(
          std::abs(*state),
          multirotor_anti_windup_gain_ *
              std::abs(saturation_error) * dt);
      *state -= std::copysign(unwind, *state);
    }
  }

  ControllerResult multirotorControl(
      const Reference& reference, const ros::Time& now) {
    ControllerResult result;
    double control_dt = mpc_dt_;
    if (!last_multirotor_control_time_.isZero()) {
      const double measured_dt =
          (now - last_multirotor_control_time_).toSec();
      if (measured_dt > 0.0 && std::isfinite(measured_dt)) {
        control_dt = clamp(measured_dt, 1e-3, 0.05);
      }
    }
    last_multirotor_control_time_ = now;

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
      const bool position_integrator_enabled =
          reference.use_position[axis] &&
          !takeoff_active_ && !landing_active_;
      if (position_integrator_enabled) {
        desired_acceleration(axis) +=
            position_integral_acceleration_state_(axis);
      } else {
        position_integral_acceleration_state_(axis) = 0.0;
      }
      const bool velocity_integrator_enabled =
          reference.use_velocity[axis] &&
          !reference.use_position[axis] &&
          !landing_active_;
      if (velocity_integrator_enabled) {
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
        desired_acceleration(axis) +=
            feedback_gain * acceleration_error +
            acceleration_feedback_integral_state_(axis);
      } else {
        acceleration_feedback_integral_state_(axis) = 0.0;
        acceleration_feedback_active_[axis] = false;
      }
      if (reference.use_jerk) {
        desired_acceleration(axis) +=
            jerk_feedforward(axis) * control_dt;
      }
    }
    const Eigen::Vector3d raw_desired_acceleration =
        desired_acceleration;
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
        max_jerk_xy_ * control_dt;
    if (horizontal_step.norm() > maximum_horizontal_step) {
      horizontal_step *=
          maximum_horizontal_step / horizontal_step.norm();
    }
    acceleration_command_state_.head<2>() += horizontal_step;
    acceleration_command_state_.z() += clamp(
        desired_acceleration.z() -
            acceleration_command_state_.z(),
        -max_jerk_z_ * control_dt,
        max_jerk_z_ * control_dt);
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
    const double raw_thrust =
        hover_throttle_ * acceleration_along_current_z / gravity_;
    result.thrust = clamp(
        raw_thrust,
        minimum_throttle_, maximum_throttle_);

    // Express all hard limits back in acceleration coordinates. This is
    // used only by the integrator update; attitude dynamics remain handled
    // by the SO(3) body-rate loop and PX4 rate controller.
    Eigen::Vector3d realizable_acceleration =
        total_acceleration - Eigen::Vector3d(0.0, 0.0, gravity_);
    realizable_acceleration +=
        rotation.col(2) *
        ((result.thrust - raw_thrust) * gravity_ /
         hover_throttle_);
    for (int axis = 0; axis < 3; ++axis) {
      const bool position_integrator_enabled =
          reference.use_position[axis] &&
          !takeoff_active_ && !landing_active_;
      const bool velocity_integrator_enabled =
          reference.use_velocity[axis] &&
          !reference.use_position[axis];
      const bool pure_acceleration_axis =
          reference.use_acceleration[axis] &&
          !reference.use_position[axis] &&
          !reference.use_velocity[axis];
      if (position_integrator_enabled) {
        updateMultirotorIntegral(
            position_reference(axis) - position(axis),
            position_integral_gain_[axis],
            position_integral_acceleration_limit_[axis],
            raw_desired_acceleration(axis),
            realizable_acceleration(axis), control_dt,
            &position_integral_acceleration_state_(axis));
      }
      if (velocity_integrator_enabled) {
        updateMultirotorIntegral(
            velocity_reference(axis) - velocity(axis),
            velocity_integral_gain_[axis],
            velocity_integral_acceleration_limit_[axis],
            raw_desired_acceleration(axis),
            realizable_acceleration(axis), control_dt,
            &velocity_integral_acceleration_state_(axis));
      }
      if (pure_acceleration_axis) {
        updateMultirotorIntegral(
            acceleration_reference(axis) -
                measured_acceleration(axis),
            acceleration_integral_gain_[axis],
            acceleration_integral_acceleration_limit_[axis],
            raw_desired_acceleration(axis),
            realizable_acceleration(axis), control_dt,
            &acceleration_feedback_integral_state_(axis));
      }
    }
    result.valid = result.body_rate.allFinite() &&
                   std::isfinite(result.thrust);
    if (!result.valid) {
      result.reason = "四旋翼控制输出包含非法数值";
    }
    return result;
  }

  FixedwingControlTarget makeFixedwingControlTarget(
      const Reference& reference,
      const double current_course,
      const double fallback_yaw,
      const double dt) {
    FixedwingControlTarget target;
    target.course = current_course;
    target.airspeed = cruise_airspeed_;

    const bool external_reference =
        !internal_reference_active_ &&
        !idle_reference_active_ &&
        !landing_active_ &&
        !fixedwing_loiter_active_;
    const bool external_setpoint =
        external_reference && !reference.trajectory_reference &&
        !reference.path_reference;
    const bool use_horizontal_position =
        external_reference &&
        (reference.use_position[0] ||
         reference.use_position[1]);
    const bool use_horizontal_velocity =
        reference.use_velocity[0] ||
        reference.use_velocity[1];

    // Yaw and captured-course hold are fallbacks. Position and velocity
    // guidance below take precedence whenever they define a horizontal path.
    if (external_reference && reference.use_yaw) {
      target.course = reference.yaw;
    } else if (external_setpoint &&
               fixedwing_setpoint_course_hold_initialized_) {
      target.course = fixedwing_setpoint_course_hold_;
    }

    bool curvature_rate_valid = false;
    double curvature_rate = 0.0;
    if (reference.use_velocity[0] &&
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

    // MultiDOFJointTrajectory has no mask for angular.z: zero conventionally
    // means "not supplied" there. PositionTarget does have an explicit mask,
    // so an enabled zero yaw_rate remains authoritative for that interface.
    const bool explicit_course_rate =
        reference.use_yaw_rate &&
        (!reference.trajectory_reference ||
         std::abs(reference.yaw_rate) > 1e-6);
    if (explicit_course_rate) {
      target.course_rate_feedforward = reference.yaw_rate;
      if (curvature_rate_valid &&
          std::abs(reference.yaw_rate - curvature_rate) > 0.10) {
        ROS_WARN_THROTTLE(
            1.0,
            "[xd_uav_controller] 固定翼参考yaw_rate"
            "与速度/加速度曲率不一致"
            "(yaw_rate=%.3f curvature=%.3f)，"
            "优先使用显式yaw_rate",
            reference.yaw_rate, curvature_rate);
      }
    } else if (curvature_rate_valid) {
      target.course_rate_feedforward = curvature_rate;
    } else if (external_setpoint && use_horizontal_velocity) {
      target.course_rate_feedforward =
          filtered_fixedwing_setpoint_course_rate_;
    }

    if (use_horizontal_position) {
      const double target_x =
          reference.use_position[0]
              ? reference.position.x
              : state_.position_odom.x;
      const double target_y =
          reference.use_position[1]
              ? reference.position.y
              : state_.position_odom.y;
      const double dx = target_x - state_.position_odom.x;
      const double dy = target_y - state_.position_odom.y;
      if (std::hypot(dx, dy) > 1.0) {
        target.course = std::atan2(dy, dx);
      }
    }

    if (use_horizontal_velocity) {
      const double horizontal_speed =
          std::hypot(reference.velocity.x,
                     reference.velocity.y);
      if (horizontal_speed > 0.5) {
        target.course = std::atan2(
            reference.velocity.y, reference.velocity.x);
        target.airspeed = clamp(
            horizontal_speed, minimum_airspeed_,
            maximum_airspeed_);

        if (use_horizontal_position) {
          double guidance_lookahead_distance =
              fixedwing_guidance_lookahead_distance_;
          if (fixedwing_adaptive_lookahead_) {
            double target_lookahead =
                fixedwing_maximum_lookahead_distance_;
            if (std::abs(target.course_rate_feedforward) > 1e-6) {
              const double turn_radius =
                  horizontal_speed /
                  std::abs(target.course_rate_feedforward);
              target_lookahead = clamp(
                  fixedwing_lookahead_turn_radius_gain_ * turn_radius,
                  fixedwing_minimum_lookahead_distance_,
                  fixedwing_maximum_lookahead_distance_);
            }
            if (!fixedwing_lookahead_initialized_) {
              filtered_fixedwing_lookahead_distance_ =
                  target_lookahead;
              fixedwing_lookahead_initialized_ = true;
            } else {
              const double filter_alpha =
                  fixedwing_lookahead_filter_time_constant_ <= 1e-6
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

          const double inverse_speed = 1.0 / horizontal_speed;
          const double path_x =
              reference.use_position[0]
                  ? reference.position.x
                  : state_.position_odom.x;
          const double path_y =
              reference.use_position[1]
                  ? reference.position.y
                  : state_.position_odom.y;
          // The guidance point stays on the local tangent. For a Path, the
          // anchor is the aircraft projection on the active segment; curvature
          // preview is handled separately as course-rate feed-forward. This
          // separation prevents a far lookahead point on the following bend
          // from pulling the aircraft off the current straight.
          const double guidance_x =
              path_x + guidance_lookahead_distance *
                           reference.velocity.x * inverse_speed;
          const double guidance_y =
              path_y + guidance_lookahead_distance *
                           reference.velocity.y * inverse_speed;
          const double guidance_dx =
              guidance_x - state_.position_odom.x;
          const double guidance_dy =
              guidance_y - state_.position_odom.y;
          if (std::hypot(guidance_dx, guidance_dy) > 1.0) {
            target.course =
                std::atan2(guidance_dy, guidance_dx);
          }
        }
      }
    }

    if (!std::isfinite(target.course)) {
      target.course = fallback_yaw;
    }
    target.enable_course_integrator = external_reference;
    target.use_altitude_feedback = reference.use_position[2];
    if (target.use_altitude_feedback) {
      target.altitude_error =
          reference.position.z - state_.position_odom.z;
    }
    if (reference.use_velocity[2]) {
      target.climb_rate_feedforward = reference.velocity.z;
    }
    if (reference.use_acceleration[2]) {
      target.vertical_acceleration_feedforward =
          reference.acceleration.z;
    }
    target.enable_altitude_integrator =
        target.use_altitude_feedback &&
        !takeoff_active_ && !landing_active_;
    return target;
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

    const FixedwingControlTarget target =
        makeFixedwingControlTarget(
            reference, current_course, yaw, dt);

    const double airspeed =
        std::max(minimum_airspeed_, state_.airspeed);
    const double course_error =
        wrapAngle(target.course - current_course);
    const double maximum_course_rate =
        gravity_ * std::tan(max_roll_) / airspeed;
    const bool course_integrator_enabled =
        target.enable_course_integrator &&
        fixedwing_course_integral_gain_ > 0.0 &&
        fixedwing_course_integral_rate_limit_ > 0.0;
    if (course_integrator_enabled) {
      const double integral_delta =
          fixedwing_course_integral_gain_ * course_error * dt;
      const double candidate = clamp(
          fixedwing_course_integral_rate_ + integral_delta,
          -fixedwing_course_integral_rate_limit_,
          fixedwing_course_integral_rate_limit_);
      const double candidate_course_rate =
          course_error / std::max(0.1, course_time_constant_) +
          target.course_rate_feedforward + candidate;
      const bool winds_up_high =
          candidate_course_rate > maximum_course_rate &&
          integral_delta > 0.0;
      const bool winds_up_low =
          candidate_course_rate < -maximum_course_rate &&
          integral_delta < 0.0;
      if (!winds_up_high && !winds_up_low) {
        fixedwing_course_integral_rate_ = candidate;
      }
    } else {
      fixedwing_course_integral_rate_ = 0.0;
    }
    const double desired_course_rate = clamp(
        course_error / std::max(0.1, course_time_constant_) +
            target.course_rate_feedforward +
            fixedwing_course_integral_rate_,
        -maximum_course_rate, maximum_course_rate);
    double desired_roll = clamp(
        -std::atan2(airspeed * desired_course_rate, gravity_),
        -max_roll_, max_roll_);
    if ((reference.trajectory_reference || reference.path_reference) &&
        std::abs(target.course_rate_feedforward) > 0.03) {
      ROS_INFO_THROTTLE(
          1.0,
          "[xd_uav_controller] 固定翼弯道跟踪: "
          "course_error=%.3f ff_rate=%.3f integral=%.3f "
          "roll=%.3f desired_roll=%.3f airspeed=%.1f",
          course_error, target.course_rate_feedforward,
          fixedwing_course_integral_rate_, roll,
          desired_roll, state_.airspeed);
    }

    // A coordinated bank increases lift demand by n=1/cos(phi). The
    // previous controller waited for the resulting drag to reduce measured
    // airspeed before adding throttle. Use the commanded bank here so the
    // compensation is present at turn entry, before that speed loss occurs.
    const double load_factor = 1.0 / std::max(
        0.20, std::cos(std::abs(desired_roll)));
    const double bank_protected_airspeed = clamp(
        minimum_airspeed_ * std::sqrt(load_factor) +
            bank_airspeed_margin_,
        minimum_airspeed_, maximum_airspeed_);
    if (takeoff_active_ || landing_active_) {
      // Dedicated takeoff and landing laws deliberately pass through low
      // airspeed and must not latch the normal airborne recovery mode.
      fixedwing_underspeed_active_ = false;
    } else if (!fixedwing_underspeed_active_ &&
               state_.airspeed < bank_protected_airspeed) {
      fixedwing_underspeed_active_ = true;
    } else if (fixedwing_underspeed_active_ &&
               state_.airspeed >
                   bank_protected_airspeed +
                       underspeed_hysteresis_) {
      fixedwing_underspeed_active_ = false;
    }

    double desired_climb_rate = target.climb_rate_feedforward;
    if (target.use_altitude_feedback) {
      desired_climb_rate +=
          target.altitude_error /
          std::max(0.1, altitude_time_constant_);
    }

    // The integral output is expressed directly as a climb-rate correction.
    // It removes persistent altitude bias without changing trajectory vz
    // feed-forward. Disable it for takeoff/landing, where dedicated pitch
    // laws and saturation would otherwise cause wind-up.
    const bool altitude_integrator_enabled =
        target.enable_altitude_integrator &&
        !fixedwing_underspeed_active_ &&
        altitude_integral_gain_ > 0.0 &&
        altitude_integral_climb_rate_limit_ > 0.0;
    if (altitude_integrator_enabled) {
      const double integral_delta =
          altitude_integral_gain_ * target.altitude_error * dt;
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
    if (fixedwing_underspeed_active_) {
      // Do not trade the remaining kinetic energy for altitude while
      // recovering. The altitude loop resumes after the hysteretic exit.
      desired_climb_rate = std::min(0.0, desired_climb_rate);
    }

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
    if (!takeoff_active_ && !landing_active_ &&
        !fixedwing_underspeed_active_) {
      // A coordinated turn needs n=1/cos(bank) times the straight-flight
      // lift. Throttle feed-forward supplies the extra energy, while this
      // small nose-up feed-forward supplies angle of attack before the
      // reactive altitude loop observes a height loss. The gain remains an
      // airframe parameter because lift-curve slope is not part of State.
      desired_pitch = clamp(
          desired_pitch -
              bank_load_factor_pitch_gain_ * (load_factor - 1.0),
          -max_pitch_, max_pitch_);
    }
    // With the repository's ROS FLU convention, positive inertial-z
    // acceleration requires a negative (nose-up) pitch rate.
    const double pitch_rate_feedforward =
        -vertical_acceleration_pitch_rate_gain_ *
        target.vertical_acceleration_feedforward /
        std::max(minimum_airspeed_, airspeed);
    double throttle = clamp(
        trim_throttle_ +
            airspeed_throttle_gain_ *
                (std::max(target.airspeed,
                          bank_protected_airspeed) -
                 state_.airspeed) +
            climb_rate_throttle_gain_ * desired_climb_rate +
            turn_load_factor_throttle_gain_ *
                (load_factor - 1.0),
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
      if (fixedwing_landing_phase_ ==
              FixedwingLandingPhase::kGlideSlope ||
          fixedwing_landing_phase_ ==
              FixedwingLandingPhase::kRollout) {
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
    if (fixedwing_underspeed_active_) {
      throttle = maximum_throttle_;
      desired_roll = clamp(
          desired_roll, -underspeed_max_roll_,
          underspeed_max_roll_);
      // ROS FLU uses negative pitch for nose-up. Limit nose-up authority;
      // below the configured minimum airspeed command a small positive
      // (nose-down) attitude to make recovery decisive.
      desired_pitch = std::max(
          desired_pitch, -underspeed_max_nose_up_pitch_);
      if (state_.airspeed < minimum_airspeed_) {
        desired_pitch = std::max(
            desired_pitch, underspeed_max_nose_up_pitch_);
      }
      ROS_WARN_THROTTLE(
          1.0,
          "[xd_uav_controller] 固定翼低空速保护: "
          "airspeed=%.2f protected=%.2f load_factor=%.2f",
          state_.airspeed, bank_protected_airspeed,
          load_factor);
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

    if (landing_active_ &&
        fixedwing_landing_phase_ ==
            FixedwingLandingPhase::kRollout) {
      // Airborne course-to-bank control has progressively less authority
      // and becomes inappropriate as the aircraft stops on the ground.
      // Reuse the measured groundspeed and configured approach speed to
      // remove that control continuously, without introducing another
      // rollout tuning parameter. At touchdown the commanded body rates
      // are zero immediately; the control manager still performs its
      // independent sustained-touchdown check before disarming.
      const double rollout_body_rate_scale =
          std::isfinite(state_.groundspeed)
              ? clamp(
                    state_.groundspeed /
                        std::max(
                            0.1,
                            fixedwing_landing_approach_airspeed_),
                    0.0, 1.0)
              : 1.0;
      result.body_rate *= rollout_body_rate_scale;
      if (landing_touchdown_) {
        result.body_rate.setZero();
      }
    }
    const bool landing_allows_zero_throttle =
        landing_active_ &&
        (fixedwing_landing_phase_ ==
             FixedwingLandingPhase::kGlideSlope ||
         fixedwing_landing_phase_ ==
             FixedwingLandingPhase::kRollout);
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
            ? (path_active_ ? "path_finite_horizon_mpc_so3"
                            : "finite_horizon_mpc_so3")
            : (landing_active_
                   ? "fixedwing_course_energy_landing"
                   : fixedwing_loiter_active_
                   ? "fixedwing_course_energy_loiter"
                   : path_active_
                   ? "fixedwing_path_course_energy"
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
      resetMultirotorControlState();
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
      if (path_active_) {
        std::string path_reason;
        if (!resolvePathReference(
                now, &reference, &path_reason)) {
          command.rejection_reason = path_reason;
          command_publisher_.publish(command);
          return;
        }
      } else if (trajectory_active_) {
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
            ? multirotorControl(reference, now)
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
  ros::Subscriber distance_sensor_subscriber_;
  ros::Subscriber reference_position_target_subscriber_;
  ros::Subscriber reference_trajectory_subscriber_;
  ros::Subscriber reference_path_subscriber_;
  ros::Subscriber simple_goal_subscriber_;
  ros::Subscriber local_alignment_valid_subscriber_;
  ros::Publisher reference_position_target_publisher_;
  ros::Publisher reference_trajectory_path_publisher_;
  ros::Publisher path_status_publisher_;
  ros::Publisher command_publisher_;
  ros::ServiceServer internal_command_server_;
  ros::Timer timer_;

  xd_uav_controller::ControlState state_;
  sensor_msgs::Range distance_sensor_;
  Reference reference_;
  Reference reference_source_;
  trajectory_msgs::MultiDOFJointTrajectory
      reference_trajectory_;
  std::vector<Eigen::Vector3d> reference_path_source_points_;
  std::vector<Eigen::Vector3d> reference_path_points_;
  std::vector<double> reference_path_lengths_;
  std::string reference_path_frame_;
  ros::Time last_state_receive_;
  ros::Time last_distance_sensor_receive_;
  ros::Time last_distance_sensor_sample_time_;
  ros::Time last_reference_receive_;
  ros::Time reference_trajectory_start_;
  ros::Time last_local_alignment_valid_receive_;
  ros::Time active_reference_transform_failure_since_;
  ros::Time last_path_status_publish_;
  bool have_state_{false};
  bool have_distance_sensor_{false};
  bool distance_sensor_velocity_valid_{false};
  bool have_reference_{false};
  bool have_reference_error_{false};
  bool have_normalized_reference_{false};
  bool have_local_alignment_valid_{false};
  bool local_alignment_valid_{false};
  bool trajectory_active_{false};
  bool path_active_{false};
  bool point_reference_latched_{false};
  uint32_t simple_goal_sequence_counter_{0};
  uint32_t path_sequence_counter_{0};
  uint32_t active_path_id_{0};
  std::size_t path_current_segment_{0};
  double path_progress_{0.0};
  double path_total_length_{0.0};
  double path_cross_track_error_{0.0};
  bool path_progress_initialized_{false};
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
  double landing_agl_setpoint_{0.0};
  bool landing_agl_setpoint_initialized_{false};
  double distance_sensor_height_{0.0};
  double filtered_distance_sensor_velocity_{0.0};
  Eigen::Vector2d landing_return_velocity_command_{
      Eigen::Vector2d::Zero()};
  double landing_course_{0.0};
  ros::Time last_landing_update_;
  Eigen::Vector3d acceleration_command_state_{
      Eigen::Vector3d::Zero()};
  Eigen::Vector3d position_integral_acceleration_state_{
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
  double path_minimum_segment_length_{0.20};
  double path_status_rate_{10.0};
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
  std::array<double, 3> position_integral_gain_{{
      0.08, 0.08, 0.15}};
  std::array<double, 3>
      position_integral_acceleration_limit_{{
          0.60, 0.60, 0.80}};
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
  double multirotor_anti_windup_gain_{1.5};
  double multirotor_path_nominal_speed_{3.0};
  double multirotor_path_lookahead_distance_{2.0};
  double multirotor_path_terminal_slowdown_distance_{4.0};
  double multirotor_path_completion_position_tolerance_{0.5};
  double multirotor_path_completion_speed_tolerance_{0.3};
  double multirotor_path_reacquisition_distance_{4.0};
  ros::Time last_multirotor_control_time_;

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
  bool fixedwing_streaming_course_rate_feedforward_{true};
  double fixedwing_course_rate_filter_time_constant_{0.25};
  double fixedwing_minimum_course_rate_speed_{2.0};
  double fixedwing_course_integral_gain_{0.08};
  double fixedwing_course_integral_rate_limit_{0.25};
  double fixedwing_course_integral_rate_{0.0};
  double filtered_fixedwing_setpoint_course_rate_{0.0};
  double previous_fixedwing_setpoint_course_{0.0};
  bool fixedwing_setpoint_course_rate_initialized_{false};
  ros::Time last_fixedwing_setpoint_course_sample_time_;
  double fixedwing_setpoint_course_hold_{0.0};
  bool fixedwing_setpoint_course_hold_initialized_{false};
  bool fixedwing_external_setpoint_mode_initialized_{false};
  uint16_t fixedwing_external_setpoint_mode_signature_{0U};
  double altitude_time_constant_{5.0};
  double altitude_integral_gain_{0.02};
  double altitude_integral_climb_rate_limit_{1.0};
  double climb_rate_pitch_gain_{0.06};
  double climb_rate_filter_time_constant_{0.50};
  double vertical_acceleration_pitch_rate_gain_{1.0};
  double bank_load_factor_pitch_gain_{0.20};
  double climb_rate_throttle_gain_{0.03};
  double turn_load_factor_throttle_gain_{0.16};
  double bank_airspeed_margin_{0.5};
  double underspeed_hysteresis_{0.7};
  double underspeed_max_roll_{0.35};
  double underspeed_max_nose_up_pitch_{0.05};
  bool fixedwing_underspeed_active_{false};
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
  double fixedwing_path_curvature_distance_{10.0};
  double fixedwing_path_curvature_preview_time_{0.30};
  double fixedwing_path_completion_radius_{20.0};
  double fixedwing_path_completion_altitude_tolerance_{5.0};
  double fixedwing_path_reacquisition_distance_{50.0};

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
    kLineCapture,
    kGlideSlope,
    kRollout,
  };
  LandingPhase landing_phase_{LandingPhase::kNone};
  FixedwingLandingPhase fixedwing_landing_phase_{
      FixedwingLandingPhase::kNone};
  std::string landing_height_source_{"odom"};
  MultirotorVerticalLandingConfig odom_landing_config_;
  DistanceSensorLandingConfig distance_sensor_landing_config_;
  double landing_return_max_velocity_{1.0};
  double landing_position_tolerance_{0.20};
  double landing_velocity_tolerance_{0.20};
  // Fixed-wing still uses its independent flat landing configuration.
  double landing_touchdown_offset_{0.15};
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
  double fixedwing_landing_guidance_course_{0.0};
  bool fixedwing_landing_guidance_course_initialized_{false};
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
