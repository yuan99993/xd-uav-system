#include <algorithm>
#include <clocale>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <geometry_msgs/AccelWithCovarianceStamped.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandCode.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/CommandVtolTransition.h>
#include <mavros_msgs/ExtendedState.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/VFR_HUD.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <xd_uav_controller/ControlCommand.h>
#include <xd_uav_controller/ControlState.h>
#include <xd_uav_controller/InternalCommand.h>
#include <xd_uav_controller/Takeoff.h>
#include <xd_uav_controller/backend_resolver.h>
#include <xd_uav_controller/command_contract.h>
#include <xd_uav_controller/control_types.h>
#include <xd_uav_control_manager/SetFlightRegime.h>
#include <xd_uav_control_manager/fixedwing_vehicle_adapter.h>
#include <xd_uav_control_manager/multirotor_vehicle_adapter.h>
#include <xd_uav_control_manager/vehicle_adapter.h>
#include <xd_uav_control_manager/vtol_landing_coordinator.h>
#include <xd_uav_control_manager/vtol_takeoff_coordinator.h>
#include <xd_uav_control_manager/vtol_vehicle_adapter.h>
#include <xd_uav_state_estimators/EstimatorStatus.h>

namespace {

double messageAge(const ros::Time& now, const ros::Time& stamp,
                  const ros::Time& receive_time,
                  const double future_stamp_tolerance) {
  if (receive_time.isZero()) {
    return std::numeric_limits<double>::infinity();
  }

  // Transport freshness and source timestamp validity are separate concerns.
  // A high-rate MAVROS stream can have a header stamp a few milliseconds in
  // front of ROS time because FCU timesync and Gazebo /clock are sampled on
  // different cycles. Treat that bounded lead as zero age, while still using
  // receive_age to detect a stream which has actually stopped.
  const double receive_age = (now - receive_time).toSec();
  if (!std::isfinite(receive_age) || receive_age < 0.0) {
    return std::numeric_limits<double>::infinity();
  }
  if (stamp.isZero()) {
    return receive_age;
  }

  const double stamp_age = (now - stamp).toSec();
  if (!std::isfinite(stamp_age) ||
      stamp_age < -future_stamp_tolerance) {
    return std::numeric_limits<double>::infinity();
  }
  return std::max(receive_age, std::max(0.0, stamp_age));
}

bool finiteVector(const geometry_msgs::Vector3& value) {
  return std::isfinite(value.x) && std::isfinite(value.y) &&
         std::isfinite(value.z);
}

bool normalizeQuaternion(
    const geometry_msgs::Quaternion& message,
    tf2::Quaternion* quaternion) {
  tf2::fromMsg(message, *quaternion);
  if (!std::isfinite(quaternion->x()) ||
      !std::isfinite(quaternion->y()) ||
      !std::isfinite(quaternion->z()) ||
      !std::isfinite(quaternion->w()) ||
      quaternion->length2() < 1e-9) {
    return false;
  }
  quaternion->normalize();
  return true;
}

void addDiagnostic(diagnostic_msgs::DiagnosticStatus* status,
                   const std::string& key,
                   const std::string& value) {
  diagnostic_msgs::KeyValue item;
  item.key = key;
  item.value = value;
  status->values.push_back(item);
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
        "[xd_uav_control_manager] 参数%s已迁移为%s；"
        "本次仍兼容旧路径",
        legacy_name.c_str(), name.c_str());
  }
}

}  // namespace

class ControlManagerNode {
 public:
  ControlManagerNode() : private_nh_("~") {
    loadParameters();
    if (xd_uav_controller::isVtolAirframe(airframe_type_)) {
      auto adapter = std::make_unique<
          xd_uav_control_manager::VtolVehicleAdapter>(airframe_type_);
      vtol_adapter_ = adapter.get();
      vehicle_adapter_ = std::move(adapter);
    } else if (airframe_type_ ==
               xd_uav_controller::AirframeType::kFixedWing) {
      vehicle_adapter_ = std::make_unique<
          xd_uav_control_manager::FixedWingVehicleAdapter>();
    } else {
      vehicle_adapter_ = std::make_unique<
          xd_uav_control_manager::MultirotorVehicleAdapter>();
    }
    odometry_subscriber_ = nh_.subscribe(
        "main_odometry", 20,
        &ControlManagerNode::odometryCallback, this);
    acceleration_subscriber_ = nh_.subscribe(
        "main_acceleration", 20,
        &ControlManagerNode::accelerationCallback, this);
    imu_subscriber_ = nh_.subscribe(
        "imu", 100, &ControlManagerNode::imuCallback, this);
    estimator_status_subscriber_ = nh_.subscribe(
        "estimator_status", 10,
        &ControlManagerNode::estimatorStatusCallback, this);
    airspeed_subscriber_ = nh_.subscribe(
        "airspeed", 20, &ControlManagerNode::airspeedCallback, this);
    mavros_state_subscriber_ = nh_.subscribe(
        "mavros_state", 10,
        &ControlManagerNode::mavrosStateCallback, this);
    mavros_extended_state_subscriber_ = nh_.subscribe(
        "mavros_extended_state", 10,
        &ControlManagerNode::mavrosExtendedStateCallback, this);
    command_subscriber_ = nh_.subscribe(
        "controller_command", 20,
        &ControlManagerNode::commandCallback, this);

    state_publisher_ =
        private_nh_.advertise<xd_uav_controller::ControlState>(
            "state", 20);
    status_publisher_ =
        private_nh_.advertise<std_msgs::String>("status", 5, true);
    diagnostics_publisher_ =
        private_nh_.advertise<diagnostic_msgs::DiagnosticArray>(
            "diagnostics", 5);
    attitude_target_publisher_ =
        nh_.advertise<mavros_msgs::AttitudeTarget>(
            "attitude_target", 20);
    offboard_server_ = private_nh_.advertiseService(
        "offboard", &ControlManagerNode::offboardCallback, this);
    cancel_offboard_server_ = private_nh_.advertiseService(
        "cancel_offboard",
        &ControlManagerNode::cancelOffboardCallback, this);
    takeoff_server_ = private_nh_.advertiseService(
        "takeoff", &ControlManagerNode::takeoffCallback, this);
    land_server_ = private_nh_.advertiseService(
        "land", &ControlManagerNode::landCallback, this);
    land_home_server_ = private_nh_.advertiseService(
        "land_home", &ControlManagerNode::landHomeCallback, this);
    cancel_land_server_ = private_nh_.advertiseService(
        "cancel_land", &ControlManagerNode::cancelLandingCallback,
        this);
    reset_failsafe_server_ = private_nh_.advertiseService(
        "reset_failsafe",
        &ControlManagerNode::resetFailsafeCallback, this);
    set_flight_regime_server_ = private_nh_.advertiseService(
        "set_flight_regime",
        &ControlManagerNode::setFlightRegimeCallback, this);
    set_mode_client_ =
        nh_.serviceClient<mavros_msgs::SetMode>("set_mode");
    arming_client_ =
        nh_.serviceClient<mavros_msgs::CommandBool>("arming");
    command_long_client_ =
        nh_.serviceClient<mavros_msgs::CommandLong>("command_long");
    vtol_transition_client_ =
        nh_.serviceClient<mavros_msgs::CommandVtolTransition>(
            vtol_transition_service_);
    controller_internal_command_client_ =
        nh_.serviceClient<xd_uav_controller::InternalCommand>(
            "controller_internal_command");

    timer_ = private_nh_.createTimer(
        ros::Duration(1.0 / std::max(2.0, setpoint_rate_)),
        &ControlManagerNode::timerCallback, this);
    transition(State::kStandby,
               "节点已启动，等待OFFBOARD或起飞服务");
    ROS_INFO("[xd_uav_control_manager] 已加载%s安全与OFFBOARD管理",
             vehicle_type_.c_str());
  }

 private:
  enum class State {
    kStandby,
    kWaitState,
    kPrestream,
    kRequestOffboard,
    kRequestArm,
    kActive,
    kLanding,
    kFailsafe,
  };

  void loadParameters() {
    std::string configured_airframe;
    if (!private_nh_.getParam("airframe_type", configured_airframe)) {
      private_nh_.param("vehicle_type", configured_airframe,
                        std::string("multirotor"));
      ROS_WARN("[xd_uav_control_manager] vehicle_type已废弃，请使用airframe_type");
    }
    airframe_type_ = xd_uav_controller::parseAirframeType(
        configured_airframe);
    if (airframe_type_ == xd_uav_controller::AirframeType::kUnknown) {
      throw std::runtime_error(
          "airframe_type必须是multirotor、fixedwing/fixed_wing、vtol或tiltrotor");
    }
    vehicle_type_ = configured_airframe;
    vehicle_type_id_ =
        airframe_type_ == xd_uav_controller::AirframeType::kFixedWing
            ? xd_uav_controller::ControlState::VEHICLE_FIXEDWING
            : xd_uav_controller::ControlState::VEHICLE_MULTIROTOR;
    private_nh_.param("vtol/transition/timeout", transition_timeout_, 10.0);
    private_nh_.param("vtol/transition/require_offboard",
                      vtol_require_offboard_, true);
    private_nh_.param("vtol/transition/require_armed",
                      vtol_require_armed_, true);
    private_nh_.param("vtol/transition/require_extended_state",
                      vtol_require_extended_state_, true);
    private_nh_.param("vtol/transition/minimum_forward_airspeed",
                      vtol_minimum_forward_airspeed_, 0.0);
    private_nh_.param("mavros/vtol_transition_service",
                      vtol_transition_service_,
                      std::string("mavros/cmd/vtol_transition"));
    if (!std::isfinite(transition_timeout_) || transition_timeout_ <= 0.0 ||
        !std::isfinite(vtol_minimum_forward_airspeed_) ||
        vtol_minimum_forward_airspeed_ < 0.0) {
      throw std::runtime_error("VTOL转换参数不在有效范围内");
    }

    loadParameterWithLegacy(
        private_nh_, "offboard/stream/setpoint_rate",
        "offboard/setpoint_rate", &setpoint_rate_, 100.0);
    loadParameterWithLegacy(
        private_nh_, "offboard/stream/prestream_duration",
        "offboard/prestream_duration", &prestream_duration_, 1.5);
    loadParameterWithLegacy(
        private_nh_, "offboard/mode_request/retry_interval",
        "offboard/request_retry_interval",
        &request_retry_interval_, 1.0);
    loadParameterWithLegacy(
        private_nh_, "offboard/mode_request/timeout",
        "offboard/request_timeout", &request_timeout_, 10.0);
    loadParameterWithLegacy(
        private_nh_, "offboard/exit/cancel_mode",
        "offboard/cancel_mode", &cancel_mode_,
        std::string("POSCTL"));

    loadParameterWithLegacy(
        private_nh_, "safety/inputs/odometry_timeout",
        "safety/odometry_timeout", &odometry_timeout_, 0.20);
    loadParameterWithLegacy(
        private_nh_, "safety/inputs/imu_timeout",
        "safety/imu_timeout", &imu_timeout_, 0.50);
    loadParameterWithLegacy(
        private_nh_, "safety/inputs/acceleration_timeout",
        "safety/acceleration_timeout", &acceleration_timeout_, 0.20);
    loadParameterWithLegacy(
        private_nh_, "safety/inputs/airspeed_timeout",
        "safety/airspeed_timeout", &airspeed_timeout_, 0.30);
    loadParameterWithLegacy(
        private_nh_, "safety/inputs/estimator_status_timeout",
        "safety/estimator_status_timeout",
        &estimator_status_timeout_, 0.30);
    private_nh_.param("safety/inputs/mavros_state_timeout",
                      mavros_state_timeout_, 1.0);
    private_nh_.param("safety/inputs/extended_state_timeout",
                      extended_state_timeout_, 1.0);
    private_nh_.param("safety/inputs/future_stamp_tolerance",
                      future_stamp_tolerance_, 0.05);
    loadParameterWithLegacy(
        private_nh_,
        "safety/inputs/airspeed_negative_tolerance",
        "safety/airspeed_negative_tolerance",
        &airspeed_negative_tolerance_, 3.0);
    loadParameterWithLegacy(
        private_nh_,
        "safety/inputs/minimum_groundspeed_for_course",
        "safety/minimum_groundspeed_for_course",
        &minimum_groundspeed_for_course_, 0.5);
    loadParameterWithLegacy(
        private_nh_, "safety/inputs/require_acceleration",
        "safety/require_acceleration", &require_acceleration_, true);
    loadParameterWithLegacy(
        private_nh_, "safety/inputs/require_localization",
        "safety/require_localization", &require_localization_, false);

    loadParameterWithLegacy(
        private_nh_, "safety/controller_command/timeout",
        "safety/command_timeout", &command_timeout_, 0.20);
    loadParameterWithLegacy(
        private_nh_,
        "safety/controller_command/invalid_grace_duration",
        "safety/invalid_grace_duration",
        &invalid_grace_duration_, 0.5);
    loadParameterWithLegacy(
        private_nh_, "safety/activation/stable_duration",
        "safety/stable_duration", &stable_duration_, 1.5);

    loadParameterWithLegacy(
        private_nh_, "safety/touchdown/confirm_duration",
        "safety/landed_confirm_duration",
        &landed_confirm_duration_, 0.5);
    loadParameterWithLegacy(
        private_nh_, "safety/touchdown/zero_thrust_duration",
        "safety/touchdown_idle_duration",
        &touchdown_idle_duration_, 0.75);
    loadParameterWithLegacy(
        private_nh_,
        "safety/touchdown/fixedwing_max_groundspeed",
        "safety/fixedwing_touchdown_max_groundspeed",
        &fixedwing_touchdown_max_groundspeed_, 2.0);
    loadParameterWithLegacy(
        private_nh_,
        "safety/touchdown/force_disarm/enabled",
        "safety/allow_force_disarm", &allow_force_disarm_, true);
    loadParameterWithLegacy(
        private_nh_,
        "safety/touchdown/force_disarm/timeout",
        "safety/force_disarm_timeout", &force_disarm_timeout_, 2.0);

    const auto positive = [](const double value) {
      return std::isfinite(value) && value > 0.0;
    };
    const auto nonnegative = [](const double value) {
      return std::isfinite(value) && value >= 0.0;
    };
    if (!positive(setpoint_rate_) || setpoint_rate_ < 2.0 ||
        !nonnegative(prestream_duration_) ||
        !positive(request_retry_interval_) ||
        !positive(request_timeout_) ||
        request_timeout_ < request_retry_interval_ ||
        cancel_mode_.empty()) {
      throw std::runtime_error(
          "offboard发送、模式请求或退出参数不在有效范围内");
    }
    if (!positive(odometry_timeout_) ||
        !positive(imu_timeout_) ||
        !positive(acceleration_timeout_) ||
        !positive(airspeed_timeout_) ||
        !positive(estimator_status_timeout_) ||
        !positive(mavros_state_timeout_) ||
        !positive(extended_state_timeout_) ||
        !nonnegative(future_stamp_tolerance_) ||
        !nonnegative(airspeed_negative_tolerance_) ||
        !nonnegative(minimum_groundspeed_for_course_) ||
        !positive(command_timeout_) ||
        !nonnegative(invalid_grace_duration_) ||
        !nonnegative(stable_duration_)) {
      throw std::runtime_error(
          "safety输入、控制器命令或激活参数不在有效范围内");
    }
    if (!positive(landed_confirm_duration_) ||
        !nonnegative(touchdown_idle_duration_) ||
        !positive(fixedwing_touchdown_max_groundspeed_) ||
        !positive(force_disarm_timeout_) ||
        (allow_force_disarm_ &&
         force_disarm_timeout_ <= touchdown_idle_duration_)) {
      throw std::runtime_error(
          "safety触地确认或上锁参数不在有效范围内");
    }
  }

  bool mavrosStateFresh(const ros::Time& now) const {
    return have_mavros_state_ &&
           messageAge(now, mavros_state_.header.stamp,
                      mavros_state_receive_,
                      future_stamp_tolerance_) <=
               mavros_state_timeout_;
  }

  bool mavrosExtendedStateFresh(const ros::Time& now) const {
    return have_mavros_extended_state_ &&
           messageAge(
               now, mavros_extended_state_.header.stamp,
               mavros_extended_state_receive_,
               future_stamp_tolerance_) <=
               extended_state_timeout_;
  }

  xd_uav_controller::FlightRegime observedFlightRegime(
      const ros::Time& now) {
    if (!xd_uav_controller::isVtolAirframe(airframe_type_)) {
      return xd_uav_controller::staticRegimeForAirframe(airframe_type_);
    }
    return vtol_adapter_->observe(
        mavros_extended_state_.vtol_state,
        mavrosExtendedStateFresh(now));
  }

  uint8_t legacyVehicleTypeForRegime(
      const xd_uav_controller::FlightRegime regime) const {
    const auto resolution = xd_uav_controller::resolveBackend(
        airframe_type_, regime, last_active_backend_);
    const auto backend = resolution.valid
                             ? resolution.backend
                             : last_active_backend_;
    return backend == xd_uav_controller::BackendId::kFixedWing
               ? xd_uav_controller::ControlState::VEHICLE_FIXEDWING
               : xd_uav_controller::ControlState::VEHICLE_MULTIROTOR;
  }

  bool regimeRequiresAirspeed(
      const xd_uav_controller::FlightRegime regime) const {
    return vehicle_adapter_ && vehicle_adapter_->requiresAirspeed(regime);
  }

  uint8_t currentVehicleAction() const {
    if (state_machine_state_ == State::kFailsafe) {
      return xd_uav_controller::ControlState::ACTION_EMERGENCY;
    }
    if (touchdown_confirmed_) {
      return xd_uav_controller::ControlState::ACTION_DISARM;
    }
    if (landing_requested_ || state_machine_state_ == State::kLanding) {
      return landing_return_home_
                 ? xd_uav_controller::ControlState::ACTION_RETURN_HOME
                 : xd_uav_controller::ControlState::ACTION_LAND;
    }
    if (arm_requested_) {
      const bool controller_takeoff_active =
          command_.action_generation == action_generation_
              ? command_.action_status ==
                    xd_uav_controller::ControlCommand::ACTION_ACTIVE
              : command_.action_generation == 0U && command_.takeoff_active;
      if (state_machine_state_ != State::kActive ||
          controller_takeoff_active) {
        return xd_uav_controller::ControlState::ACTION_TAKEOFF;
      }
    }
    if (state_machine_state_ != State::kActive) {
      return xd_uav_controller::ControlState::ACTION_NONE;
    }
    switch (command_.reference_type) {
      case xd_uav_controller::ControlCommand::REFERENCE_POSITION_TARGET:
      case xd_uav_controller::ControlCommand::REFERENCE_TRAJECTORY:
      case xd_uav_controller::ControlCommand::REFERENCE_PATH:
      case xd_uav_controller::ControlCommand::REFERENCE_SIMPLE_GOAL:
        return xd_uav_controller::ControlState::ACTION_NAVIGATE;
      case xd_uav_controller::ControlCommand::REFERENCE_IDLE:
      case xd_uav_controller::ControlCommand::REFERENCE_INTERNAL:
      default:
        return xd_uav_controller::ControlState::ACTION_HOLD;
    }
  }

  void odometryCallback(
      const nav_msgs::Odometry::ConstPtr& message) {
    odometry_ = *message;
    odometry_receive_ = ros::Time::now();
    have_odometry_ = true;
  }

  void accelerationCallback(
      const geometry_msgs::AccelWithCovarianceStamped::ConstPtr&
          message) {
    acceleration_ = *message;
    acceleration_receive_ = ros::Time::now();
    have_acceleration_ = true;
  }

  void imuCallback(const sensor_msgs::Imu::ConstPtr& message) {
    imu_ = *message;
    imu_receive_ = ros::Time::now();
    have_imu_ = true;
  }

  void estimatorStatusCallback(
      const xd_uav_state_estimators::EstimatorStatus::ConstPtr&
          message) {
    estimator_status_ = *message;
    estimator_status_receive_ = ros::Time::now();
    have_estimator_status_ = true;
  }

  void airspeedCallback(
      const mavros_msgs::VFR_HUD::ConstPtr& message) {
    airspeed_ = *message;
    airspeed_receive_ = ros::Time::now();
    have_airspeed_ = true;
  }

  void mavrosStateCallback(
      const mavros_msgs::State::ConstPtr& message) {
    mavros_state_ = *message;
    mavros_state_receive_ = ros::Time::now();
    have_mavros_state_ = true;
  }

  void mavrosExtendedStateCallback(
      const mavros_msgs::ExtendedState::ConstPtr& message) {
    mavros_extended_state_ = *message;
    mavros_extended_state_receive_ = ros::Time::now();
    have_mavros_extended_state_ = true;
  }

  void commandCallback(
      const xd_uav_controller::ControlCommand::ConstPtr& message) {
    command_ = *message;
    command_receive_ = ros::Time::now();
    have_command_ = true;
  }

  xd_uav_controller::ControlState buildControlState(
      const ros::Time& now) {
    xd_uav_controller::ControlState state;
    state.header.stamp = now;
    const auto regime = observedFlightRegime(now);
    const auto resolution = xd_uav_controller::resolveBackend(
        airframe_type_, regime, last_active_backend_);
    const auto backend = resolution.valid
                             ? resolution.backend
                             : xd_uav_controller::BackendId::kNone;
    if (backend != xd_uav_controller::BackendId::kNone &&
        backend != last_active_backend_) {
      last_active_backend_ = backend;
      ++regime_generation_;
    }
    state.vehicle_type = legacyVehicleTypeForRegime(regime);
    state.airframe_type = static_cast<uint8_t>(airframe_type_);
    state.flight_regime = static_cast<uint8_t>(regime);
    state.requested_regime = vtol_adapter_
                                 ? static_cast<uint8_t>(
                                       vtol_adapter_->transitionStatus().target)
                                 : static_cast<uint8_t>(
                                       xd_uav_controller::ControlState::REQUESTED_NONE);
    state.vehicle_action = currentVehicleAction();
    state.regime_generation = regime_generation_;
    state.action_generation = action_generation_;
    state.transition_pending =
        vtol_adapter_ && vtol_adapter_->transitionStatus().pending;

    state.odometry_age =
        have_odometry_
            ? messageAge(now, odometry_.header.stamp,
                         odometry_receive_,
                         future_stamp_tolerance_)
            : std::numeric_limits<double>::infinity();
    state.imu_age =
        have_imu_ ? messageAge(now, imu_.header.stamp, imu_receive_,
                               future_stamp_tolerance_)
                  : std::numeric_limits<double>::infinity();
    state.acceleration_age =
        have_acceleration_
            ? messageAge(now, acceleration_.header.stamp,
                         acceleration_receive_,
                         future_stamp_tolerance_)
            : std::numeric_limits<double>::infinity();
    state.airspeed_age =
        have_airspeed_
            ? messageAge(now, airspeed_.header.stamp,
                         airspeed_receive_,
                         future_stamp_tolerance_)
            : std::numeric_limits<double>::infinity();
    const double measured_airspeed =
        have_airspeed_
            ? static_cast<double>(airspeed_.airspeed)
            : std::numeric_limits<double>::quiet_NaN();
    const bool airspeed_ground_clamp_allowed =
        (mavrosStateFresh(now) && !mavros_state_.armed) ||
        (mavrosExtendedStateFresh(now) &&
         mavros_extended_state_.landed_state ==
             mavros_msgs::ExtendedState::
                 LANDED_STATE_ON_GROUND);
    state.odometry_fresh =
        state.odometry_age <= odometry_timeout_;
    state.imu_fresh = state.imu_age <= imu_timeout_;
    state.acceleration_fresh =
        state.acceleration_age <= acceleration_timeout_;
    state.airspeed_valid =
        state.airspeed_age <= airspeed_timeout_ &&
        std::isfinite(measured_airspeed) &&
        (measured_airspeed >= 0.0 ||
         (airspeed_ground_clamp_allowed &&
          measured_airspeed >=
              -airspeed_negative_tolerance_));

    tf2::Quaternion orientation;
    const bool orientation_valid =
        have_odometry_ &&
        normalizeQuaternion(odometry_.pose.pose.orientation,
                            &orientation);
    const bool odometry_values_valid =
        have_odometry_ &&
        std::isfinite(odometry_.pose.pose.position.x) &&
        std::isfinite(odometry_.pose.pose.position.y) &&
        std::isfinite(odometry_.pose.pose.position.z) &&
        finiteVector(odometry_.twist.twist.linear);
    const bool imu_values_valid =
        have_imu_ && finiteVector(imu_.angular_velocity);
    const bool acceleration_values_valid =
        have_acceleration_ &&
        finiteVector(acceleration_.accel.accel.linear) &&
        finiteVector(acceleration_.accel.accel.angular);
    const bool frame_ids_valid =
        have_odometry_ && have_imu_ &&
        !odometry_.header.frame_id.empty() &&
        !odometry_.child_frame_id.empty() &&
        imu_.header.frame_id == odometry_.child_frame_id &&
        (!require_acceleration_ ||
         acceleration_.header.frame_id ==
             odometry_.header.frame_id);

    if (have_odometry_) {
      state.header.frame_id = odometry_.header.frame_id;
      state.body_frame_id = odometry_.child_frame_id;
      state.position_odom = odometry_.pose.pose.position;
      state.orientation_odom_body =
          odometry_.pose.pose.orientation;
      if (orientation_valid) {
        const tf2::Vector3 velocity_body(
            odometry_.twist.twist.linear.x,
            odometry_.twist.twist.linear.y,
            odometry_.twist.twist.linear.z);
        const tf2::Vector3 velocity_odom =
            tf2::quatRotate(orientation, velocity_body);
        state.velocity_odom.x = velocity_odom.x();
        state.velocity_odom.y = velocity_odom.y();
        state.velocity_odom.z = velocity_odom.z();
        state.groundspeed =
            std::hypot(velocity_odom.x(), velocity_odom.y());
        state.climb_rate = velocity_odom.z();
        if (state.groundspeed >=
            minimum_groundspeed_for_course_) {
          state.course =
              std::atan2(velocity_odom.y(), velocity_odom.x());
        } else {
          double roll = 0.0;
          double pitch = 0.0;
          double yaw = 0.0;
          tf2::Matrix3x3(orientation).getRPY(
              roll, pitch, yaw);
          state.course = yaw;
        }
      }
    }
    if (have_imu_) {
      state.body_rate = imu_.angular_velocity;
    }
    if (have_acceleration_) {
      state.acceleration_odom =
          acceleration_.accel.accel.linear;
      state.angular_acceleration_odom =
          acceleration_.accel.accel.angular;
    }
    if (have_airspeed_) {
      state.airspeed =
          std::isfinite(measured_airspeed)
              ? std::max(0.0, measured_airspeed)
              : measured_airspeed;
      if (state.airspeed_valid &&
          measured_airspeed < 0.0) {
        ROS_WARN_THROTTLE(
            2.0,
            "[xd_uav_control_manager] 地面空速为%.2fm/s，"
            "在%.2fm/s负值容差内，按0m/s提供给控制器",
            measured_airspeed,
            airspeed_negative_tolerance_);
      }
    }
    if (have_estimator_status_) {
      state.localization_valid =
          estimator_status_.localization_valid;
      state.active_source = estimator_status_.active_source;
    }

    const double estimator_status_age =
        have_estimator_status_
            ? messageAge(now, estimator_status_.header.stamp,
                         estimator_status_receive_,
                         future_stamp_tolerance_)
            : std::numeric_limits<double>::infinity();
    bool valid =
        have_estimator_status_ &&
        estimator_status_age <= estimator_status_timeout_ &&
        estimator_status_.state_valid &&
        state.odometry_fresh && state.imu_fresh &&
        orientation_valid && odometry_values_valid &&
        imu_values_valid && frame_ids_valid;
    if (require_acceleration_) {
      valid = valid && state.acceleration_fresh &&
              acceleration_values_valid;
    }
    if (require_localization_) {
      valid = valid && estimator_status_.localization_valid;
    }
    if (regimeRequiresAirspeed(regime)) {
      valid = valid && state.airspeed_valid;
    }
    if (xd_uav_controller::isVtolAirframe(airframe_type_)) {
      valid = valid && mavrosExtendedStateFresh(now) &&
              regime != xd_uav_controller::FlightRegime::kUnknown;
    }
    state.state_valid = valid;

    if (valid) {
      if (valid_since_.isZero()) {
        valid_since_ = now;
      }
      state.stable =
          (now - valid_since_).toSec() >= stable_duration_;
    } else {
      valid_since_ = ros::Time();
      state.stable = false;
    }
    return state;
  }

  bool validCommand(const ros::Time& now,
                    std::string* reason) const {
    if (!have_command_) {
      *reason = "尚未收到控制器输出";
      return false;
    }
    const double age =
        messageAge(now, command_.header.stamp, command_receive_,
                   future_stamp_tolerance_);
    if (age > command_timeout_) {
      *reason = "控制器输出超时";
      return false;
    }
    if (!command_.valid) {
      *reason = "控制器拒绝输出: " +
                command_.rejection_reason;
      return false;
    }
    if (command_.vehicle_type != current_control_state_.vehicle_type) {
      *reason = "控制器输出机型不匹配";
      return false;
    }
    // 新控制器用非零backend声明完整的机架/阶段合同；旧控制器仅携带
    // vehicle_type，继续允许它在静态机型上接入。
    const bool has_regime_contract =
        command_.active_backend !=
        xd_uav_controller::ControlCommand::BACKEND_NONE;
    const auto resolution = xd_uav_controller::resolveBackend(
        airframe_type_,
        static_cast<xd_uav_controller::FlightRegime>(
            current_control_state_.flight_regime),
        last_active_backend_);
    const auto expected_backend = resolution.valid
                                      ? resolution.backend
                                      : xd_uav_controller::BackendId::kNone;
    if (has_regime_contract) {
      std::string contract_reason;
      if (!xd_uav_controller::commandMatchesStateContract(
              current_control_state_, command_, expected_backend,
              &contract_reason)) {
        *reason = "控制器输出合同不匹配: " + contract_reason;
        return false;
      }
    }
    if (command_.action_generation == current_control_state_.action_generation &&
        command_.action_status ==
            xd_uav_controller::ControlCommand::ACTION_FAILED) {
      *reason = "控制器动作失败: " + command_.action_detail;
      return false;
    }
    if (!finiteVector(command_.body_rate) ||
        !std::isfinite(command_.thrust) ||
        command_.thrust < 0.0 || command_.thrust > 1.0) {
      *reason = "控制器输出包含非法值";
      return false;
    }
    return true;
  }

  void publishAttitudeTarget(const ros::Time& now,
                             const bool refresh = true) {
    if (refresh) {
      last_safe_target_.header.frame_id =
          current_control_state_.body_frame_id;
      last_safe_target_.type_mask =
          mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;
      last_safe_target_.orientation.w = 1.0;
      last_safe_target_.body_rate = command_.body_rate;
      last_safe_target_.thrust =
          static_cast<float>(std::max(
              0.0, std::min(1.0, command_.thrust)));
      have_last_safe_target_ = true;
    }
    if (!have_last_safe_target_) {
      return;
    }
    last_safe_target_.header.stamp = now;
    attitude_target_publisher_.publish(last_safe_target_);
  }

  void publishTouchdownTarget(const ros::Time& now) {
    last_safe_target_.header.stamp = now;
    last_safe_target_.header.frame_id =
        current_control_state_.body_frame_id;
    last_safe_target_.type_mask =
        mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;
    last_safe_target_.orientation.x = 0.0;
    last_safe_target_.orientation.y = 0.0;
    last_safe_target_.orientation.z = 0.0;
    last_safe_target_.orientation.w = 1.0;
    last_safe_target_.body_rate.x = 0.0;
    last_safe_target_.body_rate.y = 0.0;
    last_safe_target_.body_rate.z = 0.0;
    last_safe_target_.thrust = 0.0F;
    have_last_safe_target_ = true;
    attitude_target_publisher_.publish(last_safe_target_);
  }

  void requestOffboard(const ros::Time& now) {
    if (!last_request_.isZero() &&
         (now - last_request_).toSec() <
             request_retry_interval_) {
      return;
    }
    mavros_msgs::SetMode service;
    service.request.custom_mode = offboard_mode_;
    const bool service_called = set_mode_client_.call(service);
    last_mode_request_accepted_ =
        service_called && service.response.mode_sent;
    ++mode_request_attempts_;
    if (!last_mode_request_accepted_) {
      ROS_WARN_THROTTLE(
          1.0, "[xd_uav_control_manager] OFFBOARD请求失败");
    }
    last_request_ = now;
  }

  void requestArm(const ros::Time& now) {
    if (!arm_requested_ ||
        (!last_request_.isZero() &&
         (now - last_request_).toSec() <
             request_retry_interval_)) {
      return;
    }
    mavros_msgs::CommandBool service;
    service.request.value = true;
    if (!arming_client_.call(service) ||
        !service.response.success) {
      ROS_WARN_THROTTLE(
          1.0, "[xd_uav_control_manager] 解锁请求失败");
    }
    last_request_ = now;
  }

  void requestDisarm(const ros::Time& now) {
    if (!last_request_.isZero() &&
        (now - last_request_).toSec() <
            request_retry_interval_) {
      return;
    }
    mavros_msgs::CommandBool service;
    service.request.value = false;
    if (!arming_client_.call(service) ||
        !service.response.success) {
      ROS_WARN_THROTTLE(
          1.0, "[xd_uav_control_manager] 上锁请求失败");
    }
    normal_disarm_attempted_ = true;
    last_request_ = now;
  }

  void requestForceDisarm(const ros::Time& now) {
    if (!allow_force_disarm_ ||
        (!last_request_.isZero() &&
         (now - last_request_).toSec() <
             request_retry_interval_)) {
      return;
    }
    mavros_msgs::CommandLong service;
    service.request.broadcast = false;
    service.request.command =
        mavros_msgs::CommandCode::COMPONENT_ARM_DISARM;
    service.request.confirmation = 0;
    service.request.param1 = 0.0F;
    service.request.param2 = 21196.0F;
    const bool accepted =
        command_long_client_.call(service) &&
        service.response.success;
    force_disarm_requested_ = true;
    if (accepted) {
      ROS_WARN(
          "[xd_uav_control_manager] PX4未识别落地，"
          "已在零推力触地状态请求强制上锁");
    } else {
      ROS_ERROR_THROTTLE(
          1.0,
          "[xd_uav_control_manager] 强制上锁请求失败");
    }
    last_request_ = now;
  }

  void transition(const State next, const std::string& reason) {
    if (state_machine_state_ == next &&
        state_reason_ == reason) {
      return;
    }
    state_machine_state_ = next;
    state_reason_ = reason;
    state_entered_ = ros::Time::now();
    last_request_ = ros::Time();
    std_msgs::String status;
    status.data = stateName(next) + ": " + reason;
    status_publisher_.publish(status);
    ROS_INFO("[xd_uav_control_manager] %s",
             status.data.c_str());
  }

  static std::string stateName(const State state) {
    switch (state) {
      case State::kStandby:
        return "STANDBY";
      case State::kWaitState:
        return "WAIT_STATE";
      case State::kPrestream:
        return "PRESTREAM";
      case State::kRequestOffboard:
        return "REQUEST_OFFBOARD";
      case State::kRequestArm:
        return "REQUEST_ARM";
      case State::kActive:
        return "ACTIVE";
      case State::kLanding:
        return "LANDING";
      case State::kFailsafe:
        return "FAILSAFE";
    }
    return "UNKNOWN";
  }

  void enterStandby(const std::string& reason) {
    xd_uav_controller::InternalCommand reset_service;
    reset_service.request.command =
        xd_uav_controller::InternalCommand::Request::RESET;
    reset_service.request.vehicle_action =
        xd_uav_controller::InternalCommand::Request::ACTION_NONE;
    reset_service.request.action_phase =
        xd_uav_controller::InternalCommand::Request::PHASE_RESET;
    reset_service.request.action_generation = action_generation_ + 1U;
    if (!controller_internal_command_client_.call(reset_service) ||
        !reset_service.response.success) {
      ROS_WARN_THROTTLE(
          1.0,
          "[xd_uav_control_manager] 控制器内部参考复位失败");
    }
    offboard_requested_ = false;
    arm_requested_ = false;
    landing_requested_ = false;
    landing_return_home_ = false;
    landing_seen_in_air_ = false;
    touchdown_confirmed_ = false;
    normal_disarm_attempted_ = false;
    force_disarm_requested_ = false;
    vtol_takeoff_.reset();
    pending_takeoff_altitude_ = 0.0;
    vtol_landing_.reset();
    vtol_vertical_descent_commanded_ = false;
    transition_error_active_ = false;
    ++action_generation_;
    invalid_since_ = ros::Time();
    landed_since_ = ros::Time();
    touchdown_confirmed_at_ = ros::Time();
    have_last_safe_target_ = false;
    transition(State::kStandby, reason);
  }

  bool offboardCallback(
      std_srvs::Trigger::Request&,
      std_srvs::Trigger::Response& response) {
    if (state_machine_state_ == State::kFailsafe) {
      response.success = false;
      response.message =
          "当前处于FAILSAFE，请先调用reset_failsafe";
      return true;
    }
    if (state_machine_state_ == State::kLanding) {
      response.success = false;
      response.message = "正在降落，不能重新请求OFFBOARD";
      return true;
    }
    if (state_machine_state_ == State::kActive &&
        mavros_state_.mode == offboard_mode_) {
      response.success = true;
      response.message = "OFFBOARD已经处于持续维持状态";
      return true;
    }
    offboard_requested_ = true;
    arm_requested_ = false;
    transition(State::kWaitState,
               "收到OFFBOARD服务请求，等待稳定控制状态");
    response.success = true;
    response.message = "已接受OFFBOARD请求";
    return true;
  }

  bool cancelOffboardCallback(
      std_srvs::Trigger::Request&,
      std_srvs::Trigger::Response& response) {
    if (state_machine_state_ == State::kFailsafe) {
      response.success = false;
      response.message =
          "当前处于FAILSAFE，拒绝覆盖PX4安全模式";
      return true;
    }
    if (!offboard_requested_ &&
        mavros_state_.mode != offboard_mode_) {
      response.success = true;
      response.message = "OFFBOARD当前未启用";
      return true;
    }

    if (mavros_state_.mode == offboard_mode_) {
      mavros_msgs::SetMode service;
      service.request.custom_mode = cancel_mode_;
      if (!set_mode_client_.call(service) ||
          !service.response.mode_sent) {
        response.success = false;
        response.message =
            "PX4拒绝切换到" + cancel_mode_ +
            "，继续维持OFFBOARD";
        return true;
      }
    }

    enterStandby(
        "用户取消OFFBOARD，已交还给" + cancel_mode_);
    response.success = true;
    response.message =
        "已取消OFFBOARD并请求切换到" + cancel_mode_;
    return true;
  }

  bool takeoffCallback(
      xd_uav_controller::Takeoff::Request& request,
      xd_uav_controller::Takeoff::Response& response) {
    if (state_machine_state_ == State::kFailsafe) {
      response.success = false;
      response.message =
          "当前处于FAILSAFE，请先调用reset_failsafe";
      return true;
    }
    if (state_machine_state_ == State::kLanding) {
      response.success = false;
      response.message = "正在降落，不能起飞";
      return true;
    }
    if (have_mavros_state_ && mavros_state_.armed) {
      response.success = false;
      response.message = "飞机已经解锁，拒绝重复起飞";
      return true;
    }

    const ros::Time now = ros::Time::now();
    const uint32_t generation = action_generation_ + 1U;
    std::string command_reason;
    bool takeoff_started = false;
    if (vtol_adapter_) {
      if (!mavrosExtendedStateFresh(now)) {
        response.success = false;
        response.message =
            "mavros/extended_state不新鲜，无法确认VTOL处于旋翼模式";
        return true;
      }
      std::string phase_reason;
      if (!vtol_takeoff_.start(observedFlightRegime(now), &phase_reason)) {
        response.success = false;
        response.message = phase_reason;
        return true;
      }
      if (vtol_takeoff_.phase() ==
          xd_uav_control_manager::VtolTakeoffPhase::kRequestHover) {
        if (!commandControllerTakeoff(
                generation, request.altitude,
                xd_uav_controller::InternalCommand::Request::
                    PHASE_REQUEST_HOVER,
                &command_reason)) {
          vtol_takeoff_.fail(command_reason);
          response.success = false;
          response.message = command_reason;
          return true;
        }
        if (!requestVtolTransition(
                xd_uav_controller::RequestedRegime::kHover,
                now, &command_reason)) {
          vtol_takeoff_.fail(command_reason);
          response.success = false;
          response.message =
              "VTOL起飞前无法切换旋翼模式: " + command_reason;
          return true;
        }
        vtol_takeoff_.transitionRequested();
      } else {
        takeoff_started = commandControllerTakeoff(
            generation, request.altitude,
            xd_uav_controller::InternalCommand::Request::PHASE_START,
            &command_reason);
        if (!takeoff_started) {
          vtol_takeoff_.fail(command_reason);
          response.success = false;
          response.message = command_reason;
          return true;
        }
        vtol_takeoff_.commandStarted();
      }
    } else {
      takeoff_started = commandControllerTakeoff(
          generation, request.altitude,
          xd_uav_controller::InternalCommand::Request::PHASE_START,
          &command_reason);
    }
    if (!vtol_adapter_ && !takeoff_started) {
      response.success = false;
      response.message = command_reason;
      return true;
    }

    offboard_requested_ = true;
    arm_requested_ = true;
    landing_requested_ = false;
    landing_return_home_ = false;
    action_generation_ = generation;
    pending_takeoff_altitude_ = request.altitude;
    transition(State::kWaitState,
               vtol_adapter_ && !takeoff_started
                   ? "收到VTOL起飞请求，先切换并确认旋翼模式"
                   : "收到起飞请求，准备OFFBOARD和解锁");
    response.success = true;
    response.message =
        vtol_adapter_ && !takeoff_started
            ? "已接受VTOL起飞请求：确认旋翼模式后再进入OFFBOARD并解锁"
            : "已接受起飞请求：将依次进入OFFBOARD、解锁并起飞";
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

  bool requestVtolTransition(
      const xd_uav_controller::RequestedRegime target,
      const ros::Time& now, std::string* reason) {
    const bool was_pending = vtol_adapter_->transitionStatus().pending;
    if (!vtol_adapter_->beginTransition(target, now, reason)) {
      return false;
    }
    transition_error_active_ = false;
    if (!vtol_adapter_->transitionStatus().pending || was_pending) {
      return true;
    }

    mavros_msgs::CommandVtolTransition service;
    service.request.state =
        target == xd_uav_controller::RequestedRegime::kForwardFlight
            ? mavros_msgs::CommandVtolTransition::Request::STATE_FW
            : mavros_msgs::CommandVtolTransition::Request::STATE_MC;
    if (!vtol_transition_client_.call(service)) {
      vtol_adapter_->recordTransportFailure(
          "MAVROS VTOL transition service unavailable");
      transition_error_active_ = true;
      *reason = vtol_adapter_->transitionStatus().detail;
      return false;
    }
    if (!service.response.success) {
      vtol_adapter_->recordServiceFailure(
          service.response.result,
          "PX4 rejected VTOL transition request");
      transition_error_active_ = true;
      *reason = vtol_adapter_->transitionStatus().detail;
      return false;
    }
    vtol_adapter_->recordServiceAccepted(service.response.result);
    *reason = "转换请求已接受，等待PX4确认实际飞行形态";
    return true;
  }

  bool commandControllerTakeoff(const uint32_t generation,
                                const double altitude,
                                const uint8_t phase,
                                std::string* reason) {
    xd_uav_controller::InternalCommand service;
    service.request.command =
        xd_uav_controller::InternalCommand::Request::TAKEOFF;
    service.request.altitude = altitude;
    service.request.vehicle_action =
        xd_uav_controller::InternalCommand::Request::ACTION_TAKEOFF;
    service.request.action_phase = phase;
    service.request.action_generation = generation;
    if (!controller_internal_command_client_.call(service)) {
      *reason = "无法调用控制器内部命令服务";
      return false;
    }
    *reason = service.response.message;
    return service.response.success;
  }

  bool commandControllerLanding(const uint32_t generation,
                                const bool return_home,
                                const uint8_t phase,
                                std::string* reason) {
    xd_uav_controller::InternalCommand service;
    service.request.command =
        return_home
            ? xd_uav_controller::InternalCommand::Request::LAND_HOME
            : xd_uav_controller::InternalCommand::Request::LAND;
    service.request.vehicle_action =
        return_home
            ? xd_uav_controller::InternalCommand::Request::ACTION_RETURN_HOME
            : xd_uav_controller::InternalCommand::Request::ACTION_LAND;
    service.request.action_phase = phase;
    service.request.action_generation = generation;
    service.request.return_home = return_home;
    if (!controller_internal_command_client_.call(service)) {
      *reason = "无法调用控制器内部命令服务";
      return false;
    }
    *reason = service.response.message;
    return service.response.success;
  }

  bool startLanding(
      const bool return_home,
      std_srvs::Trigger::Response& response) {
    if (state_machine_state_ == State::kLanding) {
      response.success = true;
      response.message = "降落已经在执行";
      return true;
    }
    const ros::Time now = ros::Time::now();
    if (state_machine_state_ != State::kActive ||
        !mavrosStateFresh(now) ||
        !mavros_state_.armed ||
        mavros_state_.mode != offboard_mode_) {
      response.success = false;
      response.message =
          "只有已解锁并处于OFFBOARD主动控制时才能降落";
      return true;
    }
    if (!mavrosExtendedStateFresh(now) ||
        mavros_extended_state_.landed_state !=
            mavros_msgs::ExtendedState::LANDED_STATE_IN_AIR) {
      response.success = false;
      response.message =
          "mavros/extended_state未实时确认飞机在空中";
      return true;
    }

    const uint32_t generation = action_generation_ + 1U;
    std::string command_reason;
    bool controller_started = false;
    if (vtol_adapter_) {
      std::string phase_reason;
      const auto regime = observedFlightRegime(now);
      if (!vtol_landing_.start(regime, &phase_reason)) {
        response.success = false;
        response.message = phase_reason;
        return true;
      }
      if (vtol_landing_.phase() ==
          xd_uav_control_manager::VtolLandingPhase::kRequestHover) {
        if (!commandControllerLanding(
                generation, return_home,
                xd_uav_controller::InternalCommand::Request::
                    PHASE_REQUEST_HOVER,
                &command_reason)) {
          vtol_landing_.fail(command_reason);
          response.success = false;
          response.message = command_reason;
          return true;
        }
        if (!requestVtolTransition(
                xd_uav_controller::RequestedRegime::kHover,
                now, &command_reason)) {
          vtol_landing_.fail(command_reason);
          transition_error_active_ = true;
          transition(State::kFailsafe,
                     "VTOL降落前转换悬停失败: " + command_reason);
          response.success = false;
          response.message = command_reason;
          return true;
        }
        vtol_landing_.transitionRequested();
      } else {
        controller_started = commandControllerLanding(
            generation, return_home,
            xd_uav_controller::InternalCommand::Request::
                PHASE_VERTICAL_DESCENT,
            &command_reason);
      }
    } else {
      const uint8_t phase =
          last_active_backend_ ==
                  xd_uav_controller::BackendId::kFixedWing
              ? xd_uav_controller::InternalCommand::Request::PHASE_APPROACH
              : xd_uav_controller::InternalCommand::Request::
                    PHASE_VERTICAL_DESCENT;
      controller_started = commandControllerLanding(
          generation, return_home, phase, &command_reason);
    }
    if ((!vtol_adapter_ ||
         vtol_landing_.phase() ==
             xd_uav_control_manager::VtolLandingPhase::kVerticalDescent) &&
        !controller_started) {
      response.success = false;
      response.message = command_reason;
      return true;
    }
    vtol_vertical_descent_commanded_ = controller_started;

    landing_requested_ = true;
    landing_return_home_ = return_home;
    action_generation_ = generation;
    arm_requested_ = false;
    landing_seen_in_air_ =
        have_mavros_extended_state_ &&
        mavros_extended_state_.landed_state ==
            mavros_msgs::ExtendedState::LANDED_STATE_IN_AIR;
    touchdown_confirmed_ = false;
    normal_disarm_attempted_ = false;
    force_disarm_requested_ = false;
    landed_since_ = ros::Time();
    touchdown_confirmed_at_ = ros::Time();
    if (last_active_backend_ ==
        xd_uav_controller::BackendId::kFixedWing && !vtol_adapter_) {
      transition(
          State::kLanding,
          return_home
              ? "收到固定翼返航降落请求，飞向配置home并执行进近"
              : "收到固定翼降落请求，沿当前航向建立进近航线");
    } else {
      transition(
          State::kLanding,
          vtol_adapter_ && !controller_started
              ? "收到VTOL降落请求，等待PX4确认悬停后再垂直下降"
              : return_home
                    ? "收到返航降落请求，先返回配置home再受控下降"
                    : "收到原地降落请求，保持水平位置并受控下降");
    }
    response.success = true;
    response.message =
        return_home
            ? "已接受返航降落请求"
            : "已接受原地降落请求";
    return true;
  }

  bool cancelLandingCallback(
      std_srvs::Trigger::Request&,
      std_srvs::Trigger::Response& response) {
    if (state_machine_state_ != State::kLanding) {
      response.success = true;
      response.message = "当前没有正在执行的降落";
      return true;
    }
    if (vtol_adapter_ &&
        (vtol_landing_.phase() ==
             xd_uav_control_manager::VtolLandingPhase::kRequestHover ||
         vtol_landing_.phase() ==
             xd_uav_control_manager::VtolLandingPhase::kWaitHover)) {
      response.success = false;
      response.message =
          "VTOL正在执行降落前形态转换，拒绝中途取消或反转";
      return true;
    }
    const ros::Time now = ros::Time::now();
    if (!mavrosStateFresh(now) || !mavros_state_.armed ||
        mavros_state_.mode != offboard_mode_) {
      response.success = false;
      response.message =
          "飞机未保持解锁OFFBOARD，不能按普通飞行状态取消降落";
      return true;
    }
    if (!mavrosExtendedStateFresh(now) ||
        mavros_extended_state_.landed_state !=
            mavros_msgs::ExtendedState::LANDED_STATE_IN_AIR) {
      response.success = false;
      response.message =
          "PX4未实时确认飞机仍在空中，拒绝取消降落";
      return true;
    }
    if (touchdown_confirmed_ || !landed_since_.isZero() ||
        (have_command_ && command_.landing_touchdown)) {
      response.success = false;
      response.message =
          "已经进入触地确认阶段，不能取消降落";
      return true;
    }

    std::string command_reason;
    if (!current_control_state_.state_valid ||
        !validCommand(now, &command_reason)) {
      response.success = false;
      response.message =
          "当前飞行控制输入无效，拒绝取消降落: " +
          command_reason;
      return true;
    }

    xd_uav_controller::InternalCommand service;
    service.request.command =
        xd_uav_controller::InternalCommand::Request::CANCEL_LANDING;
    service.request.vehicle_action =
        xd_uav_controller::InternalCommand::Request::ACTION_HOLD;
    service.request.action_phase =
        xd_uav_controller::InternalCommand::Request::PHASE_CANCEL;
    service.request.action_generation = action_generation_ + 1U;
    if (!controller_internal_command_client_.call(service)) {
      response.success = false;
      response.message = "无法调用控制器取消降落命令";
      return true;
    }
    if (!service.response.success) {
      response.success = false;
      response.message = service.response.message;
      return true;
    }

    landing_requested_ = false;
    landing_return_home_ = false;
    action_generation_ = service.request.action_generation;
    vtol_landing_.reset();
    vtol_vertical_descent_commanded_ = false;
    arm_requested_ = true;
    landing_seen_in_air_ = false;
    touchdown_confirmed_ = false;
    normal_disarm_attempted_ = false;
    force_disarm_requested_ = false;
    invalid_since_ = ros::Time();
    landed_since_ = ros::Time();
    touchdown_confirmed_at_ = ros::Time();
    transition(
        State::kActive,
        airframe_type_ ==
                xd_uav_controller::AirframeType::kFixedWing
            ? "用户取消降落，保持OFFBOARD并进入等待盘旋"
            : "用户取消降落，保持OFFBOARD并悬停");
    response.success = true;
    response.message = service.response.message +
                       "；OFFBOARD和解锁状态保持不变";
    return true;
  }

  bool resetFailsafeCallback(
      std_srvs::Trigger::Request&,
      std_srvs::Trigger::Response& response) {
    if (state_machine_state_ != State::kFailsafe) {
      response.success = true;
      response.message = "当前没有FAILSAFE";
      return true;
    }
    enterStandby("FAILSAFE已复位，等待服务请求");
    response.success = true;
    response.message = "FAILSAFE已复位";
    return true;
  }

  bool setFlightRegimeCallback(
      xd_uav_control_manager::SetFlightRegime::Request& request,
      xd_uav_control_manager::SetFlightRegime::Response& response) {
    if (!vtol_adapter_) {
      response.accepted = false;
      response.message = "当前机架不支持VTOL转换";
      return true;
    }
    const ros::Time now = ros::Time::now();
    if (state_machine_state_ != State::kActive || !mavrosStateFresh(now) ||
        (vtol_require_armed_ && !mavros_state_.armed) ||
        (vtol_require_offboard_ && mavros_state_.mode != offboard_mode_) ||
        (vtol_require_extended_state_ && !mavrosExtendedStateFresh(now))) {
      response.accepted = false;
      response.message = "VTOL转换前置状态不满足配置要求";
      return true;
    }
    xd_uav_controller::RequestedRegime target;
    if (request.target_regime ==
        xd_uav_control_manager::SetFlightRegime::Request::TARGET_HOVER) {
      target = xd_uav_controller::RequestedRegime::kHover;
    } else if (request.target_regime ==
               xd_uav_control_manager::SetFlightRegime::Request::
                   TARGET_FORWARD_FLIGHT) {
      target = xd_uav_controller::RequestedRegime::kForwardFlight;
    } else {
      response.accepted = false;
      response.message = "未知的目标飞行形态";
      return true;
    }
    if (target == xd_uav_controller::RequestedRegime::kForwardFlight &&
        !current_control_state_.airspeed_valid) {
      response.accepted = false;
      response.message = "VTOL前转换要求新鲜有效的空速输入";
      return true;
    }
    if (target == xd_uav_controller::RequestedRegime::kForwardFlight &&
        vtol_minimum_forward_airspeed_ > 0.0 &&
        current_control_state_.airspeed < vtol_minimum_forward_airspeed_) {
      response.accepted = false;
      response.message = "当前空速低于vtol/transition/minimum_forward_airspeed";
      return true;
    }

    std::string reason;
    response.accepted = requestVtolTransition(target, now, &reason);
    response.message = reason;
    return true;
  }

  void publishDiagnostics(const ros::Time& now,
                          const std::string& command_reason) {
    diagnostic_msgs::DiagnosticArray array;
    array.header.stamp = now;
    diagnostic_msgs::DiagnosticStatus status;
    status.name = "xd_uav_control_manager/offboard";
    status.hardware_id = vehicle_type_;
    status.level =
        state_machine_state_ == State::kFailsafe || transition_error_active_
            ? diagnostic_msgs::DiagnosticStatus::ERROR
            : (state_machine_state_ == State::kActive
                   ? diagnostic_msgs::DiagnosticStatus::OK
                   : diagnostic_msgs::DiagnosticStatus::WARN);
    status.message = stateName(state_machine_state_) +
                     ": " + state_reason_;
    addDiagnostic(&status, "offboard_requested",
                  offboard_requested_ ? "true" : "false");
    addDiagnostic(&status, "arm_requested",
                  arm_requested_ ? "true" : "false");
    addDiagnostic(&status, "landing_requested",
                  landing_requested_ ? "true" : "false");
    addDiagnostic(&status, "landing_touchdown",
                  command_.landing_touchdown
                      ? "true"
                      : "false");
    addDiagnostic(
        &status, "groundspeed",
        std::to_string(current_control_state_.groundspeed));
    addDiagnostic(&status, "touchdown_confirmed",
                  touchdown_confirmed_
                      ? "true"
                      : "false");
    addDiagnostic(&status, "normal_disarm_attempted",
                  normal_disarm_attempted_
                      ? "true"
                      : "false");
    addDiagnostic(&status, "force_disarm_requested",
                  force_disarm_requested_
                      ? "true"
                      : "false");
    addDiagnostic(&status, "state_valid",
                  current_control_state_.state_valid
                      ? "true"
                      : "false");
    addDiagnostic(&status, "stable",
                  current_control_state_.stable
                      ? "true"
                      : "false");
    addDiagnostic(&status, "command", command_reason);
    addDiagnostic(&status, "mavros_mode", mavros_state_.mode);
    addDiagnostic(&status, "armed",
                  mavros_state_.armed ? "true" : "false");
    addDiagnostic(&status, "input_health",
                  active_input_reason_);
    addDiagnostic(&status, "mode_request_attempts",
                  std::to_string(mode_request_attempts_));
    addDiagnostic(&status, "last_mode_request_accepted",
                  last_mode_request_accepted_ ? "true" : "false");
    addDiagnostic(&status, "airframe_type",
                  std::to_string(current_control_state_.airframe_type));
    addDiagnostic(&status, "flight_regime",
                  std::to_string(current_control_state_.flight_regime));
    addDiagnostic(&status, "requested_regime",
                  std::to_string(current_control_state_.requested_regime));
    addDiagnostic(&status, "vehicle_action",
                  std::to_string(current_control_state_.vehicle_action));
    addDiagnostic(&status, "regime_generation",
                  std::to_string(current_control_state_.regime_generation));
    addDiagnostic(&status, "action_generation",
                  std::to_string(current_control_state_.action_generation));
    addDiagnostic(&status, "controller_action_status",
                  std::to_string(command_.action_status));
    addDiagnostic(&status, "controller_action_detail",
                  command_.action_detail);
    if (vtol_adapter_) {
      const auto& transition_status = vtol_adapter_->transitionStatus();
      addDiagnostic(&status, "vtol_state_raw",
                    std::to_string(mavros_extended_state_.vtol_state));
      addDiagnostic(&status, "transition_pending",
                    transition_status.pending ? "true" : "false");
      addDiagnostic(&status, "transition_elapsed",
                    transition_status.started_at.isZero()
                        ? "0"
                        : std::to_string((now - transition_status.started_at).toSec()));
      addDiagnostic(&status, "last_transition_result",
                    std::to_string(static_cast<uint8_t>(
                        transition_status.last_result)));
      addDiagnostic(&status, "last_transition_ack",
                    std::to_string(transition_status.raw_ack_result));
      addDiagnostic(&status, "transition_detail",
                    transition_status.detail);
      addDiagnostic(&status, "vtol_landing_phase",
                    std::to_string(static_cast<uint8_t>(
                        vtol_landing_.phase())));
      addDiagnostic(&status, "vtol_takeoff_phase",
                    std::to_string(static_cast<uint8_t>(
                        vtol_takeoff_.phase())));
      addDiagnostic(&status, "transition_error",
                    transition_error_active_ ? "true" : "false");
    }
    array.status.push_back(status);
    diagnostics_publisher_.publish(array);
  }

  std::string inputFailureReason(
      const ros::Time& now, const bool command_valid,
      const std::string& command_reason,
      const bool mavros_connected) const {
    if (!mavros_connected) {
      return "MAVROS连接或mavros/state超时";
    }
    if (!have_estimator_status_) {
      return "未收到估计器状态";
    }
    if (messageAge(now, estimator_status_.header.stamp,
                   estimator_status_receive_,
                   future_stamp_tolerance_) >
        estimator_status_timeout_) {
      return "估计器状态超时";
    }
    if (!estimator_status_.state_valid) {
      return "估计器报告state_valid=false";
    }
    if (!current_control_state_.odometry_fresh) {
      return "Odometry超时";
    }
    if (!current_control_state_.imu_fresh) {
      return "IMU超时";
    }
    if (require_acceleration_ &&
        !current_control_state_.acceleration_fresh) {
      return "加速度状态超时";
    }
    if (!current_control_state_.state_valid) {
      return "控制状态数值、坐标系或必需输入无效";
    }
    if (!command_valid) {
      return command_reason;
    }
    return "正常";
  }

  bool publishWithInputGrace(
      const ros::Time& now, const bool inputs_healthy) {
    if (inputs_healthy) {
      invalid_since_ = ros::Time();
      publishAttitudeTarget(now, true);
      return true;
    }
    if (invalid_since_.isZero()) {
      invalid_since_ = now;
    }
    const double invalid_duration =
        (now - invalid_since_).toSec();
    if (invalid_duration <= invalid_grace_duration_ &&
        have_last_safe_target_) {
      publishAttitudeTarget(now, false);
      ROS_WARN_THROTTLE(
          1.0,
          "[xd_uav_control_manager] 输入短暂失效(%.2fs/%0.2fs): %s",
          invalid_duration, invalid_grace_duration_,
          active_input_reason_.c_str());
      return true;
    }
    transition(
        State::kFailsafe,
        "输入持续失效: " + active_input_reason_);
    return false;
  }

  void timerCallback(const ros::TimerEvent&) {
    const ros::Time now = ros::Time::now();
    if (vtol_adapter_) {
      const auto observed_regime = observedFlightRegime(now);
      if (vtol_adapter_->transitionStatus().last_result ==
          xd_uav_control_manager::TransitionResult::kCompleted) {
        transition_error_active_ = false;
      }
      if (state_machine_state_ == State::kLanding &&
          vtol_landing_.observe(observed_regime)) {
        transition(State::kLanding,
                   "PX4已确认悬停，等待multirotor backend完成接管");
      }
      if (state_machine_state_ == State::kWaitState &&
          vtol_takeoff_.observe(observed_regime)) {
        transition(State::kWaitState,
                   "PX4已确认旋翼模式，等待multirotor backend完成接管");
      }
      if (vtol_adapter_->checkTimeout(now, transition_timeout_)) {
        transition_error_active_ = true;
        const bool source_regime_still_stable =
            observed_regime == xd_uav_controller::FlightRegime::kHover ||
            observed_regime ==
                xd_uav_controller::FlightRegime::kForwardFlight;
        if (state_machine_state_ == State::kWaitState &&
            vtol_takeoff_.waitingForHover()) {
          vtol_takeoff_.fail(vtol_adapter_->transitionStatus().detail);
          transition(State::kFailsafe,
                     "VTOL起飞前转换旋翼模式超时: " +
                         vtol_adapter_->transitionStatus().detail);
        } else if (state_machine_state_ == State::kLanding) {
          vtol_landing_.fail(vtol_adapter_->transitionStatus().detail);
          transition(State::kFailsafe,
                     "VTOL降落前转换超时: " +
                         vtol_adapter_->transitionStatus().detail);
        } else if (!source_regime_still_stable) {
          transition(State::kFailsafe,
                     "VTOL转换超时且实际形态不安全: " +
                         vtol_adapter_->transitionStatus().detail);
        } else {
          ROS_ERROR_THROTTLE(
              1.0,
              "[xd_uav_control_manager] VTOL转换超时，"
              "PX4仍处于稳定源形态，保持当前backend: %s",
              vtol_adapter_->transitionStatus().detail.c_str());
        }
      }
    }
    current_control_state_ = buildControlState(now);
    state_publisher_.publish(current_control_state_);

    std::string command_reason;
    const bool command_valid =
        validCommand(now, &command_reason);
    const bool flight_state_valid =
        current_control_state_.state_valid &&
        current_control_state_.stable;
    const bool mavros_connected =
        mavrosStateFresh(now) && mavros_state_.connected;
    const bool extended_state_fresh =
        mavrosExtendedStateFresh(now);
    const bool active_inputs_healthy =
        current_control_state_.state_valid &&
        command_valid && mavros_connected;
    active_input_reason_ = inputFailureReason(
        now, command_valid, command_reason, mavros_connected);

    switch (state_machine_state_) {
      case State::kStandby:
        break;

      case State::kWaitState:
        if (!offboard_requested_) {
          transition(State::kStandby, "等待服务请求");
        } else if (vtol_adapter_ && vtol_takeoff_.waitingForHover()) {
          if (vtol_takeoff_.phase() !=
              xd_uav_control_manager::VtolTakeoffPhase::kReady) {
            break;
          }
          const bool multirotor_handover_confirmed =
              current_control_state_.flight_regime ==
                  xd_uav_controller::ControlState::REGIME_HOVER &&
              command_.flight_regime ==
                  xd_uav_controller::ControlCommand::REGIME_HOVER &&
              command_.regime_generation ==
                  current_control_state_.regime_generation &&
              command_.action_generation == action_generation_ &&
              command_.active_backend ==
                  xd_uav_controller::ControlCommand::BACKEND_MULTIROTOR;
          if (!multirotor_handover_confirmed) {
            break;
          }
          std::string reason;
          if (!commandControllerTakeoff(
                  action_generation_, pending_takeoff_altitude_,
                  xd_uav_controller::InternalCommand::Request::PHASE_START,
                  &reason)) {
            vtol_takeoff_.fail(reason);
            transition_error_active_ = true;
            transition(State::kFailsafe,
                       "VTOL旋翼后端接管后启动起飞失败: " + reason);
            break;
          }
          vtol_takeoff_.commandStarted();
          transition(State::kWaitState,
                     "multirotor backend已接管，准备OFFBOARD和解锁");
        } else if (flight_state_valid && command_valid &&
                   mavros_connected) {
          transition(State::kPrestream,
                     "状态稳定，开始预发送控制量");
        }
        break;

      case State::kPrestream:
        if (!flight_state_valid || !command_valid ||
            !mavros_connected) {
          transition(State::kWaitState,
                     "预发送期间状态或控制量失效");
          break;
        }
        publishAttitudeTarget(now);
        if ((now - state_entered_).toSec() >=
            prestream_duration_) {
          transition(State::kRequestOffboard,
                     "预发送完成，请求OFFBOARD");
        }
        break;

      case State::kRequestOffboard:
        if (!current_control_state_.state_valid ||
            !command_valid ||
            !mavros_connected) {
          transition(State::kWaitState,
                     "请求OFFBOARD期间输入失效: " +
                         active_input_reason_);
          break;
        }
        publishAttitudeTarget(now);
        if (mavros_state_.mode == offboard_mode_) {
          if (arm_requested_ && !mavros_state_.armed) {
            transition(State::kRequestArm,
                       "已进入OFFBOARD，准备解锁");
          } else if (landing_requested_) {
            transition(State::kLanding,
                       "已恢复OFFBOARD，继续降落");
          } else {
            transition(State::kActive,
                       "OFFBOARD持续维持中");
          }
        } else if ((now - state_entered_).toSec() >
                   request_timeout_) {
          transition(State::kFailsafe,
                     "进入OFFBOARD超时；PX4当前模式=" +
                         mavros_state_.mode);
        } else {
          requestOffboard(now);
        }
        break;

      case State::kRequestArm:
        if (!current_control_state_.state_valid ||
            !command_valid ||
            !mavros_connected ||
            mavros_state_.mode != offboard_mode_) {
          transition(State::kFailsafe,
                     "解锁期间输入或模式失效: " +
                         active_input_reason_);
          break;
        }
        publishAttitudeTarget(now);
        if (mavros_state_.armed) {
          transition(State::kActive,
                     "OFFBOARD已解锁，持续控制");
        } else if ((now - state_entered_).toSec() >
                   request_timeout_) {
          transition(State::kFailsafe, "解锁超时");
        } else {
          requestArm(now);
        }
        break;

      case State::kActive:
        if (!publishWithInputGrace(
                now, active_inputs_healthy)) {
          break;
        }
        if (mavros_state_.mode != offboard_mode_) {
          enterStandby(
              "检测到人工切出OFFBOARD，停止外部控制");
          break;
        }
        if (arm_requested_ && !mavros_state_.armed) {
          transition(State::kFailsafe,
                     "飞行期间意外上锁");
          break;
        }
        break;

      case State::kLanding: {
        if (mavros_state_.mode != offboard_mode_) {
          enterStandby(
              "降落期间人工切出OFFBOARD，停止外部控制");
          break;
        }
        if (landing_seen_in_air_ && !mavros_state_.armed) {
          enterStandby(
              "降落完成、螺旋桨已上锁，等待下一次服务请求");
          break;
        }

        if (vtol_adapter_ &&
            vtol_landing_.phase() ==
                xd_uav_control_manager::VtolLandingPhase::
                    kVerticalDescent &&
            !vtol_vertical_descent_commanded_) {
          const bool multirotor_handover_confirmed =
              current_control_state_.flight_regime ==
                  xd_uav_controller::ControlState::REGIME_HOVER &&
              command_.flight_regime ==
                  xd_uav_controller::ControlCommand::REGIME_HOVER &&
              command_.regime_generation ==
                  current_control_state_.regime_generation &&
              command_.active_backend ==
                  xd_uav_controller::ControlCommand::BACKEND_MULTIROTOR;
          if (!multirotor_handover_confirmed) {
            publishWithInputGrace(now, active_inputs_healthy);
            break;
          }
          std::string reason;
          if (!commandControllerLanding(
                  action_generation_, landing_return_home_,
                  xd_uav_controller::InternalCommand::Request::
                      PHASE_VERTICAL_DESCENT,
                  &reason)) {
            vtol_landing_.fail(reason);
            transition_error_active_ = true;
            transition(State::kFailsafe,
                       "VTOL悬停接管后启动垂直降落失败: " + reason);
            break;
          }
          vtol_vertical_descent_commanded_ = true;
          transition(State::kLanding,
                     "multirotor backend已接管，开始垂直降落");
        }

        if (touchdown_confirmed_) {
          if (vtol_adapter_) {
            vtol_landing_.touchdownConfirmed();
          }
          // 触地已经经过持续确认。此后不再让瞬时估计抖动
          // 恢复悬停推力，而是保持零推力直到PX4确认上锁。
          publishTouchdownTarget(now);
          const double touchdown_elapsed =
              (now - touchdown_confirmed_at_).toSec();
          if (allow_force_disarm_ &&
              touchdown_elapsed >= force_disarm_timeout_) {
            requestForceDisarm(now);
          } else if (touchdown_elapsed >=
                         touchdown_idle_duration_ &&
                     (!normal_disarm_attempted_ ||
                      !allow_force_disarm_)) {
            requestDisarm(now);
          }
          break;
        }

        if (!publishWithInputGrace(
                now, active_inputs_healthy)) {
          break;
        }
        const bool px4_reports_in_air =
            extended_state_fresh &&
            mavros_extended_state_.landed_state ==
                mavros_msgs::ExtendedState::
                    LANDED_STATE_IN_AIR;
        const bool px4_reports_landed =
            extended_state_fresh &&
            mavros_extended_state_.landed_state ==
                mavros_msgs::ExtendedState::
                    LANDED_STATE_ON_GROUND;
        const bool fixedwing_backend_active =
            last_active_backend_ ==
            xd_uav_controller::BackendId::kFixedWing;
        const bool fixedwing_touchdown_speed_safe =
            !fixedwing_backend_active ||
            (std::isfinite(
                 current_control_state_.groundspeed) &&
             current_control_state_.groundspeed <=
                 fixedwing_touchdown_max_groundspeed_);
        const bool controller_reports_landing_complete =
            command_.action_generation == action_generation_ &&
            command_.action_status ==
                xd_uav_controller::ControlCommand::ACTION_SUCCEEDED;
        const bool touchdown_detected =
            (controller_reports_landing_complete ||
             (command_.action_generation == 0U &&
              command_.landing_touchdown) || px4_reports_landed) &&
            fixedwing_touchdown_speed_safe;
        if (fixedwing_backend_active &&
            (controller_reports_landing_complete ||
             (command_.action_generation == 0U &&
              command_.landing_touchdown) ||
             px4_reports_landed) &&
            !fixedwing_touchdown_speed_safe) {
          ROS_WARN_THROTTLE(
              1.0,
              "[xd_uav_control_manager] 固定翼触地信号已出现，"
              "但地速%.2fm/s仍高于%.2fm/s，继续滑跑且不解锁",
              current_control_state_.groundspeed,
              fixedwing_touchdown_max_groundspeed_);
        }
        if (px4_reports_in_air &&
            !controller_reports_landing_complete &&
            !command_.landing_touchdown) {
          landing_seen_in_air_ = true;
        }
        if (landing_seen_in_air_ && touchdown_detected) {
          if (vtol_adapter_) {
            vtol_landing_.touchdownCandidate();
          }
          if (landed_since_.isZero()) {
            landed_since_ = now;
            ROS_INFO(
                "[xd_uav_control_manager] 检测到触地，"
                "正在进行持续确认");
          }
          if ((now - landed_since_).toSec() >=
              landed_confirm_duration_) {
            touchdown_confirmed_ = true;
            touchdown_confirmed_at_ = now;
            normal_disarm_attempted_ = false;
            force_disarm_requested_ = false;
            last_request_ = ros::Time();
            publishTouchdownTarget(now);
            ROS_WARN(
                "[xd_uav_control_manager] 触地已确认，"
                "输出零推力并准备上锁");
          }
        } else {
          landed_since_ = ros::Time();
        }
        break;
      }

      case State::kFailsafe:
        // 停止发布外部控制量，让PX4执行配置好的OFFBOARD-loss策略。
        break;
    }
    publishDiagnostics(now, command_reason);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber odometry_subscriber_;
  ros::Subscriber acceleration_subscriber_;
  ros::Subscriber imu_subscriber_;
  ros::Subscriber estimator_status_subscriber_;
  ros::Subscriber airspeed_subscriber_;
  ros::Subscriber mavros_state_subscriber_;
  ros::Subscriber mavros_extended_state_subscriber_;
  ros::Subscriber command_subscriber_;
  ros::Publisher state_publisher_;
  ros::Publisher status_publisher_;
  ros::Publisher diagnostics_publisher_;
  ros::Publisher attitude_target_publisher_;
  ros::ServiceServer offboard_server_;
  ros::ServiceServer cancel_offboard_server_;
  ros::ServiceServer takeoff_server_;
  ros::ServiceServer land_server_;
  ros::ServiceServer land_home_server_;
  ros::ServiceServer cancel_land_server_;
  ros::ServiceServer reset_failsafe_server_;
  ros::ServiceServer set_flight_regime_server_;
  ros::ServiceClient set_mode_client_;
  ros::ServiceClient arming_client_;
  ros::ServiceClient command_long_client_;
  ros::ServiceClient vtol_transition_client_;
  ros::ServiceClient controller_internal_command_client_;
  ros::Timer timer_;

  nav_msgs::Odometry odometry_;
  geometry_msgs::AccelWithCovarianceStamped acceleration_;
  sensor_msgs::Imu imu_;
  xd_uav_state_estimators::EstimatorStatus estimator_status_;
  mavros_msgs::VFR_HUD airspeed_;
  mavros_msgs::State mavros_state_;
  mavros_msgs::ExtendedState mavros_extended_state_;
  mavros_msgs::AttitudeTarget last_safe_target_;
  xd_uav_controller::ControlCommand command_;
  xd_uav_controller::ControlState current_control_state_;
  ros::Time odometry_receive_;
  ros::Time acceleration_receive_;
  ros::Time imu_receive_;
  ros::Time estimator_status_receive_;
  ros::Time airspeed_receive_;
  ros::Time mavros_state_receive_;
  ros::Time mavros_extended_state_receive_;
  ros::Time command_receive_;
  ros::Time valid_since_;
  ros::Time state_entered_;
  ros::Time last_request_;
  ros::Time invalid_since_;
  ros::Time landed_since_;
  ros::Time touchdown_confirmed_at_;

  bool have_odometry_{false};
  bool have_acceleration_{false};
  bool have_imu_{false};
  bool have_estimator_status_{false};
  bool have_airspeed_{false};
  bool have_mavros_state_{false};
  bool have_mavros_extended_state_{false};
  bool have_command_{false};
  bool have_last_safe_target_{false};
  bool offboard_requested_{false};
  bool arm_requested_{false};
  bool landing_requested_{false};
  bool landing_return_home_{false};
  bool landing_seen_in_air_{false};
  bool touchdown_confirmed_{false};
  bool normal_disarm_attempted_{false};
  bool force_disarm_requested_{false};
  bool last_mode_request_accepted_{false};
  uint32_t mode_request_attempts_{0};
  State state_machine_state_{State::kStandby};
  std::string state_reason_;
  std::string active_input_reason_{"尚未检查"};

  std::string vehicle_type_;
  xd_uav_controller::AirframeType airframe_type_{
      xd_uav_controller::AirframeType::kMultirotor};
  std::unique_ptr<xd_uav_control_manager::VehicleAdapter>
      vehicle_adapter_;
  xd_uav_control_manager::VtolVehicleAdapter* vtol_adapter_{nullptr};
  xd_uav_control_manager::VtolTakeoffCoordinator vtol_takeoff_;
  xd_uav_control_manager::VtolLandingCoordinator vtol_landing_;
  bool transition_error_active_{false};
  bool vtol_vertical_descent_commanded_{false};
  double pending_takeoff_altitude_{0.0};
  xd_uav_controller::BackendId last_active_backend_{
      xd_uav_controller::BackendId::kNone};
  uint32_t regime_generation_{0U};
  uint32_t action_generation_{0U};
  uint8_t vehicle_type_id_{
      xd_uav_controller::ControlState::VEHICLE_MULTIROTOR};
  double setpoint_rate_{100.0};
  double prestream_duration_{1.5};
  double request_retry_interval_{1.0};
  double request_timeout_{10.0};
  std::string offboard_mode_{"OFFBOARD"};
  std::string cancel_mode_{"POSCTL"};
  std::string vtol_transition_service_{"mavros/cmd/vtol_transition"};
  double transition_timeout_{10.0};
  bool vtol_require_offboard_{true};
  bool vtol_require_armed_{true};
  bool vtol_require_extended_state_{true};
  double vtol_minimum_forward_airspeed_{0.0};

  double odometry_timeout_{0.20};
  double imu_timeout_{0.50};
  double acceleration_timeout_{0.20};
  double airspeed_timeout_{0.30};
  double airspeed_negative_tolerance_{3.0};
  double command_timeout_{0.20};
  double estimator_status_timeout_{0.30};
  double mavros_state_timeout_{1.0};
  double extended_state_timeout_{1.0};
  double future_stamp_tolerance_{0.05};
  double stable_duration_{1.5};
  double invalid_grace_duration_{0.5};
  double landed_confirm_duration_{0.5};
  double touchdown_idle_duration_{0.75};
  double force_disarm_timeout_{2.0};
  bool allow_force_disarm_{true};
  bool require_acceleration_{true};
  bool require_localization_{false};
  double minimum_groundspeed_for_course_{0.5};
  double fixedwing_touchdown_max_groundspeed_{2.0};
};

int main(int argc, char** argv) {
  std::setlocale(LC_ALL, "");
  ros::init(argc, argv, "control_manager");
  try {
    ControlManagerNode node;
    ros::spin();
  } catch (const std::exception& exception) {
    ROS_FATAL("[xd_uav_control_manager] 启动失败: %s",
              exception.what());
    return 1;
  }
  return 0;
}
