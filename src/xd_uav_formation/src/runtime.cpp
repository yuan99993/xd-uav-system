#include <xd_uav_formation/runtime.hpp>

#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <xmlrpcpp/XmlRpcException.h>
#include <xmlrpcpp/XmlRpcValue.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>

namespace xd_uav_formation {
namespace {

Vec3 pointVec(const geometry_msgs::Point& point) {
  return {point.x, point.y, point.z};
}
Vec3 vectorVec(const geometry_msgs::Vector3& vector) {
  return {vector.x, vector.y, vector.z};
}
geometry_msgs::Point pointMsg(const Vec3& value) {
  geometry_msgs::Point point;
  point.x = value[0]; point.y = value[1]; point.z = value[2];
  return point;
}
geometry_msgs::Vector3 vectorMsg(const Vec3& value) {
  geometry_msgs::Vector3 vector;
  vector.x = value[0]; vector.y = value[1]; vector.z = value[2];
  return vector;
}
double clampValue(double value, double low, double high) {
  return std::max(low, std::min(high, value));
}
std::string cleanFrame(std::string frame) {
  while (!frame.empty() && frame.front() == '/') frame.erase(frame.begin());
  while (!frame.empty() && frame.back() == '/') frame.pop_back();
  return frame;
}
bool blockingAction(const xd_uav_controller::ControlState& state) {
  return xd_uav_formation::blockingAction(state.vehicle_action);
}
std::string nodeNameForMode(const std::string& mode) {
  return mode == "consensus" ? "formation_consensus" : "formation_follower";
}
XmlRpc::XmlRpcValue acquisitionPlanValue(const AcquisitionPlan& plan) {
  XmlRpc::XmlRpcValue parameter;
  parameter["members"].setSize(static_cast<int>(plan.members.size()));
  for (std::size_t i = 0; i < plan.members.size(); ++i)
    parameter["members"][static_cast<int>(i)] = plan.members[i];
  parameter["shape"] = plan.shape;
  parameter["spacing"] = plan.spacing;
  parameter["stamp"] = plan.stamp.toSec();
  auto points = [](const std::vector<Vec3>& values) {
    XmlRpc::XmlRpcValue result;
    result.setSize(static_cast<int>(values.size()));
    for (std::size_t i = 0; i < values.size(); ++i) {
      XmlRpc::XmlRpcValue row;
      row.setSize(3);
      for (int j = 0; j < 3; ++j) row[j] = values[i][j];
      result[static_cast<int>(i)] = row;
    }
    return result;
  };
  parameter["initial_positions"] = points(plan.initial_positions);
  parameter["slots"] = points(plan.slots);
  XmlRpc::XmlRpcValue tiers;
  tiers.setSize(static_cast<int>(plan.tiers.size()));
  for (std::size_t i = 0; i < plan.tiers.size(); ++i)
    tiers[static_cast<int>(i)] = plan.tiers[i];
  parameter["tiers"] = tiers;
  parameter["heading"] = plan.heading;
  parameter["altitude"] = plan.altitude;
  return parameter;
}

}  // namespace

Follower::Follower(ros::NodeHandle nh, const Config& config, std::string member,
                   std::shared_ptr<SharedData> shared,
                   std::shared_ptr<const FormationGeometry> geometry,
                   std::shared_ptr<tf2_ros::Buffer> tf_buffer,
                   bool shared_inputs)
    : nh_(std::move(nh)), config_(config), member_(std::move(member)),
      shared_(std::move(shared)), geometry_(std::move(geometry)),
      tf_buffer_(std::move(tf_buffer)), shared_inputs_(shared_inputs) {
  if (config_.mode != "leader_follow" && config_.mode != "consensus") {
    throw std::invalid_argument("mode must be leader_follow or consensus");
  }
  if (member_.empty() || member_.find('/') != std::string::npos) {
    throw std::invalid_argument("member must be a simple UAV namespace name");
  }
  node_name_ = "/" + member_ + "/" + nodeNameForMode(config_.mode);
  service_name_ = node_name_ + "/set_enabled";
  if (config_.mode == "leader_follow" &&
      (config_.leader_name.empty() || config_.leader_name == member_)) {
    throw std::invalid_argument("use a valid UAV and a different leader");
  }
  if (config_.team_id.empty() || config_.team_id.find('/') != std::string::npos) {
    throw std::invalid_argument("team_id must be a simple name");
  }
  if (config_.members.empty() ||
      std::set<std::string>(config_.members.begin(), config_.members.end()).size() !=
          config_.members.size() ||
      std::find(config_.members.begin(), config_.members.end(), member_) ==
          config_.members.end()) {
    throw std::invalid_argument("formation members must be unique UAV names");
  }
  if (!geometry_) throw std::invalid_argument("formation geometry is missing");
  if (config_.mode == "consensus") {
    if (config_.members.size() < 2 || config_.common_frame.empty()) {
      throw std::invalid_argument("consensus requires members and common_frame");
    }
    offset_ = validateOffset(geometry_->slots.at(static_cast<std::size_t>(
        std::find(config_.members.begin(), config_.members.end(), member_) -
        config_.members.begin())), config_.minimum_slot_distance);
  } else {
    auto offset = config_.offsets.find(member_);
    if (offset != config_.offsets.end()) {
      offset_ = validateOffset(offset->second, config_.minimum_slot_distance);
    } else if (config_.shape != "manual") {
      if (config_.slot_index < 1 || static_cast<std::size_t>(config_.slot_index) >
          geometry_->slots.size()) {
        throw std::invalid_argument("slot_index is outside formation");
      }
      offset_ = validateOffset(geometry_->slots[static_cast<std::size_t>(config_.slot_index - 1)],
                               config_.minimum_slot_distance);
    } else {
      offset_ = validateOffset({-4.0, 0.0, 0.0},
                               config_.minimum_slot_distance);
    }
  }
  if (!tf_buffer_) {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(
        ros::Duration(config_.tf_buffer_cache_time));
    owned_tf_listener_.reset(new tf2_ros::TransformListener(*tf_buffer_));
  }
  if (!shared_) shared_ = std::make_shared<SharedData>();

  const auto require_finite_positive = [](double value, const char* name,
                                           bool allow_zero = false) {
    if (!std::isfinite(value) || (allow_zero ? value < 0.0 : value <= 0.0)) {
      throw std::invalid_argument(std::string("invalid ") + name);
    }
  };
  require_finite_positive(config_.rate, "rate");
  require_finite_positive(config_.state_timeout, "state_timeout");
  require_finite_positive(config_.mavros_timeout, "mavros_timeout");
  require_finite_positive(config_.tf_max_age, "tf_max_age");
  require_finite_positive(config_.future_stamp_tolerance,
                           "future_stamp_tolerance", true);
  require_finite_positive(config_.tf_lookup_timeout, "tf_lookup_timeout", true);
  require_finite_positive(config_.tf_buffer_cache_time, "tf_buffer_cache_time");
  require_finite_positive(config_.external_quiet_time,
                           "external_quiet_time", true);
  require_finite_positive(config_.fixedwing_minimum_slot_distance,
                           "fixedwing_minimum_slot_distance");
  require_finite_positive(config_.fixedwing_min_speed, "fixedwing_min_speed");
  require_finite_positive(config_.fixedwing_max_catchup_speed,
                           "fixedwing_max_catchup_speed", true);
  require_finite_positive(config_.fixedwing_min_groundspeed_for_course,
                           "fixedwing_min_groundspeed_for_course");
  require_finite_positive(config_.multirotor_max_speed,
                           "multirotor_max_speed");
  require_finite_positive(config_.catchup_gain, "catchup_gain", true);
  require_finite_positive(config_.catchup_ki, "catchup_ki", true);
  require_finite_positive(config_.catchup_kd, "catchup_kd", true);
  require_finite_positive(config_.catchup_deadband, "catchup_deadband", true);
  require_finite_positive(config_.catchup_integral_limit,
                           "catchup_integral_limit", true);
  require_finite_positive(config_.catchup_derivative_filter_time_constant,
                           "catchup_derivative_filter_time_constant", true);
  require_finite_positive(config_.multirotor_max_catchup_speed,
                           "multirotor_max_catchup_speed", true);
  if (config_.mode == "consensus") {
    require_finite_positive(config_.consensus_gain, "consensus_gain", true);
    require_finite_positive(config_.convergence_time, "convergence_time");
    require_finite_positive(config_.max_consensus_velocity,
                             "max_consensus_velocity", true);
    require_finite_positive(config_.acquire_consensus_gain,
                             "acquire_consensus_gain", true);
    require_finite_positive(config_.acquire_max_consensus_velocity,
                             "acquire_max_consensus_velocity", true);
    require_finite_positive(config_.acquire_max_speed_adjustment,
                             "acquire_max_speed_adjustment", true);
    require_finite_positive(config_.formation_heading_rate_limit,
                             "formation_heading_rate_limit");
    require_finite_positive(config_.formation_speed_slew_rate,
                             "formation_speed_slew_rate");
    require_finite_positive(config_.formation_course_slew_rate,
                             "formation_course_slew_rate");
    require_finite_positive(config_.route_correction_gain,
                             "route_correction_gain", true);
    require_finite_positive(config_.max_route_correction_velocity,
                             "max_route_correction_velocity", true);
    require_finite_positive(config_.acquire_tolerance, "acquire_tolerance");
    require_finite_positive(config_.acquire_height_tolerance,
                             "acquire_height_tolerance");
    require_finite_positive(config_.acquire_vertical_spacing,
                             "acquire_vertical_spacing");
    require_finite_positive(config_.acquire_timeout, "acquire_timeout");
  }
  status_heartbeat_period_ = std::min(1.0, 0.5 * config_.mavros_timeout);
  idle_center_period_ = std::min(1.0, 0.5 * config_.state_timeout);

  std::string reference_topic = config_.reference_topic;
  std::string path_topic = config_.path_topic;
  std::string trajectory_topic = config_.trajectory_topic;
  auto resolve = [this](const std::string& topic) {
    if (!topic.empty() && topic.front() == '/') return topic;
    return "/" + member_ + "/" + topic;
  };
  reference_topic = resolve(reference_topic);
  path_topic = resolve(path_topic);
  trajectory_topic = resolve(trajectory_topic);
  std::string cancel_topic = config_.team_cancel_topic;
  const std::string token = "{team_id}";
  for (std::size_t at = cancel_topic.find(token); at != std::string::npos;
       at = cancel_topic.find(token, at + config_.team_id.size())) {
    cancel_topic.replace(at, token.size(), config_.team_id);
  }
  cancel_topic = resolve(cancel_topic);

  reference_pub_ = nh_.advertise<mavros_msgs::PositionTarget>(reference_topic, 1);
  path_pub_ = nh_.advertise<nav_msgs::Path>(path_topic, 1);
  trajectory_pub_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
      trajectory_topic, 1);
  team_cancel_pub_ = nh_.advertise<std_msgs::String>(cancel_topic, 1);
  status_pub_ = nh_.advertise<std_msgs::String>(node_name_ + "/status", 1, true);
  if (config_.mode == "consensus") {
    const std::string center_topic = "/formation/" + config_.team_id + "/" +
                                     member_ + "/center";
    center_pub_ = nh_.advertise<geometry_msgs::PointStamped>(center_topic, 1);
  }
  if (!shared_inputs) {
    subscribers_.push_back(nh_.subscribe<xd_uav_controller::ControlState>(
        "/" + member_ + "/control_manager/state", 1,
        [this](const xd_uav_controller::ControlState::ConstPtr& msg) {
          stateInput(*msg, ros::Time::now());
        }));
    subscribers_.push_back(nh_.subscribe<mavros_msgs::State>(
        "/" + member_ + "/mavros/state", 1,
        [this](const mavros_msgs::State::ConstPtr& msg) {
          mavrosInput(*msg, ros::Time::now());
        }));
    subscribers_.push_back(nh_.subscribe<std_msgs::String>(
        cancel_topic, 1,
        [this](const std_msgs::String::ConstPtr& msg) { teamCancelInput(*msg); }));
  }
  subscribers_.push_back(nh_.subscribe<mavros_msgs::PositionTarget>(
      reference_topic, 1,
      [this](const ros::MessageEvent<const mavros_msgs::PositionTarget>& event) {
        externalReference(event);
      }));
  subscribers_.push_back(nh_.subscribe<nav_msgs::Path>(
      path_topic, 1, [this](const nav_msgs::Path::ConstPtr& msg) {
        externalPath(msg);
      }));
  subscribers_.push_back(nh_.subscribe<trajectory_msgs::MultiDOFJointTrajectory>(
      trajectory_topic, 1,
      [this](const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr& msg) {
        externalTrajectory(msg);
      }));
  subscribers_.push_back(nh_.subscribe<std_msgs::String>(
      "/" + member_ + "/" + config_.preempt_topic, 10,
      [this](const std_msgs::String::ConstPtr& msg) { preempt(msg); }));
  subscribers_.push_back(nh_.subscribe<xd_uav_task_execute::ExecuteTaskActionGoal>(
      "/" + member_ + "/" + config_.task_goal_topic, 10,
      [this](const xd_uav_task_execute::ExecuteTaskActionGoal::ConstPtr& msg) {
        taskGoal(msg);
      }));
  if (!shared_inputs && config_.mode == "leader_follow") {
    subscribers_.push_back(nh_.subscribe<xd_uav_controller::ControlState>(
        "/" + config_.leader_name + "/control_manager/state", 1,
        [this](const xd_uav_controller::ControlState::ConstPtr& msg) {
          leaderStateInput(*msg, ros::Time::now());
        }));
    subscribers_.push_back(nh_.subscribe<mavros_msgs::State>(
        "/" + config_.leader_name + "/mavros/state", 1,
        [this](const mavros_msgs::State::ConstPtr& msg) {
          leaderMavrosInput(*msg, ros::Time::now());
        }));
    subscribers_.push_back(nh_.subscribe<xd_uav_task_execute::ExecuteTaskActionGoal>(
        "/" + config_.leader_name + "/task_execute/execute/goal", 10,
        [this](const xd_uav_task_execute::ExecuteTaskActionGoal::ConstPtr& msg) {
          leaderTaskInput(*msg);
        }));
    subscribers_.push_back(nh_.subscribe<std_msgs::String>(
        "/" + config_.leader_name + "/formation/preempt", 10,
        [this](const std_msgs::String::ConstPtr& msg) {
          leaderPreemptInput(*msg);
        }));
  }
  if (!shared_inputs && config_.mode == "consensus") {
    for (const auto& other : config_.members) {
      if (other == member_) continue;
      subscribers_.push_back(nh_.subscribe<xd_uav_controller::ControlState>(
          "/" + other + "/control_manager/state", 1,
          [this, other](const xd_uav_controller::ControlState::ConstPtr& msg) {
            peerStateInput(*msg, other, ros::Time::now());
          }));
      subscribers_.push_back(nh_.subscribe<mavros_msgs::State>(
          "/" + other + "/mavros/state", 1,
          [this, other](const mavros_msgs::State::ConstPtr& msg) {
            peerMavrosInput(*msg, other, ros::Time::now());
          }));
      subscribers_.push_back(nh_.subscribe<geometry_msgs::PointStamped>(
          "/formation/" + config_.team_id + "/" + other + "/center", 1,
          [this, other](const geometry_msgs::PointStamped::ConstPtr& msg) {
            peerCenterInput(*msg, other, ros::Time::now());
          }));
      subscribers_.push_back(nh_.subscribe<std_msgs::String>(
          "/" + other + "/formation_consensus/status", 1,
          [this, other](const std_msgs::String::ConstPtr& msg) {
            peerStatusInput(*msg, other, ros::Time::now());
          }));
    }
  }
  service_ = nh_.advertiseService(service_name_, &Follower::setEnabledService,
                                  this);
  timer_ = nh_.createTimer(ros::Duration(1.0 / config_.rate),
                           &Follower::tick, this);
  publishStatus();
  ROS_INFO("[xd_uav_formation] C++ mode=%s member=%s state_timeout=%.2fs",
           config_.mode.c_str(), member_.c_str(), config_.state_timeout);
}

Follower::~Follower() {
  timer_.stop();
  service_.shutdown();
  for (auto& subscriber : subscribers_) subscriber.shutdown();
  reference_pub_.shutdown(); path_pub_.shutdown(); trajectory_pub_.shutdown();
  team_cancel_pub_.shutdown(); status_pub_.shutdown(); center_pub_.shutdown();
}

void Follower::stateInput(const xd_uav_controller::ControlState& state,
                          const ros::Time& received) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    own_state_ = state;
    own_state_received_ = received;
    has_own_state_received_ = true;
    if (enabled_ && blockingAction(state)) beginDropout("follower_flight_action");
  }
  if (shared_ && !shared_inputs_) {
    std::lock_guard<std::mutex> lock(shared_->mutex);
    shared_->states[member_] = state;
    shared_->state_received[member_] = received;
    shared_->markChanged();
  }
}

void Follower::mavrosInput(const mavros_msgs::State& state,
                           const ros::Time& received) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    own_mavros_ = state;
    own_mavros_received_ = received;
    has_own_mavros_received_ = true;
    if (enabled_ && (!state.armed || state.mode != "OFFBOARD")) {
      beginDropout("follower_left_offboard");
    }
  }
  if (shared_ && !shared_inputs_) {
    std::lock_guard<std::mutex> lock(shared_->mutex);
    shared_->mavros[member_] = state;
    shared_->mavros_received[member_] = received;
    shared_->markChanged();
  }
}

void Follower::leaderStateInput(const xd_uav_controller::ControlState& state,
                                const ros::Time& received) {
  std::lock_guard<std::mutex> lock(mutex_);
  leader_state_ = state;
  leader_state_received_ = received;
  has_leader_state_received_ = true;
  if (enabled_ && blockingAction(state)) stop("leader_flight_action", true, false);
}

void Follower::leaderMavrosInput(const mavros_msgs::State& state,
                                 const ros::Time& received) {
  std::lock_guard<std::mutex> lock(mutex_);
  leader_mavros_ = state;
  leader_mavros_received_ = received;
  has_leader_mavros_received_ = true;
  if (enabled_ && !state.armed) stop("leader_disarmed", true, false);
}

void Follower::peerStateInput(const xd_uav_controller::ControlState& state,
                              const std::string& member,
                              const ros::Time& received) {
  std::lock_guard<std::mutex> lock(shared_->mutex);
  shared_->states[member] = state;
  shared_->state_received[member] = received;
  shared_->markChanged();
}

void Follower::peerMavrosInput(const mavros_msgs::State& state,
                               const std::string& member,
                               const ros::Time& received) {
  std::lock_guard<std::mutex> lock(shared_->mutex);
  shared_->mavros[member] = state;
  shared_->mavros_received[member] = received;
  shared_->markChanged();
}

void Follower::peerCenterInput(const geometry_msgs::PointStamped& center,
                               const std::string& member,
                               const ros::Time& received) {
  std::lock_guard<std::mutex> lock(shared_->mutex);
  shared_->centers[member] = center;
  shared_->center_received[member] = received;
  shared_->markChanged();
}

void Follower::peerStatusInput(const std_msgs::String& status,
                               const std::string& member,
                               const ros::Time& received) {
  {
    std::lock_guard<std::mutex> lock(shared_->mutex);
    if (!shared_inputs_) {
      shared_->statuses[member] = status.data;
      shared_->status_received[member] = received;
      shared_->markChanged();
    }
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (enabled_ && status.data.rfind("inactive:dropout", 0) == 0)
    markPeerDropped(member, status.data);
}

void Follower::teamCancelInput(const std_msgs::String& message) {
  if (message.data.rfind(node_name_ + ":", 0) == 0) return;
  std::lock_guard<std::mutex> lock(mutex_);
  if (!enabled_) return;
  const ros::Time now = ros::Time::now();
  const bool external_recent = last_external_reference_ &&
      (now - *last_external_reference_).toSec() < config_.external_quiet_time;
  stop("team_cancel:" + message.data, !external_recent, false);
}

void Follower::leaderTaskInput(
    const xd_uav_task_execute::ExecuteTaskActionGoal& goal) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (enabled_) stop("leader_task_goal:" + std::to_string(goal.goal.task_id),
                     true, false);
}

void Follower::leaderPreemptInput(const std_msgs::String& message) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (enabled_) stop("leader_preempt:" + message.data.substr(0, 128), true, false);
}

void Follower::clearAcquisitionPlan() {
  std::lock_guard<std::mutex> lock(shared_->acquisition_plan_mutex);
  shared_->acquisition_plan.reset();
}

void Follower::rememberOwnReference(
    const mavros_msgs::PositionTarget& message) {
  std::lock_guard<std::mutex> lock(own_reference_mutex_);
  own_reference_stamps_.insert(message.header.stamp.toNSec());
  while (own_reference_stamps_.size() > 128)
    own_reference_stamps_.erase(own_reference_stamps_.begin());
}

bool Follower::isOwnReference(
    const mavros_msgs::PositionTarget& message) const {
  std::lock_guard<std::mutex> lock(own_reference_mutex_);
  return own_reference_stamps_.count(message.header.stamp.toNSec()) != 0;
}

void Follower::externalReference(
    const ros::MessageEvent<const mavros_msgs::PositionTarget>& event) {
  const auto message = event.getConstMessage();
  const auto header = event.getConnectionHeaderPtr();
  std::string caller;
  if (header) {
    const auto it = header->find("callerid");
    if (it != header->end()) caller = it->second;
  }
  if (caller.empty()) {
    // Intra-process roscpp delivery can omit connection metadata. Match the
    // header stamp against recent outputs so external headerless publishers
    // still preempt the formation.
    if (isOwnReference(*message)) return;
    caller = "unknown";
  }
  if (caller == ros::this_node::getName()) return;
  std::lock_guard<std::mutex> lock(mutex_);
  last_external_reference_ = ros::Time::now();
  if (enabled_) beginDropout("external_setpoint_from:" + caller);
  (void)message;
}

void Follower::externalPath(const nav_msgs::Path::ConstPtr&) {
  std::lock_guard<std::mutex> lock(mutex_);
  last_external_reference_ = ros::Time::now();
  if (enabled_) beginDropout("external_path");
}

void Follower::externalTrajectory(
    const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr&) {
  std::lock_guard<std::mutex> lock(mutex_);
  last_external_reference_ = ros::Time::now();
  if (enabled_) beginDropout("external_trajectory");
}

void Follower::preempt(const std_msgs::String::ConstPtr& message) {
  std::lock_guard<std::mutex> lock(mutex_);
  last_external_reference_ = ros::Time::now();
  if (enabled_) beginDropout("preempt:" + message->data.substr(0, 128));
}

void Follower::taskGoal(
    const xd_uav_task_execute::ExecuteTaskActionGoal::ConstPtr& goal) {
  std::lock_guard<std::mutex> lock(mutex_);
  last_external_reference_ = ros::Time::now();
  if (enabled_) beginDropout("task_goal:" + std::to_string(goal->goal.task_id));
}

void Follower::publishStatus() {
  const std::string status = std::string(enabled_ ? "active:" : "inactive:") + reason_;
  if (shared_) {
    std::lock_guard<std::mutex> lock(shared_->mutex);
    shared_->statuses[member_] = status;
    shared_->status_received[member_] = ros::Time::now();
    shared_->markChanged();
  }
  std_msgs::String message;
  message.data = status;
  status_pub_.publish(message);
  last_status_publish_ = ros::Time::now();
}

void Follower::markPeerDropped(const std::string& member,
                               const std::string& reason) {
  if (member == member_ || dropped_members_.count(member)) return;
  dropped_members_.insert(member);
  active_members_dirty_ = true;
  ROS_WARN("[xd_uav_formation] excluding %s after %s",
           member.c_str(), reason.c_str());
  if (config_.mode == "consensus" && enabled_) {
    if (!acquire_stage_ || *acquire_stage_ != "following") {
      acquire_stage_ = "following";
      acquire_started_.reset();
    }
    reason_ = "following_degraded:" + member + ":" + reason;
    publishStatus();
  }
}

void Follower::beginDropout(const std::string& reason) {
  const bool was_enabled = enabled_;
  enabled_ = false;
  acquire_stage_.reset();
  acquire_started_.reset();
  dropped_members_.insert(member_);
  active_members_dirty_ = true;
  reason_ = "dropout:" + reason;
  publishStatus();
  if (was_enabled) {
    ROS_WARN("[xd_uav_formation] member %s left the formation: %s",
             member_.c_str(), reason.c_str());
  }
}

void Follower::stop(const std::string& reason, bool hold, bool broadcast) {
  const bool was_enabled = enabled_;
  enabled_ = false;
  acquire_stage_.reset();
  acquire_started_.reset();
  reason_ = reason;
  if (hold) publishHold();
  publishStatus();
  if (was_enabled && broadcast) {
    std_msgs::String message;
    message.data = node_name_ + ":" + reason;
    team_cancel_pub_.publish(message);
  }
  ROS_INFO("[xd_uav_formation] stopped: %s", reason.c_str());
}

void Follower::publishHold() {
  if (!own_state_) return;
  const auto& own = *own_state_;
  if (!fresh(own.header, true, own_state_received_, ros::Time::now()) ||
      !own.state_valid || own.vehicle_type !=
          xd_uav_controller::ControlState::VEHICLE_MULTIROTOR ||
      !own_mavros_ || !own_mavros_->armed || own_mavros_->mode != "OFFBOARD" ||
      blockingAction(own) || cleanFrame(own.header.frame_id).empty()) return;
  const Vec3 position = pointVec(own.position_odom);
  geometry_msgs::PoseStamped pose;
  pose.pose.orientation = own.orientation_odom_body;
  double heading = 0.0;
  try { heading = yaw(pose); } catch (const std::exception&) { return; }
  const auto target = positionTarget(position, heading, ros::Time::now(),
                                     std::nullopt, own.header.frame_id, own);
  rememberOwnReference(target);
  reference_pub_.publish(target);
}

void Follower::refreshMembership(const ros::Time& now,
                                 const StateSnapshot& input) {
  if (config_.mode != "consensus" || !enabled_ || !own_state_) return;
  for (const auto& name : config_.members) {
    if (name == member_ || dropped_members_.count(name)) continue;
    const auto status = input.statuses.find(name);
    if (status != input.statuses.end() &&
        status->second.rfind("inactive:dropout", 0) == 0) {
      markPeerDropped(name, status->second);
      continue;
    }
    const auto state_it = input.states.find(name);
    const auto state_time = input.state_received.find(name);
    const auto mavros_it = input.mavros.find(name);
    const auto mavros_time = input.mavros_received.find(name);
    std::string error;
    if (state_it == input.states.end() || state_time == input.state_received.end() ||
        !fresh(state_it->second.header, true, state_time->second, now)) {
      error = "state_stale";
    } else {
      const auto& state = state_it->second;
      if (!state.state_valid) error = "state_invalid";
      else if (state.header.frame_id.empty()) error = "state_frame_missing";
      else if (state.transition_pending) error = "transition_pending";
      else if (blockingAction(state)) error = "flight_action";
      else if (state.airframe_type != own_state_->airframe_type)
        error = "airframe_changed";
      else if (state.vehicle_type != own_state_->vehicle_type ||
               state.flight_regime != own_state_->flight_regime)
        error = "flight_regime_changed";
    }
    if (error.empty()) {
      if (mavros_it == input.mavros.end() ||
          mavros_time == input.mavros_received.end() ||
          (now - mavros_time->second).toSec() > config_.mavros_timeout ||
          !mavros_it->second.armed || mavros_it->second.mode != "OFFBOARD") {
        error = "left_offboard";
      }
    }
    if (error.empty()) {
      const auto center = input.centers.find(name);
      const auto received = input.center_received.find(name);
      if (center == input.centers.end() || received == input.center_received.end() ||
          !fresh(center->second.header, true, received->second, now)) {
        error = "center_stale";
      } else if (cleanFrame(center->second.header.frame_id) != config_.common_frame) {
        error = "center_frame_mismatch";
      }
    }
    if (!error.empty()) markPeerDropped(name, error);
  }
}

void Follower::resumeDegradedAcquisition() {
  if (config_.mode == "consensus" && enabled_ && !dropped_members_.empty() &&
      acquire_stage_ && *acquire_stage_ != "following") {
    acquire_stage_ = "following";
    acquire_started_.reset();
    reason_ = "following_degraded";
    publishStatus();
  }
}

std::optional<std::string> Follower::readiness(
    const ros::Time& now, const StateSnapshot& input) const {
  if (!own_state_ || !has_own_state_received_ ||
      !fresh(own_state_->header, true, own_state_received_, now)) {
    return "follower_state_stale";
  }
  const auto& own = *own_state_;
  if (!own.state_valid) return "state_invalid";
  if (own.airframe_type != xd_uav_controller::ControlState::AIRFRAME_MULTIROTOR &&
      own.airframe_type != xd_uav_controller::ControlState::AIRFRAME_FIXED_WING &&
      own.airframe_type != xd_uav_controller::ControlState::AIRFRAME_VTOL &&
      own.airframe_type != xd_uav_controller::ControlState::AIRFRAME_TILTROTOR) {
    return "unsupported_airframe";
  }
  if (own.transition_pending) return "transition_pending";
  if (own.vehicle_type == xd_uav_controller::ControlState::VEHICLE_MULTIROTOR) {
    if (own.flight_regime != xd_uav_controller::ControlState::REGIME_HOVER)
      return "hover_regime_required";
  } else if (own.vehicle_type ==
             xd_uav_controller::ControlState::VEHICLE_FIXEDWING) {
    if (own.flight_regime !=
        xd_uav_controller::ControlState::REGIME_FORWARD_FLIGHT)
      return "forward_flight_required";
    const double offset_length = std::sqrt(offset_[0] * offset_[0] +
        offset_[1] * offset_[1] + offset_[2] * offset_[2]);
    if (offset_length < config_.fixedwing_minimum_slot_distance)
      return "fixedwing_slot_too_close";
  } else {
    return "unsupported_vehicle_type";
  }
  if (blockingAction(own)) return "follower_flight_action";
  if (own.header.frame_id.empty()) return "state_frame_missing";
  if (!own_mavros_ || !has_own_mavros_received_ ||
      (now - own_mavros_received_).toSec() > config_.mavros_timeout ||
      !own_mavros_->armed || own_mavros_->mode != "OFFBOARD") {
    return "follower_offboard_required";
  }

  struct PeerView {
    std::string name;
    const xd_uav_controller::ControlState* state;
    ros::Time received;
    const mavros_msgs::State* mavros;
    ros::Time mavros_received;
    bool has_state;
    bool has_mavros;
  };
  std::vector<PeerView> peers;
  if (config_.mode == "leader_follow") {
    peers.push_back({"leader", leader_state_ ? &*leader_state_ : nullptr,
        leader_state_received_, leader_mavros_ ? &*leader_mavros_ : nullptr,
        leader_mavros_received_, leader_state_.has_value() &&
            has_leader_state_received_, leader_mavros_.has_value() &&
            has_leader_mavros_received_});
  } else {
    const auto active = activeMembers();
    for (const auto& name : active) {
      if (name == member_) continue;
      const auto state = input.states.find(name);
      const auto received = input.state_received.find(name);
      const auto mavros = input.mavros.find(name);
      const auto mavros_received = input.mavros_received.find(name);
      peers.push_back({name,
          state == input.states.end() ? nullptr : &state->second,
          received == input.state_received.end() ? ros::Time() : received->second,
          mavros == input.mavros.end() ? nullptr : &mavros->second,
          mavros_received == input.mavros_received.end() ? ros::Time() :
              mavros_received->second,
          state != input.states.end() && received != input.state_received.end(),
          mavros != input.mavros.end() && mavros_received != input.mavros_received.end()});
    }
  }
  for (const auto& peer : peers) {
    if (!peer.has_state || !peer.state ||
        !fresh(peer.state->header, true, peer.received, now)) {
      return peer.name + "_state_stale";
    }
    const auto& state = *peer.state;
    if (!state.state_valid) return "state_invalid";
    if (own.airframe_type != state.airframe_type)
      return "same_airframe_type_required";
    if (state.transition_pending) return "transition_pending";
    if (own.vehicle_type != state.vehicle_type ||
        own.flight_regime != state.flight_regime)
      return "same_flight_regime_required";
    if (blockingAction(state)) return peer.name + "_flight_action";
    if (state.header.frame_id.empty()) return "state_frame_missing";
    if (!peer.has_mavros || !peer.mavros ||
        (now - peer.mavros_received).toSec() > config_.mavros_timeout ||
        !peer.mavros->armed ||
        (config_.mode == "consensus" && peer.mavros->mode != "OFFBOARD")) {
      return config_.mode == "leader_follow" ? "leader_armed_required" :
             "peer_offboard_required:" + peer.name;
    }
  }
  if (config_.mode == "consensus") {
    for (const auto& name : activeMembers()) {
      if (name == member_) continue;
      const auto center = input.centers.find(name);
      const auto received = input.center_received.find(name);
      if (center == input.centers.end() || received == input.center_received.end() ||
          !fresh(center->second.header, true, received->second, now) ||
          cleanFrame(center->second.header.frame_id) != config_.common_frame) {
        return "neighbor_center_stale:" + name;
      }
    }
  }
  return std::nullopt;
}

bool Follower::fresh(const std_msgs::Header& header, bool has_message,
                     const ros::Time& received, const ros::Time& now) const {
  if (!has_message || header.stamp.isZero()) return false;
  const double stamp_age = (now - header.stamp).toSec();
  const double received_age = std::max(0.0, (now - received).toSec());
  return received_age <= config_.state_timeout &&
      stamp_age >= -config_.future_stamp_tolerance &&
      stamp_age <= config_.state_timeout;
}

Follower::StateSnapshot Follower::snapshot() const {
  StateSnapshot result;
  if (shared_) {
    std::lock_guard<std::mutex> lock(shared_->mutex);
    const auto snapshot = shared_->snapshotLocked();
    result.states.bind(snapshot, &snapshot->states);
    result.state_received.bind(snapshot, &snapshot->state_received);
    result.mavros.bind(snapshot, &snapshot->mavros);
    result.mavros_received.bind(snapshot, &snapshot->mavros_received);
    result.centers.bind(snapshot, &snapshot->centers);
    result.center_received.bind(snapshot, &snapshot->center_received);
    result.statuses.bind(snapshot, &snapshot->statuses);
    result.status_received.bind(snapshot, &snapshot->status_received);
  }
  return result;
}

std::vector<std::string> Follower::activeMembers() const {
  if (active_members_dirty_) {
    active_members_cache_.clear();
    for (const auto& name : config_.members) {
      if (!dropped_members_.count(name)) active_members_cache_.push_back(name);
    }
    active_members_dirty_ = false;
  }
  return active_members_cache_;
}

std::map<std::string, Vec3> Follower::consensusSlotOffsets() const {
  const auto members = activeMembers();
  std::vector<Vec3> slots;
  if (members.size() == 1) {
    slots.push_back({0.0, 0.0, 0.0});
  } else if (!members.empty()) {
    slots = formationOffsets(config_.shape, members.size(), config_.spacing);
  }
  std::map<std::string, Vec3> offsets;
  for (std::size_t i = 0; i < members.size(); ++i)
    offsets.emplace(members[i], slots[i]);
  return offsets;
}

Vec3 Follower::leaderPoseInFollowerFrame(
    const ros::Time& now, const xd_uav_controller::ControlState& leader,
    const xd_uav_controller::ControlState& own,
    geometry_msgs::PoseStamped* pose,
    geometry_msgs::Vector3Stamped* velocity,
    geometry_msgs::Vector3Stamped* acceleration) const {
  pose->header = leader.header;
  pose->pose.position = leader.position_odom;
  pose->pose.orientation = leader.orientation_odom_body;
  velocity->header = leader.header;
  velocity->vector = leader.velocity_odom;
  acceleration->header = leader.header;
  acceleration->vector = leader.acceleration_odom;
  const std::string source = cleanFrame(leader.header.frame_id);
  const std::string target = cleanFrame(own.header.frame_id);
  if (source.empty() || target.empty()) {
    throw std::invalid_argument("formation_member_frame_missing");
  }
  if (source == target) return {};
  const auto transform = tf_buffer_->lookupTransform(
      target, source, ros::Time(0), ros::Duration(config_.tf_lookup_timeout));
  if (!transform.header.stamp.isZero()) {
    const double age = (now - transform.header.stamp).toSec();
    if (age < -config_.future_stamp_tolerance || age > config_.tf_max_age) {
      throw std::invalid_argument("leader_to_follower_tf_stale");
    }
  }
  geometry_msgs::PoseStamped transformed_pose;
  geometry_msgs::Vector3Stamped transformed_velocity, transformed_acceleration;
  tf2::doTransform(*pose, transformed_pose, transform);
  tf2::doTransform(*velocity, transformed_velocity, transform);
  tf2::doTransform(*acceleration, transformed_acceleration, transform);
  *pose = transformed_pose;
  *velocity = transformed_velocity;
  *acceleration = transformed_acceleration;
  return {};
}

Vec3 Follower::positionInFollowerFrame(
    const xd_uav_controller::ControlState& state, const ros::Time& now,
    const xd_uav_controller::ControlState& own) const {
  const std::string source = cleanFrame(state.header.frame_id);
  const std::string target = cleanFrame(own.header.frame_id);
  if (source.empty() || target.empty()) {
    throw std::invalid_argument("formation_member_frame_missing");
  }
  geometry_msgs::PoseStamped pose;
  pose.header = state.header;
  pose.pose.position = state.position_odom;
  if (source != target) {
    const auto transform = tf_buffer_->lookupTransform(
        target, source, ros::Time(0), ros::Duration(0.0));
    if (!transform.header.stamp.isZero()) {
      const double age = (now - transform.header.stamp).toSec();
      if (age < -config_.future_stamp_tolerance || age > config_.tf_max_age) {
        throw std::invalid_argument("formation_member_tf_stale");
      }
    }
    geometry_msgs::PoseStamped transformed;
    tf2::doTransform(pose, transformed, transform);
    pose = transformed;
  }
  Vec3 result = pointVec(pose.pose.position);
  if (!std::all_of(result.begin(), result.end(),
                   [](double value) { return std::isfinite(value); })) {
    throw std::invalid_argument("formation_member_position_invalid");
  }
  return result;
}

std::optional<double> Follower::leaderFollowPaceAdjustment(
    const ros::Time& now, const Vec3& leader_position, double heading,
    const StateSnapshot& input) {
  const auto members = activeMembers();
  if (members.size() < 2) return std::nullopt;
  const double tx = std::cos(heading), ty = std::sin(heading);
  const double nx = -ty, ny = tx;
  std::map<std::string, double> errors;
  for (const auto& name : members) {
    const auto offset = geometry_->member_offsets.find(name);
    const Vec3* member_offset = nullptr;
    if (offset != geometry_->member_offsets.end()) member_offset = &offset->second;
    else if (name == member_) member_offset = &offset_;
    if (!member_offset) return std::nullopt;
    const xd_uav_controller::ControlState* state = nullptr;
    if (name == member_) {
      if (!own_state_) return std::nullopt;
      state = &*own_state_;
    } else {
      const auto it = input.states.find(name);
      const auto received = input.state_received.find(name);
      if (it == input.states.end() || received == input.state_received.end() ||
          !fresh(it->second.header, true, received->second, now)) {
        return std::nullopt;
      }
      state = &it->second;
    }
    Vec3 position;
    try {
      position = positionInFollowerFrame(*state, now, *own_state_);
    } catch (const std::exception&) {
      return std::nullopt;
    }
    const auto target = slotTarget(leader_position, heading, *member_offset);
    const double ex = target[0] - position[0], ey = target[1] - position[1];
    const double along = ex * tx + ey * ty;
    const double cross = std::abs(ex * nx + ey * ny);
    errors[name] = std::abs(along) <= 1.0
        ? (cross > 1.0 ? cross : along)
        : std::copysign(std::hypot(along, cross), along);
  }
  double maximum_lag = 0.0;
  for (const auto& [_, error] : errors) maximum_lag = std::max(maximum_lag, error);
  maximum_lag = std::max(0.0, maximum_lag - 1.0);
  if (maximum_lag <= 0.0 || errors[member_] > 1.0) return std::nullopt;
  const double maximum_adjustment = own_state_->vehicle_type ==
          xd_uav_controller::ControlState::VEHICLE_FIXEDWING
      ? config_.fixedwing_max_catchup_speed
      : config_.multirotor_max_catchup_speed;
  return clampValue(-config_.catchup_gain * maximum_lag,
                    -maximum_adjustment, maximum_adjustment);
}

void Follower::resetCatchupPid() {
  catchup_integral_ = {{0.0, 0.0}};
  catchup_previous_error_.reset();
  catchup_filtered_derivative_ = {{0.0, 0.0}};
  catchup_pid_stamp_.reset();
}

Vec3 Follower::pidCatchupVelocity(
    const Vec3& base_velocity, const Vec3& slot_position,
    const Vec3& own_position, const ros::Time& now,
    const xd_uav_controller::ControlState& own) {
  const double ex = slot_position[0] - own_position[0];
  const double ey = slot_position[1] - own_position[1];
  const bool fixedwing = own.vehicle_type ==
      xd_uav_controller::ControlState::VEHICLE_FIXEDWING;
  const double base_speed = std::hypot(base_velocity[0], base_velocity[1]);
  double direction_x = 0.0, direction_y = 0.0;
  std::array<double, 2> errors{{ex, ey}};
  double max_correction = config_.multirotor_max_catchup_speed;
  if (fixedwing) {
    if (base_speed < 1e-6) { resetCatchupPid(); return base_velocity; }
    direction_x = base_velocity[0] / base_speed;
    direction_y = base_velocity[1] / base_speed;
    errors = {{ex * direction_x + ey * direction_y, 0.0}};
    max_correction = std::min(config_.fixedwing_max_catchup_speed,
                               0.5 * base_speed);
  }
  for (double& error : errors) {
    if (std::abs(error) <= config_.catchup_deadband) error = 0.0;
    else error = std::copysign(std::abs(error) - config_.catchup_deadband,
                               error);
  }
  double dt = 0.0;
  if (catchup_pid_stamp_) {
    const double measured = (now - *catchup_pid_stamp_).toSec();
    if (measured > 1e-3 && measured <= 0.5) dt = measured;
  }
  std::array<double, 2> derivative{{0.0, 0.0}};
  if (catchup_previous_error_ && dt > 0.0) {
    const double tau = config_.catchup_derivative_filter_time_constant;
    const double alpha = tau <= 1e-6 ? 1.0 : dt / (tau + dt);
    for (std::size_t i = 0; i < 2; ++i) {
      const double raw = (errors[i] - (*catchup_previous_error_)[i]) / dt;
      derivative[i] = catchup_filtered_derivative_[i] + alpha *
          (raw - catchup_filtered_derivative_[i]);
    }
  }
  auto candidate_integral = catchup_integral_;
  if (dt > 0.0) {
    for (std::size_t i = 0; i < 2; ++i) {
      if (errors[i] == 0.0) candidate_integral[i] *= std::max(0.0, 1.0 - dt / 5.0);
      else candidate_integral[i] = clampValue(
          candidate_integral[i] + errors[i] * dt,
          -config_.catchup_integral_limit, config_.catchup_integral_limit);
    }
  }
  auto correctionFor = [&](const std::array<double, 2>& integral) {
    return std::array<double, 2>{{
        config_.catchup_gain * errors[0] + config_.catchup_ki * integral[0] +
            config_.catchup_kd * derivative[0],
        config_.catchup_gain * errors[1] + config_.catchup_ki * integral[1] +
            config_.catchup_kd * derivative[1]}};
  };
  auto correction = correctionFor(candidate_integral);
  double correction_norm = std::hypot(correction[0], correction[1]);
  bool saturated = correction_norm > max_correction;
  if (fixedwing) {
    const double target = base_speed + correction[0];
    saturated = saturated || target > base_speed + config_.fixedwing_max_catchup_speed ||
        target < config_.fixedwing_min_speed;
  } else {
    saturated = saturated || std::hypot(base_velocity[0] + correction[0],
        base_velocity[1] + correction[1]) > config_.multirotor_max_speed;
  }
  const bool pushes = saturated &&
      errors[0] * correction[0] + errors[1] * correction[1] > 0.0;
  if (pushes) {
    candidate_integral = catchup_integral_;
    correction = correctionFor(candidate_integral);
    correction_norm = std::hypot(correction[0], correction[1]);
  }
  catchup_integral_ = candidate_integral;
  catchup_previous_error_ = errors;
  catchup_filtered_derivative_ = derivative;
  catchup_pid_stamp_ = now;
  if (correction_norm > max_correction && correction_norm > 0.0) {
    const double scale = max_correction / correction_norm;
    correction[0] *= scale; correction[1] *= scale;
  }
  if (fixedwing) {
    const double speed = clampValue(base_speed + correction[0],
        config_.fixedwing_min_speed,
        base_speed + config_.fixedwing_max_catchup_speed);
    return {direction_x * speed, direction_y * speed, base_velocity[2]};
  }
  double vx = base_velocity[0] + correction[0];
  double vy = base_velocity[1] + correction[1];
  const double speed = std::hypot(vx, vy);
  if (speed > config_.multirotor_max_speed) {
    vx *= config_.multirotor_max_speed / speed;
    vy *= config_.multirotor_max_speed / speed;
  }
  return {vx, vy, base_velocity[2]};
}

double Follower::yaw(const geometry_msgs::PoseStamped& pose) const {
  const auto& q = pose.pose.orientation;
  const std::array<double, 4> values{{q.x, q.y, q.z, q.w}};
  double norm = 0.0;
  for (double value : values) {
    if (!std::isfinite(value)) throw std::invalid_argument("invalid_orientation");
    norm += value * value;
  }
  if (norm < 0.5) throw std::invalid_argument("invalid_orientation");
  tf2::Quaternion quaternion(q.x, q.y, q.z, q.w);
  return tf2::getYaw(quaternion);
}

mavros_msgs::PositionTarget Follower::positionTarget(
    const Vec3& position, double target_yaw, const ros::Time& now,
    const std::optional<Vec3>& velocity, const std::string& frame_id,
    const xd_uav_controller::ControlState& own) const {
  mavros_msgs::PositionTarget target;
  target.header.stamp = now;
  target.header.frame_id = frame_id.empty() ? own.header.frame_id : frame_id;
  target.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
  target.type_mask = mavros_msgs::PositionTarget::IGNORE_AFX |
      mavros_msgs::PositionTarget::IGNORE_AFY |
      mavros_msgs::PositionTarget::IGNORE_AFZ |
      mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
  if (!velocity) {
    target.type_mask |= mavros_msgs::PositionTarget::IGNORE_VX |
        mavros_msgs::PositionTarget::IGNORE_VY |
        mavros_msgs::PositionTarget::IGNORE_VZ;
  } else {
    target.velocity = vectorMsg(*velocity);
  }
  target.position = pointMsg(position);
  target.yaw = target_yaw;
  return target;
}

std::tuple<Vec3, geometry_msgs::Vector3, geometry_msgs::PoseStamped>
Follower::stateInCommonFrame(const xd_uav_controller::ControlState& state,
                             const ros::Time& now) const {
  if (!has_common_state_cache_stamp_ || common_state_cache_stamp_ != now) {
    common_state_cache_stamp_ = now;
    has_common_state_cache_stamp_ = true;
    common_state_cache_.clear();
  }
  const auto cached = common_state_cache_.find(&state);
  if (cached != common_state_cache_.end()) return cached->second;
  geometry_msgs::PoseStamped pose;
  pose.header = state.header;
  pose.pose.position = state.position_odom;
  pose.pose.orientation = state.orientation_odom_body;
  geometry_msgs::Vector3Stamped velocity;
  velocity.header = state.header;
  velocity.vector = state.velocity_odom;
  const std::string source = cleanFrame(state.header.frame_id);
  if (source.empty()) throw std::invalid_argument("state_frame_missing");
  if (source != config_.common_frame) {
    const auto transform = tf_buffer_->lookupTransform(
        config_.common_frame, source, ros::Time(0),
        ros::Duration(config_.tf_lookup_timeout));
    if (!transform.header.stamp.isZero()) {
      const double age = (now - transform.header.stamp).toSec();
      if (age < -config_.future_stamp_tolerance || age > config_.tf_max_age)
        throw std::invalid_argument("state_to_common_tf_stale");
    }
    geometry_msgs::PoseStamped transformed_pose;
    geometry_msgs::Vector3Stamped transformed_velocity;
    tf2::doTransform(pose, transformed_pose, transform);
    tf2::doTransform(velocity, transformed_velocity, transform);
    pose = transformed_pose;
    velocity = transformed_velocity;
  }
  const Vec3 position = pointVec(pose.pose.position);
  for (double value : position)
    if (!std::isfinite(value)) throw std::invalid_argument("position_in_common_frame_invalid");
  const auto converted = std::make_tuple(position, velocity.vector, pose);
  common_state_cache_.emplace(&state, converted);
  return converted;
}

std::vector<Vec3> Follower::memberPositionsInCommonFrame(
    const ros::Time& now, const StateSnapshot& input) const {
  std::vector<Vec3> positions;
  for (const auto& name : config_.members) {
    const xd_uav_controller::ControlState* state = nullptr;
    if (name == member_) {
      if (!own_state_) throw std::invalid_argument("follower_state_stale");
      state = &*own_state_;
    } else {
      auto it = input.states.find(name);
      if (it == input.states.end()) throw std::invalid_argument(name + "_state_stale");
      state = &it->second;
    }
    const auto converted = stateInCommonFrame(*state, now);
    positions.push_back(std::get<0>(converted));
  }
  return positions;
}

bool Follower::usesFixedwingConsensusGuidance(
    const xd_uav_controller::ControlState& own) const {
  return config_.mode == "consensus" && own.vehicle_type ==
      xd_uav_controller::ControlState::VEHICLE_FIXEDWING;
}

Follower::Geometry Follower::fixedwingConsensusGeometry(
    const ros::Time& now, const StateSnapshot& input,
    const xd_uav_controller::ControlState& own) {
  Geometry geometry;
  geometry.members = activeMembers();
  const auto slot_offsets = consensusSlotOffsets();
  std::vector<Vec3> positions;
  positions.reserve(geometry.members.size());
  std::vector<Vec3> offsets;
  offsets.reserve(geometry.members.size());
  for (const auto& name : geometry.members) {
    const xd_uav_controller::ControlState* state = nullptr;
    if (name == member_) state = &own;
    else {
      const auto state_it = input.states.find(name);
      if (state_it != input.states.end()) state = &state_it->second;
    }
    if (!state) throw std::invalid_argument(name + "_state_stale");
    positions.push_back(std::get<0>(stateInCommonFrame(*state, now)));
    auto offset = slot_offsets.find(name);
    if (offset == slot_offsets.end()) throw std::invalid_argument("missing consensus slot");
    offsets.push_back(offset->second);
  }
  const Vec3 centroid = sharedCenter(positions).first;
  geometry.capturing = acquire_heading_ && acquire_stage_ &&
      (*acquire_stage_ == "waiting_for_team" || *acquire_stage_ == "separating" ||
       *acquire_stage_ == "gathering" || *acquire_stage_ == "merging");
  if (geometry.capturing) {
    geometry.heading = *acquire_heading_;
    geometry.route_position = centroid;
    geometry.route_vz = 0.0;
  } else {
    double seed_heading;
    if (consensus_heading_) seed_heading = *consensus_heading_;
    else {
      const auto reference = pathReferenceState(centroid, config_.waypoints,
          config_.cruise_speed, config_.lookahead, config_.loop_path);
      seed_heading = std::atan2(reference.velocity[1], reference.velocity[0]);
    }
    std::vector<Vec3> estimates;
    for (std::size_t i = 0; i < positions.size(); ++i)
      estimates.push_back(virtualCenter(positions[i], seed_heading, offsets[i]));
    const Vec3 estimate = sharedCenter(estimates).first;
    const auto reference = pathReferenceState(estimate, config_.waypoints,
        config_.cruise_speed, config_.lookahead, config_.loop_path);
    const double desired = std::atan2(reference.velocity[1], reference.velocity[0]);
    geometry.heading = desired;
    if (consensus_heading_) {
      geometry.heading = *consensus_heading_;
      if (consensus_heading_stamp_) {
        double dt = (now - *consensus_heading_stamp_).toSec();
        if (dt > 0.0) {
          dt = std::min(dt, 0.2);
          const double error = std::atan2(std::sin(desired - geometry.heading),
                                          std::cos(desired - geometry.heading));
          const double step = clampValue(error,
              -config_.formation_heading_rate_limit * dt,
              config_.formation_heading_rate_limit * dt);
          geometry.heading += step;
          geometry.turn_rate = step / dt;
        }
      }
    }
    geometry.route_position = reference.position;
    geometry.route_vz = reference.velocity[2];
  }
  consensus_heading_ = geometry.heading;
  consensus_heading_stamp_ = now;
  for (std::size_t i = 0; i < positions.size(); ++i)
    geometry.centers.push_back(virtualCenter(positions[i], geometry.heading, offsets[i]));
  const auto mean = sharedCenter(geometry.centers);
  geometry.mean = mean.first;
  geometry.largest_error = mean.second;
  return geometry;
}

Vec3 Follower::ownCenterInCommonFrame(const ros::Time& now,
                                     const StateSnapshot& input,
                                     const xd_uav_controller::ControlState& own) {
  const auto state = stateInCommonFrame(own, now);
  const Vec3 position = std::get<0>(state);
  const auto& velocity = std::get<1>(state);
  const auto& pose = std::get<2>(state);
  if (config_.mode == "consensus") {
    const auto slot_offsets = consensusSlotOffsets();
    const Vec3& consensus_offset = slot_offsets.at(member_);
    if (usesFixedwingConsensusGuidance(own)) {
      bool ready = true;
      for (const auto& name : activeMembers()) {
        if (name == member_) continue;
        auto it = input.states.find(name);
        auto received = input.state_received.find(name);
        if (it == input.states.end() || received == input.state_received.end() ||
            !fresh(it->second.header, true, received->second, now) ||
            !it->second.state_valid || it->second.header.frame_id.empty()) {
          ready = false; break;
        }
      }
      if (ready) {
        auto geometry = fixedwingConsensusGeometry(now, input, own);
        const auto i = std::find(geometry.members.begin(), geometry.members.end(), member_);
        return geometry.centers.at(static_cast<std::size_t>(i - geometry.members.begin()));
      }
    }
    const auto active = activeMembers();
    if (!usesFixedwingConsensusGuidance(own) && active.size() > 1) {
      std::vector<Vec3> peer_points;
      bool all_fresh = true;
      for (const auto& name : active) {
        if (name == member_) continue;
        auto center = input.centers.find(name);
        auto received = input.center_received.find(name);
        if (center == input.centers.end() || received == input.center_received.end() ||
            !fresh(center->second.header, true, received->second, now) ||
            cleanFrame(center->second.header.frame_id) != config_.common_frame) {
          all_fresh = false; break;
        }
        peer_points.push_back(pointVec(center->second.point));
      }
      if (all_fresh) {
        Vec3 estimate = sharedCenter(peer_points).first;
        Vec3 center = position;
        for (int i = 0; i < 2; ++i) {
          const auto path = pathReferenceState(estimate, config_.waypoints,
              config_.cruise_speed, config_.lookahead, config_.loop_path);
          const double heading = std::atan2(path.velocity[1], path.velocity[0]);
          center = virtualCenter(position, heading, consensus_offset);
          auto all = peer_points; all.push_back(center);
          estimate = sharedCenter(all).first;
        }
        return center;
      }
    }
    Vec3 center = position;
    for (int i = 0; i < 2; ++i) {
      const auto path = pathReferenceState(center, config_.waypoints,
          config_.cruise_speed, config_.lookahead, config_.loop_path);
      center = virtualCenter(position,
          std::atan2(path.velocity[1], path.velocity[0]), consensus_offset);
    }
    return center;
  }
  double heading = 0.0;
  if (own.vehicle_type == xd_uav_controller::ControlState::VEHICLE_FIXEDWING) {
    const double speed = std::hypot(velocity.x, velocity.y);
    if (!std::isfinite(velocity.x) || !std::isfinite(velocity.y))
      throw std::invalid_argument("own_speed_too_low_for_fixedwing");
    if (own.airspeed_valid) {
      if (!std::isfinite(own.airspeed) || own.airspeed < config_.fixedwing_min_speed)
        throw std::invalid_argument("own_airspeed_too_low_for_fixedwing");
      heading = speed >= config_.fixedwing_min_groundspeed_for_course
          ? std::atan2(velocity.y, velocity.x) : yaw(pose);
    } else if (speed >= config_.fixedwing_min_speed) {
      heading = std::atan2(velocity.y, velocity.x);
    } else throw std::invalid_argument("own_speed_too_low_for_fixedwing");
  } else heading = yaw(pose);
  return virtualCenter(position, heading, offset_);
}

void Follower::publishCenter(const ros::Time& now, const StateSnapshot& input) {
  if (!own_state_ || !fresh(own_state_->header, has_own_state_received_,
      own_state_received_, now) || !own_state_->state_valid ||
      own_state_->header.frame_id.empty()) return;
  Vec3 center;
  try { center = ownCenterInCommonFrame(now, input, *own_state_); }
  catch (const std::exception&) { return; }
  geometry_msgs::PointStamped message;
  message.header.stamp = now;
  message.header.frame_id = config_.common_frame;
  message.point = pointMsg(center);
  {
    std::lock_guard<std::mutex> lock(shared_->mutex);
    shared_->centers[member_] = message;
    shared_->center_received[member_] = now;
    shared_->markChanged();
  }
  center_pub_.publish(message);
}

std::tuple<Vec3, Vec3, double> Follower::consensusCenters(
    const ros::Time& now, const StateSnapshot& input,
    const xd_uav_controller::ControlState& own) {
  if (usesFixedwingConsensusGuidance(own)) {
    auto geometry = fixedwingConsensusGeometry(now, input, own);
    const auto it = std::find(geometry.members.begin(), geometry.members.end(), member_);
    return {geometry.centers.at(static_cast<std::size_t>(it - geometry.members.begin())),
            geometry.mean, geometry.largest_error};
  }
  Vec3 center = ownCenterInCommonFrame(now, input, own);
  auto points = teamCenterPoints(center, input);
  auto result = sharedCenter(points);
  return {center, result.first, result.second};
}

std::vector<Vec3> Follower::teamCenterPoints(
    const Vec3& own_center, const StateSnapshot& input) const {
  std::vector<Vec3> result;
  for (const auto& name : activeMembers()) {
    if (name == member_) result.push_back(own_center);
    else {
      auto it = input.centers.find(name);
      if (it == input.centers.end()) throw std::invalid_argument("neighbor_center_stale:" + name);
      result.push_back(pointVec(it->second.point));
    }
  }
  return result;
}

double Follower::effectiveConvergenceTime() const {
  const double preview = config_.lookahead / std::max(config_.cruise_speed, 1e-3);
  return std::min(config_.convergence_time, std::max(0.1, preview));
}

std::pair<double, double> Follower::activeConsensusCorrection() const {
  if (acquire_stage_ && (*acquire_stage_ == "separating" ||
      *acquire_stage_ == "gathering" || *acquire_stage_ == "merging"))
    return {config_.acquire_consensus_gain, config_.acquire_max_consensus_velocity};
  return {config_.consensus_gain, config_.max_consensus_velocity};
}

PathResult Follower::pacedPathReference(const Vec3& center, double speed) const {
  speed = std::max(speed, 1e-3);
  const double lookahead = config_.lookahead * speed /
      std::max(config_.cruise_speed, 1e-3);
  return pathReferenceState(center, config_.waypoints, speed, lookahead,
                            config_.loop_path);
}

double Follower::teamPacedSpeed(
    const ros::Time& now, const std::vector<Vec3>& center_points,
    const Vec3& mean, const Vec3& path_velocity, double heading,
    double turn_rate, const StateSnapshot& input,
    const xd_uav_controller::ControlState& own) {
  const double path_speed = std::hypot(path_velocity[0], path_velocity[1]);
  const auto members = activeMembers();
  if (path_speed < 1e-6) return path_speed;
  const double tx = path_velocity[0] / path_speed, ty = path_velocity[1] / path_speed;
  const double nx = -ty, ny = tx;
  std::vector<double> progress;
  progress.reserve(center_points.size());
  for (const auto& point : center_points) {
    const double ex = mean[0] - point[0], ey = mean[1] - point[1];
    const double along = ex * tx + ey * ty;
    const double cross = std::abs(ex * nx + ey * ny);
    progress.push_back(std::abs(along) <= 1.0 ? cross : std::copysign(std::hypot(along, cross), along));
  }
  double dt = paced_speed_stamp_ ? clampValue((now - *paced_speed_stamp_).toSec(), 0.0, 0.2) : 0.0;
  const double alpha = dt / (0.8 + dt);
  for (std::size_t i = 0; i < progress.size(); ++i) {
    auto it = progress_error_filter_.find(members[i]);
    if (it == progress_error_filter_.end()) progress_error_filter_[members[i]] = progress[i];
    else it->second += alpha * (progress[i] - it->second);
    progress[i] = progress_error_filter_[members[i]];
  }
  std::vector<double> member_speeds;
  member_speeds.reserve(members.size());
  const auto slot_offsets = consensusSlotOffsets();
  for (const auto& name : members) {
    const auto& state = name == member_ ? own : input.states.at(name);
    const auto converted = stateInCommonFrame(state, now);
    const auto velocity = std::get<1>(converted);
    const Vec3 slot_turn = slotVelocity({0,0,0}, {0,0,0}, heading,
        slot_offsets.at(name), 0.0, false, turn_rate, true);
    member_speeds.push_back((velocity.x - slot_turn[0]) * tx +
                            (velocity.y - slot_turn[1]) * ty);
  }
  const bool fixedwing = own.vehicle_type == xd_uav_controller::ControlState::VEHICLE_FIXEDWING;
  const double minimum = fixedwing ? config_.fixedwing_min_speed : 0.0;
  const double gain = activeConsensusCorrection().first;
  const bool acquisition = acquire_stage_ && (*acquire_stage_ == "separating" ||
      *acquire_stage_ == "gathering" || *acquire_stage_ == "merging");
  const double max_adjust = acquisition ? config_.acquire_max_speed_adjustment :
      config_.max_consensus_velocity;
  auto targets = consensusSpeedTargets(path_speed, progress, member_speeds,
      gain, max_adjust, minimum);
  const auto member_it = std::find(members.begin(), members.end(), member_);
  const double target = targets.at(static_cast<std::size_t>(member_it - members.begin()));
  if (!paced_speed_ || !paced_speed_stamp_) paced_speed_ = target;
  else {
    dt = clampValue((now - *paced_speed_stamp_).toSec(), 0.0, 0.2);
    const double change = 2.0 * dt;
    *paced_speed_ += clampValue(target - *paced_speed_, -change, change);
  }
  paced_speed_stamp_ = now;
  return std::max(minimum, *paced_speed_);
}

Vec3 Follower::smoothCourseVelocity(const Vec3& velocity, const ros::Time& now,
                                    const xd_uav_controller::ControlState& own) {
  if (own.vehicle_type != xd_uav_controller::ControlState::VEHICLE_FIXEDWING)
    return velocity;
  const double speed = std::hypot(velocity[0], velocity[1]);
  if (speed < 1e-3) return velocity;
  const double target = std::atan2(velocity[1], velocity[0]);
  if (!filtered_course_ || !filtered_course_stamp_) {
    filtered_course_ = target; filtered_course_stamp_ = now; filtered_speed_ = speed;
    return velocity;
  }
  const double dt = clampValue((now - *filtered_course_stamp_).toSec(), 0.0, 0.2);
  if (!filtered_speed_) filtered_speed_ = speed;
  else {
    const double slew = config_.mode == "consensus" ? config_.formation_speed_slew_rate : 2.0;
    *filtered_speed_ += clampValue(speed - *filtered_speed_, -slew * dt, slew * dt);
  }
  const double course_rate = config_.mode == "consensus" ? config_.formation_course_slew_rate : 0.25;
  const double error = std::atan2(std::sin(target - *filtered_course_), std::cos(target - *filtered_course_));
  *filtered_course_ += clampValue(error, -course_rate * dt, course_rate * dt);
  filtered_course_stamp_ = now;
  return {*filtered_speed_ * std::cos(*filtered_course_),
          *filtered_speed_ * std::sin(*filtered_course_), velocity[2]};
}

Vec3 Follower::crossTrackCenter(const Vec3& center, const Vec3& mean,
                                double heading) const {
  const double nx = -std::sin(heading), ny = std::cos(heading);
  const double error = (mean[0] - center[0]) * nx + (mean[1] - center[1]) * ny;
  return {center[0] + nx * error, center[1] + ny * error, mean[2]};
}

std::vector<Vec3> Follower::consensusNeighbors(
    const Vec3& center, const Vec3& mean, double heading) const {
  if (activeMembers().size() < 2) return {};
  return {crossTrackCenter(center, mean, heading)};
}

bool Follower::teamEnabled(const ros::Time& now,
                           const StateSnapshot& input) const {
  for (const auto& name : activeMembers()) {
    if (name == member_) continue;
    auto status = input.statuses.find(name);
    auto received = input.status_received.find(name);
    if (status == input.statuses.end() || received == input.status_received.end() ||
        status->second.rfind("active:", 0) != 0 ||
        std::max(0.0, (now - received->second).toSec()) > config_.mavros_timeout)
      return false;
  }
  return true;
}

void Follower::setAcquireStage(const std::string& stage) {
  acquire_stage_ = stage;
  reason_ = stage == "following" ? "following" : "acquiring_" + stage;
  publishStatus();
}

AcquisitionPlan Follower::buildAcquisitionPlan(
    const ros::Time& now, const StateSnapshot& input) const {
  AcquisitionPlan plan;
  plan.members = config_.members;
  plan.shape = config_.shape;
  plan.spacing = config_.spacing;
  plan.stamp = now;
  plan.initial_positions = memberPositionsInCommonFrame(now, input);
  Vec3 mean{0.0, 0.0, 0.0};
  for (const auto& point : plan.initial_positions)
    for (int axis = 0; axis < 3; ++axis) mean[axis] += point[axis] / plan.initial_positions.size();
  const auto route = pathReferenceState(mean, config_.waypoints,
      config_.cruise_speed, config_.lookahead, config_.loop_path);
  plan.heading = std::atan2(route.velocity[1], route.velocity[0]);
  plan.altitude = route.position[2];
  plan.slots = acquisitionSlots(plan.initial_positions, geometry_->slots,
                                plan.heading, plan.altitude);
  double top = route.position[2];
  for (const auto& point : plan.initial_positions) top = std::max(top, point[2]);
  const double base = std::ceil(top);
  for (std::size_t i = 0; i < config_.members.size(); ++i)
    plan.tiers.push_back(base + i * config_.acquire_vertical_spacing);
  return plan;
}

void Follower::planAcquisition(const ros::Time& now,
                               const StateSnapshot& input) {
  std::optional<AcquisitionPlan> plan;
  {
    std::lock_guard<std::mutex> lock(shared_->acquisition_plan_mutex);
    plan = shared_->acquisition_plan;
  }
  if (!plan && member_ == config_.members.front()) {
    plan = buildAcquisitionPlan(now, input);
    {
      std::lock_guard<std::mutex> lock(shared_->acquisition_plan_mutex);
      shared_->acquisition_plan = plan;
    }
    nh_.setParam("/formation/" + config_.team_id + "/acquisition_plan",
                 acquisitionPlanValue(*plan));
  }
  if (!plan) {
    XmlRpc::XmlRpcValue parameter;
    if (nh_.getParam("/formation/" + config_.team_id + "/acquisition_plan", parameter)) {
      try {
        AcquisitionPlan parsed;
        for (int i = 0; i < parameter["members"].size(); ++i)
          parsed.members.push_back(static_cast<std::string>(parameter["members"][i]));
        parsed.shape = static_cast<std::string>(parameter["shape"]);
        parsed.spacing = static_cast<double>(parameter["spacing"]);
        parsed.stamp = ros::Time(static_cast<double>(parameter["stamp"]));
        auto readPoints = [](const XmlRpc::XmlRpcValue& values) {
          std::vector<Vec3> points;
          for (int i = 0; i < values.size(); ++i) {
            if (values[i].size() != 3) throw std::invalid_argument("bad acquisition point");
            Vec3 p{};
            for (int j = 0; j < 3; ++j) p[j] = static_cast<double>(values[i][j]);
            points.push_back(p);
          }
          return points;
        };
        parsed.initial_positions = readPoints(parameter["initial_positions"]);
        parsed.slots = readPoints(parameter["slots"]);
        for (int i = 0; i < parameter["tiers"].size(); ++i)
          parsed.tiers.push_back(static_cast<double>(parameter["tiers"][i]));
        parsed.heading = static_cast<double>(parameter["heading"]);
        parsed.altitude = static_cast<double>(parameter["altitude"]);
        plan = parsed;
      } catch (const std::exception&) {
        throw std::invalid_argument("shared acquisition plan is invalid");
      }
    }
  }
  const std::size_t count = config_.members.size();
  if (!plan || plan->members != config_.members || plan->shape != config_.shape ||
      plan->spacing != config_.spacing || plan->initial_positions.size() != count ||
      plan->slots.size() != count || plan->tiers.size() != count)
    throw std::invalid_argument("shared acquisition plan is missing or mismatched");
  const double age = (now - plan->stamp).toSec();
  if (age < 0.0 || age > config_.acquire_timeout)
    throw std::invalid_argument("shared acquisition plan has expired");
  for (const auto& p : plan->initial_positions)
    for (double value : p) if (!std::isfinite(value)) throw std::invalid_argument("shared acquisition plan is invalid");
  for (const auto& p : plan->slots)
    for (double value : p) if (!std::isfinite(value)) throw std::invalid_argument("shared acquisition plan is invalid");
  for (double value : plan->tiers) if (!std::isfinite(value)) throw std::invalid_argument("shared acquisition plan is invalid");
  const auto index = static_cast<std::size_t>(std::find(config_.members.begin(),
      config_.members.end(), member_) - config_.members.begin());
  acquire_slots_ = plan->slots;
  acquire_tiers_ = plan->tiers;
  acquire_initial_xy_.clear();
  acquire_initial_xy_.push_back({plan->initial_positions[index][0], plan->initial_positions[index][1]});
  acquire_heading_ = plan->heading;
  acquire_altitude_ = plan->altitude;
  if (!std::isfinite(*acquire_heading_) || !std::isfinite(*acquire_altitude_))
    throw std::invalid_argument("shared acquisition heading or altitude is invalid");
  acquire_started_ = plan->stamp;
  const double error = std::get<2>(consensusCenters(now, input, *own_state_));
  setAcquireStage(error <= config_.acquire_tolerance ? "following" : "separating");
}

mavros_msgs::PositionTarget Follower::fixedwingWaitTarget(
    const ros::Time& now, const StateSnapshot& input,
    const xd_uav_controller::ControlState& own) {
  const auto state = stateInCommonFrame(own, now);
  const auto position = std::get<0>(state);
  const auto velocity = std::get<1>(state);
  const double heading = std::atan2(velocity.y, velocity.x);
  const double speed = std::max(config_.fixedwing_min_speed, config_.cruise_speed);
  const double preview = std::max(config_.lookahead, speed * 2.0);
  const Vec3 target{position[0] + preview * std::cos(heading),
                    position[1] + preview * std::sin(heading), position[2]};
  (void)input;
  return positionTarget(target, heading, now,
      Vec3{speed * std::cos(heading), speed * std::sin(heading), 0.0},
      config_.common_frame, own);
}

mavros_msgs::PositionTarget Follower::fixedwingGatheringTarget(
    const ros::Time& now, double altitude, const StateSnapshot& input,
    const xd_uav_controller::ControlState& own) {
  return fixedwingConsensusTarget(now, input, own, altitude);
}

mavros_msgs::PositionTarget Follower::acquisitionTarget(
    const ros::Time& now, const StateSnapshot& input,
    const xd_uav_controller::ControlState& own) {
  if (acquire_stage_ && *acquire_stage_ == "waiting_for_team") {
    if (teamEnabled(now, input)) planAcquisition(now, input);
    else if (own.vehicle_type == xd_uav_controller::ControlState::VEHICLE_FIXEDWING)
      return fixedwingWaitTarget(now, input, own);
    else {
      const auto p = pointVec(own.position_odom);
      geometry_msgs::PoseStamped pose; pose.pose.orientation = own.orientation_odom_body;
      return positionTarget(p, yaw(pose), now, Vec3{0,0,0}, "", own);
    }
  }
  if (acquire_stage_ && *acquire_stage_ == "following")
    return consensusTarget(now, input, own);
  const auto positions = memberPositionsInCommonFrame(now, input);
  const std::size_t index = static_cast<std::size_t>(std::find(config_.members.begin(),
      config_.members.end(), member_) - config_.members.begin());
  if (*acquire_stage_ == "separating") {
    bool separated = true;
    for (std::size_t i = 0; i < positions.size(); ++i)
      separated = separated && std::abs(positions[i][2] - acquire_tiers_[i]) <= config_.acquire_height_tolerance;
    if (separated) setAcquireStage("gathering");
  }
  if (own.vehicle_type == xd_uav_controller::ControlState::VEHICLE_FIXEDWING) {
    const double center_error = std::get<2>(consensusCenters(now, input, own));
    if (*acquire_stage_ == "gathering" && center_error <= config_.acquire_tolerance)
      setAcquireStage("merging");
    bool merged = true;
    for (const auto& p : positions)
      merged = merged && std::abs(p[2] - *acquire_altitude_) <= config_.acquire_height_tolerance;
    if (*acquire_stage_ == "merging" && center_error <= config_.acquire_tolerance && merged)
      setAcquireStage("following");
    if (*acquire_stage_ == "separating" || *acquire_stage_ == "gathering")
      return fixedwingGatheringTarget(now, acquire_tiers_[index], input, own);
    return consensusTarget(now, input, own);
  }
  double slot_error = 0.0;
  for (std::size_t i = 0; i < positions.size(); ++i)
    slot_error = std::max(slot_error, std::hypot(positions[i][0] - acquire_slots_[i][0],
                                                 positions[i][1] - acquire_slots_[i][1]));
  if (*acquire_stage_ == "gathering" && slot_error <= config_.acquire_tolerance)
    setAcquireStage("merging");
  bool merged = true;
  for (std::size_t i = 0; i < positions.size(); ++i)
    merged = merged && std::abs(positions[i][2] - acquire_slots_[i][2]) <= config_.acquire_height_tolerance;
  if (*acquire_stage_ == "merging" && slot_error <= config_.acquire_tolerance && merged) {
    setAcquireStage("following");
    return consensusTarget(now, input, own);
  }
  Vec3 target;
  if (*acquire_stage_ == "separating")
    target = {acquire_initial_xy_[0][0], acquire_initial_xy_[0][1], acquire_tiers_[index]};
  else if (*acquire_stage_ == "gathering")
    target = {acquire_slots_[index][0], acquire_slots_[index][1], acquire_tiers_[index]};
  else target = acquire_slots_[index];
  return positionTarget(target, *acquire_heading_ + config_.yaw_offset,
                        now, Vec3{0,0,0}, config_.common_frame, own);
}

mavros_msgs::PositionTarget Follower::fixedwingConsensusTarget(
    const ros::Time& now, const StateSnapshot& input,
    const xd_uav_controller::ControlState& own,
    std::optional<double> altitude) {
  const Geometry geometry = fixedwingConsensusGeometry(now, input, own);
  const auto slot_offsets = consensusSlotOffsets();
  const Vec3& consensus_offset = slot_offsets.at(member_);
  const auto member_it = std::find(geometry.members.begin(), geometry.members.end(), member_);
  const std::size_t index = static_cast<std::size_t>(member_it - geometry.members.begin());
  const Vec3 center = geometry.centers[index], mean = geometry.mean;
  const double heading = geometry.heading;
  const double tx = std::cos(heading), ty = std::sin(heading);
  const double nx = -ty, ny = tx;
  const double ex = mean[0] - center[0], ey = mean[1] - center[1];
  double along = ex * tx + ey * ty;
  const double cross = ex * nx + ey * ny;
  const auto correction = activeConsensusCorrection();
  const bool capture_stage = acquire_stage_ && (*acquire_stage_ == "separating" ||
      *acquire_stage_ == "gathering" || *acquire_stage_ == "merging");
  const double speed_limit = capture_stage ? config_.acquire_max_speed_adjustment :
      config_.max_consensus_velocity;
  along = std::copysign(std::max(0.0, std::abs(along) - config_.catchup_deadband), along);
  const double speed_adjustment = clampValue(correction.first * along,
                                              -speed_limit, speed_limit);
  const double lateral_adjustment = clampValue(correction.first * cross,
      -correction.second, correction.second);
  double route_adjustment = 0.0;
  if (!geometry.capturing) {
    const double route_error = (geometry.route_position[0] - mean[0]) * nx +
                               (geometry.route_position[1] - mean[1]) * ny;
    route_adjustment = clampValue(config_.route_correction_gain * route_error,
        -config_.max_route_correction_velocity, config_.max_route_correction_velocity);
  }
  Vec3 velocity = slotVelocity({config_.cruise_speed * tx,
      config_.cruise_speed * ty, geometry.route_vz}, {0,0,0}, heading,
      consensus_offset, 0.0, false, geometry.turn_rate, true);
  const double along_speed = std::max(config_.fixedwing_min_speed,
      velocity[0] * tx + velocity[1] * ty + speed_adjustment);
  const double cross_speed = velocity[0] * nx + velocity[1] * ny +
                              lateral_adjustment + route_adjustment;
  velocity = smoothCourseVelocity({tx * along_speed + nx * cross_speed,
      ty * along_speed + ny * cross_speed, geometry.route_vz}, now, own);
  const auto slot = slotTarget(mean, heading, consensus_offset, config_.yaw_offset);
  const double target_altitude = altitude ? *altitude :
      (geometry.capturing ? *acquire_altitude_ + consensus_offset[2] :
       geometry.route_position[2] + consensus_offset[2]);
  auto target = positionTarget({slot[0], slot[1], target_altitude}, slot[3],
      now, velocity, config_.common_frame, own);
  target.type_mask |= mavros_msgs::PositionTarget::IGNORE_PX |
                      mavros_msgs::PositionTarget::IGNORE_PY;
  return target;
}

mavros_msgs::PositionTarget Follower::consensusTarget(
    const ros::Time& now, const StateSnapshot& input,
    const xd_uav_controller::ControlState& own) {
  if (usesFixedwingConsensusGuidance(own))
    return fixedwingConsensusTarget(now, input, own);
  const auto centers = consensusCenters(now, input, own);
  const Vec3 center = std::get<0>(centers), mean = std::get<1>(centers);
  const auto center_points = teamCenterPoints(center, input);
  const auto nominal = pathReferenceState(mean, config_.waypoints,
      config_.cruise_speed, config_.lookahead, config_.loop_path);
  double heading = std::atan2(nominal.velocity[1], nominal.velocity[0]);
  const double paced_speed = teamPacedSpeed(now, center_points, mean,
      nominal.velocity, heading, nominal.turn_rate, input, own);
  const auto path = pacedPathReference(mean, paced_speed);
  heading = std::atan2(path.velocity[1], path.velocity[0]);
  const auto slot_offsets = consensusSlotOffsets();
  const Vec3& consensus_offset = slot_offsets.at(member_);
  const auto slot = slotTarget(path.position, heading, consensus_offset, config_.yaw_offset);
  const auto neighbors = consensusNeighbors(center, mean, heading);
  const bool fixedwing = own.vehicle_type == xd_uav_controller::ControlState::VEHICLE_FIXEDWING;
  const auto correction = activeConsensusCorrection();
  Vec3 velocity = activeMembers().size() > 1
      ? consensusVelocity(path.velocity, center, center, neighbors,
          correction.first, effectiveConvergenceTime(), correction.second, true)
      : path.velocity;
  velocity = slotVelocity(velocity, {0,0,0}, heading, consensus_offset, 0.0, false,
                          path.turn_rate, true);
  velocity = catchupVelocity(velocity, path.position, path.position, 0.0,
      fixedwing ? 0.0 : 0.0, !fixedwing,
      config_.multirotor_max_speed, !fixedwing, 0.0);
  velocity = smoothCourseVelocity(velocity, now, own);
  return positionTarget({slot[0], slot[1], slot[2]}, slot[3], now,
                        velocity, config_.common_frame, own);
}

mavros_msgs::PositionTarget Follower::formationTarget(
    const ros::Time& now, const StateSnapshot& input,
    const xd_uav_controller::ControlState& own) {
  if (config_.mode == "consensus") return consensusTarget(now, input, own);
  if (!leader_state_) throw std::invalid_argument("leader_state_stale");
  geometry_msgs::PoseStamped pose;
  geometry_msgs::Vector3Stamped leader_velocity, leader_acceleration;
  leaderPoseInFollowerFrame(now, *leader_state_, own, &pose,
                            &leader_velocity, &leader_acceleration);
  const Vec3 position = pointVec(pose.pose.position);
  const Vec3 v = vectorVec(leader_velocity.vector);
  for (double value : v) if (!std::isfinite(value)) throw std::invalid_argument("leader_velocity_invalid");
  const bool fixedwing = own.vehicle_type == xd_uav_controller::ControlState::VEHICLE_FIXEDWING;
  const double speed = std::hypot(v[0], v[1]);
  const double heading = fixedwing && speed >= config_.fixedwing_min_groundspeed_for_course
      ? std::atan2(v[1], v[0]) : yaw(pose);
  const auto slot = slotTarget(position, heading, offset_, config_.yaw_offset);
  Vec3 acceleration = vectorVec(leader_acceleration.vector);
  if (!leader_state_->acceleration_fresh) acceleration = {0,0,0};
  std::optional<double> turn_rate;
  if (fixedwing && !leader_state_->acceleration_fresh) {
    const auto& q = pose.pose.orientation;
    tf2::Quaternion quaternion(q.x, q.y, q.z, q.w);
    double roll, pitch, yaw_angle;
    tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw_angle);
    turn_rate = 9.8 * std::tan(roll) / std::max(speed, config_.fixedwing_min_speed);
  } else if (!fixedwing && std::isfinite(leader_state_->body_rate.z)) {
    turn_rate = leader_state_->body_rate.z;
  }
  Vec3 velocity = slotVelocity(v, acceleration, heading, offset_, 0.0, false,
      turn_rate.value_or(0.0), turn_rate.has_value());
  const double base_speed = std::hypot(velocity[0], velocity[1]);
  const Vec3 own_position = pointVec(own.position_odom);
  velocity = pidCatchupVelocity(velocity, {slot[0], slot[1], slot[2]},
                                own_position, now, own);
  const auto pace = leaderFollowPaceAdjustment(now, position, heading, input);
  if (pace) {
    const double minimum = fixedwing ? config_.fixedwing_min_speed : 0.0;
    const double pacing_base = std::max(base_speed, minimum);
    const double maximum = fixedwing ? pacing_base + config_.fixedwing_max_catchup_speed :
        config_.multirotor_max_speed;
    const double paced = clampValue(pacing_base + *pace, minimum, maximum);
    const double corrected = std::hypot(velocity[0], velocity[1]);
    if (corrected > 1e-3) {
      velocity[0] *= paced / corrected; velocity[1] *= paced / corrected;
    }
  }
  velocity = smoothCourseVelocity(velocity, now, own);
  return positionTarget({slot[0], slot[1], slot[2]}, slot[3], now,
                        velocity, "", own);
}

bool Follower::ownReadinessFailure(const std::string& reason) const {
  if (config_.mode == "leader_follow" &&
      (reason == "same_airframe_type_required" || reason == "same_flight_regime_required"))
    return false;
  static const std::set<std::string> failures = {
      "follower_state_stale", "state_invalid", "unsupported_airframe",
      "transition_pending", "hover_regime_required", "forward_flight_required",
      "fixedwing_slot_too_close", "unsupported_vehicle_type",
      "follower_flight_action", "state_frame_missing", "follower_offboard_required",
      "same_airframe_type_required", "same_flight_regime_required"};
  return failures.count(reason) != 0;
}

bool Follower::setEnabled(bool enabled, std::string& message) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!enabled) {
    const bool was_enabled = enabled_;
    stop("stopped_by_service", was_enabled);
    message = "formation stopped";
    return true;
  }
  if (enabled_) { message = "formation already active"; return true; }
  const ros::Time now = ros::Time::now();
  if (last_external_reference_ &&
      (now - *last_external_reference_).toSec() < config_.external_quiet_time) {
    message = "another reference is active";
    return false;
  }
  const StateSnapshot input = snapshot();
  const auto failure = readiness(now, input);
  if (failure) { message = *failure; return false; }
  try {
    if (config_.mode == "consensus") {
      memberPositionsInCommonFrame(now, input);
      if (member_ == config_.members.front()) {
        const auto plan = buildAcquisitionPlan(now, input);
        {
          std::lock_guard<std::mutex> lock(shared_->acquisition_plan_mutex);
          shared_->acquisition_plan = plan;
        }
        nh_.setParam("/formation/" + config_.team_id + "/acquisition_plan",
                     acquisitionPlanValue(plan));
      } else {
        std::lock_guard<std::mutex> lock(shared_->acquisition_plan_mutex);
        if (!shared_->acquisition_plan) {
          XmlRpc::XmlRpcValue value;
          if (!nh_.getParam("/formation/" + config_.team_id + "/acquisition_plan", value)) {
            message = "start the team through /formation/set_enabled";
            return false;
          }
        }
      }
    } else {
      formationTarget(now, input, *own_state_);
    }
  } catch (const tf2::TransformException& error) {
    message = std::string("transform unavailable: ") + error.what(); return false;
  } catch (const XmlRpc::XmlRpcException& error) {
    message = std::string("ROS parameter error: ") + error.getMessage(); return false;
  } catch (const std::exception& error) {
    message = std::string("transform unavailable: ") + error.what(); return false;
  } catch (...) {
    message = "unknown error while starting formation"; return false;
  }
  dropped_members_.clear();
  active_members_dirty_ = true;
  enabled_ = true;
  if (config_.mode == "consensus") {
    paced_speed_.reset(); paced_speed_stamp_.reset(); progress_error_filter_.clear();
    consensus_heading_.reset(); consensus_heading_stamp_.reset();
    acquire_heading_.reset(); acquire_stage_ = "waiting_for_team";
    acquire_started_ = now; reason_ = "acquiring_waiting_for_team";
  } else reason_ = "following";
  filtered_course_.reset(); filtered_course_stamp_.reset(); filtered_speed_.reset();
  resetCatchupPid();
  publishStatus();
  message = config_.mode == "consensus" ? "formation gathering started" : "formation started";
  return true;
}

bool Follower::setEnabledService(std_srvs::SetBool::Request& request,
                                 std_srvs::SetBool::Response& response) {
  response.success = setEnabled(request.data, response.message);
  return true;
}

void Follower::tick(const ros::TimerEvent&) {
  std::lock_guard<std::mutex> lock(mutex_);
  const ros::Time now = ros::Time::now();
  const StateSnapshot input = snapshot();
  if (config_.mode == "consensus") {
    if (!enabled_) {
      if (!last_idle_center_publish_ ||
          (now - *last_idle_center_publish_).toSec() >= idle_center_period_) {
        publishCenter(now, input);
        last_idle_center_publish_ = now;
      }
      return;
    }
    publishCenter(now, input);
    refreshMembership(now, input);
  }
  if (!enabled_) return;
  if (!last_status_publish_ ||
      (now - *last_status_publish_).toSec() >= status_heartbeat_period_)
    publishStatus();
  const auto failure = readiness(now, snapshot());
  if (failure) {
    if (ownReadinessFailure(*failure)) beginDropout(*failure);
    else stop(*failure, true, false);
    return;
  }
  const auto fresh_input = snapshot();
  if (config_.mode == "consensus" && acquire_stage_ &&
      *acquire_stage_ != "waiting_for_team" && !teamEnabled(now, fresh_input)) {
    beginDropout("peer_formation_stopped"); return;
  }
  if (config_.mode == "consensus" && acquire_stage_ &&
      *acquire_stage_ != "following" && acquire_started_ &&
      (now - *acquire_started_).toSec() > config_.acquire_timeout) {
    beginDropout("acquisition_timeout"); return;
  }
  try {
    const auto target = config_.mode == "consensus"
        ? acquisitionTarget(now, fresh_input, *own_state_)
        : formationTarget(now, fresh_input, *own_state_);
    rememberOwnReference(target);
    reference_pub_.publish(target);
  } catch (const tf2::TransformException& error) {
    beginDropout(std::string("transform_unavailable:") + error.what());
  } catch (const XmlRpc::XmlRpcException& error) {
    beginDropout(std::string("ros_parameter_error:") + error.getMessage());
  } catch (const std::exception& error) {
    beginDropout(std::string("invalid_formation_state:") + error.what());
  } catch (...) {
    beginDropout("unknown_control_error");
  }
}

FormationRuntime::FormationRuntime(ros::NodeHandle nh, Config config,
                                   std::shared_ptr<tf2_ros::Buffer> tf_buffer,
                                   bool external_telemetry)
    : nh_(std::move(nh)), config_(std::move(config)),
      shared_(std::make_shared<SharedData>()), tf_buffer_(std::move(tf_buffer)),
      external_telemetry_(external_telemetry) {
  config_.common_frame = cleanFrame(config_.common_frame);
  auto geometry = std::make_shared<FormationGeometry>();
  if (config_.mode == "consensus") {
    if (config_.members.size() < 2 || config_.common_frame.empty())
      throw std::invalid_argument("consensus requires members and common_frame");
    geometry->slots = formationOffsets(config_.shape, config_.members.size(),
                                       config_.spacing);
    for (std::size_t i = 0; i < config_.members.size(); ++i)
      geometry->slot_offsets.emplace(config_.members[i], geometry->slots[i]);
    config_.waypoints = roundedPath(config_.waypoints, config_.corner_radius,
                                    config_.loop_path);
    (void)pathReferenceState({0.0, 0.0, 0.0}, config_.waypoints,
                             config_.cruise_speed, config_.lookahead,
                             config_.loop_path);
  } else {
    geometry->member_offsets = config_.offsets;
    if (config_.shape != "manual") {
      geometry->slots = formationOffsets(config_.shape,
          static_cast<std::size_t>(config_.follower_count), config_.spacing);
      if (config_.slot_index < 1 || static_cast<std::size_t>(config_.slot_index) >
          geometry->slots.size())
        throw std::invalid_argument("slot_index is outside formation");
    }
  }
  geometry_ = std::move(geometry);
  if (!tf_buffer_) {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(ros::Duration(config_.tf_buffer_cache_time));
  }
  try {
    for (const auto& member : config_.members) {
      auto follower = std::make_unique<Follower>(nh_, config_, member,
          shared_, geometry_, tf_buffer_, true);
      follower_by_name_[member] = follower.get();
      followers_.push_back(std::move(follower));
    }
    std::vector<std::string> names = config_.members;
    if (config_.mode == "leader_follow" &&
        std::find(names.begin(), names.end(), config_.leader_name) == names.end())
      names.push_back(config_.leader_name);
    if (!external_telemetry_) {
      for (const auto& name : names) {
        subscribers_.push_back(nh_.subscribe<xd_uav_controller::ControlState>(
            "/" + name + "/control_manager/state", 1,
            [this, name](const xd_uav_controller::ControlState::ConstPtr& message) {
              stateCallback(message, name);
            }));
        subscribers_.push_back(nh_.subscribe<mavros_msgs::State>(
            "/" + name + "/mavros/state", 1,
            [this, name](const mavros_msgs::State::ConstPtr& message) {
              mavrosCallback(message, name);
            }));
      }
    }
    const std::string status_node = config_.mode == "consensus"
        ? "formation_consensus" : "formation_follower";
    for (const auto& member : config_.members) {
      subscribers_.push_back(nh_.subscribe<std_msgs::String>(
          "/" + member + "/" + status_node + "/status", 1,
          [this, member](const std_msgs::String::ConstPtr& message) {
            statusCallback(message, member);
          }));
      if (config_.mode == "consensus") {
        subscribers_.push_back(nh_.subscribe<geometry_msgs::PointStamped>(
            "/formation/" + config_.team_id + "/" + member + "/center", 1,
            [this, member](const geometry_msgs::PointStamped::ConstPtr& message) {
              centerCallback(message, member);
            }));
      }
    }
    std::string cancel = config_.team_cancel_topic;
    const std::string token = "{team_id}";
    auto at = cancel.find(token);
    if (at != std::string::npos) cancel.replace(at, token.size(), config_.team_id);
    if (cancel.empty() || cancel.front() != '/') cancel = "/" + cancel;
    subscribers_.push_back(nh_.subscribe<std_msgs::String>(cancel, 1,
        &FormationRuntime::teamCancelCallback, this));
    if (config_.mode == "leader_follow") {
      subscribers_.push_back(nh_.subscribe<xd_uav_task_execute::ExecuteTaskActionGoal>(
          "/" + config_.leader_name + "/task_execute/execute/goal", 10,
          [this](const xd_uav_task_execute::ExecuteTaskActionGoal::ConstPtr& message) {
            leaderTaskCallback(message);
          }));
      subscribers_.push_back(nh_.subscribe<std_msgs::String>(
          "/" + config_.leader_name + "/formation/preempt", 10,
          [this](const std_msgs::String::ConstPtr& message) {
            leaderPreemptCallback(message);
          }));
    }
  } catch (...) {
    shutdown();
    throw;
  }
  ROS_INFO("[xd_uav_formation] shared %s C++ runtime ready for %zu member(s)",
           config_.mode.c_str(), followers_.size());
}

FormationRuntime::~FormationRuntime() { shutdown(); }

void FormationRuntime::stateCallback(
    const xd_uav_controller::ControlState::ConstPtr& message,
    const std::string& name) {
  const ros::Time received = ros::Time::now();
  {
    std::lock_guard<std::mutex> lock(shared_->mutex);
    shared_->states[name] = *message;
    shared_->state_received[name] = received;
    shared_->markChanged();
  }
  auto own = follower_by_name_.find(name);
  if (own != follower_by_name_.end()) own->second->stateInput(*message, received);
  else if (config_.mode == "leader_follow")
    for (auto& follower : followers_) follower->leaderStateInput(*message, received);
}

void FormationRuntime::mavrosCallback(const mavros_msgs::State::ConstPtr& message,
                                      const std::string& name) {
  const ros::Time received = ros::Time::now();
  {
    std::lock_guard<std::mutex> lock(shared_->mutex);
    shared_->mavros[name] = *message;
    shared_->mavros_received[name] = received;
    shared_->markChanged();
  }
  auto own = follower_by_name_.find(name);
  if (own != follower_by_name_.end()) own->second->mavrosInput(*message, received);
  else if (config_.mode == "leader_follow")
    for (auto& follower : followers_) follower->leaderMavrosInput(*message, received);
}

void FormationRuntime::centerCallback(
    const geometry_msgs::PointStamped::ConstPtr& message,
    const std::string& name) {
  // Local Follower::publishCenter writes this shared cache before publishing.
  if (follower_by_name_.count(name)) return;
  std::lock_guard<std::mutex> lock(shared_->mutex);
  shared_->centers[name] = *message;
  shared_->center_received[name] = ros::Time::now();
  shared_->markChanged();
}

void FormationRuntime::statusCallback(const std_msgs::String::ConstPtr& message,
                                      const std::string& name) {
  const ros::Time received = ros::Time::now();
  if (!follower_by_name_.count(name)) {
    std::lock_guard<std::mutex> lock(shared_->mutex);
    shared_->statuses[name] = message->data;
    shared_->status_received[name] = received;
    shared_->markChanged();
  }
  for (auto& follower : followers_)
    if (follower->member() != name)
      follower->peerStatusInput(*message, name, received);
}

void FormationRuntime::teamCancelCallback(const std_msgs::String::ConstPtr& message) {
  for (auto& follower : followers_) follower->teamCancelInput(*message);
}

void FormationRuntime::leaderTaskCallback(
    const xd_uav_task_execute::ExecuteTaskActionGoal::ConstPtr& message) {
  for (auto& follower : followers_) follower->leaderTaskInput(*message);
}

void FormationRuntime::leaderPreemptCallback(const std_msgs::String::ConstPtr& message) {
  for (auto& follower : followers_) follower->leaderPreemptInput(*message);
}

bool FormationRuntime::setMembers(bool enabled, std::string& message) {
  std::vector<Follower*> completed;
  for (auto& follower : followers_) {
    std::string result;
    if (!follower->setEnabled(enabled, result)) {
      message = follower->member() + ": " + result;
      if (enabled) {
        for (auto it = completed.rbegin(); it != completed.rend(); ++it) {
          std::string ignored;
          (*it)->setEnabled(false, ignored);
        }
        if (config_.mode == "consensus") clearAcquisitionPlan();
        return false;
      }
    } else completed.push_back(follower.get());
  }
  if (config_.mode == "consensus" && !enabled) clearAcquisitionPlan();
  message = enabled ? "team started" : "team stopped";
  return true;
}

void FormationRuntime::clearAcquisitionPlan() {
  {
    std::lock_guard<std::mutex> lock(shared_->acquisition_plan_mutex);
    shared_->acquisition_plan.reset();
  }
  nh_.deleteParam("/formation/" + config_.team_id + "/acquisition_plan");
}

void FormationRuntime::shutdown() {
  for (auto& subscriber : subscribers_) subscriber.shutdown();
  subscribers_.clear();
  for (auto& follower : followers_) follower.reset();
  followers_.clear();
  follower_by_name_.clear();
}

}  // namespace xd_uav_formation


