#include <xd_uav_formation/runtime.hpp>

#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <xmlrpcpp/XmlRpcException.h>
#include <xmlrpcpp/XmlRpcValue.h>

#include <algorithm>
#include <cmath>
#include <iterator>
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

std::vector<Vec3> centeredSlots(const std::string& shape, std::size_t count,
                               double spacing) {
  if (count == 0) return {};
  auto slots = formationOffsets(shape, count, spacing);
  Vec3 mean{0.0, 0.0, 0.0};
  for (const auto& slot : slots)
    for (int axis = 0; axis < 3; ++axis)
      mean[axis] += slot[axis] / static_cast<double>(slots.size());
  for (auto& slot : slots)
    for (int axis = 0; axis < 3; ++axis) slot[axis] -= mean[axis];
  return slots;
}

FormationGroupPlan makeGroupPlan(const std::vector<std::string>& members,
                                 int group_size) {
  if (group_size < 1) throw std::invalid_argument("group_size must be positive");
  FormationGroupPlan plan;
  plan.group_size = group_size;
  if (members.empty()) return plan;
  const std::size_t count = members.size();
  const std::size_t groups = (count + static_cast<std::size_t>(group_size) - 1) /
      static_cast<std::size_t>(group_size);
  const std::size_t base = count / groups;
  const std::size_t remainder = count % groups;
  std::size_t cursor = 0;
  for (std::size_t index = 0; index < groups; ++index) {
    const std::size_t size = base + (index < remainder ? 1 : 0);
    FormationGroup group;
    group.id = "group_" + std::to_string(index + 1);
    group.members.insert(group.members.end(), members.begin() + cursor,
                         members.begin() + cursor + size);
    group.nominal_size = size;
    cursor += size;
    plan.groups.push_back(std::move(group));
  }
  return plan;
}

std::map<std::string, Vec3> hierarchicalSlots(
    const FormationGroupPlan& plan, const std::string& shape,
    double inner_spacing, double requested_group_spacing,
    double minimum_distance) {
  std::vector<const FormationGroup*> active_groups;
  double maximum_inner_radius = 0.0;
  std::map<std::string, std::vector<Vec3>> inner_slots;
  for (const auto& group : plan.groups) {
    if (group.detached) continue;
    // Keep empty attached groups in the outer layout so a local dropout does
    // not move the other groups into different outer slots.
    active_groups.push_back(&group);
    auto slots = centeredSlots(shape, group.members.size(), inner_spacing);
    const auto nominal_slots = centeredSlots(shape,
        std::max(group.nominal_size, group.members.size()), inner_spacing);
    for (const auto& slot : nominal_slots)
      maximum_inner_radius = std::max(maximum_inner_radius,
                                      std::hypot(slot[0], slot[1]));
    inner_slots.emplace(group.id, std::move(slots));
  }
  std::map<std::string, Vec3> result;
  if (active_groups.empty()) return result;
  if (plan.group_size == 1) {
    // A one-aircraft group is still part of the single fleet formation. Place
    // each group center directly on the fleet shape using the normal spacing;
    // group_spacing only separates multi-aircraft groups from one another.
    const auto slots = centeredSlots(shape, active_groups.size(), inner_spacing);
    for (std::size_t i = 0; i < active_groups.size(); ++i) {
      const auto& group = *active_groups[i];
      if (group.members.empty()) continue;
      if (group.members.size() != 1)
        throw std::invalid_argument("group_size=1 requires singleton formation groups");
      result[group.members.front()] = slots[i];
    }
    for (auto first = result.begin(); first != result.end(); ++first) {
      for (auto second = std::next(first); second != result.end(); ++second) {
        const double dx = first->second[0] - second->second[0];
        const double dy = first->second[1] - second->second[1];
        const double dz = first->second[2] - second->second[2];
        if (std::sqrt(dx * dx + dy * dy + dz * dz) < minimum_distance)
          throw std::invalid_argument(
              "single-aircraft formation slots are closer than minimum separation");
      }
    }
    return result;
  }
  const double safe_pitch = 2.0 * maximum_inner_radius + minimum_distance;
  double group_spacing = std::max(
      requested_group_spacing > 0.0 ? requested_group_spacing : inner_spacing,
      safe_pitch);
  for (int attempt = 0; attempt < 8; ++attempt) {
    const auto outer_slots = centeredSlots(shape, active_groups.size(), group_spacing);
    result.clear();
    for (std::size_t group_index = 0; group_index < active_groups.size(); ++group_index) {
      const auto& group = *active_groups[group_index];
      const auto& locals = inner_slots.at(group.id);
      for (std::size_t member_index = 0; member_index < group.members.size(); ++member_index) {
        result[group.members[member_index]] = {
            outer_slots[group_index][0] + locals[member_index][0],
            outer_slots[group_index][1] + locals[member_index][1],
            outer_slots[group_index][2] + locals[member_index][2]};
      }
    }
    bool separated = true;
    for (auto first = result.begin(); first != result.end() && separated; ++first) {
      for (auto second = std::next(first); second != result.end(); ++second) {
        const double dx = first->second[0] - second->second[0];
        const double dy = first->second[1] - second->second[1];
        const double dz = first->second[2] - second->second[2];
        if (std::sqrt(dx * dx + dy * dy + dz * dz) < minimum_distance) {
          separated = false;
          break;
        }
      }
    }
    if (separated) return result;
    group_spacing *= 1.35;
  }
  throw std::invalid_argument("nested formation slots are closer than minimum separation");
}

bool sameMembers(const std::vector<std::string>& first,
                 const std::vector<std::string>& second) {
  return first.size() == second.size() &&
      std::set<std::string>(first.begin(), first.end()) ==
      std::set<std::string>(second.begin(), second.end());
}

bool compatibleSlotAssignment(const FormationGroupPlan& groups,
                              const FormationSlotAssignment& assignment,
                              const std::string& shape) {
  if (assignment.shape != shape ||
      assignment.ordered_groups.revision != groups.revision ||
      assignment.ordered_groups.groups.size() != groups.groups.size()) return false;
  for (const auto& group : groups.groups) {
    const auto& ordered = assignment.ordered_groups.groups;
    if (std::none_of(ordered.begin(), ordered.end(), [&](const auto& entry) {
          return entry.id == group.id;
        })) return false;
  }
  return true;
}

FormationGroupPlan applySlotOrder(const FormationGroupPlan& groups,
                                  const FormationSlotAssignment& assignment) {
  FormationGroupPlan result = groups;
  result.groups.clear();
  for (const auto& ordered : assignment.ordered_groups.groups) {
    const auto found = std::find_if(groups.groups.begin(), groups.groups.end(),
        [&](const auto& group) { return group.id == ordered.id; });
    if (found == groups.groups.end()) continue;
    auto group = *found;
    if (sameMembers(group.members, ordered.members)) group.members = ordered.members;
    result.groups.push_back(std::move(group));
  }
  return result;
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
  parameter["selection_revision"] =
      static_cast<int>(plan.selection_revision);
  parameter["group_revision"] = static_cast<int>(plan.group_revision);
  return parameter;
}

}  // namespace

FormationSlotAssignment assignFormationSlots(
    const FormationGroupPlan& groups,
    const std::optional<FormationSlotAssignment>& previous,
    const std::map<std::string, Vec3>& positions, const std::string& shape,
    double spacing, double group_spacing, double minimum_distance,
    double heading, double altitude) {
  const bool reuse_outer = previous &&
      compatibleSlotAssignment(groups, *previous, shape);
  auto ordered = reuse_outer ? applySlotOrder(groups, *previous) : groups;
  if (!reuse_outer) {
    const auto offsets = hierarchicalSlots(ordered, shape, spacing,
                                           group_spacing, minimum_distance);
    std::vector<Vec3> centers, outer_offsets;
    std::vector<std::size_t> movable;
    for (std::size_t i = 0; i < ordered.groups.size(); ++i) {
      const auto& group = ordered.groups[i];
      if (group.detached || group.members.empty()) continue;
      std::vector<Vec3> points, local_offsets;
      for (const auto& name : group.members) {
        points.push_back(positions.at(name));
        local_offsets.push_back(offsets.at(name));
      }
      centers.push_back(sharedCenter(points).first);
      outer_offsets.push_back(sharedCenter(local_offsets).first);
      movable.push_back(i);
    }
    if (!centers.empty()) {
      const auto origin = sharedCenter(centers).first;
      std::vector<Vec3> goals;
      for (const auto& offset : outer_offsets) {
        const auto slot = slotTarget(origin, heading, offset);
        goals.push_back({slot[0], slot[1], altitude});
      }
      const auto assignment = minimumDistanceAssignment(centers, goals);
      const auto original = ordered.groups;
      for (std::size_t i = 0; i < movable.size(); ++i)
        ordered.groups[movable[assignment[i]]] = original[movable[i]];
    }
  }

  // Use the same translated/rotated targets as acquisition. Optimize each
  // group's members independently so external-control group membership stays
  // intact. A local dropout only recomputes that group's inner ordering.
  const auto offsets = hierarchicalSlots(ordered, shape, spacing,
                                         group_spacing, minimum_distance);
  std::vector<std::string> names;
  std::vector<Vec3> starts, local_offsets;
  for (const auto& group : ordered.groups) {
    if (group.detached) continue;
    for (const auto& name : group.members) {
      names.push_back(name);
      starts.push_back(positions.at(name));
      local_offsets.push_back(offsets.at(name));
    }
  }
  if (names.empty()) return {shape, std::move(ordered)};
  const auto slots = acquisitionSlots(starts, local_offsets, heading, altitude);
  std::map<std::string, Vec3> goals;
  for (std::size_t i = 0; i < names.size(); ++i) goals[names[i]] = slots[i];
  for (auto& group : ordered.groups) {
    if (group.detached || group.members.size() < 2) continue;
    if (reuse_outer) {
      const auto& old_groups = previous->ordered_groups.groups;
      const auto old = std::find_if(old_groups.begin(), old_groups.end(),
          [&](const auto& entry) { return entry.id == group.id; });
      if (old != old_groups.end() && sameMembers(old->members, group.members)) continue;
    }
    std::vector<Vec3> points, targets;
    for (const auto& name : group.members) {
      points.push_back(positions.at(name));
      targets.push_back(goals.at(name));
    }
    const auto assignment = minimumDistanceAssignment(points, targets);
    const auto members = group.members;
    for (std::size_t i = 0; i < members.size(); ++i)
      group.members[assignment[i]] = members[i];
  }
  return {shape, std::move(ordered)};
}

Follower::Follower(ros::NodeHandle nh, const Config& config, std::string member,
                   std::shared_ptr<SharedData> shared,
                   std::shared_ptr<const FormationGeometry> geometry,
                   std::shared_ptr<tf2_ros::Buffer> tf_buffer,
                   bool shared_inputs)
    : nh_(std::move(nh)), config_(config), member_(std::move(member)),
      shared_(std::move(shared)), geometry_(std::move(geometry)),
      tf_buffer_(std::move(tf_buffer)), shared_inputs_(shared_inputs) {
  if (!shared_) shared_ = std::make_shared<SharedData>();
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
    formation_selection_ = shared_->formationSelection();
    formation_group_revision_ = shared_->formationGroupPlan().revision;
    if (config_.members.size() < 2 || config_.common_frame.empty()) {
      throw std::invalid_argument("consensus requires members and common_frame");
    }
    const auto own_slot_index = static_cast<std::size_t>(
        std::find(config_.members.begin(), config_.members.end(), member_) -
        config_.members.begin());
    offset_ = geometry_->slots.at(own_slot_index);
    if (!std::isfinite(offset_[0]) || !std::isfinite(offset_[1]) ||
        !std::isfinite(offset_[2])) {
      throw std::invalid_argument("consensus slot must be finite");
    }
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
  require_finite_positive(config_.fixedwing_max_speed, "fixedwing_max_speed");
  if (config_.fixedwing_max_speed <= config_.fixedwing_min_speed) {
    throw std::invalid_argument(
        "fixedwing_max_speed must exceed fixedwing_min_speed");
  }
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
    if (config_.cruise_speed <= config_.fixedwing_min_speed ||
        config_.cruise_speed >= config_.fixedwing_max_speed) {
      throw std::invalid_argument(
          "consensus cruise_speed must be inside the fixed-wing speed limits");
    }
    require_finite_positive(config_.formation_speed_slew_rate,
                             "formation_speed_slew_rate");
    require_finite_positive(config_.formation_course_slew_rate,
                             "formation_course_slew_rate");
    require_finite_positive(config_.route_correction_gain,
                             "route_correction_gain", true);
    require_finite_positive(config_.route_velocity_damping,
                             "route_velocity_damping", true);
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
  if (enabled_ && config_.mode == "consensus" &&
      status.data.rfind("inactive:dropout", 0) == 0) {
    const auto own_group = formationGroupFor(member_);
    const auto peer_group = formationGroupFor(member);
    if (own_group && peer_group && own_group->detached &&
        own_group->id == peer_group->id && own_group->anchor == member)
      return;
  }
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
  const ros::Time received = ros::Time::now();
  last_external_reference_ = received;
  if (enabled_) {
    beginDropout("external_setpoint_from:" + caller,
                 controlPointFromReference(*message, received));
  } else if (config_.mode == "consensus" && shared_) {
    const auto group = formationGroupFor(member_);
    if (group && group->detached && group->anchor == member_ &&
        group->anchor_external_control) {
      // Keep the group's point current for the entire external setpoint
      // stream, even after the anchor's formation follower has dropped out.
      shared_->detachFormationGroup(member_,
          controlPointFromReference(*message, received));
    }
  }
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
  if (config_.mode == "consensus" && enabled_ && acquire_stage_ &&
      *acquire_stage_ != "following") {
    restartAcquisitionForMembershipChange("excluded:" + member + ":" + reason);
  }
}

std::optional<FormationControlPoint> Follower::controlPointFromReference(
    const mavros_msgs::PositionTarget& message,
    const ros::Time& received) const {
  if (config_.mode != "consensus") return std::nullopt;
  const std::uint16_t ignored_position =
      mavros_msgs::PositionTarget::IGNORE_PX |
      mavros_msgs::PositionTarget::IGNORE_PY |
      mavros_msgs::PositionTarget::IGNORE_PZ;
  if ((message.type_mask & ignored_position) != 0 ||
      !std::isfinite(message.position.x) ||
      !std::isfinite(message.position.y) ||
      !std::isfinite(message.position.z)) {
    return std::nullopt;
  }

  geometry_msgs::PoseStamped pose;
  pose.header = message.header;
  if (pose.header.frame_id.empty()) {
    if (!own_state_ || own_state_->header.frame_id.empty()) return std::nullopt;
    pose.header.frame_id = own_state_->header.frame_id;
  }
  pose.header.frame_id = cleanFrame(pose.header.frame_id);
  if (pose.header.stamp.isZero()) pose.header.stamp = received;
  pose.pose.position = message.position;
  const bool heading_valid =
      (message.type_mask & mavros_msgs::PositionTarget::IGNORE_YAW) == 0 &&
      std::isfinite(message.yaw);
  if (heading_valid) {
    tf2::Quaternion orientation;
    orientation.setRPY(0.0, 0.0, message.yaw);
    pose.pose.orientation = tf2::toMsg(orientation);
  } else {
    pose.pose.orientation.w = 1.0;
  }

  try {
    if (pose.header.frame_id != config_.common_frame) {
      if (!tf_buffer_) return std::nullopt;
      pose = tf_buffer_->transform(pose, config_.common_frame,
          ros::Duration(config_.tf_lookup_timeout));
    }
    FormationControlPoint point;
    point.position = pointVec(pose.pose.position);
    point.heading_valid = heading_valid;
    if (heading_valid) point.heading = yaw(pose);
    point.stamp = received;
    return point;
  } catch (const tf2::TransformException& error) {
    ROS_WARN_THROTTLE(2.0,
        "[xd_uav_formation] cannot transform external control point to %s: %s",
        config_.common_frame.c_str(), error.what());
    return std::nullopt;
  } catch (const std::exception& error) {
    ROS_WARN_THROTTLE(2.0,
        "[xd_uav_formation] cannot use external control point: %s",
        error.what());
    return std::nullopt;
  }
}

void Follower::beginDropout(
    const std::string& reason,
    const std::optional<FormationControlPoint>& control_point) {
  const bool was_enabled = enabled_;
  const bool external_control =
      reason.rfind("external_setpoint", 0) == 0 ||
      reason == "external_path" || reason == "external_trajectory" ||
      reason == "follower_flight_action" ||
      reason.rfind("preempt:", 0) == 0 || reason.rfind("task_goal:", 0) == 0;
  if (config_.mode == "consensus" && external_control && shared_) {
    if (shared_->detachFormationGroup(member_, control_point)) {
      ROS_WARN("[xd_uav_formation] detaching group containing %s; member becomes its external-control anchor",
               member_.c_str());
    }
  }
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
  auto own_group = formationGroupFor(member_);
  if (own_group && own_group->detached && own_group->anchor != member_) {
    auto anchorUsable = [&](const std::string& name, bool external_control) {
      const auto state = input.states.find(name);
      const auto state_time = input.state_received.find(name);
      const auto mavros = input.mavros.find(name);
      const auto mavros_time = input.mavros_received.find(name);
      if (state == input.states.end() || state_time == input.state_received.end() ||
          !fresh(state->second.header, true, state_time->second, now) ||
          !state->second.state_valid || state->second.header.frame_id.empty() ||
          state->second.transition_pending ||
          state->second.airframe_type != own_state_->airframe_type ||
          state->second.vehicle_type != own_state_->vehicle_type ||
          state->second.flight_regime != own_state_->flight_regime ||
          (!external_control && blockingAction(state->second))) {
        return false;
      }
      if (mavros == input.mavros.end() ||
          mavros_time == input.mavros_received.end() ||
          std::max(0.0, (now - mavros_time->second).toSec()) >
              config_.mavros_timeout ||
          !mavros->second.armed ||
          (!external_control && mavros->second.mode != "OFFBOARD")) {
        return false;
      }
      return true;
    };

    const std::string old_anchor = own_group->anchor;
    if (!anchorUsable(old_anchor, own_group->anchor_external_control)) {
      std::string replacement;
      for (const auto& name : own_group->members) {
        if (name == old_anchor) continue;
        const auto status = input.statuses.find(name);
        const auto status_time = input.status_received.find(name);
        if (status == input.statuses.end() ||
            status_time == input.status_received.end() ||
            status->second.rfind("active:", 0) != 0 ||
            std::max(0.0, (now - status_time->second).toSec()) >
                config_.mavros_timeout ||
            !anchorUsable(name, false)) {
          continue;
        }
        replacement = name;
        break;
      }

      if (!replacement.empty() &&
          shared_->promoteDetachedGroupAnchor(old_anchor, replacement)) {
        dropped_members_.insert(old_anchor);
        dropped_members_.erase(replacement);
        active_members_dirty_ = true;
        own_group = formationGroupFor(member_);
        ROS_WARN("[xd_uav_formation] group %s anchor %s unavailable; promoting %s",
                 own_group ? own_group->id.c_str() : "unknown",
                 old_anchor.c_str(), replacement.c_str());
      } else {
        const auto current_group = formationGroupFor(member_);
        if (current_group && current_group->detached &&
            current_group->anchor == old_anchor) {
          beginDropout("detached_group_anchor_lost");
          return;
        }
        if (current_group && current_group->detached &&
            current_group->anchor != old_anchor) {
          dropped_members_.insert(old_anchor);
          active_members_dirty_ = true;
        }
        own_group = current_group;
      }
    }
  }
  for (const auto& name : config_.members) {
    if (name == member_) continue;
    const auto peer_group = formationGroupFor(name);
    if (own_group && own_group->detached) {
      if (!peer_group || peer_group->id != own_group->id ||
          name == own_group->anchor) continue;
    } else if (peer_group && peer_group->detached) {
      continue;
    }
    const auto status = input.statuses.find(name);
    const auto status_received = input.status_received.find(name);
    const bool status_fresh = status != input.statuses.end() &&
        status_received != input.status_received.end() &&
        std::max(0.0, (now - status_received->second).toSec()) <=
            config_.mavros_timeout;
    const bool was_dropped = dropped_members_.count(name) != 0;
    const bool status_active = status_fresh &&
        status->second.rfind("active:", 0) == 0;
    if (was_dropped && !status_active) continue;
    if (!was_dropped && status != input.statuses.end() &&
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
    if (error.empty() && !usesFixedwingConsensusGuidance(*own_state_)) {
      const auto center = input.centers.find(name);
      const auto received = input.center_received.find(name);
      if (center == input.centers.end() || received == input.center_received.end() ||
          !fresh(center->second.header, true, received->second, now)) {
        error = "center_stale";
      } else if (cleanFrame(center->second.header.frame_id) != config_.common_frame) {
        error = "center_frame_mismatch";
      }
    }
    if (!error.empty()) {
      if (!was_dropped) markPeerDropped(name, error);
    } else if (was_dropped) {
      dropped_members_.erase(name);
      active_members_dirty_ = true;
      ROS_INFO("[xd_uav_formation] restoring %s to the formation", name.c_str());
      if (acquire_stage_ && *acquire_stage_ != "following")
        restartAcquisitionForMembershipChange("restored:" + name);
    }
  }
}

void Follower::restartAcquisitionForMembershipChange(const std::string& reason) {
  if (config_.mode != "consensus" || !enabled_) return;
  acquire_force_capture_ = true;
  acquire_heading_.reset();
  acquire_altitude_.reset();
  acquire_stage_ = "waiting_for_team";
  acquire_started_ = ros::Time::now();
  consensus_heading_.reset();
  consensus_heading_stamp_.reset();
  paced_speed_.reset();
  paced_speed_stamp_.reset();
  progress_error_filter_.clear();
  resetConsensusMotion();
  clearAcquisitionPlan();
  nh_.deleteParam("/formation/" + config_.team_id + "/acquisition_plan");
  reason_ = "acquiring_waiting_for_team:" + reason;
  publishStatus();
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
    if (config_.mode == "consensus") {
      const auto offsets = consensusSlotOffsets();
      const auto own_offset = offsets.find(member_);
      if (own_offset == offsets.end()) return "missing_consensus_slot";
      for (const auto& entry : offsets) {
        if (entry.first == member_) continue;
        const double dx = own_offset->second[0] - entry.second[0];
        const double dy = own_offset->second[1] - entry.second[1];
        const double dz = own_offset->second[2] - entry.second[2];
        if (std::sqrt(dx * dx + dy * dy + dz * dz) <
            config_.fixedwing_minimum_slot_distance) {
          return "fixedwing_slot_too_close";
        }
      }
    } else {
      const double offset_length = std::sqrt(offset_[0] * offset_[0] +
          offset_[1] * offset_[1] + offset_[2] * offset_[2]);
      if (offset_length < config_.fixedwing_minimum_slot_distance)
        return "fixedwing_slot_too_close";
    }
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
    const auto own_group = formationGroupFor(member_);
    const bool external_group_anchor = own_group && own_group->detached &&
        own_group->anchor_external_control && own_group->anchor == peer.name;
    if (blockingAction(state) && !external_group_anchor)
      return peer.name + "_flight_action";
    if (state.header.frame_id.empty()) return "state_frame_missing";
    if (!peer.has_mavros || !peer.mavros ||
        (now - peer.mavros_received).toSec() > config_.mavros_timeout ||
        !peer.mavros->armed ||
        (config_.mode == "consensus" && peer.mavros->mode != "OFFBOARD" &&
         !external_group_anchor)) {
      return config_.mode == "leader_follow" ? "leader_armed_required" :
             "peer_offboard_required:" + peer.name;
    }
  }
  // Fixed-wing geometry and detached following use validated peer states
  // directly. Requiring a previous center heartbeat adds a startup race,
  // and can falsely exclude a healthy aircraft before its first heartbeat.
  if (config_.mode == "consensus" && !usesFixedwingConsensusGuidance(own)) {
    const auto own_group = formationGroupFor(member_);
    for (const auto& name : activeMembers()) {
      if (name == member_) continue;
      if (own_group && own_group->detached && name == own_group->anchor) continue;
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
  const auto plan = formationGroupPlan();
  if (plan.revision != cached_group_revision_) {
    active_members_dirty_ = true;
    cached_group_revision_ = plan.revision;
  }
  if (active_members_dirty_) {
    active_members_cache_.clear();
    const auto own_group = formationGroupFor(member_);
    for (const auto& group : plan.groups) {
      const bool own_detached_group = own_group && own_group->detached &&
          own_group->id == group.id;
      if (group.detached && !own_detached_group) continue;
      if (!group.detached && own_group && own_group->detached) continue;
      for (const auto& name : group.members) {
        if (!dropped_members_.count(name)) active_members_cache_.push_back(name);
      }
    }
    active_members_dirty_ = false;
  }
  return active_members_cache_;
}

FormationGroupPlan Follower::formationGroupPlan() const {
  return shared_ ? shared_->formationGroupPlan() : FormationGroupPlan{};
}

std::optional<FormationGroup> Follower::formationGroupFor(
    const std::string& member) const {
  const auto plan = formationGroupPlan();
  for (const auto& group : plan.groups) {
    if (std::find(group.members.begin(), group.members.end(), member) !=
        group.members.end()) return group;
  }
  return std::nullopt;
}

std::vector<FormationGroup> Follower::activeFormationGroups() const {
  const auto plan = formationGroupPlan();
  const auto own_group = formationGroupFor(member_);
  std::vector<FormationGroup> groups;
  for (const auto& source : plan.groups) {
    const bool own_detached_group = own_group && own_group->detached &&
        own_group->id == source.id;
    if (source.detached && !own_detached_group) continue;
    if (!source.detached && own_group && own_group->detached) continue;
    FormationGroup group = source;
    group.members.erase(std::remove_if(group.members.begin(), group.members.end(),
        [&](const std::string& name) { return dropped_members_.count(name) != 0; }),
        group.members.end());
    if (!group.members.empty()) groups.push_back(std::move(group));
  }
  return groups;
}

void Follower::refreshFormationSelection(const ros::Time& now) {
  if (config_.mode != "consensus" || !shared_) return;
  FormationSelection selection = shared_->formationSelection();
  if (selection.revision == formation_selection_.revision) return;
  formation_selection_ = std::move(selection);
  resetConsensusMotion();
  consensus_heading_.reset();
  consensus_heading_stamp_.reset();
  paced_speed_.reset();
  paced_speed_stamp_.reset();
  progress_error_filter_.clear();
  if (enabled_) {
    const auto group = formationGroupFor(member_);
    if (group && group->detached) {
      acquire_stage_.reset();
      acquire_started_.reset();
      reason_ = "following_detached_group:" + group->anchor;
    } else {
      acquire_heading_.reset();
      acquire_stage_ = "waiting_for_team";
      acquire_started_ = now;
      reason_ = "acquiring_waiting_for_team";
    }
    publishStatus();
  }
}

std::map<std::string, Vec3> Follower::consensusSlotOffsets() const {
  const auto own_group = formationGroupFor(member_);
  auto groups = activeFormationGroups();
  if (own_group && own_group->detached) {
    std::map<std::string, Vec3> offsets;
    for (const auto& group : groups) {
      std::vector<std::string> order;
      {
        std::lock_guard<std::mutex> lock(shared_->slot_assignment_mutex);
        auto& cached = shared_->group_slot_orders[group.id];
        // Retain slots occupied by dropped members so surviving aircraft do
        // not exchange roles when an anchor is replaced or a member fails.
        cached.erase(std::remove_if(cached.begin(), cached.end(),
            [&](const std::string& name) {
              return std::find(own_group->members.begin(), own_group->members.end(), name) ==
                  own_group->members.end();
            }), cached.end());
        for (const auto& name : group.members)
          if (std::find(cached.begin(), cached.end(), name) == cached.end())
            cached.push_back(name);
        order = cached;
      }
      const auto slots = centeredSlots(formation_selection_.shape,
                                       order.size(), config_.spacing);
      for (std::size_t i = 0; i < order.size(); ++i)
        offsets[order[i]] = slots[i];
    }
    return offsets;
  }
  FormationGroupPlan plan = formationGroupPlan();
  plan.groups.erase(std::remove_if(plan.groups.begin(), plan.groups.end(),
      [](const auto& group) { return group.detached; }), plan.groups.end());
  for (auto& group : plan.groups) {
    group.members.erase(std::remove_if(group.members.begin(), group.members.end(),
        [&](const std::string& name) {
          return dropped_members_.count(name) != 0;
        }), group.members.end());
  }
  {
    std::lock_guard<std::mutex> lock(shared_->slot_assignment_mutex);
    if (shared_->slot_assignment && shared_->slot_assignment->selection_revision ==
          formation_selection_.revision && compatibleSlotAssignment(
          plan, *shared_->slot_assignment, formation_selection_.shape))
      plan = applySlotOrder(plan, *shared_->slot_assignment);
  }
  return hierarchicalSlots(plan, formation_selection_.shape,
      config_.spacing, config_.group_spacing,
      std::max(config_.minimum_slot_distance,
               config_.fixedwing_minimum_slot_distance));
}

void Follower::ensureSlotAssignment(const ros::Time& now,
                                    const StateSnapshot& input) const {
  auto plan = formationGroupPlan();
  const auto own_group = formationGroupFor(member_);
  if (own_group && own_group->detached) return;
  plan.groups.erase(std::remove_if(plan.groups.begin(), plan.groups.end(),
      [](const auto& group) { return group.detached; }), plan.groups.end());
  for (auto& group : plan.groups) {
    group.members.erase(std::remove_if(group.members.begin(), group.members.end(),
        [&](const auto& name) { return dropped_members_.count(name) != 0; }),
        group.members.end());
  }
  // The first controller publishes the result while holding this lock. All
  // other controllers reuse it, even if they see slightly newer telemetry.
  std::lock_guard<std::mutex> lock(shared_->slot_assignment_mutex);
  const auto& cached = shared_->slot_assignment;
  const bool current_selection = cached &&
      cached->selection_revision == formation_selection_.revision;
  if (current_selection && compatibleSlotAssignment(plan, *cached, formation_selection_.shape)) {
    bool complete = true;
    for (const auto& group : plan.groups) {
      const auto& old = cached->ordered_groups.groups;
      const auto found = std::find_if(old.begin(), old.end(),
          [&](const auto& entry) { return entry.id == group.id; });
      complete = complete && found != old.end() && sameMembers(found->members, group.members);
    }
    if (complete) return;
  }
  const auto members = activeMembers();
  const auto points = memberPositionsInCommonFrame(now, input);
  std::map<std::string, Vec3> positions;
  for (std::size_t i = 0; i < members.size(); ++i) positions[members[i]] = points.at(i);
  std::vector<Vec3> centers;
  for (const auto& group : plan.groups) {
    std::vector<Vec3> group_points;
    for (const auto& name : group.members) group_points.push_back(positions.at(name));
    if (!group_points.empty()) centers.push_back(sharedCenter(group_points).first);
  }
  if (centers.empty()) throw std::invalid_argument("no active groups for slot assignment");
  const auto route = pathReferenceState(sharedCenter(centers).first,
      formation_selection_.waypoints, config_.cruise_speed, config_.lookahead,
      formation_selection_.loop_path);
  shared_->slot_assignment = assignFormationSlots(plan,
      current_selection ? cached : std::nullopt, positions,
      formation_selection_.shape, config_.spacing, config_.group_spacing,
      std::max(config_.minimum_slot_distance, config_.fixedwing_minimum_slot_distance),
      acquisitionHeading(now, input, std::atan2(route.velocity[1], route.velocity[0])),
      route.position[2]);
  shared_->slot_assignment->selection_revision = formation_selection_.revision;
  for (const auto& group : shared_->slot_assignment->ordered_groups.groups)
    shared_->group_slot_orders[group.id] = group.members;
}

void Follower::refreshFormationGroups(const ros::Time& now) {
  if (config_.mode != "consensus" || !shared_) return;
  const auto plan = shared_->formationGroupPlan();
  if (plan.revision == formation_group_revision_) return;
  formation_group_revision_ = plan.revision;
  active_members_dirty_ = true;
  const auto group = formationGroupFor(member_);
  if (group && group->detached) {
    if (cached_group_anchor_ && *cached_group_anchor_ != group->anchor)
      dropped_members_.insert(*cached_group_anchor_);
    cached_group_anchor_ = group->anchor;
  } else {
    cached_group_anchor_.reset();
  }
  if (!enabled_) return;
  if (group && group->detached) {
    acquire_stage_.reset();
    acquire_started_.reset();
    reason_ = "following_detached_group:" + group->anchor;
    resetCatchupPid();
    resetConsensusMotion();
    filtered_course_.reset();
    filtered_course_stamp_.reset();
    filtered_speed_.reset();
    publishStatus();
  } else {
    restartAcquisitionForMembershipChange("group_layout_changed");
  }
  (void)now;
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

void Follower::resetConsensusMotion() {
  consensus_correction_ = {0, 0, 0};
  consensus_correction_stamp_.reset();
  consensus_closing_velocity_ = {0, 0, 0};
  consensus_closing_velocity_stamp_.reset();
  column_reference_velocity_.reset();
  column_reference_stamp_.reset();
}

Vec3 Follower::filteredConsensusClosingVelocity(const Vec3& velocity,
    const ros::Time& now) {
  if (!consensus_closing_velocity_stamp_) {
    consensus_closing_velocity_ = velocity;
    consensus_closing_velocity_stamp_ = now;
  } else {
    const double elapsed = (now - *consensus_closing_velocity_stamp_).toSec();
    if (elapsed > 0.0) {
      const double dt = std::min(elapsed, 0.2);
      const double time_constant = config_.catchup_derivative_filter_time_constant;
      const double alpha = time_constant > 0.0 ? dt / (time_constant + dt) : 1.0;
      for (std::size_t i = 0; i < 2; ++i)
        consensus_closing_velocity_[i] += alpha *
            (velocity[i] - consensus_closing_velocity_[i]);
      consensus_closing_velocity_stamp_ = now;
    }
  }
  return consensus_closing_velocity_;
}

HorizontalMotion Follower::consensusCorrectionMotion(const Vec3& desired,
    double turn_rate, const ros::Time& now, const Vec3& nominal_velocity) {
  // Slew only feedback in the rotating fleet frame. Slewing the entire world
  // velocity separately for each aircraft delays their shared turn unequally.
  // Budget feedback before forming velocity and curvature. Raising only the
  // final speed lets a near-cancelled nominal/feedback vector generate an
  // excessive course-rate feed-forward from its small squared speed.
  auto boundedCorrection = [&](const Vec3& correction) {
    const double minimum = config_.fixedwing_min_speed;
    const double maximum = config_.fixedwing_max_speed;
    double along = std::max(minimum, nominal_velocity[0] + correction[0]);
    double cross = nominal_velocity[1] + correction[1];
    const double speed = std::hypot(along, cross);
    if (speed > maximum) {
      along *= maximum / speed;
      cross *= maximum / speed;
    }
    if (along < minimum) {
      along = minimum;
      const double cross_limit = std::sqrt(std::max(0.0,
          maximum * maximum - minimum * minimum));
      cross = clampValue(cross, -cross_limit, cross_limit);
    }
    return Vec3{along - nominal_velocity[0], cross - nominal_velocity[1], 0};
  };
  const Vec3 bounded_desired = boundedCorrection(desired);
  if (!consensus_correction_stamp_) consensus_correction_stamp_ = now;
  const double dt = clampValue((now - *consensus_correction_stamp_).toSec(), 0.0, 0.2);
  HorizontalMotion motion = horizontalMotion(consensus_correction_, {0,0,0});
  if (dt > 0.0) {
    const double heading_margin = std::max(0.01,
        config_.formation_course_slew_rate - std::abs(turn_rate));
    const double slew = std::min(config_.formation_speed_slew_rate,
        config_.fixedwing_min_speed * heading_margin);
    // Filter only feedback, not nominal rotation. This prevents stepped
    // telemetry positions from becoming noisy acceleration feed-forward.
    const double alpha = dt / (0.2 + dt);
    const Vec3 filtered{
        consensus_correction_[0] + alpha * (bounded_desired[0] - consensus_correction_[0]),
        consensus_correction_[1] + alpha * (bounded_desired[1] - consensus_correction_[1]), 0};
    motion = slewHorizontalCorrection(consensus_correction_, filtered, dt, slew);
    // The nominal turn can move the feasible feedback range since the last
    // cycle. Keep stored feedback feasible and derive acceleration from the
    // correction actually sent, so no hidden saturation state accumulates.
    const Vec3 bounded = boundedCorrection(motion.velocity);
    motion = horizontalMotion(bounded,
        {(bounded[0] - consensus_correction_[0]) / dt,
         (bounded[1] - consensus_correction_[1]) / dt, 0});
    consensus_correction_ = bounded;
    consensus_correction_stamp_ = now;
  }
  return motion;
}

HorizontalMotion Follower::columnReferenceMotion(const Vec3& velocity,
    double nominal_course_rate, const ros::Time& now) {
  // Column members follow different positions on the curved route. Derive
  // feed-forward from each member's final, already slew-limited velocity.
  Vec3 acceleration{-nominal_course_rate * velocity[1],
                     nominal_course_rate * velocity[0], 0};
  if (column_reference_velocity_ && column_reference_stamp_) {
    const double dt = (now - *column_reference_stamp_).toSec();
    if (dt >= 1e-3 && dt <= 0.2) {
      acceleration = {(velocity[0] - (*column_reference_velocity_)[0]) / dt,
                      (velocity[1] - (*column_reference_velocity_)[1]) / dt, 0};
    }
  }
  column_reference_velocity_ = velocity;
  column_reference_stamp_ = now;
  return horizontalMotion(velocity, acceleration);
}

mavros_msgs::PositionTarget Follower::motionTarget(const Vec3& position,
    const HorizontalMotion& motion, const ros::Time& now,
    const xd_uav_controller::ControlState& own, const std::string& frame) const {
  auto target = positionTarget(position,
      std::atan2(motion.velocity[1], motion.velocity[0]) + config_.yaw_offset,
      now, motion.velocity, frame.empty() ? config_.common_frame : frame, own);
  target.type_mask |= mavros_msgs::PositionTarget::IGNORE_PX |
                      mavros_msgs::PositionTarget::IGNORE_PY;
  target.type_mask &= ~(mavros_msgs::PositionTarget::IGNORE_AFX |
                        mavros_msgs::PositionTarget::IGNORE_AFY |
                        mavros_msgs::PositionTarget::IGNORE_YAW_RATE);
  target.acceleration_or_force.x = motion.acceleration[0];
  target.acceleration_or_force.y = motion.acceleration[1];
  target.yaw_rate = motion.course_rate;
  return target;
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
  // Reacquisition must use the same membership snapshot as the formation
  // slots and acquisition plan. Configured members can include aircraft that
  // have dropped out; requiring their stale states here makes every remaining
  // aircraft fail while trying to rebuild the smaller formation.
  for (const auto& name : activeMembers()) {
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

double Follower::acquisitionHeading(const ros::Time& now,
    const StateSnapshot& input, double fallback) const {
  if (!own_state_ || !usesFixedwingConsensusGuidance(*own_state_)) return fallback;
  double x = 0.0, y = 0.0;
  std::optional<double> first_heading;
  std::size_t count = 0;
  for (const auto& name : activeMembers()) {
    const xd_uav_controller::ControlState* state = nullptr;
    if (name == member_) state = &*own_state_;
    else {
      const auto found = input.states.find(name);
      if (found != input.states.end()) state = &found->second;
    }
    if (!state) continue;
    const auto velocity = std::get<1>(stateInCommonFrame(*state, now));
    const double speed = std::hypot(velocity.x, velocity.y);
    if (speed < config_.fixedwing_min_groundspeed_for_course || speed < 1e-3) continue;
    if (!first_heading) first_heading = std::atan2(velocity.y, velocity.x);
    x += velocity.x / speed;
    y += velocity.y / speed;
    ++count;
  }
  // Join along the team's current flight direction. Do not command an
  // unrelated route tangent while the aircraft are still separating/climbing.
  // Opposing courses have no meaningful circular mean; use a deterministic
  // member course in that case, shared by the acquisition plan.
  if (count > 0 && std::hypot(x, y) >= 0.2 * static_cast<double>(count))
    return std::atan2(y, x);
  return first_heading.value_or(fallback);
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
  std::map<std::string, Vec3> positions_by_member;
  std::map<std::string, Vec3> velocities_by_member;
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
    const auto converted = stateInCommonFrame(*state, now);
    const Vec3 position = std::get<0>(converted);
    positions.push_back(position);
    positions_by_member[name] = position;
    velocities_by_member[name] = vectorVec(std::get<1>(converted));
    auto offset = slot_offsets.find(name);
    if (offset == slot_offsets.end()) throw std::invalid_argument("missing consensus slot");
    offsets.push_back(offset->second);
  }
  const auto groups = activeFormationGroups();
  auto groupMeans = [](const std::vector<FormationGroup>& source,
                       const std::map<std::string, Vec3>& points) {
    std::vector<Vec3> means;
    for (const auto& group : source) {
      std::vector<Vec3> members;
      for (const auto& name : group.members) {
        const auto point = points.find(name);
        if (point != points.end()) members.push_back(point->second);
      }
      if (!members.empty()) means.push_back(sharedCenter(members).first);
    }
    return means;
  };
  const Vec3 centroid = sharedCenter(groupMeans(groups, positions_by_member)).first;
  const Vec3 mean_velocity = sharedCenter(groupMeans(groups, velocities_by_member)).first;
  const Vec3 mean_offset = sharedCenter(groupMeans(groups, slot_offsets)).first;
  const Vec3 own_velocity = velocities_by_member.at(member_);
  double maximum_slot_radius = 0.0;
  for (const auto& offset : offsets) {
    maximum_slot_radius = std::max(
        maximum_slot_radius, std::hypot(offset[0], offset[1]));
  }
  geometry.capturing = acquire_heading_ && acquire_stage_ &&
      (*acquire_stage_ == "waiting_for_team" || *acquire_stage_ == "separating" ||
       *acquire_stage_ == "gathering" || *acquire_stage_ == "merging");
  if (geometry.capturing) {
    geometry.heading = *acquire_heading_;
    geometry.route_position = centroid;
    geometry.route_vz = 0.0;
    geometry.reference_velocity = {config_.cruise_speed * std::cos(geometry.heading),
                                    config_.cruise_speed * std::sin(geometry.heading), 0};
  } else if (!enabled_ || !acquire_stage_ || *acquire_stage_ == "waiting_for_team") {
    // Idle center heartbeats must not advance the flight reference before
    // the common acquisition heading has been adopted by the team.
    const auto reference = pathReferenceState(centroid,
        formation_selection_.waypoints, config_.cruise_speed,
        config_.lookahead, formation_selection_.loop_path);
    geometry.heading = std::atan2(reference.velocity[1], reference.velocity[0]);
    geometry.route_position = reference.position;
    geometry.route_vz = reference.velocity[2];
    geometry.reference_velocity = reference.velocity;
  } else {
    const auto group_revision = formationGroupPlan().revision;
    std::lock_guard<std::mutex> lock(shared_->formation_motion_mutex);
    auto& motion = shared_->formation_motion;
    if (!motion.initialized || motion.selection_revision < formation_selection_.revision ||
        motion.group_revision < group_revision) {
      const auto initial = pathReferenceState(centroid,
          formation_selection_.waypoints, config_.cruise_speed,
          config_.lookahead, formation_selection_.loop_path);
      motion = SharedFormationMotion{};
      motion.initialized = true;
      motion.selection_revision = formation_selection_.revision;
      motion.group_revision = group_revision;
      motion.turn.heading = acquire_heading_.value_or(
          std::atan2(initial.velocity[1], initial.velocity[0]));
      motion.forward_speed = config_.cruise_speed;
      motion.stamp = now;
      motion.route_position = initial.position;
      motion.velocity = {config_.cruise_speed * std::cos(motion.turn.heading),
                         config_.cruise_speed * std::sin(motion.turn.heading),
                         initial.velocity[2]};
    }
    // publishCenter() and the control target query this in the same cycle.
    // A repeated timestamp must reuse omega/alpha, not replace them with zero.
    const double elapsed = (now - motion.stamp).toSec();
    if (elapsed > 0.0 && motion.selection_revision == formation_selection_.revision &&
        motion.group_revision == group_revision) {
      const double dt = std::min(elapsed, 0.2);
      std::map<std::string, Vec3> estimates;
      for (std::size_t i = 0; i < positions.size(); ++i)
        estimates[geometry.members[i]] =
            virtualCenter(positions[i], motion.turn.heading, offsets[i]);
      const Vec3 estimate = sharedCenter(groupMeans(groups, estimates)).first;
      const auto reference = pathReferenceState(estimate,
          formation_selection_.waypoints, config_.cruise_speed,
          config_.lookahead, formation_selection_.loop_path);
      // The furthest slot's tangential acceleration is bounded by the same
      // acceleration budget used for speed corrections. No new tuning knob.
      const double maximum_angular_acceleration = config_.formation_speed_slew_rate /
          std::max(1.0, maximum_slot_radius);
      const auto proposed_turn = advanceTurnReference(motion.turn,
          std::atan2(reference.velocity[1], reference.velocity[0]),
          reference.turn_rate * motion.forward_speed / config_.cruise_speed,
          dt, config_.formation_heading_rate_limit, maximum_angular_acceleration);
      const double old_nx = -std::sin(motion.turn.heading);
      const double old_ny = std::cos(motion.turn.heading);
      const double route_ex = reference.position[0] - estimate[0];
      const double route_ey = reference.position[1] - estimate[1];
      if (config_.route_velocity_damping > 0.0) {
        // Differentiate the same moving lookahead-point error used by P.
        // Remove nominal rotation of the mean slot offset first; otherwise
        // unequal groups can make valid turn motion look like route error.
        const double old_tx = old_ny, old_ty = -old_nx;
        const double mean_rx = old_tx * mean_offset[0] - old_ty * mean_offset[1];
        const double mean_ry = old_ty * mean_offset[0] + old_tx * mean_offset[1];
        const double center_vx = mean_velocity[0] + motion.turn.rate * mean_ry;
        const double center_vy = mean_velocity[1] - motion.turn.rate * mean_rx;
        const auto projection = pathReferenceState(estimate,
            formation_selection_.waypoints, config_.cruise_speed, 0.0,
            formation_selection_.loop_path);
        const double progress_speed = (center_vx * projection.velocity[0] +
            center_vy * projection.velocity[1]) / config_.cruise_speed;
        const double target_vx = reference.velocity[0] * progress_speed / config_.cruise_speed;
        const double target_vy = reference.velocity[1] * progress_speed / config_.cruise_speed;
        const double closing = (center_vx - target_vx) * old_nx +
            (center_vy - target_vy) * old_ny +
            motion.turn.rate * (route_ex * old_tx + route_ey * old_ty);
        const double time_constant = config_.catchup_derivative_filter_time_constant;
        const double alpha = time_constant > 0.0 ? dt / (time_constant + dt) : 1.0;
        motion.route_closing_velocity += alpha *
            (closing - motion.route_closing_velocity);
      }
      const double requested = clampValue(config_.route_correction_gain *
          (route_ex * old_nx + route_ey * old_ny) -
          config_.route_velocity_damping * motion.route_closing_velocity,
          -config_.max_route_correction_velocity, config_.max_route_correction_velocity);
      const double lateral_goal = motion.route_correction +
          (requested - motion.route_correction) * dt / (0.2 + dt);
      // Allocate rotation first. Keep the current common speed available so
      // inner aircraft can accelerate while outer aircraft decelerate at
      // turn exit. Dropping common speed in the same interpolation used to
      // double the outer slot's required deceleration and halve angular
      // braking, invalidating advanceTurnReference's stopping calculation.
      const auto plan = planFormationTurn(offsets, motion.forward_speed,
          config_.fixedwing_min_speed, config_.fixedwing_max_speed,
          proposed_turn.rate, motion.route_correction);
      const double speed_change = plan.speed - motion.forward_speed;
      const double rate_change = plan.turn_rate - motion.turn.rate;

      double largest_change = std::abs(speed_change);
      for (const auto& offset : offsets)
        largest_change = std::max(largest_change,
            std::hypot(speed_change - rate_change * offset[1],
                       rate_change * offset[0]));
      double fraction = largest_change > 0.0
          ? std::min(1.0, config_.formation_speed_slew_rate * dt / largest_change)
          : 1.0;
      if (std::abs(rate_change) > 0.0)
        fraction = std::min(fraction,
            maximum_angular_acceleration * dt / std::abs(rate_change));
      const double previous_rate = motion.turn.rate;
      double actual_speed_change = fraction * speed_change;
      const double actual_rate_change = fraction * rate_change;
      double actual_lateral_change = 0.0;

      // Spend only the remaining per-slot acceleration and speed headroom
      // on lateral route recovery, then on returning to cruise. A late route
      // correction must never stretch the angular stopping distance.
      auto remainingStep = [&](bool lateral, double requested_step) {
        double lower = -std::numeric_limits<double>::infinity();
        double upper = std::numeric_limits<double>::infinity();
        auto constrain = [&](const Vec3& offset) {
          const double dx = actual_speed_change - actual_rate_change * offset[1];
          const double dy = actual_lateral_change + actual_rate_change * offset[0];
          const double vx = motion.forward_speed + actual_speed_change -
              (previous_rate + actual_rate_change) * offset[1];
          const double vy = motion.route_correction + actual_lateral_change +
              (previous_rate + actual_rate_change) * offset[0];
          const double budget = config_.formation_speed_slew_rate * dt;
          const double other_delta = lateral ? dx : dy;
          const double radius = std::sqrt(std::max(0.0,
              budget * budget - other_delta * other_delta));
          const double delta = lateral ? dy : dx;
          lower = std::max(lower, -radius - delta);
          upper = std::min(upper, radius - delta);
          const double other_speed = lateral ? vx : vy;
          const double speed_radius = std::sqrt(std::max(0.0,
              config_.fixedwing_max_speed * config_.fixedwing_max_speed -
              other_speed * other_speed));
          const double speed = lateral ? vy : vx;
          lower = std::max(lower, (lateral ? -speed_radius : config_.fixedwing_min_speed) - speed);
          upper = std::min(upper, speed_radius - speed);
        };
        constrain({0, 0, 0});
        for (const auto& offset : offsets) constrain(offset);
        // Zero is feasible by construction; include it despite roundoff at
        // a fully used acceleration or speed boundary.
        return clampValue(requested_step, std::min(0.0, lower), std::max(0.0, upper));
      };
      actual_lateral_change = remainingStep(true,
          lateral_goal - motion.route_correction);
      actual_speed_change += remainingStep(false,
          config_.cruise_speed - (motion.forward_speed + actual_speed_change));
      motion.forward_speed += actual_speed_change;
      motion.route_correction += actual_lateral_change;
      motion.turn.rate += actual_rate_change;
      motion.turn.acceleration = (motion.turn.rate - previous_rate) / dt;
      motion.turn.heading += 0.5 * (previous_rate + motion.turn.rate) * dt;
      const double tx = std::cos(motion.turn.heading), ty = std::sin(motion.turn.heading);
      const double nx = -ty, ny = tx;
      const double omega = motion.turn.rate;
      motion.velocity = {tx * motion.forward_speed + nx * motion.route_correction,
                          ty * motion.forward_speed + ny * motion.route_correction,
                          reference.velocity[2]};
      const double along_acceleration = actual_speed_change / dt -
          omega * motion.route_correction;
      const double cross_acceleration = actual_lateral_change / dt +
          omega * motion.forward_speed;
      motion.acceleration = {
          tx * along_acceleration + nx * cross_acceleration,
          ty * along_acceleration + ny * cross_acceleration, 0};
      motion.route_position = reference.position;
      motion.stamp = now;
    }
    geometry.heading = motion.turn.heading;
    geometry.turn_rate = motion.turn.rate;
    geometry.turn_acceleration = motion.turn.acceleration;
    geometry.route_position = motion.route_position;
    geometry.route_vz = motion.velocity[2];
    geometry.reference_velocity = motion.velocity;
    geometry.reference_acceleration = motion.acceleration;
  }
  consensus_heading_ = geometry.heading;
  consensus_heading_stamp_ = now;
  // d(R^-1 * (p_i - centroid))/dt expressed back in world axes.
  // Use actual position deviations so rotation of a displaced slot does not
  // look like extra closing velocity. Fixed local slot offsets have zero
  // derivative in this rotating frame; group weights match the position law.
  const Vec3& own_position = positions_by_member.at(member_);
  geometry.relative_velocity = {
      own_velocity[0] - mean_velocity[0] +
          geometry.turn_rate * (own_position[1] - centroid[1]),
      own_velocity[1] - mean_velocity[1] -
          geometry.turn_rate * (own_position[0] - centroid[0]),
      own_velocity[2] - mean_velocity[2]};
  std::map<std::string, Vec3> centers_by_member;
  for (std::size_t i = 0; i < positions.size(); ++i) {
    const Vec3 center = virtualCenter(positions[i], geometry.heading, offsets[i]);
    geometry.centers.push_back(center);
    centers_by_member[geometry.members[i]] = center;
  }
  const auto mean = sharedCenter(groupMeans(groups, centers_by_member));
  geometry.mean = mean.first;
  geometry.largest_error = 0.0;
  for (const auto& center : geometry.centers) {
    const double dx = center[0] - geometry.mean[0];
    const double dy = center[1] - geometry.mean[1];
    // Acquisition deliberately assigns different altitude tiers. Use only
    // horizontal slot-center error for the gathering/merging gate; altitude
    // convergence is checked independently before entering following.
    geometry.largest_error = std::max(geometry.largest_error,
        std::hypot(dx, dy));
  }
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
    Vec3 estimate = position;
    Vec3 center = position;
    for (int i = 0; i < 2; ++i) {
      const auto path = pathReferenceState(estimate,
          formation_selection_.waypoints, config_.cruise_speed,
          config_.lookahead, formation_selection_.loop_path);
      const double heading = std::atan2(path.velocity[1], path.velocity[0]);
      center = virtualCenter(position, heading, consensus_offset);
      try {
        const auto group_centers = teamCenterPoints(center, input);
        if (!group_centers.empty()) estimate = sharedCenter(group_centers).first;
      } catch (const std::exception&) {
        break;
      }
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

bool Follower::publishCenter(const ros::Time& now, const StateSnapshot& input) {
  if (!own_state_ || !fresh(own_state_->header, has_own_state_received_,
      own_state_received_, now) || !own_state_->state_valid ||
      own_state_->header.frame_id.empty()) return false;
  Vec3 center;
  try { center = ownCenterInCommonFrame(now, input, *own_state_); }
  catch (const std::exception&) { return false; }
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
  return true;
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
  for (const auto& group : activeFormationGroups()) {
    std::vector<Vec3> group_points;
    for (const auto& name : group.members) {
      if (name == member_) {
        group_points.push_back(own_center);
      } else {
        auto it = input.centers.find(name);
        auto received = input.center_received.find(name);
        if (it == input.centers.end() || received == input.center_received.end() ||
            cleanFrame(it->second.header.frame_id) != config_.common_frame)
          throw std::invalid_argument("neighbor_center_stale:" + name);
        group_points.push_back(pointVec(it->second.point));
      }
    }
    if (!group_points.empty()) result.push_back(sharedCenter(group_points).first);
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
  return pathReferenceState(center, formation_selection_.waypoints, speed,
                            lookahead, formation_selection_.loop_path);
}

double Follower::teamPacedSpeed(
    const ros::Time& now, const std::vector<Vec3>& center_points,
    const Vec3& mean, const Vec3& path_velocity, double heading,
    double turn_rate, const StateSnapshot& input,
    const xd_uav_controller::ControlState& own) {
  const double path_speed = std::hypot(path_velocity[0], path_velocity[1]);
  const auto groups = activeFormationGroups();
  if (path_speed < 1e-6) return path_speed;
  const double tx = path_velocity[0] / path_speed, ty = path_velocity[1] / path_speed;
  const double nx = -ty, ny = tx;
  if (center_points.size() != groups.size())
    throw std::invalid_argument("group_center_count_mismatch");
  std::vector<double> progress;
  progress.reserve(groups.size());
  for (const auto& point : center_points) {
    const double ex = mean[0] - point[0], ey = mean[1] - point[1];
    const double along = ex * tx + ey * ty;
    const double cross = std::abs(ex * nx + ey * ny);
    progress.push_back(std::abs(along) <= 1.0 ? cross : std::copysign(std::hypot(along, cross), along));
  }
  double dt = paced_speed_stamp_ ? clampValue((now - *paced_speed_stamp_).toSec(), 0.0, 0.2) : 0.0;
  const double alpha = dt / (0.8 + dt);
  for (std::size_t i = 0; i < progress.size(); ++i) {
    auto it = progress_error_filter_.find(groups[i].id);
    if (it == progress_error_filter_.end()) progress_error_filter_[groups[i].id] = progress[i];
    else it->second += alpha * (progress[i] - it->second);
    progress[i] = progress_error_filter_[groups[i].id];
  }
  std::vector<double> member_speeds;
  member_speeds.reserve(groups.size());
  const auto slot_offsets = consensusSlotOffsets();
  for (const auto& group : groups) {
    double speed_sum = 0.0;
    std::size_t speed_count = 0;
    for (const auto& name : group.members) {
      const auto state_it = input.states.find(name);
      const auto& state = name == member_ ? own :
          (state_it != input.states.end() ? state_it->second : own);
      if (name != member_ && state_it == input.states.end())
        throw std::invalid_argument(name + "_state_stale");
      const auto converted = stateInCommonFrame(state, now);
      const auto velocity = std::get<1>(converted);
      const Vec3 slot_turn = slotVelocity({0,0,0}, {0,0,0}, heading,
          slot_offsets.at(name), 0.0, false, turn_rate, true);
      speed_sum += (velocity.x - slot_turn[0]) * tx +
                   (velocity.y - slot_turn[1]) * ty;
      ++speed_count;
    }
    member_speeds.push_back(speed_count ? speed_sum / speed_count : path_speed);
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
  const auto group_it = std::find_if(groups.begin(), groups.end(),
      [&](const FormationGroup& group) {
        return std::find(group.members.begin(), group.members.end(), member_) !=
            group.members.end();
      });
  if (group_it == groups.end()) throw std::invalid_argument("own_group_unavailable");
  const double target = targets.at(static_cast<std::size_t>(group_it - groups.begin()));
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
  if (stage == "following") acquire_force_capture_ = false;
  reason_ = stage == "following" ? "following" : "acquiring_" + stage;
  publishStatus();
}

AcquisitionPlan Follower::buildAcquisitionPlan(
    const ros::Time& now, const StateSnapshot& input) const {
  ensureSlotAssignment(now, input);
  AcquisitionPlan plan;
  plan.members = activeMembers();
  if (plan.members.empty())
    throw std::invalid_argument("no active members for acquisition");
  plan.shape = formation_selection_.shape;
  plan.spacing = config_.spacing;
  plan.selection_revision = formation_selection_.revision;
  plan.group_revision = formationGroupPlan().revision;
  plan.stamp = now;
  plan.initial_positions = memberPositionsInCommonFrame(now, input);
  if (plan.initial_positions.size() != plan.members.size())
    throw std::invalid_argument("active acquisition member positions are incomplete");
  std::map<std::string, Vec3> positions_by_member;
  for (std::size_t i = 0; i < plan.members.size(); ++i)
    positions_by_member[plan.members[i]] = plan.initial_positions[i];
  std::vector<Vec3> group_centers;
  for (const auto& group : activeFormationGroups()) {
    std::vector<Vec3> points;
    for (const auto& name : group.members) points.push_back(positions_by_member.at(name));
    group_centers.push_back(sharedCenter(points).first);
  }
  Vec3 mean = sharedCenter(group_centers).first;
  const auto route = pathReferenceState(mean, formation_selection_.waypoints,
      config_.cruise_speed, config_.lookahead,
      formation_selection_.loop_path);
  plan.heading = acquisitionHeading(now, input,
      std::atan2(route.velocity[1], route.velocity[0]));
  plan.altitude = route.position[2];
  const auto offset_map = consensusSlotOffsets();
  std::vector<Vec3> offsets;
  offsets.reserve(plan.members.size());
  for (const auto& name : plan.members) offsets.push_back(offset_map.at(name));
  plan.slots = acquisitionSlots(plan.initial_positions,
                                offsets,
                                plan.heading, plan.altitude);
  double top = route.position[2];
  for (const auto& point : plan.initial_positions) top = std::max(top, point[2]);
  const double base = std::ceil(top);
  for (std::size_t i = 0; i < plan.members.size(); ++i)
    plan.tiers.push_back(base + i * config_.acquire_vertical_spacing);
  {
    std::lock_guard<std::mutex> lock(shared_->formation_motion_mutex);
    shared_->formation_motion = SharedFormationMotion{};
  }
  return plan;
}

bool Follower::planAcquisition(const ros::Time& now,
                               const StateSnapshot& input) {
  const auto members = activeMembers();
  if (members.empty() ||
      std::find(members.begin(), members.end(), member_) == members.end())
    return false;
  const std::size_t count = members.size();
  std::optional<AcquisitionPlan> plan;
  {
    std::lock_guard<std::mutex> lock(shared_->acquisition_plan_mutex);
    plan = shared_->acquisition_plan;
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
        parsed.selection_revision = static_cast<std::uint64_t>(
            static_cast<int>(parameter["selection_revision"]));
        parsed.group_revision = static_cast<std::uint64_t>(
            static_cast<int>(parameter["group_revision"]));
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
  auto matchesCurrentTeam = [&](const std::optional<AcquisitionPlan>& candidate) {
    if (!candidate || candidate->members != members ||
        candidate->shape != formation_selection_.shape ||
        candidate->selection_revision != formation_selection_.revision ||
        candidate->group_revision != formationGroupPlan().revision ||
        candidate->spacing != config_.spacing ||
        candidate->initial_positions.size() != count ||
        candidate->slots.size() != count || candidate->tiers.size() != count)
      return false;
    const double age = (now - candidate->stamp).toSec();
    return age >= 0.0 && age <= config_.acquire_timeout;
  };
  if (!matchesCurrentTeam(plan)) plan.reset();
  const bool mayCreatePlan = shared_inputs_ || member_ == members.front();
  if (!plan && mayCreatePlan) {
    plan = buildAcquisitionPlan(now, input);
    {
      std::lock_guard<std::mutex> lock(shared_->acquisition_plan_mutex);
      shared_->acquisition_plan = plan;
    }
    nh_.setParam("/formation/" + config_.team_id + "/acquisition_plan",
                 acquisitionPlanValue(*plan));
  }
  if (!matchesCurrentTeam(plan)) return false;
  for (const auto& p : plan->initial_positions)
    for (double value : p) if (!std::isfinite(value)) throw std::invalid_argument("shared acquisition plan is invalid");
  for (const auto& p : plan->slots)
    for (double value : p) if (!std::isfinite(value)) throw std::invalid_argument("shared acquisition plan is invalid");
  for (double value : plan->tiers) if (!std::isfinite(value)) throw std::invalid_argument("shared acquisition plan is invalid");
  const auto member_it = std::find(plan->members.begin(), plan->members.end(), member_);
  if (member_it == plan->members.end()) return false;
  const auto index = static_cast<std::size_t>(member_it - plan->members.begin());
  acquire_members_ = plan->members;
  acquire_slots_ = plan->slots;
  acquire_tiers_ = plan->tiers;
  acquire_initial_xy_.clear();
  acquire_initial_xy_.push_back(
      {plan->initial_positions[index][0], plan->initial_positions[index][1]});
  acquire_heading_ = plan->heading;
  acquire_altitude_ = plan->altitude;
  if (!std::isfinite(*acquire_heading_) || !std::isfinite(*acquire_altitude_))
    throw std::invalid_argument("shared acquisition heading or altitude is invalid");
  acquire_started_ = now;
  const auto positions = memberPositionsInCommonFrame(now, input);
  if (positions.size() != count)
    throw std::invalid_argument("active acquisition positions changed");
  bool alreadyInSlots = true;
  for (std::size_t i = 0; i < count; ++i) {
    const double horizontal_error = std::hypot(
        positions[i][0] - plan->slots[i][0],
        positions[i][1] - plan->slots[i][1]);
    const double altitude_error = std::abs(positions[i][2] - plan->slots[i][2]);
    alreadyInSlots = alreadyInSlots &&
        horizontal_error <= config_.acquire_tolerance &&
        altitude_error <= config_.acquire_height_tolerance;
  }
  const bool capture = acquire_force_capture_ || !alreadyInSlots;
  setAcquireStage(capture ? "separating" : "following");
  if (!capture) acquire_force_capture_ = false;
  return true;
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
    auto waitTarget = [&]() {
      if (own.vehicle_type == xd_uav_controller::ControlState::VEHICLE_FIXEDWING)
        return fixedwingWaitTarget(now, input, own);
      const auto p = pointVec(own.position_odom);
      geometry_msgs::PoseStamped pose; pose.pose.orientation = own.orientation_odom_body;
      return positionTarget(p, yaw(pose), now, Vec3{0,0,0}, "", own);
    };
    if (!teamEnabled(now, input) || !planAcquisition(now, input) ||
        (acquire_stage_ && *acquire_stage_ == "waiting_for_team")) {
      return waitTarget();
    }
  }
  if (acquire_stage_ && *acquire_stage_ == "following")
    return consensusTarget(now, input, own);
  const auto positions = memberPositionsInCommonFrame(now, input);
  const auto member_it = std::find(acquire_members_.begin(), acquire_members_.end(), member_);
  if (member_it == acquire_members_.end() ||
      positions.size() != acquire_members_.size() ||
      acquire_tiers_.size() != acquire_members_.size() ||
      acquire_slots_.size() != acquire_members_.size())
    throw std::invalid_argument("acquisition member set changed unexpectedly");
  const std::size_t index = static_cast<std::size_t>(member_it - acquire_members_.begin());
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
  const bool acquisition_stage = acquire_stage_ &&
      (*acquire_stage_ == "waiting_for_team" ||
       *acquire_stage_ == "separating" || *acquire_stage_ == "gathering" ||
       *acquire_stage_ == "merging");
  if (!acquisition_stage &&
      (formation_selection_.shape == "column" ||
       formation_selection_.shape == "single_file")) {
    const auto groups = activeFormationGroups();
    const auto members = activeMembers();
    const auto slot_offsets = consensusSlotOffsets();
    std::map<std::string, double> progresses;
    std::map<std::string, double> phases;
    Vec3 own_position{};
    double path_length = 0.0;
    for (const auto& name : members) {
      const xd_uav_controller::ControlState* state =
          name == member_ ? &own : nullptr;
      if (!state) {
        const auto it = input.states.find(name);
        if (it == input.states.end()) {
          throw std::invalid_argument(name + "_state_stale");
        }
        state = &it->second;
      }
      const Vec3 position = std::get<0>(stateInCommonFrame(*state, now));
      if (name == member_) own_position = position;
      const auto projection = projectPathProgress(
          position, formation_selection_.waypoints,
          formation_selection_.loop_path);
      if (path_length == 0.0) {
        path_length = projection.second;
      } else if (std::abs(projection.second - path_length) > 1e-6) {
        throw std::invalid_argument("inconsistent_column_path_length");
      }
      progresses[name] = projection.first;
      phases[name] = projection.first - slot_offsets.at(name)[0];
    }
    if (groups.empty() || path_length <= 1e-6) {
      throw std::invalid_argument("column_path_projection_unavailable");
    }

    std::vector<double> group_phases;
    group_phases.reserve(groups.size());
    for (const auto& group : groups) {
      const double anchor = phases.at(group.members.front());
      double sum = 0.0;
      for (const auto& name : group.members) {
        double phase = phases.at(name);
        if (formation_selection_.loop_path) {
          double delta = std::fmod(phase - anchor + 0.5 * path_length,
                                   path_length);
          if (delta < 0.0) delta += path_length;
          phase = anchor + delta - 0.5 * path_length;
        }
        sum += phase;
      }
      group_phases.push_back(sum / static_cast<double>(group.members.size()));
    }
    double mean_phase = 0.0;
    std::vector<double> aligned_group_phases = group_phases;
    if (formation_selection_.loop_path) {
      double anchor = std::fmod(group_phases.front(), path_length);
      if (anchor < 0.0) anchor += path_length;
      double sum = anchor;
      aligned_group_phases.front() = anchor;
      for (std::size_t i = 1; i < group_phases.size(); ++i) {
        double phase = std::fmod(group_phases[i], path_length);
        if (phase < 0.0) phase += path_length;
        double delta = std::fmod(
            phase - anchor + 0.5 * path_length, path_length);
        if (delta < 0.0) delta += path_length;
        delta -= 0.5 * path_length;
        aligned_group_phases[i] = anchor + delta;
        sum += aligned_group_phases[i];
      }
      mean_phase = sum / static_cast<double>(group_phases.size());
    } else {
      for (const double phase : group_phases) mean_phase += phase;
      mean_phase /= static_cast<double>(group_phases.size());
    }

    const Vec3& own_offset = slot_offsets.at(member_);
    const auto own_group_it = std::find_if(groups.begin(), groups.end(),
        [&](const FormationGroup& group) {
          return std::find(group.members.begin(), group.members.end(), member_) !=
              group.members.end();
        });
    if (own_group_it == groups.end())
      throw std::invalid_argument("own_group_unavailable");
    const std::size_t own_group_index =
        static_cast<std::size_t>(own_group_it - groups.begin());
    const double own_phase = aligned_group_phases.at(own_group_index);
    const double phase_error = mean_phase - own_phase;
    const auto correction = activeConsensusCorrection();
    const double along_error = std::copysign(
        std::max(0.0, std::abs(phase_error) - config_.catchup_deadband),
        phase_error);
    const double speed_adjustment = clampValue(
        correction.first * along_error,
        -config_.max_consensus_velocity, config_.max_consensus_velocity);
    const double target_speed = std::max(
        config_.fixedwing_min_speed,
        config_.cruise_speed + speed_adjustment);
    const double progress_offset =
        mean_phase + own_offset[0] - progresses.at(member_);
    const PathResult route_reference = pathReferenceState(
        own_position, formation_selection_.waypoints, config_.cruise_speed,
        config_.lookahead, formation_selection_.loop_path, progress_offset);
    const PathResult projection_reference = pathReferenceState(
        own_position, formation_selection_.waypoints, config_.cruise_speed,
        0.0, formation_selection_.loop_path);

    const double projection_speed = std::hypot(
        projection_reference.velocity[0], projection_reference.velocity[1]);
    const double normal_x = projection_speed > 1e-6
                                ? -projection_reference.velocity[1] /
                                      projection_speed
                                : 0.0;
    const double normal_y = projection_speed > 1e-6
                                ? projection_reference.velocity[0] /
                                      projection_speed
                                : 0.0;
    const double cross_error =
        (projection_reference.position[0] - own_position[0]) * normal_x +
        (projection_reference.position[1] - own_position[1]) * normal_y;
    const double lateral_adjustment = clampValue(
        correction.first * cross_error,
        -correction.second, correction.second);
    const double speed_scale = target_speed /
                               std::max(config_.cruise_speed, 1e-3);
    Vec3 velocity{
        route_reference.velocity[0] * speed_scale +
            normal_x * lateral_adjustment,
        route_reference.velocity[1] * speed_scale +
            normal_y * lateral_adjustment,
        route_reference.velocity[2] * speed_scale};
    velocity = smoothCourseVelocity(velocity, now, own);
    const double route_heading = std::atan2(
        route_reference.velocity[1], route_reference.velocity[0]);
    const Slot slot = slotTarget(route_reference.position, route_heading,
                                 {0.0, 0.0, 0.0}, config_.yaw_offset);
    const double target_altitude = altitude
        ? *altitude
        : route_reference.position[2] + own_offset[2];
    const auto motion = limitHorizontalMotion(columnReferenceMotion(velocity,
        route_reference.turn_rate * speed_scale, now), config_.fixedwing_min_speed,
        config_.fixedwing_max_speed, route_heading);
    return motionTarget({slot[0], slot[1], target_altitude}, motion, now, own);
  }

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
  const bool separating = acquire_stage_ && *acquire_stage_ == "separating";
  const double relative_along = geometry.relative_velocity[0] * tx +
                                geometry.relative_velocity[1] * ty;
  const double relative_cross = geometry.relative_velocity[0] * nx +
                                geometry.relative_velocity[1] * ny;
  // Linear damping only: the former signed speed-squared prediction amplified
  // this feedback at large closing speeds. Filter measured derivatives before
  // damping both axes, including cross-track overshoot after a turn.
  const Vec3 closing = capture_stage ? Vec3{relative_along, relative_cross, 0} :
      filteredConsensusClosingVelocity({relative_along, relative_cross, 0}, now);
  const double speed_adjustment = separating ? 0.0 : clampValue(
      correction.first * along - closing[0],
      -speed_limit, speed_limit);
  const double lateral_adjustment = separating ? 0.0 : clampValue(
      correction.first * cross - closing[1],
      -correction.second, correction.second);
  const Vec3 nominal_velocity{
      geometry.reference_velocity[0] * tx + geometry.reference_velocity[1] * ty -
          geometry.turn_rate * consensus_offset[1],
      geometry.reference_velocity[0] * nx + geometry.reference_velocity[1] * ny +
          geometry.turn_rate * consensus_offset[0], 0};
  const auto feedback = consensusCorrectionMotion(
      {speed_adjustment, lateral_adjustment, 0}, geometry.turn_rate, now,
      nominal_velocity);
  const double along_acceleration = feedback.acceleration[0] -
      geometry.turn_rate * feedback.velocity[1];
  const double cross_acceleration = feedback.acceleration[1] +
      geometry.turn_rate * feedback.velocity[0];
  const Vec3 center_velocity{
      geometry.reference_velocity[0] + tx * feedback.velocity[0] + nx * feedback.velocity[1],
      geometry.reference_velocity[1] + ty * feedback.velocity[0] + ny * feedback.velocity[1],
      geometry.route_vz};
  const Vec3 center_acceleration{
      geometry.reference_acceleration[0] + tx * along_acceleration + nx * cross_acceleration,
      geometry.reference_acceleration[1] + ty * along_acceleration + ny * cross_acceleration, 0};
  const auto motion = limitHorizontalMotion(turningSlotMotion(center_velocity,
      center_acceleration, heading, consensus_offset, geometry.turn_rate,
      geometry.turn_acceleration), config_.fixedwing_min_speed,
      config_.fixedwing_max_speed, heading);
  const auto slot = slotTarget(mean, heading, consensus_offset, config_.yaw_offset);
  const double target_altitude = altitude ? *altitude :
      (geometry.capturing ? *acquire_altitude_ + consensus_offset[2] :
       geometry.route_position[2] + consensus_offset[2]);
  return motionTarget({slot[0], slot[1], target_altitude}, motion, now, own);
}

mavros_msgs::PositionTarget Follower::consensusTarget(
    const ros::Time& now, const StateSnapshot& input,
    const xd_uav_controller::ControlState& own) {
  if (usesFixedwingConsensusGuidance(own))
    return fixedwingConsensusTarget(now, input, own);
  const auto centers = consensusCenters(now, input, own);
  const Vec3 center = std::get<0>(centers), mean = std::get<1>(centers);
  const auto center_points = teamCenterPoints(center, input);
  const auto nominal = pathReferenceState(mean, formation_selection_.waypoints,
      config_.cruise_speed, config_.lookahead,
      formation_selection_.loop_path);
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
  if (config_.mode == "consensus") {
    const auto group = formationGroupFor(member_);
    if (group && group->detached)
      return detachedGroupTarget(now, input, own, *group);
    return consensusTarget(now, input, own);
  }
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

mavros_msgs::PositionTarget Follower::detachedGroupTarget(
    const ros::Time& now, const StateSnapshot& input,
    const xd_uav_controller::ControlState& own,
    const FormationGroup& group) {
  if (group.anchor.empty())
    throw std::invalid_argument("detached_group_anchor_unavailable");
  if (group.anchor == member_ && !group.control_point) {
    const auto own_common = stateInCommonFrame(own, now);
    const auto route = pathReferenceState(std::get<0>(own_common),
        formation_selection_.waypoints, config_.cruise_speed,
        config_.lookahead, formation_selection_.loop_path);
    const Vec3 velocity = smoothCourseVelocity(route.velocity, now, own);
    const double speed = std::hypot(velocity[0], velocity[1]);
    const double heading = speed > 1e-3
        ? std::atan2(velocity[1], velocity[0])
        : yaw(std::get<2>(own_common));
    return positionTarget(route.position, heading, now, velocity,
                          config_.common_frame, own);
  }
  const auto members = activeMembers();
  const auto anchor_it = std::find(members.begin(), members.end(), group.anchor);
  const auto own_it = std::find(members.begin(), members.end(), member_);
  if (anchor_it == members.end() || own_it == members.end())
    throw std::invalid_argument("detached_group_member_unavailable");
  const auto anchor_state = input.states.find(group.anchor);
  const auto anchor_received = input.state_received.find(group.anchor);
  if (anchor_state == input.states.end() || anchor_received == input.state_received.end() ||
      !fresh(anchor_state->second.header, true, anchor_received->second, now))
    throw std::invalid_argument("detached_group_anchor_state_stale");

  const auto slots = consensusSlotOffsets();
  const auto anchor_slot = slots.find(group.anchor);
  const auto own_slot = slots.find(member_);
  if (anchor_slot == slots.end() || own_slot == slots.end())
    throw std::invalid_argument("detached_group_slot_unavailable");
  Vec3 relative_offset{};
  for (int axis = 0; axis < 3; ++axis)
    relative_offset[axis] = own_slot->second[axis] - anchor_slot->second[axis];
  // The detached subgroup has no vertical slot separation. Fixed-wing
  // followers track the anchor's measured altitude and vertical motion;
  // the external control point remains the anchor's eventual destination.
  relative_offset[2] = 0.0;

  const bool fixedwing = own.vehicle_type ==
      xd_uav_controller::ControlState::VEHICLE_FIXEDWING;
  if (group.control_point && (!fixedwing || group.anchor == member_)) {
    const auto anchor_common = stateInCommonFrame(anchor_state->second, now);
    const Vec3 anchor_velocity = vectorVec(std::get<1>(anchor_common));
    const double anchor_speed = std::hypot(anchor_velocity[0], anchor_velocity[1]);
    const double anchor_heading = fixedwing && anchor_speed >=
            config_.fixedwing_min_groundspeed_for_course
        ? std::atan2(anchor_velocity[1], anchor_velocity[0])
        : yaw(std::get<2>(anchor_common));
    const double heading = group.control_point->heading_valid
        ? group.control_point->heading : anchor_heading;
    const auto slot = slotTarget(group.control_point->position, heading,
        relative_offset, config_.yaw_offset);
    // Publish the same commanded altitude to every member; only X/Y vary by
    // slot. This lets the subgroup descend with the external control point.
    return positionTarget({slot[0], slot[1], group.control_point->position[2]},
        slot[3], now, std::nullopt, config_.common_frame, own);
  }

  geometry_msgs::PoseStamped pose;
  geometry_msgs::Vector3Stamped leader_velocity, leader_acceleration;
  leaderPoseInFollowerFrame(now, anchor_state->second, own, &pose,
                            &leader_velocity, &leader_acceleration);
  const Vec3 position = pointVec(pose.pose.position);
  const Vec3 velocity = vectorVec(leader_velocity.vector);
  if (!std::all_of(velocity.begin(), velocity.end(),
                   [](double value) { return std::isfinite(value); }))
    throw std::invalid_argument("detached_group_anchor_velocity_invalid");
  const double speed = std::hypot(velocity[0], velocity[1]);
  const double heading = fixedwing &&
      speed >= config_.fixedwing_min_groundspeed_for_course
      ? std::atan2(velocity[1], velocity[0]) : yaw(pose);
  const auto slot = slotTarget(position, heading, relative_offset,
                               config_.yaw_offset);
  Vec3 acceleration = vectorVec(leader_acceleration.vector);
  if (!anchor_state->second.acceleration_fresh) acceleration = {0.0, 0.0, 0.0};
  std::optional<double> turn_rate;
  if (fixedwing && !anchor_state->second.acceleration_fresh) {
    const auto& q = pose.pose.orientation;
    tf2::Quaternion quaternion(q.x, q.y, q.z, q.w);
    double roll, pitch, yaw_angle;
    tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw_angle);
    turn_rate = 9.8 * std::tan(roll) /
        std::max(speed, config_.fixedwing_min_speed);
  } else if (!fixedwing && std::isfinite(anchor_state->second.body_rate.z)) {
    turn_rate = anchor_state->second.body_rate.z;
  }
  Vec3 reference_velocity = slotVelocity(velocity, acceleration, heading,
      relative_offset, 0.0, false, turn_rate.value_or(0.0), turn_rate.has_value());
  const Vec3 own_position = pointVec(own.position_odom);
  reference_velocity = pidCatchupVelocity(reference_velocity,
      {slot[0], slot[1], slot[2]}, own_position, now, own);
  reference_velocity = smoothCourseVelocity(reference_velocity, now, own);
  const double target_altitude = position[2];
  if (fixedwing) {
    // Follow the moving anchor, including when it is loitering around an
    // external point. Separate fixed waypoints produce separate loiter
    // circles, not a rigid subgroup formation.
    const auto nominal = horizontalMotion(velocity, acceleration);
    const auto motion = limitHorizontalMotion(columnReferenceMotion(
        reference_velocity, turn_rate.value_or(nominal.course_rate), now),
        config_.fixedwing_min_speed, config_.fixedwing_max_speed, heading);
    auto target = motionTarget({slot[0], slot[1], target_altitude}, motion,
        now, own, own.header.frame_id);
    target.type_mask &= ~(mavros_msgs::PositionTarget::IGNORE_PX |
                          mavros_msgs::PositionTarget::IGNORE_PY);
    // HorizontalMotion deliberately discards Z. Restore the transformed
    // anchor Vz so height feedback only corrects the follower's height error,
    // rather than asking it to jump directly to the final external altitude.
    if (group.control_point) {
      target.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_VZ;
      target.velocity.z = velocity[2];
    }
    return target;
  }
  return positionTarget({slot[0], slot[1], target_altitude}, slot[3], now,
                        reference_velocity, "", own);
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
  refreshFormationSelection(now);
  refreshFormationGroups(now);
  if (config_.mode == "consensus") {
    const auto group = formationGroupFor(member_);
    if (group && group->detached && group->anchor_external_control &&
        group->anchor == member_) {
      message = "external-control anchor remains outside the formation";
      return true;
    }
  }
  if (last_external_reference_ &&
      (now - *last_external_reference_).toSec() < config_.external_quiet_time) {
    message = "another reference is active";
    return false;
  }
  const StateSnapshot input = snapshot();
  const auto failure = readiness(now, input);
  if (failure) { message = *failure; return false; }
  const bool restartingAfterDropout = reason_.rfind("dropout:", 0) == 0;
  dropped_members_.clear();
  active_members_dirty_ = true;
  acquire_force_capture_ = restartingAfterDropout;
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
  enabled_ = true;
  if (config_.mode == "consensus") {
    paced_speed_.reset(); paced_speed_stamp_.reset(); progress_error_filter_.clear();
    consensus_heading_.reset(); consensus_heading_stamp_.reset();
    acquire_heading_.reset(); acquire_stage_ = "waiting_for_team";
    acquire_started_ = now; reason_ = "acquiring_waiting_for_team";
  } else reason_ = "following";
  filtered_course_.reset(); filtered_course_stamp_.reset(); filtered_speed_.reset();
  resetCatchupPid();
  resetConsensusMotion();
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
  refreshFormationSelection(now);
  refreshFormationGroups(now);
  const StateSnapshot input = snapshot();
  if (config_.mode == "consensus") {
    if (!enabled_) {
      if (!last_idle_center_publish_ ||
          (now - *last_idle_center_publish_).toSec() >= idle_center_period_) {
        if (publishCenter(now, input)) last_idle_center_publish_ = now;
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
  const auto own_group = config_.mode == "consensus"
      ? formationGroupFor(member_) : std::optional<FormationGroup>{};
  const bool detached_group = own_group && own_group->detached;
  if (config_.mode == "consensus" && !detached_group && acquire_stage_ &&
      *acquire_stage_ != "waiting_for_team" && !teamEnabled(now, fresh_input)) {
    beginDropout("peer_formation_stopped"); return;
  }
  if (config_.mode == "consensus" && !detached_group && acquire_stage_ &&
      *acquire_stage_ != "following" && acquire_started_ &&
      (now - *acquire_started_).toSec() > config_.acquire_timeout) {
    beginDropout("acquisition_timeout"); return;
  }
  try {
    if (config_.mode == "consensus" && !detached_group)
      ensureSlotAssignment(now, fresh_input);
    const auto target = config_.mode == "consensus" && !detached_group
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
  formation_state_pub_ = nh_.advertise<FormationState>(
      "/formation/" + config_.team_id + "/state", 1, true);
  auto geometry = std::make_shared<FormationGeometry>();
  if (config_.mode == "consensus") {
    if (config_.members.size() < 2 || config_.common_frame.empty())
      throw std::invalid_argument("consensus requires members and common_frame");
    if (config_.group_size < 1 || !std::isfinite(config_.group_spacing) ||
        config_.group_spacing < 0.0) {
      throw std::invalid_argument("group_size must be positive and group_spacing nonnegative");
    }
    auto group_plan = makeGroupPlan(config_.members, config_.group_size);
    shared_->updateFormationGroupPlan(group_plan);
    geometry->slots.reserve(config_.members.size());
    const auto nested_slots = hierarchicalSlots(
        group_plan, config_.shape, config_.spacing, config_.group_spacing,
        std::max(config_.minimum_slot_distance,
                 config_.fixedwing_minimum_slot_distance));
    for (const auto& member : config_.members) {
      const auto slot = nested_slots.find(member);
      if (slot == nested_slots.end())
        throw std::invalid_argument("missing nested consensus slot for " + member);
      geometry->slots.push_back(slot->second);
    }
    for (const auto& slot : geometry->slots) {
      if (!std::isfinite(slot[0]) || !std::isfinite(slot[1]) ||
          !std::isfinite(slot[2])) {
        throw std::invalid_argument("consensus slot must be finite");
      }
    }
    for (std::size_t i = 0; i < geometry->slots.size(); ++i) {
      for (std::size_t j = i + 1; j < geometry->slots.size(); ++j) {
        const double dx = geometry->slots[i][0] - geometry->slots[j][0];
        const double dy = geometry->slots[i][1] - geometry->slots[j][1];
        const double dz = geometry->slots[i][2] - geometry->slots[j][2];
        if (std::sqrt(dx * dx + dy * dy + dz * dz) <
            config_.minimum_slot_distance) {
          throw std::invalid_argument(
              "consensus slots are closer than minimum separation");
        }
      }
    }
    if (!config_.waypoints_are_final) {
      config_.waypoints = roundedPath(config_.waypoints,
                                      config_.corner_radius,
                                      config_.loop_path);
      config_.waypoints_are_final = true;
    }
    (void)pathReferenceState({0.0, 0.0, 0.0}, config_.waypoints,
                             config_.cruise_speed, config_.lookahead,
                             config_.loop_path);
    FormationSelection selection;
    selection.shape = config_.shape;
    selection.waypoints = config_.waypoints;
    selection.slots = geometry->slots;
    selection.loop_path = config_.loop_path;
    shared_->updateFormationSelection(std::move(selection));
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
  publishFormationState();
  formation_state_timer_ = nh_.createWallTimer(
      ros::WallDuration(0.5), &FormationRuntime::formationStateTimer, this);
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

void FormationRuntime::formationStateTimer(const ros::WallTimerEvent&) {
  publishFormationState();
}

void FormationRuntime::publishFormationState() {
  if (!formation_state_pub_) return;

  FormationSelection selection;
  FormationGroupPlan group_plan;
  {
    std::lock_guard<std::mutex> lock(shared_->mutex);
    selection = shared_->formation_selection;
    group_plan = shared_->formation_groups;
  }

  FormationState message;
  message.header.stamp = ros::Time::now();
  message.header.frame_id = config_.common_frame;
  message.team_id = config_.team_id;
  message.mode = config_.mode;
  message.running = running_.load();
  message.common_frame = config_.common_frame;
  message.leader_name = config_.mode == "leader_follow"
      ? config_.leader_name : std::string();
  message.shape = config_.mode == "consensus"
      ? selection.shape : config_.shape;
  message.loop_path = config_.mode == "consensus"
      ? selection.loop_path : config_.loop_path;
  message.spacing = config_.spacing;
  message.group_spacing = config_.group_spacing;
  message.configured_group_size = config_.mode == "consensus"
      ? static_cast<std::uint32_t>(std::max(0, group_plan.group_size)) : 0;
  message.members = config_.members;
  message.selection_revision = selection.revision;
  message.group_revision = group_plan.revision;
  message.waypoints.reserve(selection.waypoints.size());
  for (const auto& waypoint : selection.waypoints) {
    geometry_msgs::Point point;
    point.x = waypoint[0];
    point.y = waypoint[1];
    point.z = waypoint[2];
    message.waypoints.push_back(point);
  }
  message.groups.reserve(group_plan.groups.size());
  for (const auto& source : group_plan.groups) {
    FormationGroupInfo output;
    output.id = source.id;
    output.uavs = source.members;
    output.nominal_size = static_cast<std::uint32_t>(source.nominal_size);
    output.detached = source.detached;
    output.anchor_external_control = source.anchor_external_control;
    output.anchor = source.anchor;
    output.has_control_point = source.control_point.has_value();
    if (source.control_point) {
      output.control_point.x = source.control_point->position[0];
      output.control_point.y = source.control_point->position[1];
      output.control_point.z = source.control_point->position[2];
      output.control_heading = source.control_point->heading;
      output.control_heading_valid = source.control_point->heading_valid;
      output.control_point_stamp = source.control_point->stamp;
    }
    if (source.detached) ++message.detached_group_count;
    message.groups.push_back(std::move(output));
  }
  message.group_count = static_cast<std::uint32_t>(message.groups.size());
  formation_state_pub_.publish(message);
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
        running_.store(false);
        if (config_.mode == "consensus") clearAcquisitionPlan();
        publishFormationState();
        return false;
      }
    } else completed.push_back(follower.get());
  }
  if (config_.mode == "consensus" && !enabled) {
    clearAcquisitionPlan();
    const auto groups = shared_->formationGroupPlan();
    shared_->updateFormationGroupPlan(
        makeGroupPlan(config_.members, groups.group_size));
  }
  running_.store(enabled);
  publishFormationState();
  message = enabled ? "team started" : "team stopped";
  return true;
}

bool FormationRuntime::updateFormationSelection(
    const std::string& shape, const std::vector<Vec3>& waypoints,
    bool loop_path, std::string& message) {
  if (config_.mode != "consensus") {
    message = "shape and path commands require consensus mode";
    return false;
  }
  try {
    FormationSelection selection;
    selection.shape = shape;
    selection.waypoints = waypoints;
    selection.loop_path = loop_path;
    selection.slots.reserve(config_.members.size());
    const auto groups = shared_->formationGroupPlan();
    const auto slots = hierarchicalSlots(
        groups, shape, config_.spacing, config_.group_spacing,
        std::max(config_.minimum_slot_distance,
                 config_.fixedwing_minimum_slot_distance));
    for (const auto& member : config_.members) {
      const auto slot = slots.find(member);
      if (slot == slots.end()) continue;
      selection.slots.push_back(slot->second);
    }
    for (std::size_t i = 0; i < selection.slots.size(); ++i) {
      for (std::size_t j = i + 1; j < selection.slots.size(); ++j) {
        const double dx = selection.slots[i][0] - selection.slots[j][0];
        const double dy = selection.slots[i][1] - selection.slots[j][1];
        const double dz = selection.slots[i][2] - selection.slots[j][2];
        if (std::sqrt(dx * dx + dy * dy + dz * dz) <
            config_.minimum_slot_distance) {
          throw std::invalid_argument(
              "selected formation slots are too close");
        }
      }
    }
    (void)pathReferenceState({0.0, 0.0, 0.0}, selection.waypoints,
        config_.cruise_speed, config_.lookahead, selection.loop_path);
    shared_->updateFormationSelection(std::move(selection));
    clearAcquisitionPlan();
    publishFormationState();
    message = "shape and route selection updated";
    return true;
  } catch (const std::exception& error) {
    message = error.what();
    return false;
  }
}

bool FormationRuntime::updateGroupSize(int group_size, std::string& message) {
  if (config_.mode != "consensus") {
    message = "group sizing is only available in consensus mode";
    return false;
  }
  if (group_size < 1) {
    message = "group_size must be positive";
    return false;
  }
  try {
    const auto selection = shared_->formationSelection();
    {
      // Detachment is updated by follower callbacks. Keep the snapshot and
      // replacement atomic so a concurrent external takeover is not lost.
      std::lock_guard<std::mutex> lock(shared_->mutex);
      const auto current = shared_->formation_groups;
      FormationGroupPlan next;
      next.group_size = group_size;
      std::set<std::string> detached_ids;
      std::vector<std::string> movable_members;
      for (const auto& group : current.groups) {
        if (group.detached) {
          next.groups.push_back(group);
          detached_ids.insert(group.id);
        } else {
          movable_members.insert(movable_members.end(), group.members.begin(),
                                 group.members.end());
        }
      }
      auto resized = makeGroupPlan(movable_members, group_size);
      std::size_t id = 1;
      for (auto& group : resized.groups) {
        while (detached_ids.count("group_" + std::to_string(id))) ++id;
        group.id = "group_" + std::to_string(id++);
        next.groups.push_back(std::move(group));
      }
      std::map<std::string, std::size_t> member_order;
      for (std::size_t i = 0; i < config_.members.size(); ++i)
        member_order[config_.members[i]] = i;
      std::sort(next.groups.begin(), next.groups.end(), [&](const auto& first,
                                                            const auto& second) {
        const auto rank = [&](const FormationGroup& group) {
          std::size_t value = config_.members.size();
          for (const auto& member : group.members)
            value = std::min(value, member_order.at(member));
          return value;
        };
        return rank(first) < rank(second);
      });
      const auto slots = hierarchicalSlots(
          next, selection.shape, config_.spacing,
          config_.group_spacing,
          std::max(config_.minimum_slot_distance,
                   config_.fixedwing_minimum_slot_distance));
      std::size_t movable_member_count = 0;
      for (const auto& group : next.groups)
        if (!group.detached) movable_member_count += group.members.size();
      if (slots.size() != movable_member_count)
        throw std::invalid_argument("active group slots are incomplete");
      next.revision = current.revision + 1;
      shared_->formation_groups = std::move(next);
      shared_->markChanged();
    }
    config_.group_size = group_size;
    clearAcquisitionPlan();
    publishFormationState();
    message = "group size updated; active groups are reacquiring their slots";
    return true;
  } catch (const std::exception& error) {
    message = error.what();
    return false;
  }
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


