#include <xd_uav_formation/FormationControl.h>
#include <xd_uav_formation/runtime.hpp>
#include <ros/master.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <tf2_ros/transform_listener.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <regex>
#include <set>
#include <string>
#include <thread>
#include <utility>

namespace xd_uav_formation {
namespace {

bool simpleName(const std::string& value) {
  static const std::regex pattern("^[A-Za-z0-9_-]+$");
  return std::regex_match(value, pattern);
}

YAML::Node mergeMaps(const YAML::Node& first, const YAML::Node& second) {
  YAML::Node result(YAML::NodeType::Map);
  if (first && first.IsMap())
    for (const auto& item : first) result[item.first.as<std::string>()] = item.second;
  if (second && second.IsMap())
    for (const auto& item : second) result[item.first.as<std::string>()] = item.second;
  return result;
}

template <class T>
T value(const YAML::Node& node, const std::string& key, const T& fallback) {
  return node[key] ? node[key].as<T>() : fallback;
}

Vec3 readVec3(const YAML::Node& node, const std::string& what) {
  if (!node || !node.IsSequence() || node.size() != 3)
    throw std::invalid_argument(what + " must contain XYZ values");
  return {node[0].as<double>(), node[1].as<double>(), node[2].as<double>()};
}

Config parseConfig(const std::string& mode, const YAML::Node& node,
                   const std::string& root_team) {
  Config config;
  config.mode = mode;
  config.team_id = value<std::string>(node, "team_id", root_team);
  config.leader_name = value<std::string>(node, "leader_name", "uav1");
  config.common_frame = value<std::string>(node, "common_frame", "world");
  config.shape = value<std::string>(node, "shape", mode == "consensus" ? "circle" : "manual");
  if (node["members"] && node["members"].IsSequence())
    for (const auto& member : node["members"]) config.members.push_back(member.as<std::string>());
  if (node["offsets"] && node["offsets"].IsMap())
    for (const auto& entry : node["offsets"])
      config.offsets[entry.first.as<std::string>()] = readVec3(entry.second, "member offset");
  if (node["member_offsets"] && node["member_offsets"].IsMap())
    for (const auto& entry : node["member_offsets"])
      config.offsets[entry.first.as<std::string>()] = readVec3(entry.second, "member offset");
  if (node["waypoints"] && node["waypoints"].IsSequence())
    for (const auto& point : node["waypoints"]) config.waypoints.push_back(readVec3(point, "waypoint"));

#define XD_DOUBLE(field, key) config.field = value<double>(node, key, config.field)
#define XD_BOOL(field, key) config.field = value<bool>(node, key, config.field)
#define XD_STRING(field, key) config.field = value<std::string>(node, key, config.field)
  XD_DOUBLE(minimum_slot_distance, "minimum_slot_distance");
  XD_DOUBLE(spacing, "spacing");
  config.follower_count = value<int>(node, "follower_count", config.follower_count);
  config.slot_index = value<int>(node, "slot_index", config.slot_index);
  XD_DOUBLE(corner_radius, "corner_radius");
  XD_DOUBLE(cruise_speed, "cruise_speed");
  config.lookahead = value<double>(node, "lookahead_distance", config.lookahead);
  XD_DOUBLE(consensus_gain, "consensus_gain");
  XD_DOUBLE(convergence_time, "convergence_time");
  XD_DOUBLE(max_consensus_velocity, "max_consensus_velocity");
  config.acquire_consensus_gain = value<double>(node, "acquire_consensus_gain", config.consensus_gain);
  config.acquire_max_consensus_velocity = value<double>(node, "acquire_max_consensus_velocity", config.max_consensus_velocity);
  config.acquire_max_speed_adjustment = value<double>(node, "acquire_max_speed_adjustment", config.max_consensus_velocity);
  XD_DOUBLE(formation_heading_rate_limit, "formation_heading_rate_limit");
  XD_DOUBLE(formation_speed_slew_rate, "formation_speed_slew_rate");
  XD_DOUBLE(formation_course_slew_rate, "formation_course_slew_rate");
  config.route_correction_gain = value<double>(node, "route_correction_gain", config.consensus_gain);
  config.max_route_correction_velocity = value<double>(node, "max_route_correction_velocity", config.max_consensus_velocity);
  XD_BOOL(loop_path, "loop_path");
  config.acquire_tolerance = value<double>(node, "acquire_tolerance", std::max(2.0, config.spacing * 0.1));
  config.acquire_height_tolerance = value<double>(node, "acquire_height_tolerance", std::max(0.75, config.spacing * 0.025));
  XD_DOUBLE(acquire_vertical_spacing, "acquire_vertical_spacing");
  XD_DOUBLE(acquire_timeout, "acquire_timeout");
  XD_DOUBLE(fixedwing_minimum_slot_distance, "fixedwing_minimum_slot_distance");
  XD_DOUBLE(fixedwing_min_speed, "fixedwing_min_speed");
  XD_DOUBLE(fixedwing_max_catchup_speed, "fixedwing_max_catchup_speed");
  XD_DOUBLE(fixedwing_min_groundspeed_for_course, "fixedwing_min_groundspeed_for_course");
  XD_DOUBLE(multirotor_max_speed, "multirotor_max_speed");
  XD_DOUBLE(catchup_gain, "catchup_gain");
  XD_DOUBLE(catchup_ki, "catchup_ki");
  XD_DOUBLE(catchup_kd, "catchup_kd");
  XD_DOUBLE(catchup_deadband, "catchup_deadband");
  XD_DOUBLE(catchup_integral_limit, "catchup_integral_limit");
  XD_DOUBLE(catchup_derivative_filter_time_constant, "catchup_derivative_filter_time_constant");
  XD_DOUBLE(multirotor_max_catchup_speed, "multirotor_max_catchup_speed");
  XD_DOUBLE(yaw_offset, "yaw_offset");
  XD_DOUBLE(rate, "rate");
  XD_DOUBLE(state_timeout, "state_timeout");
  XD_DOUBLE(mavros_timeout, "mavros_timeout");
  XD_DOUBLE(tf_max_age, "tf_max_age");
  XD_DOUBLE(future_stamp_tolerance, "future_stamp_tolerance");
  XD_DOUBLE(tf_lookup_timeout, "tf_lookup_timeout");
  XD_DOUBLE(tf_buffer_cache_time, "tf_buffer_cache_time");
  XD_DOUBLE(external_quiet_time, "external_quiet_time");
  XD_STRING(reference_topic, "reference_topic");
  XD_STRING(path_topic, "path_topic");
  XD_STRING(trajectory_topic, "trajectory_topic");
  XD_STRING(team_cancel_topic, "team_cancel_topic");
  XD_STRING(preempt_topic, "preempt_topic");
  XD_STRING(task_goal_topic, "task_goal_topic");
#undef XD_DOUBLE
#undef XD_BOOL
#undef XD_STRING
  if (mode == "consensus" && config.acquire_consensus_gain == 0.1 &&
      config.consensus_gain != 0.1 && !node["acquire_consensus_gain"])
    config.acquire_consensus_gain = config.consensus_gain;
  return config;
}

std::string serviceName(const std::string& member, const std::string& mode) {
  const std::string node = mode == "consensus" ? "formation_consensus" : "formation_follower";
  return "/" + member + "/" + node + "/set_enabled";
}

std::set<std::string> existingServices() {
  XmlRpc::XmlRpcValue args, result, payload;
  args[0] = ros::this_node::getName();
  if (!ros::master::execute("getSystemState", args, result, payload, false) ||
      !payload.valid() || payload.getType() != XmlRpc::XmlRpcValue::TypeArray ||
      payload.size() < 3) {
    throw std::runtime_error("cannot inspect ROS master services");
  }
  std::set<std::string> names;
  const auto& services = payload[2];
  for (int i = 0; i < services.size(); ++i)
    if (services[i].size() > 0) names.insert(static_cast<std::string>(services[i][0]));
  return names;
}

}  // namespace

class FormationManager {
 public:
  FormationManager() : private_nh_("~") {
    std::string config_path, common_path;
    private_nh_.param<std::string>("config", config_path, "");
    private_nh_.param<std::string>("common_config", common_path, "");
    private_nh_.param("spawn_members", spawn_members_, false);
    YAML::Node profile = loadYaml(config_path);
    YAML::Node common = loadYaml(common_path);
    common = mergeMaps(common, profile["common"]);
    service_timeout_ = private_nh_.param("service_timeout",
        value<double>(common, "service_timeout", 3.0));
    if (!std::isfinite(service_timeout_) || service_timeout_ <= 0.0)
      throw std::invalid_argument("service_timeout must be positive and finite");
    loadConfigs(profile, common);

    if (spawn_members_) {
      if (configs_.empty()) throw std::invalid_argument("formation config has no configured mode");
      std::vector<std::string> members;
      for (const auto& pair : configs_)
        members.insert(members.end(), pair.second.members.begin(), pair.second.members.end());
      std::sort(members.begin(), members.end());
      members.erase(std::unique(members.begin(), members.end()), members.end());
      for (const auto& mode : {std::string("consensus"), std::string("leader_follow")}) {
        auto it = configs_.find(mode);
        if (it == configs_.end()) continue;
        const ros::Duration cache_time(it->second.tf_buffer_cache_time);
        if (!tf_buffer_ || cache_time.toSec() > tf_cache_seconds_) {
          tf_cache_seconds_ = cache_time.toSec();
          tf_buffer_.reset(new tf2_ros::Buffer(cache_time));
          tf_listener_.reset(new tf2_ros::TransformListener(*tf_buffer_));
        }
      }
      for (const auto& pair : configs_) {
        const auto services = existingServices();
        for (const auto& member : pair.second.members) {
          const std::string expected = serviceName(member, pair.first);
          if (services.count(expected))
            throw std::runtime_error(expected + " already exists; stop old formation nodes first");
        }
        auto runtime = std::make_unique<FormationRuntime>(nh_, pair.second,
                                                          tf_buffer_, true);
        runtimes_[pair.first] = std::move(runtime);
      }
      std::set<std::string> telemetry_names;
      for (const auto& pair : configs_) {
        telemetry_names.insert(pair.second.members.begin(), pair.second.members.end());
        if (pair.first == "leader_follow") telemetry_names.insert(pair.second.leader_name);
      }
      for (const auto& name : telemetry_names) {
        telemetry_subscribers_.push_back(nh_.subscribe<xd_uav_controller::ControlState>(
            "/" + name + "/control_manager/state", 1,
            [this, name](const xd_uav_controller::ControlState::ConstPtr& message) {
              for (auto& runtime : runtimes_) runtime.second->stateCallback(message, name);
            }));
        telemetry_subscribers_.push_back(nh_.subscribe<mavros_msgs::State>(
            "/" + name + "/mavros/state", 1,
            [this, name](const mavros_msgs::State::ConstPtr& message) {
              for (auto& runtime : runtimes_) runtime.second->mavrosCallback(message, name);
            }));
      }
    }
    set_service_ = nh_.advertiseService("/formation/set_enabled",
        &FormationManager::handleSet, this);
    stop_service_ = nh_.advertiseService("/formation/stop",
        &FormationManager::handleStop, this);
    ROS_INFO("[xd_uav_formation] C++ manager ready; modes=%zu; shared runtime=%s",
        configs_.size(), spawn_members_ ? "enabled" : "external nodes");
  }

 private:
  static YAML::Node loadYaml(const std::string& path) {
    if (path.empty()) return YAML::Node(YAML::NodeType::Map);
    YAML::Node node = YAML::LoadFile(path);
    if (!node || !node.IsMap()) throw std::invalid_argument("formation YAML root must be a mapping");
    return node;
  }

  void loadConfigs(const YAML::Node& profile, const YAML::Node& common) {
    const std::string root_team = value<std::string>(profile, "team_id", "team1");
    bool nested = false;
    for (const auto& mode : {std::string("leader_follow"), std::string("consensus")})
      if (profile[mode] && profile[mode].IsMap()) nested = true;
    std::vector<std::pair<std::string, YAML::Node>> sources;
    if (nested) {
      for (const auto& mode : {std::string("leader_follow"), std::string("consensus")})
        if (profile[mode] && profile[mode].IsMap()) sources.emplace_back(mode, profile[mode]);
    } else if (profile["mode"] && profile["mode"].IsScalar()) {
      const std::string mode = profile["mode"].as<std::string>();
      if (mode == "leader_follow" || mode == "consensus") sources.emplace_back(mode, profile);
    }
    for (const auto& source : sources) {
      YAML::Node merged = mergeMaps(common, source.second);
      if (!merged["team_id"]) merged["team_id"] = root_team;
      Config config = parseConfig(source.first, merged, root_team);
      if (config.members.empty()) throw std::invalid_argument(source.first + " config must contain members");
      std::set<std::string> unique;
      for (const auto& member : config.members) {
        if (!simpleName(member)) throw std::invalid_argument("members must be simple UAV names");
        if (!unique.insert(member).second) throw std::invalid_argument("members contains duplicates");
      }
      if (!simpleName(config.team_id)) throw std::invalid_argument(source.first + " has invalid team_id");
      if (source.first == "consensus" && config.members.size() < 2)
        throw std::invalid_argument("consensus requires at least two members");
      if (source.first == "leader_follow") {
        if (!simpleName(config.leader_name) || unique.count(config.leader_name))
          throw std::invalid_argument("leader_follow requires an external leader");
        for (const auto& member : config.members)
          if (!config.offsets.count(member))
            throw std::invalid_argument("leader_follow requires an offset for " + member);
      }
      config.waypoints = source.first == "consensus"
          ? roundedPath(config.waypoints, config.corner_radius, config.loop_path)
          : config.waypoints;
      configs_[source.first] = std::move(config);
    }
  }

  bool handleSet(xd_uav_formation::FormationControl::Request& request,
                 xd_uav_formation::FormationControl::Response& response) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!request.enabled && request.mode.empty()) {
      response.success = stopAll(response.message);
      return true;
    }
    std::string mode = request.mode;
    std::transform(mode.begin(), mode.end(), mode.begin(), ::tolower);
    if (mode != "consensus" && mode != "leader_follow") {
      response.success = false; response.message = "mode must be consensus or leader_follow"; return true;
    }
    std::vector<std::string> members;
    std::string team;
    if (!resolve(mode, request.members, members, team, response.message)) {
      response.success = false; return true;
    }
    if (spawn_members_) {
      auto runtime = runtimes_.find(mode);
      if (runtime == runtimes_.end()) {
        response.success = false; response.message = "no configured " + mode + " mode"; return true;
      }
      if (active_ && (std::get<0>(*active_) != team || std::get<1>(*active_) != mode ||
                      std::get<2>(*active_) != members)) {
        auto old = runtimes_.find(std::get<1>(*active_));
        std::string stop_message;
        if (old != runtimes_.end() && !old->second->setMembers(false, stop_message)) {
          response.success = false; response.message = "could not stop previous mode: " + stop_message; return true;
        }
        active_.reset();
      }
      if (!runtime->second->setMembers(request.enabled, response.message)) {
        response.success = false;
        if (request.enabled && mode == "consensus") runtime->second->clearAcquisitionPlan();
        return true;
      }
      if (request.enabled) active_ = std::make_tuple(team, mode, members);
      else if (active_ && std::get<1>(*active_) == mode) active_.reset();
      response.success = true;
      return true;
    }
    response.success = callMembers(members, mode, request.enabled, response.message);
    return true;
  }

  bool handleStop(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& response) {
    std::lock_guard<std::mutex> lock(mutex_);
    response.success = stopAll(response.message);
    return true;
  }

  bool resolve(const std::string& mode, const std::vector<std::string>& requested,
               std::vector<std::string>& members, std::string& team,
               std::string& error) {
    auto configured = configs_.find(mode);
    if (configured != configs_.end()) {
      members = configured->second.members; team = configured->second.team_id;
      if (!requested.empty() && requested != members) {
        error = "members are defined by the loaded YAML profile"; return false;
      }
    } else {
      members = requested;
      if (members.empty()) { error = "members must be specified when no profile is loaded"; return false; }
      team.clear();
      std::set<std::string> leaders;
      for (const auto& member : members) {
        const std::string ns = "/" + member + (mode == "consensus" ? "/formation_consensus" : "/formation_follower");
        std::string configured_team, configured_mode;
        if (!nh_.getParam(ns + "/team_id", configured_team) || !simpleName(configured_team)) {
          error = member + " has no valid configured team_id"; return false;
        }
        nh_.param<std::string>(ns + "/mode", configured_mode, mode == "leader_follow" ? mode : "");
        if (configured_mode != mode) { error = member + " has a different formation mode"; return false; }
        if (team.empty()) team = configured_team;
        else if (team != configured_team) { error = "members belong to different configured teams"; return false; }
        if (mode == "consensus") {
          std::vector<std::string> configured_members;
          nh_.getParam(ns + "/members", configured_members);
          if (configured_members != members) { error = member + " has a different consensus member order"; return false; }
        } else {
          std::string leader;
          nh_.param<std::string>(ns + "/leader_name", leader, "");
          if (!simpleName(leader) || leader == member) { error = member + " has no valid external leader"; return false; }
          leaders.insert(leader);
        }
      }
      if (mode == "leader_follow" && leaders.size() != 1) {
        error = "all leader-follow members must use the same leader"; return false;
      }
    }
    if (members.empty() || (mode == "consensus" && members.size() < 2)) {
      error = mode == "consensus" ? "consensus requires at least two members" : "members are required"; return false;
    }
    std::set<std::string> unique;
    for (const auto& member : members) {
      if (!simpleName(member)) { error = "members must contain valid UAV namespace names"; return false; }
      if (!unique.insert(member).second) { error = "members contains duplicates"; return false; }
    }
    return true;
  }

  bool callMembers(const std::vector<std::string>& members, const std::string& mode,
                   bool enabled, std::string& message) {
    std::vector<std::string> completed;
    for (const auto& member : members) {
      const std::string service = serviceName(member, mode);
      if (!ros::service::waitForService(service, ros::Duration(service_timeout_))) {
        message = member + ": timed out waiting for service";
        if (enabled) rollback(completed, mode);
        return false;
      }
      auto client = nh_.serviceClient<std_srvs::SetBool>(service);
      std_srvs::SetBool call; call.request.data = enabled;
      if (!client.call(call) || !call.response.success) {
        message = member + ": " + (call.response.message.empty() ? "service call failed" : call.response.message);
        if (enabled) rollback(completed, mode);
        return false;
      }
      completed.push_back(member);
    }
    message = enabled ? "team started" : "team stopped";
    return true;
  }

  void rollback(const std::vector<std::string>& members, const std::string& mode) {
    for (auto it = members.rbegin(); it != members.rend(); ++it) {
      auto client = nh_.serviceClient<std_srvs::SetBool>(serviceName(*it, mode));
      std_srvs::SetBool call; call.request.data = false; client.call(call);
    }
  }

  bool stopAll(std::string& message) {
    bool success = true;
    std::vector<std::string> errors;
    for (auto& pair : runtimes_) {
      std::string result;
      if (!pair.second->setMembers(false, result)) { success = false; errors.push_back(pair.first + ": " + result); }
    }
    if (!spawn_members_) {
      XmlRpc::XmlRpcValue args, result, payload;
      args[0] = ros::this_node::getName();
      if (!ros::master::execute("getSystemState", args, result, payload, false) ||
          !payload.valid() || payload.getType() != XmlRpc::XmlRpcValue::TypeArray ||
          payload.size() < 3) { message = "cannot inspect ROS master"; return false; }
      const std::regex pattern("^/([A-Za-z0-9_-]+)/formation_(consensus|follower)/set_enabled$");
      const auto& services = payload[2];
      for (int i = 0; i < services.size(); ++i) {
        const std::string service_name = static_cast<std::string>(services[i][0]);
        std::smatch match;
        if (!std::regex_match(service_name, match, pattern)) continue;
        const std::string mode = match[2] == "consensus" ? "consensus" : "leader_follow";
        auto client = nh_.serviceClient<std_srvs::SetBool>(service_name);
        std_srvs::SetBool call; call.request.data = false;
        if (!client.call(call) || !call.response.success) {
          success = false; errors.push_back(match[1].str() + ": stop failed");
        }
      }
    }
    active_.reset();
    if (errors.empty()) message = "all formations stopped";
    else {
      message.clear();
      for (std::size_t i = 0; i < errors.size(); ++i) { if (i) message += "; "; message += errors[i]; }
    }
    return success;
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  std::mutex mutex_;
  bool spawn_members_{false};
  double service_timeout_{3.0};
  double tf_cache_seconds_{0.0};
  std::map<std::string, Config> configs_;
  std::map<std::string, std::unique_ptr<FormationRuntime>> runtimes_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  ros::ServiceServer set_service_;
  ros::ServiceServer stop_service_;
  std::vector<ros::Subscriber> telemetry_subscribers_;
  std::optional<std::tuple<std::string, std::string, std::vector<std::string>>> active_;
};

}  // namespace xd_uav_formation

int main(int argc, char** argv) {
  ros::init(argc, argv, "formation_manager");
  try {
    xd_uav_formation::FormationManager manager;
    const unsigned int hardware_threads = std::thread::hardware_concurrency();
    const unsigned int spinner_threads = hardware_threads > 2
        ? hardware_threads / 2 : 2;
    ros::AsyncSpinner spinner(spinner_threads);
    spinner.start();
    ros::waitForShutdown();
    spinner.stop();
  } catch (const std::exception& error) {
    ROS_FATAL("[xd_uav_formation] C++ manager failed: %s", error.what());
    return 1;
  }
  return 0;
}
