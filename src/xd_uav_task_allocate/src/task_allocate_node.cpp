#include <xd_uav_task_allocate/core.hpp>
#include <xd_uav_task_allocate/CancelPlanning.h>
#include <xd_uav_task_allocate/GetMissionState.h>
#include <xd_uav_task_allocate/GlobalTargetArray.h>
#include <xd_uav_task_allocate/GlobalTarget.h>
#include <xd_uav_task_allocate/LoadSearchAreas.h>
#include <xd_uav_task_allocate/PlannerStatus.h>
#include <xd_uav_task_allocate/RescueTask.h>
#include <xd_uav_task_allocate/RescueTaskArray.h>
#include <xd_uav_task_allocate/SearchAreaArray.h>
#include <xd_uav_task_allocate/SetVehicleEnabled.h>
#include <xd_uav_task_allocate/TargetCommand.h>
#include <xd_uav_task_allocate/TaskCommand.h>
#include <xd_uav_task_allocate/VehicleCommand.h>
#include <xd_uav_controller/ControlState.h>
#include <xd_uav_controller/PathStatus.h>
#include <xd_uav_task_execute/ExecuteTaskAction.h>
#include <actionlib/client/simple_action_client.h>
#include <xd_uav_track/DetectionArray.h>
#include <target_geolocation/GeolocatedTarget.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/PositionTarget.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>
#include <xmlrpcpp/XmlRpcValue.h>
#include <boost/bind/bind.hpp>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <numeric>
#include <set>
#include <sstream>
#include <string>
#include <vector>

namespace xd_uav_task_allocate {
namespace {
constexpr char kIdle[] = "IDLE";
constexpr char kLoaded[] = "LOADED";
constexpr char kActive[] = "ACTIVE";
constexpr char kPaused[] = "PAUSED";
constexpr char kAborted[] = "ABORTED";
constexpr char kCompleted[] = "COMPLETED";

double distance3d(const Vec3& a, const Vec3& b) {
  const auto x=a[0]-b[0], y=a[1]-b[1], z=a[2]-b[2];
  return std::sqrt(x*x+y*y+z*z);
}

std::string stripSlash(std::string frame) {
  while (!frame.empty() && frame.front() == '/') frame.erase(frame.begin());
  return frame;
}

std::string xmlString(const XmlRpc::XmlRpcValue& value,
                      const std::string& fallback = {}) {
  if (value.getType() == XmlRpc::XmlRpcValue::TypeString)
    return static_cast<std::string>(value);
  return fallback;
}

double xmlDouble(const XmlRpc::XmlRpcValue& value, double fallback) {
  try {
    if (value.getType() == XmlRpc::XmlRpcValue::TypeDouble) return static_cast<double>(value);
    if (value.getType() == XmlRpc::XmlRpcValue::TypeInt) return static_cast<int>(value);
  } catch (...) {}
  return fallback;
}

bool xmlBool(const XmlRpc::XmlRpcValue& value, bool fallback) {
  try {
    if (value.getType() == XmlRpc::XmlRpcValue::TypeBoolean)
      return static_cast<bool>(value);
    if (value.getType() == XmlRpc::XmlRpcValue::TypeInt)
      return static_cast<int>(value) != 0;
  } catch (...) {}
  return fallback;
}

XmlRpc::XmlRpcValue child(const XmlRpc::XmlRpcValue& parent, const std::string& key) {
  if (parent.getType() == XmlRpc::XmlRpcValue::TypeStruct && parent.hasMember(key))
    return parent[key];
  return XmlRpc::XmlRpcValue();
}

std::string nestedString(const XmlRpc::XmlRpcValue& config,
                         const std::string& group, const std::string& key,
                         const std::string& fallback) {
  return xmlString(child(child(config, group), key), fallback);
}

double nestedDouble(const XmlRpc::XmlRpcValue& config,
                    const std::string& group, const std::string& key,
                    double fallback) {
  return xmlDouble(child(child(config, group), key), fallback);
}

}  // namespace

class TaskAllocateCoordinator {
 public:
  TaskAllocateCoordinator()
      : private_nh_("~"), tf_listener_(tf_buffer_) {
    private_nh_.param<std::string>("mission/shared_frame", shared_frame_, "world");
    shared_frame_ = stripSlash(shared_frame_);
    if (shared_frame_.empty()) throw std::runtime_error("mission/shared_frame is empty");
    private_nh_.param("geolocation/maximum_pose_time_difference_sec", pose_sync_tolerance_, 0.15);
    private_nh_.param("geolocation/tf_lookup_timeout_sec", tf_timeout_, 0.05);
    private_nh_.param("geolocation/world_odometry_timeout_sec", odometry_timeout_, 0.5);
    private_nh_.param("vehicle_health/timeout_sec", health_timeout_, 0.5);
    private_nh_.param("vehicle_health/require_state_valid", require_state_valid_, true);
    private_nh_.param("vehicle_health/require_localization_valid", require_localization_valid_, true);
    private_nh_.param("vehicle_health/require_odometry_fresh", require_odometry_fresh_, true);
    private_nh_.param("execution/require_all_scouts_ready", require_all_scouts_ready_, true);
    private_nh_.param("execution/minimum_ready_scouts", minimum_ready_scouts_, 1);
    private_nh_.param("runtime/update_rate_hz", update_rate_, 5.0);
    private_nh_.param("execution/completion_grace_sec", completion_grace_sec_, 2.0);
    private_nh_.param("target_registry/association_radius_m", association_radius_, 4.0);
    private_nh_.param("target_registry/confirmation_hits", confirmation_hits_, 5);
    private_nh_.param("target_registry/confirmation_window_sec", confirmation_window_, 2.0);
    private_nh_.param("target_registry/stale_timeout_sec", stale_timeout_, 15.0);
    private_nh_.param("target_registry/confirmation_distinct_uavs", distinct_uavs_, 2);
    private_nh_.param("target_registry/confirmation_minimum_span_sec", confirmation_minimum_span_, 0.25);
    private_nh_.param("target_registry/association_covariance_sigma", association_covariance_sigma_, 3.0);
    private_nh_.param("target_registry/maximum_association_radius_m", maximum_association_radius_, 50.0);
    private_nh_.param("target_registry/cross_class_association_radius_m", cross_class_association_radius_, 3.0);
    private_nh_.param("target_registry/stable_track_maximum_jump_m", stable_track_maximum_jump_, 20.0);
    private_nh_.param("target_registry/class_confirmation_minimum_observations", class_minimum_observations_, 3);
    private_nh_.param("target_registry/class_confirmation_minimum_ratio", class_minimum_ratio_, 0.65);
    private_nh_.param("target_registry/maximum_confirmation_position_spread_m", maximum_confirmation_spread_, 3.0);
    private_nh_.param("target_registry/position_history_size", position_history_size_, 30);
    private_nh_.param("target_registry/duplicate_bbox_iou_threshold", duplicate_bbox_iou_threshold_, 0.50);
    private_nh_.param("allocation/task_deduplication_radius_m", duplicate_radius_, 5.0);
    private_nh_.param("allocation/worker_state_timeout_sec", worker_timeout_, 1.0);
    private_nh_.param("search_allocation/hover_nominal_speed_mps", hover_speed_, 4.0);
    private_nh_.param("search_allocation/fixedwing_nominal_speed_mps", fixedwing_speed_, 15.0);
    private_nh_.param<std::string>("planner/backend", default_backend_, "planning");
    if(default_backend_=="ego_swarm")default_backend_="planning";
    private_nh_.param("planner/direct_controller_test/goal_tolerance_m", direct_goal_tolerance_, 1.0);
    private_nh_.param("planner/direct_controller_test/arrival_dwell_sec", direct_arrival_dwell_, 0.5);
    private_nh_.param<std::string>("hierarchical_search/mode", hierarchical_mode_, "");
    private_nh_.param("hierarchical_search/enabled", legacy_hierarchical_enabled_, false);
    private_nh_.param<std::string>("hierarchical_search/verification/dispatch_policy", verification_dispatch_policy_, "immediate");
    private_nh_.param("hierarchical_search/verification/altitude_m", verification_altitude_, 4.0);
    private_nh_.param("hierarchical_search/verification/lane_spacing_m", verification_lane_spacing_, 5.0);
    private_nh_.param("hierarchical_search/verification/minimum_radius_m", verification_minimum_radius_, 20.0);
    private_nh_.param("hierarchical_search/verification/maximum_radius_m", verification_maximum_radius_, 80.0);
    private_nh_.param("hierarchical_search/verification/covariance_sigma", verification_covariance_sigma_, 3.0);
    private_nh_.param<std::string>("hierarchical_search/verification/radius_mode", verification_radius_mode_, "covariance");
    private_nh_.param("hierarchical_search/verification/fixed_radius_m", verification_fixed_radius_, 30.0);
    private_nh_.param("hierarchical_search/verification/maximum_concurrent_regions", verification_max_concurrent_, 1);
    private_nh_.param("hierarchical_search/verification/reject_after_route", reject_after_verification_route_, true);
    private_nh_.param("search_area/maximum_transformed_altitude_variation_m", area_altitude_tolerance_, 0.20);
    private_nh_.param<std::string>("post_arrival/mode",post_arrival_mode_,"arrive");
    private_nh_.param<std::string>("post_arrival/action_name_template",task_execute_action_template_,"/{worker}/task_execute/execute");
    private_nh_.param<std::string>("post_arrival/handoff_mode",task_execute_handoff_mode_,"auto");
    private_nh_.param<std::string>("post_arrival/failure_policy",task_execute_failure_policy_,"fail");
    private_nh_.param("post_arrival/server_wait_timeout_sec",task_execute_server_wait_timeout_,2.0);
    private_nh_.param("post_arrival/retry_delay_sec",task_execute_retry_delay_,5.0);
    private_nh_.param("post_arrival/control_handoff_settle_sec",task_execute_handoff_settle_,0.5);
    private_nh_.param("allocation/worker_execution/target_radius_m",worker_execution_target_radius_,10.0);
    private_nh_.getParam("post_arrival/default_task",task_execute_defaults_);
    private_nh_.getParam("post_arrival/mobility_profile_overrides",task_execute_mobility_overrides_);
    private_nh_.getParam("post_arrival/class_overrides",task_execute_class_overrides_);
    private_nh_.getParam("post_arrival/worker_overrides",task_execute_worker_overrides_);
    private_nh_.getParam("allocation/worker_execution/worker_overrides",worker_execution_overrides_);

    loadGeoreference();
    readPriorities();
    readVehicles("scouts", true);
    readVehicles("workers", false);
    resolveHierarchicalSearch();
    if(verification_dispatch_policy_!="immediate"&&verification_dispatch_policy_!="after_coarse_complete")
      throw std::runtime_error("invalid hierarchical_search/verification/dispatch_policy");
    if(verification_radius_mode_!="covariance"&&verification_radius_mode_!="fixed")
      throw std::runtime_error("invalid hierarchical_search/verification/radius_mode");
    if(verification_radius_mode_=="fixed"&&verification_fixed_radius_<=0)
      throw std::runtime_error("verification fixed_radius_m must be positive");
    verification_max_concurrent_=std::max(1,verification_max_concurrent_);
    registry_ = GlobalTargetRegistry(association_radius_, confirmation_hits_,
        confirmation_window_, stale_timeout_, distinct_uavs_, confirmation_minimum_span_,
        confirmation_profiles_, association_covariance_sigma_, maximum_association_radius_,
        cross_class_association_radius_, stable_track_maximum_jump_,
        class_minimum_observations_, class_minimum_ratio_, maximum_confirmation_spread_,
        position_history_size_, duplicate_bbox_iou_threshold_);
    if (scouts_.empty()) ROS_WARN("[task_allocate_cpp] no scouts configured");
    if (workers_.empty()) ROS_WARN("[task_allocate_cpp] no workers configured");
    if(post_arrival_mode_!="arrive"&&post_arrival_mode_!="task_execute")throw std::runtime_error("post_arrival/mode must be arrive or task_execute");
    if(post_arrival_mode_=="task_execute"){
      if(task_execute_action_template_.find("{worker}")==std::string::npos)throw std::runtime_error("post_arrival/action_name_template must contain {worker}");
      if(task_execute_handoff_mode_!="auto"&&task_execute_handoff_mode_!="visual"&&task_execute_handoff_mode_!="position")throw std::runtime_error("post_arrival/handoff_mode is invalid");
      if(task_execute_failure_policy_!="fail"&&task_execute_failure_policy_!="retry")throw std::runtime_error("post_arrival/failure_policy must be fail or retry");
      for(const auto& name:workers_){std::string action_name=task_execute_action_template_;auto pos=action_name.find("{worker}");action_name.replace(pos,8,name);task_execute_clients_[name]=std::make_shared<TaskExecuteClient>(action_name,true);}
    }
    bool direct_backend = false;
    for (const auto& item : vehicles_)
      direct_backend = direct_backend || item.second.backend == "direct_controller_test";
    if (direct_backend && !private_nh_.param("planner/direct_controller_test/acknowledge_no_obstacle_avoidance", false))
      throw std::runtime_error("direct_controller_test requires explicit acknowledge_no_obstacle_avoidance=true");

    std::string area_topic, targets_topic, tasks_topic, mission_topic;
    private_nh_.param<std::string>("interfaces/input/search_areas", area_topic,
                                   "/task_allocate/search_areas");
    private_nh_.param<std::string>("interfaces/output/global_targets", targets_topic,
                                   "/task_allocate/global_targets");
    private_nh_.param<std::string>("interfaces/output/rescue_tasks", tasks_topic,
                                   "/task_allocate/rescue_tasks");
    private_nh_.param<std::string>("interfaces/output/mission_state", mission_topic,
                                   "/task_allocate/mission_state");
    areas_sub_ = nh_.subscribe(area_topic, 2, &TaskAllocateCoordinator::areasCallback, this);
    targets_pub_ = nh_.advertise<GlobalTargetArray>(targets_topic, 2, true);
    tasks_pub_ = nh_.advertise<RescueTaskArray>(tasks_topic, 2, true);
    mission_pub_ = nh_.advertise<std_msgs::String>(mission_topic, 2, true);
    verification_areas_pub_ = nh_.advertise<SearchAreaArray>(
        topicParam("interfaces/output/verification_areas", "/task_allocate/verification_areas"), 2, true);
    verification_markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
        topicParam("interfaces/output/verification_markers", "/task_allocate/verification_markers"), 2, true);

    advertiseServices();
    double hz = std::max(1.0, update_rate_);
    timer_ = nh_.createTimer(ros::Duration(1.0 / hz), &TaskAllocateCoordinator::timerCallback, this);
    publishState();
    publishMissionState();
    ROS_INFO("[task_allocate_cpp] ready: scouts=%zu workers=%zu frame=%s",
             scouts_.size(), workers_.size(), shared_frame_.c_str());
  }

 private:
  struct ActiveGoal {
    std::uint32_t id{0};
    std::string kind;
    std::uint32_t object_id{0};
    Vec3 goal{};
    double entered_tolerance_at{-1.0};
  };

  struct Vehicle {
    std::string name;
    bool scout{false};
    bool enabled{true};
    std::string mobility{"hover"};
    std::string backend{"planning"};
    std::string geodetic_topic;
    double geodetic_minimum_confidence{0.0};
    XmlRpc::XmlRpcValue config;
    PoseHistory poses;
    bool has_position{false};
    Vec3 position{};
    double position_time{0.0};
    double speed{0.0};
    bool has_health{false};
    bool healthy{false};
    double health_time{0.0};
    std::vector<Vec3> route;
    std::size_t route_index{0};
    std::optional<ActiveGoal> active;
    ros::Publisher planner_goal_pub;
    ros::Publisher planner_path_pub;
    ros::Publisher route_preview_pub;
    ros::Publisher controller_goal_pub;
    ros::Publisher controller_path_pub;
    ros::Publisher direct_status_pub;
    ros::ServiceClient cancel_client;
  };

  using TaskExecuteClient=actionlib::SimpleActionClient<xd_uav_task_execute::ExecuteTaskAction>;

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::recursive_mutex mutex_;
  std::string shared_frame_{"world"};
  std::string mission_state_{kIdle};
  std::string mission_detail_{"waiting for search areas"};
  double pose_sync_tolerance_{0.15}, tf_timeout_{0.05};
  double odometry_timeout_{0.5}, health_timeout_{0.5};
  double association_radius_{4.0}, confirmation_window_{2.0}, stale_timeout_{15.0};
  double confirmation_minimum_span_{0.25},association_covariance_sigma_{3.0};
  double maximum_association_radius_{50.0},cross_class_association_radius_{3.0};
  double stable_track_maximum_jump_{20.0},class_minimum_ratio_{0.65};
  double maximum_confirmation_spread_{3.0},duplicate_bbox_iou_threshold_{0.50};
  int class_minimum_observations_{3},position_history_size_{30};
  double duplicate_radius_{5.0}, worker_timeout_{1.0};
  double hover_speed_{4.0}, fixedwing_speed_{15.0}, update_rate_{5.0};
  double direct_goal_tolerance_{1.0}, direct_arrival_dwell_{0.5};
  double verification_altitude_{4.0},verification_lane_spacing_{5.0};
  double verification_minimum_radius_{20.0},verification_maximum_radius_{80.0};
  double verification_covariance_sigma_{3.0},verification_fixed_radius_{30.0};
  double area_altitude_tolerance_{0.20};
  double task_execute_server_wait_timeout_{2.0},task_execute_retry_delay_{5.0};
  double task_execute_handoff_settle_{0.5},worker_execution_target_radius_{10.0};
  int confirmation_hits_{5}, distinct_uavs_{2}, minimum_ready_scouts_{1};
  int next_goal_id_{1};
  bool require_state_valid_{true}, require_localization_valid_{true};
  bool require_odometry_fresh_{true}, require_all_scouts_ready_{true};
  std::string default_backend_{"planning"};
  std::string hierarchical_mode_;
  bool legacy_hierarchical_enabled_{false};
  bool hierarchical_search_enabled_{false};
  bool reject_after_verification_route_{true};
  std::string verification_dispatch_policy_{"immediate"},verification_radius_mode_{"covariance"};
  std::string post_arrival_mode_{"arrive"},task_execute_action_template_{"/{worker}/task_execute/execute"};
  std::string task_execute_handoff_mode_{"auto"},task_execute_failure_policy_{"fail"};
  XmlRpc::XmlRpcValue task_execute_defaults_,task_execute_mobility_overrides_,task_execute_worker_overrides_,task_execute_class_overrides_,worker_execution_overrides_;
  int verification_max_concurrent_{1};
  double search_completed_at_{0.0},completion_grace_sec_{2.0};
  std::set<std::string> confirmation_profiles_{"hover","fixedwing"};
  bool have_georeference_{false};
  Vec3 geodetic_datum_{};
  Vec3 geodetic_world_position_{};
  double geodetic_yaw_{0.0};
  std::map<std::string, int> class_priorities_;
  std::map<std::string, Vehicle> vehicles_;
  std::vector<std::string> scouts_, workers_;
  std::vector<PlannedArea> loaded_areas_;
  std::vector<std::uint32_t> verification_pending_;
  std::map<std::uint32_t,PlannedArea> verification_plans_;
  std::map<std::uint32_t,std::set<std::uint32_t>> verification_targets_by_region_;
  std::map<std::uint32_t,std::uint32_t> verification_region_by_target_;
  std::map<std::uint32_t,std::string> verification_scout_by_region_;
  std::map<std::string,std::uint32_t> verification_region_by_scout_;
  std::map<std::uint32_t,std::string> verification_region_states_;
  std::map<std::string,std::shared_ptr<TaskExecuteClient>> task_execute_clients_;
  std::map<std::string,std::pair<std::uint32_t,std::uint32_t>> active_task_executions_;
  std::map<std::string,std::uint32_t> worker_visual_waiting_;
  std::map<std::string,double> worker_retry_after_;
  std::uint32_t next_action_token_{1};
  ros::Subscriber areas_sub_;
  ros::Publisher targets_pub_, tasks_pub_, mission_pub_;
  ros::Publisher verification_areas_pub_;
  ros::Publisher verification_markers_pub_;
  ros::Timer timer_;
  std::vector<ros::Subscriber> subscribers_;
  std::vector<ros::ServiceServer> services_;
  GlobalTargetRegistry registry_;
  RescueTaskAllocator allocator_;

  std::string topicParam(const std::string& key, const std::string& fallback) {
    std::string value; private_nh_.param(key, value, fallback); return value;
  }

  void loadGeoreference() {
    XmlRpc::XmlRpcValue georef;
    if(!private_nh_.getParam("mission/georeference",georef))return;
    const auto datum=child(georef,"datum");
    const auto lat=child(datum,"latitude"),lon=child(datum,"longitude"),alt=child(datum,"altitude");
    if(lat.getType()==XmlRpc::XmlRpcValue::TypeInvalid||lon.getType()==XmlRpc::XmlRpcValue::TypeInvalid||alt.getType()==XmlRpc::XmlRpcValue::TypeInvalid)return;
    geodetic_datum_={{xmlDouble(lat,0),xmlDouble(lon,0),xmlDouble(alt,0)}};
    const auto position=child(georef,"position");
    if(position.getType()==XmlRpc::XmlRpcValue::TypeArray&&position.size()==3)
      geodetic_world_position_={{xmlDouble(position[0],0),xmlDouble(position[1],0),xmlDouble(position[2],0)}};
    geodetic_yaw_=xmlDouble(child(georef,"enu_to_world_yaw"),0.0);
    have_georeference_=true;
  }

  void readPriorities() {
    XmlRpc::XmlRpcValue values;
    if (!private_nh_.getParam("allocation/class_priorities", values) ||
        values.getType() != XmlRpc::XmlRpcValue::TypeStruct) return;
    for (auto it=values.begin();it!=values.end();++it)
      class_priorities_[it->first] = static_cast<int>(xmlDouble(it->second, 0.0));
  }

  void resolveHierarchicalSearch() {
    std::set<std::string> mobility;
    for(const auto& name:scouts_)mobility.insert(vehicles_.at(name).mobility);
    auto mode=hierarchical_mode_;
    std::transform(mode.begin(),mode.end(),mode.begin(),[](unsigned char c){return std::tolower(c);});
    if(mode.empty())hierarchical_search_enabled_=legacy_hierarchical_enabled_;
    else if(mode=="auto")hierarchical_search_enabled_=mobility.count("fixedwing")&&mobility.count("hover");
    else if(mode=="hierarchical")hierarchical_search_enabled_=true;
    else if(mode=="single_stage")hierarchical_search_enabled_=false;
    else throw std::runtime_error("hierarchical_search/mode must be auto, hierarchical, or single_stage");
    if(hierarchical_search_enabled_)confirmation_profiles_={"hover"};
  }

  void readVehicles(const std::string& parameter, bool scout) {
    XmlRpc::XmlRpcValue configs;
    if (!private_nh_.getParam(parameter, configs) || configs.getType() != XmlRpc::XmlRpcValue::TypeStruct) return;
    for (auto it=configs.begin();it!=configs.end();++it) {
      Vehicle vehicle;
      vehicle.name = it->first;
      vehicle.scout = scout;
      vehicle.config = it->second;
      vehicle.mobility = xmlString(child(vehicle.config, "mobility_profile"), "hover");
      if (vehicle.mobility != "fixedwing") vehicle.mobility = "hover";
      vehicle.backend = xmlString(child(child(vehicle.config,"execution"),"backend"), default_backend_);
      if (vehicle.backend == "ego_swarm") vehicle.backend = "planning";
      if(vehicle.backend!="planning"&&vehicle.backend!="direct_controller_test")
        throw std::runtime_error("vehicle execution/backend must be planning or direct_controller_test");
      vehicle.enabled = true;
      setupVehicle(vehicle);
      if (scout) scouts_.push_back(vehicle.name); else {
        workers_.push_back(vehicle.name);
        allocator_.registerWorker(vehicle.name);
      }
      vehicles_.emplace(vehicle.name, std::move(vehicle));
    }
  }

  std::string vehicleTopic(const Vehicle& v, const std::string& key,
                           const std::string& fallback) const {
    return nestedString(v.config, "topics", key, fallback);
  }

  void setupVehicle(Vehicle& v) {
    const auto& name=v.name;
    const bool direct=v.backend=="direct_controller_test";
    if (direct) {
      v.controller_goal_pub=nh_.advertise<mavros_msgs::PositionTarget>(
          vehicleTopic(v,"controller_setpoint","/"+name+"/control/reference/setpoint"),2);
      v.direct_status_pub=nh_.advertise<PlannerStatus>(
          vehicleTopic(v,"planner_status","/"+name+"/planning/status"),5,true);
      if (v.mobility=="fixedwing")
        v.controller_path_pub=nh_.advertise<nav_msgs::Path>(
            vehicleTopic(v,"controller_path","/"+name+"/control/reference/path"),1,false);
    } else {
      v.planner_goal_pub=nh_.advertise<geometry_msgs::PoseStamped>(
          vehicleTopic(v,"planner_goal","/"+name+"/planning/goal"),2,true);
      v.planner_path_pub=nh_.advertise<nav_msgs::Path>(
          vehicleTopic(v,"planner_task_path","/"+name+"/planning/task_path"),1,true);
      v.cancel_client=nh_.serviceClient<CancelPlanning>(
          vehicleTopic(v,"planner_cancel","/"+name+"/planning/cancel"));
      subscribers_.push_back(nh_.subscribe<PlannerStatus>(
          vehicleTopic(v,"planner_status","/"+name+"/planning/status"),10,
          boost::bind(&TaskAllocateCoordinator::plannerStatusCallback,this,name,boost::placeholders::_1)));
    }
    v.route_preview_pub=nh_.advertise<nav_msgs::Path>(
        vehicleTopic(v,"route_preview","/"+name+"/planning/route_preview"),1,true);
    const auto odom_topic=nestedString(v.config,"localization","world_odometry_topic",
        "/"+name+"/state_estimator/main/frames/"+shared_frame_+"/odom");
    const auto health_topic=nestedString(v.config,"health","control_state_topic",
        "/"+name+"/control_manager/state");
    subscribers_.push_back(nh_.subscribe<nav_msgs::Odometry>(odom_topic,50,
        boost::bind(&TaskAllocateCoordinator::odometryCallback,this,name,boost::placeholders::_1)));
    subscribers_.push_back(nh_.subscribe<xd_uav_controller::ControlState>(health_topic,50,
        boost::bind(&TaskAllocateCoordinator::healthCallback,this,name,boost::placeholders::_1)));
    if (v.scout) {
      XmlRpc::XmlRpcValue geo=child(v.config,"geodetic_target");
      const auto geo_topic=xmlString(child(geo,"topic"),"");
      if (!geo_topic.empty()) {
        if(!have_georeference_)throw std::runtime_error("mission/georeference datum is required for geodetic_target input");
        v.geodetic_topic=geo_topic;
        v.geodetic_minimum_confidence=xmlDouble(child(geo,"minimum_confidence"),0.0);
        if(v.geodetic_minimum_confidence<0||v.geodetic_minimum_confidence>1)
          throw std::runtime_error("geodetic_target minimum_confidence must be in [0,1]");
        subscribers_.push_back(nh_.subscribe<target_geolocation::GeolocatedTarget>(
            v.geodetic_topic,10,boost::bind(&TaskAllocateCoordinator::geodeticTargetCallback,
            this,name,boost::placeholders::_1)));
      }
      XmlRpc::XmlRpcValue topics=child(v.config,"topics");
      XmlRpc::XmlRpcValue det=child(topics,"detections");
      std::vector<std::string> detection_topics;
      if (det.getType()==XmlRpc::XmlRpcValue::TypeArray) {
        for (int i=0;i<det.size();++i) detection_topics.push_back(static_cast<std::string>(det[i]));
      } else if (det.getType()==XmlRpc::XmlRpcValue::TypeString) {
        detection_topics.push_back(static_cast<std::string>(det));
      }
      if (detection_topics.empty()) detection_topics.push_back("/"+name+"/track/detections");
      for (const auto& topic:detection_topics)
        subscribers_.push_back(nh_.subscribe<xd_uav_track::DetectionArray>(topic,10,
            boost::bind(&TaskAllocateCoordinator::detectionCallback,this,name,boost::placeholders::_1)));
    } else if(post_arrival_mode_=="task_execute"&&workerVisualSubscriptionEnabled()) {
      XmlRpc::XmlRpcValue det=child(child(v.config,"topics"),"detections");
      std::vector<std::string> detection_topics;
      if(det.getType()==XmlRpc::XmlRpcValue::TypeArray){for(int i=0;i<det.size();++i)detection_topics.push_back(static_cast<std::string>(det[i]));}
      else if(det.getType()==XmlRpc::XmlRpcValue::TypeString)detection_topics.push_back(static_cast<std::string>(det));
      if(detection_topics.empty())detection_topics.push_back("/"+name+"/track/detections");
      for(const auto& topic:detection_topics)subscribers_.push_back(nh_.subscribe<xd_uav_track::DetectionArray>(topic,10,
        boost::bind(&TaskAllocateCoordinator::workerDetectionCallback,this,name,boost::placeholders::_1)));
    }
    if (direct && v.mobility=="fixedwing") {
      subscribers_.push_back(nh_.subscribe<xd_uav_controller::PathStatus>(
          vehicleTopic(v,"controller_path_status","/"+name+"/controller/path_status"),10,
          boost::bind(&TaskAllocateCoordinator::pathStatusCallback,this,name,boost::placeholders::_1)));
    }
  }

  bool workerVisualSubscriptionEnabled() const {
    if(post_arrival_mode_!="task_execute")return false;
    if(task_execute_handoff_mode_!="position")return true;
    return overridesMayNeedVisual(task_execute_defaults_)||overridesMayNeedVisual(task_execute_mobility_overrides_)||
      overridesMayNeedVisual(task_execute_worker_overrides_)||overridesMayNeedVisual(task_execute_class_overrides_);
  }

  bool overridesMayNeedVisual(const XmlRpc::XmlRpcValue& values) const {
    if(values.getType()==XmlRpc::XmlRpcValue::TypeStruct){
      if(values.hasMember("handoff_mode"))return xmlString(values["handoff_mode"],"position")!="position";
      for(auto it=values.begin();it!=values.end();++it)if(overridesMayNeedVisual(it->second))return true;
    }
    return false;
  }

  void advertiseServices() {
    XmlRpc::XmlRpcValue names; private_nh_.getParam("interfaces/services",names);
    auto srv=[&](const std::string& key){return xmlString(child(names,key),"/task_allocate/"+key);};
    services_.push_back(nh_.advertiseService(srv("start"),&TaskAllocateCoordinator::startCallback,this));
    services_.push_back(nh_.advertiseService(srv("pause"),&TaskAllocateCoordinator::pauseCallback,this));
    services_.push_back(nh_.advertiseService(srv("resume"),&TaskAllocateCoordinator::resumeCallback,this));
    services_.push_back(nh_.advertiseService(srv("stop"),&TaskAllocateCoordinator::stopCallback,this));
    services_.push_back(nh_.advertiseService(srv("reset"),&TaskAllocateCoordinator::resetCallback,this));
    services_.push_back(nh_.advertiseService(srv("clear_all"),&TaskAllocateCoordinator::clearAllCallback,this));
    services_.push_back(nh_.advertiseService(srv("restart"),&TaskAllocateCoordinator::restartCallback,this));
    services_.push_back(nh_.advertiseService(srv("replan"),&TaskAllocateCoordinator::replanCallback,this));
    services_.push_back(nh_.advertiseService(srv("clear_results"),&TaskAllocateCoordinator::clearResultsCallback,this));
    services_.push_back(nh_.advertiseService(srv("cancel_all_tasks"),&TaskAllocateCoordinator::cancelAllTasksCallback,this));
    services_.push_back(nh_.advertiseService(srv("load_search_areas"),&TaskAllocateCoordinator::loadAreasService,this));
    services_.push_back(nh_.advertiseService(srv("get_state"),&TaskAllocateCoordinator::getStateCallback,this));
    services_.push_back(nh_.advertiseService(srv("skip_waypoint"),&TaskAllocateCoordinator::skipWaypointCallback,this));
    services_.push_back(nh_.advertiseService(srv("set_vehicle_enabled"),&TaskAllocateCoordinator::setVehicleEnabledCallback,this));
    services_.push_back(nh_.advertiseService(srv("cancel_task"),&TaskAllocateCoordinator::cancelTaskCallback,this));
    services_.push_back(nh_.advertiseService(srv("retry_task"),&TaskAllocateCoordinator::retryTaskCallback,this));
    services_.push_back(nh_.advertiseService(srv("reject_target"),&TaskAllocateCoordinator::rejectTargetCallback,this));
    services_.push_back(nh_.advertiseService(srv("retry_verification"),&TaskAllocateCoordinator::retryVerificationCallback,this));
  }

  bool ready(const Vehicle& v, double now) const {
    if (!v.enabled || !v.has_position || !v.has_health || !v.healthy) return false;
    if (now-v.position_time<0 || now-v.position_time>odometry_timeout_) return false;
    if (now-v.health_time<0 || now-v.health_time>health_timeout_) return false;
    if (v.backend=="direct_controller_test" && v.controller_goal_pub.getNumSubscribers()==0) return false;
    if (v.backend=="direct_controller_test" && v.mobility=="fixedwing" && v.controller_path_pub.getNumSubscribers()==0) return false;
    auto retry=worker_retry_after_.find(v.name);
    if(!v.scout&&retry!=worker_retry_after_.end()&&now<retry->second)return false;
    return true;
  }

  RigidTransform worldTransform(const std::string& source_frame, const ros::Time& stamp) {
    const auto source=stripSlash(source_frame);
    if (source.empty()) throw std::runtime_error("source frame is empty");
    if (source==shared_frame_) return {};
    try {
      const auto tf=tf_buffer_.lookupTransform(shared_frame_,source,stamp,ros::Duration(std::max(0.0,tf_timeout_)));
      RigidTransform out;
      out.translation={{tf.transform.translation.x,tf.transform.translation.y,tf.transform.translation.z}};
      out.rotation={{tf.transform.rotation.x,tf.transform.rotation.y,tf.transform.rotation.z,tf.transform.rotation.w}};
      return out;
    } catch (const tf2::TransformException& e) {
      throw std::runtime_error(e.what());
    }
  }

  void odometryCallback(const std::string& name, const nav_msgs::OdometryConstPtr& msg) {
    if (msg->header.stamp.isZero()) return;
    const auto frame=stripSlash(msg->header.frame_id);
    if (frame!=shared_frame_) return;
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    auto it=vehicles_.find(name); if (it==vehicles_.end()) return;
    auto& v=it->second;
    PoseSample sample;
    sample.stamp=msg->header.stamp.toSec(); sample.frame_id=frame;
    sample.position={{msg->pose.pose.position.x,msg->pose.pose.position.y,msg->pose.pose.position.z}};
    sample.orientation={{msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,
                         msg->pose.pose.orientation.z,msg->pose.pose.orientation.w}};
    try { rotateVector(sample.orientation,{{0,0,0}}); } catch (...) { return; }
    v.poses.add(sample); v.position=sample.position; v.has_position=true;
    v.position_time=ros::Time::now().toSec();
    const auto& velocity=msg->twist.twist.linear;
    v.speed=std::sqrt(velocity.x*velocity.x+velocity.y*velocity.y+velocity.z*velocity.z);
    if (!v.scout) {
      allocator_.updateWorker(name,v.position,v.position_time,ready(v,v.position_time));
      dispatchAssignments();
    }
  }

  void healthCallback(const std::string& name,
                      const xd_uav_controller::ControlStateConstPtr& msg) {
    if (msg->header.stamp.isZero()) return;
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    auto it=vehicles_.find(name); if (it==vehicles_.end()) return;
    auto& v=it->second;
    v.has_health=true; v.health_time=ros::Time::now().toSec();
    v.healthy=(msg->state_valid||!require_state_valid_) &&
      (msg->localization_valid||!require_localization_valid_) &&
      (msg->odometry_fresh||!require_odometry_fresh_);
    if (!v.scout && v.has_position)
      allocator_.updateWorker(name,v.position,v.position_time,ready(v,v.health_time));
    dispatchAssignments();
  }

  void detectionCallback(const std::string& scout,
                         const xd_uav_track::DetectionArrayConstPtr& msg) {
    if (msg->header.stamp.isZero()) return;
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (mission_state_!=kActive) return;
    auto v=vehicles_.find(scout); if (v==vehicles_.end()) return;
    auto pose=v->second.poses.closest(msg->header.stamp.toSec(),pose_sync_tolerance_);
    if (!pose) return;
    bool changed=false;
    for (const auto& candidate:msg->candidates) {
      if (!candidate.range_valid || !candidate.has_relative_position_body) continue;
      TargetObservation observation;
      observation.source_uav=scout; observation.sensor_id=msg->sensor_id;
      observation.stamp=msg->header.stamp.toSec(); observation.class_id=candidate.class_id;
      observation.confidence=candidate.confidence; observation.track_id=candidate.track_id;
      observation.track_id_is_stable=candidate.track_id_is_stable;
      observation.mobility_profile=v->second.mobility;
      const Vec3 frd{{candidate.relative_position_body[0],candidate.relative_position_body[1],candidate.relative_position_body[2]}};
      try { observation.position=relativeFrdToShared(frd,*pose); } catch (...) { continue; }
      Covariance3 covariance{};
      for (int i=0;i<9;++i) covariance[i]=candidate.position_covariance[i];
      observation.covariance=rotateCovarianceFrdToShared(covariance,pose->orientation);
      if(candidate.has_normalized_bbox){
        observation.normalized_bbox=std::array<double,4>{{candidate.normalized_bbox[0],candidate.normalized_bbox[1],candidate.normalized_bbox[2],candidate.normalized_bbox[3]}};
      }else if(candidate.has_bbox&&msg->image_width>0&&msg->image_height>0){
        observation.normalized_bbox=std::array<double,4>{{
          0.5*(candidate.bbox[0]+candidate.bbox[2])/msg->image_width,
          0.5*(candidate.bbox[1]+candidate.bbox[3])/msg->image_height,
          static_cast<double>(candidate.bbox[2]-candidate.bbox[0])/msg->image_width,
          static_cast<double>(candidate.bbox[3]-candidate.bbox[1])/msg->image_height}};
      }
      recordObservation(scout,observation,changed);
    }
    if (changed) { dispatchAssignments(); publishState(); }
  }

  void recordObservation(const std::string& scout,const TargetObservation& observation,bool& changed) {
    const auto update=registry_.observe(observation);
    changed=true;
    if(hierarchical_search_enabled_&&vehicles_.at(scout).mobility=="hover"){
      const auto region=containingVerificationRegion(update.target.position);
      if(region)attachTargetToVerificationRegion(update.target,*region);
    }
    if(update.newly_confirmed){
      const auto key=std::to_string(update.target.class_id);
      const int priority=class_priorities_.count(key)?class_priorities_.at(key):0;
      const auto& task=allocator_.ensureTask(update.target,priority,duplicate_radius_);
      if(task.target_id!=update.target.target_id)
        ROS_WARN_STREAM("[task_allocate_cpp] duplicate target "<<update.target.target_id<<" suppressed by task "<<task.task_id);
    }else if(hierarchical_search_enabled_&&vehicles_.at(scout).mobility=="fixedwing"&&update.newly_evidence_ready){
      queueVerification(update.target);
    }
  }

  std::optional<std::uint32_t> containingVerificationRegion(const Vec3& point) const {
    std::optional<std::uint32_t> best;
    for(const auto& entry:verification_plans_){
      if(!pointInsidePlan(point,entry.second))continue;
      if(!best){best=entry.first;continue;}
      const auto state=verification_region_states_.find(entry.first);
      const auto best_state=verification_region_states_.find(*best);
      const bool finished=state!=verification_region_states_.end()&&
          (state->second=="completed"||state->second=="missed");
      const bool best_finished=best_state!=verification_region_states_.end()&&
          (best_state->second=="completed"||best_state->second=="missed");
      const double length=entry.second.length;
      const double best_length=verification_plans_.at(*best).length;
      if((best_finished&&!finished)||(best_finished==finished&&
          (length<best_length||(length==best_length&&entry.first<*best))))best=entry.first;
    }
    return best;
  }

  void attachTargetToVerificationRegion(const GlobalTargetRecord& target,std::uint32_t region) {
    if(verification_region_by_target_.count(target.target_id))return;
    auto plan=verification_plans_.find(region);
    if(plan==verification_plans_.end())return;
    verification_region_by_target_[target.target_id]=region;
    verification_targets_by_region_[region].insert(target.target_id);
  }

  bool pointInsidePlan(const Vec3& point,const PlannedArea& plan) const {
    if(plan.area.boundary.empty())return false;
    double minx=plan.area.boundary.front()[0],maxx=minx,miny=plan.area.boundary.front()[1],maxy=miny;
    for(const auto& p:plan.area.boundary){minx=std::min(minx,p[0]);maxx=std::max(maxx,p[0]);miny=std::min(miny,p[1]);maxy=std::max(maxy,p[1]);}
    return point[0]>=minx&&point[0]<=maxx&&point[1]>=miny&&point[1]<=maxy;
  }

  PlannedArea buildVerificationPlan(const GlobalTargetRecord& target) const {
    const double xx=target.covariance[0],xy=0.5*(target.covariance[1]+target.covariance[3]),yy=target.covariance[4];
    double variance=0.0;
    if(std::isfinite(xx)&&std::isfinite(xy)&&std::isfinite(yy))
      variance=std::max(0.0,0.5*(xx+yy+std::sqrt(std::max(0.0,(xx-yy)*(xx-yy)+4*xy*xy))));
    const double minimum=std::max(0.1,verification_minimum_radius_);
    const double maximum=std::max(minimum,verification_maximum_radius_);
    const double radius=verification_radius_mode_=="fixed"?verification_fixed_radius_:
      std::min(maximum,std::max(minimum,std::max(0.0,verification_covariance_sigma_)*std::sqrt(variance)));
    SearchAreaDefinition area;area.area_id=target.target_id;area.altitude=verification_altitude_;
    area.lane_spacing=std::max(0.1,verification_lane_spacing_);area.priority=255;
    const double x=target.position[0],y=target.position[1];
    area.boundary={{{x-radius,y-radius}},{{x+radius,y-radius}},{{x+radius,y+radius}},{{x-radius,y+radius}}};
    PlannedArea planned;planned.area=area;planned.path=lawnmowerPath(area);planned.length=pathLength(planned.path);return planned;
  }

  void publishVerificationAreas() {
    SearchAreaArray areas;areas.header.stamp=ros::Time::now();areas.header.frame_id=shared_frame_;
    visualization_msgs::MarkerArray markers;visualization_msgs::Marker clear;clear.action=visualization_msgs::Marker::DELETEALL;markers.markers.push_back(clear);
    for(const auto& entry:verification_plans_){const auto id=entry.first;const auto& plan=entry.second;
      SearchArea area;area.area_id=id;area.altitude=plan.area.altitude;area.lane_spacing=plan.area.lane_spacing;area.priority=plan.area.priority;
      for(const auto& p:plan.area.boundary){geometry_msgs::Point32 q;q.x=p[0];q.y=p[1];q.z=plan.area.altitude;area.boundary.points.push_back(q);}areas.areas.push_back(area);
      visualization_msgs::Marker marker;marker.header=areas.header;marker.ns="verification_areas";marker.id=id;marker.type=visualization_msgs::Marker::LINE_STRIP;marker.action=visualization_msgs::Marker::ADD;marker.pose.orientation.w=1;marker.scale.x=0.8;marker.color.a=0.9;
      const auto state=verification_region_states_[id];if(state=="active"){marker.color.b=1;marker.color.r=0.1;marker.color.g=0.45;}else if(state=="completed"){marker.color.r=0.1;marker.color.g=0.9;}else if(state=="missed"){marker.color.r=1;marker.color.g=0.15;}else{marker.color.r=1;marker.color.g=0.75;}
      for(const auto& p:plan.area.boundary){geometry_msgs::Point q;q.x=p[0];q.y=p[1];q.z=plan.area.altitude;marker.points.push_back(q);}if(!marker.points.empty())marker.points.push_back(marker.points.front());markers.markers.push_back(marker);
    }
    verification_areas_pub_.publish(areas);verification_markers_pub_.publish(markers);
  }

  void queueVerification(const GlobalTargetRecord& target) {
    const auto target_id=target.target_id;
    if(verification_region_by_target_.count(target_id))return;
    PlannedArea planned;
    try{planned=buildVerificationPlan(target);}catch(const std::exception&){registry_.setStatus(target_id,TARGET_STALE);return;}
    const auto region=containingVerificationRegion(target.position);
    if(region){
      verification_region_by_target_[target_id]=*region;verification_targets_by_region_[*region].insert(target_id);
      if(verification_region_states_[*region]=="completed"||verification_region_states_[*region]=="missed")registry_.setStatus(target_id,TARGET_STALE);
      else registry_.setStatus(target_id,TARGET_VERIFYING);
      publishVerificationAreas();return;
    }
    const auto region_id=target_id;verification_plans_[region_id]=std::move(planned);
    verification_region_states_[region_id]="pending";verification_region_by_target_[target_id]=region_id;
    verification_targets_by_region_[region_id].insert(target_id);verification_pending_.push_back(region_id);
    registry_.setStatus(target_id,TARGET_VERIFYING);search_completed_at_=0.0;
    publishVerificationAreas();assignPendingVerifications();
  }

  bool coarseRoutesComplete() const {
    bool any=false;for(const auto& name:scouts_){const auto& v=vehicles_.at(name);if(v.mobility!="fixedwing")continue;any=true;if(v.route_index<v.route.size())return false;}
    return any;
  }

  bool allScoutRoutesComplete() const {
    if(scouts_.empty())return false;
    for(const auto& name:scouts_){const auto& v=vehicles_.at(name);if(v.route_index<v.route.size())return false;}
    return true;
  }

  void assignPendingVerifications() {
    if(!hierarchical_search_enabled_||mission_state_!=kActive)return;
    if(verification_dispatch_policy_=="after_coarse_complete"&&!coarseRoutesComplete())return;
    int slots=verification_max_concurrent_-static_cast<int>(verification_region_by_scout_.size());
    const double now=ros::Time::now().toSec();
    while(slots>0&&!verification_pending_.empty()){
      std::vector<std::string> available;
      for(const auto& name:scouts_){const auto& v=vehicles_.at(name);if(v.mobility=="hover"&&!verification_region_by_scout_.count(name)&&ready(v,now)&&v.route_index>=v.route.size()&&!v.active)available.push_back(name);}
      if(available.empty())break;
      const auto region=verification_pending_.front();verification_pending_.erase(verification_pending_.begin());
      auto plan_it=verification_plans_.find(region);if(plan_it==verification_plans_.end())continue;
      bool has_member=false;for(auto id:verification_targets_by_region_[region])if(registry_.targets().count(id))has_member=true;
      if(!has_member){verification_region_states_[region]="missed";continue;}
      Vec3 center{};for(const auto& p:plan_it->second.area.boundary){center[0]+=p[0];center[1]+=p[1];}center[0]/=plan_it->second.area.boundary.size();center[1]/=plan_it->second.area.boundary.size();
      auto distance_to_center=[&](const std::string& name){const auto& p=vehicles_.at(name).position;return std::hypot(p[0]-center[0],p[1]-center[1]);};
      std::sort(available.begin(),available.end(),[&](const auto& a,const auto& b){const auto da=distance_to_center(a),db=distance_to_center(b);return da<db||(da==db&&a<b);});
      const auto scout=available.front();auto& v=vehicles_.at(scout);v.route=plan_it->second.path;v.route_index=0;
      verification_region_by_scout_[scout]=region;verification_scout_by_region_[region]=scout;verification_region_states_[region]="active";
      publishRoutePreview(v);publishNextScout(v);--slots;publishVerificationAreas();
    }
  }

  void finishVerification(std::uint32_t region) {
    verification_pending_.erase(std::remove(verification_pending_.begin(),verification_pending_.end(),region),verification_pending_.end());
    auto owner=verification_scout_by_region_.find(region);
    if(owner!=verification_scout_by_region_.end()){
      auto& scout=vehicles_.at(owner->second);verification_region_by_scout_.erase(owner->second);
      if(scout.active&&scout.active->kind=="verification")scout.active.reset();
      scout.route.clear();scout.route_index=0;publishRoutePreview(scout);
      verification_scout_by_region_.erase(owner);
    }
    bool found=false;
    for(auto id:verification_targets_by_region_[region]){
      auto target=registry_.targets().find(id);if(target==registry_.targets().end())continue;
      if(target->second.status==TARGET_VERIFYING)registry_.setStatus(id,reject_after_verification_route_?TARGET_STALE:TARGET_CANDIDATE);
      const auto status=registry_.targets().at(id).status;
      if(status==TARGET_CONFIRMED||status==TARGET_ASSIGNED||status==TARGET_EXECUTING||status==TARGET_COMPLETED)found=true;
    }
    verification_region_states_[region]=found?"completed":"missed";publishVerificationAreas();assignPendingVerifications();markSearchCompleteIfReady();
  }

  void markSearchCompleteIfReady() {
    if(allScoutRoutesComplete()&&verification_pending_.empty()&&verification_region_by_scout_.empty()&&search_completed_at_<=0.0)
      search_completed_at_=ros::Time::now().toSec();
  }

  void geodeticTargetCallback(const std::string& scout,
      const target_geolocation::GeolocatedTargetConstPtr& msg) {
    const auto& fix=msg->fix;
    if(fix.header.stamp.isZero()||fix.status.status<0||msg->class_id<0||
       !std::isfinite(msg->confidence)||msg->confidence<0||msg->confidence>1||
       msg->confidence<vehicles_.at(scout).geodetic_minimum_confidence)return;
    if(!have_georeference_)return;
    TargetObservation observation;observation.source_uav=scout;
    observation.sensor_id=msg->sensor_id.empty()?"target_geolocation":msg->sensor_id;
    observation.stamp=fix.header.stamp.toSec();observation.class_id=msg->class_id;
    observation.confidence=msg->confidence;observation.track_id=msg->track_id;
    observation.track_id_is_stable=msg->track_id_is_stable;
    observation.mobility_profile=vehicles_.at(scout).mobility;
    try{observation.position=geodeticToWorld(fix.latitude,fix.longitude,fix.altitude,
        geodetic_datum_,geodetic_yaw_,geodetic_world_position_);}catch(const std::exception&){return;}
    if(fix.position_covariance_type!=sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN){
      Covariance3 covariance{};for(int i=0;i<9;++i)covariance[i]=fix.position_covariance[i];
      observation.covariance=rotateEnuCovarianceToWorld(covariance,geodetic_yaw_);
    }
    std::lock_guard<std::recursive_mutex> lock(mutex_);if(mission_state_!=kActive)return;
    bool changed=false;recordObservation(scout,observation,changed);
    if(changed){dispatchAssignments();publishState();}
  }

  XmlRpc::XmlRpcValue taskOption(const RescueTaskRecord& task,const Vehicle& worker,const std::string& key) const {
    XmlRpc::XmlRpcValue value=child(task_execute_defaults_,key);
    auto apply=[&](const XmlRpc::XmlRpcValue& group,const std::string& selector){
      const auto options=child(group,selector);const auto candidate=child(options,key);
      if(candidate.getType()!=XmlRpc::XmlRpcValue::TypeInvalid)value=candidate;
    };
    apply(task_execute_mobility_overrides_,worker.mobility);
    apply(task_execute_worker_overrides_,worker.name);
    apply(task_execute_class_overrides_,std::to_string(task.class_id));
    return value;
  }

  std::uint8_t taskExecuteType(const XmlRpc::XmlRpcValue& value) const {
    if(value.getType()==XmlRpc::XmlRpcValue::TypeString){
      const auto name=static_cast<std::string>(value);
      if(name=="arrive")return xd_uav_task_execute::ExecuteTaskGoal::ARRIVE;
      if(name=="track")return xd_uav_task_execute::ExecuteTaskGoal::TRACK;
      if(name=="observe")return xd_uav_task_execute::ExecuteTaskGoal::OBSERVE;
      if(name=="intercept")return xd_uav_task_execute::ExecuteTaskGoal::INTERCEPT;
      if(name=="direct_control")return xd_uav_task_execute::ExecuteTaskGoal::TRACK;
      throw std::invalid_argument("post_arrival task_type must be arrive, track, observe, intercept or direct_control");
    }
    const int value_int=static_cast<int>(xmlDouble(value,xd_uav_task_execute::ExecuteTaskGoal::TRACK));
    if(value_int<xd_uav_task_execute::ExecuteTaskGoal::ARRIVE||value_int>xd_uav_task_execute::ExecuteTaskGoal::INTERCEPT)
      throw std::invalid_argument("unsupported post_arrival task_type");
    return static_cast<std::uint8_t>(value_int);
  }

  std::string taskExecuteProfile(const RescueTaskRecord& task,const Vehicle& worker) const {
    auto profile=xmlString(taskOption(task,worker,"follower_profile"),"");
    if(!profile.empty())return profile;
    auto type=xmlString(taskOption(task,worker,"task_type"),"");
    std::transform(type.begin(),type.end(),type.begin(),[](unsigned char c){return std::tolower(c);});
    return type=="direct_control"?"direct_control":"";
  }

  bool taskUsesDirectControl(const RescueTaskRecord& task,const Vehicle& worker) const {
    auto profile=taskExecuteProfile(task,worker);
    std::transform(profile.begin(),profile.end(),profile.begin(),[](unsigned char c){return std::tolower(c);});
    if(post_arrival_mode_!="task_execute"||profile!="direct_control")return false;
    try{return taskExecuteType(taskOption(task,worker,"task_type"))!=xd_uav_task_execute::ExecuteTaskGoal::ARRIVE;}
    catch(const std::exception&){return false;}
  }

  std::string taskExecuteHandoff(const RescueTaskRecord& task,const Vehicle& worker) const {
    const auto configured=xmlString(taskOption(task,worker,"handoff_mode"),task_execute_handoff_mode_);
    if(configured!="auto"&&configured!="visual"&&configured!="position")throw std::invalid_argument("invalid task_execute handoff_mode");
    if(configured!="auto")return configured;
    auto profile=taskExecuteProfile(task,worker);
    std::transform(profile.begin(),profile.end(),profile.begin(),[](unsigned char c){return std::tolower(c);});
    if(profile.find("fw_metric_")==0||profile=="direct_control")return "position";
    const auto type=taskExecuteType(taskOption(task,worker,"task_type"));
    if(type==xd_uav_task_execute::ExecuteTaskGoal::OBSERVE||type==xd_uav_task_execute::ExecuteTaskGoal::INTERCEPT)return "position";
    return "visual";
  }

  xd_uav_task_execute::ExecuteTaskGoal buildTaskExecuteGoal(
      const RescueTaskRecord& task,const Vehicle& worker,int local_track_id) const {
    xd_uav_task_execute::ExecuteTaskGoal goal;
    goal.header.stamp=ros::Time::now();goal.header.frame_id=shared_frame_;
    goal.task_id=task.task_id;goal.target_id=task.target_id;goal.task_type=taskExecuteType(taskOption(task,worker,"task_type"));
    goal.worker_name=worker.name;goal.target_class_id=task.class_id;goal.target_pose.header=goal.header;
    goal.target_pose.pose.position.x=task.target_position[0];goal.target_pose.pose.position.y=task.target_position[1];goal.target_pose.pose.position.z=task.target_position[2];goal.target_pose.pose.orientation.w=1.0;
    const auto configured_track=static_cast<int>(xmlDouble(taskOption(task,worker,"local_track_id"),-1));
    goal.local_track_id=local_track_id==-2?configured_track:local_track_id;
    goal.follower_profile=taskExecuteProfile(task,worker);
    goal.required_execution_sec=std::max(0.0,xmlDouble(taskOption(task,worker,"required_execution_sec"),0.0));
    goal.maximum_duration_sec=std::max(0.0,xmlDouble(taskOption(task,worker,"maximum_duration_sec"),0.0));
    return goal;
  }

  void startTaskExecution(RescueTaskRecord& task,Vehicle& worker,int local_track_id,const std::string& detail) {
    if(allocator_.markExecuting(task.task_id))registry_.setStatus(task.target_id,TARGET_EXECUTING);
    auto client=task_execute_clients_.find(worker.name);
    if(client==task_execute_clients_.end()){handleTaskExecuteFailure(task,worker,"no task_execute action client is configured");return;}
    task.detail="waiting for task_execute action server";
    if(!client->second->waitForServer(ros::Duration(std::max(0.1,task_execute_server_wait_timeout_)))){
      handleTaskExecuteFailure(task,worker,"task_execute action server unavailable");return;
    }
    xd_uav_task_execute::ExecuteTaskGoal goal;
    try{goal=buildTaskExecuteGoal(task,worker,local_track_id);}catch(const std::exception& e){handleTaskExecuteFailure(task,worker,e.what());return;}
    const auto token=next_action_token_++;
    active_task_executions_[worker.name]={task.task_id,token};
    task.detail="task_execute goal sent";
    client->second->sendGoal(goal,
      boost::bind(&TaskAllocateCoordinator::taskExecutionDone,this,worker.name,token,boost::placeholders::_1,boost::placeholders::_2),
      boost::bind(&TaskAllocateCoordinator::taskExecutionActive,this,worker.name,token),
      boost::bind(&TaskAllocateCoordinator::taskExecutionFeedback,this,worker.name,token,boost::placeholders::_1));
    ROS_INFO_STREAM("[task_allocate_cpp] task="<<task.task_id<<" worker="<<worker.name<<" handoff="<<detail<<" task_execute started");
  }

  void handleTaskExecuteFailure(RescueTaskRecord& task,Vehicle& worker,const std::string& detail) {
    const auto message="task_execute task failed: "+detail;
    if(task_execute_failure_policy_=="retry"){
      auto released=allocator_.releaseWorker(worker.name,message);if(released)registry_.setStatus(released->target_id,TARGET_CONFIRMED);
      worker_retry_after_[worker.name]=ros::Time::now().toSec()+std::max(0.0,task_execute_retry_delay_);
    }else{
      auto failed=allocator_.cancelTask(task.task_id,message);if(failed)registry_.setStatus(failed->target_id,TARGET_CONFIRMED);
    }
    allocator_.workers()[worker.name].online=false;
    ROS_ERROR_STREAM("[task_allocate_cpp] task="<<task.task_id<<" worker="<<worker.name<<" failed: "<<detail);
  }

  void taskExecutionActive(const std::string& worker,std::uint32_t token) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);auto active=active_task_executions_.find(worker);
    if(active==active_task_executions_.end()||active->second.second!=token)return;
    auto task=allocator_.tasks().find(active->second.first);if(task!=allocator_.tasks().end())task->second.detail="task_execute action is active";
  }

  void taskExecutionFeedback(const std::string& worker,std::uint32_t token,
      const xd_uav_task_execute::ExecuteTaskFeedbackConstPtr& feedback) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);auto active=active_task_executions_.find(worker);
    if(active==active_task_executions_.end()||active->second.second!=token||!feedback)return;
    auto task=allocator_.tasks().find(active->second.first);if(task!=allocator_.tasks().end())task->second.detail="task_execute progress="+std::to_string(feedback->progress)+": "+feedback->detail;
  }

  void taskExecutionDone(const std::string& worker,std::uint32_t token,
      const actionlib::SimpleClientGoalState& state,
      const xd_uav_task_execute::ExecuteTaskResultConstPtr& result) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);auto active=active_task_executions_.find(worker);
    if(active==active_task_executions_.end()||active->second.second!=token)return;
    const auto task_id=active->second.first;active_task_executions_.erase(active);
    auto task=allocator_.tasks().find(task_id);auto vehicle=vehicles_.find(worker);
    if(task==allocator_.tasks().end()||vehicle==vehicles_.end()||task->second.assigned_worker!=worker)return;
    if(vehicle->second.backend=="direct_controller_test"&&vehicle->second.mobility!="fixedwing")publishDirectHold(vehicle->second);
    const bool success=state==actionlib::SimpleClientGoalState::SUCCEEDED&&result&&result->success;
    if(success){auto completed=allocator_.complete(task_id);if(completed)registry_.setStatus(completed->target_id,TARGET_COMPLETED);
      if(task_execute_handoff_settle_>0){worker_retry_after_[worker]=ros::Time::now().toSec()+task_execute_handoff_settle_;allocator_.workers()[worker].online=false;}}
    else {const auto message=result?result->message:"action returned no result";handleTaskExecuteFailure(task->second,vehicle->second,"action_state="+state.toString()+": "+message);}
    dispatchAssignments();checkMissionComplete();publishState();
  }

  void cancelTaskExecution(const std::string& worker,const std::string& detail) {
    auto active=active_task_executions_.find(worker);if(active==active_task_executions_.end())return;
    auto client=task_execute_clients_.find(worker);if(client!=task_execute_clients_.end())client->second->cancelGoal();
    auto vehicle=vehicles_.find(worker);
    if(vehicle!=vehicles_.end()&&vehicle->second.backend=="direct_controller_test"&&vehicle->second.mobility!="fixedwing")publishDirectHold(vehicle->second);
    auto task=allocator_.tasks().find(active->second.first);if(task!=allocator_.tasks().end())task->second.detail=detail;
    active_task_executions_.erase(active);
  }

  void cancelAllTaskExecutions(const std::string& detail,const std::string& transition) {
    std::vector<std::pair<std::string,std::uint32_t>> active;
    for(const auto& entry:active_task_executions_)active.emplace_back(entry.first,entry.second.first);
    for(const auto& entry:active){
      cancelTaskExecution(entry.first,detail);
      if(transition=="release"){
        auto task=allocator_.releaseWorker(entry.first,detail);
        if(task&&registry_.targets().count(task->target_id))registry_.setStatus(task->target_id,TARGET_CONFIRMED);
      }else if(transition=="fail"){
        auto task=allocator_.cancelTask(entry.second,detail);
        if(task&&registry_.targets().count(task->target_id))registry_.setStatus(task->target_id,TARGET_CONFIRMED);
      }
    }
  }

  void workerDetectionCallback(const std::string& worker,const xd_uav_track::DetectionArrayConstPtr& message) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if(mission_state_!=kActive||post_arrival_mode_!="task_execute"||active_task_executions_.count(worker))return;
    auto vehicle=vehicles_.find(worker);if(vehicle==vehicles_.end()||!vehicle->second.has_position)return;
    std::optional<std::uint32_t> task_id;
    auto active=vehicle->second.active;
    if(active&&active->kind=="rescue")task_id=active->object_id;
    else {auto waiting=worker_visual_waiting_.find(worker);if(waiting!=worker_visual_waiting_.end())task_id=waiting->second;}
    if(!task_id||message->image_width==0||message->image_height==0)return;
    auto task=allocator_.tasks().find(*task_id);if(task==allocator_.tasks().end()||task->second.assigned_worker!=worker)return;
    std::string handoff;try{handoff=taskExecuteHandoff(task->second,vehicle->second);}catch(...){return;}if(handoff=="position")return;
    auto worker_override=child(worker_execution_overrides_,worker);
    const double radius=std::max(0.0,xmlDouble(child(worker_override,"target_radius_m"),worker_execution_target_radius_));
    const double distance=std::hypot(vehicle->second.position[0]-task->second.target_position[0],vehicle->second.position[1]-task->second.target_position[1]);
    if(distance>radius)return;
    const xd_uav_track::DetectionCandidate* best=nullptr;
    for(const auto& candidate:message->candidates)if(candidate.class_id==task->second.class_id&&(!best||candidate.confidence>best->confidence))best=&candidate;
    if(!best)return;
    if(active&&vehicle->second.backend!="direct_controller_test"){
      std::string detail;if(!cancelVehicleGoal(vehicle->second,"visual handoff to task execution",detail)){task->second.detail="waiting for planning release: "+detail;return;}
    }else if(vehicle->second.backend=="direct_controller_test"&&vehicle->second.mobility!="fixedwing")publishDirectHold(vehicle->second);
    vehicle->second.active.reset();worker_visual_waiting_.erase(worker);
    const bool pass_track=xmlBool(child(worker_override,"pass_detection_track_id"),true);
    const int local_track=pass_track&&best->track_id_is_stable&&best->track_id>=0?best->track_id:-1;
    startTaskExecution(task->second,vehicle->second,local_track,"matching visual detection inside target radius");
    publishState();
  }

  void areasCallback(const SearchAreaArrayConstPtr& msg) {
    std::string detail; std::uint32_t count=0;
    if (!loadAreas(*msg,detail,count)) ROS_WARN_STREAM("[task_allocate_cpp] rejected areas: "<<detail);
  }

  bool loadAreas(const SearchAreaArray& msg, std::string& detail, std::uint32_t& count) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (mission_state_==kActive||mission_state_==kPaused) {detail="mission is active or paused";count=0;return false;}
    if (stripSlash(msg.header.frame_id).empty()) {detail="search-area frame_id is empty";count=0;return false;}
    RigidTransform transform;
    try { transform=worldTransform(msg.header.frame_id,msg.header.stamp); }
    catch (const std::exception& e) {detail=e.what();count=0;return false;}
    std::vector<PlannedArea> planned;
    std::ostringstream errors;
    for (const auto& input:msg.areas) {
      SearchAreaDefinition area; area.area_id=input.area_id; area.altitude=input.altitude;
      area.lane_spacing=input.lane_spacing; area.priority=input.priority;
      std::vector<double> z;
      for (const auto& p:input.boundary.points) {
        const auto w=transform.apply({{p.x,p.y,input.altitude}});
        area.boundary.push_back({{w[0],w[1]}}); z.push_back(w[2]);
      }
      if(!z.empty()&&(*std::max_element(z.begin(),z.end())-*std::min_element(z.begin(),z.end()))>area_altitude_tolerance_){errors<<"area "<<input.area_id<<": area frame is tilted relative to world; ";continue;}
      if (!z.empty()) area.altitude=std::accumulate(z.begin(),z.end(),0.0)/z.size();
      try { PlannedArea p; p.area=area; p.path=lawnmowerPath(area); p.length=pathLength(p.path); planned.push_back(std::move(p)); }
      catch (const std::exception& e) { errors<<"area "<<input.area_id<<": "<<e.what()<<"; "; }
    }
    if (planned.empty()) {detail=errors.str().empty()?"message contains no areas":errors.str();count=0;return false;}
    loaded_areas_=std::move(planned);
    for (auto& item:vehicles_) {item.second.route.clear();item.second.route_index=0;item.second.active.reset();}
    verification_pending_.clear();verification_plans_.clear();verification_targets_by_region_.clear();verification_region_by_target_.clear();verification_scout_by_region_.clear();verification_region_by_scout_.clear();verification_region_states_.clear();worker_visual_waiting_.clear();worker_retry_after_.clear();search_completed_at_=0.0;
    publishVerificationAreas();
    registry_.clear(); allocator_.clearTasks();
    mission_state_=kLoaded; mission_detail_="loaded "+std::to_string(loaded_areas_.size())+" search area(s); waiting for takeoff and start";
    detail=mission_detail_; count=loaded_areas_.size(); publishMissionState(); publishState(); return true;
  }

  bool loadAreasService(LoadSearchAreas::Request& req, LoadSearchAreas::Response& res) {
    res.success=loadAreas(req.search_areas,res.message,res.loaded_area_count);return true;
  }

  void prepareRoutes(const std::map<std::string,Vec3>& positions) {
    auto coverage_positions=positions;
    if(hierarchical_search_enabled_){
      bool has_hover=false,has_fixedwing=false;
      for(const auto& entry:positions){if(vehicles_.at(entry.first).mobility=="hover")has_hover=true;else has_fixedwing=true;}
      if(!has_fixedwing)throw std::invalid_argument("hierarchical search requires at least one ready fixed-wing scout");
      if(!has_hover)throw std::invalid_argument("hierarchical search requires at least one ready hover-profile scout");
      for(auto it=coverage_positions.begin();it!=coverage_positions.end();)if(vehicles_.at(it->first).mobility!="fixedwing")it=coverage_positions.erase(it);else ++it;
    }
    std::map<std::string,double> speeds;
    std::map<std::pair<std::string,std::uint32_t>,double> lengths;
    std::map<std::pair<std::string,std::uint32_t>,std::vector<Vec3>> fixedwing_paths;
    std::map<std::string,std::array<double,3>> fixedwing_parameters;
    for (const auto& entry:coverage_positions) {
      const auto& v=vehicles_.at(entry.first);
      const double fallback=v.mobility=="fixedwing"?fixedwing_speed_:hover_speed_;
      speeds[entry.first]=std::max(0.1,nestedDouble(v.config,"coverage","nominal_speed_mps",fallback));
      if(v.mobility!="fixedwing")continue;
      const double radius=std::max(1.0,fixedwingSetting(v,"minimum_turn_radius_m",30.0));
      const double spacing=std::max(1.0,fixedwingSetting(v,"turn_waypoint_spacing_m",15.0));
      const double lead=std::max(0.0,fixedwingSetting(v,"straight_lead_distance_m",radius));
      fixedwing_parameters[v.name]={{radius,spacing,lead}};
      for(const auto& planned:loaded_areas_){
        SearchAreaDefinition area=planned.area;
        const double lane=fixedwingSetting(v,"coverage_lane_spacing_m",0.0);
        const double altitude=fixedwingSetting(v,"coverage_altitude_m",0.0);
        if(lane>0)area.lane_spacing=lane;
        if(altitude>0)area.altitude=altitude;
        auto path=fixedwingLawnmowerPath(area,radius,spacing,lead);
        const auto key=std::make_pair(v.name,area.area_id);
        lengths[key]=pathLength(path);fixedwing_paths[key]=std::move(path);
      }
    }
    auto assignments=assignAreas(loaded_areas_,coverage_positions,speeds,lengths);
    for (auto& entry:vehicles_) {entry.second.route.clear();entry.second.route_index=0;}
    for (const auto& entry:assignments) {
      auto& route=vehicles_.at(entry.first).route;
      auto& vehicle=vehicles_.at(entry.first);
      if(vehicle.mobility=="fixedwing"){
        std::vector<std::vector<Vec3>> paths;
        for(const auto& area:entry.second)paths.push_back(fixedwing_paths.at({entry.first,area.area.area_id}));
        const auto params=fixedwing_parameters.at(entry.first);
        route=connectFixedwingPaths(paths,params[0],params[1]);
        const auto pose=vehicle.poses.latest();
        if(pose){
          const auto& q=pose->orientation;
          const double yaw=std::atan2(2.0*(q[3]*q[2]+q[0]*q[1]),1.0-2.0*(q[1]*q[1]+q[2]*q[2]));
          route=fixedwingEntryPath(coverage_positions.at(entry.first),yaw,route,params[0],params[1]);
        }
      } else {
        for (const auto& area:entry.second) route.insert(route.end(),area.path.begin(),area.path.end());
      }
      publishRoutePreview(vehicles_.at(entry.first));
    }
  }

  double fixedwingSetting(const Vehicle& vehicle,const std::string& key,double fallback) {
    const auto value=child(child(vehicle.config,"fixedwing"),key);
    if(value.getType()!=XmlRpc::XmlRpcValue::TypeInvalid)return xmlDouble(value,fallback);
    double global=fallback;private_nh_.param("planner/fixedwing/"+key,global,fallback);return global;
  }

  void publishRoutePreview(const Vehicle& v) {
    nav_msgs::Path path; path.header.stamp=ros::Time::now();path.header.frame_id=shared_frame_;
    for (const auto& p:v.route) {geometry_msgs::PoseStamped pose;pose.header=path.header;
      pose.pose.position.x=p[0];pose.pose.position.y=p[1];pose.pose.position.z=p[2];pose.pose.orientation.w=1.0;path.poses.push_back(pose);}
    const_cast<Vehicle&>(v).route_preview_pub.publish(path);
  }

  void publishPath(Vehicle& v, const std::vector<Vec3>& points, std::uint32_t id) {
    nav_msgs::Path path;path.header.seq=id;path.header.stamp=ros::Time::now();path.header.frame_id=shared_frame_;
    for (const auto& p:points) {geometry_msgs::PoseStamped pose;pose.header=path.header;pose.header.seq=id;
      pose.pose.position.x=p[0];pose.pose.position.y=p[1];pose.pose.position.z=p[2];pose.pose.orientation.w=1.0;path.poses.push_back(pose);}
    if (v.backend=="direct_controller_test") v.controller_path_pub.publish(path);
    else v.planner_path_pub.publish(path);
  }

  std::uint32_t newGoalId() { return static_cast<std::uint32_t>(next_goal_id_++); }

  void publishDirectStatus(Vehicle& v, std::uint32_t id, std::uint8_t state,
                           const std::string& detail) {
    if (v.direct_status_pub) {PlannerStatus m;m.header.stamp=ros::Time::now();m.header.frame_id=shared_frame_;m.goal_id=id;m.state=state;m.detail=detail;v.direct_status_pub.publish(m);}
  }

  void publishGoal(Vehicle& v, const Vec3& goal, const std::string& kind,
                   std::uint32_t object_id, const std::vector<Vec3>* route=nullptr) {
    const auto id=newGoalId();
    ActiveGoal active;active.id=id;active.kind=kind;active.object_id=object_id;active.goal=goal;v.active=active;
    if (v.backend=="direct_controller_test") {
      if (v.mobility=="fixedwing" && route && route->size()>=2) publishPath(v,*route,id);
      else {
        mavros_msgs::PositionTarget setpoint;setpoint.header.stamp=ros::Time::now();setpoint.header.frame_id=shared_frame_;
        setpoint.coordinate_frame=mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        setpoint.type_mask=mavros_msgs::PositionTarget::IGNORE_VX|mavros_msgs::PositionTarget::IGNORE_VY|
          mavros_msgs::PositionTarget::IGNORE_VZ|mavros_msgs::PositionTarget::IGNORE_AFX|
          mavros_msgs::PositionTarget::IGNORE_AFY|mavros_msgs::PositionTarget::IGNORE_AFZ|
          mavros_msgs::PositionTarget::IGNORE_YAW|mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
        setpoint.position.x=goal[0];setpoint.position.y=goal[1];setpoint.position.z=goal[2];v.controller_goal_pub.publish(setpoint);
      }
      publishDirectStatus(v,id,PlannerStatus::ACTIVE,"direct goal published; obstacle avoidance disabled");
    } else {
      if (route && route->size()>=2) {
        std::vector<Vec3> remaining(route->begin()+std::min<std::size_t>(v.route_index,route->size()-1),route->end());
        if (v.has_position && (remaining.empty()||distance3d(v.position,remaining.front())>1e-3)) remaining.insert(remaining.begin(),v.position);
        if (remaining.size()>=2) publishPath(v,remaining,id);
        else publishGoalPose(v,goal,id);
      } else publishGoalPose(v,goal,id);
    }
  }

  void publishGoalPose(Vehicle& v,const Vec3& goal,std::uint32_t id) {
    geometry_msgs::PoseStamped pose;pose.header.seq=id;pose.header.stamp=ros::Time::now();pose.header.frame_id=shared_frame_;
    pose.pose.position.x=goal[0];pose.pose.position.y=goal[1];pose.pose.position.z=goal[2];pose.pose.orientation.w=1.0;
    v.planner_goal_pub.publish(pose);
  }

  void publishNextScout(Vehicle& v) {
    if (mission_state_!=kActive) return;
    if (v.route_index>=v.route.size()) {
      v.active.reset();
      auto region=verification_region_by_scout_.find(v.name);
      if(region!=verification_region_by_scout_.end())finishVerification(region->second);
      else {assignPendingVerifications();markSearchCompleteIfReady();}
      return;
    }
    const auto goal=v.route[v.route_index];
    const auto endpoint_index = (v.backend=="direct_controller_test" && v.mobility!="fixedwing")
        ? v.route_index : v.route.size()-1;
    const std::string kind=verification_region_by_scout_.count(v.name)?"verification":"search";
    const auto object_id=kind=="verification"?verification_region_by_scout_.at(v.name):static_cast<std::uint32_t>(endpoint_index);
    publishGoal(v,goal,kind,object_id,&v.route);
  }

  void plannerStatusCallback(const std::string& name,const PlannerStatusConstPtr& msg) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);auto it=vehicles_.find(name);if(it==vehicles_.end())return;
    handleGoalStatus(it->second,msg->goal_id,msg->state,msg->detail);
  }

  void pathStatusCallback(const std::string& name,const xd_uav_controller::PathStatusConstPtr& msg) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);auto it=vehicles_.find(name);if(it==vehicles_.end()||!it->second.active)return;
    auto& v=it->second;if(v.active->id!=msg->path_id)return;
    std::uint8_t state=msg->state==xd_uav_controller::PathStatus::COMPLETED?PlannerStatus::REACHED:
      (msg->state==xd_uav_controller::PathStatus::ACTIVE||msg->state==xd_uav_controller::PathStatus::ACCEPTED)?PlannerStatus::ACTIVE:PlannerStatus::FAILED;
    handleGoalStatus(v,msg->path_id,state,msg->detail);
  }

  void handleGoalStatus(Vehicle& v,std::uint32_t id,std::uint8_t state,const std::string& detail) {
    if (!v.active||v.active->id!=id)return;
    const auto active=*v.active;
    if (state==PlannerStatus::ACTIVE||state==PlannerStatus::PLANNING) {
      if(active.kind=="rescue") {allocator_.markExecuting(active.object_id);auto t=allocator_.tasks().find(active.object_id);if(t!=allocator_.tasks().end())registry_.setStatus(t->second.target_id,TARGET_EXECUTING);}
      return;
    }
    if (state==PlannerStatus::REACHED) {
      if(active.kind=="search"||active.kind=="verification") {
        v.route_index=std::max(v.route_index,active.kind=="verification"?v.route.size():static_cast<std::size_t>(active.object_id+1));v.active.reset();publishNextScout(v);
        if(active.kind=="search"&&v.mobility=="fixedwing")assignPendingVerifications();
      }
      else if(active.kind=="rescue") {
        auto task=allocator_.tasks().find(active.object_id);v.active.reset();
        if(task!=allocator_.tasks().end()&&post_arrival_mode_=="task_execute"){
          std::string handoff;
          try{handoff=taskExecuteHandoff(task->second,v);}catch(const std::exception& e){handleTaskExecuteFailure(task->second,v,e.what());handoff="failed";}
          if(handoff=="position"){
            worker_visual_waiting_.erase(v.name);
            task->second.detail="worker reached target navigation point; starting position-only task handoff";
            startTaskExecution(task->second,v,-1,"navigation reached; metric position-only handoff");
          }else if(handoff=="visual"){
            worker_visual_waiting_[v.name]=task->second.task_id;
            task->second.detail="worker reached target navigation point; waiting for a matching local camera detection";
            if(v.backend=="direct_controller_test"&&v.mobility!="fixedwing")publishDirectHold(v);
          }
          dispatchAssignments();
        }else{
          auto completed=allocator_.complete(active.object_id);
          if(completed)registry_.setStatus(completed->target_id,TARGET_COMPLETED);
          dispatchAssignments();
        }
      }
    } else if (state==PlannerStatus::BLOCKED||state==PlannerStatus::FAILED) {
      if(active.kind=="rescue") {auto task=allocator_.releaseWorker(v.name,"execution backend failed: "+detail);if(task){registry_.setStatus(task->target_id,TARGET_CONFIRMED);worker_retry_after_[v.name]=ros::Time::now().toSec()+std::max(0.0,task_execute_retry_delay_);allocator_.workers()[v.name].online=false;}}
      v.active.reset();dispatchAssignments();
    }
    checkMissionComplete();publishState();
  }

  void dispatchAssignments() {
    if(mission_state_!=kActive)return;
    for(const auto& assignment:allocator_.assignPending()) {
      auto& task=allocator_.tasks().at(assignment.task.task_id);
      auto& worker=vehicles_.at(assignment.worker.name);
      registry_.setStatus(task.target_id,TARGET_ASSIGNED);
      if(taskUsesDirectControl(task,worker)){
        // direct_control owns the whole point-to-point leg. Start it as soon
        // as allocation assigns the target so the full XYZ target_pose reaches
        // task_execute without waiting for the navigation backend to report
        // REACHED at its usually wider acceptance radius.
        task.goal=task.target_position;
        startTaskExecution(task,worker,-1,"direct_control started from assigned XYZ target");
      }else{
        task.goal={{task.target_position[0],task.target_position[1],worker.position[2]}};
        publishWorkerGoal(worker,task);
        task.detail=worker.backend+" goal published";
      }
    }
    publishState();
  }

  void publishWorkerGoal(Vehicle& worker,const RescueTaskRecord& task) {
    if(worker.mobility=="fixedwing"&&worker.has_position){
      std::vector<Vec3> path{worker.position,task.goal};
      publishGoal(worker,task.goal,"rescue",task.task_id,&path);
    }else publishGoal(worker,task.goal,"rescue",task.task_id);
  }

  void checkMissionComplete() {
    if(mission_state_!=kActive)return;
    markSearchCompleteIfReady();
    if(search_completed_at_<=0.0||!verification_pending_.empty()||!verification_region_by_scout_.empty())return;
    if(ros::Time::now().toSec()-search_completed_at_<completion_grace_sec_)return;
    for(const auto& item:allocator_.tasks())if(item.second.status!=TASK_COMPLETED)return;
    mission_state_=kCompleted;mission_detail_="all search routes and rescue tasks completed";publishMissionState();
  }

  bool startCallback(std_srvs::Trigger::Request&,std_srvs::Trigger::Response& res) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if(mission_state_!=kLoaded||loaded_areas_.empty()){res.success=false;res.message="load search areas first";return true;}
    const double now=ros::Time::now().toSec();std::map<std::string,Vec3> ready_positions;
    for(const auto& n:scouts_){const auto& v=vehicles_.at(n);if(ready(v,now))ready_positions[n]=v.position;}
    const auto required=require_all_scouts_ready_?scouts_.size():std::min(scouts_.size(),static_cast<std::size_t>(std::max(1,minimum_ready_scouts_)));
    if(ready_positions.size()<required){res.success=false;res.message="waiting for ready scouts";mission_detail_=res.message;publishMissionState();return true;}
    try{prepareRoutes(ready_positions);}catch(const std::exception& e){res.success=false;res.message=e.what();return true;}
    bool any=false;for(const auto& n:scouts_)if(!vehicles_.at(n).route.empty())any=true;
    if(!any){res.success=false;res.message="no executable route was generated";return true;}
    mission_state_=kActive;mission_detail_="search execution started by operator";publishMissionState();
    for(const auto& n:scouts_)publishNextScout(vehicles_.at(n));
    dispatchAssignments();res.success=true;res.message=mission_detail_;return true;
  }

  bool pauseCallback(std_srvs::Trigger::Request&,std_srvs::Trigger::Response& res) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);if(mission_state_!=kActive){res.success=false;res.message="only ACTIVE mission can pause";return true;}
    if(!cancelActiveGoals(res.message)){res.success=false;return true;}
    cancelAllTaskExecutions("mission paused; post-arrival task returned to allocation queue","release");
    mission_state_=kPaused;mission_detail_="mission paused by operator";publishMissionState();res.success=true;res.message=mission_detail_;return true;
  }

  bool resumeCallback(std_srvs::Trigger::Request&,std_srvs::Trigger::Response& res) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);if(mission_state_!=kPaused){res.success=false;res.message="only PAUSED mission can resume";return true;}
    mission_state_=kActive;mission_detail_="mission resumed by operator";publishMissionState();
    for(const auto& n:scouts_)publishNextScout(vehicles_.at(n));
    for(const auto& n:workers_){auto& v=vehicles_.at(n);auto t=allocator_.workers().at(n).assigned_task;if(t){auto task=allocator_.tasks().find(*t);if(task!=allocator_.tasks().end())publishWorkerGoal(v,task->second);}}
    dispatchAssignments();res.success=true;res.message=mission_detail_;return true;
  }

  bool cancelActiveGoals(std::string& detail) {
    for(auto& item:vehicles_)if(item.second.active){
      auto& vehicle=item.second;
      if(vehicle.backend=="direct_controller_test") {publishDirectHold(vehicle);continue;}
      CancelPlanning srv;srv.request.goal_id=vehicle.active->id;srv.request.reason="mission state transition";
      if(!vehicle.cancel_client.call(srv)||!srv.response.success){detail="cannot cancel planning for "+item.first;return false;}}
    return true;
  }

  bool cancelVehicleGoal(Vehicle& vehicle,const std::string& reason,std::string& detail) {
    if(!vehicle.active)return true;
    if(vehicle.backend=="direct_controller_test"){publishDirectHold(vehicle);return true;}
    CancelPlanning srv;srv.request.goal_id=vehicle.active->id;srv.request.reason=reason;
    if(!vehicle.cancel_client.call(srv)||!srv.response.success){detail=srv.response.message.empty()?"planner cancel failed":srv.response.message;return false;}
    return true;
  }

  void publishDirectHold(Vehicle& v) {
    if(!v.has_position||!v.controller_goal_pub)return;
    mavros_msgs::PositionTarget hold;hold.header.stamp=ros::Time::now();hold.header.frame_id=shared_frame_;
    hold.coordinate_frame=mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    hold.type_mask=mavros_msgs::PositionTarget::IGNORE_VX|mavros_msgs::PositionTarget::IGNORE_VY|
      mavros_msgs::PositionTarget::IGNORE_VZ|mavros_msgs::PositionTarget::IGNORE_AFX|
      mavros_msgs::PositionTarget::IGNORE_AFY|mavros_msgs::PositionTarget::IGNORE_AFZ|
      mavros_msgs::PositionTarget::IGNORE_YAW|mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
    hold.position.x=v.position[0];hold.position.y=v.position[1];hold.position.z=v.position[2];
    v.controller_goal_pub.publish(hold);
  }

  bool stopCallback(std_srvs::Trigger::Request&,std_srvs::Trigger::Response& res) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if(mission_state_!=kActive&&mission_state_!=kPaused){res.success=false;res.message="mission is not running";return true;}
    std::string detail;
    if(!cancelActiveGoals(detail)){res.success=false;res.message=detail;return true;}
    cancelAllTaskExecutions("mission stopped by operator","fail");
    mission_state_=kAborted;mission_detail_="mission stopped by operator";
    for(auto& item:vehicles_){if(item.second.active&&item.second.active->kind=="rescue")allocator_.cancelTask(item.second.active->object_id,"mission stopped by operator");item.second.active.reset();}
    for(const auto& waiting:worker_visual_waiting_)allocator_.cancelTask(waiting.second,"mission stopped by operator");
    worker_visual_waiting_.clear();
    publishMissionState();publishState();res.success=true;res.message=mission_detail_;return true;
  }

  void clearResults() { registry_.clear();allocator_.clearTasks();publishState(); }

  bool resetCallback(std_srvs::Trigger::Request&,std_srvs::Trigger::Response& res) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);std::string detail;if(!cancelActiveGoals(detail)){res.success=false;res.message=detail;return true;}
    cancelAllTaskExecutions("mission reset by operator","fail");worker_visual_waiting_.clear();clearResults();worker_retry_after_.clear();for(auto& item:vehicles_){item.second.route.clear();item.second.route_index=0;item.second.active.reset();}
    mission_state_=loaded_areas_.empty()?kIdle:kLoaded;mission_detail_=loaded_areas_.empty()?"waiting for search areas":"search areas loaded; waiting for start";publishMissionState();res.success=true;res.message=mission_detail_;return true;
  }

  bool clearAllCallback(std_srvs::Trigger::Request&,std_srvs::Trigger::Response& res) {
    std_srvs::Trigger::Request req;
    if(!resetCallback(req,res)) return false;
    if(!res.success) return true;
    loaded_areas_.clear();mission_state_=kIdle;mission_detail_="mission cleared; waiting for search areas";publishMissionState();res.message=mission_detail_;return true;
  }

  bool restartCallback(std_srvs::Trigger::Request&,std_srvs::Trigger::Response& res) {
    std_srvs::Trigger::Request req;std_srvs::Trigger::Response ignored;resetCallback(req,ignored);return startCallback(req,res);
  }

  bool replanCallback(std_srvs::Trigger::Request&,std_srvs::Trigger::Response& res) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);if(mission_state_!=kActive){res.success=false;res.message="mission must be ACTIVE to replan";return true;}
    for(const auto& n:scouts_){auto& v=vehicles_.at(n);if(v.active)publishNextScout(v);}res.success=true;res.message="active routes republished";return true;
  }

  bool clearResultsCallback(std_srvs::Trigger::Request&,std_srvs::Trigger::Response& res) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if(mission_state_==kActive||mission_state_==kPaused){res.success=false;res.message="stop or reset the mission before clearing results";return true;}
    clearResults();verification_pending_.clear();verification_plans_.clear();verification_targets_by_region_.clear();verification_region_by_target_.clear();verification_scout_by_region_.clear();verification_region_by_scout_.clear();verification_region_states_.clear();worker_retry_after_.clear();worker_visual_waiting_.clear();publishVerificationAreas();
    mission_detail_="targets and rescue-task results cleared";res.success=true;res.message=mission_detail_;publishMissionState();return true;
  }
  bool cancelAllTasksCallback(std_srvs::Trigger::Request&,std_srvs::Trigger::Response& res) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    for(auto& item:vehicles_)if(item.second.active&&item.second.active->kind=="rescue"){
      std::string detail;if(!cancelVehicleGoal(item.second,"all tasks cancelled by operator",detail)){res.success=false;res.message="cannot cancel "+item.first+": "+detail;return true;}item.second.active.reset();}
    cancelAllTaskExecutions("cancelled by operator","fail");worker_visual_waiting_.clear();
    for(auto& item:allocator_.tasks())if(item.second.status!=TASK_COMPLETED){auto t=allocator_.cancelTask(item.first,"cancelled by operator");if(t)registry_.setStatus(t->target_id,TARGET_CONFIRMED);}
    res.success=true;res.message="active tasks cancelled";publishState();return true;
  }

  bool skipWaypointCallback(VehicleCommand::Request& req,VehicleCommand::Response& res) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);auto it=vehicles_.find(req.vehicle_name);if(it==vehicles_.end()||!it->second.scout){res.success=false;res.message="unknown scout";return true;}
    auto& v=it->second;if(v.route_index<v.route.size())++v.route_index;v.active.reset();publishNextScout(v);res.success=true;res.message="waypoint skipped";return true;
  }

  bool setVehicleEnabledCallback(SetVehicleEnabled::Request& req,SetVehicleEnabled::Response& res) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);auto it=vehicles_.find(req.vehicle_name);if(it==vehicles_.end()){res.success=false;res.message="unknown vehicle";return true;}
    it->second.enabled=req.enabled;if(!req.enabled){std::string detail;if(!cancelVehicleGoal(it->second,"vehicle disabled by operator",detail)){res.success=false;res.message=detail;return true;}
      const auto active_exec=active_task_executions_.find(req.vehicle_name);
      if(active_exec!=active_task_executions_.end()){const auto id=active_exec->second.first;cancelTaskExecution(req.vehicle_name,"vehicle disabled by operator; task released");auto task=allocator_.releaseWorker(req.vehicle_name,"vehicle disabled by operator; task released");if(task)registry_.setStatus(task->target_id,TARGET_CONFIRMED);(void)id;}
      if(!it->second.scout){auto task=allocator_.releaseWorker(req.vehicle_name,"worker disabled by operator");if(task)registry_.setStatus(task->target_id,TARGET_CONFIRMED);}
      auto waiting=worker_visual_waiting_.find(req.vehicle_name);if(waiting!=worker_visual_waiting_.end()){auto task=allocator_.releaseWorker(req.vehicle_name,"worker disabled while waiting for visual handoff");if(task)registry_.setStatus(task->target_id,TARGET_CONFIRMED);worker_visual_waiting_.erase(waiting);}
      it->second.active.reset();}
    dispatchAssignments();res.success=true;res.message=req.enabled?"vehicle enabled":"vehicle disabled";return true;
  }

  bool cancelTaskCallback(TaskCommand::Request& req,TaskCommand::Response& res) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);auto task_it=allocator_.tasks().find(req.task_id);
    if(task_it==allocator_.tasks().end()||task_it->second.status==TASK_COMPLETED){res.success=false;res.message="task not found or already complete";return true;}
    for(auto& item:vehicles_)if(item.second.active&&item.second.active->kind=="rescue"&&item.second.active->object_id==req.task_id){std::string detail;if(!cancelVehicleGoal(item.second,"task cancelled by operator",detail)){res.success=false;res.message="cannot cancel worker planner: "+detail;return true;}item.second.active.reset();}
    for(auto it=active_task_executions_.begin();it!=active_task_executions_.end();){if(it->second.first==req.task_id){const auto worker=it->first;++it;cancelTaskExecution(worker,"task cancelled by operator");}else ++it;}
    for(auto it=worker_visual_waiting_.begin();it!=worker_visual_waiting_.end();)if(it->second==req.task_id)it=worker_visual_waiting_.erase(it);else ++it;
    auto task=allocator_.cancelTask(req.task_id,"cancelled by operator");if(task)registry_.setStatus(task->target_id,TARGET_CONFIRMED);
    res.success=static_cast<bool>(task);res.message=task?"task cancelled":"task not found or already complete";dispatchAssignments();publishState();return true;
  }
  bool retryTaskCallback(TaskCommand::Request& req,TaskCommand::Response& res) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);auto task_it=allocator_.tasks().find(req.task_id);
    if(task_it==allocator_.tasks().end()||task_it->second.status==TASK_COMPLETED){res.success=false;res.message="task not found or complete";return true;}
    for(auto it=active_task_executions_.begin();it!=active_task_executions_.end();){if(it->second.first==req.task_id){const auto worker=it->first;++it;cancelTaskExecution(worker,"task retry requested by operator");}else ++it;}
    for(auto it=worker_visual_waiting_.begin();it!=worker_visual_waiting_.end();)if(it->second==req.task_id)it=worker_visual_waiting_.erase(it);else ++it;
    for(auto& item:vehicles_)if(item.second.active&&item.second.active->kind=="rescue"&&item.second.active->object_id==req.task_id){std::string detail;if(!cancelVehicleGoal(item.second,"task retry requested",detail)){res.success=false;res.message="cannot cancel worker planner: "+detail;return true;}item.second.active.reset();}
    auto task=allocator_.retryTask(req.task_id,"retried by operator");if(task)registry_.setStatus(task->target_id,TARGET_CONFIRMED);
    res.success=static_cast<bool>(task);res.message=task?"task returned to queue":"task not found or complete";dispatchAssignments();return true;
  }
  bool rejectTargetCallback(TargetCommand::Request& req,TargetCommand::Response& res) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);auto target=registry_.targets().find(req.target_id);
    if(target==registry_.targets().end()){res.success=false;res.message="unknown target";return true;}
    std::optional<std::uint32_t> canonical_task;
    for(const auto& entry:allocator_.tasks())if(entry.second.target_id==req.target_id){canonical_task=entry.first;break;}
    if(canonical_task)for(auto& item:vehicles_)if(item.second.active&&item.second.active->kind=="rescue"&&item.second.active->object_id==*canonical_task){std::string detail;if(!cancelVehicleGoal(item.second,"target rejected by operator",detail)){res.success=false;res.message="cannot cancel worker planner: "+detail;return true;}item.second.active.reset();}
    if(canonical_task)for(auto it=active_task_executions_.begin();it!=active_task_executions_.end();){if(it->second.first==*canonical_task){const auto worker=it->first;++it;cancelTaskExecution(worker,"target rejected by operator");}else ++it;}
    if(canonical_task){
      for(auto it=worker_visual_waiting_.begin();it!=worker_visual_waiting_.end();){
        if(it->second==*canonical_task)it=worker_visual_waiting_.erase(it);
        else ++it;
      }
    }
    allocator_.removeTargetTask(req.target_id);
    auto region=verification_region_by_target_.find(req.target_id);
    if(region!=verification_region_by_target_.end()){auto members=verification_targets_by_region_.find(region->second);if(members!=verification_targets_by_region_.end())members->second.erase(req.target_id);verification_region_by_target_.erase(region);}
    registry_.remove(req.target_id);publishVerificationAreas();dispatchAssignments();res.success=true;res.message="target rejected and removed";publishState();return true;
  }
  bool retryVerificationCallback(TargetCommand::Request& req,TargetCommand::Response& res) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    const auto id=req.target_id;
    if(!hierarchical_search_enabled_){res.success=false;res.message="hierarchical search is disabled";return true;}
    if(mission_state_!=kActive){res.success=false;res.message="mission must be ACTIVE";return true;}
    auto target=registry_.targets().find(id);if(target==registry_.targets().end()){res.success=false;res.message="unknown target";return true;}
    auto owner=verification_region_by_target_.find(id);
    if(owner!=verification_region_by_target_.end()&&(verification_region_states_[owner->second]=="pending"||verification_region_states_[owner->second]=="active")){res.success=false;res.message="target verification is already pending";return true;}
    if(target->second.status==TARGET_ASSIGNED||target->second.status==TARGET_EXECUTING||target->second.status==TARGET_COMPLETED){res.success=false;res.message="target already has rescue progress";return true;}
    if(owner==verification_region_by_target_.end())queueVerification(target->second);
    else{registry_.setStatus(id,TARGET_VERIFYING);verification_region_states_[owner->second]="pending";verification_pending_.push_back(owner->second);assignPendingVerifications();}
    publishVerificationAreas();publishState();res.success=true;res.message="target verification queued";return true;
  }

  bool getStateCallback(GetMissionState::Request&,GetMissionState::Response& res) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);res.success=true;res.state=mission_state_;res.detail=mission_detail_;res.loaded_area_count=loaded_areas_.size();res.target_count=registry_.targets().size();res.pending_verification_count=verification_pending_.size()+verification_region_by_scout_.size();res.task_count=allocator_.tasks().size();
    for(const auto& n:scouts_){const auto& v=vehicles_.at(n);std::ostringstream s;s<<n<<" route="<<v.route_index<<"/"<<v.route.size();res.route_progress.push_back(s.str());}
    for(const auto& item:vehicles_)if(item.second.active){std::ostringstream s;s<<item.first<<" goal="<<item.second.active->id<<" kind="<<item.second.active->kind;res.active_goals.push_back(s.str());}
    for(const auto& item:active_task_executions_){std::ostringstream s;s<<item.first<<" kind=task_execute,task="<<item.second.first;res.active_goals.push_back(s.str());}
    return true;
  }

  void timerCallback(const ros::TimerEvent&) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);const double now=ros::Time::now().toSec();
    if(mission_state_==kActive) {
      registry_.expire(now);
      for(const auto& task:allocator_.markOfflineWorkers(now,worker_timeout_))registry_.setStatus(task.target_id,TARGET_CONFIRMED);
      for(auto& item:vehicles_)if(item.second.active&&item.second.backend=="direct_controller_test"&&item.second.mobility!="fixedwing") {
        auto& v=item.second;const auto d=distance3d(v.position,v.active->goal);
        if(!v.has_position||!ready(v,now)){v.active->entered_tolerance_at=-1;continue;}
        const double tolerance=std::max(0.0,direct_goal_tolerance_);if(d<=tolerance){if(v.active->entered_tolerance_at<0)v.active->entered_tolerance_at=now;
          if(now-v.active->entered_tolerance_at>=std::max(0.0,direct_arrival_dwell_))handleGoalStatus(v,v.active->id,PlannerStatus::REACHED,"direct goal dwell completed");}
        else v.active->entered_tolerance_at=-1;
      }
      dispatchAssignments();checkMissionComplete();
    }
    publishState();
  }

  void publishMissionState() {std_msgs::String msg;msg.data=mission_state_;mission_pub_.publish(msg);}

  void publishState() {
    const auto stamp=ros::Time::now();GlobalTargetArray targets;targets.header.stamp=stamp;targets.header.frame_id=shared_frame_;
    for(const auto& item:registry_.targets()) {const auto& t=item.second;GlobalTarget m;m.header=targets.header;m.target_id=t.target_id;m.class_id=t.class_id;m.position.x=t.position[0];m.position.y=t.position[1];m.position.z=t.position[2];for(int i=0;i<9;++i)m.position_covariance[i]=t.covariance[i];m.confidence=t.confidence;m.observation_count=t.observation_count;m.observer_uavs.assign(t.observer_uavs.begin(),t.observer_uavs.end());m.status=t.status;targets.targets.push_back(m);}targets_pub_.publish(targets);
    RescueTaskArray tasks;tasks.header=targets.header;for(const auto& item:allocator_.tasks()){const auto& t=item.second;RescueTask m;m.header=tasks.header;m.task_id=t.task_id;m.target_id=t.target_id;m.class_id=t.class_id;m.goal.x=t.goal[0];m.goal.y=t.goal[1];m.goal.z=t.goal[2];m.priority=static_cast<std::uint8_t>(std::max(0,std::min(255,t.priority)));m.assigned_worker=t.assigned_worker;m.status=t.status;m.detail=t.detail;tasks.tasks.push_back(m);}tasks_pub_.publish(tasks);
  }

};

}  // namespace xd_uav_task_allocate

int main(int argc,char** argv) {
  ros::init(argc,argv,"task_allocate_coordinator");
  try {xd_uav_task_allocate::TaskAllocateCoordinator coordinator;ros::spin();}
  catch(const std::exception& e){ROS_FATAL("[task_allocate_cpp] startup failed: %s",e.what());return 1;}
  return 0;
}
