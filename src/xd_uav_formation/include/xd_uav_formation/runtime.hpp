#pragma once

#include <xd_uav_formation/core.hpp>
#include <xd_uav_controller/ControlState.h>
#include <xd_uav_formation/FormationControl.h>
#include <xd_uav_task_execute/ExecuteTaskActionGoal.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_srvs/SetBool.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include <array>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

namespace xd_uav_formation {

struct Config {
  std::string mode{"leader_follow"};
  std::string team_id{"team1"};
  std::string leader_name{"uav1"};
  std::string common_frame{"world"};
  std::string shape{"manual"};
  std::vector<std::string> members;
  std::map<std::string, Vec3> offsets;
  std::vector<Vec3> waypoints;
  double minimum_slot_distance{2.0};
  double spacing{80.0};
  int follower_count{1};
  int slot_index{1};
  double corner_radius{0.0};
  double cruise_speed{20.0};
  double lookahead{60.0};
  double consensus_gain{0.1};
  double convergence_time{500.0};
  double max_consensus_velocity{4.0};
  double acquire_consensus_gain{0.1};
  double acquire_max_consensus_velocity{4.0};
  double acquire_max_speed_adjustment{4.0};
  double formation_heading_rate_limit{0.10};
  double formation_speed_slew_rate{2.0};
  double formation_course_slew_rate{0.25};
  double route_correction_gain{0.1};
  double max_route_correction_velocity{4.0};
  bool loop_path{true};
  double acquire_tolerance{8.0};
  double acquire_height_tolerance{2.0};
  double acquire_vertical_spacing{8.0};
  double acquire_timeout{180.0};
  double fixedwing_minimum_slot_distance{30.0};
  double fixedwing_min_speed{11.0};
  double fixedwing_max_catchup_speed{4.0};
  double fixedwing_min_groundspeed_for_course{2.0};
  double multirotor_max_speed{5.0};
  double catchup_gain{0.25};
  double catchup_ki{0.01};
  double catchup_kd{0.20};
  double catchup_deadband{1.0};
  double catchup_integral_limit{50.0};
  double catchup_derivative_filter_time_constant{0.4};
  double multirotor_max_catchup_speed{3.0};
  double yaw_offset{0.0};
  double rate{20.0};
  double state_timeout{0.5};
  double mavros_timeout{2.5};
  double tf_max_age{1.0};
  double future_stamp_tolerance{0.5};
  double tf_lookup_timeout{0.05};
  double tf_buffer_cache_time{5.0};
  double external_quiet_time{0.5};
  double service_timeout{3.0};
  std::string reference_topic{"control/reference/setpoint"};
  std::string path_topic{"control/reference/path"};
  std::string trajectory_topic{"control/reference/trajectory"};
  std::string team_cancel_topic{"/formation/{team_id}/cancel"};
  std::string preempt_topic{"formation/preempt"};
  std::string task_goal_topic{"task_execute/execute/goal"};
};

struct AcquisitionPlan {
  std::vector<std::string> members;
  std::string shape;
  double spacing{0.0};
  ros::Time stamp;
  std::vector<Vec3> initial_positions;
  std::vector<Vec3> slots;
  std::vector<double> tiers;
  double heading{0.0};
  double altitude{0.0};
};

using ControlStateMap =
    std::map<std::string, xd_uav_controller::ControlState>;
using ReceivedTimeMap = std::map<std::string, ros::Time>;
using MavrosStateMap = std::map<std::string, mavros_msgs::State>;
using CenterMap = std::map<std::string, geometry_msgs::PointStamped>;
using StatusMap = std::map<std::string, std::string>;

struct SharedSnapshot {
  ControlStateMap states;
  ReceivedTimeMap state_received;
  MavrosStateMap mavros;
  ReceivedTimeMap mavros_received;
  CenterMap centers;
  ReceivedTimeMap center_received;
  StatusMap statuses;
  ReceivedTimeMap status_received;
};

struct SharedData : SharedSnapshot {
  std::mutex mutex;
  std::mutex acquisition_plan_mutex;
  std::uint64_t revision{0};
  std::uint64_t snapshot_revision{0};
  std::shared_ptr<const SharedSnapshot> snapshot_cache;
  std::optional<AcquisitionPlan> acquisition_plan;

  void markChanged() { ++revision; }

  // Caller holds mutex. Members reuse one immutable snapshot until an input
  // callback changes the shared team data.
  std::shared_ptr<const SharedSnapshot> snapshotLocked() {
    if (!snapshot_cache || snapshot_revision != revision) {
      snapshot_cache = std::make_shared<SharedSnapshot>(
          static_cast<const SharedSnapshot&>(*this));
      snapshot_revision = revision;
    }
    return snapshot_cache;
  }
};

template <typename Map>
class SnapshotMapView {
 public:
  using const_iterator = typename Map::const_iterator;
  using key_type = typename Map::key_type;
  using mapped_type = typename Map::mapped_type;

  void bind(std::shared_ptr<const SharedSnapshot> owner, const Map* map) {
    owner_ = std::move(owner);
    map_ = map;
  }

  const_iterator find(const key_type& key) const { return source().find(key); }
  const_iterator end() const { return source().end(); }
  const mapped_type& at(const key_type& key) const { return source().at(key); }

 private:
  const Map& source() const {
    static const Map empty;
    return map_ ? *map_ : empty;
  }

  std::shared_ptr<const SharedSnapshot> owner_;
  const Map* map_{nullptr};
};

struct FormationGeometry {
  std::map<std::string, Vec3> member_offsets;
  std::vector<Vec3> slots;
  std::map<std::string, Vec3> slot_offsets;
};

class Follower {
 public:
  Follower(ros::NodeHandle nh, const Config& config, std::string member,
           std::shared_ptr<SharedData> shared,
           std::shared_ptr<const FormationGeometry> geometry,
           std::shared_ptr<tf2_ros::Buffer> tf_buffer,
           bool shared_inputs);
  ~Follower();

  bool setEnabled(bool enabled, std::string& message);
  void stateInput(const xd_uav_controller::ControlState& state,
                  const ros::Time& received);
  void mavrosInput(const mavros_msgs::State& state,
                   const ros::Time& received);
  void leaderStateInput(const xd_uav_controller::ControlState& state,
                        const ros::Time& received);
  void leaderMavrosInput(const mavros_msgs::State& state,
                         const ros::Time& received);
  void peerStateInput(const xd_uav_controller::ControlState& state,
                      const std::string& member,
                      const ros::Time& received);
  void peerMavrosInput(const mavros_msgs::State& state,
                       const std::string& member,
                       const ros::Time& received);
  void peerCenterInput(const geometry_msgs::PointStamped& center,
                       const std::string& member,
                       const ros::Time& received);
  void peerStatusInput(const std_msgs::String& status,
                       const std::string& member,
                       const ros::Time& received);
  void teamCancelInput(const std_msgs::String& message);
  void leaderTaskInput(const xd_uav_task_execute::ExecuteTaskActionGoal& goal);
  void leaderPreemptInput(const std_msgs::String& message);
  void clearAcquisitionPlan();
  void rememberOwnReference(const mavros_msgs::PositionTarget& message);
  bool isOwnReference(const mavros_msgs::PositionTarget& message) const;
  const std::string& member() const { return member_; }
  const std::string& mode() const { return config_.mode; }
  const std::string& teamId() const { return config_.team_id; }

 private:
  struct StateSnapshot {
    SnapshotMapView<ControlStateMap> states;
    SnapshotMapView<ReceivedTimeMap> state_received;
    SnapshotMapView<MavrosStateMap> mavros;
    SnapshotMapView<ReceivedTimeMap> mavros_received;
    SnapshotMapView<CenterMap> centers;
    SnapshotMapView<ReceivedTimeMap> center_received;
    SnapshotMapView<StatusMap> statuses;
    SnapshotMapView<ReceivedTimeMap> status_received;
  };
  struct Geometry {
    std::vector<std::string> members;
    std::vector<Vec3> centers;
    Vec3 mean;
    double largest_error{0.0};
    double heading{0.0};
    double turn_rate{0.0};
    Vec3 route_position;
    double route_vz{0.0};
    bool capturing{false};
  };

  void tick(const ros::TimerEvent& event);
  bool setEnabledService(std_srvs::SetBool::Request& request,
                         std_srvs::SetBool::Response& response);
  void externalReference(
      const ros::MessageEvent<const mavros_msgs::PositionTarget>& event);
  void externalPath(const nav_msgs::Path::ConstPtr& message);
  void externalTrajectory(
      const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr& message);
  void preempt(const std_msgs::String::ConstPtr& message);
  void taskGoal(const xd_uav_task_execute::ExecuteTaskActionGoal::ConstPtr& message);
  void publishStatus();
  void markPeerDropped(const std::string& member, const std::string& reason);
  void beginDropout(const std::string& reason);
  void stop(const std::string& reason, bool hold, bool broadcast = true);
  void publishHold();
  void publishCenter(const ros::Time& now, const StateSnapshot& snapshot);
  void refreshMembership(const ros::Time& now, const StateSnapshot& snapshot);
  void resumeDegradedAcquisition();
  bool fresh(const std_msgs::Header& header, bool has_message,
             const ros::Time& received, const ros::Time& now) const;
  std::optional<std::string> readiness(
      const ros::Time& now, const StateSnapshot& snapshot) const;
  StateSnapshot snapshot() const;
  std::vector<std::string> activeMembers() const;
  std::map<std::string, Vec3> consensusSlotOffsets() const;
  Vec3 leaderPoseInFollowerFrame(const ros::Time& now,
                                 const xd_uav_controller::ControlState& leader,
                                 const xd_uav_controller::ControlState& own,
                                 geometry_msgs::PoseStamped* pose,
                                 geometry_msgs::Vector3Stamped* velocity,
                                 geometry_msgs::Vector3Stamped* acceleration) const;
  Vec3 positionInFollowerFrame(const xd_uav_controller::ControlState& state,
                               const ros::Time& now,
                               const xd_uav_controller::ControlState& own) const;
  std::optional<double> leaderFollowPaceAdjustment(
      const ros::Time& now, const Vec3& leader_position, double heading,
      const StateSnapshot& snapshot);
  void resetCatchupPid();
  Vec3 pidCatchupVelocity(const Vec3& base_velocity,
                          const Vec3& slot_position,
                          const Vec3& own_position,
                          const ros::Time& now,
                          const xd_uav_controller::ControlState& own);
  double yaw(const geometry_msgs::PoseStamped& pose) const;
  mavros_msgs::PositionTarget positionTarget(
      const Vec3& position, double yaw, const ros::Time& now,
      const std::optional<Vec3>& velocity,
      const std::string& frame_id,
      const xd_uav_controller::ControlState& own) const;
  std::tuple<Vec3, geometry_msgs::Vector3, geometry_msgs::PoseStamped>
  stateInCommonFrame(const xd_uav_controller::ControlState& state,
                     const ros::Time& now) const;
  std::vector<Vec3> memberPositionsInCommonFrame(
      const ros::Time& now, const StateSnapshot& snapshot) const;
  bool usesFixedwingConsensusGuidance(
      const xd_uav_controller::ControlState& own) const;
  Geometry fixedwingConsensusGeometry(
      const ros::Time& now, const StateSnapshot& snapshot,
      const xd_uav_controller::ControlState& own);
  Vec3 ownCenterInCommonFrame(const ros::Time& now,
                              const StateSnapshot& snapshot,
                              const xd_uav_controller::ControlState& own);
  std::tuple<Vec3, Vec3, double> consensusCenters(
      const ros::Time& now, const StateSnapshot& snapshot,
      const xd_uav_controller::ControlState& own);
  std::vector<Vec3> teamCenterPoints(
      const Vec3& own_center, const StateSnapshot& snapshot) const;
  double effectiveConvergenceTime() const;
  std::pair<double, double> activeConsensusCorrection() const;
  PathResult pacedPathReference(const Vec3& center, double speed) const;
  double teamPacedSpeed(const ros::Time& now,
                        const std::vector<Vec3>& center_points,
                        const Vec3& mean, const Vec3& path_velocity,
                        double heading, double turn_rate,
                        const StateSnapshot& snapshot,
                        const xd_uav_controller::ControlState& own);
  Vec3 smoothCourseVelocity(const Vec3& velocity, const ros::Time& now,
                            const xd_uav_controller::ControlState& own);
  Vec3 crossTrackCenter(const Vec3& center, const Vec3& mean,
                        double heading) const;
  std::vector<Vec3> consensusNeighbors(
      const Vec3& center, const Vec3& mean, double heading) const;
  bool teamEnabled(const ros::Time& now,
                   const StateSnapshot& snapshot) const;
  void setAcquireStage(const std::string& stage);
  AcquisitionPlan buildAcquisitionPlan(const ros::Time& now,
                                       const StateSnapshot& snapshot) const;
  void planAcquisition(const ros::Time& now,
                       const StateSnapshot& snapshot);
  mavros_msgs::PositionTarget fixedwingWaitTarget(
      const ros::Time& now, const StateSnapshot& snapshot,
      const xd_uav_controller::ControlState& own);
  mavros_msgs::PositionTarget fixedwingGatheringTarget(
      const ros::Time& now, double altitude,
      const StateSnapshot& snapshot,
      const xd_uav_controller::ControlState& own);
  mavros_msgs::PositionTarget acquisitionTarget(
      const ros::Time& now, const StateSnapshot& snapshot,
      const xd_uav_controller::ControlState& own);
  mavros_msgs::PositionTarget fixedwingConsensusTarget(
      const ros::Time& now, const StateSnapshot& snapshot,
      const xd_uav_controller::ControlState& own,
      std::optional<double> altitude = std::nullopt);
  mavros_msgs::PositionTarget consensusTarget(
      const ros::Time& now, const StateSnapshot& snapshot,
      const xd_uav_controller::ControlState& own);
  mavros_msgs::PositionTarget formationTarget(
      const ros::Time& now, const StateSnapshot& snapshot,
      const xd_uav_controller::ControlState& own);
  bool ownReadinessFailure(const std::string& reason) const;

  ros::NodeHandle nh_;
  const Config& config_;
  std::string member_;
  std::string node_name_;
  std::string service_name_;
  Vec3 offset_;
  std::shared_ptr<SharedData> shared_;
  std::shared_ptr<const FormationGeometry> geometry_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> owned_tf_listener_;
  bool shared_inputs_{false};
  mutable std::mutex mutex_;
  bool enabled_{false};
  std::optional<xd_uav_controller::ControlState> own_state_;
  std::optional<xd_uav_controller::ControlState> leader_state_;
  ros::Time own_state_received_;
  ros::Time leader_state_received_;
  bool has_own_state_received_{false};
  bool has_leader_state_received_{false};
  std::optional<mavros_msgs::State> own_mavros_;
  std::optional<mavros_msgs::State> leader_mavros_;
  ros::Time own_mavros_received_;
  ros::Time leader_mavros_received_;
  bool has_own_mavros_received_{false};
  bool has_leader_mavros_received_{false};
  std::optional<ros::Time> last_external_reference_;
  mutable std::mutex own_reference_mutex_;
  std::set<std::uint64_t> own_reference_stamps_;
  std::set<std::string> dropped_members_;
  mutable std::vector<std::string> active_members_cache_;
  mutable bool active_members_dirty_{true};
  std::map<std::string, double> progress_error_filter_;
  mutable ros::Time common_state_cache_stamp_;
  mutable bool has_common_state_cache_stamp_{false};
  mutable std::unordered_map<const xd_uav_controller::ControlState*,
      std::tuple<Vec3, geometry_msgs::Vector3, geometry_msgs::PoseStamped>>
      common_state_cache_;
  std::optional<double> filtered_course_;
  std::optional<ros::Time> filtered_course_stamp_;
  std::optional<double> filtered_speed_;
  std::optional<double> paced_speed_;
  std::optional<ros::Time> paced_speed_stamp_;
  std::optional<ros::Time> last_status_publish_;
  double status_heartbeat_period_{1.0};
  std::optional<ros::Time> last_idle_center_publish_;
  double idle_center_period_{1.0};
  std::array<double, 2> catchup_integral_{{0.0, 0.0}};
  std::optional<std::array<double, 2>> catchup_previous_error_;
  std::array<double, 2> catchup_filtered_derivative_{{0.0, 0.0}};
  std::optional<ros::Time> catchup_pid_stamp_;
  std::string reason_{"waiting_for_start"};
  std::optional<std::string> acquire_stage_;
  std::optional<ros::Time> acquire_started_;
  std::vector<Vec3> acquire_slots_;
  std::vector<double> acquire_tiers_;
  std::vector<std::array<double, 2>> acquire_initial_xy_;
  std::optional<double> acquire_heading_;
  std::optional<double> acquire_altitude_;
  std::optional<double> consensus_heading_;
  std::optional<ros::Time> consensus_heading_stamp_;
  ros::Publisher reference_pub_;
  ros::Publisher path_pub_;
  ros::Publisher trajectory_pub_;
  ros::Publisher team_cancel_pub_;
  ros::Publisher status_pub_;
  ros::Publisher center_pub_;
  ros::ServiceServer service_;
  ros::Timer timer_;
  std::vector<ros::Subscriber> subscribers_;
};

class FormationRuntime {
 public:
  FormationRuntime(ros::NodeHandle nh, Config config,
                   std::shared_ptr<tf2_ros::Buffer> tf_buffer,
                   bool external_telemetry = false);
  ~FormationRuntime();
  bool setMembers(bool enabled, std::string& message);
  void shutdown();
  void clearAcquisitionPlan();
  void stateCallback(const xd_uav_controller::ControlState::ConstPtr& message,
                     const std::string& member);
  void mavrosCallback(const mavros_msgs::State::ConstPtr& message,
                      const std::string& member);
  const Config& config() const { return config_; }

 private:
  void centerCallback(const geometry_msgs::PointStamped::ConstPtr& message,
                      const std::string& member);
  void statusCallback(const std_msgs::String::ConstPtr& message,
                      const std::string& member);
  void teamCancelCallback(const std_msgs::String::ConstPtr& message);
  void leaderTaskCallback(
      const xd_uav_task_execute::ExecuteTaskActionGoal::ConstPtr& message);
  void leaderPreemptCallback(const std_msgs::String::ConstPtr& message);

  ros::NodeHandle nh_;
  Config config_;
  std::shared_ptr<SharedData> shared_;
  std::shared_ptr<const FormationGeometry> geometry_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  bool external_telemetry_{false};
  std::vector<std::unique_ptr<Follower>> followers_;
  std::map<std::string, Follower*> follower_by_name_;
  std::vector<ros::Subscriber> subscribers_;
};

}  // namespace xd_uav_formation
