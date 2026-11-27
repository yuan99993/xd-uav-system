/****************************************************************************
 *
 *   Copyright (c) 2021 Jaeyoung Lim, Autonomous Systems Lab,
 *  ETH Zürich. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
/**
 * @brief Motion primtiive based Terrain planner library
 *
 * Motion primitive based terrain planner library
 *
 * @author Jaeyoung Lim <jalim@ethz.ch>
 */

#ifndef TERRAIN_PLANNER_H
#define TERRAIN_PLANNER_H

#include <ros/callback_queue.h>
#include <ros/ros.h>

#include <geographic_msgs/GeoPointStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CameraImageCaptured.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/UInt8.h>
#include <visualization_msgs/Marker.h>

#include <Eigen/Dense>
#include <limits>
#include <mutex>
#include <string>

#include <terrain_navigation/profiler.h>

#include "terrain_navigation_ros/visualization.h"
#include "terrain_planner/common.h"
#include "terrain_planner/terrain_ompl_rrt.h"

#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>

#include <dynamic_reconfigure/server.h>
// #include "terrain_navigation_ros/HeightRateTuningConfig.h"

enum class PLANNER_MODE { ACTIVE_MAPPING, EMERGENCY_ABORT, EXHAUSTIVE, GLOBAL, GLOBAL_REPLANNING, RANDOM, RETURN };

enum class PLANNER_STATE { HOLD = 1, NAVIGATE = 2, ROLLOUT = 3, ABORT = 4, RETURN = 5 };

class TerrainPlanner {
 public:
  TerrainPlanner(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);
  virtual ~TerrainPlanner();
  void Init();

 private:
  void cmdloopCallback(const ros::TimerEvent &event);
  void statusloopCallback(const ros::TimerEvent &event);
  void plannerloopCallback(const ros::TimerEvent &event);
  void publishTrajectory(std::vector<Eigen::Vector3d> trajectory);
  bool publishPlannedPath(const std::vector<Eigen::Vector3d> &trajectory);
  bool mapPositionsToWorld(const std::vector<Eigen::Vector3d> &map_positions,
                           std::vector<Eigen::Vector3d> *world_positions) const;
  bool planningPositionToMap(const Eigen::Vector3d &input_position, const std::string &input_frame,
                             Eigen::Vector3d *map_position) const;
  bool publishTerrainFrameTransform();
  bool buildStartupClimbPath(const Eigen::Vector3d &start_position, const Eigen::Vector3d &start_velocity,
                             const Eigen::Vector3d &goal_position, double *target_altitude, Path *climb_path,
                             Eigen::Vector3d *climb_end_position, Eigen::Vector3d *climb_end_velocity,
                             std::string *failure_reason);
  bool refreshControllerLoiterSettings();
  bool getGoalLoiterAltitude(const Eigen::Vector2d &goal_center, double radius, double *altitude,
                             std::string *failure_reason) const;
  // States from vehicle
  void estimatorOdometryCallback(const nav_msgs::Odometry::ConstPtr &msg);
  void estimatorStateValidCallback(const std_msgs::Bool::ConstPtr &msg);
  void mavstateCallback(const mavros_msgs::State::ConstPtr &msg);
  void mavGlobalOriginCallback(const geographic_msgs::GeoPointStampedConstPtr &msg);
  void mavImageCapturedCallback(const mavros_msgs::CameraImageCaptured::ConstPtr &msg);
  void goalCallback(const geometry_msgs::PointStamped::ConstPtr &msg);
  void planCallback(const std_msgs::Empty::ConstPtr &msg);
  void executePathCallback(const std_msgs::Empty::ConstPtr &msg);
  void plannerStateCallback(const std_msgs::UInt8::ConstPtr &msg);
  void maxAltitudeCheckCallback(const std_msgs::Bool::ConstPtr &msg);

  void MapPublishOnce(const ros::Publisher &pub, const grid_map::GridMap &map);
  void publishPositionHistory(ros::Publisher &pub, const Eigen::Vector3d &position,
                              std::vector<geometry_msgs::PoseStamped> &history_vector);
  void publishPositionSetpoints(const ros::Publisher &pub, const Eigen::Vector3d &position,
                                const Eigen::Vector3d &velocity, const double curvature);
  void publishGlobalPositionSetpoints(const ros::Publisher &pub, const double latitude, const double longitude,
                                      const double altitude, const Eigen::Vector3d &velocity, const double curvature);
  void publishReferenceMarker(const ros::Publisher &pub, const Eigen::Vector3d &position,
                              const Eigen::Vector3d &velocity, const double curvature);
  void publishVelocityMarker(const ros::Publisher &pub, const Eigen::Vector3d &position,
                             const Eigen::Vector3d &velocity);
  void publishPathSetpoints(const Eigen::Vector3d &position, const Eigen::Vector3d &velocity);
  void publishPathSegments(ros::Publisher &pub, Path &trajectory);
  void publishGoal(const ros::Publisher &pub, const Eigen::Vector3d &position, const double radius,
                   Eigen::Vector3d color = Eigen::Vector3d(1.0, 1.0, 0.0), std::string name_space = "goal");
  void publishPath(const ros::Publisher &pub, Path &path);
  void publishRallyPoints(const ros::Publisher &pub, const std::vector<Eigen::Vector3d> &positions, const double radius,
                          Eigen::Vector3d color = Eigen::Vector3d(1.0, 1.0, 0.0),
                          std::string name_space = "rallypoints");
  visualization_msgs::Marker getGoalMarker(const int id, const Eigen::Vector3d &position, const double radius,
                                           const Eigen::Vector3d color);
  void generateCircle(const Eigen::Vector3d end_position, const Eigen::Vector3d end_velocity,
                      const Eigen::Vector3d center_pos, PathSegment &trajectory);
  PathSegment generateArcTrajectory(Eigen::Vector3d rates, const double horizon, Eigen::Vector3d current_pos,
                                    Eigen::Vector3d current_vel, const double dt = 0.1);
  // void dynamicReconfigureCallback(terrain_navigation_ros::HeightRateTuningConfig &config, uint32_t level);

  void printPlannerState(PLANNER_STATE state) {
    switch (state) {
      case PLANNER_STATE::HOLD:  // Fallthrough
        std::cout << "PLANNER_STATE::HOLD" << std::endl;
        break;
      case PLANNER_STATE::NAVIGATE:  // Fallthrough
        std::cout << "PLANNER_STATE::NAVIGATE" << std::endl;
        break;
      case PLANNER_STATE::ROLLOUT:
        std::cout << "PLANNER_STATE::ROLLOUT" << std::endl;
        break;
      case PLANNER_STATE::ABORT:
        std::cout << "PLANNER_STATE::ABORT" << std::endl;
        break;
    }
  }

  Eigen::Vector4d rpy2quaternion(double roll, double pitch, double yaw);

  PLANNER_STATE finiteStateMachine(const PLANNER_STATE current_state, const PLANNER_STATE query_state);

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster terrain_tf_broadcaster_;
  ros::Publisher vehicle_path_pub_;
  ros::Publisher planned_path_pub_;
  ros::Publisher grid_map_pub_;
  ros::Publisher vehicle_pose_pub_;
  ros::Publisher camera_pose_pub_;
  ros::Publisher posehistory_pub_;
  ros::Publisher referencehistory_pub_;
  ros::Publisher global_position_setpoint_pub_;
  ros::Publisher position_target_pub_;
  ros::Publisher path_target_pub_;
  ros::Publisher planner_status_pub_;
  ros::Publisher goal_pub_;
  ros::Publisher rallypoint_pub_;
  ros::Publisher candidate_goal_pub_;
  ros::Publisher candidate_start_pub_;
  ros::Publisher tree_pub_;
  ros::Publisher vehicle_velocity_pub_;
  ros::Publisher path_segment_pub_;
  ros::Publisher reference_path_pub_;
  ros::Subscriber estimator_odom_sub_;
  ros::Subscriber estimator_state_valid_sub_;
  ros::Subscriber mavstate_sub_;
  ros::Subscriber global_origin_sub_;
  ros::Subscriber goal_sub_;
  ros::Subscriber plan_sub_;
  ros::Subscriber execute_path_sub_;
  ros::Subscriber planner_state_sub_;
  ros::Subscriber max_altitude_check_sub_;
  ros::ServiceClient msginterval_serviceclient_;

  ros::Timer cmdloop_timer_, statusloop_timer_, plannerloop_timer_;
  ros::Time plan_time_;
  ros::Time last_triggered_time_;
  ros::Time last_estimator_position_time_;
  Eigen::Vector3d goal_pos_{Eigen::Vector3d(0.0, 0.0, 20.0)};
  Eigen::Vector3d start_pos_{Eigen::Vector3d(0.0, 0.0, 20.0)};
  Eigen::Vector3d home_position_{Eigen::Vector3d(0.0, 0.0, 20.0)};
  double home_position_radius_{0.0};
  Eigen::Vector3d tracking_error_{Eigen::Vector3d::Zero()};
  ros::CallbackQueue plannerloop_queue_;
  ros::CallbackQueue statusloop_queue_;
  ros::CallbackQueue cmdloop_queue_;
  std::unique_ptr<ros::AsyncSpinner> plannerloop_spinner_;
  std::unique_ptr<ros::AsyncSpinner> statusloop_spinner_;
  std::unique_ptr<ros::AsyncSpinner> cmdloop_spinner_;

  // dynamic_reconfigure::Server<terrain_navigation_ros::HeightRateTuningConfig> server;
  // dynamic_reconfigure::Server<terrain_navigation_ros::HeightRateTuningConfig>::CallbackType f;

  PLANNER_MODE planner_mode_{PLANNER_MODE::EXHAUSTIVE};
  PLANNER_STATE planner_state_{PLANNER_STATE::HOLD};
  PLANNER_STATE query_planner_state_{PLANNER_STATE::HOLD};

  std::shared_ptr<TerrainMap> terrain_map_;

  std::shared_ptr<fw_planning::spaces::DubinsAirplaneStateSpace> dubins_state_space_;
  // std::shared_ptr<ViewUtilityMap> viewutility_map_;
  std::shared_ptr<TerrainOmplRrt> global_planner_;
  std::shared_ptr<Profiler> planner_profiler_;
  Path reference_primitive_;
  Path candidate_primitive_;
  Path rollout_primitive_;
  mavros_msgs::State current_state_;
  std::optional<GeographicLib::LocalCartesian> enu_;

  std::mutex goal_mutex_;  // protects g_i

  std::vector<Eigen::Vector3d> vehicle_position_history_;
  std::vector<geometry_msgs::PoseStamped> posehistory_vector_;
  std::vector<geometry_msgs::PoseStamped> referencehistory_vector_;
  Eigen::Vector3d vehicle_position_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d vehicle_velocity_{Eigen::Vector3d::Zero()};
  Eigen::Vector4d vehicle_attitude_{Eigen::Vector4d(1.0, 0.0, 0.0, 0.0)};
  Eigen::Vector3d last_planning_position_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d previous_start_position_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d previous_return_start_position_{Eigen::Vector3d::Zero()};

  std::vector<Eigen::Vector3d> rally_points;

  // Altitude tracking loop parameters
  /// TODO: This needs to be handed over to TECS
  double K_z_{2.0};
  double cruise_speed_{20.0};
  double max_climb_rate_control_{5.0};
  double maximum_climb_angle_{0.15};

  std::string map_path_{};
  std::string map_color_path_{};
  std::string mesh_resource_path_{};
  std::string resource_path_{};
  std::string uav_name_{"uav1"};
  std::string world_frame_{"world"};
  std::string local_origin_frame_{"uav1/local_origin"};
  std::string terrain_frame_{"uav1/terrain_map"};
  std::string planning_input_frame_{"world"};
  std::string estimator_odometry_topic_{"state_estimator/main/odom"};
  std::string estimator_state_valid_topic_{"state_estimator/state_valid"};
  std::string global_origin_topic_{"mavros/global_position/gp_origin"};
  std::string global_origin_request_service_{"mavros/cmd/command"};
  double map_altitude_margin_{120.0};
  double minimum_terrain_clearance_{50.0};
  double startup_climb_minimum_clearance_{10.0};
  double minimum_turn_radius_{66.67};
  double goal_loiter_radius_{66.67};
  int goal_loiter_direction_{1};
  double global_altitude_ceiling_{std::numeric_limits<double>::quiet_NaN()};
  double local_origin_altitude_{0.0};
  double local_origin_latitude_{0.0};
  double local_origin_longitude_{0.0};
  double planner_time_budget_{30.0};
  double estimator_pose_timeout_{1.0};
  double start_loiter_radius_{66.67};
  bool local_origin_received_{false};
  bool map_initialized_{false};
  bool estimator_state_valid_{false};
  bool vehicle_position_received_{false};
  bool goal_received_{false};
  bool terrain_bounds_reported_{false};
  bool controller_loiter_settings_loaded_{false};
  bool controller_loiter_settings_warned_{false};
  bool candidate_starts_at_current_state_{false};
  Path startup_climb_path_;
  bool planner_enabled_{false};
  bool problem_updated_{true};
  bool planned_path_published_{false};
  bool found_solution_{false};
  bool found_return_solution_{false};
};

#endif
