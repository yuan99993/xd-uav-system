/****************************************************************************
 *
 *   Copyright (c) 2023 Jaeyoung Lim, Autonomous Systems Lab,
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
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,R
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
/**
 * @brief Terrain Planner
 *
 * Terrain Planner
 *
 * @author Jaeyoung Lim <jalim@ethz.ch>
 */

#include "terrain_navigation_ros/terrain_planner.h"
#include "terrain_navigation_ros/geo_conversions.h"
#include "terrain_navigation_ros/visualization.h"

#include <grid_map_msgs/GridMap.h>
#include <mavros_msgs/CommandCode.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/Trajectory.h>
#include <planner_msgs/NavigationStatus.h>
#include <planner_msgs/Path.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/MarkerArray.h>
#include <grid_map_ros/GridMapRosConverter.hpp>

#include <limits>

TerrainPlanner::TerrainPlanner(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private)
    : nh_(nh), nh_private_(nh_private), tf_listener_(tf_buffer_) {
  nh_private_.param<std::string>("estimator_odometry_topic", estimator_odometry_topic_,
                                 "state_estimator/main/odom");
  nh_private_.param<std::string>("estimator_state_valid_topic", estimator_state_valid_topic_,
                                 "state_estimator/state_valid");
  nh_private_.param<std::string>("global_origin_topic", global_origin_topic_,
                                 "mavros/global_position/gp_origin");
  nh_private_.param<std::string>("global_origin_request_service", global_origin_request_service_,
                                 "mavros/cmd/command");
  nh_private_.param<double>("estimator_pose_timeout", estimator_pose_timeout_, 1.0);
  if (!std::isfinite(estimator_pose_timeout_) || estimator_pose_timeout_ <= 0.0) {
    ROS_WARN("[TerrainPlanner] Invalid estimator_pose_timeout; using 1.0 second");
    estimator_pose_timeout_ = 1.0;
  }

  vehicle_path_pub_ = nh_.advertise<nav_msgs::Path>("vehicle_path", 1);
  planned_path_pub_ = nh_.advertise<nav_msgs::Path>("planned_path", 1, true);

  grid_map_pub_ = nh_.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);

  posehistory_pub_ = nh_.advertise<nav_msgs::Path>("geometric_controller/path", 10);
  referencehistory_pub_ = nh_.advertise<nav_msgs::Path>("reference/path", 10);
  position_target_pub_ = nh_.advertise<visualization_msgs::Marker>("position_target", 1);
  vehicle_velocity_pub_ = nh_.advertise<visualization_msgs::Marker>("vehicle_velocity", 1);
  goal_pub_ = nh_.advertise<visualization_msgs::Marker>("goal_marker", 1);
  rallypoint_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("rallypoints_marker", 1);
  candidate_goal_pub_ = nh_.advertise<visualization_msgs::Marker>("candidate_goal_marker", 1);
  candidate_start_pub_ = nh_.advertise<visualization_msgs::Marker>("candidate_start_marker", 1);
  mavstate_sub_ =
      nh_.subscribe("mavros/state", 1, &TerrainPlanner::mavstateCallback, this, ros::TransportHints().tcpNoDelay());
  global_position_setpoint_pub_ = nh_.advertise<mavros_msgs::GlobalPositionTarget>("mavros/setpoint_raw/global", 1);
  path_target_pub_ = nh_.advertise<mavros_msgs::Trajectory>("mavros/trajectory/generated", 1);
  vehicle_pose_pub_ = nh_.advertise<visualization_msgs::Marker>("vehicle_pose_marker", 1);
  camera_pose_pub_ = nh_.advertise<visualization_msgs::Marker>("camera_pose_marker", 1);
  planner_status_pub_ = nh_.advertise<planner_msgs::NavigationStatus>("planner_status", 1);
  path_segment_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("path_segments", 1);
  tree_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("tree", 1);
  reference_path_pub_ = nh_.advertise<planner_msgs::Path>("reference_path", 1);

  estimator_odom_sub_ = nh_.subscribe(estimator_odometry_topic_, 1,
                                      &TerrainPlanner::estimatorOdometryCallback, this,
                                      ros::TransportHints().tcpNoDelay());
  estimator_state_valid_sub_ = nh_.subscribe(estimator_state_valid_topic_, 1,
                                             &TerrainPlanner::estimatorStateValidCallback, this,
                                             ros::TransportHints().tcpNoDelay());
  global_origin_sub_ = nh_.subscribe(global_origin_topic_, 1, &TerrainPlanner::mavGlobalOriginCallback,
                                     this, ros::TransportHints().tcpNoDelay());

  goal_sub_ = nh_.subscribe("terrain_planner/goal", 1, &TerrainPlanner::goalCallback, this);
  plan_sub_ = nh_.subscribe("terrain_planner/plan", 1, &TerrainPlanner::planCallback, this);
  execute_path_sub_ = nh_.subscribe("terrain_planner/execute_path", 1, &TerrainPlanner::executePathCallback, this);
  planner_state_sub_ = nh_.subscribe("terrain_planner/planner_state", 1, &TerrainPlanner::plannerStateCallback, this);
  max_altitude_check_sub_ =
      nh_.subscribe("terrain_planner/check_max_altitude", 1, &TerrainPlanner::maxAltitudeCheckCallback, this);
  msginterval_serviceclient_ = nh_.serviceClient<mavros_msgs::CommandLong>(global_origin_request_service_);

  std::string avalanche_map_path;
  nh_private.param<std::string>("terrain_path", map_path_, "resources/cadastre.tif");
  nh_private.param<std::string>("terrain_color_path", map_color_path_, "");
  nh_private.param<std::string>("resource_path", resource_path_, "resources");
  nh_private.param<std::string>("meshresource_path", mesh_resource_path_, "../resources/believer.dae");
  nh_private.param<std::string>("avalanche_map_path", avalanche_map_path, "../data/believer.dae");
  nh_private.param<double>("minimum_turn_radius", minimum_turn_radius_, 66.67);
  nh_private.param<double>("minimum_terrain_clearance", minimum_terrain_clearance_, 50.0);
  nh_private.param<double>("map_altitude_margin", map_altitude_margin_, 120.0);
  nh_private.param<double>("startup_climb_minimum_clearance", startup_climb_minimum_clearance_, 10.0);
  nh_private.param<double>("cruise_speed", cruise_speed_, 20.0);
  nh_private.param<double>("maximum_climb_rate", max_climb_rate_control_, 5.0);
  nh_private.param<double>("maximum_climb_angle", maximum_climb_angle_, 0.15);
  if (!std::isfinite(minimum_terrain_clearance_) || !std::isfinite(map_altitude_margin_) ||
      !std::isfinite(startup_climb_minimum_clearance_) || !std::isfinite(minimum_turn_radius_) ||
      minimum_terrain_clearance_ < 0.0 || map_altitude_margin_ < 0.0 ||
      startup_climb_minimum_clearance_ < 0.0 || minimum_turn_radius_ <= 0.0) {
    ROS_WARN("[TerrainPlanner] Invalid terrain altitude settings; using 50 m clearance, 120 m map margin, and 10 m startup clearance");
    minimum_terrain_clearance_ = 50.0;
    map_altitude_margin_ = 120.0;
    startup_climb_minimum_clearance_ = 10.0;
  }
  goal_loiter_radius_ = minimum_turn_radius_;
  if (!std::isfinite(cruise_speed_) || cruise_speed_ <= 0.0) cruise_speed_ = 20.0;
  if (!std::isfinite(max_climb_rate_control_) || max_climb_rate_control_ <= 0.0) max_climb_rate_control_ = 5.0;
  if (!std::isfinite(maximum_climb_angle_) || maximum_climb_angle_ <= 0.0 || maximum_climb_angle_ >= M_PI_2) {
    maximum_climb_angle_ = 0.15;
  }
  nh_private.param<double>("planner_time_budget", planner_time_budget_, 30.0);
  bool check_max_altitude = false;
  nh_private.param<bool>("check_max_altitude", check_max_altitude, false);

  std::string default_uav_name = nh_.getNamespace();
  if (!default_uav_name.empty() && default_uav_name.front() == '/') default_uav_name.erase(0, 1);
  if (default_uav_name.empty()) default_uav_name = "uav1";
  nh_private.param<std::string>("uav_name", uav_name_, default_uav_name);
  nh_private.param<std::string>("world_frame", world_frame_, "world");
  nh_private.param<std::string>("local_origin_frame", local_origin_frame_, uav_name_ + "/local_origin");
  nh_private.param<std::string>("terrain_frame", terrain_frame_, uav_name_ + "/terrain_map");
  nh_private.param<std::string>("planning_input_frame", planning_input_frame_, world_frame_);
  auto trim_frame = [](std::string& frame) {
    while (!frame.empty() && frame.front() == '/') frame.erase(0, 1);
  };
  trim_frame(world_frame_);
  trim_frame(local_origin_frame_);
  trim_frame(terrain_frame_);
  trim_frame(planning_input_frame_);


  terrain_map_ = std::make_shared<TerrainMap>(terrain_frame_, false);

  // Initialize Dubins state space
  dubins_state_space_ = std::make_shared<fw_planning::spaces::DubinsAirplaneStateSpace>(minimum_turn_radius_);
  global_planner_ = std::make_shared<TerrainOmplRrt>(ompl::base::StateSpacePtr(dubins_state_space_));
  global_planner_->setMap(terrain_map_);
  global_planner_->setMaxAltitudeCollisionChecks(check_max_altitude);

  // f = boost::bind(&TerrainPlanner::dynamicReconfigureCallback, this, _1, _2);
  // server.setCallback(f);

  planner_profiler_ = std::make_shared<Profiler>("planner");
}
TerrainPlanner::~TerrainPlanner() {
  // Destructor
}

void TerrainPlanner::Init() {
  double plannerloop_dt_ = 2.0;
  ros::TimerOptions plannerlooptimer_options(
      ros::Duration(plannerloop_dt_), boost::bind(&TerrainPlanner::plannerloopCallback, this, _1), &plannerloop_queue_);
  plannerloop_timer_ = nh_.createTimer(plannerlooptimer_options);  // Define timer for constant loop rate

  plannerloop_spinner_.reset(new ros::AsyncSpinner(1, &plannerloop_queue_));
  plannerloop_spinner_->start();

  double statusloop_dt_ = 0.5;
  ros::TimerOptions statuslooptimer_options(
      ros::Duration(statusloop_dt_), boost::bind(&TerrainPlanner::statusloopCallback, this, _1), &statusloop_queue_);
  statusloop_timer_ = nh_.createTimer(statuslooptimer_options);  // Define timer for constant loop rate

  statusloop_spinner_.reset(new ros::AsyncSpinner(1, &statusloop_queue_));
  statusloop_spinner_->start();
  double cmdloop_dt_ = 0.1;
  ros::TimerOptions cmdlooptimer_options(ros::Duration(cmdloop_dt_),
                                         boost::bind(&TerrainPlanner::cmdloopCallback, this, _1), &cmdloop_queue_);
  cmdloop_timer_ = nh_.createTimer(cmdlooptimer_options);  // Define timer for constant loop rate

  cmdloop_spinner_.reset(new ros::AsyncSpinner(1, &cmdloop_queue_));
  cmdloop_spinner_->start();
}

void TerrainPlanner::cmdloopCallback(const ros::TimerEvent &event) {
  if (!map_initialized_) return;

  if (!reference_primitive_.segments.empty()) {
    Eigen::Vector3d reference_position;
    Eigen::Vector3d reference_tangent;
    double reference_curvature{0.0};
    auto current_segment = reference_primitive_.getCurrentSegment(vehicle_position_);
    double path_progress = current_segment.getClosestPoint(vehicle_position_, reference_position, reference_tangent,
                                                           reference_curvature, 1.0);
    // Publish global position setpoints in the global frame
    EPSG map_coordinate;
    Eigen::Vector3d map_origin;
    terrain_map_->getGlobalOrigin(map_coordinate, map_origin);
    /// TODO: convert reference position to global
    const Eigen::Vector3d lv03_reference_position = reference_position + map_origin;
    Eigen::Vector3d wgs84_reference_position =
        GeoConversions::transformCoordinates(EPSG::CH1903_LV03, EPSG::WGS84, lv03_reference_position);

    double latitude = wgs84_reference_position(0);
    double longitude = wgs84_reference_position(1);
    double altitude = wgs84_reference_position(2);

    publishReferenceMarker(position_target_pub_, reference_position, reference_tangent, reference_curvature);

    // Run additional altitude control
    double altitude_correction = K_z_ * (vehicle_position_(2) - reference_position(2));
    double climb_rate = cruise_speed_ * std::sin(current_segment.flightpath_angle);
    Eigen::Vector3d velocity_reference = reference_tangent;

    velocity_reference(2) =
        std::min(std::max(altitude_correction - climb_rate, -max_climb_rate_control_), max_climb_rate_control_);

    /// Blend curvature with next segment
    double curvature_reference = reference_curvature;
    int current_segment_idx = reference_primitive_.getCurrentSegmentIndex(vehicle_position_);
    bool is_last_segment = bool(current_segment_idx >= (reference_primitive_.segments.size() - 1));
    if (!is_last_segment) {
      /// Get next segment curvature
      double next_segment_curvature = reference_primitive_.segments[current_segment_idx + 1].curvature;

      /// Blend current curvature with next curvature when close to the end
      double segment_length = current_segment.getLength();
      double cut_off_distance = 10.0;
      double portion = std::min(
          1.0, std::max((path_progress * segment_length - segment_length + cut_off_distance) / cut_off_distance, 0.0));
      curvature_reference = (1 - portion) * reference_curvature + portion * next_segment_curvature;
    }

    publishGlobalPositionSetpoints(global_position_setpoint_pub_, latitude, longitude, altitude, velocity_reference,
                                   curvature_reference);

    /// TODO: Switch mode to planner engaged
    if (current_state_.mode == "OFFBOARD") {
      publishPositionHistory(referencehistory_pub_, reference_position, referencehistory_vector_);
      tracking_error_ = reference_position - vehicle_position_;
      planner_enabled_ = true;
    } else {
      tracking_error_ = Eigen::Vector3d::Zero();
      planner_enabled_ = false;
    }
  }

  planner_msgs::NavigationStatus msg;
  msg.header.stamp = ros::Time::now();
  // msg.planner_time.data = planner_time;
  msg.state = static_cast<uint8_t>(planner_state_);
  msg.tracking_error = toVector3(tracking_error_);
  msg.enabled = planner_enabled_;
  // msg.reference_position = toVector3(reference_position);
  msg.vehicle_position = toVector3(vehicle_position_);
  planner_status_pub_.publish(msg);

  publishVehiclePose(vehicle_pose_pub_, vehicle_position_, vehicle_attitude_, mesh_resource_path_, terrain_frame_);
  publishVelocityMarker(vehicle_velocity_pub_, vehicle_position_, vehicle_velocity_);
  publishPositionHistory(posehistory_pub_, vehicle_position_, posehistory_vector_);
}

Eigen::Vector4d TerrainPlanner::rpy2quaternion(double roll, double pitch, double yaw) {
  double cy = std::cos(yaw * 0.5);
  double sy = std::sin(yaw * 0.5);
  double cp = std::cos(pitch * 0.5);
  double sp = std::sin(pitch * 0.5);
  double cr = std::cos(roll * 0.5);
  double sr = std::sin(roll * 0.5);

  Eigen::Vector4d q;
  q(0) = cr * cp * cy + sr * sp * sy;
  q(1) = sr * cp * cy - cr * sp * sy;
  q(2) = cr * sp * cy + sr * cp * sy;
  q(3) = cr * cp * sy - sr * sp * cy;

  q.normalize();

  return q;
}

void TerrainPlanner::statusloopCallback(const ros::TimerEvent &event) {
  // Check if we want to update the planner state if query state and current state is different
  planner_state_ = finiteStateMachine(planner_state_, query_planner_state_);
  if (query_planner_state_ != planner_state_) {  // Query has been rejected, reset
    query_planner_state_ = planner_state_;
  }
  // printPlannerState(planner_state_);
}

void TerrainPlanner::plannerloopCallback(const ros::TimerEvent &event) {
  const std::lock_guard<std::mutex> lock(goal_mutex_);
  refreshControllerLoiterSettings();
  if (map_initialized_) publishTerrainFrameTransform();
  if (local_origin_received_ && !map_initialized_) {
    std::cout << "[TerrainPlanner] Local origin received, loading map" << std::endl;
    const bool map_loaded = terrain_map_->Load(map_path_, map_color_path_);
    if (!map_loaded) {
      std::cout << "[TerrainPlanner]   - Failed to load map: " << map_path_ << std::endl;
      return;
    }

    terrain_map_->AddLayerDistanceTransform(minimum_terrain_clearance_, "distance_surface");
    terrain_map_->AddLayerHorizontalDistanceTransform(minimum_turn_radius_, "minimum_safe_altitude", "distance_surface");

    auto &map_data = terrain_map_->getGridMap();
    double highest_dem_elevation = -std::numeric_limits<double>::infinity();
    for (grid_map::GridMapIterator iterator(map_data); !iterator.isPastEnd(); ++iterator) {
      const double elevation = map_data.at("elevation", *iterator);
      if (std::isfinite(elevation)) highest_dem_elevation = std::max(highest_dem_elevation, elevation);
    }
    if (!std::isfinite(highest_dem_elevation)) {
      ROS_ERROR("[TerrainPlanner] Loaded DEM contains no finite elevation values");
      return;
    }
    global_altitude_ceiling_ = highest_dem_elevation + map_altitude_margin_;
    map_data.add("global_altitude_ceiling");
    map_data["global_altitude_ceiling"].setConstant(global_altitude_ceiling_);
    map_initialized_ = true;

    EPSG map_coordinate;
    Eigen::Vector3d map_origin;
    terrain_map_->getGlobalOrigin(map_coordinate, map_origin);

    if (map_initialized_) {
      std::cout << "[TerrainPlanner]   - Successfully loaded map: " << map_path_ << std::endl;
      ROS_INFO("[TerrainPlanner] Altitude layers: terrain clearance %.1f m, startup clearance %.1f m, "
               "map ceiling %.2f m (highest DEM %.2f + margin %.1f m)",
               minimum_terrain_clearance_, startup_climb_minimum_clearance_, global_altitude_ceiling_,
               highest_dem_elevation, map_altitude_margin_);
      publishTerrainFrameTransform();
      MapPublishOnce(grid_map_pub_, terrain_map_->getGridMap());
      global_planner_->setBoundsFromMap(terrain_map_->getGridMap());
    } else {
      std::cout << "[TerrainPlanner]   - Failed to load map: " << map_path_ << std::endl;
    }
    return;
  }
  if (!local_origin_received_) {
    std::cout << "Requesting global origin messages" << std::endl;
    mavros_msgs::CommandLong request_global_origin_msg;
    request_global_origin_msg.request.command = mavros_msgs::CommandCode::REQUEST_MESSAGE;
    request_global_origin_msg.request.param1 = 49;
    msginterval_serviceclient_.call(request_global_origin_msg);
    return;
  }

  // planner_profiler_->tic();

  switch (planner_mode_) {
    case PLANNER_MODE::GLOBAL: {
      // Solve planning problem with RRT*
      double time_spent_planning = ros::Duration(ros::Time::now() - plan_time_).toSec();
      if (time_spent_planning < planner_time_budget_) {
        bool found_solution = global_planner_->Solve(1.0, candidate_primitive_);
        publishTree(tree_pub_, global_planner_->getPlannerData(), global_planner_->getProblemSetup(), terrain_frame_);
      } else {
        if (!startup_climb_path_.segments.empty() && !candidate_primitive_.segments.empty()) {
          const Eigen::Vector3d climb_end = startup_climb_path_.lastSegment().states.back().position;
          const Eigen::Vector3d route_start = candidate_primitive_.firstSegment().states.front().position;
          if ((climb_end - route_start).norm() < 0.5) {
            Path combined_path;
            combined_path.appendSegment(startup_climb_path_);
            combined_path.appendSegment(candidate_primitive_);
            candidate_primitive_ = combined_path;
            startup_climb_path_.segments.clear();
          } else {
            ROS_ERROR("[TerrainPlanner] Startup climb does not connect to the planned route (gap %.2f m)",
                      (climb_end - route_start).norm());
            candidate_primitive_.segments.clear();
            startup_climb_path_.segments.clear();
          }
        }
        publishPathSegments(path_segment_pub_, candidate_primitive_);
        if (!planned_path_published_) {
          if (!candidate_primitive_.segments.empty()) {
            const auto route_positions = candidate_primitive_.position();
            if (publishPlannedPath(route_positions)) {
              ROS_INFO("[TerrainPlanner] Published global route with %zu poses on planned_path",
                       route_positions.size());
              planned_path_published_ = true;
            } else {
              ROS_WARN_THROTTLE(2.0, "[TerrainPlanner] Waiting for world/local TF before publishing global route");
            }
          } else {
            ROS_WARN("[TerrainPlanner] Planning budget expired without a global route");
            planned_path_published_ = true;
          }
        }
      }
      break;
    }
    case PLANNER_MODE::EMERGENCY_ABORT: {
      // Solve planning problem with RRT*
      if (!reference_primitive_.segments.empty()) {
        PathSegment current_segment = reference_primitive_.getCurrentSegment(vehicle_position_);
        Eigen::Vector3d start_position = current_segment.states.back().position;
        Eigen::Vector3d start_velocity = current_segment.states.back().velocity;

        if ((start_position != previous_start_position_ && !found_solution_)) {
          std::cout << "Start position changed! Updating problem" << std::endl;
          problem_updated_ = true;
        }

        /// Only update the problem when the goal is updated
        if (problem_updated_) {
          /// Generate candidate rally points
          const int num_rally_points = 3;
          rally_points.clear();
          for (int i = 0; i < num_rally_points; i++) {
            bool sample_is_valid = false;
            while (!sample_is_valid) {
              Eigen::Vector3d random_sample;
              random_sample(0) = getRandom(-200.0, 200.0);
              random_sample(1) = getRandom(-200.0, 200.0);
              Eigen::Vector3d candidate_loiter_position = start_position + random_sample;
              Eigen::Vector3d new_loiter_position;
              sample_is_valid =
                  validatePosition(terrain_map_->getGridMap(), candidate_loiter_position, new_loiter_position);
              if (sample_is_valid) {
                rally_points.push_back(new_loiter_position);
              }
            }
          }

          global_planner_->setupProblem(start_position, start_velocity, rally_points);
          /// Publish Rally points
          publishRallyPoints(rallypoint_pub_, rally_points, 66.67, Eigen::Vector3d(1.0, 1.0, 0.0));
          previous_start_position_ = start_position;
        }

        Path planner_solution_path;
        bool found_solution = global_planner_->Solve(0.5, planner_solution_path);
        found_solution_ = found_solution;

        // If a solution is found, check if the new solution is better than the previous solution
        if (found_solution_ && problem_updated_) {
          problem_updated_ = false;

          bool update_solution{false};
          update_solution = planner_solution_path.segments.empty() ? false : true;

          // If a better solution is found, update the path
          if (update_solution) {
            std::cout << "  - Updating solution" << std::endl;
            Path updated_segment;
            updated_segment.segments.clear();
            updated_segment.appendSegment(current_segment);
            updated_segment.appendSegment(planner_solution_path);

            Eigen::Vector3d end_position = planner_solution_path.lastSegment().states.back().position;
            Eigen::Vector3d end_velocity = planner_solution_path.lastSegment().states.back().velocity;
            /// TODO: Figure out which rally point the planner is using
            double min_distance_error = std::numeric_limits<double>::infinity();
            int min_distance_index = -1;
            for (int idx = 0; idx < rally_points.size(); idx++) {
              double radial_error =
                  std::abs((end_position - rally_points[idx]).norm() - dubins_state_space_->getMinTurningRadius());
              if (radial_error < min_distance_error) {
                min_distance_index = idx;
                min_distance_error = radial_error;
              }
            }
            Eigen::Vector3d radial_vector = (end_position - rally_points[min_distance_index]);
            radial_vector(2) = 0.0;  // Only consider horizontal loiters
            Eigen::Vector3d emergency_rates =
                20.0 * end_velocity.normalized().cross(radial_vector.normalized()) / radial_vector.norm();
            double horizon = 2 * M_PI / std::abs(emergency_rates(2));
            // Append a loiter at the end of the planned path
            PathSegment loiter_trajectory;
            generateCircle(end_position, end_velocity, rally_points[min_distance_index], loiter_trajectory);
            updated_segment.appendSegment(loiter_trajectory);
            reference_primitive_ = updated_segment;
            candidate_primitive_ = updated_segment;
          }
        }
        publishTree(tree_pub_, global_planner_->getPlannerData(), global_planner_->getProblemSetup(), terrain_frame_);
      }
      publishPathSegments(path_segment_pub_, candidate_primitive_);
      break;
    }
    case PLANNER_MODE::RETURN: {
      // Solve planning problem with RRT*
      if (!reference_primitive_.segments.empty()) {
        PathSegment current_segment = reference_primitive_.getCurrentSegment(vehicle_position_);
        Eigen::Vector3d start_position = current_segment.states.back().position;
        Eigen::Vector3d start_velocity = current_segment.states.back().velocity;

        if ((start_position != previous_return_start_position_ && !found_return_solution_)) {
          std::cout << "Start position changed! Updating problem" << std::endl;
          problem_updated_ = true;
        }

        /// Only update the problem when the goal is updated
        if (problem_updated_) {
          global_planner_->setupProblem(start_position, start_velocity, home_position_, home_position_radius_);
          previous_return_start_position_ = start_position;
        }

        Path planner_solution_path;
        bool found_solution = global_planner_->Solve(0.5, planner_solution_path);
        found_return_solution_ = found_solution;

        // If a solution is found, check if the new solution is better than the previous solution
        if (found_return_solution_ && problem_updated_) {
          problem_updated_ = false;

          bool update_solution{false};
          update_solution = planner_solution_path.segments.empty() ? false : true;

          // If a better solution is found, update the path
          if (update_solution) {
            std::cout << "  - Updating solution" << std::endl;
            Path updated_segment;
            updated_segment.segments.clear();
            updated_segment.appendSegment(current_segment);
            updated_segment.appendSegment(planner_solution_path);

            Eigen::Vector3d end_position = planner_solution_path.lastSegment().states.back().position;
            Eigen::Vector3d end_velocity = planner_solution_path.lastSegment().states.back().velocity;

            Eigen::Vector3d radial_vector = (end_position - home_position_);
            radial_vector(2) = 0.0;  // Only consider horizontal loiters
            Eigen::Vector3d emergency_rates =
                20.0 * end_velocity.normalized().cross(radial_vector.normalized()) / radial_vector.norm();
            double horizon = 2 * M_PI / std::abs(emergency_rates(2));
            // Append a loiter at the end of the planned path
            PathSegment loiter_trajectory;
            generateCircle(end_position, end_velocity, home_position_, loiter_trajectory);
            updated_segment.appendSegment(loiter_trajectory);
            reference_primitive_ = updated_segment;
            candidate_primitive_ = updated_segment;
          }
        }
        publishTree(tree_pub_, global_planner_->getPlannerData(), global_planner_->getProblemSetup(), terrain_frame_);
      }
      publishPathSegments(path_segment_pub_, candidate_primitive_);
      break;
    }
  }

  // double planner_time = planner_profiler_->toc();
  publishTrajectory(reference_primitive_.position());
  publishPath(reference_path_pub_, reference_primitive_);
  // publishGoal(goal_pub_, goal_pos_, 66.67, Eigen::Vector3d(0.0, 1.0, 0.0));
}

PLANNER_STATE TerrainPlanner::finiteStateMachine(const PLANNER_STATE current_state, const PLANNER_STATE query_state) {
  PLANNER_STATE next_state;
  next_state = current_state;
  switch (current_state) {
    case PLANNER_STATE::NAVIGATE: {
      // Switch to Hold when segment has been completed
      int current_segment_idx = reference_primitive_.getCurrentSegmentIndex(vehicle_position_);
      bool is_last_segment = bool(current_segment_idx >= (reference_primitive_.segments.size() - 1));
      if (is_last_segment) {
        /// TODO: Clear candidate primitive
        candidate_primitive_.segments.clear();
        candidate_starts_at_current_state_ = false;
        next_state = PLANNER_STATE::HOLD;
      }

      // Stay in hold mode if the current segment is periodic, Otherwise switch to abort
      if (query_state == PLANNER_STATE::ABORT) {
        if (!reference_primitive_.getCurrentSegment(vehicle_position_).is_periodic) {
          next_state = PLANNER_STATE::ABORT;
          planner_mode_ = PLANNER_MODE::EMERGENCY_ABORT;
        } else {
          /// TODO: Get rid of next segments and only keep current segment
        }
      } else if (query_state == PLANNER_STATE::RETURN) {
        next_state = PLANNER_STATE::RETURN;
        planner_mode_ = PLANNER_MODE::RETURN;
      }
      break;
    }
    case PLANNER_STATE::ROLLOUT: {
      /// TODO: Get rollout primitive from active mapper
      // if (candidate_primitive_.valid()) {
      // Update reference if candidate is valid
      // reference_primitive_ = candidate_primitive_;
      // std::cout << "Candidate primitive is valid" << std::endl;
      // }
      reference_primitive_ = rollout_primitive_;
      /// TODO: Add self termination
      if (query_state == PLANNER_STATE::ABORT) {
        if (reference_primitive_.getCurrentSegment(vehicle_position_).is_periodic) {
          next_state = PLANNER_STATE::HOLD;
          planner_mode_ = PLANNER_MODE::GLOBAL;
        } else {
          next_state = query_state;
          planner_mode_ = PLANNER_MODE::EMERGENCY_ABORT;
        }
      }
      break;
    }
    case PLANNER_STATE::HOLD: {
      switch (query_state) {
        case PLANNER_STATE::NAVIGATE: {
          // Check if the candidate primitive is not empty
          if (!candidate_primitive_.segments.empty()) {
            if (!candidate_starts_at_current_state_) {
              // Legacy planners can provide a start circle; an exact-state global plan already
              // starts at the aircraft and must not be redirected through a synthetic loiter.
              Eigen::Vector3d start_position = candidate_primitive_.firstSegment().states.front().position;
              Eigen::Vector3d start_velocity = candidate_primitive_.firstSegment().states.front().velocity;
              PathSegment start_loiter;
              generateCircle(start_position, start_velocity, start_pos_, start_loiter);
              candidate_primitive_.prependSegment(start_loiter);
            }

            // Add terminal loiter
            Eigen::Vector3d end_position = candidate_primitive_.lastSegment().states.back().position;
            Eigen::Vector3d end_velocity = candidate_primitive_.lastSegment().states.back().velocity;
            PathSegment terminal_loiter;
            generateCircle(end_position, end_velocity, goal_pos_, terminal_loiter);
            candidate_primitive_.appendSegment(terminal_loiter);

            reference_primitive_ = candidate_primitive_;
            next_state = query_state;
          }
          break;
        }
        case PLANNER_STATE::ROLLOUT: {
          planner_mode_ = PLANNER_MODE::ACTIVE_MAPPING;
          next_state = query_state;
          break;
        }
      }
      break;
    }
    case PLANNER_STATE::RETURN: {
      /// TODO: Check if we have a return position defined
      // Switch to Hold when segment has been completed
      int current_segment_idx = reference_primitive_.getCurrentSegmentIndex(vehicle_position_);
      bool is_last_segment = bool(current_segment_idx >= (reference_primitive_.segments.size() - 1));
      if (is_last_segment) {
        /// TODO: Clear candidate primitive
        candidate_primitive_.segments.clear();
        next_state = PLANNER_STATE::HOLD;
        found_return_solution_ = false;
        planner_mode_ = PLANNER_MODE::GLOBAL;
      }

      if (query_state == PLANNER_STATE::ABORT) {
        if (!reference_primitive_.getCurrentSegment(vehicle_position_).is_periodic) {
          next_state = PLANNER_STATE::ABORT;
          planner_mode_ = PLANNER_MODE::EMERGENCY_ABORT;
        }
      }
      break;
    }
    case PLANNER_STATE::ABORT: {
      int current_segment_idx = reference_primitive_.getCurrentSegmentIndex(vehicle_position_);
      bool is_last_segment = bool(current_segment_idx >= (reference_primitive_.segments.size() - 1));
      if (is_last_segment) {
        /// TODO: Clear candidate primitive
        candidate_primitive_.segments.clear();
        planner_mode_ = PLANNER_MODE::GLOBAL;
        found_solution_ = false;
        next_state = PLANNER_STATE::HOLD;
      }
    }
  }
  return next_state;
}

void TerrainPlanner::publishTrajectory(std::vector<Eigen::Vector3d> trajectory) {
  nav_msgs::Path msg;
  std::vector<geometry_msgs::PoseStamped> posestampedhistory_vector;
  Eigen::Vector4d orientation(1.0, 0.0, 0.0, 0.0);
  for (auto pos : trajectory) {
    auto pose = vector3d2PoseStampedMsg(pos, orientation);
    pose.header.frame_id = terrain_frame_;
    posestampedhistory_vector.insert(posestampedhistory_vector.begin(), pose);
  }
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = terrain_frame_;
  msg.poses = posestampedhistory_vector;
  vehicle_path_pub_.publish(msg);
}

bool TerrainPlanner::publishPlannedPath(const std::vector<Eigen::Vector3d> &trajectory) {
  std::vector<Eigen::Vector3d> world_positions;
  if (!mapPositionsToWorld(trajectory, &world_positions)) {
    ROS_WARN_THROTTLE(2.0, "[TerrainPlanner] Planned route not published: map-to-world conversion is unavailable");
    return false;
  }

  nav_msgs::Path msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = world_frame_;
  msg.poses.reserve(world_positions.size());
  for (const auto &position : world_positions) {
    geometry_msgs::PoseStamped pose;
    pose.header = msg.header;
    pose.pose.position.x = position.x();
    pose.pose.position.y = position.y();
    pose.pose.position.z = position.z();
    pose.pose.orientation.w = 1.0;
    msg.poses.push_back(pose);
  }
  planned_path_pub_.publish(msg);
  return true;
}

bool TerrainPlanner::mapPositionsToWorld(const std::vector<Eigen::Vector3d> &map_positions,
                                          std::vector<Eigen::Vector3d> *world_positions) const {
  if (world_positions == nullptr || !enu_) return false;

  EPSG map_coordinate;
  Eigen::Vector3d map_origin;
  terrain_map_->getGlobalOrigin(map_coordinate, map_origin);
  if (map_coordinate == EPSG::WGS84) {
    map_origin = GeoConversions::transformCoordinates(EPSG::WGS84, EPSG::CH1903_LV03, map_origin);
  }

  geometry_msgs::TransformStamped world_from_local_msg;
  try {
    world_from_local_msg = tf_buffer_.lookupTransform(
        world_frame_, local_origin_frame_, ros::Time(0), ros::Duration(0.2));
  } catch (const tf2::TransformException &error) {
    ROS_WARN_THROTTLE(2.0, "[TerrainPlanner] Waiting for TF %s -> %s: %s",
                      world_frame_.c_str(), local_origin_frame_.c_str(), error.what());
    return false;
  }

  tf2::Transform world_from_local;
  tf2::fromMsg(world_from_local_msg.transform, world_from_local);
  world_positions->clear();
  world_positions->reserve(map_positions.size());
  for (const auto &map_position : map_positions) {
    const Eigen::Vector3d lv03_position = map_origin + map_position;
    const Eigen::Vector3d wgs84_position =
        GeoConversions::transformCoordinates(EPSG::CH1903_LV03, EPSG::WGS84, lv03_position);
    double east = 0.0;
    double north = 0.0;
    double up = 0.0;
    enu_->Forward(wgs84_position.x(), wgs84_position.y(), wgs84_position.z(), east, north, up);
    const tf2::Vector3 world_position = world_from_local * tf2::Vector3(east, north, up);
    world_positions->emplace_back(world_position.x(), world_position.y(), world_position.z());
  }
  return true;
}

bool TerrainPlanner::planningPositionToMap(const Eigen::Vector3d &input_position, const std::string &input_frame,
                                           Eigen::Vector3d *map_position) const {
  if (map_position == nullptr || !map_initialized_ || !enu_) return false;
  if (input_frame == terrain_frame_) {
    *map_position = input_position;
    return true;
  }

  geometry_msgs::TransformStamped local_from_input_msg;
  try {
    local_from_input_msg = tf_buffer_.lookupTransform(
        local_origin_frame_, input_frame, ros::Time(0), ros::Duration(0.2));
  } catch (const tf2::TransformException &error) {
    ROS_WARN_THROTTLE(2.0, "[TerrainPlanner] Cannot transform planning input from %s to %s: %s",
                      input_frame.c_str(), local_origin_frame_.c_str(), error.what());
    return false;
  }

  tf2::Transform local_from_input;
  tf2::fromMsg(local_from_input_msg.transform, local_from_input);
  const tf2::Vector3 local_position =
      local_from_input * tf2::Vector3(input_position.x(), input_position.y(), input_position.z());

  double latitude = 0.0;
  double longitude = 0.0;
  double altitude = 0.0;
  enu_->Reverse(local_position.x(), local_position.y(), local_position.z(),
                latitude, longitude, altitude);
  Eigen::Vector3d lv03_position = GeoConversions::transformCoordinates(
      EPSG::WGS84, EPSG::CH1903_LV03, Eigen::Vector3d(latitude, longitude, altitude));

  EPSG map_coordinate;
  Eigen::Vector3d map_origin;
  terrain_map_->getGlobalOrigin(map_coordinate, map_origin);
  if (map_coordinate == EPSG::WGS84) {
    map_origin = GeoConversions::transformCoordinates(EPSG::WGS84, EPSG::CH1903_LV03, map_origin);
  }
  *map_position = lv03_position - map_origin;
  return true;
}

bool TerrainPlanner::publishTerrainFrameTransform() {
  if (!enu_) return false;

  EPSG map_coordinate;
  Eigen::Vector3d map_origin;
  terrain_map_->getGlobalOrigin(map_coordinate, map_origin);
  if (map_coordinate == EPSG::WGS84) {
    map_origin = GeoConversions::transformCoordinates(EPSG::WGS84, EPSG::CH1903_LV03, map_origin);
  }
  const Eigen::Vector3d origin_wgs84 =
      GeoConversions::transformCoordinates(EPSG::CH1903_LV03, EPSG::WGS84, map_origin);
  const Eigen::Vector3d east_step_wgs84 =
      GeoConversions::transformCoordinates(EPSG::CH1903_LV03, EPSG::WGS84,
                                           map_origin + Eigen::Vector3d(1.0, 0.0, 0.0));

  double origin_east = 0.0;
  double origin_north = 0.0;
  double origin_up = 0.0;
  double step_east = 0.0;
  double step_north = 0.0;
  double step_up = 0.0;
  enu_->Forward(origin_wgs84.x(), origin_wgs84.y(), origin_wgs84.z(),
                origin_east, origin_north, origin_up);
  enu_->Forward(east_step_wgs84.x(), east_step_wgs84.y(), east_step_wgs84.z(),
                step_east, step_north, step_up);

  geometry_msgs::TransformStamped world_from_local_msg;
  try {
    world_from_local_msg = tf_buffer_.lookupTransform(
        world_frame_, local_origin_frame_, ros::Time(0), ros::Duration(0.2));
  } catch (const tf2::TransformException &error) {
    ROS_WARN_THROTTLE(2.0, "[TerrainPlanner] Cannot register terrain frame until TF %s -> %s is available: %s",
                      world_frame_.c_str(), local_origin_frame_.c_str(), error.what());
    return false;
  }

  tf2::Transform world_from_local;
  tf2::fromMsg(world_from_local_msg.transform, world_from_local);
  const tf2::Vector3 world_origin = world_from_local * tf2::Vector3(origin_east, origin_north, origin_up);
  const double grid_yaw = std::atan2(step_north - origin_north, step_east - origin_east);
  tf2::Quaternion local_from_terrain;
  local_from_terrain.setRPY(0.0, 0.0, grid_yaw);
  const tf2::Quaternion world_from_terrain = world_from_local.getRotation() * local_from_terrain;

  geometry_msgs::TransformStamped terrain_transform;
  terrain_transform.header.stamp = ros::Time::now();
  terrain_transform.header.frame_id = world_frame_;
  terrain_transform.child_frame_id = terrain_frame_;
  terrain_transform.transform.translation.x = world_origin.x();
  terrain_transform.transform.translation.y = world_origin.y();
  terrain_transform.transform.translation.z = world_origin.z();
  terrain_transform.transform.rotation = tf2::toMsg(world_from_terrain);
  terrain_tf_broadcaster_.sendTransform(terrain_transform);

  if (!terrain_bounds_reported_) {
    const auto &map = terrain_map_->getGridMap();
    const grid_map::Length length = map.getLength();
    const double half_x = 0.5 * length.x();
    const double half_y = 0.5 * length.y();
    ROS_INFO("[TerrainPlanner] DEM coverage in %s is x=[%.2f, %.2f), y=[%.2f, %.2f) m, resolution=%.2f m; "
             "terrain safety checks may reject points inside this rectangle",
             terrain_frame_.c_str(), -half_x, half_x, -half_y, half_y, map.getResolution());

    const std::vector<Eigen::Vector3d> map_corners = {
        Eigen::Vector3d(-half_x, -half_y, 0.0), Eigen::Vector3d(half_x, -half_y, 0.0),
        Eigen::Vector3d(half_x, half_y, 0.0), Eigen::Vector3d(-half_x, half_y, 0.0)};
    std::vector<Eigen::Vector3d> world_corners;
    if (mapPositionsToWorld(map_corners, &world_corners)) {
      double min_world_x = std::numeric_limits<double>::infinity();
      double max_world_x = -std::numeric_limits<double>::infinity();
      double min_world_y = std::numeric_limits<double>::infinity();
      double max_world_y = -std::numeric_limits<double>::infinity();
      for (size_t i = 0; i < world_corners.size(); ++i) {
        min_world_x = std::min(min_world_x, world_corners[i].x());
        max_world_x = std::max(max_world_x, world_corners[i].x());
        min_world_y = std::min(min_world_y, world_corners[i].y());
        max_world_y = std::max(max_world_y, world_corners[i].y());
        ROS_INFO("[TerrainPlanner] WORLD DEM boundary corner %zu: [%.2f, %.2f, %.2f] m",
                 i, world_corners[i].x(), world_corners[i].y(), world_corners[i].z());
      }
      ROS_INFO("[TerrainPlanner] WORLD GOAL INPUT RANGE (axis-aligned envelope only): "
               "x=[%.2f, %.2f], y=[%.2f, %.2f] m; actual DEM boundary is the four corners above",
               min_world_x, max_world_x, min_world_y, max_world_y);
      ROS_INFO("[TerrainPlanner] Controller goal-loiter radius is %.1f m; keep goals at least this far inside the DEM boundary",
               goal_loiter_radius_);
      terrain_bounds_reported_ = true;
    } else {
      ROS_WARN("[TerrainPlanner] Cannot report DEM world bounds until the world/local-origin TF is available");
    }
  }
  return true;
}

void TerrainPlanner::estimatorOdometryCallback(const nav_msgs::Odometry::ConstPtr &msg) {
  const std::lock_guard<std::mutex> lock(goal_mutex_);
  if (!map_initialized_ || !local_origin_received_ || !enu_) {
    vehicle_position_received_ = false;
    return;
  }

  geometry_msgs::PoseStamped estimator_pose;
  estimator_pose.header = msg->header;
  estimator_pose.pose = msg->pose.pose;
  geometry_msgs::PoseStamped local_origin_pose;
  try {
    tf_buffer_.transform(estimator_pose, local_origin_pose, local_origin_frame_, ros::Duration(0.2));
  } catch (const tf2::TransformException &error) {
    vehicle_position_received_ = false;
    ROS_WARN_THROTTLE(2.0, "[TerrainPlanner] Cannot transform estimator pose from %s to %s: %s",
                      msg->header.frame_id.c_str(), local_origin_frame_.c_str(), error.what());
    return;
  }

  const Eigen::Vector3d local_position(local_origin_pose.pose.position.x, local_origin_pose.pose.position.y,
                                       local_origin_pose.pose.position.z);
  if (!local_position.allFinite()) {
    vehicle_position_received_ = false;
    ROS_WARN_THROTTLE(2.0, "[TerrainPlanner] Ignoring non-finite estimator position");
    return;
  }

  vehicle_attitude_(0) = local_origin_pose.pose.orientation.w;
  vehicle_attitude_(1) = local_origin_pose.pose.orientation.x;
  vehicle_attitude_(2) = local_origin_pose.pose.orientation.y;
  vehicle_attitude_(3) = local_origin_pose.pose.orientation.z;
  tf2::Quaternion local_origin_from_body;
  tf2::fromMsg(local_origin_pose.pose.orientation, local_origin_from_body);
  const tf2::Vector3 body_velocity(msg->twist.twist.linear.x, msg->twist.twist.linear.y,
                                   msg->twist.twist.linear.z);
  const tf2::Vector3 local_origin_velocity = tf2::quatRotate(local_origin_from_body, body_velocity);
  geometry_msgs::TransformStamped terrain_from_local_origin;
  try {
    terrain_from_local_origin =
        tf_buffer_.lookupTransform(terrain_frame_, local_origin_frame_, ros::Time(0), ros::Duration(0.2));
  } catch (const tf2::TransformException &error) {
    vehicle_position_received_ = false;
    ROS_WARN_THROTTLE(2.0, "[TerrainPlanner] Cannot transform estimator velocity from %s to %s: %s",
                      local_origin_frame_.c_str(), terrain_frame_.c_str(), error.what());
    return;
  }
  tf2::Quaternion terrain_from_local_rotation;
  tf2::fromMsg(terrain_from_local_origin.transform.rotation, terrain_from_local_rotation);
  const tf2::Vector3 terrain_velocity = tf2::quatRotate(terrain_from_local_rotation, local_origin_velocity);
  vehicle_velocity_ = Eigen::Vector3d(terrain_velocity.x(), terrain_velocity.y(), terrain_velocity.z());

  double latitude = 0.0;
  double longitude = 0.0;
  double altitude = 0.0;
  enu_->Reverse(local_position.x(), local_position.y(), local_position.z(), latitude, longitude, altitude);
  const Eigen::Vector3d wgs84_vehicle_position(latitude, longitude, altitude);
  const Eigen::Vector3d lv03_vehicle_position =
      GeoConversions::transformCoordinates(EPSG::WGS84, EPSG::CH1903_LV03, wgs84_vehicle_position);

  EPSG map_coordinate;
  Eigen::Vector3d map_origin;
  terrain_map_->getGlobalOrigin(map_coordinate, map_origin);
  if (map_coordinate == EPSG::WGS84) {
    map_origin = GeoConversions::transformCoordinates(EPSG::WGS84, EPSG::CH1903_LV03, map_origin);
  }
  vehicle_position_ = lv03_vehicle_position - map_origin;
  if (!vehicle_velocity_.allFinite()) {
    vehicle_position_received_ = false;
    ROS_WARN_THROTTLE(2.0, "[TerrainPlanner] Ignoring non-finite estimator velocity");
    return;
  }
  vehicle_position_received_ = true;
  last_estimator_position_time_ = ros::Time::now();
}

void TerrainPlanner::estimatorStateValidCallback(const std_msgs::Bool::ConstPtr &msg) {
  const std::lock_guard<std::mutex> lock(goal_mutex_);
  estimator_state_valid_ = msg->data;
  if (!estimator_state_valid_) vehicle_position_received_ = false;
}

void TerrainPlanner::MapPublishOnce(const ros::Publisher &pub, const grid_map::GridMap &map) {
  grid_map_msgs::GridMap message;
  grid_map::GridMapRosConverter::toMessage(map, message);
  pub.publish(message);
}

void TerrainPlanner::publishPositionHistory(ros::Publisher &pub, const Eigen::Vector3d &position,
                                            std::vector<geometry_msgs::PoseStamped> &history_vector) {
  unsigned int posehistory_window_ = 200;
  Eigen::Vector4d vehicle_attitude(1.0, 0.0, 0.0, 0.0);
  history_vector.insert(history_vector.begin(), vector3d2PoseStampedMsg(position, vehicle_attitude));
  if (history_vector.size() > posehistory_window_) {
    history_vector.pop_back();
  }

  nav_msgs::Path msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = terrain_frame_;
  for (auto &pose : history_vector) {
    pose.header.stamp = msg.header.stamp;
    pose.header.frame_id = terrain_frame_;
  }
  msg.poses = history_vector;

  pub.publish(msg);
}

void TerrainPlanner::publishGlobalPositionSetpoints(const ros::Publisher &pub, const double latitude,
                                                    const double longitude, const double altitude,
                                                    const Eigen::Vector3d &velocity, const double curvature) {
  using namespace mavros_msgs;
  // Publishes position setpoints sequentially as trajectory setpoints
  mavros_msgs::GlobalPositionTarget msg;
  msg.header.stamp = ros::Time::now();
  msg.coordinate_frame = GlobalPositionTarget::FRAME_GLOBAL_REL_ALT;
  msg.type_mask = 0.0;
  msg.latitude = latitude;
  msg.longitude = longitude;
  msg.altitude = altitude - local_origin_altitude_;
  msg.velocity.x = velocity(0);
  msg.velocity.y = velocity(1);
  msg.velocity.z = velocity(2);
  auto curvature_vector = Eigen::Vector3d(0.0, 0.0, -curvature);
  auto projected_velocity = Eigen::Vector3d(velocity(0), velocity(1), 0.0);
  Eigen::Vector3d lateral_acceleration = projected_velocity.squaredNorm() * curvature_vector.cross(projected_velocity);
  msg.acceleration_or_force.x = lateral_acceleration(0);
  msg.acceleration_or_force.y = lateral_acceleration(1);
  msg.acceleration_or_force.z = lateral_acceleration(2);

  pub.publish(msg);
}

void TerrainPlanner::publishPositionSetpoints(const ros::Publisher &pub, const Eigen::Vector3d &position,
                                              const Eigen::Vector3d &velocity, const double curvature) {
  using namespace mavros_msgs;
  // Publishes position setpoints sequentially as trajectory setpoints
  mavros_msgs::PositionTarget msg;
  msg.header.stamp = ros::Time::now();
  msg.coordinate_frame = PositionTarget::FRAME_LOCAL_NED;
  msg.type_mask = 0.0;
  msg.position.x = position(0);
  msg.position.y = position(1);
  msg.position.z = position(2);
  msg.velocity.x = velocity(0);
  msg.velocity.y = velocity(1);
  msg.velocity.z = velocity(2);
  auto curvature_vector = Eigen::Vector3d(0.0, 0.0, -curvature);
  auto projected_velocity = Eigen::Vector3d(velocity(0), velocity(1), 0.0);
  Eigen::Vector3d lateral_acceleration = projected_velocity.squaredNorm() * curvature_vector.cross(projected_velocity);
  msg.acceleration_or_force.x = lateral_acceleration(0);
  msg.acceleration_or_force.y = lateral_acceleration(1);
  msg.acceleration_or_force.z = lateral_acceleration(2);

  pub.publish(msg);
}

void TerrainPlanner::publishVelocityMarker(const ros::Publisher &pub, const Eigen::Vector3d &position,
                                           const Eigen::Vector3d &velocity) {
  visualization_msgs::Marker marker = vector2ArrowsMsg(position, velocity, 0, Eigen::Vector3d(1.0, 0.0, 1.0));
  marker.header.frame_id = terrain_frame_;
  pub.publish(marker);
}

void TerrainPlanner::publishReferenceMarker(const ros::Publisher &pub, const Eigen::Vector3d &position,
                                            const Eigen::Vector3d &velocity, const double curvature) {
  Eigen::Vector3d scaled_velocity = 20.0 * velocity;
  visualization_msgs::Marker marker =
      vector2ArrowsMsg(position, scaled_velocity, 0, Eigen::Vector3d(0.0, 0.0, 1.0), "reference");

  marker.header.frame_id = terrain_frame_;
  pub.publish(marker);
}

void TerrainPlanner::publishPathSegments(ros::Publisher &pub, Path &trajectory) {
  visualization_msgs::MarkerArray msg;

  std::vector<visualization_msgs::Marker> marker;
  visualization_msgs::Marker mark;
  mark.header.stamp = ros::Time::now();
  mark.header.frame_id = terrain_frame_;
  mark.action = visualization_msgs::Marker::DELETEALL;
  marker.push_back(mark);
  msg.markers = marker;
  pub.publish(msg);

  std::vector<visualization_msgs::Marker> segment_markers;
  int i = 0;
  for (auto &segment : trajectory.segments) {
    Eigen::Vector3d color = Eigen::Vector3d(1.0, 0.0, 0.0);
    if (segment.curvature > 0.0) {  // Green is DUBINS_LEFT
      color = Eigen::Vector3d(0.0, 1.0, 0.0);
    } else if (segment.curvature < 0.0) {  // Blue is DUBINS_RIGHT
      color = Eigen::Vector3d(0.0, 0.0, 1.0);
    }
    segment_markers.insert(segment_markers.begin(), trajectory2MarkerMsg(segment, i++, color));
    segment_markers.insert(segment_markers.begin(), point2MarkerMsg(segment.position().front(), i++, color));
    segment_markers.insert(segment_markers.begin(), point2MarkerMsg(segment.position().back(), i++, color));
  }
  for (auto &segment_marker : segment_markers) segment_marker.header.frame_id = terrain_frame_;
  msg.markers = segment_markers;
  pub.publish(msg);
}

void TerrainPlanner::publishGoal(const ros::Publisher &pub, const Eigen::Vector3d &position, const double radius,
                                 Eigen::Vector3d color, std::string name_space) {
  visualization_msgs::Marker marker;
  marker = getGoalMarker(1, position, radius, color);
  marker.ns = name_space;
  pub.publish(marker);
}

void TerrainPlanner::publishRallyPoints(const ros::Publisher &pub, const std::vector<Eigen::Vector3d> &positions,
                                        const double radius, Eigen::Vector3d color, std::string name_space) {
  visualization_msgs::MarkerArray marker_array;
  std::vector<visualization_msgs::Marker> markers;
  int marker_id = 1;
  for (const auto &position : positions) {
    visualization_msgs::Marker marker;
    marker = getGoalMarker(marker_id, position, radius, color);
    marker.ns = name_space;
    markers.push_back(marker);
    marker_id++;
  }
  marker_array.markers = markers;
  pub.publish(marker_array);
}

visualization_msgs::Marker TerrainPlanner::getGoalMarker(const int id, const Eigen::Vector3d &position,
                                                         const double radius, const Eigen::Vector3d color) {
  visualization_msgs::Marker marker;
  marker.header.frame_id = terrain_frame_;
  marker.header.stamp = ros::Time::now();
  marker.id = id;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD;
  std::vector<geometry_msgs::Point> points;
  /// TODO: Generate circular path
  double delta_theta = 0.05 * 2 * M_PI;
  for (double theta = 0.0; theta < 2 * M_PI + delta_theta; theta += delta_theta) {
    geometry_msgs::Point point;
    point.x = position(0) + radius * std::cos(theta);
    point.y = position(1) + radius * std::sin(theta);
    point.z = position(2);
    points.push_back(point);
  }
  marker.points = points;
  marker.scale.x = 5.0;
  marker.scale.y = 5.0;
  marker.scale.z = 5.0;
  marker.color.a = 0.5;  // Don't forget to set the alpha!
  marker.pose.orientation.w = 1.0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.color.r = color(0);
  marker.color.g = color(1);
  marker.color.b = color(2);
  return marker;
}

void TerrainPlanner::mavstateCallback(const mavros_msgs::State::ConstPtr &msg) { current_state_ = *msg; }

void TerrainPlanner::publishPathSetpoints(const Eigen::Vector3d &position, const Eigen::Vector3d &velocity) {
  using namespace mavros_msgs;
  // Publishes position setpoints sequentially as trajectory setpoints
  mavros_msgs::PositionTarget msg;
  msg.header.stamp = ros::Time::now();
  msg.coordinate_frame = PositionTarget::FRAME_LOCAL_NED;
  msg.type_mask = PositionTarget::IGNORE_AFX | PositionTarget::IGNORE_AFY | PositionTarget::IGNORE_AFZ;
  msg.position.x = position(0);
  msg.position.y = position(1);
  msg.position.z = position(2);
  msg.velocity.x = velocity(0);
  msg.velocity.y = velocity(1);
  msg.velocity.z = velocity(2);

  /// TODO: Package trajectory segments into trajectory waypoints

  mavros_msgs::Trajectory trajectory_msg;
  trajectory_msg.header.stamp = ros::Time::now();
  trajectory_msg.type = mavros_msgs::Trajectory::MAV_TRAJECTORY_REPRESENTATION_WAYPOINTS;
  trajectory_msg.point_valid[0] = true;
  trajectory_msg.point_valid[1] = true;
  trajectory_msg.point_1 = msg;
  trajectory_msg.point_2 = msg;
  trajectory_msg.point_3 = msg;
  trajectory_msg.point_4 = msg;
  trajectory_msg.point_5 = msg;

  path_target_pub_.publish(trajectory_msg);
}

void TerrainPlanner::mavGlobalOriginCallback(const geographic_msgs::GeoPointStampedConstPtr &msg) {
  const std::lock_guard<std::mutex> lock(goal_mutex_);
  std::cout << "[TerrainPlanner] Received Global Origin from FMU" << std::endl;

  local_origin_received_ = true;

  double X = static_cast<double>(msg->position.latitude);
  double Y = static_cast<double>(msg->position.longitude);
  double Z = static_cast<double>(msg->position.altitude);
  const double lat = X;
  const double lon = Y;
  const double alt = Z;
  if (!std::isfinite(lat) || !std::isfinite(lon) || !std::isfinite(alt) ||
      std::abs(lat) > 90.0 || std::abs(lon) > 180.0) {
    ROS_ERROR("[TerrainPlanner] MAVROS global origin is not a valid WGS84 latitude/longitude/altitude");
    local_origin_received_ = false;
    return;
  }
  enu_.emplace(lat, lon, alt, GeographicLib::Geocentric::WGS84());
  local_origin_altitude_ = alt;
  local_origin_latitude_ = lat;
  local_origin_longitude_ = lon;
}

bool TerrainPlanner::refreshControllerLoiterSettings() {
  ros::NodeHandle controller_nh(nh_, "controller");
  double controller_loiter_radius = 0.0;
  bool radius_loaded = controller_nh.getParam("fixedwing/modes/loiter/radius", controller_loiter_radius);
  if (!radius_loaded) {
    radius_loaded = controller_nh.getParam("fixedwing/loiter/radius", controller_loiter_radius);
  }
  if (!radius_loaded || !std::isfinite(controller_loiter_radius) || controller_loiter_radius < 5.0) {
    goal_loiter_radius_ = minimum_turn_radius_;
    goal_loiter_direction_ = 1;
    controller_loiter_settings_loaded_ = false;
    if (!controller_loiter_settings_warned_) {
      ROS_WARN("[TerrainPlanner] Controller loiter radius is unavailable or invalid under %s/controller; "
               "using minimum_turn_radius %.1f m until controller parameters are available",
               nh_.getNamespace().c_str(), minimum_turn_radius_);
      controller_loiter_settings_warned_ = true;
    }
    return false;
  }

  int controller_loiter_direction = 1;
  if (!controller_nh.getParam("fixedwing/modes/loiter/direction", controller_loiter_direction)) {
    controller_nh.getParam("fixedwing/loiter/direction", controller_loiter_direction);
  }
  controller_loiter_direction = controller_loiter_direction >= 0 ? 1 : -1;
  const bool changed = !controller_loiter_settings_loaded_ ||
                       std::abs(goal_loiter_radius_ - controller_loiter_radius) > 1e-3 ||
                       goal_loiter_direction_ != controller_loiter_direction;
  goal_loiter_radius_ = controller_loiter_radius;
  goal_loiter_direction_ = controller_loiter_direction;
  controller_loiter_settings_loaded_ = true;
  controller_loiter_settings_warned_ = false;
  if (changed) {
    ROS_INFO("[TerrainPlanner] Using controller loiter settings: radius=%.1f m direction=%s",
             goal_loiter_radius_, goal_loiter_direction_ > 0 ? "CCW" : "CW");
  }
  return true;
}

bool TerrainPlanner::getGoalLoiterAltitude(const Eigen::Vector2d &goal_center, double radius, double *altitude,
                                          std::string *failure_reason) const {
  if (altitude == nullptr || failure_reason == nullptr || !goal_center.allFinite() ||
      !std::isfinite(radius) || radius <= 0.0) {
    return false;
  }

  const grid_map::GridMap &map = terrain_map_->getGridMap();
  const double sample_spacing = std::max(1.0, 0.5 * map.getResolution());
  const int sample_count = std::max(36, static_cast<int>(std::ceil(2.0 * M_PI * radius / sample_spacing)));
  double highest_safe_altitude = -std::numeric_limits<double>::infinity();
  for (int sample = 0; sample < sample_count; ++sample) {
    const double angle = 2.0 * M_PI * sample / sample_count;
    const Eigen::Vector2d point = goal_center + radius * Eigen::Vector2d(std::cos(angle), std::sin(angle));
    if (!map.isInside(point)) {
      *failure_reason = "controller loiter circle extends beyond DEM coverage";
      return false;
    }
    const double safe_altitude = map.atPosition("distance_surface", point);
    if (!std::isfinite(safe_altitude)) {
      *failure_reason = "controller loiter circle crosses a DEM cell without valid terrain clearance";
      return false;
    }
    highest_safe_altitude = std::max(highest_safe_altitude, safe_altitude);
  }

  *altitude = highest_safe_altitude + 1.0;
  return true;
}

bool TerrainPlanner::buildStartupClimbPath(const Eigen::Vector3d &start_position,
                                           const Eigen::Vector3d &start_velocity,
                                           const Eigen::Vector3d &goal_position, double *target_altitude,
                                           Path *climb_path, Eigen::Vector3d *climb_end_position,
                                           Eigen::Vector3d *climb_end_velocity, std::string *failure_reason) {
  if (target_altitude == nullptr || climb_path == nullptr || climb_end_position == nullptr ||
      climb_end_velocity == nullptr || failure_reason == nullptr) {
    return false;
  }
  climb_path->segments.clear();
  *climb_end_position = start_position;
  *climb_end_velocity = start_velocity;
  failure_reason->clear();

  const grid_map::GridMap &map = terrain_map_->getGridMap();
  const Eigen::Vector2d start_xy = start_position.head<2>();
  if (!start_position.allFinite() || !start_velocity.allFinite() || !map.isInside(start_xy)) {
    *failure_reason = "aircraft position or velocity is invalid, or aircraft is outside DEM coverage";
    return false;
  }

  const double ground_elevation = map.atPosition("elevation", start_xy);
  if (!std::isfinite(ground_elevation) ||
      start_position.z() < ground_elevation + startup_climb_minimum_clearance_) {
    *failure_reason = "aircraft is below the configured minimum clearance for starting a climb";
    return false;
  }

  const double horizontal_velocity = start_velocity.head<2>().norm();
  if (!std::isfinite(horizontal_velocity) || horizontal_velocity < 1.0) {
    *failure_reason = "estimator velocity has no usable horizontal direction for a fixed-wing route";
    return false;
  }

  double local_safe_altitude = 0.0;
  if (!getMinimumSafeAltitude(map, start_xy, &local_safe_altitude)) {
    *failure_reason = "cannot read the terrain-clearance layer at the aircraft position";
    return false;
  }
  *target_altitude = local_safe_altitude + 1.0;
  if (*target_altitude > global_altitude_ceiling_) {
    *failure_reason = "local terrain-clearance altitude exceeds the map altitude ceiling";
    return false;
  }

  if (start_position.z() >= *target_altitude) return true;
  if (cruise_speed_ <= 0.0 || minimum_turn_radius_ <= 0.0 || max_climb_rate_control_ <= 0.0 ||
      maximum_climb_angle_ <= 0.0) {
    *failure_reason = "startup climb speed, radius, or climb limit is invalid";
    return false;
  }

  const double vertical_speed = std::min(max_climb_rate_control_, cruise_speed_ * std::sin(maximum_climb_angle_));
  if (!std::isfinite(vertical_speed) || vertical_speed <= 0.0 || vertical_speed >= cruise_speed_) {
    *failure_reason = "configured startup climb rate is invalid";
    return false;
  }
  const double flightpath_angle = std::asin(vertical_speed / cruise_speed_);
  const double horizontal_speed = std::sqrt(cruise_speed_ * cruise_speed_ - vertical_speed * vertical_speed);
  const double initial_yaw = std::atan2(start_velocity.y(), start_velocity.x());
  const double sample_dt = 0.2;
  const double max_segment_angle = M_PI_2;

  bool found_valid_direction = false;
  Path best_path;
  Eigen::Vector3d best_end_position = start_position;
  Eigen::Vector3d best_end_velocity = start_velocity;
  double best_goal_distance = std::numeric_limits<double>::infinity();

  for (const double turn_sign : {1.0, -1.0}) {
    const double omega = turn_sign * horizontal_speed / minimum_turn_radius_;
    double direction_target_altitude = local_safe_altitude + 1.0;
    Path direction_path;
    bool direction_is_valid = true;

    auto make_state = [&](double time, State *state) {
      const double yaw = initial_yaw + omega * time;
      const double x = start_position.x() + horizontal_speed / omega * (std::sin(yaw) - std::sin(initial_yaw));
      const double y = start_position.y() - horizontal_speed / omega * (std::cos(yaw) - std::cos(initial_yaw));
      const double z = start_position.z() + vertical_speed * time;
      const Eigen::Vector2d xy(x, y);
      if (!map.isInside(xy) || !std::isfinite(z) || z > global_altitude_ceiling_) return false;
      const double elevation = map.atPosition("elevation", xy);
      if (!std::isfinite(elevation) || z < elevation + startup_climb_minimum_clearance_) return false;

      state->position = Eigen::Vector3d(x, y, z);
      state->velocity = Eigen::Vector3d(horizontal_speed * std::cos(yaw), horizontal_speed * std::sin(yaw),
                                        vertical_speed);
      state->attitude = Eigen::Vector4d(std::cos(yaw / 2.0), 0.0, 0.0, std::sin(yaw / 2.0));
      return true;
    };

    // The startup circle can extend beyond the point directly below the aircraft. Check its full
    // horizontal track against the DEM-derived clearance surface before choosing its altitude.
    for (int sample = 0; sample <= 72 && direction_is_valid; ++sample) {
      const double time = (2.0 * M_PI * sample / 72.0) / std::abs(omega);
      const double yaw = initial_yaw + omega * time;
      const Eigen::Vector2d xy(start_position.x() + horizontal_speed / omega *
                                   (std::sin(yaw) - std::sin(initial_yaw)),
                               start_position.y() - horizontal_speed / omega *
                                   (std::cos(yaw) - std::cos(initial_yaw)));
      double ring_safe_altitude = 0.0;
      if (!getMinimumSafeAltitude(map, xy, &ring_safe_altitude)) {
        direction_is_valid = false;
        break;
      }
      direction_target_altitude = std::max(direction_target_altitude, ring_safe_altitude + 1.0);
    }
    if (!direction_is_valid || direction_target_altitude > global_altitude_ceiling_) continue;

    const double direction_climb_time = (direction_target_altitude - start_position.z()) / vertical_speed;
    const double direction_turn_angle = std::abs(omega) * direction_climb_time;
    const int direction_segment_count =
        std::max(1, static_cast<int>(std::ceil(direction_turn_angle / max_segment_angle)));
    for (int segment_index = 0; segment_index < direction_segment_count && direction_is_valid; ++segment_index) {
      const double segment_start_time = direction_climb_time * segment_index / direction_segment_count;
      const double segment_end_time = direction_climb_time * (segment_index + 1) / direction_segment_count;
      const int steps = std::max(1, static_cast<int>(std::ceil((segment_end_time - segment_start_time) / sample_dt)));
      PathSegment segment;
      segment.curvature = turn_sign / minimum_turn_radius_;
      segment.flightpath_angle = flightpath_angle;
      segment.dt = (segment_end_time - segment_start_time) / steps;
      segment.is_periodic = false;
      for (int step = 0; step <= steps; ++step) {
        const double time = segment_start_time + (segment_end_time - segment_start_time) * step / steps;
        State state;
        if (!make_state(time, &state)) {
          direction_is_valid = false;
          break;
        }
        segment.states.push_back(state);
      }
      if (direction_is_valid && segment.states.size() > 1) direction_path.appendSegment(segment);
    }
    if (!direction_is_valid || direction_path.segments.empty()) continue;

    const Eigen::Vector3d end_position = direction_path.lastSegment().states.back().position;
    const Eigen::Vector3d end_velocity = direction_path.lastSegment().states.back().velocity;
    const double goal_distance = (end_position - goal_position).head<2>().norm();
    if (!found_valid_direction || goal_distance < best_goal_distance) {
      found_valid_direction = true;
      best_path = direction_path;
      best_end_position = end_position;
      best_end_velocity = end_velocity;
      best_goal_distance = goal_distance;
      *target_altitude = direction_target_altitude;
    }
  }

  if (!found_valid_direction) {
    *failure_reason = "no terrain-safe climb circle fits inside the DEM and map altitude ceiling";
    return false;
  }

  *climb_path = best_path;
  *climb_end_position = best_end_position;
  *climb_end_velocity = best_end_velocity;
  return true;
}

void TerrainPlanner::goalCallback(const geometry_msgs::PointStamped::ConstPtr &msg) {
  const std::lock_guard<std::mutex> lock(goal_mutex_);
  // A rejected new message must not leave an older target armed for planning.
  goal_received_ = false;
  const Eigen::Vector3d requested_goal(msg->point.x, msg->point.y, msg->point.z);
  ROS_INFO("[TerrainPlanner] Received goal message: frame='%s', point=[%.3f, %.3f, %.3f] m",
           msg->header.frame_id.c_str(), requested_goal.x(), requested_goal.y(), requested_goal.z());
  if (!map_initialized_) {
    ROS_WARN("[TerrainPlanner] Goal rejected: terrain map is not loaded yet");
    return;
  }
  refreshControllerLoiterSettings();

  std::string input_frame = msg->header.frame_id.empty() ? planning_input_frame_ : msg->header.frame_id;
  while (!input_frame.empty() && input_frame.front() == '/') input_frame.erase(0, 1);
  if (input_frame.empty()) {
    ROS_WARN("[TerrainPlanner] Goal rejected: set header.frame_id or planning_input_frame");
    return;
  }

  if (!requested_goal.allFinite()) {
    ROS_WARN("[TerrainPlanner] Goal rejected: point contains non-finite coordinates");
    return;
  }

  Eigen::Vector3d candidate_goal;
  if (!planningPositionToMap(requested_goal, input_frame, &candidate_goal)) {
    ROS_WARN("[TerrainPlanner] Goal rejected: cannot transform from frame '%s' into '%s'",
             input_frame.c_str(), terrain_frame_.c_str());
    return;
  }
  const Eigen::Vector2d candidate_goal_xy(candidate_goal.x(), candidate_goal.y());
  if (!terrain_map_->getGridMap().isInside(candidate_goal_xy)) {
    const auto &length = terrain_map_->getGridMap().getLength();
    ROS_WARN("[TerrainPlanner] Goal rejected: transformed %s point=[%.2f, %.2f, %.2f] m is outside DEM coverage "
             "x=[%.2f, %.2f), y=[%.2f, %.2f) m",
             terrain_frame_.c_str(), candidate_goal.x(), candidate_goal.y(), candidate_goal.z(),
             -0.5 * length.x(), 0.5 * length.x(), -0.5 * length.y(), 0.5 * length.y());
    publishGoal(candidate_goal_pub_, candidate_goal, goal_loiter_radius_, Eigen::Vector3d(1.0, 0.0, 0.0), "goal");
    return;
  }

  double minimum_safe_goal_altitude = 0.0;
  std::string goal_altitude_failure_reason;
  if (!getGoalLoiterAltitude(candidate_goal.head<2>(), goal_loiter_radius_, &minimum_safe_goal_altitude,
                             &goal_altitude_failure_reason)) {
    ROS_WARN("[TerrainPlanner] Goal rejected: %s (goal loiter radius %.1f m)",
             goal_altitude_failure_reason.c_str(), goal_loiter_radius_);
    publishGoal(candidate_goal_pub_, candidate_goal, goal_loiter_radius_, Eigen::Vector3d(1.0, 0.0, 0.0), "goal");
    return;
  }
  Eigen::Vector3d new_goal = candidate_goal;
  new_goal.z() = minimum_safe_goal_altitude + 1.0;
  if (new_goal.z() > global_altitude_ceiling_) {
    ROS_WARN("[TerrainPlanner] Goal rejected: required altitude %.2f m exceeds map ceiling %.2f m",
             new_goal.z(), global_altitude_ceiling_);
    publishGoal(candidate_goal_pub_, new_goal, goal_loiter_radius_, Eigen::Vector3d(1.0, 0.0, 0.0), "goal");
    return;
  }

  goal_pos_ = new_goal;
  goal_received_ = true;
  ROS_INFO("[TerrainPlanner] Goal ACCEPTED: input frame='%s', input=[%.2f, %.2f, %.2f] m -> "
           "%s=[%.2f, %.2f, %.2f] m (controller loiter radius %.1f m; input z is ignored)",
           input_frame.c_str(), requested_goal.x(), requested_goal.y(), requested_goal.z(), terrain_frame_.c_str(),
           goal_pos_.x(), goal_pos_.y(), goal_pos_.z(), goal_loiter_radius_);
  publishGoal(candidate_goal_pub_, goal_pos_, goal_loiter_radius_, Eigen::Vector3d(0.0, 1.0, 0.0), "goal");
}

void TerrainPlanner::planCallback(const std_msgs::Empty::ConstPtr &) {
  const std::lock_guard<std::mutex> lock(goal_mutex_);
  if (!map_initialized_) {
    ROS_WARN("[TerrainPlanner] Cannot plan: terrain map is not loaded yet");
    return;
  }
  if (!goal_received_) {
    ROS_WARN("[TerrainPlanner] Cannot plan: publish terrain_planner/goal first");
    return;
  }
  if (!refreshControllerLoiterSettings()) {
    ROS_WARN("[TerrainPlanner] Cannot plan until %s/controller publishes valid fixedwing loiter parameters",
             nh_.getNamespace().c_str());
    return;
  }
  double minimum_safe_goal_altitude = 0.0;
  std::string goal_altitude_failure_reason;
  if (!getGoalLoiterAltitude(goal_pos_.head<2>(), goal_loiter_radius_, &minimum_safe_goal_altitude,
                             &goal_altitude_failure_reason) ||
      minimum_safe_goal_altitude > global_altitude_ceiling_) {
    ROS_WARN("[TerrainPlanner] Cannot plan: controller loiter circle is not terrain-safe: %s",
             goal_altitude_failure_reason.empty() ? "required altitude exceeds map ceiling"
                                                  : goal_altitude_failure_reason.c_str());
    return;
  }
  goal_pos_.z() = minimum_safe_goal_altitude;
  publishGoal(candidate_goal_pub_, goal_pos_, goal_loiter_radius_, Eigen::Vector3d(0.0, 1.0, 0.0), "goal");
  const double estimator_position_age =
      (ros::Time::now() - last_estimator_position_time_).toSec();
  if (!estimator_state_valid_ || !vehicle_position_received_ || estimator_position_age > estimator_pose_timeout_) {
    ROS_WARN("[TerrainPlanner] Cannot plan: waiting for valid state_estimator/main/odom");
    return;
  }
  const Eigen::Vector2d vehicle_position_xy(vehicle_position_.x(), vehicle_position_.y());
  if (!terrain_map_->getGridMap().isInside(vehicle_position_xy)) {
    ROS_WARN("[TerrainPlanner] Cannot plan: aircraft is outside the loaded terrain map");
    return;
  }

  Path startup_climb;
  Eigen::Vector3d route_start_position;
  Eigen::Vector3d route_start_velocity;
  double route_start_target_altitude = vehicle_position_.z();
  std::string climb_failure_reason;
  if (!buildStartupClimbPath(vehicle_position_, vehicle_velocity_, goal_pos_, &route_start_target_altitude,
                             &startup_climb, &route_start_position, &route_start_velocity,
                             &climb_failure_reason)) {
    ROS_WARN("[TerrainPlanner] Cannot plan from current aircraft state: %s", climb_failure_reason.c_str());
    publishGoal(candidate_start_pub_, vehicle_position_, start_loiter_radius_, Eigen::Vector3d(1.0, 0.0, 0.0),
                "start");
    return;
  }

  start_pos_ = vehicle_position_;
  home_position_ = vehicle_position_;
  home_position_radius_ = start_loiter_radius_;
  candidate_primitive_.segments.clear();
  startup_climb_path_ = startup_climb;
  candidate_starts_at_current_state_ = true;
  planned_path_published_ = false;
  problem_updated_ = true;
  plan_time_ = ros::Time::now();
  global_planner_->setupProblem(route_start_position, route_start_velocity, goal_pos_, goal_loiter_radius_,
                                goal_loiter_direction_);
  planner_mode_ = PLANNER_MODE::GLOBAL;
  if (startup_climb_path_.segments.empty()) {
    ROS_INFO("[TerrainPlanner] Planning directly from aircraft state [%.2f, %.2f, %.2f] to goal [%.2f, %.2f, %.2f] (budget %.1f s)",
             vehicle_position_.x(), vehicle_position_.y(), vehicle_position_.z(), goal_pos_.x(), goal_pos_.y(),
             goal_pos_.z(), planner_time_budget_);
  } else {
    ROS_INFO("[TerrainPlanner] Planning from aircraft [%.2f, %.2f, %.2f]; terrain-safe climb to %.2f m, then route from [%.2f, %.2f, %.2f] to goal [%.2f, %.2f, %.2f] (budget %.1f s)",
             vehicle_position_.x(), vehicle_position_.y(), vehicle_position_.z(), route_start_target_altitude,
             route_start_position.x(), route_start_position.y(), route_start_position.z(), goal_pos_.x(),
             goal_pos_.y(), goal_pos_.z(), planner_time_budget_);
  }
  publishGoal(candidate_start_pub_, vehicle_position_, minimum_turn_radius_, Eigen::Vector3d(0.0, 1.0, 0.0), "start");
}

void TerrainPlanner::executePathCallback(const std_msgs::Empty::ConstPtr &) {
  const std::lock_guard<std::mutex> lock(goal_mutex_);
  if (candidate_primitive_.segments.empty()) {
    ROS_WARN("[TerrainPlanner] Cannot execute path: no planned route is available");
    return;
  }
  query_planner_state_ = PLANNER_STATE::NAVIGATE;
  ROS_INFO("[TerrainPlanner] Accepted execute_path command");
}

void TerrainPlanner::plannerStateCallback(const std_msgs::UInt8::ConstPtr &msg) {
  const std::lock_guard<std::mutex> lock(goal_mutex_);
  switch (msg->data) {
    case 1:
      query_planner_state_ = PLANNER_STATE::HOLD;
      break;
    case 2:
      query_planner_state_ = PLANNER_STATE::NAVIGATE;
      break;
    case 3:
      query_planner_state_ = PLANNER_STATE::ROLLOUT;
      break;
    case 4:
      query_planner_state_ = PLANNER_STATE::ABORT;
      break;
    case 5:
      query_planner_state_ = PLANNER_STATE::RETURN;
      break;
    default:
      ROS_WARN("[TerrainPlanner] Ignoring unknown planner_state value %u (valid values: 1-5)",
               static_cast<unsigned int>(msg->data));
      return;
  }
  ROS_INFO("[TerrainPlanner] Requested planner state %u", static_cast<unsigned int>(msg->data));
}

void TerrainPlanner::maxAltitudeCheckCallback(const std_msgs::Bool::ConstPtr &msg) {
  const std::lock_guard<std::mutex> lock(goal_mutex_);
  global_planner_->setMaxAltitudeCollisionChecks(msg->data);
  ROS_INFO("[TerrainPlanner] Maximum-altitude collision checking %s", msg->data ? "enabled" : "disabled");
}

void TerrainPlanner::generateCircle(const Eigen::Vector3d end_position, const Eigen::Vector3d end_velocity,
                                    const Eigen::Vector3d center_pos, PathSegment &trajectory) {
  Eigen::Vector3d radial_vector = (end_position - center_pos);
  radial_vector(2) = 0.0;  // Only consider horizontal loiters
  Eigen::Vector3d emergency_rates =
      20.0 * end_velocity.normalized().cross(radial_vector.normalized()) / radial_vector.norm();
  double horizon = 2 * M_PI / std::abs(emergency_rates(2));
  // Append a loiter at the end of the planned path
  trajectory = generateArcTrajectory(emergency_rates, horizon, end_position, end_velocity);
  trajectory.is_periodic = true;
  return;
}

PathSegment TerrainPlanner::generateArcTrajectory(Eigen::Vector3d rate, const double horizon,
                                                  Eigen::Vector3d current_pos, Eigen::Vector3d current_vel,
                                                  const double dt) {
  PathSegment trajectory;
  trajectory.states.clear();

  double time = 0.0;
  const double current_yaw = std::atan2(-1.0 * current_vel(1), current_vel(0));
  const double climb_rate = rate(1);
  trajectory.flightpath_angle = std::asin(climb_rate / cruise_speed_);
  /// TODO: Fix sign conventions for curvature
  trajectory.curvature = -rate(2) / cruise_speed_;
  trajectory.dt = dt;
  for (int i = 0; i < std::max(1.0, horizon / dt); i++) {
    if (std::abs(rate(2)) < 0.0001) {
      rate(2) > 0.0 ? rate(2) = 0.0001 : rate(2) = -0.0001;
    }
    double yaw = rate(2) * time + current_yaw;

    Eigen::Vector3d pos =
        cruise_speed_ / rate(2) *
            Eigen::Vector3d(std::sin(yaw) - std::sin(current_yaw), std::cos(yaw) - std::cos(current_yaw), 0) +
        Eigen::Vector3d(0, 0, climb_rate * time) + current_pos;
    Eigen::Vector3d vel = Eigen::Vector3d(cruise_speed_ * std::cos(yaw), -cruise_speed_ * std::sin(yaw), -climb_rate);
    const double roll = std::atan(rate(2) * cruise_speed_ / 9.81);
    const double pitch = std::atan(climb_rate / cruise_speed_);
    Eigen::Vector4d att = rpy2quaternion(roll, -pitch, -yaw);  // TODO: why the hell do you need to reverse signs?

    State state_vector;
    state_vector.position = pos;
    state_vector.velocity = vel;
    state_vector.attitude = att;
    trajectory.states.push_back(state_vector);

    time = time + dt;
  }
  return trajectory;
}

void TerrainPlanner::publishPath(const ros::Publisher &pub, Path &path) {
  planner_msgs::Path path_msg;
  for (const auto &path_segment : path.segments) {
    planner_msgs::PathSegment segment_msg;
    segment_msg.reached = path_segment.reached;
    segment_msg.segment_start = toVector3(path_segment.states.front().position);
    Eigen::Vector3d start_velocity = path_segment.states.front().velocity;
    segment_msg.segment_tangent = toVector3(start_velocity.normalized());
    segment_msg.periodic = path_segment.is_periodic;
    segment_msg.curvature.data = double(path_segment.curvature);
    double segment_length = path_segment.getLength();
    segment_msg.segment_length.data = double(segment_length);
    path_msg.segments.push_back(segment_msg);
  }
  pub.publish(path_msg);
  return;
}
