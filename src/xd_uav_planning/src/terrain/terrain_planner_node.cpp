#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <ompl/util/RandomNumbers.h>
#include <ros/ros.h>
#include <xd_uav_task_allocate/PlannerStatus.h>

#include "terrain_navigation/path.h"
#include "terrain_navigation/terrain_map.h"
#include "terrain_planner/DubinsAirplane.hpp"
#include "terrain_planner/terrain_ompl_rrt.h"
#include "xd_uav_planning/terrain_path_validator.h"

namespace xd_uav_planning {
namespace {

// terrain_navigation samples Dubins paths in units normalized by the
// configured minimum turning radius, despite naming the argument resolution.
constexpr double kTerrainLibraryNormalizedResolution = 0.05;

bool finitePoint(const geometry_msgs::Point& point) {
  return std::isfinite(point.x) && std::isfinite(point.y) &&
         std::isfinite(point.z);
}

Eigen::Vector3d positionOf(const geometry_msgs::PoseStamped& pose) {
  return Eigen::Vector3d(pose.pose.position.x, pose.pose.position.y,
                         pose.pose.position.z);
}

}  // namespace

class TerrainPlannerNode {
 public:
  TerrainPlannerNode() : private_node_("~") {
    loadParameters();
    if (random_seed_ > 0) {
      ompl::RNG::setSeed(static_cast<std::uint_fast32_t>(random_seed_));
    }
    loadTerrain();
    path_publisher_ = node_.advertise<nav_msgs::Path>(output_topic_, 1, true);
    status_publisher_ = node_.advertise<xd_uav_task_allocate::PlannerStatus>(
        status_topic_, 10, true);
    request_subscriber_ = node_.subscribe(
        request_topic_, 1, &TerrainPlannerNode::requestCallback, this);
    publishStatus(0, xd_uav_task_allocate::PlannerStatus::IDLE,
                  "terrain_planner_ready");
  }

 private:
  void loadParameters() {
    private_node_.param<std::string>("map_path", map_path_, "");
    private_node_.param<std::string>("common_frame", common_frame_, "world");
    private_node_.param<std::string>(
        "request_topic", request_topic_, "planning/task_path");
    private_node_.param<std::string>(
        "output_topic", output_topic_, "planning/terrain_path");
    private_node_.param<std::string>(
        "status_topic", status_topic_, "planning/status");
    private_node_.param("minimum_clearance_m", minimum_clearance_m_, 50.0);
    private_node_.param("maximum_clearance_m", maximum_clearance_m_, 120.0);
    private_node_.param("safety_margin_m", safety_margin_m_, 1.0);
    private_node_.param("terrain_elevation_offset_m",
                        terrain_elevation_offset_m_, 0.0);
    private_node_.param("minimum_turning_radius_m",
                        limits_.minimum_turning_radius_m, 66.66667);
    private_node_.param("maximum_climb_angle_rad",
                        limits_.maximum_climb_angle_rad, 0.15);
    private_node_.param("validation_sample_step_m",
                        limits_.validation_sample_step_m, 1.0);
    private_node_.param("maximum_sample_gap_m",
                        limits_.maximum_sample_gap_m, 10.0);
    private_node_.param("endpoint_tolerance_m",
                        limits_.endpoint_tolerance_m, 0.01);
    private_node_.param("surface_tolerance_m",
                        limits_.surface_tolerance_m, 1e-3);
    int maximum_output_points = 10000;
    private_node_.param("maximum_output_points", maximum_output_points, 10000);
    private_node_.param("maximum_request_points", maximum_request_points_, 64);
    private_node_.param("request_timeout_s", request_timeout_s_, 2.0);
    private_node_.param("future_tolerance_s", future_tolerance_s_, 0.05);
    private_node_.param("solve_time_per_leg_s", solve_time_per_leg_s_, 5.0);
    private_node_.param("maximum_solve_attempts", maximum_solve_attempts_, 1);
    private_node_.param("minimum_request_segment_m",
                        minimum_request_segment_m_, 1.0);
    private_node_.param("minimum_output_spacing_m",
                        minimum_output_spacing_m_, 0.05);
    private_node_.param("random_seed", random_seed_, 0);

    if (map_path_.empty()) throw std::runtime_error("map_path is required");
    if (normalizeFrame(common_frame_).empty() || request_topic_.empty() ||
        output_topic_.empty() || status_topic_.empty()) {
      throw std::runtime_error("terrain planner interfaces are invalid");
    }
    if (!std::isfinite(minimum_clearance_m_) ||
        !std::isfinite(maximum_clearance_m_) ||
        !std::isfinite(safety_margin_m_) ||
        !std::isfinite(terrain_elevation_offset_m_) ||
        !std::isfinite(limits_.minimum_turning_radius_m) ||
        !std::isfinite(limits_.maximum_climb_angle_rad) ||
        !std::isfinite(limits_.validation_sample_step_m) ||
        !std::isfinite(limits_.maximum_sample_gap_m) ||
        !std::isfinite(limits_.endpoint_tolerance_m) ||
        !std::isfinite(limits_.surface_tolerance_m) ||
        !std::isfinite(request_timeout_s_) ||
        !std::isfinite(future_tolerance_s_) ||
        !std::isfinite(solve_time_per_leg_s_) ||
        !std::isfinite(minimum_request_segment_m_) ||
        !std::isfinite(minimum_output_spacing_m_) ||
        minimum_clearance_m_ <= 0.0 ||
        maximum_clearance_m_ <= minimum_clearance_m_ ||
        safety_margin_m_ < 0.0 ||
        minimum_clearance_m_ + safety_margin_m_ >=
            maximum_clearance_m_ - safety_margin_m_ ||
        limits_.minimum_turning_radius_m <= 0.0 ||
        limits_.maximum_climb_angle_rad <= 0.0 ||
        limits_.validation_sample_step_m <= 0.0 ||
        limits_.maximum_sample_gap_m <= 0.0 ||
        limits_.endpoint_tolerance_m < 0.0 ||
        limits_.surface_tolerance_m < 0.0 ||
        solve_time_per_leg_s_ <= 0.0 || maximum_solve_attempts_ < 1 ||
        maximum_solve_attempts_ > 10 || request_timeout_s_ <= 0.0 ||
        future_tolerance_s_ < 0.0 ||
        maximum_request_points_ < 2 || minimum_request_segment_m_ <= 0.0 ||
        maximum_output_points < 2 || minimum_output_spacing_m_ <= 0.0 ||
        random_seed_ < 0) {
      throw std::runtime_error("terrain planner parameters are invalid");
    }
    limits_.maximum_output_points =
        static_cast<std::size_t>(maximum_output_points);
  }

  void loadTerrain() {
    terrain_map_ = std::make_shared<TerrainMap>();
    if (!terrain_map_->initializeFromGeotiff(map_path_)) {
      throw std::runtime_error("failed to load terrain GeoTIFF: " + map_path_);
    }
    auto& map = terrain_map_->getGridMap();
    map.setFrameId(common_frame_);
    map["elevation"].array() -= terrain_elevation_offset_m_;
    const double lower_surface = minimum_clearance_m_ + safety_margin_m_;
    const double upper_surface = maximum_clearance_m_ - safety_margin_m_;
    if (!terrain_map_->AddLayerDistanceTransform(
            lower_surface, "distance_surface") ||
        !terrain_map_->AddLayerDistanceTransform(
            upper_surface, "max_elevation")) {
      throw std::runtime_error("failed to create terrain safety layers");
    }
    ROS_INFO_STREAM("terrain planner loaded " << map_path_ << " as "
                    << map.getSize().x() << "x" << map.getSize().y()
                    << " cells in frame " << common_frame_);
  }

  std::uint32_t goalId(const nav_msgs::Path& request) const {
    if (!request.poses.empty() && request.poses.front().header.seq != 0) {
      return request.poses.front().header.seq;
    }
    return 0;
  }

  std::string validateRequest(const nav_msgs::Path& request) const {
    const std::string frame = normalizeFrame(request.header.frame_id);
    if (frame != normalizeFrame(common_frame_)) return "request_frame_mismatch";
    if (request.header.stamp.isZero()) return "request_stamp_missing";
    const double age = (ros::Time::now() - request.header.stamp).toSec();
    if (age > request_timeout_s_) return "request_stale";
    if (age < -future_tolerance_s_) return "request_from_future";
    if (request.poses.size() < 2) return "request_too_short";
    if (request.poses.size() >
        static_cast<std::size_t>(maximum_request_points_)) {
      return "request_too_many_points";
    }
    const std::uint32_t goal_id = request.poses.front().header.seq;
    if (goal_id == 0) return "request_goal_id_missing";
    for (std::size_t index = 0; index < request.poses.size(); ++index) {
      const auto& pose = request.poses[index];
      if (normalizeFrame(pose.header.frame_id) != frame) {
        return "request_pose_frame_mismatch";
      }
      if (pose.header.seq != goal_id) return "request_goal_id_mismatch";
      if (!finitePoint(pose.pose.position)) return "request_not_finite";
      if (index > 0 &&
          (positionOf(pose) - positionOf(request.poses[index - 1])).norm() <
              minimum_request_segment_m_) {
        return "request_segment_too_short";
      }
    }
    return "ok";
  }

  static std::string normalizeFrame(const std::string& frame) {
    const auto first = frame.find_first_not_of('/');
    return first == std::string::npos ? std::string() : frame.substr(first);
  }

  Eigen::Vector3d waypointVelocity(const nav_msgs::Path& request,
                                   std::size_t index) const {
    Eigen::Vector3d direction;
    if (index == 0) {
      direction = positionOf(request.poses[1]) - positionOf(request.poses[0]);
    } else if (index + 1 == request.poses.size()) {
      direction = positionOf(request.poses[index]) -
                  positionOf(request.poses[index - 1]);
    } else {
      direction = positionOf(request.poses[index + 1]) -
                  positionOf(request.poses[index - 1]);
    }
    direction.z() = 0.0;
    if (direction.head<2>().norm() <= 1e-9) {
      throw std::runtime_error("request_horizontal_tangent_undefined");
    }
    return direction.normalized();
  }

  bool planLeg(const Eigen::Vector3d& start,
               const Eigen::Vector3d& start_velocity,
               const Eigen::Vector3d& goal,
               const Eigen::Vector3d& goal_velocity,
               bool search_terminal_headings, ::Path* output,
               std::string* reason) {
    // A task nav_msgs/Path specifies waypoint positions, not a mandatory
    // fixed-wing terminal heading.  Preserve the chord-derived heading on
    // the first attempt, then explore distinct, still fully constrained
    // approach headings when an interactive request has a retry budget.
    // Intermediate waypoint headings remain fixed to preserve continuity.
    static const double kTerminalHeadingOffsets[] = {
        0.0, M_PI_2, -M_PI_2, M_PI, M_PI_4, -M_PI_4,
        3.0 * M_PI_4, -3.0 * M_PI_4, M_PI_4 / 2.0, -M_PI_4 / 2.0};
    std::string last_reason = "no_exact_solution";
    for (int attempt = 1; attempt <= maximum_solve_attempts_; ++attempt) {
      // A fresh OMPL planner owns a fresh RNG stream. With random_seed=0 this
      // preserves production entropy; with a fixed seed, retries remain
      // deterministic while exploring different streams from that seed.
      auto planner = std::make_shared<TerrainOmplRrt>();
      auto space = planner->getProblemSetup()
                       ->getStateSpace()
                       ->as<fw_planning::spaces::DubinsAirplaneStateSpace>();
      space->setMinTurningRadius(limits_.minimum_turning_radius_m);
      space->setMaxClimbingAngle(limits_.maximum_climb_angle_rad);
      planner->setMap(terrain_map_);
      planner->setAltitudeLimits(maximum_clearance_m_ - safety_margin_m_,
                                 minimum_clearance_m_ + safety_margin_m_);
      planner->setBoundsFromMap(terrain_map_->getGridMap());
      Eigen::Vector3d attempt_goal_velocity = goal_velocity;
      if (search_terminal_headings && attempt > 1) {
        const std::size_t offset_index = std::min<std::size_t>(
            static_cast<std::size_t>(attempt - 1),
            sizeof(kTerminalHeadingOffsets) /
                    sizeof(kTerminalHeadingOffsets[0]) -
                1);
        const double yaw = std::atan2(goal_velocity.y(), goal_velocity.x()) +
                           kTerminalHeadingOffsets[offset_index];
        attempt_goal_velocity = Eigen::Vector3d(std::cos(yaw), std::sin(yaw),
                                                0.0);
      }
      planner->setupProblem(start, start_velocity, goal,
                            attempt_goal_velocity);
      ::Path leg;
      if (!planner->Solve(solve_time_per_leg_s_, leg)) {
        last_reason = "no_exact_solution";
        ROS_WARN_STREAM("terrain solve attempt " << attempt << "/"
                        << maximum_solve_attempts_
                        << " did not find an exact solution");
        continue;
      }
      repairNearGoal(&leg, goal, attempt_goal_velocity);
      const auto validation = validateTerrainPath(
          leg, *terrain_map_, start, goal, limits_);
      if (!validation.valid) {
        last_reason = validation.reason;
        ROS_WARN_STREAM("terrain solve attempt " << attempt << "/"
                        << maximum_solve_attempts_
                        << " failed output validation: " << last_reason);
        continue;
      }
      output->appendSegment(leg);
      return true;
    }
    *reason = last_reason;
    return false;
  }

  void repairNearGoal(::Path* path, const Eigen::Vector3d& goal,
                      const Eigen::Vector3d& goal_velocity) const {
    if (path == nullptr || path->segments.empty() ||
        path->segments.back().states.empty()) {
      return;
    }
    auto& states = path->segments.back().states;
    const Eigen::Vector3d delta = goal - states.back().position;
    const double error = delta.norm();
    if (!std::isfinite(error) || error <= limits_.endpoint_tolerance_m) {
      return;
    }
    const double climb_cosine = std::cos(limits_.maximum_climb_angle_rad);
    if (climb_cosine <= 0.0) {
      return;
    }
    const double maximum_omission =
        kTerrainLibraryNormalizedResolution *
        limits_.minimum_turning_radius_m / climb_cosine;
    if (error > maximum_omission + 1e-6) {
      ROS_WARN_STREAM("terrain library omitted requested goal by " << error
                      << " m; refusing endpoint repair");
      return;
    }
    const double horizontal_error = delta.head<2>().norm();
    if (std::abs(std::atan2(delta.z(), horizontal_error)) >
        limits_.maximum_climb_angle_rad + 1e-6) {
      return;
    }

    State endpoint = states.back();
    endpoint.position = goal;
    endpoint.velocity = goal_velocity.normalized();
    const double yaw = std::atan2(endpoint.velocity.y(),
                                  endpoint.velocity.x());
    endpoint.attitude = Eigen::Vector4d(
        std::cos(0.5 * yaw), 0.0, 0.0, std::sin(0.5 * yaw));
    states.push_back(endpoint);
  }

  nav_msgs::Path pathMessage(const ::Path& terrain_path,
                             std::uint32_t goal_id) const {
    nav_msgs::Path message;
    message.header.stamp = ros::Time::now();
    message.header.frame_id = common_frame_;
    message.header.seq = goal_id;
    Eigen::Vector3d previous = Eigen::Vector3d::Zero();
    bool have_previous = false;
    for (std::size_t segment_index = 0;
         segment_index < terrain_path.segments.size(); ++segment_index) {
      const auto& segment = terrain_path.segments[segment_index];
      for (std::size_t state_index = 0;
           state_index < segment.states.size(); ++state_index) {
        const auto& state = segment.states[state_index];
        const bool final_state =
            segment_index + 1 == terrain_path.segments.size() &&
            state_index + 1 == segment.states.size();
        if (have_previous &&
            (state.position - previous).norm() < minimum_output_spacing_m_ &&
            !final_state) {
          continue;
        }
        geometry_msgs::PoseStamped pose;
        pose.header = message.header;
        pose.header.seq = goal_id;
        pose.pose.position.x = state.position.x();
        pose.pose.position.y = state.position.y();
        pose.pose.position.z = state.position.z();
        const double yaw = std::atan2(state.velocity.y(), state.velocity.x());
        pose.pose.orientation.z = std::sin(0.5 * yaw);
        pose.pose.orientation.w = std::cos(0.5 * yaw);
        message.poses.push_back(pose);
        previous = state.position;
        have_previous = true;
      }
    }
    return message;
  }

  void requestCallback(const nav_msgs::Path::ConstPtr& request) {
    const std::uint32_t goal_id = goalId(*request);
    const std::string request_validation = validateRequest(*request);
    if (request_validation != "ok") {
      publishStatus(goal_id, xd_uav_task_allocate::PlannerStatus::FAILED,
                    "terrain_request_rejected:" + request_validation);
      return;
    }
    publishStatus(goal_id, xd_uav_task_allocate::PlannerStatus::PLANNING,
                  "terrain_planning_started");
    ::Path combined;
    try {
      for (std::size_t index = 0; index + 1 < request->poses.size(); ++index) {
        std::string reason;
        const bool search_terminal_headings =
            index + 2 == request->poses.size();
        if (!planLeg(positionOf(request->poses[index]),
                     waypointVelocity(*request, index),
                     positionOf(request->poses[index + 1]),
                     waypointVelocity(*request, index + 1),
                     search_terminal_headings, &combined, &reason)) {
          std::ostringstream detail;
          detail << "terrain_planning_blocked:leg=" << index << ":" << reason;
          publishStatus(goal_id,
                        xd_uav_task_allocate::PlannerStatus::BLOCKED,
                        detail.str());
          return;
        }
      }
    } catch (const std::exception& error) {
      publishStatus(goal_id, xd_uav_task_allocate::PlannerStatus::FAILED,
                    std::string("terrain_planning_failed:") + error.what());
      return;
    }

    const auto final_validation = validateTerrainPath(
        combined, *terrain_map_, positionOf(request->poses.front()),
        positionOf(request->poses.back()), limits_);
    if (!final_validation.valid) {
      publishStatus(goal_id, xd_uav_task_allocate::PlannerStatus::BLOCKED,
                    "terrain_output_rejected:" + final_validation.reason);
      return;
    }
    auto output = pathMessage(combined, goal_id);
    if (output.poses.size() < 2 ||
        output.poses.size() > limits_.maximum_output_points) {
      publishStatus(goal_id, xd_uav_task_allocate::PlannerStatus::FAILED,
                    "terrain_output_rejected:point_count");
      return;
    }
    path_publisher_.publish(output);
    std::ostringstream detail;
    detail << "terrain_path_ready:points=" << output.poses.size()
           << ":lower_margin_m=" << final_validation.minimum_lower_margin_m
           << ":upper_margin_m=" << final_validation.minimum_upper_margin_m;
    publishStatus(goal_id, xd_uav_task_allocate::PlannerStatus::PLANNING,
                  detail.str());
  }

  void publishStatus(std::uint32_t goal_id, std::uint8_t state,
                     const std::string& detail) {
    xd_uav_task_allocate::PlannerStatus status;
    status.header.stamp = ros::Time::now();
    status.header.frame_id = common_frame_;
    status.goal_id = goal_id;
    status.state = state;
    status.detail = detail;
    status_publisher_.publish(status);
  }

  ros::NodeHandle node_;
  ros::NodeHandle private_node_;
  ros::Subscriber request_subscriber_;
  ros::Publisher path_publisher_;
  ros::Publisher status_publisher_;
  std::shared_ptr<TerrainMap> terrain_map_;
  TerrainPathLimits limits_;
  std::string map_path_;
  std::string common_frame_;
  std::string request_topic_;
  std::string output_topic_;
  std::string status_topic_;
  double minimum_clearance_m_{50.0};
  double maximum_clearance_m_{120.0};
  double safety_margin_m_{1.0};
  double terrain_elevation_offset_m_{0.0};
  double request_timeout_s_{2.0};
  double future_tolerance_s_{0.05};
  double solve_time_per_leg_s_{5.0};
  int maximum_solve_attempts_{1};
  double minimum_request_segment_m_{1.0};
  double minimum_output_spacing_m_{0.05};
  int maximum_request_points_{64};
  int random_seed_{0};
};

}  // namespace xd_uav_planning

int main(int argc, char** argv) {
  ros::init(argc, argv, "terrain_planner");
  try {
    xd_uav_planning::TerrainPlannerNode node;
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL_STREAM("terrain planner startup failed: " << error.what());
    return 1;
  }
  return 0;
}
