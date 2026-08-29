#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <deque>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <XmlRpcValue.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <image_geometry/pinhole_camera_model.h>
#include <opencv2/core.hpp>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/Header.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <vision_msgs/Detection2D.h>
#include <vision_msgs/Detection2DArray.h>
#include <vision_msgs/ObjectHypothesisWithPose.h>

#include <sar_yolo_detector/TaskCandidate.h>
#include <sar_yolo_detector/TaskCandidateArray.h>
#include <sar_yolo_detector/GetTaskCandidates.h>
#include <sar_yolo_detector/TerrainGrid.h>
#include <sar_yolo_detector/TrackedDetection2DArray.h>
#include <sar_yolo_detector/linear_assignment.hpp>
#include <sar_yolo_detector/runtime_identity.hpp>

namespace sar_yolo_detector {
namespace {

double clamp(const double value, const double lower, const double upper) {
  return std::max(lower, std::min(upper, value));
}

double iou(const cv::Rect2d &left, const cv::Rect2d &right) {
  const cv::Rect2d intersection = left & right;
  const double union_area = left.area() + right.area() - intersection.area();
  return union_area <= 0.0 ? 0.0 : intersection.area() / union_area;
}

double pointDistance(const geometry_msgs::Point &left,
                     const geometry_msgs::Point &right) {
  const double dx = left.x - right.x;
  const double dy = left.y - right.y;
  const double dz = left.z - right.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

bool isFiniteBox(const vision_msgs::Detection2D &detection) {
  return std::isfinite(detection.bbox.center.x) &&
         std::isfinite(detection.bbox.center.y) &&
         std::isfinite(detection.bbox.size_x) &&
         std::isfinite(detection.bbox.size_y) && detection.bbox.size_x > 0.0 &&
         detection.bbox.size_y > 0.0;
}

std::vector<int> integerArrayParameter(ros::NodeHandle &node,
                                       const std::string &name) {
  XmlRpc::XmlRpcValue value;
  if (!node.getParam(name, value) ||
      value.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    return {};
  }
  std::vector<int> result;
  result.reserve(value.size());
  for (int index = 0; index < value.size(); ++index) {
    if (value[index].getType() != XmlRpc::XmlRpcValue::TypeInt) {
      throw std::invalid_argument("Parameter " + name +
                                  " must be an integer array");
    }
    result.push_back(static_cast<int>(value[index]));
  }
  return result;
}

std::vector<std::string> stringArrayParameter(ros::NodeHandle &node,
                                              const std::string &name) {
  XmlRpc::XmlRpcValue value;
  if (!node.getParam(name, value) ||
      value.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    return {};
  }
  std::vector<std::string> result;
  result.reserve(value.size());
  for (int index = 0; index < value.size(); ++index) {
    if (value[index].getType() != XmlRpc::XmlRpcValue::TypeString) {
      throw std::invalid_argument("Parameter " + name +
                                  " must be a string array");
    }
    result.push_back(static_cast<std::string>(value[index]));
  }
  return result;
}

std::vector<double> numberArrayParameter(ros::NodeHandle &node,
                                         const std::string &name) {
  XmlRpc::XmlRpcValue value;
  if (!node.getParam(name, value) ||
      value.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    return {};
  }
  std::vector<double> result;
  result.reserve(value.size());
  for (int index = 0; index < value.size(); ++index) {
    if (value[index].getType() == XmlRpc::XmlRpcValue::TypeInt) {
      result.push_back(static_cast<int>(value[index]));
    } else if (value[index].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
      result.push_back(static_cast<double>(value[index]));
    } else {
      throw std::invalid_argument("Parameter " + name +
                                  " must contain numbers");
    }
  }
  return result;
}

bool containsValue(const std::vector<int> &values, const int value) {
  return std::find(values.begin(), values.end(), value) != values.end();
}

} // namespace

class TaskPointGeneratorNode {
public:
  TaskPointGeneratorNode()
      : private_node_("~"), tf_buffer_(ros::Duration(10.0)),
        tf_listener_(tf_buffer_) {
    loadParameters();
    candidates_publisher_ = node_.advertise<TaskCandidateArray>(
        candidates_topic_, 10, false);
    task_pose_publisher_ =
        node_.advertise<geometry_msgs::PoseStamped>(task_pose_topic_, 5, false);
    snapshot_service_ = node_.advertiseService(
        candidates_topic_ + "/get_snapshot",
        &TaskPointGeneratorNode::snapshotServiceCallback, this);
    if (use_tracked_detections_) {
      tracked_detections_subscriber_ = node_.subscribe(
          tracked_detections_topic_, 5,
          &TaskPointGeneratorNode::trackedDetectionsCallback, this,
          ros::TransportHints().tcpNoDelay());
    } else {
      detections_subscriber_ = node_.subscribe(
          input_detections_topic_, 5,
          &TaskPointGeneratorNode::detectionsCallback, this,
          ros::TransportHints().tcpNoDelay());
    }
    camera_info_subscriber_ = node_.subscribe(
        camera_info_topic_, 2, &TaskPointGeneratorNode::cameraInfoCallback,
        this, ros::TransportHints().tcpNoDelay());
    if (localization_mode_ == "terrain") {
      terrain_subscriber_ = node_.subscribe(
          terrain_grid_topic_, 1, &TaskPointGeneratorNode::terrainCallback, this,
          ros::TransportHints().tcpNoDelay());
    }

    expiration_timer_ = node_.createWallTimer(
        ros::WallDuration(std::max(0.05, expiration_check_period_sec_)),
        &TaskPointGeneratorNode::expirationTimerCallback, this);
    diagnostics_.setHardwareID("sar_task_point_" + sensor_id_);
    diagnostics_.add("task_point_generator", this,
                     &TaskPointGeneratorNode::diagnosticCallback);
    diagnostic_timer_ = node_.createWallTimer(
        ros::WallDuration(1.0),
        &TaskPointGeneratorNode::diagnosticTimerCallback, this);
    snapshot_timer_ = node_.createWallTimer(
        ros::WallDuration(std::max(0.1, snapshot_period_sec_)),
        &TaskPointGeneratorNode::snapshotTimerCallback, this);

    ROS_INFO_STREAM("[SarTaskPoint] Ready: detections="
                    << input_detections_topic_ << " candidates="
                    << candidates_topic_ << " pose=" << task_pose_topic_
                    << " mode=" << localization_mode_ << " frame="
                    << task_frame_ << " uav=" << identity_.uav_id
                    << " session=" << identity_.session_uuid
                    << " task_generation="
                    << (task_generation_active_ ? "active" : "blocked"));
  }

private:
  struct Observation {
    vision_msgs::Detection2D detection;
    ros::Time stamp;
    int class_id{-1};
    std::string class_name;
    float confidence{0.0F};
    cv::Rect2d box;
    bool localization_valid{false};
    geometry_msgs::PoseStamped task_pose;
    geometry_msgs::Vector3Stamped bearing;
    std::array<double, 9> covariance{};
    bool source_track_id_valid{false};
    std::int64_t source_track_id{-1};
  };

  struct CandidateTrack {
    std::uint32_t id{0};
    int class_id{-1};
    std::string class_name;
    cv::Rect2d last_box;
    bool localization_valid{false};
    geometry_msgs::PoseStamped task_pose;
    geometry_msgs::Vector3Stamped bearing;
    std::array<double, 9> covariance{};
    float confidence{0.0F};
    float stability{0.0F};
    std::uint32_t observation_count{0};
    std::uint32_t consecutive_hits{0};
    ros::Time first_seen;
    ros::Time last_seen;
    ros::WallTime last_update_wall;
    ros::WallTime last_pose_publish_wall;
    std::deque<ros::Time> hit_stamps;
    geometry_msgs::Vector3 velocity;
    bool velocity_valid{false};
    bool source_track_id_valid{false};
    std::int64_t source_track_id{-1};
    bool confirmed{false};
    bool pose_published{false};
  };

  void loadParameters() {
    private_node_.param("input_detections_topic", input_detections_topic_,
                        std::string("sar_yolo_detector/detections"));
    private_node_.param("use_tracked_detections", use_tracked_detections_,
                        false);
    private_node_.param("tracked_detections_topic", tracked_detections_topic_,
                        std::string("sar_yolo_detector/tracked_detections"));
    private_node_.param("camera_info_topic", camera_info_topic_,
                        std::string("camera/camera_info"));
    private_node_.param("task_candidates_topic", candidates_topic_,
                        std::string("sar_yolo_detector/task_candidates"));
    private_node_.param("task_pose_topic", task_pose_topic_,
                        std::string("sar_yolo_detector/task_point"));
    private_node_.param("sensor_id", sensor_id_, std::string("camera_primary"));
    private_node_.param("task_frame", task_frame_, std::string("map"));
    private_node_.param("localization_mode", localization_mode_,
                        std::string("bearing_only"));
    private_node_.param("ground_plane_z", ground_plane_z_, 0.0);
    private_node_.param("tf_lookup_timeout_sec", tf_lookup_timeout_sec_, 0.05);
    private_node_.param("tf_use_latest_on_failure", tf_use_latest_on_failure_,
                        false);
    private_node_.param("minimum_intersection_range_m",
                        minimum_intersection_range_m_, 1.0);
    private_node_.param("maximum_intersection_range_m",
                        maximum_intersection_range_m_, 300.0);
    private_node_.param("pixel_measurement_stddev_px",
                        pixel_measurement_stddev_px_, 2.0);
    private_node_.param("ground_height_stddev_m", ground_height_stddev_m_, 1.5);
    private_node_.param("camera_position_stddev_m", camera_position_stddev_m_,
                        0.30);
    private_node_.param("camera_attitude_stddev_deg", camera_attitude_stddev_deg_,
                        0.50);
    private_node_.param("calibration_angular_stddev_deg",
                        calibration_angular_stddev_deg_, 0.25);
    private_node_.param("time_sync_stddev_sec", time_sync_stddev_sec_, 0.010);
    private_node_.param("vehicle_velocity_stddev_mps",
                        vehicle_velocity_stddev_mps_, 2.0);
    private_node_.param("camera_info_max_delta_sec", camera_info_max_delta_sec_,
                        0.10);
    private_node_.param("camera_info_allow_static", camera_info_allow_static_,
                        true);
    private_node_.param("terrain_grid_topic", terrain_grid_topic_,
                        std::string("terrain/elevation_grid"));
    private_node_.param("terrain_max_age_sec", terrain_max_age_sec_, 30.0);
    private_node_.param("terrain_ray_step_m", terrain_ray_step_m_, 2.0);
    private_node_.param("require_matching_camera_frame",
                        require_matching_camera_frame_, true);

    private_node_.param("require_detection_timestamp",
                        require_detection_timestamp_, true);
    private_node_.param("reject_out_of_order", reject_out_of_order_, true);
    private_node_.param("input_max_age_sec", input_max_age_sec_, 1.0);
    private_node_.param("future_timestamp_tolerance_sec",
                        future_timestamp_tolerance_sec_, 0.02);
    private_node_.param("minimum_detection_confidence",
                        minimum_detection_confidence_, 0.40);
    private_node_.param("min_confirmations", min_confirmations_, 3);
    private_node_.param("confirmation_window_sec", confirmation_window_sec_,
                        1.0);
    private_node_.param("track_timeout_sec", track_timeout_sec_, 2.0);
    private_node_.param("association_iou_threshold", association_iou_threshold_,
                        0.10);
    private_node_.param("association_center_distance_px",
                        association_center_distance_px_, 120.0);
    private_node_.param("association_world_distance_m",
                        association_world_distance_m_, 12.0);
    private_node_.param("association_mahalanobis_threshold",
                        association_mahalanobis_threshold_, 9.21);
    private_node_.param("confidence_ema_alpha", confidence_ema_alpha_, 0.45);
    private_node_.param("position_ema_alpha", position_ema_alpha_, 0.35);
    private_node_.param("minimum_task_priority", minimum_task_priority_, 0.55);
    private_node_.param("task_pose_update_distance_m",
                        task_pose_update_distance_m_, 2.0);
    private_node_.param("task_pose_refresh_sec", task_pose_refresh_sec_, 3.0);
    private_node_.param("publish_pending_candidates", publish_pending_candidates_,
                        true);
    private_node_.param("expiration_check_period_sec",
                        expiration_check_period_sec_, 0.20);
    private_node_.param("snapshot_period_sec", snapshot_period_sec_, 1.0);
    private_node_.param("task_generation_enabled", task_generation_enabled_,
                        false);
    private_node_.param("model_validated", model_validated_, false);
    private_node_.param("allow_experimental_model", allow_experimental_model_,
                        false);
    private_node_.param("all_task_classes", all_task_classes_, false);

    task_class_ids_ = integerArrayParameter(private_node_, "task_class_ids");
    class_names_ = stringArrayParameter(private_node_, "class_names");
    global_semantic_ids_ =
        integerArrayParameter(private_node_, "global_semantic_ids");
    class_anchor_y_fractions_ =
        numberArrayParameter(private_node_, "class_anchor_y_fractions");

    std::transform(localization_mode_.begin(), localization_mode_.end(),
                   localization_mode_.begin(), ::tolower);
    if (localization_mode_ == "ground_plane")
      localization_mode_ = "horizontal_plane";
    if (localization_mode_ != "horizontal_plane" &&
        localization_mode_ != "terrain" &&
        localization_mode_ != "bearing_only") {
      throw std::invalid_argument(
          "~localization_mode must be bearing_only, horizontal_plane, or terrain");
    }
    if (task_frame_.empty() && localization_mode_ != "bearing_only") {
      throw std::invalid_argument("~task_frame must be nonempty for localization");
    }
    minimum_intersection_range_m_ =
        std::max(0.0, minimum_intersection_range_m_);
    maximum_intersection_range_m_ = std::max(minimum_intersection_range_m_,
                                             maximum_intersection_range_m_);
    tf_lookup_timeout_sec_ = std::max(0.0, tf_lookup_timeout_sec_);
    input_max_age_sec_ = std::max(0.0, input_max_age_sec_);
    future_timestamp_tolerance_sec_ =
        std::max(0.0, future_timestamp_tolerance_sec_);
    minimum_detection_confidence_ = clamp(minimum_detection_confidence_, 0.0, 1.0);
    min_confirmations_ = std::max(1, min_confirmations_);
    confirmation_window_sec_ = std::max(0.05, confirmation_window_sec_);
    track_timeout_sec_ = std::max(confirmation_window_sec_, track_timeout_sec_);
    association_iou_threshold_ = clamp(association_iou_threshold_, 0.0, 1.0);
    association_center_distance_px_ = std::max(1.0, association_center_distance_px_);
    association_world_distance_m_ = std::max(0.1, association_world_distance_m_);
    association_mahalanobis_threshold_ =
        std::max(0.1, association_mahalanobis_threshold_);
    confidence_ema_alpha_ = clamp(confidence_ema_alpha_, 0.01, 1.0);
    position_ema_alpha_ = clamp(position_ema_alpha_, 0.01, 1.0);
    minimum_task_priority_ = clamp(minimum_task_priority_, 0.0, 1.0);
    task_pose_update_distance_m_ = std::max(0.0, task_pose_update_distance_m_);
    task_pose_refresh_sec_ = std::max(0.0, task_pose_refresh_sec_);
    camera_position_stddev_m_ = std::max(0.0, camera_position_stddev_m_);
    camera_attitude_stddev_deg_ = std::max(0.0, camera_attitude_stddev_deg_);
    calibration_angular_stddev_deg_ =
        std::max(0.0, calibration_angular_stddev_deg_);
    time_sync_stddev_sec_ = std::max(0.0, time_sync_stddev_sec_);
    vehicle_velocity_stddev_mps_ =
        std::max(0.0, vehicle_velocity_stddev_mps_);
    camera_info_max_delta_sec_ = std::max(0.0, camera_info_max_delta_sec_);
    terrain_max_age_sec_ = std::max(0.0, terrain_max_age_sec_);
    terrain_ray_step_m_ = std::max(0.1, terrain_ray_step_m_);
    snapshot_period_sec_ = std::max(0.1, snapshot_period_sec_);
    if (!global_semantic_ids_.empty() &&
        global_semantic_ids_.size() != class_names_.size()) {
      throw std::invalid_argument(
          "~global_semantic_ids must be empty or align with ~class_names");
    }
    if (!class_anchor_y_fractions_.empty() &&
        class_anchor_y_fractions_.size() != class_names_.size()) {
      throw std::invalid_argument(
          "~class_anchor_y_fractions must be empty or align with ~class_names");
    }
    for (double &fraction : class_anchor_y_fractions_)
      fraction = clamp(fraction, 0.0, 1.0);

    identity_ = loadPerceptionIdentity(
        &private_node_, sensor_id_, "generic",
        localization_mode_ == "bearing_only" ? "CAMERA_RAY"
                                               : "LOCAL_ENU:" + task_frame_);
    sensor_id_ = identity_.sensor_id;
    task_generation_active_ =
        task_generation_enabled_ &&
        (model_validated_ || allow_experimental_model_) &&
        (all_task_classes_ || !task_class_ids_.empty());
    if (task_generation_enabled_ && !task_generation_active_) {
      ROS_ERROR_STREAM(
          "[SarTaskPoint] Task generation requested but blocked: model_validated="
          << model_validated_ << " allow_experimental_model="
          << allow_experimental_model_ << " task_class_count="
          << task_class_ids_.size());
    }
  }

  void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr &message) {
    if (message->K[0] <= 0.0 || message->K[4] <= 0.0 || message->width == 0 ||
        message->height == 0) {
      std::lock_guard<std::mutex> lock(mutex_);
      ++invalid_camera_info_;
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    camera_info_history_.push_back(message);
    while (camera_info_history_.size() > 50U)
      camera_info_history_.pop_front();
    have_camera_model_ = true;
  }

  void terrainCallback(const TerrainGrid::ConstPtr &message) {
    const std::size_t expected = static_cast<std::size_t>(message->width) *
                                 static_cast<std::size_t>(message->height);
    if (message->header.frame_id != task_frame_ || message->resolution <= 0.0F ||
        message->width < 2U || message->height < 2U ||
        message->elevation.size() != expected ||
        (!message->variance.empty() && message->variance.size() != expected)) {
      std::lock_guard<std::mutex> lock(mutex_);
      ++invalid_terrain_grids_;
      last_error_ = "invalid terrain grid or frame contract";
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    terrain_grid_ = message;
  }

  bool inputAccepted(const vision_msgs::Detection2DArray &message) {
    if (message.header.stamp.isZero()) {
      if (!require_detection_timestamp_)
        return true;
      ++rejected_input_messages_;
      last_error_ = "missing detection capture timestamp";
      return false;
    }
    const ros::Time now = ros::Time::now();
    if (!now.isZero()) {
      const double age = (now - message.header.stamp).toSec();
      if (age < -future_timestamp_tolerance_sec_) {
        ++rejected_input_messages_;
        last_error_ = "detection timestamp is in the future";
        return false;
      }
      if (input_max_age_sec_ > 0.0 && age > input_max_age_sec_) {
        ++rejected_input_messages_;
        last_error_ = "detection message is stale";
        return false;
      }
    }
    if (reject_out_of_order_ && !last_input_stamp_.isZero() &&
        message.header.stamp <= last_input_stamp_) {
      ++rejected_input_messages_;
      last_error_ = "out-of-order detection timestamp";
      return false;
    }
    last_input_stamp_ = message.header.stamp;
    return true;
  }

  bool extractObservation(const vision_msgs::Detection2D &detection,
                          const std_msgs::Header &array_header,
                          const bool source_track_id_valid,
                          const std::int64_t source_track_id,
                          Observation *observation) {
    if (observation == nullptr || !isFiniteBox(detection)) {
      ++rejected_detections_;
      return false;
    }
    int selected_class = -1;
    float selected_confidence = -1.0F;
    for (const auto &hypothesis : detection.results) {
      const int class_id = static_cast<int>(hypothesis.id);
      if (!(all_task_classes_ || containsValue(task_class_ids_, class_id)) ||
          !std::isfinite(hypothesis.score) ||
          hypothesis.score < minimum_detection_confidence_) {
        continue;
      }
      if (hypothesis.score > selected_confidence) {
        selected_class = class_id;
        selected_confidence = hypothesis.score;
      }
    }
    if (selected_class < 0) {
      ++filtered_detections_;
      return false;
    }

    observation->detection = detection;
    observation->detection.header = array_header;
    observation->stamp = array_header.stamp;
    observation->class_id = selected_class;
    observation->class_name = className(selected_class);
    observation->confidence = selected_confidence;
    observation->source_track_id_valid = source_track_id_valid;
    observation->source_track_id = source_track_id;
    observation->box = cv::Rect2d(
        detection.bbox.center.x - 0.5 * detection.bbox.size_x,
        detection.bbox.center.y - 0.5 * detection.bbox.size_y,
        detection.bbox.size_x, detection.bbox.size_y);
    localize(observation, array_header);
    return true;
  }

  sensor_msgs::CameraInfoConstPtr
  cameraInfoForStamp(const ros::Time &stamp) const {
    sensor_msgs::CameraInfoConstPtr best;
    double best_delta = std::numeric_limits<double>::infinity();
    for (const auto &candidate : camera_info_history_) {
      if (candidate->header.stamp.isZero()) {
        if (camera_info_allow_static_ && !best)
          best = candidate;
        continue;
      }
      const double delta = std::abs((candidate->header.stamp - stamp).toSec());
      if (delta < best_delta) {
        best_delta = delta;
        best = candidate;
      }
    }
    if (best && !best->header.stamp.isZero() &&
        best_delta > camera_info_max_delta_sec_) {
      return sensor_msgs::CameraInfoConstPtr();
    }
    return best;
  }

  bool planeIntersection(const tf2::Vector3 &origin,
                         const tf2::Vector3 &unit_direction,
                         const double plane_height,
                         tf2::Vector3 *position,
                         double *range) const {
    if (position == nullptr || std::abs(unit_direction.z()) < 1.0e-8)
      return false;
    const double distance =
        (plane_height - origin.z()) / unit_direction.z();
    if (!std::isfinite(distance) || distance < minimum_intersection_range_m_ ||
        distance > maximum_intersection_range_m_) {
      return false;
    }
    *position = origin + distance * unit_direction;
    if (range != nullptr)
      *range = distance;
    return true;
  }

  bool terrainElevation(const TerrainGrid &grid, const double x, const double y,
                        double *height, double *variance) const {
    const double grid_x = (x - grid.origin_x) / grid.resolution;
    const double grid_y = (y - grid.origin_y) / grid.resolution;
    if (grid_x < 0.0 || grid_y < 0.0 ||
        grid_x > static_cast<double>(grid.width - 1U) ||
        grid_y > static_cast<double>(grid.height - 1U)) {
      return false;
    }
    const std::uint32_t x0 = static_cast<std::uint32_t>(std::floor(grid_x));
    const std::uint32_t y0 = static_cast<std::uint32_t>(std::floor(grid_y));
    const std::uint32_t x1 = std::min(x0 + 1U, grid.width - 1U);
    const std::uint32_t y1 = std::min(y0 + 1U, grid.height - 1U);
    const double fx = grid_x - x0;
    const double fy = grid_y - y0;
    const auto interpolate = [&](const std::vector<float> &values,
                                 const double fallback) {
      if (values.empty())
        return fallback;
      const auto value = [&](const std::uint32_t column,
                             const std::uint32_t row) {
        return static_cast<double>(
            values[static_cast<std::size_t>(row) * grid.width + column]);
      };
      const double a = value(x0, y0), b = value(x1, y0);
      const double c = value(x0, y1), d = value(x1, y1);
      if (!std::isfinite(a) || !std::isfinite(b) || !std::isfinite(c) ||
          !std::isfinite(d)) {
        return std::numeric_limits<double>::quiet_NaN();
      }
      return (1.0 - fy) * ((1.0 - fx) * a + fx * b) +
             fy * ((1.0 - fx) * c + fx * d);
    };
    const double interpolated_height =
        interpolate(grid.elevation, std::numeric_limits<double>::quiet_NaN());
    if (!std::isfinite(interpolated_height))
      return false;
    *height = interpolated_height;
    if (variance != nullptr) {
      const double value = interpolate(
          grid.variance, ground_height_stddev_m_ * ground_height_stddev_m_);
      *variance = std::isfinite(value) && value >= 0.0
                      ? value
                      : ground_height_stddev_m_ * ground_height_stddev_m_;
    }
    return true;
  }

  bool terrainIntersection(const tf2::Vector3 &origin,
                           const tf2::Vector3 &unit_direction,
                           const TerrainGrid &grid, tf2::Vector3 *position,
                           double *range, double *terrain_variance) const {
    bool have_previous = false;
    double previous_range = minimum_intersection_range_m_;
    double previous_difference = 0.0;
    for (double current_range = minimum_intersection_range_m_;
         current_range <= maximum_intersection_range_m_;
         current_range += terrain_ray_step_m_) {
      const tf2::Vector3 current = origin + current_range * unit_direction;
      double height = 0.0, variance = 0.0;
      if (!terrainElevation(grid, current.x(), current.y(), &height, &variance)) {
        have_previous = false;
        continue;
      }
      const double difference = current.z() - height;
      if (have_previous &&
          ((previous_difference >= 0.0 && difference <= 0.0) ||
           (previous_difference <= 0.0 && difference >= 0.0))) {
        const double denominator = previous_difference - difference;
        const double ratio = std::abs(denominator) < 1.0e-9
                                 ? 0.0
                                 : previous_difference / denominator;
        const double hit_range = previous_range +
                                 clamp(ratio, 0.0, 1.0) *
                                     (current_range - previous_range);
        tf2::Vector3 hit = origin + hit_range * unit_direction;
        if (!terrainElevation(grid, hit.x(), hit.y(), &height, &variance))
          return false;
        hit.setZ(height);
        *position = hit;
        if (range != nullptr)
          *range = hit_range;
        if (terrain_variance != nullptr)
          *terrain_variance = variance;
        return true;
      }
      have_previous = true;
      previous_range = current_range;
      previous_difference = difference;
    }
    return false;
  }

  static void addOuterProduct(std::array<double, 9> *covariance,
                              const tf2::Vector3 &error) {
    const double values[3] = {error.x(), error.y(), error.z()};
    for (int row = 0; row < 3; ++row) {
      for (int column = 0; column < 3; ++column) {
        (*covariance)[static_cast<std::size_t>(3 * row + column)] +=
            values[row] * values[column];
      }
    }
  }

  void computeLocalizationCovariance(
      const image_geometry::PinholeCameraModel &camera_model,
      const tf2::Transform &transform, const cv::Point2d &pixel,
      const tf2::Vector3 &origin, const tf2::Vector3 &position,
      const double range, const double surface_variance,
      std::array<double, 9> *covariance) const {
    covariance->fill(0.0);
    const auto projected = [&](const cv::Point2d &sample_pixel,
                               const double plane_height,
                               tf2::Vector3 *sample_position) {
      const cv::Point3d camera_ray =
          camera_model.projectPixelTo3dRay(sample_pixel);
      tf2::Vector3 direction = transform.getBasis() *
                               tf2::Vector3(camera_ray.x, camera_ray.y,
                                            camera_ray.z);
      if (direction.length2() < 1.0e-12)
        return false;
      direction.normalize();
      return planeIntersection(origin, direction, plane_height, sample_position,
                               nullptr);
    };

    for (int axis = 0; axis < 2; ++axis) {
      cv::Point2d plus = pixel, minus = pixel;
      if (axis == 0) {
        plus.x += pixel_measurement_stddev_px_;
        minus.x -= pixel_measurement_stddev_px_;
      } else {
        plus.y += pixel_measurement_stddev_px_;
        minus.y -= pixel_measurement_stddev_px_;
      }
      tf2::Vector3 plus_position, minus_position;
      if (projected(plus, position.z(), &plus_position) &&
          projected(minus, position.z(), &minus_position)) {
        addOuterProduct(covariance,
                        0.5 * (plus_position - minus_position));
      }
    }

    const double surface_stddev = std::sqrt(std::max(0.0, surface_variance));
    tf2::Vector3 plus_height, minus_height;
    tf2::Vector3 base_direction = position - origin;
    if (base_direction.length2() > 1.0e-12) {
      base_direction.normalize();
      if (planeIntersection(origin, base_direction,
                            position.z() + surface_stddev, &plus_height,
                            nullptr) &&
          planeIntersection(origin, base_direction,
                            position.z() - surface_stddev, &minus_height,
                            nullptr)) {
        addOuterProduct(covariance, 0.5 * (plus_height - minus_height));
      }
    }

    const double pi = std::acos(-1.0);
    const double angular_stddev =
        (camera_attitude_stddev_deg_ + calibration_angular_stddev_deg_) *
        pi / 180.0;
    const double angular_variance =
        range * range * angular_stddev * angular_stddev;
    const double timing_variance =
        std::pow(time_sync_stddev_sec_ * vehicle_velocity_stddev_mps_, 2.0);
    const double position_variance =
        camera_position_stddev_m_ * camera_position_stddev_m_;
    for (int axis = 0; axis < 3; ++axis) {
      (*covariance)[static_cast<std::size_t>(axis * 3 + axis)] +=
          position_variance + timing_variance + angular_variance;
    }
  }

  void localize(Observation *observation, const std_msgs::Header &header) {
    observation->bearing.header = header;
    observation->task_pose.header = header;
    observation->task_pose.pose.orientation.w = 1.0;
    observation->covariance.fill(0.0);

    sensor_msgs::CameraInfoConstPtr camera_info;
    TerrainGrid::ConstPtr terrain_grid;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!have_camera_model_) {
        ++localization_failures_;
        last_error_ = "camera calibration is unavailable";
        return;
      }
      camera_info = cameraInfoForStamp(header.stamp);
      terrain_grid = terrain_grid_;
    }
    if (!camera_info) {
      std::lock_guard<std::mutex> lock(mutex_);
      ++localization_failures_;
      last_error_ = "capture-time CameraInfo is unavailable";
      return;
    }
    image_geometry::PinholeCameraModel camera_model;
    camera_model.fromCameraInfo(*camera_info);
    const std::string camera_frame =
        !camera_info->header.frame_id.empty() ? camera_info->header.frame_id
                                               : header.frame_id;
    if (camera_frame.empty()) {
      std::lock_guard<std::mutex> lock(mutex_);
      ++localization_failures_;
      last_error_ = "camera frame is empty";
      return;
    }
    if (require_matching_camera_frame_ && !header.frame_id.empty() &&
        !camera_info->header.frame_id.empty() &&
        header.frame_id != camera_info->header.frame_id) {
      std::lock_guard<std::mutex> lock(mutex_);
      ++localization_failures_;
      last_error_ = "detection frame and CameraInfo frame do not match";
      return;
    }

    const double anchor_fraction =
        observation->class_id >= 0 &&
                static_cast<std::size_t>(observation->class_id) <
                    class_anchor_y_fractions_.size()
            ? class_anchor_y_fractions_[observation->class_id]
            : 0.5;
    const cv::Point2d anchor_pixel(
        observation->detection.bbox.center.x,
        observation->box.y + anchor_fraction * observation->box.height);
    if (anchor_pixel.x < 0.0 || anchor_pixel.y < 0.0 ||
        anchor_pixel.x >= camera_info->width ||
        anchor_pixel.y >= camera_info->height) {
      std::lock_guard<std::mutex> lock(mutex_);
      ++localization_failures_;
      last_error_ = "detection anchor is outside CameraInfo image bounds";
      return;
    }
    const cv::Point3d ray = camera_model.projectPixelTo3dRay(anchor_pixel);
    if (!std::isfinite(ray.x) || !std::isfinite(ray.y) || !std::isfinite(ray.z)) {
      std::lock_guard<std::mutex> lock(mutex_);
      ++localization_failures_;
      last_error_ = "invalid pixel ray";
      return;
    }
    observation->bearing.header.frame_id = camera_frame;
    tf2::Vector3 normalized_bearing(ray.x, ray.y, ray.z);
    if (normalized_bearing.length2() < 1.0e-12) {
      std::lock_guard<std::mutex> lock(mutex_);
      ++localization_failures_;
      last_error_ = "zero-length camera ray";
      return;
    }
    normalized_bearing.normalize();
    observation->bearing.vector.x = normalized_bearing.x();
    observation->bearing.vector.y = normalized_bearing.y();
    observation->bearing.vector.z = normalized_bearing.z();

    if (localization_mode_ == "bearing_only") {
      std::lock_guard<std::mutex> lock(mutex_);
      ++bearing_only_observations_;
      return;
    }

    geometry_msgs::TransformStamped transform;
    try {
      transform = tf_buffer_.lookupTransform(task_frame_, camera_frame,
                                             header.stamp,
                                             ros::Duration(tf_lookup_timeout_sec_));
    } catch (const tf2::TransformException &exception) {
      if (!tf_use_latest_on_failure_) {
        std::lock_guard<std::mutex> lock(mutex_);
        ++localization_failures_;
        last_error_ = std::string("TF at capture time unavailable: ") +
                      exception.what();
        return;
      }
      try {
        transform = tf_buffer_.lookupTransform(task_frame_, camera_frame,
                                               ros::Time(0),
                                               ros::Duration(tf_lookup_timeout_sec_));
        std::lock_guard<std::mutex> lock(mutex_);
        ++latest_tf_fallbacks_;
      } catch (const tf2::TransformException &latest_exception) {
        std::lock_guard<std::mutex> lock(mutex_);
        ++localization_failures_;
        last_error_ = std::string("TF unavailable: ") + latest_exception.what();
        return;
      }
    }

    tf2::Transform transform_tf;
    tf2::fromMsg(transform.transform, transform_tf);
    const tf2::Vector3 origin = transform_tf * tf2::Vector3(0.0, 0.0, 0.0);
    tf2::Vector3 direction = transform_tf.getBasis() * normalized_bearing;
    direction.normalize();
    tf2::Vector3 position;
    double range = 0.0;
    double surface_variance =
        ground_height_stddev_m_ * ground_height_stddev_m_;
    bool intersection_valid = false;
    if (localization_mode_ == "horizontal_plane") {
      intersection_valid = planeIntersection(origin, direction, ground_plane_z_,
                                             &position, &range);
    } else if (localization_mode_ == "terrain") {
      if (!terrain_grid) {
        std::lock_guard<std::mutex> lock(mutex_);
        ++localization_failures_;
        last_error_ = "terrain localization requested but DEM/DSM is unavailable";
        return;
      }
      if (!terrain_grid->header.stamp.isZero() && terrain_max_age_sec_ > 0.0 &&
          std::abs((header.stamp - terrain_grid->header.stamp).toSec()) >
              terrain_max_age_sec_) {
        std::lock_guard<std::mutex> lock(mutex_);
        ++localization_failures_;
        last_error_ = "DEM/DSM is stale for the capture timestamp";
        return;
      }
      intersection_valid = terrainIntersection(
          origin, direction, *terrain_grid, &position, &range,
          &surface_variance);
    }
    if (!intersection_valid) {
      std::lock_guard<std::mutex> lock(mutex_);
      ++localization_failures_;
      last_error_ = "camera ray has no valid surface intersection";
      return;
    }

    observation->localization_valid = true;
    observation->task_pose.header.stamp = header.stamp;
    observation->task_pose.header.frame_id = task_frame_;
    observation->task_pose.pose.position.x = position.x();
    observation->task_pose.pose.position.y = position.y();
    observation->task_pose.pose.position.z = position.z();
    computeLocalizationCovariance(camera_model, transform_tf, anchor_pixel,
                                  origin, position, range, surface_variance,
                                  &observation->covariance);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      ++localized_observations_;
    }
  }

  double associationCost(const Observation &observation,
                         const CandidateTrack &track) const {
    if (track.class_id != observation.class_id ||
        (ros::WallTime::now() - track.last_update_wall).toSec() >
            track_timeout_sec_) {
      return std::numeric_limits<double>::infinity();
    }
    if (track.source_track_id_valid && observation.source_track_id_valid) {
      return track.source_track_id == observation.source_track_id
                 ? 0.0
                 : std::numeric_limits<double>::infinity();
    }
    if (track.localization_valid && observation.localization_valid) {
      geometry_msgs::Point predicted = track.task_pose.pose.position;
      const double dt = std::max(
          0.0, (observation.stamp - track.last_seen).toSec());
      if (track.velocity_valid) {
        predicted.x += dt * track.velocity.x;
        predicted.y += dt * track.velocity.y;
        predicted.z += dt * track.velocity.z;
      }
      const geometry_msgs::Point &measured =
          observation.task_pose.pose.position;
      const double dx = measured.x - predicted.x;
      const double dy = measured.y - predicted.y;
      const double dz = measured.z - predicted.z;
      const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
      if (distance > association_world_distance_m_)
        return std::numeric_limits<double>::infinity();

      const double covariance_xx =
          std::max(1.0e-4, track.covariance[0] + observation.covariance[0]);
      const double covariance_xy =
          0.5 * (track.covariance[1] + track.covariance[3] +
                 observation.covariance[1] + observation.covariance[3]);
      const double covariance_yy =
          std::max(1.0e-4, track.covariance[4] + observation.covariance[4]);
      const double determinant =
          covariance_xx * covariance_yy - covariance_xy * covariance_xy;
      if (determinant <= 1.0e-9)
        return distance / association_world_distance_m_;
      const double mahalanobis =
          (covariance_yy * dx * dx - 2.0 * covariance_xy * dx * dy +
           covariance_xx * dy * dy) /
          determinant;
      if (!std::isfinite(mahalanobis) ||
          mahalanobis > association_mahalanobis_threshold_) {
        return std::numeric_limits<double>::infinity();
      }
      return std::sqrt(std::max(0.0, mahalanobis) /
                       association_mahalanobis_threshold_);
    }

    const double center_dx =
        track.last_box.x + 0.5 * track.last_box.width -
        (observation.box.x + 0.5 * observation.box.width);
    const double center_dy =
        track.last_box.y + 0.5 * track.last_box.height -
        (observation.box.y + 0.5 * observation.box.height);
    const double center_distance =
        std::sqrt(center_dx * center_dx + center_dy * center_dy);
    const double overlap = iou(track.last_box, observation.box);
    if (overlap < association_iou_threshold_ &&
        center_distance > association_center_distance_px_) {
      return std::numeric_limits<double>::infinity();
    }
    return std::max(0.0,
                    std::min(0.99, center_distance /
                                       association_center_distance_px_) -
                        0.25 * overlap);
  }

  CandidateTrack makeTrack(const Observation &observation) {
    CandidateTrack track;
    track.id = next_track_id_++;
    if (next_track_id_ == 0)
      next_track_id_ = 1;
    track.class_id = observation.class_id;
    track.class_name = observation.class_name;
    track.last_box = observation.box;
    track.localization_valid = observation.localization_valid;
    track.task_pose = observation.task_pose;
    track.bearing = observation.bearing;
    track.covariance = observation.covariance;
    track.confidence = observation.confidence;
    track.stability = static_cast<float>(0.5 * observation.confidence);
    track.observation_count = 1;
    track.consecutive_hits = 1;
    track.first_seen = observation.stamp;
    track.last_seen = observation.stamp;
    track.last_update_wall = ros::WallTime::now();
    track.hit_stamps.push_back(observation.stamp);
    track.source_track_id_valid = observation.source_track_id_valid;
    track.source_track_id = observation.source_track_id;
    ++created_tracks_;
    return track;
  }

  bool updateTrack(CandidateTrack *track, const Observation &observation) {
    const bool previously_confirmed = track->confirmed;
    track->hit_stamps.push_back(observation.stamp);
    while (!track->hit_stamps.empty() &&
           (observation.stamp - track->hit_stamps.front()).toSec() >
               confirmation_window_sec_) {
      track->hit_stamps.pop_front();
    }
    track->consecutive_hits =
        static_cast<std::uint32_t>(track->hit_stamps.size());
    track->observation_count += 1;
    const ros::Time previous_stamp = track->last_seen;
    track->last_seen = observation.stamp;
    track->last_update_wall = ros::WallTime::now();
    track->last_box = observation.box;
    track->bearing = observation.bearing;
    track->confidence = static_cast<float>(
        confidence_ema_alpha_ * observation.confidence +
        (1.0 - confidence_ema_alpha_) * track->confidence);
    if (observation.localization_valid) {
      if (track->localization_valid) {
        geometry_msgs::Point &smoothed = track->task_pose.pose.position;
        const geometry_msgs::Point &raw = observation.task_pose.pose.position;
        const double dt = (observation.stamp - previous_stamp).toSec();
        if (dt > 1.0e-3) {
          const geometry_msgs::Vector3 measured_velocity = [&]() {
            geometry_msgs::Vector3 velocity;
            velocity.x = (raw.x - smoothed.x) / dt;
            velocity.y = (raw.y - smoothed.y) / dt;
            velocity.z = (raw.z - smoothed.z) / dt;
            return velocity;
          }();
          const double velocity_alpha = 0.35;
          track->velocity.x = velocity_alpha * measured_velocity.x +
                              (1.0 - velocity_alpha) * track->velocity.x;
          track->velocity.y = velocity_alpha * measured_velocity.y +
                              (1.0 - velocity_alpha) * track->velocity.y;
          track->velocity.z = velocity_alpha * measured_velocity.z +
                              (1.0 - velocity_alpha) * track->velocity.z;
          track->velocity_valid = true;
        }
        smoothed.x = position_ema_alpha_ * raw.x +
                     (1.0 - position_ema_alpha_) * smoothed.x;
        smoothed.y = position_ema_alpha_ * raw.y +
                     (1.0 - position_ema_alpha_) * smoothed.y;
        smoothed.z = position_ema_alpha_ * raw.z +
                     (1.0 - position_ema_alpha_) * smoothed.z;
        track->task_pose.header = observation.task_pose.header;
      } else {
        track->task_pose = observation.task_pose;
      }
      track->localization_valid = true;
      track->covariance = observation.covariance;
    }
    if (observation.source_track_id_valid) {
      track->source_track_id_valid = true;
      track->source_track_id = observation.source_track_id;
    }
    const double temporal_stability = std::min(
        1.0, static_cast<double>(track->consecutive_hits) /
                 static_cast<double>(min_confirmations_));
    track->stability = static_cast<float>(
        clamp(0.5 * track->confidence + 0.5 * temporal_stability, 0.0, 1.0));
    const float priority = candidatePriority(*track);
    if (!track->confirmed && track->consecutive_hits >= min_confirmations_ &&
        priority >= minimum_task_priority_) {
      track->confirmed = true;
      ++confirmed_tracks_;
      return true;
    }
    return !previously_confirmed && track->confirmed;
  }

  float candidatePriority(const CandidateTrack &track) const {
    return static_cast<float>(clamp(track.confidence * track.stability, 0.0, 1.0));
  }

  TaskCandidate makeMessage(const CandidateTrack &track, const uint8_t status,
                            const vision_msgs::Detection2D *detection) {
    TaskCandidate message;
    message.header.stamp = track.last_seen;
    message.header.frame_id = track.localization_valid ? task_frame_
                                                        : track.bearing.header.frame_id;
    message.track_id = track.id;
    message.observation_uuid = observationUuid(identity_, track.id);
    message.event_sequence = next_event_sequence_++;
    message.provenance = identity_;
    message.class_id = static_cast<std::uint8_t>(std::max(0, track.class_id));
    message.class_name = track.class_name;
    message.semantic_type = semanticType(track.class_id);
    message.status = status;
    message.source_track_id_valid = track.source_track_id_valid;
    message.source_track_id = track.source_track_id;
    message.confidence = track.confidence;
    message.stability = track.stability;
    message.priority = candidatePriority(track);
    message.observation_count = track.observation_count;
    message.consecutive_hits = track.consecutive_hits;
    message.first_seen = track.first_seen;
    message.last_seen = track.last_seen;
    message.localization_valid = track.localization_valid;
    message.task_pose = track.task_pose;
    message.bearing = track.bearing;
    for (std::size_t index = 0; index < track.covariance.size(); ++index)
      message.position_covariance[index] = track.covariance[index];
    if (detection != nullptr)
      message.image_detection = *detection;
    message.sensor_id = sensor_id_;
    return message;
  }

  bool shouldPublishPose(const CandidateTrack &track, const bool newly_confirmed,
                         const geometry_msgs::Point &previous_point) const {
    if (!track.confirmed || !track.localization_valid ||
        candidatePriority(track) < minimum_task_priority_) {
      return false;
    }
    if (newly_confirmed || !track.pose_published)
      return true;
    if (pointDistance(previous_point, track.task_pose.pose.position) >=
        task_pose_update_distance_m_) {
      return true;
    }
    return task_pose_refresh_sec_ > 0.0 && !track.last_pose_publish_wall.isZero() &&
           (ros::WallTime::now() - track.last_pose_publish_wall).toSec() >=
               task_pose_refresh_sec_;
  }

  void detectionsCallback(const vision_msgs::Detection2DArray::ConstPtr &message) {
    std::vector<std::pair<bool, std::int64_t>> source_ids(
        message->detections.size(), std::make_pair(false, -1));
    processDetections(*message, source_ids);
  }

  void trackedDetectionsCallback(
      const TrackedDetection2DArray::ConstPtr &message) {
    if (!message->provenance.uav_id.empty() &&
        message->provenance.uav_id != identity_.uav_id) {
      ++rejected_input_messages_;
      last_error_ = "tracked detections belong to a different uav_id";
      return;
    }
    vision_msgs::Detection2DArray converted;
    converted.header = message->header;
    std::vector<std::pair<bool, std::int64_t>> source_ids;
    converted.detections.reserve(message->detections.size());
    source_ids.reserve(message->detections.size());
    for (const auto &tracked : message->detections) {
      converted.detections.push_back(tracked.detection);
      source_ids.emplace_back(tracked.track_id_is_stable, tracked.track_id);
    }
    processDetections(converted, source_ids);
  }

  void processDetections(
      const vision_msgs::Detection2DArray &message,
      const std::vector<std::pair<bool, std::int64_t>> &source_ids) {
    // ros::spin() serializes state updates. TF and DEM lookups deliberately run
    // without holding the state mutex so diagnostics and calibration callbacks
    // cannot self-deadlock if the spinner is changed later.
    ++input_messages_;
    if (!inputAccepted(message)) {
      ROS_WARN_STREAM_THROTTLE(2.0, "[SarTaskPoint] Rejected input: " << last_error_);
      return;
    }
    TaskCandidateArray output;
    output.header = message.header;
    output.array_sequence = next_array_sequence_++;
    output.full_snapshot = false;
    output.provenance = identity_;
    if (!task_generation_active_) {
      candidates_publisher_.publish(output);
      return;
    }
    std::vector<Observation> observations;
    observations.reserve(message.detections.size());
    for (std::size_t index = 0; index < message.detections.size(); ++index) {
      Observation observation;
      const bool stable_id = index < source_ids.size() && source_ids[index].first;
      const std::int64_t source_id =
          index < source_ids.size() ? source_ids[index].second : -1;
      if (extractObservation(message.detections[index], message.header, stable_id,
                             source_id, &observation)) {
        observations.push_back(std::move(observation));
      }
    }
    std::sort(observations.begin(), observations.end(),
              [](const Observation &left, const Observation &right) {
                return left.confidence > right.confidence;
              });

    std::vector<std::vector<double>> costs(
        observations.size(), std::vector<double>(tracks_.size(),
                                                 std::numeric_limits<double>::infinity()));
    for (std::size_t observation_index = 0;
         observation_index < observations.size(); ++observation_index) {
      for (std::size_t track_index = 0; track_index < tracks_.size();
           ++track_index) {
        costs[observation_index][track_index] =
            associationCost(observations[observation_index], tracks_[track_index]);
      }
    }
    const std::vector<int> assignments =
        minimumCostAssignment(costs, 1.0);
    std::vector<geometry_msgs::PoseStamped> poses_to_publish;
    for (std::size_t observation_index = 0;
         observation_index < observations.size(); ++observation_index) {
      const Observation &observation = observations[observation_index];
      int index = assignments[observation_index];
      const bool created_track = index < 0;
      bool newly_confirmed = false;
      if (created_track) {
        tracks_.push_back(makeTrack(observation));
        index = static_cast<int>(tracks_.size() - 1);
        CandidateTrack &track = tracks_[index];
        const double temporal = std::min(
            1.0, static_cast<double>(track.consecutive_hits) /
                     static_cast<double>(min_confirmations_));
        track.stability = static_cast<float>(
            clamp(0.5 * track.confidence + 0.5 * temporal, 0.0, 1.0));
        if (track.consecutive_hits >= min_confirmations_ &&
            candidatePriority(track) >= minimum_task_priority_) {
          track.confirmed = true;
          newly_confirmed = true;
          ++confirmed_tracks_;
        }
      }
      CandidateTrack &track = tracks_[static_cast<std::size_t>(index)];
      const geometry_msgs::Point previous_point = track.task_pose.pose.position;
      if (!created_track)
        newly_confirmed = updateTrack(&track, observation) || newly_confirmed;

      if (track.confirmed || publish_pending_candidates_) {
        const uint8_t status = newly_confirmed
                                   ? TaskCandidate::CONFIRMED
                                   : (track.confirmed ? TaskCandidate::UPDATED
                                                      : TaskCandidate::PENDING);
        output.candidates.push_back(makeMessage(track, status, &observation.detection));
      }
      if (shouldPublishPose(track, newly_confirmed, previous_point)) {
        geometry_msgs::PoseStamped pose = track.task_pose;
        // roscpp owns Header::seq and overwrites it on publication.  The
        // authoritative candidate identity therefore remains track_id in the
        // companion TaskCandidate message, not PoseStamped::header.seq.
        poses_to_publish.push_back(pose);
        track.pose_published = true;
        track.last_pose_publish_wall = ros::WallTime::now();
        ++published_task_poses_;
      }
    }
    candidates_publisher_.publish(output);
    published_candidate_messages_ += output.candidates.size();
    for (const auto &pose : poses_to_publish)
      task_pose_publisher_.publish(pose);
    last_error_.clear();
  }

  void expirationTimerCallback(const ros::WallTimerEvent &) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (tracks_.empty())
      return;
    TaskCandidateArray expired;
    expired.header.stamp = ros::Time::now();
    expired.header.frame_id = task_frame_;
    expired.array_sequence = next_array_sequence_++;
    expired.full_snapshot = false;
    expired.provenance = identity_;
    for (auto iterator = tracks_.begin(); iterator != tracks_.end();) {
      if ((ros::WallTime::now() - iterator->last_update_wall).toSec() <=
          track_timeout_sec_) {
        ++iterator;
        continue;
      }
      if (iterator->confirmed) {
        expired.candidates.push_back(
            makeMessage(*iterator, TaskCandidate::EXPIRED, nullptr));
        ++expired_tracks_;
      }
      iterator = tracks_.erase(iterator);
    }
    if (!expired.candidates.empty()) {
      candidates_publisher_.publish(expired);
      published_candidate_messages_ += expired.candidates.size();
    }
  }

  void snapshotTimerCallback(const ros::WallTimerEvent &) {
    std::lock_guard<std::mutex> lock(mutex_);
    TaskCandidateArray snapshot = currentSnapshot();
    candidates_publisher_.publish(snapshot);
    ++published_snapshots_;
  }

  TaskCandidateArray currentSnapshot() {
    TaskCandidateArray snapshot;
    snapshot.header.stamp = ros::Time::now();
    snapshot.header.frame_id = task_frame_;
    snapshot.array_sequence = next_array_sequence_++;
    snapshot.full_snapshot = true;
    snapshot.provenance = identity_;
    if (task_generation_active_) {
      snapshot.candidates.reserve(tracks_.size());
      for (const CandidateTrack &track : tracks_) {
        if (track.confirmed || publish_pending_candidates_) {
          snapshot.candidates.push_back(makeMessage(
              track, track.confirmed ? TaskCandidate::UPDATED
                                     : TaskCandidate::PENDING,
              nullptr));
        }
      }
    }
    return snapshot;
  }

  bool snapshotServiceCallback(GetTaskCandidates::Request &,
                               GetTaskCandidates::Response &response) {
    std::lock_guard<std::mutex> lock(mutex_);
    response.snapshot = currentSnapshot();
    return true;
  }

  void diagnosticTimerCallback(const ros::WallTimerEvent &) {
    diagnostics_.update();
  }

  void diagnosticCallback(diagnostic_updater::DiagnosticStatusWrapper &status) {
    std::lock_guard<std::mutex> lock(mutex_);
    int level = diagnostic_msgs::DiagnosticStatus::OK;
    std::string summary = "Task candidate generator running";
    if (!task_generation_active_ && task_generation_enabled_) {
      level = diagnostic_msgs::DiagnosticStatus::ERROR;
      summary = "Task generation blocked by model validation or class policy";
    } else if (localization_mode_ != "bearing_only" && !have_camera_model_) {
      level = diagnostic_msgs::DiagnosticStatus::WARN;
      summary = "Waiting for camera calibration; bearing-only candidates available";
    } else if (localization_failures_ > localized_observations_ &&
               localization_failures_ > 0) {
      level = diagnostic_msgs::DiagnosticStatus::WARN;
      summary = "Most candidate observations cannot be localised";
    }
    status.summary(level, summary);
    status.add("localization_mode", localization_mode_);
    status.add("task_frame", task_frame_);
    status.add("mission_id", identity_.mission_id);
    status.add("uav_id", identity_.uav_id);
    status.add("session_uuid", identity_.session_uuid);
    status.add("profile", identity_.profile);
    status.add("model_version", identity_.model_version);
    status.add("model_sha256", identity_.model_sha256);
    status.add("calibration_version", identity_.calibration_version);
    status.add("coordinate_contract", identity_.coordinate_contract);
    status.add("task_generation_enabled", task_generation_enabled_);
    status.add("task_generation_active", task_generation_active_);
    status.add("model_validated", model_validated_);
    status.add("allow_experimental_model", allow_experimental_model_);
    status.add("use_tracked_detections", use_tracked_detections_);
    status.add("camera_model_available", have_camera_model_);
    status.add("camera_info_history",
               static_cast<long long>(camera_info_history_.size()));
    status.add("terrain_grid_available", static_cast<bool>(terrain_grid_));
    status.add("invalid_terrain_grids",
               static_cast<long long>(invalid_terrain_grids_));
    status.add("active_tracks", static_cast<long long>(tracks_.size()));
    status.add("input_messages", static_cast<long long>(input_messages_));
    status.add("rejected_input_messages", static_cast<long long>(rejected_input_messages_));
    status.add("filtered_detections", static_cast<long long>(filtered_detections_));
    status.add("rejected_detections", static_cast<long long>(rejected_detections_));
    status.add("created_tracks", static_cast<long long>(created_tracks_));
    status.add("confirmed_tracks", static_cast<long long>(confirmed_tracks_));
    status.add("expired_tracks", static_cast<long long>(expired_tracks_));
    status.add("localized_observations", static_cast<long long>(localized_observations_));
    status.add("localization_failures", static_cast<long long>(localization_failures_));
    status.add("latest_tf_fallbacks", static_cast<long long>(latest_tf_fallbacks_));
    status.add("bearing_only_observations", static_cast<long long>(bearing_only_observations_));
    status.add("published_candidate_messages",
               static_cast<long long>(published_candidate_messages_));
    status.add("published_task_poses", static_cast<long long>(published_task_poses_));
    status.add("published_snapshots", static_cast<long long>(published_snapshots_));
    status.add("last_error", last_error_);
  }

  std::string className(const int class_id) const {
    if (class_id >= 0 && static_cast<std::size_t>(class_id) < class_names_.size())
      return class_names_[static_cast<std::size_t>(class_id)];
    return "class_" + std::to_string(class_id);
  }

  std::uint8_t semanticType(const int class_id) const {
    if (class_id >= 0 && static_cast<std::size_t>(class_id) <
                             global_semantic_ids_.size()) {
      return static_cast<std::uint8_t>(
          std::max(0, std::min(255, global_semantic_ids_[class_id])));
    }
    return TaskCandidate::SEMANTIC_UNKNOWN;
  }

  ros::NodeHandle node_;
  ros::NodeHandle private_node_;
  ros::Subscriber detections_subscriber_;
  ros::Subscriber tracked_detections_subscriber_;
  ros::Subscriber camera_info_subscriber_;
  ros::Subscriber terrain_subscriber_;
  ros::Publisher candidates_publisher_;
  ros::Publisher task_pose_publisher_;
  ros::ServiceServer snapshot_service_;
  ros::WallTimer expiration_timer_;
  ros::WallTimer diagnostic_timer_;
  ros::WallTimer snapshot_timer_;
  diagnostic_updater::Updater diagnostics_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  mutable std::mutex mutex_;

  std::deque<sensor_msgs::CameraInfoConstPtr> camera_info_history_;
  TerrainGrid::ConstPtr terrain_grid_;
  bool have_camera_model_{false};
  std::vector<CandidateTrack> tracks_;
  std::uint32_t next_track_id_{1};
  std::uint64_t next_event_sequence_{1};
  std::uint64_t next_array_sequence_{1};
  ros::Time last_input_stamp_;
  PerceptionIdentity identity_;

  std::string input_detections_topic_, camera_info_topic_, candidates_topic_;
  std::string tracked_detections_topic_, task_pose_topic_, sensor_id_;
  std::string task_frame_, localization_mode_, terrain_grid_topic_;
  std::vector<int> task_class_ids_;
  std::vector<int> global_semantic_ids_;
  std::vector<std::string> class_names_;
  std::vector<double> class_anchor_y_fractions_;
  double ground_plane_z_{0.0}, tf_lookup_timeout_sec_{0.05};
  bool tf_use_latest_on_failure_{false};
  double minimum_intersection_range_m_{1.0}, maximum_intersection_range_m_{300.0};
  double pixel_measurement_stddev_px_{2.0}, ground_height_stddev_m_{1.5};
  double camera_position_stddev_m_{0.30}, camera_attitude_stddev_deg_{0.50};
  double calibration_angular_stddev_deg_{0.25};
  double time_sync_stddev_sec_{0.010}, vehicle_velocity_stddev_mps_{2.0};
  double camera_info_max_delta_sec_{0.10};
  bool camera_info_allow_static_{true};
  double terrain_max_age_sec_{30.0}, terrain_ray_step_m_{2.0};
  bool require_matching_camera_frame_{true};
  bool require_detection_timestamp_{true}, reject_out_of_order_{true};
  double input_max_age_sec_{1.0}, future_timestamp_tolerance_sec_{0.02};
  double minimum_detection_confidence_{0.40};
  int min_confirmations_{3};
  double confirmation_window_sec_{1.0}, track_timeout_sec_{2.0};
  double association_iou_threshold_{0.10}, association_center_distance_px_{120.0};
  double association_world_distance_m_{12.0};
  double association_mahalanobis_threshold_{9.21};
  double confidence_ema_alpha_{0.45};
  double position_ema_alpha_{0.35}, minimum_task_priority_{0.55};
  double task_pose_update_distance_m_{2.0}, task_pose_refresh_sec_{3.0};
  bool publish_pending_candidates_{true};
  double expiration_check_period_sec_{0.20};
  double snapshot_period_sec_{1.0};
  bool use_tracked_detections_{false};
  bool task_generation_enabled_{false}, model_validated_{false};
  bool allow_experimental_model_{false}, all_task_classes_{false};
  bool task_generation_active_{false};

  std::uint64_t input_messages_{0}, rejected_input_messages_{0};
  std::uint64_t filtered_detections_{0}, rejected_detections_{0};
  std::uint64_t invalid_camera_info_{0}, created_tracks_{0}, confirmed_tracks_{0};
  std::uint64_t expired_tracks_{0}, localized_observations_{0};
  std::uint64_t localization_failures_{0}, latest_tf_fallbacks_{0};
  std::uint64_t invalid_terrain_grids_{0};
  std::uint64_t bearing_only_observations_{0}, published_candidate_messages_{0};
  std::uint64_t published_task_poses_{0}, published_snapshots_{0};
  std::string last_error_;
};

} // namespace sar_yolo_detector

int main(int argc, char **argv) {
  ros::init(argc, argv, "sar_task_point_generator");
  try {
    sar_yolo_detector::TaskPointGeneratorNode node;
    ros::spin();
  } catch (const std::exception &exception) {
    std::cerr << "[SarTaskPoint] Startup failed: " << exception.what()
              << std::endl;
    ROS_FATAL_STREAM("[SarTaskPoint] Startup failed: " << exception.what());
    return 1;
  }
  return 0;
}
