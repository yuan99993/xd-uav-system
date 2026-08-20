#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <deque>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <XmlRpcValue.h>
#include <cv_bridge/cv_bridge.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/TransformStamped.h>
#include <image_geometry/pinhole_camera_model.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <openssl/sha.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <sar_yolo_detector/FloodRegion.h>
#include <sar_yolo_detector/FloodRegionArray.h>
#include <sar_yolo_detector/GetFloodRegions.h>
#include <sar_mission_interfaces/GetEvidenceCrop.h>
#include <sar_yolo_detector/TerrainGrid.h>
#include <sar_yolo_detector/runtime_identity.hpp>

namespace sar_yolo_detector {
namespace {

double Clamp(const double value, const double minimum, const double maximum) {
  return std::max(minimum, std::min(value, maximum));
}

double IntersectionOverUnion(const cv::Rect& left, const cv::Rect& right) {
  const cv::Rect intersection = left & right;
  const double intersection_area = static_cast<double>(intersection.area());
  const double union_area = static_cast<double>(left.area() + right.area()) -
                            intersection_area;
  return union_area > 0.0 ? intersection_area / union_area : 0.0;
}

std::vector<int> ReadIntegerArray(ros::NodeHandle* node_handle,
                                  const std::string& key,
                                  const std::vector<int>& default_value) {
  XmlRpc::XmlRpcValue value;
  if (!node_handle->getParam(key, value)) {
    return default_value;
  }
  if (value.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    throw std::invalid_argument("Parameter '" + key + "' must be an array.");
  }

  std::vector<int> result;
  result.reserve(value.size());
  for (int index = 0; index < value.size(); ++index) {
    if (value[index].getType() != XmlRpc::XmlRpcValue::TypeInt) {
      throw std::invalid_argument("Parameter '" + key + "' must contain integers.");
    }
    result.push_back(static_cast<int>(value[index]));
  }
  return result;
}

std::vector<std::string> ReadStringArray(ros::NodeHandle* node_handle,
                                         const std::string& key) {
  XmlRpc::XmlRpcValue value;
  if (!node_handle->getParam(key, value)) {
    return std::vector<std::string>();
  }
  if (value.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    throw std::invalid_argument("Parameter '" + key + "' must be an array.");
  }

  std::vector<std::string> result;
  result.reserve(value.size());
  for (int index = 0; index < value.size(); ++index) {
    if (value[index].getType() != XmlRpc::XmlRpcValue::TypeString) {
      throw std::invalid_argument("Parameter '" + key + "' must contain strings.");
    }
    result.push_back(static_cast<std::string>(value[index]));
  }
  return result;
}

struct Observation {
  int class_id;
  cv::Rect bounding_box;
  std::vector<cv::Point> contour;
  double area_px;
};

struct RegionTrack {
  uint32_t id;
  int class_id;
  cv::Rect bounding_box;
  std::vector<cv::Point> contour;
  double area_px;
  float confidence;
  uint32_t observations;
  uint32_t consecutive_hits;
  bool confirmed;
  ros::Time first_seen;
  ros::Time last_seen;
  ros::WallTime last_update_wall;
  std::deque<ros::Time> hit_stamps;
  bool localization_valid{false};
  geometry_msgs::PoseStamped map_centroid;
  geometry_msgs::PolygonStamped world_polygon;
  std::array<double, 9> covariance{{0.0, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.0}};
  double world_area_m2{0.0};
  float world_projection_coverage{0.0F};
  cv::Rect last_published_box;
  ros::WallTime last_publish_wall;
};

struct EvidenceFrame {
  sensor_msgs::ImageConstPtr image;
  std::string sha256;
};

struct EvidenceCrop {
  sensor_msgs::CompressedImage image;
  std::string source_image_sha256;
  sensor_msgs::RegionOfInterest roi;
};

}  // namespace

class FloodRegionExtractor {
 public:
  FloodRegionExtractor()
      : node_handle_(),
        private_node_handle_("~"),
        tf_listener_(tf_buffer_),
        next_track_id_(1U),
        morphology_kernel_px_(0),
        minimum_region_area_px_(600.0),
        association_iou_threshold_(0.25),
        min_confirmations_(3U),
        confirmation_window_sec_(2.0),
        track_timeout_sec_(4.0),
        confidence_ema_alpha_(0.45),
        default_mask_confidence_(0.70),
        publish_pending_regions_(true),
        require_mask_timestamp_(true),
        reject_out_of_order_(true),
        input_max_age_sec_(1.0),
        future_timestamp_tolerance_sec_(0.02) {
    LoadParameters();
    publisher_ = node_handle_.advertise<FloodRegionArray>(regions_topic_, 10);
    snapshot_service_ = node_handle_.advertiseService(
        regions_topic_ + "/get_snapshot",
        &FloodRegionExtractor::HandleSnapshotService, this);
    subscriber_ = node_handle_.subscribe(input_mask_topic_, 3,
                                         &FloodRegionExtractor::HandleMask, this);
    camera_info_subscriber_ = node_handle_.subscribe(
        camera_info_topic_, 10, &FloodRegionExtractor::HandleCameraInfo, this);
    evidence_subscriber_ = node_handle_.subscribe(
        evidence_source_topic_, 5, &FloodRegionExtractor::HandleEvidenceImage, this);
    evidence_service_ = node_handle_.advertiseService(
        regions_topic_ + "/get_evidence",
        &FloodRegionExtractor::HandleEvidenceService, this);
    if (localization_mode_ == "terrain") {
      terrain_subscriber_ = node_handle_.subscribe(
          terrain_grid_topic_, 1, &FloodRegionExtractor::HandleTerrain, this);
    }
    expiration_timer_ = node_handle_.createWallTimer(
        ros::WallDuration(std::max(0.05, expiration_check_period_sec_)),
        &FloodRegionExtractor::HandleExpirationTimer, this);
    snapshot_timer_ = node_handle_.createWallTimer(
        ros::WallDuration(std::max(0.1, snapshot_period_sec_)),
        &FloodRegionExtractor::HandleSnapshotTimer, this);
    diagnostic_timer_ = node_handle_.createWallTimer(
        ros::WallDuration(1.0), &FloodRegionExtractor::HandleDiagnosticTimer,
        this);
    diagnostics_.setHardwareID("sar_flood_" + identity_.uav_id + "_" +
                               identity_.sensor_id);
    diagnostics_.add("flood_region_extractor", this,
                     &FloodRegionExtractor::Diagnostics);
    ROS_INFO_STREAM("sar_flood_region_extractor ready: " << input_mask_topic_
                    << " -> " << regions_topic_ << ", classes="
                    << hazard_class_ids_.size() << ", localization_mode="
                    << localization_mode_);
  }

 private:
  void LoadParameters() {
    private_node_handle_.param("input_mask_topic", input_mask_topic_,
                               std::string("sar_yolo_detector/floodnet/class_mask"));
    private_node_handle_.param("regions_topic", regions_topic_,
                               std::string("sar_yolo_detector/floodnet/regions"));
    private_node_handle_.param("sensor_id", sensor_id_, std::string("floodnet_eo_primary"));
    private_node_handle_.param("task_frame", task_frame_, std::string("map"));
    private_node_handle_.param("camera_info_topic", camera_info_topic_,
                               std::string("camera/eo/camera_info"));
    private_node_handle_.param("terrain_grid_topic", terrain_grid_topic_,
                               std::string("terrain/grid"));
    private_node_handle_.param("evidence_source_topic", evidence_source_topic_,
                               std::string("camera/eo/image_raw"));
    private_node_handle_.param("evidence_image_max_delta_sec",
                               evidence_image_max_delta_sec_, 0.10);
    private_node_handle_.param("evidence_frame_cache_size",
                               evidence_frame_cache_size_, 12);
    private_node_handle_.param("evidence_crop_cache_size",
                               evidence_crop_cache_size_, 256);
    private_node_handle_.param("evidence_jpeg_quality", evidence_jpeg_quality_, 85);
    private_node_handle_.param("morphology_kernel_px", morphology_kernel_px_, 3);
    private_node_handle_.param("minimum_region_area_px", minimum_region_area_px_, 600.0);
    private_node_handle_.param("association_iou_threshold", association_iou_threshold_, 0.25);
    int min_confirmations_parameter = static_cast<int>(min_confirmations_);
    private_node_handle_.param("min_confirmations", min_confirmations_parameter, 3);
    private_node_handle_.param("confirmation_window_sec", confirmation_window_sec_, 2.0);
    private_node_handle_.param("track_timeout_sec", track_timeout_sec_, 4.0);
    private_node_handle_.param("confidence_ema_alpha", confidence_ema_alpha_, 0.45);
    private_node_handle_.param("default_mask_confidence", default_mask_confidence_, 0.70);
    private_node_handle_.param("publish_pending_regions", publish_pending_regions_, true);
    private_node_handle_.param("require_mask_timestamp", require_mask_timestamp_, true);
    private_node_handle_.param("reject_out_of_order", reject_out_of_order_, true);
    private_node_handle_.param("input_max_age_sec", input_max_age_sec_, 1.0);
    private_node_handle_.param("future_timestamp_tolerance_sec",
                               future_timestamp_tolerance_sec_, 0.02);
    private_node_handle_.param("localization_mode", localization_mode_,
                               std::string("bearing_only"));
    private_node_handle_.param("ground_plane_z", ground_plane_z_, 0.0);
    private_node_handle_.param("camera_info_max_delta_sec",
                               camera_info_max_delta_sec_, 0.05);
    private_node_handle_.param("camera_info_allow_static",
                               camera_info_allow_static_, true);
    private_node_handle_.param("tf_lookup_timeout_sec", tf_lookup_timeout_sec_, 0.05);
    private_node_handle_.param("terrain_max_age_sec", terrain_max_age_sec_, 30.0);
    private_node_handle_.param("minimum_intersection_range_m",
                               minimum_intersection_range_m_, 1.0);
    private_node_handle_.param("maximum_intersection_range_m",
                               maximum_intersection_range_m_, 5000.0);
    private_node_handle_.param("terrain_ray_step_m", terrain_ray_step_m_, 2.0);
    private_node_handle_.param("minimum_projection_coverage",
                               minimum_projection_coverage_, 0.75);
    private_node_handle_.param("projection_position_stddev_m",
                               projection_position_stddev_m_, 1.5);
    private_node_handle_.param("projection_attitude_stddev_deg",
                               projection_attitude_stddev_deg_, 1.0);
    private_node_handle_.param("ground_height_stddev_m",
                               ground_height_stddev_m_, 1.0);
    private_node_handle_.param("contour_simplification_ratio",
                               contour_simplification_ratio_, 0.0025);
    private_node_handle_.param("maximum_contour_vertices",
                               maximum_contour_vertices_, 96);
    private_node_handle_.param("minimum_region_update_period_sec",
                               minimum_region_update_period_sec_, 0.5);
    private_node_handle_.param("region_update_iou_threshold",
                               region_update_iou_threshold_, 0.90);
    private_node_handle_.param("expiration_check_period_sec",
                               expiration_check_period_sec_, 0.20);
    private_node_handle_.param("snapshot_period_sec", snapshot_period_sec_, 1.0);

    hazard_class_ids_ = ReadIntegerArray(&private_node_handle_, "hazard_class_ids",
                                         std::vector<int>{1, 3, 5});
    class_names_ = ReadStringArray(&private_node_handle_, "class_names");
    global_semantic_ids_ = ReadIntegerArray(
        &private_node_handle_, "global_semantic_ids", std::vector<int>());
    if (input_mask_topic_.empty() || regions_topic_.empty()) {
      throw std::invalid_argument("input_mask_topic and regions_topic must not be empty.");
    }
    if (hazard_class_ids_.empty()) {
      throw std::invalid_argument("hazard_class_ids must contain at least one class.");
    }
    if (localization_mode_ != "bearing_only" &&
        localization_mode_ != "horizontal_plane" &&
        localization_mode_ != "terrain") {
      throw std::invalid_argument(
          "localization_mode must be bearing_only, horizontal_plane, or terrain");
    }

    morphology_kernel_px_ = std::max(0, morphology_kernel_px_);
    if (morphology_kernel_px_ > 0 && morphology_kernel_px_ % 2 == 0) {
      ++morphology_kernel_px_;
    }
    minimum_region_area_px_ = std::max(1.0, minimum_region_area_px_);
    association_iou_threshold_ = Clamp(association_iou_threshold_, 0.0, 1.0);
    min_confirmations_ = static_cast<uint32_t>(std::max(1, min_confirmations_parameter));
    confirmation_window_sec_ = std::max(0.0, confirmation_window_sec_);
    track_timeout_sec_ = std::max(0.1, track_timeout_sec_);
    confidence_ema_alpha_ = Clamp(confidence_ema_alpha_, 0.0, 1.0);
    default_mask_confidence_ = Clamp(default_mask_confidence_, 0.0, 1.0);
    input_max_age_sec_ = std::max(0.0, input_max_age_sec_);
    future_timestamp_tolerance_sec_ = std::max(0.0, future_timestamp_tolerance_sec_);
    expiration_check_period_sec_ = std::max(0.05, expiration_check_period_sec_);
    snapshot_period_sec_ = std::max(0.1, snapshot_period_sec_);
    camera_info_max_delta_sec_ = std::max(0.0, camera_info_max_delta_sec_);
    tf_lookup_timeout_sec_ = std::max(0.0, tf_lookup_timeout_sec_);
    terrain_max_age_sec_ = std::max(0.0, terrain_max_age_sec_);
    minimum_intersection_range_m_ = std::max(0.0, minimum_intersection_range_m_);
    maximum_intersection_range_m_ = std::max(
        minimum_intersection_range_m_ + 1.0, maximum_intersection_range_m_);
    terrain_ray_step_m_ = std::max(0.1, terrain_ray_step_m_);
    minimum_projection_coverage_ = Clamp(minimum_projection_coverage_, 0.0, 1.0);
    projection_position_stddev_m_ = std::max(0.0, projection_position_stddev_m_);
    projection_attitude_stddev_deg_ = std::max(0.0, projection_attitude_stddev_deg_);
    ground_height_stddev_m_ = std::max(0.0, ground_height_stddev_m_);
    contour_simplification_ratio_ = Clamp(contour_simplification_ratio_, 0.0, 0.1);
    maximum_contour_vertices_ = std::max(3, maximum_contour_vertices_);
    minimum_region_update_period_sec_ = std::max(0.0, minimum_region_update_period_sec_);
    region_update_iou_threshold_ = Clamp(region_update_iou_threshold_, 0.0, 1.0);
    evidence_image_max_delta_sec_ = std::max(0.0, evidence_image_max_delta_sec_);
    evidence_frame_cache_size_ = std::max(1, evidence_frame_cache_size_);
    evidence_crop_cache_size_ = std::max(1, evidence_crop_cache_size_);
    evidence_jpeg_quality_ = std::max(1, std::min(100, evidence_jpeg_quality_));
    if (!global_semantic_ids_.empty() &&
        global_semantic_ids_.size() != class_names_.size()) {
      throw std::invalid_argument(
          "global_semantic_ids must be empty or align with class_names.");
    }
    identity_ = loadPerceptionIdentity(
        &private_node_handle_, sensor_id_, "floodnet_segformer_b0",
        localization_mode_ == "bearing_only" ? "CAMERA_RAY"
                                               : "LOCAL_ENU:" + task_frame_);
    sensor_id_ = identity_.sensor_id;
  }

  void HandleCameraInfo(const sensor_msgs::CameraInfoConstPtr& message) {
    if (message->width == 0U || message->height == 0U ||
        message->K[0] <= 0.0 || message->K[4] <= 0.0) {
      ++invalid_camera_info_;
      return;
    }
    camera_info_history_.push_back(message);
    while (camera_info_history_.size() > 50U) camera_info_history_.pop_front();
  }

  void HandleEvidenceImage(const sensor_msgs::ImageConstPtr& message) {
    if (message->header.stamp.isZero() || message->data.empty()) return;
    unsigned char digest[SHA256_DIGEST_LENGTH];
    SHA256(message->data.data(), message->data.size(), digest);
    std::ostringstream stream;
    stream << std::hex << std::setfill('0');
    for (const unsigned char byte : digest)
      stream << std::setw(2) << static_cast<unsigned int>(byte);
    evidence_frames_.push_back(EvidenceFrame{message, stream.str()});
    while (evidence_frames_.size() >
           static_cast<std::size_t>(evidence_frame_cache_size_))
      evidence_frames_.pop_front();
  }

  const EvidenceFrame* EvidenceFrameForStamp(const ros::Time& stamp) const {
    const EvidenceFrame* best = nullptr;
    double best_delta = std::numeric_limits<double>::infinity();
    for (const auto& frame : evidence_frames_) {
      const double delta = std::abs((frame.image->header.stamp - stamp).toSec());
      if (delta < best_delta) {
        best = &frame;
        best_delta = delta;
      }
    }
    return best != nullptr && best_delta <= evidence_image_max_delta_sec_
        ? best : nullptr;
  }

  void CacheEvidenceCrop(const std::string& evidence_uuid,
                         const ros::Time& stamp,
                         const sensor_msgs::RegionOfInterest& roi,
                         std::string* source_image_sha256) {
    const auto cached = evidence_crops_.find(evidence_uuid);
    if (cached != evidence_crops_.end()) {
      *source_image_sha256 = cached->second.source_image_sha256;
      return;
    }
    const EvidenceFrame* frame = EvidenceFrameForStamp(stamp);
    if (frame == nullptr) return;
    *source_image_sha256 = frame->sha256;
    try {
      const cv_bridge::CvImageConstPtr image = cv_bridge::toCvShare(
          frame->image, sensor_msgs::image_encodings::BGR8);
      const cv::Rect bounds(0, 0, image->image.cols, image->image.rows);
      const cv::Rect requested(static_cast<int>(roi.x_offset),
                               static_cast<int>(roi.y_offset),
                               static_cast<int>(roi.width),
                               static_cast<int>(roi.height));
      const cv::Rect clipped = bounds & requested;
      if (clipped.area() <= 0) return;
      EvidenceCrop crop;
      crop.image.header = frame->image->header;
      crop.image.format = "jpeg";
      crop.source_image_sha256 = frame->sha256;
      crop.roi = roi;
      const std::vector<int> parameters{cv::IMWRITE_JPEG_QUALITY,
                                        evidence_jpeg_quality_};
      if (!cv::imencode(".jpg", image->image(clipped), crop.image.data,
                        parameters)) return;
      evidence_crops_[evidence_uuid] = std::move(crop);
      evidence_crop_order_.push_back(evidence_uuid);
      while (evidence_crop_order_.size() >
             static_cast<std::size_t>(evidence_crop_cache_size_)) {
        evidence_crops_.erase(evidence_crop_order_.front());
        evidence_crop_order_.pop_front();
      }
    } catch (const cv_bridge::Exception&) {
      ++evidence_failures_;
    }
  }

  bool HandleEvidenceService(
      sar_mission_interfaces::GetEvidenceCrop::Request& request,
      sar_mission_interfaces::GetEvidenceCrop::Response& response) {
    response.available = false;
    if (request.source.mission_id != identity_.mission_id ||
        request.source.uav_id != identity_.uav_id ||
        request.source.session_uuid != identity_.session_uuid) return true;
    const auto found = evidence_crops_.find(request.evidence_uuid);
    if (found == evidence_crops_.end()) return true;
    response.available = true;
    response.image = found->second.image;
    response.source_image_sha256 = found->second.source_image_sha256;
    response.roi = found->second.roi;
    return true;
  }

  void HandleTerrain(const TerrainGrid::ConstPtr& message) {
    const std::size_t expected = static_cast<std::size_t>(message->width) *
                                 static_cast<std::size_t>(message->height);
    if (message->header.frame_id != task_frame_ || message->resolution <= 0.0F ||
        message->width < 2U || message->height < 2U ||
        message->elevation.size() != expected ||
        (!message->variance.empty() && message->variance.size() != expected)) {
      ++invalid_terrain_grids_;
      return;
    }
    terrain_grid_ = message;
  }

  sensor_msgs::CameraInfoConstPtr CameraInfoForStamp(const ros::Time& stamp) const {
    sensor_msgs::CameraInfoConstPtr best;
    double best_delta = std::numeric_limits<double>::infinity();
    for (const auto& candidate : camera_info_history_) {
      if (candidate->header.stamp.isZero()) {
        if (camera_info_allow_static_ && !best) best = candidate;
        continue;
      }
      const double delta = std::abs((candidate->header.stamp - stamp).toSec());
      if (delta < best_delta) {
        best_delta = delta;
        best = candidate;
      }
    }
    if (best && !best->header.stamp.isZero() &&
        best_delta > camera_info_max_delta_sec_) return sensor_msgs::CameraInfoConstPtr();
    return best;
  }

  bool PlaneIntersection(const tf2::Vector3& origin,
                         const tf2::Vector3& direction,
                         const double plane_height,
                         tf2::Vector3* position,
                         double* range) const {
    if (std::abs(direction.z()) < 1.0e-8) return false;
    const double distance = (plane_height - origin.z()) / direction.z();
    if (!std::isfinite(distance) || distance < minimum_intersection_range_m_ ||
        distance > maximum_intersection_range_m_) return false;
    *position = origin + distance * direction;
    if (range != nullptr) *range = distance;
    return true;
  }

  bool TerrainElevation(const TerrainGrid& grid, const double x, const double y,
                        double* height, double* variance) const {
    const double gx = (x - grid.origin_x) / grid.resolution;
    const double gy = (y - grid.origin_y) / grid.resolution;
    if (gx < 0.0 || gy < 0.0 || gx > grid.width - 1U || gy > grid.height - 1U)
      return false;
    const std::uint32_t x0 = static_cast<std::uint32_t>(std::floor(gx));
    const std::uint32_t y0 = static_cast<std::uint32_t>(std::floor(gy));
    const std::uint32_t x1 = std::min(x0 + 1U, grid.width - 1U);
    const std::uint32_t y1 = std::min(y0 + 1U, grid.height - 1U);
    const double fx = gx - x0, fy = gy - y0;
    const auto interpolate = [&](const std::vector<float>& values,
                                 const double fallback) {
      if (values.empty()) return fallback;
      const auto value = [&](std::uint32_t column, std::uint32_t row) {
        return static_cast<double>(values[static_cast<std::size_t>(row) *
                                          grid.width + column]);
      };
      const double a = value(x0, y0), b = value(x1, y0);
      const double c = value(x0, y1), d = value(x1, y1);
      if (!std::isfinite(a) || !std::isfinite(b) ||
          !std::isfinite(c) || !std::isfinite(d))
        return std::numeric_limits<double>::quiet_NaN();
      return (1.0 - fy) * ((1.0 - fx) * a + fx * b) +
             fy * ((1.0 - fx) * c + fx * d);
    };
    *height = interpolate(grid.elevation,
                          std::numeric_limits<double>::quiet_NaN());
    if (!std::isfinite(*height)) return false;
    const double fallback = ground_height_stddev_m_ * ground_height_stddev_m_;
    const double sampled_variance = interpolate(grid.variance, fallback);
    *variance = std::isfinite(sampled_variance) && sampled_variance >= 0.0
                    ? sampled_variance : fallback;
    return true;
  }

  bool TerrainIntersection(const tf2::Vector3& origin,
                           const tf2::Vector3& direction,
                           const TerrainGrid& grid,
                           tf2::Vector3* position,
                           double* range,
                           double* variance) const {
    bool have_previous = false;
    double previous_range = minimum_intersection_range_m_;
    double previous_difference = 0.0;
    for (double current_range = minimum_intersection_range_m_;
         current_range <= maximum_intersection_range_m_;
         current_range += terrain_ray_step_m_) {
      const tf2::Vector3 current = origin + current_range * direction;
      double height = 0.0, sampled_variance = 0.0;
      if (!TerrainElevation(grid, current.x(), current.y(), &height,
                            &sampled_variance)) {
        have_previous = false;
        continue;
      }
      const double difference = current.z() - height;
      if (have_previous && ((previous_difference >= 0.0 && difference <= 0.0) ||
                            (previous_difference <= 0.0 && difference >= 0.0))) {
        const double denominator = previous_difference - difference;
        const double ratio = std::abs(denominator) < 1.0e-9
                                 ? 0.0 : previous_difference / denominator;
        const double hit_range = previous_range + Clamp(ratio, 0.0, 1.0) *
            (current_range - previous_range);
        tf2::Vector3 hit = origin + hit_range * direction;
        if (!TerrainElevation(grid, hit.x(), hit.y(), &height,
                              &sampled_variance)) return false;
        hit.setZ(height);
        *position = hit;
        *range = hit_range;
        *variance = sampled_variance;
        return true;
      }
      have_previous = true;
      previous_range = current_range;
      previous_difference = difference;
    }
    return false;
  }

  void LocalizeTrack(RegionTrack* track, const std_msgs::Header& header,
                     const std::uint32_t mask_width,
                     const std::uint32_t mask_height) {
    track->localization_valid = false;
    track->world_polygon = geometry_msgs::PolygonStamped();
    track->world_area_m2 = 0.0;
    track->world_projection_coverage = 0.0F;
    track->covariance.fill(0.0);
    track->covariance[0] = track->covariance[4] = track->covariance[8] = 1.0e6;
    if (localization_mode_ == "bearing_only" || track->contour.size() < 3U) return;
    const auto camera_info = CameraInfoForStamp(track->last_seen);
    if (!camera_info) {
      ++localization_failures_;
      return;
    }
    image_geometry::PinholeCameraModel camera_model;
    camera_model.fromCameraInfo(*camera_info);
    const std::string camera_frame = !camera_info->header.frame_id.empty()
        ? camera_info->header.frame_id : header.frame_id;
    geometry_msgs::TransformStamped transform;
    try {
      transform = tf_buffer_.lookupTransform(task_frame_, camera_frame,
          track->last_seen, ros::Duration(tf_lookup_timeout_sec_));
    } catch (const tf2::TransformException&) {
      ++localization_failures_;
      return;
    }
    if (localization_mode_ == "terrain" && !terrain_grid_) {
      ++localization_failures_;
      return;
    }
    if (terrain_grid_ && !terrain_grid_->header.stamp.isZero() &&
        terrain_max_age_sec_ > 0.0 &&
        std::abs((track->last_seen - terrain_grid_->header.stamp).toSec()) >
            terrain_max_age_sec_) {
      ++localization_failures_;
      return;
    }
    tf2::Transform transform_tf;
    tf2::fromMsg(transform.transform, transform_tf);
    const tf2::Vector3 origin = transform_tf * tf2::Vector3(0.0, 0.0, 0.0);
    track->world_polygon.header.stamp = track->last_seen;
    track->world_polygon.header.frame_id = task_frame_;
    double range_sum = 0.0, variance_sum = 0.0;
    std::size_t projected_count = 0U;
    for (const cv::Point& point : track->contour) {
      const cv::Point2d pixel(
          (static_cast<double>(point.x) + 0.5) * camera_info->width / mask_width,
          (static_cast<double>(point.y) + 0.5) * camera_info->height / mask_height);
      const cv::Point3d camera_ray = camera_model.projectPixelTo3dRay(pixel);
      tf2::Vector3 direction = transform_tf.getBasis() *
          tf2::Vector3(camera_ray.x, camera_ray.y, camera_ray.z);
      if (direction.length2() < 1.0e-12) continue;
      direction.normalize();
      tf2::Vector3 hit;
      double range = 0.0;
      double variance = ground_height_stddev_m_ * ground_height_stddev_m_;
      const bool projected = localization_mode_ == "horizontal_plane"
          ? PlaneIntersection(origin, direction, ground_plane_z_, &hit, &range)
          : TerrainIntersection(origin, direction, *terrain_grid_, &hit, &range,
                                &variance);
      if (!projected) continue;
      geometry_msgs::Point32 world_point;
      world_point.x = hit.x();
      world_point.y = hit.y();
      world_point.z = hit.z();
      track->world_polygon.polygon.points.push_back(world_point);
      range_sum += range;
      variance_sum += variance;
      ++projected_count;
    }
    track->world_projection_coverage = static_cast<float>(projected_count) /
                                       static_cast<float>(track->contour.size());
    if (projected_count < 3U ||
        track->world_projection_coverage < minimum_projection_coverage_) {
      track->world_polygon = geometry_msgs::PolygonStamped();
      ++localization_failures_;
      return;
    }
    double twice_area = 0.0, centroid_x_numerator = 0.0;
    double centroid_y_numerator = 0.0, centroid_z = 0.0;
    const auto& points = track->world_polygon.polygon.points;
    for (std::size_t i = 0; i < points.size(); ++i) {
      const auto& left = points[i];
      const auto& right = points[(i + 1U) % points.size()];
      const double cross = static_cast<double>(left.x) * right.y -
                           static_cast<double>(right.x) * left.y;
      twice_area += cross;
      centroid_x_numerator += (left.x + right.x) * cross;
      centroid_y_numerator += (left.y + right.y) * cross;
      centroid_z += left.z;
    }
    track->world_area_m2 = std::abs(twice_area) * 0.5;
    if (track->world_area_m2 <= 0.01 || std::abs(twice_area) < 1.0e-9) {
      ++localization_failures_;
      return;
    }
    track->map_centroid.header = track->world_polygon.header;
    track->map_centroid.pose.position.x = centroid_x_numerator / (3.0 * twice_area);
    track->map_centroid.pose.position.y = centroid_y_numerator / (3.0 * twice_area);
    track->map_centroid.pose.position.z = centroid_z / points.size();
    track->map_centroid.pose.orientation.w = 1.0;
    const double mean_range = range_sum / projected_count;
    const double angular_sigma = projection_attitude_stddev_deg_ *
                                 std::acos(-1.0) / 180.0;
    const double horizontal_variance =
        projection_position_stddev_m_ * projection_position_stddev_m_ +
        mean_range * mean_range * angular_sigma * angular_sigma;
    track->covariance.fill(0.0);
    track->covariance[0] = track->covariance[4] = horizontal_variance;
    track->covariance[8] = projection_position_stddev_m_ *
                               projection_position_stddev_m_ +
                           variance_sum / projected_count;
    track->localization_valid = true;
    ++localized_regions_;
  }

  std::string ClassName(const int class_id) const {
    if (class_id >= 0 && static_cast<std::size_t>(class_id) < class_names_.size()) {
      return class_names_[static_cast<std::size_t>(class_id)];
    }
    return std::string("class_") + std::to_string(class_id);
  }

  bool AcceptTimestamp(const ros::Time& stamp) {
    if (stamp.isZero()) {
      if (require_mask_timestamp_) {
        ROS_WARN_THROTTLE(2.0, "Rejecting a semantic mask without capture timestamp.");
        return false;
      }
      return true;
    }

    const ros::Time now = ros::Time::now();
    if (!now.isZero()) {
      const double age_sec = (now - stamp).toSec();
      if (age_sec > input_max_age_sec_) {
        ROS_WARN_THROTTLE(2.0, "Rejecting stale semantic mask (%.3f s old).", age_sec);
        return false;
      }
      if (age_sec < -future_timestamp_tolerance_sec_) {
        ROS_WARN_THROTTLE(2.0, "Rejecting semantic mask from the future (%.3f s).", -age_sec);
        return false;
      }
    }
    if (reject_out_of_order_ && !last_accepted_stamp_.isZero() && stamp <= last_accepted_stamp_) {
      ROS_WARN_THROTTLE(2.0, "Rejecting out-of-order semantic mask.");
      return false;
    }
    return true;
  }

  std::vector<Observation> ExtractObservations(const cv::Mat& class_mask) const {
    std::vector<Observation> observations;
    cv::Mat kernel;
    if (morphology_kernel_px_ > 0) {
      kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE,
                                         cv::Size(morphology_kernel_px_, morphology_kernel_px_));
    }

    for (const int class_id : hazard_class_ids_) {
      if (class_id < 0 || class_id > 255) {
        ROS_WARN_THROTTLE(5.0, "Ignoring out-of-range semantic class id %d.", class_id);
        continue;
      }
      cv::Mat binary_mask;
      cv::inRange(class_mask, cv::Scalar(class_id), cv::Scalar(class_id), binary_mask);
      if (!kernel.empty()) {
        cv::morphologyEx(binary_mask, binary_mask, cv::MORPH_OPEN, kernel);
        cv::morphologyEx(binary_mask, binary_mask, cv::MORPH_CLOSE, kernel);
      }

      std::vector<std::vector<cv::Point>> contours;
      cv::findContours(binary_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
      for (const std::vector<cv::Point>& contour : contours) {
        const double area_px = cv::contourArea(contour);
        if (area_px < minimum_region_area_px_) {
          continue;
        }
        Observation observation;
        observation.class_id = class_id;
        observation.bounding_box = cv::boundingRect(contour);
        const double perimeter = cv::arcLength(contour, true);
        cv::approxPolyDP(contour, observation.contour,
                         contour_simplification_ratio_ * perimeter, true);
        if (observation.contour.size() >
            static_cast<std::size_t>(maximum_contour_vertices_)) {
          std::vector<cv::Point> bounded;
          bounded.reserve(static_cast<std::size_t>(maximum_contour_vertices_));
          for (int index = 0; index < maximum_contour_vertices_; ++index) {
            const std::size_t source_index = static_cast<std::size_t>(index) *
                observation.contour.size() /
                static_cast<std::size_t>(maximum_contour_vertices_);
            bounded.push_back(observation.contour[source_index]);
          }
          observation.contour = std::move(bounded);
        }
        observation.area_px = area_px;
        observations.push_back(std::move(observation));
      }
    }
    return observations;
  }

  std::size_t Associate(const Observation& observation, const ros::Time& stamp,
                        const std::vector<bool>& claimed_tracks) const {
    std::size_t best_index = tracks_.size();
    double best_iou = association_iou_threshold_;
    for (std::size_t index = 0; index < tracks_.size(); ++index) {
      const RegionTrack& track = tracks_[index];
      if (claimed_tracks[index] || track.class_id != observation.class_id ||
          (stamp - track.last_seen).toSec() > track_timeout_sec_) {
        continue;
      }
      const double iou = IntersectionOverUnion(track.bounding_box, observation.bounding_box);
      if (iou >= best_iou) {
        best_iou = iou;
        best_index = index;
      }
    }
    return best_index;
  }

  uint8_t SemanticType(const int class_id) const {
    if (class_id >= 0 && static_cast<std::size_t>(class_id) <
                             global_semantic_ids_.size()) {
      return static_cast<uint8_t>(
          std::max(0, std::min(255, global_semantic_ids_[class_id])));
    }
    return FloodRegion::SEMANTIC_UNKNOWN;
  }

  FloodRegion ToMessage(const RegionTrack& track,
                        const std_msgs::Header& header,
                        const uint32_t image_width,
                        const uint32_t image_height,
                        const uint8_t status) {
    FloodRegion message;
    message.header = header;
    message.track_id = track.id;
    message.observation_uuid = observationUuid(identity_, track.id);
    message.event_sequence = next_event_sequence_++;
    message.provenance = identity_;
    message.class_id = static_cast<uint8_t>(track.class_id);
    message.class_name = ClassName(track.class_id);
    message.semantic_type = SemanticType(track.class_id);
    message.status = status;
    message.confidence = track.confidence;
    message.stability = static_cast<float>(
        std::min(1.0, static_cast<double>(track.consecutive_hits) /
                          static_cast<double>(min_confirmations_)));
    message.observation_count = track.observations;
    message.consecutive_hits = track.consecutive_hits;
    message.first_seen = track.first_seen;
    message.last_seen = track.last_seen;
    message.pixel_area = static_cast<uint32_t>(std::max(0.0, track.area_px));
    const double image_area = static_cast<double>(image_width) *
                              static_cast<double>(image_height);
    message.image_area_ratio = static_cast<float>(image_area > 0.0
        ? track.area_px / image_area : 0.0);
    message.priority = static_cast<float>(Clamp(
        message.confidence * message.stability *
            std::sqrt(std::max(0.0F, message.image_area_ratio)),
        0.0, 1.0));

    message.image_contour.header = header;
    message.image_contour.header.stamp = track.last_seen;
    message.image_contour.polygon.points.reserve(track.contour.size());
    for (const cv::Point& point : track.contour) {
      geometry_msgs::Point32 polygon_point;
      polygon_point.x = static_cast<float>(point.x);
      polygon_point.y = static_cast<float>(point.y);
      polygon_point.z = 0.0F;
      message.image_contour.polygon.points.push_back(polygon_point);
    }
    message.image_roi.x_offset = static_cast<uint32_t>(std::max(0, track.bounding_box.x));
    message.image_roi.y_offset = static_cast<uint32_t>(std::max(0, track.bounding_box.y));
    message.image_roi.width = static_cast<uint32_t>(std::max(0, track.bounding_box.width));
    message.image_roi.height = static_cast<uint32_t>(std::max(0, track.bounding_box.height));
    message.image_roi.do_rectify = false;

    message.localization_valid = track.localization_valid;
    message.map_centroid = track.map_centroid;
    message.world_polygon = track.world_polygon;
    message.world_area_m2 = track.world_area_m2;
    message.world_projection_coverage = track.world_projection_coverage;
    for (std::size_t index = 0; index < message.position_covariance.size(); ++index) {
      message.position_covariance[index] = track.covariance[index];
    }
    message.evidence_uuid = message.observation_uuid + "@" +
                            std::to_string(track.last_seen.toNSec());
    message.evidence_source_topic = evidence_source_topic_;
    CacheEvidenceCrop(message.evidence_uuid, track.last_seen, message.image_roi,
                      &message.evidence_image_sha256);
    if (evidence_crops_.count(message.evidence_uuid) != 0U) {
      message.evidence_retrieval_uri = "ros-service://" +
          node_handle_.resolveName(regions_topic_ + "/get_evidence") +
          "?evidence_uuid=" + message.evidence_uuid;
    }
    message.sensor_id = sensor_id_;
    return message;
  }

  void HandleMask(const sensor_msgs::ImageConstPtr& input_message) {
    ++received_masks_;
    last_mask_wall_ = ros::WallTime::now();
    if (!AcceptTimestamp(input_message->header.stamp)) {
      ++rejected_masks_;
      return;
    }
    if (input_message->encoding != sensor_msgs::image_encodings::MONO8) {
      ROS_ERROR_THROTTLE(2.0,
                         "Semantic mask must use MONO8 class indices; received encoding '%s'.",
                         input_message->encoding.c_str());
      ++rejected_masks_;
      return;
    }

    cv_bridge::CvImageConstPtr cv_image;
    try {
      cv_image = cv_bridge::toCvShare(input_message, sensor_msgs::image_encodings::MONO8);
    } catch (const cv_bridge::Exception& exception) {
      ROS_ERROR_THROTTLE(2.0, "Unable to access semantic mask: %s", exception.what());
      ++rejected_masks_;
      return;
    }
    if (cv_image->image.empty()) {
      ROS_WARN_THROTTLE(2.0, "Ignoring an empty semantic mask.");
      ++rejected_masks_;
      return;
    }

    // Only a structurally valid semantic mask advances the ordering watermark.
    // A malformed colour/palette message must not make the next valid capture
    // appear out of order.
    if (!input_message->header.stamp.isZero()) {
      last_accepted_stamp_ = input_message->header.stamp;
    }

    const ros::Time stamp = input_message->header.stamp.isZero() ? ros::Time::now()
                                                                   : input_message->header.stamp;
    FloodRegionArray output;
    output.header = input_message->header;
    output.array_sequence = next_array_sequence_++;
    output.full_snapshot = false;
    output.provenance = identity_;
    output.regions.reserve(16);
    last_header_ = input_message->header;
    last_image_width_ = input_message->width;
    last_image_height_ = input_message->height;

    std::vector<bool> claimed_tracks(tracks_.size(), false);
    for (const Observation& observation : ExtractObservations(cv_image->image)) {
      const std::size_t matching_index =
          Associate(observation, stamp, claimed_tracks);
      const bool is_new_track = matching_index == tracks_.size();
      bool was_confirmed = false;
      if (is_new_track) {
        RegionTrack track;
        track.id = next_track_id_++;
        track.class_id = observation.class_id;
        track.bounding_box = observation.bounding_box;
        track.contour = observation.contour;
        track.area_px = observation.area_px;
        track.confidence = static_cast<float>(default_mask_confidence_);
        track.observations = 1U;
        track.consecutive_hits = 1U;
        track.confirmed = min_confirmations_ <= 1U;
        track.first_seen = stamp;
        track.last_seen = stamp;
        track.last_update_wall = ros::WallTime::now();
        track.hit_stamps.push_back(stamp);
        tracks_.push_back(std::move(track));
        claimed_tracks.push_back(true);
        ++created_tracks_;
      } else {
        claimed_tracks[matching_index] = true;
        RegionTrack& track = tracks_[matching_index];
        was_confirmed = track.confirmed;
        track.hit_stamps.push_back(stamp);
        while (!track.hit_stamps.empty() &&
               (stamp - track.hit_stamps.front()).toSec() >
                   confirmation_window_sec_) {
          track.hit_stamps.pop_front();
        }
        track.consecutive_hits =
            static_cast<uint32_t>(track.hit_stamps.size());
        ++track.observations;
        track.bounding_box = observation.bounding_box;
        track.contour = observation.contour;
        track.area_px = observation.area_px;
        track.confidence = static_cast<float>(
            confidence_ema_alpha_ * default_mask_confidence_ +
            (1.0 - confidence_ema_alpha_) * track.confidence);
        track.last_seen = stamp;
        track.last_update_wall = ros::WallTime::now();
        track.confirmed = track.confirmed || track.consecutive_hits >= min_confirmations_;
      }

      RegionTrack& track = tracks_[is_new_track ? tracks_.size() - 1U : matching_index];
      LocalizeTrack(&track, input_message->header, input_message->width,
                    input_message->height);
      const uint8_t status = track.confirmed
          ? (was_confirmed ? FloodRegion::UPDATED : FloodRegion::CONFIRMED)
          : FloodRegion::PENDING;
      const bool geometry_changed = track.last_published_box.area() <= 0 ||
          IntersectionOverUnion(track.last_published_box, track.bounding_box) <
              region_update_iou_threshold_;
      const bool refresh_due = track.last_publish_wall.isZero() ||
          (ros::WallTime::now() - track.last_publish_wall).toSec() >=
              minimum_region_update_period_sec_;
      const bool lifecycle_event = is_new_track || (!was_confirmed && track.confirmed);
      if ((track.confirmed || publish_pending_regions_) &&
          (lifecycle_event || geometry_changed || refresh_due)) {
        output.regions.push_back(ToMessage(track, input_message->header,
                                           input_message->width, input_message->height, status));
        track.last_published_box = track.bounding_box;
        track.last_publish_wall = ros::WallTime::now();
      }
      if (!was_confirmed && track.confirmed)
        ++confirmed_tracks_;
    }

    publisher_.publish(output);
    ++published_arrays_;
    published_regions_ += output.regions.size();
  }

  void HandleExpirationTimer(const ros::WallTimerEvent&) {
    if (tracks_.empty())
      return;
    FloodRegionArray expired;
    expired.header = last_header_;
    expired.header.stamp = ros::Time::now();
    expired.array_sequence = next_array_sequence_++;
    expired.full_snapshot = false;
    expired.provenance = identity_;
    const ros::WallTime now = ros::WallTime::now();
    for (auto iterator = tracks_.begin(); iterator != tracks_.end();) {
      if ((now - iterator->last_update_wall).toSec() <= track_timeout_sec_) {
        ++iterator;
        continue;
      }
      if (iterator->confirmed) {
        expired.regions.push_back(ToMessage(*iterator, expired.header,
                                            last_image_width_, last_image_height_,
                                            FloodRegion::EXPIRED));
        ++expired_tracks_;
      }
      iterator = tracks_.erase(iterator);
    }
    if (!expired.regions.empty()) {
      publisher_.publish(expired);
      ++published_arrays_;
      published_regions_ += expired.regions.size();
    }
  }

  void HandleSnapshotTimer(const ros::WallTimerEvent&) {
    FloodRegionArray snapshot = CurrentSnapshot();
    publisher_.publish(snapshot);
    ++published_arrays_;
    published_regions_ += snapshot.regions.size();
    ++published_snapshots_;
  }

  FloodRegionArray CurrentSnapshot() {
    FloodRegionArray snapshot;
    snapshot.header = last_header_;
    snapshot.header.stamp = ros::Time::now();
    snapshot.array_sequence = next_array_sequence_++;
    snapshot.full_snapshot = true;
    snapshot.provenance = identity_;
    snapshot.regions.reserve(tracks_.size());
    for (const RegionTrack& track : tracks_) {
      if (track.confirmed || publish_pending_regions_) {
        snapshot.regions.push_back(ToMessage(
            track, snapshot.header, last_image_width_, last_image_height_,
            track.confirmed ? FloodRegion::UPDATED : FloodRegion::PENDING));
      }
    }
    return snapshot;
  }

  bool HandleSnapshotService(GetFloodRegions::Request&,
                             GetFloodRegions::Response& response) {
    response.snapshot = CurrentSnapshot();
    return true;
  }

  void HandleDiagnosticTimer(const ros::WallTimerEvent&) {
    diagnostics_.update();
  }

  void Diagnostics(diagnostic_updater::DiagnosticStatusWrapper& status) {
    int level = diagnostic_msgs::DiagnosticStatus::OK;
    std::string summary = "Flood region extractor running";
    if (received_masks_ == 0U) {
      level = diagnostic_msgs::DiagnosticStatus::WARN;
      summary = "Waiting for semantic masks";
    } else if (!last_mask_wall_.isZero() &&
               (ros::WallTime::now() - last_mask_wall_).toSec() >
                   std::max(1.0, track_timeout_sec_)) {
      level = diagnostic_msgs::DiagnosticStatus::WARN;
      summary = "Semantic mask stream is stale";
    }
    status.summary(level, summary);
    status.add("mission_id", identity_.mission_id);
    status.add("uav_id", identity_.uav_id);
    status.add("session_uuid", identity_.session_uuid);
    status.add("profile", identity_.profile);
    status.add("model_version", identity_.model_version);
    status.add("model_sha256", identity_.model_sha256);
    status.add("active_tracks", static_cast<long long>(tracks_.size()));
    status.add("received_masks", static_cast<long long>(received_masks_));
    status.add("rejected_masks", static_cast<long long>(rejected_masks_));
    status.add("created_tracks", static_cast<long long>(created_tracks_));
    status.add("confirmed_tracks", static_cast<long long>(confirmed_tracks_));
    status.add("expired_tracks", static_cast<long long>(expired_tracks_));
    status.add("published_arrays", static_cast<long long>(published_arrays_));
    status.add("published_regions", static_cast<long long>(published_regions_));
    status.add("published_snapshots", static_cast<long long>(published_snapshots_));
    status.add("localization_mode", localization_mode_);
    status.add("localized_regions", static_cast<long long>(localized_regions_));
    status.add("localization_failures", static_cast<long long>(localization_failures_));
    status.add("camera_info_samples",
               static_cast<long long>(camera_info_history_.size()));
    status.add("terrain_available", static_cast<bool>(terrain_grid_));
    status.add("maximum_contour_vertices", maximum_contour_vertices_);
    status.add("evidence_frames", static_cast<long long>(evidence_frames_.size()));
    status.add("evidence_crops", static_cast<long long>(evidence_crops_.size()));
    status.add("evidence_failures", static_cast<long long>(evidence_failures_));
  }

  ros::NodeHandle node_handle_;
  ros::NodeHandle private_node_handle_;
  ros::Subscriber subscriber_;
  ros::Subscriber camera_info_subscriber_;
  ros::Subscriber terrain_subscriber_;
  ros::Subscriber evidence_subscriber_;
  ros::Publisher publisher_;
  ros::ServiceServer snapshot_service_;
  ros::ServiceServer evidence_service_;
  ros::WallTimer expiration_timer_;
  ros::WallTimer snapshot_timer_;
  ros::WallTimer diagnostic_timer_;
  diagnostic_updater::Updater diagnostics_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::string input_mask_topic_;
  std::string regions_topic_;
  std::string sensor_id_;
  std::string task_frame_;
  std::string localization_mode_;
  std::string camera_info_topic_;
  std::string terrain_grid_topic_;
  std::string evidence_source_topic_;
  std::vector<int> hazard_class_ids_;
  std::vector<int> global_semantic_ids_;
  std::vector<std::string> class_names_;
  std::vector<RegionTrack> tracks_;
  ros::Time last_accepted_stamp_;
  std_msgs::Header last_header_;
  ros::WallTime last_mask_wall_;
  PerceptionIdentity identity_;
  std::deque<sensor_msgs::CameraInfoConstPtr> camera_info_history_;
  std::deque<EvidenceFrame> evidence_frames_;
  std::unordered_map<std::string, EvidenceCrop> evidence_crops_;
  std::deque<std::string> evidence_crop_order_;
  TerrainGrid::ConstPtr terrain_grid_;
  uint32_t next_track_id_;
  uint64_t next_event_sequence_{1U};
  uint64_t next_array_sequence_{1U};
  uint32_t last_image_width_{0U};
  uint32_t last_image_height_{0U};
  int morphology_kernel_px_;
  double minimum_region_area_px_;
  double association_iou_threshold_;
  uint32_t min_confirmations_;
  double confirmation_window_sec_;
  double track_timeout_sec_;
  double confidence_ema_alpha_;
  double default_mask_confidence_;
  bool publish_pending_regions_;
  bool require_mask_timestamp_;
  bool reject_out_of_order_;
  double input_max_age_sec_;
  double future_timestamp_tolerance_sec_;
  double expiration_check_period_sec_{0.20};
  double snapshot_period_sec_{1.0};
  double ground_plane_z_{0.0};
  double camera_info_max_delta_sec_{0.05};
  bool camera_info_allow_static_{true};
  double tf_lookup_timeout_sec_{0.05};
  double terrain_max_age_sec_{30.0};
  double minimum_intersection_range_m_{1.0};
  double maximum_intersection_range_m_{5000.0};
  double terrain_ray_step_m_{2.0};
  double minimum_projection_coverage_{0.75};
  double projection_position_stddev_m_{1.5};
  double projection_attitude_stddev_deg_{1.0};
  double ground_height_stddev_m_{1.0};
  double contour_simplification_ratio_{0.0025};
  int maximum_contour_vertices_{96};
  double minimum_region_update_period_sec_{0.5};
  double region_update_iou_threshold_{0.90};
  double evidence_image_max_delta_sec_{0.10};
  int evidence_frame_cache_size_{12};
  int evidence_crop_cache_size_{256};
  int evidence_jpeg_quality_{85};
  uint64_t received_masks_{0U};
  uint64_t rejected_masks_{0U};
  uint64_t created_tracks_{0U};
  uint64_t confirmed_tracks_{0U};
  uint64_t expired_tracks_{0U};
  uint64_t published_arrays_{0U};
  uint64_t published_regions_{0U};
  uint64_t published_snapshots_{0U};
  uint64_t invalid_camera_info_{0U};
  uint64_t invalid_terrain_grids_{0U};
  uint64_t localized_regions_{0U};
  uint64_t localization_failures_{0U};
  uint64_t evidence_failures_{0U};
};

}  // namespace sar_yolo_detector

int main(int argc, char** argv) {
  ros::init(argc, argv, "sar_flood_region_extractor");
  try {
    sar_yolo_detector::FloodRegionExtractor extractor;
    ros::spin();
  } catch (const std::exception& exception) {
    ROS_FATAL_STREAM("sar_flood_region_extractor configuration error: " << exception.what());
    return 1;
  }
  return 0;
}
