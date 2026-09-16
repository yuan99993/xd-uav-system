#include <algorithm>
#include <cctype>
#include <cmath>
#include <deque>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <XmlRpcValue.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/TransformStamped.h>
#include <opencv2/imgproc.hpp>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_msgs/String.h>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <xd_uav_detect/WorldDetectionArray.h>
#include <xd_uav_detect/localization/localization_backend.hpp>
#include <xd_uav_track/DetectionArray.h>

namespace {

bool xmlNumber(const XmlRpc::XmlRpcValue& value, double* result) {
  if (result == nullptr) return false;
  if (value.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
    *result = static_cast<double>(value);
    return std::isfinite(*result);
  }
  if (value.getType() == XmlRpc::XmlRpcValue::TypeInt) {
    *result = static_cast<int>(value);
    return true;
  }
  return false;
}

bool loadVector3(const ros::NodeHandle& nh, const std::string& name,
                 Eigen::Vector3d* vector) {
  if (vector == nullptr) return false;
  XmlRpc::XmlRpcValue values;
  if (!nh.getParam(name, values) ||
      values.getType() != XmlRpc::XmlRpcValue::TypeArray ||
      values.size() != 3) {
    return false;
  }
  for (int index = 0; index < 3; ++index) {
    double value = 0.0;
    if (!xmlNumber(values[index], &value)) return false;
    (*vector)[index] = value;
  }
  return vector->array().isFinite().all();
}

Eigen::Matrix4d transformMatrix(
    const geometry_msgs::TransformStamped& transform) {
  Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
  const auto& rotation = transform.transform.rotation;
  Eigen::Quaterniond quaternion(
      rotation.w, rotation.x, rotation.y, rotation.z);
  if (quaternion.norm() <= 1e-9) return matrix;
  quaternion.normalize();
  matrix.block<3, 3>(0, 0) = quaternion.toRotationMatrix();
  matrix(0, 3) = transform.transform.translation.x;
  matrix(1, 3) = transform.transform.translation.y;
  matrix(2, 3) = transform.transform.translation.z;
  return matrix;
}

cv::Scalar depthColor(const double depth, const double minimum,
                      const double maximum) {
  static const std::vector<cv::Vec3b> palette = []() {
    cv::Mat hsv(1, 256, CV_8UC3);
    for (int index = 0; index < 256; ++index) {
      // Near points are red; far points transition through yellow/green to blue.
      hsv.at<cv::Vec3b>(0, index) = cv::Vec3b(
          static_cast<unsigned char>(120.0 * index / 255.0), 255, 255);
    }
    cv::Mat bgr;
    cv::cvtColor(hsv, bgr, cv::COLOR_HSV2BGR);
    std::vector<cv::Vec3b> result(256);
    for (int index = 0; index < 256; ++index) {
      result[index] = bgr.at<cv::Vec3b>(0, index);
    }
    return result;
  }();
  const double normalized = std::max(
      0.0, std::min(1.0, (depth - minimum) / (maximum - minimum)));
  const cv::Vec3b color = palette[static_cast<std::size_t>(
      std::lround(255.0 * normalized))];
  return cv::Scalar(color[0], color[1], color[2]);
}

}  // namespace

class XdUavDetectNode {
 public:
  XdUavDetectNode()
      : private_nh_("~"), tf_listener_(tf_buffer_) {
    std::string localization_method{"lidar_camera"};
    private_nh_.param("localization/method", localization_method,
                      localization_method);
    private_nh_.param("localization/enabled", localization_enabled_, true);

    xd_uav_detect::LocalizationBackendConfig backend_config;
    auto& fusion_config = backend_config.lidar_camera;
    private_nh_.param("fusion/minimum_cluster_points",
                      fusion_config.minimum_cluster_points,
                      fusion_config.minimum_cluster_points);
    private_nh_.param("fusion/minimum_range_m",
                      fusion_config.minimum_range_m,
                      fusion_config.minimum_range_m);
    private_nh_.param("fusion/maximum_range_m",
                      fusion_config.maximum_range_m,
                      fusion_config.maximum_range_m);
    private_nh_.param("fusion/box_shrink_ratio",
                      fusion_config.box_shrink_ratio,
                      fusion_config.box_shrink_ratio);
    private_nh_.param("fusion/depth_cluster_tolerance_m",
                      fusion_config.depth_cluster_tolerance_m,
                      fusion_config.depth_cluster_tolerance_m);
    auto& ground_config = backend_config.camera_ground_plane;
    private_nh_.param("ground_projection/ground_plane_z_m",
                      ground_config.ground_plane_z_m,
                      ground_config.ground_plane_z_m);
    private_nh_.param("ground_projection/minimum_range_m",
                      ground_config.minimum_range_m,
                      ground_config.minimum_range_m);
    private_nh_.param("ground_projection/maximum_range_m",
                      ground_config.maximum_range_m,
                      ground_config.maximum_range_m);
    private_nh_.param("ground_projection/minimum_ray_plane_angle_deg",
                      ground_config.minimum_ray_plane_angle_deg,
                      ground_config.minimum_ray_plane_angle_deg);
    private_nh_.param("ground_projection/box_anchor_y_ratio",
                      ground_config.box_anchor_y_ratio,
                      ground_config.box_anchor_y_ratio);
    private_nh_.param("ground_projection/pixel_stddev_px",
                      ground_config.pixel_stddev_px,
                      ground_config.pixel_stddev_px);
    private_nh_.param("ground_projection/ground_height_stddev_m",
                      ground_config.ground_height_stddev_m,
                      ground_config.ground_height_stddev_m);
    private_nh_.param("ground_projection/position_stddev_m",
                      ground_config.position_stddev_m,
                      ground_config.position_stddev_m);
    auto& gimbal_config = backend_config.gimbal_laser_range;
    private_nh_.param("gimbal_range/bbox_gate_margin_px",
                      gimbal_config.bbox_gate_margin_px,
                      gimbal_config.bbox_gate_margin_px);
    private_nh_.param("gimbal_range/range_stddev_m",
                      gimbal_config.range_stddev_m,
                      gimbal_config.range_stddev_m);
    private_nh_.param("gimbal_range/angular_stddev_rad",
                      gimbal_config.angular_stddev_rad,
                      gimbal_config.angular_stddev_rad);
    private_nh_.param("gimbal_range/position_stddev_m",
                      gimbal_config.position_stddev_m,
                      gimbal_config.position_stddev_m);
    private_nh_.param("gimbal_range/require_laser_axis_in_bbox",
                      gimbal_config.require_laser_axis_in_bbox,
                      gimbal_config.require_laser_axis_in_bbox);
    bool deprecated_alias = false;
    localization_backend_ = xd_uav_detect::makeLocalizationBackend(
        localization_method, backend_config, &deprecated_alias);
    localization_method_ = localization_backend_->kind();
    localization_method_name_ = localization_backend_->name();
    if (deprecated_alias) {
      ROS_WARN("[xd_uav_detect] localization method '%s' is deprecated; "
               "use '%s'", localization_method.c_str(),
               localization_method_name_.c_str());
    }

    private_nh_.param("fusion/enabled", fusion_enabled_, true);
    private_nh_.param("calibration/calibrated", calibrated_, false);
    private_nh_.param("fusion/maximum_cloud_time_difference_sec",
                      maximum_cloud_time_difference_sec_, 0.12);
    private_nh_.param("fusion/cloud_buffer_size", cloud_buffer_size_, 10);
    private_nh_.param("gimbal_range/maximum_time_difference_sec",
                      maximum_range_time_difference_sec_, 0.05);
    private_nh_.param("gimbal_range/range_buffer_size",
                      range_buffer_size_, 20);
    private_nh_.param("gimbal_range/require_detection_timestamp",
                      gimbal_require_detection_timestamp_, true);
    private_nh_.param("gimbal_range/require_matching_camera_frame",
                      gimbal_require_matching_camera_frame_, true);
    private_nh_.param("fusion/point_stride", point_stride_, 1);
    private_nh_.param("fusion/tf_timeout_sec", tf_timeout_sec_, 0.03);
    private_nh_.param("frames/body", body_frame_, std::string("base_link"));
    private_nh_.param("frames/world", world_frame_, std::string("map"));
    private_nh_.param("frames/camera", configured_camera_frame_,
                      std::string());
    private_nh_.param("frames/laser", configured_laser_frame_, std::string());
    private_nh_.param("ground_projection/tf_use_latest_on_failure",
                      ground_tf_use_latest_on_failure_, false);
    private_nh_.param("ground_projection/require_detection_timestamp",
                      ground_require_detection_timestamp_, true);
    private_nh_.param("ground_projection/require_matching_camera_frame",
                      ground_require_matching_camera_frame_, true);
    if (localization_method_ ==
        xd_uav_detect::LocalizationBackendKind::kCameraGroundPlane) {
      private_nh_.param("ground_projection/tf_timeout_sec", tf_timeout_sec_,
                        tf_timeout_sec_);
    } else if (localization_method_ ==
               xd_uav_detect::LocalizationBackendKind::kGimbalLaserRange) {
      private_nh_.param("gimbal_range/tf_timeout_sec", tf_timeout_sec_,
                        tf_timeout_sec_);
    }
    private_nh_.param("visualization/enabled", visualization_enabled_, true);
    private_nh_.param("visualization/point_stride",
                      visualization_point_stride_, 1);
    private_nh_.param("visualization/point_radius_px",
                      visualization_point_radius_px_, 2);
    private_nh_.param("visualization/minimum_depth_m",
                      visualization_minimum_depth_m_, 0.3);
    private_nh_.param("visualization/maximum_depth_m",
                      visualization_maximum_depth_m_, 30.0);
    cloud_buffer_size_ = std::max(1, cloud_buffer_size_);
    range_buffer_size_ = std::max(1, range_buffer_size_);
    point_stride_ = std::max(1, point_stride_);
    visualization_point_stride_ = std::max(1, visualization_point_stride_);
    visualization_point_radius_px_ = std::max(1, visualization_point_radius_px_);
    visualization_minimum_depth_m_ =
        std::max(0.0, visualization_minimum_depth_m_);
    visualization_maximum_depth_m_ = std::max(
        visualization_minimum_depth_m_ + 0.1,
        visualization_maximum_depth_m_);
    maximum_cloud_time_difference_sec_ = std::max(
        0.0, maximum_cloud_time_difference_sec_);
    maximum_range_time_difference_sec_ = std::max(
        0.0, maximum_range_time_difference_sec_);
    tf_timeout_sec_ = std::max(0.0, tf_timeout_sec_);

    Eigen::Vector3d translation_xyz;
    Eigen::Vector3d rotation_ypr;
    calibration_parameters_valid_ =
        loadVector3(private_nh_, "calibration/translation_xyz",
                    &translation_xyz) &&
        loadVector3(private_nh_, "calibration/rotation_ypr", &rotation_ypr);
    if (calibration_parameters_valid_) {
      // velo2cam_calibration writes x y z yaw pitch roll for the transform
      // rotated_camera <- lidar. Convert that convention internally to the
      // ROS optical camera coordinates required by the pinhole projection.
      const Eigen::Matrix3d rotated_camera_from_lidar =
          (Eigen::AngleAxisd(rotation_ypr.x(), Eigen::Vector3d::UnitZ()) *
           Eigen::AngleAxisd(rotation_ypr.y(), Eigen::Vector3d::UnitY()) *
           Eigen::AngleAxisd(rotation_ypr.z(), Eigen::Vector3d::UnitX()))
              .toRotationMatrix();
      Eigen::Matrix3d optical_from_rotated_camera;
      optical_from_rotated_camera <<
          0.0, -1.0,  0.0,
          0.0,  0.0, -1.0,
          1.0,  0.0,  0.0;
      camera_from_lidar_.setIdentity();
      camera_from_lidar_.block<3, 3>(0, 0) =
          optical_from_rotated_camera * rotated_camera_from_lidar;
      camera_from_lidar_.block<3, 1>(0, 3) =
          optical_from_rotated_camera * translation_xyz;
    }
    if (localization_method_ ==
            xd_uav_detect::LocalizationBackendKind::kLidarCamera &&
        calibrated_ && !calibration_parameters_valid_) {
      ROS_ERROR("[xd_uav_detect] calibrated=true but translation_xyz or "
                "rotation_ypr is invalid; 3D fusion is disabled");
      calibrated_ = false;
    }

    // Launch owns UAV namespace and interface wiring. Relative defaults keep
    // rosrun usable inside an explicitly supplied ROS namespace without
    // embedding a vehicle name in this node.
    std::string detections_input{"detect/input/detections_2d"};
    std::string detections_output{"detect/detections"};
    std::string compatibility_detections_output;
    std::string world_detections_output{"detect/detections_world"};
    std::string image_topic{"camera/image_raw"};
    std::string camera_info_topic{"camera/camera_info"};
    std::string cloud_topic{"fastlio/points"};
    std::string range_topic{"gimbal/range"};
    std::string status_topic{"detect/status"};
    std::string debug_image_topic{"detect/debug/image"};
    private_nh_.param("interfaces/input/detections_2d", detections_input,
                      detections_input);
    private_nh_.param("interfaces/input/image", image_topic, image_topic);
    private_nh_.param("interfaces/input/camera_info", camera_info_topic,
                      camera_info_topic);
    private_nh_.param("interfaces/input/point_cloud", cloud_topic, cloud_topic);
    private_nh_.param("interfaces/input/laser_range", range_topic, range_topic);
    private_nh_.param("interfaces/output/detections_3d", detections_output,
                      detections_output);
    private_nh_.param("interfaces/output/detections_compat",
                      compatibility_detections_output,
                      compatibility_detections_output);
    private_nh_.param("interfaces/output/detections_world",
                      world_detections_output, world_detections_output);
    private_nh_.param("interfaces/output/status", status_topic, status_topic);
    private_nh_.param("interfaces/output/debug_image", debug_image_topic,
                      debug_image_topic);

    detections_publisher_ =
        nh_.advertise<xd_uav_track::DetectionArray>(detections_output, 10);
    if (!compatibility_detections_output.empty() &&
        nh_.resolveName(compatibility_detections_output) !=
            nh_.resolveName(detections_output)) {
      compatibility_detections_publisher_ =
          nh_.advertise<xd_uav_track::DetectionArray>(
              compatibility_detections_output, 10);
    }
    world_detections_publisher_ =
        nh_.advertise<xd_uav_detect::WorldDetectionArray>(
            world_detections_output, 10);
    status_publisher_ = nh_.advertise<std_msgs::String>(status_topic, 10);
    debug_image_publisher_ =
        nh_.advertise<sensor_msgs::Image>(debug_image_topic, 1);
    detections_subscriber_ = nh_.subscribe(
        detections_input, 2, &XdUavDetectNode::detectionsCallback, this,
        ros::TransportHints().tcpNoDelay());
    image_subscriber_ = nh_.subscribe(
        image_topic, 1, &XdUavDetectNode::imageCallback, this,
        ros::TransportHints().tcpNoDelay());
    camera_info_subscriber_ = nh_.subscribe(
        camera_info_topic, 1, &XdUavDetectNode::cameraInfoCallback, this,
        ros::TransportHints().tcpNoDelay());
    if (localization_method_ ==
        xd_uav_detect::LocalizationBackendKind::kLidarCamera) {
      cloud_subscriber_ = nh_.subscribe(
          cloud_topic, 2, &XdUavDetectNode::cloudCallback, this,
          ros::TransportHints().tcpNoDelay());
    } else if (localization_method_ ==
               xd_uav_detect::LocalizationBackendKind::kGimbalLaserRange) {
      range_subscriber_ = nh_.subscribe(
          range_topic, 10, &XdUavDetectNode::rangeCallback, this,
          ros::TransportHints().tcpNoDelay());
    }

    ROS_INFO("[xd_uav_detect] external detections=%s -> metric detections=%s; "
             "compatibility output=%s; method=%s; localization=%s; "
             "calibration=%s; debug_image=%s",
             detections_input.c_str(), detections_output.c_str(),
             compatibility_detections_output.empty()
                 ? "disabled" : compatibility_detections_output.c_str(),
             localization_method_name_.c_str(),
             localization_enabled_ ? "enabled" : "disabled",
             localization_method_ !=
                     xd_uav_detect::LocalizationBackendKind::kLidarCamera
                 ? "CameraInfo+TF"
                 : (calibrated_ ? "ready"
                                : "waiting for calibration parameters"),
             debug_image_topic.c_str());
  }

 private:
  void imageCallback(const sensor_msgs::Image::ConstPtr& message) {
    if (message->width == 0 || message->height == 0) return;
    xd_uav_detect::CameraIntrinsics intrinsics;
    sensor_msgs::PointCloud2::ConstPtr cloud;
    sensor_msgs::Range::ConstPtr range;
    xd_uav_track::DetectionArray detections;
    std::string reason;
    double cloud_difference = std::numeric_limits<double>::infinity();
    double range_difference = std::numeric_limits<double>::infinity();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      image_width_ = message->width;
      image_height_ = message->height;
      image_stamp_ = message->header.stamp;
      intrinsics = intrinsics_;
      if (localization_method_ ==
          xd_uav_detect::LocalizationBackendKind::kLidarCamera) {
        cloud = closestCloud(message->header.stamp, &cloud_difference);
      }
      detections = latest_debug_detections_;
      reason = latest_debug_reason_;
    }
    if (!visualization_enabled_ ||
        debug_image_publisher_.getNumSubscribers() == 0) {
      return;
    }
    detections.header = message->header;
    detections.image_width = message->width;
    detections.image_height = message->height;
    if (intrinsics.valid() &&
        (intrinsics.width != message->width ||
         intrinsics.height != message->height)) {
      const double scale_x = static_cast<double>(message->width) /
          intrinsics.width;
      const double scale_y = static_cast<double>(message->height) /
          intrinsics.height;
      intrinsics.fx *= scale_x;
      intrinsics.cx *= scale_x;
      intrinsics.fy *= scale_y;
      intrinsics.cy *= scale_y;
      intrinsics.width = message->width;
      intrinsics.height = message->height;
    }
    std::vector<xd_uav_detect::LidarPoint> points;
    if (cloud != nullptr) cloudPoints(*cloud, &points);
    publishDebugImage(detections, message, intrinsics, points,
                      cloud_difference, reason);
  }

  void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& message) {
    const bool projection_valid = message->P[0] > 0.0 && message->P[5] > 0.0;
    const double fx = projection_valid ? message->P[0] : message->K[0];
    const double fy = projection_valid ? message->P[5] : message->K[4];
    const double cx = projection_valid ? message->P[2] : message->K[2];
    const double cy = projection_valid ? message->P[6] : message->K[5];
    xd_uav_detect::CameraIntrinsics intrinsics;
    intrinsics.fx = fx;
    intrinsics.fy = fy;
    intrinsics.cx = cx;
    intrinsics.cy = cy;
    intrinsics.width = message->width;
    intrinsics.height = message->height;
    if (!intrinsics.valid()) {
      ROS_WARN_THROTTLE(2.0, "[xd_uav_detect] invalid CameraInfo intrinsics");
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    intrinsics_ = intrinsics;
    camera_info_stamp_ = message->header.stamp;
    camera_info_frame_ = message->header.frame_id;
    have_intrinsics_ = true;
  }

  void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& message) {
    std::lock_guard<std::mutex> lock(mutex_);
    cloud_buffer_.push_back(message);
    while (cloud_buffer_.size() >
           static_cast<std::size_t>(cloud_buffer_size_)) {
      cloud_buffer_.pop_front();
    }
  }

  void rangeCallback(const sensor_msgs::Range::ConstPtr& message) {
    std::lock_guard<std::mutex> lock(mutex_);
    range_buffer_.push_back(message);
    while (range_buffer_.size() >
           static_cast<std::size_t>(range_buffer_size_)) {
      range_buffer_.pop_front();
    }
  }

  sensor_msgs::Range::ConstPtr closestRange(
      const ros::Time& requested_stamp, double* difference_sec) const {
    sensor_msgs::Range::ConstPtr best;
    double best_difference = std::numeric_limits<double>::infinity();
    for (const auto& measurement : range_buffer_) {
      if (requested_stamp.isZero() || measurement->header.stamp.isZero()) {
        continue;
      }
      const double difference =
          std::abs((measurement->header.stamp - requested_stamp).toSec());
      if (difference < best_difference) {
        best = measurement;
        best_difference = difference;
      }
    }
    if (difference_sec != nullptr) *difference_sec = best_difference;
    return best;
  }

  sensor_msgs::PointCloud2::ConstPtr closestCloud(
      const ros::Time& requested_stamp, double* difference_sec) const {
    sensor_msgs::PointCloud2::ConstPtr best;
    double best_difference = std::numeric_limits<double>::infinity();
    for (const auto& cloud : cloud_buffer_) {
      const double difference = requested_stamp.isZero() ||
          cloud->header.stamp.isZero()
          ? 0.0 : std::abs((cloud->header.stamp - requested_stamp).toSec());
      if (difference < best_difference) {
        best = cloud;
        best_difference = difference;
      }
    }
    if (difference_sec != nullptr) *difference_sec = best_difference;
    return best;
  }

  bool pixelBox(const xd_uav_track::DetectionCandidate& candidate,
                const unsigned int width, const unsigned int height,
                xd_uav_detect::PixelBox* box) const {
    if (box == nullptr || width == 0 || height == 0) return false;
    if (candidate.has_bbox) {
      box->x_min = candidate.bbox[0];
      box->y_min = candidate.bbox[1];
      box->x_max = candidate.bbox[2];
      box->y_max = candidate.bbox[3];
    } else if (candidate.has_normalized_bbox) {
      const double center_x = candidate.normalized_bbox[0] * width;
      const double center_y = candidate.normalized_bbox[1] * height;
      const double box_width = candidate.normalized_bbox[2] * width;
      const double box_height = candidate.normalized_bbox[3] * height;
      box->x_min = center_x - 0.5 * box_width;
      box->y_min = center_y - 0.5 * box_height;
      box->x_max = center_x + 0.5 * box_width;
      box->y_max = center_y + 0.5 * box_height;
    } else {
      return false;
    }
    box->x_min = std::max(0.0, std::min<double>(width, box->x_min));
    box->x_max = std::max(0.0, std::min<double>(width, box->x_max));
    box->y_min = std::max(0.0, std::min<double>(height, box->y_min));
    box->y_max = std::max(0.0, std::min<double>(height, box->y_max));
    return box->valid();
  }

  bool cloudPoints(const sensor_msgs::PointCloud2& cloud,
                   std::vector<xd_uav_detect::LidarPoint>* points) const {
    if (points == nullptr) return false;
    points->clear();
    try {
      sensor_msgs::PointCloud2ConstIterator<float> x(cloud, "x");
      sensor_msgs::PointCloud2ConstIterator<float> y(cloud, "y");
      sensor_msgs::PointCloud2ConstIterator<float> z(cloud, "z");
      std::size_t index = 0;
      for (; x != x.end(); ++x, ++y, ++z, ++index) {
        if (index % static_cast<std::size_t>(point_stride_) != 0) continue;
        if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z)) {
          continue;
        }
        xd_uav_detect::LidarPoint point;
        point.position_lidar = Eigen::Vector3d(*x, *y, *z);
        points->push_back(point);
      }
    } catch (const std::runtime_error& error) {
      ROS_WARN_THROTTLE(2.0, "[xd_uav_detect] cannot read cloud xyz: %s",
                        error.what());
      return false;
    }
    return !points->empty();
  }

  void clearMetricState(xd_uav_track::DetectionCandidate* candidate) const {
    if (candidate == nullptr) return;
    if (candidate->range_valid && candidate->has_relative_position_body) return;
    candidate->range_valid = false;
    candidate->has_relative_position_body = false;
    candidate->has_relative_velocity_body = false;
  }

  void publishDebugImage(
      const xd_uav_track::DetectionArray& detections,
      const sensor_msgs::Image::ConstPtr& image,
      const xd_uav_detect::CameraIntrinsics& intrinsics,
      const std::vector<xd_uav_detect::LidarPoint>& points,
      const double cloud_difference, const std::string& reason) {
    if (!visualization_enabled_ || debug_image_publisher_.getNumSubscribers() == 0 ||
        image == nullptr) {
      return;
    }

    cv_bridge::CvImagePtr converted;
    try {
      converted = cv_bridge::toCvCopy(image, "bgr8");
    } catch (const cv_bridge::Exception& error) {
      ROS_WARN_THROTTLE(2.0, "[xd_uav_detect] debug image conversion failed: %s",
                        error.what());
      return;
    }
    cv::Mat canvas = converted->image;
    const unsigned int requested_width =
        detections.image_width > 0 ? detections.image_width : canvas.cols;
    const unsigned int requested_height =
        detections.image_height > 0 ? detections.image_height : canvas.rows;
    if (requested_width > 0 && requested_height > 0 &&
        (canvas.cols != static_cast<int>(requested_width) ||
         canvas.rows != static_cast<int>(requested_height))) {
      cv::resize(canvas, canvas,
                 cv::Size(requested_width, requested_height), 0.0, 0.0,
                 cv::INTER_LINEAR);
    }

    std::size_t projected_count = 0;
    double nearest_depth = std::numeric_limits<double>::infinity();
    double farthest_depth = 0.0;
    if (localization_method_ ==
            xd_uav_detect::LocalizationBackendKind::kLidarCamera &&
        calibrated_ && calibration_parameters_valid_ && intrinsics.valid()) {
      for (std::size_t index = 0; index < points.size();
           index += static_cast<std::size_t>(visualization_point_stride_)) {
        const auto& point = points[index].position_lidar;
        const Eigen::Vector4d homogeneous(
            point.x(), point.y(), point.z(), 1.0);
        const Eigen::Vector4d camera = camera_from_lidar_ * homogeneous;
        if (!camera.array().isFinite().all() || camera.z() <= 1e-4) continue;
        const int u = static_cast<int>(std::lround(
            intrinsics.fx * camera.x() / camera.z() + intrinsics.cx));
        const int v = static_cast<int>(std::lround(
            intrinsics.fy * camera.y() / camera.z() + intrinsics.cy));
        if (u < 0 || v < 0 || u >= canvas.cols || v >= canvas.rows) continue;
        cv::circle(canvas, cv::Point(u, v), visualization_point_radius_px_,
                   depthColor(camera.z(), visualization_minimum_depth_m_,
                              visualization_maximum_depth_m_),
                   cv::FILLED, cv::LINE_AA);
        ++projected_count;
        nearest_depth = std::min(nearest_depth, camera.z());
        farthest_depth = std::max(farthest_depth, camera.z());
      }
    }

    std::size_t fused_count = 0;
    for (const auto& candidate : detections.candidates) {
      xd_uav_detect::PixelBox box;
      if (!pixelBox(candidate, canvas.cols, canvas.rows, &box)) continue;
      const bool fused = candidate.range_valid &&
          candidate.has_relative_position_body;
      if (fused) ++fused_count;
      const cv::Scalar box_color = fused
          ? cv::Scalar(0, 255, 255) : cv::Scalar(0, 165, 255);
      cv::rectangle(canvas,
                    cv::Point(static_cast<int>(box.x_min),
                              static_cast<int>(box.y_min)),
                    cv::Point(static_cast<int>(box.x_max),
                              static_cast<int>(box.y_max)),
                    box_color, 2, cv::LINE_AA);
      std::ostringstream label;
      if (candidate.track_id >= 0) label << "id=" << candidate.track_id << " ";
      label << "cls=" << candidate.class_id;
      if (fused) {
        const auto& position = candidate.relative_position_body;
        const double range = std::sqrt(
            position[0] * position[0] + position[1] * position[1] +
            position[2] * position[2]);
        label << " " << std::fixed << std::setprecision(2) << range << "m";
      }
      const int label_y = std::max(18, static_cast<int>(box.y_min) - 6);
      cv::putText(canvas, label.str(),
                  cv::Point(static_cast<int>(box.x_min), label_y),
                  cv::FONT_HERSHEY_SIMPLEX, 0.5, box_color, 2, cv::LINE_AA);
    }

    cv::Mat panel = canvas.clone();
    const int panel_width = std::min(canvas.cols, 360);
    cv::rectangle(panel, cv::Rect(0, 0, panel_width, 104),
                  cv::Scalar(15, 15, 15), cv::FILLED);
    cv::addWeighted(panel, 0.62, canvas, 0.38, 0.0, canvas);
    std::vector<std::string> lines;
    if (localization_method_ ==
        xd_uav_detect::LocalizationBackendKind::kLidarCamera) {
      lines.push_back("LiDAR -> camera projection");
    } else if (localization_method_ ==
               xd_uav_detect::LocalizationBackendKind::kCameraGroundPlane) {
      lines.push_back("Camera -> ground plane");
    } else {
      lines.push_back("Gimbal single-beam range");
    }
    {
      std::ostringstream line;
      if (localization_method_ ==
          xd_uav_detect::LocalizationBackendKind::kLidarCamera) {
        line << "projected: " << projected_count << "/" << points.size();
      } else if (localization_method_ ==
                 xd_uav_detect::LocalizationBackendKind::kCameraGroundPlane) {
        line << "ground projected";
      } else {
        line << "range endpoint projected";
      }
      line << "  detections: " << detections.candidates.size()
           << "  metric: " << fused_count;
      lines.push_back(line.str());
    }
    {
      std::ostringstream line;
      if (localization_method_ ==
          xd_uav_detect::LocalizationBackendKind::kLidarCamera) {
        line << std::fixed << std::setprecision(3)
             << "cloud-image dt: "
             << (std::isfinite(cloud_difference) ? cloud_difference : -1.0)
             << "s";
        if (projected_count > 0) {
          line << "  visible: " << std::setprecision(1) << nearest_depth
               << "-" << farthest_depth << "m";
        }
      } else {
        line << "world frame: " << world_frame_;
      }
      lines.push_back(line.str());
    }
    lines.push_back("status: " + reason);
    for (std::size_t index = 0; index < lines.size(); ++index) {
      cv::putText(canvas, lines[index], cv::Point(9, 20 + 24 * index),
                  cv::FONT_HERSHEY_SIMPLEX, 0.48, cv::Scalar(235, 235, 235),
                  1, cv::LINE_AA);
    }

    if (localization_method_ ==
        xd_uav_detect::LocalizationBackendKind::kLidarCamera) {
      const int bar_height = std::min(180, std::max(60, canvas.rows - 80));
      const int bar_width = 16;
      const int bar_x = std::max(0, canvas.cols - 38);
      const int bar_y = 35;
      for (int row = 0; row < bar_height; ++row) {
        const double depth = visualization_maximum_depth_m_ -
            (visualization_maximum_depth_m_ - visualization_minimum_depth_m_) *
                row / std::max(1, bar_height - 1);
        cv::line(canvas, cv::Point(bar_x, bar_y + row),
                 cv::Point(bar_x + bar_width, bar_y + row),
                 depthColor(depth, visualization_minimum_depth_m_,
                            visualization_maximum_depth_m_));
      }
      cv::rectangle(canvas, cv::Rect(bar_x, bar_y, bar_width + 1, bar_height),
                    cv::Scalar(240, 240, 240), 1);
      cv::putText(canvas, "Depth", cv::Point(bar_x - 9, bar_y - 10),
                  cv::FONT_HERSHEY_SIMPLEX, 0.38,
                  cv::Scalar(240, 240, 240), 1, cv::LINE_AA);
      cv::putText(
          canvas,
          std::to_string(static_cast<int>(visualization_maximum_depth_m_)) +
              "m",
          cv::Point(bar_x - 3, bar_y + 14), cv::FONT_HERSHEY_SIMPLEX, 0.34,
          cv::Scalar(240, 240, 240), 1, cv::LINE_AA);
      cv::putText(
          canvas,
          std::to_string(static_cast<int>(visualization_minimum_depth_m_)) +
              "m",
          cv::Point(bar_x - 8, bar_y + bar_height - 5),
          cv::FONT_HERSHEY_SIMPLEX, 0.34, cv::Scalar(240, 240, 240), 1,
          cv::LINE_AA);
    }

    std_msgs::Header header = image->header;
    if (!detections.header.stamp.isZero()) header.stamp = detections.header.stamp;
    debug_image_publisher_.publish(
        cv_bridge::CvImage(header, "bgr8", canvas).toImageMsg());
  }

  void publishStatus(const std::size_t detections,
                     const std::size_t fused,
                     const std::string& reason) {
    std_msgs::String status;
    std::ostringstream stream;
    stream << "detections=" << detections << " metric=" << fused
           << " method=" << localization_method_name_
           << " calibrated="
           << (localization_method_ !=
                       xd_uav_detect::LocalizationBackendKind::kLidarCamera ||
                       calibrated_
                   ? "true"
                   : "false")
           << " intrinsics=" << (have_intrinsics_ ? "true" : "false")
           << " reason=" << reason;
    status.data = stream.str();
    status_publisher_.publish(status);
  }

  void updateDebugState(const xd_uav_track::DetectionArray& detections,
                        const std::string& reason) {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_debug_detections_ = detections;
    latest_debug_reason_ = reason;
  }

  void setMetricPosition(
      const Eigen::Vector3d& position_body_flu,
      const Eigen::Matrix3d& covariance_body_flu,
      xd_uav_track::DetectionCandidate* candidate) const {
    if (candidate == nullptr) return;
    const Eigen::Matrix3d flu_to_frd =
        (Eigen::Vector3d(1.0, -1.0, -1.0)).asDiagonal();
    const Eigen::Vector3d position_frd = flu_to_frd * position_body_flu;
    const Eigen::Matrix3d covariance_frd =
        flu_to_frd * covariance_body_flu * flu_to_frd;
    candidate->has_relative_position_body = true;
    candidate->relative_position_body = {
        static_cast<float>(position_frd.x()),
        static_cast<float>(position_frd.y()),
        static_cast<float>(position_frd.z())};
    candidate->position_covariance = {
        static_cast<float>(covariance_frd(0, 0)),
        static_cast<float>(covariance_frd(0, 1)),
        static_cast<float>(covariance_frd(0, 2)),
        static_cast<float>(covariance_frd(1, 0)),
        static_cast<float>(covariance_frd(1, 1)),
        static_cast<float>(covariance_frd(1, 2)),
        static_cast<float>(covariance_frd(2, 0)),
        static_cast<float>(covariance_frd(2, 1)),
        static_cast<float>(covariance_frd(2, 2))};
    candidate->range_valid = true;
  }

  void publishDetections(
      const xd_uav_track::DetectionArray& detections) {
    detections_publisher_.publish(detections);
    if (compatibility_detections_publisher_) {
      compatibility_detections_publisher_.publish(detections);
    }

    xd_uav_detect::WorldDetectionArray world_output;
    world_output.header = detections.header;
    world_output.header.frame_id = world_frame_;
    world_output.detections.resize(detections.candidates.size());

    bool needs_transform = false;
    for (const auto& candidate : detections.candidates) {
      needs_transform = needs_transform ||
          (candidate.range_valid && candidate.has_relative_position_body);
    }

    Eigen::Matrix4d world_from_body = Eigen::Matrix4d::Identity();
    bool transform_valid = false;
    if (needs_transform && !world_frame_.empty() && !body_frame_.empty() &&
        !detections.header.stamp.isZero()) {
      try {
        world_from_body = transformMatrix(tf_buffer_.lookupTransform(
            world_frame_, body_frame_, detections.header.stamp,
            ros::Duration(tf_timeout_sec_)));
        transform_valid = true;
      } catch (const tf2::TransformException& error) {
        ROS_WARN_THROTTLE(
            2.0, "[xd_uav_detect] world/body TF unavailable: %s",
            error.what());
      }
    }

    const Eigen::Matrix3d frd_to_flu =
        (Eigen::Vector3d(1.0, -1.0, -1.0)).asDiagonal();
    const Eigen::Matrix3d world_from_body_rotation =
        world_from_body.block<3, 3>(0, 0);
    for (std::size_t index = 0; index < detections.candidates.size(); ++index) {
      const auto& source = detections.candidates[index];
      auto& target = world_output.detections[index];
      target.source_candidate_index = index;
      target.track_id = source.track_id;
      target.class_id = source.class_id;
      target.confidence = source.confidence;
      target.image_source = detections.image_source;
      target.sensor_id = detections.sensor_id;
      target.detector_name = detections.detector_name;
      target.model_version = detections.model_version;
      target.position_valid = false;
      if (!transform_valid || !source.range_valid ||
          !source.has_relative_position_body) {
        continue;
      }

      const Eigen::Vector3d position_frd(
          source.relative_position_body[0],
          source.relative_position_body[1],
          source.relative_position_body[2]);
      const Eigen::Vector4d position_body(
          (frd_to_flu * position_frd).x(),
          (frd_to_flu * position_frd).y(),
          (frd_to_flu * position_frd).z(), 1.0);
      const Eigen::Vector4d position_world = world_from_body * position_body;
      Eigen::Matrix3d covariance_frd;
      for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 3; ++column) {
          covariance_frd(row, column) =
              source.position_covariance[3 * row + column];
        }
      }
      const Eigen::Matrix3d covariance_body =
          frd_to_flu * covariance_frd * frd_to_flu;
      const Eigen::Matrix3d covariance_world =
          world_from_body_rotation * covariance_body *
          world_from_body_rotation.transpose();
      if (!position_world.head<3>().array().isFinite().all() ||
          !covariance_world.array().isFinite().all()) {
        continue;
      }
      target.position_world.x = position_world.x();
      target.position_world.y = position_world.y();
      target.position_world.z = position_world.z();
      for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 3; ++column) {
          target.position_covariance_world[3 * row + column] =
              covariance_world(row, column);
        }
      }
      target.position_valid = true;
    }
    world_detections_publisher_.publish(world_output);
  }

  bool lookupGroundTransform(
      const std::string& target_frame, const std::string& camera_frame,
      const ros::Time& capture_stamp,
      geometry_msgs::TransformStamped* transform,
      std::string* error_message) {
    if (transform == nullptr || target_frame.empty() || camera_frame.empty()) {
      if (error_message != nullptr) *error_message = "empty TF frame";
      return false;
    }
    try {
      *transform = tf_buffer_.lookupTransform(
          target_frame, camera_frame,
          capture_stamp.isZero() ? ros::Time(0) : capture_stamp,
          ros::Duration(tf_timeout_sec_));
      return true;
    } catch (const tf2::TransformException& capture_error) {
      if (!ground_tf_use_latest_on_failure_ || capture_stamp.isZero()) {
        if (error_message != nullptr) *error_message = capture_error.what();
        return false;
      }
      try {
        *transform = tf_buffer_.lookupTransform(
            target_frame, camera_frame, ros::Time(0),
            ros::Duration(tf_timeout_sec_));
        ROS_WARN_THROTTLE(
            2.0,
            "[xd_uav_detect] capture-time TF unavailable; using latest TF");
        return true;
      } catch (const tf2::TransformException& latest_error) {
        if (error_message != nullptr) *error_message = latest_error.what();
        return false;
      }
    }
  }

  void detectionsCallback(
      const xd_uav_track::DetectionArray::ConstPtr& input) {
    xd_uav_track::DetectionArray output = *input;
    xd_uav_detect::CameraIntrinsics intrinsics;
    sensor_msgs::PointCloud2::ConstPtr cloud;
    sensor_msgs::Range::ConstPtr range;
    unsigned int image_width = 0;
    unsigned int image_height = 0;
    std::string camera_info_frame;
    double cloud_difference = std::numeric_limits<double>::infinity();
    double range_difference = std::numeric_limits<double>::infinity();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      intrinsics = intrinsics_;
      image_width = image_width_;
      image_height = image_height_;
      camera_info_frame = camera_info_frame_;
      if (localization_method_ ==
          xd_uav_detect::LocalizationBackendKind::kLidarCamera) {
        cloud = closestCloud(input->header.stamp, &cloud_difference);
      } else if (localization_method_ ==
                 xd_uav_detect::LocalizationBackendKind::kGimbalLaserRange) {
        range = closestRange(input->header.stamp, &range_difference);
      }
    }
    if (output.image_width == 0) {
      output.image_width = image_width > 0 ? image_width : intrinsics.width;
    }
    if (output.image_height == 0) {
      output.image_height = image_height > 0 ? image_height : intrinsics.height;
    }
    // External detectors may publish boxes in a resized image. Scale the
    // calibrated pinhole model to that coordinate system before projection.
    if (intrinsics.valid() && output.image_width > 0 &&
        output.image_height > 0 &&
        (output.image_width != intrinsics.width ||
         output.image_height != intrinsics.height)) {
      const double scale_x = static_cast<double>(output.image_width) /
          intrinsics.width;
      const double scale_y = static_cast<double>(output.image_height) /
          intrinsics.height;
      intrinsics.fx *= scale_x;
      intrinsics.cx *= scale_x;
      intrinsics.fy *= scale_y;
      intrinsics.cy *= scale_y;
      intrinsics.width = output.image_width;
      intrinsics.height = output.image_height;
    }

    auto publish_without_metric = [&](const std::string& reason) {
      for (auto& candidate : output.candidates) clearMetricState(&candidate);
      publishDetections(output);
      publishStatus(output.candidates.size(), 0, reason);
      updateDebugState(output, reason);
    };
    if (output.candidates.empty()) {
      publish_without_metric("no detections");
      return;
    }
    if (!localization_enabled_) {
      publish_without_metric("localization disabled");
      return;
    }
    if (!have_intrinsics_ || !intrinsics.valid()) {
      publish_without_metric("camera intrinsics unavailable");
      return;
    }

    if (localization_method_ ==
        xd_uav_detect::LocalizationBackendKind::kGimbalLaserRange) {
      if (gimbal_require_detection_timestamp_ &&
          input->header.stamp.isZero()) {
        publish_without_metric("gimbal detection timestamp unavailable");
        return;
      }
      if (range == nullptr ||
          range_difference > maximum_range_time_difference_sec_) {
        publish_without_metric("synchronized laser range unavailable");
        return;
      }
      if (range->header.stamp.isZero() || range->header.frame_id.empty() ||
          !std::isfinite(range->range) ||
          range->range <= range->min_range || range->range >= range->max_range) {
        publish_without_metric("invalid laser range");
        return;
      }
      const std::string camera_frame = !configured_camera_frame_.empty()
          ? configured_camera_frame_
          : (!camera_info_frame.empty() ? camera_info_frame
                                        : input->header.frame_id);
      const std::string laser_frame = !configured_laser_frame_.empty()
          ? configured_laser_frame_ : range->header.frame_id;
      if (body_frame_.empty() || camera_frame.empty() || laser_frame.empty()) {
        publish_without_metric("body/camera/laser frame unavailable");
        return;
      }
      if (gimbal_require_matching_camera_frame_ &&
          !input->header.frame_id.empty() &&
          input->header.frame_id != camera_frame) {
        publish_without_metric(
            "detection and CameraInfo camera frames do not match");
        return;
      }

      geometry_msgs::TransformStamped camera_from_laser_message;
      geometry_msgs::TransformStamped body_from_laser_message;
      try {
        camera_from_laser_message = tf_buffer_.lookupTransform(
            camera_frame, laser_frame, input->header.stamp,
            ros::Duration(tf_timeout_sec_));
        body_from_laser_message = tf_buffer_.lookupTransform(
            body_frame_, laser_frame, input->header.stamp,
            ros::Duration(tf_timeout_sec_));
      } catch (const tf2::TransformException& error) {
        ROS_WARN_THROTTLE(
            2.0, "[xd_uav_detect] gimbal laser TF unavailable: %s",
            error.what());
        publish_without_metric("camera/body laser TF unavailable");
        return;
      }

      std::vector<xd_uav_detect::PixelBox> boxes(output.candidates.size());
      for (std::size_t index = 0; index < output.candidates.size(); ++index) {
        if (output.candidates[index].range_valid &&
            output.candidates[index].has_relative_position_body) {
          continue;
        }
        pixelBox(output.candidates[index], output.image_width,
                 output.image_height, &boxes[index]);
      }
      xd_uav_detect::LocalizationInput localization_input;
      localization_input.boxes = boxes;
      localization_input.intrinsics = intrinsics;
      localization_input.range_m = range->range;
      localization_input.camera_from_sensor =
          transformMatrix(camera_from_laser_message);
      localization_input.body_from_sensor =
          transformMatrix(body_from_laser_message);
      const auto localized =
          localization_backend_->localize(localization_input);
      const bool backend_valid = std::any_of(
          localized.begin(), localized.end(),
          [](const xd_uav_detect::LocalizationEstimate& estimate) {
            return estimate.valid;
          });
      std::size_t metric_count = 0;
      for (std::size_t index = 0; index < output.candidates.size(); ++index) {
        auto& candidate = output.candidates[index];
        if (candidate.range_valid && candidate.has_relative_position_body) {
          ++metric_count;
        } else if (index < localized.size() && localized[index].valid) {
          setMetricPosition(localized[index].position_body_flu,
                            localized[index].covariance_body_flu, &candidate);
          ++metric_count;
        } else {
          clearMetricState(&candidate);
        }
      }
      publishDetections(output);
      const std::string result_reason = backend_valid
          ? "ok" : "laser axis does not identify exactly one detection";
      publishStatus(output.candidates.size(), metric_count, result_reason);
      updateDebugState(output, result_reason);
      return;
    }

    if (localization_method_ ==
        xd_uav_detect::LocalizationBackendKind::kCameraGroundPlane) {
      if (ground_require_detection_timestamp_ && input->header.stamp.isZero()) {
        publish_without_metric("ground-plane detection timestamp unavailable");
        return;
      }
      const std::string camera_frame = !configured_camera_frame_.empty()
          ? configured_camera_frame_
          : (!camera_info_frame.empty() ? camera_info_frame
                                        : input->header.frame_id);
      if (world_frame_.empty() || body_frame_.empty() || camera_frame.empty()) {
        publish_without_metric("world/body/camera frame unavailable");
        return;
      }
      if (ground_require_matching_camera_frame_ &&
          !input->header.frame_id.empty() &&
          input->header.frame_id != camera_frame) {
        publish_without_metric(
            "detection and CameraInfo camera frames do not match");
        return;
      }
      geometry_msgs::TransformStamped world_from_camera_message;
      geometry_msgs::TransformStamped body_from_camera_message;
      std::string tf_error;
      if (!lookupGroundTransform(
              world_frame_, camera_frame, input->header.stamp,
              &world_from_camera_message, &tf_error) ||
          !lookupGroundTransform(
              body_frame_, camera_frame, input->header.stamp,
              &body_from_camera_message, &tf_error)) {
        ROS_WARN_THROTTLE(
            2.0, "[xd_uav_detect] ground-plane camera TF unavailable: %s",
            tf_error.c_str());
        publish_without_metric("world/body camera TF unavailable");
        return;
      }
      const Eigen::Matrix4d world_from_camera =
          transformMatrix(world_from_camera_message);
      const Eigen::Matrix4d body_from_camera =
          transformMatrix(body_from_camera_message);
      xd_uav_detect::LocalizationInput localization_input;
      localization_input.intrinsics = intrinsics;
      localization_input.world_from_camera = world_from_camera;
      localization_input.body_from_sensor = body_from_camera;
      localization_input.boxes.resize(output.candidates.size());
      for (std::size_t index = 0; index < output.candidates.size(); ++index) {
        pixelBox(output.candidates[index], output.image_width,
                 output.image_height, &localization_input.boxes[index]);
      }
      const auto localized =
          localization_backend_->localize(localization_input);
      std::size_t projected_count = 0;
      for (std::size_t index = 0; index < output.candidates.size(); ++index) {
        auto& candidate = output.candidates[index];
        if (candidate.range_valid && candidate.has_relative_position_body) {
          ++projected_count;
          continue;
        }
        if (index >= localized.size() || !localized[index].valid) {
          clearMetricState(&candidate);
          continue;
        }
        setMetricPosition(localized[index].position_body_flu,
                          localized[index].covariance_body_flu, &candidate);
        ++projected_count;
      }
      publishDetections(output);
      const std::string result_reason = projected_count > 0
          ? "ok"
          : "no valid camera/ground intersection";
      publishStatus(output.candidates.size(), projected_count, result_reason);
      updateDebugState(output, result_reason);
      return;
    }

    if (!fusion_enabled_) {
      publish_without_metric("fusion disabled");
      return;
    }
    if (!calibrated_ || !calibration_parameters_valid_) {
      publish_without_metric("calibration parameters not enabled");
      return;
    }
    if (cloud == nullptr ||
        cloud_difference > maximum_cloud_time_difference_sec_) {
      publish_without_metric("synchronized point cloud unavailable");
      return;
    }
    std::vector<xd_uav_detect::LidarPoint> points;
    cloudPoints(*cloud, &points);
    geometry_msgs::TransformStamped body_from_lidar_message;
    try {
      const ros::Time lookup_stamp = cloud->header.stamp.isZero()
          ? ros::Time(0) : cloud->header.stamp;
      body_from_lidar_message = tf_buffer_.lookupTransform(
          body_frame_, cloud->header.frame_id, lookup_stamp,
          ros::Duration(tf_timeout_sec_));
    } catch (const tf2::TransformException& error) {
      ROS_WARN_THROTTLE(2.0,
          "[xd_uav_detect] body/lidar TF unavailable: %s", error.what());
      publish_without_metric("body/lidar TF unavailable");
      return;
    }

    if (points.empty()) {
      publish_without_metric("point cloud has no readable xyz points");
      return;
    }
    const Eigen::Matrix4d body_from_lidar =
        transformMatrix(body_from_lidar_message);
    xd_uav_detect::LocalizationInput localization_input;
    localization_input.intrinsics = intrinsics;
    localization_input.points = points;
    localization_input.camera_from_sensor = camera_from_lidar_;
    localization_input.body_from_sensor = body_from_lidar;
    localization_input.boxes.resize(output.candidates.size());
    for (std::size_t index = 0; index < output.candidates.size(); ++index) {
      pixelBox(output.candidates[index], output.image_width,
               output.image_height, &localization_input.boxes[index]);
    }
    const auto localized = localization_backend_->localize(localization_input);
    std::size_t fused_count = 0;
    for (std::size_t index = 0; index < output.candidates.size(); ++index) {
      auto& candidate = output.candidates[index];
      if (candidate.range_valid && candidate.has_relative_position_body) {
        ++fused_count;
        continue;  // Preserve a valid metric measurement from the upstream detector.
      }
      if (index >= localized.size() || !localized[index].valid) {
        clearMetricState(&candidate);
        continue;
      }
      setMetricPosition(localized[index].position_body_flu,
                        localized[index].covariance_body_flu, &candidate);
      ++fused_count;
    }
    publishDetections(output);
    const std::string result_reason =
        fused_count > 0 ? "ok" : "no target cloud cluster";
    publishStatus(output.candidates.size(), fused_count, result_reason);
    updateDebugState(output, result_reason);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber detections_subscriber_;
  ros::Subscriber image_subscriber_;
  ros::Subscriber camera_info_subscriber_;
  ros::Subscriber cloud_subscriber_;
  ros::Subscriber range_subscriber_;
  ros::Publisher detections_publisher_;
  ros::Publisher compatibility_detections_publisher_;
  ros::Publisher world_detections_publisher_;
  ros::Publisher status_publisher_;
  ros::Publisher debug_image_publisher_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::unique_ptr<xd_uav_detect::LocalizationBackend> localization_backend_;
  mutable std::mutex mutex_;
  std::deque<sensor_msgs::PointCloud2::ConstPtr> cloud_buffer_;
  std::deque<sensor_msgs::Range::ConstPtr> range_buffer_;
  xd_uav_detect::CameraIntrinsics intrinsics_;
  xd_uav_track::DetectionArray latest_debug_detections_;
  std::string latest_debug_reason_{"waiting for detections"};
  std::string camera_info_frame_;
  Eigen::Matrix4d camera_from_lidar_{Eigen::Matrix4d::Identity()};
  ros::Time image_stamp_;
  ros::Time camera_info_stamp_;
  unsigned int image_width_{0};
  unsigned int image_height_{0};
  int cloud_buffer_size_{10};
  int range_buffer_size_{20};
  int point_stride_{1};
  double maximum_cloud_time_difference_sec_{0.12};
  double maximum_range_time_difference_sec_{0.05};
  double tf_timeout_sec_{0.03};
  double visualization_minimum_depth_m_{0.3};
  double visualization_maximum_depth_m_{30.0};
  int visualization_point_stride_{1};
  int visualization_point_radius_px_{2};
  xd_uav_detect::LocalizationBackendKind localization_method_{
      xd_uav_detect::LocalizationBackendKind::kLidarCamera};
  std::string localization_method_name_{"lidar_camera"};
  bool localization_enabled_{true};
  bool fusion_enabled_{true};
  bool visualization_enabled_{true};
  bool calibrated_{false};
  bool calibration_parameters_valid_{false};
  bool have_intrinsics_{false};
  std::string body_frame_{"base_link"};
  std::string world_frame_{"map"};
  std::string configured_camera_frame_;
  std::string configured_laser_frame_;
  bool ground_tf_use_latest_on_failure_{false};
  bool ground_require_detection_timestamp_{true};
  bool ground_require_matching_camera_frame_{true};
  bool gimbal_require_detection_timestamp_{true};
  bool gimbal_require_matching_camera_frame_{true};
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "xd_uav_detect");
  XdUavDetectNode node;
  ros::spin();
  return 0;
}
