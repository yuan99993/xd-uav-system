#include <algorithm>
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
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_msgs/String.h>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <xd_uav_detect/lidar_camera_fusion.hpp>
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
    xd_uav_detect::FusionConfig fusion_config;
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
    fusion_.reset(new xd_uav_detect::LidarCameraFusion(fusion_config));

    private_nh_.param("fusion/enabled", fusion_enabled_, true);
    private_nh_.param("calibration/calibrated", calibrated_, false);
    private_nh_.param("fusion/maximum_cloud_time_difference_sec",
                      maximum_cloud_time_difference_sec_, 0.12);
    private_nh_.param("fusion/cloud_buffer_size", cloud_buffer_size_, 10);
    private_nh_.param("fusion/point_stride", point_stride_, 1);
    private_nh_.param("fusion/tf_timeout_sec", tf_timeout_sec_, 0.03);
    private_nh_.param("frames/body", body_frame_, std::string("uav1/base_link"));
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
    if (calibrated_ && !calibration_parameters_valid_) {
      ROS_ERROR("[xd_uav_detect] calibrated=true but translation_xyz or "
                "rotation_ypr is invalid; 3D fusion is disabled");
      calibrated_ = false;
    }

    std::string uav_name{"uav1"};
    private_nh_.param("vehicle/uav_name", uav_name, uav_name);
    std::string detections_input =
        "/" + uav_name + "/detect/input/detections_2d";
    std::string detections_output = "/" + uav_name + "/track/detections";
    std::string image_topic = "/" + uav_name + "/camera/image_raw";
    std::string camera_info_topic = "/" + uav_name + "/camera/camera_info";
    std::string cloud_topic = "/" + uav_name + "/fastlio/points";
    std::string status_topic = "/" + uav_name + "/detect/status";
    std::string debug_image_topic = "/" + uav_name + "/detect/debug/image";
    private_nh_.param("interfaces/input/detections_2d", detections_input,
                      detections_input);
    private_nh_.param("interfaces/input/image", image_topic, image_topic);
    private_nh_.param("interfaces/input/camera_info", camera_info_topic,
                      camera_info_topic);
    private_nh_.param("interfaces/input/point_cloud", cloud_topic, cloud_topic);
    private_nh_.param("interfaces/output/detections_3d", detections_output,
                      detections_output);
    private_nh_.param("interfaces/output/status", status_topic, status_topic);
    private_nh_.param("interfaces/output/debug_image", debug_image_topic,
                      debug_image_topic);

    detections_publisher_ =
        nh_.advertise<xd_uav_track::DetectionArray>(detections_output, 10);
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
    cloud_subscriber_ = nh_.subscribe(
        cloud_topic, 2, &XdUavDetectNode::cloudCallback, this,
        ros::TransportHints().tcpNoDelay());

    ROS_INFO("[xd_uav_detect] external detections=%s -> fused detections=%s; "
             "fusion=%s; calibration=%s; debug_image=%s",
             detections_input.c_str(), detections_output.c_str(),
             fusion_enabled_ ? "enabled" : "disabled",
             calibrated_ ? "ready" : "waiting for calibration parameters",
             debug_image_topic.c_str());
  }

 private:
  void imageCallback(const sensor_msgs::Image::ConstPtr& message) {
    if (message->width == 0 || message->height == 0) return;
    xd_uav_detect::CameraIntrinsics intrinsics;
    sensor_msgs::PointCloud2::ConstPtr cloud;
    xd_uav_track::DetectionArray detections;
    std::string reason;
    double cloud_difference = std::numeric_limits<double>::infinity();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      image_width_ = message->width;
      image_height_ = message->height;
      image_stamp_ = message->header.stamp;
      intrinsics = intrinsics_;
      cloud = closestCloud(message->header.stamp, &cloud_difference);
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
    if (calibrated_ && calibration_parameters_valid_ && intrinsics.valid()) {
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
    lines.push_back("LiDAR -> camera projection");
    {
      std::ostringstream line;
      line << "projected: " << projected_count << "/" << points.size()
           << "  detections: " << detections.candidates.size()
           << "  fused: " << fused_count;
      lines.push_back(line.str());
    }
    {
      std::ostringstream line;
      line << std::fixed << std::setprecision(3)
           << "cloud-image dt: "
           << (std::isfinite(cloud_difference) ? cloud_difference : -1.0)
           << "s";
      if (projected_count > 0) {
        line << "  visible: " << std::setprecision(1) << nearest_depth
             << "-" << farthest_depth << "m";
      }
      lines.push_back(line.str());
    }
    lines.push_back("status: " + reason);
    for (std::size_t index = 0; index < lines.size(); ++index) {
      cv::putText(canvas, lines[index], cv::Point(9, 20 + 24 * index),
                  cv::FONT_HERSHEY_SIMPLEX, 0.48, cv::Scalar(235, 235, 235),
                  1, cv::LINE_AA);
    }

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
                cv::FONT_HERSHEY_SIMPLEX, 0.38, cv::Scalar(240, 240, 240), 1,
                cv::LINE_AA);
    cv::putText(canvas,
                std::to_string(static_cast<int>(visualization_maximum_depth_m_)) +
                    "m",
                cv::Point(bar_x - 3, bar_y + 14), cv::FONT_HERSHEY_SIMPLEX,
                0.34, cv::Scalar(240, 240, 240), 1, cv::LINE_AA);
    cv::putText(canvas,
                std::to_string(static_cast<int>(visualization_minimum_depth_m_)) +
                    "m",
                cv::Point(bar_x - 8, bar_y + bar_height - 5),
                cv::FONT_HERSHEY_SIMPLEX, 0.34,
                cv::Scalar(240, 240, 240), 1, cv::LINE_AA);

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
    stream << "detections=" << detections << " fused=" << fused
           << " calibrated=" << (calibrated_ ? "true" : "false")
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

  void detectionsCallback(
      const xd_uav_track::DetectionArray::ConstPtr& input) {
    xd_uav_track::DetectionArray output = *input;
    xd_uav_detect::CameraIntrinsics intrinsics;
    sensor_msgs::PointCloud2::ConstPtr cloud;
    unsigned int image_width = 0;
    unsigned int image_height = 0;
    double cloud_difference = std::numeric_limits<double>::infinity();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      intrinsics = intrinsics_;
      image_width = image_width_;
      image_height = image_height_;
      cloud = closestCloud(input->header.stamp, &cloud_difference);
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

    std::vector<xd_uav_detect::LidarPoint> points;
    if (cloud != nullptr) cloudPoints(*cloud, &points);

    auto publish_without_fusion = [&](const std::string& reason) {
      for (auto& candidate : output.candidates) clearMetricState(&candidate);
      detections_publisher_.publish(output);
      publishStatus(output.candidates.size(), 0, reason);
      updateDebugState(output, reason);
    };
    if (output.candidates.empty()) {
      publish_without_fusion("no detections");
      return;
    }
    if (!fusion_enabled_) {
      publish_without_fusion("fusion disabled");
      return;
    }
    if (!calibrated_ || !calibration_parameters_valid_) {
      publish_without_fusion("calibration parameters not enabled");
      return;
    }
    if (!have_intrinsics_ || !intrinsics.valid()) {
      publish_without_fusion("camera intrinsics unavailable");
      return;
    }
    if (cloud == nullptr ||
        cloud_difference > maximum_cloud_time_difference_sec_) {
      publish_without_fusion("synchronized point cloud unavailable");
      return;
    }
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
      publish_without_fusion("body/lidar TF unavailable");
      return;
    }

    if (points.empty()) {
      publish_without_fusion("point cloud has no readable xyz points");
      return;
    }
    const Eigen::Matrix4d body_from_lidar =
        transformMatrix(body_from_lidar_message);
    const Eigen::Matrix3d flu_to_frd =
        (Eigen::Vector3d(1.0, -1.0, -1.0)).asDiagonal();
    std::size_t fused_count = 0;
    for (auto& candidate : output.candidates) {
      if (candidate.range_valid && candidate.has_relative_position_body) {
        ++fused_count;
        continue;  // Preserve a valid metric measurement from the upstream detector.
      }
      xd_uav_detect::PixelBox box;
      if (!pixelBox(candidate, output.image_width, output.image_height, &box)) {
        clearMetricState(&candidate);
        continue;
      }
      const auto fused = fusion_->estimate(
          points, box, intrinsics, camera_from_lidar_, body_from_lidar);
      if (!fused.valid) {
        clearMetricState(&candidate);
        continue;
      }
      const Eigen::Vector3d position_frd =
          flu_to_frd * fused.position_body_flu;
      const Eigen::Matrix3d covariance_frd =
          flu_to_frd * fused.covariance_body_flu * flu_to_frd;
      candidate.has_relative_position_body = true;
      candidate.relative_position_body = {
          static_cast<float>(position_frd.x()),
          static_cast<float>(position_frd.y()),
          static_cast<float>(position_frd.z())};
      candidate.position_covariance = {
          static_cast<float>(covariance_frd(0, 0)),
          static_cast<float>(covariance_frd(0, 1)),
          static_cast<float>(covariance_frd(0, 2)),
          static_cast<float>(covariance_frd(1, 0)),
          static_cast<float>(covariance_frd(1, 1)),
          static_cast<float>(covariance_frd(1, 2)),
          static_cast<float>(covariance_frd(2, 0)),
          static_cast<float>(covariance_frd(2, 1)),
          static_cast<float>(covariance_frd(2, 2))};
      candidate.range_valid = true;
      ++fused_count;
    }
    detections_publisher_.publish(output);
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
  ros::Publisher detections_publisher_;
  ros::Publisher status_publisher_;
  ros::Publisher debug_image_publisher_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::unique_ptr<xd_uav_detect::LidarCameraFusion> fusion_;
  mutable std::mutex mutex_;
  std::deque<sensor_msgs::PointCloud2::ConstPtr> cloud_buffer_;
  xd_uav_detect::CameraIntrinsics intrinsics_;
  xd_uav_track::DetectionArray latest_debug_detections_;
  std::string latest_debug_reason_{"waiting for detections"};
  Eigen::Matrix4d camera_from_lidar_{Eigen::Matrix4d::Identity()};
  ros::Time image_stamp_;
  ros::Time camera_info_stamp_;
  unsigned int image_width_{0};
  unsigned int image_height_{0};
  int cloud_buffer_size_{10};
  int point_stride_{1};
  double maximum_cloud_time_difference_sec_{0.12};
  double tf_timeout_sec_{0.03};
  double visualization_minimum_depth_m_{0.3};
  double visualization_maximum_depth_m_{30.0};
  int visualization_point_stride_{1};
  int visualization_point_radius_px_{2};
  bool fusion_enabled_{true};
  bool visualization_enabled_{true};
  bool calibrated_{false};
  bool calibration_parameters_valid_{false};
  bool have_intrinsics_{false};
  std::string body_frame_{"uav1/base_link"};
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "xd_uav_detect");
  XdUavDetectNode node;
  ros::spin();
  return 0;
}
