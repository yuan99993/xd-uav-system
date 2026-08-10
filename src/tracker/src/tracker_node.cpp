#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <cv_bridge/cv_bridge.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <ros/ros.h>
#include <tracker/ExternalInput.h>
#include <tracker/DetectionArray.h>
#include <tracker/NormalizedError.h>
#include <tracker/SelectTrack.h>
#include <tracker/TrackStateArray.h>
#include <tracker/TrackingOutput.h>

#include <pod_msgs/GimbalState.h>
#include <tracker/multi_track_manager.hpp>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

namespace {
constexpr double kPi = 3.14159265358979323846;
double clamp(const double value, const double low, const double high) {
  return std::max(low, std::min(high, value));
}
std::string lower(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(),
                 [](const unsigned char c) { return std::tolower(c); });
  return value;
}
struct Box {
  int x1{0}, y1{0}, x2{0}, y2{0};
  double width() const { return x2 - x1; }
  double height() const { return y2 - y1; }
  double cx() const { return 0.5 * (x1 + x2); }
  double cy() const { return 0.5 * (y1 + y2); }
};

struct Candidate {
  Box box;
  int track_id{-1};
  int class_id{-1};
  bool stable_id{false};
  double confidence{0.0};
  std::vector<float> appearance;
  bool has_relative_position_body{false};
  std::array<double, 3> relative_position_body{0.0, 0.0, 0.0};
  bool has_relative_velocity_body{false};
  std::array<double, 3> relative_velocity_body{0.0, 0.0, 0.0};
  std::array<double, 9> position_covariance{0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                             0.0, 0.0, 0.0};
  std::array<double, 9> velocity_covariance{0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                             0.0, 0.0, 0.0};
  bool range_valid{false};
};

struct CameraCalibration {
  int width{0};
  int height{0};
  double fx{0.0};
  double fy{0.0};
  double cx{0.0};
  double cy{0.0};
  std::string frame_id;
  ros::Time stamp;
  bool valid{false};
};

// PixEagle SmartTracker keeps detector association separate from the selected
// target state.  Preserve that distinction here so downstream control can
// treat a direct ID update differently from a tentative ReID recovery.
enum class AssociationMethod {
  kNone,
  kDirectId,
  kSpatial,
  kDistance,
  kAppearance,
  kByteTrackLow,
  kTentative,
  kPredicted,
  kLost,
};

const char* associationName(const AssociationMethod method) {
  switch (method) {
    case AssociationMethod::kDirectId: return "id";
    case AssociationMethod::kSpatial: return "spatial";
    case AssociationMethod::kDistance: return "distance";
    case AssociationMethod::kAppearance: return "appearance";
    case AssociationMethod::kByteTrackLow: return "bytetrack_low";
    case AssociationMethod::kTentative: return "tentative";
    case AssociationMethod::kPredicted: return "predicted";
    case AssociationMethod::kLost: return "lost";
    case AssociationMethod::kNone: return "none";
  }
  return "none";
}

double associationReliability(const AssociationMethod method) {
  switch (method) {
    case AssociationMethod::kDirectId: return 1.0;
    case AssociationMethod::kSpatial: return 0.95;
    case AssociationMethod::kAppearance: return 0.88;
    // Low-score ByteTrack recovery is useful for maintaining an existing
    // track, but it must never carry the same control authority as a fresh
    // high-confidence observation.
    case AssociationMethod::kByteTrackLow: return 0.60;
    case AssociationMethod::kDistance: return 0.78;
    case AssociationMethod::kTentative:
    case AssociationMethod::kPredicted:
    case AssociationMethod::kLost:
    case AssociationMethod::kNone: return 0.0;
  }
  return 0.0;
}

double iou(const Box& a, const Box& b) {
  const double x1 = std::max(a.x1, b.x1);
  const double y1 = std::max(a.y1, b.y1);
  const double x2 = std::min(a.x2, b.x2);
  const double y2 = std::min(a.y2, b.y2);
  const double intersection = std::max(0.0, x2 - x1) *
                              std::max(0.0, y2 - y1);
  const double union_area = a.width() * a.height() +
                            b.width() * b.height() - intersection;
  return union_area > 1e-6 ? intersection / union_area : 0.0;
}

class KalmanBox {
 public:
  void initialize(const Box& box, const double q_scale, const double r_scale) {
    process_noise_ = std::max(1e-6, q_scale);
    f_.setIdentity();
    for (int i = 0; i < 4; ++i) f_(i, i + 4) = 1.0;
    h_.setZero();
    h_(0, 0) = h_(1, 1) = h_(2, 2) = h_(3, 3) = 1.0;
    q_.setZero();
    r_.setIdentity();
    r_.diagonal() << 1.0, 1.0, 0.02, 0.02;
    r_ *= r_scale;
    p_.setIdentity();
    p_.diagonal() << 10.0, 10.0, 0.05, 0.05,
        400.0, 400.0, 0.5, 0.5;
    x_.setZero();
    x_.head<4>() = measurement(box);
  }
  void update(const Box& box) {
    const Eigen::Matrix<double, 4, 1> residual = measurement(box) - h_ * x_;
    const Eigen::Matrix4d innovation = h_ * p_ * h_.transpose() + r_;
    const Eigen::Matrix<double, 8, 4> gain =
        p_ * h_.transpose() * innovation.ldlt().solve(Eigen::Matrix4d::Identity());
    x_ += gain * residual;
    // Joseph form preserves covariance symmetry/positive semidefiniteness
    // better than the abbreviated (I-KH)P update under repeated gating.
    const Eigen::Matrix<double, 8, 8> identity =
        Eigen::Matrix<double, 8, 8>::Identity();
    const Eigen::Matrix<double, 8, 8> residual_projection = identity - gain * h_;
    p_ = residual_projection * p_ * residual_projection.transpose() +
         gain * r_ * gain.transpose();
  }
  Box predict(const double dt) {
    const double bounded_dt = clamp(dt, 1e-3, 0.25);
    f_.setIdentity();
    for (int i = 0; i < 4; ++i) f_(i, i + 4) = bounded_dt;
    // State: [cx, cy, log(width), log(height), and their velocities].
    // Log-size dynamics preserve a positive, well-conditioned box geometry.
    q_.setZero();
    const double q = process_noise_;
    const double dt2 = bounded_dt * bounded_dt;
    const double dt3 = dt2 * bounded_dt;
    const double dt4 = dt2 * dt2;
    for (const int index : {0, 1, 2, 3}) {
      const int velocity = index + 4;
      const double scale = index < 2 ? q : 0.02 * q;
      q_(index, index) = 0.25 * dt4 * scale;
      q_(index, velocity) = q_(velocity, index) = 0.5 * dt3 * scale;
      q_(velocity, velocity) = dt2 * scale;
    }
    x_ = f_ * x_;
    p_ = f_ * p_ * f_.transpose() + q_;
    return box();
  }
  double innovationNis(const Box& box) const {
    const auto residual = measurement(box) - h_ * x_;
    const Eigen::Matrix4d innovation = h_ * p_ * h_.transpose() + r_;
    return residual.dot(innovation.ldlt().solve(residual));
  }
  double positionNis(const Box& box) const {
    Eigen::Matrix<double, 2, 8> h_position;
    h_position.setZero();
    h_position(0, 0) = h_position(1, 1) = 1.0;
    const Eigen::Vector2d residual(box.cx() - x_(0), box.cy() - x_(1));
    const Eigen::Matrix2d innovation =
        h_position * p_ * h_position.transpose() + r_.topLeftCorner<2, 2>();
    return residual.dot(innovation.ldlt().solve(residual));
  }
  std::array<float, 4> covarianceDiagonal() const {
    return {static_cast<float>(std::max(0.0, p_(0, 0))),
            static_cast<float>(std::max(0.0, p_(1, 1))),
            static_cast<float>(std::max(0.0, p_(2, 2))),
            static_cast<float>(std::max(0.0, p_(3, 3)))};
  }
  // Apply a delayed-measurement correction that was computed on a historical
  // snapshot and projected to the current state.  The correction is bounded
  // by the caller; covariance is inflated slightly because the projection is
  // an approximation rather than a full Rauch-Tung-Striebel replay.
  void applyProjectedPositionCorrection(const double dx, const double dy) {
    x_(0) += dx;
    x_(1) += dy;
    p_(0, 0) += std::max(0.0, dx * dx) * 0.05;
    p_(1, 1) += std::max(0.0, dy * dy) * 0.05;
  }
  void applyImageWarp(const double a, const double b, const double tx,
                      const double c, const double d, const double ty) {
    const double old_cx = x_(0), old_cy = x_(1);
    const double old_vx = x_(4), old_vy = x_(5);
    const double width_scale = clamp(std::hypot(a, b), 0.5, 2.0);
    const double height_scale = clamp(std::hypot(c, d), 0.5, 2.0);
    x_(0) = a * old_cx + b * old_cy + tx;
    x_(1) = c * old_cx + d * old_cy + ty;
    x_(4) = a * old_vx + b * old_vy;
    x_(5) = c * old_vx + d * old_vy;
    x_(2) += std::log(width_scale);
    x_(3) += std::log(height_scale);
    Eigen::Matrix<double, 8, 8> transform =
        Eigen::Matrix<double, 8, 8>::Identity();
    transform(0, 0) = a;
    transform(0, 1) = b;
    transform(1, 0) = c;
    transform(1, 1) = d;
    transform(4, 4) = a;
    transform(4, 5) = b;
    transform(5, 4) = c;
    transform(5, 5) = d;
    p_ = transform * p_ * transform.transpose();
    p_(0, 0) += 0.25;
    p_(1, 1) += 0.25;
  }
  void setMeasurementNoiseScale(const double scale) {
    r_.setIdentity();
    r_.diagonal() << 1.0, 1.0, 0.02, 0.02;
    r_ *= std::max(1e-3, scale);
  }
  Box box() const {
    const double width = std::exp(clamp(x_(2), std::log(1.0), std::log(4096.0)));
    const double height = std::exp(clamp(x_(3), std::log(1.0), std::log(4096.0)));
    return {static_cast<int>(x_(0) - width / 2.0),
            static_cast<int>(x_(1) - height / 2.0),
            static_cast<int>(x_(0) + width / 2.0),
            static_cast<int>(x_(1) + height / 2.0)};
  }

 private:
  static Eigen::Matrix<double, 4, 1> measurement(const Box& box) {
    Eigen::Matrix<double, 4, 1> result;
    result << box.cx(), box.cy(), std::log(std::max(box.width(), 1.0)),
        std::log(std::max(box.height(), 1.0));
    return result;
  }
  Eigen::Matrix<double, 8, 1> x_;
  Eigen::Matrix<double, 8, 8> f_, q_, p_;
  Eigen::Matrix<double, 4, 8> h_;
  Eigen::Matrix4d r_;
  double process_noise_{1.0};
};

struct Result {
  bool active{false}, predicted{false};
  Box box;
  double confidence{0.0}, vx{0.0}, vy{0.0}, ax{0.0}, ay{0.0};
  double ex{0.0}, ey{0.0}, size_error{0.0}, quality{0.0};
  int missing{0};
  AssociationMethod association{AssociationMethod::kNone};
  bool reidentification_match{false};
  bool control_measurement_ready{false};
};

struct FilterSnapshot {
  ros::Time stamp;
  KalmanBox kalman;
  Box association_box;
  Box last_observation_box;
  int missing{0};
  double confidence{0.0};
  double vx{0.0}, vy{0.0}, ax{0.0}, ay{0.0};
  bool tracking{false};
  bool have_motion{false};
  bool have_confirmed_measurement{false};
  bool range_valid{false};
  std::array<double, 3> relative_position_body{0.0, 0.0, 0.0};
  std::array<double, 3> relative_velocity_body{0.0, 0.0, 0.0};
  std::array<double, 9> position_covariance{0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                             0.0, 0.0, 0.0};
  std::array<double, 9> velocity_covariance{0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                             0.0, 0.0, 0.0};
};

class TrackerNode {
 public:
  TrackerNode() : private_nh_("~"), tf_listener_(tf_buffer_) {
    private_nh_.param("frame_width", width_, 640);
    private_nh_.param("frame_height", height_, 480);
    private_nh_.param("fov_horizontal", fov_horizontal_deg_, 60.0);
    private_nh_.param("fov_vertical", fov_vertical_deg_, 45.0);
    private_nh_.param("mount_offset_yaw", mount_yaw_deg_, 0.0);
    private_nh_.param("mount_offset_pitch", mount_pitch_deg_, 0.0);
    private_nh_.param("publish_rate", rate_, 30.0);
    private_nh_.param("tracker_name", name_, std::string("TrackerCore"));
    private_nh_.param("image_source", image_source_, std::string("eo"));
    private_nh_.param("allow_image_source_switch", allow_image_source_switch_, false);
    private_nh_.param("require_image_source_metadata",
                      require_image_source_metadata_, false);
    private_nh_.param("direct_observation_hold_time", hold_time_, 0.15);
    private_nh_.param("auto_start_on_detection", auto_start_, true);
    private_nh_.param("enable_kalman_filter", kalman_enabled_, true);
    private_nh_.param("enable_motion_predictor", motion_enabled_, true);
    private_nh_.param("enable_prediction_buffer", prediction_enabled_, true);
    private_nh_.param("id_loss_tolerance_frames", max_missing_, 5);
    private_nh_.param("target_size_ratio", target_ratio_, 0.15);
    private_nh_.param("kalman_process_noise", process_noise_, 1.0);
    private_nh_.param("kalman_measurement_noise", measurement_noise_, 1.0);
    private_nh_.param("velocity_alpha", velocity_alpha_, 0.7);
    private_nh_.param("acceleration_alpha", acceleration_alpha_, 0.5);
    private_nh_.param("confidence_smoothing_alpha", confidence_alpha_, 0.8);
    private_nh_.param("track_confidence_decay_rate", confidence_decay_, 0.05);
    private_nh_.param("spatial_iou_threshold", iou_threshold_, 0.35);
    private_nh_.param("center_distance_threshold", distance_threshold_, 2.0);
    private_nh_.param("min_detection_confidence", min_confidence_, 0.0);
    private_nh_.param("innovation_gate_nis", innovation_gate_nis_, 16.0);
    private_nh_.param("association_gate_nis", association_gate_nis_, 9.21);
    private_nh_.param("max_size_change_ratio", max_size_change_ratio_, 2.5);
    private_nh_.param("max_detection_age_sec", max_detection_age_, 0.25);
    private_nh_.param("enable_appearance_reid", appearance_reid_enabled_, false);
    private_nh_.param("appearance_min_cosine", appearance_min_cosine_, 0.75);
    private_nh_.param("appearance_update_alpha", appearance_alpha_, 0.1);
    private_nh_.param("bytetrack_high_confidence", bytetrack_high_confidence_, 0.50);
    private_nh_.param("bytetrack_low_confidence", bytetrack_low_confidence_, 0.10);
    private_nh_.param("bytetrack_low_iou_threshold", bytetrack_low_iou_threshold_, 0.15);
    private_nh_.param("bytetrack_low_nis_scale", bytetrack_low_nis_scale_, 2.0);
    private_nh_.param("reacquisition_confirm_frames", reacquire_frames_, 2);
    private_nh_.param("prediction_fusion_max_offset_ratio",
                      fusion_max_offset_ratio_, 0.75);
    private_nh_.param("tracker_profile", tracker_profile_,
                      std::string("smart_botsort"));
    private_nh_.param("smart_tracker_backend", smart_backend_,
                      std::string("botsort"));
    private_nh_.param("tracking_strategy", tracking_strategy_,
                      std::string("hybrid"));
    private_nh_.param("reacquisition_mode", reacquisition_mode_,
                      std::string("balanced"));
    private_nh_.param("class_match_flexible", class_match_flexible_, true);
    private_nh_.param("search_expansion_rate", search_expansion_rate_, 0.20);
    private_nh_.param("search_expansion_cap", search_expansion_cap_, 2.0);
    private_nh_.param("appearance_distance_gate_factor",
                      appearance_distance_gate_, 3.0);
    private_nh_.param("oosm_history_duration_sec", oosm_history_duration_sec_, 0.5);
    private_nh_.param("oosm_max_future_sec", oosm_max_future_sec_, 0.02);
    private_nh_.param("oosm_max_snapshots", oosm_max_snapshots_, 40);
    private_nh_.param("enable_gmc", gmc_enabled_, false);
    private_nh_.param("gmc_image_topic", gmc_image_topic_,
                      std::string("/camera/image_raw"));
    private_nh_.param("gmc_max_features", gmc_max_features_, 300);
    private_nh_.param("gmc_min_inliers", gmc_min_inliers_, 12);
    private_nh_.param("gmc_ransac_reproj_threshold", gmc_ransac_threshold_, 3.0);
    private_nh_.param("gmc_max_image_width", gmc_max_image_width_, 320);
    private_nh_.param("gmc_max_age_sec", gmc_max_age_sec_, 0.15);
    private_nh_.param("gmc_detection_sync_tolerance_sec",
                      gmc_sync_tolerance_sec_, 0.05);
    private_nh_.param("gmc_require_detection_sync",
                      gmc_require_detection_sync_, false);
    private_nh_.param("use_camera_info", use_camera_info_, true);
    private_nh_.param("camera_info_topic", camera_info_topic_,
                      std::string("/camera/camera_info"));
    private_nh_.param("secondary_camera_info_topic", secondary_camera_info_topic_,
                      std::string(""));
    private_nh_.param("use_dynamic_camera_tf", use_dynamic_camera_tf_, true);
    private_nh_.param("body_frame", body_frame_, std::string("base_link"));
    private_nh_.param("tf_lookup_timeout_sec", tf_lookup_timeout_sec_, 0.02);
    private_nh_.param("gimbal_state_topic", gimbal_state_topic_,
                      std::string("/pod/gimbal/state"));
    private_nh_.param("gimbal_state_timeout_sec", gimbal_state_timeout_sec_, 0.15);
    private_nh_.param("require_gimbal_state_for_body_los",
                      require_gimbal_state_for_body_los_, false);
    private_nh_.param("selection_timestamp_tolerance_sec",
                      selection_timestamp_tolerance_sec_, 0.25);

    tracker::MultiTrackConfig multi_config;
    private_nh_.param("multi_track_confirmation_hits",
                      multi_config.confirmation_hits, 2);
    private_nh_.param("multi_track_occlusion_frames",
                      multi_config.occlusion_frames, 5);
    private_nh_.param("multi_track_removal_frames",
                      multi_config.removal_frames, 30);
    private_nh_.param("multi_track_maximum_tracks",
                      multi_config.maximum_tracks, 128);
    private_nh_.param("multi_track_maximum_embedding_dimension",
                      multi_config.maximum_embedding_dimension, 2048);
    maximum_embedding_dimension_ = multi_config.maximum_embedding_dimension;
    private_nh_.param("multi_track_new_confidence",
                      multi_config.minimum_new_track_confidence, 0.20);
    private_nh_.param("multi_track_update_confidence",
                      multi_config.minimum_update_confidence, 0.05);
    private_nh_.param("multi_track_iou_threshold",
                      multi_config.association_iou_threshold, 0.20);
    private_nh_.param("multi_track_center_distance",
                      multi_config.association_center_distance, 1.50);
    private_nh_.param("multi_track_appearance_cosine",
                      multi_config.appearance_minimum_cosine, 0.78);
    multi_config.process_noise = process_noise_;
    multi_config.measurement_noise = measurement_noise_;
    multi_track_manager_.reset(new tracker::MultiTrackManager(multi_config));
    output_pub_ =
        private_nh_.advertise<tracker::TrackingOutput>("tracking_output", 10);
    error_pub_ =
        private_nh_.advertise<tracker::NormalizedError>("normalized_error", 10);
    tracks_pub_ =
        private_nh_.advertise<tracker::TrackStateArray>("track_states", 10);
    input_sub_ = private_nh_.subscribe(
        "external_input", 10, &TrackerNode::inputCallback, this);
    candidates_sub_ = private_nh_.subscribe(
      "detection_candidates", 10, &TrackerNode::candidatesCallback, this);
    gimbal_state_sub_ = nh_.subscribe(
        gimbal_state_topic_, 10, &TrackerNode::gimbalStateCallback, this,
        ros::TransportHints().tcpNoDelay());
    select_track_srv_ = private_nh_.advertiseService(
        "select_track", &TrackerNode::selectTrack, this);
    if (use_camera_info_) {
      camera_info_subs_.push_back(nh_.subscribe(
          camera_info_topic_, 2, &TrackerNode::cameraInfoCallback, this));
      if (!secondary_camera_info_topic_.empty() &&
          secondary_camera_info_topic_ != camera_info_topic_) {
        camera_info_subs_.push_back(nh_.subscribe(
            secondary_camera_info_topic_, 2,
            &TrackerNode::cameraInfoCallback, this));
      }
    }
    if (gmc_enabled_) {
      image_sub_ = nh_.subscribe(gmc_image_topic_, 1,
                                 &TrackerNode::imageCallback, this,
                                 ros::TransportHints().tcpNoDelay());
    }
    timer_ = nh_.createTimer(ros::Duration(1.0 / std::max(1.0, rate_)),
                             &TrackerNode::tick, this);
    configureSmartProfile();
    gmc_max_features_ = std::max(50, std::min(2000, gmc_max_features_));
    gmc_min_inliers_ = std::max(4, std::min(gmc_max_features_, gmc_min_inliers_));
    gmc_max_image_width_ = std::max(80, std::min(1920, gmc_max_image_width_));
    gmc_ransac_threshold_ = clamp(gmc_ransac_threshold_, 0.5, 20.0);
    gmc_max_age_sec_ = clamp(gmc_max_age_sec_, 0.02, 2.0);
    if (gmc_enabled_) {
      gmc_orb_ = cv::ORB::create(gmc_max_features_);
    }
    updater_.setHardwareID("vision_tracker");
    updater_.add("selected_target_tracker", this, &TrackerNode::diagnostics);
    ROS_INFO("[TrackerNode] C++ SmartTracker started: %dx%d at %.1f Hz, profile=%s backend=%s strategy=%s reacquisition=%s",
             width_, height_, rate_, tracker_profile_.c_str(),
             smart_backend_.c_str(), tracking_strategy_.c_str(),
             reacquisition_mode_.c_str());
  }

 private:
  void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& message) {
    if (message->width == 0 || message->height == 0 ||
        !std::isfinite(message->K[0]) || !std::isfinite(message->K[4]) ||
        message->K[0] <= 0.0 || message->K[4] <= 0.0) {
      ++invalid_camera_info_count_;
      ROS_WARN_THROTTLE(2.0, "[TrackerNode] Rejected invalid CameraInfo");
      return;
    }
    CameraCalibration calibration;
    calibration.width = static_cast<int>(message->width);
    calibration.height = static_cast<int>(message->height);
    calibration.fx = message->K[0];
    calibration.fy = message->K[4];
    calibration.cx = message->K[2];
    calibration.cy = message->K[5];
    calibration.frame_id = message->header.frame_id;
    calibration.stamp = message->header.stamp;
    calibration.valid = true;
    std::lock_guard<std::mutex> lock(mutex_);
    const std::string key = calibration.frame_id.empty()
                                ? std::string("__default__")
                                : calibration.frame_id;
    camera_calibrations_[key] = calibration;
    default_calibration_key_ = key;
    have_camera_info_ = true;
  }

  // Keep the hardware attitude as a first-class input.  TF remains the
  // transform transport, while this subscription makes loss/limit/fault state
  // visible to Tracker's control-measurement gate instead of silently using a
  // stale optical transform.
  void gimbalStateCallback(const pod_msgs::GimbalState::ConstPtr& message) {
    std::lock_guard<std::mutex> lock(mutex_);
    gimbal_state_ = *message;
    gimbal_state_time_ = ros::WallTime::now();
    have_gimbal_state_ = true;
  }

  bool angularError(const Box& box, double* yaw, double* pitch) {
    if (yaw == nullptr || pitch == nullptr) return false;
    CameraCalibration calibration;
    std::string source_frame;
    ros::Time capture_stamp;
    bool gimbal_ready = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      source_frame = last_capture_frame_id_;
      capture_stamp = last_capture_stamp_;
      auto calibration_it = camera_calibrations_.find(source_frame);
      if (calibration_it == camera_calibrations_.end()) {
        calibration_it = camera_calibrations_.find(default_calibration_key_);
      }
      if (calibration_it != camera_calibrations_.end()) {
        calibration = calibration_it->second;
      }
      gimbal_ready = have_gimbal_state_ &&
          !gimbal_state_time_.isZero() &&
          (ros::WallTime::now() - gimbal_state_time_).toSec() <=
              std::max(0.02, gimbal_state_timeout_sec_) &&
          gimbal_state_.connected && gimbal_state_.attitude_valid &&
          gimbal_state_.stabilized && !gimbal_state_.limit_active &&
          gimbal_state_.fault_code.empty();
    }
    if (require_gimbal_state_for_body_los_ && !gimbal_ready) {
      last_tf_success_ = false;
      ROS_WARN_THROTTLE(2.0,
                        "[TrackerNode] Body LOS withheld: direct GimbalState is unavailable or unsafe");
      return false;
    }
    if (!use_camera_info_ || !calibration.valid) {
      const double error_x =
          clamp((box.cx() - width_ / 2.0) / (width_ / 2.0), -1.0, 1.0);
      const double error_y =
          clamp((box.cy() - height_ / 2.0) / (height_ / 2.0), -1.0, 1.0);
      *yaw = std::atan(error_x * std::tan(
          0.5 * fov_horizontal_deg_ * kPi / 180.0)) +
          mount_yaw_deg_ * kPi / 180.0;
      *pitch = std::atan(error_y * std::tan(
          0.5 * fov_vertical_deg_ * kPi / 180.0)) +
          mount_pitch_deg_ * kPi / 180.0;
      return true;
    }

    const double scale_x = static_cast<double>(calibration.width) /
                           std::max(1, width_);
    const double scale_y = static_cast<double>(calibration.height) /
                           std::max(1, height_);
    const double camera_x =
        (box.cx() * scale_x - calibration.cx) / calibration.fx;
    const double camera_y =
        (box.cy() * scale_y - calibration.cy) / calibration.fy;
    if (use_dynamic_camera_tf_ && !calibration.frame_id.empty() &&
        !body_frame_.empty() && calibration.frame_id != body_frame_) {
      geometry_msgs::Vector3Stamped camera_ray;
      camera_ray.header.frame_id = calibration.frame_id;
      camera_ray.header.stamp = capture_stamp;
      camera_ray.vector.x = camera_x;
      camera_ray.vector.y = camera_y;
      camera_ray.vector.z = 1.0;
      try {
        const geometry_msgs::Vector3Stamped body_ray = tf_buffer_.transform(
            camera_ray, body_frame_, ros::Duration(tf_lookup_timeout_sec_));
        const double horizontal =
            std::hypot(body_ray.vector.x, body_ray.vector.y);
        // REP-103 base_link uses y-left/z-up; the follower contract uses
        // right/down positive angular error.
        *yaw = std::atan2(-body_ray.vector.y, body_ray.vector.x);
        *pitch = std::atan2(-body_ray.vector.z, std::max(1e-9, horizontal));
        last_tf_success_ = true;
        return std::isfinite(*yaw) && std::isfinite(*pitch);
      } catch (const tf2::TransformException& exception) {
        last_tf_success_ = false;
        ++tf_failure_count_;
        ROS_WARN_THROTTLE(2.0,
                          "[TrackerNode] Camera-to-body TF unavailable: %s",
                          exception.what());
      }
    }
    *yaw = std::atan(camera_x) + mount_yaw_deg_ * kPi / 180.0;
    *pitch = std::atan(camera_y) + mount_pitch_deg_ * kPi / 180.0;
    return true;
  }

  void diagnostics(diagnostic_updater::DiagnosticStatusWrapper& status) {
    std::lock_guard<std::mutex> lock(mutex_);
    const double input_age = last_input_wall_.isZero()
        ? std::numeric_limits<double>::infinity()
        : (ros::WallTime::now() - last_input_wall_).toSec();
    int level = diagnostic_msgs::DiagnosticStatus::OK;
    std::string summary = "Tracker ready";
    if (input_message_count_ == 0) {
      level = diagnostic_msgs::DiagnosticStatus::WARN;
      summary = "Waiting for detections";
    } else if (input_age > std::max(1.0, 4.0 / std::max(1.0, rate_))) {
      level = diagnostic_msgs::DiagnosticStatus::WARN;
      summary = "Detection stream is stale";
    }
    if (use_camera_info_ && !have_camera_info_) {
      level = std::max(level,
                       static_cast<int>(diagnostic_msgs::DiagnosticStatus::WARN));
      summary = "CameraInfo unavailable; static FOV fallback active";
    }
    status.summary(level, summary);
    status.add("tracking_active", tracking_);
    status.add("selected_target_id", target_id_);
    status.add("persistent_track_count", static_cast<int>(last_track_count_));
    status.add("input_messages", static_cast<long long>(input_message_count_));
    status.add("rejected_inputs", static_cast<long long>(rejected_input_count_));
    status.add("selection_count", static_cast<long long>(selection_count_));
    status.add("image_source", image_source_);
    status.add("image_source_switches",
               static_cast<long long>(image_source_switch_count_));
    status.add("allow_image_source_switch", allow_image_source_switch_);
    status.add("input_age_sec", std::isfinite(input_age) ? input_age : -1.0);
    status.add("fusion_status", last_fusion_status_);
    status.add("camera_info_available", have_camera_info_);
    status.add("camera_calibration_count",
               static_cast<int>(camera_calibrations_.size()));
    status.add("dynamic_tf_enabled", use_dynamic_camera_tf_);
    const bool gimbal_fresh = have_gimbal_state_ && !gimbal_state_time_.isZero() &&
        (ros::WallTime::now() - gimbal_state_time_).toSec() <=
            std::max(0.02, gimbal_state_timeout_sec_);
    status.add("gimbal_state_topic", gimbal_state_topic_);
    status.add("gimbal_state_fresh", gimbal_fresh);
    status.add("require_gimbal_state_for_body_los", require_gimbal_state_for_body_los_);
    status.add("last_tf_success", last_tf_success_);
    status.add("tf_failures", static_cast<long long>(tf_failure_count_));
    status.add("gmc_enabled", gmc_enabled_);
    status.add("gmc_applied", gmc_applied_count_);
    status.add("gmc_rejected", gmc_rejected_count_);
    status.add("gmc_sync_rejected", gmc_sync_rejected_count_);
    if (multi_track_manager_) {
      const tracker::MultiTrackStatistics multi =
          multi_track_manager_->statistics();
      status.add("multi_created_tracks",
                 static_cast<long long>(multi.created_tracks));
      status.add("multi_removed_tracks",
                 static_cast<long long>(multi.removed_tracks));
      status.add("multi_rejected_detections",
                 static_cast<long long>(multi.rejected_detections));
    }
  }

  void configureSmartProfile() {
    tracker_profile_ = lower(tracker_profile_);
    smart_backend_ = lower(smart_backend_);
    tracking_strategy_ = lower(tracking_strategy_);
    reacquisition_mode_ = lower(reacquisition_mode_);
    if (tracker_profile_ == "smart_botsort") {
      smart_backend_ = "botsort";
    } else if (tracker_profile_ == "smart_bytetrack") {
      smart_backend_ = "bytetrack";
    } else if (tracker_profile_ == "smart_reid_hybrid") {
      smart_backend_ = "custom_reid";
      appearance_reid_enabled_ = true;
    } else if (tracker_profile_ != "classic_hybrid") {
      ROS_WARN("[TrackerNode] Unknown tracker_profile '%s'; retaining requested backend '%s'",
               tracker_profile_.c_str(), smart_backend_.c_str());
    }
    if (smart_backend_ == "custom_reid") appearance_reid_enabled_ = true;
    if (tracking_strategy_ != "id_only" && tracking_strategy_ != "spatial_only" &&
        tracking_strategy_ != "distance_only" && tracking_strategy_ != "hybrid") {
      ROS_WARN("[TrackerNode] Unknown tracking_strategy '%s'; using hybrid",
               tracking_strategy_.c_str());
      tracking_strategy_ = "hybrid";
    }
    if (reacquisition_mode_ != "aggressive" &&
        reacquisition_mode_ != "balanced" && reacquisition_mode_ != "strict") {
      ROS_WARN("[TrackerNode] Unknown reacquisition_mode '%s'; using balanced",
               reacquisition_mode_.c_str());
      reacquisition_mode_ = "balanced";
    }
    search_expansion_rate_ = std::max(0.0, search_expansion_rate_);
    search_expansion_cap_ = std::max(1.0, search_expansion_cap_);
    appearance_distance_gate_ = std::max(0.1, appearance_distance_gate_);
    oosm_history_duration_sec_ = clamp(oosm_history_duration_sec_, 0.05, 2.0);
    oosm_max_future_sec_ = clamp(oosm_max_future_sec_, 0.0, 0.25);
    oosm_max_snapshots_ = std::max(5, std::min(200, oosm_max_snapshots_));
    bytetrack_high_confidence_ = clamp(bytetrack_high_confidence_, 0.05, 0.99);
    bytetrack_low_confidence_ = clamp(bytetrack_low_confidence_, 0.0,
                                      bytetrack_high_confidence_);
    bytetrack_low_iou_threshold_ = clamp(bytetrack_low_iou_threshold_, 0.0, 0.95);
    bytetrack_low_nis_scale_ = clamp(bytetrack_low_nis_scale_, 1.0, 5.0);
  }
  bool convert(const tracker::ExternalInput& message, Box& box) const {
    const std::string source = lower(message.source);
    if ((source == "bounding_box" || source == "external_detector") &&
        message.has_bbox) {
      box = {static_cast<int>(clamp(message.bbox[0], 0, width_)),
             static_cast<int>(clamp(message.bbox[1], 0, height_)),
             static_cast<int>(clamp(message.bbox[2], 0, width_)),
             static_cast<int>(clamp(message.bbox[3], 0, height_))};
    } else if ((source == "bounding_box" ||
                source == "external_detector") &&
               message.has_normalized_bbox) {
      const double w = clamp(message.normalized_bbox[2], 0.0, 1.0);
      const double h = clamp(message.normalized_bbox[3], 0.0, 1.0);
      if (w <= 0.0 || h <= 0.0) return false;
      const double cx =
          clamp(message.normalized_bbox[0], w / 2.0, 1.0 - w / 2.0);
      const double cy =
          clamp(message.normalized_bbox[1], h / 2.0, 1.0 - h / 2.0);
      box.x1 = static_cast<int>((cx - w / 2.0) * width_);
      box.y1 = static_cast<int>((cy - h / 2.0) * height_);
      box.x2 = box.x1 + static_cast<int>(w * width_);
      box.y2 = box.y1 + static_cast<int>(h * height_);
    } else if (source == "feature_point" && message.has_feature_point) {
      const double cx = clamp(message.feature_point[0], 0.0, 1.0);
      const double cy = clamp(message.feature_point[1], 0.0, 1.0);
      const int w = static_cast<int>(0.05 * width_);
      const int h = static_cast<int>(0.05 * height_);
      box = {static_cast<int>(cx * width_) - w / 2,
             static_cast<int>(cy * height_) - h / 2,
             static_cast<int>(cx * width_) + w / 2,
             static_cast<int>(cy * height_) + h / 2};
    } else if (source == "roi" && message.has_roi) {
      const double x = clamp(message.roi[0], 0.0, 1.0);
      const double y = clamp(message.roi[1], 0.0, 1.0);
      const double w = clamp(message.roi[2], 0.0, 1.0 - x);
      const double h = clamp(message.roi[3], 0.0, 1.0 - y);
      if (w <= 0.0 || h <= 0.0) return false;
      box = {static_cast<int>(x * width_), static_cast<int>(y * height_),
             static_cast<int>((x + w) * width_),
             static_cast<int>((y + h) * height_)};
    } else {
      return false;
    }
    return box.x2 > box.x1 && box.y2 > box.y1;
  }

  Candidate makeCandidate(const tracker::DetectionCandidate& raw,
                          const Box& box) const {
    Candidate candidate;
    candidate.box = box;
    candidate.track_id = raw.track_id;
    candidate.class_id = raw.class_id;
    candidate.stable_id = raw.track_id_is_stable;
    candidate.confidence = clamp(raw.confidence, 0.0, 1.0);
    if (raw.appearance_embedding.size() <=
        static_cast<std::size_t>(std::max(0, maximum_embedding_dimension_)) &&
        std::all_of(raw.appearance_embedding.begin(),
                    raw.appearance_embedding.end(),
                    [](const float value) { return std::isfinite(value); })) {
      candidate.appearance = raw.appearance_embedding;
    }
    candidate.has_relative_position_body = raw.has_relative_position_body;
    candidate.has_relative_velocity_body = raw.has_relative_velocity_body;
    candidate.range_valid = raw.range_valid && raw.has_relative_position_body;
    for (std::size_t i = 0; i < 3; ++i) {
      candidate.relative_position_body[i] = raw.relative_position_body[i];
      candidate.relative_velocity_body[i] = raw.relative_velocity_body[i];
      if (!std::isfinite(candidate.relative_position_body[i]) ||
          !std::isfinite(candidate.relative_velocity_body[i])) {
        candidate.range_valid = false;
      }
    }
    for (std::size_t i = 0; i < 9; ++i) {
      candidate.position_covariance[i] = raw.position_covariance[i];
      candidate.velocity_covariance[i] = raw.velocity_covariance[i];
      if (!std::isfinite(candidate.position_covariance[i]) ||
          !std::isfinite(candidate.velocity_covariance[i])) {
        candidate.range_valid = false;
      }
    }
    return candidate;
  }

  bool convert(const tracker::DetectionCandidate& message, Box& box) const {
    if (message.has_bbox) {
      box = {static_cast<int>(clamp(message.bbox[0], 0, width_)),
             static_cast<int>(clamp(message.bbox[1], 0, height_)),
             static_cast<int>(clamp(message.bbox[2], 0, width_)),
             static_cast<int>(clamp(message.bbox[3], 0, height_))};
    } else if (message.has_normalized_bbox) {
      const double w = clamp(message.normalized_bbox[2], 0.0, 1.0);
      const double h = clamp(message.normalized_bbox[3], 0.0, 1.0);
      if (w <= 0.0 || h <= 0.0) return false;
      const double cx = clamp(message.normalized_bbox[0], w / 2.0, 1.0 - w / 2.0);
      const double cy = clamp(message.normalized_bbox[1], h / 2.0, 1.0 - h / 2.0);
      box = {static_cast<int>((cx - w / 2.0) * width_),
             static_cast<int>((cy - h / 2.0) * height_),
             static_cast<int>((cx + w / 2.0) * width_),
             static_cast<int>((cy + h / 2.0) * height_)};
    } else {
      return false;
    }
    return box.x2 > box.x1 && box.y2 > box.y1;
  }

  void inputCallback(const tracker::ExternalInput::ConstPtr& message) {
    const std::string command = lower(message->command);
    std::lock_guard<std::mutex> lock(mutex_);
    ++input_message_count_;
    last_input_wall_ = ros::WallTime::now();
    last_capture_frame_id_ = message->header.frame_id;
    if (command == "stop_track" || command == "reset") {
      reset();
      if (command == "reset" && multi_track_manager_) {
        multi_track_manager_->reset();
      }
      return;
    }
    if (!acceptTimestamp(message->header.stamp)) {
      ++rejected_input_count_;
      return;
    }
    Box box;
    if (!convert(*message, box)) {
      ++rejected_input_count_;
      ROS_WARN_THROTTLE(5.0, "[TrackerNode] Rejected external input");
      return;
    }
    const double confidence = clamp(message->confidence, 0.0, 1.0);
    if (command == "start_track" || (auto_start_ && !tracking_)) {
      if (isByteTrackProfile() && confidence < bytetrack_high_confidence_) {
        last_fusion_status_ = "REJECT_LOW_CONF_START";
        return;
      }
      startTracking({box, -1, message->class_id, false, confidence, {}});
    }
    if (!tracking_) return;
    pending_candidates_ = {{box, -1, message->class_id, false, confidence, {}}};
    pending_ = true;
    pending_detection_wall_ = ros::WallTime::now();
    pending_stamp_ = message->header.stamp;
  }

  void startTracking(const Candidate& candidate) {
    tracking_ = true;
    class_id_ = candidate.class_id >= 0 ? candidate.class_id : 0;
    target_id_ = candidate.stable_id ? candidate.track_id : -1;
    selected_track_id_ = candidate.track_id;
    selected_id_stable_ = candidate.stable_id;
    confidence_ = candidate.confidence;
    missing_ = 0;
    tentative_count_ = 0;
    kalman_.initialize(candidate.box, process_noise_, measurement_noise_);
    last_observation_box_ = candidate.box;
    association_box_ = candidate.box;
    history_.clear();
    filter_history_.clear();
    have_motion_ = false;
    have_confirmed_measurement_ = false;
    appearance_reference_ = candidate.appearance;
    last_filter_time_ = ros::WallTime::now();
    last_detection_wall_ = last_filter_time_;
    last_filter_ros_stamp_ = ros::Time::now();
    last_capture_stamp_ = ros::Time();
    last_inference_latency_ms_ = 0.0;
    last_control_detection_wall_ = ros::WallTime();
    range_valid_ = candidate.range_valid;
    relative_position_body_ = candidate.relative_position_body;
    relative_velocity_body_ = candidate.relative_velocity_body;
    position_covariance_ = candidate.position_covariance;
    velocity_covariance_ = candidate.velocity_covariance;
    last_fusion_status_ = "NONE";
    if (multi_track_manager_ && candidate.stable_id) {
      multi_track_manager_->setSelectedTrackId(candidate.track_id);
    }
  }

  void candidatesCallback(const tracker::DetectionArray::ConstPtr& message) {
    const std::string command = lower(message->command);
    std::lock_guard<std::mutex> lock(mutex_);
    ++input_message_count_;
    last_input_wall_ = ros::WallTime::now();
    last_capture_frame_id_ = message->header.frame_id;
    if (command == "stop_track" || command == "reset") {
      reset();
      if (command == "reset" && multi_track_manager_) {
        multi_track_manager_->reset();
      }
      return;
    }
    if (!acceptTimestamp(message->header.stamp)) {
      ++rejected_input_count_;
      return;
    }
    const std::string incoming_source = lower(message->image_source);
    if (require_image_source_metadata_ && incoming_source.empty()) {
      ++rejected_input_count_;
      last_fusion_status_ = "REJECT_MISSING_IMAGE_SOURCE";
      ROS_WARN_THROTTLE(2.0,
                        "[TrackerNode] Product mode requires DetectionArray.image_source");
      return;
    }
    if (!incoming_source.empty() && incoming_source != image_source_) {
      if (!allow_image_source_switch_) {
        ++rejected_input_count_;
        last_fusion_status_ = "REJECT_IMAGE_SOURCE_SWITCH";
        ROS_WARN_THROTTLE(2.0,
                          "[TrackerNode] Rejected image source switch %s -> %s",
                          image_source_.c_str(), incoming_source.c_str());
        return;
      }
      // EO/IR changes use independent image geometry and often independent
      // detector identities.  A switch deliberately clears the active track
      // rather than silently carrying a potentially wrong identity across
      // spectra.  The mission manager must explicitly reselect afterwards.
      reset();
      if (multi_track_manager_) multi_track_manager_->reset();
      image_source_ = incoming_source;
      ++image_source_switch_count_;
      last_fusion_status_ = "IMAGE_SOURCE_SWITCH_RESET";
    }
    tracker::ManagedDetectionFrame managed;
    if (multi_track_manager_) {
      managed = multi_track_manager_->update(*message, width_, height_);
      managed.tracks.image_source = image_source_;
      last_track_count_ = managed.tracks.tracks.size();
    } else {
      managed.candidates = *message;
      managed.tracks.header = message->header;
      managed.tracks.image_source = message->header.frame_id;
    }
    std::vector<Candidate> candidates;
    candidates.reserve(managed.candidates.candidates.size());
    for (const auto& raw : managed.candidates.candidates) {
      Box box;
      if (!convert(raw, box)) {
        ++rejected_input_count_;
        continue;
      }
      candidates.push_back(makeCandidate(raw, box));
    }
    if (!candidates.empty() &&
        (command == "start_track" || (auto_start_ && !tracking_))) {
      Candidate selected;
      if (!initialCandidate(candidates, selected)) {
        last_fusion_status_ = "REJECT_LOW_CONF_START";
      } else {
        startTracking(selected);
      }
    }
    if (tracking_ && !candidates.empty()) {
      pending_candidates_ = std::move(candidates);
      pending_ = true;
      pending_detection_wall_ = ros::WallTime::now();
      pending_stamp_ = message->header.stamp;
    }
    if (multi_track_manager_) {
      multi_track_manager_->setSelectedTrackId(target_id_);
      for (auto& track : managed.tracks.tracks) {
        track.selected = tracking_ && track.track_id == target_id_;
      }
    }
    tracks_pub_.publish(managed.tracks);
  }

  bool selectTrack(tracker::SelectTrack::Request& request,
                   tracker::SelectTrack::Response& response) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!request.start_tracking) {
      reset();
      response.success = true;
      response.message = "Selected-target tracking stopped";
      response.selected_target_id = -1;
      return true;
    }

    Candidate selected;
    std::string selected_frame_id;
    ros::Time selected_capture_stamp = request.capture_timestamp;
    if (request.use_normalized_roi) {
      tracker::DetectionCandidate roi;
      roi.has_normalized_bbox = true;
      roi.normalized_bbox = {
          request.normalized_roi[0] + 0.5F * request.normalized_roi[2],
          request.normalized_roi[1] + 0.5F * request.normalized_roi[3],
          request.normalized_roi[2], request.normalized_roi[3]};
      roi.confidence = 1.0F;
      roi.class_id = -1;
      Box box;
      if (!convert(roi, box)) {
        response.success = false;
        response.message = "Normalized ROI is invalid";
        response.selected_target_id = -1;
        return true;
      }
      selected = makeCandidate(roi, box);
      // image_source is a logical EO/IR name, not necessarily a TF frame.
      // Keep the latest validated detector frame for calibration lookup.
      selected_frame_id = last_capture_frame_id_;
    } else {
      tracker::DetectionCandidate candidate;
      std_msgs::Header candidate_header;
      if (!multi_track_manager_ ||
          !multi_track_manager_->latestCandidate(
              request.target_id, &candidate, &candidate_header)) {
        response.success = false;
        response.message = "Target is absent or currently prediction-only";
        response.selected_target_id = -1;
        return true;
      }
      if (!request.image_source.empty() && !image_source_.empty() &&
          request.image_source != image_source_) {
        response.success = false;
        response.message = "Target image source does not match request";
        response.selected_target_id = -1;
        return true;
      }
      if (!request.capture_timestamp.isZero() &&
          !candidate_header.stamp.isZero() &&
          std::abs((candidate_header.stamp - request.capture_timestamp).toSec()) >
              selection_timestamp_tolerance_sec_) {
        response.success = false;
        response.message = "Target observation is outside selection time tolerance";
        response.selected_target_id = -1;
        return true;
      }
      Box box;
      if (!convert(candidate, box)) {
        response.success = false;
        response.message = "Selected target has invalid geometry";
        response.selected_target_id = -1;
        return true;
      }
      selected = makeCandidate(candidate, box);
      selected_frame_id = candidate_header.frame_id;
      selected_capture_stamp = candidate_header.stamp;
    }
    startTracking(selected);
    // startTracking deliberately clears stale capture metadata. Restore only
    // the timestamp/frame that was validated for this explicit selection.
    last_capture_frame_id_ = selected_frame_id;
    last_capture_stamp_ = selected_capture_stamp;
    ++selection_count_;
    response.success = true;
    response.message = "Target selected";
    response.selected_target_id = target_id_;
    return true;
  }

  static Box warpBox(const Box& box, const double a, const double b,
                     const double tx, const double c, const double d,
                     const double ty) {
    const std::array<std::pair<double, double>, 4> corners{{
        {static_cast<double>(box.x1), static_cast<double>(box.y1)},
        {static_cast<double>(box.x2), static_cast<double>(box.y1)},
        {static_cast<double>(box.x2), static_cast<double>(box.y2)},
        {static_cast<double>(box.x1), static_cast<double>(box.y2)}}};
    double x1 = std::numeric_limits<double>::infinity();
    double y1 = std::numeric_limits<double>::infinity();
    double x2 = -std::numeric_limits<double>::infinity();
    double y2 = -std::numeric_limits<double>::infinity();
    for (const auto& corner : corners) {
      const double x = a * corner.first + b * corner.second + tx;
      const double y = c * corner.first + d * corner.second + ty;
      x1 = std::min(x1, x); y1 = std::min(y1, y);
      x2 = std::max(x2, x); y2 = std::max(y2, y);
    }
    return {static_cast<int>(std::lround(x1)),
            static_cast<int>(std::lround(y1)),
            static_cast<int>(std::lround(x2)),
            static_cast<int>(std::lround(y2))};
  }

  void imageCallback(const sensor_msgs::ImageConstPtr& message) {
    if (!gmc_enabled_) return;
    std::lock_guard<std::mutex> lock(mutex_);
    try {
      const ros::Time now = ros::Time::now();
      if (!message->header.stamp.isZero() && !now.isZero() &&
          now > message->header.stamp &&
          (now - message->header.stamp).toSec() > gmc_max_age_sec_) {
        ++gmc_sync_rejected_count_;
        return;
      }
      if (!message->header.frame_id.empty() &&
          !last_capture_frame_id_.empty() &&
          message->header.frame_id != last_capture_frame_id_) {
        ++gmc_sync_rejected_count_;
        return;
      }
      if (!message->header.stamp.isZero() && !gmc_last_stamp_.isZero() &&
          message->header.stamp <= gmc_last_stamp_) {
        ++gmc_sync_rejected_count_;
        return;
      }
      if (gmc_require_detection_sync_ && !message->header.stamp.isZero() &&
          !last_capture_stamp_.isZero() &&
          std::abs((message->header.stamp - last_capture_stamp_).toSec()) >
              gmc_sync_tolerance_sec_) {
        ++gmc_sync_rejected_count_;
        return;
      }
      if (!gmc_frame_id_.empty() && !message->header.frame_id.empty() &&
          gmc_frame_id_ != message->header.frame_id) {
        previous_gray_.release();
        previous_descriptors_.release();
        previous_keypoints_.clear();
      }
      gmc_frame_id_ = message->header.frame_id;
      gmc_last_stamp_ = message->header.stamp;
      const cv_bridge::CvImageConstPtr cv_ptr =
          cv_bridge::toCvShare(message, "mono8");
      if (!cv_ptr || cv_ptr->image.empty()) return;
      cv::Mat gray = cv_ptr->image;
      double resize_scale = 1.0;
      if (gray.cols > gmc_max_image_width_) {
        resize_scale = static_cast<double>(gmc_max_image_width_) /
                       std::max(1, gray.cols);
        cv::resize(gray, gray, cv::Size(), resize_scale, resize_scale,
                   cv::INTER_AREA);
      }
      std::vector<cv::KeyPoint> keypoints;
      cv::Mat descriptors;
      gmc_orb_->detectAndCompute(gray, cv::noArray(), keypoints, descriptors);
      if (previous_gray_.empty() || previous_descriptors_.empty() ||
          descriptors.empty() || keypoints.size() < 8) {
        previous_gray_ = gray.clone();
        previous_keypoints_ = std::move(keypoints);
        previous_descriptors_ = descriptors.clone();
        return;
      }
      cv::BFMatcher matcher(cv::NORM_HAMMING, false);
      std::vector<std::vector<cv::DMatch>> matches;
      matcher.knnMatch(previous_descriptors_, descriptors, matches, 2);
      std::vector<cv::Point2f> previous_points, current_points;
      for (const auto& pair : matches) {
        if (pair.size() == 2 && pair[0].distance < 0.75f * pair[1].distance) {
          previous_points.push_back(previous_keypoints_[pair[0].queryIdx].pt);
          current_points.push_back(keypoints[pair[0].trainIdx].pt);
        }
      }
      bool applied = false;
      int inlier_count = 0;
      if (previous_points.size() >= 8) {
        cv::Mat inliers;
        const cv::Mat affine = cv::estimateAffinePartial2D(
            previous_points, current_points, inliers, cv::RANSAC,
            gmc_ransac_threshold_, 2000, 0.99, 30);
        if (!affine.empty() && affine.rows == 2 && affine.cols == 3) {
          const double a = affine.at<double>(0, 0);
          const double b = affine.at<double>(0, 1);
          const double tx = affine.at<double>(0, 2) / resize_scale;
          const double c = affine.at<double>(1, 0);
          const double d = affine.at<double>(1, 1);
          const double ty = affine.at<double>(1, 2) / resize_scale;
          const double scale_x = std::hypot(a, b);
          const double scale_y = std::hypot(c, d);
          const double translation = std::hypot(tx, ty);
          inlier_count = inliers.empty() ? 0 : cv::countNonZero(inliers);
          const bool bounded = scale_x >= 0.5 && scale_x <= 2.0 &&
              scale_y >= 0.5 && scale_y <= 2.0 &&
              translation <= 0.75 * std::hypot(width_, height_) &&
              std::abs(std::atan2(c, a)) < 60.0 * kPi / 180.0;
          if (inlier_count >= gmc_min_inliers_ && bounded && tracking_) {
            kalman_.applyImageWarp(a, b, tx, c, d, ty);
            association_box_ = warpBox(association_box_, a, b, tx, c, d, ty);
            last_observation_box_ = warpBox(last_observation_box_, a, b, tx, c, d, ty);
            tentative_box_ = warpBox(tentative_box_, a, b, tx, c, d, ty);
            if (!history_.empty()) {
              history_.back().first = warpBox(history_.back().first,
                                               a, b, tx, c, d, ty);
            }
            applied = true;
          }
        }
      }
      gmc_valid_ = applied;
      gmc_inliers_ = inlier_count;
      if (applied) ++gmc_applied_count_;
      else if (!previous_points.empty()) ++gmc_rejected_count_;
      previous_gray_ = gray.clone();
      previous_keypoints_ = std::move(keypoints);
      previous_descriptors_ = descriptors.clone();
    } catch (const cv_bridge::Exception& exception) {
      ROS_WARN_THROTTLE(2.0, "[TrackerNode] GMC image conversion failed: %s",
                        exception.what());
    } catch (const cv::Exception& exception) {
      ROS_WARN_THROTTLE(2.0, "[TrackerNode] GMC OpenCV failed: %s",
                        exception.what());
    }
  }

  void reset() {
    tracking_ = pending_ = have_motion_ = have_direct_error_ = false;
    target_id_ = class_id_ = -1;
    confidence_ = vx_ = vy_ = ax_ = ay_ = 0.0;
    missing_ = 0;
    selected_track_id_ = -1;
    selected_id_stable_ = false;
    tentative_count_ = 0;
    history_.clear();
    filter_history_.clear();
    pending_candidates_.clear();
    appearance_reference_.clear();
    pending_stamp_ = last_accepted_stamp_ = ros::Time();
    last_filter_ros_stamp_ = ros::Time();
    last_capture_stamp_ = ros::Time();
    last_control_detection_wall_ = ros::WallTime();
    range_valid_ = false;
    relative_position_body_.fill(0.0);
    relative_velocity_body_.fill(0.0);
    position_covariance_.fill(0.0);
    velocity_covariance_.fill(0.0);
    last_fusion_status_ = "NONE";
    last_inference_latency_ms_ = 0.0;
    previous_gray_.release();
    previous_descriptors_.release();
    previous_keypoints_.clear();
    gmc_valid_ = false;
    if (multi_track_manager_) multi_track_manager_->setSelectedTrackId(-1);
  }

  bool acceptTimestamp(const ros::Time& stamp) {
    // Zero stamps are accepted for legacy publishers.  Stamped detector
    // streams may be out of order inside the OOSM history window.
    if (stamp.isZero()) return true;
    const ros::Time now = ros::Time::now();
    if (!now.isZero() && stamp > now &&
        (stamp - now).toSec() > oosm_max_future_sec_) {
      last_fusion_status_ = "REJECT_TOO_NEW";
      ROS_WARN_THROTTLE(1.0, "[TrackerNode] Rejected future detection");
      return false;
    }
    if (!now.isZero() && now > stamp &&
        (now - stamp).toSec() > oosm_history_duration_sec_) {
      last_fusion_status_ = "REJECT_TOO_OLD";
      ROS_WARN_THROTTLE(1.0, "[TrackerNode] Rejected detection older than OOSM window");
      return false;
    }
    if (!last_filter_ros_stamp_.isZero() && stamp < last_filter_ros_stamp_) {
      if (filter_history_.empty() || stamp < filter_history_.front().stamp) {
        last_fusion_status_ = "REJECT_TOO_OLD";
        ROS_WARN_THROTTLE(1.0, "[TrackerNode] Rejected detection outside snapshot history");
        return false;
      }
      ROS_DEBUG("[TrackerNode] Accepted delayed detection for OOSM replay");
    }
    return true;
  }

  static Box shiftBox(const Box& box, const double dx, const double dy) {
    return {static_cast<int>(std::lround(box.x1 + dx)),
            static_cast<int>(std::lround(box.y1 + dy)),
            static_cast<int>(std::lround(box.x2 + dx)),
            static_cast<int>(std::lround(box.y2 + dy))};
  }

  void recordFilterSnapshot(const ros::Time& stamp) {
    if (!tracking_ || stamp.isZero()) return;
    FilterSnapshot snapshot;
    snapshot.stamp = stamp;
    snapshot.kalman = kalman_;
    snapshot.association_box = association_box_;
    snapshot.last_observation_box = last_observation_box_;
    snapshot.missing = missing_;
    snapshot.confidence = confidence_;
    snapshot.vx = vx_;
    snapshot.vy = vy_;
    snapshot.ax = ax_;
    snapshot.ay = ay_;
    snapshot.tracking = tracking_;
    snapshot.have_motion = have_motion_;
    snapshot.have_confirmed_measurement = have_confirmed_measurement_;
    snapshot.range_valid = range_valid_;
    snapshot.relative_position_body = relative_position_body_;
    snapshot.relative_velocity_body = relative_velocity_body_;
    snapshot.position_covariance = position_covariance_;
    snapshot.velocity_covariance = velocity_covariance_;
    if (!filter_history_.empty() && stamp <= filter_history_.back().stamp) {
      return;
    }
    filter_history_.push_back(snapshot);
    while (filter_history_.size() > static_cast<std::size_t>(oosm_max_snapshots_) ||
           (!filter_history_.empty() &&
            (stamp - filter_history_.front().stamp).toSec() >
                oosm_history_duration_sec_)) {
      filter_history_.pop_front();
    }
    last_filter_ros_stamp_ = stamp;
  }

  const FilterSnapshot* findSnapshot(const ros::Time& stamp) const {
    const FilterSnapshot* selected = nullptr;
    for (const auto& snapshot : filter_history_) {
      if (snapshot.stamp <= stamp) selected = &snapshot;
      else break;
    }
    return selected;
  }

  double estimateLatencyMs(const ros::Time& capture_stamp) const {
    if (capture_stamp.isZero()) return 0.0;
    const ros::Time now = ros::Time::now();
    if (now.isZero() || now < capture_stamp) return 0.0;
    return clamp((now - capture_stamp).toSec() * 1000.0, 0.0, 10000.0);
  }

  bool applyOosmMeasurement() {
    // The first sample initializes the filter even if the detector timestamp
    // predates node startup; there is no historical state to replay yet.
    if (!have_confirmed_measurement_ || !pending_ || pending_stamp_.isZero() ||
        last_filter_ros_stamp_.isZero() ||
        pending_stamp_ >= last_filter_ros_stamp_) {
      return false;
    }
    const FilterSnapshot* snapshot = findSnapshot(pending_stamp_);
    if (snapshot == nullptr || !snapshot->tracking) {
      pending_ = false;
      last_fusion_status_ = "REJECT_TOO_OLD";
      return false;
    }

    // Reuse the normal SmartTracker association hierarchy at the historical
    // state.  Temporarily swapping only the state used by gating keeps the
    // policy identical for current and delayed observations.
    const KalmanBox current_kalman = kalman_;
    const Box current_association_box = association_box_;
    const int current_missing = missing_;
    kalman_ = snapshot->kalman;
    association_box_ = snapshot->association_box;
    missing_ = snapshot->missing;
    AssociationMethod method = AssociationMethod::kNone;
    const Candidate selected =
        selectCandidate(pending_candidates_, association_box_, &method);
    kalman_ = current_kalman;
    association_box_ = current_association_box;
    missing_ = current_missing;
    pending_ = false;

    if (selected.box.x2 <= selected.box.x1) {
      last_fusion_status_ = "REJECT_ASSOCIATION";
      return false;
    }
    if (snapshot->missing >= max_missing_ && reacquisition_mode_ == "strict" &&
        method != AssociationMethod::kDirectId &&
        method != AssociationMethod::kAppearance) {
      last_fusion_status_ = "REJECT_ASSOCIATION";
      return false;
    }

    KalmanBox historical = snapshot->kalman;
    historical.setMeasurementNoiseScale(
        measurement_noise_ / std::max(0.05, selected.confidence));
    const double nis = historical.innovationNis(selected.box);
    const bool appearance_match = method == AssociationMethod::kAppearance;
    const bool stable_id_match = method == AssociationMethod::kDirectId;
    if (!appearance_match && !stable_id_match && nis > innovation_gate_nis_) {
      last_fusion_status_ = "REJECT_NIS";
      ROS_WARN_THROTTLE(1.0,
                        "[TrackerNode] Rejected OOSM measurement by NIS %.2f > %.2f",
                        nis, innovation_gate_nis_);
      return false;
    }
    const Box historical_reference = snapshot->association_box;
    historical.update(selected.box);
    const Box corrected_historical = historical.box();
    const double max_offset = fusion_max_offset_ratio_ *
                              std::hypot(current_association_box.width(),
                                         current_association_box.height());
    const double dx = clamp(corrected_historical.cx() - historical_reference.cx(),
                            -max_offset, max_offset);
    const double dy = clamp(corrected_historical.cy() - historical_reference.cy(),
                            -max_offset, max_offset);
    kalman_.applyProjectedPositionCorrection(dx, dy);
    association_box_ = shiftBox(current_association_box, dx, dy);
    last_observation_box_ = selected.box;
    confidence_ = confidence_alpha_ * selected.confidence +
                  (1.0 - confidence_alpha_) * confidence_;
    selected_track_id_ = selected.track_id;
    selected_id_stable_ = selected.stable_id;
    target_id_ = selected.stable_id ? selected.track_id : -1;
    if (selected.class_id >= 0) class_id_ = selected.class_id;
    if (selected.range_valid) {
      range_valid_ = true;
      relative_position_body_ = selected.relative_position_body;
      relative_velocity_body_ = selected.relative_velocity_body;
      position_covariance_ = selected.position_covariance;
      velocity_covariance_ = selected.velocity_covariance;
    }
    missing_ = 0;
    have_confirmed_measurement_ = true;
    last_detection_wall_ = pending_detection_wall_.isZero()
                               ? ros::WallTime::now()
                               : pending_detection_wall_;
    last_capture_stamp_ = pending_stamp_;
    last_inference_latency_ms_ = estimateLatencyMs(pending_stamp_);
    if (method != AssociationMethod::kByteTrackLow) {
      last_control_detection_wall_ = last_detection_wall_;
    }
    if (last_accepted_stamp_.isZero() || pending_stamp_ > last_accepted_stamp_) {
      last_accepted_stamp_ = pending_stamp_;
    }
    if (appearance_reid_enabled_ && !selected.appearance.empty()) {
      if (appearance_reference_.size() != selected.appearance.size()) {
        appearance_reference_ = selected.appearance;
      } else {
        for (std::size_t i = 0; i < appearance_reference_.size(); ++i) {
          appearance_reference_[i] = static_cast<float>(
              (1.0 - appearance_alpha_) * appearance_reference_[i] +
              appearance_alpha_ * selected.appearance[i]);
        }
      }
    }
    last_fusion_status_ = "FUSED_OOSM";
    last_oosm_method_ = method;
    return true;
  }

  void updateMotion(const Box& box, const double time) {
    history_.push_back({box, time});
    while (history_.size() >
           static_cast<std::size_t>(std::max(3, max_missing_))) {
      history_.pop_front();
    }
    if (history_.size() < 2) return;
    const auto previous = history_[history_.size() - 2];
    const auto current = history_.back();
    const double dt = current.second - previous.second;
    if (dt <= 0.0) return;
    const double old_vx = vx_, old_vy = vy_;
    vx_ = velocity_alpha_ * (current.first.cx() - previous.first.cx()) / dt +
          (1.0 - velocity_alpha_) * vx_;
    vy_ = velocity_alpha_ * (current.first.cy() - previous.first.cy()) / dt +
          (1.0 - velocity_alpha_) * vy_;
    if (history_.size() >= 3) {
      ax_ = clamp(acceleration_alpha_ * (vx_ - old_vx) / dt +
                      (1.0 - acceleration_alpha_) * ax_,
                  -500.0, 500.0);
      ay_ = clamp(acceleration_alpha_ * (vy_ - old_vy) / dt +
                      (1.0 - acceleration_alpha_) * ay_,
                  -500.0, 500.0);
    }
    have_motion_ = true;
  }

  bool isByteTrackProfile() const {
    return tracker_profile_ == "smart_bytetrack" || smart_backend_ == "bytetrack";
  }

  bool initialCandidate(const std::vector<Candidate>& candidates,
                        Candidate& selected) const {
    if (!isByteTrackProfile()) {
      if (candidates.empty()) return false;
      selected = candidates.front();
      return true;
    }
    double best_confidence = bytetrack_high_confidence_;
    bool found = false;
    for (const Candidate& candidate : candidates) {
      if (candidate.confidence >= best_confidence &&
          candidate.box.x2 > candidate.box.x1) {
        if (!found || candidate.confidence > best_confidence) {
          selected = candidate;
          best_confidence = candidate.confidence;
          found = true;
        }
      }
    }
    return found;
  }

  Candidate selectByteTrackLowStage(const std::vector<Candidate>& candidates,
                                    const Box& reference,
                                    AssociationMethod* method) const {
    *method = AssociationMethod::kNone;
    Candidate selected;
    double best_iou = bytetrack_low_iou_threshold_;
    double best_nis = std::numeric_limits<double>::infinity();
    const double diagonal = std::max(1.0,
        std::hypot(reference.width(), reference.height()));
    for (const Candidate& candidate : candidates) {
      if (candidate.confidence < bytetrack_low_confidence_ ||
          candidate.confidence >= bytetrack_high_confidence_ ||
          candidate.box.x2 <= candidate.box.x1 ||
          (candidate.class_id >= 0 && class_id_ >= 0 &&
           candidate.class_id != class_id_ && !class_match_flexible_)) {
        continue;
      }
      const double width_ratio = candidate.box.width() /
          std::max(1.0, reference.width());
      const double height_ratio = candidate.box.height() /
          std::max(1.0, reference.height());
      if (width_ratio > max_size_change_ratio_ ||
          width_ratio < 1.0 / max_size_change_ratio_ ||
          height_ratio > max_size_change_ratio_ ||
          height_ratio < 1.0 / max_size_change_ratio_) continue;
      const bool id_match = selected_id_stable_ && candidate.stable_id &&
                            candidate.track_id == selected_track_id_;
      const double candidate_iou = iou(reference, candidate.box);
      const double candidate_nis = kalman_enabled_
          ? kalman_.positionNis(candidate.box)
          : 0.0;
      // Low-score recovery is intentionally restricted to the already active
      // track: ID, overlap, or a bounded Mahalanobis gate may recover it, but
      // a low-score box can never create a new target.
      const bool gated = id_match ||
          (candidate_iou >= bytetrack_low_iou_threshold_) ||
          (!kalman_enabled_ &&
           std::hypot(candidate.box.cx() - reference.cx(),
                     candidate.box.cy() - reference.cy()) / diagonal <
               distance_threshold_);
      if (!gated || (kalman_enabled_ && !id_match &&
                     candidate_nis > association_gate_nis_ *
                         bytetrack_low_nis_scale_)) continue;
      if (id_match || candidate_iou > best_iou ||
          (candidate_iou >= bytetrack_low_iou_threshold_ &&
           candidate_nis < best_nis)) {
        selected = candidate;
        best_iou = std::max(best_iou, candidate_iou);
        best_nis = candidate_nis;
      }
    }
    if (selected.box.x2 > selected.box.x1) {
      *method = AssociationMethod::kByteTrackLow;
    }
    return selected;
  }

  Candidate selectCandidate(const std::vector<Candidate>& candidates,
                            const Box& reference,
                            AssociationMethod* method) const {
    if (!isByteTrackProfile()) {
      return selectCandidateSingleStage(candidates, reference, method);
    }

    // ByteTrack stage 1: only high-confidence boxes may establish or refresh
    // the normal association. Appearance is deliberately excluded here so
    // low-quality embeddings cannot create a new target.
    std::vector<Candidate> high;
    high.reserve(candidates.size());
    for (const Candidate& candidate : candidates) {
      if (candidate.confidence >= bytetrack_high_confidence_) {
        high.push_back(candidate);
      }
    }
    Candidate selected = selectCandidateSingleStage(
        high, reference, method, false, true);
    if (selected.box.x2 > selected.box.x1) return selected;

    // ByteTrack stage 2: low-confidence boxes can only recover an existing
    // selected target.  The caller is already in a tracking state, and the
    // returned method carries reduced control authority downstream.
    return selectByteTrackLowStage(candidates, reference, method);
  }

  Candidate selectCandidateSingleStage(const std::vector<Candidate>& candidates,
                                       const Box& reference,
                                       AssociationMethod* method,
                                       const bool allow_appearance = true,
                                       const bool allow_distance = true) const {
    *method = AssociationMethod::kNone;
    Candidate none;
    // 1. Stable detector track-id match.
    if (tracking_strategy_ != "spatial_only" &&
        tracking_strategy_ != "distance_only" && selected_id_stable_) {
      for (const Candidate& candidate : candidates) {
        if (candidate.stable_id && candidate.track_id == selected_track_id_ &&
            candidate.confidence >= min_confidence_) {
          *method = AssociationMethod::kDirectId;
          return candidate;
        }
      }
    }
    if (tracking_strategy_ == "id_only") return none;
    // 2. Spatial match against the current (already predicted) search box.
    double best_iou = 0.0;
    Candidate spatial;
    if (tracking_strategy_ != "distance_only") {
      for (const Candidate& candidate : candidates) {
        if (candidate.confidence < min_confidence_ ||
          (candidate.class_id >= 0 && class_id_ >= 0 &&
           candidate.class_id != class_id_ && !class_match_flexible_)) continue;
        const double score = iou(reference, candidate.box);
        const double width_ratio = candidate.box.width() /
            std::max(1.0, reference.width());
        const double height_ratio = candidate.box.height() /
            std::max(1.0, reference.height());
        if (width_ratio > max_size_change_ratio_ ||
            width_ratio < 1.0 / max_size_change_ratio_ ||
            height_ratio > max_size_change_ratio_ ||
            height_ratio < 1.0 / max_size_change_ratio_) continue;
        if (score > best_iou) {
          best_iou = score;
          spatial = candidate;
        }
      }
    }
    if (best_iou >= iou_threshold_ &&
        (!kalman_enabled_ || kalman_.positionNis(spatial.box) <= association_gate_nis_ ||
         best_iou >= 0.90)) {
      *method = AssociationMethod::kSpatial;
      return spatial;
    }
    if (tracking_strategy_ == "spatial_only") return none;
    // Appearance is only a position-gated fallback after ID/spatial matching;
    // it cannot select a candidate outside the bounded recovery window.
    if (allow_appearance && appearance_reid_enabled_ && !appearance_reference_.empty()) {
      Candidate appearance;
      double best_similarity = appearance_min_cosine_;
      for (const Candidate& candidate : candidates) {
        if (candidate.appearance.size() != appearance_reference_.size() ||
            candidate.confidence < min_confidence_ ||
            (candidate.class_id >= 0 && class_id_ >= 0 &&
             candidate.class_id != class_id_)) continue;
        const double width_ratio = candidate.box.width() /
            std::max(1.0, reference.width());
        const double height_ratio = candidate.box.height() /
            std::max(1.0, reference.height());
        if (width_ratio > max_size_change_ratio_ ||
            width_ratio < 1.0 / max_size_change_ratio_ ||
            height_ratio > max_size_change_ratio_ ||
            height_ratio < 1.0 / max_size_change_ratio_) continue;
        const double normalized = std::hypot(candidate.box.cx() - reference.cx(),
                                             candidate.box.cy() - reference.cy()) /
            std::max(1.0, std::hypot(reference.width(), reference.height()));
        if (normalized > distance_threshold_ * appearance_distance_gate_) continue;
        double dot = 0.0, aa = 0.0, bb = 0.0;
        for (std::size_t i = 0; i < candidate.appearance.size(); ++i) {
          dot += candidate.appearance[i] * appearance_reference_[i];
          aa += candidate.appearance[i] * candidate.appearance[i];
          bb += appearance_reference_[i] * appearance_reference_[i];
        }
        const double similarity = dot / std::max(1e-9, std::sqrt(aa * bb));
        if (similarity > best_similarity) {
          best_similarity = similarity;
          appearance = candidate;
        }
      }
      if (appearance.box.x2 > appearance.box.x1) {
        *method = AssociationMethod::kAppearance;
        return appearance;
      }
    }
    // 3. Center distance fallback for rapid motion with no IoU overlap.
    if (!allow_distance) return none;
    double best_distance = std::numeric_limits<double>::infinity();
    Candidate distance;
    const double diagonal = std::hypot(reference.width(), reference.height());
    for (const Candidate& candidate : candidates) {
      if (candidate.confidence < min_confidence_ ||
          (candidate.class_id >= 0 && class_id_ >= 0 &&
           candidate.class_id != class_id_ && !class_match_flexible_)) continue;
      const double normalized = std::hypot(candidate.box.cx() - reference.cx(),
                                           candidate.box.cy() - reference.cy()) /
                                std::max(1.0, diagonal);
      if (normalized < best_distance &&
          (!kalman_enabled_ || kalman_.positionNis(candidate.box) <= association_gate_nis_)) {
        best_distance = normalized;
        distance = candidate;
      }
    }
    const double expansion = std::min(
        search_expansion_cap_, 1.0 + missing_ * search_expansion_rate_);
    if (best_distance <= distance_threshold_ * expansion) {
      *method = AssociationMethod::kDistance;
      return distance;
    }
    return none;
  }

  Box fusedPrediction(const Box& kalman_box, const double prediction_age) const {
    if (!motion_enabled_ || !have_motion_) return kalman_box;
    const double t = clamp(prediction_age, 0.0, 0.5);
    const double kx = last_observation_box_.cx() + vx_ * t + 0.5 * ax_ * t * t;
    const double ky = last_observation_box_.cy() + vy_ * t + 0.5 * ay_ * t * t;
    const double max_offset = fusion_max_offset_ratio_ *
                              std::hypot(kalman_box.width(), kalman_box.height());
    const double dx = clamp(kx - kalman_box.cx(), -max_offset, max_offset);
    const double dy = clamp(ky - kalman_box.cy(), -max_offset, max_offset);
    const double blend = clamp(0.20 + 0.10 * missing_, 0.20, 0.50);
    const double cx = kalman_box.cx() + blend * dx;
    const double cy = kalman_box.cy() + blend * dy;
    return {static_cast<int>(cx - kalman_box.width() / 2.0),
            static_cast<int>(cy - kalman_box.height() / 2.0),
            static_cast<int>(cx + kalman_box.width() / 2.0),
            static_cast<int>(cy + kalman_box.height() / 2.0)};
  }

  Result update(const double time) {
    Result result;
    if (!tracking_) return result;
    const bool input_rejection =
        last_fusion_status_.rfind("REJECT_", 0) == 0;
    if (!input_rejection) last_fusion_status_ = "NONE";
    const bool oosm_fused = applyOosmMeasurement();
    const bool oosm_rejection = !oosm_fused &&
        last_fusion_status_.rfind("REJECT_", 0) == 0;
    const ros::WallTime wall_now = ros::WallTime::now();
    const double dt = last_filter_time_.isZero() ? 1.0 / rate_ :
        clamp((wall_now - last_filter_time_).toSec(), 1e-3, 0.25);
    last_filter_time_ = wall_now;
    Box predicted = association_box_;
    if (kalman_enabled_) predicted = kalman_.predict(dt);
    association_box_ = fusedPrediction(
        predicted, last_detection_wall_.isZero() ? 0.0 :
        (wall_now - last_detection_wall_).toSec());

    Candidate selected;
    AssociationMethod selected_method = AssociationMethod::kNone;
    if (pending_) {
      pending_ = false;
      selected = selectCandidate(pending_candidates_, association_box_,
                                 &selected_method);
    }
    const bool has_measurement = selected.box.x2 > selected.box.x1;
    if (has_measurement) {
      // In strict mode, recovery after a loss may only use the detector's
      // persistent ID or a gated appearance match.  This prevents a nearby
      // same-class object from immediately steering the aircraft.
      if (missing_ >= max_missing_ && reacquisition_mode_ == "strict" &&
          selected_method != AssociationMethod::kDirectId &&
          selected_method != AssociationMethod::kAppearance) {
        selected = Candidate{};
        selected_method = AssociationMethod::kNone;
      }
    }
    if (selected.box.x2 > selected.box.x1) {
      // Long-loss spatial/distance re-acquisition must persist for several
      // frames before it can produce a direct control measurement.
      const bool needs_confirmation =
          missing_ >= max_missing_ && !selected.stable_id &&
          reacquisition_mode_ != "aggressive";
      if (needs_confirmation) {
        if (tentative_count_ == 0 ||
            iou(tentative_box_, selected.box) < iou_threshold_ * 0.5) {
          tentative_box_ = selected.box;
          tentative_count_ = 1;
        } else {
          ++tentative_count_;
        }
        if (tentative_count_ < std::max(1, reacquire_frames_)) {
          ++missing_;
          result.box = association_box_;
          result.active = prediction_enabled_ && missing_ <= max_missing_ + reacquire_frames_;
          result.predicted = result.active;
          result.association = AssociationMethod::kTentative;
          last_fusion_status_ = "TENTATIVE";
          goto finish;
        }
      }
      tentative_count_ = 0;
      bool accepted = true;
      if (kalman_enabled_) {
        kalman_.setMeasurementNoiseScale(
            measurement_noise_ / std::max(0.05, selected.confidence));
        // The initialization box is already the first measurement.  Do not
        // gate that same first callback: only gate innovations after a
        // confirmed state exists.
        const double nis = kalman_.innovationNis(selected.box);
        // Quantized detector boxes can produce a numerically over-sensitive
        // area/ratio NIS even when their projected geometry is effectively
        // identical.  Preserve the innovation gate for a real spatial jump;
        // the tight IoU bypass prevents that quantization from losing lock.
        const bool geometry_consistent = iou(predicted, selected.box) >= 0.90;
        const bool stable_id_reacquisition = selected.stable_id &&
            selected_id_stable_ && selected.track_id == selected_track_id_;
        // Appearance recovery has already passed class, size, distance, and
        // cosine-similarity gates.  Its geometric innovation may legitimately
        // be large after a long occlusion, so do not discard the verified ReID
        // solely because the short-horizon Kalman state is stale.
        const bool appearance_reacquisition =
            selected_method == AssociationMethod::kAppearance;
        if (!have_confirmed_measurement_ || stable_id_reacquisition ||
            appearance_reacquisition || geometry_consistent ||
            nis <= innovation_gate_nis_) {
          kalman_.update(selected.box);
        } else {
          ROS_WARN_THROTTLE(1.0,
                            "[TrackerNode] Rejected measurement by innovation gate (NIS %.2f > %.2f, measured %.1f/%.1f %.1fx%.1f, predicted %.1f/%.1f %.1fx%.1f)",
                            nis, innovation_gate_nis_, selected.box.cx(),
                            selected.box.cy(), selected.box.width(), selected.box.height(),
                            predicted.cx(), predicted.cy(), predicted.width(), predicted.height());
          accepted = false;
        }
        result.box = kalman_.box();
      } else {
        result.box = selected.box;
      }
      if (!accepted) {
        ++missing_;
        result.box = association_box_;
        result.active = prediction_enabled_ && missing_ <= max_missing_;
        result.predicted = result.active;
        last_fusion_status_ = "REJECT_NIS";
        goto finish;
      }
      confidence_ = confidence_alpha_ * selected.confidence +
                    (1.0 - confidence_alpha_) * confidence_;
      selected_track_id_ = selected.track_id;
      selected_id_stable_ = selected.stable_id;
      target_id_ = selected.stable_id ? selected.track_id : -1;
      if (selected.class_id >= 0) class_id_ = selected.class_id;
      range_valid_ = selected.range_valid;
      if (range_valid_) {
        relative_position_body_ = selected.relative_position_body;
        relative_velocity_body_ = selected.relative_velocity_body;
        position_covariance_ = selected.position_covariance;
        velocity_covariance_ = selected.velocity_covariance;
      }
      missing_ = 0;
      have_confirmed_measurement_ = true;
      last_detection_wall_ = pending_detection_wall_.isZero()
                                 ? wall_now
                                 : pending_detection_wall_;
      if (selected_method != AssociationMethod::kByteTrackLow) {
        last_control_detection_wall_ = last_detection_wall_;
      }
      if (!pending_stamp_.isZero()) {
        last_capture_stamp_ = pending_stamp_;
        last_inference_latency_ms_ = estimateLatencyMs(pending_stamp_);
        if (last_accepted_stamp_.isZero() || pending_stamp_ > last_accepted_stamp_) {
          last_accepted_stamp_ = pending_stamp_;
        }
      }
      last_observation_box_ = selected.box;
      association_box_ = result.box;
      // Use camera acquisition time for motion estimates whenever the
      // detector provides it; legacy zero-stamp sources retain wall-time.
      updateMotion(selected.box,
                   pending_stamp_.isZero() ? time : pending_stamp_.toSec());
      if (appearance_reid_enabled_ && !selected.appearance.empty()) {
        if (appearance_reference_.size() != selected.appearance.size()) {
          appearance_reference_ = selected.appearance;
        } else {
          for (std::size_t i = 0; i < appearance_reference_.size(); ++i) {
            appearance_reference_[i] = static_cast<float>(
                (1.0 - appearance_alpha_) * appearance_reference_[i] +
                appearance_alpha_ * selected.appearance[i]);
          }
        }
      }
      result.active = true;
      result.association = selected_method;
      result.reidentification_match =
          selected_method == AssociationMethod::kAppearance;
      result.control_measurement_ready =
          selected_method != AssociationMethod::kByteTrackLow;
      last_fusion_status_ = "FUSED_CURRENT";
    } else if (oosm_fused) {
      // The historical update has already been projected to the current state;
      // do not count this timer tick as a missing observation.
      result.active = true;
      result.predicted = false;
      result.box = association_box_;
      result.association = last_oosm_method_;
      result.reidentification_match =
          last_oosm_method_ == AssociationMethod::kAppearance;
      result.control_measurement_ready =
          last_oosm_method_ != AssociationMethod::kByteTrackLow;
    } else {
      ++missing_;
      confidence_ = std::max(0.0, confidence_ - confidence_decay_);
      result.box = association_box_;
      result.active = prediction_enabled_ && missing_ <= max_missing_;
      result.predicted = result.active;
      result.association = result.active ? AssociationMethod::kPredicted
                                         : AssociationMethod::kLost;
      if (!input_rejection && !oosm_rejection) {
        last_fusion_status_ = result.active ? "PREDICTED" : "LOST";
      }
    }
finish:
    result.confidence = confidence_;
    result.missing = missing_;
    result.vx = vx_;
    result.vy = vy_;
    result.ax = ax_;
    result.ay = ay_;
    if (result.predicted && result.association == AssociationMethod::kNone) {
      result.association = AssociationMethod::kPredicted;
    }
    if (result.active) {
      result.ex =
          clamp((result.box.cx() - width_ / 2.0) / (width_ / 2.0), -1.0, 1.0);
      result.ey = clamp(
          (result.box.cy() - height_ / 2.0) / (height_ / 2.0), -1.0, 1.0);
      const double ratio =
          result.box.width() * result.box.height() / (width_ * height_);
      result.size_error =
          clamp((ratio - target_ratio_) / target_ratio_, -1.0, 1.0);
      result.quality = confidence_;
      if (result.predicted) {
        result.quality *=
            std::max(0.0, 1.0 - missing_ / 10.0) * 0.8;
      }
      const double nx = result.box.cx() / width_;
      const double ny = result.box.cy() / height_;
      const double edge =
          1.0 -
          std::max({0.0, std::min(std::abs(nx - 0.5) * 2.0 - 0.7, 0.0),
                    std::min(std::abs(ny - 0.5) * 2.0 - 0.7, 0.0)}) /
              0.3;
      result.quality =
          clamp(result.quality * std::max(0.5, edge), 0.0, 1.0);
    }
    recordFilterSnapshot(ros::Time::now());
    return result;
  }

  void tick(const ros::TimerEvent&) {
    Result result;
    ros::WallTime detection_time;
    bool have_direct;
    std::array<double, 3> direct;
    AssociationMethod direct_association;
    double direct_quality;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      result = update(ros::WallTime::now().toSec());
      if (result.active && !result.predicted &&
          result.control_measurement_ready) {
        direct_error_ = {result.ex, result.ey, result.size_error};
        direct_association_ = result.association;
        direct_quality_ = result.quality;
        have_direct_error_ = true;
      }
      // Only a high-confidence/current or confirmed OOSM sample refreshes the
      // direct-control hold window. ByteTrack low-stage recovery may maintain
      // the track for HUD, but must not revive follower control.
      detection_time = last_control_detection_wall_;
      have_direct = have_direct_error_;
      direct = direct_error_;
      direct_association = direct_association_;
      direct_quality = direct_quality_;
    }
    publish(result, detection_time, have_direct, direct, direct_association,
            direct_quality);
    updater_.update();
  }

  void publish(const Result& result, const ros::WallTime detection_time,
               const bool have_direct, const std::array<double, 3>& direct,
               const AssociationMethod direct_association,
               const double direct_quality) {
    const ros::Time stamp = ros::Time::now();
    tracker::TrackingOutput output;
    output.header.stamp = stamp;
    output.tracking_active = result.active;
    output.target_id = target_id_;
    output.class_id = class_id_;
    output.confidence = result.confidence;
    if (result.active) {
      output.position_2d = {
          static_cast<float>(result.box.cx() / width_),
          static_cast<float>(result.box.cy() / height_)};
      output.has_position_2d = true;
      output.bbox = {result.box.x1, result.box.y1, result.box.x2, result.box.y2};
      output.has_bbox = true;
      output.normalized_bbox = {
          static_cast<float>(result.box.cx() / width_),
          static_cast<float>(result.box.cy() / height_),
          static_cast<float>(result.box.width() / width_),
          static_cast<float>(result.box.height() / height_)};
      output.has_normalized_bbox = true;
    }
    output.geometry_type = "aabb";
    output.velocity = {static_cast<float>(result.vx),
                       static_cast<float>(result.vy)};
    output.has_velocity = have_motion_;
    output.acceleration = {static_cast<float>(result.ax),
                           static_cast<float>(result.ay)};
    output.has_acceleration = have_motion_;
    output.error_x = result.ex;
    output.error_y = result.ey;
    output.error_size = result.size_error;
    output.has_normalized_error = result.active;
    output.tracking_quality = result.quality;
    output.frames_since_detection = result.missing;
    output.is_predicted = result.predicted;
    output.association_method = associationName(result.association);
    output.reidentification_match = result.reidentification_match;
    output.control_measurement_ready = result.control_measurement_ready;
    output.control_confidence = static_cast<float>(
        result.control_measurement_ready
            ? result.quality * associationReliability(result.association)
            : 0.0);
    output.capture_timestamp = last_capture_stamp_;
    output.inference_latency_ms = static_cast<float>(last_inference_latency_ms_);
    const auto covariance = kalman_enabled_
        ? kalman_.covarianceDiagonal() : std::array<float, 4>{0.f, 0.f, 0.f, 0.f};
    for (std::size_t i = 0; i < covariance.size(); ++i) {
      output.state_covariance[i] = covariance[i];
    }
    output.fusion_status = last_fusion_status_;
    output.gmc_active = gmc_valid_;
    output.gmc_inlier_count = static_cast<uint32_t>(std::max(0, gmc_inliers_));
    for (std::size_t i = 0; i < 3; ++i) {
      output.target_relative_position_body[i] =
          static_cast<float>(relative_position_body_[i]);
      output.target_relative_velocity_body[i] =
          static_cast<float>(relative_velocity_body_[i]);
    }
    for (std::size_t i = 0; i < 9; ++i) {
      output.position_covariance[i] =
          static_cast<float>(position_covariance_[i]);
      output.velocity_covariance[i] =
          static_cast<float>(velocity_covariance_[i]);
    }
    output.range_valid = range_valid_;
    output.tracker_name = name_;
    output.tracker_type = tracker_profile_ + "_kalman_motion_cpp";
    output_pub_.publish(output);

    tracker::NormalizedError error;
    error.header.stamp = stamp;
    const double age =
        detection_time.isZero()
            ? 1e6
            : std::min((ros::WallTime::now() - detection_time).toSec(), 1e6);
    const bool fresh = result.active && have_direct && age <= hold_time_;
    error.error_x = fresh ? direct[0] : result.ex;
    error.error_y = fresh ? direct[1] : result.ey;
    error.error_size = fresh ? direct[2] : result.size_error;
    error.error_valid = fresh;
    error.target_visible = fresh;
    error.is_estimated = !fresh && result.predicted;
    error.confidence = result.confidence;
    error.tracking_quality = result.quality;
    error.frames_since_detection = std::max(0, result.missing);
    error.dt_since_detection = age;
    error.association_method = associationName(
        fresh ? direct_association : result.association);
    // The direct-observation freshness invariant remains the final authority:
    // prediction and tentative recovery are useful for HUD/search only and
    // must never revive follower control without a confirmed observation.
    error.control_measurement_ready = fresh && have_direct;
    error.control_confidence = static_cast<float>(
        error.control_measurement_ready
            ? direct_quality * associationReliability(direct_association)
            : 0.0);
    error.capture_timestamp = last_capture_stamp_;
    error.inference_latency_ms = static_cast<float>(last_inference_latency_ms_);
    for (std::size_t i = 0; i < covariance.size(); ++i) {
      error.state_covariance[i] = covariance[i];
    }
    error.fusion_status = last_fusion_status_;
    error.gmc_active = gmc_valid_;
    error.gmc_inlier_count = static_cast<uint32_t>(std::max(0, gmc_inliers_));
    for (std::size_t i = 0; i < 3; ++i) {
      error.target_relative_position_body[i] =
          static_cast<float>(relative_position_body_[i]);
      error.target_relative_velocity_body[i] =
          static_cast<float>(relative_velocity_body_[i]);
    }
    for (std::size_t i = 0; i < 9; ++i) {
      error.position_covariance[i] =
          static_cast<float>(position_covariance_[i]);
      error.velocity_covariance[i] =
          static_cast<float>(velocity_covariance_[i]);
    }
    error.range_valid = range_valid_;
    double yaw_error = 0.0;
    double pitch_error = 0.0;
    const bool angular_valid = result.active &&
        angularError(result.box, &yaw_error, &pitch_error);
    error.yaw_error_rad = static_cast<float>(yaw_error);
    error.pitch_error_rad = static_cast<float>(pitch_error);
    error.has_angular_error = fresh && angular_valid;
    error_pub_.publish(error);
  }

  ros::NodeHandle nh_, private_nh_;
  int width_{640}, height_{480}, max_missing_{5};
  double rate_{30.0}, hold_time_{0.15}, target_ratio_{0.15};
  double fov_horizontal_deg_{60.0}, fov_vertical_deg_{45.0};
  double mount_yaw_deg_{0.0}, mount_pitch_deg_{0.0};
  double process_noise_{1.0}, measurement_noise_{1.0};
  double velocity_alpha_{0.7}, acceleration_alpha_{0.5};
  double confidence_alpha_{0.8}, confidence_decay_{0.05};
  double iou_threshold_{0.35}, distance_threshold_{2.0};
  double min_confidence_{0.0}, innovation_gate_nis_{16.0};
  double association_gate_nis_{9.21}, max_size_change_ratio_{2.5};
  double max_detection_age_{0.25};
  bool appearance_reid_enabled_{false};
  double appearance_min_cosine_{0.75}, appearance_alpha_{0.1};
  double bytetrack_high_confidence_{0.50}, bytetrack_low_confidence_{0.10};
  double bytetrack_low_iou_threshold_{0.15}, bytetrack_low_nis_scale_{2.0};
  bool gmc_enabled_{false}, gmc_valid_{false};
  std::string gmc_image_topic_{"/camera/image_raw"};
  int gmc_max_features_{300}, gmc_min_inliers_{12}, gmc_max_image_width_{320};
  double gmc_ransac_threshold_{3.0}, gmc_max_age_sec_{0.15};
  int gmc_inliers_{0}, gmc_applied_count_{0}, gmc_rejected_count_{0};
  double fusion_max_offset_ratio_{0.75};
  int reacquire_frames_{2};
  std::string name_, tracker_profile_{"smart_botsort"};
  std::string image_source_{"eo"};
  std::string smart_backend_{"botsort"}, tracking_strategy_{"hybrid"};
  std::string reacquisition_mode_{"balanced"};
  bool class_match_flexible_{true};
  double search_expansion_rate_{0.20}, search_expansion_cap_{2.0};
  double appearance_distance_gate_{3.0};
  double oosm_history_duration_sec_{0.5}, oosm_max_future_sec_{0.02};
  int oosm_max_snapshots_{40};
  int maximum_embedding_dimension_{2048};
  bool auto_start_{true}, kalman_enabled_{true}, motion_enabled_{true};
  bool prediction_enabled_{true}, tracking_{false}, pending_{false};
  int target_id_{-1}, class_id_{-1}, missing_{0};
  double confidence_{0.0}, pending_confidence_{0.0};
  Box last_observation_box_, association_box_, tentative_box_;
  std::vector<Candidate> pending_candidates_;
  std::vector<float> appearance_reference_;
  std::deque<FilterSnapshot> filter_history_;
  KalmanBox kalman_;
  std::deque<std::pair<Box, double>> history_;
  bool have_motion_{false}, have_confirmed_measurement_{false};
  double vx_{0.0}, vy_{0.0}, ax_{0.0}, ay_{0.0};
  ros::WallTime last_detection_wall_;
  ros::WallTime last_control_detection_wall_;
  bool range_valid_{false};
  std::array<double, 3> relative_position_body_{0.0, 0.0, 0.0};
  std::array<double, 3> relative_velocity_body_{0.0, 0.0, 0.0};
  std::array<double, 9> position_covariance_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                             0.0, 0.0, 0.0};
  std::array<double, 9> velocity_covariance_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                             0.0, 0.0, 0.0};
  ros::WallTime pending_detection_wall_;
  ros::Time pending_stamp_, last_accepted_stamp_;
  ros::WallTime last_filter_time_;
  ros::Time last_filter_ros_stamp_, last_capture_stamp_;
  std::string last_capture_frame_id_;
  double last_inference_latency_ms_{0.0};
  std::string last_fusion_status_{"NONE"};
  AssociationMethod last_oosm_method_{AssociationMethod::kNone};
  int selected_track_id_{-1}, tentative_count_{0};
  bool selected_id_stable_{false};
  bool have_direct_error_{false};
  std::array<double, 3> direct_error_{0.0, 0.0, 0.0};
  AssociationMethod direct_association_{AssociationMethod::kNone};
  double direct_quality_{0.0};
  std::unique_ptr<tracker::MultiTrackManager> multi_track_manager_;
  std::size_t last_track_count_{0};
  double selection_timestamp_tolerance_sec_{0.25};
  std::uint64_t input_message_count_{0};
  std::uint64_t rejected_input_count_{0};
  std::uint64_t selection_count_{0};
  ros::WallTime last_input_wall_;

  bool use_camera_info_{true};
  bool use_dynamic_camera_tf_{true};
  bool require_gimbal_state_for_body_los_{false};
  bool allow_image_source_switch_{false};
  bool require_image_source_metadata_{false};
  bool have_camera_info_{false};
  bool last_tf_success_{false};
  std::string camera_info_topic_{"/camera/camera_info"};
  std::string secondary_camera_info_topic_;
  std::string body_frame_{"base_link"};
  std::string gimbal_state_topic_{"/pod/gimbal/state"};
  std::string default_calibration_key_;
  double tf_lookup_timeout_sec_{0.02};
  double gimbal_state_timeout_sec_{0.15};
  std::uint64_t invalid_camera_info_count_{0};
  std::uint64_t tf_failure_count_{0};
  std::uint64_t image_source_switch_count_{0};
  std::map<std::string, CameraCalibration> camera_calibrations_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  pod_msgs::GimbalState gimbal_state_;
  ros::WallTime gimbal_state_time_;
  bool have_gimbal_state_{false};

  bool gmc_require_detection_sync_{false};
  double gmc_sync_tolerance_sec_{0.05};
  int gmc_sync_rejected_count_{0};
  ros::Time gmc_last_stamp_;
  std::string gmc_frame_id_;
  std::mutex mutex_;
  ros::Publisher output_pub_, error_pub_, tracks_pub_;
  ros::Subscriber input_sub_, candidates_sub_, image_sub_, gimbal_state_sub_;
  std::vector<ros::Subscriber> camera_info_subs_;
  ros::ServiceServer select_track_srv_;
  diagnostic_updater::Updater updater_;
  cv::Ptr<cv::ORB> gmc_orb_;
  cv::Mat previous_gray_, previous_descriptors_;
  std::vector<cv::KeyPoint> previous_keypoints_;
  ros::Timer timer_;
};
}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "tracker_node");
  TrackerNode node;
  // Tracking state is intentionally advanced in one ordered callback queue.
  ros::spin();
  return 0;
}
