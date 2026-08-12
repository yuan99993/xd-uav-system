#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include <GeographicLib/LocalCartesian.hpp>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <geographic_msgs/GeoPointStamped.h>
#include <mavros_msgs/EstimatorStatus.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Bool.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace {

bool validGeodetic(const double latitude, const double longitude,
                   const double altitude) {
  return std::isfinite(latitude) && std::isfinite(longitude) &&
         std::isfinite(altitude) && std::abs(latitude) <= 90.0 &&
         std::abs(longitude) <= 180.0;
}

bool normalize(tf2::Quaternion* quaternion) {
  if (!std::isfinite(quaternion->x()) ||
      !std::isfinite(quaternion->y()) ||
      !std::isfinite(quaternion->z()) ||
      !std::isfinite(quaternion->w()) ||
      quaternion->length2() < 1e-9) {
    return false;
  }
  quaternion->normalize();
  return true;
}

void addDiagnostic(diagnostic_msgs::DiagnosticStatus* status,
                   const std::string& key, const std::string& value) {
  diagnostic_msgs::KeyValue item;
  item.key = key;
  item.value = value;
  status->values.push_back(item);
}

void addDiagnostic(diagnostic_msgs::DiagnosticStatus* status,
                   const std::string& key, const bool value) {
  addDiagnostic(status, key,
                std::string(value ? "true" : "false"));
}

void addDiagnostic(diagnostic_msgs::DiagnosticStatus* status,
                   const std::string& key, const double value) {
  addDiagnostic(status, key, std::to_string(value));
}

}  // 匿名命名空间

class GpsGlobalAlignment {
 public:
  GpsGlobalAlignment() : private_nh_("~") {
    loadParameters();

    origin_subscriber_ = nh_.subscribe(
        origin_topic_, 2, &GpsGlobalAlignment::originCallback, this);
    position_subscriber_ = nh_.subscribe(
        position_topic_, 10, &GpsGlobalAlignment::positionCallback, this);
    imu_subscriber_ = nh_.subscribe(
        imu_topic_, 30, &GpsGlobalAlignment::imuCallback, this);
    if (require_estimator_status_) {
      estimator_status_subscriber_ = nh_.subscribe(
          estimator_status_topic_, 10,
          &GpsGlobalAlignment::estimatorStatusCallback, this);
    }

    odometry_publisher_ =
        nh_.advertise<nav_msgs::Odometry>(output_topic_, 10);
    valid_publisher_ = nh_.advertise<std_msgs::Bool>(valid_topic_, 1, true);
    diagnostics_publisher_ =
        nh_.advertise<diagnostic_msgs::DiagnosticArray>(
            diagnostics_topic_, 2);
    timer_ = private_nh_.createTimer(
        ros::Duration(1.0 / std::max(1.0, publish_rate_)),
        &GpsGlobalAlignment::timerCallback, this);
    publishValid(false);
    ROS_INFO(
        "[xd_uav_gps_global_alignment] 等待%s和%s",
        origin_topic_.c_str(), position_topic_.c_str());
  }

 private:
  void loadParameters() {
    private_nh_.param("origin_topic", origin_topic_,
                      std::string("mavros/global_position/gp_origin"));
    private_nh_.param("position_topic", position_topic_,
                      std::string("mavros/global_position/global"));
    private_nh_.param("imu_topic", imu_topic_,
                      std::string("mavros/imu/data"));
    private_nh_.param("estimator_status_topic", estimator_status_topic_,
                      std::string("mavros/estimator_status"));
    private_nh_.param("output_topic", output_topic_,
                      std::string("global_alignment/gps/body_odometry"));
    private_nh_.param("valid_topic", valid_topic_,
                      std::string("global_alignment/gps/valid"));
    private_nh_.param("diagnostics_topic", diagnostics_topic_,
                      std::string("global_alignment/gps/diagnostics"));
    private_nh_.param("local_origin_frame", local_origin_frame_,
                      std::string("uav1/local_origin"));
    private_nh_.param("body_frame", body_frame_,
                      std::string("uav1/base_link"));
    private_nh_.param("publish_rate", publish_rate_, 20.0);
    private_nh_.param("input_timeout", input_timeout_, 1.0);
    private_nh_.param("max_imu_time_difference",
                      max_imu_time_difference_, 0.10);
    private_nh_.param("require_estimator_status",
                      require_estimator_status_, false);
    private_nh_.param("allow_unknown_covariance",
                      allow_unknown_covariance_, false);
    private_nh_.param("max_horizontal_variance",
                      max_horizontal_variance_, 25.0);
    private_nh_.param("max_vertical_variance",
                      max_vertical_variance_, 100.0);
    private_nh_.param("default_orientation_variance",
                      default_orientation_variance_, 1.0);
    private_nh_.param("max_horizontal_speed",
                      max_horizontal_speed_, 120.0);
    private_nh_.param("max_vertical_speed",
                      max_vertical_speed_, 30.0);
    private_nh_.param("jump_margin", jump_margin_, 3.0);
    private_nh_.param("max_origin_change", max_origin_change_, 1.0);
  }

  void originCallback(
      const geographic_msgs::GeoPointStamped::ConstPtr& message) {
    const double latitude = message->position.latitude;
    const double longitude = message->position.longitude;
    const double altitude = message->position.altitude;
    if (!validGeodetic(latitude, longitude, altitude)) {
      last_failure_reason_ = "GPS全局原点无效";
      return;
    }

    if (have_origin_) {
      double east = 0.0;
      double north = 0.0;
      double up = 0.0;
      converter_->Forward(latitude, longitude, altitude,
                          east, north, up);
      const double change =
          std::sqrt(east * east + north * north + up * up);
      if (change > max_origin_change_) {
        ROS_WARN(
            "[xd_uav_gps_global_alignment] gp_origin变化%.3fm，"
            "重建ENU原点并清除跳变历史",
            change);
        have_last_output_ = false;
        last_position_accepted_ = false;
      }
    }

    converter_ = std::make_unique<GeographicLib::LocalCartesian>(
        latitude, longitude, altitude);
    origin_latitude_ = latitude;
    origin_longitude_ = longitude;
    origin_altitude_ = altitude;
    have_origin_ = true;
    last_origin_receive_ = ros::Time::now();
  }

  void positionCallback(
      const sensor_msgs::NavSatFix::ConstPtr& message) {
    latest_position_ = *message;
    have_position_ = true;
    new_position_ = true;
    last_position_receive_ = ros::Time::now();
  }

  void imuCallback(const sensor_msgs::Imu::ConstPtr& message) {
    tf2::Quaternion quaternion;
    tf2::fromMsg(message->orientation, quaternion);
    if (!normalize(&quaternion)) {
      last_failure_reason_ = "IMU姿态无效";
      return;
    }
    latest_imu_ = *message;
    latest_imu_.orientation = tf2::toMsg(quaternion);
    have_imu_ = true;
    last_imu_receive_ = ros::Time::now();
  }

  void estimatorStatusCallback(
      const mavros_msgs::EstimatorStatus::ConstPtr& message) {
    latest_estimator_status_ = *message;
    have_estimator_status_ = true;
    last_estimator_status_receive_ = ros::Time::now();
  }

  bool covarianceValid(std::string* reason) const {
    if (latest_position_.position_covariance_type ==
            sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN &&
        !allow_unknown_covariance_) {
      *reason = "GPS位置协方差未知";
      return false;
    }
    const double variance_east = latest_position_.position_covariance[0];
    const double variance_north = latest_position_.position_covariance[4];
    const double variance_up = latest_position_.position_covariance[8];
    if (!std::isfinite(variance_east) ||
        !std::isfinite(variance_north) ||
        !std::isfinite(variance_up) ||
        variance_east < 0.0 || variance_north < 0.0 ||
        variance_up < 0.0) {
      *reason = "GPS位置协方差数值无效";
      return false;
    }
    if (variance_east > max_horizontal_variance_ ||
        variance_north > max_horizontal_variance_ ||
        variance_up > max_vertical_variance_) {
      *reason = "GPS位置协方差超过阈值";
      return false;
    }
    return true;
  }

  bool inputsValid(const ros::Time& now, std::string* reason) const {
    if (!have_origin_) {
      *reason = "等待gp_origin";
      return false;
    }
    if (!have_position_) {
      *reason = "等待全球位置";
      return false;
    }
    if (!have_imu_) {
      *reason = "等待IMU姿态";
      return false;
    }
    if ((now - last_position_receive_).toSec() > input_timeout_) {
      *reason = "全球位置超时";
      return false;
    }
    if ((now - last_imu_receive_).toSec() > input_timeout_) {
      *reason = "IMU姿态超时";
      return false;
    }
    if (!latest_position_.header.stamp.isZero() &&
        !latest_imu_.header.stamp.isZero() &&
        std::abs((latest_position_.header.stamp -
                  latest_imu_.header.stamp).toSec()) >
            max_imu_time_difference_) {
      *reason = "GPS位置与IMU姿态时间差过大";
      return false;
    }
    if (latest_position_.status.status <
        sensor_msgs::NavSatStatus::STATUS_FIX) {
      *reason = "GPS没有有效定位";
      return false;
    }
    if (!validGeodetic(latest_position_.latitude,
                       latest_position_.longitude,
                       latest_position_.altitude)) {
      *reason = "飞机全球经纬高无效";
      return false;
    }
    if (!covarianceValid(reason)) {
      return false;
    }
    if (require_estimator_status_) {
      if (!have_estimator_status_ ||
          (now - last_estimator_status_receive_).toSec() >
              input_timeout_) {
        *reason = "PX4估计状态超时";
        return false;
      }
      if (!latest_estimator_status_.attitude_status_flag ||
          !latest_estimator_status_.pos_horiz_abs_status_flag ||
          !latest_estimator_status_.pos_vert_abs_status_flag ||
          latest_estimator_status_.gps_glitch_status_flag) {
        *reason = "PX4绝对位置估计无效";
        return false;
      }
    }
    return true;
  }

  bool motionAccepted(const std::array<double, 3>& position,
                      const ros::Time& stamp, std::string* reason) const {
    if (!have_last_output_) {
      return true;
    }
    const double dt = (stamp - last_output_stamp_).toSec();
    if (dt <= 0.0) {
      // MAVLink消息在高发布率下可能重复同一个测量时间戳；
      // 重复样本不表示定位失效，也不能用于速度跳变判断。
      return true;
    }
    const double dx = position[0] - last_output_position_[0];
    const double dy = position[1] - last_output_position_[1];
    const double dz = position[2] - last_output_position_[2];
    const double horizontal_distance = std::sqrt(dx * dx + dy * dy);
    if (horizontal_distance > max_horizontal_speed_ * dt + jump_margin_ ||
        std::abs(dz) > max_vertical_speed_ * dt + jump_margin_) {
      *reason = "GPS局部位置变化超过运动上限";
      return false;
    }
    return true;
  }

  double orientationVariance(const std::size_t index) const {
    const double variance = latest_imu_.orientation_covariance[index];
    if (std::isfinite(variance) && variance > 0.0) {
      return variance;
    }
    return default_orientation_variance_;
  }

  void publishOdometry(const std::array<double, 3>& position,
                       const ros::Time& stamp) {
    nav_msgs::Odometry message;
    message.header.stamp = stamp;
    message.header.frame_id = local_origin_frame_;
    message.child_frame_id = body_frame_;
    message.pose.pose.position.x = position[0];
    message.pose.pose.position.y = position[1];
    message.pose.pose.position.z = position[2];
    message.pose.pose.orientation = latest_imu_.orientation;

    message.pose.covariance.fill(0.0);
    for (std::size_t row = 0; row < 3; ++row) {
      for (std::size_t column = 0; column < 3; ++column) {
        message.pose.covariance[row * 6 + column] =
            latest_position_.position_covariance[row * 3 + column];
      }
    }
    message.pose.covariance[21] = orientationVariance(0);
    message.pose.covariance[28] = orientationVariance(4);
    message.pose.covariance[35] = orientationVariance(8);
    message.twist.covariance.fill(0.0);
    message.twist.covariance[0] = -1.0;
    odometry_publisher_.publish(message);
  }

  void timerCallback(const ros::TimerEvent&) {
    const ros::Time now = ros::Time::now();
    std::string reason;
    bool valid = inputsValid(now, &reason);
    if (valid && new_position_) {
      double east = 0.0;
      double north = 0.0;
      double up = 0.0;
      converter_->Forward(
          latest_position_.latitude, latest_position_.longitude,
          latest_position_.altitude, east, north, up);
      const std::array<double, 3> position{{east, north, up}};
      const ros::Time stamp =
          latest_position_.header.stamp.isZero()
              ? last_position_receive_
              : latest_position_.header.stamp;
      valid = motionAccepted(position, stamp, &reason);
      if (valid) {
        publishOdometry(position, stamp);
        last_output_position_ = position;
        last_output_stamp_ = stamp;
        have_last_output_ = true;
      }
      last_position_accepted_ = valid;
      new_position_ = false;
    } else if (valid) {
      valid = last_position_accepted_;
      if (!valid) {
        reason = "等待首个可接受GPS位置";
      }
    }

    if (!valid) {
      last_failure_reason_ = reason;
    } else {
      last_failure_reason_.clear();
    }
    publishValid(valid);
    publishDiagnostics(now, valid, reason);
  }

  void publishValid(const bool valid) {
    std_msgs::Bool message;
    message.data = valid;
    valid_publisher_.publish(message);
  }

  void publishDiagnostics(const ros::Time& now, const bool valid,
                          const std::string& reason) {
    if (!last_diagnostics_.isZero() &&
        (now - last_diagnostics_).toSec() < 0.5) {
      return;
    }
    last_diagnostics_ = now;
    diagnostic_msgs::DiagnosticArray array;
    array.header.stamp = now;
    diagnostic_msgs::DiagnosticStatus status;
    status.name = local_origin_frame_ + "/gps_global_alignment";
    status.hardware_id = body_frame_;
    status.level = valid ? diagnostic_msgs::DiagnosticStatus::OK
                         : diagnostic_msgs::DiagnosticStatus::WARN;
    status.message =
        valid ? "GPS全局位姿修正有效"
              : (reason.empty() ? last_failure_reason_ : reason);
    addDiagnostic(&status, "valid", valid);
    addDiagnostic(&status, "have_origin", have_origin_);
    addDiagnostic(&status, "have_position", have_position_);
    addDiagnostic(&status, "have_imu", have_imu_);
    addDiagnostic(&status, "require_estimator_status",
                  require_estimator_status_);
    addDiagnostic(&status, "origin_latitude", origin_latitude_);
    addDiagnostic(&status, "origin_longitude", origin_longitude_);
    addDiagnostic(&status, "origin_altitude", origin_altitude_);
    array.status.push_back(status);
    diagnostics_publisher_.publish(array);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber origin_subscriber_;
  ros::Subscriber position_subscriber_;
  ros::Subscriber imu_subscriber_;
  ros::Subscriber estimator_status_subscriber_;
  ros::Publisher odometry_publisher_;
  ros::Publisher valid_publisher_;
  ros::Publisher diagnostics_publisher_;
  ros::Timer timer_;

  std::string origin_topic_;
  std::string position_topic_;
  std::string imu_topic_;
  std::string estimator_status_topic_;
  std::string output_topic_;
  std::string valid_topic_;
  std::string diagnostics_topic_;
  std::string local_origin_frame_;
  std::string body_frame_;
  std::string last_failure_reason_;

  std::unique_ptr<GeographicLib::LocalCartesian> converter_;
  sensor_msgs::NavSatFix latest_position_;
  sensor_msgs::Imu latest_imu_;
  mavros_msgs::EstimatorStatus latest_estimator_status_;
  std::array<double, 3> last_output_position_{{0.0, 0.0, 0.0}};
  ros::Time last_origin_receive_;
  ros::Time last_position_receive_;
  ros::Time last_imu_receive_;
  ros::Time last_estimator_status_receive_;
  ros::Time last_output_stamp_;
  ros::Time last_diagnostics_;

  double origin_latitude_{std::numeric_limits<double>::quiet_NaN()};
  double origin_longitude_{std::numeric_limits<double>::quiet_NaN()};
  double origin_altitude_{std::numeric_limits<double>::quiet_NaN()};
  double publish_rate_{20.0};
  double input_timeout_{1.0};
  double max_imu_time_difference_{0.10};
  double max_horizontal_variance_{25.0};
  double max_vertical_variance_{100.0};
  double default_orientation_variance_{1.0};
  double max_horizontal_speed_{120.0};
  double max_vertical_speed_{30.0};
  double jump_margin_{3.0};
  double max_origin_change_{1.0};

  bool require_estimator_status_{false};
  bool allow_unknown_covariance_{false};
  bool have_origin_{false};
  bool have_position_{false};
  bool have_imu_{false};
  bool have_estimator_status_{false};
  bool have_last_output_{false};
  bool new_position_{false};
  bool last_position_accepted_{false};
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "gps_global_alignment");
  GpsGlobalAlignment node;
  ros::spin();
  return 0;
}
