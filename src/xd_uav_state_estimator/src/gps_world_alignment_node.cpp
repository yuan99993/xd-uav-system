#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <string>

#include <geographic_msgs/GeoPointStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

namespace {

constexpr double kPi = 3.14159265358979323846;

double wrapAngle(const double angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}

double interpolateAngle(const double from, const double to, const double alpha) {
  return wrapAngle(from + alpha * wrapAngle(to - from));
}

struct GeodeticPoint {
  double latitude = 0.0;
  double longitude = 0.0;
  double altitude = 0.0;
};

std::array<double, 3> geodeticToEcef(const GeodeticPoint& point) {
  constexpr double semi_major_axis = 6378137.0;
  constexpr double flattening = 1.0 / 298.257223563;
  constexpr double eccentricity_squared = flattening * (2.0 - flattening);

  const double latitude = point.latitude * kPi / 180.0;
  const double longitude = point.longitude * kPi / 180.0;
  const double sin_latitude = std::sin(latitude);
  const double cos_latitude = std::cos(latitude);
  const double prime_vertical_radius =
      semi_major_axis / std::sqrt(1.0 - eccentricity_squared * sin_latitude * sin_latitude);

  return {
      (prime_vertical_radius + point.altitude) * cos_latitude * std::cos(longitude),
      (prime_vertical_radius + point.altitude) * cos_latitude * std::sin(longitude),
      (prime_vertical_radius * (1.0 - eccentricity_squared) + point.altitude) * sin_latitude};
}

std::array<double, 3> geodeticToEnu(const GeodeticPoint& point, const GeodeticPoint& origin) {
  const auto ecef = geodeticToEcef(point);
  const auto origin_ecef = geodeticToEcef(origin);
  const double dx = ecef[0] - origin_ecef[0];
  const double dy = ecef[1] - origin_ecef[1];
  const double dz = ecef[2] - origin_ecef[2];

  const double latitude = origin.latitude * kPi / 180.0;
  const double longitude = origin.longitude * kPi / 180.0;
  const double sin_latitude = std::sin(latitude);
  const double cos_latitude = std::cos(latitude);
  const double sin_longitude = std::sin(longitude);
  const double cos_longitude = std::cos(longitude);

  return {
      -sin_longitude * dx + cos_longitude * dy,
      -sin_latitude * cos_longitude * dx - sin_latitude * sin_longitude * dy + cos_latitude * dz,
      cos_latitude * cos_longitude * dx + cos_latitude * sin_longitude * dy + sin_latitude * dz};
}

bool isFinite(const GeodeticPoint& point) {
  return std::isfinite(point.latitude) && std::isfinite(point.longitude) &&
         std::isfinite(point.altitude) && std::abs(point.latitude) <= 90.0 &&
         std::abs(point.longitude) <= 180.0;
}

}  // namespace

class GpsWorldAlignment {
 public:
  GpsWorldAlignment() : private_nh_("~") {
    private_nh_.param("world_frame", world_frame_, std::string("world"));
    private_nh_.param("local_origin_frame", local_origin_frame_, std::string("uav1/local_origin"));
    private_nh_.param("odom_frame", odom_frame_, std::string("uav1/odom"));
    private_nh_.param("use_altitude", use_altitude_, true);
    private_nh_.param("input_timeout", input_timeout_, 1.0);
    private_nh_.param("max_position_covariance", max_position_covariance_, 25.0);
    private_nh_.param("position_filter_alpha", position_filter_alpha_, 0.08);
    private_nh_.param("yaw_filter_alpha", yaw_filter_alpha_, 0.08);
    private_nh_.param("max_position_jump", max_position_jump_, 10.0);
    private_nh_.param("max_yaw_jump", max_yaw_jump_, 1.57079632679);
    private_nh_.param("heading_offset", heading_offset_, 0.0);
    private_nh_.param("publish_rate", publish_rate_, 30.0);

    private_nh_.param("world_datum/latitude", world_datum_.latitude,
                      std::numeric_limits<double>::quiet_NaN());
    private_nh_.param("world_datum/longitude", world_datum_.longitude,
                      std::numeric_limits<double>::quiet_NaN());
    private_nh_.param("world_datum/altitude", world_datum_.altitude,
                      std::numeric_limits<double>::quiet_NaN());

    position_filter_alpha_ = std::clamp(position_filter_alpha_, 0.0, 1.0);
    yaw_filter_alpha_ = std::clamp(yaw_filter_alpha_, 0.0, 1.0);
    publish_rate_ = std::max(1.0, publish_rate_);

    fix_subscriber_ = nh_.subscribe("gps_fix", 10, &GpsWorldAlignment::fixCallback, this);
    origin_subscriber_ =
        nh_.subscribe("gps_origin", 1, &GpsWorldAlignment::originCallback, this);
    heading_subscriber_ =
        nh_.subscribe("heading", 10, &GpsWorldAlignment::headingCallback, this);
    odometry_subscriber_ =
        nh_.subscribe("odometry", 20, &GpsWorldAlignment::odometryCallback, this);
    valid_publisher_ = private_nh_.advertise<std_msgs::Bool>("valid", 1, true);
    timer_ = private_nh_.createTimer(ros::Duration(1.0 / publish_rate_),
                                     &GpsWorldAlignment::timerCallback, this);

    publishValid(false);
    if (!isFinite(world_datum_)) {
      ROS_ERROR("gps_world_alignment: world_datum latitude/longitude/altitude are invalid");
    }
  }

 private:
  void originCallback(const geographic_msgs::GeoPointStamped::ConstPtr& message) {
    const GeodeticPoint origin{message->position.latitude, message->position.longitude,
                               message->position.altitude};
    if (!isFinite(origin)) {
      ROS_WARN_THROTTLE(2.0, "gps_world_alignment: received an invalid GPS origin");
      return;
    }

    const bool changed = !have_origin_ || std::abs(origin.latitude - gps_origin_.latitude) > 1e-10 ||
                         std::abs(origin.longitude - gps_origin_.longitude) > 1e-10 ||
                         std::abs(origin.altitude - gps_origin_.altitude) > 1e-3;
    gps_origin_ = origin;
    have_origin_ = true;
    if (changed) {
      alignment_initialized_ = false;
      publishWorldToLocalOrigin();
    }
  }

  void fixCallback(const sensor_msgs::NavSatFix::ConstPtr& message) {
    if (message->status.status == sensor_msgs::NavSatStatus::STATUS_NO_FIX ||
        !std::isfinite(message->latitude) || !std::isfinite(message->longitude) ||
        !std::isfinite(message->altitude)) {
      return;
    }

    if (message->position_covariance_type != sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN) {
      const double maximum_covariance =
          std::max({message->position_covariance[0], message->position_covariance[4],
                    message->position_covariance[8]});
      if (!std::isfinite(maximum_covariance) || maximum_covariance > max_position_covariance_) {
        ROS_WARN_THROTTLE(2.0, "gps_world_alignment: GPS covariance is too large");
        return;
      }
    }

    latest_fix_ = {message->latitude, message->longitude, message->altitude};
    fix_received_time_ = ros::Time::now();
    have_fix_ = true;
    updateAlignment();
  }

  void headingCallback(const std_msgs::Float64::ConstPtr& message) {
    if (!std::isfinite(message->data)) {
      return;
    }
    // MAVROS compass_hdg is degrees clockwise from north. ROS ENU yaw is
    // radians counter-clockwise from east.
    latest_world_body_yaw_ =
        wrapAngle(kPi / 2.0 - message->data * kPi / 180.0 + heading_offset_);
    heading_received_time_ = ros::Time::now();
    have_heading_ = true;
    updateAlignment();
  }

  void odometryCallback(const nav_msgs::Odometry::ConstPtr& message) {
    const auto& position = message->pose.pose.position;
    const auto& orientation = message->pose.pose.orientation;
    if (!std::isfinite(position.x) || !std::isfinite(position.y) || !std::isfinite(position.z) ||
        !std::isfinite(orientation.x) || !std::isfinite(orientation.y) ||
        !std::isfinite(orientation.z) || !std::isfinite(orientation.w)) {
      return;
    }

    odom_body_position_ = {position.x, position.y, position.z};
    const tf2::Quaternion quaternion(orientation.x, orientation.y, orientation.z, orientation.w);
    double roll = 0.0;
    double pitch = 0.0;
    tf2::Matrix3x3(quaternion).getRPY(roll, pitch, odom_body_yaw_);
    odometry_received_time_ = ros::Time::now();
    have_odometry_ = true;
    updateAlignment();
  }

  bool inputsAreFresh(const ros::Time& now) const {
    if (!have_origin_ || !have_fix_ || !have_heading_ || !have_odometry_ ||
        !isFinite(world_datum_)) {
      return false;
    }
    return (now - fix_received_time_).toSec() <= input_timeout_ &&
           (now - heading_received_time_).toSec() <= input_timeout_ &&
           (now - odometry_received_time_).toSec() <= input_timeout_;
  }

  void updateAlignment() {
    const ros::Time now = ros::Time::now();
    if (!inputsAreFresh(now)) {
      return;
    }

    auto local_body_position = geodeticToEnu(latest_fix_, gps_origin_);
    if (!use_altitude_) {
      local_body_position[2] = odom_body_position_[2];
    }

    const double candidate_yaw = wrapAngle(latest_world_body_yaw_ - odom_body_yaw_);
    const double cosine = std::cos(candidate_yaw);
    const double sine = std::sin(candidate_yaw);
    std::array<double, 3> candidate_translation{
        local_body_position[0] -
            (cosine * odom_body_position_[0] - sine * odom_body_position_[1]),
        local_body_position[1] -
            (sine * odom_body_position_[0] + cosine * odom_body_position_[1]),
        local_body_position[2] - odom_body_position_[2]};

    if (!alignment_initialized_) {
      translation_ = candidate_translation;
      yaw_ = candidate_yaw;
      alignment_initialized_ = true;
      ROS_INFO("gps_world_alignment: initialized %s -> %s", local_origin_frame_.c_str(),
               odom_frame_.c_str());
      return;
    }

    const double dx = candidate_translation[0] - translation_[0];
    const double dy = candidate_translation[1] - translation_[1];
    const double dz = candidate_translation[2] - translation_[2];
    const double translation_jump = std::sqrt(dx * dx + dy * dy + dz * dz);
    const double yaw_jump = std::abs(wrapAngle(candidate_yaw - yaw_));
    if (translation_jump > max_position_jump_ || yaw_jump > max_yaw_jump_) {
      ROS_WARN_THROTTLE(2.0,
                        "gps_world_alignment: rejected correction jump (position %.2f m, yaw %.1f deg)",
                        translation_jump, yaw_jump * 180.0 / kPi);
      return;
    }

    for (std::size_t index = 0; index < translation_.size(); ++index) {
      translation_[index] +=
          position_filter_alpha_ * (candidate_translation[index] - translation_[index]);
    }
    yaw_ = interpolateAngle(yaw_, candidate_yaw, yaw_filter_alpha_);
  }

  void publishWorldToLocalOrigin() {
    if (!have_origin_ || !isFinite(world_datum_)) {
      return;
    }
    const auto translation = geodeticToEnu(gps_origin_, world_datum_);
    geometry_msgs::TransformStamped transform;
    transform.header.stamp = ros::Time::now();
    transform.header.frame_id = world_frame_;
    transform.child_frame_id = local_origin_frame_;
    transform.transform.translation.x = translation[0];
    transform.transform.translation.y = translation[1];
    transform.transform.translation.z = translation[2];
    transform.transform.rotation.w = 1.0;
    static_broadcaster_.sendTransform(transform);
    ROS_INFO("gps_world_alignment: published %s -> %s at ENU [%.3f, %.3f, %.3f] m",
             world_frame_.c_str(), local_origin_frame_.c_str(), translation[0], translation[1],
             translation[2]);
  }

  void timerCallback(const ros::TimerEvent&) {
    const bool valid = alignment_initialized_ && inputsAreFresh(ros::Time::now());
    publishValid(valid);
    if (!alignment_initialized_) {
      return;
    }

    geometry_msgs::TransformStamped transform;
    transform.header.stamp = ros::Time::now();
    transform.header.frame_id = local_origin_frame_;
    transform.child_frame_id = odom_frame_;
    transform.transform.translation.x = translation_[0];
    transform.transform.translation.y = translation_[1];
    transform.transform.translation.z = translation_[2];
    tf2::Quaternion quaternion;
    quaternion.setRPY(0.0, 0.0, yaw_);
    transform.transform.rotation.x = quaternion.x();
    transform.transform.rotation.y = quaternion.y();
    transform.transform.rotation.z = quaternion.z();
    transform.transform.rotation.w = quaternion.w();
    transform_broadcaster_.sendTransform(transform);
  }

  void publishValid(const bool valid) {
    if (valid == last_published_valid_ && have_published_valid_) {
      return;
    }
    std_msgs::Bool message;
    message.data = valid;
    valid_publisher_.publish(message);
    last_published_valid_ = valid;
    have_published_valid_ = true;
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber fix_subscriber_;
  ros::Subscriber origin_subscriber_;
  ros::Subscriber heading_subscriber_;
  ros::Subscriber odometry_subscriber_;
  ros::Publisher valid_publisher_;
  ros::Timer timer_;
  tf2_ros::StaticTransformBroadcaster static_broadcaster_;
  tf2_ros::TransformBroadcaster transform_broadcaster_;

  std::string world_frame_;
  std::string local_origin_frame_;
  std::string odom_frame_;
  GeodeticPoint world_datum_;
  GeodeticPoint gps_origin_;
  GeodeticPoint latest_fix_;
  std::array<double, 3> odom_body_position_{0.0, 0.0, 0.0};
  std::array<double, 3> translation_{0.0, 0.0, 0.0};
  double odom_body_yaw_ = 0.0;
  double latest_world_body_yaw_ = 0.0;
  double yaw_ = 0.0;
  bool use_altitude_ = true;
  double input_timeout_ = 1.0;
  double max_position_covariance_ = 25.0;
  double position_filter_alpha_ = 0.08;
  double yaw_filter_alpha_ = 0.08;
  double max_position_jump_ = 10.0;
  double max_yaw_jump_ = 1.57079632679;
  double heading_offset_ = 0.0;
  double publish_rate_ = 30.0;
  bool have_origin_ = false;
  bool have_fix_ = false;
  bool have_heading_ = false;
  bool have_odometry_ = false;
  bool alignment_initialized_ = false;
  bool have_published_valid_ = false;
  bool last_published_valid_ = false;
  ros::Time fix_received_time_;
  ros::Time heading_received_time_;
  ros::Time odometry_received_time_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "gps_world_alignment");
  GpsWorldAlignment alignment;
  ros::spin();
  return 0;
}
