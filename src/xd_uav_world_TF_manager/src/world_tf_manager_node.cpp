#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <XmlRpcValue.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <geographic_msgs/GeoPointStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/static_transform_broadcaster.h>

namespace {

constexpr double kPi = 3.14159265358979323846;

std::string trimSlashes(std::string value) {
  while (!value.empty() && value.front() == '/') {
    value.erase(value.begin());
  }
  while (!value.empty() && value.back() == '/') {
    value.pop_back();
  }
  return value;
}

struct GeodeticPoint {
  double latitude{0.0};
  double longitude{0.0};
  double altitude{0.0};
};

bool validGeodetic(const GeodeticPoint& point) {
  return std::isfinite(point.latitude) && std::isfinite(point.longitude) &&
         std::isfinite(point.altitude) && std::abs(point.latitude) <= 90.0 &&
         std::abs(point.longitude) <= 180.0;
}

std::array<double, 3> geodeticToEcef(const GeodeticPoint& point) {
  constexpr double semi_major_axis = 6378137.0;
  constexpr double flattening = 1.0 / 298.257223563;
  constexpr double eccentricity_squared = flattening * (2.0 - flattening);
  const double latitude = point.latitude * kPi / 180.0;
  const double longitude = point.longitude * kPi / 180.0;
  const double sin_latitude = std::sin(latitude);
  const double cos_latitude = std::cos(latitude);
  const double radius =
      semi_major_axis / std::sqrt(1.0 - eccentricity_squared * sin_latitude * sin_latitude);
  return {{
      (radius + point.altitude) * cos_latitude * std::cos(longitude),
      (radius + point.altitude) * cos_latitude * std::sin(longitude),
      (radius * (1.0 - eccentricity_squared) + point.altitude) * sin_latitude}};
}

std::array<double, 3> geodeticToEnu(const GeodeticPoint& point,
                                    const GeodeticPoint& origin) {
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
  return {{
      -sin_longitude * dx + cos_longitude * dy,
      -sin_latitude * cos_longitude * dx - sin_latitude * sin_longitude * dy +
          cos_latitude * dz,
      cos_latitude * cos_longitude * dx + cos_latitude * sin_longitude * dy +
          sin_latitude * dz}};
}

double xmlNumber(const XmlRpc::XmlRpcValue& value) {
  if (value.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
    return static_cast<double>(value);
  }
  if (value.getType() == XmlRpc::XmlRpcValue::TypeInt) {
    return static_cast<int>(value);
  }
  throw std::runtime_error("expected numeric YAML value");
}

std::array<double, 3> readVector3(ros::NodeHandle* node_handle,
                                 const std::string& parameter,
                                 const std::array<double, 3>& fallback) {
  XmlRpc::XmlRpcValue value;
  if (!node_handle->getParam(parameter, value)) {
    return fallback;
  }
  if (value.getType() != XmlRpc::XmlRpcValue::TypeArray || value.size() != 3) {
    throw std::runtime_error(parameter + " must be a three-element list");
  }
  return {{xmlNumber(value[0]), xmlNumber(value[1]), xmlNumber(value[2])}};
}

struct Vehicle {
  std::string name;
  std::string mode{"gps_origin"};
  std::string origin_topic;
  std::string local_origin_frame;
  std::array<double, 3> position_offset{{0.0, 0.0, 0.0}};
  std::array<double, 3> fixed_position{{0.0, 0.0, 0.0}};
  double yaw_offset{0.0};
  double fixed_yaw{0.0};
  ros::Subscriber origin_subscriber;
  ros::Publisher valid_publisher;
  GeodeticPoint origin;
  geometry_msgs::TransformStamped transform;
  ros::Time last_update;
  bool valid{false};
};

}  // 匿名命名空间

class WorldTfManager {
 public:
  WorldTfManager() : private_nh_("~") {
    loadWorld();
    loadVehicles();
    diagnostics_publisher_ =
        private_nh_.advertise<diagnostic_msgs::DiagnosticArray>("diagnostics", 2);
    diagnostics_timer_ = private_nh_.createTimer(
        ros::Duration(0.5), &WorldTfManager::diagnosticsTimerCallback, this);
    publishAllTransforms();
    ROS_INFO("[xd_uav_world_tf_manager] world '%s' configured with %zu vehicle(s)",
             world_frame_.c_str(), vehicles_.size());
  }

 private:
  void loadWorld() {
    private_nh_.param("world/frame_id", world_frame_, std::string("world"));
    world_frame_ = trimSlashes(world_frame_);
    private_nh_.param("world/datum/latitude", world_datum_.latitude,
                      std::numeric_limits<double>::quiet_NaN());
    private_nh_.param("world/datum/longitude", world_datum_.longitude,
                      std::numeric_limits<double>::quiet_NaN());
    private_nh_.param("world/datum/altitude", world_datum_.altitude,
                      std::numeric_limits<double>::quiet_NaN());
    private_nh_.param("world/enu_to_world_yaw", enu_to_world_yaw_, 0.0);
    world_position_ = readVector3(&private_nh_, "world/position", {{0.0, 0.0, 0.0}});
    if (world_frame_.empty() || !validGeodetic(world_datum_)) {
      throw std::runtime_error("world frame or WGS84 datum is invalid");
    }
  }

  void loadVehicles() {
    std::vector<std::string> names;
    if (!private_nh_.getParam("vehicles", names) || names.empty()) {
      throw std::runtime_error("vehicles must contain at least one name");
    }
    std::unordered_map<std::string, std::string> child_owners;
    for (const std::string& name : names) {
      const std::string prefix = "vehicle_configs/" + name + "/";
      bool enabled = true;
      private_nh_.param(prefix + "enabled", enabled, true);
      if (!enabled) {
        continue;
      }
      auto vehicle = std::make_unique<Vehicle>();
      vehicle->name = name;
      private_nh_.param(prefix + "mode", vehicle->mode, std::string("gps_origin"));
      private_nh_.param(prefix + "origin_topic", vehicle->origin_topic,
                        "/" + name + "/mavros/global_position/gp_origin");
      private_nh_.param(prefix + "local_origin_frame", vehicle->local_origin_frame,
                        name + "/local_origin");
      vehicle->local_origin_frame = trimSlashes(vehicle->local_origin_frame);
      private_nh_.param(prefix + "yaw_offset", vehicle->yaw_offset, 0.0);
      private_nh_.param(prefix + "fixed_yaw", vehicle->fixed_yaw, 0.0);
      vehicle->position_offset =
          readVector3(&private_nh_, prefix + "position_offset", {{0.0, 0.0, 0.0}});
      vehicle->fixed_position =
          readVector3(&private_nh_, prefix + "fixed_position", {{0.0, 0.0, 0.0}});
      if (vehicle->local_origin_frame.empty()) {
        throw std::runtime_error("vehicle '" + name + "' has an empty local_origin_frame");
      }
      if (!child_owners.emplace(vehicle->local_origin_frame, name).second) {
        throw std::runtime_error("duplicate local_origin_frame: " +
                                 vehicle->local_origin_frame);
      }
      vehicle->valid_publisher =
          private_nh_.advertise<std_msgs::Bool>("vehicles/" + name + "/valid", 1, true);
      publishBoolean(vehicle->valid_publisher, false);
      Vehicle* pointer = vehicle.get();
      if (vehicle->mode == "gps_origin") {
        vehicle->origin_subscriber =
            nh_.subscribe<geographic_msgs::GeoPointStamped>(
                vehicle->origin_topic, 2,
                [this, pointer](const geographic_msgs::GeoPointStamped::ConstPtr& message) {
                  originCallback(pointer, message);
                });
      } else if (vehicle->mode == "fixed") {
        vehicle->transform =
            createTransform(*vehicle, vehicle->fixed_position, vehicle->fixed_yaw);
        vehicle->valid = true;
        vehicle->last_update = ros::Time::now();
        publishBoolean(vehicle->valid_publisher, true);
      } else if (vehicle->mode == "geodetic") {
        private_nh_.param(prefix + "origin/latitude", vehicle->origin.latitude,
                          std::numeric_limits<double>::quiet_NaN());
        private_nh_.param(prefix + "origin/longitude", vehicle->origin.longitude,
                          std::numeric_limits<double>::quiet_NaN());
        private_nh_.param(prefix + "origin/altitude", vehicle->origin.altitude,
                          std::numeric_limits<double>::quiet_NaN());
        if (!validGeodetic(vehicle->origin)) {
          throw std::runtime_error("vehicle '" + name + "' has invalid geodetic origin");
        }
        updateGeodeticVehicle(vehicle.get());
      } else {
        throw std::runtime_error("vehicle '" + name + "' has unsupported mode '" +
                                 vehicle->mode + "'");
      }
      vehicle_by_name_[name] = pointer;
      vehicles_.push_back(std::move(vehicle));
    }
    if (vehicles_.empty()) {
      throw std::runtime_error("all configured vehicles are disabled");
    }
  }

  geometry_msgs::TransformStamped createTransform(
      const Vehicle& vehicle, const std::array<double, 3>& position,
      const double yaw) const {
    geometry_msgs::TransformStamped transform;
    transform.header.stamp = ros::Time::now();
    transform.header.frame_id = world_frame_;
    transform.child_frame_id = vehicle.local_origin_frame;
    transform.transform.translation.x = position[0];
    transform.transform.translation.y = position[1];
    transform.transform.translation.z = position[2];
    tf2::Quaternion quaternion;
    quaternion.setRPY(0.0, 0.0, yaw);
    transform.transform.rotation = tf2::toMsg(quaternion);
    return transform;
  }

  void originCallback(Vehicle* vehicle,
                      const geographic_msgs::GeoPointStamped::ConstPtr& message) {
    const GeodeticPoint origin{message->position.latitude, message->position.longitude,
                               message->position.altitude};
    if (!validGeodetic(origin)) {
      ROS_WARN_THROTTLE(2.0, "[xd_uav_world_tf_manager] invalid origin for %s",
                        vehicle->name.c_str());
      return;
    }
    vehicle->origin = origin;
    updateGeodeticVehicle(vehicle);
    publishAllTransforms();
  }

  void updateGeodeticVehicle(Vehicle* vehicle) {
    const auto enu = geodeticToEnu(vehicle->origin, world_datum_);
    const double cosine = std::cos(enu_to_world_yaw_);
    const double sine = std::sin(enu_to_world_yaw_);
    const std::array<double, 3> world_position{{
        world_position_[0] + cosine * enu[0] - sine * enu[1] +
            vehicle->position_offset[0],
        world_position_[1] + sine * enu[0] + cosine * enu[1] +
            vehicle->position_offset[1],
        world_position_[2] + enu[2] + vehicle->position_offset[2]}};
    vehicle->transform = createTransform(
        *vehicle, world_position, enu_to_world_yaw_ + vehicle->yaw_offset);
    vehicle->valid = true;
    vehicle->last_update = ros::Time::now();
    publishBoolean(vehicle->valid_publisher, true);
  }

  void publishAllTransforms() {
    std::vector<geometry_msgs::TransformStamped> transforms;
    for (const auto& vehicle : vehicles_) {
      if (vehicle->valid) {
        geometry_msgs::TransformStamped transform = vehicle->transform;
        transform.header.stamp = ros::Time::now();
        transforms.push_back(transform);
      }
    }
    if (!transforms.empty()) {
      static_broadcaster_.sendTransform(transforms);
    }
  }

  void diagnosticsTimerCallback(const ros::TimerEvent&) {
    diagnostic_msgs::DiagnosticArray array;
    array.header.stamp = ros::Time::now();
    for (const auto& vehicle : vehicles_) {
      diagnostic_msgs::DiagnosticStatus status;
      status.name = "world_tf_manager/" + vehicle->name;
      status.hardware_id = vehicle->name;
      status.level = vehicle->valid ? diagnostic_msgs::DiagnosticStatus::OK
                                    : diagnostic_msgs::DiagnosticStatus::WARN;
      status.message = vehicle->valid ? "connected to world" : "waiting for origin";
      addValue(&status, "mode", vehicle->mode);
      addValue(&status, "child_frame", vehicle->local_origin_frame);
      addValue(&status, "origin_topic", vehicle->origin_topic);
      array.status.push_back(status);
    }
    diagnostics_publisher_.publish(array);
  }

  static void addValue(diagnostic_msgs::DiagnosticStatus* status,
                       const std::string& key, const std::string& value) {
    diagnostic_msgs::KeyValue item;
    item.key = key;
    item.value = value;
    status->values.push_back(item);
  }

  static void publishBoolean(const ros::Publisher& publisher, const bool value) {
    std_msgs::Bool message;
    message.data = value;
    publisher.publish(message);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  tf2_ros::StaticTransformBroadcaster static_broadcaster_;
  ros::Publisher diagnostics_publisher_;
  ros::Timer diagnostics_timer_;
  std::string world_frame_;
  GeodeticPoint world_datum_;
  std::array<double, 3> world_position_{{0.0, 0.0, 0.0}};
  double enu_to_world_yaw_{0.0};
  std::vector<std::unique_ptr<Vehicle>> vehicles_;
  std::unordered_map<std::string, Vehicle*> vehicle_by_name_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "world_tf_manager");
  try {
    WorldTfManager manager;
    ros::spin();
  } catch (const std::exception& exception) {
    ROS_FATAL("[xd_uav_world_tf_manager] startup failed: %s", exception.what());
    return 1;
  }
  return 0;
}
