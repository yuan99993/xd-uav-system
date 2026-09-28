#pragma once

#include <memory>
#include <string>

#include <Eigen/Core>
#include <GeographicLib/LocalCartesian.hpp>
#include <geographic_msgs/GeoPoint.h>

namespace xd_uav_detect {

class LocalCartesianProjector {
 public:
  LocalCartesianProjector() = default;

  bool reset(const geographic_msgs::GeoPoint& origin,
             std::string* error_message = nullptr);
  bool reverse(const Eigen::Vector3d& position_enu,
               geographic_msgs::GeoPoint* position,
               std::string* error_message = nullptr) const;
  bool initialized() const { return projector_ != nullptr; }
  const geographic_msgs::GeoPoint& origin() const { return origin_; }

  static bool validGeoPoint(const geographic_msgs::GeoPoint& point);

 private:
  geographic_msgs::GeoPoint origin_;
  std::unique_ptr<GeographicLib::LocalCartesian> projector_;
};

}  // namespace xd_uav_detect
