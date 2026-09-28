#include <xd_uav_detect/geodesy/local_cartesian_projector.hpp>

#include <cmath>
#include <exception>

namespace xd_uav_detect {
namespace {

void setError(const std::string& message, std::string* error_message) {
  if (error_message != nullptr) *error_message = message;
}

}  // namespace

bool LocalCartesianProjector::validGeoPoint(
    const geographic_msgs::GeoPoint& point) {
  return std::isfinite(point.latitude) && std::isfinite(point.longitude) &&
      std::isfinite(point.altitude) && point.latitude >= -90.0 &&
      point.latitude <= 90.0 && point.longitude >= -180.0 &&
      point.longitude <= 180.0;
}

bool LocalCartesianProjector::reset(
    const geographic_msgs::GeoPoint& origin, std::string* error_message) {
  if (!validGeoPoint(origin)) {
    projector_.reset();
    setError("invalid WGS84 origin", error_message);
    return false;
  }
  try {
    std::unique_ptr<GeographicLib::LocalCartesian> next(
        new GeographicLib::LocalCartesian(
            origin.latitude, origin.longitude, origin.altitude));
    origin_ = origin;
    projector_ = std::move(next);
    return true;
  } catch (const std::exception& error) {
    projector_.reset();
    setError(error.what(), error_message);
    return false;
  }
}

bool LocalCartesianProjector::reverse(
    const Eigen::Vector3d& position_enu,
    geographic_msgs::GeoPoint* position,
    std::string* error_message) const {
  if (position == nullptr) {
    setError("null geodetic output", error_message);
    return false;
  }
  if (!projector_) {
    setError("geographic origin unavailable", error_message);
    return false;
  }
  if (!position_enu.array().isFinite().all()) {
    setError("non-finite local ENU position", error_message);
    return false;
  }
  try {
    projector_->Reverse(position_enu.x(), position_enu.y(), position_enu.z(),
                        position->latitude, position->longitude,
                        position->altitude);
  } catch (const std::exception& error) {
    setError(error.what(), error_message);
    return false;
  }
  if (!validGeoPoint(*position)) {
    setError("projection produced invalid WGS84 position", error_message);
    return false;
  }
  return true;
}

}  // namespace xd_uav_detect
