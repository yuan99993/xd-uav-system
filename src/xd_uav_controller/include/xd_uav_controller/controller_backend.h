#pragma once

#include <string>

#include <Eigen/Core>
#include <ros/node_handle.h>
#include <ros/time.h>

#include <xd_uav_controller/ControlState.h>
#include <xd_uav_controller/control_types.h>
#include <xd_uav_controller/unified_reference.h>

namespace xd_uav_controller {

struct BackendOutput {
  Eigen::Vector3d body_rate{Eigen::Vector3d::Zero()};
  double thrust{0.0};
  bool valid{false};
  std::string reason;
};

class ControllerBackend {
 public:
  virtual ~ControllerBackend() = default;

  virtual BackendId id() const = 0;
  virtual bool configure(const ros::NodeHandle& node_handle) = 0;
  virtual bool supportsRegime(FlightRegime regime) const = 0;
  virtual bool supportsReference(ReferenceType type) const = 0;
  virtual void onActivate(const ControlState& state,
                          const UnifiedReference& reference) = 0;
  virtual void onDeactivate() = 0;
  virtual void reset() = 0;
  virtual BackendOutput update(const ControlState& state,
                               const UnifiedReference& reference,
                               const ros::Time& now,
                               double dt) = 0;
};

}  // namespace xd_uav_controller
