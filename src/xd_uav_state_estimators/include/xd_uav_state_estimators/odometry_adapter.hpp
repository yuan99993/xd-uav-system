#pragma once

#include <memory>
#include <string>

#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <xmlrpcpp/XmlRpcValue.h>

namespace xd_uav_state_estimators {

// Owns the adapters declared in sources.yaml and keeps their public output
// topics unchanged. It can run in the estimator process or standalone.
class OdometryAdapterCollection {
 public:
  OdometryAdapterCollection(const ros::NodeHandle& node_handle,
                            tf2_ros::Buffer* tf_buffer,
                            const std::string& uav_name,
                            const XmlRpc::XmlRpcValue& adapter_configs);
  ~OdometryAdapterCollection();

  OdometryAdapterCollection(const OdometryAdapterCollection&) = delete;
  OdometryAdapterCollection& operator=(const OdometryAdapterCollection&) = delete;

  bool requiresTransform() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace xd_uav_state_estimators
