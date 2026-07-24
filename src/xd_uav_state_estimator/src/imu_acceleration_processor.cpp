#include <xd_uav_state_estimator/imu_acceleration_processor.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace xd_uav_state_estimator {

ImuAccelerationProcessor::ImuAccelerationProcessor(
    const ImuAccelerationParameters& parameters)
    : parameters_(parameters), bias_(parameters.initial_bias) {
  parameters_.bias_samples = std::max<std::size_t>(parameters_.bias_samples, 1U);
  parameters_.stationary_velocity_threshold =
      std::max(parameters_.stationary_velocity_threshold, 0.0);
  parameters_.stationary_angular_velocity_threshold =
      std::max(parameters_.stationary_angular_velocity_threshold, 0.0);
  parameters_.acceleration_limit = std::max(parameters_.acceleration_limit, 0.1);

  if (!parameters_.gravity.allFinite() || !parameters_.initial_bias.allFinite()) {
    throw std::invalid_argument("IMU gravity and bias parameters must be finite");
  }

  bias_ready_ = !parameters_.estimate_bias;
}

bool ImuAccelerationProcessor::update(const Eigen::Vector3d& specific_force_odom,
                                      const Eigen::Vector3d& odometry_velocity,
                                      const Eigen::Vector3d& angular_velocity,
                                      Eigen::Vector3d* acceleration_odom) {
  if (acceleration_odom == nullptr || !specific_force_odom.allFinite() ||
      !odometry_velocity.allFinite() || !angular_velocity.allFinite()) {
    return false;
  }

  Eigen::Vector3d acceleration = specific_force_odom;
  if (parameters_.remove_gravity) {
    acceleration -= parameters_.gravity;
  }

  if (!acceleration.allFinite() || acceleration.norm() > parameters_.acceleration_limit) {
    return false;
  }

  if (!bias_ready_) {
    const bool stationary =
        odometry_velocity.norm() <= parameters_.stationary_velocity_threshold &&
        angular_velocity.norm() <= parameters_.stationary_angular_velocity_threshold;

    if (!stationary) {
      bias_sum_.setZero();
      bias_sample_count_ = 0;
      return false;
    }

    bias_sum_ += acceleration;
    ++bias_sample_count_;
    if (bias_sample_count_ < parameters_.bias_samples) {
      return false;
    }

    bias_ = bias_sum_ / static_cast<double>(bias_sample_count_);
    bias_ready_ = true;
  }

  latest_acceleration_ = acceleration - bias_;
  if (!latest_acceleration_.allFinite() ||
      latest_acceleration_.norm() > parameters_.acceleration_limit) {
    return false;
  }

  *acceleration_odom = latest_acceleration_;
  return true;
}

void ImuAccelerationProcessor::resetBias() {
  bias_sum_.setZero();
  bias_sample_count_ = 0;
  bias_ = parameters_.initial_bias;
  latest_acceleration_.setZero();
  bias_ready_ = !parameters_.estimate_bias;
}

bool ImuAccelerationProcessor::biasReady() const {
  return bias_ready_;
}

bool ImuAccelerationProcessor::calibrating() const {
  return parameters_.estimate_bias && !bias_ready_;
}

std::size_t ImuAccelerationProcessor::biasSampleCount() const {
  return bias_sample_count_;
}

std::size_t ImuAccelerationProcessor::requiredBiasSamples() const {
  return parameters_.bias_samples;
}

const Eigen::Vector3d& ImuAccelerationProcessor::bias() const {
  return bias_;
}

const Eigen::Vector3d& ImuAccelerationProcessor::latestAcceleration() const {
  return latest_acceleration_;
}

}  // 命名空间 xd_uav_state_estimator
