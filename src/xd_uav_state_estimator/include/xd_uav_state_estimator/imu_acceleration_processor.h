#pragma once

#include <Eigen/Dense>

#include <cstddef>

namespace xd_uav_state_estimator {

struct ImuAccelerationParameters {
  bool remove_gravity{true};
  Eigen::Vector3d gravity{0.0, 0.0, 9.80665};
  double acceleration_limit{30.0};

  bool estimate_bias{true};
  std::size_t bias_samples{100};
  double stationary_velocity_threshold{0.15};
  double stationary_angular_velocity_threshold{0.10};
  Eigen::Vector3d initial_bias{Eigen::Vector3d::Zero()};
};

/**
 * 将估计器里程计坐标系中表达的比力转换为完成重力补偿和偏置校正的线加速度。
 *
 * 偏置标定采用保守策略：只有平移速度和角速度连续低于配置阈值时才累计样本。
 * 标定成功前，update()返回false，使状态估计器安全退回自主预测模型。
 */
class ImuAccelerationProcessor {
public:
  explicit ImuAccelerationProcessor(const ImuAccelerationParameters& parameters);

  bool update(const Eigen::Vector3d& specific_force_odom,
              const Eigen::Vector3d& odometry_velocity,
              const Eigen::Vector3d& angular_velocity,
              Eigen::Vector3d* acceleration_odom);

  void resetBias();

  bool biasReady() const;
  bool calibrating() const;
  std::size_t biasSampleCount() const;
  std::size_t requiredBiasSamples() const;
  const Eigen::Vector3d& bias() const;
  const Eigen::Vector3d& latestAcceleration() const;

private:
  ImuAccelerationParameters parameters_;
  Eigen::Vector3d bias_sum_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d bias_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d latest_acceleration_{Eigen::Vector3d::Zero()};
  std::size_t bias_sample_count_{0};
  bool bias_ready_{false};
};

}  // 命名空间 xd_uav_state_estimator
