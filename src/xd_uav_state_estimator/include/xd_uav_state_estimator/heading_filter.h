#pragma once

#include <Eigen/Dense>
#include <ros/time.h>

namespace xd_uav_state_estimator {

struct HeadingFilterParameters {
  double initial_heading_covariance{0.25};
  double initial_rate_covariance{0.25};
  double heading_process_noise{0.01};
  double rate_process_noise{0.10};
};

/**
 * 独立的航向角与航向角速度滤波器。
 *
 * 状态排列为[yaw, yaw_rate]，所有角度均使用弧度。航向修正会对角度
 * 残差进行[-pi, pi)归一化，因此能够安全跨越正负pi边界。
 */
class HeadingFilter {
public:
  explicit HeadingFilter(const HeadingFilterParameters& parameters);

  void initialize(double heading, double rate, const ros::Time& stamp);
  void reset();
  void predict(const ros::Time& stamp);
  void correctHeading(double heading, double variance, const ros::Time& stamp);
  void correctRate(double rate, double variance, const ros::Time& stamp);

  bool initialized() const;
  bool finite() const;
  double heading() const;
  double rate() const;
  double headingVariance() const;
  double rateVariance() const;
  ros::Time stamp() const;

  static double wrapAngle(double angle);

private:
  void correctScalar(double measurement, double variance, int state_index,
                     bool angular_measurement, const ros::Time& stamp);

  HeadingFilterParameters parameters_;
  Eigen::Vector2d state_{Eigen::Vector2d::Zero()};
  Eigen::Matrix2d covariance_{Eigen::Matrix2d::Identity()};
  ros::Time stamp_;
  bool initialized_{false};
};

}  // 命名空间 xd_uav_state_estimator
