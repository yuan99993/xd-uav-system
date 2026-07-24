#include <xd_uav_state_estimator/heading_filter.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace xd_uav_state_estimator {

namespace {

constexpr double kPi = 3.14159265358979323846;

}  // 匿名命名空间

HeadingFilter::HeadingFilter(const HeadingFilterParameters& parameters)
    : parameters_(parameters) {
  reset();
}

void HeadingFilter::initialize(const double heading, const double rate,
                               const ros::Time& stamp) {
  if (!std::isfinite(heading) || !std::isfinite(rate)) {
    throw std::invalid_argument("cannot initialize heading filter with non-finite values");
  }
  state_ << wrapAngle(heading), rate;
  covariance_.setZero();
  covariance_(0, 0) = std::max(parameters_.initial_heading_covariance, 1e-12);
  covariance_(1, 1) = std::max(parameters_.initial_rate_covariance, 1e-12);
  stamp_ = stamp;
  initialized_ = true;
}

void HeadingFilter::reset() {
  state_.setZero();
  covariance_.setIdentity();
  stamp_ = ros::Time(0);
  initialized_ = false;
}

void HeadingFilter::predict(const ros::Time& stamp) {
  if (!initialized_) {
    throw std::logic_error("cannot predict an uninitialized heading filter");
  }
  const double dt = (stamp - stamp_).toSec();
  if (!std::isfinite(dt) || dt <= 0.0) {
    return;
  }

  Eigen::Matrix2d transition;
  transition << 1.0, dt,
                0.0, 1.0;
  Eigen::Matrix2d process_noise = Eigen::Matrix2d::Zero();
  process_noise(0, 0) = std::max(parameters_.heading_process_noise, 0.0) * dt;
  process_noise(1, 1) = std::max(parameters_.rate_process_noise, 0.0) * dt;
  state_ = transition * state_;
  state_(0) = wrapAngle(state_(0));
  covariance_ = transition * covariance_ * transition.transpose() + process_noise;
  covariance_ = 0.5 * (covariance_ + covariance_.transpose());
  stamp_ = stamp;
}

void HeadingFilter::correctHeading(const double heading, const double variance,
                                   const ros::Time& stamp) {
  correctScalar(heading, variance, 0, true, stamp);
}

void HeadingFilter::correctRate(const double rate, const double variance,
                                const ros::Time& stamp) {
  correctScalar(rate, variance, 1, false, stamp);
}

void HeadingFilter::correctScalar(const double measurement, const double variance,
                                  const int state_index,
                                  const bool angular_measurement,
                                  const ros::Time& stamp) {
  if (!initialized_) {
    throw std::logic_error("cannot correct an uninitialized heading filter");
  }
  if (!std::isfinite(measurement)) {
    throw std::invalid_argument("heading-filter measurement is not finite");
  }
  predict(stamp);

  const double measurement_variance = std::max(variance, 1e-12);
  const double innovation_covariance =
      covariance_(state_index, state_index) + measurement_variance;
  Eigen::Vector2d gain = covariance_.col(state_index) / innovation_covariance;
  double innovation = measurement - state_(state_index);
  if (angular_measurement) {
    innovation = wrapAngle(innovation);
  }
  state_ += gain * innovation;
  state_(0) = wrapAngle(state_(0));

  Eigen::RowVector2d observation = Eigen::RowVector2d::Zero();
  observation(state_index) = 1.0;
  const Eigen::Matrix2d identity = Eigen::Matrix2d::Identity();
  const Eigen::Matrix2d residual = identity - gain * observation;
  covariance_ = residual * covariance_ * residual.transpose() +
                gain * measurement_variance * gain.transpose();
  covariance_ = 0.5 * (covariance_ + covariance_.transpose());
  if (stamp > stamp_) {
    stamp_ = stamp;
  }
}

bool HeadingFilter::initialized() const {
  return initialized_;
}

bool HeadingFilter::finite() const {
  return state_.allFinite() && covariance_.allFinite();
}

double HeadingFilter::heading() const {
  return state_(0);
}

double HeadingFilter::rate() const {
  return state_(1);
}

double HeadingFilter::headingVariance() const {
  return covariance_(0, 0);
}

double HeadingFilter::rateVariance() const {
  return covariance_(1, 1);
}

ros::Time HeadingFilter::stamp() const {
  return stamp_;
}

double HeadingFilter::wrapAngle(const double angle) {
  if (!std::isfinite(angle)) {
    return angle;
  }
  double wrapped = std::fmod(angle + kPi, 2.0 * kPi);
  if (wrapped < 0.0) {
    wrapped += 2.0 * kPi;
  }
  return wrapped - kPi;
}

}  // 命名空间 xd_uav_state_estimator
