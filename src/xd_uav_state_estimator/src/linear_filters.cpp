#include <xd_uav_state_estimator/linear_filters.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace xd_uav_state_estimator {

namespace {

constexpr int kPosition = 0;
constexpr int kVelocity = 1;
constexpr int kAcceleration = 2;
constexpr int kX = 0;
constexpr int kY = 1;

int lateralIndex(const int state, const int axis) {
  return 2 * state + axis;
}

}  // 匿名命名空间

LinearFilters::LinearFilters(const FilterParameters& parameters) : parameters_(parameters) {
  const LateralLkf::H_t lat_h = LateralLkf::H_t::Zero();
  const AltitudeLkf::H_t alt_h = AltitudeLkf::H_t::Zero();
  lateral_lkf_ = std::make_unique<LateralLkf>(lateralA(0.01, 0.0), lateralB(0.01, 0.0), lat_h);
  altitude_lkf_ = std::make_unique<AltitudeLkf>(altitudeA(0.01, 0.0), altitudeB(0.01, 0.0), alt_h);
  configureProcessNoise();
  reset();
}

void LinearFilters::configureProcessNoise() {
  lateral_q_.setZero();
  lateral_q_(lateralIndex(kPosition, kX), lateralIndex(kPosition, kX)) = parameters_.lateral_process_position;
  lateral_q_(lateralIndex(kPosition, kY), lateralIndex(kPosition, kY)) = parameters_.lateral_process_position;
  lateral_q_(lateralIndex(kVelocity, kX), lateralIndex(kVelocity, kX)) = parameters_.lateral_process_velocity;
  lateral_q_(lateralIndex(kVelocity, kY), lateralIndex(kVelocity, kY)) = parameters_.lateral_process_velocity;
  lateral_q_(lateralIndex(kAcceleration, kX), lateralIndex(kAcceleration, kX)) = parameters_.lateral_process_acceleration;
  lateral_q_(lateralIndex(kAcceleration, kY), lateralIndex(kAcceleration, kY)) = parameters_.lateral_process_acceleration;

  altitude_q_.setZero();
  altitude_q_(kPosition, kPosition) = parameters_.altitude_process_position;
  altitude_q_(kVelocity, kVelocity) = parameters_.altitude_process_velocity;
  altitude_q_(kAcceleration, kAcceleration) = parameters_.altitude_process_acceleration;
}

void LinearFilters::initialize(const Eigen::Vector3d& position, const Eigen::Vector3d& velocity,
                               const ros::Time& stamp) {
  lateral_state_.x << position.x(), position.y(), velocity.x(), velocity.y(), 0.0, 0.0;
  lateral_state_.P.setZero();
  lateral_state_.P(lateralIndex(kPosition, kX), lateralIndex(kPosition, kX)) = parameters_.initial_position_covariance;
  lateral_state_.P(lateralIndex(kPosition, kY), lateralIndex(kPosition, kY)) = parameters_.initial_position_covariance;
  lateral_state_.P(lateralIndex(kVelocity, kX), lateralIndex(kVelocity, kX)) = parameters_.initial_velocity_covariance;
  lateral_state_.P(lateralIndex(kVelocity, kY), lateralIndex(kVelocity, kY)) = parameters_.initial_velocity_covariance;
  lateral_state_.P(lateralIndex(kAcceleration, kX), lateralIndex(kAcceleration, kX)) = parameters_.initial_acceleration_covariance;
  lateral_state_.P(lateralIndex(kAcceleration, kY), lateralIndex(kAcceleration, kY)) = parameters_.initial_acceleration_covariance;
  lateral_state_.stamp = stamp;

  altitude_state_.x << position.z(), velocity.z(), 0.0;
  altitude_state_.P.setZero();
  altitude_state_.P(kPosition, kPosition) = parameters_.initial_position_covariance;
  altitude_state_.P(kVelocity, kVelocity) = parameters_.initial_velocity_covariance;
  altitude_state_.P(kAcceleration, kAcceleration) = parameters_.initial_acceleration_covariance;
  altitude_state_.stamp = stamp;

  initialized_ = true;
}

void LinearFilters::reset() {
  lateral_state_.x.setZero();
  lateral_state_.P.setIdentity();
  lateral_state_.stamp = ros::Time(0);
  altitude_state_.x.setZero();
  altitude_state_.P.setIdentity();
  altitude_state_.stamp = ros::Time(0);
  initialized_ = false;
}

bool LinearFilters::initialized() const {
  return initialized_;
}

bool LinearFilters::finite() const {
  return lateral_state_.x.allFinite() && lateral_state_.P.allFinite() &&
         altitude_state_.x.allFinite() && altitude_state_.P.allFinite();
}

void LinearFilters::predict(const double dt, const Eigen::Vector3d& control_acceleration,
                            const bool control_input_valid, const ros::Time& stamp) {
  if (!initialized_) {
    throw std::logic_error("cannot predict an uninitialized filter");
  }
  if (!std::isfinite(dt) || dt <= 0.0) {
    return;
  }

  const double coefficient = control_input_valid ? parameters_.input_coefficient : 0.0;
  lateral_lkf_->A = lateralA(dt, coefficient);
  lateral_lkf_->B = lateralB(dt, coefficient);
  altitude_lkf_->A = altitudeA(dt, coefficient);
  altitude_lkf_->B = altitudeB(dt, coefficient);

  LateralLkf::u_t lateral_input;
  lateral_input << control_acceleration.x(), control_acceleration.y();
  AltitudeLkf::u_t altitude_input;
  altitude_input << control_acceleration.z();

  lateral_state_ = lateral_lkf_->predict(lateral_state_, lateral_input, lateral_q_, dt);
  altitude_state_ = altitude_lkf_->predict(altitude_state_, altitude_input, altitude_q_, dt);
  lateral_state_.stamp = stamp;
  altitude_state_.stamp = stamp;
}

void LinearFilters::correctPosition(const Eigen::Vector3d& position, const double lateral_variance,
                                    const double altitude_variance, const ros::Time& stamp) {
  correctLateralPosition(position.head<2>(), lateral_variance, stamp);
  correctAltitudePosition(position.z(), altitude_variance, stamp);
}

void LinearFilters::correctLateralPosition(const Eigen::Vector2d& position,
                                           const double variance,
                                           const ros::Time& stamp) {
  if (!initialized_) {
    throw std::logic_error("cannot correct an uninitialized filter");
  }

  LateralLkf::H_t lat_h = LateralLkf::H_t::Zero();
  lat_h(kX, lateralIndex(kPosition, kX)) = 1.0;
  lat_h(kY, lateralIndex(kPosition, kY)) = 1.0;
  lateral_lkf_->H = lat_h;
  LateralLkf::z_t lat_z;
  lat_z << position.x(), position.y();
  const auto lat_r = std::max(variance, 1e-12) * LateralLkf::R_t::Identity();
  lateral_state_ = lateral_lkf_->correct(lateral_state_, lat_z, lat_r);

  lateral_state_.stamp = stamp;
}

void LinearFilters::correctAltitudePosition(const double position,
                                            const double variance,
                                            const ros::Time& stamp) {
  if (!initialized_) {
    throw std::logic_error("cannot correct an uninitialized filter");
  }

  AltitudeLkf::H_t alt_h = AltitudeLkf::H_t::Zero();
  alt_h(0, kPosition) = 1.0;
  altitude_lkf_->H = alt_h;
  AltitudeLkf::z_t alt_z;
  alt_z << position;
  AltitudeLkf::R_t alt_r;
  alt_r << std::max(variance, 1e-12);
  altitude_state_ = altitude_lkf_->correct(altitude_state_, alt_z, alt_r);

  altitude_state_.stamp = stamp;
}

void LinearFilters::correctVelocity(const Eigen::Vector3d& velocity, const double lateral_variance,
                                    const double altitude_variance, const ros::Time& stamp) {
  correctLateralVelocity(velocity.head<2>(), lateral_variance, stamp);
  correctVerticalVelocity(velocity.z(), altitude_variance, stamp);
}

void LinearFilters::correctLateralVelocity(const Eigen::Vector2d& velocity,
                                           const double variance,
                                           const ros::Time& stamp) {
  if (!initialized_) {
    throw std::logic_error("cannot correct an uninitialized filter");
  }

  LateralLkf::H_t lat_h = LateralLkf::H_t::Zero();
  lat_h(kX, lateralIndex(kVelocity, kX)) = 1.0;
  lat_h(kY, lateralIndex(kVelocity, kY)) = 1.0;
  lateral_lkf_->H = lat_h;
  LateralLkf::z_t lat_z;
  lat_z << velocity.x(), velocity.y();
  const auto lat_r = std::max(variance, 1e-12) * LateralLkf::R_t::Identity();
  lateral_state_ = lateral_lkf_->correct(lateral_state_, lat_z, lat_r);

  lateral_state_.stamp = stamp;
}

void LinearFilters::correctVerticalVelocity(const double velocity,
                                            const double variance,
                                            const ros::Time& stamp) {
  if (!initialized_) {
    throw std::logic_error("cannot correct an uninitialized filter");
  }

  AltitudeLkf::H_t alt_h = AltitudeLkf::H_t::Zero();
  alt_h(0, kVelocity) = 1.0;
  altitude_lkf_->H = alt_h;
  AltitudeLkf::z_t alt_z;
  alt_z << velocity;
  AltitudeLkf::R_t alt_r;
  alt_r << std::max(variance, 1e-12);
  altitude_state_ = altitude_lkf_->correct(altitude_state_, alt_z, alt_r);

  altitude_state_.stamp = stamp;
}

void LinearFilters::setPosition(const Eigen::Vector3d& position) {
  lateral_state_.x(lateralIndex(kPosition, kX)) = position.x();
  lateral_state_.x(lateralIndex(kPosition, kY)) = position.y();
  altitude_state_.x(kPosition) = position.z();
}

void LinearFilters::setLateralPosition(const Eigen::Vector2d& position) {
  lateral_state_.x(lateralIndex(kPosition, kX)) = position.x();
  lateral_state_.x(lateralIndex(kPosition, kY)) = position.y();
}

void LinearFilters::setAltitudePosition(const double position) {
  altitude_state_.x(kPosition) = position;
}

Eigen::Vector3d LinearFilters::position() const {
  return {lateral_state_.x(lateralIndex(kPosition, kX)),
          lateral_state_.x(lateralIndex(kPosition, kY)), altitude_state_.x(kPosition)};
}

Eigen::Vector3d LinearFilters::velocity() const {
  return {lateral_state_.x(lateralIndex(kVelocity, kX)),
          lateral_state_.x(lateralIndex(kVelocity, kY)), altitude_state_.x(kVelocity)};
}

Eigen::Vector3d LinearFilters::acceleration() const {
  return {lateral_state_.x(lateralIndex(kAcceleration, kX)),
          lateral_state_.x(lateralIndex(kAcceleration, kY)), altitude_state_.x(kAcceleration)};
}

Eigen::Matrix3d LinearFilters::positionCovariance() const {
  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
  covariance(0, 0) = lateral_state_.P(lateralIndex(kPosition, kX), lateralIndex(kPosition, kX));
  covariance(1, 1) = lateral_state_.P(lateralIndex(kPosition, kY), lateralIndex(kPosition, kY));
  covariance(2, 2) = altitude_state_.P(kPosition, kPosition);
  return covariance;
}

Eigen::Matrix3d LinearFilters::velocityCovariance() const {
  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
  covariance(0, 0) = lateral_state_.P(lateralIndex(kVelocity, kX), lateralIndex(kVelocity, kX));
  covariance(1, 1) = lateral_state_.P(lateralIndex(kVelocity, kY), lateralIndex(kVelocity, kY));
  covariance(2, 2) = altitude_state_.P(kVelocity, kVelocity);
  return covariance;
}

Eigen::Matrix3d LinearFilters::accelerationCovariance() const {
  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
  covariance(0, 0) = lateral_state_.P(lateralIndex(kAcceleration, kX), lateralIndex(kAcceleration, kX));
  covariance(1, 1) = lateral_state_.P(lateralIndex(kAcceleration, kY), lateralIndex(kAcceleration, kY));
  covariance(2, 2) = altitude_state_.P(kAcceleration, kAcceleration);
  return covariance;
}

ros::Time LinearFilters::stamp() const {
  return lateral_state_.stamp;
}

LinearFilters::Snapshot LinearFilters::snapshot() const {
  Snapshot result;
  result.lateral_state = lateral_state_;
  result.altitude_state = altitude_state_;
  result.initialized = initialized_;
  return result;
}

void LinearFilters::restore(const Snapshot& snapshot) {
  lateral_state_ = snapshot.lateral_state;
  altitude_state_ = snapshot.altitude_state;
  initialized_ = snapshot.initialized;
}

LinearFilters::LateralLkf::A_t LinearFilters::lateralA(const double dt, const double input_coefficient) {
  LateralLkf::A_t matrix;
  matrix << 1.0, 0.0, dt, 0.0, 0.5 * dt * dt, 0.0,
            0.0, 1.0, 0.0, dt, 0.0, 0.5 * dt * dt,
            0.0, 0.0, 1.0, 0.0, dt, 0.0,
            0.0, 0.0, 0.0, 1.0, 0.0, dt,
            0.0, 0.0, 0.0, 0.0, 1.0 - input_coefficient * dt, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 1.0 - input_coefficient * dt;
  return matrix;
}

LinearFilters::LateralLkf::B_t LinearFilters::lateralB(const double dt, const double input_coefficient) {
  LateralLkf::B_t matrix = LateralLkf::B_t::Zero();
  matrix(lateralIndex(kAcceleration, kX), kX) = input_coefficient * dt;
  matrix(lateralIndex(kAcceleration, kY), kY) = input_coefficient * dt;
  return matrix;
}

LinearFilters::AltitudeLkf::A_t LinearFilters::altitudeA(const double dt, const double input_coefficient) {
  AltitudeLkf::A_t matrix;
  matrix << 1.0, dt, 0.5 * dt * dt,
            0.0, 1.0, dt,
            0.0, 0.0, 1.0 - input_coefficient * dt;
  return matrix;
}

LinearFilters::AltitudeLkf::B_t LinearFilters::altitudeB(const double dt, const double input_coefficient) {
  AltitudeLkf::B_t matrix;
  matrix << 0.0, 0.0, input_coefficient * dt;
  return matrix;
}

}  // 命名空间 xd_uav_state_estimator
