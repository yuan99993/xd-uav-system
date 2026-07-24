#pragma once

#include <Eigen/Dense>
#include <mrs_lib/lkf.h>
#include <ros/time.h>

#include <memory>

namespace xd_uav_state_estimator {

struct FilterParameters {
  double initial_position_covariance{1.0};
  double initial_velocity_covariance{1.0};
  double initial_acceleration_covariance{10.0};

  double lateral_process_position{0.1};
  double lateral_process_velocity{1.0};
  double lateral_process_acceleration{1.0};
  double altitude_process_position{1.0};
  double altitude_process_velocity{1.0};
  double altitude_process_acceleration{1.0};

  double input_coefficient{10.0};
};

/**
 * MRS风格的平移状态估计器。
 *
 * 水平状态排列：[x, y, vx, vy, ax, ay]。
 * 高度状态排列：[z, vz, az]。
 */
class LinearFilters {
public:
  using LateralLkf = mrs_lib::LKF<6, 2, 2>;
  using AltitudeLkf = mrs_lib::LKF<3, 1, 1>;

  struct Snapshot {
    LateralLkf::statecov_t lateral_state;
    AltitudeLkf::statecov_t altitude_state;
    bool initialized{false};
  };

  explicit LinearFilters(const FilterParameters& parameters);

  void initialize(const Eigen::Vector3d& position, const Eigen::Vector3d& velocity,
                  const ros::Time& stamp);
  void reset();

  bool initialized() const;
  bool finite() const;

  void predict(double dt, const Eigen::Vector3d& control_acceleration,
               bool control_input_valid, const ros::Time& stamp);
  void correctPosition(const Eigen::Vector3d& position, double lateral_variance,
                       double altitude_variance, const ros::Time& stamp);
  void correctLateralPosition(const Eigen::Vector2d& position, double variance,
                              const ros::Time& stamp);
  void correctAltitudePosition(double position, double variance,
                               const ros::Time& stamp);
  void correctVelocity(const Eigen::Vector3d& velocity, double lateral_variance,
                       double altitude_variance, const ros::Time& stamp);
  void correctLateralVelocity(const Eigen::Vector2d& velocity, double variance,
                              const ros::Time& stamp);
  void correctVerticalVelocity(double velocity, double variance,
                               const ros::Time& stamp);

  void setPosition(const Eigen::Vector3d& position);
  void setLateralPosition(const Eigen::Vector2d& position);
  void setAltitudePosition(double position);

  Eigen::Vector3d position() const;
  Eigen::Vector3d velocity() const;
  Eigen::Vector3d acceleration() const;

  Eigen::Matrix3d positionCovariance() const;
  Eigen::Matrix3d velocityCovariance() const;
  Eigen::Matrix3d accelerationCovariance() const;

  ros::Time stamp() const;
  Snapshot snapshot() const;
  void restore(const Snapshot& snapshot);

private:
  static LateralLkf::A_t lateralA(double dt, double input_coefficient);
  static LateralLkf::B_t lateralB(double dt, double input_coefficient);
  static AltitudeLkf::A_t altitudeA(double dt, double input_coefficient);
  static AltitudeLkf::B_t altitudeB(double dt, double input_coefficient);

  void configureProcessNoise();

  FilterParameters parameters_;
  std::unique_ptr<LateralLkf> lateral_lkf_;
  std::unique_ptr<AltitudeLkf> altitude_lkf_;
  LateralLkf::statecov_t lateral_state_;
  AltitudeLkf::statecov_t altitude_state_;
  LateralLkf::Q_t lateral_q_{LateralLkf::Q_t::Zero()};
  AltitudeLkf::Q_t altitude_q_{AltitudeLkf::Q_t::Zero()};
  bool initialized_{false};
};

}  // 命名空间 xd_uav_state_estimator
