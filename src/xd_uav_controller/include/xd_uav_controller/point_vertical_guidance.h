#pragma once

#include <algorithm>
#include <cmath>

#include <Eigen/Core>
#include <xd_uav_controller/unified_reference.h>

namespace xd_uav_controller {

// Pure position goals have no prescribed vertical speed. Explicit velocity
// fields (including enabled zeroes) must keep their existing semantics.
inline bool usesPointVerticalGuidance(const UnifiedReference& reference) {
  if (reference.path_reference || reference.trajectory_reference ||
      (reference.type != ReferenceType::kPositionTarget &&
       reference.type != ReferenceType::kSimpleGoal) ||
      !reference.use_position[2] ||
      !(reference.use_position[0] || reference.use_position[1]) ||
      reference.use_yaw_rate || reference.use_jerk) {
    return false;
  }
  for (int axis = 0; axis < 3; ++axis) {
    if (reference.use_velocity[axis] || reference.use_acceleration[axis]) {
      return false;
    }
  }
  return true;
}

struct PointVerticalConfig {
  double maximum_climb_angle{0.174533};  // radians, 10 degrees
  double maximum_descent_angle{0.209440};  // radians, 12 degrees
  double maximum_climb_rate{3.0};
  double maximum_descent_rate{4.0};
  double acceleration_limit{0.8};
  double altitude_time_constant{5.0};

  bool valid() const {
    return positive(maximum_climb_angle) && maximum_climb_angle < 1.0 &&
           positive(maximum_descent_angle) && maximum_descent_angle < 1.0 &&
           positive(maximum_climb_rate) && positive(maximum_descent_rate) &&
           positive(acceleration_limit) && positive(altitude_time_constant);
  }

 private:
  static bool positive(double value) {
    return std::isfinite(value) && value > 0.0;
  }
};

struct PointVerticalOutput {
  double climb_rate{0.0};
  bool orbit{false};
  double course{0.0};
  double course_rate{0.0};
};

class PointVerticalGuidance {
 public:
  void reset() { initialized_ = false; }

  PointVerticalOutput update(
      const PointVerticalConfig& config, const Eigen::Vector3d& position,
      const Eigen::Vector3d& velocity, const Eigen::Vector3d& goal,
      double course, double airspeed, double commanded_speed,
      double global_rate_limit, double orbit_radius, int orbit_direction,
      double maximum_roll, double altitude_tolerance, double dt,
      double altitude_integral_gain = 0.02,
      double altitude_integral_limit = 0.5) {
    const double up_limit = std::min(
        {global_rate_limit, config.maximum_climb_rate,
         airspeed * std::sin(config.maximum_climb_angle)});
    const double down_limit = std::min(
        {global_rate_limit, config.maximum_descent_rate,
         airspeed * std::sin(config.maximum_descent_angle)});
    const double direction = orbit_direction < 0 ? -1.0 : 1.0;
    if (!initialized_) {
      origin_ = position;
      progress_ = 0.0;
      climb_rate_ = velocity.z();
      altitude_integral_ = 0.0;
      const double distance = (goal - position).head<2>().norm();
      const double height = goal.z() - position.z();
      const double angle = height >= 0.0 ? config.maximum_climb_angle
                                         : config.maximum_descent_angle;
      const double rate = height >= 0.0 ? up_limit : down_limit;
      const double slope_limit = std::min(
          std::tan(angle), rate / std::max(1.0, commanded_speed));
      // Reserve distance for entry/exit acceleration, not just the constant
      // slope. A nearby high/low goal first needs an altitude capture orbit.
      orbit_ = std::abs(height) > altitude_tolerance &&
               std::abs(height) > 0.8 * slope_limit * distance;
      const double turn_speed = std::max(airspeed, commanded_speed);
      radius_ = std::max(
          orbit_radius,
          turn_speed * turn_speed /
              (9.80665 * std::tan(std::max(0.05, 0.8 * maximum_roll))));
      centre_ = position.head<2>() + direction * radius_ *
          Eigen::Vector2d(-std::sin(course), std::cos(course));
      initialized_ = true;
    }

    const double height_error = goal.z() - position.z();
    if (orbit_ && std::abs(height_error) <= altitude_tolerance &&
        std::abs(velocity.z()) <= 0.3 && std::abs(climb_rate_) <= 0.3) {
      orbit_ = false;
      origin_ = position;
      progress_ = 0.0;
    }

    PointVerticalOutput output;
    output.orbit = orbit_;
    double feedback_error = height_error;
    double requested = height_error / config.altitude_time_constant;
    if (orbit_) {
      const Eigen::Vector2d radial = position.head<2>() - centre_;
      const double radial_error = (radial.norm() - radius_) / radius_;
      output.course = std::atan2(radial.y(), radial.x()) + direction *
          (1.5707963267948966 + std::atan(radial_error));
      output.course_rate = direction * commanded_speed / radius_;
    } else {
      const Eigen::Vector2d leg = (goal - origin_).head<2>();
      const double length = leg.norm();
      if (length > 1.0) {
        const Eigen::Vector2d tangent = leg / length;
        progress_ = std::max(progress_, bound(
            (position.head<2>() - origin_.head<2>()).dot(tangent) / length,
            0.0, 1.0));
        const double height_delta = goal.z() - origin_.z();
        const double reference_height = origin_.z() + progress_ * height_delta;
        feedback_error = reference_height - position.z();
        const double along_speed = std::max(0.0, velocity.head<2>().dot(tangent));
        const double feedforward = progress_ < 1.0
            ? height_delta * along_speed / length : 0.0;
        requested = feedforward +
            feedback_error / config.altitude_time_constant;
      }
    }

    // Small trim/model errors must not leave the capture orbit permanently
    // outside its altitude tolerance. Integrate the shaped height error,
    // with clamping at the actual point-rate envelope.
    const double integral_delta = altitude_integral_gain * feedback_error * dt;
    const double candidate_integral = bound(
        altitude_integral_ + integral_delta,
        -altitude_integral_limit, altitude_integral_limit);
    const double candidate_rate = requested + candidate_integral;
    if (!((candidate_rate > up_limit && integral_delta > 0.0) ||
          (candidate_rate < -down_limit && integral_delta < 0.0))) {
      altitude_integral_ = candidate_integral;
    }
    requested += altitude_integral_;
    requested = bound(requested, -down_limit, up_limit);
    // Leave enough height to arrest the vertical motion before the endpoint.
    const double stopping_rate = std::sqrt(
        2.0 * config.acceleration_limit * std::abs(height_error));
    if (requested * height_error > 0.0) {
      requested = std::copysign(std::min(std::abs(requested), stopping_rate),
                                requested);
    }
    const double step = config.acceleration_limit * bound(dt, 0.001, 0.1);
    climb_rate_ += bound(requested - climb_rate_, -step, step);
    output.climb_rate = climb_rate_;
    return output;
  }

 private:
  static double bound(double value, double low, double high) {
    return std::max(low, std::min(high, value));
  }
  bool initialized_{false};
  bool orbit_{false};
  Eigen::Vector3d origin_{Eigen::Vector3d::Zero()};
  Eigen::Vector2d centre_{Eigen::Vector2d::Zero()};
  double progress_{0.0};
  double climb_rate_{0.0};
  double altitude_integral_{0.0};
  double radius_{100.0};
};

}  // namespace xd_uav_controller
