#pragma once

#include <algorithm>
#include <cmath>

namespace xd_uav_controller {

class VerticalTakeoffProfile {
 public:
  void reset(const double position) {
    start_position_ = position;
    position_ = position;
    velocity_ = 0.0;
    acceleration_ = 0.0;
    elapsed_ = 0.0;
  }

  void update(const double target, const double maximum_velocity,
              const double maximum_acceleration, const double dt) {
    if (!std::isfinite(target) || !std::isfinite(maximum_velocity) ||
        !std::isfinite(maximum_acceleration) || !std::isfinite(dt) ||
        maximum_velocity <= 0.0 || maximum_acceleration <= 0.0 ||
        dt <= 0.0) {
      acceleration_ = 0.0;
      return;
    }

    elapsed_ += dt;
    const double signed_distance = target - start_position_;
    const double direction = signed_distance >= 0.0 ? 1.0 : -1.0;
    const double distance = std::abs(signed_distance);
    if (distance <= 1e-12) {
      position_ = target;
      velocity_ = 0.0;
      acceleration_ = 0.0;
      return;
    }

    double acceleration_time = maximum_velocity / maximum_acceleration;
    double peak_velocity = maximum_velocity;
    double cruise_time = 0.0;
    if (distance <= maximum_velocity * maximum_velocity /
                        maximum_acceleration) {
      acceleration_time = std::sqrt(distance / maximum_acceleration);
      peak_velocity = maximum_acceleration * acceleration_time;
    } else {
      cruise_time =
          (distance - maximum_velocity * maximum_velocity /
                          maximum_acceleration) /
          maximum_velocity;
    }
    const double acceleration_distance =
        0.5 * maximum_acceleration * acceleration_time * acceleration_time;
    const double total_time = 2.0 * acceleration_time + cruise_time;

    double travelled = 0.0;
    if (elapsed_ < acceleration_time) {
      travelled = 0.5 * maximum_acceleration * elapsed_ * elapsed_;
      velocity_ = direction * maximum_acceleration * elapsed_;
      acceleration_ = direction * maximum_acceleration;
    } else if (elapsed_ < acceleration_time + cruise_time) {
      const double cruise_elapsed = elapsed_ - acceleration_time;
      travelled = acceleration_distance + peak_velocity * cruise_elapsed;
      velocity_ = direction * peak_velocity;
      acceleration_ = 0.0;
    } else if (elapsed_ < total_time) {
      const double deceleration_elapsed =
          elapsed_ - acceleration_time - cruise_time;
      travelled = acceleration_distance + peak_velocity * cruise_time +
                   peak_velocity * deceleration_elapsed -
                   0.5 * maximum_acceleration * deceleration_elapsed *
                       deceleration_elapsed;
      velocity_ = direction *
          (peak_velocity - maximum_acceleration * deceleration_elapsed);
      acceleration_ = -direction * maximum_acceleration;
    } else {
      travelled = distance;
      velocity_ = 0.0;
      acceleration_ = 0.0;
    }
    position_ = start_position_ + direction * travelled;
  }

  bool finished(const double target, const double position_tolerance,
                const double velocity_tolerance) const {
    return std::abs(target - position_) <= position_tolerance &&
           std::abs(velocity_) <= velocity_tolerance;
  }

  double position() const { return position_; }
  double velocity() const { return velocity_; }
  double acceleration() const { return acceleration_; }

 private:
  double start_position_{0.0};
  double position_{0.0};
  double velocity_{0.0};
  double acceleration_{0.0};
  double elapsed_{0.0};
};

}  // namespace xd_uav_controller
