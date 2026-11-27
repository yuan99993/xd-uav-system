#include <xd_uav_formation/core.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>

namespace xd_uav_formation {
namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;

bool finite(double value) { return std::isfinite(value); }

void requireFinite(const std::vector<double>& values, const char* message) {
  if (!std::all_of(values.begin(), values.end(), finite)) {
    throw std::invalid_argument(message);
  }
}

double clamp(double value, double low, double high) {
  return std::max(low, std::min(high, value));
}

Vec3 rotateOffset(double heading, const Vec3& offset) {
  const double c = std::cos(heading), s = std::sin(heading);
  return {c * offset[0] - s * offset[1],
          s * offset[0] + c * offset[1], offset[2]};
}

struct Segment {
  Vec3 start;
  Vec3 end;
  double dx;
  double dy;
  double length;
  double beginning;
};

}  // namespace

Vec3 validateOffset(const Vec3& offset, double minimum_distance) {
  if (!finite(minimum_distance) || minimum_distance <= 0.0) {
    throw std::invalid_argument("minimum distance must be positive and finite");
  }
  requireFinite({offset[0], offset[1], offset[2]}, "offset must be finite");
  if (std::sqrt(offset[0] * offset[0] + offset[1] * offset[1] +
                offset[2] * offset[2]) < minimum_distance) {
    throw std::invalid_argument("offset is shorter than minimum separation");
  }
  return offset;
}

std::vector<Vec3> formationOffsets(const std::string& shape,
                                   std::size_t follower_count,
                                   double spacing) {
  if (follower_count < 1 || !finite(spacing) || spacing <= 0.0) {
    throw std::invalid_argument("follower_count and spacing must be positive");
  }
  if (shape == "circle") {
    const double radius = std::max(
        spacing, spacing * static_cast<double>(follower_count) / (2.0 * kPi));
    std::vector<Vec3> slots;
    slots.reserve(follower_count);
    for (std::size_t i = 0; i < follower_count; ++i) {
      const double angle = 2.0 * kPi * static_cast<double>(i) /
                           static_cast<double>(follower_count);
      slots.push_back({radius * std::sin(angle), radius * std::cos(angle), 0.0});
    }
    return slots;
  }
  if (shape == "rectangle_compact") {
    const std::size_t columns = std::max<std::size_t>(
        1, static_cast<std::size_t>(std::sqrt(follower_count + 1)));
    const std::size_t rows =
        static_cast<std::size_t>(std::ceil(
            static_cast<double>(follower_count + 1) / columns));
    std::vector<Vec3> slots;
    slots.reserve(follower_count);
    for (std::size_t column = 0; column < columns; ++column) {
      for (std::size_t row = 0; row < rows; ++row) {
        if (column == 0 && row == 0) continue;
        slots.push_back({static_cast<double>(column) * spacing,
                         -static_cast<double>(row) * spacing, 0.0});
        if (slots.size() == follower_count) return slots;
      }
    }
  }
  if (shape == "line" || shape == "line_abreast") {
    const double center =
        0.5 * (static_cast<double>(follower_count) - 1.0);
    std::vector<Vec3> slots;
    slots.reserve(follower_count);
    for (std::size_t i = 0; i < follower_count; ++i) {
      slots.push_back({0.0,
                       (center - static_cast<double>(i)) * spacing,
                       0.0});
    }
    return slots;
  }
  if (shape == "column" || shape == "single_file") {
    const double center =
        0.5 * (static_cast<double>(follower_count) - 1.0);
    std::vector<Vec3> slots;
    slots.reserve(follower_count);
    for (std::size_t i = 0; i < follower_count; ++i) {
      slots.push_back({(center - static_cast<double>(i)) * spacing,
                       0.0, 0.0});
    }
    return slots;
  }
  if (shape == "arrow" || shape == "vee" || shape == "v_shape") {
    std::vector<Vec3> slots;
    slots.reserve(follower_count);
    slots.push_back({0.0, 0.0, 0.0});
    const double leg_step = spacing / std::sqrt(2.0);
    for (std::size_t i = 1; slots.size() < follower_count; ++i) {
      const double row = static_cast<double>((i + 1) / 2);
      const double side = i % 2 == 1 ? 1.0 : -1.0;
      slots.push_back({-row * leg_step, side * row * leg_step, 0.0});
    }
    Vec3 mean{0.0, 0.0, 0.0};
    for (const auto& slot : slots) {
      mean[0] += slot[0] / static_cast<double>(slots.size());
      mean[1] += slot[1] / static_cast<double>(slots.size());
    }
    for (auto& slot : slots) {
      slot[0] -= mean[0];
      slot[1] -= mean[1];
    }
    return slots;
  }
  throw std::invalid_argument(
      "shape must be circle, rectangle_compact, line, column, or arrow");
}

Slot slotTarget(const Vec3& position, double yaw, const Vec3& offset,
                double yaw_offset) {
  requireFinite({position[0], position[1], position[2], yaw,
                 offset[0], offset[1], offset[2], yaw_offset},
                "non-finite formation geometry");
  const double c = std::cos(yaw), s = std::sin(yaw);
  const double unwrapped = yaw + yaw_offset;
  return {position[0] + c * offset[0] - s * offset[1],
          position[1] + s * offset[0] + c * offset[1],
          position[2] + offset[2],
          std::atan2(std::sin(unwrapped), std::cos(unwrapped))};
}

Vec3 slotVelocity(const Vec3& velocity, const Vec3& acceleration,
                  double heading, const Vec3& offset,
                  double max_turn_rate, bool limit_turn_rate,
                  double requested_turn_rate, bool fixed_turn_rate) {
  requireFinite({velocity[0], velocity[1], velocity[2], acceleration[0],
                 acceleration[1], acceleration[2], heading, offset[0],
                 offset[1], offset[2]}, "non-finite formation velocity");
  if (limit_turn_rate && (!finite(max_turn_rate) || max_turn_rate <= 0.0)) {
    throw std::invalid_argument("max_turn_rate must be positive");
  }
  double turn_rate = requested_turn_rate;
  if (fixed_turn_rate) {
    if (!finite(turn_rate)) {
      throw std::invalid_argument("non-finite formation velocity");
    }
  } else {
    const double speed_squared = velocity[0] * velocity[0] +
                                 velocity[1] * velocity[1];
    turn_rate = speed_squared > 1e-6
        ? (velocity[0] * acceleration[1] - velocity[1] * acceleration[0]) /
              speed_squared
        : 0.0;
  }
  if (limit_turn_rate) {
    turn_rate = clamp(turn_rate, -max_turn_rate, max_turn_rate);
  }
  const Vec3 rotated = rotateOffset(heading, offset);
  return {velocity[0] - turn_rate * rotated[1],
          velocity[1] + turn_rate * rotated[0], velocity[2]};
}

FormationTurnPlan planFormationTurn(const std::vector<Vec3>& offsets,
    double cruise_speed, double minimum_speed, double maximum_speed,
    double requested_rate, double lateral_velocity) {
  requireFinite({cruise_speed, minimum_speed, maximum_speed, requested_rate,
      lateral_velocity}, "invalid formation turn plan");
  if (minimum_speed <= 0.0 || maximum_speed <= minimum_speed ||
      cruise_speed < minimum_speed || cruise_speed > maximum_speed)
    throw std::invalid_argument("invalid formation turn speed bounds");
  for (const auto& offset : offsets)
    requireFinite({offset[0], offset[1]}, "invalid formation turn offset");

  // Retain a feasible straight-flight fallback even for a large route error.
  const double lateral_limit = std::sqrt(std::max(0.0,
      maximum_speed * maximum_speed - cruise_speed * cruise_speed));
  lateral_velocity = clamp(lateral_velocity, -lateral_limit, lateral_limit);
  auto speedBounds = [&](double rate) {
    double lower = cruise_speed;
    double upper = std::sqrt(std::max(0.0,
        maximum_speed * maximum_speed - lateral_velocity * lateral_velocity));
    for (const auto& offset : offsets) {
      // Local slot velocity is (V - omega*y, lateral + omega*x).
      // Keeping its forward component >= minimum is conservative and avoids
      // satisfying the minimum by flying sideways or reversing direction.
      lower = std::max(lower, minimum_speed + rate * offset[1]);
      const double cross = lateral_velocity + rate * offset[0];
      if (std::abs(cross) > maximum_speed)
        return std::make_pair(lower, -1.0);
      upper = std::min(upper, rate * offset[1] + std::sqrt(std::max(0.0,
          maximum_speed * maximum_speed - cross * cross)));
    }
    return std::make_pair(lower, upper);
  };

  auto bounds = speedBounds(requested_rate);
  if (bounds.first > bounds.second) {
    // Feasible (speed, rate) pairs form a convex set. Along this rate ray,
    // zero is feasible and bisection finds the largest feasible turn rate.
    double low = 0.0, high = 1.0;
    for (int iteration = 0; iteration < 48; ++iteration) {
      const double middle = 0.5 * (low + high);
      const auto candidate = speedBounds(requested_rate * middle);
      if (candidate.first <= candidate.second) low = middle;
      else high = middle;
    }
    requested_rate *= low;
    bounds = speedBounds(requested_rate);
  }
  return {bounds.first, requested_rate, lateral_velocity};
}

TurnReference advanceTurnReference(const TurnReference& previous,
    double desired_heading, double desired_rate, double dt,
    double maximum_rate, double maximum_acceleration) {
  requireFinite({previous.heading, previous.rate, desired_heading, desired_rate,
      dt, maximum_rate, maximum_acceleration}, "invalid formation turn reference");
  if (dt <= 0.0 || maximum_rate <= 0.0 || maximum_acceleration <= 0.0)
    throw std::invalid_argument("formation turn limits and dt must be positive");
  const double error = std::atan2(std::sin(desired_heading - previous.heading),
                                  std::cos(desired_heading - previous.heading));
  const double direction = error >= 0.0 ? 1.0 : -1.0;
  const double step = maximum_acceleration * dt;
  const double relative_rate = direction * (previous.rate - desired_rate);
  // Bound the NEXT rate, including this cycle's trapezoidal heading travel
  // and the remaining braking distance. sqrt(2*a*error) alone starts braking
  // one cycle late because it ignores the travel before the next rate takes
  // effect. Work relative to the moving target's rate.
  const double stopping_rate = std::max(0.0, std::sqrt(std::max(0.0,
      2.0 * maximum_acceleration * std::abs(error) +
      0.25 * step * step - step * relative_rate)) - 0.5 * step);
  const double recovery = direction * std::min(2.0 * std::abs(error), stopping_rate);
  const double requested = clamp(desired_rate + recovery, -maximum_rate, maximum_rate);
  const double rate = clamp(requested,
      previous.rate - maximum_acceleration * dt,
      previous.rate + maximum_acceleration * dt);
  return {previous.heading + 0.5 * (previous.rate + rate) * dt,
          rate, (rate - previous.rate) / dt};
}

HorizontalMotion horizontalMotion(const Vec3& velocity, const Vec3& acceleration) {
  requireFinite({velocity[0], velocity[1], velocity[2], acceleration[0],
      acceleration[1], acceleration[2]}, "invalid horizontal motion");
  const double squared_speed = velocity[0] * velocity[0] + velocity[1] * velocity[1];
  return {velocity, acceleration, squared_speed > 0.25
      ? (velocity[0] * acceleration[1] - velocity[1] * acceleration[0]) / squared_speed
      : 0.0};
}

HorizontalMotion slewHorizontalCorrection(const Vec3& previous,
    const Vec3& desired, double dt, double maximum_acceleration) {
  requireFinite({previous[0], previous[1], desired[0], desired[1], dt,
      maximum_acceleration}, "invalid formation correction slew");
  if (dt <= 0.0 || maximum_acceleration <= 0.0)
    throw std::invalid_argument("correction slew and dt must be positive");
  const double dx = desired[0] - previous[0], dy = desired[1] - previous[1];
  const double length = std::hypot(dx, dy);
  const double scale = length > 0.0 ? std::min(1.0, maximum_acceleration * dt / length) : 0.0;
  return horizontalMotion({previous[0] + scale * dx, previous[1] + scale * dy, 0},
                           {scale * dx / dt, scale * dy / dt, 0});
}

HorizontalMotion limitHorizontalMotion(const HorizontalMotion& motion,
    double minimum_speed, double maximum_speed, double fallback_heading) {
  requireFinite({minimum_speed, maximum_speed, fallback_heading}, "invalid motion speed bounds");
  if (minimum_speed <= 0.0 || maximum_speed < minimum_speed)
    throw std::invalid_argument("motion speed bounds must be positive and ordered");
  horizontalMotion(motion.velocity, motion.acceleration);
  const double speed = std::hypot(motion.velocity[0], motion.velocity[1]);
  if (speed < 1e-6)
    return horizontalMotion({minimum_speed * std::cos(fallback_heading),
        minimum_speed * std::sin(fallback_heading), motion.velocity[2]}, {0,0,0});
  const double bounded = clamp(speed, minimum_speed, maximum_speed);
  if (bounded == speed) return motion;
  const double scale = bounded / speed;
  const double tx = motion.velocity[0] / speed, ty = motion.velocity[1] / speed;
  const double tangential = tx * motion.acceleration[0] + ty * motion.acceleration[1];
  // At a fixed speed bound only the normal acceleration remains. Preserve
  // the course-rate feed-forward instead of scaling velocity alone.
  return horizontalMotion({scale * motion.velocity[0], scale * motion.velocity[1], motion.velocity[2]},
      {scale * (motion.acceleration[0] - tx * tangential),
       scale * (motion.acceleration[1] - ty * tangential), 0});
}

HorizontalMotion turningSlotMotion(const Vec3& center_velocity,
    const Vec3& center_acceleration, double heading, const Vec3& offset,
    double turn_rate, double turn_acceleration) {
  requireFinite({turn_rate, turn_acceleration}, "invalid formation turn motion");
  const Vec3 velocity = slotVelocity(center_velocity, {0,0,0}, heading, offset,
                                     0.0, false, turn_rate, true);
  const Vec3 rotated = rotateOffset(heading, offset);
  const double omega_squared = turn_rate * turn_rate;
  const Vec3 acceleration{
      center_acceleration[0] - turn_acceleration * rotated[1] - omega_squared * rotated[0],
      center_acceleration[1] + turn_acceleration * rotated[0] - omega_squared * rotated[1],
      center_acceleration[2]};
  return horizontalMotion(velocity, acceleration);
}

Vec3 catchupVelocity(const Vec3& base_velocity, const Vec3& slot_position,
                     const Vec3& own_position, double gain,
                     double max_correction, bool limit_correction,
                     double max_horizontal_speed, bool limit_speed,
                     double min_horizontal_speed) {
  std::vector<double> values = {
      base_velocity[0], base_velocity[1], base_velocity[2],
      slot_position[0], slot_position[1], slot_position[2],
      own_position[0], own_position[1], own_position[2], gain,
      min_horizontal_speed};
  if (limit_correction) values.push_back(max_correction);
  if (limit_speed) values.push_back(max_horizontal_speed);
  requireFinite(values, "non-finite catchup input");
  if (gain < 0.0 || (limit_correction && max_correction < 0.0) ||
      (limit_speed && max_horizontal_speed <= 0.0) ||
      min_horizontal_speed < 0.0 ||
      (limit_speed && min_horizontal_speed > max_horizontal_speed)) {
    throw std::invalid_argument("invalid catchup speed limits");
  }
  const double ex = slot_position[0] - own_position[0];
  const double ey = slot_position[1] - own_position[1];
  double cx = gain * ex, cy = gain * ey;
  double correction_norm = std::hypot(cx, cy);
  if (!finite(correction_norm)) {
    throw std::invalid_argument("catchup correction overflow");
  }
  if (min_horizontal_speed > 0.0 && limit_correction) {
    const double base_speed = std::hypot(base_velocity[0], base_velocity[1]);
    if (base_speed < 1e-6) {
      throw std::invalid_argument("fixedwing reference has no course");
    }
    max_correction = std::min(max_correction, 0.5 * base_speed);
  }
  if (limit_correction && correction_norm > max_correction) {
    const double scale = max_correction / correction_norm;
    cx *= scale;
    cy *= scale;
  }
  double vx = base_velocity[0] + cx;
  double vy = base_velocity[1] + cy;
  double speed = std::hypot(vx, vy);
  if (limit_speed && speed > max_horizontal_speed) {
    const double scale = max_horizontal_speed / speed;
    vx *= scale;
    vy *= scale;
  } else if (speed < min_horizontal_speed) {
    if (speed < 1e-6) {
      vx = base_velocity[0];
      vy = base_velocity[1];
      speed = std::hypot(vx, vy);
    }
    if (speed < 1e-6) {
      throw std::invalid_argument("fixedwing reference has no course");
    }
    const double scale = min_horizontal_speed / speed;
    vx *= scale;
    vy *= scale;
  }
  return {vx, vy, base_velocity[2]};
}

Vec3 virtualCenter(const Vec3& position, double heading, const Vec3& offset) {
  const Slot slot = slotTarget({0.0, 0.0, 0.0}, heading, offset);
  return {position[0] - slot[0], position[1] - slot[1],
          position[2] - slot[2]};
}

std::pair<Vec3, double> sharedCenter(const std::vector<Vec3>& centers) {
  if (centers.empty()) {
    throw std::invalid_argument("formation centers must be finite XYZ points");
  }
  Vec3 mean{0.0, 0.0, 0.0};
  for (const auto& point : centers) {
    requireFinite({point[0], point[1], point[2]},
                  "formation centers must be finite XYZ points");
    for (std::size_t i = 0; i < 3; ++i) mean[i] += point[i];
  }
  for (double& value : mean) value /= static_cast<double>(centers.size());
  double largest_error = 0.0;
  for (const auto& point : centers) {
    largest_error = std::max(largest_error,
        std::hypot(point[0] - mean[0], point[1] - mean[1]));
  }
  return {mean, largest_error};
}

std::vector<Vec3> acquisitionSlots(const std::vector<Vec3>& positions,
                                   const std::vector<Vec3>& offsets,
                                   double heading, double altitude) {
  if (positions.empty() || positions.size() != offsets.size() ||
      !finite(heading) || !finite(altitude)) {
    throw std::invalid_argument("invalid acquisition positions or offsets");
  }
  Vec3 mean_position{0.0, 0.0, 0.0}, mean_offset{0.0, 0.0, 0.0};
  for (const auto& point : positions) {
    requireFinite({point[0], point[1], point[2]},
                  "invalid acquisition positions or offsets");
    for (std::size_t i = 0; i < 3; ++i) mean_position[i] += point[i];
  }
  std::vector<Vec3> rotated;
  rotated.reserve(offsets.size());
  for (const auto& offset : offsets) {
    requireFinite({offset[0], offset[1], offset[2]},
                  "invalid acquisition positions or offsets");
    const Vec3 slot = rotateOffset(heading, offset);
    rotated.push_back(slot);
    for (std::size_t i = 0; i < 3; ++i) mean_offset[i] += slot[i];
  }
  const double count = static_cast<double>(positions.size());
  for (std::size_t i = 0; i < 3; ++i) {
    mean_position[i] /= count;
    mean_offset[i] /= count;
  }
  const Vec3 anchor{mean_position[0] - mean_offset[0],
                    mean_position[1] - mean_offset[1], altitude};
  std::vector<Vec3> goals;
  goals.reserve(offsets.size());
  for (const auto& offset : offsets) {
    const Slot slot = slotTarget(anchor, heading, offset);
    goals.push_back({slot[0], slot[1], slot[2]});
  }
  return goals;
}

std::vector<std::size_t> minimumDistanceAssignment(
    const std::vector<Vec3>& positions, const std::vector<Vec3>& slots) {
  if (positions.size() != slots.size())
    throw std::invalid_argument("assignment requires equal aircraft and slot counts");
  const std::size_t n = positions.size();
  std::vector<std::vector<double>> costs(n, std::vector<double>(n));
  for (const auto& point : positions)
    requireFinite({point[0], point[1], point[2]}, "assignment positions must be finite");
  for (const auto& point : slots)
    requireFinite({point[0], point[1], point[2]}, "assignment slots must be finite");
  for (std::size_t i = 0; i < n; ++i) {
    for (std::size_t j = 0; j < n; ++j) {
      costs[i][j] = std::hypot(positions[i][0] - slots[j][0],
                               positions[i][1] - slots[j][1]);
      if (!finite(costs[i][j]))
        throw std::invalid_argument("assignment distance must be finite");
    }
  }
  // Shortest augmenting path form of the Hungarian algorithm, O(n^3).
  // Strict comparisons and ascending columns make equal-cost choices stable.
  std::vector<double> u(n + 1), v(n + 1);
  std::vector<std::size_t> matched(n + 1), previous(n + 1);
  for (std::size_t i = 1; i <= n; ++i) {
    matched[0] = i;
    std::size_t column = 0;
    std::vector<double> slack(n + 1, std::numeric_limits<double>::infinity());
    std::vector<bool> used(n + 1, false);
    do {
      used[column] = true;
      const std::size_t row = matched[column];
      double delta = std::numeric_limits<double>::infinity();
      std::size_t next = 0;
      for (std::size_t j = 1; j <= n; ++j) {
        if (used[j]) continue;
        const double reduced = costs[row - 1][j - 1] - u[row] - v[j];
        if (reduced < slack[j]) {
          slack[j] = reduced;
          previous[j] = column;
        }
        if (slack[j] < delta) {
          delta = slack[j];
          next = j;
        }
      }
      for (std::size_t j = 0; j <= n; ++j) {
        if (used[j]) { u[matched[j]] += delta; v[j] -= delta; }
        else slack[j] -= delta;
      }
      column = next;
    } while (matched[column] != 0);
    do {
      const std::size_t next = previous[column];
      matched[column] = matched[next];
      column = next;
    } while (column != 0);
  }
  std::vector<std::size_t> assignment(n);
  for (std::size_t j = 1; j <= n; ++j) assignment[matched[j] - 1] = j - 1;
  return assignment;
}

std::vector<std::string> ringNeighbors(
    const std::vector<std::string>& members, const std::string& own_name) {
  if (members.size() < 2 ||
      std::set<std::string>(members.begin(), members.end()).size() !=
          members.size()) {
    throw std::invalid_argument("ring requires at least two unique members");
  }
  const auto it = std::find(members.begin(), members.end(), own_name);
  if (it == members.end()) {
    throw std::invalid_argument("ring requires at least two unique members");
  }
  const std::size_t index = static_cast<std::size_t>(it - members.begin());
  std::vector<std::string> result;
  const std::string before = members[(index + members.size() - 1) % members.size()];
  const std::string after = members[(index + 1) % members.size()];
  result.push_back(before);
  if (after != before) result.push_back(after);
  return result;
}

std::vector<Vec3> roundedPath(const std::vector<Vec3>& input,
                              double radius, bool loop) {
  std::vector<Vec3> points = input;
  if (points.size() < 2 || !finite(radius) || radius < 0.0) {
    throw std::invalid_argument("path requires finite XYZ waypoints");
  }
  for (const auto& point : points) {
    requireFinite({point[0], point[1], point[2]},
                  "path requires finite XYZ waypoints");
  }
  if (radius == 0.0) return points;
  if (loop && points.back() == points.front()) points.pop_back();
  if (points.size() < 3) return points;
  std::vector<Vec3> result;
  for (std::size_t i = 0; i < points.size(); ++i) {
    const Vec3& point = points[i];
    if (!loop && (i == 0 || i == points.size() - 1)) {
      result.push_back(point);
      continue;
    }
    const Vec3& previous = points[(i + points.size() - 1) % points.size()];
    const Vec3& following = points[(i + 1) % points.size()];
    double ix = point[0] - previous[0], iy = point[1] - previous[1];
    double ox = following[0] - point[0], oy = following[1] - point[1];
    const double il = std::hypot(ix, iy), ol = std::hypot(ox, oy);
    if (std::min(il, ol) < 1e-3) {
      throw std::invalid_argument("path has a zero-length horizontal segment");
    }
    ix /= il; iy /= il; ox /= ol; oy /= ol;
    const double turn = std::atan2(ix * oy - iy * ox, ix * ox + iy * oy);
    if (std::abs(turn) < 1e-3) {
      result.push_back(point);
      continue;
    }
    if (std::abs(turn) > kPi - 1e-3) {
      throw std::invalid_argument("path has a U-turn that cannot be rounded");
    }
    const double tangent = std::tan(std::abs(turn) / 2.0);
    const double distance = std::min(
        radius * tangent, std::min(0.45 * il, 0.45 * ol));
    const double effective_radius = distance / tangent;
    const Vec3 entry{point[0] - ix * distance,
                     point[1] - iy * distance,
                     point[2] - (point[2] - previous[2]) * distance / il};
    const Vec3 exit{point[0] + ox * distance,
                    point[1] + oy * distance,
                    point[2] + (following[2] - point[2]) * distance / ol};
    const double direction = turn > 0.0 ? 1.0 : -1.0;
    const double cx = entry[0] - direction * iy * effective_radius;
    const double cy = entry[1] + direction * ix * effective_radius;
    const double first_angle = std::atan2(entry[1] - cy, entry[0] - cx);
    const std::size_t sections = std::max<std::size_t>(
        2, static_cast<std::size_t>(std::ceil(std::abs(turn) * effective_radius)));
    result.push_back(entry);
    for (std::size_t section = 1; section <= sections; ++section) {
      const double fraction = static_cast<double>(section) / sections;
      const double angle = first_angle + fraction * turn;
      result.push_back({cx + effective_radius * std::cos(angle),
                        cy + effective_radius * std::sin(angle),
                        entry[2] + fraction * (exit[2] - entry[2])});
    }
  }
  return result;
}

std::pair<double, double> projectPathProgress(
    const Vec3& position, const std::vector<Vec3>& input, bool loop) {
  std::vector<Vec3> points = input;
  if (points.size() < 2) {
    throw std::invalid_argument("path requires at least two XYZ waypoints");
  }
  requireFinite({position[0], position[1], position[2]},
                "invalid path projection position");
  for (const auto& point : points) {
    requireFinite({point[0], point[1], point[2]},
                  "path waypoint is non-finite");
  }
  if (loop && points.back() != points.front()) points.push_back(points.front());
  double total = 0.0;
  double nearest_squared = std::numeric_limits<double>::infinity();
  double nearest_s = 0.0;
  for (std::size_t i = 0; i + 1 < points.size(); ++i) {
    const Vec3& start = points[i];
    const Vec3& end = points[i + 1];
    const double dx = end[0] - start[0];
    const double dy = end[1] - start[1];
    const double length = std::hypot(dx, dy);
    if (length < 1e-3) {
      throw std::invalid_argument("path has a zero-length horizontal segment");
    }
    const double fraction = clamp(
        ((position[0] - start[0]) * dx +
         (position[1] - start[1]) * dy) /
            (length * length),
        0.0, 1.0);
    const double px = start[0] + fraction * dx;
    const double py = start[1] + fraction * dy;
    const double ex = position[0] - px;
    const double ey = position[1] - py;
    const double distance_squared = ex * ex + ey * ey;
    if (distance_squared < nearest_squared) {
      nearest_squared = distance_squared;
      nearest_s = total + fraction * length;
    }
    total += length;
  }
  return {nearest_s, total};
}

PathResult pathReferenceState(const Vec3& center,
                              const std::vector<Vec3>& input,
                              double speed, double lookahead, bool loop,
                              double progress_offset) {
  std::vector<Vec3> points = input;
  if (points.size() < 2 || !finite(speed) || !finite(lookahead) ||
      !finite(progress_offset) || speed <= 0.0 || lookahead < 0.0) {
    throw std::invalid_argument("invalid path speed, preview or center");
  }
  requireFinite({center[0], center[1], center[2]},
                "invalid path speed, preview or center");
  for (const auto& point : points) {
    requireFinite({point[0], point[1], point[2]},
                  "path waypoint is non-finite");
  }
  if (loop && points.back() != points.front()) points.push_back(points.front());
  std::vector<Segment> segments;
  double total = 0.0;
  for (std::size_t i = 0; i + 1 < points.size(); ++i) {
    const double dx = points[i + 1][0] - points[i][0];
    const double dy = points[i + 1][1] - points[i][1];
    const double length = std::hypot(dx, dy);
    if (length < 1e-3) {
      throw std::invalid_argument("path has a zero-length horizontal segment");
    }
    segments.push_back({points[i], points[i + 1], dx, dy, length, total});
    total += length;
  }
  double nearest_squared = std::numeric_limits<double>::infinity();
  double nearest_s = 0.0;
  for (const auto& segment : segments) {
    const double fraction = clamp(
        ((center[0] - segment.start[0]) * segment.dx +
         (center[1] - segment.start[1]) * segment.dy) /
            (segment.length * segment.length), 0.0, 1.0);
    const double px = segment.start[0] + fraction * segment.dx;
    const double py = segment.start[1] + fraction * segment.dy;
    const double dx = center[0] - px, dy = center[1] - py;
    const double distance_squared = dx * dx + dy * dy;
    if (distance_squared < nearest_squared) {
      nearest_squared = distance_squared;
      nearest_s = segment.beginning + fraction * segment.length;
    }
  }
  double target_s = nearest_s + lookahead + progress_offset;
  if (loop) {
    target_s = std::fmod(target_s, total);
    if (target_s < 0.0) target_s += total;
  } else {
    target_s = clamp(target_s, 0.0, total);
  }
  for (std::size_t i = 0; i < segments.size(); ++i) {
    const auto& segment = segments[i];
    if (target_s <= segment.beginning + segment.length ||
        i + 1 == segments.size()) {
      const double fraction = clamp(
          (target_s - segment.beginning) / segment.length, 0.0, 1.0);
      PathResult result;
      result.position = {
          segment.start[0] + fraction * segment.dx,
          segment.start[1] + fraction * segment.dy,
          segment.start[2] + fraction * (segment.end[2] - segment.start[2])};
      result.velocity = {speed * segment.dx / segment.length,
                         speed * segment.dy / segment.length,
                         speed * (segment.end[2] - segment.start[2]) /
                             segment.length};
      if (segments.size() < 2 || (!loop && i == 0)) {
        result.turn_rate = 0.0;
      } else {
        const auto& previous = segments[(i + segments.size() - 1) % segments.size()];
        const double cross = previous.dx * segment.dy - previous.dy * segment.dx;
        const double dot = previous.dx * segment.dx + previous.dy * segment.dy;
        const double heading_change = std::atan2(cross, dot);
        const double arc_length = 0.5 * (previous.length + segment.length);
        result.turn_rate = speed * heading_change / arc_length;
      }
      return result;
    }
  }
  throw std::logic_error("unreachable path segment");
}

Vec3 consensusVelocity(const Vec3& path_velocity,
                       const Vec3& desired_center,
                       const Vec3& own_center,
                       const std::vector<Vec3>& neighbor_centers,
                       double gain, double convergence_time,
                       double max_correction, bool limit_correction) {
  if (!finite(gain) || !finite(convergence_time) || gain < 0.0 ||
      convergence_time <= 0.0 || neighbor_centers.empty() ||
      (limit_correction && (!finite(max_correction) || max_correction < 0.0))) {
    throw std::invalid_argument("invalid consensus gains or neighbors");
  }
  Vec3 vx{path_velocity[0] +
              (desired_center[0] - own_center[0]) / convergence_time,
          path_velocity[1] +
              (desired_center[1] - own_center[1]) / convergence_time,
          path_velocity[2]};
  const double path_speed = std::hypot(path_velocity[0], path_velocity[1]);
  if (vx[0] * path_velocity[0] + vx[1] * path_velocity[1] < 0.0) {
    vx[0] = path_velocity[0];
    vx[1] = path_velocity[1];
  }
  const double magnitude = std::hypot(vx[0], vx[1]);
  if (magnitude > 1e-6) {
    vx[0] *= path_speed / magnitude;
    vx[1] *= path_speed / magnitude;
  }
  double cx = 0.0, cy = 0.0;
  for (const auto& center : neighbor_centers) {
    cx -= gain * (own_center[0] - center[0]);
    cy -= gain * (own_center[1] - center[1]);
  }
  const double correction_norm = std::hypot(cx, cy);
  if (limit_correction && correction_norm > max_correction &&
      correction_norm > 0.0) {
    const double scale = max_correction / correction_norm;
    cx *= scale;
    cy *= scale;
  }
  return {vx[0] + cx, vx[1] + cy, path_velocity[2]};
}

std::vector<double> consensusSpeedTargets(
    double path_speed, const std::vector<double>& progress_errors,
    const std::vector<double>& member_speeds, double gain,
    double max_speed_adjustment, double min_speed, double deadband) {
  if (progress_errors.empty() || progress_errors.size() != member_speeds.size() ||
      !finite(path_speed) || !finite(gain) || !finite(max_speed_adjustment) ||
      !finite(min_speed) || !finite(deadband) || path_speed <= 0.0 ||
      gain < 0.0 || max_speed_adjustment < 0.0 || min_speed < 0.0 ||
      deadband < 0.0) {
    throw std::invalid_argument("invalid consensus pacing inputs");
  }
  requireFinite(progress_errors, "invalid consensus pacing inputs");
  requireFinite(member_speeds, "invalid consensus pacing inputs");
  const double baseline = std::max(path_speed, min_speed);
  const double maximum_lag = std::max(
      0.0, *std::max_element(progress_errors.begin(), progress_errors.end()) -
               deadband);
  if (maximum_lag <= 0.0) {
    return std::vector<double>(progress_errors.size(), baseline);
  }
  const double adjustment =
      std::min(max_speed_adjustment, gain * maximum_lag);
  std::vector<double> targets;
  targets.reserve(progress_errors.size());
  for (std::size_t i = 0; i < progress_errors.size(); ++i) {
    const double error = progress_errors[i];
    const double speed = member_speeds[i];
    if (error > deadband) {
      targets.push_back(baseline + adjustment);
    } else if (error < -deadband) {
      const double pace = std::max(min_speed, baseline - adjustment);
      targets.push_back(std::max(min_speed, std::min(pace, speed)));
    } else {
      targets.push_back(baseline);
    }
  }
  return targets;
}

bool blockingAction(uint8_t action) {
  return action == 1 || action == 2 || action == 3 || action == 6 ||
         action == 7 || action == 8;
}

}  // namespace xd_uav_formation
