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
  throw std::invalid_argument("shape must be circle or rectangle_compact");
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

PathResult pathReferenceState(const Vec3& center,
                              const std::vector<Vec3>& input,
                              double speed, double lookahead, bool loop) {
  std::vector<Vec3> points = input;
  if (points.size() < 2 || !finite(speed) || !finite(lookahead) ||
      speed <= 0.0 || lookahead < 0.0) {
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
  double target_s = nearest_s + lookahead;
  if (loop) target_s = std::fmod(target_s, total);
  else target_s = std::min(target_s, total);
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
