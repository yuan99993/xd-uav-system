#include <xd_uav_track/tracklet_stitcher.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace xd_uav_track {
namespace {

bool finiteVector(const std::array<double, 3>& values) {
  return std::all_of(values.begin(), values.end(),
                     [](const double value) { return std::isfinite(value); });
}

double cosineSimilarity(const std::vector<float>& lhs,
                        const std::vector<float>& rhs) {
  if (lhs.empty() || lhs.size() != rhs.size()) return -1.0;
  double dot = 0.0;
  double lhs_norm = 0.0;
  double rhs_norm = 0.0;
  for (std::size_t index = 0; index < lhs.size(); ++index) {
    const double a = lhs[index];
    const double b = rhs[index];
    if (!std::isfinite(a) || !std::isfinite(b)) return -1.0;
    dot += a * b;
    lhs_norm += a * a;
    rhs_norm += b * b;
  }
  if (lhs_norm <= 1e-12 || rhs_norm <= 1e-12) return -1.0;
  return std::max(-1.0, std::min(1.0, dot / std::sqrt(lhs_norm * rhs_norm)));
}

double appearanceSimilarity(const TrackletObservation& observation,
                            const TrackletIdentitySnapshot& identity) {
  double best = -1.0;
  for (const auto& prototype : identity.appearance_prototypes)
    best = std::max(best, cosineSimilarity(observation.appearance, prototype));
  return best;
}

struct ScoredIdentity {
  int id{-1};
  double score{0.0};
  bool assignable{true};
};

}  // namespace

TrackletStitchMatch findTrackletStitchMatch(
    const TrackletObservation& observation,
    const std::vector<TrackletIdentitySnapshot>& identities,
    const TrackletStitchConfig& config) {
  TrackletStitchMatch result;
  if (!config.enabled || !observation.confirmed ||
      !observation.has_world_position || observation.class_id < 0 ||
      !std::isfinite(observation.stamp_sec) ||
      !finiteVector(observation.world_position) ||
      !finiteVector(observation.position_variance) ||
      (observation.has_world_velocity &&
       !finiteVector(observation.world_velocity))) {
    result.reason = "observation_not_stitchable";
    return result;
  }

  std::vector<ScoredIdentity> ranked;
  ranked.reserve(identities.size());
  for (const auto& identity : identities) {
    if (identity.identity_id < 0 || !identity.established ||
        !identity.has_world_position ||
        !finiteVector(identity.world_position) ||
        !finiteVector(identity.position_variance) ||
        (identity.has_world_velocity && !finiteVector(identity.world_velocity)))
      continue;

    const bool same_source = identity.source == observation.source;
    const bool cross_source = !same_source;
    const bool class_match = identity.class_id == observation.class_id;
    const bool compatible_same_source_class = same_source && !class_match &&
        std::find(config.compatible_class_ids.begin(),
                  config.compatible_class_ids.end(), identity.class_id) !=
            config.compatible_class_ids.end() &&
        std::find(config.compatible_class_ids.begin(),
                  config.compatible_class_ids.end(), observation.class_id) !=
            config.compatible_class_ids.end();
    // In the configured vehicle detector, class IDs are vehicle subtypes and
    // can change with viewpoint. Keep same-camera class gating strict; for
    // cross-camera metric identity, treat subtype disagreement as a penalty
    // rather than a veto. The world gate and best/second margin remain hard.
    if (!class_match && !compatible_same_source_class &&
        (same_source || !config.cross_source_metric_enabled))
      continue;
    if (same_source && identity.active_in_source) continue;
    if (cross_source && !config.cross_source_metric_enabled) continue;

    const double dt = observation.stamp_sec - identity.last_seen_sec;
    if (!std::isfinite(dt)) continue;
    const double appearance = appearanceSimilarity(observation, identity);
    const bool appearance_valid = appearance >= -1.0 + 1e-9;
    const bool appearance_match = appearance_valid &&
        appearance >= config.minimum_appearance_cosine;
    const bool long_locked_recovery = same_source && identity.locked_target &&
        dt > config.archive_ttl_sec &&
        dt <= config.locked_identity_archive_ttl_sec;
    if (same_source) {
      if (dt <= 0.0) continue;
      if (dt > config.archive_ttl_sec && !long_locked_recovery) {
        // Older identities are not eligible to receive a public ID, but a
        // strong appearance match must still count as a competing hypothesis
        // when deciding whether the locked identity is unique.
        if (dt <= config.locked_identity_archive_ttl_sec &&
            appearance_valid && appearance >=
                config.locked_identity_min_appearance_cosine)
          ranked.push_back({identity.identity_id, appearance, false});
        continue;
      }
    } else {
      if (std::abs(dt) > config.cross_source_max_time_delta_sec) continue;
      // Do not let a noisy ground projection create a broad spatial gate.
      // This is an independent upper bound in addition to Mahalanobis gating.
      bool uncertainty_bounded = true;
      for (std::size_t axis = 0; axis < 3; ++axis) {
        uncertainty_bounded = uncertainty_bounded &&
            observation.position_variance[axis] >= 0.0 &&
            identity.position_variance[axis] >= 0.0 &&
            observation.position_variance[axis] <=
                config.cross_source_max_position_variance_m2 &&
            identity.position_variance[axis] <=
                config.cross_source_max_position_variance_m2;
      }
      if (!uncertainty_bounded) continue;
    }

    if (long_locked_recovery) {
      // A vehicle may have completed turns while outside the view, so its
      // old constant-velocity projection is not meaningful. Do not turn a
      // stale position into a permissive spatial gate: long-gap lock recovery
      // is allowed only with a very strong appearance match, then still has
      // to beat every competing identity by the normal ambiguity margin.
      if (!appearance_valid || appearance <
              config.locked_identity_min_appearance_cosine) continue;
      const double class_consistency_penalty = class_match ? 0.0 : 0.15;
      const double score = appearance - class_consistency_penalty;
      if (std::isfinite(score))
        ranked.push_back({identity.identity_id, score, true});
      continue;
    }

    // Across cameras, pixel/track motion is not comparable. Use calibrated
    // world coordinates instead; an appearance descriptor can support the
    // match, but is not mandatory because view/zoom changes can alter it.
    // Within one camera, require independent trajectory or appearance
    // evidence in addition to position; nearest-neighbour position alone is
    // deliberately insufficient for similar vehicles.
    if (same_source && !appearance_match &&
        !(identity.has_world_velocity && observation.has_world_velocity))
      continue;
    const bool motion_only_recovery = same_source && !appearance_match;
    if (motion_only_recovery && dt > config.motion_only_max_gap_sec) continue;

    std::array<double, 3> predicted = identity.world_position;
    if (identity.has_world_velocity) {
      for (std::size_t axis = 0; axis < predicted.size(); ++axis)
        predicted[axis] += identity.world_velocity[axis] * dt;
    }
    double residual_norm_sq = 0.0;
    double mahalanobis_sq = 0.0;
    double raw_displacement_sq = 0.0;
    for (std::size_t axis = 0; axis < predicted.size(); ++axis) {
      const double residual = observation.world_position[axis] - predicted[axis];
      const double displacement = observation.world_position[axis] -
                                  identity.world_position[axis];
      const double variance = std::max(config.minimum_position_variance_m2,
          observation.position_variance[axis] + identity.position_variance[axis] +
          config.process_noise_m2_per_s2 * dt * dt);
      residual_norm_sq += residual * residual;
      raw_displacement_sq += displacement * displacement;
      mahalanobis_sq += residual * residual / variance;
    }
    const double maximum_distance = cross_source
        ? std::min(config.maximum_distance_m,
                   config.cross_source_max_distance_m)
        : config.maximum_distance_m;
    if (!std::isfinite(mahalanobis_sq) ||
        mahalanobis_sq > config.mahalanobis_gate_sq ||
        std::sqrt(residual_norm_sq) > maximum_distance)
      continue;
    if (motion_only_recovery &&
        std::sqrt(residual_norm_sq) > config.motion_only_max_residual_m)
      continue;
    if (same_source && dt > 0.05 && std::sqrt(raw_displacement_sq) / dt >
        config.maximum_speed_mps) continue;

    double velocity_score = 0.0;
    const bool velocity_valid = identity.has_world_velocity &&
                                observation.has_world_velocity;
    if (velocity_valid) {
      double velocity_delta_sq = 0.0;
      for (std::size_t axis = 0; axis < predicted.size(); ++axis) {
        const double delta = observation.world_velocity[axis] -
                             identity.world_velocity[axis];
        velocity_delta_sq += delta * delta;
      }
      const double velocity_delta = std::sqrt(velocity_delta_sq);
      if (!std::isfinite(velocity_delta) ||
          velocity_delta > config.maximum_velocity_delta_mps ||
          (motion_only_recovery &&
           velocity_delta > config.motion_only_max_velocity_delta_mps))
        continue;
      const double scale = std::max(0.1, config.maximum_velocity_delta_mps);
      velocity_score = std::exp(-0.5 * velocity_delta_sq / (scale * scale));
    }

    const double motion_score = std::exp(-0.5 * mahalanobis_sq);
    double weighted_score = 0.65 * motion_score;
    double total_weight = 0.65;
    if (velocity_valid) {
      weighted_score += 0.20 * velocity_score;
      total_weight += 0.20;
    }
    if (appearance_match) {
      const double normalized_appearance =
          (appearance - config.minimum_appearance_cosine) /
          std::max(1e-6, 1.0 - config.minimum_appearance_cosine);
      weighted_score += 0.15 * std::max(0.0, std::min(1.0,
                                                    normalized_appearance));
      total_weight += 0.15;
    }
    const double class_consistency_penalty = !class_match ? 0.15 : 0.0;
    const double appearance_conflict_penalty =
        motion_only_recovery && appearance_valid ? 0.10 : 0.0;
    const double score = weighted_score / total_weight -
                         class_consistency_penalty -
                         appearance_conflict_penalty;
    if (std::isfinite(score))
      ranked.push_back({identity.identity_id, score, true});
  }

  if (ranked.empty()) {
    result.reason = "no_candidate_passed_gates";
    return result;
  }
  std::sort(ranked.begin(), ranked.end(),
            [](const ScoredIdentity& lhs, const ScoredIdentity& rhs) {
              return lhs.score > rhs.score;
            });
  result.score = ranked.front().score;
  result.margin = ranked.size() > 1
      ? ranked.front().score - ranked[1].score :
        std::numeric_limits<double>::infinity();
  if (result.score < config.minimum_score) {
    result.reason = "score_below_threshold";
    return result;
  }
  if (ranked.size() > 1 && result.margin < config.minimum_score_margin) {
    result.ambiguous = true;
    result.reason = "best_second_margin_too_small";
    return result;
  }
  if (!ranked.front().assignable) {
    result.ambiguous = true;
    result.reason = "locked_identity_competitor_not_recoverable";
    return result;
  }
  result.identity_id = ranked.front().id;
  result.reason = "matched";
  return result;
}

TrackletStitchConfirmationState advanceTrackletStitchConfirmation(
    const TrackletStitchConfirmationState& previous,
    const TrackletStitchMatch& match, const double stamp_sec,
    const double maximum_gap_sec) {
  TrackletStitchConfirmationState next;
  if (match.identity_id < 0 || match.ambiguous ||
      !std::isfinite(stamp_sec)) return next;
  const bool consecutive = previous.identity_id == match.identity_id &&
      previous.consecutive_hits > 0 &&
      std::isfinite(previous.last_stamp_sec) && stamp_sec > previous.last_stamp_sec &&
      stamp_sec - previous.last_stamp_sec <= maximum_gap_sec;
  next.identity_id = match.identity_id;
  next.consecutive_hits = consecutive ? previous.consecutive_hits + 1 : 1;
  next.last_stamp_sec = stamp_sec;
  next.last_score = match.score;
  return next;
}

}  // namespace xd_uav_track
