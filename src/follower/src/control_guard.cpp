#include <follower/control_guard.hpp>

#include <algorithm>
#include <cctype>
#include <cmath>

namespace follower {
namespace {

std::string lower(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(),
                 [](const unsigned char character) {
                   return static_cast<char>(std::tolower(character));
                 });
  return value;
}

}  // namespace

OutputBackend parseOutputBackend(const std::string& value) {
  const std::string normalized = lower(value);
  if (normalized == "command_only" || normalized == "intent_only" ||
      normalized == "none") {
    return OutputBackend::kCommandOnly;
  }
  if (normalized == "mavros_body" ||
      normalized == "mavros_position_target") {
    return OutputBackend::kMavrosBody;
  }
  if (normalized == "mavros_twist_legacy") {
    return OutputBackend::kMavrosTwistLegacy;
  }
  if (normalized == "mrs_velocity" || normalized == "mrs") {
    return OutputBackend::kMrsVelocity;
  }
  return OutputBackend::kInvalid;
}

const char* outputBackendName(const OutputBackend backend) {
  switch (backend) {
    case OutputBackend::kCommandOnly: return "command_only";
    case OutputBackend::kMavrosBody: return "mavros_body";
    case OutputBackend::kMavrosTwistLegacy: return "mavros_twist_legacy";
    case OutputBackend::kMrsVelocity: return "mrs_velocity";
    case OutputBackend::kInvalid: return "invalid";
  }
  return "invalid";
}

ControlGuard::ControlGuard(const ControlGuardConfig& config) : config_(config) {
  config_.minimum_lease_sec =
      std::max(0.01, config_.minimum_lease_sec);
  config_.maximum_lease_sec =
      std::max(config_.minimum_lease_sec, config_.maximum_lease_sec);
}

LeaseResult ControlGuard::acquire(const std::string& requester,
                                  const std::string& requested_backend,
                                  const double requested_duration_sec,
                                  const double now) {
  if (!configurationValid()) {
    return currentResult(false, "Configured output backend is invalid");
  }
  if (requester.empty()) {
    return currentResult(false, "Requester must not be empty");
  }
  const OutputBackend requested = parseOutputBackend(requested_backend);
  if (requested == OutputBackend::kInvalid ||
      requested != config_.output_backend) {
    return currentResult(false,
                         "Requested backend does not match configured backend");
  }
  if (!std::isfinite(requested_duration_sec) ||
      requested_duration_sec < config_.minimum_lease_sec ||
      requested_duration_sec > config_.maximum_lease_sec) {
    return currentResult(false, "Requested lease duration is outside limits");
  }
  if (leaseActive(now) && requester != requester_) {
    return currentResult(false, "Control lease is owned by another requester");
  }
  requester_ = requester;
  expires_at_ = now + requested_duration_sec;
  lease_was_acquired_ = true;
  return currentResult(true, "Control lease acquired");
}

LeaseResult ControlGuard::release(const std::string& requester,
                                  const double now) {
  if (!leaseActive(now)) {
    requester_.clear();
    expires_at_ = 0.0;
    return currentResult(true, "Control lease already inactive");
  }
  if (requester.empty() || requester != requester_) {
    return currentResult(false, "Only the active requester may release the lease");
  }
  requester_.clear();
  expires_at_ = 0.0;
  return currentResult(true, "Control lease released");
}

bool ControlGuard::authorized(const double now) const {
  return configurationValid() &&
         (!config_.require_lease || leaseActive(now));
}

bool ControlGuard::leaseActive(const double now) const {
  return !requester_.empty() && std::isfinite(now) && now < expires_at_;
}

bool ControlGuard::leaseExpired(const double now) const {
  return lease_was_acquired_ && !requester_.empty() &&
         std::isfinite(now) && now >= expires_at_;
}

double ControlGuard::remaining(const double now) const {
  return leaseActive(now) ? std::max(0.0, expires_at_ - now) : 0.0;
}

const std::string& ControlGuard::requester() const { return requester_; }

OutputBackend ControlGuard::outputBackend() const {
  return config_.output_backend;
}

bool ControlGuard::configurationValid() const {
  return config_.output_backend != OutputBackend::kInvalid;
}

bool ControlGuard::requireLease() const { return config_.require_lease; }

LeaseResult ControlGuard::currentResult(const bool success,
                                        const std::string& message) const {
  LeaseResult result;
  result.success = success;
  result.message = message;
  result.requester = requester_;
  result.output_backend = config_.output_backend;
  result.expires_at = expires_at_;
  return result;
}

}  // namespace follower
