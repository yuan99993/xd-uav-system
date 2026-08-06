#pragma once

#include <string>

namespace follower {

enum class OutputBackend {
  kCommandOnly,
  kMavrosBody,
  kMavrosTwistLegacy,
  kMrsVelocity,
  kInvalid,
};

OutputBackend parseOutputBackend(const std::string& value);
const char* outputBackendName(OutputBackend backend);

struct ControlGuardConfig {
  bool require_lease{false};
  double minimum_lease_sec{0.10};
  double maximum_lease_sec{30.0};
  OutputBackend output_backend{OutputBackend::kCommandOnly};
};

struct LeaseResult {
  bool success{false};
  std::string message;
  std::string requester;
  OutputBackend output_backend{OutputBackend::kInvalid};
  double expires_at{0.0};
};

// Transport-independent authorization guard. Time values are monotonic
// seconds supplied by the caller, making the policy reusable in ROS tests and
// other wrappers without depending on ROS time types.
class ControlGuard {
 public:
  explicit ControlGuard(
      const ControlGuardConfig& config = ControlGuardConfig());

  LeaseResult acquire(const std::string& requester,
                      const std::string& requested_backend,
                      double requested_duration_sec, double now);
  LeaseResult release(const std::string& requester, double now);
  bool authorized(double now) const;
  bool leaseActive(double now) const;
  bool leaseExpired(double now) const;
  double remaining(double now) const;
  const std::string& requester() const;
  OutputBackend outputBackend() const;
  bool configurationValid() const;
  bool requireLease() const;

 private:
  LeaseResult currentResult(bool success, const std::string& message) const;

  ControlGuardConfig config_;
  std::string requester_;
  double expires_at_{0.0};
  bool lease_was_acquired_{false};
};

}  // namespace follower
