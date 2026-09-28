#pragma once

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <string>

namespace xd_uav_controller {

enum class AirframeType : uint8_t {
  kMultirotor = 0,
  kFixedWing = 1,
  kVtol = 2,
  kTiltrotor = 3,
  kUnknown = 255,
};

enum class FlightRegime : uint8_t {
  kHover = 0,
  kTransitionToForward = 1,
  kForwardFlight = 2,
  kTransitionToHover = 3,
  kUnknown = 255,
};

enum class RequestedRegime : uint8_t {
  kNone = 0,
  kHover = 1,
  kForwardFlight = 2,
};

enum class VehicleAction : uint8_t {
  kNone = 0,
  kArm = 1,
  kDisarm = 2,
  kTakeoff = 3,
  kHold = 4,
  kNavigate = 5,
  kLand = 6,
  kReturnHome = 7,
  kEmergency = 8,
};

enum class BackendId : uint8_t {
  kNone = 0,
  kMultirotor = 1,
  kFixedWing = 2,
};

enum class ActionStatus : uint8_t {
  kIdle = 0,
  kActive = 1,
  kSucceeded = 2,
  kFailed = 3,
};

inline std::string normalizedAirframeName(std::string name) {
  std::transform(name.begin(), name.end(), name.begin(),
                 [](const unsigned char value) {
                   return static_cast<char>(std::tolower(value));
                 });
  return name;
}

inline AirframeType parseAirframeType(const std::string& name) {
  const std::string normalized = normalizedAirframeName(name);
  if (normalized == "multirotor") {
    return AirframeType::kMultirotor;
  }
  if (normalized == "fixedwing" || normalized == "fixed_wing") {
    return AirframeType::kFixedWing;
  }
  if (normalized == "vtol") {
    return AirframeType::kVtol;
  }
  if (normalized == "tiltrotor") {
    return AirframeType::kTiltrotor;
  }
  return AirframeType::kUnknown;
}

inline bool isVtolAirframe(const AirframeType airframe) {
  return airframe == AirframeType::kVtol ||
         airframe == AirframeType::kTiltrotor;
}

inline bool supportsRegime(const AirframeType airframe,
                           const FlightRegime regime) {
  switch (airframe) {
    case AirframeType::kMultirotor:
      return regime == FlightRegime::kHover;
    case AirframeType::kFixedWing:
      return regime == FlightRegime::kForwardFlight;
    case AirframeType::kVtol:
    case AirframeType::kTiltrotor:
      return regime == FlightRegime::kHover ||
             regime == FlightRegime::kTransitionToForward ||
             regime == FlightRegime::kForwardFlight ||
             regime == FlightRegime::kTransitionToHover;
    case AirframeType::kUnknown:
      return false;
  }
  return false;
}

inline FlightRegime staticRegimeForAirframe(const AirframeType airframe) {
  switch (airframe) {
    case AirframeType::kMultirotor:
      return FlightRegime::kHover;
    case AirframeType::kFixedWing:
      return FlightRegime::kForwardFlight;
    case AirframeType::kVtol:
    case AirframeType::kTiltrotor:
    case AirframeType::kUnknown:
      return FlightRegime::kUnknown;
  }
  return FlightRegime::kUnknown;
}

inline BackendId backendForRegime(const FlightRegime regime,
                                  const BackendId source_backend =
                                      BackendId::kNone) {
  switch (regime) {
    case FlightRegime::kHover:
      return BackendId::kMultirotor;
    case FlightRegime::kForwardFlight:
      return BackendId::kFixedWing;
    case FlightRegime::kTransitionToForward:
    case FlightRegime::kTransitionToHover:
      return source_backend;
    case FlightRegime::kUnknown:
      return BackendId::kNone;
  }
  return BackendId::kNone;
}

}  // namespace xd_uav_controller
