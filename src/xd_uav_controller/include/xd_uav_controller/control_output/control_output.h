#pragma once

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <string>

namespace xd_uav_controller {

enum class ControlOutputType : uint8_t {
  kUnknown = 0,
  kBodyRate = 1,
  kAttitude = 2,
};

inline ControlOutputType parseControlOutputType(std::string name) {
  std::transform(name.begin(), name.end(), name.begin(),
                 [](const unsigned char value) {
                   return static_cast<char>(std::tolower(value));
                 });
  if (name == "body_rate" || name == "body-rate") {
    return ControlOutputType::kBodyRate;
  }
  if (name == "attitude" || name == "quaternion") {
    return ControlOutputType::kAttitude;
  }
  return ControlOutputType::kUnknown;
}

inline const char* controlOutputTypeName(const ControlOutputType type) {
  switch (type) {
    case ControlOutputType::kBodyRate:
      return "body_rate";
    case ControlOutputType::kAttitude:
      return "attitude";
    case ControlOutputType::kUnknown:
      return "unknown";
  }
  return "unknown";
}

}  // namespace xd_uav_controller
