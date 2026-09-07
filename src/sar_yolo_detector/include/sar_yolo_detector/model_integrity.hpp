#pragma once

#include <string>

namespace sar_yolo_detector {

// Returns the lowercase hexadecimal SHA-256 digest. Throws on read/hash error.
std::string sha256File(const std::string &path);

// Empty expected hashes are accepted only when require_hash is false.
void verifyModelArtifact(const std::string &path, const std::string &expected,
                         bool require_hash);

} // namespace sar_yolo_detector
