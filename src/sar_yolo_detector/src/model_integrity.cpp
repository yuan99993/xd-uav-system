#include <sar_yolo_detector/model_integrity.hpp>

#include <algorithm>
#include <cctype>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <vector>

#include <openssl/evp.h>

namespace sar_yolo_detector {

std::string sha256File(const std::string &path) {
  if (path.empty())
    throw std::invalid_argument("Model artifact path is empty");
  std::ifstream input(path, std::ios::binary);
  if (!input)
    throw std::runtime_error("Cannot open model artifact for hashing: " + path);
  EVP_MD_CTX *context = EVP_MD_CTX_new();
  if (context == nullptr)
    throw std::runtime_error("Cannot allocate SHA-256 context");
  if (EVP_DigestInit_ex(context, EVP_sha256(), nullptr) != 1) {
    EVP_MD_CTX_free(context);
    throw std::runtime_error("Cannot initialize SHA-256 context");
  }
  std::vector<char> buffer(1024U * 1024U);
  while (input) {
    input.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
    const std::streamsize count = input.gcount();
    if (count > 0 &&
        EVP_DigestUpdate(context, buffer.data(),
                         static_cast<std::size_t>(count)) != 1) {
      EVP_MD_CTX_free(context);
      throw std::runtime_error("SHA-256 update failed for: " + path);
    }
  }
  if (!input.eof()) {
    EVP_MD_CTX_free(context);
    throw std::runtime_error("Failed while reading model artifact: " + path);
  }
  unsigned char digest[EVP_MAX_MD_SIZE];
  unsigned int digest_size = 0;
  if (EVP_DigestFinal_ex(context, digest, &digest_size) != 1) {
    EVP_MD_CTX_free(context);
    throw std::runtime_error("SHA-256 finalization failed for: " + path);
  }
  EVP_MD_CTX_free(context);
  std::ostringstream output;
  output << std::hex << std::setfill('0');
  for (unsigned int index = 0; index < digest_size; ++index)
    output << std::setw(2) << static_cast<unsigned int>(digest[index]);
  return output.str();
}

void verifyModelArtifact(const std::string &path, const std::string &expected_value,
                         const bool require_hash) {
  std::string expected = expected_value;
  expected.erase(std::remove_if(expected.begin(), expected.end(), ::isspace),
                 expected.end());
  std::transform(expected.begin(), expected.end(), expected.begin(), ::tolower);
  if (expected.empty()) {
    if (require_hash)
      throw std::invalid_argument("A model_sha256 is required for: " + path);
    return;
  }
  if (expected.size() != 64U ||
      !std::all_of(expected.begin(), expected.end(), [](unsigned char value) {
        return std::isxdigit(value) != 0;
      })) {
    throw std::invalid_argument("model_sha256 must be 64 hexadecimal characters");
  }
  const std::string actual = sha256File(path);
  if (actual != expected) {
    throw std::runtime_error("Model SHA-256 mismatch for " + path +
                             ": expected " + expected + ", got " + actual);
  }
}

} // namespace sar_yolo_detector
