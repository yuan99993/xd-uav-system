#include <sar_yolo_detector/inference_backend.hpp>

#include <memory>
#include <stdexcept>
#include <string>

namespace sar_yolo_detector {

std::unique_ptr<InferenceBackend>
createTensorRtBackend(const InferenceBackendConfig &) {
  throw std::runtime_error(tensorRtBackendAvailabilityMessage());
}

bool tensorRtBackendAvailable() { return false; }

std::string tensorRtBackendAvailabilityMessage() {
  return "TensorRT backend is unavailable: build sar_yolo_detector on a host "
         "with NvInfer.h, libnvinfer and libcudart, then re-run catkin build";
}

} // namespace sar_yolo_detector
