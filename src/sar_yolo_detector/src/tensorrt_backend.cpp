#include <sar_yolo_detector/inference_backend.hpp>

#include <NvInfer.h>
#include <cuda_fp16.h>
#include <cuda_runtime_api.h>

#include <algorithm>
#include <cstring>
#include <fstream>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace sar_yolo_detector {
namespace {

class TensorRtLogger final : public nvinfer1::ILogger {
public:
  void log(const Severity severity,
           const char *const message) noexcept override {
    if (severity <= Severity::kWARNING) {
      last_message_ = message == nullptr ? "TensorRT error" : message;
    }
  }

  std::string lastMessage() const { return last_message_; }

private:
  std::string last_message_;
};

template <typename TensorRtType> struct TensorRtDeleter {
  void operator()(TensorRtType *const object) const {
    if (object != nullptr) {
#if NV_TENSORRT_MAJOR >= 10
      // TensorRT 10 removed the legacy destroy() API.  All public runtime
      // interfaces now have usable C++ destructors.
      delete object;
#else
      object->destroy();
#endif
    }
  }
};

void checkCuda(const cudaError_t result, const std::string &operation) {
  if (result != cudaSuccess) {
    throw std::runtime_error(operation + ": " + cudaGetErrorString(result));
  }
}

std::vector<char> readBinaryFile(const std::string &path) {
  std::ifstream stream(path, std::ios::binary | std::ios::ate);
  if (!stream.good()) {
    throw std::invalid_argument("TensorRT engine cannot be read: " + path);
  }
  const std::ifstream::pos_type size = stream.tellg();
  if (size <= 0)
    throw std::runtime_error("TensorRT engine is empty: " + path);
  std::vector<char> bytes(static_cast<std::size_t>(size));
  stream.seekg(0, std::ios::beg);
  stream.read(bytes.data(), size);
  if (!stream.good())
    throw std::runtime_error("Unable to read TensorRT engine: " + path);
  return bytes;
}

std::size_t dimensionsVolume(const nvinfer1::Dims &dimensions) {
  std::size_t volume = 1;
  for (int index = 0; index < dimensions.nbDims; ++index) {
    if (dimensions.d[index] <= 0) {
      throw std::runtime_error(
          "TensorRT tensor has unresolved dynamic dimensions");
    }
    volume *= static_cast<std::size_t>(dimensions.d[index]);
  }
  return volume;
}

std::size_t dataTypeSize(const nvinfer1::DataType type) {
  switch (type) {
  case nvinfer1::DataType::kFLOAT:
  case nvinfer1::DataType::kINT32:
    return 4U;
  case nvinfer1::DataType::kHALF:
    return 2U;
  case nvinfer1::DataType::kINT8:
  case nvinfer1::DataType::kBOOL:
    return 1U;
  default:
    throw std::runtime_error("Unsupported TensorRT tensor data type");
  }
}

class TensorRtBackend final : public InferenceBackend {
public:
  explicit TensorRtBackend(const InferenceBackendConfig &config) {
    if (config.engine_path.empty()) {
      throw std::invalid_argument(
          "TensorRT backend requires engine_path; build the engine offline for "
          "this GPU and TensorRT version");
    }
    const std::vector<char> serialized_engine =
        readBinaryFile(config.engine_path);
    runtime_.reset(nvinfer1::createInferRuntime(logger_));
    if (!runtime_)
      throw std::runtime_error("Unable to create TensorRT runtime");
#if NV_TENSORRT_MAJOR >= 10
    engine_.reset(runtime_->deserializeCudaEngine(
        serialized_engine.data(), serialized_engine.size()));
#else
    engine_.reset(runtime_->deserializeCudaEngine(
        serialized_engine.data(), serialized_engine.size(), nullptr));
#endif
    if (!engine_) {
      throw std::runtime_error("Unable to deserialize TensorRT engine: " +
                               logger_.lastMessage());
    }
    context_.reset(engine_->createExecutionContext());
    if (!context_)
      throw std::runtime_error("Unable to create TensorRT context");
    checkCuda(cudaStreamCreate(&stream_), "cudaStreamCreate");
    discoverBindings(config.tensorrt_output_binding_names);
  }

  ~TensorRtBackend() override {
    for (auto &entry : buffers_)
      cudaFree(entry.second.pointer);
    if (stream_ != nullptr)
      cudaStreamDestroy(stream_);
  }

  std::string name() const override { return "tensorrt"; }

  std::string details() const override {
    std::ostringstream stream;
    stream << "engine=" << engine_->getName() << ", input=" << input_name_
           << ", outputs=" << output_names_.size();
    return stream.str();
  }

  std::vector<cv::Mat> infer(const cv::Mat &input_blob) override {
    if (input_blob.empty() || input_blob.type() != CV_32F ||
        input_blob.dims != 4) {
      throw std::invalid_argument(
          "TensorRT expects a contiguous NCHW CV_32F blob");
    }
    if (!input_blob.isContinuous()) {
      throw std::invalid_argument("TensorRT input blob must be contiguous");
    }
    const nvinfer1::Dims input_dimensions = makeInputDimensions(input_blob);
    configureInputDimensions(input_dimensions);
    const nvinfer1::DataType input_type = tensorDataType(input_name_);
    if (input_type != nvinfer1::DataType::kFLOAT &&
        input_type != nvinfer1::DataType::kHALF) {
      throw std::runtime_error(
          "TensorRT input binding must use FP32 or FP16 data");
    }
    const std::size_t input_elements = input_blob.total();
    const std::size_t input_bytes = input_elements * dataTypeSize(input_type);
    ensureBuffer(input_name_, input_bytes);
    std::vector<__half> half_input;
    const void *input_data = input_blob.data;
    if (input_type == nvinfer1::DataType::kHALF) {
      half_input.resize(input_elements);
      const float *values = input_blob.ptr<float>();
      for (std::size_t index = 0; index < input_elements; ++index)
        half_input[index] = __float2half(values[index]);
      input_data = half_input.data();
    }
    checkCuda(cudaMemcpyAsync(buffers_[input_name_].pointer, input_data,
                              input_bytes, cudaMemcpyHostToDevice, stream_),
              "cudaMemcpyAsync input");

    std::vector<nvinfer1::Dims> output_dimensions;
    std::vector<nvinfer1::DataType> output_types;
    output_dimensions.reserve(output_names_.size());
    output_types.reserve(output_names_.size());
    for (const std::string &name : output_names_) {
      const nvinfer1::Dims dimensions = resolvedOutputDimensions(name);
      const nvinfer1::DataType type = tensorDataType(name);
      if (type != nvinfer1::DataType::kFLOAT &&
          type != nvinfer1::DataType::kHALF) {
        throw std::runtime_error(
            "Selected TensorRT output must use FP32 or FP16 data");
      }
      ensureBuffer(name, dimensionsVolume(dimensions) * dataTypeSize(type));
      output_dimensions.push_back(dimensions);
      output_types.push_back(type);
    }

#if NV_TENSORRT_MAJOR >= 10
    // enqueueV3 requires an address for every I/O output tensor in the
    // serialized engine, including export-time intermediate heads that are
    // intentionally not exposed to the ROS postprocessor.  Keep those
    // tensors device-resident but do not copy them back to the host.
    for (const std::string &name : all_output_names_) {
      if (std::find(output_names_.begin(), output_names_.end(), name) !=
          output_names_.end()) {
        continue;
      }
      const nvinfer1::Dims dimensions = resolvedOutputDimensions(name);
      const nvinfer1::DataType type = tensorDataType(name);
      ensureBuffer(name,
                   dimensionsVolume(dimensions) * dataTypeSize(type));
    }
#endif

    execute();
    struct HostOutput {
      nvinfer1::Dims dimensions;
      nvinfer1::DataType type;
      std::vector<float> float_values;
      std::vector<__half> half_values;
    };
    std::vector<HostOutput> host_outputs;
    host_outputs.reserve(output_names_.size());
    for (std::size_t index = 0; index < output_names_.size(); ++index) {
      HostOutput host;
      host.dimensions = output_dimensions[index];
      host.type = output_types[index];
      const std::size_t element_count = dimensionsVolume(host.dimensions);
      void *destination = nullptr;
      std::size_t bytes = 0U;
      if (host.type == nvinfer1::DataType::kFLOAT) {
        host.float_values.resize(element_count);
        destination = host.float_values.data();
        bytes = element_count * sizeof(float);
      } else {
        host.half_values.resize(element_count);
        destination = host.half_values.data();
        bytes = element_count * sizeof(__half);
      }
      host_outputs.push_back(std::move(host));
      checkCuda(cudaMemcpyAsync(destination,
                                buffers_[output_names_[index]].pointer, bytes,
                                cudaMemcpyDeviceToHost, stream_),
                "cudaMemcpyAsync output");
    }
    // One synchronization covers every output instead of adding a blocking
    // barrier for each binding.
    checkCuda(cudaStreamSynchronize(stream_), "cudaStreamSynchronize");

    std::vector<cv::Mat> outputs;
    outputs.reserve(output_names_.size());
    for (std::size_t index = 0; index < output_names_.size(); ++index) {
      const HostOutput &host = host_outputs[index];
      const nvinfer1::Dims &dimensions = host.dimensions;
      const std::size_t element_count = dimensionsVolume(dimensions);
      std::vector<int> shape;
      shape.reserve(dimensions.nbDims);
      for (int dimension = 0; dimension < dimensions.nbDims; ++dimension) {
        shape.push_back(dimensions.d[dimension]);
      }
      cv::Mat output(static_cast<int>(shape.size()), shape.data(), CV_32F);
      if (host.type == nvinfer1::DataType::kFLOAT) {
        std::memcpy(output.ptr<float>(), host.float_values.data(),
                    element_count * sizeof(float));
      } else {
        float *values = output.ptr<float>();
        for (std::size_t element = 0; element < element_count; ++element)
          values[element] = __half2float(host.half_values[element]);
      }
      outputs.push_back(output);
    }
    return outputs;
  }

private:
  struct DeviceBuffer {
    void *pointer{nullptr};
    std::size_t capacity{0};
  };

  void discoverBindings(const std::vector<std::string> &requested_outputs) {
#if NV_TENSORRT_MAJOR >= 10
    for (int index = 0; index < engine_->getNbIOTensors(); ++index) {
      const char *const name = engine_->getIOTensorName(index);
      if (engine_->getTensorIOMode(name) == nvinfer1::TensorIOMode::kINPUT) {
        if (!input_name_.empty()) {
          throw std::runtime_error(
              "TensorRT engine must have exactly one input tensor");
        }
        input_name_ = name;
      } else {
        output_names_.push_back(name);
        all_output_names_.push_back(name);
      }
    }
#else
    for (int index = 0; index < engine_->getNbBindings(); ++index) {
      const char *const name = engine_->getBindingName(index);
      if (engine_->bindingIsInput(index)) {
        if (!input_name_.empty()) {
          throw std::runtime_error(
              "TensorRT engine must have exactly one input tensor");
        }
        input_name_ = name;
        input_binding_index_ = index;
      } else {
        output_names_.push_back(name);
        all_output_names_.push_back(name);
        output_binding_indices_.push_back(index);
      }
    }
#endif
    if (input_name_.empty() || output_names_.empty()) {
      throw std::runtime_error(
          "TensorRT engine requires one input and one or more outputs");
    }
    if (!requested_outputs.empty()) {
      for (const std::string &name : requested_outputs) {
        if (std::find(output_names_.begin(), output_names_.end(), name) ==
            output_names_.end()) {
          throw std::invalid_argument(
              "Requested TensorRT output binding not found: " + name);
        }
      }
      output_names_ = requested_outputs;
#if NV_TENSORRT_MAJOR < 10
      output_binding_indices_.clear();
      for (const std::string &name : output_names_) {
        output_binding_indices_.push_back(
            engine_->getBindingIndex(name.c_str()));
      }
#endif
    }
    if (tensorDataType(input_name_) != nvinfer1::DataType::kFLOAT &&
        tensorDataType(input_name_) != nvinfer1::DataType::kHALF) {
      throw std::runtime_error(
          "TensorRT backend requires an FP32 or FP16 input binding");
    }
  }

  nvinfer1::Dims makeInputDimensions(const cv::Mat &blob) const {
    nvinfer1::Dims dimensions;
    dimensions.nbDims = blob.dims;
    for (int index = 0; index < blob.dims; ++index) {
      dimensions.d[index] = blob.size[index];
    }
    return dimensions;
  }

  void configureInputDimensions(const nvinfer1::Dims &dimensions) {
#if NV_TENSORRT_MAJOR >= 10
    if (!context_->setInputShape(input_name_.c_str(), dimensions)) {
      throw std::runtime_error("TensorRT rejected input tensor dimensions");
    }
#else
    if (!context_->setBindingDimensions(input_binding_index_, dimensions)) {
      throw std::runtime_error("TensorRT rejected input tensor dimensions");
    }
    if (!context_->allInputDimensionsSpecified()) {
      throw std::runtime_error(
          "TensorRT input dimensions are not fully specified");
    }
#endif
  }

  nvinfer1::Dims resolvedOutputDimensions(const std::string &name) const {
#if NV_TENSORRT_MAJOR >= 10
    return context_->getTensorShape(name.c_str());
#else
    const int index = engine_->getBindingIndex(name.c_str());
    return context_->getBindingDimensions(index);
#endif
  }

  nvinfer1::DataType tensorDataType(const std::string &name) const {
#if NV_TENSORRT_MAJOR >= 10
    return engine_->getTensorDataType(name.c_str());
#else
    return engine_->getBindingDataType(engine_->getBindingIndex(name.c_str()));
#endif
  }

  void ensureBuffer(const std::string &name, const std::size_t bytes) {
    DeviceBuffer &buffer = buffers_[name];
    if (buffer.pointer != nullptr && buffer.capacity >= bytes)
      return;
    if (buffer.pointer != nullptr) {
      checkCuda(cudaFree(buffer.pointer), "cudaFree resize");
      buffer.pointer = nullptr;
      buffer.capacity = 0;
    }
    checkCuda(cudaMalloc(&buffer.pointer, bytes), "cudaMalloc");
    buffer.capacity = bytes;
  }

  void execute() {
#if NV_TENSORRT_MAJOR >= 10
    if (!context_->setTensorAddress(input_name_.c_str(),
                                    buffers_[input_name_].pointer)) {
      throw std::runtime_error("TensorRT failed to set input address");
    }
    for (const std::string &name : all_output_names_) {
      if (!context_->setTensorAddress(name.c_str(), buffers_[name].pointer)) {
        throw std::runtime_error("TensorRT failed to set output address");
      }
    }
    if (!context_->enqueueV3(stream_)) {
      throw std::runtime_error("TensorRT enqueueV3 failed");
    }
#else
    std::vector<void *> bindings(
        static_cast<std::size_t>(engine_->getNbBindings()), nullptr);
    bindings[input_binding_index_] = buffers_[input_name_].pointer;
    for (std::size_t index = 0; index < output_names_.size(); ++index) {
      bindings[output_binding_indices_[index]] =
          buffers_[output_names_[index]].pointer;
    }
    if (!context_->enqueueV2(bindings.data(), stream_, nullptr)) {
      throw std::runtime_error("TensorRT enqueueV2 failed");
    }
#endif
  }

  TensorRtLogger logger_;
  std::unique_ptr<nvinfer1::IRuntime, TensorRtDeleter<nvinfer1::IRuntime>>
      runtime_;
  std::unique_ptr<nvinfer1::ICudaEngine, TensorRtDeleter<nvinfer1::ICudaEngine>>
      engine_;
  std::unique_ptr<nvinfer1::IExecutionContext,
                  TensorRtDeleter<nvinfer1::IExecutionContext>>
      context_;
  cudaStream_t stream_{nullptr};
  std::string input_name_;
  std::vector<std::string> output_names_;
  std::vector<std::string> all_output_names_;
  std::map<std::string, DeviceBuffer> buffers_;
#if NV_TENSORRT_MAJOR < 10
  int input_binding_index_{-1};
  std::vector<int> output_binding_indices_;
#endif
};

} // namespace

std::unique_ptr<InferenceBackend>
createTensorRtBackend(const InferenceBackendConfig &config) {
  return std::unique_ptr<InferenceBackend>(new TensorRtBackend(config));
}

bool tensorRtBackendAvailable() { return true; }

std::string tensorRtBackendAvailabilityMessage() {
  return "TensorRT engine backend is available";
}

} // namespace sar_yolo_detector
