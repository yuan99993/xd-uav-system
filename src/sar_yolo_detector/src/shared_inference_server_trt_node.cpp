#include <sar_yolo_detector/GetSharedModelInfo.h>
#include <sar_yolo_detector/NormalizedDetection.h>
#include <sar_yolo_detector/RunSharedInference.h>

#include <NvInfer.h>
#include <NvInferPlugin.h>
#include <cuda_fp16.h>
#include <cuda_runtime_api.h>
#include <openssl/sha.h>
#include <yaml-cpp/yaml.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cctype>
#include <cerrno>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string>
#include <sys/file.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <thread>
#include <typeinfo>
#include <unordered_map>
#include <unordered_set>
#include <unistd.h>
#include <utility>
#include <vector>

namespace {

using Clock = std::chrono::steady_clock;

void requireCuda(cudaError_t status, const char *operation) {
  if (status != cudaSuccess) {
    throw std::runtime_error(std::string(operation) + ": " +
                             cudaGetErrorString(status));
  }
}

class TrtLogger final : public nvinfer1::ILogger {
 public:
  void log(Severity severity, const char *message) noexcept override {
    if (message == nullptr) return;
    if (severity <= Severity::kERROR) {
      ROS_ERROR_STREAM("[TensorRT] " << message);
    } else if (severity == Severity::kWARNING) {
      ROS_WARN_STREAM("[TensorRT] " << message);
    } else if (severity == Severity::kINFO) {
      ROS_INFO_STREAM("[TensorRT] " << message);
    }
  }
};

template <typename T>
struct TrtDestroy {
  void operator()(T *object) const noexcept {
    if (object != nullptr) object->destroy();
  }
};

using RuntimePtr = std::unique_ptr<nvinfer1::IRuntime, TrtDestroy<nvinfer1::IRuntime>>;
using EnginePtr = std::unique_ptr<nvinfer1::ICudaEngine, TrtDestroy<nvinfer1::ICudaEngine>>;
using ContextPtr = std::unique_ptr<nvinfer1::IExecutionContext,
                                   TrtDestroy<nvinfer1::IExecutionContext>>;

std::string expandPath(std::string path) {
  if (!path.empty() && path[0] == '~') {
    const char *home = std::getenv("HOME");
    if (home != nullptr && (path.size() == 1 || path[1] == '/')) {
      path.replace(0, 1, home);
    }
  }
  return path;
}

std::string jsonEscape(const std::string &value) {
  std::ostringstream out;
  for (unsigned char ch : value) {
    switch (ch) {
      case '"': out << "\\\""; break;
      case '\\': out << "\\\\"; break;
      case '\b': out << "\\b"; break;
      case '\f': out << "\\f"; break;
      case '\n': out << "\\n"; break;
      case '\r': out << "\\r"; break;
      case '\t': out << "\\t"; break;
      default:
        if (ch < 0x20) {
          out << "\\u" << std::hex << std::setw(4) << std::setfill('0')
              << static_cast<unsigned int>(ch) << std::dec;
        } else {
          out << static_cast<char>(ch);
        }
    }
  }
  return out.str();
}

std::string jsonStringMap(const std::unordered_map<int, std::string> &values) {
  std::vector<int> keys;
  keys.reserve(values.size());
  for (const auto &entry : values) keys.push_back(entry.first);
  std::sort(keys.begin(), keys.end());
  std::ostringstream out;
  out << '{';
  bool first = true;
  for (int key : keys) {
    if (!first) out << ',';
    first = false;
    out << '"' << key << "\":\"" << jsonEscape(values.at(key)) << '"';
  }
  out << '}';
  return out.str();
}

bool isHexDigest(const std::string &value) {
  return value.size() == 64 && std::all_of(value.begin(), value.end(), [](unsigned char ch) {
    return std::isxdigit(ch) != 0;
  });
}

std::string sha256File(const std::string &path) {
  std::ifstream stream(path, std::ios::binary);
  if (!stream) throw std::runtime_error("cannot open model file: " + path);
  SHA256_CTX context;
  SHA256_Init(&context);
  char buffer[1024 * 1024];
  while (stream.good()) {
    stream.read(buffer, sizeof(buffer));
    const std::streamsize count = stream.gcount();
    if (count > 0) SHA256_Update(&context, buffer, static_cast<size_t>(count));
  }
  if (!stream.eof()) throw std::runtime_error("failed while reading model file: " + path);
  unsigned char digest[SHA256_DIGEST_LENGTH];
  SHA256_Final(digest, &context);
  std::ostringstream out;
  out << std::hex << std::setfill('0');
  for (unsigned char byte : digest) out << std::setw(2) << static_cast<unsigned int>(byte);
  return out.str();
}

std::string readFile(const std::string &path) {
  std::ifstream stream(path, std::ios::binary);
  if (!stream) throw std::runtime_error("cannot open file: " + path);
  std::ostringstream contents;
  contents << stream.rdbuf();
  if (!stream.good() && !stream.eof()) throw std::runtime_error("failed reading file: " + path);
  return contents.str();
}

YAML::Node readYaml(const std::string &path) {
  if (path.empty()) return YAML::Node(YAML::NodeType::Map);
  const std::string expanded = expandPath(path);
  if (expanded.empty()) return YAML::Node(YAML::NodeType::Map);
  return YAML::LoadFile(expanded);
}

YAML::Node mergeYaml(const YAML::Node &base, const YAML::Node &overlay) {
  if (!base.IsMap() || !overlay.IsMap()) {
    return overlay ? YAML::Clone(overlay) : YAML::Clone(base);
  }
  YAML::Node merged = YAML::Clone(base);
  for (const auto &entry : overlay) {
    const std::string key = entry.first.as<std::string>();
    const YAML::Node current = merged[key];
    if (current.IsMap() && entry.second.IsMap()) {
      merged[key] = mergeYaml(current, entry.second);
    } else {
      merged[key] = YAML::Clone(entry.second);
    }
  }
  return merged;
}

cv::Mat imageToBgr(const sensor_msgs::Image &message) {
  if (message.height == 0 || message.width == 0) {
    throw std::runtime_error("image dimensions must be positive");
  }
  const size_t height = message.height;
  const size_t width = message.width;
  const size_t step = message.step;
  std::string encoding = message.encoding;
  std::transform(encoding.begin(), encoding.end(), encoding.begin(),
                 [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });

  int channels = 0;
  int conversion = -1;
  if (encoding == "bgr8") channels = 3;
  else if (encoding == "rgb8") { channels = 3; conversion = cv::COLOR_RGB2BGR; }
  else if (encoding == "bgra8") { channels = 4; conversion = cv::COLOR_BGRA2BGR; }
  else if (encoding == "rgba8") { channels = 4; conversion = cv::COLOR_RGBA2BGR; }
  else if (encoding == "mono8" || encoding == "8uc1") {
    channels = 1;
    conversion = cv::COLOR_GRAY2BGR;
  }

  if (channels > 0) {
    const size_t minimum_step = width * static_cast<size_t>(channels);
    if (step < minimum_step || height > message.data.size() / step) {
      throw std::runtime_error("invalid image stride or data length");
    }
    const int type = channels == 1 ? CV_8UC1 : CV_MAKETYPE(CV_8U, channels);
    cv::Mat source(static_cast<int>(height), static_cast<int>(width), type,
                   const_cast<uint8_t *>(message.data.data()), step);
    cv::Mat bgr;
    if (conversion < 0) source.copyTo(bgr);
    else cv::cvtColor(source, bgr, conversion);
    return bgr;
  }

  if (encoding == "mono16" || encoding == "16uc1" || encoding == "y16") {
    if (step < width * 2 || step % 2 != 0 || height > message.data.size() / step) {
      throw std::runtime_error("invalid 16-bit image stride or data length");
    }
    std::vector<uint16_t> samples;
    samples.reserve(height * width);
    for (size_t y = 0; y < height; ++y) {
      const uint8_t *row = message.data.data() + y * step;
      for (size_t x = 0; x < width; ++x) {
        const uint16_t high = message.is_bigendian ? row[2 * x] : row[2 * x + 1];
        const uint16_t low = message.is_bigendian ? row[2 * x + 1] : row[2 * x];
        samples.push_back(static_cast<uint16_t>((high << 8) | low));
      }
    }
    std::vector<uint16_t> ordered_samples = samples;
    const auto percentile = [&ordered_samples](double fraction) {
      const size_t last = ordered_samples.size() - 1;
      const double position = fraction * last;
      const size_t lower = static_cast<size_t>(std::floor(position));
      const size_t upper = static_cast<size_t>(std::ceil(position));
      std::nth_element(ordered_samples.begin(), ordered_samples.begin() + lower,
                       ordered_samples.end());
      const double low_value = ordered_samples[lower];
      if (upper == lower) return low_value;
      std::nth_element(ordered_samples.begin(), ordered_samples.begin() + upper,
                       ordered_samples.end());
      return low_value + (ordered_samples[upper] - low_value) * (position - lower);
    };
    const double low = percentile(0.01);
    const double high = percentile(0.99);
    cv::Mat gray(static_cast<int>(height), static_cast<int>(width), CV_8UC1);
    if (high <= low) {
      gray.setTo(cv::Scalar(0));
    } else {
      const double scale = 255.0 / (high - low);
      for (size_t y = 0; y < height; ++y) {
        uint8_t *output = gray.ptr<uint8_t>(static_cast<int>(y));
        for (size_t x = 0; x < width; ++x) {
          const double value = samples[y * width + x];
          output[x] = static_cast<uint8_t>(std::max(0.0, std::min(255.0,
              (value - low) * scale)));
        }
      }
    }
    cv::Mat bgr;
    cv::cvtColor(gray, bgr, cv::COLOR_GRAY2BGR);
    return bgr;
  }
  throw std::runtime_error("unsupported image encoding: " + message.encoding);
}

struct LetterboxInfo {
  int width{0};
  int height{0};
  int resized_width{0};
  int resized_height{0};
  int left{0};
  int top{0};
  double gain{1.0};
};

struct Candidate {
  float x1{0.0f};
  float y1{0.0f};
  float x2{0.0f};
  float y2{0.0f};
  float confidence{0.0f};
  int class_id{-1};
};

struct PendingRequest {
  std::string source_id;
  ros::Time stamp;
  cv::Mat frame;
  float confidence{0.3f};
  float iou{0.3f};
  uint32_t max_detections{20};
  int image_size{0};
  std::atomic<bool> completed{false};
  std::mutex response_mutex;
  std::condition_variable response_condition;
  sar_yolo_detector::RunSharedInference::Response response;
};

void completeRequest(const std::shared_ptr<PendingRequest> &pending,
                     const sar_yolo_detector::RunSharedInference::Response &response) {
  if (!pending) return;
  std::lock_guard<std::mutex> lock(pending->response_mutex);
  if (pending->completed.load()) return;
  pending->response = response;
  pending->completed.store(true);
  pending->response_condition.notify_all();
}

float boxIou(const Candidate &left, const Candidate &right) {
  const float x1 = std::max(left.x1, right.x1);
  const float y1 = std::max(left.y1, right.y1);
  const float x2 = std::min(left.x2, right.x2);
  const float y2 = std::min(left.y2, right.y2);
  const float intersection = std::max(0.0f, x2 - x1) * std::max(0.0f, y2 - y1);
  const float area_left = std::max(0.0f, left.x2 - left.x1) *
                          std::max(0.0f, left.y2 - left.y1);
  const float area_right = std::max(0.0f, right.x2 - right.x1) *
                           std::max(0.0f, right.y2 - right.y1);
  const float denominator = area_left + area_right - intersection;
  return denominator > 0.0f ? intersection / denominator : 0.0f;
}

std::vector<Candidate> nonMaxSuppression(std::vector<Candidate> candidates,
                                         float iou_threshold, uint32_t maximum,
                                         bool agnostic) {
  std::stable_sort(candidates.begin(), candidates.end(),
                   [](const Candidate &a, const Candidate &b) {
                     return a.confidence > b.confidence;
                   });
  std::vector<Candidate> kept;
  kept.reserve(std::min<size_t>(maximum, candidates.size()));
  for (const Candidate &candidate : candidates) {
    bool suppressed = false;
    for (const Candidate &accepted : kept) {
      if (!agnostic && candidate.class_id != accepted.class_id) continue;
      if (boxIou(candidate, accepted) > iou_threshold) {
        suppressed = true;
        break;
      }
    }
    if (!suppressed) {
      kept.push_back(candidate);
      if (kept.size() >= maximum) break;
    }
  }
  return kept;
}

class SharedInferenceServer {
 public:
  SharedInferenceServer()
      : nh_(), private_nh_("~"), logger_() {
    private_nh_.param<std::string>("model_path", model_path_, "");
    private_nh_.param<std::string>("model_sha256", expected_sha256_, "");
    private_nh_.param<std::string>("config", config_path_, "");
    private_nh_.param<std::string>("overlay_config", overlay_config_path_, "");
    private_nh_.param<std::string>("lock_dir", lock_dir_, "/tmp");
    private_nh_.param("use_gpu", use_gpu_, true);
    private_nh_.param("fallback_to_cpu", fallback_to_cpu_, false);
    private_nh_.param("maximum_batch_size", requested_batch_size_, 2);
    private_nh_.param("batch_wait_ms", batch_wait_ms_, 4.0);
    private_nh_.param("maximum_capture_age_sec", maximum_age_sec_, 0.5);
    private_nh_.param("startup_warmup", startup_warmup_, true);
    private_nh_.param("opencv_threads", opencv_threads_, 1);

    cv::setNumThreads(std::max(1, std::min(4, opencv_threads_)));
    batch_wait_ms_ = std::max(0.0, std::min(10.0, batch_wait_ms_));
    maximum_age_sec_ = std::max(0.0, maximum_age_sec_);
    requested_batch_size_ = std::max(1, std::min(8, requested_batch_size_));
    maximum_batch_size_ = requested_batch_size_;
    loadClassConfig();
  }

  ~SharedInferenceServer() {
    stopWorker();
    releaseCuda();
    if (lock_fd_ >= 0) close(lock_fd_);
  }

  void run() {
    resolveModelPathAndKey();
    while (ros::ok()) {
      if (!acquireModelLock()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
        continue;
      }
      try {
        loadEngine();
        if (startup_warmup_) {
          warmup();
          if (!runtime_json_.empty() && runtime_json_.back() == '}') {
            runtime_json_.pop_back();
            runtime_json_ += ",\"startup_warmup_ms\":" +
                std::to_string(startup_warmup_ms_) + "}";
          }
        }
        startProvider();
        return;
      } catch (const std::exception &error) {
        ROS_ERROR_STREAM("shared TensorRT provider startup failed: " << error.what());
        stopWorker();
        releaseProviderLock();
        releaseCuda();
        if (ros::ok()) ros::Duration(5.0).sleep();
      }
    }
  }

 private:
  struct BatchTiming {
    double preprocess_ms{0.0};
    double inference_kernel_ms{0.0};
    double postprocess_ms{0.0};
    double normalize_ms{0.0};
  };

  void loadClassConfig() {
    YAML::Node base = readYaml(config_path_);
    YAML::Node overlay = readYaml(overlay_config_path_);
    const YAML::Node merged = mergeYaml(base, overlay);
    const YAML::Node smart = merged["SmartTracker"];
    if (smart && smart["SMART_TRACKER_ALLOWED_CLASS_IDS"]) {
      const YAML::Node ids = smart["SMART_TRACKER_ALLOWED_CLASS_IDS"];
      if (!ids.IsSequence()) {
        throw std::runtime_error("SMART_TRACKER_ALLOWED_CLASS_IDS must be a YAML sequence");
      }
      filter_classes_enabled_ = true;
      for (const YAML::Node &id : ids) allowed_classes_.insert(id.as<int>());
    }
    if (smart && smart["SMART_TRACKER_AGNOSTIC_NMS"]) {
      config_agnostic_nms_ = smart["SMART_TRACKER_AGNOSTIC_NMS"].as<bool>();
    }
    if (smart && smart["SMART_TRACKER_INFERENCE_IMAGE_SIZE"]) {
      default_image_size_ = smart["SMART_TRACKER_INFERENCE_IMAGE_SIZE"].as<int>();
      config_image_size_set_ = true;
    }
  }

  void resolveModelPathAndKey() {
    if (model_path_.empty()) throw std::runtime_error("~model_path is required");
    model_path_ = expandPath(model_path_);
    if (model_path_[0] != '/') {
      const std::string root = ros::package::getPath("sar_yolo_detector");
      if (root.empty()) throw std::runtime_error("cannot locate sar_yolo_detector package");
      model_path_ = root + "/" + model_path_;
    }

    struct stat info;
    if (lstat(model_path_.c_str(), &info) != 0) {
      throw std::runtime_error("model does not exist: " + model_path_);
    }
    if (S_ISLNK(info.st_mode) || !S_ISREG(info.st_mode)) {
      throw std::runtime_error("model must be a regular non-symlink file");
    }
    if (info.st_size <= 0 || info.st_size > 512LL * 1024LL * 1024LL) {
      throw std::runtime_error("model size is outside the configured 1..512 MiB limit");
    }
    if ((info.st_mode & 0022) != 0) {
      throw std::runtime_error("model must not be group/world writable");
    }
    if (!expected_sha256_.empty() && !isHexDigest(expected_sha256_)) {
      throw std::runtime_error("~model_sha256 must contain 64 hexadecimal characters");
    }
    actual_sha256_ = sha256File(model_path_);
    std::transform(expected_sha256_.begin(), expected_sha256_.end(),
                   expected_sha256_.begin(), [](unsigned char ch) {
                     return static_cast<char>(std::tolower(ch));
                   });
    if (!expected_sha256_.empty() && expected_sha256_ != actual_sha256_) {
      throw std::runtime_error("model SHA-256 does not match ~model_sha256");
    }
    model_key_ = expected_sha256_.empty() ? actual_sha256_ : expected_sha256_;
    service_base_ = "/sar_yolo/shared_inference/m_" + model_key_;
  }

  bool acquireModelLock() {
    const std::string lock_directory = lock_dir_.empty() ? "/tmp" : expandPath(lock_dir_);
    const std::string lock_path = lock_directory +
        "/sar_yolo_shared_inference_" + model_key_ + ".lock";
    if (lock_fd_ < 0) {
      lock_fd_ = open(lock_path.c_str(), O_CREAT | O_RDWR, 0600);
      if (lock_fd_ < 0) {
        throw std::runtime_error("cannot open model election lock: " + lock_path);
      }
    }
    if (flock(lock_fd_, LOCK_EX | LOCK_NB) == 0) {
      lock_owned_ = true;
      return true;
    }
    if (errno == EWOULDBLOCK || errno == EAGAIN) return false;
    throw std::runtime_error("cannot acquire model election lock");
  }

  void releaseProviderLock() {
    if (lock_owned_ && lock_fd_ >= 0) {
      flock(lock_fd_, LOCK_UN);
      lock_owned_ = false;
    }
  }

  static std::string metadataJson(const std::string &file, size_t *offset) {
    *offset = 0;
    if (file.size() < 4) return "";
    const auto *bytes = reinterpret_cast<const unsigned char *>(file.data());
    const uint32_t length = static_cast<uint32_t>(bytes[0]) |
        (static_cast<uint32_t>(bytes[1]) << 8) |
        (static_cast<uint32_t>(bytes[2]) << 16) |
        (static_cast<uint32_t>(bytes[3]) << 24);
    if (length == 0 || length > file.size() - 4) return "";
    const std::string candidate = file.substr(4, length);
    try {
      YAML::Node parsed = YAML::Load(candidate);
      if (!parsed.IsMap()) return "";
    } catch (const YAML::Exception &) {
      return "";
    }
    *offset = static_cast<size_t>(4) + length;
    return candidate;
  }

  void loadEngine() {
    if (!use_gpu_) {
      if (fallback_to_cpu_) {
        throw std::runtime_error("TensorRT engine inference requires CUDA; CPU fallback is unavailable");
      }
      throw std::runtime_error("TensorRT .engine artifacts require CUDA");
    }
    const int device_count = []() {
      int count = 0;
      const cudaError_t status = cudaGetDeviceCount(&count);
      return status == cudaSuccess ? count : 0;
    }();
    if (device_count <= 0) throw std::runtime_error("CUDA device is unavailable");
    requireCuda(cudaSetDevice(0), "cudaSetDevice");

    const std::string contents = readFile(model_path_);
    size_t engine_offset = 0;
    const std::string metadata_json = metadataJson(contents, &engine_offset);
    if (engine_offset == 0) {
      throw std::runtime_error("Ultralytics TensorRT engine metadata header is missing or invalid");
    }
    const YAML::Node metadata = YAML::Load(metadata_json);
    if (metadata["task"]) model_task_ = metadata["task"].as<std::string>();
    if (model_task_ != "detect") {
      throw std::runtime_error("C++ TensorRT provider currently requires a detect engine");
    }
    if (metadata["names"] && metadata["names"].IsMap()) {
      for (const auto &entry : metadata["names"]) {
        const int class_id = std::stoi(entry.first.as<std::string>());
        labels_[class_id] = entry.second.as<std::string>();
      }
    }
    if (labels_.empty()) {
      throw std::runtime_error("model metadata does not contain class names");
    }

    initLibNvInferPlugins(&logger_, "");
    runtime_.reset(nvinfer1::createInferRuntime(logger_));
    if (!runtime_) throw std::runtime_error("TensorRT runtime creation failed");
    engine_.reset(runtime_->deserializeCudaEngine(contents.data() + engine_offset,
                                                   contents.size() - engine_offset));
    if (!engine_) throw std::runtime_error("TensorRT engine deserialization failed");
    context_.reset(engine_->createExecutionContext());
    if (!context_) throw std::runtime_error("TensorRT execution context creation failed");
    requireCuda(cudaStreamCreateWithFlags(&stream_, cudaStreamNonBlocking),
                "create TensorRT CUDA stream");
    requireCuda(cudaEventCreate(&kernel_start_), "create inference start event");
    requireCuda(cudaEventCreate(&kernel_end_), "create inference end event");

    if (engine_->getNbBindings() != 2) {
      throw std::runtime_error("expected one input and one output binding in the YOLO engine");
    }
    for (int index = 0; index < engine_->getNbBindings(); ++index) {
      if (engine_->bindingIsInput(index)) {
        if (input_binding_ >= 0) throw std::runtime_error("multiple engine inputs are unsupported");
        input_binding_ = index;
      } else {
        if (output_binding_ >= 0) throw std::runtime_error("multiple engine outputs are unsupported");
        output_binding_ = index;
      }
    }
    if (input_binding_ < 0 || output_binding_ < 0) {
      throw std::runtime_error("engine input/output bindings are invalid");
    }
    const nvinfer1::Dims input_dims = engine_->getBindingDimensions(input_binding_);
    if (input_dims.nbDims != 4 || input_dims.d[1] != 3) {
      throw std::runtime_error("YOLO engine input must have NCHW RGB dimensions");
    }
    dynamic_input_ = std::find(input_dims.d, input_dims.d + input_dims.nbDims, -1) !=
                     input_dims.d + input_dims.nbDims;
    if (dynamic_input_) {
      profile_min_ = engine_->getProfileDimensions(
          0, input_binding_, nvinfer1::OptProfileSelector::kMIN);
      profile_max_ = engine_->getProfileDimensions(
          0, input_binding_, nvinfer1::OptProfileSelector::kMAX);
      if (profile_min_.nbDims != 4 || profile_max_.nbDims != 4) {
        throw std::runtime_error("engine dynamic optimization profile is invalid");
      }
      maximum_batch_size_ = std::max(1, std::min(requested_batch_size_, profile_max_.d[0]));
    } else {
      maximum_batch_size_ = std::max(1, std::min(requested_batch_size_, input_dims.d[0]));
    }

    if (!config_image_size_set_ && metadata["imgsz"] &&
        metadata["imgsz"].IsSequence() && metadata["imgsz"].size() > 0) {
      default_image_size_ = metadata["imgsz"][0].as<int>();
    }
    if (default_image_size_ < 320 || default_image_size_ > 1536 ||
        default_image_size_ % 32 != 0) {
      throw std::runtime_error("configured inference image size must be a multiple of 32 within 320..1536");
    }

    runtime_json_ = makeRuntimeJson(metadata);
    ROS_INFO_STREAM("shared TensorRT C++ provider ready: key=" << model_key_.substr(0, 12)
                    << " backend=tensorrt_cpp device=cuda batch=" << maximum_batch_size_
                    << " model=" << model_path_);
  }

  std::string makeRuntimeJson(const YAML::Node &metadata) const {
    const size_t slash = model_path_.find_last_of('/');
    std::string model_name = model_path_.substr(slash == std::string::npos ? 0 : slash + 1);
    std::ostringstream out;
    out << "{\"backend\":\"tensorrt_cpp\",\"model_format\":\"tensorrt_engine\","
        << "\"requested_device\":\"cuda\",\"effective_device\":\"cuda\","
        << "\"device\":\"cuda:0\","
        << "\"model_path\":\"" << jsonEscape(model_path_) << "\","
        << "\"requested_model_path\":\"" << jsonEscape(model_path_) << "\","
        << "\"model_name\":\"" << jsonEscape(model_name) << "\","
        << "\"model_task\":\"" << jsonEscape(model_task_) << "\","
        << "\"shared_model_key\":\"" << model_key_ << "\","
        << "\"artifact_sha256\":\"" << actual_sha256_ << "\","
        << "\"verified\":" << (expected_sha256_.empty() ? "false" : "true")
        << ",\"precision\":\""
        << (metadata["precision"] ? jsonEscape(metadata["precision"].as<std::string>()) : "unknown")
        << "\",\"maximum_batch_size\":" << maximum_batch_size_ << '}';
    return out.str();
  }

  void warmup() {
    const int size = default_image_size_;
    cv::Mat image(size, size, CV_8UC3, cv::Scalar(0, 0, 0));
    auto request = std::make_shared<PendingRequest>();
    request->source_id = "startup_warmup";
    request->frame = image;
    request->confidence = 1.0f;
    request->iou = 0.3f;
    request->max_detections = 1;
    request->image_size = size;
    const auto started = Clock::now();
    std::vector<std::shared_ptr<PendingRequest>> requests{request};
    inferBatch(requests, false);
    startup_warmup_ms_ = std::chrono::duration<double, std::milli>(Clock::now() - started).count();
    ROS_INFO("shared TensorRT C++ warmup completed in %.1f ms", startup_warmup_ms_);
  }

  void startProvider() {
    closing_.store(false);
    worker_ = std::thread(&SharedInferenceServer::workerLoop, this);
    info_service_ = nh_.advertiseService(service_base_ + "/info",
        &SharedInferenceServer::infoCallback, this);
    detect_service_ = nh_.advertiseService(service_base_ + "/detect",
        &SharedInferenceServer::detectCallback, this);
    ROS_INFO_STREAM("shared TensorRT C++ service available at " << service_base_);
    ros::AsyncSpinner spinner(std::max(4, maximum_batch_size_ + 2));
    spinner.start();
    ros::waitForShutdown();
    spinner.stop();
    info_service_.shutdown();
    detect_service_.shutdown();
    stopWorker();
    releaseProviderLock();
  }

  bool infoCallback(sar_yolo_detector::GetSharedModelInfo::Request &request,
                    sar_yolo_detector::GetSharedModelInfo::Response &response) {
    response.success = request.model_key == model_key_ && engine_ != nullptr;
    response.message = response.success ? "" : "shared model key mismatch or provider not ready";
    if (response.success) {
      response.runtime_json = runtime_json_;
      response.labels_json = jsonStringMap(labels_);
      response.model_task = model_task_;
    }
    return true;
  }

  bool detectCallback(sar_yolo_detector::RunSharedInference::Request &request,
                      sar_yolo_detector::RunSharedInference::Response &response) {
    if (request.model_key != model_key_ || !engine_) {
      response.success = false;
      response.message = "shared model key mismatch or provider not ready";
      return true;
    }
    if (isStale(request.image.header.stamp, nullptr)) {
      double age = 0.0;
      isStale(request.image.header.stamp, &age);
      response.success = false;
      std::ostringstream message;
      message << std::fixed << std::setprecision(3)
              << "stale image discarded by shared inference (age " << age << "s)";
      response.message = message.str();
      failure_count_.fetch_add(1);
      return true;
    }
    try {
      cv::Mat converted = imageToBgr(request.image);
      if (converted.empty()) {
        throw std::runtime_error("image conversion returned an empty frame");
      }
      auto pending = std::make_shared<PendingRequest>();
      pending->source_id = request.source_id.empty() ? "unknown" : request.source_id;
      pending->stamp = request.image.header.stamp;
      pending->frame = std::move(converted);
      pending->confidence = request.confidence;
      pending->iou = request.iou;
      pending->max_detections = std::max<uint32_t>(1, request.max_detections);
      pending->image_size = request.image_size;
      enqueue(pending);

      std::unique_lock<std::mutex> lock(pending->response_mutex);
      const double wait_sec = std::max(2.0, maximum_age_sec_ + 2.0);
      if (!pending->response_condition.wait_for(
              lock, std::chrono::duration<double>(wait_sec),
              [&pending]() { return pending->completed.load(); })) {
        pending->response.success = false;
        pending->response.message = "shared inference queue timed out";
        pending->completed.store(true);
        failure_count_.fetch_add(1);
      }
      response = pending->response;
      return true;
    } catch (const std::exception &error) {
      response.success = false;
      response.message = std::string(typeid(error).name()) + ": " + error.what();
      failure_count_.fetch_add(1);
      ROS_WARN_STREAM_THROTTLE(2.0, "shared TensorRT inference failed: " << response.message);
      return true;
    }
  }

  bool isStale(const ros::Time &stamp, double *age) const {
    if (maximum_age_sec_ <= 0.0 || stamp.isZero()) return false;
    const ros::Time now = ros::Time::now();
    if (now.isZero()) return false;
    const double value = (now - stamp).toSec();
    if (age != nullptr) *age = value;
    return value > maximum_age_sec_;
  }

  bool compatible(const PendingRequest &first, const PendingRequest &other) const {
    return first.frame.size() == other.frame.size() &&
           first.frame.type() == other.frame.type() &&
           first.confidence == other.confidence && first.iou == other.iou &&
           first.max_detections == other.max_detections &&
           first.image_size == other.image_size;
  }

  void enqueue(const std::shared_ptr<PendingRequest> &pending) {
    std::shared_ptr<PendingRequest> superseded;
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      auto previous_it = pending_by_source_.find(pending->source_id);
      if (previous_it != pending_by_source_.end()) {
        superseded = previous_it->second.lock();
        if (superseded && !superseded->completed.load()) {
          auto queued = std::find(pending_queue_.begin(), pending_queue_.end(), superseded);
          if (queued != pending_queue_.end()) pending_queue_.erase(queued);
          else superseded.reset();
        } else {
          superseded.reset();
        }
      }
      pending_by_source_[pending->source_id] = pending;
      pending_queue_.push_back(pending);
    }
    if (superseded) {
      sar_yolo_detector::RunSharedInference::Response response;
      response.success = false;
      response.message = "superseded by a newer frame";
      completeRequest(superseded, response);
      failure_count_.fetch_add(1);
    }
    queue_condition_.notify_one();
  }

  std::vector<std::shared_ptr<PendingRequest>> takeBatch() {
    std::unique_lock<std::mutex> lock(queue_mutex_);
    queue_condition_.wait(lock, [this]() {
      return closing_.load() || !pending_queue_.empty();
    });
    if (closing_.load()) return {};
    auto first = pending_queue_.front();
    pending_queue_.pop_front();
    auto map_entry = pending_by_source_.find(first->source_id);
    if (map_entry != pending_by_source_.end() && map_entry->second.lock() == first) {
      pending_by_source_.erase(map_entry);
    }
    std::vector<std::shared_ptr<PendingRequest>> batch{first};
    const auto deadline = Clock::now() + std::chrono::duration<double, std::milli>(batch_wait_ms_);
    while (batch.size() < static_cast<size_t>(maximum_batch_size_) && !closing_.load()) {
      auto selected = pending_queue_.end();
      for (auto candidate = pending_queue_.begin(); candidate != pending_queue_.end();) {
        if ((*candidate)->completed.load()) {
          candidate = pending_queue_.erase(candidate);
          continue;
        }
        if (compatible(*first, **candidate)) {
          selected = candidate;
          break;
        }
        ++candidate;
      }
      if (selected != pending_queue_.end()) {
        auto item = *selected;
        pending_queue_.erase(selected);
        auto item_entry = pending_by_source_.find(item->source_id);
        if (item_entry != pending_by_source_.end() && item_entry->second.lock() == item) {
          pending_by_source_.erase(item_entry);
        }
        batch.push_back(item);
        continue;
      }
      if (queue_condition_.wait_until(lock, deadline) == std::cv_status::timeout) break;
    }
    return batch;
  }

  LetterboxInfo makeLetterbox(const cv::Mat &frame, int image_size) const {
    if (frame.empty() || frame.cols <= 0 || frame.rows <= 0) {
      throw std::runtime_error("invalid image dimensions");
    }
    const double gain = std::min(static_cast<double>(image_size) / frame.cols,
                                 static_cast<double>(image_size) / frame.rows);
    const int resized_width = static_cast<int>(std::round(frame.cols * gain));
    const int resized_height = static_cast<int>(std::round(frame.rows * gain));
    int width_padding = image_size - resized_width;
    int height_padding = image_size - resized_height;
    // The dynamic Ultralytics engine uses rect=True: reduce padding to the
    // minimum stride-aligned rectangle when every batch image has one shape.
    if (dynamic_input_) {
      width_padding %= 32;
      height_padding %= 32;
    }
    const int left = static_cast<int>(std::round(width_padding / 2.0 - 0.1));
    const int top = static_cast<int>(std::round(height_padding / 2.0 - 0.1));
    LetterboxInfo result;
    result.width = resized_width + width_padding;
    result.height = resized_height + height_padding;
    result.resized_width = resized_width;
    result.resized_height = resized_height;
    result.left = left;
    result.top = top;
    result.gain = gain;
    return result;
  }

  void fillInput(const std::vector<std::shared_ptr<PendingRequest>> &batch,
                 int image_size, int input_height, int input_width,
                 size_t input_element_count) {
    if (batch.empty()) throw std::runtime_error("cannot preprocess an empty batch");
    const size_t expected = batch.size() * 3ULL * input_height * input_width;
    if (expected != input_element_count) {
      throw std::runtime_error("engine input dimensions do not match prepared images");
    }
    auto write_value = [this](size_t index, float value) {
      if (input_type_ == nvinfer1::DataType::kFLOAT) {
        static_cast<float *>(host_input_)[index] = value;
      } else if (input_type_ == nvinfer1::DataType::kHALF) {
        static_cast<__half *>(host_input_)[index] = __float2half(value);
      } else {
        throw std::runtime_error("engine input must use FP32 or FP16 data");
      }
    };
    size_t base = 0;
    for (size_t b = 0; b < batch.size(); ++b) {
      const cv::Mat &frame = batch[b]->frame;
      const LetterboxInfo info = makeLetterbox(frame, image_size);
      if (info.width != input_width || info.height != input_height) {
        throw std::runtime_error("batch images produced inconsistent letterbox shapes");
      }
      cv::Mat resized;
      cv::resize(frame, resized, cv::Size(info.resized_width, info.resized_height),
                 0.0, 0.0, cv::INTER_LINEAR);
      cv::Mat canvas(input_height, input_width, CV_8UC3,
                     cv::Scalar(114, 114, 114));
      resized.copyTo(canvas(cv::Rect(info.left, info.top,
                                     info.resized_width, info.resized_height)));
      const size_t plane = static_cast<size_t>(input_height) * input_width;
      const size_t sample_base = base + b * 3ULL * plane;
      for (int y = 0; y < input_height; ++y) {
        const cv::Vec3b *row = canvas.ptr<cv::Vec3b>(y);
        for (int x = 0; x < input_width; ++x) {
          const size_t pixel = static_cast<size_t>(y) * input_width + x;
          // Ultralytics converts BGR to RGB and normalizes to [0,1].
          write_value(sample_base + pixel, row[x][2] / 255.0f);
          write_value(sample_base + plane + pixel, row[x][1] / 255.0f);
          write_value(sample_base + 2ULL * plane + pixel, row[x][0] / 255.0f);
        }
      }
    }
  }

  void ensureBuffer(void **buffer, size_t *capacity, size_t needed,
                    const char *label) {
    if (*capacity >= needed) return;
    if (*buffer != nullptr) requireCuda(cudaFree(*buffer), "cudaFree old buffer");
    *buffer = nullptr;
    *capacity = 0;
    const cudaError_t status = cudaMalloc(buffer, needed);
    if (status != cudaSuccess) {
      throw std::runtime_error(std::string("cudaMalloc ") + label + ": " +
                               cudaGetErrorString(status));
    }
    *capacity = needed;
  }

  void ensureHostBuffer(void **buffer, size_t *capacity, size_t needed,
                        const char *label) {
    if (*capacity >= needed) return;
    if (*buffer != nullptr) requireCuda(cudaFreeHost(*buffer), "cudaFreeHost old buffer");
    *buffer = nullptr;
    *capacity = 0;
    const cudaError_t status = cudaHostAlloc(buffer, needed, cudaHostAllocDefault);
    if (status != cudaSuccess) {
      throw std::runtime_error(std::string("cudaHostAlloc ") + label + ": " +
                               cudaGetErrorString(status));
    }
    *capacity = needed;
  }

  size_t dataTypeSize(nvinfer1::DataType type) const {
    switch (type) {
      case nvinfer1::DataType::kFLOAT: return sizeof(float);
      case nvinfer1::DataType::kHALF: return sizeof(__half);
      case nvinfer1::DataType::kINT8: return sizeof(int8_t);
      case nvinfer1::DataType::kINT32: return sizeof(int32_t);
      case nvinfer1::DataType::kBOOL: return sizeof(bool);
      default: throw std::runtime_error("unsupported TensorRT binding data type");
    }
  }

  static size_t dimensionsVolume(const nvinfer1::Dims &dimensions) {
    size_t total = 1;
    for (int index = 0; index < dimensions.nbDims; ++index) {
      if (dimensions.d[index] <= 0) throw std::runtime_error("TensorRT output has unresolved dimensions");
      total *= static_cast<size_t>(dimensions.d[index]);
    }
    return total;
  }

  float outputValue(size_t index) const {
    if (output_type_ == nvinfer1::DataType::kFLOAT) {
      return static_cast<const float *>(host_output_)[index];
    }
    if (output_type_ == nvinfer1::DataType::kHALF) {
      return __half2float(static_cast<const __half *>(host_output_)[index]);
    }
    throw std::runtime_error("YOLO output must use FP32 or FP16 data");
  }

  std::vector<sar_yolo_detector::NormalizedDetection> decodeDetections(
      const PendingRequest &request, int batch_index, const nvinfer1::Dims &output_dims,
      size_t output_volume, int input_height, int input_width,
      float confidence_threshold, float iou_threshold, uint32_t max_detections) const {
    if (output_dims.nbDims != 3 || output_dims.d[0] <= batch_index) {
      throw std::runtime_error("expected batched YOLO output with three dimensions");
    }
    const int class_count = static_cast<int>(labels_.size());
    int channels = 0;
    int prediction_count = 0;
    bool channels_first = false;
    if (output_dims.d[1] == class_count + 4) {
      channels = output_dims.d[1];
      prediction_count = output_dims.d[2];
      channels_first = true;
    } else if (output_dims.d[2] == class_count + 4) {
      prediction_count = output_dims.d[1];
      channels = output_dims.d[2];
      channels_first = false;
    } else {
      throw std::runtime_error("YOLO output does not match the engine class metadata");
    }
    const size_t per_batch = output_volume / static_cast<size_t>(output_dims.d[0]);
    if (per_batch != static_cast<size_t>(channels) * prediction_count) {
      throw std::runtime_error("YOLO output tensor size does not match its shape");
    }
    const int image_size = request.image_size > 0 ? request.image_size : default_image_size_;
    const LetterboxInfo letterbox = makeLetterbox(request.frame, image_size);
    if (letterbox.width != input_width || letterbox.height != input_height) {
      throw std::runtime_error("letterbox output and TensorRT input shape differ");
    }

    std::vector<Candidate> candidates;
    candidates.reserve(prediction_count / 8);
    const size_t batch_offset = static_cast<size_t>(batch_index) * per_batch;
    for (int prediction = 0; prediction < prediction_count; ++prediction) {
      auto value = [&](int channel) {
        const size_t index = channels_first
            ? batch_offset + static_cast<size_t>(channel) * prediction_count + prediction
            : batch_offset + static_cast<size_t>(prediction) * channels + channel;
        return outputValue(index);
      };
      int best_class = -1;
      float best_score = -std::numeric_limits<float>::infinity();
      for (int class_id = 0; class_id < class_count; ++class_id) {
        const float score = value(4 + class_id);
        if (score > best_score) {
          best_score = score;
          best_class = class_id;
        }
      }
      if (best_class < 0 || !(best_score > confidence_threshold)) continue;
      if (filter_classes_enabled_ && allowed_classes_.count(best_class) == 0) continue;

      const float center_x = value(0);
      const float center_y = value(1);
      const float width = value(2);
      const float height = value(3);
      Candidate candidate;
      // Ultralytics applies NMS in letterboxed input coordinates, then clips
      // the selected boxes while scaling them back to the source frame.
      candidate.x1 = center_x - width * 0.5f;
      candidate.y1 = center_y - height * 0.5f;
      candidate.x2 = center_x + width * 0.5f;
      candidate.y2 = center_y + height * 0.5f;
      if (candidate.x2 <= candidate.x1 || candidate.y2 <= candidate.y1) continue;
      candidate.confidence = best_score;
      candidate.class_id = best_class;
      candidates.push_back(candidate);
    }

    const bool agnostic = config_agnostic_nms_;
    std::vector<Candidate> selected = nonMaxSuppression(
        std::move(candidates), iou_threshold, max_detections, agnostic);
    std::vector<sar_yolo_detector::NormalizedDetection> detections;
    detections.reserve(selected.size());
    for (size_t index = 0; index < selected.size(); ++index) {
      const Candidate &candidate = selected[index];
      sar_yolo_detector::NormalizedDetection detection;
      detection.track_id = -static_cast<int64_t>(index + 1);
      detection.class_id = candidate.class_id;
      detection.confidence = candidate.confidence;
      const int x1 = static_cast<int>(std::max(0.0f,
          (candidate.x1 - letterbox.left) / static_cast<float>(letterbox.gain)));
      const int y1 = static_cast<int>(std::max(0.0f,
          (candidate.y1 - letterbox.top) / static_cast<float>(letterbox.gain)));
      const int x2 = static_cast<int>(std::min(static_cast<float>(request.frame.cols),
          (candidate.x2 - letterbox.left) / static_cast<float>(letterbox.gain)));
      const int y2 = static_cast<int>(std::min(static_cast<float>(request.frame.rows),
          (candidate.y2 - letterbox.top) / static_cast<float>(letterbox.gain)));
      detection.aabb_xyxy[0] = static_cast<float>(x1);
      detection.aabb_xyxy[1] = static_cast<float>(y1);
      detection.aabb_xyxy[2] = static_cast<float>(x2);
      detection.aabb_xyxy[3] = static_cast<float>(y2);
      detection.center_xy[0] = (x1 + x2) / 2;
      detection.center_xy[1] = (y1 + y2) / 2;
      detection.geometry_type = "aabb";
      detection.obb_xywhr[0] = 0.0f;
      detection.obb_xywhr[1] = 0.0f;
      detection.obb_xywhr[2] = 0.0f;
      detection.obb_xywhr[3] = 0.0f;
      detection.obb_xywhr[4] = 0.0f;
      detection.polygon_xy.clear();
      detection.rotation_deg = 0.0f;
      detection.has_rotation = false;
      detection.track_id_is_stable = false;
      detections.push_back(detection);
    }
    return detections;
  }

  void inferBatch(const std::vector<std::shared_ptr<PendingRequest>> &batch,
                  bool publish_results) {
    if (batch.empty()) return;
    const auto total_started = Clock::now();
    const int image_size = batch.front()->image_size > 0
        ? batch.front()->image_size : default_image_size_;
    if (image_size < 32 || image_size > 1536 || image_size % 32 != 0) {
      throw std::runtime_error("inference image_size must be a multiple of 32 within 32..1536");
    }
    const LetterboxInfo prepared = makeLetterbox(batch.front()->frame, image_size);
    const int batch_size = static_cast<int>(batch.size());
    nvinfer1::Dims4 dimensions(batch_size, 3, prepared.height, prepared.width);
    const nvinfer1::Dims current_dimensions = engine_->getBindingDimensions(input_binding_);
    if (dynamic_input_) {
      for (int axis = 0; axis < 4; ++axis) {
        if (dimensions.d[axis] < profile_min_.d[axis] || dimensions.d[axis] > profile_max_.d[axis]) {
          throw std::runtime_error("requested input dimensions are outside the TensorRT profile");
        }
      }
      if (!context_->setBindingDimensions(input_binding_, dimensions)) {
        throw std::runtime_error("TensorRT rejected input dimensions");
      }
    } else if (dimensions.d[0] != current_dimensions.d[0] ||
               dimensions.d[1] != current_dimensions.d[1] ||
               dimensions.d[2] != current_dimensions.d[2] ||
               dimensions.d[3] != current_dimensions.d[3]) {
      throw std::runtime_error("input dimensions do not match the static TensorRT engine");
    }
    if (!context_->allInputDimensionsSpecified()) {
      throw std::runtime_error("TensorRT input dimensions are incomplete");
    }
    const nvinfer1::Dims output_dims = context_->getBindingDimensions(output_binding_);
    const size_t input_elements = dimensionsVolume(dimensions);
    const size_t output_elements = dimensionsVolume(output_dims);
    input_type_ = engine_->getBindingDataType(input_binding_);
    output_type_ = engine_->getBindingDataType(output_binding_);
    const size_t input_bytes = input_elements * dataTypeSize(input_type_);
    const size_t output_bytes = output_elements * dataTypeSize(output_type_);
    ensureBuffer(&device_input_, &device_input_capacity_, input_bytes, "input");
    ensureBuffer(&device_output_, &device_output_capacity_, output_bytes, "output");
    ensureHostBuffer(&host_input_, &host_input_capacity_, input_bytes, "input");
    ensureHostBuffer(&host_output_, &host_output_capacity_, output_bytes, "output");

    const auto preprocess_started = Clock::now();
    fillInput(batch, image_size, prepared.height, prepared.width, input_elements);
    const double preprocess_ms = std::chrono::duration<double, std::milli>(
        Clock::now() - preprocess_started).count();

    std::vector<void *> bindings(engine_->getNbBindings(), nullptr);
    bindings[input_binding_] = device_input_;
    bindings[output_binding_] = device_output_;
    requireCuda(cudaMemcpyAsync(device_input_, host_input_, input_bytes,
                               cudaMemcpyHostToDevice, stream_), "copy TensorRT input");
    requireCuda(cudaEventRecord(kernel_start_, stream_), "record inference start");
    if (!context_->enqueueV2(bindings.data(), stream_, nullptr)) {
      throw std::runtime_error("TensorRT enqueueV2 failed");
    }
    requireCuda(cudaEventRecord(kernel_end_, stream_), "record inference end");
    requireCuda(cudaMemcpyAsync(host_output_, device_output_, output_bytes,
                               cudaMemcpyDeviceToHost, stream_), "copy TensorRT output");
    requireCuda(cudaStreamSynchronize(stream_), "synchronize TensorRT inference");
    float kernel_ms = 0.0f;
    requireCuda(cudaEventElapsedTime(&kernel_ms, kernel_start_, kernel_end_),
                "read TensorRT inference duration");

    const auto postprocess_started = Clock::now();
    std::vector<std::vector<sar_yolo_detector::NormalizedDetection>> decoded;
    decoded.reserve(batch.size());
    for (size_t index = 0; index < batch.size(); ++index) {
      decoded.push_back(decodeDetections(*batch[index], static_cast<int>(index),
          output_dims, output_elements, prepared.height, prepared.width,
          batch[index]->confidence, batch[index]->iou,
          std::max<uint32_t>(1, batch[index]->max_detections)));
    }
    const double postprocess_ms = std::chrono::duration<double, std::milli>(
        Clock::now() - postprocess_started).count();
    const double total_ms = std::chrono::duration<double, std::milli>(
        Clock::now() - total_started).count();

    if (publish_results) {
      for (size_t index = 0; index < batch.size(); ++index) {
        sar_yolo_detector::RunSharedInference::Response response;
        response.success = true;
        response.message.clear();
        response.inference_ms = total_ms;
        response.detections = std::move(decoded[index]);
        completeRequest(batch[index], response);
        request_count_.fetch_add(1);
      }
      recordTiming(batch.size(), total_ms, preprocess_ms, kernel_ms, postprocess_ms);
    }
  }

  void recordTiming(size_t request_count, double inference_ms,
                    double preprocess_ms, double kernel_ms, double postprocess_ms) {
    batch_counts_[request_count]++;
    batch_stats_requests_ += request_count;
    batch_inference_ms_total_ += inference_ms;
    stage_totals_[0] += preprocess_ms;
    stage_totals_[1] += kernel_ms;
    stage_totals_[2] += postprocess_ms;
    ++stage_samples_;
    const auto now = Clock::now();
    const double elapsed = std::chrono::duration<double>(now - batch_stats_started_).count();
    if (elapsed < 10.0) return;
    const size_t batches = std::accumulate(batch_counts_.begin(), batch_counts_.end(), size_t(0));
    const double average = batches > 0 ? batch_inference_ms_total_ / batches : 0.0;
    const double stage_count = std::max<size_t>(1, stage_samples_);
    std::ostringstream mix;
    bool first = true;
    for (size_t size = 1; size < batch_counts_.size(); ++size) {
      if (!first) mix << ',';
      first = false;
      mix << size << ':' << batch_counts_[size];
    }
    ROS_INFO("shared YOLO perf: requests=%llu requests_per_sec=%.2f batches=%llu "
             "batch_mix=%s batch_inference_ms_avg=%.2f max_batch=%d "
             "preprocess_ms_avg=%.2f model_forward_ms_avg=%.2f "
             "postprocess_ms_avg=%.2f normalize_ms_avg=%.2f",
             static_cast<unsigned long long>(batch_stats_requests_),
             batch_stats_requests_ / elapsed,
             static_cast<unsigned long long>(batches), mix.str().c_str(), average,
             maximum_batch_size_, stage_totals_[0] / stage_count,
             stage_totals_[1] / stage_count, stage_totals_[2] / stage_count, 0.0);
    batch_counts_.fill(0);
    batch_stats_requests_ = 0;
    batch_inference_ms_total_ = 0.0;
    stage_totals_.fill(0.0);
    stage_samples_ = 0;
    batch_stats_started_ = now;
  }

  void workerLoop() {
    while (ros::ok() && !closing_.load()) {
      std::vector<std::shared_ptr<PendingRequest>> batch = takeBatch();
      if (batch.empty()) continue;
      std::vector<std::shared_ptr<PendingRequest>> fresh;
      const ros::Time now = ros::Time::now();
      for (const auto &pending : batch) {
        const double age = pending->stamp.isZero() || now.isZero()
            ? 0.0 : (now - pending->stamp).toSec();
        if (maximum_age_sec_ > 0.0 && age > maximum_age_sec_) {
          sar_yolo_detector::RunSharedInference::Response response;
          response.success = false;
          std::ostringstream message;
          message << std::fixed << std::setprecision(3)
                  << "stale image discarded by shared inference (age " << age << "s)";
          response.message = message.str();
          completeRequest(pending, response);
          failure_count_.fetch_add(1);
        } else if (!pending->completed.load()) {
          fresh.push_back(pending);
        }
      }
      if (fresh.empty()) continue;
      try {
        inferBatch(fresh, true);
      } catch (const std::exception &error) {
        const std::string message = std::string(typeid(error).name()) + ": " + error.what();
        ROS_ERROR_STREAM_THROTTLE(2.0, "shared YOLO inference failed: " << message);
        for (const auto &pending : fresh) {
          sar_yolo_detector::RunSharedInference::Response response;
          response.success = false;
          response.message = message;
          completeRequest(pending, response);
          failure_count_.fetch_add(1);
        }
      }
    }
  }

  void stopWorker() {
    closing_.store(true);
    queue_condition_.notify_all();
    std::vector<std::shared_ptr<PendingRequest>> pending;
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      pending.assign(pending_queue_.begin(), pending_queue_.end());
      pending_queue_.clear();
      pending_by_source_.clear();
    }
    for (const auto &item : pending) {
      sar_yolo_detector::RunSharedInference::Response response;
      response.success = false;
      response.message = "shared inference provider is stopping";
      completeRequest(item, response);
    }
    if (worker_.joinable() && worker_.get_id() != std::this_thread::get_id()) worker_.join();
  }

  void releaseCuda() {
    if (kernel_start_ != nullptr) { cudaEventDestroy(kernel_start_); kernel_start_ = nullptr; }
    if (kernel_end_ != nullptr) { cudaEventDestroy(kernel_end_); kernel_end_ = nullptr; }
    if (stream_ != nullptr) { cudaStreamDestroy(stream_); stream_ = nullptr; }
    if (device_input_ != nullptr) { cudaFree(device_input_); device_input_ = nullptr; }
    if (device_output_ != nullptr) { cudaFree(device_output_); device_output_ = nullptr; }
    if (host_input_ != nullptr) { cudaFreeHost(host_input_); host_input_ = nullptr; }
    if (host_output_ != nullptr) { cudaFreeHost(host_output_); host_output_ = nullptr; }
    device_input_capacity_ = device_output_capacity_ = 0;
    host_input_capacity_ = host_output_capacity_ = 0;
    context_.reset();
    engine_.reset();
    runtime_.reset();
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  TrtLogger logger_;
  RuntimePtr runtime_;
  EnginePtr engine_;
  ContextPtr context_;
  ros::ServiceServer info_service_;
  ros::ServiceServer detect_service_;
  std::string model_path_;
  std::string expected_sha256_;
  std::string actual_sha256_;
  std::string model_key_;
  std::string service_base_;
  std::string config_path_;
  std::string overlay_config_path_;
  std::string lock_dir_;
  std::string model_task_{"detect"};
  std::string runtime_json_;
  std::unordered_map<int, std::string> labels_;
  std::unordered_set<int> allowed_classes_;
  int lock_fd_{-1};
  bool lock_owned_{false};
  bool use_gpu_{true};
  bool fallback_to_cpu_{false};
  bool startup_warmup_{true};
  bool dynamic_input_{false};
  bool config_agnostic_nms_{false};
  bool filter_classes_enabled_{false};
  int requested_batch_size_{2};
  int maximum_batch_size_{1};
  int default_image_size_{640};
  bool config_image_size_set_{false};
  int opencv_threads_{1};
  int input_binding_{-1};
  int output_binding_{-1};
  int input_height_{0};
  int input_width_{0};
  int input_channels_{3};
  nvinfer1::Dims profile_min_{};
  nvinfer1::Dims profile_max_{};
  nvinfer1::DataType input_type_{nvinfer1::DataType::kFLOAT};
  nvinfer1::DataType output_type_{nvinfer1::DataType::kFLOAT};
  double batch_wait_ms_{4.0};
  double maximum_age_sec_{0.5};
  double startup_warmup_ms_{0.0};
  cudaStream_t stream_{nullptr};
  cudaEvent_t kernel_start_{nullptr};
  cudaEvent_t kernel_end_{nullptr};
  void *device_input_{nullptr};
  void *device_output_{nullptr};
  void *host_input_{nullptr};
  void *host_output_{nullptr};
  size_t device_input_capacity_{0};
  size_t device_output_capacity_{0};
  size_t host_input_capacity_{0};
  size_t host_output_capacity_{0};
  std::mutex queue_mutex_;
  std::condition_variable queue_condition_;
  std::deque<std::shared_ptr<PendingRequest>> pending_queue_;
  std::unordered_map<std::string, std::weak_ptr<PendingRequest>> pending_by_source_;
  std::thread worker_;
  std::atomic<bool> closing_{false};
  std::atomic<uint64_t> request_count_{0};
  std::atomic<uint64_t> failure_count_{0};
  std::array<uint64_t, 9> batch_counts_{};
  uint64_t batch_stats_requests_{0};
  double batch_inference_ms_total_{0.0};
  std::array<double, 3> stage_totals_{};
  uint64_t stage_samples_{0};
  Clock::time_point batch_stats_started_{Clock::now()};
};

}  // namespace

int main(int argc, char **argv) {
  ros::init(argc, argv, "sar_yolo_shared_model_server_cpp");
  try {
    SharedInferenceServer server;
    server.run();
  } catch (const std::exception &error) {
    ROS_FATAL_STREAM("shared TensorRT C++ provider stopped: " << error.what());
    return 1;
  }
  return 0;
}
