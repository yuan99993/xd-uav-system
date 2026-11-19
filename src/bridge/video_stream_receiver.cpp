#include <arpa/inet.h>
#include <netdb.h>
#include <sys/socket.h>
#include <unistd.h>

#include <gst/app/gstappsink.h>
#include <gst/gst.h>
#include <gst/video/video.h>

#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {

using Clock = std::chrono::steady_clock;
constexpr int kRtpPayloadType = 96;
constexpr int kReceiveBufferBytes = 8 * 1024 * 1024;
std::atomic<bool> g_stop{false};

void onSignal(int) { g_stop.store(true); }

struct Options {
  std::string server{"192.168.2.98"};
  int control_port{49999};
  std::string camera{"FrontCamera"};
  std::string image_type{"scene"};
  std::string group;
  int port{0};
  std::string local_ip;
  bool display{true};
  double scale{1.0};
  std::string decoder{"hardware"};
  std::string dump_path;
  std::string image_topic;
  std::string camera_info_topic;
  std::string camera_info_output_topic;
  std::string node_name{"airsim_video_receiver"};
  double timeout{5.0};
  double subscribe_retry_sec{2.0};
};

void printUsage(const char* program) {
  std::cout
      << "Receive and decode a ProjectAirSim H.264 camera stream.\n"
      << "Usage: " << program << " [options]\n"
      << "  --server IP --control-port N --camera NAME --image-type TYPE\n"
      << "  --group IP --port N --local-ip IP --decoder hardware|software\n"
      << "  --dump FILE --no-display --scale FACTOR\n"
      << "  --ros-image-topic TOPIC --ros-camera-info-topic TOPIC\n"
      << "  --ros-camera-info-output-topic TOPIC --ros-node-name NAME\n"
      << "  --timeout SEC --subscribe-retry-sec SEC\n";
}

std::string valueAfter(int& i, int argc, char** argv) {
  if (i + 1 >= argc) {
    throw std::runtime_error(std::string("missing value after ") + argv[i]);
  }
  return argv[++i];
}

Options parseOptions(int argc, char** argv) {
  Options o;
  for (int i = 1; i < argc; ++i) {
    const std::string arg(argv[i]);
    if (arg == "--help" || arg == "-h") {
      printUsage(argv[0]);
      std::exit(0);
    } else if (arg == "--server") {
      o.server = valueAfter(i, argc, argv);
    } else if (arg == "--control-port") {
      o.control_port = std::stoi(valueAfter(i, argc, argv));
    } else if (arg == "--camera") {
      o.camera = valueAfter(i, argc, argv);
    } else if (arg == "--image-type") {
      o.image_type = valueAfter(i, argc, argv);
    } else if (arg == "--group") {
      o.group = valueAfter(i, argc, argv);
    } else if (arg == "--port") {
      o.port = std::stoi(valueAfter(i, argc, argv));
    } else if (arg == "--local-ip") {
      o.local_ip = valueAfter(i, argc, argv);
    } else if (arg == "--no-display") {
      o.display = false;
    } else if (arg == "--scale") {
      o.scale = std::stod(valueAfter(i, argc, argv));
    } else if (arg == "--decoder") {
      o.decoder = valueAfter(i, argc, argv);
    } else if (arg == "--dump") {
      o.dump_path = valueAfter(i, argc, argv);
    } else if (arg == "--ros-image-topic") {
      o.image_topic = valueAfter(i, argc, argv);
    } else if (arg == "--ros-camera-info-topic") {
      o.camera_info_topic = valueAfter(i, argc, argv);
    } else if (arg == "--ros-camera-info-output-topic") {
      o.camera_info_output_topic = valueAfter(i, argc, argv);
    } else if (arg == "--ros-node-name") {
      o.node_name = valueAfter(i, argc, argv);
    } else if (arg == "--timeout") {
      o.timeout = std::stod(valueAfter(i, argc, argv));
    } else if (arg == "--subscribe-retry-sec") {
      o.subscribe_retry_sec = std::stod(valueAfter(i, argc, argv));
    } else {
      throw std::runtime_error("unknown option: " + arg);
    }
  }
  if (( !o.camera_info_topic.empty() || !o.camera_info_output_topic.empty()) &&
      o.image_topic.empty()) {
    throw std::runtime_error(
        "CameraInfo ROS options require --ros-image-topic");
  }
  if (o.subscribe_retry_sec <= 0.0 || o.timeout < 0.0 || o.scale <= 0.0) {
    throw std::runtime_error(
        "--subscribe-retry-sec and --scale must be positive; --timeout cannot be negative");
  }
  if (o.control_port < 1 || o.control_port > 65535 ||
      o.port < 0 || o.port > 65535) {
    throw std::runtime_error("ports must be in the range 1..65535");
  }
  if (o.decoder != "hardware" && o.decoder != "software") {
    throw std::runtime_error("--decoder must be hardware or software");
  }
  return o;
}

std::string cameraInfoTopicForCamera(const std::string& camera) {
  std::string value = camera;
  while (!value.empty() && value.front() == '/') value.erase(value.begin());
  while (!value.empty() && value.back() == '/') value.pop_back();
  const auto slash = value.find('/');
  if (slash == std::string::npos || slash == 0 || slash + 1 >= value.size()) {
    return "";
  }
  return "/Sim/SceneBasicDrone/robots/" + value.substr(0, slash) +
         "/sensors/" + value.substr(slash + 1) +
         "/scene_camera/camera_info";
}

std::string cameraInfoOutputTopicForImage(const std::string& image_topic) {
  std::string topic = image_topic;
  while (!topic.empty() && topic.back() == '/') topic.pop_back();
  const std::string suffix = "/image_raw";
  if (topic.size() >= suffix.size() &&
      topic.compare(topic.size() - suffix.size(), suffix.size(), suffix) == 0) {
    topic.replace(topic.size() - std::string("image_raw").size(),
                  std::string("image_raw").size(), "camera_info");
    return topic;
  }
  const auto slash = topic.find_last_of('/');
  if (slash != std::string::npos) return topic.substr(0, slash) + "/camera_info";
  return topic + "/camera_info";
}

class ControlClient {
 public:
  ControlClient(const std::string& server, int port,
                const std::string& camera, const std::string& image_type)
      : camera_(camera), image_type_(image_type) {
    addrinfo hints{};
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_DGRAM;
    addrinfo* result = nullptr;
    const std::string service = std::to_string(port);
    const int lookup = getaddrinfo(server.c_str(), service.c_str(), &hints, &result);
    if (lookup != 0) {
      throw std::runtime_error("cannot resolve video server " + server + ": " +
                               gai_strerror(lookup));
    }
    socket_ = socket(result->ai_family, result->ai_socktype, result->ai_protocol);
    if (socket_ < 0 || connect(socket_, result->ai_addr, result->ai_addrlen) != 0) {
      const std::string error = std::strerror(errno);
      freeaddrinfo(result);
      closeSocket();
      throw std::runtime_error("cannot open video control socket: " + error);
    }
    freeaddrinfo(result);

    timeval timeout{};
    timeout.tv_sec = 1;
    setsockopt(socket_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    sockaddr_in local{};
    socklen_t local_size = sizeof(local);
    if (getsockname(socket_, reinterpret_cast<sockaddr*>(&local), &local_size) != 0) {
      throw std::runtime_error("cannot inspect control socket address");
    }
    char ip[INET_ADDRSTRLEN]{};
    inet_ntop(AF_INET, &local.sin_addr, ip, sizeof(ip));
    client_id_ = std::string(ip) + ":" + std::to_string(ntohs(local.sin_port));
  }

  ~ControlClient() { closeSocket(); }
  ControlClient(const ControlClient&) = delete;
  ControlClient& operator=(const ControlClient&) = delete;

  const std::string& clientId() const { return client_id_; }

  std::string command(const std::string& verb, bool with_client = true) {
    std::ostringstream line;
    line << verb << " " << camera_ << " " << image_type_;
    if (with_client) line << " " << client_id_;
    line << "\n";
    const std::string payload = line.str();
    if (send(socket_, payload.data(), payload.size(), 0) < 0) {
      return "";
    }
    char buffer[2048];
    const ssize_t count = recv(socket_, buffer, sizeof(buffer) - 1, 0);
    if (count <= 0) return "";
    buffer[count] = '\0';
    std::string response(buffer);
    while (!response.empty() &&
           (response.back() == '\n' || response.back() == '\r')) {
      response.pop_back();
    }
    return response;
  }

  bool subscribe(std::string* group, int* port) {
    const std::string response = command("SUBSCRIBE");
    if (response.rfind("OK", 0) != 0) {
      std::cerr << "[ctrl] SUBSCRIBE failed: "
                << (response.empty() ? "(no reply)" : response) << std::endl;
      return false;
    }
    std::istringstream parts(response);
    std::string ok;
    parts >> ok >> *group >> *port;
    return true;
  }
  std::string heartbeat() { return command("HEARTBEAT"); }
  void requestKeyframe() { command("KEYFRAME", false); }
  void unsubscribe() { command("UNSUBSCRIBE"); }

 private:
  void closeSocket() {
    if (socket_ >= 0) {
      close(socket_);
      socket_ = -1;
    }
  }
  int socket_{-1};
  std::string camera_;
  std::string image_type_;
  std::string client_id_;
};

class RosImageOutput {
 public:
  RosImageOutput(ros::NodeHandle& nh, const Options& options) {
    if (options.image_topic.empty()) return;
    publisher_ = nh.advertise<sensor_msgs::Image>(
        options.image_topic, 1, false);
    const std::string input_info = !options.camera_info_topic.empty()
        ? options.camera_info_topic : cameraInfoTopicForCamera(options.camera);
    const std::string output_info = !options.camera_info_output_topic.empty()
        ? options.camera_info_output_topic
        : cameraInfoOutputTopicForImage(options.image_topic);
    if (!input_info.empty()) {
      if (!output_info.empty()) {
        camera_info_publisher_ =
            nh.advertise<sensor_msgs::CameraInfo>(output_info, 1, true);
      }
      camera_info_subscriber_ = nh.subscribe<sensor_msgs::CameraInfo>(
          input_info, 1, &RosImageOutput::cameraInfoCallback, this);
      std::cout << "[ros] relaying CameraInfo from " << input_info;
      if (camera_info_publisher_) std::cout << " to " << output_info;
      std::cout << std::endl;
    }
    enabled_ = true;
    std::cout << "[ros] publishing decoded images on "
              << options.image_topic << std::endl;
  }

  bool enabled() const { return enabled_; }

  void publish(GstVideoInfo& info, GstBuffer* buffer) {
    GstVideoFrame frame;
    if (!gst_video_frame_map(&frame, &info, buffer, GST_MAP_READ)) {
      ROS_WARN_THROTTLE(5.0, "cannot map GStreamer frame for ROS publishing");
      return;
    }
    const uint32_t width = GST_VIDEO_INFO_WIDTH(&info);
    const uint32_t height = GST_VIDEO_INFO_HEIGHT(&info);
    const auto* source = static_cast<const uint8_t*>(
        GST_VIDEO_FRAME_PLANE_DATA(&frame, 0));
    const int source_stride = GST_VIDEO_FRAME_PLANE_STRIDE(&frame, 0);
    sensor_msgs::Image message;
    message.header.stamp = ros::Time::now();
    message.header.frame_id = frame_id_;
    message.height = height;
    message.width = width;
    message.encoding = "bgr8";
    message.is_bigendian = 0;
    message.step = width * 3;
    message.data.resize(static_cast<size_t>(message.step) * height);
    // Convert BGRx to ROS bgr8 through OpenCV's vectorized implementation.
    // The previous scalar per-pixel loop copied 3 channels separately for
    // every pixel on every camera frame.
    cv::Mat bgrx(static_cast<int>(height), static_cast<int>(width), CV_8UC4,
                 const_cast<uint8_t*>(source), source_stride);
    cv::Mat bgr(static_cast<int>(height), static_cast<int>(width), CV_8UC3,
                message.data.data(), message.step);
    cv::cvtColor(bgrx, bgr, cv::COLOR_BGRA2BGR);
    gst_video_frame_unmap(&frame);
    publisher_.publish(message);
  }

 private:
  void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& message) {
    if (frame_id_.empty()) frame_id_ = message->header.frame_id;
    if (camera_info_publisher_) camera_info_publisher_.publish(message);
  }
  bool enabled_{false};
  std::string frame_id_;
  ros::Publisher publisher_;
  ros::Publisher camera_info_publisher_;
  ros::Subscriber camera_info_subscriber_;
};

class FrameDisplay {
 public:
  FrameDisplay(bool enabled, double scale, std::string title)
      : enabled_(enabled), scale_(scale), title_(std::move(title)) {
    if (enabled_) cv::namedWindow(title_, cv::WINDOW_NORMAL);
  }
  void show(GstVideoInfo& info, GstBuffer* buffer) {
    if (!enabled_) return;
    GstVideoFrame frame;
    if (!gst_video_frame_map(&frame, &info, buffer, GST_MAP_READ)) return;
    const int width = static_cast<int>(GST_VIDEO_INFO_WIDTH(&info));
    const int height = static_cast<int>(GST_VIDEO_INFO_HEIGHT(&info));
    const auto* data = static_cast<const uint8_t*>(
        GST_VIDEO_FRAME_PLANE_DATA(&frame, 0));
    const int stride = GST_VIDEO_FRAME_PLANE_STRIDE(&frame, 0);
    cv::Mat bgrx(height, width, CV_8UC4, const_cast<uint8_t*>(data), stride);
    cv::Mat bgr;
    cv::cvtColor(bgrx, bgr, cv::COLOR_BGRA2BGR);
    gst_video_frame_unmap(&frame);
    if (scale_ != 1.0) {
      cv::resize(bgr, bgr, cv::Size(), scale_, scale_, cv::INTER_AREA);
    }
    cv::imshow(title_, bgr);
    cv::waitKey(1);
  }
  ~FrameDisplay() {
    if (enabled_) cv::destroyWindow(title_);
  }
 private:
  bool enabled_;
  double scale_;
  std::string title_;
};

struct Pipeline {
  Pipeline() = default;
  Pipeline(const Pipeline&) = delete;
  Pipeline& operator=(const Pipeline&) = delete;
  Pipeline(Pipeline&& other) noexcept
      : pipeline(other.pipeline),
        jitter(other.jitter),
        decoded_sink(other.decoded_sink),
        dump_sink(other.dump_sink),
        bus(other.bus),
        dumping(other.dumping) {
    other.pipeline = nullptr;
    other.jitter = nullptr;
    other.decoded_sink = nullptr;
    other.dump_sink = nullptr;
    other.bus = nullptr;
    other.dumping = false;
  }
  GstElement* pipeline{nullptr};
  GstElement* jitter{nullptr};
  GstElement* decoded_sink{nullptr};
  GstElement* dump_sink{nullptr};
  GstBus* bus{nullptr};
  bool dumping{false};

  ~Pipeline() { close(); }
  void close() {
    if (pipeline) {
      gst_element_set_state(pipeline, GST_STATE_NULL);
      if (bus) gst_object_unref(bus);
      gst_object_unref(pipeline);
    }
    pipeline = nullptr;
    bus = nullptr;
    jitter = nullptr;
    decoded_sink = nullptr;
    dump_sink = nullptr;
  }
};

GstElement* makeElement(const char* factory, const char* name) {
  GstElement* element = gst_element_factory_make(factory, name);
  if (!element) {
    throw std::runtime_error(std::string("missing GStreamer element: ") + factory);
  }
  return element;
}

void linkElements(const std::vector<GstElement*>& elements) {
  for (size_t i = 1; i < elements.size(); ++i) {
    if (!gst_element_link(elements[i - 1], elements[i])) {
      throw std::runtime_error("could not link GStreamer pipeline at " +
                               std::string(GST_ELEMENT_NAME(elements[i])));
    }
  }
}

void addAll(GstElement* pipeline, const std::vector<GstElement*>& elements) {
  for (auto* element : elements) gst_bin_add(GST_BIN(pipeline), element);
}

Pipeline makePipeline(const Options& options, const std::string& group,
                     int port) {
  Pipeline p;
  p.pipeline = gst_pipeline_new("xd-video-stream-receiver");
  if (!p.pipeline) throw std::runtime_error("cannot create GStreamer pipeline");
  try {
    GstElement* source = makeElement("udpsrc", "rtp-source");
    p.jitter = makeElement("rtpjitterbuffer", "rtp-jitter");
    GstElement* depay = makeElement("rtph264depay", "h264-depay");
    GstElement* parser = makeElement("h264parse", "h264-parser");
    GstElement* tee = makeElement("tee", "parsed-stream");
    GstElement* decode_queue = makeElement("queue", "decode-queue");
    GstElement* decoder = makeElement(
        options.decoder == "hardware" ? "nvv4l2decoder" : "avdec_h264",
        "h264-decoder");
    GstElement* converter = makeElement(
        options.decoder == "hardware" ? "nvvidconv" : "videoconvert",
        "video-converter");
    GstElement* caps_filter = makeElement("capsfilter", "bgrx-caps");
    p.decoded_sink = makeElement("appsink", "decoded-frame-output");
    std::vector<GstElement*> main{
        source, p.jitter, depay, parser, tee, decode_queue,
        decoder, converter, caps_filter, p.decoded_sink};
    addAll(p.pipeline, main);

    g_object_set(source, "port", port, "buffer-size", kReceiveBufferBytes, nullptr);
    const bool multicast = !group.empty() && group != "0.0.0.0" &&
                           group != "unicast";
    if (multicast) {
      g_object_set(source, "address", group.c_str(), "auto-multicast", TRUE, nullptr);
    } else {
      g_object_set(source, "address",
                   options.local_ip.empty() ? "0.0.0.0" : options.local_ip.c_str(),
                   nullptr);
    }
    GstCaps* rtp_caps = gst_caps_from_string(
        "application/x-rtp,media=video,clock-rate=90000,"
        "encoding-name=H264,payload=96");
    g_object_set(source, "caps", rtp_caps, nullptr);
    gst_caps_unref(rtp_caps);
    g_object_set(p.jitter, "latency", 0, "drop-on-latency", TRUE,
                 "do-lost", TRUE, nullptr);
    g_object_set(parser, "config-interval", -1, nullptr);
    if (g_object_class_find_property(G_OBJECT_GET_CLASS(decoder),
                                     "low-latency-mode")) {
      g_object_set(decoder, "low-latency-mode", TRUE, nullptr);
    }
    if (g_object_class_find_property(G_OBJECT_GET_CLASS(decoder),
                                     "enable-max-performance")) {
      g_object_set(decoder, "enable-max-performance", TRUE, nullptr);
    }
    g_object_set(decode_queue, "max-size-buffers", 2, "max-size-bytes", 0,
                 "max-size-time", static_cast<guint64>(0), "leaky", 2, nullptr);
    GstCaps* bgrx_caps = gst_caps_from_string("video/x-raw,format=BGRx");
    g_object_set(caps_filter, "caps", bgrx_caps, nullptr);
    gst_caps_unref(bgrx_caps);
    g_object_set(p.decoded_sink, "emit-signals", FALSE, "sync", FALSE,
                 "max-buffers", 1, "drop", TRUE, nullptr);
    linkElements({source, p.jitter, depay, parser, tee});
    if (!gst_element_link_many(tee, decode_queue, decoder, converter,
                               caps_filter, p.decoded_sink, nullptr)) {
      throw std::runtime_error("could not link GStreamer decode branch");
    }

    if (!options.dump_path.empty()) {
      GstElement* dump_queue = makeElement("queue", "dump-queue");
      GstElement* dump_caps_filter = makeElement("capsfilter", "annexb-caps");
      p.dump_sink = makeElement("appsink", "h264-dump-output");
      addAll(p.pipeline, {dump_queue, dump_caps_filter, p.dump_sink});
      g_object_set(dump_queue, "max-size-buffers", 0, "max-size-bytes",
                   8 * 1024 * 1024, "max-size-time",
                   static_cast<guint64>(0), nullptr);
      g_object_set(p.dump_sink, "emit-signals", FALSE, "sync", FALSE,
                   "max-buffers", 0, "drop", FALSE, nullptr);
      GstCaps* dump_caps = gst_caps_from_string(
          "video/x-h264,stream-format=byte-stream,alignment=au");
      g_object_set(dump_caps_filter, "caps", dump_caps, nullptr);
      gst_caps_unref(dump_caps);
      if (!gst_element_link_many(tee, dump_queue, dump_caps_filter,
                                 p.dump_sink, nullptr)) {
        throw std::runtime_error("could not link H.264 dump branch");
      }
      p.dumping = true;
    }
    p.bus = gst_element_get_bus(p.pipeline);
    if (gst_element_set_state(p.pipeline, GST_STATE_PLAYING) ==
        GST_STATE_CHANGE_FAILURE) {
      throw std::runtime_error("could not start GStreamer decode pipeline");
    }
    return p;
  } catch (...) {
    p.close();
    throw;
  }
}

bool getUInt64(GstStructure* structure, const char* key, uint64_t* value) {
  if (!structure) return false;
  guint64 found = 0;
  if (!gst_structure_get_uint64(structure, key, &found)) return false;
  *value = found;
  return true;
}

void readPacketStats(GstElement* jitter, uint64_t* pushed, uint64_t* lost) {
  *pushed = *lost = 0;
  GstStructure* stats = nullptr;
  g_object_get(jitter, "stats", &stats, nullptr);
  getUInt64(stats, "num-pushed", pushed);
  getUInt64(stats, "num-lost", lost);
  if (stats) gst_structure_free(stats);
}

bool checkBus(Pipeline& p) {
  GstMessage* message = gst_bus_pop_filtered(
      p.bus, static_cast<GstMessageType>(GST_MESSAGE_ERROR | GST_MESSAGE_EOS));
  if (!message) return false;
  if (GST_MESSAGE_TYPE(message) == GST_MESSAGE_ERROR) {
    GError* error = nullptr;
    gchar* debug = nullptr;
    gst_message_parse_error(message, &error, &debug);
    std::string reason = error ? error->message : "unknown GStreamer error";
    if (debug && *debug) reason += " (" + std::string(debug) + ")";
    if (error) g_error_free(error);
    g_free(debug);
    gst_message_unref(message);
    throw std::runtime_error(reason);
  }
  gst_message_unref(message);
  throw std::runtime_error("GStreamer stream ended");
}

bool processSample(GstSample* sample, RosImageOutput& ros_output,
                   FrameDisplay& display, std::ofstream* dump_file,
                   uint64_t* decoded_frames) {
  if (!sample) return false;
  GstBuffer* buffer = gst_sample_get_buffer(sample);
  if (!buffer) {
    gst_sample_unref(sample);
    return false;
  }
  if (dump_file) {
    GstMapInfo map;
    if (gst_buffer_map(buffer, &map, GST_MAP_READ)) {
      dump_file->write(reinterpret_cast<const char*>(map.data),
                       static_cast<std::streamsize>(map.size));
      gst_buffer_unmap(buffer, &map);
    }
  } else {
    GstCaps* caps = gst_sample_get_caps(sample);
    GstVideoInfo info;
    gst_video_info_init(&info);
    if (caps && gst_video_info_from_caps(&info, caps)) {
      if (ros_output.enabled()) ros_output.publish(info, buffer);
      display.show(info, buffer);
      ++*decoded_frames;
    }
  }
  gst_sample_unref(sample);
  return true;
}

void runSession(const Options& options, ControlClient& control,
                RosImageOutput& ros_output, FrameDisplay& display,
                bool* subscribed) {
  std::string reply_group;
  int reply_port = 0;
  while (!g_stop.load() && (!ros_output.enabled() || ros::ok())) {
    if (control.subscribe(&reply_group, &reply_port)) {
      *subscribed = true;
      break;
    }
    std::this_thread::sleep_for(
        std::chrono::duration<double>(options.subscribe_retry_sec));
  }
  if (g_stop.load() || (ros_output.enabled() && !ros::ok())) return;
  const std::string group = options.group.empty() ? reply_group : options.group;
  const int port = options.port == 0 ? reply_port : options.port;
  if (port == 0) throw std::runtime_error(
      "server did not report a video endpoint; pass --group and --port");
  std::cout << "[ctrl] client id " << control.clientId() << std::endl;
  std::cout << "[decode] GStreamer " << options.decoder
            << " RTP/H.264 pipeline" << std::endl;

  Pipeline pipeline = makePipeline(options, group, port);
  std::ofstream dump_file;
  if (!options.dump_path.empty()) {
    dump_file.open(options.dump_path, std::ios::binary | std::ios::app);
    if (!dump_file) throw std::runtime_error("cannot open dump file: " +
                                             options.dump_path);
    std::cout << "[dump] appending H.264 stream to " << options.dump_path
              << std::endl;
  }
  std::cout << "[video] receiving " << options.camera << " ("
            << options.image_type << ") from "
            << (group.empty() ? "unicast" : group) << ":" << port << std::endl;

  auto last_heartbeat = Clock::now() - std::chrono::seconds(2);
  auto last_packet_time = Clock::now();
  auto last_stats_time = last_packet_time;
  auto last_keyframe_request = Clock::now() - std::chrono::seconds(2);
  uint64_t pushed = 0, lost = 0, previous_pushed = 0, previous_lost = 0;
  uint64_t decoded_frames = 0, previous_decoded = 0;
  int missed_heartbeats = 0;
  while (!g_stop.load() && (!ros_output.enabled() || ros::ok())) {
    ros::spinOnce();
    const auto now = Clock::now();
    if (now - last_heartbeat >= std::chrono::seconds(1)) {
      last_heartbeat = now;
      const std::string reply = control.heartbeat();
      if (reply.rfind("OK", 0) != 0) {
        ++missed_heartbeats;
        std::cerr << "[ctrl] HEARTBEAT failed (" << missed_heartbeats
                  << "/3): " << (reply.empty() ? "(no reply)" : reply)
                  << std::endl;
        if (missed_heartbeats >= 3) {
          throw std::runtime_error("three consecutive HEARTBEAT failures");
        }
      } else {
        missed_heartbeats = 0;
      }
    }

    GstSample* sample = gst_app_sink_try_pull_sample(
        GST_APP_SINK(pipeline.decoded_sink), 100 * GST_MSECOND);
    processSample(sample, ros_output, display, nullptr, &decoded_frames);
    if (pipeline.dump_sink) {
      while ((sample = gst_app_sink_try_pull_sample(
                  GST_APP_SINK(pipeline.dump_sink), 0)) != nullptr) {
        processSample(sample, ros_output, display, &dump_file, &decoded_frames);
      }
    }
    checkBus(pipeline);
    readPacketStats(pipeline.jitter, &pushed, &lost);
    if (pushed != previous_pushed) {
      last_packet_time = Clock::now();
      previous_pushed = pushed;
    }
    if (lost > previous_lost && Clock::now() - last_keyframe_request >=
                                    std::chrono::seconds(1)) {
      last_keyframe_request = Clock::now();
      control.requestKeyframe();
      previous_lost = lost;
    }
    if (options.timeout > 0.0 &&
        std::chrono::duration<double>(Clock::now() - last_packet_time).count() >
            options.timeout) {
      throw std::runtime_error("no RTP packets for " +
                               std::to_string(options.timeout) + " seconds");
    }
    if (Clock::now() - last_stats_time >= std::chrono::seconds(1)) {
      std::cout << "[stats] " << decoded_frames - previous_decoded
                << " fps | packets " << pushed << " | lost " << lost
                << " | frames " << decoded_frames << std::endl;
      previous_decoded = decoded_frames;
      last_stats_time = Clock::now();
    }
  }
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const Options options = parseOptions(argc, argv);
    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);
    gst_init(&argc, &argv);
    cv::setNumThreads(1);

    const bool ros_enabled = !options.image_topic.empty();
    // The manager may start front and down cameras (and multiple UAVs) at the
    // same time. Match rospy.init_node(..., anonymous=True) so each receiver
    // keeps its own ROS publishers instead of evicting the previous stream.
    ros::init(argc, argv, options.node_name,
              ros::init_options::AnonymousName |
                  ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;
    RosImageOutput ros_output(nh, options);
    FrameDisplay display(options.display, options.scale,
                         "ProjectAirSim " + options.camera);
    std::cout << "[video] persistent reconnect enabled (retry delay "
              << options.subscribe_retry_sec << "s)" << std::endl;

    while (!g_stop.load() && (!ros_enabled || ros::ok())) {
      bool subscribed = false;
      std::unique_ptr<ControlClient> control;
      try {
        control.reset(new ControlClient(options.server, options.control_port,
                                        options.camera, options.image_type));
        std::cout << "[ctrl] client id " << control->clientId() << std::endl;
        runSession(options, *control, ros_output, display, &subscribed);
      } catch (const std::exception& error) {
        if (!g_stop.load()) {
          std::cerr << "[video] session failed: " << error.what()
                    << "; retrying in " << options.subscribe_retry_sec
                    << "s" << std::endl;
        }
      }
      if (control && subscribed) control->unsubscribe();
      control.reset();
      if (!g_stop.load() && (!ros_enabled || ros::ok())) {
        std::this_thread::sleep_for(
            std::chrono::duration<double>(options.subscribe_retry_sec));
      }
    }
  } catch (const std::exception& error) {
    std::cerr << "[video] " << error.what() << std::endl;
    return 2;
  }
  return 0;
}
