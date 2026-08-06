#include <algorithm>
#include <cstdint>
#include <string>

#include <diagnostic_updater/diagnostic_updater.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>

class PodCameraDriver {
 public:
  PodCameraDriver() : private_nh_("~"), image_transport_(nh_) {
    private_nh_.param("eo_vendor_image_topic", eo_vendor_image_topic_,
                      std::string("/pod/vendor/eo/image_raw"));
    private_nh_.param("eo_vendor_camera_info_topic", eo_vendor_info_topic_,
                      std::string("/pod/vendor/eo/camera_info"));
    private_nh_.param("eo_image_topic", eo_image_topic_,
                      std::string("/pod/camera/eo/image_raw"));
    private_nh_.param("eo_camera_info_topic", eo_info_topic_,
                      std::string("/pod/camera/eo/camera_info"));
    private_nh_.param("enable_ir", enable_ir_, false);
    private_nh_.param("ir_vendor_image_topic", ir_vendor_image_topic_,
                      std::string("/pod/vendor/ir/image_raw"));
    private_nh_.param("ir_vendor_camera_info_topic", ir_vendor_info_topic_,
                      std::string("/pod/vendor/ir/camera_info"));
    private_nh_.param("ir_image_topic", ir_image_topic_,
                      std::string("/pod/camera/ir/image_raw"));
    private_nh_.param("ir_camera_info_topic", ir_info_topic_,
                      std::string("/pod/camera/ir/camera_info"));

    eo_image_pub_ = image_transport_.advertise(eo_image_topic_, 2);
    eo_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>(eo_info_topic_, 2);
    eo_image_sub_ = image_transport_.subscribe(
        eo_vendor_image_topic_, 2, &PodCameraDriver::eoImageCallback, this,
        image_transport::TransportHints("raw", ros::TransportHints().tcpNoDelay()));
    eo_info_sub_ = nh_.subscribe(eo_vendor_info_topic_, 2,
                                 &PodCameraDriver::eoInfoCallback, this,
                                 ros::TransportHints().tcpNoDelay());
    if (enable_ir_) {
      ir_image_pub_ = image_transport_.advertise(ir_image_topic_, 2);
      ir_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>(ir_info_topic_, 2);
      ir_image_sub_ = image_transport_.subscribe(
          ir_vendor_image_topic_, 2, &PodCameraDriver::irImageCallback, this,
          image_transport::TransportHints("raw", ros::TransportHints().tcpNoDelay()));
      ir_info_sub_ = nh_.subscribe(ir_vendor_info_topic_, 2,
                                   &PodCameraDriver::irInfoCallback, this,
                                   ros::TransportHints().tcpNoDelay());
    }
    updater_.setHardwareID("pod_camera_driver_bridge");
    updater_.add("eo", this, &PodCameraDriver::eoDiagnostics);
    updater_.add("ir", this, &PodCameraDriver::irDiagnostics);
  }

 private:
  void eoImageCallback(const sensor_msgs::ImageConstPtr& message) {
    eo_image_time_ = ros::WallTime::now();
    ++eo_image_count_;
    // Preserve camera acquisition stamp and frame_id exactly.  Drivers that
    // cannot provide capture time should leave it zero rather than forge one.
    eo_image_pub_.publish(message);
  }
  void irImageCallback(const sensor_msgs::ImageConstPtr& message) {
    ir_image_time_ = ros::WallTime::now();
    ++ir_image_count_;
    ir_image_pub_.publish(message);
  }
  void eoInfoCallback(const sensor_msgs::CameraInfoConstPtr& message) {
    eo_info_time_ = ros::WallTime::now();
    ++eo_info_count_;
    eo_info_pub_.publish(message);
  }
  void irInfoCallback(const sensor_msgs::CameraInfoConstPtr& message) {
    ir_info_time_ = ros::WallTime::now();
    ++ir_info_count_;
    ir_info_pub_.publish(message);
  }
  static bool fresh(const ros::WallTime& timestamp) {
    return !timestamp.isZero() &&
        (ros::WallTime::now() - timestamp).toSec() <= 1.0;
  }
  void eoDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& status) {
    const bool ready = fresh(eo_image_time_) && fresh(eo_info_time_);
    status.summary(ready ? diagnostic_msgs::DiagnosticStatus::OK
                         : diagnostic_msgs::DiagnosticStatus::WARN,
                   ready ? "EO image and CameraInfo forwarding" : "waiting for EO vendor stream");
    status.add("image_topic", eo_image_topic_);
    status.add("image_count", static_cast<long long>(eo_image_count_));
    status.add("camera_info_count", static_cast<long long>(eo_info_count_));
  }
  void irDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& status) {
    if (!enable_ir_) {
      status.summary(diagnostic_msgs::DiagnosticStatus::OK, "IR stream disabled by configuration");
      return;
    }
    const bool ready = fresh(ir_image_time_) && fresh(ir_info_time_);
    status.summary(ready ? diagnostic_msgs::DiagnosticStatus::OK
                         : diagnostic_msgs::DiagnosticStatus::WARN,
                   ready ? "IR image and CameraInfo forwarding" : "waiting for IR vendor stream");
    status.add("image_topic", ir_image_topic_);
    status.add("image_count", static_cast<long long>(ir_image_count_));
    status.add("camera_info_count", static_cast<long long>(ir_info_count_));
  }

  ros::NodeHandle nh_, private_nh_;
  image_transport::ImageTransport image_transport_;
  image_transport::Publisher eo_image_pub_, ir_image_pub_;
  image_transport::Subscriber eo_image_sub_, ir_image_sub_;
  ros::Publisher eo_info_pub_, ir_info_pub_;
  ros::Subscriber eo_info_sub_, ir_info_sub_;
  diagnostic_updater::Updater updater_;
  std::string eo_vendor_image_topic_, eo_vendor_info_topic_, eo_image_topic_, eo_info_topic_;
  std::string ir_vendor_image_topic_, ir_vendor_info_topic_, ir_image_topic_, ir_info_topic_;
  bool enable_ir_{false};
  ros::WallTime eo_image_time_, eo_info_time_, ir_image_time_, ir_info_time_;
  std::uint64_t eo_image_count_{0}, eo_info_count_{0}, ir_image_count_{0}, ir_info_count_{0};
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "pod_camera_driver");
  PodCameraDriver node;
  ros::spin();
  return 0;
}
