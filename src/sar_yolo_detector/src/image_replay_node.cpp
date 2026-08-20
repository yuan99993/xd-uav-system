#include <algorithm>
#include <stdexcept>
#include <string>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

namespace {

class ImageReplayNode {
public:
  ImageReplayNode() : private_node_("~") {
    std::string image_path;
    std::string image_topic;
    std::string frame_id;
    std::string publish_encoding;
    double publish_rate_hz;
    private_node_.param("image_path", image_path, std::string());
    private_node_.param("image_topic", image_topic,
                        std::string("camera/image_raw"));
    private_node_.param("frame_id", frame_id, std::string("camera_optical"));
    private_node_.param("publish_encoding", publish_encoding,
                        std::string("bgr8"));
    private_node_.param("publish_rate_hz", publish_rate_hz, 5.0);
    if (image_path.empty()) {
      throw std::invalid_argument("~image_path must name a readable image");
    }
    image_ = cv::imread(image_path, cv::IMREAD_COLOR);
    if (image_.empty()) {
      throw std::runtime_error("Cannot read replay image: " + image_path);
    }
    if (publish_encoding != "bgr8" && publish_encoding != "mono8") {
      throw std::invalid_argument("~publish_encoding must be bgr8 or mono8");
    }
    frame_id_ = frame_id;
    publish_encoding_ = publish_encoding;
    publisher_ = node_.advertise<sensor_msgs::Image>(image_topic, 1);
    const double bounded_rate = std::max(0.1, publish_rate_hz);
    timer_ = node_.createTimer(ros::Duration(1.0 / bounded_rate),
                               &ImageReplayNode::timerCallback, this);
    ROS_INFO_STREAM("[SarYoloImageReplay] Publishing " << image_path << " on "
                    << image_topic << " at " << bounded_rate << " Hz");
  }

private:
  void timerCallback(const ros::TimerEvent &) {
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.frame_id = frame_id_;
    if (publish_encoding_ == "mono8") {
      cv::Mat mono_image;
      cv::cvtColor(image_, mono_image, cv::COLOR_BGR2GRAY);
      publisher_.publish(
          cv_bridge::CvImage(header, "mono8", mono_image).toImageMsg());
      return;
    }
    publisher_.publish(cv_bridge::CvImage(header, "bgr8", image_).toImageMsg());
  }

  ros::NodeHandle node_;
  ros::NodeHandle private_node_;
  ros::Publisher publisher_;
  ros::Timer timer_;
  cv::Mat image_;
  std::string frame_id_;
  std::string publish_encoding_;
};

}  // namespace

int main(int argc, char **argv) {
  ros::init(argc, argv, "sar_yolo_image_replay");
  try {
    ImageReplayNode node;
    ros::spin();
  } catch (const std::exception &exception) {
    ROS_FATAL_STREAM("[SarYoloImageReplay] Startup failed: " << exception.what());
    return 1;
  }
  return 0;
}
