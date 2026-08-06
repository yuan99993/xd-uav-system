#include <algorithm>
#include <cstdint>
#include <exception>
#include <string>

#include <diagnostic_updater/diagnostic_updater.h>
#include <pod_msgs/GimbalState.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <tracker/DetectionArray.h>

class PodReplay {
 public:
  PodReplay() : private_nh_("~") {
    private_nh_.param("bag_path", bag_path_, std::string(""));
    private_nh_.param("source_gimbal_topic", source_gimbal_topic_,
                      std::string("/pod/gimbal/state"));
    private_nh_.param("source_detections_topic", source_detections_topic_,
                      std::string("/pod/perception/detections"));
    private_nh_.param("gimbal_state_topic", output_gimbal_topic_,
                      std::string("/pod/vendor/gimbal/state"));
    private_nh_.param("detections_topic", output_detections_topic_,
                      std::string("/pod/vendor/detections"));
    private_nh_.param("rate", rate_, 1.0);
    private_nh_.param("loop", loop_, false);
    rate_ = std::max(0.05, rate_);
    gimbal_pub_ = nh_.advertise<pod_msgs::GimbalState>(output_gimbal_topic_, 10);
    detections_pub_ = nh_.advertise<tracker::DetectionArray>(output_detections_topic_, 10);
    start_timer_ = nh_.createTimer(ros::Duration(0.1), &PodReplay::start, this, true);
    updater_.setHardwareID("pod_replay");
    updater_.add("replay", this, &PodReplay::diagnostics);
  }
 private:
  static bool sameTopic(const std::string& left, const std::string& right) {
    return left == right || (left.size() + 1 == right.size() && right.front() == '/' && right.substr(1) == left) ||
        (right.size() + 1 == left.size() && left.front() == '/' && left.substr(1) == right);
  }
  void start(const ros::TimerEvent&) {
    if (bag_path_.empty()) {
      result_ = "bag_path is empty; replay disabled";
      updater_.force_update();
      return;
    }
    do { replayOnce(); } while (loop_ && ros::ok());
    updater_.force_update();
  }
  void replayOnce() {
    try {
      rosbag::Bag bag;
      bag.open(bag_path_, rosbag::bagmode::Read);
      rosbag::View view(bag);
      ros::Time previous;
      for (const rosbag::MessageInstance& instance : view) {
        if (!ros::ok()) break;
        const ros::Time stamp = instance.getTime();
        if (!previous.isZero() && stamp > previous) {
          ros::WallDuration((stamp - previous).toSec() / rate_).sleep();
        }
        previous = stamp;
        if (sameTopic(instance.getTopic(), source_gimbal_topic_)) {
          const pod_msgs::GimbalStateConstPtr state = instance.instantiate<pod_msgs::GimbalState>();
          if (state) { gimbal_pub_.publish(state); ++gimbal_messages_; }
        } else if (sameTopic(instance.getTopic(), source_detections_topic_)) {
          const tracker::DetectionArrayConstPtr detections = instance.instantiate<tracker::DetectionArray>();
          if (detections) { detections_pub_.publish(detections); ++detection_messages_; }
        }
      }
      bag.close();
      result_ = "replay complete";
    } catch (const std::exception& exception) {
      result_ = std::string("replay failed: ") + exception.what();
      ROS_ERROR("[PodReplay] %s", result_.c_str());
    }
  }
  void diagnostics(diagnostic_updater::DiagnosticStatusWrapper& status) {
    const bool failed = result_.find("failed") != std::string::npos ||
        result_.find("empty") != std::string::npos;
    status.summary(failed ? diagnostic_msgs::DiagnosticStatus::ERROR
                          : diagnostic_msgs::DiagnosticStatus::OK, result_);
    status.add("bag_path", bag_path_);
    status.add("gimbal_messages", static_cast<long long>(gimbal_messages_));
    status.add("detection_messages", static_cast<long long>(detection_messages_));
  }
  ros::NodeHandle nh_, private_nh_;
  ros::Publisher gimbal_pub_, detections_pub_;
  ros::Timer start_timer_;
  diagnostic_updater::Updater updater_;
  std::string bag_path_, source_gimbal_topic_, source_detections_topic_;
  std::string output_gimbal_topic_, output_detections_topic_;
  std::string result_{"waiting to start replay"};
  double rate_{1.0};
  bool loop_{false};
  std::uint64_t gimbal_messages_{0}, detection_messages_{0};
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "pod_replay");
  PodReplay node;
  ros::spin();
  return 0;
}
