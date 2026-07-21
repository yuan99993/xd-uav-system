#include <ros/ros.h>
#include <std_msgs/String.h>

#include <cstdint>
#include <sstream>
#include <string>

class HeartbeatNode {
public:
  HeartbeatNode()
      : private_nh_("~"), sequence_(0) {
    private_nh_.param("rate_hz", rate_hz_, 1.0);
    private_nh_.param("topic", topic_, std::string("/xd_demo_cpp/heartbeat"));

    if (rate_hz_ <= 0.0) {
      ROS_FATAL_STREAM("Parameter '~rate_hz' must be greater than zero, got "
                       << rate_hz_);
      ros::shutdown();
      return;
    }

    publisher_ = node_handle_.advertise<std_msgs::String>(topic_, 10);
    timer_ = node_handle_.createTimer(
        ros::Duration(1.0 / rate_hz_), &HeartbeatNode::timerCallback, this);

    ROS_INFO_STREAM("Publishing heartbeat messages on '" << topic_
                    << "' at " << rate_hz_ << " Hz");
  }

private:
  void timerCallback(const ros::TimerEvent&) {
    std_msgs::String message;
    std::ostringstream text;
    text << "xd_demo_cpp heartbeat #" << sequence_++;
    message.data = text.str();
    publisher_.publish(message);
  }

  ros::NodeHandle node_handle_;
  ros::NodeHandle private_nh_;
  ros::Publisher publisher_;
  ros::Timer timer_;
  double rate_hz_;
  std::string topic_;
  std::uint64_t sequence_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "heartbeat_node");
  HeartbeatNode node;
  ros::spin();
  return 0;
}
