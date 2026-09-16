#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <gazebo/common/Events.hh>
#include <gazebo/common/PID.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>

namespace gazebo {

class GimbalJointControllerPlugin : public ModelPlugin {
 public:
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override {
    if (!ros::isInitialized()) {
      gzerr << "[gimbal_joint_controller] gazebo_ros must initialize ROS\n";
      return;
    }
    model_ = std::move(model);
    joint_controller_ = model_->GetJointController();
    const std::string robot_namespace =
        sdf->HasElement("robotNamespace")
            ? sdf->Get<std::string>("robotNamespace") : std::string();
    const std::string topic =
        sdf->HasElement("topicName")
            ? sdf->Get<std::string>("topicName")
            : std::string("gimbal/set_joint_trajectory");
    const double p = sdf->HasElement("positionP")
        ? sdf->Get<double>("positionP") : 0.20;
    const double i = sdf->HasElement("positionI")
        ? sdf->Get<double>("positionI") : 0.0;
    const double d = sdf->HasElement("positionD")
        ? sdf->Get<double>("positionD") : 0.01;
    const double maximum_effort = sdf->HasElement("maximumEffort")
        ? std::abs(sdf->Get<double>("maximumEffort")) : 0.20;

    for (const std::string& name : {std::string("gimbal_yaw_joint"),
                                    std::string("gimbal_pitch_joint")}) {
      const physics::JointPtr joint = model_->GetJoint(name);
      if (!joint) {
        gzerr << "[gimbal_joint_controller] missing joint " << name << "\n";
        continue;
      }
      joints_[name] = joint;
      joint_controller_->SetPositionPID(
          joint->GetScopedName(),
          common::PID(p, i, d, 0.0, 0.0, maximum_effort, -maximum_effort));
    }
    if (joints_.size() != 2U) return;

    ros_node_.reset(new ros::NodeHandle(robot_namespace));
    command_subscriber_ = ros_node_->subscribe(
        topic, 10, &GimbalJointControllerPlugin::commandCallback, this);
    update_connection_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&GimbalJointControllerPlugin::update, this));
    gzmsg << "[gimbal_joint_controller] listening on "
          << ros_node_->resolveName(topic) << " without pausing physics\n";
  }

 private:
  void commandCallback(
      const trajectory_msgs::JointTrajectory::ConstPtr& message) {
    if (message->points.empty()) return;
    const auto& positions = message->points.front().positions;
    if (positions.size() < message->joint_names.size()) return;

    std::unordered_map<std::string, double> next;
    for (std::size_t index = 0; index < message->joint_names.size(); ++index) {
      if (joints_.count(message->joint_names[index]) == 0U ||
          !std::isfinite(positions[index])) {
        continue;
      }
      next[message->joint_names[index]] = positions[index];
    }
    if (next.empty()) return;
    std::lock_guard<std::mutex> lock(command_mutex_);
    pending_targets_ = std::move(next);
    have_pending_targets_ = true;
  }

  void update() {
    std::unordered_map<std::string, double> targets;
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      if (!have_pending_targets_) return;
      targets.swap(pending_targets_);
      have_pending_targets_ = false;
    }
    for (const auto& item : targets) {
      joint_controller_->SetPositionTarget(
          joints_.at(item.first)->GetScopedName(), item.second);
    }
  }

  physics::ModelPtr model_;
  physics::JointControllerPtr joint_controller_;
  std::unordered_map<std::string, physics::JointPtr> joints_;
  std::unique_ptr<ros::NodeHandle> ros_node_;
  ros::Subscriber command_subscriber_;
  event::ConnectionPtr update_connection_;
  std::mutex command_mutex_;
  std::unordered_map<std::string, double> pending_targets_;
  bool have_pending_targets_{false};
};

GZ_REGISTER_MODEL_PLUGIN(GimbalJointControllerPlugin)

}  // namespace gazebo
