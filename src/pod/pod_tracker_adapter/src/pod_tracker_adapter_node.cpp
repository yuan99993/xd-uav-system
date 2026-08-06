#include <algorithm>
#include <cmath>
#include <cstdint>
#include <exception>
#include <limits>
#include <map>
#include <string>

#include <XmlRpcValue.h>
#include <ros/ros.h>
#include <tracker/DetectionArray.h>
#include <tracker/NormalizedError.h>
#include <tracker/SelectTrack.h>
#include <tracker/TrackStateArray.h>
#include <tracker/TrackingOutput.h>

#include <pod_msgs/SelectTarget.h>
#include <pod_msgs/SelectedTarget.h>
#include <pod_msgs/GimbalState.h>
#include <pod_msgs/TargetTrack.h>
#include <pod_msgs/TargetTrackArray.h>

namespace {

float clampUnit(const float value) {
  return std::max(0.0F, std::min(1.0F, value));
}

double clamp(const double value, const double minimum, const double maximum) {
  return std::max(minimum, std::min(maximum, value));
}

constexpr double kPi = 3.14159265358979323846;

class PodTrackerAdapter {
 public:
  PodTrackerAdapter() : nh_(), private_nh_("~") {
    private_nh_.param("detection_topic", detection_topic_,
                      std::string("/pod/perception/detections"));
    private_nh_.param("tracker_candidates_topic", tracker_candidates_topic_,
                      std::string("/tracker_node/detection_candidates"));
    private_nh_.param("tracking_output_topic", tracking_output_topic_,
                      std::string("/tracker_node/tracking_output"));
    private_nh_.param("tracker_track_states_topic", tracker_track_states_topic_,
                      std::string("/tracker_node/track_states"));
    private_nh_.param("tracker_select_service", tracker_select_service_,
                      std::string("/tracker_node/select_track"));
    private_nh_.param("tracks_topic", tracks_topic_,
                      std::string("/pod/perception/tracks"));
    private_nh_.param("selected_target_topic", selected_target_topic_,
                      std::string("/pod/target/selected"));
    private_nh_.param("select_target_service", select_target_service_,
                      std::string("/pod/mission/select_target"));
    private_nh_.param("image_source", image_source_, std::string("eo"));
    private_nh_.param("enable_body_los_compensation",
                      enable_body_los_compensation_, false);
    private_nh_.param("tracker_normalized_error_topic", normalized_error_topic_,
                      std::string("/tracker_node/normalized_error"));
    private_nh_.param("body_normalized_error_topic", body_error_topic_,
                      std::string("/pod/perception/body_normalized_error"));
    private_nh_.param("gimbal_state_topic", gimbal_state_topic_,
                      std::string("/pod/gimbal/state"));
    private_nh_.param("gimbal_state_timeout_sec", gimbal_state_timeout_sec_, 0.15);
    private_nh_.param("require_gimbal_state_for_body_los",
                      require_gimbal_state_for_body_los_, true);
    private_nh_.param("camera_mount_roll_deg", camera_mount_roll_deg_, 0.0);
    private_nh_.param("camera_mount_pitch_deg", camera_mount_pitch_deg_, 0.0);
    private_nh_.param("camera_mount_yaw_deg", camera_mount_yaw_deg_, 0.0);
    private_nh_.param("camera_fov_horizontal_deg", camera_fov_horizontal_deg_, 60.0);
    private_nh_.param("camera_fov_vertical_deg", camera_fov_vertical_deg_, 45.0);
    private_nh_.param("passthrough_to_tracker", passthrough_to_tracker_, true);
    private_nh_.param("use_persistent_track_states",
                      use_persistent_track_states_, true);
    private_nh_.param("enable_select_service", enable_select_service_, true);
    private_nh_.param("tracker_select_wait_timeout_sec",
                      tracker_select_wait_timeout_sec_, 3.0);
    private_nh_.param("queue_size", queue_size_, 10);
    queue_size_ = std::max(1, queue_size_);

    loadClassLabels();

    tracks_pub_ = nh_.advertise<pod_msgs::TargetTrackArray>(tracks_topic_, queue_size_);
    selected_target_pub_ = nh_.advertise<pod_msgs::SelectedTarget>(
        selected_target_topic_, 1, true);
    if (enable_body_los_compensation_) {
      body_error_pub_ = nh_.advertise<tracker::NormalizedError>(body_error_topic_,
                                                                  queue_size_);
    }
    if (passthrough_to_tracker_) {
      tracker_candidates_pub_ = nh_.advertise<tracker::DetectionArray>(
          tracker_candidates_topic_, queue_size_);
    }

    detections_sub_ = nh_.subscribe(
        detection_topic_, queue_size_, &PodTrackerAdapter::detectionsCallback, this,
        ros::TransportHints().tcpNoDelay());
    tracking_output_sub_ = nh_.subscribe(
        tracking_output_topic_, queue_size_, &PodTrackerAdapter::trackingOutputCallback,
        this, ros::TransportHints().tcpNoDelay());
    track_states_sub_ = nh_.subscribe(
        tracker_track_states_topic_, queue_size_,
        &PodTrackerAdapter::trackStatesCallback, this,
        ros::TransportHints().tcpNoDelay());
    if (enable_body_los_compensation_) {
      normalized_error_sub_ = nh_.subscribe(
          normalized_error_topic_, queue_size_,
          &PodTrackerAdapter::normalizedErrorCallback, this,
          ros::TransportHints().tcpNoDelay());
      gimbal_state_sub_ = nh_.subscribe(
          gimbal_state_topic_, queue_size_, &PodTrackerAdapter::gimbalStateCallback,
          this, ros::TransportHints().tcpNoDelay());
    }
    // Selection is low-rate. A non-persistent client intentionally performs a
    // fresh lookup so tracker restarts and adapter-before-tracker startup order
    // cannot leave a stale TCPROS service connection behind.
    tracker_select_client_ = nh_.serviceClient<tracker::SelectTrack>(
        tracker_select_service_, false);
    if (enable_select_service_) {
      select_target_server_ = nh_.advertiseService(
          select_target_service_, &PodTrackerAdapter::selectTarget, this);
    }

    ROS_INFO("[PodTrackerAdapter] detector '%s' -> tracker '%s', persistent states '%s', all tracks '%s', body LOS=%s",
             detection_topic_.c_str(), tracker_candidates_topic_.c_str(),
             tracker_track_states_topic_.c_str(), tracks_topic_.c_str(),
             enable_body_los_compensation_ ? "enabled" : "disabled");
  }

 private:
  void loadClassLabels() {
    XmlRpc::XmlRpcValue raw_labels;
    if (!private_nh_.getParam("class_labels", raw_labels)) return;

    try {
      if (raw_labels.getType() == XmlRpc::XmlRpcValue::TypeStruct) {
        for (auto it = raw_labels.begin(); it != raw_labels.end(); ++it) {
          if (it->second.getType() != XmlRpc::XmlRpcValue::TypeString) continue;
          class_labels_[std::stoi(it->first)] = static_cast<std::string>(it->second);
        }
      } else if (raw_labels.getType() == XmlRpc::XmlRpcValue::TypeArray) {
        for (int index = 0; index < raw_labels.size(); ++index) {
          if (raw_labels[index].getType() != XmlRpc::XmlRpcValue::TypeString) continue;
          class_labels_[index] = static_cast<std::string>(raw_labels[index]);
        }
      } else {
        ROS_WARN("[PodTrackerAdapter] Ignoring class_labels: expected map or list");
      }
    } catch (const std::exception& error) {
      class_labels_.clear();
      ROS_WARN("[PodTrackerAdapter] Ignoring invalid class_labels: %s", error.what());
    }
  }

  std::string classLabel(const int class_id) const {
    const auto label = class_labels_.find(class_id);
    return label == class_labels_.end()
               ? "class_" + std::to_string(class_id)
               : label->second;
  }

  void detectionsCallback(const tracker::DetectionArray::ConstPtr& message) {
    latest_detections_ = *message;
    latest_image_source_ = message->image_source.empty()
        ? image_source_ : message->image_source;
    latest_sensor_id_ = message->sensor_id;
    latest_detector_name_ = message->detector_name;
    latest_model_version_ = message->model_version;
    have_detections_ = true;
    if (passthrough_to_tracker_) tracker_candidates_pub_.publish(*message);
    // Until a persistent TrackStateArray has arrived, retain the legacy
    // detector-list output so older tracker deployments remain usable.
    if (!use_persistent_track_states_ || !have_track_states_) {
      publishDetectionTracks(true);
    }
  }

  void trackingOutputCallback(const tracker::TrackingOutput::ConstPtr& message) {
    latest_tracking_output_ = *message;
    have_tracking_output_ = true;
    // Republish the cached detector frame with the current selected target.
    // This never feeds data back into tracker and cannot change its control path.
    if (have_track_states_) publishPersistentTracks();
    else if (have_detections_) publishDetectionTracks(false);
    publishSelectedTarget();
  }

  void trackStatesCallback(const tracker::TrackStateArray::ConstPtr& message) {
    latest_track_states_ = *message;
    have_track_states_ = true;
    publishPersistentTracks();
  }

  void gimbalStateCallback(const pod_msgs::GimbalState::ConstPtr& message) {
    latest_gimbal_state_ = *message;
    gimbal_state_wall_time_ = ros::WallTime::now();
    have_gimbal_state_ = true;
  }

  static void rotateBodyFrd(const double roll, const double pitch,
                            const double yaw, double& forward,
                            double& right, double& down) {
    // GimbalState uses roll/pitch/yaw in degrees in the aircraft FRD
    // convention: positive yaw turns right and positive pitch points down.
    const double cr = std::cos(roll);
    const double sr = std::sin(roll);
    const double cp = std::cos(pitch);
    const double sp = std::sin(pitch);
    const double cy = std::cos(yaw);
    const double sy = std::sin(yaw);
    const double rolled_right = cr * right - sr * down;
    const double rolled_down = sr * right + cr * down;
    const double pitched_forward = cp * forward - sp * rolled_down;
    const double pitched_down = sp * forward + cp * rolled_down;
    forward = cy * pitched_forward - sy * rolled_right;
    right = sy * pitched_forward + cy * rolled_right;
    down = pitched_down;
  }

  void normalizedErrorCallback(const tracker::NormalizedError::ConstPtr& message) {
    tracker::NormalizedError output = *message;
    const ros::WallTime now = ros::WallTime::now();
    const bool gimbal_fresh = have_gimbal_state_ &&
        (now - gimbal_state_wall_time_).toSec() <=
            std::max(0.02, gimbal_state_timeout_sec_) &&
        latest_gimbal_state_.connected && latest_gimbal_state_.attitude_valid;
    if (message->has_angular_error && gimbal_fresh) {
      const double roll = (latest_gimbal_state_.attitude_deg.x +
                           camera_mount_roll_deg_) * kPi / 180.0;
      const double pitch = (latest_gimbal_state_.attitude_deg.y +
                            camera_mount_pitch_deg_) * kPi / 180.0;
      const double yaw = (latest_gimbal_state_.attitude_deg.z +
                          camera_mount_yaw_deg_) * kPi / 180.0;
      double forward = std::cos(message->pitch_error_rad) *
                       std::cos(message->yaw_error_rad);
      double right = std::cos(message->pitch_error_rad) *
                     std::sin(message->yaw_error_rad);
      double down = std::sin(message->pitch_error_rad);
      rotateBodyFrd(roll, pitch, yaw, forward, right, down);
      const double horizontal = std::hypot(forward, right);
      output.yaw_error_rad = std::atan2(right, forward);
      output.pitch_error_rad = std::atan2(down, std::max(1e-9, horizontal));
      output.has_angular_error = true;
      output.error_x = static_cast<float>(clamp(
          std::tan(output.yaw_error_rad) /
              std::max(1e-6, std::tan(0.5 * camera_fov_horizontal_deg_ * kPi / 180.0)),
          -1.0, 1.0));
      output.error_y = static_cast<float>(clamp(
          std::tan(output.pitch_error_rad) /
              std::max(1e-6, std::tan(0.5 * camera_fov_vertical_deg_ * kPi / 180.0)),
          -1.0, 1.0));
      output.fusion_status = "BODY_LOS_GIMBAL_COMPENSATED";
    } else if (require_gimbal_state_for_body_los_) {
      // Keep the sample visible for UI diagnostics but explicitly withdraw it
      // from aircraft control until dynamic gimbal pose is trustworthy.
      output.error_valid = false;
      output.control_measurement_ready = false;
      output.control_confidence = 0.0F;
      output.fusion_status = "BODY_LOS_GIMBAL_STATE_UNAVAILABLE";
    }
    body_error_pub_.publish(output);
  }

  bool isSelected(const tracker::DetectionCandidate& candidate) const {
    return have_tracking_output_ && latest_tracking_output_.tracking_active &&
           candidate.track_id == latest_tracking_output_.target_id;
  }

  std::uint32_t nextAge(const tracker::DetectionCandidate& candidate,
                        const bool advance_age) {
    if (!candidate.track_id_is_stable) return 1;
    auto age_it = track_ages_.find(candidate.track_id);
    if (age_it == track_ages_.end()) {
      track_ages_[candidate.track_id] = 1;
      return 1;
    }
    auto& age = age_it->second;
    if (advance_age && age < std::numeric_limits<std::uint32_t>::max()) ++age;
    return age;
  }

  pod_msgs::TargetTrack convertCandidate(
      const tracker::DetectionCandidate& candidate, const std_msgs::Header& header,
      const bool advance_age) {
    pod_msgs::TargetTrack track;
    track.header = header;
    track.target_id = candidate.track_id;
    track.class_id = candidate.class_id;
    track.class_label = classLabel(candidate.class_id);
    track.image_source = latest_image_source_;
    track.sensor_id = latest_sensor_id_;
    track.detector_name = latest_detector_name_;
    track.model_version = latest_model_version_;
    track.capture_timestamp = header.stamp;
    track.confidence = clampUnit(candidate.confidence);
    track.bbox_px = candidate.bbox;
    track.normalized_bbox = candidate.normalized_bbox;
    track.geometry_type = candidate.has_bbox ? "aabb" :
                          (candidate.has_normalized_bbox ? "normalized_aabb" : "none");
    track.has_oriented_bbox = false;
    track.oriented_bbox = {0.0F, 0.0F, 0.0F, 0.0F, 0.0F};
    track.image_velocity = {0.0F, 0.0F};
    track.age_frames = nextAge(candidate, advance_age);
    track.detected = true;
    track.predicted = false;
    track.selected = isSelected(candidate);
    track.reidentification_match = track.selected &&
                                   latest_tracking_output_.reidentification_match;
    track.control_measurement_ready = track.selected &&
                                      latest_tracking_output_.control_measurement_ready;
    track.tracking_quality = track.selected
                                ? clampUnit(latest_tracking_output_.tracking_quality)
                                : track.confidence;
    track.association_method = track.selected
                                 ? latest_tracking_output_.association_method
                                 : (candidate.track_id_is_stable ? "id" : "detected");
    track.frames_since_detection = track.selected
                                     ? std::max(0, latest_tracking_output_.frames_since_detection)
                                     : 0;
    track.range_valid = candidate.range_valid;
    track.relative_position_body.x = candidate.relative_position_body[0];
    track.relative_position_body.y = candidate.relative_position_body[1];
    track.relative_position_body.z = candidate.relative_position_body[2];
    track.relative_velocity_body.x = candidate.relative_velocity_body[0];
    track.relative_velocity_body.y = candidate.relative_velocity_body[1];
    track.relative_velocity_body.z = candidate.relative_velocity_body[2];
    for (std::size_t i = 0; i < track.position_covariance.size(); ++i) {
      track.position_covariance[i] = candidate.position_covariance[i];
    }
    return track;
  }

  void publishDetectionTracks(const bool advance_age) {
    pod_msgs::TargetTrackArray output;
    output.header = latest_detections_.header;
    if (output.header.stamp.isZero()) output.header.stamp = ros::Time::now();
    output.tracks.reserve(latest_detections_.candidates.size());
    for (const auto& candidate : latest_detections_.candidates) {
      output.tracks.push_back(convertCandidate(candidate, output.header, advance_age));
    }
    tracks_pub_.publish(output);
  }

  pod_msgs::TargetTrack convertTrackState(
      const tracker::TrackState& state, const std::string& source) const {
    pod_msgs::TargetTrack track;
    track.header = state.header;
    track.target_id = state.track_id;
    track.class_id = state.class_id;
    track.class_label = classLabel(state.class_id);
    track.image_source = source.empty() ? image_source_ : source;
    track.sensor_id = latest_sensor_id_;
    track.detector_name = latest_detector_name_;
    track.model_version = latest_model_version_;
    track.capture_timestamp = state.header.stamp;
    track.confidence = clampUnit(state.confidence);
    track.bbox_px = state.bbox;
    track.normalized_bbox = state.normalized_bbox;
    track.geometry_type = "aabb";
    track.has_oriented_bbox = false;
    track.oriented_bbox = {0.0F, 0.0F, 0.0F, 0.0F, 0.0F};
    track.image_velocity = state.image_velocity;
    track.age_frames = state.age_frames;
    track.frames_since_detection = state.frames_since_detection;
    track.detected = state.detected;
    track.predicted = state.predicted;
    track.reidentification_match = state.reidentification_match;
    track.selected = state.selected || isSelectedId(state.track_id);
    track.control_measurement_ready = state.control_measurement_ready;
    if (track.selected && have_tracking_output_) {
      track.control_measurement_ready =
          latest_tracking_output_.control_measurement_ready;
      track.tracking_quality = clampUnit(
          latest_tracking_output_.tracking_quality);
      track.association_method = latest_tracking_output_.association_method;
    } else {
      track.tracking_quality = clampUnit(state.tracking_quality);
      track.association_method = state.association_method;
    }
    track.relative_position_body.x = state.relative_position_body[0];
    track.relative_position_body.y = state.relative_position_body[1];
    track.relative_position_body.z = state.relative_position_body[2];
    track.relative_velocity_body.x = state.relative_velocity_body[0];
    track.relative_velocity_body.y = state.relative_velocity_body[1];
    track.relative_velocity_body.z = state.relative_velocity_body[2];
    for (std::size_t index = 0; index < track.position_covariance.size(); ++index) {
      track.position_covariance[index] = state.position_covariance[index];
    }
    track.range_valid = state.range_valid;
    return track;
  }

  bool isSelectedId(const int target_id) const {
    return have_tracking_output_ && latest_tracking_output_.tracking_active &&
           latest_tracking_output_.target_id == target_id;
  }

  void publishPersistentTracks() {
    pod_msgs::TargetTrackArray output;
    output.header = latest_track_states_.header;
    if (output.header.stamp.isZero()) output.header.stamp = ros::Time::now();
    output.tracks.reserve(latest_track_states_.tracks.size());
    for (const auto& state : latest_track_states_.tracks) {
      output.tracks.push_back(
          convertTrackState(state, latest_track_states_.image_source));
    }
    tracks_pub_.publish(output);
  }

  pod_msgs::SelectedTarget selectedTargetMessage(const int requested_id = -1) const {
    pod_msgs::SelectedTarget selected;
    selected.header.stamp = ros::Time::now();
    const int active_id = have_tracking_output_ &&
        latest_tracking_output_.tracking_active
            ? latest_tracking_output_.target_id : requested_id;
    selected.target_id = active_id;
    selected.selected = active_id >= 0;
    selected.tracking_active = have_tracking_output_ &&
                               latest_tracking_output_.tracking_active;
    if (have_tracking_output_ && latest_tracking_output_.target_id == active_id) {
      selected.class_id = latest_tracking_output_.class_id;
      selected.class_label = classLabel(selected.class_id);
      selected.confidence = latest_tracking_output_.confidence;
      selected.control_measurement_ready =
          latest_tracking_output_.control_measurement_ready;
      selected.association_method =
          latest_tracking_output_.association_method;
    } else {
      for (const auto& state : latest_track_states_.tracks) {
        if (state.track_id != active_id) continue;
        selected.header = state.header;
        selected.class_id = state.class_id;
        selected.class_label = classLabel(state.class_id);
        selected.confidence = state.confidence;
        selected.control_measurement_ready = state.control_measurement_ready;
        selected.association_method = state.association_method;
        break;
      }
    }
    return selected;
  }

  void publishSelectedTarget() {
    selected_target_pub_.publish(selectedTargetMessage());
  }

  bool selectTarget(pod_msgs::SelectTarget::Request& request,
                    pod_msgs::SelectTarget::Response& response) {
    tracker::SelectTrack tracker_request;
    tracker_request.request.target_id = request.target_id;
    tracker_request.request.start_tracking = request.start_tracking;
    tracker_request.request.use_normalized_roi = false;
    tracker_request.request.image_source = latest_track_states_.image_source;
    tracker_request.request.capture_timestamp = latest_track_states_.header.stamp;
    if (!tracker_select_client_.waitForExistence(
            ros::Duration(std::max(0.0, tracker_select_wait_timeout_sec_))) ||
        !tracker_select_client_.call(tracker_request)) {
      response.success = false;
      response.message = "Tracker selection service is unavailable";
      response.selected_target = selectedTargetMessage();
      return true;
    }
    response.success = tracker_request.response.success;
    response.message = tracker_request.response.message;
    response.selected_target = selectedTargetMessage(
        tracker_request.response.selected_target_id);
    response.selected_target.selected = response.success && request.start_tracking;
    selected_target_pub_.publish(response.selected_target);
    return true;
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Publisher tracker_candidates_pub_;
  ros::Publisher tracks_pub_;
  ros::Publisher selected_target_pub_;
  ros::Publisher body_error_pub_;
  ros::Subscriber detections_sub_;
  ros::Subscriber tracking_output_sub_;
  ros::Subscriber track_states_sub_;
  ros::Subscriber normalized_error_sub_;
  ros::Subscriber gimbal_state_sub_;
  ros::ServiceClient tracker_select_client_;
  ros::ServiceServer select_target_server_;

  std::string detection_topic_;
  std::string tracker_candidates_topic_;
  std::string tracking_output_topic_;
  std::string tracker_track_states_topic_;
  std::string tracker_select_service_;
  std::string tracks_topic_;
  std::string selected_target_topic_;
  std::string select_target_service_;
  std::string image_source_;
  std::string normalized_error_topic_, body_error_topic_, gimbal_state_topic_;
  std::string latest_image_source_{"eo"};
  std::string latest_sensor_id_, latest_detector_name_, latest_model_version_;
  bool passthrough_to_tracker_{true};
  bool use_persistent_track_states_{true};
  bool enable_select_service_{true};
  bool enable_body_los_compensation_{false};
  bool require_gimbal_state_for_body_los_{true};
  double tracker_select_wait_timeout_sec_{3.0};
  double gimbal_state_timeout_sec_{0.15};
  double camera_mount_roll_deg_{0.0}, camera_mount_pitch_deg_{0.0};
  double camera_mount_yaw_deg_{0.0};
  double camera_fov_horizontal_deg_{60.0}, camera_fov_vertical_deg_{45.0};
  int queue_size_{10};

  std::map<int, std::string> class_labels_;
  std::map<int, std::uint32_t> track_ages_;
  tracker::DetectionArray latest_detections_;
  tracker::TrackingOutput latest_tracking_output_;
  tracker::TrackStateArray latest_track_states_;
  pod_msgs::GimbalState latest_gimbal_state_;
  bool have_detections_{false};
  bool have_tracking_output_{false};
  bool have_track_states_{false};
  bool have_gimbal_state_{false};
  ros::WallTime gimbal_state_wall_time_;
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "pod_tracker_adapter");
  PodTrackerAdapter adapter;
  ros::spin();
  return 0;
}
