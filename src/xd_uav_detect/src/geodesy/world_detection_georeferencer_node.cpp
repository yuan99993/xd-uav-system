#include <algorithm>
#include <cmath>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geographic_msgs/GeoPointStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>

#include <xd_uav_detect/GeodeticDetectionArray.h>
#include <xd_uav_detect/WorldDetectionArray.h>
#include <xd_uav_detect/geodesy/local_cartesian_projector.hpp>

namespace xd_uav_detect {
namespace {

bool finiteTransform(const geometry_msgs::Transform& transform) {
  const auto& translation = transform.translation;
  const auto& rotation = transform.rotation;
  return std::isfinite(translation.x) && std::isfinite(translation.y) &&
      std::isfinite(translation.z) && std::isfinite(rotation.x) &&
      std::isfinite(rotation.y) && std::isfinite(rotation.z) &&
      std::isfinite(rotation.w);
}

bool transformParts(const geometry_msgs::Transform& transform,
                    Eigen::Matrix3d* rotation, Eigen::Vector3d* translation) {
  if (rotation == nullptr || translation == nullptr ||
      !finiteTransform(transform)) {
    return false;
  }
  Eigen::Quaterniond quaternion(transform.rotation.w, transform.rotation.x,
                               transform.rotation.y, transform.rotation.z);
  if (!std::isfinite(quaternion.norm()) || quaternion.norm() < 1e-12) {
    return false;
  }
  quaternion.normalize();
  *rotation = quaternion.toRotationMatrix();
  *translation = Eigen::Vector3d(transform.translation.x,
                                 transform.translation.y,
                                 transform.translation.z);
  return rotation->array().isFinite().all();
}

}  // namespace

class WorldDetectionGeoreferencer {
 public:
  WorldDetectionGeoreferencer()
      : private_nh_("~"), tf_listener_(tf_buffer_) {
    private_nh_.param<std::string>("input_topic", input_topic_,
                                   "detect/detections_world");
    private_nh_.param<std::string>("output_topic", output_topic_,
                                   "detect/detections_geodetic");
    private_nh_.param<std::string>("origin_topic", origin_topic_,
                                   "mavros/global_position/gp_origin");
    private_nh_.param<std::string>("local_origin_frame", local_origin_frame_,
                                   "local_origin");
    private_nh_.param("tf_timeout_sec", tf_timeout_sec_, 0.1);
    private_nh_.param("use_static_origin", use_static_origin_, false);
    tf_timeout_sec_ = std::max(0.0, tf_timeout_sec_);

    if (use_static_origin_) {
      geographic_msgs::GeoPoint origin;
      std::string error;
      if (!private_nh_.getParam("static_origin/latitude", origin.latitude) ||
          !private_nh_.getParam("static_origin/longitude", origin.longitude) ||
          !private_nh_.getParam("static_origin/altitude", origin.altitude)) {
        ROS_ERROR("[georeferencer] static origin parameters are incomplete");
      } else if (!projector_.reset(origin, &error)) {
        ROS_ERROR("[georeferencer] invalid static origin: %s", error.c_str());
      }
    } else {
      origin_subscriber_ = nh_.subscribe(
          origin_topic_, 1, &WorldDetectionGeoreferencer::originCallback,
          this);
    }

    publisher_ = nh_.advertise<xd_uav_detect::GeodeticDetectionArray>(
        output_topic_, 10);
    subscriber_ = nh_.subscribe(
        input_topic_, 10,
        &WorldDetectionGeoreferencer::detectionsCallback, this);
  }

 private:
  void originCallback(const geographic_msgs::GeoPointStamped::ConstPtr& msg) {
    std::string error;
    if (!projector_.reset(msg->position, &error)) {
      ROS_WARN_THROTTLE(2.0, "[georeferencer] rejected origin: %s",
                        error.c_str());
      return;
    }
  }

  void warnFailure(const std::string& reason) const {
    ROS_WARN_THROTTLE(2.0, "[georeferencer] fail closed: %s",
                      reason.c_str());
  }

  bool lookupInputTransform(const WorldDetectionArray& input,
                            Eigen::Matrix3d* rotation,
                            Eigen::Vector3d* translation,
                            std::string* error) {
    if (input.header.stamp.isZero()) {
      *error = "source timestamp is zero";
      return false;
    }
    if (input.header.frame_id.empty() || local_origin_frame_.empty()) {
      *error = "input or local-origin frame is empty";
      return false;
    }
    if (input.header.frame_id == local_origin_frame_) {
      *rotation = Eigen::Matrix3d::Identity();
      *translation = Eigen::Vector3d::Zero();
      return true;
    }
    try {
      const auto transform = tf_buffer_.lookupTransform(
          local_origin_frame_, input.header.frame_id, input.header.stamp,
          ros::Duration(tf_timeout_sec_));
      if (!transformParts(transform.transform, rotation, translation)) {
        *error = "TF contains an invalid transform";
        return false;
      }
      return true;
    } catch (const tf2::TransformException& exception) {
      *error = exception.what();
      return false;
    }
  }

  static void copyMetadata(const WorldDetection& source,
                           GeodeticDetection* target) {
    target->source_candidate_index = source.source_candidate_index;
    target->track_id = source.track_id;
    target->class_id = source.class_id;
    target->confidence = source.confidence;
    target->image_source = source.image_source;
    target->sensor_id = source.sensor_id;
    target->detector_name = source.detector_name;
    target->model_version = source.model_version;
    target->position_valid = false;
  }

  void detectionsCallback(const WorldDetectionArray::ConstPtr& input) {
    GeodeticDetectionArray output;
    output.header = input->header;
    output.header.frame_id = local_origin_frame_;
    output.detections.resize(input->detections.size());
    for (std::size_t index = 0; index < input->detections.size(); ++index) {
      copyMetadata(input->detections[index], &output.detections[index]);
    }

    std::string failure;
    Eigen::Matrix3d local_from_input_rotation = Eigen::Matrix3d::Identity();
    Eigen::Vector3d local_from_input_translation = Eigen::Vector3d::Zero();
    if (!projector_.initialized()) {
      failure = "geographic origin unavailable";
    } else if (!lookupInputTransform(*input, &local_from_input_rotation,
                                     &local_from_input_translation, &failure)) {
      // failure is populated by lookupInputTransform.
    } else {
      for (std::size_t index = 0; index < input->detections.size(); ++index) {
        const auto& source = input->detections[index];
        auto& target = output.detections[index];
        if (!source.position_valid) continue;

        const Eigen::Vector3d input_position(
            source.position_world.x, source.position_world.y,
            source.position_world.z);
        Eigen::Matrix3d input_covariance;
        for (int row = 0; row < 3; ++row) {
          for (int column = 0; column < 3; ++column) {
            input_covariance(row, column) =
                source.position_covariance_world[3 * row + column];
          }
        }
        if (!input_position.array().isFinite().all() ||
            !input_covariance.array().isFinite().all() ||
            !input_covariance.isApprox(input_covariance.transpose(), 1e-9)) {
          continue;
        }
        const Eigen::Vector3d local_position =
            local_from_input_rotation * input_position +
            local_from_input_translation;
        Eigen::Matrix3d local_covariance =
            local_from_input_rotation * input_covariance *
            local_from_input_rotation.transpose();
        local_covariance =
            0.5 * (local_covariance + local_covariance.transpose());
        if (!local_covariance.array().isFinite().all()) continue;

        std::string projection_error;
        if (!projector_.reverse(local_position, &target.position,
                                &projection_error)) {
          continue;
        }
        for (int row = 0; row < 3; ++row) {
          for (int column = 0; column < 3; ++column) {
            target.position_covariance_enu[3 * row + column] =
                local_covariance(row, column);
          }
        }
        target.position_valid = true;
      }
    }
    if (!failure.empty()) warnFailure(failure);
    publisher_.publish(output);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  ros::Subscriber subscriber_;
  ros::Subscriber origin_subscriber_;
  ros::Publisher publisher_;
  LocalCartesianProjector projector_;
  std::string input_topic_;
  std::string output_topic_;
  std::string origin_topic_;
  std::string local_origin_frame_;
  double tf_timeout_sec_{0.1};
  bool use_static_origin_{false};
};

}  // namespace xd_uav_detect

int main(int argc, char** argv) {
  ros::init(argc, argv, "world_detection_georeferencer");
  xd_uav_detect::WorldDetectionGeoreferencer node;
  ros::spin();
  return 0;
}
