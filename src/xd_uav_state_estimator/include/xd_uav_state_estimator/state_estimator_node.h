#pragma once

#include <diagnostic_msgs/DiagnosticArray.h>
#include <geometry_msgs/AccelStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <xd_uav_state_estimator/imu_acceleration_processor.h>
#include <xd_uav_state_estimator/heading_filter.h>
#include <xd_uav_state_estimator/linear_filters.h>
#include <xd_uav_state_estimator/localization_source_manager.h>
#include <xd_uav_state_estimator/EstimatorStatus.h>
#include <xd_uav_state_estimator/SwitchLocalizationSource.h>

#include <Eigen/Dense>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace xd_uav_state_estimator {

class StateEstimatorNode {
public:
  StateEstimatorNode(ros::NodeHandle nh, ros::NodeHandle private_nh);

private:
  struct SourceAlignment {
    bool initialized{false};
    std::string parent_frame;
    tf2::Quaternion rotation{0.0, 0.0, 0.0, 1.0};
    Eigen::Vector3d translation{Eigen::Vector3d::Zero()};
    std::vector<Eigen::Vector3d> translation_candidates;
    std::vector<double> heading_candidates;
  };

  struct PredictionRecord {
    ros::Time start_stamp;
    ros::Time end_stamp;
    Eigen::Vector3d acceleration{Eigen::Vector3d::Zero()};
    bool acceleration_valid{false};
  };

  struct FilterHistoryEntry {
    ros::Time stamp;
    LinearFilters::Snapshot snapshot;
  };

  struct SourceEstimator {
    SourceEstimator(const LocalizationSourceConfig& source_config,
                    const FilterParameters& filter_parameters,
                    const HeadingFilterParameters& heading_parameters)
        : config(source_config),
          filters(filter_parameters),
          heading_filter(heading_parameters) {}

    LocalizationSourceConfig config;
    LinearFilters filters;
    HeadingFilter heading_filter;
    ros::Publisher odometry_publisher;
    ros::Publisher valid_publisher;
    geometry_msgs::Quaternion orientation;
    geometry_msgs::Vector3 angular_velocity;
    ros::Time last_receive_time;
  };

  void localizationCallback(const LocalizationMeasurement& measurement,
                            const LocalizationDecision& decision);
  void imuCallback(const sensor_msgs::Imu::ConstPtr& message);
  void controlInputCallback(const geometry_msgs::AccelStamped::ConstPtr& message);
  void publishTimerCallback(const ros::TimerEvent& event);
  void diagnosticsTimerCallback(const ros::TimerEvent& event);
  bool resetCallback(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response);
  bool switchSourceCallback(
      SwitchLocalizationSource::Request& request,
      SwitchLocalizationSource::Response& response);

  void loadParameters();
  void initializeFilter(const Eigen::Vector3d& position, const Eigen::Vector3d& velocity,
                        const ros::Time& stamp);
  void initializeHeading(const geometry_msgs::Quaternion& orientation,
                         double angular_rate, bool use_source_heading,
                         const ros::Time& stamp);
  bool predictTo(const ros::Time& stamp, const Eigen::Vector3d& acceleration,
                 bool acceleration_valid);
  void recordFilterSnapshot();
  void pruneFilterHistory();
  bool rewindFilterTo(const ros::Time& stamp,
                      std::vector<PredictionRecord>* replay_records,
                      ros::Time* present_stamp);
  void replayPredictions(const std::vector<PredictionRecord>& records,
                         const ros::Time& target_stamp);
  bool alignMeasurement(const LocalizationMeasurement& measurement,
                        const LocalizationDecision& decision,
                        Eigen::Vector3d* position, Eigen::Vector3d* velocity,
                        geometry_msgs::Quaternion* orientation,
                        bool* alignment_pending);
  void initializeSourceEstimators();
  void updateSourceEstimator(const LocalizationMeasurement& measurement,
                             const Eigen::Vector3d& position,
                             const Eigen::Vector3d& velocity,
                             const geometry_msgs::Quaternion& orientation);
  void publishSourceEstimator(SourceEstimator& estimator);
  void publishSourceValidity(const ros::Time& now);
  void publishSourceOriginTransforms(const ros::Time& stamp);
  void publishState();
  void publishDiagnostics();
  bool publishEstimatorStatus(const ros::Time& now,
                              bool localization_valid);

  Eigen::Vector3d predictionAccelerationInOdom(const ros::Time& stamp,
                                               const tf2::Quaternion& body_orientation,
                                               bool* valid, std::string* source) const;
  Eigen::Vector3d controlAccelerationInOdom(const ros::Time& stamp,
                                            const tf2::Quaternion& body_orientation,
                                            bool* valid) const;

  static bool finite(double value);
  static bool finite(const geometry_msgs::Vector3& vector);
  static bool finite(const geometry_msgs::Point& point);
  static bool normalizeQuaternion(const geometry_msgs::Quaternion& message, tf2::Quaternion* quaternion);
  static Eigen::Vector3d vectorToEigen(const geometry_msgs::Vector3& vector);
  static geometry_msgs::Vector3 eigenToVector(const Eigen::Vector3d& vector);
  static Eigen::Vector3d rotateBodyToParent(const Eigen::Vector3d& vector,
                                            const tf2::Quaternion& orientation);
  static Eigen::Vector3d rotateParentToBody(const Eigen::Vector3d& vector,
                                            const tf2::Quaternion& orientation);

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  ros::Subscriber imu_subscriber_;
  ros::Subscriber control_input_subscriber_;
  ros::Publisher main_odometry_publisher_;
  ros::Publisher acceleration_publisher_;
  ros::Publisher imu_acceleration_publisher_;
  ros::Publisher innovation_publisher_;
  ros::Publisher diagnostics_publisher_;
  ros::Publisher localization_valid_publisher_;
  ros::Publisher state_valid_publisher_;
  ros::Publisher estimator_status_publisher_;
  ros::ServiceServer reset_server_;
  ros::ServiceServer switch_source_server_;
  ros::Timer publish_timer_;
  ros::Timer diagnostics_timer_;
  tf2_ros::TransformBroadcaster transform_broadcaster_;

  mutable std::mutex mutex_;
  std::unique_ptr<LinearFilters> filters_;
  std::unique_ptr<HeadingFilter> heading_filter_;
  std::unique_ptr<ImuAccelerationProcessor> imu_acceleration_processor_;
  std::unique_ptr<LocalizationSourceManager> localization_source_manager_;
  FilterParameters filter_parameters_;
  HeadingFilterParameters heading_filter_parameters_;
  std::unordered_map<std::string, std::unique_ptr<SourceEstimator>>
      source_estimators_;

  std::string uav_name_;
  std::string world_frame_;
  std::string map_frame_;
  std::string odom_frame_;
  std::string body_frame_;

  double output_rate_{100.0};
  double diagnostics_rate_{2.0};
  double input_timeout_{0.25};
  double active_localization_timeout_{0.25};
  double imu_timeout_{0.25};
  double control_input_timeout_{0.1};
  double max_prediction_step_{0.02};
  double reset_gap_{1.0};
  double max_localization_delay_{0.10};
  double filter_history_duration_{1.0};
  double max_dead_reckoning_time_{1.0};
  double position_xy_variance_{0.01};
  double position_z_variance_{0.05};
  double velocity_xy_variance_{0.01};
  double velocity_z_variance_{0.02};
  double heading_variance_{0.02};
  double heading_rate_variance_{0.02};
  double position_xy_innovation_limit_{5.0};
  double position_z_innovation_limit_{3.0};
  double velocity_xy_innovation_limit_{5.0};
  double velocity_z_innovation_limit_{3.0};
  double heading_innovation_limit_{1.57};
  double position_xy_nis_limit_{9.21};
  double position_z_nis_limit_{6.63};
  double velocity_xy_nis_limit_{9.21};
  double velocity_z_nis_limit_{6.63};
  double heading_nis_limit_{6.63};
  bool odom_twist_in_body_frame_{true};
  bool output_twist_in_body_frame_{true};
  bool publish_tf_{false};
  bool publish_source_origin_tf_{true};
  bool require_imu_{false};
  bool use_imu_prediction_{true};
  bool use_control_input_{false};
  bool use_imu_heading_rate_{true};
  bool use_mahalanobis_gate_{true};
  bool delayed_measurement_enabled_{true};
  bool replaying_prediction_history_{false};

  bool have_odom_{false};
  bool have_imu_{false};
  bool have_control_input_{false};
  bool imu_acceleration_valid_{false};
  bool healthy_{false};
  std::string health_message_{"waiting for odometry"};
  std::string prediction_source_{"model"};

  ros::Time last_odom_stamp_;
  ros::Time last_imu_stamp_;
  ros::Time last_odom_receive_time_;
  ros::Time last_imu_receive_time_;
  ros::Time last_control_receive_time_;
  std::unordered_map<std::string, ros::Time> last_source_stamps_;
  std::unordered_map<std::string, SourceAlignment> source_alignments_;
  std::deque<PredictionRecord> prediction_history_;
  std::deque<FilterHistoryEntry> filter_history_;

  geometry_msgs::Quaternion odom_orientation_;
  geometry_msgs::Vector3 odom_angular_velocity_;
  sensor_msgs::Imu latest_imu_;
  geometry_msgs::AccelStamped latest_control_input_;
  Eigen::Vector3d latest_odometry_velocity_odom_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d latest_imu_acceleration_odom_{Eigen::Vector3d::Zero()};

  Eigen::Vector3d position_innovation_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity_innovation_{Eigen::Vector3d::Zero()};
  double heading_innovation_{0.0};
  double position_xy_nis_{0.0};
  double position_z_nis_{0.0};
  double velocity_xy_nis_{0.0};
  double velocity_z_nis_{0.0};
  double heading_nis_{0.0};

  std::uint64_t received_odometry_{0};
  std::uint64_t rejected_odometry_{0};
  std::uint64_t ignored_localization_{0};
  std::uint64_t received_imu_{0};
  std::uint64_t rejected_imu_{0};
  std::uint64_t imu_prediction_count_{0};
  std::uint64_t prediction_count_{0};
  std::uint64_t correction_count_{0};
  std::uint64_t heading_correction_count_{0};
  std::uint64_t heading_rejection_count_{0};
  std::uint64_t delayed_correction_count_{0};
  std::uint64_t delayed_rejection_count_{0};
  std::uint64_t reset_count_{0};
};

}  // 命名空间 xd_uav_state_estimator
