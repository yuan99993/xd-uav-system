#pragma once

#include <array>
#include <deque>
#include <memory>
#include <string>

#include <xd_uav_track/target_guidance.hpp>

namespace xd_uav_track {

enum class FollowerProfile {
  kVelocityGround,
  kVelocityPosition,
  kVelocityDistance,
  kVelocityChase,
  kGimbalVelocityChase,
  kGimbalVelocityVector,
  kFixedWingVelocityVector,
};

enum class LateralGuidanceMode {
  kCoordinatedTurn,
  kSideslip,
};

struct PidConfig {
  double kp{0.0};
  double ki{0.0};
  double kd{0.0};
  double integral_limit{1.0};
};

struct TrackControllerConfig {
  // Tracker: selected image box validation and temporal filtering.
  double input_timeout_sec{0.25};
  double minimum_confidence{0.50};
  double confidence_hysteresis{0.08};
  double position_filter_alpha{0.40};
  double velocity_filter_alpha{0.25};
  double target_x{0.0};
  double target_y{0.0};

  // Fixed-camera velocity profiles plus dedicated MC rate and FW vector modes.
  FollowerProfile profile{FollowerProfile::kVelocityChase};
  LateralGuidanceMode lateral_guidance{
      LateralGuidanceMode::kCoordinatedTurn};
  LateralGuidanceMode gm_chase_lateral_guidance{
      LateralGuidanceMode::kCoordinatedTurn};
  LateralGuidanceMode gm_vector_lateral_guidance{
      LateralGuidanceMode::kSideslip};
  bool enable_vertical_control{true};
  bool distance_enable_yaw{false};
  double horizontal_deadband{0.03};
  double vertical_deadband{0.03};
  PidConfig lateral_pid{1.5, 0.08, 0.20, 1.0};
  PidConfig vertical_pid{1.0, 0.08, 0.15, 1.0};
  PidConfig yaw_pid{2.0, 0.05, 0.10, 0.8};
  PidConfig ground_forward_pid{4.5, 0.10, 1.50, 1.0};

  bool ground_altitude_scaling_enabled{true};
  bool ground_attitude_compensation_enabled{true};
  double ground_base_adjustment_x{0.2};
  double ground_base_adjustment_y{0.2};
  double ground_altitude_factor{0.005};
  bool ground_descend_to_target{false};
  double ground_target_altitude{3.0};
  double ground_maximum_descent_velocity{0.5};

  double initial_forward_velocity{0.0};
  double chase_forward_velocity{3.0};
  double forward_ramp_rate{0.5};
  double target_loss_forward_velocity{0.0};
  double target_loss_reset_sec{3.0};

  double maximum_forward_velocity{5.0};
  double maximum_reverse_velocity{2.0};
  double maximum_lateral_velocity{2.0};
  double maximum_vertical_velocity{1.5};
  double maximum_yaw_rate{1.2};

  bool velocity_smoothing_enabled{true};
  double velocity_ema_alpha{0.20};
  bool yaw_smoothing_enabled{true};
  double yaw_deadzone{0.01};
  double yaw_max_acceleration{1.57};
  double yaw_ema_alpha{0.70};
  bool yaw_speed_scaling_enabled{true};
  double yaw_min_speed{0.5};
  double yaw_max_speed{5.0};
  double yaw_low_speed_factor{0.5};

  // Gimbal follower input and mount calibration. Input angles use radians.
  double gimbal_input_timeout_sec{0.20};
  std::string gimbal_mount_type{"HORIZONTAL"};
  double gimbal_angle_filter_previous_weight{0.70};
  double gimbal_angle_deadzone{0.035};
  double gimbal_yaw_offset{0.0};
  double gimbal_pitch_offset{0.0};
  double gimbal_roll_offset{0.0};
  bool gimbal_invert_yaw{false};
  bool gimbal_invert_pitch{false};
  bool gimbal_invert_roll{false};

  std::string gm_chase_forward_mode{"CONSTANT"};
  double gm_chase_base_forward_speed{2.0};
  double gm_chase_forward_acceleration{2.0};
  double gm_chase_pitch_velocity_scaling{0.15};
  double gm_chase_pitch_deadzone{0.035};
  double gm_chase_maximum_lateral_angle{1.57};
  double gm_chase_maximum_vertical_angle{1.57};

  double gm_vector_velocity{3.0};
  double gm_vector_ramp_acceleration{0.25};
  bool gm_vector_enable_vertical_control{false};
  double gm_vector_yaw_rate_gain{0.5};

  // Fixed-wing follower output is a single velocity vector. Low-level
  // course/energy control and throttle remain in xd_uav_controller.
  PidConfig fw_course_pid{1.2, 0.05, 0.10, 0.35};
  PidConfig fw_climb_rate_pid{2.0, 0.05, 0.10, 0.8};
  double fw_commanded_airspeed{15.0};
  double fw_maximum_course_offset{0.70};
  double fw_maximum_climb_rate{3.0};

  // Track-quality safety. Kalman covariance is expressed in normalized image
  // coordinates, so these limits are independent of camera resolution.
  bool uncertainty_control_enabled{true};
  double uncertainty_nominal_sigma{0.05};
  double uncertainty_slow_ratio{1.0};
  double uncertainty_abort_ratio{2.5};
  double uncertainty_minimum_scale{0.25};
  double uncertainty_yaw_scale{0.65};
  double reidentification_initial_scale{0.35};
  double reidentification_recovery_sec{0.60};

  // Optional metric relative-state control for chase profiles. Values use
  // body forward/right/down axes and supplement image centering.
  bool relative_state_control_enabled{true};
  double relative_target_forward{8.0};
  double relative_target_down{0.0};
  double relative_position_gain{0.50};
  double relative_velocity_feedforward{1.0};
  double relative_maximum_correction{3.0};

  // Optional metric/FOV guidance.  It is deliberately independent of the
  // vehicle profile: fixed-wing consumes the vector as a course reference,
  // while a multicopter can consume the same FLU vector directly.
  TargetGuidanceConfig target_guidance;
};

struct VehicleState {
  double roll{0.0};
  double pitch{0.0};
  double altitude{0.0};
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double yaw{0.0};
  double receive_time{0.0};
  // Optional odometry/header time.  receive_time stays in the local arrival
  // clock used by safety timeouts; this value is used only for capture-time
  // metric fusion and OOSM replay.
  double observation_time{0.0};
  bool pose_valid{false};
  bool valid{false};
};

struct GimbalStateData {
  double receive_time{0.0};
  // GimbalStatus has an independent transport cadence from the angle stream.
  // A zero value preserves compatibility with angle-only publishers and means
  // that no status freshness check is required yet.
  double status_receive_time{0.0};
  // PixEagle gimbal convention: +yaw right, +pitch down.
  double yaw{0.0};
  double pitch{0.0};
  double roll{0.0};
  bool valid{false};
  // Optional GimbalStatus contract.  Angle-only publishers remain valid;
  // when present, these fields gate metric guidance and live FOV use.
  bool status_valid{false};
  bool healthy{true};
  bool tracking_active{true};
  bool range_valid{true};
  bool control_authority_available{true};
  double zoom_ratio{0.0};
  double horizontal_fov_rad{0.0};
  double vertical_fov_rad{0.0};
  double range_quality{0.0};
};

struct TargetMeasurement {
  double receive_time{0.0};
  // Sensor capture time for metric fusion. receive_time remains the local
  // arrival time used by the legacy image timeout path.
  double observation_time{0.0};
  unsigned int image_width{0};
  unsigned int image_height{0};
  double x_min{0.0};
  double y_min{0.0};
  double x_max{0.0};
  double y_max{0.0};
  double confidence{0.0};
  int track_id{-1};
  int class_id{-1};
  bool predicted{false};
  bool reidentification_match{false};
  double tracking_quality{1.0};
  std::string association_method{"direct"};
  std::array<double, 4> state_covariance{{0.0, 0.0, 0.0, 0.0}};
  bool has_relative_position_body{false};
  std::array<double, 3> relative_position_body{{0.0, 0.0, 0.0}};
  bool has_relative_velocity_body{false};
  std::array<double, 3> relative_velocity_body{{0.0, 0.0, 0.0}};
  bool range_valid{false};
  std::string image_source;
  // Metric position uncertainty in metres when supplied by the detector.
  // Zero means unknown and is conservatively replaced by 1 m.
  double position_sigma_m{0.0};
};

struct TrackVelocity {
  // ROS body FLU convention: forward, left, up, counter-clockwise yaw.
  double forward{0.0};
  double left{0.0};
  double up{0.0};
  double yaw_rate{0.0};

  // Normalized image coordinates: +x right, +y down, range approximately [-1, 1].
  double error_x{0.0};
  double error_y{0.0};
  double center_x{0.0};
  double center_y{0.0};
  double center_velocity_x{0.0};
  double center_velocity_y{0.0};
  double target_size_ratio{0.0};
  double input_age_sec{0.0};
  double target_loss_duration_sec{0.0};
  int track_id{-1};
  bool target_visible{false};
  bool target_predicted{false};
  bool valid{false};
  bool uncertainty_limited{false};
  bool relative_state_active{false};
  double tracking_quality{0.0};
  double uncertainty_scale{1.0};
  // Fixed-wing vector guidance intentionally supplies no separate yaw-rate;
  // the horizontal velocity direction is the complete course reference.
  bool use_yaw_rate{true};
  // Fixed-wing cannot stop in place. On target loss, release the streaming
  // reference so xd_uav_controller can enter its configured timeout loiter.
  bool release_reference_on_invalid{false};
  // Optional internal position anchor for fixed-wing orbit guidance.  These
  // fields are consumed only by xd_uav_track_node when publishing the
  // existing MAVROS PositionTarget; no ROS tracker message is changed.
  bool position_reference_valid{false};
  std::array<double, 3> position_reference{{0.0, 0.0, 0.0}};
  std::string profile;
  std::string lateral_guidance_mode;
  std::string tracking_state;
  std::string association_method;
  std::string invalid_reason;
};

class TrackController {
 public:
  explicit TrackController(const TrackControllerConfig& config);

  bool updateMeasurement(const TargetMeasurement& measurement,
                         std::string* rejection_reason = nullptr);
  // Update only the inertial metric state. This is used for a fresh standby
  // camera in a dual-source setup without replacing the active image track.
  bool updateMetricMeasurement(const TargetMeasurement& measurement,
                               std::string* rejection_reason = nullptr);
  // ``updateMeasurement()`` may retain a valid 2-D image track after its
  // metric component is rejected.  Callers that maintain a metric identity
  // anchor must use this flag rather than treating the image update itself as
  // proof that the world-frame observation was accepted.
  bool lastMetricMeasurementAccepted() const;
  // Read-only cross-camera association gate.  It projects the candidate at
  // its capture time and compares it with the current inertial target state;
  // callers use it before allowing a standby camera to update the filter.
  bool metricMeasurementCompatible(const TargetMeasurement& measurement,
                                   double maximum_distance_m,
                                   std::string* rejection_reason = nullptr) const;
  TrackVelocity compute(double now);
  void setVehicleState(const VehicleState& state);
  void setGimbalState(const GimbalStateData& state);
  void clearMeasurement(const std::string& reason = "target box is invalid");
  bool setProfile(FollowerProfile profile);
  FollowerProfile profile() const;
  void reset();

 private:
  class Pid {
   public:
    void configure(const PidConfig& config, double output_limit);
    double update(double error, double dt);
    void reset();

   private:
    PidConfig config_;
    double output_limit_{0.0};
    double integral_{0.0};
    double previous_error_{0.0};
    bool initialized_{false};
  };

  static double clamp(double value, double minimum, double maximum);
  static double applyDeadband(double value, double threshold);
  static double moveToward(double value, double target,
                           double maximum_delta);
  double smoothYaw(double raw_yaw, double dt, double forward_speed);
  bool filteredGimbalAngles(double now, double* yaw, double* pitch,
                            double* roll);
  void gimbalToBodyVector(double yaw, double pitch, double roll,
                          double* forward, double* right, double* down) const;
  void resetFollowerState();
  TrackVelocity baseOutput(double now) const;
  bool integrateMetricMeasurement(const TargetMeasurement& incoming,
                                  std::array<double, 3>* filtered_world,
                                  std::string* rejection_reason);

  static constexpr std::size_t kWorldFilterModelCount = 3;

  enum class WorldMotionModel {
    kStationary,
    kConstantVelocity,
    kCoordinatedTurn,
  };

  struct WorldFilterModel {
    bool initialized{false};
    WorldMotionModel motion_model{WorldMotionModel::kStationary};
    double stamp{0.0};
    std::array<double, 3> position{{0.0, 0.0, 0.0}};
    std::array<double, 3> velocity{{0.0, 0.0, 0.0}};
    std::array<double, 3> p_position{{1.0, 1.0, 1.0}};
    std::array<double, 3> p_position_velocity{{0.0, 0.0, 0.0}};
    std::array<double, 3> p_velocity{{25.0, 25.0, 25.0}};
    // The coordinated-turn model rotates the horizontal velocity around ENU
    // up.  Stationary/CV models keep this value at zero.
    double turn_rate_radps{0.0};
    double p_turn_rate{0.25};
  };

  struct MetricObservation {
    double stamp{0.0};
    std::array<double, 3> world_position{{0.0, 0.0, 0.0}};
    std::array<double, 3> world_velocity{{0.0, 0.0, 0.0}};
    bool velocity_valid{false};
    double sigma_m{1.0};
    int class_id{-1};
    std::string image_source;
  };

  struct WorldFilterState {
    bool initialized{false};
    double stamp{0.0};
    std::array<WorldFilterModel, kWorldFilterModelCount> models;
    std::array<double, kWorldFilterModelCount> probabilities{{
        1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0}};
  };

  struct WorldFilterHistory {
    MetricObservation observation;
    WorldFilterState state_after;
  };

  bool metricObservationFromMeasurement(const TargetMeasurement& incoming,
                                        MetricObservation* observation,
                                        std::string* rejection_reason) const;
  bool vehicleStateAtObservationTime(double stamp, VehicleState* state) const;
  bool applyMetricObservation(const MetricObservation& observation,
                              bool enforce_gate,
                              std::string* rejection_reason);
  bool replayMetricHistory(std::string* rejection_reason);
  void updateMetricStateFromWorldFilter();

  TrackControllerConfig config_;
  Pid lateral_pid_;
  Pid vertical_pid_;
  Pid yaw_pid_;
  Pid ground_forward_pid_;
  Pid fw_course_pid_;
  Pid fw_climb_rate_pid_;

  bool have_measurement_{false};
  bool ever_acquired_{false};
  bool confidence_locked_{false};
  bool filter_initialized_{false};
  bool loss_active_{false};
  bool loss_controllers_reset_{false};
  TargetMeasurement measurement_;
  double filtered_center_x_{0.0};
  double filtered_center_y_{0.0};
  double filtered_velocity_x_{0.0};
  double filtered_velocity_y_{0.0};
  double filtered_size_ratio_{0.0};
  double previous_measurement_time_{0.0};
  int filtered_track_id_{-1};

  double previous_compute_time_{0.0};
  double forward_velocity_{0.0};
  double smoothed_left_{0.0};
  double smoothed_up_{0.0};
  double yaw_rate_limited_{0.0};
  double smoothed_yaw_{0.0};
  double loss_start_time_{0.0};
  std::string cleared_reason_{"target box has not been received"};
  VehicleState vehicle_state_;
  GimbalStateData gimbal_state_;
  std::unique_ptr<TargetGuidance> target_guidance_;
  bool have_metric_state_{false};
  bool last_metric_measurement_accepted_{false};
  double metric_state_time_{0.0};
  std::array<double, 3> last_metric_position_frd_{{0.0, 0.0, 0.0}};
  std::array<double, 3> last_metric_velocity_frd_{{0.0, 0.0, 0.0}};
  bool last_metric_velocity_valid_{false};
  double last_metric_sigma_m_{1.0};
  bool have_metric_world_state_{false};
  double metric_world_time_{0.0};
  std::array<double, 3> last_metric_world_position_{{0.0, 0.0, 0.0}};
  std::array<double, 3> last_metric_world_velocity_{{0.0, 0.0, 0.0}};
  bool last_metric_world_velocity_valid_{false};
  double last_metric_world_sigma_m_{1.0};
  std::array<WorldFilterModel, kWorldFilterModelCount> world_filter_models_;
  std::array<double, kWorldFilterModelCount> world_filter_probabilities_{{
      1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0}};
  WorldFilterState world_filter_baseline_;
  std::deque<WorldFilterHistory> world_filter_history_;
  std::deque<VehicleState> vehicle_state_history_;
  bool gimbal_filter_initialized_{false};
  double filtered_gimbal_yaw_{0.0};
  double filtered_gimbal_pitch_{0.0};
  double filtered_gimbal_roll_{0.0};
  double gm_chase_forward_velocity_{0.0};
  double gm_vector_velocity_{0.0};
  double reidentification_time_{-1.0};
};

bool parseFollowerProfile(const std::string& value, FollowerProfile* profile);
const char* followerProfileName(FollowerProfile profile);
bool parseLateralGuidanceMode(const std::string& value,
                              LateralGuidanceMode* mode);
const char* lateralGuidanceModeName(LateralGuidanceMode mode);

}  // namespace xd_uav_track
