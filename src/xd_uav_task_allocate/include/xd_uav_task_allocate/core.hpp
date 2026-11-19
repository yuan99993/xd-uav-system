#pragma once

#include <array>
#include <cstdint>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace xd_uav_task_allocate {

using Vec3 = std::array<double, 3>;
using Vec2 = std::array<double, 2>;
using Quaternion = std::array<double, 4>;  // xyzw
using Covariance3 = std::array<double, 9>;

struct PoseSample {
  double stamp{0.0};
  Vec3 position{};
  Quaternion orientation{0.0, 0.0, 0.0, 1.0};
  std::string frame_id;
};

struct RigidTransform {
  Vec3 translation{};
  Quaternion rotation{0.0, 0.0, 0.0, 1.0};
  Vec3 applyVector(const Vec3& vector) const;
  Vec3 apply(const Vec3& point) const;
};

class PoseHistory {
 public:
  explicit PoseHistory(double maximum_age_sec = 5.0,
                       std::size_t maximum_size = 500);
  void add(const PoseSample& sample);
  std::optional<PoseSample> closest(double stamp,
                                    double maximum_difference_sec) const;
  std::optional<PoseSample> latest() const;

 private:
  double maximum_age_sec_;
  std::size_t maximum_size_;
  std::vector<PoseSample> samples_;
};

Vec3 rotateVector(const Quaternion& quaternion, const Vec3& vector);
Vec3 relativeFrdToShared(const Vec3& relative_frd, const PoseSample& pose,
                         const RigidTransform& world_from_reference = {});
Covariance3 rotateCovarianceFrdToShared(
    const Covariance3& covariance, const Quaternion& body_orientation,
    const Quaternion& reference_to_world = {0.0, 0.0, 0.0, 1.0});
Covariance3 rotateEnuCovarianceToWorld(const Covariance3& covariance,
                                      double enu_to_world_yaw);
Vec3 geodeticToWorld(double latitude_deg, double longitude_deg, double altitude,
                     const Vec3& datum_lla, double enu_to_world_yaw,
                     const Vec3& world_position);

struct SearchAreaDefinition {
  std::uint32_t area_id{0};
  std::vector<Vec2> boundary;
  double altitude{0.0};
  double lane_spacing{10.0};
  int priority{0};
};

struct PlannedArea {
  SearchAreaDefinition area;
  std::vector<Vec3> path;
  double length{0.0};
};

std::vector<Vec3> lawnmowerPath(const SearchAreaDefinition& area);
std::vector<Vec3> fixedwingLawnmowerPath(const SearchAreaDefinition& area,
                                         double minimum_turn_radius,
                                         double waypoint_spacing,
                                         std::optional<double> lead_distance = std::nullopt);
std::vector<Vec3> connectFixedwingPaths(const std::vector<std::vector<Vec3>>& paths,
                                        double minimum_turn_radius,
                                        double waypoint_spacing);
std::vector<Vec3> fixedwingEntryPath(const Vec3& current,
                                     double current_heading,
                                     const std::vector<Vec3>& route,
                                     double minimum_turn_radius,
                                     double waypoint_spacing);
std::vector<std::array<double, 3>> dubinsPath(
    const std::array<double, 3>& start, const std::array<double, 3>& goal,
    double minimum_turn_radius, double waypoint_spacing);
double pathLength(const std::vector<Vec3>& path);
std::map<std::string, std::vector<PlannedArea>> assignAreas(
    const std::vector<PlannedArea>& areas,
    const std::map<std::string, Vec3>& vehicle_positions,
    const std::map<std::string, double>& speeds,
    const std::map<std::pair<std::string, std::uint32_t>, double>& area_lengths = {});

enum TargetStatus : std::uint8_t {
  TARGET_CANDIDATE = 0,
  TARGET_CONFIRMED = 1,
  TARGET_ASSIGNED = 2,
  TARGET_EXECUTING = 3,
  TARGET_COMPLETED = 4,
  TARGET_STALE = 5,
  TARGET_VERIFYING = 6,
};

struct TargetObservation {
  std::string source_uav;
  std::string sensor_id;
  double stamp{0.0};
  int class_id{-1};
  Vec3 position{};
  Covariance3 covariance{};
  double confidence{0.0};
  int track_id{-1};
  bool track_id_is_stable{false};
  std::string mobility_profile{"hover"};
  std::optional<std::array<double,4>> normalized_bbox;
};

struct GlobalTargetRecord {
  std::uint32_t target_id{0};
  int class_id{-1};
  Vec3 position{};
  Covariance3 covariance{};
  double confidence{0.0};
  double first_seen{0.0};
  double last_seen{0.0};
  std::uint32_t observation_count{0};
  std::set<std::string> observer_uavs;
  std::map<std::string,std::string> observer_mobility_profiles;
  TargetStatus status{TARGET_CANDIDATE};
  std::vector<TargetObservation> observations;
  std::vector<std::pair<double,std::string>> recent_observations;
  double accumulated_weight{1.0};
  bool coarse_evidence_ready{false};
  std::set<std::tuple<std::string,std::string,int>> source_tracks;
  std::map<int,double> class_scores;
  std::vector<TargetObservation> recent_measurements;
  std::set<std::uint32_t> known_distinct_target_ids;
};

struct RegistryUpdate {
  GlobalTargetRecord target;
  bool created{false};
  bool newly_confirmed{false};
  bool newly_evidence_ready{false};
};

class GlobalTargetRegistry {
 public:
  GlobalTargetRegistry(double association_radius_m = 4.0,
                       int confirmation_hits = 5,
                       double confirmation_window_sec = 2.0,
                       double stale_timeout_sec = 15.0,
                       int confirmation_distinct_uavs = 2,
                       double confirmation_minimum_span_sec = 0.25,
                       const std::set<std::string>& confirmation_mobility_profiles = {"hover","fixedwing"},
                       double association_covariance_sigma = 3.0,
                       double maximum_association_radius_m = 50.0,
                       double cross_class_association_radius_m = 3.0,
                       double stable_track_maximum_jump_m = 20.0,
                       int class_confirmation_minimum_observations = 3,
                       double class_confirmation_minimum_ratio = 0.65,
                       double maximum_confirmation_position_spread_m = 3.0,
                       std::size_t position_history_size = 30,
                       double duplicate_bbox_iou_threshold = 0.50);
  RegistryUpdate observe(const TargetObservation& observation);
  bool setStatus(std::uint32_t target_id, TargetStatus status);
  bool remove(std::uint32_t target_id);
  void clear(bool reset_ids = true);
  std::vector<std::uint32_t> expire(double now);
  std::map<std::uint32_t, GlobalTargetRecord>& targets() { return targets_; }
  const std::map<std::uint32_t, GlobalTargetRecord>& targets() const {
    return targets_;
  }

 private:
  double association_radius_m_;
  int confirmation_hits_;
  double confirmation_window_sec_;
  double stale_timeout_sec_;
  int confirmation_distinct_uavs_;
  double confirmation_minimum_span_sec_;
  std::set<std::string> confirmation_mobility_profiles_;
  double association_covariance_sigma_{3.0};
  double maximum_association_radius_m_{50.0};
  double cross_class_association_radius_m_{3.0};
  double stable_track_maximum_jump_m_{20.0};
  int class_confirmation_minimum_observations_{3};
  double class_confirmation_minimum_ratio_{0.65};
  double maximum_confirmation_position_spread_m_{3.0};
  std::size_t position_history_size_{30};
  double duplicate_bbox_iou_threshold_{0.50};
  std::uint32_t next_target_id_{1};
  std::map<std::uint32_t, GlobalTargetRecord> targets_;
  std::map<std::tuple<std::string, std::string, int>, std::uint32_t>
      source_track_to_target_;
  GlobalTargetRecord* findTarget(const TargetObservation& observation);
  double associationRadius(const GlobalTargetRecord& target,
                           const TargetObservation& observation) const;
  bool sameSourceFrame(const GlobalTargetRecord& target,
                       const TargetObservation& observation) const;
  bool bboxDuplicate(const TargetObservation& first,
                     const TargetObservation& second) const;
  bool sameFrameDistinctTrack(const GlobalTargetRecord& target,
                              const TargetObservation& observation) const;
  bool sameFrameDuplicateTrack(const GlobalTargetRecord& target,
                               const TargetObservation& observation) const;
  bool compatible(const GlobalTargetRecord& target,
                  const TargetObservation& observation) const;
  bool positionStable(const std::vector<TargetObservation>& measurements) const;
  void refreshClassIdentity(GlobalTargetRecord& target) const;
};

enum TaskStatus : std::uint8_t {
  TASK_PENDING = 0,
  TASK_ASSIGNED = 1,
  TASK_EXECUTING = 2,
  TASK_COMPLETED = 3,
  TASK_FAILED = 4,
};

struct WorkerRecord {
  std::string name;
  Vec3 position{};
  bool online{false};
  double last_update{0.0};
  std::optional<std::uint32_t> assigned_task;
};

struct RescueTaskRecord {
  std::uint32_t task_id{0};
  std::uint32_t target_id{0};
  int class_id{-1};
  Vec3 target_position{};
  Vec3 goal{};
  int priority{0};
  std::string assigned_worker;
  TaskStatus status{TASK_PENDING};
  std::string detail{"waiting for an available worker"};
  std::set<std::uint32_t> known_distinct_target_ids;
};

struct Assignment {
  RescueTaskRecord task;
  WorkerRecord worker;
};

class RescueTaskAllocator {
 public:
  void registerWorker(const std::string& name);
  void updateWorker(const std::string& name, const Vec3& position,
                    double stamp, bool online = true);
  RescueTaskRecord& ensureTask(const GlobalTargetRecord& target, int priority,
                               double duplicate_radius_m = 0.0);
  std::vector<Assignment> assignPending();
  bool markExecuting(std::uint32_t task_id);
  std::optional<RescueTaskRecord> complete(std::uint32_t task_id);
  std::optional<RescueTaskRecord> releaseWorker(const std::string& name,
                                                const std::string& detail);
  std::vector<RescueTaskRecord> markOfflineWorkers(double now,
                                                   double timeout_sec);
  std::optional<RescueTaskRecord> cancelTask(std::uint32_t task_id,
                                             const std::string& detail);
  std::optional<RescueTaskRecord> retryTask(std::uint32_t task_id,
                                            const std::string& detail);
  std::optional<RescueTaskRecord> removeTargetTask(std::uint32_t target_id);
  void clearTasks(bool reset_ids = true);
  std::map<std::uint32_t, RescueTaskRecord>& tasks() { return tasks_; }
  const std::map<std::uint32_t, RescueTaskRecord>& tasks() const {
    return tasks_;
  }
  std::map<std::string, WorkerRecord>& workers() { return workers_; }
  const std::map<std::string, WorkerRecord>& workers() const {
    return workers_;
  }

 private:
  std::map<std::string, WorkerRecord> workers_;
  std::map<std::uint32_t, RescueTaskRecord> tasks_;
  std::map<std::uint32_t, std::uint32_t> target_to_task_;
  std::uint32_t next_task_id_{1};
};

}  // namespace xd_uav_task_allocate
