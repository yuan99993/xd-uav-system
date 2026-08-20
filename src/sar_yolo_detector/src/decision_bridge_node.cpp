#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <XmlRpcValue.h>
#include <openssl/crypto.h>
#include <openssl/hmac.h>
#include <ros/ros.h>
#include <sys/stat.h>
#include <unistd.h>

#include <sar_yolo_detector/AreaTaskAssignment.h>
#include <sar_yolo_detector/ExecutorHeartbeat.h>
#include <sar_yolo_detector/GetEvidenceCrop.h>
#include <sar_yolo_detector/GetPerceptionSnapshot.h>
#include <sar_yolo_detector/GetTaskStatuses.h>
#include <sar_yolo_detector/PerceptionCandidateArray.h>
#include <sar_yolo_detector/SubmitAreaTaskAssignment.h>
#include <sar_yolo_detector/SubmitTaskAssignment.h>
#include <sar_yolo_detector/TaskAssignment.h>
#include <sar_yolo_detector/TaskExecutionStatus.h>
#include <sar_yolo_detector/UavDecisionHeartbeat.h>

#include <sar_yolo_detector/FloodRegionArray.h>
#include <sar_yolo_detector/TaskCandidateArray.h>
#include <sar_yolo_detector/mission_state_store.hpp>

namespace sar_yolo_detector {
namespace {

constexpr std::uint16_t kProtocolMajor = 1;
constexpr std::uint16_t kProtocolMinor = 0;
constexpr const char *kInterfaceVersion = "sar_yolo_detector/1.0";

bool finite(const double value) { return std::isfinite(value); }

template <typename T>
bool contains(const std::vector<T> &values, const T &value) {
  return std::find(values.begin(), values.end(), value) != values.end();
}

std::string trim(std::string value) {
  const auto first = value.find_first_not_of(" \t\r\n");
  const auto last = value.find_last_not_of(" \t\r\n");
  if (first == std::string::npos) return std::string();
  return value.substr(first, last - first + 1U);
}

std::string sanitizeFileComponent(std::string value) {
  for (char &character : value) {
    const bool safe = (character >= 'a' && character <= 'z') ||
                      (character >= 'A' && character <= 'Z') ||
                      (character >= '0' && character <= '9') ||
                      character == '-' || character == '_';
    if (!safe) character = '_';
  }
  return value;
}

std::string defaultDatabasePath(const std::string &mission_id,
                                const std::string &uav_id) {
  const char *ros_home = std::getenv("ROS_HOME");
  std::string directory;
  if (ros_home != nullptr && *ros_home != '\0') {
    directory = ros_home;
  } else {
    const char *user_home = std::getenv("HOME");
    directory = user_home == nullptr ? "/tmp" : std::string(user_home) + "/.ros";
  }
  ::mkdir(directory.c_str(), 0700);
  return directory + "/sar_mission_" + sanitizeFileComponent(mission_id) + "_" +
         sanitizeFileComponent(uav_id) + ".sqlite3";
}

bool validPose(const geometry_msgs::PoseStamped &pose,
               const std::string &required_frame) {
  const auto &p = pose.pose.position;
  const auto &q = pose.pose.orientation;
  if (pose.header.frame_id != required_frame || !finite(p.x) || !finite(p.y) ||
      !finite(p.z) || !finite(q.x) || !finite(q.y) || !finite(q.z) ||
      !finite(q.w)) return false;
  const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  return norm > 0.99 && norm < 1.01;
}

bool validPolygon(const geometry_msgs::PolygonStamped &polygon,
                  const std::string &required_frame, double *area) {
  if (polygon.header.frame_id != required_frame ||
      polygon.polygon.points.size() < 3U) return false;
  double twice_area = 0.0;
  for (std::size_t i = 0; i < polygon.polygon.points.size(); ++i) {
    const auto &left = polygon.polygon.points[i];
    const auto &right = polygon.polygon.points[(i + 1U) % polygon.polygon.points.size()];
    if (!finite(left.x) || !finite(left.y) || !finite(left.z)) return false;
    twice_area += static_cast<double>(left.x) * right.y -
                  static_cast<double>(right.x) * left.y;
  }
  *area = std::abs(twice_area) * 0.5;
  return finite(*area) && *area > 0.01;
}

bool sameGeoReference(const sar_yolo_detector::GeoReference &left,
                      const sar_yolo_detector::GeoReference &right) {
  return left.coordinate_type == right.coordinate_type &&
         left.epsg == right.epsg && left.frame_id == right.frame_id &&
         left.map_uuid == right.map_uuid &&
         std::abs(left.origin_latitude_deg - right.origin_latitude_deg) < 1.0e-9 &&
         std::abs(left.origin_longitude_deg - right.origin_longitude_deg) < 1.0e-9 &&
         std::abs(left.origin_altitude_m - right.origin_altitude_m) < 1.0e-4 &&
         left.vertical_datum == right.vertical_datum &&
         left.geoid_model == right.geoid_model &&
         left.transform_version == right.transform_version &&
         left.valid_from == right.valid_from && left.valid_until == right.valid_until;
}

std::string hmacSha256Hex(const std::string &key,
                          const std::vector<std::uint8_t> &bytes) {
  unsigned char digest[EVP_MAX_MD_SIZE];
  unsigned int length = 0U;
  HMAC(EVP_sha256(), key.data(), static_cast<int>(key.size()), bytes.data(),
       bytes.size(), digest, &length);
  std::ostringstream stream;
  stream << std::hex << std::setfill('0');
  for (unsigned int index = 0U; index < length; ++index)
    stream << std::setw(2) << static_cast<unsigned int>(digest[index]);
  return stream.str();
}

bool constantTimeEqual(const std::string &left, const std::string &right) {
  return left.size() == right.size() &&
         CRYPTO_memcmp(left.data(), right.data(), left.size()) == 0;
}

std::string sourceKey(const PerceptionIdentity &source, const std::string &kind) {
  return kind + "\n" + source.mission_id + "\n" + source.uav_id + "\n" +
         source.session_uuid + "\n" + source.sensor_id + "\n" + source.profile;
}

sensor_msgs::RegionOfInterest detectionRoi(const vision_msgs::Detection2D &detection) {
  sensor_msgs::RegionOfInterest roi;
  roi.x_offset = static_cast<std::uint32_t>(std::max(
      0.0, std::floor(detection.bbox.center.x - 0.5 * detection.bbox.size_x)));
  roi.y_offset = static_cast<std::uint32_t>(std::max(
      0.0, std::floor(detection.bbox.center.y - 0.5 * detection.bbox.size_y)));
  roi.width = static_cast<std::uint32_t>(std::max(0.0, std::ceil(detection.bbox.size_x)));
  roi.height = static_cast<std::uint32_t>(std::max(0.0, std::ceil(detection.bbox.size_y)));
  return roi;
}

struct ValidationError {
  std::uint16_t code{sar_yolo_detector::TaskExecutionStatus::INTERNAL_ERROR};
  std::string reason;
};

}  // namespace

class DecisionBridge {
 public:
  DecisionBridge() : private_node_("~") {
    loadParameters();
    validateParameters();
    if (assignment_downlink_enabled_) loadAuthorizationKeys();
    store_.reset(new MissionStateStore(state_database_path_));
    restorePersistentState();
    if (!assignment_downlink_enabled_) {
      recovering_ = false;
      recovery_reason_.clear();
    }
    configureRos();
    publishHeartbeat();
    ROS_INFO_STREAM("SAR decision bridge v1 started: role=" << bridge_role_
                    << " mission="
                    << mission_id_ << " uav=" << uav_id_ << " session="
                    << session_uuid_ << " database=" << state_database_path_);
  }

 private:
  struct AssignmentRecord {
    std::uint64_t sequence{0};
    bool active{false};
    bool is_area{false};
    std::uint8_t state{sar_yolo_detector::TaskExecutionStatus::DISPATCHED_TO_EXECUTOR};
    std::uint64_t executor_status_sequence{0};
    ros::Time dispatch_deadline;
    std::string decision_key;
    sar_yolo_detector::TaskAssignment point;
    sar_yolo_detector::AreaTaskAssignment area;
  };

  void loadParameters() {
    private_node_.param("mission_id", mission_id_, std::string("mission_unset"));
    private_node_.param("uav_id", uav_id_, std::string("uav_01"));
    private_node_.param("session_uuid", session_uuid_, std::string());
    private_node_.param("bridge_role", bridge_role_, std::string("combined"));
    if (bridge_role_ == "scout") {
      perception_uplink_enabled_ = true;
      assignment_downlink_enabled_ = false;
    } else if (bridge_role_ == "worker") {
      perception_uplink_enabled_ = false;
      assignment_downlink_enabled_ = true;
    } else if (bridge_role_ == "combined") {
      perception_uplink_enabled_ = true;
      assignment_downlink_enabled_ = true;
    } else {
      throw std::invalid_argument(
          "~bridge_role must be scout, worker, or combined");
    }
    private_node_.param("coordinate_contract", coordinate_contract_, std::string("LOCAL_ENU:map"));
    private_node_.param("required_target_frame", required_target_frame_, std::string("map"));
    private_node_.param("geo_reference_validated", geo_reference_validated_, false);
    std::string coordinate_type;
    private_node_.param("geo_coordinate_type", coordinate_type, std::string("LOCAL_ENU"));
    geo_reference_.coordinate_type = coordinate_type == "ECEF"
        ? sar_yolo_detector::GeoReference::ECEF
        : (coordinate_type == "WGS84" ? sar_yolo_detector::GeoReference::WGS84
                                       : sar_yolo_detector::GeoReference::LOCAL_ENU);
    private_node_.param("geo_epsg", geo_reference_.epsg, std::string());
    geo_reference_.frame_id = required_target_frame_;
    private_node_.param("geo_map_uuid", geo_reference_.map_uuid, std::string("map_unset"));
    private_node_.param("geo_origin_latitude_deg", geo_reference_.origin_latitude_deg,
                        std::numeric_limits<double>::quiet_NaN());
    private_node_.param("geo_origin_longitude_deg", geo_reference_.origin_longitude_deg,
                        std::numeric_limits<double>::quiet_NaN());
    private_node_.param("geo_origin_altitude_m", geo_reference_.origin_altitude_m,
                        std::numeric_limits<double>::quiet_NaN());
    private_node_.param("geo_vertical_datum", geo_reference_.vertical_datum,
                        std::string("WGS84_ELLIPSOID"));
    private_node_.param("geo_geoid_model", geo_reference_.geoid_model, std::string());
    private_node_.param("geo_transform_version", geo_reference_.transform_version,
                        std::string("unversioned"));
    double geo_valid_from_sec = 0.0, geo_valid_until_sec = 0.0;
    private_node_.param("geo_valid_from_sec", geo_valid_from_sec, 0.0);
    private_node_.param("geo_valid_until_sec", geo_valid_until_sec, 0.0);
    if (geo_valid_from_sec > 0.0) geo_reference_.valid_from.fromSec(geo_valid_from_sec);
    if (geo_valid_until_sec > 0.0) geo_reference_.valid_until.fromSec(geo_valid_until_sec);

    private_node_.param("source_task_candidates_topic", source_task_candidates_topic_, std::string());
    private_node_.param("source_flood_regions_topic", source_flood_regions_topic_, std::string());
    private_node_.param("outbound_candidates_topic", outbound_candidates_topic_,
                        std::string("mission_interface/perception_candidates"));
    private_node_.param("inbound_assignment_topic", inbound_assignment_topic_,
                        std::string("mission_interface/task_assignments"));
    private_node_.param("inbound_area_assignment_topic", inbound_area_assignment_topic_,
                        std::string("mission_interface/area_task_assignments"));
    private_node_.param("outbound_status_topic", outbound_status_topic_,
                        std::string("mission_interface/task_status"));
    private_node_.param("heartbeat_topic", heartbeat_topic_,
                        std::string("mission_interface/heartbeat"));
    private_node_.param("executor_assignment_topic", executor_assignment_topic_,
                        std::string("mission_interface/executor_assignments"));
    private_node_.param("executor_area_assignment_topic", executor_area_assignment_topic_,
                        std::string("mission_interface/executor_area_assignments"));
    private_node_.param("executor_status_topic", executor_status_topic_,
                        std::string("mission_interface/executor_status"));
    private_node_.param("executor_heartbeat_topic", executor_heartbeat_topic_,
                        std::string("mission_interface/executor_heartbeat"));
    private_node_.param("submit_assignment_service", submit_assignment_service_,
                        std::string("mission_interface/submit_task"));
    private_node_.param("submit_area_assignment_service", submit_area_assignment_service_,
                        std::string("mission_interface/submit_area_task"));
    private_node_.param("snapshot_service", snapshot_service_name_,
                        std::string("mission_interface/get_perception_snapshot"));
    private_node_.param("statuses_service", statuses_service_name_,
                        std::string("mission_interface/get_task_statuses"));
    private_node_.param("evidence_service", evidence_service_name_,
                        std::string("mission_interface/get_evidence"));
    private_node_.param("source_evidence_service", source_evidence_service_, std::string());
    if (source_evidence_service_.empty() && !source_flood_regions_topic_.empty())
      source_evidence_service_ = source_flood_regions_topic_ + "/get_evidence";

    private_node_.param("allow_topic_commands", allow_topic_commands_, false);
    private_node_.param("require_message_authentication", require_message_authentication_, true);
    private_node_.param("require_operator_authorization", require_operator_authorization_, true);
    private_node_.param("require_known_observation", require_known_observation_, true);
    private_node_.param("require_geofence_id", require_geofence_id_, true);
    private_node_.param("require_geofence_version", require_geofence_version_, true);
    private_node_.param("maximum_assignment_age_sec", maximum_assignment_age_sec_, 5.0);
    private_node_.param("maximum_authorization_age_sec", maximum_authorization_age_sec_, 10.0);
    private_node_.param("maximum_future_skew_sec", maximum_future_skew_sec_, 0.5);
    private_node_.param("maximum_candidate_age_sec", maximum_candidate_age_sec_, 5.0);
    private_node_.param("executor_heartbeat_timeout_sec", executor_heartbeat_timeout_sec_, 2.5);
    private_node_.param("executor_ack_timeout_sec", executor_ack_timeout_sec_, 2.0);
    private_node_.param("maximum_task_duration_sec", maximum_task_duration_sec_, 86400.0);
    private_node_.param("maximum_area_m2", maximum_area_m2_, 1.0e8);
    private_node_.param("minimum_executor_battery_percent",
                        minimum_executor_battery_percent_, 20.0);
    private_node_.param("heartbeat_rate_hz", heartbeat_rate_hz_, 1.0);
    private_node_.param("evidence_source_topic", evidence_source_topic_,
                        std::string("camera/image_raw"));
    private_node_.param("state_database_path", state_database_path_, std::string());
    if (state_database_path_.empty()) state_database_path_ = defaultDatabasePath(mission_id_, uav_id_);

    XmlRpc::XmlRpcValue allowed;
    if (private_node_.getParam("allowed_decision_ids", allowed)) {
      if (allowed.getType() != XmlRpc::XmlRpcValue::TypeArray)
        throw std::invalid_argument("~allowed_decision_ids must be an array");
      for (int i = 0; i < allowed.size(); ++i) {
        if (allowed[i].getType() != XmlRpc::XmlRpcValue::TypeString)
          throw std::invalid_argument("~allowed_decision_ids entries must be strings");
        allowed_decision_ids_.insert(static_cast<std::string>(allowed[i]));
      }
    }
  }

  void validateParameters() const {
    if (mission_id_.empty() || mission_id_ == "mission_unset" || uav_id_.empty() ||
        session_uuid_.empty())
      throw std::invalid_argument("mission_id/uav_id/session_uuid must be configured");
    if (required_target_frame_.empty() || coordinate_contract_.empty())
      throw std::invalid_argument("target frame and coordinate contract are required");
    if (bridge_role_ == "scout" && source_task_candidates_topic_.empty() &&
        source_flood_regions_topic_.empty())
      throw std::invalid_argument(
          "scout bridge requires a task-candidate or flood-region source");
    if (heartbeat_rate_hz_ <= 0.0 || executor_ack_timeout_sec_ <= 0.0 ||
        executor_heartbeat_timeout_sec_ <= 0.0 ||
        minimum_executor_battery_percent_ < 0.0 ||
        minimum_executor_battery_percent_ > 100.0)
      throw std::invalid_argument("bridge timing parameters must be positive");
  }

  void loadAuthorizationKeys() {
    XmlRpc::XmlRpcValue values;
    if (!private_node_.getParam("authorization_key_files", values)) return;
    if (values.getType() != XmlRpc::XmlRpcValue::TypeStruct)
      throw std::invalid_argument("~authorization_key_files must be a dictionary");
    for (auto iterator = values.begin(); iterator != values.end(); ++iterator) {
      if (iterator->second.getType() != XmlRpc::XmlRpcValue::TypeString)
        throw std::invalid_argument("authorization key paths must be strings");
      const std::string path = static_cast<std::string>(iterator->second);
      struct stat metadata {};
      if (::lstat(path.c_str(), &metadata) != 0)
        throw std::runtime_error("cannot stat authorization key file: " + path);
      if (!S_ISREG(metadata.st_mode) || metadata.st_uid != ::geteuid() ||
          (metadata.st_mode & (S_IRWXG | S_IRWXO)) != 0)
        throw std::runtime_error(
            "authorization key must be an owner-only regular non-symlink file: " + path);
      std::ifstream stream(path, std::ios::binary);
      std::stringstream buffer;
      buffer << stream.rdbuf();
      const std::string key = trim(buffer.str());
      if ((!stream.good() && !stream.eof()) || key.size() < 32U)
        throw std::runtime_error("authorization key cannot be read or is shorter than 32 bytes");
      authorization_keys_[iterator->first] = key;
    }
    XmlRpc::XmlRpcValue bindings;
    if (private_node_.getParam("authorization_key_decision_ids", bindings)) {
      if (bindings.getType() != XmlRpc::XmlRpcValue::TypeStruct)
        throw std::invalid_argument(
            "~authorization_key_decision_ids must be a dictionary");
      for (auto iterator = bindings.begin(); iterator != bindings.end(); ++iterator) {
        if (iterator->second.getType() != XmlRpc::XmlRpcValue::TypeString)
          throw std::invalid_argument("authorization key bindings must be strings");
        authorization_key_decision_ids_[iterator->first] =
            static_cast<std::string>(iterator->second);
      }
    }
    for (const auto &entry : authorization_keys_) {
      const auto binding = authorization_key_decision_ids_.find(entry.first);
      if (binding == authorization_key_decision_ids_.end() ||
          binding->second.empty() ||
          (!allowed_decision_ids_.empty() &&
           allowed_decision_ids_.count(binding->second) == 0U))
        throw std::invalid_argument(
            "every authorization key must bind to one permitted decision_id");
    }
  }

  void configureRos() {
    heartbeat_publisher_ = node_.advertise<sar_yolo_detector::UavDecisionHeartbeat>(
        heartbeat_topic_, 1, true);
    if (perception_uplink_enabled_) {
      candidates_publisher_ =
          node_.advertise<sar_yolo_detector::PerceptionCandidateArray>(
              outbound_candidates_topic_, 10, false);
      if (!source_task_candidates_topic_.empty())
        task_candidates_subscriber_ = node_.subscribe(
            source_task_candidates_topic_, 10,
            &DecisionBridge::taskCandidatesCallback, this);
      if (!source_flood_regions_topic_.empty())
        flood_regions_subscriber_ = node_.subscribe(
            source_flood_regions_topic_, 10,
            &DecisionBridge::floodRegionsCallback, this);
      snapshot_service_ = node_.advertiseService(
          snapshot_service_name_, &DecisionBridge::snapshotCallback, this);
      evidence_service_ = node_.advertiseService(
          evidence_service_name_, &DecisionBridge::evidenceCallback, this);
      if (!source_evidence_service_.empty()) {
        source_evidence_client_ =
            node_.serviceClient<sar_yolo_detector::GetEvidenceCrop>(
                source_evidence_service_, true);
        evidence_retrieval_base_uri_ =
            "ros-service://" + node_.resolveName(evidence_service_name_);
      }
    }
    if (assignment_downlink_enabled_) {
      status_publisher_ =
          node_.advertise<sar_yolo_detector::TaskExecutionStatus>(
              outbound_status_topic_, 20, false);
      executor_assignment_publisher_ =
          node_.advertise<sar_yolo_detector::TaskAssignment>(
              executor_assignment_topic_, 10, false);
      executor_area_assignment_publisher_ =
          node_.advertise<sar_yolo_detector::AreaTaskAssignment>(
              executor_area_assignment_topic_, 10, false);
      if (allow_topic_commands_) {
        assignment_subscriber_ = node_.subscribe(
            inbound_assignment_topic_, 20,
            &DecisionBridge::assignmentCallback, this);
        area_assignment_subscriber_ = node_.subscribe(
            inbound_area_assignment_topic_, 20,
            &DecisionBridge::areaAssignmentCallback, this);
      }
      executor_status_subscriber_ = node_.subscribe(
          executor_status_topic_, 20,
          &DecisionBridge::executorStatusCallback, this);
      executor_heartbeat_subscriber_ = node_.subscribe(
          executor_heartbeat_topic_, 5,
          &DecisionBridge::executorHeartbeatCallback, this);
      submit_service_ = node_.advertiseService(
          submit_assignment_service_,
          &DecisionBridge::submitAssignmentCallback, this);
      submit_area_service_ = node_.advertiseService(
          submit_area_assignment_service_,
          &DecisionBridge::submitAreaAssignmentCallback, this);
      statuses_service_ = node_.advertiseService(
          statuses_service_name_, &DecisionBridge::statusesCallback, this);
    }
    heartbeat_timer_ = node_.createTimer(ros::Duration(1.0 / heartbeat_rate_hz_),
        &DecisionBridge::heartbeatTimerCallback, this);
    if (assignment_downlink_enabled_)
      maintenance_timer_ = node_.createTimer(
          ros::Duration(0.1), &DecisionBridge::maintenanceCallback, this);
  }

  void restorePersistentState() {
    const std::string stored_mission = store_->getMeta("mission_id", mission_id_);
    const std::string stored_uav = store_->getMeta("uav_id", uav_id_);
    if (stored_mission != mission_id_ || stored_uav != uav_id_)
      throw std::runtime_error("state database belongs to another mission/UAV");
    store_->setMeta("mission_id", mission_id_);
    store_->setMeta("uav_id", uav_id_);
    status_sequence_ = std::stoull(store_->getMeta("last_status_sequence", "0"));
    candidate_sequence_ = std::stoull(store_->getMeta("last_candidate_sequence", "0"));
    decision_sequences_ = store_->loadDecisionSequences();
    source_array_sequences_ = store_->loadCandidateSourceSequences();
    for (const auto &saved : store_->loadAssignments()) {
      AssignmentRecord record;
      record.sequence = saved.sequence;
      record.active = saved.active;
      record.is_area = saved.is_area;
      record.state = saved.state;
      record.executor_status_sequence = saved.executor_status_sequence;
      record.dispatch_deadline.fromNSec(static_cast<std::uint64_t>(
          std::max<std::int64_t>(0, saved.dispatch_deadline_ns)));
      record.decision_key = saved.decision_key;
      record.point = saved.point_assignment;
      record.area = saved.area_assignment;
      assignments_[saved.assignment_uuid] = std::move(record);
      last_assignment_sequence_ = std::max(last_assignment_sequence_, saved.sequence);
    }
    const ros::Time now = ros::Time::now();
    std::unordered_set<std::string> obsolete_candidate_sources;
    std::vector<std::pair<std::string, std::string>> obsolete_candidates;
    for (const auto &saved : store_->loadCandidates()) {
      const auto &candidate = saved.candidate;
      if (candidate.source.mission_id != mission_id_ ||
          candidate.source.uav_id != uav_id_ ||
          candidate.source.session_uuid != session_uuid_) {
        obsolete_candidate_sources.insert(saved.source_key);
        continue;
      }
      if (candidate.last_seen.isZero() ||
          (!now.isZero() &&
           (now - candidate.last_seen).toSec() > maximum_candidate_age_sec_)) {
        obsolete_candidates.emplace_back(saved.source_key,
                                           candidate.observation_uuid);
        continue;
      }
      candidate_cache_[candidate.observation_uuid] = candidate;
      candidate_sources_[candidate.observation_uuid] = saved.source_key;
      candidate_event_sequences_[saved.source_key + "\n" +
                                 candidate.observation_uuid] =
          candidate.event_sequence;
      perception_available_ = true;
    }
    if (!obsolete_candidate_sources.empty() || !obsolete_candidates.empty()) {
      store_->begin();
      try {
        for (const auto &source : obsolete_candidate_sources) {
          store_->eraseCandidateSource(source);
          source_array_sequences_.erase(source);
        }
        for (const auto &candidate : obsolete_candidates)
          if (obsolete_candidate_sources.count(candidate.first) == 0U)
            store_->eraseCandidate(candidate.first, candidate.second);
        store_->commit();
      } catch (...) {
        store_->rollback();
        throw;
      }
    }
    recovering_ = true;
  }

  sar_yolo_detector::MissionIdentity identity() const {
    sar_yolo_detector::MissionIdentity result;
    result.mission_id = mission_id_;
    result.uav_id = uav_id_;
    result.session_uuid = session_uuid_;
    return result;
  }

  bool sourceIdentityMatches(const PerceptionIdentity &source) const {
    return source.mission_id == mission_id_ && source.uav_id == uav_id_ &&
           source.session_uuid == session_uuid_;
  }

  bool geoReferenceOperational() const {
    if (!geo_reference_validated_ || geo_reference_.map_uuid.empty() ||
        geo_reference_.map_uuid == "map_unset" ||
        geo_reference_.transform_version.empty() ||
        geo_reference_.transform_version == "unversioned") return false;
    const ros::Time now = ros::Time::now();
    if ((!geo_reference_.valid_from.isZero() && now < geo_reference_.valid_from) ||
        (!geo_reference_.valid_until.isZero() && now >= geo_reference_.valid_until))
      return false;
    if (geo_reference_.coordinate_type == sar_yolo_detector::GeoReference::LOCAL_ENU)
      return finite(geo_reference_.origin_latitude_deg) &&
             finite(geo_reference_.origin_longitude_deg) &&
             finite(geo_reference_.origin_altitude_m) &&
             std::abs(geo_reference_.origin_latitude_deg) <= 90.0 &&
             std::abs(geo_reference_.origin_longitude_deg) <= 180.0;
    return !geo_reference_.epsg.empty();
  }

  sar_yolo_detector::PerceptionCandidate convert(const TaskCandidate &source) const {
    sar_yolo_detector::PerceptionCandidate output;
    output.header = source.header;
    output.observation_uuid = source.observation_uuid;
    output.event_sequence = source.event_sequence;
    output.source = identity();
    output.candidate_shape = sar_yolo_detector::PerceptionCandidate::POINT;
    output.local_track_id = source.track_id;
    output.source_track_id_valid = source.source_track_id_valid;
    output.source_track_id = source.source_track_id;
    output.raw_class_id = source.class_id;
    output.raw_class_name = source.class_name;
    output.semantic_type = source.semantic_type;
    output.lifecycle_state = source.status;
    output.confidence = source.confidence;
    output.stability = source.stability;
    output.priority = source.priority;
    output.observation_count = source.observation_count;
    output.first_seen = source.first_seen;
    output.last_seen = source.last_seen;
    output.localization_valid = source.localization_valid && geoReferenceOperational();
    output.target_pose = source.task_pose;
    output.bearing = source.bearing;
    output.position_covariance = source.position_covariance;
    output.image_region_valid = source.image_detection.bbox.size_x > 0.0 &&
                                source.image_detection.bbox.size_y > 0.0;
    if (output.image_region_valid) output.image_roi = detectionRoi(source.image_detection);
    output.sensor_id = source.sensor_id;
    output.profile = source.provenance.profile;
    output.model_version = source.provenance.model_version;
    output.model_sha256 = source.provenance.model_sha256;
    output.calibration_version = source.provenance.calibration_version;
    output.coordinate_contract = source.provenance.coordinate_contract;
    if (output.localization_valid) output.geo_reference = geo_reference_;
    output.evidence.evidence_uuid = source.observation_uuid + "@" +
                                    std::to_string(source.header.stamp.toNSec());
    output.evidence.source_topic = evidence_source_topic_;
    output.evidence.capture_stamp = source.header.stamp;
    output.evidence.roi = output.image_roi;
    return output;
  }

  sar_yolo_detector::PerceptionCandidate convert(const FloodRegion &source) const {
    sar_yolo_detector::PerceptionCandidate output;
    output.header = source.header;
    output.observation_uuid = source.observation_uuid;
    output.event_sequence = source.event_sequence;
    output.source = identity();
    output.candidate_shape = sar_yolo_detector::PerceptionCandidate::IMAGE_REGION;
    output.local_track_id = source.track_id;
    output.raw_class_id = source.class_id;
    output.raw_class_name = source.class_name;
    output.semantic_type = source.semantic_type;
    output.lifecycle_state = source.status;
    output.confidence = source.confidence;
    output.stability = source.stability;
    output.priority = source.priority;
    output.observation_count = source.observation_count;
    output.first_seen = source.first_seen;
    output.last_seen = source.last_seen;
    output.localization_valid = source.localization_valid && geoReferenceOperational();
    output.target_pose = source.map_centroid;
    output.position_covariance = source.position_covariance;
    output.image_region_valid = true;
    output.image_roi = source.image_roi;
    output.image_contour = source.image_contour;
    output.world_polygon = source.world_polygon;
    output.world_area_m2 = source.world_area_m2;
    output.world_projection_coverage = source.world_projection_coverage;
    output.sensor_id = source.sensor_id;
    output.profile = source.provenance.profile;
    output.model_version = source.provenance.model_version;
    output.model_sha256 = source.provenance.model_sha256;
    output.calibration_version = source.provenance.calibration_version;
    output.coordinate_contract = source.provenance.coordinate_contract;
    if (output.localization_valid) output.geo_reference = geo_reference_;
    output.evidence.evidence_uuid = source.evidence_uuid;
    output.evidence.source_topic = source.evidence_source_topic;
    output.evidence.capture_stamp = source.header.stamp;
    output.evidence.image_sha256 = source.evidence_image_sha256;
    output.evidence.retrieval_uri = evidence_retrieval_base_uri_.empty()
        ? source.evidence_retrieval_uri
        : evidence_retrieval_base_uri_ + "?evidence_uuid=" + source.evidence_uuid;
    output.evidence.roi = source.image_roi;
    output.evidence.contour_vertex_count = source.image_contour.polygon.points.size();
    return output;
  }

  void markEvidenceStale(const std::string &observation_uuid) {
    for (auto &entry : assignments_) {
      AssignmentRecord &record = entry.second;
      const std::string linked = record.is_area ? record.area.observation_uuid
                                                : record.point.observation_uuid;
      if (!record.active || linked != observation_uuid) continue;
      auto status = makeStatus(record,
          sar_yolo_detector::TaskExecutionStatus::EVIDENCE_STALE,
          sar_yolo_detector::TaskExecutionStatus::EVIDENCE_BECAME_STALE,
          "linked perception evidence expired; executor policy decides whether to continue");
      persistAndPublishStatus(status);
    }
  }

  void publishCandidates(const std::string &source_key, const std_msgs::Header &header,
      const std::uint64_t source_array_sequence, const bool full_snapshot,
      std::vector<sar_yolo_detector::PerceptionCandidate> candidates) {
    auto last_array = source_array_sequences_.find(source_key);
    if (last_array != source_array_sequences_.end() &&
        source_array_sequence <= last_array->second) {
      ROS_WARN_STREAM_THROTTLE(2.0, "Dropping out-of-order candidate array from " << source_key);
      return;
    }
    std::vector<sar_yolo_detector::PerceptionCandidate> accepted;
    std::vector<sar_yolo_detector::PerceptionCandidate> persistent_snapshot;
    std::unordered_set<std::string> snapshot_observations;
    for (auto &candidate : candidates) {
      if (candidate.observation_uuid.empty() || candidate.event_sequence == 0U ||
          !finite(candidate.confidence) || !finite(candidate.stability) ||
          !finite(candidate.priority)) continue;
      if (full_snapshot) snapshot_observations.insert(candidate.observation_uuid);
      const std::string event_key = source_key + "\n" + candidate.observation_uuid;
      const auto previous = candidate_event_sequences_.find(event_key);
      if (previous != candidate_event_sequences_.end() &&
          candidate.event_sequence < previous->second) {
        if (full_snapshot) {
          const auto current = candidate_cache_.find(candidate.observation_uuid);
          if (current != candidate_cache_.end() &&
              candidate_sources_[candidate.observation_uuid] == source_key)
            persistent_snapshot.push_back(current->second);
        }
        continue;
      }
      if (previous != candidate_event_sequences_.end() &&
          candidate.event_sequence == previous->second && !full_snapshot) continue;
      candidate_event_sequences_[event_key] = candidate.event_sequence;
      accepted.push_back(candidate);
      if (candidate.lifecycle_state !=
          sar_yolo_detector::PerceptionCandidate::EXPIRED)
        persistent_snapshot.push_back(candidate);
    }
    store_->begin();
    try {
      if (full_snapshot)
        store_->replaceCandidateSource(source_key, persistent_snapshot);
      for (const auto &candidate : accepted) {
        if (candidate.lifecycle_state == sar_yolo_detector::PerceptionCandidate::EXPIRED)
          store_->eraseCandidate(source_key, candidate.observation_uuid);
        else if (!full_snapshot) store_->saveCandidate(source_key, candidate);
      }
      store_->saveCandidateSourceSequence(source_key, source_array_sequence);
      store_->setMeta("last_candidate_sequence", std::to_string(candidate_sequence_ + 1U));
      store_->commit();
    } catch (...) {
      store_->rollback();
      throw;
    }
    source_array_sequences_[source_key] = source_array_sequence;
    if (full_snapshot) {
      std::vector<std::string> missing;
      for (const auto &entry : candidate_sources_)
        if (entry.second == source_key && snapshot_observations.count(entry.first) == 0U)
          missing.push_back(entry.first);
      for (const auto &observation : missing) {
        candidate_cache_.erase(observation);
        candidate_sources_.erase(observation);
        markEvidenceStale(observation);
      }
    }
    for (const auto &candidate : accepted) {
      if (candidate.lifecycle_state == sar_yolo_detector::PerceptionCandidate::EXPIRED) {
        candidate_cache_.erase(candidate.observation_uuid);
        candidate_sources_.erase(candidate.observation_uuid);
        expired_observations_.insert(candidate.observation_uuid);
        markEvidenceStale(candidate.observation_uuid);
      } else {
        candidate_cache_[candidate.observation_uuid] = candidate;
        candidate_sources_[candidate.observation_uuid] = source_key;
      }
    }
    sar_yolo_detector::PerceptionCandidateArray output;
    output.header = header;
    output.array_sequence = ++candidate_sequence_;
    output.full_snapshot = full_snapshot;
    output.source = identity();
    if (full_snapshot) {
      output.candidates.reserve(candidate_cache_.size());
      for (const auto &entry : candidate_cache_)
        output.candidates.push_back(entry.second);
      std::sort(output.candidates.begin(), output.candidates.end(),
          [](const sar_yolo_detector::PerceptionCandidate &left,
             const sar_yolo_detector::PerceptionCandidate &right) {
            return left.observation_uuid < right.observation_uuid;
          });
    } else {
      output.candidates = std::move(accepted);
    }
    candidates_publisher_.publish(output);
    perception_available_ = !candidate_cache_.empty();
    last_candidate_at_ = ros::Time::now();
  }

  void taskCandidatesCallback(const TaskCandidateArray::ConstPtr &message) {
    if (!sourceIdentityMatches(message->provenance)) return;
    std::vector<sar_yolo_detector::PerceptionCandidate> converted;
    for (const auto &candidate : message->candidates)
      if (sourceIdentityMatches(candidate.provenance)) converted.push_back(convert(candidate));
    publishCandidates(sourceKey(message->provenance, "point"), message->header,
                      message->array_sequence, message->full_snapshot, std::move(converted));
  }

  void floodRegionsCallback(const FloodRegionArray::ConstPtr &message) {
    if (!sourceIdentityMatches(message->provenance)) return;
    std::vector<sar_yolo_detector::PerceptionCandidate> converted;
    for (const auto &region : message->regions)
      if (sourceIdentityMatches(region.provenance)) converted.push_back(convert(region));
    publishCandidates(sourceKey(message->provenance, "area"), message->header,
                      message->array_sequence, message->full_snapshot, std::move(converted));
  }

  template <typename Assignment>
  std::string replayKey(const Assignment &assignment) const {
    return assignment.assignment_uuid + "\n" + std::to_string(assignment.assignment_sequence);
  }

  template <typename Assignment>
  std::string decisionKey(const Assignment &assignment) const {
    return assignment.destination.mission_id + "\n" + assignment.decision_id + "\n" +
           assignment.decision_session_uuid;
  }

  template <typename Assignment>
  std::string assignmentDigest(const Assignment &assignment) const {
    return MissionStateStore::sha256Hex(MissionStateStore::serialize(assignment));
  }

  template <typename Assignment>
  bool authenticationValid(const Assignment &assignment, ValidationError *error) const {
    if (!require_message_authentication_) return true;
    const auto key = authorization_keys_.find(assignment.authorization_key_id);
    const auto binding = authorization_key_decision_ids_.find(
        assignment.authorization_key_id);
    if (key == authorization_keys_.end() || assignment.authorization_nonce.empty() ||
        assignment.authorization_signature.size() != 64U ||
        binding == authorization_key_decision_ids_.end() ||
        binding->second != assignment.decision_id) {
      error->code = sar_yolo_detector::TaskExecutionStatus::INVALID_AUTHENTICATION;
      error->reason = "missing or unknown authorization key/nonce/signature";
      return false;
    }
    if (assignment.authorization_issued_at.isZero() ||
        std::abs((ros::Time::now() - assignment.authorization_issued_at).toSec()) >
            maximum_authorization_age_sec_) {
      error->code = sar_yolo_detector::TaskExecutionStatus::INVALID_AUTHENTICATION;
      error->reason = "authorization timestamp is outside the permitted window";
      return false;
    }
    Assignment unsigned_assignment = assignment;
    const std::string supplied = unsigned_assignment.authorization_signature;
    unsigned_assignment.authorization_signature.clear();
    const std::string expected = hmacSha256Hex(
        key->second, MissionStateStore::serialize(unsigned_assignment));
    if (!constantTimeEqual(expected, supplied) ||
        store_->nonceExists(assignment.authorization_key_id, assignment.authorization_nonce)) {
      error->code = sar_yolo_detector::TaskExecutionStatus::INVALID_AUTHENTICATION;
      error->reason = "assignment HMAC failed or nonce was already consumed";
      return false;
    }
    return true;
  }

  bool executorHeartbeatFresh() const {
    return have_executor_heartbeat_ &&
        (ros::Time::now() - last_executor_heartbeat_at_).toSec() <= executor_heartbeat_timeout_sec_;
  }

  bool bridgeReady(std::string *reason) const {
    if (!assignment_downlink_enabled_) {
      *reason = "scout_uplink_only";
      return false;
    }
    if (recovering_) {
      *reason = recovery_reason_.empty() ? "waiting for executor state reconciliation"
                                         : recovery_reason_;
      return false;
    }
    if (!geoReferenceOperational()) {
      *reason = "structured geographic reference has not been validated";
      return false;
    }
    if (require_message_authentication_ && authorization_keys_.empty()) {
      *reason = "no message authentication keys are configured";
      return false;
    }
    if (!executorHeartbeatFresh()) {
      *reason = "executor heartbeat is missing or stale";
      return false;
    }
    if (!executor_heartbeat_.ready || !executor_heartbeat_.flight_control_ok ||
        !executor_heartbeat_.localization_ok) {
      *reason = executor_heartbeat_.fault_reason.empty()
                    ? "executor/flight control/localization is not ready"
                    : executor_heartbeat_.fault_reason;
      return false;
    }
    if (!finite(executor_heartbeat_.battery_percent) ||
        executor_heartbeat_.battery_percent < minimum_executor_battery_percent_) {
      *reason = "executor battery is below the configured task threshold";
      return false;
    }
    reason->clear();
    return true;
  }

  template <typename Assignment>
  bool validateCommon(const Assignment &assignment, const bool is_area,
                      AssignmentRecord **existing, ValidationError *error) {
    if (assignment.protocol_major != kProtocolMajor || assignment.protocol_minor > kProtocolMinor) {
      error->code = sar_yolo_detector::TaskExecutionStatus::INVALID_PROTOCOL_VERSION;
      error->reason = "unsupported mission interface protocol version";
      return false;
    }
    if (assignment.assignment_uuid.empty() || assignment.decision_id.empty() ||
        assignment.decision_session_uuid.empty() ||
        assignment.destination.mission_id != mission_id_ ||
        assignment.destination.uav_id != uav_id_ ||
        assignment.destination.session_uuid != session_uuid_) {
      error->code = sar_yolo_detector::TaskExecutionStatus::INVALID_IDENTITY;
      error->reason = "assignment identity does not match this bridge";
      return false;
    }
    if (!allowed_decision_ids_.empty() && allowed_decision_ids_.count(assignment.decision_id) == 0U) {
      error->code = sar_yolo_detector::TaskExecutionStatus::INVALID_AUTHENTICATION;
      error->reason = "decision_id is not in the configured allow-list";
      return false;
    }
    if (assignment.command > sar_yolo_detector::TaskAssignment::CANCEL) {
      error->code = sar_yolo_detector::TaskExecutionStatus::INVALID_COMMAND;
      error->reason = "unknown assignment command";
      return false;
    }
    const double age = (ros::Time::now() - assignment.header.stamp).toSec();
    if (assignment.header.stamp.isZero() || age > maximum_assignment_age_sec_ ||
        age < -maximum_future_skew_sec_ || assignment.valid_until.isZero() ||
        assignment.valid_until <= ros::Time::now()) {
      error->code = sar_yolo_detector::TaskExecutionStatus::STALE_ASSIGNMENT;
      error->reason = "assignment timestamp/deadline is missing, stale, or in the future";
      return false;
    }
    if (!authenticationValid(assignment, error)) return false;
    if (require_operator_authorization_ && !assignment.operator_authorized) {
      error->code = sar_yolo_detector::TaskExecutionStatus::AUTHORIZATION_REQUIRED;
      error->reason = "deployment policy authorization assertion is required";
      return false;
    }
    if (assignment.operator_authorized && assignment.policy_version.empty()) {
      error->code = sar_yolo_detector::TaskExecutionStatus::AUTHORIZATION_REQUIRED;
      error->reason = "authorized assignments must identify the applied policy version";
      return false;
    }
    const std::string key = decisionKey(assignment);
    const auto last_sequence = decision_sequences_.find(key);
    if (last_sequence != decision_sequences_.end() &&
        assignment.assignment_sequence <= last_sequence->second) {
      error->code = sar_yolo_detector::TaskExecutionStatus::DUPLICATE_OR_OUT_OF_ORDER;
      error->reason = "assignment sequence is not newer for this decision session";
      return false;
    }
    auto found = assignments_.find(assignment.assignment_uuid);
    *existing = found == assignments_.end() ? nullptr : &found->second;
    if (assignment.command == sar_yolo_detector::TaskAssignment::ASSIGN && *existing != nullptr) {
      error->code = sar_yolo_detector::TaskExecutionStatus::DUPLICATE_OR_OUT_OF_ORDER;
      error->reason = "existing UUID must use UPDATE or CANCEL";
      return false;
    }
    if (assignment.command != sar_yolo_detector::TaskAssignment::ASSIGN) {
      if (*existing == nullptr || (*existing)->is_area != is_area ||
          !(*existing)->active) {
        error->code = sar_yolo_detector::TaskExecutionStatus::UNKNOWN_ASSIGNMENT;
        error->reason = "UPDATE/CANCEL references an unknown, terminal, or different-shape task";
        return false;
      }
      const std::string owner_id = is_area ? (*existing)->area.decision_id
                                           : (*existing)->point.decision_id;
      const std::string owner_session = is_area ? (*existing)->area.decision_session_uuid
                                                : (*existing)->point.decision_session_uuid;
      if (owner_id != assignment.decision_id || owner_session != assignment.decision_session_uuid) {
        error->code = sar_yolo_detector::TaskExecutionStatus::INVALID_AUTHENTICATION;
        error->reason = "only the owning decision session may update/cancel this task";
        return false;
      }
    }
    if (assignment.command == sar_yolo_detector::TaskAssignment::CANCEL) {
      if (!executorHeartbeatFresh()) {
        error->code = sar_yolo_detector::TaskExecutionStatus::EXECUTOR_UNAVAILABLE;
        error->reason = "cannot deliver cancellation without a fresh executor heartbeat";
        return false;
      }
      return true;
    }
    std::string ready_reason;
    if (!bridgeReady(&ready_reason)) {
      error->code = recovering_ ? sar_yolo_detector::TaskExecutionStatus::RECOVERY_MISMATCH
                                : sar_yolo_detector::TaskExecutionStatus::EXECUTOR_NOT_READY;
      error->reason = ready_reason;
      return false;
    }
    if (!finite(assignment.priority) || assignment.priority < 0.0F ||
        assignment.priority > 1.0F || assignment.task_type == 0U ||
        assignment.task_type > sar_yolo_detector::TaskAssignment::MONITOR) {
      error->code = sar_yolo_detector::TaskExecutionStatus::INVALID_CONSTRAINT;
      error->reason = "task type or priority is outside the protocol range";
      return false;
    }
    const auto &capabilities = executor_heartbeat_.capabilities;
    if (!contains(capabilities.supported_task_types, assignment.task_type) ||
        (is_area && !capabilities.supports_area_tasks) ||
        (!assignment.executor_profile.empty() &&
         !contains(capabilities.supported_executor_profiles, assignment.executor_profile))) {
      error->code = sar_yolo_detector::TaskExecutionStatus::UNSUPPORTED_TASK;
      error->reason = "executor does not advertise this task type/profile";
      return false;
    }
    const bool payload_mandatory =
        assignment.task_type == sar_yolo_detector::TaskAssignment::RESCUE ||
        assignment.task_type == sar_yolo_detector::TaskAssignment::DELIVER_SUPPLIES;
    if ((payload_mandatory && assignment.required_payload.empty()) ||
        (!assignment.required_payload.empty() &&
         !contains(capabilities.available_payloads, assignment.required_payload))) {
      error->code = sar_yolo_detector::TaskExecutionStatus::PAYLOAD_UNAVAILABLE;
      error->reason = payload_mandatory && assignment.required_payload.empty()
          ? "rescue/supply tasks must name a required payload"
          : "required payload is unavailable";
      return false;
    }
    if ((require_geofence_id_ && assignment.geofence_id.empty()) ||
        (!assignment.geofence_id.empty() &&
         !contains(capabilities.known_geofence_ids, assignment.geofence_id)) ||
        (require_geofence_version_ && assignment.geofence_version.empty()) ||
        (!assignment.geofence_version.empty() &&
         assignment.geofence_version != capabilities.geofence_version)) {
      error->code = sar_yolo_detector::TaskExecutionStatus::GEOFENCE_UNKNOWN;
      error->reason = "geofence ID/version is missing or differs from executor capabilities";
      return false;
    }
    if (!finite(assignment.maximum_speed_mps) || assignment.maximum_speed_mps <= 0.0F ||
        assignment.maximum_speed_mps > capabilities.maximum_speed_mps ||
        assignment.maximum_task_duration.toSec() <= 0.0 ||
        assignment.maximum_task_duration.toSec() > maximum_task_duration_sec_) {
      error->code = sar_yolo_detector::TaskExecutionStatus::INVALID_CONSTRAINT;
      error->reason = "speed or task duration is invalid or exceeds capabilities";
      return false;
    }
    if (assignment.command == sar_yolo_detector::TaskAssignment::ASSIGN) {
      std::size_t active = 0U;
      float lowest_priority = std::numeric_limits<float>::infinity();
      for (const auto &entry : assignments_) {
        if (!entry.second.active) continue;
        ++active;
        lowest_priority = std::min(lowest_priority,
            entry.second.is_area ? entry.second.area.priority : entry.second.point.priority);
      }
      if (active >= capabilities.maximum_active_tasks &&
          !(assignment.allow_preemption && capabilities.supports_preemption &&
            assignment.priority > lowest_priority)) {
        error->code = sar_yolo_detector::TaskExecutionStatus::CAPACITY_EXCEEDED;
        error->reason = "executor task capacity is exhausted";
        return false;
      }
    }
    return true;
  }

  template <typename Assignment>
  bool validateCandidate(const Assignment &assignment,
                         sar_yolo_detector::PerceptionCandidate **candidate,
                         ValidationError *error) {
    *candidate = nullptr;
    if (assignment.command == sar_yolo_detector::TaskAssignment::CANCEL) return true;
    if (assignment.observation_uuid.empty()) {
      if (!require_known_observation_) return true;
      error->code = sar_yolo_detector::TaskExecutionStatus::UNKNOWN_OBSERVATION;
      error->reason = "observation_uuid is required";
      return false;
    }
    auto found = candidate_cache_.find(assignment.observation_uuid);
    if (found == candidate_cache_.end()) {
      error->code = expired_observations_.count(assignment.observation_uuid) != 0U
          ? sar_yolo_detector::TaskExecutionStatus::OBSERVATION_EXPIRED
          : sar_yolo_detector::TaskExecutionStatus::UNKNOWN_OBSERVATION;
      error->reason = error->code == sar_yolo_detector::TaskExecutionStatus::OBSERVATION_EXPIRED
                          ? "perception observation has expired" : "perception observation is unknown";
      return false;
    }
    *candidate = &found->second;
    if ((*candidate)->lifecycle_state != sar_yolo_detector::PerceptionCandidate::CONFIRMED &&
        (*candidate)->lifecycle_state != sar_yolo_detector::PerceptionCandidate::UPDATED) {
      error->code = sar_yolo_detector::TaskExecutionStatus::UNKNOWN_OBSERVATION;
      error->reason = "only CONFIRMED/UPDATED observations may create tasks";
      return false;
    }
    if (assignment.semantic_type != (*candidate)->semantic_type) {
      error->code = sar_yolo_detector::TaskExecutionStatus::INVALID_CONSTRAINT;
      error->reason = "assignment semantic type does not match its observation";
      return false;
    }
    if ((ros::Time::now() - (*candidate)->last_seen).toSec() > maximum_candidate_age_sec_) {
      error->code = sar_yolo_detector::TaskExecutionStatus::OBSERVATION_EXPIRED;
      error->reason = "observation is too old for a new task";
      return false;
    }
    return true;
  }

  template <typename Assignment>
  sar_yolo_detector::TaskExecutionStatus makeStatus(const Assignment &assignment,
      const std::uint8_t state, const std::uint16_t reason_code,
      const std::string &reason) {
    sar_yolo_detector::TaskExecutionStatus status;
    status.header.stamp = ros::Time::now();
    status.assignment_uuid = assignment.assignment_uuid;
    status.assignment_sequence = assignment.assignment_sequence;
    status.status_sequence = ++status_sequence_;
    status.state = state;
    status.reason_code = reason_code;
    status.reason = reason;
    status.aircraft = identity();
    status.decision_id = assignment.decision_id;
    status.decision_session_uuid = assignment.decision_session_uuid;
    status.observation_uuid = assignment.observation_uuid;
    status.executor_id = have_executor_heartbeat_ ? executor_heartbeat_.executor_id
                                                  : "sar_decision_bridge";
    status.executor_session_uuid = have_executor_heartbeat_
        ? executor_heartbeat_.executor_session_uuid : std::string();
    status.updated_at = status.header.stamp;
    return status;
  }

  sar_yolo_detector::TaskExecutionStatus makeStatus(const AssignmentRecord &record,
      const std::uint8_t state, const std::uint16_t reason_code,
      const std::string &reason) {
    return record.is_area ? makeStatus(record.area, state, reason_code, reason)
                          : makeStatus(record.point, state, reason_code, reason);
  }

  void persistAndPublishStatus(const sar_yolo_detector::TaskExecutionStatus &status) {
    store_->begin();
    try {
      store_->saveStatus(status);
      store_->setMeta("last_status_sequence", std::to_string(status_sequence_));
      store_->commit();
    } catch (...) {
      store_->rollback();
      throw;
    }
    status_publisher_.publish(status);
  }

  template <typename Assignment>
  bool reject(const Assignment &assignment, const ValidationError &error,
              sar_yolo_detector::TaskExecutionStatus *ack) {
    *ack = makeStatus(assignment,
        sar_yolo_detector::TaskExecutionStatus::REJECTED_BY_ADAPTER,
        error.code, error.reason);
    persistAndPublishStatus(*ack);
    ROS_WARN_STREAM("Rejected assignment " << assignment.assignment_uuid << ": " << error.reason);
    return false;
  }

  PersistedAssignment persisted(const std::string &uuid,
                                const AssignmentRecord &record) const {
    PersistedAssignment saved;
    saved.assignment_uuid = uuid;
    saved.sequence = record.sequence;
    saved.active = record.active;
    saved.state = record.state;
    saved.is_area = record.is_area;
    saved.decision_key = record.decision_key;
    saved.decision_id = record.is_area ? record.area.decision_id : record.point.decision_id;
    saved.decision_session_uuid = record.is_area ? record.area.decision_session_uuid
                                                 : record.point.decision_session_uuid;
    saved.observation_uuid = record.is_area ? record.area.observation_uuid
                                            : record.point.observation_uuid;
    saved.executor_session_uuid = executor_heartbeat_.executor_session_uuid;
    saved.executor_status_sequence = record.executor_status_sequence;
    saved.dispatch_deadline_ns = record.dispatch_deadline.toNSec();
    saved.point_assignment = record.point;
    saved.area_assignment = record.area;
    return saved;
  }

  template <typename Assignment>
  bool checkReplay(const Assignment &assignment, const bool is_area,
                   sar_yolo_detector::TaskExecutionStatus *ack, bool *accepted) {
    const PersistedReplay replay = store_->loadReplay(replayKey(assignment));
    if (!replay.found) return false;
    if (replay.is_area != is_area || replay.input_digest != assignmentDigest(assignment)) {
      ValidationError error;
      error.code = sar_yolo_detector::TaskExecutionStatus::DUPLICATE_OR_OUT_OF_ORDER;
      error.reason = "assignment UUID/sequence was reused with different content";
      *accepted = reject(assignment, error, ack);
      return true;
    }
    *ack = replay.acknowledgment;
    *accepted = replay.accepted;
    status_publisher_.publish(*ack);
    return true;
  }

  template <typename Assignment>
  void commitAccepted(const Assignment &original, const bool is_area,
      const AssignmentRecord &record,
      const sar_yolo_detector::TaskExecutionStatus &ack) {
    store_->begin();
    try {
      store_->saveAssignment(persisted(original.assignment_uuid, record));
      store_->saveDecisionSequence(record.decision_key, original.assignment_sequence);
      store_->saveReplay(replayKey(original), is_area, assignmentDigest(original), true, ack);
      if (require_message_authentication_)
        store_->saveNonce(original.authorization_key_id, original.authorization_nonce,
                          replayKey(original));
      store_->saveStatus(ack);
      store_->setMeta("last_status_sequence", std::to_string(status_sequence_));
      store_->commit();
    } catch (...) {
      store_->rollback();
      throw;
    }
  }

  bool processPointAssignment(const sar_yolo_detector::TaskAssignment &input,
                              sar_yolo_detector::TaskExecutionStatus *ack) {
    bool replay_accepted = false;
    if (checkReplay(input, false, ack, &replay_accepted)) return replay_accepted;
    auto assignment = input;
    ValidationError error;
    AssignmentRecord *existing = nullptr;
    if (!validateCommon(assignment, false, &existing, &error)) return reject(input, error, ack);
    if (assignment.command == sar_yolo_detector::TaskAssignment::CANCEL) {
      const auto &previous = existing->point;
      assignment.task_type = previous.task_type;
      assignment.observation_uuid = previous.observation_uuid;
      assignment.semantic_type = previous.semantic_type;
      assignment.priority = previous.priority;
      assignment.target_pose_valid = previous.target_pose_valid;
      assignment.target_pose = previous.target_pose;
      assignment.coordinate_contract = previous.coordinate_contract;
      assignment.geo_reference = previous.geo_reference;
      assignment.arrival_tolerance_m = previous.arrival_tolerance_m;
      assignment.maximum_speed_mps = previous.maximum_speed_mps;
      assignment.maximum_task_duration = previous.maximum_task_duration;
      assignment.geofence_id = previous.geofence_id;
      assignment.geofence_version = previous.geofence_version;
      assignment.executor_profile = previous.executor_profile;
      assignment.required_payload = previous.required_payload;
    }
    sar_yolo_detector::PerceptionCandidate *candidate = nullptr;
    if (!validateCandidate(assignment, &candidate, &error)) return reject(input, error, ack);
    if (assignment.command != sar_yolo_detector::TaskAssignment::CANCEL) {
      if (!assignment.target_pose_valid && candidate != nullptr && candidate->localization_valid) {
        assignment.target_pose_valid = true;
        assignment.target_pose = candidate->target_pose;
        assignment.coordinate_contract = candidate->coordinate_contract;
        assignment.geo_reference = candidate->geo_reference;
      }
      if (!assignment.target_pose_valid) {
        error.code = sar_yolo_detector::TaskExecutionStatus::LOCALIZATION_REQUIRED;
        error.reason = "point task has no reliable navigation target";
        return reject(input, error, ack);
      }
      if (assignment.coordinate_contract != coordinate_contract_ ||
          !sameGeoReference(assignment.geo_reference, geo_reference_)) {
        error.code = sar_yolo_detector::TaskExecutionStatus::COORDINATE_CONTRACT_MISMATCH;
        error.reason = "point task geographic reference does not match the aircraft";
        return reject(input, error, ack);
      }
      const auto &capabilities = executor_heartbeat_.capabilities;
      if (!validPose(assignment.target_pose, required_target_frame_) ||
          !finite(assignment.arrival_tolerance_m) ||
          assignment.arrival_tolerance_m < capabilities.minimum_arrival_tolerance_m ||
          assignment.arrival_tolerance_m > capabilities.maximum_arrival_tolerance_m) {
        error.code = sar_yolo_detector::TaskExecutionStatus::INVALID_TARGET_POSE;
        error.reason = "target pose or arrival tolerance is invalid";
        return reject(input, error, ack);
      }
    }
    AssignmentRecord record;
    record.sequence = assignment.assignment_sequence;
    record.active = true;
    record.is_area = false;
    record.state = sar_yolo_detector::TaskExecutionStatus::DISPATCHED_TO_EXECUTOR;
    record.dispatch_deadline = ros::Time::now() + ros::Duration(executor_ack_timeout_sec_);
    record.decision_key = decisionKey(assignment);
    record.point = assignment;
    *ack = makeStatus(assignment, record.state,
        sar_yolo_detector::TaskExecutionStatus::REASON_NONE,
        "durably recorded and dispatched; awaiting executor acceptance");
    commitAccepted(input, false, record, *ack);
    assignments_[assignment.assignment_uuid] = record;
    decision_sequences_[record.decision_key] = assignment.assignment_sequence;
    last_assignment_sequence_ = std::max(last_assignment_sequence_, assignment.assignment_sequence);
    executor_assignment_publisher_.publish(assignment);
    status_publisher_.publish(*ack);
    return true;
  }

  bool processAreaAssignment(const sar_yolo_detector::AreaTaskAssignment &input,
                             sar_yolo_detector::TaskExecutionStatus *ack) {
    bool replay_accepted = false;
    if (checkReplay(input, true, ack, &replay_accepted)) return replay_accepted;
    auto assignment = input;
    ValidationError error;
    AssignmentRecord *existing = nullptr;
    if (!validateCommon(assignment, true, &existing, &error)) return reject(input, error, ack);
    if (assignment.command == sar_yolo_detector::AreaTaskAssignment::CANCEL) {
      const auto &previous = existing->area;
      assignment.task_type = previous.task_type;
      assignment.observation_uuid = previous.observation_uuid;
      assignment.global_region_uuid = previous.global_region_uuid;
      assignment.source_observation_uuids = previous.source_observation_uuids;
      assignment.semantic_type = previous.semantic_type;
      assignment.priority = previous.priority;
      assignment.target_area = previous.target_area;
      assignment.geo_reference = previous.geo_reference;
      assignment.preferred_entry_pose_valid = previous.preferred_entry_pose_valid;
      assignment.preferred_entry_pose = previous.preferred_entry_pose;
      assignment.survey_altitude_m = previous.survey_altitude_m;
      assignment.target_gsd_m_per_px = previous.target_gsd_m_per_px;
      assignment.lane_spacing_m = previous.lane_spacing_m;
      assignment.forward_overlap = previous.forward_overlap;
      assignment.side_overlap = previous.side_overlap;
      assignment.maximum_speed_mps = previous.maximum_speed_mps;
      assignment.maximum_task_duration = previous.maximum_task_duration;
      assignment.geofence_id = previous.geofence_id;
      assignment.geofence_version = previous.geofence_version;
      assignment.executor_profile = previous.executor_profile;
      assignment.required_payload = previous.required_payload;
    }
    sar_yolo_detector::PerceptionCandidate *candidate = nullptr;
    if (!validateCandidate(assignment, &candidate, &error)) return reject(input, error, ack);
    if (assignment.command != sar_yolo_detector::AreaTaskAssignment::CANCEL) {
      if (assignment.global_region_uuid.empty())
        assignment.global_region_uuid = assignment.observation_uuid;
      if (assignment.source_observation_uuids.empty())
        assignment.source_observation_uuids.push_back(assignment.observation_uuid);
      std::unordered_set<std::string> evidence_ids;
      bool evidence_list_valid = false;
      for (const auto &uuid : assignment.source_observation_uuids) {
        if (uuid.empty() || !evidence_ids.insert(uuid).second) {
          evidence_ids.clear();
          break;
        }
        if (uuid == assignment.observation_uuid) evidence_list_valid = true;
      }
      if (assignment.global_region_uuid.empty() || evidence_ids.empty() ||
          !evidence_list_valid) {
        error.code = sar_yolo_detector::TaskExecutionStatus::INVALID_CONSTRAINT;
        error.reason = "area task global region/evidence UUID set is invalid";
        return reject(input, error, ack);
      }
      if (assignment.target_area.polygon.points.empty() && candidate != nullptr &&
          candidate->localization_valid) {
        assignment.target_area = candidate->world_polygon;
        assignment.geo_reference = candidate->geo_reference;
      }
      double area = 0.0;
      const auto &capabilities = executor_heartbeat_.capabilities;
      if (!sameGeoReference(assignment.geo_reference, geo_reference_) ||
          !validPolygon(assignment.target_area, required_target_frame_, &area) ||
          area > maximum_area_m2_ || !finite(assignment.survey_altitude_m) ||
          assignment.survey_altitude_m < capabilities.minimum_survey_altitude_m ||
          assignment.survey_altitude_m > capabilities.maximum_survey_altitude_m ||
          !finite(assignment.target_gsd_m_per_px) || assignment.target_gsd_m_per_px <= 0.0F ||
          !finite(assignment.lane_spacing_m) || assignment.lane_spacing_m <= 0.0F ||
          !finite(assignment.forward_overlap) || assignment.forward_overlap < 0.0F ||
          assignment.forward_overlap >= 1.0F || !finite(assignment.side_overlap) ||
          assignment.side_overlap < 0.0F || assignment.side_overlap >= 1.0F ||
          (assignment.preferred_entry_pose_valid &&
           !validPose(assignment.preferred_entry_pose, required_target_frame_))) {
        error.code = sar_yolo_detector::TaskExecutionStatus::INVALID_CONSTRAINT;
        error.reason = "area geometry or survey parameters are invalid";
        return reject(input, error, ack);
      }
    }
    AssignmentRecord record;
    record.sequence = assignment.assignment_sequence;
    record.active = true;
    record.is_area = true;
    record.state = sar_yolo_detector::TaskExecutionStatus::DISPATCHED_TO_EXECUTOR;
    record.dispatch_deadline = ros::Time::now() + ros::Duration(executor_ack_timeout_sec_);
    record.decision_key = decisionKey(assignment);
    record.area = assignment;
    *ack = makeStatus(assignment, record.state,
        sar_yolo_detector::TaskExecutionStatus::REASON_NONE,
        "durably recorded and dispatched; awaiting executor acceptance");
    commitAccepted(input, true, record, *ack);
    assignments_[assignment.assignment_uuid] = record;
    decision_sequences_[record.decision_key] = assignment.assignment_sequence;
    last_assignment_sequence_ = std::max(last_assignment_sequence_, assignment.assignment_sequence);
    executor_area_assignment_publisher_.publish(assignment);
    status_publisher_.publish(*ack);
    return true;
  }

  void assignmentCallback(const sar_yolo_detector::TaskAssignment::ConstPtr &message) {
    sar_yolo_detector::TaskExecutionStatus ack;
    processPointAssignment(*message, &ack);
  }
  void areaAssignmentCallback(
      const sar_yolo_detector::AreaTaskAssignment::ConstPtr &message) {
    sar_yolo_detector::TaskExecutionStatus ack;
    processAreaAssignment(*message, &ack);
  }
  bool submitAssignmentCallback(sar_yolo_detector::SubmitTaskAssignment::Request &request,
      sar_yolo_detector::SubmitTaskAssignment::Response &response) {
    response.accepted = processPointAssignment(request.assignment, &response.acknowledgment);
    return true;
  }
  bool submitAreaAssignmentCallback(
      sar_yolo_detector::SubmitAreaTaskAssignment::Request &request,
      sar_yolo_detector::SubmitAreaTaskAssignment::Response &response) {
    response.accepted = processAreaAssignment(request.assignment, &response.acknowledgment);
    return true;
  }

  bool validTransition(const std::uint8_t current, const std::uint8_t next) const {
    using Status = sar_yolo_detector::TaskExecutionStatus;
    if (current == next) return true;
    if (current == Status::DISPATCHED_TO_EXECUTOR)
      return next == Status::ACCEPTED_BY_EXECUTOR || next == Status::FAILED ||
             next == Status::CANCELLED || next == Status::PREEMPTED;
    if (current == Status::ACCEPTED_BY_EXECUTOR)
      return next == Status::QUEUED || next == Status::FAILED ||
             next == Status::CANCELLED || next == Status::PREEMPTED;
    if (current == Status::QUEUED)
      return next == Status::EXECUTING || next == Status::FAILED ||
             next == Status::CANCELLED || next == Status::PREEMPTED;
    if (current == Status::EXECUTING)
      return next == Status::SUCCEEDED || next == Status::FAILED ||
             next == Status::CANCELLED || next == Status::PREEMPTED;
    return false;
  }

  bool terminalState(const std::uint8_t state) const {
    using Status = sar_yolo_detector::TaskExecutionStatus;
    return state == Status::SUCCEEDED || state == Status::FAILED ||
           state == Status::CANCELLED || state == Status::PREEMPTED ||
           state == Status::EXECUTOR_TIMEOUT;
  }

  void executorStatusCallback(
      const sar_yolo_detector::TaskExecutionStatus::ConstPtr &message) {
    auto found = assignments_.find(message->assignment_uuid);
    if (!executorHeartbeatFresh() || found == assignments_.end() ||
        message->aircraft.mission_id != mission_id_ || message->aircraft.uav_id != uav_id_ ||
        message->executor_id != executor_heartbeat_.executor_id ||
        message->executor_session_uuid != executor_heartbeat_.executor_session_uuid ||
        message->assignment_sequence != found->second.sequence) return;
    AssignmentRecord &record = found->second;
    if (message->executor_status_sequence <= record.executor_status_sequence) return;
    if (!validTransition(record.state, message->state)) {
      auto rejected = makeStatus(record, record.state,
          sar_yolo_detector::TaskExecutionStatus::INVALID_STATE_TRANSITION,
          "executor attempted an illegal task state transition");
      persistAndPublishStatus(rejected);
      return;
    }
    if (!finite(message->progress) || message->progress < 0.0F || message->progress > 1.0F ||
        (message->current_pose_valid && !validPose(message->current_pose, required_target_frame_)))
      return;
    record.state = message->state;
    record.executor_status_sequence = message->executor_status_sequence;
    record.active = !terminalState(message->state);
    sar_yolo_detector::TaskExecutionStatus output = *message;
    output.header.stamp = ros::Time::now();
    output.status_sequence = ++status_sequence_;
    output.aircraft = identity();
    output.decision_id = record.is_area ? record.area.decision_id : record.point.decision_id;
    output.decision_session_uuid = record.is_area ? record.area.decision_session_uuid
                                                  : record.point.decision_session_uuid;
    output.observation_uuid = record.is_area ? record.area.observation_uuid
                                             : record.point.observation_uuid;
    output.executor_id = executor_heartbeat_.executor_id;
    output.executor_session_uuid = executor_heartbeat_.executor_session_uuid;
    output.updated_at = output.header.stamp;
    store_->begin();
    try {
      store_->saveAssignment(persisted(found->first, record));
      store_->saveStatus(output);
      store_->setMeta("last_status_sequence", std::to_string(status_sequence_));
      store_->commit();
    } catch (...) {
      store_->rollback();
      throw;
    }
    status_publisher_.publish(output);
  }

  void executorHeartbeatCallback(
      const sar_yolo_detector::ExecutorHeartbeat::ConstPtr &message) {
    const auto &capabilities = message->capabilities;
    const double stamp_age = message->header.stamp.isZero()
        ? std::numeric_limits<double>::infinity()
        : (ros::Time::now() - message->header.stamp).toSec();
    if (message->aircraft.mission_id != mission_id_ || message->aircraft.uav_id != uav_id_ ||
        message->aircraft.session_uuid != session_uuid_ || message->executor_id.empty() ||
        message->executor_session_uuid.empty() ||
        capabilities.executor_id != message->executor_id ||
        capabilities.executor_session_uuid != message->executor_session_uuid ||
        stamp_age > executor_heartbeat_timeout_sec_ ||
        stamp_age < -maximum_future_skew_sec_ ||
        capabilities.capability_version == 0U ||
        capabilities.protocol_major != kProtocolMajor ||
        capabilities.protocol_minor > kProtocolMinor ||
        capabilities.supported_task_types.empty() ||
        capabilities.maximum_active_tasks == 0U ||
        !finite(capabilities.maximum_speed_mps) || capabilities.maximum_speed_mps <= 0.0F ||
        !finite(capabilities.minimum_arrival_tolerance_m) ||
        !finite(capabilities.maximum_arrival_tolerance_m) ||
        capabilities.minimum_arrival_tolerance_m <= 0.0F ||
        capabilities.maximum_arrival_tolerance_m < capabilities.minimum_arrival_tolerance_m ||
        !finite(capabilities.minimum_survey_altitude_m) ||
        !finite(capabilities.maximum_survey_altitude_m) ||
        capabilities.maximum_survey_altitude_m < capabilities.minimum_survey_altitude_m ||
        (require_geofence_version_ && capabilities.geofence_version.empty()) ||
        !finite(message->battery_percent) || message->battery_percent < 0.0F ||
        message->battery_percent > 100.0F) return;
    if (have_executor_heartbeat_ &&
        message->executor_session_uuid == executor_heartbeat_.executor_session_uuid &&
        message->heartbeat_sequence <= executor_heartbeat_.heartbeat_sequence) return;
    if (have_executor_heartbeat_ &&
        message->executor_session_uuid != executor_heartbeat_.executor_session_uuid) {
      recovering_ = true;
      recovery_reason_ = "executor restarted; reconciling active tasks";
    }
    executor_heartbeat_ = *message;
    have_executor_heartbeat_ = true;
    last_executor_heartbeat_at_ = ros::Time::now();
    if (recovering_) reconcileExecutorState();
  }

  void reconcileExecutorState() {
    std::unordered_map<std::string, sar_yolo_detector::TaskExecutionStatus> executor_active;
    bool orphaned = false;
    for (const auto &status : executor_heartbeat_.active_tasks) {
      const bool active_state =
          status.state == sar_yolo_detector::TaskExecutionStatus::ACCEPTED_BY_EXECUTOR ||
          status.state == sar_yolo_detector::TaskExecutionStatus::QUEUED ||
          status.state == sar_yolo_detector::TaskExecutionStatus::EXECUTING;
      if (status.assignment_uuid.empty() || status.assignment_sequence == 0U ||
          status.executor_status_sequence == 0U || !active_state ||
          status.aircraft.mission_id != mission_id_ ||
          status.aircraft.uav_id != uav_id_ ||
          status.aircraft.session_uuid != session_uuid_ ||
          status.executor_id != executor_heartbeat_.executor_id ||
          status.executor_session_uuid != executor_heartbeat_.executor_session_uuid ||
          executor_active.count(status.assignment_uuid) != 0U) {
        orphaned = true;
        continue;
      }
      executor_active[status.assignment_uuid] = status;
    }
    for (const auto &entry : executor_active)
      if (assignments_.count(entry.first) == 0U) orphaned = true;
    for (auto &entry : assignments_) {
      AssignmentRecord &record = entry.second;
      if (!record.active) continue;
      const auto remote = executor_active.find(entry.first);
      if (remote == executor_active.end()) {
        record.active = false;
        record.state = sar_yolo_detector::TaskExecutionStatus::FAILED;
        auto status = makeStatus(record, record.state,
            sar_yolo_detector::TaskExecutionStatus::RECOVERY_MISMATCH,
            "persisted active task is absent from executor recovery snapshot");
        store_->begin();
        try {
          store_->saveAssignment(persisted(entry.first, record));
          store_->saveStatus(status);
          store_->setMeta("last_status_sequence", std::to_string(status_sequence_));
          store_->commit();
        } catch (...) {
          store_->rollback();
          throw;
        }
        status_publisher_.publish(status);
      } else if (remote->second.assignment_sequence != record.sequence ||
                 (remote->second.state !=
                      sar_yolo_detector::TaskExecutionStatus::ACCEPTED_BY_EXECUTOR &&
                  remote->second.state !=
                      sar_yolo_detector::TaskExecutionStatus::QUEUED &&
                  remote->second.state !=
                      sar_yolo_detector::TaskExecutionStatus::EXECUTING)) {
        orphaned = true;
      } else {
        record.executor_status_sequence = remote->second.executor_status_sequence;
        record.state = remote->second.state;
        auto status = makeStatus(record, record.state,
            sar_yolo_detector::TaskExecutionStatus::REASON_NONE,
            "active task reconciled with executor after bridge restart");
        status.executor_status_sequence = record.executor_status_sequence;
        store_->begin();
        try {
          store_->saveAssignment(persisted(entry.first, record));
          store_->saveStatus(status);
          store_->setMeta("last_status_sequence", std::to_string(status_sequence_));
          store_->commit();
        } catch (...) {
          store_->rollback();
          throw;
        }
        status_publisher_.publish(status);
      }
    }
    recovering_ = orphaned;
    recovery_reason_ = orphaned ? "executor reports orphaned or inconsistent active tasks"
                                : std::string();
  }

  void maintenanceCallback(const ros::TimerEvent &) {
    const ros::Time now = ros::Time::now();
    for (auto &entry : assignments_) {
      AssignmentRecord &record = entry.second;
      if (!record.active || record.state !=
              sar_yolo_detector::TaskExecutionStatus::DISPATCHED_TO_EXECUTOR ||
          record.dispatch_deadline.isZero() || now <= record.dispatch_deadline) continue;
      record.active = false;
      record.state = sar_yolo_detector::TaskExecutionStatus::EXECUTOR_TIMEOUT;
      auto status = makeStatus(record, record.state,
          sar_yolo_detector::TaskExecutionStatus::EXECUTOR_ACK_TIMEOUT,
          "executor did not acknowledge the durable command before its deadline");
      store_->begin();
      try {
        store_->saveAssignment(persisted(entry.first, record));
        store_->saveStatus(status);
        store_->setMeta("last_status_sequence", std::to_string(status_sequence_));
        store_->commit();
      } catch (...) {
        store_->rollback();
        throw;
      }
      status_publisher_.publish(status);
    }
  }

  bool snapshotCallback(sar_yolo_detector::GetPerceptionSnapshot::Request &,
      sar_yolo_detector::GetPerceptionSnapshot::Response &response) {
    response.available = perception_available_;
    response.snapshot.header.stamp = ros::Time::now();
    response.snapshot.array_sequence = candidate_sequence_;
    response.snapshot.full_snapshot = true;
    response.snapshot.source = identity();
    for (const auto &entry : candidate_cache_) response.snapshot.candidates.push_back(entry.second);
    std::sort(response.snapshot.candidates.begin(), response.snapshot.candidates.end(),
        [](const sar_yolo_detector::PerceptionCandidate &left,
           const sar_yolo_detector::PerceptionCandidate &right) {
          return left.observation_uuid < right.observation_uuid;
        });
    return true;
  }

  bool evidenceCallback(sar_yolo_detector::GetEvidenceCrop::Request &request,
      sar_yolo_detector::GetEvidenceCrop::Response &response) {
    response.available = false;
    if (request.evidence_uuid.empty() ||
        request.source.mission_id != mission_id_ ||
        request.source.uav_id != uav_id_ ||
        request.source.session_uuid != session_uuid_ ||
        source_evidence_service_.empty()) return true;
    sar_yolo_detector::GetEvidenceCrop local;
    local.request = request;
    if (source_evidence_client_.call(local)) response = local.response;
    return true;
  }

  bool statusesCallback(sar_yolo_detector::GetTaskStatuses::Request &request,
      sar_yolo_detector::GetTaskStatuses::Response &response) {
    const std::uint32_t limit = request.limit == 0U ? 100U : request.limit;
    bool has_more = false;
    response.statuses = store_->loadStatuses(request.assignment_uuid,
        request.since_status_sequence, limit, &has_more,
        &response.next_status_sequence);
    response.has_more = has_more;
    return true;
  }

  void heartbeatTimerCallback(const ros::TimerEvent &) { publishHeartbeat(); }

  void publishHeartbeat() {
    sar_yolo_detector::UavDecisionHeartbeat heartbeat;
    heartbeat.header.stamp = ros::Time::now();
    heartbeat.aircraft = identity();
    heartbeat.interface_version = kInterfaceVersion;
    heartbeat.protocol_major = kProtocolMajor;
    heartbeat.protocol_minor = kProtocolMinor;
    heartbeat.coordinate_contract = coordinate_contract_;
    heartbeat.geo_reference = geo_reference_;
    std::string reason;
    heartbeat.ready_for_assignments = bridgeReady(&reason);
    heartbeat.not_ready_reason = reason;
    if (!assignment_downlink_enabled_) {
      heartbeat.bridge_state = perception_uplink_enabled_
          ? sar_yolo_detector::UavDecisionHeartbeat::READY
          : sar_yolo_detector::UavDecisionHeartbeat::DEGRADED;
    } else {
      heartbeat.bridge_state = recovering_
          ? sar_yolo_detector::UavDecisionHeartbeat::RECOVERING
          : (heartbeat.ready_for_assignments
                 ? sar_yolo_detector::UavDecisionHeartbeat::READY
                 : sar_yolo_detector::UavDecisionHeartbeat::DEGRADED);
    }
    heartbeat.heartbeat_sequence = ++heartbeat_sequence_;
    heartbeat.last_candidate_sequence = candidate_sequence_;
    heartbeat.last_assignment_sequence = last_assignment_sequence_;
    heartbeat.last_status_sequence = status_sequence_;
    heartbeat.last_candidate_at = last_candidate_at_;
    heartbeat.last_executor_heartbeat_at = last_executor_heartbeat_at_;
    if (have_executor_heartbeat_) {
      heartbeat.executor_id = executor_heartbeat_.executor_id;
      heartbeat.executor_session_uuid = executor_heartbeat_.executor_session_uuid;
    }
    for (const auto &entry : assignments_) {
      if (!entry.second.active) continue;
      ++heartbeat.active_assignment_count;
      if (heartbeat.current_assignment_uuid.empty()) heartbeat.current_assignment_uuid = entry.first;
    }
    heartbeat_publisher_.publish(heartbeat);
  }

  ros::NodeHandle node_, private_node_;
  ros::Publisher candidates_publisher_, status_publisher_, heartbeat_publisher_;
  ros::Publisher executor_assignment_publisher_, executor_area_assignment_publisher_;
  ros::Subscriber task_candidates_subscriber_, flood_regions_subscriber_;
  ros::Subscriber assignment_subscriber_, area_assignment_subscriber_;
  ros::Subscriber executor_status_subscriber_, executor_heartbeat_subscriber_;
  ros::ServiceServer submit_service_, submit_area_service_, snapshot_service_, statuses_service_;
  ros::ServiceServer evidence_service_;
  ros::ServiceClient source_evidence_client_;
  ros::Timer heartbeat_timer_, maintenance_timer_;

  std::string mission_id_, uav_id_, session_uuid_, bridge_role_;
  std::string coordinate_contract_, required_target_frame_;
  sar_yolo_detector::GeoReference geo_reference_;
  bool geo_reference_validated_{false};
  std::string source_task_candidates_topic_, source_flood_regions_topic_;
  std::string outbound_candidates_topic_, inbound_assignment_topic_;
  std::string inbound_area_assignment_topic_, outbound_status_topic_, heartbeat_topic_;
  std::string executor_assignment_topic_, executor_area_assignment_topic_;
  std::string executor_status_topic_, executor_heartbeat_topic_;
  std::string submit_assignment_service_, submit_area_assignment_service_;
  std::string snapshot_service_name_, statuses_service_name_, evidence_source_topic_;
  std::string evidence_service_name_, source_evidence_service_;
  std::string evidence_retrieval_base_uri_;
  std::string state_database_path_;

  bool allow_topic_commands_{false}, require_message_authentication_{true};
  bool perception_uplink_enabled_{true}, assignment_downlink_enabled_{true};
  bool require_operator_authorization_{true}, require_known_observation_{true};
  bool require_geofence_id_{true}, require_geofence_version_{true};
  double maximum_assignment_age_sec_{5.0}, maximum_authorization_age_sec_{10.0};
  double maximum_future_skew_sec_{0.5}, maximum_candidate_age_sec_{5.0};
  double executor_heartbeat_timeout_sec_{2.5}, executor_ack_timeout_sec_{2.0};
  double maximum_task_duration_sec_{86400.0}, maximum_area_m2_{1.0e8};
  double minimum_executor_battery_percent_{20.0};
  double heartbeat_rate_hz_{1.0};

  std::unique_ptr<MissionStateStore> store_;
  std::unordered_map<std::string, std::string> authorization_keys_;
  std::unordered_map<std::string, std::string> authorization_key_decision_ids_;
  std::unordered_set<std::string> allowed_decision_ids_;
  std::unordered_map<std::string, std::uint64_t> decision_sequences_;
  std::unordered_map<std::string, AssignmentRecord> assignments_;
  std::unordered_map<std::string, sar_yolo_detector::PerceptionCandidate> candidate_cache_;
  std::unordered_map<std::string, std::string> candidate_sources_;
  std::unordered_map<std::string, std::uint64_t> candidate_event_sequences_;
  std::unordered_map<std::string, std::uint64_t> source_array_sequences_;
  std::unordered_set<std::string> expired_observations_;
  sar_yolo_detector::ExecutorHeartbeat executor_heartbeat_;
  bool have_executor_heartbeat_{false}, recovering_{true}, perception_available_{false};
  std::string recovery_reason_;
  ros::Time last_candidate_at_, last_executor_heartbeat_at_;
  std::uint64_t candidate_sequence_{0}, status_sequence_{0};
  std::uint64_t heartbeat_sequence_{0}, last_assignment_sequence_{0};
};

}  // namespace sar_yolo_detector

int main(int argc, char **argv) {
  ros::init(argc, argv, "sar_decision_bridge");
  try {
    sar_yolo_detector::DecisionBridge bridge;
    ros::spin();
  } catch (const std::exception &exception) {
    ROS_FATAL_STREAM("Failed to start SAR decision bridge: " << exception.what());
    return 1;
  }
  return 0;
}
