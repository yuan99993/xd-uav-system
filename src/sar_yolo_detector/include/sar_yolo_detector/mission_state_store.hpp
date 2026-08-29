#pragma once

#include <cstdint>
#include <string>
#include <stdexcept>
#include <unordered_map>
#include <vector>

#include <ros/serialization.h>

#include <sar_yolo_detector/AreaTaskAssignment.h>
#include <sar_yolo_detector/PerceptionCandidate.h>
#include <sar_yolo_detector/TaskAssignment.h>
#include <sar_yolo_detector/TaskExecutionStatus.h>

struct sqlite3;

namespace sar_yolo_detector {

struct PersistedAssignment {
  std::string assignment_uuid;
  std::uint64_t sequence{0};
  bool active{false};
  std::uint8_t state{0};
  bool is_area{false};
  std::string decision_key;
  std::string decision_id;
  std::string decision_session_uuid;
  std::string observation_uuid;
  std::string executor_session_uuid;
  std::uint64_t executor_status_sequence{0};
  std::int64_t dispatch_deadline_ns{0};
  sar_yolo_detector::TaskAssignment point_assignment;
  sar_yolo_detector::AreaTaskAssignment area_assignment;
};

struct PersistedReplay {
  bool found{false};
  bool accepted{false};
  bool is_area{false};
  std::string input_digest;
  sar_yolo_detector::TaskExecutionStatus acknowledgment;
};

struct PersistedCandidate {
  std::string source_key;
  sar_yolo_detector::PerceptionCandidate candidate;
};

class MissionStateStore {
 public:
  explicit MissionStateStore(const std::string &database_path);
  ~MissionStateStore();
  MissionStateStore(const MissionStateStore &) = delete;
  MissionStateStore &operator=(const MissionStateStore &) = delete;

  void begin();
  void commit();
  void rollback() noexcept;

  void saveAssignment(const PersistedAssignment &record);
  std::vector<PersistedAssignment> loadAssignments() const;

  void saveDecisionSequence(const std::string &decision_key,
                            std::uint64_t sequence);
  std::unordered_map<std::string, std::uint64_t> loadDecisionSequences() const;

  void saveReplay(const std::string &replay_key, bool is_area,
                  const std::string &input_digest, bool accepted,
                  const sar_yolo_detector::TaskExecutionStatus &ack);
  PersistedReplay loadReplay(const std::string &replay_key) const;

  bool nonceExists(const std::string &key_id, const std::string &nonce) const;
  void saveNonce(const std::string &key_id, const std::string &nonce,
                 const std::string &replay_key);

  void saveStatus(
      const sar_yolo_detector::TaskExecutionStatus &status);
  std::vector<sar_yolo_detector::TaskExecutionStatus> loadStatuses(
      const std::string &assignment_uuid, std::uint64_t since_sequence,
      std::uint32_t limit, bool *has_more,
      std::uint64_t *next_sequence) const;

  void replaceCandidateSource(
      const std::string &source_key,
      const std::vector<sar_yolo_detector::PerceptionCandidate> &candidates);
  void saveCandidate(
      const std::string &source_key,
      const sar_yolo_detector::PerceptionCandidate &candidate);
  void eraseCandidate(const std::string &source_key,
                      const std::string &observation_uuid);
  void eraseCandidateSource(const std::string &source_key);
  std::vector<PersistedCandidate> loadCandidates() const;

  void saveCandidateSourceSequence(const std::string &source_key,
                                   std::uint64_t array_sequence);
  std::unordered_map<std::string, std::uint64_t>
  loadCandidateSourceSequences() const;

  void setMeta(const std::string &key, const std::string &value);
  std::string getMeta(const std::string &key,
                      const std::string &default_value) const;

  static std::string sha256Hex(const std::vector<std::uint8_t> &bytes);

  template <typename Message>
  static std::vector<std::uint8_t> serialize(const Message &message) {
    const std::uint32_t length = ros::serialization::serializationLength(message);
    std::vector<std::uint8_t> bytes(length);
    if (length > 0U) {
      ros::serialization::OStream stream(bytes.data(), length);
      ros::serialization::serialize(stream, message);
    }
    return bytes;
  }

  template <typename Message>
  static Message deserialize(const void *data, const int length) {
    if (data == nullptr || length <= 0) {
      throw std::runtime_error("cannot deserialize an empty ROS message blob");
    }
    std::vector<std::uint8_t> bytes(
        static_cast<const std::uint8_t *>(data),
        static_cast<const std::uint8_t *>(data) + length);
    ros::serialization::IStream stream(bytes.data(), bytes.size());
    Message message;
    ros::serialization::deserialize(stream, message);
    return message;
  }

 private:
  void execute(const std::string &sql) const;
  void initializeSchema();
  sqlite3 *database_{nullptr};
  bool transaction_active_{false};
};

}  // namespace sar_yolo_detector
