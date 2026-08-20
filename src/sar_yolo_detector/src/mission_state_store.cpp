#include <sar_yolo_detector/mission_state_store.hpp>

#include <algorithm>
#include <cstring>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <cerrno>

#include <sys/stat.h>
#include <unistd.h>

#include <openssl/sha.h>
#include <sqlite3.h>

namespace sar_yolo_detector {
namespace {

class Statement {
 public:
  Statement(sqlite3 *database, const char *sql) : database_(database) {
    if (sqlite3_prepare_v2(database, sql, -1, &statement_, nullptr) !=
        SQLITE_OK) {
      throw std::runtime_error(std::string("sqlite prepare failed: ") +
                               sqlite3_errmsg(database));
    }
  }
  ~Statement() { sqlite3_finalize(statement_); }
  sqlite3_stmt *get() const { return statement_; }
  void stepDone() const {
    if (sqlite3_step(statement_) != SQLITE_DONE) {
      throw std::runtime_error(std::string("sqlite statement failed: ") +
                               sqlite3_errmsg(database_));
    }
  }

 private:
  sqlite3 *database_;
  sqlite3_stmt *statement_{nullptr};
};

void bindText(sqlite3_stmt *statement, int index, const std::string &value) {
  if (sqlite3_bind_text(statement, index, value.c_str(), value.size(),
                        SQLITE_TRANSIENT) != SQLITE_OK) {
    throw std::runtime_error("sqlite text bind failed");
  }
}

void bindBlob(sqlite3_stmt *statement, int index,
              const std::vector<std::uint8_t> &value) {
  const void *data = value.empty() ? nullptr : value.data();
  if (sqlite3_bind_blob(statement, index, data, value.size(), SQLITE_TRANSIENT) !=
      SQLITE_OK) {
    throw std::runtime_error("sqlite blob bind failed");
  }
}

std::string columnText(sqlite3_stmt *statement, int column) {
  const auto *text = sqlite3_column_text(statement, column);
  return text == nullptr ? std::string()
                         : std::string(reinterpret_cast<const char *>(text));
}

}  // namespace

MissionStateStore::MissionStateStore(const std::string &database_path) {
  if (database_path.empty()) {
    throw std::invalid_argument("mission state database path must not be empty");
  }
  if (database_path != ":memory:") {
    struct stat metadata {};
    if (::lstat(database_path.c_str(), &metadata) == 0) {
      if (!S_ISREG(metadata.st_mode) || metadata.st_uid != ::geteuid())
        throw std::runtime_error(
            "mission state path must be an owner-controlled regular non-symlink file");
    } else if (errno != ENOENT) {
      throw std::runtime_error("cannot inspect mission state database path");
    }
  }
  const int flags = SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE |
                    SQLITE_OPEN_FULLMUTEX;
  if (sqlite3_open_v2(database_path.c_str(), &database_, flags, nullptr) !=
      SQLITE_OK) {
    const std::string error = database_ == nullptr
                                  ? "unknown sqlite open error"
                                  : sqlite3_errmsg(database_);
    if (database_ != nullptr) sqlite3_close(database_);
    database_ = nullptr;
    throw std::runtime_error("cannot open mission state database: " + error);
  }
  sqlite3_busy_timeout(database_, 2000);
  execute("PRAGMA journal_mode=WAL;");
  execute("PRAGMA synchronous=FULL;");
  execute("PRAGMA foreign_keys=ON;");
  initializeSchema();
  if (database_path != ":memory:") {
    const auto make_owner_only = [](const std::string &path) {
      if (::chmod(path.c_str(), S_IRUSR | S_IWUSR) != 0 && errno != ENOENT) {
        throw std::runtime_error("cannot restrict mission state file permissions: " +
                                 path);
      }
    };
    make_owner_only(database_path);
    make_owner_only(database_path + "-wal");
    make_owner_only(database_path + "-shm");
  }
}

MissionStateStore::~MissionStateStore() {
  rollback();
  if (database_ != nullptr) sqlite3_close(database_);
}

void MissionStateStore::execute(const std::string &sql) const {
  char *error = nullptr;
  if (sqlite3_exec(database_, sql.c_str(), nullptr, nullptr, &error) !=
      SQLITE_OK) {
    const std::string message = error == nullptr ? "unknown error" : error;
    sqlite3_free(error);
    throw std::runtime_error("sqlite execute failed: " + message);
  }
}

void MissionStateStore::initializeSchema() {
  execute("CREATE TABLE IF NOT EXISTS meta (key TEXT PRIMARY KEY, value TEXT NOT NULL);");
  const std::string existing_schema = getMeta("schema_version", "");
  if (!existing_schema.empty() && existing_schema != "1")
    throw std::runtime_error("unsupported mission state database schema " +
                             existing_schema);
  execute("CREATE TABLE IF NOT EXISTS assignments ("
          "assignment_uuid TEXT PRIMARY KEY, sequence INTEGER NOT NULL,"
          "active INTEGER NOT NULL, state INTEGER NOT NULL, is_area INTEGER NOT NULL,"
          "decision_key TEXT NOT NULL, decision_id TEXT NOT NULL,"
          "decision_session_uuid TEXT NOT NULL, observation_uuid TEXT NOT NULL,"
          "executor_session_uuid TEXT NOT NULL, executor_status_sequence INTEGER NOT NULL,"
          "dispatch_deadline_ns INTEGER NOT NULL, updated_ns INTEGER NOT NULL,"
          "payload BLOB NOT NULL);");
  execute("CREATE TABLE IF NOT EXISTS decision_sequences ("
          "decision_key TEXT PRIMARY KEY, sequence INTEGER NOT NULL);");
  execute("CREATE TABLE IF NOT EXISTS replays ("
          "replay_key TEXT PRIMARY KEY, is_area INTEGER NOT NULL,"
          "input_digest TEXT NOT NULL, accepted INTEGER NOT NULL,"
          "acknowledgment BLOB NOT NULL);");
  execute("CREATE TABLE IF NOT EXISTS authorization_nonces ("
          "key_id TEXT NOT NULL, nonce TEXT NOT NULL, replay_key TEXT NOT NULL,"
          "PRIMARY KEY(key_id, nonce));");
  execute("CREATE TABLE IF NOT EXISTS statuses ("
          "status_sequence INTEGER PRIMARY KEY, assignment_uuid TEXT NOT NULL,"
          "payload BLOB NOT NULL);");
  execute("CREATE INDEX IF NOT EXISTS statuses_assignment_idx ON statuses(assignment_uuid, status_sequence);");
  execute("CREATE TABLE IF NOT EXISTS candidates ("
          "source_key TEXT NOT NULL, observation_uuid TEXT NOT NULL,"
          "event_sequence INTEGER NOT NULL, lifecycle INTEGER NOT NULL,"
          "payload BLOB NOT NULL, PRIMARY KEY(source_key, observation_uuid));");
  execute("CREATE TABLE IF NOT EXISTS candidate_source_sequences ("
          "source_key TEXT PRIMARY KEY, array_sequence INTEGER NOT NULL);");
  setMeta("schema_version", "1");
}

void MissionStateStore::begin() {
  if (transaction_active_) throw std::logic_error("sqlite transaction nested");
  execute("BEGIN IMMEDIATE;");
  transaction_active_ = true;
}

void MissionStateStore::commit() {
  if (!transaction_active_) throw std::logic_error("no sqlite transaction");
  execute("COMMIT;");
  transaction_active_ = false;
}

void MissionStateStore::rollback() noexcept {
  if (!transaction_active_ || database_ == nullptr) return;
  sqlite3_exec(database_, "ROLLBACK;", nullptr, nullptr, nullptr);
  transaction_active_ = false;
}

void MissionStateStore::saveAssignment(const PersistedAssignment &record) {
  static const char *sql =
      "INSERT INTO assignments VALUES(?,?,?,?,?,?,?,?,?,?,?,?,?,?) "
      "ON CONFLICT(assignment_uuid) DO UPDATE SET sequence=excluded.sequence,"
      "active=excluded.active,state=excluded.state,is_area=excluded.is_area,"
      "decision_key=excluded.decision_key,decision_id=excluded.decision_id,"
      "decision_session_uuid=excluded.decision_session_uuid,"
      "observation_uuid=excluded.observation_uuid,"
      "executor_session_uuid=excluded.executor_session_uuid,"
      "executor_status_sequence=excluded.executor_status_sequence,"
      "dispatch_deadline_ns=excluded.dispatch_deadline_ns,"
      "updated_ns=excluded.updated_ns,payload=excluded.payload;";
  Statement statement(database_, sql);
  sqlite3_stmt *s = statement.get();
  bindText(s, 1, record.assignment_uuid);
  sqlite3_bind_int64(s, 2, record.sequence);
  sqlite3_bind_int(s, 3, record.active ? 1 : 0);
  sqlite3_bind_int(s, 4, record.state);
  sqlite3_bind_int(s, 5, record.is_area ? 1 : 0);
  bindText(s, 6, record.decision_key);
  bindText(s, 7, record.decision_id);
  bindText(s, 8, record.decision_session_uuid);
  bindText(s, 9, record.observation_uuid);
  bindText(s, 10, record.executor_session_uuid);
  sqlite3_bind_int64(s, 11, record.executor_status_sequence);
  sqlite3_bind_int64(s, 12, record.dispatch_deadline_ns);
  sqlite3_bind_int64(s, 13, ros::Time::now().toNSec());
  bindBlob(s, 14, record.is_area ? serialize(record.area_assignment)
                                 : serialize(record.point_assignment));
  statement.stepDone();
}

std::vector<PersistedAssignment> MissionStateStore::loadAssignments() const {
  Statement statement(database_,
      "SELECT assignment_uuid,sequence,active,state,is_area,decision_key,"
      "decision_id,decision_session_uuid,observation_uuid,executor_session_uuid,"
      "executor_status_sequence,dispatch_deadline_ns,payload FROM assignments;");
  std::vector<PersistedAssignment> result;
  while (sqlite3_step(statement.get()) == SQLITE_ROW) {
    PersistedAssignment record;
    record.assignment_uuid = columnText(statement.get(), 0);
    record.sequence = sqlite3_column_int64(statement.get(), 1);
    record.active = sqlite3_column_int(statement.get(), 2) != 0;
    record.state = static_cast<std::uint8_t>(sqlite3_column_int(statement.get(), 3));
    record.is_area = sqlite3_column_int(statement.get(), 4) != 0;
    record.decision_key = columnText(statement.get(), 5);
    record.decision_id = columnText(statement.get(), 6);
    record.decision_session_uuid = columnText(statement.get(), 7);
    record.observation_uuid = columnText(statement.get(), 8);
    record.executor_session_uuid = columnText(statement.get(), 9);
    record.executor_status_sequence = sqlite3_column_int64(statement.get(), 10);
    record.dispatch_deadline_ns = sqlite3_column_int64(statement.get(), 11);
    const void *blob = sqlite3_column_blob(statement.get(), 12);
    const int size = sqlite3_column_bytes(statement.get(), 12);
    if (record.is_area) {
      record.area_assignment =
          deserialize<sar_mission_interfaces::AreaTaskAssignment>(blob, size);
    } else {
      record.point_assignment =
          deserialize<sar_mission_interfaces::TaskAssignment>(blob, size);
    }
    result.push_back(std::move(record));
  }
  return result;
}

void MissionStateStore::saveDecisionSequence(const std::string &key,
                                              std::uint64_t sequence) {
  Statement statement(database_,
      "INSERT INTO decision_sequences VALUES(?,?) ON CONFLICT(decision_key) "
      "DO UPDATE SET sequence=excluded.sequence;");
  bindText(statement.get(), 1, key);
  sqlite3_bind_int64(statement.get(), 2, sequence);
  statement.stepDone();
}

std::unordered_map<std::string, std::uint64_t>
MissionStateStore::loadDecisionSequences() const {
  Statement statement(database_, "SELECT decision_key,sequence FROM decision_sequences;");
  std::unordered_map<std::string, std::uint64_t> result;
  while (sqlite3_step(statement.get()) == SQLITE_ROW) {
    result[columnText(statement.get(), 0)] = sqlite3_column_int64(statement.get(), 1);
  }
  return result;
}

void MissionStateStore::saveReplay(
    const std::string &key, bool is_area, const std::string &digest,
    bool accepted, const sar_mission_interfaces::TaskExecutionStatus &ack) {
  Statement statement(database_,
      "INSERT INTO replays VALUES(?,?,?,?,?) ON CONFLICT(replay_key) DO NOTHING;");
  bindText(statement.get(), 1, key);
  sqlite3_bind_int(statement.get(), 2, is_area ? 1 : 0);
  bindText(statement.get(), 3, digest);
  sqlite3_bind_int(statement.get(), 4, accepted ? 1 : 0);
  bindBlob(statement.get(), 5, serialize(ack));
  statement.stepDone();
}

PersistedReplay MissionStateStore::loadReplay(const std::string &key) const {
  Statement statement(database_,
      "SELECT is_area,input_digest,accepted,acknowledgment FROM replays WHERE replay_key=?;");
  bindText(statement.get(), 1, key);
  PersistedReplay result;
  if (sqlite3_step(statement.get()) != SQLITE_ROW) return result;
  result.found = true;
  result.is_area = sqlite3_column_int(statement.get(), 0) != 0;
  result.input_digest = columnText(statement.get(), 1);
  result.accepted = sqlite3_column_int(statement.get(), 2) != 0;
  result.acknowledgment = deserialize<sar_mission_interfaces::TaskExecutionStatus>(
      sqlite3_column_blob(statement.get(), 3),
      sqlite3_column_bytes(statement.get(), 3));
  return result;
}

bool MissionStateStore::nonceExists(const std::string &key_id,
                                    const std::string &nonce) const {
  Statement statement(database_,
      "SELECT 1 FROM authorization_nonces WHERE key_id=? AND nonce=?;");
  bindText(statement.get(), 1, key_id);
  bindText(statement.get(), 2, nonce);
  return sqlite3_step(statement.get()) == SQLITE_ROW;
}

void MissionStateStore::saveNonce(const std::string &key_id,
                                  const std::string &nonce,
                                  const std::string &replay_key) {
  Statement statement(database_,
      "INSERT INTO authorization_nonces VALUES(?,?,?);");
  bindText(statement.get(), 1, key_id);
  bindText(statement.get(), 2, nonce);
  bindText(statement.get(), 3, replay_key);
  statement.stepDone();
}

void MissionStateStore::saveStatus(
    const sar_mission_interfaces::TaskExecutionStatus &status) {
  Statement statement(database_,
      "INSERT INTO statuses VALUES(?,?,?) ON CONFLICT(status_sequence) DO NOTHING;");
  sqlite3_bind_int64(statement.get(), 1, status.status_sequence);
  bindText(statement.get(), 2, status.assignment_uuid);
  bindBlob(statement.get(), 3, serialize(status));
  statement.stepDone();
}

std::vector<sar_mission_interfaces::TaskExecutionStatus>
MissionStateStore::loadStatuses(const std::string &assignment_uuid,
                                std::uint64_t since_sequence,
                                std::uint32_t limit, bool *has_more,
                                std::uint64_t *next_sequence) const {
  const std::uint32_t bounded_limit = std::max(1U, std::min(1000U, limit));
  const char *sql_all =
      "SELECT status_sequence,payload FROM statuses WHERE status_sequence>? "
      "ORDER BY status_sequence LIMIT ?;";
  const char *sql_one =
      "SELECT status_sequence,payload FROM statuses WHERE status_sequence>? "
      "AND assignment_uuid=? ORDER BY status_sequence LIMIT ?;";
  Statement statement(database_, assignment_uuid.empty() ? sql_all : sql_one);
  sqlite3_bind_int64(statement.get(), 1, since_sequence);
  int limit_index = 2;
  if (!assignment_uuid.empty()) {
    bindText(statement.get(), 2, assignment_uuid);
    limit_index = 3;
  }
  sqlite3_bind_int(statement.get(), limit_index, bounded_limit + 1U);
  std::vector<sar_mission_interfaces::TaskExecutionStatus> result;
  std::uint64_t last = since_sequence;
  bool extra_row = false;
  while (sqlite3_step(statement.get()) == SQLITE_ROW) {
    if (result.size() >= bounded_limit) {
      extra_row = true;
      break;
    }
    last = sqlite3_column_int64(statement.get(), 0);
    result.push_back(deserialize<sar_mission_interfaces::TaskExecutionStatus>(
        sqlite3_column_blob(statement.get(), 1),
        sqlite3_column_bytes(statement.get(), 1)));
  }
  *has_more = extra_row;
  *next_sequence = last;
  return result;
}

void MissionStateStore::replaceCandidateSource(
    const std::string &source_key,
    const std::vector<sar_mission_interfaces::PerceptionCandidate> &candidates) {
  Statement remove(database_, "DELETE FROM candidates WHERE source_key=?;");
  bindText(remove.get(), 1, source_key);
  remove.stepDone();
  for (const auto &candidate : candidates) {
    if (candidate.lifecycle_state !=
        sar_mission_interfaces::PerceptionCandidate::EXPIRED) {
      saveCandidate(source_key, candidate);
    }
  }
}

void MissionStateStore::saveCandidate(
    const std::string &source_key,
    const sar_mission_interfaces::PerceptionCandidate &candidate) {
  Statement statement(database_,
      "INSERT INTO candidates VALUES(?,?,?,?,?) ON CONFLICT(source_key,observation_uuid) "
      "DO UPDATE SET event_sequence=excluded.event_sequence,"
      "lifecycle=excluded.lifecycle,payload=excluded.payload;");
  bindText(statement.get(), 1, source_key);
  bindText(statement.get(), 2, candidate.observation_uuid);
  sqlite3_bind_int64(statement.get(), 3, candidate.event_sequence);
  sqlite3_bind_int(statement.get(), 4, candidate.lifecycle_state);
  bindBlob(statement.get(), 5, serialize(candidate));
  statement.stepDone();
}

void MissionStateStore::eraseCandidate(const std::string &source_key,
                                       const std::string &observation_uuid) {
  Statement statement(database_,
      "DELETE FROM candidates WHERE source_key=? AND observation_uuid=?;");
  bindText(statement.get(), 1, source_key);
  bindText(statement.get(), 2, observation_uuid);
  statement.stepDone();
}

void MissionStateStore::eraseCandidateSource(const std::string &source_key) {
  Statement candidates(database_, "DELETE FROM candidates WHERE source_key=?;");
  bindText(candidates.get(), 1, source_key);
  candidates.stepDone();
  Statement sequence(database_,
      "DELETE FROM candidate_source_sequences WHERE source_key=?;");
  bindText(sequence.get(), 1, source_key);
  sequence.stepDone();
}

std::vector<PersistedCandidate> MissionStateStore::loadCandidates() const {
  Statement statement(database_,
      "SELECT source_key,payload FROM candidates ORDER BY source_key,observation_uuid;");
  std::vector<PersistedCandidate> result;
  while (sqlite3_step(statement.get()) == SQLITE_ROW) {
    PersistedCandidate record;
    record.source_key = columnText(statement.get(), 0);
    record.candidate = deserialize<sar_mission_interfaces::PerceptionCandidate>(
        sqlite3_column_blob(statement.get(), 1),
        sqlite3_column_bytes(statement.get(), 1));
    result.push_back(std::move(record));
  }
  return result;
}

void MissionStateStore::saveCandidateSourceSequence(
    const std::string &source_key, const std::uint64_t array_sequence) {
  Statement statement(database_,
      "INSERT INTO candidate_source_sequences VALUES(?,?) "
      "ON CONFLICT(source_key) DO UPDATE SET array_sequence=excluded.array_sequence;");
  bindText(statement.get(), 1, source_key);
  sqlite3_bind_int64(statement.get(), 2, array_sequence);
  statement.stepDone();
}

std::unordered_map<std::string, std::uint64_t>
MissionStateStore::loadCandidateSourceSequences() const {
  Statement statement(database_,
      "SELECT source_key,array_sequence FROM candidate_source_sequences;");
  std::unordered_map<std::string, std::uint64_t> result;
  while (sqlite3_step(statement.get()) == SQLITE_ROW) {
    result[columnText(statement.get(), 0)] =
        static_cast<std::uint64_t>(sqlite3_column_int64(statement.get(), 1));
  }
  return result;
}

void MissionStateStore::setMeta(const std::string &key,
                                const std::string &value) {
  Statement statement(database_,
      "INSERT INTO meta VALUES(?,?) ON CONFLICT(key) DO UPDATE SET value=excluded.value;");
  bindText(statement.get(), 1, key);
  bindText(statement.get(), 2, value);
  statement.stepDone();
}

std::string MissionStateStore::getMeta(const std::string &key,
                                       const std::string &default_value) const {
  Statement statement(database_, "SELECT value FROM meta WHERE key=?;");
  bindText(statement.get(), 1, key);
  return sqlite3_step(statement.get()) == SQLITE_ROW
             ? columnText(statement.get(), 0)
             : default_value;
}

std::string MissionStateStore::sha256Hex(
    const std::vector<std::uint8_t> &bytes) {
  unsigned char digest[SHA256_DIGEST_LENGTH];
  SHA256(bytes.data(), bytes.size(), digest);
  std::ostringstream stream;
  stream << std::hex << std::setfill('0');
  for (const unsigned char byte : digest) {
    stream << std::setw(2) << static_cast<unsigned int>(byte);
  }
  return stream.str();
}

}  // namespace sar_yolo_detector
