std_msgs/Header header
sar_yolo_detector/MissionIdentity aircraft
string executor_id
string executor_session_uuid
uint64 heartbeat_sequence
bool ready
bool flight_control_ok
bool localization_ok
float32 battery_percent
string fault_reason
sar_yolo_detector/ExecutorCapabilities capabilities

# Authoritative executor view used by the bridge after either process restarts.
sar_yolo_detector/TaskExecutionStatus[] active_tasks
