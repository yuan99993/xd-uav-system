# Downstream execution acknowledgement for a FollowerCommand.
# It distinguishes requested aircraft motion from the setpoint that the
# selected UAV backend actually accepted and published.

std_msgs/Header header
string backend
uint64 source_command_sequence
time source_capture_timestamp
bool accepted
bool executing
bool safety_gate_passed
bool failsafe_active
geometry_msgs/Vector3 applied_velocity_body
float32 applied_yaw_rate_deg_s
float32 command_age_ms
string reason
