uint8 PENDING=0
uint8 ASSIGNED=1
uint8 EXECUTING=2
uint8 COMPLETED=3
uint8 FAILED=4

std_msgs/Header header
uint32 task_id
uint32 target_id
int32 class_id
geometry_msgs/Point goal
uint8 priority
string[] allowed_vehicle_types
string assigned_worker
uint8 status
string detail
