uint8 CANDIDATE=0
uint8 CONFIRMED=1
uint8 ASSIGNED=2
uint8 EXECUTING=3
uint8 COMPLETED=4
uint8 STALE=5
uint8 VERIFYING=6

std_msgs/Header header
uint32 target_id
int32 class_id
geometry_msgs/Point position
float64[9] position_covariance
float32 confidence
uint32 observation_count
string[] observer_uavs
uint8 status
