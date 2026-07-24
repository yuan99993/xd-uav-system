uint8 WAITING=0
uint8 RUNNING=1
uint8 DEGRADED=2
uint8 DEAD_RECKONING=3
uint8 LOST=4

std_msgs/Header header
uint8 state
string state_name
bool state_valid
bool localization_valid
string active_source
float64 localization_age
float64 dead_reckoning_age
uint64 source_switches
string failure_reason
