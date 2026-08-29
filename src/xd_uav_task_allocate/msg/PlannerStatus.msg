uint8 IDLE=0
uint8 PLANNING=1
uint8 ACTIVE=2
uint8 REACHED=3
uint8 BLOCKED=4
uint8 FAILED=5

std_msgs/Header header
uint32 goal_id
uint8 state
string detail
