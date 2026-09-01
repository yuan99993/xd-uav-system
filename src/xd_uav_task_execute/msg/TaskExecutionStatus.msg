uint8 IDLE=0
uint8 STARTING=1
uint8 ACQUIRING=2
uint8 RUNNING=3
uint8 STOPPING=4
uint8 SUCCEEDED=5
uint8 FAILED=6
uint8 PREEMPTED=7

std_msgs/Header header
uint32 task_id
uint32 target_id
uint8 task_type
uint8 phase
string worker_name
float32 progress
string detail
