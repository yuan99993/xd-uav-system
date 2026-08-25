uint8 ACCEPTED=0
uint8 ACTIVE=1
uint8 REACQUIRING=2
uint8 COMPLETED=3
uint8 REJECTED=4
uint8 FAILED=5

std_msgs/Header header
uint32 path_id
uint8 state
uint32 current_segment
float32 progress
float32 cross_track_error
float32 distance_to_end
string detail
