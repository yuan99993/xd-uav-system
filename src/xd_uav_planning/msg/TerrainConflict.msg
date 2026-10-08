uint8 SPATIAL=1
uint8 SPATIOTEMPORAL=2

uint8 conflict_type
string first_uav
uint32 first_goal_id
string second_uav
uint32 second_goal_id
geometry_msgs/Point first_position
geometry_msgs/Point second_position
float64 separation_m
float64 threshold_m
bool predicted_time_valid
float64 predicted_time_s
