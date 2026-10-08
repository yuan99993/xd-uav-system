uint8 STARTING=0
uint8 READY=1
uint8 STOPPING=2
uint8 ERROR=3

string uav_name
uint32 instance
uint8 lifecycle
bool connected
bool armed
bool on_ground
string mode
geometry_msgs/Point world_origin
geometry_msgs/Point gazebo_spawn
string detail
