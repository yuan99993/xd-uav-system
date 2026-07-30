uint8 VEHICLE_MULTIROTOR=0
uint8 VEHICLE_FIXEDWING=1

std_msgs/Header header
uint8 vehicle_type
geometry_msgs/Vector3 body_rate
float64 thrust
bool valid
bool takeoff_active
bool landing_active
bool landing_touchdown
string controller
string rejection_reason
