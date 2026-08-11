uint8 VEHICLE_MULTIROTOR=0
uint8 VEHICLE_FIXEDWING=1

std_msgs/Header header
string body_frame_id
uint8 vehicle_type

geometry_msgs/Point position_odom
geometry_msgs/Vector3 velocity_odom
geometry_msgs/Vector3 acceleration_odom
geometry_msgs/Quaternion orientation_odom_body
geometry_msgs/Vector3 body_rate
geometry_msgs/Vector3 angular_acceleration_odom

float64 airspeed
float64 groundspeed
float64 course
float64 climb_rate

bool state_valid
bool localization_valid
bool odometry_fresh
bool imu_fresh
bool acceleration_fresh
bool airspeed_valid
bool stable
string active_source
float64 odometry_age
float64 imu_age
float64 acceleration_age
float64 airspeed_age
