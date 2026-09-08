uint8 VEHICLE_MULTIROTOR=0
uint8 VEHICLE_FIXEDWING=1

uint8 AIRFRAME_MULTIROTOR=0
uint8 AIRFRAME_FIXED_WING=1
uint8 AIRFRAME_VTOL=2
uint8 AIRFRAME_TILTROTOR=3
uint8 AIRFRAME_UNKNOWN=255

uint8 REGIME_HOVER=0
uint8 REGIME_TRANSITION_TO_FORWARD=1
uint8 REGIME_FORWARD_FLIGHT=2
uint8 REGIME_TRANSITION_TO_HOVER=3
uint8 REGIME_UNKNOWN=255

uint8 REQUESTED_NONE=0
uint8 REQUESTED_HOVER=1
uint8 REQUESTED_FORWARD_FLIGHT=2

uint8 ACTION_NONE=0
uint8 ACTION_ARM=1
uint8 ACTION_DISARM=2
uint8 ACTION_TAKEOFF=3
uint8 ACTION_HOLD=4
uint8 ACTION_NAVIGATE=5
uint8 ACTION_LAND=6
uint8 ACTION_RETURN_HOME=7
uint8 ACTION_EMERGENCY=8

std_msgs/Header header
string body_frame_id
uint8 vehicle_type
uint8 airframe_type
uint8 flight_regime
uint8 requested_regime
uint8 vehicle_action
uint32 regime_generation
uint32 action_generation
bool transition_pending

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
