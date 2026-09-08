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

uint8 BACKEND_NONE=0
uint8 BACKEND_MULTIROTOR=1
uint8 BACKEND_FIXED_WING=2

uint8 REFERENCE_UNKNOWN=0
uint8 REFERENCE_POSITION_TARGET=1
uint8 REFERENCE_TRAJECTORY=2
uint8 REFERENCE_PATH=3
uint8 REFERENCE_SIMPLE_GOAL=4
uint8 REFERENCE_INTERNAL=5
uint8 REFERENCE_IDLE=6

uint8 ACTION_IDLE=0
uint8 ACTION_ACTIVE=1
uint8 ACTION_SUCCEEDED=2
uint8 ACTION_FAILED=3

std_msgs/Header header
uint8 vehicle_type
uint8 airframe_type
uint8 flight_regime
uint32 regime_generation
uint8 active_backend
uint8 reference_type
string reference_source
uint32 action_generation
uint8 action_status
string action_detail
geometry_msgs/Vector3 body_rate
float64 thrust
bool valid
bool takeoff_active
bool landing_active
bool landing_touchdown
string controller
string rejection_reason
