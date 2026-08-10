std_msgs/Header header
string follower_profile
bool command_valid
bool target_visible
bool target_predicted
bool uncertainty_limited
bool relative_state_active

# ROS body FLU: x forward, y left, z up; yaw is counter-clockwise positive.
geometry_msgs/Vector3 velocity_body
float64 yaw_rate
# False for fw_velocity_vector: velocity direction is the complete course input.
bool yaw_rate_enabled

string tracking_state
float32 tracking_quality
float32 uncertainty_scale
string association_method
string invalid_reason
