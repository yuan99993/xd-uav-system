# Optional gimbal health/lock contract.  It supplements, rather than changes,
# GimbalState so legacy angle-only drivers remain wire-compatible.
std_msgs/Header header
bool valid
bool healthy
bool tracking_active
bool range_valid
bool control_authority_available
float64 zoom_ratio
float64 horizontal_fov_rad
float64 vertical_fov_rad
float32 range_quality
