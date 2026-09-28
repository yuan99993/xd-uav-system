std_msgs/Header header

uint8 POSITION=0
uint8 VELOCITY=1
uint8 HOME=2

uint8 mode

# Body-relative yaw/pitch angles for POSITION, in radians.
float64 yaw
float64 pitch

# Axis velocities for VELOCITY, or optional per-command speed limits for
# POSITION. A non-positive POSITION value uses the configured maximum.
float64 yaw_rate
float64 pitch_rate
