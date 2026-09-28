std_msgs/Header header
int32 target_id
bool valid

# Position/velocity are expressed in the shared world (odometry) frame named
# by header.frame_id. Orientation is reserved for future target tracking.
geometry_msgs/PoseWithCovariance pose
geometry_msgs/TwistWithCovariance velocity
bool has_velocity
string source
