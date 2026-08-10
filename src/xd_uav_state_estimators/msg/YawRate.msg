# yaw_rate是父坐标系下欧拉yaw角的一阶导数，不是机体系Z轴角速度。
# header.frame_id给出姿态父坐标系，child_frame_id给出姿态主体。
std_msgs/Header header
string child_frame_id
float64 yaw_rate
float64 variance
bool valid
