# header.frame_id表示速度的表达坐标系，child_frame_id表示速度对应的机体坐标系。
std_msgs/Header header
string child_frame_id
float64 x
float64 y
# 按[x-x, x-y, y-x, y-y]排列。
float64[4] covariance
bool valid
