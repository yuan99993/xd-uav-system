# header.frame_id表示航向参考坐标系，child_frame_id表示被估计的机体坐标系。
std_msgs/Header header
string child_frame_id
float64 heading
float64 variance
bool valid
