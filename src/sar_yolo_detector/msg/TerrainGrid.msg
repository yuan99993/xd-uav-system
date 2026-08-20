# Lightweight DEM/DSM grid in header.frame_id. Row-major elevations and
# variances have width*height entries; non-finite elevations are unknown.
std_msgs/Header header
float64 origin_x
float64 origin_y
float32 resolution
uint32 width
uint32 height
float32[] elevation
float32[] variance
string vertical_datum
string map_version
