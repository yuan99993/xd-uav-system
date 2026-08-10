# One selected target bounding box.
# Pixel coordinates use the image origin at the upper-left corner.

std_msgs/Header header
uint32 image_width
uint32 image_height
float32 x_min
float32 y_min
float32 x_max
float32 y_max
float32 confidence
int32 track_id
bool valid
