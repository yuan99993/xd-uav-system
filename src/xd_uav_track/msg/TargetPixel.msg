# Native pixel coordinates in the source camera image (top-left origin).
int32 track_id
int32 class_id
float32 confidence
float32 x
float32 y
int32[4] bbox               # [x_min, y_min, x_max, y_max], max exclusive
string lifecycle_state
bool detected
bool predicted
bool selected
