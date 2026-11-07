# Backend-neutral YOLO result shared by vehicle-local SmartTracker nodes.
int64 track_id
int32 class_id
float32 confidence
float32[4] aabb_xyxy
int32[2] center_xy
string geometry_type
float32[5] obb_xywhr
float32[] polygon_xy
float32 rotation_deg
bool has_rotation
bool track_id_is_stable
