# selection_type is clear, track_id, point, or roi.
# point and roi use normalized coordinates in the named source image.
string request_id
string session_id
uint64 expected_state_version
time issued_at
float32 ttl_sec
string selection_type
int32 track_id
string target_uid              # session:global_id; authoritative when supplied
string image_source
uint32 frame_seq
time capture_stamp
float32[2] point_norm
float32[4] roi_norm
string tracker_algorithm       # auto, csrt, kcf, mil, or template
