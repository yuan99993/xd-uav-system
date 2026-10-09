string target_uid
int32 track_id
int32 class_id
string class_name
bool class_valid
float32 detection_confidence
float32 tracking_quality
int32[4] bbox_xyxy              # native pixels, [xmin,ymin,xmax,ymax)
float32[2] center_px            # top-left origin; x right, y down
float32[4] bbox_cxcywh_norm
bool selected
string lifecycle_state
string measurement_source       # detector / visual_tracker / prediction / stale (identity only)
time last_detection_stamp        # last accepted image observation; retained during loss
float32 age_ms                   # capture/publish age incl. delivery; -1 means unknown
bool position_valid
string position_frame
time position_stamp
float32[3] position_xyz
float32[9] position_covariance
