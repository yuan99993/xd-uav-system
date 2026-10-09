# Persistent state for one independently filtered target.

std_msgs/Header header
time last_observation_stamp        # last accepted image observation; never advanced by prediction
time last_detection_stamp          # last real detector observation, excludes visual assistance
string measurement_source         # detector / visual_tracker / prediction
int32 track_id
int32 detector_track_id
bool detector_id_stable
int32 class_id
float32 confidence

int32[4] bbox
float32[4] normalized_bbox
float32[2] image_velocity
float32[4] state_covariance

uint32 age_frames
uint32 hit_count
uint32 frames_since_detection
string lifecycle_state             # tentative / confirmed / occluded / lost
bool detected
bool predicted
bool reidentification_match
bool selected
bool control_measurement_ready
float32 tracking_quality
string association_method          # detector_id / spatial / appearance / new / predicted

bool has_relative_position_body
string position_frame              # explicit FRD frame, independent of the image header
time position_stamp                # last accepted real metric observation, not prediction time
float32[3] relative_position_body
bool has_relative_velocity_body
float32[3] relative_velocity_body
float32[9] position_covariance
float32[9] velocity_covariance
bool range_valid
